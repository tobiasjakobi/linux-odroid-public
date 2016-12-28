/*
 * Copyright (C) 2010 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/version.h>
#include "mali_kernel_common.h"
#include "mali_kernel_linux.h"
#include "mali_osk.h"
#include "mali_osk_mali.h"
#include <linux/mali/mali_utgard.h>

#include <linux/clk.h>
#include <linux/cma.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>


#if defined(CONFIG_MALI_PM_DVFS)
	#error Platform does not implement power management via DVFS.
#endif

#define MSG_PREFIX "Mali platform: "

struct exynos4412_power_state {
	/* Frequency of the power state in MHz. */
	unsigned int freq;

	/*
	 * Threshold for switching to a different power state.
	 * Load is defined in the range [0, 1000].
	 */
	unsigned int down_threshold;
	unsigned int up_threshold;

	/* Enable turbo on leftbus DevFreq. */
	bool leftbus_turbo;
};

#define POWER_STATE(f, d, u, t) \
	{.freq = f, .down_threshold = d, .up_threshold = u, .leftbus_turbo = t}

static const struct exynos4412_power_state exynos4412_power_state_default[] = {
	POWER_STATE(160,   0,  700, false),
	POWER_STATE(266, 620,  900, false),
	POWER_STATE(350, 850,  900, false),
	POWER_STATE(440, 850, 1000, true),
};

static const struct exynos4412_power_state exynos4412_power_state_prime[] = {
	POWER_STATE(160,   0,  700, false),
	POWER_STATE(266, 620,  900, false),
	POWER_STATE(350, 850,  900, false),
	POWER_STATE(440, 850,  900, true),
	POWER_STATE(533, 950, 1000, true),
};

#undef POWER_STATE

enum exynos4412_flag_bits {
	exynos4412_runpm_suspended,
	exynos4412_power_cycle,
	exynos4412_leftbus_turbo,
};

enum opp_update_mode {
	opp_update_default,
	opp_update_init,
	opp_update_low,
	opp_update_perf,
};

struct exynos4412_drvdata {
	/*
	 * This is needed for the perf workqueue to fetch OPP entries.
	 */
	struct device *dev;

	unsigned long flags;

	struct clk *sclk;
	struct clk *clk;
	struct regulator *regulator;
	struct devfreq *leftbus_devfreq;

	unsigned long regulator_offset;
	unsigned long regulator_delay;

	unsigned int load;
	unsigned long cur_volt;

	const struct exynos4412_power_state *states;
	unsigned int num_states;
	unsigned int cur_state;

	struct workqueue_struct *power_workqueue;
	struct work_struct power_work;
};

static int exynos4412_opp_update(struct exynos4412_drvdata *data,
				 enum opp_update_mode mode,
				 int index)
{
	struct device *dev;
	struct dev_pm_opp *opp;

	int ret;
	unsigned long volt, freq;
	bool volt_first = true;
	bool leftbus_turbo;

	dev = data->dev;

	/*
	 * dev_pm_opp_find_freq_exact() should never return NULL here.
	 * This is checked in exynos4412_opp_check().
	 */
	switch (mode) {
	case opp_update_default:
	default:
		freq = data->states[index].freq;
		leftbus_turbo = data->states[index].leftbus_turbo;
		break;

	case opp_update_init:
		freq = data->states[0].freq;
		leftbus_turbo = data->states[0].leftbus_turbo;
		dev_info(dev, MSG_PREFIX "initial G3D core clock rate = %lu MHz\n", freq);
		break;

	case opp_update_low:
		freq = data->states[0].freq;
		leftbus_turbo = data->states[0].leftbus_turbo;
		break;

	case opp_update_perf:
		freq = data->states[data->num_states - 1].freq;
		leftbus_turbo = data->states[data->num_states - 1].leftbus_turbo;
		break;
	}

	freq *= 1000000;

	rcu_read_lock();

	opp = dev_pm_opp_find_freq_exact(dev, freq, true);
	BUG_ON(opp == NULL);

	volt = dev_pm_opp_get_voltage(opp);
	volt += data->regulator_offset;

	rcu_read_unlock();

	if (data->cur_volt && volt < data->cur_volt)
		volt_first = false;

	if (leftbus_turbo && !test_bit(exynos4412_leftbus_turbo, &data->flags)) {
		ret = devfreq_turbo_get(data->leftbus_devfreq);

		if (ret < 0)
			goto out;

		set_bit(exynos4412_leftbus_turbo, &data->flags);
	}

	if (volt_first) {
		ret = regulator_set_voltage(data->regulator, volt, volt);
		udelay(data->regulator_delay);

		if (ret < 0)
			goto out;
	}

	ret = clk_set_rate(data->sclk, freq);
	if (ret < 0)
		goto out;

	if (!volt_first) {
		ret = regulator_set_voltage(data->regulator, volt, volt);

		if (ret < 0)
			goto out;
	}

	if (!leftbus_turbo && test_bit(exynos4412_leftbus_turbo, &data->flags)) {
		ret = devfreq_turbo_put(data->leftbus_devfreq);

		if (ret < 0)
			goto out;

		clear_bit(exynos4412_leftbus_turbo, &data->flags);
	}

	data->cur_volt = volt;

out:
	return ret;
}

static int exynos4412_opp_check(struct device *dev, struct exynos4412_drvdata *data)
{
	int ret;
	unsigned int i;
	unsigned long freq;
	const struct exynos4412_power_state *s;
	struct dev_pm_opp *opp;

	rcu_read_lock();

	for (i = 0; i < data->num_states; i++) {
		s = &data->states[i];

		dev_dbg(dev, MSG_PREFIX "OPP check (freq = %u, down = %u, up = %u)\n",
			s->freq, s->down_threshold, s->up_threshold);

		freq = s->freq;
		freq *= 1000000;

		opp = dev_pm_opp_find_freq_exact(dev, freq, true);
		if (!opp) {
			dev_err(dev, MSG_PREFIX "failed to find OPP for freq %lu\n",
				freq);

			ret = -EFAULT;
			goto out;
		}
	}

	ret = 0;

out:
	rcu_read_unlock();
	return ret;
}

static int exynos4412_opp_init(struct device *dev, struct exynos4412_drvdata *data)
{
	int ret;

	ret = dev_pm_opp_of_add_table(dev);
	if (ret < 0)
		return ret;

	ret = exynos4412_opp_update(data, opp_update_init, -1);
	if (ret < 0)
		goto fail_update;

	data->leftbus_devfreq = devfreq_get_devfreq_by_phandle(dev, 0);
	if (IS_ERR(data->leftbus_devfreq)) {
		ret = PTR_ERR(data->leftbus_devfreq);
		goto fail_update;
	}

	return 0;

fail_update:
	dev_pm_opp_of_remove_table(dev);

	return ret;
}

static void exynos4412_opp_deinit(struct device *dev, struct exynos4412_drvdata *data)
{
	exynos4412_opp_update(data, opp_update_low, -1);
	dev_pm_opp_of_remove_table(dev);
}

static int exynos4412_power_init(struct device *dev, struct exynos4412_drvdata *data)
{
	if (of_machine_is_compatible("samsung,exynos4412-prime")) {
		data->states = exynos4412_power_state_prime;
		data->num_states = ARRAY_SIZE(exynos4412_power_state_prime);
	} else {
		data->states = exynos4412_power_state_default;
		data->num_states = ARRAY_SIZE(exynos4412_power_state_default);
	}

	return 0;
}

static int exynos4412_clock_init(struct device *dev, struct exynos4412_drvdata *data)
{
	struct device_node *np;
	struct clk *sclk;
	struct clk *clk;

	unsigned long rate;

	np = dev->of_node;

	sclk = of_clk_get_by_name(np, "sclk_g3d");
	if (IS_ERR(sclk)) {
		dev_err(dev, MSG_PREFIX "failed to get G3D core clock\n");
		goto fail_sclk;
	}

	clk = of_clk_get_by_name(np, "g3d");
	if (IS_ERR(clk)) {
		dev_err(dev, MSG_PREFIX "failed to get G3D clock\n");
		goto fail_clk;
	}

	if (clk_prepare(sclk) < 0) {
		dev_err(dev, MSG_PREFIX "failed to prepare G3D core clock\n");
		goto fail_prepare_sclk;
	}

	if (clk_prepare(clk) < 0) {
		dev_err(dev, MSG_PREFIX "failed to prepare G3D clock\n");
		goto fail_prepare_clk;
	}

	rate = clk_get_rate(clk);

	dev_info(dev, MSG_PREFIX "G3D clock rate = %lu MHz\n", rate / 1000000);

	data->sclk = sclk;
	data->clk = clk;

	return 0;

fail_prepare_clk:
	clk_unprepare(sclk);

fail_prepare_sclk:
	clk_put(clk);

fail_clk:
	clk_put(sclk);

fail_sclk:
	return -EFAULT;
}

static void exynos4412_clock_deinit(struct device *dev, struct exynos4412_drvdata *data)
{
	clk_unprepare(data->clk);
	clk_put(data->clk);

	clk_unprepare(data->sclk);
	clk_put(data->sclk);
}

#ifdef CONFIG_REGULATOR
static int exynos4412_regulator_init(struct device *dev, struct exynos4412_drvdata *data)
{
	struct regulator *regulator;
	unsigned int i;
	int ret;

	regulator = regulator_get(dev, "gpu");

	if (IS_ERR(regulator)) {
		dev_err(dev, MSG_PREFIX "failed to get regulator\n");
		return -EFAULT;
	}

	if (test_bit(exynos4412_power_cycle, &data->flags)) {
		for (i = 0; i < 2; i++) {
			ret = regulator_enable(regulator);
			if (ret < 0)
				break;

			usleep_range(4000, 10000);

			ret = regulator_disable(regulator);
			if (ret < 0)
				break;

			usleep_range(2000, 4000);
		}

		/*
		 * If power-cycling fails, try to get regulator into sane
		 * state again, and immediately exit afterwards.
		 */
		if (i != 2) {
			dev_err(dev, MSG_PREFIX "failed to power-cycle regulator\n");
			regulator_force_disable(regulator);
			regulator_put(regulator);
			return -EFAULT;
		}
	}

	ret = regulator_enable(regulator);
	if (ret < 0)
		goto out;

	/* Allow the regulator some more time to settle. */
	usleep_range(4000, 10000);

	data->regulator = regulator;
	ret = 0;

out:
	return ret;
}

static void exynos4412_regulator_deinit(struct device *dev, struct exynos4412_drvdata *data)
{
	regulator_disable(data->regulator);
	regulator_put(data->regulator);
}
#else
static int exynos4412_regulator_init(struct device *dev, struct exynos4412_drvdata *data) { return 0; }
static void exynos4412_regulator_deinit(struct device *dev, struct exynos4412_drvdata *data) {}
#endif

static void exynos4412_power_work(struct work_struct *work)
{
	struct exynos4412_drvdata *data;
	int index, ret;
	const struct exynos4412_power_state *state;

	data = container_of(work, struct exynos4412_drvdata, power_work);

	index = data->cur_state;
	state = &data->states[index];

	if (data->load > state->up_threshold)
		++index;
	else if (data->load < state->down_threshold)
		--index;

	if (index < 0)
		index = 0;
	else if (index >= data->num_states)
		index = data->num_states - 1;

	if (index == data->cur_state)
		return;

	ret = exynos4412_opp_update(data, opp_update_default, index);

	if (ret < 0)
		dev_err(data->dev, MSG_PREFIX "failed to update OPP\n");
	else
		data->cur_state = index;
}

static void exynos4412_utilization_callback(void *ctx, const struct mali_gpu_utilization_data *udata)
{
	struct exynos4412_drvdata *ddata;

	ddata = ctx;

	/*
	 * Don't try to change power mode when runpm suspended.
	 */
	if (test_bit(exynos4412_runpm_suspended, &ddata->flags))
		return;

	if (udata->utilization_gpu >= 256)
		ddata->load = 1000;
	else
		ddata->load = (udata->utilization_gpu * 1000) / 256;

	queue_work(ddata->power_workqueue, &ddata->power_work);
}

int mali_platform_runtime_suspend(struct device *dev)
{
	int ret;
	struct exynos4412_drvdata *data = dev_get_drvdata(dev);

	set_bit(exynos4412_runpm_suspended, &data->flags);

	/*
	 * Make sure that any OPP update is finished, and transition
	 * into low power state.
	 *
	 * This way we avoid restoring a high G3D core clock on resume.
	 */
	flush_work(&data->power_work);
	ret = exynos4412_opp_update(data, opp_update_low, -1);

	if (ret < 0)
		return ret;

	data->cur_state = 0;

	clk_disable(data->clk);
	clk_disable(data->sclk);

	return 0;
}

int mali_platform_runtime_resume(struct device *dev)
{
	int ret;
	struct exynos4412_drvdata *data = dev_get_drvdata(dev);

	ret = clk_enable(data->sclk);
	if (ret < 0)
		goto out;

	ret = clk_enable(data->clk);

#if defined(CONFIG_MALI_PM_NONE)
	if (ret < 0)
		goto out;

	/*
	 * If no power management mode is selected, we just transition
	 * to the highest power state and don't change it anymore.
	 */
	ret = exynos4412_opp_update(data, opp_update_perf, -1);
#endif

out:
	if (ret == 0)
		clear_bit(exynos4412_runpm_suspended, &data->flags);

	return ret;
}

int mali_platform_device_enable(struct device *dev)
{
	dev_dbg(dev, MSG_PREFIX "enable()\n");

#if defined(CONFIG_PM)
	pm_runtime_set_autosuspend_delay(dev, 2500);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
#endif

	return 0;
}

int mali_platform_device_disable(struct device *dev)
{
	dev_dbg(dev, MSG_PREFIX "disable()\n");

#if defined(CONFIG_PM)
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
#endif

	return 0;
}

int mali_platform_device_init(struct platform_device *pdev)
{
	int ret;
	u32 pval;
	struct device *dev;
	struct exynos4412_drvdata *data;

	struct mali_gpu_device_data mali_gpu_data = { 0 };
	struct cma *default_area = dma_contiguous_default_area;

	dev = &pdev->dev;

	dev_dbg(dev, MSG_PREFIX "init()\n");

	if (!dev->of_node)
		return -ENODEV;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (of_property_read_bool(dev->of_node, "regulator-power-cycle"))
		set_bit(exynos4412_power_cycle, &data->flags);

	if (!of_property_read_u32(dev->of_node, "regulator-microvolt-offset", &pval))
		data->regulator_offset = pval;

	if (!of_property_read_u32(dev->of_node, "regulator-microsecs-delay", &pval))
		data->regulator_delay = pval;

	data->dev = dev;
	set_bit(exynos4412_runpm_suspended, &data->flags);

	ret = exynos4412_power_init(dev, data);
	if (ret < 0)
		return -EFAULT;

	ret = exynos4412_regulator_init(dev, data);
	if (ret < 0)
		goto fail_regulator;

	ret = exynos4412_clock_init(dev, data);
	if (ret < 0)
		goto fail_clock;

	ret = exynos4412_opp_init(dev, data);
	if (ret < 0)
		goto fail_opp_init;

	ret = exynos4412_opp_check(dev, data);
	if (ret < 0)
		goto fail_opp_check;

#if defined(CONFIG_MALI_PM_UTILIZATION)
	data->power_workqueue = create_singlethread_workqueue("mali_power");
	if (!data->power_workqueue)
		goto fail_opp_check;

	INIT_WORK(&data->power_work, exynos4412_power_work);
#endif

	if (default_area) {
		mali_gpu_data.fb_start = cma_get_base(default_area);
		mali_gpu_data.fb_size = cma_get_size(default_area);
	}

#if defined(CONFIG_MALI_PM_UTILIZATION)
	mali_gpu_data.control_interval = 100; /* 100 ms */
	mali_gpu_data.utilization_callback = exynos4412_utilization_callback;
	mali_gpu_data.utilization_context = data;
#endif

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret < 0)
		goto fail_platform_data;

	ret = platform_device_add_data(pdev, &mali_gpu_data, sizeof(mali_gpu_data));
	if (ret < 0)
		goto fail_platform_data;

	dev_set_drvdata(dev, data);

	return 0;

fail_platform_data:
	destroy_workqueue(data->power_workqueue);

fail_opp_check:
	exynos4412_opp_deinit(dev, data);

fail_opp_init:
	exynos4412_clock_deinit(dev, data);

fail_clock:
	exynos4412_regulator_deinit(dev, data);

fail_regulator:
	kfree(data);
	return ret;
}

int mali_platform_device_deinit(struct platform_device *pdev)
{
	struct device *dev;
	struct exynos4412_drvdata *data;

	dev = &pdev->dev;
	data = dev_get_drvdata(dev);

	dev_dbg(dev, MSG_PREFIX "deinit()\n");

	set_bit(exynos4412_runpm_suspended, &data->flags);
	flush_work(&data->power_work);

#if defined(CONFIG_MALI_PM_UTILIZATION)
	destroy_workqueue(data->power_workqueue);
#endif

	exynos4412_opp_deinit(dev, data);
	exynos4412_clock_deinit(dev, data);
	exynos4412_regulator_deinit(dev, data);

	kfree(data);

	return 0;
}
