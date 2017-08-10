// SPDX-License-Identifier: GPL-2.0-only
/*
 * exynos_ppmu.c - EXYNOS PPMU (Platform Performance Monitoring Unit) support
 *
 * Copyright (c) 2014-2015 Samsung Electronics Co., Ltd.
 * Author : Chanwoo Choi <cw00.choi@samsung.com>
 *
 * This driver is based on drivers/devfreq/exynos/exynos_ppmu.c
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/devfreq-event.h>

#include "exynos-ppmu.h"

#define DISABLE_EXYNOS_PPMU_CYCLE_DEBUG

enum exynos_ppmu_type {
	EXYNOS_TYPE_PPMU,
	EXYNOS_TYPE_PPMU_V2,
};

struct exynos_ppmu_data {
	void __iomem *base;
	struct clk *clk;
};

struct exynos_ppmu {
	struct devfreq_event_dev **edev;
	struct devfreq_event_desc *desc;
	unsigned int num_events;

	struct device *dev;

	struct exynos_ppmu_data ppmu;
	enum exynos_ppmu_type ppmu_type;
};

#define PPMU_EVENT(name)			\
	{ "ppmu-event0-"#name, PPMU_PMNCNT0 },	\
	{ "ppmu-event1-"#name, PPMU_PMNCNT1 },	\
	{ "ppmu-event2-"#name, PPMU_PMNCNT2 },	\
	{ "ppmu-event3-"#name, PPMU_PMNCNT3 }

static struct __exynos_ppmu_events {
	char *name;
	int id;
} ppmu_events[] = {
	/* For Exynos3250, Exynos4 and Exynos5260 */
	PPMU_EVENT(g3d),
	PPMU_EVENT(fsys),

	/* For Exynos4 SoCs and Exynos3250 */
	PPMU_EVENT(dmc0),
	PPMU_EVENT(dmc1),
	PPMU_EVENT(cpu),
	PPMU_EVENT(rightbus),
	PPMU_EVENT(leftbus),
	PPMU_EVENT(lcd0),
	PPMU_EVENT(camif),

	/* Only for Exynos3250 and Exynos5260 */
	PPMU_EVENT(mfc),

	/* Only for Exynos4 SoCs */
	PPMU_EVENT(mfc-left),
	PPMU_EVENT(mfc-right),

	/* Only for Exynos5260 SoCs */
	PPMU_EVENT(drex0-s0),
	PPMU_EVENT(drex0-s1),
	PPMU_EVENT(drex1-s0),
	PPMU_EVENT(drex1-s1),
	PPMU_EVENT(eagle),
	PPMU_EVENT(kfc),
	PPMU_EVENT(isp),
	PPMU_EVENT(fimc),
	PPMU_EVENT(gscl),
	PPMU_EVENT(mscl),
	PPMU_EVENT(fimd0x),
	PPMU_EVENT(fimd1x),

	/* Only for Exynos5433 SoCs */
	PPMU_EVENT(d0-cpu),
	PPMU_EVENT(d0-general),
	PPMU_EVENT(d0-rt),
	PPMU_EVENT(d1-cpu),
	PPMU_EVENT(d1-general),
	PPMU_EVENT(d1-rt),

	/* For Exynos5422 SoC */
	PPMU_EVENT(dmc0_0),
	PPMU_EVENT(dmc0_1),
	PPMU_EVENT(dmc1_0),
	PPMU_EVENT(dmc1_1),
};

#if !defined(DISABLE_EXYNOS_PPMU_CYCLE_DEBUG)
static unsigned int _cc_init = 0;
static unsigned int _cc_overhead = 0;

static inline unsigned int _get_cyclecount(void)
{
	unsigned int value;
	/* read CCNT register */
	asm volatile ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(value));
	return value;
}

static inline void _init_perfcounters(void *info)
{
	unsigned int *args = info;

	const bool do_reset = (args[0] != 0);
	const bool enable_divider = (args[1] != 0);

	/* enable all counters (including cycle counter) */
	unsigned int value = BIT(0);

	if (do_reset) {
		value |= BIT(1); /* reset all counters to zero */
		value |= BIT(2); /* reset cycle counter to zero */
	}

	if (enable_divider)
		value |= BIT(3); /* enable "by 64" divider for CCNT */

	value |= BIT(4);

	/* program the performance-counter control-register */
	asm volatile ("MCR p15, 0, %0, c9, c12, 0\t\n" :: "r"(value));

	/* enable all counters */
	asm volatile ("MCR p15, 0, %0, c9, c12, 1\t\n" :: "r"(0x8000000f));

	/* clear overflows */
	asm volatile ("MCR p15, 0, %0, c9, c12, 3\t\n" :: "r"(0x8000000f));
}

static inline void _init_perfcounters_smp(void)
{
	unsigned int args[2] = {1, 0};

	if (_cc_init)
		return;

	/* enable perf counters on all CPUs */
	on_each_cpu(_init_perfcounters, (void*)args, 1);

	/* measure overhead of retrieving cycle count */
	_cc_overhead = _get_cyclecount();
	_cc_overhead = _get_cyclecount() - _cc_overhead;

	printk("DEBUG: _init_perfcounters_smp: overhead = %u\n", _cc_overhead);

	_cc_init = 1;
}
#endif

static int __exynos_ppmu_find_ppmu_id(const char *edev_name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ppmu_events); i++)
		if (!strcmp(edev_name, ppmu_events[i].name))
			return ppmu_events[i].id;

	return -EINVAL;
}

static int exynos_ppmu_find_ppmu_id(struct devfreq_event_dev *edev)
{
	return __exynos_ppmu_find_ppmu_id(edev->desc->name);
}

/*
 * The devfreq-event ops structure for PPMU v1.1
 */
static int exynos_ppmu_disable(struct devfreq_event_dev *edev)
{
	struct exynos_ppmu *info = devfreq_event_get_drvdata(edev);
	u32 pmnc;

	/* Disable all counters */
	__raw_writel(PPMU_CCNT_MASK |
		     PPMU_PMCNT0_MASK |
		     PPMU_PMCNT1_MASK |
		     PPMU_PMCNT2_MASK |
		     PPMU_PMCNT3_MASK,
		     info->ppmu.base + PPMU_CNTENC);

	/* Disable PPMU */
	pmnc = __raw_readl(info->ppmu.base + PPMU_PMNC);
	pmnc &= ~PPMU_PMNC_ENABLE_MASK;
	__raw_writel(pmnc, info->ppmu.base + PPMU_PMNC);

	return 0;
}

static int exynos_ppmu_set_event(struct devfreq_event_dev *edev)
{
	struct exynos_ppmu *info = devfreq_event_get_drvdata(edev);
	int id = exynos_ppmu_find_ppmu_id(edev);
	u32 pmnc, cntens;
	unsigned int _cc;

	if (id < 0)
		return id;

#if !defined(DISABLE_EXYNOS_PPMU_CYCLE_DEBUG)
	_init_perfcounters_smp();
	_cc = _get_cyclecount();
#endif

	/* Enable specific counter */
	cntens = __raw_readl(info->ppmu.base + PPMU_CNTENS);
	cntens |= (PPMU_CCNT_MASK | (PPMU_ENABLE << id));
	__raw_writel(cntens, info->ppmu.base + PPMU_CNTENS);

	/* Set the event of proper data type monitoring */
	__raw_writel(edev->desc->event_type, info->ppmu.base + PPMU_BEVTxSEL(id));

	/* Reset cycle counter/performance counter and enable PPMU */
	pmnc = __raw_readl(info->ppmu.base + PPMU_PMNC);
	pmnc &= ~(PPMU_PMNC_ENABLE_MASK
			| PPMU_PMNC_COUNTER_RESET_MASK
			| PPMU_PMNC_CC_RESET_MASK);
	pmnc |= (PPMU_ENABLE << PPMU_PMNC_ENABLE_SHIFT);
	pmnc |= (PPMU_ENABLE << PPMU_PMNC_COUNTER_RESET_SHIFT);
	pmnc |= (PPMU_ENABLE << PPMU_PMNC_CC_RESET_SHIFT);
	__raw_writel(pmnc, info->ppmu.base + PPMU_PMNC);

#if !defined(DISABLE_EXYNOS_PPMU_CYCLE_DEBUG)
	_cc = _get_cyclecount() - (_cc + _cc_overhead);
	printk("set:%u\n", _cc);
#endif

	return 0;
}

static int exynos_ppmu_get_event(struct devfreq_event_dev *edev,
				struct devfreq_event_data *edata)
{
	struct exynos_ppmu *info = devfreq_event_get_drvdata(edev);
	int id = exynos_ppmu_find_ppmu_id(edev);
	u32 pmnc, cntenc;
	unsigned int _cc;

	if (id < 0)
		return -EINVAL;

#if !defined(DISABLE_EXYNOS_PPMU_CYCLE_DEBUG)
	_init_perfcounters_smp();
	_cc = _get_cyclecount();
#endif

	/* Disable PPMU */
	pmnc = __raw_readl(info->ppmu.base + PPMU_PMNC);
	pmnc &= ~PPMU_PMNC_ENABLE_MASK;
	__raw_writel(pmnc, info->ppmu.base + PPMU_PMNC);

	/* Read cycle count */
	edata->total_count = __raw_readl(info->ppmu.base + PPMU_CCNT);

	/* Read performance count */
	switch (id) {
	case PPMU_PMNCNT0:
	case PPMU_PMNCNT1:
	case PPMU_PMNCNT2:
		edata->load_count
			= __raw_readl(info->ppmu.base + PPMU_PMNCT(id));
		break;
	case PPMU_PMNCNT3:
		edata->load_count =
			((__raw_readl(info->ppmu.base + PPMU_PMCNT3_HIGH) << 8)
			| __raw_readl(info->ppmu.base + PPMU_PMCNT3_LOW));
		break;
	default:
		return -EINVAL;
	}

	/* Disable specific counter */
	cntenc = __raw_readl(info->ppmu.base + PPMU_CNTENC);
	cntenc |= (PPMU_CCNT_MASK | (PPMU_ENABLE << id));
	__raw_writel(cntenc, info->ppmu.base + PPMU_CNTENC);

#if !defined(DISABLE_EXYNOS_PPMU_CYCLE_DEBUG)
	_cc = _get_cyclecount() - (_cc + _cc_overhead);
	printk("get:%u\n", _cc);
#endif

	dev_dbg(&edev->dev, "%s (event: %ld/%ld)\n", edev->desc->name,
					edata->load_count, edata->total_count);

	return 0;
}

static const struct devfreq_event_ops exynos_ppmu_ops = {
	.disable = exynos_ppmu_disable,
	.set_event = exynos_ppmu_set_event,
	.get_event = exynos_ppmu_get_event,
};

/*
 * The devfreq-event ops structure for PPMU v2.0
 */
static int exynos_ppmu_v2_disable(struct devfreq_event_dev *edev)
{
	struct exynos_ppmu *info = devfreq_event_get_drvdata(edev);
	u32 pmnc, clear;

	/* Disable all counters */
	clear = (PPMU_CCNT_MASK | PPMU_PMCNT0_MASK | PPMU_PMCNT1_MASK
		| PPMU_PMCNT2_MASK | PPMU_PMCNT3_MASK);

	__raw_writel(clear, info->ppmu.base + PPMU_V2_FLAG);
	__raw_writel(clear, info->ppmu.base + PPMU_V2_INTENC);
	__raw_writel(clear, info->ppmu.base + PPMU_V2_CNTENC);
	__raw_writel(clear, info->ppmu.base + PPMU_V2_CNT_RESET);

	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CIG_CFG0);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CIG_CFG1);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CIG_CFG2);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CIG_RESULT);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CNT_AUTO);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CH_EV0_TYPE);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CH_EV1_TYPE);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CH_EV2_TYPE);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_CH_EV3_TYPE);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_SM_ID_V);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_SM_ID_A);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_SM_OTHERS_V);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_SM_OTHERS_A);
	__raw_writel(0x0, info->ppmu.base + PPMU_V2_INTERRUPT_RESET);

	/* Disable PPMU */
	pmnc = __raw_readl(info->ppmu.base + PPMU_V2_PMNC);
	pmnc &= ~PPMU_PMNC_ENABLE_MASK;
	__raw_writel(pmnc, info->ppmu.base + PPMU_V2_PMNC);

	return 0;
}

static int exynos_ppmu_v2_set_event(struct devfreq_event_dev *edev)
{
	struct exynos_ppmu *info = devfreq_event_get_drvdata(edev);
	int id = exynos_ppmu_find_ppmu_id(edev);
	u32 pmnc, cntens;

	/* Enable all counters */
	cntens = __raw_readl(info->ppmu.base + PPMU_V2_CNTENS);
	cntens |= (PPMU_CCNT_MASK | (PPMU_ENABLE << id));
	__raw_writel(cntens, info->ppmu.base + PPMU_V2_CNTENS);

	/* Set the event of proper data type monitoring */
	__raw_writel(edev->desc->event_type, info->ppmu.base + PPMU_V2_CH_EVx_TYPE(id));

	/* Reset cycle counter/performance counter and enable PPMU */
	pmnc = __raw_readl(info->ppmu.base + PPMU_V2_PMNC);
	pmnc &= ~(PPMU_PMNC_ENABLE_MASK
			| PPMU_PMNC_COUNTER_RESET_MASK
			| PPMU_PMNC_CC_RESET_MASK
			| PPMU_PMNC_CC_DIVIDER_MASK
			| PPMU_V2_PMNC_START_MODE_MASK);
	pmnc |= (PPMU_ENABLE << PPMU_PMNC_ENABLE_SHIFT);
	pmnc |= (PPMU_ENABLE << PPMU_PMNC_COUNTER_RESET_SHIFT);
	pmnc |= (PPMU_ENABLE << PPMU_PMNC_CC_RESET_SHIFT);
	pmnc |= (PPMU_V2_MODE_MANUAL << PPMU_V2_PMNC_START_MODE_SHIFT);
	__raw_writel(pmnc, info->ppmu.base + PPMU_V2_PMNC);

	return 0;
}

static int exynos_ppmu_v2_get_event(struct devfreq_event_dev *edev,
				    struct devfreq_event_data *edata)
{
	struct exynos_ppmu *info = devfreq_event_get_drvdata(edev);
	int id = exynos_ppmu_find_ppmu_id(edev);
	u32 pmnc, cntenc;
	u32 pmcnt_high, pmcnt_low;
	u64 load_count = 0;

	/* Disable PPMU */
	pmnc = __raw_readl(info->ppmu.base + PPMU_V2_PMNC);
	pmnc &= ~PPMU_PMNC_ENABLE_MASK;
	__raw_writel(pmnc, info->ppmu.base + PPMU_V2_PMNC);

	/* Read cycle count and performance count */
	edata->total_count = __raw_readl(info->ppmu.base + PPMU_V2_CCNT);

	switch (id) {
	case PPMU_PMNCNT0:
	case PPMU_PMNCNT1:
	case PPMU_PMNCNT2:
		load_count = __raw_readl(info->ppmu.base + PPMU_V2_PMNCT(id));
		break;
	case PPMU_PMNCNT3:
		pmcnt_high = __raw_readl(info->ppmu.base + PPMU_V2_PMCNT3_HIGH);
		pmcnt_low = __raw_readl(info->ppmu.base + PPMU_V2_PMCNT3_LOW);
		load_count = ((u64)((pmcnt_high & 0xff)) << 32)
			   + (u64)pmcnt_low;
		break;
	}
	edata->load_count = load_count;

	/* Disable all counters */
	cntenc = __raw_readl(info->ppmu.base + PPMU_V2_CNTENC);
	cntenc |= (PPMU_CCNT_MASK | (PPMU_ENABLE << id));
	__raw_writel(cntenc, info->ppmu.base + PPMU_V2_CNTENC);

	dev_dbg(&edev->dev, "%25s (load: %ld / %ld)\n", edev->desc->name,
					edata->load_count, edata->total_count);
	return 0;
}

static const struct devfreq_event_ops exynos_ppmu_v2_ops = {
	.disable = exynos_ppmu_v2_disable,
	.set_event = exynos_ppmu_v2_set_event,
	.get_event = exynos_ppmu_v2_get_event,
};

static const struct of_device_id exynos_ppmu_id_match[] = {
	{
		.compatible = "samsung,exynos-ppmu",
		.data = (void *)EXYNOS_TYPE_PPMU,
	}, {
		.compatible = "samsung,exynos-ppmu-v2",
		.data = (void *)EXYNOS_TYPE_PPMU_V2,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, exynos_ppmu_id_match);

static int of_get_devfreq_events(struct device_node *np,
				 struct exynos_ppmu *info)
{
	struct devfreq_event_desc *desc;
	struct device *dev = info->dev;
	struct device_node *events_np, *node;
	int i, j, count;
	const struct of_device_id *of_id;
	int ret;

	events_np = of_get_child_by_name(np, "events");
	if (!events_np) {
		dev_err(dev,
			"failed to get child node of devfreq-event devices\n");
		return -EINVAL;
	}

	count = of_get_child_count(events_np);
	desc = devm_kcalloc(dev, count, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	info->num_events = count;

	of_id = of_match_device(exynos_ppmu_id_match, dev);
	if (of_id)
		info->ppmu_type = (enum exynos_ppmu_type)of_id->data;
	else
		return -EINVAL;

	j = 0;
	for_each_child_of_node(events_np, node) {
		for (i = 0; i < ARRAY_SIZE(ppmu_events); i++) {
			if (!ppmu_events[i].name)
				continue;

			if (of_node_name_eq(node, ppmu_events[i].name))
				break;
		}

		if (i == ARRAY_SIZE(ppmu_events)) {
			dev_warn(dev,
				"don't know how to configure events : %pOFn\n",
				node);
			continue;
		}

		switch (info->ppmu_type) {
		case EXYNOS_TYPE_PPMU:
			desc[j].ops = &exynos_ppmu_ops;
			break;
		case EXYNOS_TYPE_PPMU_V2:
			desc[j].ops = &exynos_ppmu_v2_ops;
			break;
		}

		desc[j].driver_data = info;

		of_property_read_string(node, "event-name", &desc[j].name);
		ret = of_property_read_u32(node, "event-data-type",
					   &desc[j].event_type);
		if (ret) {
			/* Set the event of proper data type counting.
			 * Check if the data type has been defined in DT,
			 * use default if not.
			 */
			if (info->ppmu_type == EXYNOS_TYPE_PPMU_V2) {
				int id;
				/* Not all registers take the same value for
				 * read+write data count.
				 */
				id = __exynos_ppmu_find_ppmu_id(desc[j].name);

				switch (id) {
				case PPMU_PMNCNT0:
				case PPMU_PMNCNT1:
				case PPMU_PMNCNT2:
					desc[j].event_type = PPMU_V2_RO_DATA_CNT
						| PPMU_V2_WO_DATA_CNT;
					break;
				case PPMU_PMNCNT3:
					desc[j].event_type =
						PPMU_V2_EVT3_RW_DATA_CNT;
					break;
				}
			} else {
				desc[j].event_type = PPMU_RO_DATA_CNT |
					PPMU_WO_DATA_CNT;
			}
		}

		j++;
	}
	info->desc = desc;

	of_node_put(events_np);

	return 0;
}

static int exynos_ppmu_parse_dt(struct exynos_ppmu *info)
{
	struct device *dev = info->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	if (!np) {
		dev_err(dev, "failed to find devicetree node\n");
		return -EINVAL;
	}

	/* Maps the memory mapped IO to control PPMU register */
	info->ppmu.base = of_iomap(np, 0);
	if (IS_ERR_OR_NULL(info->ppmu.base)) {
		dev_err(dev, "failed to map memory region\n");
		return -ENOMEM;
	}

	info->ppmu.clk = devm_clk_get(dev, "ppmu");
	if (IS_ERR(info->ppmu.clk)) {
		info->ppmu.clk = NULL;
		dev_warn(dev, "cannot get PPMU clock\n");
	}

	ret = of_get_devfreq_events(np, info);
	if (ret < 0) {
		dev_err(dev, "failed to parse exynos ppmu dt node\n");
		goto err;
	}

	return 0;

err:
	iounmap(info->ppmu.base);

	return ret;
}

static int exynos_ppmu_probe(struct platform_device *pdev)
{
	struct exynos_ppmu *info;
	struct devfreq_event_dev **edev;
	struct devfreq_event_desc *desc;
	int i, ret = 0, size;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;

	/* Parse dt data to get resource */
	ret = exynos_ppmu_parse_dt(info);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to parse devicetree for resource\n");
		return ret;
	}
	desc = info->desc;

	size = sizeof(struct devfreq_event_dev *) * info->num_events;
	info->edev = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!info->edev) {
		ret = -ENOMEM;
		goto err;
	}
	edev = info->edev;
	platform_set_drvdata(pdev, info);

	for (i = 0; i < info->num_events; i++) {
		edev[i] = devm_devfreq_event_add_edev(&pdev->dev, &desc[i]);
		if (IS_ERR(edev[i])) {
			ret = PTR_ERR(edev[i]);
			dev_err(&pdev->dev,
				"failed to add devfreq-event device\n");
			goto err;
		}

		pr_info("exynos-ppmu: new PPMU device registered %s (%s)\n",
			dev_name(&pdev->dev), desc[i].name);
	}

	ret = clk_prepare_enable(info->ppmu.clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare ppmu clock\n");
		return ret;
	}

	return 0;
err:
	iounmap(info->ppmu.base);

	return ret;
}

static int exynos_ppmu_remove(struct platform_device *pdev)
{
	struct exynos_ppmu *info = platform_get_drvdata(pdev);

	clk_disable_unprepare(info->ppmu.clk);
	iounmap(info->ppmu.base);

	return 0;
}

static struct platform_driver exynos_ppmu_driver = {
	.probe	= exynos_ppmu_probe,
	.remove	= exynos_ppmu_remove,
	.driver = {
		.name	= "exynos-ppmu",
		.of_match_table = exynos_ppmu_id_match,
	},
};
module_platform_driver(exynos_ppmu_driver);

MODULE_DESCRIPTION("Exynos PPMU(Platform Performance Monitoring Unit) driver");
MODULE_AUTHOR("Chanwoo Choi <cw00.choi@samsung.com>");
MODULE_LICENSE("GPL");
