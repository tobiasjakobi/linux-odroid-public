#
# Copyright (C) 2010-2014 ARM Limited. All rights reserved.
# 
# This program is free software and is provided to you under the terms of the GNU General Public License version 2
# as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
# 
# A copy of the licence is included with the program, and can also be obtained from Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#

#
# Makefile for the Mali GPU device driver.
#

timestamp ?= default
mali_release = r5p0-01rel0

# Skip inclusion of target platform specific sources when cleaning.
ifneq ($(PHONY),__clean)
    ifneq ($(CONFIG_MALI_TARGET_PLATFORM),)
        mali_platform = $(patsubst "%",%,$(CONFIG_MALI_TARGET_PLATFORM))
    else
        $(error No Mali target platform selected)
    endif

    mali_platform_files = $(notdir $(wildcard $(src)/platform/$(mali_platform)/*.c))
endif

maligpu-y += \
	linux/mali_osk_irq.o \
	linux/mali_osk_wq.o \
	linux/mali_osk_locks.o \
	linux/mali_osk_wait_queue.o \
	linux/mali_osk_low_level_mem.o \
	linux/mali_osk_misc.o \
	linux/mali_osk_mali.o \
	linux/mali_osk_notification.o \
	linux/mali_osk_time.o \
	linux/mali_osk_timers.o

maligpu-y += linux/mali_memory.o linux/mali_memory_os_alloc.o
maligpu-y += linux/mali_memory_external.o
maligpu-y += linux/mali_memory_block_alloc.o

maligpu-y += \
	linux/mali_ukk_mem.o \
	linux/mali_ukk_gp.o \
	linux/mali_ukk_pp.o \
	linux/mali_ukk_core.o \
	linux/mali_ukk_soft_job.o \
	linux/mali_ukk_timeline.o

maligpu-y += \
	common/mali_kernel_core.o \
	linux/mali_kernel_linux.o \
	common/mali_kernel_descriptor_mapping.o \
	common/mali_session.o \
	linux/mali_device_pause_resume.o \
	common/mali_kernel_vsync.o \
	linux/mali_ukk_vsync.o \
	linux/mali_kernel_sysfs.o \
	common/mali_mmu.o \
	common/mali_mmu_page_directory.o \
	common/mali_mem_validation.o \
	common/mali_hw_core.o \
	common/mali_gp.o \
	common/mali_pp.o \
	common/mali_pp_job.o \
	common/mali_gp_job.o \
	common/mali_soft_job.o \
	common/mali_scheduler.o \
	common/mali_executor.o \
	common/mali_group.o \
	common/mali_dlbu.o \
	common/mali_broadcast.o \
	common/mali_pm.o \
	common/mali_pmu.o \
	common/mali_user_settings_db.o \
	common/mali_kernel_utilization.o \
	common/mali_control_timer.o \
	common/mali_l2_cache.o \
	common/mali_timeline.o \
	common/mali_timeline_fence_wait.o \
	common/mali_timeline_sync_fence.o \
	common/mali_spinlock_reentrant.o \
	common/mali_pm_domain.o \
	linux/mali_osk_pm.o \
	linux/mali_pmu_power_up_down.o

maligpu-$(CONFIG_MALI400_PROFILING) += \
	linux/mali_ukk_profiling.o \
	linux/mali_osk_profiling.o

maligpu-$(CONFIG_MALI400_INTERNAL_PROFILING) += \
	linux/mali_profiling_internal.o \
	timestamp-$(TIMESTAMP)/mali_timestamp.o
ccflags-$(CONFIG_MALI400_INTERNAL_PROFILING) += \
	-I$(src)/timestamp-$(timestamp)

maligpu-$(CONFIG_SYNC) += linux/mali_sync.o
ccflags-$(CONFIG_SYNC) += -Idrivers/staging/android

maligpu-$(CONFIG_DMA_SHARED_BUFFER) += linux/mali_memory_dma_buf.o
maligpu-$(CONFIG_MALI_PM_DVFS) += common/mali_dvfs_policy.o

ifneq ($(mali_platform_files),)
    maligpu-y += $(addprefix platform/$(mali_platform)/,$(mali_platform_files:.c=.o))
endif

# Clean object files from all taget platforms.
clean-files := $(wildcard $(src)/platform/*/*.o)

ccflags-y += -I$(src) -I$(src)/include -I$(src)/common -I$(src)/linux -I$(src)/platform
ccflags-y += -DDRIVER_REV_STRING=\"$(mali_release)\"
ccflags-$(CONFIG_MALI400_DEBUG) += -DDEBUG

obj-$(CONFIG_MALI400) += maligpu.o
