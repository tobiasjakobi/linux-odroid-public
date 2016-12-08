/*
 * Copyright (C) 2011-2014 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "mali_hw_core.h"
#include "mali_osk.h"
#include "mali_kernel_common.h"
#include "mali_osk_mali.h"

#include <asm/io.h>

_mali_osk_errcode_t mali_hw_core_create(struct mali_hw_core *core, const _mali_osk_resource_t *resource, u32 reg_size)
{
	core->phys_addr = resource->base;
	core->phys_offset = resource->base - _mali_osk_resource_base_address();
	core->description = resource->description;
	core->size = reg_size;

	MALI_DEBUG_ASSERT(core->phys_offset < core->phys_addr);

	core->mapped_registers = (mali_io_address)ioremap_nocache(core->phys_addr, core->size);
	if (NULL != core->mapped_registers) {
		return _MALI_OSK_ERR_OK;
	} else {
		MALI_PRINT_ERROR("Failed to map memory region for core %s at phys_addr 0x%08X\n", core->description, core->phys_addr);
	}

	return _MALI_OSK_ERR_FAULT;
}

void mali_hw_core_delete(struct mali_hw_core *core)
{
	if (NULL != core->mapped_registers) {
		iounmap((void*)core->mapped_registers);
		core->mapped_registers = NULL;
	}
}
