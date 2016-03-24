// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2012 Samsung Electronics Co.Ltd
 * Authors: Joonyoung Shim <jy0922.shim@samsung.com>
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <drm/drm_file.h>
#include <drm/exynos_drm.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_g2d.h"
#include "exynos_drm_gem.h"

#define G2D_HW_MAJOR_VER		4
#define G2D_HW_MINOR_VER		1

/* valid register range set for user: 0x0104 ~ 0x0880 */
#define G2D_VALID_START			0x0104
#define G2D_VALID_END			0x0880

/*
 * G2D hardware register positions are always a multiple of four.
 * This enumerator stores the 'normalized' positions.
 * The reason behind this is to allow the compiler to better
 * optimize switch statements during commandlist validation.
 */
enum g2d_hw_registers {
	/* general */
	G2D_SOFT_RESET			= 0x000,
	G2D_INTEN			= 0x001,
	G2D_INTC_PEND			= 0x003,
	G2D_FIFO_STAT			= 0x004,
	G2D_AXI_MODE			= 0x007,
	G2D_DMA_SFR_BASE_ADDR		= 0x020,
	G2D_DMA_COMMAND			= 0x021,
	G2D_DMA_EXE_LIST_NUM		= 0x022,
	G2D_DMA_STATUS			= 0x023,
	G2D_DMA_HOLD_CMD		= 0x024,

	/* command */
	G2D_BITBLT_START		= 0x040,
	G2D_BITBLT_COMMAND		= 0x041,
	G2D_BLEND_FUNCTION		= 0x042,
	G2D_ROUND_MODE			= 0x043,

	/* parameter settings */
	G2D_ROTATE			= 0x080,
	G2D_SRC_MASK_DIRECT		= 0x081,
	G2D_DST_PAT_DIRECT		= 0x082,

	/* source */
	G2D_SRC_SELECT			= 0x0C0,
	G2D_SRC_BASE_ADDR		= 0x0C1,
	G2D_SRC_STRIDE			= 0x0C2,
	G2D_SRC_COLOR_MODE		= 0x0C3,
	G2D_SRC_LEFT_TOP		= 0x0C4,
	G2D_SRC_RIGHT_BOTTOM		= 0x0C5,
	G2D_SRC_PLANE2_BASE_ADDR	= 0x0C6,
	G2D_SRC_REPEAT_MODE		= 0x0C7,
	G2D_SRC_PAD_VALUE		= 0x0C8,
	G2D_SRC_A8_RGB_EXT		= 0x0C9,
	G2D_SRC_SCALE_CTRL		= 0x0CA,
	G2D_SRC_XSCALE			= 0x0CB,
	G2D_SRC_YSCALE			= 0x0CC,

	/* destination */
	G2D_DST_SELECT			= 0x100,
	G2D_DST_BASE_ADDR		= 0x101,
	G2D_DST_STRIDE			= 0x102,
	G2D_DST_COLOR_MODE		= 0x103,
	G2D_DST_LEFT_TOP		= 0x104,
	G2D_DST_RIGHT_BOTTOM		= 0x105,
	G2D_DST_PLANE2_BASE_ADDR	= 0x106,
	G2D_DST_A8_RGB_EXT		= 0x107,

	/* pattern */
	G2D_PAT_BASE_ADDR		= 0x140,
	G2D_PAT_SIZE			= 0x141,
	G2D_PAT_COLOR_MODE		= 0x142,
	G2D_PAT_OFFSET			= 0x143,
	G2D_PAT_STRIDE			= 0x144,

	/* mask */
	G2D_MSK_BASE_ADDR		= 0x148,
	G2D_MSK_STRIDE			= 0x149,
	G2D_MSK_LEFT_TOP		= 0x14A,
	G2D_MSK_RIGHT_BOTTOM		= 0x14B,
	G2D_MSK_MODE			= 0x14C,
	G2D_MSK_REPEAT_MODE		= 0x14D,
	G2D_MSK_PAD_VALUE		= 0x14E,
	G2D_MSK_SCALE_CTRL		= 0x14F,
	G2D_MSK_XSCALE			= 0x150,
	G2D_MSK_YSCALE			= 0x151,

	/* clipping window */
	G2D_CW_LEFT_TOP			= 0x180,
	G2D_CW_RIGHT_BOTTOM		= 0x181,

	/* third operand, ROP and alpha setting */
	G2D_THIRD_OPERAND		= 0x184,
	G2D_ROP4			= 0x185,
	G2D_ALPHA			= 0x186,

	/* color settings */
	G2D_FG_COLOR			= 0x1C0,
	G2D_BG_COLOR			= 0x1C1,
	G2D_BS_COLOR			= 0x1C2,
	G2D_SF_COLOR			= 0x1C3,

	/*
	 * There are more HW registers for colorkeying and gamma table
	 * in the range 0x0710~0x880. But we're not using them at
	 * the moment, so omit them for now.
	 */
};

/* G2D_SOFT_RESET */
#define G2D_SFRCLEAR			(1 << 1)
#define G2D_R				(1 << 0)

/* G2D_INTEN */
#define G2D_INTEN_ACF			(1 << 3)
#define G2D_INTEN_UCF			(1 << 2)
#define G2D_INTEN_GCF			(1 << 1)
#define G2D_INTEN_SCF			(1 << 0)

/* G2D_INTC_PEND */
#define G2D_INTP_ACMD_FIN		(1 << 3)
#define G2D_INTP_UCMD_FIN		(1 << 2)
#define G2D_INTP_GCMD_FIN		(1 << 1)
#define G2D_INTP_SCMD_FIN		(1 << 0)

/* G2D_AXI_MODE */
#define G2D_MAX_BURST_LEN_MASK		(3 << 24)
#define G2D_MAX_BURST_LEN_SHIFT		24
#define G2D_AXI_AWUSERS_SHIFT		16
#define G2D_AXI_ARUSERS_SHIFT		8
#define G2D_AXI_AWCACHE_SHIFT		4
#define G2D_AXI_ARCACHE_SHIFT		0

/* G2D_DMA_COMMAND */
#define G2D_DMA_HALT			(1 << 2)
#define G2D_DMA_CONTINUE		(1 << 1)
#define G2D_DMA_START			(1 << 0)

/* G2D_DMA_STATUS */
#define G2D_DMA_LIST_DONE_COUNT		(0xFF << 17)
#define G2D_DMA_BITBLT_DONE_COUNT	(0xFFFF << 1)
#define G2D_DMA_DONE			(1 << 0)
#define G2D_DMA_LIST_DONE_COUNT_OFFSET	17

/* G2D_DMA_HOLD_CMD */
#define G2D_USER_HOLD			(1 << 2)
#define G2D_LIST_HOLD			(1 << 1)
#define G2D_BITBLT_HOLD			(1 << 0)

/* G2D_BITBLT_START */
#define G2D_START_CASESEL		(1 << 2)
#define G2D_START_N_HOLD		(1 << 1)
#define G2D_START_BITBLT		(1 << 0)

/* G2D_BITBLT_COMMAND */
#define G2D_BITBLT_MASK_ROP4		(1 << 0)
#define G2D_BITBLT_MASK_NORMAL		(1 << 1)
#define G2D_BITBLT_ENABLE_CW		(1 << 8)
#define G2D_BITBLT_SOLID_FILL		(1 << 28)

/* G2D_SRC_SELECT */
#define G2D_SRC_SEL_DEFAULT		0x1

/* G2D_PAT_SIZE */
#define G2D_PAT_SIZE_MASK		0x1FFF
#define G2D_PAT_HEIGHT_SHIFT		13
#define G2D_PAT_OFFSET_MASK		0x1FFF
#define G2D_PAT_YOFFSET_SHIFT		13
#define G2D_PAT_DEFAULT			1

/* ROP3 masks for G2D_ROP4 */
#define G2D_ROP4_UNMASKED_ROP3		(0xFF << 0)
#define G2D_ROP4_MASKED_ROP3		(0xFF << 8)
#define G2D_ROP4_MASKED_SHIFT		8
#define G2D_ROP4_DEFAULT		0xCC

/* select masks for G2D_THIRD_OPERAND */
#define G2D_THIRD_OP_MASK_UNMASKED	0x0003
#define G2D_THIRD_OP_MASK_MASKED	0x0030
#define G2D_THIRD_OP_DEFAULT		0x0011

/*
 * buffer color format
 * - for source and destination all formats are valid
 * - for pattern all but the YCbCr/alpha/luminance ones are valid
 */
enum g2d_color_format {
	G2D_FMT_XRGB8888,
	G2D_FMT_ARGB8888,
	G2D_FMT_RGB565,
	G2D_FMT_XRGB1555,
	G2D_FMT_ARGB1555,
	G2D_FMT_XRGB4444,
	G2D_FMT_ARGB4444,
	G2D_FMT_PACKED_RGB888,
	G2D_FMT_YCbCr444,
	G2D_FMT_YCbCr422,
	G2D_FMT_YCbCr420,
	G2D_FMT_A8,
	G2D_FMT_L8,
	G2D_FMT_MAX,
	G2D_FMT_MASK = 0xF,
	G2D_FMT_YCbCr_2PLANE = (1 << 8),
};

enum g2d_mask_mode {
	G2D_MSK_1BIT,
	G2D_MSK_4BIT,
	G2D_MSK_8BIT,
	G2D_MSK_16BIT_565,
	G2D_MSK_16BIT_1555,
	G2D_MSK_16BIT_4444,
	G2D_MSK_32BIT_8888,
	G2D_MSK_4BIT_WINCE_AA_FONT,
	G2D_MSK_MAX,
	G2D_MSK_MASK = 0xF,
};

/* buffer valid length */
#define G2D_LEN_MIN			1
#define G2D_LEN_MAX			8000

#define G2D_CMDLIST_SIZE		(PAGE_SIZE / 4)
#define G2D_CMDLIST_NUM			64
#define G2D_CMDLIST_POOL_SIZE		(G2D_CMDLIST_SIZE * G2D_CMDLIST_NUM)
#define G2D_CMDLIST_DATA_NUM		(G2D_CMDLIST_SIZE / sizeof(u32) - 2)

/* maximum buffer pool size of userptr is 64MB as default */
#define G2D_MAX_POOL		(64 * 1024 * 1024)

enum g2d_buf_type {
	BUF_TYPE_SRC,
	BUF_TYPE_SRC_PLANE2,
	BUF_TYPE_DST,
	BUF_TYPE_DST_PLANE2,
	BUF_TYPE_PAT,
	BUF_TYPE_MSK,
	MAX_BUF_TYPE_NR
};

enum g2d_flag_bits {
	/*
	 * If set, suspends the runqueue worker after the currently
	 * processed node is finished.
	 */
	G2D_BIT_SUSPEND_RUNQUEUE,
	/*
	 * If set, indicates that the engine is currently busy.
	 */
	G2D_BIT_ENGINE_BUSY,
};

enum g2d_rect_type {
	RECT_TYPE_SRC,
	RECT_TYPE_DST,
	RECT_TYPE_MSK,
	RECT_TYPE_CW,
	MAX_RECT_TYPE_NR
};

/* cmdlist data structure */
struct g2d_cmdlist {
	u32		head;
	unsigned long	data[G2D_CMDLIST_DATA_NUM];
	u32		last;	/* last data offset */
};

/* Basic G2D rectangle structure. */
struct g2d_rect {
	unsigned long	left_top;
	unsigned long	right_bottom;
};

/*
 * Structure describing a buffer used in a G2D operation.
 *
 * @data: location of the buffer definition in the cmdlist
 * @format: color format of the buffer
 * @bpp: bits per pixel
 * @stride: buffer stride/pitch in bytes
 *
 * @obj: pointer to actual buffer object
 * @size: total size in bytes of the associated buffer object
 *
 * @is_userptr: 'true' if the buffer object is a userspace pointer
 * @is_ycbcr: 'true' if the buffer format is YCbCr
 *
 * For plane2 buffers the format, bpp and stride fields are not
 * set. The information is provided by the corresponding source
 * and destination buffers.
 */
struct g2d_buf_info {
	unsigned long	*data;
	unsigned int	format;
	unsigned int	bpp;
	unsigned int	stride;

	void		*obj;
	unsigned int	size;

	bool		is_userptr;
	bool		is_ycbcr;
};

/*
 * Structure describing a BitBLT unit in a G2D operation.
 *
 * @rects: src, dst, msk and cw rectangle
 *
 * @pat_size: pattern size argument
 * @pat_offset: pattern offset argument
 *
 * @src_select: source selection argument
 * @bitblt_command: bitblt command argument
 * @third_operand: third operand argument
 * @rop4: ROP4 argument
 * @rotate: rotate argument
 *
 * The structure is used to keep information necessary to
 * validate the BitBLT unit.
 */
struct g2d_bitblt_info {
	struct g2d_rect	rects[MAX_RECT_TYPE_NR];

	unsigned long	pat_size;
	unsigned long	pat_offset;

	unsigned long	src_select;
	unsigned long	bitblt_command;
	unsigned long	third_operand;
	unsigned long	rop4;
	unsigned long	rotate;
};

struct drm_exynos_pending_g2d_event {
	struct drm_pending_event	base;
	struct drm_exynos_g2d_event	event;
};

struct g2d_cmdlist_userptr {
	struct list_head	list;
	dma_addr_t		dma_addr;
	uint64_t		user_addr;
	uint64_t		size;
	unsigned int		npages;
	struct frame_vector	*vec;
	struct sg_table		*sgt;
	enum dma_data_direction direction;
	atomic_t		refcount;
};

struct g2d_cmdlist_node {
	struct list_head	list;
	struct g2d_cmdlist	*cmdlist;
	dma_addr_t		dma_addr;
	struct g2d_buf_info	buf_info[MAX_BUF_TYPE_NR];
	struct g2d_bitblt_info	bitblt_info;

	struct drm_exynos_pending_g2d_event	*event;
};

struct g2d_runqueue_node {
	struct list_head	list;
	struct list_head	run_cmdlist;
	struct list_head	event_list;
	struct drm_file		*filp;
	pid_t			pid;
	struct completion	complete;
	int			async;
};

struct g2d_data {
	struct device			*dev;
	void				*dma_priv;
	struct clk			*gate_clk;
	void __iomem			*regs;
	int				irq;
	struct workqueue_struct		*g2d_workq;
	struct work_struct		runqueue_work;
	struct drm_device		*drm_dev;
	unsigned long			flags;

	/* cmdlist */
	struct g2d_cmdlist_node		*cmdlist_node;
	struct list_head		free_cmdlist;
	struct mutex			cmdlist_mutex;
	dma_addr_t			cmdlist_pool;
	void				*cmdlist_pool_virt;
	unsigned long			cmdlist_dma_attrs;

	/* runqueue*/
	struct g2d_runqueue_node	*runqueue_node;
	struct list_head		runqueue;
	struct mutex			runqueue_mutex;
	struct kmem_cache		*runqueue_slab;

	unsigned long			current_pool;
	struct mutex			userptr_mutex;
};

static inline u32 g2d_reg_read(struct g2d_data *g2d, u32 reg_id)
{
	return readl(g2d->regs + (reg_id << 2));
}

static inline void g2d_reg_write(struct g2d_data *g2d, u32 reg_id, u32 val)
{
	writel(val, g2d->regs + (reg_id << 2));
}

static inline void g2d_add_cmd(struct g2d_cmdlist *cmdlist, unsigned long cmd,
	unsigned long val)
{
	cmdlist->data[cmdlist->last++] = cmd << 2;
	cmdlist->data[cmdlist->last++] = val;
}

static inline void g2d_hw_reset(struct g2d_data *g2d)
{
	g2d_reg_write(g2d, G2D_SOFT_RESET, G2D_R | G2D_SFRCLEAR);
	clear_bit(G2D_BIT_ENGINE_BUSY, &g2d->flags);
}

static void g2d_set_max_burst_length(struct g2d_data *g2d, unsigned len)
{
	u32 axi_mode;
	u32 burst;

	switch (len) {
	case 2:
	case 4:
	case 8:
	case 16:
		break;
	default:
		return;
	}

	/* The default max burst length is two (no bits set). */
	burst = len >> 2;

	axi_mode = g2d_reg_read(g2d, G2D_AXI_MODE);

	if ((axi_mode & G2D_MAX_BURST_LEN_MASK) == burst)
		return;

	axi_mode &= ~G2D_MAX_BURST_LEN_MASK;
	axi_mode |= burst;
	g2d_reg_write(g2d, G2D_AXI_MODE, axi_mode);
}

static void g2d_set_axi_mode(struct g2d_data *g2d, unsigned arcache,
	unsigned awcache, unsigned arusers, unsigned awusers)
{
	u32 axi_mode;

	axi_mode = g2d_reg_read(g2d, G2D_AXI_MODE);

	axi_mode &= ~0x1F1FFF;

	axi_mode |= ((arcache & 0xF) << G2D_AXI_ARCACHE_SHIFT);
	axi_mode |= ((awcache & 0xF) << G2D_AXI_AWCACHE_SHIFT);

	axi_mode |= ((arusers & 0x1F) << G2D_AXI_ARUSERS_SHIFT);
	axi_mode |= ((awusers & 0x1F) << G2D_AXI_AWUSERS_SHIFT);

	g2d_reg_write(g2d, G2D_AXI_MODE, axi_mode);
}

static void g2d_dump_axi_mode_reg(struct g2d_data *g2d)
{
	struct device *dev = g2d->dev;
	u32 axi_mode;

	axi_mode = g2d_reg_read(g2d, G2D_AXI_MODE);

	dev_info(dev, "AxCACHE = {0x%x, 0x%x}\n",
		(axi_mode >> G2D_AXI_ARCACHE_SHIFT) & 0xF,
		(axi_mode >> G2D_AXI_AWCACHE_SHIFT) & 0xF);
	dev_info(dev, "AxUSERS = {0x%x, 0x%x}\n",
		(axi_mode >> G2D_AXI_ARUSERS_SHIFT) & 0x1F,
		(axi_mode >> G2D_AXI_AWUSERS_SHIFT) & 0x1F);
}

static int g2d_init_cmdlist(struct g2d_data *g2d)
{
	struct device *dev = g2d->dev;
	struct g2d_cmdlist_node *node;
	int nr;
	int ret;

	g2d->cmdlist_dma_attrs = DMA_ATTR_WRITE_COMBINE;

	g2d->cmdlist_pool_virt = dma_alloc_attrs(to_dma_dev(g2d->drm_dev),
						G2D_CMDLIST_POOL_SIZE,
						&g2d->cmdlist_pool, GFP_KERNEL,
						g2d->cmdlist_dma_attrs);
	if (!g2d->cmdlist_pool_virt) {
		dev_err(dev, "failed to allocate dma memory\n");
		return -ENOMEM;
	}

	node = kcalloc(G2D_CMDLIST_NUM, sizeof(*node), GFP_KERNEL);
	if (!node) {
		ret = -ENOMEM;
		goto err;
	}

	for (nr = 0; nr < G2D_CMDLIST_NUM; nr++) {
		node[nr].cmdlist =
			g2d->cmdlist_pool_virt + nr * G2D_CMDLIST_SIZE;
		node[nr].dma_addr =
			g2d->cmdlist_pool + nr * G2D_CMDLIST_SIZE;

		list_add_tail(&node[nr].list, &g2d->free_cmdlist);
	}

	return 0;

err:
	dma_free_attrs(to_dma_dev(g2d->drm_dev), G2D_CMDLIST_POOL_SIZE,
			g2d->cmdlist_pool_virt,
			g2d->cmdlist_pool, g2d->cmdlist_dma_attrs);
	return ret;
}

static void g2d_fini_cmdlist(struct g2d_data *g2d)
{
	kfree(g2d->cmdlist_node);

	if (g2d->cmdlist_pool_virt && g2d->cmdlist_pool) {
		dma_free_attrs(to_dma_dev(g2d->drm_dev),
				G2D_CMDLIST_POOL_SIZE,
				g2d->cmdlist_pool_virt,
				g2d->cmdlist_pool, g2d->cmdlist_dma_attrs);
	}
}

static struct g2d_cmdlist_node *g2d_get_cmdlist(struct g2d_data *g2d)
{
	struct device *dev = g2d->dev;
	struct g2d_cmdlist_node *node;

	mutex_lock(&g2d->cmdlist_mutex);
	if (unlikely(list_empty(&g2d->free_cmdlist))) {
		dev_err(dev, "there is no free cmdlist\n");
		mutex_unlock(&g2d->cmdlist_mutex);
		return NULL;
	}

	node = list_first_entry(&g2d->free_cmdlist, struct g2d_cmdlist_node,
				list);
	list_del_init(&node->list);
	mutex_unlock(&g2d->cmdlist_mutex);

	return node;
}

static void g2d_put_cmdlist(struct g2d_data *g2d, struct g2d_cmdlist_node *node)
{
	mutex_lock(&g2d->cmdlist_mutex);
	list_move_tail(&node->list, &g2d->free_cmdlist);
	mutex_unlock(&g2d->cmdlist_mutex);
}

static void g2d_add_cmdlist_to_inuse(struct drm_exynos_file_private *file_priv,
				     struct g2d_cmdlist_node *node)
{
	struct g2d_cmdlist_node *lnode;

	if (list_empty(&file_priv->inuse_cmdlist))
		goto add_to_list;

	/* this links to base address of new cmdlist */
	lnode = list_entry(file_priv->inuse_cmdlist.prev,
				struct g2d_cmdlist_node, list);
	lnode->cmdlist->data[lnode->cmdlist->last] = node->dma_addr;

add_to_list:
	list_add_tail(&node->list, &file_priv->inuse_cmdlist);

	if (node->event)
		list_add_tail(&node->event->base.link, &file_priv->event_list);
}

static inline enum dma_data_direction g2d_get_dma_direction(
	enum g2d_buf_type buf_type)
{
	switch (buf_type) {
	case BUF_TYPE_SRC:
	case BUF_TYPE_SRC_PLANE2:
	case BUF_TYPE_PAT:
	case BUF_TYPE_MSK:
		return DMA_TO_DEVICE;
	case BUF_TYPE_DST:
	case BUF_TYPE_DST_PLANE2:
		return DMA_FROM_DEVICE;
	default:
		return DMA_NONE;
	}
}

static void g2d_userptr_put_dma_addr(struct g2d_data *g2d, void *obj)
{
	struct g2d_cmdlist_userptr *userptr = obj;
	struct sg_table *sgt = userptr->sgt;

	WARN_ON(obj == NULL);

	atomic_dec(&userptr->refcount);
	if (atomic_read(&userptr->refcount) > 0)
		return;

	/*
	 * In case the G2D engine only reads from the buffer we can skip
	 * the sync here (the buffer content was never modified).
	 */
	if (userptr->direction != DMA_TO_DEVICE)
		dma_sync_sg_for_cpu(to_dma_dev(g2d->drm_dev), sgt->sgl, sgt->nents,
			userptr->direction);
}

/*
 * Lookup an userptr via its userspace address.
 * Must be called while holding the userptr mutex.
 */
static struct g2d_cmdlist_userptr *g2d_userptr_lookup(
	const struct drm_exynos_file_private *priv,
	uint64_t user_addr)
{
	struct g2d_cmdlist_userptr *userptr;

	list_for_each_entry(userptr, &priv->userptr_list, list) {
		if (userptr->user_addr == user_addr)
			return userptr;
	}

	return NULL;
}

/*
 * Unregister an existing userptr.
 * Must be called while holding the userptr mutex.
 */
static int g2d_userptr_unregister(struct g2d_data *g2d,
				  struct g2d_cmdlist_userptr *userptr,
				  bool force)
{
	struct sg_table *sgt;
	struct frame_vector *vec;
	struct page **pages;
	unsigned long attrs = 0;

	if (force)
		goto out;

	if (atomic_read(&userptr->refcount) > 0) {
		DRM_ERROR("userptr %llx still in use.", userptr->user_addr);
		return -EBUSY;
	}

out:
	list_del(&userptr->list);

	sgt = userptr->sgt;
	vec = userptr->vec;

	/*
	 * In case the reference count is zero proper sync has
	 * already happened and we can skip it here.
	 */
	if (!force)
		attrs = DMA_ATTR_SKIP_CPU_SYNC;

	dma_unmap_sg_attrs(to_dma_dev(g2d->drm_dev), sgt->sgl, sgt->nents,
		userptr->direction, attrs);

	pages = frame_vector_pages(vec);
	if (!IS_ERR(pages)) {
		int i;

		for (i = 0; i < frame_vector_count(vec); i++)
			set_page_dirty_lock(pages[i]);
	}
	put_vaddr_frames(vec);
	frame_vector_destroy(vec);

	sg_free_table(sgt);
	kfree(sgt);

	DRM_DEBUG_KMS("userptr (addr = %llx, size = %llu) unregistered.\n",
		userptr->user_addr, userptr->size);

	g2d->current_pool -= userptr->npages << PAGE_SHIFT;

	kfree(userptr);

	return 0;
}

/*
 * Register an new userptr.
 * Must be called while holding the userptr mutex.
 */
static int g2d_userptr_register(struct g2d_data *g2d, uint64_t user_addr, 
				uint64_t size, uint32_t flags,
				struct drm_file *filp)
{
	struct drm_exynos_file_private *file_priv = filp->driver_priv;
	struct g2d_cmdlist_userptr *userptr;
	struct frame_vector *vec;
	struct sg_table *sgt;
	unsigned long start, end;
	unsigned int npages, offset;
	enum dma_data_direction dma_dir;
	int ret;

	userptr = g2d_userptr_lookup(file_priv, user_addr);
	if (userptr) {
		DRM_ERROR("userptr %llx already registered.\n", user_addr);
		return -EEXIST;
	}

	/*
	 * If the IOMMU is not available then the G2D can only operate on
	 * physically contiguous memory. Since memory allocated from
	 * userspace is usually non-contiguous the userptr functionality
	 * is of very limited use in this case.
	 * Just error out without IOMMU for now.
	 */
	if (!is_drm_iommu_supported(g2d->drm_dev)) {
		DRM_ERROR("userptr functionality disabled (IOMMU unavailable).\n");
		return -EFAULT;
	}

	if (!size) {
		DRM_DEV_ERROR(g2d->dev, "invalid userptr size.\n");
		return -EINVAL;
	}

	switch (flags) {
	case G2D_USERPTR_FLAG_READ:
		dma_dir = DMA_TO_DEVICE;
		break;
	case G2D_USERPTR_FLAG_WRITE:
		dma_dir = DMA_FROM_DEVICE;
		break;
	case G2D_USERPTR_FLAG_RW:
		dma_dir = DMA_BIDIRECTIONAL;
		break;
	default:
		DRM_ERROR("invalid userptr flags.\n");
		return -EINVAL;
	}

	start = user_addr & PAGE_MASK;
	offset = user_addr & ~PAGE_MASK;
	end = PAGE_ALIGN(user_addr + size);
	npages = (end - start) >> PAGE_SHIFT;

	if (g2d->current_pool + (npages << PAGE_SHIFT) > G2D_MAX_POOL) {
		DRM_ERROR("userptr pool exhausted.\n");
		return -ENOSPC;
	}

	userptr = kzalloc(sizeof(*userptr), GFP_KERNEL);
	if (!userptr)
		return -ENOMEM;

	atomic_set(&userptr->refcount, 0);

	vec = frame_vector_create(npages);
	if (!vec) {
		ret = -EFAULT;
		goto err_free;
	}

	ret = get_vaddr_frames(start, npages, FOLL_FORCE | FOLL_WRITE, vec);
	if (ret != npages) {
		DRM_DEV_ERROR(g2d->dev,
			      "failed to get user pages from userptr.\n");
		if (ret < 0)
			goto err_destroy_framevec;
		ret = -EFAULT;
		goto err_put_framevec;
	}

	if (frame_vector_to_pages(vec) < 0) {
		ret = -EFAULT;
		goto err_put_framevec;
	}

	sgt = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!sgt) {
		ret = -ENOMEM;
		goto err_put_framevec;
	}

	ret = sg_alloc_table_from_pages(sgt, frame_vector_pages(vec),
					npages, offset, size, GFP_KERNEL);
	if (ret < 0) {
		DRM_DEV_ERROR(g2d->dev, "failed to get sgt from pages.\n");
		goto err_free_sgt;
	}

	/* We synchronize when the userptr is first used. */
	ret = dma_map_sg_attrs(to_dma_dev(g2d->drm_dev), sgt->sgl,
		sgt->nents, dma_dir, DMA_ATTR_SKIP_CPU_SYNC);

	if (!ret) {
		DRM_DEV_ERROR(g2d->dev, "failed to setup dma map for sgt.\n");
		goto err_sg_free_table;
	}

	userptr->dma_addr = sg_dma_address(sgt->sgl);
	userptr->vec = vec;
	userptr->sgt = sgt;
	userptr->user_addr = user_addr;
	userptr->size = size;
	userptr->direction = dma_dir;
	userptr->npages = npages;

	DRM_DEBUG_KMS("userptr (addr = %llx, size = %llu) registered.\n",
		user_addr, size);

	list_add_tail(&userptr->list, &file_priv->userptr_list);
	g2d->current_pool += npages << PAGE_SHIFT;

	return 0;

err_sg_free_table:
	sg_free_table(sgt);

err_free_sgt:
	kfree(sgt);

err_put_framevec:
	put_vaddr_frames(vec);

err_destroy_framevec:
	frame_vector_destroy(vec);

err_free:
	kfree(userptr);

	return ret;
}

static dma_addr_t *g2d_userptr_get_dma_addr(struct drm_device *drm_dev,
					struct g2d_cmdlist_userptr *userptr,
					enum dma_data_direction direction)
{
	struct sg_table *sgt = userptr->sgt;

	if (atomic_read(&userptr->refcount) > 0) {
		/*
		 * We check that read/write access mode matches here. If the
		 * buffer was registered as RW (bidirectional DMA) then both
		 * read and write transfer are allowed.
		 */
		if (userptr->direction != DMA_BIDIRECTIONAL &&
		    direction != userptr->direction) {
			DRM_ERROR("dma direction mismatch\n");
			return ERR_PTR(-EFAULT);
		}

		goto out;
	}

	dma_sync_sg_for_device(to_dma_dev(drm_dev), sgt->sgl, sgt->nents,
		userptr->direction);

out:
	atomic_inc(&userptr->refcount);
	return &userptr->dma_addr;
}

static int g2d_userptr_check_idle(const struct drm_exynos_file_private *file_priv,
	uint64_t user_addr)
{
	struct g2d_cmdlist_userptr *userptr;
	int refcount;

	userptr = g2d_userptr_lookup(file_priv, user_addr);

	if (!userptr)
		return -EINVAL;

	refcount = atomic_read(&userptr->refcount);

	return (refcount == 0) ? 1 : 0;
}

static void g2d_userptr_free_all(struct g2d_data *g2d, struct drm_file *filp)
{
	struct drm_exynos_file_private *file_priv = filp->driver_priv;
	struct g2d_cmdlist_userptr *userptr, *n;

	mutex_lock(&g2d->userptr_mutex);

	list_for_each_entry_safe(userptr, n, &file_priv->userptr_list, list) {
		g2d_userptr_unregister(g2d, userptr, true);
	}

	mutex_unlock(&g2d->userptr_mutex);
}

static enum g2d_buf_type g2d_get_buf_type(struct g2d_data *g2d, unsigned int reg_offset)
{
	enum g2d_buf_type buf_type;

	switch (reg_offset) {
	case G2D_SRC_BASE_ADDR:
	case G2D_SRC_STRIDE:
	case G2D_SRC_COLOR_MODE:
		buf_type = BUF_TYPE_SRC;
		break;
	case G2D_SRC_PLANE2_BASE_ADDR:
		buf_type = BUF_TYPE_SRC_PLANE2;
		break;
	case G2D_DST_BASE_ADDR:
	case G2D_DST_STRIDE:
	case G2D_DST_COLOR_MODE:
		buf_type = BUF_TYPE_DST;
		break;
	case G2D_DST_PLANE2_BASE_ADDR:
		buf_type = BUF_TYPE_DST_PLANE2;
		break;
	case G2D_PAT_BASE_ADDR:
	case G2D_PAT_STRIDE:
		buf_type = BUF_TYPE_PAT;
		break;
	case G2D_MSK_BASE_ADDR:
	case G2D_MSK_STRIDE:
	default:
		buf_type = BUF_TYPE_MSK;
		break;
	}

	return buf_type;
}

static bool g2d_is_left_top(unsigned int reg_offset)
{
	/*
	 * For all the available coordinate registers (src, dst, msk, cw)
	 * the left/top register always has even index, while the
	 * right/bottom register has odd index.
	 * Exploit this fact here.
	 */
	return !(reg_offset & 0x1);
}

static enum g2d_rect_type g2d_get_rect_type(unsigned int reg_offset)
{
	enum g2d_rect_type rect_type;

	switch (reg_offset) {
	case G2D_SRC_LEFT_TOP:
	case G2D_SRC_RIGHT_BOTTOM:
		rect_type = RECT_TYPE_SRC;
		break;
	case G2D_DST_LEFT_TOP:
	case G2D_DST_RIGHT_BOTTOM:
		rect_type = RECT_TYPE_DST;
		break;
	case G2D_MSK_LEFT_TOP:
	case G2D_MSK_RIGHT_BOTTOM:
		rect_type = RECT_TYPE_MSK;
		break;
	case G2D_CW_LEFT_TOP:
	case G2D_CW_RIGHT_BOTTOM:
	default:
		rect_type = RECT_TYPE_CW;
		break;
	}

	return rect_type;
}

/*
 * Returns the bits per pixel (bpp) value for a
 * given G2D color format.
 */
static unsigned int g2d_get_buf_bpp(unsigned int format)
{
	unsigned int bpp;

	switch (format) {
	case G2D_FMT_XRGB8888:
	case G2D_FMT_ARGB8888:
		bpp = 32;
		break;
	case G2D_FMT_RGB565:
	case G2D_FMT_XRGB1555:
	case G2D_FMT_ARGB1555:
	case G2D_FMT_XRGB4444:
	case G2D_FMT_ARGB4444:
		bpp = 16;
		break;
	case G2D_FMT_PACKED_RGB888:
		bpp = 24;
		break;
	default:
		bpp = 8;
		break;
	}

	return bpp;
}

/*
 * Returns the bits per pixel (bpp) value for a
 * given G2D mask mode.
 */
static unsigned int g2d_get_msk_bpp(unsigned int format)
{
	unsigned int bpp;

	switch (format) {
	case G2D_MSK_1BIT:
		bpp = 1;
		break;
	case G2D_MSK_4BIT:
	case G2D_MSK_4BIT_WINCE_AA_FONT:
		bpp = 4;
		break;
	case G2D_MSK_8BIT:
		bpp = 8;
		break;
	case G2D_MSK_16BIT_565:
	case G2D_MSK_16BIT_1555:
	case G2D_MSK_16BIT_4444:
		bpp = 16;
		break;
	case G2D_MSK_32BIT_8888:
	default:
		bpp = 32;
		break;
	}

	return bpp;
}

static bool g2d_is_lumi_alpha_fmt(unsigned int format)
{
	return (format == G2D_FMT_A8 || format == G2D_FMT_L8);
}

static bool g2d_is_ycbcr_fmt(unsigned int format)
{
	switch (format) {
	case G2D_FMT_YCbCr444:
	case G2D_FMT_YCbCr422:
	case G2D_FMT_YCbCr420:
		return true;
	default:
		return false;
	}
}

/* Check if a normal / non-YCbCr rectangle is valid. */
static bool g2d_is_normal_rect_valid(const struct g2d_rect *rect,
				const struct g2d_buf_info *buf_info)
{
	u16 left_x, top_y, right_x, bottom_y;

	int width, height;
	unsigned long last_row, last_pos;

	left_x = rect->left_top & 0x1fff;
	top_y = (rect->left_top >> 16) & 0x1fff;
	right_x = rect->right_bottom & 0x1fff;
	bottom_y = (rect->right_bottom >> 16) & 0x1fff;

	/* This check also makes sure that right_x > left_x. */
	width = (int)right_x - (int)left_x;
	if (width < G2D_LEN_MIN || width > G2D_LEN_MAX)
		return false;

	/* This check also makes sure that bottom_y > top_y. */
	height = (int)bottom_y - (int)top_y;
	if (height < G2D_LEN_MIN || height > G2D_LEN_MAX)
		return false;

	/* Compute the number of bytes used by the last row. */
	last_row = ((unsigned long)right_x * (unsigned long)buf_info->bpp + 7) / 8;

	/* Compute the position of the last byte that the engine accesses. */
	last_pos = ((unsigned long)bottom_y - 1) *
		(unsigned long)buf_info->stride + last_row - 1;

	/*
	 * Since right_x > left_x and bottom_y > top_y we already know
	 * that the first_pos < last_pos (first_pos being the position
	 * of the first byte the engine accesses), it just remains to
	 * check if last_pos is smaller then the buffer size.
	 */

	if (last_pos >= buf_info->size)
		return false;

	return true;
}

/* Check if a YCbCr rectangle is valid. */
static bool g2d_is_ycbcr_rect_valid(const struct g2d_rect *rect,
				const struct g2d_buf_info *buf_info)
{
	u16 left_x, top_y, right_x, bottom_y;

	unsigned int fmt;
	bool plane2;

	int width, height;
	unsigned long last_row, last_pos;

	left_x = rect->left_top & 0x1fff;
	top_y = (rect->left_top >> 16) & 0x1fff;
	right_x = rect->right_bottom & 0x1fff;
	bottom_y = (rect->right_bottom >> 16) & 0x1fff;

	fmt = buf_info->format & G2D_FMT_MASK;
	plane2 = buf_info->format & G2D_FMT_YCbCr_2PLANE;

	switch (fmt) {
	case G2D_FMT_YCbCr420:
		if ((top_y & 0x1) || (bottom_y & 0x1))
			return false;
		/* fall-through */
	case G2D_FMT_YCbCr422:
		if ((left_x & 0x1) || (right_x & 0x1))
			return false;
		break;
	case G2D_FMT_YCbCr444:
	default:
		break;
	}

	width = (int)right_x - (int)left_x;
	if (width < G2D_LEN_MIN || width > G2D_LEN_MAX)
		return false;

	height = (int)bottom_y - (int)top_y;
	if (height < G2D_LEN_MIN || height > G2D_LEN_MAX)
		return false;

	/* Last position with offset. */
	last_pos = ((unsigned long)bottom_y - 1) *
		(unsigned long)buf_info->stride + (unsigned long)right_x;

	/*
	 * For YCbCr buffers stride equals the width of the buffer.
	 * In case of uniplanar YCbCr422 we need to accomodate for
	 * Y and CbCr being packed in a single 2-byte value.
	 */
	if (fmt == G2D_FMT_YCbCr422 && !plane2)
		last_pos *= 2;

	if (last_pos > buf_info->size)
		return false;

	/* If we have a uniplanar YCbCr format, we're done here. */
	if (!plane2)
		return true;

	/* Incrementing buf_info gives us the corresponding plane2 buffer. */
	buf_info++;

	/* Adjust for different chroma subsampling. */
	switch (fmt) {
	case G2D_FMT_YCbCr420:
		last_row = right_x;
		last_pos = (bottom_y / 2) - 1;
		break;
	case G2D_FMT_YCbCr422:
		last_row = right_x;
		last_pos = bottom_y - 1;
		break;
	case G2D_FMT_YCbCr444:
	default:
		last_row = right_x * 2;
		last_pos = bottom_y - 1;
		break;
	}

	last_pos *= buf_info->stride;
	last_pos += last_row;

	if (last_pos > buf_info->size)
		return false;

	return true;
}

static bool g2d_is_rect_valid(const struct g2d_rect *rect,
				const struct g2d_buf_info *buf_info)
{
	bool ret;

	if (buf_info->is_ycbcr)
		ret = g2d_is_ycbcr_rect_valid(rect, buf_info);
	else
		ret = g2d_is_normal_rect_valid(rect, buf_info);

	if (!ret)
		DRM_DEBUG_KMS("rectangle is invalid\n");

	return ret;
}

static bool g2d_is_pattern_valid(const struct g2d_bitblt_info* info,
				const struct g2d_buf_info *buf_info)
{
	unsigned int width, height, offset;
	unsigned int last_row, last_pos;

	width = info->pat_size & G2D_PAT_SIZE_MASK;
	if (width == 0 || width > 8000)
		goto err;

	height = (info->pat_size >> G2D_PAT_HEIGHT_SHIFT) & G2D_PAT_SIZE_MASK;
	if (height == 0 || height > 8000)
		goto err;

	offset = info->pat_offset & G2D_PAT_OFFSET_MASK;
	if (offset >= 8000)
		goto err;

	offset = (info->pat_offset >> G2D_PAT_YOFFSET_SHIFT) & G2D_PAT_OFFSET_MASK;
	if (offset >= 8000)
		goto err;

	/* For details on the computations see g2d_is_rect_valid() above. */
	last_row = (width * buf_info->bpp + 7) / 8;
	last_pos = (height - 1) * buf_info->stride + last_row;

	if (last_pos >= buf_info->size)
		goto err;

	return true;

err:
	DRM_DEBUG_KMS("pattern is invalid\n");
	return false;
}

static int g2d_map_cmdlist_buffers(struct g2d_data *g2d,
				struct g2d_cmdlist_node *node,
				struct drm_device *drm_dev,
				struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;

	int ret;
	unsigned int i;

	for (i = 0; i < MAX_BUF_TYPE_NR; ++i) {
		struct g2d_buf_info *buf_info;
		unsigned long *data;
		unsigned long handle;
		dma_addr_t *addr;

		buf_info = &node->buf_info[i];

		if (!buf_info->data)
			continue;

		/* Consider a buffer with unset stride to be invalid. */
		if (!buf_info->stride) {
			ret = -EFAULT;
			goto err;
		}

		data = buf_info->data;
		handle = data[1];

		if (buf_info->is_userptr) {
			uint64_t user_addr;
			struct g2d_cmdlist_userptr *userptr;

			if (unlikely(copy_from_user(&user_addr, (void __user *)handle,
							sizeof(uint64_t)))) {
				ret = -EFAULT;
				goto err;
			}

			userptr = g2d_userptr_lookup(file_priv, user_addr);
			if (unlikely(!userptr)) {
				ret = -EFAULT;
				goto err;
			}

			addr = g2d_userptr_get_dma_addr(drm_dev,
							userptr,
							g2d_get_dma_direction(i));
			if (IS_ERR(addr)) {
				ret = -EFAULT;
				goto err;
			}

			buf_info->size = userptr->size;
			buf_info->obj = userptr;
		} else {
			struct exynos_drm_gem *exynos_gem;

			exynos_gem = exynos_drm_gem_get(file, handle);
			if (IS_ERR(exynos_gem)) {
				ret = -EFAULT;
				goto err;
			}

			addr = &exynos_gem->dma_addr;
			buf_info->size = exynos_gem->size;
			buf_info->obj = exynos_gem;
		}

		data[1] = *addr;
	}

	return 0;

err:
	DRM_DEBUG_KMS("mapping buffer %u failed\n", i);

	return ret;
}

static void g2d_unmap_cmdlist_buffers(struct g2d_data *g2d,
				  struct g2d_cmdlist_node *node,
				  struct drm_file *filp)
{
	unsigned int i;

	for (i = 0; i < MAX_BUF_TYPE_NR; ++i) {
		struct g2d_buf_info *buf_info;

		buf_info = &node->buf_info[i];

		if (!buf_info->obj)
			continue;

		if (buf_info->is_userptr)
			g2d_userptr_put_dma_addr(g2d, buf_info->obj);
		else
			exynos_drm_gem_put(buf_info->obj);

		buf_info->obj = NULL;
	}
}

static void g2d_dma_start(struct g2d_data *g2d,
			  struct g2d_runqueue_node *runqueue_node)
{
	struct g2d_cmdlist_node *node =
				list_first_entry(&runqueue_node->run_cmdlist,
						struct g2d_cmdlist_node, list);
	/* This should enable cache-coherent read/write operations. */
	g2d_set_axi_mode(g2d, 0x2, 0x2, 0x1, 0x1);

	set_bit(G2D_BIT_ENGINE_BUSY, &g2d->flags);
	g2d_reg_write(g2d, G2D_DMA_SFR_BASE_ADDR, node->dma_addr);
	g2d_reg_write(g2d, G2D_DMA_COMMAND, G2D_DMA_START);
}

static struct g2d_runqueue_node *g2d_get_runqueue_node(struct g2d_data *g2d)
{
	struct g2d_runqueue_node *runqueue_node;

	if (list_empty(&g2d->runqueue))
		return NULL;

	runqueue_node = list_first_entry(&g2d->runqueue,
					 struct g2d_runqueue_node, list);
	list_del_init(&runqueue_node->list);
	return runqueue_node;
}

static void g2d_free_runqueue_node(struct g2d_data *g2d,
				   struct g2d_runqueue_node *runqueue_node)
{
	struct g2d_cmdlist_node *node;

	mutex_lock(&g2d->cmdlist_mutex);
	/*
	 * commands in run_cmdlist have been completed so unmap all gem
	 * objects in each command node so that they are unreferenced.
	 */
	list_for_each_entry(node, &runqueue_node->run_cmdlist, list)
		;
	list_splice_tail_init(&runqueue_node->run_cmdlist, &g2d->free_cmdlist);
	mutex_unlock(&g2d->cmdlist_mutex);

	kmem_cache_free(g2d->runqueue_slab, runqueue_node);
}

/**
 * g2d_remove_runqueue_nodes - remove items from the list of runqueue nodes
 * @g2d: G2D state object
 * @file: if not zero, only remove items with this DRM file
 *
 * Has to be called under runqueue lock.
 */
static void g2d_remove_runqueue_nodes(struct g2d_data *g2d, struct drm_file *file)
{
	struct g2d_runqueue_node *node, *n;

	if (list_empty(&g2d->runqueue))
		return;

	list_for_each_entry_safe(node, n, &g2d->runqueue, list) {
		if (file && node->filp != file)
			continue;

		list_del_init(&node->list);
		g2d_free_runqueue_node(g2d, node);
	}
}

static void g2d_runqueue_worker(struct work_struct *work)
{
	struct g2d_data *g2d = container_of(work, struct g2d_data,
					    runqueue_work);
	struct g2d_runqueue_node *runqueue_node;

	/*
	 * The engine is busy and the completion of the current node is going
	 * to poke the runqueue worker, so nothing to do here.
	 */
	if (test_bit(G2D_BIT_ENGINE_BUSY, &g2d->flags))
		return;

	mutex_lock(&g2d->runqueue_mutex);

	runqueue_node = g2d->runqueue_node;
	g2d->runqueue_node = NULL;

	if (runqueue_node) {
		pm_runtime_mark_last_busy(g2d->dev);
		pm_runtime_put_autosuspend(g2d->dev);

		complete(&runqueue_node->complete);
		if (runqueue_node->async)
			g2d_free_runqueue_node(g2d, runqueue_node);
	}

	if (!test_bit(G2D_BIT_SUSPEND_RUNQUEUE, &g2d->flags)) {
		g2d->runqueue_node = g2d_get_runqueue_node(g2d);

		if (g2d->runqueue_node) {
			pm_runtime_get_sync(g2d->dev);
			g2d_dma_start(g2d, g2d->runqueue_node);
		}
	}

	mutex_unlock(&g2d->runqueue_mutex);
}

static void g2d_finish_event(struct g2d_data *g2d, u32 cmdlist_no)
{
	struct drm_device *drm_dev = g2d->drm_dev;
	struct g2d_runqueue_node *runqueue_node = g2d->runqueue_node;
	struct drm_exynos_pending_g2d_event *e;
	struct timespec64 now;

	if (list_empty(&runqueue_node->event_list))
		return;

	e = list_first_entry(&runqueue_node->event_list,
			     struct drm_exynos_pending_g2d_event, base.link);

	ktime_get_ts64(&now);
	e->event.tv_sec = now.tv_sec;
	e->event.tv_usec = now.tv_nsec / NSEC_PER_USEC;
	e->event.cmdlist_no = cmdlist_no;

	drm_send_event(drm_dev, &e->base);
}

static irqreturn_t g2d_irq_handler(int irq, void *dev_id)
{
	struct g2d_data *g2d = dev_id;
	u32 pending;

	pending = g2d_reg_read(g2d, G2D_INTC_PEND);
	if (pending)
		g2d_reg_write(g2d, G2D_INTC_PEND, pending);

	if (pending & G2D_INTP_GCMD_FIN) {
		u32 cmdlist_no = g2d_reg_read(g2d, G2D_DMA_STATUS);

		cmdlist_no = (cmdlist_no & G2D_DMA_LIST_DONE_COUNT) >>
						G2D_DMA_LIST_DONE_COUNT_OFFSET;

		g2d_finish_event(g2d, cmdlist_no);

		g2d_reg_write(g2d, G2D_DMA_HOLD_CMD, 0);
		if (!(pending & G2D_INTP_ACMD_FIN)) {
			g2d_reg_write(g2d, G2D_DMA_COMMAND, G2D_DMA_CONTINUE);
		}
	}

	if (pending & G2D_INTP_ACMD_FIN) {
		clear_bit(G2D_BIT_ENGINE_BUSY, &g2d->flags);
		queue_work(g2d->g2d_workq, &g2d->runqueue_work);
	}

	return IRQ_HANDLED;
}

/**
 * g2d_wait_finish - wait for the G2D engine to finish the current runqueue node
 * @g2d: G2D state object
 * @file: if not zero, only wait if the current runqueue node belongs
 *        to the DRM file
 *
 * Should the engine not become idle after a 100ms timeout, a hardware
 * reset is issued.
 */
static void g2d_wait_finish(struct g2d_data *g2d, struct drm_file *file)
{
	struct device *dev = g2d->dev;

	struct g2d_runqueue_node *runqueue_node = NULL;
	unsigned int tries = 10;

	mutex_lock(&g2d->runqueue_mutex);

	/* If no node is currently processed, we have nothing to do. */
	if (!g2d->runqueue_node)
		goto out;

	runqueue_node = g2d->runqueue_node;

	/* Check if the currently processed item belongs to us. */
	if (file && runqueue_node->filp != file)
		goto out;

	mutex_unlock(&g2d->runqueue_mutex);

	/* Wait for the G2D engine to finish. */
	while (tries-- && (g2d->runqueue_node == runqueue_node))
		mdelay(10);

	mutex_lock(&g2d->runqueue_mutex);

	if (g2d->runqueue_node != runqueue_node)
		goto out;

	dev_err(dev, "wait timed out, resetting engine...\n");
	g2d_hw_reset(g2d);

	/*
	 * After the hardware reset of the engine we are going to loose
	 * the IRQ which triggers the PM runtime put().
	 * So do this manually here.
	 */
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	complete(&runqueue_node->complete);
	if (runqueue_node->async)
		g2d_free_runqueue_node(g2d, runqueue_node);

out:
	mutex_unlock(&g2d->runqueue_mutex);
}

static int g2d_validate_base_cmds(struct g2d_data *g2d,
				struct g2d_cmdlist_node *node,
				unsigned int nr)
{
	struct g2d_cmdlist *cmdlist = node->cmdlist;
	unsigned int reg_offset = 0;
	u32 index;

	for (index = cmdlist->last - 2 * nr; index < cmdlist->last; index += 2) {
		struct g2d_buf_info *buf_info;
		enum g2d_buf_type buf_type;
		unsigned long fmt;

		reg_offset = cmdlist->data[index] & 0x0fff;
		if (unlikely(reg_offset < G2D_VALID_START ||
				reg_offset > G2D_VALID_END))
			goto err;
		if (unlikely(reg_offset & 0x3))
			goto err;

		reg_offset >>= 2;

		switch (reg_offset) {
		case G2D_SRC_BASE_ADDR:
		case G2D_SRC_PLANE2_BASE_ADDR:
		case G2D_DST_BASE_ADDR:
		case G2D_DST_PLANE2_BASE_ADDR:
		case G2D_PAT_BASE_ADDR:
		case G2D_MSK_BASE_ADDR:
			buf_type = g2d_get_buf_type(g2d, reg_offset);
			buf_info = &node->buf_info[buf_type];

			/* check userptr buffer type. */
			if (cmdlist->data[index] & G2D_BUF_USERPTR) {
				buf_info->is_userptr = true;
				cmdlist->data[index] &= ~G2D_BUF_USERPTR;
			}

			/*
			 * Store pointer to the current command here, so we can patch
			 * the buffer handle later in g2d_map_cmdlist_buffers().
			 */
			buf_info->data = &cmdlist->data[index];
			break;

		case G2D_SRC_STRIDE:
		case G2D_DST_STRIDE:
		case G2D_PAT_STRIDE:
		case G2D_MSK_STRIDE:
			buf_type = g2d_get_buf_type(g2d, reg_offset);
			buf_info = &node->buf_info[buf_type];

			/*
			 * The hardware allows for signed strides for certain
			 * format, but we currently lack the infrastructure
			 * to properly handle this case.
			 */
			if (cmdlist->data[index + 1] & BIT(15))
				goto err;

			buf_info->stride = cmdlist->data[index + 1] & 0x7fff;
			break;

		case G2D_SRC_COLOR_MODE:
		case G2D_DST_COLOR_MODE:
			buf_type = g2d_get_buf_type(g2d, reg_offset);
			buf_info = &node->buf_info[buf_type];

			buf_info->format = cmdlist->data[index + 1];
			fmt = buf_info->format & G2D_FMT_MASK;

			/*
			 * The color type part of the format field is 4 bits wide, allowing for
			 * a total of 16 entries. But apparantly not all entries are valid.
			 * Some sources however suggest that b1101 and b1110 map to a 1-bit and
			 * a 4-bit format format respectively, but tests show that the engine
			 * just hangs when using these.
			 * Make sure that we only pass valid entries to the engine.
			 */
			if (fmt >= G2D_FMT_MAX)
				goto err;

			buf_info->bpp = g2d_get_buf_bpp(fmt);
			buf_info->is_ycbcr = g2d_is_ycbcr_fmt(fmt);
			break;

		case G2D_PAT_COLOR_MODE:
			buf_info = &node->buf_info[BUF_TYPE_PAT];

			buf_info->format = cmdlist->data[index + 1];
			fmt = buf_info->format & G2D_FMT_MASK;

			if (fmt >= G2D_FMT_MAX)
				goto err;

			if (g2d_is_ycbcr_fmt(fmt) || g2d_is_lumi_alpha_fmt(fmt))
				goto err;

			buf_info->bpp = g2d_get_buf_bpp(fmt);
			break;

		case G2D_MSK_MODE:
			buf_info = &node->buf_info[BUF_TYPE_MSK];

			buf_info->format = cmdlist->data[index + 1];
			fmt = buf_info->format & G2D_MSK_MASK;

			if (fmt >= G2D_MSK_MAX)
				goto err;

			buf_info->bpp = g2d_get_msk_bpp(fmt);
			break;

		default:
			goto err;
			break;
		}
	}

	return 0;

err:
	dev_err(g2d->dev, "invalid base command: 0x%x\n", reg_offset);
	return -EINVAL;
}

static int g2d_validate_bitblt_start(unsigned long value,
				const struct g2d_cmdlist_node *node)
{
	const struct g2d_bitblt_info* bitblt = &node->bitblt_info;
	const struct g2d_buf_info* buf = node->buf_info;

	unsigned int rop3;
	bool need_mask, need_pattern;

	if (!(value & G2D_START_BITBLT))
		goto fail;

	/* We need a valid destination rectangle for _every_ blit operation. */
	if (!g2d_is_rect_valid(&bitblt->rects[RECT_TYPE_DST], &buf[BUF_TYPE_DST]))
		goto fail;

	/* If we're doing a solid fill operation, all other BitBLT parameters are ignored. */
	if (bitblt->bitblt_command & G2D_BITBLT_SOLID_FILL)
		return 0;

	/* TODO: Clipping window functionality is not implemented yet. */
	if (bitblt->bitblt_command & G2D_BITBLT_ENABLE_CW)
		goto fail;

	/* TODO: Rotation is not implemented yet. */
	if (bitblt->rotate)
		goto fail;

	/* If the source uses a buffer in memory, then we need a valid source rectangle. */
	if (!(bitblt->src_select & 0x3) &&
	    !(g2d_is_rect_valid(&bitblt->rects[RECT_TYPE_SRC], &buf[BUF_TYPE_SRC])))
		goto fail;

	need_mask = false;
	need_pattern = false;

	/*
	 * If the unmasked ROP4 is using the third operand register, and the
	 * third operand uses the pattern, then we need a valid pattern rectangle.
	 */
	rop3 = bitblt->rop4 & G2D_ROP4_UNMASKED_ROP3;
	if ((rop3 & 0xf0) && !(bitblt->third_operand & G2D_THIRD_OP_MASK_UNMASKED))
		need_pattern = true;

	/* If the bitblt command uses a mask we need a valid mask rectangle. */
	if (bitblt->bitblt_command & G2D_BITBLT_MASK_NORMAL)
		need_mask = true;

	/* Check for third operand / pattern in masked ROP4s. */
	if (bitblt->bitblt_command & G2D_BITBLT_MASK_ROP4) {
		need_mask = true;

		/* Do the same check as above, only for masked ROP4 + third operand. */
		rop3 = (bitblt->rop4 & G2D_ROP4_MASKED_ROP3) >> G2D_ROP4_MASKED_SHIFT;
		if ((rop3 & 0xf0) && !(bitblt->third_operand & G2D_THIRD_OP_MASK_MASKED))
			need_pattern = true;
	}

	if (need_mask && !g2d_is_rect_valid(&bitblt->rects[RECT_TYPE_MSK], &buf[BUF_TYPE_MSK]))
		goto fail;

	if (need_pattern && !g2d_is_pattern_valid(bitblt, &buf[BUF_TYPE_PAT]))
		goto fail;

	return 0;

fail:
	return -EINVAL;
}

static int g2d_validate_cmds(struct g2d_data *g2d,
				struct g2d_cmdlist_node *node,
				unsigned int nr)
{
	struct g2d_cmdlist *cmdlist = node->cmdlist;
	unsigned int reg_offset = 0;
	u32 index;

	for (index = cmdlist->last - 2 * nr; index < cmdlist->last; index += 2) {
		enum g2d_rect_type rect_type;
		struct g2d_rect *rect;

		reg_offset = cmdlist->data[index] & 0x0fff;

		if (unlikely(reg_offset < G2D_VALID_START ||
				reg_offset > G2D_VALID_END))
			goto err;
		if (unlikely(reg_offset & 0x3))
			goto err;

		reg_offset >>= 2;

		switch (reg_offset) {
		/*
		 * Coordinate registers.
		 * These regs define the rectangles for src/dst/msk/cw.
		 */
		case G2D_SRC_LEFT_TOP:
		case G2D_SRC_RIGHT_BOTTOM:
		case G2D_DST_LEFT_TOP:
		case G2D_DST_RIGHT_BOTTOM:
		case G2D_MSK_LEFT_TOP:
		case G2D_MSK_RIGHT_BOTTOM:
		case G2D_CW_LEFT_TOP:
		case G2D_CW_RIGHT_BOTTOM:
			rect_type = g2d_get_rect_type(reg_offset);
			rect = &node->bitblt_info.rects[rect_type];

			if (g2d_is_left_top(reg_offset))
				rect->left_top = cmdlist->data[index + 1];
			else
				rect->right_bottom = cmdlist->data[index + 1];
			break;

		/*
		 * BitBLT start register.
		 * Marks the end of the current BitBLT unit.
		 */
		case G2D_BITBLT_START:
			if (g2d_validate_bitblt_start(cmdlist->data[index + 1], node) < 0)
				goto err;
			break;

		/*
		 * Vital registers.
		 * These need to be stored to validate the BitBLT unit.
		 */
		case G2D_BITBLT_COMMAND:
			node->bitblt_info.bitblt_command = cmdlist->data[index + 1];
			break;
		case G2D_ROTATE:
			node->bitblt_info.rotate = cmdlist->data[index + 1];
			break;
		case G2D_SRC_SELECT:
			node->bitblt_info.src_select = cmdlist->data[index + 1];
			break;
		case G2D_PAT_SIZE:
			node->bitblt_info.pat_size = cmdlist->data[index + 1];
			break;
		case G2D_PAT_OFFSET:
			node->bitblt_info.pat_offset = cmdlist->data[index + 1];
			break;
		case G2D_THIRD_OPERAND:
			node->bitblt_info.third_operand = cmdlist->data[index + 1];
			break;
		case G2D_ROP4:
			node->bitblt_info.rop4 = cmdlist->data[index + 1];
			break;

		/*
		 * These registers don't need any specific validation.
		 */
		case G2D_BLEND_FUNCTION:
		case G2D_ROUND_MODE:
		case G2D_SRC_MASK_DIRECT:
		case G2D_DST_PAT_DIRECT:
		case G2D_SRC_REPEAT_MODE:
		case G2D_SRC_PAD_VALUE:
		case G2D_SRC_A8_RGB_EXT:
		case G2D_SRC_SCALE_CTRL:
		case G2D_SRC_XSCALE:
		case G2D_SRC_YSCALE:
		case G2D_DST_SELECT:
		case G2D_DST_A8_RGB_EXT:
		case G2D_MSK_REPEAT_MODE:
		case G2D_MSK_PAD_VALUE:
		case G2D_MSK_SCALE_CTRL:
		case G2D_MSK_XSCALE:
		case G2D_MSK_YSCALE:
		case G2D_ALPHA:
		case G2D_FG_COLOR:
		case G2D_BG_COLOR:
		case G2D_BS_COLOR:
		case G2D_SF_COLOR:
			break;

		default:
			goto err;
			break;
		}
	}

	/*
	 * The regular commands have to end with a bitblt start.
	 */
	if (reg_offset != G2D_BITBLT_START)
		goto err;

	return 0;

err:
	dev_err(g2d->dev, "invalid command: 0x%x\n", reg_offset);
	return -EINVAL;
}

static int g2d_validate_ycbcr(struct g2d_data *g2d,
				struct g2d_cmdlist_node *node)
{
	unsigned int i;

	for (i = 0; i < 2; ++i) {
		struct g2d_buf_info *buf_info, *plane2_info;
		unsigned int smask;
		bool need_plane2;

		buf_info = &node->buf_info[i * 2];

		if (!buf_info->data)
			continue;

		if (!buf_info->is_ycbcr)
			continue;

		/*
		 * Depending on the YCbCr format we have an additional stride
		 * restriction. Also only the 422 format supports both
		 * uniplanar and biplanar mode.
		 */
		switch (buf_info->format & G2D_FMT_MASK) {
		case G2D_FMT_YCbCr444:
			smask = 0;
			need_plane2 = true;
			break;
		case G2D_FMT_YCbCr422:
			smask = 1;
			need_plane2 = false;
			break;
		case G2D_FMT_YCbCr420:
		default:
			smask = 1;
			need_plane2 = true;
		}

		if (buf_info->stride & smask)
			goto err;

		if (need_plane2 && !(buf_info->format & G2D_FMT_YCbCr_2PLANE))
			goto err;

		if (buf_info->format & G2D_FMT_YCbCr_2PLANE) {
			plane2_info = &node->buf_info[i * 2 + 1];

			if (!plane2_info->data)
				goto err;

			plane2_info->stride = buf_info->stride * 2;
		}
	}

	return 0;

err:
	dev_err(g2d->dev, "invalid YCbCr configuration\n");
	return -EINVAL;
}

static void g2d_cmdlist_prolog(struct g2d_cmdlist *cmdlist, bool event)

{
	cmdlist->last = 0;

	/*
	 * If don't clear SFR registers, the cmdlist is affected by register
	 * values of previous cmdlist. G2D hw executes SFR clear command and
	 * a next command at the same time then the next command is ignored and
	 * is executed rightly from next next command, so needs a dummy command
	 * to next command of SFR clear command.
	 */
	g2d_add_cmd(cmdlist, G2D_SOFT_RESET, G2D_SFRCLEAR);
	g2d_add_cmd(cmdlist, G2D_SRC_BASE_ADDR, 0);

	/*
	 * 'LIST_HOLD' command should be set to the DMA_HOLD_CMD_REG
	 * and GCF bit should be set to INTEN register if user wants
	 * G2D interrupt event once current command list execution is
	 * finished.
	 * Otherwise only ACF bit should be set to INTEN register so
	 * that one interrupt is occurred after all command lists
	 * have been completed.
	 */
	if (event) {
		g2d_add_cmd(cmdlist, G2D_INTEN, G2D_INTEN_ACF | G2D_INTEN_GCF);
		g2d_add_cmd(cmdlist, G2D_DMA_HOLD_CMD, G2D_LIST_HOLD);
	} else
		g2d_add_cmd(cmdlist, G2D_INTEN, G2D_INTEN_ACF);
}

/* ioctl functions */
int exynos_g2d_get_ver2_ioctl(struct drm_device *drm_dev, void *data,
			     struct drm_file *file)
{
	struct drm_exynos_g2d_get_ver2 *ver = data;

	ver->major = G2D_HW_MAJOR_VER;
	ver->minor = G2D_HW_MINOR_VER;
	ver->caps = 0;

	/*
	 * We can only properly support userptr functionality when
	 * the IOMMU is available.
	 */
	if (is_drm_iommu_supported(drm_dev))
		ver->caps |= G2D_CAP_USERPTR;

	return 0;
}

int exynos_g2d_exec_ioctl(struct drm_device *drm_dev, void *data,
			  struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct exynos_drm_private *priv = drm_dev->dev_private;
	struct g2d_data *g2d = dev_get_drvdata(priv->g2d_dev);
	struct drm_exynos_g2d_exec *req = data;
	struct g2d_runqueue_node *runqueue_node;
	struct list_head *run_cmdlist;
	struct list_head *event_list;

	runqueue_node = kmem_cache_alloc(g2d->runqueue_slab, GFP_KERNEL);
	if (unlikely(!runqueue_node))
		return -ENOMEM;

	run_cmdlist = &runqueue_node->run_cmdlist;
	event_list = &runqueue_node->event_list;
	INIT_LIST_HEAD(run_cmdlist);
	INIT_LIST_HEAD(event_list);
	init_completion(&runqueue_node->complete);
	runqueue_node->async = req->async;

	list_splice_init(&file_priv->inuse_cmdlist, run_cmdlist);
	list_splice_init(&file_priv->event_list, event_list);

	if (unlikely(list_empty(run_cmdlist))) {
		dev_err(g2d->dev, "there is no inuse cmdlist\n");
		kmem_cache_free(g2d->runqueue_slab, runqueue_node);
		return -EPERM;
	}

	mutex_lock(&g2d->runqueue_mutex);
	runqueue_node->pid = current->pid;
	runqueue_node->filp = file;
	list_add_tail(&runqueue_node->list, &g2d->runqueue);
	mutex_unlock(&g2d->runqueue_mutex);

	/* Let the runqueue know that there is work to do. */
	queue_work(g2d->g2d_workq, &g2d->runqueue_work);

	if (runqueue_node->async)
		goto out;

	wait_for_completion(&runqueue_node->complete);
	g2d_free_runqueue_node(g2d, runqueue_node);

out:
	return 0;
}

int exynos_g2d_userptr_ioctl(struct drm_device *drm_dev, void *data,
			  struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct exynos_drm_private *priv = drm_dev->dev_private;
	struct g2d_data *g2d = dev_get_drvdata(priv->g2d_dev);

	struct drm_exynos_g2d_userptr_op *userptr_op = data;
	struct g2d_cmdlist_userptr *userptr;
	int ret;

	mutex_lock(&g2d->userptr_mutex);

	switch (userptr_op->operation) {
	case G2D_USERPTR_REGISTER:
		ret = g2d_userptr_register(g2d, userptr_op->user_addr,
			userptr_op->size, userptr_op->flags, file);
		break;

	case G2D_USERPTR_UNREGISTER:
		userptr = g2d_userptr_lookup(file_priv, userptr_op->user_addr);
		if (userptr) {
			ret = g2d_userptr_unregister(g2d, userptr, false);
		} else {
			DRM_ERROR("userptr %llx not registered.\n", userptr_op->user_addr);
			ret = -EINVAL;
		}
		break;

	case G2D_USERPTR_CHECK_IDLE:
		ret = g2d_userptr_check_idle(file_priv, userptr_op->user_addr);
		break;

	default:
		ret = -EFAULT;
		break;
	}

	mutex_unlock(&g2d->userptr_mutex);
	return ret;
}

int g2d_open(struct drm_device *drm_dev, struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;

	INIT_LIST_HEAD(&file_priv->inuse_cmdlist);
	INIT_LIST_HEAD(&file_priv->event_list);
	INIT_LIST_HEAD(&file_priv->userptr_list);

	return 0;
}

void g2d_close(struct drm_device *drm_dev, struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv = file->driver_priv;
	struct exynos_drm_private *priv = drm_dev->dev_private;
	struct g2d_data *g2d;
	struct g2d_cmdlist_node *node, *n;

	if (!priv->g2d_dev)
		return;

	g2d = dev_get_drvdata(priv->g2d_dev);

	/* Remove the runqueue nodes that belong to us. */
	mutex_lock(&g2d->runqueue_mutex);
	g2d_remove_runqueue_nodes(g2d, file);
	mutex_unlock(&g2d->runqueue_mutex);

	/*
	 * Wait for the runqueue worker to finish its current node.
	 * After this the engine should no longer be accessing any
	 * memory belonging to us.
	 */
	g2d_wait_finish(g2d, file);

	/*
	 * Even after the engine is idle, there might still be stale cmdlists
	 * (i.e. cmdlisst which we submitted but never executed) around, with
	 * their corresponding GEM/userptr buffers.
	 * Properly unmap these buffers here.
	 */
	mutex_lock(&g2d->cmdlist_mutex);
	list_for_each_entry_safe(node, n, &file_priv->inuse_cmdlist, list) {
		list_move_tail(&node->list, &g2d->free_cmdlist);
	}
	mutex_unlock(&g2d->cmdlist_mutex);

	/* release all g2d_userptr in pool. */
	g2d_userptr_free_all(g2d, file);
}

static int g2d_bind(struct device *dev, struct device *master, void *data)
{
	struct g2d_data *g2d = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_private *priv = drm_dev->dev_private;
	int ret;

	g2d->drm_dev = drm_dev;

	/* allocate dma-aware cmdlist buffer. */
	ret = g2d_init_cmdlist(g2d);
	if (ret < 0) {
		dev_err(dev, "cmdlist init failed\n");
		return ret;
	}

	ret = exynos_drm_register_dma(drm_dev, dev, &g2d->dma_priv);
	if (ret < 0) {
		dev_err(dev, "failed to enable iommu.\n");
		g2d_fini_cmdlist(g2d);
		return ret;
	}
	priv->g2d_dev = dev;

	dev_info(dev, "The Exynos G2D (ver %d.%d) successfully registered.\n",
			G2D_HW_MAJOR_VER, G2D_HW_MINOR_VER);
	return 0;
}

static void g2d_unbind(struct device *dev, struct device *master, void *data)
{
	struct g2d_data *g2d = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_private *priv = drm_dev->dev_private;

	/* Suspend operation and wait for engine idle. */
	set_bit(G2D_BIT_SUSPEND_RUNQUEUE, &g2d->flags);
	g2d_wait_finish(g2d, NULL);
	priv->g2d_dev = NULL;

	cancel_work_sync(&g2d->runqueue_work);
	exynos_drm_unregister_dma(g2d->drm_dev, dev, &g2d->dma_priv);
}

static const struct component_ops g2d_component_ops = {
	.bind	= g2d_bind,
	.unbind = g2d_unbind,
};

static int g2d_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct g2d_data *g2d;
	int ret;

	g2d = devm_kzalloc(dev, sizeof(*g2d), GFP_KERNEL);
	if (!g2d)
		return -ENOMEM;

	g2d->runqueue_slab = kmem_cache_create("g2d_runqueue_slab",
			sizeof(struct g2d_runqueue_node), 0, 0, NULL);
	if (!g2d->runqueue_slab)
		return -ENOMEM;

	g2d->dev = dev;

	g2d->g2d_workq = create_singlethread_workqueue("g2d");
	if (!g2d->g2d_workq) {
		dev_err(dev, "failed to create workqueue\n");
		ret = -EINVAL;
		goto err_destroy_slab;
	}

	INIT_WORK(&g2d->runqueue_work, g2d_runqueue_worker);
	INIT_LIST_HEAD(&g2d->free_cmdlist);
	INIT_LIST_HEAD(&g2d->runqueue);

	mutex_init(&g2d->cmdlist_mutex);
	mutex_init(&g2d->runqueue_mutex);
	mutex_init(&g2d->userptr_mutex);

	g2d->gate_clk = devm_clk_get(dev, "fimg2d");
	if (IS_ERR(g2d->gate_clk)) {
		dev_err(dev, "failed to get gate clock\n");
		ret = PTR_ERR(g2d->gate_clk);
		goto err_destroy_workqueue;
	}

	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_enable(dev);
	clear_bit(G2D_BIT_SUSPEND_RUNQUEUE, &g2d->flags);
	clear_bit(G2D_BIT_ENGINE_BUSY, &g2d->flags);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	g2d->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(g2d->regs)) {
		ret = PTR_ERR(g2d->regs);
		goto err_put_clk;
	}

	g2d->irq = platform_get_irq(pdev, 0);
	if (g2d->irq < 0) {
		ret = g2d->irq;
		goto err_put_clk;
	}

	ret = devm_request_irq(dev, g2d->irq, g2d_irq_handler, 0,
								"drm_g2d", g2d);
	if (ret < 0) {
		dev_err(dev, "irq request failed\n");
		goto err_put_clk;
	}

	platform_set_drvdata(pdev, g2d);

	ret = component_add(dev, &g2d_component_ops);
	if (ret < 0) {
		dev_err(dev, "failed to register drm g2d device\n");
		goto err_put_clk;
	}

	return 0;

err_put_clk:
	pm_runtime_disable(dev);
err_destroy_workqueue:
	destroy_workqueue(g2d->g2d_workq);
err_destroy_slab:
	kmem_cache_destroy(g2d->runqueue_slab);
	return ret;
}

static int g2d_remove(struct platform_device *pdev)
{
	struct g2d_data *g2d = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &g2d_component_ops);

	/* There should be no locking needed here. */
	g2d_remove_runqueue_nodes(g2d, NULL);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	g2d_fini_cmdlist(g2d);
	destroy_workqueue(g2d->g2d_workq);
	kmem_cache_destroy(g2d->runqueue_slab);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int g2d_suspend(struct device *dev)
{
	struct g2d_data *g2d = dev_get_drvdata(dev);

	/*
	 * Suspend the runqueue worker operation and wait until the G2D
	 * engine is idle.
	 */
	set_bit(G2D_BIT_SUSPEND_RUNQUEUE, &g2d->flags);
	g2d_wait_finish(g2d, NULL);
	flush_work(&g2d->runqueue_work);

	return 0;
}

static int g2d_resume(struct device *dev)
{
	struct g2d_data *g2d = dev_get_drvdata(dev);

	clear_bit(G2D_BIT_SUSPEND_RUNQUEUE, &g2d->flags);
	queue_work(g2d->g2d_workq, &g2d->runqueue_work);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int g2d_runtime_suspend(struct device *dev)
{
	struct g2d_data *g2d = dev_get_drvdata(dev);

	clk_disable_unprepare(g2d->gate_clk);

	return 0;
}

static int g2d_runtime_resume(struct device *dev)
{
	struct g2d_data *g2d = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(g2d->gate_clk);
	if (unlikely(ret < 0))
		dev_warn(dev, "failed to enable clock.\n");

	return ret;
}
#endif

static const struct dev_pm_ops g2d_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(g2d_suspend, g2d_resume)
	SET_RUNTIME_PM_OPS(g2d_runtime_suspend, g2d_runtime_resume, NULL)
};

static const struct of_device_id exynos_g2d_match[] = {
	{ .compatible = "samsung,exynos5250-g2d" },
	{ .compatible = "samsung,exynos4212-g2d" },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_g2d_match);

struct platform_driver g2d_driver = {
	.probe		= g2d_probe,
	.remove		= g2d_remove,
	.driver		= {
		.name	= "exynos-drm-g2d",
		.owner	= THIS_MODULE,
		.pm	= &g2d_pm_ops,
		.of_match_table = exynos_g2d_match,
	},
};
