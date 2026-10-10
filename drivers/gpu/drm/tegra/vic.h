/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015, NVIDIA Corporation.
 */

#ifndef TEGRA_VIC_H
#define TEGRA_VIC_H

#include <linux/types.h>

/* VIC methods */

#define VIC_SET_FCE_UCODE_SIZE			0x0000071C
#define VIC_SET_FCE_UCODE_OFFSET		0x0000072C

/* VIC registers */

#define VIC_THI_STREAMID0	0x00000030
#define VIC_THI_STREAMID1	0x00000034

#define NV_PVIC_MISC_PRI_VIC_CG			0x000016d0
#define CG_IDLE_CG_DLY_CNT(val)			((val & 0x3f) << 0)
#define CG_IDLE_CG_EN				(1 << 6)
#define CG_WAKEUP_DLY_CNT(val)			((val & 0xf) << 16)

#define TRANSCFG_ATT(i, v)	(((v) & 0x3) << ((i) * 4))
#define TRANSCFG_SID_HW		0
#define TRANSCFG_SID_PHY	1
#define TRANSCFG_SID_FALCON	2

/* Firmware offsets */

#define VIC_UCODE_FCE_HEADER_OFFSET		(6*4)
#define VIC_UCODE_FCE_DATA_OFFSET		(7*4)
#define FCE_UCODE_SIZE_OFFSET			(2*4)

/* In-kernel detiling: block-linear 4:2:0 in, cropped pitch-linear out. */

struct falcon_gather;
struct tegra_drm;
struct tegra_drm_client;

#define VIC_CONFIG_SIZE		0x610
/* FilterStruct, 520 coefficient sets; VIC reads it on every execute. */
#define VIC_FILTER_SIZE		0x2080

/* P010 has no VIC pixel format and takes one pass per plane instead. */
enum vic_detile_pass {
	VIC_DETILE_NV12,
	VIC_DETILE_P010_LUMA,
	VIC_DETILE_P010_CHROMA,
};

struct vic_detile {
	u32 width;		/* block-linear source, coded size */
	u32 height;
	u32 left;		/* visible rectangle inside it */
	u32 top;
	u32 out_width;		/* pitch-linear destination, == visible size */
	u32 out_height;
	u32 src_stride;		/* bytes */
	u32 dst_stride;		/* bytes */
	enum vic_detile_pass pass;
};

struct tegra_drm_client *vic_find_detiler(struct tegra_drm *tegra);
void vic_fill_detile_config(void *config, const struct vic_detile *params);
void vic_emit_detile(struct falcon_gather *g, dma_addr_t config,
		     dma_addr_t filter, dma_addr_t src_luma, dma_addr_t src_chroma,
		     dma_addr_t dst_luma, dma_addr_t dst_chroma);

#endif /* TEGRA_VIC_H */
