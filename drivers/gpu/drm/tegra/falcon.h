/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015, NVIDIA Corporation.
 */

#ifndef _FALCON_H_
#define _FALCON_H_

#include <linux/align.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/wordpart.h>

#define FALCON_UCLASS_METHOD_OFFSET		0x00000040

#define FALCON_UCLASS_METHOD_DATA		0x00000044

#define FALCON_IRQMSET				0x00001010
#define FALCON_IRQMSET_WDTMR			(1 << 1)
#define FALCON_IRQMSET_HALT			(1 << 4)
#define FALCON_IRQMSET_EXTERR			(1 << 5)
#define FALCON_IRQMSET_SWGEN0			(1 << 6)
#define FALCON_IRQMSET_SWGEN1			(1 << 7)
#define FALCON_IRQMSET_EXT(v)			(((v) & 0xff) << 8)

#define FALCON_IRQDEST				0x0000101c
#define FALCON_IRQDEST_HALT			(1 << 4)
#define FALCON_IRQDEST_EXTERR			(1 << 5)
#define FALCON_IRQDEST_SWGEN0			(1 << 6)
#define FALCON_IRQDEST_SWGEN1			(1 << 7)
#define FALCON_IRQDEST_EXT(v)			(((v) & 0xff) << 8)

#define FALCON_ITFEN				0x00001048
#define FALCON_ITFEN_CTXEN			(1 << 0)
#define FALCON_ITFEN_MTHDEN			(1 << 1)

#define FALCON_IDLESTATE			0x0000104c

#define FALCON_CPUCTL				0x00001100
#define FALCON_CPUCTL_STARTCPU			(1 << 1)

#define FALCON_BOOTVEC				0x00001104

#define FALCON_DMACTL				0x0000110c
#define FALCON_DMACTL_DMEM_SCRUBBING		(1 << 1)
#define FALCON_DMACTL_IMEM_SCRUBBING		(1 << 2)

#define FALCON_DMATRFBASE			0x00001110

#define FALCON_DMATRFMOFFS			0x00001114

#define FALCON_DMATRFCMD			0x00001118
#define FALCON_DMATRFCMD_FULL			(1 << 0)
#define FALCON_DMATRFCMD_IDLE			(1 << 1)
#define FALCON_DMATRFCMD_IMEM			(1 << 4)
#define FALCON_DMATRFCMD_SIZE_256B		(6 << 8)
#define FALCON_DMATRFCMD_DMACTX(v)		(((v) & 0x7) << 12)

#define FALCON_DMATRFFBOFFS			0x0000111c

#define RISCV_BOOT_VECTOR_LO			0x00001780
#define RISCV_BOOT_VECTOR_HI			0x00001784

#define RISCV_CPUCTL				0x00001788
#define RISCV_CPUCTL_STARTCPU			(1 << 0)
#define RISCV_CPUCTL_ACTIVE_STAT_ACTIVE		(1 << 7)

#define RISCV_BCR_CTRL				0x00001a68
#define RISCV_BCR_CTRL_CORE_SELECT_RISCV	(1 << 4)

struct falcon_fw_bin_header_v1 {
	u32 magic;		/* 0x10de */
	u32 version;		/* version of bin format (1) */
	u32 size;		/* entire image size including this header */
	u32 os_header_offset;
	u32 os_data_offset;
	u32 os_size;
};

struct falcon_fw_os_app_v1 {
	u32 offset;
	u32 size;
};

struct falcon_fw_os_header_v1 {
	u32 code_offset;
	u32 code_size;
	u32 data_offset;
	u32 data_size;
};

struct falcon_fw_riscv_desc {
	u32 reserved[74];
	u32 data_offset;
	u32 data_size;
	u32 code_offset;
	u32 code_size;
};

struct falcon_firmware_section {
	unsigned long offset;
	size_t size;
};

struct falcon_firmware {
	/* Firmware after it is read but not loaded */
	const struct firmware *firmware;
	/* RISC-V firmware descriptor */
	const struct firmware *desc_firmware;

	/* Raw firmware data */
	dma_addr_t iova;
	dma_addr_t phys;
	void *virt;
	size_t size;

	/* Parsed firmware information */
	struct falcon_firmware_section bin_data;
	struct falcon_firmware_section data;
	struct falcon_firmware_section code;
};

struct falcon {
	/* Set by falcon client */
	struct device *dev;
	void __iomem *regs;

	/* Peregrine falcon, external boot */
	bool riscv;

	struct falcon_firmware firmware;
};

int falcon_init(struct falcon *falcon);
void falcon_exit(struct falcon *falcon);
int falcon_read_firmware(struct falcon *falcon, const char *firmware_name);
int falcon_load_firmware(struct falcon *falcon);
int falcon_boot(struct falcon *falcon);
void falcon_execute_method(struct falcon *falcon, u32 method, u32 data);
int falcon_wait_idle(struct falcon *falcon);

/* A host1x command buffer of falcon methods, built by the kernel. */
struct falcon_gather {
	u32 *words;
	unsigned int count;
	unsigned int size;
	int err;
};

/* One host1x INCR writing METHOD_OFFSET and METHOD_DATA. */
#define FALCON_GATHER_METHOD	(0x10000000 | \
				 (FALCON_UCLASS_METHOD_OFFSET >> 2) << 16 | 2)

static inline void falcon_gather_method(struct falcon_gather *g, u32 method,
					u32 value)
{
	if (g->count + 3 > g->size) {
		g->err = -ENOSPC;
		return;
	}

	g->words[g->count++] = FALCON_GATHER_METHOD;
	g->words[g->count++] = method;
	g->words[g->count++] = value;
}

/* Address methods carry the IOVA shifted right by 8. */
static inline void falcon_gather_address(struct falcon_gather *g, u32 method,
					 dma_addr_t iova)
{
	if (!IS_ALIGNED(iova, SZ_256) || upper_32_bits(iova >> 8))
		g->err = -EINVAL;

	falcon_gather_method(g, method, lower_32_bits(iova >> 8));
}

/* A host1x NONINCR to INCR_SYNCPT, incrementing @syncpt on OP_DONE. */
static inline void falcon_gather_op_done(struct falcon_gather *g, u32 syncpt)
{
	if (g->count + 2 > g->size) {
		g->err = -ENOSPC;
		return;
	}

	g->words[g->count++] = 0x20000001;
	g->words[g->count++] = syncpt | BIT(8);
}

#endif /* _FALCON_H_ */
