// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 mojyack <mojyack@gmail.com>
 */
#include "priv.h"

#include <core/memory.h>
#include <subdev/fb.h>
#include <subdev/timer.h>

#include <linux/mm.h>

static void
gm20b_ltc_init_num_active(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;

	nvkm_wr32(device, 0x17e27c, ltc->ltc_nr);
	nvkm_wr32(device, 0x17e000, ltc->ltc_nr);
	nvkm_wr32(device, 0x100800, ltc->ltc_nr);
}

/* No VRAM: the compression bit cache goes to system memory instead. */
static int
gm20b_ltc_oneinit(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;
	struct nvkm_fb *fb = device->fb;
	u32 param, tags_per_line, line_size, align, size;
	u64 tags;
	int ret;

	/* CBC_PARAM describes a single slice until the count is programmed. */
	ltc->ltc_nr = nvkm_rd32(device, 0x12006c);
	gm20b_ltc_init_num_active(ltc);

	param = nvkm_rd32(device, 0x17e280);
	ltc->lts_nr = param >> 28;
	tags_per_line = param & 0x0000ffff;
	line_size = 512 << ((param >> 24) & 0x0000000f);

	if (!ltc->ltc_nr || !ltc->lts_nr || !tags_per_line) {
		nvkm_error(&ltc->subdev, "bad CBC config %08x\n", param);
		return -EINVAL;
	}

	/* One tag line per 128KiB of surface, enough for all of RAM. */
	tags = ((u64)totalram_pages() << PAGE_SHIFT) >> 17;
	ltc->num_tags = min_t(u64, ALIGN(tags, 64), 0xffff);

	align = ltc->ltc_nr << 11;
	size = DIV_ROUND_UP(ltc->num_tags, tags_per_line) *
	       line_size * ltc->lts_nr * ltc->ltc_nr;
	size = ALIGN(size, SZ_64K);

	ret = nvkm_memory_new(device, NVKM_MEM_TARGET_INST, size, align, true,
			      &ltc->tag_ram);
	if (ret) {
		nvkm_warn(&ltc->subdev, "no CBC backing store: %d\n", ret);
		ltc->num_tags = 0;
	} else {
		/* Not divided by ltc_nr: GM20B wants the base in 2KiB units. */
		ltc->tag_base = nvkm_memory_addr(ltc->tag_ram) >> 11;
		nvkm_debug(&ltc->subdev, "CBC %d tags, %d bytes at %llx\n",
			   ltc->num_tags, size,
			   nvkm_memory_addr(ltc->tag_ram));
	}

	nvkm_mm_fini(&fb->tags.mm);
	return nvkm_mm_init(&fb->tags.mm, 0, 0, ltc->num_tags, 1);
}

static void
gm20b_ltc_init(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;
	int c, s;

	gm20b_ltc_init_num_active(ltc);
	nvkm_wr32(device, 0x17e278, ltc->tag_base);
	nvkm_wr32(device, 0x17e318, nvkm_rd32(device, 0x140518) | 0x00008000);

	if (!ltc->num_tags)
		return;

	/* Nothing the CBC holds yet describes the backing store. */
	nvkm_mask(device, 0x17e26c, 0x00000000, 0x00000002);
	for (c = 0; c < ltc->ltc_nr; c++) {
		for (s = 0; s < ltc->lts_nr; s++) {
			const u32 addr = 0x14046c + (c * 0x2000) + (s * 0x200);

			nvkm_wait_msec(device, 2000, addr,
				       0x00000002, 0x00000000);
		}
	}
}

/* The GPU is about to lose power, and the L2 and the CBC with it. */
static void
gm20b_ltc_suspend(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;

	if (ltc->num_tags) {
		mutex_lock(&ltc->mutex);
		nvkm_wr32(device, 0x7000c, 0x00000001);
		nvkm_wait_msec(device, 2000, 0x7000c, 0x00000003, 0x00000000);
		mutex_unlock(&ltc->mutex);
	}

	nvkm_ltc_flush(ltc);
}

static const struct nvkm_ltc_func
gm20b_ltc = {
	.oneinit = gm20b_ltc_oneinit,
	.init = gm20b_ltc_init,
	.suspend = gm20b_ltc_suspend,
	.intr = gm107_ltc_intr,
	.cbc_clear = gm107_ltc_cbc_clear,
	.cbc_wait = gm107_ltc_cbc_wait,
	.zbc_color = 16,
	.zbc_depth = 16,
	.zbc_clear_color = gm107_ltc_zbc_clear_color,
	.zbc_clear_depth = gm107_ltc_zbc_clear_depth,
	.invalidate = gf100_ltc_invalidate,
	.flush = gf100_ltc_flush,
};

int
gm20b_ltc_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_ltc **pltc)
{
	return nvkm_ltc_new_(&gm20b_ltc, device, type, inst, pltc);
}
