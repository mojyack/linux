/*
 * Copyright (c) 2026 NVIDIA Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#include "priv.h"

#include <core/memory.h>
#include <subdev/fb.h>

#include <linux/mm.h>

/* No VRAM: the compression bit cache goes to system memory instead. */
static int
gm20b_ltc_oneinit(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;
	struct nvkm_fb *fb = device->fb;
	u32 param, tags_per_line, line_size, align, size;
	u64 tags;
	int ret;

	ltc->ltc_nr = nvkm_rd32(device, 0x12006c);
	param = nvkm_rd32(device, 0x17e280);
	ltc->lts_nr = param >> 28;
	tags_per_line = param & 0x0000ffff;
	line_size = 512 << ((param >> 24) & 0x0000000f);

	if (!ltc->ltc_nr || !ltc->lts_nr || !tags_per_line) {
		nvkm_error(&ltc->subdev, "bad CBC config %08x\n", param);
		return -EINVAL;
	}

	/* One tag line per 128 KiB of surface, enough for all of RAM. */
	tags = min_t(u64, ((u64)totalram_pages() << PAGE_SHIFT) >> 17, 0xffff);
	ltc->num_tags = ALIGN(tags, 64);

	align = ltc->ltc_nr << 11;
	size = DIV_ROUND_UP(ltc->num_tags, tags_per_line) *
	       line_size * ltc->lts_nr * ltc->ltc_nr;
	size = ALIGN(size, SZ_64K);

	ret = nvkm_memory_new(device, NVKM_MEM_TARGET_INST, size, align, false,
			      &ltc->tag_ram);
	if (ret) {
		nvkm_warn(&ltc->subdev, "no CBC backing store: %d\n", ret);
		ltc->num_tags = 0;
	} else {
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

	nvkm_wr32(device, 0x17e27c, ltc->ltc_nr);
	nvkm_wr32(device, 0x17e000, ltc->ltc_nr);
	nvkm_wr32(device, 0x100800, ltc->ltc_nr);
	nvkm_wr32(device, 0x17e278, ltc->tag_base);
	nvkm_wr32(device, 0x17e318, nvkm_rd32(device, 0x140518) | 0x00008000);
}

static const struct nvkm_ltc_func
gm20b_ltc = {
	.oneinit = gm20b_ltc_oneinit,
	.init = gm20b_ltc_init,
	.intr = gm107_ltc_intr,
	.cbc_clear = gm107_ltc_cbc_clear,
	.cbc_wait = gm107_ltc_cbc_wait,
	.zbc_color = 16,
	.zbc_depth = 16,
	.zbc_clear_color = gm107_ltc_zbc_clear_color,
	.zbc_clear_depth = gm107_ltc_zbc_clear_depth,
	.invalidate = gf100_ltc_invalidate,
	.flush = gf100_ltc_flush,
	.cbc_clean = gf100_ltc_cbc_clean,
};

int
gm20b_ltc_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_ltc **pltc)
{
	return nvkm_ltc_new_(&gm20b_ltc, device, type, inst, pltc);
}
