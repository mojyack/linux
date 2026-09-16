// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 mojyack <mojyack@gmail.com>
 */
#include "priv.h"

static void
gm20b_ltc_init(struct nvkm_ltc *ltc)
{
	struct nvkm_device *device = ltc->subdev.device;

	nvkm_wr32(device, 0x17e27c, ltc->ltc_nr);
	nvkm_wr32(device, 0x17e000, ltc->ltc_nr);
	nvkm_wr32(device, 0x100800, ltc->ltc_nr);
	nvkm_wr32(device, 0x17e278, ltc->tag_base);
}

static const struct nvkm_ltc_func
gm20b_ltc = {
	.oneinit = gm200_ltc_oneinit,
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
};

int
gm20b_ltc_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_ltc **pltc)
{
	return nvkm_ltc_new_(&gm20b_ltc, device, type, inst, pltc);
}
