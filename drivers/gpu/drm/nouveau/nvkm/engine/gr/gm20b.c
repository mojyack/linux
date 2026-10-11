/*
 * Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
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
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include "gf100.h"
#include "ctxgf100.h"

#include <core/firmware.h>
#include <subdev/acr.h>
#include <subdev/timer.h>

#include <nvfw/flcn.h>

#include <nvif/class.h>
#include <nvif/cl9097.h>

/* sRGB8 thresholds of the ROP, colour in 2^-14 units and alpha in 2^-16 */
static const u16
gm20b_gr_zbc_srgb8[255] = {
	    2,     7,    12,    17,    22,    27,    32,    37,    42,    47,
	   52,    58,    63,    69,    75,    82,    88,    96,   103,   110,
	  118,   127,   136,   145,   154,   164,   174,   184,   196,   208,
	  218,   230,   244,   256,   268,   282,   296,   310,   324,   340,
	  356,   372,   388,   404,   420,   440,   456,   476,   492,   512,
	  532,   552,   572,   592,   616,   636,   660,   680,   704,   728,
	  752,   776,   800,   828,   852,   880,   904,   936,   960,   992,
	 1016,  1048,  1076,  1104,  1136,  1168,  1200,  1232,  1264,  1296,
	 1328,  1368,  1400,  1432,  1472,  1504,  1544,  1580,  1616,  1656,
	 1696,  1736,  1776,  1816,  1856,  1896,  1936,  1984,  2024,  2064,
	 2112,  2152,  2200,  2248,  2288,  2336,  2384,  2432,  2480,  2528,
	 2576,  2632,  2680,  2736,  2784,  2832,  2888,  2944,  2992,  3048,
	 3104,  3160,  3216,  3272,  3328,  3392,  3448,  3504,  3568,  3632,
	 3688,  3744,  3808,  3872,  3936,  4000,  4064,  4128,  4192,  4264,
	 4336,  4400,  4464,  4528,  4608,  4672,  4744,  4816,  4888,  4960,
	 5040,  5104,  5184,  5264,  5328,  5408,  5488,  5568,  5648,  5712,
	 5792,  5872,  5952,  6048,  6128,  6208,  6288,  6368,  6464,  6544,
	 6624,  6720,  6800,  6896,  6976,  7072,  7152,  7248,  7344,  7424,
	 7520,  7616,  7712,  7808,  7904,  8000,  8096,  8192,  8284,  8380,
	 8492,  8588,  8684,  8796,  8892,  8988,  9100,  9196,  9308,  9404,
	 9516,  9628,  9724,  9836,  9948, 10060, 10172, 10268, 10396, 10492,
	10620, 10732, 10844, 10956, 11068, 11196, 11308, 11420, 11548, 11676,
	11788, 11900, 12028, 12156, 12284, 12396, 12524, 12636, 12764, 12892,
	13020, 13148, 13276, 13404, 13548, 13676, 13820, 13948, 14076, 14204,
	14332, 14492, 14620, 14748, 14892, 15036, 15164, 15308, 15452, 15580,
	15740, 15868, 16028, 16156, 16316,
};

static const u16
gm20b_gr_zbc_srgb8_alpha[255] = {
	  129,   386,   643,   900,  1157,  1414,  1671,  1928,  2186,  2442,
	 2700,  2956,  3214,  3470,  3728,  3984,  4244,  4500,  4756,  5012,
	 5272,  5528,  5784,  6040,  6300,  6556,  6812,  7068,  7328,  7584,
	 7840,  8096,  8360,  8616,  8872,  9128,  9384,  9640,  9896, 10152,
	10416, 10672, 10928, 11184, 11440, 11696, 11952, 12208, 12472, 12728,
	12984, 13240, 13496, 13752, 14008, 14264, 14528, 14784, 15040, 15296,
	15552, 15808, 16064, 16320, 16592, 16848, 17104, 17360, 17616, 17872,
	18128, 18384, 18640, 18896, 19152, 19408, 19664, 19920, 20176, 20432,
	20704, 20960, 21216, 21472, 21728, 21984, 22240, 22496, 22752, 23008,
	23264, 23520, 23776, 24032, 24288, 24544, 24816, 25072, 25328, 25584,
	25840, 26096, 26352, 26608, 26864, 27120, 27376, 27632, 27888, 28144,
	28400, 28656, 28928, 29184, 29440, 29696, 29952, 30208, 30464, 30720,
	30976, 31232, 31488, 31744, 32000, 32256, 32512, 32784, 33040, 33296,
	33552, 33808, 34064, 34320, 34576, 34832, 35088, 35344, 35600, 35856,
	36112, 36368, 36624, 36880, 37136, 37392, 37648, 37904, 38160, 38416,
	38672, 38928, 39184, 39440, 39696, 39952, 40208, 40464, 40720, 41008,
	41264, 41520, 41776, 42032, 42288, 42544, 42800, 43056, 43312, 43568,
	43824, 44080, 44336, 44592, 44848, 45104, 45360, 45616, 45872, 46128,
	46384, 46640, 46896, 47152, 47408, 47664, 47920, 48176, 48432, 48688,
	48944, 49232, 49488, 49744, 50000, 50256, 50512, 50768, 51024, 51280,
	51536, 51792, 52048, 52304, 52560, 52816, 53072, 53328, 53584, 53840,
	54096, 54352, 54608, 54864, 55120, 55376, 55632, 55888, 56144, 56400,
	56656, 56912, 57168, 57456, 57712, 57968, 58224, 58480, 58736, 58992,
	59248, 59504, 59760, 60016, 60272, 60528, 60784, 61040, 61296, 61552,
	61808, 62064, 62320, 62576, 62832, 63088, 63344, 63600, 63856, 64112,
	64368, 64624, 64880, 65136, 65392,
};

/* floor(f * 2^shift) for a float f, saturated to [0, 2^shift] */
static u32
gm20b_gr_zbc_fixed(u32 f, int shift)
{
	int sh;

	if ((f & 0x80000000) || !f)
		return 0;
	if (f >= 0x3f800000)
		return 1 << shift;

	sh = 150 - (int)(f >> 23) - shift;
	return sh < 32 ? ((f & 0x007fffff) | 0x00800000) >> sh : 0;
}

static u32
gm20b_gr_zbc_unorm(u32 f, int bits)
{
	u32 q = gm20b_gr_zbc_fixed(f, bits + 4);

	return (q * ((1 << bits) - 1) + (1 << (bits + 3)) - 1) >> (bits + 4);
}

static u32
gm20b_gr_zbc_lookup(const u16 *thr, u32 q)
{
	u32 v = 0;

	while (v < 255 && thr[v] <= q)
		v++;
	return v;
}

/* f32 to a smaller float, rounding toward zero and saturating */
static u32
gm20b_gr_zbc_float(u32 f, int ebits, int mbits, bool sign)
{
	const int emax = (1 << ebits) - 1;
	const int e = (int)((f >> 23) & 0xff) - 127 + (emax >> 1);
	const int dsh = 1 - e + 23 - mbits;
	u32 s = f >> 31, v;

	if (s && !sign)
		return 0;

	if (e >= emax)
		v = (emax << mbits) - 1;
	else if (e > 0)
		v = (e << mbits) | ((f & 0x007fffff) >> (23 - mbits));
	else if ((f & 0x7f800000) && dsh < 32)
		v = ((f & 0x007fffff) | 0x00800000) >> dsh;
	else
		v = 0;

	return v | s << (ebits + mbits);
}

/* Measured: what this ROP stores for a clear to ds[] without ZBC */
static int
gm20b_gr_zbc_color_l2(u8 format, const u32 ds[4], u32 l2[4])
{
	u32 c[4], px[2];
	int i, n = 1;

	switch (format) {
	case FERMI_A_ZBC_COLOR_V0_FMT_A8B8G8R8:
	case FERMI_A_ZBC_COLOR_V0_FMT_A8R8G8B8:
		for (i = 0; i < 4; i++)
			c[i] = gm20b_gr_zbc_unorm(ds[i], 8);
		break;
	case FERMI_A_ZBC_COLOR_V0_FMT_A8BL8GL8RL8:
	case FERMI_A_ZBC_COLOR_V0_FMT_A8RL8GL8BL8:
		for (i = 0; i < 3; i++)
			c[i] = gm20b_gr_zbc_lookup(gm20b_gr_zbc_srgb8,
						   gm20b_gr_zbc_fixed(ds[i], 14));
		c[3] = gm20b_gr_zbc_lookup(gm20b_gr_zbc_srgb8_alpha,
					   gm20b_gr_zbc_fixed(ds[3], 16));
		break;
	case FERMI_A_ZBC_COLOR_V0_FMT_A2B10G10R10:
	case FERMI_A_ZBC_COLOR_V0_FMT_A2R10G10B10:
		for (i = 0; i < 4; i++)
			c[i] = gm20b_gr_zbc_unorm(ds[i], 10);
		/* alpha: the top 4 bits of the 10-bit value, rounded to 2 */
		c[3] = ((c[3] >> 6) * 3 + 7) / 15;
		break;
	case FERMI_A_ZBC_COLOR_V0_FMT_BF10GF11RF11:
	case FERMI_A_ZBC_COLOR_V0_FMT_RF16_GF16_BF16_AF16:
		break;
	default:
		return -EOPNOTSUPP;
	}

	switch (format) {
	case FERMI_A_ZBC_COLOR_V0_FMT_A8B8G8R8:
	case FERMI_A_ZBC_COLOR_V0_FMT_A8BL8GL8RL8:
		px[0] = c[0] | c[1] << 8 | c[2] << 16 | c[3] << 24;
		break;
	case FERMI_A_ZBC_COLOR_V0_FMT_A8R8G8B8:
	case FERMI_A_ZBC_COLOR_V0_FMT_A8RL8GL8BL8:
		px[0] = c[2] | c[1] << 8 | c[0] << 16 | c[3] << 24;
		break;
	case FERMI_A_ZBC_COLOR_V0_FMT_A2B10G10R10:
		px[0] = c[0] | c[1] << 10 | c[2] << 20 | c[3] << 30;
		break;
	case FERMI_A_ZBC_COLOR_V0_FMT_A2R10G10B10:
		px[0] = c[2] | c[1] << 10 | c[0] << 20 | c[3] << 30;
		break;
	case FERMI_A_ZBC_COLOR_V0_FMT_BF10GF11RF11:
		px[0] = gm20b_gr_zbc_float(ds[0], 5, 6, false) |
			gm20b_gr_zbc_float(ds[1], 5, 6, false) << 11 |
			gm20b_gr_zbc_float(ds[2], 5, 5, false) << 22;
		break;
	default:
		px[0] = gm20b_gr_zbc_float(ds[0], 5, 10, true) |
			gm20b_gr_zbc_float(ds[1], 5, 10, true) << 16;
		px[1] = gm20b_gr_zbc_float(ds[2], 5, 10, true) |
			gm20b_gr_zbc_float(ds[3], 5, 10, true) << 16;
		n = 2;
		break;
	}

	for (i = 0; i < 4; i++)
		l2[i] = px[i % n];
	return 0;
}

void
gm20b_gr_acr_bld_patch(struct nvkm_acr *acr, u32 bld, s64 adjust)
{
	struct flcn_bl_dmem_desc hdr;
	u64 addr;

	nvkm_robj(acr->wpr, bld, &hdr, sizeof(hdr));
	addr = ((u64)hdr.code_dma_base1 << 40 | hdr.code_dma_base << 8);
	hdr.code_dma_base  = lower_32_bits((addr + adjust) >> 8);
	hdr.code_dma_base1 = upper_32_bits((addr + adjust) >> 8);
	addr = ((u64)hdr.data_dma_base1 << 40 | hdr.data_dma_base << 8);
	hdr.data_dma_base  = lower_32_bits((addr + adjust) >> 8);
	hdr.data_dma_base1 = upper_32_bits((addr + adjust) >> 8);
	nvkm_wobj(acr->wpr, bld, &hdr, sizeof(hdr));

	flcn_bl_dmem_desc_dump(&acr->subdev, &hdr);
}

void
gm20b_gr_acr_bld_write(struct nvkm_acr *acr, u32 bld,
		       struct nvkm_acr_lsfw *lsfw)
{
	const u64 base = lsfw->offset.img + lsfw->app_start_offset;
	const u64 code = (base + lsfw->app_resident_code_offset) >> 8;
	const u64 data = (base + lsfw->app_resident_data_offset) >> 8;
	const struct flcn_bl_dmem_desc hdr = {
		.ctx_dma = FALCON_DMAIDX_UCODE,
		.code_dma_base = lower_32_bits(code),
		.non_sec_code_off = lsfw->app_resident_code_offset,
		.non_sec_code_size = lsfw->app_resident_code_size,
		.code_entry_point = lsfw->app_imem_entry,
		.data_dma_base = lower_32_bits(data),
		.data_size = lsfw->app_resident_data_size,
		.code_dma_base1 = upper_32_bits(code),
		.data_dma_base1 = upper_32_bits(data),
	};

	nvkm_wobj(acr->wpr, bld, &hdr, sizeof(hdr));
}

const struct nvkm_acr_lsf_func
gm20b_gr_fecs_acr = {
	.bld_size = sizeof(struct flcn_bl_dmem_desc),
	.bld_write = gm20b_gr_acr_bld_write,
	.bld_patch = gm20b_gr_acr_bld_patch,
};

static void
gm20b_gr_init_gpc_mmu(struct gf100_gr *gr)
{
	struct nvkm_device *device = gr->base.engine.subdev.device;
	u32 val;

	/* Bypass MMU check for non-secure boot */
	if (!device->acr) {
		nvkm_wr32(device, 0x100ce4, 0xffffffff);

		if (nvkm_rd32(device, 0x100ce4) != 0xffffffff)
			nvdev_warn(device,
			  "cannot bypass secure boot - expect failure soon!\n");
	}

	val = nvkm_rd32(device, 0x100c80);
	val &= 0xf000187f;
	nvkm_wr32(device, 0x418880, val);
	nvkm_wr32(device, 0x418890, 0);
	nvkm_wr32(device, 0x418894, 0);

	nvkm_wr32(device, 0x4188b0, nvkm_rd32(device, 0x100cc4));
	nvkm_wr32(device, 0x4188b4, nvkm_rd32(device, 0x100cc8));
	nvkm_wr32(device, 0x4188b8, nvkm_rd32(device, 0x100ccc));

	nvkm_wr32(device, 0x4188ac, nvkm_rd32(device, 0x100800));
}

static void
gm20b_gr_set_hww_esr_report_mask(struct gf100_gr *gr)
{
	struct nvkm_device *device = gr->base.engine.subdev.device;
	nvkm_wr32(device, 0x419e44, 0xdffffe);
	nvkm_wr32(device, 0x419e4c, 0x5);
}

static const struct gf100_gr_func
gm20b_gr = {
	.oneinit_tiles = gm200_gr_oneinit_tiles,
	.oneinit_sm_id = gm200_gr_oneinit_sm_id,
	.init = gk20a_gr_init,
	.init_zcull = gf117_gr_init_zcull,
	.init_gpc_mmu = gm20b_gr_init_gpc_mmu,
	.init_rop_active_fbps = gm200_gr_init_rop_active_fbps,
	.trap_mp = gf100_gr_trap_mp,
	.set_hww_esr_report_mask = gm20b_gr_set_hww_esr_report_mask,
	.fecs.reset = gf100_gr_fecs_reset,
	.rops = gm200_gr_rops,
	.ppc_nr = 1,
	.grctx = &gm20b_grctx,
	.zbc = &gf100_gr_zbc,
	.zbc_color_l2 = gm20b_gr_zbc_color_l2,
	.sclass = {
		{ -1, -1, FERMI_TWOD_A },
		{ -1, -1, KEPLER_INLINE_TO_MEMORY_B },
		{ -1, -1, MAXWELL_B, &gf100_fermi },
		{ -1, -1, MAXWELL_COMPUTE_B },
		{}
	}
};

static int
gm20b_gr_load(struct gf100_gr *gr, int ver, const struct gf100_gr_fwif *fwif)
{
	struct nvkm_subdev *subdev = &gr->base.engine.subdev;
	int ret;

	ret = nvkm_acr_lsfw_load_bl_inst_data_sig(subdev, &gr->fecs.falcon,
						  NVKM_ACR_LSF_FECS,
						  "gr/fecs_", ver, fwif->fecs);
	if (ret)
		return ret;


	if (nvkm_firmware_load_blob(subdev, "gr/", "gpccs_inst", ver,
				    &gr->gpccs.inst) ||
	    nvkm_firmware_load_blob(subdev, "gr/", "gpccs_data", ver,
				    &gr->gpccs.data))
		return -ENOENT;

	gr->firmware = true;

	return gk20a_gr_load_sw(gr, "gr/", ver);
}

#if IS_ENABLED(CONFIG_ARCH_TEGRA_210_SOC)
MODULE_FIRMWARE("nvidia/gm20b/gr/fecs_bl.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/fecs_inst.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/fecs_data.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/fecs_sig.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/gpccs_inst.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/gpccs_data.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/sw_ctx.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/sw_nonctx.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/sw_bundle_init.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/sw_method_init.bin");
#endif

static const struct gf100_gr_fwif
gm20b_gr_fwif[] = {
	{  0, gm20b_gr_load, &gm20b_gr, &gm20b_gr_fecs_acr },
	{ -1, gm200_gr_nofw },
	{}
};

int
gm20b_gr_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst, struct nvkm_gr **pgr)
{
	return gf100_gr_new_(gm20b_gr_fwif, device, type, inst, pgr);
}
