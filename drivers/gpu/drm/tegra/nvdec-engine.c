// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2022, NVIDIA Corporation.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/dma-fence.h>
#include <linux/dma-resv.h>
#include <linux/host1x.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/iosys-map.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>

#include <soc/tegra/mc.h>

#include "drm.h"
#include "falcon.h"
#include "gem.h"
#include "nvdec-engine.h"
#include "riscv.h"
#include "vic.h"
#include "vic-engine.h"

#define NVDEC_FALCON_DEBUGINFO			0x1094
#define NVDEC_TFBIF_TRANSCFG			0x2c44

#define NVDEC_METHOD_APPLICATION		0x080
#define NVDEC_METHOD_EXECUTE			0x0c0
#define NVDEC_METHOD_CONTROL			0x100
#define NVDEC_METHOD_SETUP			0x101
#define NVDEC_METHOD_INPUT			0x102
#define NVDEC_METHOD_PICTURE_INDEX		0x103
#define NVDEC_METHOD_SLICE_OFFSETS		0x104
#define NVDEC_METHOD_COLOC			0x105
#define NVDEC_METHOD_HISTORY			0x106
#define NVDEC_METHOD_STATUS			0x109
#define NVDEC_METHOD_LUMA			0x10c
#define NVDEC_METHOD_CHROMA			0x11d
#define NVDEC_H264_METHOD_MBHIST		0x140
#define NVDEC_HEVC_METHOD_SCALING_LIST		0x160
#define NVDEC_HEVC_METHOD_TILE_SIZES		0x161
#define NVDEC_HEVC_METHOD_FILTER		0x162

/* The NVDEC methods, OP_DONE, the VIC detile and OP_DONE. */
#define NVDEC_GATHER_WORDS			256

#define NVDEC_H264_SETUP_SIZE			0x2fc
#define NVDEC_H264_STATUS_OFFSET		0x300

#define NVDEC_HEVC_SETUP_SIZE			0x114
#define NVDEC_HEVC_STATUS_OFFSET		0x200
#define NVDEC_HEVC_SCALING_OFFSET		0x300
#define NVDEC_HEVC_TILES_OFFSET		0x700
#define NVDEC_HEVC_TILES_SIZE			0x900

/* Per aligned luma row, as the oracle sizes them. */
#define NVDEC_HEVC_FILTER_PER_ROW		480
#define NVDEC_HEVC_SAO_PER_ROW			3840
#define NVDEC_HEVC_BSD_PER_ROW			60

struct nvdec_h264_dpb_entry {
	__le32 flags;
	__le32 field_order_cnt[2];
	__le32 frame_idx;
};

struct nvdec_h264_prefix {
	u8 encryption[0x34];
	u8 eos[16];
	u8 explicit_eos_present;
	u8 hint_dump_enable;
	u8 reserved[2];
};

struct nvdec_h264_status {
	u8 data[0x100];
};

struct nvdec_h264_setup {
	struct nvdec_h264_prefix prefix;
	__le32 stream_len;
	__le32 slice_count;
	__le32 mbhist_buffer_size;
	__le32 gptimer_timeout_value;
	__le32 log2_max_pic_order_cnt_lsb_minus4;
	__le32 delta_pic_order_always_zero_flag;
	__le32 frame_mbs_only_flag;
	__le32 pic_width_in_mbs;
	__le32 frame_height_in_mbs;
	__le32 tile_format;
	__le32 entropy_coding_mode_flag;
	__le32 pic_order_present_flag;
	__le32 num_ref_idx_l0_active_minus1;
	__le32 num_ref_idx_l1_active_minus1;
	__le32 deblocking_filter_control_present_flag;
	__le32 redundant_pic_cnt_present_flag;
	__le32 transform_8x8_mode_flag;
	__le32 pitch_luma;
	__le32 pitch_chroma;
	__le32 luma_top_offset;
	__le32 luma_bot_offset;
	__le32 luma_frame_offset;
	__le32 chroma_top_offset;
	__le32 chroma_bot_offset;
	__le32 chroma_frame_offset;
	__le32 history_buffer_size;
	__le32 picture_flags;
	__le32 current_picture;
	__le32 current_field_order_cnt[2];
	struct nvdec_h264_dpb_entry dpb[NVDEC_H264_DPB_ENTRIES];
	u8 scaling_4x4[6][16];
	u8 scaling_8x8[2][64];
	u8 mvc[0x30];
	__le32 lossless_flags;
	u8 display[0x1c];
	u8 ssm[0xc];
};

static_assert(sizeof(struct nvdec_h264_dpb_entry) == 0x10);
static_assert(sizeof(struct nvdec_h264_prefix) == 0x48);
static_assert(sizeof(struct nvdec_h264_status) == 0x100);
static_assert(offsetof(struct nvdec_h264_setup, stream_len) == sizeof(struct nvdec_h264_prefix));
static_assert(offsetof(struct nvdec_h264_setup, dpb) == 0xc0);
static_assert(offsetof(struct nvdec_h264_setup, scaling_4x4) == 0x1c0);
static_assert(offsetof(struct nvdec_h264_setup, scaling_8x8) == 0x220);
static_assert(sizeof(struct nvdec_h264_setup) == NVDEC_H264_SETUP_SIZE);

struct nvdec_hevc_setup {
	u8 encryption[0x30];
	__le32 stream_len;
	__le32 enable_encryption;
	__le32 key_control;
	__le32 gptimer_timeout_value;
	__le32 surface_format;
	__le32 framestride[2];
	__le32 coloc_buffer_size;
	__le32 sao_buffer_offset;
	__le32 bsd_control_offset;
	__le16 pic_width_in_luma_samples;
	__le16 pic_height_in_luma_samples;
	__le32 sps_geometry;
	__le32 sps_flags;
	__le32 pps_flags0;
	s8 pps_cb_qp_offset;
	s8 pps_cr_qp_offset;
	s8 pps_beta_offset;
	s8 pps_tc_offset;
	__le32 pps_flags1;
	u8 num_ref_frames;
	u8 reserved0;
	__le16 longtermflag;
	u8 initreflistidxl0[NVDEC_HEVC_MAX_PICTURES];
	u8 initreflistidxl1[NVDEC_HEVC_MAX_PICTURES];
	__le16 ref_diff_poc[NVDEC_HEVC_MAX_PICTURES];
	u8 idr_picture_flag;
	u8 rap_picture_flag;
	u8 curr_pic_idx;
	u8 pattern_id;
	__le16 sw_hdr_skip_length;
	__le16 reserved1;
	u8 ecdma_cfg[0x18];
	__le32 dxva_flags;
	__le32 num_bits_short_term_ref_pics_in_slice;
	u8 extensions[0x38];
};

static_assert(offsetof(struct nvdec_hevc_setup, surface_format) == 0x40);
static_assert(offsetof(struct nvdec_hevc_setup, sps_geometry) == 0x5c);
static_assert(offsetof(struct nvdec_hevc_setup, pps_flags1) == 0x6c);
static_assert(offsetof(struct nvdec_hevc_setup, ref_diff_poc) == 0x94);
static_assert(offsetof(struct nvdec_hevc_setup, sw_hdr_skip_length) == 0xb8);
static_assert(sizeof(struct nvdec_hevc_setup) == NVDEC_HEVC_SETUP_SIZE);

struct nvdec_hevc_scaling_list {
	u8 dc_16x16[6];
	u8 dc_32x32[2];
	u8 reserved[8];
	u8 list_4x4[6][16];
	u8 list_8x8[6][64];
	u8 list_16x16[6][64];
	u8 list_32x32[2][64];
};

static_assert(sizeof(struct nvdec_hevc_scaling_list) == 0x3f0);

#define NVDEC_HEVC_SURFACE_TILEFORMAT		GENMASK(1, 0)
#define NVDEC_HEVC_SURFACE_GOB_HEIGHT		GENMASK(4, 2)
#define NVDEC_HEVC_SURFACE_START_CODE		GENMASK(15, 8)
#define NVDEC_HEVC_SURFACE_OUTPUT_MODE		GENMASK(23, 16)

#define NVDEC_HEVC_GEOM_CHROMA_FORMAT		GENMASK(3, 0)
#define NVDEC_HEVC_GEOM_BIT_DEPTH_LUMA		GENMASK(7, 4)
#define NVDEC_HEVC_GEOM_BIT_DEPTH_CHROMA	GENMASK(11, 8)
#define NVDEC_HEVC_GEOM_LOG2_MIN_CB		GENMASK(15, 12)
#define NVDEC_HEVC_GEOM_LOG2_MAX_CB		GENMASK(19, 16)
#define NVDEC_HEVC_GEOM_LOG2_MIN_TB		GENMASK(23, 20)
#define NVDEC_HEVC_GEOM_LOG2_MAX_TB		GENMASK(27, 24)

#define NVDEC_HEVC_SPS_HIER_INTER		GENMASK(2, 0)
#define NVDEC_HEVC_SPS_HIER_INTRA		GENMASK(5, 3)
#define NVDEC_HEVC_SPS_SCALING_LIST_EN		BIT(6)
#define NVDEC_HEVC_SPS_AMP_EN			BIT(7)
#define NVDEC_HEVC_SPS_SAO_EN			BIT(8)
#define NVDEC_HEVC_SPS_PCM_EN			BIT(9)
#define NVDEC_HEVC_SPS_PCM_DEPTH_LUMA		GENMASK(13, 10)
#define NVDEC_HEVC_SPS_PCM_DEPTH_CHROMA	GENMASK(17, 14)
#define NVDEC_HEVC_SPS_LOG2_MIN_PCM		GENMASK(21, 18)
#define NVDEC_HEVC_SPS_LOG2_MAX_PCM		GENMASK(25, 22)
#define NVDEC_HEVC_SPS_PCM_LOOP_FILTER_DIS	BIT(26)
#define NVDEC_HEVC_SPS_TEMPORAL_MVP_EN		BIT(27)
#define NVDEC_HEVC_SPS_STRONG_INTRA_SMOOTH	BIT(28)

#define NVDEC_HEVC_PPS0_DEPENDENT_SLICES	BIT(0)
#define NVDEC_HEVC_PPS0_OUTPUT_FLAG_PRESENT	BIT(1)
#define NVDEC_HEVC_PPS0_EXTRA_SLICE_BITS	GENMASK(4, 2)
#define NVDEC_HEVC_PPS0_SIGN_DATA_HIDING	BIT(5)
#define NVDEC_HEVC_PPS0_CABAC_INIT_PRESENT	BIT(6)
#define NVDEC_HEVC_PPS0_NUM_REF_IDX_L0		GENMASK(10, 7)
#define NVDEC_HEVC_PPS0_NUM_REF_IDX_L1		GENMASK(14, 11)
#define NVDEC_HEVC_PPS0_INIT_QP		GENMASK(21, 15)
#define NVDEC_HEVC_PPS0_CONSTRAINED_INTRA	BIT(22)
#define NVDEC_HEVC_PPS0_TRANSFORM_SKIP		BIT(23)
#define NVDEC_HEVC_PPS0_CU_QP_DELTA		BIT(24)
#define NVDEC_HEVC_PPS0_DIFF_CU_QP_DEPTH	GENMASK(26, 25)

#define NVDEC_HEVC_PPS1_SLICE_CHROMA_QP	BIT(0)
#define NVDEC_HEVC_PPS1_WEIGHTED_PRED		BIT(1)
#define NVDEC_HEVC_PPS1_WEIGHTED_BIPRED	BIT(2)
#define NVDEC_HEVC_PPS1_TRANSQUANT_BYPASS	BIT(3)
#define NVDEC_HEVC_PPS1_TILES_ENABLED		BIT(4)
#define NVDEC_HEVC_PPS1_ENTROPY_SYNC		BIT(5)
#define NVDEC_HEVC_PPS1_NUM_TILE_COLUMNS	GENMASK(10, 6)
#define NVDEC_HEVC_PPS1_NUM_TILE_ROWS		GENMASK(15, 11)
#define NVDEC_HEVC_PPS1_LF_ACROSS_TILES	BIT(16)
#define NVDEC_HEVC_PPS1_LF_ACROSS_SLICES	BIT(17)
#define NVDEC_HEVC_PPS1_DEBLOCK_CONTROL	BIT(18)
#define NVDEC_HEVC_PPS1_DEBLOCK_OVERRIDE	BIT(19)
#define NVDEC_HEVC_PPS1_DEBLOCK_DISABLED	BIT(20)
#define NVDEC_HEVC_PPS1_LISTS_MODIFICATION	BIT(21)
#define NVDEC_HEVC_PPS1_LOG2_PARALLEL_MERGE	GENMASK(24, 22)
#define NVDEC_HEVC_PPS1_SLICE_HDR_EXTENSION	BIT(25)

struct nvdec_engine_map {
	struct kref ref;
	struct tegra_bo *bo;
	/* Kernel-owned buffers the CPU fills are vmapped here. */
	void *cpu;
	/* Without an explicit IOMMU domain, the engine's own DMA mapping. */
	struct host1x_bo_mapping *pin;
	/* Set for an import: where its producers and consumers meet. */
	struct dma_resv *resv;
	dma_addr_t iova;
	size_t size;
	enum dma_data_direction direction;
};

struct nvdec_pool_surface {
	struct nvdec_engine_map *map;
	u8 picture_index;
	/* H.264 setup slot, NVDEC_H264_DPB_ENTRIES while this is not a reference. */
	u8 dpb_slot;
};

struct nvdec_decode_context {
	struct kref ref;
	struct nvdec_engine *engine;
	/* Serializes staging and submission of this context's pictures. */
	struct mutex lock;
	enum nvdec_codec codec;
	struct nvdec_engine_map *scratch;
	struct nvdec_engine_map *input;
	u16 width_in_mbs;
	u16 height_in_mbs;
	u16 coded_width;
	u16 coded_height;
	u32 coloc_size;
	u32 mbhist_offset;
	u32 mbhist_size;
	u32 history_offset;
	u32 history_size;
	u32 colmv_size;
	u32 filter_offset;
	u32 sao_offset;
	u32 bsd_offset;
	bool in_flight;
	/* Slices of the current picture, staged in ctx->input until the last. */
	u32 *slice_offsets;
	unsigned int slice_count;
	unsigned int max_slices;
	u32 staged;
	struct nvdec_pool_surface surfaces[NVDEC_MAX_PICTURES];
};

struct nvdec_decode_job {
	struct nvdec_decode_context *ctx;
	const struct nvdec_codec_ops *ops;
	union nvdec_request req;
	struct nvdec_engine_map *state;
	struct nvdec_engine_map *input;
	struct nvdec_engine_map *gather;
	struct nvdec_engine_map *scratch;
	struct nvdec_engine_map *surface;
	struct nvdec_engine_map *capture;
	struct nvdec_engine_map *dpb[NVDEC_MAX_REFS];
	/* What each firmware picture slot names; borrowed from the maps above. */
	struct nvdec_engine_map *pictures[NVDEC_MAX_PICTURES];
	u8 num_pictures;
	u8 picture_index;
	u8 picture_indices[NVDEC_MAX_REFS];
	u8 dpb_slots[NVDEC_MAX_REFS];
	unsigned int vic_offset;
	u32 slice_offsets_off;
	struct vic_engine *vic;
	struct dma_fence *fence;
	nvdec_engine_complete_t complete;
	void *complete_data;
	bool runtime_ref;
	bool vic_runtime_ref;
	bool submitted;
};

/* What differs between codecs in turning a request into one host1x job. */
struct nvdec_codec_ops {
	u8 application;
	u16 setup_size;
	u16 status_offset;
	/* The state buffer's VIC config, after the codec's own tables. */
	u16 vic_config;
	/* Written after the bitstream when the firmware takes slice offsets. */
	const u8 *termination;
	/* Validate, size the scratch, and name the firmware picture slots. */
	int (*prepare)(struct nvdec_decode_job *hjob);
	/* Fill the setup structure and the codec's tables in the state buffer. */
	void (*fill)(struct nvdec_decode_job *hjob);
	/* The codec's address methods, between STATUS and the pictures. */
	void (*emit)(struct nvdec_decode_job *hjob, struct falcon_gather *g);
};

struct nvdec_fence {
	struct dma_fence base;
	/* Protects the dma_fence base. */
	spinlock_t lock;
};

struct nvdec_engine_config {
	const char *firmware;
	unsigned int version;
	bool supports_sid;
	bool has_riscv;
	bool has_extra_clocks;
};

struct nvdec_engine {
	struct falcon falcon;
	struct tegra_drm_client client;
	struct host1x_channel *channel;
	struct device *dev;
	void __iomem *regs;
	struct clk_bulk_data clks[3];
	unsigned int num_clks;
	struct reset_control *reset;
	const struct nvdec_engine_config *config;

	struct tegra_drm_riscv riscv;
	phys_addr_t carveout_base;

	/* Serializes submission with PM and the reset path. */
	struct mutex recovery_lock;
	atomic_t active_jobs;
	struct completion idle;
	struct work_struct recovery_work;
	u32 recovery_generation;
	u64 h264_fence_context;
	atomic64_t h264_fence_seqno;
	struct nvdec_v4l2 *v4l2;
};

struct nvdec_engine_job {
	struct nvdec_engine *engine;
	void (*release)(struct host1x_job *job);
	void *user_data;
	nvdec_engine_job_complete_t complete;
	void *complete_data;
};

static inline struct nvdec_engine *
to_nvdec_engine(struct tegra_drm_client *client)
{
	return container_of(client, struct nvdec_engine, client);
}

static inline void nvdec_engine_writel(struct nvdec_engine *engine, u32 value,
				       unsigned int offset)
{
	writel(value, engine->regs + offset);
}

static int nvdec_engine_boot_falcon(struct nvdec_engine *engine)
{
	u32 stream_id;
	int err;

	if (engine->config->supports_sid &&
	    tegra_dev_iommu_get_stream_id(engine->dev, &stream_id)) {
		u32 value;

		value = TRANSCFG_ATT(1, TRANSCFG_SID_FALCON) |
			TRANSCFG_ATT(0, TRANSCFG_SID_HW);
		nvdec_engine_writel(engine, value, NVDEC_TFBIF_TRANSCFG);

		nvdec_engine_writel(engine, stream_id, VIC_THI_STREAMID0);
		nvdec_engine_writel(engine, stream_id, VIC_THI_STREAMID1);
	}

	err = falcon_boot(&engine->falcon);
	if (err < 0)
		return err;

	err = falcon_wait_idle(&engine->falcon);
	if (err < 0) {
		dev_err(engine->dev, "falcon boot timed out\n");
		return err;
	}

	return 0;
}

static int nvdec_engine_wait_debuginfo(struct nvdec_engine *engine,
				       const char *phase)
{
	int err;
	u32 val;

	err = readl_poll_timeout(engine->regs + NVDEC_FALCON_DEBUGINFO, val,
				 val == 0x0, 10, 100000);
	if (err) {
		dev_err(engine->dev, "failed to boot %s, debuginfo=0x%x\n",
			phase, val);
		return err;
	}

	return 0;
}

static int nvdec_engine_boot_riscv(struct nvdec_engine *engine)
{
	int err;

	err = reset_control_acquire(engine->reset);
	if (err)
		return err;

	nvdec_engine_writel(engine, 0xabcd1234, NVDEC_FALCON_DEBUGINFO);

	err = tegra_drm_riscv_boot_bootrom(&engine->riscv,
					   engine->carveout_base, 1,
					   &engine->riscv.bl_desc);
	if (err) {
		dev_err(engine->dev, "failed to execute bootloader\n");
		goto release_reset;
	}

	err = nvdec_engine_wait_debuginfo(engine, "bootloader");
	if (err)
		goto release_reset;

	err = reset_control_reset(engine->reset);
	if (err)
		goto release_reset;

	nvdec_engine_writel(engine, 0xabcd1234, NVDEC_FALCON_DEBUGINFO);

	err = tegra_drm_riscv_boot_bootrom(&engine->riscv,
					   engine->carveout_base, 1,
					   &engine->riscv.os_desc);
	if (err) {
		dev_err(engine->dev, "failed to execute firmware\n");
		goto release_reset;
	}

	err = nvdec_engine_wait_debuginfo(engine, "firmware");

release_reset:
	reset_control_release(engine->reset);

	return err;
}

static int nvdec_engine_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvdec_engine *engine = to_nvdec_engine(drm);
	int err;

	err = host1x_client_iommu_attach(client);
	if (err < 0 && err != -ENODEV) {
		dev_err(engine->dev, "failed to attach to domain: %d\n", err);
		return err;
	}

	engine->channel = host1x_channel_request(client);
	if (!engine->channel) {
		err = -ENOMEM;
		goto detach;
	}

	client->syncpts[0] = host1x_syncpt_request(client, 0);
	if (!client->syncpts[0]) {
		err = -ENOMEM;
		goto free_channel;
	}

	err = tegra_drm_register_client(tegra, drm);
	if (err < 0)
		goto free_syncpt;

	/* Inherit the host1x DMA constraints used by the old DRM client. */
	client->dev->dma_parms = client->host->dma_parms;

	return 0;

free_syncpt:
	host1x_syncpt_put(client->syncpts[0]);
free_channel:
	host1x_channel_put(engine->channel);
detach:
	host1x_client_iommu_detach(client);

	return err;
}

static int nvdec_engine_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvdec_engine *engine = to_nvdec_engine(drm);
	int err;

	client->dev->dma_parms = NULL;

	err = tegra_drm_unregister_client(tegra, drm);
	if (err < 0)
		return err;

	pm_runtime_dont_use_autosuspend(client->dev);
	pm_runtime_force_suspend(client->dev);

	host1x_syncpt_put(client->syncpts[0]);
	host1x_channel_put(engine->channel);
	host1x_client_iommu_detach(client);

	engine->channel = NULL;

	if (client->group) {
		dma_unmap_single(engine->dev, engine->falcon.firmware.phys,
				 engine->falcon.firmware.size, DMA_TO_DEVICE);
		tegra_drm_free(tegra, engine->falcon.firmware.size,
			       engine->falcon.firmware.virt,
			       engine->falcon.firmware.iova);
	} else {
		dma_free_coherent(engine->dev, engine->falcon.firmware.size,
				  engine->falcon.firmware.virt,
				  engine->falcon.firmware.iova);
	}

	return 0;
}

static const struct host1x_client_ops nvdec_engine_client_ops = {
	.init = nvdec_engine_init,
	.exit = nvdec_engine_exit,
};

static int nvdec_engine_load_falcon_firmware(struct nvdec_engine *engine)
{
	struct host1x_client *client = &engine->client.base;
	struct tegra_drm *tegra = engine->client.drm;
	dma_addr_t iova;
	size_t size;
	void *virt;
	int err;

	if (engine->falcon.firmware.virt)
		return 0;

	err = falcon_read_firmware(&engine->falcon, engine->config->firmware);
	if (err < 0)
		return err;

	size = engine->falcon.firmware.size;

	if (!client->group) {
		virt = dma_alloc_coherent(engine->dev, size, &iova, GFP_KERNEL);
		if (!virt)
			return -ENOMEM;
	} else {
		virt = tegra_drm_alloc(tegra, size, &iova);
		if (IS_ERR(virt))
			return PTR_ERR(virt);
	}

	engine->falcon.firmware.virt = virt;
	engine->falcon.firmware.iova = iova;

	err = falcon_load_firmware(&engine->falcon);
	if (err < 0)
		goto cleanup;

	if (client->group) {
		dma_addr_t phys;

		phys = dma_map_single(engine->dev, virt, size, DMA_TO_DEVICE);
		err = dma_mapping_error(engine->dev, phys);
		if (err < 0)
			goto cleanup;

		engine->falcon.firmware.phys = phys;
	}

	return 0;

cleanup:
	if (!client->group)
		dma_free_coherent(engine->dev, size, virt, iova);
	else
		tegra_drm_free(tegra, size, virt, iova);

	return err;
}

static int nvdec_engine_runtime_resume(struct device *dev)
{
	struct nvdec_engine *engine = dev_get_drvdata(dev);
	int err;

	err = clk_bulk_prepare_enable(engine->num_clks, engine->clks);
	if (err < 0)
		return err;

	usleep_range(10, 20);

	if (engine->config->has_riscv) {
		err = nvdec_engine_boot_riscv(engine);
		if (err < 0)
			goto disable;
	} else {
		err = nvdec_engine_load_falcon_firmware(engine);
		if (err < 0)
			goto disable;

		err = nvdec_engine_boot_falcon(engine);
		if (err < 0)
			goto disable;
	}

	return 0;

disable:
	clk_bulk_disable_unprepare(engine->num_clks, engine->clks);
	return err;
}

static int nvdec_engine_runtime_suspend(struct device *dev)
{
	struct nvdec_engine *engine = dev_get_drvdata(dev);

	mutex_lock(&engine->recovery_lock);
	host1x_channel_stop(engine->channel);
	clk_bulk_disable_unprepare(engine->num_clks, engine->clks);
	mutex_unlock(&engine->recovery_lock);

	return 0;
}

const struct dev_pm_ops nvdec_engine_pm_ops = {
	SET_RUNTIME_PM_OPS(nvdec_engine_runtime_suspend,
			   nvdec_engine_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

/* Power-gating the NVDEC partition is what actually resets the falcon. */
static void nvdec_engine_recovery_work(struct work_struct *work)
{
	struct nvdec_engine *engine = container_of(work, struct nvdec_engine,
						    recovery_work);
	int err;

	wait_for_completion(&engine->idle);

	err = pm_runtime_get_sync(engine->dev);
	if (err >= 0)
		err = pm_runtime_put_sync_suspend(engine->dev);
	if (err < 0)
		dev_err(engine->dev, "engine reset failed: %d\n", err);

	mutex_lock(&engine->recovery_lock);
	engine->recovery_generation++;
	host1x_syncpt_recover(engine->client.base.syncpts[0]);
	mutex_unlock(&engine->recovery_lock);

	dev_warn(engine->dev, "recovered from a job timeout (%u)\n",
		 engine->recovery_generation);
}

void nvdec_engine_recover(struct nvdec_engine *engine)
{
	schedule_work(&engine->recovery_work);
}

static void nvdec_engine_job_release(struct host1x_job *job)
{
	struct nvdec_engine_job *engine_job = job->user_data;
	struct nvdec_engine *engine = engine_job->engine;
	void (*release)(struct host1x_job *job) = engine_job->release;

	job->release = release;
	job->user_data = engine_job->user_data;

	if (engine_job->complete)
		engine_job->complete(job, engine_job->complete_data);

	if (atomic_dec_and_test(&engine->active_jobs))
		complete_all(&engine->idle);

	kfree(engine_job);

	if (release)
		release(job);
}

int nvdec_engine_submit_job(struct nvdec_engine *engine, struct host1x_job *job,
			    nvdec_engine_job_complete_t complete, void *data)
{
	struct nvdec_engine_job *engine_job;
	int err;

	engine_job = kzalloc_obj(*engine_job);
	if (!engine_job)
		return -ENOMEM;

	engine_job->engine = engine;
	engine_job->release = job->release;
	engine_job->user_data = job->user_data;
	engine_job->complete = complete;
	engine_job->complete_data = data;

	mutex_lock(&engine->recovery_lock);
	job->release = nvdec_engine_job_release;
	job->user_data = engine_job;
	if (atomic_inc_return(&engine->active_jobs) == 1)
		reinit_completion(&engine->idle);

	err = host1x_job_submit(job);
	if (err) {
		atomic_dec(&engine->active_jobs);
		job->release = engine_job->release;
		job->user_data = engine_job->user_data;
		kfree(engine_job);
	}
	mutex_unlock(&engine->recovery_lock);

	return err;
}

struct nvdec_engine_map *nvdec_engine_map_get(struct nvdec_engine_map *map)
{
	kref_get(&map->ref);
	return map;
}

static void nvdec_engine_map_release(struct kref *ref)
{
	struct nvdec_engine_map *map = container_of(ref, struct nvdec_engine_map, ref);

	if (map->cpu)
		host1x_bo_munmap(&map->bo->base, map->cpu);
	if (map->pin)
		host1x_bo_unpin(map->pin);
	drm_gem_object_put(&map->bo->gem);
	kfree(map);
}

void nvdec_engine_map_put(struct nvdec_engine_map *map)
{
	if (map)
		kref_put(&map->ref, nvdec_engine_map_release);
}

/* NVDEC's methods carry 32 bits of address after the shift by 8. */
static int nvdec_map_bo(struct nvdec_engine *engine, struct nvdec_engine_map *map,
			struct tegra_bo *bo)
{
	struct host1x_bo_mapping *pin;

	map->bo = bo;

	/* The DMA API would return physical addresses; use the domain's IOVA. */
	if (bo->mm) {
		map->iova = bo->iova;
	} else {
		pin = host1x_bo_pin(engine->dev, &bo->base, map->direction, NULL);
		if (IS_ERR(pin))
			return PTR_ERR(pin);
		map->pin = pin;
		if (pin->chunks != 1)
			return -EINVAL;
		map->iova = pin->phys;
	}

	return upper_32_bits(map->iova) ? -ERANGE : 0;
}

static struct nvdec_engine_map *
nvdec_map_alloc(size_t size, enum dma_data_direction direction)
{
	struct nvdec_engine_map *map;

	map = kzalloc_obj(*map);
	if (!map)
		return NULL;

	kref_init(&map->ref);
	map->size = size;
	map->direction = direction;
	return map;
}

/* Every engine buffer is a tegra_bo in the host1x domain, as userspace's are. */
static struct nvdec_engine_map *
nvdec_buffer_create(struct nvdec_engine *engine, size_t size, bool cpu)
{
	struct nvdec_engine_map *map;
	struct tegra_bo *bo;
	int err;

	if (!engine || !size)
		return ERR_PTR(-EINVAL);

	map = nvdec_map_alloc(size, DMA_BIDIRECTIONAL);
	if (!map)
		return ERR_PTR(-ENOMEM);

	bo = tegra_bo_create(engine->client.drm->drm, size, 0);
	if (IS_ERR(bo)) {
		kfree(map);
		return ERR_CAST(bo);
	}

	err = nvdec_map_bo(engine, map, bo);
	if (!err && cpu) {
		map->cpu = host1x_bo_mmap(&bo->base);
		if (IS_ERR(map->cpu)) {
			err = PTR_ERR(map->cpu);
			map->cpu = NULL;
		}
	}
	if (err) {
		nvdec_engine_map_put(map);
		return ERR_PTR(err);
	}

	return map;
}

struct nvdec_engine_map *
nvdec_engine_surface_create(struct nvdec_engine *engine, size_t size)
{
	return nvdec_buffer_create(engine, size, false);
}

struct nvdec_engine_map *
nvdec_engine_map_create(struct nvdec_engine *engine, struct dma_buf *dmabuf,
			size_t size, enum dma_data_direction direction)
{
	struct nvdec_engine_map *map;
	struct drm_gem_object *gem;
	int err;

	if (!dmabuf || !size || size > dmabuf->size)
		return ERR_PTR(-EINVAL);

	map = nvdec_map_alloc(size, direction);
	if (!map)
		return ERR_PTR(-ENOMEM);

	gem = tegra_gem_prime_import(engine->client.drm->drm, dmabuf);
	if (IS_ERR(gem)) {
		kfree(map);
		return ERR_CAST(gem);
	}

	map->resv = gem->resv;
	err = nvdec_map_bo(engine, map, to_tegra_bo(gem));
	if (err) {
		nvdec_engine_map_put(map);
		return ERR_PTR(err);
	}

	return map;
}

int nvdec_engine_map_wait(struct nvdec_engine_map *map, bool write)
{
	long err;

	if (!map)
		return -EINVAL;
	if (!map->resv)
		return 0;

	err = dma_resv_wait_timeout(map->resv, dma_resv_usage_rw(write), true,
				    MAX_SCHEDULE_TIMEOUT);
	return err < 0 ? err : 0;
}

int nvdec_engine_map_add_fence(struct nvdec_engine_map *map,
			       struct dma_fence *fence, bool write)
{
	int err;

	if (!map || !fence)
		return -EINVAL;
	if (!map->resv)
		return 0;

	err = dma_resv_lock(map->resv, NULL);
	if (err)
		return err;
	err = dma_resv_reserve_fences(map->resv, 1);
	if (!err)
		dma_resv_add_fence(map->resv, fence,
				   write ? DMA_RESV_USAGE_WRITE : DMA_RESV_USAGE_READ);
	dma_resv_unlock(map->resv);

	return err;
}

static bool nvdec_map_is_valid(const struct nvdec_engine_map *map,
			       enum dma_data_direction direction, size_t size)
{
	if (!map || map->size < size)
		return false;

	return map->direction == direction || map->direction == DMA_BIDIRECTIONAL;
}

static int nvdec_copy_output(struct nvdec_engine_map *input, u32 offset,
			     const struct nvdec_engine_map *output,
			     size_t payload_size)
{
	struct dma_buf *dmabuf = output->bo->dma_buf;
	struct iosys_map vmap = { };
	int err;

	err = dma_buf_begin_cpu_access(dmabuf, DMA_TO_DEVICE);
	if (err)
		return err;

	err = dma_buf_vmap(dmabuf, &vmap);
	if (!err) {
		iosys_map_memcpy_from(input->cpu + offset, &vmap, 0,
				      payload_size);
		dma_buf_vunmap(dmabuf, &vmap);
	}

	dma_buf_end_cpu_access(dmabuf, DMA_TO_DEVICE);
	return err;
}

static const char *nvdec_fence_get_driver_name(struct dma_fence *fence)
{
	return "tegra-nvdec";
}

static const char *nvdec_fence_get_timeline_name(struct dma_fence *fence)
{
	return "tegra-nvdec-h264";
}

static void nvdec_fence_release(struct dma_fence *fence)
{
	struct nvdec_fence *h264_fence =
		container_of(fence, struct nvdec_fence, base);

	kfree(h264_fence);
}

static const struct dma_fence_ops nvdec_fence_ops = {
	.get_driver_name = nvdec_fence_get_driver_name,
	.get_timeline_name = nvdec_fence_get_timeline_name,
	.release = nvdec_fence_release,
};

static struct dma_fence *
nvdec_fence_create(struct nvdec_engine *engine)
{
	struct nvdec_fence *h264_fence;

	h264_fence = kzalloc_obj(*h264_fence);
	if (!h264_fence)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&h264_fence->lock);
	dma_fence_init(&h264_fence->base, &nvdec_fence_ops,
		       &h264_fence->lock, engine->h264_fence_context,
		       atomic64_inc_return(&engine->h264_fence_seqno));
	return &h264_fence->base;
}

static int nvdec_install_fences(struct nvdec_decode_job *hjob)
{
	unsigned int i;
	int err;

	err = nvdec_engine_map_add_fence(hjob->capture, hjob->fence, true);
	for (i = 0; !err && i < NVDEC_MAX_REFS; i++) {
		if (hjob->dpb[i])
			err = nvdec_engine_map_add_fence(hjob->dpb[i], hjob->fence,
							 false);
	}

	return err;
}

static int nvdec_h264_prepare_scratch(struct nvdec_decode_context *ctx,
				      const struct nvdec_h264_request *request)
{
	struct nvdec_engine_map *scratch;
	u32 coloc_per_picture, coloc_size, mbhist_size, history_size;
	u32 mbhist_offset, history_offset, size;

	if (ctx->scratch) {
		if (ctx->width_in_mbs != request->pic_width_in_mbs ||
		    ctx->height_in_mbs != request->frame_height_in_mbs)
			return -EBUSY;
		return 0;
	}

	if (check_mul_overflow((u32)ALIGN(request->frame_height_in_mbs, 2),
			       (u32)request->pic_width_in_mbs * 64, &coloc_per_picture) ||
	    coloc_per_picture < 63)
		return -EOVERFLOW;
	coloc_per_picture = ALIGN(coloc_per_picture - 63, 0x100);
	if (check_mul_overflow(coloc_per_picture, NVDEC_H264_MAX_PICTURES,
			       &coloc_size) ||
	    check_mul_overflow((u32)request->pic_width_in_mbs, 104U, &mbhist_size) ||
	    check_mul_overflow((u32)request->pic_width_in_mbs, 0x200U, &history_size))
		return -EOVERFLOW;

	mbhist_size = ALIGN(mbhist_size, 0x100);
	if (check_add_overflow(history_size, 0x1100U, &history_size))
		return -EOVERFLOW;
	history_size = ALIGN(history_size, 0x200);
	mbhist_offset = ALIGN(coloc_size, 0x100);
	if (check_add_overflow(mbhist_offset, mbhist_size, &history_offset))
		return -EOVERFLOW;
	history_offset = ALIGN(history_offset, 0x100);
	if (check_add_overflow(history_offset, history_size, &size))
		return -EOVERFLOW;
	size = ALIGN(size, SZ_4K);

	scratch = nvdec_engine_surface_create(ctx->engine, size);
	if (IS_ERR(scratch))
		return PTR_ERR(scratch);

	ctx->scratch = scratch;
	ctx->width_in_mbs = request->pic_width_in_mbs;
	ctx->height_in_mbs = request->frame_height_in_mbs;
	ctx->coloc_size = coloc_size;
	ctx->mbhist_offset = mbhist_offset;
	ctx->mbhist_size = mbhist_size;
	ctx->history_offset = history_offset;
	ctx->history_size = history_size;
	return 0;
}

static int nvdec_prepare_input(struct nvdec_decode_context *ctx, size_t size)
{
	struct nvdec_engine_map *input;

	if (ctx->input && ctx->input->size >= size)
		return 0;

	input = nvdec_buffer_create(ctx->engine, ALIGN(size, SZ_4K), true);
	if (IS_ERR(input))
		return PTR_ERR(input);

	if (ctx->input) {
		memcpy(input->cpu, ctx->input->cpu, ctx->staged);
		nvdec_engine_map_put(ctx->input);
	}
	ctx->input = input;
	return 0;
}

/*
 * HEVC carries a whole picture in one payload and the firmware finds its
 * slices by scanning start codes, so staging is a copy behind one zero byte.
 */
static int nvdec_hevc_stage(struct nvdec_decode_context *ctx,
			    struct nvdec_engine_map *output, u32 payload_size)
{
	u32 staged = payload_size + 1;
	int err;

	if (staged < payload_size ||
	    !nvdec_map_is_valid(output, DMA_TO_DEVICE, payload_size))
		return -EINVAL;

	err = nvdec_prepare_input(ctx, staged);
	if (err)
		return err;

	*(u8 *)ctx->input->cpu = 0;
	err = nvdec_copy_output(ctx->input, 1, output, payload_size);
	if (err)
		return err;

	if (memcmp(ctx->input->cpu + 1, "\x00\x00\x01", 3))
		return -EINVAL;

	ctx->slice_count = 1;
	ctx->staged = staged;
	return 0;
}

/* Slices are staged back to back and described by slice_count + 1 offsets. */
int nvdec_engine_stage_slice(struct nvdec_decode_context *ctx,
			     struct nvdec_engine_map *output,
			     u32 payload_size, bool first,
			     unsigned int max_slices)
{
	u32 staged;
	int err;

	if (!ctx || !output || payload_size < 3 || !max_slices)
		return -EINVAL;

	if (ctx->codec == NVDEC_CODEC_HEVC) {
		mutex_lock(&ctx->lock);
		err = nvdec_hevc_stage(ctx, output, payload_size);
		mutex_unlock(&ctx->lock);
		return err;
	}

	mutex_lock(&ctx->lock);
	if (first) {
		ctx->slice_count = 0;
		ctx->staged = 0;
	}
	staged = ctx->staged;

	if (ctx->slice_count >= max_slices ||
	    check_add_overflow(staged, payload_size, &staged) ||
	    staged > U32_MAX - 16 - SZ_256 ||
	    !nvdec_map_is_valid(output, DMA_TO_DEVICE, payload_size)) {
		err = -EINVAL;
		goto unlock;
	}

	if (ctx->slice_count + 1 > ctx->max_slices) {
		unsigned int want = max(ctx->max_slices * 2, 8U);
		u32 *offsets = krealloc_array(ctx->slice_offsets, want + 1,
					      sizeof(*offsets), GFP_KERNEL);

		if (!offsets) {
			err = -ENOMEM;
			goto unlock;
		}
		ctx->slice_offsets = offsets;
		ctx->max_slices = want;
	}

	/* The offset array and the 16-byte terminator follow the bitstream. */
	err = nvdec_prepare_input(ctx, ALIGN(staged + 16, SZ_256) +
				       (ctx->slice_count + 2) * sizeof(u32));
	if (err)
		goto unlock;

	err = nvdec_copy_output(ctx->input, staged - payload_size, output,
				payload_size);
	if (err)
		goto unlock;

	if (first && memcmp(ctx->input->cpu, "\x00\x00\x01", 3) &&
	    memcmp(ctx->input->cpu, "\x00\x00\x00\x01", 4)) {
		err = -EINVAL;
		goto unlock;
	}

	ctx->slice_offsets[ctx->slice_count++] = staged - payload_size;
	ctx->staged = staged;
unlock:
	mutex_unlock(&ctx->lock);
	return err;
}

void nvdec_engine_discard_slices(struct nvdec_decode_context *ctx)
{
	if (!ctx)
		return;

	mutex_lock(&ctx->lock);
	ctx->slice_count = 0;
	ctx->staged = 0;
	mutex_unlock(&ctx->lock);
}

/* The scratch is sized from the coded resolution, so a new size needs a new one. */
void nvdec_engine_context_reset(struct nvdec_decode_context *ctx)
{
	if (!ctx)
		return;

	nvdec_engine_discard_slices(ctx);
	mutex_lock(&ctx->lock);
	nvdec_engine_map_put(ctx->scratch);
	ctx->scratch = NULL;
	ctx->width_in_mbs = 0;
	ctx->height_in_mbs = 0;
	ctx->coded_width = 0;
	ctx->coded_height = 0;
	mutex_unlock(&ctx->lock);
}

/* The decode surface and the detile destination, as every codec lays them out. */
static int nvdec_validate_frame(struct device *dev, const struct nvdec_frame *f,
				unsigned int align,
				const struct nvdec_engine_map *surface,
				const struct nvdec_engine_map *capture,
				u32 *surface_size)
{
	u32 luma_size, chroma_size, dst_size;

	dev_dbg(dev,
		"frame: coded=%ux%u crop=%ux%u+%u+%u stride=%u coff=%u dst=%u/%u payload=%u\n",
		f->coded_width, f->coded_height, f->crop_width, f->crop_height,
		f->crop_left, f->crop_top, f->luma_stride, f->chroma_offset,
		f->dst_stride, f->dst_chroma_offset, f->output_payload_size);

	if (!f->coded_width || !f->coded_height ||
	    f->coded_width > 4096 || f->coded_height > 4096 ||
	    !IS_ALIGNED(f->coded_width, align) ||
	    !IS_ALIGNED(f->coded_height, align) || !f->output_payload_size) {
		dev_dbg(dev, "reject: coded size\n");
		return -EINVAL;
	}

	if (check_mul_overflow((u32)f->luma_stride,
			       (u32)ALIGN(f->coded_height, 32), &luma_size) ||
	    check_mul_overflow((u32)f->luma_stride,
			       (u32)ALIGN(f->coded_height / 2, 16), &chroma_size) ||
	    check_add_overflow(f->chroma_offset, chroma_size, surface_size) ||
	    f->chroma_offset < luma_size || !IS_ALIGNED(f->luma_stride, 16) ||
	    f->luma_stride < f->coded_width ||
	    !nvdec_map_is_valid(surface, DMA_BIDIRECTIONAL, *surface_size)) {
		dev_dbg(dev, "reject: surface geometry/map\n");
		return -EINVAL;
	}

	if (check_mul_overflow(f->dst_stride, (u32)f->crop_height / 2, &dst_size) ||
	    check_add_overflow(f->dst_chroma_offset, dst_size, &dst_size) ||
	    !f->crop_width || !f->crop_height ||
	    (f->crop_width | f->crop_height | f->crop_left | f->crop_top) & 1 ||
	    f->crop_left + f->crop_width > f->coded_width ||
	    f->crop_top + f->crop_height > f->coded_height ||
	    f->dst_chroma_offset < f->dst_stride * (u32)f->crop_height ||
	    !IS_ALIGNED(f->dst_stride, SZ_256) ||
	    f->dst_stride < f->crop_width ||
	    !nvdec_map_is_valid(capture, DMA_FROM_DEVICE, dst_size)) {
		dev_dbg(dev, "reject: detile destination\n");
		return -EINVAL;
	}

	return 0;
}

static int nvdec_h264_validate_request(struct device *dev,
				       const struct nvdec_h264_request *request,
				       struct nvdec_engine_map * const dpb[],
				       u32 surface_size)
{
	unsigned int i;

	dev_dbg(dev,
		"h264 request: profile=%u level=%u chroma=%u depth=%u/%u log2fn=%u poc=%u/%u maxref=%u flags=0x%x pps=0x%x wbipred=%u slicegroups=%u type=%u mbs=%ux%u slices=%u\n",
		request->profile_idc, request->level_idc,
		request->chroma_format_idc, request->bit_depth_luma_minus8,
		request->bit_depth_chroma_minus8,
		request->log2_max_frame_num_minus4, request->pic_order_cnt_type,
		request->log2_max_pic_order_cnt_lsb_minus4,
		request->max_num_ref_frames, request->flags, request->pps_flags,
		request->weighted_bipred_idc, request->num_slice_groups_minus1,
		request->slice_type, request->pic_width_in_mbs,
		request->frame_height_in_mbs, request->slice_count);

	if ((request->profile_idc != 66 && request->profile_idc != 77 &&
	     request->profile_idc != 100) ||
	    request->level_idc < 10 || request->level_idc > 51 ||
	    request->chroma_format_idc != 1 || request->bit_depth_luma_minus8 ||
	    request->bit_depth_chroma_minus8 ||
	    request->log2_max_frame_num_minus4 > 12 ||
	    request->pic_order_cnt_type > 2 ||
	    request->log2_max_pic_order_cnt_lsb_minus4 > 12 ||
	    request->max_num_ref_frames > NVDEC_H264_DPB_ENTRIES ||
	    !(request->flags & NVDEC_H264_REQ_FRAME_MBS_ONLY) ||
	    (request->flags & (NVDEC_H264_REQ_MBAFF |
			       NVDEC_H264_REQ_SEPARATE_COLOUR |
			       NVDEC_H264_REQ_FIELD |
			       NVDEC_H264_REQ_BOTTOM_FIELD)) ||
	    request->pic_width_in_mbs * 16 != request->frame.coded_width ||
	    request->frame_height_in_mbs * 16 != request->frame.coded_height ||
	    request->num_slice_groups_minus1 ||
	    request->weighted_bipred_idc > 2 ||
	    (request->slice_type != NVDEC_H264_SLICE_I &&
	     request->slice_type != NVDEC_H264_SLICE_P &&
	     request->slice_type != NVDEC_H264_SLICE_B) ||
	    request->frame.output_payload_size < 3 ||
	    request->frame.output_payload_size > U32_MAX - 16 ||
	    !request->slice_count ||
	    request->slice_count > (u32)request->pic_width_in_mbs *
				   request->frame_height_in_mbs) {
		dev_dbg(dev, "h264 reject: syntax/output\n");
		return -EINVAL;
	}

	if (request->slice_type != NVDEC_H264_SLICE_I &&
	    (request->flags & NVDEC_H264_REQ_IDR)) {
		dev_dbg(dev, "h264 reject: non-I slice in an IDR picture\n");
		return -EINVAL;
	}

	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (!request->dpb[i].valid) {
			if (dpb[i]) {
				dev_dbg(dev, "h264 reject: stray dpb map %u\n", i);
				return -EINVAL;
			}
			continue;
		}
		if (request->dpb[i].fields != 3 ||
		    !nvdec_map_is_valid(dpb[i], DMA_TO_DEVICE, surface_size)) {
			dev_dbg(dev, "h264 reject: dpb %u\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static int nvdec_surface_index(struct nvdec_decode_context *ctx,
			       struct nvdec_engine_map *map, u8 *index,
			       unsigned int slots)
{
	unsigned int i;

	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++) {
		if (ctx->surfaces[i].map == map) {
			*index = ctx->surfaces[i].picture_index;
			return *index < slots ? 0 : -ENOSPC;
		}
	}

	for (i = 0; i < slots; i++) {
		if (!ctx->surfaces[i].map) {
			ctx->surfaces[i].map = nvdec_engine_map_get(map);
			ctx->surfaces[i].picture_index = i;
			ctx->surfaces[i].dpb_slot = NVDEC_H264_DPB_ENTRIES;
			*index = i;
			return 0;
		}
	}

	return -ENOSPC;
}

/* A slot names the same picture for as long as that picture is a reference. */
static int nvdec_h264_dpb_slot(struct nvdec_decode_context *ctx,
			       struct nvdec_engine_map *map, u8 *slot)
{
	struct nvdec_pool_surface *entry = NULL;
	unsigned long used = 0;
	unsigned int i;

	for (i = 0; i < NVDEC_MAX_PICTURES; i++) {
		if (!ctx->surfaces[i].map)
			continue;
		if (ctx->surfaces[i].map == map)
			entry = &ctx->surfaces[i];
		else if (ctx->surfaces[i].dpb_slot < NVDEC_H264_DPB_ENTRIES)
			used |= BIT(ctx->surfaces[i].dpb_slot);
	}
	if (!entry)
		return -EINVAL;

	if (entry->dpb_slot >= NVDEC_H264_DPB_ENTRIES) {
		i = find_first_zero_bit(&used, NVDEC_H264_DPB_ENTRIES);
		if (i >= NVDEC_H264_DPB_ENTRIES) {
			dev_dbg(ctx->engine->dev, "no free dpb slot\n");
			return -ENOSPC;
		}
		entry->dpb_slot = i;
	}
	*slot = entry->dpb_slot;
	return 0;
}

static u32 nvdec_h264_dpb_flags(const struct nvdec_h264_request *request,
				unsigned int slot, u8 picture_index)
{
	const typeof(request->dpb[0]) *dpb = &request->dpb[slot];
	u32 marking = dpb->long_term ? 2 : 1;

	return picture_index | (picture_index << 7) | (3 << 12) |
		(dpb->long_term << 14) | (marking << 17) | (marking << 21);
}

static void nvdec_h264_setup_dpb(struct nvdec_h264_setup *setup,
				 const struct nvdec_h264_request *request,
				  const u8 picture_indices[NVDEC_H264_DPB_ENTRIES],
				  const u8 dpb_slots[NVDEC_H264_DPB_ENTRIES])
{
	unsigned int i;

	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		const typeof(request->dpb[0]) *dpb = &request->dpb[i];
		struct nvdec_h264_dpb_entry *entry;

		if (!dpb->valid)
			continue;
		entry = &setup->dpb[dpb_slots[i]];
		entry->flags = cpu_to_le32(nvdec_h264_dpb_flags(request, i,
								picture_indices[i]));
		entry->field_order_cnt[0] = cpu_to_le32(dpb->top_field_order_cnt);
		entry->field_order_cnt[1] = cpu_to_le32(dpb->bottom_field_order_cnt);
		entry->frame_idx = cpu_to_le32(dpb->frame_num);
	}
}

static void nvdec_h264_fill_setup(struct nvdec_decode_job *hjob)
{
	const struct nvdec_h264_request *request = &hjob->req.h264;
	u8 current_index = hjob->picture_index;
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct nvdec_h264_setup *setup = hjob->state->cpu;
	u32 picture_flags, current_picture;

	setup->stream_len = cpu_to_le32(request->frame.output_payload_size + 16);
	setup->slice_count = cpu_to_le32(request->slice_count);
	setup->mbhist_buffer_size = cpu_to_le32(ctx->mbhist_size);
	setup->log2_max_pic_order_cnt_lsb_minus4 =
		cpu_to_le32(request->log2_max_pic_order_cnt_lsb_minus4);
	setup->delta_pic_order_always_zero_flag =
		cpu_to_le32(!!(request->flags & NVDEC_H264_REQ_DELTA_POC_ZERO));
	setup->frame_mbs_only_flag = cpu_to_le32(1);
	setup->pic_width_in_mbs = cpu_to_le32(request->pic_width_in_mbs);
	setup->frame_height_in_mbs = cpu_to_le32(request->frame_height_in_mbs);
	setup->entropy_coding_mode_flag =
		cpu_to_le32(!!(request->pps_flags & NVDEC_H264_PPS_ENTROPY_CODING));
	setup->pic_order_present_flag =
		cpu_to_le32(!!(request->pps_flags & NVDEC_H264_PPS_PIC_ORDER_PRESENT));
	setup->num_ref_idx_l0_active_minus1 =
		cpu_to_le32(request->num_ref_idx_l0_active_minus1);
	setup->num_ref_idx_l1_active_minus1 =
		cpu_to_le32(request->num_ref_idx_l1_active_minus1);
	setup->deblocking_filter_control_present_flag =
		cpu_to_le32(!!(request->pps_flags & NVDEC_H264_PPS_DEBLOCK));
	setup->redundant_pic_cnt_present_flag =
		cpu_to_le32(!!(request->pps_flags & NVDEC_H264_PPS_REDUNDANT));
	setup->transform_8x8_mode_flag =
		cpu_to_le32(!!(request->pps_flags & NVDEC_H264_PPS_TRANSFORM_8X8));
	setup->pitch_luma = cpu_to_le32(request->frame.luma_stride);
	setup->pitch_chroma = cpu_to_le32(request->frame.luma_stride);
	setup->history_buffer_size = cpu_to_le32(ctx->history_size / 256);

	picture_flags = !!(request->flags & NVDEC_H264_REQ_DIRECT_8X8) << 1;
	picture_flags |= !!(request->pps_flags & NVDEC_H264_PPS_WEIGHTED_PRED) << 2;
	picture_flags |= !!(request->pps_flags & NVDEC_H264_PPS_CONSTRAINED_INTRA) << 3;
	picture_flags |= !!request->nal_ref_idc << 4;
	picture_flags |= request->log2_max_frame_num_minus4 << 8;
	picture_flags |= request->chroma_format_idc << 12;
	picture_flags |= request->pic_order_cnt_type << 14;
	picture_flags |= ((u32)request->pic_init_qp_minus26 & 0x3f) << 16;
	picture_flags |= ((u32)request->chroma_qp_index_offset & 0x1f) << 22;
	picture_flags |= ((u32)request->second_chroma_qp_index_offset & 0x1f) << 27;
	setup->picture_flags = cpu_to_le32(picture_flags);

	current_picture = request->weighted_bipred_idc & 0x3;
	current_picture |= current_index << 2;
	current_picture |= current_index << 9;
	current_picture |= (u32)request->frame_num << 14;
	setup->current_picture = cpu_to_le32(current_picture);
	setup->current_field_order_cnt[0] = cpu_to_le32(request->top_field_order_cnt);
	setup->current_field_order_cnt[1] = cpu_to_le32(request->bottom_field_order_cnt);
	/* x264 and JM differ on Intra_8x8 reference filtering; match nvtegra. */
	setup->lossless_flags = cpu_to_le32(BIT(0));
	nvdec_h264_setup_dpb(setup, request, hjob->picture_indices, hjob->dpb_slots);
	memcpy(setup->scaling_4x4, request->scaling_4x4, sizeof(setup->scaling_4x4));
	memcpy(setup->scaling_8x8, request->scaling_8x8, sizeof(setup->scaling_8x8));
}

static void nvdec_h264_debug_job(struct nvdec_decode_job *hjob)
{
	char dpb[128] = "";
	unsigned int i, len = 0;

	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (!hjob->req.h264.dpb[i].valid)
			continue;
		len += scnprintf(dpb + len, sizeof(dpb) - len, "%s%u:%u@%u",
				 len ? "," : "", i, hjob->picture_indices[i],
				 hjob->dpb_slots[i]);
	}

	dev_dbg(hjob->ctx->engine->dev, "h264 dpb=%s current=%u\n",
		dpb[0] ? dpb : "none", hjob->picture_index);
}

static int nvdec_h264_prepare(struct nvdec_decode_job *hjob)
{
	struct nvdec_h264_request *request = &hjob->req.h264;
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct device *dev = ctx->engine->dev;
	u32 surface_size;
	unsigned int i;
	int err;

	request->slice_count = ctx->slice_count;
	err = nvdec_validate_frame(dev, &request->frame, 16, hjob->surface,
				   hjob->capture, &surface_size);
	if (!err)
		err = nvdec_h264_validate_request(dev, request, hjob->dpb,
						  surface_size);
	if (!err)
		err = nvdec_h264_prepare_scratch(ctx, request);
	if (err)
		return err;
	hjob->scratch = nvdec_engine_map_get(ctx->scratch);

	err = nvdec_surface_index(ctx, hjob->surface, &hjob->picture_index,
				  NVDEC_H264_MAX_PICTURES);
	if (err)
		return err;
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (!request->dpb[i].valid)
			continue;
		err = nvdec_surface_index(ctx, hjob->dpb[i],
					  &hjob->picture_indices[i],
					  NVDEC_H264_MAX_PICTURES);
		if (!err)
			err = nvdec_h264_dpb_slot(ctx, hjob->dpb[i],
						  &hjob->dpb_slots[i]);
		if (err)
			return err;
	}

	hjob->num_pictures = NVDEC_H264_MAX_PICTURES;
	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++)
		hjob->pictures[i] = hjob->surface;
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (request->dpb[i].valid)
			hjob->pictures[hjob->picture_indices[i]] = hjob->dpb[i];
	}

	return 0;
}

static void nvdec_h264_emit(struct nvdec_decode_job *hjob,
			    struct falcon_gather *g)
{
	struct nvdec_decode_context *ctx = hjob->ctx;

	falcon_gather_address(g, NVDEC_METHOD_SLICE_OFFSETS,
			      hjob->input->iova + hjob->slice_offsets_off);
	falcon_gather_address(g, NVDEC_METHOD_COLOC, hjob->scratch->iova);
	falcon_gather_address(g, NVDEC_H264_METHOD_MBHIST,
			      hjob->scratch->iova + ctx->mbhist_offset);
	falcon_gather_address(g, NVDEC_METHOD_HISTORY,
			      hjob->scratch->iova + ctx->history_offset);
	nvdec_h264_debug_job(hjob);
}

static const u8 nvdec_h264_termination[16] = {
	0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
};

static const struct nvdec_codec_ops nvdec_h264_ops = {
	.application = 3,
	.setup_size = NVDEC_H264_SETUP_SIZE,
	.status_offset = NVDEC_H264_STATUS_OFFSET,
	.vic_config = 0x400,
	.termination = nvdec_h264_termination,
	.prepare = nvdec_h264_prepare,
	.fill = nvdec_h264_fill_setup,
	.emit = nvdec_h264_emit,
};

static void nvdec_context_release(struct kref *ref)
{
	struct nvdec_decode_context *ctx = container_of(ref, struct nvdec_decode_context,
						       ref);
	unsigned int i;

	for (i = 0; i < NVDEC_MAX_PICTURES; i++)
		nvdec_engine_map_put(ctx->surfaces[i].map);
	nvdec_engine_map_put(ctx->input);
	nvdec_engine_map_put(ctx->scratch);
	kfree(ctx->slice_offsets);
	kfree(ctx);
}

/* Drops everything a job holds, whether or not it ever reached the engine. */
static void nvdec_job_free(struct nvdec_decode_job *hjob, bool error)
{
	unsigned int i;

	if (hjob->runtime_ref) {
		pm_runtime_mark_last_busy(hjob->ctx->engine->dev);
		pm_runtime_put_autosuspend(hjob->ctx->engine->dev);
	}
	if (hjob->vic_runtime_ref) {
		struct device *dev = vic_engine_device(hjob->vic);

		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_autosuspend(dev);
	}
	if (hjob->fence) {
		if (error)
			dma_fence_set_error(hjob->fence, -EIO);
		dma_fence_signal(hjob->fence);
		dma_fence_put(hjob->fence);
	}
	if (hjob->submitted && hjob->complete)
		hjob->complete(hjob->complete_data, error);
	nvdec_engine_map_put(hjob->gather);
	nvdec_engine_map_put(hjob->input);
	nvdec_engine_map_put(hjob->state);
	nvdec_engine_map_put(hjob->scratch);
	nvdec_engine_map_put(hjob->surface);
	nvdec_engine_map_put(hjob->capture);
	for (i = 0; i < NVDEC_MAX_REFS; i++)
		nvdec_engine_map_put(hjob->dpb[i]);
	kref_put(&hjob->ctx->ref, nvdec_context_release);
	kfree(hjob);
}

static void nvdec_decode_job_release(struct host1x_job *job)
{
	struct nvdec_decode_job *hjob = job->user_data;
	struct nvdec_engine *engine = hjob->ctx->engine;
	bool cancelled = job->cancelled;

	mutex_lock(&hjob->ctx->lock);
	hjob->ctx->in_flight = false;
	mutex_unlock(&hjob->ctx->lock);
	nvdec_job_free(hjob, cancelled);
	if (cancelled)
		nvdec_engine_recover(engine);
}

struct nvdec_decode_context *
nvdec_engine_context_create(struct nvdec_engine *engine, enum nvdec_codec codec)
{
	struct nvdec_decode_context *ctx;

	/* Engine buffers live in the host1x domain, so the engine must too. */
	if (engine->client.drm->domain && !engine->client.base.group)
		return ERR_PTR(-ENODEV);

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	kref_init(&ctx->ref);
	ctx->engine = engine;
	ctx->codec = codec;
	mutex_init(&ctx->lock);
	return ctx;
}

void nvdec_engine_context_destroy(struct nvdec_decode_context *ctx)
{
	if (!ctx)
		return;

	kref_put(&ctx->ref, nvdec_context_release);
}

void nvdec_engine_context_release_surface(struct nvdec_decode_context *ctx,
					  struct nvdec_engine_map *surface)
{
	unsigned int i;

	if (!ctx || !surface)
		return;

	mutex_lock(&ctx->lock);
	for (i = 0; i < NVDEC_MAX_PICTURES; i++) {
		if (ctx->surfaces[i].map != surface)
			continue;
		nvdec_engine_map_put(ctx->surfaces[i].map);
		ctx->surfaces[i].map = NULL;
		break;
	}
	mutex_unlock(&ctx->lock);
}

static void nvdec_fill_detile(struct nvdec_decode_job *hjob)
{
	const struct nvdec_frame *f = &hjob->req.frame;
	struct vic_detile_params params = {
		.width = f->coded_width,
		.height = f->coded_height,
		.left = f->crop_left,
		.top = f->crop_top,
		.out_width = f->crop_width,
		.out_height = f->crop_height,
		.src_stride = f->luma_stride,
		.dst_stride = f->dst_stride,
	};

	vic_engine_fill_detile_config(hjob->state->cpu + hjob->ops->vic_config,
				      &params);
}

static int nvdec_build_gather(struct nvdec_decode_job *hjob)
{
	const struct nvdec_codec_ops *ops = hjob->ops;
	const struct nvdec_frame *f = &hjob->req.frame;
	struct nvdec_engine *engine = hjob->ctx->engine;
	u32 syncpt = host1x_syncpt_id(engine->client.base.syncpts[0]);
	dma_addr_t state = hjob->state->iova;
	struct falcon_gather g = {
		.words = hjob->gather->cpu,
		.size = NVDEC_GATHER_WORDS,
	};
	unsigned int i;

	falcon_gather_method(&g, NVDEC_METHOD_APPLICATION, ops->application);
	falcon_gather_method(&g, NVDEC_METHOD_CONTROL, 0x50 | ops->application);
	falcon_gather_method(&g, NVDEC_METHOD_PICTURE_INDEX, hjob->picture_index);
	falcon_gather_address(&g, NVDEC_METHOD_SETUP, state);
	falcon_gather_address(&g, NVDEC_METHOD_INPUT, hjob->input->iova);
	falcon_gather_address(&g, NVDEC_METHOD_STATUS, state + ops->status_offset);
	ops->emit(hjob, &g);
	for (i = 0; i < hjob->num_pictures; i++) {
		falcon_gather_address(&g, NVDEC_METHOD_LUMA + i,
				      hjob->pictures[i]->iova);
		falcon_gather_address(&g, NVDEC_METHOD_CHROMA + i,
				      hjob->pictures[i]->iova + f->chroma_offset);
	}
	falcon_gather_method(&g, NVDEC_METHOD_EXECUTE, 0x100);
	falcon_gather_op_done(&g, syncpt);
	hjob->vic_offset = g.count;

	vic_engine_emit_detile(&g, state + ops->vic_config, hjob->surface->iova,
			       hjob->surface->iova + f->chroma_offset,
			       hjob->capture->iova,
			       hjob->capture->iova + f->dst_chroma_offset);
	falcon_gather_op_done(&g, syncpt);

	print_hex_dump_debug("nvdec setup: ", DUMP_PREFIX_OFFSET, 16, 4,
			     hjob->state->cpu, ops->setup_size, false);
	return g.err;
}

/* One host1x job: NVDEC and OP_DONE, then a wait into VIC, detile, OP_DONE. */
static int nvdec_launch_job(struct nvdec_decode_job *hjob,
			    struct dma_fence **fence)
{
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct nvdec_engine *engine = ctx->engine;
	struct host1x_bo *gather = &hjob->gather->bo->base;
	struct host1x_job *job;
	int err;

	job = host1x_job_alloc(engine->channel, 3, 0, true);
	if (!job)
		return -ENOMEM;

	job->client = &engine->client.base;
	job->class = HOST1X_CLASS_NVDEC;
	job->serialize = true;
	job->syncpt = host1x_syncpt_get(engine->client.base.syncpts[0]);
	job->syncpt_incrs = 2;	/* NVDEC OP_DONE, then VIC OP_DONE */
	job->timeout = 10000;
	host1x_job_add_gather(job, gather, hjob->vic_offset, 0);
	host1x_job_add_wait(job, host1x_syncpt_id(job->syncpt), 1, true,
			    HOST1X_CLASS_VIC);
	host1x_job_add_gather(job, gather, VIC_DETILE_WORDS + 2,
			      hjob->vic_offset * sizeof(u32));

	err = pm_runtime_resume_and_get(engine->dev);
	if (err < 0)
		goto put_job;
	hjob->runtime_ref = true;
	err = pm_runtime_resume_and_get(vic_engine_device(hjob->vic));
	if (err < 0)
		goto put_job;
	hjob->vic_runtime_ref = true;
	err = host1x_job_pin(job, engine->dev);
	if (err)
		goto put_job;

	/* Once submitted, the job's release frees hjob instead of the caller. */
	job->release = nvdec_decode_job_release;
	job->user_data = hjob;
	ctx->in_flight = true;
	hjob->submitted = true;
	err = nvdec_engine_submit_job(engine, job, NULL, NULL);
	if (err) {
		job->release = NULL;
		ctx->in_flight = false;
		hjob->submitted = false;
		host1x_job_unpin(job);
		goto put_job;
	}
	*fence = dma_fence_get(hjob->fence);

put_job:
	host1x_job_put(job);
	return err;
}

static int nvdec_hevc_prepare_scratch(struct nvdec_decode_context *ctx,
				      const struct nvdec_hevc_request *request)
{
	u32 aligned_width, aligned_height, colmv, coloc, filter, size;
	struct nvdec_engine_map *scratch;

	if (ctx->scratch) {
		if (ctx->coded_width != request->frame.coded_width ||
		    ctx->coded_height != request->frame.coded_height)
			return -EBUSY;
		return 0;
	}

	aligned_width = ALIGN(request->frame.coded_width, NVDEC_HEVC_CTU_SIZE);
	aligned_height = ALIGN(request->frame.coded_height, NVDEC_HEVC_CTU_SIZE);

	if (check_mul_overflow(aligned_width, aligned_height, &colmv))
		return -EOVERFLOW;
	colmv /= 16;
	if (check_mul_overflow(colmv, NVDEC_HEVC_MAX_PICTURES + 1U, &coloc) ||
	    check_mul_overflow(aligned_height,
			       (u32)(NVDEC_HEVC_FILTER_PER_ROW +
				     NVDEC_HEVC_SAO_PER_ROW +
				     NVDEC_HEVC_BSD_PER_ROW), &filter))
		return -EOVERFLOW;

	coloc = ALIGN(coloc, SZ_256);
	if (check_add_overflow(coloc, filter, &size))
		return -EOVERFLOW;
	size = ALIGN(size, SZ_4K);

	scratch = nvdec_engine_surface_create(ctx->engine, size);
	if (IS_ERR(scratch))
		return PTR_ERR(scratch);

	ctx->scratch = scratch;
	ctx->coded_width = request->frame.coded_width;
	ctx->coded_height = request->frame.coded_height;
	ctx->colmv_size = colmv;
	ctx->filter_offset = coloc;
	ctx->sao_offset = NVDEC_HEVC_FILTER_PER_ROW * aligned_height;
	ctx->bsd_offset = (NVDEC_HEVC_FILTER_PER_ROW + NVDEC_HEVC_SAO_PER_ROW) *
			  aligned_height;
	return 0;
}

static int nvdec_hevc_validate_request(struct device *dev,
				       const struct nvdec_hevc_request *request,
				       struct nvdec_engine_map * const dpb[],
				       u32 surface_size)
{
	unsigned int i, tiles;

	dev_dbg(dev,
		"hevc request: %ux%u ctb=%ux%u depth=%u cb=%u/%u tb=%u/%u qp=%u sps=0x%x pps=0x%x tiles=%ux%u refs=%u poc=%d skip=%u\n",
		request->pic_width_in_luma_samples,
		request->pic_height_in_luma_samples, request->ctb_width,
		request->ctb_height, request->bit_depth,
		request->log2_min_luma_coding_block_size,
		request->log2_max_luma_coding_block_size,
		request->log2_min_transform_block_size,
		request->log2_max_transform_block_size, request->init_qp,
		request->sps_flags, request->pps_flags, request->num_tile_columns,
		request->num_tile_rows, request->num_ref_frames,
		request->pic_order_cnt_val, request->sw_hdr_skip_length);

	if (request->bit_depth != 8 ||
	    !request->pic_width_in_luma_samples ||
	    !request->pic_height_in_luma_samples ||
	    request->pic_width_in_luma_samples > request->frame.coded_width ||
	    request->pic_height_in_luma_samples > request->frame.coded_height ||
	    request->log2_min_luma_coding_block_size < 3 ||
	    request->log2_max_luma_coding_block_size > 6 ||
	    request->log2_min_luma_coding_block_size >
	    request->log2_max_luma_coding_block_size ||
	    request->log2_min_transform_block_size < 2 ||
	    request->log2_max_transform_block_size > 5 ||
	    request->log2_min_transform_block_size >
	    request->log2_max_transform_block_size ||
	    request->max_transform_hierarchy_depth_inter > 4 ||
	    request->max_transform_hierarchy_depth_intra > 4 ||
	    request->init_qp > 127 || request->diff_cu_qp_delta_depth > 3 ||
	    request->log2_parallel_merge_level < 2 ||
	    request->log2_parallel_merge_level > 4 ||
	    request->num_extra_slice_header_bits > 7 ||
	    !request->num_ref_idx_l0_default_active ||
	    request->num_ref_idx_l0_default_active > 15 ||
	    !request->num_ref_idx_l1_default_active ||
	    request->num_ref_idx_l1_default_active > 15 ||
	    request->pps_cb_qp_offset < -12 || request->pps_cb_qp_offset > 12 ||
	    request->pps_cr_qp_offset < -12 || request->pps_cr_qp_offset > 12 ||
	    request->pps_beta_offset < -12 || request->pps_beta_offset > 12 ||
	    request->pps_tc_offset < -12 || request->pps_tc_offset > 12 ||
	    request->frame.output_payload_size < 4 ||
	    request->num_active_dpb_entries > NVDEC_HEVC_DPB_ENTRIES ||
	    request->num_ref_frames > NVDEC_HEVC_DPB_ENTRIES) {
		dev_dbg(dev, "hevc reject: syntax\n");
		return -EINVAL;
	}

	if (!request->num_tile_columns || request->num_tile_columns > 20 ||
	    !request->num_tile_rows || request->num_tile_rows > 22 ||
	    check_mul_overflow((unsigned int)request->num_tile_columns,
			       (unsigned int)request->num_tile_rows, &tiles) ||
	    tiles * 2 * sizeof(u16) > NVDEC_HEVC_TILES_OFFSET ||
	    !request->ctb_width || !request->ctb_height) {
		dev_dbg(dev, "hevc reject: tiles\n");
		return -EINVAL;
	}

	for (i = 0; i < NVDEC_HEVC_DPB_ENTRIES; i++) {
		if (!request->dpb[i].valid) {
			if (dpb[i]) {
				dev_dbg(dev, "hevc reject: stray dpb map %u\n", i);
				return -EINVAL;
			}
			continue;
		}
		if (i >= request->num_active_dpb_entries ||
		    !nvdec_map_is_valid(dpb[i], DMA_TO_DEVICE, surface_size)) {
			dev_dbg(dev, "hevc reject: dpb %u\n", i);
			return -EINVAL;
		}
	}

	if (request->num_poc_st_curr_before > NVDEC_HEVC_DPB_ENTRIES ||
	    request->num_poc_st_curr_after > NVDEC_HEVC_DPB_ENTRIES ||
	    request->num_poc_lt_curr > NVDEC_HEVC_DPB_ENTRIES) {
		dev_dbg(dev, "hevc reject: reference set size\n");
		return -EINVAL;
	}

	for (i = 0; i < request->num_poc_st_curr_before; i++)
		if (!request->dpb[request->poc_st_curr_before[i]].valid)
			goto bad_rps;
	for (i = 0; i < request->num_poc_st_curr_after; i++)
		if (!request->dpb[request->poc_st_curr_after[i]].valid)
			goto bad_rps;
	for (i = 0; i < request->num_poc_lt_curr; i++)
		if (!request->dpb[request->poc_lt_curr[i]].valid)
			goto bad_rps;

	return 0;

bad_rps:
	dev_dbg(dev, "hevc reject: reference set names an inactive dpb entry\n");
	return -EINVAL;
}

/* An unused slot still needs a surface and a POC difference: the first reference. */
static int nvdec_hevc_scratch_entry(const struct nvdec_hevc_request *r)
{
	if (r->num_poc_st_curr_before)
		return r->poc_st_curr_before[0];
	if (r->num_poc_st_curr_after)
		return r->poc_st_curr_after[0];
	if (r->num_poc_lt_curr)
		return r->poc_lt_curr[0];
	return -1;
}

static void nvdec_hevc_fill_scaling_list(void *mem,
					 const struct nvdec_hevc_request *r)
{
	struct nvdec_hevc_scaling_list *list = mem;

	memcpy(list->dc_16x16, r->scaling_dc_16x16, sizeof(list->dc_16x16));
	memcpy(list->dc_32x32, r->scaling_dc_32x32, sizeof(list->dc_32x32));
	memcpy(list->list_4x4, r->scaling_4x4, sizeof(list->list_4x4));
	memcpy(list->list_8x8, r->scaling_8x8, sizeof(list->list_8x8));
	memcpy(list->list_16x16, r->scaling_16x16, sizeof(list->list_16x16));
	memcpy(list->list_32x32, r->scaling_32x32, sizeof(list->list_32x32));
}

/* Per-tile sizes in CTBs, then the boundaries in 16-pixel units. */
static void nvdec_hevc_fill_tile_sizes(void *mem,
				       const struct nvdec_hevc_request *r)
{
	unsigned int shift = r->log2_max_luma_coding_block_size - 4;
	__le16 *bounds = (__le16 *)mem + 0x380;
	__le16 *sizes = mem;
	unsigned int i, j;
	u32 sum;

	if (!(r->pps_flags & NVDEC_HEVC_PPS_TILES)) {
		sizes[0] = cpu_to_le16(r->ctb_width);
		sizes[1] = cpu_to_le16(r->ctb_height);
		return;
	}

	if (r->pps_flags & NVDEC_HEVC_PPS_UNIFORM_SPACING) {
		for (i = 0; i < r->num_tile_columns; i++)
			*bounds++ = cpu_to_le16(((i + 1) * r->ctb_width /
						 r->num_tile_columns) << shift);
		for (i = 0; i < r->num_tile_rows; i++)
			*bounds++ = cpu_to_le16(((i + 1) * r->ctb_height /
						 r->num_tile_rows) << shift);
	} else {
		for (i = 0, sum = 0; i < r->num_tile_columns; i++) {
			sum += r->column_width[i];
			*bounds++ = cpu_to_le16(sum << shift);
		}
		for (i = 0, sum = 0; i < r->num_tile_rows; i++) {
			sum += r->row_height[i];
			*bounds++ = cpu_to_le16(sum << shift);
		}
	}

	for (i = 0; i < r->num_tile_rows; i++) {
		for (j = 0; j < r->num_tile_columns; j++) {
			*sizes++ = cpu_to_le16(r->column_width[j]);
			*sizes++ = cpu_to_le16(r->row_height[i]);
		}
	}
}

/* The three RPS classes concatenated, repeating to fill all 16 entries. */
static void nvdec_hevc_fill_reflist(u8 list[NVDEC_HEVC_MAX_PICTURES],
				    const u8 *order, unsigned int count)
{
	unsigned int i;

	for (i = 0; count && i < NVDEC_HEVC_MAX_PICTURES; i++)
		list[i] = order[i % count];
}

static void nvdec_hevc_fill_setup(struct nvdec_decode_job *hjob)
{
	const struct nvdec_hevc_request *r = &hjob->req.hevc;
	const u8 *picture_indices = hjob->picture_indices;
	u8 current_index = hjob->picture_index;
	int entry = nvdec_hevc_scratch_entry(r);
	s8 scratch_diff_poc = entry < 0 ? 0 :
		clamp_t(int, r->pic_order_cnt_val - r->dpb[entry].pic_order_cnt_val,
			S8_MIN, S8_MAX);
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct nvdec_hevc_setup *setup = hjob->state->cpu;
	u8 order0[3 * NVDEC_HEVC_DPB_ENTRIES];
	u8 order1[3 * NVDEC_HEVC_DPB_ENTRIES];
	unsigned int i, n = 0, m = 0;
	u32 mask = BIT(current_index);
	u32 word;

	setup->stream_len = cpu_to_le32(r->frame.output_payload_size);
	setup->surface_format = cpu_to_le32(FIELD_PREP(NVDEC_HEVC_SURFACE_START_CODE, 1));
	setup->framestride[0] = cpu_to_le32(r->frame.luma_stride);
	setup->framestride[1] = cpu_to_le32(r->frame.luma_stride);
	setup->coloc_buffer_size = cpu_to_le32(ctx->colmv_size / 256);
	setup->sao_buffer_offset = cpu_to_le32(ctx->sao_offset / 256);
	setup->bsd_control_offset = cpu_to_le32(ctx->bsd_offset / 256);
	setup->pic_width_in_luma_samples =
		cpu_to_le16(r->pic_width_in_luma_samples);
	setup->pic_height_in_luma_samples =
		cpu_to_le16(r->pic_height_in_luma_samples);

	word = FIELD_PREP(NVDEC_HEVC_GEOM_CHROMA_FORMAT, 1);
	word |= FIELD_PREP(NVDEC_HEVC_GEOM_BIT_DEPTH_LUMA, r->bit_depth);
	word |= FIELD_PREP(NVDEC_HEVC_GEOM_BIT_DEPTH_CHROMA, r->bit_depth);
	word |= FIELD_PREP(NVDEC_HEVC_GEOM_LOG2_MIN_CB,
			   r->log2_min_luma_coding_block_size);
	word |= FIELD_PREP(NVDEC_HEVC_GEOM_LOG2_MAX_CB,
			   r->log2_max_luma_coding_block_size);
	word |= FIELD_PREP(NVDEC_HEVC_GEOM_LOG2_MIN_TB,
			   r->log2_min_transform_block_size);
	word |= FIELD_PREP(NVDEC_HEVC_GEOM_LOG2_MAX_TB,
			   r->log2_max_transform_block_size);
	setup->sps_geometry = cpu_to_le32(word);

	word = FIELD_PREP(NVDEC_HEVC_SPS_HIER_INTER,
			  r->max_transform_hierarchy_depth_inter);
	word |= FIELD_PREP(NVDEC_HEVC_SPS_HIER_INTRA,
			   r->max_transform_hierarchy_depth_intra);
	if (r->sps_flags & NVDEC_HEVC_SPS_SCALING_LIST)
		word |= NVDEC_HEVC_SPS_SCALING_LIST_EN;
	if (r->sps_flags & NVDEC_HEVC_SPS_AMP)
		word |= NVDEC_HEVC_SPS_AMP_EN;
	if (r->sps_flags & NVDEC_HEVC_SPS_SAO)
		word |= NVDEC_HEVC_SPS_SAO_EN;
	if (r->sps_flags & NVDEC_HEVC_SPS_PCM) {
		word |= NVDEC_HEVC_SPS_PCM_EN;
		word |= FIELD_PREP(NVDEC_HEVC_SPS_PCM_DEPTH_LUMA,
				   r->pcm_sample_bit_depth_luma);
		word |= FIELD_PREP(NVDEC_HEVC_SPS_PCM_DEPTH_CHROMA,
				   r->pcm_sample_bit_depth_chroma);
		word |= FIELD_PREP(NVDEC_HEVC_SPS_LOG2_MIN_PCM,
				   r->log2_min_pcm_luma_coding_block_size);
		word |= FIELD_PREP(NVDEC_HEVC_SPS_LOG2_MAX_PCM,
				   r->log2_max_pcm_luma_coding_block_size);
	}
	if (r->sps_flags & NVDEC_HEVC_SPS_PCM_LOOP_FILTER_DISABLED)
		word |= NVDEC_HEVC_SPS_PCM_LOOP_FILTER_DIS;
	if (r->sps_flags & NVDEC_HEVC_SPS_TEMPORAL_MVP)
		word |= NVDEC_HEVC_SPS_TEMPORAL_MVP_EN;
	if (r->sps_flags & NVDEC_HEVC_SPS_STRONG_INTRA_SMOOTHING)
		word |= NVDEC_HEVC_SPS_STRONG_INTRA_SMOOTH;
	setup->sps_flags = cpu_to_le32(word);

	word = FIELD_PREP(NVDEC_HEVC_PPS0_EXTRA_SLICE_BITS,
			  r->num_extra_slice_header_bits);
	word |= FIELD_PREP(NVDEC_HEVC_PPS0_NUM_REF_IDX_L0,
			   r->num_ref_idx_l0_default_active);
	word |= FIELD_PREP(NVDEC_HEVC_PPS0_NUM_REF_IDX_L1,
			   r->num_ref_idx_l1_default_active);
	word |= FIELD_PREP(NVDEC_HEVC_PPS0_INIT_QP, r->init_qp);
	word |= FIELD_PREP(NVDEC_HEVC_PPS0_DIFF_CU_QP_DEPTH,
			   r->diff_cu_qp_delta_depth);
	if (r->pps_flags & NVDEC_HEVC_PPS_DEPENDENT_SLICE_SEGMENTS)
		word |= NVDEC_HEVC_PPS0_DEPENDENT_SLICES;
	if (r->pps_flags & NVDEC_HEVC_PPS_OUTPUT_FLAG_PRESENT)
		word |= NVDEC_HEVC_PPS0_OUTPUT_FLAG_PRESENT;
	if (r->pps_flags & NVDEC_HEVC_PPS_SIGN_DATA_HIDING)
		word |= NVDEC_HEVC_PPS0_SIGN_DATA_HIDING;
	if (r->pps_flags & NVDEC_HEVC_PPS_CABAC_INIT_PRESENT)
		word |= NVDEC_HEVC_PPS0_CABAC_INIT_PRESENT;
	if (r->pps_flags & NVDEC_HEVC_PPS_CONSTRAINED_INTRA_PRED)
		word |= NVDEC_HEVC_PPS0_CONSTRAINED_INTRA;
	if (r->pps_flags & NVDEC_HEVC_PPS_TRANSFORM_SKIP)
		word |= NVDEC_HEVC_PPS0_TRANSFORM_SKIP;
	if (r->pps_flags & NVDEC_HEVC_PPS_CU_QP_DELTA)
		word |= NVDEC_HEVC_PPS0_CU_QP_DELTA;
	setup->pps_flags0 = cpu_to_le32(word);

	setup->pps_cb_qp_offset = r->pps_cb_qp_offset;
	setup->pps_cr_qp_offset = r->pps_cr_qp_offset;
	setup->pps_beta_offset = r->pps_beta_offset;
	setup->pps_tc_offset = r->pps_tc_offset;

	word = FIELD_PREP(NVDEC_HEVC_PPS1_LOG2_PARALLEL_MERGE,
			  r->log2_parallel_merge_level);
	if (r->pps_flags & NVDEC_HEVC_PPS_TILES) {
		word |= NVDEC_HEVC_PPS1_TILES_ENABLED;
		word |= FIELD_PREP(NVDEC_HEVC_PPS1_NUM_TILE_COLUMNS,
				   r->num_tile_columns);
		word |= FIELD_PREP(NVDEC_HEVC_PPS1_NUM_TILE_ROWS,
				   r->num_tile_rows);
		if (r->pps_flags & NVDEC_HEVC_PPS_LOOP_FILTER_ACROSS_TILES)
			word |= NVDEC_HEVC_PPS1_LF_ACROSS_TILES;
	}
	if (r->pps_flags & NVDEC_HEVC_PPS_SLICE_CHROMA_QP_OFFSETS)
		word |= NVDEC_HEVC_PPS1_SLICE_CHROMA_QP;
	if (r->pps_flags & NVDEC_HEVC_PPS_WEIGHTED_PRED)
		word |= NVDEC_HEVC_PPS1_WEIGHTED_PRED;
	if (r->pps_flags & NVDEC_HEVC_PPS_WEIGHTED_BIPRED)
		word |= NVDEC_HEVC_PPS1_WEIGHTED_BIPRED;
	if (r->pps_flags & NVDEC_HEVC_PPS_TRANSQUANT_BYPASS)
		word |= NVDEC_HEVC_PPS1_TRANSQUANT_BYPASS;
	if (r->pps_flags & NVDEC_HEVC_PPS_ENTROPY_CODING_SYNC)
		word |= NVDEC_HEVC_PPS1_ENTROPY_SYNC;
	if (r->pps_flags & NVDEC_HEVC_PPS_LOOP_FILTER_ACROSS_SLICES)
		word |= NVDEC_HEVC_PPS1_LF_ACROSS_SLICES;
	if (r->pps_flags & NVDEC_HEVC_PPS_DEBLOCKING_CONTROL)
		word |= NVDEC_HEVC_PPS1_DEBLOCK_CONTROL;
	if (r->pps_flags & NVDEC_HEVC_PPS_DEBLOCKING_OVERRIDE)
		word |= NVDEC_HEVC_PPS1_DEBLOCK_OVERRIDE;
	if (r->pps_flags & NVDEC_HEVC_PPS_DEBLOCKING_DISABLED)
		word |= NVDEC_HEVC_PPS1_DEBLOCK_DISABLED;
	if (r->pps_flags & NVDEC_HEVC_PPS_LISTS_MODIFICATION)
		word |= NVDEC_HEVC_PPS1_LISTS_MODIFICATION;
	if (r->pps_flags & NVDEC_HEVC_PPS_SLICE_HEADER_EXTENSION)
		word |= NVDEC_HEVC_PPS1_SLICE_HDR_EXTENSION;
	setup->pps_flags1 = cpu_to_le32(word);

	setup->num_ref_frames = r->num_ref_frames;
	setup->idr_picture_flag = !!(r->sps_flags & NVDEC_HEVC_SPS_IDR);
	setup->rap_picture_flag = !!(r->sps_flags & NVDEC_HEVC_SPS_IRAP);
	setup->curr_pic_idx = current_index;
	/* 8-bit output needs no dithering, and 2 is what turns it off. */
	setup->pattern_id = 2;
	setup->sw_hdr_skip_length = cpu_to_le16(r->sw_hdr_skip_length);

	for (i = 0; i < NVDEC_HEVC_DPB_ENTRIES; i++) {
		if (!r->dpb[i].valid)
			continue;
		mask |= BIT(picture_indices[i]);
		setup->ref_diff_poc[picture_indices[i]] =
			cpu_to_le16(clamp_t(int, r->pic_order_cnt_val -
					    r->dpb[i].pic_order_cnt_val,
					    S8_MIN, S8_MAX));
		if (r->dpb[i].long_term)
			setup->longtermflag |=
				cpu_to_le16(BIT(15 - picture_indices[i]));
	}

	for (i = 0; i < NVDEC_HEVC_MAX_PICTURES; i++) {
		if (!(mask & BIT(i)))
			setup->ref_diff_poc[i] = cpu_to_le16(scratch_diff_poc);
	}

	for (i = 0; i < r->num_poc_st_curr_before; i++)
		order0[n++] = picture_indices[r->poc_st_curr_before[i]];
	for (i = 0; i < r->num_poc_st_curr_after; i++)
		order0[n++] = picture_indices[r->poc_st_curr_after[i]];
	for (i = 0; i < r->num_poc_lt_curr; i++)
		order0[n++] = picture_indices[r->poc_lt_curr[i]];

	for (i = 0; i < r->num_poc_st_curr_after; i++)
		order1[m++] = picture_indices[r->poc_st_curr_after[i]];
	for (i = 0; i < r->num_poc_st_curr_before; i++)
		order1[m++] = picture_indices[r->poc_st_curr_before[i]];
	for (i = 0; i < r->num_poc_lt_curr; i++)
		order1[m++] = picture_indices[r->poc_lt_curr[i]];

	nvdec_hevc_fill_reflist(setup->initreflistidxl0, order0, n);
	nvdec_hevc_fill_reflist(setup->initreflistidxl1, order1, m);

	if (r->sps_flags & NVDEC_HEVC_SPS_SCALING_LIST)
		nvdec_hevc_fill_scaling_list(hjob->state->cpu +
					     NVDEC_HEVC_SCALING_OFFSET, r);
	nvdec_hevc_fill_tile_sizes(hjob->state->cpu + NVDEC_HEVC_TILES_OFFSET, r);
}

static int nvdec_hevc_prepare(struct nvdec_decode_job *hjob)
{
	struct nvdec_hevc_request *request = &hjob->req.hevc;
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct device *dev = ctx->engine->dev;
	int entry = nvdec_hevc_scratch_entry(request);
	u32 surface_size;
	unsigned int i;
	int err;

	err = nvdec_validate_frame(dev, &request->frame, NVDEC_HEVC_CTU_SIZE,
				   hjob->surface, hjob->capture, &surface_size);
	if (!err)
		err = nvdec_hevc_validate_request(dev, request, hjob->dpb,
						  surface_size);
	if (!err)
		err = nvdec_hevc_prepare_scratch(ctx, request);
	if (err)
		return err;
	hjob->scratch = nvdec_engine_map_get(ctx->scratch);

	err = nvdec_surface_index(ctx, hjob->surface, &hjob->picture_index,
				  NVDEC_HEVC_MAX_PICTURES);
	if (err)
		return err;
	for (i = 0; i < NVDEC_HEVC_DPB_ENTRIES; i++) {
		if (!request->dpb[i].valid)
			continue;
		err = nvdec_surface_index(ctx, hjob->dpb[i],
					  &hjob->picture_indices[i],
					  NVDEC_HEVC_MAX_PICTURES);
		if (err)
			return err;
	}

	hjob->num_pictures = NVDEC_HEVC_MAX_PICTURES;
	for (i = 0; i < NVDEC_HEVC_MAX_PICTURES; i++)
		hjob->pictures[i] = entry >= 0 ? hjob->dpb[entry] : hjob->surface;
	hjob->pictures[hjob->picture_index] = hjob->surface;
	for (i = 0; i < NVDEC_HEVC_DPB_ENTRIES; i++) {
		if (request->dpb[i].valid)
			hjob->pictures[hjob->picture_indices[i]] = hjob->dpb[i];
	}

	return 0;
}

static void nvdec_hevc_emit(struct nvdec_decode_job *hjob,
			    struct falcon_gather *g)
{
	dma_addr_t state = hjob->state->iova, scratch = hjob->scratch->iova;

	falcon_gather_address(g, NVDEC_HEVC_METHOD_SCALING_LIST,
			      state + NVDEC_HEVC_SCALING_OFFSET);
	falcon_gather_address(g, NVDEC_HEVC_METHOD_TILE_SIZES,
			      state + NVDEC_HEVC_TILES_OFFSET);
	falcon_gather_address(g, NVDEC_HEVC_METHOD_FILTER,
			      scratch + hjob->ctx->filter_offset);
	falcon_gather_address(g, NVDEC_METHOD_COLOC, scratch);
}

static const struct nvdec_codec_ops nvdec_hevc_ops = {
	.application = 7,
	.setup_size = NVDEC_HEVC_SETUP_SIZE,
	.status_offset = NVDEC_HEVC_STATUS_OFFSET,
	.vic_config = 0x1000,
	.prepare = nvdec_hevc_prepare,
	.fill = nvdec_hevc_fill_setup,
	.emit = nvdec_hevc_emit,
};

static const struct nvdec_codec_ops *const nvdec_codec_ops[] = {
	[NVDEC_CODEC_H264] = &nvdec_h264_ops,
	[NVDEC_CODEC_HEVC] = &nvdec_hevc_ops,
};

/* The staged bitstream is followed by its terminator and the slice offsets. */
static void nvdec_write_slice_offsets(struct nvdec_decode_job *hjob)
{
	struct nvdec_decode_context *ctx = hjob->ctx;
	__le32 *offsets;
	unsigned int i;

	hjob->slice_offsets_off = ALIGN(ctx->staged + 16, SZ_256);
	memcpy(hjob->input->cpu + ctx->staged, hjob->ops->termination, 16);
	offsets = hjob->input->cpu + hjob->slice_offsets_off;
	for (i = 0; i < ctx->slice_count; i++)
		offsets[i] = cpu_to_le32(ctx->slice_offsets[i]);
	offsets[i] = cpu_to_le32(ctx->staged);
}

int nvdec_engine_submit(struct nvdec_decode_context *ctx,
			const union nvdec_request *request,
			struct nvdec_engine_map *surface,
			struct nvdec_engine_map *capture,
			struct nvdec_engine_map * const refs[NVDEC_MAX_REFS],
			struct dma_fence **fence,
			nvdec_engine_complete_t complete, void *data)
{
	struct nvdec_decode_job *hjob;
	unsigned int i;
	int err;

	if (!ctx || !request || !surface || !capture || !refs || !fence)
		return -EINVAL;
	*fence = NULL;

	mutex_lock(&ctx->lock);
	if (ctx->in_flight) {
		err = -EBUSY;
		goto unlock;
	}
	if (!ctx->slice_count) {
		err = -EINVAL;
		goto unlock;
	}
	hjob = kzalloc_obj(*hjob);
	if (!hjob) {
		err = -ENOMEM;
		goto unlock;
	}
	hjob->ctx = ctx;
	kref_get(&ctx->ref);
	hjob->ops = nvdec_codec_ops[ctx->codec];
	hjob->req = *request;
	hjob->req.frame.output_payload_size = ctx->staged;
	hjob->complete = complete;
	hjob->complete_data = data;
	hjob->surface = nvdec_engine_map_get(surface);
	hjob->capture = nvdec_engine_map_get(capture);
	for (i = 0; i < NVDEC_MAX_REFS; i++) {
		if (refs[i])
			hjob->dpb[i] = nvdec_engine_map_get(refs[i]);
	}

	err = hjob->ops->prepare(hjob);
	if (err)
		goto free_hjob;

	hjob->vic = vic_engine_find(ctx->engine->client.drm);
	if (!hjob->vic) {
		err = -ENODEV;
		goto free_hjob;
	}
	hjob->input = nvdec_engine_map_get(ctx->input);
	hjob->state = nvdec_buffer_create(ctx->engine, hjob->ops->vic_config +
					  VIC_CONFIG_SIZE, true);
	if (IS_ERR(hjob->state)) {
		err = PTR_ERR(hjob->state);
		hjob->state = NULL;
		goto free_hjob;
	}
	hjob->gather = nvdec_buffer_create(ctx->engine,
					   NVDEC_GATHER_WORDS * sizeof(u32), true);
	if (IS_ERR(hjob->gather)) {
		err = PTR_ERR(hjob->gather);
		hjob->gather = NULL;
		goto free_hjob;
	}
	hjob->fence = nvdec_fence_create(ctx->engine);
	if (IS_ERR(hjob->fence)) {
		err = PTR_ERR(hjob->fence);
		hjob->fence = NULL;
		goto free_hjob;
	}
	err = nvdec_install_fences(hjob);
	if (err)
		goto free_hjob;

	if (hjob->ops->termination)
		nvdec_write_slice_offsets(hjob);
	hjob->ops->fill(hjob);
	nvdec_fill_detile(hjob);
	err = nvdec_build_gather(hjob);
	if (!err)
		err = nvdec_launch_job(hjob, fence);
	if (err)
		goto free_hjob;

	mutex_unlock(&ctx->lock);
	return 0;

free_hjob:
	nvdec_job_free(hjob, true);
unlock:
	mutex_unlock(&ctx->lock);
	return err;
}

int nvdec_engine_open_channel(struct nvdec_engine *engine,
			      struct tegra_drm_context *context)
{
	context->channel = host1x_channel_get(engine->channel);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

void nvdec_engine_close_channel(struct tegra_drm_context *context)
{
	host1x_channel_put(context->channel);
}

static const struct nvdec_engine_config nvdec_t210_config = {
	.firmware = NVIDIA_TEGRA_210_NVDEC_FIRMWARE,
	.version = 0x21,
	.supports_sid = false,
};

static const struct nvdec_engine_config nvdec_t186_config = {
	.firmware = NVIDIA_TEGRA_186_NVDEC_FIRMWARE,
	.version = 0x18,
	.supports_sid = true,
};

static const struct nvdec_engine_config nvdec_t194_config = {
	.firmware = NVIDIA_TEGRA_194_NVDEC_FIRMWARE,
	.version = 0x19,
	.supports_sid = true,
};

static const struct nvdec_engine_config nvdec_t234_config = {
	.version = 0x23,
	.supports_sid = true,
	.has_riscv = true,
	.has_extra_clocks = true,
};

const struct of_device_id nvdec_engine_of_match[] = {
	{ .compatible = "nvidia,tegra210-nvdec", .data = &nvdec_t210_config },
	{ .compatible = "nvidia,tegra186-nvdec", .data = &nvdec_t186_config },
	{ .compatible = "nvidia,tegra194-nvdec", .data = &nvdec_t194_config },
	{ .compatible = "nvidia,tegra234-nvdec", .data = &nvdec_t234_config },
	{ },
};
MODULE_DEVICE_TABLE(of, nvdec_engine_of_match);

struct nvdec_engine *nvdec_engine_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct nvdec_engine *engine;
	u32 host_class;
	int err;

	err = dma_coerce_mask_and_coherent(dev, *dev->parent->dma_mask);
	if (err < 0) {
		dev_err(dev, "failed to set DMA mask: %d\n", err);
		return ERR_PTR(err);
	}

	engine = devm_kzalloc(dev, sizeof(*engine), GFP_KERNEL);
	if (!engine)
		return ERR_PTR(-ENOMEM);

	engine->config = of_device_get_match_data(dev);
	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return ERR_PTR(-ENOMEM);

	engine->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(engine->regs))
		return ERR_CAST(engine->regs);

	engine->clks[0].id = "nvdec";
	engine->num_clks = 1;
	if (engine->config->has_extra_clocks) {
		engine->num_clks = 3;
		engine->clks[1].id = "fuse";
		engine->clks[2].id = "tsec_pka";
	}

	err = devm_clk_bulk_get(dev, engine->num_clks, engine->clks);
	if (err) {
		dev_err(dev, "failed to get clock(s)\n");
		return ERR_PTR(err);
	}

	err = clk_set_rate(engine->clks[0].clk, ULONG_MAX);
	if (err < 0) {
		dev_err(dev, "failed to set clock rate\n");
		return ERR_PTR(err);
	}

	err = of_property_read_u32(dev->of_node, "nvidia,host1x-class",
				   &host_class);
	if (err < 0)
		host_class = HOST1X_CLASS_NVDEC;

	if (engine->config->has_riscv) {
		struct tegra_mc *mc;

		mc = devm_tegra_memory_controller_get(dev);
		if (IS_ERR(mc)) {
			dev_err_probe(dev, PTR_ERR(mc),
				      "failed to get memory controller handle\n");
			return ERR_CAST(mc);
		}

		err = tegra_mc_get_carveout_info(mc, 1, &engine->carveout_base,
						 NULL);
		if (err) {
			dev_err(dev, "failed to get carveout info: %d\n", err);
			return ERR_PTR(err);
		}

		engine->reset = devm_reset_control_get_exclusive_released(dev,
									  "nvdec");
		if (IS_ERR(engine->reset)) {
			dev_err_probe(dev, PTR_ERR(engine->reset),
				      "failed to get reset\n");
			return ERR_CAST(engine->reset);
		}

		engine->riscv.dev = dev;
		engine->riscv.regs = engine->regs;
		err = tegra_drm_riscv_read_descriptors(&engine->riscv);
		if (err < 0)
			return ERR_PTR(err);
	} else {
		engine->falcon.dev = dev;
		engine->falcon.regs = engine->regs;
		err = falcon_init(&engine->falcon);
		if (err < 0)
			return ERR_PTR(err);
	}

	mutex_init(&engine->recovery_lock);
	atomic_set(&engine->active_jobs, 0);
	init_completion(&engine->idle);
	complete_all(&engine->idle);
	INIT_WORK(&engine->recovery_work, nvdec_engine_recovery_work);
	engine->h264_fence_context = dma_fence_context_alloc(1);
	atomic64_set(&engine->h264_fence_seqno, 0);

	INIT_LIST_HEAD(&engine->client.base.list);
	engine->client.base.ops = &nvdec_engine_client_ops;
	engine->client.base.dev = dev;
	engine->client.base.class = host_class;
	engine->client.base.syncpts = syncpts;
	engine->client.base.num_syncpts = 1;
	engine->dev = dev;

	platform_set_drvdata(pdev, engine);

	return engine;
}

int nvdec_engine_register(struct nvdec_engine *engine)
{
	int err;

	err = host1x_client_register(&engine->client.base);
	if (err < 0) {
		dev_err(engine->dev, "failed to register host1x client: %d\n", err);
		falcon_exit(&engine->falcon);
		return err;
	}

	pm_runtime_enable(engine->dev);
	pm_runtime_use_autosuspend(engine->dev);
	pm_runtime_set_autosuspend_delay(engine->dev, 500);

	return 0;
}

void nvdec_engine_unregister(struct nvdec_engine *engine)
{
	cancel_work_sync(&engine->recovery_work);
	pm_runtime_disable(engine->dev);
	host1x_client_unregister(&engine->client.base);
	falcon_exit(&engine->falcon);
}

struct tegra_drm_client *nvdec_engine_drm_client(struct nvdec_engine *engine)
{
	return &engine->client;
}

struct nvdec_engine *
nvdec_engine_from_drm_client(struct tegra_drm_client *client)
{
	return to_nvdec_engine(client);
}

unsigned int nvdec_engine_version(struct nvdec_engine *engine)
{
	return engine->config->version;
}

bool nvdec_engine_ready(struct nvdec_engine *engine)
{
	return engine->channel && engine->client.base.syncpts[0];
}

struct device *nvdec_engine_device(struct nvdec_engine *engine)
{
	return engine->dev;
}

void nvdec_engine_set_v4l2(struct nvdec_engine *engine,
			   struct nvdec_v4l2 *v4l2)
{
	engine->v4l2 = v4l2;
}

struct nvdec_v4l2 *nvdec_engine_get_v4l2(struct nvdec_engine *engine)
{
	return engine->v4l2;
}
