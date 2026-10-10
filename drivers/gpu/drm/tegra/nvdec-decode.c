// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tegra210 NVDEC stateless decode core: turns one validated picture into an
 * NVDEC job and a VIC detile job.
 *
 * Copyright (c) 2026 mojyack <mojyack@gmail.com>
 *
 * The setup structures transcribe NVIDIA's nvdec_drv.h and the method
 * numbers its clc5b0.h, both Copyright (c) 1993-2024 NVIDIA CORPORATION &
 * AFFILIATES and MIT licensed.
 */

#include <linux/bitfield.h>
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/dma-resv.h>
#include <linux/host1x.h>
#include <linux/iosys-map.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#include <media/v4l2-ctrls.h>

#include "drm.h"
#include "falcon.h"
#include "gem.h"
#include "nvdec.h"
#include "nvdec-decode.h"
#include "vic.h"

/* Method offsets of the NVDEC class, divided by four. */
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

#define NVDEC_CONTROL_CODEC_TYPE		GENMASK(3, 0)
#define NVDEC_CONTROL_GPTIMER_ON		BIT(4)
#define NVDEC_CONTROL_ERR_CONCEAL_ON		BIT(6)
#define NVDEC_EXECUTE_AWAKEN			BIT(8)

#define NVDEC_APPLICATION_H264			3
#define NVDEC_APPLICATION_HEVC			7

/* The NVDEC job's methods and OP_DONE, then the VIC job's. */
#define NVDEC_GATHER_WORDS			256
#define NVDEC_VIC_GATHER_OFFSET		(192 * sizeof(u32))
/* The state buffer ends in one VIC config per detile pass. */
#define NVDEC_VIC_CONFIG_STRIDE		0x800
#define NVDEC_MAX_VIC_PASSES			2
#define NVDEC_JOB_TIMEOUT_MS			10000

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

/* What the firmware reports at the STATUS address once a picture is done. */
struct nvdec_status {
	__le32 mbs_correctly_decoded;
	__le32 mbs_in_error;
	__le32 cycle_count;
	__le32 error_status;
};

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
	__le32 lossless_flags;	/* NVDEC_H264_IPRED8X8_FILTER */
	u8 display[0x1c];
	u8 ssm[0xc];
};

#define NVDEC_H264_DPB_INDEX		GENMASK(6, 0)
#define NVDEC_H264_DPB_COL_IDX		GENMASK(11, 7)
#define NVDEC_H264_DPB_STATE		GENMASK(13, 12)
#define NVDEC_H264_DPB_LONG_TERM	BIT(14)
#define NVDEC_H264_DPB_IS_FIELD	BIT(16)
#define NVDEC_H264_DPB_TOP_MARKING	GENMASK(20, 17)
#define NVDEC_H264_DPB_BOTTOM_MARKING	GENMASK(24, 21)

#define NVDEC_H264_PIC_MBAFF		BIT(0)
#define NVDEC_H264_PIC_DIRECT_8X8	BIT(1)
#define NVDEC_H264_PIC_WEIGHTED_PRED	BIT(2)
#define NVDEC_H264_PIC_CONSTRAINED_INTRA BIT(3)
#define NVDEC_H264_PIC_REFERENCE	BIT(4)
#define NVDEC_H264_PIC_FIELD		BIT(5)
#define NVDEC_H264_PIC_BOTTOM_FIELD	BIT(6)
#define NVDEC_H264_PIC_SECOND_FIELD	BIT(7)
#define NVDEC_H264_PIC_LOG2_MAX_FRAME_NUM GENMASK(11, 8)
#define NVDEC_H264_PIC_CHROMA_FORMAT	GENMASK(13, 12)
#define NVDEC_H264_PIC_POC_TYPE	GENMASK(15, 14)
#define NVDEC_H264_PIC_INIT_QP		GENMASK(21, 16)
#define NVDEC_H264_PIC_CHROMA_QP	GENMASK(26, 22)
#define NVDEC_H264_PIC_SECOND_CHROMA_QP GENMASK(31, 27)

#define NVDEC_H264_CUR_WEIGHTED_BIPRED	GENMASK(1, 0)
#define NVDEC_H264_CUR_PIC_IDX		GENMASK(8, 2)
#define NVDEC_H264_CUR_COL_IDX		GENMASK(13, 9)
#define NVDEC_H264_CUR_FRAME_NUM	GENMASK(29, 14)

/* Filter Intra_8x8 reference samples, as the spec and the JM do. */
#define NVDEC_H264_IPRED8X8_FILTER	BIT(0)

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

struct nvdec_decode_map {
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
	struct nvdec_decode_map *map;
	u8 picture_index;
	/* H.264 setup slot, NVDEC_H264_DPB_ENTRIES while this is not a reference. */
	u8 dpb_slot;
};

struct nvdec_decode_context {
	struct kref ref;
	struct tegra_drm_client *client;
	struct host1x_channel *channel;
	struct device *dev;
	enum nvdec_codec codec;
	/* Both jobs of a picture increment it; dropped once a timeout locks it. */
	struct host1x_syncpt *syncpt;
	u64 fence_context;
	u64 fence_seqno;
	/* Setup, codec tables and VIC configs, rewritten for every picture. */
	struct nvdec_decode_map *state;
	struct nvdec_decode_map *gather;
	struct nvdec_decode_map *scratch;
	struct nvdec_decode_map *input;
	u16 width_in_mbs;
	u16 height_in_mbs;
	u16 coded_width;
	u16 coded_height;
	u8 picture_slots;
	u32 mbhist_offset;
	u32 mbhist_size;
	u32 history_offset;
	u32 history_size;
	u32 colmv_size;
	u32 filter_offset;
	u32 sao_offset;
	u32 bsd_offset;
	/* The staged picture in ctx->input and where each of its slices starts. */
	u32 *slice_offsets;
	unsigned int slice_count;
	unsigned int max_slices;
	u32 staged;
	struct nvdec_pool_surface surfaces[NVDEC_MAX_PICTURES];
	/* The picture between nvdec_decode_submit() and nvdec_decode_finish(). */
	struct nvdec_decode_job *job;
};

struct nvdec_fence {
	struct dma_fence base;
	/* Protects the dma_fence base. */
	spinlock_t lock;
};

struct nvdec_decode_job {
	struct nvdec_decode_context *ctx;
	const struct nvdec_codec_ops *ops;
	union nvdec_request req;
	struct nvdec_decode_map *surface;
	struct nvdec_decode_map *capture;
	struct nvdec_decode_map *dpb[NVDEC_MAX_REFS];
	/* What each firmware picture slot names; borrowed from the maps above. */
	struct nvdec_decode_map *pictures[NVDEC_MAX_PICTURES];
	u8 num_pictures;
	u8 picture_index;
	u8 picture_indices[NVDEC_MAX_REFS];
	u8 dpb_slots[NVDEC_MAX_REFS];
	u8 vic_passes;
	u32 slice_offsets_off;
	struct tegra_drm_client *vic;
	struct host1x_job *nvdec_job;
	struct host1x_job *vic_job;
	/* Signalled with the VIC job's fence, or if that job never ran. */
	struct dma_fence *fence;
	struct dma_fence *vic_fence;
	struct dma_fence_cb vic_cb;
	atomic_t signalled;
	/* Host1x jobs not yet released, plus one held by the submitter. */
	atomic_t pending;
	bool nvdec_cancelled;
	bool vic_cancelled;
	bool failed;
	bool runtime_ref;
	bool vic_runtime_ref;
	nvdec_decode_complete_t complete;
	void *complete_data;
};

/* What differs between codecs in turning a request into an NVDEC job. */
struct nvdec_codec_ops {
	u8 application;
	u16 setup_size;
	u16 status_offset;
	/* The state buffer's first VIC config, after the codec's own tables. */
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

/* Past the last VIC config; never rewritten, so it stays zero. */
#define NVDEC_VIC_FILTER	(0x1000 + NVDEC_MAX_VIC_PASSES * NVDEC_VIC_CONFIG_STRIDE)
#define NVDEC_STATE_SIZE	(NVDEC_VIC_FILTER + VIC_FILTER_SIZE)

struct nvdec_decode_map *nvdec_decode_map_get(struct nvdec_decode_map *map)
{
	kref_get(&map->ref);
	return map;
}

static void nvdec_decode_map_release(struct kref *ref)
{
	struct nvdec_decode_map *map = container_of(ref, struct nvdec_decode_map, ref);

	if (map->cpu)
		host1x_bo_munmap(&map->bo->base, map->cpu);
	if (map->pin)
		host1x_bo_unpin(map->pin);
	drm_gem_object_put(&map->bo->gem);
	kfree(map);
}

void nvdec_decode_map_put(struct nvdec_decode_map *map)
{
	if (map)
		kref_put(&map->ref, nvdec_decode_map_release);
}

struct dma_resv *nvdec_decode_map_resv(struct nvdec_decode_map *map)
{
	return map->resv;
}

/* NVDEC's methods carry 32 bits of address after the shift by 8. */
static int nvdec_map_bo(struct device *dev, struct nvdec_decode_map *map,
			struct tegra_bo *bo)
{
	struct host1x_bo_mapping *pin;

	map->bo = bo;

	/* The DMA API would return physical addresses; use the domain's IOVA. */
	if (bo->mm) {
		map->iova = bo->iova;
	} else {
		pin = host1x_bo_pin(dev, &bo->base, map->direction, NULL);
		if (IS_ERR(pin))
			return PTR_ERR(pin);
		map->pin = pin;
		if (pin->chunks != 1)
			return -EINVAL;
		map->iova = pin->phys;
	}

	return upper_32_bits(map->iova) ? -ERANGE : 0;
}

static struct nvdec_decode_map *
nvdec_map_alloc(size_t size, enum dma_data_direction direction)
{
	struct nvdec_decode_map *map;

	map = kzalloc_obj(*map);
	if (!map)
		return NULL;

	kref_init(&map->ref);
	map->size = size;
	map->direction = direction;
	return map;
}

/* Every engine buffer is a tegra_bo in the host1x domain, as userspace's are. */
static struct nvdec_decode_map *
nvdec_buffer_create(struct nvdec_decode_context *ctx, size_t size, bool cpu)
{
	struct nvdec_decode_map *map;
	struct tegra_bo *bo;
	int err;

	if (!size)
		return ERR_PTR(-EINVAL);

	map = nvdec_map_alloc(size, DMA_BIDIRECTIONAL);
	if (!map)
		return ERR_PTR(-ENOMEM);

	bo = tegra_bo_create(ctx->client->drm->drm, size, 0);
	if (IS_ERR(bo)) {
		kfree(map);
		return ERR_CAST(bo);
	}

	err = nvdec_map_bo(ctx->dev, map, bo);
	if (!err && cpu) {
		map->cpu = host1x_bo_mmap(&bo->base);
		if (IS_ERR(map->cpu)) {
			err = PTR_ERR(map->cpu);
			map->cpu = NULL;
		}
	}
	if (err) {
		nvdec_decode_map_put(map);
		return ERR_PTR(err);
	}

	return map;
}

struct nvdec_decode_map *
nvdec_decode_surface_create(struct nvdec_decode_context *ctx, size_t size)
{
	return nvdec_buffer_create(ctx, size, false);
}

struct nvdec_decode_map *
nvdec_decode_map_create(struct nvdec_decode_context *ctx, struct dma_buf *dmabuf,
			size_t size, enum dma_data_direction direction)
{
	struct nvdec_decode_map *map;
	struct drm_gem_object *gem;
	int err;

	if (!dmabuf || !size || size > dmabuf->size)
		return ERR_PTR(-EINVAL);

	map = nvdec_map_alloc(size, direction);
	if (!map)
		return ERR_PTR(-ENOMEM);

	gem = tegra_gem_prime_import(ctx->client->drm->drm, dmabuf);
	if (IS_ERR(gem)) {
		kfree(map);
		return ERR_CAST(gem);
	}

	map->resv = gem->resv;
	err = nvdec_map_bo(ctx->dev, map, to_tegra_bo(gem));
	if (err) {
		nvdec_decode_map_put(map);
		return ERR_PTR(err);
	}

	return map;
}

static bool nvdec_map_is_valid(const struct nvdec_decode_map *map,
			       enum dma_data_direction direction, size_t size)
{
	if (!map || map->size < size)
		return false;

	return map->direction == direction || map->direction == DMA_BIDIRECTIONAL;
}

static const char *nvdec_fence_get_driver_name(struct dma_fence *fence)
{
	return "tegra-nvdec";
}

static const char *nvdec_fence_get_timeline_name(struct dma_fence *fence)
{
	return "tegra-nvdec";
}

static const struct dma_fence_ops nvdec_fence_ops = {
	.get_driver_name = nvdec_fence_get_driver_name,
	.get_timeline_name = nvdec_fence_get_timeline_name,
};

static int nvdec_h264_prepare_scratch(struct nvdec_decode_context *ctx,
				      const struct nvdec_h264_request *request)
{
	struct nvdec_decode_map *scratch;
	u32 coloc_per_picture, coloc_size, mbhist_size, history_size;
	u32 mbhist_offset, history_offset, size;
	u8 slots = max_t(u8, request->max_num_ref_frames, 1) + 1;

	if (ctx->scratch) {
		if (ctx->width_in_mbs != request->pic_width_in_mbs ||
		    ctx->height_in_mbs != request->frame_height_in_mbs ||
		    slots > ctx->picture_slots)
			return -EBUSY;
		return 0;
	}

	if (check_mul_overflow((u32)ALIGN(request->frame_height_in_mbs, 2),
			       (u32)request->pic_width_in_mbs * 64, &coloc_per_picture) ||
	    coloc_per_picture < 63)
		return -EOVERFLOW;
	coloc_per_picture = ALIGN(coloc_per_picture - 63, 0x100);
	if (check_mul_overflow(coloc_per_picture, (u32)slots,
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

	scratch = nvdec_decode_surface_create(ctx, size);
	if (IS_ERR(scratch))
		return PTR_ERR(scratch);

	ctx->scratch = scratch;
	ctx->width_in_mbs = request->pic_width_in_mbs;
	ctx->height_in_mbs = request->frame_height_in_mbs;
	ctx->picture_slots = slots;
	ctx->mbhist_offset = mbhist_offset;
	ctx->mbhist_size = mbhist_size;
	ctx->history_offset = history_offset;
	ctx->history_size = history_size;
	return 0;
}

static int nvdec_prepare_input(struct nvdec_decode_context *ctx, size_t size)
{
	struct nvdec_decode_map *input;

	if (ctx->input && ctx->input->size >= size)
		return 0;

	input = nvdec_buffer_create(ctx, ALIGN(size, SZ_4K), true);
	if (IS_ERR(input))
		return PTR_ERR(input);

	/* Growing for the slice offsets keeps the bitstream already staged. */
	if (ctx->input) {
		memcpy(input->cpu, ctx->input->cpu, ctx->staged);
		nvdec_decode_map_put(ctx->input);
	}
	ctx->input = input;
	return 0;
}

static int nvdec_record_slice(struct nvdec_decode_context *ctx, u32 offset)
{
	if (ctx->slice_count + 1 > ctx->max_slices) {
		unsigned int want = max(ctx->max_slices * 2, 8U);
		u32 *offsets = krealloc_array(ctx->slice_offsets, want + 1,
					      sizeof(*offsets), GFP_KERNEL);

		if (!offsets)
			return -ENOMEM;
		ctx->slice_offsets = offsets;
		ctx->max_slices = want;
	}

	ctx->slice_offsets[ctx->slice_count++] = offset;
	return 0;
}

/* The firmware wants the offset of every slice. */
static int nvdec_scan_slices(struct nvdec_decode_context *ctx)
{
	const u8 *data = ctx->input->cpu;
	int err;
	u32 i;

	ctx->slice_count = 0;
	for (i = 0; i + 4 <= ctx->staged; i++) {
		u8 type = data[i + 3];

		if (data[i] || data[i + 1] || data[i + 2] != 1)
			continue;
		/* Data partitions would be skipped silently: Extended profile only. */
		if ((type & 0x1f) >= 2 && (type & 0x1f) <= 4)
			return -EINVAL;
		if ((type & 0x1f) != 1 && (type & 0x1f) != 5) {
			i += 2;
			continue;
		}
		err = nvdec_record_slice(ctx, i);
		if (err)
			return err;
		i += 3;
	}

	return ctx->slice_count ? 0 : -EINVAL;
}

/* An RBSP reader over a slice header, emulation prevention bytes removed. */
struct nvdec_rbsp {
	u8 data[64];
	unsigned int size;
	unsigned int bit;
	bool error;
};

static void nvdec_rbsp_init(struct nvdec_rbsp *r, const u8 *nal, u32 len)
{
	unsigned int zeros = 0;
	u32 i;

	r->size = 0;
	r->bit = 0;
	r->error = false;
	for (i = 0; i < len && r->size < sizeof(r->data); i++) {
		if (zeros >= 2 && nal[i] == 3) {
			zeros = 0;
			continue;
		}
		zeros = nal[i] ? 0 : zeros + 1;
		r->data[r->size++] = nal[i];
	}
}

static u32 nvdec_rbsp_u(struct nvdec_rbsp *r, unsigned int n)
{
	u32 value = 0;

	while (n--) {
		if (r->bit >= r->size * 8) {
			r->error = true;
			return 0;
		}
		value = value << 1 | (r->data[r->bit / 8] >> (7 - r->bit % 8) & 1);
		r->bit++;
	}

	return value;
}

static u32 nvdec_rbsp_ue(struct nvdec_rbsp *r)
{
	unsigned int zeros = 0;

	while (!nvdec_rbsp_u(r, 1) && !r->error) {
		if (++zeros > 31) {
			r->error = true;
			return 0;
		}
	}

	return BIT(zeros) - 1 + nvdec_rbsp_u(r, zeros);
}

/* 7.3.3 slice_header() up to the reference count override the setup wants. */
static int nvdec_h264_parse_slice(const struct nvdec_h264_request *request,
				  const u8 *nal, u32 len, u32 *first_mb,
				  u32 *l0, u32 *l1)
{
	bool idr = (nal[0] & 0x1f) == 5, field = false, bottom = false;
	bool mbaff = request->flags & NVDEC_H264_REQ_MBAFF;
	u32 slice_type, size_in_mbs, max_refs;
	struct nvdec_rbsp r;

	nvdec_rbsp_init(&r, nal + 1, len - 1);
	*first_mb = nvdec_rbsp_ue(&r);
	slice_type = nvdec_rbsp_ue(&r);
	nvdec_rbsp_ue(&r);					/* pic_parameter_set_id */
	nvdec_rbsp_u(&r, request->log2_max_frame_num_minus4 + 4);
	if (!(request->flags & NVDEC_H264_REQ_FRAME_MBS_ONLY)) {
		field = nvdec_rbsp_u(&r, 1);
		if (field)
			bottom = nvdec_rbsp_u(&r, 1);
	}
	if (idr)
		nvdec_rbsp_ue(&r);				/* idr_pic_id */
	if (request->pic_order_cnt_type == 0) {
		nvdec_rbsp_u(&r, request->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if ((request->pps_flags & NVDEC_H264_PPS_PIC_ORDER_PRESENT) && !field)
			nvdec_rbsp_ue(&r);			/* as long as the se(v) */
	} else if (request->pic_order_cnt_type == 1 &&
		   !(request->flags & NVDEC_H264_REQ_DELTA_POC_ZERO)) {
		nvdec_rbsp_ue(&r);				/* delta_pic_order_cnt[0] */
		if ((request->pps_flags & NVDEC_H264_PPS_PIC_ORDER_PRESENT) && !field)
			nvdec_rbsp_ue(&r);			/* delta_pic_order_cnt[1] */
	}
	/* A redundant slice would be decoded over the primary one. */
	if ((request->pps_flags & NVDEC_H264_PPS_REDUNDANT) && nvdec_rbsp_ue(&r))
		return -EINVAL;
	if (r.error || slice_type > 9 || field != !!(request->flags & NVDEC_H264_REQ_FIELD) ||
	    bottom != !!(request->flags & NVDEC_H264_REQ_BOTTOM_FIELD))
		return -EINVAL;
	/* SP and SI slices are Extended profile only, which is not supported. */
	slice_type %= 5;
	if (slice_type == V4L2_H264_SLICE_TYPE_SP ||
	    slice_type == V4L2_H264_SLICE_TYPE_SI)
		return -EINVAL;
	if (slice_type == V4L2_H264_SLICE_TYPE_B)
		nvdec_rbsp_u(&r, 1);				/* direct_spatial_mv_pred_flag */

	size_in_mbs = request->pic_width_in_mbs * request->frame_height_in_mbs;
	if (field)
		size_in_mbs /= 2;
	if (*first_mb >= size_in_mbs || (mbaff && *first_mb >= size_in_mbs / 2))
		return -EINVAL;

	*l0 = request->num_ref_idx_l0_active_minus1;
	*l1 = request->num_ref_idx_l1_active_minus1;
	if ((slice_type == V4L2_H264_SLICE_TYPE_P ||
	     slice_type == V4L2_H264_SLICE_TYPE_B) && nvdec_rbsp_u(&r, 1)) {
		*l0 = nvdec_rbsp_ue(&r);
		if (slice_type == V4L2_H264_SLICE_TYPE_B)
			*l1 = nvdec_rbsp_ue(&r);
	}

	/* 7.4.3: up to 15, or 31 for a field. */
	max_refs = field ? 31 : 15;
	return r.error || *l0 > max_refs || *l1 > max_refs ? -EINVAL : 0;
}

/* Slices must come in order; the setup takes the most references any asks for. */
static int nvdec_h264_parse_slices(struct nvdec_decode_context *ctx,
				   struct nvdec_h264_request *request)
{
	const u8 *data = ctx->input->cpu;
	u32 l0, l1, max_l0 = 0, max_l1 = 0;
	u32 first_mb, last_mb = 0;
	unsigned int i;
	u32 end;

	for (i = 0; i < ctx->slice_count; i++) {
		end = i + 1 < ctx->slice_count ? ctx->slice_offsets[i + 1] : ctx->staged;
		if (nvdec_h264_parse_slice(request, data + ctx->slice_offsets[i] + 3,
					   end - ctx->slice_offsets[i] - 3,
					   &first_mb, &l0, &l1) ||
		    (i ? first_mb <= last_mb : first_mb)) {
			dev_dbg(ctx->dev, "h264 reject: slice %u header or order\n", i);
			return -EINVAL;
		}
		last_mb = first_mb;
		max_l0 = max(max_l0, l0);
		max_l1 = max(max_l1, l1);
	}

	request->num_ref_idx_l0_active_minus1 = max_l0;
	request->num_ref_idx_l1_active_minus1 = max_l1;
	return 0;
}

/* The scratch is sized from the stream, so a new stream needs a new one. */
void nvdec_decode_context_reset(struct nvdec_decode_context *ctx)
{
	nvdec_decode_map_put(ctx->scratch);
	ctx->scratch = NULL;
	ctx->slice_count = 0;
	ctx->staged = 0;
	ctx->width_in_mbs = 0;
	ctx->height_in_mbs = 0;
	ctx->picture_slots = 0;
	ctx->coded_width = 0;
	ctx->coded_height = 0;
}

/* The decode surface and the detile destination, as every codec lays them out. */
static int nvdec_validate_frame(struct device *dev, const struct nvdec_frame *f,
				unsigned int align, unsigned int sample_bytes,
				const struct nvdec_decode_map *surface,
				const struct nvdec_decode_map *capture,
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
	    f->luma_stride < f->coded_width * sample_bytes ||
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
	    f->dst_stride < f->crop_width * sample_bytes ||
	    !nvdec_map_is_valid(capture, DMA_FROM_DEVICE, dst_size)) {
		dev_dbg(dev, "reject: detile destination\n");
		return -EINVAL;
	}

	return 0;
}

static int nvdec_h264_validate_request(struct device *dev,
				       const struct nvdec_h264_request *request,
				       struct nvdec_decode_map * const dpb[],
				       u32 surface_size)
{
	unsigned int i;

	dev_dbg(dev,
		"h264 request: profile=%u level=%u chroma=%u depth=%u/%u log2fn=%u poc=%u/%u maxref=%u flags=0x%x pps=0x%x wbipred=%u slicegroups=%u mbs=%ux%u slices=%u\n",
		request->profile_idc, request->level_idc,
		request->chroma_format_idc, request->bit_depth_luma_minus8,
		request->bit_depth_chroma_minus8,
		request->log2_max_frame_num_minus4, request->pic_order_cnt_type,
		request->log2_max_pic_order_cnt_lsb_minus4,
		request->max_num_ref_frames, request->flags, request->pps_flags,
		request->weighted_bipred_idc, request->num_slice_groups_minus1,
		request->pic_width_in_mbs,
		request->frame_height_in_mbs, request->slice_count);

	if ((request->profile_idc != 66 && request->profile_idc != 77 &&
	     request->profile_idc != 88 && request->profile_idc != 100) ||
	    request->level_idc < 10 || request->level_idc > 51 ||
	    request->chroma_format_idc != 1 || request->bit_depth_luma_minus8 ||
	    request->bit_depth_chroma_minus8 ||
	    request->log2_max_frame_num_minus4 > 12 ||
	    request->pic_order_cnt_type > 2 ||
	    request->log2_max_pic_order_cnt_lsb_minus4 > 12 ||
	    request->max_num_ref_frames > NVDEC_H264_DPB_ENTRIES ||
	    ((request->flags & NVDEC_H264_REQ_MBAFF) &&
	     (request->flags & (NVDEC_H264_REQ_FRAME_MBS_ONLY |
				NVDEC_H264_REQ_FIELD))) ||
	    (!(request->flags & NVDEC_H264_REQ_FIELD) &&
	     (request->flags & (NVDEC_H264_REQ_BOTTOM_FIELD |
				NVDEC_H264_REQ_SECOND_FIELD))) ||
	    request->pic_width_in_mbs * 16 != request->frame.coded_width ||
	    request->frame_height_in_mbs * 16 != request->frame.coded_height ||
	    request->num_slice_groups_minus1 ||
	    request->weighted_bipred_idc > 2 ||
	    request->frame.output_payload_size < 3 ||
	    request->frame.output_payload_size > U32_MAX - 16 ||
	    !request->slice_count ||
	    request->slice_count > (u32)request->pic_width_in_mbs *
				   request->frame_height_in_mbs) {
		dev_dbg(dev, "h264 reject: syntax/output\n");
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
		if (!request->dpb[i].fields || request->dpb[i].fields > 3 ||
		    !nvdec_map_is_valid(dpb[i], DMA_TO_DEVICE, surface_size)) {
			dev_dbg(dev, "h264 reject: dpb %u\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

/* An index is needed only while the surface is current or a reference. */
static void nvdec_recycle_indices(struct nvdec_decode_context *ctx,
				  struct nvdec_decode_map *cur,
				  struct nvdec_decode_map * const *dpb,
				  unsigned int entries)
{
	unsigned int i, j;

	for (i = 0; i < NVDEC_MAX_PICTURES; i++) {
		if (!ctx->surfaces[i].map || ctx->surfaces[i].map == cur)
			continue;
		for (j = 0; j < entries; j++)
			if (ctx->surfaces[i].map == dpb[j])
				break;
		if (j < entries)
			continue;
		nvdec_decode_map_put(ctx->surfaces[i].map);
		ctx->surfaces[i].map = NULL;
	}
}

static int nvdec_surface_index(struct nvdec_decode_context *ctx,
			       struct nvdec_decode_map *map, u8 *index,
			       unsigned int slots)
{
	unsigned int i;

	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++) {
		if (ctx->surfaces[i].map == map) {
			*index = ctx->surfaces[i].picture_index;
			if (*index < slots)
				return 0;
			dev_dbg(ctx->dev,
				"picture index %u is outside the %u firmware slots\n",
				*index, slots);
			return -ENOSPC;
		}
	}

	for (i = 0; i < slots; i++) {
		if (!ctx->surfaces[i].map) {
			ctx->surfaces[i].map = nvdec_decode_map_get(map);
			ctx->surfaces[i].picture_index = i;
			ctx->surfaces[i].dpb_slot = NVDEC_H264_DPB_ENTRIES;
			*index = i;
			return 0;
		}
	}

	dev_dbg(ctx->dev, "no free picture index among %u slots\n",
		slots);
	return -ENOSPC;
}

/* A slot names the same picture for as long as that picture is a reference. */
static int nvdec_h264_dpb_slot(struct nvdec_decode_context *ctx,
			       struct nvdec_decode_map *map, u8 *slot)
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
			dev_dbg(ctx->dev, "no free dpb slot\n");
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
	u32 top = (dpb->fields & NVDEC_H264_REF_TOP) ? marking : 0;
	u32 bottom = (dpb->fields & NVDEC_H264_REF_BOTTOM) ? marking : 0;

	return FIELD_PREP(NVDEC_H264_DPB_INDEX, picture_index) |
	       FIELD_PREP(NVDEC_H264_DPB_COL_IDX, picture_index) |
	       FIELD_PREP(NVDEC_H264_DPB_STATE, dpb->fields) |
	       (dpb->long_term ? NVDEC_H264_DPB_LONG_TERM : 0) |
	       (dpb->field_picture ? NVDEC_H264_DPB_IS_FIELD : 0) |
	       FIELD_PREP(NVDEC_H264_DPB_TOP_MARKING, top) |
	       FIELD_PREP(NVDEC_H264_DPB_BOTTOM_MARKING, bottom);
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
	struct nvdec_h264_setup *setup = hjob->ctx->state->cpu;
	u32 picture_flags, current_picture;

	setup->stream_len = cpu_to_le32(request->frame.output_payload_size + 16);
	setup->slice_count = cpu_to_le32(request->slice_count);
	setup->mbhist_buffer_size = cpu_to_le32(ctx->mbhist_size);
	setup->log2_max_pic_order_cnt_lsb_minus4 =
		cpu_to_le32(request->log2_max_pic_order_cnt_lsb_minus4);
	setup->delta_pic_order_always_zero_flag =
		cpu_to_le32(!!(request->flags & NVDEC_H264_REQ_DELTA_POC_ZERO));
	setup->frame_mbs_only_flag =
		cpu_to_le32(!!(request->flags & NVDEC_H264_REQ_FRAME_MBS_ONLY));
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

	picture_flags = FIELD_PREP(NVDEC_H264_PIC_LOG2_MAX_FRAME_NUM,
				   request->log2_max_frame_num_minus4) |
			FIELD_PREP(NVDEC_H264_PIC_CHROMA_FORMAT,
				   request->chroma_format_idc) |
			FIELD_PREP(NVDEC_H264_PIC_POC_TYPE,
				   request->pic_order_cnt_type) |
			FIELD_PREP(NVDEC_H264_PIC_INIT_QP,
				   (u32)request->pic_init_qp_minus26 & 0x3f) |
			FIELD_PREP(NVDEC_H264_PIC_CHROMA_QP,
				   (u32)request->chroma_qp_index_offset & 0x1f) |
			FIELD_PREP(NVDEC_H264_PIC_SECOND_CHROMA_QP,
				   (u32)request->second_chroma_qp_index_offset & 0x1f);
	if (request->flags & NVDEC_H264_REQ_MBAFF)
		picture_flags |= NVDEC_H264_PIC_MBAFF;
	if (request->flags & NVDEC_H264_REQ_DIRECT_8X8)
		picture_flags |= NVDEC_H264_PIC_DIRECT_8X8;
	if (request->pps_flags & NVDEC_H264_PPS_WEIGHTED_PRED)
		picture_flags |= NVDEC_H264_PIC_WEIGHTED_PRED;
	if (request->pps_flags & NVDEC_H264_PPS_CONSTRAINED_INTRA)
		picture_flags |= NVDEC_H264_PIC_CONSTRAINED_INTRA;
	if (request->nal_ref_idc)
		picture_flags |= NVDEC_H264_PIC_REFERENCE;
	if (request->flags & NVDEC_H264_REQ_FIELD)
		picture_flags |= NVDEC_H264_PIC_FIELD;
	if (request->flags & NVDEC_H264_REQ_BOTTOM_FIELD)
		picture_flags |= NVDEC_H264_PIC_BOTTOM_FIELD;
	if (request->flags & NVDEC_H264_REQ_SECOND_FIELD)
		picture_flags |= NVDEC_H264_PIC_SECOND_FIELD;
	setup->picture_flags = cpu_to_le32(picture_flags);

	current_picture = FIELD_PREP(NVDEC_H264_CUR_WEIGHTED_BIPRED,
				     request->weighted_bipred_idc) |
			  FIELD_PREP(NVDEC_H264_CUR_PIC_IDX, current_index) |
			  FIELD_PREP(NVDEC_H264_CUR_COL_IDX, current_index) |
			  FIELD_PREP(NVDEC_H264_CUR_FRAME_NUM, request->frame_num);
	setup->current_picture = cpu_to_le32(current_picture);
	setup->current_field_order_cnt[0] = cpu_to_le32(request->top_field_order_cnt);
	setup->current_field_order_cnt[1] = cpu_to_le32(request->bottom_field_order_cnt);
	setup->lossless_flags = cpu_to_le32(NVDEC_H264_IPRED8X8_FILTER);
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

	dev_dbg(hjob->ctx->dev, "h264 dpb=%s current=%u\n",
		dpb[0] ? dpb : "none", hjob->picture_index);
}

static int nvdec_h264_prepare(struct nvdec_decode_job *hjob)
{
	struct nvdec_h264_request *request = &hjob->req.h264;
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct device *dev = ctx->dev;
	u32 surface_size;
	unsigned int i;
	int err;

	request->slice_count = ctx->slice_count;
	err = nvdec_validate_frame(dev, &request->frame, 16, 1, hjob->surface,
				   hjob->capture, &surface_size);
	if (!err)
		err = nvdec_h264_validate_request(dev, request, hjob->dpb,
						  surface_size);
	if (!err)
		err = nvdec_h264_parse_slices(ctx, request);
	if (!err)
		err = nvdec_h264_prepare_scratch(ctx, request);
	if (err)
		return err;

	nvdec_recycle_indices(ctx, hjob->surface, hjob->dpb,
			      NVDEC_H264_DPB_ENTRIES);
	err = nvdec_surface_index(ctx, hjob->surface, &hjob->picture_index,
				  ctx->picture_slots);
	if (err)
		return err;
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (!request->dpb[i].valid)
			continue;
		err = nvdec_surface_index(ctx, hjob->dpb[i],
					  &hjob->picture_indices[i],
					  ctx->picture_slots);
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
			      hjob->ctx->input->iova + hjob->slice_offsets_off);
	falcon_gather_address(g, NVDEC_METHOD_COLOC, hjob->ctx->scratch->iova);
	falcon_gather_address(g, NVDEC_H264_METHOD_MBHIST,
			      hjob->ctx->scratch->iova + ctx->mbhist_offset);
	falcon_gather_address(g, NVDEC_METHOD_HISTORY,
			      hjob->ctx->scratch->iova + ctx->history_offset);
	nvdec_h264_debug_job(hjob);
}

static const u8 nvdec_h264_termination[16] = {
	0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
};

static const struct nvdec_codec_ops nvdec_h264_ops = {
	.application = NVDEC_APPLICATION_H264,
	.setup_size = NVDEC_H264_SETUP_SIZE,
	.status_offset = NVDEC_H264_STATUS_OFFSET,
	.vic_config = 0x400,
	.termination = nvdec_h264_termination,
	.prepare = nvdec_h264_prepare,
	.fill = nvdec_h264_fill_setup,
	.emit = nvdec_h264_emit,
};

static int nvdec_hevc_prepare_scratch(struct nvdec_decode_context *ctx,
				      const struct nvdec_hevc_request *request)
{
	u32 aligned_width, aligned_height, colmv, coloc, filter, size;
	struct nvdec_decode_map *scratch;

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

	scratch = nvdec_decode_surface_create(ctx, size);
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

/* The tile sizes the setup gets are nonzero and add up to the picture. */
static int nvdec_hevc_tiles_cover(const u16 *sizes, unsigned int count,
				  unsigned int ctbs)
{
	unsigned int i, sum = 0;

	for (i = 0; i < count; i++) {
		if (!sizes[i])
			return -EINVAL;
		sum += sizes[i];
	}

	return sum == ctbs ? 0 : -EINVAL;
}

static int nvdec_hevc_validate_request(struct device *dev,
				       const struct nvdec_hevc_request *request,
				       struct nvdec_decode_map * const dpb[],
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

	if ((request->bit_depth != 8 && request->bit_depth != 10) ||
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
	    request->log2_parallel_merge_level >
	    request->log2_max_luma_coding_block_size ||
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
	    !request->ctb_width || !request->ctb_height ||
	    nvdec_hevc_tiles_cover(request->column_width,
				   request->num_tile_columns, request->ctb_width) ||
	    nvdec_hevc_tiles_cover(request->row_height,
				   request->num_tile_rows, request->ctb_height)) {
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

/* V4L2 passes the lists row by row, the firmware takes them column by column. */
static void nvdec_hevc_fill_scaling_list(void *mem,
					 const struct nvdec_hevc_request *r)
{
	struct nvdec_hevc_scaling_list *list = mem;
	unsigned int i, x, y;

	memcpy(list->dc_16x16, r->scaling_dc_16x16, sizeof(list->dc_16x16));
	memcpy(list->dc_32x32, r->scaling_dc_32x32, sizeof(list->dc_32x32));
	for (i = 0; i < 6; i++) {
		for (y = 0; y < 4; y++)
			for (x = 0; x < 4; x++)
				list->list_4x4[i][x * 4 + y] = r->scaling_4x4[i][y * 4 + x];
		for (y = 0; y < 8; y++) {
			for (x = 0; x < 8; x++) {
				list->list_8x8[i][x * 8 + y] = r->scaling_8x8[i][y * 8 + x];
				list->list_16x16[i][x * 8 + y] = r->scaling_16x16[i][y * 8 + x];
				if (i < 2)
					list->list_32x32[i][x * 8 + y] =
						r->scaling_32x32[i][y * 8 + x];
			}
		}
	}
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
	struct nvdec_hevc_setup *setup = hjob->ctx->state->cpu;
	u8 order0[3 * NVDEC_HEVC_DPB_ENTRIES];
	u8 order1[3 * NVDEC_HEVC_DPB_ENTRIES];
	unsigned int i, n = 0, m = 0;
	u32 mask = BIT(current_index);
	u32 word;

	setup->stream_len = cpu_to_le32(r->frame.output_payload_size);
	word = FIELD_PREP(NVDEC_HEVC_SURFACE_START_CODE, 1);
	if (r->bit_depth > 8)
		word |= FIELD_PREP(NVDEC_HEVC_SURFACE_OUTPUT_MODE, 1);
	setup->surface_format = cpu_to_le32(word);
	/* 10-bit output counts the stride in samples, not bytes. */
	word = r->bit_depth > 8 ? r->frame.luma_stride / 2 : r->frame.luma_stride;
	setup->framestride[0] = cpu_to_le32(word);
	setup->framestride[1] = cpu_to_le32(word);
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
		nvdec_hevc_fill_scaling_list(hjob->ctx->state->cpu +
					     NVDEC_HEVC_SCALING_OFFSET, r);
	nvdec_hevc_fill_tile_sizes(hjob->ctx->state->cpu + NVDEC_HEVC_TILES_OFFSET, r);
}

static int nvdec_hevc_prepare(struct nvdec_decode_job *hjob)
{
	struct nvdec_hevc_request *request = &hjob->req.hevc;
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct device *dev = ctx->dev;
	unsigned int i, sample_bytes = request->bit_depth > 8 ? 2 : 1;
	int entry = nvdec_hevc_scratch_entry(request);
	u32 surface_size;
	int err;

	err = nvdec_validate_frame(dev, &request->frame, NVDEC_HEVC_CTU_SIZE,
				   sample_bytes, hjob->surface, hjob->capture,
				   &surface_size);
	if (!err)
		err = nvdec_hevc_validate_request(dev, request, hjob->dpb,
						  surface_size);
	if (!err)
		err = nvdec_hevc_prepare_scratch(ctx, request);
	if (err)
		return err;

	nvdec_recycle_indices(ctx, hjob->surface, hjob->dpb,
			      NVDEC_HEVC_DPB_ENTRIES);
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

	hjob->vic_passes = request->bit_depth > 8 ? 2 : 1;
	return 0;
}

static void nvdec_hevc_emit(struct nvdec_decode_job *hjob,
			    struct falcon_gather *g)
{
	dma_addr_t state = hjob->ctx->state->iova, scratch = hjob->ctx->scratch->iova;

	falcon_gather_address(g, NVDEC_HEVC_METHOD_SCALING_LIST,
			      state + NVDEC_HEVC_SCALING_OFFSET);
	falcon_gather_address(g, NVDEC_HEVC_METHOD_TILE_SIZES,
			      state + NVDEC_HEVC_TILES_OFFSET);
	falcon_gather_address(g, NVDEC_HEVC_METHOD_FILTER,
			      scratch + hjob->ctx->filter_offset);
	falcon_gather_address(g, NVDEC_METHOD_COLOC, scratch);
}

static const struct nvdec_codec_ops nvdec_hevc_ops = {
	.application = NVDEC_APPLICATION_HEVC,
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

/* HEVC alone has to start with 00 00 00 01; a lead zero byte extends 00 00 01. */
int nvdec_decode_stage(struct nvdec_decode_context *ctx, const void *bitstream,
		       u32 size)
{
	bool hevc = ctx->codec == NVDEC_CODEC_HEVC;
	u32 lead = 0, staged;
	int err;

	ctx->slice_count = 0;
	ctx->staged = 0;
	if (hevc) {
		if (size >= 3 && !memcmp(bitstream, "\x00\x00\x01", 3))
			lead = 1;
		else if (size < 4 || memcmp(bitstream, "\x00\x00\x00\x01", 4))
			return -EINVAL;
	}
	staged = size + lead;
	if (size < 3 || staged < size || staged > U32_MAX - 16 - SZ_256)
		return -EINVAL;

	err = nvdec_prepare_input(ctx, staged);
	if (err)
		return err;

	if (lead)
		*(u8 *)ctx->input->cpu = 0;
	memcpy(ctx->input->cpu + lead, bitstream, size);
	ctx->staged = staged;

	if (!nvdec_codec_ops[ctx->codec]->termination) {
		ctx->slice_count = 1;
		return 0;
	}

	/* The 16-byte terminator and the offset array follow the bitstream. */
	err = nvdec_scan_slices(ctx);
	if (err)
		return err;
	return nvdec_prepare_input(ctx, ALIGN(staged + 16, SZ_256) +
				   (ctx->slice_count + 1) * sizeof(u32));
}

static void nvdec_context_release(struct kref *ref)
{
	struct nvdec_decode_context *ctx = container_of(ref, struct nvdec_decode_context,
						       ref);
	unsigned int i;

	for (i = 0; i < NVDEC_MAX_PICTURES; i++)
		nvdec_decode_map_put(ctx->surfaces[i].map);
	nvdec_decode_map_put(ctx->input);
	nvdec_decode_map_put(ctx->scratch);
	nvdec_decode_map_put(ctx->state);
	nvdec_decode_map_put(ctx->gather);
	if (ctx->syncpt)
		host1x_syncpt_put(ctx->syncpt);
	kfree(ctx->slice_offsets);
	kfree(ctx);
}

struct nvdec_decode_context *
nvdec_decode_context_create(struct tegra_drm_client *client,
			    struct host1x_channel *channel, enum nvdec_codec codec)
{
	struct nvdec_decode_context *ctx;
	int err = -ENOMEM;

	/* Engine buffers live in the host1x domain, so the engine must too. */
	if (client->drm->domain && !client->base.group)
		return ERR_PTR(-ENODEV);

	/* A timed-out picture is only over once its engine has been stopped. */
	if (!client->base.ops->quiesce)
		return ERR_PTR(-ENODEV);

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	kref_init(&ctx->ref);
	ctx->client = client;
	ctx->channel = channel;
	ctx->dev = client->base.dev;
	ctx->codec = codec;
	ctx->fence_context = dma_fence_context_alloc(1);

	ctx->syncpt = host1x_syncpt_request(&client->base, 0);
	if (!ctx->syncpt)
		goto release;

	ctx->state = nvdec_buffer_create(ctx, NVDEC_STATE_SIZE, true);
	if (IS_ERR(ctx->state)) {
		err = PTR_ERR(ctx->state);
		ctx->state = NULL;
		goto release;
	}
	memset(ctx->state->cpu + NVDEC_VIC_FILTER, 0, VIC_FILTER_SIZE);

	ctx->gather = nvdec_buffer_create(ctx, NVDEC_GATHER_WORDS * sizeof(u32),
					  true);
	if (IS_ERR(ctx->gather)) {
		err = PTR_ERR(ctx->gather);
		ctx->gather = NULL;
		goto release;
	}

	return ctx;

release:
	kref_put(&ctx->ref, nvdec_context_release);
	return ERR_PTR(err);
}

void nvdec_decode_context_destroy(struct nvdec_decode_context *ctx)
{
	if (ctx)
		kref_put(&ctx->ref, nvdec_context_release);
}

void nvdec_decode_context_release_surface(struct nvdec_decode_context *ctx,
					  struct nvdec_decode_map *surface)
{
	unsigned int i;

	for (i = 0; i < NVDEC_MAX_PICTURES; i++) {
		if (ctx->surfaces[i].map != surface)
			continue;
		nvdec_decode_map_put(ctx->surfaces[i].map);
		ctx->surfaces[i].map = NULL;
		break;
	}
}

/* A timeout locks the syncpoint for good; the next picture takes a new one. */
static int nvdec_context_syncpt(struct nvdec_decode_context *ctx)
{
	if (!ctx->syncpt)
		ctx->syncpt = host1x_syncpt_request(&ctx->client->base, 0);

	return ctx->syncpt ? 0 : -ENOMEM;
}

/* P010 has no VIC format and is detiled one plane per pass. */
static void nvdec_fill_detile(struct nvdec_decode_job *hjob)
{
	const struct nvdec_frame *f = &hjob->req.frame;
	struct vic_detile params = {
		.width = f->coded_width,
		.height = f->coded_height,
		.left = f->crop_left,
		.top = f->crop_top,
		.out_width = f->crop_width,
		.out_height = f->crop_height,
		.src_stride = f->luma_stride,
		.dst_stride = f->dst_stride,
		.pass = VIC_DETILE_NV12,
	};
	unsigned int i;

	for (i = 0; i < hjob->vic_passes; i++) {
		if (hjob->vic_passes > 1)
			params.pass = i ? VIC_DETILE_P010_CHROMA :
					  VIC_DETILE_P010_LUMA;
		vic_fill_detile_config(hjob->ctx->state->cpu + hjob->ops->vic_config +
				       i * NVDEC_VIC_CONFIG_STRIDE, &params);
	}
}

/* The NVDEC methods, then in a second gather each VIC pass. */
static int nvdec_build_gathers(struct nvdec_decode_job *hjob,
			       unsigned int *nvdec_words,
			       unsigned int *vic_words)
{
	const struct nvdec_codec_ops *ops = hjob->ops;
	const struct nvdec_frame *f = &hjob->req.frame;
	struct nvdec_decode_context *ctx = hjob->ctx;
	u32 syncpt = host1x_syncpt_id(ctx->syncpt);
	dma_addr_t src_chroma = hjob->surface->iova + f->chroma_offset;
	dma_addr_t dst_chroma = hjob->capture->iova + f->dst_chroma_offset;
	dma_addr_t state = ctx->state->iova;
	struct falcon_gather g = {
		.words = ctx->gather->cpu,
		.size = NVDEC_VIC_GATHER_OFFSET / sizeof(u32),
	};
	struct falcon_gather v = {
		.words = ctx->gather->cpu + NVDEC_VIC_GATHER_OFFSET,
		.size = NVDEC_GATHER_WORDS - NVDEC_VIC_GATHER_OFFSET / sizeof(u32),
	};
	unsigned int i;

	falcon_gather_method(&g, NVDEC_METHOD_APPLICATION, ops->application);
	falcon_gather_method(&g, NVDEC_METHOD_CONTROL,
			     FIELD_PREP(NVDEC_CONTROL_CODEC_TYPE, ops->application) |
			     NVDEC_CONTROL_GPTIMER_ON | NVDEC_CONTROL_ERR_CONCEAL_ON);
	falcon_gather_method(&g, NVDEC_METHOD_PICTURE_INDEX, hjob->picture_index);
	falcon_gather_address(&g, NVDEC_METHOD_SETUP, state);
	falcon_gather_address(&g, NVDEC_METHOD_INPUT, ctx->input->iova);
	falcon_gather_address(&g, NVDEC_METHOD_STATUS, state + ops->status_offset);
	ops->emit(hjob, &g);
	for (i = 0; i < hjob->num_pictures; i++) {
		falcon_gather_address(&g, NVDEC_METHOD_LUMA + i,
				      hjob->pictures[i]->iova);
		falcon_gather_address(&g, NVDEC_METHOD_CHROMA + i,
				      hjob->pictures[i]->iova + f->chroma_offset);
	}
	falcon_gather_method(&g, NVDEC_METHOD_EXECUTE, NVDEC_EXECUTE_AWAKEN);
	falcon_gather_op_done(&g, syncpt);

	/* A P010 chroma pass takes the chroma planes through the luma methods. */
	for (i = 0; i < hjob->vic_passes; i++) {
		vic_emit_detile(&v, state + ops->vic_config +
				i * NVDEC_VIC_CONFIG_STRIDE, state + NVDEC_VIC_FILTER,
				i ? src_chroma : hjob->surface->iova, src_chroma,
				i ? dst_chroma : hjob->capture->iova, dst_chroma);
		falcon_gather_op_done(&v, syncpt);
	}

	print_hex_dump_debug("nvdec setup: ", DUMP_PREFIX_OFFSET, 16, 4,
			     ctx->state->cpu, ops->setup_size, false);
	*nvdec_words = g.count;
	*vic_words = v.count;
	return g.err ?: v.err;
}

/* The staged bitstream is followed by its terminator and the slice offsets. */
static void nvdec_write_slice_offsets(struct nvdec_decode_job *hjob)
{
	struct nvdec_decode_context *ctx = hjob->ctx;
	__le32 *offsets;
	unsigned int i;

	hjob->slice_offsets_off = ALIGN(ctx->staged + 16, SZ_256);
	memcpy(ctx->input->cpu + ctx->staged, hjob->ops->termination, 16);
	offsets = ctx->input->cpu + hjob->slice_offsets_off;
	for (i = 0; i < ctx->slice_count; i++)
		offsets[i] = cpu_to_le32(ctx->slice_offsets[i]);
	offsets[i] = cpu_to_le32(ctx->staged);
}

static void nvdec_job_signal(struct nvdec_decode_job *hjob, int err)
{
	if (atomic_xchg(&hjob->signalled, 1))
		return;

	if (err)
		dma_fence_set_error(hjob->fence, err);
	dma_fence_signal(hjob->fence);
}

/* VIC writes the CAPTURE buffer last, after waiting for NVDEC. */
static void nvdec_vic_fence_done(struct dma_fence *fence, struct dma_fence_cb *cb)
{
	struct nvdec_decode_job *hjob = container_of(cb, struct nvdec_decode_job,
						     vic_cb);

	nvdec_job_signal(hjob, fence->error);
}

static void nvdec_job_put_pending(struct nvdec_decode_job *hjob)
{
	if (atomic_dec_and_test(&hjob->pending))
		hjob->complete(hjob->complete_data);
}

/* Runs in host1x's job cleanup, which a channel stop waits for: only hands over. */
static void nvdec_host1x_job_release(struct host1x_job *job)
{
	struct nvdec_decode_job *hjob = job->user_data;

	if (job->cancelled) {
		if (job == hjob->vic_job)
			hjob->vic_cancelled = true;
		else
			hjob->nvdec_cancelled = true;
	}
	nvdec_job_put_pending(hjob);
}

static struct host1x_job *
nvdec_job_create(struct nvdec_decode_job *hjob, struct host1x_channel *channel,
		 struct tegra_drm_client *client, unsigned int offset,
		 unsigned int words, unsigned int incrs)
{
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct host1x_job *job;
	int err;

	job = host1x_job_alloc(channel, 1, 0, true);
	if (!job)
		return ERR_PTR(-ENOMEM);

	job->client = &client->base;
	job->class = client->base.class;
	job->serialize = true;
	job->syncpt = host1x_syncpt_get(ctx->syncpt);
	job->syncpt_incrs = incrs;
	job->timeout = NVDEC_JOB_TIMEOUT_MS;
	host1x_job_add_gather(job, &ctx->gather->bo->base, words, offset);

	err = host1x_job_pin(job, client->base.dev);
	if (err) {
		host1x_job_put(job);
		return ERR_PTR(err);
	}

	return job;
}

static void nvdec_job_free(struct nvdec_decode_job *hjob)
{
	unsigned int i;

	if (hjob->runtime_ref) {
		pm_runtime_mark_last_busy(hjob->ctx->dev);
		pm_runtime_put_autosuspend(hjob->ctx->dev);
	}
	if (hjob->vic_runtime_ref) {
		pm_runtime_mark_last_busy(hjob->vic->base.dev);
		pm_runtime_put_autosuspend(hjob->vic->base.dev);
	}
	if (hjob->fence)
		dma_fence_put(hjob->fence);
	if (hjob->vic_fence) {
		dma_fence_remove_callback(hjob->vic_fence, &hjob->vic_cb);
		dma_fence_put(hjob->vic_fence);
	}
	nvdec_decode_map_put(hjob->surface);
	nvdec_decode_map_put(hjob->capture);
	for (i = 0; i < NVDEC_MAX_REFS; i++)
		nvdec_decode_map_put(hjob->dpb[i]);
	kref_put(&hjob->ctx->ref, nvdec_context_release);
	kfree(hjob);
}

static struct dma_fence *nvdec_fence_create(struct nvdec_decode_context *ctx)
{
	struct nvdec_fence *fence;

	fence = kzalloc_obj(*fence);
	if (!fence)
		return NULL;

	spin_lock_init(&fence->lock);
	dma_fence_init(&fence->base, &nvdec_fence_ops, &fence->lock,
		       ctx->fence_context, ++ctx->fence_seqno);
	return &fence->base;
}

/* The CAPTURE fence may not have readers yet, unless they are done. */
static int nvdec_resv_wait_locked(struct dma_resv *resv)
{
	long ret;

	if (dma_resv_test_signaled(resv, DMA_RESV_USAGE_READ))
		return 0;

	ret = dma_resv_wait_timeout(resv, DMA_RESV_USAGE_READ, false,
				    msecs_to_jiffies(NVDEC_JOB_TIMEOUT_MS));
	return ret > 0 ? 0 : ret ?: -ETIMEDOUT;
}

/* Allocate, publish the fence once CAPTURE's readers are done, then submit. */
static int nvdec_launch(struct nvdec_decode_job *hjob)
{
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct dma_resv *resv = hjob->capture->resv;
	unsigned int nvdec_words, vic_words;
	struct host1x_job *job, *vjob;
	bool cookie;
	int err;

	err = nvdec_build_gathers(hjob, &nvdec_words, &vic_words);
	if (err)
		return err;

	hjob->fence = nvdec_fence_create(ctx);
	if (!hjob->fence)
		return -ENOMEM;

	err = pm_runtime_resume_and_get(ctx->dev);
	if (err < 0)
		return err;
	hjob->runtime_ref = true;
	err = pm_runtime_resume_and_get(hjob->vic->base.dev);
	if (err < 0)
		return err;
	hjob->vic_runtime_ref = true;

	job = nvdec_job_create(hjob, ctx->channel, ctx->client, 0, nvdec_words, 1);
	if (IS_ERR(job))
		return PTR_ERR(job);

	vjob = nvdec_job_create(hjob, hjob->vic->shared_channel, hjob->vic,
				NVDEC_VIC_GATHER_OFFSET, vic_words,
				hjob->vic_passes);
	if (IS_ERR(vjob)) {
		err = PTR_ERR(vjob);
		goto put_job;
	}

	/* Locked until published, so no reader can queue behind our jobs first. */
	if (resv) {
		dma_resv_lock(resv, NULL);
		err = dma_resv_reserve_fences(resv, 1);
		if (!err)
			err = nvdec_resv_wait_locked(resv);
		if (err) {
			dma_resv_unlock(resv);
			goto put_vjob;
		}
	}

	cookie = dma_fence_begin_signalling();
	if (resv) {
		dma_resv_add_fence(resv, hjob->fence, DMA_RESV_USAGE_WRITE);
		dma_resv_unlock(resv);
	}

	/* VIC waits on the same syncpoint for the NVDEC job, see serialize. */
	err = host1x_job_submit(job);
	if (err) {
		nvdec_job_signal(hjob, err);
		dma_fence_end_signalling(cookie);
		goto put_vjob;
	}
	hjob->nvdec_job = job;
	job->user_data = hjob;
	job->release = nvdec_host1x_job_release;
	atomic_inc(&hjob->pending);

	err = host1x_job_submit(vjob);
	if (err) {
		/* NVDEC only touches kernel buffers: let it run, the picture fails. */
		hjob->failed = true;
		nvdec_job_signal(hjob, err);
		host1x_job_unpin(vjob);
	} else {
		hjob->vic_job = vjob;
		vjob->user_data = hjob;
		vjob->release = nvdec_host1x_job_release;
		atomic_inc(&hjob->pending);

		hjob->vic_fence = dma_fence_get(vjob->fence);
		err = dma_fence_add_callback(hjob->vic_fence, &hjob->vic_cb,
					     nvdec_vic_fence_done);
		if (err)
			nvdec_job_signal(hjob, hjob->vic_fence->error);
	}
	dma_fence_end_signalling(cookie);

	host1x_job_put(vjob);
	host1x_job_put(job);
	return 0;

put_vjob:
	host1x_job_unpin(vjob);
	host1x_job_put(vjob);
put_job:
	host1x_job_unpin(job);
	host1x_job_put(job);
	return err;
}

int nvdec_decode_submit(struct nvdec_decode_context *ctx,
			const union nvdec_request *request,
			struct nvdec_decode_map *surface,
			struct nvdec_decode_map *capture,
			struct nvdec_decode_map * const refs[NVDEC_MAX_REFS],
			nvdec_decode_complete_t complete, void *data)
{
	struct nvdec_decode_job *hjob;
	unsigned int i;
	int err;

	if (!ctx->slice_count)
		return -EINVAL;

	hjob = kzalloc_obj(*hjob);
	if (!hjob)
		return -ENOMEM;

	kref_get(&ctx->ref);
	hjob->ctx = ctx;
	hjob->ops = nvdec_codec_ops[ctx->codec];
	hjob->req = *request;
	hjob->req.frame.output_payload_size = ctx->staged;
	hjob->complete = complete;
	hjob->complete_data = data;
	hjob->vic_passes = 1;
	atomic_set(&hjob->pending, 1);
	hjob->surface = nvdec_decode_map_get(surface);
	hjob->capture = nvdec_decode_map_get(capture);
	for (i = 0; i < NVDEC_MAX_REFS; i++) {
		if (refs[i])
			hjob->dpb[i] = nvdec_decode_map_get(refs[i]);
	}

	err = hjob->ops->prepare(hjob);
	if (err)
		goto free;

	hjob->vic = vic_find_detiler(ctx->client->drm);
	if (!hjob->vic || !hjob->vic->base.ops->quiesce) {
		err = -ENODEV;
		goto free;
	}

	err = nvdec_context_syncpt(ctx);
	if (err)
		goto free;

	memset(ctx->state->cpu, 0, hjob->ops->vic_config +
	       hjob->vic_passes * NVDEC_VIC_CONFIG_STRIDE);
	if (hjob->ops->termination)
		nvdec_write_slice_offsets(hjob);
	hjob->ops->fill(hjob);
	nvdec_fill_detile(hjob);

	err = nvdec_launch(hjob);
	if (err)
		goto free;

	ctx->job = hjob;
	nvdec_job_put_pending(hjob);
	return 0;

free:
	nvdec_job_free(hjob);
	return err;
}

/* After complete(): recover a hung engine, then report how the picture went. */
int nvdec_decode_finish(struct nvdec_decode_context *ctx)
{
	struct nvdec_decode_job *hjob = ctx->job;
	const struct nvdec_status *status;
	int err = 0;

	if (!hjob)
		return -EINVAL;
	ctx->job = NULL;

	/* The fence has signalled: host1x stopped the engines before cancelling. */
	if (hjob->nvdec_cancelled || hjob->vic_cancelled) {
		/* Freed, and so idle again, once host1x releases both jobs. */
		host1x_syncpt_put(ctx->syncpt);
		ctx->syncpt = NULL;
		/* VIC waits for NVDEC, so a hung NVDEC times VIC out as well. */
		dev_warn(ctx->dev, "%s timed out\n",
			 hjob->nvdec_cancelled ? "NVDEC" : "VIC");
		err = -ETIMEDOUT;
	} else if (hjob->failed) {
		err = -EIO;
	} else {
		status = ctx->state->cpu + hjob->ops->status_offset;
		dev_dbg(ctx->dev, "status: ok=%u error=%u cycles=%u code=0x%x\n",
			le32_to_cpu(status->mbs_correctly_decoded),
			le32_to_cpu(status->mbs_in_error),
			le32_to_cpu(status->cycle_count),
			le32_to_cpu(status->error_status));
		/* Decoded with concealment, which the client still has to know. */
		if (status->mbs_in_error || status->error_status)
			err = -EBADMSG;
	}

	nvdec_job_free(hjob);
	return err;
}
