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
#include "nvdec-engine.h"
#include "riscv.h"
#include "vic.h"
#include "vic-engine.h"

#define NVDEC_FALCON_DEBUGINFO			0x1094
#define NVDEC_TFBIF_TRANSCFG			0x2c44

#define NVDEC_H264_SETUP_SIZE			0x2fc
#define NVDEC_H264_STATUS_OFFSET		0x300
#define NVDEC_H264_VIC_CONFIG_OFFSET		0x400
#define NVDEC_H264_STATE_SIZE			0x1000
#define NVDEC_H264_GATHER_WORDS		135
#define NVDEC_DONE_WORDS			2
#define NVDEC_H264_VIC_OFFSET			(NVDEC_H264_GATHER_WORDS + \
						 NVDEC_DONE_WORDS)
#define NVDEC_H264_TOTAL_GATHER_WORDS		(NVDEC_H264_VIC_OFFSET + \
						 VIC_DETILE_WORDS + \
						 NVDEC_DONE_WORDS)

#define NVDEC_METHOD_INCR			0x10100002
#define NVDEC_METHOD_APPLICATION		0x080
#define NVDEC_METHOD_CONTROL		0x100
#define NVDEC_METHOD_PICTURE_INDEX	0x103
#define NVDEC_METHOD_SETUP		0x101
#define NVDEC_METHOD_INPUT		0x102
#define NVDEC_H264_METHOD_SLICE_OFFSETS	0x104
#define NVDEC_METHOD_STATUS		0x109
#define NVDEC_METHOD_COLOC		0x105
#define NVDEC_H264_METHOD_MBHIST		0x140
#define NVDEC_H264_METHOD_HISTORY		0x106
#define NVDEC_METHOD_LUMA			0x10c
#define NVDEC_METHOD_CHROMA		0x11d
#define NVDEC_METHOD_EXECUTE		0x0c0
#define NVDEC_HEVC_METHOD_SCALING_LIST		0x160
#define NVDEC_HEVC_METHOD_TILE_SIZES		0x161
#define NVDEC_HEVC_METHOD_FILTER		0x162
#define NVDEC_VP8_METHOD_PROB_DATA		0x150
#define NVDEC_VP9_METHOD_PROB_TAB		0x170
#define NVDEC_VP9_METHOD_CTX_COUNTER		0x171
#define NVDEC_VP9_METHOD_SEGMENT_READ		0x172
#define NVDEC_VP9_METHOD_SEGMENT_WRITE		0x173
#define NVDEC_VP9_METHOD_TILE_SIZE		0x174
#define NVDEC_VP9_METHOD_COL_MVWRITE		0x175
#define NVDEC_VP9_METHOD_COL_MVREAD		0x176
#define NVDEC_VP9_METHOD_FILTER		0x177

#define NVDEC_HEVC_SETUP_SIZE			0x114
#define NVDEC_HEVC_STATUS_OFFSET		0x200
#define NVDEC_HEVC_SCALING_OFFSET		0x300
#define NVDEC_HEVC_TILES_OFFSET		0x700
#define NVDEC_HEVC_TILES_SIZE			0x900
#define NVDEC_HEVC_VIC_CONFIG_OFFSET		0x1000
#define NVDEC_HEVC_STATE_SIZE			0x2000
#define NVDEC_HEVC_GATHER_WORDS		129
#define NVDEC_HEVC_VIC_OFFSET			(NVDEC_HEVC_GATHER_WORDS + \
						 NVDEC_DONE_WORDS)
#define NVDEC_HEVC_TOTAL_GATHER_WORDS		(NVDEC_HEVC_VIC_OFFSET + \
						 VIC_DETILE_WORDS + \
						 NVDEC_DONE_WORDS)

#define NVDEC_VP8_SETUP_SIZE			0xc0
#define NVDEC_VP8_STATUS_OFFSET		0x100
#define NVDEC_VP8_VIC_CONFIG_OFFSET		0x400
#define NVDEC_VP8_STATE_SIZE			0x1000
#define NVDEC_VP8_GATHER_WORDS			51
#define NVDEC_VP8_VIC_OFFSET			(NVDEC_VP8_GATHER_WORDS + \
						 NVDEC_DONE_WORDS)
#define NVDEC_VP8_TOTAL_GATHER_WORDS		(NVDEC_VP8_VIC_OFFSET + \
						 VIC_DETILE_WORDS + \
						 NVDEC_DONE_WORDS)

/* Firmware picture slots: golden, altref, last, current. */
#define NVDEC_VP8_MAX_PICTURES			4
#define NVDEC_VP8_HISTORY_PER_MB		0x200
#define NVDEC_VP8_PROB_SIZE			0x4b00

#define NVDEC_VP9_SETUP_SIZE			0x100
#define NVDEC_VP9_STATUS_OFFSET		0x100
#define NVDEC_VP9_PROBS_OFFSET			0x200
#define NVDEC_VP9_TILES_OFFSET			0x1100
#define NVDEC_VP9_TILES_SIZE			0x700
/* Two unexplained u16 constants the oracle plants in the tile-size buffer. */
#define NVDEC_VP9_TILES_MAGIC			0x37a
#define NVDEC_VP9_VIC_CONFIG_OFFSET		0x1800
#define NVDEC_VP9_STATE_SIZE			0x2000
#define NVDEC_VP9_GATHER_WORDS			69
#define NVDEC_VP9_VIC_OFFSET			(NVDEC_VP9_GATHER_WORDS + \
						 NVDEC_DONE_WORDS)
#define NVDEC_VP9_TOTAL_GATHER_WORDS		(NVDEC_VP9_VIC_OFFSET + \
						 VIC_DETILE_WORDS + \
						 NVDEC_DONE_WORDS)

#define NVDEC_VP9_FILTER_PER_ROW		988
#define NVDEC_VP9_BSD_PER_ROW			912

/* Per aligned luma row, as the oracle sizes them. */
#define NVDEC_MAX_DPB_ENTRIES			NVDEC_H264_DPB_ENTRIES

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

struct nvdec_vp8_setup {
	u8 encryption[0x34];
	__le32 gptimer_timeout_value;
	__le16 frame_width;
	__le16 frame_height;
	u8 key_frame;
	u8 version;
	u8 surface_format;
	u8 error_conceal_on;
	__le32 first_part_size;
	__le32 history_buffer_size;
	__le32 vld_buffer_size;
	__le32 framestride[2];
	__le32 luma_top_offset;
	__le32 luma_bot_offset;
	__le32 luma_frame_offset;
	__le32 chroma_top_offset;
	__le32 chroma_bot_offset;
	__le32 chroma_frame_offset;
	u8 display[0x1c];
	u8 current_output_memory_layout;
	u8 output_memory_layout[3];
	u8 segmentation_feature_data_update;
	u8 reserved1[3];
	__le32 result_value;
	__le32 partition_offset[8];
	u8 ssm[0xc];
};

static_assert(offsetof(struct nvdec_vp8_setup, frame_width) == 0x38);
static_assert(offsetof(struct nvdec_vp8_setup, first_part_size) == 0x40);
static_assert(offsetof(struct nvdec_vp8_setup, framestride) == 0x4c);
static_assert(offsetof(struct nvdec_vp8_setup, display) == 0x6c);
static_assert(offsetof(struct nvdec_vp8_setup, segmentation_feature_data_update) == 0x8c);
static_assert(sizeof(struct nvdec_vp8_setup) == NVDEC_VP8_SETUP_SIZE);

/* The entropy context the firmware reads, updates and writes back. */
struct nvdec_vp8_probs {
	u8 coeff[4][8][3][12];
	u8 y_mode[4];
	u8 uv_mode[4];
	u8 mv[2][20];
	u8 reserved[0x1c];
};

static_assert(offsetof(struct nvdec_vp8_probs, y_mode) == 0x480);
static_assert(offsetof(struct nvdec_vp8_probs, mv) == 0x488);
static_assert(sizeof(struct nvdec_vp8_probs) == 0x4cc);

#define NVDEC_VP8_SURFACE_TILEFORMAT		GENMASK(1, 0)
#define NVDEC_VP8_SURFACE_GOB_HEIGHT		GENMASK(4, 2)

struct nvdec_vp9_reference {
	__le16 width;
	__le16 height;
	__le16 stride[2];
};

struct nvdec_vp9_setup {
	u8 encryption[0x30];
	__le32 stream_len;
	__le32 enable_encryption;
	__le32 key_control;
	__le32 gptimer_timeout_value;
	__le32 surface_format;
	__le32 bsd_control_offset;
	struct nvdec_vp9_reference ref[3];
	__le16 width;
	__le16 height;
	__le16 framestride[2];
	__le32 picture_flags;
	u8 ref_frame_sign_bias[4];
	s8 loop_filter_level;
	s8 loop_filter_sharpness;
	u8 qp_y_ac;
	s8 qp_y_dc;
	s8 qp_ch_ac;
	s8 qp_ch_dc;
	s8 lossless;
	s8 transform_mode;
	s8 allow_high_precision_mv;
	s8 mcomp_filter_type;
	s8 comp_pred_mode;
	s8 comp_fixed_ref;
	s8 comp_var_ref[2];
	s8 log2_tile_columns;
	s8 log2_tile_rows;
	u8 segment_enabled;
	u8 segment_map_update;
	u8 segment_map_temporal_update;
	u8 segment_feature_mode;
	u8 segment_feature_enable[8][4];
	__le16 segment_feature_data[8][4];
	s8 mode_ref_lf_enabled;
	s8 mb_ref_lf_delta[4];
	s8 mb_mode_lf_delta[2];
	s8 reserved;
	u8 v1[8];
	u8 ssm[0xc];
};

static_assert(offsetof(struct nvdec_vp9_setup, bsd_control_offset) == 0x44);
static_assert(offsetof(struct nvdec_vp9_setup, width) == 0x60);
static_assert(offsetof(struct nvdec_vp9_setup, picture_flags) == 0x68);
static_assert(offsetof(struct nvdec_vp9_setup, qp_y_ac) == 0x72);
static_assert(offsetof(struct nvdec_vp9_setup, segment_feature_enable) == 0x84);
static_assert(offsetof(struct nvdec_vp9_setup, mode_ref_lf_enabled) == 0xe4);
static_assert(sizeof(struct nvdec_vp9_setup) == NVDEC_VP9_SETUP_SIZE);

#define NVDEC_VP9_PIC_KEY_FRAME		BIT(0)
#define NVDEC_VP9_PIC_PREV_KEY_FRAME		BIT(1)
#define NVDEC_VP9_PIC_RESOLUTION_CHANGE	BIT(2)
#define NVDEC_VP9_PIC_ERROR_RESILIENT		BIT(3)
#define NVDEC_VP9_PIC_PREV_SHOW_FRAME		BIT(4)
#define NVDEC_VP9_PIC_INTRA_ONLY		BIT(5)

struct nvdec_vp9_nmv_probs {
	u8 joints[3];
	u8 sign[2];
	u8 class0[2][1];
	u8 fp[2][3];
	u8 class0_hp[2];
	u8 hp[2];
	u8 classes[2][10];
	u8 class0_fp[2][2][3];
	u8 bits[2][10];
};

struct nvdec_vp9_probs {
	u8 kf_bmode_prob[10][10][8];
	u8 kf_bmode_prob_b[10][10][1];
	u8 ref_pred_probs[3];
	u8 mb_segment_tree_probs[7];
	u8 segment_pred_probs[3];
	u8 ref_scores[4];
	u8 prob_comppred[2];
	u8 pad0[9];
	u8 kf_uv_mode_prob[10][8];
	u8 kf_uv_mode_prob_b[10][1];
	u8 pad1[6];
	u8 inter_mode_prob[7][4];
	u8 intra_inter_prob[4];
	u8 uv_mode_prob[10][8];
	u8 tx8x8_prob[2][1];
	u8 tx16x16_prob[2][2];
	u8 tx32x32_prob[2][3];
	u8 sb_ymode_prob_b[4][1];
	u8 sb_ymode_prob[4][8];
	u8 partition_prob[2][16][4];
	u8 uv_mode_prob_b[10][1];
	u8 switchable_interp_prob[4][2];
	u8 comp_inter_prob[5];
	u8 mbskip_probs[3];
	u8 pad2[1];
	struct nvdec_vp9_nmv_probs nmvc;
	u8 single_ref_prob[5][2];
	u8 comp_ref_prob[5];
	u8 pad3[17];
	u8 coeff[4][2][2][6][6][4];
};

static_assert(offsetof(struct nvdec_vp9_probs, kf_uv_mode_prob) == 0x3a0);
static_assert(offsetof(struct nvdec_vp9_probs, inter_mode_prob) == 0x400);
static_assert(offsetof(struct nvdec_vp9_probs, partition_prob) == 0x4a0);
static_assert(offsetof(struct nvdec_vp9_probs, nmvc) == 0x53b);
static_assert(offsetof(struct nvdec_vp9_probs, coeff) == 0x5a0);
static_assert(sizeof(struct nvdec_vp9_probs) == 0xea0);

/* Written by the firmware; v4l2_vp9_frame_symbol_counts points into it. */
struct nvdec_vp9_nmv_counts {
	u32 joints[4];
	u32 sign[2][2];
	u32 classes[2][11];
	u32 class0[2][2];
	u32 bits[2][10][2];
	u32 class0_fp[2][2][4];
	u32 fp[2][4];
	u32 class0_hp[2][2];
	u32 hp[2][2];
};

struct nvdec_vp9_counts {
	u32 inter_mode[7][3][2];
	u32 y_mode[4][10];
	u32 uv_mode[10][10];
	u32 partition[16][4];
	u32 interp_filter[4][3];
	u32 intra_inter[4][2];
	u32 comp_inter[5][2];
	u32 single_ref[5][2][2];
	u32 comp_ref[5][2];
	u32 tx32x32[2][4];
	u32 tx16x16[2][3];
	u32 tx8x8[2][2];
	u32 skip[3][2];
	struct nvdec_vp9_nmv_counts mv;
	u32 coeff[4][2][2][6][6][4];
	u32 eob[4][2][2][6][6];
};

static_assert(offsetof(struct nvdec_vp9_counts, mv) == 0x528);
static_assert(offsetof(struct nvdec_vp9_counts, coeff) == 0x6d0);
static_assert(offsetof(struct nvdec_vp9_counts, eob) == 0x2ad0);
static_assert(sizeof(struct nvdec_vp9_counts) == 0x33d0);

struct nvdec_buffer {
	struct host1x_bo bo;
	struct kref ref;
	struct nvdec_engine *engine;
	struct device *dev;
	void *cpu;
	dma_addr_t dma;
	dma_addr_t iova;
	size_t size;
	struct drm_mm_node *mm;
	size_t mapped;
};

struct nvdec_engine_map {
	struct kref ref;
	struct nvdec_engine *engine;
	/* A kernel-owned surface has no dma_buf; cpu/dma describe it instead. */
	struct dma_buf *dmabuf;
	void *cpu;
	dma_addr_t dma;
	struct page **pages;
	unsigned int npages;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t iova;
	unsigned long offset;
	size_t size;
	enum dma_data_direction direction;
	struct drm_mm_node *mm;
	size_t mapped;
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
	struct nvdec_buffer *input;
	/* VP8 entropy context, seeded by the driver and updated by firmware. */
	struct nvdec_buffer *probs;
	/* VP9 symbol counts the firmware writes, plus its two odd members. */
	struct nvdec_buffer *counts;
	u32 mv_mode[7][4];
	u32 tx16p[2][4];
	u32 seg_read_offset;
	u32 seg_write_offset;
	u32 colmv_offset[2];
	bool frame_parity;
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
	struct nvdec_pool_surface surfaces[NVDEC_H264_MAX_PICTURES];
};

struct nvdec_decode_job {
	struct nvdec_decode_context *ctx;
	struct nvdec_h264_request request;
	struct nvdec_hevc_request hevc;
	struct nvdec_vp8_request vp8;
	struct nvdec_vp9_request vp9;
	struct nvdec_buffer *state;
	struct nvdec_buffer *input;
	struct nvdec_buffer *probs;
	struct nvdec_buffer *counts;
	struct nvdec_buffer *gather;
	struct nvdec_engine_map *scratch;
	struct nvdec_engine_map *surface;
	struct nvdec_engine_map *capture;
	struct nvdec_engine_map *dpb[NVDEC_H264_DPB_ENTRIES];
	struct nvdec_engine_map *scratch_ref;
	struct vic_engine *vic;
	u32 slice_offsets_off;
	struct dma_fence *fence;
	nvdec_engine_complete_t complete;
	void *complete_data;
	bool pinned;
	bool runtime_ref;
	bool vic_runtime_ref;
	bool submitted;
	bool completion_armed;
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

/* The DMA API returns physical addresses here; NVDEC's methods are 32-bit. */
static struct tegra_drm *nvdec_engine_tegra_iommu(struct nvdec_engine *engine)
{
	struct drm_device *drm;
	struct tegra_drm *tegra;

	if (!engine || !engine->client.base.group || !engine->client.base.host)
		return NULL;

	drm = dev_get_drvdata(engine->client.base.host);
	tegra = drm ? drm->dev_private : NULL;

	return (tegra && tegra->domain) ? tegra : NULL;
}

static int nvdec_engine_iommu_map(struct tegra_drm *tegra, struct sg_table *sgt,
				  size_t size, struct drm_mm_node **node,
				  size_t *mapped, dma_addr_t *iova)
{
	struct drm_mm_node *mm;
	ssize_t bytes;
	int err;

	mm = kzalloc_obj(*mm);
	if (!mm)
		return -ENOMEM;

	mutex_lock(&tegra->mm_lock);

	err = drm_mm_insert_node_generic(&tegra->mm, mm, PAGE_ALIGN(size),
					 PAGE_SIZE, 0, 0);
	if (err < 0)
		goto unlock;

	bytes = iommu_map_sgtable(tegra->domain, mm->start, sgt,
				  IOMMU_READ | IOMMU_WRITE);
	if (bytes <= 0) {
		drm_mm_remove_node(mm);
		err = bytes < 0 ? bytes : -ENOMEM;
		goto unlock;
	}

	mutex_unlock(&tegra->mm_lock);

	*node = mm;
	*mapped = bytes;
	*iova = mm->start;
	return 0;

unlock:
	mutex_unlock(&tegra->mm_lock);
	kfree(mm);
	return err;
}

static void nvdec_engine_iommu_unmap(struct tegra_drm *tegra,
				     struct drm_mm_node *node, size_t mapped)
{
	if (!node)
		return;

	mutex_lock(&tegra->mm_lock);
	iommu_unmap(tegra->domain, node->start, mapped);
	drm_mm_remove_node(node);
	mutex_unlock(&tegra->mm_lock);
	kfree(node);
}

static struct host1x_bo *nvdec_buffer_get(struct host1x_bo *bo)
{
	struct nvdec_buffer *buffer = container_of(bo, struct nvdec_buffer,
							  bo);

	kref_get(&buffer->ref);
	return bo;
}

static struct nvdec_buffer *
nvdec_buffer_ref(struct nvdec_buffer *buffer)
{
	kref_get(&buffer->ref);
	return buffer;
}

static void nvdec_buffer_release(struct kref *ref)
{
	struct nvdec_buffer *buffer = container_of(ref, struct nvdec_buffer,
							  ref);
	struct tegra_drm *tegra = nvdec_engine_tegra_iommu(buffer->engine);

	if (tegra)
		nvdec_engine_iommu_unmap(tegra, buffer->mm, buffer->mapped);
	dma_free_coherent(buffer->dev, buffer->size, buffer->cpu, buffer->dma);
	kfree(buffer);
}

static void nvdec_buffer_put(struct host1x_bo *bo)
{
	struct nvdec_buffer *buffer = container_of(bo, struct nvdec_buffer,
							  bo);

	kref_put(&buffer->ref, nvdec_buffer_release);
}

static struct host1x_bo_mapping *
nvdec_buffer_pin(struct device *dev, struct host1x_bo *bo,
		 enum dma_data_direction direction)
{
	struct nvdec_buffer *buffer = container_of(bo, struct nvdec_buffer,
							  bo);
	struct host1x_bo_mapping *map;
	int err;

	map = kzalloc_obj(*map);
	if (!map)
		return ERR_PTR(-ENOMEM);

	kref_init(&map->ref);
	map->bo = host1x_bo_get(bo);
	map->direction = direction;
	map->dev = dev;
	map->sgt = kzalloc_obj(*map->sgt);
	if (!map->sgt) {
		err = -ENOMEM;
		goto free_map;
	}

	err = dma_get_sgtable(buffer->dev, map->sgt, buffer->cpu, buffer->dma,
			      buffer->size);
	if (err)
		goto free_sgt;

	err = dma_map_sgtable(dev, map->sgt, direction, 0);
	if (err)
		goto free_sgt;

	map->phys = sg_dma_address(map->sgt->sgl);
	map->size = buffer->size;
	map->chunks = 1;
	return map;

free_sgt:
	sg_free_table(map->sgt);
	kfree(map->sgt);
free_map:
	host1x_bo_put(map->bo);
	kfree(map);
	return ERR_PTR(err);
}

static void nvdec_buffer_unpin(struct host1x_bo_mapping *map)
{
	dma_unmap_sgtable(map->dev, map->sgt, map->direction, 0);
	sg_free_table(map->sgt);
	kfree(map->sgt);
	host1x_bo_put(map->bo);
	kfree(map);
}

static void *nvdec_buffer_mmap(struct host1x_bo *bo)
{
	return container_of(bo, struct nvdec_buffer, bo)->cpu;
}

static void nvdec_buffer_munmap(struct host1x_bo *bo, void *addr)
{
}

static const struct host1x_bo_ops nvdec_buffer_ops = {
	.get = nvdec_buffer_get,
	.put = nvdec_buffer_put,
	.pin = nvdec_buffer_pin,
	.unpin = nvdec_buffer_unpin,
	.mmap = nvdec_buffer_mmap,
	.munmap = nvdec_buffer_munmap,
};

static struct nvdec_buffer *
nvdec_buffer_alloc(struct nvdec_engine *engine, size_t size)
{
	struct device *dev = engine->dev;
	struct nvdec_buffer *buffer;
	struct tegra_drm *tegra;
	struct sg_table sgt;
	int err;

	buffer = kzalloc_obj(*buffer);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	buffer->cpu = dma_alloc_coherent(dev, size, &buffer->dma, GFP_KERNEL);
	if (!buffer->cpu) {
		kfree(buffer);
		return ERR_PTR(-ENOMEM);
	}
	buffer->iova = buffer->dma;

	tegra = nvdec_engine_tegra_iommu(engine);
	if (tegra) {
		err = dma_get_sgtable(dev, &sgt, buffer->cpu, buffer->dma, size);
		if (!err) {
			err = nvdec_engine_iommu_map(tegra, &sgt, size, &buffer->mm,
						     &buffer->mapped, &buffer->iova);
			sg_free_table(&sgt);
		}
		if (err) {
			dma_free_coherent(dev, size, buffer->cpu, buffer->dma);
			kfree(buffer);
			return ERR_PTR(err);
		}
	}

	host1x_bo_init(&buffer->bo, &nvdec_buffer_ops);
	kref_init(&buffer->ref);
	buffer->engine = engine;
	buffer->dev = dev;
	buffer->size = size;
	return buffer;
}

struct nvdec_engine_map *nvdec_engine_map_get(struct nvdec_engine_map *map)
{
	kref_get(&map->ref);
	return map;
}

static void nvdec_engine_pages_free(struct nvdec_engine_map *map);

static void nvdec_engine_map_release(struct kref *ref)
{
	struct nvdec_engine_map *map = container_of(ref, struct nvdec_engine_map, ref);
	struct tegra_drm *tegra = nvdec_engine_tegra_iommu(map->engine);

	if (tegra)
		nvdec_engine_iommu_unmap(tegra, map->mm, map->mapped);
	if (map->dmabuf) {
		dma_buf_unmap_attachment_unlocked(map->attach, map->sgt,
						  map->direction);
		dma_buf_detach(map->dmabuf, map->attach);
		dma_buf_put(map->dmabuf);
	} else if (map->pages) {
		nvdec_engine_pages_free(map);
	} else {
		dma_free_coherent(map->engine->dev, map->size, map->cpu,
				  map->dma);
	}
	kfree(map);
}

int nvdec_engine_map_wait(struct nvdec_engine_map *map, bool write)
{
	long err;

	if (!map)
		return -EINVAL;
	if (!map->dmabuf)
		return 0;

	err = dma_resv_wait_timeout(map->dmabuf->resv, dma_resv_usage_rw(write),
				    true, MAX_SCHEDULE_TIMEOUT);
	return err < 0 ? err : 0;
}

int nvdec_engine_map_add_fence(struct nvdec_engine_map *map,
			       struct dma_fence *fence, bool write)
{
	int err;

	if (!map || !fence)
		return -EINVAL;
	if (!map->dmabuf)
		return 0;

	err = dma_resv_lock(map->dmabuf->resv, NULL);
	if (err)
		return err;
	err = dma_resv_reserve_fences(map->dmabuf->resv, 1);
	if (!err)
		dma_resv_add_fence(map->dmabuf->resv, fence,
				   write ? DMA_RESV_USAGE_WRITE : DMA_RESV_USAGE_READ);
	dma_resv_unlock(map->dmabuf->resv);

	return err;
}

void nvdec_engine_map_put(struct nvdec_engine_map *map)
{
	if (map)
		kref_put(&map->ref, nvdec_engine_map_release);
}

struct nvdec_engine_map *
nvdec_engine_map_create(struct nvdec_engine *engine, struct dma_buf *dmabuf,
			unsigned long offset, size_t size,
			enum dma_data_direction direction)
{
	struct nvdec_engine_map *map;
	struct tegra_drm *tegra;
	struct scatterlist *sg;
	dma_addr_t expected;
	size_t contiguous = 0;
	unsigned int i;
	int err;

	if (!dmabuf || !size || offset > dmabuf->size ||
	    size > dmabuf->size - offset)
		return ERR_PTR(-EINVAL);

	map = kzalloc_obj(*map);
	if (!map)
		return ERR_PTR(-ENOMEM);

	get_dma_buf(dmabuf);
	map->dmabuf = dmabuf;
	map->attach = dma_buf_attach(dmabuf, engine->dev);
	if (IS_ERR(map->attach)) {
		err = PTR_ERR(map->attach);
		map->attach = NULL;
		goto put_dmabuf;
	}

	map->sgt = dma_buf_map_attachment_unlocked(map->attach, direction);
	if (IS_ERR(map->sgt)) {
		err = PTR_ERR(map->sgt);
		map->sgt = NULL;
		goto detach;
	}
	map->engine = engine;

	tegra = nvdec_engine_tegra_iommu(engine);
	if (tegra) {
		err = nvdec_engine_iommu_map(tegra, map->sgt, dmabuf->size,
					     &map->mm, &map->mapped, &map->iova);
		if (err)
			goto unmap;
		map->iova += offset;
	} else {
		expected = sg_dma_address(map->sgt->sgl);
		for_each_sgtable_dma_sg(map->sgt, sg, i) {
			if (sg_dma_address(sg) != expected ||
			    check_add_overflow(contiguous, (size_t)sg_dma_len(sg),
					       &contiguous)) {
				err = -EINVAL;
				goto unmap;
			}
			expected += sg_dma_len(sg);
		}
		if (offset > contiguous || size > contiguous - offset) {
			err = -EINVAL;
			goto unmap;
		}

		map->iova = sg_dma_address(map->sgt->sgl) + offset;
	}
	map->offset = offset;
	map->size = size;
	map->direction = direction;
	kref_init(&map->ref);
	return map;

unmap:
	dma_buf_unmap_attachment_unlocked(map->attach, map->sgt, direction);
	map->sgt = NULL;
detach:
	dma_buf_detach(dmabuf, map->attach);
put_dmabuf:
	dma_buf_put(dmabuf);
	kfree(map);
	return ERR_PTR(err);
}

static void nvdec_engine_pages_free(struct nvdec_engine_map *map)
{
	unsigned int i;

	if (!map->pages)
		return;

	for (i = 0; i < map->npages; i++)
		if (map->pages[i])
			__free_page(map->pages[i]);
	kvfree(map->pages);
	map->pages = NULL;
}

/* Device-only memory: no CPU mapping and no physical contiguity needed. */
static int nvdec_engine_pages_alloc(struct nvdec_engine *engine,
				    struct tegra_drm *tegra,
				    struct nvdec_engine_map *map, size_t size)
{
	unsigned int i, npages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	struct sg_table sgt;
	int err;

	map->pages = kvcalloc(npages, sizeof(*map->pages), GFP_KERNEL);
	if (!map->pages)
		return -ENOMEM;
	map->npages = npages;

	for (i = 0; i < npages; i++) {
		map->pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!map->pages[i]) {
			err = -ENOMEM;
			goto free_pages;
		}
	}

	err = sg_alloc_table_from_pages(&sgt, map->pages, npages, 0,
					(size_t)npages << PAGE_SHIFT, GFP_KERNEL);
	if (err)
		goto free_pages;

	/* Flush stale cache lines once, the way tegra_bo does. */
	err = dma_map_sgtable(engine->dev, &sgt, DMA_BIDIRECTIONAL, 0);
	if (!err) {
		dma_unmap_sgtable(engine->dev, &sgt, DMA_BIDIRECTIONAL, 0);
		err = nvdec_engine_iommu_map(tegra, &sgt, size, &map->mm,
					     &map->mapped, &map->iova);
	}
	sg_free_table(&sgt);
	if (err)
		goto free_pages;

	return 0;

free_pages:
	nvdec_engine_pages_free(map);
	return err;
}

struct nvdec_engine_map *
nvdec_engine_surface_create(struct nvdec_engine *engine, size_t size)
{
	struct nvdec_engine_map *map;
	struct tegra_drm *tegra;
	int err;

	if (!engine || !size)
		return ERR_PTR(-EINVAL);

	map = kzalloc_obj(*map);
	if (!map)
		return ERR_PTR(-ENOMEM);

	map->engine = engine;
	map->size = size;
	map->direction = DMA_BIDIRECTIONAL;

	tegra = nvdec_engine_tegra_iommu(engine);
	if (tegra) {
		err = nvdec_engine_pages_alloc(engine, tegra, map, size);
	} else {
		/* No explicit domain: the engine needs physical contiguity. */
		map->cpu = dma_alloc_coherent(engine->dev, size, &map->dma,
					      GFP_KERNEL);
		map->iova = map->dma;
		err = map->cpu ? 0 : -ENOMEM;
	}
	if (err) {
		kfree(map);
		return ERR_PTR(err);
	}

	kref_init(&map->ref);
	return map;
}

static bool nvdec_map_is_valid(const struct nvdec_engine_map *map,
			       enum dma_data_direction direction, size_t size)
{
	if (!map || map->size < size || upper_32_bits(map->iova))
		return false;

	return map->direction == direction || map->direction == DMA_BIDIRECTIONAL;
}

static int nvdec_copy_output(struct nvdec_buffer *input, u32 offset,
			     const struct nvdec_engine_map *output,
				 size_t payload_size)
{
	struct iosys_map vmap = { };
	int err;

	err = dma_buf_begin_cpu_access(output->dmabuf, DMA_TO_DEVICE);
	if (err)
		return err;

	err = dma_buf_vmap(output->dmabuf, &vmap);
	if (!err) {
		iosys_map_memcpy_from(input->cpu + offset, &vmap, output->offset,
				      payload_size);
		dma_buf_vunmap(output->dmabuf, &vmap);
	}

	dma_buf_end_cpu_access(output->dmabuf, DMA_TO_DEVICE);
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
	for (i = 0; !err && i < NVDEC_H264_DPB_ENTRIES; i++) {
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
	if (upper_32_bits(scratch->iova)) {
		nvdec_engine_map_put(scratch);
		return -ERANGE;
	}

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
	struct nvdec_buffer *input;

	if (ctx->input && ctx->input->size >= size)
		return 0;

	input = nvdec_buffer_alloc(ctx->engine, ALIGN(size, SZ_4K));
	if (IS_ERR(input))
		return PTR_ERR(input);
	if (upper_32_bits(input->iova)) {
		nvdec_buffer_put(&input->bo);
		return -ERANGE;
	}

	if (ctx->input) {
		memcpy(input->cpu, ctx->input->cpu, ctx->staged);
		nvdec_buffer_put(&ctx->input->bo);
	}
	ctx->input = input;
	return 0;
}

/* Only HEVC needs a lead zero byte, so its scan sees 00 00 00 01. */
static int nvdec_frame_stage(struct nvdec_decode_context *ctx,
			     struct nvdec_engine_map *output, u32 payload_size)
{
	bool hevc = ctx->codec == NVDEC_CODEC_HEVC;
	u32 lead = hevc ? 1 : 0;
	u32 staged = payload_size + lead;
	int err;

	if (staged < payload_size ||
	    !nvdec_map_is_valid(output, DMA_TO_DEVICE, payload_size))
		return -EINVAL;

	err = nvdec_prepare_input(ctx, staged);
	if (err)
		return err;

	if (lead)
		*(u8 *)ctx->input->cpu = 0;
	err = nvdec_copy_output(ctx->input, lead, output, payload_size);
	if (err)
		return err;

	if (hevc && memcmp(ctx->input->cpu + 1, "\x00\x00\x01", 3))
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

	if (ctx->codec != NVDEC_CODEC_H264) {
		mutex_lock(&ctx->lock);
		err = nvdec_frame_stage(ctx, output, payload_size);
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
	/* VP8 probabilities are stream state; the next stream starts at defaults. */
	if (ctx->probs) {
		nvdec_buffer_put(&ctx->probs->bo);
		ctx->probs = NULL;
	}
	if (ctx->counts) {
		nvdec_buffer_put(&ctx->counts->bo);
		ctx->counts = NULL;
	}
	ctx->width_in_mbs = 0;
	ctx->height_in_mbs = 0;
	ctx->coded_width = 0;
	ctx->coded_height = 0;
	mutex_unlock(&ctx->lock);
}

static int nvdec_h264_validate_request(struct device *dev,
				       const struct nvdec_h264_request *request,
				       const struct nvdec_engine_map *surface,
				       const struct nvdec_engine_map *capture,
				       struct nvdec_engine_map * const dpb[])
{
	u32 luma_size, chroma_size, capture_size, dst_size;
	unsigned int i;

	dev_dbg(dev,
		"h264 request: profile=%u level=%u chroma=%u depth=%u/%u log2fn=%u poc=%u/%u maxref=%u flags=0x%x pps=0x%x wbipred=%u slicegroups=%u type=%u mbs=%ux%u stride=%u/%u coff=%u payload=%u\n",
		request->profile_idc, request->level_idc,
		request->chroma_format_idc, request->bit_depth_luma_minus8,
		request->bit_depth_chroma_minus8,
		request->log2_max_frame_num_minus4, request->pic_order_cnt_type,
		request->log2_max_pic_order_cnt_lsb_minus4,
		request->max_num_ref_frames, request->flags, request->pps_flags,
		request->weighted_bipred_idc, request->num_slice_groups_minus1,
		request->slice_type, request->pic_width_in_mbs,
		request->frame_height_in_mbs, request->luma_stride,
		request->chroma_stride, request->chroma_offset,
		request->output_payload_size);
	dev_dbg(dev, "h264 maps: capture size=%zu dir=%d hi=%u\n",
		capture ? capture->size : 0, capture ? capture->direction : -1,
		capture ? upper_32_bits(capture->iova) : 0);

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
	    !request->pic_width_in_mbs || !request->frame_height_in_mbs ||
	    request->pic_width_in_mbs > 256 || request->frame_height_in_mbs > 256 ||
	    request->num_slice_groups_minus1 ||
	    request->weighted_bipred_idc > 2 ||
	    (request->slice_type != NVDEC_H264_SLICE_I &&
	     request->slice_type != NVDEC_H264_SLICE_P &&
	     request->slice_type != NVDEC_H264_SLICE_B) ||
	    request->output_payload_size < 3 ||
	    request->output_payload_size > U32_MAX - 16 || !request->slice_count ||
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

	if (check_mul_overflow((u32)request->luma_stride,
			       (u32)request->frame_height_in_mbs * 16, &luma_size) ||
	    check_mul_overflow((u32)request->chroma_stride,
			       (u32)ALIGN(request->frame_height_in_mbs * 8, 16),
			       &chroma_size) ||
	    check_add_overflow(request->chroma_offset, chroma_size, &capture_size) ||
	    request->luma_stride != request->chroma_stride ||
	    request->chroma_offset < luma_size ||
	    !nvdec_map_is_valid(surface, DMA_BIDIRECTIONAL, capture_size)) {
		dev_dbg(dev, "h264 reject: surface geometry/map\n");
		return -EINVAL;
	}

	if (!request->crop_width || !request->crop_height ||
	    (request->crop_width | request->crop_height |
	     request->crop_left | request->crop_top) & 1 ||
	    request->crop_left + request->crop_width >
	    request->pic_width_in_mbs * 16 ||
	    request->crop_top + request->crop_height >
	    request->frame_height_in_mbs * 16) {
		dev_dbg(dev, "h264 reject: crop rectangle\n");
		return -EINVAL;
	}

	if (check_mul_overflow(request->dst_stride,
			       (u32)request->crop_height / 2, &dst_size) ||
	    check_add_overflow(request->dst_chroma_offset, dst_size, &dst_size) ||
	    request->dst_chroma_offset < request->dst_stride *
					 (u32)request->crop_height ||
	    !IS_ALIGNED(request->dst_stride, SZ_256) ||
	    request->dst_stride < request->crop_width ||
	    !nvdec_map_is_valid(capture, DMA_FROM_DEVICE, dst_size)) {
		dev_dbg(dev, "h264 reject: detile destination\n");
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
		    !nvdec_map_is_valid(dpb[i], DMA_TO_DEVICE, capture_size)) {
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

	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++) {
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

static void nvdec_h264_fill_setup(struct nvdec_decode_job *hjob, u8 current_index,
				  const u8 picture_indices[NVDEC_H264_DPB_ENTRIES],
				  const u8 dpb_slots[NVDEC_H264_DPB_ENTRIES])
{
	const struct nvdec_h264_request *request = &hjob->request;
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct nvdec_h264_setup *setup = hjob->state->cpu;
	u32 picture_flags, current_picture;

	setup->stream_len = cpu_to_le32(request->output_payload_size + 16);
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
	setup->pitch_luma = cpu_to_le32(request->luma_stride);
	setup->pitch_chroma = cpu_to_le32(request->chroma_stride);
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
	nvdec_h264_setup_dpb(setup, request, picture_indices, dpb_slots);
	memcpy(setup->scaling_4x4, request->scaling_4x4, sizeof(setup->scaling_4x4));
	memcpy(setup->scaling_8x8, request->scaling_8x8, sizeof(setup->scaling_8x8));
}

static void nvdec_emit_method(u32 *gather, unsigned int *word,
			      u32 method, u32 value)
{
	gather[(*word)++] = NVDEC_METHOD_INCR;
	gather[(*word)++] = method;
	gather[(*word)++] = value;
}

static void nvdec_h264_debug_job(struct nvdec_decode_job *hjob, u8 current_index,
				 const u8 picture_indices[NVDEC_H264_DPB_ENTRIES],
				 const u8 dpb_slots[NVDEC_H264_DPB_ENTRIES])
{
	char dpb[128] = "";
	unsigned int i, len = 0;

	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (!hjob->request.dpb[i].valid)
			continue;
		len += scnprintf(dpb + len, sizeof(dpb) - len, "%s%u:%u@%u",
				 len ? "," : "", i, picture_indices[i],
				 dpb_slots[i]);
	}

	dev_dbg(hjob->ctx->engine->dev,
		"h264 methods=0200,0400,040c,0404,0408,0410,0424,0414,0500,0418,luma[0..16],chroma[0..16],0300 dpb=%s current=%u\n",
		dpb[0] ? dpb : "none", current_index);
	print_hex_dump_debug("nvdec setup: ", DUMP_PREFIX_OFFSET, 16, 4,
			     hjob->state->cpu, NVDEC_H264_SETUP_SIZE, false);
}

/* NVDEC address methods carry the address shifted right by 8. */
static int nvdec_emit_address(u32 *gather, unsigned int *word,
			      unsigned int method, dma_addr_t iova)
{
	if (!IS_ALIGNED(iova, SZ_256) || upper_32_bits(iova >> 8))
		return -EINVAL;

	nvdec_emit_method(gather, word, method, lower_32_bits(iova >> 8));
	return 0;
}

static int nvdec_h264_build_gather(struct nvdec_decode_job *hjob,
				   u8 current_index,
				   const u8 picture_indices[NVDEC_H264_DPB_ENTRIES],
				   const u8 dpb_slots[NVDEC_H264_DPB_ENTRIES])
{
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct nvdec_engine_map *references[NVDEC_H264_MAX_PICTURES];
	u32 *gather = hjob->gather->cpu;
	unsigned int i, word = 0;
	u32 syncpt_id;
	int err;

	if (upper_32_bits(hjob->state->iova) || upper_32_bits(hjob->input->iova) ||
	    upper_32_bits(hjob->gather->iova))
		return -ERANGE;

	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++)
		references[i] = hjob->surface;
	references[current_index] = hjob->surface;
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (hjob->request.dpb[i].valid)
			references[picture_indices[i]] = hjob->dpb[i];
	}

	nvdec_emit_method(gather, &word, NVDEC_METHOD_APPLICATION, 3);
	nvdec_emit_method(gather, &word, NVDEC_METHOD_CONTROL, 0x53);
	nvdec_emit_method(gather, &word, NVDEC_METHOD_PICTURE_INDEX,
			  current_index);
	err = nvdec_emit_address(gather, &word, NVDEC_METHOD_SETUP,
				 hjob->state->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_INPUT,
					 hjob->input->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word,
					 NVDEC_H264_METHOD_SLICE_OFFSETS,
					      hjob->input->iova +
					      hjob->slice_offsets_off);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_STATUS,
					 hjob->state->iova + NVDEC_H264_STATUS_OFFSET);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_COLOC,
					 hjob->scratch->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_H264_METHOD_MBHIST,
					 hjob->scratch->iova + ctx->mbhist_offset);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_H264_METHOD_HISTORY,
					 hjob->scratch->iova + ctx->history_offset);
	for (i = 0; !err && i < NVDEC_H264_MAX_PICTURES; i++)
		err = nvdec_emit_address(gather, &word,
					 NVDEC_METHOD_LUMA + i,
					      references[i]->iova);
	for (i = 0; !err && i < NVDEC_H264_MAX_PICTURES; i++)
		err = nvdec_emit_address(gather, &word,
					 NVDEC_METHOD_CHROMA + i,
					      references[i]->iova +
					      hjob->request.chroma_offset);
	if (err)
		return err;
	nvdec_emit_method(gather, &word, NVDEC_METHOD_EXECUTE, 0x100);
	if (WARN_ON_ONCE(word != NVDEC_H264_GATHER_WORDS))
		return -EINVAL;

	syncpt_id = host1x_syncpt_id(hjob->ctx->engine->client.base.syncpts[0]);
	gather[word++] = 0x20000001;
	gather[word++] = syncpt_id | 0x100;

	err = vic_engine_emit_detile(gather, &word,
				     hjob->state->iova + NVDEC_H264_VIC_CONFIG_OFFSET,
				     hjob->surface->iova,
				     hjob->surface->iova + hjob->request.chroma_offset,
				     hjob->capture->iova,
				     hjob->capture->iova +
				     hjob->request.dst_chroma_offset);
	if (err)
		return err;

	gather[word++] = 0x20000001;
	gather[word++] = syncpt_id | 0x100;
	WARN_ON_ONCE(word != NVDEC_H264_TOTAL_GATHER_WORDS);
	nvdec_h264_debug_job(hjob, current_index, picture_indices, dpb_slots);
	return 0;
}

static void nvdec_context_release(struct kref *ref)
{
	struct nvdec_decode_context *ctx = container_of(ref, struct nvdec_decode_context,
						       ref);
	unsigned int i;

	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++)
		nvdec_engine_map_put(ctx->surfaces[i].map);
	if (ctx->input)
		nvdec_buffer_put(&ctx->input->bo);
	if (ctx->probs)
		nvdec_buffer_put(&ctx->probs->bo);
	if (ctx->counts)
		nvdec_buffer_put(&ctx->counts->bo);
	nvdec_engine_map_put(ctx->scratch);
	kfree(ctx->slice_offsets);
	kfree(ctx);
}

static void nvdec_decode_job_release(struct host1x_job *job)
{
	struct nvdec_decode_job *hjob = job->user_data;
	unsigned int i;

	if (hjob->submitted) {
		mutex_lock(&hjob->ctx->lock);
		hjob->ctx->in_flight = false;
		mutex_unlock(&hjob->ctx->lock);
	}
	if (hjob->pinned)
		host1x_job_unpin(job);
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
		if (job->cancelled)
			dma_fence_set_error(hjob->fence, -EIO);
		dma_fence_signal(hjob->fence);
		dma_fence_put(hjob->fence);
	}
	if (hjob->completion_armed && hjob->complete)
		hjob->complete(hjob->complete_data, job->cancelled);
	nvdec_buffer_put(&hjob->gather->bo);
	nvdec_buffer_put(&hjob->input->bo);
	nvdec_buffer_put(&hjob->state->bo);
	if (hjob->probs)
		nvdec_buffer_put(&hjob->probs->bo);
	if (hjob->counts)
		nvdec_buffer_put(&hjob->counts->bo);
	nvdec_engine_map_put(hjob->scratch);
	nvdec_engine_map_put(hjob->surface);
	nvdec_engine_map_put(hjob->capture);
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++)
		nvdec_engine_map_put(hjob->dpb[i]);
	if (job->cancelled)
		nvdec_engine_recover(hjob->ctx->engine);
	kref_put(&hjob->ctx->ref, nvdec_context_release);
	kfree(hjob);
}

struct nvdec_decode_context *
nvdec_engine_context_create(struct nvdec_engine *engine, enum nvdec_codec codec)
{
	struct nvdec_decode_context *ctx;

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
	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++) {
		if (ctx->surfaces[i].map != surface)
			continue;
		nvdec_engine_map_put(ctx->surfaces[i].map);
		ctx->surfaces[i].map = NULL;
		break;
	}
	mutex_unlock(&ctx->lock);
}

static void nvdec_free_job(struct nvdec_decode_job *hjob)
{
	unsigned int i;

	if (hjob->fence) {
		dma_fence_set_error(hjob->fence, -EIO);
		dma_fence_signal(hjob->fence);
		dma_fence_put(hjob->fence);
	}
	if (hjob->gather)
		nvdec_buffer_put(&hjob->gather->bo);
	if (hjob->input)
		nvdec_buffer_put(&hjob->input->bo);
	if (hjob->probs)
		nvdec_buffer_put(&hjob->probs->bo);
	if (hjob->counts)
		nvdec_buffer_put(&hjob->counts->bo);
	if (hjob->state)
		nvdec_buffer_put(&hjob->state->bo);
	nvdec_engine_map_put(hjob->scratch);
	nvdec_engine_map_put(hjob->surface);
	nvdec_engine_map_put(hjob->capture);
	for (i = 0; i < NVDEC_MAX_DPB_ENTRIES; i++)
		nvdec_engine_map_put(hjob->dpb[i]);
	kref_put(&hjob->ctx->ref, nvdec_context_release);
	kfree(hjob);
}

/* One host1x job: NVDEC gather, OP_DONE, wait into VIC, VIC gather, OP_DONE. */
static int nvdec_launch_job(struct nvdec_decode_job *hjob,
			    unsigned int gather_words, unsigned int vic_offset,
			    struct dma_fence **fence)
{
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct host1x_job *job;
	int err;

	job = host1x_job_alloc(ctx->engine->channel, 5, 0, true);
	if (!job)
		return -ENOMEM;

	job->client = &ctx->engine->client.base;
	job->class = HOST1X_CLASS_NVDEC;
	job->serialize = true;
	job->syncpt = host1x_syncpt_get(ctx->engine->client.base.syncpts[0]);
	job->syncpt_incrs = 2;	/* NVDEC OP_DONE, then VIC OP_DONE */
	job->timeout = 10000;
	job->release = nvdec_decode_job_release;
	job->user_data = hjob;
	host1x_job_add_gather(job, &hjob->gather->bo, gather_words, 0);
	host1x_job_add_gather(job, &hjob->gather->bo, NVDEC_DONE_WORDS,
			      gather_words * sizeof(u32));
	host1x_job_add_wait(job, host1x_syncpt_id(job->syncpt), 1, true,
			    HOST1X_CLASS_VIC);
	host1x_job_add_gather(job, &hjob->gather->bo, VIC_DETILE_WORDS,
			      vic_offset * sizeof(u32));
	host1x_job_add_gather(job, &hjob->gather->bo, NVDEC_DONE_WORDS,
			      (vic_offset + VIC_DETILE_WORDS) * sizeof(u32));

	err = pm_runtime_resume_and_get(ctx->engine->dev);
	if (err < 0)
		goto put_job;
	hjob->runtime_ref = true;
	err = pm_runtime_resume_and_get(vic_engine_device(hjob->vic));
	if (err < 0)
		goto put_job;
	hjob->vic_runtime_ref = true;
	err = host1x_job_pin(job, ctx->engine->dev);
	if (err)
		goto put_job;
	hjob->pinned = true;
	ctx->in_flight = true;
	hjob->submitted = true;
	err = nvdec_engine_submit_job(ctx->engine, job, NULL, NULL);
	if (err)
		goto put_job;
	hjob->completion_armed = true;
	/* Host1x owns the pin after successful submission. */
	hjob->pinned = false;
	*fence = dma_fence_get(hjob->fence);
	host1x_job_put(job);
	return 0;

put_job:
	if (hjob->submitted) {
		ctx->in_flight = false;
		hjob->submitted = false;
	}
	host1x_job_put(job);
	/* The job took no reference; the caller still frees hjob. */
	return err;
}

int nvdec_engine_h264_submit(struct nvdec_decode_context *ctx,
			     const struct nvdec_h264_request *request,
			     struct nvdec_engine_map *surface,
			     struct nvdec_engine_map *capture,
			     struct nvdec_engine_map * const dpb[NVDEC_H264_DPB_ENTRIES],
			     struct dma_fence **fence,
			     nvdec_engine_complete_t complete, void *data)
{
	static const u8 termination[16] = {
		0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
	};
	struct nvdec_decode_job *hjob;
	u8 picture_indices[NVDEC_H264_DPB_ENTRIES] = { };
	u8 dpb_slots[NVDEC_H264_DPB_ENTRIES] = { };
	u8 current_index;
	unsigned int i;
	__le32 *offsets;
	int err;

	if (!ctx || !request || !surface || !capture || !dpb || !fence)
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
	hjob->request = *request;
	hjob->request.output_payload_size = ctx->staged;
	hjob->request.slice_count = ctx->slice_count;
	hjob->complete = complete;
	hjob->complete_data = data;
	err = nvdec_h264_validate_request(ctx->engine->dev, &hjob->request,
					  surface, capture, dpb);
	if (err)
		goto free_hjob;

	err = nvdec_h264_prepare_scratch(ctx, &hjob->request);
	if (err)
		goto free_hjob;
	hjob->scratch = nvdec_engine_map_get(ctx->scratch);
	hjob->slice_offsets_off = ALIGN(ctx->staged + 16, SZ_256);

	err = nvdec_surface_index(ctx, surface, &current_index,
				  NVDEC_H264_MAX_PICTURES);
	if (err)
		goto free_hjob;
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (!hjob->request.dpb[i].valid)
			continue;
		err = nvdec_surface_index(ctx, dpb[i], &picture_indices[i],
					  NVDEC_H264_MAX_PICTURES);
		if (!err)
			err = nvdec_h264_dpb_slot(ctx, dpb[i], &dpb_slots[i]);
		if (err)
			goto free_hjob;
	}

	hjob->vic = vic_engine_find(ctx->engine->client.drm);
	if (!hjob->vic) {
		err = -ENODEV;
		goto free_hjob;
	}
	hjob->state = nvdec_buffer_alloc(ctx->engine, NVDEC_H264_STATE_SIZE);
	if (IS_ERR(hjob->state)) {
		err = PTR_ERR(hjob->state);
		hjob->state = NULL;
		goto free_hjob;
	}
	hjob->input = nvdec_buffer_ref(ctx->input);
	hjob->gather = nvdec_buffer_alloc(ctx->engine,
					  NVDEC_H264_TOTAL_GATHER_WORDS * sizeof(u32));
	if (IS_ERR(hjob->gather)) {
		err = PTR_ERR(hjob->gather);
		hjob->gather = NULL;
		goto free_hjob;
	}
	if (upper_32_bits(hjob->state->iova) || upper_32_bits(hjob->input->iova) ||
	    upper_32_bits(hjob->gather->iova)) {
		err = -ERANGE;
		goto free_hjob;
	}

	hjob->surface = nvdec_engine_map_get(surface);
	hjob->capture = nvdec_engine_map_get(capture);
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (hjob->request.dpb[i].valid)
			hjob->dpb[i] = nvdec_engine_map_get(dpb[i]);
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

	memcpy(hjob->input->cpu + ctx->staged, termination, sizeof(termination));
	offsets = hjob->input->cpu + hjob->slice_offsets_off;
	for (i = 0; i < ctx->slice_count; i++)
		offsets[i] = cpu_to_le32(ctx->slice_offsets[i]);
	offsets[i] = cpu_to_le32(ctx->staged);

	nvdec_h264_fill_setup(hjob, current_index, picture_indices, dpb_slots);
	vic_engine_fill_detile_config(hjob->state->cpu + NVDEC_H264_VIC_CONFIG_OFFSET,
				      &(struct vic_detile_params){
					.width = hjob->request.pic_width_in_mbs * 16,
					.height = hjob->request.frame_height_in_mbs * 16,
					.left = hjob->request.crop_left,
					.top = hjob->request.crop_top,
					.out_width = hjob->request.crop_width,
					.out_height = hjob->request.crop_height,
					.src_stride = hjob->request.luma_stride,
					.dst_stride = hjob->request.dst_stride,
				      });
	err = nvdec_h264_build_gather(hjob, current_index, picture_indices,
				      dpb_slots);
	if (err)
		goto free_hjob;

	err = nvdec_launch_job(hjob, NVDEC_H264_GATHER_WORDS,
			       NVDEC_H264_VIC_OFFSET, fence);
	if (err)
		goto free_hjob;
	mutex_unlock(&ctx->lock);
	return 0;

free_hjob:
	nvdec_free_job(hjob);
unlock:
	mutex_unlock(&ctx->lock);
	return err;
}

static int nvdec_hevc_prepare_scratch(struct nvdec_decode_context *ctx,
				      const struct nvdec_hevc_request *request)
{
	u32 aligned_width, aligned_height, colmv, coloc, filter, size;
	struct nvdec_engine_map *scratch;

	if (ctx->scratch) {
		if (ctx->coded_width != request->coded_width ||
		    ctx->coded_height != request->coded_height)
			return -EBUSY;
		return 0;
	}

	aligned_width = ALIGN(request->coded_width, NVDEC_HEVC_CTU_SIZE);
	aligned_height = ALIGN(request->coded_height, NVDEC_HEVC_CTU_SIZE);

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
	if (upper_32_bits(scratch->iova)) {
		nvdec_engine_map_put(scratch);
		return -ERANGE;
	}

	ctx->scratch = scratch;
	ctx->coded_width = request->coded_width;
	ctx->coded_height = request->coded_height;
	ctx->colmv_size = colmv;
	ctx->filter_offset = coloc;
	ctx->sao_offset = NVDEC_HEVC_FILTER_PER_ROW * aligned_height;
	ctx->bsd_offset = (NVDEC_HEVC_FILTER_PER_ROW + NVDEC_HEVC_SAO_PER_ROW) *
			  aligned_height;
	return 0;
}

static int nvdec_hevc_validate_request(struct device *dev,
				       const struct nvdec_hevc_request *request,
				       const struct nvdec_engine_map *surface,
				       const struct nvdec_engine_map *capture,
				       struct nvdec_engine_map * const dpb[])
{
	u32 luma_size, chroma_size, surface_size, dst_size;
	unsigned int i, tiles;

	dev_dbg(dev,
		"hevc request: %ux%u coded=%ux%u ctb=%ux%u depth=%u cb=%u/%u tb=%u/%u qp=%u sps=0x%x pps=0x%x tiles=%ux%u refs=%u poc=%d skip=%u stride=%u coff=%u payload=%u\n",
		request->pic_width_in_luma_samples,
		request->pic_height_in_luma_samples, request->coded_width,
		request->coded_height, request->ctb_width, request->ctb_height,
		request->bit_depth, request->log2_min_luma_coding_block_size,
		request->log2_max_luma_coding_block_size,
		request->log2_min_transform_block_size,
		request->log2_max_transform_block_size, request->init_qp,
		request->sps_flags, request->pps_flags, request->num_tile_columns,
		request->num_tile_rows, request->num_ref_frames,
		request->pic_order_cnt_val, request->sw_hdr_skip_length,
		request->luma_stride, request->chroma_offset,
		request->output_payload_size);

	if (request->bit_depth != 8 ||
	    !request->pic_width_in_luma_samples ||
	    !request->pic_height_in_luma_samples ||
	    request->pic_width_in_luma_samples > request->coded_width ||
	    request->pic_height_in_luma_samples > request->coded_height ||
	    request->coded_width > 4096 || request->coded_height > 4096 ||
	    !IS_ALIGNED(request->coded_width, NVDEC_HEVC_CTU_SIZE) ||
	    !IS_ALIGNED(request->coded_height, NVDEC_HEVC_CTU_SIZE) ||
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
	    request->output_payload_size < 4 ||
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

	if (check_mul_overflow((u32)request->luma_stride,
			       (u32)ALIGN(request->coded_height, 32), &luma_size) ||
	    check_mul_overflow((u32)request->luma_stride,
			       (u32)ALIGN(request->coded_height / 2, 16),
			       &chroma_size) ||
	    check_add_overflow(request->chroma_offset, chroma_size, &surface_size) ||
	    request->chroma_offset < luma_size ||
	    !nvdec_map_is_valid(surface, DMA_BIDIRECTIONAL, surface_size)) {
		dev_dbg(dev, "hevc reject: surface geometry/map\n");
		return -EINVAL;
	}

	if (check_mul_overflow(request->dst_stride,
			       (u32)request->crop_height / 2, &dst_size) ||
	    check_add_overflow(request->dst_chroma_offset, dst_size, &dst_size) ||
	    !request->crop_width || !request->crop_height ||
	    (request->crop_width | request->crop_height |
	     request->crop_left | request->crop_top) & 1 ||
	    request->crop_left + request->crop_width > request->coded_width ||
	    request->crop_top + request->crop_height > request->coded_height ||
	    request->dst_chroma_offset < request->dst_stride *
					 (u32)request->crop_height ||
	    !IS_ALIGNED(request->dst_stride, SZ_256) ||
	    request->dst_stride < request->crop_width ||
	    !nvdec_map_is_valid(capture, DMA_FROM_DEVICE, dst_size)) {
		dev_dbg(dev, "hevc reject: detile destination\n");
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

static void nvdec_hevc_fill_setup(struct nvdec_decode_job *hjob, u8 current_index,
				  const u8 picture_indices[NVDEC_HEVC_DPB_ENTRIES],
				  s8 scratch_diff_poc)
{
	const struct nvdec_hevc_request *r = &hjob->hevc;
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct nvdec_hevc_setup *setup = hjob->state->cpu;
	u8 order0[3 * NVDEC_HEVC_DPB_ENTRIES];
	u8 order1[3 * NVDEC_HEVC_DPB_ENTRIES];
	unsigned int i, n = 0, m = 0;
	u32 mask = BIT(current_index);
	u32 word;

	setup->stream_len = cpu_to_le32(r->output_payload_size);
	setup->surface_format = cpu_to_le32(FIELD_PREP(NVDEC_HEVC_SURFACE_START_CODE, 1));
	setup->framestride[0] = cpu_to_le32(r->luma_stride);
	setup->framestride[1] = cpu_to_le32(r->luma_stride);
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

static int nvdec_hevc_build_gather(struct nvdec_decode_job *hjob,
				   u8 current_index,
				   const u8 picture_indices[NVDEC_HEVC_DPB_ENTRIES])
{
	struct nvdec_engine_map *references[NVDEC_HEVC_MAX_PICTURES];
	struct nvdec_decode_context *ctx = hjob->ctx;
	u32 *gather = hjob->gather->cpu;
	unsigned int i, word = 0;
	u32 syncpt_id;
	int err;

	for (i = 0; i < NVDEC_HEVC_MAX_PICTURES; i++)
		references[i] = hjob->scratch_ref;
	references[current_index] = hjob->surface;
	for (i = 0; i < NVDEC_HEVC_DPB_ENTRIES; i++) {
		if (hjob->hevc.dpb[i].valid)
			references[picture_indices[i]] = hjob->dpb[i];
	}

	nvdec_emit_method(gather, &word, NVDEC_METHOD_APPLICATION, 7);
	nvdec_emit_method(gather, &word, NVDEC_METHOD_CONTROL, 0x57);
	nvdec_emit_method(gather, &word, NVDEC_METHOD_PICTURE_INDEX,
			  current_index);
	err = nvdec_emit_address(gather, &word, NVDEC_METHOD_SETUP,
				 hjob->state->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_INPUT,
					 hjob->input->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_STATUS,
					 hjob->state->iova +
					 NVDEC_HEVC_STATUS_OFFSET);
	if (!err)
		err = nvdec_emit_address(gather, &word,
					 NVDEC_HEVC_METHOD_SCALING_LIST,
					 hjob->state->iova +
					 NVDEC_HEVC_SCALING_OFFSET);
	if (!err)
		err = nvdec_emit_address(gather, &word,
					 NVDEC_HEVC_METHOD_TILE_SIZES,
					 hjob->state->iova +
					 NVDEC_HEVC_TILES_OFFSET);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_HEVC_METHOD_FILTER,
					 hjob->scratch->iova + ctx->filter_offset);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_COLOC,
					 hjob->scratch->iova);
	for (i = 0; !err && i < NVDEC_HEVC_MAX_PICTURES; i++) {
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_LUMA + i,
					 references[i]->iova);
		if (!err)
			err = nvdec_emit_address(gather, &word,
						 NVDEC_METHOD_CHROMA + i,
						 references[i]->iova +
						 hjob->hevc.chroma_offset);
	}
	if (err)
		return err;
	nvdec_emit_method(gather, &word, NVDEC_METHOD_EXECUTE, 0x100);
	if (WARN_ON_ONCE(word != NVDEC_HEVC_GATHER_WORDS))
		return -EINVAL;

	syncpt_id = host1x_syncpt_id(ctx->engine->client.base.syncpts[0]);
	gather[word++] = 0x20000001;
	gather[word++] = syncpt_id | 0x100;

	err = vic_engine_emit_detile(gather, &word,
				     hjob->state->iova +
				     NVDEC_HEVC_VIC_CONFIG_OFFSET,
				     hjob->surface->iova,
				     hjob->surface->iova + hjob->hevc.chroma_offset,
				     hjob->capture->iova,
				     hjob->capture->iova +
				     hjob->hevc.dst_chroma_offset);
	if (err)
		return err;

	gather[word++] = 0x20000001;
	gather[word++] = syncpt_id | 0x100;
	WARN_ON_ONCE(word != NVDEC_HEVC_TOTAL_GATHER_WORDS);

	print_hex_dump_debug("nvdec hevc setup: ", DUMP_PREFIX_OFFSET, 16, 4,
			     hjob->state->cpu, NVDEC_HEVC_SETUP_SIZE, false);
	return 0;
}

int nvdec_engine_hevc_submit(struct nvdec_decode_context *ctx,
			     const struct nvdec_hevc_request *request,
			     struct nvdec_engine_map *surface,
			     struct nvdec_engine_map *capture,
			     struct nvdec_engine_map * const dpb[NVDEC_HEVC_DPB_ENTRIES],
			     struct dma_fence **fence,
			     nvdec_engine_complete_t complete, void *data)
{
	u8 picture_indices[NVDEC_HEVC_DPB_ENTRIES] = { };
	struct nvdec_decode_job *hjob;
	s8 scratch_diff_poc = 0;
	int scratch_entry = -1;
	u8 current_index;
	unsigned int i;
	int err;

	if (!ctx || !request || !surface || !capture || !dpb || !fence ||
	    ctx->codec != NVDEC_CODEC_HEVC)
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
	hjob->hevc = *request;
	hjob->hevc.output_payload_size = ctx->staged;
	hjob->complete = complete;
	hjob->complete_data = data;
	err = nvdec_hevc_validate_request(ctx->engine->dev, &hjob->hevc, surface,
					  capture, dpb);
	if (err)
		goto free_hjob;

	err = nvdec_hevc_prepare_scratch(ctx, &hjob->hevc);
	if (err)
		goto free_hjob;
	hjob->scratch = nvdec_engine_map_get(ctx->scratch);

	err = nvdec_surface_index(ctx, surface, &current_index,
				  NVDEC_HEVC_MAX_PICTURES);
	if (err)
		goto free_hjob;
	for (i = 0; i < NVDEC_HEVC_DPB_ENTRIES; i++) {
		if (!hjob->hevc.dpb[i].valid)
			continue;
		err = nvdec_surface_index(ctx, dpb[i], &picture_indices[i],
					  NVDEC_HEVC_MAX_PICTURES);
		if (err)
			goto free_hjob;
	}

	/* An unused slot still needs a surface and a POC difference. */
	if (hjob->hevc.num_poc_st_curr_before)
		scratch_entry = hjob->hevc.poc_st_curr_before[0];
	else if (hjob->hevc.num_poc_st_curr_after)
		scratch_entry = hjob->hevc.poc_st_curr_after[0];
	else if (hjob->hevc.num_poc_lt_curr)
		scratch_entry = hjob->hevc.poc_lt_curr[0];

	hjob->vic = vic_engine_find(ctx->engine->client.drm);
	if (!hjob->vic) {
		err = -ENODEV;
		goto free_hjob;
	}
	hjob->state = nvdec_buffer_alloc(ctx->engine, NVDEC_HEVC_STATE_SIZE);
	if (IS_ERR(hjob->state)) {
		err = PTR_ERR(hjob->state);
		hjob->state = NULL;
		goto free_hjob;
	}
	hjob->input = nvdec_buffer_ref(ctx->input);
	hjob->gather = nvdec_buffer_alloc(ctx->engine,
					  NVDEC_HEVC_TOTAL_GATHER_WORDS * sizeof(u32));
	if (IS_ERR(hjob->gather)) {
		err = PTR_ERR(hjob->gather);
		hjob->gather = NULL;
		goto free_hjob;
	}
	if (upper_32_bits(hjob->state->iova) || upper_32_bits(hjob->input->iova) ||
	    upper_32_bits(hjob->gather->iova)) {
		err = -ERANGE;
		goto free_hjob;
	}

	hjob->surface = nvdec_engine_map_get(surface);
	hjob->capture = nvdec_engine_map_get(capture);
	for (i = 0; i < NVDEC_HEVC_DPB_ENTRIES; i++) {
		if (hjob->hevc.dpb[i].valid)
			hjob->dpb[i] = nvdec_engine_map_get(dpb[i]);
	}
	if (scratch_entry >= 0) {
		hjob->scratch_ref = hjob->dpb[scratch_entry];
		scratch_diff_poc = clamp_t(int, hjob->hevc.pic_order_cnt_val -
					   hjob->hevc.dpb[scratch_entry].pic_order_cnt_val,
					   S8_MIN, S8_MAX);
	} else {
		hjob->scratch_ref = hjob->surface;
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

	nvdec_hevc_fill_setup(hjob, current_index, picture_indices,
			      scratch_diff_poc);
	vic_engine_fill_detile_config(hjob->state->cpu + NVDEC_HEVC_VIC_CONFIG_OFFSET,
				      &(struct vic_detile_params){
					.width = hjob->hevc.coded_width,
					.height = hjob->hevc.coded_height,
					.left = hjob->hevc.crop_left,
					.top = hjob->hevc.crop_top,
					.out_width = hjob->hevc.crop_width,
					.out_height = hjob->hevc.crop_height,
					.src_stride = hjob->hevc.luma_stride,
					.dst_stride = hjob->hevc.dst_stride,
				      });
	err = nvdec_hevc_build_gather(hjob, current_index, picture_indices);
	if (err)
		goto free_hjob;

	err = nvdec_launch_job(hjob, NVDEC_HEVC_GATHER_WORDS,
			       NVDEC_HEVC_VIC_OFFSET, fence);
	if (err)
		goto free_hjob;
	mutex_unlock(&ctx->lock);
	return 0;

free_hjob:
	nvdec_free_job(hjob);
unlock:
	mutex_unlock(&ctx->lock);
	return err;
}

/* RFC 6386's default entropy context, in the firmware's layout. */
static const u8 nvdec_vp8_default_coeff_probs[4][8][3][11] = {
	{
		{
			{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
		},
		{
			{ 253, 136, 254, 255, 228, 219, 128, 128, 128, 128, 128 },
			{ 189, 129, 242, 255, 227, 213, 255, 219, 128, 128, 128 },
			{ 106, 126, 227, 252, 214, 209, 255, 255, 128, 128, 128 },
		},
		{
			{   1,  98, 248, 255, 236, 226, 255, 255, 128, 128, 128 },
			{ 181, 133, 238, 254, 221, 234, 255, 154, 128, 128, 128 },
			{  78, 134, 202, 247, 198, 180, 255, 219, 128, 128, 128 },
		},
		{
			{   1, 185, 249, 255, 243, 255, 128, 128, 128, 128, 128 },
			{ 184, 150, 247, 255, 236, 224, 128, 128, 128, 128, 128 },
			{  77, 110, 216, 255, 236, 230, 128, 128, 128, 128, 128 },
		},
		{
			{   1, 101, 251, 255, 241, 255, 128, 128, 128, 128, 128 },
			{ 170, 139, 241, 252, 236, 209, 255, 255, 128, 128, 128 },
			{  37, 116, 196, 243, 228, 255, 255, 255, 128, 128, 128 },
		},
		{
			{   1, 204, 254, 255, 245, 255, 128, 128, 128, 128, 128 },
			{ 207, 160, 250, 255, 238, 128, 128, 128, 128, 128, 128 },
			{ 102, 103, 231, 255, 211, 171, 128, 128, 128, 128, 128 },
		},
		{
			{   1, 152, 252, 255, 240, 255, 128, 128, 128, 128, 128 },
			{ 177, 135, 243, 255, 234, 225, 128, 128, 128, 128, 128 },
			{  80, 129, 211, 255, 194, 224, 128, 128, 128, 128, 128 },
		},
		{
			{   1,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 246,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 255, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
		},
	},
	{
		{
			{ 198,  35, 237, 223, 193, 187, 162, 160, 145, 155,  62 },
			{ 131,  45, 198, 221, 172, 176, 220, 157, 252, 221,   1 },
			{  68,  47, 146, 208, 149, 167, 221, 162, 255, 223, 128 },
		},
		{
			{   1, 149, 241, 255, 221, 224, 255, 255, 128, 128, 128 },
			{ 184, 141, 234, 253, 222, 220, 255, 199, 128, 128, 128 },
			{  81,  99, 181, 242, 176, 190, 249, 202, 255, 255, 128 },
		},
		{
			{   1, 129, 232, 253, 214, 197, 242, 196, 255, 255, 128 },
			{  99, 121, 210, 250, 201, 198, 255, 202, 128, 128, 128 },
			{  23,  91, 163, 242, 170, 187, 247, 210, 255, 255, 128 },
		},
		{
			{   1, 200, 246, 255, 234, 255, 128, 128, 128, 128, 128 },
			{ 109, 178, 241, 255, 231, 245, 255, 255, 128, 128, 128 },
			{  44, 130, 201, 253, 205, 192, 255, 255, 128, 128, 128 },
		},
		{
			{   1, 132, 239, 251, 219, 209, 255, 165, 128, 128, 128 },
			{  94, 136, 225, 251, 218, 190, 255, 255, 128, 128, 128 },
			{  22, 100, 174, 245, 186, 161, 255, 199, 128, 128, 128 },
		},
		{
			{   1, 182, 249, 255, 232, 235, 128, 128, 128, 128, 128 },
			{ 124, 143, 241, 255, 227, 234, 128, 128, 128, 128, 128 },
			{  35,  77, 181, 251, 193, 211, 255, 205, 128, 128, 128 },
		},
		{
			{   1, 157, 247, 255, 236, 231, 255, 255, 128, 128, 128 },
			{ 121, 141, 235, 255, 225, 227, 255, 255, 128, 128, 128 },
			{  45,  99, 188, 251, 195, 217, 255, 224, 128, 128, 128 },
		},
		{
			{   1,   1, 251, 255, 213, 255, 128, 128, 128, 128, 128 },
			{ 203,   1, 248, 255, 255, 128, 128, 128, 128, 128, 128 },
			{ 137,   1, 177, 255, 224, 255, 128, 128, 128, 128, 128 },
		},
	},
	{
		{
			{ 253,   9, 248, 251, 207, 208, 255, 192, 128, 128, 128 },
			{ 175,  13, 224, 243, 193, 185, 249, 198, 255, 255, 128 },
			{  73,  17, 171, 221, 161, 179, 236, 167, 255, 234, 128 },
		},
		{
			{   1,  95, 247, 253, 212, 183, 255, 255, 128, 128, 128 },
			{ 239,  90, 244, 250, 211, 209, 255, 255, 128, 128, 128 },
			{ 155,  77, 195, 248, 188, 195, 255, 255, 128, 128, 128 },
		},
		{
			{   1,  24, 239, 251, 218, 219, 255, 205, 128, 128, 128 },
			{ 201,  51, 219, 255, 196, 186, 128, 128, 128, 128, 128 },
			{  69,  46, 190, 239, 201, 218, 255, 228, 128, 128, 128 },
		},
		{
			{   1, 191, 251, 255, 255, 128, 128, 128, 128, 128, 128 },
			{ 223, 165, 249, 255, 213, 255, 128, 128, 128, 128, 128 },
			{ 141, 124, 248, 255, 255, 128, 128, 128, 128, 128, 128 },
		},
		{
			{   1,  16, 248, 255, 255, 128, 128, 128, 128, 128, 128 },
			{ 190,  36, 230, 255, 236, 255, 128, 128, 128, 128, 128 },
			{ 149,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
		},
		{
			{   1, 226, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 247, 192, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 240, 128, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
		},
		{
			{   1, 134, 252, 255, 255, 128, 128, 128, 128, 128, 128 },
			{ 213,  62, 250, 255, 255, 128, 128, 128, 128, 128, 128 },
			{  55,  93, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
		},
		{
			{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 },
		},
	},
	{
		{
			{ 202,  24, 213, 235, 186, 191, 220, 160, 240, 175, 255 },
			{ 126,  38, 182, 232, 169, 184, 228, 174, 255, 187, 128 },
			{  61,  46, 138, 219, 151, 178, 240, 170, 255, 216, 128 },
		},
		{
			{   1, 112, 230, 250, 199, 191, 247, 159, 255, 255, 128 },
			{ 166, 109, 228, 252, 211, 215, 255, 174, 128, 128, 128 },
			{  39,  77, 162, 232, 172, 180, 245, 178, 255, 255, 128 },
		},
		{
			{   1,  52, 220, 246, 198, 199, 249, 220, 255, 255, 128 },
			{ 124,  74, 191, 243, 183, 193, 250, 221, 255, 255, 128 },
			{  24,  71, 130, 219, 154, 170, 243, 182, 255, 255, 128 },
		},
		{
			{   1, 182, 225, 249, 219, 240, 255, 224, 128, 128, 128 },
			{ 149, 150, 226, 252, 216, 205, 255, 171, 128, 128, 128 },
			{  28, 108, 170, 242, 183, 194, 254, 223, 255, 255, 128 },
		},
		{
			{   1,  81, 230, 252, 204, 203, 255, 192, 128, 128, 128 },
			{ 123, 102, 209, 247, 188, 196, 255, 233, 128, 128, 128 },
			{  20,  95, 153, 243, 164, 173, 255, 203, 128, 128, 128 },
		},
		{
			{   1, 222, 248, 255, 216, 213, 128, 128, 128, 128, 128 },
			{ 168, 175, 246, 252, 235, 205, 255, 255, 128, 128, 128 },
			{  47, 116, 215, 255, 211, 212, 255, 255, 128, 128, 128 },
		},
		{
			{   1, 121, 236, 253, 212, 214, 255, 255, 128, 128, 128 },
			{ 141,  84, 213, 252, 201, 202, 255, 219, 128, 128, 128 },
			{  42,  80, 160, 240, 162, 185, 255, 205, 128, 128, 128 },
		},
		{
			{   1,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 244,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
			{ 238,   1, 255, 128, 128, 128, 128, 128, 128, 128, 128 },
		},
	},
};

static const u8 nvdec_vp8_default_y_mode_probs[4] = { 112, 86, 140, 37 };
static const u8 nvdec_vp8_default_uv_mode_probs[3] = { 162, 101, 204 };
static const u8 nvdec_vp8_default_mv_probs[2][19] = {
	{ 162, 128, 225, 146, 172, 147, 214,  39, 156, 128,
	  129, 132,  75, 145, 178, 206, 239, 254, 254 },
	{ 164, 128, 204, 170, 119, 235, 140, 230, 228, 128,
	  130, 130,  74, 148, 180, 203, 236, 254, 254 },
};

/* A key frame resets the entropy context, so this runs on every one. */
static void nvdec_vp8_init_probs(struct nvdec_decode_context *ctx)
{
	struct nvdec_vp8_probs *probs = ctx->probs->cpu;
	unsigned int i, j, k;

	memset(probs, 0, sizeof(*probs));
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 8; j++) {
			for (k = 0; k < 3; k++)
				memcpy(probs->coeff[i][j][k],
				       nvdec_vp8_default_coeff_probs[i][j][k],
				       sizeof(nvdec_vp8_default_coeff_probs[i][j][k]));
		}
	}
	memcpy(probs->y_mode, nvdec_vp8_default_y_mode_probs,
	       sizeof(nvdec_vp8_default_y_mode_probs));
	memcpy(probs->uv_mode, nvdec_vp8_default_uv_mode_probs,
	       sizeof(nvdec_vp8_default_uv_mode_probs));
	for (i = 0; i < 2; i++)
		memcpy(probs->mv[i], nvdec_vp8_default_mv_probs[i],
		       sizeof(nvdec_vp8_default_mv_probs[i]));
}

static int nvdec_vp8_prepare_scratch(struct nvdec_decode_context *ctx,
				     const struct nvdec_vp8_request *request)
{
	struct nvdec_engine_map *scratch;
	struct nvdec_buffer *probs;
	u32 mbs, history;

	if (ctx->scratch) {
		if (ctx->coded_width != request->coded_width ||
		    ctx->coded_height != request->coded_height)
			return -EBUSY;
		return 0;
	}

	mbs = request->coded_width / 16;
	history = mbs * NVDEC_VP8_HISTORY_PER_MB;

	probs = nvdec_buffer_alloc(ctx->engine, NVDEC_VP8_PROB_SIZE);
	if (IS_ERR(probs))
		return PTR_ERR(probs);
	scratch = nvdec_engine_surface_create(ctx->engine, ALIGN(history, SZ_4K));
	if (IS_ERR(scratch)) {
		nvdec_buffer_put(&probs->bo);
		return PTR_ERR(scratch);
	}
	if (upper_32_bits(probs->iova) || upper_32_bits(scratch->iova)) {
		nvdec_engine_map_put(scratch);
		nvdec_buffer_put(&probs->bo);
		return -ERANGE;
	}

	ctx->probs = probs;
	ctx->scratch = scratch;
	ctx->coded_width = request->coded_width;
	ctx->coded_height = request->coded_height;
	ctx->history_offset = 0;
	ctx->history_size = history;
	memset(probs->cpu, 0, NVDEC_VP8_PROB_SIZE);
	nvdec_vp8_init_probs(ctx);
	return 0;
}

static int nvdec_vp8_validate_request(struct device *dev,
				      const struct nvdec_vp8_request *request,
				      const struct nvdec_engine_map *surface,
				      const struct nvdec_engine_map *capture,
				      struct nvdec_engine_map * const refs[])
{
	u32 luma_size, chroma_size, surface_size, dst_size;
	unsigned int i;

	dev_dbg(dev,
		"vp8 request: coded=%ux%u crop=%ux%u version=%u flags=0x%x firstpart=%u stride=%u coff=%u payload=%u\n",
		request->coded_width, request->coded_height,
		request->crop_width, request->crop_height, request->version,
		request->flags, request->first_part_size, request->luma_stride,
		request->chroma_offset, request->output_payload_size);

	if (request->version > 3 || !request->coded_width ||
	    !request->coded_height || request->coded_width > 4096 ||
	    request->coded_height > 4096 ||
	    !IS_ALIGNED(request->coded_width, 16) ||
	    !IS_ALIGNED(request->coded_height, 16) ||
	    !request->output_payload_size ||
	    !request->first_part_size ||
	    request->first_part_size > request->output_payload_size ||
	    !IS_ALIGNED(request->luma_stride, 16)) {
		dev_dbg(dev, "vp8 reject: syntax\n");
		return -EINVAL;
	}

	if (check_mul_overflow((u32)request->luma_stride,
			       (u32)ALIGN(request->coded_height, 32), &luma_size) ||
	    check_mul_overflow((u32)request->luma_stride,
			       (u32)ALIGN(request->coded_height / 2, 16),
			       &chroma_size) ||
	    check_add_overflow(request->chroma_offset, chroma_size, &surface_size) ||
	    request->chroma_offset < luma_size ||
	    !nvdec_map_is_valid(surface, DMA_BIDIRECTIONAL, surface_size)) {
		dev_dbg(dev, "vp8 reject: surface geometry/map\n");
		return -EINVAL;
	}

	if (check_mul_overflow(request->dst_stride,
			       (u32)request->crop_height / 2, &dst_size) ||
	    check_add_overflow(request->dst_chroma_offset, dst_size, &dst_size) ||
	    !request->crop_width || !request->crop_height ||
	    (request->crop_width | request->crop_height |
	     request->crop_left | request->crop_top) & 1 ||
	    request->crop_left + request->crop_width > request->coded_width ||
	    request->crop_top + request->crop_height > request->coded_height ||
	    request->dst_chroma_offset < request->dst_stride *
					 (u32)request->crop_height ||
	    !IS_ALIGNED(request->dst_stride, SZ_256) ||
	    request->dst_stride < request->crop_width ||
	    !nvdec_map_is_valid(capture, DMA_FROM_DEVICE, dst_size)) {
		dev_dbg(dev, "vp8 reject: detile destination\n");
		return -EINVAL;
	}

	for (i = 0; i < NVDEC_VP8_REFS; i++) {
		if (refs[i] &&
		    !nvdec_map_is_valid(refs[i], DMA_TO_DEVICE, surface_size)) {
			dev_dbg(dev, "vp8 reject: reference %u\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static void nvdec_vp8_fill_setup(struct nvdec_decode_job *hjob)
{
	const struct nvdec_vp8_request *r = &hjob->vp8;
	struct nvdec_vp8_setup *setup = hjob->state->cpu;
	u8 segment_update = !!(r->flags & NVDEC_VP8_REQ_SEGMENT_UPDATE);

	memset(setup, 0, sizeof(*setup));
	setup->frame_width = cpu_to_le16(r->coded_width);
	setup->frame_height = cpu_to_le16(r->coded_height);
	setup->key_frame = !!(r->flags & NVDEC_VP8_REQ_KEY_FRAME);
	setup->version = r->version;
	setup->error_conceal_on = 1;
	setup->first_part_size = cpu_to_le32(r->first_part_size);
	setup->history_buffer_size = cpu_to_le32(hjob->ctx->history_size / SZ_256);
	setup->vld_buffer_size = cpu_to_le32(r->output_payload_size);
	/* The one stride in this ABI that is not a byte count. */
	setup->framestride[0] = cpu_to_le32(r->luma_stride / 16);
	setup->framestride[1] = cpu_to_le32(r->luma_stride / 16);
	setup->segmentation_feature_data_update = segment_update;
	/* NVIDIA's driver writes this field one byte later than its header says. */
	setup->reserved1[0] = segment_update;
}

static int nvdec_vp8_build_gather(struct nvdec_decode_job *hjob)
{
	struct nvdec_engine_map *pictures[NVDEC_VP8_MAX_PICTURES];
	struct nvdec_decode_context *ctx = hjob->ctx;
	u32 *gather = hjob->gather->cpu;
	unsigned int i, word = 0;
	u32 syncpt_id;
	int err;

	/* Slots are roles, not pinned indices: golden, altref, last, current. */
	for (i = 0; i < NVDEC_VP8_REFS; i++)
		pictures[i] = hjob->dpb[i] ?: hjob->surface;
	pictures[NVDEC_VP8_REFS] = hjob->surface;

	nvdec_emit_method(gather, &word, NVDEC_METHOD_APPLICATION, 5);
	nvdec_emit_method(gather, &word, NVDEC_METHOD_CONTROL, 0x55);
	nvdec_emit_method(gather, &word, NVDEC_METHOD_PICTURE_INDEX,
			  NVDEC_VP8_REFS);
	err = nvdec_emit_address(gather, &word, NVDEC_METHOD_SETUP,
				 hjob->state->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_INPUT,
					 hjob->input->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_STATUS,
					 hjob->state->iova +
					 NVDEC_VP8_STATUS_OFFSET);
	if (!err)
		err = nvdec_emit_address(gather, &word,
					 NVDEC_VP8_METHOD_PROB_DATA,
					 hjob->probs->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_H264_METHOD_HISTORY,
					 hjob->scratch->iova + ctx->history_offset);
	for (i = 0; !err && i < NVDEC_VP8_MAX_PICTURES; i++) {
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_LUMA + i,
					 pictures[i]->iova);
		if (!err)
			err = nvdec_emit_address(gather, &word,
						 NVDEC_METHOD_CHROMA + i,
						 pictures[i]->iova +
						 hjob->vp8.chroma_offset);
	}
	if (err)
		return err;
	nvdec_emit_method(gather, &word, NVDEC_METHOD_EXECUTE, 0x100);
	if (WARN_ON_ONCE(word != NVDEC_VP8_GATHER_WORDS))
		return -EINVAL;

	syncpt_id = host1x_syncpt_id(ctx->engine->client.base.syncpts[0]);
	gather[word++] = 0x20000001;
	gather[word++] = syncpt_id | 0x100;

	err = vic_engine_emit_detile(gather, &word,
				     hjob->state->iova +
				     NVDEC_VP8_VIC_CONFIG_OFFSET,
				     hjob->surface->iova,
				     hjob->surface->iova + hjob->vp8.chroma_offset,
				     hjob->capture->iova,
				     hjob->capture->iova +
				     hjob->vp8.dst_chroma_offset);
	if (err)
		return err;

	gather[word++] = 0x20000001;
	gather[word++] = syncpt_id | 0x100;
	WARN_ON_ONCE(word != NVDEC_VP8_TOTAL_GATHER_WORDS);

	print_hex_dump_debug("nvdec vp8 setup: ", DUMP_PREFIX_OFFSET, 16, 4,
			     hjob->state->cpu, NVDEC_VP8_SETUP_SIZE, false);
	return 0;
}

int nvdec_engine_vp8_submit(struct nvdec_decode_context *ctx,
			    const struct nvdec_vp8_request *request,
			    struct nvdec_engine_map *surface,
			    struct nvdec_engine_map *capture,
			    struct nvdec_engine_map * const refs[NVDEC_VP8_REFS],
			    struct dma_fence **fence,
			    nvdec_engine_complete_t complete, void *data)
{
	struct nvdec_decode_job *hjob;
	unsigned int i;
	int err;

	if (!ctx || !request || !surface || !capture || !refs || !fence ||
	    ctx->codec != NVDEC_CODEC_VP8)
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
	hjob->vp8 = *request;
	hjob->vp8.output_payload_size = ctx->staged;
	hjob->complete = complete;
	hjob->complete_data = data;
	err = nvdec_vp8_validate_request(ctx->engine->dev, &hjob->vp8, surface,
					 capture, refs);
	if (err)
		goto free_hjob;

	err = nvdec_vp8_prepare_scratch(ctx, &hjob->vp8);
	if (err)
		goto free_hjob;
	hjob->scratch = nvdec_engine_map_get(ctx->scratch);
	hjob->probs = nvdec_buffer_ref(ctx->probs);

	hjob->vic = vic_engine_find(ctx->engine->client.drm);
	if (!hjob->vic) {
		err = -ENODEV;
		goto free_hjob;
	}
	hjob->state = nvdec_buffer_alloc(ctx->engine, NVDEC_VP8_STATE_SIZE);
	if (IS_ERR(hjob->state)) {
		err = PTR_ERR(hjob->state);
		hjob->state = NULL;
		goto free_hjob;
	}
	hjob->input = nvdec_buffer_ref(ctx->input);
	hjob->gather = nvdec_buffer_alloc(ctx->engine,
					  NVDEC_VP8_TOTAL_GATHER_WORDS * sizeof(u32));
	if (IS_ERR(hjob->gather)) {
		err = PTR_ERR(hjob->gather);
		hjob->gather = NULL;
		goto free_hjob;
	}
	if (upper_32_bits(hjob->state->iova) || upper_32_bits(hjob->input->iova) ||
	    upper_32_bits(hjob->gather->iova)) {
		err = -ERANGE;
		goto free_hjob;
	}

	hjob->surface = nvdec_engine_map_get(surface);
	hjob->capture = nvdec_engine_map_get(capture);
	for (i = 0; i < NVDEC_VP8_REFS; i++) {
		if (refs[i])
			hjob->dpb[i] = nvdec_engine_map_get(refs[i]);
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

	if (hjob->vp8.flags & NVDEC_VP8_REQ_KEY_FRAME)
		nvdec_vp8_init_probs(ctx);
	nvdec_vp8_fill_setup(hjob);
	vic_engine_fill_detile_config(hjob->state->cpu + NVDEC_VP8_VIC_CONFIG_OFFSET,
				      &(struct vic_detile_params){
					.width = hjob->vp8.coded_width,
					.height = hjob->vp8.coded_height,
					.left = hjob->vp8.crop_left,
					.top = hjob->vp8.crop_top,
					.out_width = hjob->vp8.crop_width,
					.out_height = hjob->vp8.crop_height,
					.src_stride = hjob->vp8.luma_stride,
					.dst_stride = hjob->vp8.dst_stride,
				      });
	err = nvdec_vp8_build_gather(hjob);
	if (err)
		goto free_hjob;

	err = nvdec_launch_job(hjob, NVDEC_VP8_GATHER_WORDS,
			       NVDEC_VP8_VIC_OFFSET, fence);
	if (err)
		goto free_hjob;
	mutex_unlock(&ctx->lock);
	return 0;

free_hjob:
	nvdec_free_job(hjob);
unlock:
	mutex_unlock(&ctx->lock);
	return err;
}

static void nvdec_vp9_fill_probs(void *mem, const struct nvdec_vp9_request *r)
{
	const struct v4l2_vp9_frame_context *p = &r->probs;
	struct nvdec_vp9_probs *probs = mem;
	unsigned int i, j, k, l;

	memset(probs, 0, sizeof(*probs));

	for (i = 0; i < 10; i++) {
		for (j = 0; j < 10; j++) {
			memcpy(probs->kf_bmode_prob[i][j],
			       v4l2_vp9_kf_y_mode_prob[i][j], 8);
			probs->kf_bmode_prob_b[i][j][0] =
				v4l2_vp9_kf_y_mode_prob[i][j][8];
		}
		memcpy(probs->kf_uv_mode_prob[i], v4l2_vp9_kf_uv_mode_prob[i], 8);
		probs->kf_uv_mode_prob_b[i][0] = v4l2_vp9_kf_uv_mode_prob[i][8];
		memcpy(probs->uv_mode_prob[i], p->uv_mode[i], 8);
		probs->uv_mode_prob_b[i][0] = p->uv_mode[i][8];
	}

	memcpy(probs->mb_segment_tree_probs, r->seg_tree_probs,
	       sizeof(probs->mb_segment_tree_probs));
	memcpy(probs->segment_pred_probs, r->seg_pred_probs,
	       sizeof(probs->segment_pred_probs));

	for (i = 0; i < 7; i++)
		memcpy(probs->inter_mode_prob[i], p->inter_mode[i], 3);
	memcpy(probs->intra_inter_prob, p->is_inter, sizeof(probs->intra_inter_prob));
	memcpy(probs->tx8x8_prob, p->tx8, sizeof(probs->tx8x8_prob));
	memcpy(probs->tx16x16_prob, p->tx16, sizeof(probs->tx16x16_prob));
	memcpy(probs->tx32x32_prob, p->tx32, sizeof(probs->tx32x32_prob));
	for (i = 0; i < 4; i++) {
		memcpy(probs->sb_ymode_prob[i], p->y_mode[i], 8);
		probs->sb_ymode_prob_b[i][0] = p->y_mode[i][8];
	}
	for (i = 0; i < 16; i++) {
		memcpy(probs->partition_prob[0][i], v4l2_vp9_kf_partition_probs[i], 3);
		memcpy(probs->partition_prob[1][i], p->partition[i], 3);
	}
	memcpy(probs->switchable_interp_prob, p->interp_filter,
	       sizeof(probs->switchable_interp_prob));
	memcpy(probs->comp_inter_prob, p->comp_mode, sizeof(probs->comp_inter_prob));
	memcpy(probs->mbskip_probs, p->skip, sizeof(probs->mbskip_probs));
	memcpy(probs->single_ref_prob, p->single_ref, sizeof(probs->single_ref_prob));
	memcpy(probs->comp_ref_prob, p->comp_ref, sizeof(probs->comp_ref_prob));

	memcpy(probs->nmvc.joints, p->mv.joint, sizeof(probs->nmvc.joints));
	memcpy(probs->nmvc.sign, p->mv.sign, sizeof(probs->nmvc.sign));
	memcpy(probs->nmvc.class0, p->mv.class0_bit, sizeof(probs->nmvc.class0));
	memcpy(probs->nmvc.fp, p->mv.fr, sizeof(probs->nmvc.fp));
	memcpy(probs->nmvc.class0_hp, p->mv.class0_hp, sizeof(probs->nmvc.class0_hp));
	memcpy(probs->nmvc.hp, p->mv.hp, sizeof(probs->nmvc.hp));
	memcpy(probs->nmvc.classes, p->mv.classes, sizeof(probs->nmvc.classes));
	memcpy(probs->nmvc.class0_fp, p->mv.class0_fr, sizeof(probs->nmvc.class0_fp));
	memcpy(probs->nmvc.bits, p->mv.bits, sizeof(probs->nmvc.bits));

	for (i = 0; i < 4; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 2; k++)
				for (l = 0; l < 6; l++) {
					unsigned int m;

					for (m = 0; m < 6; m++)
						memcpy(probs->coeff[i][j][k][l][m],
						       p->coef[i][j][k][l][m], 3);
				}
}

static void nvdec_vp9_fill_tile_sizes(void *mem, const struct nvdec_vp9_request *r)
{
	unsigned int cols = 1U << r->tile_cols_log2;
	unsigned int rows = 1U << r->tile_rows_log2;
	unsigned int sb_cols = DIV_ROUND_UP(r->width, NVDEC_VP9_SB_SIZE);
	unsigned int sb_rows = DIV_ROUND_UP(r->height, NVDEC_VP9_SB_SIZE);
	__le16 *sizes = mem;
	unsigned int i, j, n = 0;

	memset(mem, 0, NVDEC_VP9_TILES_SIZE);
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			sizes[n++] = cpu_to_le16((sb_cols * (j + 1) >> r->tile_cols_log2) -
						 (sb_cols * j >> r->tile_cols_log2));
			sizes[n++] = cpu_to_le16((sb_rows * (i + 1) >> r->tile_rows_log2) -
						 (sb_rows * i >> r->tile_rows_log2));
		}
	}
	sizes[NVDEC_VP9_TILES_MAGIC] = cpu_to_le16(9);
	sizes[NVDEC_VP9_TILES_MAGIC + 1] = cpu_to_le16(1);
}

/* No V4L2 control carries these; 6.2 setup_compound_reference_mode() does. */
static void nvdec_vp9_compound_refs(const struct nvdec_vp9_request *r,
				    u8 *fixed, u8 *var)
{
	const u8 *bias = r->sign_bias;

	if (bias[0] == bias[1] && bias[0] == bias[2]) {
		*fixed = 0;
		var[0] = 0;
		var[1] = 0;
	} else if (bias[0] == bias[1]) {
		*fixed = 3;
		var[0] = 1;
		var[1] = 2;
	} else if (bias[0] == bias[2]) {
		*fixed = 2;
		var[0] = 1;
		var[1] = 3;
	} else {
		*fixed = 1;
		var[0] = 2;
		var[1] = 3;
	}
}

static void nvdec_vp9_fill_setup(struct nvdec_decode_job *hjob)
{
	const struct nvdec_vp9_request *r = &hjob->vp9;
	struct nvdec_decode_context *ctx = hjob->ctx;
	struct nvdec_vp9_setup *setup = hjob->state->cpu;
	u8 fixed_ref, var_ref[2];
	unsigned int i;
	u32 word;

	memset(setup, 0, sizeof(*setup));
	setup->stream_len = cpu_to_le32(r->output_payload_size);
	setup->bsd_control_offset =
		cpu_to_le32(ALIGN(r->coded_height, NVDEC_VP9_SB_SIZE) *
			    NVDEC_VP9_BSD_PER_ROW / SZ_256);

	/* The surface pool is per context, so no reference can be scaled. */
	for (i = 0; i < NVDEC_VP9_REFS; i++) {
		setup->ref[i].width = cpu_to_le16(r->width);
		setup->ref[i].height = cpu_to_le16(r->height);
		setup->ref[i].stride[0] = cpu_to_le16(r->luma_stride);
		setup->ref[i].stride[1] = cpu_to_le16(r->luma_stride);
	}
	setup->width = cpu_to_le16(r->width);
	setup->height = cpu_to_le16(r->height);
	setup->framestride[0] = cpu_to_le16(r->luma_stride);
	setup->framestride[1] = cpu_to_le16(r->luma_stride);

	word = 0;
	if (r->flags & NVDEC_VP9_REQ_KEY_FRAME)
		word |= NVDEC_VP9_PIC_KEY_FRAME;
	if (r->flags & NVDEC_VP9_REQ_PREV_KEY_FRAME)
		word |= NVDEC_VP9_PIC_PREV_KEY_FRAME;
	if (r->flags & NVDEC_VP9_REQ_ERROR_RESILIENT)
		word |= NVDEC_VP9_PIC_ERROR_RESILIENT;
	if (r->flags & NVDEC_VP9_REQ_PREV_SHOW_FRAME)
		word |= NVDEC_VP9_PIC_PREV_SHOW_FRAME;
	if (r->flags & NVDEC_VP9_REQ_INTRA_ONLY)
		word |= NVDEC_VP9_PIC_INTRA_ONLY;
	setup->picture_flags = cpu_to_le32(word);

	for (i = 0; i < NVDEC_VP9_REFS; i++)
		setup->ref_frame_sign_bias[i + 1] = r->sign_bias[i];

	setup->loop_filter_level = r->lf_level;
	setup->loop_filter_sharpness = r->lf_sharpness;
	setup->qp_y_ac = r->base_q_idx;
	setup->qp_y_dc = r->delta_q_y_dc;
	setup->qp_ch_dc = r->delta_q_uv_dc;
	setup->qp_ch_ac = r->delta_q_uv_ac;

	setup->lossless = !!(r->flags & NVDEC_VP9_REQ_LOSSLESS);
	setup->transform_mode = r->tx_mode;
	setup->allow_high_precision_mv = !!(r->flags & NVDEC_VP9_REQ_HIGH_PREC_MV);
	/* The firmware orders SMOOTH before EIGHTTAP; the spec is the reverse. */
	setup->mcomp_filter_type = r->interpolation_filter ^
				   (r->interpolation_filter <= 1);
	setup->comp_pred_mode = r->reference_mode;
	nvdec_vp9_compound_refs(r, &fixed_ref, var_ref);
	setup->comp_fixed_ref = fixed_ref;
	setup->comp_var_ref[0] = var_ref[0];
	setup->comp_var_ref[1] = var_ref[1];
	setup->log2_tile_columns = r->tile_cols_log2;
	setup->log2_tile_rows = r->tile_rows_log2;

	setup->segment_enabled = !!(r->flags & NVDEC_VP9_REQ_SEG_ENABLED);
	setup->segment_map_update = !!(r->flags & NVDEC_VP9_REQ_SEG_UPDATE_MAP);
	setup->segment_map_temporal_update =
		!!(r->flags & NVDEC_VP9_REQ_SEG_TEMPORAL);
	setup->segment_feature_mode = !!(r->flags & NVDEC_VP9_REQ_SEG_ABS_DELTA);
	for (i = 0; i < 8; i++) {
		unsigned int j;

		for (j = 0; j < 4; j++) {
			setup->segment_feature_enable[i][j] =
				!!(r->seg_feature_enabled[i] & BIT(j));
			setup->segment_feature_data[i][j] =
				cpu_to_le16(r->seg_feature_data[i][j]);
		}
	}
	setup->mode_ref_lf_enabled = !!(r->flags & NVDEC_VP9_REQ_LF_DELTA_ENABLED);
	memcpy(setup->mb_ref_lf_delta, r->lf_ref_deltas,
	       sizeof(setup->mb_ref_lf_delta));
	memcpy(setup->mb_mode_lf_delta, r->lf_mode_deltas,
	       sizeof(setup->mb_mode_lf_delta));

	nvdec_vp9_fill_probs(hjob->state->cpu + NVDEC_VP9_PROBS_OFFSET, r);
	nvdec_vp9_fill_tile_sizes(hjob->state->cpu + NVDEC_VP9_TILES_OFFSET, r);
	memset(ctx->counts->cpu, 0, sizeof(struct nvdec_vp9_counts));
}

static int nvdec_vp9_prepare_scratch(struct nvdec_decode_context *ctx,
				     const struct nvdec_vp9_request *request)
{
	u32 aligned_height, superblocks, segment, filter, colmv, offset, size;
	struct nvdec_engine_map *scratch;
	struct nvdec_buffer *counts;

	if (ctx->scratch) {
		if (ctx->coded_width != request->coded_width ||
		    ctx->coded_height != request->coded_height)
			return -EBUSY;
		return 0;
	}

	aligned_height = ALIGN(request->coded_height, NVDEC_VP9_SB_SIZE);
	superblocks = (request->coded_width / NVDEC_VP9_SB_SIZE) *
		      (aligned_height / NVDEC_VP9_SB_SIZE);
	segment = ALIGN(superblocks * 32, SZ_256);
	filter = aligned_height * NVDEC_VP9_FILTER_PER_ROW;
	colmv = superblocks * SZ_1K;

	offset = 0;
	ctx->seg_read_offset = offset;
	offset += segment;
	ctx->seg_write_offset = offset;
	offset += segment;
	ctx->filter_offset = offset;
	offset = ALIGN(offset + filter, SZ_256);
	ctx->colmv_offset[0] = offset;
	offset += colmv;
	ctx->colmv_offset[1] = offset;
	size = ALIGN(offset + colmv, SZ_4K);

	counts = nvdec_buffer_alloc(ctx->engine,
				    ALIGN(sizeof(struct nvdec_vp9_counts), SZ_256));
	if (IS_ERR(counts))
		return PTR_ERR(counts);
	scratch = nvdec_engine_surface_create(ctx->engine, size);
	if (IS_ERR(scratch)) {
		nvdec_buffer_put(&counts->bo);
		return PTR_ERR(scratch);
	}
	if (upper_32_bits(counts->iova) || upper_32_bits(scratch->iova)) {
		nvdec_engine_map_put(scratch);
		nvdec_buffer_put(&counts->bo);
		return -ERANGE;
	}

	ctx->counts = counts;
	ctx->scratch = scratch;
	ctx->coded_width = request->coded_width;
	ctx->coded_height = request->coded_height;
	ctx->frame_parity = 0;
	return 0;
}

static int nvdec_vp9_validate_request(struct device *dev,
				      const struct nvdec_vp9_request *request,
				      const struct nvdec_engine_map *surface,
				      const struct nvdec_engine_map *capture,
				      struct nvdec_engine_map * const refs[])
{
	u32 luma_size, chroma_size, surface_size, dst_size, tiles;
	unsigned int i;

	dev_dbg(dev,
		"vp9 request: %ux%u coded=%ux%u crop=%ux%u flags=0x%x q=%u tx=%u refmode=%u filter=%u tiles=%u/%u stride=%u coff=%u payload=%u\n",
		request->width, request->height, request->coded_width,
		request->coded_height, request->crop_width, request->crop_height,
		request->flags, request->base_q_idx, request->tx_mode,
		request->reference_mode, request->interpolation_filter,
		request->tile_cols_log2, request->tile_rows_log2,
		request->luma_stride, request->chroma_offset,
		request->output_payload_size);

	if (!request->width || !request->height ||
	    request->width > request->coded_width ||
	    request->height > request->coded_height ||
	    request->coded_width > 4096 || request->coded_height > 4096 ||
	    !IS_ALIGNED(request->coded_width, NVDEC_VP9_SB_SIZE) ||
	    !IS_ALIGNED(request->coded_height, NVDEC_VP9_SB_SIZE) ||
	    !request->output_payload_size ||
	    request->tx_mode > V4L2_VP9_TX_MODE_SELECT ||
	    request->reference_mode > V4L2_VP9_REFERENCE_MODE_SELECT ||
	    request->interpolation_filter > V4L2_VP9_INTERP_FILTER_SWITCHABLE ||
	    request->base_q_idx > 255 ||
	    !IS_ALIGNED(request->luma_stride, 16)) {
		dev_dbg(dev, "vp9 reject: syntax\n");
		return -EINVAL;
	}

	tiles = (1U << request->tile_cols_log2) * (1U << request->tile_rows_log2);
	if (request->tile_cols_log2 > 6 || request->tile_rows_log2 > 2 ||
	    tiles * 2 * sizeof(__le16) > NVDEC_VP9_TILES_MAGIC * sizeof(__le16)) {
		dev_dbg(dev, "vp9 reject: tiles\n");
		return -EINVAL;
	}

	if (check_mul_overflow((u32)request->luma_stride,
			       (u32)ALIGN(request->coded_height, 32), &luma_size) ||
	    check_mul_overflow((u32)request->luma_stride,
			       (u32)ALIGN(request->coded_height / 2, 16),
			       &chroma_size) ||
	    check_add_overflow(request->chroma_offset, chroma_size, &surface_size) ||
	    request->chroma_offset < luma_size ||
	    !nvdec_map_is_valid(surface, DMA_BIDIRECTIONAL, surface_size)) {
		dev_dbg(dev, "vp9 reject: surface geometry/map\n");
		return -EINVAL;
	}

	if (check_mul_overflow(request->dst_stride,
			       (u32)request->crop_height / 2, &dst_size) ||
	    check_add_overflow(request->dst_chroma_offset, dst_size, &dst_size) ||
	    !request->crop_width || !request->crop_height ||
	    (request->crop_width | request->crop_height |
	     request->crop_left | request->crop_top) & 1 ||
	    request->crop_left + request->crop_width > request->coded_width ||
	    request->crop_top + request->crop_height > request->coded_height ||
	    request->dst_chroma_offset < request->dst_stride *
					 (u32)request->crop_height ||
	    !IS_ALIGNED(request->dst_stride, SZ_256) ||
	    request->dst_stride < request->crop_width ||
	    !nvdec_map_is_valid(capture, DMA_FROM_DEVICE, dst_size)) {
		dev_dbg(dev, "vp9 reject: detile destination\n");
		return -EINVAL;
	}

	for (i = 0; i < NVDEC_VP9_REFS; i++) {
		if (refs[i] &&
		    !nvdec_map_is_valid(refs[i], DMA_TO_DEVICE, surface_size)) {
			dev_dbg(dev, "vp9 reject: reference %u\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static int nvdec_vp9_build_gather(struct nvdec_decode_job *hjob)
{
	struct nvdec_engine_map *pictures[NVDEC_VP9_REFS + 1];
	struct nvdec_decode_context *ctx = hjob->ctx;
	u32 *gather = hjob->gather->cpu;
	unsigned int i, word = 0;
	u32 syncpt_id, write, read;
	int err;

	/* Slots are roles, not pinned indices: last, golden, altref, current. */
	for (i = 0; i < NVDEC_VP9_REFS; i++)
		pictures[i] = hjob->dpb[i] ?: hjob->surface;
	pictures[NVDEC_VP9_REFS] = hjob->surface;

	write = ctx->colmv_offset[ctx->frame_parity];
	read = ctx->colmv_offset[!ctx->frame_parity];

	nvdec_emit_method(gather, &word, NVDEC_METHOD_APPLICATION, 9);
	nvdec_emit_method(gather, &word, NVDEC_METHOD_CONTROL, 0x59);
	nvdec_emit_method(gather, &word, NVDEC_METHOD_PICTURE_INDEX,
			  NVDEC_VP9_REFS);
	err = nvdec_emit_address(gather, &word, NVDEC_METHOD_SETUP,
				 hjob->state->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_INPUT,
					 hjob->input->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_STATUS,
					 hjob->state->iova +
					 NVDEC_VP9_STATUS_OFFSET);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_VP9_METHOD_PROB_TAB,
					 hjob->state->iova +
					 NVDEC_VP9_PROBS_OFFSET);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_VP9_METHOD_CTX_COUNTER,
					 ctx->counts->iova);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_VP9_METHOD_TILE_SIZE,
					 hjob->state->iova +
					 NVDEC_VP9_TILES_OFFSET);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_VP9_METHOD_COL_MVWRITE,
					 hjob->scratch->iova + write);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_VP9_METHOD_COL_MVREAD,
					 hjob->scratch->iova + read);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_VP9_METHOD_SEGMENT_READ,
					 hjob->scratch->iova + ctx->seg_read_offset);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_VP9_METHOD_SEGMENT_WRITE,
					 hjob->scratch->iova + ctx->seg_write_offset);
	if (!err)
		err = nvdec_emit_address(gather, &word, NVDEC_VP9_METHOD_FILTER,
					 hjob->scratch->iova + ctx->filter_offset);
	for (i = 0; !err && i < NVDEC_VP9_REFS + 1; i++) {
		err = nvdec_emit_address(gather, &word, NVDEC_METHOD_LUMA + i,
					 pictures[i]->iova);
		if (!err)
			err = nvdec_emit_address(gather, &word,
						 NVDEC_METHOD_CHROMA + i,
						 pictures[i]->iova +
						 hjob->vp9.chroma_offset);
	}
	if (err)
		return err;
	nvdec_emit_method(gather, &word, NVDEC_METHOD_EXECUTE, 0x100);
	if (WARN_ON_ONCE(word != NVDEC_VP9_GATHER_WORDS))
		return -EINVAL;

	syncpt_id = host1x_syncpt_id(ctx->engine->client.base.syncpts[0]);
	gather[word++] = 0x20000001;
	gather[word++] = syncpt_id | 0x100;

	err = vic_engine_emit_detile(gather, &word,
				     hjob->state->iova +
				     NVDEC_VP9_VIC_CONFIG_OFFSET,
				     hjob->surface->iova,
				     hjob->surface->iova + hjob->vp9.chroma_offset,
				     hjob->capture->iova,
				     hjob->capture->iova +
				     hjob->vp9.dst_chroma_offset);
	if (err)
		return err;

	gather[word++] = 0x20000001;
	gather[word++] = syncpt_id | 0x100;
	WARN_ON_ONCE(word != NVDEC_VP9_TOTAL_GATHER_WORDS);

	print_hex_dump_debug("nvdec vp9 setup: ", DUMP_PREFIX_OFFSET, 16, 4,
			     hjob->state->cpu, NVDEC_VP9_SETUP_SIZE, false);
	return 0;
}

/* Only the inter-mode tree and the TX16 row stride need converting. */
int nvdec_engine_vp9_counts(struct nvdec_decode_context *ctx,
			    struct v4l2_vp9_frame_symbol_counts *counts)
{
	unsigned int i, j, k, l, m;
	struct nvdec_vp9_counts *c;

	if (!ctx || !counts || ctx->codec != NVDEC_CODEC_VP9 || !ctx->counts)
		return -EINVAL;

	c = ctx->counts->cpu;
	/* The firmware counts the three nodes of the inter-mode tree. */
	for (i = 0; i < 7; i++) {
		ctx->mv_mode[i][0] = c->inter_mode[i][1][0];
		ctx->mv_mode[i][1] = c->inter_mode[i][2][0];
		ctx->mv_mode[i][2] = c->inter_mode[i][0][0];
		ctx->mv_mode[i][3] = c->inter_mode[i][2][1];
	}
	/* Same values as tx16x16, but V4L2 indexes rows of four. */
	for (i = 0; i < 2; i++)
		for (j = 0; j < 3; j++)
			ctx->tx16p[i][j] = c->tx16x16[i][j];

	counts->partition = &c->partition;
	counts->skip = &c->skip;
	counts->intra_inter = &c->intra_inter;
	counts->tx32p = &c->tx32x32;
	counts->tx16p = &ctx->tx16p;
	counts->tx8p = &c->tx8x8;
	counts->y_mode = &c->y_mode;
	counts->uv_mode = &c->uv_mode;
	counts->comp = &c->comp_inter;
	counts->comp_ref = &c->comp_ref;
	counts->single_ref = &c->single_ref;
	counts->mv_mode = &ctx->mv_mode;
	counts->filter = &c->interp_filter;
	counts->mv_joint = &c->mv.joints;
	counts->sign = &c->mv.sign;
	counts->classes = &c->mv.classes;
	counts->class0 = &c->mv.class0;
	counts->bits = &c->mv.bits;
	counts->class0_fp = &c->mv.class0_fp;
	counts->fp = &c->mv.fp;
	counts->class0_hp = &c->mv.class0_hp;
	counts->hp = &c->mv.hp;

	for (i = 0; i < 4; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 2; k++)
				for (l = 0; l < 6; l++)
					for (m = 0; m < 6; m++) {
						counts->coeff[i][j][k][l][m] =
							(u32 (*)[3])c->coeff[i][j][k][l][m];
						counts->eob[i][j][k][l][m][0] =
							&c->eob[i][j][k][l][m];
						counts->eob[i][j][k][l][m][1] =
							&c->coeff[i][j][k][l][m][3];
					}

	return 0;
}

int nvdec_engine_vp9_submit(struct nvdec_decode_context *ctx,
			    const struct nvdec_vp9_request *request,
			    struct nvdec_engine_map *surface,
			    struct nvdec_engine_map *capture,
			    struct nvdec_engine_map * const refs[NVDEC_VP9_REFS],
			    struct dma_fence **fence,
			    nvdec_engine_complete_t complete, void *data)
{
	struct nvdec_decode_job *hjob;
	unsigned int i;
	int err;

	if (!ctx || !request || !surface || !capture || !refs || !fence ||
	    ctx->codec != NVDEC_CODEC_VP9)
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
	hjob->vp9 = *request;
	hjob->vp9.output_payload_size = ctx->staged;
	hjob->complete = complete;
	hjob->complete_data = data;
	err = nvdec_vp9_validate_request(ctx->engine->dev, &hjob->vp9, surface,
					 capture, refs);
	if (err)
		goto free_hjob;

	err = nvdec_vp9_prepare_scratch(ctx, &hjob->vp9);
	if (err)
		goto free_hjob;
	hjob->scratch = nvdec_engine_map_get(ctx->scratch);
	hjob->counts = nvdec_buffer_ref(ctx->counts);

	hjob->vic = vic_engine_find(ctx->engine->client.drm);
	if (!hjob->vic) {
		err = -ENODEV;
		goto free_hjob;
	}
	hjob->state = nvdec_buffer_alloc(ctx->engine, NVDEC_VP9_STATE_SIZE);
	if (IS_ERR(hjob->state)) {
		err = PTR_ERR(hjob->state);
		hjob->state = NULL;
		goto free_hjob;
	}
	hjob->input = nvdec_buffer_ref(ctx->input);
	hjob->gather = nvdec_buffer_alloc(ctx->engine,
					  NVDEC_VP9_TOTAL_GATHER_WORDS * sizeof(u32));
	if (IS_ERR(hjob->gather)) {
		err = PTR_ERR(hjob->gather);
		hjob->gather = NULL;
		goto free_hjob;
	}
	if (upper_32_bits(hjob->state->iova) || upper_32_bits(hjob->input->iova) ||
	    upper_32_bits(hjob->gather->iova)) {
		err = -ERANGE;
		goto free_hjob;
	}

	hjob->surface = nvdec_engine_map_get(surface);
	hjob->capture = nvdec_engine_map_get(capture);
	for (i = 0; i < NVDEC_VP9_REFS; i++) {
		if (refs[i])
			hjob->dpb[i] = nvdec_engine_map_get(refs[i]);
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

	nvdec_vp9_fill_setup(hjob);
	vic_engine_fill_detile_config(hjob->state->cpu + NVDEC_VP9_VIC_CONFIG_OFFSET,
				      &(struct vic_detile_params){
					.width = hjob->vp9.coded_width,
					.height = hjob->vp9.coded_height,
					.left = hjob->vp9.crop_left,
					.top = hjob->vp9.crop_top,
					.out_width = hjob->vp9.crop_width,
					.out_height = hjob->vp9.crop_height,
					.src_stride = hjob->vp9.luma_stride,
					.dst_stride = hjob->vp9.dst_stride,
				      });
	err = nvdec_vp9_build_gather(hjob);
	if (err)
		goto free_hjob;

	err = nvdec_launch_job(hjob, NVDEC_VP9_GATHER_WORDS,
			       NVDEC_VP9_VIC_OFFSET, fence);
	if (err)
		goto free_hjob;

	/* Colocated alternates per frame; the segment map only after a write. */
	ctx->frame_parity = !ctx->frame_parity;
	if (hjob->vp9.flags & NVDEC_VP9_REQ_SEG_UPDATE_MAP)
		swap(ctx->seg_read_offset, ctx->seg_write_offset);
	mutex_unlock(&ctx->lock);
	return 0;

free_hjob:
	nvdec_free_job(hjob);
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
