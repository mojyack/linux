/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2026 mojyack <mojyack@gmail.com>
 */

#ifndef TEGRA_NVDEC_DECODE_H
#define TEGRA_NVDEC_DECODE_H

#include <linux/bitops.h>
#include <linux/dma-direction.h>

#include <media/v4l2-vp9.h>

struct dma_buf;
struct dma_resv;
struct host1x_channel;
struct nvdec_decode_map;
struct nvdec_decode_context;
struct tegra_drm_client;

/* Where a picture is decoded to and detiled from; common to every codec. */
struct nvdec_frame {
	u16 coded_width;
	u16 coded_height;
	/* Visible rectangle VIC detiles out of the coded picture. */
	u16 crop_left;
	u16 crop_top;
	u16 crop_width;
	u16 crop_height;
	u16 luma_stride;
	u32 chroma_offset;
	u32 dst_stride;
	u32 dst_chroma_offset;
	u32 output_payload_size;
};

/* The most references any codec passes, and the most firmware picture slots. */
#define NVDEC_MAX_REFS		16
#define NVDEC_MAX_PICTURES	17

#define NVDEC_H264_DPB_ENTRIES	16
/* 16 references plus the current picture; one firmware index each. */
#define NVDEC_H264_MAX_PICTURES	NVDEC_MAX_PICTURES

/* Copied, validated H.264 state. This is not a V4L2 control layout. */
struct nvdec_h264_request {
	struct nvdec_frame frame;
	u8 profile_idc;
	u8 level_idc;
	u8 chroma_format_idc;
	u8 bit_depth_luma_minus8;
	u8 bit_depth_chroma_minus8;
	u8 log2_max_frame_num_minus4;
	u8 pic_order_cnt_type;
	u8 log2_max_pic_order_cnt_lsb_minus4;
	u8 max_num_ref_frames;
	u8 num_ref_idx_l0_active_minus1;
	u8 num_ref_idx_l1_active_minus1;
	u8 nal_ref_idc;
	u16 flags;
	u16 frame_num;
	u16 pic_width_in_mbs;
	u16 frame_height_in_mbs;
	u32 slice_count;
	s32 top_field_order_cnt;
	s32 bottom_field_order_cnt;
	s8 pic_init_qp_minus26;
	s8 chroma_qp_index_offset;
	s8 second_chroma_qp_index_offset;
	u8 weighted_bipred_idc;
	u8 pps_flags;
	u8 num_slice_groups_minus1;
	u8 reserved;
	u8 scaling_4x4[6][16];
	u8 scaling_8x8[2][64];
	struct {
		u8 valid;
		u8 long_term;
		/* Which parities are marked as references; see NVDEC_H264_REF_*. */
		u8 fields;
		u8 field_picture;
		u16 frame_num;
		s32 top_field_order_cnt;
		s32 bottom_field_order_cnt;
	} dpb[NVDEC_H264_DPB_ENTRIES];
};

#define NVDEC_H264_REQ_FRAME_MBS_ONLY	BIT(0)
#define NVDEC_H264_REQ_MBAFF		BIT(1)
#define NVDEC_H264_REQ_DELTA_POC_ZERO	BIT(3)
#define NVDEC_H264_REQ_DIRECT_8X8	BIT(4)
#define NVDEC_H264_REQ_IDR		BIT(5)
#define NVDEC_H264_REQ_FIELD		BIT(6)
#define NVDEC_H264_REQ_BOTTOM_FIELD	BIT(7)
#define NVDEC_H264_REQ_SECOND_FIELD	BIT(8)

#define NVDEC_H264_REF_TOP	BIT(0)
#define NVDEC_H264_REF_BOTTOM	BIT(1)

#define NVDEC_H264_PPS_ENTROPY_CODING	BIT(0)
#define NVDEC_H264_PPS_PIC_ORDER_PRESENT	BIT(1)
#define NVDEC_H264_PPS_WEIGHTED_PRED	BIT(2)
#define NVDEC_H264_PPS_DEBLOCK		BIT(3)
#define NVDEC_H264_PPS_REDUNDANT	BIT(4)
#define NVDEC_H264_PPS_TRANSFORM_8X8	BIT(5)
#define NVDEC_H264_PPS_CONSTRAINED_INTRA	BIT(6)

#define NVDEC_HEVC_DPB_ENTRIES	16
/* 15 references plus the current picture; one firmware index each. */
#define NVDEC_HEVC_MAX_PICTURES	16
/* The largest coding tree unit, which the coded size is padded up to. */
#define NVDEC_HEVC_CTU_SIZE	64
/* Smallest picture side the firmware decodes correctly. */
#define NVDEC_HEVC_MIN_SIZE	129

/* Copied, validated HEVC state. This is not a V4L2 control layout. */
struct nvdec_hevc_request {
	struct nvdec_frame frame;
	u16 pic_width_in_luma_samples;
	u16 pic_height_in_luma_samples;
	u16 ctb_width;
	u16 ctb_height;
	u16 sw_hdr_skip_length;
	s32 pic_order_cnt_val;
	u32 sps_flags;
	u32 pps_flags;
	u8 bit_depth;
	u8 log2_min_luma_coding_block_size;
	u8 log2_max_luma_coding_block_size;
	u8 log2_min_transform_block_size;
	u8 log2_max_transform_block_size;
	u8 max_transform_hierarchy_depth_inter;
	u8 max_transform_hierarchy_depth_intra;
	u8 pcm_sample_bit_depth_luma;
	u8 pcm_sample_bit_depth_chroma;
	u8 log2_min_pcm_luma_coding_block_size;
	u8 log2_max_pcm_luma_coding_block_size;
	u8 num_extra_slice_header_bits;
	u8 num_ref_idx_l0_default_active;
	u8 num_ref_idx_l1_default_active;
	u8 init_qp;
	u8 diff_cu_qp_delta_depth;
	u8 num_tile_columns;
	u8 num_tile_rows;
	u8 log2_parallel_merge_level;
	u8 num_ref_frames;
	u8 num_active_dpb_entries;
	u8 num_poc_st_curr_before;
	u8 num_poc_st_curr_after;
	u8 num_poc_lt_curr;
	s8 pps_cb_qp_offset;
	s8 pps_cr_qp_offset;
	s8 pps_beta_offset;
	s8 pps_tc_offset;
	u16 column_width[20];
	u16 row_height[22];
	u8 poc_st_curr_before[NVDEC_HEVC_DPB_ENTRIES];
	u8 poc_st_curr_after[NVDEC_HEVC_DPB_ENTRIES];
	u8 poc_lt_curr[NVDEC_HEVC_DPB_ENTRIES];
	struct {
		u8 valid;
		u8 long_term;
		s32 pic_order_cnt_val;
	} dpb[NVDEC_HEVC_DPB_ENTRIES];
	u8 scaling_dc_16x16[6];
	u8 scaling_dc_32x32[2];
	u8 scaling_4x4[6][16];
	u8 scaling_8x8[6][64];
	u8 scaling_16x16[6][64];
	u8 scaling_32x32[2][64];
};

#define NVDEC_HEVC_SPS_SCALING_LIST		BIT(0)
#define NVDEC_HEVC_SPS_AMP			BIT(1)
#define NVDEC_HEVC_SPS_SAO			BIT(2)
#define NVDEC_HEVC_SPS_PCM			BIT(3)
#define NVDEC_HEVC_SPS_PCM_LOOP_FILTER_DISABLED	BIT(4)
#define NVDEC_HEVC_SPS_TEMPORAL_MVP		BIT(5)
#define NVDEC_HEVC_SPS_STRONG_INTRA_SMOOTHING	BIT(6)
#define NVDEC_HEVC_SPS_IDR			BIT(7)
#define NVDEC_HEVC_SPS_IRAP			BIT(8)

#define NVDEC_HEVC_PPS_DEPENDENT_SLICE_SEGMENTS	BIT(0)
#define NVDEC_HEVC_PPS_OUTPUT_FLAG_PRESENT	BIT(1)
#define NVDEC_HEVC_PPS_SIGN_DATA_HIDING		BIT(2)
#define NVDEC_HEVC_PPS_CABAC_INIT_PRESENT	BIT(3)
#define NVDEC_HEVC_PPS_CONSTRAINED_INTRA_PRED	BIT(4)
#define NVDEC_HEVC_PPS_TRANSFORM_SKIP		BIT(5)
#define NVDEC_HEVC_PPS_CU_QP_DELTA		BIT(6)
#define NVDEC_HEVC_PPS_SLICE_CHROMA_QP_OFFSETS	BIT(7)
#define NVDEC_HEVC_PPS_WEIGHTED_PRED		BIT(8)
#define NVDEC_HEVC_PPS_WEIGHTED_BIPRED		BIT(9)
#define NVDEC_HEVC_PPS_TRANSQUANT_BYPASS	BIT(10)
#define NVDEC_HEVC_PPS_TILES			BIT(11)
#define NVDEC_HEVC_PPS_ENTROPY_CODING_SYNC	BIT(12)
#define NVDEC_HEVC_PPS_LOOP_FILTER_ACROSS_TILES	BIT(13)
#define NVDEC_HEVC_PPS_LOOP_FILTER_ACROSS_SLICES BIT(14)
#define NVDEC_HEVC_PPS_DEBLOCKING_CONTROL	BIT(15)
#define NVDEC_HEVC_PPS_DEBLOCKING_OVERRIDE	BIT(16)
#define NVDEC_HEVC_PPS_DEBLOCKING_DISABLED	BIT(17)
#define NVDEC_HEVC_PPS_LISTS_MODIFICATION	BIT(18)
#define NVDEC_HEVC_PPS_SLICE_HEADER_EXTENSION	BIT(19)
#define NVDEC_HEVC_PPS_UNIFORM_SPACING		BIT(20)

/* Last, golden and altref; the current picture takes the fourth slot. */
#define NVDEC_VP9_REFS		3
/* The largest superblock, which the coded size is padded up to. */
#define NVDEC_VP9_SB_SIZE	64
#define NVDEC_VP9_MIN_SIZE	129

/* Copied, validated VP9 state. This is not a V4L2 control layout. */
struct nvdec_vp9_request {
	struct nvdec_frame frame;
	u16 width;
	u16 height;
	u32 flags;
	u8 tile_cols_log2;
	u8 tile_rows_log2;
	u8 base_q_idx;
	s8 delta_q_y_dc;
	s8 delta_q_uv_dc;
	s8 delta_q_uv_ac;
	u8 lf_level;
	u8 lf_sharpness;
	s8 lf_ref_deltas[4];
	s8 lf_mode_deltas[2];
	u8 tx_mode;
	u8 reference_mode;
	u8 interpolation_filter;
	u8 sign_bias[NVDEC_VP9_REFS];
	/* A reference may be coded at a size the firmware scales from. */
	u16 ref_width[NVDEC_VP9_REFS];
	u16 ref_height[NVDEC_VP9_REFS];
	u8 seg_feature_enabled[8];
	s16 seg_feature_data[8][4];
	u8 seg_tree_probs[7];
	u8 seg_pred_probs[3];
	struct v4l2_vp9_frame_context probs;
};

#define NVDEC_VP9_REQ_KEY_FRAME		BIT(0)
#define NVDEC_VP9_REQ_PREV_KEY_FRAME	BIT(1)
#define NVDEC_VP9_REQ_ERROR_RESILIENT	BIT(2)
#define NVDEC_VP9_REQ_PREV_SHOW_FRAME	BIT(3)
#define NVDEC_VP9_REQ_INTRA_ONLY	BIT(4)
#define NVDEC_VP9_REQ_LOSSLESS		BIT(5)
#define NVDEC_VP9_REQ_HIGH_PREC_MV	BIT(6)
#define NVDEC_VP9_REQ_SEG_ENABLED	BIT(7)
#define NVDEC_VP9_REQ_SEG_UPDATE_MAP	BIT(8)
#define NVDEC_VP9_REQ_SEG_TEMPORAL	BIT(9)
#define NVDEC_VP9_REQ_SEG_ABS_DELTA	BIT(10)
#define NVDEC_VP9_REQ_LF_DELTA_ENABLED	BIT(11)

/* Forward and backward references; the current picture takes the first slot. */
#define NVDEC_MPEG2_REFS	2

/* Copied, validated MPEG-2 state. This is not a V4L2 control layout. */
struct nvdec_mpeg2_request {
	struct nvdec_frame frame;
	u32 slice_count;
	u8 picture_coding_type;
	u8 picture_structure;
	u8 intra_dc_precision;
	u8 flags;
	u8 f_code[4];
	/* Raster order, which is not the order the V4L2 control carries. */
	u8 quant_intra[64];
	u8 quant_non_intra[64];
};

#define NVDEC_MPEG2_REQ_FRAME_PRED_DCT	BIT(0)
#define NVDEC_MPEG2_REQ_CONCEALMENT_MV	BIT(1)
#define NVDEC_MPEG2_REQ_INTRA_VLC	BIT(2)
#define NVDEC_MPEG2_REQ_ALT_SCAN	BIT(3)
#define NVDEC_MPEG2_REQ_Q_SCALE_TYPE	BIT(4)
#define NVDEC_MPEG2_REQ_TOP_FIELD_FIRST	BIT(5)
#define NVDEC_MPEG2_REQ_SECOND_FIELD	BIT(6)

/* Every member starts with its struct nvdec_frame. */
union nvdec_request {
	struct nvdec_frame frame;
	struct nvdec_h264_request h264;
	struct nvdec_hevc_request hevc;
	struct nvdec_vp9_request vp9;
	struct nvdec_mpeg2_request mpeg2;
};

enum nvdec_codec {
	NVDEC_CODEC_H264,
	NVDEC_CODEC_HEVC,
	NVDEC_CODEC_VP9,
	NVDEC_CODEC_MPEG2,
	NVDEC_NUM_CODECS,
};

/* Called once both jobs are done, from a context that must not sleep. */
typedef void (*nvdec_decode_complete_t)(void *data);

struct nvdec_decode_map *
nvdec_decode_map_create(struct nvdec_decode_context *ctx, struct dma_buf *dmabuf,
			size_t size, enum dma_data_direction direction);
struct nvdec_decode_map *
nvdec_decode_surface_create(struct nvdec_decode_context *ctx, size_t size);
struct nvdec_decode_map *nvdec_decode_map_get(struct nvdec_decode_map *map);
void nvdec_decode_map_put(struct nvdec_decode_map *map);
struct dma_resv *nvdec_decode_map_resv(struct nvdec_decode_map *map);

struct nvdec_decode_context *
nvdec_decode_context_create(struct tegra_drm_client *client,
			    struct host1x_channel *channel, enum nvdec_codec codec);
void nvdec_decode_context_destroy(struct nvdec_decode_context *ctx);
void nvdec_decode_context_reset(struct nvdec_decode_context *ctx);
void nvdec_decode_context_release_surface(struct nvdec_decode_context *ctx,
					  struct nvdec_decode_map *surface);
int nvdec_decode_stage(struct nvdec_decode_context *ctx, const void *bitstream,
		       u32 size);
int nvdec_decode_submit(struct nvdec_decode_context *ctx,
			const union nvdec_request *request,
			struct nvdec_decode_map *surface,
			struct nvdec_decode_map *capture,
			struct nvdec_decode_map * const refs[NVDEC_MAX_REFS],
			nvdec_decode_complete_t complete, void *data);
/* 0, -EBADMSG if decoded with concealed errors, or why it was not decoded. */
int nvdec_decode_finish(struct nvdec_decode_context *ctx);
int nvdec_decode_vp9_counts(struct nvdec_decode_context *ctx,
			    struct v4l2_vp9_frame_symbol_counts *counts);

#endif
