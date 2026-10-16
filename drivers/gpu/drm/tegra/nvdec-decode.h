/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2026 mojyack <mojyack@gmail.com>
 */

#ifndef TEGRA_NVDEC_DECODE_H
#define TEGRA_NVDEC_DECODE_H

#include <linux/bitops.h>
#include <linux/dma-direction.h>

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

/* Every member starts with its struct nvdec_frame. */
union nvdec_request {
	struct nvdec_frame frame;
	struct nvdec_h264_request h264;
};

enum nvdec_codec {
	NVDEC_CODEC_H264,
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

#endif
