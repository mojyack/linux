/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef TEGRA_NVDEC_ENGINE_H
#define TEGRA_NVDEC_ENGINE_H

#include <linux/bitops.h>
#include <linux/dma-direction.h>
#include <linux/of.h>
#include <linux/pm.h>

struct host1x_job;
struct platform_device;
struct tegra_drm_client;
struct tegra_drm_context;
struct device;
struct dma_buf;
struct dma_fence;
struct nvdec_v4l2;

struct nvdec_engine;
struct nvdec_engine_map;
struct nvdec_decode_context;

#define NVDEC_H264_DPB_ENTRIES	16
/* 16 references plus the current picture; one firmware index each. */
#define NVDEC_H264_MAX_PICTURES	17

/* Copied, validated H.264 state. This is not a V4L2 control layout. */
struct nvdec_h264_request {
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
	u8 slice_type;
	u8 nal_ref_idc;
	u8 flags;
	u16 frame_num;
	u16 pic_width_in_mbs;
	u16 frame_height_in_mbs;
	u16 luma_stride;
	u16 chroma_stride;
	/* Visible rectangle VIC detiles out of the coded picture. */
	u16 crop_left;
	u16 crop_top;
	u16 crop_width;
	u16 crop_height;
	u32 output_payload_size;
	u32 slice_count;
	u32 chroma_offset;
	u32 dst_stride;
	u32 dst_chroma_offset;
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
		u8 fields;
		u8 reserved;
		u16 frame_num;
		s32 top_field_order_cnt;
		s32 bottom_field_order_cnt;
	} dpb[NVDEC_H264_DPB_ENTRIES];
};

#define NVDEC_H264_REQ_FRAME_MBS_ONLY	BIT(0)
#define NVDEC_H264_REQ_MBAFF		BIT(1)
#define NVDEC_H264_REQ_SEPARATE_COLOUR	BIT(2)
#define NVDEC_H264_REQ_DELTA_POC_ZERO	BIT(3)
#define NVDEC_H264_REQ_DIRECT_8X8	BIT(4)
#define NVDEC_H264_REQ_IDR		BIT(5)
#define NVDEC_H264_REQ_FIELD		BIT(6)
#define NVDEC_H264_REQ_BOTTOM_FIELD	BIT(7)

#define NVDEC_H264_PPS_ENTROPY_CODING	BIT(0)
#define NVDEC_H264_PPS_PIC_ORDER_PRESENT	BIT(1)
#define NVDEC_H264_PPS_WEIGHTED_PRED	BIT(2)
#define NVDEC_H264_PPS_DEBLOCK		BIT(3)
#define NVDEC_H264_PPS_REDUNDANT	BIT(4)
#define NVDEC_H264_PPS_TRANSFORM_8X8	BIT(5)
#define NVDEC_H264_PPS_CONSTRAINED_INTRA	BIT(6)

#define NVDEC_H264_SLICE_P	0
#define NVDEC_H264_SLICE_B	1
#define NVDEC_H264_SLICE_I	2
#define NVDEC_H264_SLICE_SP	3
#define NVDEC_H264_SLICE_SI	4

typedef void (*nvdec_engine_job_complete_t)(struct host1x_job *job,
					     void *data);
typedef void (*nvdec_engine_complete_t)(void *data, bool error);

#define NVIDIA_TEGRA_210_NVDEC_FIRMWARE "nvidia/tegra210/nvdec.bin"
#define NVIDIA_TEGRA_186_NVDEC_FIRMWARE "nvidia/tegra186/nvdec.bin"
#define NVIDIA_TEGRA_194_NVDEC_FIRMWARE "nvidia/tegra194/nvdec.bin"

struct nvdec_engine *nvdec_engine_probe(struct platform_device *pdev);
int nvdec_engine_register(struct nvdec_engine *engine);
void nvdec_engine_unregister(struct nvdec_engine *engine);

struct tegra_drm_client *
nvdec_engine_drm_client(struct nvdec_engine *engine);
struct nvdec_engine *
nvdec_engine_from_drm_client(struct tegra_drm_client *client);
unsigned int nvdec_engine_version(struct nvdec_engine *engine);
struct device *nvdec_engine_device(struct nvdec_engine *engine);
bool nvdec_engine_ready(struct nvdec_engine *engine);
void nvdec_engine_set_v4l2(struct nvdec_engine *engine,
			   struct nvdec_v4l2 *v4l2);
struct nvdec_v4l2 *nvdec_engine_get_v4l2(struct nvdec_engine *engine);
int nvdec_v4l2_register(struct nvdec_engine *engine);
void nvdec_v4l2_unregister(struct nvdec_engine *engine);

int nvdec_engine_open_channel(struct nvdec_engine *engine,
			      struct tegra_drm_context *context);
void nvdec_engine_close_channel(struct tegra_drm_context *context);
void nvdec_engine_recover(struct nvdec_engine *engine);
int nvdec_engine_submit_job(struct nvdec_engine *engine,
			    struct host1x_job *job,
			    nvdec_engine_job_complete_t complete, void *data);

struct nvdec_engine_map *
nvdec_engine_map_create(struct nvdec_engine *engine, struct dma_buf *dmabuf,
			unsigned long offset, size_t size,
			enum dma_data_direction direction);
struct nvdec_engine_map *
nvdec_engine_surface_create(struct nvdec_engine *engine, size_t size);
struct nvdec_engine_map *nvdec_engine_map_get(struct nvdec_engine_map *map);
void nvdec_engine_map_put(struct nvdec_engine_map *map);
int nvdec_engine_map_wait(struct nvdec_engine_map *map, bool write);
int nvdec_engine_map_add_fence(struct nvdec_engine_map *map,
			       struct dma_fence *fence, bool write);

struct nvdec_decode_context *
nvdec_engine_context_create(struct nvdec_engine *engine);
void nvdec_engine_context_destroy(struct nvdec_decode_context *ctx);
void nvdec_engine_context_release_surface(struct nvdec_decode_context *ctx,
					  struct nvdec_engine_map *surface);
int nvdec_engine_stage_slice(struct nvdec_decode_context *ctx,
			     struct nvdec_engine_map *output,
				  u32 payload_size, bool first,
				  unsigned int max_slices);
void nvdec_engine_discard_slices(struct nvdec_decode_context *ctx);
void nvdec_engine_context_reset(struct nvdec_decode_context *ctx);
int nvdec_engine_h264_submit(struct nvdec_decode_context *ctx,
			     const struct nvdec_h264_request *request,
			     struct nvdec_engine_map *surface,
			     struct nvdec_engine_map *capture,
			     struct nvdec_engine_map * const dpb[NVDEC_H264_DPB_ENTRIES],
			     struct dma_fence **fence,
			     nvdec_engine_complete_t complete, void *data);

extern const struct dev_pm_ops nvdec_engine_pm_ops;
extern const struct of_device_id nvdec_engine_of_match[];

#endif
