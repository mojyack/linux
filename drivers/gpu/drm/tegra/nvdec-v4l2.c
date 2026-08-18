// SPDX-License-Identifier: GPL-2.0-only
/* NVDEC stateless V4L2 request interface. */

#include <linux/err.h>
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-h264.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "nvdec-engine.h"

#define NVDEC_MAX_WIDTH		4096
#define NVDEC_MAX_HEIGHT	4096
#define NVDEC_H264_CODED_SIZE	SZ_4M

/* Smallest picture the engine decodes correctly, and the coded alignment. */
static const struct nvdec_codec_size {
	u32 pixelformat;
	u16 min_width;
	u16 min_height;
	u16 align;
} nvdec_codec_sizes[] = {
	{ V4L2_PIX_FMT_H264_SLICE,   48,  16, 16 },
	{ V4L2_PIX_FMT_HEVC_SLICE,  129, 129, NVDEC_HEVC_CTU_SIZE },
	{ V4L2_PIX_FMT_VP8_FRAME,    48,  16, 16 },
};

static const struct nvdec_codec_size *nvdec_codec_size(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(nvdec_codec_sizes); i++)
		if (nvdec_codec_sizes[i].pixelformat == pixelformat)
			return &nvdec_codec_sizes[i];
	return NULL;
}

struct nvdec_v4l2 {
	struct nvdec_engine *engine;
	struct device *dev;
	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	/* Serializes the vb2 queues and ioctls of every context. */
	struct mutex lock;
};

struct nvdec_v4l2_ctx {
	struct nvdec_v4l2 *nvdec;
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_format coded_fmt;
	struct v4l2_format capture_fmt;
	/* Visible rectangle of the coded picture; VIC detiles only this. */
	struct v4l2_rect crop;
	enum nvdec_codec codec;
	struct nvdec_decode_context *decode;
	struct list_head surfaces;
	struct nvdec_v4l2_job *job;
	/* Picture being assembled; slices of one picture share it. */
	struct nvdec_h264_request picture;
	struct nvdec_hevc_request hevc;
	struct nvdec_vp8_request vp8;
	u32 last_first_mb;
};

struct nvdec_v4l2_surface {
	struct list_head list;
	struct vb2_buffer *vb;
	struct nvdec_engine_map *map;
};

struct nvdec_v4l2_job {
	struct nvdec_v4l2_ctx *ctx;
	struct vb2_v4l2_buffer *src;
	struct vb2_v4l2_buffer *dst;
	struct nvdec_v4l2_surface *surface;
	struct nvdec_engine_map *capture;
	struct nvdec_engine_map *dpb[NVDEC_H264_DPB_ENTRIES];
	struct dma_fence *fence;
	bool capture_new;
	bool aborted;
	bool completed;
};

static const struct v4l2_ctrl_config nvdec_h264_ctrls[] = {
	{ .id = V4L2_CID_STATELESS_H264_DECODE_PARAMS },
	{ .id = V4L2_CID_STATELESS_H264_SPS },
	{ .id = V4L2_CID_STATELESS_H264_PPS },
	{ .id = V4L2_CID_STATELESS_H264_SCALING_MATRIX },
	{ .id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS },
	{ .id = V4L2_CID_STATELESS_H264_SLICE_PARAMS, .dims = { 1 } },
	{
		.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.min = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.max = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.def = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
	}, {
		.id = V4L2_CID_STATELESS_H264_START_CODE,
		.min = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.def = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.min = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
		.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
	},
};

static const struct v4l2_ctrl_config nvdec_hevc_ctrls[] = {
	{ .id = V4L2_CID_STATELESS_HEVC_SPS },
	{ .id = V4L2_CID_STATELESS_HEVC_PPS },
	{ .id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS },
	{ .id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX },
	{
		.id = V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		.min = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.max = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.def = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
	}, {
		.id = V4L2_CID_STATELESS_HEVC_START_CODE,
		.min = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.max = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.def = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
		.min = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.max = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.def = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
		.min = V4L2_MPEG_VIDEO_HEVC_LEVEL_1,
		.max = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1,
	},
};

/* The firmware parses the frame header, so one control is the whole ABI. */
static const struct v4l2_ctrl_config nvdec_vp8_ctrls[] = {
	{ .id = V4L2_CID_STATELESS_VP8_FRAME },
};

static inline struct nvdec_v4l2_ctx *file_to_nvdec_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct nvdec_v4l2_ctx, fh);
}

static void nvdec_release_surface(struct nvdec_v4l2_ctx *ctx,
				  struct vb2_buffer *vb);
static void nvdec_release_surfaces(struct nvdec_v4l2_ctx *ctx);

static void nvdec_reset_coded_fmt(struct nvdec_v4l2_ctx *ctx)
{
	struct v4l2_pix_format_mplane *pix = &ctx->coded_fmt.fmt.pix_mp;
	const struct nvdec_codec_size *size =
		nvdec_codec_size(V4L2_PIX_FMT_H264_SLICE);

	memset(&ctx->coded_fmt, 0, sizeof(ctx->coded_fmt));
	ctx->coded_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	pix->width = size->min_width;
	pix->height = size->min_height;
	pix->pixelformat = V4L2_PIX_FMT_H264_SLICE;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->plane_fmt[0].sizeimage = NVDEC_H264_CODED_SIZE;
}

/* The decoded frame keeps the colorimetry the client set on the OUTPUT queue. */
static void nvdec_fill_capture_fmt(const struct nvdec_v4l2_ctx *ctx,
				   struct v4l2_format *f, u32 width, u32 height)
{
	const struct v4l2_pix_format_mplane *coded = &ctx->coded_fmt.fmt.pix_mp;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	u32 stride = ALIGN(width, 256);	/* VIC pitch-linear requirement */
	u32 aligned_height = ALIGN(height, 16);

	memset(f, 0, sizeof(*f));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	pix->width = width;
	pix->height = height;
	pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = coded->colorspace;
	pix->ycbcr_enc = coded->ycbcr_enc;
	pix->quantization = coded->quantization;
	pix->xfer_func = coded->xfer_func;
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = stride;
	pix->plane_fmt[0].sizeimage = stride * (aligned_height +
					ALIGN(DIV_ROUND_UP(height, 2), 16));
}

static void nvdec_reset_capture_fmt(struct nvdec_v4l2_ctx *ctx)
{
	ctx->crop.left = 0;
	ctx->crop.top = 0;
	ctx->crop.width = ctx->coded_fmt.fmt.pix_mp.width;
	ctx->crop.height = ctx->coded_fmt.fmt.pix_mp.height;
	nvdec_fill_capture_fmt(ctx, &ctx->capture_fmt, ctx->crop.width,
			       ctx->crop.height);
}

/* Kernel-owned block-linear surface NVDEC decodes into and VIC detiles. */
static u32 nvdec_surface_stride(const struct nvdec_v4l2_ctx *ctx)
{
	return ALIGN(ctx->coded_fmt.fmt.pix_mp.width, 64);
}

static u32 nvdec_surface_chroma_offset(const struct nvdec_v4l2_ctx *ctx)
{
	return nvdec_surface_stride(ctx) *
	       ALIGN(ctx->coded_fmt.fmt.pix_mp.height, 32);
}

static size_t nvdec_surface_size(const struct nvdec_v4l2_ctx *ctx)
{
	u32 height = ctx->coded_fmt.fmt.pix_mp.height;

	return nvdec_surface_chroma_offset(ctx) +
	       nvdec_surface_stride(ctx) * ALIGN(DIV_ROUND_UP(height, 2), 16);
}

static int nvdec_try_coded_fmt(struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct nvdec_codec_size *size;

	size = nvdec_codec_size(pix->pixelformat);
	if (!size) {
		pix->pixelformat = V4L2_PIX_FMT_H264_SLICE;
		size = nvdec_codec_size(pix->pixelformat);
	}
	pix->width = clamp_t(u32, pix->width, size->min_width, NVDEC_MAX_WIDTH);
	pix->height = clamp_t(u32, pix->height, size->min_height, NVDEC_MAX_HEIGHT);
	pix->width = ALIGN(pix->width, size->align);
	pix->height = ALIGN(pix->height, size->align);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = 0;
	pix->plane_fmt[0].sizeimage = max_t(u32, pix->plane_fmt[0].sizeimage,
					    NVDEC_H264_CODED_SIZE);
	return 0;
}

static int nvdec_queue_setup(struct vb2_queue *vq, unsigned int *nbufs,
			     unsigned int *num_planes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct nvdec_v4l2_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_format *f = V4L2_TYPE_IS_OUTPUT(vq->type) ?
		&ctx->coded_fmt : &ctx->capture_fmt;
	u32 size = f->fmt.pix_mp.plane_fmt[0].sizeimage;

	if (*num_planes) {
		if (*num_planes != 1 || sizes[0] < size)
			return -EINVAL;
	} else {
		*num_planes = 1;
		sizes[0] = size;
	}

	/* One internal surface per capture buffer, one firmware index each. */
	if (V4L2_TYPE_IS_CAPTURE(vq->type)) {
		unsigned int max = ctx->codec == NVDEC_CODEC_HEVC ?
			NVDEC_HEVC_MAX_PICTURES : NVDEC_H264_MAX_PICTURES;

		if (*nbufs > max)
			*nbufs = max;
	}

	return 0;
}

static int nvdec_buf_prepare(struct vb2_buffer *vb)
{
	struct nvdec_v4l2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_format *f = V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) ?
		&ctx->coded_fmt : &ctx->capture_fmt;
	u32 size = f->fmt.pix_mp.plane_fmt[0].sizeimage;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) &&
	    !vb2_get_plane_payload(vb, 0))
		return -EINVAL;
	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static int nvdec_buf_out_validate(struct vb2_buffer *vb)
{
	to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
	return 0;
}

static void nvdec_buf_queue(struct vb2_buffer *vb)
{
	struct nvdec_v4l2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void nvdec_buf_request_complete(struct vb2_buffer *vb)
{
	struct nvdec_v4l2_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->ctrl_hdl);
}

static void nvdec_stop_streaming(struct vb2_queue *vq)
{
	struct nvdec_v4l2_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *buf;

	while (true) {
		buf = V4L2_TYPE_IS_OUTPUT(vq->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!buf)
			break;
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
	if (V4L2_TYPE_IS_CAPTURE(vq->type)) {
		/* Drop the size-derived scratch so the next stream may differ. */
		nvdec_engine_context_reset(ctx->decode);
		nvdec_release_surfaces(ctx);
	} else {
		nvdec_engine_discard_slices(ctx->decode);
	}
}

static const struct vb2_ops nvdec_qops = {
	.queue_setup = nvdec_queue_setup,
	.buf_prepare = nvdec_buf_prepare,
	.buf_queue = nvdec_buf_queue,
	.buf_out_validate = nvdec_buf_out_validate,
	.buf_request_complete = nvdec_buf_request_complete,
	.stop_streaming = nvdec_stop_streaming,
};

static int nvdec_queue_init(void *priv, struct vb2_queue *src_vq,
			    struct vb2_queue *dst_vq)
{
	struct nvdec_v4l2_ctx *ctx = priv;
	struct nvdec_v4l2 *nvdec = ctx->nvdec;
	int err;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &nvdec_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->subsystem_flags |= VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
	src_vq->supports_requests = true;
	src_vq->requires_requests = true;
	src_vq->lock = &nvdec->lock;
	src_vq->dev = nvdec->dev;
	err = vb2_queue_init(src_vq);
	if (err)
		return err;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &nvdec_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &nvdec->lock;
	dst_vq->dev = nvdec->dev;
	return vb2_queue_init(dst_vq);
}

static const void *nvdec_ctrl_ptr(struct nvdec_v4l2_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	return ctrl ? ctrl->p_cur.p : NULL;
}

static bool nvdec_ctrl_is_new(struct nvdec_v4l2_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	return ctrl && ctrl->is_new;
}

static int nvdec_validate_ref(struct nvdec_v4l2_ctx *ctx,
			      const struct v4l2_h264_dpb_entry *entry)
{
	struct vb2_queue *cap_q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);

	if (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
		return 0;
	if (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
	    (entry->flags & ~(V4L2_H264_DPB_ENTRY_FLAG_VALID |
			      V4L2_H264_DPB_ENTRY_FLAG_ACTIVE |
			      V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)) ||
	    entry->reserved[0] || entry->reserved[1] || entry->reserved[2] ||
	    entry->reserved[3] || entry->reserved[4])
		return -EINVAL;
	if (entry->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD)
		return -EINVAL;
	if (entry->fields != V4L2_H264_FRAME_REF)
		return -EINVAL;
	return vb2_find_buffer(cap_q, entry->reference_ts) ? 0 : -EINVAL;
}

static int nvdec_validate_reflist(const struct v4l2_h264_reference *refs,
				  unsigned int count,
				  const struct v4l2_ctrl_h264_decode_params *dec)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		/* A list position no picture can fill is left unset. */
		if (!refs[i].fields)
			continue;
		if (refs[i].fields != V4L2_H264_FRAME_REF ||
		    refs[i].index >= V4L2_H264_NUM_DPB_ENTRIES ||
		    !(dec->dpb[refs[i].index].flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			return -EINVAL;
	}
	return 0;
}

static int nvdec_validate_h264_request(struct nvdec_v4l2_ctx *ctx, bool first)
{
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_decode_params *dec;
	const struct v4l2_ctrl_h264_slice_params *slice;
	const char *why;
	unsigned int i, l0, l1;

	if (!nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_H264_SPS) ||
	    !nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_H264_PPS) ||
	    !nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS) ||
	    !nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_H264_SLICE_PARAMS)) {
		why = "a required control is missing from the request";
		goto reject;
	}

	sps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SPS);
	pps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_PPS);
	dec = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	slice = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SLICE_PARAMS);
	if (!sps || !pps || !dec || !slice) {
		why = "control handler lookup failed";
		goto reject;
	}
	if (sps->profile_idc != 66 && sps->profile_idc != 77 &&
	    sps->profile_idc != 100) {
		why = "profile";
		goto reject;
	}
	if (sps->level_idc < 10 || sps->level_idc > 51 ||
	    sps->chroma_format_idc != 1 || sps->bit_depth_luma_minus8 ||
	    sps->bit_depth_chroma_minus8 ||
	    !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) ||
	    (sps->flags & (V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE |
			   V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD |
			   V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS))) {
		why = "sps";
		goto reject;
	}
	if (pps->num_slice_groups_minus1 ||
	    pps->seq_parameter_set_id != sps->seq_parameter_set_id ||
	    (pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT &&
	     !nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_H264_SCALING_MATRIX)) ||
	    (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps, slice) &&
	     !nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_H264_PRED_WEIGHTS))) {
		why = "pps";
		goto reject;
	}
	if (dec->reserved ||
	    (dec->flags & ~(V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC |
			    V4L2_H264_DECODE_PARAM_FLAG_PFRAME |
			    V4L2_H264_DECODE_PARAM_FLAG_BFRAME)) ||
	    (slice->slice_type != V4L2_H264_SLICE_TYPE_I &&
	     (dec->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC))) {
		why = "decode params";
		goto reject;
	}
	if (slice->colour_plane_id || slice->redundant_pic_cnt ||
	    slice->slice_type > V4L2_H264_SLICE_TYPE_I || slice->reserved) {
		why = "slice params";
		goto reject;
	}

	/* Slices of one picture arrive in order and must cover it exactly once. */
	if (!first && slice->first_mb_in_slice <= ctx->last_first_mb) {
		why = "slice order";
		goto reject;
	}
	if (slice->first_mb_in_slice >=
	    (u32)(sps->pic_width_in_mbs_minus1 + 1) *
	    (sps->pic_height_in_map_units_minus1 + 1)) {
		why = "first_mb_in_slice out of range";
		goto reject;
	}

	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		if (nvdec_validate_ref(ctx, &dec->dpb[i])) {
			dev_dbg(ctx->nvdec->dev,
				"h264 reject: dpb entry %u (flags 0x%x fields %u ts %llu)\n",
				i, dec->dpb[i].flags, dec->dpb[i].fields,
				dec->dpb[i].reference_ts);
			return -EINVAL;
		}
	}
	l0 = slice->num_ref_idx_l0_active_minus1 + 1;
	l1 = slice->num_ref_idx_l1_active_minus1 + 1;
	if ((slice->slice_type == V4L2_H264_SLICE_TYPE_P &&
	     nvdec_validate_reflist(slice->ref_pic_list0, l0, dec)) ||
	    (slice->slice_type == V4L2_H264_SLICE_TYPE_B &&
	     (nvdec_validate_reflist(slice->ref_pic_list0, l0, dec) ||
	      nvdec_validate_reflist(slice->ref_pic_list1, l1, dec)))) {
		why = "reference list";
		goto reject;
	}

	return 0;

reject:
	dev_dbg(ctx->nvdec->dev, "h264 reject: %s\n", why);
	return -EINVAL;
}

static struct dma_buf *nvdec_plane_dmabuf(struct vb2_buffer *vb)
{
	struct dma_buf *dmabuf;

	dmabuf = vb->planes[0].dbuf;
	if (dmabuf) {
		get_dma_buf(dmabuf);
		return dmabuf;
	}

	return vb->vb2_queue->mem_ops->get_dmabuf(vb, vb->planes[0].mem_priv, 0);
}

static struct nvdec_v4l2_surface *
nvdec_find_surface(struct nvdec_v4l2_ctx *ctx, struct vb2_buffer *vb)
{
	struct nvdec_v4l2_surface *surface;

	list_for_each_entry(surface, &ctx->surfaces, list) {
		if (surface->vb == vb)
			return surface;
	}

	return NULL;
}

static void nvdec_release_surface(struct nvdec_v4l2_ctx *ctx,
				  struct vb2_buffer *vb)
{
	struct nvdec_v4l2_surface *surface = nvdec_find_surface(ctx, vb);

	if (!surface)
		return;
	nvdec_engine_context_release_surface(ctx->decode, surface->map);
	list_del(&surface->list);
	nvdec_engine_map_put(surface->map);
	kfree(surface);
}

static void nvdec_release_surfaces(struct nvdec_v4l2_ctx *ctx)
{
	struct nvdec_v4l2_surface *surface, *tmp;

	list_for_each_entry_safe(surface, tmp, &ctx->surfaces, list)
		nvdec_release_surface(ctx, surface->vb);
}

static int nvdec_capture_surface(struct nvdec_v4l2_ctx *ctx,
				 struct vb2_buffer *vb,
				 struct nvdec_v4l2_surface **result)
{
	struct nvdec_v4l2_surface *surface;

	surface = nvdec_find_surface(ctx, vb);
	if (surface) {
		*result = surface;
		return 0;
	}

	surface = kzalloc_obj(*surface);
	if (!surface)
		return -ENOMEM;
	surface->map = nvdec_engine_surface_create(ctx->nvdec->engine,
						   nvdec_surface_size(ctx));
	if (IS_ERR(surface->map)) {
		int err = PTR_ERR(surface->map);

		kfree(surface);
		return err;
	}
	surface->vb = vb;
	list_add_tail(&surface->list, &ctx->surfaces);
	*result = surface;
	return 0;
}

static int nvdec_map_buffer(struct nvdec_v4l2_ctx *ctx, struct vb2_buffer *vb,
			    enum dma_data_direction direction,
			    unsigned long offset,
			    struct nvdec_engine_map **result)
{
	struct dma_buf *dmabuf;
	struct nvdec_engine_map *map;

	if (offset >= vb2_plane_size(vb, 0))
		return -EINVAL;
	dmabuf = nvdec_plane_dmabuf(vb);
	if (!dmabuf)
		return -ENOMEM;
	map = nvdec_engine_map_create(ctx->nvdec->engine, dmabuf, offset,
				      vb2_plane_size(vb, 0) - offset, direction);
	dma_buf_put(dmabuf);
	if (IS_ERR(map))
		return PTR_ERR(map);
	*result = map;
	return 0;
}

static int nvdec_set_codec(struct nvdec_v4l2_ctx *ctx, enum nvdec_codec codec)
{
	struct nvdec_decode_context *decode;

	if (ctx->codec == codec)
		return 0;

	decode = nvdec_engine_context_create(ctx->nvdec->engine, codec);
	if (IS_ERR(decode))
		return PTR_ERR(decode);

	nvdec_release_surfaces(ctx);
	nvdec_engine_context_destroy(ctx->decode);
	ctx->decode = decode;
	ctx->codec = codec;
	return 0;
}

static int nvdec_snapshot_h264_request(struct nvdec_v4l2_ctx *ctx, bool first)
{
	struct nvdec_h264_request *request = &ctx->picture;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_decode_params *dec;
	const struct v4l2_ctrl_h264_slice_params *slice;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling;
	const struct v4l2_pix_format_mplane *pix = &ctx->capture_fmt.fmt.pix_mp;
	unsigned int i;

	if (nvdec_validate_h264_request(ctx, first))
		return -EINVAL;
	sps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SPS);
	pps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_PPS);
	dec = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	slice = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SLICE_PARAMS);
	scaling = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	memset(request, 0, sizeof(*request));
	request->profile_idc = sps->profile_idc;
	request->level_idc = sps->level_idc;
	request->chroma_format_idc = sps->chroma_format_idc;
	request->bit_depth_luma_minus8 = sps->bit_depth_luma_minus8;
	request->bit_depth_chroma_minus8 = sps->bit_depth_chroma_minus8;
	request->log2_max_frame_num_minus4 = sps->log2_max_frame_num_minus4;
	request->pic_order_cnt_type = sps->pic_order_cnt_type;
	request->log2_max_pic_order_cnt_lsb_minus4 =
		sps->log2_max_pic_order_cnt_lsb_minus4;
	request->max_num_ref_frames = sps->max_num_ref_frames;
	request->num_ref_idx_l0_active_minus1 = slice->num_ref_idx_l0_active_minus1;
	request->num_ref_idx_l1_active_minus1 = slice->num_ref_idx_l1_active_minus1;
	request->slice_type = slice->slice_type;
	request->nal_ref_idc = dec->nal_ref_idc;
	request->frame_num = dec->frame_num;
	request->pic_width_in_mbs = sps->pic_width_in_mbs_minus1 + 1;
	request->frame_height_in_mbs = sps->pic_height_in_map_units_minus1 + 1;
	request->luma_stride = nvdec_surface_stride(ctx);
	request->chroma_stride = request->luma_stride;
	request->chroma_offset = nvdec_surface_chroma_offset(ctx);
	request->crop_left = ctx->crop.left;
	request->crop_top = ctx->crop.top;
	request->crop_width = ctx->crop.width;
	request->crop_height = ctx->crop.height;
	request->dst_stride = pix->plane_fmt[0].bytesperline;
	request->dst_chroma_offset = request->dst_stride * pix->height;
	request->top_field_order_cnt = dec->top_field_order_cnt;
	request->bottom_field_order_cnt = dec->bottom_field_order_cnt;
	request->pic_init_qp_minus26 = pps->pic_init_qp_minus26;
	request->chroma_qp_index_offset = pps->chroma_qp_index_offset;
	request->second_chroma_qp_index_offset = pps->second_chroma_qp_index_offset;
	request->weighted_bipred_idc = pps->weighted_bipred_idc;
	request->num_slice_groups_minus1 = pps->num_slice_groups_minus1;
	if (sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)
		request->flags |= NVDEC_H264_REQ_FRAME_MBS_ONLY;
	if (sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)
		request->flags |= NVDEC_H264_REQ_DELTA_POC_ZERO;
	if (sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE)
		request->flags |= NVDEC_H264_REQ_DIRECT_8X8;
	if (dec->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)
		request->flags |= NVDEC_H264_REQ_IDR;
	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		request->pps_flags |= NVDEC_H264_PPS_ENTROPY_CODING;
	if (pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT)
		request->pps_flags |= NVDEC_H264_PPS_PIC_ORDER_PRESENT;
	if (pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED)
		request->pps_flags |= NVDEC_H264_PPS_WEIGHTED_PRED;
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT)
		request->pps_flags |= NVDEC_H264_PPS_DEBLOCK;
	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)
		request->pps_flags |= NVDEC_H264_PPS_REDUNDANT;
	if (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE)
		request->pps_flags |= NVDEC_H264_PPS_TRANSFORM_8X8;
	if (pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED)
		request->pps_flags |= NVDEC_H264_PPS_CONSTRAINED_INTRA;
	if (scaling) {
		memcpy(request->scaling_4x4, scaling->scaling_list_4x4,
		       sizeof(request->scaling_4x4));
		/* 8x8 lists are ordered intra Y, inter Y, ..., unlike the 4x4 ones. */
		memcpy(request->scaling_8x8[0], scaling->scaling_list_8x8[0], 64);
		memcpy(request->scaling_8x8[1], scaling->scaling_list_8x8[1], 64);
	} else {
		memset(request->scaling_4x4, 16, sizeof(request->scaling_4x4));
		memset(request->scaling_8x8, 16, sizeof(request->scaling_8x8));
	}
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		const struct v4l2_h264_dpb_entry *dpb = &dec->dpb[i];

		if (!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		request->dpb[i].valid = 1;
		request->dpb[i].long_term =
			!!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM);
		request->dpb[i].fields = dpb->fields;
		request->dpb[i].frame_num = dpb->frame_num;
		request->dpb[i].top_field_order_cnt = dpb->top_field_order_cnt;
		request->dpb[i].bottom_field_order_cnt = dpb->bottom_field_order_cnt;
	}
	if (request->pic_width_in_mbs * 16 != ctx->coded_fmt.fmt.pix_mp.width ||
	    request->frame_height_in_mbs * 16 != ctx->coded_fmt.fmt.pix_mp.height)
		return -EINVAL;
	ctx->last_first_mb = slice->first_mb_in_slice;
	return 0;
}

static int nvdec_validate_hevc_request(struct nvdec_v4l2_ctx *ctx)
{
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	const struct v4l2_ctrl_hevc_decode_params *dec;
	struct vb2_queue *cap_q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	const struct v4l2_pix_format_mplane *coded = &ctx->coded_fmt.fmt.pix_mp;
	unsigned int i, log2_ctb, columns, rows;
	const char *why;

	if (!nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_HEVC_SPS) ||
	    !nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_HEVC_PPS) ||
	    !nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS)) {
		why = "a required control is missing from the request";
		goto reject;
	}

	sps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_SPS);
	pps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_PPS);
	dec = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);

	if (sps->bit_depth_luma_minus8 || sps->bit_depth_chroma_minus8 ||
	    sps->chroma_format_idc != 1 ||
	    (sps->flags & V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE)) {
		why = "not 8-bit 4:2:0 with a shared colour plane";
		goto reject;
	}

	if (ALIGN(sps->pic_width_in_luma_samples, NVDEC_HEVC_CTU_SIZE) !=
	    coded->width ||
	    ALIGN(sps->pic_height_in_luma_samples, NVDEC_HEVC_CTU_SIZE) !=
	    coded->height) {
		why = "SPS dimensions do not match the negotiated coded format";
		goto reject;
	}

	log2_ctb = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		   sps->log2_diff_max_min_luma_coding_block_size;
	if (log2_ctb < 4 || log2_ctb > 6) {
		why = "unsupported coding tree unit size";
		goto reject;
	}

	columns = (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) ?
		  pps->num_tile_columns_minus1 + 1 : 1;
	rows = (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) ?
	       pps->num_tile_rows_minus1 + 1 : 1;
	if (columns > ARRAY_SIZE(pps->column_width_minus1) ||
	    rows > ARRAY_SIZE(pps->row_height_minus1)) {
		why = "too many tiles";
		goto reject;
	}

	if (dec->num_active_dpb_entries > NVDEC_HEVC_DPB_ENTRIES ||
	    dec->num_poc_st_curr_before > NVDEC_HEVC_DPB_ENTRIES ||
	    dec->num_poc_st_curr_after > NVDEC_HEVC_DPB_ENTRIES ||
	    dec->num_poc_lt_curr > NVDEC_HEVC_DPB_ENTRIES) {
		why = "decode parameters name too many references";
		goto reject;
	}

	for (i = 0; i < dec->num_active_dpb_entries; i++) {
		if (dec->dpb[i].field_pic ||
		    (dec->dpb[i].flags & ~V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE) ||
		    dec->dpb[i].reserved) {
			why = "unsupported DPB entry";
			goto reject;
		}
		if (!vb2_find_buffer(cap_q, dec->dpb[i].timestamp)) {
			why = "a DPB entry names no capture buffer";
			goto reject;
		}
	}

	for (i = 0; i < dec->num_poc_st_curr_before; i++)
		if (dec->poc_st_curr_before[i] >= dec->num_active_dpb_entries)
			goto bad_rps;
	for (i = 0; i < dec->num_poc_st_curr_after; i++)
		if (dec->poc_st_curr_after[i] >= dec->num_active_dpb_entries)
			goto bad_rps;
	for (i = 0; i < dec->num_poc_lt_curr; i++)
		if (dec->poc_lt_curr[i] >= dec->num_active_dpb_entries)
			goto bad_rps;

	return 0;

bad_rps:
	why = "a reference set names an out-of-range DPB entry";
reject:
	dev_dbg(ctx->nvdec->dev, "hevc reject: %s\n", why);
	return -EINVAL;
}

static int nvdec_snapshot_hevc_request(struct nvdec_v4l2_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->capture_fmt.fmt.pix_mp;
	const struct v4l2_pix_format_mplane *coded = &ctx->coded_fmt.fmt.pix_mp;
	struct nvdec_hevc_request *request = &ctx->hevc;
	const struct v4l2_ctrl_hevc_scaling_matrix *scaling;
	const struct v4l2_ctrl_hevc_decode_params *dec;
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	unsigned int i, log2_ctb, ctb_size;
	bool tiles;

	if (nvdec_validate_hevc_request(ctx))
		return -EINVAL;

	sps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_SPS);
	pps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_PPS);
	dec = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);
	scaling = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_SCALING_MATRIX);

	memset(request, 0, sizeof(*request));
	request->pic_width_in_luma_samples = sps->pic_width_in_luma_samples;
	request->pic_height_in_luma_samples = sps->pic_height_in_luma_samples;
	request->coded_width = coded->width;
	request->coded_height = coded->height;
	request->crop_left = ctx->crop.left;
	request->crop_top = ctx->crop.top;
	request->crop_width = ctx->crop.width;
	request->crop_height = ctx->crop.height;
	request->luma_stride = nvdec_surface_stride(ctx);
	request->chroma_offset = nvdec_surface_chroma_offset(ctx);
	request->dst_stride = pix->plane_fmt[0].bytesperline;
	request->dst_chroma_offset = request->dst_stride * pix->height;
	request->pic_order_cnt_val = dec->pic_order_cnt_val;
	request->bit_depth = 8;

	log2_ctb = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		   sps->log2_diff_max_min_luma_coding_block_size;
	ctb_size = 1 << log2_ctb;
	request->ctb_width = DIV_ROUND_UP(sps->pic_width_in_luma_samples, ctb_size);
	request->ctb_height = DIV_ROUND_UP(sps->pic_height_in_luma_samples, ctb_size);

	request->log2_min_luma_coding_block_size =
		sps->log2_min_luma_coding_block_size_minus3 + 3;
	request->log2_max_luma_coding_block_size = log2_ctb;
	request->log2_min_transform_block_size =
		sps->log2_min_luma_transform_block_size_minus2 + 2;
	request->log2_max_transform_block_size =
		request->log2_min_transform_block_size +
		sps->log2_diff_max_min_luma_transform_block_size;
	request->max_transform_hierarchy_depth_inter =
		sps->max_transform_hierarchy_depth_inter;
	request->max_transform_hierarchy_depth_intra =
		sps->max_transform_hierarchy_depth_intra;

	if (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) {
		request->sps_flags |= NVDEC_HEVC_SPS_PCM;
		request->pcm_sample_bit_depth_luma =
			sps->pcm_sample_bit_depth_luma_minus1 + 1;
		request->pcm_sample_bit_depth_chroma =
			sps->pcm_sample_bit_depth_chroma_minus1 + 1;
		request->log2_min_pcm_luma_coding_block_size =
			sps->log2_min_pcm_luma_coding_block_size_minus3 + 3;
		request->log2_max_pcm_luma_coding_block_size =
			request->log2_min_pcm_luma_coding_block_size +
			sps->log2_diff_max_min_pcm_luma_coding_block_size;
	}
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED)
		request->sps_flags |= NVDEC_HEVC_SPS_SCALING_LIST;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_AMP_ENABLED)
		request->sps_flags |= NVDEC_HEVC_SPS_AMP;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET)
		request->sps_flags |= NVDEC_HEVC_SPS_SAO;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED)
		request->sps_flags |= NVDEC_HEVC_SPS_PCM_LOOP_FILTER_DISABLED;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED)
		request->sps_flags |= NVDEC_HEVC_SPS_TEMPORAL_MVP;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED)
		request->sps_flags |= NVDEC_HEVC_SPS_STRONG_INTRA_SMOOTHING;
	if (dec->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC)
		request->sps_flags |= NVDEC_HEVC_SPS_IDR;
	if (dec->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC)
		request->sps_flags |= NVDEC_HEVC_SPS_IRAP;

	request->num_extra_slice_header_bits = pps->num_extra_slice_header_bits;
	request->num_ref_idx_l0_default_active =
		pps->num_ref_idx_l0_default_active_minus1 + 1;
	request->num_ref_idx_l1_default_active =
		pps->num_ref_idx_l1_default_active_minus1 + 1;
	request->init_qp = pps->init_qp_minus26 + 26;
	request->diff_cu_qp_delta_depth = pps->diff_cu_qp_delta_depth;
	request->pps_cb_qp_offset = pps->pps_cb_qp_offset;
	request->pps_cr_qp_offset = pps->pps_cr_qp_offset;
	request->pps_beta_offset = pps->pps_beta_offset_div2 * 2;
	request->pps_tc_offset = pps->pps_tc_offset_div2 * 2;
	request->log2_parallel_merge_level =
		pps->log2_parallel_merge_level_minus2 + 2;

	if (pps->flags & V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_DEPENDENT_SLICE_SEGMENTS;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT)
		request->pps_flags |= NVDEC_HEVC_PPS_OUTPUT_FLAG_PRESENT;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_SIGN_DATA_HIDING;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT)
		request->pps_flags |= NVDEC_HEVC_PPS_CABAC_INIT_PRESENT;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED)
		request->pps_flags |= NVDEC_HEVC_PPS_CONSTRAINED_INTRA_PRED;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_TRANSFORM_SKIP;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_CU_QP_DELTA;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT)
		request->pps_flags |= NVDEC_HEVC_PPS_SLICE_CHROMA_QP_OFFSETS;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED)
		request->pps_flags |= NVDEC_HEVC_PPS_WEIGHTED_PRED;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED)
		request->pps_flags |= NVDEC_HEVC_PPS_WEIGHTED_BIPRED;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_TRANSQUANT_BYPASS;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_ENTROPY_CODING_SYNC;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_LOOP_FILTER_ACROSS_TILES;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_LOOP_FILTER_ACROSS_SLICES;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT)
		request->pps_flags |= NVDEC_HEVC_PPS_DEBLOCKING_CONTROL;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED)
		request->pps_flags |= NVDEC_HEVC_PPS_DEBLOCKING_OVERRIDE;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER)
		request->pps_flags |= NVDEC_HEVC_PPS_DEBLOCKING_DISABLED;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT)
		request->pps_flags |= NVDEC_HEVC_PPS_LISTS_MODIFICATION;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT)
		request->pps_flags |= NVDEC_HEVC_PPS_SLICE_HEADER_EXTENSION;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)
		request->pps_flags |= NVDEC_HEVC_PPS_UNIFORM_SPACING;

	tiles = pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED;
	if (tiles) {
		request->pps_flags |= NVDEC_HEVC_PPS_TILES;
		request->num_tile_columns = pps->num_tile_columns_minus1 + 1;
		request->num_tile_rows = pps->num_tile_rows_minus1 + 1;
		for (i = 0; i < request->num_tile_columns; i++)
			request->column_width[i] = pps->column_width_minus1[i] + 1;
		for (i = 0; i < request->num_tile_rows; i++)
			request->row_height[i] = pps->row_height_minus1[i] + 1;
	} else {
		request->num_tile_columns = 1;
		request->num_tile_rows = 1;
		request->column_width[0] = request->ctb_width;
		request->row_height[0] = request->ctb_height;
	}

	/* Slice-header bits the driver resolved and the firmware skips. */
	request->sw_hdr_skip_length =
		!!(pps->flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT);
	if (!(dec->flags & V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC))
		request->sw_hdr_skip_length +=
			sps->log2_max_pic_order_cnt_lsb_minus4 + 4 + 1 +
			dec->short_term_ref_pic_set_size +
			dec->long_term_ref_pic_set_size;

	request->num_active_dpb_entries = dec->num_active_dpb_entries;
	request->num_poc_st_curr_before = dec->num_poc_st_curr_before;
	request->num_poc_st_curr_after = dec->num_poc_st_curr_after;
	request->num_poc_lt_curr = dec->num_poc_lt_curr;
	request->num_ref_frames = dec->num_poc_st_curr_before +
				  dec->num_poc_st_curr_after +
				  dec->num_poc_lt_curr;
	memcpy(request->poc_st_curr_before, dec->poc_st_curr_before,
	       sizeof(request->poc_st_curr_before));
	memcpy(request->poc_st_curr_after, dec->poc_st_curr_after,
	       sizeof(request->poc_st_curr_after));
	memcpy(request->poc_lt_curr, dec->poc_lt_curr,
	       sizeof(request->poc_lt_curr));
	for (i = 0; i < dec->num_active_dpb_entries; i++) {
		request->dpb[i].valid = 1;
		request->dpb[i].long_term =
			!!(dec->dpb[i].flags &
			   V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE);
		request->dpb[i].pic_order_cnt_val = dec->dpb[i].pic_order_cnt_val;
	}

	if (scaling) {
		memcpy(request->scaling_dc_16x16, scaling->scaling_list_dc_coef_16x16,
		       sizeof(request->scaling_dc_16x16));
		memcpy(request->scaling_dc_32x32, scaling->scaling_list_dc_coef_32x32,
		       sizeof(request->scaling_dc_32x32));
		memcpy(request->scaling_4x4, scaling->scaling_list_4x4,
		       sizeof(request->scaling_4x4));
		memcpy(request->scaling_8x8, scaling->scaling_list_8x8,
		       sizeof(request->scaling_8x8));
		memcpy(request->scaling_16x16, scaling->scaling_list_16x16,
		       sizeof(request->scaling_16x16));
		memcpy(request->scaling_32x32, scaling->scaling_list_32x32,
		       sizeof(request->scaling_32x32));
	}

	return 0;
}

static int nvdec_validate_vp8_request(struct nvdec_v4l2_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *coded = &ctx->coded_fmt.fmt.pix_mp;
	const struct v4l2_ctrl_vp8_frame *frame;
	const char *why;

	if (!nvdec_ctrl_is_new(ctx, V4L2_CID_STATELESS_VP8_FRAME)) {
		why = "the frame control is missing from the request";
		goto reject;
	}

	frame = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_VP8_FRAME);
	if (!frame) {
		why = "control handler lookup failed";
		goto reject;
	}

	if (frame->version > 3) {
		why = "unsupported bitstream version";
		goto reject;
	}

	/* The decoded picture is the coded size; there is no upscaler. */
	if (frame->horizontal_scale || frame->vertical_scale) {
		why = "upscaling is not supported";
		goto reject;
	}

	if (ALIGN(frame->width, 16) != coded->width ||
	    ALIGN(frame->height, 16) != coded->height) {
		why = "frame dimensions do not match the negotiated coded format";
		goto reject;
	}

	if (!frame->first_part_size) {
		why = "empty first partition";
		goto reject;
	}

	return 0;

reject:
	dev_dbg(ctx->nvdec->dev, "vp8 reject: %s\n", why);
	return -EINVAL;
}

/* The firmware re-parses the header, so little of the control is used. */
static int nvdec_snapshot_vp8_request(struct nvdec_v4l2_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->capture_fmt.fmt.pix_mp;
	const struct v4l2_pix_format_mplane *coded = &ctx->coded_fmt.fmt.pix_mp;
	struct nvdec_vp8_request *request = &ctx->vp8;
	const struct v4l2_ctrl_vp8_frame *frame;

	if (nvdec_validate_vp8_request(ctx))
		return -EINVAL;

	frame = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_VP8_FRAME);

	memset(request, 0, sizeof(*request));
	request->coded_width = coded->width;
	request->coded_height = coded->height;
	request->crop_left = ctx->crop.left;
	request->crop_top = ctx->crop.top;
	request->crop_width = ctx->crop.width;
	request->crop_height = ctx->crop.height;
	request->luma_stride = nvdec_surface_stride(ctx);
	request->chroma_offset = nvdec_surface_chroma_offset(ctx);
	request->dst_stride = pix->plane_fmt[0].bytesperline;
	request->dst_chroma_offset = request->dst_stride * pix->height;
	request->version = frame->version;
	request->first_part_size = frame->first_part_size;
	if (frame->flags & V4L2_VP8_FRAME_FLAG_KEY_FRAME)
		request->flags |= NVDEC_VP8_REQ_KEY_FRAME;
	if ((frame->segment.flags & V4L2_VP8_SEGMENT_FLAG_ENABLED) &&
	    (frame->segment.flags & V4L2_VP8_SEGMENT_FLAG_UPDATE_FEATURE_DATA))
		request->flags |= NVDEC_VP8_REQ_SEGMENT_UPDATE;

	return 0;
}

static void nvdec_job_cleanup_maps(struct nvdec_v4l2_job *job)
{
	unsigned int i;

	nvdec_engine_map_put(job->capture);
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++)
		nvdec_engine_map_put(job->dpb[i]);
	dma_fence_put(job->fence);
}

static void nvdec_job_complete(void *data, bool error)
{
	struct nvdec_v4l2_job *job = data;
	struct nvdec_v4l2_ctx *ctx = job->ctx;

	mutex_lock(&ctx->nvdec->lock);
	if (ctx->job != job || job->completed) {
		mutex_unlock(&ctx->nvdec->lock);
		return;
	}
	job->completed = true;
	ctx->job = NULL;
	v4l2_m2m_buf_done_and_job_finish(ctx->nvdec->m2m_dev, ctx->fh.m2m_ctx,
					 error || job->aborted ? VB2_BUF_STATE_ERROR :
					 VB2_BUF_STATE_DONE);
	nvdec_job_cleanup_maps(job);
	mutex_unlock(&ctx->nvdec->lock);
	kfree(job);
}

/* A slice that holds the capture buffer is staged; the last one submits. */
static int nvdec_stage_slice(struct nvdec_v4l2_ctx *ctx,
			     struct vb2_v4l2_buffer *src, bool first)
{
	/* Only H.264 stages one slice at a time; the others a whole picture. */
	unsigned int max_slices = ctx->codec != NVDEC_CODEC_H264 ? 1 :
		ctx->picture.pic_width_in_mbs * ctx->picture.frame_height_in_mbs;
	u32 payload = vb2_get_plane_payload(&src->vb2_buf, 0);
	struct nvdec_engine_map *output;
	unsigned long offset = 0;
	int err;

	/* VP8 reaches the firmware without its uncompressed data chunk. */
	if (ctx->codec == NVDEC_CODEC_VP8) {
		offset = ctx->vp8.flags & NVDEC_VP8_REQ_KEY_FRAME ? 10 : 3;
		if (payload <= offset)
			return -EINVAL;
		payload -= offset;
	}

	err = nvdec_map_buffer(ctx, &src->vb2_buf, DMA_TO_DEVICE, offset, &output);
	if (err)
		return err;
	err = nvdec_engine_map_wait(output, false);
	if (!err)
		err = nvdec_engine_stage_slice(ctx->decode, output, payload,
					       first, max_slices);
	nvdec_engine_map_put(output);
	return err;
}

/* Each DPB entry names a capture buffer, and through it a pool surface. */
static int nvdec_pin_reference(struct nvdec_v4l2_ctx *ctx,
			       struct nvdec_v4l2_job *job, unsigned int slot,
			       u64 timestamp)
{
	struct vb2_queue *cap_q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct nvdec_v4l2_surface *surface;

	surface = nvdec_find_surface(ctx, vb2_find_buffer(cap_q, timestamp));
	if (!surface)
		return -EINVAL;

	job->dpb[slot] = nvdec_engine_map_get(surface->map);
	return 0;
}

static int nvdec_resolve_dpb(struct nvdec_v4l2_ctx *ctx,
			     struct nvdec_v4l2_job *job)
{
	unsigned int i;
	int err;

	if (ctx->codec == NVDEC_CODEC_HEVC) {
		const struct v4l2_ctrl_hevc_decode_params *dec =
			nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);

		for (i = 0; i < ctx->hevc.num_active_dpb_entries; i++) {
			err = nvdec_pin_reference(ctx, job, i,
						  dec->dpb[i].timestamp);
			if (err)
				return err;
		}

		return 0;
	}

	/* VP8 slots are roles, not DPB entries: golden, altref, last. */
	if (ctx->codec == NVDEC_CODEC_VP8) {
		const struct v4l2_ctrl_vp8_frame *frame =
			nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_VP8_FRAME);
		const u64 timestamps[NVDEC_VP8_REFS] = {
			frame->golden_frame_ts, frame->alt_frame_ts,
			frame->last_frame_ts,
		};

		/* A reference the client does not name is the current picture. */
		for (i = 0; i < NVDEC_VP8_REFS; i++)
			nvdec_pin_reference(ctx, job, i, timestamps[i]);

		return 0;
	}

	{
		const struct v4l2_ctrl_h264_decode_params *dec =
			nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);

		for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
			if (!ctx->picture.dpb[i].valid)
				continue;
			err = nvdec_pin_reference(ctx, job, i,
						  dec->dpb[i].reference_ts);
			if (err)
				return err;
		}
	}

	return 0;
}

static void nvdec_device_run(void *priv)
{
	struct nvdec_v4l2_ctx *ctx = priv;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct media_request *req = src ? src->vb2_buf.req_obj.req : NULL;
	struct nvdec_v4l2_job *job;
	const struct v4l2_ctrl_h264_slice_params *slice;
	unsigned int i;
	bool controls_set_up = false;
	bool first;
	int err;

	err = req ? v4l2_ctrl_request_setup(req, &ctx->ctrl_hdl) : -EINVAL;
	controls_set_up = !err;
	if (!err)
		err = src && dst ? 0 : -EINVAL;
	if (err)
		goto fail;
	if (ctx->codec == NVDEC_CODEC_HEVC) {
		/* Frame-based: one request carries the whole picture. */
		first = true;
		err = nvdec_snapshot_hevc_request(ctx);
	} else if (ctx->codec == NVDEC_CODEC_VP8) {
		first = true;
		err = nvdec_snapshot_vp8_request(ctx);
	} else {
		/* A picture starts at macroblock zero; m2m's new_frame is unreliable. */
		slice = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SLICE_PARAMS);
		first = !slice || !slice->first_mb_in_slice;
		err = nvdec_snapshot_h264_request(ctx, first);
	}
	if (err)
		goto fail;
	err = nvdec_stage_slice(ctx, src, first);
	if (err)
		goto fail;
	v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	controls_set_up = false;
	v4l2_m2m_buf_copy_metadata(src, dst);

	if (src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF) {
		v4l2_m2m_buf_done_and_job_finish(ctx->nvdec->m2m_dev,
						 ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_DONE);
		return;
	}

	job = kzalloc_obj(*job);
	if (!job) {
		err = -ENOMEM;
		goto fail;
	}
	job->ctx = ctx;
	job->src = src;
	job->dst = dst;
	job->capture_new = !nvdec_find_surface(ctx, &dst->vb2_buf);
	err = nvdec_capture_surface(ctx, &dst->vb2_buf, &job->surface);
	if (err)
		goto free_job;
	err = nvdec_map_buffer(ctx, &dst->vb2_buf, DMA_FROM_DEVICE, 0,
			       &job->capture);
	if (err)
		goto free_job;
	err = nvdec_resolve_dpb(ctx, job);
	if (err)
		goto free_job;
	err = nvdec_engine_map_wait(job->capture, true);
	for (i = 0; !err && i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (job->dpb[i])
			err = nvdec_engine_map_wait(job->dpb[i], false);
	}
	if (err)
		goto free_job;
	ctx->job = job;
	switch (ctx->codec) {
	case NVDEC_CODEC_HEVC:
		err = nvdec_engine_hevc_submit(ctx->decode, &ctx->hevc,
					       job->surface->map, job->capture,
					       job->dpb, &job->fence,
					       nvdec_job_complete, job);
		break;
	case NVDEC_CODEC_VP8:
		err = nvdec_engine_vp8_submit(ctx->decode, &ctx->vp8,
					      job->surface->map, job->capture,
					      job->dpb, &job->fence,
					      nvdec_job_complete, job);
		break;
	default:
		err = nvdec_engine_h264_submit(ctx->decode, &ctx->picture,
					       job->surface->map, job->capture,
					       job->dpb, &job->fence,
					       nvdec_job_complete, job);
		break;
	}
	if (err) {
		ctx->job = NULL;
		goto free_job;
	}
	return;

free_job:
	if (job->capture_new)
		nvdec_release_surface(ctx, &dst->vb2_buf);
	nvdec_job_cleanup_maps(job);
	kfree(job);
fail:
	if (controls_set_up)
		v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	nvdec_engine_discard_slices(ctx->decode);
	src->flags &= ~V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
	v4l2_m2m_buf_done_and_job_finish(ctx->nvdec->m2m_dev, ctx->fh.m2m_ctx,
					 VB2_BUF_STATE_ERROR);
}

static void nvdec_job_abort(void *priv)
{
	struct nvdec_v4l2_ctx *ctx = priv;

	if (ctx->job)
		ctx->job->aborted = true;
}

static const struct v4l2_m2m_ops nvdec_m2m_ops = {
	.device_run = nvdec_device_run,
	.job_abort = nvdec_job_abort,
};

static int nvdec_request_validate(struct media_request *req)
{
	unsigned int count = vb2_request_buffer_cnt(req);

	if (count != 1)
		return -EINVAL;
	return vb2_request_validate(req);
}

static const struct media_device_ops nvdec_media_ops = {
	.req_validate = nvdec_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

static int nvdec_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "tegra-nvdec", sizeof(cap->driver));
	strscpy(cap->card, "Tegra NVDEC", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:tegra-nvdec", sizeof(cap->bus_info));
	return 0;
}

static int nvdec_enum_fmt(struct file *file, void *priv,
			  struct v4l2_fmtdesc *f)
{
	static const u32 coded[] = {
		V4L2_PIX_FMT_H264_SLICE,
		V4L2_PIX_FMT_HEVC_SLICE,
		V4L2_PIX_FMT_VP8_FRAME,
	};

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (f->index >= ARRAY_SIZE(coded))
			return -EINVAL;
		f->pixelformat = coded[f->index];
		return 0;
	}

	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_NV12;
	return 0;
}

static int nvdec_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct nvdec_v4l2_ctx *ctx = file_to_nvdec_ctx(file);

	*f = V4L2_TYPE_IS_OUTPUT(f->type) ? ctx->coded_fmt : ctx->capture_fmt;
	return 0;
}

static int nvdec_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct nvdec_v4l2_ctx *ctx = file_to_nvdec_ctx(file);

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		return nvdec_try_coded_fmt(f);
	nvdec_fill_capture_fmt(ctx, f, ctx->crop.width, ctx->crop.height);
	return 0;
}

static int nvdec_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct nvdec_v4l2_ctx *ctx = file_to_nvdec_ctx(file);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int err;

	if (vb2_is_busy(vq))
		return -EBUSY;
	err = nvdec_try_fmt(file, priv, f);
	if (err)
		return err;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		enum nvdec_codec codec = NVDEC_CODEC_H264;

		if (f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_HEVC_SLICE)
			codec = NVDEC_CODEC_HEVC;
		else if (f->fmt.pix_mp.pixelformat == V4L2_PIX_FMT_VP8_FRAME)
			codec = NVDEC_CODEC_VP8;

		if (vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
			return -EBUSY;
		err = nvdec_set_codec(ctx, codec);
		if (err)
			return err;
		ctx->coded_fmt = *f;
		nvdec_reset_capture_fmt(ctx);
	} else {
		ctx->capture_fmt = *f;
	}
	return 0;
}

/* CROP picks what VIC detiles out; COMPOSE is all of it, there is no scaler. */
static int nvdec_g_selection(struct file *file, void *priv,
			     struct v4l2_selection *sel)
{
	struct nvdec_v4l2_ctx *ctx = file_to_nvdec_ctx(file);

	if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = ctx->coded_fmt.fmt.pix_mp.width;
		sel->r.height = ctx->coded_fmt.fmt.pix_mp.height;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = ctx->crop;
		return 0;
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = ctx->crop.width;
		sel->r.height = ctx->crop.height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int nvdec_s_selection(struct file *file, void *priv,
			     struct v4l2_selection *sel)
{
	struct nvdec_v4l2_ctx *ctx = file_to_nvdec_ctx(file);
	u32 width = ctx->coded_fmt.fmt.pix_mp.width;
	u32 height = ctx->coded_fmt.fmt.pix_mp.height;
	struct v4l2_rect r;

	if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	if (vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;

	/* VIC samples 4:2:0 chroma, so the rectangle has to stay even. */
	r.left = min_t(u32, ALIGN_DOWN(max_t(int, sel->r.left, 0), 2), width - 2);
	r.top = min_t(u32, ALIGN_DOWN(max_t(int, sel->r.top, 0), 2), height - 2);
	r.width = clamp_t(u32, ALIGN_DOWN(sel->r.width, 2), 2, width - r.left);
	r.height = clamp_t(u32, ALIGN_DOWN(sel->r.height, 2), 2, height - r.top);

	ctx->crop = r;
	sel->r = r;
	nvdec_fill_capture_fmt(ctx, &ctx->capture_fmt, r.width, r.height);
	return 0;
}

static int nvdec_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	const struct nvdec_codec_size *size;

	if (fsize->index)
		return -EINVAL;
	size = nvdec_codec_size(fsize->pixel_format);
	if (!size)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = size->min_width;
	fsize->stepwise.max_width = NVDEC_MAX_WIDTH;
	fsize->stepwise.step_width = size->align;
	fsize->stepwise.min_height = size->min_height;
	fsize->stepwise.max_height = NVDEC_MAX_HEIGHT;
	fsize->stepwise.step_height = size->align;
	return 0;
}

static const struct v4l2_ioctl_ops nvdec_ioctl_ops = {
	.vidioc_querycap = nvdec_querycap,
	.vidioc_enum_fmt_vid_out = nvdec_enum_fmt,
	.vidioc_enum_fmt_vid_cap = nvdec_enum_fmt,
	.vidioc_enum_framesizes = nvdec_enum_framesizes,
	.vidioc_g_selection = nvdec_g_selection,
	.vidioc_s_selection = nvdec_s_selection,
	.vidioc_g_fmt_vid_out_mplane = nvdec_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = nvdec_try_fmt,
	.vidioc_s_fmt_vid_out_mplane = nvdec_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = nvdec_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = nvdec_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = nvdec_s_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int nvdec_open(struct file *file)
{
	struct nvdec_v4l2 *nvdec = video_drvdata(file);
	struct nvdec_v4l2_ctx *ctx;
	unsigned int i;
	int err;

	if (!nvdec_engine_ready(nvdec->engine))
		return -ENODEV;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->nvdec = nvdec;
	INIT_LIST_HEAD(&ctx->surfaces);
	ctx->codec = NVDEC_CODEC_H264;
	ctx->decode = nvdec_engine_context_create(nvdec->engine, ctx->codec);
	if (IS_ERR(ctx->decode)) {
		err = PTR_ERR(ctx->decode);
		goto free_ctx;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, ARRAY_SIZE(nvdec_h264_ctrls) +
			       ARRAY_SIZE(nvdec_hevc_ctrls) +
			       ARRAY_SIZE(nvdec_vp8_ctrls));
	for (i = 0; i < ARRAY_SIZE(nvdec_h264_ctrls); i++) {
		v4l2_ctrl_new_custom(&ctx->ctrl_hdl, &nvdec_h264_ctrls[i], NULL);
		if (ctx->ctrl_hdl.error)
			goto free_ctrls;
	}
	for (i = 0; i < ARRAY_SIZE(nvdec_hevc_ctrls); i++) {
		v4l2_ctrl_new_custom(&ctx->ctrl_hdl, &nvdec_hevc_ctrls[i], NULL);
		if (ctx->ctrl_hdl.error)
			goto free_ctrls;
	}
	for (i = 0; i < ARRAY_SIZE(nvdec_vp8_ctrls); i++) {
		v4l2_ctrl_new_custom(&ctx->ctrl_hdl, &nvdec_vp8_ctrls[i], NULL);
		if (ctx->ctrl_hdl.error)
			goto free_ctrls;
	}
	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(nvdec->m2m_dev, ctx, nvdec_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		err = PTR_ERR(ctx->fh.m2m_ctx);
		goto free_ctrls;
	}
	nvdec_reset_coded_fmt(ctx);
	nvdec_reset_capture_fmt(ctx);
	v4l2_fh_add(&ctx->fh, file);
	return 0;

free_ctrls:
	err = ctx->ctrl_hdl.error ?: err;
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_exit(&ctx->fh);
	nvdec_engine_context_destroy(ctx->decode);
free_ctx:
	kfree(ctx);
	return err;
}

static int nvdec_release(struct file *file)
{
	struct nvdec_v4l2_ctx *ctx = file_to_nvdec_ctx(file);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	nvdec_release_surfaces(ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	v4l2_fh_exit(&ctx->fh);
	nvdec_engine_context_destroy(ctx->decode);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations nvdec_fops = {
	.owner = THIS_MODULE,
	.open = nvdec_open,
	.release = nvdec_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

int nvdec_v4l2_register(struct nvdec_engine *engine)
{
	struct nvdec_v4l2 *nvdec;
	int err;

	nvdec = devm_kzalloc(nvdec_engine_device(engine), sizeof(*nvdec), GFP_KERNEL);
	if (!nvdec)
		return -ENOMEM;
	nvdec->engine = engine;
	nvdec->dev = nvdec_engine_device(engine);
	mutex_init(&nvdec->lock);
	media_device_init(&nvdec->mdev);
	nvdec->mdev.dev = nvdec->dev;
	nvdec->mdev.ops = &nvdec_media_ops;
	strscpy(nvdec->mdev.model, "Tegra NVDEC", sizeof(nvdec->mdev.model));
	strscpy(nvdec->mdev.bus_info, "platform:tegra-nvdec",
		sizeof(nvdec->mdev.bus_info));

	nvdec->v4l2_dev.mdev = &nvdec->mdev;
	err = v4l2_device_register(nvdec->dev, &nvdec->v4l2_dev);
	if (err)
		goto cleanup_media;
	err = media_device_register(&nvdec->mdev);
	if (err)
		goto unregister_v4l2;
	nvdec->m2m_dev = v4l2_m2m_init(&nvdec_m2m_ops);
	if (IS_ERR(nvdec->m2m_dev)) {
		err = PTR_ERR(nvdec->m2m_dev);
		goto unregister_media;
	}

	video_set_drvdata(&nvdec->vdev, nvdec);
	nvdec->vdev.v4l2_dev = &nvdec->v4l2_dev;
	nvdec->vdev.fops = &nvdec_fops;
	nvdec->vdev.ioctl_ops = &nvdec_ioctl_ops;
	nvdec->vdev.lock = &nvdec->lock;
	nvdec->vdev.vfl_dir = VFL_DIR_M2M;
	nvdec->vdev.release = video_device_release_empty;
	nvdec->vdev.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	strscpy(nvdec->vdev.name, "tegra-nvdec", sizeof(nvdec->vdev.name));
	err = video_register_device(&nvdec->vdev, VFL_TYPE_VIDEO, -1);
	if (err)
		goto release_m2m;
	err = v4l2_m2m_register_media_controller(nvdec->m2m_dev, &nvdec->vdev,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (err)
		goto unregister_video;
	nvdec_engine_set_v4l2(engine, nvdec);
	v4l2_info(&nvdec->v4l2_dev, "registered /dev/video%d\n", nvdec->vdev.num);
	return 0;

unregister_video:
	video_unregister_device(&nvdec->vdev);
release_m2m:
	v4l2_m2m_release(nvdec->m2m_dev);
unregister_media:
	media_device_unregister(&nvdec->mdev);
unregister_v4l2:
	v4l2_device_unregister(&nvdec->v4l2_dev);
cleanup_media:
	media_device_cleanup(&nvdec->mdev);
	return err;
}

void nvdec_v4l2_unregister(struct nvdec_engine *engine)
{
	struct nvdec_v4l2 *nvdec = nvdec_engine_get_v4l2(engine);

	if (!nvdec)
		return;
	nvdec_engine_set_v4l2(engine, NULL);
	v4l2_m2m_unregister_media_controller(nvdec->m2m_dev);
	video_unregister_device(&nvdec->vdev);
	v4l2_m2m_release(nvdec->m2m_dev);
	media_device_unregister(&nvdec->mdev);
	v4l2_device_unregister(&nvdec->v4l2_dev);
	media_device_cleanup(&nvdec->mdev);
}
