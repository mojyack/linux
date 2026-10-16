// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tegra210 NVDEC stateless V4L2 decoder.
 *
 * Copyright (c) 2026 mojyack <mojyack@gmail.com>
 */

#include <linux/dma-buf.h>
#include <linux/dma-resv.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>

#include "drm.h"
#include "nvdec.h"
#include "nvdec-decode.h"

#define NVDEC_MAX_WIDTH		4096
#define NVDEC_MAX_HEIGHT	4096
#define NVDEC_CODED_SIZE	SZ_4M
#define NVDEC_FENCE_TIMEOUT_MS	5000
#define NVDEC_MAX_CTRLS		8

struct nvdec_v4l2 {
	struct tegra_drm_client *client;
	struct host1x_channel *channel;
	struct device *dev;
	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	/* Serializes the vb2 queues and ioctls of every context. */
	struct mutex lock;
	/* Every open context, for unregistering to revoke; under lock. */
	struct list_head contexts;
	/* Set under lock before revoking: no context may reach the engine again. */
	bool disconnected;
	bool mc_registered;
	/* device_run() hands the picture over; the engine's completion comes back. */
	struct nvdec_v4l2_ctx *curr;
	struct work_struct run_work;
	struct work_struct done_work;
};

/* A field decoded into @vb, which a field of opposite parity may complete. */
struct nvdec_field {
	struct vb2_buffer *vb;
	u64 timestamp;
	bool bottom;
	u32 frame_num;
};

struct nvdec_v4l2_ctx {
	struct nvdec_v4l2 *nvdec;
	struct list_head list;
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_hdl;
	/* Every codec's controls, and the copy the picture is decoded from. */
	struct v4l2_ctrl *ctrls[NVDEC_NUM_CODECS][NVDEC_MAX_CTRLS];
	void *ctrl_copy[NVDEC_NUM_CODECS][NVDEC_MAX_CTRLS];
	struct v4l2_format coded_fmt;
	/* The coded size rounded up to the codec's surface alignment. */
	u32 coded_width;
	u32 coded_height;
	struct v4l2_format capture_fmt;
	enum nvdec_codec codec;
	/* NULL once the engine has gone away underneath an open file. */
	struct nvdec_decode_context *decode;
	struct list_head surfaces;
	/* Picture being decoded. */
	union nvdec_request request;
	/* A decoded first field the next picture may complete, and the candidate. */
	struct nvdec_field first_field;
	struct nvdec_field next_first_field;
};

struct nvdec_v4l2_surface {
	struct list_head list;
	struct vb2_buffer *vb;
	struct nvdec_decode_map *map;
	/* The capture buffer's own mapping, kept for as long as it is queued. */
	struct nvdec_decode_map *capture;
	struct dma_buf *dbuf;
};

static int nvdec_try_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops nvdec_ctrl_ops = {
	.try_ctrl = nvdec_try_ctrl,
};

/* The firmware finds and parses the slices itself, so it takes whole pictures. */
static const struct v4l2_ctrl_config nvdec_h264_ctrls[] = {
	{ .id = V4L2_CID_STATELESS_H264_DECODE_PARAMS },
	{ .id = V4L2_CID_STATELESS_H264_SPS, .ops = &nvdec_ctrl_ops },
	{ .id = V4L2_CID_STATELESS_H264_PPS, .ops = &nvdec_ctrl_ops },
	{ .id = V4L2_CID_STATELESS_H264_SCALING_MATRIX },
	{
		.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.min = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.max = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
		.def = V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
	}, {
		.id = V4L2_CID_STATELESS_H264_START_CODE,
		.min = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.def = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
	}, {
		/* Baseline and Extended streams decode only without ASO and FMO. */
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.min = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
		.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
		.menu_skip_mask = BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED),
		.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
	},
};

static_assert(ARRAY_SIZE(nvdec_h264_ctrls) <= NVDEC_MAX_CTRLS);

static const u32 nvdec_h264_required[] = {
	V4L2_CID_STATELESS_H264_SPS,
	V4L2_CID_STATELESS_H264_PPS,
	V4L2_CID_STATELESS_H264_DECODE_PARAMS,
};

static inline struct nvdec_v4l2_ctx *file_to_nvdec_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct nvdec_v4l2_ctx, fh);
}

static void nvdec_release_surfaces(struct nvdec_v4l2_ctx *ctx);

/* The decoded frame keeps the colorimetry the client set on the OUTPUT queue. */
static void nvdec_fill_capture_fmt(const struct nvdec_v4l2_ctx *ctx,
				   struct v4l2_format *f, u32 pixelformat,
				   u32 width, u32 height)
{
	const struct v4l2_pix_format_mplane *coded = &ctx->coded_fmt.fmt.pix_mp;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	/* VIC pitch-linear requirement. */
	u32 stride = ALIGN(width, 256);
	u32 aligned_height = ALIGN(height, 16);

	memset(f, 0, sizeof(*f));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	pix->width = width;
	pix->height = height;
	pix->pixelformat = pixelformat;
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
	nvdec_fill_capture_fmt(ctx, &ctx->capture_fmt, V4L2_PIX_FMT_NV12,
			       ctx->coded_width, ctx->coded_height);
}

/* Kernel-owned block-linear surface NVDEC decodes into and VIC detiles. */
static u32 nvdec_surface_stride(const struct nvdec_v4l2_ctx *ctx)
{
	return ALIGN(ctx->coded_width, 64);
}

static u32 nvdec_surface_chroma_offset(const struct nvdec_v4l2_ctx *ctx)
{
	return nvdec_surface_stride(ctx) * ALIGN(ctx->coded_height, 32);
}

static size_t nvdec_surface_size(const struct nvdec_v4l2_ctx *ctx)
{
	u32 height = ctx->coded_height;

	return nvdec_surface_chroma_offset(ctx) +
	       nvdec_surface_stride(ctx) * ALIGN(DIV_ROUND_UP(height, 2), 16);
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
	ctx->first_field.vb = NULL;
	if (V4L2_TYPE_IS_CAPTURE(vq->type)) {
		/* Drop the size-derived scratch so the next stream may differ. */
		if (ctx->decode)
			nvdec_decode_context_reset(ctx->decode);
		nvdec_release_surfaces(ctx);
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
	src_vq->mem_ops = &vb2_dma_sg_memops;
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
	dst_vq->mem_ops = &vb2_dma_sg_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &nvdec->lock;
	dst_vq->dev = nvdec->dev;
	return vb2_queue_init(dst_vq);
}

/* The picture's value of control @id, as nvdec_copy_ctrls() took it. */
static const void *nvdec_ctrl_ptr(struct nvdec_v4l2_ctx *ctx, u32 id)
{
	unsigned int i;

	for (i = 0; i < NVDEC_MAX_CTRLS && ctx->ctrls[ctx->codec][i]; i++)
		if (ctx->ctrls[ctx->codec][i]->id == id)
			return ctx->ctrl_copy[ctx->codec][i];
	return NULL;
}

/* What the request set, as it set it, else the current value it inherits. */
static void nvdec_copy_ctrls(struct nvdec_v4l2_ctx *ctx, struct media_request *req)
{
	struct v4l2_ctrl *const *ctrls = ctx->ctrls[ctx->codec];
	void *const *copy = ctx->ctrl_copy[ctx->codec];
	struct v4l2_ctrl_handler *hdl;
	struct v4l2_ctrl_ref *ref;
	unsigned int i;

	mutex_lock(ctx->ctrl_hdl.lock);
	for (i = 0; i < NVDEC_MAX_CTRLS && ctrls[i]; i++)
		memcpy(copy[i], ctrls[i]->p_cur.p,
		       ctrls[i]->elems * ctrls[i]->elem_size);
	mutex_unlock(ctx->ctrl_hdl.lock);

	hdl = v4l2_ctrl_request_hdl_find(req, &ctx->ctrl_hdl);
	if (!hdl)
		return;

	/* A queued request no longer changes. */
	mutex_lock(hdl->lock);
	list_for_each_entry(ref, &hdl->ctrl_refs, node) {
		if (!ref->p_req_valid)
			continue;
		for (i = 0; i < NVDEC_MAX_CTRLS && ctrls[i]; i++) {
			if (ref->ctrl == ctrls[i]) {
				memcpy(copy[i], ref->p_req.p,
				       ctrls[i]->elems * ctrls[i]->elem_size);
				break;
			}
		}
	}
	mutex_unlock(hdl->lock);
	v4l2_ctrl_request_hdl_put(hdl);
}

static void nvdec_free_ctrl_copies(struct nvdec_v4l2_ctx *ctx)
{
	unsigned int i, j;

	for (i = 0; i < NVDEC_NUM_CODECS; i++)
		for (j = 0; j < NVDEC_MAX_CTRLS; j++)
			kfree(ctx->ctrl_copy[i][j]);
}

/* Whether @req itself sets control @id, rather than inheriting a value. */
static bool nvdec_request_has(struct media_request *req,
			      struct v4l2_ctrl_handler *parent, u32 id)
{
	struct v4l2_ctrl_handler *hdl;
	bool found;

	hdl = v4l2_ctrl_request_hdl_find(req, parent);
	if (!hdl)
		return false;
	found = v4l2_ctrl_request_hdl_ctrl_find(hdl, id);
	v4l2_ctrl_request_hdl_put(hdl);
	return found;
}

/* FrameHeightInMbs = (2 - frame_mbs_only_flag) * PicHeightInMapUnits. */
static u32 nvdec_h264_frame_height_in_mbs(const struct v4l2_ctrl_h264_sps *sps)
{
	return ((sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) ? 1 : 2) *
	       (sps->pic_height_in_map_units_minus1 + 1);
}

static bool nvdec_h264_mbaff(const struct v4l2_ctrl_h264_sps *sps,
			     const struct v4l2_ctrl_h264_decode_params *dec)
{
	return (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD) &&
	       !(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) &&
	       !(dec->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC);
}

/* What the hardware cannot decode, refused when the control is set. */
static int nvdec_try_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_STATELESS_H264_SPS: {
		const struct v4l2_ctrl_h264_sps *sps = ctrl->p_new.p_h264_sps;

		/* The all-zero default has to pass; monochrome is refused per picture. */
		if (sps->level_idc > 51 || sps->chroma_format_idc > 1 ||
		    sps->bit_depth_luma_minus8 || sps->bit_depth_chroma_minus8 ||
		    (sps->flags & (V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE |
				   V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS)))
			return -EINVAL;
		return 0;
	}
	case V4L2_CID_STATELESS_H264_PPS: {
		const struct v4l2_ctrl_h264_pps *pps = ctrl->p_new.p_h264_pps;

		return pps->num_slice_groups_minus1 ? -EINVAL : 0;
	}
	}

	return 0;
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
				  struct nvdec_v4l2_surface *surface)
{
	if (ctx->decode)
		nvdec_decode_context_release_surface(ctx->decode, surface->map);
	list_del(&surface->list);
	nvdec_decode_map_put(surface->capture);
	nvdec_decode_map_put(surface->map);
	kfree(surface);
}

static void nvdec_release_surfaces(struct nvdec_v4l2_ctx *ctx)
{
	struct nvdec_v4l2_surface *surface, *tmp;

	list_for_each_entry_safe(surface, tmp, &ctx->surfaces, list)
		nvdec_release_surface(ctx, surface);
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
	surface->map = nvdec_decode_surface_create(ctx->decode,
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

/* Mapped once and kept on the surface; an import may bring a new dma-buf. */
static int nvdec_capture_map(struct nvdec_v4l2_ctx *ctx,
			     struct nvdec_v4l2_surface *surface,
			     struct vb2_buffer *vb)
{
	struct dma_buf *dmabuf;

	if (surface->capture && surface->dbuf != vb->planes[0].dbuf) {
		nvdec_decode_map_put(surface->capture);
		surface->capture = NULL;
	}
	if (surface->capture)
		return 0;

	dmabuf = nvdec_plane_dmabuf(vb);
	if (!dmabuf)
		return -ENOMEM;
	surface->capture = nvdec_decode_map_create(ctx->decode, dmabuf,
						   vb2_plane_size(vb, 0),
						   DMA_FROM_DEVICE);
	dma_buf_put(dmabuf);
	if (IS_ERR(surface->capture)) {
		int err = PTR_ERR(surface->capture);

		surface->capture = NULL;
		return err;
	}
	surface->dbuf = vb->planes[0].dbuf;
	return 0;
}

static int nvdec_set_codec(struct nvdec_v4l2_ctx *ctx, enum nvdec_codec codec)
{
	struct nvdec_decode_context *decode;

	if (ctx->codec == codec)
		return 0;
	if (ctx->nvdec->disconnected)
		return -ENODEV;

	decode = nvdec_decode_context_create(ctx->nvdec->client,
					     ctx->nvdec->channel, codec);
	if (IS_ERR(decode))
		return PTR_ERR(decode);

	nvdec_release_surfaces(ctx);
	nvdec_decode_context_destroy(ctx->decode);
	ctx->decode = decode;
	ctx->codec = codec;
	return 0;
}

/* A field completes the field of the same frame decoded just before it. */
static bool nvdec_pair_field(struct nvdec_v4l2_ctx *ctx,
			     struct vb2_v4l2_buffer *src,
			     struct vb2_v4l2_buffer *dst, bool field, bool bottom,
			     u32 frame_num)
{
	const struct nvdec_field *first = &ctx->first_field;
	bool second = field && first->vb == &dst->vb2_buf &&
		      first->timestamp == src->vb2_buf.timestamp &&
		      first->bottom != bottom && first->frame_num == frame_num;

	ctx->first_field.vb = NULL;
	ctx->next_first_field.vb = field && !second ? &dst->vb2_buf : NULL;
	ctx->next_first_field.timestamp = src->vb2_buf.timestamp;
	ctx->next_first_field.bottom = bottom;
	ctx->next_first_field.frame_num = frame_num;
	return second;
}

/* Clears the request and fills the surface and detile geometry it starts with. */
static void nvdec_snapshot_frame(struct nvdec_v4l2_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->capture_fmt.fmt.pix_mp;
	struct nvdec_frame *f = &ctx->request.frame;

	memset(&ctx->request, 0, sizeof(ctx->request));
	f->coded_width = ctx->coded_width;
	f->coded_height = ctx->coded_height;
	f->crop_width = pix->width;
	f->crop_height = pix->height;
	f->luma_stride = nvdec_surface_stride(ctx);
	f->chroma_offset = nvdec_surface_chroma_offset(ctx);
	f->dst_stride = pix->plane_fmt[0].bytesperline;
	f->dst_chroma_offset = f->dst_stride * pix->height;
}

static int nvdec_validate_ref(struct nvdec_v4l2_ctx *ctx,
			      const struct v4l2_h264_dpb_entry *entry)
{
	struct vb2_queue *cap_q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);

	if (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
		return 0;
	if ((entry->flags & ~(V4L2_H264_DPB_ENTRY_FLAG_VALID |
			      V4L2_H264_DPB_ENTRY_FLAG_ACTIVE |
			      V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM |
			      V4L2_H264_DPB_ENTRY_FLAG_FIELD)) ||
	    entry->reserved[0] || entry->reserved[1] || entry->reserved[2] ||
	    entry->reserved[3] || entry->reserved[4])
		return -EINVAL;
	if (entry->fields > V4L2_H264_FRAME_REF)
		return -EINVAL;
	/* Kept for output only: not a reference, and its buffer may be gone. */
	if (!(entry->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
		return 0;
	return vb2_find_buffer(cap_q, entry->reference_ts) ? 0 : -EINVAL;
}

static int nvdec_validate_h264_request(struct nvdec_v4l2_ctx *ctx,
				       struct media_request *req)
{
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_decode_params *dec;
	const char *why;
	unsigned int i;

	sps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SPS);
	pps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_PPS);
	dec = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);

	/* Baseline and Extended: what the PPS and slice checks do not refuse. */
	if ((sps->profile_idc != 66 && sps->profile_idc != 77 &&
	     sps->profile_idc != 88 && sps->profile_idc != 100) ||
	    sps->chroma_format_idc != 1) {
		why = "sps";
		goto reject;
	}
	if (pps->seq_parameter_set_id != sps->seq_parameter_set_id ||
	    ((pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT) &&
	     !nvdec_request_has(req, &ctx->ctrl_hdl,
				V4L2_CID_STATELESS_H264_SCALING_MATRIX))) {
		why = "pps";
		goto reject;
	}
	if (dec->reserved ||
	    (dec->flags & ~(V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC |
			    V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC |
			    V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD |
			    V4L2_H264_DECODE_PARAM_FLAG_PFRAME |
			    V4L2_H264_DECODE_PARAM_FLAG_BFRAME)) ||
	    (!(dec->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC) &&
	     (dec->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD))) {
		why = "decode params";
		goto reject;
	}
	if ((sps->pic_width_in_mbs_minus1 + 1) * 16 != ctx->coded_width ||
	    nvdec_h264_frame_height_in_mbs(sps) * 16 != ctx->coded_height) {
		why = "sps dimensions do not match the coded format";
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

	return 0;

reject:
	dev_dbg(ctx->nvdec->dev, "h264 reject: %s\n", why);
	return -EINVAL;
}

static int nvdec_snapshot_h264_request(struct nvdec_v4l2_ctx *ctx,
				       struct vb2_v4l2_buffer *src,
				       struct vb2_v4l2_buffer *dst)
{
	struct nvdec_h264_request *request = &ctx->request.h264;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_decode_params *dec;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling;
	unsigned int i;

	if (nvdec_validate_h264_request(ctx, src->vb2_buf.req_obj.req))
		return -EINVAL;
	sps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SPS);
	pps = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_PPS);
	dec = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	scaling = nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	nvdec_snapshot_frame(ctx);
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
	/* The defaults; the firmware applies a slice header's override itself. */
	request->num_ref_idx_l0_active_minus1 = pps->num_ref_idx_l0_default_active_minus1;
	request->num_ref_idx_l1_active_minus1 = pps->num_ref_idx_l1_default_active_minus1;
	request->nal_ref_idc = dec->nal_ref_idc;
	request->frame_num = dec->frame_num;
	request->pic_width_in_mbs = sps->pic_width_in_mbs_minus1 + 1;
	request->frame_height_in_mbs = nvdec_h264_frame_height_in_mbs(sps);
	request->top_field_order_cnt = dec->top_field_order_cnt;
	request->bottom_field_order_cnt = dec->bottom_field_order_cnt;
	request->pic_init_qp_minus26 = pps->pic_init_qp_minus26;
	request->chroma_qp_index_offset = pps->chroma_qp_index_offset;
	request->second_chroma_qp_index_offset = pps->second_chroma_qp_index_offset;
	request->weighted_bipred_idc = pps->weighted_bipred_idc;
	request->num_slice_groups_minus1 = pps->num_slice_groups_minus1;
	if (sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)
		request->flags |= NVDEC_H264_REQ_FRAME_MBS_ONLY;
	if (nvdec_h264_mbaff(sps, dec))
		request->flags |= NVDEC_H264_REQ_MBAFF;
	if (dec->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC)
		request->flags |= NVDEC_H264_REQ_FIELD;
	if (dec->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD)
		request->flags |= NVDEC_H264_REQ_BOTTOM_FIELD;
	if (nvdec_pair_field(ctx, src, dst,
			     dec->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC,
			     dec->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD,
			     dec->frame_num))
		request->flags |= NVDEC_H264_REQ_SECOND_FIELD;
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
	if (pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT) {
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

		if (!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) ||
		    !(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			continue;
		request->dpb[i].valid = 1;
		request->dpb[i].long_term =
			!!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM);
		request->dpb[i].fields = dpb->fields;
		request->dpb[i].field_picture =
			!!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_FIELD);
		request->dpb[i].frame_num = dpb->frame_num;
		request->dpb[i].top_field_order_cnt = dpb->top_field_order_cnt;
		request->dpb[i].bottom_field_order_cnt = dpb->bottom_field_order_cnt;
	}
	return 0;
}

/* The picture being decoded: its surface, and the surfaces it predicts from. */
struct nvdec_v4l2_job {
	struct nvdec_v4l2_surface *surface;
	struct nvdec_decode_map *dpb[NVDEC_MAX_REFS];
};

/* Each DPB entry names a capture buffer, and through it a pool surface. */
static struct nvdec_v4l2_surface *
nvdec_pin_reference(struct nvdec_v4l2_ctx *ctx, struct nvdec_v4l2_job *job,
		    unsigned int slot, u64 timestamp)
{
	struct vb2_queue *cap_q = v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx);
	struct nvdec_v4l2_surface *surface;

	surface = nvdec_find_surface(ctx, vb2_find_buffer(cap_q, timestamp));
	if (!surface)
		return NULL;

	job->dpb[slot] = surface->map;
	return surface;
}

static int nvdec_resolve_h264(struct nvdec_v4l2_ctx *ctx,
			      struct nvdec_v4l2_job *job)
{
	const struct v4l2_ctrl_h264_decode_params *dec =
		nvdec_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	unsigned int i;

	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (!ctx->request.h264.dpb[i].valid)
			continue;
		if (!nvdec_pin_reference(ctx, job, i, dec->dpb[i].reference_ts))
			return -EINVAL;
	}

	return 0;
}

/* Everything that differs between the coded formats on the OUTPUT queue. */
struct nvdec_codec_desc {
	u32 pixelformat;
	/* Smallest picture the engine decodes correctly; surfaces are aligned. */
	u16 min_width;
	u16 min_height;
	u16 align;
	const struct v4l2_ctrl_config *ctrls;
	unsigned int num_ctrls;
	/* What every request of this format has to set. */
	const u32 *required;
	unsigned int num_required;
	int (*snapshot)(struct nvdec_v4l2_ctx *ctx, struct vb2_v4l2_buffer *src,
			struct vb2_v4l2_buffer *dst);
	int (*resolve)(struct nvdec_v4l2_ctx *ctx, struct nvdec_v4l2_job *job);
};

static const struct nvdec_codec_desc nvdec_codecs[] = {
	[NVDEC_CODEC_H264] = {
		.pixelformat = V4L2_PIX_FMT_H264_SLICE,
		.min_width = 48,
		.min_height = 16,
		.align = 16,
		.ctrls = nvdec_h264_ctrls,
		.num_ctrls = ARRAY_SIZE(nvdec_h264_ctrls),
		.required = nvdec_h264_required,
		.num_required = ARRAY_SIZE(nvdec_h264_required),
		.snapshot = nvdec_snapshot_h264_request,
		.resolve = nvdec_resolve_h264,
	},
};

static const struct nvdec_codec_desc *nvdec_codec_desc(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(nvdec_codecs); i++)
		if (nvdec_codecs[i].pixelformat == pixelformat)
			return &nvdec_codecs[i];
	return NULL;
}

static void nvdec_align_coded_fmt(struct nvdec_v4l2_ctx *ctx)
{
	u32 align = nvdec_codecs[ctx->codec].align;

	ctx->coded_width = ALIGN(ctx->coded_fmt.fmt.pix_mp.width, align);
	ctx->coded_height = ALIGN(ctx->coded_fmt.fmt.pix_mp.height, align);
}

static void nvdec_reset_coded_fmt(struct nvdec_v4l2_ctx *ctx)
{
	struct v4l2_pix_format_mplane *pix = &ctx->coded_fmt.fmt.pix_mp;
	const struct nvdec_codec_desc *desc = &nvdec_codecs[NVDEC_CODEC_H264];

	memset(&ctx->coded_fmt, 0, sizeof(ctx->coded_fmt));
	ctx->coded_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	pix->width = desc->min_width;
	pix->height = desc->min_height;
	pix->pixelformat = desc->pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->plane_fmt[0].sizeimage = NVDEC_CODED_SIZE;
	nvdec_align_coded_fmt(ctx);
}

static int nvdec_try_coded_fmt(struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	const struct nvdec_codec_desc *desc;

	desc = nvdec_codec_desc(pix->pixelformat);
	if (!desc) {
		desc = &nvdec_codecs[NVDEC_CODEC_H264];
		pix->pixelformat = desc->pixelformat;
	}
	pix->width = clamp_t(u32, pix->width, desc->min_width, NVDEC_MAX_WIDTH);
	pix->height = clamp_t(u32, pix->height, desc->min_height, NVDEC_MAX_HEIGHT);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = 0;
	pix->plane_fmt[0].sizeimage = max_t(u32, pix->plane_fmt[0].sizeimage,
					    NVDEC_CODED_SIZE);
	return 0;
}

/* The engine reads a copy; an imported buffer is synced with its writer first. */
static int nvdec_stage(struct nvdec_v4l2_ctx *ctx, struct vb2_v4l2_buffer *src)
{
	struct vb2_buffer *vb = &src->vb2_buf;
	u32 payload = vb2_get_plane_payload(vb, 0);
	u32 offset = vb->planes[0].data_offset;
	struct dma_buf *dbuf = NULL;
	void *data;
	long ret;
	int err;

	/* bytesused counts data_offset. */
	if (offset >= payload)
		return -EINVAL;

	if (vb->memory == VB2_MEMORY_DMABUF) {
		dbuf = vb->planes[0].dbuf;
		ret = dma_resv_wait_timeout(dbuf->resv, DMA_RESV_USAGE_WRITE, false,
					    msecs_to_jiffies(NVDEC_FENCE_TIMEOUT_MS));
		if (ret <= 0)
			return ret ?: -ETIMEDOUT;
		err = dma_buf_begin_cpu_access(dbuf, DMA_FROM_DEVICE);
		if (err)
			return err;
	}

	data = vb2_plane_vaddr(vb, 0);
	err = data ? nvdec_decode_stage(ctx->decode, data + offset,
					payload - offset) : -ENOMEM;

	if (dbuf)
		dma_buf_end_cpu_access(dbuf, DMA_FROM_DEVICE);
	return err;
}

static void nvdec_job_done(struct nvdec_v4l2_ctx *ctx, int err)
{
	struct nvdec_v4l2 *nvdec = ctx->nvdec;
	struct vb2_v4l2_buffer *src;

	/* A failed field hands its capture buffer back and pairs with nothing. */
	if (err) {
		ctx->first_field.vb = NULL;
		src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
		if (src)
			src->flags &= ~V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
	} else {
		ctx->first_field = ctx->next_first_field;
	}
	v4l2_m2m_buf_done_and_job_finish(nvdec->m2m_dev, ctx->fh.m2m_ctx,
					 err ? VB2_BUF_STATE_ERROR :
					 VB2_BUF_STATE_DONE);
}

static void nvdec_job_complete(void *data)
{
	struct nvdec_v4l2_ctx *ctx = data;

	queue_work(system_highpri_wq, &ctx->nvdec->done_work);
}

static int nvdec_run(struct nvdec_v4l2_ctx *ctx)
{
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	const struct nvdec_codec_desc *desc = &nvdec_codecs[ctx->codec];
	struct media_request *req = src->vb2_buf.req_obj.req;
	struct nvdec_v4l2_job job = {};
	struct dma_resv *resv;
	long ret;
	int err;

	err = v4l2_ctrl_request_setup(req, &ctx->ctrl_hdl);
	if (!err) {
		nvdec_copy_ctrls(ctx, req);
		err = desc->snapshot(ctx, src, dst);
	}
	v4l2_ctrl_request_complete(req, &ctx->ctrl_hdl);
	if (err)
		return err;

	err = nvdec_stage(ctx, src);
	if (err)
		return err;
	v4l2_m2m_buf_copy_metadata(src, dst);

	err = nvdec_capture_surface(ctx, &dst->vb2_buf, &job.surface);
	if (!err)
		err = nvdec_capture_map(ctx, job.surface, &dst->vb2_buf);
	if (!err)
		err = desc->resolve(ctx, &job);
	if (err)
		return err;

	/* A display may still be reading the buffer it handed back. */
	resv = nvdec_decode_map_resv(job.surface->capture);
	if (resv) {
		ret = dma_resv_wait_timeout(resv, DMA_RESV_USAGE_READ, false,
					    msecs_to_jiffies(NVDEC_FENCE_TIMEOUT_MS));
		if (ret <= 0)
			return ret ?: -ETIMEDOUT;
	}

	return nvdec_decode_submit(ctx->decode, &ctx->request, job.surface->map,
				   job.surface->capture, job.dpb,
				   nvdec_job_complete, ctx);
}

static void nvdec_run_work(struct work_struct *work)
{
	struct nvdec_v4l2 *nvdec = container_of(work, struct nvdec_v4l2, run_work);
	struct nvdec_v4l2_ctx *ctx = nvdec->curr;
	int err;

	err = nvdec_run(ctx);
	if (err) {
		dev_dbg(nvdec->dev, "picture refused: %d\n", err);
		nvdec_job_done(ctx, err);
	}
}

static void nvdec_done_work(struct work_struct *work)
{
	struct nvdec_v4l2 *nvdec = container_of(work, struct nvdec_v4l2, done_work);
	struct nvdec_v4l2_ctx *ctx = nvdec->curr;
	int err;

	err = nvdec_decode_finish(ctx->decode);
	nvdec_job_done(ctx, err);
}

/* Called with the queue lock held, so the picture is decoded off this thread. */
static void nvdec_device_run(void *priv)
{
	struct nvdec_v4l2_ctx *ctx = priv;

	ctx->nvdec->curr = ctx;
	queue_work(system_highpri_wq, &ctx->nvdec->run_work);
}

static const struct v4l2_m2m_ops nvdec_m2m_ops = {
	.device_run = nvdec_device_run,
};

static int nvdec_request_validate(struct media_request *req)
{
	const struct nvdec_codec_desc *desc;
	struct media_request_object *obj;
	struct nvdec_v4l2_ctx *ctx = NULL;
	unsigned int i;

	if (vb2_request_buffer_cnt(req) != 1)
		return -EINVAL;

	list_for_each_entry(obj, &req->objects, list) {
		if (vb2_request_object_is_buffer(obj)) {
			struct vb2_buffer *vb =
				container_of(obj, struct vb2_buffer, req_obj);

			ctx = vb2_get_drv_priv(vb->vb2_queue);
			break;
		}
	}
	if (!ctx)
		return -ENOENT;

	desc = &nvdec_codecs[ctx->codec];
	for (i = 0; i < desc->num_required; i++) {
		if (!nvdec_request_has(req, &ctx->ctrl_hdl, desc->required[i])) {
			dev_dbg(ctx->nvdec->dev, "request lacks control 0x%x\n",
				desc->required[i]);
			return -ENOENT;
		}
	}

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
	return 0;
}

static int nvdec_enum_fmt(struct file *file, void *priv,
			  struct v4l2_fmtdesc *f)
{
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (f->index >= ARRAY_SIZE(nvdec_codecs))
			return -EINVAL;
		f->pixelformat = nvdec_codecs[f->index].pixelformat;
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
	nvdec_fill_capture_fmt(ctx, f, V4L2_PIX_FMT_NV12, ctx->coded_width,
			       ctx->coded_height);
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
		enum nvdec_codec codec =
			nvdec_codec_desc(f->fmt.pix_mp.pixelformat) - nvdec_codecs;

		if (vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
			return -EBUSY;
		err = nvdec_set_codec(ctx, codec);
		if (err)
			return err;
		ctx->coded_fmt = *f;
		nvdec_align_coded_fmt(ctx);
		nvdec_reset_capture_fmt(ctx);
	} else {
		ctx->capture_fmt = *f;
	}
	return 0;
}

/* The whole coded picture is detiled; the client crops, as from any decoder. */
static int nvdec_g_selection(struct file *file, void *priv,
			     struct v4l2_selection *sel)
{
	struct nvdec_v4l2_ctx *ctx = file_to_nvdec_ctx(file);

	if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = ctx->capture_fmt.fmt.pix_mp.width;
		sel->r.height = ctx->capture_fmt.fmt.pix_mp.height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int nvdec_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	const struct nvdec_codec_desc *desc;

	if (fsize->index)
		return -EINVAL;
	desc = nvdec_codec_desc(fsize->pixel_format);
	if (!desc)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = desc->min_width;
	fsize->stepwise.max_width = NVDEC_MAX_WIDTH;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = desc->min_height;
	fsize->stepwise.max_height = NVDEC_MAX_HEIGHT;
	fsize->stepwise.step_height = 1;
	return 0;
}

static const struct v4l2_ioctl_ops nvdec_ioctl_ops = {
	.vidioc_querycap = nvdec_querycap,
	.vidioc_enum_fmt_vid_out = nvdec_enum_fmt,
	.vidioc_enum_fmt_vid_cap = nvdec_enum_fmt,
	.vidioc_enum_framesizes = nvdec_enum_framesizes,
	.vidioc_g_selection = nvdec_g_selection,
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
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,
	.vidioc_decoder_cmd = v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int nvdec_open(struct file *file)
{
	struct nvdec_v4l2 *nvdec = video_drvdata(file);
	struct nvdec_v4l2_ctx *ctx;
	unsigned int i, j;
	int err;

	/* Held until the context is listed, so that unregistering finds it. */
	if (mutex_lock_interruptible(&nvdec->lock))
		return -ERESTARTSYS;
	if (nvdec->disconnected) {
		err = -ENODEV;
		goto unlock;
	}

	ctx = kzalloc_obj(*ctx);
	if (!ctx) {
		err = -ENOMEM;
		goto unlock;
	}
	ctx->nvdec = nvdec;
	INIT_LIST_HEAD(&ctx->surfaces);
	ctx->codec = NVDEC_CODEC_H264;
	ctx->decode = nvdec_decode_context_create(nvdec->client, nvdec->channel,
						  ctx->codec);
	if (IS_ERR(ctx->decode)) {
		err = PTR_ERR(ctx->decode);
		goto free_ctx;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_ctrl_handler_init(&ctx->ctrl_hdl, 20);
	for (i = 0; i < ARRAY_SIZE(nvdec_codecs); i++) {
		for (j = 0; j < nvdec_codecs[i].num_ctrls; j++) {
			struct v4l2_ctrl *ctrl;

			ctrl = v4l2_ctrl_new_custom(&ctx->ctrl_hdl,
						    &nvdec_codecs[i].ctrls[j], NULL);
			if (!ctrl)
				break;
			ctx->ctrls[i][j] = ctrl;
			ctx->ctrl_copy[i][j] = kmalloc_array(ctrl->elems,
							     ctrl->elem_size,
							     GFP_KERNEL);
			if (!ctx->ctrl_copy[i][j]) {
				err = -ENOMEM;
				goto free_ctrls;
			}
		}
	}
	if (ctx->ctrl_hdl.error) {
		err = ctx->ctrl_hdl.error;
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

	list_add_tail(&ctx->list, &nvdec->contexts);
	mutex_unlock(&nvdec->lock);
	v4l2_fh_add(&ctx->fh, file);
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	nvdec_free_ctrl_copies(ctx);
	v4l2_fh_exit(&ctx->fh);
	nvdec_decode_context_destroy(ctx->decode);
free_ctx:
	kfree(ctx);
unlock:
	mutex_unlock(&nvdec->lock);
	return err;
}

static int nvdec_release(struct file *file)
{
	struct nvdec_v4l2_ctx *ctx = file_to_nvdec_ctx(file);
	struct nvdec_v4l2 *nvdec = ctx->nvdec;

	v4l2_fh_del(&ctx->fh, file);
	mutex_lock(&nvdec->lock);
	list_del(&ctx->list);
	/* The worker finishing its last picture may still use the context. */
	v4l2_m2m_streamoff(file, ctx->fh.m2m_ctx,
			   V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	flush_work(&nvdec->run_work);
	flush_work(&nvdec->done_work);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	nvdec_release_surfaces(ctx);
	nvdec_decode_context_destroy(ctx->decode);
	mutex_unlock(&nvdec->lock);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
	nvdec_free_ctrl_copies(ctx);
	v4l2_fh_exit(&ctx->fh);
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

/* The last close of a node that was unregistered with files open lands here. */
static void nvdec_device_release(struct video_device *vdev)
{
	struct nvdec_v4l2 *nvdec = container_of(vdev, struct nvdec_v4l2, vdev);

	v4l2_device_unregister(&nvdec->v4l2_dev);
	v4l2_m2m_release(nvdec->m2m_dev);
	media_device_cleanup(&nvdec->mdev);
	mutex_destroy(&nvdec->lock);
	kfree(nvdec);
}

struct nvdec_v4l2 *nvdec_v4l2_register(struct tegra_drm_client *client,
				       struct host1x_channel *channel)
{
	struct nvdec_v4l2 *nvdec;
	int err;

	nvdec = kzalloc_obj(*nvdec);
	if (!nvdec)
		return ERR_PTR(-ENOMEM);
	nvdec->client = client;
	nvdec->channel = channel;
	nvdec->dev = client->base.dev;
	mutex_init(&nvdec->lock);
	INIT_LIST_HEAD(&nvdec->contexts);
	INIT_WORK(&nvdec->run_work, nvdec_run_work);
	INIT_WORK(&nvdec->done_work, nvdec_done_work);

	nvdec->mdev.dev = nvdec->dev;
	media_device_init(&nvdec->mdev);
	nvdec->mdev.ops = &nvdec_media_ops;
	strscpy(nvdec->mdev.model, "Tegra NVDEC", sizeof(nvdec->mdev.model));
	nvdec->v4l2_dev.mdev = &nvdec->mdev;

	err = v4l2_device_register(nvdec->dev, &nvdec->v4l2_dev);
	if (err)
		goto cleanup_media;

	nvdec->m2m_dev = v4l2_m2m_init(&nvdec_m2m_ops);
	if (IS_ERR(nvdec->m2m_dev)) {
		err = PTR_ERR(nvdec->m2m_dev);
		goto unregister_v4l2;
	}

	video_set_drvdata(&nvdec->vdev, nvdec);
	nvdec->vdev.v4l2_dev = &nvdec->v4l2_dev;
	nvdec->vdev.fops = &nvdec_fops;
	nvdec->vdev.ioctl_ops = &nvdec_ioctl_ops;
	nvdec->vdev.lock = &nvdec->lock;
	nvdec->vdev.vfl_dir = VFL_DIR_M2M;
	nvdec->vdev.release = nvdec_device_release;
	nvdec->vdev.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	strscpy(nvdec->vdev.name, "tegra-nvdec", sizeof(nvdec->vdev.name));
	err = video_register_device(&nvdec->vdev, VFL_TYPE_VIDEO, -1);
	if (err)
		goto release_m2m;

	/* From here on files may be open: unwind as an unbind would. */
	err = v4l2_m2m_register_media_controller(nvdec->m2m_dev, &nvdec->vdev,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (err)
		goto unregister;
	nvdec->mc_registered = true;
	err = media_device_register(&nvdec->mdev);
	if (err)
		goto unregister;

	v4l2_info(&nvdec->v4l2_dev, "registered /dev/video%d\n", nvdec->vdev.num);
	return nvdec;

unregister:
	nvdec_v4l2_unregister(nvdec);
	return ERR_PTR(err);
release_m2m:
	v4l2_m2m_release(nvdec->m2m_dev);
unregister_v4l2:
	v4l2_device_unregister(&nvdec->v4l2_dev);
cleanup_media:
	media_device_cleanup(&nvdec->mdev);
	mutex_destroy(&nvdec->lock);
	kfree(nvdec);
	return ERR_PTR(err);
}

/* Across system sleep: no picture may be submitted while the engines are off. */
void nvdec_v4l2_suspend(struct nvdec_v4l2 *nvdec)
{
	if (!nvdec)
		return;

	/* Close frees the context that v4l2_m2m_suspend() waits on. */
	mutex_lock(&nvdec->lock);
	v4l2_m2m_suspend(nvdec->m2m_dev);
	flush_work(&nvdec->run_work);
	flush_work(&nvdec->done_work);
	mutex_unlock(&nvdec->lock);
}

void nvdec_v4l2_resume(struct nvdec_v4l2 *nvdec)
{
	if (nvdec)
		v4l2_m2m_resume(nvdec->m2m_dev);
}

/* The engine is going away: files may stay open, but nothing reaches it again. */
void nvdec_v4l2_unregister(struct nvdec_v4l2 *nvdec)
{
	struct nvdec_v4l2_ctx *ctx;

	if (!nvdec)
		return;

	mutex_lock(&nvdec->lock);
	nvdec->disconnected = true;
	mutex_unlock(&nvdec->lock);

	get_device(&nvdec->vdev.dev);
	media_device_unregister(&nvdec->mdev);
	if (nvdec->mc_registered)
		v4l2_m2m_unregister_media_controller(nvdec->m2m_dev);
	video_unregister_device(&nvdec->vdev);

	/* No new ioctl gets past the core; let the picture in flight finish. */
	mutex_lock(&nvdec->lock);
	v4l2_m2m_suspend(nvdec->m2m_dev);
	flush_work(&nvdec->run_work);
	flush_work(&nvdec->done_work);
	list_for_each_entry(ctx, &nvdec->contexts, list) {
		nvdec_release_surfaces(ctx);
		nvdec_decode_context_destroy(ctx->decode);
		ctx->decode = NULL;
	}
	mutex_unlock(&nvdec->lock);

	put_device(&nvdec->vdev.dev);
}
