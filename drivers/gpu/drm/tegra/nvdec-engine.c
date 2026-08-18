// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2022, NVIDIA Corporation.
 */

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
#define NVDEC_H264_DONE_WORDS			2
#define NVDEC_H264_VIC_OFFSET			(NVDEC_H264_GATHER_WORDS + \
						 NVDEC_H264_DONE_WORDS)
#define NVDEC_H264_TOTAL_GATHER_WORDS		(NVDEC_H264_VIC_OFFSET + \
						 VIC_DETILE_WORDS + \
						 NVDEC_H264_DONE_WORDS)

#define NVDEC_H264_METHOD_INCR			0x10100002
#define NVDEC_H264_METHOD_APPLICATION		0x080
#define NVDEC_H264_METHOD_CONTROL		0x100
#define NVDEC_H264_METHOD_PICTURE_INDEX	0x103
#define NVDEC_H264_METHOD_SETUP		0x101
#define NVDEC_H264_METHOD_INPUT		0x102
#define NVDEC_H264_METHOD_SLICE_OFFSETS	0x104
#define NVDEC_H264_METHOD_STATUS		0x109
#define NVDEC_H264_METHOD_COLOC		0x105
#define NVDEC_H264_METHOD_MBHIST		0x140
#define NVDEC_H264_METHOD_HISTORY		0x106
#define NVDEC_H264_METHOD_LUMA			0x10c
#define NVDEC_H264_METHOD_CHROMA		0x11d
#define NVDEC_H264_METHOD_EXECUTE		0x0c0

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

struct nvdec_h264_buffer {
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

struct nvdec_h264_surface {
	struct nvdec_engine_map *map;
	u8 picture_index;
	/* H.264 setup slot, NVDEC_H264_DPB_ENTRIES while this is not a reference. */
	u8 dpb_slot;
};

struct nvdec_h264_context {
	struct kref ref;
	struct nvdec_engine *engine;
	/* Serializes staging and submission of this context's pictures. */
	struct mutex lock;
	struct nvdec_engine_map *scratch;
	struct nvdec_h264_buffer *input;
	u16 width_in_mbs;
	u16 height_in_mbs;
	u32 coloc_size;
	u32 mbhist_offset;
	u32 mbhist_size;
	u32 history_offset;
	u32 history_size;
	bool in_flight;
	/* Slices of the current picture, staged in ctx->input until the last. */
	u32 *slice_offsets;
	unsigned int slice_count;
	unsigned int max_slices;
	u32 staged;
	struct nvdec_h264_surface surfaces[NVDEC_H264_MAX_PICTURES];
};

struct nvdec_h264_job {
	struct nvdec_h264_context *ctx;
	struct nvdec_h264_request request;
	struct nvdec_h264_buffer *state;
	struct nvdec_h264_buffer *input;
	struct nvdec_h264_buffer *gather;
	struct nvdec_engine_map *scratch;
	struct nvdec_engine_map *surface;
	struct nvdec_engine_map *capture;
	struct nvdec_engine_map *dpb[NVDEC_H264_DPB_ENTRIES];
	struct vic_engine *vic;
	u32 slice_offsets_off;
	struct dma_fence *fence;
	nvdec_engine_h264_complete_t complete;
	void *complete_data;
	bool pinned;
	bool runtime_ref;
	bool vic_runtime_ref;
	bool submitted;
	bool completion_armed;
};

struct nvdec_h264_fence {
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

static struct host1x_bo *nvdec_h264_buffer_get(struct host1x_bo *bo)
{
	struct nvdec_h264_buffer *buffer = container_of(bo, struct nvdec_h264_buffer,
							  bo);

	kref_get(&buffer->ref);
	return bo;
}

static struct nvdec_h264_buffer *
nvdec_h264_buffer_ref(struct nvdec_h264_buffer *buffer)
{
	kref_get(&buffer->ref);
	return buffer;
}

static void nvdec_h264_buffer_release(struct kref *ref)
{
	struct nvdec_h264_buffer *buffer = container_of(ref, struct nvdec_h264_buffer,
							  ref);
	struct tegra_drm *tegra = nvdec_engine_tegra_iommu(buffer->engine);

	if (tegra)
		nvdec_engine_iommu_unmap(tegra, buffer->mm, buffer->mapped);
	dma_free_coherent(buffer->dev, buffer->size, buffer->cpu, buffer->dma);
	kfree(buffer);
}

static void nvdec_h264_buffer_put(struct host1x_bo *bo)
{
	struct nvdec_h264_buffer *buffer = container_of(bo, struct nvdec_h264_buffer,
							  bo);

	kref_put(&buffer->ref, nvdec_h264_buffer_release);
}

static struct host1x_bo_mapping *
nvdec_h264_buffer_pin(struct device *dev, struct host1x_bo *bo,
		      enum dma_data_direction direction)
{
	struct nvdec_h264_buffer *buffer = container_of(bo, struct nvdec_h264_buffer,
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

static void nvdec_h264_buffer_unpin(struct host1x_bo_mapping *map)
{
	dma_unmap_sgtable(map->dev, map->sgt, map->direction, 0);
	sg_free_table(map->sgt);
	kfree(map->sgt);
	host1x_bo_put(map->bo);
	kfree(map);
}

static void *nvdec_h264_buffer_mmap(struct host1x_bo *bo)
{
	return container_of(bo, struct nvdec_h264_buffer, bo)->cpu;
}

static void nvdec_h264_buffer_munmap(struct host1x_bo *bo, void *addr)
{
}

static const struct host1x_bo_ops nvdec_h264_buffer_ops = {
	.get = nvdec_h264_buffer_get,
	.put = nvdec_h264_buffer_put,
	.pin = nvdec_h264_buffer_pin,
	.unpin = nvdec_h264_buffer_unpin,
	.mmap = nvdec_h264_buffer_mmap,
	.munmap = nvdec_h264_buffer_munmap,
};

static struct nvdec_h264_buffer *
nvdec_h264_buffer_alloc(struct nvdec_engine *engine, size_t size)
{
	struct device *dev = engine->dev;
	struct nvdec_h264_buffer *buffer;
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

	host1x_bo_init(&buffer->bo, &nvdec_h264_buffer_ops);
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

static bool nvdec_h264_map_is_valid(const struct nvdec_engine_map *map,
				    enum dma_data_direction direction, size_t size)
{
	if (!map || map->size < size || upper_32_bits(map->iova))
		return false;

	return map->direction == direction || map->direction == DMA_BIDIRECTIONAL;
}

static int nvdec_h264_copy_output(struct nvdec_h264_buffer *input, u32 offset,
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

static const char *nvdec_h264_fence_get_driver_name(struct dma_fence *fence)
{
	return "tegra-nvdec";
}

static const char *nvdec_h264_fence_get_timeline_name(struct dma_fence *fence)
{
	return "tegra-nvdec-h264";
}

static void nvdec_h264_fence_release(struct dma_fence *fence)
{
	struct nvdec_h264_fence *h264_fence =
		container_of(fence, struct nvdec_h264_fence, base);

	kfree(h264_fence);
}

static const struct dma_fence_ops nvdec_h264_fence_ops = {
	.get_driver_name = nvdec_h264_fence_get_driver_name,
	.get_timeline_name = nvdec_h264_fence_get_timeline_name,
	.release = nvdec_h264_fence_release,
};

static struct dma_fence *
nvdec_h264_fence_create(struct nvdec_engine *engine)
{
	struct nvdec_h264_fence *h264_fence;

	h264_fence = kzalloc_obj(*h264_fence);
	if (!h264_fence)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&h264_fence->lock);
	dma_fence_init(&h264_fence->base, &nvdec_h264_fence_ops,
		       &h264_fence->lock, engine->h264_fence_context,
		       atomic64_inc_return(&engine->h264_fence_seqno));
	return &h264_fence->base;
}

static int nvdec_h264_install_fences(struct nvdec_h264_job *hjob)
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

static int nvdec_h264_prepare_scratch(struct nvdec_h264_context *ctx,
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

static int nvdec_h264_prepare_input(struct nvdec_h264_context *ctx, size_t size)
{
	struct nvdec_h264_buffer *input;

	if (ctx->input && ctx->input->size >= size)
		return 0;

	input = nvdec_h264_buffer_alloc(ctx->engine, ALIGN(size, SZ_4K));
	if (IS_ERR(input))
		return PTR_ERR(input);
	if (upper_32_bits(input->iova)) {
		nvdec_h264_buffer_put(&input->bo);
		return -ERANGE;
	}

	if (ctx->input) {
		memcpy(input->cpu, ctx->input->cpu, ctx->staged);
		nvdec_h264_buffer_put(&ctx->input->bo);
	}
	ctx->input = input;
	return 0;
}

/* Slices are staged back to back and described by slice_count + 1 offsets. */
int nvdec_engine_h264_stage_slice(struct nvdec_h264_context *ctx,
				  struct nvdec_engine_map *output,
				  u32 payload_size, bool first,
				  unsigned int max_slices)
{
	u32 staged;
	int err;

	if (!ctx || !output || payload_size < 3 || !max_slices)
		return -EINVAL;

	mutex_lock(&ctx->lock);
	if (first) {
		ctx->slice_count = 0;
		ctx->staged = 0;
	}
	staged = ctx->staged;

	if (ctx->slice_count >= max_slices ||
	    check_add_overflow(staged, payload_size, &staged) ||
	    staged > U32_MAX - 16 - SZ_256 ||
	    !nvdec_h264_map_is_valid(output, DMA_TO_DEVICE, payload_size)) {
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
	err = nvdec_h264_prepare_input(ctx, ALIGN(staged + 16, SZ_256) +
				       (ctx->slice_count + 2) * sizeof(u32));
	if (err)
		goto unlock;

	err = nvdec_h264_copy_output(ctx->input, staged - payload_size, output,
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

void nvdec_engine_h264_discard_slices(struct nvdec_h264_context *ctx)
{
	if (!ctx)
		return;

	mutex_lock(&ctx->lock);
	ctx->slice_count = 0;
	ctx->staged = 0;
	mutex_unlock(&ctx->lock);
}

/* The scratch is sized from the coded resolution, so a new size needs a new one. */
void nvdec_engine_h264_context_reset(struct nvdec_h264_context *ctx)
{
	if (!ctx)
		return;

	nvdec_engine_h264_discard_slices(ctx);
	mutex_lock(&ctx->lock);
	nvdec_engine_map_put(ctx->scratch);
	ctx->scratch = NULL;
	ctx->width_in_mbs = 0;
	ctx->height_in_mbs = 0;
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
	    !nvdec_h264_map_is_valid(surface, DMA_BIDIRECTIONAL, capture_size)) {
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
	    !nvdec_h264_map_is_valid(capture, DMA_FROM_DEVICE, dst_size)) {
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
		    !nvdec_h264_map_is_valid(dpb[i], DMA_TO_DEVICE, capture_size)) {
			dev_dbg(dev, "h264 reject: dpb %u\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static int nvdec_h264_surface_index(struct nvdec_h264_context *ctx,
				    struct nvdec_engine_map *map, u8 *index)
{
	unsigned int i;

	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++) {
		if (ctx->surfaces[i].map == map) {
			*index = ctx->surfaces[i].picture_index;
			return 0;
		}
	}

	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++) {
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
static int nvdec_h264_dpb_slot(struct nvdec_h264_context *ctx,
			       struct nvdec_engine_map *map, u8 *slot)
{
	struct nvdec_h264_surface *entry = NULL;
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

static void nvdec_h264_fill_setup(struct nvdec_h264_job *hjob, u8 current_index,
				  const u8 picture_indices[NVDEC_H264_DPB_ENTRIES],
				  const u8 dpb_slots[NVDEC_H264_DPB_ENTRIES])
{
	const struct nvdec_h264_request *request = &hjob->request;
	struct nvdec_h264_context *ctx = hjob->ctx;
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

static void nvdec_h264_emit_method(u32 *gather, unsigned int *word,
				   u32 method, u32 value)
{
	gather[(*word)++] = NVDEC_H264_METHOD_INCR;
	gather[(*word)++] = method;
	gather[(*word)++] = value;
}

static void nvdec_h264_debug_job(struct nvdec_h264_job *hjob, u8 current_index,
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
static int nvdec_h264_emit_address(u32 *gather, unsigned int *word,
				   unsigned int method, dma_addr_t iova)
{
	if (!IS_ALIGNED(iova, SZ_256) || upper_32_bits(iova >> 8))
		return -EINVAL;

	nvdec_h264_emit_method(gather, word, method, lower_32_bits(iova >> 8));
	return 0;
}

static int nvdec_h264_build_gather(struct nvdec_h264_job *hjob,
				   u8 current_index,
				   const u8 picture_indices[NVDEC_H264_DPB_ENTRIES],
				   const u8 dpb_slots[NVDEC_H264_DPB_ENTRIES])
{
	struct nvdec_h264_context *ctx = hjob->ctx;
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

	nvdec_h264_emit_method(gather, &word, NVDEC_H264_METHOD_APPLICATION, 3);
	nvdec_h264_emit_method(gather, &word, NVDEC_H264_METHOD_CONTROL, 0x53);
	nvdec_h264_emit_method(gather, &word, NVDEC_H264_METHOD_PICTURE_INDEX,
			       current_index);
	err = nvdec_h264_emit_address(gather, &word, NVDEC_H264_METHOD_SETUP,
				      hjob->state->iova);
	if (!err)
		err = nvdec_h264_emit_address(gather, &word, NVDEC_H264_METHOD_INPUT,
					      hjob->input->iova);
	if (!err)
		err = nvdec_h264_emit_address(gather, &word,
					      NVDEC_H264_METHOD_SLICE_OFFSETS,
					      hjob->input->iova +
					      hjob->slice_offsets_off);
	if (!err)
		err = nvdec_h264_emit_address(gather, &word, NVDEC_H264_METHOD_STATUS,
					      hjob->state->iova + NVDEC_H264_STATUS_OFFSET);
	if (!err)
		err = nvdec_h264_emit_address(gather, &word, NVDEC_H264_METHOD_COLOC,
					      hjob->scratch->iova);
	if (!err)
		err = nvdec_h264_emit_address(gather, &word, NVDEC_H264_METHOD_MBHIST,
					      hjob->scratch->iova + ctx->mbhist_offset);
	if (!err)
		err = nvdec_h264_emit_address(gather, &word, NVDEC_H264_METHOD_HISTORY,
					      hjob->scratch->iova + ctx->history_offset);
	for (i = 0; !err && i < NVDEC_H264_MAX_PICTURES; i++)
		err = nvdec_h264_emit_address(gather, &word,
					      NVDEC_H264_METHOD_LUMA + i,
					      references[i]->iova);
	for (i = 0; !err && i < NVDEC_H264_MAX_PICTURES; i++)
		err = nvdec_h264_emit_address(gather, &word,
					      NVDEC_H264_METHOD_CHROMA + i,
					      references[i]->iova +
					      hjob->request.chroma_offset);
	if (err)
		return err;
	nvdec_h264_emit_method(gather, &word, NVDEC_H264_METHOD_EXECUTE, 0x100);
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

static void nvdec_h264_context_release(struct kref *ref)
{
	struct nvdec_h264_context *ctx = container_of(ref, struct nvdec_h264_context,
						       ref);
	unsigned int i;

	for (i = 0; i < NVDEC_H264_MAX_PICTURES; i++)
		nvdec_engine_map_put(ctx->surfaces[i].map);
	if (ctx->input)
		nvdec_h264_buffer_put(&ctx->input->bo);
	nvdec_engine_map_put(ctx->scratch);
	kfree(ctx->slice_offsets);
	kfree(ctx);
}

static void nvdec_h264_job_release(struct host1x_job *job)
{
	struct nvdec_h264_job *hjob = job->user_data;
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
	nvdec_h264_buffer_put(&hjob->gather->bo);
	nvdec_h264_buffer_put(&hjob->input->bo);
	nvdec_h264_buffer_put(&hjob->state->bo);
	nvdec_engine_map_put(hjob->scratch);
	nvdec_engine_map_put(hjob->surface);
	nvdec_engine_map_put(hjob->capture);
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++)
		nvdec_engine_map_put(hjob->dpb[i]);
	if (job->cancelled)
		nvdec_engine_recover(hjob->ctx->engine);
	kref_put(&hjob->ctx->ref, nvdec_h264_context_release);
	kfree(hjob);
}

struct nvdec_h264_context *
nvdec_engine_h264_context_create(struct nvdec_engine *engine)
{
	struct nvdec_h264_context *ctx;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	kref_init(&ctx->ref);
	ctx->engine = engine;
	mutex_init(&ctx->lock);
	return ctx;
}

void nvdec_engine_h264_context_destroy(struct nvdec_h264_context *ctx)
{
	if (!ctx)
		return;

	kref_put(&ctx->ref, nvdec_h264_context_release);
}

void nvdec_engine_h264_context_release_surface(struct nvdec_h264_context *ctx,
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

int nvdec_engine_h264_submit(struct nvdec_h264_context *ctx,
			     const struct nvdec_h264_request *request,
			     struct nvdec_engine_map *surface,
			     struct nvdec_engine_map *capture,
			     struct nvdec_engine_map * const dpb[NVDEC_H264_DPB_ENTRIES],
			     struct dma_fence **fence,
			     nvdec_engine_h264_complete_t complete, void *data)
{
	static const u8 termination[16] = {
		0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00,
	};
	struct nvdec_h264_job *hjob;
	struct host1x_job *job;
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

	err = nvdec_h264_surface_index(ctx, surface, &current_index);
	if (err)
		goto free_hjob;
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++) {
		if (!hjob->request.dpb[i].valid)
			continue;
		err = nvdec_h264_surface_index(ctx, dpb[i], &picture_indices[i]);
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
	hjob->state = nvdec_h264_buffer_alloc(ctx->engine, NVDEC_H264_STATE_SIZE);
	if (IS_ERR(hjob->state)) {
		err = PTR_ERR(hjob->state);
		hjob->state = NULL;
		goto free_hjob;
	}
	hjob->input = nvdec_h264_buffer_ref(ctx->input);
	hjob->gather = nvdec_h264_buffer_alloc(ctx->engine,
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
	hjob->fence = nvdec_h264_fence_create(ctx->engine);
	if (IS_ERR(hjob->fence)) {
		err = PTR_ERR(hjob->fence);
		hjob->fence = NULL;
		goto free_hjob;
	}
	err = nvdec_h264_install_fences(hjob);
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

	job = host1x_job_alloc(ctx->engine->channel, 5, 0, true);
	if (!job) {
		err = -ENOMEM;
		goto free_hjob;
	}
	job->client = &ctx->engine->client.base;
	job->class = HOST1X_CLASS_NVDEC;
	job->serialize = true;
	job->syncpt = host1x_syncpt_get(ctx->engine->client.base.syncpts[0]);
	job->syncpt_incrs = 2;	/* NVDEC OP_DONE, then VIC OP_DONE */
	job->timeout = 10000;
	job->release = nvdec_h264_job_release;
	job->user_data = hjob;
	host1x_job_add_gather(job, &hjob->gather->bo, NVDEC_H264_GATHER_WORDS, 0);
	host1x_job_add_gather(job, &hjob->gather->bo, NVDEC_H264_DONE_WORDS,
			      NVDEC_H264_GATHER_WORDS * sizeof(u32));
	host1x_job_add_wait(job, host1x_syncpt_id(job->syncpt), 1, true,
			    HOST1X_CLASS_VIC);
	host1x_job_add_gather(job, &hjob->gather->bo, VIC_DETILE_WORDS,
			      NVDEC_H264_VIC_OFFSET * sizeof(u32));
	host1x_job_add_gather(job, &hjob->gather->bo, NVDEC_H264_DONE_WORDS,
			      (NVDEC_H264_VIC_OFFSET + VIC_DETILE_WORDS) *
			       sizeof(u32));

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
	mutex_unlock(&ctx->lock);
	return 0;

put_job:
	if (hjob->submitted) {
		ctx->in_flight = false;
		hjob->submitted = false;
	}
	host1x_job_put(job);
	mutex_unlock(&ctx->lock);
	return err;

free_hjob:
	if (hjob->fence) {
		dma_fence_set_error(hjob->fence, -EIO);
		dma_fence_signal(hjob->fence);
		dma_fence_put(hjob->fence);
	}
	if (hjob->gather)
		nvdec_h264_buffer_put(&hjob->gather->bo);
	if (hjob->input)
		nvdec_h264_buffer_put(&hjob->input->bo);
	if (hjob->state)
		nvdec_h264_buffer_put(&hjob->state->bo);
	nvdec_engine_map_put(hjob->scratch);
	nvdec_engine_map_put(hjob->surface);
	nvdec_engine_map_put(hjob->capture);
	for (i = 0; i < NVDEC_H264_DPB_ENTRIES; i++)
		nvdec_engine_map_put(hjob->dpb[i]);
	kref_put(&hjob->ctx->ref, nvdec_h264_context_release);
	kfree(hjob);
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
