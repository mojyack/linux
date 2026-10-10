// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, NVIDIA Corporation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/host1x.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include <soc/tegra/pmc.h>

#include "drm.h"
#include "falcon.h"
#include "vic.h"

#define VIC_FALCON_DEBUGINFO			0x1094
#define VIC_DEBUGINFO_DUMMY			0xabcd1234
#define VIC_DEBUGINFO_CLEAR			0x0

struct vic_config {
	const char *firmware;
	unsigned int version;
	bool supports_sid;
	bool has_riscv;
	unsigned int transcfg_offset;
};

struct vic {
	struct falcon falcon;

	void __iomem *regs;
	struct tegra_drm_client client;
	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct reset_control *rst;
	/* With a power domain, the reset it shares, held while active. */
	struct reset_control *reset;
	/* Between late_init() and early_exit(): firmware may be loaded. */
	bool ready;
	/* Active when early_exit() suspended it: resumed again by late_init(). */
	bool resume_at_init;
	/* Out of reset and clocked. */
	bool active;
	struct mutex lock;	/* orders @active against job-timeout recovery */

	bool can_use_context;

	/* Platform configuration */
	const struct vic_config *config;
};

static inline struct vic *to_vic(struct tegra_drm_client *client)
{
	return container_of(client, struct vic, client);
}

static void vic_writel(struct vic *vic, u32 value, unsigned int offset)
{
	writel(value, vic->regs + offset);
}

static int vic_boot(struct vic *vic)
{
	u32 stream_id;
	u32 val;
	int err = 0;

	if (vic->config->supports_sid && tegra_dev_iommu_get_stream_id(vic->dev, &stream_id)) {
		u32 value;

		value = TRANSCFG_ATT(1, TRANSCFG_SID_FALCON) |
			TRANSCFG_ATT(0, TRANSCFG_SID_HW);
		vic_writel(vic, value, vic->config->transcfg_offset);

		/*
		 * STREAMID0 is used for input/output buffers. Initialize it to SID_VIC in case
		 * context isolation is not enabled, and SID_VIC is used for both firmware and
		 * data buffers.
		 *
		 * If context isolation is enabled, it will be overridden by the SETSTREAMID
		 * opcode as part of each job.
		 */
		vic_writel(vic, stream_id, VIC_THI_STREAMID0);

		/* STREAMID1 is used for firmware loading. */
		vic_writel(vic, stream_id, VIC_THI_STREAMID1);
	}

	/* setup clockgating registers */
	vic_writel(vic, CG_IDLE_CG_DLY_CNT(4) |
			CG_IDLE_CG_EN |
			CG_WAKEUP_DLY_CNT(4),
		   NV_PVIC_MISC_PRI_VIC_CG);

	if (vic->config->has_riscv) {
		/* Write a known pattern into DEBUGINFO register */
		vic_writel(vic, VIC_DEBUGINFO_DUMMY, VIC_FALCON_DEBUGINFO);
	}

	err = falcon_boot(&vic->falcon);
	if (err < 0)
		return err;

	if (vic->config->has_riscv) {
		/* Check VIC has reached a proper initialized state */
		err = readl_poll_timeout(vic->regs + VIC_FALCON_DEBUGINFO, val,
					 val == VIC_DEBUGINFO_CLEAR,
					 1000, 2000000);
		if (err) {
			dev_err(vic->dev, "VIC not initialized, timeout, val=0x%x\n", val);
			return err;
		}
	} else {
		u32 fce_ucode_size, fce_bin_data_offset;
		dma_addr_t iova;
		void *hdr;

		iova = vic->falcon.firmware.iova;
		hdr = vic->falcon.firmware.virt;
		fce_bin_data_offset = *(u32 *)(hdr + VIC_UCODE_FCE_DATA_OFFSET);

		/* Old VIC firmware needs kernel help with setting up FCE microcode. */
		if (fce_bin_data_offset != 0x0 && fce_bin_data_offset != 0xa5a5a5a5) {
			hdr = vic->falcon.firmware.virt +
				*(u32 *)(hdr + VIC_UCODE_FCE_HEADER_OFFSET);
			fce_ucode_size = *(u32 *)(hdr + FCE_UCODE_SIZE_OFFSET);

			falcon_execute_method(&vic->falcon, VIC_SET_FCE_UCODE_SIZE,
					      fce_ucode_size);
			falcon_execute_method(&vic->falcon, VIC_SET_FCE_UCODE_OFFSET,
					      (iova + fce_bin_data_offset) >> 8);
		}

		err = falcon_wait_idle(&vic->falcon);
		if (err < 0) {
			dev_err(vic->dev,
				"failed to set application ID and FCE base\n");
			return err;
		}
	}

	return 0;
}

static int vic_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	struct vic *vic = to_vic(drm);
	int err;

	err = host1x_client_iommu_attach(client);
	if (err < 0 && err != -ENODEV) {
		dev_err(vic->dev, "failed to attach to domain: %d\n", err);
		return err;
	}

	client->syncpts[0] = host1x_syncpt_request(client, 0);
	if (!client->syncpts[0]) {
		err = -ENOMEM;
		goto detach;
	}

	err = tegra_drm_register_client(tegra, drm);
	if (err < 0)
		goto free_syncpt;

	/* Legacy and CHANNEL_OPEN jobs share it: nothing else orders them. */
	vic->channel = host1x_channel_get(drm->shared_channel);

	/*
	 * Inherit the DMA parameters (such as maximum segment size) from the
	 * parent host1x device.
	 */
	client->dev->dma_parms = client->host->dma_parms;

	return 0;

free_syncpt:
	host1x_syncpt_put(client->syncpts[0]);
detach:
	host1x_client_iommu_detach(client);

	return err;
}

static int vic_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	struct vic *vic = to_vic(drm);
	int err;

	/* avoid a dangling pointer just in case this disappears */
	client->dev->dma_parms = NULL;

	err = tegra_drm_unregister_client(tegra, drm);
	if (err < 0)
		return err;

	/* Before detaching, which forgets the group the firmware was allocated for. */
	if (vic->falcon.firmware.virt) {
		if (client->group) {
			dma_unmap_single(vic->dev, vic->falcon.firmware.phys,
					 vic->falcon.firmware.size, DMA_TO_DEVICE);
			tegra_drm_free(tegra, vic->falcon.firmware.size,
				       vic->falcon.firmware.virt,
				       vic->falcon.firmware.iova);
		} else {
			dma_free_coherent(vic->dev, vic->falcon.firmware.size,
					  vic->falcon.firmware.virt,
					  vic->falcon.firmware.iova);
		}
		vic->falcon.firmware.virt = NULL;
	}

	host1x_syncpt_put(client->syncpts[0]);
	host1x_channel_put(vic->channel);
	host1x_client_iommu_detach(client);

	vic->channel = NULL;

	return 0;
}

/* A job timed out and its channel is stopped: hold the engine in reset. */
static int vic_quiesce(struct host1x_client *client)
{
	struct vic *vic = to_vic(host1x_to_drm_client(client));
	int err = 0;

	/* An inactive engine is stopped and booted on its next resume. */
	mutex_lock(&vic->lock);
	/* Acquired for as long as the engine is active, see runtime resume. */
	if (vic->active)
		err = reset_control_assert(vic->rst ?: vic->reset);
	mutex_unlock(&vic->lock);

	return err;
}

static int vic_restore(struct host1x_client *client)
{
	struct vic *vic = to_vic(host1x_to_drm_client(client));
	struct reset_control *rst = vic->rst ?: vic->reset;
	int err = 0;

	mutex_lock(&vic->lock);
	if (vic->active) {
		usleep_range(10, 20);
		err = reset_control_deassert(rst);
		if (!err) {
			usleep_range(10, 20);
			err = vic_boot(vic);
			/* Left in reset, for the next attempt. */
			if (err)
				reset_control_assert(rst);
		}
	}
	mutex_unlock(&vic->lock);

	return err;
}

static const struct host1x_client_ops vic_client_ops = {
	.init = vic_init,
	.exit = vic_exit,
};

/* Only an engine that can be reset is safe to recover from a job timeout. */
static const struct host1x_client_ops vic_client_ops_reset = {
	.init = vic_init,
	.exit = vic_exit,
	.quiesce = vic_quiesce,
	.restore = vic_restore,
};

static int vic_load_firmware(struct vic *vic)
{
	struct host1x_client *client = &vic->client.base;
	struct tegra_drm *tegra = vic->client.drm;
	static DEFINE_MUTEX(lock);
	u32 fce_bin_data_offset;
	dma_addr_t iova;
	size_t size;
	void *virt;
	int err;

	mutex_lock(&lock);

	if (vic->falcon.firmware.virt) {
		err = 0;
		goto unlock;
	}

	err = falcon_read_firmware(&vic->falcon, vic->config->firmware);
	if (err < 0)
		goto unlock;

	size = vic->falcon.firmware.size;

	if (!client->group) {
		virt = dma_alloc_coherent(vic->dev, size, &iova, GFP_KERNEL);
		if (!virt) {
			err = -ENOMEM;
			goto unlock;
		}
	} else {
		virt = tegra_drm_alloc(tegra, size, &iova);
		if (IS_ERR(virt)) {
			err = PTR_ERR(virt);
			goto unlock;
		}
	}

	vic->falcon.firmware.virt = virt;
	vic->falcon.firmware.iova = iova;

	err = falcon_load_firmware(&vic->falcon);
	if (err < 0)
		goto cleanup;

	/*
	 * In this case we have received an IOVA from the shared domain, so we
	 * need to make sure to get the physical address so that the DMA API
	 * knows what memory pages to flush the cache for.
	 */
	if (client->group) {
		dma_addr_t phys;

		phys = dma_map_single(vic->dev, virt, size, DMA_TO_DEVICE);

		err = dma_mapping_error(vic->dev, phys);
		if (err < 0)
			goto cleanup;

		vic->falcon.firmware.phys = phys;
	}

	/*
	 * Check if firmware is new enough to not require mapping firmware
	 * to data buffer domains.
	 */
	fce_bin_data_offset = *(u32 *)(virt + VIC_UCODE_FCE_DATA_OFFSET);

	if (!vic->config->supports_sid) {
		vic->can_use_context = false;
	} else if (vic->config->has_riscv) {
		vic->can_use_context = true;
	} else if (fce_bin_data_offset != 0x0 && fce_bin_data_offset != 0xa5a5a5a5) {
		/*
		 * Firmware will access FCE through STREAMID0, so context
		 * isolation cannot be used.
		 */
		vic->can_use_context = false;
		dev_warn_once(vic->dev, "context isolation disabled due to old firmware\n");
	} else {
		vic->can_use_context = true;
	}

unlock:
	mutex_unlock(&lock);
	return err;

cleanup:
	if (!client->group)
		dma_free_coherent(vic->dev, size, virt, iova);
	else
		tegra_drm_free(tegra, size, virt, iova);

	mutex_unlock(&lock);
	return err;
}

static int __maybe_unused vic_runtime_resume(struct device *dev)
{
	struct vic *vic = dev_get_drvdata(dev);
	int err;

	/* Not without the IOMMU domain, which a system resume may try. */
	if (!vic->ready)
		return -ENODEV;

	/* It allocates, which job-timeout recovery must not wait for. */
	err = vic_load_firmware(vic);
	if (err < 0)
		return err;

	err = clk_prepare_enable(vic->clk);
	if (err < 0)
		return err;

	usleep_range(10, 20);

	/* Held while active, so that a job timeout can reset the engine. */
	err = reset_control_acquire(vic->reset);
	if (err < 0)
		goto disable;

	mutex_lock(&vic->lock);

	/* Also the domain's: a failed restore may have left it asserted. */
	err = reset_control_deassert(vic->rst ?: vic->reset);
	if (err < 0)
		goto unlock;

	usleep_range(10, 20);

	err = vic_boot(vic);
	if (err < 0)
		goto assert;

	vic->active = true;
	mutex_unlock(&vic->lock);

	return 0;

assert:
	reset_control_assert(vic->rst);
unlock:
	mutex_unlock(&vic->lock);
	reset_control_release(vic->reset);
disable:
	clk_disable_unprepare(vic->clk);
	return err;
}

static void vic_power_off(struct vic *vic)
{
	reset_control_release(vic->reset);

	usleep_range(2000, 4000);

	clk_disable_unprepare(vic->clk);
}

static int __maybe_unused vic_runtime_suspend(struct device *dev)
{
	struct vic *vic = dev_get_drvdata(dev);
	int err;

	/* Waits for the channel, so before the lock that its recovery takes. */
	host1x_channel_stop(vic->channel);

	mutex_lock(&vic->lock);
	err = reset_control_assert(vic->rst);
	if (!err)
		vic->active = false;
	mutex_unlock(&vic->lock);
	if (err < 0)
		return err;

	vic_power_off(vic);

	return 0;
}

static int vic_open_channel(struct tegra_drm_client *client,
			    struct tegra_drm_context *context)
{
	struct vic *vic = to_vic(client);

	context->channel = host1x_channel_get(vic->channel);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void vic_close_channel(struct tegra_drm_context *context)
{
	host1x_channel_put(context->channel);
}

static int vic_can_use_memory_ctx(struct tegra_drm_client *client, bool *supported)
{
	struct vic *vic = to_vic(client);
	int err;

	/* This doesn't access HW so it's safe to call without powering up. */
	err = vic_load_firmware(vic);
	if (err < 0)
		return err;

	*supported = vic->can_use_context;

	return 0;
}

/* VicConfigStruct 4.0; only what the detile stage touches is broken out. */
struct vic_pipe_config {
	__le32 downsample;		/* horiz | vert << 16, U9.2 */
	u8 reserved[0x0c];
};

struct vic_output_config {
	__le32 alpha_fill;		/* AlphaFillMode | slot << 3 | background */
	u8 reserved0[0x04];		/* background, regamma */
	__le32 target_rect_h;		/* left | right << 16 */
	__le32 target_rect_v;		/* top | bottom << 16 */
};

struct vic_surface_config {
	__le32 format;			/* pixel format, chroma loc, block kind */
	__le32 size;			/* width | height << 14 */
	__le32 luma;			/* luma width | height << 14 */
	__le32 chroma;			/* chroma width | height << 14 */
};

struct vic_slot_config {
	__le32 flags;			/* SlotEnable, CurrentFieldEnable */
	u8 reserved0[0x0c];
	__le32 clamp;			/* SoftClampLow | SoftClampHigh << 10 */
	__le32 alpha;			/* PlanarAlpha | ConstantAlpha << 10 */
	u8 reserved1[0x08];
	__le32 source_rect[4];		/* left, right, top, bottom, U14.16 */
	__le32 dest_rect[2];		/* left | right << 16, top | bottom << 16 */
	u8 reserved2[0x08];
};

struct vic_slot {
	struct vic_slot_config config;
	struct vic_surface_config surface;
	u8 luma_key[0x10];
	u8 color_matrix[0x20];
	u8 gamut_matrix[0x20];
	u8 blending[0x10];
};

struct vic_config_struct {
	struct vic_pipe_config pipe;
	struct vic_output_config output;
	struct vic_surface_config out_surface;
	u8 out_color_matrix[0x20];
	u8 clear_rect[0x40];
	struct vic_slot slot[8];
};

static_assert(sizeof(struct vic_slot_config) == 0x40);
static_assert(sizeof(struct vic_slot) == 0xb0);
static_assert(offsetof(struct vic_config_struct, out_surface) == 0x020);
static_assert(offsetof(struct vic_config_struct, slot) == 0x090);
static_assert(offsetof(struct vic_slot, surface) == 0x040);
static_assert(sizeof(struct vic_config_struct) == VIC_CONFIG_SIZE);

#define VIC_T_R5G6B5			10
#define VIC_T_A8R8G8B8			32
#define VIC_T_Y8___U8V8_N420		67
#define VIC_BLK_KIND_PITCH		0
#define VIC_BLK_KIND_GENERIC_16BX2	1
#define VIC_ALPHA_FILL_SOURCE_STREAM	3

#define VIC_SURFACE_FORMAT(fmt, loc_h, loc_v, kind, height, cache)	\
	((fmt) | ((loc_h) << 7) | ((loc_v) << 9) | ((kind) << 11) |	\
	 ((height) << 15) | ((cache) << 19))
#define VIC_SURFACE_DIMS(w, h)		(((w) - 1) | (((h) - 1) << 14))
#define VIC_SURFACE_DIMS_UNUSED		0x0fffffffu
#define VIC_RECT(low, high)		((low) | ((high) << 16))

#define VIC_CONTROL_GPTIMER_ON		BIT(0)
#define VIC_CONTROL_FALCON_CONTROL	BIT(8)
#define VIC_CONTROL_CONFIG_SIZE(x)	(((x) >> 4) << 16)
#define VIC_EXECUTE_AWAKEN		BIT(8)
#define VIC_APPLICATION_COMPOSITOR	1

void vic_fill_detile_config(void *config, const struct vic_detile *params)
{
	struct vic_config_struct *c = config;
	u32 w = params->width, h = ALIGN(params->height, 2);
	u32 dw = params->out_width, dh = ALIGN(params->out_height, 2);
	u32 left = params->left, top = params->top;
	u32 format = VIC_T_Y8___U8V8_N420, step = 1, alpha_fill = 0;
	u32 chroma_loc_vert = 1;
	u32 src_rows, dst_rows;

	/* P010 is detiled one plane at a time. */
	switch (params->pass) {
	case VIC_DETILE_NV12:
		break;
	case VIC_DETILE_P010_LUMA:
		format = VIC_T_R5G6B5;
		step = 2;
		chroma_loc_vert = 0;
		break;
	case VIC_DETILE_P010_CHROMA:
		format = VIC_T_A8R8G8B8;
		step = 4;
		chroma_loc_vert = 0;
		alpha_fill = VIC_ALPHA_FILL_SOURCE_STREAM;
		w /= 2;
		h /= 2;
		dw /= 2;
		dh /= 2;
		left /= 2;
		top /= 2;
		break;
	}
	src_rows = ALIGN(h, 32);
	dst_rows = dh;

	memset(c, 0, sizeof(*c));

	c->pipe.downsample = cpu_to_le32(VIC_RECT(1 << 2, 1 << 2));

	c->output.alpha_fill = cpu_to_le32(alpha_fill);
	c->output.target_rect_h = cpu_to_le32(VIC_RECT(0, dw - 1));
	c->output.target_rect_v = cpu_to_le32(VIC_RECT(0, dh - 1));

	c->out_surface.format = cpu_to_le32(VIC_SURFACE_FORMAT(format, 0, 0,
							       VIC_BLK_KIND_PITCH, 0, 0));
	c->out_surface.size = cpu_to_le32(VIC_SURFACE_DIMS(dw, dh));
	c->out_surface.luma = cpu_to_le32(VIC_SURFACE_DIMS(params->dst_stride / step,
							   dst_rows));
	c->out_surface.chroma = step > 1 ? cpu_to_le32(VIC_SURFACE_DIMS_UNUSED) :
			cpu_to_le32(VIC_SURFACE_DIMS(params->dst_stride / 2,
						     dst_rows / 2));

	c->slot[0].config.flags = cpu_to_le32(BIT(0) | BIT(8));
	c->slot[0].config.clamp = cpu_to_le32(1023 << 10);
	c->slot[0].config.alpha = cpu_to_le32(1023 | BIT(10));
	c->slot[0].config.source_rect[0] = cpu_to_le32(left << 16);
	c->slot[0].config.source_rect[1] = cpu_to_le32((left + dw - 1) << 16);
	c->slot[0].config.source_rect[2] = cpu_to_le32(top << 16);
	c->slot[0].config.source_rect[3] = cpu_to_le32((top + dh - 1) << 16);
	c->slot[0].config.dest_rect[0] = cpu_to_le32(VIC_RECT(0, dw - 1));
	c->slot[0].config.dest_rect[1] = cpu_to_le32(VIC_RECT(0, dh - 1));

	/* Chroma sited left, block-linear GOB height 2, 32Bx8 cache width. */
	c->slot[0].surface.format =
		cpu_to_le32(VIC_SURFACE_FORMAT(format, 0, chroma_loc_vert,
					       VIC_BLK_KIND_GENERIC_16BX2, 1, 1));
	c->slot[0].surface.size = cpu_to_le32(VIC_SURFACE_DIMS(w, h));
	c->slot[0].surface.luma = cpu_to_le32(VIC_SURFACE_DIMS(params->src_stride / step,
							       src_rows));
	c->slot[0].surface.chroma = step > 1 ? cpu_to_le32(VIC_SURFACE_DIMS_UNUSED) :
			cpu_to_le32(VIC_SURFACE_DIMS(params->src_stride / 2,
						     src_rows / 2));
}

#define VIC_METHOD_APPLICATION_ID	0x080
#define VIC_METHOD_EXECUTE		0x0c0
#define VIC_METHOD_SURFACE0_LUMA	0x100
#define VIC_METHOD_SURFACE0_CHROMA_U	0x101
#define VIC_METHOD_CONTROL_PARAMS	0x1c1
#define VIC_METHOD_CONFIG_STRUCT	0x1c2
#define VIC_METHOD_FILTER_STRUCT	0x1c3
#define VIC_METHOD_OUT_LUMA		0x1c8
#define VIC_METHOD_OUT_CHROMA_U		0x1c9

void vic_emit_detile(struct falcon_gather *g, dma_addr_t config,
		     dma_addr_t filter, dma_addr_t src_luma, dma_addr_t src_chroma,
		     dma_addr_t dst_luma, dma_addr_t dst_chroma)
{
	falcon_gather_method(g, VIC_METHOD_APPLICATION_ID,
			     VIC_APPLICATION_COMPOSITOR);
	falcon_gather_method(g, VIC_METHOD_CONTROL_PARAMS,
			     VIC_CONTROL_CONFIG_SIZE(VIC_CONFIG_SIZE) |
			     VIC_CONTROL_FALCON_CONTROL | VIC_CONTROL_GPTIMER_ON);
	falcon_gather_address(g, VIC_METHOD_CONFIG_STRUCT, config);
	falcon_gather_address(g, VIC_METHOD_FILTER_STRUCT, filter);
	falcon_gather_address(g, VIC_METHOD_SURFACE0_LUMA, src_luma);
	falcon_gather_address(g, VIC_METHOD_SURFACE0_CHROMA_U, src_chroma);
	falcon_gather_address(g, VIC_METHOD_OUT_LUMA, dst_luma);
	falcon_gather_address(g, VIC_METHOD_OUT_CHROMA_U, dst_chroma);
	falcon_gather_method(g, VIC_METHOD_EXECUTE, VIC_EXECUTE_AWAKEN);
}

/* The VIC that takes VicConfigStruct 4.0, which is what the helpers above fill. */
struct tegra_drm_client *vic_find_detiler(struct tegra_drm *tegra)
{
	struct tegra_drm_client *client, *found = NULL;

	mutex_lock(&tegra->clients_lock);
	list_for_each_entry(client, &tegra->clients, list) {
		if (client->base.class == HOST1X_CLASS_VIC &&
		    client->version == 0x21) {
			found = client;
			break;
		}
	}
	mutex_unlock(&tegra->clients_lock);

	return found;
}

/* Firmware may only be loaded once the IOMMU carveout exists. */
static void vic_late_init(struct tegra_drm_client *client)
{
	struct vic *vic = to_vic(client);
	int err;

	vic->ready = true;
	pm_runtime_use_autosuspend(vic->dev);
	pm_runtime_set_autosuspend_delay(vic->dev, 500);
	pm_runtime_enable(vic->dev);

	if (vic->resume_at_init) {
		vic->resume_at_init = false;
		err = pm_runtime_resume(vic->dev);
		if (err < 0)
			dev_err(vic->dev, "failed to resume: %d\n", err);
	}
}

/* Leaves the engine off and runtime PM disabled, whoever holds a reference. */
static void vic_early_exit(struct tegra_drm_client *client)
{
	struct vic *vic = to_vic(client);
	int err;

	vic->ready = false;
	pm_runtime_dont_use_autosuspend(vic->dev);
	pm_runtime_disable(vic->dev);
	if (pm_runtime_status_suspended(vic->dev))
		return;

	err = vic_runtime_suspend(vic->dev);
	if (err < 0) {
		dev_err(vic->dev, "failed to suspend: %d\n", err);
		mutex_lock(&vic->lock);
		vic->active = false;
		mutex_unlock(&vic->lock);
		vic_power_off(vic);
	}
	pm_runtime_set_suspended(vic->dev);
	vic->resume_at_init = true;
}

static const struct tegra_drm_client_ops vic_ops = {
	.open_channel = vic_open_channel,
	.close_channel = vic_close_channel,
	.submit = tegra_drm_submit,
	.get_streamid_offset = tegra_drm_get_streamid_offset_thi,
	.can_use_memory_ctx = vic_can_use_memory_ctx,
	.late_init = vic_late_init,
	.early_exit = vic_early_exit,
};

#define NVIDIA_TEGRA_124_VIC_FIRMWARE "nvidia/tegra124/vic03_ucode.bin"

static const struct vic_config vic_t124_config = {
	.firmware = NVIDIA_TEGRA_124_VIC_FIRMWARE,
	.version = 0x40,
	.supports_sid = false,
};

#define NVIDIA_TEGRA_210_VIC_FIRMWARE "nvidia/tegra210/vic04_ucode.bin"

static const struct vic_config vic_t210_config = {
	.firmware = NVIDIA_TEGRA_210_VIC_FIRMWARE,
	.version = 0x21,
	.supports_sid = false,
};

#define NVIDIA_TEGRA_186_VIC_FIRMWARE "nvidia/tegra186/vic04_ucode.bin"

static const struct vic_config vic_t186_config = {
	.firmware = NVIDIA_TEGRA_186_VIC_FIRMWARE,
	.version = 0x18,
	.supports_sid = true,
	.transcfg_offset = 0x2044,
};

#define NVIDIA_TEGRA_194_VIC_FIRMWARE "nvidia/tegra194/vic.bin"

static const struct vic_config vic_t194_config = {
	.firmware = NVIDIA_TEGRA_194_VIC_FIRMWARE,
	.version = 0x19,
	.supports_sid = true,
	.transcfg_offset = 0x2044,
};

#define NVIDIA_TEGRA_234_VIC_FIRMWARE "nvidia/tegra234/vic.bin"

static const struct vic_config vic_t234_config = {
	.firmware = NVIDIA_TEGRA_234_VIC_FIRMWARE,
	.version = 0x23,
	.supports_sid = true,
	.transcfg_offset = 0x2044,
};

#define NVIDIA_TEGRA_264_VIC_FIRMWARE "nvidia/tegra264/vic.bin"
#define NVIDIA_TEGRA_264_VIC_DESC "nvidia/tegra264/vic.bin.desc"

static const struct vic_config vic_t264_config = {
	.firmware = NVIDIA_TEGRA_264_VIC_FIRMWARE,
	.version = 0x264,
	.supports_sid = true,
	.has_riscv = true,
	.transcfg_offset = 0x2244,
};

static const struct of_device_id tegra_vic_of_match[] = {
	{ .compatible = "nvidia,tegra124-vic", .data = &vic_t124_config },
	{ .compatible = "nvidia,tegra210-vic", .data = &vic_t210_config },
	{ .compatible = "nvidia,tegra186-vic", .data = &vic_t186_config },
	{ .compatible = "nvidia,tegra194-vic", .data = &vic_t194_config },
	{ .compatible = "nvidia,tegra234-vic", .data = &vic_t234_config },
	{ .compatible = "nvidia,tegra264-vic", .data = &vic_t264_config },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_vic_of_match);

static int vic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct vic *vic;
	int err;

	/* inherit DMA mask from host1x parent */
	err = dma_coerce_mask_and_coherent(dev, *dev->parent->dma_mask);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to set DMA mask: %d\n", err);
		return err;
	}

	vic = devm_kzalloc(dev, sizeof(*vic), GFP_KERNEL);
	if (!vic)
		return -ENOMEM;

	vic->config = of_device_get_match_data(dev);

	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	vic->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vic->regs))
		return PTR_ERR(vic->regs);

	vic->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(vic->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(vic->clk);
	}

	err = clk_set_rate(vic->clk, ULONG_MAX);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to set clock rate\n");
		return err;
	}

	if (!dev->pm_domain) {
		vic->rst = devm_reset_control_get(dev, "vic");
		if (IS_ERR(vic->rst)) {
			dev_err(&pdev->dev, "failed to get reset\n");
			return PTR_ERR(vic->rst);
		}
	} else {
		vic->reset = devm_reset_control_get_optional_exclusive_released(dev, "vic");
		if (IS_ERR(vic->reset))
			return dev_err_probe(dev, PTR_ERR(vic->reset),
					     "failed to get reset\n");
	}

	mutex_init(&vic->lock);

	vic->falcon.dev = dev;
	vic->falcon.regs = vic->regs;
	vic->falcon.riscv = vic->config->has_riscv;

	err = falcon_init(&vic->falcon);
	if (err < 0)
		return err;

	platform_set_drvdata(pdev, vic);

	INIT_LIST_HEAD(&vic->client.base.list);
	vic->client.base.ops = vic->rst || vic->reset ? &vic_client_ops_reset :
						      &vic_client_ops;
	vic->client.base.dev = dev;
	vic->client.base.class = HOST1X_CLASS_VIC;
	vic->client.base.syncpts = syncpts;
	vic->client.base.num_syncpts = 1;
	vic->dev = dev;

	INIT_LIST_HEAD(&vic->client.list);
	vic->client.version = vic->config->version;
	vic->client.ops = &vic_ops;

	err = host1x_client_register(&vic->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto exit_falcon;
	}

	return 0;

exit_falcon:
	falcon_exit(&vic->falcon);

	return err;
}

static void vic_remove(struct platform_device *pdev)
{
	struct vic *vic = platform_get_drvdata(pdev);

	host1x_client_unregister(&vic->client.base);
	falcon_exit(&vic->falcon);
}

static const struct dev_pm_ops vic_pm_ops = {
	RUNTIME_PM_OPS(vic_runtime_suspend, vic_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

struct platform_driver tegra_vic_driver = {
	.driver = {
		.name = "tegra-vic",
		.of_match_table = tegra_vic_of_match,
		.pm = &vic_pm_ops
	},
	.probe = vic_probe,
	.remove = vic_remove,
};

#if IS_ENABLED(CONFIG_ARCH_TEGRA_124_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_124_VIC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_210_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_210_VIC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_186_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_186_VIC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_194_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_194_VIC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_234_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_234_VIC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_264_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_264_VIC_FIRMWARE);
MODULE_FIRMWARE(NVIDIA_TEGRA_264_VIC_DESC);
#endif
