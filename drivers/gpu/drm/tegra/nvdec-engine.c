// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2022, NVIDIA Corporation.
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

#include <soc/tegra/mc.h>

#include "drm.h"
#include "falcon.h"
#include "nvdec-engine.h"
#include "riscv.h"
#include "vic.h"

#define NVDEC_FALCON_DEBUGINFO			0x1094
#define NVDEC_TFBIF_TRANSCFG			0x2c44

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

	host1x_channel_stop(engine->channel);
	clk_bulk_disable_unprepare(engine->num_clks, engine->clks);

	return 0;
}

const struct dev_pm_ops nvdec_engine_pm_ops = {
	SET_RUNTIME_PM_OPS(nvdec_engine_runtime_suspend,
			   nvdec_engine_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

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
