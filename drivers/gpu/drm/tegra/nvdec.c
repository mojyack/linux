// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2022, NVIDIA Corporation.
 */

#include <linux/host1x.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "drm.h"
#include "nvdec-engine.h"

/* The DRM-facing half of NVDEC; the engine itself lives in nvdec-engine.c. */
static int nvdec_drm_open_channel(struct tegra_drm_client *client,
				  struct tegra_drm_context *context)
{
	return nvdec_engine_open_channel(nvdec_engine_from_drm_client(client),
					 context);
}

static void nvdec_drm_close_channel(struct tegra_drm_context *context)
{
	nvdec_engine_close_channel(context);
}

static int nvdec_drm_can_use_memory_ctx(struct tegra_drm_client *client,
					bool *supported)
{
	*supported = true;

	return 0;
}

static const struct tegra_drm_client_ops nvdec_drm_ops = {
	.open_channel = nvdec_drm_open_channel,
	.close_channel = nvdec_drm_close_channel,
	.submit = tegra_drm_submit,
	.get_streamid_offset = tegra_drm_get_streamid_offset_thi,
	.can_use_memory_ctx = nvdec_drm_can_use_memory_ctx,
};

static int nvdec_probe(struct platform_device *pdev)
{
	struct nvdec_engine *engine;
	struct tegra_drm_client *client;

	engine = nvdec_engine_probe(pdev);
	if (IS_ERR(engine))
		return PTR_ERR(engine);

	client = nvdec_engine_drm_client(engine);
	INIT_LIST_HEAD(&client->list);
	client->version = nvdec_engine_version(engine);
	client->ops = &nvdec_drm_ops;

	return nvdec_engine_register(engine);
}

static void nvdec_remove(struct platform_device *pdev)
{
	nvdec_engine_unregister(platform_get_drvdata(pdev));
}

struct platform_driver tegra_nvdec_driver = {
	.driver = {
		.name = "tegra-nvdec",
		.of_match_table = nvdec_engine_of_match,
		.pm = &nvdec_engine_pm_ops,
	},
	.probe = nvdec_probe,
	.remove = nvdec_remove,
};

#if IS_ENABLED(CONFIG_ARCH_TEGRA_210_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_210_NVDEC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_186_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_186_NVDEC_FIRMWARE);
#endif
#if IS_ENABLED(CONFIG_ARCH_TEGRA_194_SOC)
MODULE_FIRMWARE(NVIDIA_TEGRA_194_NVDEC_FIRMWARE);
#endif
