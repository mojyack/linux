// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, NVIDIA Corporation.
 */

#include <linux/host1x.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "drm.h"
#include "vic-engine.h"

/* The DRM-facing half of VIC; the engine itself lives in vic-engine.c. */
static int vic_drm_open_channel(struct tegra_drm_client *client,
				struct tegra_drm_context *context)
{
	return vic_engine_open_channel(vic_engine_from_drm_client(client),
				       context);
}

static void vic_drm_close_channel(struct tegra_drm_context *context)
{
	vic_engine_close_channel(context);
}

static int vic_drm_can_use_memory_ctx(struct tegra_drm_client *client,
				      bool *supported)
{
	return vic_engine_can_use_context(vic_engine_from_drm_client(client),
					  supported);
}

static const struct tegra_drm_client_ops vic_drm_ops = {
	.open_channel = vic_drm_open_channel,
	.close_channel = vic_drm_close_channel,
	.submit = tegra_drm_submit,
	.get_streamid_offset = tegra_drm_get_streamid_offset_thi,
	.can_use_memory_ctx = vic_drm_can_use_memory_ctx,
};

static int vic_probe(struct platform_device *pdev)
{
	struct tegra_drm_client *client;
	struct vic_engine *engine;

	engine = vic_engine_probe(pdev);
	if (IS_ERR(engine))
		return PTR_ERR(engine);

	client = vic_engine_drm_client(engine);
	INIT_LIST_HEAD(&client->list);
	client->version = vic_engine_version(engine);
	client->ops = &vic_drm_ops;

	return vic_engine_register(engine);
}

static void vic_remove(struct platform_device *pdev)
{
	vic_engine_unregister(platform_get_drvdata(pdev));
}

struct platform_driver tegra_vic_driver = {
	.driver = {
		.name = "tegra-vic",
		.of_match_table = vic_engine_of_match,
		.pm = &vic_engine_pm_ops
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
