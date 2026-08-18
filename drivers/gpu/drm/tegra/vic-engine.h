/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef TEGRA_VIC_ENGINE_H
#define TEGRA_VIC_ENGINE_H

#include <linux/of.h>
#include <linux/pm.h>

struct platform_device;
struct tegra_drm_client;
struct tegra_drm_context;

struct vic_engine;

#define NVIDIA_TEGRA_124_VIC_FIRMWARE "nvidia/tegra124/vic03_ucode.bin"
#define NVIDIA_TEGRA_210_VIC_FIRMWARE "nvidia/tegra210/vic04_ucode.bin"
#define NVIDIA_TEGRA_186_VIC_FIRMWARE "nvidia/tegra186/vic04_ucode.bin"
#define NVIDIA_TEGRA_194_VIC_FIRMWARE "nvidia/tegra194/vic.bin"
#define NVIDIA_TEGRA_234_VIC_FIRMWARE "nvidia/tegra234/vic.bin"
#define NVIDIA_TEGRA_264_VIC_FIRMWARE "nvidia/tegra264/vic.bin"
#define NVIDIA_TEGRA_264_VIC_DESC "nvidia/tegra264/vic.bin.desc"

struct vic_engine *vic_engine_probe(struct platform_device *pdev);
int vic_engine_register(struct vic_engine *engine);
void vic_engine_unregister(struct vic_engine *engine);

struct tegra_drm_client *vic_engine_drm_client(struct vic_engine *engine);
struct vic_engine *vic_engine_from_drm_client(struct tegra_drm_client *client);
unsigned int vic_engine_version(struct vic_engine *engine);

int vic_engine_open_channel(struct vic_engine *engine,
			    struct tegra_drm_context *context);
void vic_engine_close_channel(struct tegra_drm_context *context);
int vic_engine_can_use_context(struct vic_engine *engine, bool *supported);

extern const struct dev_pm_ops vic_engine_pm_ops;
extern const struct of_device_id vic_engine_of_match[];

#endif
