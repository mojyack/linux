/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef TEGRA_NVDEC_ENGINE_H
#define TEGRA_NVDEC_ENGINE_H

#include <linux/of.h>
#include <linux/pm.h>

struct platform_device;
struct tegra_drm_client;
struct tegra_drm_context;

struct nvdec_engine;

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

int nvdec_engine_open_channel(struct nvdec_engine *engine,
			      struct tegra_drm_context *context);
void nvdec_engine_close_channel(struct tegra_drm_context *context);

extern const struct dev_pm_ops nvdec_engine_pm_ops;
extern const struct of_device_id nvdec_engine_of_match[];

#endif
