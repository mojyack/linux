/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef TEGRA_NVDEC_H
#define TEGRA_NVDEC_H

#include <linux/kconfig.h>

struct host1x_channel;
struct nvdec_v4l2;
struct tegra_drm_client;

#if IS_ENABLED(CONFIG_DRM_TEGRA_NVDEC_V4L2)
struct nvdec_v4l2 *nvdec_v4l2_register(struct tegra_drm_client *client,
				       struct host1x_channel *channel);
void nvdec_v4l2_unregister(struct nvdec_v4l2 *nvdec);
void nvdec_v4l2_suspend(struct nvdec_v4l2 *nvdec);
void nvdec_v4l2_resume(struct nvdec_v4l2 *nvdec);
#else
static inline struct nvdec_v4l2 *
nvdec_v4l2_register(struct tegra_drm_client *client,
		    struct host1x_channel *channel)
{
	return NULL;
}

static inline void nvdec_v4l2_unregister(struct nvdec_v4l2 *nvdec)
{
}

static inline void nvdec_v4l2_suspend(struct nvdec_v4l2 *nvdec)
{
}

static inline void nvdec_v4l2_resume(struct nvdec_v4l2 *nvdec)
{
}
#endif

#endif
