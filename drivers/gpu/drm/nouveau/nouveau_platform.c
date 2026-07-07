/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include "nouveau_platform.h"

#include <core/tegra.h>
#include <nvkm/subdev/clk/gk20a_devfreq.h>

static int nouveau_platform_probe(struct platform_device *pdev)
{
	const struct nvkm_device_tegra_func *func;
	struct nvkm_device *device = NULL;
	struct drm_device *drm;

	func = of_device_get_match_data(&pdev->dev);

	drm = nouveau_platform_device_create(func, pdev, &device);
	return PTR_ERR_OR_ZERO(drm);
}

static void nouveau_platform_remove(struct platform_device *pdev)
{
	struct nouveau_drm *drm = platform_get_drvdata(pdev);

	nouveau_drm_device_remove(drm);
}

#ifdef CONFIG_PM_SLEEP
static int nouveau_platform_suspend(struct device *dev)
{
	struct nouveau_drm *drm = dev_get_drvdata(dev);
	int ret;

	/* Stop the devfreq governor before quiescing the GPU. */
	gk20a_devfreq_suspend(dev);

	/* The SoC powers the GPU off in deep sleep, as the PCI path expects. */
	ret = nouveau_do_suspend(drm, false);
	if (ret)
		return ret;

	/* Rail-gate it, like the PCI path's D3hot: nvkm wants a reset GPU. */
	return nvkm_device_tegra_suspend(nvxx_device(drm));
}

static int nouveau_platform_resume(struct device *dev)
{
	struct nouveau_drm *drm = dev_get_drvdata(dev);
	int ret;

	/* Power the GPU back up before touching any of its registers. */
	ret = nvkm_device_tegra_resume(nvxx_device(drm));
	if (ret)
		return ret;

	/* Reinitialise the GPU before restarting the devfreq governor. */
	ret = nouveau_do_resume(drm, false);
	if (ret)
		return ret;

	gk20a_devfreq_resume(dev);

	return 0;
}

static SIMPLE_DEV_PM_OPS(nouveau_pm_ops, nouveau_platform_suspend,
			 nouveau_platform_resume);
#endif

#if IS_ENABLED(CONFIG_OF)
static const struct nvkm_device_tegra_func gk20a_platform_data = {
	.iommu_bit = 34,
	.require_vdd = true,
};

static const struct nvkm_device_tegra_func gm20b_platform_data = {
	.iommu_bit = 34,
	.require_vdd = true,
	.require_ref_clk = true,
};

static const struct nvkm_device_tegra_func gp10b_platform_data = {
	.iommu_bit = 36,
	/* power provided by generic PM domains */
	.require_vdd = false,
};

static const struct of_device_id nouveau_platform_match[] = {
	{
		.compatible = "nvidia,gk20a",
		.data = &gk20a_platform_data,
	},
	{
		.compatible = "nvidia,gm20b",
		.data = &gm20b_platform_data,
	},
	{
		.compatible = "nvidia,gp10b",
		.data = &gp10b_platform_data,
	},
	{ }
};

MODULE_DEVICE_TABLE(of, nouveau_platform_match);
#endif

struct platform_driver nouveau_platform_driver = {
	.driver = {
		.name = "nouveau",
		.of_match_table = of_match_ptr(nouveau_platform_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &nouveau_pm_ops,
#endif
	},
	.probe = nouveau_platform_probe,
	.remove = nouveau_platform_remove,
};
