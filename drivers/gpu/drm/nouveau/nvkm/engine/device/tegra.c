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
#include <core/tegra.h>
#ifdef CONFIG_NOUVEAU_PLATFORM_DRIVER
#include "priv.h"

#if IS_ENABLED(CONFIG_ARM_DMA_USE_IOMMU)
#include <asm/dma-iommu.h>
#endif

static void
nvkm_device_tegra_vdd_off(struct nvkm_device_tegra *tdev)
{
	int ret;

	if (!tdev->vdd_on)
		return;

	/* A failed regulator_disable() leaves our enable in place. */
	ret = regulator_disable(tdev->vdd);
	if (ret)
		dev_err(&tdev->pdev->dev, "failed to disable vdd: %d\n", ret);
	else
		tdev->vdd_on = false;
}

static int
nvkm_device_tegra_clk_on(struct nvkm_device_tegra *tdev)
{
	int ret;

	ret = clk_prepare_enable(tdev->clk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(tdev->clk_ref);
	if (ret)
		goto err_clk_ref;
	ret = clk_prepare_enable(tdev->clk_pwr);
	if (ret)
		goto err_clk_pwr;
	clk_set_rate(tdev->clk_pwr, 204000000);
	udelay(10);

	tdev->clk_on = true;
	tdev->clk_gated = false;
	return 0;

err_clk_pwr:
	clk_disable_unprepare(tdev->clk_ref);
err_clk_ref:
	clk_disable_unprepare(tdev->clk);
	return ret;
}

static void
nvkm_device_tegra_clk_off(struct nvkm_device_tegra *tdev)
{
	if (!tdev->clk_on)
		return;

	clk_disable_unprepare(tdev->clk_pwr);
	clk_disable_unprepare(tdev->clk_ref);
	if (!tdev->clk_gated)
		clk_disable(tdev->clk);
	clk_unprepare(tdev->clk);
	udelay(10);
	tdev->clk_on = false;
}

/* Clamp the GPU and hold it in reset; tdev->isolated says whether both took. */
static bool
nvkm_device_tegra_isolate(struct nvkm_device_tegra *tdev)
{
	struct device *dev = &tdev->pdev->dev;
	int clamp, rst;

	if (tdev->isolated)
		return true;

	clamp = tegra_pmc_gpu_set_clamping(tdev->pmc, true);
	if (clamp)
		dev_err(dev, "failed to clamp: %d\n", clamp);
	udelay(10);

	rst = reset_control_assert(tdev->rst);
	if (rst)
		dev_err(dev, "failed to assert reset: %d\n", rst);
	udelay(10);

	tdev->isolated = !clamp && !rst;
	return tdev->isolated;
}

/* Drain the GPU's memory client and isolate it. */
static int
nvkm_device_tegra_stop(struct nvkm_device_tegra *tdev)
{
	struct device *dev = &tdev->pdev->dev;
	int ret;

	/* The MC's assert returns at once if a request is left set. */
	ret = reset_control_deassert(tdev->mc_rst);
	if (ret)
		goto err;
	ret = reset_control_assert(tdev->mc_rst);
	if (ret)
		goto err_release;
	udelay(10);

	/* Only a reset controller error fails it; the next power-up retries. */
	if (nvkm_device_tegra_isolate(tdev))
		return 0;

	ret = -EIO;
	goto err;

err_release:
	if (reset_control_deassert(tdev->mc_rst))
		dev_err(dev, "failed to release the memory client\n");
err:
	dev_err(dev, "failed to stop the GPU: %d\n", ret);
	return ret;
}

static void
nvkm_device_tegra_power_down(struct nvkm_device_tegra *tdev)
{
	if (tdev->railgate) {
		if (tdev->clk_on && !tdev->isolated) {
			if (nvkm_device_tegra_stop(tdev))
				return;
		} else {
			/* Unclocked since its reset, so it has nothing in flight. */
			nvkm_device_tegra_isolate(tdev);
		}
	}

	nvkm_device_tegra_clk_off(tdev);

	/* Never cut the rail under a GPU that is not clamped and in reset. */
	if (!tdev->railgate || tdev->isolated)
		nvkm_device_tegra_vdd_off(tdev);
}

static int
nvkm_device_tegra_power_up(struct nvkm_device_tegra *tdev)
{
	int ret;

	if (tdev->railgate) {
		/* Finish what an earlier power_down() could not. */
		nvkm_device_tegra_power_down(tdev);
		if (!tdev->isolated)
			return -EBUSY;
	}

	if (tdev->vdd && !tdev->vdd_on) {
		ret = regulator_enable(tdev->vdd);
		if (ret)
			return ret;
		tdev->vdd_on = true;
	}

	ret = nvkm_device_tegra_clk_on(tdev);
	if (ret)
		goto err;

	if (tdev->railgate) {
		ret = tegra_pmc_gpu_set_clamping(tdev->pmc, false);
		tdev->isolated = false;
		if (ret)
			goto err;
		udelay(10);

		clk_disable(tdev->clk);
		tdev->clk_gated = true;
		ret = reset_control_deassert(tdev->rst);
		if (ret)
			goto err;
		udelay(10);
		ret = clk_enable(tdev->clk);
		if (ret)
			goto err;
		tdev->clk_gated = false;

		/* Release the flush, and re-establish it after boot or deep sleep. */
		ret = reset_control_assert(tdev->mc_rst);
		if (ret)
			goto err;
		udelay(10);
		ret = reset_control_deassert(tdev->mc_rst);
		if (ret)
			goto err;
		udelay(10);
	} else if (!tdev->pdev->dev.pm_domain) {
		reset_control_assert(tdev->rst);
		udelay(10);

		ret = tegra_pmc_powergate_remove_clamping(tdev->pmc,
							  TEGRA_POWERGATE_3D);
		if (ret)
			goto err;
		udelay(10);

		reset_control_deassert(tdev->rst);
		udelay(10);
	}

	return 0;

err:
	/* The GPU has run nothing since its reset, there is nothing to flush. */
	if (tdev->railgate)
		nvkm_device_tegra_isolate(tdev);
	nvkm_device_tegra_power_down(tdev);
	return ret;
}

/* An enable we hold on a GPU we could not isolate outlives us: keep the rail on. */
static void
nvkm_device_tegra_put_vdd(struct nvkm_device_tegra *tdev)
{
	if (IS_ERR_OR_NULL(tdev->vdd))
		return;

	if (tdev->vdd_on) {
		/* The kept consumer points at our device: keep that allocated too. */
		get_device(&tdev->pdev->dev);
		dev_err(&tdev->pdev->dev, "vdd enable not released, leaving it enabled\n");
	} else {
		regulator_put(tdev->vdd);
	}
}

static void
nvkm_device_tegra_probe_iommu(struct nvkm_device_tegra *tdev)
{
#if IS_ENABLED(CONFIG_IOMMU_API)
	struct device *dev = &tdev->pdev->dev;
	unsigned long pgsize_bitmap;
	int ret;

#if IS_ENABLED(CONFIG_ARM_DMA_USE_IOMMU)
	if (dev->archdata.mapping) {
		struct dma_iommu_mapping *mapping = to_dma_iommu_mapping(dev);

		arm_iommu_detach_device(dev);
		arm_iommu_release_mapping(mapping);
	}
#endif

	if (!tdev->func->iommu_bit)
		return;

	mutex_init(&tdev->iommu.mutex);

	if (device_iommu_mapped(dev)) {
		tdev->iommu.domain = iommu_paging_domain_alloc(dev);
		if (IS_ERR(tdev->iommu.domain))
			goto error;

		/*
		 * A IOMMU is only usable if it supports page sizes smaller
		 * or equal to the system's PAGE_SIZE, with a preference if
		 * both are equal.
		 */
		pgsize_bitmap = tdev->iommu.domain->pgsize_bitmap;
		if (pgsize_bitmap & PAGE_SIZE) {
			tdev->iommu.pgshift = PAGE_SHIFT;
		} else {
			tdev->iommu.pgshift = fls(pgsize_bitmap & ~PAGE_MASK);
			if (tdev->iommu.pgshift == 0) {
				dev_warn(dev, "unsupported IOMMU page size\n");
				goto free_domain;
			}
			tdev->iommu.pgshift -= 1;
		}

		ret = iommu_attach_device(tdev->iommu.domain, dev);
		if (ret)
			goto free_domain;

		ret = nvkm_mm_init(&tdev->iommu.mm, 0, 0,
				   (1ULL << tdev->func->iommu_bit) >>
				   tdev->iommu.pgshift, 1);
		if (ret)
			goto detach_device;
	}

	return;

detach_device:
	iommu_detach_device(tdev->iommu.domain, dev);

free_domain:
	iommu_domain_free(tdev->iommu.domain);

error:
	tdev->iommu.domain = NULL;
	tdev->iommu.pgshift = 0;
	dev_err(dev, "cannot initialize IOMMU MM\n");
#endif
}

static void
nvkm_device_tegra_remove_iommu(struct nvkm_device_tegra *tdev)
{
#if IS_ENABLED(CONFIG_IOMMU_API)
	if (tdev->iommu.domain) {
		nvkm_mm_fini(&tdev->iommu.mm);
		iommu_detach_device(tdev->iommu.domain, &tdev->pdev->dev);
		iommu_domain_free(tdev->iommu.domain);
	}
#endif
}

static struct nvkm_device_tegra *
nvkm_device_tegra(struct nvkm_device *device)
{
	return container_of(device, struct nvkm_device_tegra, device);
}

static struct resource *
nvkm_device_tegra_resource(struct nvkm_device *device, enum nvkm_bar_id bar)
{
	struct nvkm_device_tegra *tdev = nvkm_device_tegra(device);
	int idx;

	switch (bar) {
	case NVKM_BAR0_PRI: idx = 0; break;
	case NVKM_BAR1_FB : idx = 1; break;
	default:
		WARN_ON(1);
		return NULL;
	}

	return platform_get_resource(tdev->pdev, IORESOURCE_MEM, idx);
}

static resource_size_t
nvkm_device_tegra_resource_addr(struct nvkm_device *device, enum nvkm_bar_id bar)
{
	struct resource *res = nvkm_device_tegra_resource(device, bar);
	return res ? res->start : 0;
}

static resource_size_t
nvkm_device_tegra_resource_size(struct nvkm_device *device, enum nvkm_bar_id bar)
{
	struct resource *res = nvkm_device_tegra_resource(device, bar);
	return res ? resource_size(res) : 0;
}

static int
nvkm_device_tegra_irq(struct nvkm_device *device)
{
	struct nvkm_device_tegra *tdev = nvkm_device_tegra(device);

	return platform_get_irq_byname(tdev->pdev, "stall");
}

static void
nvkm_device_tegra_fini(struct nvkm_device *device, enum nvkm_suspend_state suspend)
{
	struct nvkm_device_tegra *tdev = nvkm_device_tegra(device);

	if (suspend == NVKM_POWEROFF || !tdev->railgate)
		return;

	nvkm_device_tegra_power_down(tdev);
	tdev->suspended = true;
}

/* nvkm_device_init() brings a suspended GPU back through here: power it up first. */
static int
nvkm_device_tegra_preinit(struct nvkm_device *device)
{
	struct nvkm_device_tegra *tdev = nvkm_device_tegra(device);
	int ret;

	if (!tdev->suspended)
		return 0;

	ret = nvkm_device_tegra_power_up(tdev);
	if (ret)
		return ret;

	tdev->suspended = false;
	return 0;
}

static void *
nvkm_device_tegra_dtor(struct nvkm_device *device)
{
	struct nvkm_device_tegra *tdev = nvkm_device_tegra(device);
	nvkm_device_tegra_power_down(tdev);
	nvkm_device_tegra_remove_iommu(tdev);
	nvkm_device_tegra_put_vdd(tdev);
	return tdev;
}

static const struct nvkm_device_func
nvkm_device_tegra_func = {
	.tegra = nvkm_device_tegra,
	.dtor = nvkm_device_tegra_dtor,
	.preinit = nvkm_device_tegra_preinit,
	.fini = nvkm_device_tegra_fini,
	.irq = nvkm_device_tegra_irq,
	.resource_addr = nvkm_device_tegra_resource_addr,
	.resource_size = nvkm_device_tegra_resource_size,
	.cpu_coherent = false,
};

int
nvkm_device_tegra_new(const struct nvkm_device_tegra_func *func,
		      struct platform_device *pdev,
		      const char *cfg, const char *dbg,
		      struct nvkm_device **pdevice)
{
	struct nvkm_device_tegra *tdev;
	unsigned long rate;
	int ret;

	if (!(tdev = kzalloc_obj(*tdev)))
		return -ENOMEM;

	tdev->func = func;
	tdev->pdev = pdev;

	tdev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tdev->regs))
		return PTR_ERR(tdev->regs);

	if (func->require_vdd) {
		tdev->vdd = regulator_get(&pdev->dev, "vdd");
		if (IS_ERR(tdev->vdd)) {
			ret = PTR_ERR(tdev->vdd);
			goto free;
		}
	}

	tdev->rst = devm_reset_control_get(&pdev->dev, "gpu");
	if (IS_ERR(tdev->rst)) {
		ret = PTR_ERR(tdev->rst);
		goto free;
	}

	tdev->mc_rst = devm_reset_control_get_optional_exclusive(&pdev->dev, "mc");
	if (IS_ERR(tdev->mc_rst)) {
		ret = PTR_ERR(tdev->mc_rst);
		goto free;
	}

	tdev->clk = devm_clk_get(&pdev->dev, "gpu");
	if (IS_ERR(tdev->clk)) {
		ret = PTR_ERR(tdev->clk);
		goto free;
	}

	rate = clk_get_rate(tdev->clk);
	if (rate == 0) {
		ret = clk_set_rate(tdev->clk, ULONG_MAX);
		if (ret < 0)
			goto free;

		rate = clk_get_rate(tdev->clk);

		dev_dbg(&pdev->dev, "GPU clock set to %lu\n", rate);
	}

	if (func->require_ref_clk)
		tdev->clk_ref = devm_clk_get(&pdev->dev, "ref");
	if (IS_ERR(tdev->clk_ref)) {
		ret = PTR_ERR(tdev->clk_ref);
		goto free;
	}

	tdev->clk_pwr = devm_clk_get(&pdev->dev, "pwr");
	if (IS_ERR(tdev->clk_pwr)) {
		ret = PTR_ERR(tdev->clk_pwr);
		goto free;
	}

	tdev->pmc = devm_tegra_pmc_get(&pdev->dev);
	if (IS_ERR(tdev->pmc)) {
		ret = PTR_ERR(tdev->pmc);
		goto free;
	}

	/**
	 * The IOMMU bit defines the upper limit of the GPU-addressable space.
	 */
	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(tdev->func->iommu_bit));
	if (ret)
		goto free;

	nvkm_device_tegra_probe_iommu(tdev);

	/* nvkm needs a reset to come back: the TRM's GM20B sequence, with a clamp. */
	tdev->railgate = tdev->vdd && tdev->mc_rst && !pdev->dev.pm_domain &&
			 !tegra_pmc_gpu_set_clamping(tdev->pmc, true);

	ret = nvkm_device_tegra_power_up(tdev);
	if (ret)
		goto remove;

	tdev->gpu_speedo = tegra_sku_info.gpu_speedo_value;
	tdev->gpu_speedo_id = tegra_sku_info.gpu_speedo_id;
	ret = nvkm_device_ctor(&nvkm_device_tegra_func, NULL, &pdev->dev,
			       NVKM_DEVICE_TEGRA, pdev->id, NULL,
			       cfg, dbg, &tdev->device);
	if (ret)
		goto powerdown;

	*pdevice = &tdev->device;

	return 0;

powerdown:
	nvkm_device_tegra_power_down(tdev);
remove:
	nvkm_device_tegra_remove_iommu(tdev);
free:
	nvkm_device_tegra_put_vdd(tdev);
	kfree(tdev);
	return ret;
}

bool
nvkm_device_tegra_can_suspend(struct nvkm_device *device)
{
	return nvkm_device_tegra(device)->railgate;
}
#else
int
nvkm_device_tegra_new(const struct nvkm_device_tegra_func *func,
		      struct platform_device *pdev,
		      const char *cfg, const char *dbg,
		      struct nvkm_device **pdevice)
{
	return -ENOSYS;
}
#endif
