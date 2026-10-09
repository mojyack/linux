// SPDX-License-Identifier: MIT
#include <linux/clk.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>

#include <subdev/clk.h>

#include "nouveau_drv.h"
#include "nouveau_chan.h"
#include "priv.h"
#include "gk20a_devfreq.h"
#include "gk20a.h"
#include "gp10b.h"

#define PMU_BUSY_CYCLES_NORM_MAX		1000U

#define PWR_PMU_IDLE_COUNTER_TOTAL		0U
#define PWR_PMU_IDLE_COUNTER_BUSY		4U

#define PWR_PMU_IDLE_COUNT_REG_OFFSET		0x0010A508U
#define PWR_PMU_IDLE_COUNT_REG_SIZE		16U
#define PWR_PMU_IDLE_COUNT_MASK			0x7FFFFFFFU
#define PWR_PMU_IDLE_COUNT_RESET_VALUE		(0x1U << 31U)

#define PWR_PMU_IDLE_INTR_REG_OFFSET		0x0010A9E8U
#define PWR_PMU_IDLE_INTR_ENABLE_VALUE		0U

#define PWR_PMU_IDLE_INTR_STATUS_REG_OFFSET	0x0010A9ECU
#define PWR_PMU_IDLE_INTR_STATUS_MASK		0x00000001U
#define PWR_PMU_IDLE_INTR_STATUS_RESET_VALUE	0x1U

#define PWR_PMU_IDLE_THRESHOLD_REG_OFFSET	0x0010A8A0U
#define PWR_PMU_IDLE_THRESHOLD_REG_SIZE		4U
#define PWR_PMU_IDLE_THRESHOLD_MAX_VALUE	0x7FFFFFFFU

#define PWR_PMU_IDLE_CTRL_REG_OFFSET		0x0010A50CU
#define PWR_PMU_IDLE_CTRL_REG_SIZE		16U
#define PWR_PMU_IDLE_CTRL_VALUE_MASK		0x3U
#define PWR_PMU_IDLE_CTRL_VALUE_BUSY		0x2U
#define PWR_PMU_IDLE_CTRL_VALUE_ALWAYS		0x3U
#define PWR_PMU_IDLE_CTRL_FILTER_MASK		(0x1U << 2)
#define PWR_PMU_IDLE_CTRL_FILTER_DISABLED	0x0U

#define PWR_PMU_IDLE_MASK_REG_OFFSET		0x0010A504U
#define PWR_PMU_IDLE_MASK_REG_SIZE		16U
#define PWM_PMU_IDLE_MASK_GR_ENABLED		0x1U
#define PWM_PMU_IDLE_MASK_CE_2_ENABLED		0x200000U

/**
 * struct gk20a_devfreq - Device frequency management
 */
struct gk20a_devfreq {
	/** @devfreq: devfreq device. */
	struct devfreq *devfreq;

	/** @regs: Device registers. */
	void __iomem *regs;

	/** @gov_data: Governor data. */
	struct devfreq_simple_ondemand_data gov_data;

	/** @busy_time: Busy time. */
	ktime_t busy_time;

	/** @total_time: Total time. */
	ktime_t total_time;

	/** @time_last_update: Last update time. */
	ktime_t time_last_update;

	/** @suspended: Governor suspended by gk20a_devfreq_fini(). */
	bool suspended;
};

static struct gk20a_devfreq *dev_to_gk20a_devfreq(struct device *dev)
{
	struct nouveau_drm *drm = dev_get_drvdata(dev);
	struct nvkm_subdev *subdev = nvkm_device_subdev(drm->nvkm, NVKM_SUBDEV_CLK, 0);
	struct nvkm_clk *base = nvkm_clk(subdev);

	switch (drm->nvkm->chipset) {
	case 0x13b: return gp10b_clk(base)->devfreq; break;
	default: return gk20a_clk(base)->devfreq; break;
	}
}

static void gk20a_pmu_init_perfmon_counter(struct gk20a_devfreq *gdevfreq)
{
	u32 data;

	// Set pmu idle intr status bit on total counter overflow
	writel(PWR_PMU_IDLE_INTR_ENABLE_VALUE,
	       gdevfreq->regs + PWR_PMU_IDLE_INTR_REG_OFFSET);

	writel(PWR_PMU_IDLE_THRESHOLD_MAX_VALUE,
	       gdevfreq->regs + PWR_PMU_IDLE_THRESHOLD_REG_OFFSET +
	       (PWR_PMU_IDLE_COUNTER_TOTAL * PWR_PMU_IDLE_THRESHOLD_REG_SIZE));

	// Setup counter for total cycles
	data = readl(gdevfreq->regs + PWR_PMU_IDLE_CTRL_REG_OFFSET +
		     (PWR_PMU_IDLE_COUNTER_TOTAL * PWR_PMU_IDLE_CTRL_REG_SIZE));
	data &= ~(PWR_PMU_IDLE_CTRL_VALUE_MASK | PWR_PMU_IDLE_CTRL_FILTER_MASK);
	data |= PWR_PMU_IDLE_CTRL_VALUE_ALWAYS | PWR_PMU_IDLE_CTRL_FILTER_DISABLED;
	writel(data, gdevfreq->regs + PWR_PMU_IDLE_CTRL_REG_OFFSET +
		     (PWR_PMU_IDLE_COUNTER_TOTAL * PWR_PMU_IDLE_CTRL_REG_SIZE));

	// Setup counter for busy cycles
	writel(PWM_PMU_IDLE_MASK_GR_ENABLED | PWM_PMU_IDLE_MASK_CE_2_ENABLED,
	       gdevfreq->regs + PWR_PMU_IDLE_MASK_REG_OFFSET +
	       (PWR_PMU_IDLE_COUNTER_BUSY * PWR_PMU_IDLE_MASK_REG_SIZE));

	data = readl(gdevfreq->regs + PWR_PMU_IDLE_CTRL_REG_OFFSET +
		     (PWR_PMU_IDLE_COUNTER_BUSY * PWR_PMU_IDLE_CTRL_REG_SIZE));
	data &= ~(PWR_PMU_IDLE_CTRL_VALUE_MASK | PWR_PMU_IDLE_CTRL_FILTER_MASK);
	data |= PWR_PMU_IDLE_CTRL_VALUE_BUSY | PWR_PMU_IDLE_CTRL_FILTER_DISABLED;
	writel(data, gdevfreq->regs + PWR_PMU_IDLE_CTRL_REG_OFFSET +
		     (PWR_PMU_IDLE_COUNTER_BUSY * PWR_PMU_IDLE_CTRL_REG_SIZE));
}

static u32 gk20a_pmu_read_idle_counter(struct gk20a_devfreq *gdevfreq, u32 counter_id)
{
	u32 ret;

	ret = readl(gdevfreq->regs + PWR_PMU_IDLE_COUNT_REG_OFFSET +
		    (counter_id * PWR_PMU_IDLE_COUNT_REG_SIZE));

	return ret & PWR_PMU_IDLE_COUNT_MASK;
}

static void gk20a_pmu_reset_idle_counter(struct gk20a_devfreq *gdevfreq, u32 counter_id)
{
	writel(PWR_PMU_IDLE_COUNT_RESET_VALUE, gdevfreq->regs + PWR_PMU_IDLE_COUNT_REG_OFFSET +
					       (counter_id * PWR_PMU_IDLE_COUNT_REG_SIZE));
}

static u32 gk20a_pmu_read_idle_intr_status(struct gk20a_devfreq *gdevfreq)
{
	u32 ret;

	ret = readl(gdevfreq->regs + PWR_PMU_IDLE_INTR_STATUS_REG_OFFSET);

	return ret & PWR_PMU_IDLE_INTR_STATUS_MASK;
}

static void gk20a_pmu_clear_idle_intr_status(struct gk20a_devfreq *gdevfreq)
{
	writel(PWR_PMU_IDLE_INTR_STATUS_RESET_VALUE,
	       gdevfreq->regs + PWR_PMU_IDLE_INTR_STATUS_REG_OFFSET);
}

static void gk20a_devfreq_update_utilization(struct gk20a_devfreq *gdevfreq)
{
	ktime_t now, last;
	u64 busy_cycles, total_cycles;
	u32 norm, intr_status;

	now = ktime_get();
	last = gdevfreq->time_last_update;
	gdevfreq->total_time = ktime_us_delta(now, last);

	busy_cycles = gk20a_pmu_read_idle_counter(gdevfreq, PWR_PMU_IDLE_COUNTER_BUSY);
	total_cycles = gk20a_pmu_read_idle_counter(gdevfreq, PWR_PMU_IDLE_COUNTER_TOTAL);
	intr_status = gk20a_pmu_read_idle_intr_status(gdevfreq);

	gk20a_pmu_reset_idle_counter(gdevfreq, PWR_PMU_IDLE_COUNTER_BUSY);
	gk20a_pmu_reset_idle_counter(gdevfreq, PWR_PMU_IDLE_COUNTER_TOTAL);

	if (intr_status != 0UL) {
		norm = PMU_BUSY_CYCLES_NORM_MAX;
		gk20a_pmu_clear_idle_intr_status(gdevfreq);
	} else if (total_cycles == 0ULL || busy_cycles > total_cycles) {
		norm = PMU_BUSY_CYCLES_NORM_MAX;
	} else {
		norm = (u32)div64_u64(busy_cycles * PMU_BUSY_CYCLES_NORM_MAX,
				total_cycles);
	}

	gdevfreq->busy_time = div_u64(gdevfreq->total_time * norm, PMU_BUSY_CYCLES_NORM_MAX);
	gdevfreq->time_last_update = now;
}

static int gk20a_devfreq_target(struct device *dev, unsigned long *freq,
				u32 flags)
{
	struct nouveau_drm *drm = dev_get_drvdata(dev);
	struct nvkm_subdev *subdev = nvkm_device_subdev(drm->nvkm, NVKM_SUBDEV_CLK, 0);
	struct nvkm_clk *base = nvkm_clk(subdev);
	struct nvkm_pstate *pstates = base->func->pstates;
	int nr_pstates = base->func->nr_pstates;
	int i, ret;

	for (i = 0; i < nr_pstates - 1; i++)
		if (pstates[i].base.domain[nv_clk_src_gpc] * GK20A_CLK_GPC_MDIV >= *freq)
			break;

	ret = nvkm_clk_ustate(base, pstates[i].pstate, 0);
	ret |= nvkm_clk_ustate(base, pstates[i].pstate, 1);
	if (ret) {
		nvkm_error(subdev, "cannot update clock\n");
		return ret;
	}

	*freq = pstates[i].base.domain[nv_clk_src_gpc] * GK20A_CLK_GPC_MDIV;

	return 0;
}

static int gk20a_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct nouveau_drm *drm = dev_get_drvdata(dev);
	struct nvkm_subdev *subdev = nvkm_device_subdev(drm->nvkm, NVKM_SUBDEV_CLK, 0);
	struct nvkm_clk *base = nvkm_clk(subdev);

	*freq = nvkm_clk_read(base, nv_clk_src_gpc) * GK20A_CLK_GPC_MDIV;

	return 0;
}

static void gk20a_devfreq_reset(struct gk20a_devfreq *gdevfreq)
{
	gk20a_pmu_reset_idle_counter(gdevfreq, PWR_PMU_IDLE_COUNTER_BUSY);
	gk20a_pmu_reset_idle_counter(gdevfreq, PWR_PMU_IDLE_COUNTER_TOTAL);
	gk20a_pmu_clear_idle_intr_status(gdevfreq);

	gdevfreq->busy_time = 0;
	gdevfreq->total_time = 0;
	gdevfreq->time_last_update = ktime_get();
}

static int gk20a_devfreq_get_dev_status(struct device *dev,
					struct devfreq_dev_status *status)
{
	struct nouveau_drm *drm = dev_get_drvdata(dev);
	struct gk20a_devfreq *gdevfreq = dev_to_gk20a_devfreq(dev);

	gk20a_devfreq_get_cur_freq(dev, &status->current_frequency);

	gk20a_devfreq_update_utilization(gdevfreq);

	status->busy_time = ktime_to_ns(gdevfreq->busy_time);
	status->total_time = ktime_to_ns(gdevfreq->total_time);

	gk20a_devfreq_reset(gdevfreq);

	NV_DEBUG(drm, "busy %lu total %lu %lu %% freq %lu MHz\n",
		 status->busy_time, status->total_time,
		 status->busy_time / (status->total_time / 100),
		 status->current_frequency / 1000 / 1000);

	return 0;
}

static struct devfreq_dev_profile gk20a_devfreq_profile = {
	.timer = DEVFREQ_TIMER_DELAYED,
	.polling_ms = 50,
	.target = gk20a_devfreq_target,
	.get_cur_freq = gk20a_devfreq_get_cur_freq,
	.get_dev_status = gk20a_devfreq_get_dev_status,
};

int gk20a_devfreq_init(struct nvkm_clk *base, struct gk20a_devfreq **pgdevfreq)
{
	struct nvkm_device *device = base->subdev.device;
	struct nvkm_device_tegra *tdev = device->func->tegra(device);
	struct nvkm_pstate *pstates = base->func->pstates;
	int nr_pstates = base->func->nr_pstates;
	struct gk20a_devfreq *gdevfreq = *pgdevfreq;
	int i, ret;

	if (gdevfreq) {
		/* The perfmon counters lose their setup with the GPU. */
		gk20a_pmu_init_perfmon_counter(gdevfreq);
		gk20a_devfreq_reset(gdevfreq);

		if (!gdevfreq->suspended)
			return 0;

		gdevfreq->suspended = false;
		return devfreq_resume_device(gdevfreq->devfreq);
	}

	gdevfreq = kzalloc_obj(*gdevfreq);
	if (!gdevfreq)
		return -ENOMEM;

	gdevfreq->regs = tdev->regs;

	for (i = 0; i < nr_pstates; i++) {
		unsigned long freq = pstates[i].base.domain[nv_clk_src_gpc] *
				     GK20A_CLK_GPC_MDIV;

		ret = dev_pm_opp_add(device->dev, freq, 0);
		if (ret)
			goto err_opp;
	}

	gk20a_pmu_init_perfmon_counter(gdevfreq);
	gk20a_devfreq_reset(gdevfreq);

	gk20a_devfreq_profile.initial_freq =
		nvkm_clk_read(base, nv_clk_src_gpc) * GK20A_CLK_GPC_MDIV;

	gdevfreq->gov_data.upthreshold = 45;
	gdevfreq->gov_data.downdifferential = 5;

	/* The callbacks can run before devfreq_add_device() returns. */
	*pgdevfreq = gdevfreq;

	gdevfreq->devfreq = devfreq_add_device(device->dev, &gk20a_devfreq_profile,
					       DEVFREQ_GOV_SIMPLE_ONDEMAND,
					       &gdevfreq->gov_data);
	if (IS_ERR(gdevfreq->devfreq)) {
		ret = PTR_ERR(gdevfreq->devfreq);
		goto err_opp;
	}

	return 0;

err_opp:
	*pgdevfreq = NULL;
	dev_pm_opp_remove_all_dynamic(device->dev);
	kfree(gdevfreq);
	return ret;
}

void gk20a_devfreq_fini(struct nvkm_clk *base, struct gk20a_devfreq **pgdevfreq,
			enum nvkm_suspend_state suspend)
{
	struct gk20a_devfreq *gdevfreq = *pgdevfreq;

	if (!gdevfreq || gdevfreq->suspended)
		return;

	if (suspend != NVKM_POWEROFF) {
		if (!devfreq_suspend_device(gdevfreq->devfreq))
			gdevfreq->suspended = true;
		return;
	}

	/* Every callback reaches the clk through the nouveau_drm drvdata. */
	devfreq_remove_device(gdevfreq->devfreq);
	dev_pm_opp_remove_all_dynamic(base->subdev.device->dev);
	kfree(gdevfreq);
	*pgdevfreq = NULL;
}
