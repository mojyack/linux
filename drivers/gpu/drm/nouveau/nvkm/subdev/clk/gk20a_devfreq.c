// SPDX-License-Identifier: MIT
#include <linux/clk.h>
#include <linux/interconnect.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>

#include <subdev/clk.h>

#include "nouveau_drv.h"
#include "nouveau_chan.h"
#include "priv.h"
#include "gk20a_devfreq.h"
#include "gk20a.h"

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

/* DRAM bytes per GPC Hz; nvgpu: TEGRA_GM20B_BW_PER_FREQ (64) * 750 / 1000 = 48 */
#define GK20A_DEVFREQ_BW_PER_HZ			32

/**
 * struct gk20a_devfreq - Device frequency management
 */
struct gk20a_devfreq {
	/** @devfreq: devfreq device. */
	struct devfreq *devfreq;

	/** @clk: Clock subdev whose GPC clock is scaled. */
	struct nvkm_clk *clk;

	/** @regs: Device registers. */
	void __iomem *regs;

	/** @icc_path: Memory bandwidth path to the EMC, or NULL. */
	struct icc_path *icc_path;

	/** @icc_bw: Peak bandwidth @icc_path holds, in kBps. */
	u32 icc_bw;

	/** @icc_follow: @icc_bw follows the GPC PLL changes. */
	bool icc_follow;

	/** @gov_data: Governor data. */
	struct devfreq_simple_ondemand_data gov_data;

	/** @busy_time: Busy time. */
	ktime_t busy_time;

	/** @total_time: Total time. */
	ktime_t total_time;

	/** @time_last_update: Last update time. */
	ktime_t time_last_update;

	/** @lock: Serialises the callbacks with gk20a_devfreq_fini(). */
	struct mutex lock;

	/** @active: The GPU is initialised and the callbacks may use it. */
	bool active;

	/** @suspended: devfreq_suspend_device() was called and owes a resume. */
	bool suspended;
};

static void gk20a_devfreq_release(struct device *dev, void *res)
{
}

/* Not through the drvdata: the device outlives nouveau_drm until the clk dtor. */
static struct gk20a_devfreq *dev_to_gk20a_devfreq(struct device *dev)
{
	struct gk20a_devfreq **res;

	res = devres_find(dev, gk20a_devfreq_release, NULL, NULL);
	return *res;
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

static u32 gk20a_devfreq_bw(unsigned long rate)
{
	return Bps_to_icc(rate) * GK20A_DEVFREQ_BW_PER_HZ;
}

/* A failed request leaves @icc_bw alone, so the next call retries it. */
static void gk20a_devfreq_set_bw(struct gk20a_devfreq *gdevfreq, u32 bw)
{
	int ret;

	lockdep_assert_held(&gdevfreq->clk->mutex);

	if (bw == gdevfreq->icc_bw)
		return;

	ret = icc_set_bw(gdevfreq->icc_path, 0, bw);
	if (ret) {
		dev_warn_ratelimited(gdevfreq->clk->subdev.device->dev,
				     "cannot request %u kBps of memory bandwidth: %d\n",
				     bw, ret);
		return;
	}

	gdevfreq->icc_bw = bw;
}

/* Under the clk mutex: raise the floor before the PLL, lower it after. */
void gk20a_devfreq_pre_prog(struct gk20a_devfreq *gdevfreq, u32 rate)
{
	if (gdevfreq && gdevfreq->icc_follow &&
	    gk20a_devfreq_bw(rate) > gdevfreq->icc_bw)
		gk20a_devfreq_set_bw(gdevfreq, gk20a_devfreq_bw(rate));
}

void gk20a_devfreq_post_prog(struct gk20a_devfreq *gdevfreq, u32 rate)
{
	if (gdevfreq && gdevfreq->icc_follow)
		gk20a_devfreq_set_bw(gdevfreq, gk20a_devfreq_bw(rate));
}

static unsigned long gk20a_devfreq_read_freq(struct gk20a_devfreq *gdevfreq)
{
	return nvkm_clk_read(gdevfreq->clk, nv_clk_src_gpc) * GK20A_CLK_GPC_MDIV;
}

/* Returns the GPC rate read back, or 0 when @on is false. */
static unsigned long __gk20a_devfreq_sync_bw(struct gk20a_devfreq *gdevfreq, bool on)
{
	unsigned long rate = on ? gk20a_devfreq_read_freq(gdevfreq) : 0;

	lockdep_assert_held(&gdevfreq->clk->mutex);

	if (gdevfreq->icc_path) {
		gdevfreq->icc_follow = on;
		gk20a_devfreq_set_bw(gdevfreq, gk20a_devfreq_bw(rate));
	}

	return rate;
}

/* Also retries a failed request: @icc_bw is what the core accepted. */
static unsigned long gk20a_devfreq_sync_bw(struct gk20a_devfreq *gdevfreq, bool on)
{
	struct nvkm_clk *base = gdevfreq->clk;
	unsigned long rate;

	mutex_lock(&base->mutex);
	rate = __gk20a_devfreq_sync_bw(gdevfreq, on);
	mutex_unlock(&base->mutex);

	return rate;
}

static int gk20a_devfreq_target(struct device *dev, unsigned long *freq,
				u32 flags)
{
	struct gk20a_devfreq *gdevfreq = dev_to_gk20a_devfreq(dev);
	struct nvkm_clk *base = gdevfreq->clk;
	struct nvkm_pstate *pstates = base->func->pstates;
	int nr_pstates = base->func->nr_pstates;
	unsigned long rate;
	int i, ret;

	for (i = 0; i < nr_pstates - 1; i++)
		if (pstates[i].base.domain[nv_clk_src_gpc] * GK20A_CLK_GPC_MDIV >= *freq)
			break;

	mutex_lock(&gdevfreq->lock);
	if (!gdevfreq->active) {
		ret = -EAGAIN;
		goto unlock;
	}

	ret = nvkm_clk_ustate(base, pstates[i].pstate, 0);
	ret |= nvkm_clk_ustate(base, pstates[i].pstate, 1);
	rate = gk20a_devfreq_sync_bw(gdevfreq, true);
	if (ret) {
		nvkm_error(&base->subdev, "cannot update clock\n");
		goto unlock;
	}

	/* Another pstate requester may have superseded this one. */
	*freq = rate;

unlock:
	mutex_unlock(&gdevfreq->lock);
	return ret;
}

static int gk20a_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct gk20a_devfreq *gdevfreq = dev_to_gk20a_devfreq(dev);
	int ret = 0;

	mutex_lock(&gdevfreq->lock);
	if (gdevfreq->active) {
		*freq = gk20a_devfreq_read_freq(gdevfreq);
	} else {
		*freq = gdevfreq->devfreq->previous_freq;
		ret = -EAGAIN;
	}
	mutex_unlock(&gdevfreq->lock);

	return ret;
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
	struct gk20a_devfreq *gdevfreq = dev_to_gk20a_devfreq(dev);

	mutex_lock(&gdevfreq->lock);
	if (!gdevfreq->active) {
		mutex_unlock(&gdevfreq->lock);
		return -EAGAIN;
	}

	status->current_frequency = gk20a_devfreq_read_freq(gdevfreq);

	gk20a_devfreq_update_utilization(gdevfreq);

	status->busy_time = ktime_to_ns(gdevfreq->busy_time);
	status->total_time = ktime_to_ns(gdevfreq->total_time);

	gk20a_devfreq_reset(gdevfreq);
	mutex_unlock(&gdevfreq->lock);

	nvkm_debug(&gdevfreq->clk->subdev, "busy %lu total %lu %lu %% freq %lu MHz\n",
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

int gk20a_devfreq_init(struct nvkm_clk *base, struct gk20a_devfreq **pgdevfreq,
		       bool icc)
{
	struct nvkm_device *device = base->subdev.device;
	struct nvkm_device_tegra *tdev = device->func->tegra(device);
	struct nvkm_pstate *pstates = base->func->pstates;
	int nr_pstates = base->func->nr_pstates;
	struct gk20a_devfreq *gdevfreq = *pgdevfreq, **res;
	int i, ret;

	if (gdevfreq) {
		/* The perfmon counters lose their setup with the GPU. */
		gk20a_pmu_init_perfmon_counter(gdevfreq);
		gk20a_devfreq_reset(gdevfreq);

		mutex_lock(&gdevfreq->lock);
		gdevfreq->active = true;
		mutex_unlock(&gdevfreq->lock);

		gk20a_devfreq_sync_bw(gdevfreq, true);

		if (gdevfreq->suspended) {
			gdevfreq->suspended = false;
			ret = devfreq_resume_device(gdevfreq->devfreq);
			if (ret)
				nvkm_warn(&base->subdev, "cannot resume devfreq: %d\n", ret);
		}
		return 0;
	}

	gdevfreq = kzalloc_obj(*gdevfreq);
	if (!gdevfreq)
		return -ENOMEM;

	gdevfreq->clk = base;
	gdevfreq->regs = tdev->regs;
	mutex_init(&gdevfreq->lock);
	gdevfreq->active = true;

	if (icc) {
		gdevfreq->icc_path = of_icc_get(device->dev, "dma-mem");
		if (IS_ERR(gdevfreq->icc_path)) {
			ret = PTR_ERR(gdevfreq->icc_path);
			goto err_free;
		}
	}

	res = devres_alloc(gk20a_devfreq_release, sizeof(*res), GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto err_icc;
	}
	*res = gdevfreq;
	devres_add(device->dev, res);

	for (i = 0; i < nr_pstates; i++) {
		unsigned long freq = pstates[i].base.domain[nv_clk_src_gpc] *
				     GK20A_CLK_GPC_MDIV;

		ret = dev_pm_opp_add(device->dev, freq, 0);
		if (ret)
			goto err_opp;
	}

	gk20a_pmu_init_perfmon_counter(gdevfreq);
	gk20a_devfreq_reset(gdevfreq);

	gk20a_devfreq_profile.initial_freq = gk20a_devfreq_read_freq(gdevfreq);

	gdevfreq->gov_data.upthreshold = 45;
	gdevfreq->gov_data.downdifferential = 5;

	/* The governor can program the PLL before devfreq_add_device() returns. */
	mutex_lock(&base->mutex);
	*pgdevfreq = gdevfreq;
	__gk20a_devfreq_sync_bw(gdevfreq, true);
	mutex_unlock(&base->mutex);

	gdevfreq->devfreq = devfreq_add_device(device->dev, &gk20a_devfreq_profile,
					       DEVFREQ_GOV_SIMPLE_ONDEMAND,
					       &gdevfreq->gov_data);
	if (IS_ERR(gdevfreq->devfreq)) {
		ret = PTR_ERR(gdevfreq->devfreq);
		goto err_unpublish;
	}

	return 0;

err_unpublish:
	/* devfreq_add_device() has stopped any governor it started. */
	mutex_lock(&base->mutex);
	*pgdevfreq = NULL;
	mutex_unlock(&base->mutex);
err_opp:
	dev_pm_opp_remove_all_dynamic(device->dev);
	devres_destroy(device->dev, gk20a_devfreq_release, NULL, NULL);
err_icc:
	icc_put(gdevfreq->icc_path);
err_free:
	mutex_destroy(&gdevfreq->lock);
	kfree(gdevfreq);
	return ret;
}

void gk20a_devfreq_fini(struct nvkm_clk *base, struct gk20a_devfreq *gdevfreq)
{
	int ret;

	if (!gdevfreq || !gdevfreq->active)
		return;

	/* The suspend count moves even when this fails, so always pair it. */
	gdevfreq->suspended = true;
	ret = devfreq_suspend_device(gdevfreq->devfreq);
	if (ret)
		nvkm_warn(&base->subdev, "cannot suspend devfreq: %d\n", ret);

	mutex_lock(&gdevfreq->lock);
	gdevfreq->active = false;
	mutex_unlock(&gdevfreq->lock);

	gk20a_devfreq_sync_bw(gdevfreq, false);

	/* Not every pstate request waits for the work it queued. */
	flush_work(&base->work);
}

void gk20a_devfreq_dtor(struct nvkm_clk *base, struct gk20a_devfreq **pgdevfreq)
{
	struct gk20a_devfreq *gdevfreq = *pgdevfreq;
	struct device *dev = base->subdev.device->dev;

	if (!gdevfreq)
		return;

	gk20a_devfreq_fini(base, gdevfreq);
	devfreq_remove_device(gdevfreq->devfreq);
	dev_pm_opp_remove_all_dynamic(dev);
	devres_destroy(dev, gk20a_devfreq_release, NULL, NULL);
	icc_put(gdevfreq->icc_path);
	mutex_destroy(&gdevfreq->lock);
	kfree(gdevfreq);
	*pgdevfreq = NULL;
}
