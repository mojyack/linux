/* SPDX-License-Identifier: MIT */
#ifndef __GK20A_DEVFREQ_H__
#define __GK20A_DEVFREQ_H__

#include <linux/devfreq.h>

struct gk20a_devfreq;

#if defined(CONFIG_PM_DEVFREQ)
int gk20a_devfreq_init(struct nvkm_clk *base, struct gk20a_devfreq **devfreq,
		       bool icc);
void gk20a_devfreq_fini(struct nvkm_clk *base, struct gk20a_devfreq *devfreq);
void gk20a_devfreq_dtor(struct nvkm_clk *base, struct gk20a_devfreq **devfreq);
void gk20a_devfreq_pre_prog(struct gk20a_devfreq *devfreq, u32 rate);
void gk20a_devfreq_post_prog(struct gk20a_devfreq *devfreq, u32 rate);
#else
static inline int gk20a_devfreq_init(struct nvkm_clk *base, struct gk20a_devfreq **devfreq,
				     bool icc)
{
	return 0;
}

static inline void gk20a_devfreq_fini(struct nvkm_clk *base, struct gk20a_devfreq *devfreq) { }
static inline void gk20a_devfreq_dtor(struct nvkm_clk *base, struct gk20a_devfreq **devfreq) { }
static inline void gk20a_devfreq_pre_prog(struct gk20a_devfreq *devfreq, u32 rate) { }
static inline void gk20a_devfreq_post_prog(struct gk20a_devfreq *devfreq, u32 rate) { }
#endif /* CONFIG_PM_DEVFREQ */

#endif /* __GK20A_DEVFREQ_H__ */
