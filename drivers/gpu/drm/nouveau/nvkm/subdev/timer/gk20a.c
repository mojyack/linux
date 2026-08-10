/*
 * Copyright 2012 Red Hat Inc.
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
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Ben Skeggs
 */
#include "priv.h"

#include <linux/math64.h>

static const struct nvkm_timer_func
gk20a_timer = {
	.intr = nv04_timer_intr,
	.read = nv04_timer_read,
	.time = nv04_timer_time,
	.alarm_init = nv04_timer_alarm_init,
	.alarm_fini = nv04_timer_alarm_fini,
};

int
gk20a_timer_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
		struct nvkm_timer **ptmr)
{
	return nvkm_timer_new_(&gk20a_timer, device, type, inst, ptmr);
}

/* Fed clk_m at 19.2MHz, not 31.25MHz, and the scaling registers are absent. */
#define GM20B_PTIMER_NUM 625	/* 31250000 / gcd */
#define GM20B_PTIMER_DEN 384	/* 19200000 / gcd */

static u64
gm20b_timer_read(struct nvkm_timer *tmr)
{
	return mul_u64_u32_div(nv04_timer_read(tmr), GM20B_PTIMER_NUM, GM20B_PTIMER_DEN);
}

static void
gm20b_timer_time(struct nvkm_timer *tmr, u64 time)
{
	nv04_timer_time(tmr, mul_u64_u32_div(time, GM20B_PTIMER_DEN, GM20B_PTIMER_NUM));
}

static void
gm20b_timer_alarm_init(struct nvkm_timer *tmr, u64 time)
{
	nv04_timer_alarm_init(tmr, mul_u64_u32_div(time, GM20B_PTIMER_DEN, GM20B_PTIMER_NUM));
}

static const struct nvkm_timer_func
gm20b_timer = {
	.intr = nv04_timer_intr,
	.read = gm20b_timer_read,
	.time = gm20b_timer_time,
	.alarm_init = gm20b_timer_alarm_init,
	.alarm_fini = nv04_timer_alarm_fini,
};

int
gm20b_timer_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
		struct nvkm_timer **ptmr)
{
	return nvkm_timer_new_(&gm20b_timer, device, type, inst, ptmr);
}
