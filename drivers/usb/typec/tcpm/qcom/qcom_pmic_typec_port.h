/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Ltd. All rights reserved.
 */
#ifndef __QCOM_PMIC_TYPEC_PORT_H__
#define __QCOM_PMIC_TYPEC_PORT_H__

#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/usb/tcpm.h>
#include <linux/workqueue.h>

struct pmic_typec;
struct pmic_typec_port;

/* Resources */
#define PMIC_TYPEC_MAX_IRQS				0x08

struct pmic_typec_port_irq_params {
	int				virq;
	char				*irq_name;
};

struct pmic_typec_port_resources {
	unsigned int				nr_irqs;
	const struct pmic_typec_port_irq_params	irq_params[PMIC_TYPEC_MAX_IRQS];
};

struct pmic_typec_port_irq_data {
	int				virq;
	int				irq;
	struct pmic_typec_port		*pmic_typec_port;
};

/**
 * struct pmic_typec_port - state shared by the PMIC Type-C port backends
 * @vbus_detect: report whether VBUS is currently present
 * @vbus_wait: wait for VBUS to settle after a regulator transition
 * @cc: CC state last requested by tcpm
 * @debouncing_cc: CC readings are stale until @cc_debounce_dwork runs
 * @lock: serializes register access against the interrupt handler
 */
struct pmic_typec_port {
	struct device			*dev;
	struct tcpm_port		*tcpm_port;
	struct regmap			*regmap;
	u32				base;
	unsigned int			nr_irqs;
	struct pmic_typec_port_irq_data	*irq_data;

	struct regulator		*vdd_vbus;
	bool				vbus_enabled;
	struct mutex			vbus_lock;		/* VBUS state serialization */

	bool (*vbus_detect)(struct pmic_typec_port *pmic_typec_port);
	void (*vbus_wait)(struct pmic_typec_port *pmic_typec_port, bool on);

	int				cc;
	bool				debouncing_cc;
	struct delayed_work		cc_debounce_dwork;

	spinlock_t			lock;	/* Register atomicity */
};

/* API */

extern const struct pmic_typec_port_resources pm8150b_port_res;
extern const struct pmic_typec_port_resources pmi8998_port_res;

const char *qcom_pmic_typec_cc_to_name(enum typec_cc_status cc);

void qcom_pmic_typec_port_set_cc_debounce(struct pmic_typec_port *pmic_typec_port);

int qcom_pmic_typec_port_init(struct platform_device *pdev,
			      struct pmic_typec *tcpm,
			      const struct pmic_typec_port_resources *res,
			      struct regmap *regmap,
			      u32 base,
			      irq_handler_t isr);

int qcom_pmic_typec_port_pm8150b_probe(struct platform_device *pdev,
				       struct pmic_typec *tcpm,
				       const struct pmic_typec_port_resources *res,
				       struct regmap *regmap,
				       u32 base);

int qcom_pmic_typec_port_pmi8998_probe(struct platform_device *pdev,
				       struct pmic_typec *tcpm,
				       const struct pmic_typec_port_resources *res,
				       struct regmap *regmap,
				       u32 base);

#endif /* __QCOM_PMIC_TYPE_C_PORT_H__ */
