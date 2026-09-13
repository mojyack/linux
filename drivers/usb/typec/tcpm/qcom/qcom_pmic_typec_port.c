// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023, Linaro Ltd. All rights reserved.
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/tcpm.h>
#include <linux/workqueue.h>

#include "qcom_pmic_typec.h"
#include "qcom_pmic_typec_port.h"

static const char * const typec_cc_status_name[] = {
	[TYPEC_CC_OPEN]		= "Open",
	[TYPEC_CC_RA]		= "Ra",
	[TYPEC_CC_RD]		= "Rd",
	[TYPEC_CC_RP_DEF]	= "Rp-def",
	[TYPEC_CC_RP_1_5]	= "Rp-1.5",
	[TYPEC_CC_RP_3_0]	= "Rp-3.0",
};

const char *qcom_pmic_typec_cc_to_name(enum typec_cc_status cc)
{
	if (cc > TYPEC_CC_RP_3_0)
		return "unknown";

	return typec_cc_status_name[cc];
}

static void qcom_pmic_typec_port_cc_debounce(struct work_struct *work)
{
	struct pmic_typec_port *pmic_typec_port =
		container_of(work, struct pmic_typec_port, cc_debounce_dwork.work);
	unsigned long flags;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);
	pmic_typec_port->debouncing_cc = false;
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(pmic_typec_port->dev, "Debounce cc complete\n");
}

void qcom_pmic_typec_port_set_cc_debounce(struct pmic_typec_port *pmic_typec_port)
{
	pmic_typec_port->debouncing_cc = true;
	schedule_delayed_work(&pmic_typec_port->cc_debounce_dwork,
			      msecs_to_jiffies(2));
}

static int qcom_pmic_typec_port_get_vbus(struct tcpc_dev *tcpc)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	int ret;

	mutex_lock(&pmic_typec_port->vbus_lock);
	ret = pmic_typec_port->vbus_enabled ||
	      pmic_typec_port->vbus_detect(pmic_typec_port);
	mutex_unlock(&pmic_typec_port->vbus_lock);

	return ret;
}

static int qcom_pmic_typec_port_set_vbus(struct tcpc_dev *tcpc, bool on, bool sink)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	int ret = 0;

	mutex_lock(&pmic_typec_port->vbus_lock);
	if (pmic_typec_port->vbus_enabled == on)
		goto done;

	if (on)
		ret = regulator_enable(pmic_typec_port->vdd_vbus);
	else
		ret = regulator_disable(pmic_typec_port->vdd_vbus);
	if (ret)
		goto done;

	pmic_typec_port->vbus_wait(pmic_typec_port, on);
	pmic_typec_port->vbus_enabled = on;
	tcpm_vbus_change(tcpm->tcpm_port);

done:
	dev_dbg(tcpm->dev, "set_vbus set: %d result %d\n", on, ret);
	mutex_unlock(&pmic_typec_port->vbus_lock);

	return ret;
}

static int qcom_pmic_typec_port_set_polarity(struct tcpc_dev *tcpc,
					     enum typec_cc_polarity pol)
{
	/* Polarity is set separately by phy-qcom-qmp.c */
	return 0;
}

static void qcom_pmic_typec_port_stop(struct pmic_typec *tcpm)
{
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	int i;

	for (i = 0; i < pmic_typec_port->nr_irqs; i++)
		disable_irq(pmic_typec_port->irq_data[i].irq);
}

int qcom_pmic_typec_port_init(struct platform_device *pdev,
			      struct pmic_typec *tcpm,
			      const struct pmic_typec_port_resources *res,
			      struct regmap *regmap,
			      u32 base,
			      irq_handler_t isr,
			      struct pmic_typec_port *pmic_typec_port)
{
	struct device *dev = &pdev->dev;
	struct pmic_typec_port_irq_data *irq_data;
	int i, ret, irq;

	if (!res->nr_irqs || res->nr_irqs > PMIC_TYPEC_MAX_IRQS)
		return -EINVAL;

	irq_data = devm_kcalloc(dev, res->nr_irqs, sizeof(*irq_data),
				GFP_KERNEL);
	if (!irq_data)
		return -ENOMEM;

	mutex_init(&pmic_typec_port->vbus_lock);

	pmic_typec_port->vdd_vbus = devm_regulator_get(dev, "vdd-vbus");
	if (IS_ERR(pmic_typec_port->vdd_vbus))
		return PTR_ERR(pmic_typec_port->vdd_vbus);

	pmic_typec_port->dev = dev;
	pmic_typec_port->base = base;
	pmic_typec_port->regmap = regmap;
	pmic_typec_port->nr_irqs = res->nr_irqs;
	pmic_typec_port->irq_data = irq_data;
	spin_lock_init(&pmic_typec_port->lock);
	INIT_DELAYED_WORK(&pmic_typec_port->cc_debounce_dwork,
			  qcom_pmic_typec_port_cc_debounce);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	for (i = 0; i < res->nr_irqs; i++, irq_data++) {
		irq = platform_get_irq_byname(pdev,
					      res->irq_params[i].irq_name);
		if (irq < 0)
			return irq;

		irq_data->pmic_typec_port = pmic_typec_port;
		irq_data->irq = irq;
		irq_data->virq = res->irq_params[i].virq;
		ret = devm_request_threaded_irq(dev, irq, NULL, isr,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						res->irq_params[i].irq_name,
						irq_data);
		if (ret)
			return ret;
	}

	tcpm->pmic_typec_port = pmic_typec_port;

	tcpm->tcpc.get_vbus = qcom_pmic_typec_port_get_vbus;
	tcpm->tcpc.set_vbus = qcom_pmic_typec_port_set_vbus;
	tcpm->tcpc.set_polarity = qcom_pmic_typec_port_set_polarity;

	tcpm->port_stop = qcom_pmic_typec_port_stop;

	return 0;
}
