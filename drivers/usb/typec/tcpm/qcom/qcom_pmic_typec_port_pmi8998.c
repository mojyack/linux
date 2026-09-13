// SPDX-License-Identifier: GPL-2.0
/*
 * PMI8998 Type-C port backend.
 *
 * PMI8998 has no dedicated Type-C peripheral: the same hardware DRP state
 * machine that PM8150B exposes at its own base is scattered through the USBIN
 * block instead, and reports through a single "type-c-change" interrupt.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string_choices.h>
#include <linux/usb/tcpm.h>

#include "qcom_pmic_typec.h"
#include "qcom_pmic_typec_port.h"

#define TYPE_C_STATUS_1_REG				0x0B
#define UFP_TYPEC_MASK					GENMASK(7, 5)
#define UFP_TYPEC_RDSTD					BIT(7)
#define UFP_TYPEC_RD1P5					BIT(6)
#define UFP_TYPEC_RD3P0					BIT(5)

#define TYPE_C_STATUS_2_REG				0x0C
#define DFP_TYPEC_MASK					GENMASK(3, 0)
#define DFP_RD_OPEN					BIT(3)
#define DFP_RD_RA_VCONN					BIT(2)
#define DFP_RD_RD					BIT(1)
#define DFP_RA_RA					BIT(0)

#define TYPE_C_STATUS_4_REG				0x0E
#define UFP_DFP_MODE_STATUS				BIT(7)
#define TYPEC_VBUS_STATUS				BIT(6)
#define TYPEC_VBUS_ERROR_STATUS				BIT(5)
#define TYPEC_DEBOUNCE_DONE_STATUS			BIT(4)
#define TYPEC_UFP_AUDIO_ADAPT_STATUS			BIT(3)
#define TYPEC_VCONN_OVERCURR_STATUS			BIT(2)
#define CC_ORIENTATION					BIT(1)
#define CC_ATTACHED					BIT(0)

#define TYPE_C_CFG_REG					0x58
#define FACTORY_MODE_DETECTION_EN			BIT(5)
#define VCONN_OC_CFG					BIT(1)
#define TYPE_C_OR_U_USB					BIT(0)

#define TYPE_C_CFG_2_REG				0x59
#define TYPE_C_DFP_CURRSRC_MODE				BIT(7)
#define DFP_CC_1P4V_OR_1P6V				BIT(6)
#define VCONN_SOFTSTART_CFG_MASK			GENMASK(5, 4)
#define EN_TRY_SOURCE_MODE				BIT(3)
#define TYPE_C_UFP_MODE					BIT(1)
#define EN_80UA_180UA_CUR_SOURCE			BIT(0)
#define TYPEC_SRC_RP_SEL_MASK	\
	(TYPE_C_DFP_CURRSRC_MODE | EN_80UA_180UA_CUR_SOURCE)
#define TYPEC_SRC_RP_SEL_80UA				0
#define TYPEC_SRC_RP_SEL_180UA				EN_80UA_180UA_CUR_SOURCE
#define TYPEC_SRC_RP_SEL_330UA				TYPE_C_DFP_CURRSRC_MODE

#define TYPE_C_CFG_3_REG				0x5A
#define TYPEC_LEGACY_CABLE_INT_EN			BIT(6)
#define TYPEC_NONCOMPLIANT_LEGACY_CABLE_INT_EN		BIT(5)
#define TYPEC_TRYSOURCE_DETECT_INT_EN			BIT(4)
#define TYPEC_TRYSINK_DETECT_INT_EN			BIT(3)
#define EN_TRYSINK_MODE					BIT(2)
#define EN_LEGACY_CABLE_DETECTION			BIT(1)

#define TYPE_C_INTRPT_ENB_REG				0x67
#define TYPEC_CCOUT_DETACH_INT_EN			BIT(7)
#define TYPEC_CCOUT_ATTACH_INT_EN			BIT(6)
#define TYPEC_VBUS_ERROR_INT_EN				BIT(5)
#define TYPEC_UFP_AUDIOADAPT_INT_EN			BIT(4)
#define TYPEC_DEBOUNCE_DONE_INT_EN			BIT(3)
#define TYPEC_CCSTATE_CHANGE_INT_EN			BIT(2)
#define TYPEC_VBUS_DEASSERT_INT_EN			BIT(1)
#define TYPEC_VBUS_ASSERT_INT_EN			BIT(0)

#define TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG		0x68
#define EXIT_SNK_BASED_ON_CC				BIT(7)
#define VCONN_EN_ORIENTATION				BIT(6)
#define TYPEC_VCONN_OVERCURR_INT_EN			BIT(5)
#define VCONN_EN_SRC					BIT(4)
#define VCONN_EN_VALUE					BIT(3)
#define TYPEC_POWER_ROLE_CMD_MASK			GENMASK(2, 0)
#define UFP_EN_CMD					BIT(2)
#define DFP_EN_CMD					BIT(1)
#define TYPEC_DISABLE_CMD				BIT(0)

/* Interrupt numbers */
#define PMIC_TYPEC_CHANGE_IRQ				0x0

/* TYPE_C_STATUS_1..4, read as a block by the interrupt handler */
#define TYPE_C_NR_STATUS				4
#define TYPE_C_STATUS_1					0
#define TYPE_C_STATUS_2					1
#define TYPE_C_STATUS_4					3

#define TYPE_C_STATUS_4_CC_MASK	\
	(UFP_DFP_MODE_STATUS | TYPEC_DEBOUNCE_DONE_STATUS | \
	 CC_ORIENTATION | CC_ATTACHED)

struct pmic_typec_port_pmi8998 {
	struct pmic_typec_port		port;
	u8				status[TYPE_C_NR_STATUS];
	bool				status_valid;
};

#define to_pmi8998(_port_) \
	container_of(_port_, struct pmic_typec_port_pmi8998, port)

#define status_to_cc(_status_) \
	((_status_)[TYPE_C_STATUS_4] & CC_ORIENTATION ? "cc2" : "cc1")

static int qcom_pmic_typec_port_pmi8998_read_status(struct pmic_typec_port *pmic_typec_port,
						    u8 *status)
{
	return regmap_bulk_read(pmic_typec_port->regmap,
				pmic_typec_port->base + TYPE_C_STATUS_1_REG,
				status, TYPE_C_NR_STATUS);
}

/*
 * A single interrupt reports every Type-C event, so work out what actually
 * moved by diffing the status block against the previous reading.
 */
static irqreturn_t pmic_typec_port_pmi8998_isr(int irq, void *dev_id)
{
	struct pmic_typec_port_irq_data *irq_data = dev_id;
	struct pmic_typec_port *pmic_typec_port = irq_data->pmic_typec_port;
	struct pmic_typec_port_pmi8998 *pmi8998 = to_pmi8998(pmic_typec_port);
	bool vbus_change = false;
	bool cc_change = false;
	u8 status[TYPE_C_NR_STATUS];
	unsigned long flags;
	u8 status_4_diff;

	if (qcom_pmic_typec_port_pmi8998_read_status(pmic_typec_port, status))
		return IRQ_HANDLED;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	status_4_diff = status[TYPE_C_STATUS_4] ^ pmi8998->status[TYPE_C_STATUS_4];

	if (!pmi8998->status_valid) {
		vbus_change = true;
		cc_change = true;
	} else {
		vbus_change = status_4_diff & TYPEC_VBUS_STATUS;
		cc_change = (status_4_diff & TYPE_C_STATUS_4_CC_MASK) ||
			    status[TYPE_C_STATUS_1] != pmi8998->status[TYPE_C_STATUS_1] ||
			    status[TYPE_C_STATUS_2] != pmi8998->status[TYPE_C_STATUS_2];
	}

	memcpy(pmi8998->status, status, TYPE_C_NR_STATUS);
	pmi8998->status_valid = true;

	if (pmic_typec_port->debouncing_cc)
		cc_change = false;

	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(pmic_typec_port->dev,
		"isr: status %*ph vbus_change %d cc_change %d\n",
		TYPE_C_NR_STATUS, status, vbus_change, cc_change);

	if (vbus_change)
		tcpm_vbus_change(pmic_typec_port->tcpm_port);

	if (cc_change)
		tcpm_cc_change(pmic_typec_port->tcpm_port);

	return IRQ_HANDLED;
}

static bool qcom_pmic_typec_port_pmi8998_vbus_detect(struct pmic_typec_port *pmic_typec_port)
{
	unsigned int status_4;

	if (regmap_read(pmic_typec_port->regmap,
			pmic_typec_port->base + TYPE_C_STATUS_4_REG, &status_4))
		return false;

	dev_dbg(pmic_typec_port->dev, "get_vbus: 0x%02x detect %d\n",
		status_4, !!(status_4 & TYPEC_VBUS_STATUS));

	return status_4 & TYPEC_VBUS_STATUS;
}

static void qcom_pmic_typec_port_pmi8998_vbus_wait(struct pmic_typec_port *pmic_typec_port,
						   bool on)
{
	unsigned int status_4;

	/*
	 * There is no vSafe5V/vSafe0V comparator here, only a VBUS present
	 * flag, so this cannot tell a discharged connector from a slowly
	 * decaying one.
	 */
	if (regmap_read_poll_timeout(pmic_typec_port->regmap,
				     pmic_typec_port->base + TYPE_C_STATUS_4_REG,
				     status_4, !!(status_4 & TYPEC_VBUS_STATUS) == on,
				     100, 250000))
		dev_warn(pmic_typec_port->dev, "vbus %s fail\n", str_on_off(on));
}

static int qcom_pmic_typec_port_pmi8998_get_cc(struct tcpc_dev *tcpc,
					       enum typec_cc_status *cc1,
					       enum typec_cc_status *cc2)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct device *dev = pmic_typec_port->dev;
	u8 status[TYPE_C_NR_STATUS] = {};
	enum typec_cc_status cc;
	int ret;

	if (pmic_typec_port->debouncing_cc)
		return -EBUSY;

	ret = qcom_pmic_typec_port_pmi8998_read_status(pmic_typec_port, status);
	if (ret)
		return ret;

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	if (!(status[TYPE_C_STATUS_4] & CC_ATTACHED))
		goto done;

	if (status[TYPE_C_STATUS_4] & UFP_DFP_MODE_STATUS) {
		switch (status[TYPE_C_STATUS_2] & DFP_TYPEC_MASK) {
		case DFP_RA_RA:
			cc = TYPEC_CC_RA;
			*cc1 = TYPEC_CC_RA;
			*cc2 = TYPEC_CC_RA;
			break;
		case DFP_RD_OPEN:
		case DFP_RD_RD:
			cc = TYPEC_CC_RD;
			break;
		case DFP_RD_RA_VCONN:
			cc = TYPEC_CC_RD;
			*cc1 = TYPEC_CC_RA;
			*cc2 = TYPEC_CC_RA;
			break;
		default:
			dev_warn(dev, "unexpected dfp status %.2x\n",
				 status[TYPE_C_STATUS_2]);
			cc = TYPEC_CC_RD;
			break;
		}
	} else {
		switch (status[TYPE_C_STATUS_1] & UFP_TYPEC_MASK) {
		case UFP_TYPEC_RDSTD:
			cc = TYPEC_CC_RP_DEF;
			break;
		case UFP_TYPEC_RD1P5:
			cc = TYPEC_CC_RP_1_5;
			break;
		case UFP_TYPEC_RD3P0:
			cc = TYPEC_CC_RP_3_0;
			break;
		default:
			dev_warn(dev, "unexpected ufp status %.2x\n",
				 status[TYPE_C_STATUS_1]);
			cc = TYPEC_CC_RP_DEF;
			break;
		}
	}

	if (status[TYPE_C_STATUS_4] & CC_ORIENTATION)
		*cc2 = cc;
	else
		*cc1 = cc;

done:
	dev_dbg(dev, "get_cc: status %*ph cc1 %s cc2 %s attached %d cc=%s\n",
		TYPE_C_NR_STATUS, status,
		qcom_pmic_typec_cc_to_name(*cc1), qcom_pmic_typec_cc_to_name(*cc2),
		!!(status[TYPE_C_STATUS_4] & CC_ATTACHED), status_to_cc(status));

	return 0;
}

static int qcom_pmic_typec_port_pmi8998_set_cc(struct tcpc_dev *tcpc,
					       enum typec_cc_status cc)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct device *dev = pmic_typec_port->dev;
	unsigned int currsrc;
	unsigned long flags;
	bool source = true;
	int ret = 0;

	/*
	 * The Rp current source encoding of TYPE_C_CFG_2 is not documented;
	 * this mirrors the three levels PM8150B selects through its dedicated
	 * TYPEC_CURRSRC_CFG register and has not been verified against a CC
	 * measurement.
	 */
	switch (cc) {
	case TYPEC_CC_OPEN:
	case TYPEC_CC_RP_DEF:
		currsrc = TYPEC_SRC_RP_SEL_80UA;
		break;
	case TYPEC_CC_RP_1_5:
		currsrc = TYPEC_SRC_RP_SEL_180UA;
		break;
	case TYPEC_CC_RP_3_0:
		currsrc = TYPEC_SRC_RP_SEL_330UA;
		break;
	case TYPEC_CC_RD:
		currsrc = TYPEC_SRC_RP_SEL_80UA;
		source = false;
		break;
	default:
		dev_warn(dev, "unexpected set_cc %d\n", cc);
		return -EINVAL;
	}

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	if (source)
		ret = regmap_update_bits(pmic_typec_port->regmap,
					 pmic_typec_port->base + TYPE_C_CFG_2_REG,
					 TYPEC_SRC_RP_SEL_MASK, currsrc);
	if (!ret) {
		pmic_typec_port->cc = cc;
		qcom_pmic_typec_port_set_cc_debounce(pmic_typec_port);
	}

	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(dev, "set_cc: %s currsrc=%x mode %s\n",
		qcom_pmic_typec_cc_to_name(cc), currsrc,
		source ? "source" : "sink");

	return ret;
}

static int qcom_pmic_typec_port_pmi8998_set_vconn(struct tcpc_dev *tcpc, bool on)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct device *dev = pmic_typec_port->dev;
	unsigned int orientation, status_4, mask, value;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pmic_typec_port->lock, flags);

	ret = regmap_read(pmic_typec_port->regmap,
			  pmic_typec_port->base + TYPE_C_STATUS_4_REG, &status_4);
	if (ret)
		goto done;

	/* Set VCONN on the inversion of the active CC channel */
	orientation = (status_4 & CC_ORIENTATION) ? 0 : VCONN_EN_ORIENTATION;
	if (on) {
		mask = VCONN_EN_ORIENTATION | VCONN_EN_VALUE;
		value = orientation | VCONN_EN_VALUE;
	} else {
		mask = VCONN_EN_VALUE;
		value = 0;
	}

	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 mask, value);
done:
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);

	dev_dbg(dev, "set_vconn: orientation %d value 0x%02x state %s\n",
		orientation, value, str_on_off(on));

	return ret;
}

static int qcom_pmic_typec_port_pmi8998_set_role(struct pmic_typec_port *pmic_typec_port,
						 u8 role, u8 try_mode)
{
	int ret;

	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + TYPE_C_CFG_2_REG,
				 EN_TRY_SOURCE_MODE,
				 try_mode & EN_TRY_SOURCE_MODE);
	if (ret)
		return ret;

	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + TYPE_C_CFG_3_REG,
				 EN_TRYSINK_MODE, try_mode & EN_TRYSINK_MODE);
	if (ret)
		return ret;

	return regmap_update_bits(pmic_typec_port->regmap,
				  pmic_typec_port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				  TYPEC_POWER_ROLE_CMD_MASK, role);
}

static int qcom_pmic_typec_port_pmi8998_start_toggling(struct tcpc_dev *tcpc,
						       enum typec_port_type port_type,
						       enum typec_cc_status cc)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct device *dev = pmic_typec_port->dev;
	unsigned long flags;
	u8 role, try_mode;
	int ret;

	switch (port_type) {
	case TYPEC_PORT_SRC:
		role = DFP_EN_CMD;
		try_mode = 0;
		break;
	case TYPEC_PORT_SNK:
		role = UFP_EN_CMD;
		try_mode = 0;
		break;
	case TYPEC_PORT_DRP:
		role = 0;
		try_mode = EN_TRYSINK_MODE;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(dev, "start_toggling: port_type %d current cc %d new %d\n",
		port_type, pmic_typec_port->cc, cc);

	spin_lock_irqsave(&pmic_typec_port->lock, flags);
	qcom_pmic_typec_port_set_cc_debounce(pmic_typec_port);
	ret = qcom_pmic_typec_port_pmi8998_set_role(pmic_typec_port,
						    TYPEC_DISABLE_CMD, 0);
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);
	if (ret)
		return ret;

	/* The state machine needs to reach idle before it can be re-armed */
	usleep_range(5000, 5100);

	spin_lock_irqsave(&pmic_typec_port->lock, flags);
	ret = qcom_pmic_typec_port_pmi8998_set_role(pmic_typec_port,
						    role, try_mode);
	spin_unlock_irqrestore(&pmic_typec_port->lock, flags);
	if (ret)
		return ret;

	/*
	 * type-c-change is edge triggered, so nothing fires if the state
	 * machine settles back into the state it was already in - which is
	 * what happens whenever a cable is attached before tcpm starts
	 * toggling. Have tcpm sample the port once either way.
	 */
	tcpm_cc_change(pmic_typec_port->tcpm_port);

	return 0;
}

#define TYPEC_INTR_EN_MASK			  \
	(TYPEC_CCOUT_DETACH_INT_EN		| \
	 TYPEC_CCOUT_ATTACH_INT_EN		| \
	 TYPEC_VBUS_ERROR_INT_EN		| \
	 TYPEC_UFP_AUDIOADAPT_INT_EN		| \
	 TYPEC_DEBOUNCE_DONE_INT_EN		| \
	 TYPEC_CCSTATE_CHANGE_INT_EN		| \
	 TYPEC_VBUS_DEASSERT_INT_EN		| \
	 TYPEC_VBUS_ASSERT_INT_EN)

static int qcom_pmic_typec_port_pmi8998_start(struct pmic_typec *tcpm,
					      struct tcpm_port *tcpm_port)
{
	struct pmic_typec_port *pmic_typec_port = tcpm->pmic_typec_port;
	struct pmic_typec_port_pmi8998 *pmi8998 = to_pmi8998(pmic_typec_port);
	unsigned int mask, value;
	int i, ret;

	/*
	 * Type-C rather than micro-USB detection, no factory mode, and stay in
	 * Attached.SRC when VCONN over-current happens.
	 */
	mask = TYPE_C_OR_U_USB | FACTORY_MODE_DETECTION_EN | VCONN_OC_CFG;
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + TYPE_C_CFG_REG, mask, 0);
	if (ret)
		goto done;

	/* Maximum VCONN softstart, CC threshold 1.6V in source mode */
	mask = VCONN_SOFTSTART_CFG_MASK | DFP_CC_1P4V_OR_1P6V | TYPE_C_UFP_MODE;
	value = VCONN_SOFTSTART_CFG_MASK | DFP_CC_1P4V_OR_1P6V;
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + TYPE_C_CFG_2_REG,
				 mask, value);
	if (ret)
		goto done;

	/* tcpm handles legacy cables itself, and the try.* results are unused */
	mask = TYPEC_LEGACY_CABLE_INT_EN | TYPEC_NONCOMPLIANT_LEGACY_CABLE_INT_EN |
	       TYPEC_TRYSOURCE_DETECT_INT_EN | TYPEC_TRYSINK_DETECT_INT_EN |
	       EN_LEGACY_CABLE_DETECTION;
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + TYPE_C_CFG_3_REG, mask, 0);
	if (ret)
		goto done;

	ret = regmap_write(pmic_typec_port->regmap,
			   pmic_typec_port->base + TYPE_C_INTRPT_ENB_REG,
			   TYPEC_INTR_EN_MASK);
	if (ret)
		goto done;

	/* Configure VCONN for software control, start disabled */
	mask = VCONN_EN_SRC | VCONN_EN_VALUE | TYPEC_POWER_ROLE_CMD_MASK;
	ret = regmap_update_bits(pmic_typec_port->regmap,
				 pmic_typec_port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 mask, VCONN_EN_SRC);
	if (ret)
		goto done;

	pmi8998->status_valid = false;
	pmic_typec_port->tcpm_port = tcpm_port;

	for (i = 0; i < pmic_typec_port->nr_irqs; i++)
		enable_irq(pmic_typec_port->irq_data[i].irq);

done:
	return ret;
}

int qcom_pmic_typec_port_pmi8998_probe(struct platform_device *pdev,
				       struct pmic_typec *tcpm,
				       const struct pmic_typec_port_resources *res,
				       struct regmap *regmap,
				       u32 base)
{
	struct pmic_typec_port_pmi8998 *pmi8998;
	struct pmic_typec_port *pmic_typec_port;
	int ret;

	pmi8998 = devm_kzalloc(&pdev->dev, sizeof(*pmi8998), GFP_KERNEL);
	if (!pmi8998)
		return -ENOMEM;

	ret = qcom_pmic_typec_port_init(pdev, tcpm, res, regmap, base,
					pmic_typec_port_pmi8998_isr,
					&pmi8998->port);
	if (ret)
		return ret;

	pmic_typec_port = tcpm->pmic_typec_port;
	pmic_typec_port->vbus_detect = qcom_pmic_typec_port_pmi8998_vbus_detect;
	pmic_typec_port->vbus_wait = qcom_pmic_typec_port_pmi8998_vbus_wait;

	tcpm->tcpc.set_cc = qcom_pmic_typec_port_pmi8998_set_cc;
	tcpm->tcpc.get_cc = qcom_pmic_typec_port_pmi8998_get_cc;
	tcpm->tcpc.set_vconn = qcom_pmic_typec_port_pmi8998_set_vconn;
	tcpm->tcpc.start_toggling = qcom_pmic_typec_port_pmi8998_start_toggling;

	tcpm->port_start = qcom_pmic_typec_port_pmi8998_start;

	return 0;
}

const struct pmic_typec_port_resources pmi8998_port_res = {
	.irq_params = {
		{
			.irq_name = "type-c-change",
			.virq = PMIC_TYPEC_CHANGE_IRQ,
		},
	},
	.nr_irqs = 1,
};
