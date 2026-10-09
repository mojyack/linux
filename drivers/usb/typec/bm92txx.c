// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ROHM BM92T36 USB Type-C / USB Power Delivery controller
 *
 * Copyright (c) 2020-2023 CTCaer <ctcaer@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/usb/pd.h>
#include <linux/usb/pd_vdo.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/workqueue.h>

/* Registers */
#define ALERT_STATUS_REG	0x02	/* Clears on read */
#define STATUS1_REG		0x03
#define STATUS2_REG		0x04
#define COMMAND_REG		0x05
#define CONFIG1_REG		0x06
#define DEV_CAPS_REG		0x07
#define READ_PDOS_SRC_REG	0x08	/* Partner source PDOs, 28 bytes */
#define CONFIG2_REG		0x17
#define DP_STATUS_REG		0x18
#define DP_ALERT_EN_REG		0x19
#define VENDOR_CONFIG_REG	0x1A
#define AUTO_NGT_FIXED_REG	0x20
#define AUTO_NGT_BATT_REG	0x23
#define SYS_CONFIG1_REG		0x26
#define SYS_CONFIG2_REG		0x27
#define CURRENT_PDO_REG		0x28
#define CURRENT_RDO_REG		0x2B
#define ALERT_ENABLE_REG	0x2E
#define SYS_CONFIG3_REG		0x2F
#define SET_RDO_REG		0x30
#define FW_TYPE_REG		0x4B
#define FW_REVISION_REG		0x4C
#define INCOMING_VDM_REG	0x50	/* 28 bytes */
#define OUTGOING_VDM_REG	0x60	/* 28 bytes */

/* ALERT_STATUS_REG */
#define ALERT_SNK_FAULT		BIT(0)
#define ALERT_SRC_FAULT		BIT(1)
#define ALERT_CMD_DONE		BIT(2)
#define ALERT_PLUGPULL		BIT(3)
#define ALERT_DP_EVENT		BIT(6)
#define ALERT_VDM_RECEIVED	BIT(11)
#define ALERT_CONTRACT		BIT(12)
#define ALERT_SRC_PLUGIN	BIT(13)
#define ALERT_PDO		BIT(14)

/* STATUS1_REG */
#define STATUS1_FAULT		GENMASK(1, 0)
#define STATUS1_LASTCMD		GENMASK(6, 4)
#define STATUS1_INSERT		BIT(7)
#define STATUS1_DR		GENMASK(9, 8)
#define STATUS1_VSAFE		BIT(10)
#define STATUS1_CSIDE		BIT(11)	/* CC2 is the active line */
#define STATUS1_SRC_MODE	BIT(12)

#define LASTCMD_COMPLETE	0
#define LASTCMD_ABORTED		2

#define DATA_ROLE_UFP		1
#define DATA_ROLE_DFP		2

/* STATUS2_REG */
#define STATUS2_VCONN_ON	BIT(9)
#define STATUS2_EM_CABLE	BIT(12)
#define STATUS2_OTG_INSERT	BIT(13)

/* DP_STATUS_REG */
#define DP_STATUS_SIGNAL_ON	BIT(7)
#define DP_STATUS_DP_EN		BIT(15)

/* CONFIG1_REG */
#define CONFIG1_SPDSRC		GENMASK(15, 14)
#define SPDSRC12_ON		0
#define SPDSRC12_OFF		3

/* VENDOR_CONFIG_REG */
#define VENDOR_CONFIG_OCP_DISABLE BIT(2)

/* COMMAND_REG */
#define ABORT_LASTCMD_SENT_CMD	0x0101
#define PS_RDY_CMD		0x0505
#define GET_SRC_CAP_CMD		0x0606
#define SEND_RDO_CMD		0x0707
#define PD_HARD_RST_CMD		0x0808
#define SYS_RESET_CMD		0x0D0D
#define SEND_VDM_CMD		0x1111
#define ACCEPT_VDM_CMD		0x1616
#define DR_SWAP_CMD		0x1818
#define DP_ENTER_MODE_CMD	0x3131
#define DP_CFG_AND_START_HPD_CMD 0x3636

#define BM92T_PD_REVISION	0x0200
#define BM92T_CMD_TIMEOUT_MS	1000
#define BM92T_RETRY_MS		100
#define BM92T_FAULT_POLL_MS	2000
#define BM92T_RAISE_DELAY_MS	2000
#define BM92T_DP_CFG_RETRIES	3
#define BM92T_RDO_RETRIES	2

enum bm92t_state {
	BM92T_IDLE,
	BM92T_DRAIN_ACCEPT,
	BM92T_RDO_SENT,
	BM92T_PS_RDY_SENT,
	BM92T_DR_SWAP_SENT,
	BM92T_DISC_ID_SENT,
	BM92T_DISC_ID_ACCEPT,
	BM92T_DISC_SVID_SENT,
	BM92T_DISC_SVID_ACCEPT,
	BM92T_DISC_MODE_SENT,
	BM92T_DISC_MODE_ACCEPT,
	BM92T_DP_ENTER_SENT,
	BM92T_DP_CFG_SENT,
	BM92T_NUM_STATES
};

static const char * const bm92t_state_names[] = {
	[BM92T_IDLE]		= "idle",
	[BM92T_DRAIN_ACCEPT]	= "drain-accept",
	[BM92T_RDO_SENT]	= "rdo-sent",
	[BM92T_PS_RDY_SENT]	= "ps-rdy-sent",
	[BM92T_DR_SWAP_SENT]	= "dr-swap-sent",
	[BM92T_DISC_ID_SENT]	= "disc-id-sent",
	[BM92T_DISC_ID_ACCEPT]	= "disc-id-accept",
	[BM92T_DISC_SVID_SENT]	= "disc-svid-sent",
	[BM92T_DISC_SVID_ACCEPT] = "disc-svid-accept",
	[BM92T_DISC_MODE_SENT]	= "disc-mode-sent",
	[BM92T_DISC_MODE_ACCEPT] = "disc-mode-accept",
	[BM92T_DP_ENTER_SENT]	= "dp-enter-sent",
	[BM92T_DP_CFG_SENT]	= "dp-cfg-sent",
};

static_assert(ARRAY_SIZE(bm92t_state_names) == BM92T_NUM_STATES);

struct bm92t_info {
	struct device *dev;
	struct i2c_client *client;
	struct workqueue_struct *wq;
	struct work_struct work;
	struct delayed_work timer;

	/* Fields below are owned by the work item */
	enum bm92t_state state;
	unsigned long deadline;
	u16 alert;
	u16 deferred;			/* idle events seen while draining */
	bool irq_pending;		/* the hard handler masked the IRQ */
	bool poll;
	unsigned long poll_at;
	bool faulted;
	bool rearm_after_reset;
	bool vbus_on;
	bool vbus_blocked;
	enum usb_role usb_role;

	bool attached;
	bool negotiated;
	bool caps_changed;
	unsigned int rdo_retries;
	bool discovered;
	bool partner_drd;
	unsigned int limit_ma;		/* published to the charger */
	unsigned int pending_ma;
	unsigned long raise_at;
	u32 dp_conf;
	unsigned int dp_cfg_retries;

	/* Fields below are protected by lock */
	struct mutex lock;
	bool stopped;

	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_pd_identity partner_ident;
	struct typec_cable *cable;
	struct typec_plug *plug;
	struct typec_altmode *port_altmode[2];
	struct typec_altmode *port_dp;
	struct typec_altmode *partner_dp;
	struct fwnode_handle *connector;

	struct usb_role_switch *role_sw;
	struct regulator *vbus;
	struct power_supply *psy;
	u32 sink_pdos[PDO_MAX_OBJECTS];
	unsigned int nr_sink_pdos;

	struct dentry *debugfs;
	u16 last_alert;
};

static int bm92t_read(struct bm92t_info *info, u8 reg, void *buf, size_t len)
{
	struct i2c_msg msgs[] = {
		{ .addr = info->client->addr, .len = 1, .buf = &reg },
		{ .addr = info->client->addr, .flags = I2C_M_RD,
		  .len = len, .buf = buf },
	};
	int ret;

	ret = i2c_transfer(info->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err_ratelimited(info->dev, "read 0x%02x failed: %d\n",
				    reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int bm92t_write(struct bm92t_info *info, const void *buf, size_t len)
{
	int ret;

	ret = i2c_master_send(info->client, buf, len);
	if (ret != len) {
		dev_err_ratelimited(info->dev, "write 0x%02x failed: %d\n",
				    *(const u8 *)buf, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int bm92t_read16(struct bm92t_info *info, u8 reg, u16 *val)
{
	__le16 buf;
	int ret;

	ret = bm92t_read(info, reg, &buf, sizeof(buf));
	if (!ret)
		*val = le16_to_cpu(buf);

	return ret;
}

static int bm92t_write16(struct bm92t_info *info, u8 reg, u16 val)
{
	u8 buf[3] = { reg, val & 0xff, val >> 8 };

	return bm92t_write(info, buf, sizeof(buf));
}

/* A length byte, then one little-endian word */
static int bm92t_read32(struct bm92t_info *info, u8 reg, u32 *val)
{
	u8 buf[5];
	int ret;

	ret = bm92t_read(info, reg, buf, sizeof(buf));
	if (!ret)
		*val = get_unaligned_le32(&buf[1]);

	return ret;
}

static int bm92t_update16(struct bm92t_info *info, u8 reg, u16 mask, u16 val)
{
	u16 old;
	int ret;

	ret = bm92t_read16(info, reg, &old);
	if (ret || (old & mask) == val)
		return ret;

	return bm92t_write16(info, reg, (old & ~mask) | val);
}

static void bm92t_set_state(struct bm92t_info *info, enum bm92t_state state)
{
	WRITE_ONCE(info->state, state);
	info->deadline = jiffies + msecs_to_jiffies(BM92T_CMD_TIMEOUT_MS);
	dev_dbg(info->dev, "state %s\n", bm92t_state_names[state]);
}

/* Enter @state only when the command that leads to it went out */
static int bm92t_send_cmd(struct bm92t_info *info, u16 cmd,
			  enum bm92t_state state)
{
	int ret;

	ret = bm92t_write16(info, COMMAND_REG, cmd);
	dev_dbg(info->dev, "cmd 0x%04x: %d\n", cmd, ret);
	bm92t_set_state(info, ret ? BM92T_IDLE : state);

	return ret;
}

static int bm92t_send_vdm(struct bm92t_info *info, u32 hdr, const void *data,
			  size_t len, enum bm92t_state state)
{
	u8 buf[2 + 4 + 8];
	int ret;

	if (WARN_ON(len > sizeof(buf) - 6))
		return -EINVAL;

	buf[0] = OUTGOING_VDM_REG;
	buf[1] = 4 + len;
	put_unaligned_le32(hdr, &buf[2]);
	memcpy(&buf[6], data, len);

	ret = bm92t_write(info, buf, 6 + len);
	if (ret) {
		bm92t_set_state(info, BM92T_IDLE);
		return ret;
	}

	return bm92t_send_cmd(info, SEND_VDM_CMD, state);
}

static bool bm92t_lastcmd_ok(struct bm92t_info *info, u16 status1)
{
	unsigned int lastcmd = FIELD_GET(STATUS1_LASTCMD, status1);

	if (lastcmd != LASTCMD_COMPLETE)
		dev_dbg(info->dev, "%s: command status %u\n",
			bm92t_state_names[info->state], lastcmd);

	return lastcmd == LASTCMD_COMPLETE;
}

/* Returns the number of objects after a header answering @svid/@cmd */
static int bm92t_read_vdm(struct bm92t_info *info, u8 *vdm, size_t size,
			  u16 svid, int cmd)
{
	unsigned int len;
	u32 hdr;
	int ret;

	ret = bm92t_read(info, INCOMING_VDM_REG, vdm, size);
	if (ret)
		return ret;

	dev_dbg(info->dev, "VDM %*ph\n", (int)size, vdm);

	len = vdm[0];
	if (len < 4 || len > size - 1 || len % 4)
		return -EPROTO;

	hdr = get_unaligned_le32(&vdm[1]);
	if (PD_VDO_VID(hdr) != svid)
		return -EPROTO;

	if (cmd >= 0 && (!PD_VDO_SVDM(hdr) || PD_VDO_CMD(hdr) != cmd ||
			 PD_VDO_CMDT(hdr) != CMDT_RSP_ACK))
		return -EPROTO;

	return len / 4 - 1;
}

/* Board sink capability at @mv, from the connector's sink-pdos */
static unsigned int bm92t_sink_ma(struct bm92t_info *info, unsigned int mv)
{
	unsigned int i;

	for (i = 0; i < info->nr_sink_pdos; i++) {
		u32 pdo = info->sink_pdos[i];

		if (pdo_type(pdo) == PDO_TYPE_FIXED &&
		    pdo_fixed_voltage(pdo) == mv)
			return pdo_fixed_current(pdo);
	}

	return 0;
}

static int bm92t_psy_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct bm92t_info *info = power_supply_get_drvdata(psy);
	unsigned int ma = READ_ONCE(info->limit_ma);

	if (psp != POWER_SUPPLY_PROP_CURRENT_MAX)
		return -EINVAL;

	/* Without a contract the charger's own detection stands */
	if (!ma)
		return -ENODATA;

	val->intval = ma * 1000;

	return 0;
}

static const enum power_supply_property bm92t_psy_props[] = {
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc bm92t_psy_desc = {
	.name = "bm92t-source",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = bm92t_psy_props,
	.num_properties = ARRAY_SIZE(bm92t_psy_props),
	.get_property = bm92t_psy_get_property,
};

/* Lower the charger's input limit at once, raise it only once settled */
static void bm92t_set_limit(struct bm92t_info *info, unsigned int ma)
{
	info->pending_ma = ma;

	if (ma > info->limit_ma) {
		if (!info->raise_at)
			info->raise_at = jiffies +
				msecs_to_jiffies(BM92T_RAISE_DELAY_MS);
		return;
	}

	info->raise_at = 0;
	if (ma == info->limit_ma)
		return;

	WRITE_ONCE(info->limit_ma, ma);
	power_supply_changed(info->psy);
}

static void bm92t_raise_limit(struct bm92t_info *info)
{
	if (!info->raise_at || time_before(jiffies, info->raise_at))
		return;

	info->raise_at = 0;
	dev_info(info->dev, "input current limit %u mA\n", info->pending_ma);
	WRITE_ONCE(info->limit_ma, info->pending_ma);
	power_supply_changed(info->psy);
}

/* Derive the input current from the contract the controller holds now */
static int bm92t_update_contract(struct bm92t_info *info)
{
	unsigned int ma;
	u32 pdo, rdo;
	int ret;

	ret = bm92t_read32(info, CURRENT_RDO_REG, &rdo);
	if (ret)
		return ret;

	if (!rdo) {
		typec_set_pwr_opmode(info->port, TYPEC_PWR_MODE_USB);
		bm92t_set_limit(info, 0);
		return 0;
	}

	ret = bm92t_read32(info, CURRENT_PDO_REG, &pdo);
	if (ret)
		return ret;

	ma = min(rdo_op_current(rdo),
		 bm92t_sink_ma(info, pdo_fixed_voltage(pdo)));

	typec_set_pwr_opmode(info->port, TYPEC_PWR_MODE_PD);
	if (info->partner)
		typec_partner_set_pd_revision(info->partner, BM92T_PD_REVISION);

	if (ma != info->pending_ma)
		dev_dbg(info->dev, "contract %u mV %u mA (RDO %08x)\n",
			pdo_fixed_voltage(pdo), rdo_op_current(rdo), rdo);

	bm92t_set_limit(info, ma);

	return 0;
}

/* Pick the fixed source PDO that gives the board the most power */
static int bm92t_request_pdo(struct bm92t_info *info)
{
	unsigned int i, n, best = 0, best_mw = 0, best_ma = 0, flags = 0;
	u8 buf[1 + 4 * PDO_MAX_OBJECTS + 1];
	u32 rdo;
	int ret;

	ret = bm92t_read(info, READ_PDOS_SRC_REG, buf, sizeof(buf) - 1);
	if (ret)
		return ret;

	n = min_t(unsigned int, buf[0] / 4, PDO_MAX_OBJECTS);
	for (i = 0; i < n; i++) {
		u32 pdo = get_unaligned_le32(&buf[1 + 4 * i]);
		unsigned int mv, src_ma, snk_ma, ma;

		if (i == 0)
			info->partner_drd = pdo & PDO_FIXED_DATA_SWAP;

		if (pdo_type(pdo) != PDO_TYPE_FIXED)
			continue;

		mv = pdo_fixed_voltage(pdo);
		src_ma = pdo_fixed_current(pdo);
		snk_ma = bm92t_sink_ma(info, mv);
		ma = min(src_ma, snk_ma);
		dev_dbg(info->dev, "source PDO %u: %u mV %u mA\n",
			i + 1, mv, src_ma);

		if (ma && mv * ma / 1000 >= best_mw) {
			best = i + 1;
			best_mw = mv * ma / 1000;
			best_ma = ma;
			flags = src_ma < snk_ma ? RDO_CAP_MISMATCH : 0;
		}
	}

	if (!best)
		return -ENODEV;

	rdo = RDO_FIXED(best, best_ma, best_ma, RDO_USB_COMM | flags);

	dev_dbg(info->dev, "requesting PDO %u at %u mA\n", best, best_ma);

	buf[0] = SET_RDO_REG;
	buf[1] = 4;
	put_unaligned_le32(rdo, &buf[2]);
	ret = bm92t_write(info, buf, 6);
	if (ret)
		return ret;

	return bm92t_send_cmd(info, SEND_RDO_CMD, BM92T_RDO_SENT);
}

static int bm92t_set_vbus(struct bm92t_info *info, bool on)
{
	int ret;

	if (!info->vbus || on == info->vbus_on)
		return 0;

	ret = on ? regulator_enable(info->vbus) : regulator_disable(info->vbus);
	if (ret) {
		dev_err(info->dev, "failed to %s VBUS: %d\n",
			on ? "enable" : "disable", ret);
		return ret;
	}

	info->vbus_on = on;

	return 0;
}

static int bm92t_set_usb_role(struct bm92t_info *info, enum usb_role role)
{
	int ret;

	if (role == info->usb_role)
		return 0;

	ret = usb_role_switch_set_role(info->role_sw, role);
	if (ret) {
		dev_err(info->dev, "failed to set USB role %s: %d\n",
			usb_role_string(role), ret);
		return ret;
	}

	info->usb_role = role;

	return 0;
}

static void bm92t_retry_soon(struct bm92t_info *info)
{
	info->poll = true;
	info->poll_at = jiffies + msecs_to_jiffies(BM92T_RETRY_MS);
}

static void bm92t_update_roles(struct bm92t_info *info, u16 status1,
			       u16 status2)
{
	bool source = status1 & STATUS1_SRC_MODE;
	bool sourcing = source && (status2 & STATUS2_OTG_INSERT);
	unsigned int dr = FIELD_GET(STATUS1_DR, status1);
	enum usb_role role = USB_ROLE_NONE;

	typec_set_pwr_role(info->port, source ? TYPEC_SOURCE : TYPEC_SINK);
	typec_set_vconn_role(info->port, status2 & STATUS2_VCONN_ON ?
			     TYPEC_SOURCE : TYPEC_SINK);

	if (bm92t_set_vbus(info, sourcing && !info->vbus_blocked))
		bm92t_retry_soon(info);

	if (dr == DATA_ROLE_DFP || sourcing) {
		typec_set_data_role(info->port, TYPEC_HOST);
		/* Never host a device that VBUS does not reach */
		if (!sourcing || info->vbus_on || !info->vbus)
			role = USB_ROLE_HOST;
	} else if (dr == DATA_ROLE_UFP) {
		typec_set_data_role(info->port, TYPEC_DEVICE);
		role = USB_ROLE_DEVICE;
	}

	if (bm92t_set_usb_role(info, role))
		bm92t_retry_soon(info);
}

static void bm92t_altmode_active(struct typec_altmode *alt)
{
	if (alt)
		typec_altmode_update_active(alt, true);
}

static void bm92t_unregister_altmode(struct typec_altmode **alt)
{
	if (*alt)
		typec_unregister_altmode(*alt);
	*alt = NULL;
}

static void bm92t_attach(struct bm92t_info *info, u16 status1, u16 status2)
{
	struct typec_partner_desc desc = { .identity = &info->partner_ident };

	memset(&info->partner_ident, 0, sizeof(info->partner_ident));
	info->partner = typec_register_partner(info->port, &desc);
	if (IS_ERR(info->partner)) {
		dev_err(info->dev, "failed to register partner: %pe\n",
			info->partner);
		info->partner = NULL;
	}

	typec_set_orientation(info->port, status1 & STATUS1_CSIDE ?
			      TYPEC_ORIENTATION_REVERSE :
			      TYPEC_ORIENTATION_NORMAL);

	if (status2 & STATUS2_EM_CABLE) {
		struct typec_cable_desc cable = { .type = USB_PLUG_TYPE_C };
		struct typec_plug_desc plug = { .index = TYPEC_PLUG_SOP_P };

		info->cable = typec_register_cable(info->port, &cable);
		if (IS_ERR(info->cable)) {
			info->cable = NULL;
		} else {
			info->plug = typec_register_plug(info->cable, &plug);
			if (IS_ERR(info->plug))
				info->plug = NULL;
		}
	}

	info->attached = true;
	info->deferred = 0;
	info->negotiated = false;
	info->caps_changed = false;
	info->rdo_retries = BM92T_RDO_RETRIES;
	info->discovered = false;
	info->partner_drd = false;
	info->dp_conf = 0;
}

static void bm92t_detach(struct bm92t_info *info)
{
	bm92t_set_state(info, BM92T_IDLE);

	bm92t_unregister_altmode(&info->partner_dp);
	if (info->plug)
		typec_unregister_plug(info->plug);
	if (info->cable)
		typec_unregister_cable(info->cable);
	if (info->partner)
		typec_unregister_partner(info->partner);
	info->plug = NULL;
	info->cable = NULL;
	info->partner = NULL;

	typec_set_orientation(info->port, TYPEC_ORIENTATION_NONE);
	typec_set_pwr_opmode(info->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(info->port, TYPEC_SINK);
	typec_set_vconn_role(info->port, TYPEC_SINK);
	typec_set_data_role(info->port, TYPEC_DEVICE);

	if (bm92t_set_vbus(info, false) || bm92t_set_usb_role(info, USB_ROLE_NONE))
		bm92t_retry_soon(info);
	bm92t_set_limit(info, 0);

	info->attached = false;
	info->vbus_blocked = false;
}

static void bm92t_discover(struct bm92t_info *info)
{
	info->discovered = true;
	bm92t_send_vdm(info, VDO(USB_SID_PD, 1, SVDM_VER_1_0,
				 CMD_DISCOVER_IDENT),
		       NULL, 0, BM92T_DISC_ID_SENT);
}

/* After a contract: take the host role if needed, then discover */
static void bm92t_contract_ready(struct bm92t_info *info, u16 status1)
{
	unsigned int dr = FIELD_GET(STATUS1_DR, status1);

	bm92t_set_state(info, BM92T_IDLE);

	if (info->discovered)
		return;

	if (dr == DATA_ROLE_DFP) {
		bm92t_discover(info);
	} else if (dr == DATA_ROLE_UFP && info->partner_drd) {
		bm92t_send_cmd(info, DR_SWAP_CMD, BM92T_DR_SWAP_SENT);
	} else {
		/* A device to this partner: nothing to discover */
		info->discovered = true;
	}
}

static void bm92t_dp_active(struct bm92t_info *info)
{
	bm92t_altmode_active(info->partner_dp);
	bm92t_set_state(info, BM92T_IDLE);
}

static void bm92t_dp_done(struct bm92t_info *info)
{
	bm92t_set_state(info, BM92T_IDLE);
}

/* Pin assignments both ends support, preferring D, which keeps USB */
static int bm92t_dp_pin(struct bm92t_info *info, u32 vdo)
{
	unsigned long pins = DP_CAP_PIN_ASSIGN_DFP_D(info->port_dp->vdo) &
			     DP_CAP_PIN_ASSIGN_UFP_D(vdo);

	if (pins & BIT(DP_PIN_ASSIGN_D))
		return DP_PIN_ASSIGN_D;
	if (pins & BIT(DP_PIN_ASSIGN_C))
		return DP_PIN_ASSIGN_C;
	if (pins & BIT(DP_PIN_ASSIGN_E))
		return DP_PIN_ASSIGN_E;

	return -ENODEV;
}

static int bm92t_dp_register(struct bm92t_info *info, u32 vdo)
{
	struct typec_altmode_desc desc = {
		.svid = USB_TYPEC_DP_SID,
		.mode = USB_TYPEC_DP_MODE,
		.vdo = vdo,
	};
	int pin;

	if (!(DP_CAP_CAPABILITY(vdo) & DP_CAP_UFP_D))
		return -ENODEV;

	pin = bm92t_dp_pin(info, vdo);
	if (pin < 0) {
		dev_info(info->dev, "no common DP pin assignment (%08x)\n", vdo);
		return pin;
	}

	info->dp_conf = DP_CONF_UFP_U_AS_UFP_D |
			FIELD_PREP(DP_CONF_SIGNALLING_MASK,
				   DP_CONF_SIGNALLING_HBR3) |
			DP_CONF_SET_PIN_ASSIGN(BIT(pin));
	info->dp_cfg_retries = BM92T_DP_CFG_RETRIES;

	if (info->partner && !info->partner_dp) {
		info->partner_dp = typec_partner_register_altmode(info->partner,
								  &desc);
		if (IS_ERR(info->partner_dp))
			info->partner_dp = NULL;
	}

	return 0;
}

static void bm92t_dp_configure(struct bm92t_info *info)
{
	u8 buf[6] = { OUTGOING_VDM_REG, 4 };

	put_unaligned_le32(info->dp_conf, &buf[2]);
	if (bm92t_write(info, buf, sizeof(buf)))
		bm92t_set_state(info, BM92T_IDLE);
	else
		bm92t_send_cmd(info, DP_CFG_AND_START_HPD_CMD,
			       BM92T_DP_CFG_SENT);
}

/* DP alt mode is entered: adopt a live link, configure a dark one */
static void bm92t_dp_entered(struct bm92t_info *info, u16 dp_status)
{
	if (dp_status & DP_STATUS_SIGNAL_ON) {
		dev_info(info->dev, "DP link already up, adopting it\n");
		bm92t_dp_active(info);
	} else {
		bm92t_dp_configure(info);
	}
}

/* A VDM was sent: accept the reply when it comes */
static void bm92t_vdm_sent(struct bm92t_info *info, u16 alert, u16 status1,
			   enum bm92t_state accept)
{
	if (alert & ALERT_VDM_RECEIVED)
		bm92t_send_cmd(info, ACCEPT_VDM_CMD, accept);
	else if ((alert & ALERT_CMD_DONE) && !bm92t_lastcmd_ok(info, status1))
		bm92t_set_state(info, BM92T_IDLE);
}

/* Advance the transaction in flight. Returns false while still waiting. */
static bool bm92t_step_done(struct bm92t_info *info, u16 alert, u16 status1)
{
	if (!(alert & ALERT_CMD_DONE))
		return false;

	if (!bm92t_lastcmd_ok(info, status1)) {
		bm92t_set_state(info, BM92T_IDLE);
		return false;
	}

	return true;
}

static void bm92t_step(struct bm92t_info *info, u16 alert, u16 status1,
		       u16 dp_status)
{
	u8 vdm[29];
	u32 vdo;
	int i, n, ret;

	switch (info->state) {
	case BM92T_IDLE:
		/* An unanswered partner VDM blocks the mailbox for ours */
		if (alert & ALERT_VDM_RECEIVED) {
			info->deferred = alert & ~ALERT_VDM_RECEIVED;
			bm92t_send_cmd(info, ACCEPT_VDM_CMD, BM92T_DRAIN_ACCEPT);
			break;
		}

		if (status1 & STATUS1_SRC_MODE) {
			if ((alert & ALERT_SRC_PLUGIN) && !info->discovered &&
			    FIELD_GET(STATUS1_DR, status1) == DATA_ROLE_DFP)
				bm92t_discover(info);
			break;
		}

		/* Request after the controller's own contract, or it aborts */
		if (alert & ALERT_PDO)
			info->caps_changed = true;
		if (!(alert & ALERT_CONTRACT) ||
		    (info->negotiated && !info->caps_changed))
			break;

		/* A plug-in wake-up brings the contract before the PDOs */
		if ((alert & ALERT_PLUGPULL) && !(alert & ALERT_PDO))
			msleep(500);

		info->negotiated = true;
		info->caps_changed = false;
		ret = bm92t_request_pdo(info);
		if (ret == -ENODEV) {
			dev_info(info->dev, "no usable source PDO\n");
			bm92t_contract_ready(info, status1);
		}
		break;

	case BM92T_DRAIN_ACCEPT:
		info->deferred |= alert & ~(ALERT_CMD_DONE | ALERT_VDM_RECEIVED);
		if (!(alert & ALERT_CMD_DONE))
			break;

		bm92t_read(info, INCOMING_VDM_REG, vdm, sizeof(vdm));
		bm92t_set_state(info, BM92T_IDLE);
		alert = info->deferred;
		info->deferred = 0;
		if (alert)
			bm92t_step(info, alert, status1, dp_status);
		break;

	case BM92T_RDO_SENT:
		if (alert & ALERT_CONTRACT) {
			bm92t_send_cmd(info, PS_RDY_CMD, BM92T_PS_RDY_SENT);
		} else if ((alert & ALERT_CMD_DONE) &&
			   !bm92t_lastcmd_ok(info, status1)) {
			/* Busy negotiating by itself: ask again after it */
			if (FIELD_GET(STATUS1_LASTCMD, status1) == LASTCMD_ABORTED &&
			    info->rdo_retries) {
				info->rdo_retries--;
				info->negotiated = false;
				bm92t_set_state(info, BM92T_IDLE);
			} else {
				bm92t_contract_ready(info, status1);
			}
		}
		break;

	case BM92T_PS_RDY_SENT:
		if (alert & ALERT_CMD_DONE)
			bm92t_contract_ready(info, status1);
		break;

	case BM92T_DR_SWAP_SENT:
		if (!bm92t_step_done(info, alert, status1))
			break;
		if (FIELD_GET(STATUS1_DR, status1) == DATA_ROLE_DFP)
			bm92t_discover(info);
		else
			bm92t_set_state(info, BM92T_IDLE);
		break;

	case BM92T_DISC_ID_SENT:
		bm92t_vdm_sent(info, alert, status1, BM92T_DISC_ID_ACCEPT);
		break;

	case BM92T_DISC_ID_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		n = bm92t_read_vdm(info, vdm, sizeof(vdm), USB_SID_PD,
				   CMD_DISCOVER_IDENT);
		if (n < 3) {
			dev_info(info->dev, "no partner identity: %d\n", n);
			bm92t_set_state(info, BM92T_IDLE);
			break;
		}

		info->partner_ident.id_header = get_unaligned_le32(&vdm[5]);
		info->partner_ident.cert_stat = get_unaligned_le32(&vdm[9]);
		info->partner_ident.product = get_unaligned_le32(&vdm[13]);
		for (i = 0; i < min(n - 3, 3); i++)
			info->partner_ident.vdo[i] =
				get_unaligned_le32(&vdm[17 + 4 * i]);
		if (info->partner)
			typec_partner_set_identity(info->partner);

		dev_info(info->dev, "partner %04x:%04x type %u\n",
			 PD_IDH_VID(info->partner_ident.id_header),
			 PD_PRODUCT_PID(info->partner_ident.product),
			 PD_IDH_PTYPE(info->partner_ident.id_header));

		if (info->port_dp)
			bm92t_send_vdm(info, VDO(USB_SID_PD, 1, SVDM_VER_1_0,
						 CMD_DISCOVER_SVID),
				       NULL, 0, BM92T_DISC_SVID_SENT);
		else
			bm92t_set_state(info, BM92T_IDLE);
		break;

	case BM92T_DISC_SVID_SENT:
		bm92t_vdm_sent(info, alert, status1, BM92T_DISC_SVID_ACCEPT);
		break;

	case BM92T_DISC_SVID_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		n = bm92t_read_vdm(info, vdm, sizeof(vdm), USB_SID_PD,
				   CMD_DISCOVER_SVID);
		/* Two SVIDs per object, the first in the upper half */
		for (i = 0; i < n; i++) {
			vdo = get_unaligned_le32(&vdm[5 + 4 * i]);
			if (vdo >> 16 == USB_TYPEC_DP_SID ||
			    (vdo & 0xffff) == USB_TYPEC_DP_SID)
				break;
		}

		if (n <= 0 || i == n) {
			bm92t_set_state(info, BM92T_IDLE);
			break;
		}

		bm92t_send_vdm(info, VDO(USB_TYPEC_DP_SID, 1, SVDM_VER_1_0,
					 CMD_DISCOVER_MODES),
			       NULL, 0, BM92T_DISC_MODE_SENT);
		break;

	case BM92T_DISC_MODE_SENT:
		bm92t_vdm_sent(info, alert, status1, BM92T_DISC_MODE_ACCEPT);
		break;

	case BM92T_DISC_MODE_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		n = bm92t_read_vdm(info, vdm, sizeof(vdm), USB_TYPEC_DP_SID,
				   CMD_DISCOVER_MODES);
		if (n < 1 || bm92t_dp_register(info, get_unaligned_le32(&vdm[5]))) {
			bm92t_set_state(info, BM92T_IDLE);
			break;
		}

		bm92t_send_cmd(info, DP_ENTER_MODE_CMD, BM92T_DP_ENTER_SENT);
		break;

	case BM92T_DP_ENTER_SENT:
		if (!(alert & ALERT_CMD_DONE))
			break;

		/* Refused when DP is entered already */
		if (!bm92t_lastcmd_ok(info, status1) &&
		    !(dp_status & DP_STATUS_DP_EN)) {
			bm92t_dp_done(info);
			break;
		}

		bm92t_dp_entered(info, dp_status);
		break;

	case BM92T_DP_CFG_SENT:
		if (!(alert & ALERT_CMD_DONE))
			break;

		if (bm92t_lastcmd_ok(info, status1))
			bm92t_dp_active(info);
		else if (info->dp_cfg_retries--)
			bm92t_dp_configure(info);
		else
			bm92t_dp_done(info);
		break;

	case BM92T_NUM_STATES:
		break;
	}
}

static void bm92t_timeout(struct bm92t_info *info)
{
	enum bm92t_state state = info->state;
	u16 status1;

	dev_warn(info->dev, "%s timed out\n", bm92t_state_names[state]);

	bm92t_write16(info, COMMAND_REG, ABORT_LASTCMD_SENT_CMD);
	bm92t_set_state(info, BM92T_IDLE);

	/* The contract the controller holds stands: carry on from it */
	if ((state == BM92T_RDO_SENT || state == BM92T_PS_RDY_SENT) &&
	    !bm92t_read16(info, STATUS1_REG, &status1))
		bm92t_contract_ready(info, status1);
}

static void bm92t_hw_init(struct bm92t_info *info)
{
	bm92t_update16(info, VENDOR_CONFIG_REG, VENDOR_CONFIG_OCP_DISABLE, 0);
	bm92t_update16(info, CONFIG1_REG, CONFIG1_SPDSRC,
		       FIELD_PREP(CONFIG1_SPDSRC, SPDSRC12_ON));
}

static int bm92t_service(struct bm92t_info *info)
{
	u16 alert, status1, status2, dp_status;
	int ret;

	ret = bm92t_read16(info, ALERT_STATUS_REG, &alert);
	if (ret)
		return ret;

	/* The read cleared it: keep it until the event is handled */
	info->alert |= alert;

	ret = bm92t_read16(info, STATUS1_REG, &status1) ?:
	      bm92t_read16(info, STATUS2_REG, &status2) ?:
	      bm92t_read16(info, DP_STATUS_REG, &dp_status);
	if (ret)
		return ret;

	alert = info->alert;
	WRITE_ONCE(info->last_alert, alert);

	dev_dbg(info->dev, "alert %04x status1 %04x status2 %04x dp %04x %s\n",
		alert, status1, status2, dp_status,
		bm92t_state_names[info->state]);

	if (alert & ALERT_SNK_FAULT)
		dev_err(info->dev, "sink fault\n");
	if (alert & ALERT_SRC_FAULT)
		dev_err(info->dev, "source fault\n");

	/* A reset returns the controller to defaults */
	if (info->rearm_after_reset) {
		info->rearm_after_reset = false;
		msleep(100);
		bm92t_hw_init(info);
	}

	if (FIELD_GET(STATUS1_FAULT, status1)) {
		info->alert = 0;
		/* A faulted controller may never raise another alert */
		info->poll = true;
		info->poll_at = jiffies + msecs_to_jiffies(BM92T_FAULT_POLL_MS);
		/* Our own VBUS would keep VSAFE up */
		if (info->attached)
			bm92t_detach(info);
		if (status1 & (STATUS1_INSERT | STATUS1_VSAFE)) {
			dev_warn_once(info->dev,
				      "fault on an occupied port, waiting for unplug\n");
			return 0;
		}

		/* A wedged partner reads empty too: reset until it is gone */
		if (!info->faulted)
			dev_err(info->dev, "internal fault %lu, resetting\n",
				FIELD_GET(STATUS1_FAULT, status1));
		info->faulted = true;
		bm92t_write16(info, COMMAND_REG, SYS_RESET_CMD);
		msleep(33);
		bm92t_read16(info, ALERT_STATUS_REG, &alert);
		info->rearm_after_reset = true;
		return 0;
	}
	info->faulted = false;

	if (alert & ALERT_PLUGPULL)
		info->vbus_blocked = false;

	/* Hard-reset only while sourcing: a sink negotiation flags it too */
	if ((alert & ALERT_SRC_FAULT) && (status1 & STATUS1_SRC_MODE)) {
		bm92t_write16(info, COMMAND_REG, PD_HARD_RST_CMD);
		info->vbus_blocked = true;
		info->negotiated = false;
		bm92t_set_state(info, BM92T_IDLE);
	}

	if (!(status1 & STATUS1_INSERT)) {
		info->alert = 0;
		if (info->attached)
			bm92t_detach(info);
		else if (bm92t_set_usb_role(info, USB_ROLE_NONE))
			bm92t_retry_soon(info);
		return 0;
	}

	if (!info->attached) {
		bm92t_attach(info, status1, status2);

		/* Attached before we looked: nothing will announce the source */
		if (!(alert & ALERT_PLUGPULL) && !(status1 & STATUS1_SRC_MODE))
			bm92t_write16(info, COMMAND_REG, GET_SRC_CAP_CMD);
	}

	bm92t_update_roles(info, status1, status2);

	if (!(status1 & STATUS1_SRC_MODE)) {
		ret = bm92t_update_contract(info);
		if (ret)
			return ret;
	}

	info->alert = 0;

	if (alert != ALERT_DP_EVENT)
		bm92t_step(info, alert, status1, dp_status);

	return 0;
}

static void bm92t_arm_timer(struct bm92t_info *info)
{
	unsigned long next = 0;

	if (info->state != BM92T_IDLE)
		next = info->deadline;
	if (info->poll && (!next || time_before(info->poll_at, next)))
		next = info->poll_at;
	if (info->raise_at && (!next || time_before(info->raise_at, next)))
		next = info->raise_at;

	mutex_lock(&info->lock);
	if (!info->stopped && next)
		mod_delayed_work(info->wq, &info->timer,
				 time_after(next, jiffies) ? next - jiffies : 0);
	else if (!info->stopped)
		cancel_delayed_work(&info->timer);
	mutex_unlock(&info->lock);
}

/* The only context that talks to the controller */
static void bm92t_run(struct bm92t_info *info)
{
	bool irq_owned;

	mutex_lock(&info->lock);
	if (info->stopped) {
		mutex_unlock(&info->lock);
		return;
	}
	mutex_unlock(&info->lock);

	irq_owned = READ_ONCE(info->irq_pending);
	if (irq_owned || (info->poll && !time_before(jiffies, info->poll_at))) {
		if (bm92t_service(info)) {
			bm92t_retry_soon(info);
		} else {
			if (info->poll && !time_before(jiffies, info->poll_at))
				info->poll = false;
			if (irq_owned) {
				WRITE_ONCE(info->irq_pending, false);
				enable_irq(info->client->irq);
			}
		}
	}

	if (info->state != BM92T_IDLE && !time_before(jiffies, info->deadline))
		bm92t_timeout(info);

	bm92t_raise_limit(info);

	bm92t_arm_timer(info);
}

static void bm92t_work(struct work_struct *work)
{
	bm92t_run(container_of(work, struct bm92t_info, work));
}

static void bm92t_timer(struct work_struct *work)
{
	bm92t_run(container_of(to_delayed_work(work), struct bm92t_info,
			       timer));
}

static void bm92t_kick(struct bm92t_info *info)
{
	queue_work(info->wq, &info->work);
}

static irqreturn_t bm92t_irq(int irq, void *data)
{
	struct bm92t_info *info = data;

	/* The alert is a level: keep it masked until it is read */
	disable_irq_nosync(irq);
	WRITE_ONCE(info->irq_pending, true);
	bm92t_kick(info);

	return IRQ_HANDLED;
}

/* Stop the work item and keep it stopped: nothing touches the chip after */
static void bm92t_stop(struct bm92t_info *info)
{
	disable_irq(info->client->irq);

	mutex_lock(&info->lock);
	info->stopped = true;
	mutex_unlock(&info->lock);

	cancel_work_sync(&info->work);
	cancel_delayed_work_sync(&info->timer);
}

static void bm92t_start(struct bm92t_info *info)
{
	mutex_lock(&info->lock);
	info->stopped = false;
	mutex_unlock(&info->lock);

	info->poll = true;
	info->poll_at = jiffies;
	enable_irq(info->client->irq);
	bm92t_kick(info);
}

static int bm92t_regs_show(struct seq_file *s, void *data)
{
	static const struct {
		const char *name;
		u8 reg;
		bool wide;
	} regs[] = {
		{ "STATUS1", STATUS1_REG },
		{ "STATUS2", STATUS2_REG },
		{ "DP_STATUS", DP_STATUS_REG },
		{ "CONFIG1", CONFIG1_REG },
		{ "CONFIG2", CONFIG2_REG },
		{ "SYS_CONFIG1", SYS_CONFIG1_REG },
		{ "SYS_CONFIG2", SYS_CONFIG2_REG },
		{ "SYS_CONFIG3", SYS_CONFIG3_REG },
		{ "VENDOR_CONFIG", VENDOR_CONFIG_REG },
		{ "DEV_CAPS", DEV_CAPS_REG },
		{ "ALERT_ENABLE", ALERT_ENABLE_REG },
		{ "DP_ALERT_EN", DP_ALERT_EN_REG },
		{ "AUTO_NGT_FIXED", AUTO_NGT_FIXED_REG, true },
		{ "AUTO_NGT_BATT", AUTO_NGT_BATT_REG, true },
		{ "CURRENT_PDO", CURRENT_PDO_REG, true },
		{ "CURRENT_RDO", CURRENT_RDO_REG, true },
	};
	struct bm92t_info *info = s->private;
	unsigned int i;
	int ret;

	/* ALERT_STATUS clears on read: show what the work item last saw */
	seq_printf(s, "%-15s 0x%04x\n", "ALERT_STATUS",
		   READ_ONCE(info->last_alert));

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u32 val32;
		u16 val16;

		if (regs[i].wide) {
			ret = bm92t_read32(info, regs[i].reg, &val32);
			if (!ret)
				seq_printf(s, "%-15s 0x%08x\n", regs[i].name,
					   val32);
		} else {
			ret = bm92t_read16(info, regs[i].reg, &val16);
			if (!ret)
				seq_printf(s, "%-15s 0x%04x\n", regs[i].name,
					   val16);
		}
		if (ret)
			return ret;
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bm92t_regs);

static int bm92t_state_show(struct seq_file *s, void *data)
{
	struct bm92t_info *info = s->private;

	seq_printf(s, "%s\n", bm92t_state_names[READ_ONCE(info->state)]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bm92t_state);

static void bm92t_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

static int bm92t_register_port(struct bm92t_info *info)
{
	struct typec_capability cap = {
		.pd_revision = BM92T_PD_REVISION,
		.svdm_version = SVDM_VER_1_0,
		.orientation_aware = 1,
		.no_mode_control = true,
		.driver_data = info,
	};
	unsigned int i;
	int ret;

	ret = typec_get_fw_cap(&cap, info->connector);
	if (ret)
		return ret;

	info->port = typec_register_port(info->dev, &cap);
	if (IS_ERR(info->port))
		return PTR_ERR(info->port);

	typec_port_register_altmodes(info->port, NULL, info,
				     info->port_altmode,
				     ARRAY_SIZE(info->port_altmode));
	for (i = 0; i < ARRAY_SIZE(info->port_altmode); i++)
		if (info->port_altmode[i] &&
		    info->port_altmode[i]->svid == USB_TYPEC_DP_SID)
			info->port_dp = info->port_altmode[i];

	return 0;
}

static void bm92t_unregister_port(struct bm92t_info *info)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(info->port_altmode); i++)
		bm92t_unregister_altmode(&info->port_altmode[i]);
	typec_unregister_port(info->port);
}

static int bm92t_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	struct bm92t_info *info;
	u16 fw_type, fw_rev, status1;
	int ret;

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "no alert interrupt\n");

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->client = client;
	mutex_init(&info->lock);
	INIT_WORK(&info->work, bm92t_work);
	INIT_DELAYED_WORK(&info->timer, bm92t_timer);
	i2c_set_clientdata(client, info);

	ret = bm92t_read16(info, FW_TYPE_REG, &fw_type) ?:
	      bm92t_read16(info, FW_REVISION_REG, &fw_rev);
	if (ret)
		return dev_err_probe(dev, ret, "controller not responding\n");
	dev_dbg(dev, "firmware type 0x%04x revision 0x%04x\n", fw_type, fw_rev);

	info->connector = device_get_named_child_node(dev, "connector");
	if (!info->connector)
		return dev_err_probe(dev, -ENODEV, "no connector\n");

	ret = fwnode_property_count_u32(info->connector, "sink-pdos");
	if (ret <= 0 || ret > PDO_MAX_OBJECTS) {
		ret = dev_err_probe(dev, -EINVAL, "invalid sink-pdos\n");
		goto err_fwnode;
	}
	info->nr_sink_pdos = ret;
	ret = fwnode_property_read_u32_array(info->connector, "sink-pdos",
					     info->sink_pdos,
					     info->nr_sink_pdos);
	if (ret)
		goto err_fwnode;

	info->vbus = devm_of_regulator_get_optional(dev,
						    to_of_node(info->connector),
						    "vbus");
	if (IS_ERR(info->vbus)) {
		ret = PTR_ERR(info->vbus);
		if (ret != -ENODEV) {
			dev_err_probe(dev, ret, "failed to get VBUS\n");
			goto err_fwnode;
		}
		info->vbus = NULL;
	}

	info->role_sw = fwnode_usb_role_switch_get(info->connector);
	if (IS_ERR_OR_NULL(info->role_sw)) {
		ret = dev_err_probe(dev, info->role_sw ?
				    PTR_ERR(info->role_sw) : -EPROBE_DEFER,
				    "no USB role switch\n");
		goto err_fwnode;
	}
	info->usb_role = usb_role_switch_get_role(info->role_sw);

	info->wq = alloc_ordered_workqueue("bm92t", 0);
	if (!info->wq) {
		ret = -ENOMEM;
		goto err_role_sw;
	}
	ret = devm_add_action_or_reset(dev, bm92t_destroy_wq, info->wq);
	if (ret)
		goto err_role_sw;

	/* The connector supplies the charger: that is what it names */
	psy_cfg.drv_data = info;
	psy_cfg.fwnode = info->connector;
	info->psy = devm_power_supply_register(dev, &bm92t_psy_desc, &psy_cfg);
	if (IS_ERR(info->psy)) {
		ret = PTR_ERR(info->psy);
		goto err_role_sw;
	}

	ret = bm92t_register_port(info);
	if (ret)
		goto err_role_sw;

	/* Leave the role alone until the port says what it is */
	ret = devm_request_irq(dev, client->irq, bm92t_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), info);
	if (ret)
		goto err_port;

	/* A command a previous driver left behind would block ours */
	if (!bm92t_read16(info, STATUS1_REG, &status1) &&
	    (status1 & STATUS1_INSERT) &&
	    FIELD_GET(STATUS1_LASTCMD, status1) != LASTCMD_COMPLETE)
		bm92t_write16(info, COMMAND_REG, ABORT_LASTCMD_SENT_CMD);

	bm92t_write16(info, DP_ALERT_EN_REG, 0);
	bm92t_hw_init(info);

	info->debugfs = debugfs_create_dir("bm92txx", NULL);
	debugfs_create_file("regs", 0400, info->debugfs, info, &bm92t_regs_fops);
	debugfs_create_file("state", 0444, info->debugfs, info,
			    &bm92t_state_fops);

	bm92t_start(info);

	return 0;

err_port:
	bm92t_unregister_port(info);
err_role_sw:
	usb_role_switch_put(info->role_sw);
err_fwnode:
	fwnode_handle_put(info->connector);
	return ret;
}

/* Hand the port back in a defined state */
static void bm92t_release(struct bm92t_info *info)
{
	if (info->attached)
		bm92t_detach(info);
	bm92t_set_vbus(info, false);
	bm92t_set_usb_role(info, USB_ROLE_NONE);
}

static void bm92t_remove(struct i2c_client *client)
{
	struct bm92t_info *info = i2c_get_clientdata(client);

	bm92t_stop(info);
	debugfs_remove_recursive(info->debugfs);
	bm92t_release(info);
	bm92t_unregister_port(info);
	usb_role_switch_put(info->role_sw);
	fwnode_handle_put(info->connector);
}

static void bm92t_shutdown(struct i2c_client *client)
{
	struct bm92t_info *info = i2c_get_clientdata(client);

	bm92t_stop(info);
	bm92t_update16(info, CONFIG1_REG, CONFIG1_SPDSRC,
		       FIELD_PREP(CONFIG1_SPDSRC, SPDSRC12_OFF));
	bm92t_set_vbus(info, false);
}

static int bm92t_suspend(struct device *dev)
{
	struct bm92t_info *info = dev_get_drvdata(dev);

	bm92t_stop(info);

	return 0;
}

static int bm92t_resume(struct device *dev)
{
	bm92t_start(dev_get_drvdata(dev));

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(bm92t_pm_ops, bm92t_suspend, bm92t_resume);

static const struct of_device_id bm92t_of_match[] = {
	{ .compatible = "rohm,bm92t36" },
	{ }
};
MODULE_DEVICE_TABLE(of, bm92t_of_match);

static const struct i2c_device_id bm92t_id[] = {
	{ "bm92t36" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bm92t_id);

static struct i2c_driver bm92t_driver = {
	.driver = {
		.name = "bm92t",
		.of_match_table = bm92t_of_match,
		.pm = pm_sleep_ptr(&bm92t_pm_ops),
	},
	.id_table = bm92t_id,
	.probe = bm92t_probe,
	.remove = bm92t_remove,
	.shutdown = bm92t_shutdown,
};
module_i2c_driver(bm92t_driver);

MODULE_AUTHOR("CTCaer <ctcaer@gmail.com>");
MODULE_DESCRIPTION("ROHM BM92T36 USB Type-C / USB PD controller driver");
MODULE_LICENSE("GPL");
