// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ROHM BM92T36 USB Type-C / USB Power Delivery controller
 *
 * Copyright (c) 2020-2023 CTCaer <ctcaer@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
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
#define PDOS_SNK_CONS_REG	0x33	/* Provisioned sink PDOs, 16 bytes */
#define PDOS_SRC_PROV_REG	0x3C	/* Provisioned source PDOs, 28 bytes */
#define FW_TYPE_REG		0x4B
#define FW_REVISION_REG		0x4C
#define INCOMING_VDM_REG	0x50	/* 28 bytes */
#define OUTGOING_VDM_REG	0x60	/* 28 bytes */

/* ALERT_STATUS_REG */
#define ALERT_SNK_FAULT		BIT(0)
#define ALERT_SRC_FAULT		BIT(1)
#define ALERT_CMD_DONE		BIT(2)
#define ALERT_PLUGPULL		BIT(3)
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
#define STATUS1_CMD_BUSY	BIT(13)

#define LASTCMD_COMPLETE	0
#define LASTCMD_ABORTED		2

#define DATA_ROLE_UFP		1
#define DATA_ROLE_DFP		2

/* STATUS2_REG */
#define STATUS2_VDM_IN		BIT(0)	/* a VDM waits for ACCEPT_VDM */
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
#define BM92T_RESET_SETTLE_MS	100
#define BM92T_PLUG_SETTLE_MS	500
#define BM92T_RAISE_DELAY_MS	2000
#define BM92T_REQ_TIMEOUT_MS	3000
#define BM92T_DP_CFG_RETRIES	3
#define BM92T_RDO_RETRIES	2
#define BM92T_SUBMIT_TRIES	3
#define BM92T_ABORT_TRIES	3
#define BM92T_STUCK_POLL_MS	1000
#define BM92T_DP_POLL_MS	1000
#define BM92T_DP_REPLY_RETRY_MS	20
#define BM92T_DP_REPLY_SLOTS	4

/* Nintendo dock protocol */
#define USB_SID_NINTENDO	0x057E
#define PID_NINTENDO_DOCK	0x2003

#define ND_READ			0
#define ND_WRITE		1
#define ND_HOST			1
#define ND_DOCK			2
#define ND_BUSY			BIT(0)

#define ND_CMD_LED		0x01
#define ND_CMD_DEVICE_STATE	0x16
#define ND_CMD_DP_SIGNAL_DISABLE 0x1C
#define ND_CMD_HUB_CONTROL	0x20

#define ND_HUB_RETRIES		10
#define ND_HUB_DELAY_MS		500
#define ND_HUB_RETRY_MS		250
#define ND_REARM_DELAY_MS	500

#define BM92T_DP_ACK(cmd)	(VDO(USB_TYPEC_DP_SID, 1, SVDM_VER_1_0, \
				     VDO_OPOS(USB_TYPEC_DP_MODE) | \
				     VDO_CMDT(CMDT_RSP_ACK) | (cmd)))

enum bm92t_state {
	BM92T_IDLE,
	BM92T_ABORT_SENT,
	BM92T_ABORT_WRITE,
	BM92T_STUCK,
	BM92T_DRAIN_ACCEPT,
	BM92T_SRC_CAP_SENT,
	BM92T_HARD_RST_SENT,
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
	BM92T_DP_QUERY_SENT,
	BM92T_DP_QUERY_ACCEPT,
	BM92T_DP_CFG_SENT,
	BM92T_ND_ENTER_SENT,
	BM92T_ND_ENTER_ACCEPT,
	BM92T_ND_QUERY_SENT,
	BM92T_ND_QUERY_ACCEPT,
	BM92T_ND_LED_SENT,
	BM92T_ND_LED_ACCEPT,
	BM92T_ND_HUB_WAIT,
	BM92T_ND_HUB_SENT,
	BM92T_ND_HUB_ACCEPT,
	BM92T_ND_REQ_SENT,
	BM92T_ND_REQ_ACCEPT,
	BM92T_NUM_STATES
};

static const char * const bm92t_state_names[] = {
	[BM92T_IDLE]		= "idle",
	[BM92T_ABORT_SENT]	= "abort-sent",
	[BM92T_ABORT_WRITE]	= "abort-write",
	[BM92T_STUCK]		= "stuck",
	[BM92T_DRAIN_ACCEPT]	= "drain-accept",
	[BM92T_SRC_CAP_SENT]	= "src-cap-sent",
	[BM92T_HARD_RST_SENT]	= "hard-rst-sent",
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
	[BM92T_DP_QUERY_SENT]	= "dp-query-sent",
	[BM92T_DP_QUERY_ACCEPT]	= "dp-query-accept",
	[BM92T_DP_CFG_SENT]	= "dp-cfg-sent",
	[BM92T_ND_ENTER_SENT]	= "nd-enter-sent",
	[BM92T_ND_ENTER_ACCEPT]	= "nd-enter-accept",
	[BM92T_ND_QUERY_SENT]	= "nd-query-sent",
	[BM92T_ND_QUERY_ACCEPT]	= "nd-query-accept",
	[BM92T_ND_LED_SENT]	= "nd-led-sent",
	[BM92T_ND_LED_ACCEPT]	= "nd-led-accept",
	[BM92T_ND_HUB_WAIT]	= "nd-hub-wait",
	[BM92T_ND_HUB_SENT]	= "nd-hub-sent",
	[BM92T_ND_HUB_ACCEPT]	= "nd-hub-accept",
	[BM92T_ND_REQ_SENT]	= "nd-req-sent",
	[BM92T_ND_REQ_ACCEPT]	= "nd-req-accept",
};

static_assert(ARRAY_SIZE(bm92t_state_names) == BM92T_NUM_STATES);

/* States waiting for the partner's reply to a VDM of ours */
static const unsigned long bm92t_vdm_states =
	BIT(BM92T_DISC_ID_SENT) | BIT(BM92T_DISC_SVID_SENT) |
	BIT(BM92T_DISC_MODE_SENT) | BIT(BM92T_DP_QUERY_SENT) |
	BIT(BM92T_ND_ENTER_SENT) | BIT(BM92T_ND_QUERY_SENT) |
	BIT(BM92T_ND_LED_SENT) | BIT(BM92T_ND_HUB_SENT) |
	BIT(BM92T_ND_REQ_SENT);

static_assert(BM92T_NUM_STATES <= BITS_PER_LONG);

enum bm92t_req_state {
	BM92T_REQ_NONE,
	BM92T_REQ_PENDING,
	BM92T_REQ_ACTIVE,
	BM92T_REQ_DONE,
};

/* A Nintendo dock request: unstructured VDM payload after the header */
struct bm92t_nd_msg {
	u8 data[8];
	u8 len;
};

static const struct {
	const char *name;
	u8 reg;
	u8 len;
} bm92t_dump_regs[] = {
	{ "STATUS1", STATUS1_REG, 2 },
	{ "STATUS2", STATUS2_REG, 2 },
	{ "DP_STATUS", DP_STATUS_REG, 2 },
	{ "CONFIG1", CONFIG1_REG, 2 },
	{ "CONFIG2", CONFIG2_REG, 2 },
	{ "SYS_CONFIG1", SYS_CONFIG1_REG, 2 },
	{ "SYS_CONFIG2", SYS_CONFIG2_REG, 2 },
	{ "SYS_CONFIG3", SYS_CONFIG3_REG, 2 },
	{ "VENDOR_CONFIG", VENDOR_CONFIG_REG, 2 },
	{ "DEV_CAPS", DEV_CAPS_REG, 2 },
	{ "ALERT_ENABLE", ALERT_ENABLE_REG, 2 },
	{ "DP_ALERT_EN", DP_ALERT_EN_REG, 2 },
	{ "AUTO_NGT_FIXED", AUTO_NGT_FIXED_REG, 5 },
	{ "AUTO_NGT_BATT", AUTO_NGT_BATT_REG, 5 },
	{ "CURRENT_PDO", CURRENT_PDO_REG, 5 },
	{ "CURRENT_RDO", CURRENT_RDO_REG, 5 },
};

/* State of one attachment, cleared when it begins and when it ends */
struct bm92t_attachment {
	bool attached;
	bool source;
	bool want_src_cap;
	unsigned int caps_gen;		/* source caps announced */
	unsigned int asked_gen;		/* the caps our last request used */
	bool contract_seen;		/* a contract we did not ask for */
	unsigned long negotiate_at;
	bool negotiated;
	unsigned int rdo_retries;
	bool ps_rdy_due;		/* contract in before the RDO completed */
	unsigned int fails;		/* consecutive failed submissions */
	u16 replay;			/* events a failed step runs again on */
	enum bm92t_state replay_state;
	bool want_discover;
	unsigned long discover_at;
	bool discovered;
	bool partner_drd;
	bool vdm_pending;		/* an unsolicited partner VDM */

	u32 dp_vdo;
	u32 dp_conf;
	unsigned int dp_cfg_retries;
	bool dp_tried;
	bool dp_live;
	bool dp_exited;
	bool dp_hpd;

	bool nintendo;
	bool nd_setup_due;
	unsigned int hub_retries;
	u8 nd_cmd;
	bool rearm;
	unsigned long rearm_at;
};

struct bm92t_info {
	struct device *dev;
	struct i2c_client *client;
	struct workqueue_struct *wq;
	struct work_struct work;
	struct delayed_work timer;

	/* Fields below are owned by the work item */
	enum bm92t_state state;
	unsigned long deadline;
	unsigned int abort_tries;
	bool stuck;			/* reported, until an abort completes */
	unsigned int cmd_seq;		/* commands written */
	unsigned int done_seq;		/* cmd_seq when CMD_DONE was read */
	unsigned long busy_since;	/* CMD_BUSY with nothing of ours */
	bool uncertain;			/* the last service failed */
	u16 alert;
	u16 status1;
	u16 status2;
	u16 dp_status;
	bool irq_pending;		/* the hard handler masked the IRQ */
	bool poll;
	unsigned long poll_at;
	bool faulted;
	bool hw_init_pending;
	unsigned long hw_init_at;
	bool vbus_on;
	bool vbus_blocked;
	enum usb_role usb_role;
	unsigned int limit_ma;		/* published to the charger */
	unsigned int pending_ma;
	u32 pending_pdo, pending_rdo;	/* the contract pending_ma is from */
	unsigned long raise_at;
	struct bm92t_attachment att;

	/* Fields below are protected by lock */
	struct mutex lock;
	bool stopped;
	bool nd_ready;			/* a dock attachment takes requests */
	enum bm92t_req_state req_state;
	unsigned int req_id;		/* the request in req_msg */
	unsigned int active_req;	/* the request being sent */
	struct bm92t_nd_msg req_msg;
	int req_status;
	struct completion req_done;
	bool dump_wanted;
	int dump_status;
	u8 dump[ARRAY_SIZE(bm92t_dump_regs)][5];
	struct completion dump_done;
	bool dp_enter_wanted;
	bool dp_reply_ok;
	bool dp_told;			/* the DP alt mode driver has the Enter */
	struct {
		u32 hdr;
		u32 vdo;
		int count;
	} dp_replies[BM92T_DP_REPLY_SLOTS];
	unsigned int dp_nr_replies;
	u32 dp_status_vdo;
	u32 dp_pin;
	struct delayed_work dp_work;
	struct mutex req_mutex;		/* one synchronous caller at a time */

	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_pd_identity partner_ident;
	struct typec_cable *cable;
	struct typec_plug *plug;
	struct typec_altmode *port_altmode[2];
	struct typec_altmode *port_dp;
	struct typec_altmode *port_nd;
	struct typec_altmode *partner_dp;
	struct typec_altmode *partner_nd;
	struct fwnode_handle *connector;

	struct usb_role_switch *role_sw;
	struct regulator *vbus;
	struct power_supply *psy;
	u32 sink_pdos[PDO_MAX_OBJECTS];
	unsigned int nr_sink_pdos;
	unsigned int op_sink_mw;

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

/* A length byte, then nothing or one little-endian word */
static int bm92t_read32(struct bm92t_info *info, u8 reg, u32 *val)
{
	u8 buf[5];
	int ret;

	ret = bm92t_read(info, reg, buf, sizeof(buf));
	if (ret)
		return ret;

	if (buf[0] != 0 && buf[0] != 4) {
		dev_err_ratelimited(info->dev, "0x%02x: length %u\n",
				    reg, buf[0]);
		return -EPROTO;
	}

	*val = buf[0] ? get_unaligned_le32(&buf[1]) : 0;

	return 0;
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

static void bm92t_set_state_ms(struct bm92t_info *info,
			       enum bm92t_state state, unsigned int ms)
{
	WRITE_ONCE(info->state, state);
	info->deadline = jiffies + msecs_to_jiffies(ms);
	dev_dbg(info->dev, "state %s\n", bm92t_state_names[state]);
}

static void bm92t_set_state(struct bm92t_info *info, enum bm92t_state state)
{
	bm92t_set_state_ms(info, state, BM92T_CMD_TIMEOUT_MS);
}

/* The mailbox is ours once the controller is done with the last command */
static int bm92t_mailbox_free(struct bm92t_info *info)
{
	u16 status1;
	int ret;

	ret = bm92t_read16(info, STATUS1_REG, &status1);
	if (ret)
		return ret;

	if (!(status1 & STATUS1_CMD_BUSY)) {
		info->busy_since = 0;
		return 0;
	}

	if (!info->busy_since)
		info->busy_since = jiffies ?: 1;

	return -EBUSY;
}

/* Completions in a snapshot taken before this belong to older commands */
static int bm92t_write_cmd(struct bm92t_info *info, u16 cmd)
{
	int ret;

	info->cmd_seq++;
	ret = bm92t_write16(info, COMMAND_REG, cmd);
	dev_dbg(info->dev, "cmd 0x%04x: %d\n", cmd, ret);

	return ret;
}

static int __bm92t_send_cmd(struct bm92t_info *info, u16 cmd,
			    enum bm92t_state state)
{
	int ret;

	ret = bm92t_write_cmd(info, cmd);
	if (!ret) {
		info->att.fails = 0;
		bm92t_set_state(info, state);
	}

	return ret;
}

/* Enter @state only when the command that leads to it went out */
static int bm92t_send_cmd(struct bm92t_info *info, u16 cmd,
			  enum bm92t_state state)
{
	return bm92t_mailbox_free(info) ?: __bm92t_send_cmd(info, cmd, state);
}

static int bm92t_send_vdm(struct bm92t_info *info, u32 hdr, const void *data,
			  size_t len, enum bm92t_state state)
{
	u8 buf[2 + 4 + 8];

	if (WARN_ON(len > sizeof(buf) - 6))
		return -EINVAL;

	buf[0] = OUTGOING_VDM_REG;
	buf[1] = 4 + len;
	put_unaligned_le32(hdr, &buf[2]);
	memcpy(&buf[6], data, len);

	return bm92t_mailbox_free(info) ?:
	       bm92t_write(info, buf, 6 + len) ?:
	       __bm92t_send_cmd(info, SEND_VDM_CMD, state);
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
	unsigned int i, ma = 0;

	for (i = 0; i < info->nr_sink_pdos; i++) {
		u32 pdo = info->sink_pdos[i];

		if (pdo_type(pdo) == PDO_TYPE_FIXED) {
			if (pdo_fixed_voltage(pdo) == mv)
				ma = max(ma, pdo_fixed_current(pdo));
		} else if (pdo_min_voltage(pdo) <= mv &&
			   mv <= pdo_max_voltage(pdo)) {
			ma = max(ma, pdo_max_current(pdo));
		}
	}

	return ma;
}

static int bm92t_psy_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct bm92t_info *info = power_supply_get_drvdata(psy);
	unsigned int ma = READ_ONCE(info->limit_ma);

	if (psp != POWER_SUPPLY_PROP_CURRENT_MAX)
		return -EINVAL;

	/* No usable sink contract: the charger's own detection decides */
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

static void bm92t_publish_limit(struct bm92t_info *info, unsigned int ma)
{
	if (ma == info->limit_ma)
		return;

	dev_info(info->dev, "input current limit %u mA\n", ma);
	WRITE_ONCE(info->limit_ma, ma);
	power_supply_changed(info->psy);
}

/* Lower the charger's input limit at once, raise it once a contract settled */
static void bm92t_set_limit(struct bm92t_info *info, unsigned int ma, u32 pdo,
			    u32 rdo)
{
	bool same = pdo == info->pending_pdo && rdo == info->pending_rdo;

	info->pending_ma = ma;
	info->pending_pdo = pdo;
	info->pending_rdo = rdo;

	if (!ma || ma <= info->limit_ma) {
		info->raise_at = 0;
		bm92t_publish_limit(info, ma);
	} else if (!info->raise_at || !same) {
		info->raise_at = jiffies + msecs_to_jiffies(BM92T_RAISE_DELAY_MS);
	}
}

/* Called right after a read of the contract the raise is for */
static void bm92t_raise_limit(struct bm92t_info *info)
{
	if (!info->raise_at || time_before(jiffies, info->raise_at))
		return;

	info->raise_at = 0;
	bm92t_publish_limit(info, info->pending_ma);
}

/* A fixed supply contract the board can take: its current, else 0 */
static unsigned int bm92t_contract_ma(struct bm92t_info *info, u32 pdo,
				      u32 rdo)
{
	unsigned int mv = pdo_fixed_voltage(pdo);
	unsigned int ma = rdo_op_current(rdo);
	unsigned int snk_ma;

	if (pdo_type(pdo) != PDO_TYPE_FIXED || !rdo_index(rdo) || !mv ||
	    ma > pdo_fixed_current(pdo)) {
		dev_warn_ratelimited(info->dev,
				     "unusable contract PDO %08x RDO %08x\n",
				     pdo, rdo);
		return 0;
	}

	snk_ma = bm92t_sink_ma(info, mv);
	if (!snk_ma) {
		dev_warn_ratelimited(info->dev,
				     "contract at %u mV is not in sink-pdos\n",
				     mv);
		return 0;
	}

	return min(ma, snk_ma);
}

static int bm92t_read_contract(struct bm92t_info *info, u32 *pdo, u32 *rdo)
{
	*pdo = 0;

	return bm92t_read32(info, CURRENT_RDO_REG, rdo) ?:
	       (*rdo ? bm92t_read32(info, CURRENT_PDO_REG, pdo) : 0);
}

/* Follow the contract the controller holds now */
static void bm92t_update_contract(struct bm92t_info *info, u32 pdo, u32 rdo)
{
	unsigned int ma = 0;

	if (rdo) {
		typec_set_pwr_opmode(info->port, TYPEC_PWR_MODE_PD);
		if (info->partner)
			typec_partner_set_pd_revision(info->partner,
						      BM92T_PD_REVISION);
	} else {
		typec_set_pwr_opmode(info->port, TYPEC_PWR_MODE_USB);
	}

	/* The charger takes nothing from a port that sources */
	if (info->att.source)
		return;

	if (rdo)
		ma = bm92t_contract_ma(info, pdo, rdo);

	if (pdo != info->pending_pdo || rdo != info->pending_rdo)
		dev_dbg(info->dev, "contract PDO %08x RDO %08x: %u mA\n",
			pdo, rdo, ma);

	bm92t_set_limit(info, ma, pdo, rdo);
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
		unsigned int mv, src_ma, ma;

		if (i == 0)
			info->att.partner_drd = pdo & PDO_FIXED_DATA_SWAP;

		if (pdo_type(pdo) != PDO_TYPE_FIXED)
			continue;

		mv = pdo_fixed_voltage(pdo);
		src_ma = pdo_fixed_current(pdo);
		ma = min(src_ma, bm92t_sink_ma(info, mv));
		dev_dbg(info->dev, "source PDO %u: %u mV %u mA\n",
			i + 1, mv, src_ma);

		if (ma && mv * ma / 1000 >= best_mw) {
			best = i + 1;
			best_mw = mv * ma / 1000;
			best_ma = ma;
		}
	}

	if (!best)
		return -ENODEV;

	if (best_mw < info->op_sink_mw)
		flags = RDO_CAP_MISMATCH;

	rdo = RDO_FIXED(best, best_ma, best_ma, RDO_USB_COMM | flags);

	dev_dbg(info->dev, "requesting PDO %u at %u mA, %u mW%s\n", best,
		best_ma, best_mw, flags ? ", capability mismatch" : "");

	buf[0] = SET_RDO_REG;
	buf[1] = 4;
	put_unaligned_le32(rdo, &buf[2]);

	return bm92t_mailbox_free(info) ?:
	       bm92t_write(info, buf, 6) ?:
	       __bm92t_send_cmd(info, SEND_RDO_CMD, BM92T_RDO_SENT);
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

static void bm92t_retry_at(struct bm92t_info *info, unsigned int ms)
{
	unsigned long at = jiffies + msecs_to_jiffies(ms);

	if (!info->poll || time_before(at, info->poll_at))
		info->poll_at = at;
	info->poll = true;
}

static void bm92t_retry_soon(struct bm92t_info *info)
{
	bm92t_retry_at(info, BM92T_RETRY_MS);
}

static void bm92t_discover_later(struct bm92t_info *info, unsigned int ms)
{
	struct bm92t_attachment *att = &info->att;

	att->want_discover = true;
	att->discover_at = jiffies + msecs_to_jiffies(ms);
}

/* Nothing attached: no VBUS from us and no USB role, retried until both hold */
static int bm92t_release_port(struct bm92t_info *info)
{
	int ret = bm92t_set_vbus(info, false);
	int ret2 = bm92t_set_usb_role(info, USB_ROLE_NONE);

	if (ret || ret2)
		bm92t_retry_soon(info);

	return ret ?: ret2;
}

static void bm92t_update_roles(struct bm92t_info *info, u16 status1,
			       u16 status2)
{
	bool source = status1 & STATUS1_SRC_MODE;
	bool sourcing = source && (status2 & STATUS2_OTG_INSERT);
	unsigned int dr = FIELD_GET(STATUS1_DR, status1);
	enum usb_role role = USB_ROLE_NONE;
	bool host;

	typec_set_pwr_role(info->port, source ? TYPEC_SOURCE : TYPEC_SINK);
	typec_set_vconn_role(info->port, status2 & STATUS2_VCONN_ON ?
			     TYPEC_SOURCE : TYPEC_SINK);

	if (bm92t_set_vbus(info, sourcing && !info->vbus_blocked &&
			   !info->hw_init_pending))
		bm92t_retry_soon(info);

	/* Without a reported data role a source is DFP, as at attach */
	if (dr == DATA_ROLE_DFP || dr == DATA_ROLE_UFP)
		host = dr == DATA_ROLE_DFP;
	else if (sourcing)
		host = true;
	else
		goto set_role;

	typec_set_data_role(info->port, host ? TYPEC_HOST : TYPEC_DEVICE);
	if (!host)
		role = USB_ROLE_DEVICE;
	/* Never host a device that our VBUS does not reach */
	else if (!sourcing || info->vbus_on || !info->vbus)
		role = USB_ROLE_HOST;

set_role:
	if (bm92t_set_usb_role(info, role))
		bm92t_retry_soon(info);
}

static void __bm92t_req_done(struct bm92t_info *info, int status)
{
	lockdep_assert_held(&info->lock);

	info->req_status = status;
	info->req_state = BM92T_REQ_DONE;
	complete(&info->req_done);
}

/* The request this transaction sends is done, if its caller still waits */
static void bm92t_req_finish(struct bm92t_info *info, int status)
{
	mutex_lock(&info->lock);
	if (info->req_state == BM92T_REQ_ACTIVE &&
	    info->req_id == info->active_req)
		__bm92t_req_done(info, status);
	mutex_unlock(&info->lock);
}

/* The dock's attachment ends: fail its request, admit no other */
static void bm92t_req_cancel(struct bm92t_info *info, int status)
{
	mutex_lock(&info->lock);
	info->nd_ready = false;
	if (info->req_state == BM92T_REQ_PENDING ||
	    info->req_state == BM92T_REQ_ACTIVE)
		__bm92t_req_done(info, status);
	mutex_unlock(&info->lock);
}

static bool bm92t_in_req(struct bm92t_info *info)
{
	return info->state == BM92T_ND_REQ_SENT ||
	       info->state == BM92T_ND_REQ_ACCEPT;
}

/* Give up on the command in flight: abort it and wait for the mailbox */
static void bm92t_abort(struct bm92t_info *info, int status)
{
	if (bm92t_in_req(info))
		bm92t_req_finish(info, status);

	/* Not written, nothing to complete: written again at the deadline */
	if (bm92t_write_cmd(info, ABORT_LASTCMD_SENT_CMD))
		bm92t_set_state_ms(info, BM92T_ABORT_WRITE, BM92T_RETRY_MS);
	else
		bm92t_set_state(info, BM92T_ABORT_SENT);
}

/* Back to reset defaults, set up again once they settle */
static void bm92t_reset(struct bm92t_info *info)
{
	u16 alert;

	if (bm92t_write_cmd(info, SYS_RESET_CMD))
		return;

	msleep(33);
	bm92t_read16(info, ALERT_STATUS_REG, &alert);
	bm92t_set_state(info, BM92T_IDLE);
	info->busy_since = 0;
	info->hw_init_pending = true;
	info->hw_init_at = jiffies + msecs_to_jiffies(BM92T_RESET_SETTLE_MS);
	bm92t_retry_at(info, BM92T_RESET_SETTLE_MS);
}

static void bm92t_altmode_set_active(struct typec_altmode *alt, bool active)
{
	if (alt && alt->active != active)
		typec_altmode_update_active(alt, active);
}

static void bm92t_unregister_altmode(struct typec_altmode **alt)
{
	if (*alt)
		typec_unregister_altmode(*alt);
	*alt = NULL;
}

/* Register what the port shows of this attachment, as far as it can */
static void bm92t_register_partner(struct bm92t_info *info, u16 status2)
{
	struct typec_partner_desc desc = { .identity = &info->partner_ident };
	struct typec_cable_desc cable = { .type = USB_PLUG_TYPE_C };
	struct typec_plug_desc plug = { .index = TYPEC_PLUG_SOP_P };
	void *obj;

	if (!info->partner) {
		obj = typec_register_partner(info->port, &desc);
		if (IS_ERR(obj)) {
			dev_err(info->dev, "failed to register partner: %pe\n",
				obj);
			bm92t_retry_soon(info);
		} else {
			info->partner = obj;
		}
	}

	/* The controller may learn of the e-marker after the attach */
	if ((status2 & STATUS2_EM_CABLE) && !info->cable) {
		obj = typec_register_cable(info->port, &cable);
		if (IS_ERR(obj)) {
			bm92t_retry_soon(info);
		} else {
			info->cable = obj;
			obj = typec_register_plug(info->cable, &plug);
			if (IS_ERR(obj)) {
				typec_unregister_cable(info->cable);
				info->cable = NULL;
				bm92t_retry_soon(info);
			} else {
				info->plug = obj;
			}
		}
	}
}

static void bm92t_attach(struct bm92t_info *info, u16 status1, u16 status2,
			 u16 alert, bool replaced)
{
	struct bm92t_attachment *att = &info->att;

	memset(att, 0, sizeof(*att));
	memset(&info->partner_ident, 0, sizeof(info->partner_ident));

	att->attached = true;
	att->source = status1 & STATUS1_SRC_MODE;
	att->rdo_retries = BM92T_RDO_RETRIES;

	typec_set_orientation(info->port, status1 & STATUS1_CSIDE ?
			      TYPEC_ORIENTATION_REVERSE :
			      TYPEC_ORIENTATION_NORMAL);
	bm92t_register_partner(info, status2);

	/* Attached before we looked: nothing will announce the partner */
	if (!(alert & ALERT_PLUGPULL) ||
	    (replaced && !(alert & (ALERT_PDO | ALERT_CONTRACT)))) {
		if (att->source)
			bm92t_discover_later(info, 0);
		else
			att->want_src_cap = true;
	}
}

static void bm92t_dp_reply(struct bm92t_info *info, u32 hdr, u32 vdo,
			   int count);

static void bm92t_detach(struct bm92t_info *info)
{
	/* A wedged mailbox stays wedged across partners */
	if (info->state != BM92T_IDLE && info->state != BM92T_STUCK &&
	    info->state != BM92T_ABORT_SENT && info->state != BM92T_ABORT_WRITE)
		bm92t_abort(info, -ENODEV);
	bm92t_req_cancel(info, -ENODEV);

	/* No DP alt mode driver call may reach the objects going away */
	mutex_lock(&info->lock);
	info->dp_reply_ok = false;
	info->dp_enter_wanted = false;
	info->dp_nr_replies = 0;
	info->dp_told = false;
	mutex_unlock(&info->lock);
	cancel_delayed_work_sync(&info->dp_work);

	bm92t_unregister_altmode(&info->partner_dp);
	bm92t_unregister_altmode(&info->partner_nd);
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

	bm92t_set_limit(info, 0, 0, 0);
	memset(&info->att, 0, sizeof(info->att));
	info->vbus_blocked = false;
}

/* Count a failed submission; true once it is time to give up */
static bool bm92t_submit_failed(struct bm92t_info *info, int ret,
				const char *what)
{
	struct bm92t_attachment *att = &info->att;

	/* The partner cannot do it: nothing to retry */
	if (ret == -ENODEV || ret == -EPROTO) {
		att->fails = 0;
		return true;
	}

	/* A busy controller is not a failure: it is still working */
	if (ret == -EBUSY) {
		bm92t_retry_soon(info);
		return false;
	}

	if (++att->fails < BM92T_SUBMIT_TRIES) {
		bm92t_retry_soon(info);
		return false;
	}

	dev_err(info->dev, "%s failed: %d\n", what, ret);
	att->fails = 0;

	return true;
}

static bool bm92t_caps_changed(struct bm92t_info *info)
{
	return info->att.caps_gen != info->att.asked_gen;
}

/* Done with the caps a request used; newer caps still want one */
static void bm92t_negotiation_done(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;

	att->negotiated = true;
	if (!bm92t_caps_changed(info))
		att->contract_seen = false;
	bm92t_discover_later(info, 0);
	bm92t_set_state(info, BM92T_IDLE);
}

/* What discovery finds is registered under the partner: wait for it */
static bool bm92t_discovery_wanted(struct bm92t_info *info)
{
	return info->att.want_discover && !info->att.discovered &&
	       info->partner;
}

/* Ask again after the controller's next contract, a limited number of times */
static void bm92t_renegotiate(struct bm92t_info *info, bool now)
{
	struct bm92t_attachment *att = &info->att;

	if (!att->rdo_retries) {
		bm92t_negotiation_done(info);
		return;
	}

	att->rdo_retries--;
	att->negotiated = false;
	att->contract_seen = now;
	att->negotiate_at = jiffies;
	bm92t_set_state(info, BM92T_IDLE);
}

static bool bm92t_negotiation_wanted(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;

	return !att->source && att->contract_seen &&
	       (bm92t_caps_changed(info) || !att->negotiated);
}

static bool bm92t_negotiation_due(struct bm92t_info *info)
{
	return bm92t_negotiation_wanted(info) &&
	       !time_before(jiffies, info->att.negotiate_at);
}

static void bm92t_negotiate(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;
	unsigned int gen = att->caps_gen;
	int ret;

	ret = bm92t_request_pdo(info);
	if (!ret) {
		att->negotiated = true;
		att->asked_gen = gen;
		att->contract_seen = false;
	} else if (ret == -ENODEV) {
		dev_info(info->dev, "no usable source PDO\n");
		att->asked_gen = gen;
		bm92t_negotiation_done(info);
	} else if (bm92t_submit_failed(info, ret, "contract request")) {
		att->asked_gen = gen;
		bm92t_negotiation_done(info);
	}
}

static int bm92t_send_disc_id(struct bm92t_info *info)
{
	return bm92t_send_vdm(info, VDO(USB_SID_PD, 1, SVDM_VER_1_0,
					CMD_DISCOVER_IDENT),
			      NULL, 0, BM92T_DISC_ID_SENT);
}

/* Discover the partner as DFP, swapping the data role first if we can */
static void bm92t_discover(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;
	unsigned int dr = FIELD_GET(STATUS1_DR, info->status1);
	int ret;

	if (dr == DATA_ROLE_DFP)
		ret = bm92t_send_disc_id(info);
	else if (dr == DATA_ROLE_UFP && att->partner_drd && !att->source)
		ret = bm92t_send_cmd(info, DR_SWAP_CMD, BM92T_DR_SWAP_SENT);
	else
		ret = -ENODEV;	/* a device to this partner */

	if (!ret || bm92t_submit_failed(info, ret, "discovery"))
		att->discovered = true;
}

static int bm92t_nd_send(struct bm92t_info *info, const struct bm92t_nd_msg *msg,
			 enum bm92t_state state)
{
	info->att.nd_cmd = msg->data[2];

	return bm92t_send_vdm(info, VDO(USB_SID_NINTENDO, 0, 0, 0),
			      msg->data, msg->len, state);
}

static void bm92t_nd_cmd(struct bm92t_nd_msg *msg, u8 rw, u8 cmd)
{
	*msg = (struct bm92t_nd_msg) {
		.data = { rw, ND_HOST, cmd, 0 },
		.len = 4,
	};
}

static void bm92t_nd_led(struct bm92t_nd_msg *msg, u8 duty, u8 time_on,
			 u8 time_off, u8 fade)
{
	*msg = (struct bm92t_nd_msg) {
		.data = { ND_WRITE, ND_HOST, ND_CMD_LED, 0,
			  fade, time_off, time_on, duty },
		.len = 8,
	};
}

static void bm92t_nd_dp_sleep(struct bm92t_nd_msg *msg, bool sleep)
{
	bm92t_nd_cmd(msg, sleep, ND_CMD_DP_SIGNAL_DISABLE);
}

/* Check the dock's reply to the last command; returns its status byte */
static int bm92t_nd_reply(struct bm92t_info *info, u8 *vdm, size_t size)
{
	int n;

	n = bm92t_read_vdm(info, vdm, size, USB_SID_NINTENDO, -1);
	if (n < 1)
		return n < 0 ? n : -EPROTO;

	if (vdm[6] != ND_DOCK || vdm[7] != info->att.nd_cmd + 1)
		return -EPROTO;

	return vdm[5];
}

/* A request's result: the dock's status turned into an error code */
static int bm92t_nd_result(struct bm92t_info *info, u8 *vdm, size_t size)
{
	int ret = bm92t_nd_reply(info, vdm, size);

	if (ret < 0)
		return ret;

	return ret & ND_BUSY ? -EBUSY : 0;
}

static int bm92t_nd_hub_enable(struct bm92t_info *info)
{
	struct bm92t_nd_msg msg;

	bm92t_nd_cmd(&msg, ND_WRITE, ND_CMD_HUB_CONTROL);

	return bm92t_nd_send(info, &msg, BM92T_ND_HUB_SENT);
}

static int bm92t_nd_enter(struct bm92t_info *info)
{
	return bm92t_send_vdm(info, VDO(USB_SID_NINTENDO, 1, SVDM_VER_1_0,
					VDO_OPOS(1) | CMD_ENTER_MODE),
			      NULL, 0, BM92T_ND_ENTER_SENT);
}

static int bm92t_nd_start(struct bm92t_info *info)
{
	struct bm92t_nd_msg msg;

	bm92t_nd_cmd(&msg, ND_READ, ND_CMD_DEVICE_STATE);

	return bm92t_nd_send(info, &msg, BM92T_ND_QUERY_SENT);
}

static int bm92t_nd_send_led(struct bm92t_info *info)
{
	struct bm92t_nd_msg msg;

	bm92t_nd_led(&msg, 128, 0, 0, 64);

	return bm92t_nd_send(info, &msg, BM92T_ND_LED_SENT);
}

/* What a Status Update answers: our DP_STATUS seen from the partner */
static u32 bm92t_dp_status_vdo(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;
	u32 vdo;

	if (!att->dp_live)
		return 0;

	vdo = DP_STATUS_ENABLED | DP_STATUS_CON_UFP_D;
	if (DP_CONF_GET_PIN_ASSIGN(att->dp_conf) & BIT(DP_PIN_ASSIGN_D))
		vdo |= DP_STATUS_PREFER_MULTI_FUNC;
	if (att->dp_hpd)
		vdo |= DP_STATUS_HPD_STATE;

	return vdo;
}

static bool bm92t_dp_told(struct bm92t_info *info)
{
	bool told;

	mutex_lock(&info->lock);
	told = info->dp_told;
	mutex_unlock(&info->lock);

	return told;
}

/* Tell a bound DP alt mode driver what the controller did by itself */
static void bm92t_dp_sync(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;
	struct typec_altmode *alt = info->partner_dp;
	bool wanted, told, queued = false;
	unsigned int i;
	u32 vdo, hdr;

	if (!alt)
		return;

	vdo = bm92t_dp_status_vdo(info);

	mutex_lock(&info->lock);
	info->dp_status_vdo = vdo;
	info->dp_pin = DP_CONF_GET_PIN_ASSIGN(att->dp_conf);
	wanted = info->dp_enter_wanted;
	if (att->dp_live)
		info->dp_enter_wanted = false;
	told = info->dp_told;
	for (i = 0; i < info->dp_nr_replies; i++) {
		hdr = info->dp_replies[i].hdr;
		queued |= hdr == BM92T_DP_ACK(CMD_ENTER_MODE) ||
			  hdr == BM92T_DP_ACK(CMD_EXIT_MODE);
	}
	mutex_unlock(&info->lock);

	/* A driver binds by setting its ops; without one we own "active" */
	if (!READ_ONCE(alt->ops)) {
		bm92t_altmode_set_active(alt, att->dp_live);
		return;
	}

	/* What it was told last has not reached it yet */
	if (queued)
		return;

	if (att->dp_live) {
		if (!alt->active && (wanted || !told)) {
			/* As if the partner acked the Enter Mode */
			bm92t_dp_reply(info, BM92T_DP_ACK(CMD_ENTER_MODE), 0, 0);
		} else if (alt->active && !told) {
			/* Bound after we marked the mode active */
			if (typec_altmode_attention(info->port_dp, vdo)) {
				bm92t_retry_soon(info);
			} else {
				mutex_lock(&info->lock);
				info->dp_told = true;
				mutex_unlock(&info->lock);
			}
		}
	} else if (alt->active && told) {
		bm92t_dp_reply(info, BM92T_DP_ACK(CMD_EXIT_MODE), 0, 0);
	}
}

/* DP_STATUS changes the partner made or the controller saw */
static void bm92t_dp_update(struct bm92t_info *info, u16 dp_status)
{
	struct bm92t_attachment *att = &info->att;
	bool hpd = dp_status & DP_STATUS_SIGNAL_ON;

	if (!info->partner_dp)
		return;

	if (att->dp_live && !(dp_status & DP_STATUS_DP_EN)) {
		dev_info(info->dev, "DP alt mode exited (%04x)\n", dp_status);
		att->dp_live = false;
		att->dp_exited = true;
	} else if (att->dp_exited && (dp_status & DP_STATUS_DP_EN) &&
		   info->state == BM92T_IDLE) {
		dev_info(info->dev, "DP alt mode entered again (%04x)\n",
			 dp_status);
		att->dp_live = true;
		att->dp_exited = false;
		att->dp_hpd = hpd;
	}

	if (att->dp_live && hpd != att->dp_hpd) {
		att->dp_hpd = hpd;
		if (READ_ONCE(info->partner_dp->ops) && bm92t_dp_told(info))
			typec_altmode_attention(info->port_dp,
						bm92t_dp_status_vdo(info));
	}

	/* No DP alert is enabled: DP_STATUS is polled while DP is up */
	if (att->dp_live || att->dp_exited)
		bm92t_retry_at(info, BM92T_DP_POLL_MS);
}

static void bm92t_dp_active(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;

	att->dp_live = true;
	att->dp_hpd = info->dp_status & DP_STATUS_SIGNAL_ON;

	if (att->nintendo) {
		att->rearm = true;
		att->rearm_at = jiffies + msecs_to_jiffies(ND_REARM_DELAY_MS);
		att->nd_setup_due = true;
	}
	bm92t_set_state(info, BM92T_IDLE);
}

static void bm92t_dp_done(struct bm92t_info *info)
{
	if (info->att.nintendo)
		info->att.nd_setup_due = true;
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
	struct bm92t_attachment *att = &info->att;
	struct typec_altmode_desc desc = {
		.svid = USB_TYPEC_DP_SID,
		.mode = USB_TYPEC_DP_MODE,
		.vdo = vdo,
	};
	struct typec_altmode *alt;
	u32 signal;
	int pin;

	/* The dock claims a plug but fills in the receptacle field */
	if (info->att.nintendo && !(vdo & DP_CAP_RECEPTACLE)) {
		vdo = (vdo & ~GENMASK(23, 8)) | DP_CAP_UFP_D_PIN_ASSIGN(vdo) << 8;
		desc.vdo = vdo;
	}

	if (!(DP_CAP_CAPABILITY(vdo) & DP_CAP_UFP_D))
		return -ENODEV;

	pin = bm92t_dp_pin(info, vdo);
	if (pin < 0) {
		dev_info(info->dev, "no common DP pin assignment (%08x)\n", vdo);
		return pin;
	}

	signal = DP_CAP_DP_SIGNALLING(info->port_dp->vdo) &
		 DP_CAP_DP_SIGNALLING(vdo);

	/* Set up only with the mode registered: a cached VDO means both */
	if (!info->partner_dp) {
		alt = typec_partner_register_altmode(info->partner, &desc);
		if (IS_ERR(alt)) {
			dev_err(info->dev, "failed to register DP mode: %pe\n",
				alt);
			return PTR_ERR(alt);
		}

		info->partner_dp = alt;
		mutex_lock(&info->lock);
		info->dp_reply_ok = true;
		mutex_unlock(&info->lock);
	}

	att->dp_vdo = vdo;
	att->dp_conf = DP_CONF_UFP_U_AS_UFP_D |
		       FIELD_PREP(DP_CONF_SIGNALLING_MASK, signal) |
		       DP_CONF_SET_PIN_ASSIGN(BIT(pin));
	att->dp_cfg_retries = BM92T_DP_CFG_RETRIES;

	return 0;
}

static int bm92t_dp_configure(struct bm92t_info *info)
{
	u8 buf[6] = { OUTGOING_VDM_REG, 4 };

	put_unaligned_le32(info->att.dp_conf, &buf[2]);

	return bm92t_mailbox_free(info) ?:
	       bm92t_write(info, buf, sizeof(buf)) ?:
	       __bm92t_send_cmd(info, DP_CFG_AND_START_HPD_CMD,
				BM92T_DP_CFG_SENT);
}

/* DP alt mode is entered: adopt a live link, configure a dark one */
static int bm92t_dp_entered(struct bm92t_info *info, u16 dp_status)
{
	if (!(dp_status & DP_STATUS_SIGNAL_ON))
		return bm92t_dp_configure(info);

	dev_info(info->dev, "DP link already up, adopting it\n");
	bm92t_dp_active(info);

	return 0;
}

/* Role bits tell the bare mode VDO from a stale Enter Mode reply */
static int bm92t_dp_mailbox_vdo(struct bm92t_info *info, u32 *vdo)
{
	u8 buf[5];
	int ret;

	ret = bm92t_read(info, INCOMING_VDM_REG, buf, sizeof(buf));
	if (ret)
		return ret;

	*vdo = get_unaligned_le32(&buf[1]);
	if (buf[0] != 4 || !DP_CAP_CAPABILITY(*vdo))
		return -EPROTO;

	return 0;
}

static int bm92t_dp_query(struct bm92t_info *info)
{
	return bm92t_send_vdm(info, VDO(USB_TYPEC_DP_SID, 1, SVDM_VER_1_0,
					CMD_DISCOVER_MODES),
			      NULL, 0, BM92T_DP_QUERY_SENT);
}

static int bm92t_dp_enter(struct bm92t_info *info)
{
	info->att.dp_tried = true;

	return bm92t_send_cmd(info, DP_ENTER_MODE_CMD, BM92T_DP_ENTER_SENT);
}

/* A live link nobody identified: the controller's record may describe it */
static bool bm92t_dp_adopt_due(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;

	return info->port_dp && att->discovered && !att->dp_tried &&
	       (info->dp_status & DP_STATUS_DP_EN) &&
	       FIELD_GET(STATUS1_DR, info->status1) == DATA_ROLE_DFP;
}

static bool bm92t_is_dock(struct bm92t_info *info)
{
	struct usb_pd_identity *id = &info->partner_ident;

	return PD_IDH_PTYPE(id->id_header) == IDH_PTYPE_AMA &&
	       PD_IDH_VID(id->id_header) == USB_SID_NINTENDO &&
	       PD_PRODUCT_PID(id->product) == PID_NINTENDO_DOCK;
}

/* A submission inside a transaction failed: true to give up on it */
static bool bm92t_step_failed(struct bm92t_info *info, u16 alert, int ret,
			      const char *what)
{
	struct bm92t_attachment *att = &info->att;

	if (bm92t_submit_failed(info, ret, what)) {
		att->replay = 0;
		return true;
	}

	/* The next service runs this step again on the same events */
	att->replay = alert & (ALERT_CMD_DONE | ALERT_VDM_RECEIVED);
	att->replay_state = info->state;

	return false;
}

/* A VDM was sent: accept the reply when it comes. True when given up. */
static bool bm92t_vdm_sent(struct bm92t_info *info, u16 alert, u16 status1,
			   enum bm92t_state accept)
{
	int ret;

	if (alert & ALERT_VDM_RECEIVED) {
		ret = bm92t_send_cmd(info, ACCEPT_VDM_CMD, accept);
		if (ret && bm92t_step_failed(info, alert, ret, "VDM accept")) {
			info->att.vdm_pending = true;
			bm92t_set_state(info, BM92T_IDLE);
			return true;
		}
	} else if ((alert & ALERT_CMD_DONE) &&
		   !bm92t_lastcmd_ok(info, status1)) {
		bm92t_set_state(info, BM92T_IDLE);
		return true;
	}

	return false;
}

/* Advance the transaction in flight. Returns false while still waiting. */
static bool bm92t_step_done(struct bm92t_info *info, u16 alert, u16 status1)
{
	if (!(alert & ALERT_CMD_DONE))
		return false;

	if (!bm92t_lastcmd_ok(info, status1)) {
		if (info->state == BM92T_ND_REQ_ACCEPT)
			bm92t_req_finish(info, -EIO);
		bm92t_set_state(info, BM92T_IDLE);
		return false;
	}

	return true;
}

/* True when the transaction ends here: @ret failed and is not retried */
static bool bm92t_try(struct bm92t_info *info, u16 alert, int ret,
		      const char *what)
{
	return ret && bm92t_step_failed(info, alert, ret, what);
}

static void bm92t_step(struct bm92t_info *info, u16 alert, u16 status1,
		       u16 dp_status)
{
	struct bm92t_attachment *att = &info->att;
	struct typec_altmode_desc desc;
	struct typec_altmode *alt;
	u8 vdm[29];
	u32 vdo;
	int i, n, ret;

	switch (info->state) {
	case BM92T_IDLE:
	case BM92T_ABORT_WRITE:
	case BM92T_STUCK:
		break;

	case BM92T_ABORT_SENT:
		/* Only an idle mailbox with a good result ends the abort */
		if ((alert & ALERT_CMD_DONE) &&
		    !(status1 & STATUS1_CMD_BUSY) &&
		    FIELD_GET(STATUS1_LASTCMD, status1) == LASTCMD_COMPLETE) {
			info->abort_tries = 0;
			info->busy_since = 0;
			info->stuck = false;
			bm92t_set_state(info, BM92T_IDLE);
		} else if (alert & ALERT_CMD_DONE) {
			dev_dbg(info->dev, "abort: status1 %04x\n", status1);
		}
		break;

	case BM92T_DRAIN_ACCEPT:
		if (!(alert & ALERT_CMD_DONE))
			break;
		bm92t_read(info, INCOMING_VDM_REG, vdm, sizeof(vdm));
		bm92t_set_state(info, BM92T_IDLE);
		break;

	case BM92T_SRC_CAP_SENT:
		if (!(alert & ALERT_CMD_DONE))
			break;
		/* A source that sends no caps still gets discovered */
		bm92t_discover_later(info, BM92T_CMD_TIMEOUT_MS);
		bm92t_set_state(info, BM92T_IDLE);
		break;

	case BM92T_HARD_RST_SENT:
		if (alert & ALERT_CMD_DONE)
			bm92t_set_state(info, BM92T_IDLE);
		break;

	case BM92T_RDO_SENT:
		/* Newer caps: whatever this request gets is not the answer */
		if (bm92t_caps_changed(info)) {
			if ((alert & ALERT_CMD_DONE) || att->contract_seen) {
				att->ps_rdy_due = false;
				att->negotiated = false;
				bm92t_set_state(info, BM92T_IDLE);
			}
			break;
		}
		if (alert & ALERT_CONTRACT)
			att->ps_rdy_due = true;
		if (att->ps_rdy_due && (status1 & STATUS1_CMD_BUSY))
			break;
		if (att->ps_rdy_due) {
			ret = bm92t_send_cmd(info, PS_RDY_CMD, BM92T_PS_RDY_SENT);
			if (!ret) {
				att->ps_rdy_due = false;
			} else if (bm92t_submit_failed(info, ret, "PS_RDY")) {
				att->ps_rdy_due = false;
				bm92t_renegotiate(info, true);
			}
		} else if ((alert & ALERT_CMD_DONE) &&
			   !bm92t_lastcmd_ok(info, status1)) {
			/* Busy negotiating by itself: ask again after it */
			if (FIELD_GET(STATUS1_LASTCMD, status1) == LASTCMD_ABORTED)
				bm92t_renegotiate(info, false);
			else
				bm92t_negotiation_done(info);
		}
		break;

	case BM92T_PS_RDY_SENT:
		if (!(alert & ALERT_CMD_DONE))
			break;
		if (bm92t_lastcmd_ok(info, status1)) {
			bm92t_negotiation_done(info);
		} else {
			dev_warn(info->dev, "PS_RDY failed: %lu\n",
				 FIELD_GET(STATUS1_LASTCMD, status1));
			bm92t_renegotiate(info, true);
		}
		break;

	case BM92T_DR_SWAP_SENT:
		if (!bm92t_step_done(info, alert, status1))
			break;
		if (FIELD_GET(STATUS1_DR, status1) != DATA_ROLE_DFP ||
		    bm92t_try(info, alert, bm92t_send_disc_id(info),
			      "Discover Identity"))
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
			if (!bm92t_try(info, alert, n < 0 ? n : -EPROTO,
				       "identity read"))
				break;
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
		typec_partner_set_identity(info->partner);

		dev_info(info->dev, "partner %04x:%04x type %u\n",
			 PD_IDH_VID(info->partner_ident.id_header),
			 PD_PRODUCT_PID(info->partner_ident.product),
			 PD_IDH_PTYPE(info->partner_ident.id_header));

		if (bm92t_is_dock(info) && info->port_nd)
			ret = bm92t_nd_enter(info);
		else if (info->port_dp)
			ret = bm92t_send_vdm(info, VDO(USB_SID_PD, 1, SVDM_VER_1_0,
						       CMD_DISCOVER_SVID),
					     NULL, 0, BM92T_DISC_SVID_SENT);
		else
			ret = -ENODEV;
		if (bm92t_try(info, alert, ret, "discovery"))
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

		if (n < 0)
			ret = n;
		else if (!n || i == n)
			ret = -ENODEV;
		else
			ret = bm92t_send_vdm(info, VDO(USB_TYPEC_DP_SID, 1,
						       SVDM_VER_1_0,
						       CMD_DISCOVER_MODES),
					     NULL, 0, BM92T_DISC_MODE_SENT);
		if (bm92t_try(info, alert, ret, "Discover Modes"))
			bm92t_set_state(info, BM92T_IDLE);
		break;

	case BM92T_DISC_MODE_SENT:
		bm92t_vdm_sent(info, alert, status1, BM92T_DISC_MODE_ACCEPT);
		break;

	case BM92T_DISC_MODE_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		n = bm92t_read_vdm(info, vdm, sizeof(vdm), USB_TYPEC_DP_SID,
				   CMD_DISCOVER_MODES);
		if (n < 0)
			ret = n;
		else if (!n)
			ret = -ENODEV;
		else
			ret = bm92t_dp_register(info,
						get_unaligned_le32(&vdm[5])) ?:
			      bm92t_dp_enter(info);
		if (bm92t_try(info, alert, ret, "DP entry"))
			bm92t_set_state(info, BM92T_IDLE);
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

		if (att->dp_vdo) {
			ret = bm92t_dp_entered(info, dp_status);
		} else {
			ret = bm92t_dp_mailbox_vdo(info, &vdo);
			if (!ret)
				ret = bm92t_dp_register(info, vdo) ?:
				      bm92t_dp_entered(info, dp_status);
			else if (ret == -EPROTO)
				/* Ask the partner when the mailbox holds no VDO */
				ret = bm92t_dp_query(info);
		}
		if (bm92t_try(info, alert, ret, "DP setup"))
			bm92t_dp_done(info);
		break;

	case BM92T_DP_QUERY_SENT:
		if (bm92t_vdm_sent(info, alert, status1, BM92T_DP_QUERY_ACCEPT))
			bm92t_dp_done(info);
		break;

	case BM92T_DP_QUERY_ACCEPT:
		if (!bm92t_step_done(info, alert, status1)) {
			if (info->state == BM92T_IDLE)
				bm92t_dp_done(info);
			break;
		}

		n = bm92t_read_vdm(info, vdm, sizeof(vdm), USB_TYPEC_DP_SID,
				   CMD_DISCOVER_MODES);
		if (n < 0)
			ret = n;
		else if (!n)
			ret = -ENODEV;
		else
			ret = bm92t_dp_register(info,
						get_unaligned_le32(&vdm[5]));
		/* The link may have come up while we asked */
		if (!ret)
			ret = bm92t_read16(info, DP_STATUS_REG, &dp_status);
		if (!ret) {
			info->dp_status = dp_status;
			ret = bm92t_dp_entered(info, dp_status);
		}
		if (bm92t_try(info, alert, ret, "DP setup")) {
			dev_info(info->dev, "no DP mode VDO: %d\n", ret);
			bm92t_dp_done(info);
		}
		break;

	case BM92T_DP_CFG_SENT:
		if (!(alert & ALERT_CMD_DONE))
			break;

		if (bm92t_lastcmd_ok(info, status1)) {
			bm92t_dp_active(info);
		} else if (!att->dp_cfg_retries) {
			bm92t_dp_done(info);
		} else {
			ret = bm92t_dp_configure(info);
			if (!ret)
				att->dp_cfg_retries--;
			else if (bm92t_try(info, alert, ret, "DP configure"))
				bm92t_dp_done(info);
		}
		break;

	case BM92T_ND_ENTER_SENT:
		bm92t_vdm_sent(info, alert, status1, BM92T_ND_ENTER_ACCEPT);
		break;

	case BM92T_ND_ENTER_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		n = bm92t_read_vdm(info, vdm, sizeof(vdm), USB_SID_NINTENDO,
				   CMD_ENTER_MODE);
		if (n < 0) {
			if (!bm92t_try(info, alert, n, "dock mode"))
				break;
			dev_err(info->dev, "dock refused its alt mode: %d\n", n);
			bm92t_set_state(info, BM92T_IDLE);
			break;
		}

		if (!info->partner_nd) {
			desc = (struct typec_altmode_desc) {
				.svid = USB_SID_NINTENDO,
				.mode = 1,
			};
			alt = typec_partner_register_altmode(info->partner,
							     &desc);
			if (IS_ERR(alt)) {
				if (!bm92t_try(info, alert, PTR_ERR(alt),
					       "dock mode registration"))
					break;
				dev_err(info->dev,
					"failed to register dock mode: %pe\n",
					alt);
				bm92t_set_state(info, BM92T_IDLE);
				break;
			}
			info->partner_nd = alt;
			bm92t_altmode_set_active(alt, true);
		}

		att->nintendo = true;
		mutex_lock(&info->lock);
		info->nd_ready = true;
		mutex_unlock(&info->lock);
		if (!info->port_dp)
			bm92t_dp_done(info);
		else if (bm92t_try(info, alert, bm92t_dp_enter(info), "DP entry"))
			bm92t_dp_done(info);
		break;

	case BM92T_ND_QUERY_SENT:
		bm92t_vdm_sent(info, alert, status1, BM92T_ND_QUERY_ACCEPT);
		break;

	case BM92T_ND_QUERY_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		ret = bm92t_nd_reply(info, vdm, sizeof(vdm));
		if (ret < 0 || vdm[0] < 12)
			dev_warn(info->dev, "no dock state: %d\n", ret);
		else
			dev_info(info->dev, "dock state %*ph\n", 4, &vdm[9]);

		if (bm92t_try(info, alert, bm92t_nd_send_led(info), "dock LED")) {
			att->hub_retries = ND_HUB_RETRIES;
			bm92t_set_state_ms(info, BM92T_ND_HUB_WAIT,
					   ND_HUB_DELAY_MS);
		}
		break;

	case BM92T_ND_LED_SENT:
		if (bm92t_vdm_sent(info, alert, status1, BM92T_ND_LED_ACCEPT)) {
			att->hub_retries = ND_HUB_RETRIES;
			bm92t_set_state_ms(info, BM92T_ND_HUB_WAIT,
					   ND_HUB_DELAY_MS);
		}
		break;

	case BM92T_ND_LED_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		ret = bm92t_nd_result(info, vdm, sizeof(vdm));
		if (ret)
			dev_warn(info->dev, "dock LED setup failed: %d\n", ret);

		/* The hub powers up behind the LED command */
		att->hub_retries = ND_HUB_RETRIES;
		bm92t_set_state_ms(info, BM92T_ND_HUB_WAIT, ND_HUB_DELAY_MS);
		break;

	case BM92T_ND_HUB_WAIT:
		break;

	case BM92T_ND_HUB_SENT:
		bm92t_vdm_sent(info, alert, status1, BM92T_ND_HUB_ACCEPT);
		break;

	case BM92T_ND_HUB_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		ret = bm92t_nd_result(info, vdm, sizeof(vdm));
		if (ret == -EBUSY && att->hub_retries--) {
			bm92t_set_state_ms(info, BM92T_ND_HUB_WAIT,
					   ND_HUB_RETRY_MS);
			break;
		}

		if (ret)
			dev_err(info->dev, "dock hub enable failed: %d\n", ret);
		bm92t_set_state(info, BM92T_IDLE);
		break;

	case BM92T_ND_REQ_SENT:
		if (bm92t_vdm_sent(info, alert, status1, BM92T_ND_REQ_ACCEPT))
			bm92t_req_finish(info, -EIO);
		break;

	case BM92T_ND_REQ_ACCEPT:
		if (!bm92t_step_done(info, alert, status1))
			break;

		ret = bm92t_nd_result(info, vdm, sizeof(vdm));
		if (ret)
			dev_dbg(info->dev, "dock request %02x: %d\n",
				att->nd_cmd, ret);
		bm92t_req_finish(info, ret);
		bm92t_set_state(info, BM92T_IDLE);
		break;

	case BM92T_NUM_STATES:
		break;
	}
}

static void bm92t_timeout(struct bm92t_info *info)
{
	enum bm92t_state state = info->state;
	u16 status1;
	int ret;

	switch (state) {
	case BM92T_ND_HUB_WAIT:
		/* Not a command: the dock's hub is due */
		ret = bm92t_nd_hub_enable(info);
		if (!ret)
			return;
		if (!bm92t_submit_failed(info, ret, "dock hub enable")) {
			bm92t_set_state_ms(info, BM92T_ND_HUB_WAIT,
					   BM92T_RETRY_MS);
			return;
		}
		bm92t_set_state(info, BM92T_IDLE);
		return;

	case BM92T_ABORT_WRITE:
	case BM92T_ABORT_SENT:
		if (++info->abort_tries < BM92T_ABORT_TRIES) {
			bm92t_abort(info, -ETIMEDOUT);
			return;
		}
		if (!info->stuck)
			dev_err(info->dev, "abort never completed, mailbox stuck\n");
		info->stuck = true;
		info->abort_tries = 0;
		bm92t_set_state_ms(info, BM92T_STUCK, BM92T_STUCK_POLL_MS);
		return;

	case BM92T_STUCK:
		/* Start over with an abort once the controller is not busy */
		if (!bm92t_read16(info, STATUS1_REG, &status1) &&
		    !(status1 & STATUS1_CMD_BUSY))
			bm92t_abort(info, -ETIMEDOUT);
		else
			bm92t_set_state_ms(info, BM92T_STUCK,
					   BM92T_STUCK_POLL_MS);
		return;

	default:
		break;
	}

	dev_warn(info->dev, "%s timed out\n", bm92t_state_names[state]);
	bm92t_abort(info, -ETIMEDOUT);

	/* The contract the controller holds stands: carry on from it */
	if (state == BM92T_RDO_SENT || state == BM92T_PS_RDY_SENT) {
		info->att.negotiated = true;
		bm92t_discover_later(info, 0);
	} else if (state == BM92T_SRC_CAP_SENT) {
		bm92t_discover_later(info, 0);
	}
}

/* Protection first: without OCP the port must not source */
static int bm92t_hw_init(struct bm92t_info *info)
{
	return bm92t_update16(info, VENDOR_CONFIG_REG,
			      VENDOR_CONFIG_OCP_DISABLE, 0) ?:
	       bm92t_update16(info, CONFIG1_REG, CONFIG1_SPDSRC,
			      FIELD_PREP(CONFIG1_SPDSRC, SPDSRC12_ON)) ?:
	       bm92t_write16(info, DP_ALERT_EN_REG, 0);
}

static void bm92t_handle_fault(struct bm92t_info *info, u16 status1)
{
	info->alert = 0;
	/* A faulted controller may never raise another alert */
	bm92t_retry_at(info, BM92T_FAULT_POLL_MS);

	if (info->att.attached)
		bm92t_detach(info);

	/* Our own VBUS would keep VSAFE up */
	if (bm92t_release_port(info) && info->vbus_on)
		return;

	if (status1 & (STATUS1_INSERT | STATUS1_VSAFE)) {
		dev_warn_once(info->dev,
			      "fault on an occupied port, waiting for unplug\n");
		return;
	}

	/* A wedged partner reads empty too: reset until it is gone */
	if (!info->faulted)
		dev_err(info->dev, "internal fault %lu, resetting\n",
			FIELD_GET(STATUS1_FAULT, status1));
	info->faulted = true;
	bm92t_reset(info);
}

/* Record what the alert means for work that waits for the mailbox */
static void bm92t_note_events(struct bm92t_info *info, u16 alert)
{
	struct bm92t_attachment *att = &info->att;
	bool ours;

	if ((alert & ALERT_VDM_RECEIVED) &&
	    !(bm92t_vdm_states & BIT(info->state)))
		att->vdm_pending = true;

	if (att->source) {
		if (alert & ALERT_SRC_PLUGIN)
			bm92t_discover_later(info, 0);
		return;
	}

	if (alert & ALERT_PDO) {
		att->caps_gen++;
		att->rdo_retries = BM92T_RDO_RETRIES;
	}

	/* A contract after newer caps than our request's is the controller's */
	ours = (info->state == BM92T_RDO_SENT ||
		info->state == BM92T_PS_RDY_SENT) && !bm92t_caps_changed(info);

	/* Request after the controller's own contract, or it aborts */
	if ((alert & ALERT_CONTRACT) && !ours) {
		att->contract_seen = true;
		att->negotiate_at = jiffies;
		/* A plug-in wake-up brings the contract before the PDOs */
		if ((alert & ALERT_PLUGPULL) && !bm92t_caps_changed(info))
			att->negotiate_at += msecs_to_jiffies(BM92T_PLUG_SETTLE_MS);
	}
}

/* A completion read before the last command was written is not its own */
static u16 bm92t_fresh(struct bm92t_info *info, u16 alert)
{
	if (info->done_seq != info->cmd_seq)
		alert &= ~ALERT_CMD_DONE;

	return alert;
}

static int bm92t_service(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;
	u16 alert, status1, status2, dp_status;
	u32 pdo = 0, rdo = 0;
	bool source, replaced;
	int ret;

	ret = bm92t_read16(info, ALERT_STATUS_REG, &alert);
	if (ret)
		return ret;

	/* The read cleared it: keep it until the event is handled */
	if (alert & ALERT_CMD_DONE)
		info->done_seq = info->cmd_seq;
	info->alert |= alert;

	ret = bm92t_read16(info, STATUS1_REG, &status1) ?:
	      bm92t_read16(info, STATUS2_REG, &status2) ?:
	      bm92t_read16(info, DP_STATUS_REG, &dp_status);
	/* Every read that can fail comes before anything acts on the alert */
	if (!ret && (status1 & STATUS1_INSERT) &&
	    !FIELD_GET(STATUS1_FAULT, status1))
		ret = bm92t_read_contract(info, &pdo, &rdo);
	if (ret)
		return ret;

	alert = info->alert;
	info->status1 = status1;
	info->status2 = status2;
	info->dp_status = dp_status;
	WRITE_ONCE(info->last_alert, alert);

	dev_dbg(info->dev, "alert %04x status1 %04x status2 %04x dp %04x %s\n",
		alert, status1, status2, dp_status,
		bm92t_state_names[info->state]);

	if (alert & ALERT_SNK_FAULT)
		dev_err(info->dev, "sink fault\n");
	if (alert & ALERT_SRC_FAULT)
		dev_err(info->dev, "source fault\n");

	/* A reset returned the controller to defaults: set it up again */
	if (info->hw_init_pending) {
		if (time_before(jiffies, info->hw_init_at)) {
			bm92t_retry_at(info, BM92T_RESET_SETTLE_MS);
		} else if (bm92t_hw_init(info)) {
			bm92t_retry_soon(info);
		} else {
			dev_dbg(info->dev, "set up again after the reset\n");
			info->hw_init_pending = false;
		}
	}

	if (FIELD_GET(STATUS1_FAULT, status1)) {
		bm92t_handle_fault(info, status1);
		return 0;
	}
	info->faulted = false;

	if (!(status1 & STATUS1_INSERT)) {
		info->alert = 0;
		if (att->attached)
			bm92t_detach(info);
		if (info->state == BM92T_STUCK)
			bm92t_reset(info);	/* only ever on an empty port */
		else if (info->state == BM92T_ABORT_SENT)
			bm92t_step(info, bm92t_fresh(info, alert), status1,
				   dp_status);
		bm92t_release_port(info);
		return 0;
	}

	/* A plug event on an occupied port: whatever is there is new */
	replaced = att->attached && (alert & ALERT_PLUGPULL);
	if (replaced) {
		dev_dbg(info->dev, "partner replaced\n");
		bm92t_detach(info);
	}

	if (alert & ALERT_PLUGPULL)
		info->vbus_blocked = false;

	if (!att->attached)
		bm92t_attach(info, status1, status2, alert, replaced);
	else
		bm92t_register_partner(info, status2);

	/* A power role swap ends what the old role was doing */
	source = status1 & STATUS1_SRC_MODE;
	if (source != att->source) {
		dev_dbg(info->dev, "now %s\n", source ? "source" : "sink");
		att->source = source;
		att->asked_gen = att->caps_gen;
		att->contract_seen = false;
		att->negotiated = false;
		att->rdo_retries = BM92T_RDO_RETRIES;
		if (source)
			bm92t_discover_later(info, 0);
		bm92t_set_limit(info, 0, 0, 0);
	}

	/* Hard-reset only while sourcing: a sink negotiation flags it too */
	if ((alert & ALERT_SRC_FAULT) && source) {
		info->vbus_blocked = true;
		att->negotiated = false;
		if (bm92t_in_req(info))
			bm92t_req_finish(info, -EIO);
		if (__bm92t_send_cmd(info, PD_HARD_RST_CMD,
				     BM92T_HARD_RST_SENT))
			bm92t_set_state(info, BM92T_IDLE);
	}

	bm92t_update_roles(info, status1, status2);

	bm92t_update_contract(info, pdo, rdo);
	bm92t_raise_limit(info);

	info->alert = 0;

	bm92t_note_events(info, alert);
	alert = bm92t_fresh(info, alert);
	if (att->replay && att->replay_state == info->state)
		alert |= att->replay;
	att->replay = 0;
	bm92t_step(info, alert, status1, dp_status);
	bm92t_dp_update(info, dp_status);

	return 0;
}

static bool bm92t_nd_next(struct bm92t_info *info, struct bm92t_nd_msg *msg,
			  bool *is_req)
{
	struct bm92t_attachment *att = &info->att;
	u16 dp_status;

	*is_req = false;

	mutex_lock(&info->lock);
	if (info->req_state == BM92T_REQ_PENDING) {
		*msg = info->req_msg;
		info->req_state = BM92T_REQ_ACTIVE;
		info->active_req = info->req_id;
		*is_req = true;
	}
	mutex_unlock(&info->lock);

	if (*is_req || !att->attached || !att->nintendo)
		return *is_req;

	if (!att->rearm || time_before(jiffies, att->rearm_at))
		return false;

	if (bm92t_read16(info, DP_STATUS_REG, &dp_status)) {
		bm92t_retry_soon(info);
		return false;
	}

	att->rearm = false;
	if (!(dp_status & DP_STATUS_DP_EN) || (dp_status & DP_STATUS_SIGNAL_ON))
		return false;

	dev_info(info->dev, "DP up but dark (%04x), waking it\n", dp_status);
	bm92t_nd_dp_sleep(msg, false);

	return true;
}

/* Start the work that waited for the mailbox, one command at a time */
static void bm92t_idle(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;
	struct bm92t_nd_msg msg;
	bool is_req;
	int ret;

	/* Busy with no command of ours in flight: take the mailbox back */
	if (info->busy_since &&
	    time_after(jiffies, info->busy_since +
				msecs_to_jiffies(BM92T_CMD_TIMEOUT_MS))) {
		dev_warn(info->dev, "controller busy with no command of ours\n");
		info->busy_since = 0;
		bm92t_abort(info, -EBUSY);
		return;
	}

	if (!att->attached)
		return;

	/* Until accepted, a partner VDM holds back the reply to ours */
	if (att->vdm_pending || (info->status2 & STATUS2_VDM_IN)) {
		ret = bm92t_send_cmd(info, ACCEPT_VDM_CMD, BM92T_DRAIN_ACCEPT);
		if (!ret || bm92t_submit_failed(info, ret, "VDM drain"))
			att->vdm_pending = false;
		return;
	}

	if (att->want_src_cap) {
		ret = bm92t_send_cmd(info, GET_SRC_CAP_CMD, BM92T_SRC_CAP_SENT);
		if (!ret) {
			att->want_src_cap = false;
		} else if (bm92t_submit_failed(info, ret, "GET_SRC_CAP")) {
			att->want_src_cap = false;
			bm92t_discover_later(info, 0);
		}
		return;
	}

	if (bm92t_negotiation_due(info)) {
		bm92t_negotiate(info);
		return;
	}

	if (bm92t_discovery_wanted(info) &&
	    !time_before(jiffies, att->discover_at)) {
		bm92t_discover(info);
		if (info->state != BM92T_IDLE || !att->discovered)
			return;
	}

	if (bm92t_dp_adopt_due(info)) {
		dev_info(info->dev, "DP up without discovery, adopting it\n");
		ret = bm92t_dp_enter(info);
		if (ret && !bm92t_submit_failed(info, ret, "DP adoption"))
			att->dp_tried = false;
		return;
	}

	if (att->nd_setup_due) {
		ret = bm92t_nd_start(info);
		if (!ret || bm92t_submit_failed(info, ret, "dock setup"))
			att->nd_setup_due = false;
		return;
	}

	if (!bm92t_nd_next(info, &msg, &is_req))
		return;

	ret = bm92t_nd_send(info, &msg, BM92T_ND_REQ_SENT);
	if (!ret)
		return;

	if (bm92t_submit_failed(info, ret, is_req ? "dock request" :
						     "DP wake")) {
		if (is_req)
			bm92t_req_finish(info, ret);
		return;
	}

	/* Sent again from the next idle */
	if (!is_req) {
		att->rearm = true;
		att->rearm_at = jiffies;
		return;
	}
	mutex_lock(&info->lock);
	if (info->req_state == BM92T_REQ_ACTIVE &&
	    info->req_id == info->active_req)
		info->req_state = BM92T_REQ_PENDING;
	mutex_unlock(&info->lock);
}

static void bm92t_dump(struct bm92t_info *info)
{
	unsigned int i;
	int ret = 0;

	mutex_lock(&info->lock);
	if (!info->dump_wanted) {
		mutex_unlock(&info->lock);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(bm92t_dump_regs) && !ret; i++)
		ret = bm92t_read(info, bm92t_dump_regs[i].reg, info->dump[i],
				 bm92t_dump_regs[i].len);

	info->dump_status = ret;
	info->dump_wanted = false;
	complete(&info->dump_done);
	mutex_unlock(&info->lock);
}

static void bm92t_arm_timer(struct bm92t_info *info)
{
	struct bm92t_attachment *att = &info->att;
	unsigned long next = 0;

	/* Nothing falls due while uncertain: only the service retry runs */
	if (info->state != BM92T_IDLE && !info->uncertain)
		next = info->deadline;
	if (info->poll && (!next || time_before(info->poll_at, next)))
		next = info->poll_at;
	if (info->raise_at && !info->uncertain &&
	    (!next || time_before(info->raise_at, next)))
		next = info->raise_at;
	/* An overdue wake waits for the retry of what held it back */
	if (att->rearm && info->state == BM92T_IDLE && !info->uncertain &&
	    (time_after(att->rearm_at, jiffies) || !info->poll) &&
	    (!next || time_before(att->rearm_at, next)))
		next = att->rearm_at;
	if (info->state == BM92T_IDLE && bm92t_negotiation_wanted(info) &&
	    time_after(att->negotiate_at, jiffies) &&
	    (!next || time_before(att->negotiate_at, next)))
		next = att->negotiate_at;
	if (info->state == BM92T_IDLE && bm92t_discovery_wanted(info) &&
	    time_after(att->discover_at, jiffies) &&
	    (!next || time_before(att->discover_at, next)))
		next = att->discover_at;

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
	bool due;

	mutex_lock(&info->lock);
	if (info->stopped) {
		mutex_unlock(&info->lock);
		return;
	}
	mutex_unlock(&info->lock);

	due = READ_ONCE(info->irq_pending) || info->uncertain;
	if (info->poll && !time_before(jiffies, info->poll_at)) {
		/* Due: the service below asks again if it needs to */
		info->poll = false;
		due = true;
	}
	/* A raise is published by a service that read its contract */
	if (info->raise_at && !time_before(jiffies, info->raise_at))
		due = true;
	if (due) {
		info->uncertain = bm92t_service(info);
		if (info->uncertain) {
			bm92t_retry_soon(info);
		} else if (READ_ONCE(info->irq_pending)) {
			WRITE_ONCE(info->irq_pending, false);
			enable_irq(info->client->irq);
		}
	}

	/* Nothing new on what we could not read back */
	if (!info->uncertain) {
		if (info->state != BM92T_IDLE &&
		    !time_before(jiffies, info->deadline))
			bm92t_timeout(info);

		if (info->state == BM92T_IDLE)
			bm92t_idle(info);

		bm92t_dp_sync(info);
	}

	bm92t_dump(info);
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

/* Replies to the DP alt mode driver, which calls back into us, in order */
static void bm92t_dp_reply_work(struct work_struct *work)
{
	struct bm92t_info *info = container_of(to_delayed_work(work),
					       struct bm92t_info, dp_work);
	u32 hdr, vdo;
	int count, ret;

	mutex_lock(&info->lock);
	if (!info->dp_reply_ok || !info->dp_nr_replies) {
		mutex_unlock(&info->lock);
		return;
	}
	hdr = info->dp_replies[0].hdr;
	vdo = info->dp_replies[0].vdo;
	count = info->dp_replies[0].count;
	mutex_unlock(&info->lock);

	ret = typec_altmode_vdm(info->port_dp, hdr, &vdo, count);

	mutex_lock(&info->lock);
	if (!info->dp_reply_ok) {
		mutex_unlock(&info->lock);
		return;
	}

	/* Busy with its own work: the same reply again shortly */
	if (ret == -EBUSY) {
		schedule_delayed_work(&info->dp_work,
				      msecs_to_jiffies(BM92T_DP_REPLY_RETRY_MS));
		mutex_unlock(&info->lock);
		return;
	}

	if (ret)
		dev_dbg(info->dev, "DP reply %08x not taken: %d\n", hdr, ret);
	else if (hdr == BM92T_DP_ACK(CMD_ENTER_MODE))
		info->dp_told = true;
	else if (hdr == BM92T_DP_ACK(CMD_EXIT_MODE))
		info->dp_told = false;

	info->dp_nr_replies--;
	memmove(&info->dp_replies[0], &info->dp_replies[1],
		info->dp_nr_replies * sizeof(info->dp_replies[0]));
	if (info->dp_nr_replies)
		schedule_delayed_work(&info->dp_work, 0);
	mutex_unlock(&info->lock);

	/* What the driver now knows may call for the next notification */
	bm92t_kick(info);
}

static void bm92t_dp_reply(struct bm92t_info *info, u32 hdr, u32 vdo,
			   int count)
{
	unsigned int n;

	mutex_lock(&info->lock);
	n = info->dp_nr_replies;
	if (info->dp_reply_ok && n < BM92T_DP_REPLY_SLOTS) {
		info->dp_replies[n].hdr = hdr;
		info->dp_replies[n].vdo = vdo;
		info->dp_replies[n].count = count;
		info->dp_nr_replies++;
		if (!n)
			schedule_delayed_work(&info->dp_work, 0);
	} else if (info->dp_reply_ok) {
		dev_warn(info->dev, "DP reply %08x dropped\n", hdr);
	}
	mutex_unlock(&info->lock);
}

static int bm92t_dp_op_enter(struct typec_altmode *alt, u32 *vdo)
{
	struct bm92t_info *info = typec_altmode_get_drvdata(alt);

	mutex_lock(&info->lock);
	info->dp_enter_wanted = true;
	mutex_unlock(&info->lock);
	bm92t_kick(info);

	return 0;
}

static int bm92t_dp_op_exit(struct typec_altmode *alt)
{
	return -EOPNOTSUPP;
}

static int bm92t_dp_op_vdm(struct typec_altmode *alt, u32 header,
			   const u32 *data, int count)
{
	struct bm92t_info *info = typec_altmode_get_drvdata(alt);
	u32 hdr = VDO(USB_TYPEC_DP_SID, 1, SVDM_VER_1_0,
		      VDO_OPOS(USB_TYPEC_DP_MODE) | PD_VDO_CMD(header));
	u32 vdo = 0;
	int n = 1;

	if (PD_VDO_CMDT(header) != CMDT_INIT)
		return 0;

	mutex_lock(&info->lock);
	switch (PD_VDO_CMD(header)) {
	case DP_CMD_STATUS_UPDATE:
		vdo = info->dp_status_vdo;
		hdr |= VDO_CMDT(vdo ? CMDT_RSP_ACK : CMDT_RSP_NAK);
		n = 2;
		break;
	case DP_CMD_CONFIGURE:
		/* The controller runs the pin assignment it picked */
		if (count > 1 && (!DP_CONF_GET_PIN_ASSIGN(data[0]) ||
				  (info->dp_status_vdo &&
				   DP_CONF_GET_PIN_ASSIGN(data[0]) & info->dp_pin)))
			hdr |= VDO_CMDT(CMDT_RSP_ACK);
		else
			hdr |= VDO_CMDT(CMDT_RSP_NAK);
		break;
	default:
		hdr |= VDO_CMDT(CMDT_RSP_NAK);
		break;
	}
	mutex_unlock(&info->lock);

	bm92t_dp_reply(info, hdr, vdo, n);

	return 0;
}

static const struct typec_altmode_ops bm92t_dp_ops = {
	.enter = bm92t_dp_op_enter,
	.exit = bm92t_dp_op_exit,
	.vdm = bm92t_dp_op_vdm,
};

/* Send a dock request through the work item and wait for its reply */
static int bm92t_nd_request(struct bm92t_info *info,
			    const struct bm92t_nd_msg *msg)
{
	int ret;

	mutex_lock(&info->req_mutex);

	mutex_lock(&info->lock);
	if (info->stopped) {
		mutex_unlock(&info->lock);
		mutex_unlock(&info->req_mutex);
		return -ESHUTDOWN;
	}
	if (!info->nd_ready) {
		mutex_unlock(&info->lock);
		mutex_unlock(&info->req_mutex);
		return -ENODEV;
	}
	info->req_msg = *msg;
	info->req_id++;
	info->req_state = BM92T_REQ_PENDING;
	reinit_completion(&info->req_done);
	mutex_unlock(&info->lock);

	bm92t_kick(info);
	wait_for_completion_timeout(&info->req_done,
				    msecs_to_jiffies(BM92T_REQ_TIMEOUT_MS));

	mutex_lock(&info->lock);
	ret = info->req_state == BM92T_REQ_DONE ? info->req_status : -ETIMEDOUT;
	info->req_state = BM92T_REQ_NONE;
	mutex_unlock(&info->lock);

	mutex_unlock(&info->req_mutex);

	return ret;
}

static void bm92t_nd_set_led(struct bm92t_info *info, u8 duty, u8 time_on,
			     u8 time_off, u8 fade)
{
	struct bm92t_nd_msg msg;
	int ret;

	bm92t_nd_led(&msg, duty, time_on, time_off, fade);
	ret = bm92t_nd_request(info, &msg);
	if (ret && ret != -ENODEV)
		dev_warn(info->dev, "dock LED request failed: %d\n", ret);
}

/* Stop the work item and keep it stopped: nothing touches the chip after */
static void bm92t_stop(struct bm92t_info *info)
{
	disable_irq(info->client->irq);

	mutex_lock(&info->lock);
	info->stopped = true;
	if (info->req_state == BM92T_REQ_PENDING ||
	    info->req_state == BM92T_REQ_ACTIVE)
		__bm92t_req_done(info, -ESHUTDOWN);
	if (info->dump_wanted) {
		info->dump_wanted = false;
		info->dump_status = -ESHUTDOWN;
		complete(&info->dump_done);
	}
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
	struct bm92t_info *info = s->private;
	unsigned int i;
	int ret;

	mutex_lock(&info->req_mutex);

	mutex_lock(&info->lock);
	if (info->stopped) {
		mutex_unlock(&info->lock);
		mutex_unlock(&info->req_mutex);
		return -ESHUTDOWN;
	}
	info->dump_wanted = true;
	reinit_completion(&info->dump_done);
	mutex_unlock(&info->lock);

	bm92t_kick(info);
	wait_for_completion_timeout(&info->dump_done,
				    msecs_to_jiffies(BM92T_REQ_TIMEOUT_MS));

	mutex_lock(&info->lock);
	ret = info->dump_wanted ? -ETIMEDOUT : info->dump_status;
	info->dump_wanted = false;

	/* ALERT_STATUS clears on read: show what the work item last saw */
	if (!ret)
		seq_printf(s, "%-15s 0x%04x\n", "ALERT_STATUS",
			   READ_ONCE(info->last_alert));

	for (i = 0; i < ARRAY_SIZE(bm92t_dump_regs) && !ret; i++) {
		const u8 *v = info->dump[i];

		if (bm92t_dump_regs[i].len == 2)
			seq_printf(s, "%-15s 0x%04x\n", bm92t_dump_regs[i].name,
				   get_unaligned_le16(v));
		else
			seq_printf(s, "%-15s 0x%08x (%u)\n",
				   bm92t_dump_regs[i].name,
				   get_unaligned_le32(&v[1]), v[0]);
	}
	mutex_unlock(&info->lock);

	mutex_unlock(&info->req_mutex);

	return ret;
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

/* The controller only does what its firmware was provisioned for */
static int bm92t_check_caps(struct bm92t_info *info,
			    const struct typec_capability *cap)
{
	if (cap->type != TYPEC_PORT_DRP || cap->data != TYPEC_PORT_DRD ||
	    cap->prefer_role != TYPEC_NO_PREFERRED_ROLE) {
		dev_err(info->dev,
			"connector must be dual-role power and data without a preferred role\n");
		return -EINVAL;
	}

	if (info->port_dp && !(DP_CAP_CAPABILITY(info->port_dp->vdo) &
			       DP_CAP_DFP_D)) {
		dev_err(info->dev, "DisplayPort mode %08x is not DFP_D\n",
			info->port_dp->vdo);
		return -EINVAL;
	}

	return 0;
}

static int bm92t_register_port(struct bm92t_info *info)
{
	struct typec_altmode_desc nd = { .svid = USB_SID_NINTENDO, .mode = 1 };
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

	typec_port_register_altmodes(info->port, &bm92t_dp_ops, info,
				     info->port_altmode,
				     ARRAY_SIZE(info->port_altmode));
	for (i = 0; i < ARRAY_SIZE(info->port_altmode); i++)
		if (info->port_altmode[i] &&
		    info->port_altmode[i]->svid == USB_TYPEC_DP_SID)
			info->port_dp = info->port_altmode[i];

	info->port_nd = typec_port_register_altmode(info->port, &nd);
	if (IS_ERR(info->port_nd))
		info->port_nd = NULL;

	return bm92t_check_caps(info, &cap);
}

static void bm92t_unregister_port(struct bm92t_info *info)
{
	unsigned int i;

	bm92t_unregister_altmode(&info->port_nd);
	for (i = 0; i < ARRAY_SIZE(info->port_altmode); i++)
		bm92t_unregister_altmode(&info->port_altmode[i]);
	typec_unregister_port(info->port);
}

/* A board supply that takes @ma anywhere from @min_mv to @max_mv */
static bool bm92t_board_takes(struct bm92t_info *info, unsigned int min_mv,
			      unsigned int max_mv, unsigned int ma)
{
	unsigned int i;

	if (min_mv == max_mv)
		return bm92t_sink_ma(info, min_mv) >= ma;

	for (i = 0; i < info->nr_sink_pdos; i++) {
		u32 pdo = info->sink_pdos[i];

		if (pdo_type(pdo) == PDO_TYPE_VAR &&
		    pdo_min_voltage(pdo) <= min_mv &&
		    pdo_max_voltage(pdo) >= max_mv &&
		    pdo_max_current(pdo) >= ma)
			return true;
	}

	return false;
}

/* The board must take whatever the firmware negotiates by itself */
static int bm92t_check_fw_sink(struct bm92t_info *info)
{
	u8 buf[1 + 16];
	unsigned int i, n;
	bool ok;
	int ret;

	ret = bm92t_read(info, PDOS_SNK_CONS_REG, buf, sizeof(buf));
	if (ret)
		return ret;

	if (!buf[0] || buf[0] % 4 || buf[0] > sizeof(buf) - 1) {
		dev_err(info->dev, "firmware sink PDOs: length %u\n", buf[0]);
		return -EPROTO;
	}

	n = buf[0] / 4;
	for (i = 0; i < n; i++) {
		u32 pdo = get_unaligned_le32(&buf[1 + 4 * i]);

		dev_dbg(info->dev, "firmware sink PDO %08x\n", pdo);

		switch (pdo_type(pdo)) {
		case PDO_TYPE_FIXED:
			ok = bm92t_board_takes(info, pdo_fixed_voltage(pdo),
					       pdo_fixed_voltage(pdo),
					       pdo_fixed_current(pdo));
			break;
		case PDO_TYPE_VAR:
			ok = pdo_min_voltage(pdo) <= pdo_max_voltage(pdo) &&
			     bm92t_board_takes(info, pdo_min_voltage(pdo),
					       pdo_max_voltage(pdo),
					       pdo_max_current(pdo));
			break;
		default:
			ok = false;
			break;
		}

		if (!ok) {
			dev_err(info->dev,
				"firmware sink PDO %08x is not covered by sink-pdos\n",
				pdo);
			return -EINVAL;
		}
	}

	return 0;
}

static int bm92t_read_sink_pdos(struct bm92t_info *info)
{
	struct device *dev = info->dev;
	unsigned int i;
	int ret;

	ret = fwnode_property_count_u32(info->connector, "sink-pdos");
	if (ret <= 0 || ret > PDO_MAX_OBJECTS)
		return dev_err_probe(dev, -EINVAL, "invalid sink-pdos\n");
	info->nr_sink_pdos = ret;

	ret = fwnode_property_read_u32_array(info->connector, "sink-pdos",
					     info->sink_pdos,
					     info->nr_sink_pdos);
	if (ret)
		return ret;

	for (i = 0; i < info->nr_sink_pdos; i++) {
		u32 pdo = info->sink_pdos[i];

		if ((pdo_type(pdo) != PDO_TYPE_FIXED &&
		     pdo_type(pdo) != PDO_TYPE_VAR) ||
		    (!i && (pdo_type(pdo) != PDO_TYPE_FIXED ||
			    pdo_fixed_voltage(pdo) != 5000)))
			return dev_err_probe(dev, -EINVAL,
					     "sink-pdos: fixed and variable supplies, fixed 5 V first\n");
	}

	ret = fwnode_property_read_u32(info->connector, "op-sink-microwatt",
				       &info->op_sink_mw);
	if (ret)
		return dev_err_probe(dev, ret, "no op-sink-microwatt\n");
	info->op_sink_mw /= 1000;

	return 0;
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
	mutex_init(&info->req_mutex);
	init_completion(&info->req_done);
	init_completion(&info->dump_done);
	INIT_WORK(&info->work, bm92t_work);
	INIT_DELAYED_WORK(&info->dp_work, bm92t_dp_reply_work);
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

	ret = bm92t_read_sink_pdos(info) ?: bm92t_check_fw_sink(info);
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
		goto err_port;

	/* Leave the role alone until the port says what it is */
	ret = devm_request_irq(dev, client->irq, bm92t_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), info);
	if (ret)
		goto err_port;

	/* A command a previous driver left behind would block ours */
	ret = bm92t_read16(info, STATUS1_REG, &status1);
	if (!ret && (status1 & STATUS1_INSERT) &&
	    FIELD_GET(STATUS1_LASTCMD, status1) != LASTCMD_COMPLETE)
		ret = bm92t_write16(info, COMMAND_REG, ABORT_LASTCMD_SENT_CMD);
	ret = ret ?: bm92t_hw_init(info);
	if (ret) {
		dev_err_probe(dev, ret, "controller setup failed\n");
		goto err_port;
	}

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
	if (info->att.attached)
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

	bm92t_nd_set_led(info, 0, 0, 0, 128);
	bm92t_stop(info);
	bm92t_update16(info, CONFIG1_REG, CONFIG1_SPDSRC,
		       FIELD_PREP(CONFIG1_SPDSRC, SPDSRC12_OFF));
	bm92t_set_vbus(info, false);
}

static int bm92t_suspend(struct device *dev)
{
	struct bm92t_info *info = dev_get_drvdata(dev);

	/* Breathing dock LED while asleep */
	bm92t_nd_set_led(info, 32, 1, 255, 255);
	bm92t_stop(info);

	return 0;
}

static int bm92t_resume(struct device *dev)
{
	struct bm92t_info *info = dev_get_drvdata(dev);

	bm92t_start(info);
	bm92t_nd_set_led(info, 128, 0, 0, 64);

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
