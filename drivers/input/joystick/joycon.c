// SPDX-License-Identifier: GPL-2.0+
/*
 * Nintendo Joy-Con serial gamepad driver
 *
 * Copyright (c) 2018 Max Thomas
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/crc8.h>
#include <linux/gpio/consumer.h>
#include <linux/hex.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nintendo-joycon.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define JOYCON_BAUD (1000000)

/* Frame commands. */
#define JOYCON_CMD_WIRED	(0x91)
#define JOYCON_CMD_HID		(0x92)
#define JOYCON_CMD_INIT_REPLY	(0x94)
#define JOYCON_CMD_HANDSHAKE	(0xA5)

/* Wired subcommands, which open the rail session. */
#define JOYCON_WIRED_GET_INFO		(0x01)
#define JOYCON_WIRED_HID_CONNECT	(0x10)
#define JOYCON_WIRED_HID_DISCONNECT	(0x11)
#define JOYCON_WIRED_SET_HID_RATE	(0x12)

/* Output report ids: the rumble-only one carries no subcommand. */
#define JOYCON_OUT_SUBCOMMAND	(0x01)
#define JOYCON_OUT_RUMBLE	(0x10)
/* Any HID frame asks for an input report; what it carries does not matter. */
#define JOYCON_OUT_POLL		(0x1F)

/* Input report ids. */
#define JOYCON_IN_SUBCMD_REPLY	(0x21)
#define JOYCON_IN_FULL		(0x30)

#define JOYCON_HID_MANUAL_PAIRING (0x1)
#define JOYCON_HID_DEVICE_INFO (0x2)
#define JOYCON_HID_SET_REPORT_MODE (0x3)
#define JOYCON_HID_SET_SHIPMENT (0x8)
#define JOYCON_HID_SPI_READ    (0x10)
#define JOYCON_HID_ENABLE_IMU (0x40)
#define JOYCON_HID_ENABLE_VIBRATION (0x48)
#define JOYCON_HID_GET_REGVOLT (0x50)
#define JOYCON_HID_SET_CHARGE (0x51)

/* Bit 7 of the byte ahead of the echoed subcommand; clear on a refusal. */
#define JOYCON_SUBCMD_ACK	BIT(7)

/* Charge switches: bit 2 arms the 100mA path, bit 4 the 200mA one. */
#define JOYCON_CHARGE_200MA (0x14)

/* Battery and connection byte of an input report. */
#define JOYCON_BAT_CHARGING	BIT(4)
#define JOYCON_BAT_LEVEL	GENMASK(7, 5)

/* Six 12-bit values in nine bytes; 0xFF for the stick a half does not have. */
#define JOYCON_CAL_FCT_STRIDE \
	(JC_CAL_FCT_DATA_RIGHT_ADDR - JC_CAL_FCT_DATA_LEFT_ADDR)
#define JOYCON_CAL_USR_STRIDE \
	(JC_CAL_USR_RIGHT_MAGIC_ADDR - JC_CAL_USR_LEFT_MAGIC_ADDR)

/* An SPI read reply: the 32-bit address and the length ahead of the data. */
#define JOYCON_SPI_HDR_SIZE (5)

/* Three IMU samples follow the vibrator report byte in a full input report. */
#define JOYCON_IMU_OFF (13)
#define JOYCON_IMU_SAMPLES (3)
#define JOYCON_IMU_REPORT_SIZE \
	(JOYCON_IMU_OFF + JOYCON_IMU_SAMPLES * sizeof(struct joycon_imu_data))

/* What a sample is worth in time when the measured interval is unusable. */
#define JOYCON_IMU_DFLT_DELTA_MS (16)
#define JOYCON_IMU_MAX_DELTA_MS (100)

/* The magic and the calibration itself, in one read. */
#define JOYCON_IMU_CAL_USR_LEN \
	(JC_CAL_USR_MAGIC_SIZE + JC_IMU_CAL_DATA_SIZE)

/* One slot per pairing; the live one carries the magic, and pairing appends. */
#define JOYCON_PAIRING_ADDR		(0x2000)
#define JOYCON_PAIRING_SECTION_SIZE	(0x1000)
#define JOYCON_PAIRING_SLOT_SIZE	(0x26)
#define JOYCON_PAIRING_SLOTS \
	(JOYCON_PAIRING_SECTION_SIZE / JOYCON_PAIRING_SLOT_SIZE)
#define JOYCON_PAIRING_READ_LEN		(0x1a)
#define JOYCON_PAIRING_MAGIC		(0x95)
#define JOYCON_PAIRING_BODY_SIZE	(0x22)
#define JOYCON_PAIRING_HOST_OFF		(0x04)
#define JOYCON_PAIRING_LTK_OFF		(0x0a)
#define JOYCON_PAIRING_LTK_SIZE		(16)

/* Steps of one sequence, entered through REQ_HOST; a reply names the step. */
#define JOYCON_PAIR_REQ_ADDR		(0x01)
#define JOYCON_PAIR_REQ_LTK		(0x02)
#define JOYCON_PAIR_REQ_SAVE		(0x03)
#define JOYCON_PAIR_REQ_HOST		(0x04)
#define JOYCON_PAIR_CAPS		(0x68)
#define JOYCON_PAIR_NAME_LEN		(20)

/* Enables fast connect and real sleep; sent once, just before pairing. */
#define JOYCON_SHIPMENT_CLEAR		(0x00)

#define JOYCON_CRC8_POLY (0x8D)

/* The poll; everything else runs on a multiple of it. */
#define JOYCON_POLL_MS		(16)
#define JOYCON_SLOW_MS		(200)
/* Four slow ticks without a report and the session is opened again. */
#define JOYCON_GONE_MS		(4 * JOYCON_SLOW_MS)
/* Out of system sleep a controller takes about two seconds to answer. */
#define JOYCON_RESYNC_MS	(5000)
/* A reply comes with the next report; a subcommand without one is resent. */
#define JOYCON_SUBCMD_TIMEOUT_MS (3 * JOYCON_POLL_MS)
/* An init step after the handshake: up to 400 ms measured (right, 4.20). */
#define JOYCON_INIT_REPLY_MS	(1000)
/* Covers a first shipment clear, which writes flash before it answers. */
#define JOYCON_REQ_TIMEOUT_MS	(3000)

/* Wakes a rail that has gone quiet, ahead of a fresh handshake. */
static const u8 joycon_wake[4] = {0xA1, 0xA2, 0xA3, 0xA4};

/* Has a layout of its own: a reply echo and a magic where the sizes would be. */
static const u8 joycon_handshake[] = {
	0x19, 0x01, 0x03, 0x07, 0x00, 0xA5, 0x02, 0x01, 0x7E, 0x00, 0x00, 0x00
};

/* type, destination, group; the destination byte differs by direction. */
static const u8 joycon_magic_send[3] = {0x19, 0x01, 0x03};
static const u8 joycon_magic_recv[3] = {0x19, 0x81, 0x03};

/* Several times larger than the biggest frame a controller sends. */
#define JOYCON_RX_BUF_SIZE (512)

struct joycon_frame_header {
	u8 magic[3];
	/* Size of everything that follows this field. */
	__le16 total_size;
	u8 cmd;
	u8 subcmd;
	__le16 payload_size;
	u8 status;
	u8 payload_crc;
	/* Covers cmd up to and including payload_crc. */
	u8 header_crc;
} __packed;

#define JOYCON_HEADER_CRC_LEN \
	(offsetof(struct joycon_frame_header, header_crc) - \
	 offsetof(struct joycon_frame_header, cmd))

/* A subcommand payload is a fixed 48 bytes: 10 of header, 38 of arguments. */
#define JOYCON_SUBCMD_DATA_SIZE	(48)
#define JOYCON_SUBCMD_ARGS_OFF	(10)
#define JOYCON_SUBCMD_ARGS_SIZE	(JOYCON_SUBCMD_DATA_SIZE - JOYCON_SUBCMD_ARGS_OFF)

/* A poll, a rumble report and a subcommand: the most one tick sends. */
#define JOYCON_TX_SIZE \
	(3 * sizeof(struct joycon_frame_header) + 1 + \
	 (1 + 1 + JC_RUMBLE_DATA_SIZE) + (1 + JOYCON_SUBCMD_DATA_SIZE))

/* The rail session opens in this order, each step on the last one's reply. */
static const struct {
	u8 id;
	/* Where hekate requires a zero status. */
	bool status_ok;
} joycon_init_steps[] = {
	{ JOYCON_CMD_HANDSHAKE },
	/* Its reply carries the controller's address. */
	{ JOYCON_WIRED_GET_INFO, true },
	{ JOYCON_WIRED_HID_DISCONNECT },
	{ JOYCON_WIRED_HID_CONNECT, true },
	/* 1 megabaud carries the 15ms report rate already. */
	{ JOYCON_WIRED_SET_HID_RATE },
};

/* The SPI reads a controller is walked through, in the order they are sent. */
enum joycon_cal_step {
	JOYCON_CAL_STICK_FCT,
	JOYCON_CAL_STICK_USR,
	JOYCON_CAL_IMU_FCT,
	JOYCON_CAL_IMU_USR,
	JOYCON_CAL_PAIRING,
	JOYCON_CAL_STEPS,
};

/* What userspace has asked for through sysfs, carried out by the work. */
enum joycon_req {
	JOYCON_REQ_NONE,
	JOYCON_REQ_PAIR,
};

struct joycon_type_info {
	u8 type;
	const char *name;
	u16 product;
	const struct joycon_ctlr_button_mapping *btn_map;
	/* Never a grip, so SL/SR take the unused triggers. */
	const struct joycon_ctlr_button_mapping *btn_map_s;
	bool left;
};

/* Only the two halves fit on a rail. */
static const struct joycon_type_info joycon_types[] = {
	{ JOYCON_CTLR_TYPE_JCL, "Nintendo Switch Left Joy-Con (Serial)",
	  JC_PRODUCT_JOYCONL, left_joycon_button_mappings,
	  left_joycon_s_button_mappings, true },
	{ JOYCON_CTLR_TYPE_JCR, "Nintendo Switch Right Joy-Con (Serial)",
	  JC_PRODUCT_JOYCONR, right_joycon_button_mappings,
	  right_joycon_s_button_mappings, false },
};

/* Per-Joy-Con data. */

struct joycon {
	struct serdev_device *serdev;
	struct device *dev;
	struct power_supply_desc batt_desc;
	struct gpio_desc *charge_gpio;
	struct regulator *charge_reg;

	/* The only writer to the port, and the only one to (un)register. */
	struct workqueue_struct *wq;
	struct delayed_work work;

	/* One sysfs request at a time; it waits for the work, not for this. */
	struct mutex req_mutex;

	/* Everything below; never held to write or to (un)register. */
	struct mutex lock;
	/* The work may be queued: not in system sleep, not once removed. */
	bool running;

	/* Opening the session: the step whose reply is awaited. */
	unsigned int init_step;
	bool init_sent;
	unsigned long init_deadline;
	bool initialized;
	/* Handshaking again after resume, keeping what is registered. */
	bool resync;
	unsigned long resync_deadline;
	/* Left or swapped: the work takes the devices away. */
	bool gone;
	unsigned long last_rx;
	unsigned long next_slow;

	/* The subcommand awaiting its reply, 0 if none: one at a time. */
	u8 pending;
	unsigned long pending_deadline;
	enum joycon_cal_step pending_cal;

	enum joycon_req req;
	/* Told apart from the request before it by a write still returning. */
	unsigned int req_gen;
	/* The request's next step is due. */
	bool req_send;
	int req_err;
	struct completion req_done;
	/* Pairing: the step's subcommand (0 once saved), and the step. */
	u8 pair_subcmd;
	u8 pair_type;
	u8 pair_host[6];

	/* Learnt from the controller, and dropped when it comes or goes. */
	const struct joycon_type_info *info;
	bool unsupported;
	bool full_mode;
	bool vib_enabled;
	bool imu_enabled;
	bool regvolt_due;
	/* Which steps of the chain have been answered, indexed by the enum. */
	bool cal_parsed[JOYCON_CAL_STEPS];
	struct joycon_stick_cal left_cal_x, left_cal_y;
	struct joycon_stick_cal right_cal_x, right_cal_y;
	struct joycon_imu_cal accel_cal, gyro_cal;
	u8 mac[6];

	/* What we hand userspace, advanced by the interval a report covers. */
	unsigned int imu_timestamp_us;
	unsigned int imu_last_pkt_ms;

	/* 0 until the controller has answered. */
	unsigned int regvolt_mv;
	u8 bat_con;
	bool bat_valid;

	u8 pairing_host[6];
	u8 pairing_ltk[JOYCON_PAIRING_LTK_SIZE];
	/* Where the walk is, and how many slots it has read. */
	u8 pairing_slot;
	u8 pairing_reads;
	bool pairing_valid;

	/* Policy, then what is applied: the switch and the regulator vote. */
	bool charge_wanted;
	bool charge_inhibit;
	bool charge_on;
	bool charge_reg_on;
	bool charge_acked;

	struct input_dev *input_dev;
	struct input_dev *imu_dev;
	struct power_supply *batt_dev;
	bool batt_registering;
	/* Embedded rather than allocated: a rail may be re-populated forever. */
	char phys[64];
	char uniq[24];
	char imu_name[64];

	/* Written from the force feedback callback, which cannot sleep. */
	spinlock_t rumble_lock;
	u8 rumble_data[JC_RUMBLE_DATA_SIZE];
	bool rumble_on;
	unsigned int rumble_zero_left;

	/* Partial frame carried over between receive_buf() calls. */
	u8 rx_buf[JOYCON_RX_BUF_SIZE];
	size_t rx_len;
};

/* What one tick put on the wire, for when the write fails. */
struct joycon_tx {
	u8 subcmd;
	bool req;
	unsigned int req_gen;
};

/* Taken away outside the lock: unregistering waits for their callbacks. */
struct joycon_devs {
	struct input_dev *input;
	struct input_dev *imu;
	struct power_supply *batt;
};

DECLARE_CRC8_TABLE(joycon_crc_table);

/* Copy @len bytes back to front, as the controller stores addresses and keys. */
static void joycon_memcpy_rev(u8 *dst, const u8 *src, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++)
		dst[i] = src[len - 1 - i];
}

static size_t joycon_frame_craft(u8 *buf, u8 cmd, u8 subcmd,
				 const void *payload, u16 len)
{
	struct joycon_frame_header *hdr = (struct joycon_frame_header *)buf;

	memcpy(hdr->magic, joycon_magic_send, sizeof(hdr->magic));
	hdr->total_size = cpu_to_le16(sizeof(*hdr) -
				      offsetofend(typeof(*hdr), total_size) + len);
	hdr->cmd = cmd;
	hdr->subcmd = subcmd;
	hdr->payload_size = cpu_to_le16(len);
	hdr->status = 0;
	memcpy(buf + sizeof(*hdr), payload, len);
	hdr->payload_crc = crc8(joycon_crc_table, buf + sizeof(*hdr), len, 0);
	hdr->header_crc = crc8(joycon_crc_table, &hdr->cmd,
			       JOYCON_HEADER_CRC_LEN, 0);

	return sizeof(*hdr) + len;
}

/* Output report @id; @data starts with the packet counter. */
static size_t joycon_hid_craft(u8 *buf, u8 id, const u8 *data, size_t len)
{
	u8 payload[1 + JOYCON_SUBCMD_DATA_SIZE] = { id };

	memcpy(payload + 1, data, len);

	return joycon_frame_craft(buf, JOYCON_CMD_HID, 0, payload, 1 + len);
}

static size_t joycon_subcmd_craft(u8 *buf, u8 subcmd, const u8 *args,
				  size_t len)
{
	static const u8 rumble_neutral[JC_RUMBLE_DATA_SIZE] = {
		0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40
	};
	u8 data[JOYCON_SUBCMD_DATA_SIZE] = { };

	/* data[0] is the console's packet counter; the controller ignores ours. */
	memcpy(&data[1], rumble_neutral, sizeof(rumble_neutral));
	data[9] = subcmd;
	memcpy(&data[JOYCON_SUBCMD_ARGS_OFF], args, len);

	return joycon_hid_craft(buf, JOYCON_OUT_SUBCOMMAND, data, sizeof(data));
}

/* Both CRCs; the handshake reply has no payload fields to check. */
static bool joycon_frame_valid(const u8 *frame, size_t len)
{
	const struct joycon_frame_header *hdr =
		(const struct joycon_frame_header *)frame;
	size_t payload_len;

	if (crc8(joycon_crc_table, &hdr->cmd, JOYCON_HEADER_CRC_LEN, 0) !=
	    hdr->header_crc)
		return false;

	if (hdr->cmd == JOYCON_CMD_HANDSHAKE)
		return true;

	payload_len = le16_to_cpu(hdr->payload_size);
	return sizeof(*hdr) + payload_len == len &&
	       crc8(joycon_crc_table, frame + sizeof(*hdr), payload_len, 0) ==
	       hdr->payload_crc;
}

static const struct joycon_type_info *joycon_type(u8 ctlr_type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(joycon_types); i++)
		if (joycon_types[i].type == ctlr_type)
			return &joycon_types[i];

	return NULL;
}

static bool joycon_charge_target(const struct joycon *joycon)
{
	return joycon->charge_wanted && !joycon->charge_inhibit;
}

/* Power the rail; the controller's own switch is armed from the poll. */
static int joycon_update_charging(struct joycon *joycon)
{
	bool on = joycon_charge_target(joycon);
	int err;

	lockdep_assert_held(&joycon->lock);

	if (on && !joycon->charge_reg_on) {
		err = regulator_enable(joycon->charge_reg);
		if (err)
			return err;
		joycon->charge_reg_on = true;
	}

	if (joycon->charge_on != on) {
		gpiod_set_value_cansleep(joycon->charge_gpio, on);
		joycon->charge_on = on;
		joycon->charge_acked = false;
		if (joycon->batt_dev)
			power_supply_changed(joycon->batt_dev);
	}

	/* The switch is open either way; a vote still held is retried. */
	if (!on && joycon->charge_reg_on) {
		err = regulator_disable(joycon->charge_reg);
		if (err)
			return err;
		joycon->charge_reg_on = false;
	}

	return 0;
}

/* What the controller is attached for; retried from the slow tick. */
static void joycon_set_charging(struct joycon *joycon, bool on)
{
	int err;

	/* The knob is on the controller's battery, so it leaves with it. */
	if (!on)
		joycon->charge_inhibit = false;
	joycon->charge_wanted = on;

	err = joycon_update_charging(joycon);
	if (err)
		dev_err_ratelimited(joycon->dev, "cannot %s charging %d\n",
				    on ? "enable" : "disable", err);
}

static const enum power_supply_property joycon_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

/* The level the battery byte reports, read as hid-nintendo reads it. */
static int joycon_capacity_level(u8 bat_con)
{
	static const u8 levels[] = {
		POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL,
		POWER_SUPPLY_CAPACITY_LEVEL_LOW,
		POWER_SUPPLY_CAPACITY_LEVEL_NORMAL,
		POWER_SUPPLY_CAPACITY_LEVEL_HIGH,
		POWER_SUPPLY_CAPACITY_LEVEL_FULL,
	};
	unsigned int level = FIELD_GET(JOYCON_BAT_LEVEL, bat_con);

	return level < ARRAY_SIZE(levels) ? levels[level] :
	       POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}

static int joycon_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct joycon *joycon = power_supply_get_drvdata(psy);
	int level;

	guard(mutex)(&joycon->lock);

	level = joycon->bat_valid ? joycon_capacity_level(joycon->bat_con) :
		POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		/* Bit 0, "powered", stays clear on the rails even while charging. */
		if (!joycon->bat_valid)
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else if (joycon->bat_con & JOYCON_BAT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (!joycon->charge_on)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (level == POWER_SUPPLY_CAPACITY_LEVEL_FULL)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = level;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!joycon->regvolt_mv)
			return -ENODATA;
		val->intval = joycon->regvolt_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = joycon->charge_inhibit ?
			      POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
			      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Inhibiting drops the rail's vote on the 5V boost, not just the switch. */
static int joycon_battery_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct joycon *joycon = power_supply_get_drvdata(psy);
	bool inhibit_was, on_was;
	int err;

	if (psp != POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR)
		return -EINVAL;
	/* The core passes on any behaviour it knows, not just the ones listed. */
	if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
	    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
		return -EINVAL;

	guard(mutex)(&joycon->lock);

	/* A detached battery's write would carry over to the next controller. */
	if (psy != joycon->batt_dev && !joycon->batt_registering)
		return -ENODEV;

	inhibit_was = joycon->charge_inhibit;
	on_was = joycon->charge_on;
	joycon->charge_inhibit =
		val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;

	err = joycon_update_charging(joycon);
	if (err) {
		/* The vote is still held: this only closes the switch again. */
		joycon->charge_inhibit = inhibit_was;
		joycon_update_charging(joycon);
		return err;
	}

	/* A switch that moved has notified already. */
	if (joycon->charge_inhibit != inhibit_was &&
	    joycon->charge_on == on_was)
		power_supply_changed(psy);

	return 0;
}

static int joycon_battery_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

/* Called from the force feedback timer, so it only stages the bytes. */
static int joycon_play_effect(struct input_dev *dev, void *data,
			      struct ff_effect *effect)
{
	struct joycon *joycon = input_get_drvdata(dev);
	u16 amp_l = effect->u.rumble.strong_magnitude;
	u16 amp_r = effect->u.rumble.weak_magnitude;
	u8 encoded[JC_RUMBLE_DATA_SIZE];

	if (effect->type != FF_RUMBLE)
		return 0;

	/* A half takes the pair for its own side out of the same eight bytes. */
	joycon_encode_rumble(encoded, JC_RUMBLE_DFLT_LOW_FREQ,
			     JC_RUMBLE_DFLT_HIGH_FREQ,
			     amp_l * (u32)joycon_max_rumble_amp / 65535);
	joycon_encode_rumble(encoded + 4, JC_RUMBLE_DFLT_LOW_FREQ,
			     JC_RUMBLE_DFLT_HIGH_FREQ,
			     amp_r * (u32)joycon_max_rumble_amp / 65535);

	guard(spinlock_irqsave)(&joycon->rumble_lock);

	memcpy(joycon->rumble_data, encoded, sizeof(joycon->rumble_data));
	joycon->rumble_on = amp_l || amp_r;
	/* Silence has to be sent too, and the controller may miss one packet. */
	if (!joycon->rumble_on)
		joycon->rumble_zero_left = JC_RUMBLE_ZERO_AMP_PKT_CNT;

	return 0;
}

/* Byte for byte as a console sends it; the host address goes out reversed. */
static size_t joycon_pair_request(const struct joycon *joycon, u8 *req)
{
	u8 *p = req;

	*p++ = JOYCON_PAIR_REQ_HOST;

	joycon_memcpy_rev(p, joycon->pair_host, sizeof(joycon->pair_host));
	p += sizeof(joycon->pair_host);

	memcpy(p, (u8[]){ 0x00, 0x04, 0x3c }, 3);
	p += 3;

	/* What the controller remembers this host as. */
	strscpy_pad(p, "Nintendo Switch", JOYCON_PAIR_NAME_LEN);
	p += JOYCON_PAIR_NAME_LEN;

	*p++ = JOYCON_PAIR_CAPS;
	*p++ = 0x00;

	memcpy(p, (u8[]){ 0x85, 0xaf, 0xa4, 0x69, 0x2a, 0x00 }, 6);
	p += 6;

	return p - req;
}

/* Where the slot we are currently probing lives. */
static u32 joycon_pairing_addr(const struct joycon *joycon)
{
	return JOYCON_PAIRING_ADDR +
	       joycon->pairing_slot * JOYCON_PAIRING_SLOT_SIZE;
}

/* Walk the slots again, from the last live one: a pairing moves it on. */
static void joycon_pairing_forget(struct joycon *joycon)
{
	joycon->cal_parsed[JOYCON_CAL_PAIRING] = false;
	joycon->pairing_valid = false;
	joycon->pairing_reads = 0;
}

/* Open the session again, keeping the controller and what is registered. */
static void joycon_resync(struct joycon *joycon, unsigned int timeout_ms)
{
	joycon->initialized = false;
	joycon->resync = true;
	joycon->resync_deadline = jiffies + msecs_to_jiffies(timeout_ms);
	joycon->init_step = 0;
	joycon->init_sent = false;
	joycon->pending = 0;
}

/* A failed pairing may still be answered: reopen the session, reread. */
static void joycon_req_finish(struct joycon *joycon, int err)
{
	if (joycon->req == JOYCON_REQ_NONE)
		return;

	if (err) {
		if (joycon->initialized)
			joycon_resync(joycon, JOYCON_RESYNC_MS);
		joycon_pairing_forget(joycon);
	}

	joycon->req = JOYCON_REQ_NONE;
	joycon->req_send = false;
	joycon->pair_subcmd = 0;
	joycon->req_err = err;
	complete(&joycon->req_done);
}

/* Everything learnt from a controller, dropped when one comes or goes. */
static void joycon_reset_ctlr_state(struct joycon *joycon)
{
	joycon->initialized = false;
	joycon->resync = false;
	joycon->gone = false;
	joycon->init_step = 0;
	joycon->init_sent = false;
	joycon->pending = 0;

	joycon->info = NULL;
	joycon->unsupported = false;
	joycon->full_mode = false;
	joycon->vib_enabled = false;
	joycon->imu_enabled = false;
	joycon->regvolt_due = false;
	memset(joycon->cal_parsed, 0, sizeof(joycon->cal_parsed));
	/* Until the flash has been read; the raw readings are unusable without. */
	joycon_stick_cal_defaults(&joycon->left_cal_x, &joycon->left_cal_y);
	joycon_stick_cal_defaults(&joycon->right_cal_x, &joycon->right_cal_y);
	joycon_imu_cal_defaults(&joycon->accel_cal, &joycon->gyro_cal);
	joycon->imu_last_pkt_ms = 0;
	joycon->imu_timestamp_us = 0;
	joycon->regvolt_mv = 0;
	joycon->bat_con = 0;
	joycon->bat_valid = false;
	joycon_pairing_forget(joycon);
	joycon->pairing_slot = 0;

	scoped_guard(spinlock_irqsave, &joycon->rumble_lock) {
		joycon->rumble_on = false;
		joycon->rumble_zero_left = 0;
	}
}

/* The controller left: fail what was asked of it, collect its devices. */
static void joycon_detach(struct joycon *joycon, struct joycon_devs *devs)
{
	lockdep_assert_held(&joycon->lock);

	devs->input = joycon->input_dev;
	devs->imu = joycon->imu_dev;
	devs->batt = joycon->batt_dev;
	joycon->input_dev = NULL;
	joycon->imu_dev = NULL;
	joycon->batt_dev = NULL;

	joycon_req_finish(joycon, -ENODEV);
	joycon_set_charging(joycon, false);
	joycon_reset_ctlr_state(joycon);
}

static void joycon_unregister(struct joycon *joycon, struct joycon_devs *devs)
{
	if (devs->imu)
		input_unregister_device(devs->imu);

	if (devs->input) {
		input_unregister_device(devs->input);
		dev_info(joycon->dev, "disconnected\n");
	}

	if (devs->batt)
		power_supply_unregister(devs->batt);
}

/* Everything but the name, which differs between the two devices we make. */
static void joycon_set_dev_ids(struct input_dev *dev, struct joycon *joycon,
			       const struct joycon_type_info *info)
{
	dev->phys = joycon->phys;
	dev->uniq = joycon->uniq;
	dev->dev.parent = joycon->dev;
	dev->id.bustype = BUS_RS232;
	dev->id.vendor = JC_VENDOR_NINTENDO;
	dev->id.product = info->product;
	dev->id.version = 0x0100;

	input_set_drvdata(dev, joycon);
}

static struct input_dev *joycon_input_create(struct joycon *joycon,
					     const struct joycon_type_info *info)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return ERR_PTR(-ENOMEM);

	snprintf(joycon->phys, sizeof(joycon->phys), "%s/input0",
		 dev_name(joycon->dev));
	snprintf(joycon->uniq, sizeof(joycon->uniq), "%pM", joycon->mac);

	dev->name = info->name;
	joycon_set_dev_ids(dev, joycon, info);

	if (info->left)
		joycon_config_left_stick(dev);
	else
		joycon_config_right_stick(dev);

	joycon_config_buttons(dev, info->btn_map);
	joycon_config_buttons(dev, info->btn_map_s);

	input_set_capability(dev, EV_FF, FF_RUMBLE);
	err = input_ff_create_memless(dev, NULL, joycon_play_effect);
	if (err)
		goto err_free;

	err = input_register_device(dev);
	if (err)
		goto err_free;

	return dev;

err_free:
	input_free_device(dev);
	return ERR_PTR(err);
}

/* A second device, as hid-nintendo does it: the axes would collide otherwise. */
static struct input_dev *joycon_imu_create(struct joycon *joycon,
					   const struct joycon_type_info *info)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return NULL;

	snprintf(joycon->imu_name, sizeof(joycon->imu_name), "%s (IMU)",
		 info->name);

	dev->name = joycon->imu_name;
	joycon_set_dev_ids(dev, joycon, info);
	joycon_config_imu(dev);

	err = input_register_device(dev);
	if (err) {
		input_free_device(dev);
		dev_err(joycon->dev, "cannot register imu device %d\n", err);
		return NULL;
	}

	return dev;
}

/* Outside the lock: registering reads the power supply back. */
static void joycon_register(struct joycon *joycon,
			    const struct joycon_type_info *info)
{
	struct power_supply_config psy_cfg = {
		.fwnode = dev_fwnode(joycon->dev),
		.drv_data = joycon,
	};
	struct power_supply *batt;
	struct input_dev *input, *imu;

	input = joycon_input_create(joycon, info);
	if (IS_ERR(input)) {
		dev_err_ratelimited(joycon->dev, "cannot register input device %pe\n",
				    input);
		return;
	}

	/* Neither is fatal: the controller is still perfectly usable. */
	imu = joycon_imu_create(joycon, info);

	/* Writes during registration, from the add uevent, are this battery's. */
	scoped_guard(mutex, &joycon->lock)
		joycon->batt_registering = true;
	batt = power_supply_register(joycon->dev, &joycon->batt_desc, &psy_cfg);
	if (IS_ERR(batt)) {
		dev_err(joycon->dev, "cannot register power supply %pe\n", batt);
		batt = NULL;
	}

	/* A controller that left meanwhile is taken away by the next tick. */
	scoped_guard(mutex, &joycon->lock) {
		joycon->batt_registering = false;
		joycon->input_dev = input;
		joycon->imu_dev = imu;
		joycon->batt_dev = batt;
	}

	dev_info(joycon->dev, "%s\n", input->name);
}

/* Every JOYCON_SLOW_MS: the empty rail, the voltage and charging retries. */
static void joycon_slow_tick(struct joycon *joycon)
{
	unsigned long now = jiffies;

	if (time_before(now, joycon->next_slow))
		return;
	joycon->next_slow = now + msecs_to_jiffies(JOYCON_SLOW_MS);

	/* Silent: empty, or dropped the session to erase its pairing flash. */
	if (joycon->initialized &&
	    time_after(now, joycon->last_rx + msecs_to_jiffies(JOYCON_GONE_MS)))
		joycon_resync(joycon, JOYCON_GONE_MS);
	if (joycon->resync && time_after(now, joycon->resync_deadline))
		joycon->gone = true;

	if (joycon->charge_on != joycon_charge_target(joycon) ||
	    joycon->charge_reg_on != joycon_charge_target(joycon))
		joycon_set_charging(joycon, joycon->charge_wanted);

	if (joycon->initialized)
		joycon->regvolt_due = true;
}

/* Addresses in chain order; the pairing slot moves, so it asks for its own. */
static const struct {
	u32 addr;
	u8 len;
} joycon_cal_reads[JOYCON_CAL_STEPS] = {
	[JOYCON_CAL_STICK_FCT] = { JC_CAL_FCT_DATA_LEFT_ADDR,
				   2 * JC_CAL_STICK_DATA_SIZE },
	[JOYCON_CAL_STICK_USR] = { JC_CAL_USR_LEFT_MAGIC_ADDR,
				   2 * JOYCON_CAL_USR_STRIDE },
	[JOYCON_CAL_IMU_FCT]   = { JC_IMU_CAL_FCT_DATA_ADDR,
				   JC_IMU_CAL_DATA_SIZE },
	[JOYCON_CAL_IMU_USR]   = { JC_IMU_CAL_USR_MAGIC_ADDR,
				   JOYCON_IMU_CAL_USR_LEN },
	[JOYCON_CAL_PAIRING]   = { 0, JOYCON_PAIRING_READ_LEN },
};

static u32 joycon_cal_addr(const struct joycon *joycon, enum joycon_cal_step step)
{
	return joycon_cal_reads[step].addr ?: joycon_pairing_addr(joycon);
}

/* Userspace's request first, then setup, then the voltage; 0 for none. */
static u8 joycon_next_subcmd(struct joycon *joycon, u8 *args, size_t *len)
{
	int i;

	*len = 0;

	if (joycon->req_send) {
		joycon->req_send = false;

		if (joycon->pair_subcmd == JOYCON_HID_SET_SHIPMENT)
			args[(*len)++] = JOYCON_SHIPMENT_CLEAR;
		else if (joycon->pair_type == JOYCON_PAIR_REQ_HOST)
			*len = joycon_pair_request(joycon, args);
		else
			args[(*len)++] = joycon->pair_type;
		return joycon->pair_subcmd;
	}

	if (!joycon->info)
		return joycon->unsupported ? 0 : JOYCON_HID_DEVICE_INFO;

	/* Polls go unanswered after a Bluetooth reconnect attempt until this. */
	if (!joycon->full_mode) {
		args[(*len)++] = JOYCON_IN_FULL;
		return JOYCON_HID_SET_REPORT_MODE;
	}

	/* The motor ignores rumble packets until this arrives. */
	if (!joycon->vib_enabled) {
		args[(*len)++] = 1;
		return JOYCON_HID_ENABLE_VIBRATION;
	}

	if (!joycon->imu_enabled) {
		args[(*len)++] = 1;
		return JOYCON_HID_ENABLE_IMU;
	}

	if (joycon->charge_on && !joycon->charge_acked) {
		args[(*len)++] = JOYCON_CHARGE_200MA;
		return JOYCON_HID_SET_CHARGE;
	}

	for (i = 0; i < JOYCON_CAL_STEPS; i++) {
		if (joycon->cal_parsed[i])
			continue;

		/* A 32-bit little endian address, then the length. */
		put_unaligned_le32(joycon_cal_addr(joycon, i), args);
		args[4] = joycon_cal_reads[i].len;
		*len = JOYCON_SPI_HDR_SIZE;
		joycon->pending_cal = i;
		return JOYCON_HID_SPI_READ;
	}

	if (joycon->regvolt_due) {
		joycon->regvolt_due = false;
		return JOYCON_HID_GET_REGVOLT;
	}

	return 0;
}

/* The init step that is due, or the poll and what rides on it. */
static size_t joycon_tx_compose(struct joycon *joycon, u8 *buf,
				struct joycon_tx *tx)
{
	u8 rumble[1 + JC_RUMBLE_DATA_SIZE] = { };
	u8 args[JOYCON_SUBCMD_ARGS_SIZE];
	unsigned long now = jiffies;
	size_t len, args_len;
	bool send_rumble;

	if (!joycon->initialized) {
		/* Output report rate 15 ms. */
		static const u8 hid_rate[] = { 0x0F, 0x00, 0x00, 0x00 };
		u8 id;

		if (joycon->init_sent && time_before(now, joycon->init_deadline))
			return 0;
		/* Unanswered: start over, which also wakes a quiet rail. */
		if (joycon->init_sent)
			joycon->init_step = 0;

		joycon->init_sent = true;
		joycon->init_deadline = now + msecs_to_jiffies(joycon->init_step ?
			JOYCON_INIT_REPLY_MS : JOYCON_SLOW_MS);

		id = joycon_init_steps[joycon->init_step].id;
		if (id == JOYCON_CMD_HANDSHAKE) {
			/* Unsent means no controller took it: CTS holds TX off. */
			serdev_device_write_flush(joycon->serdev);
			memcpy(buf, joycon_wake, sizeof(joycon_wake));
			memcpy(buf + sizeof(joycon_wake), joycon_handshake,
			       sizeof(joycon_handshake));
			return sizeof(joycon_wake) + sizeof(joycon_handshake);
		}
		if (id == JOYCON_WIRED_SET_HID_RATE)
			return joycon_frame_craft(buf, JOYCON_CMD_WIRED, id,
						  hid_rate, sizeof(hid_rate));
		return joycon_frame_craft(buf, JOYCON_CMD_WIRED, id, NULL, 0);
	}

	len = joycon_frame_craft(buf, JOYCON_CMD_HID, 0,
				 (u8[]){ JOYCON_OUT_POLL }, 1);

	/* Vibration lasts only as long as the packets keep coming. */
	scoped_guard(spinlock_irqsave, &joycon->rumble_lock) {
		send_rumble = joycon->rumble_on || joycon->rumble_zero_left;
		if (send_rumble && !joycon->rumble_on)
			joycon->rumble_zero_left--;
		memcpy(&rumble[1], joycon->rumble_data, sizeof(joycon->rumble_data));
	}
	if (send_rumble)
		len += joycon_hid_craft(buf + len, JOYCON_OUT_RUMBLE, rumble,
					sizeof(rumble));

	/* Unanswered: dropped, and its step comes round again. A pairing waits. */
	if (joycon->pending && joycon->pending != joycon->pair_subcmd &&
	    time_after(now, joycon->pending_deadline))
		joycon->pending = 0;

	/* One subcommand a poll, and one in flight: a controller drops others. */
	if (joycon->pending)
		return len;

	tx->req = joycon->req_send;
	tx->req_gen = joycon->req_gen;
	tx->subcmd = joycon_next_subcmd(joycon, args, &args_len);
	if (!tx->subcmd)
		return len;

	joycon->pending = tx->subcmd;
	joycon->pending_deadline =
		now + msecs_to_jiffies(JOYCON_SUBCMD_TIMEOUT_MS);

	return len + joycon_subcmd_craft(buf + len, tx->subcmd, args, args_len);
}

/* A short write leaves the controller to resynchronise on the next magic. */
static void joycon_tx_failed(struct joycon *joycon, const struct joycon_tx *tx,
			     int err)
{
	dev_err_ratelimited(joycon->dev, "write failed %d\n", err);

	if (!tx->subcmd)
		return;

	if (joycon->pending == tx->subcmd)
		joycon->pending = 0;
	if (tx->req && tx->req_gen == joycon->req_gen)
		joycon_req_finish(joycon, err);
}

static void joycon_work(struct work_struct *work)
{
	struct joycon *joycon = container_of(to_delayed_work(work),
					     struct joycon, work);
	const struct joycon_type_info *info = NULL;
	struct joycon_devs devs = { };
	struct joycon_tx tx = { };
	u8 buf[JOYCON_TX_SIZE];
	size_t len;
	int ret;

	scoped_guard(mutex, &joycon->lock) {
		joycon_slow_tick(joycon);
		if (joycon->gone)
			joycon_detach(joycon, &devs);
		else if (joycon->initialized && joycon->info && !joycon->input_dev)
			info = joycon->info;
	}

	joycon_unregister(joycon, &devs);
	if (info)
		joycon_register(joycon, info);

	scoped_guard(mutex, &joycon->lock)
		len = joycon_tx_compose(joycon, buf, &tx);

	if (len) {
		ret = serdev_device_write(joycon->serdev, buf, len,
					  msecs_to_jiffies(JOYCON_SLOW_MS));
		scoped_guard(mutex, &joycon->lock) {
			if (ret != len)
				joycon_tx_failed(joycon, &tx, ret < 0 ? ret : -EIO);
		}
	}

	guard(mutex)(&joycon->lock);
	if (joycon->running)
		queue_delayed_work(joycon->wq, &joycon->work,
				   msecs_to_jiffies(JOYCON_POLL_MS));
}

/* Which pair is live follows from the half; Y is negated, like hid-nintendo. */
static void joycon_report_sticks(struct joycon *joycon, const u8 *stick)
{
	struct input_dev *dev = joycon->input_dev;
	s32 raw_x, raw_y;

	if (joycon->info->left) {
		joycon_unpack_stick_pair(stick, &raw_x, &raw_y);
		input_report_abs(dev, ABS_X,
				 joycon_map_stick_val(&joycon->left_cal_x, raw_x));
		input_report_abs(dev, ABS_Y,
				 -joycon_map_stick_val(&joycon->left_cal_y, raw_y));
	} else {
		joycon_unpack_stick_pair(stick + 3, &raw_x, &raw_y);
		input_report_abs(dev, ABS_RX,
				 joycon_map_stick_val(&joycon->right_cal_x, raw_x));
		input_report_abs(dev, ABS_RY,
				 -joycon_map_stick_val(&joycon->right_cal_y, raw_y));
	}
}

/* Adopt a decoded pair if it is usable; the defaults stay otherwise. */
static void joycon_apply_stick_cal(struct joycon *joycon, const u8 *packed,
				   bool left, const char *what)
{
	struct joycon_stick_cal cal_x, cal_y;

	joycon_parse_stick_cal(&cal_x, &cal_y, packed, left);
	if (!joycon_stick_cal_valid(&cal_x, &cal_y)) {
		dev_warn(joycon->dev, "unusable %s stick calibration\n", what);
		return;
	}

	if (left) {
		joycon->left_cal_x = cal_x;
		joycon->left_cal_y = cal_y;
	} else {
		joycon->right_cal_x = cal_x;
		joycon->right_cal_y = cal_y;
	}
	dev_dbg(joycon->dev, "%s stick calibration\n", what);
}

/* What each step may answer, and the step that follows; 0 once saved. */
static const struct {
	u8 type;
	u8 reached;
	u8 next;
} joycon_pair_steps[] = {
	/* A host the controller does not already hold has to be walked on. */
	{ JOYCON_PAIR_REQ_HOST, JOYCON_PAIR_REQ_ADDR, JOYCON_PAIR_REQ_LTK },
	{ JOYCON_PAIR_REQ_HOST, JOYCON_PAIR_REQ_SAVE, 0 },
	{ JOYCON_PAIR_REQ_LTK, JOYCON_PAIR_REQ_LTK, JOYCON_PAIR_REQ_SAVE },
	{ JOYCON_PAIR_REQ_SAVE, JOYCON_PAIR_REQ_SAVE, 0 },
};

/* A step goes out only once the one before it has been answered. */
static void joycon_pair_reply(struct joycon *joycon, u8 subcmd,
			      const u8 *data, size_t len)
{
	int i;

	if (joycon->req != JOYCON_REQ_PAIR || subcmd != joycon->pair_subcmd)
		return;

	if (subcmd == JOYCON_HID_SET_SHIPMENT) {
		joycon->pair_subcmd = JOYCON_HID_MANUAL_PAIRING;
		joycon->pair_type = JOYCON_PAIR_REQ_HOST;
		joycon->req_send = true;
		return;
	}

	/* The reply names the step reached; what follows it can be the key. */
	for (i = 0; len && i < ARRAY_SIZE(joycon_pair_steps); i++)
		if (joycon_pair_steps[i].type == joycon->pair_type &&
		    joycon_pair_steps[i].reached == data[0])
			break;

	if (!len || i == ARRAY_SIZE(joycon_pair_steps)) {
		dev_err(joycon->dev, "pairing step %#04x answered %#04x\n",
			joycon->pair_type, len ? data[0] : 0);
		joycon_req_finish(joycon, -EIO);
		return;
	}

	dev_dbg(joycon->dev, "pairing step %#04x reached %#04x\n",
		joycon->pair_type, data[0]);

	if (joycon_pair_steps[i].next) {
		joycon->pair_type = joycon_pair_steps[i].next;
		joycon->req_send = true;
		return;
	}

	/* Saved: done once the record read back names the host. */
	joycon->pair_subcmd = 0;
	joycon_pairing_forget(joycon);
}

/* The record read back after a save is what the pairing is judged on. */
static void joycon_pair_verified(struct joycon *joycon)
{
	if (!joycon->pairing_valid ||
	    memcmp(joycon->pairing_host, joycon->pair_host,
		   sizeof(joycon->pair_host))) {
		dev_err(joycon->dev, "pairing to %pM not saved\n",
			joycon->pair_host);
		joycon_req_finish(joycon, -EIO);
		return;
	}

	dev_info(joycon->dev, "paired to %pM over the rail\n",
		 joycon->pair_host);
	joycon_req_finish(joycon, 0);
}

/* An unused slot has no magic; step on until one is live or all were read. */
static void joycon_parse_pairing(struct joycon *joycon, const u8 *data)
{
	/* Read before a pairing started, so it may describe the old record. */
	if (joycon->req == JOYCON_REQ_PAIR && joycon->pair_subcmd)
		return;

	if (data[0] != JOYCON_PAIRING_MAGIC ||
	    data[1] != JOYCON_PAIRING_BODY_SIZE) {
		joycon->pairing_slot = (joycon->pairing_slot + 1) %
				       JOYCON_PAIRING_SLOTS;
		if (++joycon->pairing_reads == JOYCON_PAIRING_SLOTS) {
			joycon->cal_parsed[JOYCON_CAL_PAIRING] = true;
			dev_dbg(joycon->dev, "not paired to any host\n");
			if (joycon->req == JOYCON_REQ_PAIR)
				joycon_pair_verified(joycon);
		}
		return;
	}

	memcpy(joycon->pairing_host, data + JOYCON_PAIRING_HOST_OFF,
	       sizeof(joycon->pairing_host));

	/* Stored back to front; turn it around for readers. */
	joycon_memcpy_rev(joycon->pairing_ltk, data + JOYCON_PAIRING_LTK_OFF,
			  JOYCON_PAIRING_LTK_SIZE);

	joycon->pairing_valid = true;
	joycon->cal_parsed[JOYCON_CAL_PAIRING] = true;

	dev_dbg(joycon->dev, "paired to %pM (slot %u)\n",
		joycon->pairing_host, joycon->pairing_slot);

	if (joycon->req == JOYCON_REQ_PAIR)
		joycon_pair_verified(joycon);
}

/* A reply that is not the read asked for, or is short, leaves it to be resent. */
static void joycon_spi_reply(struct joycon *joycon, const u8 *reply, size_t len)
{
	enum joycon_cal_step step = joycon->pending_cal;
	const u8 *data = reply + JOYCON_SPI_HDR_SIZE;
	u8 want = joycon_cal_reads[step].len;
	bool left = joycon->info->left;

	if (len < JOYCON_SPI_HDR_SIZE + want ||
	    get_unaligned_le32(reply) != joycon_cal_addr(joycon, step) ||
	    reply[4] != want) {
		dev_dbg(joycon->dev, "unexpected spi reply %*ph\n",
			(int)min_t(size_t, len, JOYCON_SPI_HDR_SIZE), reply);
		return;
	}

	switch (step) {
	case JOYCON_CAL_STICK_FCT:
		joycon_apply_stick_cal(joycon, left ? data :
				       data + JOYCON_CAL_FCT_STRIDE,
				       left, "factory");
		break;
	case JOYCON_CAL_STICK_USR:
		/* Absent on an untrimmed stick; keep the factory numbers. */
		if (!left)
			data += JOYCON_CAL_USR_STRIDE;
		if (data[0] == JC_CAL_USR_MAGIC_0 && data[1] == JC_CAL_USR_MAGIC_1)
			joycon_apply_stick_cal(joycon, data + JC_CAL_USR_MAGIC_SIZE,
					       left, "user");
		break;
	case JOYCON_CAL_IMU_FCT:
		joycon_parse_imu_cal(&joycon->accel_cal, &joycon->gyro_cal, data);
		dev_dbg(joycon->dev, "imu factory calibration\n");
		break;
	case JOYCON_CAL_IMU_USR:
		/* Absent unless the user has trimmed it; keep the factory numbers. */
		if (data[0] == JC_CAL_USR_MAGIC_0 && data[1] == JC_CAL_USR_MAGIC_1) {
			joycon_parse_imu_cal(&joycon->accel_cal, &joycon->gyro_cal,
					     data + JC_CAL_USR_MAGIC_SIZE);
			dev_dbg(joycon->dev, "imu user calibration\n");
		}
		break;
	case JOYCON_CAL_PAIRING:
		/* Marks the step done itself, on a live slot or the last one. */
		joycon_parse_pairing(joycon, data);
		return;
	default:
		return;
	}

	joycon->cal_parsed[step] = true;
}

/* Only the reply to the subcommand in flight; a refusal leaves it to be resent. */
static void joycon_subcmd_reply(struct joycon *joycon, u8 ack, u8 subcmd,
				const u8 *data, size_t len)
{
	if (!subcmd || subcmd != joycon->pending) {
		dev_dbg(joycon->dev, "stale reply to %#04x\n", subcmd);
		return;
	}
	joycon->pending = 0;

	if (!(ack & JOYCON_SUBCMD_ACK)) {
		dev_dbg(joycon->dev, "subcommand %#04x refused\n", subcmd);
		if (subcmd == joycon->pair_subcmd)
			joycon_req_finish(joycon, -EIO);
		return;
	}

	switch (subcmd) {
	case JOYCON_HID_MANUAL_PAIRING:
	case JOYCON_HID_SET_SHIPMENT:
		joycon_pair_reply(joycon, subcmd, data, len);
		break;
	case JOYCON_HID_DEVICE_INFO:
		/* The firmware version, then the controller type. */
		if (len < 3)
			break;
		joycon->info = joycon_type(data[2]);
		if (!joycon->info) {
			dev_warn(joycon->dev, "unsupported controller type %#04x\n",
				 data[2]);
			joycon->unsupported = true;
			break;
		}
		dev_dbg(joycon->dev, "type %u, firmware %X.%02X\n",
			data[2], data[0], data[1]);
		break;
	case JOYCON_HID_SET_REPORT_MODE:
		joycon->full_mode = true;
		break;
	case JOYCON_HID_ENABLE_VIBRATION:
		joycon->vib_enabled = true;
		break;
	case JOYCON_HID_ENABLE_IMU:
		joycon->imu_enabled = true;
		break;
	case JOYCON_HID_SET_CHARGE:
		joycon->charge_acked = true;
		break;
	case JOYCON_HID_GET_REGVOLT:
		if (len < 2)
			break;
		/* The first reading is news; later ones drift by millivolts. */
		if (!joycon->regvolt_mv && joycon->batt_dev)
			power_supply_changed(joycon->batt_dev);
		/* The regulated battery voltage is the raw value * 2.5 */
		joycon->regvolt_mv = get_unaligned_le16(data) * 5 / 2;
		break;
	case JOYCON_HID_SPI_READ:
		joycon_spi_reply(joycon, data, len);
		break;
	}
}

/*
 * One report holds three samples the controller took over the interval since
 * the last one, so they are timestamped a third of that interval apart.
 */
static void joycon_report_imu(struct joycon *joycon, const u8 *raw)
{
	bool right = !joycon->info->left;
	unsigned int msecs = jiffies_to_msecs(jiffies);
	unsigned int delta = msecs - joycon->imu_last_pkt_ms;
	int i;

	/* The first report of a controller has nothing to measure against. */
	if (!joycon->imu_last_pkt_ms || delta > JOYCON_IMU_MAX_DELTA_MS)
		delta = JOYCON_IMU_DFLT_DELTA_MS;
	joycon->imu_last_pkt_ms = msecs;

	for (i = 0; i < JOYCON_IMU_SAMPLES; i++) {
		struct joycon_imu_data data;

		joycon_parse_imu_data(raw + i * sizeof(data), &data);
		joycon_report_imu_sample(joycon->imu_dev, &joycon->accel_cal,
					 &joycon->gyro_cal, &data, right,
					 joycon->imu_timestamp_us);

		joycon->imu_timestamp_us += delta * 1000 / JOYCON_IMU_SAMPLES;
	}
}

static void joycon_report_input(struct joycon *joycon, const u8 *report,
				size_t size)
{
	const struct joycon_ctlr_button_mapping *btn;
	struct input_dev *idev = joycon->input_dev;
	u32 raw;

	if (!idev)
		return;

	raw = report[3] | report[4] << 8 | report[5] << 16;

	for (btn = joycon->info->btn_map; btn->code; btn++)
		input_report_key(idev, btn->code, !!(raw & btn->bit));
	for (btn = joycon->info->btn_map_s; btn->code; btn++)
		input_report_key(idev, btn->code, !!(raw & btn->bit));

	joycon_report_sticks(joycon, report + 6);

	input_sync(idev);

	/* Only a report that carries them, and only once the IMU is enabled. */
	if (joycon->imu_dev && joycon->imu_enabled &&
	    size >= JOYCON_IMU_REPORT_SIZE)
		joycon_report_imu(joycon, report + JOYCON_IMU_OFF);
}

/* Every report carries it; notify on what a consumer can see change. */
static void joycon_parse_bat_con(struct joycon *joycon, u8 bat_con)
{
	u8 mask = JOYCON_BAT_LEVEL | JOYCON_BAT_CHARGING;
	bool changed = !joycon->bat_valid ||
		       ((joycon->bat_con ^ bat_con) & mask);

	joycon->bat_con = bat_con;
	joycon->bat_valid = true;

	if (changed && joycon->batt_dev)
		power_supply_changed(joycon->batt_dev);
}

/* A report from a session that is not open yet is left alone. */
static void joycon_hid_report(struct joycon *joycon, const u8 *report,
			      size_t size)
{
	if (!joycon->initialized || !size)
		return;

	switch (report[0]) {
	case JOYCON_IN_FULL:
		/* Buttons and both sticks span the first twelve bytes. */
		if (size < 12) {
			dev_dbg(joycon->dev, "short input report of %zu bytes\n", size);
			return;
		}

		joycon->last_rx = jiffies;
		joycon_parse_bat_con(joycon, report[2]);
		joycon_report_input(joycon, report, size);
		break;
	case JOYCON_IN_SUBCMD_REPLY:
		/* The acknowledgment and the echoed subcommand follow the input. */
		if (size < 15) {
			dev_dbg(joycon->dev, "short hid reply of %zu bytes\n", size);
			return;
		}

		joycon->last_rx = jiffies;
		joycon_parse_bat_con(joycon, report[2]);
		joycon_subcmd_reply(joycon, report[13], report[14], report + 15,
				    size - 15);
		break;
	default:
		dev_dbg(joycon->dev, "unhandled report %#x\n", report[0]);
		break;
	}
}

/* Restore what lives in the session; a new controller starts from scratch. */
static void joycon_init_done(struct joycon *joycon)
{
	if (joycon->resync) {
		joycon->resync = false;
		joycon->full_mode = false;
		joycon->vib_enabled = false;
		joycon->imu_enabled = false;
		joycon->imu_last_pkt_ms = 0;
		joycon->charge_acked = false;
	}

	joycon->initialized = true;
	joycon->last_rx = jiffies;
	joycon_set_charging(joycon, true);
}

static void joycon_init_reply(struct joycon *joycon, u8 id, u8 status,
			      const u8 *payload, size_t len)
{
	u8 mac[sizeof(joycon->mac)];

	if (joycon->initialized ||
	    id != joycon_init_steps[joycon->init_step].id) {
		dev_dbg(joycon->dev, "stale init reply %#04x\n", id);
		return;
	}

	if (joycon_init_steps[joycon->init_step].status_ok && status) {
		dev_dbg(joycon->dev, "init step %#04x status %#04x\n", id, status);
		return;
	}

	if (id == JOYCON_WIRED_GET_INFO) {
		/* Payload byte 0 is the type; 1 to 6 the address, back to front. */
		if (len < 1 + sizeof(mac))
			return;
		joycon_memcpy_rev(mac, payload + 1, sizeof(mac));

		/* Swapped while asleep: take the old one away first. */
		if (joycon->resync && memcmp(mac, joycon->mac, sizeof(mac))) {
			joycon->gone = true;
			return;
		}
		memcpy(joycon->mac, mac, sizeof(mac));
		dev_dbg(joycon->dev, "controller %pM\n", joycon->mac);
	}

	joycon->init_sent = false;
	if (++joycon->init_step == ARRAY_SIZE(joycon_init_steps))
		joycon_init_done(joycon);
}

static void joycon_frame_parse(struct joycon *joycon, const u8 *frame,
			       size_t size)
{
	const struct joycon_frame_header *hdr =
		(const struct joycon_frame_header *)frame;

	if (size < sizeof(*hdr)) {
		dev_dbg(joycon->dev, "runt frame of %zu bytes\n", size);
		return;
	}

	if (!joycon_frame_valid(frame, size)) {
		dev_dbg(joycon->dev, "corrupt frame %*ph\n", (int)sizeof(*hdr),
			frame);
		return;
	}

	guard(mutex)(&joycon->lock);

	switch (hdr->cmd) {
	case JOYCON_CMD_HID:
		joycon_hid_report(joycon, frame + sizeof(*hdr),
				  size - sizeof(*hdr));
		break;
	case JOYCON_CMD_INIT_REPLY:
		joycon_init_reply(joycon, hdr->subcmd, hdr->status,
				  frame + sizeof(*hdr), size - sizeof(*hdr));
		break;
	case JOYCON_CMD_HANDSHAKE:
		joycon_init_reply(joycon, hdr->cmd, 0, NULL, 0);
		break;
	default:
		dev_dbg(joycon->dev, "unhandled command %#x\n", hdr->cmd);
		break;
	}
}

/* A magic split across the end counts as a start, and is kept for next time. */
static size_t joycon_find_magic(const u8 *buf, size_t len)
{
	size_t i;

	for (i = 0; i + sizeof(joycon_magic_recv) <= len; i++)
		if (!memcmp(buf + i, joycon_magic_recv, sizeof(joycon_magic_recv)))
			return i;

	for (; i < len; i++)
		if (!memcmp(buf + i, joycon_magic_recv, len - i))
			return i;

	return len;
}

/* Dispatch every complete frame held in the receive buffer, then compact it. */
static void joycon_rx_process(struct joycon *joycon)
{
	size_t pos = 0;

	while (pos < joycon->rx_len) {
		const u8 *frame = joycon->rx_buf + pos;
		size_t avail = joycon->rx_len - pos;
		size_t skip, frame_len;

		skip = joycon_find_magic(frame, avail);
		if (skip) {
			dev_dbg(joycon->dev, "resync: dropped %zu bytes\n", skip);
			pos += skip;
			continue;
		}

		if (avail < offsetofend(struct joycon_frame_header, total_size))
			break;		/* size field not here yet */

		frame_len = offsetofend(struct joycon_frame_header, total_size) +
			    get_unaligned_le16(frame +
				offsetof(struct joycon_frame_header, total_size));

		/* The magic was payload; resume at the next byte. */
		if (frame_len > sizeof(joycon->rx_buf)) {
			dev_dbg(joycon->dev, "resync: bogus frame length %zu\n",
				frame_len);
			pos++;
			continue;
		}

		if (avail < frame_len)
			break;		/* rest of the frame is still in flight */

		joycon_frame_parse(joycon, frame, frame_len);
		pos += frame_len;
	}

	joycon->rx_len -= pos;
	memmove(joycon->rx_buf, joycon->rx_buf + pos, joycon->rx_len);
}

static size_t joycon_serdev_receive_buf(struct serdev_device *serdev,
					const u8 *buf, size_t len)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	size_t off = 0;

	/* Not aligned to frames: accumulate, and let rx_process() cut them. */
	while (off < len) {
		size_t chunk;

		/* Holds no frame we will ever complete: drop it to resync. */
		if (joycon->rx_len == sizeof(joycon->rx_buf)) {
			dev_dbg(joycon->dev, "resync: receive buffer full\n");
			joycon->rx_len = 0;
		}

		chunk = min(len - off, sizeof(joycon->rx_buf) - joycon->rx_len);
		memcpy(joycon->rx_buf + joycon->rx_len, buf + off, chunk);
		joycon->rx_len += chunk;
		off += chunk;

		joycon_rx_process(joycon);
	}

	return len;
}

static const struct serdev_device_ops joycon_ops = {
	.receive_buf = joycon_serdev_receive_buf,
	/* serdev_device_write() blocks for its whole timeout without this. */
	.write_wakeup = serdev_device_write_wakeup,
};

static int joycon_serdev_probe(struct serdev_device *serdev)
{
	struct joycon *joycon;
	unsigned int speed;
	int err;

	joycon = devm_kzalloc(&serdev->dev, sizeof(*joycon), GFP_KERNEL);
	if (!joycon)
		return -ENOMEM;

	joycon->serdev = serdev;
	joycon->dev = &serdev->dev;
	mutex_init(&joycon->lock);
	mutex_init(&joycon->req_mutex);
	spin_lock_init(&joycon->rumble_lock);
	init_completion(&joycon->req_done);
	INIT_DELAYED_WORK(&joycon->work, joycon_work);
	joycon_reset_ctlr_state(joycon);

	joycon->wq = devm_alloc_ordered_workqueue(&serdev->dev, "joycon-%s", 0,
						  dev_name(&serdev->dev));
	if (!joycon->wq)
		return -ENOMEM;

	/* devm for the name; the core still reads it during the unregister. */
	joycon->batt_desc.name = devm_kasprintf(&serdev->dev, GFP_KERNEL,
						"joycon_battery_%s", dev_name(&serdev->dev));
	if (!joycon->batt_desc.name)
		return -ENOMEM;

	joycon->batt_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	joycon->batt_desc.properties = joycon_battery_props;
	joycon->batt_desc.num_properties = ARRAY_SIZE(joycon_battery_props);
	joycon->batt_desc.get_property = joycon_battery_get_property;
	joycon->batt_desc.set_property = joycon_battery_set_property;
	joycon->batt_desc.property_is_writeable = joycon_battery_property_is_writeable;
	joycon->batt_desc.charge_behaviours =
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE);

	joycon->charge_gpio = devm_gpiod_get(&serdev->dev, "charge", GPIOD_OUT_LOW);
	if (IS_ERR(joycon->charge_gpio))
		return dev_err_probe(&serdev->dev, PTR_ERR(joycon->charge_gpio),
				     "cannot get charge-gpio\n");

	joycon->charge_reg = devm_regulator_get(&serdev->dev, "charge");
	if (IS_ERR(joycon->charge_reg))
		return dev_err_probe(&serdev->dev, PTR_ERR(joycon->charge_reg),
				     "cannot get charge-supply\n");

	serdev_device_set_drvdata(serdev, joycon);
	serdev_device_set_client_ops(serdev, &joycon_ops);

	err = serdev_device_open(serdev);
	if (err)
		return dev_err_probe(&serdev->dev, err, "cannot open serdev\n");

	serdev_device_set_flow_control(serdev, true);
	speed = serdev_device_set_baudrate(serdev, JOYCON_BAUD);
	if (speed != JOYCON_BAUD) {
		serdev_device_close(serdev);
		return dev_err_probe(&serdev->dev, -EINVAL,
				     "cannot set %u baud, got %u\n",
				     JOYCON_BAUD, speed);
	}

	scoped_guard(mutex, &joycon->lock) {
		/* 0 can be ahead: jiffies start below it on 32-bit. */
		joycon->next_slow = jiffies;
		joycon->running = true;
	}
	queue_delayed_work(joycon->wq, &joycon->work, 0);

	return 0;
}

static void joycon_serdev_remove(struct serdev_device *serdev)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	struct joycon_devs devs = { };

	scoped_guard(mutex, &joycon->lock)
		joycon->running = false;
	cancel_delayed_work_sync(&joycon->work);

	/* No more receive_buf(), so nothing else can reach joycon from here. */
	serdev_device_close(serdev);

	scoped_guard(mutex, &joycon->lock) {
		joycon_detach(joycon, &devs);
		/* No tick to retry it: the enable leaks, and the put warns. */
		if (joycon->charge_reg_on)
			dev_err(joycon->dev, "charge supply left enabled\n");
	}
	joycon_unregister(joycon, &devs);

	/* joycon itself is devm memory: devres frees it after this returns. */
}

/* Unanswered polls would read as an empty rail; no request is in flight. */
static int joycon_suspend(struct device *dev)
{
	struct joycon *joycon = dev_get_drvdata(dev);

	scoped_guard(mutex, &joycon->lock)
		joycon->running = false;
	cancel_delayed_work_sync(&joycon->work);

	return 0;
}

/* A controller pulled or swapped meanwhile still goes the usual way. */
static int joycon_resume(struct device *dev)
{
	struct joycon *joycon = dev_get_drvdata(dev);

	scoped_guard(mutex, &joycon->lock) {
		/* The controller drops its rail session while the host is silent. */
		if (joycon->initialized || joycon->resync)
			joycon_resync(joycon, JOYCON_RESYNC_MS);
		joycon->init_step = 0;
		joycon->init_sent = false;
		joycon->pending = 0;
		joycon->running = true;
	}
	queue_delayed_work(joycon->wq, &joycon->work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(joycon_pm_ops, joycon_suspend, joycon_resume);

static const struct of_device_id joycon_uart_of_match[] = {
	{ .compatible = "nintendo,joycon-uart" },
	{ },
};
MODULE_DEVICE_TABLE(of, joycon_uart_of_match);

/* Post a pairing to @host to the work and wait for its outcome. */
static int joycon_request(struct joycon *joycon, const u8 *host)
{
	long ret;
	int err;

	if (!mutex_trylock(&joycon->req_mutex))
		return -EBUSY;

	scoped_guard(mutex, &joycon->lock) {
		/* Sent once the session is open again. */
		if (!joycon->initialized && !joycon->resync) {
			mutex_unlock(&joycon->req_mutex);
			return -ENODEV;
		}

		reinit_completion(&joycon->req_done);
		joycon->req = JOYCON_REQ_PAIR;
		joycon->req_gen++;
		joycon->req_send = true;
		joycon->req_err = 0;
		memcpy(joycon->pair_host, host, sizeof(joycon->pair_host));
		joycon->pair_subcmd = JOYCON_HID_SET_SHIPMENT;
		/* Unknown from the first step on, until read back. */
		joycon_pairing_forget(joycon);
	}

	ret = wait_for_completion_killable_timeout(&joycon->req_done,
						   msecs_to_jiffies(JOYCON_REQ_TIMEOUT_MS));

	scoped_guard(mutex, &joycon->lock) {
		joycon_req_finish(joycon, ret < 0 ? ret : -ETIMEDOUT);
		err = joycon->req_err;
	}

	mutex_unlock(&joycon->req_mutex);

	return err;
}

/* Whether the record read describes the controller now on the rail. */
static bool joycon_pairing_known(const struct joycon *joycon)
{
	return joycon->initialized && joycon->pairing_valid;
}

/* -ENODATA until the controller answers, and for one never paired. */
static ssize_t pairing_host_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct joycon *joycon = dev_get_drvdata(dev);
	u8 host[sizeof(joycon->pairing_host)];

	scoped_guard(mutex, &joycon->lock) {
		if (!joycon_pairing_known(joycon))
			return -ENODATA;
		memcpy(host, joycon->pairing_host, sizeof(host));
	}

	return sysfs_emit(buf, "%pM\n", host);
}

/* A controller holds one host, so this replaces the console it came from. */
static ssize_t pairing_host_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct joycon *joycon = dev_get_drvdata(dev);
	u8 addr[6];
	int err;

	if (!mac_pton(buf, addr))
		return -EINVAL;

	err = joycon_request(joycon, addr);

	return err ?: count;
}
static DEVICE_ATTR_RW(pairing_host);

/* The host and its key from one record: a secret, so root only. */
static ssize_t pairing_record_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct joycon *joycon = dev_get_drvdata(dev);
	u8 host[sizeof(joycon->pairing_host)];
	u8 ltk[JOYCON_PAIRING_LTK_SIZE];
	ssize_t len;

	scoped_guard(mutex, &joycon->lock) {
		if (!joycon_pairing_known(joycon))
			return -ENODATA;
		memcpy(host, joycon->pairing_host, sizeof(host));
		memcpy(ltk, joycon->pairing_ltk, sizeof(ltk));
	}

	len = sysfs_emit(buf, "%pM %*phN\n", host, (int)sizeof(ltk), ltk);
	memzero_explicit(ltk, sizeof(ltk));

	return len;
}
static DEVICE_ATTR_ADMIN_RO(pairing_record);

static struct attribute *joycon_attrs[] = {
	&dev_attr_pairing_host.attr,
	&dev_attr_pairing_record.attr,
	NULL,
};
ATTRIBUTE_GROUPS(joycon);

static struct serdev_device_driver joycon_serdev_driver = {
	.probe = joycon_serdev_probe,
	.remove = joycon_serdev_remove,
	.driver = {
		.name = "joycon-uart",
		.of_match_table = joycon_uart_of_match,
		.pm = pm_sleep_ptr(&joycon_pm_ops),
		.dev_groups = joycon_groups,
	},
};

static int __init joycon_init(void)
{
	/* Before the driver, which can be probed the moment it registers. */
	crc8_populate_msb(joycon_crc_table, JOYCON_CRC8_POLY);

	return serdev_device_driver_register(&joycon_serdev_driver);
}

static void __exit joycon_exit(void)
{
	serdev_device_driver_unregister(&joycon_serdev_driver);
}

module_init(joycon_init);
module_exit(joycon_exit);

MODULE_AUTHOR("Max Thomas <mtinc2@gmail.com>");
MODULE_DESCRIPTION("Nintendo Joy-Con serial gamepad driver");
MODULE_LICENSE("GPL");
