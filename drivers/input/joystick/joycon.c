// SPDX-License-Identifier: GPL-2.0+
/*
 * Nintendo Joy-Con serial gamepad driver
 *
 * Copyright (c) 2018 Max Thomas
 */

#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/hex.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/crc8.h>
#include <linux/power_supply.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/nintendo-joycon.h>

/*
 * Sent every poll to ask for an input report. A well formed extended command
 * frame, except that the byte at offset 10 is 0x69 where a crafted one would
 * carry 0; it is covered by the CRC, so it is replayed verbatim.
 */
static const u8 joycon_poll_input[0xD] = {
	0x19, 0x01, 0x03, 0x08, 0x00, 0x92, 0x00, 0x01, 0x00, 0x00, 0x69, 0x2D, 0x1F
};

/* The last step of the handshake, replayed verbatim for the same reason. */
static const u8 joycon_init_unk3_cmd[0x10] = {
	0x19, 0x01, 0x03, 0x0B, 0x00, 0x91, 0x12, 0x04,
	0x00, 0x00, 0x12, 0xA6, 0x0F, 0x00, 0x00, 0x00
};

#define JOYCON_COMMAND_EXTSEND   (0x91)
#define JOYCON_COMMAND_EXTRET    (0x92)
#define JOYCON_COMMAND_INITRET   (0x94)
#define JOYCON_COMMAND_HANDSHAKE (0xA5)

#define JOYCON_INIT_MAC      (0x1)
#define JOYCON_INIT_UNK1     (0x11)
#define JOYCON_INIT_UNK2     (0x10)
#define JOYCON_INIT_UNK3     (0x12)

#define JOYCON_EXT_INPUT      (0x30)
#define JOYCON_EXT_HIDCOMMAND (0x21)

/* Output report ids: the rumble-only one carries no subcommand. */
#define JOYCON_OUT_SUBCOMMAND (0x1)
#define JOYCON_OUT_RUMBLE     (0x10)

#define JOYCON_HID_MANUAL_PAIRING (0x1)
#define JOYCON_HID_DEVICE_INFO (0x2)
#define JOYCON_HID_SET_HCI_STATE (0x6)
#define JOYCON_HID_SET_SHIPMENT (0x8)
#define JOYCON_HID_SPI_READ    (0x10)
#define JOYCON_HID_ENABLE_IMU (0x40)
#define JOYCON_HID_ENABLE_VIBRATION (0x48)
#define JOYCON_HID_GET_REGVOLT (0x50)
#define JOYCON_HID_SET_CHARGE (0x51)

/* Charge switches: bit 2 arms the 100mA path, bit 4 the 200mA one. */
#define JOYCON_CHARGE_200MA (0x14)

/* Battery and connection byte of an input report. */
#define JOYCON_BAT_CHARGING BIT(4)

/* Six 12-bit values in nine bytes; 0xFF for the stick a half does not have. */
#define JOYCON_CAL_FCT_STRIDE \
	(JC_CAL_FCT_DATA_RIGHT_ADDR - JC_CAL_FCT_DATA_LEFT_ADDR)
#define JOYCON_CAL_USR_STRIDE \
	(JC_CAL_USR_RIGHT_MAGIC_ADDR - JC_CAL_USR_LEFT_MAGIC_ADDR)

/* Subcmd, the 32-bit address and the length byte precede the payload. */
#define JOYCON_SPI_DATA_OFF (6)

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
#define JOYCON_PAIR_TIMEOUT_MS		(3000)

/* Enables fast connect and real sleep; sent once, just before pairing. */
#define JOYCON_SHIPMENT_CLEAR		(0x00)

/* Reboot and page the host. Not needed to pair - a record takes effect at once. */
#define JOYCON_HCI_STATE_RECONNECT	(0x01)

#define JOYCON_BATT_MV_MIN (3300)
#define JOYCON_BATT_MV_MAX (4200)

#define JOYCON_CRC8_POLY (0x8D)
#define JOYCON_CRC8_INIT (0x00)

/* Wakes a rail that has gone quiet, ahead of a fresh handshake. */
static const u8 joycon_wake[4] = {0xA1, 0xA2, 0xA3, 0xA4};

/* Unanswered polls before the rail counts as empty. */
#define JOYCON_RECONNECT_POLLS (4)

/* type, destination, group; the destination byte differs by direction. */
static const u8 joycon_magic_send[3] = {0x19, 0x01, 0x03};
static const u8 joycon_magic_recv[3] = {0x19, 0x81, 0x03};

/* An init reply carries its payload behind the header fields we do not use. */
#define JOYCON_INITRET_PAYLOAD_OFF (6)

/* Several times larger than the biggest frame a controller sends. */
#define JOYCON_RX_BUF_SIZE (512)

struct joycon_uart_initial {
	u8 magic[3];
	/* Size of everything that follows this field. */
	__le16 total_size;
} __packed;

#define JOYCON_CMD_DATA_SIZE	(5)
/* The CRC covers the high byte of the size, the command and the data. */
#define JOYCON_CRC_OFF		(offsetof(struct joycon_uart_initial, total_size) + 1)
#define JOYCON_CRC_LEN		(1 + 1 + JOYCON_CMD_DATA_SIZE)

struct joycon_uart_header {
	struct joycon_uart_initial initial;
	u8 command;
	u8 data[JOYCON_CMD_DATA_SIZE];
	u8 crc;
} __packed;

/* A subcommand payload is a fixed 48 bytes: 10 of header, 38 of arguments. */
#define JOYCON_SUBCMD_DATA_SIZE	(48)
#define JOYCON_SUBCMD_ARGS_OFF	(10)
#define JOYCON_SUBCMD_ARGS_SIZE	(JOYCON_SUBCMD_DATA_SIZE - JOYCON_SUBCMD_ARGS_OFF)

struct joycon_subcmd_packet {
	struct joycon_uart_header pre;
	u8 command;
	u8 data[JOYCON_SUBCMD_DATA_SIZE];
} __packed;

/* The SPI reads a controller is walked through, in the order they are sent. */
enum joycon_cal_step {
	JOYCON_CAL_STICK_FCT,
	JOYCON_CAL_STICK_USR,
	JOYCON_CAL_IMU_FCT,
	JOYCON_CAL_IMU_USR,
	JOYCON_CAL_PAIRING,
	JOYCON_CAL_STEPS,
};

/* Per-Joy-Con data. */

struct joycon {
	struct serdev_device *serdev;
	struct input_dev *input_dev;
	struct input_dev *imu_dev;
	struct power_supply *batt_dev;
	struct device *dev;
	struct power_supply_desc batt_desc;
	struct gpio_desc *charge_gpio;
	struct regulator *charge_reg;

	bool charge_on;
	bool charge_acked;
	bool initialized;
	bool imu_enabled;
	/* Which steps of the chain have been answered, indexed by the enum. */
	bool cal_parsed[JOYCON_CAL_STEPS];

	int timeout_samples;
	int num_samples;
	int reconnect_threshold;

	/* JOYCON_CTLR_TYPE_*, learnt from the device info reply */
	u8 ctlr_type;
	const struct joycon_ctlr_button_mapping *btn_map;
	const struct joycon_ctlr_button_mapping *btn_map_s;
	struct joycon_stick_cal left_cal_x, left_cal_y;
	struct joycon_stick_cal right_cal_x, right_cal_y;
	struct joycon_imu_cal accel_cal, gyro_cal;

	/* What we hand userspace, advanced by the interval a report covers. */
	unsigned int imu_timestamp_us;
	unsigned int imu_last_pkt_ms;

	/* Comes and goes with the controller; input_lock guards it. */
	struct mutex input_lock;
	/* Embedded rather than allocated: a rail may be re-populated forever. */
	char phys[64];
	char uniq[24];
	char imu_name[64];

	struct workqueue_struct *wq;
	/* They re-arm themselves, so remove() has to be able to cancel them. */
	struct delayed_work timeout_work;
	struct delayed_work input_work;

	u16 regvolt;
	u8 bat_con;
	u8 mac[6];

	/* Published with a release, so a sysfs reader never sees half of it. */
	u8 pairing_host[6];
	u8 pairing_ltk[JOYCON_PAIRING_LTK_SIZE];
	u8 pairing_slot;
	bool pairing_valid;

	/* Rail pairing in flight: the step awaiting a reply, 0 when idle. */
	u8 pair_step;
	u8 pair_type;
	u8 pair_host_req[6];
	int pair_err;
	struct completion pair_done;

	/* Written from the force feedback callback, which cannot sleep. */
	spinlock_t rumble_lock;
	u8 rumble_data[JC_RUMBLE_DATA_SIZE];
	bool rumble_on;
	unsigned int rumble_zero_left;

	/* Partial frame carried over between receive_buf() calls. */
	u8 rx_buf[JOYCON_RX_BUF_SIZE];
	size_t rx_len;
};

DECLARE_CRC8_TABLE(joycon_crc_table);

/* Copy @len bytes back to front, as the controller stores addresses and keys. */
static void joycon_memcpy_rev(u8 *dst, const u8 *src, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++)
		dst[i] = src[len - 1 - i];
}

static const enum power_supply_property joycon_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static int joycon_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct joycon *joycon = power_supply_get_drvdata(psy);
	u32 regvolt_scaled, total_scaled;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (joycon->bat_con & JOYCON_BAT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (joycon->charge_on)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* regvolt is 0 until a controller answers, and this is unsigned. */
		if (joycon->regvolt <= JOYCON_BATT_MV_MIN) {
			val->intval = 0;
			break;
		}
		regvolt_scaled = ((joycon->regvolt - JOYCON_BATT_MV_MIN) * 100);
		total_scaled = (JOYCON_BATT_MV_MAX - JOYCON_BATT_MV_MIN);
		val->intval = min(regvolt_scaled / total_scaled, 100u);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = joycon->regvolt * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = JOYCON_BATT_MV_MAX * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = JOYCON_BATT_MV_MIN * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Covers the command and its data, so the size has to be final first. */
static void joycon_packet_seal(struct joycon_uart_header *pre)
{
	pre->crc = crc8(joycon_crc_table, (u8 *)pre + JOYCON_CRC_OFF,
			JOYCON_CRC_LEN, JOYCON_CRC8_INIT);
}

static u16 joycon_packet_craft(u8 *out, u8 command, const u8 *data, u16 size)
{
	struct joycon_uart_header *pre = (struct joycon_uart_header *)out;

	if (WARN_ON_ONCE(size > sizeof(pre->data)))
		return 0;

	memcpy(pre->initial.magic, joycon_magic_send, sizeof(pre->initial.magic));

	pre->initial.total_size = cpu_to_le16(sizeof(*pre) - sizeof(pre->initial));
	pre->command = command;

	if (data)
		memcpy(pre->data, data, size);

	joycon_packet_seal(pre);

	return sizeof(*pre);
}

static u16 joycon_extcommand_craft(u8 *out, u8 command, const u8 *data, u16 size)
{
	struct joycon_subcmd_packet *packet = (struct joycon_subcmd_packet *)out;
	u16 tail = size + sizeof(packet->command);
	__be16 subcmd_size = cpu_to_be16(tail);
	u16 packet_size;

	if (WARN_ON_ONCE(size > sizeof(packet->data)))
		return 0;

	/* Same command byte as a reply carries; the magic marks the direction. */
	packet_size = joycon_packet_craft(out, JOYCON_COMMAND_EXTRET,
					  (u8 *)&subcmd_size, sizeof(subcmd_size));

	le16_add_cpu(&packet->pre.initial.total_size, tail);
	packet->command = command;
	if (data)
		memcpy(packet->data, data, size);
	joycon_packet_seal(&packet->pre);

	return packet_size + tail;
}

static void joycon_send_command(struct serdev_device *serdev, u8 command,
				const u8 *data, u16 size)
{
	u8 frame[sizeof(struct joycon_uart_header)] = { };

	size = joycon_packet_craft(frame, command, data, size);
	serdev_device_write(serdev, frame, size, msecs_to_jiffies(200));
}

static void joycon_send_extcommand(struct serdev_device *serdev, u8 command,
				   const u8 *data, u16 size)
{
	u8 frame[sizeof(struct joycon_subcmd_packet)] = { };

	size = joycon_extcommand_craft(frame, command, data, size);
	serdev_device_write(serdev, frame, size, msecs_to_jiffies(200));
}

static void joycon_send_hidcommand(struct serdev_device *serdev, u8 command,
				   const u8 *data, u16 size)
{
	static const u8 rumble_neutral[JC_RUMBLE_DATA_SIZE] = {
		0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40
	};
	u8 args[JOYCON_SUBCMD_DATA_SIZE] = { };

	if (WARN_ON_ONCE(size > JOYCON_SUBCMD_ARGS_SIZE))
		return;

	/* args[0] is the console's packet counter; the controller ignores ours. */
	memcpy(&args[1], rumble_neutral, sizeof(rumble_neutral));
	args[9] = command;

	if (data)
		memcpy(&args[JOYCON_SUBCMD_ARGS_OFF], data, size);

	joycon_send_extcommand(serdev, JOYCON_OUT_SUBCOMMAND, args, sizeof(args));
}

/* Power the rail; the controller's own switch is armed from the poll. */
static void joycon_set_charging(struct joycon *joycon, bool on)
{
	int err;

	if (joycon->charge_on == on)
		return;

	if (on) {
		err = regulator_enable(joycon->charge_reg);
		if (err) {
			dev_err_once(joycon->dev,
				     "cannot enable charge-supply %d\n", err);
			return;
		}

		gpiod_set_value(joycon->charge_gpio, 1);
	} else {
		gpiod_set_value(joycon->charge_gpio, 0);
		regulator_disable(joycon->charge_reg);
	}

	joycon->charge_on = on;
	joycon->charge_acked = false;
}

/* The packet counter and the rumble data, with no subcommand behind it. */
static void joycon_send_rumble(struct joycon *joycon)
{
	u8 data[1 + JC_RUMBLE_DATA_SIZE] = { 0 };

	scoped_guard(spinlock_irqsave, &joycon->rumble_lock)
		memcpy(&data[1], joycon->rumble_data, sizeof(joycon->rumble_data));

	joycon_send_extcommand(joycon->serdev, JOYCON_OUT_RUMBLE, data,
			       sizeof(data));
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
static void joycon_send_pair_request(struct joycon *joycon)
{
	u8 req[1 + 6 + 3 + JOYCON_PAIR_NAME_LEN + 2 + 6] = { JOYCON_PAIR_REQ_HOST };
	u8 *p = req + 1;

	memcpy(p, joycon->pair_host_req, sizeof(joycon->pair_host_req));
	p += sizeof(joycon->pair_host_req);

	memcpy(p, (u8[]){ 0x00, 0x04, 0x3c }, 3);
	p += 3;

	/* What the controller remembers this host as. */
	strscpy(p, "Nintendo Switch", JOYCON_PAIR_NAME_LEN);
	p += JOYCON_PAIR_NAME_LEN;

	*p++ = JOYCON_PAIR_CAPS;
	*p++ = 0x00;

	memcpy(p, (u8[]){ 0x85, 0xaf, 0xa4, 0x69, 0x2a, 0x00 }, 6);

	joycon_send_hidcommand(joycon->serdev, JOYCON_HID_MANUAL_PAIRING,
			       req, sizeof(req));
}

/* Where the slot we are currently probing lives. */
static u32 joycon_pairing_addr(const struct joycon *joycon)
{
	return JOYCON_PAIRING_ADDR +
	       joycon->pairing_slot * JOYCON_PAIRING_SLOT_SIZE;
}

/* Drop what we know of the record, so the poller walks the slots again. */
static void joycon_pairing_forget(struct joycon *joycon)
{
	joycon->cal_parsed[JOYCON_CAL_PAIRING] = false;
	joycon->pairing_valid = false;
	joycon->pairing_slot = 0;
}

/* Everything learnt from a controller, dropped when one comes or goes. */
static void joycon_reset_ctlr_state(struct joycon *joycon)
{
	joycon->ctlr_type = 0;
	joycon->btn_map = NULL;
	joycon->btn_map_s = NULL;
	joycon->rumble_on = false;
	joycon->rumble_zero_left = 0;
	joycon->imu_enabled = false;
	memset(joycon->cal_parsed, 0, sizeof(joycon->cal_parsed));
	joycon->imu_last_pkt_ms = 0;
	joycon->imu_timestamp_us = 0;
	joycon->pair_step = 0;
	joycon_pairing_forget(joycon);
}

/* SPI read subcommand: a 32-bit little endian address then a length byte. */
static void joycon_spi_read(struct joycon *joycon, u32 addr, u8 len)
{
	u8 req[5] = { addr, addr >> 8, addr >> 16, addr >> 24, len };

	joycon_send_hidcommand(joycon->serdev, JOYCON_HID_SPI_READ,
			       req, sizeof(req));
}

/* Registered on the first device-info reply, once the half is known. */

struct joycon_type_info {
	u8 type;
	const char *name;
	u16 product;
	const struct joycon_ctlr_button_mapping *btn_map;
	/* Never a grip, so SL/SR take the unused triggers. */
	const struct joycon_ctlr_button_mapping *btn_map_s;
	bool left_stick;
	bool right_stick;
};

/* The last entry is the fallback, and has to stay last. */
static const struct joycon_type_info joycon_types[] = {
	{ JOYCON_CTLR_TYPE_JCL, "Nintendo Switch Left Joy-Con (Serial)",
	  JC_PRODUCT_JOYCONL, left_joycon_button_mappings,
	  left_joycon_s_button_mappings, true, false },
	{ JOYCON_CTLR_TYPE_JCR, "Nintendo Switch Right Joy-Con (Serial)",
	  JC_PRODUCT_JOYCONR, right_joycon_button_mappings,
	  right_joycon_s_button_mappings, false, true },
	{ 0, "Nintendo Switch Pro Controller (Serial)",
	  JC_PRODUCT_PROCON, procon_button_mappings, NULL, true, true },
};

static const struct joycon_type_info *joycon_type(u8 ctlr_type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(joycon_types) - 1; i++)
		if (joycon_types[i].type == ctlr_type)
			return &joycon_types[i];

	return &joycon_types[ARRAY_SIZE(joycon_types) - 1];
}

/* Everything but the name, which differs between the two devices we make. */
static void joycon_set_dev_ids(struct input_dev *dev, struct joycon *joycon,
			       const struct joycon_type_info *info)
{
	dev->phys = joycon->phys;
	dev->uniq = joycon->uniq;
	dev->dev.parent = &joycon->serdev->dev;
	dev->id.bustype = BUS_VIRTUAL;
	dev->id.vendor = JC_VENDOR_NINTENDO;
	dev->id.product = info->product;
	dev->id.version = 0x0100;

	input_set_drvdata(dev, joycon);
}

/* A second device, as hid-nintendo does it: the axes would collide otherwise. */
static struct input_dev *joycon_imu_register(struct joycon *joycon,
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

static int joycon_input_register(struct joycon *joycon)
{
	const struct joycon_type_info *info = joycon_type(joycon->ctlr_type);
	struct serdev_device *serdev = joycon->serdev;
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	snprintf(joycon->phys, sizeof(joycon->phys), "%s/input0",
		 dev_name(&serdev->dev));
	snprintf(joycon->uniq, sizeof(joycon->uniq), "%pM", joycon->mac);

	dev->name = info->name;
	joycon_set_dev_ids(dev, joycon, info);

	joycon->btn_map = info->btn_map;
	joycon->btn_map_s = info->btn_map_s;

	if (info->left_stick)
		joycon_config_left_stick(dev);
	if (info->right_stick)
		joycon_config_right_stick(dev);

	joycon_config_buttons(dev, joycon->btn_map);
	if (joycon->btn_map_s)
		joycon_config_buttons(dev, joycon->btn_map_s);

	input_set_capability(dev, EV_FF, FF_RUMBLE);
	/* Not fatal: everything but rumble still works without it. */
	err = input_ff_create_memless(dev, NULL, joycon_play_effect);
	if (err)
		dev_err(joycon->dev, "cannot create force feedback %d\n", err);

	err = input_register_device(dev);
	if (err) {
		input_free_device(dev);
		return err;
	}

	/* Until the flash has been read; the raw samples are unusable without. */
	joycon_imu_cal_defaults(&joycon->accel_cal, &joycon->gyro_cal);

	scoped_guard(mutex, &joycon->input_lock) {
		/* Not fatal, and the receive path takes it with input_dev. */
		joycon->imu_dev = joycon_imu_register(joycon, info);
		/* release: the receive path must not see this early */
		smp_store_release(&joycon->input_dev, dev);
	}
	dev_info(joycon->dev, "%s\n", dev->name);

	return 0;
}

/* Registered with the input device; the name is built once, at probe. */
static int joycon_battery_register(struct joycon *joycon)
{
	struct power_supply_config psy_cfg = {
		.fwnode = dev_fwnode(joycon->dev),
		.drv_data = joycon,
	};

	joycon->batt_dev = power_supply_register(joycon->dev, &joycon->batt_desc,
						 &psy_cfg);
	if (IS_ERR(joycon->batt_dev)) {
		int err = PTR_ERR(joycon->batt_dev);

		joycon->batt_dev = NULL;
		return err;
	}

	return 0;
}

static void joycon_battery_unregister(struct joycon *joycon)
{
	struct power_supply *psy = joycon->batt_dev;

	joycon->batt_dev = NULL;
	if (psy)
		power_supply_unregister(psy);
}

/* The next controller on this rail may be the other half. */
static void joycon_input_unregister(struct joycon *joycon)
{
	struct input_dev *dev, *imu_dev;

	scoped_guard(mutex, &joycon->input_lock) {
		dev = joycon->input_dev;
		imu_dev = joycon->imu_dev;
		joycon->input_dev = NULL;
		joycon->imu_dev = NULL;
	}

	if (imu_dev)
		input_unregister_device(imu_dev);

	if (dev) {
		input_unregister_device(dev);
		dev_info(joycon->dev, "disconnected\n");
	}

	joycon_reset_ctlr_state(joycon);
}

static void joycon_timeout_handler(struct work_struct *work)
{
	struct joycon *joycon = container_of(to_delayed_work(work),
					     struct joycon, timeout_work);
	struct serdev_device *serdev = joycon->serdev;
	int err;

	if (joycon->num_samples - joycon->timeout_samples == 0)
		joycon->reconnect_threshold++;
	else
		joycon->reconnect_threshold = 0;

	/* The rail is empty, so take the device away. */
	if (joycon->reconnect_threshold >= JOYCON_RECONNECT_POLLS) {
		joycon->initialized = false;
		joycon->regvolt = 0;
		joycon->bat_con = 0;
		joycon_set_charging(joycon, false);

		joycon_input_unregister(joycon);
		joycon_battery_unregister(joycon);

		err = serdev_device_write(serdev, joycon_wake,
					  sizeof(joycon_wake), msecs_to_jiffies(200));
		if (err < 0)
			goto done;

		joycon_send_command(serdev, JOYCON_COMMAND_HANDSHAKE, (u8[]){0x02, 0x01, 0x7E}, 3);

		serdev_device_write_flush(serdev);

		joycon->reconnect_threshold = 0;
		dev_dbg(joycon->dev, "sent handshake\n");
	}

	if (!joycon->initialized)
		goto done;

	/* One subcommand a tick, and an unanswered one is simply lost. */
	if (joycon->charge_on && !joycon->charge_acked)
		joycon_send_hidcommand(serdev, JOYCON_HID_SET_CHARGE,
				       (u8[]){ JOYCON_CHARGE_200MA }, 1);
	else
		joycon_send_hidcommand(serdev, JOYCON_HID_GET_REGVOLT, NULL, 0);

done:
	joycon->timeout_samples = joycon->num_samples;
	queue_delayed_work(joycon->wq, &joycon->timeout_work, msecs_to_jiffies(200));
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

/* One a tick, in order: each waits for the reply to the one before it. */
static void joycon_send_next_cal_read(struct joycon *joycon)
{
	int i;

	for (i = 0; i < JOYCON_CAL_STEPS; i++) {
		if (joycon->cal_parsed[i])
			continue;

		joycon_spi_read(joycon,
				joycon_cal_reads[i].addr ?:
					joycon_pairing_addr(joycon),
				joycon_cal_reads[i].len);
		return;
	}
}

static void joycon_input_handler(struct work_struct *work)
{
	struct joycon *joycon = container_of(to_delayed_work(work),
					     struct joycon, input_work);
	bool rumble;

	if (!joycon->initialized)
		goto done;

	/* Before the calibration reads below, which need somewhere to land. */
	if (joycon->ctlr_type && !joycon->input_dev) {
		int err = joycon_input_register(joycon);

		if (err) {
			dev_err_once(joycon->dev,
				     "cannot register input device %d\n", err);
			goto done;
		}

		/* Not fatal: the controller is still perfectly usable. */
		err = joycon_battery_register(joycon);
		if (err)
			dev_err_once(joycon->dev,
				     "cannot register power supply %d\n", err);

		/* The motor ignores rumble packets until this arrives. */
		joycon_send_hidcommand(joycon->serdev,
				       JOYCON_HID_ENABLE_VIBRATION,
				       (u8[]){ 1 }, 1);
	}

	serdev_device_write(joycon->serdev, joycon_poll_input,
			    sizeof(joycon_poll_input), msecs_to_jiffies(200));

	/* Vibration lasts only as long as the packets keep coming. */
	scoped_guard(spinlock_irqsave, &joycon->rumble_lock) {
		rumble = joycon->rumble_on || joycon->rumble_zero_left;
		if (rumble && !joycon->rumble_on)
			joycon->rumble_zero_left--;
	}
	if (rumble)
		joycon_send_rumble(joycon);

	if (!joycon->ctlr_type)
		joycon_send_hidcommand(joycon->serdev, JOYCON_HID_DEVICE_INFO, NULL, 0);

	/* One subcommand a tick: a controller drops all but the first. */
	if (joycon->ctlr_type && !joycon->imu_enabled)
		joycon_send_hidcommand(joycon->serdev, JOYCON_HID_ENABLE_IMU,
				       (u8[]){ 1 }, 1);

	if (joycon->imu_enabled)
		joycon_send_next_cal_read(joycon);

done:
	queue_delayed_work(joycon->wq, &joycon->input_work, msecs_to_jiffies(16));
}

/* Which pair is live follows from the type; Y is negated, like hid-nintendo. */
static void joycon_report_sticks(struct joycon *joycon, const u8 *stick)
{
	struct input_dev *dev = joycon->input_dev;
	s32 raw_x, raw_y;

	if (joycon->ctlr_type != JOYCON_CTLR_TYPE_JCR) {
		joycon_unpack_stick_pair(stick, &raw_x, &raw_y);
		input_report_abs(dev, ABS_X,
				 joycon_map_stick_val(&joycon->left_cal_x, raw_x));
		input_report_abs(dev, ABS_Y,
				 -joycon_map_stick_val(&joycon->left_cal_y, raw_y));
	}

	if (joycon->ctlr_type != JOYCON_CTLR_TYPE_JCL) {
		joycon_unpack_stick_pair(stick + 3, &raw_x, &raw_y);
		input_report_abs(dev, ABS_RX,
				 joycon_map_stick_val(&joycon->right_cal_x, raw_x));
		input_report_abs(dev, ABS_RY,
				 -joycon_map_stick_val(&joycon->right_cal_y, raw_y));
	}
}

/* Adopt a decoded pair, if it is usable. Returns true when it was taken. */
static bool joycon_apply_stick_cal(struct joycon *joycon, const u8 *packed,
				   bool left)
{
	struct joycon_stick_cal cal_x, cal_y;

	joycon_parse_stick_cal(&cal_x, &cal_y, packed, left);
	if (!joycon_stick_cal_valid(&cal_x, &cal_y))
		return false;

	if (left) {
		joycon->left_cal_x = cal_x;
		joycon->left_cal_y = cal_y;
	} else {
		joycon->right_cal_x = cal_x;
		joycon->right_cal_y = cal_y;
	}

	return true;
}

/* A step goes out only once the one before it has been answered. */
static void joycon_pair_advance(struct joycon *joycon, u8 subcmd,
				const u8 *packet, u32 size)
{
	u8 next = 0;

	if (joycon->pair_step != subcmd || size < 2)
		return;

	if (subcmd == JOYCON_HID_SET_SHIPMENT) {
		joycon->pair_step = JOYCON_HID_MANUAL_PAIRING;
		joycon->pair_type = JOYCON_PAIR_REQ_HOST;
		joycon_send_pair_request(joycon);
		return;
	}

	dev_dbg(joycon->dev, "pair step %#04x reply %*ph\n", joycon->pair_type,
		(int)min(size, 36u), packet);

	/* A host the controller does not already hold has to be walked on. */
	if (joycon->pair_type == JOYCON_PAIR_REQ_HOST &&
	    packet[1] == JOYCON_PAIR_REQ_ADDR)
		next = JOYCON_PAIR_REQ_LTK;
	else if (joycon->pair_type == JOYCON_PAIR_REQ_LTK)
		next = JOYCON_PAIR_REQ_SAVE;

	if (next) {
		joycon->pair_type = next;
		joycon_send_hidcommand(joycon->serdev, JOYCON_HID_MANUAL_PAIRING,
				       &joycon->pair_type, 1);
		return;
	}

	if (packet[1] != JOYCON_PAIR_REQ_SAVE) {
		dev_err(joycon->dev, "pairing refused at step %#04x (%#04x)\n",
			joycon->pair_type, packet[1]);
		joycon->pair_err = -EIO;
		goto done;
	}

	/* Re-read the record rather than trust the REQ_LTK reply. */
	joycon_pairing_forget(joycon);
	joycon->pair_err = 0;

	/* Reversed: the request holds the address the way it goes on the wire. */
	dev_info(joycon->dev, "paired to %pMR over the rail\n",
		 joycon->pair_host_req);
done:
	joycon->pair_step = 0;
	complete(&joycon->pair_done);
}

/* An unused slot has no magic; step on until one is live or the array ends. */
static void joycon_parse_pairing(struct joycon *joycon, const u8 *data, u32 len)
{
	if (len < JOYCON_PAIRING_READ_LEN)
		return;

	if (data[0] != JOYCON_PAIRING_MAGIC ||
	    data[1] != JOYCON_PAIRING_BODY_SIZE) {
		if (++joycon->pairing_slot >= JOYCON_PAIRING_SLOTS) {
			joycon->cal_parsed[JOYCON_CAL_PAIRING] = true;
			dev_dbg(joycon->dev, "not paired to any host\n");
		}
		return;
	}

	memcpy(joycon->pairing_host, data + JOYCON_PAIRING_HOST_OFF,
	       sizeof(joycon->pairing_host));

	/* Stored back to front; turn it around for readers. */
	joycon_memcpy_rev(joycon->pairing_ltk, data + JOYCON_PAIRING_LTK_OFF,
			  JOYCON_PAIRING_LTK_SIZE);

	joycon->cal_parsed[JOYCON_CAL_PAIRING] = true;
	/* Pairs with the acquire in the sysfs readers. */
	smp_store_release(&joycon->pairing_valid, true);

	dev_dbg(joycon->dev, "paired to %pM (slot %u)\n",
		joycon->pairing_host, joycon->pairing_slot);
}

static void joycon_parse_spi_read(struct serdev_device *serdev, const u8 *packet, u32 size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	const u8 *data = packet + JOYCON_SPI_DATA_OFF;
	bool left = joycon->ctlr_type != JOYCON_CTLR_TYPE_JCR;
	bool right = joycon->ctlr_type != JOYCON_CTLR_TYPE_JCL;
	u32 addr, data_len;

	/* The claimed length overstates the frame, so bound by what arrived. */
	if (!joycon->input_dev || size < JOYCON_SPI_DATA_OFF)
		return;

	addr = get_unaligned_le32(packet + 1);
	data_len = size - JOYCON_SPI_DATA_OFF;

	if (addr == JC_CAL_FCT_DATA_LEFT_ADDR &&
	    data_len >= JOYCON_CAL_FCT_STRIDE + JC_CAL_STICK_DATA_SIZE) {
		if (left && joycon_apply_stick_cal(joycon, data, true))
			dev_dbg(joycon->dev, "left stick factory calibration\n");
		if (right && joycon_apply_stick_cal(joycon,
						    data + JOYCON_CAL_FCT_STRIDE, false))
			dev_dbg(joycon->dev, "right stick factory calibration\n");

		joycon->cal_parsed[JOYCON_CAL_STICK_FCT] = true;
	} else if (addr == JC_CAL_USR_LEFT_MAGIC_ADDR) {
		/* Absent on an untrimmed stick; keep the factory numbers. */
		if (data_len >= 2 * JOYCON_CAL_USR_STRIDE) {
			const u8 *usr[2] = { data, data + JOYCON_CAL_USR_STRIDE };
			bool want[2] = { left, right };
			int i;

			for (i = 0; i < 2; i++) {
				if (!want[i] ||
				    usr[i][0] != JC_CAL_USR_MAGIC_0 ||
				    usr[i][1] != JC_CAL_USR_MAGIC_1)
					continue;

				if (joycon_apply_stick_cal(joycon,
							   usr[i] + JC_CAL_USR_MAGIC_SIZE,
							   i == 0))
					dev_dbg(joycon->dev,
						"%s stick user calibration\n",
						i == 0 ? "left" : "right");
			}
		}

		joycon->cal_parsed[JOYCON_CAL_STICK_USR] = true;
	} else if (addr == JC_IMU_CAL_FCT_DATA_ADDR) {
		if (data_len >= JC_IMU_CAL_DATA_SIZE) {
			joycon_parse_imu_cal(&joycon->accel_cal,
					     &joycon->gyro_cal, data);
			dev_dbg(joycon->dev, "imu factory calibration\n");
		}

		joycon->cal_parsed[JOYCON_CAL_IMU_FCT] = true;
	} else if (addr == JC_IMU_CAL_USR_MAGIC_ADDR) {
		/* Absent unless the user has trimmed it; keep the factory numbers. */
		if (data_len >= JOYCON_IMU_CAL_USR_LEN &&
		    data[0] == JC_CAL_USR_MAGIC_0 && data[1] == JC_CAL_USR_MAGIC_1) {
			joycon_parse_imu_cal(&joycon->accel_cal, &joycon->gyro_cal,
					     data + JC_CAL_USR_MAGIC_SIZE);
			dev_dbg(joycon->dev, "imu user calibration\n");
		}

		joycon->cal_parsed[JOYCON_CAL_IMU_USR] = true;
	} else if (!joycon->cal_parsed[JOYCON_CAL_PAIRING] &&
		   addr == joycon_pairing_addr(joycon)) {
		joycon_parse_pairing(joycon, data, data_len);
	}
}

static void joycon_hidret_parse(struct serdev_device *serdev, const u8 *packet, u32 size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);

	switch (packet[0]) {
	case JOYCON_HID_MANUAL_PAIRING:
	case JOYCON_HID_SET_SHIPMENT:
		joycon_pair_advance(joycon, packet[0], packet, size);
		break;
	case JOYCON_HID_DEVICE_INFO:
		if (size < 4)
			break;
		/* The reply repeats; registering the device announces it. */
		joycon->ctlr_type = packet[3];
		dev_dbg(joycon->dev, "type %u, firmware %X.%02X\n",
			packet[3], packet[1], packet[2]);
		break;
	case JOYCON_HID_ENABLE_IMU:
		joycon->imu_enabled = true;
		break;
	case JOYCON_HID_SPI_READ:
		joycon_parse_spi_read(serdev, packet, size);
		break;
	case JOYCON_HID_SET_CHARGE:
		joycon->charge_acked = true;
		break;
	case JOYCON_HID_GET_REGVOLT:
		if (size < 3)
			break;
		/* The regulated battery voltage is the raw value * 2.5 */
		joycon->regvolt = get_unaligned_le16(packet + 1);
		joycon->regvolt *= 5;
		joycon->regvolt /= 2;
		break;
	default:
		dev_dbg(joycon->dev, "unhandled hid reply %#x\n", packet[0]);
		break;
	}
}

/*
 * One report holds three samples the controller took over the interval since
 * the last one, so they are timestamped a third of that interval apart.
 */
static void joycon_report_imu(struct joycon *joycon, const u8 *raw)
{
	bool right = joycon->ctlr_type == JOYCON_CTLR_TYPE_JCR;
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

/* input_lock covers lookup and reporting against joycon_timeout_handler(). */
static void joycon_report_input(struct joycon *joycon, const u8 *packet,
				u32 size)
{
	const struct joycon_ctlr_button_mapping *btn;
	struct input_dev *idev;
	u32 raw;

	joycon->bat_con = packet[2];

	guard(mutex)(&joycon->input_lock);

	/* Pairs with the release in joycon_input_register(). */
	idev = smp_load_acquire(&joycon->input_dev);
	if (!idev)
		return;

	raw = packet[3] | packet[4] << 8 | packet[5] << 16;

	for (btn = joycon->btn_map; btn->code; btn++)
		input_report_key(idev, btn->code, !!(raw & btn->bit));
	if (joycon->btn_map_s)
		for (btn = joycon->btn_map_s; btn->code; btn++)
			input_report_key(idev, btn->code, !!(raw & btn->bit));

	joycon_report_sticks(joycon, packet + 6);

	input_sync(idev);

	/* Only a report that carries them, and only once the IMU is enabled. */
	if (joycon->imu_dev && size >= JOYCON_IMU_REPORT_SIZE)
		joycon_report_imu(joycon, packet + JOYCON_IMU_OFF);
}

static void joycon_extret_parse(struct serdev_device *serdev, const u8 *packet, u32 size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);

	if (!size)
		return;

	switch (packet[0]) {
	case JOYCON_EXT_INPUT:
		/* Counted before any bail-out: this says the rail answers. */
		joycon->num_samples++;

		/* Buttons and both sticks span the first twelve bytes. */
		if (size < 12) {
			dev_dbg(joycon->dev, "short input report of %u bytes\n", size);
			break;
		}

		joycon_report_input(joycon, packet, size);
		break;
	case JOYCON_EXT_HIDCOMMAND:
		if (size < 15) {
			dev_dbg(joycon->dev, "short hid reply of %u bytes\n", size);
			break;
		}

		joycon_hidret_parse(serdev, packet + 14, size - 14);
		break;
	default:
		dev_dbg(joycon->dev, "unhandled ext reply %#x\n", packet[0]);
		break;
	}
}

static void joycon_initret_parse(struct serdev_device *serdev, const u8 *packet, u32 size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);

	switch (packet[0]) {
	case JOYCON_INIT_MAC:
		/* Payload byte 0 is the type; 1 to 6 the address, back to front. */
		if (size < JOYCON_INITRET_PAYLOAD_OFF + 1 + sizeof(joycon->mac))
			break;
		joycon_memcpy_rev(joycon->mac,
				  packet + JOYCON_INITRET_PAYLOAD_OFF + 1,
				  sizeof(joycon->mac));
		dev_dbg(joycon->dev, "controller %pM\n", joycon->mac);

		/* 1 megabaud carries the 15ms report rate already. */
		joycon_send_command(serdev, JOYCON_COMMAND_EXTSEND, (u8[]){JOYCON_INIT_UNK1}, 1);
		break;
	case JOYCON_INIT_UNK1:
		joycon_send_command(serdev, JOYCON_COMMAND_EXTSEND, (u8[]){JOYCON_INIT_UNK2}, 1);
		break;
	case JOYCON_INIT_UNK2:
		serdev_device_write(serdev, joycon_init_unk3_cmd,
				    sizeof(joycon_init_unk3_cmd), msecs_to_jiffies(200));
		break;
	case JOYCON_INIT_UNK3:
		joycon_reset_ctlr_state(joycon);
		joycon->num_samples = 0;
		joycon->initialized = true;

		joycon_set_charging(joycon, true);
		break;
	default:
		dev_dbg(joycon->dev, "unhandled init reply %#x\n", packet[0]);
		break;
	}
}

static void joycon_packet_parse(struct serdev_device *serdev, const u8 *packet, size_t size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	struct joycon_uart_header *header = (struct joycon_uart_header *)packet;

	if (size < sizeof(*header)) {
		dev_dbg(joycon->dev, "runt frame of %zu bytes\n", size);
		return;
	}

	switch (header->command) {
	case JOYCON_COMMAND_EXTRET:
		joycon_extret_parse(serdev, packet + sizeof(*header),
				    size - sizeof(*header));
		break;
	case JOYCON_COMMAND_INITRET:
		joycon_initret_parse(serdev,
				     packet + sizeof(header->initial) + 1,
				     size - sizeof(header->initial) - 1);
		break;
	case JOYCON_COMMAND_HANDSHAKE:
		dev_dbg(joycon->dev, "handshake accepted\n");

		joycon_send_command(serdev, JOYCON_COMMAND_EXTSEND, (u8[]){JOYCON_INIT_MAC}, 1);
		break;
	default:
		dev_dbg(joycon->dev, "unhandled command %#x\n", header->command);
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

		if (avail < sizeof(struct joycon_uart_initial))
			break;		/* size field not here yet */

		frame_len = sizeof(struct joycon_uart_initial) +
			    le16_to_cpu(((struct joycon_uart_initial *)frame)->total_size);

		/* The magic was payload; resume at the next byte. */
		if (frame_len > sizeof(joycon->rx_buf)) {
			dev_dbg(joycon->dev, "resync: bogus frame length %zu\n",
				frame_len);
			pos++;
			continue;
		}

		if (avail < frame_len)
			break;		/* rest of the frame is still in flight */

		joycon_packet_parse(joycon->serdev, frame, frame_len);
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
	int err;

	joycon = devm_kzalloc(&serdev->dev, sizeof(*joycon), GFP_KERNEL);
	if (!joycon)
		return -ENOMEM;

	joycon->serdev = serdev;
	joycon->dev = &serdev->dev;
	mutex_init(&joycon->input_lock);
	spin_lock_init(&joycon->rumble_lock);
	init_completion(&joycon->pair_done);
	INIT_DELAYED_WORK(&joycon->timeout_work, joycon_timeout_handler);
	INIT_DELAYED_WORK(&joycon->input_work, joycon_input_handler);

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

	joycon->charge_gpio = devm_gpiod_get(&serdev->dev, "charge", GPIOD_OUT_LOW);
	if (IS_ERR(joycon->charge_gpio))
		return dev_err_probe(&serdev->dev, PTR_ERR(joycon->charge_gpio),
				     "cannot get charge-gpio\n");

	joycon->charge_reg = devm_regulator_get(&serdev->dev, "charge");
	if (IS_ERR(joycon->charge_reg))
		return dev_err_probe(&serdev->dev, PTR_ERR(joycon->charge_reg),
				     "cannot get charge-supply\n");

	serdev_device_set_drvdata(serdev, joycon);

	err = serdev_device_open(serdev);
	if (err) {
		serdev_device_set_drvdata(serdev, NULL);
		return dev_err_probe(&serdev->dev, err, "cannot open serdev\n");
	}
	serdev_device_set_flow_control(serdev, true);
	serdev_device_set_baudrate(serdev, 1000000);

	serdev_device_set_client_ops(serdev, &joycon_ops);

	queue_delayed_work(joycon->wq, &joycon->timeout_work, msecs_to_jiffies(200));
	queue_delayed_work(joycon->wq, &joycon->input_work, msecs_to_jiffies(200));

	return 0;
}

static void joycon_serdev_remove(struct serdev_device *serdev)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);

	/* They re-arm, so cancel - and while the port is open. */
	cancel_delayed_work_sync(&joycon->timeout_work);
	cancel_delayed_work_sync(&joycon->input_work);

	/* No more receive_buf(), so nothing else can reach joycon from here. */
	serdev_device_close(serdev);

	joycon_set_charging(joycon, false);

	/* Not devm: both have to come and go with the controller. */
	joycon_input_unregister(joycon);
	joycon_battery_unregister(joycon);

	/* joycon itself is devm memory: devres frees it after this returns. */
}

static const struct of_device_id joycon_uart_of_match[] = {
	{ .compatible = "nintendo,joycon-uart" },
	{ },
};
MODULE_DEVICE_TABLE(of, joycon_uart_of_match);

/* -ENODATA until the controller answers, and for one never paired. */
static ssize_t pairing_host_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct joycon *joycon = dev_get_drvdata(dev);

	/* acquire: the key and the address are stored before the flag */
	if (!smp_load_acquire(&joycon->pairing_valid))
		return -ENODATA;

	return sysfs_emit(buf, "%pM\n", joycon->pairing_host);
}

/* The key is a secret: root only, unlike the address it goes with. */
static ssize_t pairing_key_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct joycon *joycon = dev_get_drvdata(dev);

	/* acquire: the key and the address are stored before the flag */
	if (!smp_load_acquire(&joycon->pairing_valid))
		return -ENODATA;

	return sysfs_emit(buf, "%*phN\n", JOYCON_PAIRING_LTK_SIZE,
			  joycon->pairing_ltk);
}
static DEVICE_ATTR_ADMIN_RO(pairing_key);

/* A controller holds one host, so this replaces the console it came from. */
static ssize_t pairing_host_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct joycon *joycon = dev_get_drvdata(dev);
	u8 addr[6];

	if (!mac_pton(buf, addr))
		return -EINVAL;

	if (!joycon->initialized)
		return -ENODEV;

	if (joycon->pair_step)
		return -EBUSY;

	/* The controller wants it back to front. */
	joycon_memcpy_rev(joycon->pair_host_req, addr, sizeof(addr));

	joycon->pair_err = -ETIMEDOUT;
	reinit_completion(&joycon->pair_done);

	/* Published before the first step goes out, since the reply drives the rest. */
	joycon->pair_step = JOYCON_HID_SET_SHIPMENT;
	joycon_send_hidcommand(joycon->serdev, JOYCON_HID_SET_SHIPMENT,
			       (u8[]){ JOYCON_SHIPMENT_CLEAR }, 1);

	if (!wait_for_completion_timeout(&joycon->pair_done,
					 msecs_to_jiffies(JOYCON_PAIR_TIMEOUT_MS))) {
		joycon->pair_step = 0;
		dev_err(joycon->dev, "pairing timed out\n");
		return -ETIMEDOUT;
	}

	return joycon->pair_err ? joycon->pair_err : count;
}
static DEVICE_ATTR_RW(pairing_host);

/* Only a way to reach Bluetooth without undocking. */
static ssize_t reconnect_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct joycon *joycon = dev_get_drvdata(dev);
	bool go;

	if (kstrtobool(buf, &go))
		return -EINVAL;
	if (!go)
		return count;
	if (!joycon->initialized)
		return -ENODEV;

	/* It reboots, so there is no reply and nothing to wait for. */
	joycon_send_hidcommand(joycon->serdev, JOYCON_HID_SET_HCI_STATE,
			       (u8[]){ JOYCON_HCI_STATE_RECONNECT }, 1);
	dev_info(joycon->dev, "restarting to reconnect over bluetooth\n");

	return count;
}
static DEVICE_ATTR_WO(reconnect);

static struct attribute *joycon_attrs[] = {
	&dev_attr_pairing_host.attr,
	&dev_attr_pairing_key.attr,
	&dev_attr_reconnect.attr,
	NULL,
};
ATTRIBUTE_GROUPS(joycon);

static struct serdev_device_driver joycon_serdev_driver = {
	.probe = joycon_serdev_probe,
	.remove = joycon_serdev_remove,
	.driver = {
		.name = "joycon-uart",
		.of_match_table = joycon_uart_of_match,
		.dev_groups = joycon_groups,
	},
};

static int __init joycon_init(void)
{
	/* Before the driver, which can be probed the moment it registers. */
	crc8_populate_lsb(joycon_crc_table, JOYCON_CRC8_POLY);

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
