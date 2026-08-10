// SPDX-License-Identifier: GPL-2.0+
/*
 * HID driver for Nintendo Switch Joy-Cons and Pro Controllers, as well as
 * Nintendo Switch 2 Joy-Cons, Pro Controller, and GameCube Controller
 *
 * Copyright (c) 2019-2021 Daniel J. Ogorchock <djogorchock@gmail.com>
 * Portions Copyright (c) 2020 Nadia Holmquist Pedersen <nadia@nhp.sh>
 * Copyright (c) 2022 Emily Strickland <linux@emily.st>
 * Copyright (c) 2023 Ryan McClelland <rymcclel@gmail.com>
 * Copyright (c) 2026 Valve Software
 *
 * The following resources/projects were referenced for this driver:
 *   https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 *   https://gitlab.com/pjranki/joycon-linux-kernel (Peter Rankin)
 *   https://github.com/FrotBot/SwitchProConLinuxUSB
 *   https://github.com/MTCKC/ProconXInput
 *   https://github.com/Davidobot/BetterJoyForCemu
 *   https://gist.github.com/shinyquagsire23/66f006b46c56216acbaac6c1e2279b64
 *   https://github.com/ndeadly/switch2_controller_research
 *   hid-wiimote kernel hid driver
 *   hid-logitech-hidpp driver
 *   hid-sony driver
 *
 * This driver supports the Nintendo Switch Joy-Cons and Pro Controllers. The
 * Pro Controllers can either be used over USB or Bluetooth.
 *
 * This driver also incorporates support for Nintendo Switch Online controllers
 * for the NES, SNES, Sega Genesis, and N64.
 *
 * The driver will retrieve the factory calibration info from the controllers,
 * so little to no user calibration should be required.
 *
 */

#include <linux/nintendo-joycon.h>

#include "hid-ids.h"
#include "hid-nintendo.h"
#include <linux/unaligned.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/idr.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include "usbhid/usbhid.h"

/*
 * Reference the url below for the following HID report defines:
 * https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 */

/* Output Reports */
#define JC_OUTPUT_RUMBLE_AND_SUBCMD	 0x01
#define JC_OUTPUT_FW_UPDATE_PKT		 0x03
#define JC_OUTPUT_RUMBLE_ONLY		 0x10
#define JC_OUTPUT_MCU_DATA		 0x11
#define JC_OUTPUT_USB_CMD		 0x80

/* Subcommand IDs */
#define JC_SUBCMD_STATE			 0x00
#define JC_SUBCMD_MANUAL_BT_PAIRING	 0x01
#define JC_SUBCMD_REQ_DEV_INFO		 0x02
#define JC_SUBCMD_SET_REPORT_MODE	 0x03
#define JC_SUBCMD_TRIGGERS_ELAPSED	 0x04
#define JC_SUBCMD_GET_PAGE_LIST_STATE	 0x05
#define JC_SUBCMD_SET_HCI_STATE		 0x06
#define JC_SUBCMD_RESET_PAIRING_INFO	 0x07
#define JC_SUBCMD_LOW_POWER_MODE	 0x08
#define JC_SUBCMD_SPI_FLASH_READ	 0x10
#define JC_SUBCMD_SPI_FLASH_WRITE	 0x11
#define JC_SUBCMD_RESET_MCU		 0x20
#define JC_SUBCMD_SET_MCU_CONFIG	 0x21
#define JC_SUBCMD_SET_MCU_STATE		 0x22
#define JC_SUBCMD_SET_PLAYER_LIGHTS	 0x30
#define JC_SUBCMD_GET_PLAYER_LIGHTS	 0x31
#define JC_SUBCMD_SET_HOME_LIGHT	 0x38
#define JC_SUBCMD_ENABLE_IMU		 0x40
#define JC_SUBCMD_SET_IMU_SENSITIVITY	 0x41
#define JC_SUBCMD_WRITE_IMU_REG		 0x42
#define JC_SUBCMD_READ_IMU_REG		 0x43
#define JC_SUBCMD_ENABLE_VIBRATION	 0x48
#define JC_SUBCMD_GET_REGULATED_VOLTAGE	 0x50

/* Input Reports */
#define JC_INPUT_BUTTON_EVENT		 0x3F
#define JC_INPUT_SUBCMD_REPLY		 0x21
#define JC_INPUT_IMU_DATA		 0x30
#define JC_INPUT_MCU_DATA		 0x31
#define JC_INPUT_USB_RESPONSE		 0x81

/* Feature Reports */
#define JC_FEATURE_LAST_SUBCMD		 0x02
#define JC_FEATURE_OTA_FW_UPGRADE	 0x70
#define JC_FEATURE_SETUP_MEM_READ	 0x71
#define JC_FEATURE_MEM_READ		 0x72
#define JC_FEATURE_ERASE_MEM_SECTOR	 0x73
#define JC_FEATURE_MEM_WRITE		 0x74
#define JC_FEATURE_LAUNCH		 0x75

/* USB Commands */
#define JC_USB_CMD_CONN_STATUS		 0x01
#define JC_USB_CMD_HANDSHAKE		 0x02
#define JC_USB_CMD_BAUDRATE_3M		 0x03
#define JC_USB_CMD_NO_TIMEOUT		 0x04
#define JC_USB_CMD_EN_TIMEOUT		 0x05
#define JC_USB_RESET			 0x06
#define JC_USB_PRE_HANDSHAKE		 0x91
#define JC_USB_SEND_UART		 0x92

/* Hat values for pro controller's d-pad */
#define JC_MAX_DPAD_MAG		1
#define JC_DPAD_FUZZ		0
#define JC_DPAD_FLAT		0

/* Under most circumstances IMU reports are pushed every 15ms; use as default */
#define JC_IMU_DFLT_AVG_DELTA_MS	15
/* How many samples to sum before calculating average IMU report delta */
#define JC_IMU_SAMPLES_PER_DELTA_AVG	300
/* Controls how many dropped IMU packets at once trigger a warning message */
#define JC_IMU_DROPPED_PKT_WARNING	3

static const u16 JC_RUMBLE_PERIOD_MS = 50;

/* States for controller state machine */
enum joycon_ctlr_state {
	JOYCON_CTLR_STATE_INIT,
	JOYCON_CTLR_STATE_READ,
	JOYCON_CTLR_STATE_REMOVED,
	JOYCON_CTLR_STATE_SUSPENDED,
};

static const struct joycon_ctlr_button_mapping nescon_button_mappings[] = {
	{ BTN_SOUTH,	JC_BTN_A,	},
	{ BTN_EAST,	JC_BTN_B,	},
	{ BTN_TL,	JC_BTN_L,	},
	{ BTN_TR,	JC_BTN_R,	},
	{ BTN_SELECT,	JC_BTN_MINUS,	},
	{ BTN_START,	JC_BTN_PLUS,	},
	{ /* sentinel */ },
};

static const struct joycon_ctlr_button_mapping snescon_button_mappings[] = {
	{ BTN_EAST,	JC_BTN_A,	},
	{ BTN_SOUTH,	JC_BTN_B,	},
	{ BTN_NORTH,	JC_BTN_X,	},
	{ BTN_WEST,	JC_BTN_Y,	},
	{ BTN_TL,	JC_BTN_L,	},
	{ BTN_TR,	JC_BTN_R,	},
	{ BTN_TL2,	JC_BTN_ZL,	},
	{ BTN_TR2,	JC_BTN_ZR,	},
	{ BTN_SELECT,	JC_BTN_MINUS,	},
	{ BTN_START,	JC_BTN_PLUS,	},
	{ /* sentinel */ },
};

static const struct joycon_ctlr_button_mapping gencon_button_mappings[] = {
	{ BTN_WEST,	JC_BTN_A,	}, /* A */
	{ BTN_SOUTH,	JC_BTN_B,	}, /* B */
	{ BTN_EAST,	JC_BTN_R,	}, /* C */
	{ BTN_TL,	JC_BTN_X,	}, /* X MD/GEN 6B Only */
	{ BTN_NORTH,	JC_BTN_Y,	}, /* Y MD/GEN 6B Only */
	{ BTN_TR,	JC_BTN_L,	}, /* Z MD/GEN 6B Only */
	{ BTN_SELECT,	JC_BTN_ZR,	}, /* Mode */
	{ BTN_START,	JC_BTN_PLUS,	},
	{ BTN_MODE,	JC_BTN_HOME,	},
	{ BTN_Z,	JC_BTN_CAP,	},
	{ /* sentinel */ },
};

static const struct joycon_ctlr_button_mapping n64con_button_mappings[] = {
	{ BTN_A,		JC_BTN_A,	},
	{ BTN_B,		JC_BTN_B,	},
	{ BTN_TL2,		JC_BTN_ZL,	}, /* Z */
	{ BTN_TL,		JC_BTN_L,	},
	{ BTN_TR,		JC_BTN_R,	},
	{ BTN_TR2,		JC_BTN_LSTICK,	}, /* ZR */
	{ BTN_START,		JC_BTN_PLUS,	},
	{ BTN_SELECT,		JC_BTN_Y,	}, /* C UP */
	{ BTN_X,		JC_BTN_ZR,	}, /* C DOWN */
	{ BTN_Y,		JC_BTN_X,	}, /* C LEFT */
	{ BTN_C,		JC_BTN_MINUS,	}, /* C RIGHT */
	{ BTN_MODE,		JC_BTN_HOME,	},
	{ BTN_Z,		JC_BTN_CAP,	},
	{ /* sentinel */ },
};

enum joycon_msg_type {
	JOYCON_MSG_TYPE_NONE,
	JOYCON_MSG_TYPE_USB,
	JOYCON_MSG_TYPE_SUBCMD,
};

struct joycon_rumble_output {
	u8 output_id;
	u8 packet_num;
	u8 rumble_data[8];
} __packed;

struct joycon_subcmd_request {
	u8 output_id; /* must be 0x01 for subcommand, 0x10 for rumble only */
	u8 packet_num; /* incremented every send */
	u8 rumble_data[8];
	u8 subcmd_id;
	u8 data[]; /* length depends on the subcommand */
} __packed;

struct joycon_subcmd_reply {
	u8 ack; /* MSB 1 for ACK, 0 for NACK */
	u8 id; /* id of requested subcmd */
	u8 data[]; /* will be at most 35 bytes */
} __packed;

struct joycon_input_report {
	u8 id;
	u8 timer;
	u8 bat_con; /* battery and connection info */
	u8 button_status[3];
	u8 left_stick[3];
	u8 right_stick[3];
	u8 vibrator_report;

	union {
		struct joycon_subcmd_reply subcmd_reply;
		/* IMU input reports contain 3 samples */
		u8 imu_raw_bytes[sizeof(struct joycon_imu_data) * 3];
	};
} __packed;

#define JC_MAX_RESP_SIZE	(sizeof(struct joycon_input_report) + 35)
#define JC_RUMBLE_QUEUE_SIZE	8

static const char * const joycon_player_led_names[] = {
	LED_FUNCTION_PLAYER1,
	LED_FUNCTION_PLAYER2,
	LED_FUNCTION_PLAYER3,
	LED_FUNCTION_PLAYER4,
};
#define JC_NUM_LEDS		ARRAY_SIZE(joycon_player_led_names)
#define JC_NUM_LED_PATTERNS 8
/* Taken from https://www.nintendo.com/my/support/qa/detail/33822 */
static const enum led_brightness joycon_player_led_patterns[JC_NUM_LED_PATTERNS][JC_NUM_LEDS] = {
	{ 1, 0, 0, 0 },
	{ 1, 1, 0, 0 },
	{ 1, 1, 1, 0 },
	{ 1, 1, 1, 1 },
	{ 1, 0, 0, 1 },
	{ 1, 0, 1, 0 },
	{ 1, 0, 1, 1 },
	{ 0, 1, 1, 0 },
};

/* Each physical controller is associated with a joycon_ctlr struct */
struct joycon_ctlr {
	struct hid_device *hdev;
	struct input_dev *input;
	u32 player_id;
	struct led_classdev leds[JC_NUM_LEDS]; /* player leds */
	struct led_classdev home_led;
	enum joycon_ctlr_state ctlr_state;
	spinlock_t lock;
	u8 mac_addr[6];
	char *mac_addr_str;
	enum joycon_ctlr_type ctlr_type;

	/* The following members are used for synchronous sends/receives */
	enum joycon_msg_type msg_type;
	u8 subcmd_num;
	struct mutex output_mutex;
	u8 input_buf[JC_MAX_RESP_SIZE];
	wait_queue_head_t wait;
	bool received_resp;
	u8 usb_ack_match;
	u8 subcmd_ack_match;
	bool received_input_report;
	unsigned int last_input_report_msecs;
	unsigned int last_subcmd_sent_msecs;
	unsigned int consecutive_valid_report_deltas;
	unsigned int subcmd_rate_exhaustions;
	bool subcmd_rate_relaxed;

	/* factory calibration data */
	struct joycon_stick_cal left_stick_cal_x;
	struct joycon_stick_cal left_stick_cal_y;
	struct joycon_stick_cal right_stick_cal_x;
	struct joycon_stick_cal right_stick_cal_y;

	struct joycon_imu_cal accel_cal;
	struct joycon_imu_cal gyro_cal;

	/* power supply data */
	struct power_supply *battery;
	struct power_supply_desc battery_desc;
	u8 battery_capacity;
	bool battery_charging;
	bool host_powered;

	/* rumble */
	u8 rumble_data[JC_RUMBLE_QUEUE_SIZE][JC_RUMBLE_DATA_SIZE];
	int rumble_queue_head;
	int rumble_queue_tail;
	struct workqueue_struct *rumble_queue;
	struct work_struct rumble_worker;
	unsigned int rumble_msecs;
	u16 rumble_ll_freq;
	u16 rumble_lh_freq;
	u16 rumble_rl_freq;
	u16 rumble_rh_freq;
	unsigned short rumble_zero_countdown;

	/* imu */
	struct input_dev *imu_input;
	bool imu_first_packet_received; /* helps in initiating timestamp */
	unsigned int imu_timestamp_us; /* timestamp we report to userspace */
	unsigned int imu_last_pkt_ms; /* used to calc imu report delta */
	/* the following are used to track the average imu report time delta */
	unsigned int imu_delta_samples_count;
	unsigned int imu_delta_samples_sum;
	unsigned int imu_avg_delta_ms;
};

/* Helper macros for checking controller type */
#define jc_type_is_joycon(ctlr) \
	(ctlr->hdev->product == USB_DEVICE_ID_NINTENDO_JOYCONL || \
	 ctlr->hdev->product == USB_DEVICE_ID_NINTENDO_JOYCONR || \
	 ctlr->hdev->product == USB_DEVICE_ID_NINTENDO_CHRGGRIP)
#define jc_type_is_procon(ctlr) \
	(ctlr->hdev->product == USB_DEVICE_ID_NINTENDO_PROCON)
#define jc_type_is_chrggrip(ctlr) \
	(ctlr->hdev->product == USB_DEVICE_ID_NINTENDO_CHRGGRIP)

/* Does this controller have inputs associated with left joycon? */
#define jc_type_has_left(ctlr) \
	(ctlr->ctlr_type == JOYCON_CTLR_TYPE_JCL || \
	 ctlr->ctlr_type == JOYCON_CTLR_TYPE_PRO || \
	 ctlr->ctlr_type == JOYCON_CTLR_TYPE_N64)

/* Does this controller have inputs associated with right joycon? */
#define jc_type_has_right(ctlr) \
	(ctlr->ctlr_type == JOYCON_CTLR_TYPE_JCR || \
	 ctlr->ctlr_type == JOYCON_CTLR_TYPE_PRO)

/*
 * Controller device helpers
 *
 * These look at the device ID known to the HID subsystem to identify a device,
 * but take caution: some NSO devices lie about themselves (NES Joy-Cons and
 * Sega Genesis controller). See type helpers below.
 *
 * These helpers are most useful early during the HID probe or in conjunction
 * with the capability helpers below.
 */
static inline bool joycon_device_is_chrggrip(struct joycon_ctlr *ctlr)
{
	return ctlr->hdev->product == USB_DEVICE_ID_NINTENDO_CHRGGRIP;
}

/*
 * Controller type helpers
 *
 * These are slightly different than the device-ID-based helpers above. They are
 * generally more reliable, since they can distinguish between, e.g., Genesis
 * versus SNES, or NES Joy-Cons versus regular Switch Joy-Cons. They're most
 * useful for reporting available inputs. For other kinds of distinctions, see
 * the capability helpers below.
 *
 * They have two major drawbacks: (1) they're not available until after we set
 * the reporting method and then request the device info; (2) they can't
 * distinguish all controllers (like the Charging Grip from the Pro controller.)
 */
static inline bool joycon_type_is_left_joycon(struct joycon_ctlr *ctlr)
{
	return ctlr->ctlr_type == JOYCON_CTLR_TYPE_JCL;
}

static inline bool joycon_type_is_right_joycon(struct joycon_ctlr *ctlr)
{
	return ctlr->ctlr_type == JOYCON_CTLR_TYPE_JCR;
}

static inline bool joycon_type_is_procon(struct joycon_ctlr *ctlr)
{
	return ctlr->ctlr_type == JOYCON_CTLR_TYPE_PRO ||
	       ctlr->ctlr_type == JOYCON_CTLR_TYPE_LIC_PRO;
}

static inline bool joycon_type_is_snescon(struct joycon_ctlr *ctlr)
{
	return ctlr->ctlr_type == JOYCON_CTLR_TYPE_SNES;
}

static inline bool joycon_type_is_gencon(struct joycon_ctlr *ctlr)
{
	return ctlr->ctlr_type == JOYCON_CTLR_TYPE_GEN;
}

static inline bool joycon_type_is_n64con(struct joycon_ctlr *ctlr)
{
	return ctlr->ctlr_type == JOYCON_CTLR_TYPE_N64;
}

static inline bool joycon_type_is_left_nescon(struct joycon_ctlr *ctlr)
{
	return ctlr->ctlr_type == JOYCON_CTLR_TYPE_NESL;
}

static inline bool joycon_type_is_right_nescon(struct joycon_ctlr *ctlr)
{
	return ctlr->ctlr_type == JOYCON_CTLR_TYPE_NESR;
}

static inline bool joycon_type_is_any_joycon(struct joycon_ctlr *ctlr)
{
	return joycon_type_is_left_joycon(ctlr) ||
	       joycon_type_is_right_joycon(ctlr) ||
	       joycon_device_is_chrggrip(ctlr);
}

static inline bool joycon_type_is_any_nescon(struct joycon_ctlr *ctlr)
{
	return joycon_type_is_left_nescon(ctlr) ||
	       joycon_type_is_right_nescon(ctlr);
}

/*
 * Controller capability helpers
 *
 * These helpers combine the use of the helpers above to detect certain
 * capabilities during initialization. They are always accurate but (since they
 * use type helpers) cannot be used early in the HID probe.
 */
static inline bool joycon_has_imu(struct joycon_ctlr *ctlr)
{
	return joycon_device_is_chrggrip(ctlr) ||
	       joycon_type_is_any_joycon(ctlr) ||
	       joycon_type_is_procon(ctlr);
}

static inline bool joycon_has_joysticks(struct joycon_ctlr *ctlr)
{
	return joycon_device_is_chrggrip(ctlr) ||
	       joycon_type_is_any_joycon(ctlr) ||
	       joycon_type_is_procon(ctlr) ||
	       joycon_type_is_n64con(ctlr);
}

static inline bool joycon_has_rumble(struct joycon_ctlr *ctlr)
{
	return joycon_device_is_chrggrip(ctlr) ||
	       joycon_type_is_any_joycon(ctlr) ||
	       joycon_type_is_procon(ctlr) ||
	       joycon_type_is_n64con(ctlr);
}

static inline bool joycon_using_usb(struct joycon_ctlr *ctlr)
{
	return ctlr->hdev->bus == BUS_USB;
}

static int __joycon_hid_send(struct hid_device *hdev, u8 *data, size_t len)
{
	u8 *buf;
	int ret;

	buf = kmemdup(data, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = hid_hw_output_report(hdev, buf, len);
	kfree(buf);
	if (ret < 0)
		hid_dbg(hdev, "Failed to send output report ret=%d\n", ret);
	return ret;
}

static void joycon_wait_for_input_report(struct joycon_ctlr *ctlr)
{
	int ret;

	/*
	 * If we are in the proper reporting mode, wait for an input
	 * report prior to sending the subcommand. This improves
	 * reliability considerably.
	 */
	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_READ) {
		unsigned long flags;

		spin_lock_irqsave(&ctlr->lock, flags);
		ctlr->received_input_report = false;
		spin_unlock_irqrestore(&ctlr->lock, flags);
		ret = wait_event_timeout(ctlr->wait,
					 ctlr->received_input_report,
					 HZ / 4);
		/* We will still proceed, even with a timeout here */
		if (!ret)
			hid_warn(ctlr->hdev,
				 "timeout waiting for input report\n");
	}
}

/*
 * Sending subcommands and/or rumble data at too high a rate can cause bluetooth
 * controller disconnections.
 */
#define JC_INPUT_REPORT_MIN_DELTA	8
#define JC_INPUT_REPORT_MAX_DELTA	17
#define JC_SUBCMD_TX_OFFSET_MS		4
#define JC_SUBCMD_VALID_DELTA_REQ	3
#define JC_SUBCMD_RATE_MAX_ATTEMPTS	25
#define JC_SUBCMD_RATE_MAX_FAILURES	4
#define JC_SUBCMD_RATE_LIMITER_USB_MS	20
#define JC_SUBCMD_RATE_LIMITER_BT_MS	60
#define JC_SUBCMD_RATE_LIMITER_MS(ctlr)	((ctlr)->hdev->bus == BUS_USB ? JC_SUBCMD_RATE_LIMITER_USB_MS : JC_SUBCMD_RATE_LIMITER_BT_MS)
static void joycon_enforce_subcmd_rate_strict(struct joycon_ctlr *ctlr)
{
	unsigned int current_ms;
	unsigned long subcmd_delta;
	int consecutive_valid_deltas = 0;
	int attempts = 0;
	unsigned long flags;

	if (unlikely(ctlr->ctlr_state != JOYCON_CTLR_STATE_READ))
		return;

	do {
		joycon_wait_for_input_report(ctlr);
		current_ms = jiffies_to_msecs(jiffies);
		subcmd_delta = current_ms - ctlr->last_subcmd_sent_msecs;

		spin_lock_irqsave(&ctlr->lock, flags);
		consecutive_valid_deltas = ctlr->consecutive_valid_report_deltas;
		spin_unlock_irqrestore(&ctlr->lock, flags);

		attempts++;
	} while ((consecutive_valid_deltas < JC_SUBCMD_VALID_DELTA_REQ ||
		  subcmd_delta < JC_SUBCMD_RATE_LIMITER_MS(ctlr)) &&
		 ctlr->ctlr_state == JOYCON_CTLR_STATE_READ &&
		 attempts < JC_SUBCMD_RATE_MAX_ATTEMPTS);

	if (attempts >= JC_SUBCMD_RATE_MAX_ATTEMPTS) {
		hid_warn(ctlr->hdev, "%s: exceeded max attempts", __func__);

		if (++ctlr->subcmd_rate_exhaustions == JC_SUBCMD_RATE_MAX_FAILURES) {
			ctlr->subcmd_rate_relaxed = true;
			hid_info(ctlr->hdev,
				 "input report cadence does not fit the %d-%dms window; using the legacy subcommand throttle\n",
				 JC_INPUT_REPORT_MIN_DELTA,
				 JC_INPUT_REPORT_MAX_DELTA);
		}
		return;
	}

	ctlr->last_subcmd_sent_msecs = current_ms;

	/*
	 * Wait a short time after receiving an input report before
	 * transmitting. This should reduce odds of a TX coinciding with an RX.
	 * Minimizing concurrent BT traffic with the controller seems to lower
	 * the rate of disconnections.
	 */
	msleep(JC_SUBCMD_TX_OFFSET_MS);
}

/* The rate limiter as it was before commit d750d1480362, without the report
 * cadence requirement.
 */
static void joycon_enforce_subcmd_rate_legacy(struct joycon_ctlr *ctlr)
{
	static const unsigned int max_subcmd_rate_ms = 25;
	unsigned int current_ms = jiffies_to_msecs(jiffies);
	unsigned int delta_ms = current_ms - ctlr->last_subcmd_sent_msecs;

	while (delta_ms < max_subcmd_rate_ms &&
	       ctlr->ctlr_state == JOYCON_CTLR_STATE_READ) {
		joycon_wait_for_input_report(ctlr);
		current_ms = jiffies_to_msecs(jiffies);
		delta_ms = current_ms - ctlr->last_subcmd_sent_msecs;
	}
	ctlr->last_subcmd_sent_msecs = current_ms;
}

static void joycon_enforce_subcmd_rate(struct joycon_ctlr *ctlr)
{
	if (ctlr->subcmd_rate_relaxed)
		joycon_enforce_subcmd_rate_legacy(ctlr);
	else
		joycon_enforce_subcmd_rate_strict(ctlr);
}

static int joycon_hid_send_sync(struct joycon_ctlr *ctlr, u8 *data, size_t len,
				u32 timeout)
{
	int ret;
	int tries = 2;

	/*
	 * The controller occasionally seems to drop subcommands. In testing,
	 * doing one retry after a timeout appears to always work.
	 */
	while (tries--) {
		joycon_enforce_subcmd_rate(ctlr);

		ret = __joycon_hid_send(ctlr->hdev, data, len);
		if (ret < 0) {
			memset(ctlr->input_buf, 0, JC_MAX_RESP_SIZE);
			return ret;
		}

		ret = wait_event_timeout(ctlr->wait, ctlr->received_resp,
					 timeout);
		if (!ret) {
			hid_dbg(ctlr->hdev,
				"synchronous send/receive timed out\n");
			if (tries) {
				hid_dbg(ctlr->hdev,
					"retrying sync send after timeout\n");
			}
			memset(ctlr->input_buf, 0, JC_MAX_RESP_SIZE);
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
			break;
		}
	}

	ctlr->received_resp = false;
	return ret;
}

static int joycon_send_usb(struct joycon_ctlr *ctlr, u8 cmd, u32 timeout)
{
	int ret;
	u8 buf[2] = {JC_OUTPUT_USB_CMD};

	buf[1] = cmd;
	ctlr->usb_ack_match = cmd;
	ctlr->msg_type = JOYCON_MSG_TYPE_USB;
	ret = joycon_hid_send_sync(ctlr, buf, sizeof(buf), timeout);
	if (ret)
		hid_dbg(ctlr->hdev, "send usb command failed; ret=%d\n", ret);
	return ret;
}

static int joycon_send_subcmd(struct joycon_ctlr *ctlr,
			      struct joycon_subcmd_request *subcmd,
			      size_t data_len, u32 timeout)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&ctlr->lock, flags);
	/*
	 * If the controller has been removed, just return ENODEV so the LED
	 * subsystem doesn't print invalid errors on removal.
	 */
	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_REMOVED) {
		spin_unlock_irqrestore(&ctlr->lock, flags);
		return -ENODEV;
	}
	memcpy(subcmd->rumble_data, ctlr->rumble_data[ctlr->rumble_queue_tail],
	       JC_RUMBLE_DATA_SIZE);
	spin_unlock_irqrestore(&ctlr->lock, flags);

	subcmd->output_id = JC_OUTPUT_RUMBLE_AND_SUBCMD;
	subcmd->packet_num = ctlr->subcmd_num;
	if (++ctlr->subcmd_num > 0xF)
		ctlr->subcmd_num = 0;
	ctlr->subcmd_ack_match = subcmd->subcmd_id;
	ctlr->msg_type = JOYCON_MSG_TYPE_SUBCMD;

	ret = joycon_hid_send_sync(ctlr, (u8 *)subcmd,
				   sizeof(*subcmd) + data_len, timeout);
	if (ret < 0)
		hid_dbg(ctlr->hdev, "send subcommand failed; ret=%d\n", ret);
	else
		ret = 0;
	return ret;
}

/* Supply nibbles for flash and on. Ones correspond to active */
static int joycon_set_player_leds(struct joycon_ctlr *ctlr, u8 flash, u8 on)
{
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_SET_PLAYER_LIGHTS;
	req->data[0] = (flash << 4) | on;

	hid_dbg(ctlr->hdev, "setting player leds\n");
	return joycon_send_subcmd(ctlr, req, 1, HZ/4);
}

static int joycon_set_home_led(struct joycon_ctlr *ctlr, enum led_brightness brightness)
{
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 5] = { 0 };
	u8 *data;

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_SET_HOME_LIGHT;
	data = req->data;
	data[0] = 0x01;
	data[1] = brightness << 4;
	data[2] = brightness | (brightness << 4);
	data[3] = 0x11;
	data[4] = 0x11;

	hid_dbg(ctlr->hdev, "setting home led brightness\n");
	return joycon_send_subcmd(ctlr, req, 5, HZ/4);
}

static int joycon_request_spi_flash_read(struct joycon_ctlr *ctlr,
					 u32 start_addr, u8 size, u8 **reply)
{
	struct joycon_subcmd_request *req;
	struct joycon_input_report *report;
	u8 buffer[sizeof(*req) + 5] = { 0 };
	u8 *data;
	int ret;

	if (!reply)
		return -EINVAL;

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_SPI_FLASH_READ;
	data = req->data;
	put_unaligned_le32(start_addr, data);
	data[4] = size;

	hid_dbg(ctlr->hdev, "requesting SPI flash data\n");
	ret = joycon_send_subcmd(ctlr, req, 5, HZ);
	if (ret) {
		hid_err(ctlr->hdev, "failed reading SPI flash; ret=%d\n", ret);
	} else {
		report = (struct joycon_input_report *)ctlr->input_buf;
		/* The read data starts at the 6th byte */
		*reply = &report->subcmd_reply.data[5];
	}
	return ret;
}

/*
 * User calibration's presence is denoted with a magic byte preceding it.
 * returns 0 if magic val is present, 1 if not present, < 0 on error
 */
static int joycon_check_for_cal_magic(struct joycon_ctlr *ctlr, u32 flash_addr)
{
	int ret;
	u8 *reply;

	ret = joycon_request_spi_flash_read(ctlr, flash_addr,
					    JC_CAL_USR_MAGIC_SIZE, &reply);
	if (ret)
		return ret;

	return reply[0] != JC_CAL_USR_MAGIC_0 || reply[1] != JC_CAL_USR_MAGIC_1;
}

static int joycon_read_stick_calibration(struct joycon_ctlr *ctlr, u16 cal_addr,
					 struct joycon_stick_cal *cal_x,
					 struct joycon_stick_cal *cal_y,
					 bool left_stick)
{
	struct joycon_stick_cal parsed_x, parsed_y;
	u8 *raw_cal;
	int ret;

	ret = joycon_request_spi_flash_read(ctlr, cal_addr,
					    JC_CAL_STICK_DATA_SIZE, &raw_cal);
	if (ret)
		return ret;

	joycon_parse_stick_cal(&parsed_x, &parsed_y, raw_cal, left_stick);
	if (!joycon_stick_cal_valid(&parsed_x, &parsed_y))
		return -EINVAL;

	*cal_x = parsed_x;
	*cal_y = parsed_y;

	return 0;
}

static void joycon_use_default_calibration(struct hid_device *hdev,
					   struct joycon_stick_cal *cal_x,
					   struct joycon_stick_cal *cal_y,
					   const char *stick, int ret)
{
	hid_warn(hdev,
		 "Failed to read %s stick cal, using defaults; e=%d\n",
		 stick, ret);

	joycon_stick_cal_defaults(cal_x, cal_y);
}

static int joycon_request_calibration(struct joycon_ctlr *ctlr)
{
	u16 left_stick_addr = JC_CAL_FCT_DATA_LEFT_ADDR;
	u16 right_stick_addr = JC_CAL_FCT_DATA_RIGHT_ADDR;
	int ret;

	hid_dbg(ctlr->hdev, "requesting cal data\n");

	/* check if user stick calibrations are present */
	if (!joycon_check_for_cal_magic(ctlr, JC_CAL_USR_LEFT_MAGIC_ADDR)) {
		left_stick_addr = JC_CAL_USR_LEFT_DATA_ADDR;
		hid_info(ctlr->hdev, "using user cal for left stick\n");
	} else {
		hid_info(ctlr->hdev, "using factory cal for left stick\n");
	}
	if (!joycon_check_for_cal_magic(ctlr, JC_CAL_USR_RIGHT_MAGIC_ADDR)) {
		right_stick_addr = JC_CAL_USR_RIGHT_DATA_ADDR;
		hid_info(ctlr->hdev, "using user cal for right stick\n");
	} else {
		hid_info(ctlr->hdev, "using factory cal for right stick\n");
	}

	/* read the left stick calibration data */
	ret = joycon_read_stick_calibration(ctlr, left_stick_addr,
					    &ctlr->left_stick_cal_x,
					    &ctlr->left_stick_cal_y,
					    true);

	if (ret)
		joycon_use_default_calibration(ctlr->hdev,
					       &ctlr->left_stick_cal_x,
					       &ctlr->left_stick_cal_y,
					       "left", ret);

	/* read the right stick calibration data */
	ret = joycon_read_stick_calibration(ctlr, right_stick_addr,
					    &ctlr->right_stick_cal_x,
					    &ctlr->right_stick_cal_y,
					    false);

	if (ret)
		joycon_use_default_calibration(ctlr->hdev,
					       &ctlr->right_stick_cal_x,
					       &ctlr->right_stick_cal_y,
					       "right", ret);

	hid_dbg(ctlr->hdev, "calibration:\n"
			    "l_x_c=%d l_x_max=%d l_x_min=%d\n"
			    "l_y_c=%d l_y_max=%d l_y_min=%d\n"
			    "r_x_c=%d r_x_max=%d r_x_min=%d\n"
			    "r_y_c=%d r_y_max=%d r_y_min=%d\n",
			    ctlr->left_stick_cal_x.center,
			    ctlr->left_stick_cal_x.max,
			    ctlr->left_stick_cal_x.min,
			    ctlr->left_stick_cal_y.center,
			    ctlr->left_stick_cal_y.max,
			    ctlr->left_stick_cal_y.min,
			    ctlr->right_stick_cal_x.center,
			    ctlr->right_stick_cal_x.max,
			    ctlr->right_stick_cal_x.min,
			    ctlr->right_stick_cal_y.center,
			    ctlr->right_stick_cal_y.max,
			    ctlr->right_stick_cal_y.min);

	return 0;
}

static int joycon_request_imu_calibration(struct joycon_ctlr *ctlr)
{
	u16 imu_cal_addr = JC_IMU_CAL_FCT_DATA_ADDR;
	u8 *raw_cal;
	int ret;

	/* check if user calibration exists */
	if (!joycon_check_for_cal_magic(ctlr, JC_IMU_CAL_USR_MAGIC_ADDR)) {
		imu_cal_addr = JC_IMU_CAL_USR_DATA_ADDR;
		hid_info(ctlr->hdev, "using user cal for IMU\n");
	} else {
		hid_info(ctlr->hdev, "using factory cal for IMU\n");
	}

	/* request IMU calibration data */
	hid_dbg(ctlr->hdev, "requesting IMU cal data\n");
	ret = joycon_request_spi_flash_read(ctlr, imu_cal_addr,
					    JC_IMU_CAL_DATA_SIZE, &raw_cal);
	if (ret) {
		hid_warn(ctlr->hdev,
			 "Failed to read IMU cal, using defaults; ret=%d\n",
			 ret);

		joycon_imu_cal_defaults(&ctlr->accel_cal, &ctlr->gyro_cal);
		return ret;
	}

	joycon_parse_imu_cal(&ctlr->accel_cal, &ctlr->gyro_cal, raw_cal);

	ret = joycon_imu_cal_zero_divisors(&ctlr->accel_cal, &ctlr->gyro_cal);
	if (ret)
		hid_warn(ctlr->hdev, "inaccurate IMU divisors (%d)\n", ret);

	hid_dbg(ctlr->hdev, "IMU calibration:\n"
			    "a_o[0]=%d a_o[1]=%d a_o[2]=%d\n"
			    "a_s[0]=%d a_s[1]=%d a_s[2]=%d\n"
			    "g_o[0]=%d g_o[1]=%d g_o[2]=%d\n"
			    "g_s[0]=%d g_s[1]=%d g_s[2]=%d\n",
			    ctlr->accel_cal.offset[0],
			    ctlr->accel_cal.offset[1],
			    ctlr->accel_cal.offset[2],
			    ctlr->accel_cal.scale[0],
			    ctlr->accel_cal.scale[1],
			    ctlr->accel_cal.scale[2],
			    ctlr->gyro_cal.offset[0],
			    ctlr->gyro_cal.offset[1],
			    ctlr->gyro_cal.offset[2],
			    ctlr->gyro_cal.scale[0],
			    ctlr->gyro_cal.scale[1],
			    ctlr->gyro_cal.scale[2]);

	return 0;
}

static int joycon_set_report_mode(struct joycon_ctlr *ctlr)
{
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_SET_REPORT_MODE;
	req->data[0] = 0x30; /* standard, full report mode */

	hid_dbg(ctlr->hdev, "setting controller report mode\n");
	return joycon_send_subcmd(ctlr, req, 1, HZ);
}

static int joycon_enable_rumble(struct joycon_ctlr *ctlr)
{
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_ENABLE_VIBRATION;
	req->data[0] = 0x01; /* note: 0x00 would disable */

	hid_dbg(ctlr->hdev, "enabling rumble\n");
	return joycon_send_subcmd(ctlr, req, 1, HZ/4);
}

static int joycon_enable_imu(struct joycon_ctlr *ctlr)
{
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_ENABLE_IMU;
	req->data[0] = 0x01; /* note: 0x00 would disable */

	hid_dbg(ctlr->hdev, "enabling IMU\n");
	return joycon_send_subcmd(ctlr, req, 1, HZ);
}

static void joycon_input_report_parse_imu_data(struct joycon_ctlr *ctlr,
					       struct joycon_input_report *rep,
					       struct joycon_imu_data *imu_data)
{
	u8 *raw = rep->imu_raw_bytes;
	int i;

	for (i = 0; i < 3; i++) {
		joycon_parse_imu_data(raw, &imu_data[i]);
		/* point to next imu sample */
		raw += sizeof(struct joycon_imu_data);
	}
}

static void joycon_parse_imu_report(struct joycon_ctlr *ctlr,
				    struct joycon_input_report *rep)
{
	struct joycon_imu_data imu_data[3] = {0}; /* 3 reports per packet */
	struct input_dev *idev = ctlr->imu_input;
	unsigned int msecs = jiffies_to_msecs(jiffies);
	unsigned int last_msecs = ctlr->imu_last_pkt_ms;
	int i;

	joycon_input_report_parse_imu_data(ctlr, rep, imu_data);

	/*
	 * There are complexities surrounding how we determine the timestamps we
	 * associate with the samples we pass to userspace. The IMU input
	 * reports do not provide us with a good timestamp. There's a quickly
	 * incrementing 8-bit counter per input report, but it is not very
	 * useful for this purpose (it is not entirely clear what rate it
	 * increments at or if it varies based on packet push rate - more on
	 * the push rate below...).
	 *
	 * The reverse engineering work done on the joy-cons and pro controllers
	 * by the community seems to indicate the following:
	 * - The controller samples the IMU every 1.35ms. It then does some of
	 *   its own processing, probably averaging the samples out.
	 * - Each imu input report contains 3 IMU samples, (usually 5ms apart).
	 * - In the standard reporting mode (which this driver uses exclusively)
	 *   input reports are pushed from the controller as follows:
	 *      * joy-con (bluetooth): every 15 ms
	 *      * joy-cons (in charging grip via USB): every 15 ms
	 *      * pro controller (USB): every 15 ms
	 *      * pro controller (bluetooth): every 8 ms (this is the wildcard)
	 *
	 * Further complicating matters is that some bluetooth stacks are known
	 * to alter the controller's packet rate by hardcoding the bluetooth
	 * SSR for the switch controllers (android's stack currently sets the
	 * SSR to 11ms for both the joy-cons and pro controllers).
	 *
	 * In my own testing, I've discovered that my pro controller either
	 * reports IMU sample batches every 11ms or every 15ms. This rate is
	 * stable after connecting. It isn't 100% clear what determines this
	 * rate. Importantly, even when sending every 11ms, none of the samples
	 * are duplicates. This seems to indicate that the time deltas between
	 * reported samples can vary based on the input report rate.
	 *
	 * The solution employed in this driver is to keep track of the average
	 * time delta between IMU input reports. In testing, this value has
	 * proven to be stable, staying at 15ms or 11ms, though other hardware
	 * configurations and bluetooth stacks could potentially see other rates
	 * (hopefully this will become more clear as more people use the
	 * driver).
	 *
	 * Keeping track of the average report delta allows us to submit our
	 * timestamps to userspace based on that. Each report contains 3
	 * samples, so the IMU sampling rate should be avg_time_delta/3. We can
	 * also use this average to detect events where we have dropped a
	 * packet. The userspace timestamp for the samples will be adjusted
	 * accordingly to prevent unwanted behvaior.
	 */
	if (!ctlr->imu_first_packet_received) {
		ctlr->imu_timestamp_us = 0;
		ctlr->imu_delta_samples_count = 0;
		ctlr->imu_delta_samples_sum = 0;
		ctlr->imu_avg_delta_ms = JC_IMU_DFLT_AVG_DELTA_MS;
		ctlr->imu_first_packet_received = true;
	} else {
		unsigned int delta = msecs - last_msecs;
		unsigned int dropped_pkts;
		unsigned int dropped_threshold;

		/* avg imu report delta housekeeping */
		ctlr->imu_delta_samples_sum += delta;
		ctlr->imu_delta_samples_count++;
		if (ctlr->imu_delta_samples_count >=
		    JC_IMU_SAMPLES_PER_DELTA_AVG) {
			ctlr->imu_avg_delta_ms = ctlr->imu_delta_samples_sum /
						 ctlr->imu_delta_samples_count;
			ctlr->imu_delta_samples_count = 0;
			ctlr->imu_delta_samples_sum = 0;
		}

		/* don't ever want divide by zero shenanigans */
		if (ctlr->imu_avg_delta_ms == 0) {
			ctlr->imu_avg_delta_ms = 1;
			hid_warn(ctlr->hdev, "calculated avg imu delta of 0\n");
		}

		/* useful for debugging IMU sample rate */
		hid_dbg(ctlr->hdev,
			"imu_report: ms=%u last_ms=%u delta=%u avg_delta=%u\n",
			msecs, last_msecs, delta, ctlr->imu_avg_delta_ms);

		/* check if any packets have been dropped */
		dropped_threshold = ctlr->imu_avg_delta_ms * 3 / 2;
		dropped_pkts = (delta - min(delta, dropped_threshold)) /
				ctlr->imu_avg_delta_ms;
		if (dropped_pkts > JC_IMU_DROPPED_PKT_WARNING) {
			hid_warn_ratelimited(ctlr->hdev,
				 "compensating for %u dropped IMU reports\n",
				 dropped_pkts);
			hid_warn_ratelimited(ctlr->hdev,
				 "delta=%u avg_delta=%u\n",
				 delta, ctlr->imu_avg_delta_ms);
		}
	}
	ctlr->imu_last_pkt_ms = msecs;

	/* Each IMU input report contains three samples */
	for (i = 0; i < 3; i++) {
		hid_dbg(ctlr->hdev, "raw_gyro: g_x=%d g_y=%d g_z=%d\n",
			imu_data[i].gyro_x, imu_data[i].gyro_y,
			imu_data[i].gyro_z);
		hid_dbg(ctlr->hdev, "raw_accel: a_x=%d a_y=%d a_z=%d\n",
			imu_data[i].accel_x, imu_data[i].accel_y,
			imu_data[i].accel_z);

		joycon_report_imu_sample(idev, &ctlr->accel_cal, &ctlr->gyro_cal,
					 &imu_data[i],
					 jc_type_is_joycon(ctlr) &&
					 jc_type_has_right(ctlr),
					 ctlr->imu_timestamp_us);

		/* convert to micros and divide by 3 (3 samples per report). */
		ctlr->imu_timestamp_us += ctlr->imu_avg_delta_ms * 1000 / 3;
	}
}

static void joycon_handle_rumble_report(struct joycon_ctlr *ctlr, struct joycon_input_report *rep)
{
	unsigned long flags;
	unsigned long msecs = jiffies_to_msecs(jiffies);

	spin_lock_irqsave(&ctlr->lock, flags);
	if (IS_ENABLED(CONFIG_NINTENDO_FF) && rep->vibrator_report &&
	    ctlr->ctlr_state != JOYCON_CTLR_STATE_REMOVED &&
	    (msecs - ctlr->rumble_msecs) >= JC_RUMBLE_PERIOD_MS &&
	    (ctlr->rumble_queue_head != ctlr->rumble_queue_tail ||
	     ctlr->rumble_zero_countdown > 0)) {
		/*
		 * When this value reaches 0, we know we've sent multiple
		 * packets to the controller instructing it to disable rumble.
		 * We can safely stop sending periodic rumble packets until the
		 * next ff effect.
		 */
		if (ctlr->rumble_zero_countdown > 0)
			ctlr->rumble_zero_countdown--;
		queue_work(ctlr->rumble_queue, &ctlr->rumble_worker);
	}

	spin_unlock_irqrestore(&ctlr->lock, flags);
}

static void joycon_parse_battery_status(struct joycon_ctlr *ctlr, struct joycon_input_report *rep)
{
	u8 tmp;
	unsigned long flags;

	spin_lock_irqsave(&ctlr->lock, flags);

	tmp = rep->bat_con;
	ctlr->host_powered = tmp & BIT(0);
	ctlr->battery_charging = tmp & BIT(4);
	tmp = tmp >> 5;

	switch (tmp) {
	case 0: /* empty */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		break;
	case 1: /* low */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		break;
	case 2: /* medium */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	case 3: /* high */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		break;
	case 4: /* full */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		break;
	default:
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		hid_warn(ctlr->hdev, "Invalid battery status\n");
		break;
	}

	spin_unlock_irqrestore(&ctlr->lock, flags);
}

static void joycon_report_left_stick(struct joycon_ctlr *ctlr,
				     struct joycon_input_report *rep)
{
	u16 raw_x;
	u16 raw_y;
	s32 x;
	s32 y;

	raw_x = hid_field_extract(ctlr->hdev, rep->left_stick, 0, 12);
	raw_y = hid_field_extract(ctlr->hdev, rep->left_stick + 1, 4, 12);

	x = joycon_map_stick_val(&ctlr->left_stick_cal_x, raw_x);
	y = -joycon_map_stick_val(&ctlr->left_stick_cal_y, raw_y);

	input_report_abs(ctlr->input, ABS_X, x);
	input_report_abs(ctlr->input, ABS_Y, y);
}

static void joycon_report_right_stick(struct joycon_ctlr *ctlr,
				      struct joycon_input_report *rep)
{
	u16 raw_x;
	u16 raw_y;
	s32 x;
	s32 y;

	raw_x = hid_field_extract(ctlr->hdev, rep->right_stick, 0, 12);
	raw_y = hid_field_extract(ctlr->hdev, rep->right_stick + 1, 4, 12);

	x = joycon_map_stick_val(&ctlr->right_stick_cal_x, raw_x);
	y = -joycon_map_stick_val(&ctlr->right_stick_cal_y, raw_y);

	input_report_abs(ctlr->input, ABS_RX, x);
	input_report_abs(ctlr->input, ABS_RY, y);
}

static void joycon_report_dpad(struct joycon_ctlr *ctlr,
			       struct joycon_input_report *rep)
{
	int hatx = 0;
	int haty = 0;
	u32 btns = hid_field_extract(ctlr->hdev, rep->button_status, 0, 24);

	if (btns & JC_BTN_LEFT)
		hatx = -1;
	else if (btns & JC_BTN_RIGHT)
		hatx = 1;

	if (btns & JC_BTN_UP)
		haty = -1;
	else if (btns & JC_BTN_DOWN)
		haty = 1;

	input_report_abs(ctlr->input, ABS_HAT0X, hatx);
	input_report_abs(ctlr->input, ABS_HAT0Y, haty);
}

static void joycon_report_buttons(struct joycon_ctlr *ctlr,
				  struct joycon_input_report *rep,
				  const struct joycon_ctlr_button_mapping button_mappings[])
{
	const struct joycon_ctlr_button_mapping *button;
	u32 status = hid_field_extract(ctlr->hdev, rep->button_status, 0, 24);

	for (button = button_mappings; button->code; button++)
		input_report_key(ctlr->input, button->code, status & button->bit);
}

static void joycon_parse_report(struct joycon_ctlr *ctlr,
				struct joycon_input_report *rep)
{
	unsigned long flags;
	unsigned long msecs = jiffies_to_msecs(jiffies);
	unsigned long report_delta_ms = msecs - ctlr->last_input_report_msecs;

	if (joycon_has_rumble(ctlr))
		joycon_handle_rumble_report(ctlr, rep);

	joycon_parse_battery_status(ctlr, rep);

	if (joycon_type_is_left_joycon(ctlr)) {
		joycon_report_left_stick(ctlr, rep);
		joycon_report_buttons(ctlr, rep, left_joycon_button_mappings);
		if (!joycon_device_is_chrggrip(ctlr))
			joycon_report_buttons(ctlr, rep, left_joycon_s_button_mappings);
	} else if (joycon_type_is_right_joycon(ctlr)) {
		joycon_report_right_stick(ctlr, rep);
		joycon_report_buttons(ctlr, rep, right_joycon_button_mappings);
		if (!joycon_device_is_chrggrip(ctlr))
			joycon_report_buttons(ctlr, rep, right_joycon_s_button_mappings);
	} else if (joycon_type_is_procon(ctlr)) {
		joycon_report_left_stick(ctlr, rep);
		joycon_report_right_stick(ctlr, rep);
		joycon_report_dpad(ctlr, rep);
		if (ctlr->hdev->vendor == USB_VENDOR_ID_HORI)
			joycon_report_buttons(ctlr, rep, lic_hori_procon_button_mappings);
		else
			joycon_report_buttons(ctlr, rep, procon_button_mappings);
	} else if (joycon_type_is_any_nescon(ctlr)) {
		joycon_report_dpad(ctlr, rep);
		joycon_report_buttons(ctlr, rep, nescon_button_mappings);
	} else if (joycon_type_is_snescon(ctlr)) {
		joycon_report_dpad(ctlr, rep);
		joycon_report_buttons(ctlr, rep, snescon_button_mappings);
	} else if (joycon_type_is_gencon(ctlr)) {
		joycon_report_dpad(ctlr, rep);
		joycon_report_buttons(ctlr, rep, gencon_button_mappings);
	} else if (joycon_type_is_n64con(ctlr)) {
		joycon_report_left_stick(ctlr, rep);
		joycon_report_dpad(ctlr, rep);
		joycon_report_buttons(ctlr, rep, n64con_button_mappings);
	}

	input_sync(ctlr->input);

	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->last_input_report_msecs = msecs;
	/*
	 * Was this input report a reasonable time delta compared to the prior
	 * report? We use this information to decide when a safe time is to send
	 * rumble packets or subcommand packets.
	 */
	if (report_delta_ms >= JC_INPUT_REPORT_MIN_DELTA &&
	    report_delta_ms <= JC_INPUT_REPORT_MAX_DELTA) {
		if (ctlr->consecutive_valid_report_deltas < JC_SUBCMD_VALID_DELTA_REQ)
			ctlr->consecutive_valid_report_deltas++;
	} else {
		ctlr->consecutive_valid_report_deltas = 0;
	}
	/*
	 * Our consecutive valid report tracking is only relevant for
	 * bluetooth-connected controllers. For USB devices, we're beholden to
	 * USB's underlying polling rate anyway. Always set to the consecutive
	 * delta requirement.
	 */
	if (ctlr->hdev->bus == BUS_USB)
		ctlr->consecutive_valid_report_deltas = JC_SUBCMD_VALID_DELTA_REQ;

	spin_unlock_irqrestore(&ctlr->lock, flags);

	/*
	 * Immediately after receiving a report is the most reliable time to
	 * send a subcommand to the controller. Wake any subcommand senders
	 * waiting for a report.
	 */
	if (unlikely(mutex_is_locked(&ctlr->output_mutex))) {
		spin_lock_irqsave(&ctlr->lock, flags);
		ctlr->received_input_report = true;
		spin_unlock_irqrestore(&ctlr->lock, flags);
		wake_up(&ctlr->wait);
	}

	/* parse IMU data if present */
	if ((rep->id == JC_INPUT_IMU_DATA) && joycon_has_imu(ctlr))
		joycon_parse_imu_report(ctlr, rep);
}

static int joycon_send_rumble_data(struct joycon_ctlr *ctlr)
{
	int ret;
	unsigned long flags;
	struct joycon_rumble_output rumble_output = { 0 };

	spin_lock_irqsave(&ctlr->lock, flags);
	/*
	 * If the controller has been removed, just return ENODEV so the LED
	 * subsystem doesn't print invalid errors on removal.
	 */
	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_REMOVED) {
		spin_unlock_irqrestore(&ctlr->lock, flags);
		return -ENODEV;
	}
	memcpy(rumble_output.rumble_data,
	       ctlr->rumble_data[ctlr->rumble_queue_tail],
	       JC_RUMBLE_DATA_SIZE);
	spin_unlock_irqrestore(&ctlr->lock, flags);

	rumble_output.output_id = JC_OUTPUT_RUMBLE_ONLY;
	rumble_output.packet_num = ctlr->subcmd_num;
	if (++ctlr->subcmd_num > 0xF)
		ctlr->subcmd_num = 0;

	joycon_enforce_subcmd_rate(ctlr);

	ret = __joycon_hid_send(ctlr->hdev, (u8 *)&rumble_output,
				sizeof(rumble_output));
	return ret;
}

static void joycon_rumble_worker(struct work_struct *work)
{
	struct joycon_ctlr *ctlr = container_of(work, struct joycon_ctlr,
							rumble_worker);
	unsigned long flags;
	bool again = true;
	int ret;

	while (again) {
		mutex_lock(&ctlr->output_mutex);
		ret = joycon_send_rumble_data(ctlr);
		mutex_unlock(&ctlr->output_mutex);

		/* -ENODEV means the controller was just unplugged */
		spin_lock_irqsave(&ctlr->lock, flags);
		if (ret < 0 && ret != -ENODEV &&
		    ctlr->ctlr_state != JOYCON_CTLR_STATE_REMOVED)
			hid_warn(ctlr->hdev, "Failed to set rumble; e=%d", ret);

		ctlr->rumble_msecs = jiffies_to_msecs(jiffies);
		if (ctlr->rumble_queue_tail != ctlr->rumble_queue_head) {
			if (++ctlr->rumble_queue_tail >= JC_RUMBLE_QUEUE_SIZE)
				ctlr->rumble_queue_tail = 0;
		} else {
			again = false;
		}
		spin_unlock_irqrestore(&ctlr->lock, flags);
	}
}

#if IS_ENABLED(CONFIG_NINTENDO_FF)
static const u16 JOYCON_MAX_RUMBLE_HIGH_FREQ	= 1253;
static const u16 JOYCON_MIN_RUMBLE_HIGH_FREQ	= 82;
static const u16 JOYCON_MAX_RUMBLE_LOW_FREQ	= 626;
static const u16 JOYCON_MIN_RUMBLE_LOW_FREQ	= 41;

static void joycon_clamp_rumble_freqs(struct joycon_ctlr *ctlr)
{
	unsigned long flags;

	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->rumble_ll_freq = clamp(ctlr->rumble_ll_freq,
				     JOYCON_MIN_RUMBLE_LOW_FREQ,
				     JOYCON_MAX_RUMBLE_LOW_FREQ);
	ctlr->rumble_lh_freq = clamp(ctlr->rumble_lh_freq,
				     JOYCON_MIN_RUMBLE_HIGH_FREQ,
				     JOYCON_MAX_RUMBLE_HIGH_FREQ);
	ctlr->rumble_rl_freq = clamp(ctlr->rumble_rl_freq,
				     JOYCON_MIN_RUMBLE_LOW_FREQ,
				     JOYCON_MAX_RUMBLE_LOW_FREQ);
	ctlr->rumble_rh_freq = clamp(ctlr->rumble_rh_freq,
				     JOYCON_MIN_RUMBLE_HIGH_FREQ,
				     JOYCON_MAX_RUMBLE_HIGH_FREQ);
	spin_unlock_irqrestore(&ctlr->lock, flags);
}

static int joycon_set_rumble(struct joycon_ctlr *ctlr, u16 amp_r, u16 amp_l,
			     bool schedule_now)
{
	u8 data[JC_RUMBLE_DATA_SIZE];
	u16 amp;
	u16 freq_r_low;
	u16 freq_r_high;
	u16 freq_l_low;
	u16 freq_l_high;
	unsigned long flags;
	int next_rq_head;

	spin_lock_irqsave(&ctlr->lock, flags);
	freq_r_low = ctlr->rumble_rl_freq;
	freq_r_high = ctlr->rumble_rh_freq;
	freq_l_low = ctlr->rumble_ll_freq;
	freq_l_high = ctlr->rumble_lh_freq;
	/* limit number of silent rumble packets to reduce traffic */
	if (amp_l != 0 || amp_r != 0)
		ctlr->rumble_zero_countdown = JC_RUMBLE_ZERO_AMP_PKT_CNT;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	/* right joy-con */
	amp = amp_r * (u32)joycon_max_rumble_amp / 65535;
	joycon_encode_rumble(data + 4, freq_r_low, freq_r_high, amp);

	/* left joy-con */
	amp = amp_l * (u32)joycon_max_rumble_amp / 65535;
	joycon_encode_rumble(data, freq_l_low, freq_l_high, amp);

	spin_lock_irqsave(&ctlr->lock, flags);

	next_rq_head = ctlr->rumble_queue_head + 1;
	if (next_rq_head >= JC_RUMBLE_QUEUE_SIZE)
		next_rq_head = 0;

	/* Did we overrun the circular buffer?
	 * If so, be sure we keep the latest intended rumble state.
	 */
	if (next_rq_head == ctlr->rumble_queue_tail) {
		hid_dbg(ctlr->hdev, "rumble queue is full");
		/* overwrite the prior value at the end of the circular buf */
		next_rq_head = ctlr->rumble_queue_head;
	}

	ctlr->rumble_queue_head = next_rq_head;
	memcpy(ctlr->rumble_data[ctlr->rumble_queue_head], data,
	       JC_RUMBLE_DATA_SIZE);

	/* don't wait for the periodic send (reduces latency) */
	if (schedule_now && ctlr->ctlr_state != JOYCON_CTLR_STATE_REMOVED)
		queue_work(ctlr->rumble_queue, &ctlr->rumble_worker);

	spin_unlock_irqrestore(&ctlr->lock, flags);

	return 0;
}

static int joycon_play_effect(struct input_dev *dev, void *data,
						     struct ff_effect *effect)
{
	struct joycon_ctlr *ctlr = input_get_drvdata(dev);

	if (effect->type != FF_RUMBLE)
		return 0;

	return joycon_set_rumble(ctlr,
				 effect->u.rumble.weak_magnitude,
				 effect->u.rumble.strong_magnitude,
				 true);
}
#endif /* IS_ENABLED(CONFIG_NINTENDO_FF) */

static void joycon_config_dpad(struct input_dev *idev)
{
	input_set_abs_params(idev,
			     ABS_HAT0X,
			     -JC_MAX_DPAD_MAG,
			     JC_MAX_DPAD_MAG,
			     JC_DPAD_FUZZ,
			     JC_DPAD_FLAT);
	input_set_abs_params(idev,
			     ABS_HAT0Y,
			     -JC_MAX_DPAD_MAG,
			     JC_MAX_DPAD_MAG,
			     JC_DPAD_FUZZ,
			     JC_DPAD_FLAT);
}

static void joycon_config_rumble(struct joycon_ctlr *ctlr)
{
#if IS_ENABLED(CONFIG_NINTENDO_FF)
	/* set up rumble */
	input_set_capability(ctlr->input, EV_FF, FF_RUMBLE);
	input_ff_create_memless(ctlr->input, NULL, joycon_play_effect);
	ctlr->rumble_ll_freq = JC_RUMBLE_DFLT_LOW_FREQ;
	ctlr->rumble_lh_freq = JC_RUMBLE_DFLT_HIGH_FREQ;
	ctlr->rumble_rl_freq = JC_RUMBLE_DFLT_LOW_FREQ;
	ctlr->rumble_rh_freq = JC_RUMBLE_DFLT_HIGH_FREQ;
	joycon_clamp_rumble_freqs(ctlr);
	joycon_set_rumble(ctlr, 0, 0, false);
	ctlr->rumble_msecs = jiffies_to_msecs(jiffies);
#endif
}

static int joycon_imu_input_create(struct joycon_ctlr *ctlr)
{
	struct hid_device *hdev;
	const char *imu_name;
	int ret;

	hdev = ctlr->hdev;

	/* configure the imu input device */
	ctlr->imu_input = devm_input_allocate_device(&hdev->dev);
	if (!ctlr->imu_input)
		return -ENOMEM;

	ctlr->imu_input->id.bustype = hdev->bus;
	ctlr->imu_input->id.vendor = hdev->vendor;
	ctlr->imu_input->id.product = hdev->product;
	ctlr->imu_input->id.version = hdev->version;
	ctlr->imu_input->uniq = ctlr->mac_addr_str;
	ctlr->imu_input->phys = hdev->phys;

	imu_name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s (IMU)", ctlr->input->name);
	if (!imu_name)
		return -ENOMEM;

	ctlr->imu_input->name = imu_name;

	input_set_drvdata(ctlr->imu_input, ctlr);

	joycon_config_imu(ctlr->imu_input);

	ret = input_register_device(ctlr->imu_input);
	if (ret)
		return ret;

	return 0;
}

static int joycon_input_create(struct joycon_ctlr *ctlr)
{
	struct hid_device *hdev;
	int ret;

	hdev = ctlr->hdev;

	ctlr->input = devm_input_allocate_device(&hdev->dev);
	if (!ctlr->input)
		return -ENOMEM;
	ctlr->input->id.bustype = hdev->bus;
	ctlr->input->id.vendor = hdev->vendor;
	ctlr->input->id.product = hdev->product;
	ctlr->input->id.version = hdev->version;
	ctlr->input->uniq = ctlr->mac_addr_str;
	ctlr->input->name = hdev->name;
	ctlr->input->phys = hdev->phys;
	input_set_drvdata(ctlr->input, ctlr);

	if (joycon_type_is_right_joycon(ctlr)) {
		joycon_config_right_stick(ctlr->input);
		joycon_config_buttons(ctlr->input, right_joycon_button_mappings);
		if (!joycon_device_is_chrggrip(ctlr))
			joycon_config_buttons(ctlr->input, right_joycon_s_button_mappings);
	} else if (joycon_type_is_left_joycon(ctlr)) {
		joycon_config_left_stick(ctlr->input);
		joycon_config_buttons(ctlr->input, left_joycon_button_mappings);
		if (!joycon_device_is_chrggrip(ctlr))
			joycon_config_buttons(ctlr->input, left_joycon_s_button_mappings);
	} else if (joycon_type_is_procon(ctlr)) {
		joycon_config_left_stick(ctlr->input);
		joycon_config_right_stick(ctlr->input);
		joycon_config_dpad(ctlr->input);
		if (ctlr->hdev->vendor == USB_VENDOR_ID_HORI)
			joycon_config_buttons(ctlr->input, lic_hori_procon_button_mappings);
		else
			joycon_config_buttons(ctlr->input, procon_button_mappings);
	} else if (joycon_type_is_any_nescon(ctlr)) {
		joycon_config_dpad(ctlr->input);
		joycon_config_buttons(ctlr->input, nescon_button_mappings);
	} else if (joycon_type_is_snescon(ctlr)) {
		joycon_config_dpad(ctlr->input);
		joycon_config_buttons(ctlr->input, snescon_button_mappings);
	} else if (joycon_type_is_gencon(ctlr)) {
		joycon_config_dpad(ctlr->input);
		joycon_config_buttons(ctlr->input, gencon_button_mappings);
	} else if (joycon_type_is_n64con(ctlr)) {
		joycon_config_dpad(ctlr->input);
		joycon_config_left_stick(ctlr->input);
		joycon_config_buttons(ctlr->input, n64con_button_mappings);
	}

	if (joycon_has_imu(ctlr)) {
		ret = joycon_imu_input_create(ctlr);
		if (ret)
			return ret;
	}

	if (joycon_has_rumble(ctlr))
		joycon_config_rumble(ctlr);

	ret = input_register_device(ctlr->input);
	if (ret)
		return ret;

	return 0;
}

/* Because the subcommand sets all the leds at once, the brightness argument is ignored */
static int joycon_player_led_brightness_set(struct led_classdev *led,
					    enum led_brightness brightness)
{
	struct device *dev = led->dev->parent;
	struct hid_device *hdev = to_hid_device(dev);
	struct joycon_ctlr *ctlr;
	int val = 0;
	int i;
	int ret;

	ctlr = hid_get_drvdata(hdev);
	if (!ctlr) {
		hid_err(hdev, "No controller data\n");
		return -ENODEV;
	}

	for (i = 0; i < JC_NUM_LEDS; i++)
		val |= ctlr->leds[i].brightness << i;

	mutex_lock(&ctlr->output_mutex);
	ret = joycon_set_player_leds(ctlr, 0, val);
	mutex_unlock(&ctlr->output_mutex);

	return ret;
}

static int joycon_home_led_brightness_set(struct led_classdev *led,
					  enum led_brightness brightness)
{
	struct device *dev = led->dev->parent;
	struct hid_device *hdev = to_hid_device(dev);
	struct joycon_ctlr *ctlr;
	int ret;

	ctlr = hid_get_drvdata(hdev);
	if (!ctlr) {
		hid_err(hdev, "No controller data\n");
		return -ENODEV;
	}
	mutex_lock(&ctlr->output_mutex);
	ret = joycon_set_home_led(ctlr, brightness);
	mutex_unlock(&ctlr->output_mutex);
	return ret;
}

static DEFINE_IDA(nintendo_player_id_allocator);

static int joycon_leds_create(struct joycon_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	struct device *dev = &hdev->dev;
	const char *d_name = dev_name(dev);
	struct led_classdev *led;
	int led_val = 0;
	char *name;
	int ret;
	int i;
	int player_led_pattern;

	/* configure the player LEDs */
	ctlr->player_id = U32_MAX;
	ret = ida_alloc(&nintendo_player_id_allocator, GFP_KERNEL);
	if (ret < 0) {
		hid_warn(hdev, "Failed to allocate player ID, skipping; ret=%d\n", ret);
		goto home_led;
	}
	ctlr->player_id = ret;
	player_led_pattern = ret % JC_NUM_LED_PATTERNS;
	hid_info(ctlr->hdev, "assigned player %d led pattern", player_led_pattern + 1);

	for (i = 0; i < JC_NUM_LEDS; i++) {
		name = devm_kasprintf(dev, GFP_KERNEL, "%s:%s:%s",
				      d_name,
				      "green",
				      joycon_player_led_names[i]);
		if (!name)
			return -ENOMEM;

		led = &ctlr->leds[i];
		led->name = name;
		led->brightness = joycon_player_led_patterns[player_led_pattern][i];
		led->max_brightness = 1;
		led->brightness_set_blocking =
					joycon_player_led_brightness_set;
		led->flags = LED_CORE_SUSPENDRESUME | LED_HW_PLUGGABLE;

		led_val |= joycon_player_led_patterns[player_led_pattern][i] << i;
	}
	mutex_lock(&ctlr->output_mutex);
	ret = joycon_set_player_leds(ctlr, 0, led_val);
	mutex_unlock(&ctlr->output_mutex);
	if (ret) {
		hid_warn(hdev, "Failed to set players LEDs, skipping registration; ret=%d\n", ret);
		goto home_led;
	}

	for (i = 0; i < JC_NUM_LEDS; i++) {
		led = &ctlr->leds[i];
		ret = devm_led_classdev_register(&hdev->dev, led);
		if (ret) {
			hid_err(hdev, "Failed to register player %d LED; ret=%d\n", i + 1, ret);
			return ret;
		}
	}

home_led:
	/* configure the home LED */
	if (jc_type_has_right(ctlr)) {
		name = devm_kasprintf(dev, GFP_KERNEL, "%s:%s:%s",
				      d_name,
				      "blue",
				      LED_FUNCTION_PLAYER5);
		if (!name)
			return -ENOMEM;

		led = &ctlr->home_led;
		led->name = name;
		led->brightness = 0;
		led->max_brightness = 0xF;
		led->brightness_set_blocking = joycon_home_led_brightness_set;
		led->flags = LED_CORE_SUSPENDRESUME | LED_HW_PLUGGABLE;

		/* Set the home LED to 0 as default state */
		mutex_lock(&ctlr->output_mutex);
		ret = joycon_set_home_led(ctlr, 0);
		mutex_unlock(&ctlr->output_mutex);
		if (ret) {
			hid_warn(hdev, "Failed to set home LED, skipping registration; ret=%d\n", ret);
			return 0;
		}

		ret = devm_led_classdev_register(&hdev->dev, led);
		if (ret) {
			hid_err(hdev, "Failed to register home LED; ret=%d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int joycon_battery_get_property(struct power_supply *supply,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct joycon_ctlr *ctlr = power_supply_get_drvdata(supply);
	unsigned long flags;
	int ret = 0;
	u8 capacity;
	bool charging;
	bool powered;

	spin_lock_irqsave(&ctlr->lock, flags);
	capacity = ctlr->battery_capacity;
	charging = ctlr->battery_charging;
	powered = ctlr->host_powered;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = capacity;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (charging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (capacity == POWER_SUPPLY_CAPACITY_LEVEL_FULL &&
			 powered)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property joycon_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_STATUS,
};

static int joycon_power_supply_create(struct joycon_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	struct power_supply_config supply_config = { .drv_data = ctlr, };
	const char * const name_fmt = "nintendo_switch_controller_battery_%s";
	int ret = 0;

	/* Set initially to unknown before receiving first input report */
	ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

	/* Configure the battery's description */
	ctlr->battery_desc.properties = joycon_battery_props;
	ctlr->battery_desc.num_properties =
					ARRAY_SIZE(joycon_battery_props);
	ctlr->battery_desc.get_property = joycon_battery_get_property;
	ctlr->battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	ctlr->battery_desc.use_for_apm = 0;
	ctlr->battery_desc.name = devm_kasprintf(&hdev->dev, GFP_KERNEL,
						 name_fmt,
						 dev_name(&hdev->dev));
	if (!ctlr->battery_desc.name)
		return -ENOMEM;

	ctlr->battery = devm_power_supply_register(&hdev->dev,
						   &ctlr->battery_desc,
						   &supply_config);
	if (IS_ERR(ctlr->battery)) {
		ret = PTR_ERR(ctlr->battery);
		hid_err(hdev, "Failed to register battery; ret=%d\n", ret);
		return ret;
	}

	return power_supply_powers(ctlr->battery, &hdev->dev);
}

static int joycon_read_info(struct joycon_ctlr *ctlr)
{
	int ret;
	int i;
	int j;
	struct joycon_subcmd_request req = { 0 };
	struct joycon_input_report *report;

	req.subcmd_id = JC_SUBCMD_REQ_DEV_INFO;
	ret = joycon_send_subcmd(ctlr, &req, 0, 2 * HZ);
	if (ret) {
		hid_err(ctlr->hdev, "Failed to get joycon info; ret=%d\n", ret);
		return ret;
	}

	report = (struct joycon_input_report *)ctlr->input_buf;

	for (i = 4, j = 0; j < 6; i++, j++)
		ctlr->mac_addr[j] = report->subcmd_reply.data[i];

	ctlr->mac_addr_str = devm_kasprintf(&ctlr->hdev->dev, GFP_KERNEL, "%pMU",
					    ctlr->mac_addr);
	if (!ctlr->mac_addr_str)
		return -ENOMEM;
	hid_info(ctlr->hdev, "controller MAC = %s\n", ctlr->mac_addr_str);

	/*
	 * Retrieve the type so we can distinguish the controller type
	 * Unfortantly the hdev->product can't always be used due to a ?bug?
	 * with the NSO Genesis controller. Over USB, it will report the
	 * PID as 0x201E, but over bluetooth it will report the PID as 0x2017
	 * which is the same as the NSO SNES controller. This is different from
	 * the rest of the controllers which will report the same PID over USB
	 * and bluetooth.
	 */
	ctlr->ctlr_type = report->subcmd_reply.data[2];
	hid_dbg(ctlr->hdev, "controller type = 0x%02X\n", ctlr->ctlr_type);

	return 0;
}

static int joycon_init(struct hid_device *hdev)
{
	struct joycon_ctlr *ctlr = hid_get_drvdata(hdev);
	int ret = 0;

	mutex_lock(&ctlr->output_mutex);
	/* if handshake command fails, assume ble pro controller */
	if (joycon_using_usb(ctlr) && !joycon_send_usb(ctlr, JC_USB_CMD_HANDSHAKE, HZ)) {
		hid_dbg(hdev, "detected USB controller\n");
		/* set baudrate for improved latency */
		ret = joycon_send_usb(ctlr, JC_USB_CMD_BAUDRATE_3M, HZ);
		if (ret) {
			/*
			 * We can function with the default baudrate.
			 * Provide a warning, and continue on.
			 */
			hid_warn(hdev, "Failed to set baudrate (ret=%d), continuing anyway\n", ret);
		}
		/* handshake */
		ret = joycon_send_usb(ctlr, JC_USB_CMD_HANDSHAKE, HZ);
		if (ret) {
			hid_err(hdev, "Failed handshake; ret=%d\n", ret);
			goto out_unlock;
		}
		/*
		 * Set no timeout (to keep controller in USB mode).
		 * This doesn't send a response, so ignore the timeout.
		 */
		joycon_send_usb(ctlr, JC_USB_CMD_NO_TIMEOUT, HZ/10);
	} else if (jc_type_is_chrggrip(ctlr)) {
		hid_err(hdev, "Failed charging grip handshake\n");
		ret = -ETIMEDOUT;
		goto out_unlock;
	}

	/* needed to retrieve the controller type */
	ret = joycon_read_info(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to retrieve controller info; ret=%d\n",
			ret);
		goto out_unlock;
	}

	if (joycon_has_joysticks(ctlr)) {
		/* get controller calibration data, and parse it */
		if (ctlr->ctlr_type == JOYCON_CTLR_TYPE_LIC_PRO) {
			/*
			 * Licensed controllers may have incompatible SPI flash
			 * layouts. Use default calibration values.
			 */
			hid_info(hdev, "using default cal for licensed controller\n");
			joycon_use_default_calibration(hdev,
						       &ctlr->left_stick_cal_x,
						       &ctlr->left_stick_cal_y,
						       "left", 0);
			joycon_use_default_calibration(hdev,
						       &ctlr->right_stick_cal_x,
						       &ctlr->right_stick_cal_y,
						       "right", 0);
		} else {
			ret = joycon_request_calibration(ctlr);
			if (ret) {
				/*
				 * We can function with default calibration, but
				 * it may be inaccurate. Provide a warning, and
				 * continue on.
				 */
				hid_warn(hdev, "Analog stick positions may be inaccurate\n");
			}
		}
	}

	if (joycon_has_imu(ctlr)) {
		/* get IMU calibration data, and parse it */
		ret = joycon_request_imu_calibration(ctlr);
		if (ret) {
			/*
			 * We can function with default calibration, but it may be
			 * inaccurate. Provide a warning, and continue on.
			 */
			hid_warn(hdev, "Unable to read IMU calibration data\n");
		}

		/* Enable the IMU */
		ret = joycon_enable_imu(ctlr);
		if (ret) {
			if (ctlr->ctlr_type == JOYCON_CTLR_TYPE_LIC_PRO) {
				hid_dbg(hdev, "IMU enable failed for licensed controller, continuing\n");
				ret = 0;
			} else {
				hid_err(hdev, "Failed to enable the IMU; ret=%d\n", ret);
				goto out_unlock;
			}
		}
	}

	/* Set the reporting mode to 0x30, which is the full report mode */
	ret = joycon_set_report_mode(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to set report mode; ret=%d\n", ret);
		goto out_unlock;
	}

	if (joycon_has_rumble(ctlr)) {
		/* Enable rumble */
		ret = joycon_enable_rumble(ctlr);
		if (ret) {
			if (ctlr->ctlr_type == JOYCON_CTLR_TYPE_LIC_PRO) {
				hid_dbg(hdev, "rumble enable failed for licensed controller, continuing\n");
				ret = 0;
			} else {
				hid_err(hdev, "Failed to enable rumble; ret=%d\n", ret);
				goto out_unlock;
			}
		}
	}

out_unlock:
	mutex_unlock(&ctlr->output_mutex);
	return ret;
}

/* Common handler for parsing inputs */
static int joycon_ctlr_read_handler(struct joycon_ctlr *ctlr, u8 *data,
							      int size)
{
	if (data[0] == JC_INPUT_SUBCMD_REPLY || data[0] == JC_INPUT_IMU_DATA ||
	    data[0] == JC_INPUT_MCU_DATA) {
		/*
		 * The whole struct is cast and parsed below, including the
		 * IMU/subcmd union, not just the 12-byte partial header this
		 * used to check for.
		 */
		if (size >= sizeof(struct joycon_input_report))
			joycon_parse_report(ctlr,
					    (struct joycon_input_report *)data);
	}

	return 0;
}

static int joycon_ctlr_handle_event(struct joycon_ctlr *ctlr, u8 *data,
							      int size)
{
	int ret = 0;
	bool match = false;
	struct joycon_input_report *report;

	if (unlikely(mutex_is_locked(&ctlr->output_mutex)) &&
	    ctlr->msg_type != JOYCON_MSG_TYPE_NONE) {
		switch (ctlr->msg_type) {
		case JOYCON_MSG_TYPE_USB:
			if (size < 2)
				break;
			if (data[0] == JC_INPUT_USB_RESPONSE &&
			    data[1] == ctlr->usb_ack_match)
				match = true;
			break;
		case JOYCON_MSG_TYPE_SUBCMD:
			if (size < sizeof(struct joycon_input_report) ||
			    data[0] != JC_INPUT_SUBCMD_REPLY)
				break;
			report = (struct joycon_input_report *)data;
			if (report->subcmd_reply.id == ctlr->subcmd_ack_match)
				match = true;
			break;
		default:
			break;
		}

		if (match) {
			memcpy(ctlr->input_buf, data,
			       min(size, (int)JC_MAX_RESP_SIZE));
			ctlr->msg_type = JOYCON_MSG_TYPE_NONE;
			ctlr->received_resp = true;
			wake_up(&ctlr->wait);

			/* This message has been handled */
			return 1;
		}
	}

	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_READ)
		ret = joycon_ctlr_read_handler(ctlr, data, size);

	return ret;
}

static int joycon_event(struct hid_device *hdev,
			      struct hid_report *report, u8 *raw_data, int size)
{
	struct joycon_ctlr *ctlr = hid_get_drvdata(hdev);

	if (size < 1)
		return -EINVAL;

	return joycon_ctlr_handle_event(ctlr, raw_data, size);
}

static int joycon_probe(struct hid_device *hdev,
			    const struct hid_device_id *id)
{
	int ret;
	struct joycon_ctlr *ctlr;

	hid_dbg(hdev, "probe - start\n");

	ctlr = devm_kzalloc(&hdev->dev, sizeof(*ctlr), GFP_KERNEL);
	if (!ctlr) {
		ret = -ENOMEM;
		goto err;
	}

	ctlr->hdev = hdev;
	ctlr->ctlr_state = JOYCON_CTLR_STATE_INIT;
	ctlr->rumble_queue_head = 0;
	ctlr->rumble_queue_tail = 0;
	hid_set_drvdata(hdev, ctlr);
	mutex_init(&ctlr->output_mutex);
	init_waitqueue_head(&ctlr->wait);
	spin_lock_init(&ctlr->lock);
	ctlr->rumble_queue = alloc_workqueue("hid-nintendo-rumble_wq",
					     WQ_FREEZABLE | WQ_MEM_RECLAIM | WQ_PERCPU,
					     0);
	if (!ctlr->rumble_queue) {
		ret = -ENOMEM;
		goto err;
	}
	INIT_WORK(&ctlr->rumble_worker, joycon_rumble_worker);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "HID parse failed\n");
		goto err_wq;
	}

	/*
	 * Patch the hw version of pro controller/joycons, so applications can
	 * distinguish between the default HID mappings and the mappings defined
	 * by the Linux game controller spec. This is important for the SDL2
	 * library, which has a game controller database, which uses device ids
	 * in combination with version as a key.
	 */
	hdev->version |= 0x8000;

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "HW start failed\n");
		goto err_wq;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "cannot start hardware I/O\n");
		goto err_stop;
	}

	hid_device_io_start(hdev);

	ret = joycon_init(hdev);
	if (ret) {
		hid_err(hdev, "Failed to initialize controller; ret=%d\n", ret);
		goto err_io_stop;
	}

	/* Initialize the leds */
	ret = joycon_leds_create(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to create leds; ret=%d\n", ret);
		goto err_io_stop;
	}

	/* Initialize the battery power supply */
	ret = joycon_power_supply_create(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to create power_supply; ret=%d\n", ret);
		goto err_ida;
	}

	ret = joycon_input_create(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to create input device; ret=%d\n", ret);
		goto err_ida;
	}

	ctlr->ctlr_state = JOYCON_CTLR_STATE_READ;

	hid_dbg(hdev, "probe - success\n");
	return 0;

err_ida:
	ida_free(&nintendo_player_id_allocator, ctlr->player_id);
err_io_stop:
	hid_device_io_stop(hdev);
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
err_wq:
	destroy_workqueue(ctlr->rumble_queue);
err:
	hid_err(hdev, "probe - fail = %d\n", ret);
	return ret;
}

static void joycon_remove(struct hid_device *hdev)
{
	struct joycon_ctlr *ctlr = hid_get_drvdata(hdev);
	unsigned long flags;

	hid_dbg(hdev, "remove\n");

	/* Prevent further attempts at sending subcommands. */
	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->ctlr_state = JOYCON_CTLR_STATE_REMOVED;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	destroy_workqueue(ctlr->rumble_queue);
	ida_free(&nintendo_player_id_allocator, ctlr->player_id);

	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static int joycon_resume(struct hid_device *hdev)
{
	struct joycon_ctlr *ctlr = hid_get_drvdata(hdev);
	int ret;

	hid_dbg(hdev, "resume\n");
	if (!joycon_using_usb(ctlr)) {
		hid_dbg(hdev, "no-op resume for bt ctlr\n");
		ctlr->ctlr_state = JOYCON_CTLR_STATE_READ;
		return 0;
	}

	ret = joycon_init(hdev);
	if (ret)
		hid_err(hdev,
			"Failed to restore controller after resume: %d\n",
			ret);
	else
		ctlr->ctlr_state = JOYCON_CTLR_STATE_READ;

	return ret;
}

static int joycon_suspend(struct hid_device *hdev, pm_message_t message)
{
	struct joycon_ctlr *ctlr = hid_get_drvdata(hdev);

	hid_dbg(hdev, "suspend: %d\n", message.event);
	/*
	 * Avoid any blocking loops in suspend/resume transitions.
	 *
	 * joycon_enforce_subcmd_rate() can result in repeated retries if for
	 * whatever reason the controller stops providing input reports.
	 *
	 * This has been observed with bluetooth controllers which lose
	 * connectivity prior to suspend (but not long enough to result in
	 * complete disconnection).
	 */
	ctlr->ctlr_state = JOYCON_CTLR_STATE_SUSPENDED;
	return 0;
}

/*
 * =============================================================================
 * Switch 2 support
 * =============================================================================
 */
#define NS2_BTNR_B	BIT(0)
#define NS2_BTNR_A	BIT(1)
#define NS2_BTNR_Y	BIT(2)
#define NS2_BTNR_X	BIT(3)
#define NS2_BTNR_R	BIT(4)
#define NS2_BTNR_ZR	BIT(5)
#define NS2_BTNR_PLUS	BIT(6)
#define NS2_BTNR_RS	BIT(7)

#define NS2_BTNL_DOWN	BIT(0)
#define NS2_BTNL_RIGHT	BIT(1)
#define NS2_BTNL_LEFT	BIT(2)
#define NS2_BTNL_UP	BIT(3)
#define NS2_BTNL_L	BIT(4)
#define NS2_BTNL_ZL	BIT(5)
#define NS2_BTNL_MINUS	BIT(6)
#define NS2_BTNL_LS	BIT(7)

#define NS2_BTN3_C	BIT(4)
#define NS2_BTN3_SR	BIT(6)
#define NS2_BTN3_SL	BIT(7)

#define NS2_BTN_U1_Y	BIT(0)
#define NS2_BTN_U1_X	BIT(1)
#define NS2_BTN_U1_B	BIT(2)
#define NS2_BTN_U1_A	BIT(3)
#define NS2_BTN_U1_SR	BIT(4)
#define NS2_BTN_U1_SL	BIT(5)
#define NS2_BTN_U1_R	BIT(6)
#define NS2_BTN_U1_ZR	BIT(7)

#define NS2_BTN_U2_MINUS	BIT(0)
#define NS2_BTN_U2_PLUS		BIT(1)
#define NS2_BTN_U2_RS		BIT(2)
#define NS2_BTN_U2_LS		BIT(3)
#define NS2_BTN_U2_HOME		BIT(4)
#define NS2_BTN_U2_CAPTURE	BIT(5)
#define NS2_BTN_U2_C		BIT(6)

#define NS2_BTN_U3_DOWN		BIT(0)
#define NS2_BTN_U3_UP		BIT(1)
#define NS2_BTN_U3_RIGHT	BIT(2)
#define NS2_BTN_U3_LEFT		BIT(3)
#define NS2_BTN_U3_SR		BIT(4)
#define NS2_BTN_U3_SL		BIT(5)
#define NS2_BTN_U3_L		BIT(6)
#define NS2_BTN_U3_ZL		BIT(7)

#define NS2_BTN_U4_GR		BIT(0)
#define NS2_BTN_U4_GL		BIT(1)
#define NS2_BTN_U4_HEADSET	BIT(5)

#define NS2_BTN_JCR_HOME	BIT(0)
#define NS2_BTN_JCR_GR		BIT(2)
#define NS2_BTN_JCR_C		NS2_BTN3_C
#define NS2_BTN_JCR_SR		NS2_BTN3_SR
#define NS2_BTN_JCR_SL		NS2_BTN3_SL

#define NS2_BTN_JCL_CAPTURE	BIT(0)
#define NS2_BTN_JCL_GL		BIT(2)
#define NS2_BTN_JCL_SR		NS2_BTN3_SR
#define NS2_BTN_JCL_SL		NS2_BTN3_SL

#define NS2_BTN_PRO_HOME	BIT(0)
#define NS2_BTN_PRO_CAPTURE	BIT(1)
#define NS2_BTN_PRO_GR		BIT(2)
#define NS2_BTN_PRO_GL		BIT(3)
#define NS2_BTN_PRO_C		NS2_BTN3_C

#define NS2_BTN_GC_HOME		BIT(0)
#define NS2_BTN_GC_CAPTURE	BIT(1)
#define NS2_BTN_GC_C		NS2_BTN3_C

#define NS2_TRIGGER_RANGE	4095
#define NS2_AXIS_MIN		-32768
#define NS2_AXIS_MAX		32767

#define NS2_MAX_PLAYER_ID	8

#define NS2_MAX_INIT_RETRIES	4

#define NS2_FLASH_ADDR_SERIAL			0x13002
#define NS2_FLASH_ADDR_FACTORY_PRIMARY_CALIB	0x130a8
#define NS2_FLASH_ADDR_FACTORY_SECONDARY_CALIB	0x130e8
#define NS2_FLASH_ADDR_FACTORY_TRIGGER_CALIB	0x13140
#define NS2_FLASH_ADDR_USER_PRIMARY_CALIB	0x1fc040
#define NS2_FLASH_ADDR_USER_SECONDARY_CALIB	0x1fc080

#define NS2_FLASH_SIZE_SERIAL 0x10
#define NS2_FLASH_SIZE_FACTORY_AXIS_CALIB 9
#define NS2_FLASH_SIZE_FACTORY_TRIGGER_CALIB 2
#define NS2_FLASH_SIZE_USER_AXIS_CALIB 11

#define NS2_USER_CALIB_MAGIC 0xa1b2

#define NS2_FEATURE_BUTTONS	BIT(0)
#define NS2_FEATURE_ANALOG	BIT(1)
#define NS2_FEATURE_IMU		BIT(2)
#define NS2_FEATURE_MOUSE	BIT(4)
#define NS2_FEATURE_RUMBLE	BIT(5)
#define NS2_FEATURE_MAGNETO	BIT(7)

enum switch2_subcmd_flash {
	NS2_SUBCMD_FLASH_READ_BLOCK = 0x01,
	NS2_SUBCMD_FLASH_WRITE_BLOCK = 0x02,
	NS2_SUBCMD_FLASH_ERASE_BLOCK = 0x03,
	NS2_SUBCMD_FLASH_READ = 0x04,
	NS2_SUBCMD_FLASH_WRITE = 0x05,
};

enum switch2_subcmd_init {
	NS2_SUBCMD_INIT_SELECT_REPORT = 0xa,
	NS2_SUBCMD_INIT_USB = 0xd,
};

enum switch2_subcmd_feature_select {
	NS2_SUBCMD_FEATSEL_GET_INFO = 0x1,
	NS2_SUBCMD_FEATSEL_SET_MASK = 0x2,
	NS2_SUBCMD_FEATSEL_CLEAR_MASK = 0x3,
	NS2_SUBCMD_FEATSEL_ENABLE = 0x4,
	NS2_SUBCMD_FEATSEL_DISABLE = 0x5,
};

enum switch2_subcmd_grip {
	NS2_SUBCMD_GRIP_GET_INFO = 0x1,
	NS2_SUBCMD_GRIP_ENABLE_BUTTONS = 0x2,
	NS2_SUBCMD_GRIP_GET_INFO_EXT = 0x3,
};

enum switch2_subcmd_led {
	NS2_SUBCMD_LED_P1 = 0x1,
	NS2_SUBCMD_LED_P2 = 0x2,
	NS2_SUBCMD_LED_P3 = 0x3,
	NS2_SUBCMD_LED_P4 = 0x4,
	NS2_SUBCMD_LED_ALL_ON = 0x5,
	NS2_SUBCMD_LED_ALL_OFF = 0x6,
	NS2_SUBCMD_LED_PATTERN = 0x7,
	NS2_SUBCMD_LED_BLINK = 0x8,
};

enum switch2_subcmd_fw_info {
	NS2_SUBCMD_FW_INFO_GET = 0x1,
};

enum switch2_ctlr_type {
	NS2_CTLR_TYPE_JCL = 0x00,
	NS2_CTLR_TYPE_JCR = 0x01,
	NS2_CTLR_TYPE_PRO = 0x02,
	NS2_CTLR_TYPE_GC = 0x03,
};

enum switch2_report_id {
	NS2_REPORT_UNIFIED = 0x05,
	NS2_REPORT_JCL = 0x07,
	NS2_REPORT_JCR = 0x08,
	NS2_REPORT_PRO = 0x09,
	NS2_REPORT_GC = 0x0a,
};

enum switch2_init_step {
	NS2_INIT_READ_SERIAL,
	NS2_INIT_GET_FIRMWARE_INFO,
	NS2_INIT_READ_FACTORY_PRIMARY_CALIB,
	NS2_INIT_READ_FACTORY_SECONDARY_CALIB,
	NS2_INIT_READ_FACTORY_TRIGGER_CALIB,
	NS2_INIT_READ_USER_PRIMARY_CALIB,
	NS2_INIT_READ_USER_SECONDARY_CALIB,
	NS2_INIT_SET_FEATURE_MASK,
	NS2_INIT_ENABLE_FEATURES,
	NS2_INIT_ENABLE_RUMBLE,
	NS2_INIT_GRIP_BUTTONS,
	NS2_INIT_REPORT_FORMAT,
	NS2_INIT_INPUT,
	NS2_INIT_SET_PLAYER_LEDS,
	NS2_INIT_FINISH,
	NS2_INIT_DONE,
};

struct switch2_version_info {
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
	uint8_t ctlr_type;
	__le32 unk;
	int8_t dsp_major;
	int8_t dsp_minor;
	int8_t dsp_patch;
	int8_t dsp_type;
};

struct switch2_axis_calibration {
	uint16_t neutral;
	uint16_t negative;
	uint16_t positive;
};

struct switch2_stick_calibration {
	struct switch2_axis_calibration x;
	struct switch2_axis_calibration y;
};

struct switch2_hd_rumble {
	uint16_t hi_freq : 10;
	uint16_t hi_amp : 10;
	uint16_t lo_freq : 10;
	uint16_t lo_amp : 10;
} __packed;

struct switch2_erm_rumble {
	uint16_t error;
	uint16_t amplitude;
};

struct switch2_controller {
	struct hid_device *hdev;
	struct switch2_cfg_intf *cfg;
	struct kref refcount;

	char name[64];
	char phys[64];
	struct list_head entry;
	struct mutex lock;

	enum switch2_ctlr_type ctlr_type;
	enum switch2_init_step init_step;
	int init_retries;
	struct input_dev __rcu *input;
	char serial[NS2_FLASH_SIZE_SERIAL + 1];
	struct switch2_version_info version;

	struct switch2_stick_calibration stick_calib[2];
	uint8_t lt_zero;
	uint8_t rt_zero;

	uint32_t player_id;
	struct led_classdev *leds;

#if IS_ENABLED(CONFIG_NINTENDO_FF)
	spinlock_t rumble_lock;
	uint8_t rumble_seq;
	union {
		struct switch2_hd_rumble hd;
		struct switch2_erm_rumble sd;
	} rumble;
	uint64_t last_rumble_work;
	struct delayed_work rumble_work;
	uint8_t *rumble_buffer;
#endif
};

enum gc_rumble {
	GC_RUMBLE_OFF = 0,
	GC_RUMBLE_ON = 1,
	GC_RUMBLE_STOP = 2,
};

/*
 * The highest rumble level for "HD Rumble" is strong enough to potentially damage the controller,
 * and also leaves your hands feeling like melted jelly, so we set a semi-arbitrary scaling factor
 * to artificially limit the maximum for safety and comfort. It is currently unknown if the Switch
 * 2 itself does something similar, but it's quite likely.
 *
 * This value must be between 0 and 1024, otherwise the math below will overflow.
 */
#define RUMBLE_MAX 450u

/*
 * Semi-arbitrary values used to simulate the "rumble" sensation of an eccentric rotating
 * mass type haptic motor on the Switch 2 controllers' linear resonant actuator type haptics.
 *
 * The units used are unknown, but the values must be between 0 and 1023.
 */
#define RUMBLE_HI_FREQ 0x187
#define RUMBLE_LO_FREQ 0x112

static DEFINE_MUTEX(switch2_controllers_lock);
static LIST_HEAD(switch2_controllers);

struct switch2_ctlr_button_mapping {
	uint32_t code;
	int byte;
	uint32_t bit;
};

static const struct switch2_ctlr_button_mapping ns2_left_joycon_button_mappings[] = {
	{ BTN_DPAD_LEFT,	0, NS2_BTNL_LEFT,	},
	{ BTN_DPAD_UP,		0, NS2_BTNL_UP,		},
	{ BTN_DPAD_DOWN,	0, NS2_BTNL_DOWN,	},
	{ BTN_DPAD_RIGHT,	0, NS2_BTNL_RIGHT,	},
	{ BTN_TL,		0, NS2_BTNL_L,		},
	{ BTN_TL2,		0, NS2_BTNL_ZL,		},
	{ BTN_SELECT,		0, NS2_BTNL_MINUS,	},
	{ BTN_THUMBL,		0, NS2_BTNL_LS,		},
	{ KEY_RECORD,		1, NS2_BTN_JCL_CAPTURE,	},
	{ BTN_GRIPR,		1, NS2_BTN_JCL_SL,	},
	{ BTN_GRIPR2,		1, NS2_BTN_JCL_SR,	},
	{ BTN_GRIPL,		1, NS2_BTN_JCL_GL,	},
	{ /* sentinel */ },
};

static const struct switch2_ctlr_button_mapping ns2_left_joycon_button_unified_mappings[] = {
	{ BTN_DPAD_LEFT,	2, NS2_BTN_U3_LEFT,	},
	{ BTN_DPAD_UP,		2, NS2_BTN_U3_UP,	},
	{ BTN_DPAD_DOWN,	2, NS2_BTN_U3_DOWN,	},
	{ BTN_DPAD_RIGHT,	2, NS2_BTN_U3_RIGHT,	},
	{ BTN_TL,		2, NS2_BTN_U3_L,	},
	{ BTN_TL2,		2, NS2_BTN_U3_ZL,	},
	{ BTN_SELECT,		1, NS2_BTN_U2_MINUS,	},
	{ BTN_THUMBL,		1, NS2_BTN_U2_LS,	},
	{ KEY_RECORD,		1, NS2_BTN_U2_CAPTURE,	},
	{ BTN_GRIPR,		2, NS2_BTN_U3_SL,	},
	{ BTN_GRIPR2,		2, NS2_BTN_U3_SR,	},
	{ BTN_GRIPL,		3, NS2_BTN_U4_GL,	},
	{ /* sentinel */ },
};

static const struct switch2_ctlr_button_mapping ns2_right_joycon_button_mappings[] = {
	{ BTN_SOUTH,	0, NS2_BTNR_A,		},
	{ BTN_EAST,	0, NS2_BTNR_B,		},
	{ BTN_NORTH,	0, NS2_BTNR_X,		},
	{ BTN_WEST,	0, NS2_BTNR_Y,		},
	{ BTN_TR,	0, NS2_BTNR_R,		},
	{ BTN_TR2,	0, NS2_BTNR_ZR,		},
	{ BTN_START,	0, NS2_BTNR_PLUS,	},
	{ BTN_THUMBR,	0, NS2_BTNR_RS,		},
	{ BTN_C,	1, NS2_BTN_JCR_C,	},
	{ BTN_MODE,	1, NS2_BTN_JCR_HOME,	},
	{ BTN_GRIPL2,	1, NS2_BTN_JCR_SL,	},
	{ BTN_GRIPL,	1, NS2_BTN_JCR_SR,	},
	{ BTN_GRIPR,	1, NS2_BTN_JCR_GR,	},
	{ /* sentinel */ },
};

static const struct switch2_ctlr_button_mapping ns2_right_joycon_button_unified_mappings[] = {
	{ BTN_SOUTH,	0, NS2_BTN_U1_A,	},
	{ BTN_EAST,	0, NS2_BTN_U1_B,	},
	{ BTN_NORTH,	0, NS2_BTN_U1_X,	},
	{ BTN_WEST,	0, NS2_BTN_U1_Y,	},
	{ BTN_TR,	0, NS2_BTN_U1_R,	},
	{ BTN_TR2,	0, NS2_BTN_U1_ZR	},
	{ BTN_START,	1, NS2_BTN_U2_PLUS,	},
	{ BTN_THUMBR,	1, NS2_BTN_U2_RS,	},
	{ BTN_C,	1, NS2_BTN_U2_C,	},
	{ BTN_MODE,	1, NS2_BTN_U2_HOME,	},
	{ BTN_GRIPL2,	0, NS2_BTN_U1_SL,	},
	{ BTN_GRIPL,	0, NS2_BTN_U1_SR,	},
	{ BTN_GRIPR,	3, NS2_BTN_U4_GR,	},
	{ /* sentinel */ },
};

static const struct switch2_ctlr_button_mapping ns2_procon_mappings[] = {
	{ BTN_SOUTH,	0, NS2_BTNR_A,		},
	{ BTN_EAST,	0, NS2_BTNR_B,		},
	{ BTN_NORTH,	0, NS2_BTNR_X,		},
	{ BTN_WEST,	0, NS2_BTNR_Y,		},
	{ BTN_TL,	1, NS2_BTNL_L,		},
	{ BTN_TR,	0, NS2_BTNR_R,		},
	{ BTN_TL2,	1, NS2_BTNL_ZL,		},
	{ BTN_TR2,	0, NS2_BTNR_ZR,		},
	{ BTN_SELECT,	1, NS2_BTNL_MINUS,	},
	{ BTN_START,	0, NS2_BTNR_PLUS,	},
	{ BTN_THUMBL,	1, NS2_BTNL_LS,		},
	{ BTN_THUMBR,	0, NS2_BTNR_RS,		},
	{ BTN_MODE,	2, NS2_BTN_PRO_HOME	},
	{ KEY_RECORD,	2, NS2_BTN_PRO_CAPTURE	},
	{ BTN_GRIPR,	2, NS2_BTN_PRO_GR	},
	{ BTN_GRIPL,	2, NS2_BTN_PRO_GL	},
	{ BTN_C,	2, NS2_BTN_PRO_C	},
	{ /* sentinel */ },
};

static const struct switch2_ctlr_button_mapping ns2_procon_unified_mappings[] = {
	{ BTN_SOUTH,	0, NS2_BTN_U1_A,	},
	{ BTN_EAST,	0, NS2_BTN_U1_B,	},
	{ BTN_NORTH,	0, NS2_BTN_U1_X,	},
	{ BTN_WEST,	0, NS2_BTN_U1_Y,	},
	{ BTN_TL,	2, NS2_BTN_U3_L,	},
	{ BTN_TR,	0, NS2_BTN_U1_R,	},
	{ BTN_TL2,	2, NS2_BTN_U3_ZL,	},
	{ BTN_TR2,	0, NS2_BTN_U1_ZR,	},
	{ BTN_SELECT,	1, NS2_BTN_U2_MINUS,	},
	{ BTN_START,	1, NS2_BTN_U2_PLUS,	},
	{ BTN_THUMBL,	1, NS2_BTN_U2_LS,	},
	{ BTN_THUMBR,	1, NS2_BTN_U2_RS,	},
	{ BTN_MODE,	1, NS2_BTN_U2_HOME	},
	{ KEY_RECORD,	1, NS2_BTN_U2_CAPTURE	},
	{ BTN_GRIPR,	3, NS2_BTN_U4_GR	},
	{ BTN_GRIPL,	3, NS2_BTN_U4_GL	},
	{ BTN_C,	1, NS2_BTN_U2_C		},
	{ /* sentinel */ },
};

static const struct switch2_ctlr_button_mapping ns2_gccon_mappings[] = {
	{ BTN_SOUTH,	0, NS2_BTNR_A,		},
	{ BTN_EAST,	0, NS2_BTNR_B,		},
	{ BTN_NORTH,	0, NS2_BTNR_X,		},
	{ BTN_WEST,	0, NS2_BTNR_Y,		},
	{ BTN_TL2,	1, NS2_BTNL_L,		},
	{ BTN_TR2,	0, NS2_BTNR_R,		},
	{ BTN_TL,	1, NS2_BTNL_ZL,		},
	{ BTN_TR,	0, NS2_BTNR_ZR,		},
	{ BTN_SELECT,	1, NS2_BTNL_MINUS,	},
	{ BTN_START,	0, NS2_BTNR_PLUS,	},
	{ BTN_MODE,	2, NS2_BTN_GC_HOME	},
	{ KEY_RECORD,	2, NS2_BTN_GC_CAPTURE	},
	{ BTN_C,	2, NS2_BTN_GC_C		},
	{ /* sentinel */ },
};

static const struct switch2_ctlr_button_mapping ns2_gccon_unified_mappings[] = {
	{ BTN_SOUTH,	0, NS2_BTN_U1_A,	},
	{ BTN_EAST,	0, NS2_BTN_U1_B,	},
	{ BTN_NORTH,	0, NS2_BTN_U1_X,	},
	{ BTN_WEST,	0, NS2_BTN_U1_Y,	},
	{ BTN_TL2,	2, NS2_BTN_U3_L,	},
	{ BTN_TR2,	0, NS2_BTN_U1_R,	},
	{ BTN_TL,	2, NS2_BTN_U3_ZL	},
	{ BTN_TR,	0, NS2_BTN_U1_ZR	},
	{ BTN_SELECT,	1, NS2_BTN_U2_MINUS,	},
	{ BTN_START,	1, NS2_BTN_U2_PLUS,	},
	{ BTN_MODE,	1, NS2_BTN_U2_HOME	},
	{ KEY_RECORD,	1, NS2_BTN_U2_CAPTURE	},
	{ BTN_C,	1, NS2_BTN_U2_C		},
	{ /* sentinel */ },
};

static const uint8_t switch2_init_cmd_data[] = {
	/*
	 * The last 6 bytes of this packet are the MAC address of
	 * the console, but we don't need that for USB
	 */
	0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static const uint8_t switch2_one_data[] = { 0x01, 0x00, 0x00, 0x00 };

static const uint8_t switch2_feature_mask[] = {
	NS2_FEATURE_BUTTONS | NS2_FEATURE_ANALOG | NS2_FEATURE_IMU | NS2_FEATURE_RUMBLE,
	0x00, 0x00, 0x00
};

static int switch2_init_controller(struct switch2_controller *ns2);

static void switch2_init_step_done(struct switch2_controller *ns2, enum switch2_init_step step)
{
	if (ns2->init_step != step)
		return;

	ns2->init_retries = 0;
	ns2->init_step++;
}

static inline bool switch2_ctlr_is_joycon(enum switch2_ctlr_type type)
{
	return type == NS2_CTLR_TYPE_JCL || type == NS2_CTLR_TYPE_JCR;
}

static struct switch2_controller *switch2_get_controller(const char *phys)
{
	struct switch2_controller *ns2;

	guard(mutex)(&switch2_controllers_lock);
	list_for_each_entry(ns2, &switch2_controllers, entry) {
		if (strncmp(ns2->phys, phys, sizeof(ns2->phys)) == 0) {
			if (kref_get_unless_zero(&ns2->refcount))
				return ns2;
		}
	}
	ns2 = kzalloc_obj(*ns2);
	if (!ns2)
		return ERR_PTR(-ENOMEM);

	kref_init(&ns2->refcount);
	mutex_init(&ns2->lock);
	INIT_LIST_HEAD(&ns2->entry);
	list_add(&ns2->entry, &switch2_controllers);
	strscpy(ns2->phys, phys, sizeof(ns2->phys));
#if IS_ENABLED(CONFIG_NINTENDO_FF)
	spin_lock_init(&ns2->rumble_lock);
#endif
	return ns2;
}

static void switch2_controller_put(struct switch2_controller *ns2)
{
	struct input_dev *input;

	mutex_lock(&ns2->lock);
	rcu_read_lock();
	input = rcu_dereference(ns2->input);
	rcu_read_unlock();

	rcu_assign_pointer(ns2->input, NULL);
	synchronize_rcu();

	ns2->init_step = 0;
	mutex_unlock(&ns2->lock);

	if (input)
		input_unregister_device(input);
}

static void switch2_kref_put(struct kref *refcount)
{
	struct switch2_controller *ns2 = container_of(refcount,
					struct switch2_controller, refcount);

	guard(mutex)(&switch2_controllers_lock);
	list_del_init(&ns2->entry);
	mutex_destroy(&ns2->lock);
	kfree(ns2);
}

#if IS_ENABLED(CONFIG_NINTENDO_FF)
static void switch2_encode_rumble(struct switch2_hd_rumble *rumble, uint8_t buffer[5])
{
	buffer[0] = rumble->hi_freq;
	buffer[1] = (rumble->hi_freq >> 8) | (rumble->hi_amp << 2);
	buffer[2] = (rumble->hi_amp >> 6) | (rumble->lo_freq << 4);
	buffer[3] = (rumble->lo_freq >> 4) | (rumble->lo_amp << 6);
	buffer[4] = rumble->lo_amp >> 2;
}

static int switch2_play_effect(struct input_dev *dev, void *data, struct ff_effect *effect)
{
	struct switch2_controller *ns2 = input_get_drvdata(dev);
	unsigned long flags;

	if (effect->type != FF_RUMBLE)
		return 0;

	if (!ns2)
		return -ENODEV;

	spin_lock_irqsave(&ns2->rumble_lock, flags);
	if (ns2->ctlr_type == NS2_CTLR_TYPE_GC) {
		ns2->rumble.sd.amplitude = max(effect->u.rumble.strong_magnitude,
			(uint16_t) (effect->u.rumble.weak_magnitude >> 1));
	} else {
		ns2->rumble.hd.hi_freq = RUMBLE_HI_FREQ;
		ns2->rumble.hd.lo_freq = RUMBLE_LO_FREQ;
		ns2->rumble.hd.hi_amp = effect->u.rumble.weak_magnitude * RUMBLE_MAX >> 16;
		ns2->rumble.hd.lo_amp = effect->u.rumble.strong_magnitude * RUMBLE_MAX >> 16;
	}
	spin_unlock_irqrestore(&ns2->rumble_lock, flags);

	schedule_delayed_work(&ns2->rumble_work, 0);

	return 0;
}

static void switch2_rumble_work(struct work_struct *work)
{
	struct switch2_controller *ns2 = container_of(to_delayed_work(work),
						      struct switch2_controller, rumble_work);
	unsigned long flags;
	bool active;
	int ret = 0;

	spin_lock_irqsave(&ns2->rumble_lock, flags);
	ns2->rumble_buffer[0x1] = 0x50 | ns2->rumble_seq;
	if (ns2->ctlr_type == NS2_CTLR_TYPE_GC) {
		ns2->rumble_buffer[0] = 3;
		if (ns2->rumble.sd.amplitude == 0) {
			ns2->rumble_buffer[2] = GC_RUMBLE_STOP;
			ns2->rumble.sd.error = 0;
			active = false;
		} else {
			if (ns2->rumble.sd.error < ns2->rumble.sd.amplitude) {
				ns2->rumble_buffer[2] = GC_RUMBLE_ON;
				ns2->rumble.sd.error += U16_MAX - ns2->rumble.sd.amplitude;
			} else {
				ns2->rumble_buffer[2] = GC_RUMBLE_OFF;
				ns2->rumble.sd.error -= ns2->rumble.sd.amplitude;
			}
			active = true;
		}
	} else {
		ns2->rumble_buffer[0] = 1;
		switch2_encode_rumble(&ns2->rumble.hd, &ns2->rumble_buffer[0x2]);
		active = ns2->rumble.hd.hi_amp || ns2->rumble.hd.lo_amp;
		if (ns2->ctlr_type == NS2_CTLR_TYPE_PRO) {
			/*
			 * The Pro Controller contains separate LRAs on each
			 * side that can be controlled individually.
			 */
			ns2->rumble_buffer[0] = 2;
			ns2->rumble_buffer[0x11] = 0x50 | ns2->rumble_seq;
			switch2_encode_rumble(&ns2->rumble.hd, &ns2->rumble_buffer[0x12]);
		}
	}
	ns2->rumble_seq = (ns2->rumble_seq + 1) & 0xF;
	spin_unlock_irqrestore(&ns2->rumble_lock, flags);

	if (active) {
		unsigned long interval = msecs_to_jiffies(4);
		uint64_t current_jiffies = get_jiffies_64();

		if (!ns2->last_rumble_work)
			ns2->last_rumble_work = current_jiffies;
		else
			ns2->last_rumble_work += interval;

		/* Reschedule a little early to make sure the buffer never underruns */
		interval -= msecs_to_jiffies(2);
		if (ns2->last_rumble_work + interval >= current_jiffies)
			schedule_delayed_work(&ns2->rumble_work,
				ns2->last_rumble_work + interval - current_jiffies);
		else
			schedule_delayed_work(&ns2->rumble_work, 0);
	} else {
		ns2->last_rumble_work = 0;
	}

	mutex_lock(&ns2->lock);
	if (!ns2->hdev) {
		cancel_delayed_work(&ns2->rumble_work);
	} else {
		ret = hid_hw_output_report(ns2->hdev, ns2->rumble_buffer, 64);
		/*
		 * Don't log on ENODEV, ESHUTDOWN, or EPROTO, which can happen
		 * mid-hotplug. Also cancel any further work on ENODEV or
		 * ESHUTDOWN as they're clear indications that the endpoint
		 * is dead.
		 */
		if (ret == -ENODEV || ret == -ESHUTDOWN)
			cancel_delayed_work(&ns2->rumble_work);
		else if (ret < 0 && ret != -EPROTO)
			hid_warn_ratelimited(ns2->hdev,
				"Failed to send output report ret=%d\n", ret);
	}
	mutex_unlock(&ns2->lock);
}
#endif

static int switch2_set_leds(struct switch2_controller *ns2)
{
	int i;
	uint8_t message[8] = { 0 };

	for (i = 0; i < JC_NUM_LEDS; i++)
		message[0] |= (!!ns2->leds[i].brightness) << i;

	if (!ns2->cfg)
		return -ENOTCONN;
	return ns2->cfg->send_command(NS2_CMD_LED, NS2_SUBCMD_LED_PATTERN,
		&message, sizeof(message),
		ns2->cfg);
}

static int switch2_player_led_brightness_set(struct led_classdev *led,
					    enum led_brightness brightness)
{
	struct device *dev = led->dev->parent;
	struct input_dev *input = to_input_dev(dev);
	struct switch2_controller *ns2 = input_get_drvdata(input);

	if (!ns2)
		return -ENODEV;

	guard(mutex)(&ns2->lock);
	return switch2_set_leds(ns2);
}

static void switch2_config_buttons(struct input_dev *idev,
	const struct switch2_ctlr_button_mapping button_mappings[])
{
	const struct switch2_ctlr_button_mapping *button;

	for (button = button_mappings; button->code; button++)
		input_set_capability(idev, EV_KEY, button->code);
}

static int switch2_input_ref(struct input_dev *input)
{
	struct switch2_controller *ns2 = input_get_drvdata(input);

	kref_get(&ns2->refcount);

	return 0;
}

static void switch2_input_deref(struct input_dev *input)
{
	struct switch2_controller *ns2 = input_get_drvdata(input);

	kref_put(&ns2->refcount, switch2_kref_put);
}

static int switch2_init_input(struct switch2_controller *ns2)
{
	struct input_dev *input;
	struct hid_device *hdev = ns2->hdev;
	int player_led_pattern;
	int i;
	int ret;

	rcu_read_lock();
	input = rcu_dereference(ns2->input);
	rcu_read_unlock();

	if (input) {
		switch2_init_step_done(ns2, NS2_INIT_INPUT);
		return 0;
	}

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	input_set_drvdata(input, ns2);
	input->open = switch2_input_ref;
	input->close = switch2_input_deref;
	input->dev.parent = &hdev->dev;
	input->id.bustype = hdev->bus;
	input->id.vendor = hdev->vendor;
	input->id.product = hdev->product;
	input->id.version = hdev->version;
	input->uniq = ns2->serial;
	input->name = ns2->name;
	input->phys = hdev->phys;

	switch (ns2->ctlr_type) {
	case NS2_CTLR_TYPE_JCL:
		input_set_abs_params(input, ABS_X, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_Y, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		switch2_config_buttons(input, ns2_left_joycon_button_mappings);
		break;
	case NS2_CTLR_TYPE_JCR:
		input_set_abs_params(input, ABS_X, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_Y, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		switch2_config_buttons(input, ns2_right_joycon_button_mappings);
		break;
	case NS2_CTLR_TYPE_GC:
		input_set_abs_params(input, ABS_X, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_Y, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_RX, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_RY, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_Z, 0, NS2_TRIGGER_RANGE, 32, 128);
		input_set_abs_params(input, ABS_RZ, 0, NS2_TRIGGER_RANGE, 32, 128);
		input_set_abs_params(input, ABS_HAT0X, -1, 1, 0, 0);
		input_set_abs_params(input, ABS_HAT0Y, -1, 1, 0, 0);
		switch2_config_buttons(input, ns2_gccon_mappings);
		break;
	case NS2_CTLR_TYPE_PRO:
		input_set_abs_params(input, ABS_X, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_Y, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_RX, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_RY, NS2_AXIS_MIN, NS2_AXIS_MAX, 32, 128);
		input_set_abs_params(input, ABS_HAT0X, -1, 1, 0, 0);
		input_set_abs_params(input, ABS_HAT0Y, -1, 1, 0, 0);
		switch2_config_buttons(input, ns2_procon_mappings);
		break;
	default:
		input_free_device(input);
		return -EINVAL;
	}

#if IS_ENABLED(CONFIG_NINTENDO_FF)
	ns2->rumble_buffer = devm_kzalloc(&input->dev, 64, GFP_KERNEL);
	if (!ns2->rumble_buffer) {
		input_free_device(input);
		return -ENOMEM;
	}
	ret = devm_delayed_work_autocancel(&input->dev, &ns2->rumble_work, switch2_rumble_work);
	if (ret < 0) {
		input_free_device(input);
		return ret;
	}

	input_set_capability(input, EV_FF, FF_RUMBLE);
	ret = input_ff_create_memless(input, NULL, switch2_play_effect);
	if (ret) {
		input_free_device(input);
		return ret;
	}
#endif

	hid_info(ns2->hdev, "Firmware version %u.%u.%u (type %i)\n", ns2->version.major,
		ns2->version.minor, ns2->version.patch, ns2->version.ctlr_type);
	if (ns2->version.dsp_type >= 0)
		hid_info(ns2->hdev, "DSP version %u.%u.%u\n", ns2->version.dsp_major,
			ns2->version.dsp_minor, ns2->version.dsp_patch);

	ret = input_register_device(input);
	if (ret < 0) {
		hid_err(ns2->hdev, "Failed to register input; ret=%d\n", ret);
		input_free_device(input);
		return ret;
	}

	player_led_pattern = ns2->player_id % JC_NUM_LED_PATTERNS;
	hid_dbg(hdev, "assigned player %d led pattern", player_led_pattern + 1);

	ns2->leds = devm_kcalloc(&input->dev, JC_NUM_LEDS, sizeof(*ns2->leds), GFP_KERNEL);
	if (!ns2->leds) {
		input_unregister_device(input);
		return -ENOMEM;
	}

	for (i = 0; i < JC_NUM_LEDS; i++) {
		struct led_classdev *led = &ns2->leds[i];

		led->brightness = joycon_player_led_patterns[player_led_pattern][i];
		led->max_brightness = 1;
		led->brightness_set_blocking = switch2_player_led_brightness_set;
		led->flags = LED_CORE_SUSPENDRESUME | LED_HW_PLUGGABLE | LED_RETAIN_AT_SHUTDOWN;
		char *name = devm_kasprintf(&input->dev, GFP_KERNEL, "%s:%s:%s",
				      dev_name(&input->dev),
				      "green",
				      joycon_player_led_names[i]);

		if (!name) {
			dev_err(&input->dev, "Failed to allocate name for player %d LED; ret=%d\n",
				i + 1, ret);
			break;
		}

		led->name = name;
		ret = devm_led_classdev_register(&input->dev, led);
		if (ret < 0) {
			dev_err(&input->dev, "Failed to register player %d LED; ret=%d\n",
				i + 1, ret);
			break;
		}
	}

	rcu_assign_pointer(ns2->input, input);
	synchronize_rcu();

	switch2_init_step_done(ns2, NS2_INIT_INPUT);
	return switch2_init_controller(ns2);
}

static bool switch2_parse_stick_calibration(struct switch2_stick_calibration *calib,
	const uint8_t *data)
{
	static const uint8_t UNCALIBRATED[9] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
	};
	if (memcmp(UNCALIBRATED, data, sizeof(UNCALIBRATED)) == 0)
		return false;

	calib->x.neutral = data[0];
	calib->x.neutral |= (data[1] & 0x0F) << 8;

	calib->y.neutral = data[1] >> 4;
	calib->y.neutral |= data[2] << 4;

	calib->x.positive = data[3];
	calib->x.positive |= (data[4] & 0x0F) << 8;

	calib->y.positive = data[4] >> 4;
	calib->y.positive |= data[5] << 4;

	calib->x.negative = data[6];
	calib->x.negative |= (data[7] & 0x0F) << 8;

	calib->y.negative = data[7] >> 4;
	calib->y.negative |= data[8] << 4;

	return true;
}

static void switch2_handle_flash_read(struct switch2_controller *ns2, uint8_t size,
	uint32_t address, const uint8_t *data)
{
	bool ok;

	switch (address) {
	case NS2_FLASH_ADDR_SERIAL:
		if (size != NS2_FLASH_SIZE_SERIAL)
			return;
		memcpy(ns2->serial, data, size);
		switch2_init_step_done(ns2, NS2_INIT_READ_SERIAL);
		break;
	case NS2_FLASH_ADDR_FACTORY_PRIMARY_CALIB:
		if (size != NS2_FLASH_SIZE_FACTORY_AXIS_CALIB)
			return;
		switch2_init_step_done(ns2, NS2_INIT_READ_FACTORY_PRIMARY_CALIB);
		ok = switch2_parse_stick_calibration(&ns2->stick_calib[0], data);
		if (ns2->hdev) {
			if (ok) {
				hid_dbg(ns2->hdev, "Got factory primary stick calibration:\n");
				hid_dbg(ns2->hdev, "Left max: %i, neutral: %i, right max: %i\n",
					ns2->stick_calib[0].x.negative,
					ns2->stick_calib[0].x.neutral,
					ns2->stick_calib[0].x.positive);
				hid_dbg(ns2->hdev, "Down max: %i, neutral: %i, up max: %i\n",
					ns2->stick_calib[0].y.negative,
					ns2->stick_calib[0].y.neutral,
					ns2->stick_calib[0].y.positive);
			} else {
				hid_dbg(ns2->hdev, "Factory primary stick calibration not present\n");
			}
		}
		break;
	case NS2_FLASH_ADDR_FACTORY_SECONDARY_CALIB:
		if (size != NS2_FLASH_SIZE_FACTORY_AXIS_CALIB)
			return;
		switch2_init_step_done(ns2, NS2_INIT_READ_FACTORY_SECONDARY_CALIB);
		ok = switch2_parse_stick_calibration(&ns2->stick_calib[1], data);
		if (ns2->hdev) {
			if (ok) {
				hid_dbg(ns2->hdev, "Got factory secondary stick calibration:\n");
				hid_dbg(ns2->hdev, "Left max: %i, neutral: %i, right max: %i\n",
					ns2->stick_calib[1].x.negative,
					ns2->stick_calib[1].x.neutral,
					ns2->stick_calib[1].x.positive);
				hid_dbg(ns2->hdev, "Down max: %i, neutral: %i, up max: %i\n",
					ns2->stick_calib[1].y.negative,
					ns2->stick_calib[1].y.neutral,
					ns2->stick_calib[1].y.positive);
			} else {
				hid_dbg(ns2->hdev, "Factory secondary stick calibration not present\n");
			}
		}
		break;
	case NS2_FLASH_ADDR_FACTORY_TRIGGER_CALIB:
		if (size != NS2_FLASH_SIZE_FACTORY_TRIGGER_CALIB)
			return;
		switch2_init_step_done(ns2, NS2_INIT_READ_FACTORY_TRIGGER_CALIB);
		if (data[0] != 0xFF && data[1] != 0xFF) {
			ns2->lt_zero = data[0];
			ns2->rt_zero = data[1];

			if (ns2->hdev) {
				hid_dbg(ns2->hdev, "Got factory trigger calibration:\n");
				hid_dbg(ns2->hdev, "Left zero point: %i\n", ns2->lt_zero);
				hid_dbg(ns2->hdev, "Right zero point: %i\n", ns2->rt_zero);
			}
		} else if (ns2->hdev) {
			hid_dbg(ns2->hdev, "Factory trigger calibration not present\n");
		}
		break;
	case NS2_FLASH_ADDR_USER_PRIMARY_CALIB:
		if (size != NS2_FLASH_SIZE_USER_AXIS_CALIB)
			return;
		switch2_init_step_done(ns2, NS2_INIT_READ_USER_PRIMARY_CALIB);
		if (get_unaligned_le16((__le16 *)data) != NS2_USER_CALIB_MAGIC) {
			if (ns2->hdev)
				hid_dbg(ns2->hdev, "No user primary stick calibration present\n");
			break;
		}

		ok = switch2_parse_stick_calibration(&ns2->stick_calib[0], &data[2]);
		if (ns2->hdev) {
			if (ok) {
				hid_dbg(ns2->hdev, "Got user primary stick calibration:\n");
				hid_dbg(ns2->hdev, "Left max: %i, neutral: %i, right max: %i\n",
					ns2->stick_calib[0].x.negative,
					ns2->stick_calib[0].x.neutral,
					ns2->stick_calib[0].x.positive);
				hid_dbg(ns2->hdev, "Down max: %i, neutral: %i, up max: %i\n",
					ns2->stick_calib[0].y.negative,
					ns2->stick_calib[0].y.neutral,
					ns2->stick_calib[0].y.positive);
			} else {
				hid_dbg(ns2->hdev, "No user primary stick calibration present\n");
			}
		}
		break;
	case NS2_FLASH_ADDR_USER_SECONDARY_CALIB:
		if (size != NS2_FLASH_SIZE_USER_AXIS_CALIB)
			return;
		switch2_init_step_done(ns2, NS2_INIT_READ_USER_SECONDARY_CALIB);
		if (get_unaligned_le16((__le16 *)data) != NS2_USER_CALIB_MAGIC) {
			if (ns2->hdev)
				hid_dbg(ns2->hdev, "No user secondary stick calibration present\n");
			break;
		}

		ok = switch2_parse_stick_calibration(&ns2->stick_calib[1], &data[2]);
		if (ns2->hdev) {
			if (ok) {
				hid_dbg(ns2->hdev, "Got user secondary stick calibration:\n");
				hid_dbg(ns2->hdev, "Left max: %i, neutral: %i, right max: %i\n",
					ns2->stick_calib[1].x.negative,
					ns2->stick_calib[1].x.neutral,
					ns2->stick_calib[1].x.positive);
				hid_dbg(ns2->hdev, "Down max: %i, neutral: %i, up max: %i\n",
					ns2->stick_calib[1].y.negative,
					ns2->stick_calib[1].y.neutral,
					ns2->stick_calib[1].y.positive);
			} else {
				hid_dbg(ns2->hdev, "No user secondary stick calibration present\n");
			}
		}
		break;
	}
}

static void switch2_report_buttons(struct input_dev *input, const uint8_t *bytes,
	const struct switch2_ctlr_button_mapping button_mappings[])
{
	const struct switch2_ctlr_button_mapping *button;

	for (button = button_mappings; button->code; button++)
		input_report_key(input, button->code, bytes[button->byte] & button->bit);
}

static void switch2_report_axis(struct input_dev *input, struct switch2_axis_calibration *calib,
	int axis, bool invert, int value)
{
	if (calib && calib->neutral && calib->negative && calib->positive) {
		value -= calib->neutral;
		value *= NS2_AXIS_MAX + 1;
		if (value < 0)
			value /= calib->negative;
		else
			value /= calib->positive;
	} else {
		value = (value - 2048) * 16;
	}

	if (invert)
		value = -value;
	input_report_abs(input, axis,
		clamp(value, NS2_AXIS_MIN, NS2_AXIS_MAX));
}

static void switch2_report_stick(struct input_dev *input, struct switch2_stick_calibration *calib,
	int x, bool invert_x, int y, bool invert_y, const uint8_t *data)
{
	switch2_report_axis(input, &calib->x, x, invert_x, data[0] | ((data[1] & 0x0F) << 8));
	switch2_report_axis(input, &calib->y, y, invert_y, (data[1] >> 4) | (data[2] << 4));
}

static void switch2_report_trigger(struct input_dev *input, uint8_t zero, int abs, uint8_t data)
{
	int value = (NS2_TRIGGER_RANGE + 1) * (data - zero);

	if (zero != 232)
		value /= (232 - zero);
	input_report_abs(input, abs, clamp(value, 0, NS2_TRIGGER_RANGE));
}

static int switch2_event(struct hid_device *hdev, struct hid_report *report, uint8_t *raw_data,
	int size)
{
	struct switch2_controller *ns2 = hid_get_drvdata(hdev);
	struct input_dev *input;

	if (report->type != HID_INPUT_REPORT)
		return 0;

	if (size < 15)
		return -EINVAL;

	guard(rcu)();
	input = rcu_dereference(ns2->input);

	if (!input)
		return 0;

	switch (report->id) {
	case NS2_REPORT_UNIFIED:
		if (size < 0x3f)
			return -EINVAL;

		switch (ns2->ctlr_type) {
		case NS2_CTLR_TYPE_JCL:
			switch2_report_stick(input, &ns2->stick_calib[0],
				ABS_X, false, ABS_Y, true, &raw_data[11]);
			switch2_report_buttons(input, &raw_data[5],
				ns2_left_joycon_button_unified_mappings);
			break;
		case NS2_CTLR_TYPE_JCR:
			switch2_report_stick(input, &ns2->stick_calib[0],
				ABS_X, false, ABS_Y, true, &raw_data[14]);
			switch2_report_buttons(input, &raw_data[5],
				ns2_right_joycon_button_unified_mappings);
			break;
		case NS2_CTLR_TYPE_GC:
			input_report_abs(input, ABS_HAT0X,
				!!(raw_data[7] & NS2_BTN_U3_RIGHT) -
				!!(raw_data[7] & NS2_BTN_U3_LEFT));
			input_report_abs(input, ABS_HAT0Y,
				!!(raw_data[7] & NS2_BTN_U3_DOWN) -
				!!(raw_data[7] & NS2_BTN_U3_UP));
			switch2_report_buttons(input, &raw_data[5], ns2_gccon_unified_mappings);
			switch2_report_stick(input, &ns2->stick_calib[0],
				ABS_X, false, ABS_Y, true, &raw_data[11]);
			switch2_report_stick(input, &ns2->stick_calib[1],
				ABS_RX, false, ABS_RY, true, &raw_data[14]);
			switch2_report_trigger(input, ns2->lt_zero, ABS_Z, raw_data[0x3d]);
			switch2_report_trigger(input, ns2->rt_zero, ABS_RZ, raw_data[0x3e]);
			break;
		case NS2_CTLR_TYPE_PRO:
			input_report_abs(input, ABS_HAT0X,
				!!(raw_data[7] & NS2_BTN_U3_RIGHT) -
				!!(raw_data[7] & NS2_BTN_U3_LEFT));
			input_report_abs(input, ABS_HAT0Y,
				!!(raw_data[7] & NS2_BTN_U3_DOWN) -
				!!(raw_data[7] & NS2_BTN_U3_UP));
			switch2_report_buttons(input, &raw_data[5], ns2_procon_unified_mappings);
			switch2_report_stick(input, &ns2->stick_calib[0],
				ABS_X, false, ABS_Y, true, &raw_data[11]);
			switch2_report_stick(input, &ns2->stick_calib[1],
				ABS_RX, false, ABS_RY, true, &raw_data[14]);
			break;
		}
		break;
	case NS2_REPORT_JCL:
		switch2_report_stick(input, &ns2->stick_calib[0], ABS_X, false,
			ABS_Y, true, &raw_data[6]);
		switch2_report_buttons(input, &raw_data[3], ns2_left_joycon_button_mappings);
		break;
	case NS2_REPORT_JCR:
		switch2_report_stick(input, &ns2->stick_calib[0], ABS_X, false,
			ABS_Y, true, &raw_data[6]);
		switch2_report_buttons(input, &raw_data[3], ns2_right_joycon_button_mappings);
		break;
	case NS2_REPORT_GC:
		input_report_abs(input, ABS_HAT0X,
			!!(raw_data[4] & NS2_BTNL_RIGHT) -
			!!(raw_data[4] & NS2_BTNL_LEFT));
		input_report_abs(input, ABS_HAT0Y,
			!!(raw_data[4] & NS2_BTNL_DOWN) -
			!!(raw_data[4] & NS2_BTNL_UP));
		switch2_report_buttons(input, &raw_data[3], ns2_gccon_mappings);
		switch2_report_stick(input, &ns2->stick_calib[0], ABS_X, false,
			ABS_Y, true, &raw_data[6]);
		switch2_report_stick(input, &ns2->stick_calib[1], ABS_RX, false,
			ABS_RY, true, &raw_data[9]);
		switch2_report_trigger(input, ns2->lt_zero, ABS_Z, raw_data[13]);
		switch2_report_trigger(input, ns2->rt_zero, ABS_RZ, raw_data[14]);
		break;
	case NS2_REPORT_PRO:
		input_report_abs(input, ABS_HAT0X,
			!!(raw_data[4] & NS2_BTNL_RIGHT) -
			!!(raw_data[4] & NS2_BTNL_LEFT));
		input_report_abs(input, ABS_HAT0Y,
			!!(raw_data[4] & NS2_BTNL_DOWN) -
			!!(raw_data[4] & NS2_BTNL_UP));
		switch2_report_buttons(input, &raw_data[3], ns2_procon_mappings);
		switch2_report_stick(input, &ns2->stick_calib[0], ABS_X, false,
			ABS_Y, true, &raw_data[6]);
		switch2_report_stick(input, &ns2->stick_calib[1], ABS_RX, false,
			ABS_RY, true, &raw_data[9]);
		break;
	default:
		return -EINVAL;
	}

	input_sync(input);
	return 0;
}

static int switch2_features_enable(struct switch2_controller *ns2, int features)
{
	__le32 feature_bits = __cpu_to_le32(features);

	if (!ns2->cfg)
		return -ENOTCONN;
	return ns2->cfg->send_command(NS2_CMD_FEATSEL, NS2_SUBCMD_FEATSEL_ENABLE,
		&feature_bits, sizeof(feature_bits),
		ns2->cfg);
}

static int switch2_read_flash(struct switch2_controller *ns2, uint32_t address,
	uint8_t size)
{
	uint8_t message[8] = { size, 0x7e };

	if (!ns2->cfg)
		return -ENOTCONN;
	put_unaligned_le32(address, &message[4]);
	return ns2->cfg->send_command(NS2_CMD_FLASH, NS2_SUBCMD_FLASH_READ, message,
		sizeof(message), ns2->cfg);
}

static int switch2_set_player_id(struct switch2_controller *ns2, uint32_t player_id)
{
	int i;
	int player_led_pattern = player_id % JC_NUM_LED_PATTERNS;

	for (i = 0; i < JC_NUM_LEDS; i++)
		ns2->leds[i].brightness = joycon_player_led_patterns[player_led_pattern][i];

	return switch2_set_leds(ns2);
}

static int switch2_set_report_format(struct switch2_controller *ns2, enum switch2_report_id fmt)
{
	__le32 format_id = __cpu_to_le32(fmt);

	if (!ns2->cfg)
		return -ENOTCONN;
	return ns2->cfg->send_command(NS2_CMD_INIT, NS2_SUBCMD_INIT_SELECT_REPORT,
		&format_id, sizeof(format_id),
		ns2->cfg);
}

int switch2_init_controller(struct switch2_controller *ns2)
{
	if (ns2->init_step == NS2_INIT_DONE)
		return 0;

	if (!ns2->cfg)
		return -ENOTCONN;

	if (ns2->init_retries > NS2_MAX_INIT_RETRIES) {
		if (ns2->init_retries == NS2_MAX_INIT_RETRIES + 1) {
			dev_err(ns2->cfg->dev, "Failed to configure controller\n");
			ns2->init_retries++;
		}
		return -EIO;
	}

	ns2->init_retries++;
	switch (ns2->init_step) {
	case NS2_INIT_READ_SERIAL:
		return switch2_read_flash(ns2, NS2_FLASH_ADDR_SERIAL,
			NS2_FLASH_SIZE_SERIAL);
	case NS2_INIT_GET_FIRMWARE_INFO:
		return ns2->cfg->send_command(NS2_CMD_FW_INFO, NS2_SUBCMD_FW_INFO_GET,
			NULL, 0, ns2->cfg);
	case NS2_INIT_READ_FACTORY_PRIMARY_CALIB:
		return switch2_read_flash(ns2, NS2_FLASH_ADDR_FACTORY_PRIMARY_CALIB,
			NS2_FLASH_SIZE_FACTORY_AXIS_CALIB);
	case NS2_INIT_READ_FACTORY_SECONDARY_CALIB:
		if (switch2_ctlr_is_joycon(ns2->ctlr_type)) {
			switch2_init_step_done(ns2, ns2->init_step);
			return switch2_init_controller(ns2);
		}
		return switch2_read_flash(ns2, NS2_FLASH_ADDR_FACTORY_SECONDARY_CALIB,
			NS2_FLASH_SIZE_FACTORY_AXIS_CALIB);
	case NS2_INIT_READ_FACTORY_TRIGGER_CALIB:
		if (ns2->ctlr_type != NS2_CTLR_TYPE_GC) {
			switch2_init_step_done(ns2, ns2->init_step);
			return switch2_init_controller(ns2);
		}
		return switch2_read_flash(ns2, NS2_FLASH_ADDR_FACTORY_TRIGGER_CALIB,
			NS2_FLASH_SIZE_FACTORY_TRIGGER_CALIB);
	case NS2_INIT_READ_USER_PRIMARY_CALIB:
		return switch2_read_flash(ns2, NS2_FLASH_ADDR_USER_PRIMARY_CALIB,
			NS2_FLASH_SIZE_USER_AXIS_CALIB);
	case NS2_INIT_READ_USER_SECONDARY_CALIB:
		if (switch2_ctlr_is_joycon(ns2->ctlr_type)) {
			switch2_init_step_done(ns2, ns2->init_step);
			return switch2_init_controller(ns2);
		}
		return switch2_read_flash(ns2, NS2_FLASH_ADDR_USER_SECONDARY_CALIB,
			NS2_FLASH_SIZE_USER_AXIS_CALIB);
	case NS2_INIT_SET_FEATURE_MASK:
		return ns2->cfg->send_command(NS2_CMD_FEATSEL, NS2_SUBCMD_FEATSEL_SET_MASK,
			switch2_feature_mask, sizeof(switch2_feature_mask), ns2->cfg);
	case NS2_INIT_ENABLE_FEATURES:
		return switch2_features_enable(ns2, NS2_FEATURE_BUTTONS |
			NS2_FEATURE_ANALOG | NS2_FEATURE_RUMBLE);
	case NS2_INIT_ENABLE_RUMBLE:
		/*
		 * It is unclear what this packet is supposed to be for, but it
		 * appears to be needed for rumble to work reliably. The reply
		 * data indicates it might be a query of some sort, but we
		 * ignore the reply so long as it doesn't return an error.
		 */
		return ns2->cfg->send_command(0x11, 1, NULL, 0, ns2->cfg);
	case NS2_INIT_GRIP_BUTTONS:
		if (!switch2_ctlr_is_joycon(ns2->ctlr_type)) {
			switch2_init_step_done(ns2, ns2->init_step);
			return switch2_init_controller(ns2);
		}
		return ns2->cfg->send_command(NS2_CMD_GRIP, NS2_SUBCMD_GRIP_ENABLE_BUTTONS,
			switch2_one_data, sizeof(switch2_one_data),
			ns2->cfg);
	case NS2_INIT_REPORT_FORMAT:
		switch (ns2->ctlr_type) {
		case NS2_CTLR_TYPE_JCL:
			return switch2_set_report_format(ns2, NS2_REPORT_JCL);
		case NS2_CTLR_TYPE_JCR:
			return switch2_set_report_format(ns2, NS2_REPORT_JCR);
		case NS2_CTLR_TYPE_PRO:
			return switch2_set_report_format(ns2, NS2_REPORT_PRO);
		case NS2_CTLR_TYPE_GC:
			return switch2_set_report_format(ns2, NS2_REPORT_GC);
		default:
			switch2_init_step_done(ns2, ns2->init_step);
			return switch2_init_controller(ns2);
		}
	case NS2_INIT_INPUT:
		if (ns2->hdev)
			return switch2_init_input(ns2);
		break;
	case NS2_INIT_SET_PLAYER_LEDS:
		return switch2_set_player_id(ns2, ns2->player_id);
	case NS2_INIT_FINISH:
		return ns2->cfg->send_command(NS2_CMD_INIT, NS2_SUBCMD_INIT_USB,
			switch2_init_cmd_data, sizeof(switch2_init_cmd_data), ns2->cfg);
	default:
		WARN_ON_ONCE(1);
		break;
	}
	return 0;
}

int switch2_receive_command(struct switch2_controller *ns2,
	const uint8_t *message, size_t length)
{
	const struct switch2_cmd_header *header;
	int ret = 0;

	if (length < 8)
		return -EINVAL;

	print_hex_dump_debug("got cmd: ", DUMP_PREFIX_OFFSET, 16, 1, message, length, false);

	mutex_lock(&ns2->lock);

	header = (const struct switch2_cmd_header *)message;
	if (!(header->flags & NS2_FLAG_OK)) {
		if (ns2->cfg)
			dev_warn(ns2->cfg->dev, "Packet error %02x replying to command %x:%x",
				header->flags, header->command, header->subcommand);
		ret = -EIO;
		goto exit;
	}
	message = &message[8];
	length -= 8;

	switch (header->command) {
	case NS2_CMD_FLASH:
		if (header->subcommand == NS2_SUBCMD_FLASH_READ) {
			uint8_t read_size;
			uint32_t read_address;

			if (length < 8) {
				ret = -EINVAL;
				goto exit;
			}
			read_size = message[0];
			read_address = get_unaligned_le32(&message[4]);
			if (length < read_size + 8) {
				ret = -EINVAL;
				goto exit;
			}
			switch2_handle_flash_read(ns2, read_size, read_address, &message[8]);
		}
		break;
	case NS2_CMD_INIT:
		if (header->subcommand == NS2_SUBCMD_INIT_USB)
			switch2_init_step_done(ns2, NS2_INIT_FINISH);
		else if (header->subcommand == NS2_SUBCMD_INIT_SELECT_REPORT)
			switch2_init_step_done(ns2, NS2_INIT_REPORT_FORMAT);
		break;
	case NS2_CMD_GRIP:
		if (header->subcommand == NS2_SUBCMD_GRIP_ENABLE_BUTTONS)
			switch2_init_step_done(ns2, NS2_INIT_GRIP_BUTTONS);
		break;
	case NS2_CMD_LED:
		if (header->subcommand == NS2_SUBCMD_LED_PATTERN)
			switch2_init_step_done(ns2, NS2_INIT_SET_PLAYER_LEDS);
		break;
	case NS2_CMD_FEATSEL:
		if (header->subcommand == NS2_SUBCMD_FEATSEL_SET_MASK)
			switch2_init_step_done(ns2, NS2_INIT_SET_FEATURE_MASK);
		else if (header->subcommand == NS2_SUBCMD_FEATSEL_ENABLE)
			switch2_init_step_done(ns2, NS2_INIT_ENABLE_FEATURES);
		break;
	case NS2_CMD_FW_INFO:
		if (header->subcommand == NS2_SUBCMD_FW_INFO_GET) {
			if (length < sizeof(ns2->version)) {
				ret = -EINVAL;
				goto exit;
			}
			memcpy(&ns2->version, message, sizeof(ns2->version));
			ns2->ctlr_type = ns2->version.ctlr_type;
			switch2_init_step_done(ns2, NS2_INIT_GET_FIRMWARE_INFO);
		}
		break;
	case 0x11:
		if (header->subcommand == 1)
			switch2_init_step_done(ns2, NS2_INIT_ENABLE_RUMBLE);
		break;
	default:
		break;
	}

exit:
	if (ns2->init_step < NS2_INIT_DONE)
		switch2_init_controller(ns2);

	mutex_unlock(&ns2->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(switch2_receive_command);

int switch2_controller_attach_cfg(const char *phys, struct switch2_cfg_intf *cfg)
{
	struct switch2_controller *ns2 = switch2_get_controller(phys);
	int ret = 0;

	if (IS_ERR(ns2))
		return PTR_ERR(ns2);

	mutex_lock(&ns2->lock);
	if (ns2->cfg) {
		ret = -EBUSY;
		goto out;
	}
	cfg->parent = ns2;
	ns2->cfg = cfg;

	if (ns2->hdev)
		ret = switch2_init_controller(ns2);

	if (ret < 0) {
		if (cfg->shutdown)
			cfg->shutdown(cfg);
		cfg->parent = NULL;
		ns2->cfg = NULL;
	}

out:
	mutex_unlock(&ns2->lock);

	if (ret < 0)
		kref_put(&ns2->refcount, switch2_kref_put);
	return ret;
}
EXPORT_SYMBOL_GPL(switch2_controller_attach_cfg);

void switch2_controller_detach_cfg(struct switch2_controller *ns2)
{
	mutex_lock(&ns2->lock);
	if (!ns2->cfg || WARN_ON(ns2 != ns2->cfg->parent)) {
		mutex_unlock(&ns2->lock);
		return;
	}
	if (ns2->cfg->shutdown)
		ns2->cfg->shutdown(ns2->cfg);
	ns2->cfg->parent = NULL;
	ns2->cfg = NULL;
	mutex_unlock(&ns2->lock);
	switch2_controller_put(ns2);
	kref_put(&ns2->refcount, switch2_kref_put);
}
EXPORT_SYMBOL_GPL(switch2_controller_detach_cfg);

static int switch2_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct switch2_controller *ns2;
	struct usb_device *udev;
	char phys[64];
	int ret;

	if (!hid_is_usb(hdev))
		return -ENODEV;

	udev = hid_to_usb_dev(hdev);
	if (usb_make_path(udev, phys, sizeof(phys)) < 0)
		return -EINVAL;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed %d\n", ret);
		return ret;
	}

	ns2 = switch2_get_controller(phys);
	if (IS_ERR(ns2))
		return PTR_ERR(ns2);

	mutex_lock(&ns2->lock);
	if (ns2->hdev) {
		mutex_unlock(&ns2->lock);
		hid_err(hdev,
			"Second hdev tried to claim same controller, first=%p vs second=%p\n",
			ns2->hdev, hdev);
		kref_put(&ns2->refcount, switch2_kref_put);
		return -EBUSY;
	}
	ns2->hdev = hdev;
	hid_set_drvdata(hdev, ns2);

	switch (hdev->product | (hdev->vendor << 16)) {
	default:
		strscpy(ns2->name, hdev->name, sizeof(ns2->name));
		break;
	/* Some controllers have slightly wrong names so we override them */
	case USB_DEVICE_ID_NINTENDO_NS2_JOYCONR | (USB_VENDOR_ID_NINTENDO << 16):
		/* Missing the "2" in the name */
		strscpy(ns2->name, "Nintendo Joy-Con 2 (R)", sizeof(ns2->name));
		break;
	case USB_DEVICE_ID_NINTENDO_NS2_GCCON | (USB_VENDOR_ID_NINTENDO << 16):
		/* Has "Nintendo" in the name twice */
		strscpy(ns2->name, "Nintendo GameCube Controller", sizeof(ns2->name));
		break;
	}

	ns2->player_id = U32_MAX;
	ret = ida_alloc(&nintendo_player_id_allocator, GFP_KERNEL);
	if (ret < 0)
		hid_warn(hdev, "Failed to allocate player ID, skipping; ret=%d\n", ret);
	else
		ns2->player_id = ret;

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "hw_start failed %d\n", ret);
		goto err_cleanup;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hw_open failed %d\n", ret);
		goto err_stop;
	}

	ret = 0;
	if (ns2->cfg)
		ret = switch2_init_controller(ns2);

	if (!ret) {
		mutex_unlock(&ns2->lock);
		return 0;
	}

	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
err_cleanup:
	ida_free(&nintendo_player_id_allocator, ns2->player_id);
	ns2->hdev = NULL;
	mutex_unlock(&ns2->lock);
	switch2_controller_put(ns2);
	kref_put(&ns2->refcount, switch2_kref_put);

	return ret;
}

static void switch2_remove(struct hid_device *hdev)
{
	struct switch2_controller *ns2 = hid_get_drvdata(hdev);

	switch2_controller_put(ns2);
	mutex_lock(&ns2->lock);
	ns2->hdev = NULL;
	ida_free(&nintendo_player_id_allocator, ns2->player_id);
	mutex_unlock(&ns2->lock);
	kref_put(&ns2->refcount, switch2_kref_put);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id nintendo_hid_devices[] = {
	/* Switch devices */
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_PROCON) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_SNESCON) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_GENCON) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_N64CON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_PROCON) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_CHRGGRIP) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_JOYCONL) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_JOYCONR) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_SNESCON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_GENCON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_N64CON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_HORI,
			 USB_DEVICE_ID_HORI_WIRELESS_SWITCH_PAD) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_PDP,
			 USB_DEVICE_ID_PDP_SWITCH_AFTERGLOW) },
	/* Switch 2 devices */
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_NS2_JOYCONL) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_NS2_JOYCONR) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_NS2_PROCON) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_NS2_GCCON) },
	{ }
};
MODULE_DEVICE_TABLE(hid, nintendo_hid_devices);

static bool nintendo_is_switch2(struct hid_device *hdev)
{
	return hdev->vendor == USB_VENDOR_ID_NINTENDO &&
		hdev->product >= USB_DEVICE_ID_NINTENDO_NS2_JOYCONR;
}

static void nintendo_hid_remove(struct hid_device *hdev)
{
	if (nintendo_is_switch2(hdev))
		switch2_remove(hdev);
	else
		joycon_remove(hdev);
}

static int nintendo_hid_event(struct hid_device *hdev,
			      struct hid_report *report, u8 *raw_data, int size)
{
	if (nintendo_is_switch2(hdev))
		return switch2_event(hdev, report, raw_data, size);
	else
		return joycon_event(hdev, report, raw_data, size);
}

static const __u8 *nintendo_hid_report_fixup(struct hid_device *hdev,
			    __u8 *rdesc, unsigned int *rsize)
{
	if (hdev->vendor == USB_VENDOR_ID_NINTENDO &&
		hdev->product == USB_DEVICE_ID_NINTENDO_NS2_JOYCONR &&
		*rsize == 100 && rdesc[24] == NS2_REPORT_JCL)
		/*
		 * The NYXI Hyperion 3 right Joy-Con accidentally reports as the
		 * left one. Since the HID subsystem rejects report IDs that
		 * aren't listed in the descriptor, we need to patch it.
		 */
		rdesc[24] = NS2_REPORT_JCR;
	return rdesc;
}

static int nintendo_hid_probe(struct hid_device *hdev,
			    const struct hid_device_id *id)
{
	if (nintendo_is_switch2(hdev))
		return switch2_probe(hdev, id);
	else
		return joycon_probe(hdev, id);
}

static int nintendo_hid_resume(struct hid_device *hdev)
{
	if (nintendo_is_switch2(hdev))
		return 0;
	else
		return joycon_resume(hdev);
}

static int nintendo_hid_suspend(struct hid_device *hdev, pm_message_t message)
{
	if (nintendo_is_switch2(hdev))
		return 0;
	else
		return joycon_suspend(hdev, message);
}

static struct hid_driver nintendo_hid_driver = {
	.name		= "nintendo",
	.id_table	= nintendo_hid_devices,
	.report_fixup	= nintendo_hid_report_fixup,
	.probe		= nintendo_hid_probe,
	.remove		= nintendo_hid_remove,
	.raw_event	= nintendo_hid_event,
	.resume		= pm_ptr(nintendo_hid_resume),
	.suspend	= pm_ptr(nintendo_hid_suspend),
};
static int __init nintendo_init(void)
{
	return hid_register_driver(&nintendo_hid_driver);
}

static void __exit nintendo_exit(void)
{
	hid_unregister_driver(&nintendo_hid_driver);
	ida_destroy(&nintendo_player_id_allocator);
}

module_init(nintendo_init);
module_exit(nintendo_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan McClelland <rymcclel@gmail.com>");
MODULE_AUTHOR("Emily Strickland <linux@emily.st>");
MODULE_AUTHOR("Daniel J. Ogorchock <djogorchock@gmail.com>");
MODULE_AUTHOR("Vicki Pfau <vi@endrift.com>");
MODULE_DESCRIPTION("Driver for Nintendo Switch Controllers");
