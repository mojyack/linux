/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Definitions shared by the Nintendo Switch controller drivers.
 *
 * The same protocol however the controller is attached, so hid-nintendo and
 * joycon must present an identical evdev contract; joycond depends on it.
 */

#ifndef _LINUX_NINTENDO_JOYCON_H
#define _LINUX_NINTENDO_JOYCON_H

#include <linux/bits.h>
#include <linux/input.h>
#include <linux/math.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* USB product ids, also used to identify the controller to userspace */
#define JC_PRODUCT_JOYCONL		 0x2006
#define JC_PRODUCT_JOYCONR		 0x2007
#define JC_PRODUCT_PROCON		 0x2009
#define JC_VENDOR_NINTENDO		 0x057e

/* Controller type received as part of device info */
enum joycon_ctlr_type {
	JOYCON_CTLR_TYPE_JCL  = 0x01,
	JOYCON_CTLR_TYPE_JCR  = 0x02,
	JOYCON_CTLR_TYPE_PRO  = 0x03,
	JOYCON_CTLR_TYPE_LIC_PRO = 0x06,
	JOYCON_CTLR_TYPE_NESL = 0x09,
	JOYCON_CTLR_TYPE_NESR = 0x0A,
	JOYCON_CTLR_TYPE_SNES = 0x0B,
	JOYCON_CTLR_TYPE_GEN  = 0x0D,
	JOYCON_CTLR_TYPE_N64  = 0x0C,
};

/*
 * All the controller's button values are stored in a u32.
 * They can be accessed with bitwise ANDs.
 */
#define JC_BTN_Y	 BIT(0)
#define JC_BTN_X	 BIT(1)
#define JC_BTN_B	 BIT(2)
#define JC_BTN_A	 BIT(3)
#define JC_BTN_SR_R	 BIT(4)
#define JC_BTN_SL_R	 BIT(5)
#define JC_BTN_R	 BIT(6)
#define JC_BTN_ZR	 BIT(7)
#define JC_BTN_MINUS	 BIT(8)
#define JC_BTN_PLUS	 BIT(9)
#define JC_BTN_RSTICK	 BIT(10)
#define JC_BTN_LSTICK	 BIT(11)
#define JC_BTN_HOME	 BIT(12)
#define JC_BTN_CAP	 BIT(13) /* capture button */
#define JC_BTN_DOWN	 BIT(16)
#define JC_BTN_UP	 BIT(17)
#define JC_BTN_RIGHT	 BIT(18)
#define JC_BTN_LEFT	 BIT(19)
#define JC_BTN_SR_L	 BIT(20)
#define JC_BTN_SL_L	 BIT(21)
#define JC_BTN_L	 BIT(22)
#define JC_BTN_ZL	 BIT(23)

/* Magic value denoting presence of user calibration */
#define JC_CAL_USR_MAGIC_0		 0xB2
#define JC_CAL_USR_MAGIC_1		 0xA1
#define JC_CAL_USR_MAGIC_SIZE		 2

/* SPI storage addresses of user calibration data */
#define JC_CAL_USR_LEFT_MAGIC_ADDR	 0x8010
#define JC_CAL_USR_LEFT_DATA_ADDR	 0x8012
#define JC_CAL_USR_LEFT_DATA_END	 0x801A
#define JC_CAL_USR_RIGHT_MAGIC_ADDR	 0x801B
#define JC_CAL_USR_RIGHT_DATA_ADDR	 0x801D
#define JC_CAL_STICK_DATA_SIZE \
	(JC_CAL_USR_LEFT_DATA_END - JC_CAL_USR_LEFT_DATA_ADDR + 1)

/* SPI storage addresses of factory calibration data */
#define JC_CAL_FCT_DATA_LEFT_ADDR	 0x603d
#define JC_CAL_FCT_DATA_RIGHT_ADDR	 0x6046

/* The raw analog joystick values will be mapped in terms of this magnitude */
#define JC_MAX_STICK_MAG		 32767
#define JC_STICK_FUZZ			 250
#define JC_STICK_FLAT			 500

struct joycon_stick_cal {
	s32 max;
	s32 min;
	s32 center;
};

struct joycon_ctlr_button_mapping {
	u32 code;
	u32 bit;
};

/*
 * D-pad is configured as buttons for the left Joy-Con only!
 */
static const struct joycon_ctlr_button_mapping left_joycon_button_mappings[] = {
	{ BTN_TL,		JC_BTN_L,	},
	{ BTN_TL2,		JC_BTN_ZL,	},
	{ BTN_SELECT,		JC_BTN_MINUS,	},
	{ BTN_THUMBL,		JC_BTN_LSTICK,	},
	{ BTN_DPAD_UP,		JC_BTN_UP,	},
	{ BTN_DPAD_DOWN,	JC_BTN_DOWN,	},
	{ BTN_DPAD_LEFT,	JC_BTN_LEFT,	},
	{ BTN_DPAD_RIGHT,	JC_BTN_RIGHT,	},
	{ BTN_Z,		JC_BTN_CAP,	},
	{ /* sentinel */ },
};

/*
 * The unused *right*-side triggers become the SL/SR triggers for the *left*
 * Joy-Con, if and only if we're not using a charging grip.
 */
static const struct joycon_ctlr_button_mapping left_joycon_s_button_mappings[] = {
	{ BTN_TR,	JC_BTN_SL_L,	},
	{ BTN_TR2,	JC_BTN_SR_L,	},
	{ /* sentinel */ },
};

static const struct joycon_ctlr_button_mapping right_joycon_button_mappings[] = {
	{ BTN_EAST,	JC_BTN_A,	},
	{ BTN_SOUTH,	JC_BTN_B,	},
	{ BTN_NORTH,	JC_BTN_X,	},
	{ BTN_WEST,	JC_BTN_Y,	},
	{ BTN_TR,	JC_BTN_R,	},
	{ BTN_TR2,	JC_BTN_ZR,	},
	{ BTN_START,	JC_BTN_PLUS,	},
	{ BTN_THUMBR,	JC_BTN_RSTICK,	},
	{ BTN_MODE,	JC_BTN_HOME,	},
	{ /* sentinel */ },
};

/*
 * The unused *left*-side triggers become the SL/SR triggers for the *right*
 * Joy-Con, if and only if we're not using a charging grip.
 */
static const struct joycon_ctlr_button_mapping right_joycon_s_button_mappings[] = {
	{ BTN_TL,	JC_BTN_SL_R,	},
	{ BTN_TL2,	JC_BTN_SR_R,	},
	{ /* sentinel */ },
};

static const struct joycon_ctlr_button_mapping procon_button_mappings[] = {
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
	{ BTN_THUMBL,	JC_BTN_LSTICK,	},
	{ BTN_THUMBR,	JC_BTN_RSTICK,	},
	{ BTN_MODE,	JC_BTN_HOME,	},
	{ BTN_Z,	JC_BTN_CAP,	},
	{ /* sentinel */ },
};

/* Licensed Pro Controllers (e.g. HORI) swap X/Y bits in the report */
static const struct joycon_ctlr_button_mapping lic_procon_button_mappings[] = {
	{ BTN_EAST,	JC_BTN_A,	},
	{ BTN_SOUTH,	JC_BTN_B,	},
	{ BTN_NORTH,	JC_BTN_Y,	},
	{ BTN_WEST,	JC_BTN_X,	},
	{ BTN_TL,	JC_BTN_L,	},
	{ BTN_TR,	JC_BTN_R,	},
	{ BTN_TL2,	JC_BTN_ZL,	},
	{ BTN_TR2,	JC_BTN_ZR,	},
	{ BTN_SELECT,	JC_BTN_MINUS,	},
	{ BTN_START,	JC_BTN_PLUS,	},
	{ BTN_THUMBL,	JC_BTN_LSTICK,	},
	{ BTN_THUMBR,	JC_BTN_RSTICK,	},
	{ BTN_MODE,	JC_BTN_HOME,	},
	{ BTN_Z,	JC_BTN_CAP,	},
	{ /* sentinel */ },
};

static inline void joycon_config_buttons(struct input_dev *idev,
					 const struct joycon_ctlr_button_mapping button_mappings[])
{
	const struct joycon_ctlr_button_mapping *button;

	for (button = button_mappings; button->code; button++)
		input_set_capability(idev, EV_KEY, button->code);
}

static inline void joycon_config_left_stick(struct input_dev *idev)
{
	input_set_abs_params(idev, ABS_X, -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
			     JC_STICK_FUZZ, JC_STICK_FLAT);
	input_set_abs_params(idev, ABS_Y, -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
			     JC_STICK_FUZZ, JC_STICK_FLAT);
}

static inline void joycon_config_right_stick(struct input_dev *idev)
{
	input_set_abs_params(idev, ABS_RX, -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
			     JC_STICK_FUZZ, JC_STICK_FLAT);
	input_set_abs_params(idev, ABS_RY, -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
			     JC_STICK_FUZZ, JC_STICK_FLAT);
}

/* Scale a raw stick reading onto -JC_MAX_STICK_MAG..JC_MAX_STICK_MAG. */
static inline s32 joycon_map_stick_val(const struct joycon_stick_cal *cal, s32 val)
{
	s32 center = cal->center;
	s32 min = cal->min;
	s32 max = cal->max;
	s32 new_val;

	if (val > center) {
		new_val = (val - center) * JC_MAX_STICK_MAG;
		new_val /= (max - center);
	} else {
		new_val = (center - val) * -JC_MAX_STICK_MAG;
		new_val /= (center - min);
	}
	new_val = clamp(new_val, (s32)-JC_MAX_STICK_MAG, (s32)JC_MAX_STICK_MAG);
	return new_val;
}

/* Unpack a pair of 12-bit values from the three bytes holding them. */
static inline void joycon_unpack_stick_pair(const u8 *packed, s32 *first,
					    s32 *second)
{
	*first = packed[0] | ((packed[1] & 0xF) << 8);
	*second = (packed[1] >> 4) | (packed[2] << 4);
}

/*
 * JC_CAL_STICK_DATA_SIZE bytes of six 12-bit values: the left stick leads with
 * the deltas above centre, the right one with the centre itself.
 */
static inline void joycon_parse_stick_cal(struct joycon_stick_cal *cal_x,
					  struct joycon_stick_cal *cal_y,
					  const u8 *packed, bool left)
{
	s32 v[6];
	int i;

	for (i = 0; i < 3; i++)
		joycon_unpack_stick_pair(packed + i * 3, &v[i * 2],
					 &v[i * 2 + 1]);

	if (left) {
		cal_x->center = v[2];
		cal_y->center = v[3];
		cal_x->max = v[2] + v[0];
		cal_y->max = v[3] + v[1];
		cal_x->min = v[2] - v[4];
		cal_y->min = v[3] - v[5];
	} else {
		cal_x->center = v[0];
		cal_y->center = v[1];
		cal_x->min = v[0] - v[2];
		cal_y->min = v[1] - v[3];
		cal_x->max = v[0] + v[4];
		cal_y->max = v[1] + v[5];
	}
}

/* All-0xFF, or a centre outside its own range: the values are not usable. */
static inline bool joycon_stick_cal_valid(const struct joycon_stick_cal *cal_x,
					  const struct joycon_stick_cal *cal_y)
{
	const struct joycon_stick_cal *cal[2] = { cal_x, cal_y };
	int i;

	for (i = 0; i < 2; i++)
		if (cal[i]->min < 0 || cal[i]->max > 0xFFF ||
		    cal[i]->min >= cal[i]->center ||
		    cal[i]->center >= cal[i]->max)
			return false;

	return true;
}

/* SPI storage addresses of IMU factory calibration data */
#define JC_IMU_CAL_FCT_DATA_ADDR	 0x6020
#define JC_IMU_CAL_FCT_DATA_END	 0x6037
#define JC_IMU_CAL_DATA_SIZE \
	(JC_IMU_CAL_FCT_DATA_END - JC_IMU_CAL_FCT_DATA_ADDR + 1)
/* SPI storage addresses of IMU user calibration data */
#define JC_IMU_CAL_USR_MAGIC_ADDR	 0x8026
#define JC_IMU_CAL_USR_DATA_ADDR	 0x8028

/*
 * The controller's accelerometer has a sensor resolution of 16bits and is
 * configured with a range of +-8000 milliGs. Therefore, the resolution can be
 * calculated thus: (2^16-1)/(8000 * 2) = 4.096 digits per milliG
 * Resolution per G (rather than per millliG): 4.096 * 1000 = 4096 digits per G
 * Alternatively: 1/4096 = .0002441 Gs per digit
 */
#define JC_IMU_MAX_ACCEL_MAG		32767
#define JC_IMU_ACCEL_RES_PER_G		4096
#define JC_IMU_ACCEL_FUZZ		10
#define JC_IMU_ACCEL_FLAT		0

/*
 * The controller's gyroscope has a sensor resolution of 16bits and is
 * configured with a range of +-2000 degrees/second.
 * Digits per dps: (2^16 -1)/(2000*2) = 16.38375
 * dps per digit: 16.38375E-1 = .0610
 *
 * STMicro recommends in the datasheet to add 15% to the dps/digit. This allows
 * the full sensitivity range to be saturated without clipping. This yields more
 * accurate results, so it's the technique this driver uses.
 * dps per digit (corrected): .0610 * 1.15 = .0702
 * digits per dps (corrected): .0702E-1 = 14.247
 *
 * Now, 14.247 truncating to 14 loses a lot of precision, so we rescale the
 * min/max range by 1000.
 */
#define JC_IMU_PREC_RANGE_SCALE	1000
/* Note: change mag and res_per_dps if prec_range_scale is ever altered */
#define JC_IMU_MAX_GYRO_MAG		32767000 /* (2^16-1)*1000 */
#define JC_IMU_GYRO_RES_PER_DPS		14247 /* (14.247*1000) */
#define JC_IMU_GYRO_FUZZ		10
#define JC_IMU_GYRO_FLAT		0

/* Used until the flash has been read, and when what it holds is unusable */
#define JC_IMU_DFLT_ACCEL_OFFSET	0
#define JC_IMU_DFLT_ACCEL_SCALE		16384
#define JC_IMU_DFLT_GYRO_OFFSET		0
#define JC_IMU_DFLT_GYRO_SCALE		13371

struct joycon_imu_cal {
	s16 offset[3];
	s16 scale[3];
};

/* A full input report carries three of these behind the sticks. */
struct joycon_imu_data {
	s16 accel_x;
	s16 accel_y;
	s16 accel_z;
	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;
} __packed;

static inline void joycon_config_imu(struct input_dev *idev)
{
	static const u16 accel_axes[3] = { ABS_X, ABS_Y, ABS_Z };
	static const u16 gyro_axes[3] = { ABS_RX, ABS_RY, ABS_RZ };
	int i;

	for (i = 0; i < 3; i++) {
		input_set_abs_params(idev, accel_axes[i], -JC_IMU_MAX_ACCEL_MAG,
				     JC_IMU_MAX_ACCEL_MAG, JC_IMU_ACCEL_FUZZ,
				     JC_IMU_ACCEL_FLAT);
		input_abs_set_res(idev, accel_axes[i], JC_IMU_ACCEL_RES_PER_G);

		input_set_abs_params(idev, gyro_axes[i], -JC_IMU_MAX_GYRO_MAG,
				     JC_IMU_MAX_GYRO_MAG, JC_IMU_GYRO_FUZZ,
				     JC_IMU_GYRO_FLAT);
		input_abs_set_res(idev, gyro_axes[i], JC_IMU_GYRO_RES_PER_DPS);
	}

	__set_bit(EV_MSC, idev->evbit);
	__set_bit(MSC_TIMESTAMP, idev->mscbit);
	__set_bit(INPUT_PROP_ACCELEROMETER, idev->propbit);
}

static inline void joycon_imu_cal_defaults(struct joycon_imu_cal *accel,
					   struct joycon_imu_cal *gyro)
{
	int i;

	for (i = 0; i < 3; i++) {
		accel->offset[i] = JC_IMU_DFLT_ACCEL_OFFSET;
		accel->scale[i] = JC_IMU_DFLT_ACCEL_SCALE;
		gyro->offset[i] = JC_IMU_DFLT_GYRO_OFFSET;
		gyro->scale[i] = JC_IMU_DFLT_GYRO_SCALE;
	}
}

/* JC_IMU_CAL_DATA_SIZE bytes: the offsets then the scales, accelerometer first. */
static inline void joycon_parse_imu_cal(struct joycon_imu_cal *accel,
					struct joycon_imu_cal *gyro,
					const u8 *raw)
{
	int i;

	for (i = 0; i < 3; i++) {
		int j = i * 2;

		accel->offset[i] = get_unaligned_le16(raw + j);
		accel->scale[i] = get_unaligned_le16(raw + j + 6);
		gyro->offset[i] = get_unaligned_le16(raw + j + 12);
		gyro->scale[i] = get_unaligned_le16(raw + j + 18);
	}
}

/* An equal pair means the axis is uncalibrated; the raw reading is all we have. */
static inline s32 joycon_imu_cal_divisor(const struct joycon_imu_cal *cal, int axis)
{
	s32 divisor = cal->scale[axis] - cal->offset[axis];

	return divisor ? divisor : 1;
}

/*
 * These calculations (which use the controller's calibration settings to
 * improve the final values) are based on those found in the community's
 * reverse-engineering repo. The final value given to userspace is always in
 * terms of the axis resolution joycon_config_imu() provided.
 *
 * Currently only the gyro calculations subtract the calibration offsets from
 * the raw value itself. In testing, doing the same for the accelerometer raw
 * values decreased accuracy.
 */
static inline s32 joycon_map_accel_val(const struct joycon_imu_cal *cal, int axis,
				       s16 raw)
{
	return (s32)raw * cal->scale[axis] / joycon_imu_cal_divisor(cal, axis);
}

/*
 * The gyro values are multiplied by the precision-saving scaling factor to
 * prevent large inaccuracies due to truncation of the resolution value which
 * would otherwise occur. To prevent overflow (without resorting to 64 bit
 * integer math), the mult_frac macro is used.
 */
static inline s32 joycon_map_gyro_val(const struct joycon_imu_cal *cal, int axis,
				      s16 raw)
{
	return mult_frac(JC_IMU_PREC_RANGE_SCALE * (raw - cal->offset[axis]),
			 cal->scale[axis], joycon_imu_cal_divisor(cal, axis));
}

/* One sample as it sits in an input report: six little endian 16-bit values. */
static inline void joycon_parse_imu_data(const u8 *raw,
					 struct joycon_imu_data *data)
{
	data->accel_x = get_unaligned_le16(raw + 0);
	data->accel_y = get_unaligned_le16(raw + 2);
	data->accel_z = get_unaligned_le16(raw + 4);
	data->gyro_x = get_unaligned_le16(raw + 6);
	data->gyro_y = get_unaligned_le16(raw + 8);
	data->gyro_z = get_unaligned_le16(raw + 10);
}

/*
 * The right Joy-Con carries its IMU the other way up, so all but the X axes
 * are negated to agree with the left one and the Pro controller:
 *   X: positive is pointing toward the triggers
 *   Y: positive is pointing to the left
 *   Z: positive is pointing up (out of the buttons/sticks)
 * The axes follow the right-hand rule.
 */
static inline void joycon_report_imu_sample(struct input_dev *idev,
					    const struct joycon_imu_cal *accel_cal,
					    const struct joycon_imu_cal *gyro_cal,
					    const struct joycon_imu_data *data,
					    bool right, unsigned int timestamp_us)
{
	static const u16 accel_axes[3] = { ABS_X, ABS_Y, ABS_Z };
	static const u16 gyro_axes[3] = { ABS_RX, ABS_RY, ABS_RZ };
	const s16 accel_raw[3] = { data->accel_x, data->accel_y, data->accel_z };
	const s16 gyro_raw[3] = { data->gyro_x, data->gyro_y, data->gyro_z };
	int i;

	input_event(idev, EV_MSC, MSC_TIMESTAMP, timestamp_us);

	for (i = 0; i < 3; i++) {
		s32 accel = joycon_map_accel_val(accel_cal, i, accel_raw[i]);
		s32 gyro = joycon_map_gyro_val(gyro_cal, i, gyro_raw[i]);

		if (right && i != 0) {
			accel = -accel;
			gyro = -gyro;
		}

		input_report_abs(idev, accel_axes[i], accel);
		input_report_abs(idev, gyro_axes[i], gyro);
	}

	input_sync(idev);
}

/* Rumble is eight bytes: a frequency/amplitude pair for either half. */
#define JC_RUMBLE_DATA_SIZE		 8

/* frequency/amplitude tables for rumble */
struct joycon_rumble_freq_data {
	u16 high;
	u8 low;
	u16 freq; /* Hz*/
};

struct joycon_rumble_amp_data {
	u8 high;
	u16 low;
	u16 amp;
};

/*
 * These tables are from
 * https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md
 */
static const struct joycon_rumble_freq_data joycon_rumble_frequencies[] = {
	/* high, low, freq */
	{ 0x0000, 0x01,   41 }, { 0x0000, 0x02,   42 }, { 0x0000, 0x03,   43 },
	{ 0x0000, 0x04,   44 }, { 0x0000, 0x05,   45 }, { 0x0000, 0x06,   46 },
	{ 0x0000, 0x07,   47 }, { 0x0000, 0x08,   48 }, { 0x0000, 0x09,   49 },
	{ 0x0000, 0x0A,   50 }, { 0x0000, 0x0B,   51 }, { 0x0000, 0x0C,   52 },
	{ 0x0000, 0x0D,   53 }, { 0x0000, 0x0E,   54 }, { 0x0000, 0x0F,   55 },
	{ 0x0000, 0x10,   57 }, { 0x0000, 0x11,   58 }, { 0x0000, 0x12,   59 },
	{ 0x0000, 0x13,   60 }, { 0x0000, 0x14,   62 }, { 0x0000, 0x15,   63 },
	{ 0x0000, 0x16,   64 }, { 0x0000, 0x17,   66 }, { 0x0000, 0x18,   67 },
	{ 0x0000, 0x19,   69 }, { 0x0000, 0x1A,   70 }, { 0x0000, 0x1B,   72 },
	{ 0x0000, 0x1C,   73 }, { 0x0000, 0x1D,   75 }, { 0x0000, 0x1e,   77 },
	{ 0x0000, 0x1f,   78 }, { 0x0000, 0x20,   80 }, { 0x0400, 0x21,   82 },
	{ 0x0800, 0x22,   84 }, { 0x0c00, 0x23,   85 }, { 0x1000, 0x24,   87 },
	{ 0x1400, 0x25,   89 }, { 0x1800, 0x26,   91 }, { 0x1c00, 0x27,   93 },
	{ 0x2000, 0x28,   95 }, { 0x2400, 0x29,   97 }, { 0x2800, 0x2a,   99 },
	{ 0x2c00, 0x2b,  102 }, { 0x3000, 0x2c,  104 }, { 0x3400, 0x2d,  106 },
	{ 0x3800, 0x2e,  108 }, { 0x3c00, 0x2f,  111 }, { 0x4000, 0x30,  113 },
	{ 0x4400, 0x31,  116 }, { 0x4800, 0x32,  118 }, { 0x4c00, 0x33,  121 },
	{ 0x5000, 0x34,  123 }, { 0x5400, 0x35,  126 }, { 0x5800, 0x36,  129 },
	{ 0x5c00, 0x37,  132 }, { 0x6000, 0x38,  135 }, { 0x6400, 0x39,  137 },
	{ 0x6800, 0x3a,  141 }, { 0x6c00, 0x3b,  144 }, { 0x7000, 0x3c,  147 },
	{ 0x7400, 0x3d,  150 }, { 0x7800, 0x3e,  153 }, { 0x7c00, 0x3f,  157 },
	{ 0x8000, 0x40,  160 }, { 0x8400, 0x41,  164 }, { 0x8800, 0x42,  167 },
	{ 0x8c00, 0x43,  171 }, { 0x9000, 0x44,  174 }, { 0x9400, 0x45,  178 },
	{ 0x9800, 0x46,  182 }, { 0x9c00, 0x47,  186 }, { 0xa000, 0x48,  190 },
	{ 0xa400, 0x49,  194 }, { 0xa800, 0x4a,  199 }, { 0xac00, 0x4b,  203 },
	{ 0xb000, 0x4c,  207 }, { 0xb400, 0x4d,  212 }, { 0xb800, 0x4e,  217 },
	{ 0xbc00, 0x4f,  221 }, { 0xc000, 0x50,  226 }, { 0xc400, 0x51,  231 },
	{ 0xc800, 0x52,  236 }, { 0xcc00, 0x53,  241 }, { 0xd000, 0x54,  247 },
	{ 0xd400, 0x55,  252 }, { 0xd800, 0x56,  258 }, { 0xdc00, 0x57,  263 },
	{ 0xe000, 0x58,  269 }, { 0xe400, 0x59,  275 }, { 0xe800, 0x5a,  281 },
	{ 0xec00, 0x5b,  287 }, { 0xf000, 0x5c,  293 }, { 0xf400, 0x5d,  300 },
	{ 0xf800, 0x5e,  306 }, { 0xfc00, 0x5f,  313 }, { 0x0001, 0x60,  320 },
	{ 0x0401, 0x61,  327 }, { 0x0801, 0x62,  334 }, { 0x0c01, 0x63,  341 },
	{ 0x1001, 0x64,  349 }, { 0x1401, 0x65,  357 }, { 0x1801, 0x66,  364 },
	{ 0x1c01, 0x67,  372 }, { 0x2001, 0x68,  381 }, { 0x2401, 0x69,  389 },
	{ 0x2801, 0x6a,  397 }, { 0x2c01, 0x6b,  406 }, { 0x3001, 0x6c,  415 },
	{ 0x3401, 0x6d,  424 }, { 0x3801, 0x6e,  433 }, { 0x3c01, 0x6f,  443 },
	{ 0x4001, 0x70,  453 }, { 0x4401, 0x71,  462 }, { 0x4801, 0x72,  473 },
	{ 0x4c01, 0x73,  483 }, { 0x5001, 0x74,  494 }, { 0x5401, 0x75,  504 },
	{ 0x5801, 0x76,  515 }, { 0x5c01, 0x77,  527 }, { 0x6001, 0x78,  538 },
	{ 0x6401, 0x79,  550 }, { 0x6801, 0x7a,  562 }, { 0x6c01, 0x7b,  574 },
	{ 0x7001, 0x7c,  587 }, { 0x7401, 0x7d,  600 }, { 0x7801, 0x7e,  613 },
	{ 0x7c01, 0x7f,  626 }, { 0x8001, 0x00,  640 }, { 0x8401, 0x00,  654 },
	{ 0x8801, 0x00,  668 }, { 0x8c01, 0x00,  683 }, { 0x9001, 0x00,  698 },
	{ 0x9401, 0x00,  713 }, { 0x9801, 0x00,  729 }, { 0x9c01, 0x00,  745 },
	{ 0xa001, 0x00,  761 }, { 0xa401, 0x00,  778 }, { 0xa801, 0x00,  795 },
	{ 0xac01, 0x00,  812 }, { 0xb001, 0x00,  830 }, { 0xb401, 0x00,  848 },
	{ 0xb801, 0x00,  867 }, { 0xbc01, 0x00,  886 }, { 0xc001, 0x00,  905 },
	{ 0xc401, 0x00,  925 }, { 0xc801, 0x00,  945 }, { 0xcc01, 0x00,  966 },
	{ 0xd001, 0x00,  987 }, { 0xd401, 0x00, 1009 }, { 0xd801, 0x00, 1031 },
	{ 0xdc01, 0x00, 1053 }, { 0xe001, 0x00, 1076 }, { 0xe401, 0x00, 1100 },
	{ 0xe801, 0x00, 1124 }, { 0xec01, 0x00, 1149 }, { 0xf001, 0x00, 1174 },
	{ 0xf401, 0x00, 1199 }, { 0xf801, 0x00, 1226 }, { 0xfc01, 0x00, 1253 }
};

#define joycon_max_rumble_amp	(1003)
static const struct joycon_rumble_amp_data joycon_rumble_amplitudes[] = {
	/* high, low, amp */
	{ 0x00, 0x0040,    0 },
	{ 0x02, 0x8040,   10 }, { 0x04, 0x0041,   12 }, { 0x06, 0x8041,   14 },
	{ 0x08, 0x0042,   17 }, { 0x0a, 0x8042,   20 }, { 0x0c, 0x0043,   24 },
	{ 0x0e, 0x8043,   28 }, { 0x10, 0x0044,   33 }, { 0x12, 0x8044,   40 },
	{ 0x14, 0x0045,   47 }, { 0x16, 0x8045,   56 }, { 0x18, 0x0046,   67 },
	{ 0x1a, 0x8046,   80 }, { 0x1c, 0x0047,   95 }, { 0x1e, 0x8047,  112 },
	{ 0x20, 0x0048,  117 }, { 0x22, 0x8048,  123 }, { 0x24, 0x0049,  128 },
	{ 0x26, 0x8049,  134 }, { 0x28, 0x004a,  140 }, { 0x2a, 0x804a,  146 },
	{ 0x2c, 0x004b,  152 }, { 0x2e, 0x804b,  159 }, { 0x30, 0x004c,  166 },
	{ 0x32, 0x804c,  173 }, { 0x34, 0x004d,  181 }, { 0x36, 0x804d,  189 },
	{ 0x38, 0x004e,  198 }, { 0x3a, 0x804e,  206 }, { 0x3c, 0x004f,  215 },
	{ 0x3e, 0x804f,  225 }, { 0x40, 0x0050,  230 }, { 0x42, 0x8050,  235 },
	{ 0x44, 0x0051,  240 }, { 0x46, 0x8051,  245 }, { 0x48, 0x0052,  251 },
	{ 0x4a, 0x8052,  256 }, { 0x4c, 0x0053,  262 }, { 0x4e, 0x8053,  268 },
	{ 0x50, 0x0054,  273 }, { 0x52, 0x8054,  279 }, { 0x54, 0x0055,  286 },
	{ 0x56, 0x8055,  292 }, { 0x58, 0x0056,  298 }, { 0x5a, 0x8056,  305 },
	{ 0x5c, 0x0057,  311 }, { 0x5e, 0x8057,  318 }, { 0x60, 0x0058,  325 },
	{ 0x62, 0x8058,  332 }, { 0x64, 0x0059,  340 }, { 0x66, 0x8059,  347 },
	{ 0x68, 0x005a,  355 }, { 0x6a, 0x805a,  362 }, { 0x6c, 0x005b,  370 },
	{ 0x6e, 0x805b,  378 }, { 0x70, 0x005c,  387 }, { 0x72, 0x805c,  395 },
	{ 0x74, 0x005d,  404 }, { 0x76, 0x805d,  413 }, { 0x78, 0x005e,  422 },
	{ 0x7a, 0x805e,  431 }, { 0x7c, 0x005f,  440 }, { 0x7e, 0x805f,  450 },
	{ 0x80, 0x0060,  460 }, { 0x82, 0x8060,  470 }, { 0x84, 0x0061,  480 },
	{ 0x86, 0x8061,  491 }, { 0x88, 0x0062,  501 }, { 0x8a, 0x8062,  512 },
	{ 0x8c, 0x0063,  524 }, { 0x8e, 0x8063,  535 }, { 0x90, 0x0064,  547 },
	{ 0x92, 0x8064,  559 }, { 0x94, 0x0065,  571 }, { 0x96, 0x8065,  584 },
	{ 0x98, 0x0066,  596 }, { 0x9a, 0x8066,  609 }, { 0x9c, 0x0067,  623 },
	{ 0x9e, 0x8067,  636 }, { 0xa0, 0x0068,  650 }, { 0xa2, 0x8068,  665 },
	{ 0xa4, 0x0069,  679 }, { 0xa6, 0x8069,  694 }, { 0xa8, 0x006a,  709 },
	{ 0xaa, 0x806a,  725 }, { 0xac, 0x006b,  741 }, { 0xae, 0x806b,  757 },
	{ 0xb0, 0x006c,  773 }, { 0xb2, 0x806c,  790 }, { 0xb4, 0x006d,  808 },
	{ 0xb6, 0x806d,  825 }, { 0xb8, 0x006e,  843 }, { 0xba, 0x806e,  862 },
	{ 0xbc, 0x006f,  881 }, { 0xbe, 0x806f,  900 }, { 0xc0, 0x0070,  920 },
	{ 0xc2, 0x8070,  940 }, { 0xc4, 0x0071,  960 }, { 0xc6, 0x8071,  981 },
	{ 0xc8, 0x0072, joycon_max_rumble_amp }
};

#define JC_RUMBLE_DFLT_LOW_FREQ		 160
#define JC_RUMBLE_DFLT_HIGH_FREQ	 320
#define JC_RUMBLE_ZERO_AMP_PKT_CNT	 5

static inline struct joycon_rumble_freq_data joycon_find_rumble_freq(u16 freq)
{
	const size_t length = ARRAY_SIZE(joycon_rumble_frequencies);
	const struct joycon_rumble_freq_data *data = joycon_rumble_frequencies;
	int i = 0;

	if (freq > data[0].freq) {
		for (i = 1; i < length - 1; i++) {
			if (freq > data[i - 1].freq && freq <= data[i].freq)
				break;
		}
	}

	return data[i];
}

static inline struct joycon_rumble_amp_data joycon_find_rumble_amp(u16 amp)
{
	const size_t length = ARRAY_SIZE(joycon_rumble_amplitudes);
	const struct joycon_rumble_amp_data *data = joycon_rumble_amplitudes;
	int i = 0;

	if (amp > data[0].amp) {
		for (i = 1; i < length - 1; i++) {
			if (amp > data[i - 1].amp && amp <= data[i].amp)
				break;
		}
	}

	return data[i];
}

static inline void joycon_encode_rumble(u8 *data, u16 freq_low, u16 freq_high, u16 amp)
{
	struct joycon_rumble_freq_data freq_data_low;
	struct joycon_rumble_freq_data freq_data_high;
	struct joycon_rumble_amp_data amp_data;

	freq_data_low = joycon_find_rumble_freq(freq_low);
	freq_data_high = joycon_find_rumble_freq(freq_high);
	amp_data = joycon_find_rumble_amp(amp);

	data[0] = (freq_data_high.high >> 8) & 0xFF;
	data[1] = (freq_data_high.high & 0xFF) + amp_data.high;
	data[2] = freq_data_low.low + ((amp_data.low >> 8) & 0xFF);
	data[3] = amp_data.low & 0xFF;
}

#endif /* _LINUX_NINTENDO_JOYCON_H */
