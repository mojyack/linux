// SPDX-License-Identifier: GPL-2.0
/*
 * ROHM BH1730FVC ambient light sensor driver
 *
 * Copyright (c) 2026 mojyack <mojyack@gmail.com>
 *
 * Based on a previous BH1730FVC driver:
 * Copyright (c) 2018 Google, Inc.
 * Author: Pierre Bourdon <delroth@google.com>
 *
 * Based on a previous non-iio BH1730FVC driver:
 * Copyright (C) 2012 Samsung Electronics. All rights reserved.
 * Author: Won Huh <won.huh@samsung.com>
 *
 * Datasheet:
 *  https://fscdn.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1730fvc-e.pdf
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#define BH1730_CMD			BIT(7)
#define BH1730_CMD_SW_RESET		(BH1730_CMD | 0x64)

#define BH1730_REG_CONTROL		0x00
#define BH1730_REG_TIMING		0x01
#define BH1730_REG_GAIN			0x07
#define BH1730_REG_DATA0LOW		0x14

#define BH1730_CONTROL_POWER		BIT(0)
#define BH1730_CONTROL_ADC_EN		BIT(1)
#define BH1730_CONTROL_ONE_TIME		BIT(3)
#define BH1730_CONTROL_ADC_VALID	BIT(4)

/* The power-on default, 38 cycles: the datasheet's lux formula reference */
#define BH1730_ITIME			0xda
#define BH1730_ITIME_US			102570
/* Measurement time, (964 * 38 + 714) internal clocks of 2.8 us, 150 ms max */
#define BH1730_TMT_US			104569
#define BH1730_TMT_MAX_US		150000

#define BH1730_POWER_ON_US		2000

/* Below this peak count, a higher gain is worth another conversion */
#define BH1730_PEAK_LOW			1024

enum {
	BH1730_DATA0,	/* visible */
	BH1730_DATA1,	/* infrared */
};

static const int bh1730_gains[] = { 1, 2, 64, 128 };

/* DATA1 / DATA0 * 1000 upper bound, DATA0 and DATA1 coefficients * 1000 */
static const struct {
	u16 ratio;
	u16 c0;
	u16 c1;
} bh1730_lux_coeffs[] = {
	{  260, 1290, 2733 },
	{  550,  795,  859 },
	{ 1090,  510,  345 },
	{ 2130,  276,  130 },
};

struct bh1730_data {
	struct i2c_client *client;
	/* Serializes conversions and the settings below */
	struct mutex lock;
	unsigned int gain;	/* index into bh1730_gains, for the raw channels */
	unsigned int lux_gain;	/* last gain the illuminance conversion settled on */
	int calibscale;		/* micro */
};

static int bh1730_write(struct bh1730_data *data, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(data->client, BH1730_CMD | reg, val);
}

static int bh1730_measure(struct bh1730_data *data, unsigned int gain,
			  u16 counts[2])
{
	struct i2c_client *client = data->client;
	u8 buf[4];
	int ret, ctl;

	/* Stop a conversion a failed read left behind, clear its ADC_VALID */
	ret = bh1730_write(data, BH1730_REG_CONTROL, 0);
	if (ret)
		return ret;

	ret = i2c_smbus_read_byte_data(client,
				       BH1730_CMD | BH1730_REG_DATA0LOW);
	if (ret < 0)
		return ret;

	ret = bh1730_write(data, BH1730_REG_GAIN, gain);
	if (ret)
		return ret;

	ret = bh1730_write(data, BH1730_REG_TIMING, BH1730_ITIME);
	if (ret)
		return ret;

	ret = bh1730_write(data, BH1730_REG_CONTROL,
			   BH1730_CONTROL_POWER | BH1730_CONTROL_ADC_EN |
			   BH1730_CONTROL_ONE_TIME);
	if (ret)
		goto stop;

	fsleep(BH1730_TMT_US);

	ret = read_poll_timeout(i2c_smbus_read_byte_data, ctl,
				ctl < 0 || ctl & BH1730_CONTROL_ADC_VALID,
				2000, BH1730_TMT_MAX_US - BH1730_TMT_US, false,
				client, BH1730_CMD | BH1730_REG_CONTROL);
	if (!ret && ctl < 0)
		ret = ctl;
	if (ret)
		goto stop;

	ret = i2c_smbus_read_i2c_block_data(client,
					    BH1730_CMD | BH1730_REG_DATA0LOW,
					    sizeof(buf), buf);
	if (ret < 0)
		return ret;
	if (ret != sizeof(buf))
		return -EIO;

	counts[BH1730_DATA0] = get_unaligned_le16(&buf[0]);
	counts[BH1730_DATA1] = get_unaligned_le16(&buf[2]);

	return 0;

stop:
	bh1730_write(data, BH1730_REG_CONTROL, 0);
	return ret;
}

static unsigned int bh1730_next_gain(unsigned int gain, unsigned int peak)
{
	unsigned int i;

	if (peak == U16_MAX)
		return 0;

	if (peak >= BH1730_PEAK_LOW)
		return gain;

	/* The highest gain that keeps the peak below half scale */
	for (i = ARRAY_SIZE(bh1730_gains) - 1; i > gain; i--)
		if (peak * bh1730_gains[i] / bh1730_gains[gain] < BIT(15))
			return i;

	return gain;
}

static int bh1730_read_lux(struct bh1730_data *data, int *val, int *val2)
{
	unsigned int gain = data->lux_gain, next, ratio, i, tries;
	u16 counts[2];
	u64 microlux;
	u32 rem;
	s32 num;
	int ret;

	for (tries = 0; ; tries++) {
		ret = bh1730_measure(data, gain, counts);
		if (ret)
			return ret;

		next = bh1730_next_gain(gain, max(counts[BH1730_DATA0],
						  counts[BH1730_DATA1]));
		if (next == gain || tries == 2)
			break;

		gain = next;
	}
	data->lux_gain = next;

	if (counts[BH1730_DATA0] == U16_MAX || counts[BH1730_DATA1] == U16_MAX)
		return -ERANGE;

	num = 0;
	if (counts[BH1730_DATA0]) {
		ratio = counts[BH1730_DATA1] * 1000 / counts[BH1730_DATA0];
		for (i = 0; i < ARRAY_SIZE(bh1730_lux_coeffs); i++) {
			if (ratio < bh1730_lux_coeffs[i].ratio) {
				num = bh1730_lux_coeffs[i].c0 * counts[BH1730_DATA0] -
				      bh1730_lux_coeffs[i].c1 * counts[BH1730_DATA1];
				break;
			}
		}
	}

	/* Past its zero crossing, the last segment joins the 0 lx tail */
	num = max(num, 0);

	microlux = div_u64((u64)num * data->calibscale,
			   1000 * bh1730_gains[gain]);
	*val = div_u64_rem(microlux, MICRO, &rem);
	*val2 = rem;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int bh1730_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct bh1730_data *data = iio_priv(indio_dev);
	u16 counts[2];
	int ret;

	guard(mutex)(&data->lock);

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		return bh1730_read_lux(data, val, val2);
	case IIO_CHAN_INFO_RAW:
		ret = bh1730_measure(data, data->gain, counts);
		if (ret)
			return ret;
		*val = counts[chan->address];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = bh1730_gains[data->gain];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_INT_TIME:
		*val = 0;
		*val2 = BH1730_ITIME_US;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = data->calibscale / MICRO;
		*val2 = data->calibscale % MICRO;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int bh1730_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct bh1730_data *data = iio_priv(indio_dev);
	unsigned int i;

	guard(mutex)(&data->lock);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (val2)
			return -EINVAL;
		for (i = 0; i < ARRAY_SIZE(bh1730_gains); i++) {
			if (val == bh1730_gains[i]) {
				data->gain = i;
				return 0;
			}
		}
		return -EINVAL;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val < 0 || val >= 1000 || val2 < 0 || (!val && !val2))
			return -EINVAL;
		data->calibscale = val * MICRO + val2;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bh1730_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*vals = bh1730_gains;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(bh1730_gains);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static const struct iio_info bh1730_info = {
	.read_raw = bh1730_read_raw,
	.write_raw = bh1730_write_raw,
	.read_avail = bh1730_read_avail,
};

#define BH1730_INTENSITY_CHANNEL(_mod, _addr) {				\
	.type = IIO_INTENSITY,						\
	.modified = 1,							\
	.channel2 = _mod,						\
	.address = _addr,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |	\
				    BIT(IIO_CHAN_INFO_INT_TIME),	\
	.info_mask_shared_by_type_available =				\
		BIT(IIO_CHAN_INFO_HARDWAREGAIN),			\
}

static const struct iio_chan_spec bh1730_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_CALIBSCALE),
	},
	BH1730_INTENSITY_CHANNEL(IIO_MOD_LIGHT_CLEAR, BH1730_DATA0),
	BH1730_INTENSITY_CHANNEL(IIO_MOD_LIGHT_IR, BH1730_DATA1),
};

static int bh1730_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct bh1730_data *data;
	struct iio_dev *indio_dev;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	data->calibscale = MICRO;

	ret = devm_mutex_init(dev, &data->lock);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable VCC\n");

	ret = devm_regulator_get_enable(dev, "dvi");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable DVI\n");

	fsleep(BH1730_POWER_ON_US);

	/* Every register to its default, ADC powered down */
	ret = i2c_smbus_write_byte(client, BH1730_CMD_SW_RESET);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reset\n");

	indio_dev->info = &bh1730_info;
	indio_dev->name = "bh1730";
	indio_dev->channels = bh1730_channels;
	indio_dev->num_channels = ARRAY_SIZE(bh1730_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id bh1730_of_match[] = {
	{ .compatible = "rohm,bh1730fvc" },
	{ }
};
MODULE_DEVICE_TABLE(of, bh1730_of_match);

static struct i2c_driver bh1730_driver = {
	.probe = bh1730_probe,
	.driver = {
		.name = "bh1730",
		.of_match_table = bh1730_of_match,
	},
};
module_i2c_driver(bh1730_driver);

MODULE_AUTHOR("mojyack <mojyack@gmail.com>");
MODULE_AUTHOR("Pierre Bourdon <delroth@google.com>");
MODULE_DESCRIPTION("ROHM BH1730FVC ambient light sensor driver");
MODULE_LICENSE("GPL");
