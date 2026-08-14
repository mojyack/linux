// SPDX-License-Identifier: GPL-2.0-only
/*
 * ON/OFF key driver for Maxim MAX77620 PMICs
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77620.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>

struct max77620_onkey {
	struct input_dev *input;
	struct regmap *rmap;
};

static irqreturn_t max77620_onkey_irq(int irq, void *data)
{
	struct max77620_onkey *onkey = data;
	unsigned int edges, stat;
	bool pressed;
	int ret;

	/* ONOFFIRQ is clear-on-read: a bit left set masks its own source. */
	ret = regmap_read(onkey->rmap, MAX77620_REG_ONOFFIRQ, &edges);
	if (ret < 0) {
		dev_err(onkey->input->dev.parent,
			"failed to read ONOFFIRQ: %d\n", ret);
		return IRQ_NONE;
	}

	ret = regmap_read(onkey->rmap, MAX77620_REG_ONOFFSTAT, &stat);
	if (ret < 0) {
		dev_err(onkey->input->dev.parent,
			"failed to read ONOFFSTAT: %d\n", ret);
		return IRQ_NONE;
	}

	/* A tap that completed already leaves both edges latched at level 0. */
	pressed = stat & MAX77620_ONOFFSTAT_EN0;
	if (!pressed &&
	    (edges & (MAX77620_ONOFFIRQ_EN0_F | MAX77620_ONOFFIRQ_EN0_R)) ==
	    (MAX77620_ONOFFIRQ_EN0_F | MAX77620_ONOFFIRQ_EN0_R)) {
		input_report_key(onkey->input, KEY_POWER, 1);
		input_sync(onkey->input);
	}

	input_report_key(onkey->input, KEY_POWER, pressed);
	input_sync(onkey->input);

	return IRQ_HANDLED;
}

static int max77620_onkey_probe(struct platform_device *pdev)
{
	struct max77620_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct max77620_onkey *onkey;
	unsigned int val;
	int irq, ret;

	onkey = devm_kzalloc(&pdev->dev, sizeof(*onkey), GFP_KERNEL);
	if (!onkey)
		return -ENOMEM;

	onkey->rmap = chip->rmap;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Drop whatever the bootloader left latched, or nothing ever fires. */
	ret = regmap_read(onkey->rmap, MAX77620_REG_ONOFFIRQ, &val);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to clear ONOFFIRQ\n");

	onkey->input = devm_input_allocate_device(&pdev->dev);
	if (!onkey->input)
		return -ENOMEM;

	onkey->input->name = "max77620-onkey";
	onkey->input->phys = "max77620-onkey/input0";
	onkey->input->id.bustype = BUS_I2C;
	input_set_capability(onkey->input, EV_KEY, KEY_POWER);

	ret = input_register_device(onkey->input);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register input device\n");

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
					max77620_onkey_irq, IRQF_ONESHOT,
					"max77620-onkey", onkey);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to request ON/OFF interrupt\n");

	ret = devm_device_init_wakeup(&pdev->dev);
	if (ret)
		return ret;

	return devm_pm_set_wake_irq(&pdev->dev, irq);
}

static const struct of_device_id max77620_onkey_of_match[] = {
	{ .compatible = "maxim,max77620-onkey", },
	{ }
};
MODULE_DEVICE_TABLE(of, max77620_onkey_of_match);

static struct platform_driver max77620_onkey_driver = {
	.driver = {
		.name = "max77620-onkey",
		.of_match_table = max77620_onkey_of_match,
	},
	.probe = max77620_onkey_probe,
};
module_platform_driver(max77620_onkey_driver);

MODULE_DESCRIPTION("MAX77620 ON/OFF key driver");
MODULE_LICENSE("GPL");
