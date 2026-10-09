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
#include <linux/property.h>
#include <linux/regmap.h>

#define MAX77620_ONOFFIRQ_EN0	(MAX77620_ONOFFIRQ_EN0_F | MAX77620_ONOFFIRQ_EN0_R)

struct max77620_onkey {
	struct device *dev;
	struct input_dev *input;
	struct regmap *map;
	unsigned int code;
};

static irqreturn_t max77620_onkey_irq(int irq, void *data)
{
	struct max77620_onkey *onkey = data;
	unsigned int edges, stat;
	bool pressed;
	int ret;

	/* ONOFFIRQ is clear-on-read: a bit left set masks its own source. */
	ret = regmap_read(onkey->map, MAX77620_REG_ONOFFIRQ, &edges);
	if (ret < 0) {
		dev_err(onkey->dev, "failed to read ONOFFIRQ: %d\n", ret);
		return IRQ_NONE;
	}

	ret = regmap_read(onkey->map, MAX77620_REG_ONOFFSTAT, &stat);
	if (ret < 0) {
		dev_err(onkey->dev, "failed to read ONOFFSTAT: %d\n", ret);
		return IRQ_NONE;
	}

	/* A tap that completed already leaves both edges latched at level 0. */
	pressed = stat & MAX77620_ONOFFSTAT_EN0;
	if (!pressed && (edges & MAX77620_ONOFFIRQ_EN0) == MAX77620_ONOFFIRQ_EN0) {
		input_report_key(onkey->input, onkey->code, 1);
		input_sync(onkey->input);
	}

	input_report_key(onkey->input, onkey->code, pressed);
	input_sync(onkey->input);

	return IRQ_HANDLED;
}

static int max77620_onkey_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *fwnode;
	struct max77620_onkey *onkey;
	unsigned int val;
	bool wakeup;
	int irq, ret;

	onkey = devm_kzalloc(dev, sizeof(*onkey), GFP_KERNEL);
	if (!onkey)
		return -ENOMEM;

	onkey->dev = dev;
	onkey->map = dev_get_regmap(dev->parent, NULL);
	if (!onkey->map)
		return -ENODEV;

	fwnode = device_get_named_child_node(dev->parent, "onkey");
	if (fwnode_property_read_u32(fwnode, "linux,code", &onkey->code))
		onkey->code = KEY_POWER;
	wakeup = fwnode_property_read_bool(fwnode, "wakeup-source");
	fwnode_handle_put(fwnode);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	onkey->input = devm_input_allocate_device(dev);
	if (!onkey->input)
		return -ENOMEM;

	onkey->input->name = "max77620-onkey";
	onkey->input->phys = "max77620-onkey/input0";
	onkey->input->id.bustype = BUS_I2C;
	input_set_capability(onkey->input, EV_KEY, onkey->code);

	ret = input_register_device(onkey->input);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	ret = devm_request_threaded_irq(dev, irq, NULL, max77620_onkey_irq,
					IRQF_ONESHOT, "max77620-onkey", onkey);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to request ON/OFF interrupt\n");

	/* Only the EN0 edges: the other sources would run this handler too. */
	ret = regmap_write(onkey->map, MAX77620_REG_ONOFFIRQM,
			   0xff & ~MAX77620_ONOFFIRQ_EN0);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to write ONOFFIRQM\n");

	/* After the IRQ is requested, so no edge can latch unseen in between. */
	ret = regmap_read(onkey->map, MAX77620_REG_ONOFFIRQ, &val);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to clear ONOFFIRQ\n");

	if (!wakeup)
		return 0;

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	return devm_pm_set_wake_irq(dev, irq);
}

static struct platform_driver max77620_onkey_driver = {
	.driver = {
		.name = "max77620-onkey",
	},
	.probe = max77620_onkey_probe,
};
module_platform_driver(max77620_onkey_driver);

MODULE_DESCRIPTION("MAX77620 ON/OFF key driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:max77620-onkey");
