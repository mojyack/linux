// SPDX-License-Identifier: GPL-2.0-only

#include <linux/devm-helpers.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77620.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define MAX77620_ONOFFIRQ_EN0	(MAX77620_ONOFFIRQ_EN0_F | MAX77620_ONOFFIRQ_EN0_R)
#define MAX77620_ONKEY_RETRIES	10
#define MAX77620_ONKEY_RETRY_MS	100

struct max77620_onkey {
	struct device *dev;
	struct input_dev *input;
	struct regmap *map;
	struct mutex lock; /* serializes probe, the handler and the retry */
	struct delayed_work retry;
	unsigned int code;
	unsigned int edges;
	unsigned int retries;
	bool level;
	bool ready;
};

static void max77620_onkey_report(struct max77620_onkey *onkey, bool pressed)
{
	input_report_key(onkey->input, onkey->code, pressed);
	input_sync(onkey->input);
}

static int max77620_onkey_update(struct max77620_onkey *onkey)
{
	unsigned int val, away, back;
	bool level;
	int ret;

	/* ONOFFIRQ is clear-on-read: a bit left set masks its own source. */
	ret = regmap_read(onkey->map, MAX77620_REG_ONOFFIRQ, &val);
	if (ret)
		return ret;

	if (onkey->ready)
		onkey->edges |= val & MAX77620_ONOFFIRQ_EN0;

	ret = regmap_read(onkey->map, MAX77620_REG_ONOFFSTAT, &val);
	if (ret)
		return ret;

	level = val & MAX77620_ONOFFSTAT_EN0;
	if (!onkey->ready) {
		onkey->level = level;
		return 0;
	}

	/* Edges arrive in turn, starting with the one leaving onkey->level. */
	away = onkey->level ? MAX77620_ONOFFIRQ_EN0_F : MAX77620_ONOFFIRQ_EN0_R;
	back = onkey->edges & ~away;
	if (onkey->edges & away) {
		max77620_onkey_report(onkey, !onkey->level);
		if (back) {
			/* Both edges: only the level tells whether one more followed. */
			max77620_onkey_report(onkey, onkey->level);
			if (level != onkey->level)
				max77620_onkey_report(onkey, level);
		} else {
			level = !onkey->level;
		}
		onkey->level = level;
	}
	onkey->edges = 0;

	return 0;
}

static void max77620_onkey_run(struct max77620_onkey *onkey)
{
	int ret;

	guard(mutex)(&onkey->lock);

	ret = max77620_onkey_update(onkey);
	if (!ret) {
		if (onkey->retries)
			dev_info(onkey->dev, "ON/OFF registers readable again\n");
		onkey->retries = 0;
	} else if (onkey->retries++ < MAX77620_ONKEY_RETRIES) {
		dev_warn(onkey->dev, "failed to read ON/OFF registers: %d, retrying\n", ret);
		queue_delayed_work(system_freezable_wq, &onkey->retry,
				   msecs_to_jiffies(MAX77620_ONKEY_RETRY_MS));
	} else {
		dev_err(onkey->dev, "failed to read ON/OFF registers: %d, giving up\n", ret);
		onkey->retries = 0;
	}
}

static void max77620_onkey_retry(struct work_struct *work)
{
	max77620_onkey_run(container_of(to_delayed_work(work),
					struct max77620_onkey, retry));
}

static irqreturn_t max77620_onkey_irq(int irq, void *data)
{
	max77620_onkey_run(data);

	return IRQ_HANDLED;
}

static int max77620_onkey_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *fwnode;
	struct max77620_onkey *onkey;
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

	if (onkey->code > KEY_MAX)
		return dev_err_probe(dev, -EINVAL, "invalid linux,code %u\n",
				     onkey->code);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_mutex_init(dev, &onkey->lock);
	if (ret)
		return ret;

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

	/* Canceled after the IRQ is freed and before the input device goes. */
	ret = devm_delayed_work_autocancel(dev, &onkey->retry,
					   max77620_onkey_retry);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, max77620_onkey_irq,
					IRQF_ONESHOT, "max77620-onkey", onkey);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to request ON/OFF interrupt\n");

	/* Only the EN0 edges: the other sources would run this handler too. */
	ret = regmap_write(onkey->map, MAX77620_REG_ONOFFIRQM,
			   0xff & ~MAX77620_ONOFFIRQ_EN0);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to write ONOFFIRQM\n");

	/* Edges latched before this point are discarded, by us or the handler. */
	scoped_guard(mutex, &onkey->lock) {
		ret = max77620_onkey_update(onkey);
		onkey->ready = !ret;
	}
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read ON/OFF registers\n");

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
