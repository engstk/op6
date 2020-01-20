
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/project_info.h>

#include "fingerprint_detect.h"
int fp_version;
bool screen_off = false;

static int fingerprint_detect_request_named_gpio(
		struct fingerprint_detect_data *fp_detect,
		const char *label, int *gpio)
{
	struct device *dev = fp_detect->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		*gpio = rc;
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_info(dev, "%s - gpio: %d\n", label, *gpio);
	return 0;
}

static ssize_t sensor_version_get(struct device *device,
			     struct device_attribute *attribute,
			     char *buffer)
{
	struct fingerprint_detect_data *fp_detect = dev_get_drvdata(device);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", fp_detect->sensor_version);
}

static DEVICE_ATTR(sensor_version, S_IRUSR, sensor_version_get, NULL);

static struct attribute *attributes[] = {
	&dev_attr_sensor_version.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

int fp_pinctrl_init(struct fingerprint_detect_data *fp_dev)
{
	int ret = 0;
	struct device *dev = fp_dev->dev;

	fp_dev->fp_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(fp_dev->fp_pinctrl)) {
		dev_err(dev, "Target does not use pinctrl\n");
		ret = PTR_ERR(fp_dev->fp_pinctrl);
		goto err;
	}

	fp_dev->id_en_init =
		pinctrl_lookup_state(fp_dev->fp_pinctrl, "fp_id_init");
	if (IS_ERR_OR_NULL(fp_dev->id_en_init)) {
		dev_err(dev, "Cannot get en active pinstate\n");
		ret = PTR_ERR(fp_dev->id_en_init);
		goto err;
	}

	ret = pinctrl_select_state(fp_dev->fp_pinctrl,
		fp_dev->id_en_init);
	if (ret) {
		dev_err(dev, "can not set %s pins\n", "id_en_init");
		goto err;
	}

err:
	fp_dev->fp_pinctrl = NULL;
	fp_dev->id_en_init = NULL;
	return ret;
}

int fp_pinctrl_id(struct fingerprint_detect_data *fp_dev, int en)
{
	int ret = 0;
	struct device *dev = fp_dev->dev;
	fp_dev->fp_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(fp_dev->fp_pinctrl)) {
		dev_err(dev, "Target does not use pinctrl\n");
		ret = PTR_ERR(fp_dev->fp_pinctrl);
		goto err;
	}

	if (en == 1) {
		fp_dev->id_state_up =
			pinctrl_lookup_state(fp_dev->fp_pinctrl, "fp_id_up");
		if (IS_ERR_OR_NULL(fp_dev->id_state_up)) {
			dev_err(dev, "Cannot fp_id_up pinstate\n");
			ret = PTR_ERR(fp_dev->id_state_up);
			goto err;
		}
		ret = pinctrl_select_state(fp_dev->fp_pinctrl,
			fp_dev->id_state_up);
		if (ret) {
			dev_err(dev, "can not set %s pins\n", "fp_id_up");
			goto err;
		}
	} else {
	fp_dev->id_state_down =
		pinctrl_lookup_state(fp_dev->fp_pinctrl, "fp_id_down");
		if (IS_ERR_OR_NULL(fp_dev->id_state_down)) {
			dev_err(dev, "Cannot get fp_id_down pinstate\n");
			ret = PTR_ERR(fp_dev->id_state_down);
			goto err;
		}
		ret = pinctrl_select_state(fp_dev->fp_pinctrl,
			fp_dev->id_state_down);
		if (ret) {
			dev_err(dev, "can not set %s pins\n", "fp_id_down");
			goto err;
		}
	}

err:
	fp_dev->fp_pinctrl = NULL;
	fp_dev->id_state_up = NULL;
	fp_dev->id_state_down = NULL;
	return ret;
}

static int fingerprint_detect_probe(struct platform_device *pdev)
{
	int id0 = 0, id1 = 0, id2 = 0;
	int rc = 0;
	int i;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	struct fingerprint_detect_data *fp_detect =
		devm_kzalloc(dev, sizeof(*fp_detect),
			GFP_KERNEL);
	if (!fp_detect) {
		dev_err(dev,
			"failed to allocate memory for struct fingerprint_detect_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	pr_info("%s\n", __func__);

	fp_detect->dev = dev;
	dev_set_drvdata(dev, fp_detect);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}
	if (of_property_read_bool(fp_detect->dev->of_node, "oem,enchilada"))
		fp_detect->project_version = 0x02;
	else if (of_property_read_bool(fp_detect->dev->of_node, "oem,fajta"))
		fp_detect->project_version = 0x03;
	else
		fp_detect->project_version = 0x01;

	if (fp_detect->project_version < 0x03) {
		rc = fp_pinctrl_init(fp_detect);
		if (rc)
			goto exit;
	}

	rc = fingerprint_detect_request_named_gpio(fp_detect, "fp-gpio-id0",
			&fp_detect->id0_gpio);
	if (gpio_is_valid(fp_detect->id0_gpio)) {
	dev_err(dev, "%s: gpio_is_valid(fp_detect->id0_gpio=%d)\n",
		__func__, fp_detect->id0_gpio);
	}

	if (fp_detect->project_version < 0x03) {
		rc = fingerprint_detect_request_named_gpio(fp_detect,
		"fp-gpio-id1", &fp_detect->id1_gpio);
		if (gpio_is_valid(fp_detect->id1_gpio)) {
			dev_err(dev, "%s: gpio_is_valid(fp_detect->id1_gpio=%d)\n",
				__func__, fp_detect->id1_gpio);
		}

		rc = fingerprint_detect_request_named_gpio(fp_detect,
		"fp-gpio-id2", &fp_detect->id2_gpio);
		if (gpio_is_valid(fp_detect->id2_gpio)) {
		dev_err(dev, "%s: gpio_is_valid(fp_detect->id2_gpio=%d)\n",
			__func__, fp_detect->id2_gpio);
		}
	}
	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	/**
    *           ID0(GPIO91)   ID1(GPIO92)   ID1(GPIO95)
    *   fpc1245
    *   O-film   1            1             1
    *   Primax   1            0             0
    *   truly    0            0             1
    *

    *   goodix   1            1             0
    *   Primax   0            0             0
    *   truly    0            1             1
    *fingerchip/
    *   qtech    0            1             0
    *   Goodix   1            0             1
    */

	for (i = 1; i < 4; i++) {
		usleep_range(5000, 10000);
		rc = fp_pinctrl_id(fp_detect, i%2);
		if (rc)
			goto exit;
		id0 = gpio_get_value(fp_detect->id0_gpio);
		if (fp_detect->project_version < 0x03) {
			id1 = gpio_get_value(fp_detect->id1_gpio);
			id2 = gpio_get_value(fp_detect->id2_gpio);
		}
		pr_info("%s: %d%d%d\n", __func__, id0, id1, id2);
	}

	if (fp_detect->project_version < 0x03) {
		if (id0 && id1 && id2) {
			push_component_info(FINGERPRINTS,
				"fpc1228", "FPC");
			fp_detect->sensor_version = 0x01;
		} else if (id0 && !id1 && !id2) {
			push_component_info(FINGERPRINTS,
				"fpc1245", "FPC(Primax)");
			fp_detect->sensor_version = 0x01;
		} else if (!id0 && !id1 && id2) {
			push_component_info(FINGERPRINTS, "gf5228", "goodix");
			fp_detect->sensor_version = 0x03;
		} else if (id0 && id1 && !id2) {
			if (fp_detect->project_version == 0x02) {
				push_component_info(FINGERPRINTS,
				"goodix5228", "goodix");
				fp_detect->sensor_version = 0x03;
			} else {
				push_component_info(FINGERPRINTS,
				"goodix5228", "goodix");
				fp_detect->sensor_version = 0x03;
			}
		} else if (!id0 && !id1 && !id2) {
			push_component_info(FINGERPRINTS,
				"fpc1263", "FPC(Primax)");
			fp_detect->sensor_version = 0x02;
		} else if (!id0 && id1 && id2) {
			if (fp_detect->project_version == 0x02) {
				push_component_info(FINGERPRINTS,
					"gfp5288", "Goodix");
				fp_detect->sensor_version = 0x03;
			} else {
				push_component_info(FINGERPRINTS,
					"fpc1263", "FPC(truly)");
				fp_detect->sensor_version = 0x02;
			}
		} else if (!id0 && id1 && !id2) {
			push_component_info(FINGERPRINTS,
				"fpc1263", "FPC(f/p)");
			fp_detect->sensor_version = 0x02;
		} else if (id0 && !id1 && id2) {
			push_component_info(FINGERPRINTS, "gfp5288", "Goodix");
			fp_detect->sensor_version = 0x03;
		} else {
			push_component_info(FINGERPRINTS, "goodix", "goodix");
		}
	} else {
		if (id0) {
			push_component_info(FINGERPRINTS,
				"goodix9508", "goodix");
			fp_detect->sensor_version = 0x04;
		} else {
			push_component_info(FINGERPRINTS,
				"sileadgsl7000", "silead");
			fp_detect->sensor_version = 0x05;
		}
	}
	fp_version = fp_detect->sensor_version;
	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
}


static const struct of_device_id fingerprint_detect_of_match[] = {
	{ .compatible = "oneplus,fpdetect", },
	{}
};
MODULE_DEVICE_TABLE(op, fingerprint_detect_of_match);

static struct platform_driver fingerprint_detect_driver = {
	.driver = {
		.name		= "fingerprint_detect",
		.owner		= THIS_MODULE,
		.of_match_table = fingerprint_detect_of_match,
	},
	.probe = fingerprint_detect_probe,
};
module_platform_driver(fingerprint_detect_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("yale liu");
MODULE_DESCRIPTION("Fingerprint detect device driver.");
