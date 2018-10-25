/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif
#include <linux/oneplus/boot_mode.h>

int gf_pinctrl_init(struct gf_dev* gf_dev)
{
	int ret = 0;
	struct device *dev = &gf_dev->spi->dev;

	gf_dev->gf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(gf_dev->gf_pinctrl)) {
		dev_err(dev, "Target does not use pinctrl\n");
		ret = PTR_ERR(gf_dev->gf_pinctrl);
		goto err;
	}

	gf_dev->gpio_state_enable =
		pinctrl_lookup_state(gf_dev->gf_pinctrl, "fp_en_init");
	if (IS_ERR_OR_NULL(gf_dev->gpio_state_enable)) {
		dev_err(dev, "Cannot get active pinstate\n");
		ret = PTR_ERR(gf_dev->gpio_state_enable);
		goto err;
	}

	gf_dev->gpio_state_disable =
		pinctrl_lookup_state(gf_dev->gf_pinctrl, "fp_dis_init");
	if (IS_ERR_OR_NULL(gf_dev->gpio_state_disable)) {
		dev_err(dev, "Cannot get active pinstate\n");
		ret = PTR_ERR(gf_dev->gpio_state_disable);
		goto err;
	}

	return 0;
err:
	gf_dev->gf_pinctrl = NULL;
	gf_dev->gpio_state_enable = NULL;
	gf_dev->gpio_state_disable = NULL;
	return ret;
}
int gf_parse_dts(struct gf_dev* gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;
	u32 voltage_supply[2];
	u32 current_supply;

	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("failed to request reset gpio, rc = %d\n", rc);
		goto err_reset;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);

	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("falied to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);
	rc = devm_gpio_request(dev, gf_dev->enable_gpio, "goodix_en");
	if (rc) {
		pr_err("failed to request enable gpio, rc = %d\n", rc);
		goto err_irq;
	}



/***********power regulator_get****************/
	gf_dev->vdd_3v2 = regulator_get(dev, "vdd-3v2");
	if (IS_ERR(gf_dev->vdd_3v2)) {
		rc = PTR_ERR(gf_dev->vdd_3v2);
		pr_err("Regulator get failed vdd rc=%d\n", rc);
	}


	rc = of_property_read_u32_array(np, "vdd-voltage", voltage_supply, 2);
	if (rc < 0)
		pr_err("%s: Failed to get regulator vdd voltage\n", __func__);

	gf_dev->regulator_vdd_vmin = voltage_supply[0];
	gf_dev->regulator_vdd_vmax = voltage_supply[1];

	rc = regulator_set_voltage(gf_dev->vdd_3v2,
		gf_dev->regulator_vdd_vmin, gf_dev->regulator_vdd_vmax);
	if (rc < 0)
		pr_err("%s:00Failed to set regulator voltage vdd\n", __func__);


	rc = of_property_read_u32(np, "vdd-current",
		&current_supply);
	if (rc < 0)
		pr_err("%s: Failed to get regulator vdd current\n", __func__);

	gf_dev->regulator_vdd_current = current_supply;

	rc = regulator_set_load(gf_dev->vdd_3v2,
		gf_dev->regulator_vdd_current);
	if (rc < 0)
		pr_err("%s: Failed to set regulator current vdd\n", __func__);


	rc = regulator_enable(gf_dev->vdd_3v2);
	if (rc)
		pr_err("Regulator vdd enable failed rc=%d\n", rc);

	if (get_boot_mode() ==  MSM_BOOT_MODE__FACTORY)
	{
		rc = regulator_disable(gf_dev->vdd_3v2);
		if (rc)
			pr_err("Regulator vdd disable failed rc=%d\n", rc);
	}

err_irq:
	devm_gpio_free(dev, gf_dev->reset_gpio);
err_reset:
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n",__func__);
	if (gpio_is_valid(gf_dev->irq_gpio))
	{
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio))
	{
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev* gf_dev)
{
	int rc = 0;

	pr_info("---- power on ok ----\n");

	return rc;
}

int gf_power_off(struct gf_dev* gf_dev)
{
    int rc = 0;

    pr_info("---- power off ----\n");

	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if(gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if(gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

