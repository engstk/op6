
/*
 * drm_sysfs.c - Modifications to drm_sysfs_class.c to support
 *               extra sysfs attribute from DRM. Normal drm_sysfs_class
 *               does not allow adding attributes.
 *
 * Copyright (c) 2004 Jon Smirl <jonsmirl@gmail.com>
 * Copyright (c) 2003-2004 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (c) 2003-2004 IBM Corp.
 *
 * This file is released under the GPLv2
 *
 */

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/gfp.h>
#include <linux/err.h>
#include <linux/export.h>

#include <drm/drm_sysfs.h>
#include <drm/drmP.h>
#include "drm_internal.h"

#include <linux/list.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/init.h>
#include <drm/drm_mipi_dsi.h>

#define to_drm_minor(d) dev_get_drvdata(d)
#define to_drm_connector(d) dev_get_drvdata(d)

static struct device_type drm_sysfs_device_minor = {
	.name = "drm_minor"
};

struct class *drm_class;

static char *drm_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "dri/%s", dev_name(dev));
}

static CLASS_ATTR_STRING(version, S_IRUGO, "drm 1.1.0 20060810");

/**
 * drm_sysfs_init - initialize sysfs helpers
 *
 * This is used to create the DRM class, which is the implicit parent of any
 * other top-level DRM sysfs objects.
 *
 * You must call drm_sysfs_destroy() to release the allocated resources.
 *
 * Return: 0 on success, negative error code on failure.
 */
int drm_sysfs_init(void)
{
	int err;

	drm_class = class_create(THIS_MODULE, "drm");
	if (IS_ERR(drm_class))
		return PTR_ERR(drm_class);

	err = class_create_file(drm_class, &class_attr_version.attr);
	if (err) {
		class_destroy(drm_class);
		drm_class = NULL;
		return err;
	}

	drm_class->devnode = drm_devnode;
	return 0;
}

/**
 * drm_sysfs_destroy - destroys DRM class
 *
 * Destroy the DRM device class.
 */
void drm_sysfs_destroy(void)
{
	if (IS_ERR_OR_NULL(drm_class))
		return;
	class_remove_file(drm_class, &class_attr_version.attr);
	class_destroy(drm_class);
	drm_class = NULL;
}

/*
 * Connector properties
 */
static ssize_t status_store(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(device);
	struct drm_device *dev = connector->dev;
	enum drm_connector_force old_force;
	int ret;

	ret = mutex_lock_interruptible(&dev->mode_config.mutex);
	if (ret)
		return ret;

	old_force = connector->force;

	if (sysfs_streq(buf, "detect"))
		connector->force = 0;
	else if (sysfs_streq(buf, "on"))
		connector->force = DRM_FORCE_ON;
	else if (sysfs_streq(buf, "on-digital"))
		connector->force = DRM_FORCE_ON_DIGITAL;
	else if (sysfs_streq(buf, "off"))
		connector->force = DRM_FORCE_OFF;
	else
		ret = -EINVAL;

	if (old_force != connector->force || !connector->force) {
		DRM_DEBUG_KMS("[CONNECTOR:%d:%s] force updated from %d to %d or reprobing\n",
			      connector->base.id,
			      connector->name,
			      old_force, connector->force);

		connector->funcs->fill_modes(connector,
					     dev->mode_config.max_width,
					     dev->mode_config.max_height);
	}

	mutex_unlock(&dev->mode_config.mutex);

	return ret ? ret : count;
}

static ssize_t status_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	struct drm_connector *connector = to_drm_connector(device);
	enum drm_connector_status status;

	status = READ_ONCE(connector->status);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			drm_get_connector_status_name(status));
}

static ssize_t dpms_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	struct drm_connector *connector = to_drm_connector(device);
	int dpms;

	dpms = READ_ONCE(connector->dpms);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			drm_get_dpms_name(dpms));
}

static ssize_t enabled_show(struct device *device,
			    struct device_attribute *attr,
			   char *buf)
{
	struct drm_connector *connector = to_drm_connector(device);
	bool enabled;

	enabled = READ_ONCE(connector->encoder);

	return snprintf(buf, PAGE_SIZE, enabled ? "enabled\n" : "disabled\n");
}

static ssize_t edid_show(struct file *filp, struct kobject *kobj,
			 struct bin_attribute *attr, char *buf, loff_t off,
			 size_t count)
{
	struct device *connector_dev = kobj_to_dev(kobj);
	struct drm_connector *connector = to_drm_connector(connector_dev);
	unsigned char *edid;
	size_t size;
	ssize_t ret = 0;

	mutex_lock(&connector->dev->mode_config.mutex);
	if (!connector->edid_blob_ptr)
		goto unlock;

	edid = connector->edid_blob_ptr->data;
	size = connector->edid_blob_ptr->length;
	if (!edid)
		goto unlock;

	if (off >= size)
		goto unlock;

	if (off + count > size)
		count = size - off;
	memcpy(buf, edid + off, count);

	ret = count;
unlock:
	mutex_unlock(&connector->dev->mode_config.mutex);

	return ret;
}

static ssize_t modes_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	struct drm_connector *connector = to_drm_connector(device);
	struct drm_display_mode *mode;
	int written = 0;

	mutex_lock(&connector->dev->mode_config.mutex);
	list_for_each_entry(mode, &connector->modes, head) {
		written += snprintf(buf + written, PAGE_SIZE - written, "%s\n",
				    mode->name);
	}
	mutex_unlock(&connector->dev->mode_config.mutex);

	return written;
}

static ssize_t dsi_panel_command_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_get_dsi_panel_command(connector, buf);

	return ret;
}

static ssize_t dsi_panel_command_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_update_dsi_panel_command(connector, buf, count);
	if (ret)
		pr_err("Failed to update dsi panel command, ret=%d\n", ret);

	return count;
}
static ssize_t dsi_seed_command_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_get_dsi_seed_command(connector, buf);

	return ret;
}

static ssize_t dsi_seed_command_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = dsi_display_update_dsi_seed_command(connector, buf, count);
	if (ret)
		pr_err("Failed to update dsi seed command, ret=%d\n", ret);

	return count;
}

static ssize_t acl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int acl_mode = 0;

	acl_mode = dsi_display_get_acl_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "acl mode = %d\n"
					"0--acl mode(off)\n"
					"1--acl mode(5)\n"
					"2--acl mode(10)\n"
					"3--acl mode(15)\n",
					acl_mode);
	return ret;
}

static ssize_t acl_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int acl_mode = 0;

	ret = kstrtoint(buf, 10, &acl_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_acl_mode(connector, acl_mode);
	if (ret)
		pr_err("set acl mode(%d) fail\n", acl_mode);

	return count;
}

static ssize_t hbm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int hbm_mode = 0;

	hbm_mode = dsi_display_get_hbm_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "hbm mode = %d\n"
					"0--hbm mode(off)\n"
					"1--hbm mode(464)\n"
					"2--hbm mode(498)\n"
					"3--hbm mode(532)\n"
					"4--hbm mode(566)\n"
					"5--hbm mode(600)\n",
					hbm_mode);
	return ret;
}

static ssize_t hbm_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int hbm_mode = 0;

	ret = kstrtoint(buf, 10, &hbm_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_hbm_mode(connector, hbm_mode);
	if (ret)
		pr_err("set hbm mode(%d) fail\n", hbm_mode);

	return count;
}
static ssize_t op_friginer_print_hbm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int op_hbm_mode = 0;

	op_hbm_mode = dsi_display_get_fp_hbm_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "OP_FP mode = %d\n"
			"0--finger-hbm mode(off)\n"
			"1--finger-hbm mode(600)\n",
			op_hbm_mode);
	return ret;
}

static ssize_t op_friginer_print_hbm_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int op_hbm_mode = 0;

	ret = kstrtoint(buf, 10, &op_hbm_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_fp_hbm_mode(connector, op_hbm_mode);
	if (ret)
		pr_err("set hbm mode(%d) fail\n", op_hbm_mode);

	return count;
}

static ssize_t aod_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_mode = 0;

	aod_mode = dsi_display_get_aod_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", aod_mode);
	return ret;
}

static ssize_t aod_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_mode = 0;

	ret = kstrtoint(buf, 10, &aod_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_aod_mode(connector, aod_mode);
	if (ret)
		pr_err("set AOD mode(%d) fail\n", aod_mode);

	return count;
}

static ssize_t aod_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_mode = 0;

	aod_mode = dsi_display_get_aod_mode_test(connector);

	ret = scnprintf(buf, PAGE_SIZE, "AOD mode = %d\n"
			"0--AOD off\n"
			"1--AOD(10nit-alpm)\n"
			"2--AOD(50nit-alpm)\n"
			"3--AOD(10nit-hlpm)\n"
			"4--AOD(50nit-hlpm)\n",
			aod_mode);
	return ret;
}

static ssize_t aod_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_mode = 0;

	ret = kstrtoint(buf, 10, &aod_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_aod_mode_test(connector, aod_mode);
	if (ret)
		pr_err("set AOD mode(%d) fail\n", aod_mode);

	return count;
}
static ssize_t aod_disable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_disable = 0;

	aod_disable = dsi_display_get_aod_disable(connector);

	ret = scnprintf(buf, PAGE_SIZE, "AOD disable = %d\n"
			"0--AOD enable\n"
			"1--AOD disable\n",
			aod_disable);
	return ret;
}

static ssize_t aod_disable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int aod_disable = 0;

	ret = kstrtoint(buf, 10, &aod_disable);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_aod_disable(connector, aod_disable);
	if (ret)
		pr_err("set AOD disable(%d) fail\n", aod_disable);

	return count;
}


static ssize_t panel_serial_number_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int panel_year = 0;
	int panel_mon = 0;
	int panel_day = 0;
	int panel_hour = 0;
	int panel_min = 0;
	int ret = 0;

	dsi_display_get_serial_number(connector);

	panel_year = dsi_display_get_serial_number_year(connector);
	panel_mon = dsi_display_get_serial_number_mon(connector);
	panel_day = dsi_display_get_serial_number_day(connector);
	panel_hour = dsi_display_get_serial_number_hour(connector);
	panel_min = dsi_display_get_serial_number_min(connector);

	ret = scnprintf(buf, PAGE_SIZE, "%04d/%02d/%02d %02d:%02d\n",
			panel_year, panel_mon, panel_day,
			panel_hour, panel_min);

	pr_info("panel year = %d, mon = %d, day = %d, hour = %d, min = %d\n",
			panel_year, panel_mon, panel_day,
			panel_hour, panel_min);

	return ret;
}

int current_freq;
static ssize_t dynamic_dsitiming_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = scnprintf(buf, PAGE_SIZE, "current_freq = %d\n",
			current_freq);
	return ret;
}

static ssize_t dynamic_dsitiming_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int freq_value = 0;

	ret = kstrtoint(buf, 10, &freq_value);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	current_freq = freq_value;
	pr_info("freq setting=%d\n", current_freq);

	if (ret)
		pr_err("set dsi freq (%d) fail\n", current_freq);

	return count;
}

static ssize_t panel_mismatch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int wrong_panel = 0;

	dsi_display_panel_mismatch_check(connector);

	wrong_panel = dsi_display_panel_mismatch(connector);
	ret = scnprintf(buf, PAGE_SIZE, "panel mismatch = %d\n"
			"0--(panel match)\n"
			"1--(panel mismatch)\n",
			wrong_panel);
	return ret;
}

int oneplus_panel_alpha;
int oneplus_force_screenfp;
int op_dimlayer_bl_enable;
int op_dp_enable;
int op_dither_enable;
extern int oneplus_get_panel_brightness_to_alpha(void);

static ssize_t oneplus_display_get_dim_alpha(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			oneplus_get_panel_brightness_to_alpha());
}

static ssize_t oneplus_display_set_dim_alpha(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;

	//sscanf(buf, "%x", &oneplus_panel_alpha);
	ret = kstrtoint(buf, 10, &oneplus_force_screenfp);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	return count;
}

static ssize_t oneplus_display_get_forcescreenfp(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	oneplus_force_screenfp = dsi_display_get_fp_hbm_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "OP_FP mode = %d\n"
			"0--finger-hbm mode(off)\n"
			"1--finger-hbm mode(600)\n",
			oneplus_force_screenfp);
	return snprintf(buf, PAGE_SIZE, "%d\n", oneplus_force_screenfp);

}

static ssize_t oneplus_display_set_forcescreenfp(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	//sscanf(buf, "%x", &oneplus_force_screenfp);
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;

	ret = kstrtoint(buf, 10, &oneplus_force_screenfp);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_fp_hbm_mode(connector, oneplus_force_screenfp);
	if (ret)
		pr_err("set hbm mode(%d) fail\n", oneplus_force_screenfp);
	return count;
}

static ssize_t op_display_get_dimlayer_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", op_dimlayer_bl_enable);
}

static ssize_t op_display_set_dimlayer_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err = sscanf(buf, "%d", &op_dimlayer_bl_enable);

	if (err < 0)
	pr_err("op_display_set_dimlayer_enable sscanf failed");
	return count;
}

static ssize_t op_display_get_dither_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", op_dither_enable);
}

static ssize_t op_display_set_dither_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err = sscanf(buf, "%d", &op_dither_enable);

	if (err < 0)
	pr_err("op_display_set_dither_enable sscanf failed");
	return count;
}

static ssize_t op_display_get_dp_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", op_dp_enable);
}

static ssize_t op_display_set_dp_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err = sscanf(buf, "%d", &op_dp_enable);

	if (err < 0)
	pr_err("op_display_set_dp_enable sscanf failed");
	return count;
}

extern  ssize_t oneplus_display_notify_fp_press(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

extern  ssize_t oneplus_display_notify_dim(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

extern  ssize_t oneplus_display_notify_aod_hid(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);


static ssize_t native_display_p3_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_p3_mode = 0;

	native_display_p3_mode = dsi_display_get_native_display_p3_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display p3 mode = %d\n"
											"0--native display p3 mode Off\n"
											"1--native display p3 mode On\n",
											native_display_p3_mode);
	return ret;
}

static ssize_t native_display_p3_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_p3_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_p3_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_native_display_p3_mode(connector, native_display_p3_mode);
	if (ret) {
		pr_err("set native_display_p3  mode(%d) fail\n", native_display_p3_mode);
	}
	return count;
}
static ssize_t native_display_wide_color_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_wide_color_mode = 0;

	native_display_wide_color_mode = dsi_display_get_native_display_wide_color_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display wide color mode = %d\n"
											"0--native display wide color mode Off\n"
											"1--native display wide color mode On\n",
											native_display_wide_color_mode);
	return ret;
}

static ssize_t native_display_customer_p3_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_customer_p3_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_customer_p3_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_customer_p3_mode(connector, native_display_customer_p3_mode);
	if (ret) {
		pr_err("set customer p3  mode(%d) fail\n", native_display_customer_p3_mode);
	}
	return count;
}

static ssize_t native_display_customer_p3_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_customer_p3_mode = 0;

	native_display_customer_p3_mode = dsi_display_get_customer_p3_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display customer p3 mode = %d\n"
											"0--native display customer p3 mode Off\n"
											"1--native display customer p3 mode On\n",
											native_display_customer_p3_mode);
	return ret;
}
static ssize_t native_display_customer_srgb_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_customer_srgb_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_customer_srgb_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_customer_srgb_mode(connector, native_display_customer_srgb_mode);
	if (ret) {
		pr_err("set customer srgb  mode(%d) fail\n", native_display_customer_srgb_mode);
	}
	return count;
}

static ssize_t native_display_customer_srgb_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_customer_srgb_mode = 0;

	native_display_customer_srgb_mode = dsi_display_get_customer_srgb_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display customer srgb mode = %d\n"
											"0--native display customer srgb mode Off\n"
											"1--native display customer srgb mode On\n",
											native_display_customer_srgb_mode);
	return ret;
}


static ssize_t native_display_wide_color_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_wide_color_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_wide_color_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_native_display_wide_color_mode(connector, native_display_wide_color_mode);
	if (ret) {
		pr_err("set native_display_p3  mode(%d) fail\n", native_display_wide_color_mode);
	}
	return count;
}

static ssize_t native_display_srgb_color_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_srgb_color_mode = 0;

	native_display_srgb_color_mode = dsi_display_get_native_display_srgb_color_mode(connector);

	ret = scnprintf(buf, PAGE_SIZE, "native display srgb color mode = %d\n"
											"0--native display srgb color mode Off\n"
											"1--native display srgb color mode On\n",
											native_display_srgb_color_mode);
	return ret;
}

static ssize_t native_display_srgb_color_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct drm_connector *connector = to_drm_connector(dev);
	int ret = 0;
	int native_display_srgb_color_mode = 0;

	ret = kstrtoint(buf, 10, &native_display_srgb_color_mode);
	if (ret) {
		pr_err("kstrtoint failed. ret=%d\n", ret);
		return ret;
	}

	ret = dsi_display_set_native_display_srgb_color_mode(connector, native_display_srgb_color_mode);
	if (ret) {
		pr_err("set native_display_srgb  mode(%d) fail\n", native_display_srgb_color_mode);
	}
	return count;
}
/******************************************************************/
static DEVICE_ATTR_RW(status);
static DEVICE_ATTR_RO(enabled);
static DEVICE_ATTR_RO(dpms);
static DEVICE_ATTR_RO(modes);
static DEVICE_ATTR_RW(acl);
static DEVICE_ATTR_RW(hbm);
static DEVICE_ATTR_RW(op_friginer_print_hbm);
static DEVICE_ATTR_RW(aod);
static DEVICE_ATTR_RW(aod_disable);
static DEVICE_ATTR_RW(native_display_p3_mode);
static DEVICE_ATTR_RW(native_display_wide_color_mode);
static DEVICE_ATTR_RW(native_display_srgb_color_mode);
static DEVICE_ATTR_RW(native_display_customer_p3_mode);
static DEVICE_ATTR_RW(native_display_customer_srgb_mode);
static DEVICE_ATTR_RO(panel_serial_number);
static DEVICE_ATTR_RW(dynamic_dsitiming);
static DEVICE_ATTR_RO(panel_mismatch);
static DEVICE_ATTR_RW(aod_test);
static DEVICE_ATTR(dim_alpha, S_IRUGO|S_IWUSR, oneplus_display_get_dim_alpha,
	oneplus_display_set_dim_alpha);
static DEVICE_ATTR(force_screenfp, S_IRUGO|S_IWUSR,
	oneplus_display_get_forcescreenfp, oneplus_display_set_forcescreenfp);
static DEVICE_ATTR(notify_fppress, S_IRUGO|S_IWUSR, NULL,
	oneplus_display_notify_fp_press);
static DEVICE_ATTR(notify_dim, S_IRUGO|S_IWUSR, NULL,
	oneplus_display_notify_dim);
static DEVICE_ATTR(notify_aod, S_IRUGO|S_IWUSR, NULL,
	oneplus_display_notify_aod_hid);

static DEVICE_ATTR(dp_en, S_IRUGO|S_IWUSR,
       op_display_get_dp_enable, op_display_set_dp_enable);
static DEVICE_ATTR(dither_en, S_IRUGO|S_IWUSR,
       op_display_get_dither_enable, op_display_set_dither_enable);
static DEVICE_ATTR(dimlayer_bl_en, S_IRUGO|S_IWUSR,
       op_display_get_dimlayer_enable, op_display_set_dimlayer_enable);

static DEVICE_ATTR_RW(dsi_panel_command);
static DEVICE_ATTR_RW(dsi_seed_command);

static struct attribute *connector_dev_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_enabled.attr,
	&dev_attr_dpms.attr,
	&dev_attr_modes.attr,
	&dev_attr_acl.attr,
	&dev_attr_hbm.attr,
	&dev_attr_op_friginer_print_hbm.attr,
	&dev_attr_aod.attr,
	&dev_attr_aod_disable.attr,
	&dev_attr_native_display_p3_mode.attr,
	&dev_attr_native_display_wide_color_mode.attr,
	&dev_attr_native_display_srgb_color_mode.attr,
	&dev_attr_native_display_customer_p3_mode.attr,
	&dev_attr_native_display_customer_srgb_mode.attr,
	&dev_attr_panel_serial_number.attr,
	&dev_attr_dynamic_dsitiming.attr,
	&dev_attr_panel_mismatch.attr,
	&dev_attr_aod_test.attr,
	&dev_attr_force_screenfp.attr,
	&dev_attr_dim_alpha.attr,
	&dev_attr_notify_fppress.attr,
	&dev_attr_notify_dim.attr,
	&dev_attr_notify_aod.attr,
	&dev_attr_dsi_panel_command.attr,
	&dev_attr_dsi_seed_command.attr,
	&dev_attr_dimlayer_bl_en.attr,
	&dev_attr_dp_en.attr,
	&dev_attr_dither_en.attr,
	NULL
};

static struct bin_attribute edid_attr = {
	.attr.name = "edid",
	.attr.mode = 0444,
	.size = 0,
	.read = edid_show,
};

static struct bin_attribute *connector_bin_attrs[] = {
	&edid_attr,
	NULL
};

static const struct attribute_group connector_dev_group = {
	.attrs = connector_dev_attrs,
	.bin_attrs = connector_bin_attrs,
};

static const struct attribute_group *connector_dev_groups[] = {
	&connector_dev_group,
	NULL
};

/**
 * drm_sysfs_connector_add - add a connector to sysfs
 * @connector: connector to add
 *
 * Create a connector device in sysfs, along with its associated connector
 * properties (so far, connection status, dpms, mode list & edid) and
 * generate a hotplug event so userspace knows there's a new connector
 * available.
 */
int drm_sysfs_connector_add(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;

	if (connector->kdev)
		return 0;

	connector->kdev =
		device_create_with_groups(drm_class, dev->primary->kdev, 0,
					  connector, connector_dev_groups,
					  "card%d-%s", dev->primary->index,
					  connector->name);
	DRM_DEBUG("adding \"%s\" to sysfs\n",
		  connector->name);

	if (IS_ERR(connector->kdev)) {
		DRM_ERROR("failed to register connector device: %ld\n", PTR_ERR(connector->kdev));
		return PTR_ERR(connector->kdev);
	}

	/* Let userspace know we have a new connector */
	drm_sysfs_hotplug_event(dev);

	return 0;
}

/**
 * drm_sysfs_connector_remove - remove an connector device from sysfs
 * @connector: connector to remove
 *
 * Remove @connector and its associated attributes from sysfs.  Note that
 * the device model core will take care of sending the "remove" uevent
 * at this time, so we don't need to do it.
 *
 * Note:
 * This routine should only be called if the connector was previously
 * successfully registered.  If @connector hasn't been registered yet,
 * you'll likely see a panic somewhere deep in sysfs code when called.
 */
void drm_sysfs_connector_remove(struct drm_connector *connector)
{
	if (!connector->kdev)
		return;
	DRM_DEBUG("removing \"%s\" from sysfs\n",
		  connector->name);

	device_unregister(connector->kdev);
	connector->kdev = NULL;
}

/**
 * drm_sysfs_hotplug_event - generate a DRM uevent
 * @dev: DRM device
 *
 * Send a uevent for the DRM device specified by @dev.  Currently we only
 * set HOTPLUG=1 in the uevent environment, but this could be expanded to
 * deal with other types of events.
 */
void drm_sysfs_hotplug_event(struct drm_device *dev)
{
	char *event_string = "HOTPLUG=1";
	char *envp[] = { event_string, NULL };

	DRM_DEBUG("generating hotplug event\n");

	kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, envp);
}
EXPORT_SYMBOL(drm_sysfs_hotplug_event);

static void drm_sysfs_release(struct device *dev)
{
	kfree(dev);
}

/**
 * drm_sysfs_minor_alloc() - Allocate sysfs device for given minor
 * @minor: minor to allocate sysfs device for
 *
 * This allocates a new sysfs device for @minor and returns it. The device is
 * not registered nor linked. The caller has to use device_add() and
 * device_del() to register and unregister it.
 *
 * Note that dev_get_drvdata() on the new device will return the minor.
 * However, the device does not hold a ref-count to the minor nor to the
 * underlying drm_device. This is unproblematic as long as you access the
 * private data only in sysfs callbacks. device_del() disables those
 * synchronously, so they cannot be called after you cleanup a minor.
 */
struct device *drm_sysfs_minor_alloc(struct drm_minor *minor)
{
	const char *minor_str;
	struct device *kdev;
	int r;

	if (minor->type == DRM_MINOR_CONTROL)
		minor_str = "controlD%d";
	else if (minor->type == DRM_MINOR_RENDER)
		minor_str = "renderD%d";
	else
		minor_str = "card%d";

	kdev = kzalloc(sizeof(*kdev), GFP_KERNEL);
	if (!kdev)
		return ERR_PTR(-ENOMEM);

	device_initialize(kdev);
	kdev->devt = MKDEV(DRM_MAJOR, minor->index);
	kdev->class = drm_class;
	kdev->type = &drm_sysfs_device_minor;
	kdev->parent = minor->dev->dev;
	kdev->release = drm_sysfs_release;
	dev_set_drvdata(kdev, minor);

	r = dev_set_name(kdev, minor_str, minor->index);
	if (r < 0)
		goto err_free;

	return kdev;

err_free:
	put_device(kdev);
	return ERR_PTR(r);
}

/**
 * drm_class_device_register - Register a struct device in the drm class.
 *
 * @dev: pointer to struct device to register.
 *
 * @dev should have all relevant members pre-filled with the exception
 * of the class member. In particular, the device_type member must
 * be set.
 */

int drm_class_device_register(struct device *dev)
{
	if (!drm_class || IS_ERR(drm_class))
		return -ENOENT;

	dev->class = drm_class;
	return device_register(dev);
}
EXPORT_SYMBOL_GPL(drm_class_device_register);

void drm_class_device_unregister(struct device *dev)
{
	return device_unregister(dev);
}
EXPORT_SYMBOL_GPL(drm_class_device_unregister);
