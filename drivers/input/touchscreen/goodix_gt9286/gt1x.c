/* drivers/input/touchscreen/gt1x.c
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 1.6
 */

#include "gt1x_generic.h"
#ifdef CONFIG_GTP_TYPE_B_PROTOCOL
#include <linux/input/mt.h>
#endif

static struct input_dev *input_dev;
static spinlock_t irq_lock;
static int irq_disabled;
#ifndef CONFIG_GTP_INT_SEL_SYNC
#include <linux/pinctrl/consumer.h>
static struct pinctrl *default_pctrl;
#endif
#ifdef CONFIG_OF
static struct regulator *vdd_ana;
int gt1x_rst_gpio;
int gt1x_int_gpio;
int gt1x_1v8_gpio;
#endif
int tp_detect_flag = 0;

#ifdef CONFIG_SUPPORT_EARSENSE
int es_enabled = 0;
int ear_flag;
int es_touch_count;
#endif

static int gt1x_register_powermanger(void);
static int gt1x_unregister_powermanger(void);

int GTP_DEBUG_ON=0;
int GTP_DEBUG_ARRAY_ON=0;
int GTP_DEBUG_FUNC_ON	=0;

static ssize_t store_tp(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int tmp = 0;
	int ret;

	ret = kstrtoint(buf, 10, &tmp);
	if (ret >= 0) {
		GTP_DEBUG_ON = tmp;
		GTP_DEBUG_ARRAY_ON = tmp;
		GTP_DEBUG_FUNC_ON = tmp;
	} else {
	GTP_ERROR("invalid content: '%s', length = %zd\n",
	buf, size);
	}

	return size;
}
static DEVICE_ATTR(tp_debug_log, 0664, NULL, store_tp);

/**
 * gt1x_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct i2c_msg msg = {
		.flags = 0,
		.addr = gt1x_i2c_client->addr,
	};
	return _do_i2c_write(&msg, addr, buffer, len);
}

/**
 * gt1x_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct i2c_msg msgs[2] = {
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH},
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = I2C_M_RD}
	};
	return _do_i2c_read(msgs, addr, buffer, len);
}

/**
 * gt1x_irq_enable - enable irq function.
 *
 */
void gt1x_irq_enable(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_disabled) {
		irq_disabled = 0;
		spin_unlock_irqrestore(&irq_lock, irqflags);
		enable_irq(gt1x_i2c_client->irq);
	} else {
		spin_unlock_irqrestore(&irq_lock, irqflags);
	}
}

/**
 * gt1x_irq_enable - disable irq function.
 *  disable irq and wait bottom half
 *  thread(gt1x_ts_work_thread)
 */
void gt1x_irq_disable(void)
{
	unsigned long irqflags;

	/* because there is an irq enable action in
	 * the bottom half thread, we need to wait until
	 * bottom half thread finished.
	 */
	synchronize_irq(gt1x_i2c_client->irq);
	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_disabled) {
		irq_disabled = 1;
		spin_unlock_irqrestore(&irq_lock, irqflags);
		disable_irq(gt1x_i2c_client->irq);
	} else {
		spin_unlock_irqrestore(&irq_lock, irqflags);
	}
}

#ifndef CONFIG_OF
int gt1x_power_switch(s32 state)
{
    return 0;
}
#endif

int gt1x_debug_proc(u8 * buf, int count)
{
	return -1;
}

#ifdef CONFIG_GTP_CHARGER_SWITCH
u32 gt1x_get_charger_status(void)
{
	/*
	 * Need to get charger status of
	 * your platform.
	 */
	 return 0;
}
#endif

/**
 * gt1x_ts_irq_handler - External interrupt service routine
 *		for interrupt mode.
 * @irq:  interrupt number.
 * @dev_id: private data pointer.
 * Return: Handle Result.
 *  		IRQ_WAKE_THREAD: top half work finished,
 *  		wake up bottom half thread to continue the rest work.
 */
static irqreturn_t gt1x_ts_irq_handler(int irq, void *dev_id)
{
	unsigned long irqflags;

	/* irq top half, use nosync irq api to
	 * disable irq line, if irq is enabled,
	 * then wake up bottom half thread */
	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_disabled) {
		irq_disabled = 1;
		spin_unlock_irqrestore(&irq_lock, irqflags);
		disable_irq_nosync(gt1x_i2c_client->irq);
		return IRQ_WAKE_THREAD;
	} else {
		spin_unlock_irqrestore(&irq_lock, irqflags);
		return IRQ_HANDLED;
	}
}

/**
 * gt1x_touch_down - Report touch point event .
 * @id: trackId
 * @x:  input x coordinate
 * @y:  input y coordinate
 * @w:  input pressure
 * Return: none.
 */
void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#ifdef CONFIG_GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

	input_report_key(input_dev, BTN_TOUCH, 1);
#ifdef CONFIG_GTP_TYPE_B_PROTOCOL
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
#else
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
#endif

	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(input_dev, ABS_MT_PRESSURE, size);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);

#ifndef CONFIG_GTP_TYPE_B_PROTOCOL
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_touch_up -  Report touch release event.
 * @id: trackId
 * Return: none.
 */
void gt1x_touch_up(s32 id)
{
#ifdef CONFIG_GTP_TYPE_B_PROTOCOL
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
#else
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_ts_work_thread - Goodix touchscreen work function.
 * @iwork: work struct of gt1x_workqueue.
 * Return: none.
 */
static irqreturn_t gt1x_ts_work_thread(int irq, void *data)
{
	u8 point_data[11] = { 0 };
	u8 end_cmd = 0;
	u8 finger = 0;
	s32 ret = 0;

#ifdef CONFIG_SUPPORT_EARSENSE
    if (es_enabled == 1) {
        /*buffer[0] = 0x2;
	    ret = gt1x_i2c_write(0x804D, buffer, 1);
	    if (ret < 0) {
		    GTP_ERROR("I2C write end_cmd  error!");
		    return ret;
	    }*/
        while (ear_flag);
    }
 #endif

 	if (update_info.status) {
        	GTP_DEBUG("Ignore interrupts during fw update.");
        	return IRQ_HANDLED;
	}

#ifdef CONFIG_GTP_GESTURE_WAKEUP
	ret = gesture_event_handler(input_dev);
	if (ret >= 0)
		goto exit_work_func;
#endif

	if (gt1x_halt) {
		GTP_DEBUG("Ignore interrupts after suspend");
        return IRQ_HANDLED;
	}
#ifdef CONFIG_GTP_TOUCH_DETECT
	if(touch_detect_flag == 1) {
        	touch_detect_flag = 0;
		touch_rowdata_event_handler(input_dev);
		gt1x_touch_detect_switch(SWITCH_OFF);
		goto exit_work_func;
	}
#endif

	ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
	if (ret < 0) {
		GTP_ERROR("I2C transfer error!");
#ifndef CONFIG_GTP_ESD_PROTECT
		gt1x_power_reset();
#endif
		goto exit_work_func;
	}

	finger = point_data[0];
	if (finger == 0x00)
		gt1x_request_event_handler();

#ifdef CONFIG_SUPPORT_EARSENSE
	es_touch_count = finger & 0x0f;
#endif

	if ((finger & 0x80) == 0) {
#ifdef CONFIG_HOTKNOT_BLOCK_RW
		if (!hotknot_paired_flag)
#endif
		{
			goto exit_eint;
		}
	}

#ifdef CONFIG_HOTKNOT_BLOCK_RW
	ret = hotknot_event_handler(point_data);
	if (!ret)
		goto exit_work_func;
#endif

#ifdef CONFIG_GTP_PROXIMITY
	ret = gt1x_prox_event_handler(point_data);
	if (ret > 0)
		goto exit_work_func;
#endif

#ifdef CONFIG_GTP_WITH_STYLUS
	ret = gt1x_touch_event_handler(point_data, input_dev, pen_dev);
#else
	ret = gt1x_touch_event_handler(point_data, input_dev, NULL);
#endif

exit_work_func:
	if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
		ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
		if (ret < 0)
			GTP_ERROR("I2C write end_cmd  error!");
	}
exit_eint:
	gt1x_irq_enable();
	return IRQ_HANDLED;
}

/*
 * Devices Tree support,
*/
#ifdef CONFIG_OF
/**
 * gt1x_parse_dt - parse platform infomation form devices tree.
 */
static int gt1x_parse_dt(struct device *dev)
{
	struct device_node *np;
	int ret = 0;

	if (!dev)
		return -ENODEV;

	np = dev->of_node;
	gt1x_int_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
	gt1x_rst_gpio = of_get_named_gpio(np, "goodix,reset-gpio", 0);
	gt1x_1v8_gpio = of_get_named_gpio(np, "goodix,1v8-gpio", 0);
    	if (!gpio_is_valid(gt1x_int_gpio) || !gpio_is_valid(gt1x_rst_gpio) || !gpio_is_valid(gt1x_1v8_gpio)) {
		GTP_ERROR("Invalid GPIO, irq-gpio:%d, rst-gpio:%d",
		gt1x_int_gpio, gt1x_rst_gpio);
        return -EINVAL;
	}

	vdd_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vdd_ana)) {
		GTP_ERROR("regulator get of vdd_ana failed");
		ret = PTR_ERR(vdd_ana);
		vdd_ana = NULL;
		return ret;
	}

	return 0;
}

/**
 * gt1x_power_switch - power switch .
 * @on: 1-switch on, 0-switch off.
 * return: 0-succeed, -1-faileds
 */
int gt1x_power_switch(int on)
{

	int ret;
	struct i2c_client *client = gt1x_i2c_client;

	if (!client || !vdd_ana)
	return -1;

	if (on) {
		GTP_DEBUG("GTP power on.");
		ret = regulator_enable(vdd_ana);
		GTP_GPIO_OUTPUT(GTP_1V8_PORT, 1);
	} else {
		GTP_DEBUG("GTP power off.");
		ret = regulator_disable(vdd_ana);
		GTP_GPIO_OUTPUT(GTP_1V8_PORT, 0);
	}

	usleep_range(10000,10001);
	return ret;

}
#endif

static void gt1x_release_resource(void)
{
	if (gpio_is_valid(GTP_INT_PORT)) {
		gpio_direction_input(GTP_INT_PORT);
		gpio_free(GTP_INT_PORT);
	}

	if (gpio_is_valid(GTP_RST_PORT)) {
		gpio_direction_output(GTP_RST_PORT, 0);
		gpio_free(GTP_RST_PORT);
	}

	if (gpio_is_valid(GTP_1V8_PORT)) {
		gpio_direction_output(GTP_1V8_PORT, 0);
		gpio_free(GTP_1V8_PORT);
	}

#ifndef CONFIG_GTP_INT_SEL_SYNC
	if (default_pctrl) {
		pinctrl_put(default_pctrl);
		default_pctrl = NULL;
	}
#endif

#ifdef CONFIG_OF
	if (vdd_ana) {
		gt1x_power_switch(SWITCH_OFF);
        regulator_put(vdd_ana);
		vdd_ana = NULL;
	}
#endif

	if (input_dev) {
		input_unregister_device(input_dev);
		input_dev = NULL;
	}
}

/**
 * gt1x_request_gpio - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_gpio(void)
{
	s32 ret = 0;

	ret = gpio_request(GTP_1V8_PORT, "GTP_1V8_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_1V8_PORT, ret);
		ret = -ENODEV;
	}

	ret = gpio_request(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		gt1x_i2c_client->irq = gpio_to_irq(GTP_INT_PORT);
	}

	ret = gpio_request(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	return ret;
}

/**
 * gt1x_request_irq - Request interrupt.
 * Return
 *      0: succeed, -1: failed.
 */
static s32 gt1x_request_irq(void)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG("INT trigger type:%x", gt1x_int_type);
	ret = devm_request_threaded_irq(&gt1x_i2c_client->dev,
			gt1x_i2c_client->irq,
			gt1x_ts_irq_handler,
			gt1x_ts_work_thread,
			irq_table[gt1x_int_type],
			gt1x_i2c_client->name,
			gt1x_i2c_client);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		return -1;
	} else {
		gt1x_irq_disable();
		return 0;
	}
}

/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *      0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#ifdef CONFIG_GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#ifdef CONFIG_GTP_TYPE_B_PROTOCOL
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(input_dev, GTP_MAX_TOUCH, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GTP_MAX_TOUCH);
#endif
#endif
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#ifdef CONFIG_GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++)
		input_set_capability(input_dev, EV_KEY, gt1x_touch_key_array[index]);
#endif

#ifdef CONFIG_GTP_GESTURE_WAKEUP
	input_set_capability(input_dev, EV_KEY, KEY_GES_REGULAR);
	input_set_capability(input_dev, EV_KEY, KEY_GES_CUSTOM);
#endif

#ifdef CONFIG_GTP_CHANGE_X2Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_x_max, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_dev->name = "goodix-ts";
	input_dev->phys = "input/ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	ret = input_register_device(input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}

	return 0;
}



/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, <0: failed.
 */
static int gt1x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;

	/* do NOT remove these logs */
	GTP_INFO("GTP Driver Version: %s,slave addr:%02xh",
			GTP_DRIVER_VERSION, client->addr);

	gt1x_i2c_client = client;
	spin_lock_init(&irq_lock);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

#ifdef CONFIG_OF	/* device tree support */
	if (client->dev.of_node) {
		ret = gt1x_parse_dt(&client->dev);
		if (ret < 0)
			return -EINVAL;
	}
#else
#error [GOODIX]only support devicetree platform
#endif

	/*
	 * Pinctrl pull-up state is required by INT gpio if
	 * your kernel has the output restriction of gpio tied
	 * to IRQ line(kernel3.13 and later version).
	 */
#ifndef CONFIG_GTP_INT_SEL_SYNC
	default_pctrl = pinctrl_get_select_default(&client->dev);
	if (IS_ERR(default_pctrl)) {
		GTP_ERROR("Please add default pinctrl state"
				"(pull-up irq-gpio)");
		return PTR_ERR(default_pctrl);
	}
#endif

	/* gpio resource */
	ret = gt1x_request_gpio();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		goto exit_clean;
	}

	/* power on */
	ret = gt1x_power_switch(SWITCH_ON);
	if (ret < 0) {
		GTP_ERROR("Power on failed");
		goto exit_clean;
	}

	/* reset ic & do i2c test */
	ret = gt1x_reset_guitar();
	if (ret != 0) {
		ret = gt1x_power_switch(SWITCH_OFF);
		if (ret < 0)
			goto exit_clean;
		ret = gt1x_power_switch(SWITCH_ON);
		if (ret < 0)
			goto exit_clean;
		ret = gt1x_reset_guitar(); /* retry */
		if (ret != 0) {
			GTP_ERROR("Reset guitar failed!");
			goto exit_clean;
		}
	}
	tp_detect_flag =1;/*goodix detect ok 1*/
	/* check firmware, initialize and send
	 * chip configuration data, initialize nodes */
	gt1x_init();

	ret = gt1x_request_input_dev();
	if (ret < 0)
		goto err_input;

	ret = gt1x_request_irq();
	if (ret < 0)
		goto err_irq;

#ifdef CONFIG_GTP_ESD_PROTECT
	/* must before auto update */
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#ifdef CONFIG_GTP_AUTO_UPDATE
	do {
		struct task_struct *thread = NULL;
		thread = kthread_run(gt1x_auto_update_proc,
				(void *)NULL,
				"gt1x_auto_update");
		if (IS_ERR(thread))
			GTP_ERROR("Failed to create auto-update thread: %d.", ret);
	} while (0);
#endif

	gt1x_register_powermanger();
	gt1x_irq_enable();

	if (device_create_file(&client->dev, &dev_attr_tp_debug_log)) {
		GTP_ERROR("device_create_file failt");
		goto err_irq;
	}
	tp_detect_flag =1;/*goodix detect ok 1*/
	return 0;

err_irq:
err_input:
	gt1x_deinit();
exit_clean:
	gt1x_release_resource();
	GTP_ERROR("GTP probe failed:%d tp_detect_flag %d", ret , tp_detect_flag);
	tp_detect_flag =0;/*goodix detect fail 1*/
	return -ENODEV;
}

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
	GTP_INFO("GTP driver removing...");
	gt1x_unregister_powermanger();

	gt1x_deinit();
	gt1x_release_resource();

    return 0;
}

#if defined(CONFIG_FB)
/* frame buffer notifier block control the suspend/resume procedure */
static struct notifier_block gt1x_fb_notifier;

static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;

#ifdef CONFIG_GTP_INCELL_PANEL
#ifndef FB_EARLY_EVENT_BLANK
	#error Need add FB_EARLY_EVENT_BLANK to fbmem.c
#endif

	if (ev_data && ev_data->data && event == FB_EARLY_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resume by fb notifier.");
			gt1x_resume();
        }
    }
#else
	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resume by fb notifier.");
			gt1x_resume();
        }
    }
#endif

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_POWERDOWN) {
			GTP_DEBUG("Suspend by fb notifier.");
			gt1x_suspend();
		}
	}

	return 0;
}
#elif defined(CONFIG_MSM_RDM_NOTIFY)
static struct notifier_block msm_drm_notif;
static int msm_drm_notifier_callback(
	struct notifier_block *self, unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;

	if (event != MSM_DRM_EARLY_EVENT_BLANK
			&& MSM_DRM_EVENT_BLANK != event)
	return 0;
	/*add for morgan for EID447*/
	if (evdata->id != MSM_DRM_PRIMARY_DISPLAY)
	return 0;

	if ((evdata) && (evdata->data)) {
		blank = evdata->data;
	GTP_DEBUG("%s blank[%d],event[0x%lx],evdata->id[%d]\n",
				__func__, *blank, event, evdata->id);

	if ((*blank == MSM_DRM_BLANK_UNBLANK_CUST)
		&& (event == MSM_DRM_EARLY_EVENT_BLANK)) {
			GTP_DEBUG("%s TP resume start\n", __func__);
			gt1x_resume();
			GTP_DEBUG("%sTP resume end\n", __func__);
	} else if (((*blank == MSM_DRM_BLANK_POWERDOWN)
		&& (event == MSM_DRM_EARLY_EVENT_BLANK))
		|| (*blank == MSM_DRM_BLANK_NORMAL)) {
			GTP_DEBUG("%s:TP suspend start\n", __func__);
			gt1x_suspend();
			GTP_DEBUG("%s:TP suspend end\n", __func__);
		}
	}
	return 0;

}
#endif

#if defined(CONFIG_PM)
/**
 * gt1x_ts_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_suspend(struct device *dev)
{
    return gt1x_suspend();
}

/**
 * gt1x_ts_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_resume(struct device *dev)
{
	return gt1x_resume();
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops gt1x_ts_pm_ops = {
	.suspend = gt1x_pm_suspend,
	.resume = gt1x_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void gt1x_ts_early_suspend(struct early_suspend *h)
{
	gt1x_suspend();
}

static void gt1x_ts_late_resume(struct early_suspend *h)
{
	gt1x_resume();
}

static struct early_suspend gt1x_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = gt1x_ts_early_suspend,
	.resume = gt1x_ts_late_resume,
};
#endif


static int gt1x_register_powermanger(void)
{
#if   defined(CONFIG_FB)
	gt1x_fb_notifier.notifier_call = gtp_fb_notifier_callback;
	fb_register_client(&gt1x_fb_notifier);

#elif defined(CONFIG_MSM_RDM_NOTIFY)
	msm_drm_notif.notifier_call = msm_drm_notifier_callback;
	msm_drm_register_client(&msm_drm_notif);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	register_early_suspend(&gt1x_early_suspend);
#endif
	return 0;
}

static int gt1x_unregister_powermanger(void)
{
#if   defined(CONFIG_FB)
	fb_unregister_client(&gt1x_fb_notifier);

#elif defined(CONFIG_MSM_RDM_NOTIFY)
	msm_drm_unregister_client(&msm_drm_notif);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&gt1x_early_suspend);
#endif
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gt1x_match_table[] = {
		{.compatible = "goodix,gt1x",},
		{ },
};
#endif

static const struct i2c_device_id gt1x_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct i2c_driver gt1x_ts_driver = {
	.probe = gt1x_ts_probe,
	.remove = gt1x_ts_remove,
	.id_table = gt1x_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = gt1x_match_table,
#endif
#if defined(CONFIG_PM)
		   .pm = &gt1x_ts_pm_ops,
#endif
		   },
};

#ifndef TPD_DETECT
/**
 * gt1x_ts_init - Driver Install function.
 * Return   0---succeed.
 */
static int __init gt1x_ts_init(void)
{
	GTP_INFO("GTP driver installing...");

	if (i2c_add_driver(&gt1x_ts_driver)!= 0 ) {
		GTP_INFO("unable to add goodix i2c driver.\n");
		return -1;
	}

	return 0;
}

/**
 * gt1x_ts_exit - Driver uninstall function.
 * Return   0---succeed.
 */
static void __exit gt1x_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&gt1x_ts_driver);
}

#ifdef CONFIG_SUPPORT_EARSENSE
/**
 * gt1x_EarSense_switch - EarSense switch .
 * @on: 1-EarSense on, 0-EarSense off.
 * return: 0-succeed, -1-faileds
 */
int gt1x_EarSense_switch(int on)
{

	int ret;
	u8 buffer[3] = { 0x80, 0x40, 0x61 };

	if (on) {
		GTP_DEBUG("GTP EarSense on.");
		buffer[2] = 0x60;
	} else {
		GTP_DEBUG("GTP EarSense off.");
		buffer[2] = 0x61;
	}

	ret = gt1x_i2c_write(gt1x_i2c_client->addr, buffer, 3);
	if (ret < 0)
		GTP_ERROR("I2C write end_cmd  error!");

	msleep(10);
	return ret;

}

int gt1x_EarSense_ReadRawdata_NoTouch(void)
{
	int ret;
	int timerout_count = 0;
	u8 buffer[50] = { 0x80, 0x4C, 0x01 };

	ret = gt1x_i2c_write(gt1x_i2c_client->addr, buffer, 3);
	if (ret < 0)
	{
		GTP_ERROR("I2C write end_cmd  error!");
		return ret;
	}

	while(1)
	{
		gt1x_i2c_read(0x804C, buffer, 1);
		if (ret < 0)
		{
			return ret;
		}

		if(0x02 == buffer[0])
			break;

		if(0x03 == buffer[0])
		{
			return 5;
		}

		if(timerout_count > 100)
		{
			return -91;
		}
		msleep(2);
		timerout_count ++;
	}


	gt1x_i2c_read(0xA05E, buffer, 50);
	if (ret < 0)
	{
		return ret;
	}

	//??0x804C
	buffer[0] = 0x80;
	buffer[1] = 0x4C;
	buffer[2] = 0x00;
	ret = gt1x_i2c_write(gt1x_i2c_client->addr, buffer, 3);
	if (ret < 0)
	{
		GTP_ERROR("I2C write end_cmd  error!");
		return ret;
	}

	return ret;
}

int gt1x_EarSense_ReadDiffdata_Touch(void)
{
	int ret;
	int timerout_count = 0;
	u8 buffer[50] = { 0x80, 0x4C, 0x01 };

	ret = gt1x_i2c_read(0x804d, buffer, 1);
	if (ret < 0)
	{
		return ret;
	}
	if(0x01 == buffer[0])
	{
		buffer[0] = 0x80;
		buffer[1] = 0x4d;
		buffer[2] = 0x02;
		ret = gt1x_i2c_write(gt1x_i2c_client->addr, buffer, 3);
		if (ret < 0)
		{
			GTP_ERROR("I2C write end_cmd  error!");
			return ret;
		}
	}
	else
		{
			return -92;

	}

	gt1x_i2c_read(0xA090, buffer, 544);
	if (ret < 0)
	{
		return ret;
	}

	gt1x_i2c_read(0x804B, buffer, 1);
	if (ret < 0)
	{
		return ret;
	}

	while(1)
	{
		gt1x_i2c_read(0xa31a, buffer, 1);
		if (ret < 0)
		{
			return ret;
		}
		if(0x00 == buffer[0])
			break;

		gt1x_i2c_read(0x804d, buffer, 1);
		if (ret < 0)
		{
			return ret;
		}

		if(timerout_count > 100)
		{
			return -96;
		}
		timerout_count ++;
	}

	//??0x804d
	buffer[0] = 0x80;
	buffer[1] = 0x4d;
	buffer[2] = 0x00;
	ret = gt1x_i2c_write(gt1x_i2c_client->addr, buffer, 3);
	if (ret < 0)
	{
		GTP_ERROR("I2C write end_cmd  error!");
		return ret;
	}

	return ret;
}
#endif

module_init(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL v2");
#endif
