/*
 *
 * Copyright
 * Aurthor:suzhiguang@oneplus.com
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include "fsa4480.h"

#define I2C_RETRIES 2
#define I2C_RETRY_DELAY 5 /* ms */

struct fsa4480 *gFsa4480;
bool fsa4480_enable = false;
EXPORT_SYMBOL_GPL(fsa4480_enable);
extern bool audio_adapter_flag;

/*
*suzhiguang:read register value
*register address
*read value
*/
int fsa4480_read_data(struct fsa4480 *fsa,
				unsigned char reg,
				int len, unsigned char value[])
{
	struct i2c_client *fsa_client;
	int err;
	int tries = 0;
	int error = 0;
	struct i2c_msg msgs[] = {
		{
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};

	if (fsa == NULL) {
		pr_err("fsa read data:No device available\n");
		return -1;
	}

	if (fsa->i2c) {
		fsa_client = fsa->i2c;
		msgs[0].addr = fsa_client->addr;
		msgs[1].addr = fsa_client->addr;

		do {
			err = i2c_transfer(fsa_client->adapter, msgs,
							ARRAY_SIZE(msgs));
			if (err != ARRAY_SIZE(msgs))
				msleep_interruptible(I2C_RETRY_DELAY);
		} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

		if (err != ARRAY_SIZE(msgs)) {
			dev_err(&fsa_client->dev, "read transfer error %d\n",
									err);
			error = -1;
		}

	} else {
		pr_err("No device available\n");
		error = -1;
	}
	return error;
}

/*
*suzhiguang:write register value
*/
int fsa4480_write_data(struct fsa4480 *fsa, char *writebuf, int writelen)
{
	int ret = 0;
	struct i2c_client *client;

	client = fsa->i2c;
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("fsa4480_write_data [IIC]: i2c_write error, ret=%d", ret);
	}

	return ret;
}

/*
*suzhiguang:write register value
*register address
*write value
*/
int fsa4480_i2c_write_reg(struct fsa4480 *fsa, unsigned char regaddr, unsigned char regvalue)
{
	unsigned char buf[2] = {0};

	if (fsa == NULL) {
		pr_err("fsa write reg: No device available\n");
		return -1;
	}

	buf[0] = regaddr;
	buf[1] = regvalue;
	return fsa4480_write_data(fsa, buf, sizeof(buf));
}

#if 0
static void parseString(const char *buf)
{
	int i,j;
	char commandStr[10];
	char regStr[10];
	char valueStr[10];
	int reg, value;

	pr_err("fsa input buffer is %s\n", buf);
	for(i=0,j=0; buf[i]!=' ' && buf[i]!='\0'; i++,j++){
		commandStr[j] = buf[i];
	}
	commandStr[j] = '\0';
	pr_err("commandStr is %s\n", commandStr);
	if (buf[i] =='\0')
		return;

	for(; buf[i]==' ' && buf[i]!='\0'; i++){}
	if (buf[i] == '\0')
		return;


	for(j=0;buf[i]!=' ' && buf[i]!='\0';i++,j++) {
		regStr[j] = buf[i];
	}
	regStr[j] = '\0';
	pr_err("regStr is %s\n", regStr);
	if (buf[i] =='\0')
		return;

	for(; buf[i]==' ' && buf[i]!='\0'; i++){}
	if (buf[i] == '\0')
		return;


	for(j=0;buf[i]!=' ' && buf[i]!='\0';i++,j++) {
		valueStr[j] = buf[i];
	}
	valueStr[j] = '\0';
	pr_err("regStr is %s\n", valueStr);

	reg = atoi(regStr);
	value = atoi(valueStr);

	pr_err("parse comand is %s reg =0x%x value = 0x%x\n", commandStr, reg, value);
}
#endif

static ssize_t fsa4480_state_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned char reg04 = 0 , reg05 = 0, reg17 = 0, val = 0;
        unsigned char regValue = 0;

    pr_err("%s",__func__);

	if (gFsa4480 == NULL) {
	    pr_err("%s:No fsa device found.\n",__func__);
		 return  -EINVAL;
	}

//	parseString(buf);

    if (sysfs_streq(buf, "lr")) {

		pr_err("%s lr\n",__func__);
				fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, 0x9f);
				fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWSEL, 0x00);
		fsa4480_read_data(gFsa4480, FSA4480_SWEN, 1, &regValue);
		pr_err("%s FSA4480_SWEN = %x\n", __func__, regValue);
		fsa4480_read_data(gFsa4480, FSA4480_SWSEL, 1, &regValue);
		pr_err("%s FSA4480_SWSEL = %x\n", __func__, regValue);

    } else if(sysfs_streq(buf, "mic")) {

    } else if(sysfs_streq(buf, "all")) {

		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, 0x98);
		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWSEL, 0x00);

		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, 0x9b);
		fsa4480_i2c_write_reg(gFsa4480, FSA4480_FUN_EN, 0x0d);
		msleep_interruptible(30);

		fsa4480_read_data(gFsa4480, FSA4480_JACTYP, 1, &reg17);
		fsa4480_read_data(gFsa4480, FSA4480_SWEN, 1, &reg04);
		fsa4480_read_data(gFsa4480, FSA4480_SWSEL, 1, &reg05);
		msleep_interruptible(10);
		val = reg17;
		switch(val)
		{
			case 0x08:		// 4 pole,
			case 0x04:		// 4 pole,
							// automatically configure if it is 4 pole audio jack
				break;
			case 0x02:		// 3 pole
					fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, (reg04 & 0x7f));
					msleep_interruptible(50);
					fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, (reg04 | 0x80));
				break;
			default:
				pr_err("No audio accessory was recognized\n");
			break;
		}
    } else if(sysfs_streq(buf, "off")) {
		fsa4480_read_data(gFsa4480, FSA4480_SWEN, 1, &reg04);
		fsa4480_read_data(gFsa4480, FSA4480_SWSEL, 1, &reg05);

		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, (reg04 & 0x98));
		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWSEL, (reg05 | 0x18));
		msleep_interruptible(30);
    } else {
        count = -EINVAL;
    }
    return count;
}

static ssize_t fsa4480_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int err = 0;
	u8 reg = 0x0;
	u8 statu = 0x0;
	char *temp = buf;
	ssize_t buf_size = 0;

	if (gFsa4480 == NULL) {
	    pr_err("%s:No fsa device found.\n",__func__);
		 return  -EINVAL;
	}

	for (reg = 0x0; reg <= FSA4480_RESET; reg++) {
		err = fsa4480_read_data(gFsa4480, reg, 1, &statu);
		if (err < 0) {
			pr_err("%s: read reg %#04x is failed\n", __func__, reg);
		}
		sprintf(temp,"%#04x: %#04x\n", reg, statu);
		memset(&statu, 0, sizeof(statu));
		while (*temp != '\0') {
			buf_size++;
			temp++;
		}
	}

	*temp = '\0';

	pr_err("%s:%s\n", __func__, buf);

	return buf_size;
}

static struct device_attribute fsa4480_state_attr =
     __ATTR(test, 0444, fsa4480_state_show, fsa4480_state_store);


/*
*suzhiguang:fsa init.
*/
#if 0
static void fsa4480_init(struct fsa4480 *fsa4480)
{
	fsa4480_i2c_write_reg(fsa4480, FSA4480_OVPMSK, 0xff);		// OVP Interrupt Mask: ffh= disable all OVP interrupt
//	fsa4480_i2c_write_reg(FSA4480_DECMSK, 0x00); 	// Audio Jack and Moisture Detection Interrupt Mask:

	fsa4480_i2c_write_reg(fsa4480, FSA4480_AUDDATA0, 0x20);		// set mic threshold data0
	fsa4480_i2c_write_reg(fsa4480, FSA4480_AUDDATA1, 0xff);		// set mic threshold data1


/*
	//Configure audio turn on timing to suppress pop noise
	fsa4480_i2c_write_reg(FSA4480_CNT_L,0x01);		//	Set Audio L slow turn on timing,
	fsa4480_i2c_write_reg(FSA4480_CNT_R,0x01);		//	Set Audio R slow turn on timing,
	fsa4480_i2c_write_reg(FSA4480_CNT_MIC,0x01);		//	Set Audio MIC slow turn on timing,
	fsa4480_i2c_write_reg(FSA4480_CNT_SEN,0x01);		//	Set Audio Sense slow turn on timing,
	fsa4480_i2c_write_reg(FSA4480_CNT_GND,0x01);		//	Set Audio Ground slow turn on timing,
	fsa4480_i2c_write_reg(FSA4480_TIM_R,0x00);		//	Set Delay between R and L
	fsa4480_i2c_write_reg(FSA4480_TIM_MIC,0x00);		//	Set Delay between Mic and L
	fsa4480_i2c_write_reg(FSA4480_TIM_SEN,0x00);		//	Set Delay between Sense and L
	fsa4480_i2c_write_reg(FSA4480_TIM_GND,0x00);		//	Set Delay between Analog Ground and L
*/

/*
	//configure moisture detection
	fsa4480_i2c_write_reg(FSA4480_RES_SET, 0x00);    //  select which pin is enabled for moisture detection
	fsa4480_i2c_write_reg(FSA4480_RES_THR, 0x16);	// 	set detection threshold: 16h = 22 or 220 Kohm
	fsa4480_i2c_write_reg(FSA4480_RES_INV, 0x00);	// 	set detection interval: 00h =single

*/

	fsa4480_i2c_write_reg(fsa4480, FSA4480_FUN_EN, 0x0d); 	// Function Setting:  h4b= DEC open drain output,
	fsa4480_i2c_write_reg(fsa4480, FSA4480_SWEN, 0x80);		// Switch Enable: 80h= enable FSA4480 device
//	fsa4480_i2c_write_reg(FSA4480_SWSEL, 0x18);		// set default switch:  18h= to DP/DN
}
#endif

/*2018/06/14 @bsp add for support notify audio adapter switch*/
static int cc_audio_adapter_detect_callback(struct notifier_block *nb,
				unsigned long value, void *data)
{
	unsigned char reg04 =0 , reg05 =0, reg17 =0, val = 0;

	if (gFsa4480 == NULL) {
		pr_err("%s:No fsa device found.\n",__func__);
		return NOTIFY_OK;
	}

	if (value == 1) {
		pr_err("%s:audio_adapter attached!\n", __func__);
		pr_err("%s:open the audio switch!\n", __func__);

		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, 0x9f);
		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWSEL, 0x00);
		msleep_interruptible(1);
		fsa4480_i2c_write_reg(gFsa4480, FSA4480_FUN_EN, 0x0d);
		msleep_interruptible(30);

		fsa4480_read_data(gFsa4480, FSA4480_JACTYP, 1, &reg17);
		fsa4480_read_data(gFsa4480, FSA4480_SWEN, 1, &reg04);
		fsa4480_read_data(gFsa4480, FSA4480_SWSEL, 1, &reg05);
		pr_err("%s:FSA4480_JACTYP =%x FSA4480_SWEN =%x FSA4480_SWSEL =%x\n",
				__func__, reg17, reg04, reg05);

		msleep_interruptible(10);
		val = reg17;
		switch(val)
		{
			case 0x08:		// 4 pole,
			case 0x04:		// 4 pole,
							// automatically configure if it is 4 pole audio jack
				break;
			case 0x02:		// 3 pole
					reg04 |= 0x7;
					fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, (reg04 & 0x7f));
					msleep_interruptible(50);
					fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, (reg04 | 0x80));
					fsa4480_read_data(gFsa4480, FSA4480_SWEN, 1, &reg04);
					pr_err("%s: reg04 is %#x\n", __func__, reg04);
				break;
			default:
				pr_err("fsa4480 No audio accessory was recognized");

		}

		if (gpio_is_valid(gFsa4480->mbhc_en)) {
			gpio_set_value_cansleep(gFsa4480->mbhc_en, 1);
			pr_err("gFsa4480->mbhc_en set to 1\n");
		} else {
			pr_err("gpio_is_valid failed\n");
		}

	} else if (value == 0) {
		pr_err("%s:audio_adapter removal!\n", __func__);
		pr_err("%s:close the audio switch!\n", __func__);

		if (gpio_is_valid(gFsa4480->mbhc_en)) {
			gpio_set_value_cansleep(gFsa4480->mbhc_en, 0);
			pr_err("gpio set value to 0\n");
		} else {
			pr_err("gpio_is_valid failed\n");
		}
		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, 0x80);
		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWSEL, 0x18);
		usleep_range(50, 55);
		fsa4480_i2c_write_reg(gFsa4480, FSA4480_SWEN, 0x98);
	} else
		pr_err("%s:audio adapter value = %lu\n", __func__, value);

	return NOTIFY_OK;
}

static struct notifier_block typec_cc_notifier = {
	.notifier_call = cc_audio_adapter_detect_callback,
};

/* liuhaituo@MM.Audio 2018/8/8 Solve not detected the headset after restarting the phone
 *  after plugging in the headset
 */
static void call_wcd_detect_headset(struct work_struct *work)
{
	if (audio_adapter_flag)
		cc_audio_adapter_detect_callback(NULL, 1, NULL);

	pr_err("%s: enter\n", __func__);
	register_cc_notifier_client(&typec_cc_notifier);
	pr_err("%s: exit\n", __func__);
}

/*
*suzhiguang:fsa probe.
*/
static int fsa4480_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct fsa4480 *fsa4480;
	struct device_node *np = i2c->dev.of_node;
	unsigned char regValue = 0;
	int ret = 0;

	pr_err("fsa4480_i2c_probe addr=0x%x\n", i2c->addr);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	fsa4480 = devm_kzalloc(&i2c->dev, sizeof(struct fsa4480), GFP_KERNEL);
	if (fsa4480 == NULL)
		return -ENOMEM;

	fsa4480->dev = &i2c->dev;
	fsa4480->i2c = i2c;
	i2c_set_clientdata(i2c, fsa4480);

	if (fsa4480_read_data(fsa4480, FSA4480_DEVID, 1, &regValue) < 0) {
		pr_err("fsa4480 read device id error\n");
		return -EINVAL;
	} else {
		pr_err("fsa4480 device id = %x\n", regValue);
	}

	if (regValue == 0x9) {
		pr_err("fsa:fsa4480 found\n");
		gFsa4480 = fsa4480;
		fsa4480_enable = true;
	} else {
		pr_err("fsa:no fsa4480 found.\n");
		return 0;
	}

	fsa4480->mbhc_en = of_get_named_gpio(np, "mbhc_en", 0);
	if (fsa4480->mbhc_en < 0){
		pr_err("fsa4480 of get gpio failed\n");
		return ret;
	} else {
		ret = devm_gpio_request_one(&i2c->dev, fsa4480->mbhc_en,
			GPIOF_OUT_INIT_LOW, "FSA4480_MBHC");
		if (ret) {
			pr_err("%s devm_gpio_request_one fsa4480->mbhc_en failed\n",__func__);
			return ret;
		}
	}
	//fsa4480_init(fsa4480);

    ret = sysfs_create_file(&i2c->dev.kobj, &fsa4480_state_attr.attr);
    if(ret < 0)
    {
        pr_err("%s sysfs_create_file fsa4480_state_attr error.",__func__);
    }

	/* liuhaituo@MM.Audio 2018/8/8 Solve not detected the headset after restarting the phone
	 * after plugging in the headset*/
	INIT_DELAYED_WORK(&fsa4480->call_wcd_dwork, call_wcd_detect_headset);
	schedule_delayed_work(&fsa4480->call_wcd_dwork, msecs_to_jiffies(8000));
	return 0;
}

static int fsa4480_i2c_remove(struct i2c_client *i2c)
{
/*2018/06/14 @bsp add for support notify audio adapter switch*/
	unregister_cc_notifier_client(&typec_cc_notifier);
	return 0;
}

static const struct i2c_device_id fsa4480_i2c_id[] = {
	{ "fsa4480", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fsa4480_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id fsa4480_dt_match[] = {
	{ .compatible = "fsa,fsa4480" },
	{ },
};
#endif

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = "fsa4480",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(fsa4480_dt_match),
	},
	.probe =    fsa4480_i2c_probe,
	.remove =   fsa4480_i2c_remove,
	.id_table = fsa4480_i2c_id,
};

static int __init fsa4480_i2c_init(void)
{
	int ret = 0;

	pr_err("fsa4480 driver fsa4480_i2c_init\n");

	ret = i2c_add_driver(&fsa4480_i2c_driver);

	return ret;
}
module_init(fsa4480_i2c_init);

static void __exit fsa4480_i2c_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}
module_exit(fsa4480_i2c_exit);

MODULE_DESCRIPTION("ASoC FSA4480 driver");
MODULE_LICENSE("GPL");

