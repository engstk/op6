/* drivers/input/touchscreen/gt1x_extents.c
 *
 * 2010 - 2016 Goodix Technology.
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/input.h>

#include <asm/uaccess.h>
#include <linux/proc_fs.h>	/*proc */

#include <asm/ioctl.h>
#include "gt1x_generic.h"

#ifdef CONFIG_GTP_GESTURE_WAKEUP

#define GESTURE_NODE "goodix_gesture"
#define GESTURE_MAX_POINT_COUNT    64

#pragma pack(1)
typedef struct {
	u8 ic_msg[6];		/*from the first byte */
	u8 gestures[4];
	u8 data[3 + GESTURE_MAX_POINT_COUNT * 4 + 80];	/*80 bytes for extra data */
} st_gesture_data;
#pragma pack()

#define SETBIT(longlong, bit)   (longlong[bit/8] |=  (1 << bit%8))
#define CLEARBIT(longlong, bit) (longlong[bit/8] &=(~(1 << bit%8)))
#define QUERYBIT(longlong, bit) (!!(longlong[bit/8] & (1 << bit%8)))

#define CHKBITS_32          32
#define CHKBITS_16          16
#define CHKBITS_8           8

int gesture_enabled = 0;    /* module switch */
DOZE_T gesture_doze_status = DOZE_DISABLED; /* doze status */

static u8 gestures_flag[32]; /* gesture flag, every bit stands for a gesture */
static st_gesture_data gesture_data; /* gesture data buffer */
static struct mutex gesture_data_mutex; /* lock for gesture data */

static ssize_t gt1x_gesture_data_read(struct file *file, char __user * page, size_t size, loff_t * ppos)
{
	s32 ret = -1;
	GTP_DEBUG("visit gt1x_gesture_data_read. ppos:%d", (int)*ppos);
	if (*ppos) {
		return 0;
	}
	if (size == 4) {
		ret = copy_to_user(((u8 __user *) page), "GT1X", 4);
		return 4;
	}
	ret = simple_read_from_buffer(page, size, ppos, &gesture_data, sizeof(gesture_data));

	GTP_DEBUG("Got the gesture data.");
	return ret;
}

static ssize_t gt1x_gesture_data_write(struct file *filp, const char __user * buff, size_t len, loff_t * off)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();

	ret = copy_from_user(&gesture_enabled, buff, 1);
	if (ret) {
		GTP_ERROR("copy_from_user failed.");
		return -EPERM;
	}

	GTP_DEBUG("gesture enabled:%x, ret:%d", gesture_enabled, ret);

	return len;
}

/**
 * calc_checksum - Calc checksum.
 * @buf: data to be calc
 * @len: length of buf.
 * @bits: checkbits
 * Return true-pass, false:not pass.
 */
static bool calc_checksum(u8 *buf, int len, int bits)
{
    int i;

    if (bits == CHKBITS_16) {
        u16 chksum, *b = (u16 *)buf;

        if (len % 2) {
            return false;
        }

        len /= 2;
        for (i = 0, chksum = 0; i < len; i++) {
            if (i == len - 1)
                chksum += le16_to_cpu(b[i]);
            else
                chksum += be16_to_cpu(b[i]);
        }
        return chksum == 0 ? true : false;
    } else if (bits == CHKBITS_8) {
        u8 chksum;

        for (i = 0, chksum =0; i < len; i++) {
            chksum += buf[i];
        }
        return chksum == 0 ? true : false;
    }

    return false;
}

int gesture_enter_doze(void)
{
	int retry = 0;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("Entering doze mode...");
	while (retry++ < 5) {
		if (!gt1x_send_cmd(0x08, 0)) {
			gesture_doze_status = DOZE_ENABLED;
			GTP_DEBUG("Working in doze mode!");
			return 0;
		}
		msleep(10);
	}
	GTP_ERROR("Send doze cmd failed.");
	return -1;
}

s32 gesture_event_handler(struct input_dev * dev)
{
	u8 doze_buf[4] = { 0 }, ges_type;
    static int err_flag1 = 0, err_flag2 = 0;
	int len, extra_len, need_chk;
    unsigned int key_code;
    s32 ret = 0;

    if (DOZE_ENABLED != gesture_doze_status) {
        return -1;
    }

    /** package: -head 4B + track points + extra info-
        * - head -
        *  doze_buf[0]: gesture type,
        *  doze_buf[1]: number of gesture points ,
        *  doze_buf[2]: protocol type,
        *  doze_buf[3]: gesture extra data length.
        */
	ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE, doze_buf, 4);
	if (ret < 0) {
        return 0;
    }

    ges_type = doze_buf[0];
    len = doze_buf[1];
    need_chk = doze_buf[2] & 0x80;
    extra_len = doze_buf[3];

    GTP_DEBUG("0x%x = 0x%02X,0x%02X,0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE,
        doze_buf[0], doze_buf[1], doze_buf[2], doze_buf[3]);

    if (len > GESTURE_MAX_POINT_COUNT) {
		GTP_ERROR("Gesture contain too many points!(%d)", len);
		len = GESTURE_MAX_POINT_COUNT;
	}

	if (extra_len > 32) {
	    GTP_ERROR("Gesture contain too many extra data!(%d)", extra_len);
	    extra_len = 32;
    }

    /* get gesture extra info */
    if (extra_len >= 0) {
        u8 ges_data[extra_len + 1];

        /* head 4 + extra data * 4 + chksum 1 */
    	ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE + 4,
            ges_data, extra_len + 1);
    	if (ret < 0) {
    		GTP_ERROR("Read extra gesture data failed.");
    		return 0;
    	}

        if (likely(need_chk)) { /* calc checksum */
            bool val;

            ges_data[extra_len] += doze_buf[0] + doze_buf[1]
                + doze_buf[2] + doze_buf[3];

            val = calc_checksum(ges_data, extra_len + 1, CHKBITS_8);
            if (unlikely(!val)) { /* check failed */
                GTP_ERROR("Gesture checksum error.");
                if (err_flag1) {
                    err_flag1 = 0;
                    ret = 0;
                    goto clear_reg;
                } else {
                    /* just return 0 without clear reg,
                                    this will receive another int, we
                                    check the data in the next frame */
                    err_flag1 = 1;
                    return 0;
                }
            }
            err_flag1 = 0;
        }

        mutex_lock(&gesture_data_mutex);
        memcpy(&gesture_data.data[4 + len * 4], ges_data, extra_len);
        mutex_unlock(&gesture_data_mutex);
    }

    /* check gesture type (if available?) */
	if (ges_type == 0 || !QUERYBIT(gestures_flag, ges_type)) {
		GTP_INFO("Gesture[0x%02X] has been disabled.", doze_buf[0]);
        doze_buf[0] = 0x00;
		gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
        gesture_enter_doze();
		return 0;
	}

    /* get gesture point data */
    if (len > 0) { /* coor num * 4 + chksum 2*/
        u8 ges_data[len * 4 + 2];

        ret = gt1x_i2c_read(GES_BUFFER_ADDR, ges_data, len * 4);
        if (ret < 0) {
            GTP_ERROR("Read gesture data failed.");
            return 0;
        }

        /* checksum reg for gesture point data */
        ret = gt1x_i2c_read(0x819F, &ges_data[len * 4], 2);
        if (ret < 0) {
            GTP_ERROR("Read gesture data failed.");
            return 0;
        }

        if (likely(need_chk)) {
            bool val = calc_checksum(ges_data,
               len * 4 + 2, CHKBITS_16);
            if (unlikely(!val)) { /* check failed */
                GTP_ERROR("Gesture checksum error.");
                if (err_flag2) {
                    err_flag2 = 0;
                    ret = 0;
                    goto clear_reg;
                } else {
                    err_flag2 = 1;
                    return 0;
                }
            }

            err_flag2 = 0;
        }

        mutex_lock(&gesture_data_mutex);
        memcpy(&gesture_data.data[4], ges_data, len * 4);
        mutex_unlock(&gesture_data_mutex);
    }

    mutex_lock(&gesture_data_mutex);
	gesture_data.data[0] = ges_type;	// gesture type
	gesture_data.data[1] = len;	        // gesture points number
	gesture_data.data[2] = doze_buf[2] & 0x7F; // protocol type
	gesture_data.data[3] = extra_len;   // gesture date length
	mutex_unlock(&gesture_data_mutex);

    /* get key code */
    key_code = ges_type < 16? KEY_GES_CUSTOM : KEY_GES_REGULAR;
	GTP_DEBUG("Gesture: 0x%02X, points: %d", doze_buf[0], doze_buf[1]);

	input_report_key(dev, key_code, 1);
	input_sync(dev);
	input_report_key(dev, key_code, 0);
	input_sync(dev);

clear_reg:
	doze_buf[0] = 0; // clear ges flag
	gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
	return ret;
}

void gesture_clear_wakeup_data(void)
{
	mutex_lock(&gesture_data_mutex);
	memset(gesture_data.data, 0, 4);
	mutex_unlock(&gesture_data_mutex);
}

void gt1x_gesture_debug(int on)
{
    if (on) {
        gesture_enabled = 1;
        memset(gestures_flag, 0xFF, sizeof(gestures_flag));
    } else {
        gesture_enabled = 0;
        memset(gestures_flag, 0x00, sizeof(gestures_flag));
        gesture_doze_status = DOZE_DISABLED;
    }
    GTP_DEBUG("Gesture debug %s", on ? "on":"off");
}

#endif // GTP_GESTURE_WAKEUP


#ifdef CONFIG_GTP_TOUCH_DETECT
#define  GTP_RAWDATA_COOR_ADDR 0xB53C
#define drv_num 16
#define sens_num  33
#define  touch_level 80
 u8 gtp_background_reset_flag = 1;
 u8 gtp_background_double_check = 1;
 u8 gtp_background_double_flag = 0;
 s16 raw_data_background[drv_num][sens_num];
 u8  raw_data_buf[drv_num*sens_num * 2 ] = {0};//{0x8B98>>8,0x8B98&0xFF};//BC40
 s16 raw_data[drv_num][sens_num]={{0},{0}};

s32 touch_rowdata_event_handler(struct input_dev * dev) {

    //s16 raw_data_background[drv_num][sens_num];
    u8 point_data[4] = { 0 };
    u8  temp,k,j;
    u16 touch_up=0;
    u16 touch_down=0;
    s32 ret = 0;

    ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, 1);
    if (ret < 0){
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        return -1;
    }

    GTP_DEBUG("_owen_ point_data:%x", point_data[0]);

    if((point_data[0]&0x80) == 0x80)
    {
        ret = gt1x_i2c_read(GTP_RAWDATA_COOR_ADDR, raw_data_buf, (sens_num*drv_num * 2));
        if (ret < 0) {
            GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
	     return -1;
        }

        for(k = 0;k < drv_num; k++) {
        	for(j = 0;j < sens_num; j++){
        		raw_data[k][j] = ((raw_data_buf[(drv_num*sens_num * 2 + 1) - (k * sens_num * 2 + j * 2)]) | (raw_data_buf[(drv_num*sens_num * 2 + 1) - (k * sens_num * 2 + j * 2 + 1)] << 8));
        		if(gtp_background_reset_flag == 1){
                                gtp_background_reset_flag = 0;
        			raw_data_background[k][j] = raw_data[k][j];
                        }
        		raw_data[k][j] = raw_data_background[k][j] - raw_data[k][j];
        		if( raw_data[k][j]  > touch_level ) {
                           if(j < 18)
                                touch_up++;
                           else
                                touch_down++;
                       }
        	}
        }

        GTP_DEBUG("data_diff___%x,%x, %x, %x, %x", raw_data[0][0], raw_data[0][1], raw_data[0][2], raw_data[0][3],raw_data[0][4]);
        GTP_DEBUG("_owen_ point_up,down:%d, %d", touch_up,touch_up);

        if(touch_up > 3 || touch_down > 3){
    		temp = 3;
        }else{
                temp = 20;
        }

       if(gtp_background_double_flag == 1) {
            gtp_background_double_flag = 0;
            if(touch_up > 0 || touch_down > 0)
                temp = 100;
        }

    	GTP_INFO("temp%d", temp);
    	for(k = 0;k < drv_num; k++)
    	{
    		for(j = 0;j < sens_num; j++)
    		{
    			raw_data_background[k][j] -= raw_data[k][j]*temp/100;
    		}
    	}

    }
    return 0;
}
#endif

//HotKnot module
#ifdef CONFIG_GTP_HOTKNOT
#include <linux/firmware.h>
#define HOTKNOT_NODE "hotknot"
#define HOTKNOT_VERSION  "GOODIX,GT1X"
#define HN_AUTH_HEADER	14
#define GT1X_HN_JUMP_FW	"gt1x_hotknot_jump_fw.bin"
#define GT1X_HN_AUTH_FW	"gt1x_hotknot_auth_fw.bin"
#define GT2X_HN_AUTH_FW	"gt2x_hotknot_auth_fw.bin"

u8 hotknot_enabled = 0;
u8 hotknot_transfer_mode = 0;

static int hotknot_open(struct inode *node, struct file *flip)
{
	GTP_DEBUG("Hotknot is enabled.");
	hotknot_enabled = 1;
	return 0;
}

static int hotknot_release(struct inode *node, struct file *filp)
{
	GTP_DEBUG("Hotknot is disabled.");
	hotknot_enabled = 0;
	return 0;
}

static s32 hotknot_enter_transfer_mode(void)
{
	int ret = 0;
	u8 buffer[5] = { 0 };

	hotknot_transfer_mode = 1;
#ifdef CONFIG_GTP_ESD_PROTECT
	gt1x_esd_switch(SWITCH_OFF);
#endif

	gt1x_irq_disable();
	gt1x_send_cmd(GTP_CMD_HN_TRANSFER, 0);
	msleep(100);
	gt1x_irq_enable();

	ret = gt1x_i2c_read(0x8140, buffer, sizeof(buffer));
	if (ret) {
		hotknot_transfer_mode = 0;
		return ret;
	}

	buffer[4] = 0;
	GTP_DEBUG("enter transfer mode: %s ", buffer);
	if (strcmp(buffer, "GHot")) {
		hotknot_transfer_mode = 0;
		return ERROR_HN_VER;
	}

	return 0;
}

static s32 hotknot_load_hotknot_subsystem(void)
{
	return hotknot_enter_transfer_mode();
}

static int gt1x_load_auth_subsystem(void)
{
	const struct firmware *jump_fw, *auth_fw;
	int ret;

	GTP_INFO("Request %s", GT1X_HN_JUMP_FW);
	ret = request_firmware(&jump_fw, GT1X_HN_JUMP_FW,
			&gt1x_i2c_client->dev);
	if (ret < 0) {
		GTP_ERROR("Failed to get %s", GT1X_HN_JUMP_FW);
		return ret;
	} else if (jump_fw->size != SZ_4K) {
		ret = -EINVAL;
		GTP_ERROR("Firmware size not match");
		goto out1;
	}

	GTP_INFO("Request %s", GT1X_HN_AUTH_FW);
	ret = request_firmware(&auth_fw, GT1X_HN_AUTH_FW,
			&gt1x_i2c_client->dev);
	if (ret < 0) {
		GTP_ERROR("Failed to get %s", GT1X_HN_AUTH_FW);
		goto out1;
	} else if (auth_fw->size != SZ_4K + HN_AUTH_HEADER) {
		ret = -EINVAL;
		GTP_ERROR("Firmware size not match");
		goto out0;
	}

	GTP_INFO("hotknot load jump code.");
	ret = gt1x_load_patch((u8 *)jump_fw->data, SZ_4K, 0, SZ_8K);
	if (ret < 0) {
		GTP_ERROR("Load jump code fail!");
		goto out0;
	}

	GTP_INFO("hotknot load auth code.");
	ret = gt1x_load_patch((u8 *)&auth_fw->data[HN_AUTH_HEADER],
			SZ_4K, 4096, SZ_8K);
	if (ret < 0) {
		GTP_ERROR("Load auth system fail!");
		goto out0;
	}

out0:
	release_firmware(auth_fw);
out1:
	release_firmware(jump_fw);
	return ret;
}

static int gt2x_load_auth_subsystem(void)
{
	const struct firmware *auth_fw;
	int ret;

	GTP_INFO("Request %s", GT2X_HN_AUTH_FW);
	ret = request_firmware(&auth_fw, GT2X_HN_AUTH_FW,
			&gt1x_i2c_client->dev);
	if (ret < 0) {
		GTP_ERROR("Failed to get %s", GT2X_HN_AUTH_FW);
		return ret;
	} else if (auth_fw->size != SZ_4K + HN_AUTH_HEADER) {
		ret = -EINVAL;
		GTP_ERROR("Firmware size not match");
		goto out0;
	}

	GTP_INFO("hotknot load auth code.");
	ret = gt1x_load_patch((u8 *)&auth_fw->data[HN_AUTH_HEADER],
			SZ_4K, 0, SZ_1K * 6);
	if (ret < 0)
		GTP_ERROR("Load auth system fail!");

out0:
	release_firmware(auth_fw);
	return ret;
}

static s32 hotknot_load_authentication_subsystem(void)
{
	s32 ret = 0;
	u8 buffer[5] = { 0 };

	ret = gt1x_hold_ss51_dsp_no_reset();
	if (ret < 0) {
		GTP_ERROR("Hold ss51 fail!");
		return ERROR;
	}

	if (gt1x_chip_type == CHIP_TYPE_GT1X)
		ret = gt1x_load_auth_subsystem();
	else /* GT2X */
		ret = gt2x_load_auth_subsystem();

	if (ret < 0)
		return ret;

	ret = gt1x_startup_patch();
	if (ret < 0) {
		GTP_ERROR("Startup auth system fail!");
		return ret;
	}

	ret = gt1x_i2c_read(GTP_REG_VERSION, buffer, 4);
	if (ret < 0) {
		GTP_ERROR("i2c read error!");
		return ERROR_IIC;
	}
	buffer[4] = 0;
	GTP_INFO("Current System version: %s", buffer);
	return 0;
}

static s32 hotknot_recovery_main_system(void)
{
	gt1x_irq_disable();
	gt1x_reset_guitar();
	gt1x_irq_enable();
#ifdef CONFIG_GTP_ESD_PROTECT
	gt1x_esd_switch(SWITCH_ON);
#endif
	hotknot_transfer_mode = 0;
	return 0;
}

#ifdef CONFIG_HOTKNOT_BLOCK_RW
DECLARE_WAIT_QUEUE_HEAD(bp_waiter);
static u8 got_hotknot_state = 0;
static u8 got_hotknot_extra_state = 0;
static u8 wait_hotknot_state = 0;
static u8 force_wake_flag = 0;
static u8 block_enable = 0;
s32 hotknot_paired_flag = 0;

static s32 hotknot_block_rw(u8 rqst_hotknot_state, s32 wait_hotknot_timeout)
{
	s32 ret = 0;

	wait_hotknot_state |= rqst_hotknot_state;
	GTP_DEBUG("Goodix tool received wait polling state:0x%x,timeout:%d, all wait state:0x%x", rqst_hotknot_state, wait_hotknot_timeout, wait_hotknot_state);
	got_hotknot_state &= (~rqst_hotknot_state);

	set_current_state(TASK_INTERRUPTIBLE);
	if (wait_hotknot_timeout <= 0) {
		wait_event_interruptible(bp_waiter, force_wake_flag || rqst_hotknot_state == (got_hotknot_state & rqst_hotknot_state));
	} else {
		wait_event_interruptible_timeout(bp_waiter, force_wake_flag || rqst_hotknot_state == (got_hotknot_state & rqst_hotknot_state), wait_hotknot_timeout);
	}

	wait_hotknot_state &= (~rqst_hotknot_state);

	if (rqst_hotknot_state != (got_hotknot_state & rqst_hotknot_state)) {
		GTP_ERROR("Wait 0x%x block polling waiter failed.", rqst_hotknot_state);
		ret = -1;
	}
 if(force_wake_flag) {
     got_hotknot_extra_state = 0x07;  // force wakeup report as departed
   }
	force_wake_flag = 0;
	return ret;
}

void hotknot_wakeup_block(void)
{
	GTP_DEBUG("Manual wakeup all block polling waiter!");
	got_hotknot_state = 0;
	wait_hotknot_state = 0;
	force_wake_flag = 1;
	wake_up_interruptible(&bp_waiter);
}

s32 hotknot_event_handler(u8 * data)
{
	u8 hn_pxy_state = 0;
	u8 hn_pxy_state_bak = 0;
	static u8 hn_paired_cnt = 0;
	u8 hn_state_buf[10] = { 0 };
	u8 finger = data[0];
	u8 id = 0;

	if (block_enable && !hotknot_paired_flag && (finger & 0x0F)) {
		id = data[1];
		hn_pxy_state = data[2] & 0x80;
		hn_pxy_state_bak = data[3] & 0x80;
		if ((32 == id) && (0x80 == hn_pxy_state) && (0x80 == hn_pxy_state_bak)) {
#ifdef HN_DBLCFM_PAIRED
			if (hn_paired_cnt++ < 2) {
				return 0;
			}
#endif
			GTP_DEBUG("HotKnot paired!");
			if (wait_hotknot_state & HN_DEVICE_PAIRED) {
				GTP_DEBUG("INT wakeup HN_DEVICE_PAIRED block polling waiter");
				got_hotknot_state |= HN_DEVICE_PAIRED;
				wake_up_interruptible(&bp_waiter);
			}
			block_enable = 0;
			hotknot_paired_flag = 1;
			return 0;
		} else {
			got_hotknot_state &= (~HN_DEVICE_PAIRED);
			hn_paired_cnt = 0;
		}
	}

	if (hotknot_paired_flag) {
		s32 ret = -1;
		ret = gt1x_i2c_read(GTP_REG_HN_STATE, hn_state_buf, 6);
		if (ret < 0) {
			GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
			return 0;
		}

		got_hotknot_state = 0;

		GTP_DEBUG("wait_hotknot_state:%x", wait_hotknot_state);
		GTP_DEBUG("[0x8800~0x8803]=0x%x,0x%x,0x%x,0x%x", hn_state_buf[0], hn_state_buf[1], hn_state_buf[2], hn_state_buf[3]);

		if (wait_hotknot_state & HN_MASTER_SEND) {
			if ((0x03 == hn_state_buf[0]) || (0x04 == hn_state_buf[0])
			    || (0x07 == hn_state_buf[0])) {
				GTP_DEBUG("Wakeup HN_MASTER_SEND block polling waiter");
				got_hotknot_state |= HN_MASTER_SEND;
				got_hotknot_extra_state = hn_state_buf[0];
				wake_up_interruptible(&bp_waiter);
			}
		} else if (wait_hotknot_state & HN_SLAVE_RECEIVED) {
			if ((0x03 == hn_state_buf[1]) || (0x04 == hn_state_buf[1])
			    || (0x07 == hn_state_buf[1])) {
				GTP_DEBUG("Wakeup HN_SLAVE_RECEIVED block polling waiter:0x%x", hn_state_buf[1]);
				got_hotknot_state |= HN_SLAVE_RECEIVED;
				got_hotknot_extra_state = hn_state_buf[1];
				wake_up_interruptible(&bp_waiter);
			}
		} else if (wait_hotknot_state & HN_MASTER_DEPARTED) {
			if (0x07 == hn_state_buf[0]) {
				GTP_DEBUG("Wakeup HN_MASTER_DEPARTED block polling waiter");
				got_hotknot_state |= HN_MASTER_DEPARTED;
				wake_up_interruptible(&bp_waiter);
			}
		} else if (wait_hotknot_state & HN_SLAVE_DEPARTED) {
			if (0x07 == hn_state_buf[1]) {
				GTP_DEBUG("Wakeup HN_SLAVE_DEPARTED block polling waiter");
				got_hotknot_state |= HN_SLAVE_DEPARTED;
				wake_up_interruptible(&bp_waiter);
			}
		}
		return 0;
	}

	return -1;
}
#endif //HOTKNOT_BLOCK_RW
#endif //GTP_HOTKNOT

#define GOODIX_MAGIC_NUMBER        'G'
#define NEGLECT_SIZE_MASK           (~(_IOC_SIZEMASK << _IOC_SIZESHIFT))

#define GESTURE_ENABLE              _IO(GOODIX_MAGIC_NUMBER, 1)	// 1
#define GESTURE_DISABLE             _IO(GOODIX_MAGIC_NUMBER, 2)
#define GESTURE_FLAG_SET            _IO(GOODIX_MAGIC_NUMBER, 3)
#define GESTURE_FLAG_CLEAR          _IO(GOODIX_MAGIC_NUMBER, 4)
//#define SET_ENABLED_GESTURE         (_IOW(GOODIX_MAGIC_NUMBER, 5, u8) & NEGLECT_SIZE_MASK)
#define GESTURE_DATA_OBTAIN         (_IOR(GOODIX_MAGIC_NUMBER, 6, u8) & NEGLECT_SIZE_MASK)
#define GESTURE_DATA_ERASE          _IO(GOODIX_MAGIC_NUMBER, 7)

//#define HOTKNOT_LOAD_SUBSYSTEM      (_IOW(GOODIX_MAGIC_NUMBER, 6, u8) & NEGLECT_SIZE_MASK)
#define HOTKNOT_LOAD_HOTKNOT        _IO(GOODIX_MAGIC_NUMBER, 20)
#define HOTKNOT_LOAD_AUTHENTICATION _IO(GOODIX_MAGIC_NUMBER, 21)
#define HOTKNOT_RECOVERY_MAIN       _IO(GOODIX_MAGIC_NUMBER, 22)
//#define HOTKNOT_BLOCK_RW            (_IOW(GOODIX_MAGIC_NUMBER, 6, u8) & NEGLECT_SIZE_MASK)
#define HOTKNOT_DEVICES_PAIRED      _IO(GOODIX_MAGIC_NUMBER, 23)
#define HOTKNOT_MASTER_SEND         _IO(GOODIX_MAGIC_NUMBER, 24)
#define HOTKNOT_SLAVE_RECEIVE       _IO(GOODIX_MAGIC_NUMBER, 25)
//#define HOTKNOT_DEVICES_COMMUNICATION
#define HOTKNOT_MASTER_DEPARTED     _IO(GOODIX_MAGIC_NUMBER, 26)
#define HOTKNOT_SLAVE_DEPARTED      _IO(GOODIX_MAGIC_NUMBER, 27)
#define HOTKNOT_VENDOR_VERSION      (_IOR(GOODIX_MAGIC_NUMBER, 28, u8) & NEGLECT_SIZE_MASK)
#define HOTKNOT_WAKEUP_BLOCK        _IO(GOODIX_MAGIC_NUMBER, 29)

#define IO_IIC_READ                  (_IOR(GOODIX_MAGIC_NUMBER, 100, u8) & NEGLECT_SIZE_MASK)
#define IO_IIC_WRITE                 (_IOW(GOODIX_MAGIC_NUMBER, 101, u8) & NEGLECT_SIZE_MASK)
#define IO_RESET_GUITAR              _IO(GOODIX_MAGIC_NUMBER, 102)
#define IO_DISABLE_IRQ               _IO(GOODIX_MAGIC_NUMBER, 103)
#define IO_ENABLE_IRQ                _IO(GOODIX_MAGIC_NUMBER, 104)
#define IO_GET_VERISON               (_IOR(GOODIX_MAGIC_NUMBER, 110, u8) & NEGLECT_SIZE_MASK)
#define IO_PRINT                     (_IOW(GOODIX_MAGIC_NUMBER, 111, u8) & NEGLECT_SIZE_MASK)
#define IO_VERSION                   "V1.3-20150420"

#define CMD_HEAD_LENGTH             20
static s32 io_iic_read(u8 * data, void __user * arg)
{
	s32 err = ERROR;
	s32 data_length = 0;
	u16 addr = 0;

	err = copy_from_user(data, arg, CMD_HEAD_LENGTH);
	if (err) {
		GTP_ERROR("Can't access the memory.");
		return ERROR_MEM;
	}

	addr = data[0] << 8 | data[1];
	data_length = data[2] << 8 | data[3];

	err = gt1x_i2c_read(addr, &data[CMD_HEAD_LENGTH], data_length);
	if (!err) {
		err = copy_to_user(&((u8 __user *) arg)[CMD_HEAD_LENGTH], &data[CMD_HEAD_LENGTH], data_length);
		if (err) {
			GTP_ERROR("ERROR when copy to user.[addr: %04x], [read length:%d]", addr, data_length);
			return ERROR_MEM;
		}
		err = CMD_HEAD_LENGTH + data_length;
	}
	//GTP_DEBUG("IIC_READ.addr:0x%4x, length:%d, ret:%d", addr, data_length, err);
	//GTP_DEBUG_ARRAY((&data[CMD_HEAD_LENGTH]), data_length);

	return err;
}

static s32 io_iic_write(u8 * data)
{
	s32 err = ERROR;
	s32 data_length = 0;
	u16 addr = 0;

	addr = data[0] << 8 | data[1];
	data_length = data[2] << 8 | data[3];

	err = gt1x_i2c_write(addr, &data[CMD_HEAD_LENGTH], data_length);
	if (!err) {
		err = CMD_HEAD_LENGTH + data_length;
	}

	//GTP_DEBUG("IIC_WRITE.addr:0x%4x, length:%d, ret:%d", addr, data_length, err);
	//GTP_DEBUG_ARRAY((&data[CMD_HEAD_LENGTH]), data_length);
	return err;
}

//@return, 0:operate successfully
//         > 0: the length of memory size ioctl has accessed,
//         error otherwise.
static long gt1x_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 value = 0;
	s32 ret = 0;		//the initial value must be 0
	u8 *data = NULL;
    int cnt = 30;

    /* Blocking when firmwaer updating */
    while (cnt-- && update_info.status) {
        ssleep(1);
    }

	//GTP_DEBUG("IOCTL CMD:%x", cmd);
	/* GTP_DEBUG("command:%d, length:%d, rw:%s",
	        _IOC_NR(cmd),
	        _IOC_SIZE(cmd),
	        (_IOC_DIR(cmd) & _IOC_READ) ? "read" : (_IOC_DIR(cmd) & _IOC_WRITE) ? "write" : "-");
        */

	if (_IOC_DIR(cmd)) {
		s32 err = -1;
		s32 data_length = _IOC_SIZE(cmd);
		data = (u8 *) kzalloc(data_length, GFP_KERNEL);
		memset(data, 0, data_length);

		if (_IOC_DIR(cmd) & _IOC_WRITE) {
			err = copy_from_user(data, (void __user *)arg, data_length);
			if (err) {
				GTP_ERROR("Can't access the memory.");
				kfree(data);
				return -1;
			}
		}
	} else {
		value = (u32) arg;
	}

	switch (cmd & NEGLECT_SIZE_MASK) {
	case IO_GET_VERISON:
		if ((u8 __user *) arg) {
			ret = copy_to_user(((u8 __user *) arg), IO_VERSION, sizeof(IO_VERSION));
			if (!ret) {
				ret = sizeof(IO_VERSION);
			}
			GTP_INFO("%s", IO_VERSION);
		}
		break;
	case IO_IIC_READ:
		ret = io_iic_read(data, (void __user *)arg);
		break;

	case IO_IIC_WRITE:
		ret = io_iic_write(data);
		break;

	case IO_RESET_GUITAR:
        gt1x_irq_disable();
		gt1x_reset_guitar();
        gt1x_irq_enable();
		break;

	case IO_DISABLE_IRQ:
		gt1x_irq_disable();
#ifdef CONFIG_GTP_ESD_PROTECT
		gt1x_esd_switch(SWITCH_OFF);
#endif
		break;

	case IO_ENABLE_IRQ:
		gt1x_irq_enable();
#ifdef CONFIG_GTP_ESD_PROTECT
		gt1x_esd_switch(SWITCH_ON);
#endif
		break;

		//print a string to syc log messages between application and kernel.
	case IO_PRINT:
		if (data)
			GTP_INFO("%s", (char *)data);
		break;

#ifdef CONFIG_GTP_GESTURE_WAKEUP
	case GESTURE_ENABLE:
		GTP_DEBUG("Gesture switch ON.");
		gesture_enabled = 1;
		break;

	case GESTURE_DISABLE:
		GTP_DEBUG("Gesture switch OFF.");
		gesture_enabled = 0;
		break;

	case GESTURE_FLAG_SET:
		SETBIT(gestures_flag, (u8) value);
		GTP_DEBUG("Gesture flag: 0x%02X enabled.", value);
		break;

	case GESTURE_FLAG_CLEAR:
		CLEARBIT(gestures_flag, (u8) value);
        GTP_DEBUG("Gesture flag: 0x%02X disabled.", value);
		break;

	case GESTURE_DATA_OBTAIN:
		GTP_DEBUG("Obtain gesture data.");
		mutex_lock(&gesture_data_mutex);
		ret = copy_to_user(((u8 __user *) arg), &gesture_data.data, 4 + gesture_data.data[1] * 4 + gesture_data.data[3]);
		if (ret) {
			GTP_ERROR("ERROR when copy gesture data to user.");
			ret = ERROR_MEM;
		} else {
			ret = 4 + gesture_data.data[1] * 4 + gesture_data.data[3];
		}
        mutex_unlock(&gesture_data_mutex);
		break;

	case GESTURE_DATA_ERASE:
		GTP_DEBUG("ERASE_GESTURE_DATA");
		gesture_clear_wakeup_data();
		break;
#endif // GTP_GESTURE_WAKEUP

#ifdef CONFIG_GTP_HOTKNOT
    case HOTKNOT_VENDOR_VERSION:
        ret =  copy_to_user(((u8 __user *) arg), HOTKNOT_VERSION, sizeof(HOTKNOT_VERSION));
        if (!ret) {
            ret = sizeof(HOTKNOT_VERSION);
        }
        break;
	case HOTKNOT_LOAD_HOTKNOT:
		ret = hotknot_load_hotknot_subsystem();
		break;

	case HOTKNOT_LOAD_AUTHENTICATION:
#ifdef CONFIG_GTP_ESD_PROTECT
		gt1x_esd_switch(SWITCH_OFF);
#endif
		ret = hotknot_load_authentication_subsystem();
		break;

	case HOTKNOT_RECOVERY_MAIN:
		ret = hotknot_recovery_main_system();
		break;
#ifdef CONFIG_HOTKNOT_BLOCK_RW
	case HOTKNOT_DEVICES_PAIRED:
		hotknot_paired_flag = 0;
		force_wake_flag = 0;
		block_enable = 1;
		ret = hotknot_block_rw(HN_DEVICE_PAIRED, (s32) value);
		break;

	case HOTKNOT_MASTER_SEND:
		ret = hotknot_block_rw(HN_MASTER_SEND, (s32) value);
		if (!ret)
			ret = got_hotknot_extra_state;
		break;

	case HOTKNOT_SLAVE_RECEIVE:
		ret = hotknot_block_rw(HN_SLAVE_RECEIVED, (s32) value);
		if (!ret)
			ret = got_hotknot_extra_state;
		break;

	case HOTKNOT_MASTER_DEPARTED:
		ret = hotknot_block_rw(HN_MASTER_DEPARTED, (s32) value);
		break;

	case HOTKNOT_SLAVE_DEPARTED:
		ret = hotknot_block_rw(HN_SLAVE_DEPARTED, (s32) value);
		break;

	case HOTKNOT_WAKEUP_BLOCK:
		hotknot_wakeup_block();
		break;
#endif //HOTKNOT_BLOCK_RW
#endif //GTP_HOTKNOT

	default:
		GTP_INFO("Unknown cmd.");
		ret = -1;
		break;
	}

	if (data != NULL) {
		kfree(data);
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gt1x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *arg32 = compat_ptr(arg);

    if (!file->f_op || !file->f_op->unlocked_ioctl)
        return -ENOTTY;

    return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
}
#endif

static const struct file_operations gt1x_fops = {
	.owner = THIS_MODULE,
#ifdef CONFIG_GTP_GESTURE_WAKEUP
	.read = gt1x_gesture_data_read,
	.write = gt1x_gesture_data_write,
#endif
	.unlocked_ioctl = gt1x_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = gt1x_compat_ioctl,
#endif
};

#ifdef CONFIG_GTP_HOTKNOT
static const struct file_operations hotknot_fops = {
	.open = hotknot_open,
	.release = hotknot_release,
	.unlocked_ioctl = gt1x_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = gt1x_compat_ioctl,
#endif
};

static struct miscdevice hotknot_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = HOTKNOT_NODE,
	.fops = &hotknot_fops,
};
#endif

#ifdef CONFIG_SUPPORT_EARSENSE
#ifdef PAGESIZE
#undef PAGESIZE
#endif
#define PAGESIZE 512

static u16 data_debug[50];
static int flag = 0;

static ssize_t num_x_chans_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4];

	ret = sprintf(page, "%02X\n", 16);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t num_y_chans_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4];

	ret = sprintf(page, "%02X\n", 17);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t es_touch_count_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];

    ret = sprintf(page, "%02X\n", es_touch_count);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t mutual_delta_data_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	u8 mutual_delta[544];
	u8 buffer[1];
	u8 buffer2[10];
	u8 buffer3[1] = {0x00};
	int16_t data;
	u8 cor_num;
	int16_t *ptr;
	int16_t *ptr_p= NULL;
	int x;

	if (*ppos) {
		return 0;
	}

	ptr = kzalloc(4096, GFP_KERNEL);
	if (ptr == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	ptr_p = ptr;

	ret = gt1x_i2c_read(0x804D, buffer, 1);
	if (ret < 0)
		return ret;

    if (buffer[0] == 1) {
        buffer[0] = 0x2;
	    ret = gt1x_i2c_write(0x804D, buffer, 1);
	    if (ret < 0) {
		    GTP_ERROR("I2C write end_cmd  error!");
		    return ret;
	    }
	}

	gt1x_i2c_read(0xA090, mutual_delta, 544);
	for (x = 0; x < 544; x++) {
	    //if (x%32 == 0)
	    //    ptr_p += sprintf(ptr_p, "\n");
        data = (mutual_delta[x] << 8) | mutual_delta[x+1];
	    //ptr_p += sprintf(ptr_p, "%4d", data);
	    *ptr_p++ = data;
	    x++;
	}
	//ptr_p += sprintf(ptr_p, "\n");

    ret = gt1x_i2c_read(0x804B, &cor_num, 1);
    GTP_ERROR("cor_num = %d!\n", cor_num);

    while(cor_num--) {
		ret = gt1x_i2c_read(0xa31a, buffer2, 7);
		if (ret < 0) {
		    GTP_ERROR("I2C read end_cmd error!");
		    return ret;
	    }
	}

	ret = gt1x_i2c_write(0x804D, buffer3, 1);
	if (ret < 0) {
		GTP_ERROR("I2C write end_cmd error!");
		return ret;
	}
	ret = gt1x_i2c_write(0x804B, buffer3, 1);
	if (ret < 0) {
		GTP_ERROR("I2C write end_cmd error!");
		return ret;
	}

    ret = simple_read_from_buffer(user_buf, count, ppos, ptr, (ptr_p-ptr)*2);

	return ret;
}

static ssize_t mutual_raw_data_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	u8 mutual_raw[544];
	int timerout_count = 0;
	u8 buffer[1] = {0x01};
	u16 data;
	int16_t *ptr;
	int16_t *ptr_p= NULL;
	int x;

	ear_flag = 1;
	if (*ppos) {
	    ret = 0;
		goto exit;
	}

	ptr = kzalloc(4096, GFP_KERNEL);
	if (ptr == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	ptr_p = ptr;

	ret = gt1x_i2c_write(0x804C, buffer, 1);
	if (ret < 0)
	{
		GTP_ERROR("I2C write end_cmd  error!");
		goto exit;
	}

	while(1) {
		gt1x_i2c_read(0x804C, buffer, 1);
		if (ret < 0)
		    goto exit;

        GTP_ERROR("I2C write 742  error!");
		if (0x02 == buffer[0])
			break;

		if (0x03 == buffer[0]) {
		    GTP_ERROR("I2C write 747  error!");
		    ret = 5;
			goto exit;
		}

		if (timerout_count > 100) {
		    GTP_ERROR("I2C write 752  error!");
		    ret = -91;
			goto exit;
		}
		GTP_ERROR("gt1x timerout_count = %d\n", timerout_count);
		msleep(1);
		timerout_count ++;
	}

	gt1x_i2c_read(0x9DD4, mutual_raw, 544);

	for (x = 0; x < 544; x++) {
		//if (x%32 == 0)
	    //    ptr_p += sprintf(ptr_p, "\n");
        data = (mutual_raw[x] << 8) | mutual_raw[x+1];
	    //ptr_p += sprintf(ptr_p, "%4d ", data);
	    *ptr_p++ = data;
	    x++;
	}
	//ptr_p += sprintf(ptr_p, "\n");

	ret = simple_read_from_buffer(user_buf, count, ppos, ptr, (ptr_p-ptr)*2);

exit:
    ear_flag = 0;
	return ret;
}

static ssize_t self_raw_tx_data_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	u8 self_tx_raw[32];
	u8 buffer[1] = {0x01};
	int timerout_count = 0;
	u16 data[50];
	u16 data_tx_self;
	int16_t *ptr;
	int16_t *ptr_p= NULL;
	int x;

    ear_flag = 1;
	if (*ppos) {
		goto exit;
	}

	ptr = kzalloc(4096, GFP_KERNEL);
	if (ptr == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	ptr_p = ptr;

    ret = gt1x_i2c_write(0x804C, buffer, 1);
	if (ret < 0) {
		GTP_ERROR("I2C write end_cmd  error!");
		goto exit;
	}

	while(1) {
		gt1x_i2c_read(0x804C, buffer, 1);
		if (ret < 0)
		    goto exit;

        GTP_ERROR("I2C write error!");
		if (0x02 == buffer[0])
			break;

		if (0x03 == buffer[0]) {
		    GTP_ERROR("I2C write error!");
			goto exit;
		}

		if (timerout_count > 100) {
		    GTP_ERROR("I2C write error!");
		    ret = -91;
			goto exit;
		}
		GTP_ERROR("gt1x timerout_count = %d\n", timerout_count);
		msleep(1);
		timerout_count ++;
	}

	gt1x_i2c_read(0xA018, self_tx_raw, 32);
	flag++;
	for (x = 0; x < 32; x++) {
		//if (x%32 == 0)
	    //    ptr_p += sprintf(ptr_p, "\n");
        data_tx_self = (self_tx_raw[x] << 8) | self_tx_raw[x+1];
        *ptr_p++ = data_tx_self;
        x++;
        if (flag == 1)
            data_debug[x/2] = data_tx_self;
        data[x/2] = data_tx_self;
	    //ptr_p += sprintf(ptr_p, "%5d ", data_self);
	}
	//ptr_p += sprintf(ptr_p, "\n");

	/*for (i = 0; i < 50; i++) {
        ptr_p += sprintf(ptr_p, "%5d ", data_debug[i]-data[i]);
	}*/

    ret = simple_read_from_buffer(user_buf, count, ppos, ptr, (ptr_p-ptr)*2);

exit:
    ear_flag = 0;
	return ret;
}

static ssize_t self_raw_rx_data_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	u8 self_rx_raw[66];
	u8 buffer[1] = {0x01};
	int timerout_count = 0;
	u16 data[50];
	u16 data_rx_self;
	int16_t *ptr;
	int16_t *ptr_p= NULL;
	int x;

	ear_flag = 1;
	if (*ppos) {
		goto exit;
	}

	ptr = kzalloc(4096, GFP_KERNEL);
	if (ptr == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	ptr_p = ptr;

	ret = gt1x_i2c_write(0x804C, buffer, 1);
	if (ret < 0) {
		GTP_ERROR("I2C write end_cmd  error!");
		goto exit;
	}

	while(1) {
		gt1x_i2c_read(0x804C, buffer, 1);
		if (ret < 0)
		    goto exit;

		GTP_ERROR("I2C write error!");
		if (0x02 == buffer[0])
			break;

		if (0x03 == buffer[0]) {
		    GTP_ERROR("I2C write error!");
			goto exit;
		}

		if (timerout_count > 100) {
		    GTP_ERROR("I2C write error!");
		    ret = -91;
			goto exit;
		}
		GTP_ERROR("gt1x timerout_count = %d\n", timerout_count);
		msleep(1);
		timerout_count ++;
	}

	gt1x_i2c_read(0xA018+32, self_rx_raw, 66);
	flag++;
	for (x = 0; x < 66; x++) {
		//if (x%32 == 0)
	    //    ptr_p += sprintf(ptr_p, "\n");
        data_rx_self = (self_rx_raw[x] << 8) | self_rx_raw[x+1];
        *ptr_p++ = data_rx_self;
        x++;
        if (flag == 1)
            data_debug[x/2] = data_rx_self;
        data[x/2] = data_rx_self;
	    //ptr_p += sprintf(ptr_p, "%5d ", data_self);
	}
	//ptr_p += sprintf(ptr_p, "\n");

	/*for (i = 0; i < 50; i++) {
        ptr_p += sprintf(ptr_p, "%5d ", data_debug[i]-data[i]);
	}*/

    ret = simple_read_from_buffer(user_buf, count, ppos, ptr, (ptr_p-ptr)*2);

exit:
    ear_flag = 0;
	return ret;
}

static ssize_t es_enabled_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];

    printk("bean set es_enable ret = %d\n", ret);
    ret = sprintf(page, "%d\n", es_enabled);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t es_mode_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
    u8 buffer[10];
    char *ptr;
	char *ptr_p= NULL;

	if (*ppos) {
		return 0;
	}

	ptr = kzalloc(4096, GFP_KERNEL);
	if (ptr == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	ptr_p = ptr;

    ret = gt1x_i2c_read(0x8040, &buffer[0], 1);
    ret = gt1x_i2c_read(0x907B, &buffer[1], 1);
    ret = gt1x_i2c_read(0x804B, &buffer[2], 1);
    ret = gt1x_i2c_read(0x804D, &buffer[3], 1);

    printk("bean set es_mode ret = %d\n", ret);
    ptr_p += sprintf(ptr_p, "es set = %02x,es_mode = %02x 804B = %02x 804D = %02x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
	ret = simple_read_from_buffer(user_buf, count, ppos, ptr, ptr_p-ptr);
	return ret;
}

static ssize_t es_enabled_proc_write_func(struct file *file, const char __user *page, size_t count, loff_t *ppos)
{
	int ret = 0;
	char buf[10] = {0};
	u8 buffer[3] = {0x61, 0x00, 0x9F};

	if (copy_from_user(buf, page, count)){
		GTP_ERROR("%s: read proc input error.\n", __func__);
		return count;
	}

	if ('0' == buf[0]) {
		es_enabled = 0;
	} else if('1' == buf[0]) {
		es_enabled = 1;
	}

	if (es_enabled) {
		GTP_DEBUG("GTP EarSense on.");
		buffer[0] = 0x60;
		buffer[1] = 0x00;
		buffer[2] = 0xA0;
	} else {
		GTP_DEBUG("GTP EarSense off.");
		buffer[0] = 0x61;
		buffer[1] = 0x00;
		buffer[2] = 0x9F;
	}

	ret = gt1x_i2c_write(0x8040, buffer, 3);
	if (ret < 0)
		GTP_ERROR("I2C write end_cmd  error!");

	msleep(10);

	return count;
}

static const struct file_operations num_x_chans_proc_fops = {
	.read =  num_x_chans_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations num_y_chans_proc_fops = {
	.read =  num_y_chans_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations es_touch_count_proc_fops = {
	.read =  es_touch_count_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations mutual_delta_data_proc_fops = {
	.read =  mutual_delta_data_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations mutual_raw_data_proc_fops = {
	.read =  mutual_raw_data_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations self_raw_tx_data_proc_fops = {
	.read =  self_raw_tx_data_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations self_raw_rx_data_proc_fops = {
	.read =  self_raw_rx_data_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations es_enabled_proc_fops = {
	.write = es_enabled_proc_write_func,
	.read =  es_enabled_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations es_mode_proc_fops = {
	.read =  es_mode_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

#define GTP_ENABLE_TPEDGE_LIMIT
#ifdef PAGESIZE
#undef PAGESIZE
#endif
#define PAGESIZE 512

static struct proc_dir_entry *prEntry_tp = NULL;
static int limit_enable=1;

#ifdef GTP_ENABLE_TPEDGE_LIMIT
static ssize_t limit_enable_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	ssize_t ret =0;
	char page[PAGESIZE];

	GTP_INFO("the limit_enable is: %d\n", limit_enable);
	ret = sprintf(page, "%d\n", limit_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t limit_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8]={0};

	if (count > 2)
		count = 2;
	if (copy_from_user(buf, buffer, count))
	{
		GTP_ERROR("%s: read proc input error.\n", __func__);
		return count;
	}

	return count;
}

static const struct file_operations proc_limit_enable =
{
	.read = limit_enable_read,
	.write = limit_enable_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif


#ifdef CONFIG_GTP_GESTURE_WAKEUP
static ssize_t tp_gesture_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_gesture_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{

	return count;
}

static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t gesture_switch_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t gesture_switch_write_func(struct file *file, const char __user *page, size_t count, loff_t *ppos)
{
	return count;
}

static const struct file_operations tp_gesture_proc_fops = {
	.write = tp_gesture_write_func,
	.read =  tp_gesture_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations gesture_switch_proc_fops = {
	.write = gesture_switch_write_func,
	.read =  gesture_switch_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations coordinate_proc_fops = {
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

static ssize_t vendor_id_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4];
	ret = sprintf(page, "%d\n",8);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct file_operations vendor_id_proc_fops = {
	.read =  vendor_id_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static int init_gtp_proc(void)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_tmp  = NULL;
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if( prEntry_tp == NULL ){
		ret = -ENOMEM;
		GTP_ERROR("Couldn't create touchpanel\n");
	}

#ifdef CONFIG_GTP_GESTURE_WAKEUP
	prEntry_tmp = proc_create( "gesture_enable", 0666, prEntry_tp,
		&tp_gesture_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create gesture_enable\n");
	}
	prEntry_tmp = proc_create( "gesture_switch", 0666, prEntry_tp,
		&gesture_switch_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		GTP_ERROR("Couldn't create gesture_switch\n");
	}
	prEntry_tmp = proc_create("coordinate", 0444, prEntry_tp,
		&coordinate_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create coordinate\n");
	}
#endif

#ifdef GTP_ENABLE_TPEDGE_LIMIT
	prEntry_tmp = proc_create("tpedge_limit_enable", 0666,
		prEntry_tp, &proc_limit_enable);
	if( prEntry_tmp == NULL ){
		ret = -ENOMEM;
	GTP_ERROR("Couldn't create tp_limit_enable\n");
	}
#endif

	prEntry_tmp = proc_create("vendor_id", 0444, prEntry_tp,
		&vendor_id_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create vendor_id\n");
	}

#ifdef CONFIG_SUPPORT_EARSENSE
	prEntry_tmp = proc_create("num_x_chans", 0440, prEntry_tp,
		&num_x_chans_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create num_x_chans\n");
	}

	prEntry_tmp = proc_create("num_y_chans", 0440, prEntry_tp,
		&num_y_chans_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create num_y_chans\n");
	}

	prEntry_tmp = proc_create("es_touch_count", 0440, prEntry_tp,
		&es_touch_count_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create es_touch_count\n");
	}

	prEntry_tmp = proc_create("mutual_delta_data", 0440, prEntry_tp,
		&mutual_delta_data_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create mutual_delta_data\n");
	}

	prEntry_tmp = proc_create("mutual_raw_data", 0440, prEntry_tp,
		&mutual_raw_data_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create mutual_raw_data\n");
	}

	prEntry_tmp = proc_create("self_raw_tx_data", 0440, prEntry_tp,
		&self_raw_tx_data_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create mutual_raw_data\n");
	}

	prEntry_tmp = proc_create("self_raw_rx_data", 0440, prEntry_tp,
		&self_raw_rx_data_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create mutual_raw_data\n");
	}

	prEntry_tmp = proc_create("es_enabled", 0660, prEntry_tp,
		&es_enabled_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create mutual_raw_data\n");
	}

	prEntry_tmp = proc_create("es_mode", 0440, prEntry_tp,
		&es_mode_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        GTP_ERROR("Couldn't create mutual_raw_data\n");
	}
#endif

	return ret;
}


s32 gt1x_init_node(void)
{
#ifdef CONFIG_GTP_GESTURE_WAKEUP
	struct proc_dir_entry *proc_entry = NULL;
	mutex_init(&gesture_data_mutex);
	memset(gestures_flag, 0, sizeof(gestures_flag));
	memset((u8 *) & gesture_data, 0, sizeof(st_gesture_data));

	proc_entry = proc_create(GESTURE_NODE, 0666, NULL, &gt1x_fops);
	if (proc_entry == NULL) {
		GTP_ERROR("CAN't create proc entry /proc/%s.", GESTURE_NODE);
		return -1;
	} else {
		GTP_INFO("Created proc entry /proc/%s.", GESTURE_NODE);
	}
#endif

#ifdef CONFIG_GTP_HOTKNOT
	if (misc_register(&hotknot_misc_device)) {
		GTP_ERROR("CAN't create misc device in /dev/hotknot.");
		return -1;
	} else {
		GTP_INFO("Created misc device in /dev/hotknot.");
	}
#endif
	init_gtp_proc();
	return 0;
}

void gt1x_deinit_node(void)
{
#ifdef CONFIG_GTP_GESTURE_WAKEUP
	remove_proc_entry(GESTURE_NODE, NULL);
#endif

#ifdef CONFIG_GTP_HOTKNOT
	misc_deregister(&hotknot_misc_device);
#endif
}
