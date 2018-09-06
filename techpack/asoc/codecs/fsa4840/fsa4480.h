#ifndef __FSA4480_INC__
#define __FSA4480_INC__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
/*2018/06/14 @bsp add for support notify audio adapter switch*/
#include <linux/notifier.h>

#define 	FSA4480_DEVID		0x00
#define		FSA4480_OVPMSK		0x01
#define		FSA4480_OVPFLG		0x02
#define		FSA4480_OVPST		0x03
#define		FSA4480_SWEN		0x04
#define		FSA4480_SWSEL		0x05
#define		FSA4480_SWST0		0x06
#define		FSA4480_SWST1		0x07
#define		FSA4480_CNT_L		0x08
#define		FSA4480_CNT_R		0x09
#define		FSA4480_CNT_MIC		0x0A
#define		FSA4480_CNT_SEN		0x0B
#define		FSA4480_CNT_GND		0x0C
#define		FSA4480_TIM_R		0x0D
#define		FSA4480_TIM_MIC		0x0E
#define		FSA4480_TIM_SEN		0x0F
#define		FSA4480_TIM_GND		0x10
#define		FSA4480_ACC_DET		0x11
#define		FSA4480_FUN_EN		0x12
#define		FSA4480_RES_SET		0x13
#define		FSA4480_RES_VAL		0x14
#define		FSA4480_RES_THR		0x15
#define		FSA4480_RES_INV		0x16
#define		FSA4480_JACTYP		0x17
#define		FSA4480_DECINT		0x18
#define		FSA4480_DECMSK		0x19
#define		FSA4480_AUDREG1		0x1A
#define		FSA4480_AUDREG2		0x1B
#define		FSA4480_AUDDATA0	0x1C
#define		FSA4480_AUDDATA1	0x1D
#define		FSA4480_RESET		0x1E
#define		FSA4480_CURSET		0x1F

#define		FSA4480_ALLSW		0
#define		FSA4480_USB			1
#define		FSA4480_Audio		2
#define		FSA4480_MIC			3
#define		FSA4480_SBU			4

struct fsa4480 {
	struct i2c_client *i2c;
	struct mutex fsa_lock;
	wait_queue_head_t wq;
	struct device *dev;
	int mbhc_en;
	struct delayed_work call_wcd_dwork;
};

/*2018/06/14 @bsp add for support notify audio adapter switch*/
extern int register_cc_notifier_client(struct notifier_block *nb);
extern int unregister_cc_notifier_client(struct notifier_block *nb);

#endif
