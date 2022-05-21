/* drivers/input/touchscreen/gt1x_tpd.c
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
 * Version: 1.0   
 * Revision Record: 
 *      V1.0:  first release. 2014/09/28.
 *
 */

#include "gt1x_tpd_custom.h"
#include "gt1x_generic.h"

#ifdef GTP_CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#if TPD_SUPPORT_I2C_DMA
#include <linux/dma-mapping.h>
#endif

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif
#include <linux/proc_fs.h> 

#include "circle_point.h"
extern struct tpd_device *tpd;
static spinlock_t irq_lock;
static int tpd_flag = 0;
static int tpd_irq_flag;
int tpd_halt = 0;
static int tpd_eint_mode = 1;
static struct task_struct *thread = NULL;
static int tpd_polling_time = 50;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
DEFINE_MUTEX(i2c_access);

#if TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT] = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#ifdef GTP_CONFIG_OF
static unsigned int tpd_touch_irq;
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc);
#else
static void tpd_eint_interrupt_handler(void);
#endif

//static unsigned int tpd_touch_irq;
//static void tpd_eint_interrupt_handler(void);
static int tpd_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);

#ifndef MT6572
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En, kal_bool ACT_Polarity, void (EINT_FUNC_PTR) (void), kal_bool auto_umask);
#endif

#define GTP_DRIVER_NAME  "gt1x"

//#define SUPPORT_GLOVE_MODE
#define SUPPORT_GESTURE
#define SUPPORT_REPORT_COORDINATE
struct Coordinate {
    int x;
    int y;
};
static  Point Point_input[64];
static  Point Point_output[4];
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
static uint32_t clockwise=1;

#ifdef SUPPORT_GESTURE
static atomic_t double_enable;
atomic_t glove_mode_enable;
atomic_t is_in_suspend;
static uint32_t gesture;
static uint32_t gesture_upload;

static DEFINE_SEMAPHORE(suspend_sem);//after suspend sucsess  , can start resume

#define DTAP_DETECT          0xCC
#define UP_VEE_DETECT        0x76 
#define DOWN_VEE_DETECT      0x5e
#define LEFT_VEE_DETECT      0x3e 
#define RIGHT_VEE_DETECT     0x63
#define CIRCLE_DETECT        0x6f
#define DOUSWIP_DETECT       0x48 
#define DOUUPSWIP_DETECT     0x4E
#define RIGHT_SLIDE_DETECT   0xAA
#define LEFT_SLIDE_DETECT    0xbb
#define DOWN_SLIDE_DETECT    0xAB
#define UP_SLIDE_DETECT      0xBA
#define M_DETECT			 0x6D
#define W_DETECT			 0x77


#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W	
#define CustomGestrue       14  //Custom
#endif


#ifdef SUPPORT_GESTURE
struct proc_dir_entry *prEntry_tp = NULL; 
static struct proc_dir_entry *prEntry_dtap = NULL;
static struct proc_dir_entry *prEntry_coodinate  = NULL; 
static int init_goodix_proc(void);
static int tp_double_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos);
static int tp_double_write_func(struct file *file, const char *buffer, unsigned long count,void *data);
static int coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
//static int glove_mode_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos);
//static int glove_mode_write_func(struct file *file,const char *buffer, unsigned long count,void *data);
static const struct file_operations gt1x_gesture = {
	.owner = THIS_MODULE,
	.read  = tp_double_read_func,
	.write = tp_double_write_func,
};
static const struct file_operations gt1x_gesture_coor = {
	.owner = THIS_MODULE,
	.read  = coordinate_proc_read_func,
};
#endif

#ifdef SUPPORT_GLOVE_MODE
static const struct file_operations gt1x_gesture_glove_mode= {
	.owner = THIS_MODULE,
	.read  = glove_mode_read_func,
	.write= glove_mode_write_func,
};
#endif
//Chenggang.Li@BSP.TP add for 16021 2016/04/27 for gesture
/************start*************/
int firmware_id;
char manu_name[12];

/************end*************/


static const struct i2c_device_id tpd_i2c_id[] = { {GTP_DRIVER_NAME, 0}, {} };
static unsigned short force[] = { 0, GTP_I2C_ADDRESS, I2C_CLIENT_END, I2C_CLIENT_END };
static const unsigned short *const forces[] = { force, NULL };

//static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO(GTP_DRIVER_NAME, (GTP_I2C_ADDRESS >> 1)) };

static struct i2c_driver tpd_i2c_driver = {
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.detect = tpd_i2c_detect,
	.driver.name = GTP_DRIVER_NAME,
	.id_table = tpd_i2c_id,
	.address_list = (const unsigned short *)forces,
};

#if TPD_SUPPORT_I2C_DMA
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
struct mutex dma_mutex;

static s32 i2c_dma_write_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret = 0;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = addr;
	struct i2c_msg msg = {
		.flags = !I2C_M_RD,
		.ext_flag = (gt1x_i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		.timing = I2C_MASTER_CLOCK,
		.buf = (u8 *) gpDMABuf_pa,
	};

	mutex_lock(&dma_mutex);
	while (pos != len) {
		if (len - pos > (IIC_DMA_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH)) {
			transfer_length = IIC_DMA_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH;
		} else {
			transfer_length = len - pos;
		}
		
		gpDMABuf_va[0] = (address >> 8) & 0xFF;
		gpDMABuf_va[1] = address & 0xFF;
		memcpy(&gpDMABuf_va[GTP_ADDR_LENGTH], &buffer[pos], transfer_length);

		msg.len = transfer_length + GTP_ADDR_LENGTH;

		ret = i2c_transfer(gt1x_i2c_client->adapter, &msg, 1);
		if (ret != 1) {
			GTP_ERROR("I2c Transfer error! (%d)", ret);
			ret = ERROR_IIC;
			break;
		}
		ret = 0;
		pos += transfer_length;
		address += transfer_length;
	}
	mutex_unlock(&dma_mutex);
	return ret;
}

static s32 i2c_dma_read_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret = ERROR;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = addr;
	u8 addr_buf[GTP_ADDR_LENGTH] = { 0 };
	struct i2c_msg msgs[2] = {
		{
		 .flags = 0,	//!I2C_M_RD,
		 .addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		 .timing = I2C_MASTER_CLOCK,
		 .len = GTP_ADDR_LENGTH,
		 .buf = addr_buf,
		 },
		{
		 .flags = I2C_M_RD,
		 .ext_flag = (gt1x_i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		 .addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		 .timing = I2C_MASTER_CLOCK,
		 .buf = (u8 *) gpDMABuf_pa,
		 },
	};
	mutex_lock(&dma_mutex);
	while (pos != len) {
		if (len - pos > IIC_DMA_MAX_TRANSFER_SIZE) {
			transfer_length = IIC_DMA_MAX_TRANSFER_SIZE;
		} else {
			transfer_length = len - pos;
		}

		msgs[0].buf[0] = (address >> 8) & 0xFF;
		msgs[0].buf[1] = address & 0xFF;
		msgs[1].len = transfer_length;

		ret = i2c_transfer(gt1x_i2c_client->adapter, msgs, 2);
		if (ret != 2) {
			GTP_ERROR("I2C Transfer error! (%d)", ret);
			ret = ERROR_IIC;
			break;
		}
		ret = 0;
		memcpy(&buffer[pos], gpDMABuf_va, transfer_length);
		pos += transfer_length;
		address += transfer_length;
	};
	mutex_unlock(&dma_mutex);
	return ret;
}

#else

static s32 i2c_write_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret;

	struct i2c_msg msg = {
		.flags = 0,
		.addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG),	//remain
		.timing = I2C_MASTER_CLOCK,
	};

	ret = _do_i2c_write(&msg, addr, buffer, len);
	return ret;
}

static s32 i2c_read_mtk(u16 addr, u8 * buffer, s32 len)
{
	int ret;
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };

	struct i2c_msg msgs[2] = {
		{
		 .addr = ((gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH,
		 .timing = I2C_MASTER_CLOCK},
		{
		 .addr = ((gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		 .flags = I2C_M_RD,
		 .timing = I2C_MASTER_CLOCK},
	};

	ret = _do_i2c_read(msgs, addr, buffer, len);
	return ret;
}
#endif /* TPD_SUPPORT_I2C_DMA */

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
#if TPD_SUPPORT_I2C_DMA
	return i2c_dma_read_mtk(addr, buffer, len);
#else
	return i2c_read_mtk(addr, buffer, len);
#endif
}

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
#if TPD_SUPPORT_I2C_DMA
	return i2c_dma_write_mtk(addr, buffer, len);
#else
	return i2c_write_mtk(addr, buffer, len);
#endif
}

#ifdef TPD_REFRESH_RATE
/*******************************************************
Function:
    Write refresh rate

Input:
    rate: refresh rate N (Duration=5+N ms, N=0~15)

Output:
    Executive outcomes.0---succeed.
*******************************************************/
static u8 gt1x_set_refresh_rate(u8 rate)
{
	u8 buf[1] = { rate };

	if (rate > 0xf) {
		GTP_ERROR("Refresh rate is over range (%d)", rate);
		return ERROR_VALUE;
	}

	GTP_INFO("Refresh rate change to %d", rate);
	return gt1x_i2c_write(GTP_REG_REFRESH_RATE, buf, sizeof(buf));
}

/*******************************************************
Function:
    Get refresh rate

Output:
    Refresh rate or error code
*******************************************************/
static u8 gt1x_get_refresh_rate(void)
{
	int ret;
	u8 buf[1] = { 0x00 };
	ret = gt1x_i2c_read(GTP_REG_REFRESH_RATE, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	GTP_INFO("Refresh rate is %d", buf[0]);
	return buf[0];
}

//=============================================================
static ssize_t show_refresh_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = gt1x_get_refresh_rate();
	if (ret < 0)
		return 0;
	else
		return sprintf(buf, "%d\n", ret);
}

static ssize_t store_refresh_rate(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//u32 rate = 0;
	gt1x_set_refresh_rate(simple_strtoul(buf, NULL, 16));
	return size;
}

static DEVICE_ATTR(tpd_refresh_rate, 0664, show_refresh_rate, store_refresh_rate);

static struct device_attribute *gt9xx_attrs[] = {
	&dev_attr_tpd_refresh_rate,
};
#endif
//=============================================================

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, "mtk-tpd");
	return 0;
}

static int tpd_power_on(void)
{
	gt1x_power_switch(SWITCH_ON);

	gt1x_select_addr();
	msleep(10);

	if (gt1x_get_chip_type() != 0) {
		return -1;
	}

	if (gt1x_reset_guitar() != 0) {
		return -1;
	}

	return 0;
}

void gt1x_irq_enable(void)
{
   	unsigned long flag;
    
    spin_lock_irqsave(&irq_lock, flag);
    if (!tpd_irq_flag) { // 0-disabled
        tpd_irq_flag = 1;  // 1-enabled
#ifdef GTP_CONFIG_OF
        enable_irq(tpd_touch_irq);
#else
	    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
    }
    spin_unlock_irqrestore(&irq_lock, flag);  
}

void gt1x_irq_disable(void)
{  
    unsigned long flag;

    spin_lock_irqsave(&irq_lock, flag);
    if (tpd_irq_flag) {
        tpd_irq_flag = 0;
#ifdef GTP_CONFIG_OF
        disable_irq(tpd_touch_irq);
#else
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
    }
    spin_unlock_irqrestore(&irq_lock, flag);
}

void gt1x_power_switch(s32 state)
{
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(10);

	switch (state) {
	case SWITCH_ON:
		GTP_DEBUG("Power switch on!");
#ifdef MT6573
		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#else // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )
#ifdef TPD_POWER_SOURCE_CUSTOM
		hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
		//hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
		hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif
#endif
		break;
	case SWITCH_OFF:
		GTP_DEBUG("Power switch off!");
#ifdef MT6573
		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
#else
#ifdef TPD_POWER_SOURCE_1800
		hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_CUSTOM
		hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#else
		//hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
#endif
#endif
		break;
	default:
		GTP_ERROR("Invalid power switch command!");
		break;
	}
}


static int tpd_irq_registration(void)
{
#ifdef GTP_CONFIG_OF
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};
	GTP_INFO("Device Tree Tpd_irq_registration!");
	
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if(node){
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		tpd_touch_irq = irq_of_parse_and_map(node, 0);
		GTP_INFO("Device gt1x_int_type = %d!", gt1x_int_type);
		if (!gt1x_int_type)	//EINTF_TRIGGER
		{
			ret = request_irq(tpd_touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_RISING, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		} else {
			ret = request_irq(tpd_touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		}
	}else{
		GTP_ERROR("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	GTP_INFO("tpd_irq_registration::irq:%d, debounce:%d-%d:", tpd_touch_irq, ints[0], ints[1]);
	return ret;
    
#else

    #ifndef MT6589
	if (!gt1x_int_type) {	/*EINTF_TRIGGER */
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
	} else {
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	}

    #else
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);

	if (!gt1x_int_type) {
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
	} else {
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tpd_eint_interrupt_handler, 1);
	}
    #endif
    return 0;
#endif
}


static s32 tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 err = 0;

#if GTP_HAVE_TOUCH_KEY
	s32 idx = 0;
#endif
	printk("...enter probe 16021.....\n");
	gt1x_i2c_client = client;
	spin_lock_init(&irq_lock);
    init_goodix_proc();
	if (gt1x_init()) {
		/* TP resolution == LCD resolution, no need to match resolution when initialized fail */
		gt1x_abs_x_max = 0;
		gt1x_abs_y_max = 0;
		
	}
	

	thread = kthread_run(tpd_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		GTP_INFO(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}
#if GTP_HAVE_TOUCH_KEY
	for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++) {
		input_set_capability(tpd->dev, EV_KEY, gt1x_touch_key_array[idx]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE);
#endif

#ifdef SUPPORT_GESTURE
	 input_set_capability(tpd->dev, EV_KEY, KEY_F4);//doulbe-tap resume	
#endif

	GTP_GPIO_AS_INT(GTP_INT_PORT);

	msleep(50);

/* interrupt registration */

	tpd_irq_registration();
	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	/*  must before auto update */
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE

	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x_auto_update");
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		GTP_INFO(TPD_DEVICE " failed to create auto-update thread: %d\n", err);
	}
#endif

#ifdef SUPPORT_GESTURE
	atomic_set(&double_enable,0); 
	atomic_set(&glove_mode_enable,0); 
	atomic_set(&is_in_suspend,0);
#endif
     
	tpd_load_status = 1;

	return 0;
}

/* static void tpd_eint_interrupt_handler(void)
{
	//TPD_DEBUG_PRINT_INT;

	tpd_flag = 1;
    //GTP_INFO("interrupt func %d\n",__LINE__);
	wake_up_interruptible(&waiter);
} */

#ifdef GTP_CONFIG_OF
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
    TPD_DEBUG_PRINT_INT;
	
	tpd_flag = 1;
	//GTP_INFO(".......tpd_flag=1......\n");
	/* use _nosync to avoid deadlock */
    spin_lock(&irq_lock);
    tpd_irq_flag = 0;
	disable_irq_nosync(tpd_touch_irq);
    spin_unlock(&irq_lock);
 	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void tpd_eint_interrupt_handler(void)
{
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
    gt1x_irq_disable();
	wake_up_interruptible(&waiter);
}
#endif


void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(tpd->dev, id);
	//input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_report_key(tpd->dev, BTN_TOUCH, 1);		
	input_report_key(tpd->dev,	BTN_TOOL_FINGER, 1);
#else
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		//input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		//input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	}
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
#endif
	GTP_ERROR("tpd_down::x[%d] y[%d]=[%d %d]\n", id,id,x,y);
	input_sync(tpd->dev);
	
#ifdef TPD_HAVE_BUTTON
	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {
		tpd_button(x, y, 1);
	}
#endif
}

void gt1x_touch_up(s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(tpd->dev, id);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
	input_report_key(tpd->dev, BTN_TOUCH, 0);		
	input_report_key(tpd->dev, BTN_TOOL_FINGER, 0);
	input_sync(tpd->dev);
#else
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
#endif
#ifdef TPD_HAVE_BUTTON
	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {
		tpd_button(0, 0, 0);
	}
#endif
}

#if GTP_CHARGER_SWITCH
#ifdef MT6573
#define CHR_CON0      (0xF7000000+0x2FA00)
#else
extern kal_bool upmu_is_chr_det(void);
#endif

u32 gt1x_get_charger_status(void)
{
	u32 chr_status = 0;
#ifdef MT6573
	chr_status = *(volatile u32 *)CHR_CON0;
	chr_status &= (1 << 13);
#else /* ( defined(MT6575) || defined(MT6577) || defined(MT6589) ) */
	chr_status = upmu_is_chr_det();
#endif
	return chr_status;
}
#endif

int ClockWise(Point *p,int n)
{

	int i,j,k;
	int count = 0;
	//double z;
	long int z;
	if (n < 3)
		return -1;
	for (i=0;i<n;i++) 
	{
		j = (i + 1) % n;
		k = (i + 2) % n;
		if( (p[i].x==p[j].x) && (p[j].x==p[j].y) )
		   continue;
		z = (p[j].x - p[i].x) * (p[k].y - p[j].y);
		z -= (p[j].y - p[i].y) * (p[k].x - p[j].x);
		if (z < 0)
			count--;
		else if (z > 0)
			count++;
	}
    
	printk("ClockWise count = %d\n",count);
	if (count > 0)
		return 1; 
	else if (count < 0)
		return 0;
	else
		return 0;
}

static int tpd_event_handler(void *unused)
{
	u8 finger = 0;
	u8 end_cmd = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };
	struct sched_param param = {.sched_priority = RTPM_PRIO_CPU_CALLBACK };//RTPM_PRIO_TPD
	
    u8 doze_buf[3];
	u8 clear_buf[1];
	u8 coordinate_single[260];
	u8 coordinate_size;
	u8 length = 0;
	int i = 0;
	int j = 0;
	double clock=0;
	printk("creat tpd_event_handler successful!\n");
	sched_setscheduler(current, SCHED_RR, &param);
	do {
	
		set_current_state(TASK_INTERRUPTIBLE);

		if (tpd_eint_mode) {
			wait_event_interruptible(waiter, tpd_flag != 0);
			tpd_flag = 0;
		} else {
			GTP_DEBUG("Polling coordinate mode!");
			msleep(tpd_polling_time);
		}

		set_current_state(TASK_RUNNING);
		mutex_lock(&i2c_access);
		
#if GTP_GESTURE_WAKEUP
#ifdef SUPPORT_GESTURE
	 
        if (DOZE_ENABLED == gesture_doze_status)
        {
        
				ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE, doze_buf, 2);
				GTP_DEBUG("0x814C = 0x%02X,ret=%d\n", doze_buf[0],ret);  
				if (ret == 0 )
				{  
				   
					if(doze_buf[0] != 0)
					{
					    memset(coordinate_single, 0, 260);
					    coordinate_size=doze_buf[1];
						GTP_ERROR("report gesture::original gesture=%d  coordinate_size=%d \n",doze_buf[0],coordinate_size);
						if(coordinate_size>64) //mingqiang.guo@phone.bsp add coordinate_size*4 can not large > 260 
							coordinate_size = 64;
				    	ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE_DETAIL, coordinate_single, coordinate_size*4);
							//rendong.shi add gesture
							switch (doze_buf[0]) 
							{
								case DTAP_DETECT:
								gesture = DouTap;		
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x =  0; 
								Point_1st.y =  0; 
								clockwise = 0 ;
								break;

								case UP_VEE_DETECT :
								gesture = UpVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case DOWN_VEE_DETECT :
								gesture = DownVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case LEFT_VEE_DETECT:
								gesture =  LeftVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case RIGHT_VEE_DETECT :
								gesture =  RightVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;	
								
								case CIRCLE_DETECT  :
								gesture =  Circle;
								j = 0; 
								for(i = 0; i < coordinate_size;i++)
								{
									Point_input[i].x = coordinate_single[j]  |  (coordinate_single[j+1] << 8);
									Point_input[i].y = coordinate_single[j+2] |  (coordinate_single[j+3] << 8);
									j = j+4;
									GTP_INFO("Point_input[%d].x = %d,Point_input[%d].y = %d\n",i,Point_input[i].x,i,Point_input[i].y)	;					
								}

								clockwise = ClockWise(&Point_input[0],coordinate_size-2);
								GetCirclePoints(&Point_input[0], coordinate_size,Point_output);
								Point_start.x = Point_input[0].x;
								Point_start.y = Point_input[0].y;

								Point_end.x = Point_input[coordinate_size-1].x;
								Point_end.y = Point_input[coordinate_size-1].y;

								Point_1st.x = Point_output[0].x;
								Point_1st.y = Point_output[0].y;
						
								Point_2nd.x = Point_output[1].x;
								Point_2nd.y = Point_output[1].y;
						
								Point_3rd.x = Point_output[2].x;
								Point_3rd.y = Point_output[2].y;
						
								Point_4th.x = Point_output[3].x;
								Point_4th.y = Point_output[3].y;
								break;
								
								case DOUSWIP_DETECT  :
								gesture =  DouSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
								Point_2nd.x = coordinate_single[12]  |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;
							
								case DOUUPSWIP_DETECT:
								gesture =  DouSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
								Point_2nd.x = coordinate_single[12]  |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;
								
								case RIGHT_SLIDE_DETECT :
								gesture =  Left2RightSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
							    break;
								
								case LEFT_SLIDE_DETECT :
								gesture =  Right2LeftSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case DOWN_SLIDE_DETECT  :
								gesture =  Up2DownSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
							    break;
								
								case UP_SLIDE_DETECT :
							    gesture =  Down2UpSwip;
							    Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;	
								
								case M_DETECT  :
								gesture =  Mgestrue;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
								Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_2nd.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_2nd.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_3rd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_3rd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
							    break;
								
								case W_DETECT :
								gesture =  Wgestrue;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
								Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_2nd.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_2nd.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_3rd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_3rd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;	
								
								default:
								if((doze_buf[0]>=1)&&(doze_buf[0]<=15))
							{
								gesture =  doze_buf[0] + OPPO_CUSTOM_GESTURE_ID_BASE;  //user custom gesture 
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_2nd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								Point_3rd.x = coordinate_single[16] |  (coordinate_single[17] << 8);
								Point_3rd.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_3rd.x = coordinate_single[20] |  (coordinate_single[21] << 8);
								Point_3rd.y = coordinate_single[22] |  (coordinate_single[23] << 8); 
							}
							else
							{
								gesture =  doze_buf[0];
							}
							    break;
					
			                }
							GTP_ERROR("report gesture::detect %s gesture\n", gesture == DouTap ? "double tap" :
                                                        gesture == UpVee ? "up vee" :
                                                        gesture == DownVee ? "down vee" :
                                                        gesture == LeftVee ? "(>)" :
                                                        gesture == RightVee ? "(<)" :
                                                        gesture == Circle ? "circle" :
														gesture == DouSwip ? "(||)" :
                                                        gesture == Left2RightSwip ? "(-->)" :
                                                        gesture == Right2LeftSwip ? "(<--)" :
                                                        gesture == Up2DownSwip ? "up to down |" :
                                                        gesture == Down2UpSwip ? "down to up |" :
                                                        gesture == Mgestrue ? "(M)" :
														gesture == Wgestrue ? "(W)" : "oppo custom gesture");

					//		if( (gesture==LeftVee) || (gesture==RightVee) || (gesture==DouSwip)  )
					//		{
					//			GTP_INFO("gesture=LeftVeetp  %d go to doze\n",gesture);
					//			gesture_enter_doze();
					//		} 

							if(gesture > OPPO_CUSTOM_GESTURE_ID_BASE )
							{
								ret = gesture_event_handler(tpd->dev);
								if (ret >= 0) 
								{
									gt1x_irq_enable();
									mutex_unlock(&i2c_access);
									continue;
								}
							}
							else
							{
								GTP_ERROR("report gesture::KEY_F4!!! gesture=%d\n",gesture);
								gesture_upload=gesture;
								input_report_key(tpd->dev, KEY_F4, 1);
								input_sync(tpd->dev);
								input_report_key(tpd->dev, KEY_F4, 0);
								input_sync(tpd->dev);
								clear_buf[0] = 0x00;
								gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, clear_buf, 1);

							}

					}
					else       
					{
					        GTP_ERROR("report gesture::Unknow gesture!!!code =%d \n",doze_buf[0]);
					     	//clear_buf[0] = 0x00;
							//gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, clear_buf, 1);	
							//gesture_enter_doze();
					}	
				}
             gt1x_irq_enable();
		     mutex_unlock(&i2c_access);
            continue;
       }

#endif
#endif
		if (tpd_halt) {
			mutex_unlock(&i2c_access);
			GTP_DEBUG("return for interrupt after suspend...  ");
			continue;
		}

		/* read coordinates */
		ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
		if (ret < 0) {
			GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
			gt1x_power_reset();
#endif
			goto exit_work_func;
		}
		
		finger = point_data[0];

		/* response to a ic request */
		if (finger == 0x00) {
			gt1x_request_event_handler();
		}

		if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
			if (!hotknot_paired_flag)
#endif
			{
				gt1x_irq_enable();
				mutex_unlock(&i2c_access);
				GTP_ERROR("buffer not ready:0x%02x", finger); 
				continue;
				//goto exit_work_func;
			}
		}
#if HOTKNOT_BLOCK_RW
		ret = hotknot_event_handler(point_data);
		if (!ret) {
			goto exit_work_func;
		}
#endif

#if GTP_PROXIMITY
		ret = gt1x_prox_event_handler(point_data);
		if (ret > 0) {
			goto exit_work_func;
		}
#endif

#if GTP_WITH_STYLUS
		ret = gt1x_touch_event_handler(point_data, tpd->dev, pen_dev);
#else
		ret = gt1x_touch_event_handler(point_data, tpd->dev, NULL);
#endif
		if (ret) {
			gt1x_irq_enable();
			mutex_unlock(&i2c_access);
			continue;
		}

exit_work_func:

		if (!gt1x_rawdiff_mode) {
			ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
			if (ret < 0) {
				GTP_INFO("I2C write end_cmd  error!");
			}
		}
		gt1x_irq_enable();
		mutex_unlock(&i2c_access);
	} while (!kthread_should_stop());
return 0;
}

int gt1x_debug_proc(u8 * buf, int count)
{
	char mode_str[50] = { 0 };
	int mode;

	sscanf(buf, "%s %d", (char *)&mode_str, &mode);

	/***********POLLING/EINT MODE switch****************/
	if (strcmp(mode_str, "polling") == 0) {
		if (mode >= 10 && mode <= 200) {
			GTP_INFO("Switch to polling mode, polling time is %d", mode);
			tpd_eint_mode = 0;
			tpd_polling_time = mode;
			tpd_flag = 1;
			wake_up_interruptible(&waiter);
		} else {
			GTP_INFO("Wrong polling time, please set between 10~200ms");
		}
		return count;
	}
	if (strcmp(mode_str, "eint") == 0) {
		GTP_INFO("Switch to eint mode");
		tpd_eint_mode = 1;
		return count;
	}
	/**********************************************/
	if (strcmp(mode_str, "switch") == 0) {
		if (mode == 0)	// turn off
			tpd_off();
		else if (mode == 1)	//turn on
			tpd_on();
		else
			GTP_ERROR("error mode :%d", mode);
		return count;
	}

	return -1;
}

static u16 convert_productname(u8 * name)
{
	int i;
	u16 product = 0;
	for (i = 0; i < 4; i++) {
		product <<= 4;
		if (name[i] < '0' || name[i] > '9') {
			product += '*';
		} else {
			product += name[i] - '0';
		}
	}
	return product;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
	gt1x_deinit();
	
	return 0;
}

static int tpd_local_init(void)
{
#ifdef TPD_POWER_SOURCE_CUSTOM
#ifdef GTP_CONFIG_OF
#ifdef CONFIG_ARCH_MT6580    
	tpd->reg = regulator_get(tpd->tpd_dev,TPD_POWER_SOURCE_CUSTOM);// get pointer to regulator structure    
	if (IS_ERR(tpd->reg)) {        
		GTP_ERROR("regulator_get() failed.");    
		}
#endif
#endif
#endif

#if TPD_SUPPORT_I2C_DMA
	mutex_init(&dma_mutex);
	gpDMABuf_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, IIC_DMA_MAX_TRANSFER_SIZE, &gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va) {
		GTP_ERROR("Allocate DMA I2C Buffer failed!");
		return -1;
	}
	memset(gpDMABuf_va, 0, IIC_DMA_MAX_TRANSFER_SIZE);
#endif
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		GTP_ERROR("unable to add i2c driver.");
		return -1;
	}

	if (tpd_load_status == 0)	// disable auto load touch driver for linux3.0 porting
	{
		GTP_ERROR("add error touch panel driver.");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (GTP_MAX_TOUCH - 1), 0, 0);
#if TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);	// initialize tpd button data
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

	// set vendor string
	tpd->dev->id.vendor = 0x00;
	tpd->dev->id.product = convert_productname(gt1x_version.product_id);
	tpd->dev->id.version = (gt1x_version.patch_id >> 8);

	GTP_INFO("end %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

#ifdef SUPPORT_GESTURE

static int tp_double_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos)
{                           
    char pagesize[512];
    int len = 0;
	GTP_INFO("double tap enable is: %d\n", atomic_read(&double_enable));
	len = sprintf(pagesize, "%d\n", atomic_read(&double_enable));
	len = simple_read_from_buffer(page, size, ppos, pagesize, strlen(pagesize)); 
	return len; 
}

static int tp_double_write_func(struct file *file,const char *buffer, unsigned long count,void *data)
{
	int ret = 0;

	char buf[10] = {0};
	static int in_suspend_gustrue_status;
	
	if (count > 10) 
		return count;

	if (copy_from_user( buf, buffer, count)) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	 
	sscanf(buf,"%d",&ret);

	down(&suspend_sem);
	GTP_INFO("double_write_func  %d\n",ret);
	if( atomic_read(&is_in_suspend) )
	{
		if(in_suspend_gustrue_status == ret)
		{
			GTP_INFO("do not need operate when gesture status is same\n");
			up(&suspend_sem);
			return count;
		}

		in_suspend_gustrue_status = ret; 
		switch(ret)
		{
			case 0:
				gt1x_wakeup_sleep_gesture();
				gt1x_enter_sleep_gesture();
				break;
			case 1:
				gt1x_wakeup_sleep_gesture();
				gesture_enter_doze_gesture();
				break;


			default:
				GTP_INFO("Please enter 0 or 1 to open or close the double-tap function\n");
		}
		up(&suspend_sem);
		return count;//can not save double_enable flag when is_in_suspend
		
	}
	else
	{
		switch(ret)
		{	
			case 0:
				GTP_DEBUG("tp_guesture_func will be disable\n");
				break;
			case 1:
				GTP_DEBUG("tp_guesture_func will be enable\n");
				break;
			default:
				GTP_DEBUG("Please enter 0 or 1 to open or close the double-tap function\n");
		}
	}
	up(&suspend_sem);

	if((ret == 0 )||(ret == 1))
	{
		atomic_set(&double_enable,ret);
		in_suspend_gustrue_status = ret;
	}

	return count;
}

#ifdef SUPPORT_REPORT_COORDINATE
static int coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char page[512];

	len =sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	GTP_INFO("%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);
   printk("return ret=%d len=%d\n",ret,len);

	return ret;
}
#endif
#endif

s32 gt1x_init_glovemode_panel(void)
{
	s32 ret = 0;
	u8 cfg_len = 0;
    u8 buf;
#if GTP_DRIVER_SEND_CFG
	u8 sensor_id = 0;
	 u8 cfg_grp0[] = GTP_CFG_GROUP0_GLOVE;
	 u8 cfg_grp1[] = GTP_CFG_GROUP1_GLOVE;
	 u8 cfg_grp2[] = GTP_CFG_GROUP2_GLOVE;
	 u8 cfg_grp3[] = GTP_CFG_GROUP3_GLOVE;
	 u8 cfg_grp4[] = GTP_CFG_GROUP4_GLOVE;
	 u8 cfg_grp5[] = GTP_CFG_GROUP5_GLOVE;
	 u8 *cfgs[] = {
		cfg_grp0, cfg_grp1, cfg_grp2,
		cfg_grp3, cfg_grp4, cfg_grp5
	};
	u8 cfg_lens[] = {
		CFG_GROUP_LEN(cfg_grp0),
		CFG_GROUP_LEN(cfg_grp1),
		CFG_GROUP_LEN(cfg_grp2),
		CFG_GROUP_LEN(cfg_grp3),
		CFG_GROUP_LEN(cfg_grp4),
		CFG_GROUP_LEN(cfg_grp5)
	};

#if GTP_CHARGER_SWITCH
	 u8 cfg_grp0_charger[] = GTP_CFG_GROUP0_CHARGER;
	 u8 cfg_grp1_charger[] = GTP_CFG_GROUP1_CHARGER;
	 u8 cfg_grp2_charger[] = GTP_CFG_GROUP2_CHARGER;
	 u8 cfg_grp3_charger[] = GTP_CFG_GROUP3_CHARGER;
	 u8 cfg_grp4_charger[] = GTP_CFG_GROUP4_CHARGER;
	 u8 cfg_grp5_charger[] = GTP_CFG_GROUP5_CHARGER;
	 u8 *cfgs_charger[] = {
		cfg_grp0_charger, cfg_grp1_charger, cfg_grp2_charger,
		cfg_grp3_charger, cfg_grp4_charger, cfg_grp5_charger
	};
	u8 cfg_lens_charger[] = {
		CFG_GROUP_LEN(cfg_grp0_charger),
		CFG_GROUP_LEN(cfg_grp1_charger),
		CFG_GROUP_LEN(cfg_grp2_charger),
		CFG_GROUP_LEN(cfg_grp3_charger),
		CFG_GROUP_LEN(cfg_grp4_charger),
		CFG_GROUP_LEN(cfg_grp5_charger)
	};
#endif /* end  GTP_CHARGER_SWITCH */
	GTP_DEBUG("Config Groups Length: %d, %d, %d, %d, %d, %d", cfg_lens[0], cfg_lens[1], cfg_lens[2], cfg_lens[3], cfg_lens[4], cfg_lens[5]);
	sensor_id = gt1x_version.sensor_id;
	if (sensor_id >= 6 || cfg_lens[sensor_id] < GTP_CONFIG_MIN_LENGTH || cfg_lens[sensor_id] > GTP_CONFIG_MAX_LENGTH) {
		sensor_id = 0;
	}
	cfg_len = cfg_lens[sensor_id];
   
	if (cfg_len < GTP_CONFIG_MIN_LENGTH || cfg_len > GTP_CONFIG_MAX_LENGTH) {
		GTP_ERROR("CTP_CONFIG_GROUP%d is INVALID CONFIG GROUP! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id + 1);
		return -1;
	}

	gt1x_irq_disable();
	memset(gt1x_config, 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(gt1x_config, cfgs[sensor_id], cfg_len);
     
	/* clear the flag, avoid failure when send the_config of driver. */
	gt1x_config[0] &= 0x7F;

#if GTP_CUSTOM_CFG
	gt1x_config[RESOLUTION_LOC] = (u8) GTP_MAX_WIDTH;
	gt1x_config[RESOLUTION_LOC + 1] = (u8) (GTP_MAX_WIDTH >> 8);
	gt1x_config[RESOLUTION_LOC + 2] = (u8) GTP_MAX_HEIGHT;
	gt1x_config[RESOLUTION_LOC + 3] = (u8) (GTP_MAX_HEIGHT >> 8);

	GTP_INFO("Res: %d * %d, trigger: %d", GTP_MAX_WIDTH, GTP_MAX_HEIGHT, GTP_INT_TRIGGER);

	if (GTP_INT_TRIGGER == 0) {	/* RISING  */
		gt1x_config[TRIGGER_LOC] &= 0xfe;
	} else if (GTP_INT_TRIGGER == 1) {	/* FALLING */
		gt1x_config[TRIGGER_LOC] |= 0x01;
	}
#endif /* END GTP_CUSTOM_CFG */

#if GTP_CHARGER_SWITCH
	GTP_DEBUG("Charger Config Groups Length: %d, %d, %d, %d, %d, %d", cfg_lens_charger[0],
			cfg_lens_charger[1], cfg_lens_charger[2], cfg_lens_charger[3], cfg_lens_charger[4], cfg_lens_charger[5]);
	memset(gt1x_config_charger, 0, sizeof(gt1x_config_charger));
	if (cfg_lens_charger[sensor_id] == cfg_len) {
		memcpy(gt1x_config_charger, cfgs_charger[sensor_id], cfg_len);
	}
	gt1x_config_charger[0] &= 0x7F;
#if GTP_CUSTOM_CFG
	gt1x_config_charger[RESOLUTION_LOC] = (u8) GTP_MAX_WIDTH;
	gt1x_config_charger[RESOLUTION_LOC + 1] = (u8) (GTP_MAX_WIDTH >> 8);
	gt1x_config_charger[RESOLUTION_LOC + 2] = (u8) GTP_MAX_HEIGHT;
	gt1x_config_charger[RESOLUTION_LOC + 3] = (u8) (GTP_MAX_HEIGHT >> 8);
	if (GTP_INT_TRIGGER == 0) {	/* RISING  */
		gt1x_config_charger[TRIGGER_LOC] &= 0xfe;
	} else if (GTP_INT_TRIGGER == 1) {	/* FALLING */
		gt1x_config_charger[TRIGGER_LOC] |= 0x01;
	}
#endif /* END GTP_CUSTOM_CFG */
	if (cfg_lens_charger[sensor_id] != cfg_len) {
		memset(gt1x_config_charger, 0, sizeof(gt1x_config_charger));
	}
#endif /* END GTP_CHARGER_SWITCH */

#else /* DRIVER NOT SEND CONFIG */
	cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = gt1x_i2c_read(GTP_REG_CONFIG_DATA, gt1x_config, cfg_len);
	if (ret < 0) {
		gt1x_irq_enable();	
		return ret;
	}
#endif /* END GTP_DRIVER_SEND_CFG */

	//GTP_DEBUG_FUNC();
	/* match resolution when gt1x_abs_x_max & gt1x_abs_y_max have been set already */
	if ((gt1x_abs_x_max == 0) && (gt1x_abs_y_max == 0)) {
		gt1x_abs_x_max = (gt1x_config[RESOLUTION_LOC + 1] << 8) + gt1x_config[RESOLUTION_LOC];
		gt1x_abs_y_max = (gt1x_config[RESOLUTION_LOC + 3] << 8) + gt1x_config[RESOLUTION_LOC + 2];
		gt1x_int_type = (gt1x_config[TRIGGER_LOC]) & 0x03;
//VENDOR_EDIT delete for suspend current 		gt1x_wakeup_level = !(gt1x_config[MODULE_SWITCH3_LOC] & 0x20);
	} else {
		gt1x_config[RESOLUTION_LOC] = (u8) gt1x_abs_x_max;
		gt1x_config[RESOLUTION_LOC + 1] = (u8) (gt1x_abs_x_max >> 8);
		gt1x_config[RESOLUTION_LOC + 2] = (u8) gt1x_abs_y_max;
		gt1x_config[RESOLUTION_LOC + 3] = (u8) (gt1x_abs_y_max >> 8);
		set_reg_bit(gt1x_config[MODULE_SWITCH3_LOC], 5, !gt1x_wakeup_level);
		gt1x_config[TRIGGER_LOC] = (gt1x_config[TRIGGER_LOC] & 0xFC) | gt1x_int_type;
#if GTP_CHARGER_SWITCH
		gt1x_config_charger[RESOLUTION_LOC] = (u8) gt1x_abs_x_max;
		gt1x_config_charger[RESOLUTION_LOC + 1] = (u8) (gt1x_abs_x_max >> 8);
		gt1x_config_charger[RESOLUTION_LOC + 2] = (u8) gt1x_abs_y_max;
		gt1x_config_charger[RESOLUTION_LOC + 3] = (u8) (gt1x_abs_y_max >> 8);
		set_reg_bit(gt1x_config[MODULE_SWITCH3_LOC], 5, !gt1x_wakeup_level);
		gt1x_config[TRIGGER_LOC] = (gt1x_config[TRIGGER_LOC] & 0xFC) | gt1x_int_type;
#endif
	}
    
	//GTP_INFO("X_MAX=%d,Y_MAX=%d,TRIGGER=0x%02x,WAKEUP_LEVEL=%d", gt1x_abs_x_max, gt1x_abs_y_max, gt1x_int_type, gt1x_wakeup_level);

	gt1x_cfg_length = cfg_len;
	ret = gt1x_send_cfg(gt1x_config, gt1x_cfg_length);

    gt1x_irq_enable();	
	return ret;
}


#ifdef SUPPORT_GLOVE_MODE
static int glove_mode_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos)
{                           
    char pagesize[512];
    int len = 0;
	GTP_INFO("glove_mode enable is: %d\n", atomic_read(&glove_mode_enable));
	len = sprintf(pagesize, "%d\n", atomic_read(&glove_mode_enable));
	len = simple_read_from_buffer(page, size, ppos, pagesize, strlen(pagesize)); 
	return len; 
}
static int glove_mode_write_func(struct file *file,const char *buffer, unsigned long count,void *data)
{

	int ret = 0;

	char buf[10] = {0};

	if (count > 10) 
		return count;

	if (copy_from_user( buf, buffer, count)) {
		GTP_INFO(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	sscanf(buf,"%d",&ret);
	GTP_DEBUG("%s :buf = %d,ret = %d\n",__func__,*buf,ret);
	if( ret ==  atomic_read(&glove_mode_enable) )
	{
		GTP_INFO("glove status is same , not need operate\n");
		return count;
	}
	
	if((ret == 0 )||(ret == 1))
	{
		atomic_set(&glove_mode_enable,ret);
		if(  1 ==  atomic_read(&is_in_suspend) )
		{
			GTP_DEBUG("when in suspend , operate glove mode after resume\n");
			return count;
		}
	}

    down(&suspend_sem); 
	switch(ret)
	{	
		case 0:
			GTP_INFO("glove mode  will be disable\n");
			gt1x_init_panel();
			msleep(150);
			break;

		case 1:
			GTP_INFO(" glove mode will be enable\n");
			gt1x_init_glovemode_panel();
			msleep(150);
			break;

		default:
			GTP_INFO("Please enter 0 or 1 to open or close the double-tap function\n");
	}
    up(&suspend_sem); 

	return count;
}
#endif

static int init_goodix_proc(void)
{
	int ret = 0;
	
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if(prEntry_tp == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_gt1x_proc: Couldn't create TP proc entry\n");
		return ret;
	}
	
#ifdef SUPPORT_GESTURE
	prEntry_dtap = proc_create("double_tap_enable", 0777, prEntry_tp, &gt1x_gesture);
	if(prEntry_dtap == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		return ret;
	}
	GTP_INFO("create gt1x gesture proc success\n");
#endif

#ifdef SUPPORT_GLOVE_MODE
prEntry_coodinate =  proc_create("glove_mode_enable", 0777, prEntry_tp, &gt1x_gesture_glove_mode);
    if(prEntry_coodinate == NULL)
    {	   
		ret = -ENOMEM;	   
		GTP_ERROR(KERN_INFO"glove_mode_enable : Couldn't create proc entry\n");
		return ret;
    }
	GTP_INFO("create  glove_mode_enable  proc success\n");
#endif 

#ifdef SUPPORT_REPORT_COORDINATE
	prEntry_coodinate =  proc_create("coordinate", 0777, prEntry_tp, &gt1x_gesture_coor);
    if(prEntry_coodinate == NULL)
    {	   
		ret = -ENOMEM;	   
		GTP_ERROR(KERN_INFO"init_gt1x_proc: Couldn't create proc entry\n");
		return ret;
    }
	GTP_INFO("create gt1x gesture_coor proc success\n");
    
#endif


}
/* Function to manage low power suspend */
static void tpd_suspend(struct early_suspend *h)
{
	s32 ret = -1;
#if GTP_HOTKNOT && !HOTKNOT_BLOCK_RW
	u8 buf[1] = { 0 };
#endif
     
	down(&suspend_sem); 
	GTP_ERROR("....tpd suspend enter...\n");

	if( atomic_read(&glove_mode_enable) )
	{
		gt1x_init_panel(); 
	}

	atomic_set(&is_in_suspend,1);

#if GTP_PROXIMITY
	if (gt1x_proximity_flag == 1) {
		GTP_INFO("Suspend: proximity is detected!");
		up(&suspend_sem); 
		return;
	}
#endif

#if GTP_HOTKNOT
	if (hotknot_enabled) {
#if HOTKNOT_BLOCK_RW
		if (hotknot_paired_flag) {
			GTP_INFO("Suspend: hotknot is paired!");
			up(&suspend_sem); 
			return;
		}
#else
		gt1x_i2c_read(GTP_REG_HN_PAIRED, buf, sizeof(buf));
		GTP_DEBUG("0x81AA: 0x%02X", buf[0]);
		if (buf[0] == 0x55) {
			GTP_INFO("Suspend: hotknot is paired!");
			up(&suspend_sem); 
			return;
		}
#endif
	}
#endif
	tpd_halt = 1;
	//GTP_INFO("%d\n",__LINE__);
#if GTP_ESD_PROTECT
	gt1x_esd_switch(SWITCH_OFF);
#endif

#if GTP_CHARGER_SWITCH
	gt1x_charger_switch(SWITCH_OFF);
#endif

	mutex_lock(&i2c_access);
  
#if GTP_GESTURE_WAKEUP
	gesture_clear_wakeup_data();
#ifdef SUPPORT_GESTURE	
	if(1 == atomic_read(&double_enable))
	{
	     GTP_ERROR("tp suspend::gesture is open!\n");
	     ret = gesture_enter_doze();
    }
	else
	{
	  	gt1x_irq_disable();
		ret = gt1x_enter_sleep();
		if (ret < 0)
		{
			GTP_ERROR("GTP early suspend failed.");
		}
	}
#endif 

#endif
	up(&suspend_sem); 
	mutex_unlock(&i2c_access);
	GTP_ERROR("....tpd suspend end...\n");
	msleep(58);
}

/* Function to manage power-on resume */
static void tpd_resume(struct early_suspend *h)
{
	s32 ret = -1;
	
	atomic_set(&is_in_suspend,0);
    down(&suspend_sem); 
	GTP_INFO("...tpd resume enter...\n");
	
#if GTP_PROXIMITY
	if (gt1x_proximity_flag == 1) {
		GTP_INFO("Resume: proximity is on!");
		up(&suspend_sem); 
		return;
	}
#endif

#if GTP_HOTKNOT
	if (hotknot_enabled) {
#if HOTKNOT_BLOCK_RW
		if (hotknot_paired_flag) {
			hotknot_paired_flag = 0;
			GTP_INFO("Resume: hotknot is paired!");
			up(&suspend_sem); 
			return;
		}
#endif
	}
#endif

	ret = gt1x_wakeup_sleep();
	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
#if GTP_HOTKNOT
	if (!hotknot_enabled) {
		gt1x_send_cmd(GTP_CMD_HN_EXIT_SLAVE, 0);
	}
#endif

#if GTP_CHARGER_SWITCH
	gt1x_charger_config(0);
	gt1x_charger_switch(SWITCH_ON);
#endif

    if( atomic_read(&glove_mode_enable) )
	{
		gt1x_init_glovemode_panel();
	}

	tpd_halt = 0;
	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	gt1x_esd_switch(SWITCH_ON);
#endif
	GTP_INFO("...tpd resume end...\n");
	up(&suspend_sem); 
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "gt9xx",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

void tpd_off(void)
{
	gt1x_power_switch(SWITCH_OFF);
	tpd_halt = 1;
	GTP_INFO("%d,%s\n",__LINE__,__func__);
	gt1x_irq_disable();
}

void tpd_on(void)
{
	s32 ret = -1, retry = 0;

	while (retry++ < 5) {
		ret = tpd_power_on();
		if (ret < 0) {
			GTP_ERROR("I2C Power on ERROR!");
		}

		ret = gt1x_send_cfg(gt1x_config, gt1x_cfg_length);
		if (ret == 0) {
			GTP_DEBUG("Wakeup sleep send gt1x_config success.");
			break;
		}
	}
	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
	//gt1x_irq_enable();
	tpd_halt = 0;
}

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	int ID1, ID2;
	
	GTP_ERROR("Goodix::tpd_driver_init\n");

	ID1 = mt_get_gpio_in(GPIO_TP_ID1_16021);//gpio21
	ID2 = mt_get_gpio_in(GPIO_TP_ID2_16021);//gpio19
	
	printk("ID1=%d ID2=%d\n", ID1,ID2);
	if( ID1==1&&ID2==0 ){
		printk("TP IS OFILM\n");
		firmware_id=0x16021100;
		strcpy(manu_name, "TP_OFILM");
	}else if(ID1==0&&ID2==1){
		printk("TP IS BIEL\n");
		firmware_id=0x16021600;
		strcpy(manu_name, "TP_BIEL");
	} 
	
	i2c_register_board_info(TPD_I2C_NUMBER, &i2c_tpd, 1);

	if (tpd_driver_add(&tpd_device_driver) < 0) {
		GTP_INFO("add generic driver failed\n");
	}else{
		GTP_INFO("add generic driver succeed!!!!!!!!!\n");
	}
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	GTP_INFO("MediaTek gt91xx touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
