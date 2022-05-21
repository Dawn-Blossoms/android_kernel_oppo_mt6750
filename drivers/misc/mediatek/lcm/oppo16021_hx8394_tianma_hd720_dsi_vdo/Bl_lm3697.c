/*****************************************************************
* Copyright (c) 2004-2016 OPPO Mobile communication Corp.ltd.,
* VENDOR_EDIT
* Description: Source file for LCM driver IC.
* Version   : 1.1
* Date      : 2016-05-10
* Author    : liping-m@PhoneSW.Multimedia
*----------------------Revision History--------------------------
* None.
*****************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
//#include <linux/oppo_devices_list.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>
#endif
#include <cust_gpio_usage.h>
#ifndef FPGA_EARLY_PORTING
#include <cust_i2c.h>
#endif

//#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, add for kpoc
//volatile int OPPO_LED_ON = 1;
//#endif/*VENDOR_EDIT*/

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif


//#define BL_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL
#define BL_I2C_BUSNUM  0
#define LM3697_SLAVE_ADDR_WRITE 	0x6C  //For BL (8 bit)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
#define LCD_BL_EN (GPIO11 | 0x80000000)
#define I2C_ID_NAME "LM3697"
static struct i2c_board_info __initdata BL_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, LM3697_SLAVE_ADDR_WRITE>>1)};
static struct i2c_client *I2C_LM3697 = NULL;
static const struct i2c_device_id DeviceId[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};


static int BL_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int BL_remove(struct i2c_client *client);

/* liping-m@PhoneSW.Multimedia, 2016/01/22	ADD for CTA LCD video mode */
unsigned int bl3697_esd_recovery_backlight_level = 2;

//extern OPPO_BKL_DEV oppo_bkl_dev;

static struct i2c_driver BL_I2C_driver = {
	.id_table	= DeviceId,
	.probe		= BL_probe,
	.remove		= BL_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "Black light",
	},
};

static int BL_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	I2C_LM3697  = client;
	#ifndef BUILD_LK
	//hwPowerOn(MT6325_POWER_LDO_VGP3, VOL_1800, "LCD_BACKLIGHT");
	printk("srd BL_probe\n");
	#endif
	return 0;
}
static int BL_remove(struct i2c_client *client)
{
	I2C_LM3697 = NULL;
	i2c_unregister_device(client);
	return 0;
}

//int LM3697_Write_Byte(unsigned char addr, unsigned char value)
int LM3697_Write_Byte(kal_uint8 addr,  kal_uint8 value)
{
	int ret = 0;
	struct i2c_client *client = I2C_LM3697;
	kal_uint8 write_data[2]={0};
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("Bl lm3697 Write I2C fail !!\n");
	return ret ;
}
EXPORT_SYMBOL(LM3697_Write_Byte);

int LM3697_Read_Byte(kal_uint8 addr, char  *value, int length)
{
         int ret = 0;
         struct i2c_client *client = I2C_LM3697;
         struct i2c_msg msg[2] = {
                   {
					   .addr = client->addr,
                       .buf = &addr,
                       .len = 1,
                   },
                   {
                       .addr = client->addr,
                       .flags = I2C_M_RD,
                       .buf = value,
                       .len = length,
                   },
         };
     ret = i2c_transfer(client->adapter, msg, 2);
         if(ret != 2) {
                   printk("Bl lm3697 Read I2C fail !!ret = %d\n",ret);
                   ret = -EIO;
		}else{
		ret = 0;
		}
    return ret ;
}
EXPORT_SYMBOL(LM3697_Read_Byte);

int lm3697_setbacklight(unsigned int level)
{
   // printk("%s level is %d\n", __func__, level);

	mt_set_gpio_mode(LCD_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BL_EN, GPIO_OUT_ONE);

    if (level == 0) {
        LM3697_Write_Byte(0x20,0x00);
	    LM3697_Write_Byte(0x21,level);
    } else {
        LM3697_Write_Byte(0x24,0x01);
		LM3697_Write_Byte(0x20,0x00);//Bank Level
		LM3697_Write_Byte(0x21,level);//Bank Level
		bl3697_esd_recovery_backlight_level =level;
    }
    printk("%s level is %d\n", __func__, level);
    return level;
}

static int __init BL_I2C_Init(void)
{
   i2c_register_board_info(BL_I2C_BUSNUM, &BL_board_info, 1);
   i2c_add_driver(&BL_I2C_driver);
   return 0;
}

static void __exit BL_I2C_Exit(void)
{
  i2c_del_driver(&BL_I2C_driver);
}

module_init(BL_I2C_Init);
module_exit(BL_I2C_Exit);

MODULE_AUTHOR("Liping-m");
MODULE_DESCRIPTION("MTK LM3697 I2C Driver");
MODULE_LICENSE("GPL");

#else

static struct mt_i2c_t I2C_Device;

int LM3697_Write_Byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    I2C_Device.id = BL_I2C_BUSNUM; //I2C1
    I2C_Device.addr = (LM3697_SLAVE_ADDR_WRITE >>1);//(SlaveID >> 1);
    I2C_Device.mode = ST_MODE;
    I2C_Device.speed = 100;
    len = 2;

    ret_code = i2c_write(&I2C_Device, write_data, len);

    return ret_code;
}

int lm3697_setbacklight(unsigned int level)
{
    //dprintf(0, "%s level is %d\n", __func__, level);

    if (level == 0) {
        LM3697_Write_Byte(0x20,0x00);
	    LM3697_Write_Byte(0x21,level);
    } else {
        LM3697_Write_Byte(0x24,0x01);
		LM3697_Write_Byte(0x20,0x00);//Bank Level
		LM3697_Write_Byte(0x21,level);//Bank Level
    }

    return level;
}

#endif
