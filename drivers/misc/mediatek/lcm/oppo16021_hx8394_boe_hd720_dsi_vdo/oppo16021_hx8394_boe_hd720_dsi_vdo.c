/*****************************************************************
* Copyright (c) 2004-2014 OPPO Mobile communication Corp.ltd.,
* VENDOR_EDIT
* Description: Source file for LCM driver IC.
* Version   : 1.0
* Date      : 2015-12-23
* Author    : liping-m@PhoneSW.Multimedia
*----------------------Revision History--------------------------
* None.
*****************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
//#include <linux/oppo_devices_list.h>
#include <soc/oppo/device_info.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h> 
	#include <platform/mt_pmic.h>
	#include <string.h>
    #include <platform/boot_mode.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>
    #include <mach/upmu_common.h>
    #include <mach/mt_boot_common.h>
#endif
#include <cust_gpio_usage.h>
#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_i2c.h>
#endif
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif


static const unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util;

static const unsigned char LCD_MODULE_ID = 0x00;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))
#define UDELAY(n) 											(lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 
#define dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update)

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH  	    (720)
#define FRAME_HEIGHT 		(1280)
#define PHYSICAL_WIDTH      (68)
#define PHYSICAL_HEIGHT     (122)

//#ifndef FPGA_EARLY_PORTING
//#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
//#endif

#define GPIO_65132_ENP (GPIO12 | 0x80000000)
#define GPIO_65132_ENN (GPIO5 | 0x80000000)
#define LCD_BL_EN (GPIO11 | 0x80000000)
#define GPIO_LCM_RESET_PIN  (GPIO158| 0x80000000)


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif


extern int LM3630A_Write_Byte(kal_uint8 addr,  kal_uint8 value);
extern int LCMPower_Write_Byte(kal_uint8 addr,  kal_uint8 value);

#ifdef VENDOR_EDIT/* OPPO 2016-01-29 sjc Modify for charging */
extern void oppo_chg_set_led_status(bool val);
#endif /* VENDOR_EDIT */


#define REGFLAG_DELAY      0xFC
#define REGFLAG_UDELAY     0xFB

#define REGFLAG_END_OF_TABLE   0xFD   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW  0xFE
#define REGFLAG_RESET_HIGH  0xFF


struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

//update initial param for IC hx8394 
static struct LCM_setting_table lcm_initialization_video_setting[] = {		
	{0XB9,3,{0XFF,0X83,0X94}},

	{0XBA,6,{0X63,0X03,0X68,0X6B,0XB2,0XC0}},

	{0XB1,10,{0X48,0X0B,0X6B,0X09,0X33,0X54,0X71,0X71,0X2C,0X43}},

	{0XCC,1,{0X0B}},
	{0XCC,1,{0X07}},

	{0XE0,58,{0X00,0X04,0X0c,0X12,0X14,0X18,0X1B,0X19,0X33,0X42,0X54,0X54,0X5B,0X71,0X79,0X7B,0X89,0X8C,0X88,0X97,0XA8,0X53,0X53,0X58,0X5D,0X62,0X6B,0X78,0X7F,0X00,0X04,0X0c,0X12,0X14,0X18,0X1B,0X19,0X33,0X42,0X54,0X54,0X5B,0X71,0X79,0X7B,0X89,0X8C,0X88,0X97,0XA8,0X53,0X53,0X58,0X5D,0X62,0X6B,0X78,0X7F}},

	{REGFLAG_DELAY,20,{}},

	{0XD2,1,{0X66}}, 

	{0XB2,6,{0X00,0X80,0X64,0X0C,0X06,0X2F}},

	{0XB4,22,{0X21,0X7F,0X21,0X7F,0X21,0X7F,0X01,0X0C,0X84,0X75,0X00,0X3F,0X21,0X7F,0X21,0X7F,0X21,0X7F,0X01,0X0C,0X84,0X3F}},

	{0XD3,33,{0X00,0X00,0X00,0X00,0X00,0X00,0X08,0X08,0X32,0X10,0X05,0X00,0X05,0X32,0X13,0XC1,0X00,0X01,0X32,0X10,0X08,0X00,0X00,0X37,0X03,0X07,0X07,0X37,0X05,0X05,0X37,0X0C,0X40}},

	{0XD5,44,{0X18,0X18,0X18,0X18,0X22,0X23,0X20,0X21,0X04,0X05,0X06,0X07,0X00,0X01,0X02,0X03,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X19,0X19,0X19,0X19}},

	{0XD6,44,{0X18,0X18,0X19,0X19,0X21,0X20,0X23,0X22,0X03,0X02,0X01,0X00,0X07,0X06,0X05,0X04,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X19,0X19,0X18,0X18}},

	{0XC0,2,{0X1F,0X73}},

	{0XB6,2,{0X90,0X90}},
	  
	{0XD4,1,{0X02}},

	{0XBD,1,{0X01}},

	{0XB1,1,{0X00}},

	{0XBD,1,{0X00}},
	 
	{0x11,1,{0x00}},		
	{REGFLAG_DELAY, 150, {}},
			
	{0x29,1,{0x00}},	
	{REGFLAG_DELAY,10,{}},	
	{REGFLAG_END_OF_TABLE,0x00,{}}	 
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 2, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}},
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_level_setting[] = {
    {0x55, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;
				
			case REGFLAG_UDELAY :
				UDELAY(table[i].count);
				break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void push_table22(void *handle,struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;
				
			case REGFLAG_UDELAY :
				UDELAY(table[i].count);
				break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V22(handle, cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

#ifndef BUILD_LK
//extern LCD_DEV lcd_dev;
#endif

static void lcm_get_params(LCM_PARAMS *params)
{
    int boot_mode = 0;
    //hw_version_identify();

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

    params->physical_width = PHYSICAL_WIDTH;
    params->physical_height = PHYSICAL_HEIGHT;
 

       // params->dsi.mode   = CMD_MODE;// NON IPC;
	   params->dsi.mode   = BURST_VDO_MODE; //BURST_VDO_MODE;
       // params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
       // params->dbi.te_edge_polarity = LCM_POLARITY_RISING;


	/* Command mode setting */
	params->dsi.LANE_NUM				=LCM_FOUR_LANE; //LCM_THREE_LANE; //LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      		= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 16;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 24;
	params->dsi.horizontal_backporch				= 100;
	params->dsi.horizontal_frontporch				= 50;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

 	params->dsi.LPX=5;  
	params->dsi.PLL_CLOCK = 241; //327; //247;  //220; //465;
    params->dsi.ssc_disable=1;//MUST Disable SSC

	//clk continuous video mode
	params->dsi.cont_clock=0;
	
	params->dsi.clk_lp_per_line_enable =0;
    if (g_boot_mode == META_BOOT) {
        boot_mode++;
        LCD_DEBUG("META_BOOT\n");
    }
    if (g_boot_mode == ADVMETA_BOOT) {
        boot_mode++;
        LCD_DEBUG("ADVMETA_BOOT\n");
    }
    if (g_boot_mode == ATE_FACTORY_BOOT) {
        boot_mode++;
        LCD_DEBUG("ATE_FACTORY_BOOT\n");
    }
    if (g_boot_mode == FACTORY_BOOT) {
        boot_mode++;
        LCD_DEBUG("FACTORY_BOOT\n");
    }
    if (boot_mode == 0) {
        LCD_DEBUG("neither META_BOOT or FACTORY_BOOT\n");
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable = 0;
		//params->dsi.lcm_esd_check_table[0].cmd			= 0x09; 
		//params->dsi.lcm_esd_check_table[0].count		= 2; 
		//params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
		//params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
    }

#ifndef BUILD_LK
    //lcd_dev = LCD_JDI_R63417_VIDEO;
	register_device_proc("lcd", "hx8394f", "16021 boe video mode");	
#endif	
}



static void lcm_init_power(void)
{
    LCD_DEBUG("[soso] lcm_init_power\n");

	//For LCM 5V Power
	LCMPower_Write_Byte(0x00,0x0F);//+5.5V
	LCMPower_Write_Byte(0x03,0x0F);//DIS Address
	LCMPower_Write_Byte(0x01,0x0F);//-5.5V
	LCMPower_Write_Byte(0xFF,0xF0);//DIS Address
	
    // FOR LCM +5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);

	MDELAY(7);
	
    //LCD RESET
    mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
    MDELAY(1);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(2);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(5);

    // FOR LCM -5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
	MDELAY(10); //> 5ms


}

static void lcm_suspend_power(void)
{
    LCD_DEBUG("[soso] lcm_suspend_power\n");
    //For BL
	mt_set_gpio_mode(LCD_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BL_EN, GPIO_OUT_ZERO);
    LCD_DEBUG(" lcm_suspend_power BL has disabled\n");

    //LCD RESET
    mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(2);
    // FOR LCM +5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);
    MDELAY(10);

    // FOR LCM -5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
  
}

static void lcm_resume_power(void)
{
    LCD_DEBUG("[soso] lcm_resume_power \n");
	lcm_init_power();
}


static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;

    LCD_DEBUG("[soso]lcm_initialization_setting\n");


    // when phone initial , config output high, enable backlight drv chip 
   // push_table(lcm_initialization_cmd_setting, sizeof(lcm_initialization_cmd_setting) / sizeof(struct LCM_setting_table), 1);
     push_table(lcm_initialization_video_setting, sizeof(lcm_initialization_video_setting) / sizeof(struct LCM_setting_table), 1);  

    //Enable BL
    mt_set_gpio_mode(LCD_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BL_EN, GPIO_OUT_ONE);
	MDELAY(2); //> 1ms

	//LM3630A_Write_Byte(0x01,0x19);  //open PWM
    LM3630A_Write_Byte(0x01,0x18);  //close PWM
    	
    LM3630A_Write_Byte(0x00,0x1F);  //linear mode
	LM3630A_Write_Byte(0x05,0x14);
	LM3630A_Write_Byte(0x06,0x14);

	//LM3630A_Write_Byte(0x03,0xFF);//Bank Level
	//LM3630A_Write_Byte(0x04,0xFF);//Bank Level
}


static void lcm_suspend(void)
{
	

	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);  
	
	//printk(KERN_ERR,"[SOSO]lcm_suspend IPC_I2C_Init\n");
	LCD_DEBUG("[soso] lcm_suspend \n");

#ifdef VENDOR_EDIT/* OPPO 2016-01-29 sjc Modify for charging */
	oppo_chg_set_led_status(false);
#endif /* VENDOR_EDIT */

}

static void lcm_resume(void)
{
    LCD_DEBUG("[soso] lcm_resume\n");
	lcm_init();

#ifdef VENDOR_EDIT/* OPPO 2016-01-29 sjc Modify for charging */
	oppo_chg_set_led_status(true);
#endif /* VENDOR_EDIT */
	
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
#if 1
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);	
#endif
}


static unsigned int lcm_compare_id(void)
{
#if 0
    unsigned char LCD_ID_value = 0;
    LCD_ID_value = which_lcd_module_triple();
    if(LCD_MODULE_ID == LCD_ID_value)
    {
        return 1;
    }
    else
    {
        return 0;
    }
#endif
}

int  boe_lcm_set_cabc_mode(void *handle,unsigned int level)
{
	// Refresh value of cabc level.	
    int mapped_level = 0;
	
    if (level==1) {
        mapped_level = 0x01;
    } else if(level ==2) {
        mapped_level = 0x02;
    }else if(level ==3){
		mapped_level = 0x03;
	}else{
		mapped_level = 0x00;
	}
	printk("boe_lcm_set_cabc_mode = %d\n",level);
    lcm_cabc_level_setting[0].para_list[0] = mapped_level;
    push_table22(handle,lcm_cabc_level_setting, sizeof(lcm_cabc_level_setting) / sizeof(struct LCM_setting_table), 1);

    return 0;	
}

// return TRUE: need recovery
// return FALSE: No need recovery

static unsigned int lcm_esd_check(void)
{



	return FALSE;//No ESD
}

static void lcm_setbacklight(unsigned int level)
{
	// Refresh value of backlight level.
#ifdef BUILD_LK
	dprintf(CRITICAL, "%s [soso] level is %d\n", __func__, level);
#else
    printk("%s [soso] level is %d\n", __func__, level);
#endif

    if (level == 0) {
        LM3630A_Write_Byte(0x03,level);
	    LM3630A_Write_Byte(0x04,level);
        LM3630A_Write_Byte(0x00,0x9D);
    } else {	
    	LM3630A_Write_Byte(0x00,0x1F);
		LM3630A_Write_Byte(0x04,00);//Bank Level
    	LM3630A_Write_Byte(0x03,level);//Bank Level
    }
}


LCM_DRIVER oppo16021_hx8394_boe_hd720_dsi_vdo_lcm_drv=
{
    .name           	= "oppo16021_hx8394_boe_hd720_dsi_vdo",
    .set_util_funcs 	= lcm_set_util_funcs,
    .get_params     	= lcm_get_params,
    .init           	= lcm_init,
    .suspend        	= lcm_suspend,
    .resume         	= lcm_resume,
//     .compare_id     	= lcm_compare_id,
     .init_power		= lcm_init_power,
     .resume_power = lcm_resume_power,
     .suspend_power = lcm_suspend_power,
//     .esd_check = lcm_esd_check,
     .set_backlight = lcm_setbacklight,
//	 .ata_check		= lcm_ata_check,
//#if (LCM_DSI_CMD_MODE)
	 .update         = lcm_update,
//#endif
};
