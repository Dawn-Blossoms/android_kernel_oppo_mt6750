/************************************************************************************
** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oppo77_12015\kernel\battery\battery
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      for dc-dc sn111008 charg
** 
** Version: 1.0
** Date created: 21:03:46,05/04/2012
** Author: Fanhong.Kong@ProDrv.CHG
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
************************************************************************************************************/
#ifdef CONFIG_OPPO_CHARGER_MTK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <asm/unaligned.h>

#include <linux/xlog.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>
#include <cust_eint.h>
#include <linux/power_supply.h>

#include <linux/wakelock.h>
#include <linux/gpio.h>

#include <mach/battery_meter.h>
#include <mach/charging.h>
#include <mach/battery_common.h>
#include <soc/oppo/device_info.h>

#else
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <soc/oppo/device_info.h>

#endif
#include "../oppo_charger.h"
#include "../oppo_gauge.h"
#include "../oppo_vooc.h"
#include "oppo_bq27541.h"


static struct i2c_client *new_client = NULL;

static struct bms_bq27541 *bq27541_di = NULL;

static DEFINE_MUTEX(bq27541_i2c_access);

/**********************************************************
  *
  *   [I2C Function For Read/Write bq27541] 
  *
  *********************************************************/
int bq27541_read_i2c(int cmd, int *returnData)
{
	if(!new_client) {
		pr_err(" new_client NULL,return\n");
		return 0;
	}
	if(cmd == BQ27541_BQ27411_CMD_INVALID)
		return 0;

    mutex_lock(&bq27541_i2c_access);
#ifdef CONFIG_OPPO_CHARGER_MTK
    new_client->timing = 300;
#endif
    *returnData = i2c_smbus_read_word_data(new_client,cmd);
    
    mutex_unlock(&bq27541_i2c_access); 
	//pr_err(" cmd = 0x%x, returnData = 0x%x\r\n",cmd,*returnData)  ; 
	if(*returnData < 0)
    	return 1;
	else
		return 0;
}

int bq27541_i2c_txsubcmd(int cmd, int writeData)
{
	if(!new_client) {
		pr_err(" new_client NULL,return\n");
		return 0;
	}
	if(cmd == BQ27541_BQ27411_CMD_INVALID)
		return 0;
		
    mutex_lock(&bq27541_i2c_access);
#ifdef CONFIG_OPPO_CHARGER_MTK
	new_client->timing = 300;
#endif
    i2c_smbus_write_word_data(new_client,cmd,writeData); 
    mutex_unlock(&bq27541_i2c_access);
    return 0;
}



/* OPPO 2013-08-24 wangjc Add begin for add adc interface. */
static int bq27541_get_battery_cc(void)//sjc20150105
{
	int ret;
	int cc = 0;

	if(!bq27541_di) {
		return 0;
	}
	if (atomic_read(&bq27541_di->suspended) == 1)
		return bq27541_di->cc_pre;

	if (oppo_vooc_get_allow_reading() == true) {
		ret = bq27541_read_i2c(bq27541_di->cmd_addr.reg_cc, &cc);
		if (ret) {
			dev_err(bq27541_di->dev, "error reading cc.\n");
			return ret;
		}
	} else {
		if(bq27541_di->cc_pre)
			return bq27541_di->cc_pre;
		else
			return 0;
	}
	
	bq27541_di->cc_pre = cc;
	return cc;
}

static int bq27541_get_battery_fcc(void)//sjc20150105
{
	int ret;
	int fcc = 0;

	if(!bq27541_di) {
		return 0;
	}
	if (atomic_read(&bq27541_di->suspended) == 1)
		return bq27541_di->fcc_pre;

	if (oppo_vooc_get_allow_reading() == true) {
		ret = bq27541_read_i2c(bq27541_di->cmd_addr.reg_fcc, &fcc);
		if (ret) {
			dev_err(bq27541_di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		if(bq27541_di->fcc_pre)
			return bq27541_di->fcc_pre;
		else
			return 0;
	}

	bq27541_di->fcc_pre = fcc;
	return fcc;
}

static int bq27541_get_battery_soh(void)//sjc20150105
{
	int ret;
	int soh = 0;

	if(!bq27541_di) {
		return 0;
	}
	if (atomic_read(&bq27541_di->suspended) == 1)
		return bq27541_di->soh_pre;

	if (oppo_vooc_get_allow_reading() == true) {
		ret = bq27541_read_i2c(bq27541_di->cmd_addr.reg_soh, &soh);
		if (ret) {
			dev_err(bq27541_di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		if(bq27541_di->soh_pre)
			return bq27541_di->soh_pre;
		else
			return 0;
	}

	bq27541_di->soh_pre = soh;
	return soh;
}


static int bq27541_soc_calibrate(int soc)
{
	unsigned int soc_calib;
	//int counter_temp = 0;
/*	
	if(!bq27541_di->batt_psy){	
		bq27541_di->batt_psy = power_supply_get_by_name("battery");
		bq27541_di->soc_pre = soc;
	}
*/	
	if(!bq27541_di) {
		return 0;
	}
	soc_calib = soc;
	
	if(soc >= 100)
		soc_calib = 100;
	else if(soc < 0)
		soc_calib = 0;
	bq27541_di->soc_pre = soc_calib;
	//pr_info("soc:%d, soc_calib:%d\n", soc, soc_calib);
	return soc_calib;
}


/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */



static void bq27541_cntl_cmd(int subcmd)
{
	bq27541_i2c_txsubcmd(BQ27541_BQ27411_REG_CNTL, subcmd);
}

static int bq27541_get_battery_mvolts(void)
{
	int ret;
	int volt = 0;

	if(!bq27541_di) {
		return 0;
	}
	if(atomic_read(&bq27541_di->suspended) == 1) {
		return bq27541_di->batt_vol_pre;
	}

	if(oppo_vooc_get_allow_reading() == true) {
		ret = bq27541_read_i2c(bq27541_di->cmd_addr.reg_volt, &volt);
		if (ret) {
			dev_err(bq27541_di->dev, "error reading voltage,ret:%d\n",ret);
			return bq27541_di->batt_vol_pre;
		}
	} else {
		return bq27541_di->batt_vol_pre;
	}

	bq27541_di->batt_vol_pre = volt * 1000;

	return volt * 1000;
}

static int bq27541_get_battery_temperature(void)
{
	int ret;
	int temp = 0;
	static int count = 0;

	if(!bq27541_di) {
		return 0;
	}
	if(atomic_read(&bq27541_di->suspended) == 1) {
		return bq27541_di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
	}

	if(oppo_vooc_get_allow_reading() == true) {
		ret = bq27541_read_i2c(bq27541_di->cmd_addr.reg_temp, &temp);
		if (ret) {
			count++;
			dev_err(bq27541_di->dev, "error reading temperature\n");
			if(count > 1) {
				count = 0;
				
				bq27541_di->temp_pre = -400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
				return -400;
			} else {
				return bq27541_di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
			}
		}
		count = 0;
	} else {
		return bq27541_di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;	
	}

	bq27541_di->temp_pre = temp;

	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

static int bq27541_get_batt_remaining_capacity(void)
{
	int ret;
	int cap = 0;

	if(!bq27541_di) {
		return 0;
	}
	if(atomic_read(&bq27541_di->suspended) == 1) {
		return bq27541_di->rm_pre;
	}
	if(oppo_vooc_get_allow_reading() == true) {
		ret = bq27541_read_i2c(bq27541_di->cmd_addr.reg_rm, &cap);
		if (ret) {
			dev_err(bq27541_di->dev, "error reading capacity.\n");
			return ret;
		}
	}
	bq27541_di->rm_pre = cap;
	return cap;
}

static int bq27541_get_battery_soc(void)
{
	int ret;
	int soc = 0;

	if(!bq27541_di) {
		return 50;
	}
	if(atomic_read(&bq27541_di->suspended) == 1) {
		return bq27541_di->soc_pre;
	}

	if(oppo_vooc_get_allow_reading() == true) {
		ret = bq27541_read_i2c(bq27541_di->cmd_addr.reg_soc, &soc);
		if (ret) {
			dev_err(bq27541_di->dev, "error reading soc.ret:%d\n",ret);
			goto read_soc_err;
		}
	} else {
		if(bq27541_di->soc_pre)
			return bq27541_di->soc_pre;
		else
			return 0;
	}

	soc = bq27541_soc_calibrate(soc);
	return soc;
	
read_soc_err:
	if(bq27541_di->soc_pre)
		return bq27541_di->soc_pre;
	else
		return 0;
}


static int bq27541_get_average_current(void)
{
	int ret;
	int curr = 0;

	if(!bq27541_di) {
		return 0;
	}
	if(atomic_read(&bq27541_di->suspended) == 1) {
		return -bq27541_di->current_pre;
	}

	if(oppo_vooc_get_allow_reading() == true) {
		ret = bq27541_read_i2c(bq27541_di->cmd_addr.reg_ai, &curr);
		if (ret) {
			dev_err(bq27541_di->dev, "error reading current.\n");
			return bq27541_di->current_pre;
		}
	} else {
		return -bq27541_di->current_pre;
	}
	// negative current
	if(curr&0x8000)
		curr = -((~(curr-1))&0xFFFF);
	bq27541_di->current_pre = curr;
	return -curr;
}

static bool bq27541_get_battery_authenticate(void)
{
	return true;
}

static void bq27541_set_battery_full(bool full)
{
	//Do nothing
}

static struct oppo_gauge_operations bq27541_batt_gauge = {
	.get_battery_mvolts		= bq27541_get_battery_mvolts,
	.get_battery_temperature	= bq27541_get_battery_temperature,
	.get_batt_remaining_capacity = bq27541_get_batt_remaining_capacity,
	.get_battery_soc			= bq27541_get_battery_soc,
	.get_average_current		= bq27541_get_average_current,
	.get_battery_fcc			= bq27541_get_battery_fcc,
	.get_battery_cc			= bq27541_get_battery_cc,
	.get_battery_soh			= bq27541_get_battery_soh,
	.get_battery_authenticate	= bq27541_get_battery_authenticate,
	.set_battery_full			= bq27541_set_battery_full,
};

static void gauge_set_cmd_addr(struct bms_bq27541 *di, int device_type)
{
	if(device_type == DEVICE_BQ27541){
		di->cmd_addr.reg_cntl = BQ27541_BQ27411_REG_CNTL;
		di->cmd_addr.reg_temp = BQ27541_REG_TEMP;
		di->cmd_addr.reg_volt = BQ27541_REG_VOLT;
		di->cmd_addr.reg_flags = BQ27541_REG_FLAGS;
		di->cmd_addr.reg_nac = BQ27541_REG_NAC;
		di->cmd_addr.reg_fac = BQ27541_REG_FAC;
		di->cmd_addr.reg_rm = BQ27541_REG_RM;
		di->cmd_addr.reg_fcc = BQ27541_REG_FCC;
		di->cmd_addr.reg_ai = BQ27541_REG_AI;
		di->cmd_addr.reg_si = BQ27541_REG_SI;
		di->cmd_addr.reg_mli = BQ27541_REG_MLI;
		di->cmd_addr.reg_ap = BQ27541_REG_AP;
		di->cmd_addr.reg_soc = BQ27541_REG_SOC;
		di->cmd_addr.reg_inttemp = BQ27541_REG_INTTEMP;
		di->cmd_addr.reg_soh = BQ27541_REG_SOH;
		di->cmd_addr.flag_dsc = BQ27541_FLAG_DSC;
		di->cmd_addr.flag_fc = BQ27541_FLAG_FC;
		di->cmd_addr.cs_dlogen = BQ27541_CS_DLOGEN;
		di->cmd_addr.cs_ss = BQ27541_CS_SS;

		di->cmd_addr.reg_ar = BQ27541_REG_AR;
		di->cmd_addr.reg_artte = BQ27541_REG_ARTTE;
		di->cmd_addr.reg_tte = BQ27541_REG_TTE;
		di->cmd_addr.reg_ttf = BQ27541_REG_TTF;
		di->cmd_addr.reg_stte = BQ27541_REG_STTE;
		di->cmd_addr.reg_mltte = BQ27541_REG_MLTTE;
		di->cmd_addr.reg_ae = BQ27541_REG_AE;
		di->cmd_addr.reg_ttecp = BQ27541_REG_TTECP;
		di->cmd_addr.reg_cc = BQ27541_REG_CC;
		di->cmd_addr.reg_nic = BQ27541_REG_NIC;
		di->cmd_addr.reg_icr = BQ27541_REG_ICR;
		di->cmd_addr.reg_logidx = BQ27541_REG_LOGIDX;
		di->cmd_addr.reg_logbuf = BQ27541_REG_LOGBUF;
		di->cmd_addr.reg_dod0 = BQ27541_REG_DOD0;

		di->cmd_addr.subcmd_cntl_status = BQ27541_SUBCMD_CTNL_STATUS;
		di->cmd_addr.subcmd_device_type = BQ27541_SUBCMD_DEVCIE_TYPE;
		di->cmd_addr.subcmd_fw_ver = BQ27541_SUBCMD_FW_VER;
		di->cmd_addr.subcmd_dm_code = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_prev_macw = BQ27541_SUBCMD_PREV_MACW;
		di->cmd_addr.subcmd_chem_id = BQ27541_SUBCMD_CHEM_ID;
		di->cmd_addr.subcmd_set_hib = BQ27541_SUBCMD_SET_HIB;
		di->cmd_addr.subcmd_clr_hib = BQ27541_SUBCMD_CLR_HIB;
		di->cmd_addr.subcmd_set_cfg = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_sealed = BQ27541_SUBCMD_SEALED;
		di->cmd_addr.subcmd_reset = BQ27541_SUBCMD_RESET;
		di->cmd_addr.subcmd_softreset = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_exit_cfg = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_enable_dlog = BQ27541_SUBCMD_ENABLE_DLOG;
		di->cmd_addr.subcmd_disable_dlog = BQ27541_SUBCMD_DISABLE_DLOG;
		di->cmd_addr.subcmd_enable_it = BQ27541_SUBCMD_ENABLE_IT;
		di->cmd_addr.subcmd_disable_it = BQ27541_SUBCMD_DISABLE_IT;

		di->cmd_addr.subcmd_hw_ver = BQ27541_SUBCMD_HW_VER;
		di->cmd_addr.subcmd_df_csum = BQ27541_SUBCMD_DF_CSUM;
		di->cmd_addr.subcmd_bd_offset = BQ27541_SUBCMD_BD_OFFSET;
		di->cmd_addr.subcmd_int_offset = BQ27541_SUBCMD_INT_OFFSET;
		di->cmd_addr.subcmd_cc_ver = BQ27541_SUBCMD_CC_VER;
		di->cmd_addr.subcmd_ocv = BQ27541_SUBCMD_OCV;
		di->cmd_addr.subcmd_bat_ins = BQ27541_SUBCMD_BAT_INS;
		di->cmd_addr.subcmd_bat_rem = BQ27541_SUBCMD_BAT_REM;
		di->cmd_addr.subcmd_set_slp = BQ27541_SUBCMD_SET_SLP;
		di->cmd_addr.subcmd_clr_slp = BQ27541_SUBCMD_CLR_SLP;
		di->cmd_addr.subcmd_fct_res = BQ27541_SUBCMD_FCT_RES;
		di->cmd_addr.subcmd_cal_mode = BQ27541_SUBCMD_CAL_MODE;
	} else {		//device_bq27411
		di->cmd_addr.reg_cntl = BQ27411_REG_CNTL;
		di->cmd_addr.reg_temp = BQ27411_REG_TEMP;
		di->cmd_addr.reg_volt = BQ27411_REG_VOLT;
		di->cmd_addr.reg_flags = BQ27411_REG_FLAGS;
		di->cmd_addr.reg_nac = BQ27411_REG_NAC;
		di->cmd_addr.reg_fac = BQ27411_REG_FAC;
		di->cmd_addr.reg_rm = BQ27411_REG_RM;
		di->cmd_addr.reg_fcc = BQ27411_REG_FCC;
		di->cmd_addr.reg_ai = BQ27411_REG_AI;
		di->cmd_addr.reg_si = BQ27411_REG_SI;
		di->cmd_addr.reg_mli = BQ27411_REG_MLI;
		di->cmd_addr.reg_ap = BQ27411_REG_AP;
		di->cmd_addr.reg_soc = BQ27411_REG_SOC;
		di->cmd_addr.reg_inttemp = BQ27411_REG_INTTEMP;
		di->cmd_addr.reg_soh = BQ27411_REG_SOH;
		di->cmd_addr.flag_dsc = BQ27411_FLAG_DSC;
		di->cmd_addr.flag_fc = BQ27411_FLAG_FC;
		di->cmd_addr.cs_dlogen = BQ27411_CS_DLOGEN;
		di->cmd_addr.cs_ss = BQ27411_CS_SS;
		/*bq27541 external standard cmds*/
		di->cmd_addr.reg_ar = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_artte = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_tte = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_ttf = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_stte = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_mltte = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_ae = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_ttecp = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_cc = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_nic = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_icr = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_logidx = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_logbuf = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.reg_dod0 = BQ27541_BQ27411_CMD_INVALID;

		di->cmd_addr.subcmd_cntl_status = BQ27411_SUBCMD_CNTL_STATUS;
		di->cmd_addr.subcmd_device_type = BQ27411_SUBCMD_DEVICE_TYPE;
		di->cmd_addr.subcmd_fw_ver = BQ27411_SUBCMD_FW_VER;
		di->cmd_addr.subcmd_dm_code = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_prev_macw = BQ27411_SUBCMD_PREV_MACW;
		di->cmd_addr.subcmd_chem_id = BQ27411_SUBCMD_CHEM_ID;
		di->cmd_addr.subcmd_set_hib = BQ27411_SUBCMD_SET_HIB;
		di->cmd_addr.subcmd_clr_hib = BQ27411_SUBCMD_CLR_HIB;
		di->cmd_addr.subcmd_set_cfg = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_sealed = BQ27411_SUBCMD_SEALED;
		di->cmd_addr.subcmd_reset = BQ27411_SUBCMD_RESET;
		di->cmd_addr.subcmd_softreset = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_exit_cfg = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_enable_dlog = BQ27411_SUBCMD_ENABLE_DLOG;
		di->cmd_addr.subcmd_disable_dlog = BQ27411_SUBCMD_DISABLE_DLOG;
		di->cmd_addr.subcmd_enable_it = BQ27411_SUBCMD_ENABLE_IT;
		di->cmd_addr.subcmd_disable_it = BQ27411_SUBCMD_DISABLE_IT;
		/*bq27541 external sub cmds*/
		di->cmd_addr.subcmd_hw_ver = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_df_csum = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_bd_offset = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_int_offset = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_cc_ver = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_ocv = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_bat_ins = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_bat_rem = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_set_slp = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_clr_slp = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_fct_res = BQ27541_BQ27411_CMD_INVALID;
		di->cmd_addr.subcmd_cal_mode = BQ27541_BQ27411_CMD_INVALID;
	}
}

static void bq27541_hw_config(struct bms_bq27541 *di)
{
	int ret = 0, flags = 0, device_type = 0, fw_ver = 0;

	bq27541_cntl_cmd(BQ27541_BQ27411_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27541_read_i2c(BQ27541_BQ27411_REG_CNTL, &flags);
	udelay(66);
	ret = bq27541_read_i2c(BQ27541_BQ27411_REG_CNTL, &flags);
	if (ret < 0) {
		di->device_type = DEVICE_BQ27541;
		pr_err(" error reading register %02x ret = %d\n",
			  BQ27541_BQ27411_REG_CNTL, ret);
		return ;
	}
	udelay(66);
	bq27541_cntl_cmd(BQ27541_BQ27411_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27541_cntl_cmd(BQ27541_BQ27411_SUBCMD_DEVICE_TYPE);
	udelay(66);
	bq27541_read_i2c(BQ27541_BQ27411_REG_CNTL, &device_type);
	
	udelay(66);
	bq27541_cntl_cmd(BQ27541_BQ27411_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27541_cntl_cmd(BQ27541_BQ27411_SUBCMD_FW_VER);
	udelay(66);
	bq27541_read_i2c(BQ27541_BQ27411_REG_CNTL, &fw_ver);

	if(device_type == DEVICE_TYPE_BQ27411) {
		di->device_type = DEVICE_BQ27411;
	} else {
		di->device_type = DEVICE_BQ27541;
		bq27541_cntl_cmd(BQ27541_BQ27411_SUBCMD_CTNL_STATUS);
		udelay(66);
		bq27541_cntl_cmd(BQ27541_BQ27411_SUBCMD_ENABLE_IT);	
	}
	gauge_set_cmd_addr(di, di->device_type);
	dev_err(di->dev, "DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION is 0x%02X\n",
			device_type, fw_ver);
}

static int sealed(void)
{
	//return control_cmd_read(di, CONTROL_STATUS) & (1 << 13);
	int value = 0;
	
	bq27541_cntl_cmd(CONTROL_STATUS);
	//bq27541_cntl_cmd(di,CONTROL_STATUS);
	msleep(10);
	bq27541_read_i2c(CONTROL_STATUS, &value);
	chg_debug(" REG_CNTL: 0x%x\n", value);

	if (bq27541_di->device_type == DEVICE_BQ27541)
		return value & BIT(14);
	else if (bq27541_di->device_type == DEVICE_BQ27411)
		return value & BIT(13);
	else 
		return 1;
}

static int seal(void)
{
	int i = 0;

	if(sealed()){
		pr_err("bq27541/27411 sealed,return\n");
		return 1;
	}
	bq27541_cntl_cmd(SEAL_SUBCMD);
	msleep(10);
	for(i = 0;i < SEAL_POLLING_RETRY_LIMIT;i++){
		if (sealed())
			return 1;
		msleep(10);
	}
	return 0;
}


static int unseal(u32 key)
{
	int i = 0;

	if (!sealed())
		goto out;
	
	if(bq27541_di->device_type == DEVICE_BQ27541){
		//bq27541_write(CONTROL_CMD, key & 0xFFFF, false, di);
		bq27541_cntl_cmd(0x1115);
		msleep(10);
		//bq27541_write(CONTROL_CMD, (key & 0xFFFF0000) >> 16, false, di);
		bq27541_cntl_cmd(0x1986);
		msleep(10);
	} 
	else if(bq27541_di->device_type == DEVICE_BQ27411){
	//bq27541_write(CONTROL_CMD, key & 0xFFFF, false, di);
		bq27541_cntl_cmd(0x8000);
		msleep(10);
		//bq27541_write(CONTROL_CMD, (key & 0xFFFF0000) >> 16, false, di);
		bq27541_cntl_cmd(0x8000);
		msleep(10);
	}
	bq27541_cntl_cmd(0xffff);
	msleep(10);
	bq27541_cntl_cmd(0xffff);
	msleep(10);

	while (i < SEAL_POLLING_RETRY_LIMIT) {
		i++;
		if (!sealed())
			break;
		msleep(10);
	}

out:
	chg_debug( "bq27541 : i=%d\n", i);

	if ( i == SEAL_POLLING_RETRY_LIMIT) {
		pr_err("bq27541 failed\n");
		return 0;
	} else {
		return 1;
	}
}

static void register_gauge_devinfo(struct bms_bq27541 *di)
{
	int ret = 0;
	char *version;
	char *manufacture;
	
	switch (di->device_type) {
		case DEVICE_BQ27541:
			version = "bq27541";
			manufacture = "TI";
			break;
		case DEVICE_BQ27411:
			version = "bq27411";
			manufacture = "TI";
			break;
		default:
			version = "unknown";
			manufacture = "UNKNOWN";
			break;
	}

	ret = register_device_proc("gauge", version, manufacture);
	if (ret)
		pr_err("register_gauge_devinfo fail\n");
}

static void bq27541_reset(struct i2c_client *client)
{
	int ui_soc = oppo_chg_get_ui_soc();
	
	if (bq27541_batt_gauge.get_battery_mvolts() <= 3300 * 1000 
			&& bq27541_batt_gauge.get_battery_mvolts() > 2500 * 1000
			&& ui_soc == 0 
			&& bq27541_batt_gauge.get_battery_temperature() > 150) {
		if (!unseal(BQ27541_UNSEAL_KEY)) {
			pr_err("bq27541 unseal fail !\n");
			return;
		}
		chg_debug( "bq27541 unseal OK vol = %d,ui_soc = %d,temp = %d!\n",bq27541_batt_gauge.get_battery_mvolts(),ui_soc,bq27541_batt_gauge.get_battery_temperature());
		
		if (bq27541_di->device_type == DEVICE_BQ27541)
			bq27541_cntl_cmd(BQ27541_RESET_SUBCMD);
		else if (bq27541_di->device_type == DEVICE_BQ27411)
			bq27541_cntl_cmd(BQ27411_RESET_SUBCMD);//27411
		msleep(50);

		if (bq27541_di->device_type == DEVICE_BQ27411){
			if (!seal())
				pr_err("bq27411 seal fail\n");
		}
		msleep(150);
		chg_debug("bq27541_reset,point = %d\r\n",bq27541_batt_gauge.get_battery_soc());
	}
	return;
}


static int bq27541_resume(struct i2c_client *client)
{	
	atomic_set(&bq27541_di->suspended, 0);
	bq27541_get_battery_soc();
	return 0;
}

static int bq27541_suspend(struct i2c_client *client, pm_message_t mesg)
{
	atomic_set(&bq27541_di->suspended, 1);
	return 0;
}

static int bq27541_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
	struct bms_bq27541 *di;
	struct oppo_gauge_chip	*chip; 

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}
	
	i2c_set_clientdata(client, di);	
	di->dev = &client->dev;
	di->client = client;
	atomic_set(&di->suspended, 0);
	new_client = client;
	
	bq27541_hw_config(di);
#if 0
	INIT_DELAYED_WORK(&di->hw_config, bq27541_hw_config);
	schedule_delayed_work(&di->hw_config, 0);
#endif
	di->soc_pre = 50;
	di->batt_vol_pre = 3800000;
	di->current_pre = 999;
	bq27541_di = di;
	
	chip = devm_kzalloc(&client->dev,
			sizeof(struct oppo_gauge_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("kzalloc() failed.\n");
		new_client = NULL;
		bq27541_di = NULL;
		return -ENOMEM;
	}
		
	chip->client = client;
    chip->dev = &client->dev;
	chip->gauge_ops = &bq27541_batt_gauge;
	chip->device_type = bq27541_di->device_type;
	oppo_gauge_init(chip);
	register_gauge_devinfo(di);
	
	chg_debug(" success\n");
	return 0;                                                                                       

}
/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/
 

static const struct of_device_id bq27541_match[] = {
	{ .compatible = "oppo,bq27541-battery"},
	{ },
};

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-battery", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27541_id);


static struct i2c_driver bq27541_i2c_driver = {
	.driver		= {
		.name = "bq27541-battery",
		.owner	= THIS_MODULE,
		.of_match_table = bq27541_match,
	},
	.probe		= bq27541_driver_probe,
	.shutdown	= bq27541_reset,
	.resume 	= bq27541_resume,
	.suspend	= bq27541_suspend,
	.id_table	= bq27541_id,
};
/*----------------------------------------------------------------------------*/
/*static void  bq27541_exit(void)
{
	i2c_del_driver(&bq27541_i2c_driver);
}*/

/*----------------------------------------------------------------------------*/

module_i2c_driver(bq27541_i2c_driver);
MODULE_DESCRIPTION("Driver for bq27541 charger chip");
MODULE_LICENSE("GPL v2");

