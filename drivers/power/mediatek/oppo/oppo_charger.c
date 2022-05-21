/**********************************************************************************
* Copyright (c)  2008-2015  Guangdong OPPO Mobile Comm Corp., Ltd
* VENDOR_EDIT
* Description: Charger IC management module for charger system framework.
*              Manage all charger IC and define abstarct function flow.
* Version   : 1.0
* Date      : 2015-06-22
* Author    : fanhui@PhoneSW.BSP
* 			: Fanhong.Kong@ProDrv.CHG		   	
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    fanhui@PhoneSW.BSP    			Created for new architecture
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
* Revision 1.1    2016-03-07    wenbin.liu@SW.Bsp.Driver   		edit for log optimize
***********************************************************************************/
#include <linux/delay.h>
#include <linux/power_supply.h>	
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#ifdef CONFIG_OPPO_CHARGER_MTK
#include <mach/mt_boot.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#else
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/rtc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#include <linux/msm_bcl.h>
#include <linux/ktime.h>
#include <linux/kernel.h>
#endif

#include "oppo_charger.h"
#include "oppo_gauge.h"
#include "oppo_vooc.h"

static struct oppo_chg_chip *g_charger_chip = NULL;

#define OPPO_CHG_UPDATE_INTERVAL_SEC	5
#define OPPO_CHG_UPDATE_INIT_DELAY	round_jiffies_relative(msecs_to_jiffies(500))	/* first run after init 10s */
#define OPPO_CHG_UPDATE_INTERVAL	round_jiffies_relative(msecs_to_jiffies(OPPO_CHG_UPDATE_INTERVAL_SEC*1000))	/* update cycle 5s */

#define OPPO_CHG_DEFAULT_CHARGING_CURRENT	512

#define CHG_NO_SUSPEND_NO_DISABLE					0
#define CHG_DISABLE										1
#define CHG_SUSPEND									2

int Enable_CHARGER_LOG = 2;



// wenbin.liu@SW.Bsp.Driver, 2016/03/01  Add for log tag 
#define charger_xlog_printk(num, fmt,...) \
  do { \
    if (Enable_CHARGER_LOG >= (int)num) { \
		printk(KERN_NOTICE pr_fmt("[OPPO_CHG][%s]"fmt),__func__,##__VA_ARGS__); \
    } \
  } while (0)




void (*synaptics_chg_mode_enable)(int enable, int is_fast_charge); //enable  0: no   1: slow  3:  quickly charge
static void oppo_chg_turn_off_charging(struct oppo_chg_chip *chip);
static void oppo_chg_turn_on_charging(struct oppo_chg_chip *chip);

static void oppo_chg_variables_init(struct oppo_chg_chip *chip);
static void oppo_chg_update_work(struct work_struct *work);
static void oppo_chg_protection_check(struct oppo_chg_chip *chip);
static void oppo_chg_get_battery_data(struct oppo_chg_chip *chip);
static void oppo_chg_check_tbatt_status(struct oppo_chg_chip *chip);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void chg_early_suspend(struct early_suspend *h);
static void chg_late_resume(struct early_suspend *h);
#elif CONFIG_FB
static int fb_notifier_callback(struct notifier_block *nb, unsigned long event, void *data);
#endif
/****************************************/
static int reset_mcu_delay = 0;
static bool vbatt_higherthan_4180mv = false;

#ifdef CONFIG_OPPO_CHARGER_MTK
static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_OTG_SWITCH,
	POWER_SUPPLY_PROP_OTG_ONLINE,
};

static int usb_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct oppo_chg_chip *chip = container_of(psy, struct oppo_chg_chip, usb_psy);

	if(chip->charger_exist)
	{
		//if (chip->charger_type==STANDARD_HOST || chip->charger_type==CHARGING_HOST) {
		if(chip->charger_type == POWER_SUPPLY_TYPE_USB) {
			chip->usb_online = true;
			chip->usb_psy.type = POWER_SUPPLY_TYPE_USB;
		}
	} else {
		chip->usb_online = false;
		
	}
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:     
		val->intval = chip->usb_online;
		break;
	case POWER_SUPPLY_PROP_OTG_SWITCH:		
        val->intval = chip->otg_switch;
        break;
	case POWER_SUPPLY_PROP_OTG_ONLINE:		
        val->intval = chip->otg_online;
        break;	
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int usb_property_is_writeable(struct power_supply *psy,
    enum power_supply_property psp)
{
	switch (psp) {
    case POWER_SUPPLY_PROP_OTG_SWITCH:
        return 1;
    default:
        break;
    }
    
    return 0;
}

static int usb_set_property(struct power_supply *psy,
    enum power_supply_property psp,
    const union power_supply_propval *val)
{
	int ret = 0;
	struct oppo_chg_chip *chip = container_of(psy, struct oppo_chg_chip, usb_psy);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_OTG_SWITCH:
        if(val->intval == 1){
			chip->otg_switch = true;
		} else {
			chip->otg_switch = false;			
			chip->otg_online = false;
		}
		charger_xlog_printk(CHG_LOG_CRTI, " otg_switch:%d\n",chip->otg_switch);
        break;

	default:
        ret = -EINVAL;
        break;
	}
	return ret;
}

static void oppo_chg_usb_psy_init(struct oppo_chg_chip *chip)
{
	chip->usb_psy.name = "usb";
	chip->usb_psy.type = POWER_SUPPLY_TYPE_USB;
	chip->usb_psy.properties = usb_props;
	chip->usb_psy.num_properties = ARRAY_SIZE(usb_props);
	chip->usb_psy.get_property = usb_get_property;
	chip->usb_psy.set_property = usb_set_property;
    chip->usb_psy.property_is_writeable = usb_property_is_writeable;
}

static void usb_update(struct oppo_chg_chip *chip)
{
	if(chip->charger_exist)
	{
		//if (chip->charger_type==STANDARD_HOST || chip->charger_type==CHARGING_HOST) {
		if (chip->charger_type == POWER_SUPPLY_TYPE_USB) {
			chip->usb_online = true;
			chip->usb_psy.type = POWER_SUPPLY_TYPE_USB;
		}
	} else {
		chip->usb_online = false;
		
	}
	power_supply_changed(&chip->usb_psy);
}
#endif


static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,	
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_AUTHENTICATE,
	POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
	POWER_SUPPLY_PROP_BATTERY_REQUEST_POWEROFF,
	POWER_SUPPLY_PROP_CHARGE_TECHNOLOGY,
	POWER_SUPPLY_PROP_FAST_CHARGE,	
	POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE,	//add for MMI_CHG_TEST
	POWER_SUPPLY_PROP_BATTERY_FCC,
	POWER_SUPPLY_PROP_BATTERY_SOH,
	POWER_SUPPLY_PROP_BATTERY_CC,
	POWER_SUPPLY_PROP_BATTERY_RM,
	POWER_SUPPLY_PROP_BATTERY_NOTIFY_CODE,
#ifdef CONFIG_OPPO_CHARGER_MTK
	POWER_SUPPLY_PROP_adjust_power,
#else
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_FLASH_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_FLASH_ACTIVE,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_FORCE_TLIM,	
#endif
	POWER_SUPPLY_PROP_ADAPTER_FW_UPDATE,
#ifdef VENDOR_EDIT
/*chaoying.chen@EXP.BaseDrv.charge,2016/01/28 add internal capacity node for 15311 */	
	POWER_SUPPLY_PROP_INTERNAL_CAPACITY,
#endif //VENDOR_EDIT
	POWER_SUPPLY_PROP_VOOCCHG_ING,
};

static int ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct oppo_chg_chip *chip = container_of(psy, struct oppo_chg_chip, ac_psy);

	if(chip->charger_exist)
	{
		/*if ((chip->charger_type == NONSTANDARD_CHARGER) || (chip->charger_type == STANDARD_CHARGER) ||
			(chip->charger_type == APPLE_2_1A_CHARGER) || (chip->charger_type == APPLE_1_0A_CHARGER)  ||
			(chip->charger_type == APPLE_0_5A_CHARGER)) */
		if((chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP) || (oppo_vooc_get_fastchg_started() == true)
			|| (oppo_vooc_get_fastchg_to_normal() == true) || (oppo_vooc_get_fastchg_to_warm() == true)
			|| (oppo_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE) || (oppo_vooc_get_btb_temp_over() == true)) {
			chip->ac_online = true;
		} else {
			chip->ac_online = false;
		}
	}
	else
	{
		if((oppo_vooc_get_fastchg_started() == true) || (oppo_vooc_get_fastchg_to_normal() == true) 
			|| (oppo_vooc_get_fastchg_to_warm() == true) || (oppo_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE)
			|| (oppo_vooc_get_btb_temp_over() == true)) {
			chip->ac_online = true;
		} else {
			chip->ac_online = false;
		}
	}
#ifdef CONFIG_OPPO_CHARGER_MTK
	if(chip->ac_online)
		charger_xlog_printk(CHG_LOG_CRTI, "chg_exist:%d, ac_online:%d\n",chip->charger_exist,chip->ac_online);
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->ac_online;
		 break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}


static int battery_property_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	int rc = 0;
	
	switch (psp) {
	case POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE:
		rc = 1;
		break;
#ifndef CONFIG_OPPO_CHARGER_MTK
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
	case POWER_SUPPLY_PROP_FORCE_TLIM:
//	case POWER_SUPPLY_PROP_RERUN_AICL:
		rc = 1;
		break;
#endif
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int battery_set_property(struct power_supply *psy,
	enum power_supply_property psp,
	const union power_supply_propval *val)
{
	int ret = 0;
	struct oppo_chg_chip *chip = container_of(psy, struct oppo_chg_chip, batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE:
		if(val->intval == 0){
			chip->mmi_chg = 0;
			oppo_chg_turn_off_charging(chip);			
		}
		else {
			chip->mmi_chg = 1;
			oppo_chg_turn_on_charging(chip);
		}
		break;
#ifndef CONFIG_OPPO_CHARGER_MTK
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		chip->chg_ops->set_system_temp_level(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		ret = chip->chg_ops->otg_pulse_skip_disable(chip,
				REASON_FLASH_ENABLED, val->intval);
		break;
	case POWER_SUPPLY_PROP_FORCE_TLIM:
		ret = chip->chg_ops->tlim_en(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		ret = chip->chg_ops->set_dp_dm(chip, val->intval);
		break;
#endif
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int battery_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct oppo_chg_chip *chip = container_of(psy, struct oppo_chg_chip, batt_psy);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if(oppo_vooc_get_fastchg_started() == true || oppo_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (!chip->authenticate)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = chip->prop_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = oppo_chg_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->batt_exist;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->ui_soc;
		break;        
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CONFIG_OPPO_CHARGER_MTK
		val->intval = chip->batt_volt;
#else
		val->intval = chip->batt_volt * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		chip->icharging = oppo_gauge_get_batt_current();
        val->intval = chip->icharging;	
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chip->temperature;
		break;     
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = chip->charger_volt;
		break;
		
	case POWER_SUPPLY_PROP_AUTHENTICATE:		
        val->intval = chip->authenticate;	
		break;
	case POWER_SUPPLY_PROP_CHARGE_TIMEOUT:
		val->intval = chip->chging_over_time; 
		break;			
	case POWER_SUPPLY_PROP_BATTERY_REQUEST_POWEROFF:
		val->intval = chip->request_power_off; 
		break;
	case POWER_SUPPLY_PROP_CHARGE_TECHNOLOGY:
		val->intval = chip->vooc_project;
		break;
	case POWER_SUPPLY_PROP_FAST_CHARGE:
		val->intval = oppo_chg_show_vooc_logo_ornot();
#ifdef CONFIG_OPPO_CHARGER_MTK
		if(val->intval)
			charger_xlog_printk(CHG_LOG_CRTI, "vooc_logo:%d\n", val->intval);
#endif
		break;
	case POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE:	//add for MMI_CHG TEST
        val->intval = chip->mmi_chg;
        break;
	case POWER_SUPPLY_PROP_BATTERY_FCC:
		val->intval = chip->batt_fcc;
        break;
	case POWER_SUPPLY_PROP_BATTERY_SOH:
		val->intval = chip->batt_soh;
        break;
	case POWER_SUPPLY_PROP_BATTERY_CC:
		val->intval = chip->batt_cc;
        break;
	case POWER_SUPPLY_PROP_BATTERY_RM:
		val->intval = chip->batt_rm;
        break;
	case POWER_SUPPLY_PROP_BATTERY_NOTIFY_CODE:
		val->intval = chip->notify_code;
        break;
	case POWER_SUPPLY_PROP_ADAPTER_FW_UPDATE:
		val->intval = oppo_vooc_get_adapter_update_status();
		break;
#ifdef VENDOR_EDIT
/*chaoying.chen@EXP.BaseDrv.charge,2016/01/28 add internal capacity node for 15311 */		
	case POWER_SUPPLY_PROP_INTERNAL_CAPACITY:
		val->intval = chip->soc;
		break;
#endif //VENDOR_EDIT
	case POWER_SUPPLY_PROP_VOOCCHG_ING:
		val->intval = oppo_vooc_get_fastchg_ing();
		break;
#ifdef CONFIG_OPPO_CHARGER_MTK
	case POWER_SUPPLY_PROP_adjust_power:
		val->intval = -1;
		break;
#else
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->otg_pulse_skip_dis;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chip->pulse_cnt;
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = chip->chg_ops->calc_flash_current(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = chip->chg_ops->get_aicl_ma(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_FORCE_TLIM:
		val->intval = 0;
		break;
#endif
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void oppo_chg_ac_psy_init(struct oppo_chg_chip *chip)
{
	chip->ac_psy.name = "ac";
	chip->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->ac_psy.properties = ac_props;
	chip->ac_psy.num_properties = ARRAY_SIZE(ac_props);
	chip->ac_psy.get_property = ac_get_property;
}

static void oppo_chg_batt_psy_init(struct oppo_chg_chip *chip)
{
	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = battery_props;
	chip->batt_psy.num_properties = ARRAY_SIZE(battery_props);
	chip->batt_psy.get_property = battery_get_property;
	chip->batt_psy.set_property = battery_set_property;
	chip->batt_psy.property_is_writeable = battery_property_is_writeable;
#ifndef CONFIG_OPPO_CHARGER_MTK
	chip->previous_soc = -EINVAL;
	chip->batt_psy.external_power_changed = oppo_chg_external_power_changed;
#endif
}

static ssize_t chg_log_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char write_data[32] = {0};
	
	if (copy_from_user(&write_data, buff, len)) {
		chg_err("bat_log_write error.\n");
		return -EFAULT;
	}

	if (write_data[0] == '1') {
		charger_xlog_printk(CHG_LOG_CRTI, "enable battery driver log system\n");
		Enable_CHARGER_LOG = 1;
	} else if (write_data[0] == '2') {
		charger_xlog_printk(CHG_LOG_CRTI, "enable battery driver log system:2\n");
		Enable_CHARGER_LOG = 2;
	} else {
		charger_xlog_printk(CHG_LOG_CRTI, "Disable battery driver log system\n");
		Enable_CHARGER_LOG = 0;
	}

	return len;
}
static ssize_t chg_log_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[32] = {0};
	int len = 0;	

	if(Enable_CHARGER_LOG == 1)
		read_data[0] = '1';
	else if(Enable_CHARGER_LOG == 2)
		read_data[0] = '2';
	else
		read_data[0] = '0';
	
	len = sprintf(page,"%s",read_data);	
	if(len > *off)	   
		len -= *off;	
	else	   
		len = 0;
	if(copy_to_user(buff,page,(len < count ? len : count)))
	{	   
		return -EFAULT;
	}
	*off += len < count ? len : count;	
	return (len < count ? len : count);	
}

static const struct file_operations chg_log_proc_fops = {
	.write = chg_log_write,
	.read = chg_log_read,
};

static int init_proc_chg_log(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("charger_log", 0664, NULL, &chg_log_proc_fops);
	if (!p) 
		chg_err("proc_create chg_log_proc_fops fail!\n");

	return 0;
}

static ssize_t chg_cycle_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	char proc_chg_cycle_data[16];

	if (copy_from_user(&proc_chg_cycle_data, buff, count)) {
		chg_err("chg_cycle_write error.\n");
		return -EFAULT;
	}

	if (strncmp(proc_chg_cycle_data, "en808", 5) == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, "allow charging.\n");
		g_charger_chip->chg_ops->charger_unsuspend(g_charger_chip);
		g_charger_chip->chg_ops->charging_enable(g_charger_chip);
		g_charger_chip->mmi_chg = 1;
	} else if (strncmp(proc_chg_cycle_data, "dis808", 6) == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, "not allow charging.\n");
		g_charger_chip->chg_ops->charging_disable(g_charger_chip, false);
		g_charger_chip->chg_ops->charger_suspend(g_charger_chip);
		g_charger_chip->mmi_chg = 0;
	} else {
		return -EFAULT;
	}

	return count;
}

static const struct file_operations chg_cycle_proc_fops = {
	.write = chg_cycle_write,
	.llseek = noop_llseek,
};

static void init_proc_chg_cycle(void)
{
	if (!proc_create("charger_cycle", S_IWUSR | S_IWGRP | S_IWOTH, NULL, &chg_cycle_proc_fops));
		chg_err("proc_create chg_cycle_proc_fops fail!\n");
}

int oppo_chg_init(struct oppo_chg_chip *chip)
{
	int rc;
	
#ifndef CONFIG_OPPO_CHARGER_MTK	
	struct power_supply *usb_psy;
#endif

	if (!chip->chg_ops) {
		dev_err(chip->dev, "charger operations cannot be NULL\n");
		return -1;
	}
	oppo_chg_variables_init(chip);
	oppo_chg_get_battery_data(chip);
#ifdef CONFIG_OPPO_CHARGER_MTK	
	oppo_chg_usb_psy_init(chip);

	rc = power_supply_register(chip->dev, &chip->usb_psy);
    if (rc < 0) {
            dev_err(chip->dev, "power supply register usb_psy failed, errno: %d\n", rc);
            goto usb_psy_reg_failed;
    }
#else
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
        dev_err(chip->dev, "USB psy not found; deferring probe\n");
        //return -EPROBE_DEFER;
		goto usb_psy_reg_failed;
    }
	
	chip->usb_psy = usb_psy;
#endif	

	oppo_chg_ac_psy_init(chip);

	rc = power_supply_register(chip->dev, &chip->ac_psy);
	if (rc < 0) {
		dev_err(chip->dev, "power supply register dc_psy failed, errno: %d\n", rc);
		goto ac_psy_reg_failed;
	}
	

	oppo_chg_batt_psy_init(chip);

	rc = power_supply_register(chip->dev, &chip->batt_psy);
    if (rc < 0) {
            dev_err(chip->dev, "power supply register batt_psy failed, errno: %d\n", rc);
            goto batt_psy_reg_failed;
    }
#ifndef CONFIG_OPPO_CHARGER_MTK
	chip->psy_registered = true;
#endif
	g_charger_chip = chip;
	wake_lock_init(&chip->suspend_lock, WAKE_LOCK_SUSPEND, "battery suspend wakelock");

	INIT_DELAYED_WORK(&chip->update_work, oppo_chg_update_work);  
	
#if 0
#ifndef CONFIG_OPPO_CHARGER_MTK
	power_supply_set_present(chip->usb_psy, chip->chg_ops->check_chrdet_status());
#endif
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	chip->chg_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	chip->chg_early_suspend.suspend = chg_early_suspend;
	chip->chg_early_suspend.resume = chg_late_resume;
	register_early_suspend(&chip->chg_early_suspend);
#elif CONFIG_FB
	chip->chg_fb_notify.notifier_call = fb_notifier_callback;
	rc = fb_register_client(&chip->chg_fb_notify);
	if (rc)
		pr_err("Unable to register chg_fb_notify: %d\n", rc);
#endif

	init_proc_chg_log();
	init_proc_chg_cycle();
	schedule_delayed_work(&chip->update_work, OPPO_CHG_UPDATE_INIT_DELAY);
	charger_xlog_printk(CHG_LOG_CRTI, " end\n");

	return 0;
batt_psy_reg_failed:
	power_supply_unregister(&chip->ac_psy);
ac_psy_reg_failed:
#ifdef CONFIG_OPPO_CHARGER_MTK	
	power_supply_unregister(&chip->usb_psy);
#endif
usb_psy_reg_failed:
	return rc;

}


//--------------------------------------------------------
int oppo_chg_parse_dt(struct oppo_chg_chip *chip)
{

    int rc;
    struct device_node *node = chip->dev->of_node;
    int batt_cold_degree_negative,batt_removed_degree_negative;

    
    if (!node) {
        dev_err(chip->dev, "device tree info. missing\n");
        return -EINVAL;
    }

/*hardware init*/
	rc = of_property_read_u32(node, "qcom,input_current_charger_ma", &chip->limits.input_current_charger_ma);
    if (rc) {
        chip->limits.input_current_charger_ma = OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA;
    }

	rc = of_property_read_u32(node, "qcom,input_current_usb_ma", &chip->limits.input_current_usb_ma);
    if (rc) {
        chip->limits.input_current_usb_ma = OPCHG_INPUT_CURRENT_LIMIT_USB_MA;
    }

	rc = of_property_read_u32(node, "qcom,input_current_led_ma", &chip->limits.input_current_led_ma);
    if (rc) {
        chip->limits.input_current_led_ma = OPCHG_INPUT_CURRENT_LIMIT_LED_MA;
    }

	rc = of_property_read_u32(node, "qcom,input_current_led_ma_forcmcc", &chip->limits.input_current_led_ma_forcmcc);
    if (rc) {
        chip->limits.input_current_led_ma_forcmcc = 500;
    }

#ifdef OPPO_CMCC_TEST
	chip->limits.input_current_led_ma = chip->limits.input_current_led_ma_forcmcc;
#endif

	rc = of_property_read_u32(node, "qcom,input_current_camera_ma", &chip->limits.input_current_camera_ma);
    if (rc) {
        chip->limits.input_current_usb_ma = OPCHG_INPUT_CURRENT_LIMIT_CAMERA_MA;
    }
     
    chip->limits.iterm_disabled = of_property_read_bool(node, "qcom,iterm-disabled");
  
    rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->limits.iterm_ma);
    if (rc < 0) {
        chip->limits.iterm_ma = -EINVAL;
    }

    rc = of_property_read_u32(node, "qcom,recharge-mv", &chip->limits.recharge_mv);
    if (rc < 0) {
        chip->limits.recharge_mv = -EINVAL;
    }
	
	/*-19C*/	
	rc = of_property_read_u32(node, "qcom,removed_bat_decidegc", &batt_removed_degree_negative);
	if (rc < 0) {
        chip->limits.removed_bat_decidegc = -19;
    }
	else {
        chip->limits.removed_bat_decidegc = -batt_removed_degree_negative;
    }
	
	
/*-3~0 C*/	
	rc = of_property_read_u32(node, "qcom,cold_bat_decidegc", &batt_cold_degree_negative);
	if (rc < 0) {
        chip->limits.cold_bat_decidegc = -EINVAL;
    }
	else {
        chip->limits.cold_bat_decidegc = -batt_cold_degree_negative;
    }
	
	rc = of_property_read_u32(node, "qcom,temp_cold_vfloat_mv",&chip->limits.temp_cold_vfloat_mv);
	if(rc < 0)
		chg_err(" temp_cold_vfloat_mv fail\n");
	

	rc = of_property_read_u32(node, "qcom,temp_cold_fastchg_current_ma",&chip->limits.temp_cold_fastchg_current_ma);
	if(rc < 0)
		chg_err(" temp_cold_fastchg_current_ma fail\n");

/*0~5 C*/		
	rc = of_property_read_u32(node, "qcom,little_cold_bat_decidegc", &chip->limits.little_cold_bat_decidegc);
	if (rc < 0) {
		chip->limits.little_cold_bat_decidegc = -EINVAL;
	}
	
	rc = of_property_read_u32(node, "qcom,temp_little_cold_vfloat_mv", &chip->limits.temp_little_cold_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_little_cold_vfloat_mv = -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,temp_little_cold_fastchg_current_ma", &chip->limits.temp_little_cold_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.temp_little_cold_fastchg_current_ma = -EINVAL;
	}
	
/*5~12 C*/		
	rc = of_property_read_u32(node, "qcom,cool_bat_decidegc", &chip->limits.cool_bat_decidegc);
    if (rc < 0) {
        chip->limits.cool_bat_decidegc = -EINVAL;
    }
	
	rc = of_property_read_u32(node, "qcom,temp_cool_vfloat_mv", &chip->limits.temp_cool_vfloat_mv);
    if (rc < 0) {
        chip->limits.temp_cool_vfloat_mv = -EINVAL;
    }

    rc = of_property_read_u32(node, "qcom,temp_cool_fastchg_current_ma_high", &chip->limits.temp_cool_fastchg_current_ma_high);
    if (rc < 0) {
        chip->limits.temp_cool_fastchg_current_ma_high = -EINVAL;
    }

	rc = of_property_read_u32(node, "qcom,temp_cool_fastchg_current_ma_low", &chip->limits.temp_cool_fastchg_current_ma_low);
    if (rc < 0) {
        chip->limits.temp_cool_fastchg_current_ma_low = -EINVAL;
    }
	
/*12~16 C*/		
	rc = of_property_read_u32(node, "qcom,little_cool_bat_decidegc", &chip->limits.little_cool_bat_decidegc);
    if (rc < 0) {
        chip->limits.little_cool_bat_decidegc = -EINVAL;
    }
	
	rc = of_property_read_u32(node, "qcom,temp_little_cool_vfloat_mv", &chip->limits.temp_little_cool_vfloat_mv);
    if (rc < 0) {
        chip->limits.temp_little_cool_vfloat_mv = -EINVAL;
    }

    rc = of_property_read_u32(node, "qcom,temp_little_cool_fastchg_current_ma",
                            &chip->limits.temp_little_cool_fastchg_current_ma);
    if (rc < 0) {
        chip->limits.temp_little_cool_fastchg_current_ma = -EINVAL;
    }	
	
/*16~45 C*/		
	rc = of_property_read_u32(node, "qcom,normal_bat_decidegc",&chip->limits.normal_bat_decidegc);
	if(rc < 0)
		chg_err(" normal_bat_decidegc fail\n");

	rc = of_property_read_u32(node, "qcom,temp_normal_fastchg_current_ma", &chip->limits.temp_normal_fastchg_current_ma);
    if (rc) {
		chip->limits.temp_normal_fastchg_current_ma = OPCHG_FAST_CHG_MAX_MA;
    }

    rc = of_property_read_u32(node, "qcom,temp_normal_vfloat_mv_normalchg", &chip->limits.temp_normal_vfloat_mv_normalchg);
    if (rc < 0) {
        chip->limits.temp_normal_vfloat_mv_normalchg = 4320;
    }

	rc = of_property_read_u32(node, "qcom,temp_normal_vfloat_mv_voocchg", &chip->limits.temp_normal_vfloat_mv_voocchg);
    if (rc < 0) {
        chip->limits.temp_normal_vfloat_mv_voocchg = 4352;
    }
		
/*45~55 C*/		
    rc = of_property_read_u32(node, "qcom,warm_bat_decidegc", &chip->limits.warm_bat_decidegc);
    if (rc < 0) {
        chip->limits.warm_bat_decidegc = -EINVAL;
    }

	rc = of_property_read_u32(node, "qcom,temp_warm_vfloat_mv", &chip->limits.temp_warm_vfloat_mv);
    if (rc < 0) {
        chip->limits.temp_warm_vfloat_mv = -EINVAL;
    }
   
    rc = of_property_read_u32(node, "qcom,temp_warm_fastchg_current_ma",
                            &chip->limits.temp_warm_fastchg_current_ma);
    if (rc < 0) {
        chip->limits.temp_warm_fastchg_current_ma = -EINVAL;
    }

/*>55 C*/	    
    rc = of_property_read_u32(node, "qcom,hot_bat_decidegc", &chip->limits.hot_bat_decidegc);
    if (rc < 0) {
        chip->limits.hot_bat_decidegc = -EINVAL;
    }

    
/*non standard battery*/
	rc = of_property_read_u32(node, "qcom,non_standard_vfloat_mv",
						&chip->limits.non_standard_vfloat_mv);
	if (rc < 0)
		chip->limits.non_standard_vfloat_mv = -EINVAL;
		
	rc = of_property_read_u32(node, "qcom,non_standard_fastchg_current_ma",
						&chip->limits.non_standard_fastchg_current_ma);
	if (rc < 0)
		chip->limits.non_standard_fastchg_current_ma = -EINVAL;

/*vfloat_sw_limit*/	
	rc = of_property_read_u32(node, "qcom,non_standard_vfloat_sw_limit", &chip->limits.non_standard_vfloat_sw_limit);
    if (rc < 0) {
        chip->limits.non_standard_vfloat_sw_limit = 3960;
    }

	rc = of_property_read_u32(node, "qcom,cold_vfloat_sw_limit", &chip->limits.cold_vfloat_sw_limit);
    if (rc < 0) {
        chip->limits.cold_vfloat_sw_limit = 3960;
    }

	rc = of_property_read_u32(node, "qcom,little_cold_vfloat_sw_limit", &chip->limits.little_cold_vfloat_sw_limit);
    if (rc < 0) {
        chip->limits.little_cold_vfloat_sw_limit = 4330;
    }

	rc = of_property_read_u32(node, "qcom,cool_vfloat_sw_limit", &chip->limits.cool_vfloat_sw_limit);
    if (rc < 0) {
        chip->limits.cool_vfloat_sw_limit = 4330;
    }

	rc = of_property_read_u32(node, "qcom,little_cool_vfloat_sw_limit", &chip->limits.little_cool_vfloat_sw_limit);
    if (rc < 0) {
        chip->limits.little_cool_vfloat_sw_limit = 4330;
    }

	rc = of_property_read_u32(node, "qcom,normal_vfloat_sw_limit", &chip->limits.normal_vfloat_sw_limit);
    if (rc < 0) {
        chip->limits.normal_vfloat_sw_limit = 4330;
    }

	rc = of_property_read_u32(node, "qcom,warm_vfloat_sw_limit", &chip->limits.warm_vfloat_sw_limit);
    if (rc < 0) {
        chip->limits.warm_vfloat_sw_limit = 4060;
    }
	
	rc = of_property_read_u32(node, "qcom,max_chg_time_sec",
						&chip->limits.max_chg_time_sec);
	if (rc < 0)
		chip->limits.max_chg_time_sec = 36000;

	rc = of_property_read_u32(node, "qcom,charger_hv_thr",
						&chip->limits.charger_hv_thr);
	if (rc < 0)
		chip->limits.charger_hv_thr = 5800;

	rc = of_property_read_u32(node, "qcom,charger_lv_thr",
						&chip->limits.charger_lv_thr);
	if (rc < 0)
		chip->limits.charger_lv_thr = 3400;
		
	rc = of_property_read_u32(node, "qcom,vbatt_full_thr",
						&chip->limits.vbatt_full_thr);
	if (rc < 0)
		chip->limits.vbatt_full_thr = 4400;
		
	rc = of_property_read_u32(node, "qcom,vbatt_hv_thr",
						&chip->limits.vbatt_hv_thr);
	if (rc < 0)
		chip->limits.vbatt_hv_thr = 4500;	

	rc = of_property_read_u32(node, "qcom,vfloat_step_mv",
						&chip->limits.vfloat_step_mv);
	if (rc < 0)
		chip->limits.vfloat_step_mv = 16;	
	
	chip->vooc_project = of_property_read_bool(node, "qcom,vooc_project");

	chip->suspend_after_full = of_property_read_bool(node, "qcom,suspend_after_full");
	
	chip->check_batt_full_by_sw = of_property_read_bool(node, "qcom,check_batt_full_by_sw");

	chip->external_gauge = of_property_read_bool(node, "qcom,external_gauge");

	chip->chg_ctrl_by_lcd = of_property_read_bool(node, "qcom,chg_ctrl_by_lcd");
	chip->chg_ctrl_by_camera = of_property_read_bool(node, "qcom,chg_ctrl_by_camera");
	
    charger_xlog_printk(CHG_LOG_CRTI, "input_current_charger_ma = %d,input_current_usb_ma = %d,temp_normal_fastchg_current_ma = %d,temp_normal_vfloat_mv_normalchg = %d,temp_normal_vfloat_mv_voocchg = %d,\
		iterm_ma = %d,recharge_mv = %d,cold_bat_decidegc = %d,temp_cold_vfloat_mv = %d,temp_cold_fastchg_current_ma = %d,\
		little_cold_bat_decidegc = %d,temp_little_cold_vfloat_mv = %d,temp_little_cold_fastchg_current_ma = %d,cool_bat_decidegc = %d,\
		temp_cool_vfloat_mv = %d,temp_cool_fastchg_current_ma_high = %d,temp_cool_fastchg_current_ma_low = %d,little_cool_bat_decidegc = %d,temp_little_cool_vfloat_mv = %d,\
		temp_little_cool_fastchg_current_ma = %d,normal_bat_decidegc = %d,warm_bat_decidegc = %d,temp_warm_vfloat_mv = %d,\
		temp_warm_fastchg_current_ma = %d,hot_bat_decidegc = %d,non_standard_vfloat_mv = %d,non_standard_fastchg_current_ma = %d,\
		max_chg_time_sec = %d,charger_hv_thr = %d,charger_lv_thr = %d,vbatt_full_thr = %d,vbatt_hv_thr = %d,vfloat_step_mv = %d,vooc_project = %d,suspend_after_full = %d,ext_gauge = %d\n", 
		chip->limits.input_current_charger_ma, chip->limits.input_current_usb_ma,chip->limits.temp_normal_fastchg_current_ma,chip->limits.temp_normal_vfloat_mv_normalchg,
		chip->limits.temp_normal_vfloat_mv_voocchg, chip->limits.iterm_ma, chip->limits.recharge_mv, chip->limits.cold_bat_decidegc, chip->limits.temp_cold_vfloat_mv, 
		chip->limits.temp_cold_fastchg_current_ma, chip->limits.little_cold_bat_decidegc, chip->limits.temp_little_cold_vfloat_mv, chip->limits.temp_little_cold_fastchg_current_ma, 
		chip->limits.cool_bat_decidegc,chip->limits.temp_cool_vfloat_mv, chip->limits.temp_cool_fastchg_current_ma_high, chip->limits.temp_cool_fastchg_current_ma_low, 
		chip->limits.little_cool_bat_decidegc, chip->limits.temp_little_cool_vfloat_mv,chip->limits.temp_little_cool_fastchg_current_ma, chip->limits.normal_bat_decidegc,
		chip->limits.warm_bat_decidegc,chip->limits.temp_warm_vfloat_mv,chip->limits.temp_warm_fastchg_current_ma, chip->limits.hot_bat_decidegc,chip->limits.non_standard_vfloat_mv,
		chip->limits.non_standard_fastchg_current_ma, chip->limits.max_chg_time_sec, chip->limits.charger_hv_thr, chip->limits.charger_lv_thr, chip->limits.vbatt_full_thr, 
		chip->limits.vbatt_hv_thr, chip->limits.vfloat_step_mv, chip->vooc_project, chip->suspend_after_full, chip->external_gauge);
    return 0;
}

static void oppo_chg_set_charging_current(struct oppo_chg_chip *chip)
{
	int charging_current = OPPO_CHG_DEFAULT_CHARGING_CURRENT;

	switch(chip->tbatt_status){
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:		
		case BATTERY_STATUS__HIGH_TEMP:
			return;
		case BATTERY_STATUS__COLD_TEMP:			
			charging_current = chip->limits.temp_cold_fastchg_current_ma;
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:			
			charging_current = chip->limits.temp_little_cold_fastchg_current_ma;
			break;
		case BATTERY_STATUS__COOL_TEMP:
			if(vbatt_higherthan_4180mv)
				charging_current = chip->limits.temp_cool_fastchg_current_ma_low;
			else
				charging_current = chip->limits.temp_cool_fastchg_current_ma_high;
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
			charging_current = chip->limits.temp_little_cool_fastchg_current_ma;
			break;
		case BATTERY_STATUS__NORMAL:		
			charging_current = chip->limits.temp_normal_fastchg_current_ma;
			break;
		case BATTERY_STATUS__WARM_TEMP:	
			charging_current = chip->limits.temp_warm_fastchg_current_ma;
			break;
		default:
			break;
	}

	if((!chip->authenticate) && (charging_current > chip->limits.non_standard_fastchg_current_ma))
	{
		charging_current = chip->limits.non_standard_fastchg_current_ma;
    	charger_xlog_printk(CHG_LOG_CRTI, "no high battery,set charging current = %d\n",chip->limits.non_standard_fastchg_current_ma);            
	}
	if (charging_current == 0)
		return;
	
	chip->chg_ops->charging_current_write_fast(chip, charging_current);
}

static void oppo_chg_set_input_current_limit(struct oppo_chg_chip *chip)
{
	int current_limit;	
/*
	switch(chip->charger_type){
		case CHARGER_UNKNOWN:		
			return;
		case STANDARD_HOST:
			current_limit = chip->limits.input_current_usb_ma;
			break;
		case NONSTANDARD_CHARGER:
		case APPLE_0_5A_CHARGER:
			current_limit = chip->limits.input_current_charger_ma;
			break;
		case CHARGING_HOST:
		case STANDARD_CHARGER:		
		case APPLE_2_1A_CHARGER:
		case APPLE_1_0A_CHARGER:
			current_limit = chip->limits.input_current_charger_ma;
			break;
		default:
			return;
	}
*/
	switch(chip->charger_type){
		case POWER_SUPPLY_TYPE_UNKNOWN:		
			return;
		case POWER_SUPPLY_TYPE_USB:
			current_limit = chip->limits.input_current_usb_ma;
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
			current_limit = chip->limits.input_current_charger_ma;
			break;
		default:
			return;
	}
	charger_xlog_printk(CHG_LOG_CRTI," led_on = %d,current_limit = %d,input = %d\n", chip->led_on,current_limit,chip->limits.input_current_led_ma);
	if((chip->chg_ctrl_by_lcd) && (chip->led_on) && (current_limit > chip->limits.input_current_led_ma)) {
		current_limit = chip->limits.input_current_led_ma;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]LED STATUS CHANGED, IS ON\n");
		if((chip->chg_ctrl_by_camera) && (chip->camera_on) && (current_limit > chip->limits.input_current_camera_ma)) {
			current_limit = chip->limits.input_current_camera_ma;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]CAMERA STATUS CHANGED, IS ON\n");  
		}
	} else if((chip->chg_ctrl_by_camera) && (chip->camera_on) && (current_limit > chip->limits.input_current_camera_ma)) {
		current_limit = chip->limits.input_current_camera_ma;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]CAMERA STATUS CHANGED, IS ON\n");            
	}
	
/*
 	if(TEMP_RISE_LED == val)
	{
		if((chip->led_on) && (current_limit > chip->limits.input_current_led_ma))
		{
			current_limit = chip->limits.input_current_led_ma;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]LED STATUS CHANGED, IS ON\n");
		}
	}
	else if(TEMP_RISE_CAMERA == val)
	{
		if((chip->camera_on) && (current_limit > chip->limits.input_current_camera_ma))
		{
			current_limit = chip->limits.input_current_camera_ma;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]CAMERA STATUS CHANGED, IS ON\n");            
		}
	}
	else 
	{
		if((chip->led_on) && (current_limit > chip->limits.input_current_led_ma))
		{
			current_limit = chip->limits.input_current_led_ma;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]LED STATUS CHANGED, IS ON\n");
		}
		else if((chip->camera_on) && (current_limit > chip->limits.input_current_camera_ma))
		{
			current_limit = chip->limits.input_current_camera_ma;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]CAMERA STATUS CHANGED, IS ON\n");            
		}
			
	}
*/
	chip->chg_ops->input_current_write(chip, current_limit);
}
static int oppo_chg_get_float_voltage(struct oppo_chg_chip *chip)
{
	int flv = chip->limits.temp_normal_vfloat_mv_normalchg;
	switch(chip->tbatt_status){
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
		case BATTERY_STATUS__HIGH_TEMP:
			return flv;
		case BATTERY_STATUS__COLD_TEMP:			
			flv = chip->limits.temp_cold_vfloat_mv;
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:			
			flv = chip->limits.temp_little_cold_vfloat_mv;
			break;
		case BATTERY_STATUS__COOL_TEMP:	
			flv = chip->limits.temp_cool_vfloat_mv;
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
			flv = chip->limits.temp_little_cool_vfloat_mv;
		case BATTERY_STATUS__NORMAL:
			if(oppo_vooc_get_fastchg_to_normal() && chip->charging_state != CHARGING_STATUS_FULL)
				flv = chip->limits.temp_normal_vfloat_mv_voocchg;
			else
				flv = chip->limits.temp_normal_vfloat_mv_normalchg;
			break;
		case BATTERY_STATUS__WARM_TEMP:	
			flv = chip->limits.temp_warm_vfloat_mv;
			break;
		default:
			break;
	}
	return flv;
}

static void oppo_chg_set_float_voltage(struct oppo_chg_chip *chip)
{
	int flv = oppo_chg_get_float_voltage(chip);
	
	if((!chip->authenticate) && (flv > chip->limits.non_standard_vfloat_mv))
	{
		flv = chip->limits.non_standard_vfloat_mv;
    	charger_xlog_printk(CHG_LOG_CRTI, "no high battery,set float voltage = %d\n",chip->limits.non_standard_vfloat_mv);		
	}

	chip->chg_ops->float_voltage_write(chip, flv);
	chip->vfloat_new = flv;
}

#define VFLOAT_OVER_NUM		2
static void oppo_chg_vfloat_over_check(struct oppo_chg_chip *chip)
{
	static int vfloat_over_count = 0;

	if (!chip->mmi_chg)
		return;
	if(!chip->charger_exist) {
		vfloat_over_count = 0;
		return ;
	}
	if (chip->check_batt_full_by_sw)
		return;

	if(chip->charging_state == CHARGING_STATUS_FULL) {
		vfloat_over_count = 0;
		return ;
	}

	if ( (chip->batt_volt >= chip->limits.cold_vfloat_sw_limit 
			&& chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) || 
		(chip->batt_volt >= chip->limits.little_cold_vfloat_sw_limit
			&& chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) ||
		(chip->batt_volt >= chip->limits.cool_vfloat_sw_limit
			&& chip->tbatt_status == BATTERY_STATUS__COOL_TEMP) ||
		(chip->batt_volt >= chip->limits.little_cool_vfloat_sw_limit
			&& chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP) ||
		(chip->batt_volt >= chip->limits.normal_vfloat_sw_limit
			&& chip->tbatt_status == BATTERY_STATUS__NORMAL) ||
		(chip->batt_volt >= chip->limits.warm_vfloat_sw_limit
			&& chip->tbatt_status == BATTERY_STATUS__WARM_TEMP)	|| 
		(!chip->authenticate && (chip->batt_volt >= chip->limits.non_standard_vfloat_sw_limit)) ) {
		vfloat_over_count++;
		if (vfloat_over_count > VFLOAT_OVER_NUM) {
			vfloat_over_count = 0;
			chip->vfloat_new -= chip->limits.vfloat_step_mv;
			chip->chg_ops->float_voltage_write(chip, chip->vfloat_new);
		}
	} else {
		vfloat_over_count = 0;
	}
}

static void oppo_chg_check_vbatt_higher_than_4180mv(struct oppo_chg_chip *chip)
{
	static bool vol_high_pre = false;
	static int lower_count = 0, higher_count = 0;

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
	/* This is mainly for high temperature aging test */
	if (chip->tbatt_status == BATTERY_STATUS__HIGH_TEMP && chip->batt_volt < 4180) {
		if (chip->chg_ops->get_charging_enable(chip) == false) {
			oppo_chg_turn_off_charging(chip);
			chip->chg_ops->charger_unsuspend(chip);
		}
	}
#endif

	if (!chip->mmi_chg || !chip->charger_exist || chip->tbatt_status != BATTERY_STATUS__COOL_TEMP ) {
		vbatt_higherthan_4180mv = false;
		vol_high_pre = false;
		lower_count = 0;
		higher_count = 0;
		return;
	}

	if(chip->batt_volt > 4180) {
		higher_count++;
		if(higher_count > 2) {
			lower_count = 0;
			higher_count = 3;
			vbatt_higherthan_4180mv = true;
		}
	} else if(vbatt_higherthan_4180mv) {
		if(chip->batt_volt < 4000) {
			lower_count++;
			if(lower_count > 2) {
				lower_count = 3;
				higher_count = 0;
				vbatt_higherthan_4180mv = false;
			}
		} 
	}
	//chg_err(" tbatt_status:%d,batt_volt:%d,vol_high_pre:%d,vbatt_higherthan_4180mv:%d\n",
		//chip->tbatt_status,chip->batt_volt,vol_high_pre,vbatt_higherthan_4180mv);
	if(vol_high_pre != vbatt_higherthan_4180mv) {
		vol_high_pre = vbatt_higherthan_4180mv;
		oppo_chg_set_charging_current(chip);
	}
}

static void oppo_chg_turn_on_charging(struct oppo_chg_chip *chip)
{
	if (!chip->authenticate)
		return;
	if (!chip->mmi_chg)
		return;
	if(oppo_vooc_get_allow_reading() == false)
		return;
	chip->chg_ops->hardware_init(chip);
	if (chip->check_batt_full_by_sw)
		chip->chg_ops->set_charging_term_disable(chip);
	//chip->chg_ops->charger_unsuspend(chip);
	//chip->chg_ops->charging_enable(chip);
	oppo_chg_check_tbatt_status(chip);
	oppo_chg_set_float_voltage(chip);
	oppo_chg_set_charging_current(chip);
	oppo_chg_set_input_current_limit(chip);
	chip->chg_ops->term_current_set(chip, chip->limits.iterm_ma);
}

static void oppo_chg_turn_off_charging(struct oppo_chg_chip *chip)
{
	bool flag = false;

	if(oppo_vooc_get_allow_reading() == false)
		return;
	
	switch(chip->tbatt_status){
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
			break;
		case BATTERY_STATUS__HIGH_TEMP:
			if (chip->batt_volt < 4250)
				flag = true;
			break;
		case BATTERY_STATUS__COLD_TEMP:			
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:
		case BATTERY_STATUS__COOL_TEMP:		
			chip->chg_ops->charging_current_write_fast(chip, chip->limits.temp_cold_fastchg_current_ma);
			msleep(50);
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
		case BATTERY_STATUS__NORMAL:				
			chip->chg_ops->charging_current_write_fast(chip, chip->limits.temp_cool_fastchg_current_ma_high);
			msleep(50);
			chip->chg_ops->charging_current_write_fast(chip, chip->limits.temp_cold_fastchg_current_ma);
			msleep(50);
			break;
		case BATTERY_STATUS__WARM_TEMP:	
			chip->chg_ops->charging_current_write_fast(chip, chip->limits.temp_cold_fastchg_current_ma);
			msleep(50);
			if (chip->batt_volt < 4250)
				flag = true;
			break;
		default:
			break;
	}
	
    //charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] oppo_chg_turn_off_charging !!\n");
	chip->chg_ops->charging_disable(chip, flag);
	//chip->chg_ops->reset_charger(chip);
}

static int oppo_chg_check_suspend_or_disable(struct oppo_chg_chip *chip)
{
	if(chip->suspend_after_full) {
		if(chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP 
			|| chip->tbatt_status == BATTERY_STATUS__COOL_TEMP
			|| chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP
			|| chip->tbatt_status == BATTERY_STATUS__NORMAL) {
			return CHG_SUSPEND;
		} else if(chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
			return CHG_DISABLE;
		} else {
			return CHG_NO_SUSPEND_NO_DISABLE;
		}
	} else {
		if(chip->tbatt_status == BATTERY_STATUS__COLD_TEMP 
			|| chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {
			return CHG_DISABLE;
		} else {
			return CHG_NO_SUSPEND_NO_DISABLE;
		}
	}
}

static void oppo_chg_voter_charging_start(struct oppo_chg_chip *chip, OPPO_CHG_STOP_VOTER voter)
{
	chip->chging_on = true;
	chip->stop_voter &= ~(int)voter;
	oppo_chg_turn_on_charging(chip);

	switch (voter)
	{
		case CHG_STOP_VOTER__FULL:
			chip->charging_state = CHARGING_STATUS_CCCV;
			if(oppo_vooc_get_allow_reading() == true) {
				chip->chg_ops->charger_unsuspend(chip);
				chip->chg_ops->charging_enable(chip);
			}
			break;
			
		case CHG_STOP_VOTER__VCHG_ABNORMAL:	
			chip->charging_state = CHARGING_STATUS_CCCV;
			if(oppo_vooc_get_allow_reading() == true) {
				chip->chg_ops->charger_unsuspend(chip);
			}
			break;
			
		case CHG_STOP_VOTER__BATTTEMP_ABNORMAL:
		case CHG_STOP_VOTER__VBAT_TOO_HIGH:
		case CHG_STOP_VOTER__MAX_CHGING_TIME:
			chip->charging_state = CHARGING_STATUS_CCCV;
			break;
			
		default:
			break;
	}

}


static void oppo_chg_voter_charging_stop(struct oppo_chg_chip *chip, OPPO_CHG_STOP_VOTER voter)
{
	int suspend_or_disable = CHG_NO_SUSPEND_NO_DISABLE;
	
	chip->chging_on = false;
	chip->stop_voter |= (int)voter;
	oppo_chg_turn_off_charging(chip);
	
	switch (voter)
	{
		case CHG_STOP_VOTER__FULL:
			chip->charging_state = CHARGING_STATUS_FULL;
			if(oppo_vooc_get_allow_reading() == true) {
				suspend_or_disable = oppo_chg_check_suspend_or_disable(chip);
				if(suspend_or_disable == CHG_SUSPEND)
					chip->chg_ops->charger_suspend(chip);
				else if(suspend_or_disable == CHG_DISABLE)
					oppo_chg_turn_off_charging(chip);
					//chip->chg_ops->charging_disable(chip);
			}
			break;
			
		case CHG_STOP_VOTER__VCHG_ABNORMAL:			
			chip->charging_state = CHARGING_STATUS_FAIL;
			chip->total_time = 0;
			if(oppo_vooc_get_allow_reading() == true)
			{
				chip->chg_ops->charger_suspend(chip);
			}
			break;
		case CHG_STOP_VOTER__BATTTEMP_ABNORMAL:
		case CHG_STOP_VOTER__VBAT_TOO_HIGH:
			chip->charging_state = CHARGING_STATUS_FAIL;
			chip->total_time = 0;
			break;
		case CHG_STOP_VOTER__MAX_CHGING_TIME:
			chip->charging_state = CHARGING_STATUS_FAIL;
			break;
		default:
			break;
	}
	
}

#define HYSTERISIS_DECIDEGC       		20
#define INITIAL_TEMP_INVALID			999
static void battery_temp_anti_shake_handle(struct oppo_chg_chip *chip)
{
	static int temp_pre = INITIAL_TEMP_INVALID;
	int temp_cur = chip->temperature, low_shake = 0, high_shake = 0;

	if(temp_pre == INITIAL_TEMP_INVALID) {
		temp_pre = temp_cur;
	}
	if(temp_cur > temp_pre) {	//get warm
		low_shake = -HYSTERISIS_DECIDEGC;
		high_shake = 0;
	} else {	//get cool
		low_shake = 0;
		high_shake = HYSTERISIS_DECIDEGC;
	}
	
	if(chip->tbatt_status == BATTERY_STATUS__HIGH_TEMP)											/*>53C*/
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound ;					/*-3C*/
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;		/*0C*/
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;						/*5C*/
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;		/*12C*/
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;					/*16C*/
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;						/*45C*/
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound + low_shake;			/*53C*/
	}
	else if(chip->tbatt_status == BATTERY_STATUS__LOW_TEMP)			/*<-3C*/
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound + high_shake;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	}
	else if(chip->tbatt_status == BATTERY_STATUS__COLD_TEMP)			/*-3C~0C*/
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound + low_shake;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound + high_shake;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	}
	else if(chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP)			/*0C-5C*/
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound ;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound + low_shake;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound + high_shake;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	}
	else if(chip->tbatt_status == BATTERY_STATUS__COOL_TEMP)			/*5C~12C*/
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound + low_shake;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound + high_shake;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	}
	else if(chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP)		/*12C~16C*/	
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound + low_shake;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound + high_shake;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	}
	else if(chip->tbatt_status == BATTERY_STATUS__NORMAL)				/*16C~45C*/
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound ;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	}
	else if(chip->tbatt_status == BATTERY_STATUS__WARM_TEMP)			/*45C~53C*/
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound + low_shake;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound + high_shake;
	}
	else//BATTERY_STATUS__REMOVED								/*<-19C*/
	{
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	}
	temp_pre = temp_cur;
}


#define TEMP_CNT 1
static bool oppo_chg_check_tbatt_is_good(struct oppo_chg_chip *chip)
{
	static bool ret = true;
	static int temp_counts = 0;
	int batt_temp = chip->temperature;
	OPPO_CHG_TBATT_STATUS tbatt_status = chip->tbatt_status;
	
	if (batt_temp > chip->limits.hot_bat_decidegc || batt_temp < chip->limits.cold_bat_decidegc) {
		temp_counts++;
		if (temp_counts >= TEMP_CNT) {

			temp_counts = 0;
			ret = false;
			
			if (batt_temp <= chip->limits.removed_bat_decidegc) {
				//chip->notify_code |= OPPO_CHG_BTEMP_REM_LOW;
				tbatt_status = BATTERY_STATUS__REMOVED;
			} else if (batt_temp > chip->limits.hot_bat_decidegc) {
				//chip->notify_code |= OPPO_CHG_BTEMP_HIGH;
				tbatt_status = BATTERY_STATUS__HIGH_TEMP;
			} else {
				//chip->notify_code |= OPPO_CHG_BTEMP_LOW;
				tbatt_status = BATTERY_STATUS__LOW_TEMP;
			}
		}
	} else {
		temp_counts = 0;
		ret = true;
		
		if(batt_temp >= chip->limits.warm_bat_decidegc)					//45C
			tbatt_status = BATTERY_STATUS__WARM_TEMP;
		else if(batt_temp >= chip->limits.normal_bat_decidegc)			//16C
			tbatt_status = BATTERY_STATUS__NORMAL;
		else if(batt_temp >= chip->limits.little_cool_bat_decidegc)		//12C
			tbatt_status = BATTERY_STATUS__LITTLE_COOL_TEMP;
		else if(batt_temp >= chip->limits.cool_bat_decidegc)			//5C
			tbatt_status = BATTERY_STATUS__COOL_TEMP;
		else if(batt_temp >= chip->limits.little_cold_bat_decidegc)		//0C
			tbatt_status = BATTERY_STATUS__LITTLE_COLD_TEMP;
		else if(batt_temp >= chip->limits.cold_bat_decidegc)		//-3C
			tbatt_status = BATTERY_STATUS__COLD_TEMP;
		else
			tbatt_status = BATTERY_STATUS__COLD_TEMP;
	}
	
	if (tbatt_status == BATTERY_STATUS__REMOVED)
		chip->batt_exist = false;
	else
		chip->batt_exist = true;


	if(tbatt_status != chip->tbatt_status)
	{
		chip->tbatt_status = tbatt_status;
		if(oppo_vooc_get_allow_reading() == true)
		{
			oppo_chg_set_float_voltage(chip);
			oppo_chg_set_charging_current(chip);
		}
				
		battery_temp_anti_shake_handle(chip);
	}

	return ret;

}

static void oppo_chg_check_tbatt_status(struct oppo_chg_chip *chip)
{
	int batt_temp = chip->temperature;
	OPPO_CHG_TBATT_STATUS tbatt_status = chip->tbatt_status;

	if(batt_temp > chip->limits.hot_bat_decidegc)				//53C
		tbatt_status = BATTERY_STATUS__HIGH_TEMP;
	else if(batt_temp >= chip->limits.warm_bat_decidegc)		//45C
		tbatt_status = BATTERY_STATUS__WARM_TEMP;
	else if(batt_temp >= chip->limits.normal_bat_decidegc)		//16C
		tbatt_status = BATTERY_STATUS__NORMAL;
	else if(batt_temp >= chip->limits.little_cool_bat_decidegc)	//12C
		tbatt_status = BATTERY_STATUS__LITTLE_COOL_TEMP;
	else if(batt_temp >= chip->limits.cool_bat_decidegc)		//5C
		tbatt_status = BATTERY_STATUS__COOL_TEMP;
	else if(batt_temp >= chip->limits.little_cold_bat_decidegc)		//0C
		tbatt_status = BATTERY_STATUS__LITTLE_COLD_TEMP;
	else if(batt_temp >= chip->limits.cold_bat_decidegc)		//-3C
		tbatt_status = BATTERY_STATUS__COLD_TEMP;
	else if(batt_temp > chip->limits.removed_bat_decidegc)		//-20C	
		tbatt_status = BATTERY_STATUS__LOW_TEMP;
	else
		tbatt_status = BATTERY_STATUS__REMOVED;
		
	if (tbatt_status == BATTERY_STATUS__REMOVED)
		chip->batt_exist = false;
	else
		chip->batt_exist = true;

	chip->tbatt_status = tbatt_status;
}


#define VCHG_CNT 1
static bool oppo_chg_check_vchg_is_good(struct oppo_chg_chip *chip)
{
	static bool ret = true;
	static int vchg_counts = 0;
	int chg_volt = chip->charger_volt;
	OPPO_CHG_VCHG_STATUS vchg_status = chip->vchg_status;

	if(oppo_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE) {
		return true;
	}
	
#if 0
	if (chg_volt > chip->limits.charger_hv_thr || chg_volt < chip->limits.charger_lv_thr) {
		vchg_counts++;
		if (vchg_counts >= VCHG_CNT) {
			vchg_counts = 0;
			ret = false;
			
			if (chg_volt > chip->limits.charger_hv_thr)
			{
				//chip->notify_code |= OPPO_CHG_VCHG_HIGH;
				vchg_status = CHARGER_STATUS__VOL_HIGH;
			}				
			else
			{
				//chip->notify_code |= OPPO_CHG_VCHG_LOW;
				vchg_status = CHARGER_STATUS__VOL_LOW;
			}
		}
	} else {
		vchg_counts = 0;
		ret = true;
		vchg_status = CHARGER_STATUS__GOOD;
	}
#else
	if (chg_volt > chip->limits.charger_hv_thr) {
		vchg_counts++;
		if (vchg_counts >= VCHG_CNT) {
			vchg_counts = 0;
			ret = false;
			//chip->notify_code |= OPPO_CHG_VCHG_HIGH;
			vchg_status = CHARGER_STATUS__VOL_HIGH;			
		}
	} else {
		vchg_counts = 0;
		ret = true;
		vchg_status = CHARGER_STATUS__GOOD;
	}
#endif

	if(vchg_status != chip->vchg_status)
	{
		chip->vchg_status = vchg_status;
	}

	return ret;
}

#define VBAT_CNT 1

static bool oppo_chg_check_vbatt_is_good(struct oppo_chg_chip *chip)
{
	static bool ret = true;
	static int vbat_counts = 0;
	int batt_volt = chip->batt_volt;
	
	if (batt_volt >= chip->limits.vbatt_hv_thr) {
		vbat_counts++;
		if (vbat_counts >= VBAT_CNT) {
			
			vbat_counts = 0;
			ret = false;
			chip->vbatt_over = true;
		}
	} 
	else {
		vbat_counts = 0;
		ret = true;
		chip->vbatt_over = false;
	}
	
	return ret;
}

#ifndef OPPO_CMCC_TEST
static bool oppo_chg_check_time_is_good(struct oppo_chg_chip *chip)
{
	if (chip->total_time >= chip->limits.max_chg_time_sec) {
		chip->total_time = chip->limits.max_chg_time_sec;
		chip->chging_over_time = true;
		return false;
	} else {
		chip->chging_over_time = false;
		return true;
	}
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void chg_early_suspend(struct early_suspend *h)
{
	if (g_charger_chip) {
		g_charger_chip->led_on = false;
		g_charger_chip->led_status_change = true;
	}
}

static void chg_late_resume(struct early_suspend *h)
{
	if (g_charger_chip) {
		g_charger_chip->led_on = true;
		g_charger_chip->led_status_change = true;
	}
}

void oppo_chg_set_led_status(bool val)
{
	//Do nothing
}
EXPORT_SYMBOL(oppo_chg_set_led_status);
#elif CONFIG_FB
static int fb_notifier_callback(struct notifier_block *nb, unsigned long event, void *data)
{
	int blank;
	struct fb_event *evdata = data;

	if (!g_charger_chip)
		return 0;

	if (evdata && evdata->data) {
		if (event == FB_EVENT_BLANK) {
			blank = *(int *)evdata->data;
			if (blank == FB_BLANK_UNBLANK) {
				g_charger_chip->led_on = true;
				g_charger_chip->led_status_change = true;
			} else if (blank == FB_BLANK_POWERDOWN) {
				g_charger_chip->led_on = false;
				g_charger_chip->led_status_change = true;
			}
		}
	}
	return 0;
}

void oppo_chg_set_led_status(bool val)
{
	//Do nothing
}
EXPORT_SYMBOL(oppo_chg_set_led_status);
#else
void oppo_chg_set_led_status(bool val)
{
	charger_xlog_printk(CHG_LOG_CRTI," val = %d\n", val);
	if(!g_charger_chip)
		return;
	else {
		g_charger_chip->led_on = val;		
		g_charger_chip->led_status_change = true;
	}
}
EXPORT_SYMBOL(oppo_chg_set_led_status);
#endif

void oppo_chg_set_camera_status(bool val)
{
	if(!g_charger_chip)
		return;
	else
	{
		g_charger_chip->camera_on = val;
	}
		
}
EXPORT_SYMBOL(oppo_chg_set_camera_status);


#ifndef CONFIG_OPPO_CHARGER_MTK
void opchg_check_lcd_on(void)
{
	if(!g_charger_chip)
		return;
	else
	{
		g_charger_chip->led_on = true;
	}
}
EXPORT_SYMBOL(opchg_check_lcd_on);
void opchg_check_lcd_off(void)
{
	if(!g_charger_chip)
		return;
	else
	{
		g_charger_chip->led_on = false;
	}
}
EXPORT_SYMBOL(opchg_check_lcd_off);
void opchg_check_camera_on(void)
{
	if(!g_charger_chip)
		return;
	else
	{
		g_charger_chip->camera_on = true;
	}

}
EXPORT_SYMBOL(opchg_check_camera_on);
void opchg_check_camera_off(void)
{
	if(!g_charger_chip)
		return;
	else
	{
		g_charger_chip->camera_on = false;
	}
}
EXPORT_SYMBOL(opchg_check_camera_off);
void opchg_check_earphone_on(void)
{
	
}
EXPORT_SYMBOL(opchg_check_earphone_on);
void opchg_check_earphone_off(void)
{
	
}
EXPORT_SYMBOL(opchg_check_earphone_off);
#endif


static void oppo_chg_check_led_on_ichging(struct oppo_chg_chip *chip)
{
	
	if (chip->led_status_change)
	{
		chip->led_status_change = false;
		if(oppo_vooc_get_allow_reading() == true)
		{
			oppo_chg_set_input_current_limit(chip);
		}
	}
}

static void oppo_chg_check_camera_on_ichging(struct oppo_chg_chip *chip)
{
	static bool camera_pre = false;
	
	if (chip->camera_on != camera_pre)
	{
		camera_pre = chip->camera_on;
		if(oppo_vooc_get_allow_reading() == true)
		{
			oppo_chg_set_input_current_limit(chip);
		}
	}
	
}

static void oppo_chg_battery_authenticate_check(struct oppo_chg_chip *chip)
{
	static bool charger_exist_pre = false;

	if (charger_exist_pre ^ chip->charger_exist) {
		charger_exist_pre = chip->charger_exist;
		if (chip->charger_exist && !chip->authenticate)
			chip->authenticate = oppo_gauge_get_batt_authenticate();
	}
}
	

void oppo_chg_variables_reset(struct oppo_chg_chip *chip, bool in)
{
	if (in) {
		chip->charger_exist = true;
		chip->chging_on = true;
	} else {
		chip->charger_exist = false;
		chip->chging_on = false;
#ifndef CONFIG_OPPO_CHARGER_MTK
		chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
#endif
	}

#ifdef CONFIG_OPPO_CHARGER_MTK
	chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
#endif
	
	//chip->charger_volt = 5000;
	chip->vchg_status = CHARGER_STATUS__GOOD;
	
	chip->batt_full = false;	
	chip->tbatt_status = BATTERY_STATUS__NORMAL;
	chip->vbatt_over = 0;	

	chip->total_time = 0;
	chip->chging_over_time = 0;	
	chip->in_rechging = 0;
	//chip->batt_volt = 0;
	//chip->temperature = 0;
	
	chip->stop_voter = 0x00;
	chip->charging_state=CHARGING_STATUS_CCCV;
	chip->mmi_chg = 1;

	chip->notify_code = 0;
	chip->notify_flag = 0;

	chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound ;
	chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound ;
	chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
	chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
	chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
	chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
	chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	reset_mcu_delay = 0;
	chip->aicl_suspend = false;
	oppo_chg_battery_authenticate_check(chip);
}


static void oppo_chg_variables_init(struct oppo_chg_chip *chip)
{
	chip->charger_exist = false;
	chip->chging_on = false;
	chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->charger_volt = 0;
	chip->vchg_status = CHARGER_STATUS__GOOD;
	chip->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	
	chip->batt_exist = true;
	chip->batt_full = false;	
	chip->tbatt_status = BATTERY_STATUS__NORMAL;
	chip->vbatt_over = 0;	

	chip->total_time = 0;
	chip->chging_over_time = 0;	
	chip->in_rechging = 0;
	
	chip->batt_volt = 3800;
	chip->icharging = 0;
	chip->temperature = 250;
	chip->soc = 0;
	chip->ui_soc = 50;
	chip->notify_code = 0;
	chip->notify_flag = 0;
	chip->request_power_off = 0;

	chip->led_on = true;
	chip->camera_on = 0;
	
	chip->stop_voter = 0x00;
	chip->charging_state=CHARGING_STATUS_CCCV;
	chip->mmi_chg = 1;
	chip->usb_online = false;
	chip->otg_switch = false;
	chip->otg_online = false;
	chip->boot_mode = chip->chg_ops->get_boot_mode();
	chip->boot_reason = chip->chg_ops->get_boot_reason();

	chip->anti_shake_bound.cold_bound = chip->limits.cold_bat_decidegc;
	chip->anti_shake_bound.little_cold_bound = chip->limits.little_cold_bat_decidegc;
	chip->anti_shake_bound.cool_bound = chip->limits.cool_bat_decidegc;
	chip->anti_shake_bound.little_cool_bound = chip->limits.little_cool_bat_decidegc;
	chip->anti_shake_bound.normal_bound = chip->limits.normal_bat_decidegc;
	chip->anti_shake_bound.warm_bound = chip->limits.warm_bat_decidegc;
	chip->anti_shake_bound.hot_bound = chip->limits.hot_bat_decidegc;	

}

static void oppo_chg_fail_action(struct oppo_chg_chip *chip)
{
    chg_err("[BATTERY] BAD Battery status... Charging Stop !!\n");
	chip->charging_state = CHARGING_STATUS_FAIL;
	chip->chging_on = false;

	chip->batt_full = false;
	chip->in_rechging = 0;
}

#define D_RECHGING_CNT					5
static void oppo_chg_check_rechg_status(struct oppo_chg_chip *chip)
{
	int nReChgingVol;
	int nBatVol = chip->batt_volt;
	static int rechging_cnt = 0;
	
	if(chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
		nReChgingVol = oppo_chg_get_float_voltage(chip) - 300;
	} else if(chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {
		nReChgingVol = oppo_chg_get_float_voltage(chip) - 200;
	} else {
		nReChgingVol = oppo_chg_get_float_voltage(chip);
		if(nReChgingVol > chip->limits.temp_normal_vfloat_mv_normalchg) {
			nReChgingVol = chip->limits.temp_normal_vfloat_mv_normalchg;
		}
		nReChgingVol = nReChgingVol - chip->limits.recharge_mv;
	}
	if(nBatVol <= nReChgingVol) {
		rechging_cnt++;
	} else {
		rechging_cnt = 0;
	}
	
	if(rechging_cnt > D_RECHGING_CNT)
	{
		charger_xlog_printk(CHG_LOG_CRTI, "[Battery] Battery rechg begin! batt_volt = %d, nReChgingVol = %d\r\n", nBatVol, nReChgingVol);
		rechging_cnt = 0;
		oppo_chg_voter_charging_start(chip, CHG_STOP_VOTER__FULL);//now rechging!
		chip->in_rechging = true;
	}
}

static void oppo_chg_full_action(struct oppo_chg_chip *chip)
{
	charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] Battery full !!\n");  
		
	oppo_chg_voter_charging_stop(chip, CHG_STOP_VOTER__FULL);
	//chip->charging_state = CHARGING_STATUS_FULL;
	chip->batt_full = true;
	chip->total_time = 0;
	oppo_chg_check_rechg_status(chip);
}

void oppo_charger_detect_check(struct oppo_chg_chip *chip)
{	
	static bool charger_resumed = true;
		
	if (chip->chg_ops->check_chrdet_status()) {
		wake_lock(&chip->suspend_lock);
#ifdef CONFIG_OPPO_CHARGER_MTK
		charger_xlog_printk(CHG_LOG_CRTI, "Charger in 0 charger_type=%d\n", chip->charger_type);
#endif
		if(chip->charger_type == POWER_SUPPLY_TYPE_UNKNOWN) {
			oppo_chg_variables_reset(chip, true);
			chip->charger_type = chip->chg_ops->get_charger_type();
			charger_xlog_printk(CHG_LOG_CRTI, "Charger in 1 charger_type=%d\n", chip->charger_type);
#ifdef CONFIG_OPPO_CHARGER_MTK
			if((chip->chg_ops->usb_connect) && (chip->charger_type == POWER_SUPPLY_TYPE_USB))
				chip->chg_ops->usb_connect();
#endif		
			if(oppo_vooc_get_fastchg_to_normal() == true || oppo_vooc_get_fastchg_to_warm() == true) {
				//do nothing
				charger_xlog_printk(CHG_LOG_CRTI, "fast_to_normal or to_warm 1,don't turn on charge here\n");  
			} else {
				charger_resumed = chip->chg_ops->check_charger_resume(chip);
				oppo_chg_turn_on_charging(chip);
			}
			//chg_err("Charger in, charger_type=%d\n", chip->charger_type);
		} else {
			if(oppo_vooc_get_fastchg_to_normal() == true || oppo_vooc_get_fastchg_to_warm() == true) {
				//do nothing
				charger_xlog_printk(CHG_LOG_CRTI, "fast_to_normal or to_warm 2,don't turn on charge here\n");  
			} else if(oppo_vooc_get_fastchg_started() == false && charger_resumed == false) {
				charger_resumed = chip->chg_ops->check_charger_resume(chip);
				oppo_chg_turn_on_charging(chip);
			}
		}
	} else {
		oppo_chg_variables_reset(chip, false);
		oppo_gauge_set_batt_full(false);
#ifdef CONFIG_OPPO_CHARGER_MTK
		if(chip->chg_ops->usb_disconnect)
			chip->chg_ops->usb_disconnect();
#endif
		if(chip->chg_ops->get_charging_enable(chip) == true)
			oppo_chg_turn_off_charging(chip);
		//chg_err("Charger out\n");
		wake_unlock(&chip->suspend_lock);
	}
}

#define RETRY_COUNTS	12
static void oppo_chg_get_battery_data(struct oppo_chg_chip *chip)
{
	static int ui_soc_cp_flag = 0;
	static int soc_load = 0;
	int remain_100_thresh = 97;
	static int retry_counts = 0;
	
	chip->batt_volt = oppo_gauge_get_batt_mvolts()/1000;
	chip->icharging = oppo_gauge_get_batt_current();
	chip->temperature = oppo_gauge_get_batt_temperature();
	chip->soc = oppo_gauge_get_batt_soc();
	chip->charger_volt = chip->chg_ops->get_charger_volt();
	
	if(ui_soc_cp_flag == 0) {
		if ((chip->soc < 0 || chip->soc > 100) && retry_counts < RETRY_COUNTS) {
			charger_xlog_printk(CHG_LOG_CRTI, "[Battery]oppo_chg_get_battery_data,chip->soc[%d],retry_counts[%d]\r\n", chip->soc, retry_counts);
			retry_counts ++;
			chip->soc = 50;
			goto next;
		}

		ui_soc_cp_flag = 1;
		soc_load = chip->chg_ops->get_rtc_soc();
		chip->soc_load = soc_load;
		if ((chip->soc < 0 || chip->soc > 100) && soc_load > 0 && soc_load <= 100)
			chip->soc = soc_load;
		if((soc_load != 0) && ((abs(soc_load-chip->soc)) <= 20)){
			if(chip->suspend_after_full && chip->external_gauge)
				remain_100_thresh = 95;
			else if(chip->suspend_after_full && !chip->external_gauge)
				remain_100_thresh = 92;
			else if(!chip->suspend_after_full && chip->external_gauge)
				remain_100_thresh = 97;
			else if(!chip->suspend_after_full && !chip->external_gauge)
				remain_100_thresh = 95;
			else
				remain_100_thresh = 97;
		 	if(chip->soc < soc_load) {
				if(soc_load == 100 && chip->soc > remain_100_thresh) 
					chip->ui_soc = soc_load;
				else
					chip->ui_soc = soc_load - 1;
			} else {
				chip->ui_soc = soc_load;
			}
		} else {
			chip->ui_soc = chip->soc;
			if (!chip->external_gauge && soc_load == 0 && chip->soc < 5) {
				chip->ui_soc = 0;
			}
		}	
		charger_xlog_printk(CHG_LOG_CRTI, "[Battery]oppo_chg_get_battery_data,soc_load = %d,soc = %d\r\n", soc_load, chip->soc);
	}
next:
	return;
}


/*need to extend it*/
static void oppo_chg_set_aicl_point(struct oppo_chg_chip *chip)
{
	if(oppo_vooc_get_allow_reading() == true) {
		chip->chg_ops->set_aicl_point(chip, chip->batt_volt);
	}
}

#define AICL_DELAY_15MIN			180
static void oppo_chg_check_aicl_input_limit(struct oppo_chg_chip *chip)
{
	static int aicl_delay_count = 0;
	
#ifdef CONFIG_OPPO_CHARGER_MTK
	if(chip->charging_state == CHARGING_STATUS_FAIL || chip->batt_full == true
		|| ((chip->tbatt_status != BATTERY_STATUS__NORMAL) && (chip->tbatt_status != BATTERY_STATUS__LITTLE_COOL_TEMP))
		|| chip->ui_soc > 85 || oppo_vooc_get_fastchg_started() == true)
	{
		aicl_delay_count = 0;
		return;
	}
	
	if(aicl_delay_count > AICL_DELAY_15MIN){
		aicl_delay_count = 0;
		if(oppo_vooc_get_allow_reading() == true){
			oppo_chg_set_input_current_limit(chip);
		}
	} else {
		aicl_delay_count++;
	}
#else
	if(chip->charging_state == CHARGING_STATUS_FAIL || chip->batt_full == true 
		|| ((chip->tbatt_status != BATTERY_STATUS__NORMAL) && (chip->tbatt_status != BATTERY_STATUS__LITTLE_COOL_TEMP))
		|| ((chip->ui_soc > 85) && (chip->aicl_suspend == false)) || oppo_vooc_get_fastchg_started() == true)
	{
		aicl_delay_count = 0;
		return;
	}
	
	if(aicl_delay_count > AICL_DELAY_15MIN) {
		aicl_delay_count = 0;
		if(oppo_vooc_get_allow_reading() == true)
			oppo_chg_set_input_current_limit(chip);
	} else if(chip->aicl_suspend == true && chip->charger_volt > 4450 && chip->charger_volt < 5800){
		aicl_delay_count = 0;
		if(oppo_vooc_get_allow_reading() == true){
			chip->chg_ops->rerun_aicl(chip);
			oppo_chg_set_input_current_limit(chip);
		}
		charger_xlog_printk(CHG_LOG_CRTI,"chip->charger_volt=%d\n",chip->charger_volt);
	} else {
		aicl_delay_count++;
	}
#endif	
}

static void oppo_chg_aicl_check(struct oppo_chg_chip *chip)
{
	if(oppo_vooc_get_fastchg_started() == false) {
		oppo_chg_set_aicl_point(chip);
#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
	return;
#endif
		oppo_chg_check_aicl_input_limit(chip);
	}
}

static void oppo_chg_protection_check(struct oppo_chg_chip *chip)
{
	if(false == oppo_chg_check_tbatt_is_good(chip))
	{
		chg_err("oppo_chg_check_tbatt_is_good func ,false!\n");
		oppo_chg_voter_charging_stop(chip, CHG_STOP_VOTER__BATTTEMP_ABNORMAL);
	}
	else
	{
		if((chip->stop_voter & CHG_STOP_VOTER__BATTTEMP_ABNORMAL) == CHG_STOP_VOTER__BATTTEMP_ABNORMAL)
		{
			charger_xlog_printk(CHG_LOG_CRTI, "oppo_chg_check_tbatt_is_good func ,true! To Normal\n");
			oppo_chg_voter_charging_start(chip, CHG_STOP_VOTER__BATTTEMP_ABNORMAL);
		}
	}

	if(false == oppo_chg_check_vchg_is_good(chip))
	{
		chg_err("oppo_chg_check_vchg_is_good func ,false!\n");
		oppo_chg_voter_charging_stop(chip, CHG_STOP_VOTER__VCHG_ABNORMAL);
	}
	else
	{
		if((chip->stop_voter & CHG_STOP_VOTER__VCHG_ABNORMAL) == CHG_STOP_VOTER__VCHG_ABNORMAL)
		{
			charger_xlog_printk(CHG_LOG_CRTI, "oppo_chg_check_vchg_is_good func ,true! To Normal\n");
			oppo_chg_voter_charging_start(chip, CHG_STOP_VOTER__VCHG_ABNORMAL);
		}
	}

#ifdef FEATURE_VBAT_PROTECT
	if(false == oppo_chg_check_vbatt_is_good(chip))
	{
		chg_err("oppo_chg_check_vbatt_is_good func ,false!\n");
		oppo_chg_voter_charging_stop(chip, CHG_STOP_VOTER__VBAT_TOO_HIGH);
	}	
#endif

#ifndef OPPO_CMCC_TEST
	if(false == oppo_chg_check_time_is_good(chip))
	{
		chg_err("oppo_chg_check_time_is_good func ,false!\n");
		oppo_chg_voter_charging_stop(chip, CHG_STOP_VOTER__MAX_CHGING_TIME);
	}
#endif

	if(chip->chg_ctrl_by_lcd) {
		oppo_chg_check_led_on_ichging(chip);
	}

	if(chip->chg_ctrl_by_camera) {
		oppo_chg_check_camera_on_ichging(chip);	
	}
}


static void battery_notify_tbat_check(struct oppo_chg_chip *chip)
{
	if(BATTERY_STATUS__HIGH_TEMP == chip->tbatt_status)
	{
		if(chip->charger_exist)
		{
				chip->notify_code |= 1 << Notify_Bat_Over_Temp;
				charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_temp(%d) > 55'C\n", chip->temperature);
		}
	}
	
	if(BATTERY_STATUS__LOW_TEMP == chip->tbatt_status )
	{
		if(chip->charger_exist)
		{
			chip->notify_code |= 1 << Notify_Bat_Low_Temp;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_temp(%d) < -10'C\n", chip->temperature);
		}
	}

	
	if(BATTERY_STATUS__REMOVED == chip->tbatt_status)
	{
		chip->notify_code |= 1 << Notify_Bat_Not_Connect;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_temp(%d) < -19'C\n", chip->temperature);
	}
        
}

static void battery_notify_authenticate_check(struct oppo_chg_chip *chip)
{
	if (!chip->authenticate) {
		chip->notify_code |= 1 << Notify_Bat_Not_Connect;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_authenticate is false!\n");
	}
}

static void battery_notify_vcharger_check(struct oppo_chg_chip *chip)
{
    if(CHARGER_STATUS__VOL_HIGH == chip->vchg_status)
	{
		chip->notify_code |= 1 << Notify_Charger_Over_Vol;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] check_charger_off_vol(%d) > 5800mV\n", chip->charger_volt);
	}
			
	if(	CHARGER_STATUS__VOL_LOW == chip->vchg_status)
	{
		chip->notify_code |= 1 << Notify_Charger_Low_Vol;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] check_charger_off_vol(%d) < 3400mV\n", chip->charger_volt);
	} 
}

static void battery_notify_vbat_check(struct oppo_chg_chip *chip)
{
    if(true == chip->vbatt_over)
	{
		chip->notify_code |= 1 << Notify_Bat_Over_Vol;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] Battery is over VOL! Notify \n");
	}
	else
	{
	
		if ((chip->batt_full) && (chip->charger_exist))
		{
			if(chip->tbatt_status == BATTERY_STATUS__WARM_TEMP)
			{
				chip->notify_code |=  1 << Notify_Bat_Full_Pre_High_Temp;
			}
			else if((chip->tbatt_status == BATTERY_STATUS__COLD_TEMP))
			{
				chip->notify_code |=  1 << Notify_Bat_Full_Pre_Low_Temp2;
			}
			else if(!chip->authenticate)
			{
				//chip->notify_code |=  1 << Notify_Bat_Full_THIRD_BATTERY;
			}
			else
			{
				if(chip->ui_soc == 100)
					chip->notify_code |=  1 << Notify_Bat_Full;
			}
			
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] FULL,tbatt_status:%d,notify_code:%d\n", 
				chip->tbatt_status, chip->notify_code);
		}
	}
}

static void battery_notify_max_charging_time_check(struct oppo_chg_chip *chip)
{
	if(true == chip->chging_over_time)
	{
		chip->notify_code |= 1 << Notify_Chging_OverTime;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] Charging is OverTime!Notify \n");
	}
}

static void battery_notify_flag_check(struct oppo_chg_chip *chip)
{
	if(chip->notify_code & (1 << Notify_Chging_OverTime))
	{
		chip->notify_flag = Notify_Chging_OverTime;
	}
	else if(chip->notify_code & (1 << Notify_Charger_Over_Vol))
	{
		chip->notify_flag = Notify_Charger_Over_Vol;
	}
	else if(chip->notify_code & (1 << Notify_Charger_Low_Vol))
	{
		chip->notify_flag = Notify_Charger_Low_Vol;
	}
	else if(chip->notify_code & (1 << Notify_Bat_Over_Temp))
	{
		chip->notify_flag = Notify_Bat_Over_Temp;
	}
	else if(chip->notify_code & (1 << Notify_Bat_Low_Temp))
	{
		chip->notify_flag = Notify_Bat_Low_Temp;
	}
	else if(chip->notify_code & (1 << Notify_Bat_Not_Connect))
	{
		chip->notify_flag = Notify_Bat_Not_Connect;
	}
	else if(chip->notify_code & (1 << Notify_Bat_Over_Vol))
	{
		chip->notify_flag = Notify_Bat_Over_Vol;
	}
	else if(chip->notify_code & (1 << Notify_Bat_Full_Pre_High_Temp))
	{
		chip->notify_flag = Notify_Bat_Full_Pre_High_Temp;
	}
	else if(chip->notify_code & (1 << Notify_Bat_Full_Pre_Low_Temp2))
	{
		chip->notify_flag = Notify_Bat_Full_Pre_Low_Temp2;
	}
	else if(chip->notify_code & (1 << Notify_Bat_Full))
	{
		chip->notify_flag = Notify_Bat_Full;
	}
	else
	{
		chip->notify_flag = 0;
	}
}



static void oppo_chg_battery_notify_check(struct oppo_chg_chip *chip)
{	
	chip->notify_code = 0x0000;
	
	battery_notify_tbat_check(chip);

	battery_notify_authenticate_check(chip);
	
	battery_notify_vcharger_check(chip);

	battery_notify_vbat_check(chip);
	
	battery_notify_max_charging_time_check(chip);
	
	battery_notify_flag_check(chip);
}

int oppo_chg_get_prop_batt_health(struct oppo_chg_chip *chip)
{
	int bat_health= POWER_SUPPLY_HEALTH_GOOD;
	bool vbatt_over = chip->vbatt_over;
	OPPO_CHG_TBATT_STATUS tbatt_status = chip->tbatt_status;

	if(vbatt_over == true)
	{
		bat_health=POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}
	else if(tbatt_status==BATTERY_STATUS__REMOVED)	
	{
		bat_health=POWER_SUPPLY_HEALTH_DEAD;
	}
	else if(tbatt_status==BATTERY_STATUS__HIGH_TEMP)
	{
		bat_health=POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	else if(tbatt_status==BATTERY_STATUS__LOW_TEMP)
	{
		bat_health=POWER_SUPPLY_HEALTH_COLD;
	}
	else 
	{
		bat_health=POWER_SUPPLY_HEALTH_GOOD;
	}

	return bat_health;
}

static bool oppo_chg_soc_reduce_slow_when_1(struct oppo_chg_chip *chip)
{
	static int reduce_count = 0;
	int reduce_count_limit = 0;

	if(chip->batt_exist == false)
		return false;
	
	if(chip->charger_exist)
		reduce_count_limit = 12;
	else
		reduce_count_limit = 4;
	
	if(chip->batt_volt < 3410)
		reduce_count++;
	else
		reduce_count = 0;
	charger_xlog_printk(CHG_LOG_CRTI, "batt_vol:%d,reduce_count:%d\n", chip->batt_volt,reduce_count);
	if(reduce_count > reduce_count_limit){
		reduce_count = reduce_count_limit + 1;
		return true;
	} else {
		return false;
	}
}

#define SOC_SYNC_UP_RATE_10S  				2
#define SOC_SYNC_UP_RATE_60S  				12
#define SOC_SYNC_DOWN_RATE_300S				60
#define SOC_SYNC_DOWN_RATE_150S 			30
#define SOC_SYNC_DOWN_RATE_90S				18
#define SOC_SYNC_DOWN_RATE_60S 				12
#define SOC_SYNC_DOWN_RATE_40S 				8
#define SOC_SYNC_DOWN_RATE_30S 				6
#define SOC_SYNC_DOWN_RATE_15S 				3
#define TEN_MINUTES							600

static void oppo_chg_update_ui_soc(struct oppo_chg_chip *chip)
{
	static int soc_down_count = 0, soc_up_count = 0, ui_soc_pre = 50;
	int soc_down_limit = 0,soc_up_limit = 0;
	unsigned long sleep_tm = 0 , soc_reduce_margin = 0;
	bool vbatt_too_low = false;

	if(chip->ui_soc == 100) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_300S;
	} else if(chip->ui_soc >= 95) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_150S;
	} else if(chip->ui_soc >= 60) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_60S;
	} else if(chip->charger_exist && chip->ui_soc == 1) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_90S;
	} else {
		soc_down_limit = SOC_SYNC_DOWN_RATE_40S;
	}

	if(chip->batt_exist && chip->batt_volt < 3300 && chip->batt_volt > 2500) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_15S;
		vbatt_too_low = true;
		charger_xlog_printk(CHG_LOG_CRTI, "batt_volt:%d, vbatt_too_low:%d\n", chip->batt_volt, vbatt_too_low);
	}
	if(chip->batt_full) {
		soc_up_limit = SOC_SYNC_UP_RATE_60S;
	} else {
		soc_up_limit = SOC_SYNC_UP_RATE_10S;
	}
	if(chip->charger_exist && chip->batt_exist && chip->batt_full && chip->mmi_chg) {
		chip->sleep_tm_sec = 0;
		if((chip->tbatt_status == BATTERY_STATUS__NORMAL) || (chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP)
			|| (chip->tbatt_status == BATTERY_STATUS__COOL_TEMP) || (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP)) {
			soc_down_count = 0;
			soc_up_count++;
			if(soc_up_count >= soc_up_limit) {
    				soc_up_count = 0;
				chip->ui_soc++;
    			}  
			if(chip->ui_soc >= 100) {
				chip->ui_soc = 100;
				chip->prop_status = POWER_SUPPLY_STATUS_FULL;
			} else {
				chip->prop_status = POWER_SUPPLY_STATUS_CHARGING;
			}
		} else {
			chip->prop_status = POWER_SUPPLY_STATUS_FULL;	
		}
		if(chip->ui_soc != ui_soc_pre)
			charger_xlog_printk(CHG_LOG_CRTI, "full ui_soc:%d,soc:%d,up_limit:%d\n", chip->ui_soc, chip->soc, soc_up_limit);
	} else if(chip->charger_exist && chip->batt_exist && (CHARGING_STATUS_FAIL != chip->charging_state) && chip->mmi_chg) {
		chip->sleep_tm_sec = 0;
		chip->prop_status = POWER_SUPPLY_STATUS_CHARGING;
		if(chip->soc == chip->ui_soc){
			soc_down_count = 0;
			soc_up_count = 0;
			//charger_xlog_printk(CHG_LOG_CRTI, "[Battery] Can sync due to ui_soc=%d, soc=%d\r\n",  chip->ui_soc, chip->soc);
		} else if(chip->soc > chip->ui_soc) {
			soc_down_count = 0;
			soc_up_count++;
			if(soc_up_count >= soc_up_limit) {
				soc_up_count = 0;
				chip->ui_soc++;
			}
		} else if(chip->soc < chip->ui_soc) {
			soc_up_count = 0;
			soc_down_count++;
			if(soc_down_count >= soc_down_limit) {
				soc_down_count = 0;
				chip->ui_soc--;
			}
		}
		if(chip->ui_soc != ui_soc_pre) {
			charger_xlog_printk(CHG_LOG_CRTI, "charging ui_soc:%d,soc:%d,down_limit:%d,up_limit:%d\n", 
				chip->ui_soc, chip->soc, soc_down_limit, soc_up_limit);
		}
	} else {
		chip->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		soc_up_count = 0;
		if (chip->soc <= chip->ui_soc || vbatt_too_low) {
			if (soc_down_count > soc_down_limit)
				soc_down_count = soc_down_limit + 1;
			else
				soc_down_count++;
			sleep_tm = chip->sleep_tm_sec;
			if(chip->sleep_tm_sec > 0) {
				soc_reduce_margin = chip->sleep_tm_sec / TEN_MINUTES;
				if(soc_reduce_margin == 0) {
					if((chip->ui_soc - chip->soc) > 5) {
						chip->ui_soc--;
						soc_down_count = 0;
						chip->sleep_tm_sec = 0;
					}
				} else if(soc_reduce_margin < (chip->ui_soc - chip->soc)) {
					chip->ui_soc -= soc_reduce_margin;
					soc_down_count = 0;
					chip->sleep_tm_sec = 0;
				} else if(soc_reduce_margin >= (chip->ui_soc - chip->soc)) {
					chip->ui_soc = chip->soc;
					soc_down_count = 0;
					chip->sleep_tm_sec = 0;
				}
			}
			if (soc_down_count >= soc_down_limit && (chip->soc < chip->ui_soc || vbatt_too_low)){
				chip->sleep_tm_sec = 0;
				soc_down_count = 0;
				chip->ui_soc--;
			}
		}
		if(chip->ui_soc != ui_soc_pre) {
			charger_xlog_printk(CHG_LOG_CRTI, "discharging ui_soc:%d,soc:%d,down_limit:%d,sleep_tm:%ld\n",
				chip->ui_soc, chip->soc, soc_down_limit, sleep_tm);
		}
	}

	if(chip->ui_soc < 2) {
		if(oppo_chg_soc_reduce_slow_when_1(chip) == true)
			chip->ui_soc = 0;
		else
			chip->ui_soc = 1;
	}
	if (chip->ui_soc != ui_soc_pre) {
		ui_soc_pre = chip->ui_soc;
		chip->chg_ops->set_rtc_soc(chip->ui_soc);
		if (chip->chg_ops->get_rtc_soc() != chip->ui_soc) {
			charger_xlog_printk(CHG_LOG_CRTI, "set soc fail:[%d, %d], try again...\n", chip->ui_soc, chip->chg_ops->get_rtc_soc());
			chip->chg_ops->set_rtc_soc(chip->ui_soc);
		}
	}
}

static void battery_update(struct oppo_chg_chip *chip)
{
	chip->batt_fcc = oppo_gauge_get_batt_fcc();
	chip->batt_cc = oppo_gauge_get_batt_cc();
	chip->batt_soh = oppo_gauge_get_batt_soh();
	chip->batt_rm = oppo_gauge_get_remaining_capacity();
	
	oppo_chg_update_ui_soc(chip);

	power_supply_changed(&chip->batt_psy);
	
 }

static void oppo_chg_battery_update_status(struct oppo_chg_chip *chip)
{
#ifdef CONFIG_OPPO_CHARGER_MTK
	usb_update(chip);
#endif	
	battery_update(chip);
}

#define RESET_MCU_DELAY_15S			3
static void oppo_chg_fast_switch_check(struct oppo_chg_chip *chip)
{
	if(chip->mmi_chg == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, " mmi_chg,return\n"); 
		return ;
	}
	//if((chip->charger_type==STANDARD_CHARGER) || (chip->charger_type==NONSTANDARD_CHARGER) ||(chip->charger_type==APPLE_2_1A_CHARGER) || (chip->charger_type==APPLE_1_0A_CHARGER))
	if(chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP)
	{
		//charger_xlog_printk(CHG_LOG_CRTI, " fast_chg_started:%d\n",oppo_vooc_get_fastchg_started()); 
		if(oppo_vooc_get_fastchg_started() == false) {
			oppo_vooc_switch_fast_chg();
		}
		if(!oppo_vooc_get_fastchg_started() && !oppo_vooc_get_fastchg_dummy_started()) {
			reset_mcu_delay++;
			if(reset_mcu_delay == RESET_MCU_DELAY_15S) {
				charger_xlog_printk(CHG_LOG_CRTI, "  reset mcu again\n"); 
				oppo_vooc_set_ap_clk_high();
				oppo_vooc_reset_mcu();
			} else if(reset_mcu_delay > RESET_MCU_DELAY_15S) {
				reset_mcu_delay = RESET_MCU_DELAY_15S + 1;
			}
		}
	}
}

#define FULL_COUNTS_SW		5
#define FULL_COUNTS_HW		3
static bool oppo_chg_check_vbatt_is_full_by_sw(struct oppo_chg_chip *chip)
{
	static bool ret_sw = false;
	static bool ret_hw = false;
	static int vbat_counts_sw = 0;
	static int vbat_counts_hw = 0;
	int vbatt_full_vol_sw;
	int vbatt_full_vol_hw;

	if (!chip->check_batt_full_by_sw)
		return false;
	
	if (!chip->charger_exist) {
		vbat_counts_sw = 0;
		vbat_counts_hw = 0;
		ret_sw = false;
		ret_hw = false;
		return false;
	}

	vbatt_full_vol_hw = oppo_chg_get_float_voltage(chip);
	if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
		vbatt_full_vol_sw = chip->limits.cold_vfloat_sw_limit;
		//vbatt_full_vol_hw = chip->limits.temp_cold_vfloat_mv;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {
		vbatt_full_vol_sw = chip->limits.little_cold_vfloat_sw_limit;
		//vbatt_full_vol_hw = chip->limits.temp_little_cold_vfloat_mv;
	} else if (chip->tbatt_status == BATTERY_STATUS__COOL_TEMP) {
		vbatt_full_vol_sw = chip->limits.cool_vfloat_sw_limit;
		//vbatt_full_vol_hw = chip->limits.temp_cool_vfloat_mv;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP) {
		vbatt_full_vol_sw = chip->limits.little_cool_vfloat_sw_limit;
		//vbatt_full_vol_hw = chip->limits.temp_little_cool_vfloat_mv;
	} else if (chip->tbatt_status == BATTERY_STATUS__NORMAL) {
		vbatt_full_vol_sw = chip->limits.normal_vfloat_sw_limit;
		//vbatt_full_vol_hw = chip->limits.temp_normal_vfloat_mv_normalchg;
	} else if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP) {
		vbatt_full_vol_sw = chip->limits.warm_vfloat_sw_limit;
		//vbatt_full_vol_hw = chip->limits.temp_warm_vfloat_mv;
	} else {
		vbat_counts_sw = 0;
		vbat_counts_hw = 0;
		ret_sw = 0;
		ret_hw = 0;
		return false;
	}
	if (!chip->authenticate) {
		vbatt_full_vol_sw = chip->limits.non_standard_vfloat_sw_limit;
		vbatt_full_vol_hw = chip->limits.non_standard_vfloat_mv;
	}

	/* use SW Vfloat to check */
	if (chip->batt_volt > vbatt_full_vol_sw) {
		if (chip->icharging < 0 && (chip->icharging * -1) <= chip->limits.iterm_ma) {
			vbat_counts_sw++;
			if (vbat_counts_sw > FULL_COUNTS_SW) {
				vbat_counts_sw = 0;
				ret_sw = true;
			}
		} else if (chip->icharging >= 0) {
			vbat_counts_sw++;
			if (vbat_counts_sw > FULL_COUNTS_SW * 2) {
				vbat_counts_sw = 0;
				ret_sw = true;
				charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] Battery full by sw when icharging>=0!!\n");
			}
		} else {
			vbat_counts_sw = 0;
			ret_sw = false;
		}
	} else {
		vbat_counts_sw = 0;
		ret_sw = false;
	}
	
	/* use HW Vfloat to check */
	if (chip->batt_volt >= vbatt_full_vol_hw + 18) {
		vbat_counts_hw++;
		if (vbat_counts_hw >= FULL_COUNTS_HW) {
			vbat_counts_hw = 0;
			ret_hw = true;
		}
	} else {
		vbat_counts_hw = 0;
		ret_hw = false;
	}
	
	if (ret_sw == true || ret_hw == true) {
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] Battery full by sw[%s] !!\n", (ret_sw == true) ? "S" : "H");
		ret_sw = ret_hw = false;
		return true;
	} else {
		return false;
	}
}

#define FULL_DELAY_COUNTS		4
static void oppo_chg_check_status_full(struct oppo_chg_chip *chip)
{
    int is_batt_full = 0;
	static int fastchg_present_wait_count = 0;
	
	if(oppo_vooc_get_allow_reading() == false)
	{
		is_batt_full = 0;
		fastchg_present_wait_count = 0;
	}
	else
	{
		if(((oppo_vooc_get_fastchg_to_normal()== true) || (oppo_vooc_get_fastchg_to_warm() == true))	&& (fastchg_present_wait_count <= FULL_DELAY_COUNTS))		
		{		
			is_batt_full = 0;			
			fastchg_present_wait_count++;
			if(fastchg_present_wait_count == FULL_DELAY_COUNTS && chip->chg_ops->get_charging_enable(chip) == false
				&& chip->charging_state != CHARGING_STATUS_FULL) {
				oppo_chg_turn_on_charging(chip);
			}
			//chg_err("bq24196_check_status,fastchg_present_wait_count = %d\r\n",fastchg_present_wait_count);
		} else {
			is_batt_full = chip->chg_ops->read_full(chip);
			fastchg_present_wait_count = 0;
		}
	}
	
    //chg_err("  is_batt_full=%d,bat_vol=%d,BMT_status.bat_charging_state = %d\n",is_batt_full,chip->batt_volt,chip->charging_state);
    
	if((is_batt_full == 1) || (chip->charging_state == CHARGING_STATUS_FULL) || oppo_chg_check_vbatt_is_full_by_sw(chip)) {
		oppo_chg_full_action(chip);
		if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP || chip->tbatt_status == BATTERY_STATUS__COOL_TEMP
				|| chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP || chip->tbatt_status == BATTERY_STATUS__NORMAL) {
			oppo_gauge_set_batt_full(true);
		}
    } else if(chip->charging_state == CHARGING_STATUS_FAIL) {
		oppo_chg_fail_action(chip);
		/*MoFei@EXP.BaseDrv.charge,2016/05/10 motify for charging status */
		oppo_gauge_set_batt_full(false);
	} else {
		chip->charging_state = CHARGING_STATUS_CCCV;
		/*MoFei@EXP.BaseDrv.charge,2016/05/10 add for charging  status*/
		oppo_gauge_set_batt_full(false);
	}
}

static void oppo_chg_kpoc_power_off_check(struct oppo_chg_chip *chip)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if(chip->boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || chip->boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
	{
		if( (chip->chg_ops->check_chrdet_status() == false)  && (chip->charger_volt < 2500))	//vbus < 2.5V
		{
			if((oppo_vooc_get_fastchg_to_normal() == false) && (oppo_vooc_get_fastchg_to_warm() == false)
				&& (oppo_vooc_get_adapter_update_status() != ADAPTER_FW_NEED_UPDATE)
				&& (oppo_vooc_get_btb_temp_over() == false))
			{
				charger_xlog_printk(CHG_LOG_CRTI, "[pmic_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
				//primary_display_suspend();
				//battery_charging_control(CHARGING_CMD_SET_POWER_OFF,NULL);
				chip->chg_ops->set_power_off();
			}		
		}
	}
#endif	
}

static void oppo_chg_print_log(struct oppo_chg_chip *chip)
{

#if 0
// wenbin.liu@SW.Bsp.Driver, 2016/02/29  Delete for log tag 
#ifdef FEATURE_PRINT_CHGR_LOG
	charger_xlog_printk(CHG_LOG_CRTI,"[VENDOR CHGR] chgr_exist = %d, chgr_type = %d,chgr_vol = %d, g_boot_mode = %d, otg_online = %d\n",
		chip->charger_exist, chip->charger_type, chip->charger_volt, chip->boot_mode, chip->otg_online);
#endif

#ifdef FEATURE_PRINT_BAT_LOG
	charger_xlog_printk(CHG_LOG_CRTI,"[VENDOR BAT] bat_exist = %d, bat_full = %d, chging_on = %d, in_rechg = %d, chging_state = %d, total_time = %d\n",
		chip->batt_exist, chip->batt_full, chip->chging_on, chip->in_rechging, chip->charging_state, chip->total_time);
#endif

#ifdef FEATURE_PRINT_GAUGE_LOG
	charger_xlog_printk(CHG_LOG_CRTI,"[VENDOR GAUGE] temp = %d, bat_vol = %d, icharging = %d, soc = %d, ui_soc = %d, soc_load = %d\n",
		chip->temperature, chip->batt_volt, chip->icharging, chip->soc, chip->ui_soc, chip->soc_load);
#endif

#ifdef FEATURE_PRINT_STATUS_LOG
	charger_xlog_printk(CHG_LOG_CRTI,"[VENDOR STATUS] vbatt_over = 0x%x, over_time = %d, vchg_status = %d,  tbatt_status = %d, stop_voter = %d,  notify_code = 0x%x\n",
		chip->vbatt_over, chip->chging_over_time, chip->vchg_status, chip->tbatt_status, chip->stop_voter, chip->notify_code);
#endif

#ifdef FEATURE_PRINT_OTHER_LOG
	charger_xlog_printk(CHG_LOG_CRTI,"[VENDOR OTHER] otg_switch = %d, mmi_chg = %d, g_boot_reason = %d, g_boot_mode = %d\r\n",
	chip->otg_switch, chip->mmi_chg, chip->boot_reason, chip->boot_mode);
#endif
#endif

// wenbin.liu@SW.Bsp.Driver, 2016/02/29  Add for log tag 
	charger_xlog_printk(CHG_LOG_CRTI," CHGR[ %d / %d / %d / %d / %d], BAT[ %d / %d / %d / %d / %d / %d ], GAUGE[ %d / %d / %d / %d / %d / %d ], \
STATUS[ 0x%x / %d / %d / %d / %d / 0x%x ], OTHER[ %d / %d / %d / %d ]\n",
	chip->charger_exist, chip->charger_type, chip->charger_volt, chip->boot_mode,chip->otg_online, 
	chip->batt_exist, chip->batt_full, chip->chging_on, chip->in_rechging, chip->charging_state, chip->total_time, 
	chip->temperature, chip->batt_volt, chip->icharging, chip->soc, chip->ui_soc, chip->soc_load, 
	chip->vbatt_over, chip->chging_over_time, chip->vchg_status, chip->tbatt_status, chip->stop_voter, chip->notify_code, 
	chip->otg_switch, chip->mmi_chg, chip->boot_reason, chip->boot_mode);


#ifdef FEATURE_PRINT_FASTCHG_LOG	
	oppo_vooc_print_log();
#endif

}

static void oppo_chg_other_thing(struct oppo_chg_chip *chip)
{
	if(oppo_vooc_get_allow_reading() == true){
		oppo_chg_vfloat_over_check(chip);
		chip->chg_ops->kick_wdt(chip);
		chip->chg_ops->dump_registers(chip);
	}
	if(chip->charger_exist)
		chip->total_time += OPPO_CHG_UPDATE_INTERVAL_SEC;
	oppo_chg_print_log(chip);
}

#ifndef CONFIG_OPPO_SURPORT_PID 
/*MoFei@EXP.BaseDrv.charge,2016/06/12 add for support pid algorithm*/
#define IFLOAT_COUNT	10
#define IFLOAT_STEP_MA	30
static void oppo_chg_ifloat_check_and_set(struct oppo_chg_chip *chip)
{
	static int ichg = 0;
	static int ifloat_check_count = 0;
	static int current_ma = 0;
	bool set_current_flag = false;

	if (current_ma == 0) {
		current_ma = chip->limits.temp_little_cold_fastchg_current_ma;
	}

	if (!chip->charger_exist || chip->tbatt_status != BATTERY_STATUS__LITTLE_COLD_TEMP) {
		current_ma = chip->limits.temp_little_cold_fastchg_current_ma;
		ifloat_check_count = 0;
		ichg = 0;
		return;
	}

	if (chip->batt_volt > 4200 || chip->led_on) {
		ifloat_check_count = 0;
		ichg = 0;
		return;
	}

	if (chip->icharging < 0) {
		ifloat_check_count ++;
		ichg = ichg + chip->icharging;
	}

	if (ifloat_check_count == IFLOAT_COUNT) {
		ichg = -1 * ichg / ifloat_check_count;
		if (ichg < chip->limits.temp_little_cold_fastchg_current_ma - IFLOAT_STEP_MA) {
			ichg = current_ma + IFLOAT_STEP_MA;
			set_current_flag = true;
		} else if (ichg > chip->limits.temp_little_cold_fastchg_current_ma + IFLOAT_STEP_MA) {
			ichg = current_ma - IFLOAT_STEP_MA;
			set_current_flag = true;
		} else {
			ifloat_check_count = 0;
			ichg = 0;
		}
	}

	if (set_current_flag == true) {
		charger_xlog_printk(CHG_LOG_CRTI, "charging_current_write_fast[%d]\n", ichg);
		if (ichg > chip->limits.temp_normal_fastchg_current_ma) {
			ichg = chip->limits.temp_normal_fastchg_current_ma;
		} else if (ichg < 103) {//(512*20%)
			ichg = 103;
		}
		chip->chg_ops->charging_current_write_fast(chip, ichg);
		current_ma = ichg;
		ifloat_check_count = 0;
		ichg = 0;
	}
}
#endif /*CONFIG_OPPO_SURPORT_PID*/

static void oppo_chg_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oppo_chg_chip *chip = container_of(dwork, struct oppo_chg_chip, update_work);
	
	oppo_charger_detect_check(chip);

	oppo_chg_get_battery_data(chip);

	if (chip->charger_exist)
	{
		oppo_chg_aicl_check(chip);
		oppo_chg_protection_check(chip);
		oppo_chg_check_vbatt_higher_than_4180mv(chip);
		oppo_chg_battery_notify_check(chip);
		oppo_chg_check_status_full(chip);
		oppo_chg_fast_switch_check(chip);
	}
#ifndef CONFIG_OPPO_SURPORT_PID 
/*MoFei@EXP.BaseDrv.charge,2016/06/12 add for support pid algorithm*/
	oppo_chg_ifloat_check_and_set(chip);
#endif

	oppo_chg_battery_update_status(chip);
	
	oppo_chg_kpoc_power_off_check(chip);
	
	oppo_chg_other_thing(chip);

	/* run again after interval */
	schedule_delayed_work(&chip->update_work, OPPO_CHG_UPDATE_INTERVAL);
}

bool oppo_chg_wake_update_work(void)
{
	int shedule_work = 0;

	if(!g_charger_chip) {
		chg_err(" g_charger_chip NULL,return\n");
		return true;
	}
	shedule_work = mod_delayed_work(system_wq, &g_charger_chip->update_work, 0);
	
	return true;
}

void oppo_chg_kick_wdt(void)
{
	if(!g_charger_chip) {
		return ;
	}
	if(oppo_vooc_get_allow_reading() == true)
		g_charger_chip->chg_ops->kick_wdt(g_charger_chip);
}

void oppo_chg_disable_charge(void)
{
	if(!g_charger_chip) {
		return ;
	}
	if(oppo_vooc_get_allow_reading() == true)
		g_charger_chip->chg_ops->charging_disable(g_charger_chip, false);
}

void oppo_chg_unsuspend_charger(void)
{
	if(!g_charger_chip) {
		return ;
	}
	if(oppo_vooc_get_allow_reading() == true)
		g_charger_chip->chg_ops->charger_unsuspend(g_charger_chip);
}

int oppo_chg_get_batt_volt(void)
{
	if(!g_charger_chip)
		return 4000;
	else
		return g_charger_chip->batt_volt;
}

int oppo_chg_get_ui_soc(void)
{
	if(!g_charger_chip)
		return 50;
	else
		return g_charger_chip->ui_soc;
}

int oppo_chg_get_soc(void)
{
	if(!g_charger_chip)
		return 50;
	else
		return g_charger_chip->soc;
}

void oppo_chg_soc_update_when_resume(unsigned long sleep_tm_sec)
{
	if(!g_charger_chip)
		return ;
	g_charger_chip->sleep_tm_sec = sleep_tm_sec;
	g_charger_chip->soc = oppo_gauge_get_batt_soc();
	oppo_chg_update_ui_soc(g_charger_chip);
}

int oppo_chg_get_chg_type(void)
{
	if(!g_charger_chip)
		return POWER_SUPPLY_TYPE_UNKNOWN; 	
	else
		return g_charger_chip->charger_type;
}

int oppo_chg_get_chg_temperature(void)
{
	if(!g_charger_chip)
		return 250;		
	else
		return g_charger_chip->temperature;
}

int oppo_chg_get_notify_flag(void)
{
	if(!g_charger_chip)
		return 0;		
	else
		return g_charger_chip->notify_flag;
}

int oppo_chg_show_vooc_logo_ornot(void)
{
	if(!g_charger_chip)
		return 0;
	
	if(oppo_vooc_get_fastchg_started()) {
		return 1;
	} else if (oppo_vooc_get_fastchg_to_normal() == true 
		|| oppo_vooc_get_fastchg_to_warm() == true
		|| oppo_vooc_get_fastchg_dummy_started() == true 
		|| oppo_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE) {
		if(g_charger_chip->prop_status != POWER_SUPPLY_STATUS_FULL && 
			(g_charger_chip->stop_voter == CHG_STOP_VOTER__FULL || g_charger_chip->stop_voter == CHG_STOP_VOTER_NONE))
			return 1;
		else
			return 0;
	} else {
		return 0;
	}
}

bool get_otg_switch(void)
{
	if(!g_charger_chip)
		return 0;
		
	else
		return g_charger_chip->otg_switch;
}

bool get_otg_online(void)
{
	if(!g_charger_chip)
		return 0;
		
	else
		return g_charger_chip->otg_online;
}

void set_otg_online(bool online)
{
	if(g_charger_chip)
		g_charger_chip->otg_online = online;
}



bool oppo_chg_get_batt_full(void)
{
	if(!g_charger_chip)
		return 0;		
	else
		return g_charger_chip->batt_full;
}

bool oppo_chg_get_rechging_status(void)
{
	if(!g_charger_chip)
		return 0;		
	else
		return g_charger_chip->in_rechging;
}


bool oppo_chg_check_chip_is_null(void)
{
	if(!g_charger_chip) 
		return true;
	else
		return false;
}

void oppo_chg_set_charger_type_unknown(void)
{
	if(g_charger_chip) {
		g_charger_chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	} 
}

int oppo_chg_get_charger_voltage(void)
{
	if(!g_charger_chip) {
		return -500;
	} else {
		return g_charger_chip->chg_ops->get_charger_volt();
	}
}

