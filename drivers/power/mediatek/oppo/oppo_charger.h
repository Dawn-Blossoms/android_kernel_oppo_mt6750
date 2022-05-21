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
***********************************************************************************/



#ifndef _OPPO_CHARGER_H_
#define _OPPO_CHARGER_H_

#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#elif CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#ifdef CONFIG_OPPO_CHARGER_MTK
#include <linux/i2c.h>
//#include <mach/charging.h> //charger type
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#else
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/qpnp/qpnp-adc.h>
#endif

#define CHG_LOG_CRTI 1
#define CHG_LOG_FULL 2


#define OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA        2000
#define OPCHG_INPUT_CURRENT_LIMIT_USB_MA        	500
#define OPCHG_INPUT_CURRENT_LIMIT_LED_MA        	1200
#define OPCHG_INPUT_CURRENT_LIMIT_CAMERA_MA        	1000

#define OPCHG_FAST_CHG_MAX_MA                   2000

#define FEATURE_PRINT_CHGR_LOG
#define FEATURE_PRINT_BAT_LOG
#define FEATURE_PRINT_GAUGE_LOG
#define FEATURE_PRINT_STATUS_LOG
//#define FEATURE_PRINT_OTHER_LOG
#define FEATURE_PRINT_FASTCHG_LOG

#define FEATURE_PRINT_VOTE_LOG
#define FEATURE_PRINT_ICHGING_LOG

#define FEATURE_VBAT_PROTECT

#define     Notify_Charger_Over_Vol                   	1 
#define     Notify_Charger_Low_Vol                    	2 
#define     Notify_Bat_Over_Temp                      	3
#define     Notify_Bat_Low_Temp                       	4
#define     Notify_Bat_Not_Connect                    	5
#define     Notify_Bat_Over_Vol                       	6
#define     Notify_Bat_Full                           	7
#define     Notify_Chging_Current                     	8
#define		Notify_Chging_OverTime					  	9
#define		Notify_Bat_Full_Pre_High_Temp			  	10
#define		Notify_Bat_Full_Pre_Low_Temp2			  	11
#define		Notify_Bat_Full_THIRD_BATTERY			  	14



// wenbin.liu@SW.Bsp.Driver, 2016/03/15 Add for add log tag 
#define chg_debug(fmt, ...) \
	printk(KERN_NOTICE pr_fmt("[OPPO_CHG][%s]"fmt),__func__,##__VA_ARGS__)

#define chg_err(fmt, ...) \
	printk(KERN_ERR pr_fmt("[OPPO_CHG][%s]"fmt),__func__,##__VA_ARGS__)

#if 0
#define dev_err(dev, format, ...)  printk(KERN_ERR pr_fmt("[OPPO_CHG][%s]%s %s:"format),\
	__func__,dev_driver_string(dev),dev_name(dev), ##__VA_ARGS__);
#endif

#if 0
typedef enum {
	CHARGER_UNKNOWN = 0,
	STANDARD_HOST,		/* USB : 450mA */
	CHARGING_HOST,
	NONSTANDARD_CHARGER,	/* AC : 450mA~1A */
	STANDARD_CHARGER,	/* AC : ~1A */
	APPLE_2_1A_CHARGER,	/* 2.1A apple charger */
	APPLE_1_0A_CHARGER,	/* 1A apple charger */
	APPLE_0_5A_CHARGER,	/* 0.5A apple charger */
	WIRELESS_CHARGER,
}OPPO_CHG_CHARGER_TYPE;

#endif



typedef enum
{
	CHG_STOP_VOTER_NONE							=	0,
	CHG_STOP_VOTER__BATTTEMP_ABNORMAL			=	(1 << 0),
	CHG_STOP_VOTER__VCHG_ABNORMAL				= 	(1 << 1),
	CHG_STOP_VOTER__VBAT_TOO_HIGH				=	(1 << 2),
	CHG_STOP_VOTER__MAX_CHGING_TIME				=	(1 << 3),
	CHG_STOP_VOTER__FULL						=	(1 << 4),

}OPPO_CHG_STOP_VOTER;


typedef enum
{
	CHARGER_STATUS__GOOD,
	CHARGER_STATUS__VOL_HIGH,
	CHARGER_STATUS__VOL_LOW,
	CHARGER_STATUS__INVALID
}OPPO_CHG_VCHG_STATUS;


typedef enum
{
	BATTERY_STATUS__NORMAL = 0,				/*16C~45C*/
	BATTERY_STATUS__REMOVED,				/*<-20C*/
	BATTERY_STATUS__LOW_TEMP,				/*<-3C*/
	BATTERY_STATUS__HIGH_TEMP,				/*>55C*/
	BATTERY_STATUS__COLD_TEMP,				/*-3C~0C*/
	BATTERY_STATUS__LITTLE_COLD_TEMP,		/*0C~5C*/
	BATTERY_STATUS__COOL_TEMP,				/*5C~12C*/
	BATTERY_STATUS__LITTLE_COOL_TEMP,		/*12C~16C*/
	BATTERY_STATUS__WARM_TEMP,				/*45C~55C*/
	BATTERY_STATUS__INVALID
}OPPO_CHG_TBATT_STATUS;



typedef enum
{
	CHARGING_STATUS_CCCV 		=	0X01,
	CHARGING_STATUS_FULL   		=	0X02,
	CHARGING_STATUS_FAIL    	=	0X03,
}OPPO_CHG_CHARGING_STATUS;

struct tbatt_anti_shake{
	int cold_bound;
	int little_cold_bound;
	int cool_bound;
	int little_cool_bound;
	int normal_bound;
	int warm_bound;
	int hot_bound;
};

struct oppo_chg_limits {
	int input_current_charger_ma;
	int input_current_usb_ma;
	int input_current_led_ma;
	int input_current_led_ma_forcmcc;
	int input_current_camera_ma;
	int iterm_ma;
	bool iterm_disabled;
	int recharge_mv;

	int removed_bat_decidegc; //-19C
	
	int cold_bat_decidegc;  //-3C
	int temp_cold_vfloat_mv;
	int temp_cold_fastchg_current_ma;

	int little_cold_bat_decidegc;  //0C
	int temp_little_cold_vfloat_mv;
	int temp_little_cold_fastchg_current_ma;
	
	int cool_bat_decidegc;	//5C
	int temp_cool_vfloat_mv;
	int temp_cool_fastchg_current_ma_high;
	int temp_cool_fastchg_current_ma_low;
	
	int little_cool_bat_decidegc;	//12C
	int temp_little_cool_vfloat_mv;
	int temp_little_cool_fastchg_current_ma;
	
	int normal_bat_decidegc;	//16C
	int temp_normal_fastchg_current_ma;
	int temp_normal_vfloat_mv_normalchg;
	int temp_normal_vfloat_mv_voocchg;
	
	int warm_bat_decidegc;		//45C
	int temp_warm_vfloat_mv;
	int temp_warm_fastchg_current_ma;
	
	int hot_bat_decidegc;		//53C
	int non_standard_vfloat_mv;
	int non_standard_fastchg_current_ma;
	int max_chg_time_sec;
	int charger_hv_thr;
	int charger_lv_thr;
	int vbatt_full_thr;
	int vbatt_hv_thr;
	int vfloat_step_mv;

	int non_standard_vfloat_sw_limit;
	int cold_vfloat_sw_limit;
	int little_cold_vfloat_sw_limit;
	int cool_vfloat_sw_limit;
	int little_cool_vfloat_sw_limit;
	int normal_vfloat_sw_limit;
	int warm_vfloat_sw_limit;
	
};

struct battery_data{	
	int		BAT_STATUS;
	int 	BAT_HEALTH;
    int 	BAT_PRESENT;
    int 	BAT_TECHNOLOGY;
    int 	BAT_CAPACITY;
    /* Add for Battery Service*/
    int 	BAT_batt_vol;
    int 	BAT_batt_temp;
	
    /* Add for EM */
    int 	BAT_TemperatureR;
    int 	BAT_TempBattVoltage;
    int 	BAT_InstatVolt;
    int 	BAT_BatteryAverageCurrent;
    int 	BAT_BatterySenseVoltage;
    int 	BAT_ISenseVoltage;
    int 	BAT_ChargerVoltage;
	int 	battery_request_poweroff;//low battery in sleep
	int 	fastcharger;
	int 	charge_technology;
    /* Dual battery */
	int 	BAT_MMI_CHG;			//for MMI_CHG_TEST
	int 	BAT_FCC;
	int 	BAT_SOH;
	int 	BAT_CC;
};

#ifndef CONFIG_OPPO_CHARGER_MTK

#define PPPP

struct smbchg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

struct ilim_entry {
	int vmin_uv;
	int vmax_uv;
	int icl_pt_ma;
	int icl_lv_ma;
	int icl_hv_ma;
};

struct ilim_map {
	int			num;
	struct ilim_entry	*entries;
};

struct parallel_usb_cfg {
	struct power_supply		*psy;
	int				min_current_thr_ma;
	int				min_9v_current_thr_ma;
	int				allowed_lowering_ma;
	int				current_max_ma;
	bool				avail;
	struct mutex			lock;
	int				initial_aicl_ma;
	ktime_t				last_disabled;
	bool				enabled_once;
};

#endif

struct oppo_chg_chip {
	struct i2c_client	*client;
	struct device       *dev;
	const struct oppo_chg_operations *chg_ops;
	
	struct power_supply	ac_psy;
#ifdef CONFIG_OPPO_CHARGER_MTK	
	struct power_supply	usb_psy;
#else
	struct power_supply	*usb_psy;
#endif
	struct power_supply	batt_psy;
//	struct battery_data battery_main;
	struct delayed_work	update_work;
	struct wake_lock	suspend_lock;
	atomic_t charger_suspended;
	
	struct oppo_chg_limits	limits;
	struct tbatt_anti_shake anti_shake_bound;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend chg_early_suspend;
#elif CONFIG_FB
	struct notifier_block chg_fb_notify;
#endif
	
	bool		charger_exist;
	int			charger_type;
	int			charger_volt;
	int 		charger_volt_pre;
	
	int			temperature;
	int			batt_volt;
	int 		vfloat_new;
	int			icharging;
	int 		soc;
	int			ui_soc;
	int			soc_load;
	bool		authenticate;
	int			hw_aicl_point;
	int			sw_aicl_point;

	int			batt_fcc;
	int			batt_cc;
	int			batt_soh;
	int			batt_rm;

	bool		batt_exist;
	bool		batt_full;
	bool		chging_on;
	bool		in_rechging;
	int 		charging_state;
	int			total_time;
	unsigned long sleep_tm_sec;
	
	bool		vbatt_over;
	bool		chging_over_time;
	int			vchg_status;
	int			tbatt_status;
	int			prop_status;
	int			stop_voter;
	int			notify_code;
	int			notify_flag;
	int			request_power_off;

	bool		led_on;
	bool		led_status_change;
	bool		camera_on;

	bool 		ac_online;
#ifdef 	CONFIG_OPPO_CHARGER_MTK
	bool		usb_online;
#endif
	bool 		otg_switch;
	bool 		otg_online;
	int			mmi_chg;
	int			boot_reason;
	int			boot_mode;
	bool		vooc_project;
	bool		suspend_after_full;
	bool		check_batt_full_by_sw;
	bool		external_gauge;
	bool		chg_ctrl_by_lcd;
	bool		chg_ctrl_by_camera;
	bool		aicl_suspend;
	
#ifndef CONFIG_OPPO_CHARGER_MTK
	struct spmi_device		*spmi;
	int				schg_version;
	
	/* peripheral register address bases */
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				otg_base;
	u16				misc_base;

	int				fake_battery_soc;
	u8				revision[4];

	/* configuration parameters */
	int				iterm_ma;
	int				usb_max_current_ma;
	int				dc_max_current_ma;
	int				usb_target_current_ma;
	int				usb_tl_current_ma;
	int				dc_target_current_ma;
	int				target_fastchg_current_ma;
	int				cfg_fastchg_current_ma;
	int				fastchg_current_ma;
	int				vfloat_mv;
	int				fastchg_current_comp;
	int				float_voltage_comp;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				bmd_pin_src;
	int				jeita_temp_hard_limit;
	bool			use_vfloat_adjustments;
	bool			iterm_disabled;
	bool			bmd_algo_disabled;
	bool			soft_vfloat_comp_disabled;
	bool			chg_enabled;
	bool			charge_unknown_battery;
	bool			chg_inhibit_en;
	bool			chg_inhibit_source_fg;
	bool			low_volt_dcin;
	bool			cfg_chg_led_support;
	bool			cfg_chg_led_sw_ctrl;
	bool			vbat_above_headroom;
	bool			force_aicl_rerun;
	bool			hvdcp3_supported;
	u8				original_usbin_allowance;
	struct parallel_usb_cfg		parallel;
	struct delayed_work		parallel_en_work;
	struct dentry			*debug_root;

	/* wipower params */
	struct ilim_map			wipower_default;
	struct ilim_map			wipower_pt;
	struct ilim_map			wipower_div2;
	struct qpnp_vadc_chip		*vadc_dev;
	bool				wipower_dyn_icl_avail;
	struct ilim_entry		current_ilim;
	struct mutex			wipower_config;
	bool				wipower_configured;
	struct qpnp_adc_tm_btm_param	param;

	/* flash current prediction */
	int				rpara_uohm;
	int				rslow_uohm;
	int				vled_max_uv;

	/* vfloat adjustment */
	int				max_vbat_sample;
	int				n_vbat_samples;

	/* status variables */
	int				battchg_disabled;
	int				usb_suspended;
	int				dc_suspended;
	int				wake_reasons;
	int				previous_soc;
	int				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	int				otg_retries;
	ktime_t				otg_enable_time;
	bool				aicl_deglitch_short;
	bool				sw_esr_pulse_en;
	bool				safety_timer_en;
	bool				aicl_complete;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	const char			*battery_type;
	bool				very_weak_charger;
	bool				parallel_charger_detected;
	u32				wa_flags;
	/* jeita and temperature */
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	/* irqs */
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int				recharge_irq;
	int				fastchg_irq;
	int				safety_timeout_irq;
	int				power_ok_irq;
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
	int				usbid_change_irq;
	int				chg_error_irq;
	bool				enable_aicl_wake;

	/* psy */
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	int				dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	bool			other_sdp;

	struct smbchg_regulator		otg_vreg;
	struct smbchg_regulator		ext_otg_vreg;
	struct work_struct		usb_set_online_work;
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		hvdcp_det_work;
	struct delayed_work		dump_reg_work;
	struct delayed_work             update_opchg_thread_work;
    struct delayed_work             opchg_delayed_wakeup_work;
//    struct wakeup_source            source;
	spinlock_t			sec_access_lock;
	struct mutex			current_change_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			battchg_disabled_lock;
	struct mutex			usb_en_lock;
	struct mutex			dc_en_lock;
	struct mutex			fcc_lock;
	struct mutex			pm_lock;
	/* aicl deglitch workaround */
	unsigned long			first_aicl_seconds;
	int				aicl_irq_count;
	struct mutex			usb_status_lock;
	bool				hvdcp_3_det_ignore_uv;
	struct completion		src_det_lowered;
	struct completion		src_det_raised;
	struct completion		usbin_uv_lowered;
	struct completion		usbin_uv_raised;
	int				pulse_cnt;
	struct led_classdev		led_cdev;
	u8       bat_status;
  	bool     suspending;
  	int 	bat_instant_vol;
  	int 	charging_current;
  	int 	input_limit_flags;
  	int 	input_current_max;
  	int 	voltage_max;
  	int 	charging_enable;
  	int 	capacity;
  	char 	*charger_name;
  	int 	bat_charging_state;
#endif
};

struct oppo_chg_operations {
	void (*dump_registers) (struct oppo_chg_chip *chip);
	int (*kick_wdt) (struct oppo_chg_chip *chip);
	int (*hardware_init) (struct oppo_chg_chip *chip);
	int (*charging_current_write_fast) (struct oppo_chg_chip *chip, int cur);	
	void (*set_aicl_point) (struct oppo_chg_chip *chip, int vbatt);
	int (*input_current_write) (struct oppo_chg_chip *chip, int cur);
	int (*float_voltage_write) (struct oppo_chg_chip *chip, int cur);
	int (*term_current_set) (struct oppo_chg_chip *chip, int cur);
	int (*charging_enable) (struct oppo_chg_chip *chip);
	int (*charging_disable) (struct oppo_chg_chip *chip, bool flag);
	int (*get_charging_enable) (struct oppo_chg_chip *chip);
	int (*charger_suspend) (struct oppo_chg_chip *chip);
	int (*charger_unsuspend) (struct oppo_chg_chip *chip);
	int (*set_rechg_vol) (struct oppo_chg_chip *chip, int vol);
	int (*reset_charger) (struct oppo_chg_chip *chip);
	int (*read_full) (struct oppo_chg_chip *chip);
	int (*otg_enable) (void);
	int (*otg_disable) (void);
	int (*set_charging_term_disable) (struct oppo_chg_chip *chip);
	bool (*check_charger_resume) (struct oppo_chg_chip *chip);
	
	int (*get_charger_type) (void);
	int (*get_charger_volt) (void);
	
	bool (*check_chrdet_status) (void);
	int (*get_boot_mode)(void);
	int (*get_boot_reason)(void);
#ifdef CONFIG_OPPO_CHARGER_MTK	
	int (*get_instant_vbatt)(kal_bool);
#else
	int (*get_instant_vbatt)(void);
#endif

	int (*get_rtc_soc)(void);
	int (*set_rtc_soc)(int val);
	void (*set_power_off) (void);
	
	void (*usb_connect) (void);
	void (*usb_disconnect) (void);
#ifndef CONFIG_OPPO_CHARGER_MTK
	int (*get_aicl_ma) (struct oppo_chg_chip *chip);
	void(*rerun_aicl)(struct oppo_chg_chip *chip);
	int (*tlim_en)(struct oppo_chg_chip *chip,bool);
	int (*set_system_temp_level)(struct oppo_chg_chip *chip, int);
	int(*otg_pulse_skip_disable)(struct oppo_chg_chip *chip,enum skip_reason, bool);
	int(*set_dp_dm)(struct oppo_chg_chip *chip,int);
	int(*calc_flash_current)(struct oppo_chg_chip *chip);
#endif
};


/**
 * oppo_chg_init - initialize oppo_chg_chip
 * @chip: pointer to the oppo_chg_cip
 * @clinet: i2c client of the chip
 *
 * Returns: 0 - success; -1/errno - failed
 */
 int oppo_chg_parse_dt(struct oppo_chg_chip *chip);
 
int oppo_chg_init(struct oppo_chg_chip *chip);
void oppo_charger_detect_check(struct oppo_chg_chip *chip);
int oppo_chg_get_prop_batt_health(struct oppo_chg_chip *chip);

bool oppo_chg_wake_update_work(void);
void oppo_chg_soc_update_when_resume(unsigned long sleep_tm_sec);
int oppo_chg_get_batt_volt(void);

int oppo_chg_get_ui_soc(void);
int oppo_chg_get_soc(void);
int oppo_chg_get_chg_temperature(void);

void oppo_chg_kick_wdt(void);
void oppo_chg_disable_charge(void);
void oppo_chg_unsuspend_charger(void);

int oppo_chg_get_chg_type(void);

int oppo_chg_get_notify_flag(void);
int oppo_chg_show_vooc_logo_ornot(void);

bool get_otg_switch(void);
bool get_otg_online(void);
void set_otg_online(bool online);

bool oppo_chg_get_batt_full(void);
bool oppo_chg_get_rechging_status(void);

bool oppo_chg_check_chip_is_null(void);
void oppo_chg_set_charger_type_unknown(void);
int oppo_chg_get_charger_voltage(void);

#ifndef CONFIG_OPPO_CHARGER_MTK
void oppo_chg_variables_reset(struct oppo_chg_chip *chip, bool in);
void oppo_chg_external_power_changed(struct power_supply *psy);
#endif

#endif /*_OPPO_CHARGER_H_*/
