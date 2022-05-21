ifeq ($(CONFIG_OPPO_CHARGER_MTK),y)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
ifeq ($(CONFIG_MTK_HAFG_20),y)
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_battery_mtk_fg_20.o
else
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_battery_mtk.o
endif
else
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_battery_qcom.o
endif 

obj-$(CONFIG_OPPO_CHARGER)	+= oppo_bq2202a.o
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_bq24196.o

