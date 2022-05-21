ifeq ($(CONFIG_OPPO_CHARGER_MTK),y)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
subdir-ccflags-y += -Werror
subdir-ccflags-y += -Wno-unused-variable
endif

obj-$(CONFIG_OPPO_CHARGER)      += charger_ic/
obj-$(CONFIG_OPPO_CHARGER)      += gauge_ic/
obj-$(CONFIG_OPPO_CHARGER)      += vooc_ic/

obj-$(CONFIG_OPPO_CHARGER)	+= oppo_charger.o
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_gauge.o
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_vooc.o
