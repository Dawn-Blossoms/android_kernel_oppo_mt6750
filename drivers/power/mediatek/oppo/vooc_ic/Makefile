ifeq ($(CONFIG_OPPO_CHARGER_MTK),y)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
obj-$(CONFIG_OPPO_CHARGER)      += oppo_vooc_fw_mtk.o
else
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_vooc_fw_qcom.o
endif

obj-$(CONFIG_OPPO_CHARGER)	+= oppo_pic16f.o
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_stm8s.o
obj-$(CONFIG_OPPO_CHARGER)	+= oppo_vooc_adapter.o
