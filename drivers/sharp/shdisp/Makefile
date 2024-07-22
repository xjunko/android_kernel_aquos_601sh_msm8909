obj-$(CONFIG_SHDISP) += shdisp_kerl.o
obj-$(CONFIG_SHDISP) += shdisp_system.o
obj-$(CONFIG_SHDISP) += shdisp_pm.o
obj-$(CONFIG_SHDISP) += shdisp_panel.o
ifeq ($(CONFIG_SHDISP_BDIC_BD6118GU),y)
obj-$(CONFIG_SHDISP) += shdisp_bd6118gu.o
obj-$(CONFIG_SHDISP) += shdisp_bd6118gu_main.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_COLUMBUS),y)
obj-$(CONFIG_SHDISP) += shdisp_columbus.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_SUBDISPLAY),y)
obj-$(CONFIG_SHDISP) += shdisp_subdisplay.o
obj-$(CONFIG_SHDISP) += shdisp_exclusion.o
endif
obj-$(CONFIG_SHDISP) += shdisp_dbg.o
#obj-$(CONFIG_SHDISP) += shlcdc_eventlog.o
EXTRA_CFLAGS    += -DSH_BUILD_ID=\"$(SH_BUILD_ID)\"
EXTRA_CFLAGS    += -DUSE_LINUX
ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHDISP_FACTORY_MODE_ENABLE
endif
