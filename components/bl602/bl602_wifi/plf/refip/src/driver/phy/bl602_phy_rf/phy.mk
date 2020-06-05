phy_cpp_defines := -DCFG_PHY_TRPC="1" 
phy_cpp_defines += -DCFG_PHY_ADAPT="1"
phy_cpp_defines += -DCFG_RF_ICAL="1"
phy_cpp_defines += -DCFG_RF_TCAL="0"
phy_cpp_defines += -DCFG_RF_TCAL_SIM="0"
phy_cpp_defines += -DCFG_PHY_TRPC_DEBUG="0"
phy_cpp_defines += -DCFG_PHY_ADAPT_RSSI_PRINT="1"
phy_cpp_defines += -DCFG_PHY_ADAPT_CE_PRINT="0"


phy_export_dirs := plf/refip/src/driver/phy/bl602_phy_rf
phy_top_dirs := plf/refip/src/driver/phy
phyif_top_dirs := plf/refip/src/driver/phyif

phy_inc_dirs := $(phy_top_dirs) \
				$(phy_top_dirs)/bl602_phy_rf \
				$(phy_top_dirs)/bl602_phy_rf/rf/Inc \
				$(phyif_top_dirs) 

rf_inc_dirs  := $(phy_top_dirs)/bl602_phy_rf/rf/Inc \

phy_src_dirs := $(phy_top_dirs) \
                $(phy_top_dirs)/bl602_phy_rf \
                $(phy_top_dirs)/bl602_phy_rf/rf/Src \
				$(phyif_top_dirs) 

ifeq ($(CONFIG_CASE_RFTEST),1)

plf_phy	:= $(phy_top_dirs)/rf.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_bl602.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_adapt.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_hal.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_trpc.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_tcal.c \
		   $(phy_top_dirs)/bl602_phy_rf/agcmem_bl602.c \
		   $(phy_top_dirs)/bl602_phy_rf/rfc_bl602.c \
		   $(phy_top_dirs)/bl602_phy_rf/rf/Src/bl602_rf_private.c \
		   $(phy_top_dirs)/bl602_phy_rf/rf/Src/bl602_calib_data.c

ifeq ($(CONFIG_CASE_MFG_HTOL),1)
plf_phy += $(plf_top_dirs)/driver/phy/bl602_phy_rf/phy_bl602_cli_api.c
else
plf_phy += $(plf_top_dirs)/driver/phy/bl602_phy_rf/phy_bl602_cli.c
endif

else

plf_phy	:= $(phy_top_dirs)/rf.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_bl602.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_adapt.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_hal.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_trpc.c \
		   $(phy_top_dirs)/bl602_phy_rf/phy_tcal.c \
		   $(phy_top_dirs)/bl602_phy_rf/agcmem_bl602.c \
		   $(phy_top_dirs)/bl602_phy_rf/rfc_bl602.c \
		   $(phy_top_dirs)/bl602_phy_rf/rf/Src/bl602_rf_private.c \
		   $(phy_top_dirs)/bl602_phy_rf/rf/Src/bl602_calib_data.c \
		   $(phyif_top_dirs)/phyif_utils.c \

endif
