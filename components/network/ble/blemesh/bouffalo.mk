CFLAGS += -DCONFIG_BT_MESH -DCONFIG_BT_MESH_PROV
#CFLAGS += -DCONFIG_BT_SETTINGS
ifeq ($(CONFIG_BT_MESH_PB_ADV),1)
CFLAGS += -DCONFIG_BT_MESH_PB_ADV
endif
ifeq ($(CONFIG_BT_MESH_PB_GATT),1)
CFLAGS += -DCONFIG_BT_MESH_PB_GATT
endif
ifeq ($(CONFIG_BT_MESH_FRIEND),1)
CFLAGS += -DCONFIG_BT_MESH_FRIEND
endif
ifeq ($(CONFIG_BT_MESH_LOW_POWER),1)
CFLAGS += -DCONFIG_BT_MESH_LOW_POWER
endif
ifeq ($(CONFIG_BT_MESH_PROXY),1)
CFLAGS += -DCONFIG_BT_MESH_PROXY
endif
ifeq ($(CONFIG_BT_MESH_GATT_PROXY),1)
CFLAGS += -DCONFIG_BT_MESH_GATT_PROXY
endif
ifeq ($(CONFIG_BLE_STACK_DBG_PRINT),1)
CFLAGS += -DCFG_BLE_STACK_DBG_PRINT
endif
ifeq ($(CONFIG_BT_MESH_MODEL_GEN_SRV),1)
CFLAGS += -DCONFIG_BT_MESH_MODEL_GEN_SRV
endif 

# Component Makefile
#
## These include paths would be exported to project level
COMPONENT_ADD_INCLUDEDIRS    += src \
                                src/include  \
                                src/mesh_cli_cmds  \
                                src/mesh_models/include  \
                                src/mesh_models/server/include

## not be exported to project level
COMPONENT_PRIV_INCLUDEDIRS   :=

## This component's src 
COMPONENT_SRCS   := src/access.c \
					src/adv.c \
					src/beacon.c \
					src/cfg_cli.c \
					src/cfg_srv.c \
					src/crypto.c \
					src/health_cli.c \
					src/health_srv.c \
					src/main.c \
					src/net.c \
					src/prov.c \
					src/proxy.c \
					src/settings.c \
					src/transport.c \
					src/mesh_cli_cmds/mesh_cli_cmds.c \

ifeq ($(CONFIG_BT_MESH_FRIEND),1)
COMPONENT_SRCS   += src/friend.c
endif

ifeq ($(CONFIG_BT_MESH_LOW_POWER),1)
COMPONENT_SRCS   += src/lpn.c
endif

ifeq ($(CONFIG_BT_MESH_MODEL_GEN_SRV),1)
COMPONENT_SRCS   += src/mesh_models/server/common_srv.c \
                    src/mesh_models/server/gen_srv.c 
endif

COMPONENT_OBJS   := $(patsubst %.c,%.o, $(COMPONENT_SRCS))
COMPONENT_SRCDIRS:= src \
                    src/mesh_cli_cmds \
                    src/mesh_models/server

include $(COMPONENT_PATH)/../ble_common.mk