#clear_vars
LOCAL_SRCS_FILE:=

LHAL_MODULE_DIR:= BSP/LHalDriver
LHAL_MODULE_OUT_DIR:= $(TARGET_OUT_PATH)/BSP/LHalDriver
LHAL_MODULE_SRC_DIR := $(LHAL_MODULE_DIR)/Src

LHALDRIVER_CFLAGS:= 
LHALDRIVER_INCLUDE:=  -I $(LHAL_MODULE_DIR)/Inc

lhaldriver_sources := lhal_uart.c \

lhaldriver_objs := $(addprefix $(LHAL_MODULE_OUT_DIR)/, $(subst .c,.o,$(lhaldriver_sources)))
base_objects += $(lhaldriver_objs)

$(LHAL_MODULE_OUT_DIR)/%.o:$(LHAL_MODULE_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo "cc $<"
	$(AT)$(CC) -c $(GLOBAL_CFLAGS) $(LHALDRIVER_CFLAGS) $(GLOBAL_INCLUDE) $(COMMON_INCLUDE) $(LHALDRIVER_INCLUDE) $< -o $@

.PHONY: lhaldriver
lhaldriver: $(lhaldriver_objs)
	@echo  "lhaldriver_objs is $(lhaldriver_objs)"
