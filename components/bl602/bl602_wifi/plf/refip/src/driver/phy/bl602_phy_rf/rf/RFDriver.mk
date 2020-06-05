#clear_vars
LOCAL_SRCS_FILE:=

MODULE_DIR:= BSP/RFDriver
SUBMODULE_OUT_DIR:= $(TARGET_OUT_PATH)/BSP/RFDriver
SUBMODULE_SRC_DIR := $(MODULE_DIR)/Src

RFDRIVER_CFLAGS:= 
RFDRIVER_INCLUDE:=  -I $(MODULE_DIR)/Inc \

rfdriver_sources := bl602_rf_private.c bl602_calib_data.c

rfdriver_objs := $(addprefix $(SUBMODULE_OUT_DIR)/, $(subst .c,.o,$(rfdriver_sources)))
base_objects += $(rfdriver_objs)

$(SUBMODULE_OUT_DIR)/%.o:$(SUBMODULE_SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo "cc $<"
	$(AT)$(CC) -c $(GLOBAL_CFLAGS) $(RFDRIVER_CFLAGS) $(GLOBAL_INCLUDE) $(COMMON_INCLUDE) $(RFDRIVER_INCLUDE) $< -o $@

.PHONY: rfdriver
rfdriver: $(rfdriver_objs)
	@echo  "rfdriver_objs is $(rfdriver_objs)"
