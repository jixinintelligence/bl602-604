# Component Makefile
#
## These include paths would be exported to project level
COMPONENT_ADD_INCLUDEDIRS += bl602_hal

## not be exported to project level
COMPONENT_PRIV_INCLUDEDIRS :=

## This component's src
COMPONENT_SRCS := bl602_hal/bl_uart.c \
                  bl602_hal/bl_chip.c \
                  bl602_hal/bl_cks.c \
                  bl602_hal/bl_sys.c \
                  bl602_hal/bl_dma.c \
                  bl602_hal/bl_irq.c \
                  bl602_hal/bl_sec.c \
                  bl602_hal/bl_boot2.c \
                  bl602_hal/bl_timer.c \
                  bl602_hal/bl_gpio.c \
                  bl602_hal/bl_gpio_cli.c \
                  bl602_hal/bl_efuse.c \
                  bl602_hal/bl_flash.c \
                  bl602_hal/bl_pwm.c \
                  bl602_hal/bl_sec_aes.c \
                  bl602_hal/bl_sec_sha.c \
                  bl602_hal/bl_wifi.c \
                  bl602_hal/bl_wdt.c \
                  bl602_hal/bl_wdt_cli.c \
                  bl602_hal/hal_uart.c \
                  bl602_hal/hal_gpio.c \
                  bl602_hal/hal_pwm.c \
                  bl602_hal/hal_boot2.c \
                  bl602_hal/hal_sys.c \
                  bl602_hal/hal_board.c \
                  bl602_hal/bl_adc.c \


COMPONENT_SRCDIRS := bl602_hal

COMPONENT_OBJS := $(patsubst %.c,%.o, $(COMPONENT_SRCS))
COMPONENT_OBJS := $(patsubst %.cpp,%.o, $(COMPONENT_OBJS))

##
CPPFLAGS += -DARCH_RISCV
ifndef CONFIG_USE_STD_DRIVER
CPPFLAGS += -DBL602_USE_HAL_DRIVER
endif
