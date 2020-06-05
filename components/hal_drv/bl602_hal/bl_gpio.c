/*
 * Copyright (c) 2020 Bouffalolab.
 *
 * This file is part of
 *     *** Bouffalolab Software Dev Kit ***
 *      (see www.bouffalolab.com).
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Bouffalo Lab nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdint.h>

#include <bl602_glb.h>
#include <bl602_gpio.h>
#include "bl_gpio.h"

#define GPIO_FUNC_NUM_IN_BL602 (GPIO0_FUN_SWGPIO_0)
int bl_gpio_enable_output(uint8_t pin, uint8_t pullup, uint8_t pulldown)
{
    GLB_GPIO_Cfg_Type cfg;

    cfg.drive=0;
    cfg.smtCtrl=1;
    cfg.gpioPin = pin;
    cfg.gpioFun = GPIO_FUNC_NUM_IN_BL602;//all the function number of GPIO is the same, we use def from GPIO0 here
    cfg.gpioMode = GPIO_MODE_OUTPUT;
    cfg.pullType = GPIO_PULL_NONE;
    if (pullup) {
        cfg.pullType = GPIO_PULL_UP;
    }
    if (pulldown) {
        cfg.pullType = GPIO_PULL_DOWN;
    }
    GLB_GPIO_Init(&cfg);

    return 0;
}

int bl_gpio_enable_input(uint8_t pin, uint8_t pullup, uint8_t pulldown)
{
    GLB_GPIO_Cfg_Type cfg;

    cfg.drive=0;
    cfg.smtCtrl=1;
    cfg.gpioPin = pin;
    cfg.gpioFun = GPIO_FUNC_NUM_IN_BL602;//all the function number of GPIO is the same, we use def from GPIO0 here
    cfg.gpioMode = GPIO_MODE_INPUT;
    cfg.pullType = GPIO_PULL_NONE;
    if (pullup) {
        cfg.pullType = GPIO_PULL_UP;
    }
    if (pulldown) {
        cfg.pullType = GPIO_PULL_DOWN;
    }
    GLB_GPIO_Init(&cfg);

    return 0;
}

int bl_gpio_output_set(uint8_t pin, uint8_t value)
{
    GLB_GPIO_Write((GLB_GPIO_Type)pin, value ? 1 : 0);
    return 0;
}

int bl_gpio_input_get(uint8_t pin, uint8_t *value)
{
    *value = GLB_GPIO_Read((GLB_GPIO_Type)pin);
    return 0;
}

int bl_gpio_input_get_value(uint8_t pin)
{
    return GLB_GPIO_Read((GLB_GPIO_Type)pin);
}
