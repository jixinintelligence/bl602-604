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
#include "phy_hal.h"
#include "bl602_glb.h"
#include "bl602_gpio.h"
#include "bl602_adc.h"

struct phy_hal_tag
{
    int16_t temperature;
    uint8_t capcode;
};

static uint32_t fem_gpio_oe;
static uint32_t fem_gpio_cfgctl10; // use GPIO20,GPIO21
static uint32_t fem_gpio_inited = 0;
static struct phy_hal_tag hal_env;


void hal_fem_gpio_on()
{
    GLB_GPIO_Cfg_Type cfg;

    cfg.gpioPin = 20;
    cfg.gpioFun = GPIO20_FUN_FEM_GPIO_0;
    cfg.gpioMode = GPIO_MODE_OUTPUT;
    cfg.pullType = GPIO_PULL_NONE;
    GLB_GPIO_Init(&cfg);

    cfg.gpioPin = 21;
    cfg.gpioFun = GPIO21_FUN_FEM_GPIO_1;
    cfg.gpioMode = GPIO_MODE_OUTPUT;
    cfg.pullType = GPIO_PULL_NONE;
    GLB_GPIO_Init(&cfg);
}

void hal_fem_gpio_off()
{
    if (fem_gpio_inited) {
        BL_WR_WORD(GLB_BASE+GLB_GPIO_OUTPUT_EN_OFFSET, fem_gpio_oe);
        BL_WR_WORD(GLB_BASE+GLB_GPIO_CFGCTL10_OFFSET, fem_gpio_cfgctl10);
    }
}

uint8_t hal_get_capcode()
{
    return ((*(volatile uint32_t*)0x4000f884) >> 22) & 0x3f;
}

void hal_set_capcode(uint32_t capcode)
{
    uint32_t rval = (*(volatile uint32_t*)0x4000f884);    
    *(volatile uint32_t*)0x4000f884 = (rval & ~(0x3f << 16) & ~(0x3f << 22)) | (capcode << 16) | (capcode << 22);
    // printf("-> new capcode %ld (%lx)", capcode, (*(volatile uint32_t*)0x4000f884));
}

bool hal_get_temperature(int16_t *temperature)
{
    // if adc is in use
    //     return false;
    *temperature = hal_env.temperature;
    return true;
}

void hal_set_temperature(int16_t temperature)
{
    hal_env.temperature = temperature;
}
