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
#include <device/vfs_pwm.h>
#include <vfs_err.h>
#include <vfs_register.h>
#include <hal/soc/pwm.h>
#include <aos/kernel.h>

#include <bl_pwm.h>
#include "hal_pwm.h"
#include <libfdt.h>

#include <utils_log.h>

typedef struct pwm_priv_data {
    aos_mutex_t    mutex;
} pwm_priv_data_t;

static int8_t inited;
static pwm_dev_t *dev_pwm0 = NULL;
static pwm_dev_t *dev_pwm1 = NULL;
static pwm_dev_t *dev_pwm2 = NULL;
static pwm_dev_t *dev_pwm3 = NULL;
static pwm_dev_t *dev_pwm4 = NULL;

static int pwm_dev_malloc(pwm_dev_t **pdev)
{
    if (*pdev) {
        log_error("arg err.\r\n");
        return -1;
    }

    *pdev = pvPortMalloc(sizeof(pwm_dev_t));
    if (*pdev == 0) {
        log_error("mem err.\r\n");
        return -1;
    }

    (*pdev)->priv = NULL;
    (*pdev)->priv = pvPortMalloc(sizeof(pwm_priv_data_t));
    if ((*pdev)->priv == NULL) {
        log_error("mem err.\r\n");
        return -1;
    }

    return 0;
}

static void pwm_dev_setdef(pwm_dev_t **pdev, uint8_t id)
{
    if (*pdev == NULL) {
        log_error("mem err.\r\n");
        return;
    }

    (*pdev)->port = id;
}

static int dev_pwm_init(uint8_t id, const char *path)
{
    pwm_dev_t **pdev = NULL;
    int ret;

    if ((id >= 3) || (path == 0)) {
        log_error("arg err.\r\n");
        return -1;
    }

    switch (id) {
        case 0:
        {
            pdev = &dev_pwm0;
        } break;
        case 1:
        {
            pdev = &dev_pwm1;
        } break;
        case 2:
        {
            pdev = &dev_pwm2;
        } break;
        case 3:
        {
            pdev = &dev_pwm3;
        } break;
        case 4:
        {
            pdev = &dev_pwm4;
        } break;
        default:
        {
            log_error("err.\r\n");
            return -1;
        } break;
    }

    if (pwm_dev_malloc(pdev) != 0) {
        return -1;
    }

    pwm_dev_setdef(pdev, id);
    ret = aos_register_driver(path, &pwm_ops, *pdev);
    if (ret != VFS_SUCCESS) {
        return ret;
    }

    return 0;
}

int32_t hal_pwm_init(pwm_dev_t *pwm)
{
    pwm_priv_data_t *data;

    data = pwm->priv;
    if (aos_mutex_new(&(data->mutex))) {
        /*we should assert here?*/
        return -1;
    }
    return 0;
}

int32_t hal_pwm_finalize(pwm_dev_t *pwm)
{
    pwm_priv_data_t *data;

    data = pwm->priv;
    aos_mutex_free(&(data->mutex));

    return 0;
}

#define BL_FDT32_TO_U8(addr, byte_offset)   ((uint8_t)fdt32_to_cpu(*(uint32_t *)((uint8_t *)addr + byte_offset)))
#define BL_FDT32_TO_U16(addr, byte_offset)  ((uint16_t)fdt32_to_cpu(*(uint32_t *)((uint8_t *)addr + byte_offset)))
#define BL_FDT32_TO_U32(addr, byte_offset)  ((uint32_t)fdt32_to_cpu(*(uint32_t *)((uint8_t *)addr + byte_offset)))
static void fdt_pwm_module_init(const void *fdt, int pwm_offset)
{
    #define PWM_MODULE_MAX 5
    //const char *path = "/dev/pwm0";
    int offset1 = 0;

    const uint32_t *addr_prop = 0;
    int lentmp = 0;
    const char *result = 0;
    int countindex = 0;
    int i;

    uint8_t id;
    char *path = NULL;
    uint8_t pin;
    uint32_t freq;

    const char *pwm_node[PWM_MODULE_MAX] = {
        "pwm@4000A420",
        "pwm@4000A440",
        "pwm@4000A460",
        "pwm@4000A480",
        "pwm@4000A4A0",
    };

    for (i = 0; i < PWM_MODULE_MAX; i++) {
        offset1 = fdt_subnode_offset(fdt, pwm_offset, pwm_node[i]);
        if (0 >= offset1) {
            log_info("pwm[%d] %s NULL.\r\n", i, pwm_node[i]);
            continue;
        }
        countindex = fdt_stringlist_count(fdt, offset1, "status");
        if (countindex != 1) {
            log_info("pwm[%d] status_countindex = %d NULL.\r\n", i, countindex);
            continue;
        }
        result = fdt_stringlist_get(fdt, offset1, "status", 0, &lentmp);
        if ((lentmp != 4) || (memcmp("okay", result, 4) != 0)) {
            log_info("pwm[%d] status = %s\r\n", i, result);
            continue;
        }

        /* set path */
        countindex = fdt_stringlist_count(fdt, offset1, "path");
        if (countindex != 1) {
            log_info("pwm[%d] path_countindex = %d NULL.\r\n", i, countindex);
            continue;
        }
        result = fdt_stringlist_get(fdt, offset1, "path", 0, &lentmp);
        if ((lentmp < 0) || (lentmp > 32))
        {
            log_info("pwm[%d] path lentmp = %d\r\n", i, lentmp);
        }
        path = (char *)result;

        /* set id */
        addr_prop = fdt_getprop(fdt, offset1, "id", &lentmp);
        if (addr_prop == NULL) {
            log_info("pwm[%d] id NULL.\r\n", i);
            continue;
        }
        id = BL_FDT32_TO_U8(addr_prop, 0);

        /* set pin */
        addr_prop = fdt_getprop(fdt, offset1, "pin", &lentmp);
        if (addr_prop == NULL) {
            log_info("pwm[%d] pin NULL.\r\n", i);
            continue;
        }
        pin = BL_FDT32_TO_U8(addr_prop, 0);

        /* set freq */
        addr_prop = fdt_getprop(fdt, offset1, "freq", &lentmp);
        if (addr_prop == NULL) {
            log_info("pwm[%d] freq NULL.\r\n", i);
            continue;
        }
        freq = BL_FDT32_TO_U32(addr_prop, 0);

        log_info("path = %s, id = %d, pin = %d, freq = %ld\r\n",
                path, id, pin, freq);
        bl_pwm_init(id, pin, freq);
        if (dev_pwm_init(id, (const char *)path) != 0) {
            log_error("dev_pwm_init err.\r\n");
        }
    }
}

int32_t vfs_pwm_init(uint32_t fdt, uint32_t dtb_pwm_offset)
{
    if (inited == 1) {
        return VFS_SUCCESS;
    }

    fdt_pwm_module_init((const void *)fdt, (int)dtb_pwm_offset);

    inited = 1;

    return VFS_SUCCESS;
}

int32_t hal_pwm_start(pwm_dev_t *pwm)
{
    bl_pwm_start(pwm->port);
    return 0;
}

int32_t hal_pwm_stop(pwm_dev_t *pwm)
{
    bl_pwm_stop(pwm->port);
    return 0;
}

int32_t hal_pwm_para_chg(pwm_dev_t *pwm, pwm_config_t para)
{
    return 0;
}

int32_t hal_pwm_set_duty(pwm_dev_t *pwm, float duty)
{
    //log_info("set duty %d, %ld\r\n", pwm->port, (uint16_t)duty);
    bl_pwm_set_duty(pwm->port, duty);
    return 0;
}

int32_t hal_pwm_set_freq(pwm_dev_t *pwm, uint32_t freq)
{
    log_error("not support.\r\n");
    bl_pwm_set_freq(pwm->port, freq);
    return 0;
}

