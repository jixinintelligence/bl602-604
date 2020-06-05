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
/** @file
 *  @brief Bluetooth subsystem logging helpers.
 */

/*
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __BT_LOG_H
#define __BT_LOG_H

#include <zephyr.h>

#include <bluetooth.h>
#include <hci_host.h>

#if defined(BL70X)
#include "bl_print.h"
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(BT_DBG_ENABLED)
#define BT_DBG_ENABLED 1
#endif

#if BT_DBG_ENABLED
#define LOG_LEVEL LOG_LEVEL_DBG
#else
#define LOG_LEVEL CONFIG_BT_LOG_LEVEL
#endif

//LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL);

#if defined(BFLB_BLE)
//#if defined(CFG_BLE_STACK_DBG_PRINT)
#define BT_DBG(fmt, ...)
#if defined(BL70X)
#define BT_ERR(fmt, ...)   bl_print(SYSTEM_UART_ID, PRINT_MODULE_BLE_STACK, fmt"\r\n", ##__VA_ARGS__)
#elif defined(BL602) || defined(BL702)
#define BT_ERR(fmt, ...)   printf(fmt"\r\n", ##__VA_ARGS__)
#if defined(CONFIG_BT_STACK_PTS)
#define BT_STACK_PTS_DBG(fmt, ...)  printf(fmt"\r\n", ##__VA_ARGS__)
#endif
//#endif
#define BT_WARN(fmt, ...) 
#define BT_INFO(fmt, ...)
#else
#define BT_DBG(fmt, ...)
#define BT_ERR(fmt, ...)
#define BT_WARN(fmt, ...)
#define BT_INFO(fmt, ...)


#endif /*CFG_BLE_STACK_DBG_PRINT*/

#if defined(CONFIG_BT_STACK_PTS)
#define BT_STACK_PTS_SDBG(str, len, reversal) \
{	\
	BT_STACK_PTS_DBG("uuid = [");	\
	u8_t i = 0; \
	for(i=0;i<len;i++){	\
		if(reversal)	\
			BT_STACK_PTS_DBG("%02x", str[len-1-i]); \
		else	\
			BT_STACK_PTS_DBG("%02x", str[i]);	\
	}	\
	BT_STACK_PTS_DBG("]\r\n");\
}

#endif 

#else
#define BT_DBG(fmt, ...) LOG_DBG(fmt, ##__VA_ARGS__)
#define BT_ERR(fmt, ...) LOG_ERR(fmt, ##__VA_ARGS__)
#define BT_WARN(fmt, ...) LOG_WRN(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...) LOG_INF(fmt, ##__VA_ARGS__)



#if defined(CONFIG_BT_ASSERT_VERBOSE)
#define BT_ASSERT_PRINT(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define BT_ASSERT_PRINT(fmt, ...)
#endif /* CONFIG_BT_ASSERT_VERBOSE */

#if defined(CONFIG_BT_ASSERT_PANIC)
#define BT_ASSERT_DIE k_panic
#else
#define BT_ASSERT_DIE k_oops
#endif /* CONFIG_BT_ASSERT_PANIC */
#endif /*BFLB_BLE*/

#if defined(CONFIG_BT_ASSERT)
#if defined(BFLB_BLE)
extern void user_vAssertCalled(void);
#define BT_ASSERT(cond) if( ( cond ) == 0 ) user_vAssertCalled()
#else
#define BT_ASSERT(cond) if (!(cond)) { \
				BT_ASSERT_PRINT("assert: '" #cond \
						"' failed\n"); \
				BT_ASSERT_DIE(); \
			}
#endif/*BFLB_BLE*/
#else
#if defined(BFLB_BLE)
#define BT_ASSERT(cond)
#else
#define BT_ASSERT(cond) __ASSERT_NO_MSG(cond)
#endif /*BFLB_BLE*/
#endif/* CONFIG_BT_ASSERT*/

#define BT_HEXDUMP_DBG(_data, _length, _str) \
		LOG_HEXDUMP_DBG((const u8_t *)_data, _length, _str)

#if defined(BFLB_BLE)
static inline char *log_strdup(const char *str)
{
	return (char *)str;
}
#endif

/* NOTE: These helper functions always encodes into the same buffer storage.
 * It is the responsibility of the user of this function to copy the information
 * in this string if needed.
 */
const char *bt_hex_real(const void *buf, size_t len);
const char *bt_addr_str_real(const bt_addr_t *addr);
const char *bt_addr_le_str_real(const bt_addr_le_t *addr);

/* NOTE: log_strdup does not guarantee a duplication of the string.
 * It is therefore still the responsibility of the user to handle the
 * restrictions in the underlying function call.
 */
#define bt_hex(buf, len) log_strdup(bt_hex_real(buf, len))
#define bt_addr_str(addr) log_strdup(bt_addr_str_real(addr))
#define bt_addr_le_str(addr) log_strdup(bt_addr_le_str_real(addr))

#ifdef __cplusplus
}
#endif

#endif /* __BT_LOG_H */
