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
/* log.c - logging helpers */

/*
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Helper for printk parameters to convert from binary to hex.
 * We declare multiple buffers so the helper can be used multiple times
 * in a single printk call.
 */

#include <stddef.h>
#include <zephyr/types.h>
#include <zephyr.h>
#include <misc/util.h>
#include <bluetooth.h>
#include <hci_host.h>

const char *bt_hex_real(const void *buf, size_t len)
{
	static const char hex[] = "0123456789abcdef";
	static char str[129];
	const u8_t *b = buf;
	int i;

	len = MIN(len, (sizeof(str) - 1) / 2);

	for (i = 0; i < len; i++) {
		str[i * 2]     = hex[b[i] >> 4];
		str[i * 2 + 1] = hex[b[i] & 0xf];
	}

	str[i * 2] = '\0';

	return str;
}

const char *bt_addr_str_real(const bt_addr_t *addr)
{
	static char str[BT_ADDR_STR_LEN];

	bt_addr_to_str(addr, str, sizeof(str));

	return str;
}

const char *bt_addr_le_str_real(const bt_addr_le_t *addr)
{
	static char str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, str, sizeof(str));

	return str;
}


