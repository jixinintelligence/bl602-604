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
/**
 * @file rpa.c
 * Resolvable Private Address Generation and Resolution
 */

/*
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stddef.h>
#include <errno.h>
#include <string.h>
#include <atomic.h>
#include <misc/util.h>
#include <misc/byteorder.h>
#include <misc/stack.h>

#include <constants.h>
#include <aes.h>
#include <utils.h>
#include <cmac_mode.h>
#include <../include/bluetooth/crypto.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_RPA)
#define LOG_MODULE_NAME bt_rpa
#include "log.h"

static int ah(const u8_t irk[16], const u8_t r[3], u8_t out[3])
{
	u8_t res[16];
	int err;

	BT_DBG("irk %s", bt_hex(irk, 16));
	BT_DBG("r %s", bt_hex(r, 3));

	/* r' = padding || r */
	memcpy(res, r, 3);
	(void)memset(res + 3, 0, 13);

	err = bt_encrypt_le(irk, res, res);
	if (err) {
		return err;
	}

	/* The output of the random address function ah is:
	 *      ah(h, r) = e(k, r') mod 2^24
	 * The output of the security function e is then truncated to 24 bits
	 * by taking the least significant 24 bits of the output of e as the
	 * result of ah.
	 */
	memcpy(out, res, 3);

	return 0;
}

#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CTLR_PRIVACY)
bool bt_rpa_irk_matches(const u8_t irk[16], const bt_addr_t *addr)
{
	u8_t hash[3];
	int err;

	BT_DBG("IRK %s bdaddr %s", bt_hex(irk, 16), bt_addr_str(addr));

	err = ah(irk, addr->val + 3, hash);
	if (err) {
		return false;
	}

	return !memcmp(addr->val, hash, 3);
}
#endif

#if defined(CONFIG_BT_PRIVACY) || defined(CONFIG_BT_CTLR_PRIVACY)
int bt_rpa_create(const u8_t irk[16], bt_addr_t *rpa)
{
	int err;

	err = bt_rand(rpa->val + 3, 3);
	if (err) {
		return err;
	}

	BT_ADDR_SET_RPA(rpa);

	err = ah(irk, rpa->val + 3, rpa->val);
	if (err) {
		return err;
	}

	BT_DBG("Created RPA %s", bt_addr_str((bt_addr_t *)rpa->val));

	return 0;
}
#else
int bt_rpa_create(const u8_t irk[16], bt_addr_t *rpa)
{
	return -ENOTSUP;
}
#endif /* CONFIG_BT_PRIVACY */

