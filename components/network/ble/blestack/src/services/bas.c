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
 *  @brief GATT Battery Service
 */

/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <zephyr/types.h>

#include "bluetooth.h"
#include "conn.h"
#include "gatt.h"
#include "uuid.h"
#include "bas.h"

#if !defined(BFLB_BLE)
#define LOG_LEVEL CONFIG_BT_GATT_BAS_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(bas);
#endif

static struct bt_gatt_ccc_cfg  blvl_ccc_cfg[BT_GATT_CCC_MAX] = {};

static u8_t battery_level = 100U;

static void blvl_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				       u16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

#if !defined(BFLB_BLE)
	LOG_INF("BAS Notifications %s", notif_enabled ? "enabled" : "disabled");
#endif
}

static ssize_t read_blvl(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       u16_t len, u16_t offset)
{
	u8_t lvl8 = battery_level;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &lvl8,
				 sizeof(lvl8));
}

static struct bt_gatt_attr attrs[]= {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
	BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_blvl, NULL,
			       &battery_level),
	BT_GATT_CCC(blvl_ccc_cfg, blvl_ccc_cfg_changed),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ,
             NULL, NULL, NULL),
};

struct bt_gatt_service bas = BT_GATT_SERVICE(attrs);

void bas_init(void)
{	
    bt_gatt_service_register(&bas);
}

u8_t bt_gatt_bas_get_battery_level(void)
{
	return battery_level;
}

int bt_gatt_bas_set_battery_level(u8_t level)
{
	int rc;

	if (level > 100U) {
		return -EINVAL;
	}

	battery_level = level;

	rc = bt_gatt_notify(NULL, &bas.attrs[1], &level, sizeof(level));

	return rc == -ENOTCONN ? 0 : rc;
}

#if !defined(BFLB_BLE)
SYS_INIT(bas_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif
