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
 *  @brief Internal API for Generic Attribute Profile handling.
 */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define BT_GATT_CENTRAL_ADDR_RES_NOT_SUPP	0
#define BT_GATT_CENTRAL_ADDR_RES_SUPP		1

#include <gatt.h>

#define BT_GATT_PERM_READ_MASK			(BT_GATT_PERM_READ | \
						BT_GATT_PERM_READ_ENCRYPT | \
						BT_GATT_PERM_READ_AUTHEN)
#define BT_GATT_PERM_WRITE_MASK			(BT_GATT_PERM_WRITE | \
						BT_GATT_PERM_WRITE_ENCRYPT | \
						BT_GATT_PERM_WRITE_AUTHEN)
#define BT_GATT_PERM_ENCRYPT_MASK		(BT_GATT_PERM_READ_ENCRYPT | \
						BT_GATT_PERM_WRITE_ENCRYPT)
#define BT_GATT_PERM_AUTHEN_MASK		(BT_GATT_PERM_READ_AUTHEN | \
						BT_GATT_PERM_WRITE_AUTHEN)

void bt_gatt_init(void);
void bt_gatt_connected(struct bt_conn *conn);
void bt_gatt_encrypt_change(struct bt_conn *conn);
void bt_gatt_disconnected(struct bt_conn *conn);

bool bt_gatt_change_aware(struct bt_conn *conn, bool req);

int bt_gatt_store_ccc(u8_t id, const bt_addr_le_t *addr);

int bt_gatt_clear(u8_t id, const bt_addr_le_t *addr);

#if defined(CONFIG_BT_GATT_CLIENT)
void bt_gatt_notification(struct bt_conn *conn, u16_t handle,
			  const void *data, u16_t length);
#else
static inline void bt_gatt_notification(struct bt_conn *conn, u16_t handle,
					const void *data, u16_t length)
{
}
#endif /* CONFIG_BT_GATT_CLIENT */

struct bt_gatt_attr;

/* Check attribute permission */
u8_t bt_gatt_check_perm(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t mask);
