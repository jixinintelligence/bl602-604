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
 *  @brief Attribute Protocol handling.
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_BLUETOOTH_ATT_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_ATT_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <misc/slist.h>

/* Error codes for Error response PDU */
#define BT_ATT_ERR_INVALID_HANDLE		0x01
#define BT_ATT_ERR_READ_NOT_PERMITTED		0x02
#define BT_ATT_ERR_WRITE_NOT_PERMITTED		0x03
#define BT_ATT_ERR_INVALID_PDU			0x04
#define BT_ATT_ERR_AUTHENTICATION		0x05
#define BT_ATT_ERR_NOT_SUPPORTED		0x06
#define BT_ATT_ERR_INVALID_OFFSET		0x07
#define BT_ATT_ERR_AUTHORIZATION		0x08
#define BT_ATT_ERR_PREPARE_QUEUE_FULL		0x09
#define BT_ATT_ERR_ATTRIBUTE_NOT_FOUND		0x0a
#define BT_ATT_ERR_ATTRIBUTE_NOT_LONG		0x0b
#define BT_ATT_ERR_ENCRYPTION_KEY_SIZE		0x0c
#define BT_ATT_ERR_INVALID_ATTRIBUTE_LEN	0x0d
#define BT_ATT_ERR_UNLIKELY			0x0e
#define BT_ATT_ERR_INSUFFICIENT_ENCRYPTION	0x0f
#define BT_ATT_ERR_UNSUPPORTED_GROUP_TYPE	0x10
#define BT_ATT_ERR_INSUFFICIENT_RESOURCES	0x11
#define BT_ATT_ERR_DB_OUT_OF_SYNC		0x12
#define BT_ATT_ERR_VALUE_NOT_ALLOWED		0x13

/* Common Profile Error Codes (from CSS) */
#define BT_ATT_ERR_WRITE_REQ_REJECTED		0xfc
#define BT_ATT_ERR_CCC_IMPROPER_CONF		0xfd
#define BT_ATT_ERR_PROCEDURE_IN_PROGRESS	0xfe
#define BT_ATT_ERR_OUT_OF_RANGE			0xff

typedef void (*bt_att_func_t)(struct bt_conn *conn, u8_t err,
			      const void *pdu, u16_t length,
			      void *user_data);
typedef void (*bt_att_destroy_t)(void *user_data);

/* ATT request context */
struct bt_att_req {
	sys_snode_t node;
	bt_att_func_t func;
	bt_att_destroy_t destroy;
	struct net_buf_simple_state state;
	struct net_buf *buf;
#if defined(CONFIG_BT_SMP)
	bool retrying;
#endif /* CONFIG_BT_SMP */
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_ATT_H_ */
