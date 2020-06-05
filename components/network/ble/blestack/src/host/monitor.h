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
 *  @brief Custom monitor protocol logging over UART
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_BT_DEBUG_MONITOR)

#define BT_MONITOR_NEW_INDEX	0
#define BT_MONITOR_DEL_INDEX	1
#define BT_MONITOR_COMMAND_PKT	2
#define BT_MONITOR_EVENT_PKT	3
#define BT_MONITOR_ACL_TX_PKT	4
#define BT_MONITOR_ACL_RX_PKT	5
#define BT_MONITOR_SCO_TX_PKT	6
#define BT_MONITOR_SCO_RX_PKT	7
#define BT_MONITOR_OPEN_INDEX	8
#define BT_MONITOR_CLOSE_INDEX	9
#define BT_MONITOR_INDEX_INFO	10
#define BT_MONITOR_VENDOR_DIAG	11
#define BT_MONITOR_SYSTEM_NOTE	12
#define BT_MONITOR_USER_LOGGING	13
#define BT_MONITOR_NOP		255

#define BT_MONITOR_TYPE_PRIMARY	0
#define BT_MONITOR_TYPE_AMP	1

/* Extended header types */
#define BT_MONITOR_COMMAND_DROPS 1
#define BT_MONITOR_EVENT_DROPS   2
#define BT_MONITOR_ACL_RX_DROPS  3
#define BT_MONITOR_ACL_TX_DROPS  4
#define BT_MONITOR_SCO_RX_DROPS  5
#define BT_MONITOR_SCO_TX_DROPS  6
#define BT_MONITOR_OTHER_DROPS   7
#define BT_MONITOR_TS32          8

#define BT_MONITOR_BASE_HDR_LEN  6

#if defined(CONFIG_BT_BREDR)
#define BT_MONITOR_EXT_HDR_MAX 19
#else
#define BT_MONITOR_EXT_HDR_MAX 15
#endif

struct bt_monitor_hdr {
	u16_t  data_len;
	u16_t  opcode;
	u8_t   flags;
	u8_t   hdr_len;

	u8_t   ext[BT_MONITOR_EXT_HDR_MAX];
} __packed;

struct bt_monitor_ts32 {
	u8_t   type;
	u32_t  ts32;
} __packed;

struct bt_monitor_new_index {
	u8_t  type;
	u8_t  bus;
	u8_t  bdaddr[6];
	char  name[8];
} __packed;

struct bt_monitor_user_logging {
	u8_t  priority;
	u8_t  ident_len;
} __packed;

static inline u8_t bt_monitor_opcode(struct net_buf *buf)
{
	switch (bt_buf_get_type(buf)) {
	case BT_BUF_CMD:
		return BT_MONITOR_COMMAND_PKT;
	case BT_BUF_EVT:
		return BT_MONITOR_EVENT_PKT;
	case BT_BUF_ACL_OUT:
		return BT_MONITOR_ACL_TX_PKT;
	case BT_BUF_ACL_IN:
		return BT_MONITOR_ACL_RX_PKT;
	default:
		return BT_MONITOR_NOP;
	}
}

void bt_monitor_send(u16_t opcode, const void *data, size_t len);

void bt_monitor_new_index(u8_t type, u8_t bus, bt_addr_t *addr,
			  const char *name);

#else /* !CONFIG_BT_DEBUG_MONITOR */

#define bt_monitor_send(opcode, data, len)
#define bt_monitor_new_index(type, bus, addr, name)

#endif
