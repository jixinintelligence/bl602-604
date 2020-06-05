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

#ifndef _HCI_CONTROLLER_H_
#define _HCI_CONTROLLER_H_

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
extern s32_t    hci_hbuf_total;
extern u32_t    hci_hbuf_sent;
extern u32_t    hci_hbuf_acked;
extern atomic_t hci_state_mask;

#define HCI_STATE_BIT_RESET 0
#endif

#define HCI_CLASS_EVT_REQUIRED    0
#define HCI_CLASS_EVT_DISCARDABLE 1
#define HCI_CLASS_EVT_CONNECTION  2
#define HCI_CLASS_ACL_DATA        3

#if defined(CONFIG_SOC_FAMILY_NRF5)
#define BT_HCI_VS_HW_PLAT BT_HCI_VS_HW_PLAT_NORDIC
#if defined(CONFIG_SOC_SERIES_NRF51X)
#define BT_HCI_VS_HW_VAR  BT_HCI_VS_HW_VAR_NORDIC_NRF51X;
#elif defined(CONFIG_SOC_SERIES_NRF52X)
#define BT_HCI_VS_HW_VAR  BT_HCI_VS_HW_VAR_NORDIC_NRF52X;
#endif
#else
#define BT_HCI_VS_HW_PLAT 0
#define BT_HCI_VS_HW_VAR  0
#endif /* CONFIG_SOC_FAMILY_NRF5 */

void hci_init(struct k_poll_signal *signal_host_buf);
struct net_buf *hci_cmd_handle(struct net_buf *cmd);
#if !defined(BFLB_BLE)
void hci_evt_encode(struct radio_pdu_node_rx *node_rx, struct net_buf *buf);
s8_t hci_get_class(struct radio_pdu_node_rx *node_rx);
#if defined(CONFIG_BT_CONN)
int hci_acl_handle(struct net_buf *acl, struct net_buf **evt);
void hci_acl_encode(struct radio_pdu_node_rx *node_rx, struct net_buf *buf);
void hci_num_cmplt_encode(struct net_buf *buf, u16_t handle, u8_t num);
#endif
#endif//!defined(BFLB_BLE)
#endif /* _HCI_CONTROLLER_H_ */
