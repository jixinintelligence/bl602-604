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
/****************************************************************************
FILE NAME
    ble_tp_svc.h

DESCRIPTION
NOTES
*/
/****************************************************************************/

#ifndef _BLE_TP_SVC_H_
#define _BLE_TP_SVC_H_

#include "config.h"

//07af27a6-9c22-11ea-9afe-02fcdc4e7412
#define BT_UUID_SVC_BLE_TP              BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x07af27a6, 0x9c22, 0x11ea, 0x9afe, 0x02fcdc4e7412))
//07af27a7-9c22-11ea-9afe-02fcdc4e7412
#define BT_UUID_CHAR_BLE_TP_TX      BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x07af27a7, 0x9c22, 0x11ea, 0x9afe, 0x02fcdc4e7412))
//07af27a8-9c22-11ea-9afe-02fcdc4e7412
#define BT_UUID_CHAR_BLE_TP_RX      BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x07af27a8, 0x9c22, 0x11ea, 0x9afe, 0x02fcdc4e7412))


#define BT_CHAR_BLE_TP_TX_ATTR_INDEX	(2)
#define BT_CHAR_BLE_TP_RX_ATTR_INDEX	(4)

bool enable_notify;

int ble_tp_notify();
void ble_tp_init();
struct bt_gatt_attr *get_attr(u8_t index);

#endif 

