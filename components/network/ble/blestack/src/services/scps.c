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
#include "bluetooth.h"
#include "gatt.h"
#include "uuid.h"
#include "scps.h"

struct scan_intvl_win {
	u16_t scan_intvl;
	u16_t scan_win;
} __packed;

static struct scan_intvl_win intvl_win = {
	.scan_intvl = BT_GAP_SCAN_FAST_INTERVAL,
	.scan_win = BT_GAP_SCAN_FAST_WINDOW,
};

static ssize_t scan_intvl_win_write(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr, const void *buf,
			  u16_t len, u16_t offset, u8_t flags)
{
    const u8_t *data = buf;
    intvl_win.scan_intvl = sys_get_le16(data);
    data += 2;
    intvl_win.scan_win = sys_get_le16(data);
}

static struct bt_gatt_attr attrs[]= {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_SCPS),
	BT_GATT_CHARACTERISTIC(BT_UUID_SCPS_SCAN_INTVL_WIN,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_NONE, NULL, NULL,
			       &intvl_win)
};

static struct bt_gatt_service scps = BT_GATT_SERVICE(attrs);

bool scps_init(u16_t scan_intvl, u16_t scan_win)
{	
    int err;
    
    if (scan_intvl < 0x0004 || scan_intvl > 0x4000) {
        return false;
    }
    
    if (scan_win < 0x0004 || scan_win > 0x4000) {
        return false;
    }
    
    if (scan_win > scan_intvl) {
        return false;
    }

    intvl_win.scan_intvl = scan_intvl;
    intvl_win.scan_win = scan_win;
    
    err = bt_gatt_service_register(&scps);

    return err?false:true;
}
