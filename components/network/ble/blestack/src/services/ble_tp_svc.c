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
    ble_tp_svc.c

DESCRIPTION
NOTES
*/
/****************************************************************************/

#include <errno.h>
#include <stdbool.h>

#include "bluetooth.h"
#include "conn.h"
#include "gatt.h"
#include "uuid.h"
#include "ble_tp_svc.h"


static void ble_tp_connected(struct bt_conn *conn, u8_t err);
static void ble_tp_disconnected(struct bt_conn *conn, u8_t reason);


struct bt_conn *ble_tp_conn;


static struct bt_conn_cb ble_tp_conn_callbacks = {
	.connected	=   ble_tp_connected,
	.disconnected	=   ble_tp_disconnected,
};

/*************************************************************************
NAME    
    ble_tp_connected
*/
static void ble_tp_connected(struct bt_conn *conn, u8_t err)
{   
	printf("%s\n",__func__);

	ble_tp_conn = conn;
}

/*************************************************************************
NAME    
    ble_tp_disconnected
*/
static void ble_tp_disconnected(struct bt_conn *conn, u8_t reason)
{ 
	printf("%s\n",__func__);
	
	ble_tp_conn = NULL;
}

/*************************************************************************
NAME    
    ble_tp_notify
*/ 

int ble_tp_notify()
{
	int err = -1;
	u16_t slen = 244;

	char data[244] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

	if(ble_tp_conn && enable_notify)
	{
	       printf("bletp tx\n");
		err = bt_gatt_notify(ble_tp_conn, get_attr(BT_CHAR_BLE_TP_TX_ATTR_INDEX), data, slen);
	}

	return err;
}

/*************************************************************************
NAME    
    ble_tp_ccc_cfg_changed
*/ 
static void ble_tp_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	enable_notify = (value == BT_GATT_CCC_NOTIFY);
	printf("%s, Notify enable? %d\n", __func__, enable_notify);

}

/*************************************************************************
NAME    
    ble_tp_recv
*/ 
static int ble_tp_recv(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr, const void *buf,
			  u16_t len, u16_t offset, u8_t flags)
{
	printf("bletp rx\n");
	return 0;
}

/*************************************************************************
*  DEFINE : attrs 
*/
static struct bt_gatt_attr attrs[]= {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_SVC_BLE_TP),

	BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_BLE_TP_TX,
							BT_GATT_CHRC_NOTIFY,
							BT_GATT_PERM_READ, 
							NULL, 
							NULL,
							NULL),

	BT_GATT_CCC(ble_tp_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_BLE_TP_RX,
							BT_GATT_CHRC_WRITE_WITHOUT_RESP,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							NULL, 
							ble_tp_recv,
							NULL)
};

/*************************************************************************
NAME    
    get_attr
*/
struct bt_gatt_attr *get_attr(u8_t index)
{
	return &attrs[index];
}


struct bt_gatt_service ble_tp_server = BT_GATT_SERVICE(attrs);


/*************************************************************************
NAME    
    ble_tp_init
*/
void ble_tp_init()
{
	bt_conn_cb_register(&ble_tp_conn_callbacks);
	bt_gatt_service_register(&ble_tp_server);
}



