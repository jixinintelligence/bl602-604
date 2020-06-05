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
#include <stdlib.h>
#include "conn.h"
#include "gatt.h"
#include "hci_core.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cli.h"
#include "stack_cli_cmds.h"

#if defined(CONFIG_BT_STACK_PTS)
#include "keys.h"
#include "rpa.h"
#endif
#define 		PASSKEY_MAX  		0xF423F
#define 		NAME_LEN 			30
#define 		CHAR_SIZE_MAX       512

static u8_t 	selected_id = BT_ID_DEFAULT;
bool 			ble_inited 	= false;

#if defined(CONFIG_BT_CONN)
struct bt_conn *default_conn = NULL;
#endif

#if defined(CONFIG_BT_STACK_PTS)

#define LIM_ADV_TIME    		30720
#define LIM_SCAN_TIME   		10240

const u8_t peer_irk[16] = 
{
		0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef,
		0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef
};
			    
const u8_t pts_address[6] = 
{
		0xE9,0x20,0xF2,0xDC,0x1b,0x00
};		

const u8_t discover_mode[] = 
{
		(BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR)
};

static const u8_t service_uuid[16] = 
{
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f
};
							   		 		   
static const u8_t service_data[]  		= {0x00,0x01};					   		
static const u8_t data_manu[4]  		= {0x71,0x01,0x04,0x13};
static const u8_t tx_power[1]   		= {0x50};
static const u8_t data_appearance[2] 	= {0x80, 0x01};
static const u8_t name[] 				= "BL70X-BLE-DEV";		
extern volatile u8_t	event_flag;

#endif

struct bt_data ad_discov[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    #if defined(BL602)
    BT_DATA(BT_DATA_NAME_COMPLETE, "BL602-BLE-DEV", 13),
    #else
    BT_DATA(BT_DATA_NAME_COMPLETE, "BL70X-BLE-DEV", 13),
    #endif
};
#if defined(BL602) || (BL702)
#define vOutputString(...)  printf(__VA_ARGS__)
#else
#define vOutputString(...)  bl_print(SYSTEM_UART_ID, PRINT_MODULE_BLE_STACK/*PRINT_MODULE_CLI*/, __VA_ARGS__)
#endif

static void cmd_init(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_get_device_name(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_set_device_name(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_start_scan(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_stop_scan(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_read_local_address(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_start_advertise(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_stop_advertise(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_connect_le(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_disconnect(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_select_conn(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_unpair(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_conn_update(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_security(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_auth(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_auth_cancel(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_auth_passkey_confirm(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_auth_pairing_confirm(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_auth_passkey(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_exchange_mtu(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_discover(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_read(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_write(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_write_without_rsp(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_subscribe(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_unsubscribe(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_set_data_len(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);

#if defined(CONFIG_BT_STACK_PTS)
static void cmd_start_scan_timeout(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_add_dev_to_resolve_list(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_pts_address_register(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_set_flag(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_set_smp_flag(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_clear_smp_flag(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static int  cmd_bondable(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_read_uuid(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static int  cmd_mread(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv); 
static void cmd_discover_uuid_128(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_read_uuid_128(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#if defined(CONFIG_BT_SMP)
static void cmd_set_mitm(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif
#if defined(PTS_GAP_SLAVER_CONFIG_NOTIFY_CHARC)
static void cmd_notify(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif
#if defined(PTS_GAP_SLAVER_CONFIG_INDICATE_CHARC)
static void cmd_indicate(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif
static int cmd_register_pts_svc(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#if defined(CONFIG_BT_WHITELIST)
static int cmd_bt_le_whitelist_add(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static int cmd_wl_connect(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif
static void cmd_prepare_write(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif

#if defined(CONFIG_SET_TX_PWR)
static void cmd_set_tx_pwr(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif

#if defined(BL602)||(BL702)
const struct cli_command btStackCmdSet[] STATIC_CLI_CMD_ATTRIBUTE = {
#else
const struct cli_command btStackCmdSet[] = {
#endif
    /*1.The cmd string to type, 2.Cmd description, 3.The function to run, 4.Number of parameters*/
    {"cmd_init", "\r\ncmd_init:[Initialize]\r\n Parameter[Null]\r\n", cmd_init},
	{"cmd_get_device_name", "\r\ncmd_get_device_name:[Read local device name]\r\n Parameter[Null]\r\n", cmd_get_device_name},
	{"cmd_set_device_name", "\r\ncmd_set_device_name:\r\n\
	 [Lenth of name]\r\n\
	 [name]\r\n", cmd_set_device_name},
	#if defined(CONFIG_BT_STACK_PTS)
	{"cmd_pts_address_register", "\r\ncmd_pts_address_register:\r\n\
     [Address type, 0:non-rpa, 1:rpa, 2:public adderss]\r\n\
     [Pts_address, e.g.peer_addr]\r\n" , cmd_pts_address_register}, 

	{"cmd_set_flag", "\r\ncmd_set_flag:[Set flag for different applications]\r\n\
     [Flag, e.g.0,1]\r\n", cmd_set_flag},
    {"cmd_set_smp_flag", "\r\ncmd_set_smp_flag:[Set flag for SM test]\r\n\
     [Flag, e.g.0,1]\r\n", cmd_set_smp_flag},
    {"cmd_clear_smp_flag", "\r\ncmd_clear_smp_flag:[Clear smp test flag]\r\n\
     [Flag, e.g.0,1]\r\n", cmd_clear_smp_flag},

	{"cmd_bondable", "\r\ncmd_bondable:[Enable or disable bond ]\r\n\
     [State, e.g.0x01:bondable mode, 0x00:non-bondable mode]\r\n", cmd_bondable},
	#endif

    #if defined(CONFIG_BT_OBSERVER)
    #if defined(CONFIG_BT_STACK_PTS)
    {"cmd_start_scan", "\r\ncmd_start_scan:\r\n\
     [Scan type, 0:passive scan, 1:active scan]\r\n\
     [Duplicate filtering, 0:Disable duplicate filtering, 1:Enable duplicate filtering]\r\n\
     [Scan interval, 0x0004-4000,e.g.0080]\r\n\
     [Scan window, 0x0004-4000,e.g.0050]\r\n\
     [Is_RPA,0:non-rpa, 1:rpa]\r\n", cmd_start_scan},
     
    {"cmd_start_scan_timeout", "\r\ncmd_start_scan_timeout:\r\n\
     [Scan type, 0:passive scan, 1:active scan]\r\n\
     [Duplicate filtering, 0:Disable duplicate filtering, 1:Enable duplicate filtering]\r\n\
     [Scan interval, 0x0004-4000,e.g.0080]\r\n\
     [Scan window, 0x0004-4000,e.g.0050]\r\n\
     [Is_RPA, 0:non-rpa, 1:rpa]\r\n",cmd_start_scan_timeout},
     //[Time, e.g. 2800]\r\n" , cmd_start_scan_timeout},

	{"cmd_add_dev_to_resolve_list", "\r\ncmd_add_dev_to_resolve_list:\r\n\
     [Address type, 0:non-rpa, 1:rpa, 2:public adderss]\r\n\
     [Peer_address, e.g.peer_addr]\r\n" , cmd_add_dev_to_resolve_list},

    #else
    {"cmd_start_scan", "\r\ncmd_start_scan:\r\n\
     [Scan type, 0:passive scan, 1:active scan]\r\n\
     [Duplicate filtering, 0:Disable duplicate filtering, 1:Enable duplicate filtering]\r\n\
     [Scan interval, 0x0004-4000,e.g.0080]\r\n\
     [Scan window, 0x0004-4000,e.g.0050]\r\n", cmd_start_scan},
    #endif//CONFIG_BT_STACK_PTS
    {"cmd_stop_scan", "\r\ncmd_stop_scan:[Stop scan]\r\n\
      Parameter[Null]\r\n", cmd_stop_scan},
    #endif

    #if defined(CONFIG_BT_PERIPHERAL)
    #if defined(CONFIG_BT_STACK_PTS)
    {"cmd_start_adv", "\r\ncmd_start_adv:\r\n\
     [Adv type,0:adv_ind,1:adv_scan_ind,2:adv_nonconn_ind 3: adv_direct_ind]\r\n\
     [Mode, 0:discov, 1:non-discov,2:limit discoverable]\r\n\
     [Addr_type, 0:non-rpa,1:rpa,2:public]\r\n\
     [Adv Interval Min,0x0020-4000,e.g.0030]\r\n\
     [Adv Interval Max,0x0020-4000,e.g.0060]\r\n", cmd_start_advertise},
	#if defined(PTS_GAP_SLAVER_CONFIG_NOTIFY_CHARC)
    {"cmd_notify", "\r\ncmd_notify:\r\n\
     [Lenth, e.g. 0000]\r\n\
     [Data,  e.g. 00]\r\n",cmd_notify},
    #endif /*PTS_GAP_SLAVER_CONFIG_NOTIFY_CHARC*/
    #if defined(PTS_GAP_SLAVER_CONFIG_INDICATE_CHARC)
    {"cmd_indicate", 	"\r\ncmd_indicate:\r\n\
     [Lenth, e.g. 0000]\r\n\
     [Data,  e.g. 00]\r\n",cmd_indicate},
    #endif /*PTS_GAP_SLAVER_CONFIG_NOTIFY_CHARC*/
    
	{"cmd_register_pts_svc", "\r\ncmd_notify:\r\n",cmd_register_pts_svc},
    #else 
    {"cmd_start_adv", "\r\ncmd_start_adv:\r\n\
     [Adv type,0:adv_ind,1:adv_scan_ind,2:adv_nonconn_ind]\r\n\
     [Mode, 0:discov, 1:non-discov]\r\n\
     [Adv Interval Min,0x0020-4000,e.g.0030]\r\n\
     [Adv Interval Max,0x0020-4000,e.g.0060]\r\n", cmd_start_advertise},
    #endif /*CONFIG_BT_STACK_PTS*/
     
    {"cmd_stop_adv", "\r\ncmd_stop_adv:[Stop advertising]\r\n\
     Parameter[Null]\r\n", cmd_stop_advertise},
	{"cmd_read_local_address", "\r\ncmd_read_local_address:[Read local address]\r\n", 
	  cmd_read_local_address},
     
    #endif //#if defined(CONFIG_BT_PERIPHERAL)

    #if defined(CONFIG_BT_CONN)
    #if defined(CONFIG_BT_CENTRAL)
    #if defined(CONFIG_BT_STACK_PTS)
	{"cmd_connect_le", "\r\ncmd_connect_le:[Connect remote device]\r\n\
     [Address type,0:ADDR_RAND, 1:ADDR_RPA_OR_PUBLIC,2:ADDR_PUBLIC, 3:ADDR_RPA_OR_RAND]\r\n\
     [Address value, e.g.112233AABBCC]\r\n", cmd_connect_le},

	#if defined(CONFIG_BT_WHITELIST)
	{"cmd_wl_connect", "\r\ncmd_wl_connect:[Autoconnect whilt list device]\r\n\
	 [Enable, 0x01:enable, 0x02:disable]\r\n\
	 [Address type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND]\r\n",cmd_wl_connect},

	{"cmd_bt_le_whitelist_add", "\r\ncmd_bt_le_whitelist_add:[add device to white list]\r\n\
	 [Address type,0:ADDR_RAND, 1:ADDR_RPA_OR_PUBLIC,2:ADDR_PUBLIC, 3:ADDR_RPA_OR_RAND]\r\n\
	 [Address value, e.g.112233AABBCC]\r\n",cmd_bt_le_whitelist_add},
	#endif 
    #else
    {"cmd_connect_le", "\r\ncmd_connect_le:[Connect remote device]\r\n\
     [Address type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND]\r\n\
     [Address value, e.g.112233AABBCC]\r\n", cmd_connect_le},
    #endif //CONFIG_BT_STACK_PTS
    #endif //#if defined(CONFIG_BT_CENTRAL)
    
    #if defined(CONFIG_BT_STACK_PTS)
	{"cmd_disconnect", "\r\ncmd_disconnect:[Disconnect remote device]\r\n\
     [Address type, 0:ADDR_RAND, 1:ADDR_RPA_OR_PUBLIC,2:ADDR_PUBLIC, 3:ADDR_RPA_OR_RAND]\r\n\
     [Address value,e.g.112233AABBCC]\r\n", cmd_disconnect},
	#else
    {"cmd_disconnect", "\r\ncmd_disconnect:[Disconnect remote device]\r\n\
     [Address type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND]\r\n\
     [Address value,e.g.112233AABBCC]\r\n", cmd_disconnect},
    #endif //CONFIG_BT_STACK_PTS
	
    {"cmd_select_conn", "\r\ncmd_select_conn:[Select a specific connection]\r\n\
     [Address type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND]\r\n\
     [Address value, e.g.112233AABBCC]\r\n", cmd_select_conn},
     
    {"cmd_unpair", "\r\ncmd_unpair:[Unpair connection]\r\n\
     [Address type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND]\r\n\
     [Address value, all 0: unpair all connection, otherwise:unpair specific connection]\r\n", cmd_unpair},

    {"cmd_conn_update", "\r\ncmd_conn_update:\r\n\
     [Conn Interval Min,0x0006-0C80,e.g.0030]\r\n\
     [Conn Interval Max,0x0006-0C80,e.g.0030]\r\n\
     [Conn Latency,0x0000-01f3,e.g.0004]\r\n\
     [Supervision Timeout,0x000A-0C80,e.g.0010]\r\n", cmd_conn_update},
    #endif //#if defined(CONFIG_BT_CONN)
 
    #if defined(CONFIG_BT_SMP)
    {"cmd_security", "\r\ncmd_security:[Start security]\r\n\
     [Security level, Default value 4, 2:BT_SECURITY_MEDIUM, 3:BT_SECURITY_HIGH, 4:BT_SECURITY_FIPS]\r\n", cmd_security},
    {"cmd_auth", "\r\ncmd_auth:[Register auth callback]\r\n", cmd_auth},
    {"cmd_auth_cancel", "\r\ncmd_auth_cancel:[Register auth callback]\r\n", cmd_auth_cancel},
    {"cmd_auth_passkey_confirm", "\r\ncmd_auth_passkey_confirm:[Confirm passkey]\r\n", cmd_auth_passkey_confirm},
    {"cmd_auth_pairing_confirm", "\r\ncmd_auth_pairing_confirm:[Confirm pairing in secure connection]\r\n", cmd_auth_pairing_confirm},
    {"cmd_auth_passkey", "\r\ncmd_auth_passkey:[Input passkey]\r\n\
     [Passkey, 00000000-000F423F]", cmd_auth_passkey},

    #if defined(CONFIG_BT_STACK_PTS)
	 {"cmd_set_mitm", "\r\ncmd_set_mitm:[set MIMT]\r\n\
     [State, 0x01:define,0x00:undefine]\r\n", cmd_set_mitm},
    #endif
    
    #endif //#if defined(CONFIG_BT_SMP)

    #if defined(CONFIG_BT_GATT_CLIENT)
    {"cmd_exchange_mtu", "\r\ncmd_exchange_mtu:[Exchange mtu]\r\n Parameter[Null]\r\n", cmd_exchange_mtu},
    {"cmd_discover", "\r\ncmd_discover:[Gatt discovery]\r\n\
     [Discovery type, 0:Primary, 1:Secondary, 2:Include, 3:Characteristic, 4:Descriptor]\r\n\
     [Uuid value, 2 Octets, e.g.1800]\r\n\
     [Start handle, 2 Octets, e.g.0001]\r\n\
     [End handle, 2 Octets, e.g.ffff]\r\n", cmd_discover},
	#if defined(CONFIG_BT_STACK_PTS)
	{"cmd_discover_uuid_128", "\r\ncmd_discover_uuid_128:[Gatt discovery]\r\n\
     [Discovery type, 0:Primary, 1:Secondary, 2:Include, 3:Characteristic, 4:Descriptor]\r\n\
     [Uuid, 16 Octets, e.g.0000A00C000000000123456789abcdef]\r\n\
     [Start handle, 2 Octets, e.g.0001]\r\n\
	 [End handle, 2 Octets, e.g.ffff]\r",cmd_discover_uuid_128},
	#endif
    {"cmd_read", "\r\ncmd_read:[Gatt Read]\r\n\
     [Attribute handle, 2 Octets]\r\n\
     [Value offset, 2 Octets]\r\n", cmd_read},
	#if defined(CONFIG_BT_STACK_PTS)
	{"cmd_read_uuid_128", "\r\ncmd_read_uuid:[Gatt Read by uuid 128 bit]\r\n\
     [Uuid, 16 Octets]\r\n\
     [Start_handle, 2 Octets]\r\n\
     [End_handle, 2 Octets]\r\n", cmd_read_uuid_128},
	{"cmd_read_uuid", "\r\ncmd_read_uuid:[Gatt Read by uuid]\r\n\
     [Uuid, 2 Octets]\r\n\
     [Start_handle, 2 Octets]\r\n\
     [End_handle, 2 Octets]\r\n", cmd_read_uuid},
    {"cmd_mread", "\r\ncmd_mread:[Gatt Read multiple]\r\n\
     <handle 1> <handle 2> ...\r\n",cmd_mread},
	#endif
    {"cmd_write", "\r\ncmd_write:[Gatt write]\r\n\
     [Attribute handle, 2 Octets]\r\n\
     [Value offset, 2 Octets]\r\n\
     [Value length, 2 Octets]\r\n\
     [Value data]\r\n", cmd_write},

	#if defined(CONFIG_BT_STACK_PTS)
	 {"cmd_prepare_write", "\r\ncmd_prepare_write:[Gatt prepare write]\r\n\
     [Attribute handle, 2 Octets]\r\n\
     [Value offset, 2 Octets]\r\n\
     [Value length, 2 Octets]\r\n\
     [Value data]\r\n", cmd_prepare_write},
	#endif
	
    {"cmd_write_without_rsp", "\r\ncmd_write_without_rsp:[Gatt write without response]\r\n\
     [Sign, 0: No need signed, 1:Signed write cmd if no smp]\r\n\
     [Attribute handle, 2 Octets]\r\n\
     [Value length, 2 Octets]\r\n\
     [Value data]\r\n", cmd_write_without_rsp},

    {"cmd_subscribe", "\r\ncmd_subscribe:[Gatt subscribe]\r\n\
     [CCC handle, 2 Octets]\r\n\
     [Value handle, 2 Octets]\r\n\
     [Value, 1:notify, 2:indicate]\r\n", cmd_subscribe},
     {"cmd_unsubscribe", "\r\ncmd_unsubscribe:[Gatt unsubscribe]\r\n Parameter[Null]\r\n", cmd_unsubscribe},
    #endif /*CONFIG_BT_GATT_CLIENT*/

    {"cmd_set_data_len",
    "\r\ncmd_set_data_len:[LE Set Data Length]\r\n\
    [tx octets, 2 octets]\r\n\
    [tx time, 2 octets]\r\n",
    cmd_set_data_len},

    #if defined(CONFIG_SET_TX_PWR)
    {"cmd_set_tx_pwr",
    "\r\ncmd_set_tx_pwr:[Set tx power mode]\r\n\
    [mode, 1 octet, value:5,6,7]\r\n",
    cmd_set_tx_pwr},
    #endif

    #if defined(BL70X)
    {NULL, "No handler/Invalid command", NULL},
    #endif

};
#if defined(CONFIG_BT_PERIPHERAL)
#if defined(CONFIG_BT_STACK_PTS)
#if defined(PTS_GAP_SLAVER_CONFIG_READ_CHARC)
static u8_t report[]= {
		0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,
		0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,
		0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09
};

static ssize_t read_pts_long_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
	const char *lvalue = "PTS-LONG-VALUE0123456789abcdef1122";

	return bt_gatt_attr_read(conn, attr, buf, len, offset, lvalue,
				 strlen(lvalue));
}

static ssize_t read_pts_name(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
	const char *name = "PTS_NAME";

	return bt_gatt_attr_read(conn, attr, buf, len, offset, name,strlen(name));
}

static ssize_t pts_read_report(
			struct bt_conn *conn,
			const struct bt_gatt_attr *attr, void *buf,
			u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, 
				attr->user_data,
				sizeof(report)/sizeof(u8_t));
}

#endif /* PTS_GAP_SLAVER_CONFIG_READ_CHARC */

#if defined(PTS_GAP_SLAVER_CONFIG_WRITE_CHARC)
#define 	TEST_LVAL_MAX    	30
#define 	TEST_SVAL_MAX   	4

static u16_t test_len 			= 0;
static u16_t test_slen 			= 0;

static u8_t short_buf[TEST_SVAL_MAX]= 
{
			0x01,0x02,0x03,0x04				
};
static u8_t long_buf[TEST_LVAL_MAX]	=
{
			0x01,0x02,0x03,0x04,
			0x05,0x06,0x07,0x08,
			0x09,0x0a,0x11,0x12,
			0x13,0x14,0x15,0x16,
			0x17,0x18,0x19,0x1a,
			0x21,0x22,0x23,0x24,
			0x25,0x26,0x27,0x28,
			0x29,0x2a
};	

static ssize_t pts_write_short_value(struct bt_conn *conn,const struct bt_gatt_attr *bt_attr, 
					void *buf,u16_t len, u16_t offset,u8_t flags)
{
	u8_t i 			= 0;
	u16_t tlen 		= len;
	u8_t *data 		= (u8_t*)buf;

	for(i=0;i<tlen;i++)
		vOutputString("data[%d]->[0x%x]\r\n",i,data[i]);

	/*The rest of space is enough.*/
	if(TEST_SVAL_MAX - test_slen >= tlen)
	{
		(void)memcpy(&short_buf[test_slen],data,tlen);
		test_slen += tlen;
	}
	else
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	
	/*If prepare write, it will return 0 */
	if(flags == BT_GATT_WRITE_FLAG_PREPARE)
		tlen = 0;
		
	return tlen;
}

static ssize_t pts_read_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{	
	u8_t *data 		= short_buf;
	u16_t data_len 	= sizeof(short_buf)/sizeof(u8_t);

	if(test_len)
	{
		data_len = test_len;
		data = long_buf;
		test_len = 0;
	}
	
	return bt_gatt_attr_read(conn, attr, buf, len, offset, data,data_len);
}


static ssize_t pts_read_long_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
	u8_t *data 		= long_buf;
	u16_t data_len 	= sizeof(long_buf)/sizeof(u8_t);

	/*Get new value */
	if(test_len > 0 && test_len <= TEST_LVAL_MAX)
	{
		data = long_buf;
		data_len = test_len;
	}
	
	return bt_gatt_attr_read(conn, attr, buf, len, offset, data,data_len);
}

static ssize_t pts_write_long_value(
				struct bt_conn *conn,
				const struct bt_gatt_attr *bt_attr, 
				void *buf,u16_t len, u16_t offset,u8_t flags)
{
	u16_t tlen 		= len;
	u8_t *data 		= (u8_t*)buf;
	u8_t i		 	= 0;

	/*Reset test value */
	if(!offset)
	{
		test_len = 0;
		(void)memset(long_buf,0,TEST_LVAL_MAX);
	}
	else if(offset > TEST_LVAL_MAX)
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	/*The rest of space is enough.*/
	if(TEST_LVAL_MAX - test_len >= tlen)
	{
		(void)memcpy(&long_buf[test_len],data,tlen);
		test_len += tlen;
	}
	else
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	
	/*If prepare write, it will return 0 */
	if(flags == BT_GATT_WRITE_FLAG_PREPARE)
		tlen = 0;
	
	return tlen;
}

#endif /*PTS_GAP_SLAVER_CONFIG_WRITE_CHARC*/

#if defined(PTS_GAP_SLAVER_CONFIG_NOTIFY_CHARC) || defined(PTS_GAP_SLAVER_CONFIG_INDICATE_CHARC)
static bool notify_status 	= 0;
static u8_t battery_level 	= 100;

static void notify(const struct bt_gatt_attr *attr, u16_t value)
{
	ARG_UNUSED(attr);	
	
	notify_status = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t pts_read_value(
			struct bt_conn *conn,	
			const struct bt_gatt_attr *bt_attr, 
			void *buf,u16_t len, u16_t offset,u8_t flags)
{
	const char *data 	= "PTS_NAME";
	u16_t length 		= strlen(data);

	return bt_gatt_attr_read(conn, bt_attr, buf, len, offset, data,length);
}

static ssize_t pts_write_value(
			struct bt_conn *conn,
			const struct bt_gatt_attr *bt_attr, 
			void *buf,u16_t len, u16_t offset,u8_t flags)
{
	return len;
}

#endif /*PTS_GAP_SLAVER_CONFIG_NOTIFY_CHARC*/


static ssize_t pts_read_value(
			struct bt_conn *conn,	
			const struct bt_gatt_attr *bt_attr, 
			void *buf,u16_t len, u16_t offset,u8_t flags)
{
	const char *data 	= "PTS_NAME";
	u16_t length 		= strlen(data);

	return bt_gatt_attr_read(conn, bt_attr, buf, len, offset, data,length);
}


static struct bt_gatt_attr pts_attr[] = {

	BT_GATT_CHARACTERISTIC
	(		
			BT_UUID_PTS_AUTH_CHAR, 
			BT_GATT_CHRC_READ,
			BT_GATT_PERM_READ_AUTHEN, 
			pts_read_value, 
			NULL, 
			NULL
	),

	#if defined(PTS_GAP_SLAVER_CONFIG_WRITE_CHARC)

	/* Case : GATT/SR/GAR/BV-03-C
	 * Verify that a Generic Attribute Profile server can support writing a Characteristic
	 * Value selected by handle
	 */	
	BT_GATT_CHARACTERISTIC
	(
			BT_UUID_PTS_CHAR_WRITE_VALUE,
	        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
	        pts_read_value,
	        pts_write_short_value, 
	        NULL
	),

	/* Case : GATT/SR/GAR/BV-01-C
	 * Verify that a Generic Attribute Profile server can support a write to a 
	 * characteristic without response
	 */	
    BT_GATT_CHARACTERISTIC
    (
    		BT_UUID_PTS_CHAR_WRITE_NORSP,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			pts_read_value, 
			pts_write_short_value,
			NULL
	),
			        
	/* Case : GATT/SR/GAR/BI-03-C, GATT/SR/GAR/BI-12-C 
	 * Verify that a Generic Attribute Profile server can detect and reject a 
	 * Write Characteristic Value Request to a non-writeable Characteristic Value 
	 * and issue a Write Not Permitted Response.
	 */			        
	BT_GATT_CHARACTERISTIC
	(
			BT_UUID_PTS_CHAR_WRITE_AUTHEN,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, 
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_AUTHEN,
			pts_read_long_value, 
			pts_write_long_value,
			NULL
	),
	
	BT_GATT_CHARACTERISTIC
	(		
			BT_UUID_PTS_CHAR_WRITE_AUTHEN,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, 
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_AUTHEN,
			pts_read_value, 
			pts_write_long_value, 
			NULL
	),

	/* Case : GATT/SR/GAW/BV-05-C */
	BT_GATT_CHARACTERISTIC
	(		
			BT_UUID_PTS_CHAR_WRITE_LONGVAL,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, 
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			pts_read_long_value, 
			pts_write_long_value, 
			NULL
	),
			        
	/* Case : GATT/SR/GAW/BV-10-C */
    BT_GATT_CHARACTERISTIC
    (		
    		BT_UUID_PTS_CHAR_WRITE_2LONGVAL,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, 
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			pts_read_long_value, 
			pts_write_long_value, 
			NULL
	),

	BT_GATT_DESCRIPTOR
	(		
			BT_UUID_PTS_CHAR_WRITE_L_DES, 
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			pts_read_long_value, 
			pts_write_long_value, 
			NULL
	),

#endif

	#if defined(PTS_GAP_SLAVER_CONFIG_READ_CHARC)
	
	/* Case : GATT/SR/GAR/BI-04-C
	 * Using authenticated link-key for read access. handle is 15
	 */
	BT_GATT_CHARACTERISTIC
	(
			BT_UUID_PTS_CHAR_READ_AUTHEN,
	       	BT_GATT_CHRC_READ, 
	       	BT_GATT_PERM_READ_AUTHEN,
	       	read_pts_name, 
	       	NULL, 
	       	NULL
	),

	/* Case : GATT/SR/GAR/BI-06-C
	 * Verify that a Generic Attribute Profile server can detect and reject a Read 
	 * Characteristic by UUID Request to a non-readable Characteristic Value and issue 
	 * a Read Not Permitted Response
	 */
	BT_GATT_CHARACTERISTIC
	(		
			BT_UUID_PTS_CHAR_READ_NOPERM,
			BT_GATT_CHRC_READ,
			BT_GATT_PERM_NONE,
			read_pts_name, 
			NULL, 
			NULL
	),

	/* Case : GATT/SR/GAR/BV-04-C;GATT/SR/GAR/BI-13-C
	 * Verify that a Generic Attribute Profile server can support reading a 
	 * long Characteristic Value selected by handle.
	 */
	BT_GATT_CHARACTERISTIC
	(
			BT_UUID_PTS_CHAR_READ_LONGVAL,
			BT_GATT_CHRC_READ, 
			BT_GATT_PERM_READ,
			read_pts_long_value, 
			NULL, 
			NULL
	),

	/* Case : GATT/SR/GAR/BV-06-C;GATT/SR/GAR/BV-07-C;GATT/SR/GAR/BV-08-C
	 * Verify that a Generic Attribute Profile server can support reading a Long 
	 * Characteristic Descriptor selected by handle.
	 */
	BT_GATT_DESCRIPTOR
	(
			BT_UUID_PTS_CHAR_READ_LVAL_REF, 
			BT_GATT_PERM_READ,
			pts_read_report, 
			NULL,  
			report
	),
	       
	/* Case : GATT/SR/GAR/BI-12-C
	 * Verify that a Generic Attribute Profile server can detect and reject a Read Long 
	 * Characteristic Value Request to a non-readable Characteristic Value and issue a 
	 * Read Not Permitted Response.
	 */	       
	BT_GATT_CHARACTERISTIC
	(
			BT_UUID_PTS_CHAR_READ_L_NOPERM,
			BT_GATT_CHRC_READ, 
			BT_GATT_PERM_NONE,
			read_pts_long_value, 
			NULL, 
			NULL
	),

	#endif /* PTS_GAP_SLAVER_CONFIG_READ_CHARC */

	#if defined(PTS_GAP_SLAVER_CONFIG_NOTIFY_CHARC)
	BT_GATT_CHARACTERISTIC
	(		
			BT_UUID_PTS_CHAR_NOTIFY_CHAR,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			BT_GATT_PERM_READ ,
			pts_read_value, 
			NULL, 
			&battery_level
	),
						   
	BT_GATT_CCC
	(		
			notify,	
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
	),
	#endif
	
	#if defined(PTS_GAP_SLAVER_CONFIG_INDICATE_CHARC)
	BT_GATT_CHARACTERISTIC
	(
			BT_UUID_PTS_CHAR_INDICATE_CHAR,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
			BT_GATT_PERM_READ ,
			pts_read_value, 
			NULL, 
			&battery_level
	),
						   
	BT_GATT_CCC
	(
			notify,	
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
	),
	#endif

};

struct bt_gatt_service pts_svc = BT_GATT_SERVICE(pts_attr);

static void notify_cb(struct bt_conn *conn, void *user_data)
{
	vOutputString("%s: Nofication sent to conn %p\r\n",__func__,conn);
}

static void cmd_notify(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	struct bt_gatt_notify_params 	params;
	struct bt_gatt_attr				bt_attr;
	u8_t 		data = 0;
	u16_t		len = 0;
	int			err;

	if(!default_conn)
	{
		vOutputString("Not connected\r\n");
		return;
	}
	
	co_get_uint16_from_string(&argv[1], &len);
	co_get_bytearray_from_string(&argv[2], (u8_t *)&data, len);

	memset(&params, 0, sizeof(params));

	params.uuid = pts_attr[0].uuid;
	params.attr = &pts_attr[0];
	params.data = &data;
	params.len 	= len;
	params.func = notify_cb;

	vOutputString("len = [0x%x]\r\n",params.len);
	
	err = bt_gatt_notify_cb(default_conn, &params);
	if(err)
	{
		vOutputString("Failed to notifition [%d]\r\n",err);
	}
}

static void indicate_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	if (err != 0U) 
	{
		vOutputString("Indication fail");	
	} 
	else 
	{
		vOutputString("Indication success");
	}
}

static void cmd_indicate(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	struct bt_gatt_indicate_params 	params;
	struct bt_gatt_attr				bt_attr;
	u8_t 		data[16];
	u16_t		len = 0;
	int			err;

	if(!default_conn)
	{
		vOutputString("Not connected\r\n");
		return;
	}
	
	co_get_uint16_from_string(&argv[1], &len);
	
	if(len > 16)
	{ 
		len = 16;
	}
	
	co_get_bytearray_from_string(&argv[2], data, len);

	memset(&params, 0, sizeof(params));

	params.attr = &pts_attr[0];
	params.data = data;
	params.len 	= len;
	params.func = indicate_cb;

	vOutputString("len = [0x%x]\r\n",params.len);
	
	err = bt_gatt_indicate(default_conn, &params);
	if(err)
	{
		vOutputString("Failed to notifition [%d]\r\n",err);
	}
}


#endif /* CONFIG_BT_STACK_PTS */
#endif /* CONFIG_BT_PERIPHERAL */

#if defined(CONFIG_BT_CONN)
static void connected(struct bt_conn *conn, u8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		vOutputString("Failed to connect to %s (%u) \r\n", addr,
			     err);
		return;
	}

	vOutputString("Connected: %s \r\n", addr);

	if (!default_conn) {
		default_conn = conn;
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	vOutputString("Disconnected: %s (reason %u) \r\n", addr, reason);

	if (default_conn == conn) {
		default_conn = NULL;
	}
}

static void le_param_updated(struct bt_conn *conn, u16_t interval,
			     u16_t latency, u16_t timeout)
{
	vOutputString("LE conn param updated: int 0x%04x lat %d to %d \r\n", interval, latency, timeout);
}

#if defined(CONFIG_BT_SMP)
static void identity_resolved(struct bt_conn *conn, const bt_addr_le_t *rpa,
			      const bt_addr_le_t *identity)
{
	char addr_identity[BT_ADDR_LE_STR_LEN];
	char addr_rpa[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(identity, addr_identity, sizeof(addr_identity));
	bt_addr_le_to_str(rpa, addr_rpa, sizeof(addr_rpa));

	vOutputString("Identity resolved %s -> %s \r\n", addr_rpa, addr_identity);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	vOutputString("Security changed: %s level %u \r\n", addr, level);
}
#endif

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_SMP)
	.identity_resolved = identity_resolved,
	.security_changed = security_changed,
#endif
};
#endif //CONFIG_BT_CONN

static void cmd_init(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if(ble_inited){
        vOutputString("Has initialized \r\n");
        return;
    }

    #if defined(CONFIG_BT_CONN)
    default_conn = NULL;
    bt_conn_cb_register(&conn_callbacks);
    #endif
    ble_inited = true;
    vOutputString("Init successfully \r\n");
}

static void cmd_get_device_name(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	const char *device_name = bt_get_name();

	if(device_name){
		vOutputString("device_name: %s\r\n",device_name);
	}else
		vOutputString("Failed to read device name\r\n");
}

static void cmd_set_device_name(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	uint8_t 	dname[CONFIG_BT_DEVICE_NAME_MAX + 1];
	uint16_t   	len = 0;
	int	  		err = 0;

	(void)memset(dname,0,CONFIG_BT_DEVICE_NAME_MAX + 1);
	co_get_uint16_from_string(&argv[1], &len);
	if(len <= (CONFIG_BT_DEVICE_NAME_MAX + 1) && len > 0){
		co_get_bytearray_from_string(&argv[2], dname, len);
		vOutputString("%s\r\n",dname);
	}
	else{
		vOutputString("Length of name is invalid parameter\r\n");
		return;
	}
	
	err = bt_set_name((char*)dname);
	if(err){
		vOutputString("Failed to set device name\r\n");
	}else
		vOutputString("Set the device name successfully\r\n");
	
}


#if defined(CONFIG_BT_OBSERVER)
static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
    u8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
        len = (data->data_len > NAME_LEN - 1)?(NAME_LEN - 1):(data->data_len);
		memcpy(name, data->data, len);
		return false;		
	default:
		return true;
	}
}

#if defined(CONFIG_BT_STACK_PTS)
static bool ad_flag_data_cb(struct bt_data *data, void *user_data)
{
    char *ad_flag = user_data;
	
    switch(data->type){
        case BT_DATA_FLAGS:
            memcpy(ad_flag, data->data, data->data_len);
            return false;
		
        default:
            return true;
    }
}
#endif

#if defined(CONFIG_BT_STACK_PTS)
char *pts_cmplt_name = "PTS-GAP-224B";
char *pts_short_name = "PTS-GAP";
extern bt_addr_le_t pts_addr;
#endif
static void device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t evtype,
			 struct net_buf_simple *buf)
{
	char 		le_addr[BT_ADDR_LE_STR_LEN];
	char 		name[NAME_LEN];

	#if defined(CONFIG_BT_STACK_PTS)
	uint8_t 	i = 0;
	int 		err = 0;
	u8_t 		dst_address[6];
	char 		ad_flag[1];
	struct net_buf_simple 	abuf;
	struct bt_conn 			*conn;
	
	(void)memset(ad_flag, 0, sizeof(ad_flag));
	(void)memset(dst_address, 0, sizeof(dst_address));
	memcpy(&abuf,buf,sizeof(struct net_buf_simple));
	
	#endif

	(void)memset(name, 0, sizeof(name));
	bt_data_parse(buf, data_cb, name);
	bt_addr_le_to_str(addr, le_addr, sizeof(le_addr));
	
	#if defined(CONFIG_BT_STACK_PTS)
    if(!memcmp(&pts_addr, addr, sizeof(bt_addr_le_t)) ||
       !memcmp(name,pts_cmplt_name, sizeof(*pts_cmplt_name)) ||
       !memcmp(name,pts_short_name, sizeof(*pts_short_name))){
       
   		if(memcmp(&pts_addr, addr, sizeof(bt_addr_le_t)))
        	memcpy(&pts_addr, addr, sizeof(pts_addr));  
	
    #endif
	vOutputString("[DEVICE]: %s, AD evt type %u, RSSI %i %s \r\n",le_addr, evtype, rssi, name);
	
    #if defined(CONFIG_BT_STACK_PTS)
		
	bt_data_parse(&abuf,ad_flag_data_cb,ad_flag);

	if(*ad_flag & 0x01){	
	    vOutputString("Advertising data : 'Limited Discovered Mode' flag is set one\r\n");
	}else
	    vOutputString("Advertising data : 'Limited Discovered Mode' flag is not set\r\n");

	if(*ad_flag & 0x02){
		vOutputString("Advertising data : 'General Discovered Mode' flag is set one\r\n");
    }else
     	vOutputString("Advertising data : 'General Discovered Mode' flag is not set\r\n");
	}

    #endif
}

static void cmd_start_scan(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    struct bt_le_scan_param scan_param;
    int err;

    (void)err;

    #if defined(CONFIG_BT_STACK_PTS)
    u8_t is_rpa;

    if(argc != 6){
    #else
    if(argc != 5){
    #endif
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }

    co_get_bytearray_from_string(&argv[1], &scan_param.type, 1);
    
    co_get_bytearray_from_string(&argv[2], &scan_param.filter_dup, 1);
    
    co_get_uint16_from_string(&argv[3], &scan_param.interval);
    
    co_get_uint16_from_string(&argv[4], &scan_param.window);

    #if defined(CONFIG_BT_STACK_PTS)
    co_get_bytearray_from_string(&argv[5], (uint8_t *)&is_rpa, 1);
    err = bt_le_scan_start(&scan_param, device_found, is_rpa);
    #else
    err = bt_le_scan_start(&scan_param, device_found);
    #endif
    
    if(err){
        vOutputString("Failed to start scan (err %d) \r\n", err);
    }else{
        vOutputString("Start scan successfully \r\n");
    }
}

#if defined(CONFIG_BT_STACK_PTS)
static void cmd_add_dev_to_resolve_list(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	struct bt_keys key;
	bt_addr_le_t addr;
	u8_t type = 0;
	u8_t i=0;
	int err;

	memset(&key,0,sizeof(struct bt_keys));
	
	co_get_bytearray_from_string(&argv[1], &type, 1);	
	co_get_bytearray_from_string(&argv[2], (uint8_t *)addr.a.val, 6);

	if(type == 0)
		addr.type = BT_ADDR_LE_PUBLIC;
   	else if(type == 1)
		addr.type = BT_ADDR_LE_RANDOM;

	memcpy(&key.addr,&addr,sizeof(bt_addr_le_t));
	memcpy(key.irk.val,peer_irk,16);

    bt_id_add(&key);

}


#if defined(CONFIG_BT_PERIPHERAL)
static int cmd_register_pts_svc(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int		err;
	
	err = bt_gatt_service_register(&pts_svc);
	if(err){
		vOutputString("Failed to register PTS service\r\n");
	}else{
		vOutputString("Succeed to register PTS service\r\n");
	}

    return 0;
}
#endif
#if defined(CONFIG_BT_WHITELIST)
static int cmd_wl_connect(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
	unsigned char enable = 0U;
	struct bt_le_conn_param param = {
		.interval_min =  BT_GAP_INIT_CONN_INT_MIN,
		.interval_max =  BT_GAP_INIT_CONN_INT_MAX,
		.latency = 0,
		.timeout = 400,
	};
	/*Auto connect whitelist device, enable : 0x01, disable : 0x02*/
	co_get_bytearray_from_string(&argv[1], &enable, 1);
	/*Address type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND*/
	co_get_bytearray_from_string(&argv[2], &param.own_address_type, 1);

	if(enable == 0x01){		
		err = bt_conn_create_auto_le(&param);
		if(err){
			vOutputString("Auto connect failed (err = [%d])\r\n",err);
			return err;
		}
	}else if(enable == 0x02){
		err = bt_conn_create_auto_stop();
		if(err){
			vOutputString("Auto connect stop (err = [%d])\r\n",err);
			return err;
		}
	}

	return 0;
}

static int cmd_bt_le_whitelist_add(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	bt_addr_le_t waddr;
	int 		err;
	u8_t 		i=0;
	u8_t 		val[6];
	
	if(argc != 3){
		vOutputString("Number of Parameters is not correct (argc = [%d])\r\n",argc);
		return ;
	}

	err = bt_le_whitelist_clear();
	if(err){
		vOutputString("Clear white list device failed (err = [%d])\r\n",err);
	}

	co_get_bytearray_from_string(&argv[1], &waddr.type, 1);
	co_get_bytearray_from_string(&argv[2], val, 6);
	
	co_reverse_bytearray(val, waddr.a.val, 6);

	err = bt_le_whitelist_add(&waddr);
	if(err){
		vOutputString("Failed to add device to whitelist (err = [%d])\r\n",err);
	}
}

#endif 
static void cmd_start_scan_timeout(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	struct bt_le_scan_param scan_param;
	uint16_t time = 0;
    int err;
	u8_t addre_type;
	
    (void)err;
	
    co_get_bytearray_from_string(&argv[1], &scan_param.type, 1);
    
    co_get_bytearray_from_string(&argv[2], &scan_param.filter_dup, 1);
    
    co_get_uint16_from_string(&argv[3], &scan_param.interval);
    
    co_get_uint16_from_string(&argv[4], &scan_param.window);

 
    co_get_bytearray_from_string(&argv[5], (uint8_t *)&addre_type, 1);

	co_get_uint16_from_string(&argv[6], (uint16_t *)&time);
	
    err = bt_le_scan_start(&scan_param, device_found, addre_type);
   
    if(err){
        vOutputString("Failed to start scan (err %d) \r\n", err);
    }else{
        vOutputString("Start scan successfully \r\n");
    }

	k_sleep(time);
	bt_le_scan_stop();
	vOutputString("Scan stop \r\n");
}

static void cmd_pts_address_register(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	bt_addr_le_t addr;
	u8_t type;
	char le_addr[BT_ADDR_LE_STR_LEN];
	
	co_get_bytearray_from_string(&argv[1], &type, 1);
	co_get_bytearray_from_string(&argv[2], addr.a.val, 6);

	if(type == 0){
		addr.type = BT_ADDR_LE_PUBLIC;
	}else if(type == 1)
		addr.type = BT_ADDR_LE_RANDOM;

	memcpy(&pts_addr,&addr,sizeof(bt_addr_le_t));

	bt_addr_le_to_str(&pts_addr, le_addr, sizeof(le_addr));
	
	vOutputString("Pts address %s \r\n",le_addr);	
}


static void cmd_set_flag(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err = 0;
	u8_t flag;
	co_get_bytearray_from_string(&argv[1],&flag,1);
	event_flag	= flag;
	
	vOutputString("Event flag = [0x%x] \r\n",event_flag);
}

static void cmd_set_smp_flag(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	u8_t flag;
	
	co_get_bytearray_from_string(&argv[1],&flag,1);
	bt_set_smpflag((smp_test_id)flag);

	vOutputString("Smp flag = [0x%x] \r\n",flag);
}

static void cmd_clear_smp_flag(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	u8_t flag;
	
	co_get_bytearray_from_string(&argv[1],&flag,1);
	bt_clear_smpflag((smp_test_id)flag);

	vOutputString("Clear smp flag \r\n");
}

static int cmd_bondable(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	const char bondable;

	co_get_bytearray_from_string(&argv[1], &bondable, 1);

	if(bondable == 0x01)
		bt_set_bondable(true);
	else if(bondable == 0x00)
		bt_set_bondable(false);
	else
		vOutputString("Bondable status is unknow \r\n");

	return 0;
}


#endif /*CONFIG_BT_STACK_PTS */


static void cmd_stop_scan(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
    
	err = bt_le_scan_stop();
	if (err) {
		vOutputString("Stopping scanning failed (err %d)\r\n", err);
	} else {
		vOutputString("Scan successfully stopped \r\n");
	}
}
#endif //#if defined(CONFIG_BT_OBSERVER)


#if defined(CONFIG_BT_PERIPHERAL)
static void cmd_read_local_address(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int ret;
	bt_addr_le_t adv_addr;
	char le_addr[BT_ADDR_LE_STR_LEN];
	
	memset(&adv_addr,0,sizeof(bt_addr_le_t));
	ret = bt_get_local_address(&adv_addr);
	if(!ret){
		bt_addr_le_to_str(&adv_addr, le_addr, sizeof(le_addr));
		vOutputString("Local addr : %s\r\n",le_addr);

	}else{
		vOutputString("Failed to read address\r\n");
	}
}

static void cmd_start_advertise(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    struct bt_le_adv_param param;
	const struct bt_data *ad;
	size_t ad_len;
	int err = 0;
    uint8_t adv_type, tmp;
	
    #if defined(CONFIG_BT_STACK_PTS)
    struct bt_data pts_ad ;
	bt_addr_le_t pts; 
	struct bt_conn *conn;
	u8_t adder_type = 0;
	u8_t is_ad = 0;
	
	memset(pts.a.val,0,6);
	memset(&pts_ad,0,sizeof(struct bt_data));
    #endif

    #if defined(CONFIG_BT_STACK_PTS)
    if(argc != 4 && argc != 6){
    #else
    if(argc != 3 && argc != 5){
    #endif
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }

	#if defined(CONFIG_BT_STACK_PTS)
	switch(event_flag){
		case ad_type_service_uuid:
			pts_ad.type = BT_DATA_UUID128_ALL;
			pts_ad.data = service_uuid;
			pts_ad.data_len = sizeof(service_uuid);
			is_ad = 1;
		break;
		
		case ad_type_local_name:
			pts_ad.type = BT_DATA_NAME_COMPLETE;
			pts_ad.data = name;
			pts_ad.data_len = 13;
			is_ad = 1;
		break;

		case ad_type_flags:
			pts_ad.type = BT_DATA_FLAGS;
			pts_ad.data = discover_mode;
			pts_ad.data_len = sizeof(discover_mode);
			is_ad = 1;
		break;

        case ad_type_manu_data:
			pts_ad.type = BT_DATA_MANUFACTURER_DATA;
			pts_ad.data = data_manu;
			pts_ad.data_len = sizeof(data_manu);
			is_ad = 1;
		break;

		case ad_type_tx_power_level:
			pts_ad.type = BT_DATA_TX_POWER;
			pts_ad.data = tx_power;
			pts_ad.data_len = sizeof(tx_power);
			is_ad = 1;
		break;

        case ad_type_service_data:
			pts_ad.type = BT_DATA_SVC_DATA16;
			pts_ad.data = service_data;
			pts_ad.data_len = sizeof(service_data);
			is_ad = 1;
		break;

        case ad_type_appearance:
			pts_ad.type = BT_DATA_GAP_APPEARANCE;
			pts_ad.data = data_appearance;
			pts_ad.data_len = sizeof(data_appearance);
			is_ad = 1;
		break;
	
		default:
			break;
	}
	#endif
        
    param.id = selected_id;
    param.interval_min = BT_GAP_ADV_FAST_INT_MIN_2;
    param.interval_max = BT_GAP_ADV_FAST_INT_MAX_2;

    /*Get adv type, 0:adv_ind,  1:adv_scan_ind, 2:adv_nonconn_ind 3: adv_direct_ind*/
    co_get_bytearray_from_string(&argv[1], &adv_type, 1);
    
    if(adv_type == 0){
        param.options = (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_ONE_TIME);
    }else if(adv_type == 1){
        param.options = BT_LE_ADV_OPT_USE_NAME;
    }else if(adv_type == 2){
        param.options = 0;
    #if defined(CONFIG_BT_STACK_PTS)
	}else if(adv_type == 3){
		param.options = (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME);
    #endif
    }else{
        vOutputString("Arg1 is invalid\r\n");
        return;
    }

    /*Get mode, 0:General discoverable,  1:non discoverable, 2:limit discoverable*/
    co_get_bytearray_from_string(&argv[2], &tmp, 1);

    if(tmp == 0 || tmp == 1 || tmp == 2){
		
        if(tmp == 1)
            ad_discov[0].data = 0;
    #if defined(CONFIG_BT_STACK_PTS)
		else if(tmp == 2)
			ad_discov[0].data = &discover_mode[0];
			
		if(is_ad == 1){
		
			ad = &pts_ad;
			ad_len = 1;
			
		}else{
    #endif
        ad = ad_discov;
        ad_len = ARRAY_SIZE(ad_discov);

	 #if defined(CONFIG_BT_STACK_PTS)
		}
	 #endif	
		
    }else{
        vOutputString("Arg2 is invalid\r\n");
        return;
    }

    #if defined(CONFIG_BT_STACK_PTS)
    /*upper address type, 0:non-resolvable private address,1:resolvable private address,2:public address*/
    //co_get_bytearray_from_string(&argv[3], (u8_t *)&param.addr_type, 1);
    co_get_bytearray_from_string(&argv[3], (u8_t *)&adder_type, 1);

	if(adder_type == 0)
		param.addr_type = BT_ADDR_TYPE_NON_RPA;
	else if(adder_type == 1)
		param.addr_type = BT_ADDR_TYPE_RPA;
	else if(adder_type == 2)
		param.addr_type = BT_ADDR_LE_PUBLIC;
	else
		vOutputString("Invaild address type\r\n");
	
    if(argc == 6){
        co_get_uint16_from_string(&argv[4], &param.interval_min);
        co_get_uint16_from_string(&argv[5], &param.interval_max);
    }  

	if(adv_type == 3){	
		param.interval_min = 0;
		param.interval_max = 0;
	}
	
    #else
	
    if(argc == 5){
        co_get_uint16_from_string(&argv[3], &param.interval_min);
        co_get_uint16_from_string(&argv[4], &param.interval_max);
    }
	
    #endif//CONFIG_BT_STACK_PTS
    
    if(adv_type == 1){
		
        err = bt_le_adv_start(&param, ad, ad_len, &ad_discov[0], 1);
		
    #if defined(CONFIG_BT_STACK_PTS)
	
    }else if(adv_type == 3){
    
    	pts.type = BT_ADDR_LE_PUBLIC;
		
		memcpy(pts.a.val,pts_address,6);
		
		conn = bt_conn_create_slave_le(&pts,&param);
	
		if(!conn){
        	err = 1; 
		}
	    else{
	        bt_conn_unref(conn);
	    }
		
    #endif
	
    }else{
        err = bt_le_adv_start(&param, ad, ad_len, NULL, 0);
    }
 
    if(err){
        vOutputString("Failed to start advertising\r\n");
    }else{
        vOutputString("Advertising started\r\n");
    }

    #if defined(CONFIG_BT_STACK_PTS)
	if(tmp == 2){
		
		k_sleep(LIM_ADV_TIME);
		
		bt_le_adv_stop();
		
		vOutputString("Adv timeout \r\n");
	}
    #endif
	
}

static void cmd_stop_advertise(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if(bt_le_adv_stop()){
        vOutputString("Failed to stop advertising\r\n");	
    }else{ 
        vOutputString("Advertising stopped\r\n");
    }
}


#endif //#if defined(CONFIG_BT_PERIPHERAL)

#if defined(CONFIG_BT_CONN)
#if defined(CONFIG_BT_CENTRAL)
static void cmd_connect_le(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    bt_addr_le_t addr;
    struct bt_conn *conn;
    u8_t  addr_val[6];
	s8_t  type = -1;

	struct bt_le_conn_param param = {
		.interval_min =  BT_GAP_INIT_CONN_INT_MIN,
		.interval_max =  BT_GAP_INIT_CONN_INT_MAX,
		.latency = 0,
		.timeout = 400,
#if defined(CONFIG_BT_STACK_PTS)
		.own_address_type = BT_ADDR_LE_PUBLIC_ID,
#endif
	};

    if(argc != 3){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
       
    co_get_bytearray_from_string(&argv[1], (u8_t *)&type, 1);

	#if defined(CONFIG_BT_STACK_PTS)
	/*Get addr type,0:ADDR_RAND, 1:ADDR_RPA_OR_PUBLIC,2:ADDR_PUBLIC, 3:ADDR_RPA_OR_RAND*/
	if(type == 0)
		addr.type = 1; /*ADDR_RAND*/
	else if(type == 1)
		addr.type = 2; /*ADDR_RPA_OR_PUBLIC*/
	else if(type == 2)
		addr.type = 0; /*ADDR_PUBLIC*/
	else if(type == 3)
		addr.type = 3; /*ADDR_RPA_OR_RAND*/
	else 
		vOutputString("adderss type is unknow [0x%x]\r\n",type);
	#else 
	/*Get addr type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND*/
		addr.type = type;
	#endif

    co_get_bytearray_from_string(&argv[2], addr_val, 6);

    co_reverse_bytearray(addr_val, addr.a.val, 6);
    
    conn = bt_conn_create_le(&addr, /*BT_LE_CONN_PARAM_DEFAULT*/&param);

    if(!conn){
        vOutputString("Connection failed\r\n");
    }else{
        vOutputString("Connection pending\r\n");
    }
}

#endif //#if defined(CONFIG_BT_CENTRAL)

static void cmd_disconnect(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    bt_addr_le_t addr;
    u8_t  addr_val[6];
    struct bt_conn *conn;
	s8_t  type = -1;
	
    if(argc != 3){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    co_get_bytearray_from_string(&argv[1], (u8_t *)&type, 1);
    co_get_bytearray_from_string(&argv[2], addr_val, 6);
    co_reverse_bytearray(addr_val, addr.a.val, 6);

	#if defined(CONFIG_BT_STACK_PTS)
	/*Get addr type,0:ADDR_RAND, 1:ADDR_RPA_OR_PUBLIC,2:ADDR_PUBLIC, 3:ADDR_RPA_OR_RAND*/
	if(type == 0)
		addr.type = 1; /*ADDR_RAND*/
	else if(type == 1)
		addr.type = 2; /*ADDR_RPA_OR_PUBLIC*/
	else if(type == 2)
		addr.type = 0; /*ADDR_PUBLIC*/
	else if(type == 3)
		addr.type = 3; /*ADDR_RPA_OR_RAND*/
	else 
		vOutputString("adderss type is unknow\r\n");
    #else 
	/*Get addr type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND*/
		addr.type = type;
	#endif

    conn = bt_conn_lookup_addr_le(selected_id, &addr);

    if(!conn){
        vOutputString("Not connected\r\n");
        return;
    }

    if(bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN)){
        vOutputString("Disconnection failed\r\n");
    }else{
        vOutputString("Disconnect successfully\r\n");
		#if defined(CONFIG_BT_STACK_PTS)
		//bt_conn_unref(conn);
		//return;
		#endif
    }
    bt_conn_unref(conn);
}

static void cmd_select_conn(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    bt_addr_le_t addr;
    struct bt_conn *conn;
    u8_t  addr_val[6];

    if(argc != 3){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }

    /*Get addr type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND*/
    co_get_bytearray_from_string(&argv[1], &addr.type, 1);

    co_get_bytearray_from_string(&argv[2], addr_val, 6);

    co_reverse_bytearray(addr_val, addr.a.val, 6);
    
    conn = bt_conn_lookup_addr_le(selected_id, &addr);
           
    if(!conn){
        vOutputString("No matching connection found\r\n");
        return;
    }

    if(default_conn){
        bt_conn_unref(default_conn);
    }

    default_conn = conn;
}

static void cmd_unpair(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    bt_addr_le_t addr;
    u8_t  addr_val[6];
    int err;

    if(argc != 3){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }

    /*Get addr type, 0:ADDR_PUBLIC, 1:ADDR_RAND, 2:ADDR_RPA_OR_PUBLIC, 3:ADDR_RPA_OR_RAND*/
    co_get_bytearray_from_string(&argv[1], &addr.type, 1);
    
    co_get_bytearray_from_string(&argv[2], addr_val, 6);

    co_reverse_bytearray(addr_val, addr.a.val, 6);
        
    err = bt_unpair(selected_id, &addr);

    if(err){
        vOutputString("Failed to unpair\r\n");
    }else{
        vOutputString("Unpair successfully\r\n");
    }
}

static void cmd_conn_update(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	struct bt_le_conn_param param;
	int err;

    if(argc != 5){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    co_get_uint16_from_string(&argv[1], &param.interval_min);
    co_get_uint16_from_string(&argv[2], &param.interval_max);
    co_get_uint16_from_string(&argv[3], &param.latency);
    co_get_uint16_from_string(&argv[4], &param.timeout);	
#if defined(CONFIG_BT_STACK_PTS)
	if(event_flag == dir_connect_req)
		err = bt_conn_le_param_update(default_conn, &param);
	else
		err = pts_bt_conn_le_param_update(default_conn, &param);
#else
	err = bt_conn_le_param_update(default_conn, &param);
#endif
	if (err) {
		vOutputString("conn update failed (err %d)\r\n", err);
	} else {
		vOutputString("conn update initiated\r\n");
	}
}
#endif //#if defined(CONFIG_BT_CONN)

#if defined(CONFIG_BT_SMP)
static void cmd_security(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int err;
   	u8_t sec_level = /*BT_SECURITY_FIPS*/BT_SECURITY_L4;

    if(!default_conn){
        vOutputString("Please firstly choose the connection using cmd_select_conn\r\n");
        return;
    }

    if(argc == 2)
        co_get_bytearray_from_string(&argv[1], &sec_level, 1);
    
    err = bt_conn_set_security(default_conn, sec_level);

    if(err){
        vOutputString("Failed to start security, (err %d) \r\n", err);
    }else{
        vOutputString("Start security successfully\r\n");
    }
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));    

    vOutputString("passkey_str is: %06u\r\n", passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	vOutputString("Confirm passkey for %s: %06u\r\n", addr, passkey);
}

static void auth_passkey_entry(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	vOutputString("Enter passkey for %s\r\n", addr);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	
	vOutputString("Pairing cancelled: %s\r\n", addr);
}

static void auth_pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	vOutputString("Confirm pairing for %s\r\n", addr);
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	#if defined(CONFIG_BT_STACK_PTS)
	int err = bt_le_whitelist_clear();
	if(err){
		vOutputString("Clear white list device failed (err = [%d])\r\n",err);
	}
	bt_le_whitelist_add(bt_conn_get_dst(conn));
	#endif
	vOutputString("%s with %s\r\n", bonded ? "Bonded" : "Paired", addr);
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	vOutputString("Pairing failed with %s\r\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = auth_passkey_entry,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_failed = auth_pairing_failed,
	.pairing_complete = auth_pairing_complete,
};

static void cmd_auth(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int err;

    err = bt_conn_auth_cb_register(&auth_cb_display);

    if(err){
        vOutputString("Auth callback has already been registered\r\n");
    }else{
        vOutputString("Register auth callback successfully\r\n");
    }
}

static void cmd_auth_cancel(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	struct bt_conn *conn;
    
	if (default_conn) {
		conn = default_conn;
	}else {
		conn = NULL;
	}

	if (!conn) {
        vOutputString("Not connected\r\n");
		return;
	}

	bt_conn_auth_cancel(conn);
}

static void cmd_auth_passkey_confirm(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    
	if (!default_conn) {
        vOutputString("Not connected\r\n");
		return;
	}

	bt_conn_auth_passkey_confirm(default_conn);
}

static void cmd_auth_pairing_confirm(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
   
	if (!default_conn) {
        vOutputString("Not connected\r\n");
		return;
	}

	bt_conn_auth_pairing_confirm(default_conn);
}

static void cmd_auth_passkey(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	uint32_t passkey;

    if(argc != 2){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
	if (!default_conn) {
        vOutputString("Not connected\r\n");
		return;
	}

    passkey = atoi(argv[1]);
	if (passkey > PASSKEY_MAX) {
        vOutputString("Passkey should be between 0-999999\r\n");
		return;
	}

	bt_conn_auth_passkey_entry(default_conn, passkey);
}

#if defined(CONFIG_BT_STACK_PTS)
static void cmd_set_mitm(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	u8_t enable = 0;

	co_get_bytearray_from_string(&argv[1],&enable,1);

	if(enable == 0x01)
		bt_set_mitm(true);
	else if(enable == 0x00)
		bt_set_mitm(false);
	else
		vOutputString("Inviad parameter\r\n");
}

#endif

#endif //#if defined(CONFIG_BT_SMP)

#if defined(CONFIG_BT_GATT_CLIENT)
static void exchange_func(struct bt_conn *conn, u8_t err,
			  struct bt_gatt_exchange_params *params)
{
	vOutputString("Exchange %s\r\n", err == 0U ? "successful" : "failed");
}

static struct bt_gatt_exchange_params exchange_params;

static void cmd_exchange_mtu(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
    
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

	exchange_params.func = exchange_func;

	err = bt_gatt_exchange_mtu(default_conn, &exchange_params);
	if (err) {
		vOutputString("Exchange failed (err %d)\r\n", err);
	} else {
		vOutputString("Exchange pending\r\n");
	}
}

static struct bt_gatt_discover_params discover_params;
static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
#if defined(CONFIG_BT_STACK_PTS)
static struct bt_uuid_128 uuid_128 = BT_UUID_INIT_128(0);
#endif 

static void print_chrc_props(u8_t properties)
{
	vOutputString("Properties: ");

	if (properties & BT_GATT_CHRC_BROADCAST) {
		vOutputString("[bcast]\r\n");
	}

	if (properties & BT_GATT_CHRC_READ) {
		vOutputString("[read]\r\n");
	}

	if (properties & BT_GATT_CHRC_WRITE) {
		vOutputString("[write]\r\n");
	}

	if (properties & BT_GATT_CHRC_WRITE_WITHOUT_RESP) {
		vOutputString("[write w/w rsp]\r\n");
	}

	if (properties & BT_GATT_CHRC_NOTIFY) {
		vOutputString("[notify]\r\n");
	}

	if (properties & BT_GATT_CHRC_INDICATE) {
		vOutputString("[indicate]");
	}

	if (properties & BT_GATT_CHRC_AUTH) {
		vOutputString("[auth]\r\n");
	}

	if (properties & BT_GATT_CHRC_EXT_PROP) {
		vOutputString("[ext prop]\r\n");
	}
}

static u8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *params)
{
	struct bt_gatt_service_val *gatt_service;
	struct bt_gatt_chrc *gatt_chrc;
	struct bt_gatt_include *gatt_include;
	char str[37];

	if (!attr) {
		vOutputString( "Discover complete\r\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	switch (params->type) {
	case BT_GATT_DISCOVER_SECONDARY:
	case BT_GATT_DISCOVER_PRIMARY:
		gatt_service = attr->user_data;
		bt_uuid_to_str(gatt_service->uuid, str, sizeof(str));
		vOutputString("Service %s found: start handle %x, end_handle %x\r\n", str, attr->handle, gatt_service->end_handle);
		break;
	case BT_GATT_DISCOVER_CHARACTERISTIC:
		gatt_chrc = attr->user_data;
		bt_uuid_to_str(gatt_chrc->uuid, str, sizeof(str));
		vOutputString("Characteristic %s found: attr->handle %x  chrc->handle %x \r\n", str, attr->handle,gatt_chrc->value_handle);
		print_chrc_props(gatt_chrc->properties);
		break;
	case BT_GATT_DISCOVER_INCLUDE:
		gatt_include = attr->user_data;
		bt_uuid_to_str(gatt_include->uuid, str, sizeof(str));
		vOutputString("Include %s found: handle %x, start %x, end %x\r\n", str, attr->handle,
			    gatt_include->start_handle, gatt_include->end_handle);
		break;
	default:
		bt_uuid_to_str(attr->uuid, str, sizeof(str));
		vOutputString("Descriptor %s found: handle %x\r\n", str, attr->handle);
		break;
	}

	return BT_GATT_ITER_CONTINUE;
}

#if defined(CONFIG_BT_STACK_PTS)
static void cmd_discover_uuid_128(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
    u8_t disc_type;
	u8_t val[16];
	u8_t i=0;

	(void)memset(val,0x0,16);
	
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}
	
	discover_params.func = discover_func;
	discover_params.start_handle = 0x0001;
	discover_params.end_handle = 0xffff;

    co_get_bytearray_from_string(&argv[1], &disc_type, 1);
    if(disc_type == 0){
        discover_params.type = BT_GATT_DISCOVER_PRIMARY;
    }else if(disc_type == 1){
        discover_params.type = BT_GATT_DISCOVER_SECONDARY;
    }else if(disc_type == 2){
        discover_params.type = BT_GATT_DISCOVER_INCLUDE;
    }else if(disc_type == 3){
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    }else if(disc_type == 4){
        discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
    }else{
        vOutputString("Invalid discovery type\r\n");
        return;
    }
	co_get_bytearray_from_string(&argv[2], val, 16);

	co_reverse_bytearray(val, uuid_128.val, 16);
		
	/*Set array value to 0 */
	(void)memset(val, 0x0, 16);
	
	if(!memcmp(uuid_128.val, val, 16))	
		discover_params.uuid = NULL;
	else
    	discover_params.uuid = &uuid_128.uuid;
       
    co_get_uint16_from_string(&argv[3], &discover_params.start_handle);
    co_get_uint16_from_string(&argv[4], &discover_params.end_handle);

	err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
		vOutputString("Discover failed (err %d)\r\n", err);
	} else {
		vOutputString("Discover pending\r\n");
	}
}
#endif

static void cmd_discover(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
    u8_t disc_type;

    if(argc != 5){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

	discover_params.func = discover_func;
	discover_params.start_handle = 0x0001;
	discover_params.end_handle = 0xffff;

    co_get_bytearray_from_string(&argv[1], &disc_type, 1);
    if(disc_type == 0){
        discover_params.type = BT_GATT_DISCOVER_PRIMARY;
    }else if(disc_type == 1){
        discover_params.type = BT_GATT_DISCOVER_SECONDARY;
    }else if(disc_type == 2){
        discover_params.type = BT_GATT_DISCOVER_INCLUDE;
    }else if(disc_type == 3){
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    }else if(disc_type == 4){
        discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
    }else{
        vOutputString("Invalid discovery type\r\n");
        return;
    }
    co_get_uint16_from_string(&argv[2], &uuid.val);
	
    if(uuid.val)
        discover_params.uuid = &uuid.uuid;
    else
        discover_params.uuid = NULL;

    co_get_uint16_from_string(&argv[3], &discover_params.start_handle);
    co_get_uint16_from_string(&argv[4], &discover_params.end_handle);

	err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
		vOutputString("Discover failed (err %d)\r\n", err);
	} else {
		vOutputString("Discover pending\r\n");
	}
}

static struct bt_gatt_read_params read_params;

static u8_t read_func(struct bt_conn *conn, u8_t err, struct bt_gatt_read_params *params, const void *data, u16_t length)
{
	vOutputString("Read complete: err %u length %u \r\n", err, length);

	char str[22]; 
	u8_t *buf = (u8_t *)data;

	memset(str,0,15);
	
	if(length > 0 && length <= sizeof(str)){
		memcpy(str,buf,length);
		vOutputString("device name : %s \r\n",str);
		
		#if defined(CONFIG_BT_STACK_PTS)
		u8_t i = 0;
		for(i=0;i<length;i++)
			vOutputString("data[%d] = [0x%x]\r\n", i, buf[i]);	
		
		#endif
	}

	if (!data) {
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_CONTINUE;
}

#if defined(CONFIG_BT_STACK_PTS)
static void cmd_read_uuid_128(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
	u8_t val[16];
	struct bt_uuid_128 uuid = BT_UUID_INIT_128(0);

	(void)memset(val, 0, 16);
	
    if(argc != 4){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}
	
    read_params.func = read_func;
	read_params.handle_count = 0;
	read_params.by_uuid.start_handle = 0x0001;
	read_params.by_uuid.end_handle = 0xffff;
	
	co_get_bytearray_from_string(&argv[1], val, 16);
	co_reverse_bytearray(val, uuid.val, 16);
	//(void)memcpy(uuid.val, val, 16)

	(void)memset(val, 0, 16);
	
	if(!memcmp(uuid.val, val, 16))
		read_params.by_uuid.uuid = NULL;
	else
		read_params.by_uuid.uuid = &uuid.uuid;
	
	co_get_uint16_from_string(&argv[2], &read_params.by_uuid.start_handle);
	co_get_uint16_from_string(&argv[3], &read_params.by_uuid.end_handle);
	
	err = bt_gatt_read(default_conn, &read_params);
	if (err) {
		vOutputString("Read failed (err %d)\r\n", err);
	} else {
		vOutputString("Read pending\r\n");
	}

}

static void cmd_read_uuid(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
	struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
	
    if(argc != 4){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}
	
    read_params.func = read_func;
	read_params.handle_count = 0;
	read_params.by_uuid.start_handle = 0x0001;
	read_params.by_uuid.end_handle = 0xffff;
	
	co_get_uint16_from_string(&argv[1], &uuid.val);
	if(uuid.val)
		read_params.by_uuid.uuid = &uuid.uuid;
	
	co_get_uint16_from_string(&argv[2], &read_params.by_uuid.start_handle);
	co_get_uint16_from_string(&argv[3], &read_params.by_uuid.end_handle);
	
	err = bt_gatt_read(default_conn, &read_params);
	if (err) {
		vOutputString("Read failed (err %d)\r\n", err);
	} else {
		vOutputString("Read pending\r\n");
	}

}

static int cmd_mread(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv) 
{
	u16_t h[8];
	int i, err;

	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return -1;
	}

	if((argc-1) > ARRAY_SIZE(h)){	
		vOutputString("Enter max %lu handle items to read\r\n",ARRAY_SIZE(h));
		return -1;
	}

	for (i = 0; i < argc - 1; i++) {
		co_get_uint16_from_string(&argv[i + 1],&h[i]);
	}	

	read_params.func = read_func;
	read_params.handle_count = i;
	read_params.handles = h; /* not used in read func */

	vOutputString("i = [%d]\r\n",i);
	
	err = bt_gatt_read(default_conn, &read_params);
	if (err) {
		vOutputString("Read failed (err %d)\r\n", err);
	} else {
		vOutputString("Read pending\r\n");
	}
	
	return err;
}


#endif

static void cmd_read(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;

    if(argc != 3){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

    co_get_uint16_from_string(&argv[1], &read_params.single.handle);
    co_get_uint16_from_string(&argv[2], &read_params.single.offset);

    read_params.func = read_func;
	read_params.handle_count = 1;

	err = bt_gatt_read(default_conn, &read_params);
	if (err) {
		vOutputString("Read failed (err %d)\r\n", err);
	} else {
		vOutputString("Read pending\r\n");
	}
}

static struct bt_gatt_write_params write_params;
static u8_t gatt_write_buf[CHAR_SIZE_MAX];

static void write_func(struct bt_conn *conn, u8_t err,
		       struct bt_gatt_write_params *params)
{
	vOutputString("Write complete: err %u \r\n", err);

	(void)memset(&write_params, 0, sizeof(write_params));
}

#if defined(CONFIG_BT_STACK_PTS)
static void cmd_prepare_write(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
    uint16_t data_len;

    if(argc != 5){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

	if (write_params.func) {
		vOutputString("Write ongoing\r\n");
		return;
	}

    co_get_uint16_from_string(&argv[1], &write_params.handle);
    co_get_uint16_from_string(&argv[2], &write_params.offset);
    co_get_uint16_from_string(&argv[3], &write_params.length);
    data_len = write_params.length > sizeof(gatt_write_buf)? (sizeof(gatt_write_buf)):(write_params.length);
    co_get_bytearray_from_string(&argv[4], gatt_write_buf, data_len);
    
	write_params.data = gatt_write_buf;
	write_params.length = data_len;
	write_params.func = write_func;
	
 	err = bt_gatt_prepare_write(default_conn, &write_params);	

	if (err) {
		vOutputString("Prepare write failed (err %d)\r\n", err);
	} else {
		vOutputString("Prepare write pending\r\n");
	}

}
#endif

static void cmd_write(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
    uint16_t data_len;

    if(argc != 5){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

	if (write_params.func) {
		vOutputString("Write ongoing\r\n");
		return;
	}

    co_get_uint16_from_string(&argv[1], &write_params.handle);
    co_get_uint16_from_string(&argv[2], &write_params.offset);
    co_get_uint16_from_string(&argv[3], &write_params.length);
    data_len = write_params.length > sizeof(gatt_write_buf)? (sizeof(gatt_write_buf)):(write_params.length);
    co_get_bytearray_from_string(&argv[4], gatt_write_buf, data_len);
    
	write_params.data = gatt_write_buf;
	write_params.length = data_len;
	write_params.func = write_func;
	
	err = bt_gatt_write(default_conn, &write_params);

	if (err) {
		vOutputString("Write failed (err %d)\r\n", err);
	} else {
		vOutputString("Write pending\r\n");
	}
}

static void cmd_write_without_rsp(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	u16_t handle;
	int err;
	u16_t len;
	bool sign;

    if(argc != 5){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
   
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

    co_get_bytearray_from_string(&argv[1], (uint8_t *)&sign, 1);
    co_get_uint16_from_string(&argv[2], &handle);
	co_get_uint16_from_string(&argv[3], &len);
    len = len > sizeof(gatt_write_buf)? (sizeof(gatt_write_buf)):(len);
	co_get_bytearray_from_string(&argv[4], gatt_write_buf, len);
	
	err = bt_gatt_write_without_response(default_conn, handle, gatt_write_buf, len, sign);

	vOutputString("Write Complete (err %d)\r\n", err);
}

static struct bt_gatt_subscribe_params subscribe_params;

static u8_t notify_func(struct bt_conn *conn,
			struct bt_gatt_subscribe_params *params,
			const void *data, u16_t length)
{
	if (!params->value) {
		vOutputString("Unsubscribed\r\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	vOutputString("Notification: data %p length %u\r\n", data, length);

	return BT_GATT_ITER_CONTINUE;
}

static void cmd_subscribe(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;

    (void)err;

    if(argc != 4){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
	if (subscribe_params.value_handle) {
		vOutputString( "Cannot subscribe: subscription to %x already exists\r\n", subscribe_params.value_handle);
		return;
	}

	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

    co_get_uint16_from_string(&argv[1], &subscribe_params.ccc_handle);
    co_get_uint16_from_string(&argv[2], &subscribe_params.value_handle);
    co_get_uint16_from_string(&argv[3], &subscribe_params.value);
	subscribe_params.notify = notify_func;

	err = bt_gatt_subscribe(default_conn, &subscribe_params);
	if (err) {
		vOutputString("Subscribe failed (err %d)\r\n", err);
	} else {
		vOutputString("Subscribed\r\n");
	}

}

static void cmd_unsubscribe(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;

    (void)err;
    
	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

	if (!subscribe_params.value_handle) {
		vOutputString("No subscription found\r\n");
		return;
	}

	err = bt_gatt_unsubscribe(default_conn, &subscribe_params);
	if (err) {
		vOutputString("Unsubscribe failed (err %d)\r\n", err);
	} else {
		vOutputString("Unsubscribe success\r\n");
	}
}
#endif /* CONFIG_BT_GATT_CLIENT */

static void cmd_set_data_len(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	u16_t tx_octets;
	u16_t tx_time;
	int err;

	if(argc != 3){
	    vOutputString("Number of Parameters is not correct\r\n");
	    return;
	}

	if (!default_conn) {
		vOutputString("Not connected\r\n");
		return;
	}

	co_get_uint16_from_string(&argv[1], &tx_octets);
	co_get_uint16_from_string(&argv[2], &tx_time);

	err = bt_le_set_data_len(default_conn, tx_octets, tx_time);
	if (err) {
		vOutputString("cmd_set_data_len, LE Set Data Length (err %d)\r\n", err);
	}
	else
	{
		vOutputString("cmd_set_data_len, LE Set Data Length success\r\n");
	}
}

#if defined(CONFIG_SET_TX_PWR)
enum {
    RFC_AUTO = 0,
    RFC_WLAN_11B,
    RFC_WLAN_11G,
    RFC_WLAN_11N,
    RFC_BT_BLE,
    RFC_BT_BLE_6,
    RFC_BT_BLE_12,
    RFC_BT_BLE_20
};

static void cmd_set_tx_pwr(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    u8_t mode;
    int err;

    if(argc != 2){
	    vOutputString("Number of Parameters is not correct\r\n");
	    return;
	}

    co_get_bytearray_from_string(&argv[1],&mode,1);

    if(mode < RFC_BT_BLE_6 || mode > RFC_BT_BLE_20){
        vOutputString("cmd_set_tx_pwr, invalid value, value shall be in [%d - %d]\r\n", RFC_BT_BLE_6, RFC_BT_BLE_20);
        return;
    }
        
    err = bt_set_tx_pwr(mode);

    if(err){
		vOutputString("cmd_set_tx_pwr, Fail to set tx power (err %d)\r\n", err);
	}
	else{
		vOutputString("cmd_set_tx_pwr, Set tx power successfully\r\n");
	}
    
}
#endif


int blestack_cli_register(void)
{
    #if defined(CONFIG_BT_STACK_PTS)
    memcpy(&pts_addr.a, BT_ADDR_NONE, sizeof(pts_addr.a));
    #endif
    #if defined(BL602)
    //aos_cli_register_commands(btStackCmdSet, sizeof(btStackCmdSet)/sizeof(btStackCmdSet[0]));
    #elif defined(BL70X)
    const struct cli_command *cmdSet = btStackCmdSet;
    while(cmdSet->pcCommand){        
        FreeRTOS_CLIRegisterCommand(cmdSet);
        cmdSet++;
    }
    #endif

    return 0;
}
