#include <stdlib.h>
#include "conn.h"
#include "gatt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cli.h"
#include "mesh_cli_cmds.h"
#include "include/mesh.h"
#include "errno.h"

#include "mesh.h"
#include "net.h"
#include "transport.h"
#include "foundation.h"
#include "mesh_settings.h"
#include "adv.h"
#include "beacon.h"
#include "hci_core.h"
#if defined(CONFIG_BT_MESH_MODEL_GEN_SRV)
#include "gen_srv.h"
#endif

#if defined(CONFIG_BT_SETTINGS)
#include "easyflash.h"
#endif

#define CUR_FAULTS_MAX 4

bool blemesh_inited = false;
#if defined(CONFIG_BT_MESH_LOW_POWER)
//below value is for Tmall Genie
u8_t dev_uuid[16] = {0xA8,0x01,0x71,0x5e,0x1c,0x00,0x00,0xe4,0x46,0x46,0x63,0xa7,0xf8,0x02,0x00,0x00};
u8_t auth_value[16] = {0x78,0x8A,0xE3,0xEE,0x0F,0x2A,0x7E,0xFA,0xD3,0x67,0x35,0x81,0x41,0xFE,0x1B,0x06};
#else
u8_t dev_uuid[16] = {0xA8,0x01,0x71,0xe0,0x1a,0x00,0x00,0x0f,0x7e,0x35,0x63,0xa7,0xf8,0x02,0x00,0x00};
u8_t auth_value[16] = {0x7f,0x80,0x1a,0xf4,0xa0,0x8c,0x50,0x39,0xae,0x7d,0x7b,0x44,0xa0,0x92,0xd9,0xc2};
#endif

static bt_mesh_input_action_t input_act;
static u8_t input_size;
static u8_t cur_faults[CUR_FAULTS_MAX];
static u8_t reg_faults[CUR_FAULTS_MAX * 2];

static struct {
	u16_t local;
	u16_t dst;
	u16_t net_idx;
	u16_t app_idx;
} net = {
	.local = BT_MESH_ADDR_UNASSIGNED,
	.dst = BT_MESH_ADDR_UNASSIGNED,
};

#if defined(BL602)
#define vOutputString(...)  printf(__VA_ARGS__)
#else
#define vOutputString(...)  bl_print(SYSTEM_UART_ID, PRINT_MODULE_CLI, __VA_ARGS__)
#endif

static void prov_reset(void);
static void link_open(bt_mesh_prov_bearer_t bearer);
static void link_close(bt_mesh_prov_bearer_t bearer);
static int output_number(bt_mesh_output_action_t action, u32_t number);
static int output_string(const char *str);
static void prov_input_complete(void);
static void prov_complete(u16_t net_idx, u16_t addr);
static void prov_reset(void);
static int input(bt_mesh_input_action_t act, u8_t size);
static void get_faults(u8_t *faults, u8_t faults_size, u8_t *dst, u8_t *count);
static void health_current_status(struct bt_mesh_health_cli *cli, u16_t addr,
				  u8_t test_id, u16_t cid, u8_t *faults, size_t fault_count);
static int fault_get_cur(struct bt_mesh_model *model, u8_t *test_id,
			 u16_t *company_id, u8_t *faults, u8_t *fault_count);
static int fault_get_reg(struct bt_mesh_model *model, u16_t cid,
			 u8_t *test_id, u8_t *faults, u8_t *fault_count);
static int fault_clear(struct bt_mesh_model *model, uint16_t cid);
static int fault_test(struct bt_mesh_model *model, uint8_t test_id, uint16_t cid);
static void attn_on(struct bt_mesh_model *model);
static void attn_off(struct bt_mesh_model *model);

static void cmd_mesh_init(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_set_dev_uuid(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_input_num(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_input_str(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_pb(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_reset(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_net_send(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_seg_send(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_rpl_clr(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_ivu_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_iv_update(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void cmd_fault_set(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);

#if defined(CONFIG_BT_MESH_LOW_POWER)
static void cmd_lpn_set(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif

static struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
    .oob_info = 0,
	.link_open = link_open,
	.link_close = link_close,
	.complete = prov_complete,
	.reset = prov_reset,
	.static_val = auth_value,
	.static_val_len = 16,
	.output_size = 6,
	.output_actions = (BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING),
	.output_number = output_number,
	.output_string = output_string,
	.input_size = 6,
	.input_actions = (BT_MESH_ENTER_NUMBER | BT_MESH_ENTER_STRING),
	.input = input,
	.input_complete = prov_input_complete,
};

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.fault_get_cur = fault_get_cur,
	.fault_get_reg = fault_get_reg,
	.fault_clear = fault_clear,
	.fault_test = fault_test,
	.attn_on = attn_on,
	.attn_off = attn_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

static struct bt_mesh_health_cli health_cli = {
	.current_status = health_current_status,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, CUR_FAULTS_MAX);

static struct bt_mesh_cfg_cli cfg_cli = {
};

static struct bt_mesh_cfg_srv cfg_srv = {
	.relay = BT_MESH_RELAY_ENABLED,
	.beacon = BT_MESH_BEACON_ENABLED,//BT_MESH_BEACON_DISABLED,
#if defined(CONFIG_BT_MESH_FRIEND)
	.frnd = BT_MESH_FRIEND_DISABLED,
#else
	.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BT_MESH_GATT_PROXY)
	.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
	.gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif

	.default_ttl = 7,

	/* 3 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(2, 20),
	.relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

#if defined(CONFIG_BT_MESH_MODEL_GEN_SRV)
struct bt_mesh_gen_onoff_srv onoff_srv = {
};
#endif

static struct bt_mesh_model sig_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL_HEALTH_CLI(&health_cli),
	#if defined(CONFIG_BT_MESH_MODEL_GEN_SRV)
	BT_MESH_MODEL_GEN_ONOFF(&onoff_srv),
	#endif
};

static struct bt_mesh_model vendor_models[] = {
	BT_MESH_MODEL_VND(0x003F, 0x002A, NULL, NULL, NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, sig_models, vendor_models),
};

static const struct bt_mesh_comp comp = {
	.cid = BL_COMP_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

#if defined(BL602)
const struct cli_command btMeshCmdSet[] STATIC_CLI_CMD_ATTRIBUTE = {
#else
const struct cli_command btMeshCmdSet[] = {
#endif
    {"cmd_mesh_init", "\r\ncmd_mesh_init:[Initialize]\r\n Parameter[Null]\r\n", cmd_mesh_init},
    {"cmd_set_dev_uuid", "\r\ncmd_input_num:[input number in provisionging procedure]\r\n\
     [Size:16 Octets, e.g.112233445566778899AA]\r\n", cmd_set_dev_uuid},
    {"cmd_pb", "\r\ncmd_pb:[Enable or disable provisioning]\r\n\
     [bear, 1:adv bear, 2:gatt bear]\r\n\
     [enable, 0:disable provisioning, 1:enable provisioning]\r\n", cmd_pb},
    
    {"cmd_reset", "\r\ncmd_reset:[Reset the state of the local mesh node]\r\n Parameter[Null]\r\n", cmd_reset},
    {"cmd_net_send", "\r\ncmd_net_send:[Send a network packet]\r\n Parameter[TTL CTL SRC DST]\r\n", cmd_net_send},
    {"cmd_seg_send", "\r\ncmd_seg_send:[Send a segmented message]\r\n Parameter[SRC DST]\r\n", cmd_seg_send},
    {"cmd_rpl_clr", "\r\ncmd_rpl_clr:[Clear replay protection list]\r\n Parameter[Null]\r\n", cmd_rpl_clr},
    {"cmd_ivu_test", "\r\ncmd_ivu_test:[Enable or disable iv update test mode]\r\n\
     [enable, 0:disable, 1:enable]\r\n", cmd_ivu_test},
    {"cmd_iv_update", "\r\ncmd_iv_update:[Enable or disable iv update procedure]\r\n\
     [enable, 0:disable, 1:enable by sending secure network beacons]\r\n", cmd_iv_update},
    {"cmd_fault_set", "\r\ncmd_fault_set:[Set current fault or registered fault values]\r\n\
     [type, 0:current fault, 1:registered fault]\r\n\
     [fault, fault array in hex format]\r\n", cmd_fault_set},
    #if defined(CONFIG_BT_MESH_LOW_POWER)
    {"cmd_lpn_set", "\r\ncmd_lpn_set:[Enable or disable low power node]\r\n\
     [enable, 0:disable lpn, 1:enable lpn]\r\n", cmd_lpn_set},
    #endif
    {"cmd_input_num", "\r\ncmd_input_num:[input number in provisionging procedure]\r\n\
     [Max Size:16 Octets, e.g.112233445566778899AA]\r\n", cmd_input_num},
     
    {"cmd_input_str", "\r\ncmd_input_str:[input Alphanumeric in provisionging procedure]\r\n\
     [Max Size:16 Characters, e.g.123ABC]\r\n", cmd_input_str},

    #if defined(BL70X)
    {NULL, NULL, "No handler / Invalid command", NULL}
    #endif
};

#if defined(CONFIG_BT_MESH_LOW_POWER)
static void cmd_lpn_set(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    static bool lpn_enabled;
    u8_t enable;
    int err;


    co_get_bytearray_from_string(&argv[1], &enable, 1);
    
    if(enable){
        if(lpn_enabled){
            vOutputString("LPN already enabled\r\n");
            return;
        }
        
        err = bt_mesh_lpn_set(true);
        if(err){
            vOutputString("Failed to enable LPN\r\n");
        }else{
            lpn_enabled = true;
            vOutputString("Enable LPN successfully\r\n");
        }     
    }else{
        if(!lpn_enabled){
            vOutputString("LPN already disabled\r\n");
            return;
        }

        err = bt_mesh_lpn_set(false);
        if(err){
            vOutputString("Failed to disable LPN\r\n");
        }else{
            lpn_enabled = false;
            vOutputString("Disable LPN successfully\r\n");
        }
    }
}

static void lpn_cb(u16_t friend_addr, bool established)
{
	if (established) {
		vOutputString("Friendship (as LPN) established to Friend 0x%04x\r\n", friend_addr);
	} else {
		vOutputString("Friendship (as LPN) lost with Friend 0x%04x\r\n", friend_addr);
	}
}
#endif

static void cmd_mesh_init(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int err;

    if(blemesh_inited){
        vOutputString("Has initialized \r\n");
        return;
    }

    err = bt_mesh_init(&prov, &comp);
    if(err){
        vOutputString("Failed to init \r\n");
        return;
    }
        
    blemesh_inited = true;
    vOutputString("Init successfully \r\n");

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        mesh_set();
        mesh_commit();
    }

    if (bt_mesh_is_provisioned()) {
		vOutputString("Mesh network restored from flash\r\n");
	} else {
		vOutputString("Use pb-adv or pb-gatt to enable advertising\r\n");
	}

#ifdef CONFIG_BT_MESH_PTS
    int i;
    
    // used for almost all test cases
    vOutputString("[PTS] TSPX_bd_addr_iut: ");
    for(i=0; i<6; i++) {
        vOutputString("%02X", bt_dev.id_addr[0].a.val[i]);
    }
    vOutputString("\r\n");
    
    // used for almost all test cases
    vOutputString("[PTS] TSPX_device_uuid: ");
    for(i=0; i<16; i++) {
        vOutputString("%02X", dev_uuid[i]);
    }
    vOutputString("\r\n");
    
    // used for test case MESH/NODE/RLY/BV-02-C
    vOutputString("[PTS] TSPX_maximum_network_message_cache_entries: %d\r\n", CONFIG_BT_MESH_MSG_CACHE_SIZE);
    
    // used for test case MESH/NODE/CFG/CFGR/BV-01-C
    vOutputString("[PTS] TSPX_iut_supports_relay: %s\r\n", cfg_srv.relay == BT_MESH_RELAY_ENABLED ? "TRUE" : "FALSE");
    
    // used for test case MESH/NODE/CFG/SL/BV-03-C
    vOutputString("[PTS] TSPX_vendor_model_id: %04X%04X\r\n", vendor_models[0].vnd.company, vendor_models[0].vnd.id);
#endif

#if IS_ENABLED(CONFIG_BT_MESH_LOW_POWER)
	bt_mesh_lpn_set_cb(lpn_cb);
#endif
}

static const char *bearer2str(bt_mesh_prov_bearer_t bearer)
{
	switch (bearer) {
	case BT_MESH_PROV_ADV:
		return "PB-ADV";
	case BT_MESH_PROV_GATT:
		return "PB-GATT";
	default:
		return "unknown";
	}
}

#if defined(CONFIG_BT_MESH_PROV)
static void cmd_pb(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int err;
    uint8_t bearer;
    uint8_t enable;

    if(argc != 3){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
    co_get_bytearray_from_string(&argv[1], &bearer, 1);
    co_get_bytearray_from_string(&argv[2], &enable, 1);
    
	if (enable) {
		err = bt_mesh_prov_enable(bearer);
		if (err) {
			vOutputString("Failed to enable %s (err %d)\r\n", bearer2str(bearer), err);
		} else {
			vOutputString("%s enabled\r\n", bearer2str(bearer));
		}
	} else {
		err = bt_mesh_prov_disable(bearer);
		if (err) {
			vOutputString("Failed to disable %s (err %d)\r\n",
				    bearer2str(bearer), err);
		} else {
			vOutputString("%s disabled\r\n", bearer2str(bearer));
		}
	}
}
#endif

static void link_open(bt_mesh_prov_bearer_t bearer)
{
	vOutputString("Provisioning link opened on %s\r\n", bearer2str(bearer));
}

static void link_close(bt_mesh_prov_bearer_t bearer)
{
	vOutputString("Provisioning link closed on %s\r\n", bearer2str(bearer));
}

static int output_number(bt_mesh_output_action_t action, u32_t number)
{
	vOutputString("OOB Number: %u\r\n", number);
	return 0;
}

static int output_string(const char *str)
{
	vOutputString("OOB String: %s\r\n", str);
	return 0;
}

static void prov_input_complete(void)
{
	vOutputString("Input complete\r\n");
}

static void prov_complete(u16_t net_idx, u16_t addr)
{
	vOutputString("Local node provisioned, net_idx 0x%04x address 0x%04x\r\n", net_idx, addr);
	net.net_idx = net_idx,
	net.local = addr;
	net.dst = addr;
}

static void prov_reset(void)
{
	vOutputString("The local node has been reset and needs reprovisioning\r\n");
}

static void cmd_set_dev_uuid(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if(argc != 2){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }

    vOutputString("device uuid is %s\r\n", argv[1]);
    co_get_bytearray_from_string(&argv[1], dev_uuid, 16);
}

static void cmd_input_num(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;
    uint32_t num;

    if(argc != 2){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
     
    num = strtoul(argv[1], NULL, 10);

	if (input_act != BT_MESH_ENTER_NUMBER) {
		vOutputString("A number hasn't been requested!\r\n");
		return;
	}

	if (strlen(argv[1]) < input_size) {
		vOutputString("Too short input (%u digits required)\r\n", input_size);
		return;
	}

	err = bt_mesh_input_number(num);
	if (err) {
		vOutputString("Numeric input failed (err %d)\r\n", err);
		return;
	}

	input_act = BT_MESH_NO_INPUT;
}

static void cmd_input_str(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;

    if(argc != 2){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }

	if (input_act != BT_MESH_ENTER_STRING) {
		vOutputString("A string hasn't been requested!\r\n");
		return;
	}

	if (strlen(argv[1]) < input_size) {
		vOutputString("Too short input (%u characters required)\r\n", input_size);
		return;
	}

	err = bt_mesh_input_string(argv[1]);
	if (err) {
		vOutputString("String input failed (err %d)\r\n", err);
		return;
	}

	input_act = BT_MESH_NO_INPUT;
}

static int input(bt_mesh_input_action_t act, u8_t size)
{
	switch (act) {
	case BT_MESH_ENTER_NUMBER:
		vOutputString("Enter a number (max %u digits) with: input-num <num>\r\n", size);
		break;
	case BT_MESH_ENTER_STRING:
		vOutputString("Enter a string (max %u chars) with: input-str <str>\r\n", size);
		break;
	default:
		vOutputString("Unknown input action %u (size %u) requested!\r\n", act, size);
		return -EINVAL;
	}

	input_act = act;
	input_size = size;
	return 0;
}

static void cmd_reset(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	bt_mesh_reset();
	vOutputString("Local node reset complete\r\n");
}

static void cmd_net_send(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t ttl;
    uint8_t ctl;
    uint16_t src;
    uint16_t dst;
    uint8_t payload[16] = {0x00};
    
    if(argc != 5){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
    if (!bt_mesh_is_provisioned()) {
        vOutputString("Local node is not yet provisioned\r\n");
        return;
    }
    
    co_get_bytearray_from_string(&argv[1], &ttl, 1);
    co_get_bytearray_from_string(&argv[2], &ctl, 1);
    co_get_uint16_from_string(&argv[3], &src);
    co_get_uint16_from_string(&argv[4], &dst);
    
    struct bt_mesh_msg_ctx ctx = {
        .net_idx = net.net_idx,
        .app_idx = ctl ? BT_MESH_KEY_UNUSED : BT_MESH_KEY_DEV,
        .addr = dst,
        .send_rel = 0,
        .send_ttl = ttl,
    };
    
    struct bt_mesh_net_tx tx = {
        .sub = bt_mesh_subnet_get(ctx.net_idx),
        .ctx = &ctx,
        .src = src,
        .xmit = bt_mesh_net_transmit_get(),
        .friend_cred = 0,
    };
    
    struct net_buf *buf = bt_mesh_adv_create(BT_MESH_ADV_DATA, tx.xmit, K_NO_WAIT);
    if (!buf) {
        vOutputString("Out of network buffers\r\n");
        return;
    }
    
    vOutputString("Sending network packet\r\n");
    
    net_buf_reserve(buf, BT_MESH_NET_HDR_LEN);
    net_buf_add_mem(buf, payload, sizeof(payload));
    
    bt_mesh_net_send(&tx, buf, NULL, NULL);
}

static uint16_t get_app_idx(void)
{
    int i;
    
    for (i = 0; i < ARRAY_SIZE(bt_mesh.app_keys); i++) {
        struct bt_mesh_app_key *key = &bt_mesh.app_keys[i];
        
        if (key->net_idx != BT_MESH_KEY_UNUSED) {
            return key->app_idx;
        }
    }
    
    return BT_MESH_KEY_UNUSED;
}

static void cmd_seg_send(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    NET_BUF_SIMPLE_DEFINE(sdu, BT_MESH_TX_SDU_MAX);
    uint16_t src;
    uint16_t dst;
    uint8_t payload[12] = {0xFF};
    
    if(argc != 3){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
    if (!bt_mesh_is_provisioned()) {
        vOutputString("Local node is not yet provisioned\r\n");
        return;
    }
    
    co_get_uint16_from_string(&argv[1], &src);
    co_get_uint16_from_string(&argv[2], &dst);
    
    struct bt_mesh_msg_ctx ctx = {
        .net_idx = net.net_idx,
        .app_idx = get_app_idx(),
        .addr = dst,
        .send_rel = 1,
        .send_ttl = 0,
    };
    
    struct bt_mesh_net_tx tx = {
        .sub = bt_mesh_subnet_get(ctx.net_idx),
        .ctx = &ctx,
        .src = src,
        .xmit = bt_mesh_net_transmit_get(),
        .friend_cred = 0,
    };
    
    vOutputString("Sending segmented message\r\n");
    
    net_buf_simple_add_mem(&sdu, payload, sizeof(payload));
    
    bt_mesh_trans_send(&tx, &sdu, NULL, NULL);
}

static void cmd_rpl_clr(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
#if defined(CONFIG_BT_SETTINGS)
    int err;
    err = ef_del_env(NV_MESH_RPL);
    if(err) {
        vOutputString("Failed to clear replay protection list\r\n");
    } else {
        vOutputString("Replay protection list (size: %d) cleared\r\n", CONFIG_BT_MESH_CRPL);
    }
#endif
    
    memset(bt_mesh.rpl, 0, sizeof(bt_mesh.rpl));
}

static void cmd_ivu_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t enable;
    
    if(argc != 2){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
    co_get_bytearray_from_string(&argv[1], &enable, 1);
    
#if defined(CONFIG_BT_MESH_IV_UPDATE_TEST)
    bt_mesh_iv_update_test(enable);
#endif
    
    if (enable) {
        vOutputString("IV Update test mode enabled\r\n");
    } else {
        vOutputString("IV Update test mode disabled\r\n");
    }
}

static void cmd_iv_update(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t enable;
    
    if(argc != 2){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
    if (!bt_mesh_is_provisioned()) {
        vOutputString("Local node is not yet provisioned\r\n");
        return;
    }
    
    co_get_bytearray_from_string(&argv[1], &enable, 1);
    
    if (enable) {
        vOutputString("IV Update procedure started\r\n");
        
#if defined(CONFIG_BT_MESH_IV_UPDATE_TEST)
        bt_mesh_iv_update();
#endif
        
        if (!atomic_test_bit(bt_mesh.flags, BT_MESH_IVU_INITIATOR)) {
            bt_mesh_beacon_ivu_initiator(1);
        }
    } else {
        vOutputString("IV Update procedure stopped\r\n");
        
        if (atomic_test_bit(bt_mesh.flags, BT_MESH_IVU_INITIATOR)) {
            bt_mesh_beacon_ivu_initiator(0);
        }
    }
}

static void cmd_fault_set(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t type;
    int i;
    
    if(argc != 3){
        vOutputString("Number of Parameters is not correct\r\n");
        return;
    }
    
    co_get_bytearray_from_string(&argv[1], &type, 1);
    
    if (type == 0) {
        if(strlen(argv[2])/2 >= sizeof(cur_faults)) {
            co_get_bytearray_from_string(&argv[2], cur_faults, sizeof(cur_faults));
        } else {
            memset(cur_faults, 0x00, sizeof(cur_faults));
            co_get_bytearray_from_string(&argv[2], cur_faults, strlen(argv[2])/2);
        }
        
        vOutputString("Current Fault: ");
        for(i=0; i<sizeof(cur_faults); i++) {
            vOutputString("%02X", cur_faults[i]);
        }
        vOutputString("\r\n");
    } else {
        if(strlen(argv[2])/2 >= sizeof(reg_faults)) {
            co_get_bytearray_from_string(&argv[2], reg_faults, sizeof(reg_faults));
        } else {
            memset(reg_faults, 0x00, sizeof(reg_faults));
            co_get_bytearray_from_string(&argv[2], reg_faults, strlen(argv[2])/2);
        }
        
        vOutputString("Registered Fault: ");
        for(i=0; i<sizeof(reg_faults); i++) {
            vOutputString("%02X", reg_faults[i]);
        }
        vOutputString("\r\n");
    }
}

static void get_faults(u8_t *faults, u8_t faults_size, u8_t *dst, u8_t *count)
{
	u8_t i, limit = *count;

	for (i = 0U, *count = 0U; i < faults_size && *count < limit; i++) {
		if (faults[i]) {
			*dst++ = faults[i];
			(*count)++;
		}
	}
}

void show_faults(u8_t test_id, u16_t cid, u8_t *faults, size_t fault_count)
{
	size_t i;

	if (!fault_count) {
		vOutputString("Health Test ID 0x%02x Company ID 0x%04x: no faults\r\n", test_id, cid);
		return;
	}

	vOutputString("Health Test ID 0x%02x Company ID 0x%04x Fault Count %zu:\r\n", test_id, cid, fault_count);

	for (i = 0; i < fault_count; i++) {
		vOutputString("\t0x%02x", faults[i]);
	}
}

static void health_current_status(struct bt_mesh_health_cli *cli, u16_t addr,
				  u8_t test_id, u16_t cid, u8_t *faults,
				  size_t fault_count)
{
	vOutputString("Health Current Status from 0x%04x\r\n", addr);
	show_faults(test_id, cid, faults, fault_count);
}

static int fault_get_cur(struct bt_mesh_model *model, u8_t *test_id,
			 u16_t *company_id, u8_t *faults, u8_t *fault_count)
{
	vOutputString("Sending current faults\r\n");

	*test_id = 0x00;
	*company_id = BT_COMP_ID_LF;

	get_faults(cur_faults, sizeof(cur_faults), faults, fault_count);

	return 0;
}

static int fault_get_reg(struct bt_mesh_model *model, u16_t cid,
			 u8_t *test_id, u8_t *faults, u8_t *fault_count)
{
	if (cid != BT_COMP_ID_LF) {
		vOutputString("Faults requested for unknown Company ID 0x%04x\r\n", cid);
		return -EINVAL;
	}

	vOutputString("Sending registered faults\r\n");

	*test_id = 0x00;

	get_faults(reg_faults, sizeof(reg_faults), faults, fault_count);

	return 0;
}

static int fault_clear(struct bt_mesh_model *model, uint16_t cid)
{
	if (cid != BT_COMP_ID_LF) {
		return -EINVAL;
	}

	(void)memset(reg_faults, 0, sizeof(reg_faults));

	return 0;
}

static int fault_test(struct bt_mesh_model *model, uint8_t test_id, uint16_t cid)
{
	if (cid != BT_COMP_ID_LF) {
		return -EINVAL;
	}

	if (test_id != 0x00) {
		return -EINVAL;
	}

	return 0;
}

static void attn_on(struct bt_mesh_model *model)
{
#ifdef CONFIG_BT_MESH_PTS
	vOutputString("[PTS] Attention timer on\r\n");
#endif
}

static void attn_off(struct bt_mesh_model *model)
{
#ifdef CONFIG_BT_MESH_PTS
	vOutputString("[PTS] Attention timer off\r\n");
#endif
}

#if defined(CONFIG_BT_MESH_GATT_PROXY)
static void __attribute__((unused)) cmd_ident(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int err;

	err = bt_mesh_proxy_identity_enable();
	if (err) {
		vOutputString("Failed advertise using Node Identity (err ""%d)\r\n", err);
	}
}
#endif /* MESH_GATT_PROXY */

int blemesh_cli_register(void)
{
    #if defined(BL602)
    //aos_cli_register_commands(btStackCmdSet, sizeof(btMeshCmdSet)/sizeof(btMeshCmdSet[0]));
    #elif defined(BL70X)
    struct cli_command *cmdSet = btMeshCmdSet;
    while(cmdSet->pcCommand){
        FreeRTOS_CLIRegisterCommand(cmdSet);
        cmdSet++;
    }
    #endif

    return 0;
}
