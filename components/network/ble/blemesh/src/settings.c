/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/types.h>
#include <util.h>
#include <byteorder.h>

//#include <settings/settings.h>

#include <net/buf.h>

#include <hci_host.h>
#include <conn.h>
#include <include/mesh.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_SETTINGS)
#define LOG_MODULE_NAME bt_mesh_settings
#include "log.h"

#include "mesh_config.h"
#include "mesh.h"
#include "net.h"
#include "crypto.h"
#include "transport.h"
#include "access.h"
#include "foundation.h"
#include "proxy.h"
#include "settings.h"
#if defined(BFLB_BLE)
#include "mesh_settings.h"
#include "easyflash.h"
#endif

/* Tracking of what storage changes are pending for App and Net Keys. We
 * track this in a separate array here instead of within the respective
 * bt_mesh_app_key and bt_mesh_subnet structs themselves, since once a key
 * gets deleted its struct becomes invalid and may be reused for other keys.
 */
static struct key_update {
	u16_t key_idx:12,    /* AppKey or NetKey Index */
	      valid:1,       /* 1 if this entry is valid, 0 if not */
	      app_key:1,     /* 1 if this is an AppKey, 0 if a NetKey */
	      clear:1;       /* 1 if key needs clearing, 0 if storing */
} key_updates[CONFIG_BT_MESH_APP_KEY_COUNT + CONFIG_BT_MESH_SUBNET_COUNT];

static struct k_delayed_work pending_store;

/* Mesh network storage information */
struct net_val {
	u16_t primary_addr;
	u8_t  dev_key[16];
} __packed;

/* Sequence number storage */
struct seq_val {
	u8_t val[3];
} __packed;

/* Heartbeat Publication storage */
struct hb_pub_val {
	u16_t dst;
	u8_t  period;
	u8_t  ttl;
	u16_t feat;
	u16_t net_idx:12,
	      indefinite:1;
};

/* Miscellaneous configuration server model states */
struct cfg_val {
	u8_t net_transmit;
	u8_t relay;
	u8_t relay_retransmit;
	u8_t beacon;
	u8_t gatt_proxy;
	u8_t frnd;
	u8_t default_ttl;
};

/* IV Index & IV Update storage */
struct iv_val {
	u32_t iv_index;
	u8_t  iv_update:1,
	      iv_duration:7;
} __packed;

/* Replay Protection List storage */
struct rpl_val {
	u32_t seq:24,
	      old_iv:1;
};

/* NetKey storage information */
struct net_key_val {
	u8_t kr_flag:1,
	     kr_phase:7;
	u8_t val[2][16];
} __packed;

/* AppKey storage information */
struct app_key_val {
	u16_t net_idx;
	bool  updated;
	u8_t  val[2][16];
} __packed;

struct mod_pub_val {
	u16_t addr;
	u16_t key;
	u8_t  ttl;
	u8_t  retransmit;
	u8_t  period;
	u8_t  period_div:4,
	      cred:1;
};

/* We need this so we don't overwrite app-hardcoded values in case FCB
 * contains a history of changes but then has a NULL at the end.
 */
static struct {
	bool valid;
	struct cfg_val cfg;
} stored_cfg;

#if defined(BFLB_BLE)
struct net_key {
    u16_t net_idx;
	struct net_key_val key_val;
} __packed;

struct app_key {
    u16_t app_idx;
	struct app_key_val key_val;
} __packed;

struct mesh_rpl_val {
	u16_t src;
	struct rpl_val rpl;
} __packed;

u16_t mesh_netkey_idx[CONFIG_BT_MESH_SUBNET_COUNT];
u16_t mesh_appkey_idx[CONFIG_BT_MESH_APP_KEY_COUNT];

static void encode_mod_path(struct bt_mesh_model *mod, bool vnd,
			    const char *key, char *path, size_t path_len);

static int net_set(void)
{
	struct net_val net;
	int err;

    err = bt_settings_get_bin(NV_MESH_NET, (u8_t *)&net, sizeof(net), NULL);
	if (err || !net.primary_addr) {
		bt_mesh_comp_unprovision();
		(void)memset(bt_mesh.dev_key, 0, sizeof(bt_mesh.dev_key));
		return 0;
	}

	memcpy(bt_mesh.dev_key, net.dev_key, sizeof(bt_mesh.dev_key));
	bt_mesh_comp_provision(net.primary_addr);

	BT_DBG("Provisioned with primary address 0x%04x", net.primary_addr);
	BT_DBG("Recovered DevKey %s", bt_hex(bt_mesh.dev_key, 16));

	return 0;
}

static int iv_set(void)
{
	struct iv_val iv;
	int err;

    err = bt_settings_get_bin(NV_MESH_IV, (u8_t *)&iv, sizeof(iv), NULL);

	if (err) {
		bt_mesh.iv_index = 0U;
		atomic_clear_bit(bt_mesh.flags, BT_MESH_IVU_IN_PROGRESS);
		return 0;
	}

	bt_mesh.iv_index = iv.iv_index;
	atomic_set_bit_to(bt_mesh.flags, BT_MESH_IVU_IN_PROGRESS, iv.iv_update);
	bt_mesh.ivu_duration = iv.iv_duration;

	BT_DBG("IV Index 0x%04x (IV Update Flag %u) duration %u hours",
	       iv.iv_index, iv.iv_update, iv.iv_duration);

	return 0;
}

static int seq_set(void)
{
	struct seq_val seq;
	int err;

    err = bt_settings_get_bin(NV_MESH_SEQ, (u8_t *)&seq, sizeof(seq), NULL);
	if (err) {
		bt_mesh.seq = 0U;
		return 0;
	}

	bt_mesh.seq = ((u32_t)seq.val[0] | ((u32_t)seq.val[1] << 8) |
		       ((u32_t)seq.val[2] << 16));

	if (CONFIG_BT_MESH_SEQ_STORE_RATE > 0) {
		/* Make sure we have a large enough sequence number. We
		 * subtract 1 so that the first transmission causes a write
		 * to the settings storage.
		 */
		bt_mesh.seq += (CONFIG_BT_MESH_SEQ_STORE_RATE -
				(bt_mesh.seq % CONFIG_BT_MESH_SEQ_STORE_RATE));
		bt_mesh.seq--;
	}

	BT_DBG("Sequence Number 0x%06x", bt_mesh.seq);

	return 0;
}

static int rpl_set(void)
{
	bt_settings_get_bin(NV_MESH_RPL, (u8_t *)bt_mesh.rpl, sizeof(bt_mesh.rpl), NULL);
	return 0;
}

static int net_key_set(void)
{
	struct bt_mesh_subnet *sub;
	struct net_key_val key;
    char path[20];
	int i, j, err;

    err = bt_settings_get_bin("bt/mesh/NetKeyIndex", (u8_t *)mesh_netkey_idx, sizeof(mesh_netkey_idx), NULL);
    if(err){
        BT_ERR("Failed to load netkey index");
        return err;
    }
    for(i = 0; i < CONFIG_BT_MESH_SUBNET_COUNT; i++){
        if(mesh_netkey_idx[i] == BT_MESH_KEY_UNUSED){
            continue;
        }
            
        snprintk(path, sizeof(path), "bt/mesh/NetKey/%x", mesh_netkey_idx[i]);            
        err = bt_settings_get_bin((const char *)path, (u8_t *)&key, sizeof(key), NULL);
        if(err){
            BT_ERR("Failed to load netkey with netkey index %d", mesh_netkey_idx[i]);
            return err;
        }
        sub = bt_mesh_subnet_get(mesh_netkey_idx[i]);
        
        if (!sub){ 
            for (j = 0; j < ARRAY_SIZE(bt_mesh.sub); j++) {
		        if (bt_mesh.sub[j].net_idx == BT_MESH_KEY_UNUSED) {
			       sub = &bt_mesh.sub[j];
			       break;
		        }
	        }

            if (!sub){
		        BT_ERR("No space to allocate a new subnet");
		        return -ENOMEM;
	        }

        }

        if(sub){
            sub->net_idx = mesh_netkey_idx[i];
            sub->kr_flag = key.kr_flag;
            sub->kr_phase = key.kr_phase;
            memcpy(sub->keys[0].net, &key.val[0], 16);
            memcpy(sub->keys[1].net, &key.val[1], 16);
             
            BT_DBG("NetKeyIndex 0x%03x recovered from storage", mesh_netkey_idx[i]);
        }
    }

	return 0;
}

static int app_key_set(void)
{
	struct bt_mesh_app_key *app;
	struct app_key_val key;
    char path[20];
	int err, i;

    err = bt_settings_get_bin("bt/mesh/AppKeyIndex", (u8_t *)mesh_appkey_idx, sizeof(mesh_appkey_idx), NULL);
    if(err){
        BT_ERR("Failed to load appkey index");
        return err;
    }

    for(i = 0; i < CONFIG_BT_MESH_APP_KEY_COUNT; i++){
       if(mesh_appkey_idx[i] == BT_MESH_KEY_UNUSED){
           continue;
       }
           
       snprintk(path, sizeof(path), "bt/mesh/AppKey/%x", mesh_appkey_idx[i]);            
       err = bt_settings_get_bin((const char *)path, (u8_t *)&key, sizeof(key), NULL);
       if(err){
           BT_ERR("Failed to load netkey with netkey index %d", mesh_appkey_idx[i]);
           return err;
       }

       app = bt_mesh_app_key_find(mesh_appkey_idx[i]);
       if (!app) {
   	       app = bt_mesh_app_key_alloc(mesh_appkey_idx[i]);
       }

       if (!app) {
	       BT_ERR("No space for a new app key");
		   return -ENOMEM;
	   }

       app->net_idx = key.net_idx;
       app->app_idx = mesh_appkey_idx[i];
       app->updated = key.updated;
       memcpy(app->keys[0].val, key.val[0], 16);
       memcpy(app->keys[1].val, key.val[1], 16);
       
       bt_mesh_app_id(app->keys[0].val, &app->keys[0].id);
       bt_mesh_app_id(app->keys[1].val, &app->keys[1].id);
       
       BT_DBG("AppKeyIndex 0x%03x recovered from storage", mesh_app_key[i].app_idx);
    } 

	return 0;
}

static int hb_pub_set(void)
{
	struct bt_mesh_hb_pub *pub = bt_mesh_hb_pub_get();
	struct hb_pub_val hb_val;
	int err;

	if (!pub) {
		return -ENOENT;
	}

    err = bt_settings_get_bin(NV_MESH_HBPUB, (u8_t *)&hb_val, sizeof(hb_val), NULL);
    
	if (err) {
		pub->dst = BT_MESH_ADDR_UNASSIGNED;
		pub->count = 0U;
		pub->ttl = 0U;
		pub->period = 0U;
		pub->feat = 0U;

		return 0;
	}

	pub->dst = hb_val.dst;
	pub->period = hb_val.period;
	pub->ttl = hb_val.ttl;
	pub->feat = hb_val.feat;
	pub->net_idx = hb_val.net_idx;

	if (hb_val.indefinite) {
		pub->count = 0xffff;
	} else {
		pub->count = 0U;
	}

	BT_DBG("Restored heartbeat publication");

	return 0;
}

static int cfg_set(void)
{
	struct bt_mesh_cfg_srv *cfg = bt_mesh_cfg_get();
	int err;

	if (!cfg) {
		return -ENOENT;
	}

    err = bt_settings_get_bin(NV_MESH_CFG, (u8_t *)&stored_cfg.cfg, sizeof(stored_cfg.cfg), NULL);
    
	if (err) {
		stored_cfg.valid = false;
		return 0;
	}

	stored_cfg.valid = true;
	BT_DBG("Restored configuration state");

	return 0;
}

static int mod_set_bind(struct bt_mesh_model *mod, bool vnd)
{
    char path[20];

    encode_mod_path(mod, vnd, "bind", path, sizeof(path));
    
    bt_settings_get_bin((const char *)path, (u8_t *)(mod->keys), sizeof(mod->keys), NULL);
	
	return 0;
}

static int mod_set_sub(struct bt_mesh_model *mod, bool vnd)
{
    char path[20];

	/* Start with empty array regardless of cleared or set value */
	(void)memset(mod->groups, 0, sizeof(mod->groups));

    encode_mod_path(mod, vnd, "sub", path, sizeof(path));

    bt_settings_get_bin((const char *)path, (u8_t *)(mod->groups), sizeof(mod->groups), NULL);
	
	return 0;
}

static int mod_set_pub(struct bt_mesh_model *mod, bool vnd)
{
	struct mod_pub_val pub;
	int err;
    char path[20];

	if (!mod->pub) {
		BT_WARN("Model has no publication context!");
		return -EINVAL;
	}

    encode_mod_path(mod, vnd, "pub", path, sizeof(path));

    err = bt_settings_get_bin((const char *)path, (u8_t *)&pub, sizeof(pub), NULL);
    
	if (err) {
		mod->pub->addr = BT_MESH_ADDR_UNASSIGNED;
		mod->pub->key = 0U;
		mod->pub->cred = 0U;
		mod->pub->ttl = 0U;
		mod->pub->period = 0U;
		mod->pub->retransmit = 0U;
		mod->pub->count = 0U;

		BT_DBG("Cleared publication for model");
		return 0;
	}

	mod->pub->addr = pub.addr;
	mod->pub->key = pub.key;
	mod->pub->cred = pub.cred;
	mod->pub->ttl = pub.ttl;
	mod->pub->period = pub.period;
	mod->pub->retransmit = pub.retransmit;
	mod->pub->count = 0U;

	BT_DBG("Restored model publication, dst 0x%04x app_idx 0x%03x",
	       pub.addr, pub.key);

	return 0;
}

static int mod_set(bool vnd, struct bt_mesh_model *model)
{
    mod_set_bind(model, vnd);
    mod_set_sub(model, vnd);
	mod_set_pub(model, vnd);
    
	return 0;
}

static int sig_mod_set(void)
{
    struct bt_mesh_elem *elem;
    struct bt_mesh_model *model;
    struct bt_mesh_comp * comp;
    int i, j;
    int err;

    comp = bt_mesh_comp_get();
    
    for(i = 0; i < comp->elem_count; i++) {
        elem = &comp->elem[i];
        for(j = 0; j < elem->model_count; j++) {
            model = &elem->models[j];
            err = mod_set(false, model);
            if(err)
                return err;
        }
    }

    return 0;
}

static int vnd_mod_set(void)
{
	struct bt_mesh_elem *elem;
    struct bt_mesh_model *model;
    struct bt_mesh_comp * comp;
    int i, j;
    int err;

    comp = bt_mesh_comp_get();
    for(i = 0; i < comp->elem_count; i++) {
        elem = &comp->elem[i];
        for(j = 0; j < elem->vnd_model_count; j++) {
            model = &elem->vnd_models[j];
            err = mod_set(true, model);
            if(err)
                return err;
        }
    }
    
    return 0;
}

#else
static inline int mesh_x_set(settings_read_cb read_cb, void *cb_arg, void *out,
			     size_t read_len)
{
	ssize_t len;

	len = read_cb(cb_arg, out, read_len);
	if (len < 0) {
		BT_ERR("Failed to read value (err %zu)", len);
		return len;
	}

	BT_HEXDUMP_DBG(out, len, "val");

	if (len != read_len) {
		BT_ERR("Unexpected value length (%zu != %zu)", len, read_len);
		return -EINVAL;
	}

	return 0;
}

static int net_set(const char *name, size_t len_rd, settings_read_cb read_cb,
		   void *cb_arg)
{
	struct net_val net;
	int err;

	if (len_rd == 0) {
		BT_DBG("val (null)");

		bt_mesh_comp_unprovision();
		(void)memset(bt_mesh.dev_key, 0, sizeof(bt_mesh.dev_key));
		return 0;
	}

	err = mesh_x_set(read_cb, cb_arg, &net, sizeof(net));
	if (err) {
		BT_ERR("Failed to set \'net\'");
		return err;
	}

	memcpy(bt_mesh.dev_key, net.dev_key, sizeof(bt_mesh.dev_key));
	bt_mesh_comp_provision(net.primary_addr);

	BT_DBG("Provisioned with primary address 0x%04x", net.primary_addr);
	BT_DBG("Recovered DevKey %s", bt_hex(bt_mesh.dev_key, 16));

	return 0;
}

static int iv_set(const char *name, size_t len_rd, settings_read_cb read_cb,
		  void *cb_arg)
{
	struct iv_val iv;
	int err;

	if (len_rd == 0) {
		BT_DBG("IV deleted");

		bt_mesh.iv_index = 0U;
		atomic_clear_bit(bt_mesh.flags, BT_MESH_IVU_IN_PROGRESS);
		return 0;
	}

	err = mesh_x_set(read_cb, cb_arg, &iv, sizeof(iv));
	if (err) {
		BT_ERR("Failed to set \'iv\'");
		return err;
	}

	bt_mesh.iv_index = iv.iv_index;
	atomic_set_bit_to(bt_mesh.flags, BT_MESH_IVU_IN_PROGRESS, iv.iv_update);
	bt_mesh.ivu_duration = iv.iv_duration;

	BT_DBG("IV Index 0x%04x (IV Update Flag %u) duration %u hours",
	       iv.iv_index, iv.iv_update, iv.iv_duration);

	return 0;
}

static int seq_set(const char *name, size_t len_rd, settings_read_cb read_cb,
		   void *cb_arg)
{
	struct seq_val seq;
	int err;

	if (len_rd == 0) {
		BT_DBG("val (null)");

		bt_mesh.seq = 0U;
		return 0;
	}

	err = mesh_x_set(read_cb, cb_arg, &seq, sizeof(seq));
	if (err) {
		BT_ERR("Failed to set \'seq\'");
		return err;
	}

	bt_mesh.seq = ((u32_t)seq.val[0] | ((u32_t)seq.val[1] << 8) |
		       ((u32_t)seq.val[2] << 16));

	if (CONFIG_BT_MESH_SEQ_STORE_RATE > 0) {
		/* Make sure we have a large enough sequence number. We
		 * subtract 1 so that the first transmission causes a write
		 * to the settings storage.
		 */
		bt_mesh.seq += (CONFIG_BT_MESH_SEQ_STORE_RATE -
				(bt_mesh.seq % CONFIG_BT_MESH_SEQ_STORE_RATE));
		bt_mesh.seq--;
	}

	BT_DBG("Sequence Number 0x%06x", bt_mesh.seq);

	return 0;
}

static struct bt_mesh_rpl *rpl_find(u16_t src)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bt_mesh.rpl); i++) {
		if (bt_mesh.rpl[i].src == src) {
			return &bt_mesh.rpl[i];
		}
	}

	return NULL;
}

static struct bt_mesh_rpl *rpl_alloc(u16_t src)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bt_mesh.rpl); i++) {
		if (!bt_mesh.rpl[i].src) {
			bt_mesh.rpl[i].src = src;
			return &bt_mesh.rpl[i];
		}
	}

	return NULL;
}

static int rpl_set(const char *name, size_t len_rd,
		   settings_read_cb read_cb, void *cb_arg)
{
	struct bt_mesh_rpl *entry;
	struct rpl_val rpl;
	int err;
	u16_t src;

	if (!name) {
		BT_ERR("Insufficient number of arguments");
		return -ENOENT;
	}

	src = strtol(name, NULL, 16);
	entry = rpl_find(src);

	if (len_rd == 0) {
		BT_DBG("val (null)");
		if (entry) {
			(void)memset(entry, 0, sizeof(*entry));
		} else {
			BT_WARN("Unable to find RPL entry for 0x%04x", src);
		}

		return 0;
	}

	if (!entry) {
		entry = rpl_alloc(src);
		if (!entry) {
			BT_ERR("Unable to allocate RPL entry for 0x%04x", src);
			return -ENOMEM;
		}
	}

	err = mesh_x_set(read_cb, cb_arg, &rpl, sizeof(rpl));
	if (err) {
		BT_ERR("Failed to set `net`");
		return err;
	}

	entry->seq = rpl.seq;
	entry->old_iv = rpl.old_iv;

	BT_DBG("RPL entry for 0x%04x: Seq 0x%06x old_iv %u", entry->src,
	       entry->seq, entry->old_iv);

	return 0;
}

static int net_key_set(const char *name, size_t len_rd,
		       settings_read_cb read_cb, void *cb_arg)
{
	struct bt_mesh_subnet *sub;
	struct net_key_val key;
	int i, err;
	u16_t net_idx;

	if (!name) {
		BT_ERR("Insufficient number of arguments");
		return -ENOENT;
	}

	net_idx = strtol(name, NULL, 16);
	sub = bt_mesh_subnet_get(net_idx);

	if (len_rd == 0) {
		BT_DBG("val (null)");
		if (!sub) {
			BT_ERR("No subnet with NetKeyIndex 0x%03x", net_idx);
			return -ENOENT;
		}

		BT_DBG("Deleting NetKeyIndex 0x%03x", net_idx);
		bt_mesh_subnet_del(sub, false);
		return 0;
	}

	err = mesh_x_set(read_cb, cb_arg, &key, sizeof(key));
	if (err) {
		BT_ERR("Failed to set \'net-key\'");
		return err;
	}

	if (sub) {
		BT_DBG("Updating existing NetKeyIndex 0x%03x", net_idx);

		sub->kr_flag = key.kr_flag;
		sub->kr_phase = key.kr_phase;
		memcpy(sub->keys[0].net, &key.val[0], 16);
		memcpy(sub->keys[1].net, &key.val[1], 16);

		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(bt_mesh.sub); i++) {
		if (bt_mesh.sub[i].net_idx == BT_MESH_KEY_UNUSED) {
			sub = &bt_mesh.sub[i];
			break;
		}
	}

	if (!sub) {
		BT_ERR("No space to allocate a new subnet");
		return -ENOMEM;
	}

	sub->net_idx = net_idx;
	sub->kr_flag = key.kr_flag;
	sub->kr_phase = key.kr_phase;
	memcpy(sub->keys[0].net, &key.val[0], 16);
	memcpy(sub->keys[1].net, &key.val[1], 16);

	BT_DBG("NetKeyIndex 0x%03x recovered from storage", net_idx);

	return 0;
}

static int app_key_set(const char *name, size_t len_rd,
		       settings_read_cb read_cb, void *cb_arg)
{
	struct bt_mesh_app_key *app;
	struct app_key_val key;
	u16_t app_idx;
	int err;

	if (!name) {
		BT_ERR("Insufficient number of arguments");
		return -ENOENT;
	}

	app_idx = strtol(name, NULL, 16);

	if (len_rd == 0) {
		BT_DBG("val (null)");
		BT_DBG("Deleting AppKeyIndex 0x%03x", app_idx);

		app = bt_mesh_app_key_find(app_idx);
		if (app) {
			bt_mesh_app_key_del(app, false);
		}

		return 0;
	}

	err = mesh_x_set(read_cb, cb_arg, &key, sizeof(key));
	if (err) {
		BT_ERR("Failed to set \'app-key\'");
		return err;
	}

	app = bt_mesh_app_key_find(app_idx);
	if (!app) {
		app = bt_mesh_app_key_alloc(app_idx);
	}

	if (!app) {
		BT_ERR("No space for a new app key");
		return -ENOMEM;
	}

	app->net_idx = key.net_idx;
	app->app_idx = app_idx;
	app->updated = key.updated;
	memcpy(app->keys[0].val, key.val[0], 16);
	memcpy(app->keys[1].val, key.val[1], 16);

	bt_mesh_app_id(app->keys[0].val, &app->keys[0].id);
	bt_mesh_app_id(app->keys[1].val, &app->keys[1].id);

	BT_DBG("AppKeyIndex 0x%03x recovered from storage", app_idx);

	return 0;
}

static int hb_pub_set(const char *name, size_t len_rd,
		      settings_read_cb read_cb, void *cb_arg)
{
	struct bt_mesh_hb_pub *pub = bt_mesh_hb_pub_get();
	struct hb_pub_val hb_val;
	int err;

	if (!pub) {
		return -ENOENT;
	}

	if (len_rd == 0) {
		pub->dst = BT_MESH_ADDR_UNASSIGNED;
		pub->count = 0U;
		pub->ttl = 0U;
		pub->period = 0U;
		pub->feat = 0U;

		BT_DBG("Cleared heartbeat publication");
		return 0;
	}

	err = mesh_x_set(read_cb, cb_arg, &hb_val, sizeof(hb_val));
	if (err) {
		BT_ERR("Failed to set \'hb_val\'");
		return err;
	}

	pub->dst = hb_val.dst;
	pub->period = hb_val.period;
	pub->ttl = hb_val.ttl;
	pub->feat = hb_val.feat;
	pub->net_idx = hb_val.net_idx;

	if (hb_val.indefinite) {
		pub->count = 0xffff;
	} else {
		pub->count = 0U;
	}

	BT_DBG("Restored heartbeat publication");

	return 0;
}

static int cfg_set(const char *name, size_t len_rd,
		   settings_read_cb read_cb, void *cb_arg)
{
	struct bt_mesh_cfg_srv *cfg = bt_mesh_cfg_get();
	int err;

	if (!cfg) {
		return -ENOENT;
	}

	if (len_rd == 0) {
		stored_cfg.valid = false;
		BT_DBG("Cleared configuration state");
		return 0;
	}


	err = mesh_x_set(read_cb, cb_arg, &stored_cfg.cfg,
			 sizeof(stored_cfg.cfg));
	if (err) {
		BT_ERR("Failed to set \'cfg\'");
		return err;
	}

	stored_cfg.valid = true;
	BT_DBG("Restored configuration state");

	return 0;
}

static int mod_set_bind(struct bt_mesh_model *mod, size_t len_rd,
			settings_read_cb read_cb, void *cb_arg)
{
	ssize_t len;
	int i;

	/* Start with empty array regardless of cleared or set value */
	for (i = 0; i < ARRAY_SIZE(mod->keys); i++) {
		mod->keys[i] = BT_MESH_KEY_UNUSED;
	}

	if (len_rd == 0) {
		BT_DBG("Cleared bindings for model");
		return 0;
	}

	len = read_cb(cb_arg, mod->keys, sizeof(mod->keys));
	if (len < 0) {
		BT_ERR("Failed to read value (err %zu)", len);
		return len;
	}


	BT_DBG("Decoded %zu bound keys for model", len / sizeof(mod->keys[0]));
	return 0;
}

static int mod_set_sub(struct bt_mesh_model *mod, size_t len_rd,
		       settings_read_cb read_cb, void *cb_arg)
{
	ssize_t len;

	/* Start with empty array regardless of cleared or set value */
	(void)memset(mod->groups, 0, sizeof(mod->groups));

	if (len_rd == 0) {
		BT_DBG("Cleared subscriptions for model");
		return 0;
	}

	len = read_cb(cb_arg, mod->groups, sizeof(mod->groups));
	if (len < 0) {
		BT_ERR("Failed to read value (err %zu)", len);
		return len;
	}

	BT_DBG("Decoded %zu subscribed group addresses for model",
	       len / sizeof(mod->groups[0]));
	return 0;
}

static int mod_set_pub(struct bt_mesh_model *mod, size_t len_rd,
		       settings_read_cb read_cb, void *cb_arg)
{
	struct mod_pub_val pub;
	int err;

	if (!mod->pub) {
		BT_WARN("Model has no publication context!");
		return -EINVAL;
	}

	if (len_rd == 0) {
		mod->pub->addr = BT_MESH_ADDR_UNASSIGNED;
		mod->pub->key = 0U;
		mod->pub->cred = 0U;
		mod->pub->ttl = 0U;
		mod->pub->period = 0U;
		mod->pub->retransmit = 0U;
		mod->pub->count = 0U;

		BT_DBG("Cleared publication for model");
		return 0;
	}

	err = mesh_x_set(read_cb, cb_arg, &pub, sizeof(pub));
	if (err) {
		BT_ERR("Failed to set \'model-pub\'");
		return err;
	}

	mod->pub->addr = pub.addr;
	mod->pub->key = pub.key;
	mod->pub->cred = pub.cred;
	mod->pub->ttl = pub.ttl;
	mod->pub->period = pub.period;
	mod->pub->retransmit = pub.retransmit;
	mod->pub->count = 0U;

	BT_DBG("Restored model publication, dst 0x%04x app_idx 0x%03x",
	       pub.addr, pub.key);

	return 0;
}

static int mod_set(bool vnd, const char *name, size_t len_rd,
		   settings_read_cb read_cb, void *cb_arg)
{
	struct bt_mesh_model *mod;
	u8_t elem_idx, mod_idx;
	u16_t mod_key;
	int len;
	const char *next;

	if (!name) {
		BT_ERR("Insufficient number of arguments");
		return -ENOENT;
	}

	mod_key = strtol(name, NULL, 16);
	elem_idx = mod_key >> 8;
	mod_idx = mod_key;

	BT_DBG("Decoded mod_key 0x%04x as elem_idx %u mod_idx %u",
	       mod_key, elem_idx, mod_idx);

	mod = bt_mesh_model_get(vnd, elem_idx, mod_idx);
	if (!mod) {
		BT_ERR("Failed to get model for elem_idx %u mod_idx %u",
		       elem_idx, mod_idx);
		return -ENOENT;
	}

	len = settings_name_next(name, &next);

	if (!next) {
		BT_ERR("Insufficient number of arguments");
		return -ENOENT;
	}

	if (!strncmp(next, "bind", len)) {
		return mod_set_bind(mod, len_rd, read_cb, cb_arg);
	}

	if (!strncmp(next, "sub", len)) {
		return mod_set_sub(mod, len_rd, read_cb, cb_arg);
	}

	if (!strncmp(next, "pub", len)) {
		return mod_set_pub(mod, len_rd, read_cb, cb_arg);
	}

	BT_WARN("Unknown module key %s", next);
	return -ENOENT;
}

static int sig_mod_set(const char *name, size_t len_rd,
		       settings_read_cb read_cb, void *cb_arg)
{
	return mod_set(false, name, len_rd, read_cb, cb_arg);
}

static int vnd_mod_set(const char *name, size_t len_rd,
		       settings_read_cb read_cb, void *cb_arg)
{
	return mod_set(true, name, len_rd, read_cb, cb_arg);
}
#endif

const struct mesh_setting {
	const char *name;
    #if defined(BFLB_BLE)
    int (*func)(void);
    #else
	int (*func)(const char *name, size_t len_rd,
		    settings_read_cb read_cb, void *cb_arg);
    #endif
} settings[] = {
	{ "Net", net_set },
	{ "IV", iv_set },
	{ "Seq", seq_set },
	{ "RPL", rpl_set },
	{ "NetKey", net_key_set },
	{ "AppKey", app_key_set },
	{ "HBPub", hb_pub_set },
	{ "Cfg", cfg_set },
	{ "s", sig_mod_set },
	{ "v", vnd_mod_set },
};

#if defined(BFLB_BLE)
int mesh_set(void)
{
    int i;

    for(i = 0; i < ARRAY_SIZE(settings); i++){
        settings[i].func();
    }

    return 0;
}
#else
static int mesh_set(const char *name, size_t len_rd,
		    settings_read_cb read_cb, void *cb_arg)
{
	int i, len;
	const char *next;

	if (!name) {
		BT_ERR("Insufficient number of arguments");
		return -EINVAL;
	}

	len = settings_name_next(name, &next);

	for (i = 0; i < ARRAY_SIZE(settings); i++) {
		if (!strncmp(settings[i].name, name, len)) {
			return settings[i].func(next, len_rd, read_cb, cb_arg);
		}
	}

	BT_WARN("No matching handler for key %s", log_strdup(name));

	return -ENOENT;
}
#endif

static int subnet_init(struct bt_mesh_subnet *sub)
{
	int err;

	err = bt_mesh_net_keys_create(&sub->keys[0], sub->keys[0].net);
	if (err) {
		BT_ERR("Unable to generate keys for subnet");
		return -EIO;
	}

	if (sub->kr_phase != BT_MESH_KR_NORMAL) {
		err = bt_mesh_net_keys_create(&sub->keys[1], sub->keys[1].net);
		if (err) {
			BT_ERR("Unable to generate keys for subnet");
			(void)memset(&sub->keys[0], 0, sizeof(sub->keys[0]));
			return -EIO;
		}
	}

	if (IS_ENABLED(CONFIG_BT_MESH_GATT_PROXY)) {
		sub->node_id = BT_MESH_NODE_IDENTITY_STOPPED;
	} else {
		sub->node_id = BT_MESH_NODE_IDENTITY_NOT_SUPPORTED;
	}

	/* Make sure we have valid beacon data to be sent */
	bt_mesh_net_beacon_update(sub);

	return 0;
}

static void commit_mod(struct bt_mesh_model *mod, struct bt_mesh_elem *elem,
		       bool vnd, bool primary, void *user_data)
{
	if (mod->pub && mod->pub->update &&
	    mod->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		s32_t ms = bt_mesh_model_pub_period_get(mod);
		if (ms) {
			BT_DBG("Starting publish timer (period %u ms)", ms);
			k_delayed_work_submit(&mod->pub->timer, ms);
		}
	}
}

#if defined(BFLB_BLE)
int mesh_commit(void)
#else
static int mesh_commit(void)
#endif
{
	struct bt_mesh_hb_pub *hb_pub;
	struct bt_mesh_cfg_srv *cfg;
	int i;

	BT_DBG("sub[0].net_idx 0x%03x", bt_mesh.sub[0].net_idx);

	if (bt_mesh.sub[0].net_idx == BT_MESH_KEY_UNUSED) {
		/* Nothing to do since we're not yet provisioned */
		return 0;
	}

	if (IS_ENABLED(CONFIG_BT_MESH_PB_GATT)) {
		bt_mesh_proxy_prov_disable(true);
	}

	for (i = 0; i < ARRAY_SIZE(bt_mesh.sub); i++) {
		struct bt_mesh_subnet *sub = &bt_mesh.sub[i];
		int err;

		if (sub->net_idx == BT_MESH_KEY_UNUSED) {
			continue;
		}

		err = subnet_init(sub);
		if (err) {
			BT_ERR("Failed to init subnet 0x%03x", sub->net_idx);
		}
	}

	if (bt_mesh.ivu_duration < BT_MESH_IVU_MIN_HOURS) {
		k_delayed_work_submit(&bt_mesh.ivu_timer, BT_MESH_IVU_TIMEOUT);
	}

	bt_mesh_model_foreach(commit_mod, NULL);

	hb_pub = bt_mesh_hb_pub_get();
	if (hb_pub && hb_pub->dst != BT_MESH_ADDR_UNASSIGNED &&
	    hb_pub->count && hb_pub->period) {
		BT_DBG("Starting heartbeat publication");
		k_work_submit(&hb_pub->timer.work);
	}

	cfg = bt_mesh_cfg_get();
	if (cfg && stored_cfg.valid) {
		cfg->net_transmit = stored_cfg.cfg.net_transmit;
		cfg->relay = stored_cfg.cfg.relay;
		cfg->relay_retransmit = stored_cfg.cfg.relay_retransmit;
		cfg->beacon = stored_cfg.cfg.beacon;
		cfg->gatt_proxy = stored_cfg.cfg.gatt_proxy;
		cfg->frnd = stored_cfg.cfg.frnd;
		cfg->default_ttl = stored_cfg.cfg.default_ttl;
	}

	atomic_set_bit(bt_mesh.flags, BT_MESH_VALID);

	bt_mesh_net_start();

	return 0;
}

#if !defined(BFLB_BLE)
SETTINGS_STATIC_HANDLER_DEFINE(bt_mesh, "bt/mesh", NULL, mesh_set, mesh_commit,
			       NULL);
#endif

/* Pending flags that use K_NO_WAIT as the storage timeout */
#define NO_WAIT_PENDING_BITS (BIT(BT_MESH_NET_PENDING) |           \
			      BIT(BT_MESH_IV_PENDING) |            \
			      BIT(BT_MESH_SEQ_PENDING))

/* Pending flags that use CONFIG_BT_MESH_STORE_TIMEOUT */
#define GENERIC_PENDING_BITS (BIT(BT_MESH_KEYS_PENDING) |          \
			      BIT(BT_MESH_HB_PUB_PENDING) |        \
			      BIT(BT_MESH_CFG_PENDING) |           \
			      BIT(BT_MESH_MOD_PENDING))

static void schedule_store(int flag)
{
	s32_t timeout, remaining;

	atomic_set_bit(bt_mesh.flags, flag);

	if (atomic_get(bt_mesh.flags) & NO_WAIT_PENDING_BITS) {
		timeout = K_NO_WAIT;
	} else if (atomic_test_bit(bt_mesh.flags, BT_MESH_RPL_PENDING) &&
		   (!(atomic_get(bt_mesh.flags) & GENERIC_PENDING_BITS) ||
		    (CONFIG_BT_MESH_RPL_STORE_TIMEOUT <
		     CONFIG_BT_MESH_STORE_TIMEOUT))) {
		timeout = K_SECONDS(CONFIG_BT_MESH_RPL_STORE_TIMEOUT);
	} else {
		timeout = K_SECONDS(CONFIG_BT_MESH_STORE_TIMEOUT);
	}

	remaining = k_delayed_work_remaining_get(&pending_store);
	if (remaining && remaining < timeout) {
		BT_DBG("Not rescheduling due to existing earlier deadline");
		return;
	}

	BT_DBG("Waiting %d seconds", timeout / MSEC_PER_SEC);

	k_delayed_work_submit(&pending_store, timeout);
}

static void clear_iv(void)
{
	int err;

	err = settings_delete(NV_MESH_IV);
	if (err) {
		BT_ERR("Failed to clear IV");
	} else {
		BT_DBG("Cleared IV");
	}
}

static void clear_net(void)
{
	int err;

	err = settings_delete(NV_MESH_NET);

	if (err) {
		BT_ERR("Failed to clear Network, err(%d)", err);
	} else {
		BT_DBG("Cleared Network");
	}
}

static void store_pending_net(void)
{
	struct net_val net;
	int err;

	BT_DBG("addr 0x%04x DevKey %s", bt_mesh_primary_addr(),
	       bt_hex(bt_mesh.dev_key, 16));

	net.primary_addr = bt_mesh_primary_addr();
	memcpy(net.dev_key, bt_mesh.dev_key, 16);

	err = settings_save_one(NV_MESH_NET, (const u8_t *)&net, sizeof(net));

	if (err) {
		BT_ERR("Failed to store Network value");
	} else {
		BT_DBG("Stored Network value");
	}
}

void bt_mesh_store_net(void)
{
	schedule_store(BT_MESH_NET_PENDING);
}

static void store_pending_iv(void)
{
	struct iv_val iv;
	int err;

	iv.iv_index = bt_mesh.iv_index;
	iv.iv_update = atomic_test_bit(bt_mesh.flags, BT_MESH_IVU_IN_PROGRESS);
	iv.iv_duration = bt_mesh.ivu_duration;

	err = settings_save_one(NV_MESH_IV, (const u8_t *)&iv, sizeof(iv));

	if (err) {
		BT_ERR("Failed to store IV value");
	} else {
		BT_DBG("Stored IV value");
	}
}

void bt_mesh_store_iv(bool only_duration)
{
	schedule_store(BT_MESH_IV_PENDING);

	if (!only_duration) {
		/* Always update Seq whenever IV changes */
		schedule_store(BT_MESH_SEQ_PENDING);
	}
}

static void store_pending_seq(void)
{
	struct seq_val seq;
	int err;

	seq.val[0] = bt_mesh.seq;
	seq.val[1] = bt_mesh.seq >> 8;
	seq.val[2] = bt_mesh.seq >> 16;

	err = settings_save_one(NV_MESH_SEQ, (const u8_t *)&seq, sizeof(seq));

	if (err) {
		BT_ERR("Failed to stor Seq value");
	} else {
		BT_DBG("Stored Seq value");
	}
}

void bt_mesh_store_seq(void)
{
	if (CONFIG_BT_MESH_SEQ_STORE_RATE &&
	    (bt_mesh.seq % CONFIG_BT_MESH_SEQ_STORE_RATE)) {
		return;
	}

	schedule_store(BT_MESH_SEQ_PENDING);
}


#if defined(BFLB_BLE)
static void store_rpl(struct bt_mesh_rpl *entry)
{
	bt_settings_set_bin(NV_MESH_RPL, (const u8_t *)bt_mesh.rpl, sizeof(bt_mesh.rpl));
}

static void clear_rpl(void)
{
	int err;

	err = ef_del_env(NV_MESH_RPL);

	if (err) {
		BT_ERR("Failed to clear RPL err(%d)", err);
	} else {
		BT_DBG("Cleared RPL");
	}

	(void)memset(bt_mesh.rpl, 0, sizeof(bt_mesh.rpl));
}
#else
static void store_rpl(struct bt_mesh_rpl *entry)
{
	struct rpl_val rpl;
	char path[18];
	int err;

	BT_DBG("src 0x%04x seq 0x%06x old_iv %u", entry->src, entry->seq,
	       entry->old_iv);

	rpl.seq = entry->seq;
	rpl.old_iv = entry->old_iv;

	snprintk(path, sizeof(path), "bt/mesh/RPL/%x", entry->src);

	err = settings_save_one(path, &rpl, sizeof(rpl));

	if (err) {
		BT_ERR("Failed to store RPL %s value", log_strdup(path));
	} else {
		BT_DBG("Stored RPL %s value", log_strdup(path));
	}
}

static void clear_rpl(void)
{
	int i, err;

	BT_DBG("");

	for (i = 0; i < ARRAY_SIZE(bt_mesh.rpl); i++) {
		struct bt_mesh_rpl *rpl = &bt_mesh.rpl[i];
		char path[18];

		if (!rpl->src) {
			continue;
		}

		snprintk(path, sizeof(path), "bt/mesh/RPL/%x", rpl->src);

		err = settings_delete(path);

		if (err) {
			BT_ERR("Failed to clear RPL");
		} else {
			BT_DBG("Cleared RPL");
		}

		(void)memset(rpl, 0, sizeof(*rpl));
	}
}
#endif

static void store_pending_rpl(void)
{
	int i;

	BT_DBG("");

	for (i = 0; i < ARRAY_SIZE(bt_mesh.rpl); i++) {
		struct bt_mesh_rpl *rpl = &bt_mesh.rpl[i];

		if (rpl->store) {
			rpl->store = false;
			store_rpl(rpl);
		}
	}
}

static void store_pending_hb_pub(void)
{
	struct bt_mesh_hb_pub *pub = bt_mesh_hb_pub_get();
	struct hb_pub_val val;
	int err;

	if (!pub) {
		return;
	}

	if (pub->dst == BT_MESH_ADDR_UNASSIGNED) {
		err = settings_delete(NV_MESH_HBPUB);
	} else {
		val.indefinite = (pub->count == 0xffff);
		val.dst = pub->dst;
		val.period = pub->period;
		val.ttl = pub->ttl;
		val.feat = pub->feat;
		val.net_idx = pub->net_idx;
        
		err = settings_save_one(NV_MESH_HBPUB, (const u8_t *)&val, sizeof(val));
	}

	if (err) {
		BT_ERR("Failed to store Heartbeat Publication");
	} else {
		BT_DBG("Stored Heartbeat Publication");
	}
}

static void store_pending_cfg(void)
{
	struct bt_mesh_cfg_srv *cfg = bt_mesh_cfg_get();
	struct cfg_val val;
	int err;

	if (!cfg) {
		return;
	}

	val.net_transmit = cfg->net_transmit;
	val.relay = cfg->relay;
	val.relay_retransmit = cfg->relay_retransmit;
	val.beacon = cfg->beacon;
	val.gatt_proxy = cfg->gatt_proxy;
	val.frnd = cfg->frnd;
	val.default_ttl = cfg->default_ttl;

	err = settings_save_one(NV_MESH_CFG, (const u8_t *)&val, sizeof(val));

	if (err) {
		BT_ERR("Failed to store configuration value");
	} else {
		BT_DBG("Stored configuration value");
        #if !defined(BFLB_BLE)
		BT_HEXDUMP_DBG(&val, sizeof(val), "raw value");
        #endif
	}
}

static void clear_cfg(void)
{
	int err;

	err = settings_delete(NV_MESH_CFG);

	if (err) {
		BT_ERR("Failed to clear configuration err(%d)", err);
	} else {
		BT_DBG("Cleared configuration");
	}
}

static void clear_app_key(u16_t app_idx)
{
	char path[20];
	int err;
    #if defined(BFLB_BLE)
    bool del = true;
    #endif

	snprintk(path, sizeof(path), "bt/mesh/AppKey/%x", app_idx);
	err = settings_delete(path);
	if (err) {
		BT_ERR("Failed to clear AppKeyIndex 0x%03x", app_idx);
	} else {
		BT_DBG("Cleared AppKeyIndex 0x%03x", app_idx);
	}

    #if defined(BFLB_BLE)
    for(int i = 0; i < CONFIG_BT_MESH_APP_KEY_COUNT; i++){
        if(mesh_appkey_idx[i] == app_idx){
            mesh_appkey_idx[i] = BT_MESH_KEY_UNUSED;
        }else if(mesh_appkey_idx[i] != BT_MESH_KEY_UNUSED){
            del = false;    
        }
    }

    if(del){
        err = settings_delete("bt/mesh/AppKeyIndex");
        if (err)
		    BT_ERR("Failed to delte AppKeyIndex after clear app key");
    }else{
        err = settings_save_one("bt/mesh/AppKeyIndex", (const u8_t *)mesh_netkey_idx, sizeof(mesh_netkey_idx));
        if (err)
		    BT_ERR("Failed to store AppKeyIndex after clear app key");
    }
    #endif
}

static void clear_net_key(u16_t net_idx)
{
	char path[20];
	int err;
    #if defined(BFLB_BLE)
    bool del = true;
    #endif

	BT_DBG("NetKeyIndex 0x%03x", net_idx);
	
    snprintk(path, sizeof(path), "bt/mesh/NetKey/%x", net_idx);

	err = settings_delete(path);

	if (err) {
		BT_ERR("Failed to clear NetKeyIndex 0x%03x", net_idx);
	} else {
		BT_DBG("Cleared NetKeyIndex 0x%03x", net_idx);
	}

    #if defined(BFLB_BLE)
    for(int i = 0; i < CONFIG_BT_MESH_SUBNET_COUNT; i++){
        if(mesh_netkey_idx[i] == net_idx){
            mesh_netkey_idx[i] = BT_MESH_KEY_UNUSED;
        }else if(mesh_netkey_idx[i] != BT_MESH_KEY_UNUSED){
            del = false;    
        }
    }

    if(del){
        err = settings_delete("bt/mesh/NetKeyIndex");
        if (err)
		    BT_ERR("Failed to delte NetKeyIndex after clear net key");
    }else{
        err = settings_save_one("bt/mesh/NetKeyIndex", (const u8_t *)mesh_netkey_idx, sizeof(mesh_netkey_idx));
        if (err)
		    BT_ERR("Failed to store NetKeyIndex after clear net key");
    }
    #endif
}

static void store_net_key(struct bt_mesh_subnet *sub)
{
	struct net_key_val key;
	char path[20];
	int err;
    #if defined(BFLB_BLE)
    u8_t empty_entry = 0xff;
    #endif

	BT_DBG("NetKeyIndex 0x%03x NetKey %s", sub->net_idx,
	       bt_hex(sub->keys[0].net, 16));

	memcpy(&key.val[0], sub->keys[0].net, 16);
	memcpy(&key.val[1], sub->keys[1].net, 16);
	key.kr_flag = sub->kr_flag;
	key.kr_phase = sub->kr_phase;

	snprintk(path, sizeof(path), "bt/mesh/NetKey/%x", sub->net_idx);
    err = settings_save_one(path, (const u8_t *)&key, sizeof(key));

	if (err) {
		BT_ERR("Failed to store NetKey value");
	} else {
		BT_DBG("Stored NetKey value");
	}

    #if defined(BFLB_BLE)
    for(int i = 0; i < CONFIG_BT_MESH_SUBNET_COUNT; i++){
        if(mesh_netkey_idx[i] == sub->net_idx)
            return;
        else if(empty_entry == 0xff && mesh_netkey_idx[i] == BT_MESH_KEY_UNUSED)
            empty_entry= i;    
    }
    
    if(empty_entry != 0xff){
        mesh_netkey_idx[empty_entry] = sub->net_idx;
        err = settings_save_one("bt/mesh/NetKeyIndex", (const u8_t *)mesh_netkey_idx, sizeof(mesh_netkey_idx));
        if(err){
            BT_ERR("Failed to store bt/mesh/NetKeyIndex");
        }
    }else{
        BT_ERR("Failed to store NetKey index %d", sub->net_idx);
    }
    #endif
}

static void store_app_key(struct bt_mesh_app_key *app)
{
	struct app_key_val key;
	char path[20];
	int err;
    #if defined(BFLB_BLE)
    u8_t empty_entry = 0xff;
    #endif

	key.net_idx = app->net_idx;
	key.updated = app->updated;
	memcpy(key.val[0], app->keys[0].val, 16);
	memcpy(key.val[1], app->keys[1].val, 16);

	snprintk(path, sizeof(path), "bt/mesh/AppKey/%x", app->app_idx);

	err = settings_save_one(path, (const u8_t *)&key, sizeof(key));
	if (err) {
		BT_ERR("Failed to store AppKey %s value", path);
	} else {
		BT_DBG("Stored AppKey %s value", path);
	}

    #if defined(BFLB_BLE)
    for(int i = 0; i < CONFIG_BT_MESH_APP_KEY_COUNT; i++){
        if(mesh_appkey_idx[i] == app->app_idx)
            return;
        else if(empty_entry == 0xff && mesh_appkey_idx[i] == BT_MESH_KEY_UNUSED)
            empty_entry= i;    
    }

    if(empty_entry != 0xff){
        mesh_appkey_idx[empty_entry] = app->app_idx;
        err = settings_save_one("bt/mesh/AppKeyIndex", (const u8_t *)mesh_appkey_idx, sizeof(mesh_appkey_idx));
        if(err){
            BT_ERR("Failed to store bt/mesh/AppKeyIndex");
        }
    }else{
        BT_ERR("Failed to store AppKey index %d", app->app_idx);
    }
    #endif
}

static void store_pending_keys(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(key_updates); i++) {
		struct key_update *update = &key_updates[i];

		if (!update->valid) {
			continue;
		}

		if (update->clear) {
			if (update->app_key) {
				clear_app_key(update->key_idx);
			} else {
				clear_net_key(update->key_idx);
			}
		} else {
			if (update->app_key) {
				struct bt_mesh_app_key *key;

				key = bt_mesh_app_key_find(update->key_idx);
				if (key) {
					store_app_key(key);
				} else {
					BT_WARN("AppKeyIndex 0x%03x not found",
					       update->key_idx);
				}

			} else {
				struct bt_mesh_subnet *sub;

				sub = bt_mesh_subnet_get(update->key_idx);
				if (sub) {
					store_net_key(sub);
				} else {
					BT_WARN("NetKeyIndex 0x%03x not found",
					       update->key_idx);
				}
			}
		}

		update->valid = 0U;
	}
}

static void encode_mod_path(struct bt_mesh_model *mod, bool vnd,
			    const char *key, char *path, size_t path_len)
{
	u16_t mod_key = (((u16_t)mod->elem_idx << 8) | mod->mod_idx);

	if (vnd) {
		snprintk(path, path_len, "bt/mesh/v/%x/%s", mod_key, key);
	} else {
		snprintk(path, path_len, "bt/mesh/s/%x/%s", mod_key, key);
	}
}

static void store_pending_mod_bind(struct bt_mesh_model *mod, bool vnd)
{
	u16_t keys[CONFIG_BT_MESH_MODEL_KEY_COUNT];
	char path[20];
	int i, count, err;

    #if defined(BFLB_BLE)
    memset(keys, 0, sizeof(keys));
    #endif
    
	for (i = 0, count = 0; i < ARRAY_SIZE(mod->keys); i++) {
		if (mod->keys[i] != BT_MESH_KEY_UNUSED) {
			keys[count++] = mod->keys[i];
			BT_DBG("model key 0x%04x", mod->keys[i]);
		}
	}

	encode_mod_path(mod, vnd, "bind", path, sizeof(path));

	if (count) {
        #if defined(BFLB_BLE)
        err = bt_settings_set_bin((const char *)path, (const u8_t *)keys, sizeof(keys)); 
        #else
		err = settings_save_one(path, keys, count * sizeof(keys[0]));
        #endif
	} else {
	    #if defined(BFLB_BLE)
        err = ef_del_env((const char *)path);
        #else
		err = settings_delete(path);
        #endif
	}

	if (err) {
        #if defined(BFLB_BLE)
        BT_ERR("Failed to store mod bind");
        #else
		BT_ERR("Failed to store %s value", log_strdup(path));
        #endif
	} else {
	    #if defined(BFLB_BLE)
        BT_DBG("Stored mod bind");
        #else
		BT_DBG("Stored %s value", log_strdup(path));
        #endif
	}
}

static void store_pending_mod_sub(struct bt_mesh_model *mod, bool vnd)
{
	u16_t groups[CONFIG_BT_MESH_MODEL_GROUP_COUNT];
	char path[20];
	int i, count, err;

    #if defined(BFLB_BLE)
    memset(groups, 0, sizeof(groups));
    #endif
    
	for (i = 0, count = 0; i < ARRAY_SIZE(mod->groups); i++) {
		if (mod->groups[i] != BT_MESH_ADDR_UNASSIGNED) {
			groups[count++] = mod->groups[i];
		}
	}

	encode_mod_path(mod, vnd, "sub", path, sizeof(path));

	if (count) {
        #if defined(BFLB_BLE)
        err = bt_settings_set_bin((const char *)path, (const u8_t *)groups, sizeof(groups)); 
        #else
		err = settings_save_one(path, groups,
					count * sizeof(groups[0]));
        #endif
	} else {
	    #if defined(BFLB_BLE)
        err = ef_del_env((const char *)path);
        #else
		err = settings_delete(path);
        #endif
	}

	if (err) {
        #if defined(BFLB_BLE)
        BT_ERR("Failed to store mod sub");
        #else
		BT_ERR("Failed to store %s value", log_strdup(path));
        #endif
	} else {
	    #if defined(BFLB_BLE)
        BT_DBG("Stored mod sub");
        #else
		BT_DBG("Stored %s value", log_strdup(path));
        #endif
	}
}

static void store_pending_mod_pub(struct bt_mesh_model *mod, bool vnd)
{
	struct mod_pub_val pub;
	char path[20];
	int err;
    
	encode_mod_path(mod, vnd, "pub", path, sizeof(path));

	if (!mod->pub || mod->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
        #if defined(BFLB_BLE)
        err = ef_del_env((const char *)path);
        #else
		err = settings_delete(path);
        #endif
	} else {
		pub.addr = mod->pub->addr;
		pub.key = mod->pub->key;
		pub.ttl = mod->pub->ttl;
		pub.retransmit = mod->pub->retransmit;
		pub.period = mod->pub->period;
		pub.period_div = mod->pub->period_div;
		pub.cred = mod->pub->cred;

        #if defined(BFLB_BLE)
        err = bt_settings_set_bin((const char *)path, (const u8_t *)&pub, sizeof(pub));
        #else
		err = settings_save_one(path, &pub, sizeof(pub));
        #endif
	}

	if (err) {
        #if defined(BFLB_BLE)
        BT_ERR("Failed to store mod sub");
        #else
		BT_ERR("Failed to store %s value", log_strdup(path));
        #endif
	} else {
	    #if defined(BFLB_BLE)
        BT_DBG("Stored %s value");
        #else
		BT_DBG("Stored %s value", log_strdup(path));
        #endif
	}
}

static void store_pending_mod(struct bt_mesh_model *mod,
			      struct bt_mesh_elem *elem, bool vnd,
			      bool primary, void *user_data)
{
	if (!mod->flags) {
		return;
	}

	if (mod->flags & BT_MESH_MOD_BIND_PENDING) {
		mod->flags &= ~BT_MESH_MOD_BIND_PENDING;
		store_pending_mod_bind(mod, vnd);
	}

	if (mod->flags & BT_MESH_MOD_SUB_PENDING) {
		mod->flags &= ~BT_MESH_MOD_SUB_PENDING;
		store_pending_mod_sub(mod, vnd);
	}

	if (mod->flags & BT_MESH_MOD_PUB_PENDING) {
		mod->flags &= ~BT_MESH_MOD_PUB_PENDING;
		store_pending_mod_pub(mod, vnd);
	}
}

static void store_pending(struct k_work *work)
{
	BT_DBG("");

	if (atomic_test_and_clear_bit(bt_mesh.flags, BT_MESH_RPL_PENDING)) {
		if (atomic_test_bit(bt_mesh.flags, BT_MESH_VALID)) {
			store_pending_rpl();
		} else {
			clear_rpl();
		}
	}

	if (atomic_test_and_clear_bit(bt_mesh.flags, BT_MESH_KEYS_PENDING)) {
		store_pending_keys();
	}

	if (atomic_test_and_clear_bit(bt_mesh.flags, BT_MESH_NET_PENDING)) {
		if (atomic_test_bit(bt_mesh.flags, BT_MESH_VALID)) {
			store_pending_net();
		} else {
			clear_net();
		}
	}

	if (atomic_test_and_clear_bit(bt_mesh.flags, BT_MESH_IV_PENDING)) {
		if (atomic_test_bit(bt_mesh.flags, BT_MESH_VALID)) {
			store_pending_iv();
		} else {
			clear_iv();
		}
	}

	if (atomic_test_and_clear_bit(bt_mesh.flags, BT_MESH_SEQ_PENDING)) {
		store_pending_seq();
	}

	if (atomic_test_and_clear_bit(bt_mesh.flags, BT_MESH_HB_PUB_PENDING)) {
		store_pending_hb_pub();
	}

	if (atomic_test_and_clear_bit(bt_mesh.flags, BT_MESH_CFG_PENDING)) {
		if (atomic_test_bit(bt_mesh.flags, BT_MESH_VALID)) {
			store_pending_cfg();
		} else {
			clear_cfg();
		}
	}

	if (atomic_test_and_clear_bit(bt_mesh.flags, BT_MESH_MOD_PENDING)) {
		bt_mesh_model_foreach(store_pending_mod, NULL);
	}
}

void bt_mesh_store_rpl(struct bt_mesh_rpl *entry)
{
	entry->store = true;
	schedule_store(BT_MESH_RPL_PENDING);
}

static struct key_update *key_update_find(bool app_key, u16_t key_idx,
					  struct key_update **free_slot)
{
	struct key_update *match;
	int i;

	match = NULL;
	*free_slot = NULL;

	for (i = 0; i < ARRAY_SIZE(key_updates); i++) {
		struct key_update *update = &key_updates[i];

		if (!update->valid) {
			*free_slot = update;
			continue;
		}

		if (update->app_key != app_key) {
			continue;
		}

		if (update->key_idx == key_idx) {
			match = update;
		}
	}

	return match;
}

void bt_mesh_store_subnet(struct bt_mesh_subnet *sub)
{
	struct key_update *update, *free_slot;

	BT_DBG("NetKeyIndex 0x%03x", sub->net_idx);

	update = key_update_find(false, sub->net_idx, &free_slot);
	if (update) {
		update->clear = 0U;
		schedule_store(BT_MESH_KEYS_PENDING);
		return;
	}

	if (!free_slot) {
		store_net_key(sub);
		return;
	}

	free_slot->valid = 1U;
	free_slot->key_idx = sub->net_idx;
	free_slot->app_key = 0U;
	free_slot->clear = 0U;

	schedule_store(BT_MESH_KEYS_PENDING);
}

void bt_mesh_store_app_key(struct bt_mesh_app_key *key)
{
	struct key_update *update, *free_slot;

	BT_DBG("AppKeyIndex 0x%03x", key->app_idx);

	update = key_update_find(true, key->app_idx, &free_slot);
	if (update) {
		update->clear = 0U;
		schedule_store(BT_MESH_KEYS_PENDING);
		return;
	}

	if (!free_slot) {
		store_app_key(key);
		return;
	}

	free_slot->valid = 1U;
	free_slot->key_idx = key->app_idx;
	free_slot->app_key = 1U;
	free_slot->clear = 0U;

	schedule_store(BT_MESH_KEYS_PENDING);
}

void bt_mesh_store_hb_pub(void)
{
	schedule_store(BT_MESH_HB_PUB_PENDING);
}

void bt_mesh_store_cfg(void)
{
	schedule_store(BT_MESH_CFG_PENDING);
}

void bt_mesh_clear_net(void)
{
	schedule_store(BT_MESH_NET_PENDING);
	schedule_store(BT_MESH_IV_PENDING);
	schedule_store(BT_MESH_CFG_PENDING);
}

void bt_mesh_clear_subnet(struct bt_mesh_subnet *sub)
{
	struct key_update *update, *free_slot;

	BT_DBG("NetKeyIndex 0x%03x", sub->net_idx);

	update = key_update_find(false, sub->net_idx, &free_slot);
	if (update) {
		update->clear = 1U;
		schedule_store(BT_MESH_KEYS_PENDING);
		return;
	}

	if (!free_slot) {
		clear_net_key(sub->net_idx);
		return;
	}

	free_slot->valid = 1U;
	free_slot->key_idx = sub->net_idx;
	free_slot->app_key = 0U;
	free_slot->clear = 1U;

	schedule_store(BT_MESH_KEYS_PENDING);
}

void bt_mesh_clear_app_key(struct bt_mesh_app_key *key)
{
	struct key_update *update, *free_slot;

	BT_DBG("AppKeyIndex 0x%03x", key->app_idx);

	update = key_update_find(true, key->app_idx, &free_slot);
	if (update) {
		update->clear = 1U;
		schedule_store(BT_MESH_KEYS_PENDING);
		return;
	}

	if (!free_slot) {
		clear_app_key(key->app_idx);
		return;
	}

	free_slot->valid = 1U;
	free_slot->key_idx = key->app_idx;
	free_slot->app_key = 1U;
	free_slot->clear = 1U;

	schedule_store(BT_MESH_KEYS_PENDING);
}

void bt_mesh_clear_rpl(void)
{
	schedule_store(BT_MESH_RPL_PENDING);
}

void bt_mesh_store_mod_bind(struct bt_mesh_model *mod)
{
	mod->flags |= BT_MESH_MOD_BIND_PENDING;
	schedule_store(BT_MESH_MOD_PENDING);
}

void bt_mesh_store_mod_sub(struct bt_mesh_model *mod)
{
	mod->flags |= BT_MESH_MOD_SUB_PENDING;
	schedule_store(BT_MESH_MOD_PENDING);
}

void bt_mesh_store_mod_pub(struct bt_mesh_model *mod)
{
	mod->flags |= BT_MESH_MOD_PUB_PENDING;
	schedule_store(BT_MESH_MOD_PENDING);
}

void bt_mesh_settings_init(void)
{
    int i;
    for(i = 0; i < ARRAY_SIZE(mesh_netkey_idx); i++){
        mesh_netkey_idx[i] = BT_MESH_KEY_UNUSED;
    }

    for(i = 0; i < ARRAY_SIZE(mesh_appkey_idx); i++){
        mesh_appkey_idx[i] = BT_MESH_KEY_UNUSED;
    }
    
	k_delayed_work_init(&pending_store, store_pending);
}
