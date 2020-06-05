/** @file
 *  @brief wifiprov  Service
 */
#include "blsync_ble.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/types.h>
#include <bluetooth.h>
#include <conn.h>
#include <gatt.h>
#include <uuid.h>
#include <blog.h>
#include <stdio.h>
#include "cJSON.h"

static bl_ble_sync_t *gp_index = NULL;

static void wifiprov_ccc_cfg_changed(const struct bt_gatt_attr *attr,u16_t value);

static ssize_t write_data(struct bt_conn *conn,
                   const struct bt_gatt_attr *attr, const void *buf,
                   u16_t len, u16_t offset, u8_t flags);

static ssize_t read_data(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, void *buf,
                         u16_t len, u16_t offset);

static void scan_complete_cb(void *param)
{
    blesync_wifi_item_t *item = param;

    gp_index->scaning = 0;

    if (gp_index->w_ap_item >= WIFI_SCAN_ITEMS_MAX) {
        return;
    }
    memcpy(&gp_index->ap_item[gp_index->w_ap_item], item, sizeof(blesync_wifi_item_t));
    gp_index->w_ap_item++;
}

static void wifi_state_get_cb(void *p_arg)
{
    struct wifi_state *p_state = (struct wifi_state *)p_arg;
    gp_index->state.state = p_state->state;
    strcpy(gp_index->state.ip, p_state->ip);
    strcpy(gp_index->state.mask, p_state->mask);
    strcpy(gp_index->state.gw, p_state->gw);
    xSemaphoreGive(gp_index->xSemaphore);
}

static void wifiprov_ccc_cfg_changed(const struct bt_gatt_attr *attr,u16_t value)
{

}

static ssize_t read_data(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, void *buf,
                         u16_t len, u16_t offset)
{
    ssize_t ret;

    if (gp_index->rbuf_len == 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_PDU);
    }

    blog_info("read length %d\r\n", gp_index->rbuf_len);
    pro_trans_ack(gp_index->pro_handle);

    ret = bt_gatt_attr_read(conn, attr, buf, len, offset,
                            gp_index->attr_read_buf, gp_index->rbuf_len);
    gp_index->rbuf_len = 0;
    return ret;
}

static ssize_t write_data(struct bt_conn *conn,
                   const struct bt_gatt_attr *attr, const void *buf,
                   u16_t len, u16_t offset, u8_t flags)
{
    gp_index->p_cur_conn = conn;

    blog_info("write length %d\r\n", len);
    blog_info_hexdump("data", (uint8_t *)buf, len);

    gp_index->send_buf.len = len;
    memcpy(gp_index->send_buf.buf, buf, len);
    xQueueSend(gp_index->xQueue1, &gp_index->send_buf, 0);

    return len;
}

static int __ble_bytes_send(void *p_drv, const void *buf, size_t bytes)
{
    if (gp_index->p_cur_conn == NULL) {
        return -1;
    }
    if (bytes >= BLE_PROV_BUF_SIZE) {
        return -1;
    }
    memcpy(gp_index->attr_read_buf, buf, bytes);
    gp_index->rbuf_len = bytes;

    blog_info(" ble send size %d\r\n", bytes);

    blog_info_hexdump("data", (uint8_t *)buf, bytes);
    printf("\r\n");

    return 0;
}

static int __recv_event(void *p_drv, struct pro_event *p_event)
{
    char bssid[20] = {0};
    blesync_wifi_item_t *p_item;
    cJSON *p_root;
    char *json_str;

    blog_info("event type %d, id %d\r\n", p_event->type, p_event->event_id);

    if (p_event->type == PROTOCOL_EVENT_DATA) {

        switch (p_event->event_id) {
        case DATA_STA_WIFI_BSSID:
            break;
        case DATA_STA_WIFI_SSID:
            memset(gp_index->conn_info.ssid, 0, sizeof(gp_index->conn_info.ssid));
            memcpy(gp_index->conn_info.ssid, p_event->p_buf, p_event->length);
            break;
        case DATA_STA_WIFI_PASSWORD:
            memset(gp_index->conn_info.pask, 0, sizeof(gp_index->conn_info.pask));
            memcpy(gp_index->conn_info.pask, p_event->p_buf, p_event->length);
            break;
        default:
            return PRO_ERROR;
        }

    } else if (p_event->type == PROTOCOL_EVENT_CMD) {
        
        switch (p_event->event_id) {
          case CMD_CONNECT_WIFI:
              if(gp_index->wifi_func->local_connect_remote_ap) {
                  gp_index->wifi_func->local_connect_remote_ap(&gp_index->conn_info);
              }
              break;
          case CMD_DISCONNECT_WIFI:
              if(gp_index->wifi_func->local_disconnect_remote_ap) {
                  gp_index->wifi_func->local_disconnect_remote_ap();
              }
              break;
          case CMD_WIFI_SCAN:
              if(gp_index->wifi_func->local_wifi_scan) {
                  gp_index->r_ap_item = 0;
                  gp_index->w_ap_item = 0;
                  gp_index->scaning = 1;
                  gp_index->wifi_func->local_wifi_scan(scan_complete_cb);
              }
              break;
          case CMD_WIFI_DATA_GET:

              if (gp_index->w_ap_item == 0) {
                  return PRO_NOT_READY;
              }

              if (gp_index->r_ap_item == gp_index->w_ap_item) {
                  gp_index->r_ap_item = 0;
                  gp_index->w_ap_item = 0;
                  blog_info("wifi list end\r\n");
                  break;
              }
              p_item = &gp_index->ap_item[gp_index->r_ap_item];
              blog_info("item_nums %d\r\n", gp_index->r_ap_item++);
              p_root = cJSON_CreateObject();

              sprintf(bssid, "%02X:%02X:%02X:%02X:%02X:%02X",
                      p_item->bssid[0],
                      p_item->bssid[1],
                      p_item->bssid[2],
                      p_item->bssid[3],
                      p_item->bssid[4],
                      p_item->bssid[5]);

              cJSON_AddNumberToObject(p_root, "chan", p_item->channel);
              cJSON_AddStringToObject(p_root, "bssid", bssid);
              cJSON_AddNumberToObject(p_root, "rssi", p_item->rssi);
              cJSON_AddStringToObject(p_root, "ssid", p_item->ssid);

              //0: open; 1:wep; 2:WPA/WPA2 - PSK; 3: WPA/WPA2 - Enterprise; 0xFF: unknown
              cJSON_AddNumberToObject(p_root, "auth", p_item->auth);
              json_str = cJSON_Print(p_root);

              pro_trans_layer_ack_read(gp_index->pro_handle, json_str, strlen(json_str));
              cJSON_Delete(p_root);
              vPortFree(json_str);

              break;
          case CMD_PROV_STOP:
              if (gp_index->complete_cb) {
                  gp_index->complete_cb(gp_index->p_arg);
              }
              break;
          case CMD_WIFI_STATE_GET:
              if(gp_index->wifi_func->local_wifi_state_get) {
                  gp_index->wifi_func->local_wifi_state_get(wifi_state_get_cb);
                  if (xSemaphoreTake(gp_index->xSemaphore,
                                     (TickType_t)PRO_CONFIG_TIMEOUT) != pdTRUE) {
                      return PRO_ERROR;
                  }
                  p_root = cJSON_CreateObject();
                  cJSON_AddNumberToObject(p_root, "state", gp_index->state.state);
                  cJSON_AddStringToObject(p_root, "ip", gp_index->state.ip);
                  cJSON_AddStringToObject(p_root, "gw", gp_index->state.gw);
                  cJSON_AddStringToObject(p_root, "mask", gp_index->state.mask);
                  json_str = cJSON_Print(p_root);

                  pro_trans_layer_ack_read(gp_index->pro_handle, json_str, strlen(json_str));
                  cJSON_Delete(p_root);
                  vPortFree(json_str);
              }
              break;
          case CMD_GET_VERSION:
              pro_trans_layer_ack_read(gp_index->pro_handle,
                                       BLSYNC_BLE_VERSION,
                                       sizeof(BLSYNC_BLE_VERSION));
              break;
          default:
              return PRO_ERROR;
          }
    }

    return PRO_OK;
}

const static struct pro_func ble_gatt_func = {
    __ble_bytes_send,
    __recv_event,
};

const static cJSON_Hooks __g_cjson_hooks = {
    pvPortMalloc,
    vPortFree,
};

static void __bl_ble_sync_task (void *p_arg)
{
    uint16_t protocol_mtu = 0;
    struct queue_buf recv_buf;
    bl_ble_sync_t *index = (bl_ble_sync_t *)p_arg;

    cJSON_InitHooks((cJSON_Hooks *)&__g_cjson_hooks);
    index->pro_handle = pro_trans_init(&index->pro_dev, &ble_gatt_func, NULL);
    index->task_runing = 1;

    index->xSemaphore = xSemaphoreCreateBinaryStatic(&index->xSemaphoreBuffer);
    index->xQueue1 = xQueueCreateStatic(BLE_PROV_QUEUE_NUMS,
                                        sizeof(index->buf[0]),
                                        (uint8_t *)index->buf,
                                        &index->xQueueBuffer);

    while (!index->stop_flag) {

        if (xQueueReceive(index->xQueue1,
                          &recv_buf,
                          (TickType_t)PRO_CONFIG_TIMEOUT) == pdTRUE) {
            if (index->p_cur_conn) {
                protocol_mtu = bt_gatt_get_mtu(index->p_cur_conn) - 3;
            }

            if (pro_trans_read(index->pro_handle, recv_buf.buf,
                               recv_buf.len, protocol_mtu) != PRO_OK) {
                blog_info("protocol analyze failed\r\n");
            }

        } else {
            pro_trans_reset(index->pro_handle);
        }
    }
    index->task_runing = 0;
    vTaskDelete(index->task_handle);
}

static struct bt_gatt_attr attrs[]= {
    BT_GATT_PRIMARY_SERVICE(BT_UUID_WIFI_PROV),
    BT_GATT_CHARACTERISTIC(BT_UUID_WIFIPROV_READ,
                                BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                BT_GATT_PERM_READ ,
                                read_data,
                                NULL,
                                NULL),

    BT_GATT_CCC(wifiprov_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_WIFIPROV_WRITE,
                                BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                BT_GATT_PERM_WRITE,
                                NULL,
                                write_data,
                                NULL),
};

const static struct bt_gatt_service wifiprov_server = BT_GATT_SERVICE(attrs);

int bl_ble_sync_start(bl_ble_sync_t *index,
                       struct blesync_wifi_func *func,
                       pfn_complete_cb_t cb,
                       void *cb_arg)
{
    if (index == NULL || func == NULL) {
        blog_info("ble sync init failed\r\n");
        return -1;
    }

    memset(index, 0, sizeof(bl_ble_sync_t));
    gp_index = index;
    index->wifi_func = func;
    index->complete_cb = cb;
    index->p_arg = cb_arg;
    gp_index->scaning = 0;
    index->task_runing = 0;
    index->stop_flag = 0;

    bt_gatt_service_register((struct bt_gatt_service *)&wifiprov_server);

    index->task_handle = xTaskCreateStatic(__bl_ble_sync_task,
                                    (char*)"pro",
                                    BLE_PROV_TASK_STACK_SIZE,
                                    index,
                                    10,
                                    index->stack,
                                    &index->task);
    return 0;
}

int bl_ble_sync_stop(bl_ble_sync_t *index)
{
    if (index == NULL) {
        return -1;
    }

    bt_gatt_service_unregister((struct bt_gatt_service *)&wifiprov_server);
    index->stop_flag = 1;

    if (xTaskGetCurrentTaskHandle() == gp_index->task_handle) {
        while(gp_index->scaning == 1) {
            vTaskDelay(10);
        }
        vTaskDelete(index->task_handle);
    } else {
        while((gp_index->scaning == 1) || (index->task_runing == 1)) {
            vTaskDelay(10);
        }
    }
    return 0;
}

