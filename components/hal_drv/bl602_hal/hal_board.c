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
#include <stdint.h>
#include <stdio.h>
#include <bl_efuse.h>
#include <bl_wifi.h>
#include <hal_boot2.h>
#include <hal_sys.h>

#include <libfdt.h>

#include <utils_log.h>

#define BL_FDT32_TO_U8(addr, byte_offset)   ((uint8_t)fdt32_to_cpu(*(uint32_t *)((uint8_t *)addr + byte_offset)))
#define BL_FDT32_TO_U16(addr, byte_offset)  ((uint16_t)fdt32_to_cpu(*(uint32_t *)((uint8_t *)addr + byte_offset)))
#define BL_FDT32_TO_U32(addr, byte_offset)  ((uint32_t)fdt32_to_cpu(*(uint32_t *)((uint8_t *)addr + byte_offset)))

static uint32_t factory_addr = 0;

#ifndef FEATURE_WIFI_DISABLE
#include <bl60x_fw_api.h>
static int update_mac_config_get_mac_from_dtb(const void *fdt, int offset1, uint8_t mac_addr[6])
{
    int lentmp;
    const uint8_t *addr_prop = 0;

    /* set sta_mac_addr ap_mac_addr */
    addr_prop = fdt_getprop(fdt, offset1, "sta_mac_addr", &lentmp);
    if (6 == lentmp) {

        memcpy(mac_addr, addr_prop, 6);
        log_info("sta_mac_addr :\r\n");
        log_buf(mac_addr, 6);

        bl_wifi_sta_mac_addr_set(mac_addr);
    } else {
        log_error("sta_mac_addr NULL.\r\n");
        return -1;
    }

    addr_prop = fdt_getprop(fdt, offset1, "ap_mac_addr", &lentmp);
    if (6 == lentmp) {

        memcpy(mac_addr, addr_prop, 6);
        log_info("ap_mac_addr :\r\n");
        log_buf(mac_addr, 6);

        bl_wifi_ap_mac_addr_set(mac_addr);
    } else {
        log_error("ap_mac_addr NULL.\r\n");
        return -1;
    }

    return 0;
}

static int update_mac_config_get_mac_from_efuse(uint8_t mac_addr[6])
{
    uint8_t result_or, result_and;

    bl_efuse_read_mac(mac_addr);
    result_or = mac_addr[0] | mac_addr[1] | mac_addr[2] | mac_addr[3] | mac_addr[4] | mac_addr[5];
    result_and = mac_addr[0] & mac_addr[1] & mac_addr[2] & mac_addr[3] & mac_addr[4] & mac_addr[5];

    if (0 == result_or || 1 == result_and) {
        /*all zero or one found in efuse*/
        return -1;
    }
    return 0;
}

static int update_mac_config_get_mac_from_factory(uint8_t mac_addr[6])
{
    uint8_t result_or, result_and;

    if (bl_efuse_read_mac_factory(mac_addr)) {
        return -1;
    }
    result_or = mac_addr[0] | mac_addr[1] | mac_addr[2] | mac_addr[3] | mac_addr[4] | mac_addr[5];
    result_and = mac_addr[0] & mac_addr[1] & mac_addr[2] & mac_addr[3] & mac_addr[4] & mac_addr[5];
    if (0 == result_or || 1 == result_and) {
        /*all zero or one found in efuse*/
        return -1;
    }
    return 0;
}

/*
 * Update MAC address according to order string
 * BFM:
 *  'B' for EFUSE built-in MAC address
 *  'F' for Flash built-in MAC address
 *  'M' for manufacutre configured EFUSE built-in MAC address
 * */
#define MAC_ORDER_ADDR_LEN_MAX      (3)
static void update_mac_config_with_order(const void *fdt, int offset1, const char *order)
{
    int i, set, len;
    uint8_t mac_addr[6];
    static const uint8_t mac_default[] = {0x18, 0xB9, 0x05, 0x88, 0x88, 0x88};

    set = 0;
    len = strlen(order);
    for (i = 0; i < MAC_ORDER_ADDR_LEN_MAX && i < len; i++) {
        switch (order[i]) {
            case 'B':
            {
                if (0 == update_mac_config_get_mac_from_efuse(mac_addr)) {
                    set = 1;
                    log_debug("get MAC from B ready\r\n");
                    goto break_scan;
                } else {
                    log_debug("get MAC from B failed\r\n");
                }
            }
            break;
            case 'F':
            {
                if (0 == update_mac_config_get_mac_from_dtb(fdt, offset1, mac_addr)) {
                    set = 1;
                    log_debug("get MAC from F ready\r\n");
                    goto break_scan;
                } else {
                    log_debug("get MAC from F failed\r\n");
                }
            }
            break;
            case 'M':
            {
                if (0 == update_mac_config_get_mac_from_factory(mac_addr)) {
                    set = 1;
                    log_debug("get MAC from M ready\r\n");
                    goto break_scan;
                } else {
                    log_debug("get MAC from M failed\r\n");
                }
            }
            break;
            default:
            {
                BL_ASSERT(0);
            }
        }
    }
break_scan:
    if (0 == set) {
        log_info("Using Default MAC address\r\n");
        memcpy(mac_addr, mac_default, 6);
    }
    //FIXME maybe we should set a different MAC address
    log_info("Set MAC addrress %02X:%02X:%02X:%02X:%02X:%02X\r\n",
            mac_addr[0],
            mac_addr[1],
            mac_addr[2],
            mac_addr[3],
            mac_addr[4],
            mac_addr[5]
    );
    bl_wifi_ap_mac_addr_set(mac_addr);
    bl_wifi_sta_mac_addr_set(mac_addr);
}

static void update_mac_config(const void *fdt, int offset1)
{
    int countindex = 0, lentmp = 0;
    const char *result = 0;
    char mac_mode[4];

    countindex = fdt_stringlist_count(fdt, offset1, "mode");
    if (1 == countindex) {
        result = fdt_stringlist_get(fdt, offset1, "mode", 0, &lentmp);
        log_info("MAC address mode length %d\r\n", lentmp);
        if (lentmp <= MAC_ORDER_ADDR_LEN_MAX) {
            memcpy(mac_mode, result, lentmp);
            mac_mode[3] = '\0';
            log_info("MAC address mode is %s\r\n", mac_mode);
            update_mac_config_with_order(fdt, offset1, mac_mode);
        }
    }
}


static int update_xtal_config_get_mac_from_factory(uint32_t capcode[5])
{
    uint8_t capcode_efuse = 0;

    if (bl_efuse_read_capcode(&capcode_efuse)) {
        return -1;
    }
    /*efuse only have one capcode entry, so we fill the left with hardcode*/
    capcode[0] = capcode_efuse;
    capcode[1] = capcode_efuse;
    capcode[2] = 1;
    capcode[3] = 60;
    capcode[4] = 60;

    return 0;
}

static int update_xtal_config_get_mac_from_dtb(const void *fdt, int offset1, uint32_t capcode[5])
{
    const uint8_t *addr_prop = 0;
    int lentmp = 0;

    addr_prop = fdt_getprop(fdt, offset1, "xtal", &lentmp);

    if (5*4 == lentmp) {
        log_info(
            "xtal dtb in DEC :%u %u %u %u %u\r\n",
            BL_FDT32_TO_U8(addr_prop, 4*0),
            BL_FDT32_TO_U8(addr_prop, 4*1),
            BL_FDT32_TO_U8(addr_prop, 4*2),
            BL_FDT32_TO_U8(addr_prop, 4*3),
            BL_FDT32_TO_U8(addr_prop, 4*4)
        );
        capcode[0] = BL_FDT32_TO_U8(addr_prop, 4*0);
        capcode[1] = BL_FDT32_TO_U8(addr_prop, 4*1);
        capcode[2] = BL_FDT32_TO_U8(addr_prop, 4*2);
        capcode[3] = BL_FDT32_TO_U8(addr_prop, 4*3);
        capcode[4] = BL_FDT32_TO_U8(addr_prop, 4*4);
    } else {
        log_error("xtal dtb NULL.");
        return -1;
    }
    return 0;
}

#define XTAL_ORDER_ADDR_LEN_MAX      (2)
static void update_xtal_config_with_order(const void *fdt, int offset1, const char *order)
{
    int i, set, len;
    uint32_t capcode[5];

    set = 0;
    len = strlen(order);
    for (i = 0; i < XTAL_ORDER_ADDR_LEN_MAX && i < len; i++) {
        switch (order[i]) {
            case 'F':
            {
                if (0 == update_xtal_config_get_mac_from_dtb(fdt, offset1, capcode)) {
                    set = 1;
                    log_debug("get xtal from F ready\r\n");
                    goto break_scan;
                } else {
                    log_debug("get xtal from F failed\r\n");
                }
            }
            break;
            case 'M':
            {
                if (0 == update_xtal_config_get_mac_from_factory(capcode)) {
                    set = 1;
                    log_debug("get xtal from M ready\r\n");
                    goto break_scan;
                } else {
                    log_debug("get xtal from M failed\r\n");
                }
            }
            break;
            default:
            {
                BL_ASSERT(0);
            }
        }
    }
break_scan:
    if (0 == set) {
        log_info("Using Default xtal\r\n");
        capcode[0] = 50;
        capcode[1] = 50;
        capcode[2] = 1;
        capcode[3] = 60;
        capcode[4] = 60;
    }
    hal_sys_capcode_update(capcode[0], capcode[1]);
}

static void update_xtal_config(const void *fdt, int offset1)
{
    int lentmp = 0, countindex;
    char xtal_mode[3];
    const char *result = 0;

    countindex = fdt_stringlist_count(fdt, offset1, "xtal_mode");
    if (1 == countindex) {
        result = fdt_stringlist_get(fdt, offset1, "xtal_mode", 0, &lentmp);
        log_info("xtal_mode length %d\r\n", lentmp);
        if (lentmp <= XTAL_ORDER_ADDR_LEN_MAX) {
            memcpy(xtal_mode, result, lentmp);
            xtal_mode[sizeof(xtal_mode) - 1] = '\0';
            log_info("xtal_mode is %s\r\n", xtal_mode);
            update_xtal_config_with_order(fdt, offset1, xtal_mode);
        }
    }
}

static int update_poweroffset_config_get_mac_from_dtb(const void *fdt, int offset1, int8_t poweroffset[14])
{
    int lentmp = 0, i;
    const uint8_t *addr_prop = 0;

#define PWR_OFFSET_BASE (10)
    addr_prop = fdt_getprop(fdt, offset1, "pwr_offset", &lentmp);
    if (14*4 == lentmp) {
        for (i = 0; i < 14; i++) {
            poweroffset[i] = BL_FDT32_TO_U32(addr_prop, 4*i);
        }
        log_info("pwr_offset from dtb:\r\n");
        log_buf(poweroffset, 14);
        for (i = 0; i < 14; i++) {
            poweroffset[i] -= PWR_OFFSET_BASE;
        }
        log_info("pwr_offset from dtb (rebase on %d):\r\n", PWR_OFFSET_BASE);
        //TODO FIXME log buffer
        //log_buf_int8(poweroffset, 14);
    }  else {
        log_error("pwr_offset NULL. lentmp = %d\r\n", lentmp);
        return -1;
    }
    return 0;
}

static void update_poweroffset_config_with_order(const void *fdt, int offset1, const char *order)
{
    int i, set, len, j;
    int8_t poweroffset[14], poweroffset_tmp[14];

    memset(poweroffset, 0, sizeof(poweroffset));
    memset(poweroffset_tmp, 0, sizeof(poweroffset_tmp));
    set = 0;
    len = strlen(order);
    for (i = 0; i < XTAL_ORDER_ADDR_LEN_MAX && i < len; i++) {
        switch (order[i]) {
            case 'B':
            case 'b':
            {
                if (0 == bl_efuse_read_pwroft(poweroffset_tmp)) {
                    set = 1;
                    log_debug("get pwr offset from B(b) ready\r\n");
                    if ('B' == order[i]) {
                        /*non-incremental mode*/
                        for (j = 0; j < sizeof(poweroffset); j++) {
                            poweroffset[j] = poweroffset_tmp[j];
                        }
                        log_debug("Use pwr offset from B only\r\n");
                        goto break_scan;
                    } else {
                        /*incremental mode*/
                        log_debug("Use pwr offset from b in incremental mode\r\n");
                        for (j = 0; j < sizeof(poweroffset); j++) {
                            poweroffset[j] += poweroffset_tmp[j];
                        }
                    }
                } else {
                    log_debug("get pwr offset from B(b) failed\r\n");
                }
            }
            break;
            case 'F':
            case 'f':
            {
                if (0 == update_poweroffset_config_get_mac_from_dtb(fdt, offset1, poweroffset_tmp)) {
                    set = 1;
                    log_debug("get pwr offset from F(f) ready\r\n");
                    if ('B' == order[i]) {
                        /*non-incremental mode*/
                        for (j = 0; j < sizeof(poweroffset); j++) {
                            poweroffset[j] = poweroffset_tmp[j];
                        }
                        log_debug("Use pwr offset from F only\r\n");
                        goto break_scan;
                    } else {
                        /*incremental mode*/
                        log_debug("Use pwr offset from f in incremental mode\r\n");
                        for (j = 0; j < sizeof(poweroffset); j++) {
                            poweroffset[j] += poweroffset_tmp[j];
                        }
                    }
                    goto break_scan;
                } else {
                    log_debug("get pwr offset from F(f) failed\r\n");
                }
            }
            break;
            default:
            {
                BL_ASSERT(0);
            }
        }
    }
break_scan:
    if (0 == set) {
        log_info("Using Default pwr offset\r\n");//all zeros actually
    }
    //log_buf_int8(poweroffset, 14);
    //TODO FIXME POWER OFFSET
    //bl60x_fw_power_offset_set(poweroffset);
}


#define PWR_OFFSET_ORDER_ADDR_LEN_MAX      (2)
static void update_poweroffset_config(const void *fdt, int offset1)
{
    int lentmp = 0, countindex;
    char pwr_mode[3];
    const char *result = 0;

    countindex = fdt_stringlist_count(fdt, offset1, "pwr_mode");
    if (1 == countindex) {
        result = fdt_stringlist_get(fdt, offset1, "pwr_mode", 0, &lentmp);
        log_info("pwr_mode length %d\r\n", lentmp);
        if (lentmp <= PWR_OFFSET_ORDER_ADDR_LEN_MAX) {
            memcpy(pwr_mode, result, lentmp);
            pwr_mode[sizeof(pwr_mode) - 1] = '\0';
            log_info("pwr_mode is %s\r\n", pwr_mode);
            update_poweroffset_config_with_order(fdt, offset1, pwr_mode);
        }
    }
}

static int hal_board_load_fdt_info(const void *dtb)
{
    const void *fdt = (const void *)dtb;/* const */

    int wifi_offset = 0;    /* subnode wifi */
    int offset1 = 0;        /* subnode offset1 */
    const uint8_t *addr_prop = 0;
    const char *result = 0;

    int lentmp = 0;
    int countindex = 0;
    int i;

    wifi_offset = fdt_subnode_offset(fdt, 0, "wifi");
    if (!(wifi_offset > 0)) {
       log_error("wifi NULL.\r\n");
    }

    offset1 = fdt_subnode_offset(fdt, wifi_offset, "brd_rf");
    if (offset1 > 0) {
        uint32_t channel_div_table[15];
        uint16_t channel_cnt_table[14];
        uint16_t lo_fcal_div = 0;

        /* set xtal */
        update_xtal_config(fdt, offset1);

        /* set channel_div_table, channel_cnt_table, lo_fcal_div */
        addr_prop = fdt_getprop(fdt, offset1, "channel_div_table", &lentmp);
        if (15*4 == lentmp) {
            for (i = 0; i < 15; i++) {
                channel_div_table[i] = BL_FDT32_TO_U32(addr_prop, 4*i);
            }
            log_info("channel_div_table :\r\n");
            log_buf(channel_div_table, 15*4);
        }  else {
            log_error("channel_div_table NULL.\r\n");
        }

        addr_prop = fdt_getprop(fdt, offset1, "channel_cnt_table", &lentmp);
        if (14*4 == lentmp) {
            for (i = 0; i < 14; i++) {
                channel_cnt_table[i] = BL_FDT32_TO_U16(addr_prop, 4*i);
            }
            log_info("channel_cnt_table :\r\n");
            log_buf(channel_cnt_table, 14*4);
        }  else {
            log_error("channel_cnt_table NULL.\r\n");
        }

        addr_prop = fdt_getprop(fdt, offset1, "lo_fcal_div", &lentmp);
        if (4 == lentmp) {
            lo_fcal_div = BL_FDT32_TO_U16(addr_prop, 4*0);
            log_info("lo_fcal_div : %d\r\n", lo_fcal_div);
        }  else {
            log_error("lo_fcal_div NULL.\r\n");
        }

        //TODO FIXME POWER
        //bl60x_fw_rf_table_set(channel_div_table, channel_cnt_table, lo_fcal_div);
    }

    offset1 = fdt_subnode_offset(fdt, wifi_offset, "mac");
    if (offset1 > 0) {
        update_mac_config(fdt, offset1);
    }

    offset1 = fdt_subnode_offset(fdt, wifi_offset, "region");
    if (offset1 > 0) {
        /* set country_code */
        addr_prop = fdt_getprop(fdt, offset1, "country_code", &lentmp);
        if (4 == lentmp) {
            log_info("country_code : %d\r\n", BL_FDT32_TO_U8(addr_prop, 4*0));

            bl_wifi_country_code_set(BL_FDT32_TO_U8(addr_prop, 4*0));
        }  else {
            log_error("country_code NULL.\r\n");
        }
    }

    offset1 = fdt_subnode_offset(fdt, wifi_offset, "brd_rf");
    if (offset1 > 0)
    {
        /* set tx_pwr_tbl */
        uint8_t pwr_table[16*4];

        addr_prop = fdt_getprop(fdt, offset1, "pwr_table", &lentmp);
        if (16*4*4 == lentmp) {
            for (i = 0; i < 16*4; i++) {
                pwr_table[i] = BL_FDT32_TO_U32(addr_prop, 4*i);
            }
            log_info("pwr_table :\r\n");
            log_buf(pwr_table, 16*4);

            //TODO FIXME POWER
            //bl_wifi_power_table_set((bl_tx_pwr_tbl_t *)pwr_table);
        }  else {
            log_error("pwr_table NULL. lentmp = %d\r\n", lentmp);
        }

        update_poweroffset_config(fdt, offset1);
    }

    offset1 = fdt_subnode_offset(fdt, wifi_offset, "ap");
    if (offset1 > 0)
    {
        /* set ssid pwd */
        uint8_t ap_ssid[32];
        uint8_t ap_ssid_len = 0;
        uint8_t ap_psk[64];
        uint8_t ap_psk_len = 0;
        uint8_t ap_channel = 0;

        countindex = fdt_stringlist_count(fdt, offset1, "ssid");
        if (1 == countindex) {
            result = fdt_stringlist_get(fdt, offset1, "ssid", 0, &lentmp);
            if ((lentmp > 0) &&(lentmp<32)) {/* !NULL */
                log_info("ap_ssid string[%d] = %s, ap_ssid_len = %d\r\n", 0, result, lentmp);
                memcpy(ap_ssid, result, lentmp);
                ap_ssid[lentmp] = '\0';
                ap_ssid_len = lentmp;
            }
        }

        countindex = fdt_stringlist_count(fdt, offset1, "pwd");
        if (1 == countindex) {
            result = fdt_stringlist_get(fdt, offset1, "pwd", 0, &lentmp);
            if ((lentmp > 0) &&(lentmp<32)) {/* !NULL */
                log_info("ap_psk string[%d] = %s, ap_psk_len = %d\r\n", 0, result, lentmp);
                memcpy(ap_psk, result, lentmp);
                ap_psk[lentmp] = '\0';
                ap_psk_len = lentmp;
            }
        }

        addr_prop = fdt_getprop(fdt, offset1, "ap_channel", &lentmp);
        if (addr_prop) {
            log_info("ap_channel = %ld\r\n", BL_FDT32_TO_U32(addr_prop, 0));

            ap_channel = BL_FDT32_TO_U32(addr_prop, 0);
        } else {
            log_error("ap_channel NULL.\r\n");
        }

        bl_wifi_ap_info_set(ap_ssid, ap_ssid_len,
                            ap_psk, ap_psk_len,
                            ap_channel);
    }

    return 0;
}
#endif

uint32_t hal_board_get_factory_addr(void)
{
    return factory_addr;
}

int hal_board_cfg(uint8_t board_code)
{
    int ret;
    uint32_t size;

    ret = hal_boot2_partition_addr_active("factory", &factory_addr, &size);
    printf("[MAIN] [BOARD] [FLASH] addr from partition is %08x, ret is %d\r\n", (unsigned int)factory_addr, ret);
    if (0 == factory_addr) {
        printf("[MAIN] [BOARD] [FLASH] Dead loop. Reason: NO valid Param Parition found\r\n");
        while (1) {
        }
    }

    ret = hal_boot2_partition_bus_addr_active("factory", &factory_addr, &size);
    printf("[MAIN] [BOARD] [XIP] addr from partition is %08x, ret is %d\r\n", (unsigned int)factory_addr, ret);
    if (0 == factory_addr) {
        printf("[MAIN] [BOARD] [XIP] Dead loop. Reason: NO valid Param Parition found\r\n");
        while (1) {
        }
    }

#ifndef FEATURE_WIFI_DISABLE
    hal_board_load_fdt_info((const void *)factory_addr);
#endif

    return 0;
}
