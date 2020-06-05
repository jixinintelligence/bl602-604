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
#ifndef __OAD_H__
#define __OAD_H__

#include "types.h"
#include "hci_host.h"
#include "work_q.h"

#define LOCAL_MANU_CODE 0x07AF
#define LOCAL_FILE_VER  00000001

#define OAD_OPCODE_SIZE 1
//00070000-0745-4650-8d93-df59be2fc10a
#define BT_UUID_OAD                     BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x00070000, 0x0745, 0x4650, 0x8d93, 0xdf59be2fc10a))
//00070001-0745-4650-8d93-df59be2fc10a
#define BT_UUID_OAD_DATA_IN    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x00070001, 0x0745, 0x4650, 0x8d93, 0xdf59be2fc10a))
//00070002-0745-4650-8d93-df59be2fc10a
#define BT_UUID_OAD_DATA_OUT BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x00070002, 0x0745, 0x4650, 0x8d93, 0xdf59be2fc10a))

enum{
    OAD_SUCC = 0x00,
    OAD_ABORT,
    OAD_INVALID_IMAG,
    OAD_REQ_MORE_DATA,
    OAD_MALORMED_CMD,
    OAD_UPGRD_CMPLT,
};

enum{
    OAD_TYPE_IDENTITY = 0x00,
    OAD_TYPE_IMAG_TRANS
};

enum{
    OAD_CMD_IMAG_IDENTITY = 0x00,
    OAD_CMD_IMAG_BLOCK_REQ,
    OAD_CMD_IMAG_BLOCK_RESP,
    OAD_CMD_IMAG_UPGRD_END,
};

struct oad_file_info{
    u16_t manu_code;
    u32_t file_ver;
} __packed;

struct oad_env_tag{
    struct oad_file_info file_info;
    u32_t cur_file_size;
    u32_t upgrd_file_ver;
    u32_t upgrd_file_size;
    u32_t upgrd_offset;
    u32_t upgrd_crc32;
    #if defined(CFG_BOOT2_ENABLED)
    struct k_delayed_work upgrd_work;
    #endif
    u32_t new_img_addr;
};

struct oad_image_identity_t{
    struct oad_file_info file_info;
    u32_t file_size;
    u32_t crc32;
} __packed;

struct oad_block_req_t{
    struct oad_file_info file_info;
    u32_t file_offset;
} __packed;

#define OAD_BLK_RSP_DATA_OFFSET 12
struct oad_block_rsp_t{
    uint8_t status;
    struct oad_file_info file_info;
    u32_t file_offset;
    u8_t data_size;
    u8_t *pdata;
} __packed;

struct oad_upgrd_end_t{
    u8_t status;
    struct oad_file_info file_info;
} __packed;

#endif //__OAD_H__
