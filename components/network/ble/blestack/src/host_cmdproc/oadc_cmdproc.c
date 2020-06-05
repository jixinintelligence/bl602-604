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
#include "oadc_cmdproc.h"
#include "oad.h"

struct oad_cmdproc_block_req_t
{
    struct oad_file_info file_info;
    uint32_t file_offset;
    uint8_t data_len;
}__packed;

struct oad_cmdproc_upgrd_end_t
{
    uint8_t status;
    struct oad_file_info file_info;
}__packed;

struct oad_cmdproc_req_t
{
    uint8_t cmd_id;
    union{
        struct oad_file_info file_info;
        struct oad_cmdproc_block_req_t block_req;
        struct oad_cmdproc_upgrd_end_t upgrd_end;
    }q;
}__packed;

void oad_cmdproc_start(void)
{
    uint8_t req[1];
    req[0] = OAD_CMDPROC_START;

    cmdproc_data_send(CMDPROC_TYPE_OAD, req, sizeof(req));
}

void oad_cmdproc_image_identity(struct oad_file_info* file_info)
{    
    struct oad_cmdproc_req_t req;
    req.cmd_id = OAD_CMDPROC_IMAGE_IDENTITY;
    memcpy(&req.q.file_info, file_info, sizeof(*file_info));
    cmdproc_data_send(CMDPROC_TYPE_OAD, (uint8_t *)&req, sizeof(req)); 
}

void oad_cmdproc_block_req(struct oad_file_info* file_info, uint32_t file_offset, uint8_t data_len)
{
    struct oad_cmdproc_req_t req;

    req.cmd_id = OAD_CMDPROC_BLOCK_REQ;
    memcpy(&req.q.block_req.file_info, file_info, sizeof(*file_info));
    req.q.block_req.file_offset = file_offset;
    req.q.block_req.data_len = data_len;
    cmdproc_data_send(CMDPROC_TYPE_OAD, (uint8_t *)&req, sizeof(req)); 
}

void oad_cmdproc_upgrd_end(uint8_t status, struct oad_file_info* file_info)
{
    struct oad_cmdproc_req_t req;

    req.cmd_id = OAD_CMDPROC_UPGRD_END;
    req.q.upgrd_end.status = status;
    memcpy(&req.q.upgrd_end.file_info, file_info, sizeof(*file_info));

    cmdproc_data_send(CMDPROC_TYPE_OAD, (uint8_t *)&req, sizeof(req)); 
}
void oad_cmdproc_enable(cmdproc_recv_cb cb)
{
    cmdproc_callback_set(CMDPROC_TYPE_OAD, cb);
    cmdproc_init();
}
