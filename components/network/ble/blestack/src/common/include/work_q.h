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
#ifndef WORK_Q_H
#define WORK_Q_H
#include "atomic.h"
#include "zephyr.h"

#if defined(BFLB_BLE)
struct k_work_q {
    struct k_fifo fifo;
};

typedef struct{
    bl_timer_t timer;
    struct k_delayed_work *delay_work;
}timer_rec_d;

int k_work_q_start();

enum {
    K_WORK_STATE_PENDING,
};
struct k_work;
/* work define*/
typedef void (*k_work_handler_t)(struct k_work *work);
struct k_work {
    void *_reserved;
    k_work_handler_t handler;
    atomic_t flags[1];
};

#define _K_WORK_INITIALIZER(work_handler) \
        { \
        ._reserved = NULL, \
        .handler = work_handler, \
        .flags = { 0 } \
        }

#define K_WORK_INITIALIZER __DEPRECATED_MACRO _K_WORK_INITIALIZER


int k_work_init(struct k_work *work, k_work_handler_t handler);
void k_work_submit(struct k_work *work);

/*delay work define*/
struct k_delayed_work {
    struct k_work work;
    struct k_work_q *work_q;
    k_timer_t timer;
};

void k_delayed_work_init(struct k_delayed_work *work, k_work_handler_t handler);
int k_delayed_work_submit(struct k_delayed_work *work, uint32_t delay);
int k_delayed_work_cancel(struct k_delayed_work *work);
s32_t k_delayed_work_remaining_get(struct k_delayed_work *work);
void k_delayed_work_del_timer(struct k_delayed_work *work);

int add_timer_record(struct k_delayed_work *delay_work);
timer_rec_d *get_timer_record(void *hdl);
int remv_timer_record(struct k_delayed_work *delay_work);
#endif
#endif /* WORK_Q_H */
