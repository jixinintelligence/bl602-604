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
#include <zephyr.h>
#include <misc/util.h>
#include <misc/dlist.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BLUETOOTH_DEBUG_CORE)

#include <log.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "atomic.h"

#include "errno.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#if defined(BL702)
#include "bl702.h"
#endif

void k_queue_init(struct k_queue *queue)
{
    int size = 20;
    uint8_t blk_size = sizeof(void *) + 1;


    queue->hdl = xQueueCreate(size, blk_size);
    BT_ASSERT(queue->hdl != NULL);

    sys_dlist_init(&queue->poll_events);
}

void k_queue_insert(struct k_queue *queue, void *prev, void *data)
{
    BaseType_t ret;
    (void) ret;
    
    ret = xQueueSend(queue->hdl, &data, portMAX_DELAY);
    BT_ASSERT(ret == pdPASS);
}

void k_queue_append(struct k_queue *queue, void *data)
{
    k_queue_insert(queue, NULL, data);
}

void k_queue_insert_from_isr(struct k_queue *queue, void *prev, void *data)
{
    BaseType_t xHigherPriorityTaskWoken;
    
    xQueueSendFromISR(queue->hdl, &data, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void k_queue_append_from_isr(struct k_queue *queue, void *data)
{
    k_queue_insert_from_isr(queue, NULL, data);
}

void k_queue_free(struct k_queue *queue)
{
    vQueueDelete(queue->hdl);
}

void k_queue_prepend(struct k_queue *queue, void *data)
{
    k_queue_insert(queue, NULL, data);
}

void k_queue_append_list(struct k_queue *queue, void *head, void *tail)
{
    struct net_buf *buf_tail = (struct net_buf *)head;

    for (buf_tail = (struct net_buf *)head; buf_tail; buf_tail = buf_tail->frags) {
        k_queue_append(queue, buf_tail);
    }
}

void *k_queue_get(struct k_queue *queue, s32_t timeout)
{
    void *msg = NULL;
    unsigned int t = timeout;
    BaseType_t ret;

    (void) ret;

    if (timeout == K_FOREVER) {
        t = BL_WAIT_FOREVER;
    } else if (timeout == K_NO_WAIT) {
        t = BL_NO_WAIT;
    }

    ret = xQueueReceive(queue->hdl, &msg, t == BL_WAIT_FOREVER ? portMAX_DELAY : ms2tick(t));
    if (ret == pdPASS) {
        return msg;
    } else {
        return NULL;
    }
}

int k_queue_is_empty(struct k_queue *queue)
{
     return uxQueueMessagesWaiting(queue->hdl)? 0:1;
}

int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit)
{
    if (NULL == sem) {
        BT_ERR("sem is NULL\n");
        return -EINVAL;
    }

    sem->sem.hdl = xSemaphoreCreateCounting(limit, initial_count);
    sys_dlist_init(&sem->poll_events);
    return 0;
}

int k_sem_take(struct k_sem *sem, uint32_t timeout)
{
    BaseType_t ret;
    unsigned int t = timeout;

    (void) ret;
    if (timeout == K_FOREVER) {
        t = BL_WAIT_FOREVER;
    } else if (timeout == K_NO_WAIT) {
        t = BL_NO_WAIT;
    }

    if(NULL == sem){
        return -1;
    }

    ret = xSemaphoreTake(sem->sem.hdl, t == BL_WAIT_FOREVER ? portMAX_DELAY : ms2tick(t));
    return ret == pdPASS ? 0 : -1;
}

int k_sem_give(struct k_sem *sem)
{
    BaseType_t ret;
    (void) ret;
    
    if (NULL == sem) {
        BT_ERR("sem is NULL\n");
        return -EINVAL;
    }

    ret = xSemaphoreGive(sem->sem.hdl);
    return  ret == pdPASS ? 0 : -1;
}

int k_sem_delete(struct k_sem *sem)
{
    if (NULL == sem) {
        BT_ERR("sem is NULL\n");
        return -EINVAL;
    }

    vSemaphoreDelete(sem->sem.hdl);
    return 0;
}

unsigned int k_sem_count_get(struct k_sem *sem)
{
    return uxQueueMessagesWaiting(sem->sem.hdl);
}

void k_mutex_init(struct k_mutex *mutex)
{

    if (NULL == mutex) {
        BT_ERR("mutex is NULL\n");
        return ;
    }

    mutex->mutex.hdl = xSemaphoreCreateMutex();
    BT_ASSERT(mutex->mutex.hdl != NULL);
    sys_dlist_init(&mutex->poll_events);
}

int64_t k_uptime_get()
{
    return k_now_ms();
}

u32_t k_uptime_get_32(void)
{
    return (u32_t)k_now_ms();
}

int k_thread_create(struct k_thread *new_thread, const char *name,
                    size_t stack_size, k_thread_entry_t entry,
                    int prio)
{
    stack_size /= sizeof(StackType_t);
    xTaskCreate(entry, name, stack_size, NULL, prio, (void *)(&new_thread->task));
    
    return new_thread->task? 0 : -1;
}

int k_yield(void)
{
    taskYIELD();
    return 0;
}

void k_sleep(s32_t dur_ms)
{
    TickType_t ticks;
    ticks = pdMS_TO_TICKS(dur_ms);
    vTaskDelay(ticks);
}

unsigned int irq_lock(void)
{
	taskENTER_CRITICAL();
    return 1;
}

void irq_unlock(unsigned int key)
{
	taskEXIT_CRITICAL();
}

int k_is_in_isr(void)
{
    #if defined(ARCH_RISCV)
    return (xPortIsInsideInterrupt());
    #else
    /* IRQs + PendSV (14) + SYSTICK (15) are interrupts. */
    return (__get_IPSR() > 13);
    #endif

    return 0;
}

void k_timer_init(k_timer_t *timer, k_timer_handler_t handle, void *args)
{
    BT_ASSERT(timer != NULL);
    timer->handler = handle;
    timer->args = args;

    timer->timer.hdl = xTimerCreate("Timer", pdMS_TO_TICKS(1000), 0, 0, (TimerCallbackFunction_t)(timer->handler)); 
    BT_ASSERT(timer->timer.hdl != NULL);
}

void k_timer_start(k_timer_t *timer, uint32_t timeout)
{
    BaseType_t ret;
    (void) ret;
    
    BT_ASSERT(timer != NULL);
    timer->timeout = timeout;
    timer->start_ms = k_now_ms();

    ret = xTimerStop(timer->timer.hdl, 0);
    BT_ASSERT(ret == pdPASS);
    ret = xTimerChangePeriod(timer->timer.hdl, pdMS_TO_TICKS(timeout), 0);
    BT_ASSERT(ret == pdPASS);
    ret = xTimerStart(timer->timer.hdl, 0);
    BT_ASSERT(ret == pdPASS);
}

void k_timer_stop(k_timer_t *timer)
{
    BaseType_t ret;

    (void) ret;
    BT_ASSERT(timer != NULL);
  
    ret = xTimerStop(timer->timer.hdl, 0);
    BT_ASSERT(ret == pdPASS);
}

void k_timer_delete(k_timer_t *timer)
{
    BaseType_t ret;
    (void) ret;
    
    BT_ASSERT(timer != NULL);
    
    ret = xTimerDelete(timer->timer.hdl, 0);
    BT_ASSERT(ret == pdPASS);
}

long long k_now_ms(void)
{
    return (long long)(xTaskGetTickCount() * 1000)/configTICK_RATE_HZ;   
}
