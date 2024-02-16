/*
 * RB2, next generation model railroad controller software
 * Copyright (C) 2020 Tams Elektronik GmbH and Andreas Kretzer
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include "rb2.h"
#include "arch/cc.h"
#include "arch/sys_arch.h"
#include "lwip/err.h"
#include "lwip/sys.h"

/*
 * ================================================================================================
 * Semaphore functions
 * ================================================================================================
 */
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
    *sem = xSemaphoreCreateBinary();
    if (!*sem) return ERR_MEM;
    if (count) xSemaphoreGive(*sem);
    return ERR_OK;
}

void sys_sem_free(sys_sem_t *sem)
{
    vSemaphoreDelete(*sem);
}

void sys_sem_signal(sys_sem_t *sem)
{
    xSemaphoreGive(*sem);
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    TickType_t t;
    
    if (!timeout) {
		xSemaphoreTake(*sem, portMAX_DELAY);
		t = 0;
    } else {
		t = xTaskGetTickCount();
		if (xSemaphoreTake(*sem, pdMS_TO_TICKS(timeout)) != pdTRUE) return SYS_ARCH_TIMEOUT;
		t = (xTaskGetTickCount() - t) * portTICK_PERIOD_MS;
    }
    return t;
}

#if defined(sys_sem_valid)		// just because of a define in an unused subfolder (test) of lwIP
#undef sys_sem_valid
#endif
int sys_sem_valid(sys_sem_t *sem)
{
    return (sem != NULL && *sem != NULL);
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
    if (sem) *sem = NULL;
}

/*
 * ================================================================================================
 * Mutex functions
 * ================================================================================================
 */
err_t sys_mutex_new(sys_mutex_t *mutex)
{
    *mutex = xSemaphoreCreateMutex();
    if (!*mutex) return ERR_MEM;
    return ERR_OK;
}

void sys_mutex_free(sys_mutex_t *mutex)
{
	if (mutex && *mutex) vSemaphoreDelete(*mutex);
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
	if (mutex && *mutex) xSemaphoreTake(*mutex, portMAX_DELAY);
}

void sys_mutex_unlock(sys_mutex_t *mutex)
{
	if (mutex && *mutex) xSemaphoreGive(*mutex);
}

#if defined(sys_mutex_valid)		// just because of a define in an unused subfolder (test) of lwIP
#undef sys_mutex_valid
#endif
int sys_mutex_valid(sys_mutex_t *mutex)
{
    return (mutex != NULL && *mutex != NULL);
}

void sys_mutex_set_invalid(sys_mutex_t *mutex)
{
	if (mutex && *mutex) *mutex = NULL;
}

/*
 * ================================================================================================
 * Mailbox (Queues) functions
 * ================================================================================================
 */
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	if (!mbox) return ERR_MEM;
    *mbox = xQueueCreate(size, sizeof(void *));
    if (!*mbox) return ERR_MEM;
    return ERR_OK;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
    if (mbox && *mbox) vQueueDelete(*mbox);
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
	if (mbox && *mbox) xQueueSendToBack (*mbox, &msg, portMAX_DELAY);
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	if (!mbox || !*mbox) return ERR_MEM;
    if (xQueueSendToBack (*mbox, &msg, 0) != pdTRUE) return ERR_MEM;
    return ERR_OK;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	TickType_t t;
	void *rxmsg;

	if (!mbox || !*mbox) return 0;

	if (!timeout) {
		xQueueReceive (*mbox, &rxmsg, portMAX_DELAY);
		t = 0;
	} else {
		t = xTaskGetTickCount();
		if (xQueueReceive (*mbox, &rxmsg, pdMS_TO_TICKS(timeout)) != pdTRUE) return SYS_ARCH_TIMEOUT;
		t = (xTaskGetTickCount() - t) * portTICK_PERIOD_MS;
	}
	if (msg) *msg = rxmsg;
	return t;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    void *rxmsg;
    
	if (!mbox || !*mbox) return SYS_MBOX_EMPTY;

	if (xQueueReceive (*mbox, &rxmsg, 0) != pdTRUE) return SYS_MBOX_EMPTY;
    if (msg) *msg = rxmsg;
    return 0;
}

#if defined(sys_mbox_valid)		// just because of a define in an unused subfolder (test) of lwIP
#undef sys_mbox_valid
#endif
int sys_mbox_valid(sys_mbox_t *mbox)
{
    return (mbox != NULL && *mbox != NULL);
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
    if (mbox) *mbox = NULL;
}

/*
 * ================================================================================================
 * Other functions
 * ================================================================================================
 */
void sys_init (void)
{
    // TODO: implement function
}

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
    sys_thread_t tid = NULL;
    
    xTaskCreate(thread, name, stacksize, arg, prio, &tid);
    return tid;
}

u32_t sys_now(void)
{
    return xTaskGetTickCount();
}

static sys_prot_t protect_level;

sys_prot_t sys_arch_protect(void)
{
	if (!protect_level) vTaskSuspendAll();
	protect_level++;
    return protect_level;
}

void sys_arch_unprotect(sys_prot_t pval)
{
    if (pval && protect_level > 0) protect_level--;
    if (!protect_level) xTaskResumeAll();
}

void sys_tcpip_alive (void)
{
	printf ("%s()\n", __func__);
}

void sys_tcpip_started (void)
{
	printf ("%s()\n", __func__);
}
