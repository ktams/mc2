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

/**
 * Lock a mutex and create it, if it doesn't already exist (i.e. lazy loading).
 * We must pay attention to avoid creating two mutexes through two different
 * tasks doing the same at the same time. Therefor we create a CRITICAL-SECION
 * that handles the mutex creation.
 *
 * There is however a problem: Inside the critical section, we should not call
 * any FreeRTOS functions - including creating a mutex / semaphore. We solve this
 * by first creating a mutex outside the critical section and then assign it to
 * the real mutex if it still does not exist. If it meanwhile exists (created by
 * a different thread) we free our temporary mutex after the critical section.
 *
 * \param mutex		a pointer to a local SemaphoreHandle_t variable (aka. mutex)
 * \param tout		the time (in ms) to wait for aquireing the mutex before giving up
 * \param caller	the calling function name (supply "__func__")
 * \return			<i>true</i> if lock could be aquired, <i>false</i> otherwise
 */
bool mutex_lock(SemaphoreHandle_t * volatile mutex, TickType_t tout, const char *caller)
{
	SemaphoreHandle_t tmp;

	if (!mutex) return false;
	if (!caller) caller = __func__;

	if (!*mutex) {		// pre-check if the mutex already exists (ATTENTION: a second task could do that too - possible race condition!)
		if ((tmp = xSemaphoreCreateMutex()) == NULL) {	// create a new (temporary) mutex outside the critical section
			log_error ("%s(): could not create mutex!\n", caller);
			return false;
		}
		taskENTER_CRITICAL();			// from here on, no task switch can occure
		if (!*mutex) {					// this time the check is real and cannot be done concurrently in another task
			*mutex = tmp;				// assign the created mutex to the referenced mutex
			tmp = NULL;					// and NULL-out the temporary reference
		}
		taskEXIT_CRITICAL();			// back to normal ...
		if (tmp) vSemaphoreDelete(tmp);	// if the temporary mutex was not used, we must destroy it after leaving the critical section
	}

	if (xSemaphoreTake(*mutex, pdMS_TO_TICKS(tout)) == pdTRUE) return true;
	log_error ("%s(): could not aquire mutex (%lums)!\n", caller, tout);
	return false;
}

/**
 * Unlocks the mutex.
 *
 * \param mutex		a pointer to a local SemaphoreHandle_t variable (aka. mutex)
 */
void mutex_unlock(SemaphoreHandle_t *mutex)
{
	if (mutex && *mutex) {
		xSemaphoreGive(*mutex);
	}
}

/**
 * Frees memory allocated for a mutex.
 * It is assured, that no two threads can do that at the very same time.
 *
 * \param mutex		a pointer to a local SemaphoreHandle_t variable (aka. mutex)
 */
void mutex_destroy (SemaphoreHandle_t *mutex)
{
	SemaphoreHandle_t tmp = NULL;

	taskENTER_CRITICAL();			// from here on, no task switch can occure
	if (mutex) {
		tmp = *mutex;
		*mutex = NULL;
	}
	taskEXIT_CRITICAL();
	if (tmp) vSemaphoreDelete(tmp);
}
