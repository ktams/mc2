/*
 * eventlistener.c
 *
 *  Created on: 15.12.2019
 *      Author: Andi
 */

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

/**
 * \file
 * Push events to registered handlers.
 *
 * ATTENTION: If you try do any logging using the log_msg() or log_error() functions
 * in any part of the event handling, these loggings will trigger another event!
 * This may flood the queue and waste a lot of processor resources!
 *
 * CAVEAT: All the registered eventhandlers may do that ... so this is a dangerous
 * thing in general! Therefor the EVENT_LOGMSG is currently blocked.
 */

#include <stdio.h>
#include <stdlib.h>
#include "rb2.h"
#include "timers.h"
#include "events.h"

#define MAX_MUTEX_WAIT		100			///< maximum waittime (in ms) for the list mutex to become available
#define TIMER_OVERFLOW		(1 << 31)	///< the topmost bit marks a time difference, that tells us that the current time is later than the defined timeout
#define MAX_PENDING_EVENTS	64			///< the queue length for pending events

static volatile struct evtListener *listener;		///< the currently active listeners
static SemaphoreHandle_t mutex;						///< locking for access to listener list
static TimerHandle_t timer;
static TaskHandle_t worker;							///< the thread id of the worker thread
static QueueHandle_t evtqueue;						///< the eventqueue to post events to

static const char *event_name (enum event evt) __attribute__((unused));
static const char *event_name (enum event evt)
{
	switch (evt) {
		case EVENT_TIMEOUT:				return str(EVENT_TIMEOUT);
		case EVENT_SYS_STATUS:			return str(EVENT_SYS_STATUS);
		case EVENT_LOCO_SPEED:			return str(EVENT_LOCO_SPEED);
		case EVENT_LOCO_FUNCTION:		return str(EVENT_LOCO_FUNCTION);
		case EVENT_LOCO_PARAMETER:		return str(EVENT_LOCO_PARAMETER);
		case EVENT_TURNOUT:				return str(EVENT_TURNOUT);
		case EVENT_FEEDBACK:			return str(EVENT_FEEDBACK);
		case EVENT_CURRENT:				return str(EVENT_CURRENT);
		case EVENT_INSTANEOUS_CURRENT:	return str(EVENT_INSTANEOUS_CURRENT);
		case EVENT_NEWLOCO:				return str(EVENT_NEWLOCO);
		case EVENT_BOOSTER:				return str(EVENT_BOOSTER);
		case EVENT_SNIFFER:				return str(EVENT_SNIFFER);
		case EVENT_PROTOCOL:			return str(EVENT_PROTOCOL);
		case EVENT_ACCESSORY:			return str(EVENT_ACCESSORY);
		case EVENT_ENVIRONMENT:			return str(EVENT_ENVIRONMENT);
		case EVENT_CONTROLS:			return str(EVENT_CONTROLS);
		case EVENT_RAILCOM:				return str(EVENT_RAILCOM);
		case EVENT_ACCFMT:				return str(EVENT_ACCFMT);
		case EVENT_LOCO_DB:				return str(EVENT_LOCO_DB);
		case EVENT_MODELTIME:			return str(EVENT_MODELTIME);
		case EVENT_LOGMSG:				return str(EVENT_LOGMSG);
		case EVENT_BIDIDEV:				return str(EVENT_BIDIDEV);
		case EVENT_EXTCONTROL:			return str(EVENT_EXTCONTROL);
		case EVENT_LIGHTS:				return str(EVENT_LIGHTS);
		case EVENT_ENBOOT:				return str(EVENT_ENBOOT);
		case EVENT_CONSIST:				return str(EVENT_CONSIST);
		case EVENT_FBNEW:				return str(EVENT_FBNEW);
		case EVENT_MAX_EVENT:			return str(EVENT_MAX_EVENT);
		case EVENT_DEREGISTER_ALL:		return str(EVENT_DEREGISTER_ALL);
		default:						return "(unknown)";
	}
}

static void event_timerFire(TimerHandle_t t)
{
	(void) t;
//	log_msg (LOG_DEBUG, "%s() fire TIMEOUT\n", __func__);
	event_fire(EVENT_TIMEOUT, 0, NULL);
}

static void event_stopTimer (void)
{
	if (timer) {
		xTimerStop(timer, 5);
	}
}

static void event_startTimer (TickType_t tim)
{
	event_stopTimer();
	if (!timer) {
		timer = xTimerCreate("eventTimer", tim, pdFALSE, NULL, event_timerFire);
		if (!timer) return;
	}

	if (tim == 0 || tim & TIMER_OVERFLOW) return;		// don't care about a timer that has a duration of > 23 days!
	xTimerChangePeriod(timer, tim, 5);
}

/**
 * Calculates the current shortest timeout that we have to wait for.
 * This function should only be called when the mutex is held, because
 * we must scan the list of listeners (but don't change them).
 */
static TickType_t event_calcTimeout (void)
{
	struct evtListener *l;
	TickType_t now, diff, d;

	now = xTaskGetTickCount();
	l = (struct evtListener *) listener;

	diff = TIMER_OVERFLOW;
	while (l && diff) {
		if (l->to_tim) {
			d = l->to_tim - now;
			if (d & TIMER_OVERFLOW) {
				log_error ("%s(): handler %p already timed out (@%s to=%lu)\n", __func__, l->handler, timestamp(l->to_tim), l->timeout);
				vTaskDelay(10);
				return 2;
			} else {
				if (d < diff) diff = d;
			}
		}
		l = l->next;
	}
	if (diff < 2) return 2;
	return diff;
}

static bool event_isDue (struct evtListener *l, uint32_t ev_mask, TickType_t now)
{
	// first check for timeout event
	if (/* (ev_mask & EVENT_TIMEOUT) && l->ev_mask & EVENT_TIMEOUT && */ l->timeout) {	// this listener is (also) waiting for timeouts - check it
//		if ((l->to_tim) == now || ((l->to_tim - now) & TIMER_OVERFLOW)) return true;	// yes, timeout is due!
		if (!!(time_check(now, l->to_tim))) return true;
	}

	// now check for individual events but ignore (mask out) the EVENT_TIMEOUT
	ev_mask &= ~EVENT_TIMEOUT;
	return !!(l->ev_mask & ev_mask);
}

/**
 * This is a thread function, that calls all the registered handlers for an
 * event that has fired. All callbacks are executed in the context of this
 * thread.
 *
 * The list of listeners is scanned for interested ones and then the handler
 * function is called in the context of this thread. If the handler returns
 * <i>false</i> it is removed from the listener list.
 *
 * While this thread is running, the list mutex is taken.
 *
 * @param pvParameter	the thread invocation parameter - this is the allocated eventT from the thread creator and must be freed in the end
 */
static void event_worker (void *pvParameter)
{
	struct evtListener *l, **lpp;
	TickType_t now;
	eventT e;
	uint32_t ev_mask;

	(void) pvParameter;

	if ((evtqueue = xQueueCreate(MAX_PENDING_EVENTS, sizeof(eventT))) == NULL) {
		worker = NULL;
		vTaskDelete(NULL);
	}
	worker = xTaskGetCurrentTaskHandle();

	for (;;) {
		if (xQueueReceive(evtqueue, &e, portMAX_DELAY)) {
//			log_msg (LOG_DEBUG, "%s() event %d (%s) received\n", __func__, e.ev, event_name(e.ev));
//			vTaskDelay(10);
			ev_mask = 1 << e.ev;
			if (mutex_lock(&mutex, MAX_MUTEX_WAIT, __func__)) {
				event_stopTimer();		// we can stop the timer and will recalculate the timeout after the callbacks are done
				now = xTaskGetTickCount();

				lpp = (struct evtListener **) &listener;
				while ((l = *lpp) != NULL) {
					if (event_isDue(l, ev_mask, now)) {
						if (!l->handler(&e, l->private)) {	// if the handler returns false, we take it out of the list of listeners
							*lpp = l->next;
							free (l);
							l = NULL;		// lpp will not be advanced (see below)!
						} else if (l->timeout) {
							l->to_tim = now + l->timeout;
						}
					}
					if (l) lpp = &l->next;	// only advance, if the listener is not taken out of the list
				}

				event_startTimer(event_calcTimeout());
				mutex_unlock(&mutex);
			}

			if (e.src && e.flags & EVTFLAG_FREE_SRC) free (e.src);
		}
	}
}

/**
 * Register an event handler for a specified event.
 * To register a single handler for multiple events, just call this function
 * multiple times with the same handler function and private data.
 *
 * @param evt		the event that we are registering for
 * @param handler	the handler function that is called if one of the registered events fires
 * @param prv		private data for the handler
 * @param timeout	A timeout in ticks (that is ms here) or 0 to define no timeout (infinit waiting for an event).
 * 					This timeout can only be specified once. The first call that sets a timeout is the "winning" one.
 * @return			0 for successful register or an error code otherwise
 */
int event_register (enum event evt, ev_handler handler, void *prv, TickType_t timeout)
{
	struct evtListener *l, **lpp;
	TickType_t to;

	if (!handler) return -2;								// wrong paramter (without a handler, this registration would stay for ever!
	if (!worker) xTaskCreate(event_worker, "EVENTworker", 2048, NULL, 3, NULL);		// run with slighly raised priority

	if (!mutex_lock(&mutex, MAX_MUTEX_WAIT, __func__)) return -1;		// we could not get the lock - bad luck
	lpp = (struct evtListener **) &listener;

	if (timeout) {		// YES, we want to set a timeout
		event_stopTimer();
		to = xTaskGetTickCount() + timeout;
	} else {			// NO, don't care for timeouts
		to = 0;
	}

	while ((l = *lpp) != NULL) {
		if (l->handler == handler && l->private == prv) {	// we already know this handler ...
			l->ev_mask |= 1 << evt;			// ... so just add another event that this handler is waiting for
			if (timeout && !l->timeout) {	// ... and maybe add a timeout
				l->timeout = timeout;
				l->ev_mask |= 1 << EVENT_TIMEOUT;
				l->to_tim = to;
			}
//			printf ("%s(): adding event %d for existing handler\n", __func__, evt);
			break;
		}
		lpp = &l->next;
	}
	if (!l) {	// a new listener registers
		if ((l = malloc (sizeof(*l))) == NULL) {
			if (timeout) event_startTimer(event_calcTimeout());
			mutex_unlock(&mutex);
			return -4;										// no RAM?
		}
//		printf ("%s(): new handler for event %d\n", __func__, evt);
		l->next = NULL;
		l->handler = handler;
		l->timeout = timeout;
		l->to_tim = to;
		l->ev_mask = 1 << evt;
		if (timeout) l->ev_mask |= 1 << EVENT_TIMEOUT;	// if a timeout is set, we automatically also accept TIMEOUT events
		l->private = prv;
		*lpp = l;
	}

	if (timeout) event_startTimer(event_calcTimeout());
	mutex_unlock(&mutex);
	return 0;
}

/**
 * De-Register an event handler for a specified event.
 * To de-register a handler for all events, specify EVENT_DEREGISTER_ALL for the event.
 * The handler is identified by the handler function and private data.
 *
 * @param evt		the event that should be de-registered
 * @param handler	the handler function that was originally registered
 * @param prv		private data for the handler (used to identify a specific instance)
 * @return			0 for successful register or an error code otherwise
 */
int event_deregister (enum event evt, ev_handler handler, void *prv)
{
	struct evtListener *l, **lpp;

	if (!handler) return -2;								// wrong paramter (without a handler, this registration would stay for ever!

	if (!mutex_lock(&mutex, MAX_MUTEX_WAIT, __func__)) return -1;		// we could not get the lock - bad luck
	lpp = (struct evtListener **) &listener;

	while ((l = *lpp) != NULL) {
		if (l->handler == handler && l->private == prv) {	// this is the handler we are looking for ...
			if (evt == EVENT_DEREGISTER_ALL) l->ev_mask = 0;	// clear all registered event types
			else l->ev_mask &= ~(1 << evt);						// just clear the given event type
			if ((l->ev_mask & ~(1 << EVENT_TIMEOUT)) == 0) {	// if only the EVENT_TIMEOUT is left, we can drop this handler
				*lpp = l->next;
				free (l);
			}
			break;
		}
		lpp = &l->next;
	}
	mutex_unlock(&mutex);
	return 0;
}

/**
 * Fire an event.
 * The event is packed into a small structure and then put to a queue. The queue
 * is than serviced by an independant thread. This thread then checks all listeners
 * and serially calls their handler functions.
 *
 * If no listeners are registered, the worker thread isn't running or the queue
 * is not existing we can shortcut this to an immediate return, because the pointer
 * and other variables can be atomically checked without taking the mutex.
 *
 * If the timeout is specified as 0, the event is ignored if it can't be queued
 * up immediately.
 *
 * \param evt		the event to fire
 * \param param		an additional integer parameter (meaning depends on event)
 * \param src		an additional pointer parameter (meaning depends on event)
 * \param flags		flags that should be added to the event (EVTFLAG_...)
 * \param timeout	a timeout in ticks for posting the event to the queue
 * \return			0 if event was queued, negative error code on failure
 * \see				event_worker()
 */
int event_fireEx (enum event evt, int param, void *src, uint32_t flags, TickType_t timeout)
{
	eventT e;

//	if (evt == EVENT_LOGMSG) return -1;
	if (listener && worker && evtqueue && evt < EVENT_MAX_EVENT && evt <= 31) {		// check that all conditions are met
		e.ev = evt;
		e.param = param;
		e.tid = xTaskGetCurrentTaskHandle();
		e.src = src;
		e.flags = flags;
		if (xQueueSend(evtqueue, &e, timeout) == pdTRUE) return 0;
	}

	// OK, some conditions are not met - we perhaps must free the src argument and return a corresponding result
	if ((flags & EVTFLAG_FREE_SRC) && (src != NULL)) free (src);
	if (!listener) return 0;							// OK, no one is listening - that is no error (even if other things might indicate that)
	if (!worker || !evtqueue) return -2;				// worker not running or queue not there
	if (evt >= EVENT_MAX_EVENT || evt > 31) return -3;	// event outside 0 .. 31, maybe we later have to expand this ... this is an error
	return -1;											// this means that no queue space was available
}

/**
 * Fire an event.
 * The event is packed into a small structure and then put to a queue. The queue
 * is than serviced by an independant thread. This thread then checks all listeners
 * and serially calls their handler functions.
 *
 * This function just calls \ref event_fireEx() with the flags cleared and a standard
 * timeout of \ref QUEUE_WAIT_TIME, which usually is sufficient.
 *
 * \param evt		the event to fire
 * \param param		an additional integer parameter (meaning depends on event)
 * \param src		an additional pointer parameter (meaning depends on event)
 * \return			0 if event was queued, negative error code on failure
 * \see				fire_eventEx()
 */
int event_fire (enum event evt, int param, void *src)
{
	return event_fireEx (evt, param, src, 0, QUEUE_WAIT_TIME);
}
