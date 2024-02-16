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
/*
 * modeltime.c
 *
 *  Created on: 01.09.2020
 *      Author: Andi
 *
 * Run the virtual model time including speedup and event generation.
 */

#include <stdio.h>
#include "rb2.h"
#include "events.h"
#include "decoder.h"
#include "timers.h"

#define TICKS_PER_MINUTE	(pdMS_TO_TICKS(60 * 1000))		///< number of ticks for a full minute at realtime speed
#define TIMER_WAIT			100								///< ticks to wait when sending messages to the timer task
#define TIMER_MAX_FACTOR	63								///< the maximum acceleration for the model time
#define TIMER_MAX_YEAR		4095							///< the year can range from 0 .. 4095

static SemaphoreHandle_t mutex;	///< a mutex to control access to the time structure
static TimerHandle_t timer;

static volatile struct modeltime theTime;

static void mt_timerCallback (TimerHandle_t t)
{
	static TickType_t last_time = 0;

	(void) t;

	if (!mutex_lock(&mutex, 200, __func__)) return;

	if (++theTime.min >= 60) {
		theTime.min = 0;
		if (++theTime.hour >= 24) {
			theTime.hour = 0;
			if (++theTime.wday > 6) theTime.wday = 0;
			if (++theTime.mday > daysInMonth(theTime.year, theTime.mon)) {
				theTime.mday = 1;
				if (++theTime.mon > 12) {
					theTime.mon = 1;
					theTime.year++;
				}
			}
		}
	}

//	printf ("%s(): %s, %02d.%02d.%04d %2d:%02d (%dx)\n", __func__, weekday(theTime.wday),
//			theTime.mday, theTime.mon, theTime.year,
//			theTime.hour, theTime.min, theTime.speedup);
	mutex_unlock(&mutex);
	event_fire(EVENT_MODELTIME, 0, (void *) &theTime);
	if (theTime.hour == 0 && theTime.min == 0) {		// Date packet at 0:00 o'clock (midnight)
		sigq_queuePacket(sigq_modelDatePacket(theTime.year, theTime.mon, theTime.mday));
	}
	// if there was at least half a minute real time since last time announcement, we will send such an announcement on the track
	if ((xTaskGetTickCount() - last_time) > (TICKS_PER_MINUTE / 2)) {
		sigq_queuePacket(sigq_modelTimePacket(theTime.hour, theTime.min, theTime.wday, theTime.speedup, false));
		last_time = xTaskGetTickCount();
	}
}

void mt_init (void)
{

	// let's start with noon, January 1st, 2020 - this was a wednesday (2)
	theTime.year = 2020;
	theTime.mon = 1;
	theTime.mday = 1;
	theTime.wday = 2;
	theTime.hour = 12;
	theTime.min = 0;
	theTime.speedup = 1;

	if ((timer = xTimerCreate("ModelTime", TICKS_PER_MINUTE, pdTRUE, NULL, mt_timerCallback)) != NULL) {
		xTimerStart(timer, TIMER_WAIT);
	}
}

static void mt_commitTimer (void)
{
	if (!timer) {
		timer = xTimerCreate("ModelTime", TICKS_PER_MINUTE, pdTRUE, NULL, mt_timerCallback);
		if (!timer) return;
	}

	if (theTime.speedup == 0) {
		xTimerStop(timer, TIMER_WAIT);
	} else {
		xTimerChangePeriod(timer, (TICKS_PER_MINUTE + theTime.speedup / 2) / theTime.speedup, TIMER_WAIT);	// will also start the timer
	}
	event_fire(EVENT_MODELTIME, 0, (void *) &theTime);
}

void mt_speedup (int factor)
{
	if (factor < 0) factor = 0;
	if (factor > TIMER_MAX_FACTOR) factor = TIMER_MAX_FACTOR;
	if (theTime.speedup == factor) return;

	if (mutex_lock(&mutex, 100, __func__)) {
		theTime.speedup = factor;
		mutex_unlock(&mutex);
		mt_commitTimer();
		sigq_queuePacket(sigq_modelTimePacket(theTime.hour, theTime.min, theTime.wday, theTime.speedup, true));
	}
}

void mt_setdatetime (int year, int mon, int mday, int hour, int min)
{
	bool dateChanged, timeChanged;

	dateChanged = timeChanged = false;

	if (mutex_lock(&mutex, 100, __func__)) {
		if (year >= 0 && year <= TIMER_MAX_YEAR) {
			if (theTime.year != year) dateChanged = true;
			theTime.year = year;
		}
		if (mon >= 1 && mon <= 12) {
			if (theTime.mon != mon) dateChanged = true;
			theTime.mon = mon;
		}
		if (mday >= 1 && mday <= 31) {
			if (mday > daysInMonth(theTime.year, theTime.mon)) mday = daysInMonth(theTime.year, theTime.mon);
			if (theTime.mday != mday) dateChanged = true;
			theTime.mday = mday;
		}
		if (hour >= 0 && hour <= 23) {
			if (theTime.hour != hour) timeChanged = true;
			theTime.hour = hour;
		}
		if (min >= 0 && min <= 59) {
			if (theTime.min != min) timeChanged = true;
			theTime.min = min;
		}
		theTime.wday = calc_weekday(theTime.year, theTime.mon, theTime.mday);
		mutex_unlock(&mutex);
	}
	if (dateChanged || timeChanged) {
		mt_commitTimer();
	} else {
		mt_report();	// just send an immediate event to adjust the clock
	}
	if (dateChanged) sigq_queuePacket(sigq_modelDatePacket(theTime.year, theTime.mon, theTime.mday));
	if (timeChanged) sigq_queuePacket(sigq_modelTimePacket(theTime.hour, theTime.min, theTime.wday, theTime.speedup, true));
}

/**
 * Fire an event with the current date and time.
 * This can be requested from the CGI interface to read
 * current model time and date.
 *
 * BE WARNED: the reported time doesn't include any sub-minute
 * timing information, so you cannot simply start with the
 * reported minute - instead hold your clock at the minute
 * value (with seconds internally set to zero) until you receive
 * the next minute-event.
 */
void mt_report (void)
{
	event_fire(EVENT_MODELTIME, 0, (void *) &theTime);
}
