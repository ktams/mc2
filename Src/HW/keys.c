/*
 * keys.c
 *
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

#include <stdio.h>
#include "rb2.h"
#include "config.h"

static unsigned filter[NUMBER_OF_KEYS];
static QueueHandle_t keyqueue;
static int mb_short;
static bool mb_signaled;
static int dcc_short;
static bool dcc_signaled;
static volatile uint32_t active_keys;	// a bitmap representing active keys (index by key value)

#define KEY_PRESSED		1
#define KEY_STABLE		0
#define KEY_RELEASED	-1
#define KEY_QUEUELEN	20		///< number of queued up key events in the queue

#define FILTER_BITS		10								///< the length of the filter (in ms or bits)
#define FILTER_MASK		((1 << FILTER_BITS) - 1)		///< a bitmask to set or clear intermediate filter bits
#define FILTER_TEST		((1 << (FILTER_BITS - 1)) | 1)	///< the bits to test if the enclosing bits must be set or cleared
#define FLIP_POSITION	(3 << (FILTER_BITS - 1))		///< two bits which indicate the toggling of a key contact

static int key_filter (unsigned *f, bool state)
{
	*f <<= 1;
	if (state) *f |= 1;
	if ((*f & FILTER_TEST) == FILTER_TEST) {		// both bits are set: set all bits between the TEST-pattern "ends"
		*f |= FILTER_MASK;
	} else if ((*f & FILTER_TEST) == 0) {			// both bits are cleared: clear all bits between the TEST-pattern "ends"
		*f &= ~FILTER_MASK;
	}

	switch ((*f & FLIP_POSITION) >> (FILTER_BITS - 1)) {
		case 0b01: return KEY_PRESSED;
		case 0b10: return KEY_RELEASED;
	}
	return KEY_STABLE;
}

/**
 * Read and debounce the two pushbuttons and short-inputs from booster
 * interfaces.
 *
 * This function is called from vApplicationTickHook() in interrupt
 * context of the FreeRTOS scheduler (ms timing).
 */
void key_scan (void)
{
	struct sysconf *cfg;
	keyevent k;
	int keystatus;

	if (keyqueue == NULL) return;	// if we have no key queue, we can do nothing

	keystatus = key_filter(&filter[0], KEY1_PRESSED());	// check <TASTER1> -> GO KEY
	if (keystatus != KEY_STABLE) {
		if (keystatus == KEY_PRESSED) {
			k = MAKE(KEY_GO);
			active_keys |= (1 << KEY_GO);
		} else {
			k = BREAK(KEY_GO);
			active_keys &= ~(1 << KEY_GO);
		}
		xQueueSendToBackFromISR(keyqueue, &k, NULL);
	}
	keystatus = key_filter(&filter[1], KEY2_PRESSED());	// check <TASTER2> -> STOP KEY
	if (keystatus != KEY_STABLE) {
		if (keystatus == KEY_PRESSED) {
			k = MAKE(KEY_STOP);
			active_keys |= (1 << KEY_STOP);
		} else {
			k = BREAK(KEY_STOP);
			active_keys &= ~(1 << KEY_STOP);
		}
		xQueueSendToBackFromISR(keyqueue, &k, NULL);
	}
	keystatus = key_filter(&filter[2], BIDIBUS_ACK());	// check <BiDiBus-ACK> -> more than 10ms LOW means EMERGENCY STOP
	if (keystatus != KEY_STABLE) {
		if (keystatus == KEY_PRESSED) {
			k = MAKE(KEY_BIDIB_ACK);
			active_keys |= (1 << KEY_BIDIB_ACK);
		} else {
			k = BREAK(KEY_BIDIB_ACK);
			active_keys &= ~(1 << KEY_BIDIB_ACK);
		}
		xQueueSendToBackFromISR(keyqueue, &k, NULL);
	}

	cfg = cnf_getconfig();
	if (MB_ISSHORT()) mb_short += 2;		// incrementing at twice the rate of decrementing
	else if (mb_short > 0) mb_short--;
	if (!mb_signaled && mb_short > cfg->mmshort * 2) {
		k = MAKE(MB_SHORT);
		mb_signaled = true;
		active_keys |= (1 << MB_SHORT);
		xQueueSendToBackFromISR(keyqueue, &k, NULL);
	}
	if (DCC_ISSHORT()) dcc_short += 2;		// incrementing at twice the rate of decrementing
	else if (dcc_short > 0) dcc_short--;
	if (!dcc_signaled && dcc_short > cfg->dccshort * 2) {
		k = MAKE(DCC_SHORT);
		dcc_signaled = true;
		active_keys |= (1 << DCC_SHORT);
		xQueueSendToBackFromISR(keyqueue, &k, NULL);
	}
	if (mb_short > cfg->mmshort * 2) mb_short = cfg->mmshort * 2;		// don't let the counter run away
	if (dcc_short > cfg->dccshort * 2) dcc_short = cfg->dccshort * 2;	// don't let the counter run away
}

/**
 * Allocate the key event queue
 */
void key_init (void)
{
	if (keyqueue == NULL) {
		keyqueue = xQueueCreate(KEY_QUEUELEN, sizeof(keyevent));
		if (!keyqueue) {
			fprintf(stderr, "%s(): FATAL: cannot create key queue\n", __func__);
		}
	}
}

/**
 * get the latest event from the key queue
 */
keyevent key_getEvent (TickType_t waittime)
{
	BaseType_t rc;
	keyevent k;

	if (keyqueue == NULL) {
		key_init();					// retry queue creation
		if (waittime == portMAX_DELAY) waittime = 100;
		vTaskDelay(waittime);		// slow down a possible loop checking for keys
		return NOKEY;
	}

	rc = xQueueReceive(keyqueue, &k, waittime);

	if (rc == pdTRUE) return k;
	return NOKEY;
}

/**
 * called from signal generation when boosters are switched on.
 */
void key_resetShort (void)
{
	mb_short = dcc_short = 0;
	mb_signaled = dcc_signaled = false;
	active_keys &= ~((1 << MB_SHORT) | (1 << DCC_SHORT));
}

bool key_isActive (int key)
{
	return !!(active_keys & (1 << key));
}
