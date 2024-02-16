/**
 * @file dcc_pt.c
 *
 * @author Andi
 * @date   03.05.2020
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
#include <string.h>
#include "rb2.h"
#include "decoder.h"

#define CURRENT_OBSERVATION_LENGTH		50		///< the length of the queue to check for a stablilised current as base current for ACK recognition
#define CURRENT_WINDOW					3		///< plus / minus this value forms the stabilisation window for decoder current before start programming
#define CURRENT_TIMEOUT					5000	///< we don't wait longer than that for a current to stabilise
#define MIN_CURRENT						0		///< the minimum current we expect
#define DCCCV_READ_TIME					50		///< reply window for read commands
#define DCCCV_WRITE_TIME				100		///< reply window for write commands
#define ACK_PULSE_CURRENT				60		///< we need a 60mA pulse at least
#define ACK_COUNT_THRESHOLD				1000	///< the number of recognised ACK threshold current consumption events @400kHz sampling rate (> 1,9ms)
#define MAX_RETRIES						3		///< we try only a limited number of attempts
#define DCC_CV_REPEAT					5

static volatile int ack_count;					///< incremented by ACK-Current callback

typedef int (*ptCoreFunc)(int idleCurrent, int cv, int bit, uint8_t data);

/**
 * Wait for decoder current to settle to a stable value inside the defined (relative) window.
 *
 * \return		an average idle current in mA on success or -1 on timeout
 */
static int dccpt_waitStableCurrent (void)
{
	int base[CURRENT_OBSERVATION_LENGTH];
	TickType_t start;
	int cidx, c, shortcnt;;

	memset (base, 0, sizeof(base));
	cidx = c = shortcnt = 0;
	start = xTaskGetTickCount();

	do {
		if ((xTaskGetTickCount() - start) > CURRENT_TIMEOUT) {
			if (c < MIN_CURRENT) return ERR_NO_LOCO;
			return ERR_UNSTABLE;
		}
		vTaskDelay(1);
		c = an_getProgCurrent(10);
		if (c > 200) {
			if (++shortcnt > 10) return ERR_SHORT;
		}
		base[cidx] = c;
		if (++cidx >= CURRENT_OBSERVATION_LENGTH) cidx = 0;
	} while ((c < MIN_CURRENT) || (c > (base[cidx] + 3)) || (c < (base[cidx] - 3)));

	for (cidx = 0, c = 0; cidx < CURRENT_OBSERVATION_LENGTH; cidx++) {
		if (base[cidx] > c) c = base[cidx];
	}
	return c;
}

static void dccpt_ACKcb (int adval)
{
	(void) adval;

	ack_count++;
}

static int dccpt_waitack (int cidle, TickType_t waittime)
{
	int rc, waitcnt;

	while (!sigq_isIdle()) taskYIELD();		// wait until the program or read packet is fetched from queue
	vTaskDelay(10);		// wait an additional time
	adc_ACKcurrent(cidle + ACK_PULSE_CURRENT, dccpt_ACKcb);
	ack_count = 0;
	vTaskDelay(waittime);		// this is the ACK window for reading
	rc = ack_count;
	waitcnt = 100;
	do {
		ack_count = 0;
		vTaskDelay (2);
		waitcnt--;
	} while (ack_count > 5 && waitcnt > 0);		// wait until ACK from decoder is finished (or timeout occures)
	adc_ACKcurrent(0, NULL);
	return rc;
}

static int dccpt_readByteCore (int idle, int cv, int b, uint8_t data)
{
	int bit, ack0, ack1;

	(void) b;

	data = 0;

	for (bit = 0; bit < 8; bit++) {
		sigq_dcc_cvVerfyBit(cv, bit, false, DCC_CV_REPEAT);
		ack0 = dccpt_waitack(idle, DCCCV_READ_TIME);
		log_msg (LOG_INFO, "Bit %d [0]: %d\n", bit, ack0);

		if (bit == 0) {
			sigq_dcc_cvVerfyBit(cv, bit, true, DCC_CV_REPEAT);
			ack1 = dccpt_waitack(idle, DCCCV_READ_TIME);
			log_msg (LOG_INFO, "Bit %d [1]: %d\n", bit, ack1);
			if ((ack0 < (ACK_COUNT_THRESHOLD / 2)) && (ack1 > ACK_COUNT_THRESHOLD)) {
				data |= 1;		// in this case we are at bit0
			} else if ((ack1 < (ACK_COUNT_THRESHOLD / 2)) && (ack0 < (ACK_COUNT_THRESHOLD / 2))) {
				return ERR_NO_LOCO;
			} else if ((ack1 > ACK_COUNT_THRESHOLD) && (ack0 > ACK_COUNT_THRESHOLD)) {
				return ERR_CV_UNSUPPORTED;
			}
		} else if (ack0 < (ACK_COUNT_THRESHOLD / 2)) {
			data |= 1 << bit;
		}
	}

	sigq_dcc_cvVerfyByte(cv, data, 10);
	ack0 = dccpt_waitack(idle, DCCCV_READ_TIME);
	log_msg (LOG_INFO, "Byte [0x%02x]: %d\n", data, ack0);
	if (ack0 > (ACK_COUNT_THRESHOLD)) return data;		// SUCCESS
	return ERR_CV_COMPARE;
}

static int dccpt_readBitCore (int idle, int cv, int b, uint8_t data)
{
	int ack0, ack1;

	(void) data;

	sigq_dcc_cvVerfyBit(cv, b, false, DCC_CV_REPEAT);
	ack0 = dccpt_waitack(idle, DCCCV_READ_TIME);
	log_msg (LOG_INFO, "Bit %d [0]: %d\n", b, ack0);
	sigq_dcc_cvVerfyBit(cv, b, true, DCC_CV_REPEAT);
	ack1 = dccpt_waitack(idle, DCCCV_READ_TIME);
	log_msg (LOG_INFO, "Bit %d [1]: %d\n", b, ack1);

	if ((ack0 < (ACK_COUNT_THRESHOLD / 2)) && (ack1 > ACK_COUNT_THRESHOLD)) {
		return 1;
	} else if ((ack1 < (ACK_COUNT_THRESHOLD / 2)) && (ack0 > ACK_COUNT_THRESHOLD)) {
		return 0;
	}

	return ERR_CV_UNSUPPORTED;
}

static int dccpt_verifyBitCore (int idle, int cv, int b, uint8_t data)
{
	int ack;

	sigq_dcc_cvVerfyBit(cv, b, !!data, DCC_CV_REPEAT);
	ack = dccpt_waitack(idle, DCCCV_READ_TIME);
	log_msg (LOG_INFO, "Bit %d [%d]: %d\n", b, (data) ? 1 : 0, ack);

	return (ack > ACK_COUNT_THRESHOLD) ? 1 : ERR_CV_COMPARE;
}

static int dccpt_writeBitCore (int idle, int cv, int b, uint8_t data)
{
	int ack;

	sigq_dcc_cvWriteBit(cv, b, data, 10);
	ack = dccpt_waitack(idle, DCCCV_WRITE_TIME);
	log_msg (LOG_INFO, "Bit %d [%d]: %d\n", b, data, ack);
	if (ack > (ACK_COUNT_THRESHOLD)) return data;		// SUCCESS
	return ERR_CV_WRITE;
}

static int dccpt_writeByteCore (int idle, int cv, int b, uint8_t data)
{
	int ack;

	(void) b;

	sigq_dcc_cvWriteByte(cv, data, 10);
	ack = dccpt_waitack(idle, DCCCV_WRITE_TIME);
	log_msg (LOG_INFO, "Byte [0x%02x]: %d\n", data, ack);
	if (ack > (ACK_COUNT_THRESHOLD)) return data;		// SUCCESS
	return ERR_CV_WRITE;
}

static int dccpt_loopFunction (int cv, int b, uint8_t data, ptCoreFunc func)
{
	int idle, tries, rc;

	if (cv < MIN_DCC_CVADR || cv > MAX_DCC_CVADR) return ERR_INTERNAL;
	if (!func) return ERR_INTERNAL;

	log_msg (LOG_INFO, "%s() START\n", __func__);
	tries = 0;

	while (tries < MAX_RETRIES) {
		sig_setMode(TM_STOP);
		adc_CCmonitor(0);
		tries++;
		rc = 0;
		adc_CCmonitor(400);		// current limiter
		sig_setMode(TM_DCCPROG);
		while (!ts_voltageLevelReached()) vTaskDelay(10);	// wait for voltage to reach it's level
		log_msg (LOG_INFO, "%s(): Voltage stable\n", __func__);
		vTaskDelay(700);									// give the decoder on the track a little more time

		idle = dccpt_waitStableCurrent();
		log_msg (LOG_INFO, "%s(): IDLE-current %dmA\n", __func__, idle);
		if (idle < 0) {
			rc = idle;
			continue;
		}
		if ((rc = func(idle, cv, b, data)) >= 0) break;		// SUCCESS
	}

	if (rc < 0) {
		log_msg (LOG_INFO, "%s(): CV %d => ERR %d\n", __func__, cv + 1, rc);
	} else {
		log_msg (LOG_INFO, "%s(): CV %d = 0x%02x (%d)\n", __func__, cv + 1, rc, rc);
	}

	sig_setMode(TM_STOP);
	adc_CCmonitor(0);
	return rc;
}

int dccpt_cvReadByte (int cv)
{
	return dccpt_loopFunction(cv, 0, 0, dccpt_readByteCore);
}

int dccpt_cvWriteByte (int cv, uint8_t data)
{
	return dccpt_loopFunction(cv, 0, data, dccpt_writeByteCore);
}

int dccpt_cvReadBit (int cv, int bit)
{
	return dccpt_loopFunction(cv, bit, 0, dccpt_readBitCore);
}

int dccpt_cvWriteBit (int cv, int bit, uint8_t data)
{
	return dccpt_loopFunction(cv, bit, data, dccpt_writeBitCore);
}

static struct {
	int			cv;								// the CV to operate on
	int			b;								// a possible bit position
	uint8_t 	data;							// a value
	void		(*cb)(int rc, void *priv);		// the requested callback function (may be NULL)
	void		*priv;							// a private parameter for the callback (may be NULL)
} bgnd;

static void dccpt_loopBackground (void *pvParameter)
{
	ptCoreFunc	func;							// the core func
	int rc;

	func = (ptCoreFunc) pvParameter;
	rc = dccpt_loopFunction(bgnd.cv, bgnd.b, bgnd.data, func);
	if (bgnd.cb) bgnd.cb (rc, bgnd.priv);
	vTaskDelete(NULL);		// end this task
}

void dccpt_cvReadByteBG (int cv, void (*cb)(int, void *), void *priv)
{
	bgnd.cv = cv;
	bgnd.b = bgnd.data = 0;
	bgnd.cb = cb;
	bgnd.priv = priv;
	if (xTaskCreate(dccpt_loopBackground, "PT-LOOP", configMINIMAL_STACK_SIZE, dccpt_readByteCore, 1, NULL) != pdPASS) {
		if (cb) cb (ERR_INTERNAL, priv);
	}
}

void dccpt_cvWriteByteBG (int cv, uint8_t data, void (*cb)(int, void *), void *priv)
{
	bgnd.cv = cv;
	bgnd.b = 0;
	bgnd.data = data;
	bgnd.cb = cb;
	bgnd.priv = priv;
	if (xTaskCreate(dccpt_loopBackground, "PT-LOOP", configMINIMAL_STACK_SIZE, dccpt_writeByteCore, 1, NULL) != pdPASS) {
		if (cb) cb (ERR_INTERNAL, priv);
	}
}

void dccpt_cvReadBitBG (int cv, int bit, void (*cb)(int, void *), void *priv)
{
	bgnd.cv = cv;
	bgnd.b = bit;
	bgnd.data = 0;
	bgnd.cb = cb;
	bgnd.priv = priv;
	if (xTaskCreate(dccpt_loopBackground, "PT-LOOP", configMINIMAL_STACK_SIZE, dccpt_readBitCore, 1, NULL) != pdPASS) {
		if (cb) cb (ERR_INTERNAL, priv);
	}
}

void dccpt_cvVerifyBitBG (int cv, int bit, uint8_t data, void (*cb)(int, void *), void *priv)
{
	bgnd.cv = cv;
	bgnd.b = bit;
	bgnd.data = data;
	bgnd.cb = cb;
	bgnd.priv = priv;
	if (xTaskCreate(dccpt_loopBackground, "PT-LOOP", configMINIMAL_STACK_SIZE, dccpt_verifyBitCore, 1, NULL) != pdPASS) {
		if (cb) cb (ERR_INTERNAL, priv);
	}
}

void dccpt_cvWriteBitBG (int cv, int bit, uint8_t data, void (*cb)(int, void *), void *priv)
{
	bgnd.cv = cv;
	bgnd.b = bit;
	bgnd.data = data;
	bgnd.cb = cb;
	bgnd.priv = priv;
	if (xTaskCreate(dccpt_loopBackground, "PT-LOOP", configMINIMAL_STACK_SIZE, dccpt_writeBitCore, 1, NULL) != pdPASS) {
		if (cb) cb (ERR_INTERNAL, priv);
	}
}

