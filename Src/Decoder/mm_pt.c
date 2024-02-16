/*
 * mm_pt.c
 *
 *  Created on: 20.03.2021
 *      Author: Andi
 */

/*
 * RB2, next generation model railroad controller software
 * Copyright (C) 2021 Tams Elektronik GmbH and Andreas Kretzer
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

#include "rb2.h"
#include "decoder.h"

static bool mmpt_findDecoderRange (int basecurrent, int from, int to)
{
	static bool fwd;

	struct packet *pkt;
	int loco, trackcurrent;
	int i;

	if (from < MIN_LOCO_ADR) from = MIN_LOCO_ADR;
	if (to > MAX_MM_ADR) to = MAX_MM_ADR;
	while (!sigq_isIdle()) vTaskDelay(2);
	for (i = 0; i < 3; i++) {
		for (loco = from; loco <= to; loco++) {
			if ((pkt = sigq_genPacket(NULL, FMT_MM2_14, QCMD_SETSPEED)) != NULL) {
				pkt->adr = loco;
				pkt->value.i32 = (fwd) ? 0x88 : 8;
				pkt->funcs[0] = FUNC_LIGHT | FUNC(3) | FUNC(4);
				sigq_queuePacket(pkt);
			}
		}
		while (!sigq_isIdle()) vTaskDelay(2);
	}
	while (!sigq_isIdle()) vTaskDelay(2);
	vTaskDelay(600);
	trackcurrent = an_getTrackCurrent();
//	for (i = 0; i < 3; i++) {
		for (loco = from; loco <= to; loco++) {
			if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_MM_REVERSE)) != NULL) {
				pkt->adr = loco;
				pkt->repeat = 8;
				sigq_queuePacket(pkt);
			}
		}
		while (!sigq_isIdle()) vTaskDelay(2);
//	}

	i = 0;
	while (i < 5) {
		vTaskDelay(5);
		if (an_getTrackCurrent() < basecurrent + 5) i++;
	}
	fwd = !fwd;
	return (trackcurrent > (basecurrent + 30));
}

static void mmpt_stopDecoders (int from, int to)
{
	struct packet *pkt;
	int loco;

	if (from < MIN_LOCO_ADR) from = MIN_LOCO_ADR;
	if (to > MAX_MM_ADR) to = MAX_MM_ADR;
	log_msg (LOG_INFO, "%s(): stoppig from %d to %d\n", __func__, from, to);
	while (!sigq_isIdle()) vTaskDelay(10);
	for (loco = from; loco <= to; loco++) {
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_SETSPEED)) != NULL) {
			pkt->adr = loco;
			pkt->repeat = 2;
			sigq_queuePacket(pkt);
		}
	}
	while (!sigq_isIdle()) vTaskDelay(10);
}

int mmpt_findDecoder (int from, int to)
{
	int basecurrent;
	int adr, count, end, loco;

	if (from < MIN_LOCO_ADR) from = MIN_LOCO_ADR;
	if (to > MAX_MM_ADR) to = MAX_MM_ADR;
	log_msg (LOG_INFO, "%s() from %d to %d\n", __func__, from, to);

	sig_setMode(TM_TAMSPROG);
	adc_CCmonitor(600);		// current limiter
	while (!ts_voltageLevelReached()) vTaskDelay(10);	// wait for voltage to reach it's level
	vTaskDelay(500);
	mmpt_stopDecoders(from, to);
	vTaskDelay(500);

	basecurrent = an_getTrackCurrent();
	log_msg (LOG_INFO, "%s(): Base current %dmA @ %d.%dV\n", __func__, basecurrent, ts_getPtVoltage() / 10, ts_getPtVoltage() % 10);

	adr = from;
	count = 16;
	end = to;
	loco = 0;
	do {
		if ((adr + count - 1) > to) count = to - adr + 1;
		log_msg (LOG_INFO, "%s() search %d -> %d\n", __func__, adr, adr + count - 1);
		if (mmpt_findDecoderRange(basecurrent, adr, adr + count - 1)) {
			end = adr + count;
			log_msg (LOG_INFO, "%s() found in range %d -> %d\n", __func__, adr, end - 1);
			if (count == 1) loco = adr;
			count >>= 1;
		} else {
			adr += count;
		}
		if (count == 0 || adr >= end) break;
	} while (!loco);

	adc_CCmonitor(0);
	sig_setMode(TM_STOP);

	log_msg (LOG_INFO, "%s() loco = %d\n", __func__, loco);
	return loco;
}

int mmpt_enterProgram (int adr)
{
	struct packet *pkt;

	log_msg (LOG_INFO, "%s() %d\n", __func__, adr);
	if (adr < MIN_LOCO_ADR || adr > MAX_MM_ADR) adr = 80;
	if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_MM_REVERSE)) != NULL) {
		pkt->repeat = 75;
		pkt->adr = adr;
		sig_setMode(TM_GO);
		sigq_queuePacket(pkt);
	}
	return 0;
}

/**
 * The "modern" way of writing CVs to MM decoders.
 * This is accomplished by alternating loco addresses with reversing direction
 * first for the CV address and then for CV value.
 * The address of the loco should be 78 (older TAMS decoders) or 80 (Märklin
 * and newer TAMS decoders).
 */
int mmpt_cvProg (int adr, int cv, int val)
{
	struct packet *pkt;

	log_msg (LOG_INFO, "%s(%d) CV%d = %d\n", __func__, adr, cv, val);
	if (adr < MIN_LOCO_ADR || adr > MAX_MM_ADR) adr = 80;		// senseful default
	if (cv <= 0 || cv > MAX_MM_ADR) return -1;
	if (val < 0 || val > 255) return -2;
	if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_MM_REVERSE)) != NULL) {
		pkt->repeat = 200;
		pkt->adr = adr;
		sig_setMode(TM_TAMSPROG);
		adc_CCmonitor(600);		// current limiter
		sigq_queuePacket(pkt);
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_SETSPEED)) != NULL) {
			pkt->repeat = 100;
			pkt->adr = adr;
			sigq_queuePacket(pkt);
		}
		while (!ts_voltageLevelReached()) vTaskDelay(10);	// wait for voltage to reach it's level

		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_SETSPEED)) != NULL) {
			pkt->repeat = 200;
			pkt->adr = cv;
			sigq_queuePacket(pkt);
		}
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_MM_REVERSE)) != NULL) {
			pkt->repeat = 300;
			pkt->adr = cv;
			sigq_queuePacket(pkt);
		}
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_SETSPEED)) != NULL) {
			pkt->repeat = 200;
			pkt->adr = cv;
			sigq_queuePacket(pkt);
		}

		vTaskDelay(500);
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_SETSPEED)) != NULL) {
			pkt->repeat = 200;
			pkt->adr = val;
			sigq_queuePacket(pkt);
		}
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_MM_REVERSE)) != NULL) {
			pkt->repeat = 300;
			pkt->adr = val;
			sigq_queuePacket(pkt);
		}
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_SETSPEED)) != NULL) {
			pkt->repeat = 300;
			pkt->adr = val;
			sigq_queuePacket(pkt);
		}
		vTaskDelay(15000);
	}

	sig_setMode(TM_STOP);
	return 0;
}

int mmpt_tamsLDW2address (int adr)
{
	struct packet *pkt;

	log_msg (LOG_INFO, "%s() %d\n", __func__, adr);
	if (adr < MIN_LOCO_ADR || adr > MAX_MM_ADR) return -1;
	if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_MM_REVERSE)) != NULL) {
		pkt->repeat = 75;
		pkt->adr = 78;
		sig_setMode(TM_TAMSPROG);
		adc_CCmonitor(600);		// current limiter
		sigq_queuePacket(pkt);
		while (!ts_voltageLevelReached()) vTaskDelay(10);	// wait for voltage to reach it's level

		vTaskDelay(3000);
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_SETSPEED)) != NULL) {
			pkt->repeat = 10;
			pkt->adr = adr;
			pkt->funcs[0] = FUNC_LIGHT;
			sigq_queuePacket(pkt);
		}
		vTaskDelay(500);
		if ((pkt = sigq_genPacket(NULL, FMT_MM1_14, QCMD_SETSPEED)) != NULL) {
			pkt->repeat = 10;
			pkt->adr = adr;
			sigq_queuePacket(pkt);
		}
		vTaskDelay(1000);
	}

	sig_setMode(TM_STOP);
	return 0;
}

/**
 * Serial online programming of old TAMS Decoders (LD-W3).
 * Up to four settings can be programmed: address, start speed,
 * maximum speed and accellearation / decelleration.
 *
 * The decoder address is always programmed, other settings are
 * coded with speed (1-14 or 1-27?) and skipped when a zero speed defined.
 * The settings are accepted when the "function" is switched on (and off).
 */
int mmpt_onlineLDW3Prog (int adr, int startspeed, int maxspeed, int speedramp)
{
	/* currently not implemented */
	(void) adr;
	(void) startspeed;
	(void) maxspeed;
	(void) speedramp;

	return 0;
}

/**
 * Program the decoder address of a LDW11 decoder using MM2 with F0
 */
int mmpt_tamsLDW11Adr (int adr)
{
	/* currently not implemented */
	(void) adr;

	return 0;
}

/**
 * Factory reset a LD-W11 decoder using MM2 with F2
 */
int mmpt_tamsLDW11Reset (void)
{
	/* currently not implemented */
	return 0;
}

/**
 * Setting accelleration and decelleration on a LD-W11 decoder using MM2 with F1
 */
int mmpt_tamsLDW11Accel (int accel)
{
	/* currently not implemented */
	(void) accel;

	return 0;
}

/**
 * Setting start and maximum speed on a LD-W11 decoder using MM2 with F3
 */
int mmpt_tamsLDW11MinMaxSpeed (int minspeed, int maxspeed)
{
	/* currently not implemented */
	(void) minspeed;
	(void) maxspeed;

	return 0;
}
