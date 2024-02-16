/*
 * m3_pt.c
 *
 *  Created on: 27.11.2020
 *      Author: andi
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
#include "decoder.h"

static void m3pt_beacon (uint32_t beacon, uint16_t announce, int packets)
{
	struct packet *p;

	while (packets > 0) {
		if ((p = sigq_m3BeaconPacket(beacon, announce, 1)) != NULL) {
			sigq_queuePacket(p);
			vTaskDelay(100);
		}
		packets--;
	}
}

uint32_t m3pt_getUID (void)
{
	enum trackmode old_tm;
	uint32_t uid, beacon;
	uint16_t announce;
	int rc = 0;

	old_tm = sig_getMode();
	beacon = sig_getM3Beacon();
	announce = sig_getM3AnnounceCounter();
	log_msg(LOG_INFO, "%s() starting prog track BEACON %ld:%d\n", __func__, beacon, announce);
	sig_setMode(TM_TESTDRIVE);
	while (!ts_voltageLevelReached()) vTaskDelay(10);	// wait for voltage to reach it's level
	vTaskDelay(250);									// give the decoder on the track a little more time

	log_msg(LOG_INFO, "%s() sending deregister-beacon\n", __func__);
	m3pt_beacon(beacon ^ 0xAA55, announce ^ 0xA55A, 10);

	log_msg(LOG_INFO, "%s() sending standard-beacon\n", __func__);
	m3pt_beacon(beacon, announce, 10);

	log_msg(LOG_INFO, "%s() calling sig_searchM3Loco()\n", __func__);
	uid = 0;
	rc = sig_searchM3Loco (&uid);
	log_msg(LOG_INFO, "%s() sig_searchM3Loco() = %d, UID 0x%08lx\n", __func__, rc, uid);

	log_msg(LOG_INFO, "%s() ending prog track\n", __func__);
	sig_setMode(old_tm);
	return (rc > 0) ? uid : 0;
}

int m3pt_setAddress (uint32_t uid, int adr)
{
	struct packet *p;
	enum trackmode old_tm;
	uint32_t beacon;
	uint16_t announce;
	int rc = -1;

	old_tm = sig_getMode();
	beacon = sig_getM3Beacon();
	announce = sig_getM3AnnounceCounter();
	printf ("%s() starting prog track BEACON %ld:%d\n", __func__, beacon, announce);
	sig_setMode(TM_TESTDRIVE);
	while (!ts_voltageLevelReached()) vTaskDelay(10);	// wait for voltage to reach it's level
	vTaskDelay(250);									// give the decoder on the track a little more time

	printf ("%s() sending standard-beacon\n", __func__);
	m3pt_beacon(beacon, announce, 10);


	printf ("%s() assigning UID %lu address %d\n", __func__, uid, adr);
	if ((p = sigq_m3NewAddress(uid, adr)) != NULL) {
		sigq_queuePacket(p);
		while (!sigq_isIdle()) vTaskDelay(10);
		vTaskDelay(500);								// give the packet a little time to be sent out
	}
	m3pt_beacon(beacon, announce, 3);
	while (!sigq_isIdle()) vTaskDelay(10);
	vTaskDelay(2000);								// give the packet a little time to be sent out

	printf ("%s() ending prog track\n", __func__);
	sig_setMode(old_tm);
	return rc;
}

int m3pt_readCV (int adr, cvadrT cva, int bytes, int repeat, reply_handler handler, flexval priv)
{
	struct packet *p;
	enum trackmode old_tm;
	uint32_t beacon;
	uint16_t announce;

	old_tm = sig_getMode();
	beacon = sig_getM3Beacon();
	announce = sig_getM3AnnounceCounter();
	log_msg (LOG_INFO, "%s() starting prog track BEACON %ld:%d\n", __func__, beacon, announce);
	sig_setMode(TM_TESTDRIVE);
	while (!ts_voltageLevelReached()) vTaskDelay(10);	// wait for voltage to reach it's level
	m3pt_beacon(beacon, announce, 2);
	vTaskDelay(250);									// give the decoder on the track a little more time

	log_msg (LOG_INFO, "%s() sending standard-beacon\n", __func__);
	m3pt_beacon(beacon, announce, 10);

	log_msg (LOG_INFO, "%s() ADR %d read CV %d.%d (%d bytes)\n", __func__, adr, cva.m3cv, cva.m3sub, bytes);
	while (repeat > 0) {
		if ((p = sigq_m3ReadCV(adr, cva, bytes, handler, priv)) != NULL) {
			sigq_queuePacket(p);
			while (!sigq_isIdle()) vTaskDelay(5);
		}
		if ((p = sigq_m3BeaconPacket(beacon, announce, 1)) != NULL) {
			sigq_queuePacket(p);
			while (!sigq_isIdle()) vTaskDelay(5);
		}
		repeat--;
	}
	vTaskDelay(500);								// give the packet a little time to be sent out
	m3pt_beacon(beacon, announce, 3);
	while (!sigq_isIdle()) vTaskDelay(10);
	vTaskDelay(2000);								// give the packet a little time to be sent out

	log_msg (LOG_INFO, "%s() ending prog track\n", __func__);
	sig_setMode(old_tm);
	return 0;
}

int m3pt_writeCV (int adr, cvadrT cva, uint8_t val, int repeat)
{
	struct packet *p;
	enum trackmode old_tm;
	uint32_t beacon;
	uint16_t announce;

	old_tm = sig_getMode();
	beacon = sig_getM3Beacon();
	announce = sig_getM3AnnounceCounter();
	printf ("%s() starting prog track BEACON %ld:%d\n", __func__, beacon, announce);
	sig_setMode(TM_TESTDRIVE);
	while (!ts_voltageLevelReached()) vTaskDelay(10);	// wait for voltage to reach it's level
	vTaskDelay(250);									// give the decoder on the track a little more time

	log_msg (LOG_INFO, "%s() sending standard-beacon\n", __func__);
	m3pt_beacon(beacon, announce, 10);

	log_msg (LOG_INFO, "%s() ADR %d write CV %d.%d = %d\n", __func__, adr, cva.m3cv, cva.m3sub, val);
	if ((p = sigq_m3WriteCV(adr, cva, val, repeat)) != NULL) {
		sigq_queuePacket(p);
		while (!sigq_isIdle()) vTaskDelay(10);
		vTaskDelay(500);								// give the packet a little time to be sent out
	}
	m3pt_beacon(beacon, announce, 3);
	while (!sigq_isIdle()) vTaskDelay(10);
	vTaskDelay(2000);								// give the packet a little time to be sent out

	log_msg (LOG_INFO, "%s() ending prog track\n", __func__);
	sig_setMode(old_tm);
	return 0;
}
