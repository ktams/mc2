/*
 * bidib.c
 *
 *  Created on: 19.12.2020
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
#include <string.h>
#include "rb2.h"
#include "events.h"
#include "lwip/sockets.h"
#include "config.h"
#include "bidib.h"

#define SYS_DISABLED			0x0001	///< we received a MSG_SYS_DISABLE - so no instant reporting allowed
#define IDENTIFY				0x0002	///< we have an active identify running

static struct {
	uint32_t			opflags;		///< operational status flags
	enum opmode			opmode;			///< wether we are SERVER or CONTROLLER
} status;

uint8_t myUID[BIDIB_UID_LEN];			///< my system UID (globaly available)
static uint16_t tickoffset;				///< offset BiDiB system time to our internal FreeRTOS tick counter, only 16 LSBits

/*
 * ===================================================================================
 * Communication interface for BiDiBus (the lower level)
 * ===================================================================================
 */

void bidib_busError (uint8_t errcode, uint8_t adr)
{
	bidibmsg_t *m = NULL;

	if (bidib_opmode() == BIDIB_CONTROLLER) {
		BDBctrl_busError(errcode, adr);
	} else {
		switch (errcode) {
			case BIDIB_ERR_SUBTIME:
			case BIDIB_ERR_SUBCRC:
			case BIDIB_ERR_SUBPAKET:
				m = bidib_errorMessage(LOCAL_NODE(), errcode, 1, &adr);
				break;
			default:
				break;
		}
		if (m) netBDB_postMessages(m);
	}
}

/*
 * ===================================================================================
 * Startup and helpers
 * ===================================================================================
 */

bidibmsg_t *bidib_errorMessage (struct bidibnode *n, uint8_t code, int len, uint8_t *extra)
{
	uint8_t data[64], *p;

	p = data;
	*p++ = code;
	if (len > 0 && extra) {
		if (len > (int) (sizeof(data) - 1)) len = sizeof(data) - 1;
		memcpy (p, extra, len);
	}
	return bidib_genMessage(n, MSG_SYS_ERROR, len + 1, data);
}

void bidib_extControl (bool on)
{
//	bidibmsg_t *m;

//	BDBnode_resetNodeList();
//	if ((m = bidib_genMessage(NULL, MSG_SYS_DISABLE, 0, NULL)) != NULL) BDBus_resetBus(m);
	if (on) {
		rt.ctrl |= EXTCTRL_BIDIB;
		status.opmode = BIDIB_SERVER;
		BDBsrv_updateFeatures();		// maybe some features have changed ... reflect that on new connection
	} else {
		rt.ctrl &= ~EXTCTRL_BIDIB;
		status.opmode = BIDIB_CONTROLLER;
		rgb_identify(false);			// turn off identification in case it was active before
	}
	event_fire(EVENT_EXTCONTROL, rt.ctrl, NULL);
}

enum opmode bidib_opmode (void)
{
	return status.opmode;
}

bool bidib_isSysDisabled (void)
{
	return !!(status.opflags & SYS_DISABLED);
}

void bidib_sysDisable (void)
{
	status.opflags |= SYS_DISABLED;
}

void bidib_sysEnable (void)
{
	status.opflags &= ~SYS_DISABLED;
}

uint16_t bidib_getSysTime (void)
{
	return ((xTaskGetTickCount() & 0xFFFF) + tickoffset) & 0xFFFF;
}

void bidib_identify (bool on)
{
	bidibmsg_t *m;
	uint8_t data[2];

	if (on) status.opflags |= IDENTIFY;
	else status.opflags &= ~IDENTIFY;
	rgb_identify(on);
	data[0] = (on) ? 1 : 0;
	if (bidib_opmode() == BIDIB_SERVER) {
		m = bidib_genMessage(LOCAL_NODE(), MSG_SYS_IDENTIFY_STATE, 1, data);
		if (m) netBDB_postMessages(m);
	}
}

void bidib_identifyToggle (void)
{
	bidib_identify (!(status.opflags & IDENTIFY));
}

/**
 * Start all tasks that are needed for the whole BiDiB system.
 * Initialise our own UID.
 */
void bidib_start (void)
{
	uint8_t *id = myUID;

#ifdef BIDIB_SNIFFER
	log_enable(LOG_BIDIB);
#else
	//log_enable(LOG_BIDIB);		// $$$$$
#endif

	*id++ = BIDIB_CLASS;					// Class
	*id++ = BIDIB_XCLASS;					// X-Class
	*id++ = hwinfo->manufacturer;			// manufacturer
	*id++ = BIDIB_PRODUCTID_TAMS;			// product
	*id++ = (hwinfo->serial >> 0) & 0xFF;	// serial LSB
	*id++ = (hwinfo->serial >> 8) & 0xFF;	// serial mid byte
	*id++ = (hwinfo->serial >> 16) & 0xFF;	// serial MSB

	BDBnode_resetNodeList();
	status.opmode = BIDIB_CONTROLLER;
	bidib_load();		// load settings from bidib.ini

	BDBsrv_start();
	netBDB_start();
	xTaskCreate(BDBus, "BiDiBus", 2048, NULL, 1, NULL);
	xTaskCreate(BDBctrl_controller, "BiDiB-Control", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
}
