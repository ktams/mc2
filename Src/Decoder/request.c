/*
 * request.c
 *
 *  Created on: 26.03.2021
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

/**
 * @file request.c
 *
 * We need to diverge requests from external controls to the BiDiB system
 * or the real controlmechanism depending on wether we are currently controlled
 * by a BiDiB client or not.
 *
 * If we are controlled by BiDiB, all requests regarding speed and function for
 * mobile decoders and switching accessories should be routed thru the BiDiB
 * system. I.e., the request is sent to the Client and the client than is sending
 * the requests back to us, if it is willing to do so.
 *
 * The external control may be on these interfaces:
 *   - EasyNet
 *   - XPressNet
 *   - LocoNet
 *   - MCan
 *   - Sniffer
 *   - HTML client
 */

#include "rb2.h"
#include "bidib.h"

#define FILTER		1	/* deactive the filter for now (WDP) */

static void rq_fillFuncs (uint32_t funcs, uint8_t *data)
{
	if (data) {
		data[0] = ((funcs & FUNC_F1_F4) >> 1) | ((funcs & FUNC_LIGHT) << 4);		// 3 reserved MSBs, FL, F4, F3, F2, F1
		data[1] = (funcs & FUNC_F5_F12) >> 5;
		data[2] = (funcs & FUNC_F13_F20) >> 13;
		data[3] = (funcs & FUNC_F21_F28) >> 21;
	}
}

/**
 * Internal function to setup the data packet for the BiDiB request.
 *
 * \param l			the pointer to the loco data to get format information from
 * \param active	the bitmask that defines which parts should be marked active in the BiDiB request
 * \param speed		the new speed (if BIDIB_CS_DRIVE_SPEED_BIT is set in 'active')
 * \param newfuncs	the new functions to set, depending on the various bits in 'active'
 * \return			0 if everything is OK or a failure code otherwise
 */
static int rq_csDriveManual (ldataT *l, uint8_t active, int speed, uint32_t newfuncs)
{
	bidibmsg_t *m;
	uint8_t data[16];

	if (!l) return -1;
	data[0] = l->loco->adr & 0xFF;
	data[1] = (l->loco->adr >> 8) & 0xFF;
	data[2] = bidib_fmt2code(l->loco->fmt);
	data[3] = active;
	data[4] = speed;
	rq_fillFuncs(newfuncs, &data[5]);
	if ((m = bidib_genMessage(LOCAL_NODE(), MSG_CS_DRIVE_MANUAL, 9, data)) == NULL) return -2;
	netBDB_postMessages(m);
	return 0;
}

/**
 * Request a function setting from external control and forward it either
 * to BiDiB or directly to the system function.
 *
 * \param adr		the loco address we wish to control
 * \param newfuncs	the new function bits for F0 to F31 max.
 * \param mask		a mask that defines which functions to set
 * \return			0 if everything is OK, an errorcode otherwise
 */
int rq_setFuncMasked (int adr, uint32_t newfuncs, uint32_t mask)
{
	ldataT *l;
	uint8_t active;

	if (FILTER && bidib_opmode() == BIDIB_SERVER) {
		if ((l = loco_call(adr, true)) == NULL) return -1;
		if ((l->funcs[0] & mask) == (newfuncs & mask)) return 0;
		active = 0;
		if (mask & FUNC_F0_F4) active |= BIDIB_CS_DRIVE_F0F4_BIT;
		if (mask & FUNC_F5_F8) active |= BIDIB_CS_DRIVE_F5F8_BIT;
		if (mask & FUNC_F9_F12) active |= BIDIB_CS_DRIVE_F9F12_BIT;
		if (mask & FUNC_F13_F20) active |= BIDIB_CS_DRIVE_F13F20_BIT;
		if (mask & FUNC_F21_F28) active |= BIDIB_CS_DRIVE_F21F28_BIT;
		return rq_csDriveManual(l, active, bidib_speed2msg(l->speed, l->loco->fmt), newfuncs);
	} else {
		return loco_setFuncMasked(adr, newfuncs, mask);
	}
	return 0;
}

/**
 * Request a speed change from external control and forward it either
 * to BiDiB or directly to the system function.
 *
 * \param adr		the loco address we wish to control
 * \param speed		the new speed that should be set
 * \return			0 if everything is OK, an errorcode otherwise
 */
int rq_setSpeed (int adr, int speed)
{
	ldataT *l;

	if (FILTER && bidib_opmode() == BIDIB_SERVER) {
		if ((l = loco_call(adr, true)) == NULL) return -1;
		if (l->speed == speed) return 0;
		return rq_csDriveManual(l, BIDIB_CS_DRIVE_SPEED_BIT, bidib_speed2msg(speed, l->loco->fmt), l->funcs[0]);
	} else {
		return loco_setSpeed(adr, speed);
	}
	return 0;
}

/**
 * Request an emergency stop from external control and forward it either
 * to BiDiB or directly to the system function.
 *
 * \param adr		the loco address we wish to stop
 * \return			0 if everything is OK, an errorcode otherwise
 */
int rq_emergencyStop (int adr)
{
	ldataT *l;

	if (FILTER && bidib_opmode() == BIDIB_SERVER) {
		if ((l = loco_call(adr, true)) == NULL) return -1;
		return rq_csDriveManual(l, BIDIB_CS_DRIVE_SPEED_BIT, (l->speed & 0x80) | 1, l->funcs[0]);
	} else {
		return loco_emergencyStop(adr);
	}
	return 0;
}
