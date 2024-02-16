/*
 * defaults.h
 *
 *  Created on: 15.12.2020
 *      Author: ktams
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

#ifndef DEFAULTS_H_
#define DEFAULTS_H_

#define 	CNF_DEF_IPMETHOD			IPMETHOD_DHCP
#define 	CNF_DEF_P50_port			8050				// P50 -> P = ASCII 80, 50 -> 50, so we take 8050 as default
#define 	CNF_DEF_BIDIB_port			62875				// netBiDiB default as defined at bidib.org
#define		CNF_DEF_BIDIB_user			"(default)"			// a configurable user supplied name of the device
#define 	CNF_DEF_Sysflags			SYSFLAG_LIGHTEFFECTS	// Lighteffects on
#define 	CNF_DEF_Sigflags			(SIGFLAG_RAILCOM | SIGFLAG_DCCA | SIGFLAG_M3ENABLED)
#define 	CNF_DEF_Locopurge			20					// 20 Minutes to purge a unused address

#define 	CNF_DEF_mmshort				100					// external MM booster short after 100ms
#define 	CNF_DEF_dccshort			100					// external DCC booster short after 100ms

#define		CNF_DEF_s88modules			1
#define 	CNF_DEF_s88frequency		2000
#define 	CNF_DEF_s88frequency		2000

#define		CNF_DEF_mmrepeat			3
#define		CNF_DEF_MAXmmrepeat			10
#define 	CNF_DEF_mminterpck_fast		625
#define 	CNF_DEF_mminterpck_slow		1250
#define 	CNF_DEF_mmpause				1500
#define 	CNF_DEF_MAXmmpause			5000
#define 	CNF_DEF_MINmmpause			1000

#define 	CNF_DEF_dccrepeat			3
#define 	CNF_DEF_MAXdccrepeat		10
#define 	CNF_DEF_dccpomrepeat		3
#define 	CNF_DEF_MAXdccpomrepeat		30
#define 	CNF_DEF_dccpreamble			16
#define 	CNF_DEF_MAXdccpreamble		30
#define 	CNF_DEF_tailbits			2
#define		CNF_DEF_rc_tailbits			4
#define 	CNF_DEF_dcctim_one			116
#define 	CNF_DEF_MAXdcctim_one		150
#define 	CNF_DEF_MINdcctim_one		80
#define 	CNF_DEF_dcctim_zero			200
#define 	CNF_DEF_MAXdcctim_zero		800
#define 	CNF_DEF_MINdcctim_zero		160

#define 	CNF_DEF_m3repeat			3
#define 	CNF_DEF_MAXm3repeat			10
#define		CNF_DEF_m3beacon			0x1234		///< All RBs should use this Station-ID
#define		CNF_DEF_m3announce			0xA5		///< this is the Standard-Announcecounter

#define 	CNF_DEF_accrepeat			3
#define 	CNF_MAX_TRNTrepeat			30

#define 	CNF_DEF_LOCO_FMT			FMT_DCC_28
#define 	CNF_DEF_TURNOUT_FMT			TFMT_DCC

#define 	CNF_DEF_MAX_PURGE			480
#define 	CNF_DEF_MAX_CANMODULES		64
#define 	CNF_DEF_MAX_LNETMODULES		64
#define 	CNF_DEF_MAX_S88MODULES		64
#define 	CNF_DEF_MIN_S88FREQUENCY	500
#define 	CNF_DEF_MAX_S88FREQUENCY	5000

#endif /* DEFAULTS_H_ */
