/**
 * @file xpressnet.h
 *
 * @author Kersten
 * @date   06.08.2020
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

#ifndef __XPRESSNET_H__
#define __XPRESSNET_H__

/*
 * some defines for the bus protocol
 */


#define CMD_SPEED14		0x10	///< speed 14 steps
#define CMD_SPEED27		0x11	///< speed 27 simulated steps
#define CMD_SPEED28		0x12	///< speed 28 steps
#define CMD_SPEED128	0x13	///< speed 128 steps

#define CMD_FG1			0x20	///< function group 1: F0 .. F4
#define CMD_FG2			0x21	///< function group 2: F5 .. F8
#define CMD_FG3			0x22	///< function group 3: F9 .. F12
#define CMD_FG4			0x23	///< function group 1: F13 .. F20
#define CMD_FG4R		0xF3	///< function group 1: F13 .. F20	ATTENTION -> ROCO MultiMouse
#define CMD_FG5			0x28	///< function group 1: F21 .. F28

#define CMD_POM			0x30	///< programming on main
#define CMD_PTREAD		0x18	///< programming on programming track
#define CMD_PTRESULT	0x10

#define CMD_BUILD_DT	0x43	///< build DT

#define CMD_STOP		0x80
#define CMD_START		0x81

#define MAX_NODES			32			///< number of XPressNet nodes (beware not to use node 0!)
#define NODEFLG_ACTIVE		0x0001		///< this node is in use (if not set, this node address is free)
#define XPN_ALIVE			15000

/*
 * some prototypes
 */

#endif
