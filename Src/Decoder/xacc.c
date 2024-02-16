/*
 * xacc.c
 *
 *  Created on: 05.06.2020
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

#include "rb2.h"
#include "decoder.h"

int xacc_aspect (int adr, int aspect)
{
	struct packet *p;
	extaccT *x;

	if (!loco_lock(__func__)) return -1;

	if ((x = db_getExtacc(adr)) != NULL) {
		p = sigq_extaccPacket (x, aspect);
		loco_unlock();
		if (p) sigq_queuePacket(p);
	} else {
		loco_unlock();
	}

	return 0;
}
