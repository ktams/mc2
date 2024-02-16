/*
 * m3_pom.c
 *
 *  Created on: 26.12.2021
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
#include "config.h"
#include "decoder.h"

int m3pom_readCV (int adr, cvadrT cva, int bytes, reply_handler handler, flexval priv)
{
	struct packet *p;

	if ((p = sigq_m3ReadCV(adr, cva, bytes, handler, priv)) == NULL) return -3;
	sigq_queuePacket(p);

	return 0;
}

int m3pom_writeCV (int adr, cvadrT cva, uint8_t val, int repeat, reply_handler handler, flexval priv)
{
	struct packet *p;

	if ((p = sigq_m3WriteCV(adr, cva, val, repeat)) == NULL) return -3;
	sigq_queuePacket(p);
	if (!handler) return 0;
	return m3pom_readCV(adr, cva, 1, handler, priv);
}

int m3pom_writeCVar (int adr, cvadrT cva, uint8_t *val, int bytes, int repeat, reply_handler handler, flexval priv)
{
	struct packet *p;

	if ((p = sigq_m3WriteCVar(adr, cva, val, bytes, repeat)) == NULL) return -3;
	sigq_queuePacket(p);
	if (!handler) return 0;
	return m3pom_readCV(adr, cva, bytes, handler, priv);
}

/**
 * Searches for a m3 decoder with a UID that matches the first len bits
 * of the given UID. With len = 0, all decoders that have now loco address
 * (SID = Schienen-ID) yet.
 *
 * This functions allows to realize a binary "tree" search of decoders
 * on the track.
 *
 * \param uid	a 32 bit value that is used to query decoders
 * \param len	the significant length of the UID for comparision purposes
 * \param r		the reply structure used to send back the information
 * \return		zero for success, an error code otherwise
 */
//int m3_scan (uint32_t uid, int len)
//{
//	struct packet *p;
//
//	// no locking required - we don't touch or read the loco list
//	if ((p = sigq_m3SearchPacket(uid, len)) == NULL) {
//		return -1;
//	}
//
//	sigq_queuePacket(p);
//
//	return 0;
//}

/**
 * Sets a new decoder address (SID, Schienen-ID) for a decoder with the
 * specified UID.
 *
 * \param uid	a 32 bit value that is used to query decoders
 * @param adr	the loco address that schould be assigned to the decoder
 * \return		zero for success, an error code otherwise
 */
int m3_setAddress (uint32_t uid, int adr)
{
	struct packet *p;

	// no locking required - we don't touch or read the loco list
	if ((p = sigq_m3NewAddress(uid, adr)) == NULL) return -1;

	sigq_queuePacket(p);

	return 0;
}

