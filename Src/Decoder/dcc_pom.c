/*
 * dcc_pom.c
 *
 *  Created on: 16.05.2021
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

#include <stdio.h>
#include "rb2.h"
#include "config.h"
#include "decoder.h"

int dccpom_readByte (int adr, dec_type dt, int cv, reply_handler handler, flexval fv)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	enum fmt fmt;
	enum queue_cmd cmd;
	struct packet *p;

	switch (dt) {
		case DECODER_DCC_MOBILE:
			if (adr <= 0 || adr > MAX_LOCO_ADR) return -1;
			fmt = FMT_DCC_28;
			cmd = QCMD_DCC_POM_READ;
			break;
		case DECODER_DCC_ACC:
			if (adr <= 0 || adr > MAX_DCC_ACCESSORY) return -1;
			fmt = TFMT_DCC;
			cmd = QCMD_DCC_POM_ACC_READ;
			break;
		case DECODER_DCC_EXT:
			if (adr <= 0 || adr > MAX_DCC_EXTACC) return -1;
			fmt = TFMT_DCC;
			cmd = QCMD_DCC_POM_EXT_READ;
			break;
		default:					// others are not allowed
			return -1;
	}

	if (cv < MIN_DCC_CVADR || cv > MAX_DCC_CVADR) return -2;
	if ((p = sigq_genPacket(NULL, fmt, cmd)) == NULL) return -3;

	p->adr = adr;
	p->repeat = fcfg->dcc.pomrepeat;		// override packet repeat count
	p->cva.cv = cv;							// store CV address in the unstructured linear cv member of cva
	p->cb = handler;
	p->priv = fv;
	sigq_queuePacket(p);

	return 0;
}

int dccpom_writeByte (int adr, dec_type dt, int cv, int val, reply_handler handler, flexval priv)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	enum fmt fmt;
	enum queue_cmd cmd;
	struct packet *p;

	switch (dt) {
		case DECODER_DCC_MOBILE:
			if (adr <= 0 || adr > MAX_DCC_ADR) return -1;
			fmt = FMT_DCC_28;
			cmd = QCMD_DCC_POM_WRITE;
			break;
		case DECODER_DCC_ACC:
			if (adr <= 0 || adr > MAX_DCC_ACCESSORY) return -1;
			fmt = TFMT_DCC;
			cmd = QCMD_DCC_POM_ACC_WRITE;
			break;
		case DECODER_DCC_EXT:
			if (adr <= 0 || adr > MAX_DCC_EXTACC) return -1;
			fmt = TFMT_DCC;
			cmd = QCMD_DCC_POM_EXT_WRITE;
			break;
		default:					// others are not allowed
			return -1;
	}

	if (cv < MIN_DCC_CVADR || cv > MAX_DCC_CVADR) return -2;
	if ((p = sigq_genPacket(NULL, fmt, cmd)) == NULL) return -3;
	if ((val < 0x00) || (val > 0xFF)) return -4;

	p->adr = adr;
	p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
	p->cva.cv = cv;
	p->value.i32 = val;
	p->cb = handler;
	p->priv = priv;
	sigq_queuePacket(p);

	return 0;
}

int dccpom_writeBytesShortForm (int adr, dec_type dt, int cv, uint8_t *val, int cnt, reply_handler handler, flexval priv)
{
	struct packet *p;
	enum queue_cmd cmd;
	ldataT *l;
	(void) dt;

	if (cv < 2 && cv > 5) return -2;	// all other reserved
	if ((l = loco_call(adr, true)) != NULL) {		// illegal decoder address or out of memory
		switch (cnt) {
			case 1: cmd = QCMD_DCC_XWR1; break;
			case 2: cmd = QCMD_DCC_XWR2; break;
			default: return -4;
		}
		if ((p = sigq_dcc_pomShortWrite(l, cmd, cv, val)) == NULL) return -3;

		log_error ("%s(): Jawoll!\n", __func__);
		p->cb = handler; //handler;
		p->priv = priv;
		sigq_queuePacket(p);
	}

	return 0;
}

int dccxpom_writeBytes (int adr, dec_type dt, int cv, uint8_t *val, int cnt, reply_handler handler, flexval priv)
{
	struct packet *p;
	enum queue_cmd cmd;
	ldataT *l;
	(void) dt;

	if (cv < MIN_DCC_CVADR || cv > MAX_DCC_EXTCVADR) return -2;
	if ((l = loco_call(adr, true)) != NULL) {		// illegal decoder address or out of memory
		switch (cnt) {
			case 1: cmd = QCMD_DCC_XPOM_WR_BYTE1; break;
			case 2: cmd = QCMD_DCC_XPOM_WR_BYTE2; break;
			case 3: cmd = QCMD_DCC_XPOM_WR_BYTE3; break;
			case 4: cmd = QCMD_DCC_XPOM_WR_BYTE4; break;
			default: return -4;
		}
		if ((p = sigq_dcc_xPom(l, cmd, cv, val)) == NULL) return -3;

		p->cb = handler; //handler;
		p->priv = priv;
		sigq_queuePacket(p);
	}

	return 0;
}

int dccpom_writeBit (int adr, dec_type dt, int cv, uint8_t bit, bool val, reply_handler handler, flexval priv)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	enum fmt fmt;
	enum queue_cmd cmd;
	struct packet *p;

	switch (dt) {
		case DECODER_DCC_MOBILE:
			if (adr <= 0 || adr > MAX_DCC_ADR) return -1;
			fmt = FMT_DCC_28;
			cmd = QCMD_DCC_POM_WRITEBIT;
			break;
		case DECODER_DCC_ACC:
			if (adr <= 0 || adr > MAX_DCC_ACCESSORY) return -1;
			fmt = TFMT_DCC;
			cmd = QCMD_DCC_POM_ACC_WRITEBIT;
			break;
		case DECODER_DCC_EXT:
			if (adr <= 0 || adr > MAX_DCC_EXTACC) return -1;
			fmt = TFMT_DCC;
			cmd = QCMD_DCC_POM_EXT_WRITEBIT;
			break;
		default:					// others are not allowed
			return -1;
	}

	if (cv < MIN_DCC_CVADR || cv > MAX_DCC_CVADR) return -2;
	if ((p = sigq_genPacket(NULL, fmt, cmd)) == NULL) return -3;
	if (bit > 7) return -4;

	p->adr = adr;
	p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
	p->cva.cv = cv;
	p->value.bitval = val ? 1 : 0;
	p->value.bitpos = bit;
	p->cb = handler;
	p->priv = priv;
	sigq_queuePacket(p);

	return 0;
}

/**
 * Writes a special sequence of POM-CV write accesses without waiting for a reply.
 * No reply is ever expected.
 *
 * We will use a loco address that probably no one uses. But even if such a loco is
 * on the track,it should ignore the writes because the here used CV7 is a readonly
 * vendor ID.
 *
 * First, the vendor ID is written to CV7 - this should prepare the receiver (currently
 * we only have boosters on our mind) for getting a paramter byte to the same CV.
 * This idea was first invented by Lenz (to our knowledge).
 *
 * Even though the paramter byte may mean everything, there are already some values
 * predefined to adjust the track voltage and current limits.
 *
 * \param vid		the DCC vendor ID to use as a key (62 for Tams)
 * \param param		the parameter byte which is sent to the device(s)
 */
int dccpom_boosterConf (int vid, uint8_t param)
{
	struct packet *p;

	if ((vid & 0xFF) == param) return -2;		// VID and param must be two distinct values!

	// send the "enable" key (= vendor ID)
	if ((p = sigq_genPacket(NULL, FMT_DCC_28, QCMD_DCC_POM_WRITE)) == NULL) return -3;
	p->adr = MAX_DCC_ADR;
	p->repeat = 16;			// override packet repeat count
	p->cva.cv = 6;			// this is CV7!
	p->value.i32 = vid & 0xFF;
	sigq_queuePacket(p);
	// send the parameter
	if ((p = sigq_genPacket(NULL, FMT_DCC_28, QCMD_DCC_POM_WRITE)) == NULL) return -3;
	p->adr = MAX_DCC_ADR;
	p->repeat = 20;			// override packet repeat count
	p->cva.cv = 6;			// this is CV7!
	p->value.i32 = param;
	sigq_queuePacket(p);
	return 0;
}
