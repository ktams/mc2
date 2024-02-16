/**
 * @file easynet.c
 *
 * @author Andi
 * @date   16.04.2020
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
#include "yaffsfs.h"
#include "easynet.h"
#include "decoder.h"
#include "events.h"
#include "config.h"
#include "intelhex.h"

#define ALIVE_VALUE			210			// full live

// format definitions for easynet
#define FMT_M3				0x00		// The new Märklin format MFX
#define FMT_MOTOROLA1		0x01
#define FMT_MOTOROLA2		0x02
#define FMT_DCC				0x03
#define FMT_MAX				0x04		// the first illegal format
#define FMT_MASK			0x03		// the valid format bits

// speed definitions for easynet
#define TRAINSPEEDS_14		0x00		// 14 Speeds all Formats (126 Speeds for MFX)
#define TRAINSPEEDS_MM27A	0x04		// 27 Speeds Motorola (alternating speeds)
#define TRAINSPEEDS_MM27B	0x08		// 27 Speeds Motorola (toggle Trit 5)
#define TRAINSPEEDS_DCC28	0x04		// 28 Speeds DCC
#define TRAINSPEEDS_DCC126	0x08		// 126 Speeds DCC
#define TRAINSPEEDS_MFX126	0x00		// 126 Speeds MFX
#define TRAINSPEEDS_MASK	0x0C

enum mc_state {
	MCSTAT_STOP = 0,		///< no signal generation, power shut down
	MCSTAT_SHORT,			///< same as STAT_STOP except that it automatically entered after a short
	MCSTAT_HALT,			///< signal is generated with all speeds set to 0 (emergency halt is sent as soon as this state is entered)
	MCSTAT_GO,				///< normal operation with signal generation
	MCSTAT_PROGRAM,			///< a loco is programmed on the programming track
	MCSTAT_PRERESET,		///< the system is about to reset (all users are informed)
	MCSTAT_RESET,			///< the real reset via EasyNet is performed
	MCSTAT_TPM,				///< Tams-Programing-Mode (program Tams MM decoder on track)
	MCSTAT_DOWNLOAD,		///< Download software to EasyNet slaves
};

static const struct {
	enum trackmode	tm;
	enum mc_state	mc;
} statMapping[] = {
	{ TM_STOP,		MCSTAT_STOP },
	{ TM_SHORT,		MCSTAT_SHORT },
	{ TM_HALT,		MCSTAT_HALT },
	{ TM_SIGON,		MCSTAT_GO },
	{ TM_GO,		MCSTAT_GO },
	{ TM_DCCPROG,	MCSTAT_PROGRAM },
	{ TM_RESET,		MCSTAT_PRERESET },
	{ TM_TESTDRIVE,	MCSTAT_GO },
	{ TM_TAMSPROG,	MCSTAT_TPM },
};

static struct en_client clients[EN_MAXUNIT + 1];
static volatile TaskHandle_t tid;
static volatile bool stop;

struct en_client *en_getClients(void)
{
	return clients;
}

static void en_controlEvent (int busadr, int reason)
{
	struct extDevice *dev;

	if ((dev = calloc (1, sizeof(*dev))) == NULL) return;
	dev->bus = BUS_EASYNET;
	dev->id = busadr;
	dev->tp = DEV_CONTROL;
	dev->serial = clients[busadr].serno;
	sprintf (dev->hwrev, "%lu", clients[busadr].hw_no);
	snprintf (dev->swrev, sizeof(dev->swrev), "%ld.%ld.%ld", (clients[busadr].sw_no >> 16) & 0xFF, (clients[busadr].sw_no >> 8) & 0xFF, (clients[busadr].sw_no >> 0) & 0xFF);
	event_fireEx(EVENT_CONTROLS, reason, dev, EVTFLAG_FREE_SRC, QUEUE_WAIT_TIME);
}

void en_reportControls (void)
{
	int i;

	for (i = 0; i <= EN_MAXUNIT; i++) {
		if (clients[i].alive > 0) en_controlEvent(i, 1);
	}
}

static uint8_t crc8_update (uint8_t crc, uint8_t c)
{
   int i;

   crc = crc ^ c;
   for (i = 0; i < 8; i++) {
      if (crc & 0x80)
	 crc = (crc << 1) ^ CRC8_POLYNOM;
      else
	 crc = crc << 1;
   }
   return crc;
}

uint8_t bus_blockcrc (en_block *blk)
{
   int i;
   uint8_t *p, crc;

   crc = 0;
   p = blk->bytes;
   for (i = 0; i < BUS_BLOCKLEN - 1; i++) {
      crc = crc8_update (crc, *p++);
   }
   if (crc & 0x80) crc = ~crc;		// if MSB is set, CRC must be transmitted as it's inverse

   return crc;
}

bool bus_chkblock (en_block *blk)
{
	uint8_t crc;

	if ((blk->words[0] & 0x80808080uL) != 0x00000080uL) return false;		// only the address byte may have a set MSB
	if ((blk->words[1] & 0x80808080uL) != 0x00000000uL) return false;		// none of the data bytes may have a set MSB

	crc = bus_blockcrc (blk);
	if (crc != blk->crc) return false;	// wrong CRC detected

	return true;
}

static uint8_t bus_locofmt2en (enum fmt fmt)
{
	switch (fmt) {
		case FMT_MM1_14:	return TRAINSPEEDS_14 | FMT_MOTOROLA1;
		case FMT_MM2_14:	return TRAINSPEEDS_14 | FMT_MOTOROLA2;
		case FMT_MM2_27A:	return TRAINSPEEDS_MM27A | FMT_MOTOROLA2;
		case FMT_MM2_27B:	return TRAINSPEEDS_MM27B | FMT_MOTOROLA2;
		case FMT_M3_126:	return TRAINSPEEDS_MFX126 | FMT_M3;
		case FMT_DCC_14:	return TRAINSPEEDS_14 | FMT_DCC;
		case FMT_DCC_28:	return TRAINSPEEDS_DCC28 | FMT_DCC;
		case FMT_DCC_126:	/* FALL THRU */
		case FMT_DCC_SDF:	return TRAINSPEEDS_DCC126 | FMT_DCC;
		default: return 0;
	}
}

static enum fmt bus_en2locofmt (uint8_t speedfmt)
{
	switch (speedfmt & FMT_MASK) {
		case FMT_M3:
			return FMT_M3_126;
			break;
		case FMT_MOTOROLA1:
#if 0
			switch (speedfmt & TRAINSPEEDS_MASK) {
				case TRAINSPEEDS_14: return FMT_MM1_14;
			}
			break;
#else
			return FMT_MM1_14;
#endif
		case FMT_MOTOROLA2:
			switch (speedfmt & TRAINSPEEDS_MASK) {
				case TRAINSPEEDS_14: return FMT_MM2_14;
				case TRAINSPEEDS_MM27A: return FMT_MM2_27A;
				case TRAINSPEEDS_MM27B: return FMT_MM2_27B;
			}
			break;
		case FMT_DCC:
			switch (speedfmt & TRAINSPEEDS_MASK) {
				case TRAINSPEEDS_14: return FMT_DCC_14;
				case TRAINSPEEDS_DCC28: return FMT_DCC_28;
				case TRAINSPEEDS_DCC126: return FMT_DCC_126;
			}
			break;
	}
	return FMT_UNKNOWN;
}

/**
 * Set the 14 LSB of the given short value to two consecutive addresses
 * in the byte array used for transmission. Because of the lack of 8-bit
 * transfers, only 14 bits may be transmitted. This function can be used
 * for i.e. loco addresses and such stuff, which never should need more
 * than 14 bits.
 *
 * \param ar	the position in the array (as uint8_t pointer) where to write the two 7-bit bytes
 * \param v		the value that should be encoded in the array
 * \return		the new position in the array where to continue to write data to or NULL if
 * 				no array position was given (this must be treated as error).
 */
static uint8_t *bus_set14bit (uint8_t *ar, uint16_t v)
{
	if (!ar) return NULL;

	*ar++ = v & 0x7F;
	*ar++ = (v >> 7) & 0x7F;
	return ar;
}

/**
 * Read a 14 bit value from two consecutive addresses in the byte array and
 * return that value. Because of the lack of 8-bit transfers, only 14 bits may
 * be transmitted. This function can be used for i.e. loco addresses and such
 * stuff, which never should need more than 14 bits.
 *
 * \param ar	the position in the array (as uint8_t pointer) where to read the two 7-bit bytes
 * \return		the interpreted value from the two 7-bit bytes
 */
static uint16_t bus_get14bit (uint8_t *ar)
{
	if (!ar) return 0;

	return (ar[0] & 0x7F) | ((ar[1] & 0x7F) << 7);
}

static void en_serno2array(uint32_t serno, uint8_t *ar)
{
	ar[0] = (serno >> 21) & 0x07;		// 3 MSBs
	ar[1] = (serno >> 14) & 0x7F;		// 7 more bits
	ar[2] = (serno >>  7) & 0x7F;		// 7 more bits
	ar[3] = (serno >>  0) & 0x7F;		// 7 LSBs

}

static uint32_t en_array2serno (uint8_t *ar)
{
	return ((ar[0] & 0x07) << 21) | ((ar[1] & 0x7F) << 14) | ((ar[2] & 0x7F) << 7) | ((ar[3] & 0x7F) << 0);
}

void en_sendBlock (uint8_t adr, uint8_t cmd, uint8_t *data)
{
	en_block blk;

	blk.adr = adr | 0x80;
	blk.cmd = cmd;
	if (data) {
		memcpy(blk.data, data, sizeof(blk.data));
	} else {
		memset(blk.data, 0, sizeof(blk.data));
	}
	blk.crc = bus_blockcrc(&blk);
	spi_sendblock(&blk);
}

static void en_setUnitAddress (uint32_t serno)
{
	uint8_t data[BUS_DATALEN];
	int i;

	for (i = 0; i <= EN_MAXUNIT; i++) {
		if (clients[i].alive <= 0) break;
	}

	if (i > EN_MAXUNIT) {
		fprintf (stderr, "%s(): no free addresses\n", __func__);
		return;		// request silently ignored - no free addresses
	}

	log_msg (LOG_EASYNET, "%s(): Client #%lu -> Unit %d\n", __func__, serno, i);

	en_serno2array(serno, data);
	data[4] = data[5] = 0;
	en_sendBlock(i, CMD_SETUNITADR, data);
	memset (&clients[i], 0, sizeof (clients[i]));
	clients[i].serno = serno;
	clients[i].alive = ALIVE_VALUE;
	en_sendBlock(i, CMD_VERSION, NULL);
	event_fire (EVENT_CONTROLS, i, NULL);
}

static void en_sendStatus (enum trackmode tm)
{
	static int torefresh;		// a refresh counter to cyclically transmit turnout status data (@TODO implementation follows later)

	enum mc_state mcst;
	uint8_t data[BUS_DATALEN];
	uint8_t *p;
	int i;

	mcst = (enum mc_state) tm;	// the states are almost equal ...
	for (i = 0; i < DIM(statMapping); i++) {
		if (statMapping[i].tm == tm) {
			mcst = statMapping[i].mc;
			break;
		}
	}

	if ((torefresh << 4) > 128) torefresh = 0;
	p = data;
	memset (p, 0, sizeof(data));

	*p++ = mcst & 0x7F;
	p = bus_set14bit(p, torefresh);
	// other data will be turnout status information
	*p++ = 0;
	*p++ = 0;
	*p = 0;
	*p |= 0x04;		// set the M3-enabled flag
	en_sendBlock(BUS_BROADCAST, CMD_SYSSTATUS, data);

	torefresh++;
}

static void en_doReset (void)
{
	en_sendBlock(BUS_BROADCAST, CMD_DORESET, NULL);
}

static void en_hotplug (uint8_t cmd, uint8_t hp[])
{
	uint8_t data[BUS_DATALEN];

	data[0] = hp[0] & 0x07;
	data[1] = hp[1] & 0x7F;
	data[2] = hp[2] & 0x7F;
	data[3] = hp[3] & 0x7F;
	data[4] = data[5] = 0;
	en_sendBlock(BUS_BROADCAST, cmd, data);
}

static void en_statuspoll (int ctrl)
{
	if (ctrl < EN_MINUNIT || ctrl > EN_MAXUNIT) return;

	en_sendBlock(ctrl, CMD_STATUSPOLL, NULL);
	if (clients[ctrl].alive > 0) {
		if (--clients[ctrl].alive == 0) {
			log_msg (LOG_EASYNET, "%s() UNIT %d vanished\n", __func__, ctrl);
			en_controlEvent(ctrl, 0);
			clients[ctrl].serno = 0;
		}
	}
}

static bool POMwrite_handler(struct decoder_reply *msg, flexval priv)
{
	(void) priv;

	if(!msg->len) {
		printf("%s() no answer, try again.\n", __func__);
	} else {
		printf("%s(): POM answer: decoder adr.: %d, length: %d, data: %d, %d, %d, %d, %d, %d.....\n", __func__,
				msg->adr, msg->len, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
	}
	return false;
}

static bool POMread_handler(struct decoder_reply *msg, flexval priv)
{
	(void) priv;

	if(!msg->len) {
		printf("%s(): no answer, try again.\n", __func__);
	} else {
		printf("%s(): POM answer: decoder adr.: %d, length: %d, data: %d, %d, %d, %d, %d, %d.....\n", __func__,
				msg->adr, msg->len, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
	}
	return false;
}

static void en_requestConfig (uint8_t unit, en_block *blk)
{
	struct sysconf *sc;
	uint8_t data[BUS_DATALEN];
	uint8_t *p, ui8;
	turnoutT *to;
	uint32_t uid;
	uint16_t locoAdr, CVTmp;
	int i;

	sc = cnf_getconfig();
	switch(blk->data[0]) {
		case CNFRQ_GETFLAGS:
			printf("%s() CNFRQ_GETFLAGS: sysFlags %ld\n", __func__, sc->sysflags);
			p = data;
			*p++ = 0;
			*p++ = sc->sysflags & 0x7F;
			*p++ = (sc->sysflags >>7) & 0x7F;
			*p++ = (sc->sysflags >>14) & 0x7F;
			*p++ = (sc->sysflags >>21) & 0x7F;
			*p++ = 0;
			en_sendBlock(unit, CMD_CONFIG, data);
			break;

		case CNFRQ_SETFLAGS:
			// only the two bits SYSFLAG_LONGPAUSE and SYSFLAG_DEFAULTDCC may be changed!
			sc->sysflags |= (blk->data[1] | (blk->data[2]<<7)) & (SYSFLAG_LONGPAUSE | SYSFLAG_DEFAULTDCC);
			printf("%s() CNFRQ_SETFLAGS: set sysFlags %ld\n", __func__, sc->sysflags);
//					if (sc->sysflags & SYSFLAG_LONGPAUSE) {
//						if(ui8_MM_Pause < 160) {	// < 4ms?
//							ui8_MM_Pause = 161;
//						}
//					} else if (lastsysFlags & SYSFLAG_LONGPAUSE) {
//						ui8_MM_Pause = 60;
//					}
//					lastsysFlags = sc->sysflags;
			event_fire (EVENT_PROTOCOL, sc->sysflags, NULL);
			event_fire (EVENT_BOOSTER, sc->sysflags, NULL);
			cnf_triggerStore(__func__);
			break;

		case CNFRQ_CLRFLAGS:
			// only the two bits SYSFLAG_LONGPAUSE and SYSFLAG_DEFAULTDCC may be changed!
			sc->sysflags &= ~((blk->data[1] | (blk->data[2]<<7)) & (SYSFLAG_LONGPAUSE | SYSFLAG_DEFAULTDCC));
			printf("%s(): clear sysFlags\n", __func__);
			event_fire (EVENT_PROTOCOL, sc->sysflags, NULL);
			event_fire (EVENT_BOOSTER, sc->sysflags, NULL);
			cnf_triggerStore(__func__);
			break;

		case CNFRQ_S88MODULES:
			if(blk->data[1] >= 100)
			{	// ask for no of modules
//						printf("%s() CNFRQ_S88MODULES: ask no of s88: %d\n", __func__, s88_getModules());
				p = data;
				*p++ = CNFRQ_S88MODULES;
				*p++ = s88_getModules();
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				en_sendBlock(unit, CMD_CONFIG, data);
			}
			else
			{
				s88_setModules(blk->data[1]);
//						printf("%s() CNFRQ_S88MODULES: set no of s88 modules: %d\n", __func__, s88_getModules());
			}
			break;

		case CNFRQ_S88SUM:
#ifdef CENTRAL_FEEDBACK
			CVTmp = fb_getModuleState(blk->data[1]);
#else
			CVTmp = s88_getInput(blk->data[1]);
#endif
			p = data;
			*p++ = CNFRQ_S88SUM;
			*p++ = CVTmp & 0x7F;
			*p++ = (CVTmp >> 7) & 0x7F;
			*p++ = (CVTmp >> 14) & 0x03;
			*p++ = 0;
			*p++ = 0;
			en_sendBlock(unit, CMD_CONFIG, data);
//					printf("%s() CNFRQ_S88SUM: get s88 module %d data %d\n", __func__, blk->data[1], CVTmp);
			break;

		case CNFRQ_SHORTTIME:
			if(blk->data[1]) {
				printf("%s() CNFRQ_SHORTTIME: set booster short time: %d\n", __func__, blk->data[2] * 5);
				ts_setSensitivity(blk->data[2] * 5);
			} else {
				printf("%s() CNFRQ_SHORTTIME: get booster short time\n", __func__);
			}
			p = data;
			*p++ = CNFRQ_SHORTTIME;
			*p++ = ts_getSensitivity()/5;
			*p++ = 0;
			*p++ = 0;
			*p++ = 0;
			*p++ = 0;
			en_sendBlock(unit, CMD_CONFIG, data);
			break;

		case CNFRQ_TURNOUTFMT:
			if((blk->data[1] == 0x7F) && (blk->data[2] == 0x7F)) {
				if (blk->data[3] == 0x7F) {
					printf("%s() CNFRQ_TURNOUTFMT: get general turnout address format\n", __func__);
				} else {
					db_setTurnoutFmt (0, blk->data[3] == 3 ? TFMT_DCC : TFMT_MM);
					printf("%s() CNFRQ_TURNOUTFMT: set general turnout address format to %s\n", __func__, blk->data[3] == 1 ? "MM" : "DCC");
					event_fire (EVENT_ACCESSORY, 0, NULL);
				}
				p = data;
				*p++ = CNFRQ_TURNOUTFMT;
				*p++ = 0x7F;
				*p++ = 0x7F;
				*p++ = (db_getTurnout(0)->fmt == TFMT_DCC) ? 3 : 1;
				*p++ = 0;
				*p++ = 0;
				en_sendBlock(unit, CMD_CONFIG, data);
			} else {
				CVTmp = (blk->data[2]<<7) | blk->data[1];
				CVTmp = (CVTmp >> 1) + 1;
				to = db_getTurnout (CVTmp);
				if (blk->data[3] == 0x7F) {
					p = data;
					*p++ = CNFRQ_TURNOUTFMT;
					*p++ = blk->data[1];
					*p++ = blk->data[2];
					*p++ = (to->fmt == TFMT_MM) ? 1 : 3;
					*p++ = 0;
					*p++ = 0;
					en_sendBlock(unit, CMD_CONFIG, data);
					printf("%s() CNFRQ_TURNOUTFMT: get turnout address %d format: %d\n", __func__, CVTmp, to->fmt);
				} else {
					CVTmp = (((CVTmp - 1) >> 2) << 2) + 1;
					ui8 = (blk->data[3] == 1) ? TFMT_MM : TFMT_DCC;
					db_setTurnoutFmt(CVTmp++, ui8);
					db_setTurnoutFmt(CVTmp++, ui8);
					db_setTurnoutFmt(CVTmp++, ui8);
					db_setTurnoutFmt(CVTmp, ui8);
					printf("%s() CNFRQ_TURNOUTFMT: set turnout from address %d to adress %d to format: %s\n", __func__,
							CVTmp - 3, CVTmp, (blk->data[3] == 1) ? "MM" : "DCC");
				}
			}
			break;

		case CNFRQ_DCC_RDBYTE:
			p = data;
			*p++ = STAT_PROGRAM;
			*p++ = 0;
			en_sendBlock(BUS_BROADCAST, CMD_SYSSTATUS, data);
			CVTmp = blk->data[2]<<7 | blk->data[1];
			i = dccpt_cvReadByte(CVTmp);
			p = data;
			*p++ = CNFRQ_DCC_PROGRESULT;
			if(i < 0) {
				*p++ = 0x7F;
				*p++ = 0x7F;
				*p++ = 3;
			} else {
				*p++ = i & 0x7F;
				*p++ = (i >> 7) & 0x7F;
				*p++ = 0;
			}
			*p++ = 0;
			*p++ = 0;
			en_sendBlock(unit, CMD_CONFIG, data);
			printf("%s() CNFRQ_DCC_RDBYTE: DCC read byte %d on prog track\n", __func__, CVTmp);
			break;

		case CNFRQ_DCC_WRBYTE:
			p = data;
			*p++ = STAT_PROGRAM;
			*p++ = 0;
			en_sendBlock(BUS_BROADCAST, CMD_SYSSTATUS, data);
			CVTmp = blk->data[2]<<7 | blk->data[1];
			i = dccpt_cvWriteByte (CVTmp, blk->data[4]<<7 | blk->data[3]);
			p = data;
			*p++ = CNFRQ_DCC_PROGRESULT;
			if(i < 0) {
				*p++ = 0x7F;
				*p++ = 0x7F;
				*p++ = 3;
			} else {
				*p++ = i & 0x7F;
				*p++ = (i >> 7) & 0x7F;
				*p++ = 0;
			}
			*p++ = 0;
			*p++ = 0;
			en_sendBlock(unit, CMD_CONFIG, data);
			printf("%s() CNFRQ_DCC_WRBYTE: DCC write byte %d on prog track. Value: %d\n", __func__, CVTmp, blk->data[4]<<7 | blk->data[3]);
			break;

		case CNFRQ_DCC_RDBIT:
			printf("%s() CNFRQ_DCC_RDBIT: DCC read bit on prog track\n", __func__);
			fprintf(stderr,"%s() CNFRQ_DCC_RDBIT: ToDo: implementieren\n", __func__);
			break;

		case CNFRQ_DCC_WRBIT:
			printf("%s() CNFRQ_DCC_WRBIT: DCC write bit on prog track\n", __func__);
			fprintf(stderr,"%s() CNFRQ_DCC_WRBIT: ToDo: implementieren\n", __func__);
			break;

		case CNFRQ_SETMFXADR:
			locoAdr = blk->data[2]<<7 | blk->data[1];
			i = sig_searchM3Loco(&uid);
			if(i > 0) {
				m3_setAddress(uid, locoAdr);
				p = data;
				*p++ = CNFRQ_SETMFXADR;
				*p++ = uid & 0x7F;
				*p++ = (uid >> 7) & 0x7F;
				*p++ = (uid >> 14) & 0x7F;
				*p++ = (uid >> 21) & 0x7F;
				*p++ = (uid >> 28) & 0x7F;
				en_sendBlock(unit, CMD_CONFIG, data);
				printf("%s() CNFRQ_SETMFXADR: program a MFX loco UID=0x%lx to address %d\n", __func__, uid, locoAdr);
			} else {
				p = data;
				*p++ = CNFRQ_SETMFXADR;
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				en_sendBlock(unit, CMD_CONFIG, data);
				printf("%s() CNFRQ_SETMFXADR: ERROR %d\n", __func__, i);
			}
			break;
	}
}

/**
 * Check if any of the locos in a consist is controlled by that client.
 *
 * \param cl		pointer to the client to check
 * \param l			the loco data structure (may or may not be part of a consist)
 * \return			true if any of the locos in a consist is controlled by that client, false otherwise
 */
static bool en_checkLoco (struct en_client *cl, ldataT *l)
{
	ldataT *c;

	if (!cl || !cl->alive) return false;

	c = l;
	do {
		if (cl->loco == c->loco->adr) return true;
		c = c->consist;
	} while (c && c != l);
	return false;
}

static void en_override (ldataT *l, int source)
{
	uint8_t data[BUS_DATALEN];
	uint8_t *p;
	int i;

//	log_msg (LOG_EASYNET, "%s(%d): for loco %d\n", __func__, source, (l) ? l->loco->adr : -1);
	if (!l || !l->loco) return;

	p = bus_set14bit(data, l->loco->adr);
	*p++ = l->speed & 0x7F;				//speed
	*p++ = (l->speed & 0x80 ? 0x40 : 0) | ((l->funcs[0] >> 10) & 0x3F);	//dir (bit 6)/functions
	*p++ = (l->funcs[0] >> 3) & 0x7F;	//functions
	*p++ = (l->funcs[0] & 0x07) << 4;	//functions
	en_sendBlock(BUS_BROADCAST, CMD_OVERRIDE, data);
	p = &data[2];		// the loco number remains as is
	*p++ = (l->funcs[0] >> 16) & 0x7F;
	*p++ = (l->funcs[0] >> 23) & 0x7F;
	*p++ = (l->funcs[0] >> 30) & 0x03;
	*p = 0;
	en_sendBlock(BUS_BROADCAST, CMD_EXFUNCS, data);

	for (i = 0; i <= EN_MAXUNIT; i++) {
		if (i != source && en_checkLoco(&clients[i], l)) {
//		if (clients[i].alive && clients[i].loco == l->loco->adr && i != source) {
//			log_msg (LOG_EASYNET, "%s(%d) blocking %d\n", __func__, source, i);
			clients[i].blocked = 2;	// to be fair, block the next two requests
		}
	}
}

/**
 * To keep HC-2 happy, we will report any Version below 2.4.0 as
 * 2.4.0, so no "Update MasterControl" is bothering us.
 */
static bool en_reportDummyVersion (void)
{
	if (SOFT_VERSION_MAJOR < 2) return true;
	if ((SOFT_VERSION_MAJOR == 2) && (SOFT_VERSION_MINOR < 4)) return true;

	return false;
}

static void en_request (en_block *blk)
{
	static char name[12];

	struct consist *c;
	uint8_t data[BUS_DATALEN];
	ldataT *l;
	locoT *ldb;
	uint16_t adr, adr1, adr2, cv;
	uint8_t *p, unit;
	uint8_t speed;
	uint32_t newfuncs;
	int i;

	if (!blk) return;
	unit = blk->adr & 0x7F;
	adr = bus_get14bit(blk->data);

	if (blk->cmd != ANS_SETSPEED) log_msg (LOG_EASYNET, "%s(): Unit %d CMD = 0x%02x (loco=%d)\n", __func__, unit, blk->cmd, adr);
	memset (data, 0, sizeof(data));

	switch (blk->cmd) {
		case ANS_REQUESTSTATUS:
			switch(blk->data[0]) {
				case STAT_STOP:
					sig_setMode(TM_STOP);
					break;

				case STAT_HALT:
					sig_setMode(TM_HALT);
					break;

				case STAT_GO:
					sig_setMode(TM_GO);
					break;

				case STAT_PRERESET:
					seg_reboot();
					break;

				case STAT_RESET:
					reboot();
					break;
			}
			break;

		case ANS_LOCODB_SELECT:		// TODO change NON-ASCII characters to some sensful default and observe UTF-8 coding
			ldb = db_lookupLocoSorted(adr - 1);			// EasyNet uses 1-based DB indizees
			if (ldb) log_msg (LOG_EASYNET, "%s(LOCODB_SELECT %d): ADR %d '%s'\n", __func__, adr, ldb->adr, ldb->name);
			p = bus_set14bit (data, ldb->adr);
			*p++ = bus_locofmt2en(ldb->fmt);
			if (ldb->name == 0) {		// loco without name -> "-/-"
				*p++ = '-';
				*p++ = '/';
			} else {
				*p++ = ldb->name[0];
				*p++ = ldb->name[1];
			}
			*p++ = 0;
			en_sendBlock(unit, CMD_LOCODB_DATA0, data);
			if (*ldb->name == 0) {		// loco without name -> "-/-"
				memset(data, 0, BUS_DATALEN);
				p = data;
				*p++ = '-';
				*p++ = 0;
			} else {
				memcpy (data, &ldb->name[2], BUS_DATALEN);
			}
			en_sendBlock(unit, CMD_LOCODB_DATA1, data);
			if (*ldb->name == 0) {		// loco without name
				memset(data, 0, BUS_DATALEN);
			} else {
				memcpy (data, &ldb->name[7], BUS_DATALEN);
			}
			en_sendBlock(unit, CMD_LOCODB_DATA2, data);
			break;

		case ANS_LOCODB_NEXT:
			p = bus_set14bit(data, db_indexSorted_next(adr - 1) + 1);	// EasyNet uses 1-based DB indizees and 0 for first lookup
			en_sendBlock(unit, CMD_LOCODB_INDEX, data);
			break;

		case ANS_LOCODB_PREV:
			p = bus_set14bit(data, db_indexSorted_prev(adr - 1) + 1);	// EasyNet uses 1-based DB indizees
			en_sendBlock(unit, CMD_LOCODB_INDEX, data);
			break;

		case ANS_LOCODB_DELETE:
			ldb = db_lookupLocoSorted(adr - 1);			// EasyNet uses 1-based DB indizees
			if (!ldb) break;
			log_msg (LOG_EASYNET, "%s() LOCODB_DELETE idx=%d adr=%d\n", __func__, adr - 1, ldb->adr);
			db_removeLoco(ldb);
			break;

		case ANS_LOCODB_DATA0:
			fprintf(stderr,"%s() ANS_LOCODB_DATA0: ToDo: implementieren\n", __func__);
			break;

		case ANS_LOCODB_DATA1:
			name[0] = blk->data[2];
			name[1] = blk->data[3];
			name[2] = blk->data[4];
			break;

		case ANS_LOCODB_DATA2:
			name[3] = blk->data[2];
			name[4] = blk->data[3];
			name[5] = blk->data[4];
			break;

		case ANS_LOCODB_DATA3:
			name[6] = blk->data[2];
			name[7] = blk->data[3];
			name[8] = blk->data[4];
			break;

		case ANS_LOCODB_DATA4:
			name[9] = blk->data[2];
			name[10] = blk->data[3];
			name[11] = blk->data[4];
			ldb = db_lookupLocoSorted(adr - 1);			// EasyNet uses 1-based DB indizees
			db_setLocoName(ldb->adr, name);
			break;

		case ANS_VERSION:
			if(!blk->data[0]) {	// send version of MC2
				p = data;
				*p++ = hwinfo->HW;
				if (en_reportDummyVersion()) {
					*p++ = 2;
					*p++ = 4;
					*p++ = 0;
				} else {
					*p++ = SOFT_VERSION_MAJOR;
					*p++ = SOFT_VERSION_MINOR;
					*p++ = SOFT_VERSION_SUB;
				}
				*p++ = 0;
				*p++ = 0;
				en_sendBlock(unit, CMD_VERSION, data);
				p = data;
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				*p++ = 0;
				en_sendBlock(BUS_BROADCAST, CMD_VERSION, data);
			} else {	//version of control
				clients[unit].hw_no = blk->data[0];
				clients[unit].sw_no = blk->data[1] << 16 | blk->data[2] << 8 | blk->data[3];
				printf("%s() Control: %d, HW: %ld, SW: %lx.%lx.%lx\n", __func__, unit, clients[unit].hw_no,
					(clients[unit].sw_no >> 16 & 0xFF), (clients[unit].sw_no >> 8) & 0xFF, clients[unit].sw_no & 0xFF);
				en_controlEvent (unit, 1);
			}
			break;

		case ANS_REQUESTLOCO:
			if ((l = loco_call (adr, true)) == NULL) return;		// cannot grant access to loco - something is wrong (?)
			clients[unit].loco = adr;
			clients[unit].blocked = 0;
			p = bus_set14bit(data, adr);
			*p++ = l->speed & 0x7F;
			*p++ = ((l->speed >> 1) & 0x40) | ((l->funcs[0] >> 10) & 0x3F);
			*p++ = (l->funcs[0] >> 3) & 0x7F;
			*p = (l->funcs[0] << 4) & 0x70;
			if (l->funcs[0] & (1 << 16)) *p |= 0x08;
			if (l->consist) *p |= 1;		// announce consist
			en_sendBlock(unit, CMD_YIELDLOCO, data);
			p = &data[2];		// the loco number remains as is
			*p++ = bus_locofmt2en(l->loco->fmt);
			i = db_lookupIndex(l->loco) + 1;
			p = bus_set14bit(p, i);
			*p++ = 0;
			en_sendBlock(unit, CMD_LOCOFORMAT, data);
			p = &data[2];		// the loco number remains as is
			*p++ = (l->funcs[0] >> 16) & 0x7F;
			*p++ = (l->funcs[0] >> 23) & 0x7F;
			*p++ = (l->funcs[0] >> 30) & 0x03;
			*p = 0;
			en_sendBlock(unit, CMD_EXFUNCS, data);
			if (l->consist) {
				p = bus_set14bit(data, l->consist->loco->adr);
				*p++ = 0;				// speed is not needed
				*p++ = (l->consist->funcs[0] >> 3) & 0x7F;
				*p = (l->consist->funcs[0] << 4) & 0x70;
				if (l->consist->funcs[0] & (1 << 16)) *p |= 0x08;
				en_sendBlock(unit, CMD_YIELDTRACTION, data);
			}
			break;

		case ANS_SETSPEED:
			if (clients[unit].blocked > 0) {
				clients[unit].blocked--;
			} else {
				/*
				 * here are bits used in this block:
				 * data[2] = <speed:7>
				 * data[3] = <dir:1> <F15:1> <F14:1> <13:1> <F12:1> <F11:1> <F10:1>
				 * data[4] = <F9:1> <F8:1> <F7:1> <F6:1> F5:1> F4:1> <F3:1>
				 * data[5] = <F2:1> <F1:1> <F0:1> 0b0000:4
				 */
				l = loco_call (adr, true);
				speed = blk->data[2] | ((blk->data[3] & 0x040) << 1);
//				log_msg(LOG_INFO, "%s(): ADR %d speed %c%d\n", __func__, adr, (speed & 0x80) ? 'F' : 'R', speed & 0x7F);
				newfuncs = ((blk->data[3] & 0x3F) << 10) | ((blk->data[4] & 0x7F) << 3) | ((blk->data[5] & 0x70) >> 4);
				if (l && (speed != l->speed || newfuncs != (l->funcs[0] & FUNC_F0_F15))) {
					rq_setSpeed (adr, speed);
					rq_setFuncMasked(adr, newfuncs, FUNC_F0_F15);
					en_override(l, unit);
				}
			}
			break;

		case ANS_SETLOCOFORMAT:
			if ((l = loco_call (adr, true)) != NULL) {
				db_setLocoFmt(adr, bus_en2locofmt(blk->data[2]));
				p = bus_set14bit(data, adr);
				*p++ = bus_locofmt2en(l->loco->fmt);
				en_sendBlock(unit, CMD_LOCOFORMAT, data);
			}
			break;

		case ANS_CLEARTRACTION:
			consist_remove(adr);
			break;

		case ANS_SETFUNCEX:
			if (clients[unit].blocked > 0) {
				clients[unit].blocked--;
			} else {
				l = loco_call (adr, true);
				newfuncs = ((blk->data[2] & 0x7F) << 16) | ((blk->data[3] & 0x7F) << 23) | ((blk->data[4] & 0x03) << 30);
				if (l && (l->funcs[0] & FUNC_F16_F31) != newfuncs) {
					rq_setFuncMasked(adr, newfuncs, FUNC_F16_F31);
					en_override(l, unit);
				}
			}
			break;

		case ANS_BINSTATE:
			if (clients[unit].blocked > 0) {
				clients[unit].blocked--;
			} else {
				l = loco_call (adr, true);
				newfuncs = ((blk->data[3] & 0x7F) << 7) | (blk->data[2] & 0x7F);
				if(newfuncs <= 32) {	// we handle functions till 31 not as binary states!
					if(blk->data[4] & 0x40) {
						switch(newfuncs) {
							case 29:
								newfuncs = 0x20000000;
								break;
							case 30:
								newfuncs = 0x40000000;
								break;
							case 31:
								newfuncs = 0x80000000;
								break;
							case 32:
								fprintf(stderr,"%s() functions from f32 to f68 not yet implemented...\n", __func__);
								break;
						}
					} else {
						newfuncs = 0;
					}
					rq_setFuncMasked(adr, newfuncs, FUNC_F16_F31);
					en_override(l, unit);
				} else if(newfuncs > 68) {
					loco_setBinState (adr, newfuncs, (blk->data[4] & 0x40) ? 1: 0);
				} else {
					fprintf(stderr,"%s() functions from f32 to f68 not yet implemented...\n", __func__);
				}
			}
			break;

		case ANS_FMAPLIST:
			p = bus_set14bit(data, adr);
			*p++ = 0;
			*p++ = 0;
			*p++ = 0;
			*p++ = 0;
			en_sendBlock(unit, CMD_FMAPLIST, data);
			printf("%s() ANS_FMAPLIST %x %x %x %x %x %x\n", __func__,blk->data[0], blk->data[1], blk->data[2],
					blk->data[3], blk->data[4], blk->data[5]);
			fprintf(stderr,"%s() ANS_FMAPLIST: ToDo: implementieren\n", __func__);
			break;

		case ANS_REQUESTTRACTION:
			adr1 = (blk->data[1] << 7) | blk->data[0];
			adr2 = (blk->data[3] << 7) | blk->data[2];
			log_msg (LOG_EASYNET, "%s() ANS_REQUESTTRACTION: loco %d + loco %d\n", __func__, adr1, adr2);
			c = consist_coupleAdd(adr1, adr2);
			if (c && (l = loco_call(adr2, true)) != NULL) {
				p = bus_set14bit(data, adr2);
				*p++ = l->speed & 0x7F;
				*p++ = ((l->funcs[0] >> 10) & 0x3F);	// speed not needed
				*p++ = (l->funcs[0] >> 3) & 0x7F;
				*p = (l->funcs[0] << 4) & 0x70;
				if (l->funcs[0] & (1 << 16)) *p |= 0x08;
				en_sendBlock(unit, CMD_YIELDTRACTION, data);
			}
			break;

		case ANS_CONFIG:
			en_requestConfig(unit, blk);
			break;

		case ANS_DCCONTRACK:
			cv = bus_get14bit(&blk->data[2]);
			printf("%s() ANS_DCCONTRACK: DCC-OnTrack programming Adr %d CV %d VAL: %d\n", __func__, adr, cv, blk->data[4]);
			dccpom_writeByte(adr, DECODER_DCC_MOBILE, cv, blk->data[4], POMwrite_handler, fvNULL);
			break;

		case ANS_DCCRAILCOM:
			cv = bus_get14bit(&blk->data[2]);
			printf("%s() ANS_DCCRAILCOM: RailCom read byte: Adr %d CV %d\n", __func__, adr, cv);
			dccpom_readByte(adr, DECODER_DCC_MOBILE, cv, POMread_handler, fvNULL);
			break;

		case ANS_SETMAGNET:
			printf("%s() ANS_SETMAGNET: turnout %d dir: %d param %d\n", __func__, (adr>>1) + 1, adr & 1, blk->data[3]);
			trnt_switch((adr >> 1) + 1, adr & 1, blk->data[3]);
			p = data;
			*p++ = 0;
			*p++ = 0;
			*p++ = 0;
			*p++ = 0;
			*p++ = 0;
			*p++ = 0;
			en_sendBlock(BUS_BROADCAST, CMD_MAGNET, data);
			break;

		default:
			fprintf(stderr, "%s() unknown cmd: %x; data %x %x %x %x %x %x\n", __func__, blk->cmd,
					blk->data[0], blk->data[1], blk->data[2], blk->data[3], blk->data[4], blk->data[5]);
			break;
	}
}

static bool en_eventHandler (eventT *e, void *arg)
{
	ldataT *l;

	(void) arg;

	if (e->tid == tid) return true;		// this event is triggered by our own activity - ignore it

	switch (e->ev) {
		case EVENT_LOCO_SPEED:
		case EVENT_LOCO_FUNCTION:
			l = (ldataT *) e->src;
			en_override(l, -1);
			break;
		default:		// to keep compiler happy
			break;
	}

	return true;
}

void easynet (void *pvParameter)
{
	en_block blk;
	uint8_t hp[4], hp_state = 0;
	int seq = 0;
	int ctrl;
	uint32_t serno = 0;

	(void) pvParameter;

	log_enable(LOG_EASYNET);		// $$$$$$$
	log_msg (LOG_INFO, "%s() STARTING\n", __func__);
	spi_init(false);
	tid = xTaskGetCurrentTaskHandle();

	en_doReset();

//	event_register(EVENT_S88, en_pushS88, NULL, 0);
	event_register(EVENT_LOCO_FUNCTION, en_eventHandler, NULL, 0);
	event_register(EVENT_LOCO_SPEED, en_eventHandler, NULL, 0);
//	event_register(EVENT_PROTOCOL, en_changeFlags, NULL, 0);

	stop = false;
	while (!stop) {
		while (spi_getblock(&blk)) {
//			log_msg (LOG_INFO, "%s() RX %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", __func__,
//					blk.adr, blk.cmd, blk.data[0], blk.data[1], blk.data[2], blk.data[3], blk.data[4], blk.data[5], blk.crc);
			if (blk.adr == BUS_BROADCAST) {
				switch (blk.cmd) {
					case ANS_HOTPLUG0:
						hp[0] = blk.data[0];
						hp[1] = hp[2] = hp[3] = 0;
						hp_state = 1;
						seq = HOTPLUG_SCAN;
						break;
					case ANS_HOTPLUG1:
						hp[0] = blk.data[0];
						hp[1] = blk.data[1];
						hp[2] = hp[3] = 0;
						hp_state = 2;
						seq = HOTPLUG_SCAN;
						break;
					case ANS_HOTPLUG2:
						hp[0] = blk.data[0];
						hp[1] = blk.data[1];
						hp[2] = blk.data[2];
						hp[3] = 0;
						hp_state = 3;
						seq = HOTPLUG_SCAN;
						break;
					case ANS_HOTPLUG3:
						serno = en_array2serno(blk.data);
						en_setUnitAddress(serno);
						seq = HOTPLUG_SCAN;
						break;
				}
			} else {
				ctrl = blk.adr & 0x7F;
				if (ctrl >= EN_MINUNIT && ctrl <= EN_MAXUNIT) {
					clients[ctrl].alive = ALIVE_VALUE;
					en_request(&blk);
				}
			}
		}

		if (seq < 0 || seq > POST_STATUS) seq = 0;
		while (seq <= EN_MAXUNIT && clients[seq].alive <= 0) seq++;
		if (seq == HOTPLUG_SCAN) {
			switch (hp_state) {
				case 0:
					hp[1] = hp[2] = hp[3] = 0;
					en_hotplug(CMD_HOTPLUG0, hp);
					hp[0] = (hp[0] + 1) & 0x07;
					break;
				case 1:
					hp[2] = hp[3] = 0;
					en_hotplug(CMD_HOTPLUG1, hp);
					hp[1] = (hp[1] + 1) & 0x7F;
					if (hp[1] == 0) hp_state--;		// up one level
					break;
				case 2:
					hp[3] = 0;
					en_hotplug(CMD_HOTPLUG2, hp);
					hp[2] = (hp[2] + 1) & 0x7F;
					if (hp[2] == 0) hp_state--;		// up one level
					break;
				case 3:
					en_hotplug(CMD_HOTPLUG3, hp);
					hp[3] = (hp[3] + 1) & 0x7F;
					if (hp[3] == 0) hp_state--;		// up one level
					break;
			}
		} else if (seq == POST_STATUS) {
			en_sendStatus(rt.tm);
		} else {
			en_statuspoll(seq);
		}

		if (hp_state == 0) seq++;
	}

	event_deregister(EVENT_LOCO_FUNCTION, en_eventHandler, NULL);
	event_deregister(EVENT_LOCO_SPEED, en_eventHandler, NULL);
	tid = 0;
	stop = false;
	log_msg (LOG_INFO, "%s(): terminating\n", __func__);
	vTaskDelete(NULL);
}

/*
 * =========================================================================================
 * the boot protocol
 * =========================================================================================
 */

#define READSIZE			4096		///< read the HEX file with this block size and store it binary in memory chunks
#define CHUNKSIZE			64			///< the binary file contents in 64-Byte Units
#define TINYSIZE			64			///< tiny 64-Byte Blocks (32 Words)
#define SMALLSIZE			128			///< mid sized 128-Byte Blocks (64 Words)
#define BIGSIZE				256			///< big 256-Byte Blocks (128 Words) for ATmega128

#define BOOTCRC_POLYNOM		0xA001		///< DEC polynom X^16+X^15+X^13+X
#define BOOTCRC_START		0xFFFF		///< CRC start value
#define BOOTCRC_SIZE		sizeof(uint16_t)

enum bootstate {
	BSTATE_STARTUP = 0,					///< wait for the first valid request from remote device
	BSTATE_BLOCKTRANSMIT,				///< actively transmissting blocks
	BSTATE_BLOCKREPEAT,					///< the current block was repeated
	BSTATE_ENDBLOCK,					///< transmitting last block
	BSTATE_RECOVER,						///< recover from CRC error (maybe the communication is out of phase)
	BSTATE_FINISHED,					///< we are all done
};

struct __attribute__((packed)) data64 {
   uint16_t			 start;				///< start address (16 bit only!)
   uint8_t			 mem[TINYSIZE];		///< the data
   uint16_t			 crc16;				///< 16 bit checksum (CRC polynom 0xA001)
};

struct __attribute__((packed)) data128 {
	uint16_t		 start;				///< start address (16 bit only!)
	uint8_t			 mem[SMALLSIZE];	///< the data
	uint16_t		 crc16;				///< 16 bit checksum (CRC polynom 0xA001)
};

struct __attribute__((packed)) data256 {
   uint32_t			 start;				///< start address (32 bit!)
   uint8_t			 mem[BIGSIZE];		///< the data
   uint16_t			 crc16;				///< 16 bit checksum (CRC polynom 0xA001)
   uint16_t			 fill;				///< two filler bytes with the idle character to bump up the size to a multiple of 4
};

static union mblock {
   struct data64	 tiny;				///< tiny block (64 bytes)
   struct data128	 middle;			///< mid size block (128 bytes)
   struct data256	 big;				///< big block (256 bytes)
} bootblk;

struct chunk {
   struct chunk		*next;
   uint32_t			 adr;
   uint8_t			 mem[CHUNKSIZE];
};

struct en_bootProgress progress;

static void en_bootFreeChunks (struct chunk *ch)
{
	struct chunk *tmp;

	while ((tmp = ch) != NULL) {
		ch = ch->next;
		free (tmp);
	}
}

static int en_bootCountBlocks (struct chunk *chunks)
{
   struct chunk *ch;
   int cnt;

   ch = chunks;
   cnt = 0;
   while (ch) {
      cnt++;
      ch = ch->next;
   }

   return cnt;
}

static int en_bootAddData (struct chunk **chpp, uint32_t adr, uint8_t *data, int len)
{
	struct chunk *ch;
	int i, j;

	if (!chpp || !data || len <= 0) return 0;	// can't do anything ...

	while ((ch = *chpp) != NULL && ch->adr + CHUNKSIZE <= adr) chpp = &ch->next;
	if (!ch || ch->adr > adr) {		// insert a new block
		if ((ch = malloc (sizeof(*ch))) == NULL) return -1;
		memset (ch->mem, 0xFF, sizeof(ch->mem));
		ch->adr = (adr / CHUNKSIZE) * CHUNKSIZE;
		ch->next = *chpp;
		*chpp = ch;
	}

	for (i = adr - ch->adr, j = 0; i < CHUNKSIZE && j < len; i++, j++) ch->mem[i] = data[j];
	if (j < len) return en_bootAddData (chpp, adr + j, &data[j], len - j);	// should data overflow the current chunk, add it to the next chunk

	return 0;
}

static uint16_t en_bootCRC (uint16_t crc, uint8_t *p, int len)
{
	int i;

	while (len > 0) {
		crc ^= *p++;
		for (i = 0; i < 8; i++) {
			if (crc & 1) crc = (crc >> 1) ^ BOOTCRC_POLYNOM;
			else crc >>= 1;
		}
		len--;
	}
	return crc;
}

static union mblock *en_bootGetBlock (struct chunk *ch, union mblock *mb, uint32_t adr, int size)
{
	uint8_t *p;

	if (!mb || !ch) return NULL;

	if (size < SMALLSIZE) size = TINYSIZE;
	if (size > SMALLSIZE) size = BIGSIZE;
	adr = (adr / size) * size;					// align address to a block boundary

	progress.current = 0;
	while (ch && ch->adr < adr) {
		progress.current++;
		ch = ch->next;	// search for first chunk to be copied to memblock
	}

	switch (size) {
		case TINYSIZE:
			mb->tiny.start = (uint16_t) adr;
			if (!ch) mb->tiny.start = 0xFFFF;		// end marker (outside memory region)
			p = mb->tiny.mem;
			break;
		case SMALLSIZE:
			mb->middle.start = (uint16_t) adr;
			if (!ch) mb->middle.start = 0xFFFF;		// end marker (outside memory region)
			p = mb->middle.mem;
			break;
		case BIGSIZE:
			mb->big.start = adr;
			if (!ch) mb->big.start = 0xFFFFFFFF;	// end marker (outside memory region)
			p = mb->big.mem;
			break;
		default:
			p = mb->big.mem;		// to keep compiler happy
			break;
	}

	memset (p, 0xFF, size);

	while (ch && ch->adr < (adr + size)) {
		memcpy (p + ch->adr - adr, ch->mem, CHUNKSIZE);	// the chunk data always fits into the memblock!
		ch = ch->next;
	}

	switch (size) {
		case TINYSIZE:
			mb->tiny.crc16 = en_bootCRC (0xFFFF, (uint8_t *) &mb->tiny.start, size + sizeof(mb->tiny.start));
			break;
		case SMALLSIZE:
			mb->middle.crc16 = en_bootCRC (0xFFFF, (uint8_t *) &mb->middle.start, size + sizeof(mb->middle.start));
			break;
		case BIGSIZE:
			mb->big.crc16 = en_bootCRC (0xFFFF, (uint8_t *) &mb->big.start, size + sizeof(mb->big.start));
			mb->big.fill = 0xFEFE;
			break;
	}
	return mb;
}

static enum bootstate en_bootBlockTransmit (struct chunk *chunks, union mblock *mb, int c, enum bootstate state)
{
	int size;
	uint32_t adr = 0;

	if (state == BSTATE_STARTUP) {
		if (c != 'T') return state;
		return BSTATE_BLOCKTRANSMIT;
	}

	switch (c) {		// check received characters and ignore any spurious chars
		case '*':
		case '#':
			size = BIGSIZE;
			adr = mb->big.start;
			break;
		case '+':
		case '?':
			size = SMALLSIZE;
			adr = mb->middle.start;
			break;
		case '-':
		case '%':
			size = TINYSIZE;
			adr = mb->tiny.start;
			break;
		case 'A':
		case 'C':
		case 'T':
		case 'L':
		case 'F':
			break;
		default:		// allow only a the chars explicitly listed above! (problematic startup with HandControl-2)
			mb->big.start = 0;
			return BSTATE_STARTUP;
	}

	switch (c) {
		case '*':		// next block 256 byte
		case '+':		// next block 128 byte
		case '-':		// next block 64 byte
			state = BSTATE_BLOCKTRANSMIT;
			adr += size;
			/* FALL THRU */
		case '#':		// repeat last block 256 byte
		case '?':		// repeat last block 128 byte
		case '%':		// repeat last block 64 byte
			if (c == '#') {
				if (state == BSTATE_BLOCKREPEAT) return BSTATE_RECOVER;
				else state = BSTATE_BLOCKREPEAT;
			}
			mb = en_bootGetBlock(chunks, mb, adr, size);
			log_msg (LOG_INFO, "%s(): %d blocks transmitted / size=%d c='%c' (%d 0x%02x)\n", __func__, progress.current, size, c, c, c);
			event_fire(EVENT_ENBOOT, 0, &progress);
			switch (size) {
				case TINYSIZE:
//					log_msg (LOG_INFO, "%s(): TX block 0x%08x (len %d crc 0x%04x)\n", __func__, mb->tiny.start, size, mb->tiny.crc16);
					spi_write((uint32_t *) mb, size + sizeof(mb->tiny.start) + BOOTCRC_SIZE);
					if (mb->tiny.start == 0xFFFF) state = BSTATE_ENDBLOCK;
					break;
				case SMALLSIZE:
//					log_msg (LOG_INFO, "%s(): TX block 0x%08x (len %d crc 0x%04x)\n", __func__, mb->middle.start, size, mb->middle.crc16);
					spi_write((uint32_t *) mb, size + sizeof(mb->middle.start) + BOOTCRC_SIZE);
					if (mb->middle.start == 0xFFFF) state = BSTATE_ENDBLOCK;
					break;
				case BIGSIZE:
//					log_msg (LOG_INFO, "%s(): TX block 0x%08lx (len %d crc 0x%04x)\n", __func__, mb->big.start, size, mb->big.crc16);
					spi_write((uint32_t *) mb, size + sizeof(mb->big.start) + BOOTCRC_SIZE);
					if (mb->big.start == 0xFFFFFFFF) state = BSTATE_ENDBLOCK;
					break;
				default:
					log_error ("%s(): illegal size %d\n", __func__, size);
					break;
			}
			break;
		case 'A':		// address error - restart from beginning
			mb->big.start = 0;	// this also sets the start address of other sizes to zero
			break;
		case 'C':		// CRC error - recover and retransmit block
			return BSTATE_RECOVER;
		case 'T':		// timeout (no data) - retransmit block
			if (state == BSTATE_RECOVER) return BSTATE_BLOCKTRANSMIT;
			return BSTATE_RECOVER;
		case 'L':		// length error (timeout) - retransmit block
			if (state == BSTATE_RECOVER) return BSTATE_BLOCKTRANSMIT;
			break;
		case 'F':		// node has received terminating block
			return BSTATE_FINISHED;
	}

	return state;
}

static void en_bootMode (struct chunk *ch)
{
	enum bootstate bs;
	TickType_t to;
	int c;

	log_msg (LOG_INFO, "%s(): start update\n", __func__);

	if (!ch) {
		log_msg (LOG_WARNING, "%s() ERROR: no memory chunks\n", __func__);
		return;
	}

	progress.total = en_bootCountBlocks(ch);
	progress.current = 0;
	log_msg (LOG_INFO, "%s(): %d blocks total\n", __func__, progress.total);
	event_fire(EVENT_ENBOOT, 0, &progress);

	if (tid) {
		stop = true;
		while (tid) vTaskDelay(10);
	}
	spi_init(true);

	to = tim_timeout(10000);
	while (!tim_isover(to) && (c = spi_getchar()) != EOF) ;

	to = tim_timeout(10000);
	bs = BSTATE_STARTUP;
	bootblk.big.start = 0;
	while (bs != BSTATE_FINISHED && !tim_isover(to)) {
		if ((c = spi_getchar()) != EOF) {
			log_msg(LOG_INFO,  "%s() got '%c'\n", __func__, c);
			bs = en_bootBlockTransmit (ch, &bootblk, c, bs);
			to = tim_timeout(1000);
		}
	}
	event_fire(EVENT_ENBOOT, 0, NULL);

	SPI1->CR1 = 0;		// disable SPI1
	SPI1->CR1 = 0;		// disable SPI1
	SPI1->CR1 = 0;		// disable SPI1
	SPI1->CR1 = 0;		// disable SPI1
	log_msg (LOG_INFO, "%s() finished\n", __func__);
	xTaskCreate(easynet, "EasyNet", 1024, NULL, 1, NULL);
}

int en_bootReadBuffer (void *arg, uint8_t *buf, int len)
{
	static struct chunk *chunks;
	static uint8_t input[256];		// maximum line fragment we can remember between calls
	static uint8_t *end;
	static struct ihexdata ihex;

	int fill, rc;
	uint8_t *p, *s;

	(void) arg;

	if (len < 0) {			// initialisation call
		log_msg (LOG_INFO, "%s(): INIT\n", __func__);
		end = NULL;
		en_bootFreeChunks(chunks);
		chunks = NULL;
		memset (&ihex, 0, sizeof(ihex));
	} else if (buf == NULL) {
		log_msg (LOG_INFO, "%s(): START UPDATE\n", __func__);
		vTaskDelay(200);
		en_bootMode(chunks);
		en_bootFreeChunks(chunks);
		chunks = NULL;
	} else {
//		log_msg (LOG_INFO, "%s(): DATA len=%d input=%p end=%p\n", __func__, len, input, end);
//		vTaskDelay(20);
		if (!end) end = input;
		while (len > 0 && ihex.state == IHEX_READING) {
			fill = sizeof(input) - (end - input);		// space in the static linebuffer
			if (fill > len) fill = len;
//			log_msg (LOG_INFO, "%s(): FILL len=%d fill=%d end=%p\n", __func__, len, fill, end);
//			vTaskDelay(20);
			memcpy (end, buf, fill);
			end += fill;
			len -= fill;
			buf += fill;
			p = input;
			do {
				while (p < end && (*p == '\r' || *p == '\n')) p++;		// skip everything that is a line ending
				s = p;
				while (s < end && *s != '\r' && *s != '\n') s++;
				if (s > p && s < end && (*s == '\r' || *s == '\n')) {		// a complete line is in the buffer - interpret it
					rc = ihex_readline(&ihex, (char *) p);
					if (rc < 0) {
						vTaskDelay(100);
						break;
					}
					if (rc > 0) {
						en_bootAddData(&chunks, ihex.segadr + ihex.reladr, ihex.data, rc);
					}
					while (s < end && (*s == '\r' || *s == '\n')) s++;		// skip everything that is a line ending
					p = s;													// and position p to new line start
				}
			} while (s < end);
			if (p < end) {		// there is a left over - move to beginning of buffer
				memmove (input, p, end - p);
				end = input + (end - p);
			} else {			// everything is read, so restart at beginning of buffer
				end = input;
			}
		}
	}
	return 0;
}
