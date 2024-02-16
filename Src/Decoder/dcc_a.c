/*
 * dcc_a.c
 *
 *  Created on: 11.01.2022
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
#include <string.h>
#include "rb2.h"
#include "decoder.h"
#include "config.h"
#include "events.h"

/*
 * According to RCN-218 the following data spaces are known:
 *	A - a special format (shortINFO) which is always 6 bytes in size and transmitted in a single railcom window
 *	0 - extended capabilities
 *		byte 0: support of certain (advanced) DCC commands (also in shortinfo byte 3)
 *		byte 1: bitmap support of supported data spaces (also in shortinfo byte 4)
 *		byte 2: protocol capabilities (also in decoder state, definition is missing)
 *		byte 3: protocol capabilities (also in decoder state, definition is missing)
 *	1 - space info - a bitmap of supported data spaces (first byte also in short info byte 4 and data space 0 byte 1)
 *	2 - ShortGUI
 *	3 - CV mirror
 *	4 - function icons
 *	5 - long name and description (writeable)
 *	6 - product name (not writeable)
 *	7 - product specific information
 */

/**
 * The CID (Central ID), is the Unique ID of this station as a 16 bit value.
 * The lower 16 bits of the serial number are XORed with the manufacturer ID (LSB)
 * and the Bits 16 ~ 23 of the serial number for the MSB. The latter will probably
 * always stay zero, so the upper byte of the CID will equal the second byte of
 * the serial number.
 */
#define CID		((uint16_t) (*(uint16_t *)hwinfo->serial) ^ (hwinfo->manufacturer | ((hwinfo->serial >> 8) & 0xFF00)))
#define START_DYN_ADR		1000	///< the address where we start to supply dynamic addresses
#define PREFIX_SHORT_ADR	0x3800	///< a prefix that denotes an address to be the short address (1 .. 127)
#define PREFIX_EXT_ADR		0x2800	///< a prefix for the 11 bit address of an extended acessory decoder (up to 0x2F00)
#define PREFIX_ACC_ADR		0x3000	///< a prefix for the 11 bit address of a basic acessory decoder (up to 0x3700)
#define NOTIFY_TIMEOUT		1000	///< number of timer ticks (= 1ms) to wait for the callback to happen

typedef struct dccaCandidate dccaCandidateT;

struct rcBlock {
	union {
		uint8_t				header;		///< the complete header byte
		struct {
			uint8_t			size:5;		///< the block size 0..31 (total = <size> + 2 byte)
			uint8_t			cont:1;		///< if set, this is a continuation block
			uint8_t			noresp:1;	///< if cleared, this is a resonse, otherwise it is an unsolicited message (ignored here!)
			uint8_t			fmtA:1;		///< if set, this is a FMT-A block and the other bits of this byte are interpreted different
		};
	};
	uint8_t				data[5];
};

struct dccaCandidate {
	dccaCandidateT		*next;		///< linked list of candidates
	locoT				*dec;		///< the decoder data if already known
	uint16_t			 vid;		///< 12 bits of manufacturer ID (vendor-ID)
	uint32_t			 uid;		///< the device serial number
	int					 retry;		///< number of retries already done
	uint8_t				 si[6];		///< the received SortInfo block
};

static dccaCandidateT *preset;		///< a list of candidates that we want to check for existance
static dccaCandidateT *waiting;		///< a list of candidates that answered the select ShortInfo request and are waiting for further handling

enum dcca_state {
	/*
	 * 25.04.2022 A.Kre: IDEE! Beim Starten machen wir einen State DCCASTAT_STARTUP, der ein LOGON_ENABLE(NOW)
	 * im 50ms Rhythmus aussendent (5 ~ 10x). Die Ergebnisse werden insofern ignoriert, dass lediglich die Info,
	 * ob überhaupt irgendjemand geantwortet hat vermerkt wird. Wenn dem so ist, wird im nächsten Step
	 * DCCASTAT_PREASSIGN allen bekannten DCC-A Decodern ihre Adresse zugeteilt (LOGON_ASSIGN). Alle antwortenden
	 * Decoder werden ins Refresh aufgenommen und bei Bedarf neu ausgelesen, sofern die Change-Bits gestezt sind.
	 *
	 * ACHTUNG: Das Auslesen der Decoder muss als eigene Routine (letztlich mit eigenem (Sub-)Status) erfolgen,
	 * da sonst das Springen zwischen den Status nicht funktioniert.
	 */
	DCCASTAT_LOGONIDLE = 0,			///< try to identify single decoders using LOGON_ENABLE(ALL) with a long timeout
	DCCASTAT_ISOLATION,				///< after a collision, we must separate the decoders
	DCCASTAT_SHORTINFO,				///< request the short info packet from the decoder
	DCCASTAT_ASSIGN,				///< assign an SID to the decoder
	DCCASTAT_DATASPACE,				///< request the available data spaces
	DCCASTAT_CLEAR_CHGFLAGS,		///< clear the change flags using the subcode 0xFF with SetDecoderState command
	// the following states are merely internal states of the block reader and not used as main states
	DCCASTAT_BLOCK_REQUEST,			///< substate for block transfer: the start info to request a block is sent
	DCCASTAT_BLOCK_START,			///< substate for block transfer: the DATA_BLOCK_START is sent
	DCCASTAT_BLOCK_CONT,			///< substate for block transfer: the DATA_BLOCK_CONT is sent
	DCCASTAT_BLOCK_OK,				///< virtual substate for block transfer: the data block was received successfully
	DCCASTAT_BLOCK_ERROR,			///< virtual substate for block transfer: the data block reception was corrupt
//	DCCASTAT_SHORTGUI,				///< request ShortGUI (Block #2) from decoder
	DCCASTAT_STARTUP,				///< special state to denote starting when booster is switched on
};

enum msgtype {
	DCCAMSG_NODATA = 0,				///< no information received in the railcom cutout
	DCCAMSG_DATA,					///< we received a valid data block with 6 valid bytes (some bytes may be unused)
	DCCAMSG_COLLISION,				///< we received some data but it was not valid (6-8-decoding error or less than 6 bytes)
	DCCAMSG_ACK,					///< we received an ACK without further data bytes
	DCCAMSG_BARRIER,				/**< A timed message sent to close a timeout window after the DCC-A packet was sent on the track.
	 	 	 	 	 	 	 	 	 	 Any answer that arrives later will be discarded */
};

struct dcca_message {
	enum msgtype	 msg;			///< the type of message we received
	uint8_t			 data[6];		///< the data bytes from the railcom window (if used)
};

struct dcca_info {
	uint32_t		 vid;			///< the vendor ID (usually, only 8 bits are used but up to 12 bits are reserved for the VID)
	uint32_t		 uid;			///< the 32 bit unique ID of the decoder (i.e. serial number)
	cvadrT			 cva;			///< for reading data blocks: the CV address
	int				 len;			///< the number of CVs to read as block - will be split up into 6-byte
	uint8_t			 data[6];		///< the up to 6 data bytes that may be received
	uint8_t			 chg_flags;		///< from ID13: change flags
	uint16_t		 chg_count;		///< from ID13: 12 bit change count
	uint8_t			 capa[4];		///< the decoder capabilities (from ID13, ShortInfo or BLOCK0)
	uint8_t			 maxfunc;		///< the highest function / output number supported by the decoder
	uint16_t		 adr;			///< the address the decoder want's (not in real DCC-encoding, see RCN218, appendix D)
	bool			 newloco;		///< if this flag is set, we must read all blocks from decoder, because we don't know anything about if
	uint8_t			 blk;			///< for Block-Requetest: the block ID to read (this will be the start value
	uint8_t			*blkptr;		///< the current data position while reading the block
	uint8_t			 blkhead;		///< the header byte read while reading a block
	uint8_t			 blkdata[32];	///< the block data including the final CRC byte
};

struct blkread {
	TaskHandle_t		 task;		///< the task to notify
	int					 len;		///< the current length of the block, must be preset to 0
	uint8_t				 spaceID;	///< the current space ID for CRC initialisation
	uint8_t				 head;		///< the header byte from the current block
	uint8_t				 crc;		///< the incremental CRC value while receiving, must be preset to data space ID
	uint8_t				*data;		///< the current data write position
};

// return codes for the data space reader (dccA_blockReader())
#define SPACERX_FAIL		1		///< something went wrong - we can't continue
#define SPACERX_CONTINUE	2		///< continue reading
#define SPACERX_COMPLETE	3		///< data space was transferred complete

static struct dcca_info info;

static const uint8_t crc_array[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
	0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
	0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
	0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
	0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
	0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
	0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
	0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
	0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
	0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
	0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
	0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
	0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
	0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
	0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
	0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
	0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

static const char *funcinfo[] = {
	"not available",
	"switching",
	"momentary",
	"trigger type"
};

/**
 * Calculate the DCC-A CRC for packets longer than 5 bytes (i.e. 6 bytes or more
 * including the pseudo address byte). This algorithm is also used for DCC-A answers.
 *
 * \param crc		the start value or value from a previous (part-) run
 * \param data		a pointer to the data array over which the CRC is to be calculated
 * \param len		the number of bytes to include in CRC calculation
 * \return			the resulting CRC byte
 */
uint8_t dccA_CRCcont (uint8_t crc, volatile uint8_t *data, int len)
{
	if (!data || len <= 0) return 0;
	while (len > 0) {
		crc = crc_array[*data++ ^ crc];
		len--;
	}
	return crc;
}

/**
 * Calculate the DCC-A CRC for packets longer than 5 bytes (i.e. 6 bytes or more
 * including the pseudo address byte). This algorithm is also used for DCC-A answers.
 * The start value is 0x00 - if another start value is needed, use dcc_aCRCcont().
 *
 * \param data		a pointer to the data array over which the CRC is to be calculated
 * \param len		the number of bytes to include in CRC calculation
 * \return			the resulting CRC byte
 * \see				dcc_aCRCcont()
 */
uint8_t dccA_CRC (volatile uint8_t *data, int len)
{
	return dccA_CRCcont(0, data, len);
}

static void dccA_candidateCleanup (void)
{
	dccaCandidateT *tmp;

	// cleanup lists
	while ((tmp = waiting) != NULL) {
		waiting = tmp->next;
		free (tmp);
	}
	while ((tmp = preset) != NULL) {
		preset = tmp->next;
		free (tmp);
	}
}

static bool dccA_candidateGenerate (locoT *l, void *priv)
{
	dccaCandidateT *c;

	if (!l || !priv) return false;									// errneous call!
	if ((l->config != CONF_DCCA) || !l->vid || !l->uid) return true;	// ignore this decoder
	if ((c = calloc(1, sizeof(*c))) == NULL) return false;
	c->dec = l;
	c->vid = l->vid & 0xFFF;
	c->uid = l->uid;

	list_append(priv, c);

	return true;
}

static void dccA_candidateStartup (void)
{
	dccaCandidateT *c;

	log_msg(LOG_INFO, "%s() starting\n", __func__);
	dccA_candidateCleanup();
	db_iterateLoco(dccA_candidateGenerate, &preset);

	c = preset;
	while (c) {
		log_msg (LOG_INFO, "%s() ADR %d VID %d UID 0x%08lx\n", __func__, c->dec->adr, c->vid, c->uid);
		c = c->next;
	}
}

/**
 * Decode the decoder proposed address according to RCN-218. The addresses contain
 * some bits specifying the type of address requested.
 *
 * Decoders in FW upgrade mode send address requests that begin with 0x3F and
 * result in an error here (0 is returned).
 *
 * \param wanted		the type-coded address the decoder asks for
 * \return				the resulting "linear" address to use for the decoder or 0 in case of error
 */
static int dccA_decodeAddress (uint16_t wanted)
{
	switch (wanted & 0x3800) {
		case 0x0000:
		case 0x0800:
		case 0x1000:
		case 0x1800:
		case 0x2000:	// long address for mobile decoders
			log_msg (LOG_DCCA, "%s() long mobile decoder ADR %u\n", __func__, wanted);
			return wanted;
			break;
		case PREFIX_EXT_ADR:	// extended accessory decoders
			wanted &= 0x07FF;
			log_msg (LOG_DCCA, "%s() extended accessory decoder ADR %u\n", __func__, wanted);
			return wanted;
		case PREFIX_ACC_ADR:	// basic accessory decoders
			wanted &= 0x07FF;
			log_msg (LOG_DCCA, "%s() basic accessory decoder ADR %u\n", __func__, wanted);
			return wanted;
		case PREFIX_SHORT_ADR:	// short address for mobile decoders
			if ((wanted & 0x3F00) == 0x3F00) {
				log_msg (LOG_DCCA, "%s() decoder in FW-Update mode ADR 0x%04X\n", __func__, wanted);
				break;
			} else {
				wanted &= 0x007F;
				log_msg (LOG_DCCA, "%s() short mobile decoder ADR %u\n", __func__, wanted);
				return wanted;
			}
		default:
			log_msg (LOG_WARNING, "%s() unknown decoder address request 0x%04x\n", __func__, wanted);
			break;
	}
	return 0;
}

static bool dccA_callback (struct decoder_reply *dm, flexval priv)
{
	TaskHandle_t task;
	uint16_t adr;
	uint8_t crc;

	if (!dm) {
//		log_msg (LOG_DEBUG, "%s() NO REPLY structure\n", __func__);
		return false;
	}
//	log_msg (LOG_DEBUG, "%s() MT=%d dm=%p\n", __func__, dm->mt, dm);

	task = (TaskHandle_t) priv.p;

	if (dm->len > 6) {	// just as a precaution
		log_msg (LOG_WARNING, "%s() LEN=%d is too big (max. 6 expected)\n", __func__, dm->len);
		dm->len = 6;
	}
	crc = dccA_CRC(dm->data, dm->len);
	memcpy (info.data, dm->data, dm->len);

	switch (dm->mt) {
		case DECODERMSG_DECSTATE:
			info.chg_flags = ((dm->data[0] & 0x0F) << 4) | ((dm->data[1] & 0xF0) >> 4);
			info.chg_count = ((dm->data[1] & 0x0F) << 8) | dm->data[2];
			log_msg (LOG_DCCA, "%s(): ID13 received CHG-Flags 0x%02x CHG-count %u Protocol capabilities 0x%02x 0x%02x\n",
					__func__, info.chg_flags, info.chg_count, dm->data[3], dm->data[4]);
			if (crc != 0) log_msg (LOG_WARNING, "%s() wrong CRC 0x%02x (should be 0x00)\n", __func__, crc);
			break;
		case DECODERMSG_UNIQUE:
			memset (&info, 0, sizeof(info));
			info.vid = ((dm->data[0] & 0x0F) << 8) | dm->data[1];
			info.uid = (dm->data[2] << 24) | (dm->data[3] << 16) | (dm->data[4] << 8) | (dm->data[5] << 0);
			log_msg (LOG_DCCA, "%s() ID15 (len %d) VID 0x%03lx UID 0x%08lx\n", __func__, dm->len, info.vid, info.uid);
			break;
		case DECODERMSG_SHORTINFO:
			if ((dm->data[0] & 0xC0) != 0x80) log_msg (LOG_WARNING, "%s(): Shortinfo does not start with 0b10...\n", __func__);
			if (crc != 0) log_msg (LOG_WARNING, "%s() wrong CRC 0x%02x (should be 0x00)\n", __func__, crc);
			info.adr = ((dm->data[0] & 0x3F) << 8) | dm->data[1];
			info.maxfunc = dm->data[2];
			info.capa[0] = dm->data[3];
			info.capa[1] = dm->data[4];
			adr = dccA_decodeAddress(info.adr);
			log_msg (LOG_DCCA, "%s(): wants ADR %d (0x%04x), maxfunc %d capabilities: 0x%02x 0x%02x\n", __func__,
					adr, info.adr, info.maxfunc, info.capa[0], info.capa[1]);
			break;
		case DECODERMSG_COLLISION:
			log_msg (LOG_WARNING, "%s(): COLLISION\n", __func__);
			if (crc != 0) log_msg (LOG_WARNING, "%s() wrong CRC 0x%02x (should be 0x00)\n", __func__, crc);
			log_msg (LOG_DCCA, "%s() %02x %02x %02x %02x %02x %02x\n", __func__,
					dm->data[0], dm->data[1], dm->data[2], dm->data[3], dm->data[4], dm->data[5]);
			break;
		case DECODERMSG_DCCABLOCK:
			log_msg (LOG_DCCA, "%s(): data block received\n", __func__);
			if (info.blkptr == NULL) {
				info.blkhead = dm->data[0];
				info.blkptr = info.blkdata;
				memcpy (info.blkptr, &dm->data[1], dm->len - 1);
				info.blkptr += dm->len - 1;
			} else {
				memcpy (info.blkptr, dm->data, dm->len);
				info.blkptr += dm->len;
			}
			if ((info.blkptr - info.blkdata) >= ((info.blkhead & 0x1F) + 1)) {	// block is complete
				crc = dccA_CRCcont(info.blk, &info.blkhead, 1);		// CRC start value is block number, the header is part of the blockcheck
				crc = dccA_CRCcont(crc, info.blkdata, (info.blkhead & 0x1F) + 1);
				log_msg (LOG_DCCA, "%s() Block received, %d data bytes, CRC %sOK (0x%02x)\n", __func__, info.blkhead & 0x1F,
						(crc) ? "NOT " : "", crc);
//				dm->id[0] = (crc) ? RC_DCCA_BLOCK_CRC : RC_DCCA_BLOCK_END;
			}
			break;
		case DECODERMSG_ACK:
			log_msg (LOG_DCCA, "%s() ACK received\n", __func__);
			break;
		case DECODERMSG_NOANSWER:
		case DECODERMSG_TIMEOUT:
//			if (dm->ack) {
//				dm->id[0] = RC_ACK1;	// patch the code to ACK, may be used with block read (no other answer in first request)
//				log_msg (LOG_DCCA, "%s() NO Answer, but ACK\n", __func__);
//			}
//			else if (dm->nack) log_msg (LOG_WARNING, "%s() NO Answer, but NACK\n", __func__);
//			else log_msg (LOG_DCCA, "%s() NO ANSWER\n", __func__);
			break;
		default:
			log_msg (LOG_WARNING, "%s(): unexpeced MT %d\n", __func__, dm->mt);
			break;
	}
	xTaskNotify(task, dm->mt, eSetValueWithOverwrite);
	return false;	// de-register this callback
}

static bool dccA_blockReader (struct decoder_reply *dm, flexval priv)
{
	struct blkread *br = (struct blkread *) priv.p;
	uint8_t *p;
	int rc, bytecnt;

	if (!dm || !br) return false;

	rc = SPACERX_FAIL;
	switch (dm->mt) {
		case DECODERMSG_DCCABLOCK:
			bytecnt = dm->len;
			p = dm->data;					// we will copy this data block
			if (br->len == 0) {				// this is the first cutout of the block containing the header byte
				br->head = dm->data[0];		// store the header byte in a separate space
				bytecnt--;					// reduce the effective number of "data" bytes receieved
				p++;						// skip the header byte in the decoder answer
			}
			if ((br->len + bytecnt) > ((br->head & 0x1F) + 1)) bytecnt = (br->head & 0x1F) + 1 - br->len;
			log_msg (LOG_DCCA, "%s(): data block received (%d bytes, eff. %d)\n", __func__, dm->len, bytecnt);
			if (br->len == 0) {							// this is the first cutout of the block containing the header byte
				log_msg (LOG_DCCA, "%s(): start packet header = 0x%02x\n", __func__, br->head);
				br->crc = br->spaceID;
				br->crc = dccA_CRCcont(br->crc, &br->head, 1);
			}
			if (br->data) {
				memcpy (br->data, p, bytecnt);
				br->data += bytecnt;
				br->len += bytecnt;
			}
			br->crc = dccA_CRCcont(br->crc, p, bytecnt);
			if (br->len >= ((br->head & 0x1F) + 1)) {	// block reception complete (but may need more blocks)
				if (br->crc) {							// CRC error ruins the complete block and any preceeding block of the same data space
					log_msg (LOG_INFO, "%s(): br->len=%d, CRC-ERROR\n", __func__, br->len);
					rc = SPACERX_FAIL;
				} else if ((br->head & 0x1F) < 0x1F) {	// that's it, folks!
					rc = SPACERX_COMPLETE;
					log_msg (LOG_INFO, "%s(): br->len=%d, COMPLETE\n", __func__, br->len);
					if (br->data) br->data--;			// the last byte stored was the CRC - this should not be treated as data byte
				} else {								// we must read the next block, so reset len, step back data pointer and send CONTINUE
					rc = SPACERX_CONTINUE;
					br->len = 0;
					if (br->data) br->data--;			// the last byte stored was the CRC - next block should overwrite this non-data byte
				}
			} else {
				log_msg (LOG_INFO, "%s(): br->len=%d, CONTINUE\n", __func__, br->len);
				rc = SPACERX_CONTINUE;
			}
			break;
		case DECODERMSG_ACK:
			rc = SPACERX_CONTINUE;	// continue with block read (no other answer in first request)
			log_msg (LOG_DCCA, "%s() NO Answer, but ACK\n", __func__);
			break;
		case DECODERMSG_TIMEOUT:
//			if (dm->ack) {
//				rc = SPACERX_CONTINUE;	// continue with block read (no other answer in first request)
//				log_msg (LOG_DCCA, "%s() NO Answer, but ACK\n", __func__);
//			}
//			else if (dm->nack) log_msg (LOG_WARNING, "%s() NO Answer, but NACK\n", __func__);
			break;
		default:
			log_msg (LOG_WARNING, "%s(): unexpeced MT %d\n", __func__, dm->mt);
			break;
	}


	xTaskNotify(br->task, rc, eSetValueWithOverwrite);
	return false;	// de-register this callback
}

static void dccA_dumpBlock (locoT *l, uint8_t *data, int len)
{
	(void) l;
	hexdump(data, len);
}

static uint8_t *dccA_copyString (char *target, int maxlen, uint8_t *data, int len)
{
	int slen;

	if (!target || !data || maxlen <= 0 || len <= 0) return NULL;
	slen = strnlen ((char *) data, min(len, maxlen - 1));
	memcpy (target, data, slen);
	target[slen] = 0;
	return data + slen + 1;
}

/**
 * Interpretion of data space 6 "Product Information"
 *
 * \param l			the loco data struct
 * \param data		the data received for the data space
 * \param len		the number of bytes received
 */
static void dccA_ProductInformation (locoT *l, uint8_t *data, int len)
{
	uint8_t *p;

	if (!l || !data || len < 2) return;
	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));

	if (l->dcca) {
		p = dccA_copyString(l->dcca->vendor, sizeof(l->dcca->vendor), data, len);
		len -= p - data;
		data = p;
		p = dccA_copyString(l->dcca->product, sizeof(l->dcca->product), data, len);
		len -= p - data;
		data = p;
		p = dccA_copyString(l->dcca->hw_version, sizeof(l->dcca->hw_version), data, len);
		len -= p - data;
		data = p;
		p = dccA_copyString(l->dcca->fw_version, sizeof(l->dcca->fw_version), data, len);

		log_msg (LOG_DCCA, "%s() VENDOR  '%s'\n", __func__, l->dcca->vendor);
		log_msg (LOG_DCCA, "%s() PRODUCT '%s'\n", __func__, l->dcca->product);
		log_msg (LOG_DCCA, "%s() HW      '%s'\n", __func__, l->dcca->hw_version);
		log_msg (LOG_DCCA, "%s() FW      '%s'\n", __func__, l->dcca->fw_version);
	}
}

/**
 * Interpretion of data space 5 "long Name" (and user description)
 *
 * \param l			the loco data struct
 * \param data		the data received for the data space
 * \param len		the number of bytes received
 */
static void dccA_LongName (locoT *l, uint8_t *data, int len)
{
	uint8_t *p;

	log_msg (LOG_DCCA, "%s() len %d\n", __func__, len);
	if (!l || !data || len < 2) return;
	if (*data == 0xFF) return;
	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));

	if (l->dcca) {
		p = dccA_copyString(l->name, sizeof(l->name), data, len);
		len -= p - data;
		data = p;
		p = dccA_copyString(l->dcca->userdesc, sizeof(l->dcca->userdesc), data, len);

		log_msg (LOG_DCCA, "%s() NAME '%s'\n", __func__, l->name);
		log_msg (LOG_DCCA, "%s() DESC '%s'\n", __func__, l->dcca->userdesc);
	}
}

/**
 * Interpretion of data space 4 "Icons"
 *
 * \param l			the loco data struct
 * \param data		the data received for the data space
 * \param len		the number of bytes received
 */
static void dccA_Icons (locoT *l, uint8_t *data, int len)
{
	int i, f;

	if (!l || !data || len < 2) return;
	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));

	if (l->dcca) {
//		l->dcca->userimage = (data[0] << 8) | data[1];	// a user supplied image number from it's personal gallery (not to confuse with ShortGUI)
//		log_msg (LOG_DCCA, "%s() Userimage %u\n", __func__, l->dcca->userimage);
		for (i = 0; i < len; i+= 2) {
			f = data[i];
			db_locoFuncIcon(l, f, data[i + 1]);
			log_msg (LOG_DCCA, "%s() F%d Icon %d\n", __func__, f, data[i + 1]);
		}
	}
}

/**
 * Interpretion of data space 2 "ShortGUI"
 *
 * \param l			the loco data struct
 * \param data		the data received for the data space
 * \param len		the number of bytes received
 */
static void dccA_ShortGUI (locoT *l, uint8_t *data, int len)
{
	int f, fcount, ftim;
	uint8_t ftype;

	if (!l || !data || len < 11) return;
	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));

	if (l->dcca) {
		memcpy (l->dcca->shortname, data, 8);
		l->dcca->shortname[8] = 0;	// be sure to have the short name null-terminated
		l->dcca->decoderimage = (data[8] << 8) | data[9];
		l->dcca->decodericon = data[10] & 0x0F;
		ftype = (data[10] >> 6) & 0x03;
		if (ftype == 0 || ftype == 1) ftim = 0;
		else if (ftype == 2) ftim = -1;
		else ftim = 10;
		db_locoFuncTiming(l, 0, ftim);

		log_msg (LOG_DCCA, "%s() short Name = %s\n", __func__, l->dcca->shortname);
		log_msg (LOG_DCCA, "%s() Picture-Index %u\n", __func__, l->dcca->decoderimage);
		log_msg (LOG_DCCA, "%s() principle symbol %u\n", __func__, l->dcca->decodericon);
		log_msg (LOG_DCCA, "%s() Func/Light: %s\n", __func__, funcinfo[ftype]);
		fcount = (len - 11) * 4;	// the rest of the block (as indicated by LEN in headerbyte) with four funcs per byte
		for (f = 1; f <= fcount && f <= l->maxfunc; f++) {
			ftype = (data[11 + ((f - 1) >> 2)] >> (((f - 1) % 4) * 2)) & 0x03;
			switch (ftype) {
				case 0b00:
					ftim = 0;
					break;
				case 0b01:
					ftim = 0;
					break;
				case 0b10:
					ftim = -1;
					break;
				case 0b11:
					ftim = 10;		// standard 1 second
					break;
			}
			db_locoFuncTiming(l, f, ftim);
			log_msg (LOG_DCCA, "%s() F%d %s\n", __func__, f, funcinfo[ftype]);
		}
	}
}

/**
 * Read a complete data space from the decoder specified by vid and uid.
 *
 * A data space is the "Datenraum" in RailCommunity-speach. It may consist of undefined
 * length and is transferred in blocks of 31 data bytes maximum (taking 33 bytes including
 * its header and CRC byte). These blocks are transferred in multiple railcom cutouts at 6
 * bytes each. Data transfer may start directly with the SELECT call to the decoder or
 * alternatively with the answer to the BLOCK_START request. All following DATA_CONT requests
 * must be answered with the next 6 bytes of the block data.
 *
 * A data space is transferred in blocks of data bytes starting with a header byte and terminated
 * with a CRC byte. The header contains a bit that signifies if the block is the first block
 * of a transfer or a follow-up block. Each block is checked by a CRC that uses the data space
 * ID as the start value and runs from the header byte over all data bytes. For check purposes,
 * the CRC is included in the check run and the result should be zero. Each block is transferred
 * in one or more railcom cutouts with 6 bytes each. A maximum of 6 cutouts is neccessary to
 * transfer a block.
 *
 * The data space is complete when a block with less than 31 data bytes is sent by the decoder.
 * This means, that we usually don't know how big a data space will be. we therefore reserve
 * a static data block where we store the received block data.
 *
 * The sequence of decoder requests is as follows:
 *   - SELECT(ReadBlock), contains data space ID and addresses the decoder - no other decoder may answer to the folling packets
 *   - DATA_START, the one and only start message, even if a continuation block starts
 *   - DATA_CONT, for all cutouts that are needed to transfer all other blocks
 *   - continue with DATA_CONT until an error occures or a block with less than 31 data bytes is received successfully
 *
 * \param spaceID	the "space" index to read (except ID 3, which is the CV read block and should be trated different)
 * \param vid		the vendor ID (up to 12 bits but usually only the lower 8 bits are used)
 * \param uid		the vendor defined unique ID of the decoder (full 32 bits)
 * \return			a state that denotes if the data space was received succcessfully or not
 * \see				RailCommunity RCN-218
 */
static enum dcca_state dccA_readDataSpace (int spaceID, uint32_t vid, uint32_t uid, void (blkInterpreter)(locoT *, uint8_t *, int), locoT *l)
{
	static uint8_t rxbuf[2048];

	struct blkread br;
	struct packet *p;
	enum dcca_state stat;
	flexval fv;
	int retry;

	if (spaceID == 3) return DCCASTAT_BLOCK_ERROR;		// for reading the CV block, we should use a different function ...

	br.task = xTaskGetCurrentTaskHandle();
	br.len = 0;
	br.spaceID = spaceID;
	br.data = rxbuf;
	fv.p = &br;
	retry = 5;

	stat = DCCASTAT_BLOCK_REQUEST;
	while (stat != DCCASTAT_BLOCK_ERROR && stat != DCCASTAT_BLOCK_OK) {
		switch (stat) {
			case DCCASTAT_BLOCK_REQUEST:
				log_msg (LOG_INFO, "%s() reading data space %d\n", __func__, spaceID);
				p = sigq_dcca_selectBlock(vid, uid, spaceID, dccA_blockReader, fv);
				break;
			case DCCASTAT_BLOCK_START:
				p = sigq_dcca_getDataStart(dccA_blockReader, fv);
				break;
			case DCCASTAT_BLOCK_CONT:
				p = sigq_dcca_getDataCont(dccA_blockReader, fv);
				break;
			default:
				return DCCASTAT_BLOCK_ERROR;
		}
		if (p) {
			sigq_queuePacket(p);
			switch ((int) ulTaskNotifyTake(pdTRUE, NOTIFY_TIMEOUT)) {
				case SPACERX_CONTINUE:
					if (stat == DCCASTAT_BLOCK_REQUEST) stat = DCCASTAT_BLOCK_START;
					else stat = DCCASTAT_BLOCK_CONT;
					break;
				case SPACERX_COMPLETE:
					stat = DCCASTAT_BLOCK_OK;
					if (blkInterpreter) blkInterpreter(l, rxbuf, br.data - rxbuf);
					break;
				default:
					if (--retry > 0) {
						log_msg (LOG_WARNING, "%s(): data space %d: ERROR - RETRY\n", __func__, spaceID);
						stat = DCCASTAT_BLOCK_REQUEST;
						br.len = 0;
						br.data = rxbuf;
					} else {
						log_msg (LOG_WARNING, "%s(): data space %d: ERROR\n", __func__, spaceID);
						stat = DCCASTAT_BLOCK_ERROR;
					}
					break;
			}
		}
	}

	return stat;
}

static locoT *dccA_createLoco (locoT *l, int adr)
{
	if (l == NULL) l = db_getLoco(adr, true);
	if (l == NULL) {	// should not happen!
		log_error("%s(): cannot allocate a loco slot\n", __func__);
		return NULL;
	}
	if (adr <= 0) adr = l->adr;
	// must be a DCC loco with either 28 or 126 speed steps (28 speeds steps as the default!
	if (l->fmt != FMT_DCC_14 && l->fmt != FMT_DCC_28 && l->fmt != FMT_DCC_126) db_setLocoFmt(adr, FMT_DCC_28);
	db_setLocoVID(adr, info.vid);
	db_setLocoUID(adr, info.uid);
	db_setLocoMaxfunc(adr, info.maxfunc);
	l->flags |= DEC_DCCA;
	loco_call(adr, true);	// take the new loco into the refresh list
	return l;
}

static locoT *dccA_getAddress (dec_type dt, int wanted)
{
	locoT *l;

	if (dt == DECODER_DCC_MOBILE) {
		if ((l = db_getLoco(wanted, false)) == NULL) return dccA_createLoco(NULL, wanted);			// OK, loco is not yet defined
		if ((l->vid == info.vid) && (l->uid == info.uid)) return dccA_createLoco(l, wanted);		// OK - we know this loco, allow it to use the address
		if ((l = db_findLocoUID(info.vid, info.uid)) != NULL) return dccA_createLoco(l, l->adr);	// found loco based on VID/UID
		return dccA_createLoco(db_addFreeAdr(START_DYN_ADR), 0);
	} else if (dt == DECODER_DCC_ACC) {
	} else if (dt == DECODER_DCC_EXT) {
	}
	return NULL;
}

/**
 * vorläufiger Handler ...
 */
static bool dccA_cb (struct decoder_reply *dm, flexval priv)
{
	struct dcca_message msg;

	(void) priv;
	(void) msg;

	switch (dm->mt) {
		case DECODERMSG_DCCABLOCK:
		case DECODERMSG_SHORTINFO:
		case DECODERMSG_UNIQUE:
		case DECODERMSG_DECSTATE:
			break;
		case DECODERMSG_COLLISION:
			break;
		case DECODERMSG_NOANSWER:
			break;
		case DECODERMSG_ACK:
			break;
		default:
			break;
	}
	return false;
}

/**
 * The Thread that handles the DCC-A machinery.
 * It should sleep whenever the track signal is off.
 *
 * \param pvParamter	the thread creation parameter - unused here
 */
void dccA_service (void *pvParameter)
{
	enum dcca_state stat;
	struct fmtconfig *fc;
	TickType_t timeout, last_transmission;
	uint8_t session;
	struct packet *p;
	locoT *l;
	int rc, iso_retry, retry, dataspace;
	uint16_t coded_adr, wanted_adr;
	bool clear_changeFlags, new_loco;
	dec_msgtype mt;
	flexval fv;

	(void) pvParameter;

	rc = i2c_read(I2C4, MAC_EEPROM, 0, 1, &session, 1);
	if (rc != 0) session = 0;
	session++;
//	session += 4;	// force full registration each time we restart the mc² (for testing purposes)
	rc = i2c_write(I2C4, MAC_EEPROM, 0, 1, &session, 1);
	if (rc != 0) log_error ("%s() cannot write back the session ID (error %d)\n", __func__, rc);
	log_enable(LOG_DCCA);		// $$$$$ should later be disabled

	log_msg (LOG_DCCA, "%s() Starting with CID 0x%04x Session %u\n", __func__, CID, session);
	stat = DCCASTAT_STARTUP;
	fc = cnf_getFMTconfig();
	fv.p = xTaskGetCurrentTaskHandle();
	timeout = last_transmission = 0;
	dataspace = 0;
	l = NULL;

	mt = DECODERMSG_INVALID;
	retry = 0;			// just to overcome the compiler warning of uninitialised usage
	for (;;) {
		if (timeout) {		// calculate timeout as interpacket-timeout without the time spent in waiting for answers
			if (last_transmission) timeout -= xTaskGetTickCount() - last_transmission;
			if (timeout > 0 && timeout < 10000) vTaskDelay(timeout);	// timeout calculation may wrap "below zero"
		}
		last_transmission = xTaskGetTickCount();

		// DCC-A functionality only in GO and HALT states and if DCC-A is enabled, else reset state and idle by waiting
		if ((rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TESTDRIVE) || !(fc->sigflags & SIGFLAG_DCCA)) {
			stat = DCCASTAT_STARTUP;
			timeout = 100;
			continue;
		}

		xTaskNotifyStateClear(NULL);
		switch (stat) {
			case DCCASTAT_STARTUP:
				dccA_candidateStartup();
				stat = DCCASTAT_LOGONIDLE;
				/* FALL THRU */
			case DCCASTAT_LOGONIDLE:
				if ((p = sigq_dcca_logonEnableNow(CID, session, dccA_callback, fv)) != NULL) {
//					log_msg (LOG_INFO, "%s() LOGON_ENABLE(NOW)\n", __func__);
					sigq_queuePacket(p);
					mt = (dec_msgtype) ulTaskNotifyTake(pdTRUE, NOTIFY_TIMEOUT);
//					log_msg (LOG_INFO, "%s() received %d\n", __func__, mt);
					if (mt == DECODERMSG_UNIQUE) {						// YES! we have got a valid answer.
						stat = DCCASTAT_SHORTINFO;
						iso_retry = 0;
						retry = 5;
						log_msg (LOG_DCCA, "%s() switch DCCASTAT_LOGONIDLE => DCCASTAT_SHORTINFO\n", __func__);
						timeout = 0;
					} else if (mt == DECODERMSG_COLLISION) {		// more than one decoder answered
						stat = DCCASTAT_ISOLATION;
						log_msg (LOG_DCCA, "%s() switch DCCASTAT_LOGONIDLE => DCCASTAT_ISOLATION\n", __func__);
						iso_retry = 200;
						timeout = 20;
					} else {
						timeout = 300;
					}
				} else {
					timeout = 300;	// if nothing happened, we can wait for a while
				}
				break;
			case DCCASTAT_ISOLATION:
				if ((p = sigq_dcca_logonEnableAll(CID, session, dccA_callback, fv)) != NULL) {
					sigq_queuePacket(p);
					mt = (dec_msgtype) ulTaskNotifyTake(pdTRUE, NOTIFY_TIMEOUT);
					if (mt == DECODERMSG_UNIQUE) {						// YES! we have got a valid answer.
						stat = DCCASTAT_SHORTINFO;
						retry = 5;
						log_msg (LOG_DCCA, "%s() switch DCCASTAT_ISOLATION => DCCASTAT_SHORTINFO\n", __func__);
						timeout = 0;
					} else if ((rc == DECODERMSG_TIMEOUT) || (rc == DECODERMSG_COLLISION)) {
						if (--iso_retry <= 0) {
							stat = DCCASTAT_LOGONIDLE;
							log_msg (LOG_DCCA, "%s() switch DCCASTAT_ISOLATION => DCCASTAT_LOGONIDLE\n", __func__);
							timeout = 300;		// if nothing happened, we can wait for a while
						} else {
							timeout = 40;
						}
					}
				}
				break;
			case DCCASTAT_SHORTINFO:
				if ((p = sigq_dcca_selectShortInfo(info.vid, info.uid, dccA_callback, fv)) != NULL) {
					log_msg (LOG_DCCA, "%s() requesting SHORTINFO (retry = %d)\n", __func__, retry);
					sigq_queuePacket(p);
					mt = (dec_msgtype) ulTaskNotifyTake(pdTRUE, NOTIFY_TIMEOUT);
					if (mt == DECODERMSG_SHORTINFO) {				// YES! we have got a valid answer.
						log_msg (LOG_DCCA, "%s() switch DCCASTAT_SHORTINFO => DCCASTAT_ASSIGN\n", __func__);
						stat = DCCASTAT_ASSIGN;
						l = NULL;
						retry = 5;
						timeout = 0;
					} else {
						if (--retry <= 0) {
							log_msg (LOG_DCCA, "%s() no answer, DCCASTAT_SHORTINFO => DCCASTAT_LOGONIDLE\n", __func__);
							stat = DCCASTAT_LOGONIDLE;
						}
						timeout = 50;
					}
				}
				break;
			case DCCASTAT_ASSIGN:
 				switch (info.adr & 0x3800) {
					case 0x0000:
					case 0x0800:
					case 0x1000:
					case 0x1800:
					case 0x2000:	// long address for mobile decoders
						wanted_adr = info.adr;
						if ((l = dccA_getAddress(DECODER_DCC_MOBILE, wanted_adr)) != NULL) coded_adr = l->adr;
						break;
					case PREFIX_EXT_ADR:	// extended accessory decoders
						wanted_adr = info.adr & 0x7FF;
						if ((l = dccA_getAddress(DECODER_DCC_EXT, wanted_adr)) != NULL) coded_adr = l->adr | PREFIX_EXT_ADR;
						break;
					case PREFIX_ACC_ADR:	// basic accessory decoders
						wanted_adr = info.adr & 0x7FF;
						if ((l = dccA_getAddress(DECODER_DCC_ACC, wanted_adr)) != NULL) coded_adr = l->adr | PREFIX_ACC_ADR;
						break;
					case PREFIX_SHORT_ADR:	// short address for mobile decoders
						wanted_adr = info.adr & 0x7F;
						if ((l = dccA_getAddress(DECODER_DCC_MOBILE, wanted_adr)) != NULL) {
							if (l->adr <= 127) {
								coded_adr = l->adr | PREFIX_SHORT_ADR;
							} else {
								coded_adr = l->adr;
							}
						}
						break;
					default:
						log_msg (LOG_WARNING, "%s() unknown decoder address request 0x%04x\n", __func__, info.adr);
						stat = DCCASTAT_LOGONIDLE;
						log_msg (LOG_DCCA, "%s() switch DCCASTAT_ASSIGN => DCCASTAT_LOGONIDLE\n", __func__);
						timeout = 50;
						continue;		// continue for-loop
				}
 				new_loco = false;
 				if (!l) {
 					coded_adr = 0;		// we assign a "PARK" address to the decoder
 				} else {
 					if (!l->dcca) {		// there was no dcca sub-structure, so this loco was not known before (might have been deleted!)
 						new_loco = true;
 						l->dcca = calloc (1, sizeof(*l->dcca));
 					}
 					if (l->dcca) l->dcca->adr_req = wanted_adr;
 				}
				if ((p = sigq_dcca_logonAssign(info.vid, info.uid, coded_adr, dccA_callback, fv)) != NULL) {
					sigq_queuePacket(p);
					log_msg (LOG_DCCA, "%s() assigning 0x%04x = %u (retry = %d)\n", __func__, coded_adr, (l) ? l->adr : 0, retry);
					mt = (dec_msgtype) ulTaskNotifyTake(pdTRUE, NOTIFY_TIMEOUT);
					if (mt == DECODERMSG_DECSTATE) {					// YES! we have got a valid answer.
						log_msg (LOG_DCCA, "%s() decoder is now assigned\n", __func__);
						if (l) l->config = CONF_DCCA;
						if (info.chg_flags || new_loco) {		// Changes detected or not yet read in
							dataspace = info.capa[1];
							stat = DCCASTAT_DATASPACE;
							retry = 5;
							clear_changeFlags = true;
							log_msg (LOG_DCCA, "%s() switch DCCASTAT_ASSIGN => DCCASTAT_DATASPACE\n", __func__);
						} else {
							if (info.chg_flags) {
								stat = DCCASTAT_CLEAR_CHGFLAGS;
								retry = 5;
								log_msg (LOG_DCCA, "%s() switch DCCASTAT_ASSIGN => DCCASTAT_CLEAR_CHGFLAGS\n", __func__);
							} else {
								stat = DCCASTAT_LOGONIDLE;
								log_msg (LOG_DCCA, "%s() switch DCCASTAT_ASSIGN => DCCASTAT_LOGONIDLE\n", __func__);
							}
						}
						timeout = 50;		// in all cases, we give the decoder a little time to relax
					} else {
						log_msg (LOG_DCCA, "%s() retrying ASSIGN!\n", __func__);
						if (--retry <= 0) {
							log_msg (LOG_DCCA, "%s() no answer, DCCASTAT_ASSIGN => DCCASTAT_LOGONIDLE\n", __func__);
							stat = DCCASTAT_LOGONIDLE;
							timeout = 20;
						}
					}
				}
				break;
			case DCCASTAT_DATASPACE:
				if (dataspace & 0x77) {
					if (dataspace & 0x01) {
						if (dccA_readDataSpace(0, info.vid, info.uid, dccA_dumpBlock, l) != DCCASTAT_BLOCK_OK) clear_changeFlags = false;
						dataspace &= ~0x01;
					} else if (dataspace & 0x02) {
						if (dccA_readDataSpace(1, info.vid, info.uid, dccA_dumpBlock, l) != DCCASTAT_BLOCK_OK) clear_changeFlags = false;
						dataspace &= ~0x02;
					} else if (dataspace & 0x04) {
						if (dccA_readDataSpace(2, info.vid, info.uid, dccA_ShortGUI, l) != DCCASTAT_BLOCK_OK) clear_changeFlags = false;
						dataspace &= ~0x04;
					} else if (dataspace & 0x10) {
						if (dccA_readDataSpace(4, info.vid, info.uid, dccA_Icons, l) != DCCASTAT_BLOCK_OK) clear_changeFlags = false;
						dataspace &= ~0x10;
					} else if (dataspace & 0x20) {
						if (dccA_readDataSpace(5, info.vid, info.uid, dccA_LongName, l) != DCCASTAT_BLOCK_OK) clear_changeFlags = false;
						dataspace &= ~0x20;
					} else if (dataspace & 0x40) {
						if (dccA_readDataSpace(6, info.vid, info.uid, dccA_ProductInformation, l) != DCCASTAT_BLOCK_OK) clear_changeFlags = false;
						dataspace &= ~0x40;
					}
				} else if (clear_changeFlags) {
					stat = DCCASTAT_CLEAR_CHGFLAGS;
					retry = 5;
					log_msg (LOG_DCCA, "%s() switch DCCASTAT_DATASPACE => DCCASTAT_CLEAR_CHGFLAGS\n", __func__);
				} else {
					stat = DCCASTAT_LOGONIDLE;
					log_msg (LOG_DCCA, "%s() Errors reading dataspaces: switch DCCASTAT_DATASPACE => DCCASTAT_LOGONIDLE\n", __func__);
					timeout = 20;
				}
				break;
			case DCCASTAT_CLEAR_CHGFLAGS:
				if ((p = sigq_dcca_decoderState(info.vid, info.uid, 0xFF, dccA_callback, fv)) != NULL) {
					sigq_queuePacket(p);
					log_msg (LOG_DCCA, "%s() clearing changeflags (retry = %d)\n", __func__, retry);
					mt = (dec_msgtype) ulTaskNotifyTake(pdTRUE, NOTIFY_TIMEOUT);
					if (mt == DECODERMSG_ACK) {
						stat = DCCASTAT_LOGONIDLE;
						timeout = 50;
						log_msg (LOG_DCCA, "%s() switch DCCASTAT_CLEAR_CHGFLAGS => DCCASTAT_LOGONIDLE\n", __func__);
					} else {
						log_msg (LOG_DCCA, "%s() retry clearing changeflags!\n", __func__);
						if (--retry <= 0) {
							log_msg (LOG_DCCA, "%s() no answer, DCCASTAT_CLEAR_CHGFLAGS => DCCASTAT_LOGONIDLE\n", __func__);
							stat = DCCASTAT_LOGONIDLE;
							timeout = 20;
						}
					}
				}
				break;
			default:
				break;
		}
	}
}
