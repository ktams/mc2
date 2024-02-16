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

#include <stdlib.h>
#include <string.h>
#include "rb2.h"
#include "decoder.h"
#include "events.h"
#include "config.h"

#define QUEUE_COUNT		3				///< we support up to 3 queues for signal generation (a hardware limit)

struct sigqueue {
	struct packet	*packets;			///< the packet list in this queue
	uint32_t		 class;				///< a bitfield with signal classes (enum sigclass) supported by this queue
};

//static struct sigqueue queue[QUEUE_COUNT];	///< future packet queues (qork in progress)
static struct packet *packets;			///< current packet queue
static SemaphoreHandle_t mutex;

/**
 * @ingroup Track
 * @{
 */

/**
 * Append a packet to the end of the current list of packets.
 * If the update flag is set and a matching packet is encountered
 * (loco number, command and track format) this packet is updated
 * with the new data and the original packet is freed.
 *
 * If the mutex cannot be aquired, the packet is dropped.
 *
 * \param p			the allocated packet memory to append to the end of the list
 * \param update	if set and a matching packet is encountered, the found packet is updated
 * 					and the packet-to-append is silently discarded
 */
static void sigq_appendPacket (struct packet *p, bool update)
{
	struct packet **pp, *q;

	if (!p) return;

	if (!mutex_lock(&mutex, 20, __func__)) {
		free (p);
		return;
	}
	p->next = NULL;
	pp = &packets;
	while ((q = *pp) != NULL) {
		if (update && q->adr == p->adr && q->fmt == p->fmt && q->cmd == p->cmd) {
			q->param = p->param;
			q->value = p->value;
			q->repeat = p->repeat;
			free (p);
			mutex_unlock(&mutex);
			return;
		}
		pp = &q->next;
	}
	*pp = p;
	mutex_unlock(&mutex);
}

/**
 * Insert a packet in front of the queue. This is normally an urgent packet
 * for whatever reason. This function will not scan for later packets which
 * might override the effect of this packet, so use wisely!
 *
 * The function will shift earlier packets, that where inserted before but have
 * not yet been put to track, farther down the queue - so the functionality is
 * not that of an "emergency"-queue.
 *
 * \param p		the packet to insert.
 */
static void sigq_insertPacket (struct packet *p)
{
	if (!p) return;

	if (!mutex_lock(&mutex, 20, __func__)) return;
	p->next = packets;
	packets = p;
	mutex_unlock(&mutex);
}

static bool sigq_isAccCommand (enum queue_cmd cmd)
{
	switch (cmd) {
		case QCMD_MAGNET_ON:
		case QCMD_MAGNET_OFF:
		case QCMD_ACC_RESET:
		case QCMD_DCC_POM_ACC_READ:
		case QCMD_DCC_POM_ACC_WRITE:
		case QCMD_DCC_POM_ACC_WRITEBIT:
		case QCMD_DCC_POM_EXT_READ:
		case QCMD_DCC_POM_EXT_WRITE:
		case QCMD_DCC_POM_EXT_WRITEBIT:
		case QCMD_DCC_XACCASPECT:
		case QCMD_DCC_ACCNOP:
		case QCMD_DCC_EXTACCNOP:
			return true;
		default:
			return false;
	}
}

/**
 * Allocate a packet structure on the heap and fill in some basic stuff from the parameters.
 * This is the module-internal function that gets the repeat count from the format settings.
 *
 * \param adr		the loco address if any
 * \param format	the track format for that packet
 * \param cmd		the command to set
 * \return			a pointer to an allocated packet or NULL, if out of memory
 */
static struct packet *_sigq_genPacket (int adr, enum fmt format, enum queue_cmd cmd)
{
	struct fmtconfig *f = cnf_getFMTconfig();
	struct packet *p;

	if ((p = calloc(1, sizeof(*p))) != NULL) {
		p->cmd = cmd;
		p->adr = adr;
		p->fmt = format;
		if (sigq_isAccCommand(cmd)) p->repeat = f->accrepeat;
		else if (FMT_IS_MM(format)) p->repeat = f->mm.repeat;
		else if (FMT_IS_DCC(format)) p->repeat = f->dcc.repeat;
		else if (FMT_IS_M3(format)) p->repeat = f->m3.repeat;
		else p->repeat = 1;		// should never happen
	}
	return p;
}

/**
 * Allocate a packet structure on the heap and fill in some basic stuff from the parameters.
 *
 * \param l			the loco if this packet is related to a loco command, else NULL
 * \param format	the loco format for that packet (needed if a NON-loco command is created)
 * \param cmd		the command to set
 * \return			a pointer to an allocated packet or NULL, if out of memory
 */
struct packet *sigq_genPacket (const ldataT *l, enum fmt format, enum queue_cmd cmd)
{
	struct packet *p;
	int adr;

	if (l) {
		format = l->loco->fmt;
		adr = l->loco->adr;
	} else {
		adr = 0;
	}
	if ((p = _sigq_genPacket(adr, format, cmd)) != NULL) {
		if (FMT_IS_MM1(p->fmt) && p->cmd == QCMD_MM_FDFUNCS) p->fmt = FMT_MM1_FD;
		if (l) memcpy (p->funcs, l->funcs, sizeof(p->funcs));	// always supply the functions (we may need them anyway)!
	}
	return p;
}

/**
 * append a DCC idle packet to the queue. This can be used to insert some
 * delay for older decoders (they don't like packet bursts) or for some DCC
 * programming tasks.
 *
 * \param repeat	the number of idle packets to send
 * \return			0 for success, -1 for error (out-of-memory, unsupported format, ...)
 */
int sigq_dcc_idle (int repeat)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_IDLE)) == NULL) return -1;
	p->repeat = repeat;				// override packet repeat count
	sigq_appendPacket(p, false);
	return 0;
}

/**
 * append a DCC reset packet to the queue. This can be used to get DCC decoders
 * switching to programming mode.
 *
 * \param repeat	the number of reset packets to send
 * \return			0 for success, -1 for error (out-of-memory, unsupported format, ...)
 */
int sigq_dcc_reset (int repeat)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_RESET)) == NULL) return -1;
	p->repeat = repeat;				// override packet repeat count
	sigq_appendPacket(p, false);
	return 0;
}

/**
 * DCC programming track function to verify a single bit in a CV.
 *
 * \param cv		the CV (0-based) to verify the bit to
 * \param bit		the bit number to verify (0 - 7)
 * \param val		the value to compare the bit with (0/false or 1/true)
 * \param repeat	the repeat count if not successfull
 * \return			a packet with the neccessary settings
 */
int sigq_dcc_cvVerfyBit (int cv, int bit, bool val, int repeat)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_PT_VERIFYBIT)) == NULL) return -1;
	p->repeat = repeat;				// override packet repeat count
	p->cva.cv = cv;
	p->value.bitval = (val) ? 1 : 0;
	p->value.bitpos = bit;
	sigq_appendPacket(p, false);
	return 0;
}

/**
 * DCC programming track function to write a single bit in a CV.
 *
 * \param cv		the CV (0-based) to write the bit to
 * \param bit		the bit number to write (0 - 7)
 * \param val		the value to write to the bit with (0/false or 1/true)
 * \param repeat	the repeat count if not successfull
 * \return			a packet with the neccessary settings
 */
int sigq_dcc_cvWriteBit (int cv, int bit, bool val, int repeat)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_PT_WRITEBIT)) == NULL) return -1;
	p->repeat = repeat;				// override packet repeat count
	p->cva.cv = cv;
	p->value.bitval = (val) ? 1 : 0;
	p->value.bitpos = bit;
	sigq_appendPacket(p, false);
	return 0;
}

/**
 * DCC programming track function to verify a CV byte.
 *
 * \param cv		the CV (0-based) to verify the bit to
 * \param val		the value to compare with the CV
 * \param repeat	the repeat count if not successfull
 * \return			a packet with the neccessary settings
 */
int sigq_dcc_cvVerfyByte (int cv, uint8_t val, int repeat)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_PT_VERIFYBYTE)) == NULL) return -1;
	p->repeat = repeat;				// override packet repeat count
	p->cva.cv = cv;
	p->value.i32 = val;
	sigq_appendPacket(p, false);
	return 0;
}

/**
 * DCC programming track function to write a CV byte.
 *
 * \param cv		the CV (0-based) to write the value to
 * \param val		the value to write to the CV
 * \param repeat	the repeat count if not successfull
 * \return			a packet with the neccessary settings
 */
int sigq_dcc_cvWriteByte (int cv, uint8_t val, int repeat)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_PT_WRITEBYTE)) == NULL) return -1;
	p->repeat = repeat;				// override packet repeat count
	p->cva.cv = cv;
	p->value.i32 = val;
	sigq_appendPacket(p, false);
	return 0;
}

/**
 * Create a new speed packet for the given loco. The speed must already be in a
 * valid range (responsibility of the upper layer!). The upper layer should also
 * make sure, that the information in ldataT pointer is not clobbered while we
 * extract the information. That means, it's best to hold the loco mutex while
 * calling this function.
 *
 * Currently, speed packets are sent twice for DCC and M3 and four times for
 * the Motorola formats.
 *
 * MM2 packets should be sent out with direction information.
 *
 * \param l			pointer to the loco that this packet should be created for
 * \param speed		the speed to senmd to the decoder, useful for intermediate speedsteps not corresponding to current loco speed
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_speedPacket (const ldataT *l, int speed)
{
	struct packet *p;

	if (!l || !l->loco) return NULL;		// if no sensefull information is supplied, we won't create a packet
	if ((p = sigq_genPacket(l, 0, QCMD_SETSPEED)) == NULL) return NULL;
	p->value.i32 = speed & 0xFF;				// we only ever support speedcodes up to 127 + direction bit!
	if (l->loco->fmt == FMT_MM2_27A) p->cmd = QCMD_MM_SETSPEED_27A;	// this is special ...

	return p;
}

/**
 * Generates a packet to send a function to the loco.
 *
 * \param l		pointer to the loco that this packet should be created for
 * \param f		the function that should be transmitted (the value of that function is included
 * 				in the current function bit status of the loco)
 * \return		a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_funcPacket (const ldataT *l, int f)
{
	struct packet *p = NULL;

	if (!l || !l->loco) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if (f < 0 || f >= LOCO_MAX_FUNCS) return NULL;	// ignore functions that are really out of range

	switch (l->loco->fmt) {
		case FMT_MM1_14:
			if (f == 0) {
				p = sigq_speedPacket(l, l->speed);
			} else {
				p = sigq_genPacket(l, 0, QCMD_MM_FDFUNCS);
			}
			break;
		case FMT_MM2_14:
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			if (f == 0) {
				p = sigq_speedPacket(l, l->speed);
			} else if (f <= 4) {
				p = sigq_genPacket(l, 0, QCMD_MM_SETF1 + f - 1);
				if (p) p->value.i32 = l->speed & 0xFF;
			}
			break;
		case FMT_DCC_14:
		case FMT_DCC_28:
		case FMT_DCC_126:
		case FMT_DCC_SDF:
			if (f == 0 && l->loco->fmt == FMT_DCC_14) {		// F0 is included in speed packet for the 14 speed decoders only
				p = sigq_speedPacket(l, l->speed);
			} else if (f <= 4) {							// this includes F0 for 28 and 126 speed decoders
				p = sigq_genPacket(l, 0, QCMD_DCC_SETF1_4);
			} else if (f <= 8) {
				p = sigq_genPacket(l, 0, QCMD_DCC_SETF5_8);
			} else if (f <= 12) {
				p = sigq_genPacket(l, 0, QCMD_DCC_SETF9_12);
			} else if (f <= 20) {
				p = sigq_genPacket(l, 0, QCMD_DCC_SETF13_20);
			} else if (f <= 28) {
				p = sigq_genPacket(l, 0, QCMD_DCC_SETF21_28);
			}
			break;
		case FMT_M3_126:
			if (f <= 15) {
				p = sigq_genPacket(l, 0, QCMD_SETFUNC);
			} else {
				p = sigq_genPacket(l, 0, QCMD_M3_SINGLEFUNC);
				if (p) p->param.i32 = f;
			}
			break;
		default:
			break;
	}
	return p;
}

/**
 * Generates an emergency stop packet for the loco.
 *
 * \param l		pointer to the loco that this packet should be created for
 * \return		a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_emergencyStopPacket (const ldataT *l)
{
	struct packet *p;

	if (!l || !l->loco) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if ((p = sigq_genPacket(l, 0, QCMD_EMERGENYSTOP)) != NULL) {	// this is an important information
		if (p->repeat < 5) p->repeat = 5;					// as this is importent, repeat it at least 5 times
		p->value.i32 = l->speed & 0x80;						// the current direction should be kept
	}
	return p;
}

/**
 * Generates a binary state packet for a DCC decoder.
 *
 * \param l		pointer to the loco that this packet should be created for
 * \param state	the state number to control
 * \param on	the new status of the addressed state (ON or OFF)
 * \return		a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_binStatePacket (const ldataT *l, int state, bool on)
{
	struct packet *p;

	if (!l || !l->loco) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if ((p = sigq_genPacket(l, 0, QCMD_DCC_BINSTATE)) != NULL) {
		if (p->repeat < 5) p->repeat = 5;		// according to RCN212 this packet should be transmitted at least 3 times!
		p->param.i32 = state;					// the state address
		p->value.i32 = on;						// the new status of the binary state
	}
	return p;
}

/**
 * Generates a packet to send a SDF combination packet to the loco (DCC only).
 *
 * \param l		pointer to the loco that this packet should be created for
 * \return		a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_sdfPacket (const ldataT *l)
{
	struct packet *p;

	if (!l || !l->loco) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if (l->loco->fmt != FMT_DCC_SDF) return NULL;
	if ((p = sigq_genPacket(l, 0, QCMD_DCC_SDF)) != NULL) {
		p->value.i32 = l->speed & 0xFF;
		p->param.i32 = l->loco->maxfunc;
	}

	return p;
}

/**
 * Generates a m3 beacon packet
 *
 * This can be used to force a decoder to become unassigned (it looses
 * it's track address and cannot be controlled anymore). This is for
 * programming purposes only, because all decoders will be logged out
 * on the track.
 *
 * The basic usage pattern is:
 *   - switch on power on the programming track and wait for output to power up
 *   - send some m3 beacon packets with an arbitrary beacon and announce counter
 *   - wait a short moment until the track output becomes idle
 *   - send some m3 beacon packets with the target beacon and announce counter
 *     that is used in normal drive mode (this is the latest point where the
 *     decoder will be unaddressed.
 *   - do a binary tree search for the decoder address (\ref sig_searchM3Loco())
 *   - supply a track address to the found decoder ID
 *
 * \param beacon	the 32 bit station ID to send
 * \param announce	the 16 bit announce counter to send
 * \param repeat	the packet repeat counter
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_m3BeaconPacket (uint32_t beacon, uint16_t announce, int repeat)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_M3_126, QCMD_M3_BEACON)) != NULL) {
		p->repeat = repeat;				// override packet repeat count
		p->param.u32 = beacon;
		p->value.u32 = announce;
	}
	return p;
}

/**
 * Generates a m3 decoder search packet
 *
 * \param uid		the 32 bit UID that should be used in searches
 * \param len		the length (in bits, starting from MSB, range 0 .. 32) that should be compared by the decoder
 * \param handler	a function that handles the result of this read out
 * \param priv		a private function parameter
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_m3SearchPacket (uint32_t uid, int len, reply_handler handler, flexval priv)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_M3_126, QCMD_M3_SEARCH)) != NULL) {
		p->repeat = 1;				// override packet repeat count
		p->param.i32 = len;
		if (p->param.i32 < 0) p->param.i32 = 0;
		if (p->param.i32 > 32) p->param.i32 = 32;
		p->value.u32 = uid;
		p->cb = handler;
		p->priv = priv;
	}
	return p;
}

/**
 * Generates a m3 packet that assigns a new address to the decoder with the given UID.
 *
 * \param uid		the 32 bit UID of the tageted decoder
 * \param adr		the new address on track that should be assigned
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_m3NewAddress (uint32_t uid, int adr)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_M3_126, QCMD_M3_NADR)) != NULL) {
		p->adr = adr;
		p->value.u32 = uid;
	}
	return p;
}

/**
 * Generates a m3 packet that reads bytes from subaddress of CV
 *
 * \param adr		the loco decoder address to use
 * \param cva		the CV address to read
 * \param bytes		the number of bytes to read (1, 2, 4, or 8)
 * \param handler	a function that handles the result of this read out
 * \param priv		a private function parameter
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_m3ReadCV (int adr, cvadrT cva, int bytes, reply_handler handler, flexval priv)
{
	struct packet *p;

	if (cva.m3cv < MIN_M3_CVADR || cva.m3cv > MAX_M3_CVADR || cva.m3sub < 0 || cva.m3sub > MAX_M3_CVSUBADR) return NULL;
	if ((p = sigq_genPacket(0, FMT_M3_126, QCMD_M3_CVREAD)) != NULL) {
		p->adr = adr;
		p->cva = cva;
		p->param.i32 = bytes;
		p->repeat = 1;
		p->cb = handler;
		p->priv = priv;
	}
	return p;
}

/**
 * Generates a m3 packet that writes multiple bytes to a CV (CA, Configuration Array)
 *
 * \param adr		the loco decoder address to use
 * \param cva		the CV and subaddress to write
 * \param val		the values to write
 * \param len		how much bytes to write (1, 2, 4 bytes, may be limited by the decoder)
 * \param repeat	the number of packets to send
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_m3WriteCVar (int adr, cvadrT cva, uint8_t *val, int len, int repeat)
{
	struct packet *p;

	if (cva.m3cv < MIN_M3_CVADR || cva.m3cv > MAX_M3_CVADR || cva.m3sub < 0 || cva.m3sub > MAX_M3_CVSUBADR) return NULL;
	if ((p = sigq_genPacket(0, FMT_M3_126, QCMD_M3_CVWRITE)) != NULL) {
		p->adr = adr;
		p->cva = cva;
		p->value.ui8[0] = val[0];	// just copy 4 bytes - the <len> parameter specifies how much bytes are used
		p->value.ui8[1] = val[1];
		p->value.ui8[2] = val[2];
		p->value.ui8[3] = val[3];
		p->param.i32 = len;
		p->repeat = repeat;
	}
	return p;
}

/**
 * Generates a m3 packet that writes a byte to subaddress of CV
 *
 * \param adr		the loco decoder address to use
 * \param cva		the CV and subaddress to write
 * \param val		the value to write
 * \param repeat	the number of packets to send
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_m3WriteCV (int adr, cvadrT cva, uint32_t val, int repeat)
{
	uint8_t c;

	c = val & 0xFF;
	return sigq_m3WriteCVar(adr, cva, &c, 1, repeat);
}

/**
 * Generate a packet for the model time in DCC format.
 * Because there are too many single values, we already
 * create the resulting 3 bytes of packed information
 * here as the u32 value.
 *
 * @param h			the hours (0 .. 23) to send
 * @param m			the minutes (0 .. 59) to send
 * @param wd		the week day (0 .. 6 with 0 = Monday) to send
 * @param factor	a speedup factor for the model time in the range 0 (stopped), 1 (real time) .. 63 (very fast)
 * @param update	a flag that informs the decoders, that the time was arbitrarily set, i.e. jumped
 * @return			the generated packet or NULL, if an error occured
 */
struct packet *sigq_modelTimePacket (int h, int m, int wd, int factor, bool update)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_MODELTIME)) != NULL) {
		p->repeat = 1;						// override packet repeat count
		p->value.u32 = (0b00) << 22;		// the code 0b00 signifies the transmission of model TIME
		p->value.u32 |= (m & 0x3F) << 16;
		p->value.u32 |= (wd & 0x07) << 13;
		p->value.u32 |= (h & 0x1F) << 8;
		p->value.u32 |= (factor & 0x3F) << 0;
		if (update) p->value.u32 |= 0x80;
	}
	return p;
}

/**
 * Generate a packet for the model date in DCC format.
 * Because there are too many single values, we already
 * create the resulting 3 bytes of packed information
 * here as the u32 value.
 *
 * @param y		the year (0 .. 4095) to send
 * @param m		the month (1 .. 12) to send
 * @param d		the day in month (1 .. 31) to send
 * @return		the generated packet or NULL, if an error occured
 */
struct packet *sigq_modelDatePacket (int y, int m, int d)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_MODELTIME)) != NULL) {
		p->repeat = 1;						// override packet repeat count
		p->value.u32 = (0b01) << 22;		// the code 0b01 signifies the transmission of model DATE
		p->value.u32 |= (d & 0x1F) << 16;
		p->value.u32 |= (m & 0x0F) << 12;
		p->value.u32 |= (y & 0xFFF) << 0;
	}
	return p;
}

/**
 * Generate a packet that will transport the current system time
 * (in milliseconds since system startup).
 *
 * This is defined only for DCC.
 *
 * @return	the generated packet or NULL, if an error occured
 */
struct packet *sigq_sysTimePacket (void)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_14, QCMD_DCC_SYSTIME)) != NULL) {
		p->repeat = 1;						// override packet repeat count
		p->value.u32 = xTaskGetTickCount();
	}
	return p;
}

/**
 * Append a magnet switching packet to the queue.
 *
 * \param t			the turnout to switch
 * \param thrown	true, if the turnouit should be switched to thrown, false for straight
 * \param on		true to switch on magnet, false to switch off
 * \param repeat	the number of packets to send
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_magnetPacket (turnoutT *t, bool thrown, bool on)
{
	struct packet *p;

	if (!t || !t->adr) return NULL;		// no information to create packet
	if ((p = sigq_genPacket(NULL, t->fmt, (on) ? QCMD_MAGNET_ON : QCMD_MAGNET_OFF)) != NULL) {
		p->adr = t->adr;
		p->param.i32 = thrown;
	}
	return p;
}

struct packet *sigq_accNop (turnoutT *acc)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, TFMT_DCC, QCMD_DCC_ACCNOP)) != NULL) {
		if (acc) p->adr = acc->adr;
		else p->adr = 0x7FF;		// max. 11 bit value as address
	}
	return p;
}

struct packet *sigq_extaccNop (extaccT *xacc)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, TFMT_DCC, QCMD_DCC_EXTACCNOP)) != NULL) {
		if (xacc) p->adr = xacc->adr;
		else p->adr = 0x7FF;		// max. 11 bit value as address
	}
	return p;
}

/**
 * Create a packet to switch the aspect (value) of an extended accessory decoder.
 *
 * \param xacc		the extended accessory decoder to manipulate
 * \param aspect	the new value (aspect) to send to the decoder
 * \param repeat	number of repetions for this packet
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_extaccPacket (extaccT *xacc, int aspect)
{
	struct packet *p;

	if (!xacc || !xacc->adr) return NULL;		// no information to create packet
	if ((p = sigq_genPacket(NULL, xacc->fmt, QCMD_DCC_XACCASPECT)) != NULL) {
		p->adr = xacc->adr;
		p->value.i32 = aspect;
	}
	return p;
}

#if 0
/**
 * Append a DCC OTCV verify (== read) command to the packet queue
 *
 * \param l			the mobile decoder to read
 * \param cv		the CV to read
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_OTCVreadPacket (const ldataT *l, int cv)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	struct packet *p = NULL;

	if (!l || !l->loco) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if (!FMT_IS_DCC(l->loco->fmt)) return NULL;		// this command is only intended for DCC decoders

	if ((p = sigq_genPacket(l, 0, QCMD_DCC_POM_READ)) != NULL) {
		p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
		p->cva.cv = cv;							// store CV address in the unstructured linear cv member of cva
	}
	return p;
}

/**
 * Append a DCC OTCV verify (== read) command to the packet queue
 *
 * \param t			the accessory decoder to read
 * \param cv		the CV to read
 * \param repeat	the number of packets to send
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_OTCVreadPacketAcc (const turnoutT *t, int cv)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	struct packet *p = NULL;

	if (!t) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if (!FMT_IS_DCC(t->fmt) || !FMT_IS_TURNOUT(t->fmt)) return NULL;		// this command is only intended for DCC accessory decoders

	if ((p = sigq_genPacket(NULL, t->fmt, QCMD_DCC_POM_ACC_READ)) != NULL) {
		p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
		p->adr = t->adr;
		p->cva.cv = cv;							// store CV address in the unstructured linear cv member of cva
	}
	return p;
}

/**
 * Append a DCC OTCV verify (== read) command to the packet queue
 *
 * \param ext		the extended accessory decoder to read
 * \param cv		the CV to read
 * \param repeat	the number of packets to send
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_OTCVreadPacketExt (const extaccT *ext, int cv)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	struct packet *p = NULL;

	if (!ext) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if (!FMT_IS_DCC(ext->fmt) || !FMT_IS_TURNOUT(ext->fmt)) return NULL;		// this command is only intended for DCC accessory decoders

	if ((p = sigq_genPacket(NULL, ext->fmt, QCMD_DCC_POM_EXT_READ)) != NULL) {
		p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
		p->adr = ext->adr;
		p->cva.cv = cv;							// store CV address in the unstructured linear cv member of cva
	}
	return p;
}

/**
 * Append a DCC OTCV write command to the packet queue
 *
 * \param l			the loco decoder that will be addressed
 * \param cv		the CV to write
 * \param val		the value to write
 * \param repeat	the number of packets to send
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_OTCVwritePacket (const ldataT *l, int cv, uint8_t val)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	struct packet *p = NULL;

	if (!l || !l->loco) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if (!FMT_IS_DCC(l->loco->fmt)) return NULL;		// this command is only intended for DCC decoders

	if ((p = sigq_genPacket(l, 0, QCMD_DCC_POM_WRITE)) != NULL) {
		p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
		p->cva.cv = cv;							// store CV address in the unstructured linear cv member of cva
		p->value.u32 = val;
	}
	return p;
}

/**
 * Append a DCC OTCV write command to the packet queue
 *
 * \param t			the accessory decoder to write to
 * \param cv		the CV to write
 * \param val		the value to write
 * \param repeat	the number of packets to send
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_OTCVwritePacketAcc (const turnoutT *t, int cv, uint8_t val)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	struct packet *p = NULL;

	if (!t) return NULL;						// if no sensefull information is supplied, we won't create a packet
	if (!FMT_IS_DCC(t->fmt)) return NULL;		// this command is only intended for DCC decoders

	if ((p = sigq_genPacket(NULL, t->fmt, QCMD_DCC_POM_ACC_WRITE)) != NULL) {
		p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
		p->adr = t->adr;
		p->cva.cv = cv;							// store CV address in the unstructured linear cv member of cva
		p->value.u32 = val;
	}
	return p;
}

/**
 * Append a DCC OTCV write command to the packet queue
 *
 * \param ext		the extended accessory decoder to write to
 * \param cv		the CV to write
 * \param val		the value to write
 * \param repeat	the number of packets to send
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_OTCVwritePacketExt (const extaccT *ext, int cv, uint8_t val)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	struct packet *p = NULL;

	if (!ext) return NULL;						// if no sensefull information is supplied, we won't create a packet
	if (!FMT_IS_DCC(ext->fmt)) return NULL;		// this command is only intended for DCC decoders

	if ((p = sigq_genPacket(NULL, ext->fmt, QCMD_DCC_POM_EXT_WRITE)) != NULL) {
		p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
		p->adr = ext->adr;
		p->cva.cv = cv;							// store CV address in the unstructured linear cv member of cva
		p->value.u32 = val;
	}
	return p;
}
#endif

/**
 * Use short form CV write access to a mapped CV register.
 * The command may contain one or two bytes for single CVs
 * or CV pairs respectively. See RailCommunity documentation
 * for more info.
 *
 * \param l			the mobile decoder to send the CV data to
 * \param cmd		which queue command to use (QCMD_DCC_XWR1 or QCMD_DCC_XWR2)
 * \param cv_code	the four bit CV code (not directly a CV but mapped according RailCommunity)
 * \param val		a pointer to a byte array containing one or two bytes
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_dcc_pomShortWrite (const ldataT *l, enum queue_cmd cmd, uint8_t cv_code, uint8_t *val)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	struct packet *p = NULL;

	if (!l || !l->loco) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if (!FMT_IS_DCC(l->loco->fmt)) return NULL;		// this command is only intended for DCC decoders
	if ((cmd != QCMD_DCC_XWR1) && (cmd != QCMD_DCC_XWR2)) return NULL;	// only these two command codes are allowed

	if ((p = sigq_genPacket(l, 0, cmd)) != NULL) {
		p->repeat = fcfg->dcc.pomrepeat;			// override packet repeat count
		p->param.i32 = cv_code & 0x0F;				// only 4 bits make up the CV selector (mapping to real CV defined by RailCommunity)
		p->value.ui8[0] = val[0];					// in all cases, we simply copy two bytes ... needed or not!
		p->value.ui8[1] = val[1];
	}
	return p;
}

/**
 * Use extended POM (xPOM) CV read/write access to up to four CVs.
 * When a bit write is requested, val[0] should already contain the
 * correct coding (1111DBBB - see RailCommunity documentation)
 *
 * \param l			the mobile decoder to send the CV data to
 * \param cmd		which queue command to use (one of the QCMD_DCC_XPOM_... commands)
 * \param cv		the 24 bit CV address
 * \param val		a pointer to a byte array containing one up to four bytes
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
struct packet *sigq_dcc_xPom (const ldataT *l, enum queue_cmd cmd, int cv, uint8_t *val)
{
	struct fmtconfig *fcfg = cnf_getFMTconfig();
	struct packet *p = NULL;

	if (!l || !l->loco) return NULL;				// if no sensefull information is supplied, we won't create a packet
	if (!FMT_IS_DCC(l->loco->fmt)) return NULL;		// this command is only intended for DCC decoders

	switch (cmd) {
		case QCMD_DCC_XPOM_RD_BLK:
		case QCMD_DCC_XPOM_WR_BIT:
		case QCMD_DCC_XPOM_WR_BYTE1:
		case QCMD_DCC_XPOM_WR_BYTE2:
		case QCMD_DCC_XPOM_WR_BYTE3:
		case QCMD_DCC_XPOM_WR_BYTE4:
			if ((p = sigq_genPacket(l, 0, cmd)) != NULL) {
				p->repeat = fcfg->dcc.pomrepeat;	// override packet repeat count
				p->cva.cv = cv;						// store CV address in the unstructured linear cv member of cva
				p->value.ui8[0] = val[0];			// in all cases, we simply copy four bytes ... needed or not!
				p->value.ui8[1] = val[1];
				p->value.ui8[2] = val[2];
				p->value.ui8[3] = val[3];
			}
			break;
		default:			// only special commands are allowed
			break;
	}

	return p;
}

/********************************************************************************************************
 * DCC-A packets ****************************************************************************************
 ********************************************************************************************************/
static struct packet *sigq_dcca_logonEnable (uint16_t cid, uint8_t session, enum queue_cmd cmd, reply_handler cb, flexval priv)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_126, cmd)) != NULL) {
		p->repeat = 1;		// always!
		p->param.u32 = cid;
		p->value.u32 = session;
		p->cb = cb;
		p->priv = priv;
	}
	return p;
}

struct packet *sigq_dcca_logonEnableAll (uint16_t cid, uint8_t session, reply_handler cb, flexval priv)
{
	return sigq_dcca_logonEnable(cid, session, QCMD_DCCA_LOGON_ENABLE_ALL, cb, priv);
}

struct packet *sigq_dcca_logonEnableLoco (uint16_t cid, uint8_t session, reply_handler cb, flexval priv)
{
	return sigq_dcca_logonEnable(cid, session, QCMD_DCCA_LOGON_ENABLE_LOCO, cb, priv);
}

struct packet *sigq_dcca_logonEnableAcc (uint16_t cid, uint8_t session, reply_handler cb, flexval priv)
{
	return sigq_dcca_logonEnable(cid, session, QCMD_DCCA_LOGON_ENABLE_ACC, cb, priv);
}

struct packet *sigq_dcca_logonEnableNow (uint16_t cid, uint8_t session, reply_handler cb, flexval priv)
{
	return sigq_dcca_logonEnable(cid, session, QCMD_DCCA_LOGON_ENABLE_NOW, cb, priv);
}

/**
 * Create a SELECT packet. the use of some fields depends on the select sub command.
 *
 * \param mfr		the decoder's manufacturer ID - usually only 8 bits but may takes 12 bits in final packet
 * \param uid		the decoder's serial number or UID as burned into the decoder by the manufacturer
 * \param cmd		the command for the packet - is later expanded to DCC-A SELECT with appropriate subcommand
 * \param blk		a block number to read or subcode
 * \param cv		a 24 bit linear CV address to start reading from, if block is the CV block 3
 * \param cvcount	the number of CV to read from block 3
 * \return			a pointer to the allocated packet or NULL, if an error occured
 */
static struct packet *sigq_dcca_select (uint16_t mfr, uint32_t uid, enum queue_cmd cmd, uint8_t blk,
	uint32_t cv, uint8_t cvcount, reply_handler cb, flexval priv)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_126, cmd)) != NULL) {
		p->repeat = 1;				// always!
		p->adr = mfr;				// adr is misused for the 12 bit manufacturer code
		p->param.u32 = uid;
		p->cva.cv = cv;
		p->value.ui8[0] = cvcount;	// misused for number of CVs to read or parameter of SetDeocderState (should currently be 0xFF)
		p->value.ui8[1] = blk;		// misused for block number when reading one of the blocks
		p->cb = cb;
		p->priv = priv;
	}
	return p;
}

struct packet *sigq_dcca_selectShortInfo (uint16_t mfr, uint32_t uid, reply_handler cb, flexval priv)
{
	return sigq_dcca_select(mfr, uid, QCMD_DCCA_SELECT_SHORTINFO, 0, 0, 0, cb, priv);
}

struct packet *sigq_dcca_selectBlock (uint16_t mfr, uint32_t uid, int blk, reply_handler cb, flexval priv)
{
	return sigq_dcca_select(mfr, uid, QCMD_DCCA_SELECT_RDBLOCK, blk, 0, 0, cb, priv);
}

struct packet *sigq_dcca_selectCVBlock (uint16_t mfr, uint32_t uid, uint32_t cv, uint8_t cnt, reply_handler cb, flexval priv)
{
	return sigq_dcca_select(mfr, uid, QCMD_DCCA_SELECT_RDBLOCK, 3, cv, cnt, cb, priv);
}

struct packet *sigq_dcca_decoderState (uint16_t mfr, uint32_t uid, uint8_t param, reply_handler cb, flexval priv)
{
	return sigq_dcca_select(mfr, uid, QCMD_DCCA_SELECT_DEC_STATUS, 0, 0, param, cb, priv);
}

struct packet *sigq_dcca_logonAssign (uint16_t mfr, uint32_t uid, int adr, reply_handler cb, flexval priv)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_126, QCMD_DCCA_LOGON_ASSIGN)) != NULL) {
		p->repeat = 1;
		p->adr = mfr;			// adr is misused for the 12 bit manufacturer code
		p->param.u32 = uid;
		p->value.i32 = adr;		// this is the decoder address to be assigned in DCC-A coding (RCN-218, appendix D)
		p->cb = cb;
		p->priv = priv;
	}
	return p;
}

struct packet *sigq_dcca_getDataStart (reply_handler cb, flexval priv)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_126, QCMD_DCCA_GETDATA_START)) != NULL) {
		p->repeat = 1;
		p->cb = cb;
		p->priv = priv;
	}
	return p;
}

struct packet *sigq_dcca_getDataCont (reply_handler cb, flexval priv)
{
	struct packet *p;

	if ((p = sigq_genPacket(NULL, FMT_DCC_126, QCMD_DCCA_GETDATA_CONT)) != NULL) {
		p->repeat = 1;
		p->cb = cb;
		p->priv = priv;
	}
	return p;
}

/********************************************************************************************************
 * Queue handling ***************************************************************************************
 ********************************************************************************************************/
/**
 * Append the given packet to the end of the packet queue.
 *
 * \param p		the packet to queue up
 */
void sigq_queuePacket (struct packet *p)
{
	sigq_appendPacket(p, false);
	if (p->cmd == QCMD_SETSPEED && p->fmt == FMT_DCC_14) sigq_dcc_idle(1);	// for the very old DCC decoders, add an idle packet to give them time ...
}

static const locoT dummy = {
	.adr = 3,
	.fmt = FMT_DCC_28,
};

static const ldataT dummylok = {
	.loco = (locoT *) &dummy,
	.speed = 0x80,
};

/**
 * Construct a packet for the loco we get from the refresh list.
 * The kind of function we send as a refresh can be made dependable
 * on it's age member.
 *
 * \return	A pointer to an allocated packet containing whatever we think might be usefull as a refresh.
 * 			A NULL is returned if we cannot get any refresh information for whatever reason.
 */
static struct packet* sigq_getrefresh(void)
{
	static TickType_t accnop, extnop;

	struct fmtconfig *cfg;
	struct packet *p = NULL;
	const ldataT *l;
	const locoT *lok;
	int cycles;

	cfg = cnf_getFMTconfig();
	if (cfg->sigflags & SIGFLAG_DCCNOP) {
		if (!accnop)
			accnop = tim_timeout(250);
		if (!extnop)
			extnop = tim_timeout(500);
	} else {
		accnop = extnop = 0;
	}

	if (tim_isover(accnop)) {
		p = sigq_accNop(NULL);
		accnop = tim_timeout(500);
	} else if (tim_isover(extnop)) {
		p = sigq_extaccNop(NULL);
		extnop = tim_timeout(500);
	} else {
		l = loco_refresh();
		if (!l || (lok = l->loco) == NULL) {
			l = &dummylok;
			lok = l->loco;
		}
		switch (lok->fmt) {
			case FMT_MM1_14:
				// send speed (incl. F0)
				p = sigq_speedPacket(l, l->speed);
				break;
			case FMT_MM2_14:
			case FMT_MM2_27A:
			case FMT_MM2_27B:
				switch (l->age % 5) {
					case 0:
						// Speed + direction (incl. F0)
						p = sigq_speedPacket(l, l->speed);
						if (lok->fmt == FMT_MM2_27A) {// if we must send two packet, the current packet is inserted in front of queue
							// to generate MM27a even speeds, we send "speed + 1" + "speed" in two packets
							if ((l->speed & 0x7F) > 0 && (l->speed & 1) == 0) {	// this is an even speed step, so send two packets
								p->repeat = 2;// as with the other speed packets, this is going to be sent twice
								sigq_insertPacket(p);// put the "real speed" packet to the front of the queue
								p = sigq_speedPacket(l, l->speed + 1);// ... and immedeately send out the "speed+1" packet
							}
						}
						break;
					case 1:
						// Speed + F1 (incl. F0)
						p = sigq_funcPacket(l, 1);
						break;
					case 2:
						// Speed + F2 (incl. F0)
						p = sigq_funcPacket(l, 2);
						break;
					case 3:
						// Speed + F3 (incl. F0)
						p = sigq_funcPacket(l, 3);
						break;
					case 4:
						// Speed + F4 (incl. F0)
						p = sigq_funcPacket(l, 4);
						break;
				}
				break;
			case FMT_DCC_14:
			case FMT_DCC_28:
			case FMT_DCC_126:
				cycles = 1;
				if (lok->fmt == FMT_DCC_14) {	// a theroretical decoder with 14 speeds and only F0 only ever needs the speed packet
					if (lok->maxfunc > 0)
						cycles++;				// but as soon as it has more functions, it will need more
				} else {
					cycles++;					// all other decoders are assumed to have at least F0 and makes a function packet necessary
				}
				if (lok->maxfunc > 4)
					cycles++;
				if (lok->maxfunc > 8)
					cycles++;
				if (lok->maxfunc > 12)
					cycles++;
				if (lok->maxfunc > 20)
					cycles++;
				switch (l->age % cycles) {
					case 0:						// Speed + direction (incl. F0 on 14-speed decoders)
						p = sigq_speedPacket(l, l->speed);
						break;
					case 1:						// F1 - F4 (incl. F0 on 28 and 126 speed decoders)
						p = sigq_funcPacket(l, 1);
						break;
					case 2:
						// F5 - F8
						p = sigq_funcPacket(l, 5);
						break;
					case 3:
						// F9 - F12
						p = sigq_funcPacket(l, 9);
						break;
					case 4:
						// F13 - F20
						p = sigq_funcPacket(l, 13);
						break;
					case 5:
						// F21 - F28
						p = sigq_funcPacket(l, 21);
						break;
				}
				break;
			case FMT_DCC_SDF:
				p = sigq_sdfPacket(l);
				break;
			case FMT_M3_126:					// send speed + direction + F0 - F15 (F16 to F127 are never refreshed)
				p = sigq_genPacket(l, 0, QCMD_M3_SPEEDFUNC);
				p->repeat = 1;					// override packet repeat count
				if (p)
					p->value.i32 = l->speed & 0xFF;
				break;
			default:
				return NULL;
		}
	}

	if (p) {		// here we change some aspects of the generated packet ...
#if 0
		if (FMT_IS_MM(p->fmt)) p->repeat = 2;
		else p->repeat = 1;
#else
		p->repeat = 1;
#endif
	}
	return p;
}

struct packet *sigq_getpacket(bool do_refresh)
{
	struct packet *p;

	if (!mutex_lock(&mutex, 20, __func__)) return NULL;
	if ((p = packets) != NULL) packets = p->next;
	mutex_unlock(&mutex);
	if (!p && do_refresh) p = sigq_getrefresh();
	return p;
}

/**
 * Push back a packet to the queue. This must be done if the previously fetched
 * packet cannt be handed over to the signal generation interrupt. This is the
 * case in the following situations:
 *   - a DCC-POM was requested while another POM for the same decoder address is still active
 *   - a DCC-XPOM was requested while another POM for the same decoder address is still active
 *   - a DCC-XPOM was requested while no free XPOM slot was available
 *   - a DCC-A action was requested while another DCC-A action is still running (should not happen though)
 *
 * If the mutex cannot be accuired, the packet is lost and must be freed.
 *
 * \param p		the packet to push back into the queue, NULL is ignored
 */
void sigq_pushBack (struct packet *p)
{
	if (!p) return;		// nothing to do ...

	if (mutex_lock(&mutex, 20, __func__)) {
		p->next = packets;
		packets = p;
		mutex_unlock(&mutex);
	} else {
		free (p);	// sorry, this packet is lost!
	}
}

int sigq_flush (void)
{
	struct packet *p;

	if (!mutex_lock(&mutex, 100, __func__)) return -1;
	while ((p = packets) != NULL) {
		packets = p->next;
		free (p);
	}
	mutex_unlock(&mutex);
	return 0;
}

/**
 * Check if the signal queue is idle (empty).
 * Attention: the last packet might be currently under work in signal generation interrupt.
 */
bool sigq_isIdle (void)
{
	return (packets == NULL);
}

/**
 * @}
 */
