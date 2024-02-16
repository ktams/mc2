/**
 * @file sniffer_m3.c
 *
 * @author Andi
 * @date   03.11.2019
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
#include "decoder.h"

/*
 * The sniffer and the M3-Decoder both use TIM2 with input capture.
 * The sniffer uses CH2 input and the M3 decoder uses CH4 input.
 *
 * The kernel clock to the timer is 200MHz. We use a prescaler of 20 to
 * get an effective count rate of 10MHz, which gives us a resolution
 * of 100ns (0,1µs).
 */

#define DEBUG_ONLY				0		///< if set to != 0 the indentified packets are only logged and not taken as control input
#define M3REPLY_DEBUGRDS		0		///< if set to != 0 the M3 reply debug is activated producing a CSV-File with all edges, etc.

#define QUEUE_LENGTH			128
#define DCC_PACKET_MAXLEN		20		///< this should be enough even for some newer format additions
#define M3_PACKET_MAXLEN		16		///< @todo check this out!
#define MM_PACKET_BITS			18		///< each MM (half) packet consists of 9 trits (making up 18 bits)
#define SIGNAL_MAX_WAIT			300		///< waiting for edges will return after this time without detected edge

#define DEBUG(X,FMT,...)		do { if (X) printf (FMT, __VA_ARGS__); } while (0)

enum dccphase {
	PH_DCC_SYNC = 0,				///< Startphase, waiting for synchronisation
	PH_DCC_RXBYTE,					///< receive the bits of one byte (16 edges, 8 pairs making up the 8 data bits)
	PH_DCC_STOPBIT1,				///< try to receive the first half of an interbyte stop bit (long pulse) or end the packet
	PH_DCC_STOPBIT2,				///< try to receive the second half of an interbyte stop bit (long pulse) or drop the packet
};

enum mmphase {
	PH_MM_SYNC = 0,					///< Startphase, waiting for synchronisation
	PH_MM_FIRSTHALF,				///< receive the first 18 bits (9 trits)
	PH_MM_INTERACKETGAP,			///< check for a reasonable pause inbetween the two "halves"
	PH_MM_SECONDHALF,				///< receive the second 18 bits (9 trits)
};

enum m3phase {
	PH_M3_SYNC = 0,					///< Startphase, waiting for synchronisation
	PH_M3_RECEIVE,					///< receive the data bits of a packet
};

enum mmedge {
	MMEDGE_NONE = 0,				///< any edge length that is not one of the following four "timings" or just undefined
	MMEDGE_FAST_SHORT,				///< short part of a MM fast packet (13us)
	MMEDGE_SLOW_SHORT,				///< short part of a MM slow packet (26us)
	MMEDGE_FAST_LONG,				///< short part of a MM fast packet (104us - 13us = 91us)
	MMEDGE_SLOW_LONG,				///< short part of a MM slow packet (208us - 26us = 182us)
};

#define MM_IS_SHORT(x)	((x) == MMEDGE_FAST_SHORT || (x) == MMEDGE_SLOW_SHORT)

struct dcc_packet {
	enum dccphase ph;				///< the current phase of the decoder
	int len;						///< the length (in bytes) of the packet
	int idx;						///< current half bit / edge index of received byte
	uint8_t data[DCC_PACKET_MAXLEN];	///< the data bytes of the packet
};

struct mm_packet {
	enum mmphase ph;				///< the current phase of the decoder
	bool fast;///< if set, this packet uses fast encoding (accessory decoders), else slow encoding (loco decoders)
	int len;		///< current length (in bits) of the current "half" packet
	uint32_t data1;			///< the data bits of first "half" packet
	uint32_t data2;	///< the data bits of second "half" packet (for comparision)
};

struct m3_packet {
	enum m3phase ph;				///< the current phase of the decoder
	int len;			///< current length (in bits) of the packet
	uint8_t data[M3_PACKET_MAXLEN];	///< the data bytes of the packet
};

struct decoder {
	uint8_t cmd;
	uint8_t mask;
	void (*func)(int, uint8_t*, int);
};

#define DCC_FAIL		100
#define DCC_TIME		800
#define MM_PAUSE_MIN	6000

#define DCC_SYNCMASK	0x7FFFF			///< the DCC sync pattern consists of at least 19 pulses
#define DCC_PREAMBLE	0x7FFFC			///< the DCC sync pattern is a preamble of at least 17 short pulses followed by two long pulses
#define M3_SYNCMASK		0x3F			///< check last 6 edges for a M3 synch pattern
#define M3_SYNCPATTERN	0b010010		///< this represents LSLLSL sync pattern for m3 (meaning: L=long, S=short)

static QueueHandle_t timings;
static volatile bool startup;

uint32_t volatile ui32DisplayFilter;	/* Bit	display
 --- DCC ---
 * 0	loco 28
 * 1	loco 128
 * 2	loco SDF
 * 3	spare
 * 4	loco function f0 - f4
 * 5	loco function f5 - f8
 * 6	loco function f9 - f12
 * 7	loco function f13 - f20
 * 8	loco function f21 - f28
 * 9	loco function f29 - f36
 * 10	loco function f37 - f44
 * 11	loco function f45 - f52
 * 12	loco function f53 - f60
 * 13	loco function f61 - f68
 * 14	spare
 * 15	RailCom
 * 16	basic accessory
 * 17	extended accessory
 * 18	spare
 * 19	spare
 --- MM ---
 * 20	loco
 * 21	accessory
 * 22
 */

static void init_tim2(void) {
	TIM2->CR1 = 0;					// disable and reset TIM2

	TIM2->CR2 = 0;					// no settings are used, idle states are LOW
	TIM2->SMCR = 0;				// no settings are used (slave mode is disabled)
	TIM2->DIER = 0;					// start with disabling interrupts
	TIM2->SR = 0;					// clear all status bits
	TIM2->BDTR = TIM_BDTR_OSSI;	// we keep control over the outputs, event if MOE is cleared

	// channel 1 is not used, channel 2 is capture input on TI2 with filter length 4
	TIM2->CCMR1 = (0b0010 << TIM_CCMR1_IC2F_Pos) | (0b01 << TIM_CCMR1_CC2S_Pos);
	// channel 3 is not used, channel 4 is capture input on TI4 with filter length 8 @ Fint
	TIM2->CCMR2 = (0b0011 << TIM_CCMR2_IC4F_Pos) | (0b01 << TIM_CCMR2_CC4S_Pos);

	// trigger on both edges for CH2 and negative edge CH4, enable the channels
	TIM2->CCER = TIM_CCER_CC4P | TIM_CCER_CC4E | TIM_CCER_CC2NP | TIM_CCER_CC2P
	        | TIM_CCER_CC2E;

	TIM2->PSC = 19;					// select a prescaler of 20 (PSC + 1)
	TIM2->ARR = 0xFFFFFFFFul;// this is also the reset value - we will use the whole 32 bits
	TIM2->TISEL = 0;// we will use default mapping (i.e. TI4 is CH4, TI2 is CH2 and the rest can be ignored)
	TIM2->AF1 = 0;		// we don't use any break input (we break for nobody!)

	NVIC_SetPriority(TIM2_IRQn, 4);	// set this as high priority (highest priority allowed to call FreeRTOS functions)
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->EGR = TIM_EGR_UG;				// update the registers
	TIM2->SR = 0;						// clear a possibly pending interrupt
	SET_BIT(TIM2->CR1, TIM_CR1_CEN);// enable the timer (currently without any interrupts - just free running
}

#if 0
bool snifferIsRunning (void)
{
	return (TIM2->DIER & TIM_DIER_CC2IE);
}

void sniffer_start (void)
{
	if (timings == NULL) return;				// don't start sniffer without a proper initialisation (in the sniffer thread)
	if (snifferIsRunning()) return;
	startup = true;
	if (timings) {
		xQueueReset(timings);
		SET_BIT (TIM2->DIER, TIM_DIER_CC2IE);		// enable the sniffer interrupt
	}
}

void sniffer_stop (void)
{
	CLEAR_BIT (TIM2->DIER, TIM_DIER_CC2IE);
}
#endif

#if DEBUG_ONLY
static void dcc_basicspeed (int adr, uint8_t *d, int len)
{
	bool rev;
	int speed;

	(void) len;

	rev = ((*d & 0x20) == 0);
	speed = ((*d & 0x0F) << 1) | ((*d & 0x10) >> 4);
	if ((speed & 0x1E) == 0x02) {	// emergency STOP
		log_msg (LOG_INFO, "%s(%d) EMERGENCY STOP %c\n", __func__, adr, (rev) ? 'R' : 'F');
	} else {
		log_msg (LOG_INFO, "%s(%d) %c%d\n", __func__, adr, (rev) ? 'R' : 'F', speed);
	}
}

static void dcc_speedfunc (int adr, uint8_t *d, int len)
{
	bool rev;
	int speed;

	rev = ((d[1] & 0x80) == 0);
	speed = d[1] & 0x7F;
	if (speed == 0x01) {	// emergency STOP
		log_msg (LOG_INFO, "%s(%d) EMERGENCY STOP %c ", __func__, adr, (rev) ? 'R' : 'F');
	} else {
		log_msg (LOG_INFO, "%s(%d) %c%d ", __func__, adr, (rev) ? 'R' : 'F', speed);
	}
	switch (len) {
		case 3:
			printf ("F0-F7=0x%02x\n", d[2]);
			break;
		case 4:
			printf ("F0-F7=0x%02x F8-F15=0x%02x\n", d[2], d[3]);
			break;
		case 5:
			printf ("F0-F7=0x%02x F8-F15=0x%02x F16-F23=0x%02x\n", d[2], d[3], d[4]);
			break;
		case 6:
			printf ("F0-F7=0x%02x F8-F15=0x%02x F16-F23=0x%02x F24-F31=0x%02x\n", d[2], d[3], d[4], d[5]);
			break;
	}
}

static void dcc_speed128 (int adr, uint8_t *d, int len)
{
	bool rev;
	int speed;

	(void) len;

	rev = ((d[1] & 0x80) == 0);
	speed = d[1] & 0x7F;
	if (speed == 0x01) {	// emergency STOP
		log_msg (LOG_INFO, "%s(%d) EMERGENCY STOP %c\n", __func__, adr, (rev) ? 'R' : 'F');
	} else {
		log_msg (LOG_INFO, "%s(%d) %c%d\n", __func__, adr, (rev) ? 'R' : 'F', speed);
	}
}

static const struct decoder dcc_mobile[] = {
	{ 0b01000000, 0b11000000, dcc_basicspeed },
	{ 0b00111100, 0b11111111, dcc_speedfunc },
	{ 0b00111111, 0b11111111, dcc_speed128 },
	{ 0, 0, NULL }		// MASK == 0 ends table
};
#endif

/**
 * Analyses a complete DCC packet.
 *
 * \param p		the packet bytes to analyse
 * \return		true if the packet is at least syntactically correct (length / XOR)
 */
static bool dcc_interpret(struct dcc_packet *p)
{
#if DEBUG_ONLY
	const struct decoder *dec;
#endif
	uint16_t adr;
	uint8_t *d, xor;
	bool loco = false;
	uint32_t newfuncs;
	int i;

	if (p->len < 3) {
		log_error("%s(): packet too short!\n", __func__);
		return false;
	}

	d = p->data;
	for (i = 0, xor = 0; i < p->len; i++) {
		xor ^= *d++;
	}
	if (xor) {
		log_error("%s(): packet XOR ERROR!\n", __func__);
		return false;
	}

	adr = p->data[0];
	d = &p->data[1];
#if DEBUG_ONLY
	dec = NULL;
#endif
	if (adr == 0) {					// broadcast
		log_msg(LOG_INFO, "%s(): BROADCAST loco decoders\n", __func__);
	} else if (adr <= 127) {		// short address mobile decoder
//		log_msg (LOG_INFO, "%s(): SHORT Address: %d / loco decoder\n", __func__, adr);
		loco = true;
#if DEBUG_ONLY
		dec = dcc_mobile;
#endif
	} else if (adr <= 191) {		// basic and extended accessory decoder
		// Paket:		1 0 A  A  A  A  A  A   |  1  A    A   A  D A  A  R
		// Adresse: 	– – A7 A6 A5 A4 A3 A2  |  – /A10 /A9 /A8 – A1 A0 –
		adr = ((adr & 0x3F) << 2) | (((*d & 0x70) ^ 0x70) << 4)
		        | ((*d & 0x06) >> 1);
		if (*d & 0x80) {
			trnt_switch((adr >> 2), *d & 1, 1);
			if (ACC_B) log_msg(LOG_INFO, "%s(): Address: %d / basic accessory -> DIR = %c\n", __func__, (adr >> 2), (*d & 1) ? '|' : '/');
		} else {
//			log_msg(LOG_WARNING, "%s(): extended accessory: not implemented yet!\n", __func__);
			if (ACC_E) log_msg(LOG_INFO, "%s(): Address: %d / extended accessory -> aspect = %d %s\n", __func__, adr,
					*(d + 1) & 0x7F, (*(d + 1)) ? "on" : "off");
		}
	} else if (adr <= 231) {		// long address mobile decoder
		adr = (adr & 0x3F) << 8 | *d++;
//		log_msg (LOG_INFO, "%s(): LONG Address: %d / loco decoder\n", __func__, adr);
		loco = true;
#if DEBUG_ONLY
		dec = dcc_mobile;
#endif
	} else if (adr <= 254) {		// reserved address range
		log_msg(LOG_INFO, "%s(): RESERVED Address: %d\n", __func__, adr);
	} else {	// adr == 255		// idle address
//		log_msg (LOG_INFO, "%s(): ILDE Address: %d\n", __func__, adr);
	}

#if DEBUG_ONLY
	if (dec) {
		while (dec->mask) {
			if ((*d & dec->mask) == dec->cmd) {
				if (dec->func) dec->func (adr, d, p->len - (d - p->data));
				break;
			}
			dec++;
		}
	}
#else
	if (loco) {
		uint8_t ui8;
		switch (*d & 0xE0) {
			case 0x20:
				//--------------------- Spd128 -----------------------------------
				ui8 = *(++d) & 0x7F;
				if (ui8 == 1) {
					loco_emergencyStop(adr);
					DEBUG(LOCO128, "%s(): Address: %d / loco decoder -> speed 128: emergency stop!\n", __func__, adr);
				} else {
					if (ui8) ui8--;
					ui8 |= *d & 0x80;
					DEBUG(LOCO128, "%s(): Address: %d / loco decoder -> speed 128 %s: %d\n", __func__, adr,
					        (ui8 & 0x80) ? "forward" : "backward", ui8 & 0x7F);
					rq_setSpeed(adr, (int) ui8);
				}
				break;

			case 0x30:
				//--------------------- Speed commands -----------------------------------
				if (*d == 0x3C) {
					//--------------------- SDF -----------------------------------
					ui8 = *(++d) & 0x7F;
					if (ui8 == 1) {
						loco_emergencyStop(adr);
						DEBUG(LOCO128, "%s(): Address: %d / loco decoder -> speed 128: emergency stop!\n", __func__, adr);
					} else {
						if (ui8) ui8--;
						ui8 |= *d & 0x80;
						// functions...
						// ToDo: Funktionen extrahieren und setzen.
						DEBUG(LOCO128, "%s(): Address: %d / loco decoder -> speed 128 %s: %d\n", __func__, adr,
						        (ui8 & 0x80) ? "forward" : "backward", ui8 & 0x7F);
						rq_setSpeed(adr, (int) ui8);
					}
				}
				break;

			case 0x60:
				//--------------------- Spd28 forward-----------------------------
				ui8 = *d & 0x0F;
				if (ui8 > 1) {
					ui8 -= 1;				// 14 FS
					ui8 <<= 1;				// 14 * 2 FS
					if (*d & 0x10) ui8++;	// 28 FS
				}
				if (ui8 > 1) {
					rq_setSpeed(adr, (int) ((ui8 - 1) | 0x80));
					DEBUG(LOCO28, "%s(): Address: %d / loco decoder -> speed 28 forward: %d\n", __func__, adr, ui8 - 1);
				} else if (ui8 == 1) {
					loco_emergencyStop(adr);
					DEBUG(LOCO28, "%s(): Address: %d / loco decoder -> speed 28: emergency stop!\n", __func__, adr);
				} else {
					rq_setSpeed(adr, (int) 0x80);
					DEBUG(LOCO28, "%s(): Address: %d / loco decoder -> speed 28 forward: 0 (HALT)\n", __func__, adr);
				}
				break;

			case 0x40:
				//--------------------- Spd28 reverse ----------------------------
				ui8 = *d & 0x0F;
				if (ui8 > 1) {
					ui8 = ui8 - 1;			// 14FS
					ui8 <<= 1;				// 14 * 2 FS
					if (*d & 0x10)
						ui8++;				// 28FS
				}
				if (ui8 > 1) {
					rq_setSpeed(adr, (int) (ui8 - 1));
					DEBUG(LOCO28, "%s(): Address: %d / loco decoder -> speed 28 backward: %d\n", __func__, adr, ui8 - 1);
				} else if (ui8 == 1) {
					loco_emergencyStop(adr);
					DEBUG(LOCO28, "%s(): Address: %d / loco decoder -> speed 28: emergency stop!\n", __func__, adr);
				} else {
					rq_setSpeed(adr, 0);
					DEBUG(LOCO28, "%s(): Address: %d / loco decoder -> speed 28 backward: 0 (HALT)\n", __func__, adr);
				}
				break;

			case 0x80:
				//--------------------- Funct1 (f0 - f4) -------------------------
				newfuncs = (*d << 1) | ((*d & 0x10) >> 4);
				rq_setFuncMasked(adr, newfuncs, FUNC_F0_F4);
				DEBUG(LOCOfunc1, "%s(): Address: %d / loco decoder -> functions F0 - f4: f0=%d, f1=%d, f2=%d, f3=%d, f4=%d\n",
				        __func__, adr, !!(*d & 0x10), *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8));
				break;

			case 0xA0:
				//--------------------- Funct2 (f5 - f12)  -----------------------
				if ((*d & 0xF0) == 0xB0) {
					newfuncs = *d << 5;
					rq_setFuncMasked(adr, newfuncs, FUNC_F5_F8);
					DEBUG(LOCOfunc2, "%s(): Address: %d / loco decoder -> functions f5 - f8: f5=%d, f6=%d, f7=%d, f8=%d\n",
					        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8));
				} else {
					newfuncs = *d << 9;
					rq_setFuncMasked(adr, newfuncs, FUNC_F9_F12);
					DEBUG(LOCOfunc3, "%s(): Address: %d / loco decoder -> functions f9 - f12: f9=%d, f10=%d, f11=%d, f12=%d\n",
					        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8));
				}
				break;

			case 0xC0:
				//--------------------- F13 bis F68 ------------------------------
				switch (*d++) {
					case 0xDE:	//F13 - F20
						newfuncs = *d << 13;
						rq_setFuncMasked(adr, newfuncs, FUNC_F13_F20);
						DEBUG(LOCOfunc4,
						        "%s(): Address: %d / loco decoder -> functions f13 - f20: f13=%d, f14=%d, f15=%d, f16=%d f17=%d, f18=%d, f19=%d, f20=%d\n",
						        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4),!!(*d & 8), !!(*d & 0x10), !!(*d & 0x20), !!(*d & 0x40), !!(*d & 0x80));
						break;

					case 0xDF:	//F21 bis F28
						newfuncs = *d << 21;
						rq_setFuncMasked(adr, newfuncs, FUNC_F21_F28);
						DEBUG(LOCOfunc5,
						        "%s(): Address: %d / loco decoder -> functions f21 - f28: f21=%d, f22=%d, f23=%d, f24=%d f25=%d, f26=%d, f27=%d, f28=%d\n",
						        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8), !!(*d & 0x10), !!(*d & 0x20), !!(*d & 0x40), !!(*d & 0x80));
						break;

					case 0xD8:	//F29 bis F36
						DEBUG(LOCOfunc6,
						        "%s(): Address: %d / loco decoder -> functions f29 - f36: f29=%d, f30=%d, f31=%d, f32=%d f33=%d, f34=%d, f35=%d, f36=%d\n",
						        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8), !!(*d & 0x10), !!(*d & 0x20), !!(*d & 0x40), !!(*d & 0x80));
						break;

					case 0xD9:	//F37 bis F44
						DEBUG(LOCOfunc7,
						        "%s(): Address: %d / loco decoder -> functions f37 - f44: f37=%d, f38=%d, f39=%d, f40=%d f41=%d, f42=%d, f43=%d, f44=%d\n",
						        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8), !!(*d & 0x10), !!(*d & 0x20), !!(*d & 0x40), !!(*d & 0x80));
						break;

					case 0xDA:	//F45 bis F52
						DEBUG(LOCOfunc8,
						        "%s(): Address: %d / loco decoder -> functions f45 - f52: f45=%d, f46=%d, f47=%d, f48=%d f49=%d, f50=%d, f51=%d, f52=%d\n",
						        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8), !!(*d & 0x10), !!(*d & 0x20), !!(*d & 0x40), !!(*d & 0x80));
						break;

					case 0xDB:	//F53 bis F60
						DEBUG(LOCOfunc9,
						        "%s(): Address: %d / loco decoder -> functions f53 - f60: f53=%d, f54=%d, f55=%d, f56=%d f57=%d, f58=%d, f59=%d, f60=%d\n",
						        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8), !!(*d & 0x10), !!(*d & 0x20), !!(*d & 0x40), !!(*d & 0x80));
						break;

					case 0xDC:	//F61 bis F68
						DEBUG(LOCOfunc10,
						        "%s(): Address: %d / loco decoder -> functions f61 - f68: f61=%d, f62=%d, f63=%d, f64=%d f65=%d, f66=%d, f67=%d, f68=%d\n",
						        __func__, adr, *d & 1, !!(*d & 2), !!(*d & 4), !!(*d & 8), !!(*d & 0x10), !!(*d & 0x20), !!(*d & 0x40), !!(*d & 0x80));
						break;
				}
				break;

			case 0xE0:
				//--------------------- POM --------------------------------------
				log_msg(LOG_INFO, "programming on main\n");
				break;

			default:
				log_msg(LOG_WARNING, " I don't know...\n");
				break;
		}
	}
#endif
	return true;
}

/**
 * The base part of the DCC decoder.
 * It is (re-)initialised, when called with a zero time (which otherwise is physically impossible).
 *
 * \param t		the time between the last and the current edge (i.e. pulse width) in 1/10us (100ns)
 * \return		true, if a valid packet was decoded
 */
static bool sniffer_dcc(uint32_t t)
{
	static uint32_t sr;	// a shift register with '1' for each short edge and '0' for long edges
	static struct dcc_packet pkt;

	if (t == 0) {
		pkt.ph = PH_DCC_SYNC;
		sr = 0;				// no '1' bits (preamble) received yet
		return false;
	}

	sr <<= 1;
	if (t < DCC_TIME) sr |= 1;

	switch (pkt.ph) {
		case PH_DCC_SYNC:
			if ((sr & DCC_SYNCMASK) == DCC_PREAMBLE) {
				pkt.ph = PH_DCC_RXBYTE;	// this is a DCC preamble + 1 zero (start-)bit - switch to receive
				pkt.len = pkt.idx = 0;
			}
			break;
		case PH_DCC_RXBYTE:
			pkt.idx++;
			if ((pkt.idx & 1) == 0) {	// event number of edges received
				if ((sr & 0b11) == 0b01 || (sr & 0b11) == 0b10) {// phase error (short/long or long/short in a single bit)
					pkt.ph = PH_DCC_SYNC;
					break;
				}
				pkt.data[pkt.len] <<= 1;
				if (sr & 1)
					pkt.data[pkt.len] |= 1;
				if (pkt.idx >= 16) {
					pkt.len++;
					pkt.ph = PH_DCC_STOPBIT1;
				}
			}
			break;
		case PH_DCC_STOPBIT1:
			if (sr & 1) {	// we have a short edge - that is the packet end marker
				pkt.ph = PH_DCC_SYNC;// start over with looking for synchronisation
				sr = 0;		// we should discard all 1-bits that may still belong to the packet!
				return dcc_interpret(&pkt);		// if packet is syntactically coorect it counts as valid packet
			} else {
				pkt.ph = PH_DCC_STOPBIT2;
			}
			break;
		case PH_DCC_STOPBIT2:
			if (sr & 1) {	// invalid inter-byte bit - drop (ignore) this packet
				pkt.ph = PH_DCC_SYNC;// restart over with looking for synchronisation
				sr = 0;		// we should discard all 1-bits that may still belong to the packet!
			} else if (pkt.len >= DCC_PACKET_MAXLEN) {	// the next byte would overflow our buffer - oversized packet
				pkt.ph = PH_DCC_SYNC;// restart over with looking for synchronisation
				sr = 0;		// we should discard all 1-bits that may still belong to the packet!
			} else {
				pkt.ph = PH_DCC_RXBYTE; // start over with receiving next byte
				pkt.idx = 0;			// the bit index is reset
			}
			break;
		default:					// emergency break
			pkt.ph = PH_DCC_SYNC;
			break;
	}

	return false;
}

static const uint8_t MM_revtable[] = { 0x50, 0xE5, 0x36, 0x1B, 0xC1, 0xC3, 0xC2,
        0xC4, 0x12, 0xF7, 0x48, 0x2D, 0x09, 0xEE, 0x3F, 0x24, 0x91, 0x99, 0x95,
        0x9D, 0x93, 0x9B, 0x97, 0x9F, 0x92, 0x9A, 0x96, 0x9E, 0x94, 0x9C, 0x98,
        0xA0, 0x06, 0xEB, 0x3C, 0x21, 0xD9, 0xDB, 0xDA, 0xDC, 0x18, 0xFD, 0x4E,
        0x33, 0x0F, 0xF4, 0x45, 0x2A, 0x03, 0xE8, 0x39, 0x1E, 0xCD, 0xCF, 0xCE,
        0xD0, 0x15, 0xFA, 0x4B, 0x30, 0x0C, 0xF1, 0x42, 0x27, 0x51, 0x71, 0x61,
        0x81, 0x59, 0x79, 0x69, 0x89, 0x55, 0x75, 0x65, 0x85, 0x5D, 0x7D, 0x6D,
        0x8D, 0xBF, 0x73, 0x63, 0x83, 0x5B, 0xC0, 0x6B, 0x8B, 0x57, 0x77, 0x67,
        0x87, 0x5F, 0x7F, 0x6F, 0x8F, 0x52, 0x72, 0x62, 0x82, 0x5A, 0x7A, 0x6A,
        0x8A, 0x56, 0x76, 0x66, 0x86, 0x5E, 0x7E, 0x6E, 0x8E, 0x54, 0x74, 0x64,
        0x84, 0x5C, 0x7C, 0x6C, 0x8C, 0x58, 0x78, 0x68, 0x88, 0x60, 0x80, 0x70,
        0x90, 0x02, 0xE7, 0x38, 0x1D, 0xC9, 0xCB, 0xCA, 0xCC, 0x14, 0xF9, 0x4A,
        0x2F, 0x0B, 0xF0, 0x41, 0x26, 0xB1, 0xB9, 0xB5, 0xBD, 0xB3, 0xBB, 0xB7,
        0x53, 0xB2, 0xBA, 0xB6, 0xBE, 0xB4, 0xBC, 0xB8, 0x7B, 0x08, 0xED, 0x3E,
        0x23, 0xE1, 0xE3, 0xE2, 0xE4, 0x1A, 0xFF, 0x00, 0x35, 0x11, 0xF6, 0x47,
        0x2C, 0x05, 0xEA, 0x3B, 0x20, 0xD5, 0xD7, 0xD6, 0xD8, 0x17, 0xFC, 0x4D,
        0x32, 0x0E, 0xF3, 0x44, 0x29, 0x01, 0xE6, 0x37, 0x1C, 0xC5, 0xC7, 0xC6,
        0xC8, 0x13, 0xF8, 0x49, 0x2E, 0x0A, 0xEF, 0x40, 0x25, 0xA1, 0xA9, 0xA5,
        0xAD, 0xA3, 0xAB, 0xA7, 0xAF, 0xA2, 0xAA, 0xA6, 0xAE, 0xA4, 0xAC, 0xA8,
        0xB0, 0x07, 0xEC, 0x3D, 0x22, 0xDD, 0xDF, 0xDE, 0xE0, 0x19, 0xFE, 0x4F,
        0x34, 0x10, 0xF5, 0x46, 0x2B, 0x04, 0xE9, 0x3A, 0x1F, 0xD1, 0xD3, 0xD2,
        0xD4, 0x16, 0xFB, 0x4C, 0x31, 0x0D, 0xF2, 0x43, 0x28, };

static const uint8_t nibblereverse[] = { 0b0000, 0b1000, 0b0100, 0b1100, 0b0010,
        0b1010, 0b0110, 0b1110, 0b0001, 0b1001, 0b0101, 0b1101, 0b0011, 0b1011,
        0b0111, 0b1111 };

static uint8_t mm_bytereverse(uint8_t val) {
	return (nibblereverse[val & 0x0F] << 4) | (nibblereverse[(val >> 4) & 0x0F]);
}

#if DEBUG_ONLY
/**
 * Extracts bits 7, 5, 3 and 1 from the MM-trits #6 - #9 and
 * returns the reversed bitorder (transmission is LSB first, so
 * will be seen in the MSB position).
 *
 * Scorzony explained the bit coding as follows:
 *		A   B   C   D		<- first part of each trit, compatibility bits (if ABCD == EFGH)
 *		  E   F   G   H		<- second part of each trit, if different from ABCD means MM2
 *		 #6  #7  #8  #9		<- TRIT numbers
 *
 * \param c		the MM data byte from track data
 * \return		the resulting 4 bit value from Scorzoni's bits ABCD in normal numeric representation
 */
static uint8_t mm_extractDCBA (uint8_t c)
{
	return ((c & 0x80) >> 7) | ((c & 0x20) >> 4) | ((c & 0x08) >> 1) | ((c & 0x02) << 2);
}

/**
 * Extracts bits 6, 4, 2 and 0 from the MM-trits #6 - #9 and
 * returns the reversed bitorder (transmission is LSB first, so
 * will be seen in the MSB position).
 *
 * Scorzony explained the bit coding as follows:
 *		A   B   C   D		<- first part of each trit, compatibility bits (if ABCD == EFGH)
 *		  E   F   G   H		<- second part of each trit, if different from ABCD means MM2
 *		 #6  #7  #8  #9		<- TRIT numbers
 *
 * \param c		the MM data byte from track data
 * \return		the resulting 4 bit value from Scorzoni's bits EFGH in normal numeric representation
 */
static uint8_t mm_extractHGFE (uint8_t c)
{
	return ((c & 0x40) >> 6) | ((c & 0x10) >> 3) | ((c & 0x04) >> 0) | ((c & 0x01) << 3);
}
#endif

static uint8_t mm_funcHandler (int adr, int f, uint8_t funcs, bool on)
{
	uint8_t mask;

	if (f < 1 || f > 4) return funcs;	// only F1 - F4 are allowed
	mask = 0x02 << f;

	if (on) funcs |= mask;
	else funcs &= ~mask;
	loco_setFunc(adr, f, on);
	if (LOCOMM) printf(" F%d: %s", f, (on) ? "on" : "off");
	return funcs;
}

static bool mm_interpret(struct mm_packet *p)
{
	static uint8_t ui8Func, bDirection;

	ldataT *l;
	uint8_t adr, ui8MM_DATA, ui8TempFS, ui8;
#if DEBUG_ONLY
	uint8_t abcd, efgh;
#endif
	if (p->data1 != p->data2) {
		log_error("%s(): data mismatch: 0x%05lx <> 0x%05lx\n", __func__,
		        p->data1, p->data2);
		return false;
	}

	adr = MM_revtable[(p->data1 >> 10) & 0xFF];
	ui8Func = (p->data1 >> 8) & 3;

#if DEBUG_ONLY
	ui8MM_DATA = p->data1 & 0xFF;
	abcd = mm_extractDCBA(ui8MM_DATA);
	efgh = mm_extractHGFE(ui8MM_DATA);
	if (p->fast && ui8Func) {			// function decoder
		if (abcd != efgh) {				// MM2 not allowed for function decoders
			log_error ("%s(%d) FUNCTION does not allow MM2 signaling\n", __func__, adr);
		} else {
			log_msg (LOG_INFO, "%s(%d) F4-F1 = %c%c%c%c\n", __func__, adr,
					(abcd & 0x08) ? 'X' : '-', (abcd & 0x04) ? 'X' : '-',
					(abcd & 0x02) ? 'X' : '-', (abcd & 0x01) ? 'X' : '-');
		}
	} else if (p->fast && !ui8Func) {	// accessory (turnout) decoder
		int subadr;
		bool r;
		if (abcd != efgh) {				// MM2 not allowed for accessory decoders
			log_error ("%s(%d) ACCESSORY does not allow MM2 signaling\n", __func__, adr);
		} else {
			subadr = (abcd >> 1) & 0x03;
			r = !(abcd & 0x01);
			if ((ui8MM_DATA & 0x03) == 0) {
				log_msg (LOG_INFO, "%s(%d): T%d / ALL OFF\n", __func__, adr, adr + 1 + subadr);
			} else {
				log_msg (LOG_INFO, "%s(%d) T%d %c ON\n", __func__, adr, adr + 1 + subadr, r ? 'R' : 'G');
			}
		}
	} else {							// mobile decoder
		if (abcd == efgh) {					// MM1
			if (abcd == 0) {
				log_msg (LOG_INFO, "%s(%d) MM1 FUNC=%c HALT\n", __func__, adr, ui8Func ? 'X' : '-');
			} else if (abcd == 1) {
				log_msg (LOG_INFO, "%s(%d) MM1 FUNC=%c REVERSE\n", __func__, adr, ui8Func ? 'X' : '-');
			} else {
				log_msg (LOG_INFO, "%s(%d) MM1 FUNC=%c speed %d\n", __func__, adr, ui8Func ? 'X' : '-', abcd - 1);
			}
		} else {							// MM2
			if ((abcd < 8) && efgh == 0b1010) {				// forward lower speed
				if (abcd == 0) log_msg (LOG_INFO, "%s(%d) MM2 FWD FUNC=%c HALT\n", __func__, adr, (ui8Func & 0x02) ? 'X' : '-');
				else if (abcd == 1) log_msg (LOG_INFO, "%s(%d) MM2 FWD FUNC=%c EMERGENCY STOP\n", __func__, adr, (ui8Func & 0x02) ? 'X' : '-');
				else log_msg (LOG_INFO, "%s(%d) MM2 FWD FUNC=%c %d\n", __func__, adr, (ui8Func & 0x02) ? 'X' : '-', abcd - 1);
			} else if ((abcd >= 8) && efgh == 0b0010) {		// forward higher speed
				log_msg (LOG_INFO, "%s(%d) MM2 FWD FUNC=%c %d\n", __func__, adr, (ui8Func & 0x02) ? 'X' : '-', abcd - 1);
			} else if ((abcd < 8) && efgh == 0b1101) {		// reverse higher speed
				if (abcd == 0) log_msg (LOG_INFO, "%s(%d) MM2 REV FUNC=%c HALT\n", __func__, adr, (ui8Func & 0x02) ? 'X' : '-');
				else if (abcd == 1) log_msg (LOG_INFO, "%s(%d) MM2 REV FUNC=%c EMERGENCY STOP\n", __func__, adr, (ui8Func & 0x02) ? 'X' : '-');
				else log_msg (LOG_INFO, "%s(%d) MM2 REV FUNC=%c %d\n", __func__, adr, (ui8Func & 0x02) ? 'X' : '-', abcd - 1);
			} else if ((abcd >= 8) && efgh == 0b0101) {		// reverse higher speed
				log_msg (LOG_INFO, "%s(%d) MM2 REV FUNC=%c %d\n", __func__, adr, (ui8Func & 0x02) ? 'X' : '-', abcd - 1);
			} else {
				if (((efgh & 0b0111) == 0b0011) || ((abcd == 3) && (efgh == 0b0101)) || ((abcd == 11) && efgh == 0b1010)) {
					log_msg (LOG_INFO, "%s(%d) MM2 F1=%c FUNC=%c %d\n", __func__, adr, (efgh & 8) ? 'X' : '-', (ui8Func & 0x02) ? 'X' : '-', (abcd) ? abcd - 1 : 0);
				} else if (((efgh & 0b0111) == 0b0100) || ((abcd == 4) && (efgh == 0b0101)) || ((abcd == 12) && efgh == 0b1010)) {
					log_msg (LOG_INFO, "%s(%d) MM2 F2=%c FUNC=%c %d\n", __func__, adr, (efgh & 8) ? 'X' : '-', (ui8Func & 0x02) ? 'X' : '-', (abcd) ? abcd - 1 : 0);
				} else if (((efgh & 0b0111) == 0b0110) || ((abcd == 6) && (efgh == 0b0101)) || ((abcd == 14) && efgh == 0b1010)) {
					log_msg (LOG_INFO, "%s(%d) MM2 F3=%c FUNC=%c %d\n", __func__, adr, (efgh & 8) ? 'X' : '-', (ui8Func & 0x02) ? 'X' : '-', (abcd) ? abcd - 1 : 0);
				} else if (((efgh & 0b0111) == 0b0111) || ((abcd == 7) && (efgh == 0b0101)) || ((abcd == 15) && efgh == 0b1010)) {
					log_msg (LOG_INFO, "%s(%d) MM2 F4=%c FUNC=%c %d\n", __func__, adr, (efgh & 8) ? 'X' : '-', (ui8Func & 0x02) ? 'X' : '-', (abcd) ? abcd - 1 : 0);
				} else {
					log_msg (LOG_INFO, "%s(%d) MM2 false coding abcd=%x efgh=%x\n", __func__, adr, abcd, efgh);
				}
			}
		}
	}
#else
	// existing code used a bit reversed coding ... mirroring MSB <-> LSB
	ui8MM_DATA = mm_bytereverse(p->data1 & 0xFF);

	if (p->fast) {			// accessory
		ui8 = 0;
		switch (ui8MM_DATA) {
			case 0xC3:			// 1g
				ui8 = 2;
				DEBUG(ACCMM, "%s(): Address: %03d / 1g\n", __func__, adr);
				break;

			case 0xC0:			// 1r
				ui8 = 3;
				DEBUG(ACCMM, "%s(): Address: %03d / 1r\n", __func__, adr);
				break;

			case 0xCF:			// 2g
				adr++;
				ui8 = 2;
				DEBUG(ACCMM, "%s(): Address: %03d / 2g\n", __func__, adr);
				break;

			case 0xCC:			// 2r
				adr++;
				ui8 = 3;
				DEBUG(ACCMM, "%s(): Address: %03d / 2r\n", __func__, adr);
				break;

			case 0xF3:			// 3g
				adr += 2;
				ui8 = 2;
				DEBUG(ACCMM, "%s(): Address: %03d / 3g\n", __func__, adr);
				break;

			case 0xF0:			// 3r
				adr += 2;
				ui8 = 3;
				DEBUG(ACCMM, "%s(): Address: %03d / 3r\n", __func__, adr);
				break;

			case 0xFF:			// 4g
				adr += 3;
				ui8 = 2;
				DEBUG(ACCMM, "%s(): Address: %03d / 4g\n", __func__, adr);
				break;

			case 0xFC:			// 4r
				adr += 3;
				ui8 = 3;
				DEBUG(ACCMM, "%s(): Address: %03d / 4r\n", __func__, adr);
				break;

			default:
				if ((ui8MM_DATA & 0xC0) == 0) DEBUG(ACCMM, "%s(): Address: %03d / ALL OFF\n", __func__, adr);
				else DEBUG(ACCMM, "%s(): Address: %03d / unknown control code 0x%02x\n", __func__, adr, ui8MM_DATA);
				break;
		}
		if (ui8) trnt_switch(adr, ui8 & 1, 1);
	} else {				// loco
		if ((l = loco_call(adr, true)) == NULL) return false;
		bDirection = (l->speed & 0x80) ? 1 : 0;
		if (LOCOMM) log_msg(LOG_INFO, "%s(): Address: %03d\n", __func__, adr);
		switch (ui8MM_DATA) {	//--- Ausnahmen  im MM-neu-Format rückgängig machen ---
			case 0x27:
				ui8MM_DATA = 0x0F;
				break;
			case 0x32:
				ui8MM_DATA = 0x30;
				break;
			case 0x36:
				ui8MM_DATA = 0x3C;
				break;
			case 0x37:
				ui8MM_DATA = 0x3F;
				break;
			case 0xCD:
				ui8MM_DATA = 0xCF;
				break;
			case 0xD8:
				ui8MM_DATA = 0xF0;
				break;
			case 0xDC:
				ui8MM_DATA = 0xFC;
				break;
			case 0xDD:
				ui8MM_DATA = 0xFF;
				break;
		}
#if 1
		rq_setFuncMasked(adr, (ui8Func & 3) ? FUNC_LIGHT : 0, FUNC_LIGHT);
		if (LOCOMM) log_msg(LOG_INFO, "F0: %d%d ", ui8Func & 1, (ui8Func & 2) >> 1);

		// check additional info for direction or function
		switch (ui8MM_DATA & 0x2A) {
			case 0x22:			// loco backward
				bDirection = 0;
				if (LOCOMM) log_msg(LOG_INFO, "/ direction: backward ");
				break;
			case 0x08:			// loco forward
				bDirection = 1;
				if (LOCOMM) log_msg(LOG_INFO, "/ direction: forward ");
				break;
			case 0x0A:			// F1
				ui8Func = mm_funcHandler(adr, 1, ui8Func, ui8MM_DATA & 0x80);
				break;
			case 0x20:			// F2
				ui8Func = mm_funcHandler(adr, 2, ui8Func, ui8MM_DATA & 0x80);
				break;
			case 0x28:			// F3
				ui8Func = mm_funcHandler(adr, 3, ui8Func, ui8MM_DATA & 0x80);
				break;
			case 0x2A:			// F4
				ui8Func = mm_funcHandler(adr, 4, ui8Func, ui8MM_DATA & 0x80);
				break;
		}
#else
		//--- Richtung auswerten ---
		if (ui8MM_DATA & 2) {
			if (!(ui8MM_DATA & 8)) {
				if (ui8MM_DATA & 0x20) {
					bDirection = 0;
					if (LOCOMM) log_msg(LOG_INFO, " / direction: %s", bDirection ? "forward" : "backward");
				}
			}
		} else {
			if (ui8MM_DATA & 8) {
				if (!(ui8MM_DATA & 0x20)) {
					bDirection = 1;
					if (LOCOMM) log_msg(LOG_INFO, " / direction: %s", bDirection ? "forward" : "backward");
				}
			}
		}

		rq_setFuncMasked(adr, (ui8Func & 3) ? FUNC_LIGHT : 0, FUNC_LIGHT);
		if (LOCOMM) log_msg(LOG_INFO, " F0: %d%d", ui8Func & 1, (ui8Func & 2) >> 1);

		// Funktionen auswerten
		if (ui8MM_DATA & 2) {						// F1, F4 oder nix
			if (ui8MM_DATA & 8) {
				if (ui8MM_DATA & 0x20) {			// ist F4
					if (ui8MM_DATA & 0x80) {
						ui8Func |= 0x20;			// F4 on
						loco_setFunc(adr, 4, true);
						if (LOCOMM) printf(" F4: on");
					} else {
						ui8Func &= ~0x20;			// F4 off
						loco_setFunc(adr, 4, false);
						if (LOCOMM) printf(" F4: off");
					}
				} else {							// ist F1
					if (ui8MM_DATA & 0x80) {
						ui8Func |= 0x04;			// F1 on
						loco_setFunc(adr, 1, true);
						if (LOCOMM) printf(" F1: on");
					} else {
						ui8Func &= ~0x04;			// F1 off
						loco_setFunc(adr, 1, false);
						if (LOCOMM) printf(" F1: off");
					}
				}
			}
		} else {									// F2, F3 oder nix
			if (!(ui8MM_DATA & 8)) {
				if (ui8MM_DATA & 0x20) {			// ist F2
					if (ui8MM_DATA & 0x80) {
						ui8Func |= 0x08;			// F2 on
						loco_setFunc(adr, 2, true);
						if (LOCOMM) printf(" F2: on");
					} else {
						ui8Func &= ~0x08;			// F2 off
						loco_setFunc(adr, 2, false);
						if (LOCOMM) printf(" F2: off");
					}
				}
			} else {
				if (ui8MM_DATA & 0x20) {			//ist F3
					if (ui8MM_DATA & 0x80) {
						ui8Func |= 0x10;			//F3 on
						loco_setFunc(adr, 3, true);
						if (LOCOMM) printf(" F3: on");
					} else {
						ui8Func &= ~0x10;			//F3 off
						loco_setFunc(adr, 3, false);
						if (LOCOMM) printf(" F3: off");
					}
				}
			}
		}
#endif

		// Fahrstufen berechnen
		ui8TempFS = 0;
		if (ui8MM_DATA & 0x01) ui8TempFS++;
		if (ui8MM_DATA & 0x04) ui8TempFS |= 0x02;
		if (ui8MM_DATA & 0x10) ui8TempFS |= 0x04;
		if (ui8MM_DATA & 0x40) ui8TempFS |= 0x08;
		if (ui8TempFS == 1) {	// change direction
			if (LOCOMM) printf(" change direction");
		}
		if (ui8TempFS) ui8TempFS--;	//Fahrstufe korrigieren.
		ui8TempFS = (ui8TempFS * loco_getSpeeds(l->loco))/14;
		if (bDirection) ui8TempFS |= 0x80;
		rq_setSpeed(adr, (int) ui8TempFS);
		if (LOCOMM) printf(" / speed: %d\n", ui8TempFS & 0x7F);
	}
#endif
	return true;
}

/**
 * The base part of the MM decoder.
 * It is (re-)initialised, when called with a zero time (which otherwise is physically impossible).
 *
 * \param t		the time between the last and the current edge (i.e. pulse width) in 1/10us (100ns)
 * \return		true, if a valid packet was decoded
 */
static bool sniffer_mm(uint32_t t)
{
	static struct mm_packet pkt;
	static enum mmedge first;

	enum mmedge edge = MMEDGE_NONE;

	if (t == 0) {
		pkt.ph = PH_MM_SYNC;
		return false;
	}
	if (pkt.ph == PH_MM_FIRSTHALF || pkt.ph == PH_MM_SECONDHALF) {
		if (t > 85 && t < 180)
			edge = MMEDGE_FAST_SHORT;
		else if (t > 185 && t < 360)
			edge = MMEDGE_SLOW_SHORT;
		else if (t > 700 && t < 1200)
			edge = MMEDGE_FAST_LONG;
		else if (t > 1600 && t < 2200)
			edge = MMEDGE_SLOW_LONG;
		else if (first != MMEDGE_NONE && pkt.len == MM_PACKET_BITS - 1) {	// the last bit may coincide with a pause
			if (MM_IS_SHORT(first))
				edge = (pkt.fast) ? MMEDGE_FAST_LONG : MMEDGE_SLOW_LONG;
			else
				edge = (pkt.fast) ? MMEDGE_FAST_SHORT : MMEDGE_SLOW_SHORT;
			switch (edge) {		// correct the remaining timing
				case MMEDGE_FAST_SHORT:
					t -= 130;
					break;
				case MMEDGE_SLOW_SHORT:
					t -= 260;
					break;
				case MMEDGE_FAST_LONG:
					t -= 910;
					break;
				case MMEDGE_SLOW_LONG:
					t -= 1820;
					break;
				default:
					break;
			}
			if (t > MM_PAUSE_MIN)
				xQueueSendToFront(timings, &t, 5);	// insert this remaining time to front of timing queue
		} else {
			pkt.ph = PH_MM_SYNC;
//			log_error ("%s() unexpected edge len %lu\n", __func__, t);
		}
		if (pkt.ph == PH_MM_FIRSTHALF && pkt.len == 0 && first == MMEDGE_NONE) {// decide, wether we deal with fast or slow packet encodings
			pkt.fast = (edge == MMEDGE_FAST_LONG || edge == MMEDGE_FAST_SHORT);
		}
		if (pkt.fast && (edge == MMEDGE_SLOW_LONG || edge == MMEDGE_SLOW_SHORT))
			pkt.ph = PH_MM_SYNC;	// wrong timing
		if (!pkt.fast
		        && (edge == MMEDGE_FAST_LONG || edge == MMEDGE_FAST_SHORT))
			pkt.ph = PH_MM_SYNC;	// wrong timing
	}

	switch (pkt.ph) {
		case PH_MM_SYNC:
			if (t > MM_PAUSE_MIN) {
				pkt.ph = PH_MM_FIRSTHALF;	// this long pause either starts MM fast or slow decoding
				pkt.len = 0;
				pkt.data1 = pkt.data2 = 0;
				first = MMEDGE_NONE;
			}
			break;
		case PH_MM_FIRSTHALF:
		case PH_MM_SECONDHALF:
			if (first == MMEDGE_NONE) {
				first = edge;
			} else {
				if (MM_IS_SHORT(first) == MM_IS_SHORT(edge)) {	// two consecutive short or long edges - illegal (out-of-sync)
					log_error("%s(): two equal edges!\n", __func__);
					pkt.ph = PH_MM_SYNC;
					break;
				}
				if (pkt.ph == PH_MM_FIRSTHALF) {
					pkt.data1 <<= 1;
					if (!MM_IS_SHORT(first))
						pkt.data1 |= 1;
				} else {
					pkt.data2 <<= 1;
					if (!MM_IS_SHORT(first))
						pkt.data2 |= 1;
				}
				pkt.len++;
				first = MMEDGE_NONE;
				if (pkt.len >= MM_PACKET_BITS) {
					if (pkt.ph == PH_MM_FIRSTHALF) {	// continue with inter-packet-gap
						pkt.ph = PH_MM_INTERACKETGAP;
						pkt.len = 0;
					} else {							// packet reception complete - check and interpret packet, restart from sync
						if (mm_interpret(&pkt)) {		// this packet could be decoded, so it is valid
							pkt.ph = PH_MM_SYNC;
							return true;
						} else {						// the two halves didn't match, so maybe we should advance by half a packet
							pkt.data1 = pkt.data2;
							pkt.data2 = 0;
							pkt.len = 0;
							pkt.ph = PH_MM_INTERACKETGAP;
						}
					}
				}
			}
			break;
		case PH_MM_INTERACKETGAP:
			if (t > MM_PAUSE_MIN) {
				pkt.ph = PH_MM_SECONDHALF;
				pkt.len = 0;
			} else {		// no pause between packet halves?
				pkt.ph = PH_MM_SYNC;
			}
			break;
		default:					// emergency break
			pkt.ph = PH_MM_SYNC;
			break;
	}

	return false;
}

/**
 * The base part of the M3 decoder.
 * It is (re-)initialised, when called with a zero time (which otherwise is physically impossible).
 *
 * \param t		the time between the last and the current edge (i.e. pulse width) in 1/10us (100ns)
 * \return		true, if a valid packet was decoded
 */
static bool sniffer_m3(uint32_t t)
{
	static struct m3_packet pkt;
	static uint32_t sr;	// a shift register with '1' for each short edge and '0' for long edges

	if (t == 0) {
		pkt.ph = PH_M3_SYNC;
		sr = M3_SYNCMASK;	// insert six '1' bits to make sure that a m3 sync is only recognized after at least 6 phases
		return false;
	}

	switch (pkt.ph) {
		case PH_M3_SYNC:
			sr <<= 1;
			if (t < DCC_TIME) sr |= 1;
			if ((sr & M3_SYNCMASK) == M3_SYNCPATTERN) {
				pkt.ph = PH_M3_RECEIVE;	// this is a M3 sync pattern - switch to receive
				pkt.len = 0;
			}
			break;
		case PH_M3_RECEIVE:
			pkt.ph = PH_M3_SYNC;	// @todo currently, we don't decode m3 packets
			break;
		default:					// emergency break
			pkt.ph = PH_M3_SYNC;
			break;
	}
	return false;
}

void sniffer(void *pvParameter)
{
	TickType_t lastvalid;
	uint32_t t;
	bool valid, running;

	(void) pvParameter;

	log_msg(LOG_INFO, "%s(): STARTUP\n", __func__);
	init_tim2();
	timings = xQueueCreate(QUEUE_LENGTH, sizeof(uint32_t));
	if (timings == NULL) {
		log_error("%s(): failed to create queue - give up!\n", __func__);
		vTaskDelete(NULL);		// give up / end task / quit
	}
	SET_BIT(TIM2->DIER, TIM_DIER_CC2IE);
	startup = true;
	running =  true;
	lastvalid = 0;

	for (;;) {
		if (xQueueReceive(timings, &t, SIGNAL_MAX_WAIT)) {	// we have got an edge
			if (startup) {	// force decoder initialisation and ignore first timing after starting up
				t = 0;
				startup = false;
			}
			valid = false;
			valid |= sniffer_dcc(t);
			valid |= sniffer_mm(t);
			valid |= sniffer_m3(t);
			if (valid) {
				lastvalid = xTaskGetTickCount();
				if (!running && (rt.tm == TM_STOP || rt.tm == TM_SHORT)) {
					log_msg (LOG_INFO, "%s() valid packet received - GO\n", __func__);
					sig_setMode(TM_GO);
				}
				running = true;
			}
		} else {											// no edge received in SIGNAL_MAX_WAIT timer ticks (ms)
			if ((xTaskGetTickCount() - lastvalid) > 500) {
				if (running && (rt.tm == TM_GO || rt.tm == TM_HALT)) {
					log_msg (LOG_INFO, "%s() no more packets received - STOP\n", __func__);
					sig_setMode(TM_STOP);
				}
				running = false;
				startup = true;
			}
		}
	}
}

// =======================================================================================================================
// === m3 reply handling =================================================================================================
// =======================================================================================================================

#define M3REPLY_TIME			190		///< the nominal timing of the RDS carrier (19µs / 52,63kHz)

static struct m3_reply {
	volatile int			decoderadr;	///< the address of the decoder
	volatile dec_type		dtype;		///< the type of the decoder (currently only mobile decoders are known)
	volatile dec_msgtype	mtype;		///< the message type of the answer (here only the M3 codes are used
	volatile cvadrT			cva;		///< the CV address structure if reading or writing CVs
	volatile flexval		flex;		///< a fexible member (used fot number of data bytes)
	volatile int			carrier;	///< count every detected edge that corresponds to the carrier frequency
	int						bits;		///< number of bits received
	uint8_t					data[16];	///< max. 8 data bytes + CRC byte + possible left-overs at the end (usually only 3 bits)
	bool					start;		///< a signal to the interrupt handler to pre-initialise everything
} reply;

#if M3REPLY_DEBUGRDS
#define DEBUGRDS_ENTRIES	8192
struct m3pll {
	uint32_t			pllpos;
	uint32_t			tim2edge;
	int					offset;
	int					offdelta;
	int					totaldelta;
	int					pllidx;
};
static struct m3pll *m3p;		// base pointer to allocated structure
static struct m3pll *m3int;		// pointer used in interrupt to fill the struct
#endif

static bool m3crc(uint8_t *data, int cnt)
{
	uint16_t crc = 0x00FF;
	int i;

	for (i = 0; i < cnt; i++) {
		crc = crc ^ (crc << 1) ^ (crc << 2);
		crc ^= data[i];
		if (crc & 0x100) crc ^= 0x0107;
		if (crc & 0x200) crc ^= 0x020E;
	}
	return ((crc & 0xFF) == data[cnt]);
}

/**
 * ATTENTION: called from Interrupt context
 */
void m3reply_enable(dec_type dt, int adr, rdbk_type rdt, cvadrT cva, flexval fv)
{
	memset(&reply, 0, sizeof(reply));
	reply.start = true;
	reply.decoderadr = adr;
	reply.dtype = dt;
	reply.mtype = rdt == READBACK_M3DATA ? DECODERMSG_M3DATA : DECODERMSG_M3BIN;
	reply.cva = cva;
	reply.flex = fv;
	SET_BIT(TIM2->DIER, TIM_DIER_CC4IE);	// enable CH4 input capture interrupt
}

/**
 * ATTENTION: called from Interrupt context
 */
void m3reply_disable (struct bitbuffer *bb)
{
	if (READ_BIT(TIM2->DIER, TIM_DIER_CC4IE)) {
		if (reply.mtype == DECODERMSG_M3BIN) {
			reply.data[0] = (reply.carrier > 100) ? 1 : 0;
			reply_callback(bb, DECODERMSG_M3BIN, 1, reply.data);
		} else {
			if (reply.bits >= 16 && m3crc(reply.data, (reply.bits >> 3) - 1)) {
				reply_callback(bb, DECODERMSG_M3DATA, (reply.bits >> 3) - 1, reply.data);
			} else {
				reply_callback(bb, DECODERMSG_ERR, 0, NULL);
			}
		}
		CLEAR_BIT(TIM2->DIER, TIM_DIER_CC4IE);						// disable CH4 input capture interrupt
	}
}

// =======================================================================================================================
enum rdsStatus {
	RDS_SEARCHING = 0,				///< RDS-PLL not locked in any way
	RDS_BITLOCK,					///< RDS-PLL is also locked to bit position - search for bit pattern 010
	RDS_READING,					///< we have found the bit pattern 010 and are now reading data
};

/*
 * A structure holding all status information that is needed to decode the RDS stream
 */
struct rdsdata {
	uint32_t		lastedge;		///< the TIM2 counter position of the previous edge
	uint32_t		raster;			///< the edge position of the synthetic carrier for deviation measurement
	int				idx;			///< the carrier count index inside a bit window
	int				totalidx;		///< the overall carrier count index for debug purposes (to show where some state change occured)
	int				lockcount;		///< number of consecutive found edges for determining if we are locked to carrier frequency
	enum rdsStatus	stat;			///< the overall PLL status
	bool			zero;			///< a marker if an addition carrier phase swap was detected
	int				offsets[16];	///< two sliding comparision windows with 8 entries each
	int				offidx;			///< an index to this offset window
	int				maxoffset;		///< checking for the highest offset-drift
	int				offsetpos;		///< the index count at highest offset-drift
};

void TIM2_IRQHandler(void)
{
	static uint32_t ui32OldTime;
	static struct rdsdata rds;

	uint32_t uiT0_Tick;
	uint32_t edge, distance;
	int offset, diff, i, offold, offnew;
	bool halfcycle, fullcycle;

	// If we are reading m3 answers, we capture the timing of the negative edge of the phase comparator.
	// Capture timing is in 1/10 µs (i.e. 100ns) per tick.
	if ((TIM2->DIER & TIM_DIER_CC4IE) && (TIM2->SR & TIM_SR_CC4IF)) {
		TIM2->SR = ~TIM_SR_CC4IF;
		edge = TIM2->CCR4;

		distance = edge - rds.lastedge;
		if (distance < 140) {// ignore edges that come too fast (these are probably distortions)
			goto skip_edge;
		} else if (distance > 300) {	// we probably missed an edge
			rds.idx++;
			rds.raster += M3REPLY_TIME;
		}
		rds.lastedge = edge;

		if (reply.start) {// initialize the pll structure when starting a new read session
//			irqdbg_printf("%s() START\n", __func__);
			memset(&rds, 0, sizeof(rds));
			rds.stat = RDS_SEARCHING;			// we are always locked :-)
			rds.raster = edge;
			reply.start = false;				// initialisation done
#if M3REPLY_DEBUGRDS
			m3int = m3p;
			m3int->pllpos = rds.raster;
			m3int->tim2edge = edge;
			m3int->offset = 0;
			m3int->offdelta = 0;
			m3int->totaldelta = 0;
			m3int->pllidx = 0;
			m3int++;
			m3int->pllidx = -1;			// a stop code
#endif
			goto skip_edge;
		}

		rds.totalidx++;
		rds.idx++;									// count edges ...
		rds.raster += M3REPLY_TIME;					// ... and advance the "reference" edgepos
		offset = (int) (rds.raster - edge);			// calculate the offset between the edge of the reference clock and the real edge

		if (rds.offidx < 0 || rds.offidx >= DIM(rds.offsets))
			rds.offidx = 0;
		rds.offsets[rds.offidx++] = offset;
		for (i = 0, offold = offnew = 0; i < DIM(rds.offsets); i++) {
			if (i < 8)
				offold += rds.offsets[(rds.offidx + i) % DIM(rds.offsets)];
			else
				offnew += rds.offsets[(rds.offidx + i) % DIM(rds.offsets)];
		}
		diff = (offnew - offold + 4) >> 3;			// diff now contains the average offset of the last 8 edges
		diff = abs(diff);
		if (diff > 50) {
			if (diff > rds.maxoffset) {
				rds.maxoffset = diff;
				rds.offsetpos = rds.idx;
			}
		} else {
			if (diff < 20)
				reply.carrier++;					// count pure carrier edges for binary answers (decoder search)
			if (rds.maxoffset > 50) {				// a carrier swap was detected @ rds.offsetpos
				halfcycle = (rds.offsetpos >= 20 && rds.offsetpos <= 28);
				fullcycle = (rds.offsetpos >= 44 && rds.offsetpos <= 52);
				switch (rds.stat) {
					case RDS_SEARCHING:
						if (fullcycle) rds.lockcount++;
						else rds.lockcount = 0;
						if (rds.lockcount > 8) {
							rds.stat = RDS_BITLOCK;
							reply.data[0] = 0xFF;	// put eight ones to the shift register
							rds.zero = false;
//							irqdbg_printf("RDS bit locked @%d\n", totalidx);
						}
						rds.idx -= rds.offsetpos;
						break;
					case RDS_BITLOCK:
						if (fullcycle) {
							reply.data[0] <<= 1;
							if (!rds.zero) reply.data[0] |= 1;
							rds.zero = false;
							if ((reply.data[0] & 0b111) == 0b010) {
								reply.bits = 0;
								rds.stat = RDS_READING;
//								irqdbg_printf("RDS start pattern @%d\n", totalidx);
							}
							rds.idx -= rds.offsetpos;
						} else if (halfcycle) {
							rds.zero = true;
						}
						break;
					case RDS_READING:
						if (fullcycle) {
							reply.data[reply.bits >> 3] <<= 1;
							if (!rds.zero) reply.data[reply.bits >> 3] |= 1;
							reply.bits++;
							rds.zero = false;
							rds.idx -= rds.offsetpos;
						} else if (halfcycle) {
							rds.zero = true;
						}
						break;
				}
			}
			rds.maxoffset = 0;
		}
	}

skip_edge:

	if ((TIM2->DIER & TIM_DIER_CC2IE) && (TIM2->SR & TIM_SR_CC2IF)) {
		TIM2->SR = ~TIM_SR_CC2IF;
		uiT0_Tick = TIM2->CCR2 - ui32OldTime;
		ui32OldTime = TIM2->CCR2;
		if (timings != NULL) xQueueSendToBackFromISR(timings, &uiT0_Tick, NULL);
	}

	NVIC_ClearPendingIRQ(TIM2_IRQn);
}
