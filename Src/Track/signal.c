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
#include <stdlib.h>
#include "rb2.h"
#include "config.h"
#include "decoder.h"
#include "events.h"
#include "bidib.h"

/**
 * \ingroup Track
 * \{
 */

/*
 * TIM1 controls track signal.
 * Kernel clock will be 200MHz (a tick is 5ns)
 * We will use a prescaler of 200 to get an effective tick rate of 1µs for the timers
 *
 * Currently four compare channels of TIM1 will be used:
 *   - CH1 (PA08): generating signal for controlling the cutout transistor that normally bridge the receiver resistor
 *   - CH2 (PA09): signal that controls the "left" leg of the booster H-bridge
 *   - CH3 (PA10): signal that controls the "right" leg of the booster H-bridge
 *   - CH4 (PA11): the "virgin" track signal for external boosters - does not contain cutouts (sends '1'-bits instead)
 *
 * TIM3 controls the output to the Märklin 5-pin booster connector using CH3 just like TIM1/CH4
 * TIM8 controls the output to the CDE/DCC 3-pin booster connector using CH1 just like TIM1/CH4
 */
//#define FAST_TIMER				///< $$$$ TEST: make timer 10x as fast - 1 tick is 100ns $$$$

static volatile uint16_t TIM3_RCR;		// repeat counter register as a software emulation using TIM3_IRQn with CC3IE

#define MMBOOSTER_CCR			TIM3->CCR3
#define MMBOOSTER_ARR			TIM3->ARR
#define MMBOOSTER_RCR			TIM3_RCR	///< ATTENTION: this is a software emulation! TIM3 has no RCR!
#define DCCBOOSTER_CCR			TIM8->CCR1
#define DCCBOOSTER_ARR			TIM8->ARR
#define DCCBOOSTER_RCR			TIM8->RCR

#define BUFFER_COUNT			32			///< number of available buffers to prepare signals

// definition of DCC signal components
#define COMP_DCC_PREAMBLE		0x0001		///< generate a preamble (should always be done!)
#define COMP_DCC_DATA			0x0002		///< output the data bits
#define COMP_DCC_TAIL1			0x0004		///< generate a single "stop"-bit
#define COMP_DCC_CUTOUT_HD		0x0008		///< begin the cutout / send a normal '1'-bit on the raw channel
#define COMP_DCC_CUTOUT_TAIL	0x0010		///< end the cutout / send a normal '1'-bit on the raw channel
#define COMP_DCC_TAIL4			0x0020		///< generate four more "stop"-bits instead of the cutout
#define COMP_DCC_PACKETEND		0x0040		///< a single additional packet end bit to have one more component to catch the ACK from railcom window#2
#define COMP_DCC_CUTOUT			(COMP_DCC_CUTOUT_HD | COMP_DCC_CUTOUT_TAIL | COMP_DCC_PACKETEND)

// definition of MM signal components
#define COMP_MM_PACKET_GAP		0x0001		///< we start with pause to give the decoders time to prepare for data bits (1,5ms / 4,025ms)
#define COMP_MM_DATA1			0x0002		///< data phase for the first internal repeat
#define COMP_MM_REPEAT_GAP		0x0004		///< a gap between the double packets (1.250µs for MMSLOW or 625µs for MMFAST)
#define COMP_MM_DATA2			0x0008		///< the repetition of the data phase
#define COMP_MM_END_GAP			0x0010		///< we end with pause to give the decoders time to validate last packet (1,5ms / 4,025ms)

// definition of M3 signal components
#define COMP_M3_FLAG1			0x0001		///< generate the starting flag (always present!)
#define COMP_M3_DATA			0x0002		///< the data portion of the packet (always present!)
#define COMP_M3_FLAG2			0x0004		///< 11 or 11,5 sync flags before a reply window
#define COMP_M3_REPLYSTART		0x0008		///< the 4 bits 0b0011 to tell the decoder to switch on its reply hardware
#define COMP_M3_REPLYWIN1		0x0010		///< the first 6ms window for 1-bit replies
#define COMP_M3_FLAG3			0x0020		///< the delimiter flags between two simple reply windows (second window unused)
#define COMP_M3_REPLYWIN2		0x0040		///< the second 6ms window for 1-bit replies (opposite polarity to COMP_M3_REPLYWIN1
#define COMP_M3_REPLYSYNC		0x0080		///< begin of reply window for data with 23 periods of 912µs
#define COMP_M3_REPLYDATA		0x0100		///< reply window for data with sync pulses every 456µs
#define COMP_M3_FLAG4			0x0200		///< end flag terminating the answer together with COMP_M3_ENDFLAG (doubled end flag after reply)
#define COMP_M3_ENDFLAG			0x0400		///< end flag terminating the packet
#define COMP_M3_FILLBIT			0x0800		///< not a real part of the signal, but may be reached, if the signal would end in a high level
/* ... (maybe) more to come ... */

#ifdef FAST_TIMER
#define TIME_MMSLOW_SHORT		280
#define TIME_MMSLOW_LONG		1840
#define TIME_MMSLOW_PERIOD		2080
#define TIME_MMFAST_SHORT		160
#define TIME_MMFAST_LONG		900
#define TIME_MMFAST_PERIOD		1040
#define TIME_MMPAUSE_LONG		40250
#define TIME_MMPAUSE_SHORT		15000
#define TIME_INTERPACKET_SLOW	12500
#define TIME_INTERPACKET_FAST	6250

#define TIME_DCCPERIOD_ONE		1160
#define TIME_DCCPERIOD_ZERO		2000
#define TIME_CUTOUT_START		300

#define TIME_M3PERIOD0			1000	// duration of '0' bits
#define TIME_M3PERIOD1			500		// duration of '1' bits, will be applied two times
#define TIME_M3_RDSMARKER		250		// the marker pulses to synchronise the RDS signal are 25µs
#define TIME_M3_RDS_PERIOD		9120	// this is the bit time for a complete RDS bit, RDS markers are sent twice that often in data phase
#define TIME_M3REPLYWINDOW		60000
#else
// signal timings in µs ===========================================
#if 1		// Timings adjustment for Motorola as in MC1
#define TIME_MMSLOW_SHORT		28
#define TIME_MMSLOW_LONG		184
#define TIME_MMSLOW_PERIOD		208
#define TIME_MMFAST_SHORT		16
#define TIME_MMFAST_LONG		90
#define TIME_MMFAST_PERIOD		104
#define TIME_MMPAUSE_LONG		4025
#define TIME_MMPAUSE_SHORT		1500
#define TIME_INTERPACKET_SLOW	1250
#define TIME_INTERPACKET_FAST	625
#else
#define TIME_MMSLOW_SHORT		26
#define TIME_MMSLOW_LONG		182
#define TIME_MMSLOW_PERIOD		(TIME_MMSLOW_SHORT + TIME_MMSLOW_LONG)
#define TIME_MMFAST_SHORT		13
#define TIME_MMFAST_LONG		91
#define TIME_MMFAST_PERIOD		(TIME_MMFAST_SHORT + TIME_MMFAST_LONG)
#define TIME_MMPAUSE_LONG		4025
#define TIME_MMPAUSE_SHORT		1500
#define TIME_INTERPACKET_SLOW	1250
#define TIME_INTERPACKET_FAST	625
#endif

#define TIME_DCCPERIOD_ONE		116
#define TIME_DCCPERIOD_ZERO		200
#define TIME_CUTOUT_START		30

#define TIME_M3PERIOD0			100		// duration of '0' bits
#define TIME_M3PERIOD1			50		// duration of '1' bits, will be applied two times
#define TIME_M3_RDSMARKER		25		// the marker pulses to synchronise the RDS signal are 25µs
#define TIME_M3_RDS_PERIOD		912		// this is the bit time for a complete RDS bit, RDS markers are sent twice that often in data phase
#define TIME_M3REPLYWINDOW		6000
// ================================================================
#endif

#define f0(p)	(!!(p->funcs[0] & FUNC_LIGHT))		// handy macro to check for LIGHT function bit in packet

static TaskHandle_t SIGtask;					///< the task handle to be signaled by the interrupt handler requesting next packet preparation
static struct bitbuffer buffers[BUFFER_COUNT];	///< a static array of buffers to prepare the signal
static struct bitbuffer idle;					///< a special buffer that is sent to the track, if no valid buffer can be found
static struct bitbuffer reset;					///< a special buffer that is sent to the track in progmode, if no valid buffer can be found
static struct bitbuffer mmidle;					///< a special buffer that is sent to the track in TAMS progmode, if no valid buffer can be found
static struct bitbuffer m3Beacon;				///< a fixed beacon packet for M3
static struct fmtconfig *fmtcfg;				///< a permament pointer to the signal generation settings
static volatile int		signals;				///< a bitmap with the active signal outputs (see BOOSTER_xxx in rb2.h)

static struct {
	struct bitbuffer	* volatile microqueue[4];		///< the bit buffers currently in use. Don't use more than four entries!
	struct bitbuffer	* volatile xpom[4];				///< pointer to the maximum four bitbuffers, that may contain XPOM commands
	struct bitbuffer	* volatile dccA;				///< a DCC-A command sequence may consist of up to three commands (with the last one to be repeated)
} queue;

#define QSLOT_STD0		(queue.microqueue[0])
#define QSLOT_STD1		(queue.microqueue[1])
#define QSLOT_STD2		(queue.microqueue[2])
#define QSLOT_STD3		(queue.microqueue[3])
#define QSLOT_XPOM00	(queue.xpom[0])
#define QSLOT_XPOM01	(queue.xpom[1])
#define QSLOT_XPOM10	(queue.xpom[2])
#define QSLOT_XPOM11	(queue.xpom[3])
#define QSLOT_DCCA		(queue.dccA)

/*
 * ========================================================================================
 * some generic helper routines for signal generation
 * ========================================================================================
 */

/**
 * Generic initialisation of the TIM1 timer peripheral for
 * signal generation. The channels 1 to 4 use PWM mode with
 * preload enable (that is, the CCRx registers are buffered
 * and updated at the update event).
 *
 * PWM mode 1 is starting with a HIGH level and switches to LOW
 * when the counter matches the CCRx register. Setting the CCRx
 * register to 0 generates a constantly LOW output, setting it
 * to the period generates a complete HIGH.
 *
 * PWM mode 2 is the inverted logic, starts LOW and sets HIGH
 * at counter match with its CCRx register. Setting 0 to CCRx
 * generates a constant HIGH and setting it to the period a
 * constant LOW.
 *
 * CH1 controls the transistors for the RC cutout<br>
 * CH2 + CH3 control the power stage driver inputs and form an anti-phase<br>
 * CH4 supplies the logical track signal which is output to the BiDiBus interface
 */
static void sig_init_TIM1 (void)
{
	TIM1->CR1 = 0;	// disable and reset TIM1

	TIM1->CR1 = TIM_CR1_ARPE;		// ARR is buffered
	TIM1->CR2 = 0;					// no settings are used, idle states are LOW
	TIM1->SMCR = 0;					// no settings are used (slave mode is disabled)
	TIM1->DIER = 0;					// start with disabling interrupts
	TIM1->SR = 0;					// clear all status bits
	TIM1->BDTR = TIM_BDTR_OSSI;		// we keep control over the outputs, even if MOE is cleared

	// channel 1 uses PWM mode 2 with preload enabled
	TIM1->CCMR1  = TIM_CCMR1_OC1PE | (0b111 << TIM_CCMR1_OC1M_Pos);
	// channel 2 uses PWM mode 1 with preload enabled
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE | (0b110 << TIM_CCMR1_OC2M_Pos);
	// channel 3 uses PWM mode 2 with preload enabled
	TIM1->CCMR2  = TIM_CCMR2_OC3PE | (0b111 << TIM_CCMR2_OC3M_Pos);
	// channel 4 uses PWM mode 1 with preload enabled
	TIM1->CCMR2 |= TIM_CCMR2_OC4PE | (0b110 << TIM_CCMR2_OC4M_Pos);
	// enable channels 1, 2, 3 and 4
	TIM1->CCER = TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;

#ifdef FAST_TIMER
	TIM1->PSC = 19;					// select a prescaler of 20 (PSC + 1)
#else
	TIM1->PSC = 199;				// select a prescaler of 200 (PSC + 1)
#endif
	TIM1->RCR = 0;					// we don't use the repetition counter
	TIM1->AF1 = 0;					// we don't use any break input (we break for nobody!)
	TIM1->AF2 = 0;					// we don't use any break input (we break for nobody!)

    NVIC_SetPriority(TIM1_UP_IRQn, 6);		// set this as quite high priority
//    NVIC_SetPriority(TIM1_CC_IRQn, 4);		// this one is even higher ($$$$ not used anymore)
	TIM1->SR = 0;							// clear a possibly pending interrupt
}

#ifndef HW_REV07
/**
 * Generic initialisation of the TIM3 timer peripheral for
 * signal generation on 5 pin Märklin booster output. The
 * channel 3 uses PWM mode with preload enable (that is, the
 * CCRx registers are buffered and updated at the update event).
 *
 * \see sig_init_TIM1()
 */
static void sig_init_TIM3 (void)
{
	TIM3->CR1 = 0;	// disable and reset TIM1

	TIM3->CR1 = TIM_CR1_ARPE;		// ARR is buffered
	TIM3->CR2 = 0;					// no settings are used, idle states are LOW
	TIM3->SMCR = 0;					// no settings are used (slave mode is disabled)
	TIM3->DIER = 0;					// start with disabling interrupts
	TIM3->SR = 0;					// clear all status bits

	// channel 3 uses PWM mode 1 with preload enabled
	TIM3->CCMR2  = TIM_CCMR2_OC3PE | (0b110 << TIM_CCMR2_OC3M_Pos);
	// enable channel 3
	TIM3->CCER = TIM_CCER_CC3E;

#ifdef FAST_TIMER
	TIM3->PSC = 19;					// select a prescaler of 20 (PSC + 1)
#else
	TIM3->PSC = 199;				// select a prescaler of 200 (PSC + 1)
#endif
	TIM3->AF1 = 0;					// we don't use any break input (we break for nobody!)
	TIM3->AF2 = 0;					// we don't use any break input (we break for nobody!)

    NVIC_SetPriority(TIM3_IRQn, 5);	// set this as quite high priority
    TIM3->SR = 0;					// clear a possibly pending interrupt
}

/**
 * Generic initialisation of the TIM8 timer peripheral for
 * signal generation on 3 pin CDE booster output. The
 * channel 1 uses PWM mode with preload enable (that is, the
 * CCRx registers are buffered and updated at the update event).
 *
 * \see sig_init_TIM1()
 */
static void sig_init_TIM8 (void)
{
	TIM8->CR1 = 0;	// disable and reset TIM1

	TIM8->CR1 = TIM_CR1_ARPE;		// ARR is buffered
	TIM8->CR2 = 0;					// no settings are used, idle states are LOW
	TIM8->SMCR = 0;					// no settings are used (slave mode is disabled)
	TIM8->DIER = 0;					// start with disabling interrupts
	TIM8->SR = 0;					// clear all status bits
	if (hwinfo->HW >= 0x14) {
		TIM8->BDTR = 0;					// for CDE we relinquish the control over the outputs, they should go to Hi-Z
	} else {
		TIM8->BDTR = TIM_BDTR_OSSI;		// we keep control over the outputs, even if MOE is cleared
	}

	// channel 1 uses PWM mode 1 with preload enabled
	TIM8->CCMR1  = TIM_CCMR1_OC1PE | (0b110 << TIM_CCMR1_OC1M_Pos);
	// enable channel 1
	TIM8->CCER = TIM_CCER_CC1E;

#ifdef FAST_TIMER
	TIM8->PSC = 19;					// select a prescaler of 20 (PSC + 1)
#else
	TIM8->PSC = 199;				// select a prescaler of 200 (PSC + 1)
#endif
	TIM8->RCR = 0;					// we don't use the repetition counter
	TIM8->AF1 = 0;					// we don't use any break input (we break for nobody!)
	TIM8->AF2 = 0;					// we don't use any break input (we break for nobody!)

    TIM8->SR = 0;					// clear a possibly pending interrupt
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 6);		// set this as quite high priority
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);				// always enabled for use of TIM13!
}
#endif	/* !HW_REV07 */

/**
 * Add some data bits (up to 32) to the bitarray. The bitarray is treated
 * as big endian and the bits are added 'to the right'.
 * This function is used, when the underlying format is not byte oriented
 * (i.e. M3).
 *
 * \param ar		the bitarray to add the bits
 * \param bitpos	the current valid length of the bitarray, i.e. the position to add new bits
 * \param data		the up to 32 bits of new data that should be added
 * \param nbits		the number of bits to add to the bit array
 * \return			the new number of bits in the bitarray
 */
static int sig_addbits (uint8_t *ar, int bitpos, uint32_t data, int nbits)
{
	int idx, bidx, b;
	uint8_t mask, val;

	while (nbits > 0) {
		idx = bitpos >> 3;					// the array index (byte wise)
		bidx = bitpos & 7;					// the bit index (inside the byte)
		mask = 0xFF >> bidx;				// the mask of the available bits in this byte
		b = 8 - bidx;						// the number of bits that still fit into the current byte
		if (b > nbits) {					// we have more room than bits to put to array - we must left shift the data
			val = (data << (b - nbits)) & mask;	// extract the last 'nbits' bits from the data
			bitpos += nbits;
		} else {						// we can fill up the current byte completely
			val = (data >> (nbits - b)) & mask;	// extract the next 'b' bits from the data
			bitpos += b;
		}
		ar[idx] &= ~mask;					// clear the bits that may be set to new values
		ar[idx] |= val;

		nbits -= b;
	}

	return bitpos;
}

static sigT sig_getTrackFormat (enum fmt format)
{
	switch (format) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			return SIGNAL_MMSLOW;
		case FMT_MM1_FD:
		case TFMT_MM:
			return SIGNAL_MMFAST;
		case FMT_DCC_14:
		case FMT_DCC_28:
		case FMT_DCC_126:
		case FMT_DCC_SDF:
		case TFMT_DCC:
			return SIGNAL_DCC;
		case FMT_M3_126:
			return SIGNAL_M3;
		default:
			return SIGNAL_UNKNOWN;
	}
}

/*
 * ========================================================================================
 * MM1 + MM2 signal preparation (block level)
 * ========================================================================================
 */
#define MM2_DATAMASK			0b10101010	///< a mask to mask out each second bit of the four data trits in MM2 format
#define MM2_FUNCOFF_ESCAPE		0b01000100	///< the escaped code to switch a function off in MM2 if normal code collides with MM1 speed step
#define MM2_FUNCON_ESCAPE		0b00010001	///< the escaped code to switch a function on in MM2 if normal code collides with MM1 speed step

/**
 * Table for looking up address representation from LOCO-ID
 */
static const uint8_t MM_adrtable[] = {
   0xAA, 0xC0, 0x80, 0x30,	// '10101010', '11000000', '10000000', '00110000'
   0xF0, 0xB0, 0x20, 0xE0,	// '11110000', '10110000', '00100000', '11100000'
   0xA0, 0x0C, 0xCC, 0x8C,	// '10100000', '00001100', '11001100', '10001100'
   0x3C, 0xFC, 0xBC, 0x2C,	// '00111100', '11111100', '10111100', '00101100'
   0xEC, 0xAC, 0x08, 0xC8,	// '11101100', '10101100', '00001000', '11001000'
   0x88, 0x38, 0xF8, 0xB8,	// '10001000', '00111000', '11111000', '10111000'
   0x28, 0xE8, 0xA8, 0x03,	// '00101000', '11101000', '10101000', '00000011'
   0xC3, 0x83, 0x33, 0xF3,	// '11000011', '10000011', '00110011', '11110011'
   0xB3, 0x23, 0xE3, 0xA3,	// '10110011', '00100011', '11100011', '10100011'
   0x0F, 0xCF, 0x8F, 0x3F,	// '00001111', '11001111', '10001111', '00111111'
   0xFF, 0xBF, 0x2F, 0xEF,	// '11111111', '10111111', '00101111', '11101111'
   0xAF, 0x0B, 0xCB, 0x8B,	// '10101111', '00001011', '11001011', '10001011'
   0x3B, 0xFB, 0xBB, 0x2B,	// '00111011', '11111011', '10111011', '00101011'
   0xEB, 0xAB, 0x02, 0xC2,	// '11101011', '10101011', '00000010', '11000010'
   0x82, 0x32, 0xF2, 0xB2,	// '10000010', '00110010', '11110010', '10110010'
   0x22, 0xE2, 0xA2, 0x0E,	// '00100010', '11100010', '10100010', '00001110'
   0xCE, 0x8E, 0x3E, 0xFE,	// '11001110', '10001110', '00111110', '11111110'
   0xBE, 0x2E, 0xEE, 0xAE,	// '10111110', '00101110', '11101110', '10101110'
   0x0A, 0xCA, 0x8A, 0x3A,	// '00001010', '11001010', '10001010', '00111010'
   0xFA, 0xBA, 0x2A, 0xEA,	// '11111010', '10111010', '00101010', '11101010'
   0x00, 0x40, 0x60, 0x97,	// '00000000', '01000000', '01100000', '10010111'
   0x70, 0x48, 0x68, 0x58,	// '01110000', '01001000', '01101000', '01011000'
   0x78, 0x44, 0x64, 0x54,	// '01111000', '01000100', '01100100', '01010100'
   0x74, 0x4C, 0x6C, 0x5C,	// '01110100', '01001100', '01101100', '01011100'
   0x7C, 0x42, 0x62, 0x52,	// '01111100', '01000010', '01100010', '01010010'
   0x72, 0x4A, 0x6A, 0x5A,	// '01110010', '01001010', '01101010', '01011010'
   0x7A, 0x46, 0x66, 0x56,	// '01111010', '01000110', '01100110', '01010110'
   0x76, 0x4E, 0x6E, 0x5E,	// '01110110', '01001110', '01101110', '01011110'
   0x7E, 0x41, 0x61, 0x51,	// '01111110', '01000001', '01100001', '01010001'
   0x71, 0x49, 0x69, 0x59,	// '01110001', '01001001', '01101001', '01011001'
   0x79, 0x45, 0x65, 0x9F,	// '01111001', '01000101', '01100101', '10011111'
   0x75, 0x4D, 0x6D, 0x5D,	// '01110101', '01001101', '01101101', '01011101'
   0x7D, 0x43, 0x63, 0x53,	// '01111101', '01000011', '01100011', '01010011'
   0x73, 0x4B, 0x6B, 0x5B,	// '01110011', '01001011', '01101011', '01011011'
   0x7B, 0x47, 0x67, 0x57,	// '01111011', '01000111', '01100111', '01010111'
   0x77, 0x4F, 0x6F, 0x5F,	// '01110111', '01001111', '01101111', '01011111'
   0x7F, 0x10, 0x18, 0x14,	// '01111111', '00010000', '00011000', '00010100'
   0x1C, 0x12, 0x1A, 0x16,	// '00011100', '00010010', '00011010', '00010110'
   0x1E, 0x11, 0x19, 0x15,	// '00011110', '00010001', '00011001', '00010101'
   0x1D, 0x13, 0x1B, 0x17,	// '00011101', '00010011', '00011011', '00010111'
   0x1F, 0xD0, 0xD8, 0xD4,	// '00011111', '11010000', '11011000', '11010100'
   0xDC, 0xD2, 0xDA, 0xD6,	// '11011100', '11010010', '11011010', '11010110'
   0xDE, 0xD1, 0xD9, 0xD5,	// '11011110', '11010001', '11011001', '11010101'
   0xDD, 0xD3, 0xDB, 0xD7,	// '11011101', '11010011', '11011011', '11010111'
   0xDF, 0x90, 0x98, 0x94,	// '11011111', '10010000', '10011000', '10010100'
   0x9C, 0x92, 0x9A, 0x96,	// '10011100', '10010010', '10011010', '10010110'
   0x9E, 0x91, 0x99, 0x95,	// '10011110', '10010001', '10011001', '10010101'
   0x9D, 0x93, 0x9B, 0x50,	// '10011101', '10010011', '10011011', '01010000'
   0x55, 0x04, 0x06, 0x05,	// '01010101', '00000100', '00000110', '00000101'
   0x07, 0xC4, 0xC6, 0xC5,	// '00000111', '11000100', '11000110', '11000101'
   0xC7, 0x84, 0x86, 0x85,	// '11000111', '10000100', '10000110', '10000101'
   0x87, 0x34, 0x36, 0x35,	// '10000111', '00110100', '00110110', '00110101'
   0x37, 0xF4, 0xF6, 0xF5,	// '00110111', '11110100', '11110110', '11110101'
   0xF7, 0xB4, 0xB6, 0xB5,	// '11110111', '10110100', '10110110', '10110101'
   0xB7, 0x24, 0x26, 0x25,	// '10110111', '00100100', '00100110', '00100101'
   0x27, 0xE4, 0xE6, 0xE5,	// '00100111', '11100100', '11100110', '11100101'
   0xE7, 0xA4, 0xA6, 0xA5,	// '11100111', '10100100', '10100110', '10100101'
   0xA7, 0x01, 0xC1, 0x81,	// '10100111', '00000001', '11000001', '10000001'
   0x31, 0xF1, 0xB1, 0x21,	// '00110001', '11110001', '10110001', '00100001'
   0xE1, 0xA1, 0x0D, 0xCD,	// '11100001', '10100001', '00001101', '11001101'
   0x8D, 0x3D, 0xFD, 0xBD,	// '10001101', '00111101', '11111101', '10111101'
   0x2D, 0xED, 0xAD, 0x09,	// '00101101', '11101101', '10101101', '00001001'
   0xC9, 0x89, 0x39, 0xF9,	// '11001001', '10001001', '00111001', '11111001'
   0xB9, 0x29, 0xE9, 0xA9	// '10111001', '00101001', '11101001', '10101001'
};

/**
 * Coding of the four MM data bits as trits (2 bits / trit), where all trits are
 * either '00' or '11'. The codes cover the really transmitted data portion. If
 * speeds are encoded, speed values with index 0 means halt, index 1 means
 * "Reverse the direction / emergency stop" and index 2 to 15 mean speed 1 to 14.
 * This table can also be used for function decoders controlling F1 to F4.
 *
 * Pay attention to the transmission sequence: the LSB (or least significant trit)
 * is sent first and so is recorded in the two MSBits of each value!
 */
static const uint8_t MM_datacodes[] = {
	0x00, 0xC0, 0x30, 0xF0, 0x0C, 0xCC, 0x3C, 0xFC,
	0x03, 0xC3, 0x33, 0xF3, 0x0F, 0xCF, 0x3F, 0xFF
};

/**
 * Coding of the turnout decoder bits selecting the output to activate.
 *
 * Placement: Trit#5 D0 D1 D2 S (each as double bits / trits)<br>
 * Trit#5 is always cleared for turnouts (a set Trit#5 will address function decoders)
 * D2 .. D0: address of the output to activate with LSB first<br>
 *
 * The sequence seems a little bit strange, but the idea behind it is the
 * following: Märklin defines the first output as the "round/red/thrown"
 * side of the first turnout of that decoder. Our internal schema is
 * just swapped, i.e. subaddress 0 means straight for the first turnout.
 * Consequently, the bytes are swapped at pair level:
 *    1 - 0 - 3 - 2- 5 - 4 - 7 - 6
 */
static const uint8_t MM_turnoutdata[] = {
   0x30, 0x00, 0x3C, 0x0C, 0x33, 0x03, 0x3f, 0x0f
};

/**
 * Look up the trits that form the requested loco address.
 * This is simply a table read.
 *
 * @param loco	the number of the loco to address
 * @see			MM_adrtable
 */
static int sig_mmLookup (int loco)
{
   if (loco > MAX_MM_ADR) loco = 0;
   return MM_adrtable[loco];
}

/**
 * Build a packet buffer for speed packets. For MM2 it will contain a
 * direction information and for MM27b speed coding, the trit #5 will
 * be manipulated. MM27a must be handled in the signal queue processing
 * (see sigq_setSpeed()) but will be processed here with all 27 speed
 * steps resulting in only 14 different speed steps in the final packet.
 *
 * The speeds will be mangled according to the following rules:
 *   - the direction flag (0x80) is stripped off the speed and remenbered separately
 *   - if the speed coding is setup for 27 speed steps and speed is not null:
 *      - increment speed (1 -> 2, 2->3, ... 27 -> 28), now even speeds are native and odd ones are "half speed steps"
 *      - if MM27b is to used, manipulate trit #5 for every odd speed step
 *      - shift right speed by 1 position (division by 2) to come to the native steps 1 .. 14
 *   - if speed is not null, increment it to skip the REVERSE coding of "1" ([1 .. 14] => [2 .. 15])
 *   - look up the basic data code for the 4 trits (8 bits) that will represent the speed step
 *   - if format is MM2 we will now code the direction:
 *      - mask out the second bit of each trit (Scorzoni's page named them bits EFGH)
 *      - add a special set of bits for EFGH, depending on direction and speed area (steps 2..8 are different from 9..15)
 *   - the resulting four trits (8 bits) are packetd to ar[1] and ar[2] respectively
 *
 * \param format	the decoder format
 * \param speed		the speed as 8 bit value including the direction bit 0x80
 * \param f0		the status of the LIGHT function F0 to be coded in trit #5
 * \param ar		the bit array where the packet is coded to
 */
static void sig_mmSpeed (enum fmt format, int speed, bool f0, uint8_t *ar)
{
	bool rev;
	uint8_t s;

	ar[1] = (f0) ? (0b11 << 6) : 0;		// trit #5 is the function bit

	rev = !(speed & 0x80);
	speed &= 0x7F;
	if (FMT_IS_MM27(format) && speed > 0) {		// 27 speed step handling as preprocessing non-stop speeds
		// for both 27 speed methodes increment speed by one (precompensation of shift operation)
		speed++;
		// 27 speed methode B: manipulate trit #5 for odd speed steps
		if (FMT_IS_MM27B(format) && (speed & 1)) ar[1] ^= 1 << 6;
		// for both 27 speed methodes cut down the speed to the range 1..14
		speed >>= 1;
	}
	if (speed) speed++;				// shift speed for both 14 and 27 speed steps to skip emergency stop code

	s = MM_datacodes[speed & 0x0F];

	if (FMT_IS_MM2(format)) {		// speed packet includes direction information
		s &= MM2_DATAMASK;			// mask out each second bit of each trit in the speed information (Scorzoni's bits EFGH)
		if (rev) {						// the reverse speeds -0 / -2 to -15
			if (speed < 8) s |= 0b01000101;		// Scorzoni: EFGH = 1011 (speed is already incremented to skip over emergency stop code)
			else s |= 0b01000100;				// Scorzoni: EFGH = 1010
		} else {						// the forward speeds +0 / +2 to +15
			if (speed < 8) s |= 0b00010001;		// Scorzoni: EFGH = 0101 (speed is already incremented to skip over emergency stop code)
			else s |= 0b00010000;				// Scorzoni: EFGH = 0100
		}
	}
	ar[1] |= s >> 2;				// the trits #6 to #8 are stored in the 6 LSBs of ar[1]
	ar[2] = s << 6;					// the least transmitted trit #9 is put to the two MSBs of the ar[2]
}

/**
 * Construct an emergency stop packet. Emergency stop is always coded
 * as 14-speed-steps with code 1 and for MM2 including the direction.
 *
 * \param format	the loco format - used to distinguish between MM1 and MM2
 * \param rev		specify the direction of travel that should be sent if format is MM2
 * \param f0		the value of the function F0 (to code in trit 5)
 * \param ar		the bit array to fill with the information
 */
static void sig_mmEmergencyStop (enum fmt format, bool rev, bool f0, uint8_t *ar)
{
	uint8_t s;

	ar[1] = (f0) ? 0b11 << 6 : 0;	// trit #5 is the function bit
	s = MM_datacodes[1];			// this is the emergency stop code
	if (FMT_IS_MM2(format)) {		// speed packet includes direction information
		s &= MM2_DATAMASK;			// mask out each second bit of each trit in the speed information (Scorzoni's bits EFGH)
		if (rev) {					// reverse emergency stop
			s |= 0b01000101;		// Scorzoni: EFGH = 1011
		} else {					// forward emergency stop
			s |= 0b00010001;		// Scorzoni: EFGH = 0101
		}
	}

	ar[1] |= s >> 2;				// the trits #6 to #8 are stored in the 6 LSBs of ar[1]
	ar[2] = s << 6;					// the least transmitted trit #9 is put to the two MSBs of the ar[2]
}

/**
 * Build a block for MM2 function switching. The speed will be transmitted
 * without direction information (but maybe with 27 speed steps in case of
 * MM27B).
 *
 * The function and its state is coded in the second bit of each speed trit
 * and then a lot of special handling is applied to avoid all those combinations
 * that could be misinterpretet as MM1 speed information (i.e. the combinations,
 * where the data trits consist of only 0b00 or 0b11).
 *
 * \param format	the loco format - should be one of the MM2-formats
 * \param speed		the speed to send together with the function infomation
 * \param funcs		the function state F0 to F4
 * \param f			the function to code (F1 to F4)
 * \param ar		the bit array to fill with the information
 */
static void sig_mmFunc (enum fmt format, int speed, uint32_t funcs, int f, uint8_t *ar)
{
	uint8_t s;
	bool on, f0;

	f0 = funcs & FUNC_LIGHT;
	ar[1] = (f0) ? (0b11 << 6) : 0;	// trit #5 is the function bit

	speed &= 0x7F;								// the direction will be ignored here
	if (FMT_IS_MM27(format) && speed > 0) {		// 27 speed step handling as preprocessing non-stop speeds
		// for both 27 speed methodes increment speed by one (precompensation of shift operation)
		speed++;
		// 27 speed methode B: manipulate trit 5 for odd speed steps
		if (FMT_IS_MM27B(format) && (speed & 1)) ar[1] ^= 1 << 6;
		// for both 27 speed methodes cut down the speed to the range 1..14
		speed >>= 1;
	}
	if (speed) speed++;				// shift speed for both 14 and 27 speed steps to skip emergency stop code

	s = MM_datacodes[speed & 0x0F] & MM2_DATAMASK;	// mask out each second bit of each trit in the speed information (Scorzoni's bits EFGH)
	on = !!(funcs & (1 << f));
	switch (f) {
		case 1:		// transmit F1
			if (speed == 3 && !on) s |= MM2_FUNCOFF_ESCAPE;
			else if (speed == 11 && on) s |= MM2_FUNCON_ESCAPE;
			else s |= (on) ? 0b01010001 : 0b01010000;
			break;
		case 2:		// transmit F2
			if (speed == 4 && !on) s |= MM2_FUNCOFF_ESCAPE;
			else if (speed == 12 && on) s |= MM2_FUNCON_ESCAPE;
			else s |= (on) ? 0b00000101 : 0b00000100;
			break;
		case 3:		// transmit F3
			if (speed == 6 && !on) s |= MM2_FUNCOFF_ESCAPE;
			else if (speed == 14 && on) s |= MM2_FUNCON_ESCAPE;
			else s |= (on) ? 0b00010101 : 0b00010100;
			break;
		case 4:		// transmit F4
			if (speed == 7 && !on) s |= MM2_FUNCOFF_ESCAPE;
			else if (speed == 15 && on) s |= MM2_FUNCON_ESCAPE;
			else s |= (on) ? 0b01010101 : 0b01010100;
			break;
	}

	ar[1] |= s >> 2;				// the trits #6 to #8 are stored in the 6 LSBs of ar[1]
	ar[2] = s << 6;					// the least transmitted trit #9 is put to the two MSBs of the ar[2]
}

/**
 * Build a block for MM1 function decoders.
 *
 * \param funcs		the function state F1 to F4 in bits 1 to 4 (F0 / LIGHT is ignored)
 * \param ar		the bit array to fill with the information
 */
static void sig_mmFdFunc (uint32_t funcs, uint8_t *ar)
{
	uint8_t f;

	f = MM_datacodes[(funcs >> 1) & 0x0F];	// index table with F1 - F4
	ar[1] = 0b11 << 6;		// trit #5 distinguishes between turnout and function decoders
	ar[1] |= f >> 2;		// the trits #6 to #8 are stored in the 6 LSBs of ar[1]
	ar[2] = f << 6;			// the least transmitted trit #9 is put to the two MSBs of the ar[2]
}

/**
 * Build a block for MM turnouts.
 * The address is already converted to a magnet address in the range of 0..2047.
 * Decoder addresses range from 0..255, turnouts on the decoder are 2 bits and
 * the direction (thrown or straight) is one LSB.
 *
 * The trit #5 is always cleared (0b00). If the magnets will be switched off, the
 * two LSBs from the turnout data table must be masked away.
 */
static void sig_mmTurnout (int adr, uint8_t *ar, bool on)
{
	ar[1] = MM_turnoutdata[adr & 0x07];			// 6 LSBs are the trits #6 to #8 (trit #5 is 0b00 for turnouts - see table)
	if (on) ar[2] = 0xC0;						// 2 MSBs (representing the last trit #9) are set
	else ar[2] = 0;								// 2 MSBs (representing the last trit #9) are cleared
}

/**
 * Render a packet for the Märklin Motorola format (MM1 / MM2, FAST / SLOW with speed
 * coding 14, 27A or 27B).
 *
 * \param p		the high level packet that is to be rendered to the bitbuffer
 * \param bb	the bitbuffer which should be filled up with the rendered bit contents
 * \return		the bitbuffer as provided by the bb parameter or NULL, if not able to translate the request
 * \see			sig_renderBuffer()
 */
static struct bitbuffer *sig_renderMM (struct packet *p, struct bitbuffer *bb)
{
	int adr;

	bb->bits = 18;		// always fixed!
	bb->components = COMP_MM_PACKET_GAP | COMP_MM_DATA1 | COMP_MM_REPEAT_GAP | COMP_MM_DATA2 | COMP_MM_END_GAP;

	bb->databits[0] = sig_mmLookup(p->adr);	// all decoders use the same address encoding
	bb->dt = DECODER_MM_MOBILE;				// most commands

	switch (p->cmd) {
		case QCMD_SETSPEED:
		case QCMD_MM_SETSPEED_27A:
			sig_mmSpeed(p->fmt, p->value.i32, p->funcs[0] & FUNC_LIGHT, bb->databits);
			break;
		case QCMD_MM_REVERSE:
		case QCMD_EMERGENYSTOP:
			sig_mmEmergencyStop(p->fmt, p->value.i32 & 0x80, p->funcs[0] & 1, bb->databits);
			break;
		case QCMD_MM_SETF1:
			sig_mmFunc(p->fmt, p->value.i32, p->funcs[0], 1, bb->databits);
			break;
		case QCMD_MM_SETF2:
			sig_mmFunc(p->fmt, p->value.i32, p->funcs[0], 2, bb->databits);
			break;
		case QCMD_MM_SETF3:
			sig_mmFunc(p->fmt, p->value.i32, p->funcs[0], 3, bb->databits);
			break;
		case QCMD_MM_SETF4:
			sig_mmFunc(p->fmt, p->value.i32, p->funcs[0], 4, bb->databits);
			break;
		case QCMD_MM_FDFUNCS:
			sig_mmFdFunc(p->funcs[0], bb->databits);
			bb->dt = DECODER_MM_FUNC;
			break;
		case QCMD_MAGNET_ON:
		case QCMD_MAGNET_OFF:
			adr = (p->adr - 1) << 1;		// map turnout addresses 1..1020 to the range 0..1019 and then to a magnet address
			if (p->param.i32) adr |= 1;		// if the param is set, the turnout should be thrown
			bb->databits[0] = sig_mmLookup((adr >> 3) + 1);		// mapped turnout 0 is switched by decoder 1
			sig_mmTurnout (adr, bb->databits, (p->cmd == QCMD_MAGNET_ON));
			bb->dt = DECODER_MM_ACC;
			break;
		default:
			return NULL;
	}

	return bb;
}

/*
 * ========================================================================================
 * DCC signal preparation (block level)
 * ========================================================================================
 */

/*
 * DCC instruction byte prefixes (most significant 3 or 4 bits of the first command byte)
 */
#define DCC_CONTROL				0b00000000					///< decoder control
#define DCC_CONSIST				0b00010000					///< consist control

#define DCC_ADVANCED			0b00100000					///< advanced operations instructions
#define DCC_ADVANCED_SPEED		(DCC_ADVANCED | 0b00011111)	///< set speed (126 speed steps)
#define DCC_ADVANCED_RESTICT	(DCC_ADVANCED | 0b00011110)	///< restrict speed

#define DCC_SPEED_REV			0b01000000					///< reverse direction speed command
#define DCC_SPEED_FWD			0b01100000					///< forward direction speed command
#define DCC_FUNC_ONE			0b10000000					///< function group one instruction
#define DCC_FUNC_TWO			0b10100000					///< function group two instruction
#define DCC_FUTURE_EXP			0b11000000					///< future expansion
#define DCC_STATECTL_L			(DCC_FUTURE_EXP | 0b00000)	///< Binary State Control - long format
#define DCC_STATECTL_S			(DCC_FUTURE_EXP | 0b11101)	///< Binary State Control - short format
#define DCC_EXT_FUNCS			(DCC_FUTURE_EXP | 0b11000)	///< F13 to F68 are coded with these prefixes
#define DCC_F13_F20				(DCC_EXT_FUNCS | 0b110)		///< Functions F13 - F20
#define DCC_F21_F28				(DCC_EXT_FUNCS | 0b111)		///< Functions F21 - F28
#define DCC_F29_F36				(DCC_EXT_FUNCS | 0b000)		///< Functions F29 - F36
#define DCC_F37_F44				(DCC_EXT_FUNCS | 0b001)		///< Functions F37 - F44
#define DCC_F45_F52				(DCC_EXT_FUNCS | 0b010)		///< Functions F45 - F52
#define DCC_F53_F60				(DCC_EXT_FUNCS | 0b011)		///< Functions F53 - F60
#define DCC_F61_F68				(DCC_EXT_FUNCS | 0b100)		///< Functions F61 - F68
#define DCC_PT_LONG				0b01110000					///< access to CVs (PT, long format)
#define DCC_POM_LONG			0b11100000					///< access to CVs (POM, long format)
#define DCC_POM_SHORT			0b11110000					///< access to CVs (POM only, short format)
#define DCC_EXT_ACC				0b0							///< Extended Accessory Decoder Packet

/**
 * Calculate the XOR-Checksum of the data bits and append it to the array
 *
 * \param ar		the bitarray to build up the coding
 * \param pos		the bits position to add the checksum (i.e. current length)
 * \return			the new number of bits in the bitarray
 */
static int sig_dccChecksum (uint8_t *ar, int pos)
{
	int idx;
	uint8_t chksum = 0;

	for (idx = 0; idx < (pos >> 3); idx++) {
		chksum ^= ar[idx];
	}
	ar[idx] = chksum;
	return pos + 8;
}

/**
 * Write a standard decoder address to the first or the first + second byte of the packet.
 * For DCC format, this will always be put at byte 0 of the bitarray.
 *
 * \param adr		the decoder address to code into the packet
 * \param ar		the bitarray to build up the coding
 * \return			the new number of bits in the bitarray
 */
static int sig_dccAddress(int adr, uint8_t *ar)
{
	if (adr <= 127 && !(fmtcfg->sigflags & SIGFLAG_DCC_LONG_ADR)) {		// short addresses are encoded as 7 bits in the first byte of the packet
		ar[0] = adr & 0x7F;						// just put the 7 address bits to the first byte - the stating 0-bit will indicate a short address
		return 8;
	}
	ar[0] = 0b11000000 | ((adr >> 8) & 0x3F);	// a starting 0b11 + the six address MSBs will signal a long decoder address
	ar[1] = adr & 0xFF;							// the 8 LSBs of the address make up the second byte
	return 16;
}

/**
 * Write a standard accessory decoder address to the first two bytes of the packet.
 *
 * The address is a turnout address (controlling a pair of outputs) in the range
 * of 1 .. 2043 inclusive and will be mapped to the on-track address 4 .. 2046.
 * The addresses 2044 - 2047 will be mapped to on-track addresses 0 .. 3. An
 * emergency stop code is defined with on-track address 2047 and can be set by
 * calling this function with address 0.
 * This basic format will later be enriched with a direction and an on/off bit
 * without adding additional data bytes.
 *
 * \param adr		the decoder address to code into the packet
 * \param ar		the bitarray to build up the coding
 * \return			the new number of bits in the bitarray
 */
static int sig_accAddress(int adr, uint8_t *ar)
{
	if (adr == 0) {
		adr = 2047;
	} else {
		adr += 3;									// map turnout addresses 1..2043 to the range 4..2046
		if ((adr >= 2047) && (adr <= 2050)) adr -= 2047;	// re-map addresses 2044..2047 to the range 0..3
	}

	ar[0] = (0b10 << 6) | ((adr >> 2) & 0x3F);	// the decoder address bits A7-A2 + 0b10... as MSBs
	ar[1] = ((~adr >> 4) & 0x70);				// the decoder address bits A10-A8 are inverted
	ar[1] |= 0x80 | ((adr << 1) & 0x06);		// the decoder address bits A1-A0 and a permanently set bit 7
	return 16;
}

/**
 * Write an extended accessory decoder address to the first two bytes of the packet.
 *
 * The address is an extended accessory decoder address (controlling an eight bit value)
 * in the range of 1 .. 2043 inclusive and will be mapped to the on-track address 4 .. 2046.
 * The addresses 2044 - 2047 will be mapped to on-track addresses 0 .. 3. An
 * emergency stop code is defined with on-track address 2047 with a data byte of 0.
 *
 * \param adr		the decoder address to code into the packet
 * \param ar		the bitarray to build up the coding
 * \return			the new number of bits in the bitarray
 */
static int sig_extAddress(int adr, uint8_t *ar)
{
	if (adr == 0) {
		adr = 2047;
	} else {
		adr += 3;									// map turnout addresses 1..2043 to the range 4..2046
		if ((adr >= 2047) && (adr <= 2050)) adr -= 2047;	// re-map addresses 2044..2047 to the range 0..3
	}

	ar[0] = 0b10000000 | ((adr >> 2) & 0x3F);	// the decoder address bits A7-A2 + 0b10... as MSBs
	ar[1] = ((~adr >> 4) & 0x70);				// the decoder address bits A10-A8 are inverted
	ar[1] |= ((adr << 1) & 0x06) | 0x01;		// the decoder address bits A1-A0 and a permanently set bit 0
	return 16;
}

/**
 * Helper function to skip writing an address to the array and initialise
 * the bb->bits member to zero when no real address is needed (system commands
 * and programming track manipulation).
 *
 * \param adr		the decoder address to code into the packet
 * \param ar		the bitarray to build up the coding
 * \return			the new number of bits in the bitarray
 */
static int sig_nullAddress(int adr, uint8_t *ar)
{
	(void) adr;
	(void) ar;

	return 0;
}

/**
 * Helper function to write the broadcast address to the array and initialise
 * the bb->bits member to 8.
 *
 * \param adr		the decoder address to code into the packet
 * \param ar		the bitarray to build up the coding
 * \return			the new number of bits in the bitarray
 */
static int sig_dccBroadcast(int adr, uint8_t *ar)
{
	(void) adr;
	ar[0] = 0;

	return 8;
}

/**
 * Put the speed coding to the bitbuffer. For 14 speed step decoders, we also must put
 * the state of the F0 function to the packet.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccSpeed (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;
	int speed;

	speed = p->value.i32;
	switch (p->fmt) {
		case FMT_DCC_14:
			if (speed & 0x7F) speed += 1;		// speed = 1 is the emergency stop code
			ar[idx] = ((speed & 0x80) ? DCC_SPEED_FWD : DCC_SPEED_REV) | (speed & 0x0F);
			if (f0(p)) ar[idx] |= 0x10;
			return bb->bits + 8;
		case FMT_DCC_28:
			if (speed & 0x7F) speed += 3;		// emergency stop codes: LSB is ignored, from the remaining 4 bits, 0 is halt and 1 is emergency stop
			ar[idx] = ((speed & 0x80) ? DCC_SPEED_FWD : DCC_SPEED_REV) | ((speed >> 1) & 0x0F);
			if (speed & 0x01) ar[idx] |= 0x10;
			return bb->bits + 8;
		case FMT_DCC_126:
		case FMT_DCC_SDF:
			if (speed & 0x7F) speed += 1;          // speed = 1 is the emergency stop code
			ar[idx] = DCC_ADVANCED_SPEED;
			ar[idx + 1] = speed;
			return bb->bits + 16;
		default:
			return bb->bits;
	}
}

/**
 * Put the emergency coding to the bitbuffer. For 14 speed step decoders, we also must put
 * the state of the F0 function to the packet. Additionally, we will keep the direction.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccEmergencyStop (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	switch (p->fmt) {
		case FMT_DCC_14:
		case FMT_DCC_28:
			ar[idx] = ((p->value.i32 & 0x80) ? DCC_SPEED_FWD : DCC_SPEED_REV) | 0x01;
			if (p->fmt == FMT_DCC_14 && (f0(p))) ar[idx] |= 0x10;
			return bb->bits + 8;
		case FMT_DCC_126:
		case FMT_DCC_SDF:
			ar[idx] = DCC_ADVANCED_SPEED;
			ar[idx + 1] = (p->value.i32 & 0x80) | 0x01;
			return bb->bits + 16;
		default:
			return 0;
	}
}

/**
 * Send a function packet to the decoder. The format is requiered to know if F0
 * should be added to a packet using QCMD_DCC_SETF1_4, which should only be done
 * for decoders having more than 14 speeds. For 14 speed decodes, the F0 is transmitted
 * as part of the speed command.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccFunctions (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	switch (p->cmd) {
		case QCMD_DCC_SETF1_4:
			ar[idx] = DCC_FUNC_ONE | ((p->funcs[0] >> 1) & 0x0F);
			// code F0 to bit 5 if more than 14 speeds are supported
			if (p->fmt != FMT_DCC_14 && (f0(p))) ar[idx] |= 0x10;
			return bb->bits + 8;
		case QCMD_DCC_SETF5_8:
			ar[idx] = DCC_FUNC_TWO | ((p->funcs[0] >> 5) & 0x0F) | 0x10;	// adding bit 5 (0x10): we mean F5 - F8
			return bb->bits + 8;
		case QCMD_DCC_SETF9_12:
			ar[idx] = DCC_FUNC_TWO | ((p->funcs[0] >> 9) & 0x0F);			// not adding bit 5: we mean F9 - F12
			return bb->bits + 8;
		case QCMD_DCC_SETF13_20:
			ar[idx] = DCC_F13_F20;
			ar[idx + 1] = (p->funcs[0] >> 13) & 0xFF;
			return bb->bits + 16;
		case QCMD_DCC_SETF21_28:
			ar[idx] = DCC_F21_F28;
			ar[idx + 1] = (p->funcs[0] >> 21) & 0xFF;
			return bb->bits + 16;
		case QCMD_DCC_SETF29_36:
			ar[idx] = DCC_F29_F36;
			ar[idx + 1] = ((p->funcs[0] >> 29) | (p->funcs[1] << 3)) & 0xFF;
			return bb->bits + 16;
		case QCMD_DCC_SETF37_44:
			ar[idx] = DCC_F37_F44;
			ar[idx + 1] = (p->funcs[1] >> 5) & 0xFF;
			return bb->bits + 16;
		case QCMD_DCC_SETF45_52:
			ar[idx] = DCC_F45_F52;
			ar[idx + 1] = (p->funcs[1] >> 13) & 0xFF;
			return bb->bits + 16;
		case QCMD_DCC_SETF53_60:
			ar[idx] = DCC_F53_F60;
			ar[idx + 1] = (p->funcs[1] >> 21) & 0xFF;
			return bb->bits + 16;
		case QCMD_DCC_SETF61_68:
			ar[idx] = DCC_F61_F68;
			ar[idx + 1] = ((p->funcs[1] >> 29) | (p->funcs[2] << 3)) & 0xFF;
			return bb->bits + 16;
		default:
			return bb->bits;
	}
}

/**
 * Send a binary state packet to the decoder.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccBinaryState (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;
	int state = p->param.i32;

	ar[idx + 1] = state & 0x7F;				// seven LSBs of state number
	if (p->value.i32) ar[idx + 1] |= 0x80;	// the state (ON or OFF)
	if (state > 127) {
		ar[idx] = 0b11000000;				// binary state long form
		ar[idx + 2] = (state >> 7) & 0xFF;	// eight MSBs of the state number
		idx += 3;							// using 3 bytes
	} else {
		ar[idx] = 0b11011101;				// binary state short form
		idx += 2;							// using only 2 bytes
	}
	return idx << 3;
}

/**
 * Send a SDF packet to the decoder. SDF stands for Speed, Direction and Function
 * and means a combination packet of these three informations in a single packet.
 * This type of packet seems to be senseful only as a refresh packet.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccsdf (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	ar[idx++] = 0b00111100;		// SDF code
	ar[idx++] = p->value.i32 & 0xFF;	// speed
	ar[idx++] = p->funcs[0] & 0xFF;		// F0 - F7
	// p->param.i32 holds the loco's maxfunc information
	if (p->param.i32 >= 8) ar[idx++] = (p->funcs[0] >> 8) & 0xFF;	// F8 - F15
	if (p->param.i32 >= 16) ar[idx++] = (p->funcs[0] >> 16) & 0xFF;	// F16 - F23
	if (p->param.i32 >= 24) ar[idx++] = (p->funcs[0] >> 24) & 0xFF;	// F24 - F31

	return idx << 3;
}

/**
 * Enrich the two byte packet with the information for the output to
 * address (straight or thrown) and the activation status (on or off).
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_accSwitch (struct packet *p, struct bitbuffer *bb)
{
	volatile uint8_t *ar = bb->databits;

	if (!p->param.i32) ar[1] |= 1;		// if the param is set, the turnout should be thrown (RCN-213: R = 0), else it is set (R = 1)
	if (p->cmd == QCMD_MAGNET_ON) ar[1] |= 0x08;	// the D-bit is set for the ON command and stays at zero for the OFF command
	return 16;							// these packet stay at 16 bits
}

/**
 * Send the aspect information to an extended accessory decoder
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_extAspect (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	ar[idx] = p->value.i32 & 0xFF;
	return bb->bits + 8;
}

/**
 * Programming on PT or Main: read a byte via Railcom
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccReadByte (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	if (bb->bits > 0) {		// POM mode
		ar[idx] = DCC_POM_LONG | (0b01 << 2) | ((p->cva.cv >> 8) & 0x03);	// KK = 0b01 - check/read byte, 2 MSBs of the CV address
		if ((ar[0] & 0xC0) == 0x80 && (ar[1] & 0x80)) {	// special for basic accessory decoders with POM
			ar[1] |= 0x08;		// bit 3 in second byte was a flag bit "C" and must now always be set
		}
		bb->rdt = READBACK_POM;
		bb->cva = p->cva;
	} else {			// PT mode
		bb->rdt = READBACK_DCC_PT;
		bb->dcc.preamble = 20;
		ar[idx] = DCC_PT_LONG | (0b01 << 2) | ((p->cva.cv >> 8) & 0x03);	// KK = 0b01 - check/read byte, 2 MSBs of the CV address
	}
	ar[idx + 1] = p->cva.cv & 0xFF;
	ar[idx + 2] = p->value.i32 & 0xFF;			// the comparision byte - can be ignored in POM mode as we are interested in railcom answers
	return bb->bits + 24;
}

/**
 * Programming on PT or Main: read/verify a bit (indeed, only used for PT bit verification)
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccReadBit (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	if (bb->bits > 0) {		// POM mode
		ar[idx] = DCC_POM_LONG | (0b10 << 2) | ((p->cva.cv >> 8) & 0x03);	// KK = 0b10 - bit manipulation, 2 MSBs of the CV address
		if ((ar[0] & 0xC0) == 0x80 && (ar[1] & 0x80)) {	// special for basic accessory decoders with POM
			ar[1] |= 0x08;		// bit 3 in second byte was a flag bit "C" and must now always be set
			bb->rdt = READBACK_POM;
			bb->param = p->value;
			bb->cva = p->cva;
		}
	} else {			// PT mode
		bb->rdt = READBACK_DCC_PT;
		bb->dcc.preamble = 20;
		ar[idx] = DCC_PT_LONG | (0b10 << 2) | ((p->cva.cv >> 8) & 0x03);	// KK = 0b10 - bit manipulation, 2 MSBs of the CV address
	}
	ar[idx + 1] = p->cva.cv & 0xFF;
	ar[idx + 2] = 0b11100000 | (p->value.bitval ? 0x08 : 0x00) | (p->value.bitpos & 0x07);
	return bb->bits + 24;
}

/**
 * Programming on PT or Main: write a byte
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccWriteByte (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	if (bb->bits > 0) {		// POM mode
		ar[idx] = DCC_POM_LONG | (0b11 << 2) | ((p->cva.cv >> 8) & 0x03);	// KK = 0b11 - write byte, 2 MSBs of the CV address
		if ((ar[0] & 0xC0) == 0x80 && (ar[1] & 0x80)) {						// special for basic accessory decoders with POM
			ar[1] |= 0x08;									// bit 3 in second byte was a flag bit "C" and must now always be set
		}
		bb->rdt = READBACK_POMWRITE;
		bb->dcc.targetval = p->value.i32 & 0xFF;	// this is what we expect from the acknowledgment of the railcom message
		bb->cva = p->cva;
	} else {			// PT mode
		bb->rdt = READBACK_DCC_PT;
		bb->dcc.preamble = 20;
		ar[idx] = DCC_PT_LONG | (0b11 << 2) | ((p->cva.cv >> 8) & 0x03);	// KK = 0b11 - write byte, 2 MSBs of the CV address
	}
	ar[idx + 1] = p->cva.cv & 0xFF;
	ar[idx + 2] = p->value.i32 & 0xFF;
	return bb->bits + 24;
}

/**
 * Programming on PT or Main: write a bit
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccWriteBit (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	if (bb->bits > 0) {		// POM mode
		ar[idx] = DCC_POM_LONG | (0b10 << 2) | ((p->cva.cv >> 8) & 0x03);	// KK = 0b10 - bit manipulation, 2 MSBs of the CV address
		if ((ar[0] & 0xC0) == 0x80 && (ar[1] & 0x80)) {	// special for basic accessory decoders with POM
			ar[1] |= 0x08;		// bit 3 in second byte was a flag bit "C" and must now always be set
		}
		bb->rdt = READBACK_POM;
		bb->cva = p->cva;
	} else {			// PT mode
		bb->rdt = READBACK_DCC_PT;
		bb->dcc.preamble = 20;
		ar[idx] = DCC_PT_LONG | (0b10 << 2) | ((p->cva.cv >> 8) & 0x03);	// KK = 0b10 - bit manipulation, 2 MSBs of the CV address
	}
	ar[idx + 1] = p->cva.cv & 0xFF;
	ar[idx + 2] = 0b11110000 | (p->value.bitval ? 0x08 : 0x00) | (p->value.bitpos);
	return bb->bits + 24;
}

/**
 * Programming on Main: short form write one or two bytes to a mobile decoder.
 * There is a short list of CVs or CV combinations defined by RailCommunity.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccShortPomWrite (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	ar[idx] = 0b11110000 | (p->param.i32 & 0x0F);		// one of sixteen possible values, most are reserved
	ar[idx + 1] = p->value.ui8[0];						// this byte is always valid
	ar[idx + 2] = p->value.ui8[1];						// this byte is only valid, if the command was QCMD_DCC_XPOM_WR2
	bb->rdt = READBACK_POM;
	return bb->bits + ((p->cmd == QCMD_DCC_XWR2) ? 24 : 16);
}

/**
 * Programming on Main: extended 24-bit CV addressing with up to 4 bytes
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccXPom (struct packet *p, struct bitbuffer *bb)
{
	static uint8_t seq;

	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;
	int bcnt;

	ar[idx] = 0b11100000 | (seq & 0x03);
	switch (seq & 0x03) {
		case 0: bb->fmt = SIGNAL_DCC_XPOM00; break;
		case 1: bb->fmt = SIGNAL_DCC_XPOM01; break;
		case 2: bb->fmt = SIGNAL_DCC_XPOM10; break;
		case 3: bb->fmt = SIGNAL_DCC_XPOM11; break;
	}
	seq++;

	switch (p->cmd) {
		case QCMD_DCC_XPOM_RD_BLK:
		case QCMD_DCC_XPOM_ACC_RD_BLK:
		case QCMD_DCC_XPOM_EXT_RD_BLK:
			ar[idx] |= 0b01 << 2;
			bcnt = 4;
			break;
		case QCMD_DCC_XPOM_WR_BYTE1:
		case QCMD_DCC_XPOM_ACC_WR_BYTE1:
		case QCMD_DCC_XPOM_EXT_WR_BYTE1:
			ar[idx] |= 0b11 << 2;
			bcnt = 5;
			break;
		case QCMD_DCC_XPOM_WR_BYTE2:
		case QCMD_DCC_XPOM_ACC_WR_BYTE2:
		case QCMD_DCC_XPOM_EXT_WR_BYTE2:
			ar[idx] |= 0b11 << 2;
			bcnt = 6;
			break;
		case QCMD_DCC_XPOM_WR_BYTE3:
		case QCMD_DCC_XPOM_ACC_WR_BYTE3:
		case QCMD_DCC_XPOM_EXT_WR_BYTE3:
			ar[idx] |= 0b11 << 2;
			bcnt = 7;
			break;
		case QCMD_DCC_XPOM_WR_BYTE4:
		case QCMD_DCC_XPOM_ACC_WR_BYTE4:
		case QCMD_DCC_XPOM_EXT_WR_BYTE4:
			ar[idx] |= 0b11 << 2;
			bcnt = 8;
			break;
		case QCMD_DCC_XPOM_WR_BIT:
		case QCMD_DCC_XPOM_ACC_WR_BIT:
		case QCMD_DCC_XPOM_EXT_WR_BIT:
			ar[idx] |= 0b10 << 2;
			bcnt = 5;
			break;
		default:
			return 0;		// illegal request
	}
	ar[idx + 1] = (p->cva.cv >> 16) & 0xFF;		// 24 bit CV address in big endian coding
	ar[idx + 2] = (p->cva.cv >>  8) & 0xFF;
	ar[idx + 3] = (p->cva.cv >>  0) & 0xFF;
	ar[idx + 4] = (p->value.ui8[0]);				// just copy all four possible bytes
	ar[idx + 5] = (p->value.ui8[1]);
	ar[idx + 6] = (p->value.ui8[2]);
	ar[idx + 7] = (p->value.ui8[3]);
	bb->rdt = READBACK_XPOM;

	log_msg (LOG_INFO, "%s() bcnt = %d ar:\t\t", __func__, bcnt);
	for (int i = 0; i < idx + 8; i++) {
		printf (" 0x%02x", ar[i]);
	}
	printf ("\n");


	return bb->bits + (bcnt * 8);
}

/**
 * Generate a DCC idle packet
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccIdle (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	(void) p;

	ar[idx] = 0xFF;					// SYSTEM command address
	ar[idx + 1] = 0x00;
	return bb->bits + 16;
}

/**
 * Generate a DCC reset packet
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccReset (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	(void) p;

	ar[idx] = 0x00;
	ar[idx + 1] = 0x00;
	bb->dcc.preamble = 20;			// use at least 20 preamble bits
	bb->dcc.tail = 6;				// use 6 tails bits including the cutout
	bb->components &= ~COMP_DCC_CUTOUT;
	bb->components |= COMP_DCC_TAIL4;
	return 16;
}

/**
 * Generate a DCC model time packet
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccModelTime (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	ar[idx] = 0b11000001;		// command for model time
	ar[idx + 1] = (p->value.u32 >> 16) & 0xFF;		// model time is already packed and will be transmitted as big endian value
	ar[idx + 2] = (p->value.u32 >>  8) & 0xFF;
	ar[idx + 3] = (p->value.u32 >>  0) & 0xFF;
	bb->components &= ~COMP_DCC_CUTOUT;
	bb->components |= COMP_DCC_TAIL4;

	return bb->bits + 32;
}

/**
 * Generate a DCC system time packet
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccSysTime (struct packet *p, struct bitbuffer *bb)
{
	int idx = bb->bits >> 3;		// pos should always point to a byte boundary
	volatile uint8_t *ar = bb->databits;

	ar[idx] = 0b11000010;		// command for system time
	ar[idx + 1] = (p->value.u32 >> 8) & 0xFF;		// system time in milliseconds as 16 bit big endian value
	ar[idx + 2] = (p->value.u32 >> 0) & 0xFF;
	bb->components &= ~COMP_DCC_CUTOUT;
	bb->components |= COMP_DCC_TAIL4;

	return bb->bits + 24;
}

/**
 * Write a NOP to an accessory decoder to give it (or all in case of Broadcast)
 * the opportunity to send a railcom answer.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_accNOP (struct packet *p, struct bitbuffer *bb)
{
	volatile uint8_t *ar = bb->databits;

	(void) p;

	ar[1] &= ~0x81;		// clear bits 0 and 7 in second address byte
	ar[1] |= 0x08;		// set bit 3 in second address byte
	bb->rdt = READBACK_ACC_SRQ;
	return 16;			// this command stays at 16 bits
}

/**
 * Write a NOP to an extended accessory decoder to give it (or all in case of Broadcast)
 * the opportunity to send a railcom answer.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_extNOP (struct packet *p, struct bitbuffer *bb)
{
	volatile uint8_t *ar = bb->databits;

	(void) p;

	ar[1] &= ~0x80;		// clear bit 7 in second address byte
	ar[1] |= 0x09;		// set bits 0 and 3 in second address byte
	bb->rdt = READBACK_ACC_SRQ;
	return 16;			// this command stays at 16 bits
}

/**
 * Send out a LOGON_ENABLE(ALL) DCC-A packet.
 *
 * \param p			the packet with all needed information
 * \param bb		the bitbuffer to manipulate
 * \return			the new number of bits in the bitarray
 */
static int sig_dccaLogon (struct packet *p, struct bitbuffer *bb)
{
	volatile uint8_t *ar = bb->databits;

	bb->rdt = READBACK_DCCA_ID;
	ar[0] = 0xFE;							// the special "address" for all DCC-A operations
	ar[1] = 0xFC;							// the basic command stub
	bb->fmt = SIGNAL_DCC_A;					// mark this bit buffer as active special DCC-A buffer
	bb->adr = ar[0];						// help the reply function to better match answers
	switch (p->cmd) {
		case QCMD_DCCA_LOGON_ENABLE_ALL:
			ar[1] |= 0b00;
			break;
		case QCMD_DCCA_LOGON_ENABLE_LOCO:
			ar[1] |= 0b01;
			break;
		case QCMD_DCCA_LOGON_ENABLE_ACC:
			ar[1] |= 0b10;
			break;
		case QCMD_DCCA_LOGON_ENABLE_NOW:
			ar[1] |= 0b11;
			break;
		default:
			return 0;
	}
	ar[2] = (p->param.u32 >> 8) & 0xFF;		// two bytes CID (MSB first)
	ar[3] = (p->param.u32 >> 0) & 0xFF;
	ar[4] = p->value.u32 & 0xFF;			// the session ID with a single byte
	return 40;			// this command is always 40 bits long (i.e. 5 Bytes, no CRC needed)
}

static int sig_dccaSelect (struct packet *p, struct bitbuffer *bb)
{
	volatile uint8_t *ar = bb->databits;

	bb->rdt = READBACK_DCCA_DATA;
	ar[0] = 0xFE;							// the special "address" for all DCC-A operations
	ar[1] = 0xD0;							// the basic command stub
	ar[1] |= (p->adr >> 8) & 0x0F;			// the four MSBs of the manufacturer ID (usually, only 8-bit manufacturer IDs are used
	ar[2] = p->adr & 0xFF;					// the eight LSBs of the manufacturer ID
	ar[3] = (p->param.u32 >> 24) & 0xFF;	// MSB of the UID
	ar[4] = (p->param.u32 >> 16) & 0xFF;
	ar[5] = (p->param.u32 >>  8) & 0xFF;
	ar[6] = (p->param.u32 >>  0) & 0xFF;	// LSB of the UID
	bb->fmt = SIGNAL_DCC_A;					// mark this bit buffer as active special DCC-A buffer
	bb->adr = ar[0];						// help the reply function to better match answers
	switch (p->cmd) {						// supply the sub command byte, length and other parameters as needed
		case QCMD_DCCA_SELECT_SHORTINFO:
			ar[7] = 0xFF;
			ar[8] = dccA_CRC(ar, 8);
			bb->rdt = READBACK_DCCA_SHORTINFO;
			return 72;
		case QCMD_DCCA_SELECT_RDBLOCK:
			ar[7] = 0xFE;
			ar[8] = p->value.ui8[1];
			if (ar[8] == 3) {				// data space 3 is the CV data block, we must add the CV address and count
				ar[9] = (p->cva.cv >> 16) & 0xFF;
				ar[10] = (p->cva.cv >> 8) & 0xFF;
				ar[11] = (p->cva.cv >> 0) & 0xFF;
				ar[12] = (p->value.ui8[0]);
				ar[13] = dccA_CRC(ar, 13);
				return 112;
			} else {
				ar[9] = dccA_CRC(ar, 9);
				return 80;
			}
		case QCMD_DCCA_SELECT_DEC_STATUS:
			ar[7] = 0xFB;
			ar[8] = p->value.ui8[0];
			ar[9] = dccA_CRC(ar, 9);
			bb->rdt = READBACK_DCCA_ACK;
			return 80;
		default:	// invalid command -> return 0 bits
			break;
	}
	return 0;		// we get here, if the switch didn't "return" -> invalid command?
}

static int sig_dccaGetData (struct packet *p, struct bitbuffer *bb)
{
	volatile uint8_t *ar = bb->databits;

	bb->rdt = READBACK_DCCA_DATA;
	ar[0] = 0xFE;							// the special "address" for all DCC-A operations
	bb->fmt = SIGNAL_DCC_A;					// mark this bit buffer as active special DCC-A buffer
	bb->adr = ar[0];						// help the reply function to better match answers
	switch (p->cmd) {
		case QCMD_DCCA_GETDATA_START:
			ar[1] = 0x00;
			break;
		case QCMD_DCCA_GETDATA_CONT:
			ar[1] = 0x01;
			break;
		default:
			return 0;
	}
	return 16;
}

static int sig_dccaLogonAssign (struct packet *p, struct bitbuffer *bb)
{
	volatile uint8_t *ar = bb->databits;

	bb->rdt = READBACK_DCCA_ID;
	ar[0] = 0xFE;							// the special "address" for all DCC-A operations
	ar[1] = 0xE0;							// the basic command stub
	ar[1] |= (p->adr >> 8) & 0x0F;			// the four MSBs of the manufacturer ID (usually, only 8-bit manufacturer IDs are used
	ar[2] = p->adr & 0xFF;					// the eight LSBs of the manufacturer ID
	ar[3] = (p->param.u32 >> 24) & 0xFF;	// MSB of the UID
	ar[4] = (p->param.u32 >> 16) & 0xFF;
	ar[5] = (p->param.u32 >>  8) & 0xFF;
	ar[6] = (p->param.u32 >>  0) & 0xFF;	// LSB of the UID
	ar[7] = 0xC0 | ((p->value.u32 >> 8) & 0x3F);	// two bits 0b11 and 6 upper bits of the new track address (coding see RCN218, appendix D)
	ar[8] = p->value.u32 & 0xFF;
	ar[9] = dccA_CRC(ar, 9);
	bb->fmt = SIGNAL_DCC_A;					// mark this bit buffer as active special DCC-A buffer
	bb->adr = ar[0];						// help the reply function to better match answers
	return 80;
}

static struct dcc_renderer {
	enum queue_cmd	cmd;										///< the command that will be rendered
	dec_type		dt;											///< the decoder type that should be set for railcom purposes
	int (*adrfunc) (int adr, uint8_t *ar);				///< the address setting function
	int (*content) (struct packet *p, struct bitbuffer *bb);	///< the function that will render the variable data to the packet
} dcc_render[] = {
	{ QCMD_SETSPEED,				DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccSpeed },
	{ QCMD_EMERGENYSTOP,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccEmergencyStop },
	{ QCMD_DCC_SETF1_4,				DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccFunctions },
	{ QCMD_DCC_SETF5_8,				DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccFunctions },
	{ QCMD_DCC_SETF9_12,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccFunctions },
	{ QCMD_DCC_SETF13_20,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccFunctions },
	{ QCMD_DCC_SETF21_28,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccFunctions },
	{ QCMD_DCC_SETF29_36,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccFunctions },
	{ QCMD_DCC_BINSTATE,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccBinaryState },
	{ QCMD_MAGNET_ON,				DECODER_DCC_ACC,	sig_accAddress,		sig_accSwitch },
	{ QCMD_MAGNET_OFF,				DECODER_DCC_ACC,	sig_accAddress,		sig_accSwitch },
	{ QCMD_DCC_XACCASPECT,			DECODER_DCC_EXT,	sig_extAddress,		sig_extAspect },
	{ QCMD_DCC_POM_READ,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccReadByte },
	{ QCMD_DCC_POM_WRITE,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccWriteByte },
	{ QCMD_DCC_POM_WRITEBIT,		DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccWriteBit },
	{ QCMD_DCC_XWR1,				DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccShortPomWrite },
	{ QCMD_DCC_XWR2,				DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccShortPomWrite },
	{ QCMD_DCC_XPOM_RD_BLK,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_WR_BIT,			DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_WR_BYTE1,		DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_WR_BYTE2,		DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_WR_BYTE3,		DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_WR_BYTE4,		DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_ACC_RD_BLK,		DECODER_DCC_ACC,	sig_accAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_ACC_WR_BIT,		DECODER_DCC_ACC,	sig_accAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_ACC_WR_BYTE1,	DECODER_DCC_ACC,	sig_accAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_ACC_WR_BYTE2,	DECODER_DCC_ACC,	sig_accAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_ACC_WR_BYTE3,	DECODER_DCC_ACC,	sig_accAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_ACC_WR_BYTE4,	DECODER_DCC_ACC,	sig_accAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_EXT_RD_BLK,		DECODER_DCC_EXT,	sig_extAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_EXT_WR_BIT,		DECODER_DCC_EXT,	sig_extAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_EXT_WR_BYTE1,	DECODER_DCC_EXT,	sig_extAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_EXT_WR_BYTE2,	DECODER_DCC_EXT,	sig_extAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_EXT_WR_BYTE3,	DECODER_DCC_EXT,	sig_extAddress,		sig_dccXPom },
	{ QCMD_DCC_XPOM_EXT_WR_BYTE4,	DECODER_DCC_EXT,	sig_extAddress,		sig_dccXPom },
	{ QCMD_DCC_POM_ACC_READ,		DECODER_DCC_ACC,	sig_accAddress,		sig_dccReadByte },
	{ QCMD_DCC_POM_ACC_WRITE,		DECODER_DCC_ACC,	sig_accAddress,		sig_dccWriteByte },
	{ QCMD_DCC_POM_ACC_WRITEBIT,	DECODER_DCC_ACC,	sig_accAddress,		sig_dccWriteBit },
	{ QCMD_DCC_POM_EXT_READ,		DECODER_DCC_EXT,	sig_extAddress,		sig_dccReadByte },
	{ QCMD_DCC_POM_EXT_WRITE,		DECODER_DCC_EXT,	sig_extAddress,		sig_dccWriteByte },
	{ QCMD_DCC_POM_EXT_WRITEBIT,	DECODER_DCC_EXT,	sig_extAddress,		sig_dccWriteBit },
	{ QCMD_DCC_PT_VERIFYBYTE,		DECODER_ANY,		sig_nullAddress,	sig_dccReadByte },
	{ QCMD_DCC_PT_VERIFYBIT,		DECODER_ANY,		sig_nullAddress,	sig_dccReadBit },
	{ QCMD_DCC_PT_WRITEBYTE,		DECODER_ANY,		sig_nullAddress,	sig_dccWriteByte },
	{ QCMD_DCC_PT_WRITEBIT,			DECODER_ANY,		sig_nullAddress,	sig_dccWriteBit },
	{ QCMD_DCC_IDLE,				DECODER_ANY,		sig_nullAddress,	sig_dccIdle },
	{ QCMD_DCC_RESET,				DECODER_ANY,		sig_nullAddress,	sig_dccReset },
	{ QCMD_DCC_MODELTIME,			DECODER_ANY,		sig_dccBroadcast,	sig_dccModelTime },
	{ QCMD_DCC_SYSTIME,				DECODER_ANY,		sig_dccBroadcast,	sig_dccSysTime },
	{ QCMD_DCC_ACCNOP,				DECODER_DCC_ACC,	sig_accAddress,		sig_accNOP },
	{ QCMD_DCC_EXTACCNOP,			DECODER_DCC_EXT,	sig_extAddress,		sig_extNOP },
	{ QCMD_DCCA_LOGON_ENABLE_ALL,	DECODER_ANY,		sig_nullAddress,	sig_dccaLogon },
	{ QCMD_DCCA_LOGON_ENABLE_LOCO,	DECODER_ANY,		sig_nullAddress,	sig_dccaLogon },
	{ QCMD_DCCA_LOGON_ENABLE_ACC,	DECODER_ANY,		sig_nullAddress,	sig_dccaLogon },
	{ QCMD_DCCA_LOGON_ENABLE_NOW,	DECODER_ANY,		sig_nullAddress,	sig_dccaLogon },
	{ QCMD_DCCA_SELECT_SHORTINFO,	DECODER_ANY,		sig_nullAddress,	sig_dccaSelect },
	{ QCMD_DCCA_SELECT_RDBLOCK,		DECODER_ANY,		sig_nullAddress,	sig_dccaSelect },
	{ QCMD_DCCA_SELECT_DEC_STATUS,	DECODER_ANY,		sig_nullAddress,	sig_dccaSelect },
	{ QCMD_DCCA_GETDATA_START,		DECODER_ANY,		sig_nullAddress,	sig_dccaGetData },
	{ QCMD_DCCA_GETDATA_CONT,		DECODER_ANY,		sig_nullAddress,	sig_dccaGetData },
	{ QCMD_DCCA_LOGON_ASSIGN,		DECODER_ANY,		sig_nullAddress,	sig_dccaLogonAssign },
	{ QCMD_DCC_SDF,					DECODER_DCC_MOBILE,	sig_dccAddress,		sig_dccsdf },

	{ QCMD_NOP, DECODER_ANY, NULL, NULL }
};

/**
 * Render a packet for the DCC format.
 *
 * \param p		the high level packet that is to be rendered to the bitbuffer
 * \param bb	the bitbuffer which should be filled up with the rendered bit contents
 * \return		the bitbuffer as provided by the bb parameter or NULL, if not able to translate the request
 * \see			sig_renderBuffer()
 */
static struct bitbuffer *sig_renderDCC (struct packet *p, struct bitbuffer *bb)
{
	struct dcc_renderer *dr;

	if (fmtcfg->sigflags & SIGFLAG_RAILCOM) {
		bb->components = COMP_DCC_PREAMBLE | COMP_DCC_DATA | COMP_DCC_TAIL1 | COMP_DCC_CUTOUT;
		bb->dcc.tail = fmtcfg->dcc.rc_tailbits;
	} else {
		bb->components = COMP_DCC_PREAMBLE | COMP_DCC_DATA | COMP_DCC_TAIL1 | COMP_DCC_TAIL4;
		bb->dcc.tail = fmtcfg->dcc.tailbits;
	}
	bb->dcc.preamble = fmtcfg->dcc.preamble;
	bb->rdt = READBACK_STANDARD;	// the default

	dr = dcc_render;
	while (dr->cmd != QCMD_NOP) {
		if (dr->cmd == p->cmd) {
			bb->dt = dr->dt;
			if (dr->adrfunc) bb->bits = dr->adrfunc(p->adr, bb->databits);	// generate address information
			if (dr->content) bb->bits = dr->content(p, bb);					// add function arguments
			break;
		}
		dr++;
	}

	if (bb->rdt == READBACK_DCC_PT) {
		bb->dcc.tail = fmtcfg->dcc.tailbits;
		bb->components &= ~COMP_DCC_CUTOUT;
		bb->components |= COMP_DCC_TAIL4;
	}

	if (dr->cmd == QCMD_NOP || bb->bits <= 0) return NULL;
	bb->bits = sig_dccChecksum(bb->databits, bb->bits);

	return bb;
}

/*
 * ========================================================================================
 * M3 signal preparation (block level)
 * ========================================================================================
 */
static int sig_m3Address(uint16_t adr, uint8_t *ar)
{
	uint32_t adrfield;
	int bits;

	if (adr <= 127) {
		adrfield = (0b10 << 7) | adr;
		bits = 9;
	} else if (adr <= 511) {
		adrfield = (0b110 << 9) | adr;
		bits = 12;
	} else if (adr <= 2047) {
		adrfield = (0b1110 << 11) | adr;
		bits = 15;
	} else {
		adrfield = (0b1111L << 14) | adr;
		bits = 18;
	}
	return sig_addbits(ar, 0, adrfield, bits);
}

static int sig_m3Speed(uint8_t speed, uint8_t *ar, int bitpos)
{
	uint32_t speedfield;
	bool rev;

	rev = !(speed & 0x80);	// direction is reversed (0 = forward, 1 = reverse)
	speed &= 0x7F;
	if (speed != 0) speed++;	// speed 1 is the emergency stop code - skip it
	if ((speed % 16) == 0) {	// send a shortened code with only 7 data bits
		speedfield = (0b000 << 4) | ((rev) ? 0x8 : 0x0) | speed >> 4;
		return sig_addbits(ar, bitpos, speedfield, 7);
	}
	// we must send the full speed information
	speedfield = (0b001 << 8) | ((rev) ? 0x80 : 0x0) | speed;
	return sig_addbits(ar, bitpos, speedfield, 11);
}

static int sig_m3EmergencyStop(bool fwd, uint8_t *ar, int bitpos)
{
	uint32_t speedfield;

	// we must send the full speed information
	speedfield = (0b001 << 8) | ((fwd) ? 0x00 : 0x80) | 1;		// code for speed, direction and speed step 1
	return sig_addbits(ar, bitpos, speedfield, 11);
}

static int sig_m3Functions(uint32_t *funcs, uint8_t *ar, int bitpos)
{
	uint32_t funcfield, f_low;
	int bits;

	f_low = funcs[0] & 0xFFFF;		// we only handle F0 - F15 here
	if ((f_low & 0xFFF0) == 0) {			// only F0 - F3 have set bits
		funcfield = (0b010 << 4) | (f_low & 0x000F);
		bits = 7;
	} else if ((f_low & 0xFF00) == 0) {		// only F0 - F7 have set bits
		funcfield = (0b0110 << 8) | (f_low & 0x00FF);
		bits = 12;
	} else {								// we need to transmit F0 - F15 all together
		funcfield = (0b0111 << 16) | (f_low & 0xFFFF);
		bits = 20;
	}
	return sig_addbits(ar, bitpos, funcfield, bits);
}

static int sig_m3SingleFunc(uint32_t *funcs, int f, uint8_t *ar, int bitpos)
{
	uint32_t funcfield;
	int fidx;
	bool on;

	if (f >= LOCO_MAX_FUNCS) return bitpos;
	fidx = f / BITS_PER_WORD;
	on = !!(funcs[fidx] & (1 << f % BITS_PER_WORD));

	// the bit position 1 is a constant '0': 0b100NNNNNNNN0F (NNN=function to switch, F=new state of the function)
	funcfield = (0b100 << 9) | ((f & 0x7F) << 2) | ((on) ? 0b1 : 0b0);
	return sig_addbits(ar, bitpos, funcfield, 12);
}

static int sig_m3cvread(cvadrT cva, int bytes, uint8_t *ar, int bitpos)
{
	uint32_t cv;

	cv = (0b111000u << 18) | (cva.cv << 2);
	if (bytes == 2) cv |= 0b01;
	else if (bytes == 4) cv |= 0b10;
	else if (bytes == 8) cv |= 0b11;
	return sig_addbits(ar, bitpos, cv, 24);
}

static int sig_m3cvwrite(cvadrT cva, uint8_t *val, int bytes, uint8_t *ar, int bitpos)
{
	uint32_t cv;

	cv = (0b111001u << 18) | (cva.cv << 2);
	if (bytes == 2) cv |= 0b01;
	else if (bytes == 4) cv |= 0b10;
//	else if (bytes == 8) cv |= 0b11;	// not yet supported by decoders, nor by our sigq structure
	bitpos = sig_addbits(ar, bitpos, cv, 24);
	switch (bytes) {
		default:
		case 1:
			bitpos = sig_addbits(ar, bitpos, val[0], 8);
			break;
		case 2:
			bitpos = sig_addbits(ar, bitpos, val[0], 8);
			bitpos = sig_addbits(ar, bitpos, val[1], 8);
			break;
		case 4:
			bitpos = sig_addbits(ar, bitpos, val[0], 8);
			bitpos = sig_addbits(ar, bitpos, val[1], 8);
			bitpos = sig_addbits(ar, bitpos, val[2], 8);
			bitpos = sig_addbits(ar, bitpos, val[3], 8);
			break;
	}
	return bitpos;
}

static int sig_m3Beacon(uint8_t *ar, uint32_t id, uint16_t announce)
{
	int bitpos;

	bitpos = sig_m3Address(0, ar);							// broadcast address 0
	bitpos = sig_addbits(ar, bitpos, 0b111101, 6);			// 6 bit command code 111.101
	bitpos = sig_addbits(ar, bitpos, id, 32);				// Station->ID
	return sig_addbits(ar, bitpos, announce, 16);			// announce counter
}

static int sig_m3Search(uint32_t uid, int bits, uint8_t *ar)
{
	int bitpos;

	bitpos = sig_m3Address(0, ar);							// broadcast address 0
	bitpos = sig_addbits(ar, bitpos, 0b111010, 6);			// 6 bit command code 111.010
	bitpos = sig_addbits(ar, bitpos, bits, 6);				// number of address bits to query
	return sig_addbits(ar, bitpos, uid, 32);				// the UID to check for
}

static int sig_m3Nadr(uint32_t uid, uint16_t newadr, uint8_t *ar)
{
	int bitpos;

	bitpos = sig_m3Address(0, ar);							// broadcast address 0
	bitpos = sig_addbits(ar, bitpos, 0b111011, 6);			// 6 bit command code 111.011
	bitpos = sig_addbits(ar, bitpos, newadr, 14);			// 14 bits of the new address (track address, i.e. loco number)
	return sig_addbits(ar, bitpos, uid, 32);				// target address (UID of decoder)
}

#if 0	// currently not used - to keep compiler happy
static int sig_m3Ping(uint32_t uid, uint16_t adr, uint8_t *ar)
{
	int bitpos;

	bitpos = sig_m3Address(adr, ar);						// the loco address
	bitpos = sig_addbits(ar, bitpos, 0b111100, 6);			// 6 bit command code 111.100
	return sig_addbits(ar, bitpos, uid, 32);				// target UID of decoder
}
#endif

#define M3_POLYNOM	0x07	// CRC-Polynom: x^8 + x^2 + x^1 + x^0
#define M3_INITIAL	0x7F	// Initialwert

static int sig_m3crc(uint8_t *ar, int bits) {
	uint16_t crc = M3_INITIAL;
	int i;
	bool b;

	if (bits < 0) bits = 0;

	for (i = 0; i < bits; i++) {
		crc <<= 1;
		if (crc & 0x100) crc = (crc & 0xFF) ^ M3_POLYNOM;
		b = !!(ar[i >> 3] & (0x80 >> (i & 7)));
		if (b) crc ^= 0x01;
	}
	for (i = 0; i < 8; i++) {	// 8 more '0' bits representing a cleared crc
		crc <<= 1;
		if (crc & 0x100) crc = (crc & 0xFF) ^ M3_POLYNOM;
	}
	return sig_addbits(ar, bits, crc, 8);
}

/**
 * Render a packet for the M3 format.
 *
 * \param p		the high level packet that is to be rendered to the bitbuffer
 * \param bb	the bitbuffer which should be filled up with the rendered bit contents
 * \return		the bitbuffer as provided by the bb parameter or NULL, if not able to translate the request
 * \see			sig_renderBuffer()
 */
static struct bitbuffer *sig_renderM3 (struct packet *p, struct bitbuffer *bb)
{
	// most commands will go to mobile decodes, so we start off with a decoder address and standard signal components
	bb->bits = sig_m3Address(p->adr, bb->databits);
	bb->components = COMP_M3_FLAG1 | COMP_M3_DATA | COMP_M3_ENDFLAG;
	bb->dt = DECODER_M3_MOBILE;		// the only M3 option ...
	bb->adr = p->adr;

	switch (p->cmd) {
		case QCMD_SETSPEED:
			bb->bits = sig_m3Speed(p->value.i32, bb->databits, bb->bits);
			break;
		case QCMD_EMERGENYSTOP:
			bb->bits = sig_m3EmergencyStop((p->value.i32 & 0x80) ? true : false, bb->databits, bb->bits);
			break;
		case QCMD_SETFUNC:
			bb->bits = sig_m3Functions(p->funcs, bb->databits, bb->bits);
			break;
		case QCMD_M3_SINGLEFUNC:
			bb->bits = sig_m3SingleFunc(p->funcs, p->param.i32, bb->databits, bb->bits);
			break;
		case QCMD_M3_SPEEDFUNC:
			bb->bits = sig_m3Speed(p->value.i32, bb->databits, bb->bits);
			bb->bits = sig_m3Functions(p->funcs, bb->databits, bb->bits);
			break;
		case QCMD_M3_BEACON:
			bb->bits = sig_m3Beacon(bb->databits, p->param.u32, p->value.u32);
			break;
		case QCMD_M3_SEARCH:
			bb->bits = sig_m3Search(p->value.u32, p->param.i32, bb->databits);
			bb->components |= COMP_M3_FLAG2 | COMP_M3_REPLYSTART | COMP_M3_REPLYWIN1;	// additional flags
			bb->components |= COMP_M3_FLAG3 | COMP_M3_REPLYWIN2 | COMP_M3_FLAG4;		// additional flags
			bb->adr = 0;
			bb->rdt = READBACK_M3BIN;
			break;
		case QCMD_M3_NADR:
			bb->bits = sig_m3Nadr(p->value.u32, p->adr, bb->databits);
			break;
		case QCMD_M3_CVREAD:	// p->param is the amount of bytes to read (1, 2, 4 or 8 bytes)
			bb->bits = sig_m3cvread(p->cva, p->param.i32, bb->databits, bb->bits);
			bb->components |= COMP_M3_FLAG2 | COMP_M3_REPLYSTART | COMP_M3_REPLYSYNC | COMP_M3_REPLYDATA | COMP_M3_FLAG4;
			bb->m3.replybits = 3 + (8 * p->param.i32) + 8 + 4;		// 3 startbits, 8 data bits per byte, 8 crc-bits, 4 additional bits
			bb->cva = p->cva;			// transfer this information up to the reader function to be able to form a complete answer
			bb->param.i32 = p->param.i32;	// this is the number of requested answer bytes (excluding the CRC)
			bb->rdt = READBACK_M3DATA;
			break;
		case QCMD_M3_CVWRITE:	// p->value is the value to write, p->param is the amount of bytes to write (1, 2 or 4)
			bb->bits = sig_m3cvwrite(p->cva, p->value.ui8, p->param.i32, bb->databits, bb->bits);
			break;
		default:
			return NULL;
	}
	bb->bits = sig_m3crc(bb->databits, bb->bits);

	return bb;
}

/*
 * ========================================================================================
 * the main task handling signal generation
 * ========================================================================================
 */

/**
 * Render the bit representation of a packet request for the interrupt handling.
 * This translates the hight level packet buffer to the very detailed information that
 * controls the behavior of the timer compare outputs managed by the interrupt(s).
 *
 * If anything goes wrong (i.e. impossible request for a certain type of format),
 * NULL is returned. This will in turn lead to not presenting a packet to the interrupt
 * which in turn will choose to send out an idle packet instead (i.e. use the \ref idle
 * packet instead of \ref irq_next).
 *
 * This function just checks the requested track format of the high level packet and
 * then calls one of the format dependant functions sig_renderMM(), sig_renderDCC() or
 * sig_renderM3(). If an unknown format is encountered it will instantly return a
 * NULL pointer effectively ignoring and throwing away the high level packet request.
 *
 * \param p		the high level packet that is to be rendered to the bitbuffer
 * \param bb	the bitbuffer which should be filled up with the rendered bit contents
 * \return		the bitbuffer as provided by the bb parameter or NULL, if not able to translate the request
 */
static struct bitbuffer *sig_renderBuffer (struct packet *p, struct bitbuffer *bb)
{
	if (!p || !bb) return NULL;

	memset ((void *) bb, 0, sizeof(*bb));	// clear bitbuffer to have a clean starting point
	bb->cb = p->cb;							// prepare a possible callback
	bb->priv = p->priv;
	bb->fmt = sig_getTrackFormat(p->fmt);	// we (almost) always can preset the track format ...
	bb->repeat = p->repeat;					// ... and copy the packet repeat
	bb->adr = p->adr;

	if ((p->cmd == QCMD_SETSPEED || p->cmd == QCMD_MM_SETSPEED_27A) && rt.tm == TM_HALT) p->value.u32 &= 0x80;

	switch (bb->fmt) {
		case SIGNAL_MMSLOW:
		case SIGNAL_MMFAST:
			return sig_renderMM (p, bb);
		case SIGNAL_DCC:
			return sig_renderDCC(p, bb);
		case SIGNAL_M3:
			return sig_renderM3 (p, bb);
		default:
			break;
	}
	return NULL;
}

#define M3REPLY_TIMEOUT		1
#define M3REPLY_ACK			2
#define M3REPLY_NACK		3

static bool sig_m3SearchCallback (struct decoder_reply *msg, flexval fv)
{
	TaskHandle_t task;
	uint32_t val;

	task = (TaskHandle_t) fv.p;
	switch (msg->mt) {
		default:
		case DECODERMSG_TIMEOUT:
			val = M3REPLY_TIMEOUT;
			break;
		case DECODERMSG_M3BIN:
			val = (msg->data[0]) ? M3REPLY_ACK : M3REPLY_NACK;
			break;
	}
	xTaskNotify(task, val, eSetValueWithOverwrite);
	return false;	// de-register this callback
}

/**
 * Search for m3 decoders without loco address (SID) and return the found UID
 * and number of decoders found (future expansion - currently only a singel decoder can be found!).
 *
 * \param id	pointer to a variable that will receive the UID of the decoder, if any
 * \return		the number of decoders found (0 .. n - currently max. 1) or a negative error code
 */
int sig_searchM3Loco (uint32_t *id)
{
	uint32_t rc;
	uint32_t uid, mask;
	flexval fv;
	struct packet *p;
	int len;

	if (!id) return -1;


	uid = 0;
	len = 0;
	while (len <= 32) {
//		reply_register(DECODER_M3_MOBILE, 0, DECODERMSG_M3BIN, sig_m3SearchCallback, task, 500);
		mask = 1 << (32 - len);		// if (len == 0) we will have a zero mask - that is intended!
		fv.p = xTaskGetCurrentTaskHandle();
		if ((p = sigq_m3SearchPacket(uid, len, sig_m3SearchCallback, fv)) == NULL) {
			log_error ("%s(): cannot create packet @ UID=0x%lX len %d!\n", __func__, uid, len);
			return -2;
		}
		sigq_queuePacket(p);

//		if (m3_scan(uid, len) != 0) {
//			log_error ("%s(): cannot gen packet @ UID=0x%lX len %d!\n", __func__, uid, len);
//			return -2;
//		}
		rc = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
		if (!rc) {
			log_error ("%s(): TIMEOUT @ UID=0x%lX len %d!\n", __func__, uid, len);
			return -3;
		}
		if (rc == M3REPLY_TIMEOUT) rc = M3REPLY_NACK;		// both should be equivalent
		if ((rc == M3REPLY_NACK) && (mask == 0)) {
			log_msg(LOG_INFO, "%s(): no decoder found\n", __func__);
			return 0;
		} else if ((rc == M3REPLY_NACK) && ((uid & mask) != 0)) {
			log_error ("%s(): lost tree search (bit %d neither 0 nor 1?)\n", __func__, len - 1);
			return -4;
		}
		if (rc == M3REPLY_NACK) {		// no loco found with a cleared bit - try a set bit
			uid |= mask;
		} else {
			len++;
		}
	}

	if (len > 32) {
		log_msg(LOG_INFO, "%s(): LOCO UID 0x%08lx\n", __func__, uid);
		*id = uid;
		return 1;
	}
	log_error ("%s(): failed\n", __func__);
	return -5;
}

/**
 * Start main (builtin) booster signal generation
 */
static void sig_prepareMainBooster (void)
{
	if (TIM1->CR1 & TIM_CR1_CEN) return;		// signal already running

	NVIC_DisableIRQ(TIM1_UP_IRQn);				// just to be sure: disable interrupt in NVIC
	NVIC_DisableIRQ(TIM1_CC_IRQn);				// just to be sure: disable interrupt in NVIC
	CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);			// just to be sure: disable timer
	if (!SIGtask) return;						// if the signal task is not running, we will not start signal generation

	key_resetShort();							// reset the short-counters for Märklin and DCC boosters
	sigq_flush();								// flush packets from signal queue to start with a clean state

	// start signal generation with preamble bits
	TIM1->ARR = fmtcfg->dcc.tim_one - 1;
	TIM1->CCR1 = fmtcfg->dcc.tim_one;
	TIM1->CCR2 = fmtcfg->dcc.tim_one >> 1;
	TIM1->CCR3 = fmtcfg->dcc.tim_one >> 1;
	if (signals & BOOSTER_BIDIB) {
		TIM1->CCR4 = fmtcfg->dcc.tim_one >> 1;
	} else {
		TIM1->CCR4 = 0;
	}
	TIM1->CNT = 0;								// preset counter to 0
	TIM1->RCR = 0;
	TIM1->EGR = TIM_EGR_UG;						// generate an update event
	SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);			// set master output enable

	TIM1->SR = 0;								// clear a possibly pending interrupt
	SET_BIT (TIM1->DIER, TIM_DIER_UIE);			// enable update interupt (happens at timer reload time)
	CLEAR_BIT(TIM1->SR, TIM_SR_CC4IF);			// when starting, the compare interrupt should be OFF
}

#ifndef HW_REV07
/**
 * Start Märklin (5-pol) booster signal generation
 */
static void sig_prepareMklnBooster (void)
{
	CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN);			// just to be sure: disable timer
	MMBOOSTER_ARR = fmtcfg->dcc.tim_one - 1;
	MMBOOSTER_CCR = fmtcfg->dcc.tim_one >> 1;
	MMBOOSTER_RCR = 0;
	TIM3->CNT = 0;								// preset counter to 0
	CLEAR_BIT(TIM3->CR1, TIM_CR1_UDIS);			// enable updates
	TIM3->EGR = TIM_EGR_UG;						// generate an update event

	TIM3->SR = 0;								// clear a possibly pending interrupt
	SET_BIT (TIM3->DIER, TIM_DIER_CC3IE);		// enable compare interupt channel 3
}

/**
 * Start DCC (CDE, 3-pol) booster signal generation
 */
static void sig_prepareCDEBooster (void)
{
	CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);			// just to be sure: disable timer
	DCCBOOSTER_ARR = fmtcfg->dcc.tim_one - 1;
	DCCBOOSTER_CCR = fmtcfg->dcc.tim_one >> 1;
	DCCBOOSTER_RCR = 0;
	TIM8->CNT = 0;								// preset counter to 0
	TIM8->EGR = TIM_EGR_UG;						// generate an update event
	SET_BIT(TIM8->BDTR, TIM_BDTR_MOE);			// set master output enable
	TIM8->CNT = 3;								// pre-compensate for some small delay in OPAMP

	TIM8->SR = 0;								// clear a possibly pending interrupt
}
#endif

/**
 * Immediately stop signal generation (for whatever reason).
 *
 * This will disable the signal generation even in the middle of a packet
 * and discard all scheduled packets.
 */
static void sig_stopBooster (void)
{
	NVIC_DisableIRQ(TIM1_UP_IRQn);					// disable interrupt in NVIC
	NVIC_DisableIRQ(TIM1_CC_IRQn);					// disable interrupt in NVIC

	signals = 0;									// all signal generation is suspended
	ts_boosteroff();								// turn off booster
	ts_setCCMode (0);								// stop limiter function
	MKLNBST_OFF();									// turn off Märklin booster

	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
	CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);				// disable timer
#ifndef HW_REV07
	NVIC_DisableIRQ(TIM3_IRQn);						// disable interrupt in NVIC (not for TIM8, because it shares its vector with TIM13)
	CLEAR_BIT(TIM8->BDTR, TIM_BDTR_MOE);
	CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN);				// disable timer
	TIM3->CNT = 0xFFFF;								// ensure that there is LOW on the output pin of Märklin booster
	CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);				// disable timer
#endif
	BDBctrl_boosterOff();							// switch off BiDiB booster (broadcast MSG_BOOST_OFF)
	event_fire(EVENT_CURRENT, 0, NULL);				// fire an event
}

/**
 * Start or stop signal generation independant of the boosters.
 * When stopping the signal, the boosters must be turned off.
 *
 * This independant switching is (currently) only supported for BiDiB.
 *
 * \param booster	a bit array with bits for the different signal outputs.
 * 					If no bit is set, all signal generation should be switched off.
 */
static void sig_enable (int booster)
{
	signals = booster;		// 'signals' is evaluated by the timer IRQs to see, if a particular booster should be shut off

	if (booster) {
		if (booster & BOOSTER_BUILTIN)	sig_prepareMainBooster();
#ifndef HW_REV07
		if (booster & BOOSTER_MM)		sig_prepareMklnBooster();
		if (booster & BOOSTER_CDE)		sig_prepareCDEBooster();
#endif
		if (booster & BOOSTER_BUILTIN) {
			SET_BIT (TIM1->CR1, TIM_CR1_CEN);			// enable TIM1 (internal booster, ext. signal representation)
			NVIC_ClearPendingIRQ(TIM1_UP_IRQn);			// clear a possibly pending interrupt request in NVIC
			NVIC_EnableIRQ(TIM1_UP_IRQn);				// enable interrupt in NVIC
			NVIC_ClearPendingIRQ(TIM1_CC_IRQn);			// clear a possibly pending interrupt request in NVIC
			NVIC_EnableIRQ(TIM1_CC_IRQn);				// enable interrupt in NVIC
		}
#ifndef HW_REV07
		if (booster & BOOSTER_MM) {
			SET_BIT (TIM3->CR1, TIM_CR1_CEN);			// enable TIM3 (Märklin 5-pol connector)
			NVIC_ClearPendingIRQ(TIM3_IRQn);			// clear a possibly pending interrupt request in NVIC
			NVIC_EnableIRQ(TIM3_IRQn);					// enable interrupt in NVIC
		}
		if (booster & BOOSTER_CDE) {
			SET_BIT (TIM8->CR1, TIM_CR1_CEN);			// enable TIM8 (DCC booster - CDE connector)
		}
#endif
	} else {		// no boosters defined - turn all off
		sig_stopBooster();
	}
}

enum trackmode sig_setMode (enum trackmode mode)
{
	static SemaphoreHandle_t mutex;

	if (rt.tm == TM_OVERTTEMP && mode != TM_TEMPOK && mode != TM_RESET) return rt.tm;

	if (mode != rt.tm && mutex_lock(&mutex, 10, __func__)) {
		switch (mode) {
			case TM_STOP:
			case TM_TEMPOK:
				sig_stopBooster();
				mode = TM_STOP;
				break;
			case TM_SHORT:
				sig_stopBooster();
				break;
			case TM_HALT:
				if (rt.tm != TM_GO) mode = rt.tm;		// this mode is only reachable from GO modus, so we must stay at the previous mode
				break;
			case TM_GO:
				if (rt.tm == TM_SHORT) {
					event_fire(EVENT_SYS_STATUS, SYSEVENT_STOP, NULL);	// fire an intermediate event, that SHORT status ended
				}
				if (rt.tm != TM_HALT) {							// if we come from TM_HALT, we don't restart the timers - just change operational mode
					sig_enable(BOOSTER_ALL);
					ts_boosteron(false);
					MKLNBST_ON();
					BDBctrl_boosterOn();						// switch on BiDiB booster (broadcast MSG_BOOST_ON)
					rgb_go();
					event_fire(EVENT_CURRENT, 0, NULL);			// fire an event
				}
				break;
			case TM_SIGON:
				if (rt.tm == TM_GO || rt.tm == TM_HALT) {		// the signal generation is already running - just switch off the boosters
					sig_stopBooster();
				} else {										// the signal generation is not running - only start the signals without boosters
					sig_enable(BOOSTER_ALL);
				}
				break;
			case TM_DCCPROG:
			case TM_TAMSPROG:
				sig_stopBooster();
				sig_enable(BOOSTER_BUILTIN);
				ts_boosteron(true);
				break;
			case TM_TESTDRIVE:
				if (rt.tm != TM_STOP) {
					sig_stopBooster();
				}
				sig_enable(BOOSTER_BUILTIN);
				ts_boosteron(true);
				break;
			case TM_POWERFAIL:
				sig_stopBooster();
				break;
			case TM_RESET:
				sig_stopBooster();
				break;
			case TM_OVERTTEMP:
				sig_stopBooster();
				break;
		}
		if (mode == TM_SHORT && rt.tm == TM_STOP) {
			printf ("%s(): SHORT when already in STOP - ignored\n", __func__);
		} else {
			rt.tm = mode;
		}
		mutex_unlock(&mutex);
	}
	switch (rt.tm) {
		case TM_STOP:
		case TM_TEMPOK:		// cannot happen, but too keep compiler happy
			rgb_stop();
			seg_stop();
			event_fire(EVENT_SYS_STATUS, SYSEVENT_STOP, NULL);	// fire an event
			break;
		case TM_SHORT:
			rgb_short();
			seg_short();
			event_fire(EVENT_SYS_STATUS, SYSEVENT_SHORT, NULL);	// fire an event
			break;
		case TM_HALT:
			event_fire(EVENT_SYS_STATUS, SYSEVENT_HALT, NULL);	// fire an event
			break;
		case TM_SIGON:
			event_fire(EVENT_SYS_STATUS, SYSEVENT_SIGON, NULL);	// fire an event
			break;
		case TM_GO:
			event_fire(EVENT_SYS_STATUS, SYSEVENT_GO, NULL);	// fire an event
			break;
		case TM_DCCPROG:
		case TM_TAMSPROG:
			seg_progmode();
			break;
		case TM_TESTDRIVE:
			seg_testdrive();
			event_fire(EVENT_SYS_STATUS, SYSEVENT_TESTDRIVE, NULL);	// fire an event
			break;
		case TM_POWERFAIL:
			seg_powerfail();
			break;
		case TM_RESET:
			rgb_stop ();
			event_fire(EVENT_SYS_STATUS, SYSEVENT_RESET, NULL);	// fire an event
			seg_reboot();
			break;
		case TM_OVERTTEMP:
			rgb_overtemp();
			seg_overtemp();
			event_fire(EVENT_SYS_STATUS, SYSEVENT_OVERTEMP, NULL);	// fire an event
			break;
	}
	return rt.tm;
}

enum trackmode sig_getMode (void)
{
	return rt.tm;
}

uint32_t sig_getM3Beacon (void)
{
	return fmtcfg->m3.beacon;
}

uint16_t sig_getM3AnnounceCounter (void)
{
	return fmtcfg->m3.announce;
}

void sig_setM3Beacon (uint32_t bid, uint16_t announce)
{
	fmtcfg->m3.beacon = bid;
	fmtcfg->m3.announce = announce;
}

static void sig_default (void)
{
	// prepare the one-and-only idle block
	memset (&idle, 0, sizeof(idle));
	idle.fmt = SIGNAL_DCC;
	idle.bits = 24;
	idle.databits[0] = 0xFF;		// DCC System-Broadcast	(real broadcasts use address 0x00)
	idle.databits[1] = 0x00;		// DCC_CONTROL
	idle.databits[2] = 0xFF;		// DCC CRC
	idle.components = COMP_DCC_PREAMBLE | COMP_DCC_DATA | COMP_DCC_TAIL1 | COMP_DCC_TAIL4;
	idle.dcc.preamble = 12;			// use 12 preamble bits
	idle.dcc.tail = 5;				// use five tails bits including the cutout
	idle.rdt = READBACK_STANDARD;

	// prepare the one-and-only MM-idle block
	memset (&mmidle, 0, sizeof(idle));
	mmidle.fmt = SIGNAL_MMSLOW;
	mmidle.bits = 18;
	mmidle.databits[0] = sig_mmLookup(0);		// the MM system defines an address of 0 as a broadcast / pause packet
	mmidle.databits[1] = 0x00;		// FUNC is off, upper lower speed bits 0
	mmidle.databits[2] = 0x00;		// upper speed bits 0
	mmidle.components = COMP_MM_PACKET_GAP | COMP_MM_DATA1 | COMP_MM_REPEAT_GAP | COMP_MM_DATA2 | COMP_MM_END_GAP;

	// prepare the one-and-only reset block
	memset (&reset, 0, sizeof(reset));
	reset.fmt = SIGNAL_DCC;
	reset.bits = 24;
	reset.databits[0] = 0x00;		// DCC Broadcast
	reset.databits[1] = 0x00;		// DCC_CONTROL
	reset.databits[2] = 0x00;		// DCC CRC
	reset.components = COMP_DCC_PREAMBLE | COMP_DCC_DATA | COMP_DCC_TAIL1 | COMP_DCC_TAIL4;
	reset.dcc.preamble = 20;		// use 20 preamble bits
	reset.dcc.tail = 6;				// use six tails bits including the cutout
	reset.rdt = READBACK_DCC_PT;

	// prepare the one-and-only m3 beacon block
	memset (&m3Beacon, 0, sizeof(m3Beacon));
	m3Beacon.fmt = SIGNAL_M3;
	m3Beacon.bits = sig_m3Beacon(m3Beacon.databits, fmtcfg->m3.beacon, fmtcfg->m3.announce);
	m3Beacon.bits = sig_m3crc(m3Beacon.databits, m3Beacon.bits);
	m3Beacon.components = COMP_M3_FLAG1 | COMP_M3_DATA | COMP_M3_ENDFLAG;
}

/**
 * Get and prepare a bit buffer from the packet for signal generation.
 * The packet itself can be freed in the end if it was rendered to a
 * buffer. In case the special buffers (DCC-A and XPOM) are already in
 * use, the packet must be pushed back to the packet queue instead.
 *
 * \param p		the packet to render
 * \return		a pointer to a bit buffer or NULL under some circumstances
 */
static struct bitbuffer *sig_getBuffer (struct packet *p)
{
	static unsigned int rounds;

	struct bitbuffer *bb;

	bb = &buffers[rounds % BUFFER_COUNT];
	bb = sig_renderBuffer(p, bb);		// grab next buffers and prepare the packet (may return NULL instead of the bit buffer)
	if (bb) rounds++;					// that buffer now is in use and we should advance to the next buffer for the next round
	free (p);							// free the packet here instead in the caller because the caller can't know if we pushed the packet back
	return bb;
}

static int sig_countStdSlots (void)
{
	int cnt = 0;

	if (QSLOT_STD0) cnt++;
	if (QSLOT_STD1) cnt++;
	if (QSLOT_STD2) cnt++;
	if (QSLOT_STD3) cnt++;
	return cnt;
}

static int sig_countXpomSlots (void)
{
	int cnt = 0;

	if (QSLOT_XPOM00) cnt++;
	if (QSLOT_XPOM01) cnt++;
	if (QSLOT_XPOM10) cnt++;
	if (QSLOT_XPOM11) cnt++;
	return cnt;
}

void vSigGeneration (void *pvParameters)
{
	struct bitbuffer *bb, *onHold;
	struct packet *p;
	TickType_t next_systime, next_m3beacon;
//	int cnt = 0;

	(void) pvParameters;		// not used

	sig_default();
	SIGtask = xTaskGetCurrentTaskHandle();
	next_systime = next_m3beacon = 0;

	sig_init_TIM1();
#ifndef HW_REV07
	sig_init_TIM3();
	sig_init_TIM8();
#endif
	fmtcfg = cnf_getFMTconfig();
	bb = onHold = NULL;

	for (;;) {
		// if we could assemble a bit buffer and didn't need to put it on hold, we can just create the next one without waiting
		if (!bb || onHold) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		bb = onHold;
		onHold = NULL;
		switch (rt.tm) {
			case TM_HALT:
			case TM_GO:
			case TM_TESTDRIVE:
				// check for hold back buffer or generate next buffer from signal queue
				if (!bb) {
					// drop packets that are M3 if M3 is disabled
					while ((p = sigq_getpacket(false)) != NULL) {
						if (!FMT_IS_M3(p->fmt) || (fmtcfg->sigflags & SIGFLAG_M3ENABLED)) break;
						free (p);
					}
					if (p) bb = sig_getBuffer(p);
				}
				// if nothing else is to do we can check for m3beacon, DCC time packet or a refresh cycle
				if (!bb && sig_countStdSlots() < 2) {	// not much to do - insert one of the m3 beacon, DCC time packet or a refresh
					if ((fmtcfg->sigflags & SIGFLAG_M3ENABLED) && tim_isoverUnset(next_m3beacon) && m3_inRefresh()) {		// insert a M3-Beacon every 500ms
						m3Beacon.bits = sig_m3Beacon(m3Beacon.databits, fmtcfg->m3.beacon, fmtcfg->m3.announce);
						m3Beacon.bits = sig_m3crc(m3Beacon.databits, m3Beacon.bits);
						m3Beacon.repeat = 1;
						next_m3beacon = tim_timeout(500);
						bb = &m3Beacon;
//						log_msg (LOG_INFO, "%s() m3 beacon packet %p\n", __func__, bb);
					} else if (tim_isoverUnset(next_systime)) {				// send a DCC time packet every 100s
//						log_msg (LOG_INFO, "%s() systime packet %p (planned at TICK %lu)\n", __func__, bb, next_systime);
						bb = sig_getBuffer(sigq_sysTimePacket());
						next_systime = tim_timeout(100000);
					} else {												// OK, no special tasks here - supply a refresh packet
						bb = sig_getBuffer(sigq_getpacket(true));
//						log_msg (LOG_INFO, "%s() refresh %p\n", __func__, bb);
					}
				}
//				if (bb) log_msg(LOG_INFO, "%s() ADR=%d FMT=%d\n", __func__, bb->adr, bb->fmt);
//				else log_msg(LOG_INFO, "%s() no buffer grabbed\n", __func__);
				break;
			case TM_DCCPROG:
			case TM_TAMSPROG:
				// check for hold back buffer or generate next buffer from signal queue
				if (!bb) {
					// drop packets that are M3 if M3 is disabled
					while ((p = sigq_getpacket(false)) != NULL) {
						if (!FMT_IS_M3(p->fmt)) break;
						free (p);
					}
					if (p) bb = sig_getBuffer(p);
				}
				break;
			default:		// any STOP condition should clear the queue and timeouts (so reset just everything)
				next_systime = next_m3beacon = 0;
				bb = NULL;
				memset (&queue, 0, sizeof(queue));
				sigq_flush();
				continue;		// wait for next wakeup
		}

		// now, if we have a bitbuffer, try to place it in the correct slot for the interrupt
		if (bb) switch (bb->fmt) {
			case SIGNAL_DCC_XPOM00:	// this must go a fixed position
				if (!QSLOT_XPOM00) {
					reply_register(bb->dt, bb->adr, DECODERMSG_XPOM00, bb->cb, bb->priv, 500);
					QSLOT_XPOM00 = bb;
				} else onHold = bb;	// slot is not free - put buffer on hold
				break;
			case SIGNAL_DCC_XPOM01:	// this must go a fixed position
				if (!QSLOT_XPOM01) {
					reply_register(bb->dt, bb->adr, DECODERMSG_XPOM01, bb->cb, bb->priv, 500);
					QSLOT_XPOM01 = bb;
				} else onHold = bb;	// slot is not free - put buffer on hold
				break;
			case SIGNAL_DCC_XPOM10:	// this must go a fixed position
				if (!QSLOT_XPOM10) {
					reply_register(bb->dt, bb->adr, DECODERMSG_XPOM10, bb->cb, bb->priv, 500);
					QSLOT_XPOM10 = bb;
				} else onHold = bb;	// slot is not free - put buffer on hold
				break;
			case SIGNAL_DCC_XPOM11:	// this must go a fixed position
				if (!QSLOT_XPOM11) {
					reply_register(bb->dt, bb->adr, DECODERMSG_XPOM11, bb->cb, bb->priv, 500);
					QSLOT_XPOM11 = bb;
				} else onHold = bb;	// slot is not free - put buffer on hold
				break;
			case SIGNAL_DCC_A:		// this must go a fixed position
				if (!QSLOT_DCCA) {
					reply_register(bb->dt, bb->adr, DECODERMSG_UNIQUE, bb->cb, bb->priv, 500);
					reply_register(bb->dt, bb->adr, DECODERMSG_SHORTINFO, bb->cb, bb->priv, 500);
					reply_register(bb->dt, bb->adr, DECODERMSG_DCCABLOCK, bb->cb, bb->priv, 500);
					reply_register(bb->dt, bb->adr, DECODERMSG_COLLISION, bb->cb, bb->priv, 500);
					reply_register(bb->dt, bb->adr, DECODERMSG_DECSTATE, bb->cb, bb->priv, 500);
					reply_register(bb->dt, bb->adr, DECODERMSG_ACK, bb->cb, bb->priv, 500);
					reply_register(bb->dt, bb->adr, DECODERMSG_NOANSWER, bb->cb, bb->priv, 500);
					QSLOT_DCCA = bb;
				} else onHold = bb;	// slot is not free - put buffer on hold
				break;
			case SIGNAL_M3:
				if (!QSLOT_STD0) QSLOT_STD0 = bb;
				else if (!QSLOT_STD1) QSLOT_STD1 = bb;
				else if (!QSLOT_STD2) QSLOT_STD2 = bb;
				else if (!QSLOT_STD3) QSLOT_STD3 = bb;
				else onHold = bb;	// no free slot - put buffer on hold
				if (!onHold) {
					reply_register(bb->dt, bb->adr, DECODERMSG_M3BIN, bb->cb, bb->priv, 500);
					reply_register(bb->dt, bb->adr, DECODERMSG_M3DATA, bb->cb, bb->priv, 500);
					reply_register(bb->dt, bb->adr, DECODERMSG_ERR, bb->cb, bb->priv, 500);
				}
				break;
			default:				// try to put these standard buffers to one of the standard queue positions
				if (!QSLOT_STD0) QSLOT_STD0 = bb;
				else if (!QSLOT_STD1) QSLOT_STD1 = bb;
				else if (!QSLOT_STD2) QSLOT_STD2 = bb;
				else if (!QSLOT_STD3) QSLOT_STD3 = bb;
				else onHold = bb;	// no free slot - put buffer on hold
				if (!onHold) reply_register(bb->dt, bb->adr, DECODERMSG_POM, bb->cb, bb->priv, 500);
				break;
		}
	}
}

/*
 * ========================================================================================
 * Functions and structures for RailCom reporting (all functions will probably be called
 * from interrupt context).
 * ========================================================================================
 */

void sig_rcAck (struct bitbuffer *bb)
{
	if (bb) bb->ack = true;
}

static volatile struct bitbuffer *prev;		///< the last complete packet that was sent out and not yet acknowledged
void sig_bidibACK (void)
{
	irqdbg_printf("%s()\n", __func__);
	if (prev) prev->ack = true;
}

/*
 * ========================================================================================
 * Timer programming
 * ========================================================================================
 */

/**
 * Programm the next timing to TIM1 for implementing the beginning
 * of a Railcom cutout.
 *
 * We immediately switch off the transistors that bridge the railcom
 * detector resistor. We start a short positive edge on the "left"
 * H-bridge leg (the one that is controlled by CH2 of TIM1) and than
 * bring both legs to GND, effectively shorting the track terminals.
 * This is the moment when we can start receiving RailCom answers.
 *
 * Please keep in mind: This timing is prepared when the output of the
 * previous timing is just starting. The real effect of this programming
 * is output by the Hardware when the previous timing ends. Since the
 * previous timing was the terminating 1 bit, it will take approxmately
 * 116µs until it becomes effective.
 *
 * Nevertheless keep in mind, that the user could adjust the timing of
 * DCC one bits between 80µs and 150µs. To keep the railcom cutout to
 * the defined timings, all DCC bits will be output with the standard
 * timing plus a little stretch factor to have a little more time for
 * the railcom cutout window.
 */
static void sig_setCutoutHd (void)
{
	uint16_t ph1, period;

	ph1 = (TIME_DCCPERIOD_ONE >> 1) + 3;	// stretch pulse timing by 3µs
	period = TIME_DCCPERIOD_ONE + 6;		// stretch bit timing by 6µs to elongate the cutoutphase

	TIM1->CCR1 = TIME_CUTOUT_START + 10;	// switch off cutout bridging transistors shortly after cutout physically starts
	TIM1->CCR2 = TIME_CUTOUT_START;			// "left" leg of H bridge is HIGH for first half of the first DCC 1bit pulse
	TIM1->CCR3 = period;					// "right" leg of H bridge stays low for the whole timing
	if (signals & BOOSTER_BIDIB) {
		TIM1->CCR4 = ph1;					// the "pure" signal continues to output DCC 1's (elongated by 1 µs to still be 50% of whole duration)
	} else {
		TIM1->CCR4 = 0;						// BiDiB signal output is silenced
	}
	TIM1->ARR = period - 1;
	TIM1->RCR = 0;							// this startup sequence is output a single time
#ifndef HW_REV07
	MMBOOSTER_CCR = ph1;					// Märklin booster
	DCCBOOSTER_CCR = ph1;					// CDE booster
	MMBOOSTER_ARR = period - 1;
	DCCBOOSTER_ARR = period - 1;
	MMBOOSTER_RCR = 0;
	DCCBOOSTER_RCR = 0;
#endif
}

/**
 * Programm the next timing to TIM1 for implementing the rest (tail part)
 * of a Railcom cutout. At the time this function is called, the TIM1
 * hardware just starts to output the timings and signals that were
 * programmed in sig_setCutoutHd() above. That means, that all the railcom
 * timings are related to the instant, where this function is called
 * (except for interrupt reaction time which should be very short).
 *
 * The railcom receiver windows are controlled by TIM7 in railcom.c.
 * TIM7 timer is started by calling railcomTrigger().
 *
 * \param bb		the current bitbuffer that contains the information that belongs to this cutout
 */
static void sig_setCutoutTail (struct bitbuffer *bb)
{
	uint16_t ph1, period, rpt;

	railcomTrigger(bb);

	ph1 = (TIME_DCCPERIOD_ONE >> 1) + 6;
	period = TIME_DCCPERIOD_ONE + 12;	// the cutout is stretched by 1x6µs + 3x12µs and results in 506µs (including 30µs pulse at the beginning)
	rpt = fmtcfg->dcc.rc_tailbits - 2;	// "- 2": one tailbit is already sent out and the RCR register takes "repeat - 1"

	TIM1->CCR1 = 0;					// switch off cutout bridging transistors
	TIM1->CCR2 = 0;					// "left" leg of H bridge is LOW
	TIM1->CCR3 = period;			// "right" leg of H bridge stays low for the whole timing
	if (signals & BOOSTER_BIDIB) {
		TIM1->CCR4 = ph1;			// the "pure" signal continues to output DCC 1's (elongated by 1 µs to still be 50% of whole duration)
	} else {
		TIM1->CCR4 = 0;				// BiDiB signal output is silenced
	}
	TIM1->ARR = period - 1;			// stretch timing by 2µs per bit to elongate the cutoutphase by 8µs in the end
	TIM1->RCR = rpt;				// specify repetition of this timing (main booster)

#ifndef HW_REV07
	MMBOOSTER_CCR = ph1;			// Märklin booster
	DCCBOOSTER_CCR = ph1;			// CDE booster
	MMBOOSTER_ARR = period - 1;
	DCCBOOSTER_ARR = period - 1;
	MMBOOSTER_RCR = rpt;			// specify repetition of this timing (Märklin booster)
	DCCBOOSTER_RCR = rpt;			// specify repetition of this timing (CDE booster)
#endif
}

/**
 * Programm the next timing to TIM1 for the standard output case. This includes
 * a possible repetition of this one pattern (i.e. for DCC preamble).
 *
 * This function can not be used for the cutout, because CCR1, CCR2 and CCR3
 * must be set to special values!
 *
 * \param ph1		the timing for the first part of the signal (in timer ticks)
 * \param ph2		the timing for the second part of the signal (in timer ticks)
 * \param rpt		number of repetitions this pattern should be output
 */
static void sig_setTiming (int booster, uint16_t ph1, uint16_t ph2, uint16_t rpt)
{
	uint16_t period = ph1 + ph2;

	if (rpt == 0) rpt = 1;
	if (booster & BOOSTER_BUILTIN) {
		TIM1->CCR1 = period;		// the cutout bridging transistors should always be ON
		TIM1->CCR2 = ph1;			// "left" leg of H bridge
		TIM1->CCR3 = ph1;			// "right" leg of H bridge
		if (signals & BOOSTER_BIDIB) {
			TIM1->CCR4 = ph1;		// the "pure" signal (same as "left" leg) without possible cutouts
		} else {
			TIM1->CCR4 = 0;			// silence the BiDiB output
		}
		TIM1->ARR = period - 1;
		TIM1->RCR = rpt - 1;
	}

#ifndef HW_REV07
	if (booster & BOOSTER_MM) {		// Märklin booster
		MMBOOSTER_CCR = ph1;
		MMBOOSTER_ARR = period - 1;
		MMBOOSTER_RCR = rpt - 1;
	}

	if (booster & BOOSTER_CDE) {	// CDE booster
		DCCBOOSTER_CCR = ph1;
		DCCBOOSTER_ARR = period - 1;
		DCCBOOSTER_RCR = rpt - 1;
	}
#endif
}

/*
 * ========================================================================================
 * DCC signal handling (bit level)
 * ========================================================================================
 */

/**
 * Build up the stream by sending the required parts (components) of the packet.
 * Bytes inside the data portion are sent MSB first. Each data byte is precceeded
 * with a '0' start bit.
 *
 * \param bb	the bitbuffer to get the timing data from
 */
static void sig_dccGenerator (int booster, struct bitbuffer *bb)
{
	int idx, bit;

	switch (bb->current_comp) {
		case COMP_DCC_PREAMBLE:			// send out preamble bits
			bb->pos = 0;					// prepare for the data phase
			bb->dcc.startbit = false;		// we must begin with a start bit (0)
			sig_setTiming(booster, (fmtcfg->dcc.tim_one >> 1), (fmtcfg->dcc.tim_one >> 1), bb->dcc.preamble);
			bb->current_comp <<= 1;			// next state
			break;
		case COMP_DCC_DATA:				// send out data bits
			idx = bb->pos >> 3;
			bit = bb->pos & 0x07;
			if (bit == 0 && !bb->dcc.startbit) {
				// gen a startbit ...
				sig_setTiming(booster, (fmtcfg->dcc.tim_zero >> 1), (fmtcfg->dcc.tim_zero >> 1), 1);
				bb->dcc.startbit = true;	// mark, that startbit was sent
				break;
			}
			if (bb->databits[idx] & (0x80 >> bit)) {
				sig_setTiming(booster, (fmtcfg->dcc.tim_one >> 1), (fmtcfg->dcc.tim_one >> 1), 1);
			} else {
				sig_setTiming(booster, (fmtcfg->dcc.tim_zero >> 1), (fmtcfg->dcc.tim_zero >> 1), 1);
			}
			bb->dcc.startbit = false;
			if (++bb->pos >= bb->bits) {
				bb->current_comp <<= 1;		// next state
			}
			break;
		case COMP_DCC_PACKETEND:		// send out an additional tail bit
		case COMP_DCC_TAIL1:			// send out a single tail bit
			sig_setTiming(booster, (fmtcfg->dcc.tim_one >> 1), (fmtcfg->dcc.tim_one >> 1), 1);
			bb->current_comp <<= 1;			// next state
			break;
		case COMP_DCC_CUTOUT_HD:		// begin cutout with one '1' bit
			sig_setCutoutHd();
			bb->current_comp <<= 1;			// next state
			break;
		case COMP_DCC_CUTOUT_TAIL:		// end cutout with one more '1' bit
			sig_setCutoutTail(bb);
			bb->current_comp <<= 1;			// next state
			break;
		case COMP_DCC_TAIL4:			// send out the rest of the tail bits
			sig_setTiming(booster, (fmtcfg->dcc.tim_one >> 1), (fmtcfg->dcc.tim_one >> 1), bb->dcc.tail - 1);
			bb->current_comp <<= 1;			// next state
			break;
	}
}

/*
 * ========================================================================================
 * Märklin/Motorola signal handling (bit level)
 * ========================================================================================
 */
static void sig_mmGenerator (int booster, struct bitbuffer *bb)
{
	int idx, bit;

	switch (bb->current_comp) {
		case COMP_MM_PACKET_GAP:		// start gap
		case COMP_MM_END_GAP:			// end gap
			bb->pos = 0;				// prepare for the data phase
			sig_setTiming(booster, 0, fmtcfg->mm.pause, 1);
			bb->current_comp <<= 1;
			break;
		case COMP_MM_DATA1:				// data packet (1st time)
		case COMP_MM_DATA2:				// data packet (2nd time)
			idx = bb->pos >> 3;
			bit = bb->pos & 0x07;
			if (bb->databits[idx] & (0x80 >> bit)) {	// bit is set, so output a long pulse followed by a short gap
				if (bb->fmt == SIGNAL_MMFAST)
					sig_setTiming(booster, TIME_MMFAST_LONG, TIME_MMFAST_PERIOD - TIME_MMFAST_LONG, 1);
				else
					sig_setTiming(booster, TIME_MMSLOW_LONG, TIME_MMSLOW_PERIOD - TIME_MMSLOW_LONG, 1);
			} else {									// bit is clear, so output a short pulse followed by a long gap
				if (bb->fmt == SIGNAL_MMFAST)
					sig_setTiming(booster, TIME_MMFAST_SHORT, TIME_MMFAST_PERIOD - TIME_MMFAST_SHORT, 1);
				else
					sig_setTiming(booster, TIME_MMSLOW_SHORT, TIME_MMSLOW_PERIOD - TIME_MMSLOW_SHORT, 1);
			}
			bb->pos++;
			if (bb->pos >= bb->bits) bb->current_comp <<= 1;
			break;
		case COMP_MM_REPEAT_GAP:		// inter-frame gap
			if (bb->fmt == SIGNAL_MMFAST)
				sig_setTiming(booster, 0, fmtcfg->mm.interpck_fast, 1);
			else
				sig_setTiming(booster, 0, fmtcfg->mm.interpck_slow, 1);
			bb->pos = 0;				// prepare for the data phase
			bb->current_comp <<= 1;
			break;
	}
}

/*
 * ========================================================================================
 * M3 signal handling (bit level)
 * ========================================================================================
 */

static uint16_t sig_m3NextTiming (struct bitbuffer *bb)
{
	switch (bb->current_comp) {
		case COMP_M3_FLAG1:
			if (!bb->m3.flagcnt) {
				bb->pos = 0;				// prepare for the data phase
				bb->m3.flagcnt = 6;			// the standard start flag (with format interleaving, one flag is sometimes not enough: 6 half flags!)
			}
			/* FALL THRU */
		case COMP_M3_FLAG2:
			if (!bb->m3.flagcnt) bb->m3.flagcnt = 22;		// 11 flags before reply window (22 half flags)
			/* FALL THRU */
		case COMP_M3_FLAG3:
			if (!bb->m3.flagcnt) bb->m3.flagcnt = 2;		// the delimiter flag between the reply windows for 1-bit replies (2 half flags)
			/* FALL THRU */
		case COMP_M3_FLAG4:
			if (!bb->m3.flagcnt) {		// additional flag for ending a reply window with two flags (2 half flags)
				bb->m3.flagcnt = 2;
				m3reply_disable(bb);
			}
			/* FALL THRU */
		case COMP_M3_ENDFLAG:
			if (!bb->m3.flagcnt) bb->m3.flagcnt = 2;		// the standard single end flag (2 half flags)
			switch (++bb->m3.flagphase) {
				case 1:
					return TIME_M3PERIOD0;
				case 2:
					return TIME_M3PERIOD1;
				case 3:
					if (--bb->m3.flagcnt == 0) {
						bb->current_comp <<= 1;
					}
					bb->m3.flagphase = 0;
					return TIME_M3PERIOD0;
				default:	// ??? this should not happen!
					bb->m3.flagcnt = bb->m3.flagphase = 0;
					break;
			}
			break;		// will not be reached - every case results in a return (except if default: may be reached in error!)
		case COMP_M3_DATA:
			if (bb->m3.onebits >= 8) {
				bb->m3.onebits = 0;
				if (bb->pos >= bb->bits) bb->current_comp <<= 1;		// switch to next component phase
				return TIME_M3PERIOD0;		// bit stuffing: add a '0' bit after eight contiguous '1' bits
			}
			if (bb->databits[bb->pos >> 3] & (0x80 >> (bb->pos & 0x07))) {		// a '1' bit
				bb->m3.halfbit = !bb->m3.halfbit;
				if (!bb->m3.halfbit) {		// second part of the '1' bit, advance to next bit
					bb->m3.onebits++;
					// switch to next component phase (not, if we just arrived at 8 '1' bits - in this case we must add a final stuff bit)
					if (++bb->pos >= bb->bits && bb->m3.onebits < 8) bb->current_comp <<= 1;
				}
				return TIME_M3PERIOD1;
			} else {															// a '0' bit
				bb->m3.onebits = 0;
				bb->m3.halfbit = false;
				if (++bb->pos >= bb->bits) bb->current_comp <<= 1;		// switch to next component phase
				return TIME_M3PERIOD0;
			}
			break;		// will not be reached - every case results in a return
		case COMP_M3_REPLYSTART:				// generate the 0b0011 pattern
			if (!bb->m3.flagcnt) bb->m3.flagcnt = 6;
			switch (--bb->m3.flagcnt) {		// we count backwards, so 4 and 5 are the zero bits and the rest are half '1' bits
				case 0:		// end of sequence - advance to next component phase
					bb->current_comp <<= 1;
					/* FALL THRU */
				case 1:
				case 2:
				case 3:
					return TIME_M3PERIOD1;
				case 4:
				case 5:
					return TIME_M3PERIOD0;
			}
			break;		// will not be reached - every case results in a return
		case COMP_M3_REPLYWIN1:
			bb->current_comp <<= 1;
			m3reply_enable(bb->dt, bb->adr, bb->rdt, bb->cva, bb->param);
			return TIME_M3REPLYWINDOW;
		case COMP_M3_REPLYWIN2:
			bb->current_comp <<= 1;
			m3reply_disable(bb);
			return TIME_M3REPLYWINDOW;
		case COMP_M3_REPLYSYNC:
			if (!bb->m3.flagcnt) {				// 23 sync impulses, even counts generate short pulses, odd ones long pauses
				bb->m3.flagcnt = 46;
				m3reply_enable(bb->dt, bb->adr, bb->rdt, bb->cva, bb->param);
			}
			if (--bb->m3.flagcnt == 0) bb->current_comp <<= 1;		// reached last sync pulse
			if (bb->m3.flagcnt & 1) return TIME_M3_RDS_PERIOD - TIME_M3_RDSMARKER;
			return TIME_M3_RDSMARKER;
		case COMP_M3_REPLYDATA:
			if (!bb->m3.flagcnt) bb->m3.flagcnt = bb->m3.replybits * 4 + 1;	// two pulses per bit, two phases per pulse - see above (even/odd now swapped)!
			if (--bb->m3.flagcnt == 0) bb->current_comp <<= 1;		// the last sync pulse
			if (bb->m3.flagcnt & 1) return TIME_M3_RDSMARKER;
			return (TIME_M3_RDS_PERIOD / 2) - TIME_M3_RDSMARKER;
		case COMP_M3_FILLBIT:		// the frame is finished, but we may need another filler time for the LOW portion of the PWM (ph2)
			return TIME_M3PERIOD1;
	}
	// this code should never be reached ... just in case ...
	bb->current_comp <<= 1;
	return TIME_M3PERIOD0;
}

static void sig_m3Generator (int booster, struct bitbuffer *bb)
{
	uint16_t ph1, ph2;

	ph1 = sig_m3NextTiming(bb);
	// if the first timing advanced the component, we must skip all non-set components!
	while (bb->current_comp && ((bb->components & bb->current_comp) == 0)) bb->current_comp <<= 1;
	ph2 = sig_m3NextTiming(bb);
	sig_setTiming(booster, ph1, ph2, 1);
}

/*
 * ========================================================================================
 * Interrupt handler (block oriented view, calls the format individual generator funcs).
 * The interrupt is called, when the previously programmed timing starts it's execution (double buffering!).
 * ========================================================================================
 */
void TIM1_UP_IRQHandler (void)
{
	static struct bitbuffer *bb = NULL;
	static int slot, xpomslot;

//	int booster = 0;
	bool last_was_mm = false;

    BaseType_t xHigherPriorityTaskWoken = 0;

	TIM1->SR = 0;		// clear all interrupt flags
//	if (TIM1->CR1 & TIM_CR1_CEN) booster |= BOOSTER_BUILTIN;
//	if (TIM3->CR1 & TIM_CR1_CEN) booster |= BOOSTER_MM;
//	if (TIM8->CR1 & TIM_CR1_CEN) booster |= BOOSTER_CDE;

	if (bb) {	// if we still have a packet, check for complete status
		// advance to next set component (i.e. skip over unset component bits)
		while (bb->current_comp && ((bb->components & bb->current_comp) == 0)) bb->current_comp <<= 1;

		// check if the current packet is done - repeat it or fetch a new one
		if (bb->current_comp == 0) {	// all components are sent out - check for acknowledgement and other stuff
			last_was_mm = (bb->fmt == SIGNAL_MMSLOW) || (bb->fmt == SIGNAL_MMFAST);		// if we ended with a MM-GAP we don't need starting GAP
			bb->repeat--;
			if (bb->ack || (bb->repeat <= 0)) {			// we can free that slot and request a fillup from foreground thread
				prev = NULL;							// already acknowledged or too late
				if (bb == QSLOT_DCCA) QSLOT_DCCA = NULL;
				else if (bb == QSLOT_XPOM00) QSLOT_XPOM00 = NULL;
				else if (bb == QSLOT_XPOM01) QSLOT_XPOM01 = NULL;
				else if (bb == QSLOT_XPOM10) QSLOT_XPOM10 = NULL;
				else if (bb == QSLOT_XPOM11) QSLOT_XPOM11 = NULL;
				else if (bb == QSLOT_STD0) QSLOT_STD0 = NULL;
				else if (bb == QSLOT_STD1) QSLOT_STD1 = NULL;
				else if (bb == QSLOT_STD2) QSLOT_STD2 = NULL;
				else if (bb == QSLOT_STD3) QSLOT_STD3 = NULL;
				vTaskNotifyGiveFromISR (SIGtask, &xHigherPriorityTaskWoken);	// trigger the upper signal generator instance to supply a new packet
			} else {
				prev = bb;					// prepare for BiDiBus-ACK
			}
			bb = NULL;		// force grabbing a new buffer
		}
	}

	if (!bb) {		// select a new buffer
		if (QSLOT_DCCA) {					// top priority - DCC-A is always delivered as fast as possible
			bb = QSLOT_DCCA;
//			irqdbg_printf("%s() DCC-A slot 0x%02x 0x%02x 0x%02x\n", __func__, bb->databits[0], bb->databits[1], bb->databits[2]);
		} else if (sig_countXpomSlots()) {	// second priority - XPOM should be sent fast
			do {
				if (queue.xpom[xpomslot]) bb = queue.xpom[xpomslot];
				if (++xpomslot >= DIM(queue.xpom)) xpomslot = 0;
			} while (!bb);
//			irqdbg_printf("%s() XPOM slot\n", __func__);
		} else if (sig_countStdSlots()) {	// lowest priority - check for anything else to do
			do {
				if (queue.microqueue[slot]) bb = queue.microqueue[slot];
				if (++slot >= DIM(queue.microqueue)) slot = 0;
			} while (!bb);
//			irqdbg_printf("%s() standard slot\n", __func__);
		} else {							// no valid buffer in any slot, so we must send some idle information
			bb = &idle;
			if (rt.tm == TM_DCCPROG) bb = &reset;
			else if (rt.tm == TM_TAMSPROG) bb = &mmidle;
			bb->repeat = 1;
//			irqdbg_printf("%s() idle slot\n", __func__);
			vTaskNotifyGiveFromISR (SIGtask, &xHigherPriorityTaskWoken);	// trigger the upper signal generator instance to supply a new packet
		}
		bb->current_comp = 1;					// we restart with component 1
		bb->pos = 0;							// prepare for the data phase
		bb->ack = false;
		if (bb->fmt == SIGNAL_M3) {
			bb->m3.flagcnt = 0;
			bb->m3.flagphase = 0;
			bb->m3.onebits = 0;
			bb->m3.halfbit = false;
		} else if (((bb->fmt == SIGNAL_MMSLOW) || (bb->fmt == SIGNAL_MMFAST)) && last_was_mm) {		// both this and the last one were MM
			bb->current_comp <<= 1;				// skip over starting gap
		}
	}

	if (bb) {
		switch (bb->fmt) {
			case SIGNAL_DCC:
			case SIGNAL_DCC_XPOM00:
			case SIGNAL_DCC_XPOM01:
			case SIGNAL_DCC_XPOM10:
			case SIGNAL_DCC_XPOM11:
			case SIGNAL_DCC_A:
				sig_dccGenerator(signals, bb);
				break;
			case SIGNAL_MMSLOW:
			case SIGNAL_MMFAST:
				sig_mmGenerator(signals, bb);
				break;
			case SIGNAL_M3:
				sig_m3Generator(signals, bb);
				break;
			case SIGNAL_UNKNOWN:
				bb->current_comp = 0;	// this terminates the packet immedeately
				break;
		}
	} else {		// this should never happen, because we - at least - have the idle packet
		sig_setTiming(signals, 0, 500, 1);
	}

    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}

#ifndef HW_REV07
/**
 * Software emulation of the Repeat-Count-Register (RCR) for TIM3.
 *
 * The timers for signal generation use shadow registeres to make timer updates less critical.
 * The timer hardware internally works with shadow registers and the application only writes to
 * "preload" registers. At the end of a timer run (that is when the timer reaches its maximum
 * count and resets the CNT register to zero) it copies the "application" registers to the
 * shadow registers (for ARR, CCxR and RCR) and works with these registers for the next round.
 *
 * The repeat count register (RCR) is not available on TIM3, so we have to emulate both the
 * "application" and the "shadow" register in software. We must suppress the shadow update, as
 * long as the RCR-shadow has a value > 0. This can be done by setting the UDIS (Update DISable)
 * bit in CR1 of TIM3. We manage this emulation in the Capture-Compare interrupt for channel 3
 * of TIM3.
 *
 * Timing: The main timing is programmed from TIM1 update interrupt (see TIM1_UP_IRQHandler()).
 * This interrupt fires after TIM1 has finished his update event (transferring application registers
 * to their shadow counterparts) and calculates new values for the CCxR and ARR registers (ARR =>
 * Automatic Reload Register = total count before reset). It also computes a repeat count and
 * stores it in the appropriate RCR. These values get in effect at the next update interrupt.
 *
 * For TIM3 we use the compare interrupt. It triggers at some point between two successive update
 * events (for a DCC signal, this will always be the exact midpoint). When this interrupt runs,
 * the new values for the application registeres have already been set and if we do nothing else,
 * they will become effective (i.e. copied to the shadow registers) at the next update event.
 * We emulate an RCR shadow register, that disables the update if greater than zero. If it becomes
 * zero, update is enabled again and the emulated application RCR is copied to the emulated shadow
 * RCR as if it would be done by the hardware at the next update event.
 */
void TIM3_IRQHandler (void)
{
	static uint16_t RCR_shadow;

	if (TIM3->SR & TIM_SR_CC3IF) {
		if (RCR_shadow > 0) {
			SET_BIT(TIM3->CR1, TIM_CR1_UDIS);
			RCR_shadow--;
		} else {
			CLEAR_BIT(TIM3->CR1, TIM_CR1_UDIS);
			RCR_shadow = TIM3_RCR;
		}
	}

	TIM3->SR = 0;		// clear all interrupt flags
}
#endif	/* !HW_REV07 */

void TIM8_UP_TIM13_IRQHandler (void)
{
	// Handle TIM13 update event
	if ((TIM13->DIER & TIM_DIER_UIE) && (TIM13->SR & TIM_SR_UIF)) {
		TIM13->SR &= ~TIM_SR_UIF;
		tim13_updateIRQ();		// call the real handler, located in HW/spi.c
	}

#ifndef HW_REV07
	// if ((TIM8->DIER & TIM_DIER_xxx) && (TIM8->SR & TIM_SR_xxx)) {
		TIM8->SR = 0;		// clear all interrupt flags
	// }
#endif	/* !HW_REV07 */

	NVIC_ClearPendingIRQ (TIM8_UP_TIM13_IRQn);
}

/**
 * \}
 */
