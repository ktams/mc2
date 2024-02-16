/*
 * feebdback.c
 *
 *  Created on: 10.07.2021
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

/**
 * \file
 * \brief Feedback handling from the layout
 *
 * "Feedback" is simple Free/Occupied information and so implemented as a single
 * bit per feedback input. Some special handling is needed for protocols that need
 * to not only know, what the current status is but also the information, if something
 * changed and if there was a short occupied state between two polls. This must be
 * handled within these protocols. Here we only gather the information and send
 * events for every change.
 *
 * Feedback from the various system busses is consolidated here. For the sake
 * of the old P50x protocol, we will combine all feedback information in a single
 * bitarray holding the information of up to MAX_FEEDBACK (currently 64k) feedback
 * inputs (using 64k @ 8 Bits per Byte = 8kB).
 *
 * The supported busses are:
 *   - classical s88 with 16 input bits per module, can form strings of infinit length
 *   - LocoNet feedback modules covering up to 4096 bits (the equivalent of 256 s88 modules)
 *   - CAN modules with up to 16 bits each and an address space of 64k bits
 *   - BiDiB modules with up to 128 bits each, not bound to any linear address which
 *     will get a system internal mapping address
 *
 * All types of modules share the same "address space" with the s88 system beeing
 * the only one, that has no idea of an individual address. So, any s88 modules connected
 * to the system via the s88 bus are serving feedback bits from address zero onwards.
 * The configuration of the supported number of s88 modules "reserves" the equivalent
 * number of bits.
 *
 * All systems may report bits in any of their supported ranges and thereby may
 * overlap. It is the responsibility of the end user to configure the module
 * addresses in a way, that such an overlap is not happening.
 *
 * To be precise, BiDiB doesn't support the idea of an address either. In the
 * BiDiB system, everything is connected via the UID of the respective device.
 * Therefore we have to connect BiDiB feedback modules to "module addresses"
 * by configuration. Any module that is not configured cannot be part of this
 * feedback abstraction but will of course operate normal in the BiDiB system.
 *
 * Because the LocoNet devices are limited to a 4K address space and the s88 bus
 * serves addresses from zero, the LocoNet modules should only use the address space
 * beyond the s88 modules. If there are more than 255 s88 modules, the address space
 * of the LocoNet is exhausted and no such modules can be used.
 *
 * The other side of this feedback information are the protocols, that work with the
 * information gathered from the inputs. These protocols have limits of their own.
 * The systems we currently support are:
 *	 - P50 as the original Maerklin protocol can handle up 31 moduls of 16 bits (496 bits)
 *	 - P50x is the IntelliBox version of this protocol and supports up to 255 modules (4080 bits)
 *	 - BiDiB can support any number of virtual BiDiB-modules that can report up to 256 bits each
 *	 - EasyNet currently does not support feedback (it was only there for P50x support)
 *	 - LocoNet ???
 *	 - z21 can report occupancy with the LAN_LOCONET_DETECTOR in a full 16-bit range (64k)
 *	 - z21 (R-Bus) can handle two groups of 10 modules with 8 bits each (160 bits)
 */

#include <string.h>
#include "rb2.h"
#include "events.h"
#include "bidib.h"

#define MAX_MODCOUNT	((MAX_FEEDBACKS + 15) / 16)	///< number of classical s88 modules that would represent the total number of feeback bits

static SemaphoreHandle_t mutex;						///< we lock the access to the feedback array
static uint16_t feedback[MAX_MODCOUNT];				///< all feedback bits in 16-bit units (s88 size)

/**
 * A short helper function to really send the event.
 * The memory for this event (the src member in the finally proagated
 * event structure) is allocated from temporary space.
 *
 * \param module		the 0-based (s88-)module number as an address, where the change occured
 * \param st			the 16 bit status word that results from the reported change
 * \param chg			a bit for every changed feedback bit
 */
static void fb_sendevent (int module, uint16_t st, uint16_t chg)
{
	fbeventT *evt;

	if ((evt = tmpbuf (sizeof(*evt))) != NULL) {
		evt->module = module;
		evt->status = st;
		evt->chgflag = chg;
		event_fireEx(EVENT_FBNEW, 0, evt, 0, 20);
	}
}

/**
 * Insert new data (feedbacks) into the system. Any offset and any number of bits may be presented.
 * To make that clear: the offset may also point to any bit inside 16-bit s88 words.
 * The data bits are organised as feedback #1 in the LSB and feedback #8 in the MSB of a byte.
 * The feedback array itself is organised with a different ordering based on 16-bit values:
 * feedback #1 is the MSB (bit 15, 0x8000) and Feedback #16 is the LSB (bit 0, 0x0001).
 *
 * An example with offset 11 and len 3 bits:
 *    bit 0 of data[0] -> bit 5 feedback[0]
 *    bit 1 of data[0] -> bit 4 feedback[0]
 *    bit 2 of data[0] -> bit 3 feedback[0]
 *
 * \param offset			the bit/feedback offset in the array (0-based: 0-15 = s88 #1, 16-31 = s88 #2, ...)
 * \param len				the number of reported feedback bits (i.e. bits to overwrite)
 * \param data				the data bytes with the LSB of the first byte representing the new status of the feedback at address 'offset'
 */
static void _fb_update (int offset, int len, uint8_t *data)
{
	uint16_t *ar, armask;
	uint8_t datamask;
	int midx;
	uint16_t changed;

	if (len <= 0 || !data) return;			// nothing to do - ignore the call
	if (offset < 0) return;					// a negative offset is not allowed
	if (offset >= MAX_FEEDBACKS) return;	// beyond array - ignore the data
	if ((offset + len) >= MAX_FEEDBACKS) {	// length + offset reach beyond array - trim length down to end of array
		len = MAX_FEEDBACKS - offset;
	}

	midx = offset / 16;
	ar = &feedback[midx];
	armask = 0x8000 >> (offset % 16);
	datamask = 1;
	changed = 0;
	while (len > 0) {
		if (armask == 0) {		// right SHIFT operation beyond bit 0
			if (changed) {
				fb_sendevent(midx, *ar, changed);
				changed = 0;
			}
			ar++;
			armask = 0x8000;
			midx++;
		}
		if (datamask == 0) {	// left SHIFT operation beyond bit 7
			data++;
			datamask = 1;
		}
		if (!!(*ar & armask) != !!(*data & datamask)) {		// BOOLEAN compare!
			*ar &= ~armask;
			if (*data & datamask) *ar |= armask;
			changed |= armask;
		}
		armask >>= 1;
		datamask <<= 1;
		len--;
	}
	if (changed) fb_sendevent(midx, *ar, changed);		// if anything was altered in the last module
}

/**
 * Get the bits from a complete module (uint16_t, MSB is feedback #1, LSB is feedback #16)
 *
 * \param mod		the module in range 0 .. MAX_MODCOUNT - 1
 * \return			the 16 bits that make up the feedback status of the s88 module
 */
uint16_t fb_getModuleState (int mod)
{
	if (mod < 0 || mod >= MAX_MODCOUNT) return 0;
	return feedback[mod];
}

/**
 * Get the bits from a half module (uint8_t, MSB is feedback #1/#9, LSB is feedback #8/#16)
 *
 * \param hmod		the half module index in range 0 .. (MAX_MODCOUNT * 2) - 1
 * \return			the 8 bits that make up the feedback status of the s88 half-module
 */
uint8_t fb_getHalfModuleState (int hmod)
{
	if (hmod < 0 || hmod >= MAX_MODCOUNT * 2) return 0;
	if (hmod & 1) return feedback[hmod >> 1] & 0xFF;
	return feedback[hmod >> 1] >> 8;
}

/**
 * Revert the bit ordering for feedback from MSB -> LSB = #1 -> #8
 * to LSB -> MSB = #1 -> #8 feedback bit.
 *
 * \param b		the byte to reverse it's bits
 * \return		the reversed byte
 */
uint8_t fb_msb2lsb8 (uint8_t b)
{
	static const uint8_t nibbles[] = {	0b0000, 0b1000, 0b0100, 0b1100, 0b0010, 0b1010, 0b0110, 0b1110,
										0b0001, 0b1001, 0b0101, 0b1101, 0b0011, 0b1011, 0b0111, 0b1111 };

	return (nibbles[b & 0x0F] << 4) | nibbles[b >> 4];
}

/**
 * Revert the bit ordering for feedback from MSB -> LSB = #1 -> #16
 * to LSB -> MSB = #1 - #16 feedback bit.
 *
 * \param b		the short integer to reverse it's bits
 * \return		the reversed word
 */
uint16_t fb_msb2lsb16 (uint16_t w)
{
	return (fb_msb2lsb8(w & 0xFF) << 8) | fb_msb2lsb8(w >> 8);
}

/**
 * Read in the current status of the s88 system.
 * s88 begins always at fixed module address zero!
 * The data organisation corresponds to our feedback organisation, because s88
 * was the primary source of bit-organisation. So we have a 1:1 correspondance
 * of s88 bits to storage in feedback bits array.
 *
 * \param modules	number of valid input words (aka. number of s88 modules)
 * \param data		the bit array as 16-bit words
 */
void fb_s88input (int modules, uint16_t *data)
{
	int mod;
	uint16_t change;

	if (mutex_lock(&mutex, 40, __func__)) {
		for (mod = 0; mod < modules; mod++) {
			if (feedback[mod] != data[mod]) {
				change = feedback[mod] ^ data[mod];
				feedback[mod] = data[mod];		// write back changed value
				fb_sendevent(mod, feedback[mod], change);
			}
		}
		mutex_unlock(&mutex);
	}
}

/**
 * Support for single bit reports as occupied / freed for systems that support
 * this type of message:
 *	- LocoNet sensors report single bit events in the range of 4K feedback bits.
 *	- CAN modules are programmed to a specific base address and report single
 *	  feedback events of up to 16 bits per Module. They can reach up to 64K bits.
 *	- BiDiB modules can be mapped to any address and may have up to 128 feedback
 *	  inputs. Most of the time they will only report single events, though.
 *
 * \param adr		the feedback bit address reported by the module (0-based)
 * \param occupy	the binary state of this input
 */
void fb_BitInput (int adr, bool occupy)
{
	uint16_t data, mask;
	int module;

	module = adr >> 4;
	if (module < 0 || module >= MAX_MODCOUNT) return;
	mask = 0x8000 >> (adr & 0x0F);
//	log_msg (LOG_INFO, "%s() ADR/MOD %d/%d state %s\n", __func__, adr, module, occupy ? "OCCUPY" : "FREE");

	if (mutex_lock(&mutex, 40, __func__)) {
		data = feedback[module];			// read current status
		if (occupy) data |= mask;
		else data &= ~mask;
		if (data != feedback[module]) {		// send an event only if a change is detected
			feedback[module] = data;		// write back changed value
			fb_sendevent(module, feedback[module], mask);
		}
		mutex_unlock(&mutex);
	}
}

/**
 * Insert a block of new data (feedbacks) into the system. Any offset and any
 * number of bits may be presented. This function is a wrapper to aquire the
 * mutex.
 *
 * \param offset			the bit/feedback offset in the array (0-based: 0-15 = s88 #1, 16-31 = s88 #2, ...)
 * \param len				the number of reported feedback bits (i.e. bits to overwrite)
 * \param data				the data bytes with the LSB of the first byte representing the new status of the feedback at address 'offset'
 * \see						_fb_update()
 */
void fb_rangeInput (int offset, int len, uint8_t *data)
{
	if (mutex_lock(&mutex, 40, __func__)) {
		_fb_update(offset, len, data);
		mutex_unlock(&mutex);
	}
}
