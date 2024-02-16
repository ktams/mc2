/*
 * m3_config.c
 *
 *  Created on: 28.12.2021
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
#include "config.h"
#include "decoder.h"

enum blkType {
	BLK_ENDLIST = 0x00,				///< this block type denotes the end of the block listing (i.e. should never show up on real decoders)
	BLK_DESCRIPTION = 0x01,			///< description of the configuration, this is always located at CV 0
	BLK_FUNCTIONS = 0x02,			///< function related settings
	BLK_AUTOMATICFUNC = 0x03,		///< automatic functions
	BLK_FUNCMAPPING = 0x04,			///< mapping of functions to hardware
	BLK_MOTORCONFIG = 0x05,			///< setting of motor related parameters
	BLK_HARDWARE = 0x06,			///< hardware functions
	BLK_PROTOCOL = 0x07,			///< additional protocol definitions
	BLK_SOUND = 0x08,				///< sound definitions
	BLK_EXTRAOPTIONS =0x09,			///< additional options
	BLK_M3PLUS = 0x0A,				///< settings for the mfx+(R) functions
};

enum qState {	// the query state
	QSTAT_BLOCKHEAD0 = 0,			///< reading the block header (first 4 bytes, CVx.0 should be the block descriptor with CA-Type 0x01)
	QSTAT_BLOCKHEAD1,				///< reading the block header (next 2 bytes)
	QSTAT_CADATA,					///< read CA data
};

struct funcMapping {
	uint8_t				 group;		///< function group
	uint8_t				 s1;		///< detail symbol information S1
	uint8_t				 s2;		///< detail symbol information S2
	uint8_t				 symbol;	///< the mapped symbol (if > 0)
};

static const struct funcMapping fmap[] = {
	// functions without group information
	{ 0x00, 0x20, 0x00, 9 },		// automatic coupling (Telex)
	{ 0x00, 0x20, 0x40, 9 },		// automatic coupling (Telex), rear
	{ 0x00, 0x20, 0x80, 9 },		// automatic coupling (Telex), front
	{ 0x00, 0x21, 0x00, 10 },		// smoke generator
	{ 0x00, 0x22, 0x00, 11 },		// panthograph
	{ 0x00, 0x22, 0x40, 11 },		// panthograph, rear
	{ 0x00, 0x22, 0x80, 11 },		// panthograph, front
	{ 0x00, 0x23, 0x00, 12 },		// high beam
	{ 0x00, 0x23, 0x40, 12 },		// high beam (rear)
	{ 0x00, 0x23, 0x80, 12 },		// high beam (front)
	{ 0x00, 0x24, 0x00, 13 },		// bell
	{ 0x00, 0x25, 0x00, 14 },		// horn
	{ 0x00, 0x26, 0x00, 15 },		// whistle
	{ 0x00, 0x28, 0x00, 17 },		// fan
	{ 0x00, 0x28, 0x01, 17 },		// fan
	{ 0x00, 0x29, 0x00, 42 },		// compressor / pump
	{ 0x00, 0x29, 0x01, 45 },		// air pump
	{ 0x00, 0x29, 0x04, 60 },		// injector
	{ 0x00, 0x29, 0x05, 46 },		// water pump
	{ 0x00, 0x2A, 0x00, 18 },		// shovelling coals
	{ 0x00, 0x2B, 0x00, 22 },		// crane arm up/down
	{ 0x00, 0x2B, 0x01, 22 },		// crane arm up/down
	{ 0x00, 0x2B, 0x02, 22 },		// crane arm up/down
	{ 0x00, 0x2B, 0x03, 25 },		// crane arm turn
	{ 0x00, 0x2B, 0x04, 25 },		// crane arm turn
	{ 0x00, 0x2B, 0x07, 23 },		// crane hook up/down
	{ 0x00, 0x2B, 0x08, 23 },		// crane hook up/down

	// unspecific
	{ 0x02, 0x02, 0x00, 0 },		// unspecific function - no icon

	// loco lighting
	{ 0x03, 0x03, 0x00, 1 },		// Forehead lights
	{ 0x03, 0x03, 0x40, 34 },		// Driver's Cab lighting rear
	{ 0x03, 0x03, 0x80, 3 },		// Driver's Cab lighting front
	{ 0x03, 0x03, 0xC0, 3 },		// Driver's Cab lighting front + rear
	{ 0x03, 0x23, 0x00, 12 },		// high beam

	// other lighting
	{ 0x04, 0x04, 0x00, 2 },		// Interior lighting

	// technical lighting
	{ 0x05, 0x05, 0x00, 74 },		// general lighting
	{ 0x05, 0x05, 0x02, 20 },		// train destination display
	{ 0x05, 0x05, 0x03, 24 },		// engine lighting
	{ 0x05, 0x05, 0x09, 53 },		// firebox

	// technical functions
	{ 0x06, 0x20, 0x00, 9 },		// automatic coupling (Telex)
	{ 0x06, 0x21, 0x00, 10 },		// smoke generator
	{ 0x06, 0x22, 0x00, 11 },		// panthograph

	// sound
	{ 0x07, 0x07, 0x00, 5 },		// general noise
	{ 0x07, 0x07, 0x01, 21 },		// breaks squealing
	{ 0x07, 0x07, 0x02, 11 },		// panthograph sound
	{ 0x07, 0x07, 0x05, 28 },		// coupling noise
	{ 0x07, 0x07, 0x06, 29 },		// rail bump
	{ 0x07, 0x07, 0x08, 32 },		// Conductor's whistle
	{ 0x07, 0x07, 0x09, 73 },		// switching stage (electric locomotive)
	{ 0x07, 0x07, 0x0A, 43 },		// releasing compressed air
	{ 0x07, 0x07, 0x0D, 40 },		// stoker
	{ 0x07, 0x07, 0x0E, 67 },		// generator
	{ 0x07, 0x07, 0x0F, 33 },		// buffer shock
	{ 0x07, 0x07, 0x10, 40 },		// stoker
	{ 0x07, 0x07, 0x11, 5 },		// general noise
	{ 0x07, 0x07, 0x13, 26 },		// Releasing steam (cylinder steam)
	{ 0x07, 0x07, 0x14, 4 },		// engine noise
	{ 0x07, 0x07, 0x2E, 55 },		// sanding
	{ 0x07, 0x24, 0x00, 13 },		// bell
	{ 0x07, 0x25, 0x00, 14 },		// horn
	{ 0x07, 0x26, 0x00, 15 },		// whistle
	{ 0x07, 0x27, 0x00, 16 },		// open doors
	{ 0x07, 0x27, 0x01, 16 },		// close doors

	// announcements
	{ 0x09, 0x09, 0x00, 6 },		// station announcement
	{ 0x09, 0x09, 0x01, 6 },		// station announcement
	{ 0x09, 0x09, 0x02, 6 },		// station announcement
	{ 0x09, 0x09, 0x04, 6 },		// station announcement
	{ 0x09, 0x09, 0x05, 6 },		// station announcement

	// behavioral stuff
	{ 0x0A, 0x0A, 0x00, 7 },		// shunting gear
	{ 0x0A, 0x0A, 0x01, 7 },		// shunting gear
	{ 0x0B, 0x0B, 0x00, 8 },		// start-up/braking deceleration

	// end of list
	{ 0xFF, 0x00, 0x00, 0 },		// the first byte otherwise never has a set bit (masked out, means momentary function)
};

static const struct funcMapping fmapGroups[] = {
	{ 0x03, 0x00, 0x00, 1 },		// head- / taillight generic
	{ 0x04, 0x00, 0x00, 2 },		// generic lighting (inside)
	{ 0x05, 0x00, 0x00, 2 },		// generic lighting (outside)
	{ 0x07, 0x00, 0x00, 5 },		// generic sound
	{ 0x08, 0x00, 0x00, 5 },		// generic sound (background)
	{ 0x09, 0x00, 0x00, 5 },		// generic sound (speech)
	{ 0x0A, 0x00, 0x00, 7 },		// shunting gear
	{ 0x0B, 0x00, 0x00, 8 },		// start-up/braking deceleration

	// end of list
	{ 0xFF, 0x00, 0x00, 0 },		// the first byte otherwise never has a set bit (masked out, means momentary function)
};

struct decoder {
	int					 adr;		///< the address of the decoder
	ldataT				*l;			///< the loco that we deal with
	struct stepStack	*stack;		///< the stack of actions to handle
	TaskHandle_t		 caller;	///< a thread that is waiting for the outcome - send it a notification (return code of operation)
	cvadrT				 cva;		///< the current (or next) CV to read
	uint8_t				 ca[64];	///< the current CA that is read in
	int					 caLen;		///< for CAs that are read linear, this is the currently read in length
	int					 retry;		///< count the retries on single reads and stop if this gets too high
	int					 rdLen;		///< maximum number of bytes to read in a single command
	int					 wrLen;		///< maximum number of bytes to write in a single command
	struct block		*blocks;	///< all blocks, the first one is the root information
	uint8_t				 manufacturer;	///< the manufacturer of teh decoder (as DCC manufacturer ID, 0x83=Märklin, 0x97=ESU)

	// $$$$$ old stuff
	struct block		*curBlock;	///< the block we are currently reading
	struct group		*curGroup;	///< the group we are currently reading
	struct ca			*curCa;		///< the CA we are currently reading
	uint8_t				*blkList;	///< pointer to the data array containing the block numbers
	enum qState			 stat;		///< the current status of query
};

struct step {
	int (*func)(struct decoder *, int param);
	int					 param;
};

struct stepStack {
	struct stepStack	*next;		///< a linked list of stacked steps
	const struct step	*step;		///< the currently handled step
};

struct CAdef {
	uint8_t				 ca;
	const char			*name;		///< a descriptive name
	int					 bytes;		///< if > 0 (i.e. not variable) read that many bytes, if 0 the length is variable (like name and block list)
	int (*reader)(struct ca *);		///< a function that tells us the next length to read or zero if this CA is done
	void (*interpreter)(struct ca *, int);	///< a function that prints the interpretion of this CA's data
};

struct BLKdef {
	enum blkType		 blktype;
	const char			*name;		///< a descriptive name
	const struct CAdef	*cas;		///< an array
};

struct ca {
	struct ca			*next;		///< linked list of CAs
	const struct CAdef	*caDesc;	///< the descriptor for that CA
	int					 cv;		///< the CV of this CA
	int					 len;		///< the number of bytes read for this CA
	uint8_t				 caType;	///< the type of this CA (meaning varies, except for caType=0x01 which is the block descriptor)
	uint8_t				 data[63];	///< a CA may contain up to 63 data bytes plus the caType byte, which is always at CVx.0
};

struct group {
	struct group		*next;		///< linked list of groups
	int					 cv;		///< the CV where this group begins
	struct ca			*cas;		///< the CAs that are belonging to this group
};

struct block {
	struct block		*next;		///< linked list of blocks
	const struct BLKdef	*blkDesc;	///< a pointer to the block description
	struct group		*groups;	///< list of groups in this block
	int					 cv;		///< the CV where this block begins
	enum blkType		 bt;		///< the type of the block
	uint8_t				 version;	///< the version (should be 0x01)
	uint8_t				 grpCount;	///< number of groups in this block
	uint8_t				 caPerGrp;	///< number of CAs per group (1+grpCount*caPerGrp is the total amount of CAs this block is using)
};

// forward declarations
static bool m3_cvCallback (struct decoder_reply *msg, flexval priv);
static bool m3_funcCallback (struct decoder_reply *msg, flexval priv);
static void m3_cvReader (struct decoder *d, cvadrT cva, int bytes);

static void pushSteps (struct decoder *d, const struct step *steps)
{
	struct stepStack *stack;

	if (!d || !steps) return;

	if ((stack = malloc(sizeof(*stack))) != NULL) {
		stack->step = steps;
		stack->next = d->stack;
		d->stack = stack;
	}
}

static void popSteps (struct decoder *d)
{
	struct stepStack *stack;

	if (!d || !d->stack) return;
	stack = d->stack;
	d->stack = stack->next;
	free (stack);
}

static int endStepList (struct decoder *d, int param)
{
	(void) param;

	popSteps(d);
	return 0;
}

static struct block *m3_getBlock (struct decoder *d, enum blkType t)
{
	struct block *b;

	if (!d) return NULL;

	b = d->blocks;
	while (b && b->bt != t) b = b->next;
	return b;
}

static struct ca *m3_getCA (struct decoder *d, enum blkType t, uint8_t grp, uint8_t caID)
{
	struct block *b;
	struct group *g;
	struct ca *ca;

	if ((b = m3_getBlock(d, t)) == NULL) return NULL;

	g = b->groups;
	while (g) {
		if (--grp == 0) {
			ca = g->cas;
			while (ca) {
				if (ca->caType == caID) return ca;
				ca = ca->next;
			}
		}
		g = g->next;
	}
	return NULL;
}

/**
 * Calculate the read size for a CA with predefined byte length
 *
 * \param ca	the current ca to read
 * \return		the number of bytes to read in next step or zero if this CA is complete
 */
static int m3_caByteLength (struct ca *ca)
{
	int bcnt;

	if (!ca || !ca->caDesc) return 0;
	bcnt = ca->caDesc->bytes - ca->len;
	if (bcnt >= 8) return 8;
	if (bcnt >= 4) return 4;
	if (bcnt >= 2) return 2;
	if (bcnt >= 1) return 1;
	return 0;
}

/**
 * Estimate the read size for a CA with variable byte length that is
 * zero terminated. The byte length defined in the configuration structure
 * is a limit for the length to read from decoder.
 *
 * As we cannot know how much data is left when the terminating null byte
 * is not found yet, we use a practical read size of 4 bytes.
 *
 * \param ca	the current ca to read
 * \return		the number of bytes to read in next step or zero if this CA is complete
 */
static int m3_caNullTerm (struct ca *ca)
{
	int bcnt;
	uint8_t *b;

	if (!ca || !ca->caDesc) return 0;
	if (ca->len >= ca->caDesc->bytes) return 0;		// we already reached the maximum length - stop here
	if (ca->len == 0) return 4;						// let's start with the first four bytes

	b = ca->data;
	while (b < &ca->data[ca->len - 1] && *b) b++;
	if (*b) {	// no null byte found yet - continue reading
		bcnt = ca->caDesc->bytes - ca->len;
		if (bcnt >= 4) return 4;
		if (bcnt >= 2) return 2;
		if (bcnt >= 1) return 1;
	}
	return 0;		// we are done
}

/**
 * Formats the generic part of a CA output line in a temporary string buffer
 *
 * \param ca		the CA that should be printed out
 * \return			a temporary string with the generic content
 * \see				tmp64()
 */
static char *m3_caPrintHeader (struct ca *ca)
{
	char *tmp = tmp64();

	sprintf (tmp, "         CV %3d: CA 0x%02X %-32.32s", ca->cv, ca->caType, ca->caDesc->name);
	return tmp;
}

static void m3_caPrintBytes (struct ca *ca, int group)
{
	char buf[256], *s;
	int i;

	(void) group;

	s = buf;
	for (i = 0; i < ca->len; i++) s += sprintf (s, "%02X ", ca->data[i]);

	log_msg (LOG_INFO, "%s %s\n", m3_caPrintHeader(ca), buf);
}

static void m3_caPrintString (struct ca *ca, int group)
{
	(void) group;

	log_msg (LOG_INFO, "%s %s\n", m3_caPrintHeader(ca), ca->data);
}

static void m3_caPrintIdent (struct ca *ca, int group)
{
	const char *symbol;
	const char *manufacturer;
	int article;

	(void) group;

	if (ca->data[0] == 0x01) {
		switch (ca->data[1]) {
			case 0x08: symbol = "(no Picture)"; break;
			case 0x09: symbol = "E-Lok"; break;
			case 0x0A: symbol = "Diesellok"; break;
			case 0x0B: symbol = "Dampflock"; break;
			default: symbol = "(unknown)"; break;
		}
	} else {
		symbol = "(untyped)";
	}
	switch (ca->data[4]) {
		case 0x83: manufacturer = "Trix/Märklin"; break;
		case 0x97: manufacturer = "ESU"; break;
		default: manufacturer = "(unknown manufacturer)"; break;
	}
	article = (ca->data[5] << 16) | (ca->data[6] << 8) | (ca->data[7] << 0);
	log_msg (LOG_INFO, "%s %s %s %d\n", m3_caPrintHeader(ca), manufacturer, symbol, article);
}

static const struct funcMapping *m3_mapFunc (uint8_t *data)
{
	const struct funcMapping *fm;

	if (!data || (!data[0] && !data[1] && !data[2])) return NULL;		// this no mapping

	fm = fmap;
	while (!(fm->group & 0x80)) {
		if ((data[0] & 0x7F) == fm->group && data[1] == fm->s1 && data[2] == fm->s2) break;
		fm++;
	}
	if (fm->group & 0x80) {
		fm = fmapGroups;
		while (!(fm->group & 0x80)) {
			if ((data[0] & 0x7F) == fm->group) break;
			fm++;
		}
	}
	return (fm->group & 0x80) ? NULL : fm;
}

static void m3_caPrintFuncMap (struct ca *ca, int group)
{
	const struct funcMapping *fm;
	char func[32];

	if (group <= 16) sprintf (func, "F%d", group - 1);
	else if (group == 17) sprintf (func, "STOP function");
	else if (group == 18) sprintf (func, "DRIVE function");
	else sprintf (func, "Sensor %d", group - 18);

	if (ca->data[0] || ca->data[1] || ca->data[2]) {
		fm = fmap;
		while (!(fm->group & 0x80)) {
			if ((ca->data[0] & 0x7F) == fm->group && ca->data[1] == fm->s1 && ca->data[2] == fm->s2) break;
			fm++;
		}
		if (fm->group & 0x80) {
			fm = fmapGroups;
			while (!(fm->group & 0x80)) {
				if ((ca->data[0] & 0x7F) == fm->group) break;
				fm++;
			}
		}
		if (fm->group & 0x80) {
			log_msg (LOG_INFO, "%s %s %sno ICON for %02X %02X %02X\n", m3_caPrintHeader(ca), func,
					(ca->data[0] & 0x80) ? "momentary " : "", ca->data[0], ca->data[1], ca->data[2]);
		} else {
			log_msg (LOG_INFO, "%s %s %s-> SYM %d\n", m3_caPrintHeader(ca), func,
					(ca->data[0] & 0x80) ? "momentary " : "", fm->symbol);
		}
	} else {
		log_msg (LOG_INFO, "%s %s not specified\n", m3_caPrintHeader(ca), func);
	}
}

/**
 * The status and data while reading the configuration area of an M3 decoder
 */
static const struct CAdef blk01[] = {	// configuration / Root block
	{ 0x10, "Manufacturer", 8, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "Production", 8, m3_caByteLength, m3_caPrintBytes },
	{ 0x12, "Bootloader/Firmware B", 8, m3_caByteLength, m3_caPrintBytes },
	{ 0x13, "Firmware /Firmware A", 12, m3_caByteLength, m3_caPrintBytes },
	{ 0x14, "Protocol information", 5, m3_caByteLength, m3_caPrintBytes },
	{ 0x15, "(unknown)", 4, m3_caByteLength, m3_caPrintBytes },
	{ 0x16, "loco identification", 8, m3_caByteLength, m3_caPrintIdent },
	{ 0x17, "block table", 63, m3_caNullTerm, m3_caPrintBytes },
	{ 0x18, "loco name", 16, m3_caNullTerm, m3_caPrintString },
	{ 0x19, "user data", 16, m3_caNullTerm, m3_caPrintString },
	{ 0x1A, "(unknown)", 8, m3_caByteLength, m3_caPrintBytes },
	{ 0x1B, "hardware revision", 12, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk02[] = {	// functions
	{ 0x10, "drive functions", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "switch functions", 16, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk03[] = {	// automatic functions
	{ 0x10, "halt function", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "drive function", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x12, "sensor function", 1, m3_caByteLength, m3_caPrintBytes },		// may repeat several times with a function for each sensor input
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk04[] = {	// function mapping
	{ 0x10, "function symbol", 3, m3_caByteLength, m3_caPrintFuncMap },
	{ 0x12, "function FWD", 4, m3_caByteLength, m3_caPrintBytes },
	{ 0x13, "function REV", 4, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk05[] = {	// motor configuration
	{ 0x10, "motor count", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "motor type", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x12, "motor frequency", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x13, "acceleration, deceleration", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x14, "trimming", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x15, "PID params", 4, m3_caByteLength, m3_caPrintBytes },
	{ 0x16, "break track", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x17, "speed curve", 28, m3_caByteLength, m3_caPrintBytes },
	{ 0x18, "tacho", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x19, "reverse operation", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk06[] = {	// hardware functions
	{ 0x10, "configuration", 3, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "internal functions", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x12, "sound functions", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk07[] = {	// additional protocols
	{ 0x10, "protocol", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "protocol config", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x12, "always active functions", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x13, "MM/DCC addresses", 4, m3_caByteLength, m3_caPrintBytes },
	{ 0x14, "analog voltages", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk08[] = {	// sound
	{ 0x10, "volume", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "sound type", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x12, "steam sound", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x13, "speed sound", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x14, "random sound", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x15, "break sound", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x16, "automatic sound", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk09[] = {	// extra options
	{ 0x10, "storage option A", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "storage option B", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x12, "output configuration", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct CAdef blk0A[] = {	// mfx+(R)
	{ 0x10, "supply", 3, m3_caByteLength, m3_caPrintBytes },
	{ 0x11, "maximum supply", 6, m3_caByteLength, m3_caPrintBytes },
	{ 0x12, "supply usage", 3, m3_caByteLength, m3_caPrintBytes },
	{ 0x13, "supply reserve", 3, m3_caByteLength, m3_caPrintBytes },
	{ 0x14, "limp home speeds", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x15, "operation time", 4, m3_caByteLength, m3_caPrintBytes },
	{ 0x16, "ODO / distance", 4, m3_caByteLength, m3_caPrintBytes },
	{ 0x17, "load", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x18, "gear factor", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x19, "interval", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x1A, "mfx+(R)", 1, m3_caByteLength, m3_caPrintBytes },
	{ 0x1B, "cockpit", 2, m3_caByteLength, m3_caPrintBytes },
	{ 0x00, NULL, 0, NULL, NULL }		// end marker
};

static const struct BLKdef blocks[] = {
	{ BLK_DESCRIPTION, "Configuration/Root", blk01 },
	{ BLK_FUNCTIONS, "Functions", blk02 },
	{ BLK_AUTOMATICFUNC, "automatic Functions", blk03 },
	{ BLK_FUNCMAPPING, "Function mapping", blk04 },
	{ BLK_MOTORCONFIG, "Motor configuration", blk05 },
	{ BLK_HARDWARE, "Hardware Functions", blk06 },
	{ BLK_PROTOCOL, "additional protocols", blk07 },
	{ BLK_SOUND, "Sound", blk08 },
	{ BLK_EXTRAOPTIONS, "extra Options", blk09 },
	{ BLK_M3PLUS, "mfx+(R)", blk0A },
	{ BLK_ENDLIST, "-END-", NULL }
};

static void m3_printGroup (struct group *grp, int idx)
{
	struct ca *ca;

	log_msg(LOG_INFO, "      CV %d GROUP #%d\n", grp->cv, idx);
	ca = grp->cas;
	while (ca) {
		if (ca->caDesc && ca->caDesc->interpreter) ca->caDesc->interpreter(ca, idx);
		ca = ca->next;
	}
}

static void m3_printBlock (struct block *blk)
{
	struct group *grp;
	int idx;

	log_msg (LOG_INFO, "   CV %d BLOCK 0x%02x %20.20s %d groups @ %d CAs/group\n", blk->cv, blk->bt, blk->blkDesc->name,
			blk->grpCount, blk->caPerGrp);
	grp = blk->groups;
	idx = 1;
	while (grp) {
		m3_printGroup(grp, idx);
		grp = grp->next;
		idx++;
	}
}

static void m3_freeDecoder (struct decoder *d)
{
	struct block *b;
	struct group *g;
	struct ca *c;

	while ((b = d->blocks) != NULL) {
		d->blocks = b->next;
		while ((g = b->groups) != NULL) {
			b->groups = g->next;
			while ((c = g->cas) != NULL) {
				g->cas = c->next;
				free (c);
			}
			free (g);
		}
		free (b);
	}
	free (d);
}

static void m3_printDecoder (void *pvParameter)
{
	struct decoder *d;
	struct block *blk;

	d = (struct decoder *) pvParameter;

	if (d) {
		log_msg(LOG_INFO, "%s() ADR %d\n", __func__, d->adr);
		blk = d->blocks;
		while (blk) {
			vTaskDelay(50);
			m3_printBlock(blk);
			blk = blk->next;
		}
		if (d->caller) {
			xTaskNotify(d->caller, 0, eSetValueWithOverwrite);
		} else {
			m3_freeDecoder (d);
		}
	}
	vTaskDelete(NULL);
}

static struct ca *m3_addCA (struct decoder *d, int cv)
{
	struct ca *p, **pp;

	if (!d || !d->curGroup) return NULL;
	if ((p = calloc (1, sizeof(*p))) == NULL) return NULL;
	p->cv = cv;
	pp = &d->curGroup->cas;
	while (*pp) pp = &(*pp)->next;
	*pp = p;
	d->curCa = p;

	return p;
}

static struct group *m3_addGroup (struct decoder *d, int cv)
{
	struct group *p, **pp;

	if (!d || !d->curBlock) return NULL;
	if ((p = calloc (1, sizeof(*p))) == NULL) return NULL;
	p->cv = cv;
	pp = &d->curBlock->groups;
	while (*pp) pp = &(*pp)->next;
	*pp = p;
	d->curGroup = p;
	d->curCa = NULL;

	return p;
}

static struct block *m3_addBlock (struct decoder *d, int cv)
{
	struct block *p, **pp;

	if (!d) return NULL;
	if ((p = calloc (1, sizeof(*p))) == NULL) return NULL;
	p->cv = cv;
	pp = &d->blocks;
	while (*pp) pp = &(*pp)->next;
	*pp = p;
	d->curBlock = p;
	d->curGroup = NULL;
	d->curCa = NULL;

	return p;
}

static bool m3_cvCallback (struct decoder_reply *msg, flexval priv)
{
	struct decoder *d;
	struct block *blk;
	struct group *grp;
	struct ca *ca;
	const struct CAdef *cad;
	const struct BLKdef *m3blk;
	cvadrT cva;
	int i, bcnt;

	d = (struct decoder *) priv.p;
	if (msg->mt == DECODERMSG_TIMEOUT) {		// maybe the track is inactive - nothing happened
		log_msg (LOG_ERROR, "%s() timed out - giving up\n", __func__);
		if (d->caller) {
			xTaskNotify(d->caller, 1, eSetValueWithOverwrite);
		} else {
			m3_freeDecoder (d);
		}
		return false;
	}
	if (msg->mt != DECODERMSG_M3DATA) {		// seems to be an error
		log_msg(LOG_WARNING, "%s() error MT=%d CV%d.%d bytes=%ld (try %d)\n", __func__, msg->mt, msg->cva.m3cv, msg->cva.m3sub,
				msg->param.i32, d->retry);
		if (++d->retry > 20) {
			log_msg (LOG_ERROR, "%s() excessive retries - giving up\n", __func__);
			if (d->caller) {
				xTaskNotify(d->caller, 2, eSetValueWithOverwrite);
			} else {
				m3_freeDecoder (d);
			}
			return false;
		}
		m3_cvReader(d, msg->cva, msg->param.i32);
		return false;
	}

	log_msg(LOG_INFO, "%s() CV%d.%d %d bytes\n", __func__, msg->cva.m3cv,  msg->cva.m3sub, msg->len);

	d->retry = 0;
	cva = msg->cva;		// as a default, the current CV address is taken

	switch (d->stat) {
		case QSTAT_BLOCKHEAD0:
			if (msg->data[0] != BLK_DESCRIPTION) {
				log_msg (LOG_WARNING, "%s() First CA in Block @ CV%d.%d is not the block descriptor (is 0x%02x, should be 0x01)\n",
						__func__, msg->cva.m3cv, msg->cva.m3sub, msg->data[0]);
			}
			if ((blk = m3_addBlock(d, msg->cva.m3cv)) == NULL) break;
			blk->bt = msg->data[1];
			blk->version = msg->data[2];
			m3blk = blocks;
			while ((m3blk->blktype != blk->bt) && (m3blk->blktype != BLK_ENDLIST)) m3blk++;
			if (m3blk->blktype != BLK_ENDLIST) blk->blkDesc = m3blk;
			cva.m3sub += msg->len;
			d->stat = QSTAT_BLOCKHEAD1;
			m3_cvReader(d, cva, 2);
			return false;

		case QSTAT_BLOCKHEAD1:
			if ((blk = d->curBlock) == NULL) break;	// something went wrong - quit
			blk->grpCount = msg->data[0];
			blk->caPerGrp = msg->data[1];
			for (i = 0; i < blk->grpCount; i++) {		// create all groups (groups are somewhat virtual!)
				m3_addGroup(d, blk->cv + 1 + i * blk->caPerGrp);	// the CV of the group is really the CV of it's first CA
			}
			d->curGroup = blk->groups;
			cva.m3cv = blk->cv;
			cva.m3sub = 0;
			d->stat = QSTAT_CADATA;
			m3_cvReader(d, cva, 1);
			return false;

		case QSTAT_CADATA:			// either just the CA type byte or CA data is received here - depends on msg->cva.m3sub
			if (msg->cva.m3sub == 0) {	// CA type byte received - create new CA
				if ((ca = m3_addCA(d, msg->cva.m3cv)) == NULL) break;
				ca->caType = msg->data[0];
				if ((ca->caType == 0x17) && d->curBlock && (d->curBlock->bt == BLK_DESCRIPTION)) {	// This block will be the block list
					d->blkList = &ca->data[0];
				}
				if (d->curBlock && d->curBlock->blkDesc) {					// try to find a CA description
					cad = d->curBlock->blkDesc->cas;
					while (cad && cad->ca != 0) {
						if (cad->ca == ca->caType) {
							ca->caDesc = cad;
							break;
						}
						cad++;
					}
				}
			} else {
				if ((ca = d->curCa) == NULL) break;
				memcpy (&ca->data[ca->len], msg->data, msg->len);
				ca->len += msg->len;
			}
			if (ca->caDesc && (bcnt = ca->caDesc->reader(ca)) > 0) {	// do we have to read data?
				cva.m3cv = ca->cv;
				cva.m3sub = 1 + ca->len;
				m3_cvReader(d, cva, bcnt);
				return false;
			}
			// OK, no further data to read. Let's check for next CA/group or block
			blk = d->curBlock;
			grp = d->curGroup;
			if (++cva.m3cv >= (grp->cv + blk->caPerGrp)) {				// check for next group
				d->curGroup = grp->next;
			}
			if (d->curGroup) {											// we are still reading grouped CAs
				cva.m3sub = 0;
				m3_cvReader(d, cva, 1);
				return false;
			}
			// OK - let's start with next block if any
			if (d->blkList && *d->blkList) {
				cva.m3cv = *d->blkList * 4;
				cva.m3sub = 0;
				d->stat = QSTAT_BLOCKHEAD0;
				d->blkList++;
				m3_cvReader(d, cva, 4);
				return false;
			}
			break;
	}

	xTaskCreate(m3_printDecoder, "M3Config", configMINIMAL_STACK_SIZE * 4, d, 2, NULL);
	return false;
}

/**
 * Read the M3 decoder information (and currently print out to log console).
 * This function assumes, that the track is on GO but may also be used as
 * programming track function, if the output is currently switched to the
 * programming track.
 *
 * \param adr		the loco address (M3) to query
 * \return			currently always 0 or a negative error code
 */
int m3_readDecoder (int adr)
{
	struct decoder *d;
	struct packet *p;
	cvadrT cva;
	flexval fv;

	log_msg(LOG_INFO, "%s() START\n", __func__);
	if ((d = calloc(1, sizeof(*d))) == NULL) {
		log_error ("%s() no memory - exit\n", __func__);
		return -1;
	}
	cva.m3cv = 0;
	cva.m3sub = 0;
	fv.p = d;
	if ((p = sigq_m3ReadCV(adr, cva, 4, m3_cvCallback, fv)) == NULL) {
		m3_freeDecoder (d);
		return -3;
	}

	d->adr = adr;
//	reply_register(DECODER_M3_MOBILE, adr, DECODERMSG_M3DATA, m3_cvCallback, d, 2000);
//	reply_register(DECODER_M3_MOBILE, adr, DECODERMSG_NOANSWER, m3_cvCallback, d, 0);
//	reply_register(DECODER_M3_MOBILE, adr, DECODERMSG_READERROR, m3_cvCallback, d, 0);
	sigq_queuePacket(p);

	return 0;
}

static int m3_checkCA (struct decoder *d, int param)
{
	if (d->ca[0] != param) return -1;
	return 0;
}

static int m3_getMaxRdWr (struct decoder *d, int param)
{
	(void) param;

	if (d->cva.m3sub == 0) {
		d->cva.m3sub = 5;
		return 1;
	} else if (d->cva.m3sub == 5) {
		d->rdLen = 1 << (d->ca[5] & 0x03);
		d->wrLen = 1 << ((d->ca[5] >> 2) & 0x03);
		return 0;		// try to read blocklist
	}

	return -1;
}

static int m3_getManufacturer (struct decoder *d, int param)
{
	(void) param;

	if (d->caLen == 0) {
		d->cva.m3cv = 8;
		d->cva.m3sub = 4;
		return 1;
	}
	d->manufacturer = d->ca[4];
	db_setLocoVID(d->adr, d->manufacturer);
	return 0;
}

static int m3_getName (struct decoder *d, int param)
{
	uint8_t *p;

	(void) param;

	if (d->caLen == 0) {
		d->cva.m3cv = 3;		// should be here
		d->cva.m3sub = 0;
		return 8;
	}
	p = d->ca;
	if (*p != 0x18) return -1;	// we assume that we have CA-Typ 0x18 here (the loco name)
	p++;
	while (p < &d->ca[d->caLen] && *p) p++;
	if (p < &d->ca[d->caLen]) {		// the name is now null terminated
		if (d->l && d->l->loco) db_setLocoName(d->l->loco->adr, (char *) &d->ca[1]);
		return 0;
	}
	d->cva.m3sub = d->caLen;
	return 8;
}

static int m3_getBlockList (struct decoder *d, int param)
{
	uint8_t *p;
	struct block *b, **bpp;

	(void) param;

	if (d->caLen == 0) {
		d->cva.m3cv = 4;		// should be here
		d->cva.m3sub = 0;
		return 4;
	}
	p = d->ca;
	if (*p != 0x17) return -1;	// we assume that we have CA-Typ 0x17 here (the blocklist)
	p++;
	while (p < &d->ca[d->caLen] && *p) p++;
	if (p < &d->ca[d->caLen]) {		// the list is now terminated by a blocklist entry of 0x00
		p = &d->ca[1];		// generate all the blocks
		bpp = &d->blocks;
		while (*p) {
			if ((b = calloc(1, sizeof(*b))) == NULL) return -1;
			b->cv = *p * 4;
			*bpp = b;
			bpp = &b->next;
			p++;
		}
		return 0;
	}
	d->cva.m3sub = d->caLen;
	return 8;
}

static int m3_readBlockList (struct decoder *d, int param)
{
	struct block *b;
	const struct BLKdef *bdef;

	// check which block to read next
	b = d->blocks;
	while (b) {
		if (!b->bt || !b->caPerGrp) break;
		b = b->next;
	}
	if (!b) return 0;		// OK, we are done

	if (d->caLen == 0) {			// read CA typ (should be 0x01 for block descriptor) and the block type (2 bytes)
		d->cva.m3cv = b->cv;
		d->cva.m3sub = 0;
		d->caLen = 0;
		return 2;
	} else if (d->caLen == 2) {		// read # of groups and CA per group (2 bytes)
		if (d->ca[0] != 0x01) return -1;
		b->bt = d->ca[1];
		d->cva.m3sub = 4;
		return 2;
	} else if (d->caLen == 6) {		// block descriptor read successfully
		b->grpCount = d->ca[4];
		b->caPerGrp = d->ca[5];
		bdef = blocks;
		while ((bdef->blktype != b->bt) && (bdef->blktype != BLK_ENDLIST)) bdef++;
		if (bdef->blktype != BLK_ENDLIST) b->blkDesc = bdef;
		d->caLen = 0;
		return m3_readBlockList(d, param);
	}

	return -1;
}

static int m3_readFunctions (struct decoder *d, int param)
{
	struct block *b;
	struct ca *ca;

	(void) param;

	if (d->caLen == 0) {
		if ((b = m3_getBlock(d, BLK_FUNCTIONS)) == NULL) return -1;
		d->cva.m3cv = b->cv + 1;
		d->cva.m3sub = 0;
		return 1;
	}
	if (d->caLen == 1) {
		if (d->ca[0] != 0x11) {
			if ((b = m3_getBlock(d, BLK_FUNCTIONS)) == NULL) return -1;
			d->cva.m3cv++;
			if (d->cva.m3cv > b->cv + b->grpCount * b->caPerGrp) return -1;
			return 1;
		}
		return 8;
	}
	if (d->caLen < 17) {
		d->cva.m3sub = d->caLen;
		return 17 - d->caLen;
	}
	if ((b = m3_getBlock(d, BLK_FUNCTIONS)) == NULL) return -1;
	d->curBlock = b;	// this should be changed ... (will be done later when cleaning up code)
	m3_addGroup(d, b->cv + 1);
	ca = m3_addCA(d, d->cva.m3cv);
	if (ca) {
		ca->caType = d->ca[0];
		ca->len = d->caLen - 1;
		memcpy (ca->data, &d->ca[1], d->caLen - 1);
	}
	return 0;
}

static int m3_getFuncIcon (struct decoder *d, int param)
{
	const struct funcMapping *fm;
	struct block *b;
	struct ca *ca;

	if ((ca = m3_getCA(d, BLK_FUNCTIONS, 1, 0x11)) == NULL) return -1;
	if ((b = m3_getBlock(d, BLK_FUNCMAPPING)) == NULL) return -1;
	if (d->caLen == 0) {
		d->cva.m3cv = b->cv + ca->data[param];
		d->cva.m3sub = 0;
		return 4;
	}

	if ((fm = m3_mapFunc(&d->ca[1])) != NULL) {
		db_locoFuncIcon(d->l->loco, param, fm->symbol);
		db_locoFuncTiming(d->l->loco, param, (d->ca[1] & 0x80) ? -1 : 0);
	}
	return 0;
}

static const struct step funcsAndName[] = {
	{ m3_checkCA, 0x14 },		// make sure that the first CA in block @ CV 0 is the protocol information
	{ m3_getMaxRdWr, 0 },		// check maximum read and write length
	{ m3_getManufacturer, 0 },	// read manufacturer code frmo CV 8.4
	{ m3_getName, 0 },			// read the Name
	{ m3_getBlockList, 0 },		// read the block list
	{ m3_readBlockList, 0 },	// read all types and sizes of the block list
	{ m3_readFunctions, 0 },	// read in the function table set out in BLOCK 0x02, CA-Type 0x11
	{ m3_getFuncIcon, 0 },		// map the function icon for F0
	{ m3_getFuncIcon, 1 },		// map the function icon for F1
	{ m3_getFuncIcon, 2 },		// map the function icon for F2
	{ m3_getFuncIcon, 3 },		// map the function icon for F3
	{ m3_getFuncIcon, 4 },		// map the function icon for F4
	{ m3_getFuncIcon, 5 },		// map the function icon for F5
	{ m3_getFuncIcon, 6 },		// map the function icon for F6
	{ m3_getFuncIcon, 7 },		// map the function icon for F7
	{ m3_getFuncIcon, 8 },		// map the function icon for F8
	{ m3_getFuncIcon, 9 },		// map the function icon for F9
	{ m3_getFuncIcon, 10 },		// map the function icon for F10
	{ m3_getFuncIcon, 11 },		// map the function icon for F11
	{ m3_getFuncIcon, 12 },		// map the function icon for F12
	{ m3_getFuncIcon, 13 },		// map the function icon for F13
	{ m3_getFuncIcon, 14 },		// map the function icon for F14
	{ m3_getFuncIcon, 15 },		// map the function icon for F15
	{ endStepList, 0 }			// pop current list and continue with previous list (or end action if stack gets empty)
};

/**
 * A small helper function that triggers the next read command.
 * If getting a packet fails, the caller is notyfied (if set) or
 * the decoder structure is freed.
 *
 * \param d		the decoder status and data structure
 * \param cva		the current CV address including the subaddress
 * \param bytes		the number of bytes to read (should be 1, 2, 4 or 8
 */
static void m3_cvReader (struct decoder *d, cvadrT cva, int bytes)
{
	struct packet *p;
	flexval fv;

	fv.p = d;
	if ((p = sigq_m3ReadCV(d->adr, cva, bytes, m3_funcCallback, fv)) == NULL) {
		if (d->caller) {
			xTaskNotify(d->caller, 3, eSetValueWithOverwrite);
		} else {
			m3_freeDecoder (d);
		}
		return;
	}
	sigq_queuePacket(p);
}

static bool m3_funcCallback (struct decoder_reply *msg, flexval priv)
{
	struct decoder *d;
	int bcnt;

	d = (struct decoder *) priv.p;
	if (msg->mt == DECODERMSG_TIMEOUT) {		// maybe the track is inactive - nothing happened
		log_msg (LOG_ERROR, "%s() timed out - giving up\n", __func__);
		if (d->caller) {
			xTaskNotify(d->caller, -1, eSetValueWithOverwrite);
		} else {
			m3_freeDecoder (d);
		}
		return false;
	}
	if (msg->mt != DECODERMSG_M3DATA) {		// seems to be an error
		log_msg(LOG_WARNING, "%s() error MT=%d CV%d.%d bytes=%ld (try %d)\n", __func__, msg->mt, msg->cva.m3cv, msg->cva.m3sub,
				msg->param.i32, d->retry);
		if (++d->retry > 20) {
			log_msg (LOG_ERROR, "%s() excessive retries - giving up\n", __func__);
			if (d->caller) {
				xTaskNotify(d->caller, -2, eSetValueWithOverwrite);
			} else {
				m3_freeDecoder (d);
			}
			return false;
		}
		m3_cvReader(d, msg->cva, msg->param.i32);
		return false;
	}

	log_msg(LOG_INFO, "%s() CV%d.%d %d bytes\n", __func__, msg->cva.m3cv,  msg->cva.m3sub, msg->len);
	memcpy (&d->ca[msg->cva.m3sub], msg->data, msg->len);
	d->caLen = msg->cva.m3sub + msg->len;	// this is not true, if the read was a sparse read (i.e. some bytes where skipped)

	d->retry = 0;

	while (d->stack && d->stack->step && (bcnt = d->stack->step->func(d, d->stack->step->param)) >= 0) {
		if (bcnt == 0) {
			d->stack->step++;
			d->caLen = 0;
			continue;
		}
		if (bcnt > d->rdLen) bcnt = d->rdLen;
		m3_cvReader(d, d->cva, bcnt);
		return false;
	}

	xTaskNotify(d->caller, 0, eSetValueWithOverwrite);
	return false;
}

/**
 * Read part of the M3 decoder information (namely the function icons and
 * decoder name).
 * This function assumes, that the track is on GO but may also be used as
 * programming track function, if the output is currently switched to the
 * programming track.
 *
 * \param adr		the loco address (M3) to query
 * \return			currently always 0 or a negative error code
 */
int m3_readFuncs (int adr)
{
	struct decoder *d;
	struct packet *p;
	struct block *b;
	struct ca *ca;
	flexval fv;
	int rc;
	uint8_t *c;

	log_msg(LOG_INFO, "%s() START\n", __func__);
	if ((d = calloc(1, sizeof(*d))) == NULL) {
		log_error ("%s() no memory - exit\n", __func__);
		return -1;
	}
	d->wrLen = d->rdLen = 1;		// we start with very basic capabilities
	d->cva.m3cv = 1;
	d->cva.m3sub = 0;
	d->caller = xTaskGetCurrentTaskHandle();
	xTaskNotifyStateClear(NULL);
	d->adr = adr;
	if ((d->l = loco_call(adr, true)) == NULL) {
		log_error("%s() cannot get/create loco with address %d\n", __func__, adr);
		m3_freeDecoder (d);
		return -1;
	}
	if (!d->l->loco || !FMT_IS_M3(d->l->loco->fmt)) {
		log_error ("%s() loco %d is not in M3 format - give up\n", __func__, adr);
		m3_freeDecoder (d);
		return -2;
	}

	fv.p = d;
	if ((p = sigq_m3ReadCV(adr, d->cva, 1, m3_funcCallback, fv)) == NULL) {
		m3_freeDecoder (d);
		return -3;
	}
	pushSteps(d, funcsAndName);

	sigq_queuePacket(p);

	rc = (int) ulTaskNotifyTake(pdTRUE, 10000);
	// add func-icons and name to data base
	if (rc == 0) {
		log_msg (LOG_INFO, "%s() Manufacturer 0x%02x\n", __func__, d->manufacturer);
		log_msg (LOG_INFO, "%s() rdLen=%d wrLen=%d\n", __func__, d->rdLen, d->wrLen);
		log_msg (LOG_INFO, "%s() Name='%s'\n", __func__, d->l->loco->name);
		b = d->blocks;
		while (b) {
			log_msg (LOG_INFO, "%s() BLOCK 0x%02x %-20.20s @ CV %d, %d groups @ %d CA/GRP\n", __func__, b->bt, b->blkDesc->name,
					b->cv, b->grpCount, b->caPerGrp);
			b = b->next;
		}
		b = m3_getBlock(d, BLK_FUNCMAPPING);
		if (b && (ca = m3_getCA(d, BLK_FUNCTIONS, 1, 0x11)) != NULL) {
			c = ca->data;
			while (c < &ca->data[ca->len]) {
				log_msg (LOG_INFO, "%s(): F @ CV %d\n", __func__, b->cv + *c);
				c++;
			}
		}
		d->l->loco->config = CONF_M3;

		// do it!
	}
	m3_freeDecoder (d);

	return rc;
}

/**
 * Set the name of the decoder (write to CV 3)
 *
 * This function assumes, that the track is on GO but may also be used as
 * programming track function, if the output is currently switched to the
 * programming track.
 *
 * \param adr		the loco address (M3) to set the name
 * \param name		the new name string (is truncated if longer than 16 bytes)
 * \return			currently always 0 or a negative error code
 */
int m3_setName (int adr, char *name)
{
	struct packet *p;
	cvadrT cva;
	uint8_t *s;
	int len, bytes;

	if ((s = (uint8_t *) name) == NULL) return -1;
	len = strlen(name) + 1;		// include the null byte
	if (len > 16) len = 16;		// if truncated, the CA will contain a nullbyte at the non-existant subaddress 17

	cva.m3cv = 3;
	cva.m3sub = 1;
	while (cva.m3sub < len + 1) {
		bytes = len + 1 - cva.m3sub;
		if (bytes >= 4) bytes = 4;
		else if (bytes >= 2) bytes = 2;
		else bytes = 1;
		if ((p = sigq_m3WriteCVar(adr, cva, s, bytes, 1)) == NULL) return -3;
		sigq_queuePacket(p);
		s += bytes;
		cva.m3sub += bytes;
	}
	return 0;
}
