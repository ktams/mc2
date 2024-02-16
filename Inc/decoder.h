/*
 * decoder.h
 *
 *  Created on: 24.12.2019
 *      Author: Andi
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

#include "bidib.h"

/**
 * @ingroup Track
 * @{
 */
#ifndef __DECODER_H__
#define __DECODER_H__

#define MIN_LOCO_ADR			1					///< loco addresses are counted from 1
#define MAX_MM_ADR				255					///< maximum address for Motorola format
#define MAX_DCC_ADR				10239				///< maximum address for DCC format
#define DCCA_PREFIX				0xFE				///< address byte for DCC-A commands (from reserved address space)
#define MAX_M3_ADR				16383				///< maximum address for M3 format
#define MAX_LOCO_ADR			MAX_M3_ADR			///< the absolute maximum allowed address

#define MIN_TURNOUT				1					///< turnout addresses are counted from 1
#define MAX_MM_TURNOUT			1024				///< the last turnout address in MM format
#define MAX_DCC_ACCESSORY		2047				///< the last turnout address in DCC format
#define MAX_TURNOUT				MAX_DCC_ACCESSORY	///< the absolute maximum allowed turnout address
#define MAX_DCC_EXTACC			MAX_DCC_ACCESSORY	///< the last extended accessory decoder address (only DCC format supported)

#define LOCO_NAME_LEN			64					///< a maximum length for a loco name
#define LOCO_MAX_FUNCS			128					///< maximum number of funcs that are supported (0 .. 127)
#define BITS_PER_WORD			32					///< number of bits in a word (of the basic type used for function status storage)
#define MAX_FUNC_WORDS			((LOCO_MAX_FUNCS + BITS_PER_WORD - 1) / BITS_PER_WORD)	///< number of words needed to hold all function states
#define MAX_ICON_INDEX			127					///< function icons can take values of 0 .. 127 inclusive

#define MIN_DCC_CVADR			0					///< the lowest CV number allowed in DCC system
#define MAX_DCC_CVADR			1023				///< the highest CV number allowed in DCC system
#define MAX_DCC_EXTCVADR		((2 << 24) - 1)		///< extended CV addressing with 24 bits (xPOM)

#define MIN_M3_CVADR			0					///< the lowest CV number allowed in m3 system
#define MAX_M3_CVADR			1023				///< the highest CV number allowed in m3 system
#define MAX_M3_CVSUBADR			63					///< m3 CVs have subaddresses 0 .. 63 (0x3F)

#define MAX_CONSISTLENGTH		2					///< maximum length of a consist (currently limited to double-traction)
#define MAX_TESTCMD_BYTES		24					///< number of bytes in a (DCC-) test command, including final XOR

// error codes used when reading CVs via programming track
#define ERR_NO_LOCO				-1					///< no decocder detected (current draw to small)
#define ERR_CV_UNSUPPORTED		-2					///< the decoder didn't react to a 0-bit nor a 1-bit comparision
#define ERR_CV_COMPARE			-3					///< the comparision didn't succeed with the value read bitwise
#define ERR_CV_WRITE			-4					///< the value could not be written (possibly a read-only variable)
#define ERR_SHORT				-10					///< there was a short on the programming track
#define ERR_UNSTABLE			-11					///< the idle / base current of the decoder is not stable enough
#define ERR_INTERRUPTED			-12					///< programming procedure was interrupted
#define ERR_INTERNAL			-20					///< internal error (call with out-of-range CV or without an action function for the loop)

#define BITBUFFER_BYTES			20					///< number of bytes to reserve for bit information in the bitbuffer (-> 160 bits)

/**
 * Signal classes for routing signals to the outputs.
 */
enum sigclass {
	SIGCLASS_MOBILE = 0,			///< packets for mobile decoders
	SIGCLASS_ACCESSORY,				///< packets für basic and extended accessory decoders
	/* maybe more to come later, but for now it is only these two ... */
};

/**
 * Type of decoders. Used mostly for decoder replies and such stuff.
 * It will also be used for address representation in queue commands
 * to cut down the huge amount of different commands in some contexts.
 */
typedef enum {
	DECODER_ANY = 0,				///< if we wait for messages, we can wait for any type of decoder - supply address 0 to get every reply
	DECODER_DCC_MOBILE,				///< DCC: a mobile (loco) decoder
	DECODER_DCC_ACC,				///< DCC: a basic accessory decoder
	DECODER_DCC_EXT,				///< DCC: an extended accessory decoder
	DECODER_MM_MOBILE,				///< MM: a mobile (loco) decoder (either MM1 or MM2)
	DECODER_MM_FUNC,				///< MM: a function decoder (only used with MM1)
	DECODER_MM_ACC,					///< MM: a basic accessory decoder (turnout or function)
	DECODER_M3_MOBILE,				///< M3: an M3 mobile (loco) decoder
	DECODER_DCC_MOBILE_ALT,			///< DCC: a mobile decoder with old mapping of railcom ID3 (speed instead of position)
} dec_type;

/**
 * The type of message that was received from a decoder.
 * The concrete type sometimes depend on the decoder type and the request
 * that was sent to it.
 */
typedef enum {
	DECODERMSG_INVALID = 0,			///< this message is invalidated - only for internal usage and must not be forwared anywhere
	DECODERMSG_ANY,					///< any message matches this message type
	DECODERMSG_TIMEOUT,				///< instead of receiving a message a timeout occured (you cannot wait for that event)
	DECODERMSG_READERROR,			///< a read could not be properly done, some error occured (but not a timeout, which means "no answer at all")
	DECODERMSG_NOANSWER,			///< there was no answer detected (i.e. missing decoder?)
	DECODERMSG_POM,					///< a message from a POM access
	DECODERMSG_XPOM00,				///< a message from XPOM accesses including the "serial" ID 0b00
	DECODERMSG_XPOM01,				///< a message from XPOM accesses including the "serial" ID 0b01
	DECODERMSG_XPOM10,				///< a message from XPOM accesses including the "serial" ID 0b10
	DECODERMSG_XPOM11,				///< a message from XPOM accesses including the "serial" ID 0b11
	DECODERMSG_DYN,					///< a spontaneous data of the decoder
	DECODERMSG_M3BIN,				///< a simple YES/NO answer from M3 reply window (usually, only YES will be reported, NO is detected as timeout)
	DECODERMSG_M3DATA,				///< a data response from M3 decoder
	DECODERMSG_PGACK,				///< an acknowledge current impulse was read on programming track
	DECODERMSG_ADRL,				///< the low byte of the decoder address (in window #1)
	DECODERMSG_ADRH,				///< the high byte of the decoder address (in window #1)
	DECODERMSG_EXT,					///< location services
	DECODERMSG_STAT1,				///< the STAT1 message (STAT2 is obsolete)
	DECODERMSG_TIME,				///< a time message from decoder
	DECODERMSG_ERR,					///< error message from decoder
	DECODERMSG_SPEED,				///< old meaning of ID3: current speed
	DECODERMSG_DECSTATE,			///< state of a decoder (railcom ID13 for DCC-A)
	DECODERMSG_UNIQUE,				///< unique ID of a decoder (railcom ID15 for DCC-A)
	DECODERMSG_DCCABLOCK,			///< a DCC-A data block using all 6 Bytes
	DECODERMSG_SHORTINFO,			///< a DCC-A data block representing the SHORTINFO block
	DECODERMSG_COLLISION,			///< a special code for DCC-A decoder search
	DECODERMSG_ACK,					///< a special code for DCC-A replies: if an ACK instead of a block arrives, we deliver this ACK
	DECODERMSG_NACK,				///< just in case a decoder answers with NACK, we schould delivert that, too!
	DECODERMSG_RUNTIME,				///< the time since when decoder is on track - for identifying a decoder via XF2 off to address 0
	DECODERMSG_SRQ,					///< an SRQ from railcom channel #1 with accessory decoders
	DECODERMSG_ADDRESS,				///< ADR-H and ADR-L are detected and make up a valid address (only reported if different from previous)

	DECODERMSG_UNKNOWN				///< delivered with all bytes remaining if the message is not decodable
} dec_msgtype;

/**
 * The type of readback window to use.
 * We have two read back systems: m3 and RailCom. Additionally, there
 * may be the BiDiBus, but that is currently not used in this fashion.
 */
typedef enum {
	READBACK_STANDARD = 0,			///< any type of default handling for the relevant system (i.e. interpreting IDs for RailCom)
	READBACK_POM,					///< wait for a POM message that carries read CV data
	READBACK_XPOM,					///< wait for a XPOM message that carries read CV data (always 4 bytes
	READBACK_POMWRITE,				///< wait for a POM message that carries read CV data that equals an expected value
	READBACK_DCCA_ID,				///< the whole RailCom window is used as one ID packet (ID13 and ID15 answers for DCC-A)
	READBACK_DCCA_DATA,				///< the whole RailCom window is used as one DATA area of up to 6 complete bytes (8 encode bytes)
	READBACK_DCCA_SHORTINFO,		///< a special version of READBACK_DCCA_DATA, interpret result as "special format A"
	READBACK_DCCA_ACK,				///< a special version of READBACK_DCCA_DATA, check for ACK also in window #1
	READBACK_ACC_SRQ,				///< service request for (extended) accessory decoders
	READBACK_DCC_PT,				///< read back acknowledges (current pulses) from programming track (done outside interrupt)
	READBACK_M3BIN,					///< read a binary state from a decoder (i.e. a positive answer or nothing), corresponds to DECODERMSG_M3BIN
	READBACK_M3DATA,				///< read a binary state from a decoder (i.e. a positive answer or nothing), corresponds to DECODERMSG_M3DATA
} rdbk_type;

/**
 * Definition of the generic track format including the speed
 * information.
 */
enum fmt {
	FMT_UNKNOWN = 0,				///< just to have something like this
	FMT_MM1_14,						///< Märklin-Motorola 1 with 14 speeds
	FMT_MM2_14,						///< Märklin-Motorola 2 with 14 speeds
	FMT_MM2_27A,					///< Märklin-Motorola 2 with 27 speeds (send alternating speeds)
	FMT_MM2_27B,					///< Märklin-Motorola 2 with 27 speeds (manipulate trit 5)
	FMT_M3_126,						///< M3 with 126 speeds
	FMT_DCC_14,						///< the DCC format with 14 speeds (old decoders)
	FMT_DCC_28,						///< the DCC format with 28 speeds
	FMT_DCC_126,					///< the DCC format with 126 speeds
	FMT_DCC_SDF,					///< the DCC format with 126 speeds understanding the new SDF (Speed, Direction, Function) command

	FMT_MM1_FD,						///< MM-Format for function decoders
	TFMT_MM,						///< MM-Format for turnouts
	TFMT_DCC,						///< DCC-Format for turnouts and extended accessory decoders
	TFMT_BIDIB,						///< special for accessories/turnouts: mapped to a BiDiB node
};

enum configtype {
	CONF_MANUAL,					///< address and other information was specified manually
	CONF_M3,						///< information was gathered by readout via M3
	CONF_DCCA,						///< address and configuration was acquired via DCC-A readout
	CONF_RAILCOMPLUS,				///< data was acquired  via RailCom+
};

#define FMT_IS_MM1(f)		((f) == FMT_MM1_14)
#define FMT_IS_MM2(f)		((f) == FMT_MM2_27B || (f) == FMT_MM2_14 || (f) == FMT_MM2_27A)	// optimisation: 27B should be most common
#define FMT_IS_MM(f)		(FMT_IS_MM2(f) || FMT_IS_MM1(f) || (f) == TFMT_MM)				// optimisation: MM2 should be most common
#define FMT_IS_MM27A(f)		((f) == FMT_MM2_27A)
#define FMT_IS_MM27B(f)		((f) == FMT_MM2_27B)
#define FMT_IS_MM27(f)		(FMT_IS_MM27A(f) || FMT_IS_MM27B(f))
#define FMT_IS_DCC(f)		((f) == FMT_DCC_14 || (f) == FMT_DCC_28 || (f) == FMT_DCC_126 || (f) == FMT_DCC_SDF || (f) == TFMT_DCC)
#define FMT_IS_M3(f)		((f) == FMT_M3_126)
#define FMT_IS_TURNOUT(f)	((f) == TFMT_MM || (f) == TFMT_DCC)

// some function- and functiongroup-definitions (only the first 31 functions + light)
#define FUNC(x)				(1 << ((x) & 31))
#define FUNC_LIGHT			FUNC(0)
#define FUNC_F0_F4			0x0000001F		// DCC 28, 126 speed steps
#define FUNC_F1_F4			0x0000001E		// DCC 14 speed steps, MM1 FD
#define FUNC_F5_F8			0x000001E0		// DCC
#define FUNC_F1_F8			0x000001FE		// DCC
#define FUNC_F9_F12			0x00001E00		// DCC
#define FUNC_F5_F12			0x00001FE0		// DCC
#define FUNC_F13_F20		0x001FE000		// DCC
#define FUNC_F21_F28		0x1FE00000		// DCC
#define FUNC_F29_F31		0xE0000000		// DCC (rest up to first 32-bit uint)
#define FUNC_F0_F15			0x0000FFFF		// M3
#define FUNC_F16_F31		0xFFFF0000		// M3
#define FUNC_F9_F16			0x0001FE00		// P50x
#define FUNC_F17_F31		0xFFFE0000		// P50x
#define FUNC_F5_F11			0x00000FE0		// Loconet
#define FUNC_F13_F19		0x000FE000		// Loconet
#define FUNC_F21_F27		0x0FE00000		// Loconet
#define FUNC_F12_F20_F28	0x10101000		// Loconet

typedef enum {
	SIGNAL_UNKNOWN = 0,				///< undefined signal buffer - usually you should skip that packet
	SIGNAL_DCC,						///< marks a bitbuffer as generating DCC signal
	SIGNAL_DCC_XPOM00,				///< marks a bitbuffer as generating DCC signal - special marking for XPOM commands
	SIGNAL_DCC_XPOM01,				///< marks a bitbuffer as generating DCC signal - special marking for XPOM commands
	SIGNAL_DCC_XPOM10,				///< marks a bitbuffer as generating DCC signal - special marking for XPOM commands
	SIGNAL_DCC_XPOM11,				///< marks a bitbuffer as generating DCC signal - special marking for XPOM commands
	SIGNAL_DCC_A,					///< marks a bitbuffer as generating DCC signal - special marking for DCC-A commands
	SIGNAL_MMSLOW,					///< marks a bitbuffer as generating MM SLOW signal (for locos)
	SIGNAL_MMFAST,					///< marks a bitbuffer as generating MM FAST signal (for turnouts and utilities)
	SIGNAL_M3,						///< marks a bitbuffer as generating M3 compatible signal
} sigT;

/**
 * The queue commands detail what kind of command is to be assembled, when the
 * packet is is to be formed to a bitbuffer for interrupt handling. The concrete
 * construction of the bitbuffer and the range of information sent therein is
 * still dependant on the decoder format.
 */
enum queue_cmd {
	// generic commands
	QCMD_NOP = 0,					///< a dummy command as NOP for empty or removed queue entries
	QCMD_SETSPEED,					///< supply a new speed to the loco (for MM2 with 27 speeds this is the "standard" MM27B)
	QCMD_REVERSEDIR,				///< reverse direction of the loco
	QCMD_EMERGENYSTOP,				///< do an emergency stop for the loco
	QCMD_SETFUNC,					///< set functions (a generic call - some formats use special commands)

	// magnet (turnout) commands
	QCMD_MAGNET_ON,					///< switch on a magnet
	QCMD_MAGNET_OFF,				///< switch off a magnet

	// some MM-only commands
	QCMD_MM_SETSPEED_27A,			///< a special speed for MM27A, which must not be optimized away on queue updates
	QCMD_MM_REVERSE,				///< a special speed for MM1 to reverse direction (also used as emergency stop)
	QCMD_MM_SETF1,					///< manipulate the F1 function
	QCMD_MM_SETF2,					///< manipulate the F2 function
	QCMD_MM_SETF3,					///< manipulate the F3 function
	QCMD_MM_SETF4,					///< manipulate the F4 function
	QCMD_MM_FDFUNCS,				///< set functions on function decoders (MM1, MM-Fast)

	// some DCC-only commands
	QCMD_DCC_RESET,					///< a DCC reset packet
	QCMD_ACC_RESET,					///< a DCC reset packet for accessory decodes
	QCMD_DCC_IDLE,					///< a DCC idle packet
	QCMD_DCC_SETF1_4,				///< transmit functions (F0) F1 - F4
	QCMD_DCC_SETF5_8,				///< transmit functions F5 - F8
	QCMD_DCC_SETF9_12,				///< transmit functions F9 - F12
	QCMD_DCC_SETF13_20,				///< transmit functions F13 - F20
	QCMD_DCC_SETF21_28,				///< transmit functions F21 - F28
	QCMD_DCC_SETF29_36,				///< transmit functions F29 - F36
	QCMD_DCC_SETF37_44,				///< transmit functions F37 - F44
	QCMD_DCC_SETF45_52,				///< transmit functions F45 - F52
	QCMD_DCC_SETF53_60,				///< transmit functions F53 - F60
	QCMD_DCC_SETF61_68,				///< transmit functions F61 - F68
	QCMD_DCC_PT_WRITEBYTE,			///< 'Direct' Service Mode: Write Byte
	QCMD_DCC_PT_VERIFYBYTE,			///< 'Direct' Service Mode: Verify Byte
	QCMD_DCC_PT_WRITEBIT,			///< 'Direct' Service Mode: Bitwise write
	QCMD_DCC_PT_VERIFYBIT,			///< 'Direct' Service Mode: Bitwise verify
	QCMD_DCC_PT_WRITEPHYSREG,		///< 'Physical register access': Write Byte
	QCMD_DCC_PT_VERIFYPHYSREG,		///< 'Physical register access': Verify Byte
	QCMD_DCC_POM_READ,				///< 'On-Track' CV read byte from mobile decoder
	QCMD_DCC_POM_WRITE,				///< 'On-Track' CV write byte to mobile decoder
	QCMD_DCC_POM_WRITEBIT,			///< 'On-Track' CV write bit to mobile decoder
	QCMD_DCC_POM_ACC_READ,			///< 'On-Track' CV read byte from accessory decoder
	QCMD_DCC_POM_ACC_WRITE,			///< 'On-Track' CV write byte to accessory decoder
	QCMD_DCC_POM_ACC_WRITEBIT,		///< 'On-Track' CV write bit to accessory decoder
	QCMD_DCC_POM_EXT_READ,			///< 'On-Track' CV read byte from extended accessory decoder
	QCMD_DCC_POM_EXT_WRITE,			///< 'On-Track' CV write byte to extended accessory decoder
	QCMD_DCC_POM_EXT_WRITEBIT,		///< 'On-Track' CV write bit to extended accessory decoder
	QCMD_DCC_XWR1,					///< 'On-Track' CV write byte to mobile decoder short form with one byte (only special registers)
	QCMD_DCC_XWR2,					///< 'On-Track' CV write byte to mobile decoder short form with two byte (only special registers)
	QCMD_DCC_XPOM_RD_BLK,			///< 'On-Track' mobile decoder extended CV (24 bit) read block of 4 bytes
	QCMD_DCC_XPOM_WR_BIT,			///< 'On-Track' mobile decoder extended CV (24 bit) write bit
	QCMD_DCC_XPOM_WR_BYTE1,			///< 'On-Track' mobile decoder extended CV (24 bit) write one byte
	QCMD_DCC_XPOM_WR_BYTE2,			///< 'On-Track' mobile decoder extended CV (24 bit) write two bytes
	QCMD_DCC_XPOM_WR_BYTE3,			///< 'On-Track' mobile decoder extended CV (24 bit) write three bytes
	QCMD_DCC_XPOM_WR_BYTE4,			///< 'On-Track' mobile decoder extended CV (24 bit) write four bytes
	QCMD_DCC_XPOM_ACC_RD_BLK,		///< 'On-Track' basic accessory extended CV (24 bit) read block of 4 bytes
	QCMD_DCC_XPOM_ACC_WR_BIT,		///< 'On-Track' basic accessory extended CV (24 bit) write bit
	QCMD_DCC_XPOM_ACC_WR_BYTE1,		///< 'On-Track' basic accessory extended CV (24 bit) write one byte
	QCMD_DCC_XPOM_ACC_WR_BYTE2,		///< 'On-Track' basic accessory extended CV (24 bit) write two bytes
	QCMD_DCC_XPOM_ACC_WR_BYTE3,		///< 'On-Track' basic accessory extended CV (24 bit) write three bytes
	QCMD_DCC_XPOM_ACC_WR_BYTE4,		///< 'On-Track' basic accessory extended CV (24 bit) write four bytes
	QCMD_DCC_XPOM_EXT_RD_BLK,		///< 'On-Track' extended accessory extended CV (24 bit) read block of 4 bytes
	QCMD_DCC_XPOM_EXT_WR_BIT,		///< 'On-Track' extended accessory extended CV (24 bit) write bit
	QCMD_DCC_XPOM_EXT_WR_BYTE1,		///< 'On-Track' extended accessory extended CV (24 bit) write one byte
	QCMD_DCC_XPOM_EXT_WR_BYTE2,		///< 'On-Track' extended accessory extended CV (24 bit) write two bytes
	QCMD_DCC_XPOM_EXT_WR_BYTE3,		///< 'On-Track' extended accessory extended CV (24 bit) write three bytes
	QCMD_DCC_XPOM_EXT_WR_BYTE4,		///< 'On-Track' extended accessory extended CV (24 bit) write four bytes
	QCMD_DCC_BINSTATE,				///< transmit a binary state information
	QCMD_DCC_XACCASPECT,			///< switch an aspect on an extended accessory decoder (currently only DCC knows such decoders ...)
	QCMD_DCC_MODELTIME,				///< send model time or date on track
	QCMD_DCC_SYSTIME,				///< set the system time in milliseconds (since system startup)
	QCMD_DCC_ACCNOP,				///< give accessory decoders a chance to report incidents with RailCom
	QCMD_DCC_EXTACCNOP,				///< give extended accessory decoders a chance to report incidents with RailCom
	QCMD_DCC_SDF,					///< a combined packet with Speed, Direction and Functions (optimising the track refresh)
	QCMD_DCCA_LOGON_ENABLE_ALL,		///< enable DCC-A logon for all decoders
	QCMD_DCCA_LOGON_ENABLE_LOCO,	///< enable DCC-A logon for mobile (loco) decoders
	QCMD_DCCA_LOGON_ENABLE_ACC,		///< enable DCC-A logon for stationary (accessory) decoders
	QCMD_DCCA_LOGON_ENABLE_NOW,		///< enable DCC-A logon for all decoders ignoring the backoff
	QCMD_DCCA_SELECT_SHORTINFO,		///< the SELECT command to request the ShortInfo block
	QCMD_DCCA_SELECT_RDBLOCK,		///< the SELECT command to request a DCC-A block (extended capabilities, space info, ShortGUI or CV block)
	QCMD_DCCA_SELECT_DEC_STATUS,	///< the SELECT command to (re-)set the decoder state (currently only to reset change flags)
	QCMD_DCCA_GETDATA_START,		///< the GET_DATA_START just after having sent the SELECT
	QCMD_DCCA_GETDATA_CONT,			///< the GET_DATA_CONT for all remaining bytes in a block transfer
	QCMD_DCCA_LOGON_ASSIGN,			///< assign a (temporary) track address to the decoder

	// some M3-only commands
	QCMD_M3_BEACON,					///< send out an M3 beacon
	QCMD_M3_SEARCH,					///< search for unregistered M3-decoders
	QCMD_M3_NADR,					///< send an NADR packet to assign a new loco number to the decoder
	QCMD_M3_PING,					///< ping a decoder to check, if it is still there (or at least has excepted his new address)
	QCMD_M3_SHORTSPEED,				///< a shortened speed command for speeds divisable by 16 (inkl. speed 0)
	QCMD_M3_SPEEDFUNC,				///< combined packet with speed and F0 - F15
	QCMD_M3_SINGLEFUNC,				///< set a single function (for functions beyond F15)
	QCMD_M3_CVREAD,					///< read from the M3 configuration space
	QCMD_M3_CVWRITE,				///< write to the M3 configuration space
};

struct consist {
	struct consist	*next;						///< linked list of consists
	int				 adr[MAX_CONSISTLENGTH];	///< the addresses of the locos beeing part of that consist, negative for reversed direction
};

/*
 * Defines for locoT.flags (flags for decoder specialties)
 */
#define DEC_DCCA					0x0001		///< this decoder supports DCC-A

/*
 * Defines for ldataT.flags (flags for operational behavior)
 */
#define LOCO_CONSIST_REVERSE		0x0001		///< in a consist this loco is to be operated with reversed direction

typedef struct function funcT;
struct function {
	funcT			*next;						///< linked list of funtions
	uint16_t		 fnum;						///< the number of the function
	uint16_t		 icon;						///< a mapped icon (0 = no icon, representation of other values is left to the relevant GUI)
	int				 timing;					///< meaning: 0: toggle func, <0: momentary func, >0: activated for this number of 1/10s
};

/**
 * This struct holds additional information that is only available when
 * read via DCC-A commands. It will be stored in the loco.ini together
 * with other loco related information.
 *
 * Strings i theis structure are UTF-8 coded and zero terminated.
 */
struct dccaInfo {
	char			 vendor[41];				///< vendor name (data space 6)
	char			 product[41];				///< product name (data space 6)
	char			 hw_version[21];			///< hardware version (data space 6)
	char			 fw_version[41];			///< soft- / firmware version (data space 6)
	char			 shortname[10];				///< a short name from the shortGUI, maybe different from the long name in struct loco
	char			 userdesc[64];				///< a user description (data space 5)
	uint16_t		 decoderimage;				///< a more detailled picture to use (see TN-218 or RCD-218)
	uint8_t			 decodericon;				///< simple functional symbol for the decoder (4 bits are used, see RCN-218)
	int				 adr_req;					///< the ID (loco number) that the loco requested/wished on DCC-A assignment
	int				 userimage;					///< a customized user image as reference number (some index in a user gallery)
};

/**
 * This structure holds descriptive elements for each loco. This is the data
 * that should be stored in a file after change. Life loco data is hold in a
 * separate structure which itself references this one.
 */
typedef struct loco locoT;
struct loco {
	locoT			*next;						///< linked list of loco definitions
	int				 adr;						///< the ID (loco number) for this entry
	int				 maxfunc;					///< number of highest available function (i.e. 0 means F0 only, 4 means F0 plus F1 - F4, ...)
	enum fmt		 fmt;						///< the basic format information
	enum configtype	 config;					///< the type of configuration that led to the current/initial settings
	uint32_t		 vid;						///< a vendort ID if known
	uint32_t		 uid;						///< a UID if the decoder supports it and we can access it
	uint32_t		 flags;						///< decoder and format relevant flags (DEC_...)
	funcT			*funcs;						///< a list of function properties, unlisted functions are standard switching without icon
	struct dccaInfo	*dcca;						///< optional information gathered thru DCC-A commands
	char			 name[LOCO_NAME_LEN];		///< a name given to this loco (must be null terminated)
};

typedef struct ldata ldataT;
struct ldata {
	ldataT			*next;						///< linked list of live loco data
	locoT			*loco;						///< the connection to the base information of this loco
	ldataT			*consist;					///< a list of linked locos (multitraction / consist) organized as a ring
	TickType_t		 purgeTime;					///< time left until the loco leave the refresh (ms)
	uint32_t		 flags;						///< operational flags LOCO_...
	int				 speed;						///< current speed as positive integer value including the direction bit (bit 7) as for DCC
	uint32_t		 funcs[MAX_FUNC_WORDS];		///< a bit array holding the state of all functions
	int				 age;						///< a count for successive refresh cycles, may be used to outdate unused locos
};

typedef struct turnout turnoutT;
struct turnout {
	turnoutT		*next;						///< linked list of turnout definitions
	int				 adr;						///< the ID (turnout number) for this entry, one-based (0 is the default turnout format)
	enum fmt		 fmt;						///< the decoder format
	TickType_t		 start;						///< the time at which the turnout was energized
	uint8_t			 uid[BIDIB_UID_LEN];		///< the node where the turnout is connected (only when fmt is TFMT_BIDIB)
	uint8_t			 aspect;					///< the aspect (unit) inside the node
	unsigned		 dir:1;						///< the current direction
	unsigned		 on:1;						///< a flag to reproduce the energized state
};

typedef struct extacc extaccT;
struct extacc {
	extaccT			*next;						///< linked list of extended accessory decoder definitions
	int				 adr;						///< the ID (decoder address) for this entry
	enum fmt		 fmt;						///< the decoder format (currently we are only aware of DCC decoders -> use TFMT_DCC)
	// maybe we will need more
};

typedef union cvadr cvadrT;
union cvadr {
	uint32_t		 cv;						///< the complete CV as a single/simple 32 bit value
	struct {
		uint32_t	 m3sub:6;					///< M3: subaddress (offset in a configuration array CA)
		uint32_t	 m3cv:10;					///< M3: the CV address that addresses a complete CA
	};
	struct {
		uint32_t	 dcccv:8;					///< DCC: CV address inside a block, for block indexed access via CV31/CV32
		uint32_t	 dccblk:16;					///< DCC: the block address of a block indexed access (value of CV31/CV32)
	};
};

struct m3block {
	struct m3block	*next;						///< linked list of blocks
	uint8_t			blkadr;						///< the CV address of this block (multiply by 4 to get the real CV address)
	uint8_t			blktype;					///< the block type code
	uint8_t			groups;						///< number of groups in this block
	uint8_t			caPerGrp;					///< number of CAs per group
};

struct m3decoder {
	int				manufacturer;				///< the DCC manufacturer code of the decoder (0x83=Trix/Märklin, 0x97=ESU)
	struct m3block	*blocks;					///< the blocks in this configuration (read from BLOCK 0 CA 0x17 (usually found at CV 4)
	uint8_t			maxRdBytes;					///< maximum bytes to be read in a single call (two bits coding: 0=1, 1=2, 2=4, 3=8 Bytes)
	uint8_t			maxWrBytes;					///< maximum bytes to be written in a single call (two bits coding as described for maxRdBytes)
	uint8_t			blocklist[63];				///< a list of block starts (CV = blockstart * 4)
	uint8_t			name[17];					///< the name from the decoder configuration, null terminated UTF-8 string
};

/**
 * A message received via railcom, m3-reply or from external BiDi-Bus.
 * It is tailored for DCC responses, where you can have up to four messages
 * in a single reply window (railcom cutout).
 */
//struct decoder_message {
//	int				 adr;						///< the deocder address we are expecting the answer from
//	flexval			 param;						///< additional parameter
//	cvadrT			 cva;						///< an address of a CV that was read. This may consist of different parts bit-shifted together
//	dec_type		 dtype;						///< the type of decoder we are expecting the answer from
//	dec_msgtype		 mt;						///< the message type (mostly mapped from railcom ID)
//	rcid			 id[4];						///< a maximum of four messages may be included in this reply
//	uint8_t			 len[4];					///< the amount of valid data in the received message for each of the messages
//	uint8_t			 data[16];					///< shared data bytes for all messages contained in the answer (like up to 16 CV variables, etc.)
//	bool			 ack;						///< at least one ACK was received
//	bool			 nack;						///< at least one NACK was received
//};

struct decoder_reply {
	int				 adr;						///< the deocder address we are expecting the answer from
	dec_type		 dtype;						///< the type of decoder we are expecting the answer from
	dec_msgtype		 mt;						///< the message type (mostly mapped from railcom ID or command)
	cvadrT			 cva;						///< an address of a CV that was read. This may consist of different parts bit-shifted together
	flexval			 param;						///< an additional parameter for some commands (bin-state, number of CVs in m3, ...)
	uint8_t			 len;						///< the amount of valid data in the received message for each of the messages
	uint8_t			 data[16];					///< data bytes for the message contained in the answer (like up to 16 CV variables, etc.)
};

typedef bool (*reply_handler) (struct decoder_reply *, flexval);	///< generic decoder callback handler (registered using reply_register())
//typedef void (*track_handler) (struct decoder_message *, void *);	///< a track command specific callback handler

struct packet {
	struct packet	*next;						///< a singly linked list of scheduled packets
	reply_handler	 cb;						///< a callback function for decoder replies
	flexval			 priv;						///< an additional private argument for the callback
	union {
		struct {								// the normal data for standard packets
			int				 adr;				///< the number/address of the loco (aka. ID), may also be a turnout number or address of an extended accessory decoder
			cvadrT			 cva;				///< for addressing CVs in a decoder for reading or writing
			flexval			 param;				///< a parameter for the command
			flexval			 value;				///< a value for the command
			uint32_t		 funcs[MAX_FUNC_WORDS];		///< we always copy all functions to the packet just to have it always on hand
		};
		struct {
			int				 len;				///< number of bytes to send
			uint8_t			 test[MAX_TESTCMD_BYTES];	///< data for experimental DCC commands
		};
	};
	int				 repeat;					///< number of repetitions on the track
	enum queue_cmd	 cmd;						///< the command to execute
	enum fmt		 fmt;						///< the signal format of the decoder
};

/**
 * A structure holding the bit-representation of a single track frame in any of the
 * supported formats. It also contains reference information for read back technologies
 * like mfx(R) or RailCom(R).
 */
struct bitbuffer {
	reply_handler	 cb;						///< a callback function for decoder replies
	flexval			 priv;						///< an additional private argument for the callback
	sigT			 fmt;						///< the basic definition, how the bits in this buffer are modeled on the track
	dec_type		 dt;						///< for decoder replies: the type of decoder
	rdbk_type		 rdt;						///< for decoder replies: the type of answer we expect
	int				 adr;						///< for decoder replies: the high level address of the decoder
	cvadrT			 cva;						///< a more complete version of CV addresses during read and write of CVs
	flexval			 param;						///< for decoder replies: an additional information parameter (CV, M3-UID, ...)
	int				 repeat;					///< how often to repeat this block
	uint32_t		 components;				///< a bit array with format dependant signal components
	uint32_t		 current_comp;				///< a shift register that "scans" the activated components
	uint8_t			 databits[BITBUFFER_BYTES];	///< this is the bitbuffer for the main part of the signal
	int				 bits;						///< number of valid bits in the databits buffer
	int				 pos;						///< the current position in the (data-)bit array
	volatile bool	 ack;						///< set if any kind of feedback registers an acknowledge
//	uint8_t			 lastdata;					///< when writing a CV: the last value reported in a railcom answer, may be different to what we have written
	union {
		struct {		// DCC information
			int			preamble;				///< how much preamble bits to send
			int			tail;					///< how much tails bits to send
			uint8_t		targetval;				///< can be set to an expected POM value (CV content)
			uint8_t		lastval;				///< the last value acknowledged from decoder on a POM write
			bool		startbit;				///< if true, the startbit was sent and we can start with bit 0 of next data byte
			bool		valreceived;			///< if true, a read back CV value was received, but probably not the right one
		} dcc;
		struct {		// MM (fast & slow) information
			int			pause;					///< the length (in µs) for the inter-packet gap
		} mm;
		struct {		// M3 information
			int			flagcnt;				///< number of "half" flags to send (-> ususally 2), also used to count other fixed stuff
			int			flagphase;				///< a flag consists of L-S-L-L-S-L (L=long 100µs, S=short 50µs) in 3 phases: L+S, L+L, S+L
			int			onebits;				///< a counter for successive one-bits to handle bit stuffing
			int			replybits;				///< how much 'half' reply sync pulses we must generate (depend on expected number of bytes to receive in reply)
			bool		halfbit;				///< if we had to send the first part of a '1' bit in PWM L-phase, remember to start next phase with second half
		} m3;
	};
};

/*
 * Prototypes Decoder/consist.c
 */
struct consist *consist_findConsist(int adr);
struct consist *_consist_couple (int adr1, int adr2);
struct consist *consist_couple (int adr1, int adr2);
struct consist *consist_coupleAdd (int adr1, int adr2);
void _consist_unlink (ldataT *l);
bool consist_dissolve (uint16_t adr);
bool consist_remove (uint16_t adr);
void consist_event (void);
struct consist *consist_getConsists(void);

/*
 * Prototypes Decoder/dcc_a.c
 */
uint8_t dccA_CRCcont (uint8_t crc, volatile uint8_t *data, int len);
uint8_t dccA_CRC (volatile uint8_t *data, int len);
void dccA_service (void *pvParameter);

/*
 * Prototypes Decoder/dcc_pom.c
 */
int dccpom_readByte (int adr, dec_type dt, int cv, reply_handler handler, flexval priv);
int dccpom_writeByte (int adr, dec_type dt, int cv, int val, reply_handler handler, flexval priv);
int dccpom_writeBytesShortForm (int adr, dec_type dt, int cv, uint8_t *val, int cnt, reply_handler handler, flexval priv);
int dccxpom_writeBytes (int adr, dec_type dt, int cv, uint8_t *val, int cnt, reply_handler handler, flexval priv);
int dccpom_writeBit (int adr, dec_type dt, int cv, uint8_t bit, bool val, reply_handler handler, flexval priv);
int dccpom_boosterConf (int vid, uint8_t param);

/*
 * Prototypes Decoder/dcc_pt.c
 */
int dccpt_cvReadByte (int cv);
int dccpt_cvWriteByte (int cv, uint8_t data);
int dccpt_cvReadBit (int cv, int bit);
int dccpt_cvWriteBit (int cv, int bit, uint8_t data);
void dccpt_cvReadByteBG (int cv, void (*cb)(int, void *), void *priv);
void dccpt_cvWriteByteBG (int cv, uint8_t data, void (*cb)(int, void *), void *priv);
void dccpt_cvReadBitBG (int cv, int bit, void (*cb)(int, void *), void *priv);
void dccpt_cvVerifyBitBG (int cv, int bit, uint8_t data, void (*cb)(int, void *), void *priv);
void dccpt_cvWriteBitBG (int cv, int bit, uint8_t data, void (*cb)(int, void *), void *priv);

/*
 * Prototypes Decoder/decoderdb.c
 */
void db_triggerStore (const char *caller);
void db_freeLocos (void);
void db_freeTurnouts (void);
int db_indexSorted_next (int idx);
int db_indexSorted_prev (int idx);
locoT *db_lookupLocoSorted (int idx);
int db_lookupIndex (locoT *loco);
int db_getSpeeds (enum fmt fmt);
locoT *db_locoSanitize (locoT *l);
locoT *_db_getLoco (int adr, bool add);
locoT *db_getLoco (int adr, bool add);
locoT *db_addFreeAdr (int base);
locoT *db_findLocoUID (uint32_t vid, uint32_t uid);
locoT *db_changeAdr (int adr, uint32_t vid, uint32_t uid);
funcT *db_getLocoFunc (locoT *l, int func);
void db_setLocoFmt (int adr, enum fmt fmt);
void db_setLocoVID (int adr, uint32_t vid);
void db_setLocoUID (int adr, uint32_t uid);
void db_setLocoName (int adr, char *name);
void db_setLocoMaxfunc (int adr, int maxfunc);
void db_locoFuncIcon (locoT *l, int func, int icon);
void db_locoFuncTiming (locoT *l, int func, int tim);
locoT *db_newLoco (int adr, enum fmt fmt, int maxfunc, char *name, char *uid);
void db_removeLoco (locoT *l);
turnoutT *db_turnoutSanitize (turnoutT *t);
turnoutT *db_lookupTurnout (int adr);
turnoutT *db_lookupBidibTurnout (uint8_t *uid, int aspect);
bool db_clearBidibTurnout (uint8_t *uid);
turnoutT *db_getTurnout (int adr);
enum fmt db_string2fmt (char *s);
const char *db_fmt2string(enum fmt format);
void db_iterateLoco (bool (*func)(locoT *, void *), void *priv);
int db_init(void);
void db_setTurnoutFmt (int adr, enum fmt fmt);
extaccT *db_extaccSanitize (extaccT *x);
extaccT *db_lookupExtacc (int adr);
extaccT *db_getExtacc (int adr);

/*
 * Prototypes Decoder/loco.c
 */
bool loco_lock(const char *caller);
void loco_unlock (void);
TickType_t loco_purgetime (void);
void _loco_remove (ldataT *l);
ldataT *_loco_getRefreshLink (locoT *l);
void loco_remove (ldataT *l);
ldataT *loco_call (int adr, bool add);
int loco_setFuncMasked (int adr, uint32_t newfuncs, uint32_t mask);
int loco_setFunc (int adr, int f, bool on);
int loco_setBinState (int adr, int state, bool on);
int loco_getSpeeds (locoT *l);
int loco_setSpeed (int adr, int speed);
int loco_emergencyStop (int adr);
bool m3_inRefresh (void);
//int m3pom_writeCV(int adr, cvadrT cva, uint8_t val, int repeat);
void loco_freeRefreshList(void);
ldataT *loco_refresh(void);
ldataT *loco_iterateNext (ldataT *cur);

/*
 * Prototypes Decoder/m3_config.c
 */
int m3_readDecoder (int adr);
int m3_readFuncs (int adr);
int m3_setName (int adr, char *name);

/*
 * Prototypes Decoder/m3_pom.c
 */
int m3pom_readCV (int adr, cvadrT cva, int bytes, reply_handler handler, flexval priv);
int m3pom_writeCV (int adr, cvadrT cva, uint8_t val, int repeat, reply_handler handler, flexval priv);
int m3pom_writeCVar (int adr, cvadrT cva, uint8_t *val, int bytes, int repeat, reply_handler handler, flexval priv);
//int m3_scan (uint32_t uid, int len);
int m3_setAddress (uint32_t uid, int adr);

/*
 * Prototypes Decoder/m3_pt.c
 */
uint32_t m3pt_getUID (void);
int m3pt_setAddress (uint32_t uid, int adr);
int m3pt_readCV (int adr, cvadrT cva, int bytes, int repeat, reply_handler handler, flexval priv);
int m3pt_writeCV (int adr, cvadrT cva, uint8_t val, int repeat);

/*
 * Prototypes Decoder/mm_pt.c
 */
int mmpt_findDecoder (int from, int to);
int mmpt_enterProgram (int adr);
int mmpt_cvProg (int adr, int cv, int val);
int mmpt_tamsLDW2address (int adr);

/*
 * Prototypes Decoder/reply.c
 */
int reply_register (dec_type dt, int adr, dec_msgtype msgtp, reply_handler handler, flexval priv, TickType_t timeout);
void reply_deliver (dec_type dt, int adr, dec_msgtype mt, cvadrT cva, flexval param, int len, uint8_t *data);
bool rc_event_handler(struct decoder_reply *msg, flexval priv);
void reply_callback (struct bitbuffer *bb, dec_msgtype mt, int len, uint8_t *data);

/*
 * Prototypes Decoder/request.c
 */
int rq_setFuncMasked (int adr, uint32_t newfuncs, uint32_t mask);
int rq_setSpeed (int adr, int speed);
int rq_emergencyStop (int adr);

/*
 * Prototypes Decoder/turnout.c
 */
void trnt_service (void *pvParameter);
char *trnt_getRanges (void);
void trnt_setMinTime (int ms);
int trnt_getMinTime (void);
void trnt_setMaxTime (int ms);
int trnt_getMaxTime (void);
int trnt_switchTimed (int adr, bool thrown, TickType_t tim);
int trnt_switch (int adr, bool thrown, bool on);

/*
 * Prototypes Decoder/xacc.c
 */
int xacc_aspect (int adr, int aspect);

/*
 * Prototypes Track/signal.c
 */
int sig_searchM3Loco (uint32_t *id);
enum trackmode sig_setMode (enum trackmode mode);
enum trackmode sig_getMode (void);
void sig_ACK (void);
bool sig_isACK (void);
uint32_t sig_getM3Beacon (void);
uint16_t sig_getM3AnnounceCounter (void);
void sig_setM3Beacon (uint32_t bid, uint16_t announce);
void vSigGeneration (void *pvParameters);
void sig_rcAck (struct bitbuffer *bb);
void sig_bidibACK (void);

/*
 * Prototypes Track/sigqueue.c
 */
struct packet *sigq_genPacket (const ldataT *l, enum fmt format, enum queue_cmd cmd);
int sigq_dcc_idle (int repeat);
int sigq_dcc_reset (int repeat);
int sigq_dcc_cvVerfyBit (int cv, int bit, bool val, int repeat);
int sigq_dcc_cvWriteBit (int cv, int bit, bool val, int repeat);
int sigq_dcc_cvVerfyByte (int cv, uint8_t val, int repeat);
int sigq_dcc_cvWriteByte (int cv, uint8_t val, int repeat);
struct packet *sigq_speedPacket (const ldataT *l, int speed);
struct packet *sigq_funcPacket (const ldataT *l, int f);
struct packet *sigq_emergencyStopPacket (const ldataT *l);
struct packet *sigq_binStatePacket (const ldataT *l, int state, bool on);
struct packet *sigq_sdfPacket (const ldataT *l);
struct packet *sigq_m3BeaconPacket (uint32_t beacon, uint16_t announce, int repeat);
struct packet *sigq_m3SearchPacket (uint32_t uid, int len, reply_handler handler, flexval priv);
struct packet *sigq_m3NewAddress (uint32_t uid, int adr);
struct packet *sigq_m3ReadCV (int adr, cvadrT cva, int bytes, reply_handler handler, flexval priv);
struct packet *sigq_m3WriteCVar (int adr, cvadrT cva, uint8_t *val, int len, int repeat);
struct packet *sigq_m3WriteCV (int adr, cvadrT cva, uint32_t val, int repeat);
struct packet *sigq_modelTimePacket (int h, int m, int wd, int factor, bool update);
struct packet *sigq_modelDatePacket (int y, int m, int d);
struct packet *sigq_sysTimePacket (void);
struct packet *sigq_magnetPacket (turnoutT *t, bool thrown, bool on);
struct packet *sigq_extaccPacket (extaccT *xacc, int aspect);
struct packet *sigq_OTCVreadPacket (const ldataT *l, int cv);
struct packet *sigq_OTCVreadPacketAcc (const turnoutT *t, int cv);
struct packet *sigq_OTCVreadPacketExt (const extaccT *ext, int cv);
struct packet *sigq_OTCVwritePacket (const ldataT *l, int cv, uint8_t val);
struct packet *sigq_OTCVwritePacketAcc (const turnoutT *t, int cv, uint8_t val);
struct packet *sigq_OTCVwritePacketExt (const extaccT *ext, int cv, uint8_t val);
struct packet *sigq_dcc_pomShortWrite (const ldataT *l, enum queue_cmd cmd, uint8_t cv_code, uint8_t *val);
struct packet *sigq_dcc_xPom (const ldataT *l, enum queue_cmd cmd, int cv, uint8_t *val);
struct packet *sigq_dcca_logonEnableAll (uint16_t cid, uint8_t session, reply_handler cb, flexval priv);
struct packet *sigq_dcca_logonEnableLoco (uint16_t cid, uint8_t session, reply_handler cb, flexval priv);
struct packet *sigq_dcca_logonEnableAcc (uint16_t cid, uint8_t session, reply_handler cb, flexval priv);
struct packet *sigq_dcca_logonEnableNow (uint16_t cid, uint8_t session, reply_handler cb, flexval priv);
struct packet *sigq_dcca_selectShortInfo (uint16_t mfr, uint32_t uid, reply_handler cb, flexval priv);
struct packet *sigq_dcca_selectBlock (uint16_t mfr, uint32_t uid, int blk, reply_handler cb, flexval priv);
struct packet *sigq_dcca_selectCVBlock (uint16_t mfr, uint32_t uid, uint32_t cv, uint8_t cnt, reply_handler cb, flexval priv);
struct packet *sigq_dcca_decoderState (uint16_t mfr, uint32_t uid, uint8_t param, reply_handler cb, flexval priv);
struct packet *sigq_dcca_logonAssign (uint16_t mfr, uint32_t uid, int adr, reply_handler cb, flexval priv);
struct packet *sigq_dcca_getDataStart (reply_handler cb, flexval priv);
struct packet *sigq_dcca_getDataCont (reply_handler cb, flexval priv);
void sigq_queuePacket (struct packet *p);
struct packet *sigq_getpacket(bool do_refresh);
void sigq_pushBack (struct packet *p);
int sigq_flush (void);
bool sigq_isIdle (void);

/*
 * Prototypes Track/sniffer_m3.c
 *
 * (the thread routines are declared in rb2.h)
 */
void m3reply_enable (dec_type dt, int adr, rdbk_type rdt, cvadrT cva, flexval fv);
void m3reply_disable (struct bitbuffer *bb);

/*
 * Prototypes Track/railcom.c
 *
 * (the thread routine is declared in rb2.h)
 */
void rc_Startinfo (void);
BaseType_t rc_POMfilter (int cvval);
BaseType_t rc_POMend (void);
BaseType_t railcomTarget (struct bitbuffer *bb);
void railcomTrigger (struct bitbuffer *bb);

/**
 * @}
 */
#endif /* __DECODER_H__ */
