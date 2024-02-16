/**
 * @file easynet.h
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

#ifndef __EASYNET_H__
#define __EASYNET_H__

/*
 * =========================================================================================
 * some defines for the bus protocol
 * =========================================================================================
 */

#define CRC8_POLYNOM			7		///< CRC-8, poly = x^8 + x^2 + x^1 + x^0, init = 0

#define EN_MINUNIT				0		///< controls range from EN_MINUNIT .. EN_MAXUNIT (inclusive)
#define EN_MAXUNIT				63
#define HOTPLUG_SCAN			EN_MAXUNIT + 1
#define POST_STATUS				EN_MAXUNIT + 2

#define BUS_BROADCAST			0xFF	///< 0x7F is the broadcast, the 8th bit is set for all addresses
#define BUS_DATALEN				6
#define BUS_BLOCKLEN			(BUS_DATALEN + 3)	///< 1.Byte: Address, 2.Byte: Command, last Byte: CRC

struct en_client {
	uint32_t	serno;
	uint32_t	hw_no;
	uint32_t	sw_no;
	int			alive;
	int			loco;					///< the currently controlled loco
	int			blocked;				///< a counter of ANS_SETSPEED requests that ought to be ignored because of a foreign override
};

typedef union {
	struct {
		uint8_t		adr;				///< the addressing of the block with MSB (bit 7) set. 0xFF is broadcast
		uint8_t		cmd;				///< the 7 bit command or answer from client unit
		uint8_t		data[BUS_DATALEN];	///< the data portion of the block (only 7 bits per byte maay be used!)
		uint8_t		crc;				///< the CRC of the block, probably inverted if MSB was set
	};
	uint8_t		bytes[BUS_BLOCKLEN];	///< the complete block as a byte array
	uint32_t	words[BUS_BLOCKLEN / 4];	///< two words for easier access in transmission and checks for set byte-MSBs, the last byte is not included
} en_block;

struct en_bootProgress {
	int			total;					///< total number of blocks to download to node
	int			current;				///< current block number that is downloading to node
};

// Messages can range from 0x00 to 0x7F (they must not use the topmost bit)
// Messages on EasyBus (EasyNet?): Do not use 0x00! It might be sent unintentional!

#define CMD_IDENTIFY			0x01	///< send type of unit (unused ...)
#define CMD_STATUSPOLL			0x02	///< send loco, speed, direction, funcs, and STOP/GO
#define CMD_SYSSTATUS			0x03	///< Broadcast (no answer): set system state
#define CMD_SETUNITADR			0x04	///< the unit with the matching serial number is assign the adress
#define CMD_FREELOCO			0x05	///< free this loco (may be ignored if loco is locked)
#define CMD_WAITFORSLOT			0x06	///< wait another round for loco allocation (queue is full)
#define CMD_YIELDLOCO			0x07	///< allow the unit to use this loco
#define CMD_YIELDTRACTION		0x08	///< this is the second loco of a double traction
#define CMD_LOCOFORMAT			0x09	///< information about the loco format and capabilities
#define CMD_OVERRIDE			0x0A	///< override speed and function with a new value
#define CMD_REPORTLOCO			0x0B	///< report controlled loco and serial number (for status storage)
#define CMD_RESTORELOCO			0x0C	///< restore the loco if serial number matches
#define CMD_EXFUNCS				0x0D	///< the function bits F16 - F31
#define CMD_LOCOINFO			0x0E	///< inform the control about the status of the loco requested with ANS_LOCOINFO
#define CMD_TRACTIONINFO		0x0F	///< inform the control about the status of a traction loco requested with ANS_LOCOINFO

#define CMD_LOCODB_INDEX		0x10	///< delivers index from DB-Request (prev / next / alloc)
#define CMD_LOCODB_DATA0		0x11	///< Data block 0 from Loco DB entry
#define CMD_LOCODB_DATA1		0x12	///< Data block 1 from Loco DB entry
#define CMD_LOCODB_DATA2		0x13	///< Data block 2 from Loco DB entry

#define CMD_TOSTATUS			0x19	///< Send the status of a turnout (direction and magnetstatus)

#define CMD_VERSION				0x1C	///< Send or query the software version

#define CMD_FMAP				0x1E	///< answer to single function mapping get/set/clr
#define CMD_FMAPLIST			0x1F	///< answer with all functions mapping (multiple occurences may be necessary)

#define CMD_DORESET				0x20	///< Do a reset (don't wait any longer)
#define CMD_ERRMSG				0x21	///< send an errormessage to the user (2. LCD line)
#define CMD_LCDLIGHT			0x22	///< switch off (or back on) LCD backlight

#define CMD_MAGNET				0x23	///< magnet switched on or off
#define CMD_CONFIG				0x24	///< answer to system config requests
#define CMD_TOUT_CTRL			0x25	///< switch on and off timeout in a slave (for remote PT-actions)

/*
 * Hotplug commands check to see if there are units that do not yet have a busadress.
 * It scans for the 24 bit serial number. Since only 7 bits in each data byte are usable
 * (eigth bit is the Start-Of-Block marker) we split the 24 serial number bits into 3 times
 * 7 bits plus a 3 bit part (most significant). The Scan is done as a tree scan with 4
 * levels.
 */
#define CMD_HOTPLUG0			0x40	///< Hotplug query with topmost 3 bits
#define CMD_HOTPLUG1			0x41	///< Hotplug query with topmost 3 + next 7 bits
#define CMD_HOTPLUG2			0x42	///< Hotplug query with topmost 3 + next 14 bits
#define CMD_HOTPLUG3			0x43	///< Hotplug query with all 24 bits

// Answer codes from slave units
#define ANS_REQUESTLOCO			0x01	///< request to gain control over a loco
#define ANS_REQUESTTRACTION		0x02	///< request to establish a double traction
#define ANS_SETSPEED			0x03	///< standard answer: current speed and function (F0 - F15) status
#define ANS_SETLOCOFORMAT		0x04	///< request to change loco format settings
#define ANS_CLEARTRACTION		0x05	///< request to clear a double traction
#define ANS_SETMAGNET			0x06	///< switch magnet
#define ANS_DCCONTRACK			0x07	///< DCC-OnTrack programming request
#define ANS_DCCRAILCOM			0x08	///< DCC-OnTrack programming request (RailCom read)

#define ANS_REPORTLOCO			0x0B	///< report controlled loco and serial number (same code as CMD_REPORTLOCO)

#define ANS_SETFUNCEX			0x0D	///< 16 additional functions bits (F16 - F31)
#define ANS_LOCOINFO			0x0E	///< Query Format, speed and functions of a loco without taking control

#define ANS_LOCODB_SELECT		0x10	///< get loco data from Loco DB
#define ANS_LOCODB_NEXT			0x11	///< get next index from Loco DB
#define ANS_LOCODB_PREV			0x12	///< get previous index from Loco DB
#define ANS_LOCODB_DELETE		0x13	///< delete index from Loco DB
#define ANS_LOCODB_DATA0		0x14	///< Data block 0 for Loco DB entry
#define ANS_LOCODB_DATA1		0x15	///< Data block 1 for Loco DB entry
#define ANS_LOCODB_DATA2		0x16	///< Data block 2 for Loco DB entry
#define ANS_LOCODB_DATA3		0x17	///< Data block 3 for Loco DB entry
#define ANS_LOCODB_DATA4		0x18	///< Data block 4 for Loco DB entry

#define ANS_TOSTATUS			0x19	///< Query the status of a turnout (direction and magnetstatus)
#define ANS_TOHINT				0x1A	///< SBIF sends a bit array to indicate it's stored knowledge of turnout directions
#define ANS_BINSTATE			0x1B	///< Switch binary state function in a (DCC-)loco ON or OFF
#define ANS_VERSION				0x1C	///< Send or query the software version
#define ANS_FMAPGET				0x1D	///< read a single function mapping for a loco
#define ANS_FMAPSET				0x1E	///< write/clear a single function mapping for a loco
#define ANS_FMAPLIST			0x1F	///< request the function mapping list for a loco

#define ANS_REQUESTSTATUS		0x20	///< request for a specified status
// status definition
#define STAT_STOP			0		// no signal generation, power shut down
#define STAT_SHORT			1		// same as STAT_STOP except that it automatically entered after a short
#define STAT_HALT			2		// signal is generated with all speeds set to 0 (emergency halt is sent
									// as soon as this state is entered)
#define STAT_GO				3		// normal operation with signal generation
#define STAT_PROGRAM		4		// a loco is programmed on the programming track
#define STAT_PRERESET		5		// the system is about to reset (all users are informed)
#define STAT_RESET			6		// the real reset via EasyNet is performed
#define STAT_TPM			7		// Tams-Programing-Mode (program Tams MM decoder on track)
#define STAT_DOWNLOAD		8		// Download software to EasyNet slaves

#define ANS_CONFIG				0x24	///< system config requests

// Hotplug answers are just the same as the hotplug commands
#define ANS_HOTPLUG0			CMD_HOTPLUG0
#define ANS_HOTPLUG1			CMD_HOTPLUG1
#define ANS_HOTPLUG2			CMD_HOTPLUG2
#define ANS_HOTPLUG3			CMD_HOTPLUG3

// special communication
#define ANS_DEBUGMSG			0x70	///< a debug string of up to BUS_DATALEN characters or terimated by a null character

// different hardware types (coded in the last byte of flash - inside the Bootloader)
#define HW_CENTRALUNIT			0x01	///< Central Unit
#define HW_CONTROL				0x02	///< LokControl / PhoneControl (see HW_PHONECONTROL)
#define HW_SWITCHBOARD			0x03	///< Switchboard Interface (SBIF)
#define HW_XPRESSNET			0x04	///< XpressNet-Adapter

#define HW_PHONECONTROL			0x80	///< HW_CONTROL with PAD7 low

// errorcodes for communication (CMD_ERRMSG, ctrl_errmsg())
#define ERR_OK					0x00	///< not used, just to clearify it
#define ERR_INCOMPATIBLEFORMAT	0x01	///< the (speed-)formats for a traction are incompatible

// system config requests
#define CNFRQ_GETFLAGS			0x00	///< read system flags
#define CNFRQ_SETFLAGS			0x01	///< set flags specified in next byte
#define CNFRQ_CLRFLAGS			0x02	///< clear flags specified in next byte
#define CNFRQ_SHORTTIME			0x03	///< get or set the short-time of the booster
#define CNFRQ_S88MODULES		0x04	///< set (0 .. MAX_S88MODULES) or request (0x7F) number of configured s88 modules
#define CNFRQ_S88SUM			0x05	///< get s88-SUM from module
#define CNFRQ_S88DATA			0x06	///< get s88-DATA from module
#define CNFRQ_TURNOUTFMT		0x07	///< get or set turnout address format
#define CNFRQ_SAVESTATUS		0x08	///< save or restore layout status
#define CNFRQ_SETMFXADR			0x09	///< program a MFX loco to a supplied address
#define CNFRQ_INFO_MFXADR		0x0A	///< program a MFX loco to a supplied address
#define CNFRQ_CONF_RESET		0x0B	///< reset all configuration data
#define CNFRQ_DCC_RDBYTE		0x0C	///< DCC read byte on prog track
#define CNFRQ_DCC_WRBYTE		0x0D	///< DCC write byte on prog track
#define CNFRQ_DCC_RDBIT			0x0E	///< DCC read bit on prog track
#define CNFRQ_DCC_WRBIT			0x0F	///< DCC write bit on prog track
#define CNFRQ_DCC_RDPHYSREG		0x10	///< DCC read physical register on prog track
#define CNFRQ_DCC_WRPHYSREG		0x11	///< DCC write physical register on prog track
#define CNFRQ_DCC_RDPAGEREG		0x12	///< DCC read page register on prog track
#define CNFRQ_DCC_WRPAGEREG		0x13	///< DCC write page register on prog track
#define CNFRQ_DCC_WRADRONLY		0x14	///< DCC write address only on prog track
#define CNFRQ_DCC_PROGRESULT	0x15	///< result of the above CNFRQ_DCC_... programmings
#define CNFRQ_DCC_ONTRACK		0x16	///< DCC OnTrack-Programming

/*
 * Prototypes HW/spi.c
 */
int spi_init (bool bootmode);
bool spi_getblock (en_block *blk);
void spi_sendblock (en_block *blk);
int spi_getchar (void);
int spi_write (uint32_t *data, int len);

/*
 * Prototypes Interface/easynet.c
 */
uint8_t bus_blockcrc (en_block *blk);
bool bus_chkblock (en_block *blk);
struct en_client *en_getClients(void);
void en_sendBlock (uint8_t adr, uint8_t cmd, uint8_t *data);
int en_bootReadBuffer (void *arg, uint8_t *buf, int len);
//void en_bootMode (char *fname);

#endif	/* __EASYNET_H__ */
