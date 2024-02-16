/*
 * bidibserver.c
 *
 *  Created on: 04.03.2021
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

#include <string.h>
#include "rb2.h"
#include "events.h"
#include "timers.h"
#include "config.h"
#include "bidib.h"

static TimerHandle_t diagtimer;			///< the booster diagnose timer

static uint8_t BDBsrv_boosterVoltage (struct bidibnode *n, struct nodefeature *nf, uint8_t val);
static uint8_t BDBsrv_currentLimit (struct bidibnode *n, struct nodefeature *nf, uint8_t val);
static uint8_t BDBsrv_diagnosticTimerChange (struct bidibnode *n, struct nodefeature *nf, uint8_t val);
static uint8_t BDBsrv_setSupport (struct bidibnode *n, struct nodefeature *nf, uint8_t val);

static const struct nodefeature features[] = {
	//-- booster
	{ FEATURE_BST_VOLT_ADJUSTABLE,				1, NULL },		// booster output voltage is adjustable
	{ FEATURE_BST_VOLT,							0, BDBsrv_boosterVoltage },		// booster output voltage setting (unit: V)
	{ FEATURE_BST_CUTOUT_AVAILABLE,				1, NULL },		// booster can do cutout for railcom
	{ FEATURE_BST_CUTOUT_ON,					1, BDBnf_featureWriteBool },		// cutout is enabled
//	{ FEATURE_BST_TURNOFF_TIME,					0, NULL },		// time in ms until booster turns off in case of a short (unit 2ms)
//	{ FEATURE_BST_INRUSH_TURNOFF_TIME,			0, NULL },		// time in ms until booster turns off in case of a short after the first power up (unit 2ms)
	{ FEATURE_BST_AMPERE_ADJUSTABLE,			1, NULL },		// booster output current is adjustable
	{ FEATURE_BST_AMPERE,						0, BDBsrv_currentLimit },	// booster output current value (special coding, 6.400mA)
	{ FEATURE_BST_CURMEAS_INTERVAL,				200, BDBsrv_diagnosticTimerChange },	// current update interval
	{ FEATURE_BST_INHIBIT_AUTOSTART,			0, NULL },		// 1: Booster does no automatic BOOST_ON when DCC at input wakes up.
	{ FEATURE_BST_INHIBIT_LOCAL_ONOFF,			0, NULL },		// 1: Booster announces local STOP/GO key stroke only, no local action

	//-- bidi detection
//	{ FEATURE_BM_ADDR_DETECT_AVAILABLE,			0, NULL },		// detector ic capable to detect loco address
//	{ FEATURE_BM_ADDR_DETECT_ON,				0, NULL },		// address detection enabled
//	{ FEATURE_BM_ADDR_AND_DIR,					0, NULL },		// addresses contain direction
//	{ FEATURE_BM_ISTSPEED_AVAILABLE,			0, NULL },		// speed messages available
//	{ FEATURE_BM_ISTSPEED_INTERVAL,				0, NULL },		// speed update interval (unit 10ms)
	{ FEATURE_BM_CV_AVAILABLE,					1, NULL },		// CV readback available
	{ FEATURE_BM_CV_ON,							1, NULL },		// CV readback enabled
//	{ FEATURE_BM_DYN_STATE_INTERVAL,			0, NULL },		// transmit interval of MSG_BM_DYN_STATE (unit 100ms)
//	{ FEATURE_BM_RCPLUS_AVAILABLE,				0, NULL },		// 1: RailcomPlus messages available
//	{ FEATURE_BM_POSITION_ON,					0, NULL },		// position messages enabled
//	{ FEATURE_BM_POSITION_SECACK,				0, NULL },		// secure position ack interval (unit: 10ms), 0: none

	//-- accessory
//	{ FEATURE_ACCESSORY_COUNT,					0, NULL },		// number of objects
//	{ FEATURE_ACCESSORY_SURVEILLED,				0, NULL },		// 1: announce if operated outside bidib
//	{ FEATURE_ACCESSORY_MACROMAPPED,			0, NULL },		// 1..n: no of accessory aspects are mapped to macros

	//-- control
//	{ FEATURE_CTRL_INPUT_COUNT,					0, NULL },		// number of inputs for keys
//	{ FEATURE_CTRL_INPUT_NOTIFY,				0, NULL },		// 1: report a keystroke to host
//	{ FEATURE_CTRL_SWITCH_COUNT,				0, NULL },		// number of switch ports (direct controlled)
//	{ FEATURE_CTRL_LIGHT_COUNT,					0, NULL },		// number of light ports (direct controlled)
//	{ FEATURE_CTRL_SERVO_COUNT,					0, NULL },		// number of servo ports (direct controlled)
//	{ FEATURE_CTRL_SOUND_COUNT,					0, NULL },		// number of sound ports (direct controlled)
//	{ FEATURE_CTRL_MOTOR_COUNT,					0, NULL },		// number of motor ports (direct controlled)
//	{ FEATURE_CTRL_ANALOGOUT_COUNT,				0, NULL },		// number of analog ports (direct controlled)
//	{ FEATURE_CTRL_STRETCH_DIMM,				0, NULL },		// additional time stretch for dimming (for light ports)
//	{ FEATURE_CTRL_BACKLIGHT_COUNT,				0, NULL },		// number of backlight ports (intensity direct controlled)
//	{ FEATURE_CTRL_MAC_LEVEL,					0, NULL },		// supported macro level
//	{ FEATURE_CTRL_MAC_SAVE,					0, NULL },		// number of permanent storage places for macros
//	{ FEATURE_CTRL_MAC_COUNT,					0, NULL },		// number of macros
//	{ FEATURE_CTRL_MAC_SIZE,					0, NULL },		// length of each macro (entries)
//	{ FEATURE_CTRL_MAC_START_MAN,				0, NULL },		// (local) manual control of macros enabled
//	{ FEATURE_CTRL_MAC_START_DCC,				0, NULL },		// (local) dcc control of macros enabled
//	{ FEATURE_CTRL_PORT_QUERY_AVAILABLE,		0, NULL },		// 1: node will answer to output queries via MSG_LC_PORT_QUERY
//	{ FEATURE_SWITCH_CONFIG_AVAILABLE,			0, NULL },		// (deprecated, version >= 0.6 uses availability of PCFG_IO_CTRL) 1: node has possibility to configure switch ports
//	{ FEATURE_CTRL_PORT_FLAT_MODEL,				0, NULL },		// node uses flat port model, "low" number of addressable ports
//	{ FEATURE_CTRL_PORT_FLAT_MODEL_EXTENDED,	0, NULL },		// node uses flat port model, "high" number of addressable ports

	//-- dcc gen
//	{ FEATURE_GEN_SPYMODE,						0, NULL },		// 1: watch bidib handsets
	{ FEATURE_GEN_WATCHDOG,						0, NULL },		// 0: no watchdog, 1: permanent update of MSG_CS_SET_STATE required, unit 100ms
//	{ FEATURE_GEN_DRIVE_ACK,					0, NULL },		// not used
//	{ FEATURE_GEN_SWITCH_ACK,					0, NULL },		// not used
//	{ FEATURE_GEN_LOK_DB_SIZE,					0, NULL },		//
//	{ FEATURE_GEN_LOK_DB_STRING,				0, NULL },		//
	{ FEATURE_GEN_POM_REPEAT,					2, NULL },		// number of POM repeats
	{ FEATURE_GEN_DRIVE_BUS,					1, NULL },		// 1: this node drive the DCC bus.
//	{ FEATURE_GEN_LOK_LOST_DETECT,				0, NULL },		// 1: command station annouces lost loco
	{ FEATURE_GEN_NOTIFY_DRIVE_MANUAL,			3, NULL },		// Bit 0: DCC gen reports manual drive operation, Bit 1: same for accessories
	{ FEATURE_GEN_START_STATE,					0, NULL },		// 1: power up state, 0=off, 1=on
//	{ FEATURE_GEN_RCPLUS_AVAILABLE,				0, NULL },		// (deprecated) 1: supports rcplus messages
	{ FEATURE_GEN_EXT_AVAILABLE,			 	0, BDBsrv_setSupport },		/* bitfield ext. support (will be filled dynamically at startup):
																				0: RailCom+
																				1: M4
																				2: DCCA
																				3: DCC-SDF
																				4: MM
																				7: MSG_CS_QUERY
																				others: reserved */
//	{ FEATURE_CELL_NUMBER,						0, NULL },		// (logical) reference mark of generator (0=single system, 1..n:'area mark')
//	{ FEATURE_RF_CHANNEL,						0, NULL },		// used rf channel, 0..83 channel assignment in 2.4 GHz band

//	{ FEATURE_STRING_DEBUG,						0, NULL },		// use namespace 1 for debug strings, 0:n.a./silent (default); 1=mode 1
	{ FEATURE_STRING_SIZE,					   24, NULL },		// length of user strings, 0:n.a (default); allowed 8..24
	{ FEATURE_RELEVANT_PID_BITS,				8, NULL },		// how many bits of 'vendor32' are relevant for PID (default 16, LSB aligned)
	{ FEATURE_FW_UPDATE_MODE,					0, NULL },		// 0: no fw-update, 1: intel hex (max. 10 byte / record)
//	{ FEATURE_EXTENSION,						0, NULL },		// 1: reserved for future expansion
};

/**
 * A local decoder table for the type of decoder that is addressed
 * in some context (i.e. MSG_CS_POM)
 */
static const dec_type decodertype[] = {
	DECODER_DCC_MOBILE, DECODER_DCC_MOBILE, DECODER_DCC_ACC, DECODER_DCC_EXT
};

static uint8_t BDBsrv_boosterVoltage (struct bidibnode *n, struct nodefeature *nf, uint8_t val)
{
	(void) n;

	if (!nf) return 0;
	val = ts_setVoltage(val * 10) / 10;
	cnf_triggerStore(__func__);

	return val;
}

static uint8_t BDBsrv_currentLimit (struct bidibnode *n, struct nodefeature *nf, uint8_t val)
{
	int mA;

	(void) n;

	if (!nf) return 0;
	mA = bidib_code2current(val);
	mA = ts_setCurrentMilliAmpere(mA);
	cnf_triggerStore(__func__);

	return bidib_current2code(mA);
}

#define EXT_RCPLUS		0x01
#define EXT_M4			0x02
#define EXT_DCCA		0x04
#define EXT_DCC_SDF		0x08
#define EXT_MM			0x10
#define EXT_UNDEF5		0x20
#define EXT_UNDEF6		0x40
#define EXT_CSQUERY		0x80
#define EXT_SUPPORTED_FEATURES	(EXT_M4 | EXT_DCCA | EXT_DCC_SDF | EXT_MM | EXT_CSQUERY)
#define EXT_FIXED_FEATURES	(EXT_DCC_SDF | EXT_MM | EXT_CSQUERY)

/**
 * Enable / Disable certain support flags (see feature FEATURE_GEN_EXT_AVAILABLE)
 */
static uint8_t BDBsrv_setSupport (struct bidibnode *n, struct nodefeature *nf, uint8_t val)
{
	struct fmtconfig *fmtcfg;
	uint8_t diff;

	(void) n;

	if (!nf) return 0;
	fmtcfg = cnf_getFMTconfig();
	val &= EXT_SUPPORTED_FEATURES;	// mask out unused bits #5, #6 and currently unsupported flag #0 RailCom+
	val |= EXT_FIXED_FEATURES;
	diff = val ^ nf->value;
	if (diff & EXT_M4) {		// switch on/off M4 support
		if (val & EXT_M4) fmtcfg->sigflags |= SIGFLAG_M3ENABLED;
		else fmtcfg->sigflags &= ~SIGFLAG_M3ENABLED;
	}
	if (diff & EXT_DCCA) {		// switch on/off DCC-A support
		if (val & EXT_DCCA) fmtcfg->sigflags |= SIGFLAG_DCCA;
		else fmtcfg->sigflags &= ~SIGFLAG_DCCA;
	}
	if (diff) cnf_triggerStore(__func__);

	return val;
}

static void BDBsrv_identify (struct bidibnode *n, bidibmsg_t *msg)
{
	(void) n;

	if (msg->datalen >= 1) {
		bidib_identify(msg->data[0]);
	}
}

static void BDBsrv_resetSystem (struct bidibnode *n, bidibmsg_t *msg)
{
	(void) msg;
	(void) n;

	reboot();
}

static void BDBsrv_sysDisable (struct bidibnode *n, bidibmsg_t *msg)
{
	(void) msg;
	(void) n;

	bidib_sysDisable();
	BDBnode_reportEnable(false);
}

static void BDBsrv_sysEnable (struct bidibnode *n, bidibmsg_t *msg)
{
	(void) msg;
	(void) n;

	bidib_sysEnable();
	BDBnode_reportEnable(true);
}

static bidibmsg_t *BDBsrv_boosterStatus (struct bidibnode *n)
{
	uint8_t data[2];

	switch (rt.tm) {
		case TM_STOP:		data[0] = BIDIB_BST_STATE_OFF; break;
		case TM_SHORT:		data[0] = BIDIB_BST_STATE_OFF_SHORT; break;
		case TM_HALT:		data[0] = BIDIB_BST_STATE_ON; break;
		case TM_GO:			data[0] = BIDIB_BST_STATE_ON; break;
		case TM_DCCPROG:	data[0] = BIDIB_BST_STATE_ON; break;
		case TM_TAMSPROG:	data[0] = BIDIB_BST_STATE_ON; break;
		default:			data[0] = BIDIB_BST_STATE_OFF; break;
	}
	return bidib_genMessage(n, MSG_BOOST_STAT, 1, data);
}

static void BDBsrv_boosterOff (struct bidibnode *n, bidibmsg_t *msg)
{
	bool sigon;

	(void) n;

	if (msg->datalen < 1) return;
	sigon = (rt.tm == TM_HALT || rt.tm == TM_GO || rt.tm == TM_SIGON);
	if (sigon) {
		if (rt.tm == TM_SIGON) event_fire(EVENT_SYS_STATUS, SYSEVENT_SIGON, NULL);		// sig_setMode() won't fire event if status is not changed
		else sig_setMode(TM_SIGON);
	} else {
		if (rt.tm == TM_STOP) event_fire(EVENT_SYS_STATUS, SYSEVENT_STOP, NULL);		// sig_setMode() won't fire event if status is not changed
		else sig_setMode(TM_STOP);
	}
}

static void BDBsrv_boosterOn (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m = NULL;
	uint8_t data[2];
	bool sigon;

	if (msg->datalen < 1) return;
	// check if we already have a valid signal - if not, this should be treatet as a soft error
	sigon = (rt.tm == TM_HALT || rt.tm == TM_GO || rt.tm == TM_SIGON);

	if (!sigon) {
		data[0] = BIDIB_BST_STATE_OFF_NO_DCC;
		m = bidib_genMessage(n, MSG_BOOST_STAT, 1, data);
		netBDB_postMessages(m);
	} else {
		if (rt.tm == TM_GO) event_fire(EVENT_SYS_STATUS, SYSEVENT_GO, NULL);		// sig_setMode() won't fire event if status is not changed
		else sig_setMode(TM_GO);
	}
}

static void BDBsrv_boosterDiag (struct bidibnode *n)
{
	uint8_t data[8];

	if ((bidib_opmode() == BIDIB_SERVER) && !(bidib_isSysDisabled())) {
		data[0] = BIDIB_BST_DIAG_I;
		data[1] = MAINBST_ISON() ? bidib_current2code(an_getTrackCurrent()) : 0;
		data[2] = BIDIB_BST_DIAG_V;
		data[3] = ts_getVoltage();
		data[4] = BIDIB_BST_DIAG_T;
		data[5] = an_getTemperature();
		netBDB_postMessages(bidib_genMessage(n, MSG_BOOST_DIAGNOSTIC, 6, data));
	}
}

static void BDBsrv_boosterQuery (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m = NULL;

	(void) msg;

	m = BDBsrv_boosterStatus(n);
	netBDB_postMessages(m);
	BDBsrv_boosterDiag(n);
}

static void BDBsrv_setState (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m = NULL;
	struct nodefeature *ft;
	uint8_t data[2];

	if (msg->datalen < 1) return;
	switch (msg->data[0]) {
		case BIDIB_CS_STATE_OFF:
			sig_setMode(TM_STOP);
			break;
		case BIDIB_CS_STATE_STOP:
			sig_setMode(TM_HALT);		// send an emergency stop for all locos
			break;
		case BIDIB_CS_STATE_SOFTSTOP:
			sig_setMode(TM_HALT);
			break;
		case BIDIB_CS_STATE_GO:
		case BIDIB_CS_STATE_GO_IGN_WD:
			if ((ft = bidib_readFeature(LOCAL_NODE(), FEATURE_BST_INHIBIT_AUTOSTART)) != NULL && ft->value > 0) {
				if (rt.tm != TM_GO && rt.tm != TM_HALT) {	// in this case the boosters should stay on
					sig_setMode(TM_SIGON);
				}
			} else {
				sig_setMode(TM_GO);
			}
			break;
		case BIDIB_CS_STATE_PROG:
			sig_setMode(TM_STOP);
			break;
		case BIDIB_CS_STATE_QUERY:		// only query the track mode
			break;
		default:
			return;
	}
	switch (rt.tm) {
		case TM_STOP:		data[0] = BIDIB_CS_STATE_OFF; break;
		case TM_SHORT:		data[0] = BIDIB_CS_STATE_OFF; break;
		case TM_HALT:		data[0] = BIDIB_CS_STATE_STOP; break;
		case TM_SIGON:		data[0] = BIDIB_CS_STATE_GO; break;
		case TM_GO:			data[0] = BIDIB_CS_STATE_GO; break;
		case TM_DCCPROG:	data[0] = BIDIB_CS_STATE_PROGBUSY; break;
		case TM_TAMSPROG:	data[0] = BIDIB_CS_STATE_PROGBUSY; break;
		default:			data[0] = BIDIB_CS_STATE_OFF; break;
	}
	m = bidib_genMessage(n, MSG_CS_STATE, 1, data);
	netBDB_postMessages(m);
}

/**
 * Handle the MSG_CS_DRIVE comand to control loco operation.
 *
 * \param n			the node we got the request from
 * \param msg		the message as we received it
 */
static void BDBsrv_loco (struct bidibnode *n, bidibmsg_t *msg)
{
	locoT *lo;
	ldataT *l;
	bidibmsg_t *m;
	uint8_t data[4], speed;
	uint16_t adr;
	enum fmt fmt;
	int db_speeds, req_speeds;

	data[0] = msg->data[0];		// preset answer with address bytes
	data[1] = msg->data[1];

	if (msg->datalen < 9) {
		m = bidib_errorMessage(n, BIDIB_ERR_SIZE, 1, &msg->seq);
	} else if (rt.tm != TM_HALT && rt.tm != TM_GO && rt.tm != TM_SIGON) {	// track output not active - loco cannot be updated
		data[2] = 0;
		m = bidib_genMessage(n, MSG_CS_DRIVE_ACK, 3, data);
	} else {
		adr = msg->data[1] << 8 | msg->data[0];
		fmt = bidib_code2fmt(msg->data[2] & 0x0F);
		lo = db_getLoco(adr, false);	// lo will be NULL if a new loco is to be created in the next step
		l = loco_call(adr, true);		// get the loco in the refresh stack and create a new loco if it didn't exist before
		if (lo == NULL) {		// we had to create a new loco from scratch - set all format aspects as commanded
			db_setLocoFmt(adr, fmt);
		} else {				// the loco already exits - change format only if the speeds don't match
			req_speeds = db_getSpeeds(fmt);
			db_speeds = db_getSpeeds(lo->fmt);
			if (req_speeds == 27) req_speeds = 28;	// make all 27 speeds equal to 28 speeds (from request)
			if (db_speeds == 27) db_speeds = 28;	// make all 27 speeds equal to 28 speeds (from loco DB)
			if (db_speeds != req_speeds) db_setLocoFmt(adr, fmt);	// if speeds don't match, replace the format
		}

		if (msg->data[3] == 0) {		// remove loco from refresh stack
			loco_remove(l);
			data[2] = 1;
			m = bidib_genMessage(n, MSG_CS_DRIVE_ACK, 3, data);
		} else if (l) {
			speed = bidib_msg2speed(msg->data[4], l->loco->fmt);
			if (msg->data[3] & BIDIB_CS_DRIVE_SPEED_BIT) {
				if ((speed & 0x7F) == 0x7F) loco_emergencyStop(adr);
				else loco_setSpeed(adr, speed);
			}
			if (msg->data[3] & BIDIB_CS_DRIVE_F0F4_BIT) loco_setFuncMasked(adr, ((msg->data[5] << 1) & 0x1E) | ((msg->data[5] >> 4) & 0x01), FUNC_F0_F4);
			switch (msg->data[3] & (BIDIB_CS_DRIVE_F5F8_BIT | BIDIB_CS_DRIVE_F9F12_BIT)) {
				case BIDIB_CS_DRIVE_F5F8_BIT:
					loco_setFuncMasked(adr, msg->data[6] << 5, FUNC_F5_F8);
					break;
				case BIDIB_CS_DRIVE_F9F12_BIT:
					loco_setFuncMasked(adr, msg->data[6] << 5, FUNC_F9_F12);
					break;
				case (BIDIB_CS_DRIVE_F5F8_BIT | BIDIB_CS_DRIVE_F9F12_BIT):
					loco_setFuncMasked(adr, msg->data[6] << 5, FUNC_F5_F12);
					break;
			}
			if (msg->data[3] & BIDIB_CS_DRIVE_F13F20_BIT) loco_setFuncMasked(adr, msg->data[7] << 13, FUNC_F13_F20);
			if (msg->data[3] & BIDIB_CS_DRIVE_F21F28_BIT) loco_setFuncMasked(adr, msg->data[8] << 21, FUNC_F21_F28);
			data[2] = 1;
			m = bidib_genMessage(n, MSG_CS_DRIVE_ACK, 3, data);
		} else {	// loco not set for various reasons - report failure
			data[2] = 0;
			m = bidib_genMessage(n, MSG_CS_DRIVE_ACK, 3, data);
		}
	}

	if (m) netBDB_postMessages(m);
}

/**
 * Handle the MSG_CS_BIN_STATE comand to control loco functions.
 *
 * \param n			the node we got the request from
 * \param msg		the message as we received it
 */
static void BDBsrv_binstate (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	uint8_t text[64];

	// Not implemented yet!
	(void) msg;
	bidib_addString(text, "Function not implemented yet!", sizeof(text));
	m = bidib_errorMessage(n, BIDIB_ERR_TXT, *text + 1, text);
	if (m) netBDB_postMessages(m);
}

/**
 * Handle the MSG_CS_ACCESSORY command to switch accessory and extended accessory
 * decoder outputs.
 *
 * BiDiB addresses the decoder outputs as DCC would have to do it. As a basic
 * accessory decoder usually has 4 outputs and decoder address 0 can't be
 * used, the first usable turnout address is 4 (output 0 of decoder 1). Since
 * the numbering is traditionally zero-based and we internally use one-based
 * turnout addresses, there is a difference of only THREE between the BiDiB
 * command and our trnt_*() functions.
 *
 * \param n			the node we got the request from
 * \param msg		the message as we received it
 */
static void BDBsrv_accessory (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	uint8_t data[4];
	uint16_t adr;
	bool extacc, timing;
	TickType_t tim;

	data[0] = msg->data[0];		// preset answer with address bytes
	data[1] = msg->data[1];

	if (msg->datalen < 4) {
		m = bidib_errorMessage(n, BIDIB_ERR_SIZE, 1, &msg->seq);
	} else if (rt.tm != TM_HALT && rt.tm != TM_GO) {	// track output not active - accessory cannot be switched
		data[2] = 0;
		m = bidib_genMessage(n, MSG_CS_ACCESSORY_ACK, 3, data);
	} else {
		adr = msg->data[1] << 8 | msg->data[0];
		log_msg(LOG_INFO, "%s() RAW address %d\n", __func__, adr);
		if (cnf_getconfig()->sysflags & SYSFLAG_ACC_LOGICAL) {		// Rockrail (?) sends zero-based addresses instead of DCC system addresses
			adr += 1;			// User turnout address '1' is transmitted as '0' -> fix
		} else {
			adr -= 4 - 1;		// BiDiB address 'correcture': User turnout address '1' is transmitted as '4' (as in DCC track format)
		}
		extacc = !!(msg->data[2] & 0x80);
		timing = !!(msg->data[2] & 0x40);
		if (timing) {
			if (msg->data[3] & 0x80) {		// timing in seconds
				tim = pdMS_TO_TICKS((msg->data[3] & 0x7F) * 1000);
			} else {						// timing in 100ms units
				tim = pdMS_TO_TICKS((msg->data[3] & 0x7F) * 100);
			}
			if (extacc) {
				// NOT ALLOWED! - will be ignored and acknowledged
			} else {
				trnt_switchTimed(adr, (msg->data[2] & 0x1F) == 0, tim);
			}
		} else {
			if (extacc) {
				xacc_aspect(adr, msg->data[2] & 0x1F);
			} else {
				trnt_switch(adr, (msg->data[2] & 0x1F) == 0, (msg->data[2] & 0x20) != 0);
			}
		}
		data[2] = 1;
		m = bidib_genMessage(n, MSG_CS_ACCESSORY_ACK, 3, data);
	}
	if (m) netBDB_postMessages(m);
}

static bool BDBsrv_pomAnswer (struct decoder_reply *dm, flexval priv)
{
	struct bidibnode *n;
	bidibmsg_t *m;
	uint8_t data[6];

	n = (struct bidibnode *) priv.p;

	if (n && dm && dm->mt == DECODERMSG_POM && dm->len >= 1) {
		log_msg (LOG_INFO, "%s(): ADR %d CV %ld len %d VAL[0]=%d\n", __func__, dm->adr, dm->cva.cv, dm->len, dm->data[0]);
		data[0] = dm->adr & 0xFF;
		data[1] = (dm->adr >> 8) & 0x3F;
		if (dm->dtype == DECODER_DCC_ACC) data[1] |= (0b01 << 6);
		if (dm->dtype == DECODER_DCC_EXT) data[1] |= (0b11 << 6);
		data[2] = dm->cva.cv & 0xFF;
		data[3] = (dm->cva.cv >> 8) & 0xFF;
		data[4] = dm->data[0];
		m = bidib_genMessage(n, MSG_BM_CV, 5, data);
		if (m) netBDB_postMessages(m);
	}
	return false;
}

static void BDBsrv_pom (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	uint8_t data[6], ack, mid;
	uint16_t adr;
	uint32_t did, cv;
	dec_type dt;
	flexval fv;

	if (msg->datalen < 10) {	// minimum of one data byte results in 10 bytes command length, more may needed for multi-byte commands
		m = bidib_errorMessage(n, BIDIB_ERR_SIZE, 1, &msg->seq);
	} else {
		adr = msg->data [0] | (msg->data[1] << 8);					// standard decoder addressing
		did = adr | (msg->data[2] << 16) | (msg->data[3] << 24);	// decoder ID addressing (together with MID - manufacturer ID)
		mid = msg->data[4];											// the MID in case of decoder ID addressing
		cv = msg->data[6] | (msg->data[7] << 8) | (msg->data[8] << 16);		// 24 bit CV addresses only for XPoM
		ack = 0;													// this means a failure
		if (mid == 0) {
			dt = decodertype[(adr & 0xC000) >> 14];
		} else {
			dt = DECODER_ANY;
		}

		(void) did;		// currently not used - keep compiler happy
		fv.p = n;

		// prepare answer with MSG_CS_POM_ACK
		memcpy (data, msg->data, 5);		// copy the first five bytes of message data
		if (rt.tm == TM_GO && mid == 0) {	// we need to be in GO state, we currently only support standard decoder addressing
			switch (msg->data[5]) {
				case BIDIB_CS_POM_RD_BLOCK:
					break;
				case BIDIB_CS_POM_RD_BYTE:
					dccpom_readByte(adr & 0x3FFF, dt, cv, BDBsrv_pomAnswer, fv);
					ack = 1;
					break;
				case BIDIB_CS_POM_WR_BIT:
					dccpom_writeBit(adr & 0x3FFF, dt, cv, msg->data[9] & 0x07, !!(msg->data[9] & 0x08), BDBsrv_pomAnswer, fv);
					break;
				case BIDIB_CS_POM_WR_BYTE:
					dccpom_writeByte(adr & 0x3FFF, dt, cv, msg->data[9], BDBsrv_pomAnswer, fv);
					break;
				case BIDIB_CS_XWR_BYTE1:
					break;
				case BIDIB_CS_XWR_BYTE2:
					break;
				case BIDIB_CS_xPOM_RD_BLOCK:
					break;
				case BIDIB_CS_xPOM_WR_BIT:
					break;
				case BIDIB_CS_xPOM_WR_BYTE1:
					break;
				case BIDIB_CS_xPOM_WR_BYTE2:
					break;
				case BIDIB_CS_xPOM_WR_BYTE3:
					break;
				case BIDIB_CS_xPOM_WR_BYTE4:
					break;
			}
		} else {			// addressing via decoder ID (not implemented yet)
			/* error! ACK stays zero */
		}
		data[5] = ack;
		m = bidib_genMessage(n, MSG_CS_POM_ACK, 6, data);
	}
	if (m) netBDB_postMessages(m);
}

/**
 * Prepare a MSG_CS_DRIVE_STATE for the given loco address.
 *
 * \param adr		the address of the loco (or other object) to report
 * \param opcode	the opcode parameter encodes, if this is a single report or a list report
 * 					and with list reports, if this is the last report or if there are more
 * 					objects to report (see BiDiB.org)
 * \return			a constructed MS_CS_DRIVE_STATE block or NULL in case of error
 */
static bidibmsg_t *BDBsrv_driveState (uint16_t adr, uint8_t opcode)
{
	ldataT *l;
	locoT *loco;
	uint8_t data[10];

	data[0] = opcode;
	switch (opcode & 0x0F) {
		case 1:
			if ((l = loco_call(adr, false)) == NULL) {
				if ((loco = db_getLoco(adr, false)) == NULL) loco = db_getLoco(0, false);	// as a last ressort, we take the default loco
				data[1] = adr & 0xFF;
				data[2] = (adr >> 8) & 0xFF;
				data[3] = bidib_fmt2code(loco->fmt);
				data[4] = 0x00;									// function / speed validity bitmap
				data[5] = 0x80;									// speed (FWD with FS0)
				data[6] = data[7] =	data[8] = data[9] = 0;		// the functions are all zero
			} else {
				loco = l->loco;
				data[1] = loco->adr & 0xFF;
				data[2] = (loco->adr >> 8) & 0xFF;
				data[3] = bidib_fmt2code(loco->fmt);
				data[4] = BIDIB_CS_DRIVE_SPEED_BIT | BIDIB_CS_DRIVE_F0F4_BIT;
				if (loco->maxfunc >= 5) data[4] |= BIDIB_CS_DRIVE_F5F8_BIT;
				if (loco->maxfunc >= 9) data[4] |= BIDIB_CS_DRIVE_F9F12_BIT;
				if (loco->maxfunc >= 13) data[4] |= BIDIB_CS_DRIVE_F13F20_BIT;
				if (loco->maxfunc >= 21) data[4] |= BIDIB_CS_DRIVE_F21F28_BIT;
				data[5] = bidib_speed2msg(l->speed, l->loco->fmt);
				data[6] = ((l->funcs[0] & FUNC_F1_F4) >> 1) | ((l->funcs[0] & FUNC_LIGHT) << 4);	// 3 reserved MSBs, FL, F4, F3, F2, F1
				data[7] = (l->funcs[0] & FUNC_F5_F12) >> 5;
				data[8] = (l->funcs[0] & FUNC_F13_F20) >> 13;
				data[9] = (l->funcs[0] & FUNC_F21_F28) >> 21;
			}
			break;
		default:
			return NULL;
	}
	return bidib_genMessage(LOCAL_NODE(), MSG_CS_DRIVE_STATE, 10, data);
}

static void BDBsrv_locoReportThread (void *pvParameter)
{
	bidibmsg_t *m;
	ldataT *l;

	(void) pvParameter;

	l = NULL;
	while ((l = loco_iterateNext(l)) != NULL) {
		m = BDBsrv_driveState(l->loco->adr, (l->next) ? 0x81 : 0xC1);
		if (m) netBDB_postMessages(m);
	}
	vTaskDelete(NULL);		// end the task
}

static void BDBsrv_query (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m = NULL;
	uint16_t adr;

	if (msg->datalen < 1) {			// minimum of one data byte
		m = bidib_errorMessage(n, BIDIB_ERR_SIZE, 1, &msg->seq);
	} else {
		if (msg->datalen < 3) {			// no address specified (should be a list query)
			adr = 0;
		} else {						// an address is specified - but it still could be a list query
			adr = msg->data[1] | (msg->data[2] << 8);
		}

		switch (msg->data[0] & 0x0F) {
			case 1:		// object type is "loco"
				if (msg->data[0] & 0x80) {		// report all locos (list report, address is ignored)
				    xTaskCreate(BDBsrv_locoReportThread, "BiDiB-Report", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
				} else if (adr > 0) {			// only report the addressed loco
					m = BDBsrv_driveState(adr, 0x41);	// END-OF-LIST + object type (1 = loco)
				} else {						// reporting single loco with no address is not supported
					m = bidib_errorMessage(n, BIDIB_ERR_PARAMETER, 1, &msg->seq);
				}
				break;
			default:
				m = bidib_errorMessage(n, BIDIB_ERR_PARAMETER, 1, &msg->seq);
				break;
		}
	}
	if (m) netBDB_postMessages(m);
}

static void BDBsrv_progCB (int rc, void *priv)
{
	int cv = (int) priv;
	bidibmsg_t *m;
	uint8_t data[6];
	int bytes;

	data[1] = 0;				// time is always zero here (more or less unsupported)
	data[2] = cv & 0xFF;		// copy CV address to answer
	data[3] = (cv >> 8) & 0x03;
	bytes = 4;					// standard length without data byte

	switch (rc) {
		case ERR_CV_UNSUPPORTED:
			data[0] = BIDIB_CS_PROG_NO_ANSWER;
			break;
		case ERR_CV_COMPARE:
			data[0] = BIDIB_CS_PROG_VERIFY_FAILED;
			break;
		case ERR_SHORT:
			data[0] = BIDIB_CS_PROG_SHORT;
			break;
		case ERR_INTERRUPTED:
			data[0] = BIDIB_CS_PROG_STOPPED;
			break;
		default:
			if (rc >= 0) {
				data[0] = BIDIB_CS_PROG_OKAY;
				data[4] = rc;
				bytes = 5;
			} else {
				data[0] = BIDIB_CS_PROG_NO_ANSWER;
			}
			break;
	}
	m = bidib_genMessage(LOCAL_NODE(), MSG_CS_PROG_STATE, bytes, data);
	if (m) netBDB_postMessages(m);
}

static void BDBsrv_prog (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	uint8_t data[6];
	int cv;
	int minlen;

	minlen = 3;		// at least OPCODE and CV address are required
	if ((msg->data[0] == BIDIB_CS_PROG_RDWR_BIT) || (msg->data[0] == BIDIB_CS_PROG_WR_BYTE)) minlen = 4;	// data byte is also needed
	if (msg->datalen < minlen) {
		m = bidib_errorMessage(n, BIDIB_ERR_SIZE, 1, &msg->seq);
	} else {
		cv = msg->data[1] | (msg->data[2] << 8);		// 10 bit CV address

		data[0] = BIDIB_CS_PROG_START;	// tell the controller, that we received the command
		data[1] = 0;					// time is invalid (unsuported)
		data[2] = msg->data[1];			// copy CV address to answer
		data[3] = msg->data[2];

//		if (rt.tm == TM_DCCPROG || rt.tm == TM_STOP) {	// we need to be in DCC programming state (or stopped - we will switch to prgramming)
			switch (msg->data[0]) {
				case BIDIB_CS_PROG_BREAK:
					break;
				case BIDIB_CS_PROG_QUERY:
					if (rt.tm == TM_DCCPROG) data[0] = BIDIB_CS_PROG_RUNNING;
					break;
				case BIDIB_CS_PROG_RD_BYTE:
					dccpt_cvReadByteBG(cv & 0x3FF, BDBsrv_progCB, (void *) cv);
					break;
				case BIDIB_CS_PROG_RDWR_BIT:
					if (msg->data[3] & 0x10) {	// write bit
						dccpt_cvWriteBitBG(cv & 0x3FF, msg->data[3] & 0x07, msg->data[3] & 0x08, BDBsrv_progCB, (void *) cv);
					} else {					// verify bit
						dccpt_cvVerifyBitBG(cv & 0x3FF, msg->data[3] & 0x07, msg->data[3] & 0x08, BDBsrv_progCB, (void *) cv);
					}
					break;
				case BIDIB_CS_PROG_WR_BYTE:
					dccpt_cvWriteByteBG(cv & 0x3FF, msg->data[3], BDBsrv_progCB, (void *) cv);
					break;
			}
//		}
		m = bidib_genMessage(n, MSG_CS_PROG_STATE, 4, data);
	}
	if (m) netBDB_postMessages(m);
}

#if 1		// Vorbereitung auf Tabellen-Dekoder in BDBsrv_upstream()
static void BDBnode_identifyState (struct bidibnode *n, bidibmsg_t *m)
{
	if (m->data[0]) n->flags |= NODEFLG_IDENTIFY;
	else n->flags &= ~NODEFLG_IDENTIFY;
	BDBnode_nodeEvent();
}

static void BDBnode_featureCount (struct bidibnode *n, bidibmsg_t *m)
{
	int cnt;

	cnt = m->data[0];
	if (n->features) free(n->features);
	n->features = NULL;
	n->featureidx = 0;
	if (cnt > 0) {
		if ((n->features = calloc (cnt, sizeof(*n->features))) != NULL) {
			n->featurecount = cnt;
		} else {
			n->featurecount = 0;
		}
	}
}

static void BDBnode_feature (struct bidibnode *n, bidibmsg_t *m)
{
	struct nodefeature *nf;

	if (n->features && m->datalen >= 2) {
		if (n->state != NS_READ_FEATURES && n->state != NS_AUTOREAD_FEATURES) {
			if ((nf = bidib_readFeature(n, m->data[0])) != NULL) {
				nf->value = m->data[1];
			}
		} else {
			if (n->featureidx < n->featurecount) {
				n->features[n->featureidx].feature = m->data[0];
				n->features[n->featureidx].value = m->data[1];
				n->featureidx++;
			}
			if (n->featureidx >= n->featurecount) {
				bidib_sortFeature(n);
			}
		}
	}
}

static void BDBnode_string (struct bidibnode *n, bidibmsg_t *m)
{
	int len;

	if (m->data[0] == 0) {		// Namespace: normal Strings (Product adn Username)
		switch (m->data[1]) {
			case 0:		// Productname
				len = m->data[2];
				if (len > MAX_PRODUCT_STRING) len = MAX_PRODUCT_STRING;
				strncpy (n->product, (char *) &m->data[3], len);
				n->product[len] = 0;
				break;
			case 1:		// Username
				len = m->data[2];
				if (len > MAX_USER_STRING) len = MAX_USER_STRING;
				strncpy (n->user, (char *) &m->data[3], len);
				n->user[len] = 0;
				break;
		}
	}
}

static void BDBnode_ntabCount (struct bidibnode *n, bidibmsg_t *m)
{
	(void) m;

	BDBnode_freeNodeList(n->children);
	n->children = NULL;
	BDBnode_nodeEvent();
}

static void BDBnode_nodeTab (struct bidibnode *n, bidibmsg_t *m)
{
	struct bidibnode *bn;

	if (m->data[1] != 0) {		// 0 is the reporting node itself - ignore
		bn = BDBnode_createNode(&m->data[2], m->data[1]);
		BDBnode_insertNode(n, bn);
	}
}

static void BDBnode_nodeNA (struct bidibnode *n, bidibmsg_t *m)
{
	struct bidibnode *bn;

	if ((bn = BDBnode_lookupChild(n, m->data[0])) != NULL) {
		BDBnode_dropNode(bn);
	}
}

static void BDBnode_nodeLost (struct bidibnode *n, bidibmsg_t *m)
{
	struct bidibnode *bn;

	if ((bn = BDBnode_lookupChild(n, m->data[1])) != NULL) {
		BDBnode_dropNode(bn);
	}
}

static const struct msgdecoder sniffing[] = {
	{ MSG_SYS_IDENTIFY_STATE,	BDBnode_identifyState },
	{ MSG_FEATURE_COUNT,		BDBnode_featureCount },
	{ MSG_FEATURE,				BDBnode_feature },
	{ MSG_STRING,				BDBnode_string },
	{ MSG_NODETAB_COUNT,		BDBnode_ntabCount },
	{ MSG_NODETAB,				BDBnode_nodeTab },
	{ MSG_NODE_NEW,				BDBnode_nodeTab },			// handled the same as MSG_NODETAB
	{ MSG_NODE_NA,				BDBnode_nodeNA },
	{ MSG_NODE_LOST,			BDBnode_nodeLost },
	{ MSG_ACCESSORY_STATE,		BDBctrl_accessoryState },	// from bidibctrl.c
	{ MSG_BM_DCCA,				BDBctrl_dcca },				// from bidibctrl.c
	{ MSG_BM_OCC,				BDBctrl_bmOCC },			// from bidibctrl.c
	{ MSG_BM_FREE,				BDBctrl_bmFREE },			// from bidibctrl.c
	{ MSG_BM_MULTIPLE,			BDBctrl_bmMULTIPLE },		// from bidibctrl.c
	{ 0, NULL }
};
#endif

/**
 * Interpret upstream messages.
 * This is just "sniffing" if we are controlled by a TCP client.
 *
 * \param m	a pointer tio the message to interpret
 */
void BDBsrv_upstream (bidibmsg_t *m)
{
	struct bidibnode *n /*, *bn*/;
	const struct msgdecoder *u;
//	int cnt, len;

	if ((n = BDBnode_lookupNode(m->adrstack)) != NULL) {
#if 1
		u = sniffing;
		while (u->msg) {
			if (u->msg == m->msg) {
				if (u->handler) u->handler (n, m);
				break;
			}
			u++;
		}
#else
		switch (m->msg) {		// TODO: change to table driven decoding ...
			case MSG_SYS_IDENTIFY_STATE:
				if (m->data[0]) n->flags |= NODEFLG_IDENTIFY;
				else n->flags &= ~NODEFLG_IDENTIFY;
				BDBnode_nodeEvent();
				break;
			case MSG_FEATURE_COUNT:
				cnt = m->data[0];
				if (n->features) free(n->features);
				n->features = NULL;
				n->featureidx = 0;
				if (cnt > 0) {
					if ((n->features = calloc (cnt, sizeof(*n->features))) != NULL) {
						n->featurecount = cnt;
					} else {
						n->featurecount = 0;
					}
				}
				break;
			case MSG_FEATURE:
				if (n->features && m->datalen >= 2) {
					if (n->featureidx < n->featurecount) {
						n->features[n->featureidx].feature = m->data[0];
						n->features[n->featureidx].value = m->data[1];
						n->featureidx++;
					}
					if (n->featureidx >= n->featurecount) {
						bidib_sortFeature(n);
					}
				}
				break;
			case MSG_STRING:
				if (m->data[0] == 0) {		// Namespace: normal Strings (Product adn Username)
					switch (m->data[1]) {
						case 0:		// Productname
							len = m->data[2];
							if (len > MAX_PRODUCT_STRING) len = MAX_PRODUCT_STRING;
							strncpy (n->product, (char *) &m->data[3], len);
							n->product[len] = 0;
							break;
						case 1:		// Username
							len = m->data[2];
							if (len > MAX_USER_STRING) len = MAX_USER_STRING;
							strncpy (n->user, (char *) &m->data[3], len);
							n->user[len] = 0;
							break;
					}
				}
				break;
			case MSG_NODETAB_COUNT:
				BDBnode_freeNodeList(n->children);
				n->children = NULL;
				BDBnode_nodeEvent();
				break;
			case MSG_NODETAB:
			case MSG_NODE_NEW:	// behaves exactly the same as MSG_NODETAB
				if (m->data[1] == 0) break;	// this is the node itself - ignore
				bn = BDBnode_createNode(&m->data[2], m->data[1]);
				BDBnode_insertNode(n, bn);
				break;
			case MSG_NODE_NA:
				if ((bn = BDBnode_lookupChild(n, m->data[0])) != NULL) {
					BDBnode_dropNode(bn);
				}
				break;
			case MSG_NODE_LOST:
				if ((bn = BDBnode_lookupChild(n, m->data[1])) != NULL) {
					BDBnode_dropNode(bn);
				}
				break;
			case MSG_ACCESSORY_STATE:
				BDBctrl_accessoryState(n, m);
				break;
		}
#endif
	}
}

void BDBsrv_readControls (bidibmsg_t *msgs)
{
	ldataT *l;
	bidibmsg_t *m;
	int adr, speed;
	uint32_t newfuncs, mask;

	m = msgs;

	while (m) {
		switch (m->msg) {
			case MSG_CS_DRIVE_MANUAL:
				if (m->datalen >= 9) {
					adr = (m->data[1] << 8) | m->data[0];
					if ((l = loco_call(adr, true)) != NULL) {
						speed = bidib_msg2speed(m->data[4], l->loco->fmt);
						if (m->data[3] & BIDIB_CS_DRIVE_SPEED_BIT) loco_setSpeed(adr, speed);
						newfuncs = ((m->data[5] & 0x0F) << 1) | ((m->data[5] >> 4) & FUNC_LIGHT);
						newfuncs |= (m->data[6] << 5) | (m->data[7] << 13) | (m->data[8] << 21);
						mask = 0;
						if (m->data[3] & BIDIB_CS_DRIVE_F0F4_BIT) mask |= FUNC_F0_F4;
						if (m->data[3] & BIDIB_CS_DRIVE_F5F8_BIT) mask |= FUNC_F5_F8;
						if (m->data[3] & BIDIB_CS_DRIVE_F9F12_BIT) mask |= FUNC_F9_F12;
						if (m->data[3] & BIDIB_CS_DRIVE_F13F20_BIT) mask |= FUNC_F13_F20;
						if (m->data[3] & BIDIB_CS_DRIVE_F21F28_BIT) mask |= FUNC_F21_F28;
						if (mask) loco_setFuncMasked(adr, newfuncs, mask);
					}
				}
				break;
			case MSG_CS_ACCESSORY_MANUAL:
				break;
		}
		m = m->next;
	}
}

static bool BDBsrv_eventhandler (eventT *e, void *priv)
{
	bidibmsg_t *m;
	uint8_t data[8];

	(void) priv;
	if (bidib_opmode() == BIDIB_CONTROLLER) return true;
//	if (e->tid == netBDB_getTask()) return true;			// this event was triggered thru an action from netBDB service

	switch (e->ev) {
		case EVENT_SYS_STATUS:
			switch (e->param) {
				case SYSEVENT_STOP:
				case SYSEVENT_SIGON:
//					data[0] = BIDIB_BST_STATE_OFF_HERE;
					data[0] = BIDIB_BST_STATE_OFF;
					break;
				case SYSEVENT_SHORT:
					data[0] = BIDIB_BST_STATE_OFF_SHORT;
					break;
				case SYSEVENT_HALT:
				case SYSEVENT_GO:
//					data[0] = BIDIB_BST_STATE_ON_HERE;
					data[0] = BIDIB_BST_STATE_ON;
					break;
				case SYSEVENT_OVERTEMP:
					data[0] = BIDIB_BST_STATE_OFF_HOT;
					break;
			}
			if ((m = bidib_genMessage(LOCAL_NODE(), MSG_BOOST_STAT, 1, data)) != NULL) {
				netBDB_postMessages(m);
			}
			break;
		default:
			log_error("%s(): unhandled event %d\n", __func__, e->ev);
			break;
	}
	return true;
}

static bool _BDBsrv_fbHandler (struct bidibnode *n, int module, uint16_t status)
{
	struct bidibnode *child;
//	struct feedback_map *fm;
	struct virtual_feedback *vfb;
	bidibmsg_t *m;
	uint8_t data[4];

	log_msg (LOG_INFO, "%s() module %d status 0x%04x\n", __func__, module, status & 0xFFFF);
	if (!n) n = BDBnode_getRoot();
	child = n->children;
	while (child) {
		if ((child->uid[0] & BIDIB_CLASS_OCCUPANCY) && child->private) {
			vfb = child->private;
			if (vfb->base == module * 16) {
				data[0] = 0;
				data[1] = 16;
				data[2] = fb_msb2lsb8(status >> 8);
				data[3] = fb_msb2lsb8(status & 0xFF);
				if ((m = bidib_genMessage(child, MSG_BM_MULTIPLE, 4, data)) != NULL) {
					netBDB_postMessages(m);
				}
				return true;
			}
		}
		if (child->children && _BDBsrv_fbHandler(child, module, status)) break;
		child = child->next;
	}
	return false;
}

static bool BDBsrv_fbHandler (eventT *e, void *priv)
{
	fbeventT *fbevt;

	(void) priv;
	if (bidib_opmode() == BIDIB_CONTROLLER) return true;
	fbevt = e->src;

	_BDBsrv_fbHandler(NULL, fbevt->module, fbevt->status);
	return true;
}

static bool BDBsrv_replyhandler (struct decoder_reply *msg, flexval priv)
{
	static int last_adr;
	static uint32_t last_cv;
	static TickType_t last_time;

	bidibmsg_t *m;
	uint8_t data[16];
	int temp, speed;

	(void) priv;
	if (bidib_opmode() == BIDIB_CONTROLLER) return true;

	switch (msg->mt) {
		case DECODERMSG_POM:
			if (msg->adr == last_adr && msg->cva.cv == last_cv && (xTaskGetTickCount() - last_time) < 100) break;
			log_msg(LOG_INFO, "%s() ADR %d CV%lu 0x%02x\n", __func__, msg->adr, msg->cva.cv, msg->data[0]);
			last_adr = msg->adr;
			last_cv = msg->cva.cv;
			last_time = xTaskGetTickCount();
			data[0] = (msg->adr >> 0) & 0xFF;
			data[1] = (msg->adr >> 8) & 0x3F;
			data[2] = (msg->cva.cv >> 0) & 0xFF;
			data[3] = (msg->cva.cv >> 8) & 0x03;
			data[4] = msg->data[0];
			if ((m = bidib_genMessage(LOCAL_NODE(), MSG_BM_CV, 5, data)) != NULL) {
				netBDB_postMessages(m);
			}
			break;
		case DECODERMSG_EXT:
			log_msg(LOG_INFO, "%s() ADR %d EXT\n", __func__, msg->adr);
			break;
		case DECODERMSG_STAT1:
			log_msg(LOG_INFO, "%s() ADR %d STAT1\n", __func__, msg->adr);
			break;
		case DECODERMSG_ERR:
			log_msg(LOG_INFO, "%s() ADR %d ERR\n", __func__, msg->adr);
			break;
		case DECODERMSG_DYN:
			log_msg(LOG_INFO, "%s() ADR %d DYN DV %d = %d\n", __func__, msg->adr, msg->data[1], msg->data[0]);
			data[0] = 0;	// MNUM - number of detector (???? what is this?)
			data[1] = (msg->adr >> 0) & 0xFF;
			data[2] = (msg->adr >> 8) & 0x3F;	// TODO: encode decoder type in upper two bits
			switch (msg->data[1]) {				// check which DV is reported
				case 0:		// real speed part 1	// only sent if speed is <= 255km/h
				case 1:		// real speed part 2
					speed = msg->data[0];
					if (msg->data[1] == 1) speed += 256;	// speed beyond the range of speed part 1
					data[0] = (msg->adr >> 0) & 0xFF;
					data[1] = (msg->adr >> 8) & 0x3F;
					data[2] = speed & 0xFF;
					data[3] = (speed >> 8) & 0xFF;
					if ((m = bidib_genMessage(LOCAL_NODE(), MSG_BM_SPEED, 4, data)) != NULL) {
						netBDB_postMessages(m);
					}
					break;
				case 7:		// receiving stats (error rate in percent)
					data[3] = 1;	// DYN_NUM in BiDiB system
					data[4] = msg->data[0];
					if ((m = bidib_genMessage(LOCAL_NODE(), MSG_BM_DYN_STATE, 5, data)) != NULL) {
						netBDB_postMessages(m);
					}
					break;
				case 8:		// fill level 1 in percent
				case 9:		// fill level 2 in percent
				case 10:	// fill level 3 in percent
					data[3] = msg->data[1] - 5;		// DV 8 .. 10 map to DYN_NUM 3 .. 5 (other fill level DVs are not supported yet)
					data[4] = msg->data[0];
					if ((m = bidib_genMessage(LOCAL_NODE(), MSG_BM_DYN_STATE, 5, data)) != NULL) {
						netBDB_postMessages(m);
					}
					break;
				case 26:	// temperature from -50°C to +205°C
					data[3] = 2;		// DYN_NUM 2 reports tempertures between -30°C and +127°C as signed byte
					temp = msg->data[0] - 50;
					if (temp < -30) temp = -30;
					else if (temp > 127) temp = 127;
					data[4] = (uint8_t) (temp & 0xFF);
					if ((m = bidib_genMessage(LOCAL_NODE(), MSG_BM_DYN_STATE, 5, data)) != NULL) {
						netBDB_postMessages(m);
					}
					break;
			}
			break;
		case DECODERMSG_XPOM00:
		case DECODERMSG_XPOM01:
		case DECODERMSG_XPOM10:
		case DECODERMSG_XPOM11:
			log_msg(LOG_INFO, "%s() ADR %d XPOM%c%c\n", __func__, msg->adr,
				(msg->mt == DECODERMSG_XPOM00 || msg->mt == DECODERMSG_XPOM01) ? '0' : '1',
				(msg->mt == DECODERMSG_XPOM00 || msg->mt == DECODERMSG_XPOM10) ? '0' : '1');
			break;
		case DECODERMSG_DECSTATE:
			log_msg(LOG_INFO, "%s() ADR %d DECSTATE\n", __func__, msg->adr);
			break;
		case DECODERMSG_TIME:
			log_msg(LOG_INFO, "%s() ADR %d TIME\n", __func__, msg->adr);
			break;
		case DECODERMSG_UNIQUE:
			log_msg(LOG_INFO, "%s() ADR %d UNIQUE\n", __func__, msg->adr);
			break;
		default:
			break;
	}
	return true;
}

static uint8_t BDBsrv_diagnosticTimerChange (struct bidibnode *n, struct nodefeature *nf, uint8_t val)
{
	(void) n;

	if (!nf) return 0;
	if (!diagtimer) return nf->value;		// cannot change value as long as the timer is not constructed
	if (val == 0) {		// switch off reporting
		xTimerStop(diagtimer, 50);
	} else {			// set new timer value
		if (val < 10) val = 10;
		xTimerChangePeriod(diagtimer, pdMS_TO_TICKS(10 * val), 50);
	}

	return val;
}

static void BDBsrv_diagnosticTimer (TimerHandle_t tim)
{
	(void) tim;
	BDBsrv_boosterDiag(LOCAL_NODE());
}

static void BDBsrv_initFeatures (struct bidibnode *n)
{
	struct fmtconfig *fmtcfg;
	struct nodefeature *ft;

	if ((ft = bidib_readFeature(n, FEATURE_BST_VOLT)) != NULL) ft->value = ts_getVoltage() / 10;
	if ((ft = bidib_readFeature(n, FEATURE_BST_AMPERE)) != NULL) ft->value = bidib_current2code(ts_getCurrentMilliAmpere());
	if ((ft = bidib_readFeature(n, FEATURE_GEN_EXT_AVAILABLE)) != NULL) {
		fmtcfg = cnf_getFMTconfig();
		ft->value = EXT_FIXED_FEATURES;
		if (fmtcfg->sigflags & SIGFLAG_DCCA) ft->value |= EXT_DCCA;
		if (fmtcfg->sigflags & SIGFLAG_M3ENABLED) ft->value |= EXT_M4;
	}
}

static const struct msgdecoder downstream[] = {
	{ MSG_SYS_GET_MAGIC,		BDBnf_sendSysMagic },
	{ MSG_SYS_GET_P_VERSION,	BDBnf_sendPVersion },
	{ MSG_SYS_ENABLE,			BDBsrv_sysEnable },
	{ MSG_SYS_DISABLE,			BDBsrv_sysDisable },
	{ MSG_SYS_GET_UNIQUE_ID,	BDBnf_sendUniqueID },
	{ MSG_SYS_GET_SW_VERSION,	BDBnf_sendVersionInfo },
	{ MSG_SYS_PING,				BDBnf_sendPong },
	{ MSG_SYS_IDENTIFY,			BDBsrv_identify },
	{ MSG_SYS_RESET,			BDBsrv_resetSystem },
	{ MSG_NODETAB_GETALL,		BDBnf_reportNodetab },
	{ MSG_NODETAB_GETNEXT,		BDBnf_nextNodetab },
	{ MSG_NODE_CHANGED_ACK,		BDBnode_changeACK },
	{ MSG_SYS_GET_ERROR,		BDBnf_getError },
	{ MSG_FEATURE_GETALL,		BDBnf_reportFeatures },
	{ MSG_FEATURE_GETNEXT,		BDBnf_getNextFeature },
	{ MSG_FEATURE_GET,			BDBnf_getFeature },
	{ MSG_FEATURE_SET,			BDBnf_setFeature },
	{ MSG_STRING_GET,			BDBnf_getString },
	{ MSG_STRING_SET,			BDBnf_setString },
	{ MSG_BOOST_OFF,			BDBsrv_boosterOff },
	{ MSG_BOOST_ON,				BDBsrv_boosterOn },
	{ MSG_BOOST_QUERY,			BDBsrv_boosterQuery },
	{ MSG_CS_SET_STATE,			BDBsrv_setState },
	{ MSG_CS_DRIVE,				BDBsrv_loco },
	{ MSG_CS_BIN_STATE,			BDBsrv_binstate },
	{ MSG_CS_ACCESSORY,			BDBsrv_accessory },
	{ MSG_CS_POM,				BDBsrv_pom },
	{ MSG_CS_QUERY,				BDBsrv_query },
	{ MSG_CS_PROG,				BDBsrv_prog },
	{ MSG_LOCAL_PING,			BDBnf_sendPong },
	{ 0, NULL }
};

/**
 * Called when switching to external control to make sure, some changeable features
 * get setup to current values.
 */
void BDBsrv_updateFeatures (void)
{
	BDBsrv_initFeatures(LOCAL_NODE());
}

struct bidibnode *BDBsrv_genLocalNode (void)
{
	struct bidibnode *n;

	if ((n = BDBnode_createNode(myUID, 0)) == NULL) return NULL;
	if ((n->features = malloc(sizeof (features))) != NULL) {
		memcpy (n->features, features, sizeof(features));
		n->featurecount = DIM(features);
		BDBsrv_initFeatures(n);
	}
	strncpy (n->product, BIDIB_PRODSTR_TAMS, MAX_PRODUCT_STRING);
	n->product[MAX_PRODUCT_STRING] = 0;
	strncpy (n->user, cnf_getconfig()->bidib.user, MAX_USER_STRING);
	n->user[MAX_USER_STRING] = 0;
	n->pversion = BIDIB_VERSION;
	n->downstream = downstream;
	n->flags |= NODEFLG_VIRTUAL;

	return n;
}

void BDBsrv_start (void)
{
	struct nodefeature *ft;

	// first, create the timer with a default setting (time zero is not allowed)
	diagtimer = xTimerCreate("BoosterDiag", pdMS_TO_TICKS(2000), pdTRUE, NULL, BDBsrv_diagnosticTimer);
	if ((ft = bidib_readFeature(BDBnode_getRoot(), FEATURE_BST_CURMEAS_INTERVAL)) != NULL) {
		if (diagtimer && (ft->value > 0)) {
			xTimerChangePeriod(diagtimer, pdMS_TO_TICKS(ft->value * 10), 20);
		}
	} else {
		if (diagtimer) xTimerStart(diagtimer, 20);		// if feature is not present, start timer with the default value
	}
	event_register(EVENT_SYS_STATUS, BDBsrv_eventhandler, NULL, 0);
	event_register(EVENT_FBNEW, BDBsrv_fbHandler, NULL, 0);
	reply_register (DECODER_ANY, 0, DECODERMSG_ANY, BDBsrv_replyhandler, fvNULL, 0);
}
