/*
 * virtualnode.c
 *
 *  Created on: 12.07.2021
 *      Author: andi
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
#include "bidib.h"

#if 0	// a list of defined feature to copy from ...
const static struct nodefeature allfeatures[] = {
	//-- occupancy
	{ FEATURE_BM_SIZE,							0, NULL },		// number of occupancy detectors
	{ FEATURE_BM_ON,							1, NULL },		// occupancy detection on/off
	{ FEATURE_BM_SECACK_AVAILABLE,				0, NULL },		// secure ack available
	{ FEATURE_BM_SECACK_ON,						0, NULL },		// secure ack on/off
	{ FEATURE_BM_CURMEAS_AVAILABLE,				0, NULL },		// occupancy detectors with current measurement results
	{ FEATURE_BM_CURMEAS_INTERVAL,				0, NULL },		// interval for current measurements
	{ FEATURE_BM_DC_MEAS_AVAILABLE,				0, NULL },		// (dc) measurements available, even if track voltage is off
	{ FEATURE_BM_DC_MEAS_ON,					0, NULL },		// dc measurement enabled
	//-- bidi detection
	{ FEATURE_BM_ADDR_DETECT_AVAILABLE,			0, NULL },		// detector ic capable to detect loco address
	{ FEATURE_BM_ADDR_DETECT_ON,				0, NULL },		// address detection enabled
	{ FEATURE_BM_ADDR_AND_DIR,					0, NULL },		// addresses contain direction
	{ FEATURE_BM_ISTSPEED_AVAILABLE,			0, NULL },		// speed messages available
	{ FEATURE_BM_ISTSPEED_INTERVAL,				0, NULL },		// speed update interval
	{ FEATURE_BM_CV_AVAILABLE,					0, NULL },		// CV readback available
	{ FEATURE_BM_CV_ON,							0, NULL },		// CV readback enabled
	//-- booster
	{ FEATURE_BST_VOLT_ADJUSTABLE,				1, NULL },		// booster output voltage is adjustable
	{ FEATURE_BST_VOLT,							0, NULL },		// booster output voltage setting (unit: V)
	{ FEATURE_BST_CUTOUT_AVAILABLE,				1, NULL },		// booster can do cutout for railcom
	{ FEATURE_BST_CUTOUT_ON,					1, NULL },		// cutout is enabled
	{ FEATURE_BST_TURNOFF_TIME,					0, NULL },		// time in ms until booster turns off in case of a short (unit 2ms)
	{ FEATURE_BST_INRUSH_TURNOFF_TIME,			0, NULL },		// time in ms until booster turns off in case of a short after the first power up (unit 2ms)
	{ FEATURE_BST_AMPERE_ADJUSTABLE,			1, NULL },		// booster output current is adjustable
	{ FEATURE_BST_AMPERE,						196, NULL },	// booster output current value (special coding, 6.400mA)
	{ FEATURE_BST_CURMEAS_INTERVAL,				200, NULL },	// current update interval
	{ FEATURE_BST_INHIBIT_AUTOSTART,			0, NULL },		// 1: Booster does no automatic BOOST_ON when DCC at input wakes up.
	{ FEATURE_BST_INHIBIT_LOCAL_ONOFF,			0, NULL },		// 1: Booster announces local STOP/GO key stroke only, no local action

	//-- bidi detection
	{ FEATURE_BM_DYN_STATE_INTERVAL,			0, NULL },		// transmit interval of MSG_BM_DYN_STATE (unit 100ms)
	{ FEATURE_BM_RCPLUS_AVAILABLE,				0, NULL },		// 1: RailcomPlus messages available
	//-- occupancy
	{ FEATURE_BM_TIMESTAMP_ON,					0, NULL },		// 1: OCC will be sent with timestamp
	//-- bidi detection
	{ FEATURE_BM_POSITION_ON,					0, NULL },		// position messages enabled
	{ FEATURE_BM_POSITION_SECACK,				0, NULL },		// secure position ack interval (unit: 10ms), 0: none

	//-- accessory
	{ FEATURE_ACCESSORY_COUNT,					0, NULL },		// number of objects
	{ FEATURE_ACCESSORY_SURVEILLED,				0, NULL },		// 1: announce if operated outside bidib
	{ FEATURE_ACCESSORY_MACROMAPPED,			0, NULL },		// 1..n: no of accessory aspects are mapped to macros

	//-- control
	{ FEATURE_CTRL_INPUT_COUNT,					0, NULL },		// number of inputs for keys
	{ FEATURE_CTRL_INPUT_NOTIFY,				0, NULL },		// 1: report a keystroke to host
	{ FEATURE_CTRL_SWITCH_COUNT,				0, NULL },		// number of switch ports (direct controlled)
	{ FEATURE_CTRL_LIGHT_COUNT,					0, NULL },		// number of light ports (direct controlled)
	{ FEATURE_CTRL_SERVO_COUNT,					0, NULL },		// number of servo ports (direct controlled)
	{ FEATURE_CTRL_SOUND_COUNT,					0, NULL },		// number of sound ports (direct controlled)
	{ FEATURE_CTRL_MOTOR_COUNT,					0, NULL },		// number of motor ports (direct controlled)
	{ FEATURE_CTRL_ANALOGOUT_COUNT,				0, NULL },		// number of analog ports (direct controlled)
	{ FEATURE_CTRL_STRETCH_DIMM,				0, NULL },		// additional time stretch for dimming (for light ports)
	{ FEATURE_CTRL_BACKLIGHT_COUNT,				0, NULL },		// number of backlight ports (intensity direct controlled)
	{ FEATURE_CTRL_MAC_LEVEL,					0, NULL },		// supported macro level
	{ FEATURE_CTRL_MAC_SAVE,					0, NULL },		// number of permanent storage places for macros
	{ FEATURE_CTRL_MAC_COUNT,					0, NULL },		// number of macros
	{ FEATURE_CTRL_MAC_SIZE,					0, NULL },		// length of each macro (entries)
	{ FEATURE_CTRL_MAC_START_MAN,				0, NULL },		// (local) manual control of macros enabled
	{ FEATURE_CTRL_MAC_START_DCC,				0, NULL },		// (local) dcc control of macros enabled
	{ FEATURE_CTRL_PORT_QUERY_AVAILABLE,		0, NULL },		// 1: node will answer to output queries via MSG_LC_PORT_QUERY
	{ FEATURE_SWITCH_CONFIG_AVAILABLE,			0, NULL },		// (deprecated, version >= 0.6 uses availability of PCFG_IO_CTRL) 1: node has possibility to configure switch ports
	{ FEATURE_CTRL_PORT_FLAT_MODEL,				0, NULL },		// node uses flat port model, "low" number of addressable ports
	{ FEATURE_CTRL_PORT_FLAT_MODEL_EXTENDED,	0, NULL },		// node uses flat port model, "high" number of addressable ports

	//-- dcc gen
	{ FEATURE_GEN_SPYMODE,						0, NULL },		// 1: watch bidib handsets
	{ FEATURE_GEN_WATCHDOG,						0, NULL },		// 0: no watchdog, 1: permanent update of MSG_CS_SET_STATE required, unit 100ms
	{ FEATURE_GEN_DRIVE_ACK,					0, NULL },		// not used
	{ FEATURE_GEN_SWITCH_ACK,					0, NULL },		// not used
	{ FEATURE_GEN_LOK_DB_SIZE,					0, NULL },		//
	{ FEATURE_GEN_LOK_DB_STRING,				0, NULL },		//
	{ FEATURE_GEN_POM_REPEAT,					0, NULL },		// supported service modes
	{ FEATURE_GEN_DRIVE_BUS,					1, NULL },		// 1: this node drive the dcc bus.
	{ FEATURE_GEN_LOK_LOST_DETECT,				0, NULL },		// 1: command station annouces lost loco
	{ FEATURE_GEN_NOTIFY_DRIVE_MANUAL,			1, NULL },		// 1: dcc gen reports manual operation
	{ FEATURE_GEN_START_STATE,					0, NULL },		// 1: power up state, 0=off, 1=on
	{ FEATURE_GEN_RCPLUS_AVAILABLE,				0, NULL },		// (deprecated) 1: supports rcplus messages
	{ FEATURE_GEN_EXT_AVAILABLE,			 0x1A, NULL },		// bitfield ext. support: 0:RailCom+, 1:M4, 2:DCCA, 3:DCC-SDF, 4:MM, other: reserved
	{ FEATURE_CELL_NUMBER,						0, NULL },		// (logical) reference mark of generator (0=single system, 1..n:'area mark')
	{ FEATURE_RF_CHANNEL,						0, NULL },		// used rf channel, 0..83 channel assignment in 2.4 GHz band

	{ FEATURE_STRING_DEBUG,						0, NULL },		// use namespace 1 for debug strings, 0:n.a./silent (default); 1=mode 1
	{ FEATURE_STRING_SIZE,					   24, NULL },		// length of user strings, 0:n.a (default); allowed 8..24
	{ FEATURE_RELEVANT_PID_BITS,				8, NULL },		// how many bits of 'vendor32' are relevant for PID (default 16, LSB aligned)
	{ FEATURE_FW_UPDATE_MODE,					0, NULL },		// 0: no fw-update, 1: intel hex (max. 10 byte / record)
	{ FEATURE_EXTENSION,						0, NULL },		// 1: reserved for future expansion
};
#endif		/* example */

static const struct nodefeature bridge[] = {		// internal HUBs
	{ FEATURE_STRING_SIZE,					   24, NULL },		// length of user strings, 0:n.a (default); allowed 8..24
	{ FEATURE_RELEVANT_PID_BITS,			    8, NULL },		// how many bits of 'vendor32' are relevant for PID (default 16, LSB aligned)
	{ FEATURE_FW_UPDATE_MODE,					0, NULL },		// 0: no fw-update, 1: intel hex (max. 10 byte / record)
};

static const struct nodefeature feedback[] = {		// s88, lnet and mcan feedback modules
	{ FEATURE_BM_SIZE,						   16, NULL },		// number of occupancy detectors
	{ FEATURE_BM_ON,							1, NULL },		// occupancy detection on/off
	{ FEATURE_BM_SECACK_AVAILABLE,				0, NULL },		// secure ack available
	{ FEATURE_BM_SECACK_ON,						0, NULL },		// secure ack on/off
	{ FEATURE_BM_TIMESTAMP_ON,					0, NULL },		// 1: OCC will be sent with timestamp
	{ FEATURE_STRING_SIZE,					   24, NULL },		// length of user strings, 0:n.a (default); allowed 8..24
	{ FEATURE_RELEVANT_PID_BITS,			    8, NULL },		// how many bits of 'vendor32' are relevant for PID (default 16, LSB aligned)
	{ FEATURE_FW_UPDATE_MODE,					0, NULL },		// 0: no fw-update, 1: intel hex (max. 10 byte / record)
};

/*
 * Forward declaration of static functions
 */
static void BDBvn_sysDisable (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_sysEnable (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_identify (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_reportNodetabSingle (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_nextNodetabSingle (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_getRange (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_mirrorOCC (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_mirrorFREE (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_mirrorMultiple (struct bidibnode *n, bidibmsg_t *msg);
static void BDBvn_getConfidence (struct bidibnode *n, bidibmsg_t *msg);

static const struct msgdecoder bridge_down[] = {
	{ MSG_SYS_GET_MAGIC,		BDBnf_sendSysMagic },
	{ MSG_SYS_GET_P_VERSION,	BDBnf_sendPVersion },
	{ MSG_SYS_ENABLE,			BDBvn_sysEnable },
	{ MSG_SYS_DISABLE,			BDBvn_sysDisable },
	{ MSG_SYS_GET_UNIQUE_ID,	BDBnf_sendUniqueID },
	{ MSG_SYS_GET_SW_VERSION,	BDBnf_sendVersionInfo },
	{ MSG_SYS_PING,				BDBnf_sendPong },
	{ MSG_SYS_IDENTIFY,			BDBvn_identify },
//	{ MSG_SYS_RESET,			BDBsrv_resetSystem },
	{ MSG_NODETAB_GETALL,		BDBnf_reportNodetab },
	{ MSG_NODETAB_GETNEXT,		BDBnf_nextNodetab },
	{ MSG_NODE_CHANGED_ACK,		BDBnode_changeACK },
	{ MSG_SYS_GET_ERROR,		BDBnf_getError },
	{ MSG_FEATURE_GETALL,		BDBnf_reportFeatures },
	{ MSG_FEATURE_GETNEXT,		BDBnf_getNextFeature },
	{ MSG_FEATURE_GET,			BDBnf_getFeature },
	{ MSG_FEATURE_SET,			BDBnf_setFeature },
	{ MSG_SYS_CLOCK,			BDBnf_sysClock },
	{ MSG_STRING_GET,			BDBnf_getString },
	{ MSG_STRING_SET,			BDBnf_setString },
	{ 0, NULL }
};

static const struct msgdecoder fb_down[] = {
	{ MSG_SYS_GET_MAGIC,		BDBnf_sendSysMagic },
	{ MSG_SYS_GET_P_VERSION,	BDBnf_sendPVersion },
	{ MSG_SYS_ENABLE,			BDBvn_sysEnable },
	{ MSG_SYS_DISABLE,			BDBvn_sysDisable },
	{ MSG_SYS_GET_UNIQUE_ID,	BDBnf_sendUniqueID },
	{ MSG_SYS_GET_SW_VERSION,	BDBnf_sendVersionInfo },
	{ MSG_SYS_PING,				BDBnf_sendPong },
	{ MSG_SYS_IDENTIFY,			BDBvn_identify },
	{ MSG_NODETAB_GETALL,		BDBvn_reportNodetabSingle },
	{ MSG_NODETAB_GETNEXT,		BDBvn_nextNodetabSingle },
	{ MSG_SYS_GET_ERROR,		BDBnf_getError },
	{ MSG_FEATURE_GETALL,		BDBnf_reportFeatures },
	{ MSG_FEATURE_GETNEXT,		BDBnf_getNextFeature },
	{ MSG_FEATURE_GET,			BDBnf_getFeature },
	{ MSG_FEATURE_SET,			BDBnf_setFeature },
	{ MSG_SYS_CLOCK,			BDBnf_sysClock },
	{ MSG_STRING_GET,			BDBnf_getString },
	{ MSG_STRING_SET,			BDBnf_setString },
	{ MSG_BM_GET_RANGE,			BDBvn_getRange },
	{ MSG_BM_MIRROR_OCC,		BDBvn_mirrorOCC },
	{ MSG_BM_MIRROR_FREE,		BDBvn_mirrorFREE },
	{ MSG_BM_MIRROR_MULTIPLE,	BDBvn_mirrorMultiple },
	{ MSG_BM_GET_CONFIDENCE,	BDBvn_getConfidence },
	{ 0, NULL }
};

enum fbtype {
	FBTYPE_S88,								///< create a virtual s88 node
	FBTYPE_MCAN,							///< create a virtual mCAN node
	FBTYPE_LNET,							///< create a virtual LocoNet node
};

struct vfeedback {
	struct vfeedback	*next;				///< singly linked list
	int					 module;			///< the module number (0-based) that this module is responsible for
	struct bidibnode	*node;				///< the node structure
	uint16_t			 oldstat;			///< the last status successfully sent to host
};

static struct vfeedback *virtual_fb;

/*
 * ==================================================================================================
 * === Handler functions for BiDiB messages =========================================================
 * ==================================================================================================
 */
static void BDBvn_sysDisable (struct bidibnode *n, bidibmsg_t *msg)
{
	(void) msg;

	if (n) n->flags |= NODEFLG_SYSDISABLE;
}

static void BDBvn_sysEnable (struct bidibnode *n, bidibmsg_t *msg)
{
	(void) msg;

	if (n) n->flags &= ~NODEFLG_SYSDISABLE;
}

static void BDBvn_identify (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	uint8_t data[2];

	if (!n || !msg) return;
	if (msg->datalen >= 1) {
		if (msg->data[0]) n->flags |= NODEFLG_IDENTIFY;
		else n->flags &= ~NODEFLG_IDENTIFY;
		data[0] = (n->flags & NODEFLG_IDENTIFY) ? 1 : 0;
		if ((m = bidib_genMessage(n, MSG_SYS_IDENTIFY_STATE, 1, data)) != NULL) BDBnode_uplink(NULL, m);
	}
}

/**
 * For devices that don't habe a real node tab.
 * We always report a single device.
 *
 * \param n		the node that schould be queried
 * \param msg	the message containing the query
 */
static void BDBvn_reportNodetabSingle (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	uint8_t cnt;

	(void) msg;

	if (!n) return;
	cnt = 1;
	if ((m = bidib_genMessage(n, MSG_NODETAB_COUNT, 1, &cnt)) != NULL) BDBnode_uplink(NULL, m);
	n->ntab_version = 1;	// just as a flag, that reporting of the single device may start
}

/**
 * For devices that don't habe a real node tab.
 * We always report ntab version #1 with our own UID as node address #0.
 *
 * \param n		the node that schould be queried
 * \param msg	the message containing the query
 */
static void BDBvn_nextNodetabSingle (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	uint8_t data[16];

	(void) msg;

	if (!n) return;
	if (n->ntab_version) {		// check if this was set in BDBvn_reportNodetabSingle()
		data[0] = 1;			// nodetab version
		data[1] = 0;			// node address
		memcpy (&data[2], n->uid, BIDIB_UID_LEN);
		m = bidib_genMessage(n, MSG_NODETAB, 2 + BIDIB_UID_LEN, data);
		n->ntab_version = 0;	// signal end of node table
	} else {
		data[0] = 255;
		m = bidib_genMessage(n, MSG_NODE_NA, 1, data);
	}
	if (m) BDBnode_uplink(NULL, m);
}

static void BDBvn_getRange (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	struct virtual_feedback *vfb;
	uint8_t start, end, errcode, *data, *p;
	int idx;

	if (!n || !msg || msg->datalen < 2) return;
	if ((vfb = n->private) == NULL) {	// should never be seen, because we always should have our private structure!
		errcode = 1;	// should never be seen, because we always should have our private structure!
		m = bidib_errorMessage(n, BIDIB_ERR_HW, 1, &errcode);
	} else {
		start = msg->data[0] & ~0x07;		// set start to a byte range
		end = (msg->data[1] + 7) & ~0x07;	// set end just behind the next byte range (end is exclusive)
		if (end > (vfb->base + vfb->count)) end = vfb->base + vfb->count;
		if (start >= end) {					// out-of-range error
			m = bidib_errorMessage(n, BIDIB_ERR_PARAMETER, 1, &msg->seq);
		} else {				// continue with processing
			p = data = tmpbuf((end - start) / 8 + 2);
			*p++ = start;
			*p++ = end - start;
			for (idx = (start / 8); idx < (end / 8); idx ++) {
				*p++ = vfb->status[idx];
			}
			m = bidib_genMessage(n, MSG_BM_MULTIPLE, p - data, data);
		}
	}
	if (m) BDBnode_uplink(NULL, m);
}

static void BDBvn_mirrorOCC (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	struct nodefeature *ft;
	struct virtual_feedback *vfb;
	uint8_t bit, errcode, data[3];

	if (!msg || msg->datalen < 1) return;
	if ((vfb = n->private) == NULL) {
		errcode = 1;	// should never be seen, because we always should have our private structure!
		m = bidib_errorMessage(n, BIDIB_ERR_HW, 1, &errcode);
	} else {
		bit = msg->data[0];
		if (bit >= vfb->count) {
			if ((m = bidib_errorMessage(n, BIDIB_ERR_PARAMETER, 1, &msg->seq))) BDBnode_uplink(NULL, m);
			return;
		}
		if ((ft = bidib_readFeature(n, FEATURE_BM_SECACK_ON)) != NULL && ft->value > 0) {		// check if SEACK is enabled
			if (!bs_isset(vfb->bitset, bit)) {		// status mismatch - report again
				data[0] = bit;
				if ((m = bidib_genMessage(n, MSG_BM_FREE, 1, data)) != NULL) BDBnode_uplink(NULL, m);
			}
		}
	}
}

static void BDBvn_mirrorFREE (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	struct nodefeature *ft;
	struct virtual_feedback *vfb;
	uint8_t bit, errcode, data[3];

	if (!msg || msg->datalen < 1) return;
	if ((vfb = n->private) == NULL) {
		errcode = 1;	// should never be seen, because we always should have our private structure!
		m = bidib_errorMessage(n, BIDIB_ERR_HW, 1, &errcode);
	} else {
		bit = msg->data[0];
		if (bit > 15) {
			if ((m = bidib_errorMessage(n, BIDIB_ERR_PARAMETER, 1, &msg->seq))) BDBnode_uplink(NULL, m);
			return;
		}
		if ((ft = bidib_readFeature(n, FEATURE_BM_SECACK_ON)) != NULL && ft->value > 0) {		// check if SEACK is enabled
			if (bs_isset(vfb->bitset, bit)) {		// status mismatch - report again
				data[0] = bit;
				if ((m = bidib_genMessage(n, MSG_BM_OCC, 1, data)) != NULL) BDBnode_uplink(NULL, m);
			}
		}
	}
}

static void BDBvn_mirrorMultiple (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	struct nodefeature *ft;
	struct virtual_feedback *vfb;
	uint8_t start, len, errcode, *data, *p;
	int idx;

	if (!msg || msg->datalen < 3) return;
	if ((vfb = n->private) == NULL) {
		errcode = 1;	// should never be seen, because we always should have our private structure!
		m = bidib_errorMessage(n, BIDIB_ERR_HW, 1, &errcode);
	} else {
		start = msg->data[0];
		len = msg->data[1];
		if ((start & 0x07) || (len & 0x07) || (start + len > vfb->count) || (msg->datalen < (2 + (len / 8)))) {
			if ((m = bidib_errorMessage(n, BIDIB_ERR_PARAMETER, 1, &msg->seq))) BDBnode_uplink(NULL, m);
			return;
		}

		if ((ft = bidib_readFeature(n, FEATURE_BM_SECACK_ON)) != NULL && ft->value > 0) {		// check if SEACK is enabled
			p = &msg->data[2];
			for (idx = (start / 8); idx < ((start + len) / 8); idx++) {
				if (vfb->status[idx] != *p) {
					p = data = tmpbuf(len / 8 + 2);
					*p++ = start;
					*p++ = len;
					for (idx = (start / 8); idx < ((start + len) / 8); idx ++) {
						*p++ = vfb->status[idx];
					}
					if ((m = bidib_genMessage(n, MSG_BM_MULTIPLE, p - data, data)) != NULL) BDBnode_uplink(NULL, m);
					return;
				}
			}
		}
	}
}

static void BDBvn_getConfidence (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;
	uint8_t data[3];

	(void) msg;

	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	if ((m = bidib_genMessage(n, MSG_BM_CONFIDENCE, 3, data)) != NULL) BDBnode_uplink(NULL, m);
}

/*
 * ==================================================================================================
 * === Creation and maping functions ================================================================
 * ==================================================================================================
 */

static void BDBvn_delMapping (struct bidibnode *n)
{
	struct vfeedback *vfp, **pp;

	pp = &virtual_fb;
	while (*pp && (*pp)->node != n) pp = &(*pp)->next;
	if ((vfp = *pp) != NULL) {
		*pp = vfp->next;
		free (vfp);
	}
}

static void BDBvn_addMapping (struct bidibnode *n, int module)
{
	struct vfeedback *vfp, **pp;

	BDBvn_delMapping(n);		// delete a potential previous mapping
	pp = &virtual_fb;
	while (*pp && (*pp)->module < module) pp = &(*pp)->next;
	if ((*pp)->module == module) {	// replace nodepointer - should not happen ...
		vfp = *pp;
		vfp->node = n;
	} else {						// create a new pointer entry here
		if ((vfp = malloc (sizeof(*vfp))) != NULL) {
			vfp->next = *pp;
			vfp->module = module;
			vfp->node = n;
			vfp->oldstat = 0;	// OR: get the current state from feedback.c
			*pp = vfp;
		}
	}
}

/**
 * Write the calculated UID to the array, pointed to by the uid parameter.
 * The UID is based on the 12 LSBs of the real serial number of the RB2,
 * a product ID of 8 bits (0xA0=mc2 itself, 0xA1 - 0xAF for virtual nodes),
 * the requested class and an 12-bit index count.
 *
 * \param uid		pointer to a byte array receiving the seven UID bytes
 * \param class		the requested CLASS (currently only BIDIB_CLASS_BRIDGE and BIDIB_CLASS_OCCUPANCY are expected)
 * \param prodId	the 8 bit product ID of the virtual device
 * \param idx		a 12 bit index for the created serial numbers
 */
static void BDBvn_createUID (uint8_t *uid, uint8_t class, uint8_t prodId, int idx)
{
	if (!uid) return;

	uid[0] = class;							// Class
	uid[1] = 0;								// X-Class
	uid[2] = hwinfo->manufacturer;			// manufacturer;
	uid[3] = prodId;						// product ID (8 bits)
	uid[4] = (hwinfo->serial >> 4) & 0xFF;	// upper 8 serial bits
	uid[5] = (hwinfo->serial << 4) & 0xF0;	// 4 serial LS bits
	uid[5] |= (idx >> 4) & 0x0F;			// 4 idx MS bits
	uid[6] = idx & 0xFF;					// 8 idx LSBs as serial
}

/**
 * Common function to create a virtual node of any kind.
 * With an 8 bit product ID (always the virtual MC2 id '0xA1') there are
 * 24 bits left for the individual serial number of the device. We take the
 * lower 12 bits of the MC2 serial number and stuff another 12 bits in the
 * upper bits for individual virtual nodes.
 *
 * \param parent		where to position the node in the tree
 * \param ft			the feature structure to clone for the new node
 * \param ftcount		the length of this feature structure
 * \param serial		up to 12 bits of serial number information that is written to the uppermost 12 bits of the resulting serial number.
 * \param class			the class bits for this node
 * \param prodId		the product ID of the virtual node
 * \return				an allocated node structure with cloned feature array.
 * 						This node is not yet inserted into the child list of it's parent
 * 						to complete the settings before it appears in the tree.
 */
static struct bidibnode *BDBvn_create (struct bidibnode *parent, const struct nodefeature *ft, int ftcount, int serial, uint8_t class, int prodId)
{
	struct bidibnode *n;
	uint8_t uid[BIDIB_UID_LEN];
	uint8_t adr;

	if (!parent) return NULL;
	BDBvn_createUID(uid, class, prodId, serial);
	adr = BDBnode_getFreeAddress(parent, (parent->parent == NULL) ? 64 : 1);
	if ((n = BDBnode_createNode(uid, adr)) == NULL) return NULL;

	if (ft && ftcount > 0 && (n->features = malloc (ftcount * sizeof(*ft))) != NULL) {
		memcpy (n->features, ft, ftcount * sizeof(*ft));
		n->featurecount = ftcount;
	}
	*n->product = 0;
	*n->user = 0;
	n->pversion = BIDIB_VERSION;
	n->flags |= NODEFLG_VIRTUAL;

	return n;
}

/**
 * Create a new virtual bridge device (HUB). Serial numbers are offset by
 * a fixed value (BIDIB_VIRTIF_SNR_OFFSET).
 *
 * \param parent		where to position the node in the tree
 * \param serial		up to 12 bits of serial number information, see BDBvn_create()
 * \return				an allocated node structure with cloned feature array and already inserted in the tree
 */
struct bidibnode *BDBvn_newBridge (struct bidibnode *parent, int serial)
{
	struct bidibnode *n;
	bidibmsg_t *m;
	uint8_t data[16];

	if (!parent) parent = BDBnode_getRoot();
	if ((n = BDBvn_create(parent, bridge, DIM(bridge), serial, BIDIB_CLASS_BRIDGE, BIDIB_PID_VIRT_HUB)) != NULL) {
		strncpy (n->product, BIDIB_PRODSTR_VIRT_IF, MAX_PRODUCT_STRING);
		n->product[MAX_PRODUCT_STRING] = 0;
		sprintf (n->user, "virtual HUB #%d", serial);
		n->downstream = bridge_down;
		BDBnode_insertNode(parent, n);
		data[0] = parent->ntab_version++;
		data[1] = n->localadr;
		memcpy (&data[2], n->uid, BIDIB_UID_LEN);
		m = bidib_genMessage(parent, MSG_NODE_NEW, 2 + BIDIB_UID_LEN, data);
		netBDB_postMessages(m);
	}

	return n;
}

/**
 * Create a new virtual feedback module.
 *
 * \param parent		where to position the node in the tree
 * \param serial		up to 12 bits of serial number information, see BDBvn_create()
 * \param fbt			the type / bus of this feedback module, only used for some strings (eyecandy)
 * \param prodId		the product ID of the virtual node
 * \param fbbase		the 0-based first feedback number that is represented by this node
 * \param fbcount		the number of feedback bits represented by this node - maximum is 128 bits
 * \return				an allocated node structure with cloned feature array and already inserted in the tree
 */
struct bidibnode *BDBvn_newFeedback (struct bidibnode *parent, int serial, enum fbtype fbt, int prodId, int fbbase, int fbcount)
{
	struct bidibnode *n;
	struct virtual_feedback *vfb;
	bidibmsg_t *m;
	uint8_t data[16];
	int fbarray;

	if (fbbase >= MAX_FEEDBACKS) return NULL;
	if (fbcount > 128) fbcount = 128;
	if (fbbase + fbcount > MAX_FEEDBACKS) fbcount = MAX_FEEDBACKS - fbbase;
	fbarray = (fbcount + 31) / 32;	// number of 32-bit words needed as feedback storage

	if ((n = BDBvn_create(parent, feedback, DIM(feedback), serial, BIDIB_CLASS_OCCUPANCY, prodId)) != NULL) {
		switch (fbt) {
			case FBTYPE_S88:
				strncpy (n->product, BIDIB_PRODSTR_VIRT_S88, MAX_PRODUCT_STRING);
				sprintf (n->user, "s88 #%d", serial);
				break;
			case FBTYPE_MCAN:
				strncpy (n->product, BIDIB_PRODSTR_VIRT_MC, MAX_PRODUCT_STRING);
				sprintf (n->user, "mcan #%d", serial);
				break;
			case FBTYPE_LNET:
				strncpy (n->product, BIDIB_PRODSTR_VIRT_LN, MAX_PRODUCT_STRING);
				sprintf (n->user, "LocoNet #%d", serial);
				break;
		}
		n->product[MAX_PRODUCT_STRING] = 0;			// force termination of string
		n->downstream = fb_down;
		vfb = calloc (1, sizeof(*vfb) + fbarray * sizeof(uint32_t));	//	allocate memory for the bitarray
		vfb->base = fbbase;
		vfb->count = fbcount;
		vfb->bitset = (uint32_t *) vfb->status;		// bitset and status are the same except for the data type
		n->private = vfb;
		BDBnode_insertNode(parent, n);
		data[0] = parent->ntab_version++;
		data[1] = n->localadr;
		memcpy (&data[2], n->uid, BIDIB_UID_LEN);
		log_msg (LOG_INFO, "%s() base=%d count=%d UID=%s\n", __func__, vfb->base, vfb->count, bidib_formatUID(n->uid));
		m = bidib_genMessage(parent, MSG_NODE_NEW, 2 + BIDIB_UID_LEN, data);
		netBDB_postMessages(m);
	}

	return n;
}

/**
 * Create a new virtual s88 feedback module.
 *
 * \param parent		where to position the node in the tree
 * \param serial		up to 12 bits of serial number information, see BDBvn_create()
 * \return				an allocated node structure with cloned feature array and already inserted in the tree
 */
struct bidibnode *BDBvn_newS88 (struct bidibnode *parent, int serial)
{
	return BDBvn_newFeedback(parent, serial, FBTYPE_S88, BIDIB_PID_VIRT_S88, (serial - 1) * 16 + 0, 16);
}

/**
 * Create a new virtual Märklin CAN feedback module.
 *
 * \param parent		where to position the node in the tree
 * \param serial		up to 12 bits of serial number information, see BDBvn_create()
 * \return				an allocated node structure with cloned feature array and already inserted in the tree
 */
struct bidibnode *BDBvn_newMCAN (struct bidibnode *parent, int serial)
{
	return BDBvn_newFeedback(parent, serial, FBTYPE_MCAN, BIDIB_PID_VIRT_MCAN, (serial - 1) * 16 + FB_MCAN_OFFSET, 16);
}

/**
 * Create a new virtual LocoNet feedback module.
 *
 * \param parent		where to position the node in the tree
 * \param serial		up to 12 bits of serial number information, see BDBvn_create()
 * \return				an allocated node structure with cloned feature array and already inserted in the tree
 */
struct bidibnode *BDBvn_newLNET (struct bidibnode *parent, int serial)
{
	return BDBvn_newFeedback(parent, serial, FBTYPE_LNET, BIDIB_PID_VIRT_LNET, (serial - 1) * 16 + FB_LNET_OFFSET, 16);
}

int BDBvn_feedbackModules (int oldcount, int count, int maxcount, int hubSerial)
{
	struct bidibnode *root, *n;
	uint8_t uid[BIDIB_UID_LEN];

	if (count > maxcount) count = maxcount;
	if (count < 0) count = 0;
	if (oldcount < 0) oldcount = 0;
	BDBvn_createUID(uid, BIDIB_CLASS_BRIDGE, BIDIB_PID_VIRT_HUB, hubSerial);
	root = BDBnode_lookupNodeByUID(uid, NULL);
	if (!root) oldcount = 0;

	if (oldcount != count) {
		if (count == 0 && root) {
			BDBnode_dropNode(root);
		} else {
			if (!root) root = BDBvn_newBridge(BDBnode_getRoot(), hubSerial);
			while (root && oldcount < count) {
				oldcount++;
				if (hubSerial == BIDIB_HUB_S88) BDBvn_newS88(root, oldcount);
				else if (hubSerial == BIDIB_HUB_MCAN) BDBvn_newMCAN(root, oldcount);
				else if (hubSerial == BIDIB_HUB_LNET) BDBvn_newLNET(root, oldcount);
			}
			while (root && oldcount > count) {
				if ((n = BDBnode_lookupChild(root, oldcount)) != NULL) BDBnode_dropNode(n);
				oldcount--;
			}
		}
	}

	return count;
}

void BDBvn_clearFbMappings (void)
{
	struct vfeedback *vfp;

	while ((vfp = virtual_fb) != NULL) {
		virtual_fb = vfp->next;
		free (vfp);
	}
}

static int BDBvn_fbMapping (uint8_t prodId, int idx, int fbbase, int fbcount)
{
	struct bidibnode *n;
	uint8_t uid[BIDIB_UID_LEN];

	if (idx <= 0 || idx > BIDIB_MAX_FB_PER_TYPE) return -1;		// only #1 to #4095 are allowed as index
	if (fbbase < s88_getModules() * 16 || fbbase >= MAX_FEEDBACKS) return -2;
	if (fbbase + fbcount > MAX_FEEDBACKS) fbcount = MAX_FEEDBACKS - fbbase;
	BDBvn_createUID(uid, BIDIB_CLASS_OCCUPANCY, prodId, idx);
	if (( n = BDBnode_lookupNodeByUID(uid, NULL)) != NULL) {	// update this node
	} else {													// create a new node
	}
	return 0;
}

int BDBvn_lnetMapping (int idx, int fbbase, int fbcount)
{
	return BDBvn_fbMapping(BIDIB_PID_VIRT_LNET, idx, fbbase, fbcount);
}

int BDBvn_mcanMapping (int idx, int fbbase, int fbcount)
{
	return BDBvn_fbMapping(BIDIB_PID_VIRT_MCAN, idx, fbbase, fbcount);
}

#if 0	/* currently not used */
void BDBvn_feedbackStatus (struct bidibnode *n, uint16_t newstate)
{
	struct virtual_feedback *vfb;
	bidibmsg_t *m = NULL;
	uint8_t st[2], mask8, *p, data[4], diff, mnum;
	uint16_t mask16;
	int bits, msg;

	if (!n || ((vfb = n->private) == NULL)) return;

	// first map 16 bits from newstate (s88 bit logic) to two bytes (BiDiB logic)
	st[0] = st[1] = 0;
	for (mask16 = 0x8000, mask8 = 0x01, p = st; mask16; mask16 >>= 1, mask8 <<= 1) {
		if (!mask8) {
			p++;
			mask8 = 0x01;
		}
		if (newstate & mask16) *p |= mask8;
	}
	if (st[0] == vfb->status[0] && st[1] == vfb->status[1]) return;	// nothing to report

	// now let's see, if this is a single event
	bits = bc_byte(st[0] ^ vfb->status[0]) + bc_byte(st[1] ^ vfb->status[1]);
	if (bits > 1) {						// report with MSG_BM_MULTIPLE, always report both bytes
		p = data;
		*p++ = 0;
		*p++ = 16;
		*p++ = st[0];
		*p++ = st[1];
		m = bidib_genMessage(n, MSG_BM_MULTIPLE, 4, data);
	} else {							// report a single bit using MSG_BM_OCC or MSG_BM_FREE
		if (st[0] != vfb->status[0]) {		// the difference is in the first 8 bits
			mnum = 0;
			diff = st[0] ^ vfb->status[0];
			msg = (diff & st[0]) ? MSG_BM_OCC : MSG_BM_FREE;
		} else {							// the difference is in the second 8 bits
			mnum = 8;
			diff = st[1] ^ vfb->status[1];
			msg = (diff & st[1]) ? MSG_BM_OCC : MSG_BM_FREE;
		}
		while ((diff >>= 1) != 0) mnum++;
		m = bidib_genMessage(n, msg, 1, &mnum);
	}

	// finally update the internal status and send the message
	vfb->status[0] = st[0];
	vfb->status[1] = st[1];
	if (m) BDBnode_uplink(NULL, m);
}

/**
 * Look up a node that represents the feedback of the given index.
 * The index is taken as zero-based s88 module index number, each
 * supporting 16 feedback bits.
 *
 * The serial numbers (12 bits of the UID serial number) of the created
 * feedback modules start with 1 for s88-index 0!
 *
 * \param prodId	the type of feedbackmodule (BIDIB_PID_VIRTUAL_xxx with xxx = S88 / MCAN / LNET
 * \param idx		the zero based index in s88 modules
 * \return			the found node or NULL, if such a node does not exist
 */
struct bidibnode *BDBvn_getFeebackNode (int prodId, int idx)
{
	uint8_t uid[BIDIB_UID_LEN];

	BDBvn_createUID(uid, BIDIB_CLASS_OCCUPANCY, prodId, idx + 1);
	return BDBnode_lookupNodeByUID(uid, NULL);
}
#endif
