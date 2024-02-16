/*
 * bidibutil.c
 *
 *  Created on: 28.12.2020
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

#include <stdio.h>
#include <string.h>
#include "rb2.h"
#include "config.h"
#include "bidib.h"

/* ======================================================================================= */
/* ===== BiDiB debug infrastructure ====================================================== */
/* ======================================================================================= */

/**
 * A strcuture to hold interpreter information used for the debug output
 */
struct decoder {
	uint8_t		msg;							///< the message byte
	char		*command;						///< a DEBUG string for this command code
	char *(*handler)(uint8_t *data, int len);	///< a function that formats and returns the contents of the message data
};

static SemaphoreHandle_t mutex;

static char *bidib_outputClock (uint8_t *data, int len);
static char *bidib_outputAccessory (uint8_t *data, int len);
static char *bidib_outputNodeUID (uint8_t *data, int len);
static char *bidib_outputUID (uint8_t *data, int len);
static char *bidib_outputString (uint8_t *data, int len);
static char *bidib_outputPascalString (uint8_t *data, int len);
static char *bidib_outputNamespaceString (uint8_t *data, int len);
static char *bidib_outputLocalLink (uint8_t *data, int len);
static char *bidib_outputProtocollVersion (uint8_t *data, int len);

#define DECODER(x, f)	{ x, #x, f }

// All downward message codes with MSB cleared (i.e. 0x00 .. 0x7F)
static const struct decoder bidib_downDecoder[] = {
	DECODER (MSG_SYS_GET_MAGIC,				NULL ),
	DECODER (MSG_SYS_GET_P_VERSION,			NULL ),
	DECODER (MSG_SYS_ENABLE,				NULL ),
	DECODER (MSG_SYS_DISABLE,				NULL ),
	DECODER (MSG_SYS_GET_UNIQUE_ID,			NULL ),
	DECODER (MSG_SYS_GET_SW_VERSION,		NULL ),
	DECODER (MSG_SYS_PING,					NULL ),
	DECODER (MSG_SYS_IDENTIFY,				NULL ),
	DECODER (MSG_SYS_RESET,					NULL ),
	DECODER (MSG_GET_PKT_CAPACITY,			NULL ),
	DECODER (MSG_NODETAB_GETALL,			NULL ),
	DECODER (MSG_NODETAB_GETNEXT,			NULL ),
	DECODER (MSG_NODE_CHANGED_ACK,			NULL ),
	DECODER (MSG_SYS_GET_ERROR,				NULL ),
	DECODER (MSG_FW_UPDATE_OP,				NULL ),

	//-- feature and user config messages
	DECODER (MSG_FEATURE_GETALL,			NULL ),
	DECODER (MSG_FEATURE_GETNEXT,			NULL ),
	DECODER (MSG_FEATURE_GET,				NULL ),
	DECODER (MSG_FEATURE_SET,				NULL ),
	DECODER (MSG_VENDOR_ENABLE,				NULL ),
	DECODER (MSG_VENDOR_DISABLE,			NULL ),
	DECODER (MSG_VENDOR_SET,				NULL ),
	DECODER (MSG_VENDOR_GET,				NULL ),
	DECODER (MSG_SYS_CLOCK,					bidib_outputClock ),
	DECODER (MSG_STRING_GET,				NULL ),
	DECODER (MSG_STRING_SET,				NULL ),

	//-- occupancy messages
	DECODER (MSG_BM_GET_RANGE,				NULL ),
	DECODER (MSG_BM_MIRROR_MULTIPLE,		NULL ),
	DECODER (MSG_BM_MIRROR_OCC,				NULL ),
	DECODER (MSG_BM_MIRROR_FREE,			NULL ),
	DECODER (MSG_BM_ADDR_GET_RANGE,			NULL ),
	DECODER (MSG_BM_GET_CONFIDENCE,			NULL ),
	DECODER (MSG_BM_MIRROR_POSITION,		NULL ),

	//-- booster messages
	DECODER (MSG_BOOST_OFF,					NULL ),
	DECODER (MSG_BOOST_ON,					NULL ),
	DECODER (MSG_BOOST_QUERY,				NULL ),

	//-- accessory control messages
	DECODER (MSG_ACCESSORY_SET,				NULL ),
	DECODER (MSG_ACCESSORY_GET,				NULL ),
	DECODER (MSG_ACCESSORY_PARA_SET,		NULL ),
	DECODER (MSG_ACCESSORY_PARA_GET,		NULL ),
	DECODER (MSG_ACCESSORY_GETALL,			NULL ),

	//-- switch/light/servo control messages
	DECODER (MSG_LC_PORT_QUERY_ALL,			NULL ),
	DECODER (MSG_LC_OUTPUT,					NULL ),
	DECODER (MSG_LC_CONFIG_SET,				NULL ),
	DECODER (MSG_LC_CONFIG_GET,				NULL ),
	DECODER (MSG_LC_KEY_QUERY,				NULL ),
	DECODER (MSG_LC_OUTPUT_QUERY,			NULL ),
	DECODER (MSG_LC_PORT_QUERY,				NULL ),
	DECODER (MSG_LC_CONFIGX_GET_ALL,		NULL ),
	DECODER (MSG_LC_CONFIGX_SET,			NULL ),
	DECODER (MSG_LC_CONFIGX_GET,			NULL ),

	//-- macro messages
	DECODER (MSG_LC_MACRO_HANDLE,			NULL ),
	DECODER (MSG_LC_MACRO_SET,				NULL ),
	DECODER (MSG_LC_MACRO_GET,				NULL ),
	DECODER (MSG_LC_MACRO_PARA_SET,			NULL ),
	DECODER (MSG_LC_MACRO_PARA_GET,			NULL ),

	//-- distributed control messages
	DECODER (MSG_DDIS,						NULL ),

	DECODER (MSG_CS_ALLOCATE,				NULL ),
	DECODER (MSG_CS_SET_STATE,				NULL ),
	DECODER (MSG_CS_DRIVE,					NULL ),
	DECODER (MSG_CS_ACCESSORY,				bidib_outputAccessory ),
	DECODER (MSG_CS_BIN_STATE,				NULL ),
	DECODER (MSG_CS_POM,					NULL ),
	DECODER (MSG_CS_RCPLUS,					NULL ),
	DECODER (MSG_CS_M4,						NULL ),
	DECODER (MSG_CS_QUERY,					NULL ),
	DECODER (MSG_CS_DCCA,					NULL ),

	//-- service mode
	DECODER (MSG_CS_PROG,					NULL ),

	DECODER (MSG_LOCAL_LOGON_ACK,			bidib_outputNodeUID ),
	DECODER (MSG_LOGON_ACK,					NULL ),
	DECODER (MSG_LOCAL_PING,				NULL ),
	DECODER (MSG_LOCAL_LOGON_REJECTED,		bidib_outputUID ),
	DECODER (MSG_LOGON_REJECTED,			NULL ),
	DECODER (MSG_LOCAL_ACCESSORY,			NULL ),
	DECODER (MSG_LOCAL_SYNC,				NULL ),
	DECODER (MSG_LOCAL_DISCOVER,			NULL ),
	DECODER (MSG_LOCAL_BIDIB_DOWN,			NULL ),


	{ 0, NULL, NULL }
};

// All upward message codes with MSB set (i.e. 0x80 .. 0xFF)
static const struct decoder bidib_upDecoder[] = {
	DECODER (MSG_SYS_MAGIC,					NULL ),
	DECODER (MSG_SYS_PONG,					NULL ),
	DECODER (MSG_SYS_P_VERSION,				NULL ),
	DECODER (MSG_SYS_UNIQUE_ID,				NULL ),
	DECODER (MSG_SYS_SW_VERSION,			NULL ),
	DECODER (MSG_SYS_ERROR,					NULL ),
	DECODER (MSG_SYS_IDENTIFY_STATE,		NULL ),
	DECODER (MSG_NODETAB_COUNT,				NULL ),
	DECODER (MSG_NODETAB,					NULL ),
	DECODER (MSG_PKT_CAPACITY,				NULL ),
	DECODER (MSG_NODE_NA,					NULL ),
	DECODER (MSG_NODE_LOST,					NULL ),
	DECODER (MSG_NODE_NEW,					NULL ),
	DECODER (MSG_STALL,						NULL ),
	DECODER (MSG_FW_UPDATE_STAT,			NULL ),

	DECODER (MSG_FEATURE,					NULL ),
	DECODER (MSG_FEATURE_NA,				NULL ),
	DECODER (MSG_FEATURE_COUNT,				NULL ),
	DECODER (MSG_VENDOR,					NULL ),
	DECODER (MSG_VENDOR_ACK,				NULL ),
	DECODER (MSG_STRING,					bidib_outputNamespaceString ),

	DECODER (MSG_BM_OCC,					NULL ),
	DECODER (MSG_BM_FREE,					NULL ),
	DECODER (MSG_BM_MULTIPLE,				NULL ),
	DECODER (MSG_BM_ADDRESS,				NULL ),
	DECODER (MSG_BM_ACCESSORY,				NULL ),
	DECODER (MSG_BM_CV,						NULL ),
	DECODER (MSG_BM_SPEED,					NULL ),
	DECODER (MSG_BM_CURRENT,				NULL ),
	DECODER (MSG_BM_BLOCK_CV,				NULL ),
	DECODER (MSG_BM_XPOM,					NULL ),
	DECODER (MSG_BM_CONFIDENCE,				NULL ),
	DECODER (MSG_BM_DYN_STATE,				NULL ),
	DECODER (MSG_BM_RCPLUS,					NULL ),
	DECODER (MSG_BM_DCCA,					NULL ),
	DECODER (MSG_BM_POSITION,				NULL ),

	DECODER (MSG_BOOST_STAT,				NULL ),
	DECODER (MSG_BOOST_CURRENT,				NULL ),
	DECODER (MSG_BOOST_DIAGNOSTIC,			NULL ),

	DECODER (MSG_ACCESSORY_STATE,			NULL ),
	DECODER (MSG_ACCESSORY_PARA,			NULL ),
	DECODER (MSG_ACCESSORY_NOTIFY,			NULL ),

	DECODER (MSG_LC_STAT,					NULL ),
	DECODER (MSG_LC_NA,						NULL ),
	DECODER (MSG_LC_CONFIG,					NULL ),
	DECODER (MSG_LC_KEY,					NULL ),
	DECODER (MSG_LC_WAIT,					NULL ),
	DECODER (MSG_LC_CONFIGX,				NULL ),

	DECODER (MSG_LC_MACRO_STATE,			NULL ),
	DECODER (MSG_LC_MACRO,					NULL ),
	DECODER (MSG_LC_MACRO_PARA,				NULL ),

	DECODER (MSG_UDIS,						NULL ),

	DECODER (MSG_CS_ALLOC_ACK,				NULL ),
	DECODER (MSG_CS_STATE,					NULL ),
	DECODER (MSG_CS_DRIVE_ACK,				NULL ),
	DECODER (MSG_CS_ACCESSORY_ACK,			NULL ),
	DECODER (MSG_CS_POM_ACK,				NULL ),
	DECODER (MSG_CS_DRIVE_MANUAL,			NULL ),
	DECODER (MSG_CS_DRIVE_EVENT,			NULL ),
	DECODER (MSG_CS_ACCESSORY_MANUAL,		NULL ),
	DECODER (MSG_CS_RCPLUS_ACK,				NULL ),
	DECODER (MSG_CS_M4_ACK,					NULL ),
	DECODER (MSG_CS_DRIVE_STATE,			NULL ),
	DECODER (MSG_CS_DCCA_ACK,				NULL ),

	DECODER (MSG_CS_PROG_STATE,				NULL ),

	DECODER (MSG_LOCAL_LOGON,				bidib_outputUID ),
	DECODER (MSG_LOCAL_PONG,				NULL ),
	DECODER (MSG_LOCAL_LOGOFF,				bidib_outputUID ),
	DECODER (MSG_LOCAL_ANNOUNCE,			NULL ),
	DECODER (MSG_LOCAL_BIDIB_UP,			NULL ),

	DECODER (MSG_LOCAL_PROTOCOL_SIGNATURE,	bidib_outputString ),
	DECODER (MSG_LOCAL_LINK,				bidib_outputLocalLink ),

	{ 0, NULL, NULL }
};

static const struct decoder bidib_localLinkDecoder[] = {
	DECODER (BIDIB_LINK_DESCRIPTOR_PROD_STRING,	bidib_outputPascalString ),
	DECODER (BIDIB_LINK_DESCRIPTOR_USER_STRING,	bidib_outputPascalString ),
	DECODER (BIDIB_LINK_DESCRIPTOR_P_VERSION,	bidib_outputProtocollVersion ),
	DECODER (BIDIB_LINK_NODE_UNAVAILABLE,		NULL ),
	DECODER (BIDIB_LINK_NODE_AVAILABLE ,		NULL ),
	DECODER (BIDIB_LINK_PAIRING_REQUEST,		NULL ),
	DECODER (BIDIB_LINK_STATUS_UNPAIRED,		NULL ),
	DECODER (BIDIB_LINK_STATUS_PAIRED,			NULL ),
	DECODER (BIDIB_LINK_DESCRIPTOR_UID,			NULL ),

	{ 0, NULL, NULL }
};

static const struct decoder bidib_errmsgs[] = {
	DECODER (BIDIB_ERR_NONE,					NULL ),
	DECODER (BIDIB_ERR_TXT,						NULL ),
	DECODER (BIDIB_ERR_CRC,						NULL ),
	DECODER (BIDIB_ERR_SIZE,					NULL ),
	DECODER (BIDIB_ERR_SEQUENCE,				NULL ),
	DECODER (BIDIB_ERR_PARAMETER,				NULL ),
	DECODER (BIDIB_ERR_BUS,						NULL ),
	DECODER (BIDIB_ERR_ADDRSTACK,				NULL ),
	DECODER (BIDIB_ERR_IDDOUBLE,				NULL ),
	DECODER (BIDIB_ERR_SUBCRC,					NULL ),
	DECODER (BIDIB_ERR_SUBTIME,					NULL ),
	DECODER (BIDIB_ERR_SUBPAKET,				NULL ),
	DECODER (BIDIB_ERR_OVERRUN,					NULL ),
	DECODER (BIDIB_ERR_HW,						NULL ),
	DECODER (BIDIB_ERR_RESET_REQUIRED,			NULL ),
	DECODER (BIDIB_ERR_NO_SECACK_BY_HOST,		NULL ),

	{ 0, NULL, NULL }
};

static const struct decoder *bidib_matchCommand (uint8_t cmd, const struct decoder *d)
{
	while (d && d->command && d->msg != cmd) d++;
	return d;
}

static char *bidib_outputString (uint8_t *data, int len)
{
	char *tmp = tmp64();

	snprintf (tmp, 64, "%*.*s", len, len, (char *) data);
	return tmp;
}

static char *bidib_outputPascalString (uint8_t *data, int len)
{
	char *tmp = tmp64();

	len = *data++;
	snprintf (tmp, 64, "%*.*s", len, len, (char *) data);
	return tmp;
}

static const char *bidib_namepsace (int id)
{
	if (id == 0) return "VARIABLES";
	if (id == 1) return "DEBUG-Streams";
	return "(reserved)";
}

static char *bidib_outputNamespaceString (uint8_t *data, int len)
{
	char *tmp = tmp256();
	int namespace, strid, length;

	(void) len;

	namespace = *data++;
	strid = *data++;
	length = *data++;
	snprintf (tmp, 256, "NS=%d [%s] ID=%d \"%*.*s\"", namespace, bidib_namepsace(namespace), strid, length, length, (char *) data);
	return tmp;
}

static const char *wdays[] = {
	"Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"
};

static char *bidib_outputClock (uint8_t *data, int len)
{
	char *tmp = tmp64();
	int minute, hour, wday, factor;

	(void) len;

	if ((minute = data[0] & 0x3F) > 59) minute = 0;
	if ((hour   = data[1] & 0x1F) > 23) hour = 0;
	if ((wday   = data[2] & 0x07) >  6) wday = 0;
	if ((factor = data[3] & 0x1F) > 31) factor = 1;
	snprintf (tmp, 64, "%s, %d:%02d F=%d", wdays[wday], hour, minute, factor);
	return tmp;
}

static char *bidib_outputProtocollVersion (uint8_t *data, int len)
{
	char *tmp = tmp64();

	(void) len;

	snprintf (tmp, 64, "%d.%d", data[1], data[0]);
	return tmp;
}

/**
 * Log an UID to log console
 *
 * \param data		pointer to the UID bytes
 * \param len		length of data (not used here, must always be 7, just to have a matching prototype for decoder table)
 */
static char *bidib_outputUID (uint8_t *data, int len)
{
	char *tmp = tmp64();

	(void) len;

	snprintf (tmp, 64, "%s", bidib_formatUID(data));
	return tmp;
}

static char *bidib_outputAccessory (uint8_t *data, int len)
{
	char *tmp = tmp64();
	char aspect[8];
	uint16_t adr;
	bool ext, timing, on;
	int tim;

	(void) len;

	adr = ((data[1] << 8) | data[0]) - 3;
	ext = !!(data[2] & 0x80);
	timing = !!(data[2] & 0x40);
	on = !!(data[2] & 0x20);

	if (ext) {
		sprintf (aspect, "%d", data[2] & 0x1F);
	} else {
		sprintf (aspect, "%c", (data[2] & 0x1F) ? 'G' : 'R');
	}
	if (timing) {
		tim = data[3] & 0x7F;
		if (data[3] & 0x80) tim *= 10;
		snprintf (tmp, 64, "%s-ACC %d -> %s %d.%ds", (ext) ? "EXTENDED" : "BASIC", adr, aspect, tim / 10, tim % 10);
	} else {
		snprintf (tmp, 64, "%s-ACC %d -> %s %s", (ext) ? "EXTENDED" : "BASIC", adr, aspect, (on) ? "ON" : "OFF");
	}
	return tmp;
}

/**
 * Log the node address and an UID to log console
 *
 * \param data		pointer to the node address and UID bytes
 * \param len		length of data (not used here, must always be 7, just to have a matching prototype for decoder table)
 */
static char *bidib_outputNodeUID (uint8_t *data, int len)
{
	char *tmp = tmp64();

	(void) len;

	snprintf (tmp, 64, "%d %s", *data, bidib_formatUID(data + 1));
	return tmp;
}

static char *bidib_dump (uint8_t *data, int len)
{
	char *dump, *s;
	int i;

	s = dump = tmp256();
	for (i = 0; i < len; i++) {
		if (i) *s++ = ' ';
		s += sprintf (s, "%02x", data[i]);
	}
	*s = 0;
	return dump;
}

static char *bidib_outputLocalLink (uint8_t *data, int len)
{
	const struct decoder *d;
	char *buf, msgnum[8], *content;

	sprintf (msgnum, "0x%02x", *data);	// just in case this message is not known

	d = bidib_matchCommand(*data, bidib_localLinkDecoder);
	if (d->command && d->handler) {
		content = d->handler(data + 1, len - 1);
	} else {
		content = bidib_dump(data + 1, len - 1);
	}
	buf = tmp256();
	sprintf (buf, "%s %s", (d->command) ? d->command : msgnum, content);
	return buf;
}

void bidib_debugSingleMessage (const char *caller, struct bidibmsg *msg, bool up)
{
	const struct decoder *d;
	char msgnum[8], *content;

	if (!mutex_lock(&mutex, 5, __func__)) {
		log_error ("%s(%s): cannot accquire MUTEX\n", __func__, up ? "TX" : "RX");
		return;
	}
	sprintf (msgnum, "0x%02x", msg->msg);	// just in case this message is not known
	d = bidib_matchCommand(msg->msg, (msg->msg & 0x80) ? bidib_upDecoder : bidib_downDecoder);
	if (d->command && d->handler) {
		content = d->handler(msg->data, msg->datalen);
	} else {
		content = bidib_dump(msg->data, msg->datalen);
	}

	log_msg (LOG_BIDIB, "%s %s() %s [%s #%d] %s\n",
			up ? "^^" : "vv", (caller) ? caller : __func__, (d->command) ? d->command : msgnum,
			bidib_formatAdrStack(msg->adrstack), msg->seq, content);
	mutex_unlock(&mutex);
}

void bidib_debugMessages (const char *caller, struct bidibmsg *msg, bool up)
{
	while (msg) {
		bidib_debugSingleMessage(caller, msg, up);
		msg = msg->next;
	}
}

void bidib_debugError (const char *caller, struct bidibmsg *msg)
{
	const struct decoder *d;
	char errnum[8], *content;

	if (!mutex_lock(&mutex, 5, __func__)) {
		log_error ("%s(): cannot accquire MUTEX\n", __func__);
		return;
	}
	sprintf (errnum, "0x%02x", msg->data[0]);	// just in case this message is not known
	d = bidib_matchCommand(msg->data[0], bidib_errmsgs);
	if (d->command && d->handler) {
		content = d->handler(msg->data, msg->datalen);
	} else {
		content = bidib_dump(msg->data, msg->datalen);
	}

	log_msg (LOG_BIDIB, "%s() %s [%s #%d] %s\n",
			(caller) ? caller : __func__, (d->command) ? d->command : errnum,
			bidib_formatAdrStack(msg->adrstack), msg->seq, content);
	mutex_unlock(&mutex);
}

/**
 * Format an UID to a temporary string.
 *
 * \param uid		pointer to the seven bytes that make up the UID for be formatted
 * \return			pointer to a tmp64() string containing the formatted output
 * \see				tmp64()
 */
char *bidib_formatUID (uint8_t *uid)
{
	char *tmp = tmp64();

	sprintf (tmp, "0x%02x 0x%02x 0x%02x 0x%02x%02x%02x%02x", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6]);
	return tmp;
}

/* ======================================================================================= */
/* ===== BiDiB Device Address functions ================================================== */
/* ======================================================================================= */
/*
 * Addresses in BiDiB are of variable length and can use up to five bytes.
 * In the message they are encoded as one byte for each level, starting with the address
 * byte of the highest level and then subdividing the node tree down to level 4. This
 * address stack is than terminated with a null byte, and so may be 5 bytes in total when
 * all four levels are used (the real address then makes up a 32bit integer).
 *
 * Inside this software, we want to map this variable-length address representation to an
 * easier manageable uint32_t. The topmost byte will represent the top level, the next
 * lower byte is level 2 and so on. A zero in any byte of this uint32_t will implicitly
 * clear all lower bytes, because an address like 1.0.3 (0x01000300 as uint32_t) is not
 * allowed in the system.
 *
 * A third representation of the address is the string format, which is intended for humans.
 * The levels are separated by dots, similar to an IPv4 numeric address but with variable
 * length.
 *
 * The following functions deal with the conversion between these representations, whereas
 * the universal internal representation with an uint32_t is used as the central format.
 */

/**
 * Interpret the address stack from a raw packet buffer and return
 * the result as 32 bit unsigned.
 *
 * \param p		pointer to the bytes that represent the address stack
 * \return		the internal representation of the address found in the packet
 */
adrstack_t bidib_adrReadMsg (uint8_t *p)
{
	uint32_t stack;

	if (p == NULL || *p == 0) return 0;
	stack = 0;
	while (*p && !(stack & (0xFF << 24))) {
		stack <<= 8;
		stack |= *p++;
	}
	while (!(stack & (0xFF << 24))) stack <<= 8;		// shift the topmost level up the MSB of the stack word

	return stack;
}

/**
 * Write the internal stack representation to a message buffer.
 *
 * \param p			pointer to the message buffer where we should plate the address stack bytes
 * \param stack		the address stack in internal representation
 * \return			the new pointer into the message buffer, where the message number (index) should be placed
 */
uint8_t *bidib_adrWriteMsg (uint8_t *p, adrstack_t stack)
{
	if (p) {
		*p = (stack >> 24) & 0xFF;
		if (*p) { *++p = (stack >> 16) & 0xFF; }
		if (*p) { *++p = (stack >>  8) & 0xFF; }
		if (*p) { *++p = (stack >>  0) & 0xFF; }
		if (*p) *++p = 0;	// terminate the last level if it wasn't terminated earlier
		p++;
	}
	return p;
}

/**
 * Calculate the length of the address stack when serializing to a byte buffer.
 * This can be one to five bytes.
 *
 * \param stack		the address stack as unsigned integer with the least significant byte as the first layer
 * \return			the number of bytes necessary for representing the address stack in the message buffer
 */
static int bidib_adrStackLen (uint32_t stack)
{
	int adrlen;

	adrlen = 1;
	while (stack) {
		adrlen++;
		stack <<= 8;
	}
	return adrlen;
}

/**
 * Format the address stack as human readable string. The string returned is a
 * temporaray buffer taken from tmp64().
 *
 * Interpreting the address stack has one exception: if the stack is zero, the
 * local node or a broadcast is addressed. In this case we display that as "0".
 *
 * \param stack		the address stack to format to a human readable string
 * \return			a temporary string with ASCII reprensentation of the address stack in decimal dotted format
 */
char *bidib_formatAdrStack (adrstack_t stack)
{
	char *result, *s;

	result = tmp64();
	if (stack == 0) {
		sprintf (result, "0");
	} else {
		s = result;
		while (stack) {
			if (s > result) *s++ = '.';
			s += sprintf (s, "%lu", (stack >> 24) & 0xFF);
			stack <<= 8;
		}
	}

	return result;
}

/**
 * Create a subaddress of the given node address.
 *
 * \param nodeadr	the address of the node that controls the subnode (i.e. the hub)
 * \param subadr	the subaddress on the hub's local bus
 * \return			a new node address stack with the resulting bytes or 0 if no deeper
 * 					level can be excepted
 */
//adrstack_t bidib_genSubAdr (adrstack_t nodeadr, uint8_t subadr)
//{
//	adrstack_t adr, mask;
//
//	adr = subadr << 24;;
//	mask = 0xFF << 24;
//	while (mask && (nodeadr & mask)) {
//		mask >>= 8;
//		adr >>= 8;
//	}
//	if (!mask) return 0;	// error - can't add a deeper level
//	adr |= nodeadr;
//	return adr;
//}

/**
 * Take a numerical value with the LSB representing the lowest level and having
 * superior levels upward from that. For our internal representation, we need
 * to shift the address left as long as the uppermost byte (MSB) is zero. As a
 * special value, zero itself will always give zero as the result.
 *
 * \param adr		the right aligned address with up to four bytes of address information
 * \return			the left aligned representations as adrstack_t
 */
adrstack_t bidib_num2stack (uint32_t adr)
{
	while (adr && ((adr & 0xFF000000) == 0)) adr <<= 8;
	return adr;
}

static adrstack_t _bidib_getAddress (struct bidibnode *n, adrstack_t adr)
{
	if (!n || n->localadr == 0) return adr;		// no parent node or parent is the root node with address 0.0.0.0
	adr >>= 8;
	adr |= n->localadr << 24;
	return _bidib_getAddress(n->parent, adr);
}

adrstack_t bidib_getAddress (struct bidibnode *n)
{
	return _bidib_getAddress(n, 0);
}

/* ======================================================================================= */
/* ===== BiDiB packet generation functions =============================================== */
/* ======================================================================================= */

/**
 * Calculate the number of bytes needed to serialize a message to a byte buffer.
 * This includes all bytes needed, even the leading length byte which later must
 * be set to the length excluding itself!
 *
 * \param bm		the message to serialize
 * \return			number of byte this message will occupy in a message packet
 */
int bidib_packSize (bidibmsg_t *bm)
{
	if (bm) {
		return bm->datalen + 3 + bidib_adrStackLen(bm->adrstack);		// account for M_LENGTH, MSG_NUM, MSG_TYPE and adress stack
	}
	return 0;
}

/**
 * Add a UID to a data buffer.
 *
 * If the UID is NULL, the own UID is taken instead.
 *
 * \param p		the current buffer pointer inside the packet data
 * \param uid	pointer to the seven bytes that make up the UID
 * \return		the buffer pointer after adding the UID bytes
 */
uint8_t *bidib_addUID (uint8_t *p, uint8_t *uid)
{
	int i;

	if (!p) return NULL;
	if (!uid) uid = myUID;

	if (uid) {		// add the given UID
		for (i = 0; i < 7; i++) {
			*p++ = *uid++;
		}
	}

	return p;
}

/**
 * Add a string with prepended length information (like a PASCAL string).
 * If the string is not given or empty or if the maximum allowed length
 * is 0 (or even negative) no string is placed in the data block but the
 * length information is supplied as 0 bytes (occupying one byte!).
 *
 * The function may return a NULL pointer, if the mesgdata pointer is not supplied.
 *
 * \param msgdata	pointer to the data block where the string should be placed
 * \param str		the string that should be written to the data block
 * \param maxlen	the maximum length of the string itself (not including the length byte)
 * \return			a pointer to the position right behind the added string
 */
uint8_t *bidib_addString (uint8_t *msgdata, char *str, int maxlen)
{
	if (!msgdata) return NULL;
	if (str && *str && maxlen > 0) {
		*msgdata = sprintf ((char *) &msgdata[1], "%.*s", maxlen, str);
	} else {
		*msgdata = 0;
	}

	return &msgdata[*msgdata + 1];
}

/**
 * Add a string with prepended namespace, string ID and length information.
 * The function works just like bidib_addString() above but additionally
 * sets the namespace and string ID values in addition to the string length
 * information.
 *
 * The function may return a NULL pointer, if the mesgdata pointer is not supplied.
 *
 * \param msgdata	pointer to the data block where the string should be placed
 * \param ns		the namespace (currently NS 0 and NS 1 are defined)
 * \param id		the string ID in the namespace, each namespace has different meaning for the ID used
 * \param str		the string that should be written to the data block
 * \param maxlen	the maximum length of the string itself (not including the length byte)
 * \return			a pointer to the position right behind the added string
 * \see				bidib_addString()
 */
uint8_t *bidib_addNsIdString (uint8_t *msgdata, uint8_t ns, uint8_t id, char *str, int maxlen)
{
	if (!msgdata) return NULL;

	*msgdata++ = ns;
	*msgdata++ = id;
	return bidib_addString(msgdata, str, maxlen);
}

/**
 * Free the list of messages.
 *
 * \param m		the list of messages (singly linked list)
 */
void bidib_freeMessages (bidibmsg_t *m)
{
	bidibmsg_t *tmp;

	while ((tmp = m) != NULL) {
		m = m->next;
		free (tmp);
	}
}

/**
 * Utility function to create a bidibmsg_t structure from single components.
 *
 * \param n				the bidib node that is creating this message (for address stack and sequence numbering)
 * \param msg			the message type code MSG_TYPE, i.e. the command
 * \param len			the length of the following data byte array
 * \param data			the optional data bytes as parameters of the msg (= MSG_TYPE)
 * \return				a dynamically allocated packet block or NULL on any type of error
 */
bidibmsg_t *bidib_genMessage (struct bidibnode *n, uint8_t msg, int len, uint8_t *data)
{
	bidibmsg_t *bm;

	if (len && !data) return NULL;		// data field is announced but missing

	if ((bm = calloc (1, sizeof(*bm) + len)) != NULL) {
		if (n) {		// if this is node-related, fill in the address stack and probably the message sequence
			bm->adrstack = bidib_getAddress(n);
			if (!bidib_isLocal(msg)) {
				bm->seq = n->txmsgnum;
				if (++n->txmsgnum == 0) n->txmsgnum = 1;
			}
		}
		bm->msg = msg;
		bm->datalen = len;
		if (len > 0) memcpy (bm->data, data, len);
	}

	return bm;
}

/**
 * Pack a single message to an array of bytes. The caller is responsable for allocating
 * enough space for the "serialized" packet. This funcion only ever packs one single
 * message to the buffer. Packing a list of messages must be done by the caller, because
 * the buffer sizes of the various transport media vary.
 *
 * \param bm			the structure that should be packet to byte array
 * \param data			the caller provided byte buffer to render the message
 * \return				the new position inside the packet buffer for the next message
 */
uint8_t *bidib_packMessage (bidibmsg_t *bm, uint8_t *data)
{
	uint8_t msglen;

	if (!data) return NULL;
	if (bm) {
		msglen = bidib_packSize(bm);
		*data++ = msglen - 1;		// don't count the M_LENGTH itself
		data = bidib_adrWriteMsg(data, bm->adrstack);
		*data++ = bm->seq;
		*data++ = bm->msg;
		if (bm->datalen) {
			memcpy (data, bm->data, bm->datalen);
			data += bm->datalen;
		}
	}

	return data;		// return the new pointer position for the next packet
}

/**
 * Pack a list of messages to an array of bytes. The caller is responsable for allocating
 * enough space for the "serialized" packets. This funcion may pack several message in
 * one byte buffer, but stops after having used up all space.
 *
 * \param bm			the structure list that should be packet to byte array
 * \param data			the caller provided byte buffer to render the message
 * \param maxlen		the maximum space for the data packing
 * \return				the effective number of bytes put to the data buffer
 */
int bidib_packAllMessages (bidibmsg_t *bm, uint8_t *data, int maxlen)
{
	uint8_t *p;
	int len;

	p = data;
	while (bm) {
		len = bidib_packSize(bm);
		if (len > maxlen) break;
		p = bidib_packMessage(bm, p);
		maxlen -= len;
		bm = bm->next;
	}

	return p - data;
}

/**
 * Unpack a (list of) bidibmsg_t structure(s) from a byte buffer. Since more than one
 * message can be coded in a single packet, we may deliver a list of messages from a
 * single packet.
 *
 * \param pkt			pointer to the byte buffer we should analyse
 * \param packetlen		the length of that buffer
 * \param adr			the local node address from which this packet stems (used for a possible error message)
 * \return				a (list of) message(s) or NULL if no message could be decoded
 */
bidibmsg_t *bidib_unpackMessages (uint8_t *pkt, int packetlen, uint8_t adr)
{
	bidibmsg_t *msgs, **msgpp;
	int messagelen, adrstacklen, datalen;
	uint8_t *end, *p;
	adrstack_t stack;

	if (!pkt || packetlen <= 0) return NULL;
	end = pkt + packetlen;					// points just behind the last byte
	msgs = NULL;
	msgpp = &msgs;
	while (pkt < end) {
		p = pkt;
		messagelen = *p++;							// MSG_LENGTH includes address stack, MSG_NUM, MSG_TYPE and optional message data
		if (messagelen < 3) {
			fprintf (stderr, "%s(): illegal MESSAGE-LENGTH %d - packet ignored\n", __func__, messagelen);
			bidib_errorMessage(LOCAL_NODE(), BIDIB_ERR_SUBPAKET, 1, &adr);
			return msgs;		// return only what we have received as valid packet
		}
		stack = bidib_adrReadMsg(p);
		adrstacklen = bidib_adrStackLen(stack);
		if (adr) stack = (stack >> 8) | (adr << 24);	// insert the address from the node we received this message from
//		log_msg (LOG_BIDIB, "%s(): STACK=0x%08lx, Stacklen=%d\n", __func__, stack, adrstacklen);
		datalen = messagelen - adrstacklen - 2;
		if ((*msgpp = calloc (1, sizeof(*msgs) + datalen)) == NULL) break;
		(*msgpp)->adrstack = stack;
		p += adrstacklen;
		(*msgpp)->seq = *p++;
		(*msgpp)->msg = *p++;
		(*msgpp)->datalen = datalen;
		memcpy ((*msgpp)->data, p, datalen);		// subtract the already interpreted bytes and add one for the MSG_LENGTH byte

		pkt += messagelen + 1;			// the messagelen doesn't include the MSG_LENGTH byte itself
		if (pkt > end) {				// STOP: the message was bigger than the (rest of the) buffer provided - last packet was invalid
			free (*msgpp);
			*msgpp = NULL;
			break;						// we must not continue!
		}
		msgpp = &(*msgpp)->next;
	}

	return msgs;
}

/* ======================================================================================= */
/* ===== BiDiB message handling helpers ================================================== */
/* ======================================================================================= */

/**
 * Check the (downstream) message code for one of the defined
 * broadcast commands. This is independent from an address stack
 * 'zero', because, for example, the interface link is always
 * addressed with a zero-stack, regardless of the command beeing
 * a broadcast or not.
 *
 * Broadcast messages don't use the sequence number (MSG_NUM).
 *
 * \param msgcode	the message byte (MSG_TYPE)
 * \return			true if it is one of the broadcast messages
 */
bool bidib_isBroadcast (uint8_t msgcode)
{
	switch (msgcode) {
		case MSG_SYS_ENABLE:
		case MSG_SYS_DISABLE:
		case MSG_SYS_RESET:
		case MSG_SYS_CLOCK:
		case MSG_BOOST_OFF:
		case MSG_BOOST_ON:
		case MSG_LOCAL_ACCESSORY:
		case MSG_LOCAL_SYNC: return true;
	}
	return false;
}

/**
 * Check the message code should be treated as LOCAL message.
 *
 * LOCAL messages don't use the sequence number (MSG_NUM).
 *
 * \param msgcode	the message byte (MSG_TYPE)
 * \return			true if it is a messages
 */
bool bidib_isLocal (uint8_t msgcode)
{
	return ((msgcode & 0x70) == 0x70);
}

/**
 * Convert a current in mA to the representation in the BiDiB system.
 * This representation is coding the current in a prograssive way.
 *
 * \param current	a current in mA
 * \return			the coded byte that best represents the current
 */
uint8_t bidib_current2code (int current)
{
	if (current <= 0) return 0;
	if (current < 16) return (uint8_t) current;
	if (current < 206) return (uint8_t)((current + 2) / 4 + 12);
	if (current < 1248) return (uint8_t)((current + 8) / 16 + 51);
	if (current < 5344) return (uint8_t)((current + 32) / 64 + 108);
	if (current < 20300) return (uint8_t)((current + 128) / 256 + 171);
	return 254;		// overcurrent code
}

/**
 * Convert a code from the BiDiB system to a current in mA.
 * This coding is done in a progressive way.
 *
 * \param code		the coding of the current as a single byte
 * \return			the current expressed in mA or -1 for a SHORT condition or -2 for reserved codes
 */
int bidib_code2current (uint8_t code)
{
	if (code <= 15) return code;
	if (code <= 63) return (code - 12) * 4;
	if (code <= 127) return (code - 51) * 16;
	if (code <= 191) return (code - 108) * 64;
	if (code <= 250) return (code - 171) * 256;
	if (code <= 253) {
		fprintf (stderr, "%s(): reserved current code %d\n", __func__, code);
		return -2;
	}
	if (code == 254) return -1;			// this codes "SHORT"
	return 0;							// code "unknown current" is expressed as 0
}

/**
 * Convert the interface speed of BiDiB to internal representation.
 * We agree with the DCC format regarding the direction bit 0x80. If it
 * is set, the direction should be forward, else it is reversed.
 * The emergency stop code is defined as a speed of 1 (plus a possible
 * direction bit). This code is then converted to 0x7F plus the given
 * direction bit.
 *
 * \param speed			the speed byte as we received it from the interface
 * \param targetfmt		the format that the loco uses
 * \return				the format dependent speed coding including direction
 * 						or a speed of 0x7F plus direction bit in case of emergency
 * 						stop code
 */
uint8_t bidib_msg2speed (uint8_t speed, enum fmt targetfmt)
{
	int sp;

	sp = speed & 0x7F;
	if (sp == 0) return speed;
	if (sp == 1) return speed | 0x7F;
	switch (db_getSpeeds(targetfmt)) {
		case 14:
			sp = ((sp - 1) + 8) / 9;
			break;
		case 27:
			sp = ((sp - 1) * 3 + 11) / 14;
			break;
		case 28:
			sp = ((sp - 1) * 2 + 7) / 9;
			break;
		case 126:
			sp -= 1;
			break;
	}
	return ((speed & 0x80) | sp);
}

/**
 * Convert the loco speed to BiDiB representation.
 * We agree with the DCC format regarding the direction bit 0x80. If it
 * is set, the direction should be forward, else it is reversed.
 * No emergency stop code is used here, as this is ment to report a real
 * speed and emergency stop yields a zero speed.
 *
 * \param speed			the speed byte from the refresh stack
 * \param sourcefmt		the format that the loco uses
 * \return				the BiDiB speed coding including direction (0, 2 .. 127)
 */
uint8_t bidib_speed2msg (uint8_t speed, enum fmt sourcefmt)
{
	int sp;

	sp = speed & 0x7F;
	if (sp == 0) return speed;
	switch (db_getSpeeds(sourcefmt)) {
		case 14:
			sp = sp * 9 + 1;
			break;
		case 27:
			sp = (sp * 14) / 3 + 1;
			break;
		case 28:
			sp = (sp * 9) / 2 + 1;
			break;
		case 126:
			sp += 1;
			break;
	}
	return ((speed & 0x80) | sp);
}

uint8_t bidib_fmt2code (enum fmt fmt)
{
	switch (fmt) {
		case FMT_MM1_14:
		case FMT_MM2_14:	return BIDIB_CS_DRIVE_FORMAT_MM14;
		case FMT_MM2_27A:	return BIDIB_CS_DRIVE_FORMAT_MM27A;
		case FMT_MM2_27B:	return BIDIB_CS_DRIVE_FORMAT_MM27B;
		case FMT_M3_126:	return BIDIB_CS_DRIVE_FORMAT_M4;
		case FMT_DCC_14:	return BIDIB_CS_DRIVE_FORMAT_DCC14;
		case FMT_DCC_28:	return BIDIB_CS_DRIVE_FORMAT_DCC28;
		case FMT_DCC_126:	return BIDIB_CS_DRIVE_FORMAT_DCC128;
		case FMT_DCC_SDF:	return BIDIB_CS_DRIVE_FORMAT_DCC128P;
		default:			return BIDIB_CS_DRIVE_FORMAT_DCC28;
	}
}

enum fmt bidib_code2fmt (uint8_t code)
{
	switch (code) {
		case BIDIB_CS_DRIVE_FORMAT_DCC14:	return FMT_DCC_14;
		case BIDIB_CS_DRIVE_FORMAT_DCC128P:	return FMT_DCC_SDF;
		case BIDIB_CS_DRIVE_FORMAT_DCC28:	return FMT_DCC_28;
		case BIDIB_CS_DRIVE_FORMAT_DCC128:	return FMT_DCC_126;
		case BIDIB_CS_DRIVE_FORMAT_MM14:	return FMT_MM2_14;
		case BIDIB_CS_DRIVE_FORMAT_MM27A:	return FMT_MM2_27A;
		case BIDIB_CS_DRIVE_FORMAT_MM27B:	return FMT_MM2_27B;
		case BIDIB_CS_DRIVE_FORMAT_M4:		return FMT_M3_126;
		default:							return FMT_UNKNOWN;
	}
}

/**
 * Prepare a string message including the namespace and string index.
 *
 * \param ns		the namespace, currently supported 0: generic strings and 1: DEBUG streams
 * \param id		the string ID in the given namespace
 * \param str		the string to send, may be NULL if string is to be unset or does not exist
 * \return			a message buffer allocated from heap
 * \see				bidib_genMessage()
 */
bidibmsg_t *bidib_string (struct bidibnode *n, uint8_t ns, uint8_t id, char *str)
{
	uint8_t data[32];	// this is sufficient for a max. 24 character string + info

	data[0] = ns;
	data[1] = id;
	if (str) {
		data[2] = strlen(str);
		if (data[2] > 24) data[2] = 24;
		memcpy (&data[3], str, data[2]);
	} else {
		data[2] = 0;
	}
	return bidib_genMessage(n, MSG_STRING, data[2] + 3, data);
}

/* ======================================================================================= */
/* ===== BiDiB feature handling ========================================================== */
/* ======================================================================================= */

/**
 * A local sort function for the feature table of foreign devices
 * for use in the qsort() function call.
 *
 * \param p1	pointer to the first node feature to compare
 * \param p2	pointer to the second node feature to compare
 * \return		an integer telling qsort how to handle these elements
 */
static int bidib_compareFeatures (const void *p1, const void *p2)
{
	struct nodefeature *nf1, *nf2;

	nf1 = (struct nodefeature *) p1;
	nf2 = (struct nodefeature *) p2;
	if (nf1->feature > nf2->feature) return 1;
	if (nf1->feature < nf2->feature) return -1;
	return 0;	// that should not happen
}

/**
 * Sort the feature table of a BiDiB-device (node) using the qsort() algorithm.
 *
 * \param n		the node structure that contains the feature array
 */
void bidib_sortFeature (struct bidibnode *n)
{
	if (n && n->features && n->featurecount > 1) {
		qsort(n->features, n->featurecount, sizeof(*n->features), bidib_compareFeatures);
	}
}

/**
 * Look up a feature in the node and return the address of the feature structure.
 * If that feature could not be found for any reason, return NULL.
 *
 * \param n			the node to search for the feature
 * \param ft		the feature to query
 * \return			pointer to the feature structure or NULL, if lookup failed
 */
struct nodefeature *bidib_readFeature (struct bidibnode *n, uint8_t ft)
{
	int i;

	if (n && n->features) {
		for (i = 0; i < n->featurecount; i++) {
			if (n->features[i].feature == ft) return &n->features[i];
		}
	}
	return NULL;
}

/**
 * Get the current value of a feature in the node as it is known to the system.
 * No actual query is performed, only the internal data structure is searched
 * for the feature.
 *
 * \param n			the node to search for the feature
 * \param ft		the feature to query
 * \return			the value of that feature or zero, if the feature could not be found
 */
uint8_t bidib_getFeatureValue (struct bidibnode *n, uint8_t ft)
{
	struct nodefeature *nf;

	if ((nf = bidib_readFeature(n, ft)) != NULL) {
		return nf->value;
	}
	return 0;
}

/**
 * Write a value to a feature in the node.
 *
 * \param n			the node to search for the feature
 * \param ft		the feature to query
 * \param val		the new value for that feature
 */
void bidib_setFeature (struct bidibnode *n, uint8_t ft, uint8_t val)
{
	bidibmsg_t *msg;
	uint8_t data[2];

	if (n) {
		data[0] = ft;
		data[1] = val;
		msg = bidib_genMessage(n, MSG_FEATURE_SET, 2, data);
		BDBnode_downlink(NULL, msg);
	}
}

/* ======================================================================================= */
/* ===== BiDiB file storage ============================================================== */
/* ======================================================================================= */

static void ini_storeNodes (struct ini_section **root, struct bidibnode *n)
{
	struct ini_section *ini;
	char *tmp = tmp64();
	int i;

	if (!root) return;
	while (n) {
		if (n->flags & NODEFLG_VIRTUAL) {
			sprintf (tmp, "ND%02x%02x%02x%02x%02x%02x%02x", n->uid[0], n->uid[1], n->uid[2], n->uid[3], n->uid[4], n->uid[5], n->uid[6]);
			if ((ini = ini_addSection(root, tmp)) != NULL) {
				ini_addItem(ini, "user", n->user);
				for (i = 0; i < n->featurecount; i++) {
					if (n->features[i].setter) {	// store all settable features
						sprintf (tmp, "FT%u", n->features[i].feature);
						ini_addIntItem(ini, tmp, n->features[i].value);
					}
				}
			}
			if (n->children) ini_storeNodes(root, n->children);
		}
		n = n->next;
	}
}

static void ini_storeFBmap (struct ini_section **root)
{
	struct sysconf *cfg;
	struct ini_section *ini;
	struct bidib_feedback *fb;
	char tmp[16];

	if (!root) return;
	if ((ini = ini_addSection(root, "s88map")) != NULL) {
		cfg = cnf_getconfig();
		fb = cfg->bidibfb;
		while (fb) {
			sprintf (tmp, "%02x%02x%02x%02x%02x%02x%02x", fb->uid[0], fb->uid[1], fb->uid[2], fb->uid[3], fb->uid[4], fb->uid[5], fb->uid[6]);
			ini_addIntItem(ini, tmp, fb->s88base);
			fb = fb->next;
		}
	}
}

static struct ini_section *bidib_generateIni (void)
{
	struct ini_section *ini, *root;
	struct sysconf *cfg;

	cfg = cnf_getconfig();
	root = NULL;
	if ((ini = ini_addSection(&root, "global")) != NULL) {
		ini_addIntItem(ini, "port", cfg->bidib.port);
		ini_addItem(ini, "user", cfg->bidib.user);
	}
	ini_storeNodes(&root, BDBnode_getRoot());
	netBDB_genClientStore(&root);
	ini_storeFBmap(&root);
	return root;
}

static bool bidib_readUIDfromString (uint8_t *uid, char *s)
{
	uint8_t *p;

	if ((p = uid) == NULL) return false;
	while (p < &uid[7]) {	// we always must have 7 encoded bytes
		if (*s >= '0' && *s <= '9') *p = (*s - '0') << 4;
		else if (*s >= 'A' && *s <= 'F') *p = (*s - 'A' + 10) << 4;
		else if (*s >= 'a' && *s <= 'f') *p = (*s - 'a' + 10) << 4;
		else return false;
		s++;
		if (*s >= '0' && *s <= '9') *p |= (*s - '0');
		else if (*s >= 'A' && *s <= 'F') *p |= (*s - 'A' + 10);
		else if (*s >= 'a' && *s <= 'f') *p |= (*s - 'a' + 10);
		else return false;
		s++;
		p++;
	}
	return (*s == 0);		// String must end exactly here!
}

/**
 * Add a s88 mapping for a BiDiB node. To identify the node, only the last
 * 5 bytes of the UID are needed (class and classx bits may change thru a
 * software update). If that node is currently connected and the node has
 * the BIDIB_CLASS_OCCUPANCY bit set in it's class bits, the feedback info
 * is appended in the node's private data pointer.
 *
 * \param uid			the 7 bytes of the node UID, first two bytes are ignored
 * \param s88base		the s88 occupancy address the first bit of the node is mapped to
 */
void bidib_addFBmap (uint8_t *uid, int s88base)
{
	struct sysconf *cfg;
	struct bidibnode *n;
	struct bidib_feedback *bf, **bfpp;
	struct feedback_map *fm;
	int cmp;

	if (!uid || s88base < 0) return;	// ignore senseless calls

	cfg = cnf_getconfig();
	bfpp = &cfg->bidibfb;

	// search for an entry that matches or stop at the position, where a new entry should be inserted
	while ((bf = *bfpp) != NULL && (cmp = memcmp (&(*bfpp)->uid[2], &uid[2], BIDIB_UID_LEN - 2)) < 0) bfpp = &bf->next;

	if (!bf || cmp > 0) {		// we must create a new entry and insert it here into the list
		if ((bf = malloc(sizeof(*bf))) == NULL) return;		// just drop this mapping (what else could be done?)
		memcpy (bf->uid, uid, sizeof(bf->uid));
		bf->next = *bfpp;
		*bfpp = bf;
	}

	bf->s88base = s88base;
	n = BDBnode_lookupNodeByShortUID(&uid[2], NULL);
	if (n && n->uid[0] & BIDIB_CLASS_OCCUPANCY) {	// the node is currently connected and features occupancy functionality ...
		memcpy (bf->uid, n->uid, sizeof(bf->uid));		// we should write the complete real UID to the structure
		if ((fm = n->private) == NULL) fm = malloc(sizeof (*fm));
		if (fm) {		// to be sure that allocation was successfull
			fm->base = s88base;
			n->private = fm;
		}
	}
	log_msg(LOG_INFO, "%s() UID %s s88base = %d\n", __func__, bidib_formatUID(bf->uid), bf->s88base);
}

/**
 * Remove a feedback mapping of a BiDiB node.
 *
 * \param uid			the 7 bytes of the node UID, first two bytes are ignored
 */
void bidib_dropFBmap (uint8_t *uid)
{
	struct sysconf *cfg;
	struct bidibnode *n;
	struct bidib_feedback *bf, **bfpp;

	if (!uid) return;			// ignore senseless calls

	cfg = cnf_getconfig();
	bfpp = &cfg->bidibfb;
	while ((bf = *bfpp) != NULL && memcmp (&(*bfpp)->uid[2], &uid[2], BIDIB_UID_LEN - 2)) bfpp = &bf->next;
	if (bf) {	// we have found an entry that matched - remove if
		*bfpp = bf->next;
		bf->next = NULL;		// paranoia!
		free (bf);
	}

	if ((n = BDBnode_lookupNodeByShortUID(&uid[2], NULL)) != NULL) {	// the node is connected
		if (n->uid[0] & BIDIB_CLASS_OCCUPANCY && n->private) {		// node supports occupancy and already has a mapping structure
			free (n->private);
			n->private = NULL;
		}
	}
}

static void bidib_interpretFBmap (struct ini_section *ini)
{
	struct key_value *kv;
	uint8_t uid[BIDIB_UID_LEN];
	int s88base;

	kv = ini->kv;

	while (kv) {
		bidib_readUIDfromString(uid, kv->key);
		s88base = atoi(kv->value);
		bidib_addFBmap(uid, s88base);
		kv = kv->next;
	}
}

static void bidib_interpretIni (struct ini_section *ini)
{
	uint8_t uid[7];
	char *product, *user, *tmp;
	struct key_value *kv;
	struct sysconf *cfg;
	struct bidibnode *n;
	int i;


	while (ini) {
		if (ini->name[0] == 'C' && ini->name[1] == 'L') {
			if (bidib_readUIDfromString(uid, &ini->name[2])) {
				product = user = NULL;
				if ((kv = kv_lookup(ini->kv, "product")) != NULL) product = kv->value;
				if ((kv = kv_lookup(ini->kv, "user")) != NULL) user = kv->value;
				netBDB_addTrustedClient(uid, product, user);
			}
		} else if (ini->name[0] == 'N' && ini->name[1] == 'D') {
			if (bidib_readUIDfromString(uid, &ini->name[2])) {
				if ((n = BDBnode_lookupNodeByUID(uid, LOCAL_NODE())) != NULL) {
					kv_strcpy(kv_lookup(ini->kv, "user"), n->user, sizeof(n->user));
					for (i = 0; i < n->featurecount; i++) {
						if (n->features[i].setter) {
							tmp = tmp64();
							sprintf (tmp, "FT%u", n->features[i].feature);
							if ((kv = kv_lookup(ini->kv, tmp)) != NULL && kv->value) {
								n->features[i].value = atoi(kv->value);
							}
						}
					}
				}
			}
		} else if (!strcasecmp("global", ini->name)) {
			cfg = cnf_getconfig();
			if ((kv = kv_lookup(ini->kv, "port")) != NULL && kv->value) {
				int port = atoi(kv->value);
				if (port <= 0 || port > UINT16_MAX) {
					log_error("%s(%s): invalid port value %d\n", __func__, kv->key, port);
				} else {
					cfg->bidib.port = port;
				}
			}
			kv_strcpy(kv_lookup(ini->kv, "user"), cfg->bidib.user, sizeof(cfg->bidib.user));
		} else if (!strcasecmp("s88map", ini->name)) {
			bidib_interpretFBmap(ini);
		}
		ini = ini->next;
	}
}

void bidib_store (void)
{
	struct ini_section *ini;

	log_msg (LOG_INFO, "%s() Storing BiDiB information\n", __func__);
	ini = bidib_generateIni();

	ini_writeFile(CONFIG_BIDIB, ini);
	ini_free(ini);
	log_msg (LOG_INFO, "%s() Storage finished\n", __func__);
}

void bidib_load (void)
{
	struct ini_section *ini;

	// TODO clear current setup first???

	if ((ini = ini_readFile(CONFIG_BIDIB)) != NULL) {
		bidib_interpretIni(ini);
		ini_free(ini);
	}
}
