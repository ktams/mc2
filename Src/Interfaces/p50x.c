/**
 * @file p50x.c
 *
 * @author Andi
 * @date   24.05.2020
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
#include <stdarg.h>
#include "rb2.h"
#include "decoder.h"
#include "lwip/sockets.h"
#include "config.h"
#include "events.h"

#define P50X_UDP			0				///< if enabled, the service is also available as UDP (not implemented yet)
#define P50X_STACK			2048			///< allocated stack for the P50X interpreter
#define P50X_PRIO			1				///< priority of the created interpreter thread
#define MAX_CMDLEN			256				///< the maximum length of a command
#define MAX_PARAMS			32				///< maximum number of parameters in a P50Xa command line
#define P50X_MAXFBMODULES	255				///< P50x is limited to 255 modules (#1 - #255, #0 is not a valid module number!)

// operation error codes
#define OK					0x00			///< OK, command accepted and executed
#define XERROR				0x01			///< generic error - currently only for "command not found"
#define XBADPRM				0x02			///< parameter error (parameter is out of allowable range)
#define XPWOFF				0x06			///< command not executed due to status STOP
#define XNOLSPC				0x08			///< no space in command buffer - repeat the command at a later time
#define XNOTSPC				0x09			///< turnout queue is full - repeat command at a later time
#define XNODATA				0x0A			///< no data available
#define XNOSLOT				0x0B			///< refresh queue full, no status available
#define XLKBUSY				0x0D			///< decoder controlled by a different instance - command ignored
#define XBADTNP				0x0E			///< the turnout is not avaiable with current format definition
#define XNOTIMPL			0x3F			///< command is not implemented (yet)
// from here on, the codes are treated as warnings
#define XLKHALT				0x41			///< decoder command executed but sent to track because of "HALT" status
#define XLKPOFF				0x42			///< decoder command accepted but not executed because of "STOP" status
#define NOANSWER			-1				////< do _NOT_ send an Answer (maybe it's already sent!)

// PT error reporting
#define PTERR_OK			0x00			///< command OK
#define PTERR_ERROR			0x01			///< generic error
#define PTERR_YES			0x02			///< answer is YES
#define PTERR_NO			0x03			///< answer is NO
#define PTERR_BUSY			0x04			///< Busy!
#define PTERR_SHORT			0x05			///< Short!
#define PTERR_NODEC			0x06			///< No decoder found
#define PTERR_NOACK			0x07			///< No ACK from decoder
#define PTERR_NOPAGE		0x08			///< No ACK from decoder in page mode
#define PTERR_BITRD			0x09			///< Error in Bit read
#define PTERR_TIMEOUT		0x10			///< Timeout

#define FLAG_IFEXT			0x0001			///< connection is in EXT-mode (i.e. no further 'X' needed)
#define FLAG_S88AUTORESET	0x0002			///< this connection is in s88-autoreset mode
/* flags 0x0004 - 0x0008 reserved for future operational modes */

// event flags to be reported in p50xb_XEvent()/0xC8
#define EVT_PWROFF			0x0010			///< there has been a PWROFF meanwhile
#define EVT_EXTSHORT		0x0020			///< an external booster had a SHORT condition
#define EVT_INTSHORT		0x0040			///< the internal booster had a SHORT condition
#define EVT_OVERHEAT		0x0080			///< overheat has been detected
#define EVT_STATUS			0x0100			///< an XStatus should be issued
// event flags grouping
#define EVT_MASK1			(EVT_PWROFF)
#define EVT_MASK2			(EVT_EXTSHORT | EVT_INTSHORT | EVT_OVERHEAT | EVT_STATUS)
#define EVT_MASK			(EVT_MASK1 | EVT_MASK2)
#define MORE_EVENTS			0x80			///< Bit 7 of a Event-Report byte should be set to announce a followup Event-Report byte

struct parameter {
	char			*text;					///< A pointer in the command line that contains the raw text. This will be a null-terminated string.
	int				 value;					///< The numerical value (if any)
	bool			 numeric;				///< true, if parameter could be interpreted as numerical value
	bool			 supplied;				///< true, if parameter was specified, else false (you can skip parameters by using kommas!)
};

struct loco_change {
	struct loco_change	*next;				///< linked list of change entries
	uint16_t			 adr;				///< address of loco
	uint16_t			 funcs;				///< the funcs from F0 to F8
	enum fmt			 fmt;			///< the decoder format
	uint8_t				 speed;				///< current speed and direction
};

struct trnt_event {
	struct trnt_event	*next;				///< linked list of event entries
	uint16_t			 adr_st;			///< address and status of turnout (fmt as reported via interface)
};

struct connection {
	int					 sock;				///< the active socket we are communication with
	TaskHandle_t		 tid;				///< the task handle (task ID) of the thread that own this structure
	TickType_t			 timeout;			///< a timeout to reset the input buffer to empty
	char				*cmd;				///< the ASCII command inside data - either &data[0] or &data[1] depending on FLAG_IFEXT
	uint8_t		 		 data[MAX_CMDLEN];	///< the command line received
	struct parameter	 param[MAX_PARAMS];	///< the current list of parameters in P50Xa commands
	int					 pcount;			///< the valid number of parameters
	int					 idx;				///< index in data array to store received bytes to
	int					 last_turnout;		///< the last switched turnout (for switching it off)
	int					 flags;				///< various flags
	int					 rc;				///< a possible return code from one of the called functions
	SemaphoreHandle_t	 mutex;				///< a mutex to handle concurrent access to status changes (loco and turnout lists, s88 status, ...)
	struct loco_change	*loco;				///< a linked list of loco change antries
	struct trnt_event	*trnt;				///< a linked list of turnout activities
	uint16_t			 s88Sum[P50X_MAXFBMODULES];					///< summation of s88 status (each s88 module contains 16 bits)
	uint32_t			 s88EvFlag[(P50X_MAXFBMODULES + 31) / 32];	///< a flag for each changed s88 module
};

/*
 * ===============================================================================================
 * Helper functions ==============================================================================
 * ===============================================================================================
 */

/**
 * Check current command for P50X format (either p50xa or p50xb). This is true, if the command
 * begins with an 'x'/'X' or the connection is set to permanently being in X-mode.
 *
 * An empty command will always be treated as not being P50X (i.e. is a P50 command)
 *
 * \param con	pointer to the connection info
 * \return		true, if the command will be interpreted as P50X
 */
static bool is_p50x (struct connection *con)
{
   if (!con || !con->idx) return false;				// no data to check for P50X
   return ((con->flags & FLAG_IFEXT) || (con->data[0] == 'X') || (con->data[0] == 'x'));
}

/**
 * Check is current command is a P50Xa (P50X ASCII) command (in contrast to P50Xb, which
 * denotes a P50X binary command).
 *
 * An empty command is treated as P50Xb, a command that starts with 'x' or 'X' on a connection
 * that is not permanently switched to P50X is treated as P50Xa command to suppress the timeout
 * and thereby enabling the manual input via telnet session.
 *
 * Commands that have at least one character of the sequence (i.e. at least one character for
 * connections that are permanently set to P50X or other connections that have received more
 * than the leading 'x'/'X') are judged according to that character. If it has the MSB set
 * (i.e. >= 0x80) it is a binary command.
 *
 * \param con	pointer to the connection info
 * \return		true, if the command will be interpreted in P50Xa mode, false otherwise
 */
static bool is_p50xa (struct connection *con)
{
   if (!con || !is_p50x(con)) return false;		// this is no P50X command at all

   // if we get here it's guarantied that a minimum of one character is read!
   con->cmd = NULL;
   if (!(con->flags & FLAG_IFEXT) && con->idx < 2) return true;		// there is no data yet for the check - assume P50Xa!
   if (con->flags & FLAG_IFEXT) con->cmd = (char *) &con->data[0];	// let's check the first command byte
   else con->cmd = (char *) &con->data[1];
   if (*con->cmd >= 0x80) con->cmd = NULL;		// if the command is not ASCII, we don't want the 'cmd' member to point to the data array
   return (con->cmd != NULL);
}

static const uint8_t speed14[] = {
   0, 2, 12, 21, 29, 38, 48, 57, 67, 76, 86, 95, 105, 114, 127
};
static const uint8_t speed27[] = {
   0, 2, 7, 12, 16, 21, 26, 30, 35, 40, 44, 49, 54, 58, 63, 68,
   72, 77, 82, 86, 91, 96, 100, 105, 110, 115, 121, 127
};
static const uint8_t speed28[] = {
   0, 2, 7, 11, 16, 20, 25, 29, 34, 38, 43, 48, 53, 57, 63, 67,
   72, 76, 81, 86, 91, 95, 100, 105, 109, 114, 118, 123, 127
};

/**
 * Translate the internal loco speed to interface speed 0 .. 127 depending on
 * format capabilities. The emergency stop speed step '1' is always skipped.
 *
 * \param speed		the speed in decoder coding (format dependent, direction bit ignored)
 * \param format	the decoder format to decode the number of supported speed steps (14, 27, 28 or 126)
 * \return			the speed in interface coding (0 - 127, no direction bit supplied)
 */
static int p50x_speed2if (int speed, enum fmt format)
{
	speed &= 0x7F;			// mask out direction bit
	if (!speed) return 0;	// speed zero stays zero :-)

	switch (format) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_DCC_14:
			if (speed > 14) return 0;
			return speed14[speed];
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			if (speed > 27) return 0;
			return speed27[speed];
		case FMT_DCC_28:
			if (speed > 28) return 0;
			return speed28[speed];
		default:				// all 126 speed step formats (FMT_DCC_126, FMT_M3_126)
			return speed + 1;
	}
}

/*
 * Translate interface speed 0 .. 127 to real speed depending on format capabilities.
 * The emergency stop speed step '1' must already have taken into account (it will
 * give a speed of zero though).
 *
 * \param speed		the speed in interface coding (0 - 127, direction bit ignored)
 * \param format	the decoder format to decode the number of supported speed steps (14, 27, 28 or 126)
 * \return			the speed in decoder coding (fmt dependent, no direction bit supplied)
 */
static int p50x_if2speed (int speed, enum fmt format)
{
	speed &= 0x7F;			// mask out direction bit
	if (!speed) return 0;	// speed zero stays zero :-)
	speed--;				// speed == 1 is emergency stop - skip it

	switch (format) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_DCC_14:
			return (speed + 8) / 9;
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			return (speed * 3 + 11) / 14;
		case FMT_DCC_28:
			return (speed * 2 + 7) / 9;
		default:				// all 126 speed step formats (FMT_DCC_126, FMT_M3_126)
			return speed;
	}
}

/*
 * Translate the P50 interface speed (0 .. 14) to the decoder speed.
 * The direction reversal speed coding 0x0F is translated to a speed of 0.
 *
 * \param speed		the speed in p50 coding (0 - 14, no direction bit)
 * \param format	the decoder format to decode the number of supported speed steps (14, 27, 28 or 126)
 * \return			the speed in decoder coding (fmt dependent, no direction bit supplied)
 */
static int p50x_p50speed (int speed, enum fmt format)
{
	speed &= 0x0F;			// mask out any excessive bits (P50 only supports 14 speeds)
	if (!speed) return 0;	// speed zero stays zero :-)
	if (speed == 0x0F) return 0;	// this is the direction reversal code

	switch (format) {
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			return speed * 2 - 1;
		case FMT_DCC_28:
			return speed * 2;
		case FMT_M3_126:
		case FMT_DCC_126:
		case FMT_DCC_SDF:
			return speed * 9;
		default:				// the default (direct) mapping of 14 speed codes is just 1:1
			return speed;
	}
}

/**
 * Set an SO (Special Option).
 * This came from the IB and was a little bit supported by the MasterControl.
 *
 * \param adr	the SO address (1 to 999) that is to be set
 * \param val	the new value for the addressed SO in the range 0..255
 * \return		0 for success or -1 if SO is not supported
 */
static int p50x_SOset (int adr, uint8_t val)
{
	(void) val;

	switch (adr) {
		case 999:
			// This is not yet supported, see below
			break;
		default:
			return -1;		// this SO is not supported
	}

	// If any changes had been made, we should save the new status.
	// Most of the time, the change itself will trigger a storage command, though.
	return 0;
}

/**
 * Query an SO (Special Option).
 * This came from the IB and was a little bit supported by the MasterControl.
 *
 * \param adr	the SO address (1 to 999) that is queried
 * \return		the conents of this option in the range of 0..255 or -1 if the requested SO is not supported
 */
static int p50x_SOget (int adr)
{
	int c;

	switch (adr) {
		case 1:
			return 5;		// always report 57600 Baud (this is a fake)
		case 999:
			c = 0;
			/*
			 * @TODO We should show the distribution of specific signal parts on he booster outputs as bitmap.
			 * Currently defined for MasterControl:
			 *  - bit #0: no magnet commands on normal track booster
			 *  - bit #1: no magnet commands on break-booster
			 */
			return c;
			break;
	}
	return -1;		// SO not implemented
}

/**
 * Handle events coming in. The events that stem from out own activity are
 * ignored (just like in other event handlers).
 *
 * \param e		the event structure that describes the nature and details of the event
 * \param priv	the private information supplied at event registration (in this case it is the connection structure)
 * \return		always true to continue to receive events
 */
static bool p50x_eventhandler (eventT *e, void *priv)
{
	struct connection *con;
	struct loco_change *lc;
	struct trnt_event *te;
	struct s88_status *s88;
	fbeventT *fbevt;
	ldataT *l;
	turnoutT *t;
	int i, s88modCount;

	con = (struct connection *) priv;
	if (e->tid == con->tid) return true;	// this is an event we triggered ourself, so don't report back!

	if (mutex_lock(&con->mutex, 20, __func__)) {		// if we cannot aquire the mutex rapidly, we give up and ignore the reported event
		switch (e->ev) {
			case EVENT_SYS_STATUS:
				if (rt.tm == TM_STOP || rt.tm == TM_SHORT) con->flags |= EVT_PWROFF;
				con->flags |= EVT_STATUS;
				break;
			case EVENT_LOCO_SPEED:
			case EVENT_LOCO_FUNCTION:
				l = (ldataT *) e->src;
				if ((lc = malloc (sizeof(*lc))) == NULL) break;
				lc->next = NULL;
				lc->adr = l->loco->adr;
				lc->fmt = l->loco->fmt;
				lc->funcs = l->funcs[0] & (FUNC_LIGHT | FUNC_F1_F8);
				lc->speed = l->speed;
				if ((lc->speed & 0x7F) > 0) lc->speed++;		// skip emergency stop code
				list_append(&con->loco, lc);
				break;
			case EVENT_TURNOUT:
				t = (turnoutT *) e->src;
				if ((te = malloc (sizeof(*te))) == NULL) break;
				printf ("%s() EVENT_TURNOUT adr %d\n", __func__, t->adr);
				te->next = NULL;
				te->adr_st = t->adr & 0x7FF;
				if (!t->dir) te->adr_st |= 0x8000;	// highest bit reports direction: 1: straight / 0: thrown (inverted from our internal sense)
				if (t->on) te->adr_st |= 0x4000;	// bit #14 (bit #6 of high byte) denotes the "powered" state
				list_append(&con->trnt, te);
				break;
			case EVENT_FEEDBACK:
				s88 = (struct s88_status *) e->src;
				s88modCount = s88->modcnt;
				if (s88modCount > P50X_MAXFBMODULES) s88modCount = P50X_MAXFBMODULES;
				for (i = 0; i < s88modCount; i++) {
					con->s88Sum[i] |= s88->sum[i];
				}
				for (i = 0; i < ((s88modCount + 31) / 32); i++) {
					con->s88EvFlag[i] |= s88->evFlag[i];
				}
				break;
			case EVENT_FBNEW:
				fbevt = (fbeventT *) e->src;
				if (fbevt->module >= 0 && fbevt->module < P50X_MAXFBMODULES) {
					con->s88Sum[fbevt->module] |= fbevt->status;
					bs_set(con->s88EvFlag, fbevt->module);
				}
				break;
			default:
				printf ("%s(): event %d\n", __func__, e->ev);
				break;
		}
		mutex_unlock(&con->mutex);
	}
	return true;
}

/*
 * ===============================================================================================
 * the P50X binary commands ======================================================================
 * ===============================================================================================
 */

// forward declaration to call p50 commands from P50Xb-mode via p50xb_XP50Len1() and p50xb_XP50Len2()
static int p50_interpret (struct connection *con, uint8_t *data, int len);

static void p50xb_sendbuf (int sock, uint8_t *data, int len)
{
	lwip_send (sock, data, len, 0);
}

static void p50xb_error (int sock, int errcode)
{
	uint8_t err = errcode & 0xFF;

	if (errcode != NOANSWER) p50xb_sendbuf(sock, &err, 1);
}

static ldataT *p50xb_getloco (int sock, uint8_t *cmd, bool add)
{
   ldataT *l;
   unsigned short adr;

   adr = cmd[1] | (cmd[2] << 8);
   printf ("%s(): ADR %d\n", __func__, adr);
   if (adr == 0 || adr > MAX_LOCO_ADR) {
      p50xb_error(sock, XBADPRM);
      return NULL;
   }

   l = loco_call(adr, add);

   if (!l) {
      if (add) p50xb_error(sock, XNOSLOT);
      else p50xb_error(sock, XNODATA);
      return NULL;
   }

   return l;
}

static int _XLok (int sock, uint8_t *cmd, bool ifspeed)
{
	ldataT *l;
	uint32_t newfuncs;
	uint8_t speed;

	if ((l = p50xb_getloco (sock, cmd, true)) == NULL) return NOANSWER;

	speed = cmd[3];
	if (ifspeed && speed == 1) {	// emergency stop
		speed = 0;
		loco_emergencyStop(l->loco->adr);
	} else {
		if (ifspeed) speed = p50x_if2speed (cmd[3], l->loco->fmt);
	}
	if (cmd[4] & 0x20) speed |= 0x80;	// merge direction bit

	// if direction changes, send an intermediate STOP-Speed to circumvent emergency stop
	if ((speed & 0x80) != (l->speed & 0x80)) rq_setSpeed(l->loco->adr, l->speed & 0x80);
	rq_setSpeed (l->loco->adr, speed);

	newfuncs = ((cmd[4] & 0x0F) << 1) | ((cmd[4] & 0x10) >> 4);
	if (cmd[4] & 0x80) {		// ChgF-Bit is set -> replace F0 and F4..F1
		rq_setFuncMasked(l->loco->adr, newfuncs, FUNC_F0_F4);
	} else {					// ChgF-Bit is clear -> change only F0
		rq_setFuncMasked(l->loco->adr, newfuncs, FUNC_LIGHT);
	}

	if (rt.tm == TM_GO || rt.tm == TM_TAMSPROG) return OK;
	if (rt.tm == TM_HALT) return XLKHALT;
	return XLKPOFF;
}

static int p50xb_XLok (struct connection *con)
{
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	return _XLok (con->sock, cmd, true);
}

static int p50xb_XLokX (struct connection *con)
{
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	return _XLok (con->sock, cmd, false);
}

static int p50xb_XLokSts (struct connection *con)
{
	ldataT *l;
	uint32_t funcs;
	uint8_t buf[16], *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;

	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	buf[0] = OK;
	buf[1] = p50x_speed2if (l->speed, l->loco->fmt);	// interface speed
	funcs = (l->funcs[0] >> 1) & 0x0F;	// F4..F1
	if (l->funcs[0] & 1) funcs |= 0x10;
	if (l->speed & 0x80) funcs |= 0x20;
	buf[2] = funcs & 0xFF;					// funcs and direction
	buf[3] = l->speed & 0x7F;				// the real speed

	p50xb_sendbuf(con->sock, buf, 4);
	return NOANSWER;
}

static int p50xb_XLokCfg (struct connection *con)
{
	ldataT *l;
	uint8_t buf[16], *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	buf[0] = OK;
	if (FMT_IS_MM1(l->loco->fmt)) buf[1] = 1;
	else if (FMT_IS_MM2(l->loco->fmt)) buf[1] = 2;
	else if (FMT_IS_DCC(l->loco->fmt)) buf[1] = 3;
	else buf[1] = 0;		// M3

	switch (l->loco->fmt) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_DCC_14:
			buf[2] = 14;
			break;
		case FMT_MM2_27A:
			buf[2] = 27;
			break;
		case FMT_MM2_27B:
		case FMT_DCC_28:
			buf[2] = 28;
			break;
		case FMT_DCC_126:
		case FMT_DCC_SDF:
		case FMT_M3_126:
			buf[2] = 126;
			break;
		default:		// what can we do here
			break;
	}
	buf[3] = 0xFF;		// virtual loco (not supported)
	buf[4] = 0xFF;		// virtual loco (not supported)

	p50xb_sendbuf(con->sock, buf, 5);
	return NOANSWER;
}

static int p50xb_XLokCfgSet (struct connection *con)
{
	ldataT *l;
	enum fmt format;
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	switch (cmd[3]) {
		case 0:					// M3 formats (only option is FMT_M3_126)
			if (cmd[4] != 126) return XBADPRM;
			format = FMT_M3_126;
			break;
		case 1:					// MM1 format (14 speeds only)
			switch (cmd[4]) {
				case 14:
					format = FMT_MM1_14;
					break;
				default:
					return XBADPRM;
			}
			break;
		case 2:					// MM2 formats (14, 27A or 27B speeds)
			switch (cmd[4]) {
				case 14:
					format = FMT_MM2_14;
					break;
				case 27:
					format = FMT_MM2_27A;
					break;
				case 28:
					format = FMT_MM2_27B;
					break;
				default:
					return XBADPRM;
			}
			break;
		case 3:					// DCC formats (14, 28 or 126 speeds)
			switch (cmd[4]) {
				case 14:
					format = FMT_DCC_14;
					break;
				case 28:
					format = FMT_DCC_28;
					break;
				case 126:
					format = FMT_DCC_126;
					break;
				default:
					return XBADPRM;
			}
			break;
		default:
			return XBADPRM;
	}

	db_setLocoFmt(l->loco->adr, format);
	return OK;
}

static int p50xb_Xm3Sid (struct connection *con)
{
	ldataT *l;
	uint32_t mac;
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if (rt.tm != TM_GO) return XLKPOFF;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;
	db_setLocoFmt(l->loco->adr, FMT_M3_126);
	db_setLocoMaxfunc(l->loco->adr, 31);
	memset (l->funcs, 0, sizeof(l->funcs));
	l->speed = 0;

	mac = ((uint32_t) cmd[3] << 0) | ((uint32_t) cmd[4] << 8) | ((uint32_t) cmd[5] << 16) | ((uint32_t) cmd[6] << 24);
	m3_setAddress(mac, l->loco->adr);

	return OK;
}

static int p50xb_XFunc (struct connection *con)
{
	ldataT *l;
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	rq_setFuncMasked(l->loco->adr, cmd[3] << 1, FUNC_F1_F8);

	return OK;
}

static int p50xb_XFuncX (struct connection *con)
{
	ldataT *l;
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	rq_setFuncMasked(l->loco->adr, cmd[3] << 9, FUNC_F9_F16);

	return OK;
}

static int p50xb_XFunc34 (struct connection *con)
{
	ldataT *l;
	uint32_t funcs;
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	// cmd[3] contains F17..F24
	// cmd[4] contains F25..F31 (a hypothetical F32 could be included but is not defined so far)
	funcs = (cmd[3] << 17) | (cmd[4] << 25);
	rq_setFuncMasked(l->loco->adr, funcs, FUNC_F17_F31);

	return OK;
}

static int p50xb_XBinSt (struct connection *con)
{
	ldataT *l;
	uint8_t *cmd;
	int flag_adr;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;
	if (!FMT_IS_DCC(l->loco->fmt)) return XBADPRM;
	flag_adr = (cmd[3] & 0x7F) | (cmd[4] << 7);

	loco_setBinState(l->loco->adr, flag_adr, (cmd[3] & 0x80) ? ON : OFF);

	return OK;
}

static int p50xb_XFuncSts (struct connection *con)
{
	ldataT *l;
	uint8_t *cmd;
	uint8_t buf[16];

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	buf[0] = OK;
	buf[1] = (l->funcs[0] >> 1) & 0xFF;
	p50xb_sendbuf(con->sock, buf, 2);

	return NOANSWER;
}

static int p50xb_XFuncXSts (struct connection *con)
{
	ldataT *l;
	uint8_t buf[16], *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	buf[0] = OK;
	buf[1] = (l->funcs[0] >> 9) & 0xFF;
	p50xb_sendbuf(con->sock, buf, 2);

	return NOANSWER;
}

static int p50xb_XFunc34Sts (struct connection *con)
{
	ldataT *l;
	uint8_t buf[16], *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	if ((l = p50xb_getloco (con->sock, cmd, true)) == NULL) return NOANSWER;

	buf[0] = OK;
	buf[1] = (l->funcs[0] >> 17) & 0xFF;
	buf[2] = (l->funcs[0] >> 25) & 0x7F;
	p50xb_sendbuf(con->sock, buf, 3);

	return NOANSWER;
}

static int p50xb_XTrnt (struct connection *con)
{
	int adr;
	uint8_t *cmd;
	bool thrown, on;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	adr = cmd[1] | ((cmd[2] & 0x07) << 8);
	if (adr >= MAX_TURNOUT) return XBADPRM;
	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TAMSPROG) return XPWOFF;
	thrown = !(cmd[2] & 0x80);
	on = !!(cmd[2] & 0x40);

//	if (con->last_turnout != NO_TURNOUT_EVENT) {
//		mag_switch (con->last_turnout, FALSE, chan);
//		con->last_turnout = NO_TURNOUT_EVENT;
//	}
	if (trnt_switch(adr, thrown, on)) return XNOTSPC;
	if (on) con->last_turnout = adr;
	return OK;
}

static int p50xb_XTrntX (struct connection *con)
{
	int adr;
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	adr = cmd[1] | ((cmd[2] & 0x07) << 8);
	if (adr >= MAX_DCC_EXTACC) return XBADPRM;
	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TAMSPROG) return XPWOFF;

	xacc_aspect (adr, cmd[2] >> 3);
	return OK;
}

static int p50xb_XTrntSts (struct connection *con)
{
	int adr;
	turnoutT *t;
	uint8_t buf[16], *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	adr = cmd[1] | ((cmd[2] & 0x07) << 8);
	if ((t = db_getTurnout(adr)) == NULL) return XBADPRM;

	buf[0] = OK;
	buf[1] = (!t->dir) ? 0x04 : 0x00;
	if (t->fmt == TFMT_DCC) buf[1] |= 0x01;
	p50xb_sendbuf(con->sock, buf, 2);

	return NOANSWER;
}

static int p50xb_XTrntGrp (struct connection *con)
{
	int grp, n, begin, end;
	turnoutT *t;
	uint8_t buf[16], *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	grp = cmd[1];

	begin = (grp - 1) * 8 + 1;
	end = begin + 8;
	buf[0] = OK;
	buf[1] = buf[2] = 0;
	for (n = begin; n < end; n++) {		// direction of first turnout in group is reported in bit #0
		buf[1] >>= 1;
		if ((t = db_getTurnout(n)) != NULL) {
			if (!t->dir) buf[1] |= 0x80;
		}
	}
	p50xb_sendbuf(con->sock, buf, 3);

	return NOANSWER;
}

/**
 * Return the current state of the requested s88 module.
 * This will report the "unbuffered" status of this module.
 * No event flags are needed or modified.
 *
 * \param con		the current connection structure
 * \return			a code for the caller to reply with on the connection
 */
static int p50xb_XSensor (struct connection *con)
{
	uint8_t buf[16], *cmd;
	uint16_t mstat;
	int module;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	module = cmd[1] - 1;
//	if (module < 0 || module >= MAX_FBMODULES) return XBADPRM;		// module number out-of-range / 01.10.2023 A.Kre: cannot happen!
	buf[0] = OK;
#ifdef CENTRAL_FEEDBACK
	mstat = fb_getModuleState(module);
#else
	mstat = s88_getInput(module);
#endif
	buf[1] = (mstat >> 8) & 0x0FF;
	buf[2] = (mstat >> 0) & 0xFF;
	p50xb_sendbuf(con->sock, buf, 3);
	bs_clear(con->s88EvFlag, module);

	return NOANSWER;
}

/**
 * Reset the event status of all s88 modules and re-check if any
 * s88 bits are set. This will transfer the current status of the
 * input bits to the local, connection individual copy of s88 bits
 * and set the event bit for every module that contains at least
 * one set bit.
 *
 * PC software can use this call on startup to efficiently query
 * the current s88 status by checking the s88 events after this call.
 *
 * \param con		the current connection structure
 * \return			a code for the caller to reply with on the connection
 */
static int p50xb_XSensOff (struct connection *con)
{
	int i;

	memset (con->s88EvFlag, 0, sizeof(con->s88EvFlag));

	for (i = 0; i < s88_getModules(); i++) {
#ifdef CENTRAL_FEEDBACK
		con->s88Sum[i] = fb_getModuleState(i);
#else
		con->s88Sum[i] = s88_getInput(i);
#endif
		if (con->s88Sum[i]) {
			bs_set(con->s88EvFlag, i);
		}
	}

	return OK;
}

/**
 * Query the configuration of the s88 subsystem.
 *
 * \param con		the current connection structure
 * \return			a code for the caller to reply with on the connection
 */
static int p50xb_X88PGet (struct connection *con)
{
	uint8_t buf[16], *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;

	buf[0] = OK;

	switch (cmd[1]) {
		case 0:		// number of s88 'half' modules
			buf[1] = s88_getModules() * 2;
			break;
		case 3:		// status of s88 autoreset flag
			buf[1] = (con->flags & FLAG_S88AUTORESET) ? 1 : 0;
			break;
		default:
			return XBADPRM;
	}
	p50xb_sendbuf(con->sock, buf, 2);

	return NOANSWER;
}

/**
 * Configure the s88 subsystem.
 *
 * \param con		the current connection structure
 * \return			a code for the caller to reply with on the connection
 */
static int p50xb_X88PSet (struct connection *con)
{
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;

	switch (cmd[1]) {
		case 0:
#if (MAX_FBMODULES < 127)
			if (cmd[2] > MAX_FBMODULES * 2) return XBADPRM;
#endif
//			s88_setModules((cmd[2] + 1) >> 1);
			break;
		case 3:
			if (cmd[2]) con->flags |= FLAG_S88AUTORESET;
			else con->flags &= ~FLAG_S88AUTORESET;
			break;
		default:
			return XBADPRM;
	}

	return OK;
}

#define SERIAL_AS_BCD
static int p50xb_XVer (struct connection *con)
{
	uint8_t buf[16];
#ifdef SERIAL_AS_BCD
	char snr[16], *s;
	uint8_t *p;
#endif

	buf[0] = 3;										// three bytes version identifier
	buf[1] = SOFT_VERSION_MAJOR;
	buf[2] = SOFT_VERSION_MINOR;
	buf[3] = SOFT_VERSION_SUB;
#ifdef SERIAL_AS_BCD
	buf[4] = 4;
	sprintf (snr, "%08d", hwinfo->serial);
	s = snr;
	p = &buf[5];
	while (*s) {
		*p = (*s++ - '0') << 4;
		*p |= (*s++ - '0');
		p++;
	}
	p50xb_sendbuf(con->sock, buf, 9);
#else
	buf[4] = 3;										// three bytes serial number
	buf[5] = (hwinfo->serial >> 16) & 0xFF;			// MSB of serial #
	buf[6] = (hwinfo->serial >> 8) & 0xFF;			// middle of serial #
	buf[7] = (hwinfo->serial >> 0) & 0xFF;			// LSB of serial #
	p50xb_sendbuf(con->sock, buf, 8);
#endif

	return OK;		// terminates the answer with a 0x00
}

static int p50xb_XStatus (struct connection *con)
{
	uint8_t c;

	c = 0;
	if (rt.tm == TM_GO || rt.tm == TM_TAMSPROG || rt.tm == TM_HALT) c |= 0x08;
	if (rt.tm == TM_HALT) c |= 0x10;
	p50xb_sendbuf(con->sock, &c, 1);

	return NOANSWER;
}

static int p50xb_XSoSet (struct connection *con)
{
	uint8_t *cmd;
	int so;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	so = cmd[1] | (cmd[2] << 8);
	if (p50x_SOset(so, cmd[3])) return XBADPRM;

	return OK;
}

static int p50xb_XSoGet (struct connection *con)
{
	uint8_t buf[16], *cmd;
	int so, rc;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	so = cmd[1] | (cmd[2] << 8);

	rc = p50x_SOget(so);
	if (rc < 0) return XBADPRM;

	buf[0] = OK;
	buf[1] = rc & 0xFF;
	p50xb_sendbuf(con->sock, buf, 2);

	return NOANSWER;
}

static int p50xb_XHalt (struct connection *con)
{
	(void) con;

	sig_setMode(TM_HALT);
	return OK;
}

static int p50xb_XPwrOff (struct connection *con)
{
	(void) con;

	sig_setMode(TM_STOP);
	return OK;
}

static int p50xb_XPwrOn (struct connection *con)
{
	(void) con;

	sig_setMode(TM_GO);
	return OK;
}

static int p50xb_XNop (struct connection *con)
{
	(void) con;

	return OK;
}

static int p50xb_XP50Len1 (struct connection *con)
{
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	cmd++;		// skip P50Xb command identifier 0xC6
	p50_interpret(con, cmd, 1);

	return NOANSWER;
}

static int p50xb_XP50Len2 (struct connection *con)
{
	uint8_t *cmd;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;
	cmd++;		// skip P50Xb command identifier 0xC7
	p50_interpret(con, cmd, 2);

	return NOANSWER;
}

/**
 * Query the events that are active for the current connection.
 * Events that only consist of a single flag are cleared as they are
 * reported. Other events stay active as long as the corresponding
 * information remains unchanged.
 *
 * \param con		the current connection structure
 * \return			a code for the caller to reply with on the connection
 */
static int p50xb_XEvent (struct connection *con)
{
	uint8_t buf[16];
	int len = 0;

	mutex_lock(&con->mutex, 100, __func__);		// this should work! If the mutex cannot be aquired, we simply ignore it
	buf[len] = 0;		// start with "no event to report"
	if (con->loco) buf[len] |= 0x01;
	if (!bs_isempty(con->s88EvFlag, MAX_FBMODULES)) buf[len] |= 0x04;
	if (con->flags & EVT_PWROFF) buf[len] |= 0x08;
	if (con->trnt) buf[len] |= 0x20;
	con->flags &= ~EVT_MASK1;
	len++;
	if (con->flags & EVT_MASK) {
		buf[len - 1] |= MORE_EVENTS;	// more events to report
		buf[len] = 0;		// start with "no event to report"
		if (con->flags & EVT_EXTSHORT) buf[len] |= 0x01;
		if (con->flags & EVT_INTSHORT) buf[len] |= 0x04;
		if (con->flags & EVT_OVERHEAT) buf[len] |= 0x20;
		if (con->flags & EVT_STATUS)   buf[len] |= 0x40;
		con->flags &= ~EVT_MASK2;
		len++;
	}
	/* --- currently we have no other events to report --- */
//	if (con->flags & EVT_MASK) {
//		buf[len - 1] |= MORE_EVENTS;	// more events to report
//		buf[len] = 0;		// start with "no event to report"
//		if (con->flags & EVT_xxx) buf[len] |= 0x01;
//		con->flags &= ~EVT_MASK3;
//	}

	mutex_unlock(&con->mutex);
	p50xb_sendbuf(con->sock, buf, len);
	printf ("%s(): %d bytes sent\n", __func__, len);

	return NOANSWER;
}

static int p50xb_XEvtLok (struct connection *con)
{
	uint8_t buf[5 * 64];	// space for 64 locos to report in a single packet
	uint8_t *p;
	struct loco_change *lc;

	mutex_lock(&con->mutex, 100, __func__);		// this should work! If the mutex cannot be aquired, we simply ignore it
	p = buf;
	while ((lc = con->loco) != NULL) {
		con->loco = lc->next;
		p[0] = p50x_speed2if(lc->speed, lc->fmt);
		p[1] = lc->funcs >> 1;
		p[2] = lc->adr & 0xFF;
		p[3] = (lc->adr >> 8) & 0x3F;
		if (lc->funcs & FUNC_LIGHT) p[3] |= 0x40;	// copy the function (F0 / LIGHT) status to bit#6
		if (lc->speed & 0x80) p[3] |= 0x80;			// copy the direction bit of the speed to bit#7
		p[4] = lc->speed & 0x7F;					// the real decoder speed
		p += 5;
		if ((unsigned) (p - buf) >= sizeof(buf)) {
			p50xb_sendbuf(con->sock, buf, p - buf);
			p = buf;
		}
		free (lc);
	}
	if ((p - buf) > 0) {
		p50xb_sendbuf(con->sock, buf, p - buf);	// send a partial buffer
		printf ("%s(): %d bytes sent\n", __func__, p - buf);
	}
	mutex_unlock(&con->mutex);

	return 0x80;		// no more locos to report (this is not an error code!)
}

static int p50xb_XEvtTrn (struct connection *con)
{
	uint8_t buf[256], *p;
	struct trnt_event *te;
	int i, events;

	p = buf;
	if (!mutex_lock(&con->mutex, 20, __func__)) {		// if we cannot aquire the mutex, we report an empty turnout event list and don't clear the event status bit
		*p++ = 0;
	} else {
		events = list_len(con->trnt);
		if (events > (int) ((sizeof(buf) - 1) / 2)) events = (sizeof(buf) - 1) / 2;
		*p++ = events;
		for (i = 0; i < events; i++) {
			te = con->trnt;
			con->trnt = te->next;
			*p++ = te->adr_st & 0xFF;
			*p++ = (te->adr_st >> 8) & 0xFF;
			free (te);
		}
		mutex_unlock(&con->mutex);
	}
	p50xb_sendbuf(con->sock, buf, p - buf);

	return NOANSWER;
}

/**
 * Report all "changed" s88 module bits. This generates a list with
 * the s88 module number (one-based here) and the two bytes of the
 * current state. A null byte as module number terminates the list.
 *
 * The reports are requested by the host application and MUST NOT miss
 * any s88 ACTIVATION (SET bit). It also SHOULD report cleared bits if
 * no activation is to be reported anymore. This leads to a logic, where
 * no short SET-impulses can be missed, but short CLEAR-impulses may be
 * overlooked, which should not lead to problems. When all active bits
 * are reported and the status is now CLEARED, this should be reported
 * on the next event-poll.
 *
 * The report is based on the s88 event markers (one bit per module).
 * The idea is as follows:
 *   1.	The s88 thread sends an event every time a changed status is seen on any s88 module
 *   2.	Our event handler sums up the event flags and s88 bits (using OR)
 *   3.	For every set event bit, the local summation of s88 data bits plus the actual status as seen by
 *		the s88 thread is reported. This thus may include set bits, that didn't reach us by an event,
 *		but will also help us on next host polling to report cleared bits. See also next items, which
 *		explain clearing the local summation.
 *   4.	The individual module event flag only gets cleared, if the reported bit-summation from [3] is
 *		equal to the current state from the s88 thread. If this is not the case, there are probably
 *		cleared bits, that were not reported.
 *   5.	The local summation is cleared to zero and will be filled up on the next event triggered by the
 *		s88 thread.
 *   6.	Even if no new event fires, but the p50x event bit is still set and the local summation stays at
 *		zero, we will report the state of meanwhile cleared s88 bits on the next event-poll from the
 *		host because the report in [3] adds the current status to the cleared local summation.
 *
 * \param con		the current connection structure
 * \return			a code for the caller to reply with on the connection
 */
static int p50xb_XEvtSen (struct connection *con)
{
	uint8_t buf[3 * MAX_FBMODULES + 1];	// space for the maximum length result (incl. the terminating 0x00 denoting end-of-list)
	uint8_t *p;
	uint16_t s88;
	int i, modules;

	mutex_lock(&con->mutex, 100, __func__);		// this should work! If the mutex cannot be aquired, we simply ignore it
	modules = s88_getModules();
	for (i = 0, p = buf; i < modules; i++) {
		if (bs_isset(con->s88EvFlag, i)) {
			printf ("%s(): events in module %d\n", __func__, i);
#ifdef CENTRAL_FEEDBACK
			s88 = con->s88Sum[i] | fb_getModuleState(i);	// add any new bits arrived since last update
			if (s88 == fb_getHalfModuleState(i)) bs_clear(con->s88EvFlag, i);
#else
			s88 = con->s88Sum[i] | s88_getInput(i);			// add any new bits arrived since last update
			if (s88 == s88_getInput(i)) bs_clear(con->s88EvFlag, i);
#endif
			con->s88Sum[i] = 0;
			p[0] = i + 1;		// in interface, modules are 1-based
			p[1] = (s88 >> 8) & 0xFF;
			p[2] = (s88 >> 0) & 0xFF;
			p += 3;
		}
	}
	*p++ = 0;		// End-of-List marker
	p50xb_sendbuf(con->sock, buf, p - buf);
	printf ("%s(): %d bytes sent\n", __func__, p - buf);
	mutex_unlock(&con->mutex);

	return NOANSWER;
}

static int p50xb_XEvtPT (struct connection *con)
{
	uint8_t c;

	// currently this function is only a dummy (as it was in MasterControl ...)
	c = OK;
	p50xb_sendbuf(con->sock, &c, 1);

	return NOANSWER;
}

static int p50xb_XDCC_PAX (struct connection *con)
{
	uint8_t *cmd;
	int adr, cv;

	cmd = con->data;
	if (!(con->flags & FLAG_IFEXT)) cmd++;

	adr = (cmd[1] | ((cmd[2] & 0x07) << 8));
	cv = (cmd[3] | ((cmd[4] & 0x07) << 8)) - 1;
	if (adr >= MAX_TURNOUT) return XBADPRM;
	if (cv > MAX_DCC_CVADR) return XBADPRM;

	if (cmd[0] == 0xD9) {	// 0xD9: XDCC_PARX ('R' for "READ")
		dccpom_readByte(adr, DECODER_DCC_EXT, cv, NULL, fvNULL);
	} else {				// 0xD8: XDCC_PAX
		dccpom_writeByte(adr, DECODER_DCC_EXT, cv, cmd[5], NULL, fvNULL);
	}

	return OK;
}

static const struct p50xbcmd {		///< command table for P50Xa commands
   uint8_t		 cmd;						///< the command byte
   uint8_t		 len;						///< the amount of bytes needed to complete this command
   int (*func)(struct connection *con);		///< the action function
} p50xbcmds[] = {
	{ 0x80, 5, p50xb_XLok },
	{ 0x81, 5, p50xb_XLokX },
	{ 0x84, 3, p50xb_XLokSts },
	{ 0x85, 3, p50xb_XLokCfg },
	{ 0x86, 5, p50xb_XLokCfgSet },
	{ 0x87, 7, p50xb_Xm3Sid },
	{ 0x88, 4, p50xb_XFunc },
	{ 0x89, 4, p50xb_XFuncX },
	{ 0x8A, 5, p50xb_XFunc34 },
	{ 0x8B, 5, p50xb_XBinSt },
	{ 0x8C, 3, p50xb_XFuncSts },
	{ 0x8D, 3, p50xb_XFuncXSts },
	{ 0x8E, 3, p50xb_XFunc34Sts },
	{ 0x90, 3, p50xb_XTrnt },
	{ 0x91, 3, p50xb_XTrntX },
	{ 0x94, 3, p50xb_XTrntSts },
	{ 0x95, 2, p50xb_XTrntGrp },
	{ 0x98, 2, p50xb_XSensor },
	{ 0x99, 1, p50xb_XSensOff },
	{ 0x9C, 2, p50xb_X88PGet },
	{ 0x9D, 3, p50xb_X88PSet },
	{ 0xA0, 1, p50xb_XVer },
	{ 0xA2, 1, p50xb_XStatus },
	{ 0xA3, 4, p50xb_XSoSet },
	{ 0xA4, 3, p50xb_XSoGet },
	{ 0xA5, 1, p50xb_XHalt },
	{ 0xA6, 1, p50xb_XPwrOff },
	{ 0xA7, 1, p50xb_XPwrOn },
	{ 0xC4, 1, p50xb_XNop },
	{ 0xC6, 2, p50xb_XP50Len1 },
	{ 0xC7, 3, p50xb_XP50Len2 },
	{ 0xC8, 1, p50xb_XEvent },
	{ 0xC9, 1, p50xb_XEvtLok },
	{ 0xCA, 1, p50xb_XEvtTrn },
	{ 0xCB, 1, p50xb_XEvtSen },
	{ 0xCE, 1, p50xb_XEvtPT },
	{ 0xD8, 6, p50xb_XDCC_PAX },		// command XDCC_PAX
	{ 0xD9, 5, p50xb_XDCC_PAX },		// command XDCC_PARX

	{ 0, 0, NULL }				// end of list
};

static int p50xb_interpret (struct connection *con)
{
	const struct p50xbcmd *ct;
	uint8_t cmd;
	int len;
//	int i;

//	printf ("%s(): %d bytes\n\t", __func__, con->idx);
//	for (i = 0; i < con->idx; i++) {
//		printf ("%02x ", con->data[i]);
//	}
//	putchar('\n');

	ct = p50xbcmds;
	cmd = con->data[0];
	len = con->idx;
	if (!(con->flags & FLAG_IFEXT)) {
		cmd = con->data[1];		// skip the leading 'X'
		len--;					// don't count the 'X' as received byte for interpretion pruposes
	}

	while (ct->cmd >= 0x80) {
		if (cmd == ct->cmd) {
			if (len < ct->len) return 0;							// command not yet complete - don't eat any characters
			con->rc = ct->func(con);
			p50xb_error(con->sock, con->rc);
			if (!(con->flags & FLAG_IFEXT)) return ct->len + 1;		// take care of leading 'X' to be counted as char to be dropped
			return ct->len;
		}
		ct++;
	}
	p50xb_error(con->sock, XERROR);
	return con->idx;	// drop all bytes received
}

/*
 * ===============================================================================================
 * the P50X ASCII commands =======================================================================
 * ===============================================================================================
 */
static void p50xa_send (int sock, const char *fmt, ...) __attribute((format(printf, 2, 3)));

static void _p50xa_send (int sock, const char *buf)
{
	lwip_send (sock, buf, strlen(buf), 0);
}

static void p50xa_send (int sock, const char *fmt, ...)
{
	va_list ap;
	char *buf;

	if (!fmt) return;

	buf = tmp256();
	va_start (ap, fmt);
	vsnprintf(buf, 256, fmt, ap);
	va_end (ap);

	_p50xa_send(sock, buf);
}

static void p50xa_error (int sock, int errcode)
{
	if (errcode == OK || errcode == NOANSWER) return;	// don't output anything if the error code signals everything is OK / nothing to output

	if (errcode >= 0x40) p50xa_send (sock, "WARNING: ");
	else p50xa_send (sock, "ERROR: ");
	switch (errcode) {
		case XERROR:
			p50xa_send(sock, "unknown command");
			break;
		case XBADPRM:
			p50xa_send (sock, "bad parameter value");
			break;
		case XPWOFF:
			p50xa_send (sock, "power is Off");
			break;
		case XNOTSPC:
			p50xa_send (sock, "turnout queue full");
			break;
		case XNOLSPC:
			p50xa_send (sock, "command queue full");
			break;
		case XNODATA:
			p50xa_send (sock, "no data");
			break;
		case XNOSLOT:
			p50xa_send (sock, "no slot available");
			break;
		case XLKBUSY:
			p50xa_send (sock, "lok busy");
			break;
		case XBADTNP:
			p50xa_send (sock, "illegal turnout adress");
			break;
		case XLKHALT:
			p50xa_send (sock, "MC in HALT");
			break;
		case XLKPOFF:
			p50xa_send (sock, "MC in STOP");
			break;
		case XNOTIMPL:
			p50xa_send (sock, "not implemented (yet)");
			break;
	}
	if (errcode >= 0x40) p50xa_send(sock, "\r");	// normal output may follow
}

static void __attribute__((unused)) p50xa_pterror (int sock, int errcode)
{
	switch (errcode) {
		case PTERR_OK:
			p50xa_send (sock, "Ok");
			break;
		case PTERR_ERROR:
			p50xa_send (sock, "Error");
			break;
		case PTERR_YES:
			p50xa_send (sock, "Yes");
			break;
		case PTERR_NO:
			p50xa_send (sock, "No");
			break;
		case PTERR_BUSY:
			p50xa_send (sock, "Busy!");
			break;
		case PTERR_SHORT:
			p50xa_send (sock, "Short!");
			break;
		case PTERR_NODEC:
			p50xa_send (sock, "No decoder");
			break;
		case PTERR_NOACK:
			p50xa_send (sock, "No ack");
			break;
		case PTERR_NOPAGE:
			p50xa_send (sock, "No page");
			break;
		case PTERR_BITRD:
			p50xa_send (sock, "Bit read error");
			break;
		case PTERR_TIMEOUT:
			p50xa_send (sock, "Timeout");
			break;
	}
}

static bool p50xa_isCmdChar (char c)
{
	if (!c) return false;								// clearly: this is the end of the hole string
	if (c == ',' || c == '$' || c == '%') return false;	// chars that belong to parameters
	if (c >= '0' && c <= '9') return false;				// cmds may not contain digits
	if (isspace (c)) return false;						// blanks (and CR/NL) also terminate the command
	return true;
}

static int p50xa_args (char *args, struct parameter *prm)
{
	char *s = args;
	char *d, *q, delim;
	int p_idx = 0;
	bool comma;

	while (*s && p_idx < MAX_PARAMS) {
		while (isblank (*s)) s++;
		if (!*s) return p_idx;		// after whitespace at the end of the command

		if (*s == ',') {			// beginn loop again to process next parameter
			s++;
			p_idx++;
			prm++;
			continue;
		}

		// set prm->text to current position (possibly after a quotation character)
		delim = 0;
		if (*s == '\'' || *s == '"') delim = *s++;
		prm->text = s;

		if (delim) {	// scan string for next occurence of the delimiter character
			while (*s && *s != delim) s++;
		} else {		// scan string for space or komma
			while (*s && *s != ',' && !isblank(*s)) s++;
		}
		// After this scan, we sure have some kind of delimiting character which we might overwrite with a null byte.
		// The only special case is when we hit the end of the parameters in which case we must not advance 's'.
		comma = (*s == ',');
		if (*s) *s++ = 0;

		d = q = prm->text;
		if (*prm->text == '$') {			// scan as hex value
			d++;
			prm->value = strtoul (d, &q, 16);
		} else if (*prm->text == '%') {		// scan as binary value
			d++;
			prm->value = strtoul (d, &q, 2);
		} else {							// scan as decimal value
			prm->value = strtol (d, &q, 10);
			if (q > d) prm->numeric = true;
		}
		if (q > d) prm->numeric = true;
		prm->supplied = (strlen(prm->text) > 0);

		// advance to next parameter
		while (*s && isblank (*s)) s++;					// skip optional whitespace
		if (!comma && *s == ',') s++;					// skip optional comma
		p_idx++;
		prm++;
	}

	return p_idx;
}

static int p50xa_argrange (struct connection *con, int idx, int min, int max, int old)
{
	int v;

	if (idx >= MAX_PARAMS) return old;
	if (idx >= con->pcount) return old;
	if (!con->param[idx].supplied) return old;
	if (!con->param[idx].numeric) return old;

	v = con->param[idx].value;
	if (v >= min && v <= max) return v;
	return old;
}

static int p50xa_getarg (struct connection *con, int idx, int old)
{
	if (idx >= MAX_PARAMS) return old;
	if (idx >= con->pcount) return old;
	if (!con->param[idx].supplied) return old;
	if (!con->param[idx].numeric) return old;

	return con->param[idx].value;
}

static void __attribute__((unused)) p50xa_debug (struct connection *con, char *cmd, int len)
{
	struct parameter *p;
	int i;

	printf ("%s(): %*.*s\n", __func__, len, len, cmd);
	for (i = 0, p = con->param; i < con->pcount; i++, p++) {
		if (p->supplied) {
			if (!p->numeric) printf ("\t%d - '%s'\n", i, p->text);
			else printf ("\t%d - '%s' = %u (0x%04x)\n", i, p->text, p->value, p->value);
		} else {
			printf ("\t%d - NOT SUPPLIED\n", i);
		}
	}
}

static int p50xa_help_generic (struct connection *con)
{
	p50xa_send(con->sock, "HL Help for lok commands\r");
	p50xa_send(con->sock, "HF Help for function commands\r");
	p50xa_send(con->sock, "HT Help for turnout commands");
	return OK;
}

static int p50xa_help_loco (struct connection *con)
{
	p50xa_send(con->sock, "L Lok# {, [Speed], [FL], [Dir], [F1], [F2], [F3], [F4]}");
	return OK;
}

static int p50xa_help_turnout (struct connection *con)
{
	p50xa_send(con->sock, "T {Trnt#, [Color], [Status]}");
	return OK;
}

static int p50xa_help_function (struct connection *con)
{
	p50xa_send(con->sock, "F Lok# {, [F1], [F2], [F3], [F4], [F5], [F6], [F7], [F8]}\r");
	p50xa_send(con->sock, "FX Lok# {, [F9], [F10], [F11], [F12], [F13], [F14], [F15], [F16]}");
	return OK;
}

static int p50xa_stop (struct connection *con)
{
	(void) con;

	sig_setMode(TM_STOP);
	return OK;
}

static int p50xa_go (struct connection *con)
{
	(void) con;

	sig_setMode(TM_GO);
	return OK;
}

static int p50xa_halt (struct connection *con)
{
	(void) con;

	sig_setMode(TM_HALT);
	return OK;
}

static int p50xa_loco (struct connection *con)
{
	int adr, speed, ifspeed;
	ldataT *l;
	uint32_t funcs;
	bool dir;

	if ((adr = p50xa_argrange(con, 0, 1, MAX_LOCO_ADR, 0)) == 0) return XBADPRM;
	if (con->pcount >= 2) {
		if ((l = loco_call(adr, true)) == NULL) return XNOSLOT;

		funcs = 0;
		if (p50xa_getarg(con, 2, l->funcs[0] & FUNC_LIGHT)) funcs |= FUNC_LIGHT;
		if (p50xa_getarg(con, 4, l->funcs[0] & FUNC(1))) funcs |= FUNC(1);
		if (p50xa_getarg(con, 5, l->funcs[0] & FUNC(2))) funcs |= FUNC(2);
		if (p50xa_getarg(con, 6, l->funcs[0] & FUNC(3))) funcs |= FUNC(3);
		if (p50xa_getarg(con, 7, l->funcs[0] & FUNC(4))) funcs |= FUNC(4);
		rq_setFuncMasked(adr, funcs, FUNC_F0_F4);

		ifspeed = p50xa_argrange (con, 1, 0, 127, p50x_speed2if(l->speed, l->loco->fmt));
		if (ifspeed == 1) {       // do an emergency stop for that loco .....
			speed = 0;
			loco_emergencyStop(adr);
		} else {
			speed = p50x_if2speed (ifspeed, l->loco->fmt);
		}
		dir = l->speed & 0x80;
		if (con->pcount >= 4 && con->param[3].supplied) {
			if (con->param[3].numeric) {
				dir = !!(con->param[3].value);
			} else {
				if (*con->param[3].text == 'F' || *con->param[3].text == 'f') dir = true;
				if (*con->param[3].text == 'R' || *con->param[3].text == 'r') dir = false;
			}
		}
		if (dir) speed |= 0x80;
		// if direction changes, send an intermediate STOP-Speed to circumvent emergency stop
		if ((speed & 0x80) != (l->speed & 0x80)) rq_setSpeed (adr, l->speed & 0x80);
		rq_setSpeed(adr, speed);
		if (rt.tm == TM_HALT) return XLKHALT;
		if (rt.tm != TM_GO) return XLKPOFF;
	} else {
		if ((l = loco_call(adr, false)) == NULL) return XNODATA;
	}

	// Now print result as answer
	p50xa_send(con->sock, "L %d %d %c %c %c %c %c %c", adr, p50x_speed2if (l->speed, l->loco->fmt),
			(l->funcs[0] & FUNC_LIGHT) ? '1' : '0', (l->speed & 0x80) ? 'f' : 'r',
			(l->funcs[0] & FUNC(1)) ? '1' : '0', (l->funcs[0] & FUNC(2)) ? '1' : '0',
			(l->funcs[0] & FUNC(3)) ? '1' : '0', (l->funcs[0] & FUNC(4)) ? '1' : '0');

	return OK;
}

static int p50xa_locoProtocol (struct connection *con)
{
	int adr;
	ldataT *l;

	if ((adr = p50xa_argrange(con, 0, 1, MAX_LOCO_ADR, 0)) == 0) return XBADPRM;
	if ((l = loco_call(adr, false)) == NULL) {
		p50xa_send(con->sock, "unused");
	} else {
		if (FMT_IS_M3(l->loco->fmt)) p50xa_send (con->sock, "m3");
		else if (FMT_IS_MM1(l->loco->fmt)) p50xa_send (con->sock, "Motorola Old");
		else if (FMT_IS_MM2(l->loco->fmt)) p50xa_send (con->sock, "Motorola New");
		else if (FMT_IS_DCC(l->loco->fmt)) p50xa_send (con->sock, "DCC");
		else p50xa_send (con->sock, "UNKNOWN");
	}
	return OK;
}

static int p50xa_functionF1F8 (struct connection *con)
{
	int adr;
	uint32_t funcs;
	ldataT *l;

	if ((adr = p50xa_argrange(con, 0, 1, MAX_LOCO_ADR, 0)) == 0) return XBADPRM;
	if (con->pcount >= 2) {
		if ((l = loco_call(adr, true)) == NULL) return XNOSLOT;

		funcs = 0;
		if (p50xa_getarg(con, 1, l->funcs[0] & FUNC(1))) funcs |= FUNC(1);
		if (p50xa_getarg(con, 2, l->funcs[0] & FUNC(2))) funcs |= FUNC(2);
		if (p50xa_getarg(con, 3, l->funcs[0] & FUNC(3))) funcs |= FUNC(3);
		if (p50xa_getarg(con, 4, l->funcs[0] & FUNC(4))) funcs |= FUNC(4);
		if (p50xa_getarg(con, 5, l->funcs[0] & FUNC(5))) funcs |= FUNC(5);
		if (p50xa_getarg(con, 6, l->funcs[0] & FUNC(6))) funcs |= FUNC(6);
		if (p50xa_getarg(con, 7, l->funcs[0] & FUNC(7))) funcs |= FUNC(7);
		if (p50xa_getarg(con, 8, l->funcs[0] & FUNC(8))) funcs |= FUNC(8);
		rq_setFuncMasked(adr, funcs, FUNC_F1_F8);
		if (rt.tm != TM_GO && rt.tm != TM_HALT) return XLKPOFF;
	} else {
		if ((l = loco_call(adr, false)) == NULL) return XNODATA;
	}

	// Now print result as answer
	p50xa_send(con->sock, "F %u %c %c %c %c %c %c %c %c", adr,
			(l->funcs[0] & FUNC(1)) ? '1' : '0', (l->funcs[0] & FUNC(2)) ? '1' : '0',
			(l->funcs[0] & FUNC(3)) ? '1' : '0', (l->funcs[0] & FUNC(4)) ? '1' : '0',
			(l->funcs[0] & FUNC(5)) ? '1' : '0', (l->funcs[0] & FUNC(6)) ? '1' : '0',
			(l->funcs[0] & FUNC(7)) ? '1' : '0', (l->funcs[0] & FUNC(8)) ? '1' : '0');

	return OK;
}

static int p50xa_functionF9F16 (struct connection *con)
{
	int adr;
	uint32_t funcs;
	ldataT *l;

	if ((adr = p50xa_argrange(con, 0, 1, MAX_LOCO_ADR, 0)) == 0) return XBADPRM;
	if (con->pcount >= 2) {
		if ((l = loco_call(adr, true)) == NULL) return XNOSLOT;

		funcs = 0;
		if (p50xa_getarg(con, 1, l->funcs[0] & FUNC(9))) funcs |= FUNC(9);
		if (p50xa_getarg(con, 2, l->funcs[0] & FUNC(10))) funcs |= FUNC(10);
		if (p50xa_getarg(con, 3, l->funcs[0] & FUNC(11))) funcs |= FUNC(11);
		if (p50xa_getarg(con, 4, l->funcs[0] & FUNC(12))) funcs |= FUNC(12);
		if (p50xa_getarg(con, 5, l->funcs[0] & FUNC(13))) funcs |= FUNC(13);
		if (p50xa_getarg(con, 6, l->funcs[0] & FUNC(14))) funcs |= FUNC(14);
		if (p50xa_getarg(con, 7, l->funcs[0] & FUNC(15))) funcs |= FUNC(15);
		if (p50xa_getarg(con, 8, l->funcs[0] & FUNC(16))) funcs |= FUNC(16);
		rq_setFuncMasked(adr, funcs, FUNC_F9_F16);
		if (rt.tm != TM_GO && rt.tm != TM_HALT) return XLKPOFF;
	} else {
		if ((l = loco_call(adr, false)) == NULL) return XNODATA;
	}

	// Now print result as answer
	p50xa_send(con->sock, "FX %u %c %c %c %c %c %c %c %c", adr,
			(l->funcs[0] & FUNC(9)) ? '1' : '0', (l->funcs[0] & FUNC(10)) ? '1' : '0',
			(l->funcs[0] & FUNC(11)) ? '1' : '0', (l->funcs[0] & FUNC(12)) ? '1' : '0',
			(l->funcs[0] & FUNC(13)) ? '1' : '0', (l->funcs[0] & FUNC(14)) ? '1' : '0',
			(l->funcs[0] & FUNC(15)) ? '1' : '0', (l->funcs[0] & FUNC(16)) ? '1' : '0');

	return OK;
}

static int p50xa_turnout (struct connection *con)
{
	int adr;
	turnoutT *t;
	bool on, thrown;

	if ((adr = p50xa_argrange(con, 0, 1, MAX_TURNOUT, 0)) == 0) return XBADPRM;
	if ((t = db_getTurnout(adr)) == NULL) return XBADPRM;

	if (con->param[1].supplied) {	// switch turnout
		if (rt.tm != TM_GO && rt.tm != TM_HALT) return XPWOFF;
		switch (*con->param[1].text) {
			case '0':
			case 'r':
			case 'R':
				thrown = true;
				break;
			case '1':
			case 'g':
			case 'G':
				thrown = false;
				break;
			default:
				return XBADPRM;
		}
		on = p50xa_getarg(con, 2, 0);
		if (trnt_switch(adr, thrown, on)) return XNOTSPC;
	}

	p50xa_send(con->sock, "T %d %c %d", t->adr, t->dir ? 'r' : 'g', t->on);

	return OK;
}

static int p50xa_status (struct connection *con)
{
	switch (rt.tm) {
		case TM_STOP:
		case TM_OVERTTEMP:
		case TM_TEMPOK:
			p50xa_send(con->sock, "Pwr off");
			break;
		case TM_SHORT:
			p50xa_send(con->sock, "SHORT!");
			break;
		case TM_HALT:
			p50xa_send(con->sock, "Halted!");
			break;
		case TM_SIGON:
		case TM_GO:
			p50xa_send(con->sock, "Pwr on");
			break;
		case TM_DCCPROG:
			p50xa_send(con->sock, "DCC program");
			break;
		case TM_TAMSPROG:
			p50xa_send(con->sock, "TPM program");
			break;
		case TM_RESET:
			p50xa_send(con->sock, "RESET");
			break;
		case TM_TESTDRIVE:
		case TM_POWERFAIL:
			/* currently ignored */
			break;
	}
	return OK;
}

static int p50xa_version (struct connection *con)
{
	p50xa_send(con->sock, "MC2 Revision " SOFT_VERSION "\r");
	p50xa_send(con->sock, "SerNr. %d", hwinfo->serial);
	return OK;
}

static int p50xa_magtimer (struct connection *con)
{
	int t;

	if ((t = p50xa_argrange(con, 0, 2, 40, -1)) >= 0) trnt_setMinTime(t * 50);
	if ((t = p50xa_argrange(con, 1, 2, 100, -1)) >= 0) trnt_setMaxTime(t * 50);

	p50xa_send(con->sock, "MT %d %d", (trnt_getMinTime() + 25) / 50, (trnt_getMaxTime() + 25) / 50);
	return OK;
}

static int p50xa_s88autoreset (struct connection *con)
{
	int on;

	if ((on = p50xa_argrange(con, 0, 0, 1, -1)) >= 0) {
		if (on) con->flags |= FLAG_S88AUTORESET;
		else con->flags &= ~FLAG_S88AUTORESET;
	}

	p50xa_send(con->sock, "SR %d", (con->flags & FLAG_S88AUTORESET) ? 1 : 0);
	return OK;
}

static int p50xa_s88readout (struct connection *con)
{
	char answer[64], *s;
	int module, i;
	uint16_t *m, mask;

	if ((module = p50xa_argrange(con, 0, 1, s88_getModules(), 0)) == 0) return XBADPRM;
	m = &con->s88Sum[module - 1];

	sprintf (answer, "Module #%d (input 1..8 & 9..16) = ", module);
	s = answer + strlen(answer);
	for (i = 0, mask = 0x100; i < 16; i++,  mask <<= 1) {
		if (i == 8) {
			*s++ = ' ';
			mask = 1;
		}
		*s++ = (*m & mask) ? '1' : '0';
	}
	*s = 0;
	if (con->flags & FLAG_S88AUTORESET) *m = 0;
	p50xa_send(con->sock, answer);
	return OK;
}

static int p50xa_s88Modules (struct connection *con)
{
	int count;

	if (con->pcount > 0) {
		count = p50xa_argrange(con, 0, 0, MAX_FBMODULES, s88_getModules() << 1);
//		s88_setModules((count + 1) >> 1);		// maybe we have to round up
	}
	p50xa_send(con->sock, "SE %d", s88_getModules() << 1);
	return OK;
}

static int p50xa_sysReset (struct connection *con)
{
	(void) con;

	reboot();
	return OK;
}

static int p50xa_baudrate (struct connection *con)
{
	if (!con->pcount) {	// only report baudrate, if tried to be set, we can ignore this attempt :-)
		p50xa_send (con->sock, "57600");	// we always report a dummy baudrate
	}
	return OK;		// it is always OK and never can changed
}

static int p50xa_SO (struct connection *con)
{
	int soadr, val;

	if (!con->pcount) return XBADPRM;

	soadr = p50xa_argrange(con, 0, 0, 999, -1);
	if (con->pcount == 1) {		// read SO variable
		val = p50x_SOget(soadr);
		if (val < 0) return XBADPRM;
	} else {					// write SO variable
		val = p50xa_argrange(con, 1, 0, 255, -1);
		if (val < 0) return XBADPRM;
		if (p50x_SOset(soadr, val)) return XBADPRM;
	}
	p50xa_send(con->sock, "%d", val & 0xFF);

	return OK;
}

static int p50xa_prgcv (struct connection *con)
{
	int cv17, cv18;

	cv17 = p50xa_argrange (con, 0, 192, 231, -1);
	cv18 = p50xa_argrange (con, 1, 0, 255, -1);
	if (cv17 < 0 || cv18 < 0) return XBADPRM;
	p50xa_send(con->sock, "Long Addr = %u", ((cv17 & 0x3F) << 8) + cv18);

	return OK;
}

static int p50xa_prgca (struct connection *con)
{
	int adr;

	if ((adr = p50xa_argrange (con, 0, 1, MAX_DCC_ADR, 0)) == 0) return XBADPRM;
	p50xa_send(con->sock, "Addr %d = $%04X => CV17 = %d ($%02X), CV18 = %d ($%02X)", adr, adr,
			((adr >> 8) & 0x3F) + 0xC0, ((adr >> 8) & 0x3F) + 0xC8, adr & 0xFF, adr & 0xFF);

	return OK;
}

static bool p50xa_maintrackCallback (struct decoder_reply *msg, flexval priv)
{
	struct connection *con;
	int result;

	con = (struct connection *) priv.p;

	result = msg->data[0];
	if (msg->mt == DECODERMSG_NOANSWER) {
		p50xa_pterror(con->sock, PTERR_NOACK);
	} else {
		p50xa_send(con->sock, "%s %d %ld %d", (msg->dtype == DECODER_DCC_MOBILE) ? "PD" : (msg->dtype == DECODER_DCC_ACC) ? "PA" : "PX",
				msg->adr, msg->param.i32 + 1, result);
	}
	xTaskNotifyGive(con->tid);
	return false;
}

static int p50xa_pgMaintrack (struct connection *con)
{
	int adr, cv, val;
	flexval fv;

	if (rt.tm != TM_GO && rt.tm != TM_HALT) return XPWOFF;

	if ((cv = p50xa_argrange (con, 1, MIN_DCC_CVADR + 1, MAX_DCC_CVADR + 1, 0) - 1) < 0) return XBADPRM;
	val = p50xa_argrange (con, 2, 0, 255, -1);
	fv.p = con;

	if (!strncasecmp (con->cmd, "PD", 2)) {
		if ((adr = p50xa_argrange (con, 0, 1, MAX_DCC_ADR, 0)) == 0) return XBADPRM;
		if (val >= 0) {
			if (dccpom_writeByte(adr, DECODER_DCC_MOBILE, cv, val, p50xa_maintrackCallback, fv) != 0) return XERROR;
		} else {
			if (dccpom_readByte(adr, DECODER_DCC_MOBILE, cv, p50xa_maintrackCallback, fv) != 0) return XERROR;
		}
	} else if (!strncasecmp (con->cmd, "PA", 2)) {
		if ((adr = p50xa_argrange (con, 0, 1, MAX_DCC_ACCESSORY >> 2, 0)) == 0) return XBADPRM;
		if (val >= 0) {
			if (dccpom_writeByte(adr, DECODER_DCC_ACC, cv, val, p50xa_maintrackCallback, fv) != 0) return XERROR;
//			if (trnt_writecv(adr, cv, val, p50xa_maintrackCallback, (void *) con) != 0) return XERROR;
		} else {
			if (dccpom_readByte(adr, DECODER_DCC_ACC, cv, p50xa_maintrackCallback, fv) != 0) return XERROR;
//			if (trnt_readcv(adr, cv, p50xa_maintrackCallback, (void *) con) != 0) return XERROR;
		}
	} else if (!strncasecmp (con->cmd, "PX", 2)) {
		if ((adr = p50xa_argrange (con, 0, 1, MAX_DCC_EXTACC, 0)) == 0) return XBADPRM;
		if (val >= 0) {
			if (dccpom_writeByte(adr, DECODER_DCC_EXT, cv, val, p50xa_maintrackCallback, fv) != 0) return XERROR;
//			if (xacc_writecv(adr, cv, val, p50xa_maintrackCallback, (void *) con) != 0) return XERROR;
		} else {
			if (dccpom_readByte(adr, DECODER_DCC_EXT, cv, p50xa_maintrackCallback, fv) != 0) return XERROR;
//			if (xacc_readcv(adr, cv, p50xa_maintrackCallback, (void *) con) != 0) return XERROR;
		}
	} else if (!strncasecmp (con->cmd, "PE", 2)) {	// @TODO: there is no separate function to do that, yet!
		if ((adr = p50xa_argrange (con, 0, 1, MAX_DCC_ACCESSORY, 0)) == 0) return XBADPRM;
		return XNOTIMPL;
	} else {
		return XBADPRM;
	}

	xTaskNotifyStateClear(NULL);
	if (ulTaskNotifyTake(1, 5000) == 0) p50xa_pterror (con->sock, PTERR_TIMEOUT);

	return OK;
}

//#define ERR_NO_LOCO					-1		///< no decocder detected (current draw to small)
//#define ERR_SHORT						-10		///< there was a short on the programming track
static int p50xa_pgRdProgtrack (struct connection *con)
{
	int rc, cv, mask, bit;
	char response[64], *s;

	if ((cv = p50xa_argrange (con, 0, MIN_DCC_CVADR + 1, MAX_DCC_CVADR + 1, 0) - 1) < 0) return XBADPRM;
	bit = -1;

	rc = 0;		// $$$$ to keep compiler happy until the unimplemented functions are finally implemented
	switch (con->cmd[3]) {
		case 'R':		// physical register mode
		case 'r':
			if (cv > 7) return XBADPRM;
			return XNOTIMPL;
			break;
		case 'P':		// page mode
		case 'p':
			return XNOTIMPL;
			break;
		case 'D':		// direct mode (byte)
		case 'd':
			rc = dccpt_cvReadByte(cv);
			break;
		case 'B':		// direct mode (bit) - the IB returns the whole byte, so we do the same if the bit number is not supplied :-)
		case 'b':
			if ((bit = p50xa_argrange (con, 1, 0, 7, -1)) < 0) {
				rc = dccpt_cvReadByte(cv);
			} else {
				rc = dccpt_cvReadBit(cv, bit);
			}
			break;
	}

	if (rc < 0) {
		switch (rc) {
			case -1:		// no loco
				p50xa_pterror(con->sock, PTERR_NODEC);
				break;
			case -10:		// SHORT
				p50xa_pterror(con->sock, PTERR_SHORT);
				break;
			default:
				p50xa_pterror(con->sock, PTERR_ERROR);
				break;
		}
	} else {
		if (bit >= 0 && bit <= 7) {
			sprintf (response, "%d", rc);
		} else {
			sprintf (response, "%d = $%02X = %%", rc, rc);
			s = response + strlen(response);
			for (mask = 0x80; mask; mask >>= 1) {
				*s++ = (rc & mask) ? '1' : '0';
			}
			*s = 0;
		}
		_p50xa_send (con->sock, response);
	}

	return OK;
}

static int p50xa_pgWrProgtrack (struct connection *con)
{
	int rc, cv, bit, val;

	if ((cv = p50xa_argrange (con, 0, MIN_DCC_CVADR + 1, MAX_DCC_CVADR + 1, 0) - 1) < 0) return XBADPRM;

	rc = 0;		// $$$$ to keep compiler happy until the unimplemented functions are finally implemented
	switch (con->cmd[3]) {
		case 'R':
		case 'r':
			if (cv > 7) return XBADPRM;
			return XNOTIMPL;
			break;
		case 'P':
		case 'p':
			return XNOTIMPL;
			break;
		case 'D':
		case 'd':
			if ((val = p50xa_argrange (con, 1, 0, 255, -1)) < 0) return XBADPRM;
			rc = dccpt_cvWriteByte(cv, val);
			break;
		case 'B':
		case 'b':
			if ((bit = p50xa_argrange (con, 1, 0, 7, -1)) < 0) return XBADPRM;
			if ((val = p50xa_argrange (con, 2, 0, 1, -1)) < 0) return XBADPRM;
			rc = dccpt_cvWriteBit(cv, bit, val);
			break;
	}

	if (rc < 0) {
		switch (rc) {
			case -1:		// no loco
				p50xa_pterror(con->sock, PTERR_NODEC);
				break;
			case -10:		// SHORT
				p50xa_pterror(con->sock, PTERR_SHORT);
				break;
			default:
				p50xa_pterror(con->sock, PTERR_ERROR);
				break;
		}
	}

	return OK;
}

static int p50xa_pgRdLongaddr (struct connection *con)
{
	int adr, cv17, cv18, rc;

	rc = cv17 = dccpt_cvReadByte (16);
	if (cv17 < 192 || cv17 > 231) {		// CV17 has a limited range to be valid
		p50xa_pterror(con->sock, PTERR_ERROR);
		return OK;
	}

	if (rc >= 0) cv18 = dccpt_cvReadByte (17);
	//	if (rc >= 0) rc = dccpt_cvWriteBit (28, 5, 1);		// @TODO: not implemented yet!

	if (rc < 0) {
		switch (rc) {
			case -1:		// no loco
				p50xa_pterror(con->sock, PTERR_NODEC);
				break;
			case -10:		// SHORT
				p50xa_pterror(con->sock, PTERR_SHORT);
				break;
			default:
				p50xa_pterror(con->sock, PTERR_ERROR);
				break;
		}
	} else {
		adr = ((cv17 & 0x3F) << 8) | (cv18 & 0xFF);
		p50xa_send(con->sock, "%d", adr);
	}

	return OK;
}

static int p50xa_pgWrLongaddr (struct connection *con)
{
	int adr, cv17, cv18, rc;

	if ((adr = p50xa_argrange (con, 0, 128, MAX_DCC_ADR, 0)) == 0) return XBADPRM;
	cv17 = (adr >> 8) | 0xC0;
	cv18 = adr & 0xFF;

	rc = dccpt_cvWriteByte (16, cv17);
	if (rc >= 0) rc = dccpt_cvWriteByte (17, cv18);
	if (rc >= 0) rc = dccpt_cvWriteBit (28, 5, 1);

	if (rc < 0) {
		switch (rc) {
			case -1:		// no loco
				p50xa_pterror(con->sock, PTERR_NODEC);
				break;
			case -10:		// SHORT
				p50xa_pterror(con->sock, PTERR_SHORT);
				break;
			default:
				p50xa_pterror(con->sock, PTERR_ERROR);
				break;
		}
	}

	return OK;
}

static const char *p50xa_fmt2str (enum fmt fmt)
{
	switch (fmt) {
		case FMT_MM1_14:
			return "MM1";
		case FMT_MM2_14:
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			return "MM2";
		case FMT_DCC_14:
		case FMT_DCC_28:
		case FMT_DCC_126:
		case FMT_DCC_SDF:
			return "DCC";
		case FMT_M3_126:
			return "M3";
		default:
			return NULL;
	}
}

static const char *p50xa_speed2str (enum fmt fmt)
{
	switch (fmt) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_DCC_14:
			return "14";
		case FMT_MM2_27A:
			return "27a";
		case FMT_MM2_27B:
			return "27b";
		case FMT_DCC_28:
			return "28";
		case FMT_DCC_126:
		case FMT_DCC_SDF:
		case FMT_M3_126:
			return "126";
		default:
			return NULL;
	}
}

static bool _p50xa_locdump (locoT *l, void *priv)
{
	struct connection *con;
	const char *fmt, *speeds;
	bool senduid;

	con = (struct connection *) priv;
	if (!con || !l) return false;
	fmt = p50xa_fmt2str(l->fmt);
	speeds = p50xa_speed2str(l->fmt);
	if (!fmt || !speeds) return true;	// illegal settings are not dumped!

	senduid = false;
	if (con->pcount >= 1 && con->param[0].supplied){
		if ((!strcasecmp("UID",con->param[0].text)) && (l->uid)) senduid = true;
	}


	if (*l->name) {
		if (senduid){
			p50xa_send (con->sock, "%d, %s, %s, %s, 0x%08lX\r", l->adr, speeds, fmt, l->name, l->uid);
		} else {
			p50xa_send (con->sock, "%d, %s, %s, %s\r", l->adr, speeds, fmt, l->name);
		}
	} else {
		if (senduid){
			p50xa_send (con->sock, "%d, %s, %s, , 0x%08lX\r", l->adr, speeds, fmt,  l->uid);
		} else {
			p50xa_send (con->sock, "%d, %s, %s\r", l->adr, speeds, fmt);
		}

	}
	return true;
}

static int p50xa_locdump (struct connection *con)
{
	db_iterateLoco(_p50xa_locdump, con);
	p50xa_send (con->sock, "*END*");
	return OK;
}

static int p50xa_loccfg (struct connection *con)
{
	locoT *l;
	int adr, speeds;
	char *s;
	enum fmt fmt;

	fmt = FMT_UNKNOWN;
	speeds = 0;

	if ((adr = p50xa_argrange(con, 0, 1, MAX_LOCO_ADR, 0)) == 0) return XBADPRM;
	if ((l = db_getLoco(adr, true)) == 0) return XERROR;

	if (con->pcount >= 2 && con->param[1].supplied) {
		speeds = atoi(con->param[1].text);
		if (speeds == 27) {
			if (con->param[1].text[2] == 'B' || con->param[1].text[2] == 'b') speeds = -27;		// everything that is not 'B' is 'A'!
		}
	} else {
		switch (l->fmt) {
			case FMT_MM1_14:
			case FMT_MM2_14:
			case FMT_DCC_14:
				speeds = 14;
				break;
			case FMT_MM2_27A:
				speeds = 27;
				break;
			case FMT_MM2_27B:
				speeds = -27;
				break;
			case FMT_DCC_28:
				speeds = 28;
				break;
			case FMT_DCC_126:
			case FMT_DCC_SDF:
			case FMT_M3_126:
				speeds = 126;
				break;
			default:
				return XERROR;
		}
	}

	if (con->pcount >= 3 && con->param[2].supplied) {
		s = con->param[2].text;
	} else {
		if (FMT_IS_DCC(l->fmt)) s = "DCC";
		else if (FMT_IS_MM1(l->fmt)) s = "MM1";
		else if (FMT_IS_MM2(l->fmt)) s = "MM2";
		else if (FMT_IS_M3(l->fmt)) s = "M3";
		else return XERROR;
	}

	// now create the new format for that loco
	switch (speeds) {
		case 14:
			if (!strcasecmp ("MM1", s)) fmt = FMT_MM1_14;
			else if (!strcasecmp ("MM2", s)) fmt = FMT_MM2_14;
			else if (!strcasecmp ("DCC", s)) fmt = FMT_DCC_14;
			else return XBADPRM;
			break;
		case 27:
			if (!strcasecmp ("MM2", s)) fmt = FMT_MM2_27A;
			else return XBADPRM;
			break;
		case -27:
			if (!strcasecmp ("MM2", s)) fmt = FMT_MM2_27B;
			else return XBADPRM;
			break;
		case 28:
			if (!strcasecmp ("DCC", s)) fmt = FMT_DCC_28;
			else return XBADPRM;
			break;
		case 126:
			if (!strcasecmp ("DCC", s)) fmt = FMT_DCC_126;
			else if (!strcasecmp ("M3", s)) fmt = FMT_M3_126;
			else return XBADPRM;
			break;
		default:
			return XBADPRM;
	}

	db_setLocoFmt(l->adr, fmt);
	return OK;
}

static int p50xa_locdelete (struct connection *con)
{
	locoT *l;
	int adr;

	if ((adr = p50xa_argrange(con, 0, 1, MAX_LOCO_ADR, 0)) == 0) return XBADPRM;
	if ((l = db_getLoco(adr, true)) == NULL) return XERROR;
	db_removeLoco(l);
	return OK;
}

static int p50xa_locadd (struct connection *con)
{
	locoT *l;
	int adr, speeds;
	char *name, *s, *uid;
	enum fmt fmt;

	name = NULL;
	uid = NULL;
	fmt = FMT_UNKNOWN;
	speeds = 28;		// a sensefull default

	if ((adr = p50xa_argrange(con, 0, 1, MAX_LOCO_ADR, 0)) == 0) return XBADPRM;
	if (con->pcount >= 2 && con->param[1].supplied) {
		speeds = atoi(con->param[1].text);
		if (speeds == 27) {
			if (con->param[1].text[2] == 'B' || con->param[1].text[2] == 'b') speeds = -27;		// everything that is not 'B' is 'A'!
		}
		if (speeds > 126) speeds = 126;
	}
	if (con->pcount < 3 || !con->param[2].supplied) return XBADPRM;
	s = con->param[2].text;
	switch (speeds) {
		case 14:
			if (!strcasecmp ("MM1", s)) fmt = FMT_MM1_14;
			else if (!strcasecmp ("MM2", s)) fmt = FMT_MM2_14;
			else if (!strcasecmp ("DCC", s)) fmt = FMT_DCC_14;
			else return XBADPRM;
			break;
		case 27:
			if (!strcasecmp ("MM2", s)) fmt = FMT_MM2_27A;
			else return XBADPRM;
			break;
		case -27:
			if (!strcasecmp ("MM2", s)) fmt = FMT_MM2_27B;
			else return XBADPRM;
			break;
		case 28:
			if (!strcasecmp ("DCC", s)) fmt = FMT_DCC_28;
			else return XBADPRM;
			break;
		case 126:
			if (!strcasecmp ("DCC", s)) fmt = FMT_DCC_126;
			else if (!strcasecmp ("M3", s)) fmt = FMT_M3_126;
			else return XBADPRM;
			break;
		default:
			return XBADPRM;
	}
	if (con->pcount >= 4 && con->param[3].supplied) name = con->param[3].text;
	if (con->pcount >= 5 && con->param[4].supplied) uid = con->param[4].text;
	if ((l = db_newLoco(adr, fmt, 28, name, uid)) == 0) return XERROR;
	return OK;
}

static int p50xa_locclear (struct connection *con)
{
	(void) con;

	db_freeLocos();
	db_triggerStore(__func__);
	return OK;
}

static int p50xa_cfgdump (struct connection *con)
{
	struct sysconf *sc;
	struct consist *c;
	turnoutT *t;
	locoT *l;
	int i;

	// dump some info about the MC --------------------------------------------
	p50xa_send(con->sock, "[INFO]\r");
	p50xa_send(con->sock, "VERSION %s\r", SOFT_VERSION);
	p50xa_send(con->sock, "HARDWARE %x.%x\r", hwinfo->HW >> 4, hwinfo->HW & 0xF);
	p50xa_send(con->sock, "MCU STM32H743\r");
	p50xa_send(con->sock, "SERIAL %d\r", hwinfo->serial);

	// dump all locos currently known -----------------------------------------
	p50xa_send(con->sock, "[LOCO]\r");
	db_iterateLoco(_p50xa_locdump, con);

	// dump all known traktions -----------------------------------------------
	p50xa_send(con->sock, "[TRAKTIONS]\r");
	c = consist_getConsists();
	while (c) {
		p50xa_send(con->sock, "%d", abs(c->adr[0]));
		if (c->adr[0] < 0) p50xa_send(con->sock, "!");
		for (i = 1; i < MAX_CONSISTLENGTH && c->adr[i] != 0; i++) {
			p50xa_send(con->sock, ", %d", abs(c->adr[i]));
			if (c->adr[i] < 0) p50xa_send(con->sock, "!");
		}
		p50xa_send(con->sock, "\r");
		c = c->next;
	}

	// dump all known function mappings ---------------------------------------
	p50xa_send(con->sock, "[FUNCMAPS]\r");
	// @TODO implement a dump loop

	// dump the fmt of the accessory decoder groups ------------------------
	p50xa_send(con->sock, "[ACCFMT]\r");
	for (i = 0; i <= (MAX_MM_TURNOUT / 4); i++) {
		if ((t = db_lookupTurnout(i * 4 + 1)) != NULL) {
			p50xa_send(con->sock, "%d, %s\r", i, t->fmt == TFMT_MM ? "MM" : "DCC");
		}
	}

	// dump all system settings -----------------------------------------------
	sc = cnf_getconfig();
	l = db_getLoco(0, false);		// get the default loco format
	p50xa_send(con->sock, "[SYSTEM]\r");
	p50xa_send(con->sock, "LONGPAUSE %s\r", (sc->sysflags & SYSFLAG_LONGPAUSE) ? "yes" : "no");
	p50xa_send(con->sock, "DEFAULTDCC %s\r", (FMT_IS_DCC(l->fmt)) ? "yes" : "no");
	p50xa_send(con->sock, "SHORTTIME %d\r", ts_getSensitivity());
	p50xa_send(con->sock, "s88MODULES %d\r", s88_getModules());
	p50xa_send(con->sock, "MAGMINTIME %d\r", trnt_getMinTime());
	p50xa_send(con->sock, "MAGMAXTIME %d\r", trnt_getMaxTime());
	p50xa_send(con->sock, "BAUDRATE 57600\r");

	p50xa_send(con->sock, "*END*");

	return OK;
}

static int p50xa_cfgacc (struct connection *con)
{
	enum fmt fmt;
	turnoutT *t;
	int i, adr;

	if (con->pcount < 2 || !con->param[1].supplied) return XBADPRM;

	if (!strcasecmp ("MM", con->param[1].text)) fmt = TFMT_MM;
	else if (!strcasecmp ("DCC", con->param[1].text)) fmt = TFMT_DCC;
	else return XBADPRM;

	if (!con->param[0].supplied) {		// set new default format
		if ((t = db_lookupTurnout (0)) == NULL) return XERROR;
		t->fmt = fmt;
	} else {
		if ((adr = p50xa_argrange(con, 0, 0, 255, -1)) < 0) return XBADPRM;
		for (i = (adr * 4 + 1); i < (adr * 4 + 5); i++) {
			db_setTurnoutFmt (i, fmt);
		}
	}
	return OK;
}

static int p50xa_railcom (struct connection *con)
{
	struct fmtconfig *fc = cnf_getFMTconfig();
	int rcflag;

	if (con->pcount >= 1 && con->param[1].supplied) {
		if ((rcflag = p50xa_argrange(con, 0, 0, 7, -1)) < 0) return XBADPRM;
		if ((!!rcflag) != (!!(fc->sigflags & SIGFLAG_RAILCOM))) cnf_triggerStore(__func__);
		if (rcflag) fc->sigflags |= SIGFLAG_RAILCOM;
		else fc->sigflags &= ~(SIGFLAG_RAILCOM | SIGFLAG_DCCA);
	}

	p50xa_send(con->sock, "RC %d", (fc->sigflags & SIGFLAG_RAILCOM) ? 7 : 0);
	return OK;
}

static int p50xa_railcomRead (struct connection *con)
{
	struct fmtconfig *fc = cnf_getFMTconfig();
	int adr, cv, rpt;

	if (rt.tm != TM_GO && rt.tm != TM_HALT) return XPWOFF;

	if ((adr = p50xa_argrange (con, 0, 1, MAX_DCC_ADR, 0)) == 0) return XBADPRM;
	if ((cv = p50xa_argrange (con, 1, MIN_DCC_CVADR + 1, MAX_DCC_CVADR + 1, 0) - 1) < 0) return XBADPRM;
//	rpt = p50xa_argrange (con, 2, 1, 255, -1);		// this parameter is ignored
	rpt = fc->dcc.pomrepeat;

	if (dccpom_readByte(adr, DECODER_DCC_MOBILE, cv, NULL, fvNULL) != 0) return XERROR;

	p50xa_send(con->sock, "RCR %d %d %d", adr, cv, rpt);
	return OK;
}

static int p50xa_m3uid (struct connection *con)
{
	uint32_t uid;
	uint16_t adr = 0;

	if (con->pcount >= 1 && con->param[0].supplied) adr = con->param[0].value;

	uid = m3pt_getUID();
	if (adr > 0 && uid > 0) {
		m3pt_setAddress(uid, adr);
	}
	if (uid > 0) {
		p50xa_send(con->sock, "%08lX", uid);
	} else {
		p50xa_send(con->sock, "ERROR");
	}
	return OK;
}

static int p50xa_m3cvwriteImpl (struct connection *con, bool pt)
{
	// [adr], [cv], [sub], [val], [repeat])
	uint16_t adr;
	int val, repeat;
	cvadrT cva;

	if (con->pcount < 4) return XBADPRM;
	if (!con->param[0].supplied || !con->param[1].supplied || !con->param[2].supplied || !con->param[3].supplied) return XBADPRM;
	adr = con->param[0].value;
	if (adr > MAX_M3_ADR) return XBADPRM;
	cva.m3cv = con->param[1].value;
	if (cva.m3cv < MIN_M3_CVADR || cva.m3cv > MAX_M3_CVADR) return XBADPRM;
	cva.m3sub = con->param[2].value;
	if (cva.m3sub < 0 || cva.m3sub > MAX_M3_CVSUBADR) return XBADPRM;
	val = con->param[3].value;
	if (val < 0 || val > 0xFF) return XBADPRM;
	repeat = 8;
	if (con->pcount >= 5 && con->param[4].supplied) repeat = con->param[4].value;
	if (repeat < 1 || repeat > 100) return XBADPRM;

	if (pt) {
		m3pt_writeCV(adr, cva, val, repeat);
	} else {
		m3pom_writeCV(adr, cva, val, repeat, NULL, fvNULL);
	}

	return OK;
}

static int p50xa_m3PTcvwrite (struct connection *con)
{
	return p50xa_m3cvwriteImpl(con, true);
}

static int p50xa_m3cvwrite (struct connection *con)
{
	return p50xa_m3cvwriteImpl(con, false);
}

static const struct p50xacmd {		///< command table for P50Xa commands
	const char		*cmd;					///< the string that makes up the command
	int (*func)(struct connection *con);	///< the function to call
} p50xacmds[] = {
	{ "?",			p50xa_help_generic },	// print a short help (generic)
	{ "H",			p50xa_help_generic },	// print a short help (generic)
	{ "HL",			p50xa_help_loco },		// print a short help for loco command
	{ "HT",			p50xa_help_turnout },	// print a short help for turnout command
	{ "HF",			p50xa_help_function },	// print a short help for function command
	{ ".",			p50xa_stop },			// set system to STOP
	{ "STOP",		p50xa_stop },			// set system to STOP
	{ "!",			p50xa_go },				// set system to GO
	{ "GO",			p50xa_go },				// set system to GO
	{ "HALT",		p50xa_halt },			// set system to HALT

	{ "L",			p50xa_loco },			// Lok {Lok#, [Speed], [FL], [Dir], [F1], [F2], [F3], [F4]}
	{ "LC",			p50xa_locoProtocol },	// Lok Format {Lok#}
	{ "F",			p50xa_functionF1F8 },	// Function {Lok#, [F1], [F2], [F3], [F4], [F5], [F6], [F7], [F8]}
	{ "FX",			p50xa_functionF9F16 },	// X-Function {Lok#, [F9], [F10], [F11], [F12], [F13], [F14], [F15], [F16]}
	{ "FM",			NULL },					// Function mapping {Lok# {[Fn][,map]} {[...]}}
	{ "T",			p50xa_turnout },		// T {Trnt#, [Color], [Status]}

	{ "Y",			p50xa_status },			// reports status
	{ "V",			p50xa_version },		// report software version and serial number
	{ "MT",			p50xa_magtimer },		// turnout timer (min, max)
	{ "SR",			p50xa_s88autoreset },	// S88 autoreset on / off
	{ "SS",			p50xa_s88readout },		// S88-Modul read out
	{ "SE",			p50xa_s88Modules },		// number of s88 (half-)modules
	{ "@@",			p50xa_sysReset },		// Do a cold reset
	{ "B",			p50xa_baudrate },		// B [baud_rate]
	{ "SO",			p50xa_SO },				// SO SO-adress [SO-value]

	// DCC programming
	{ "CV",			p50xa_prgcv },			// calculate CV17/CV18 to DCC long address
	{ "CA",			p50xa_prgca },			// calculate DCC long address to CV17/CV18
	{ "PD",			p50xa_pgMaintrack },	// programming mobile decoder on main track
	{ "PA",			p50xa_pgMaintrack },	// programming accessory decoder on main track
	{ "PTRR",		p50xa_pgRdProgtrack },	// Read register mode
	{ "PTWR",		p50xa_pgWrProgtrack },	// Write register mode
	{ "PTRP",		p50xa_pgRdProgtrack },	// Read page mode
	{ "PTWP",		p50xa_pgWrProgtrack },	// Write page mode
	{ "PTRD",		p50xa_pgRdProgtrack },	// Read direct mode (byte)
	{ "PTWD",		p50xa_pgWrProgtrack },	// Write direct mode (byte)
	{ "PTRB",		p50xa_pgRdProgtrack },	// Read direct mode (bit)
	{ "PTWB",		p50xa_pgWrProgtrack },	// Write direct mode (bit)
	{ "PTRL",		p50xa_pgRdLongaddr },	// Read long address from CV17/CV18
	{ "PTWL",		p50xa_pgWrLongaddr },	// Write long address to CV17/CV18

	// own extensions
	{ "PX",			p50xa_pgMaintrack },	// Programming extended accessory decoder on main track
	{ "PE",			p50xa_pgMaintrack },	// Programming basic accessory decoder (single channel) on main track
	{ "LS",			p50xa_loccfg },			// add a loco with format and number of supported speedsteps LS Lok# {, [fmt], [steps]}

	{ "LOCDUMP",	p50xa_locdump },		// dump all locos
	{ "LOCADD",		p50xa_locadd },			// add loco to loco DB
	{ "LOCDELETE",	p50xa_locdelete },		// remove single loco from loco DB
	{ "LOCCLEAR",	p50xa_locclear },		// clear loco DB
	{ "TRKDUMP",	NULL },					// dump all tractions
	{ "TRKADD",		NULL },					// add a traction
	{ "TRKCLEAR",	NULL },					// clear all tractions
	{ "MAPDUMP",	NULL },					// dump all function mappings
	{ "CFGDUMP",	p50xa_cfgdump },		// dump all settings (as a backup helper)
	{ "CFGACC",		p50xa_cfgacc },			// configure accessory decoder (CFGACC [modAdr], [fmt]), modAdr = 0 -> turnouts 1..4, fmt = { MM | DCC }
	{ "CFGSYS",		NULL },					// some system configuration

	{ "TPM",		NULL },					// enter Tams Programming Mode
	{ "RC",			p50xa_railcom },		// Switch railcom on or off
	{ "DCCA",		NULL },					// Switch DCCA on or off
	{ "RCR",		p50xa_railcomRead },	// RailCom-Read command
	{ "SWUPDATE",	NULL },					// Enter EasyNet Update Mode
	{ "MFX",		p50xa_m3uid },			// Read m3 MAC and (optionally) set loco address (MFX [adr])
	{ "PM",			p50xa_m3cvwrite },		// Write m3 CV values (PM [adr], [cv], [sub], [val], [repeat]) on main
	{ "PTPM",		p50xa_m3PTcvwrite },	// Write m3 CV values (PM [adr], [cv], [sub], [val], [repeat]) on PT
	{ "MRST",		NULL },					// Master-Reset (Werksreset)
	{ "CVER",		NULL },					// report EasyNet client versions

	{ NULL, NULL }
};

static int p50xa_interpret (struct connection *con)
{
	const struct p50xacmd *ct;
	char *s, *args, *end;

	if (!con || !con->cmd) return XERROR;
	s = con->cmd;

	// let's first check for a proper line ending and replace it with a null byte to deal with a terminated C-string
	end = (char *) &con->data[con->idx];
	while (s < end && *s != '\r' && *s != '\n') s++;
	if (*s != '\r' && *s != '\n') return 0;		// we cannot read anything from this input - try to get more
	*s++ = 0;									// terminate the line with null byte
	while (s < end && isspace(*s)) s++;			// skip all following whitespace (includes any CR/NL charcaters)
	if (s < end) end = s;						// we have interpreted the buffer up to this point

	// now let's check for the command part of the line
	s = con->cmd;
	while (p50xa_isCmdChar(*s)) s++;
	args = s;	// may point to the null byte

	// check and split up the arguments
	memset (con->param, 0, sizeof(con->param));
	con->pcount = p50xa_args(args, con->param);
//	p50xa_debug(con, cmd, args - cmd);

	// now interpret command and arguments
	if (!strcmp(con->cmd, "ZzA0")) {			// special case: we compare case sensitive for switching the interface mode
		con->flags &= ~FLAG_IFEXT;
	} else if (!strcmp(con->cmd, "ZzA1")) {		// special case: we compare case sensitive for switching the interface mode
		con->flags |= FLAG_IFEXT;
	} else if (args > con->cmd) {				// the rest compares case insensitive via the command table (if a command is given at all)
		ct = p50xacmds;
		while (ct->cmd) {
			if (!strncasecmp(con->cmd, ct->cmd, args - con->cmd)) {
				if (ct->func) {
					con->rc = ct->func(con);
					p50xa_error(con->sock, con->rc);
				} else {
					p50xa_error(con->sock, XNOTIMPL);
				}
				break;
			}
			ct++;
		}
		if (!ct->cmd) {
			p50xa_error(con->sock, XERROR);
		}
	}

	// end was set behind the last character we could interpret - so we should drop everything before that from the buffer
	return end - (char *) con->data;
}

/*
 * ===============================================================================================
 * the P50 commands ==============================================================================
 * ===============================================================================================
 */

static void p50_speedfunc (struct connection *con, uint8_t *data)
{
	ldataT *l;
	int adr;
	int speed;

	(void) con;

	speed = data[0] & 0x0F;		// the 4 LSB of the first byte encode the speed
	adr = data[1];				// p50 allows addresses 0 .. 255 only!
	if ((l = loco_call(adr, true)) != NULL) {
		loco_setFunc(adr, 0, data[0] & 0x10);
		if (speed == 0x0F) {		// this encodes a direction change
			speed = (l->speed & 0x80) ^ 0x80;	// reverse direction and set speed to zero
		} else {
			speed = (l->speed & 0x80) | p50x_p50speed(speed, l->loco->fmt);
		}
		rq_setSpeed(adr, speed);
	}
}

static void p50_functions (struct connection *con, uint8_t *data)
{
	int adr;
	uint32_t func;

	(void) con;

	func = data[0] & 0x0F;		// the 4 LSB of the first byte encode the functions F1 - F4
	adr = data[1];				// p50 allows addresses 0 .. 255 only!
	rq_setFuncMasked(adr, func, FUNC_F1_F4);
}

static void p50_turnout (struct connection *con, uint8_t *data)
{
	int adr;

	/* from old master control (if switching a different accessory, turn off the last switched-on turnout):
	if (ifs->last_turnout != NO_TURNOUT_EVENT) {
		mag_switch (ifs->last_turnout, FALSE, chan);
		ifs->last_turnout = NO_TURNOUT_EVENT;
	}
	*/

	(void) con;

	adr = (int) data[1];
	switch (data[0]) {		// only 0x20, 0x21 and 0x22 are defined commands - rest is ignored
		case 0x20:
			trnt_switch(adr, false, false);
			trnt_switch(adr, true, false);
			break;
		case 0x21:
			trnt_switch(adr, false, true);
			break;
		case 0x22:
			trnt_switch(adr, true, true);
			break;
	}
}

static void p50_startstop (struct connection *con, uint8_t *data)
{
	(void) con;

	if (data[0] == 0x60) sig_setMode(TM_GO);
	if (data[0] == 0x61) sig_setMode(TM_STOP);
}

static void p50_s88dumpMulti (struct connection *con, uint8_t *data)
{
	int i, param;
	uint16_t s88data;

	param = data[0] & 0x1F;
	if (!param)	{
		con->flags &= ~FLAG_S88AUTORESET;
	} else {
		for (i = 0; i < param; i++) {
			s88data = htons(con->s88Sum[i]);			// p50 expects the MSB first
			lwip_send(con->sock, &s88data, sizeof(s88data), (i < (param - 1)) ? MSG_MORE : 0);
			if (con->flags & FLAG_S88AUTORESET) con->s88Sum[i] = 0;
		}
	}
}

static void p50_s88dumpSingle (struct connection *con, uint8_t *data)
{
	int param;
	uint16_t s88data;

	param = data[0] & 0x1F;
	if (!param)	{
		con->flags |= FLAG_S88AUTORESET;
	} else {
		s88data = htons(con->s88Sum[param - 1]);			// p50 expects the MSB first
		lwip_send(con->sock, &s88data, sizeof(s88data), 0);
		if (con->flags & FLAG_S88AUTORESET) con->s88Sum[param - 1] = 0;
	}
}

/*
 * Function Table:
 * 0x00 - 0x1F	Speed & Function
 * 0x20 - 0x22	Turnouts (off, straight, branch)
 * 0x40 - 0x4F	states of F1 ... F4
 * 0x60 - 0x61	START / STOP
 * 0x80 - 0x9F	s88 reset "off" OR Dump n s88 units
 * 0xC0 - 0xDF  s88 reset "on" OR Dump s88 unit n only
 */

static const struct p50cmd {
	uint8_t			code;				///< the upper three bits of the first byte make up the function code
	uint8_t			len;				///< the length of this command
	void (*func)(struct connection *con, uint8_t *data);
} p50cmds[] = {
	{ 0x00, 2, p50_speedfunc },
	{ 0x20, 2, p50_turnout },
	{ 0x40, 2, p50_functions },
	{ 0x60, 1, p50_startstop },
	{ 0x80, 1, p50_s88dumpMulti },
	{ 0xC0, 1, p50_s88dumpSingle },
	{ 0, 0, NULL }
};

static int p50_interpret (struct connection *con, uint8_t *data, int len)
{
	const struct p50cmd *cmd;
	int i;

	printf ("%s(): %d bytes\n\t", __func__, len);
	for (i = 0; i < len; i++) {
		printf ("%02x ", data[i]);
	}
	putchar('\n');

	cmd = p50cmds;
	while (cmd->len > 0) {
		if (cmd->code == (data[0] & 0xE0)) {
			if (len < cmd->len) return 0;		// command not yet complete - don't eat any characters
			cmd->func(con, data);
			return cmd->len;
		}
		cmd++;
	}
	return len;	// if no command matched, we discard all received characters
}

static void p50x_tcpHandler (void *pvParameter)
{
	struct connection *con;
	int rc, c_read;
	void *p;

	if ((con = calloc (1, sizeof(*con))) == NULL) {		// if we cannot acquire memory, we have to quit!
		lwip_close((int) pvParameter);
		vTaskDelete(NULL);		// won't return
	}

	con->sock = (int) pvParameter;
	con->tid = xTaskGetCurrentTaskHandle();
	con->flags = FLAG_S88AUTORESET;

	printf ("%s(): Starting with FD=%d\n", __func__, con->sock);
//	event_register(EVENT_FEEDBACK, p50x_eventhandler, con, 0);
	event_register(EVENT_FBNEW, p50x_eventhandler, con, 0);
	event_register(EVENT_SYS_STATUS, p50x_eventhandler, con, 0);
	event_register(EVENT_LOCO_FUNCTION, p50x_eventhandler, con, 0);
	event_register(EVENT_LOCO_SPEED, p50x_eventhandler, con, 0);
	event_register(EVENT_TURNOUT, p50x_eventhandler, con, 0);

	for (;;) {
		if (con->idx >= MAX_CMDLEN) con->idx = 0;		// if we receive an oversized line and cannot interpret it, we must throw away the buffer content
		rc = lwip_read(con->sock, &con->data[con->idx], sizeof(con->data) - con->idx);
		if (rc <= 0) break;
		if (con->idx > 0 && tim_isover(con->timeout)) {
			memmove (con->data, &con->data[con->idx], rc);
			con->idx = rc;
		} else {
			con->idx += rc;
		}
		if (is_p50xa(con)) con->timeout = 0;
		else con->timeout = tim_timeout(200);

		do {
			if (is_p50x(con)) {
				if (is_p50xa(con)) {
					c_read = p50xa_interpret(con);
					if (c_read > 0) {
						p50xa_send(con->sock, "\r]");
					}
				} else {
					c_read = p50xb_interpret(con);
				}
			} else {
				c_read = p50_interpret(con, con->data, con->idx);
			}

			if (c_read == con->idx) {	// everything is read - clear buffer
				con->idx = 0;
			} else if (c_read > 0) {	// only part of buffer was read
				memmove (con->data, &con->data[c_read], con->idx - c_read);
				con->idx -= c_read;
			}
		} while (c_read > 0 && con->idx > 0);
	}

	event_deregister(EVENT_DEREGISTER_ALL, p50x_eventhandler, con);

	// now free the allocated structure and sublists
	mutex_lock(&con->mutex, portMAX_DELAY, __func__);
	lwip_close(con->sock);
	while ((p = con->loco) != NULL) {
		con->loco = con->loco->next;
		free (p);
	}
	while ((p = con->trnt) != NULL) {
		con->trnt = con->trnt->next;
		free (p);
	}
	mutex_unlock(&con->mutex);
	mutex_destroy(&con->mutex);
	free (con);

	printf ("%s(): connection closed\n", __func__);
	vTaskDelete(NULL);
}

#if (P50X_UDP != 0)
static void p50_udpReceiver(void *pvParameter)
{
	struct connection con;
	struct sockaddr from;
	struct sockaddr_in local, *remote;
	socklen_t fromlen;
	int rc;

	(void) pvParameter;

	memset (&con, 0, sizeof(con));

	con.sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = INADDR_ANY;
	local.sin_port = htons (5000);
	local.sin_len = sizeof(local);
	lwip_bind(con.sock, (struct sockaddr *) &local, sizeof(local));

	printf ("%s(): Starting at port 5000 UDP\n", __func__);
	for (;;) {
		rc = lwip_recvfrom(con.sock, con.data, sizeof(con.data) - con.idx, 0, &from, &fromlen);
		if (rc <= 0) break;
		remote = (struct sockaddr_in *) &from;
		printf ("%s(): %d bytes from %s:%d\n", __func__, rc, inet_ntoa(remote->sin_addr.s_addr), ntohs(remote->sin_port));
		con.idx += rc;
		p50xb_interpret(&con);
		con.idx = 0;
	}

	lwip_close(con.sock);
	printf ("%s(): finished\n", __func__);
	vTaskDelete(NULL);
}
#endif

int p50x_start (uint16_t port)
{
#if (P50X_UDP != 0)
	xTaskCreate(p50_udpReceiver, "P50X_UDP", P50X_STACK, NULL, P50X_PRIO, NULL);
#endif
	return tcpsrv_startserver(port, p50x_tcpHandler, P50X_STACK, P50X_PRIO);
}
