/*
 * udpcomm.c
 *
 *  Created on: 25.09.2022
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
#include "rb2.h"
#include "lwip/sockets.h"
#include "decoder.h"
#include "events.h"
#include "config.h"

#define XBUS_COMMANDS					0x0040

// all basic commands
#define LAN_GET_SERIAL_NUMBER				0x0010
#define LAN_GET_COMMON_SETTINGS				0x0012
#define LAN_GET_CODE						0x0018
#define LAN_GET_HWINFO						0x001A
#define LAN_LOGOFF							0x0030
#define LAN_SET_BROADCATSFLAGS				0x0050
#define LAN_GET_BROADCATSFLAGS				0x0051
#define LAN_GET_LOCOMODE					0x0060
#define LAN_SET_LOCOMODE					0x0061
#define LAN_GET_TURNOUTMODE					0x0070
#define LAN_SET_TURNOUTMODE					0x0071
#define LAN_RMBUS_DATACHANGED				0x0080
#define LAN_RMBUS_GETDATA					0x0081
#define LAN_RMBUS_PROGRAMMODULE				0x0082
#define LAN_SYSTEMSTATE_DATACHANGED			0x0084
#define LAN_SYSTEMSTATE_GETDATA				0x0085
#define LAN_RAILCOM_DATACHANGED				0x0088
#define LAN_RAILCOM_GETDATA					0x0089
#define LAN_LOCONET_Z21_RX					0x00A0
#define LAN_LOCONET_Z21_TX					0x00A1
#define LAN_LOCONET_FROM_LAN				0x00A2
#define LAN_LOCONET_DISPATCH_ADDR			0x00A3
#define LAN_LOCONET_DETECTOR				0x00A4
#define LAN_BOOSTER_SET_POWER				0x00B2
#define LAN_BOOSTER_GET_DESCRIPTION			0x00B8
#define LAN_BOOSTER_SET_DESCRIPTION			0x00B9
#define LAN_BOOSTER_SYSTEMSTATE_DATACHANGED	0x00BA
#define LAN_BOOSTER_SYSTEMSTATE_GETDATA		0x00BB
#define LAN_CAN_DETECTOR					0x00C4
#define LAN_CAN_DEVICE_GET_DESCRIPTION		0x00C8
#define LAN_CAN_DEVICE_SET_DESCRIPTION		0x00C9
#define LAN_CAN_BOOSTER_SYSTEMSTATE_CHGD	0x00CA
#define LAN_CAN_BOOSTER_SET_TRACKPOWER		0x00CB
#define LAN_DECODER_GET_DESCRIPTION			0x00D8
#define LAN_DECODER_SET_DESCRIPTION			0x00D9
#define LAN_DECODER_SYSTEMSTATE_DATACHANGED	0x00DA
#define LAN_DECODER_SYSTEMSTATE_GETDATA		0x00DB
#define LAN_ZLINK_GET_HWINFO				0x00E8

#define FEEDBACK_MODULES			192
#define PURGE_TIMEOUT				(180 * 1000)		///< a 3 minute timeout for purging of clients
#define MAX_SUBSCRIBED_LOCOS		16					///< maximum number of locos that a client can subscribe to
#define PKTPOOL_SIZE				2048				///< we allocate a buffer pool of this size for sending packets
#define PKTPOOL_MAXBUF				128					///< the maximum size of a single buffer
#define PKTPOOL_ALIGN				4					///< the alignment of the buffers
/*
 * "Broadcast" flags for clients
 */
#define BCFLG_GENERIC				0x00000001			///< everything about loco and switch commands, Track power, short cuircit and programming
#define BCFLG_RBUSCHANGE			0x00000002			///< changes on R-Bus are reported (not used - we have no R-Bus)
#define BCFLG_RAILCOMCHANGE			0x00000004			///< changes in railcom data from subscribed locos will be reported
#define BCFLG_SYSSTATE				0x00000100			///< various system state information (including currents and voltages)
#define BCFLG_ALL_LOCOS				0x00010000			///< report all loco changes (produces lots of network traffic)
#define BCFLG_CANBUS_BOOSTER		0x00020000			///< report CAN bus booster messages
#define BCFLG_ALL_RAILCOM			0x00040000			///< report all railcom changes from all locos (produces lots of network traffic)
#define BCFLG_CANBUS_OCCUPY			0x00080000			///< report occupancy information from CAN bus interface
#define BCFLG_LOCONET_GENERIC		0x01000000			///< generic loco net events are forwared (without locos and turnouts)
#define BCFLG_LOCONET_LOCO			0x02000000			///< loco related information from loconet is forwared
#define BCFLG_LOCONET_TURNOUT		0x04000000			///< turnout related information from loconet is forwared
#define BCFLG_LOCONET_OCCUPY		0x08000000			///< occupancy related information from loconet is forwared

// some bitfield defines for various commands
#define purgTimeOff					0x00
#define purgTime1min				0x01
#define purgTime2min				0x02
#define purgTime4min				0x03
#define purgTime8min				0x04
#define purgTime15min				0x05
#define purgTime30min				0x06
#define purgTime60min				0x07

#define csEmergencyStop				0x01				///< Der Nothalt ist eingeschaltet 
#define csTrackVoltageOff			0x02				///< Die Gleisspannung ist abgeschaltet 
#define csShortCircuit				0x04				///< Kurzschluss 
#define csProgrammingModeActive		0x20				///< Der Programmiermodus ist aktiv 

#define cseHighTemperature			0x01				///< zu hohe Temperatur 
#define csePowerLost				0x02				///< zu geringe Eingangsspannung 
#define cseShortCircuitExternal		0x04				///< am externen Booster-Ausgang 
#define cseShortCircuitInternal		0x08				///< am Hauptgleis oder Programmiergleis 
#define cseRCN213					0x20				///< Weichenadressierung gem. RCN213

#define capDCC						0x01				///< beherrscht DCC 
#define capMM						0x02				///< beherrscht MM 
//#define capReserved				0x04				///< reserviert für zukünftige Erweiterungen 
#define capRailCom					0x08				///< RailCom ist aktiviert 
#define capLocoCmds					0x10				///< akzeptiert LAN-Befehle für Lokdecoder 
#define capAccessoryCmds			0x20				///< akzeptiert LAN-Befehle für Zubehördecoder 
#define capDetectorCmds				0x40				///< akzeptiert LAN-Befehle für Belegtmelder  
#define capNeedsUnlockCode			0x80				///< benötigt Freischaltcode (z21start)

static TaskHandle_t z21_task;			///< the task to be notified after communication completion
static uint8_t *pktpool;				///< a buffer to easyly allocate transmission buffers from
static volatile int poolidx;			///< an index in the buffer pool

typedef struct client z21clntT;

struct client {
	z21clntT			*next;			///< singly linked list
	TickType_t			 tout;			///< the time when we should purge a client from the list
	struct sockaddr_in	 saddr;			///< the client IP address
	int					 size;			///< the (valid) size of the socket address
	uint32_t			 subscriptions;	///< "broadcast" subscriptions of that client
	int					 lidx;			///< the index at which new loco subscriptions are stored (wrap around!)
	uint16_t			 loco[MAX_SUBSCRIBED_LOCOS];	///< the currently controlled loco
	uint8_t				 speed[MAX_SUBSCRIBED_LOCOS];	///< the last commanded speed (for some quirks in MM2-27B format)
};

static int sock;

static uint16_t oldFeedback[FEEDBACK_MODULES]; ///< 192 feedback modules

static z21clntT *clients;						///< the list of currently "connected" clients
static SemaphoreHandle_t mutex;				///< a mutex to control access to the list of clients

static const uint8_t speed27[] = {
   0, 7, 10, 15, 19, 24, 29, 33, 38, 43, 47, 52, 57, 61, 66, 71,
   75, 80, 85, 89, 94, 99, 103, 108, 113, 118, 124, 127
};

/* ============================================================================================*/
/* === Helper functions for sending out packets ===============================================*/
/* ============================================================================================*/
static uint8_t xor (uint8_t *d, int len)
{
	uint8_t ui8 = 0;
	int i;

	for (i = 0; i < len; i++) {
		ui8 ^= *d++;
	}
	return ui8;
}

/**
 * Send a packet to the client.
 *
 * \param z			the z21 client device
 * \param pkt		the packet without header (must have been allocated with z21_getPacket())
 * \param pktlen	the packet length excluding the header size
 */
static void z21_sendPacket (z21clntT *z, uint16_t cmd, uint8_t *pkt, uint16_t pktlen)
{
//	char ipv4[32];
	uint8_t *p;

	pkt -= 4;
	pktlen += 4;
	p = pkt;

	*p++ = pktlen & 0xFF;
	*p++ = (pktlen >> 8) & 0xFF;
	*p++ = cmd & 0xFF;
	*p++ = (cmd >> 8) & 0xFF;
//	inet_ntoa_r(z->saddr.sin_addr.s_addr, ipv4, sizeof(ipv4));
//	log_msg (LOG_DEBUG, "%s(%s) pkt %p size %u\n", __func__, ipv4, pkt, pktlen);
	lwip_sendto(sock, pkt, pktlen, 0, (struct sockaddr *)&z->saddr, z->size);
}

/**
 * Send an XBUs packet (header command will be LE 0x0040) to the client.
 *
 * \param z			the z21 client device
 * \param xpkt		the XBus packet without checksum (must have been allocated with z21_getXPacket())
 * \param xpktlen	the XBus packet length excluding the checksum
 */
static void z21_sendXBus (z21clntT *z, uint8_t *xpkt, uint16_t xpktlen)
{
	xpkt[xpktlen] = xor (xpkt, xpktlen);
	z21_sendPacket(z, XBUS_COMMANDS, xpkt, xpktlen + 1);
}

static uint8_t *z21_getPacket (int siz)
{
	uint8_t *p;

	siz += 4;		// room to prepend the four header bytes (cmd and packet size)
	if (siz <= 0 || siz > PKTPOOL_MAXBUF) {
		log_error ("%s() unsupported packet size %d\n", __func__, siz);
		return NULL;
	}

	siz = ((siz + PKTPOOL_ALIGN - 1) / PKTPOOL_ALIGN) * PKTPOOL_ALIGN;
	taskENTER_CRITICAL();
	if ((poolidx + siz) > PKTPOOL_SIZE) poolidx = 0;
	p = &pktpool[poolidx];
	poolidx += siz;
	taskEXIT_CRITICAL();
	return p + 4;			// in the end we can always get back these four bytes to insert the header
}

static uint8_t *z21_getXPacket (int size)
{
	return z21_getPacket(size + 1);		// add one byte for XOR checksum
}

/* ============================================================================================*/
/* === Handling of client list ================================================================*/
/* ============================================================================================*/

/**
 * Purging clients that where not active for a cetain time.
 * This function should be called _before_ any event related
 * transmission and _after_ each client frame handling.
 */
static void z21_purgeRun (void)
{
	z21clntT *z, **zpp;
	char ipaddr[32];

	if (mutex_lock(&mutex, 20, __func__)) {
		zpp = &clients;
		while ((z = *zpp) != NULL) {
			if (tim_isover(z->tout)) {
				inet_ntoa_r(z->saddr.sin_addr.s_addr, ipaddr, sizeof(ipaddr));
				log_msg (LOG_DEBUG, "%s() Purging client @%s:%d\n", __func__, ipaddr, ntohs(z->saddr.sin_port));
				*zpp = z->next;
				free (z);
			} else {
				zpp = &z->next;
			}
		}
		mutex_unlock(&mutex);
	}
}

static void z21_purgeClient (z21clntT *z)
{
	z21clntT **zpp;

	if (!z) return;
	if (mutex_lock(&mutex, 20, __func__)) {
		zpp = &clients;
		while (*zpp != NULL && *zpp != z) zpp = &(*zpp)->next;
		if (*zpp == z) {
			*zpp = z->next;
			free (z);
		}
		mutex_unlock(&mutex);
	}
}

/**
 * Look up a client that matched the given IP address or create a new entry for the
 * list. Can only fail, if memory allocation fails.
 *
 * \param saddr			the IP address of the client to look up
 * \param size			the valid length of the socket address structure
 * \return				a pointer to the found or new allocated client structure
 */
static z21clntT *z21_lookupClient (struct sockaddr_in *saddr, int size)
{
	z21clntT *z;
	char ipaddr[32];

	if (!saddr || size <= 0) return NULL;

	if (mutex_lock(&mutex, 20, __func__)) {
		z = clients;
		while (z) {
			if (size == z->size && !memcmp (&z->saddr, saddr, size)) {
				z->tout = tim_timeout(PURGE_TIMEOUT);
				mutex_unlock(&mutex);
				return z;
			}
			z = z->next;
		}
	}
	if ((z = calloc (1, sizeof(*z))) != NULL) {
		z->tout = tim_timeout(PURGE_TIMEOUT);
		z->size = size;
		memcpy (&z->saddr, saddr, sizeof(z->saddr));
		z->next = clients;
		inet_ntoa_r(z->saddr.sin_addr.s_addr, ipaddr, sizeof(ipaddr));
		log_msg (LOG_DEBUG, "%s() new client @%s:%d\n", __func__, ipaddr, ntohs(z->saddr.sin_port));
		clients = z;
	}
	mutex_unlock(&mutex);
	return z;
}

/**
 * Put a loco into the list of subscribed locos.
 * This list is a kind of FIFO and will replace the first subscribed
 * loco when the 17th loco is subscribed to. Even if that first loco
 * recently was used thruout the last time it will be dropped from
 * the list.
 *
 * Observe the necissity to also subscribe to the events (aka. "broadcasts").
 *
 * \param z			the client that requests a subscription
 * \param adr		the loco's address to subscribe to
 */
static void z21_subscribeLoco (z21clntT *z, uint16_t adr)
{
	int i;

	for (i = 0; i < MAX_SUBSCRIBED_LOCOS; i++) {
		if (z->loco[i] == adr) return;		// this address is already in the scubscription list
	}
	z->loco[z->lidx] = adr;
	if (++z->lidx >= MAX_SUBSCRIBED_LOCOS) z->lidx = 0;
}

/**
 * Check if a control has subrscribed to loco events for the given address.
 *
 * \param z			the client to check for loco subscriptions
 * \param adr		the address of the loco to check for
 * \return			true if the control has a valid subscription, false otherwise
 */
static bool z21_checkLocoSubscription (z21clntT *z, int adr)
{
	int i;

	for (i = 0; i < MAX_SUBSCRIBED_LOCOS; i++) {
		if (z->loco[i] == adr) return true;		// YES, the address is in the subscription list
	}
	return false;		// not in list ...
}

/**
 * Check if a control has subscribed to an event (or one of multiple given events).
 *
 * \param z				the client to check for event subscriptions
 * \param subscription	a list of subscriptions to check for (though it is most useful if you only specify a single flag)
 * \return				true if the control has at least a subscription for one of the given list, false otherwise
 */
static bool z21_checkSubscriptionFlag (z21clntT *z, uint32_t subscription)
{
	return (z->subscriptions & subscription) != 0;		// YES, this client has subscribed to at least one of the given subscriptions
}

/**
 * Iterate over the list of active clients and execute a callback function
 * with each client.
 *
 * \param func			the function to call with every client
 * \param priv			a private function argument for the callback
 */
static void z21_iterate (void (func)(z21clntT *, void *), void *priv)
{
	z21clntT *z;

	if (!func) return;		// no action function to execute - ignore this call

	if (mutex_lock(&mutex, 20, __func__)) {
		z = clients;
		while (z) {
			func(z, priv);
			z = z->next;
		}
		mutex_unlock(&mutex);
	}
}

/**
 * Map a FS27 speed of MM2_27B back to the FS126 coding of the protocol
 *
 * \param fs27speed		the speed coded as FS0 to FS27 without emergency stop code (i.e. value 0 .. 27 inclusive)
 * \return				the resulting FS126 code observing the emergency stop code (i.e. value 0, 2 .. 127 inclusive)
 */
static int z21_mapSpeedTo126 (int fs27speed)
{
	fs27speed &= 0x7F;
	if (fs27speed > DIM(speed27)) return speed27[27];
	return speed27[fs27speed];
}

/**
 * Map a FS126 speed from z21 to MM2_27B FS27 coding
 *
 * \param fs126speed	the speed coded as FS0 to FS126 including emergency stop code (i.e. value 0 .. 127 inclusive)
 * \return				the resulting FS27 code without the emergency stop code (i.e. value 0 .. 27 inclusive)
 */
static int z21_mapSpeedTo27 (int fs126speed)
{
	fs126speed &= 0x7F;
	if (fs126speed <= 1) return 0;
	return (fs126speed * 100) / 477 + 1;
}

/* ===================================================================================================*/
/* === Some functions that send generic packets either as answers or brodcasts in case of an event ===*/
/* ===================================================================================================*/

static void _z21_lanXmessage (z21clntT *z, uint8_t xhd, uint8_t db0)
{
	uint8_t *xpkt, *p;

	p = xpkt = z21_getXPacket(2);
	*p++ = xhd;
	*p++ = db0;
	z21_sendXBus(z, xpkt, p - xpkt);
}

static void z21_lanXStopMsg (z21clntT *z)
{
	_z21_lanXmessage(z, 0x81, 0x00);
}

static void z21_lanXStatusMsg (z21clntT *z, int mode)
{
	_z21_lanXmessage(z, 0x61, mode & 0xFF);
}

static void z21_lanXLocoInfo (z21clntT *z, ldataT *l)
{
	uint8_t *xpkt, *p, speed;
	bool fwd;
	int i;

	speed = l->speed & 0x7F;
	fwd = !!(l->speed & 0x80);

	p = xpkt = z21_getXPacket(16);
	*p++ = 0xEF;
	*p++ = (uint8_t) (l->loco->adr >> 8);	// Adresse
	*p++ = l->loco->adr & 0xFF;
	switch (l->loco->fmt) {
		case FMT_MM1_14:
		case FMT_DCC_14:
			if (speed) speed++;		// skip emergency stop
			*p++ = 0;
			break;
		case FMT_DCC_28:
			if (speed) speed += 3;	// skip emergency stop
			speed = ((speed >> 1) & 0x0F) | ((speed << 4) & 0x10);		// move LSB of speed to V5 (bit position 4)
			*p++ = 2;
			break;
		case FMT_MM2_14:
			if (speed) speed++;		// skip emergency stop
			*p++ = 2;
			break;
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			for (i = 0; i < DIM(z->speed); i++) {
				if (z->loco[i] == l->loco->adr) {
					if (z21_mapSpeedTo27(z->speed[i]) == speed) {
//						log_msg (LOG_INFO, "%s() LOCO %d remembered speed = %d (matched)\n", __func__, z->loco[i], z->speed[i]);
						speed = z->speed[i];
					} else {
//						log_msg (LOG_INFO, "%s() LOCO %d remembered speed = %d (NOT matched)\n", __func__, z->loco[i], z->speed[i]);
						speed = z21_mapSpeedTo126(speed);
					}
					break;
				}
			}
			if (i >= DIM(z->speed)) speed = z21_mapSpeedTo126(speed);
			*p++ = 4;
			break;
		case FMT_DCC_126:
		case FMT_DCC_SDF:
		case FMT_M3_126:
			if (speed) speed++;		// skip emergency stop
			*p++ = 4;
			break;
		default:
			return;
	}
	*p++ = (fwd) ? 0x80 | speed : speed;
//	log_msg (LOG_INFO, "%s() Adr %d FMT %d/%d Speed = 0x%02x (%d %s)\n", __func__, l->loco->adr, xpkt[3],
//		l->loco->fmt, xpkt[4], l->speed & 0x7F, (l->speed & 0x80 ? "FWD" : "REV"));

	*p++ = ((l->funcs[0] >> 1) & 0x0F) | ((l->funcs[0] << 4) & 0x10);	// F0, F4 - F1 (MSB -> LSB)
	*p++ = (l->funcs[0] >>  5) & 0xFF;	//  F5 bis F12
	*p++ = (l->funcs[0] >> 13) & 0xFF;	// F13 bis F20
	*p++ = (l->funcs[0] >> 21) & 0xFF;	// F21 bis F28
	*p++ = (l->funcs[0] >> 29) & 0x07;	// F29 bis F31
	z21_sendXBus(z, xpkt, p - xpkt);
}

void z21_xCvResult (z21clntT *z, int cv, uint8_t val)
{
	uint8_t *xpkt, *p;

	p = xpkt = z21_getXPacket(16);
	*p++ = 0x64;		// LAN_X_CV_RESULT
	*p++ = 0x14;
	*p++ = (cv >> 8) & 0xFF;
	*p++ = cv & 0xFF;
	*p++ = val & 0xFF;
	z21_sendXBus(z, xpkt, p - xpkt);
}

static bool POMread_handler(struct decoder_reply *msg, flexval priv)
{
	z21clntT *z;

	z = (z21clntT *) priv.p;

	if(!msg || !msg->len) {
		log_msg(LOG_DEBUG, "%s: no answer, try again.\n", __func__);
	} else {
		log_msg(LOG_DEBUG, "%s: POM answer: decoder adr.: %d, length: %d, data: %d, %d, %d, %d.....\n", __func__,
				msg->adr, msg->len, msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
		if (msg->mt == DECODERMSG_POM) {
			z21_xCvResult(z, msg->cva.cv, msg->data[0]);
		} else {
			z21_lanXStatusMsg(z, 0x13);
		}
	}
	return false;
}

/* ============================================================================================*/
/* === Iteration functions for several events =================================================*/
/* ============================================================================================*/

/**
 * Iteration function for turnout events.
 *
 * \param z			the client from the iteration
 * \param priv		contains a pointer to the turnout that was switched
 */
static void z21_evtTurnout (z21clntT *z, void *priv)
{
	turnoutT *t;
	uint8_t *xpkt, *p;
	int turnout;
//	if (z21_checkSubscriptionFlag(z, BCFLG_GENERIC | BCFLG_LOCONET_TURNOUT)) {
		t = (turnoutT *) priv;
		turnout = t->adr - 1;
		p = xpkt = z21_getXPacket(4);
		*p++ = 0x43;
		*p++ = (turnout >> 8) & 0xFF;
		*p++ = turnout & 0xFF;
		*p++ = (t->dir) ? 1 : 2;
		z21_sendXBus(z, xpkt, p - xpkt);
//	}
}

static void z21_evtSpeedFunc (z21clntT *z, void *priv)
{
	ldataT *l;

	l = (ldataT *) priv;
	if (   ((z->subscriptions & BCFLG_GENERIC) && z21_checkLocoSubscription(z, l->loco->adr))
		||   z->subscriptions & BCFLG_ALL_LOCOS) {
		z21_lanXLocoInfo(z, l);
	}
}

static void z21_evtFeedback (z21clntT *z, void *priv)
{
	struct s88_status *st;
	uint16_t fbindex, mask;
	uint8_t *pkt, *p;
	int i;

	st = (struct s88_status *) priv;

	if (z->subscriptions & BCFLG_LOCONET_OCCUPY) {
		fbindex = 0;
		for (i = 0; i < st->modcnt; i++) {
			if (oldFeedback[i] ^ st->sum[i]) {	// something changed
				for (mask = 0x8000; mask; mask >>= 1, fbindex++) {
					if ((st->sum[i] ^ oldFeedback[i]) & mask) {
						p = pkt = z21_getPacket(4);
						*p++ = 0x01;
						*p++ = fbindex & 0xFF;		// feedback input No
						*p++ = (fbindex >> 8) & 0xFF;
						*p++ = !!(st->sum[i] & mask);
						z21_sendPacket(z, LAN_LOCONET_DETECTOR, pkt, p - pkt);
					}
				}
				oldFeedback[i] = st->sum[i];
			} else {
				fbindex += 16;
			}
		}
	}
}

static void z21_evt_FBNew (z21clntT *z, void *priv)
{
	fbeventT *fbevt;
	uint8_t *pkt, *p;
	uint16_t mask;
	int fbindex, i;

	fbevt = (fbeventT *) priv;
	fbindex = fbevt->module * 16;
	p = pkt = z21_getPacket(4);
	if (z->subscriptions & BCFLG_LOCONET_OCCUPY) {
		for (mask = 0x8000; mask; mask >>= 1, fbindex++) {
			if ((fbevt->status ^ oldFeedback[fbevt->module]) & mask) {
				p = pkt;
				*p++ = 0x01;
				*p++ = fbindex & 0xFF;
				*p++ = (fbindex >> 8) & 0xFF;
				*p++ = !!(fbevt->status & mask);
				z21_sendPacket(z, LAN_LOCONET_DETECTOR, pkt, p - pkt);	// TODO multiple changes could be packeted into one UDP packet
			}
		}
	}
	if (z->subscriptions & BCFLG_RBUSCHANGE && fbevt->module < 10) {
		p = pkt;
		if (fbevt->module < 5) {	// group #0 (RMBUS #1 .. RMBUS #10)
			*p++ = 0;
			for (i = 0; i < 10; i++) *p++ = fb_msb2lsb8(fb_getHalfModuleState(i));
		} else {					// group #1 (RMBUS #11 .. RMBUS #20)
			*p++ = 1;
			for (i = 10; i < 20; i++) *p++ = fb_msb2lsb8(fb_getHalfModuleState(i));
		}
		z21_sendPacket(z, LAN_LOCONET_DETECTOR, pkt, p - pkt);
	}
}

static void z21_trackMode (z21clntT *z, void *priv)
{
	eventT *e;

	e = (eventT *) priv;
	switch (e->param) {
		case SYSEVENT_STOP:
		case SYSEVENT_RESET:
		case SYSEVENT_OVERTEMP:
			z21_lanXStatusMsg (z, 0x00);
			break;
		case SYSEVENT_HALT:
			z21_lanXStopMsg(z);
			break;
		case SYSEVENT_GO:
			z21_lanXStatusMsg (z, 0x01);
			break;
		case SYSEVENT_SHORT:
			z21_lanXStatusMsg (z, 0x08);
			break;
		default:
			return;		// do not handle other events ... just say nothing!
	}
}

static bool z21_eventhandler (eventT *e, void *priv)
{
	(void) priv;

	// ATTENTION: as we rely on some broadcast messages informing clients of the result of their own
	// activity, we must not ignore events that were triggered by ourself!
//	if (e->tid == z21_task) return true;				// this is an event we triggered ourself, so don't report back!
	z21_purgeRun();
//	log_msg (LOG_DEBUG, "%s() Evt %d (%s task)\n", __func__, e->ev, (e->tid == z21_task) ? "own" : "other");
	switch (e->ev) {
		case EVENT_SYS_STATUS:
			z21_iterate(z21_trackMode, e);
			break;
		case EVENT_LOCO_SPEED:
		case EVENT_LOCO_FUNCTION:
			z21_iterate(z21_evtSpeedFunc, e->src);
			break;
		case EVENT_TURNOUT:
			z21_iterate(z21_evtTurnout, e->src);
			break;
		case EVENT_FEEDBACK:
			z21_iterate(z21_evtFeedback, e->src);
			break;
		case EVENT_FBNEW:
			z21_iterate(z21_evt_FBNew, e->src);
			break;
		default:
			break;
	}

	return true;
}

static void z21_xGetVersion (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint8_t *xpkt, *p;

	(void) xcmd;
	(void) packet;

	p = xpkt = z21_getXPacket(4);
	*p++ = 0x63;
	*p++ = 0x21;
	*p++ = 0x39;
	*p++ = 0x12;

	z21_sendXBus(z, xpkt, p - xpkt);
}

static void z21_xGetStatus (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint8_t *xpkt, *p;

	(void) xcmd;
	(void) packet;

	p = xpkt = z21_getXPacket(3);
	*p++ = 0x62;
	*p++ = 0x22;
	switch (rt.tm) {				// CentralState
		case TM_HALT:		*p++ = csEmergencyStop; break;
		case TM_STOP:		*p++ = csTrackVoltageOff; break;
		case TM_SHORT:		*p++ = csShortCircuit | csTrackVoltageOff; break;
		case TM_DCCPROG:	*p++ = csProgrammingModeActive; break;
		default:			*p++ = 0; break;
	}

	z21_sendXBus(z, xpkt, p - xpkt);
}

static void z21_xSetTrackPower (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	(void) xcmd;

	if (packet[5] == 0x80) sig_setMode(TM_STOP);
	else sig_setMode(TM_GO);

	if (!z21_checkSubscriptionFlag(z, BCFLG_GENERIC)) {	// not subscribed, so we need to send an answer here directly
		z21_lanXStatusMsg (z, (packet[5] == 0x80) ? 0x00 : 0x01);
	}
}

static void z21_xCvRead (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint8_t *xpkt, *p;
	int cv, rc;

	(void) xcmd;

	cv = (packet[6] << 8) + packet[7];	// CV-Nr
	rc = dccpt_cvReadByte (cv);
	log_msg (LOG_DEBUG, "%s: PT read-> %d\n", __func__, rc);
	if (rc >= 0) {		// OK, we have read a value
		p = xpkt = z21_getXPacket(5);
		*p++ = 0x64;
		*p++ = 0x14;
		*p++ = (cv >> 8) & 0xFF;
		*p++ = cv & 0xFF;
		*p++ = rc & 0xFF;
		z21_sendXBus(z, xpkt, p - xpkt);
	} else {			// Oh, no! An error occured
		z21_lanXStatusMsg (z, (rc == ERR_SHORT) ? 0x12 : 0x13);
	}
}

static void z21_xCvWrite (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint8_t *xpkt, *p;
	int cv, rc;
	uint8_t val;

	(void) xcmd;

	cv = (packet[6] << 8) + packet[7];	// CV-Nr
	val = packet[8];					// new value for the CV
	rc = dccpt_cvWriteByte (cv, val);
	log_msg (LOG_DEBUG, "%s: PT write-> %d\n", __func__, rc);
	if (rc >= 0) {		// OK, success writing a new value
		p = xpkt = z21_getXPacket(5);
		*p++ = 0x64;
		*p++ = 0x14;
		*p++ = (cv >> 8) & 0xFF;
		*p++ = cv & 0xFF;
		*p++ = rc & 0xFF;
		z21_sendXBus(z, xpkt, p - xpkt);
	} else {			// Oh, no! An error occured
		z21_lanXStatusMsg (z, (rc == ERR_SHORT) ? 0x12 : 0x13);
	}
}

static void z21_xGetTurnoutInfo (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint8_t *xpkt, *p;
	turnoutT *t;
	int adr;

	(void) xcmd;

	adr = (packet[5] << 8) + packet[6];	// turnout address
	if ((t = db_getTurnout(adr)) != NULL) {
		p = xpkt = z21_getXPacket(4);
		*p++ = 0x43;
		*p++ = (adr >> 8) & 0xFF;
		*p++ = adr & 0xFF;
		*p++ = t->dir + 1;
		z21_sendXBus(z, xpkt, p - xpkt);
	}
}

static void z21_xSetTurnout (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	int adr;
	bool activate, thrown;

	(void) z;
	(void) xcmd;

	adr = (packet[5] << 8) + packet[6] + 1;	// turnout address
	thrown = !(packet[7] & 0x01);
	activate = !!(packet[7] & 0x08);
	trnt_switch(adr, thrown, activate);
	/* no answer - event reporting only! */
}

static void z21_xSetStop (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	(void) xcmd;
	(void) packet;

	sig_setMode(TM_HALT);
	if (!z21_checkSubscriptionFlag(z, BCFLG_GENERIC)) z21_lanXStopMsg(z);
}

static void z21_xGetLocoInfo (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint16_t adr;
	ldataT *l;

	(void) xcmd;

	adr = ((packet[6] & 0x3F) << 8) | packet[7];
	if ((l = loco_call(adr, true)) == NULL) return;
	z21_subscribeLoco(z, adr);
	z21_lanXLocoInfo(z, l);
}

static void z21_xSetLocoDrive (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint8_t db0, speed;
	ldataT *l;
	int loco, i;
	enum fmt fmt;
	bool fwd, estop;

	(void) z;
	(void) xcmd;

	db0 = packet[5];
	loco = ((packet[6] << 8) + packet[7]) & 0x3FFF;
	speed = packet[8] & 0x7F;
	fwd = !!(packet[8] & 0x80);
	estop = false;
//	log_msg (LOG_INFO, "%s() Adr %d FMT %d Speed = 0x%02x (%d %s)\n", __func__, loco, db0 & 0x0F,
//		packet[8], speed, (packet[8] & 0x80 ? "FWD" : "REV"));

	if ((l = loco_call(loco, true)) == NULL) return;
	fmt = l->loco->fmt;
	switch(db0 & 0x07) {
		case 0:	// 14 FS DCC / MM1
			if (FMT_IS_MM(fmt)) {
				fmt = FMT_MM1_14;
			} else if (FMT_IS_DCC (fmt)) {
				fmt = FMT_DCC_14;
			}
			if (speed == 1) estop = true;
			if (speed > 0) speed--;
			break;
		case 2:	// 28 FS DCC oder MM2 14 FS
			if (FMT_IS_MM(fmt)) {
				fmt = FMT_MM2_14;
				if (speed == 1) estop = true;
				if (speed > 0) speed--;
			} else if (FMT_IS_DCC (fmt)) {
				fmt = FMT_DCC_28;
				speed = ((speed & 0x0F) << 1) | ((speed >> 4) & 1);		// move speed bit V5 to least significant position
				if (speed == 2 || speed == 3) estop = true;
				if (speed <= 3) speed = 0;
				else speed -= 3;
			}
			break;
		case 3:	// 128 FS DCC oder MM2 28 FS
			if (speed == 1) {
				estop = true;
				speed = 0;
			}
			if (FMT_IS_MM(fmt)) {
				fmt = FMT_MM2_27B;
				for (i = 0; i < DIM(z->speed); i++) {
					if (z->loco[i] == loco) {
						z->speed[i] = speed;		// remember last commanded speed
						log_msg (LOG_INFO, "%s() LOCO %d set speed = %d\n", __func__, loco, speed);
						break;
					}
				}
				speed = z21_mapSpeedTo27(speed);
				if (speed > 27) speed = 27;
			} else if (FMT_IS_DCC (fmt)) {
				fmt = FMT_DCC_126;
				if (speed > 0) speed--;
			}
			break;
		default:
			log_msg (LOG_WARNING, "%s(): unknown loco (speed) format %d\n", __func__, db0 & 0x07);
			break;
	}
	if (fmt != l->loco->fmt) db_setLocoFmt (loco, fmt);
	if (estop) {
		rq_emergencyStop(loco);
	} else {
		if (fwd != !!(l->speed & 0x80)) {		// the direction changed - change direction and set speed to zero
			rq_setSpeed(loco, (fwd) ? 0x80 : 0x00);
		} else {
			rq_setSpeed(loco, (fwd) ? speed | 0x80 : speed);
		}
	}
	/* no answer - event reporting only! */
}

static void z21_xSetLocoFunction (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	ldataT *l;
	int loco;
	uint32_t mask, nfunc;

	(void) z;
	(void) xcmd;

	loco = ((packet[6] << 8) + packet[7]) & 0x3FFF;
	if ((l = loco_call(loco, true)) == NULL) return;
	mask = 1 << (packet[8] & 0x3F);
	if (!mask) return;	// cannot handle functions beyond F31

	switch (packet[8] & 0xC0) {
		case 0x00: nfunc = l->funcs[0] & ~mask; break;
		case 0x40: nfunc = l->funcs[0] | mask; break;
		case 0x80: nfunc = l->funcs[0] ^ mask; break;
		default: return;		// illegal request not handled
	}
	rq_setFuncMasked(loco, nfunc, mask);
	/* no answer - event reporting only! */
}

static void z21_xSetLocoFunctionGroup (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	ldataT *l;
	int loco;
	uint8_t grp, func;

	(void) z;
	(void) xcmd;

	loco = ((packet[6] << 8) + packet[7]) & 0x3FFF;
	if ((l = loco_call(loco, true)) == NULL) return;
	grp = packet[8];
	func = packet[9];

	switch (grp) {
		case 0x20:	// F0, F4 - F1
			rq_setFuncMasked(loco, ((func & 0x0F) << 1) | ((func >> 4) & 1), FUNC_F0_F4);
			break;
		case 0x21:	// F8 - F5
			rq_setFuncMasked(loco, (func & 0x0F) << 5, FUNC_F5_F8);
			break;
		case 0x22:	// F12 - F9
			rq_setFuncMasked(loco, (func & 0x0F) << 9, FUNC_F9_F12);
			break;
		case 0x23:	// F20 - F13
			rq_setFuncMasked(loco, (func & 0x0F) << 13, FUNC_F13_F20);
			break;
		case 0x28:	// F28 - F21
			rq_setFuncMasked(loco, func << 21, FUNC_F21_F28);
			break;
		case 0x29:	// F36 - F29
			rq_setFuncMasked(loco, func << 29, FUNC_F29_F31);
			break;
		case 0x2A:	// F44 - F37 (not supported yet)
			break;
		case 0x2B:	// F52 - F45 (not supported yet)
			break;
		case 0x50:	// F60 - F53 (not supported yet)
			break;
		case 0x51:	// F68 - F61 (not supported yet)
			break;
	}
	/* no answer - event reporting only! */
}

static void z21_xSetLoco (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint8_t db0;

	db0 = packet[5];
	if ((db0 & 0xF0) == 0x10) z21_xSetLocoDrive (z, xcmd, packet);
	else if (db0 == 0xF8) z21_xSetLocoFunction (z, xcmd, packet);
	else z21_xSetLocoFunctionGroup(z, xcmd, packet);
}

static void z21_xCvPom (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	flexval fv;
	uint16_t cv;
	uint8_t opt;
	bool acc;
	int adr;

	(void) xcmd;

	adr = ((packet[6] << 8) | packet[7]) & 0x3FFF;
	cv = ((packet[8] << 8) | packet[9]) & 0x03FF;
	opt = packet[8] & 0xFC;
	acc = (packet[5] == 0x31);
	if (rt.tm != TM_GO && rt.tm != TM_HALT) sig_setMode(TM_GO);		// ensure power on track

	switch (opt) {
		case 0xEC:		// LAN_X_CV_POM_WRITE_BYTE / LAN_X_CV_POM_ACCESSORY_WRITE_BYTE
			dccpom_writeByte(adr, (acc) ? DECODER_DCC_ACC : DECODER_DCC_MOBILE, cv, packet[10], NULL, fvNULL);
			break;
		case 0xE8:		// LAN_X_CV_POM_WRITE_BIT / LAN_X_CV_POM_ACCESSORY_WRITE_BIT
			dccpom_writeBit(adr, (acc) ? DECODER_DCC_ACC : DECODER_DCC_MOBILE, cv, packet[10] & 0x07, !!(packet[10] & 0x08), NULL, fvNULL);
			break;
		case 0xE4:		// LAN_X_CV_POM_READ_BYTE / LAN_X_CV_POM_ACCESSORY_READ_BYTE
			fv.p = z;
			dccpom_readByte(adr, (acc) ? DECODER_DCC_ACC : DECODER_DCC_MOBILE, cv, POMread_handler, fv);
			break;
	}
}

static void z21_xGetFWVersion (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	uint8_t *p, *xpkt;

	(void) xcmd;
	(void) packet;

	p = xpkt = z21_getXPacket(4);
	*p++ = 0xF3;
	*p++ = 0x0A;
	*p++ = BCD(SOFT_VERSION_MAJOR);
	*p++ = BCD(SOFT_VERSION_MINOR);
	z21_sendXBus(z, xpkt, p - xpkt);
}

static void z21_xDummy (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	(void) z;
	(void) packet;

	z21_lanXStatusMsg(z, 0x82);
	log_msg (LOG_WARNING, "%s() CMD 0x%02x not implemented\n", __func__, xcmd);
}

static void z21_xDummySub (z21clntT *z, uint16_t xcmd, uint8_t *packet)
{
	(void) z;

	z21_lanXStatusMsg(z, 0x82);
	log_msg (LOG_WARNING, "%s() CMD 0x%02x/%02x not implemented\n", __func__, xcmd, packet[5]);
}

static const struct z21Xdecoder {
	uint8_t		cmd;		///< the one byte basic XBus command
	uint8_t		db0;		///< additional sub-command byte (not for all commands, though)
	bool		chkdb0;		///< if set, db0 contains a sub-command byte, otherwise db0 is already some packet data
	void (*func) (z21clntT *, uint16_t cmd, uint8_t *);
} z21Xcmds[] = {
	{ 0x21, 0x21, true,  z21_xGetVersion },		// LAN_X_GET_VERSION
	{ 0x21, 0x24, true,  z21_xGetStatus },		// LAN_X_GET_STATUS
	{ 0x21, 0x80, true,  z21_xSetTrackPower },	// LAN_X_SET_TRACK_POWER_OFF
	{ 0x21, 0x81, true,  z21_xSetTrackPower },	// LAN_X_SET_TRACK_POWER_ON
	{ 0x22, 0x11, true,  z21_xDummySub },		// LAN_X_DCC_READ_REGISTER
	{ 0x23, 0x11, true,  z21_xCvRead },			// LAN_X_CV_READ
	{ 0x23, 0x12, true,  z21_xDummySub },		// LAN_X_DCC_WRITE_REGISTER
	{ 0x24, 0x12, true,  z21_xCvWrite },		// LAN_X_CV_WRITE
	{ 0x24, 0xFF, true,  z21_xDummySub },		// LAN_X_MM_WRITE_BYTE
	{ 0x43, 0x00, false, z21_xGetTurnoutInfo },	// LAN_X_GET_TURNOUT_INFO
	{ 0x44, 0x00, false, z21_xDummy },			// LAN_X_GET_EXT_ACCESSORY_INFO
	{ 0x53, 0x00, false, z21_xSetTurnout },		// LAN_X_SET_TURNOUT
	{ 0x54, 0x00, false, z21_xDummy },			// LAN_X_SET_EXT_ACCESSORY
	{ 0x80, 0x00, false, z21_xSetStop },		// LAN_X_SET_STOP
	{ 0xE3, 0xF0, true,  z21_xGetLocoInfo },	// LAN_X_GET_LOCO_INFO
	{ 0xE4, 0x00, false, z21_xSetLoco },		// LAN_X_SET_LOCO_DRIVE / LAN_X_SET_LOCO_FUNCTION / LAN_X_SET_LOCO_FUNCTION_GROUP
	{ 0xE5, 0x5F, true,  z21_xDummySub },		// LAN_X_SET_BINARY_STATE
	{ 0xE6, 0x30, true,  z21_xCvPom },			// LAN_X_CV_POM_WRITE_BYTE / LAN_X_CV_POM_WRITE_BIT / LAN_X_CV_POM_READ_BYTE
	{ 0xE6, 0x31, true,  z21_xCvPom },			// LAN_X_CV_POM_ACCESSORY_WRITE_BYTE / LAN_X_CV_POM_ACCESSORY_WRITE_BIT / LAN_X_CV_POM_ACCESSORY_READ_BYTE
	{ 0xF1, 0x0A, true,  z21_xGetFWVersion },	// LAN_X_GET_FIRMWARE_VERSION

	{ 0, 0, false, NULL }						// end of list
};

static void z21_Xbus (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	const struct z21Xdecoder *zxd;
	uint8_t xcmd, db0;

	(void) cmd;

//	if (xor(&packet[4], pktlen - 4) != 0) return;	// XOR including XOR-checksum must yield 0!

	xcmd = packet[4];
	db0 = packet[5];
	zxd = z21Xcmds;
	while (zxd->func != NULL) {
		if (zxd->cmd == xcmd && (!zxd->chkdb0 || zxd->db0 == db0)) {
			zxd->func (z, xcmd, packet);
			break;
		}
		zxd++;
	}
	if (zxd->func == NULL) {
		log_msg (LOG_WARNING, "%s() unknown CMD 0x%04x/0x%02x (len %d)\n", __func__, xcmd, db0, pktlen);
		z21_lanXStatusMsg(z, 0x82);
	}
}

static void z21_getSerialNumber (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;

	(void) cmd;
	(void) packet;
	(void) pktlen;

	p = pkt = z21_getPacket(4);
	*p++ = hwinfo->serial & 0xFF;
	*p++ = (hwinfo->serial >> 8) & 0xFF;
	*p++ = (hwinfo->serial >> 16) & 0xFF;
	*p++ = (hwinfo->serial >> 24) & 0xFF;
	z21_sendPacket(z, LAN_GET_SERIAL_NUMBER, pkt, p - pkt);
//	log_msg (LOG_DEBUG, "%s(): get serial number: %d\n", __func__, hwinfo->serial);
}

static inline uint8_t z21_getPurgeTimeout (void)
{
	struct sysconf *sc;

	sc = cnf_getconfig();
	if (sc->locopurge >= 60) return purgTime60min;
	if (sc->locopurge >= 30) return purgTime30min;
	if (sc->locopurge >= 15) return purgTime15min;
	if (sc->locopurge >= 8) return purgTime8min;
	if (sc->locopurge >= 4) return purgTime4min;
	if (sc->locopurge >= 2) return purgTime2min;
	if (sc->locopurge >= 1) return purgTime1min;
	return purgTimeOff;
}

static void z21_getCommonSettings (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;

	(void) cmd;
	(void) packet;
	(void) pktlen;

	p = pkt = z21_getPacket(10);
	*p++ = cnf_getFMTconfig()->sigflags & SIGFLAG_RAILCOM ? 1 : 0;	// bool, RC an/aus
	*p++ = 0;		// bool, CV29 bei langer Adresse mit setzen
	*p++ = 1;		// KeyStopMode (Taste auf der Z21) bitmask: ToggleEmergencyStop -> 1, EnableStopOnStart -> 2
	*p++ = 3;		// ProgrammingType:  ptNothing -> 0, ptBitOnly -> 1,  ptByteOnly -> 2, both -> 3
	*p++ = 1;		// LocoNet: Stromversorgung ein (das ist nicht die Spannungsversorgung - was heißt das nun wieder?)
	*p++ = 0x80;	// LocoNetFastClockRate: deaktiviert!
	*p++ = 3;		// LocoNetMode: lnmOff -> 0, lnmGatewayOnly -> 1, lnmSlave -> 2, lnmMaster -> 3
	*p++ = 4;		// ExtSettings: AccessoryStartGroup1 -> 4
	*p++ = z21_getPurgeTimeout();
	*p++ = 0;		// reserved

	z21_sendPacket(z, LAN_GET_COMMON_SETTINGS, pkt, p - pkt);
//	log_msg (LOG_DEBUG, "%s(): LAN_GET_COMMON_SETTINGS\n", __func__);
}

static void z21_getCode (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;

	(void) cmd;
	(void) packet;
	(void) pktlen;

	p = pkt = z21_getPacket(2);
	*p++ = 0;					// no features are locked
	z21_sendPacket(z, LAN_GET_CODE, pkt, p - pkt);
//	log_msg (LOG_DEBUG, "%s(): %d\n", __func__, pkt[0]);
}

static void z21_getHWinfo (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;

	(void) cmd;
	(void) packet;
	(void) pktlen;

	p = pkt = z21_getPacket(8);
	*p++ = 0x12;				// HW-Type (0x00000212 = Z21 XL Booster)
	*p++ = 0x02;
	*p++ = 0x00;
	*p++ = 0x00;
	*p++ = SOFT_VERSION_SUB;	// FW-Version
	*p++ = SOFT_VERSION_MINOR;
	*p++ = SOFT_VERSION_MAJOR;
	*p++ = 0x00;
	z21_sendPacket(z, LAN_GET_HWINFO, pkt, p - pkt);
//	log_msg (LOG_DEBUG, "%s()\n", __func__);
}

static void z21_logoff (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	char ipv4[32];

	(void) cmd;
	(void) packet;
	(void) pktlen;

	inet_ntoa_r(z->saddr.sin_addr.s_addr, ipv4, sizeof(ipv4));
	z21_purgeClient(z);
	log_msg (LOG_DEBUG, "%s(%s)\n", __func__, ipv4);
	/* no answer! */
}

static void z21_setBroadcastFlags (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	(void) cmd;
	(void) pktlen;

	z->subscriptions = packet[4] | (packet[5] << 8) | (packet[6] << 16) | (packet[7] << 24);
	/* no answer! */
}

static void z21_getBroadcastFlags (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;

	(void) cmd;
	(void) packet;
	(void) pktlen;

	p = pkt = z21_getPacket(4);
	*p++ = (z->subscriptions >>  0) & 0xFF;
	*p++ = (z->subscriptions >>  8) & 0xFF;
	*p++ = (z->subscriptions >> 16) & 0xFF;
	*p++ = (z->subscriptions >> 24) & 0xFF;
	z21_sendPacket(z, LAN_GET_BROADCATSFLAGS, pkt, p - pkt);
//	log_msg (LOG_DEBUG, "%s()\n", __func__);
}

static void z21_getLocoMode (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;
	locoT *l;
	int adr;

	(void) cmd;
	(void) packet;
	(void) pktlen;

	adr = ((packet[4] << 8) | packet[5]) & 0x3FFF;
	if ((l = db_getLoco(adr, false)) == NULL) l = db_getLoco(0, false);
	p = pkt = z21_getPacket(4);
	*p++ = packet[4];
	*p++ = packet[5];
	if (FMT_IS_MM(l->fmt)) *p++ = 1;	// MM format
	else *p++ = 0;						// everything else is DCC (M3 is handled as DCC)

	z21_sendPacket(z, LAN_GET_LOCOMODE, pkt, p - pkt);
//	log_msg (LOG_DEBUG, "%s()\n", __func__);
}

static void z21_setLocoMode (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	locoT *l;
	int adr;
	enum fmt fmt;
	bool dcc;

	(void) z;
	(void) cmd;
	(void) pktlen;

	adr = ((packet[4] << 8) | packet[5]) & 0x3FFF;
	if ((l = db_getLoco(adr, true)) != NULL) {
		dcc = (adr >= 256 || packet[6] == 0);
		fmt = FMT_UNKNOWN;
		switch (l->fmt) {
			case FMT_MM1_14:
			case FMT_MM2_14:
				if (dcc) fmt = FMT_DCC_14;
				break;
			case FMT_MM2_27A:
			case FMT_MM2_27B:
				if (dcc) fmt = FMT_DCC_28;
				break;
			case FMT_DCC_14:
				if (!dcc) fmt = FMT_MM1_14;
				break;
			case FMT_DCC_28:
				if (!dcc) fmt = FMT_MM2_14;
				break;
			case FMT_DCC_SDF:
			case FMT_DCC_126:
				if (!dcc) fmt = FMT_MM2_27B;
				break;
			default:
				break;
		}
		if (fmt != FMT_UNKNOWN) db_setLocoFmt(adr, fmt);
	}
	/* no answer! */
}

static void z21_getTurnoutMode (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;
	turnoutT *t;
	int adr;
	bool dcc;

	(void) cmd;
	(void) pktlen;

	adr = (packet[4] << 8) | packet[5];
	if (adr >= 256 || (t = db_getTurnout(adr * 4 + 1)) == NULL) dcc = true;
	else if (FMT_IS_DCC(t->fmt)) dcc = 1;
	else dcc = 0;

	p = pkt = z21_getPacket(4);
	*p++ = packet[4];
	*p++ = packet[5];
	*p++ = (dcc) ? 0 : 1;
	z21_sendPacket(z, LAN_GET_TURNOUTMODE, pkt, p - pkt);
//	log_msg (LOG_DEBUG, "%s() decoder #%d (#%d ~ #%d)\n", __func__, adr, adr * 4 + 1, adr * 4 + 4);
}

static void z21_setTurnoutMode (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	int adr, i;
	bool dcc;

	(void) z;
	(void) cmd;
	(void) pktlen;

	adr = (packet[4] << 8) | packet[5];
	dcc = (adr >= 256 || packet[6] == 0);
	for (i = adr * 4 + 1; i <= adr * 4 + 4; i++) {
		db_setTurnoutFmt(i, (dcc) ? TFMT_DCC : TFMT_MM);
	}
//	log_msg (LOG_DEBUG, "%s() decoder #%d (#%d ~ #%d) => %s\n", __func__, adr, adr * 4 + 1, adr * 4 + 4, (dcc) ? "DCC" : "MM");
	/* no answer! */
}

static void z21_rmbusGetdata (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;
	int grp, i;

	(void) cmd;
	(void) pktlen;

	grp = packet[4];
	p = pkt = z21_getPacket(11);
	*p++ = grp;
	for (i = 0; i < 10; i++) {
		*p++ = fb_msb2lsb8(fb_getHalfModuleState(grp * 10 + i));
	}
	z21_sendPacket(z, LAN_RMBUS_DATACHANGED, pkt, p - pkt);
}

static void z21_systemStateGetData (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;
	int v;

	(void) cmd;
	(void) packet;
	(void) pktlen;

	p = pkt = z21_getPacket(16);
	v = an_getTrackCurrent();		// instant current in mA (we use the filtered version here)
	*p++ = v & 0xFF;
	*p++ = (v >> 8) & 0xFF;
	v = an_getProgCurrent(4);		// the programming current in mA filtered with 4 samples
	*p++ = v & 0xFF;
	*p++ = (v >> 8) & 0xFF;
	v = an_getTrackCurrent();		// filtered current in mA (same as instant current from above)
	*p++ = v & 0xFF;
	*p++ = (v >> 8) & 0xFF;
	v = an_getTemperature();		// the internal temperature in °C
	*p++ = v & 0xFF;
	*p++ = (v >> 8) & 0xFF;
	v = an_getSupply();				// the supply voltage from mains adapter in mV
	*p++ = v & 0xFF;
	*p++ = (v >> 8) & 0xFF;
	v = ts_getVoltage() * 100;		// track voltage in mV
	*p++ = v & 0xFF;
	*p++ = (v >> 8) & 0xFF;
	switch (rt.tm) {				// CentralState
		case TM_HALT:		*p++ = csEmergencyStop; break;
		case TM_STOP:		*p++ = csTrackVoltageOff; break;
		case TM_SHORT:		*p++ = csShortCircuit | csTrackVoltageOff; break;
		case TM_DCCPROG:	*p++ = csProgrammingModeActive; break;
		default:			*p++ = 0; break;
	}
	switch(rt.tm) {					// CentralStateEx
		case TM_OVERTTEMP:	*p++ = cseHighTemperature; break;
		case TM_POWERFAIL:	*p++ = csePowerLost; break;
		case TM_SHORT:		*p++ = cseShortCircuitInternal; break;
		default:			*p++ = 0; break;
	}
	*p++ = 0;						// reserved
	*p++ = capDCC | capMM | capRailCom | capLocoCmds | capAccessoryCmds;	// Capabilities

	z21_sendPacket(z, LAN_SYSTEMSTATE_DATACHANGED, pkt, p - pkt);
}

static void z21_loconetDispatch (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	uint8_t *pkt, *p;
	int adr, slot;

	(void) cmd;
	(void) pktlen;

	adr = packet[4] + (packet[5] << 8);	// loco to dispatch
	slot = ln_dispatchLoco (adr);
	p = pkt = z21_getPacket(4);
	*p++ = adr & 0xFF;
	*p++ = (adr >> 8) & 0xFF;
	*p++ = (slot > 0) ? (slot & 0xFF) : 0;
	z21_sendPacket(z, LAN_LOCONET_DISPATCH_ADDR, pkt, p - pkt);
//	log_msg (LOG_DEBUG, "%s(): loco to dispatch: %d => SLOT %d\n", __func__, adr, slot);

}

static void z21_dummy (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	(void) z;
	(void) packet;

	log_msg (LOG_INFO, "%s() CMD 0x%04x (len %d) not implemented\n", __func__, cmd, pktlen);
}

static void z21_notImplemented (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	(void) z;
	(void) packet;

	z21_lanXStatusMsg(z, 0x82);
	log_msg (LOG_INFO, "%s() CMD 0x%04x (len %d) not implemented\n", __func__, cmd, pktlen);
}

static const struct z21decoder {
	uint16_t		cmd;
	void (*func) (z21clntT *, uint16_t cmd, uint8_t *, uint16_t);
} z21cmds[] = {
	{ XBUS_COMMANDS, z21_Xbus },		// X-Bus commands - see table z21Xcmds[]

	{ LAN_GET_SERIAL_NUMBER,			z21_getSerialNumber },
	{ LAN_GET_COMMON_SETTINGS,			z21_getCommonSettings },
	{ LAN_GET_CODE,						z21_getCode },
	{ LAN_GET_HWINFO,					z21_getHWinfo },
	{ LAN_LOGOFF,						z21_logoff },
	{ LAN_SET_BROADCATSFLAGS,			z21_setBroadcastFlags },
	{ LAN_GET_BROADCATSFLAGS,			z21_getBroadcastFlags },
	{ LAN_GET_LOCOMODE,					z21_getLocoMode },
	{ LAN_SET_LOCOMODE,					z21_setLocoMode },
	{ LAN_GET_TURNOUTMODE,				z21_getTurnoutMode },
	{ LAN_SET_TURNOUTMODE,				z21_setTurnoutMode },
	{ LAN_RMBUS_GETDATA,				z21_rmbusGetdata },
	{ LAN_RMBUS_PROGRAMMODULE,			z21_notImplemented },
	{ LAN_SYSTEMSTATE_GETDATA,			z21_systemStateGetData },
	{ LAN_RAILCOM_GETDATA,				z21_dummy },
	{ LAN_LOCONET_FROM_LAN,				z21_dummy },
	{ LAN_LOCONET_DISPATCH_ADDR,		z21_loconetDispatch },
	{ LAN_LOCONET_DETECTOR,				z21_notImplemented },
	{ LAN_CAN_DETECTOR,					z21_notImplemented },
	{ LAN_CAN_DEVICE_GET_DESCRIPTION,	z21_notImplemented },
	{ LAN_CAN_DEVICE_SET_DESCRIPTION,	z21_notImplemented },
	{ LAN_CAN_BOOSTER_SET_TRACKPOWER,	z21_dummy },
	{ LAN_BOOSTER_SET_POWER,			z21_dummy },
	{ LAN_BOOSTER_GET_DESCRIPTION,		z21_dummy },
	{ LAN_BOOSTER_SET_DESCRIPTION,		z21_dummy },
	{ LAN_BOOSTER_SYSTEMSTATE_GETDATA,	z21_dummy },
	{ LAN_DECODER_GET_DESCRIPTION,		z21_dummy },
	{ LAN_DECODER_SET_DESCRIPTION,		z21_dummy },
	{ LAN_DECODER_SYSTEMSTATE_GETDATA,	z21_dummy },
	{ LAN_ZLINK_GET_HWINFO,				z21_dummy },

	{ 0, NULL }							// end of list
};

void z21_request (z21clntT *z, uint16_t cmd, uint8_t *packet, uint16_t pktlen)
{
	const struct z21decoder *zd;

	zd = z21cmds;
	while (zd->func != NULL) {
		if (zd->cmd == cmd) {
			zd->func (z, cmd, packet, pktlen);
			break;
		}
		zd++;
	}
	if (zd->func == NULL) {
		log_msg (LOG_WARNING, "%s() unknown CMD 0x%04x (len %d)\n", __func__, cmd, pktlen);
		z21_lanXStatusMsg(z, 0x82);
	}
}

#if 1		// we currently don't need that debug - byt it may be useful later on ...
static void z21_debugPacket (z21clntT *z, uint8_t *frame, uint16_t cmd, uint16_t pktlen, int len)
{
	(void) z;
	(void) frame;
	(void) cmd;
	(void) pktlen;
	(void) len;
//	log_msg (LOG_WARNING, "%s() CMD 0x%04x (pktlen %d, len %d)\n", __func__, cmd, pktlen, len);
}
#else
static void z21_debugPacket (z21clntT *z, uint8_t *frame, uint16_t cmd, uint16_t pktlen, int len)
{
	char *dump, *s, ipv4[32];
	int i;

	s = dump = tmpbuf(256);
	for (i = 4; i < pktlen; i++) {
		s += sprintf (s, " 0x%02X", frame[i]);
	}
	*s = 0;
	inet_ntoa_r(z->saddr.sin_addr.s_addr, ipv4, sizeof(ipv4));
	log_msg (LOG_DEBUG, "%s(%s) CMD 0x%02x (pktlen %u / %d)%s\n", __func__, ipv4, cmd, pktlen, len, dump);
	vTaskDelay(10);
}
#endif

static void z21_interpret (z21clntT *z, uint8_t *frame, int len)
{
	uint16_t pktlen;
	uint16_t cmd;

	while (len >= 4) {
		pktlen = frame[0] + (frame[1] << 8);
		if (pktlen < 4) break;	// we receive some null packets ...
		cmd = frame[2] + (frame[3] << 8);
		z21_debugPacket(z, frame, cmd, pktlen, len);
		if (len >= pktlen) z21_request(z, cmd, frame, pktlen);
		frame += pktlen;
		len -= pktlen;
	}
}

static int udp_createSocket (int port)
{
	struct sockaddr_in sa;
	int s;

	if ((s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) >= 0) {
		memset (&sa, 0, sizeof(sa));
		sa.sin_addr.s_addr = INADDR_ANY;
		sa.sin_family = AF_INET;
		sa.sin_len = sizeof(sa);
		sa.sin_port = htons(port);
		lwip_bind(s, (struct sockaddr *) &sa, sa.sin_len);
	}
	return s;
}

void z21_service (void *pvParameter)
{
	static uint8_t frame[1500];

	struct sockaddr_in xClient;
	socklen_t xSize;
	z21clntT *z;
	int port;
	ssize_t len;

	port = (int) pvParameter;

	if ((pktpool = malloc (PKTPOOL_SIZE)) == NULL) {
		log_error ("%s(): cannot allocate the packet buffer pool\n", __func__);
		vTaskDelete(NULL);
	}
	if ((sock = udp_createSocket(port)) < 0) {
		log_error ("%s(): cannot create / bind the socket to port %d\n", __func__, port);
		vTaskDelete(NULL);
	}

	log_msg (LOG_DEBUG, "%s(port = %d) starting with socket %d\n", __func__, port, sock);
	z21_task = xTaskGetCurrentTaskHandle();
	event_register(EVENT_SYS_STATUS, z21_eventhandler, NULL, 0);
	event_register(EVENT_LOCO_SPEED, z21_eventhandler, NULL, 0);
	event_register(EVENT_LOCO_FUNCTION, z21_eventhandler, NULL, 0);
	event_register(EVENT_TURNOUT, z21_eventhandler, NULL, 0);
	event_register(EVENT_FEEDBACK, z21_eventhandler, NULL, 0);
	event_register(EVENT_FBNEW, z21_eventhandler, NULL, 0);
	memset (oldFeedback, 0, sizeof(oldFeedback));

	for (;;) {
		xSize = sizeof (xClient);
		len = lwip_recvfrom(sock, frame, sizeof(frame), 0, (struct sockaddr *)&xClient, &xSize);
		if (len > 0) {
			if ((z = z21_lookupClient(&xClient, xSize)) != NULL) {
				z21_interpret(z, frame, len);
				z21_purgeRun();
			}
		}
	}
}
