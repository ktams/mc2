/*
 * netbidib.c
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
#include "rb2.h"
#include "bidib.h"
#include "config.h"
#include "events.h"
#include "lwip/sockets.h"

#define BIDIBSERVER_STACK		2048			///< allocated stack for the netBiDiB server interpreter
#define BIDIBSERVER_PRIO		1				///< the standard priority for the server thread

//#define TRUST_ALWAYS							///< if defined, all connecting clients are trusted

// Flag-Defines for the conninfo flags field
#define CONFLAG_LOGON			0x0001			///< we should try to log onto this client
#define CONFLAG_TRUSTED			0x0002			///< this is a client node we trust

enum bidib_connstate {
	CON_STARTUP = 0,					///< the connection is established on the TCP level but nothing else. This is the starting state.
	CON_NULL,							///< link uninitialized, other node is still unknown
	CON_UNPAIRED,						///< both sides are untrusted or don't know about each other
	CON_THEIR_REQUEST,					///< the other side trusts us, but we probably must wait for the user to acknowledge trust
	CON_MY_REQUEST,						///< we have invited the other party to trust us
	CON_PAIRED,							///< we are paired with the other side
	CON_CONTROL,						///< we are now controlled by the other side
};

struct conninfo {
	struct conninfo		*next;					///< linked list of connection informations
	int					 s;						///< the connected socket
	uint8_t				 uid[BIDIB_UID_LEN];	///< the remote UID we are connected to
	uint8_t				 node;					///< the node number we have got from upper side
	uint16_t			 pversion;				///< the supported protocol version from other side
	enum bidib_connstate state;					///< the connection status we are in
	char				 prodstring[32];		///< the received product string
	char				 userstring[32];		///< the received user string
	uint32_t			 flags;					///< some operational flags
	uint8_t				 packet[256];			///< packet receiver buffer
	int					 rxcount;				///< current fill status of the receiver buffer
};

struct txmessage {
	struct conninfo		*ci;					///< the connection over which the messages are to be transmitted
	bidibmsg_t			*msgs;					///< the message(s) that should be sent
};

static struct conninfo *connections;			///< the list of active connections (no matter what state they have
static volatile struct conninfo *control;		///< connection that currently has the control - no two connection may be in this state at the same time
static SemaphoreHandle_t mutex;					///< a mutex to control access to the list of connections
static QueueHandle_t txpipe;					///< a dedicated TX pipe with struct txmessage elements
static TaskHandle_t task;						///< the server task (can be queried via netBDB_getTask())

/**
 * Send all linked messages over the socket.
 * The messages are packed to a byte buffer and this buffer is then
 * transmitted to the socket. If the currently prepared message has
 * the next pointer set, then the flag MSG_MORE is used to indicate
 * that it may make sense to buffer the contents before transmission.
 *
 * \param s		the socket to use for sending
 * \param m		the linked messages to send
 * \return		the number of bytes transmitted or a negative value on error
 */
static int netBDB_sendMessages (int s, bidibmsg_t *m)
{
	int rc, len, total;
	uint8_t *packet;

	if (s < 0 || !m) return 0;
//	bidib_debugMessages (__func__, m, true);
	if ((packet = malloc (300)) == NULL) return -1;
	total = 0;

	while (m) {
		len = bidib_packMessage(m, packet) - packet;
		if (len > 0) {
			rc = lwip_send(s, packet, len, (m->next) ? MSG_MORE : 0);
			if (rc != len) break;
			total += rc;
		}
		m = m->next;
	}

	free (packet);
	return total;
}

static void netBDB_postMessagesLocal (struct conninfo *ci, bidibmsg_t *msgs)
{
	struct txmessage tx;
	bidibmsg_t *m;

	if (!msgs) return;

	if (!ci) ci = (struct conninfo *) control;

	if (ci && ci->s >= 0 && txpipe) {
		m = msgs;
		while (m) {		// for packets originating from a local server patch the message sequence number
			if (m->msg == MSG_STALL) {
				log_msg(LOG_WARNING, "%s() STALL [%s #%u]\n", __func__, bidib_formatAdrStack(m->adrstack), m->seq);
			}
			bidib_debugSingleMessage(__func__, m, true);
			m = m->next;
		}
		tx.ci = ci;
		tx.msgs = msgs;
		if (xQueueSend(txpipe, &tx, 100) != pdTRUE) bidib_freeMessages(msgs);
	} else {
		bidib_freeMessages(msgs);
	}
}

static void bidib_sendPairingRequest (struct conninfo *ci)
{
	uint8_t data[32], *p;
	bidibmsg_t *m;

	p = data;
	*p++ = BIDIB_LINK_PAIRING_REQUEST;
	p = bidib_addUID(p, NULL);			// own UID
	p = bidib_addUID(p, ci->uid);		// remote UID
	m = bidib_genMessage(NULL, MSG_LOCAL_LINK, p - data, data);
	netBDB_postMessagesLocal(ci, m);
}

static void bidib_sendPairedStatus (struct conninfo *ci, bool paired)
{
	uint8_t data[32], *p;
	bidibmsg_t *m;

	p = data;
	*p++ = (paired) ? BIDIB_LINK_STATUS_PAIRED : BIDIB_LINK_STATUS_UNPAIRED;
	p = bidib_addUID(p, NULL);			// own UID
	p = bidib_addUID(p, ci->uid);		// remote UID
	m = bidib_genMessage(NULL, MSG_LOCAL_LINK, p - data, data);
	netBDB_postMessagesLocal(ci, m);
}

static void bidib_sendLocalLogon (struct conninfo *ci)
{
	uint8_t data[8];
	bidibmsg_t *m;

	bidib_addUID(data, NULL);			// own UID
	m = bidib_genMessage(NULL, MSG_LOCAL_LOGON, BIDIB_UID_LEN, data);
	netBDB_postMessagesLocal(ci, m);
}

static void bidib_sendLocalLogoff (struct conninfo *ci)
{
	uint8_t data[8];
	bidibmsg_t *m;

	bidib_addUID(data, NULL);			// own UID
	m = bidib_genMessage(NULL, MSG_LOCAL_LOGOFF, BIDIB_UID_LEN, data);
	netBDB_postMessagesLocal(ci, m);
}

static void netBDB_sendLogonReject (struct conninfo *ci)
{
	uint8_t data[8];
	bidibmsg_t *m;

	bidib_addUID(data, ci->uid);			// remote UID
	m = bidib_genMessage(NULL, MSG_LOCAL_LOGON_REJECTED, BIDIB_UID_LEN, data);
	netBDB_postMessagesLocal(ci, m);
}

static void netBDB_addClient (int s)
{
	struct conninfo *ci, **cipp;

	log_msg (LOG_BIDIB, "%s() new Client with SOCK=%d\n", __func__, s);
	if ((ci = calloc(1, sizeof(*ci))) != NULL) {
		ci->s = s;
		mutex_lock(&mutex, 20, __func__);
		cipp = &connections;
		while (*cipp) cipp = &(*cipp)->next;
		*cipp = ci;
		mutex_unlock(&mutex);
	}
}

static void netBDB_disconnectClient (struct conninfo *ci)
{
	if (!ci) return;
	if (ci == control) {
		control = NULL;			// we are "uncontrolled" again
		log_msg (LOG_BIDIB, "%s(): CONTROL SESSION terminated\n", __func__);
		bidib_extControl(false);
	}
	if (ci->s >= 0) {
		lwip_close(ci->s);
		log_msg (LOG_BIDIB, "%s() connection to %s (%s/%s) closed\n", __func__, bidib_formatUID(ci->uid), ci->prodstring, ci->userstring);
		ci->s = -1;
	}
	ci->rxcount = 0;
	ci->flags &= ~CONFLAG_LOGON;
	ci->state = CON_STARTUP;
}

static struct conninfo *netBDB_removeClient (struct conninfo *ci)
{
	struct conninfo **cipp;

	if (!ci) return NULL;

	if (ci->s >= 0) log_msg (LOG_BIDIB, "%s() Client with SOCK=%d died\n", __func__, ci->s);
	netBDB_disconnectClient(ci);

	mutex_lock(&mutex, 20, __func__);
	cipp = &connections;
	while (*cipp != NULL && *cipp != ci) cipp = &(*cipp)->next;
	if (*cipp) {
		*cipp = ci->next;
		free (ci);
	} else {
		fprintf (stderr, "%s(): client %p not found\n", __func__, ci);
	}
	mutex_unlock(&mutex);
	return *cipp;
}

static struct conninfo *netBDB_lookupClient (uint8_t *uid)
{
	struct conninfo *ci;

	if (!uid) return NULL;
	mutex_lock(&mutex, 20, __func__);
	ci = connections;
	while (ci && memcmp(ci->uid, uid, sizeof(ci->uid))) ci = ci->next;
	mutex_unlock(&mutex);
	return ci;
}

static struct conninfo *netBDB_checkClient (struct conninfo *ci, uint8_t *uid)
{
	struct conninfo *srch;

	if ((srch = netBDB_lookupClient(uid)) != NULL) {
		netBDB_disconnectClient(srch);
		srch->s = ci->s;
		srch->state = ci->state;
		ci->s = -1;
		netBDB_removeClient(ci);
		ci = srch;
	} else {
		memcpy (ci->uid, uid, sizeof(ci->uid));
	}

//	log_msg (LOG_BIDIB, "%s(): List of clients:\n", __func__);
//	for (srch = connections; srch; srch = srch->next) {
//		log_msg (LOG_BIDIB, "\t%s P %s U %s\n", bidib_formatUID(srch->uid), srch->prodstring, srch->userstring);
//	}
	return ci;
}

static struct conninfo *bidib_interpretLocalLink (struct conninfo *ci, bidibmsg_t *m)
{
	size_t len;

	if (!ci || !m) return ci;

	switch (m->data[0]) {
		case BIDIB_LINK_DESCRIPTOR_UID:
			ci->state = CON_NULL;
			ci->flags &= ~CONFLAG_LOGON;
			ci = netBDB_checkClient(ci, &m->data[1]);
			if (ci->flags & CONFLAG_TRUSTED) {
				ci->state = CON_MY_REQUEST;
				bidib_sendPairedStatus(ci, true);
			} else {
				ci->state = CON_UNPAIRED;
				bidib_sendPairedStatus(ci, false);
			}
			break;
		case BIDIB_LINK_DESCRIPTOR_PROD_STRING:
			len = m->data[1];
			if (len > sizeof(ci->prodstring)) len = sizeof(ci->prodstring) - 1;
			strncpy (ci->prodstring, (char *) &m->data[2], len);
			ci->prodstring[len] = 0;
			break;
		case BIDIB_LINK_DESCRIPTOR_USER_STRING:
			len = m->data[1];
			if (len > sizeof(ci->userstring)) len = sizeof(ci->userstring) - 1;
			strncpy (ci->userstring, (char *) &m->data[2], len);
			ci->userstring[len] = 0;
			break;
		case BIDIB_LINK_DESCRIPTOR_P_VERSION:
			ci->pversion = m->data[1] | m->data[2] << 8;
			break;
		case BIDIB_LINK_STATUS_PAIRED:
			if (ci->state == CON_MY_REQUEST) {
				ci->state = CON_PAIRED;
				if (!(ci->flags & CONFLAG_TRUSTED)) {
					ci->flags |= CONFLAG_TRUSTED;
					bidib_store();
					log_msg (LOG_BIDIB, "%s(): LINK to %s now TRUSTED\n", __func__, bidib_formatUID(ci->uid));
				}
				if (!control) ci->flags |= CONFLAG_LOGON;
				else bidib_sendLocalLogoff(ci);
			} else if (ci->state == CON_PAIRED) {
				log_msg (LOG_BIDIB, "%s(): connection request on LINK to %s\n", __func__, bidib_formatUID(ci->uid));
				if (!control) ci->flags |= CONFLAG_LOGON;
				else bidib_sendLocalLogoff(ci);
			}
			break;
		case BIDIB_LINK_STATUS_UNPAIRED:
			if (ci->state == CON_PAIRED) {
				log_msg (LOG_BIDIB, "%s(): LINK to %s now UNPAIRED\n", __func__, bidib_formatUID(ci->uid));
			}
			ci->state = CON_UNPAIRED;
			if (ci->flags & CONFLAG_TRUSTED) {
				ci->flags &= ~CONFLAG_TRUSTED;
				bidib_store();
			}
			break;
		case BIDIB_LINK_PAIRING_REQUEST:
			log_msg (LOG_BIDIB, "%s() PAIRING REQUEST while state = %d client %strusted\n", __func__, ci->state, ci->flags & CONFLAG_TRUSTED ? "" : "un");
			if (ci->state == CON_PAIRED) {
				bidib_sendPairedStatus(ci, true);
				if (!control) ci->flags |= CONFLAG_LOGON;
				else bidib_sendLocalLogoff(ci);
			} else if (ci->state == CON_MY_REQUEST) {
//				bidib_sendPairingRequest(ci);
				bidib_sendPairedStatus(ci, true);
			} else if (ci->state == CON_UNPAIRED) {
				ci->state = CON_THEIR_REQUEST;		// this is only temporary while waiting for user action - could be ignored
				bidib_sendPairingRequest(ci);
#ifdef TRUST_ALWAYS
				ci->state = CON_MY_REQUEST;
				bidib_sendPairedStatus(ci, true);
#else
				if (key_pairing()) {
					ci->state = CON_MY_REQUEST;
					bidib_sendPairedStatus(ci, true);
				} else {
					bidib_sendPairedStatus(ci, false);
					ci->state = CON_UNPAIRED;
				}
#endif
			}
			break;
	}
	bidib_freeMessages(m);
	return ci;
}

/**
 * Pre-interpret a single message (callee is responsible for splitting multiple messages
 * apart). We must decide what to do depending on the type of message:
 *   - local-link messages are always interpreted here
 *   - the logon, logon-ACK and logon-REJECT messages may change the "controlled" state
 *   - other messages are forwarded down the stack, if we are controlled externally
 *
 * \param ci		the connection that received the message
 * \param m			the message to handle
 */
static void bidib_interpretMessage (struct conninfo *ci, bidibmsg_t *m)
{
	struct bidibnode *root;

	if (!ci || !m) return;

	if (m->msg == MSG_LOCAL_LINK) {				// the MSG_LOCAL_LINK is handled separatly
		ci = bidib_interpretLocalLink(ci, m);
	} else if (ci->state == CON_PAIRED) {
		if (m->msg == MSG_LOCAL_LOGON_ACK) {		// TODO check UID for a match with our own ID
			ci->node = m->data[0];
			ci->flags &= ~CONFLAG_LOGON;
			ci->state = CON_CONTROL;
			bidib_extControl(true);
			control = ci;
			log_msg (LOG_BIDIB, "%s(): LOGON accepted as Node %d\n", __func__, ci->node);
			if ((root = BDBnode_getRoot()) != NULL) root->rxmsgnum = root->txmsgnum = 1;
		} else if (m->msg == MSG_LOCAL_LOGON) {	// someone thinks, we are a client - reject this LOGON-attempt
			netBDB_sendLogonReject (ci);
		} else if (m->msg == MSG_LOCAL_LOGON_REJECTED) {
			ci->flags &= ~CONFLAG_LOGON;
			log_msg (LOG_BIDIB, "%s(): LOGON REJECTED\n", __func__);
		}
		bidib_freeMessages(m);
	} else if (ci->state == CON_CONTROL) {
		if (m->msg == MSG_LOCAL_LOGON_REJECTED) {
			ci->flags &= ~CONFLAG_LOGON;
			control = NULL;
			ci->state = CON_PAIRED;
			bidib_extControl(false);
			bidib_freeMessages(m);
		} else {
			BDBnode_downlink (NULL, m);
		}
	}
}

static void netBDB_announce (void *pvParameter)
{
	struct sockaddr_in dest;
	bidibmsg_t *msg, **msgpp;
	uint8_t buf[64];
	int s = -1;
	int flags, len;

	(void) pvParameter;

	log_msg (LOG_INFO, "%s() started\n", __func__);

	// Creating the destination address
	dest.sin_family = AF_INET;
	dest.sin_addr.s_addr = INADDR_BROADCAST;
	dest.sin_port = htons(BIDIB_PORT);
	dest.sin_len = sizeof(dest);

	// Creating the discovery packet
	msg = NULL;
	msgpp = &msg;
	// --- Protocol signature
	sprintf ((char *) buf, "BiDiB");
	*msgpp = bidib_genMessage(NULL, MSG_LOCAL_PROTOCOL_SIGNATURE, strlen((char *)buf), buf);
	if (*msgpp) msgpp = &(*msgpp)->next;
	// --- link description
	buf[0] = BIDIB_LINK_DESCRIPTOR_UID;			// -----------
	bidib_addUID(&buf[1], NULL);
	buf[8]  = BIDIB_LINK_DESCRIPTOR_P_VERSION;	// -----------
	buf[9]  = (BIDIB_VERSION >> 0) & 0xFF;
	buf[10] = (BIDIB_VERSION >> 8) & 0xFF;
	*msgpp = bidib_genMessage(NULL, MSG_LOCAL_LINK, 11, buf);
	if (*msgpp) msgpp = &(*msgpp)->next;
	// --- TCP announcement
	buf[0] = BIDIB_ANNOUNCEMENT_SERVER_TCP_NODE;
	buf[1] = (BIDIB_PORT >> 8) & 0xFF;
	buf[2] = (BIDIB_PORT >> 0) & 0xFF;
	*msgpp = bidib_genMessage(NULL, MSG_LOCAL_ANNOUNCE, 3, buf);
	if (*msgpp) msgpp = &(*msgpp)->next;

	len = bidib_packAllMessages(msg, buf, sizeof(buf));
	log_msg (LOG_BIDIB, "%s(): Packet size = %d\n", __func__, len);

	for (;;) {
		if (s < 0 && rt.en && netif_is_link_up(rt.en) && netif_is_up(rt.en)) {
			s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (s >= 0) {
				flags = lwip_fcntl(s, F_GETFL, 0);
				lwip_fcntl (s, F_SETFL, flags | O_NONBLOCK);
			}
		}
		if (s >= 0) {
			if (lwip_sendto(s, buf, len, 0, (const struct sockaddr *) &dest, sizeof(dest)) != len) {
				fprintf (stderr, "%s(): FAILED to send announcement\n", __func__);
				lwip_close(s);
				s = -1;
			} else {
//				log_msg (LOG_BIDIB, "%s() announcement sent\n",__func__);
			}
		}
		vTaskDelay(5000);
	}
}

static void bidib_startupMessages (struct conninfo *ci)
{
	bidibmsg_t *msgs, *m;
	struct sysconf *cfg;
	uint8_t packet[256], *p;
	char *sig;

	cfg = cnf_getconfig();

#if 1
	sig = BIDIB_SIGNATURE_TAMS;
#else
	if (hwinfo->manufacturer == DCC_MANUFACTURER_TAMS) sig = BIDIB_SIGNATURE_TAMS;
	else if (hwinfo->manufacturer == DCC_MANUFACTURER_LSD) sig = BIDIB_SIGNATURE_LSD;
	else sig = "BiDiB";
#endif
	m = bidib_genMessage(NULL, MSG_LOCAL_PROTOCOL_SIGNATURE, strlen(sig), (uint8_t *) sig);
	netBDB_postMessagesLocal(ci, m);

	p = packet;
	*p++ = BIDIB_LINK_DESCRIPTOR_UID;
	p = bidib_addUID(p, NULL);
	m = msgs = bidib_genMessage(NULL, MSG_LOCAL_LINK, p - packet, packet);

	p = packet;
	*p++ = BIDIB_LINK_DESCRIPTOR_PROD_STRING;
#if 1
	p = bidib_addString(p, BIDIB_PRODSTR_TAMS, 24);
#else
	if (hwinfo->manufacturer == DCC_MANUFACTURER_TAMS) p = bidib_addString(p, BIDIB_PRODSTR_TAMS, 24);
	else if (hwinfo->manufacturer == DCC_MANUFACTURER_LSD) p = bidib_addString(p, BIDIB_PRODSTRING_LSD, 24);
	else p = bidib_addString(p, "OTHER", 24);
#endif
	m->next = bidib_genMessage(NULL, MSG_LOCAL_LINK, p - packet, packet);
	m = m->next;

	p = packet;
	*p++ = BIDIB_LINK_DESCRIPTOR_USER_STRING;
	p = bidib_addString(p, cfg->bidib.user, 24);
	m->next = bidib_genMessage(NULL, MSG_LOCAL_LINK, p - packet, packet);
	m = m->next;

	packet[0] = BIDIB_LINK_DESCRIPTOR_P_VERSION;
	packet[1] = (BIDIB_VERSION >> 0) & 0xFF;
	packet[2] = (BIDIB_VERSION >> 8) & 0xFF;
	m->next = bidib_genMessage(NULL, MSG_LOCAL_LINK, 3, packet);
	m = m->next;

	netBDB_postMessagesLocal(ci, msgs);
}

static int netBDB_packetLength (uint8_t *buf, int len)
{
	uint8_t *p, *end;

	p = buf;
	end = p + len;
	while ((p + *p + 1) <= end) p += *p + 1;
	return p - buf;
}

static int netBDB_readFromClient (struct conninfo *ci)
{
	bidibmsg_t *msgs, *m;
	int rc, msgtotal;

	if (!ci) return -1;
	rc = lwip_read(ci->s, &ci->packet[ci->rxcount], sizeof(ci->packet) - ci->rxcount);
//	log_msg (LOG_BIDIB, "%s(FD=%d): received %d bytes\n", __func__, ci->s, rc);
	if (rc > 0) {
		ci->rxcount += rc;		// buffer increased
		msgtotal = netBDB_packetLength(ci->packet, ci->rxcount);
		msgs = bidib_unpackMessages(ci->packet, msgtotal, 0);
		ci->rxcount -= msgtotal;		// this amount of data is a leftover - beginning of new packet(s)
		if (ci->rxcount > 0) memmove (ci->packet, &ci->packet[msgtotal], ci->rxcount);
		if (ci->state == CON_STARTUP) {		// check, that the first message is a valid PROTOCOL_SIGNATURE message
			if (   msgs != NULL
				&& msgs->msg == MSG_LOCAL_PROTOCOL_SIGNATURE
				&& msgs->datalen >= 5
				&& !strncmp("BiDiB", (char *) msgs->data, 5)) {
				ci->state = CON_NULL;
				bidib_startupMessages (ci);
			}
			else return -1;						// ... if not, we immedeately close the connection
		}
		bidib_debugMessages(__func__, msgs, false);
		while ((m = msgs) != NULL) {		// handle each message separately
			msgs = msgs->next;
			m->next = NULL;
			bidib_interpretMessage (ci, m);
			if ((ci->state == CON_PAIRED) && (ci->flags & CONFLAG_LOGON)) {
				bidib_sendLocalLogon(ci);
			}
		}

	}
	return rc;
}

/**
 * A Thread that waits for packages to be sent over one of the available connections.
 *
 * It creates a queue where all the requests can be posted using a <code>struct txmessage</code> structure.
 * The function tries to accumulate as much packets as possible as long as the connection information is
 * the same for all packets.
 *
 * \param pvParameter	unused thread parameter
 */
static void netBDB_writer (void *pvParameter)
{
	struct txmessage tx;
	struct conninfo *ci;
	bidibmsg_t *msgs, **msgpp;

	(void) pvParameter;

	if ((txpipe = xQueueCreate(32, sizeof (struct txmessage))) == NULL) {
		fprintf (stderr, "%s() cannot create queue for message piping\n", __func__);
		vTaskDelete (NULL);
	}

	for (;;) {
		if (xQueueReceive(txpipe, &tx, portMAX_DELAY) == pdTRUE) {
			ci = tx.ci;			// save the first connection and it's messages
			msgs = tx.msgs;
			msgpp = &msgs;
			while (xQueuePeek(txpipe, &tx, 0) == pdTRUE && tx.ci == ci) {	// check if the next message addresses the same connection
				if (xQueueReceive(txpipe, &tx, 0) != pdTRUE) break;			// really retrieve the next set of messages - should not fail ...
				while (*msgpp) msgpp = &(*msgpp)->next;
				*msgpp = tx.msgs;
			}
			netBDB_sendMessages(ci->s, msgs);
			bidib_freeMessages(msgs);
		}
	}
	vTaskDelete(NULL);
}

TaskHandle_t netBDB_getTask (void)
{
	return task;
}

static void netBDB_server (void *pvParameter)
{
	struct sysconf *cfg;
	struct conninfo *ci, *cinext;
	struct sockaddr_in client;
	socklen_t size;
	int accept_socket, newsocket, nfds, rc;
	fd_set rfds, efds;

	(void) pvParameter;

	cfg = cnf_getconfig();

	if ((accept_socket = tcp_listenSocket(cfg->bidib.port, 0)) < 0) {
		fprintf (stderr, "%s() Cannot create server socket\n", __func__);
		vTaskDelete(NULL);
	}
	task = xTaskGetCurrentTaskHandle();

	for (;;) {
		FD_ZERO(&rfds);
		FD_ZERO(&efds);
		FD_SET(accept_socket, &rfds);
		FD_SET(accept_socket, &efds);
		nfds = accept_socket + 1;
		ci = connections;
		while (ci) {
			if (ci->s >= 0) {
				FD_SET (ci->s, &rfds);
				FD_SET (ci->s, &efds);
				if (ci->s >= nfds) nfds = ci->s + 1;
			}
			ci = ci->next;
		}
		rc = lwip_select(nfds, &rfds, NULL, &efds, NULL);
		if (rc < 0) break;		// an error condition
		if (rc > 0) {			// at least one fd_set has bits set
			if (FD_ISSET(accept_socket, &efds)) break;	// error in accept socket
			if (FD_ISSET(accept_socket, &rfds)) {		// new connection available
				size = sizeof (client);
				newsocket = lwip_accept(accept_socket, (struct sockaddr *)&client, &size);
				log_msg (LOG_BIDIB, "%s(): ACCEPT socket = %d\n", __func__, newsocket);
				if (newsocket >= 0) netBDB_addClient(newsocket);
			}
			ci = connections;
			while (ci) {
				cinext = ci->next;		// just in case 'ci' is dropped (on connection with a known UID)
				if (ci->s >= 0 && ci->s < nfds) {
					if (FD_ISSET (ci->s, &efds)) {
						netBDB_disconnectClient(ci);
					} else if (ci->s < nfds && FD_ISSET (ci->s, &rfds)) {
						rc = netBDB_readFromClient (ci);
						if (rc <= 0) {
							netBDB_disconnectClient(ci);
						}
					}
				}
				ci = cinext;
			}
		}
	}

	// close all sockets
	while (connections) netBDB_removeClient (connections);
	lwip_close(accept_socket);

	seg_pairing(false);				// just to be sure
	log_msg(LOG_INFO, "%s(): connection(s) closed\n", __func__);
	vTaskDelete(NULL);
}

void netBDB_postMessages (bidibmsg_t *m)
{
	if (bidib_opmode() == BIDIB_SERVER) {
		netBDB_postMessagesLocal(NULL, m);
		BDBsrv_readControls(m);				// sniff information from local or remote foreign manual controls
	} else {
		bidib_freeMessages(m);
	}
}

/**
 * Try to log on to the client with the specified UID.
 *
 * \param uid		the 7 bytes UID of the client to log on to
 * \return			0 for logon triggered, 1 for already logged on, a negative error message on fault
 */
int netBDB_logon (uint8_t *uid)
{
	struct conninfo *ci;
	int rc = 0;

	if (!uid) return -1;			// wrong usage ...

	ci = netBDB_lookupClient(uid);
	if (ci) {
		if (control) {
			if (ci == control) rc = 1;					// we are already logged on to the requested client (i.e. SUCCESS)
			else rc = -2;								// we currently are connected to a different client, log off first!
		} else {
			if (ci->s < 0) rc = -4;						// client currently not connected
			if (ci->state != CON_PAIRED) rc = -5;		// client currently not paired
		}
	} else rc = -3;										// client not found

	if (rc == 0) {		// none of the above checkes failed or showed an already existing connection, so let's logon now
		ci->flags |= CONFLAG_LOGON;
		bidib_sendLocalLogon(ci);
	}

	return rc;
}

/**
 * Log off frmo current connection, if any. It is no fault if we
 * are currently not connected.
 *
 * return		0 for success or a negative error code
 */
int netBDB_logoff (void)
{
	if (!mutex_lock(&mutex, 20, __func__)) return -1;
	if (control) {
		bidib_sendLocalLogoff((struct conninfo *) control);
		control->flags &= ~CONFLAG_LOGON;
		if (control->state == CON_CONTROL) control->state = CON_PAIRED;		// paranoia!
		control = NULL;
	}
	mutex_unlock(&mutex);
	return 0;
}

void netBDB_addTrustedClient (uint8_t *uid, char *product, char *user)
{
	struct conninfo *ci, **cipp;

	if (!uid) return;
	log_msg (LOG_BIDIB, "%s() UID %s, P %s, U %s\n", __func__, bidib_formatUID(uid), (product) ? product : "(no product)", (user) ? user : "(no user)");
	if ((ci = calloc(1, sizeof(*ci))) != NULL) {
		ci->s = -1;
		memcpy (ci->uid, uid, sizeof(ci->uid));
		if (product) strncpy (ci->prodstring, product, MAX_PRODUCT_STRING);
		if (user) strncpy (ci->userstring, user, MAX_USER_STRING);
		ci->flags = CONFLAG_TRUSTED;
		mutex_lock(&mutex, 20, __func__);
		cipp = &connections;
		while (*cipp) cipp = &(*cipp)->next;
		*cipp = ci;
		mutex_unlock(&mutex);
	}
}

void netBDB_genClientStore (struct ini_section **root)
{
	struct ini_section *ini;
	struct conninfo *ci;
	char buf[32];

	mutex_lock(&mutex, 20, __func__);
	ci = connections;
	while (ci) {
		if (ci->flags & CONFLAG_TRUSTED) {	// only PAIRED (i.e. trusted) clients are stored
			sprintf (buf, "CL%02x%02x%02x%02x%02x%02x%02x", ci->uid[0], ci->uid[1], ci->uid[2], ci->uid[3], ci->uid[4], ci->uid[5], ci->uid[6]);
			if ((ini = ini_addSection(root, buf)) == NULL) break;	// ERROR - this may create a memory hole!
			ini_addItem(ini, "product", ci->prodstring);
			ini_addItem(ini, "user", ci->userstring);
		}
		ci = ci->next;
	}

	mutex_unlock(&mutex);
}

void netBDB_start (void)
{
	// create the netBiDiB writer thread
	xTaskCreate(netBDB_writer, "BiDiB-TXPIPE", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	// create the netBiDiB UDP-Announce thread
	xTaskCreate(netBDB_announce, "BiDiBannounce", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	// listen for connections
	xTaskCreate(netBDB_server, "BiDiB-SRV", BIDIBSERVER_STACK, NULL, BIDIBSERVER_PRIO, NULL);
}
