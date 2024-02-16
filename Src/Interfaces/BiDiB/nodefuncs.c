/*
 * nodefuncs.c
 *
 *  Created on: 01.08.2021
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
#include "bidib.h"
#include "config.h"

void BDBnf_sendSysMagic (struct bidibnode *n, bidibmsg_t *msg)
{
	uint8_t data[8];
	bidibmsg_t *m;

	(void) msg;

	n->rxmsgnum = n->txmsgnum = 0;
	data[0] = (BIDIB_SYS_MAGIC >> 0) & 0xFF;
	data[1] = (BIDIB_SYS_MAGIC >> 8) & 0xFF;
	m = bidib_genMessage(n, MSG_SYS_MAGIC, 2, data);
	netBDB_postMessages(m);
}

void BDBnf_sendPVersion (struct bidibnode *n, bidibmsg_t *msg)
{
	uint8_t data[8];
	bidibmsg_t *m;

	(void) msg;

	data[0] = n->pversion & 0xFF;
	data[1] = n->pversion >> 8;
	m = bidib_genMessage(n, MSG_SYS_P_VERSION, 2, data);
	netBDB_postMessages(m);
}

void BDBnf_sendUniqueID (struct bidibnode *n, bidibmsg_t *msg)
{
	uint8_t data[8];
	bidibmsg_t *m;

	(void) msg;

	bidib_addUID(data, (n) ? n->uid : NULL);
	m = bidib_genMessage(n, MSG_SYS_UNIQUE_ID, BIDIB_UID_LEN, data);
	netBDB_postMessages(m);
}

void BDBnf_sendVersionInfo (struct bidibnode *n, bidibmsg_t *msg)
{
	uint8_t data[8];
	bidibmsg_t *m;

	(void) msg;

	data[0] = SOFT_VERSION_SUB;
	data[1] = SOFT_VERSION_MINOR;
	data[2] = SOFT_VERSION_MAJOR;
	m = bidib_genMessage(n, MSG_SYS_SW_VERSION, 3, data);
	netBDB_postMessages(m);
}

void BDBnf_sendPong (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;

	if (msg->msg == MSG_LOCAL_PING) {
		m = bidib_genMessage(n, MSG_LOCAL_PONG, 0, NULL);
	} else {
		m = bidib_genMessage(n, MSG_SYS_PONG, 1, msg->data);	// return the pinged byte
	}
	netBDB_postMessages(m);
}

void BDBnf_reportNodetab (struct bidibnode *n, bidibmsg_t *msg)
{
	struct bidibnode *child;
	struct ntab_report *nr;
	bidibmsg_t *m;
	uint8_t cnt;
	int i;

	(void) msg;

	if (!n) return;
	if (n->ntab_rep) free (n->ntab_rep);
	n->ntab_rep = NULL;
	cnt = 1;
	child = n->children;
	while (child) {
		cnt++;
		child = child->next;
	}

	if ((nr = malloc (sizeof(*nr) + cnt * sizeof(*nr->nodes))) == NULL) return;
	nr->nodecount = cnt;
	nr->ntab_version = n->ntab_version;	// we use a copy while reporting, just in case it changes meanwhile
	nr->nodes[0].nodeadr = 0;
	memcpy (nr->nodes[0].uid, n->uid, BIDIB_UID_LEN);
	for (i = 1, child = n->children; i < cnt && child; i++, child = child->next) {
		nr->nodes[i].nodeadr = child->localadr;
		memcpy (nr->nodes[i].uid, child->uid, BIDIB_UID_LEN);
	}
	nr->nodeidx = 0;
	n->ntab_rep = nr;
	if ((m = bidib_genMessage(n, MSG_NODETAB_COUNT, 1, &cnt)) != NULL) {
		netBDB_postMessages(m);
	}
}

void BDBnf_nextNodetab (struct bidibnode *n, bidibmsg_t *msg)
{
	struct ntab_report *nr;
	bidibmsg_t *m;
	uint8_t data[16];

	(void) msg;

	if (!n) return;
	if ((nr = n->ntab_rep) != NULL) {
		data[0] = nr->ntab_version;
		data[1] = nr->nodes[nr->nodeidx].nodeadr;
		memcpy (&data[2], nr->nodes[nr->nodeidx].uid, BIDIB_UID_LEN);
		if ((m = bidib_genMessage(n, MSG_NODETAB, 9, data)) != NULL) {
			netBDB_postMessages(m);
		}
		if (++nr->nodeidx >= nr->nodecount) {
			n->ntab_rep = NULL;
			free (nr);
		}
	} else {
		data[0] = 255;
		if ((m = bidib_genMessage(n, MSG_NODE_NA, 1, data)) != NULL) {
			netBDB_postMessages(m);
		}
	}
}

static bidibmsg_t *BDBnf_readFeature (struct bidibnode *n, int f)
{
	struct nodefeature *ft;
	uint8_t data[2];

	data[0] = f;
	if ((ft = bidib_readFeature(n, f)) != NULL) {
		data[1] = ft->value;
		return bidib_genMessage(n, MSG_FEATURE, 2, data);
	}
	return bidib_genMessage(n, MSG_FEATURE_NA, 1, data);;
}

static bidibmsg_t *BDBnf_readNextFeature (struct bidibnode *n, int idx)
{
	uint8_t data[2];

	if (idx < 0 || idx >= n->featurecount) {
		data[0] = 255;
		return bidib_genMessage(n, MSG_FEATURE_NA, 1, data);
	}
	return BDBnf_readFeature(n, n->features[idx].feature);
}

void BDBnf_getNextFeature (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;

	(void) msg;

	m = BDBnf_readNextFeature(n, n->featureidx++);
	netBDB_postMessages(m);
}

void BDBnf_reportFeatures (struct bidibnode *n, bidibmsg_t *msg)
{
	uint8_t data[2];
	bidibmsg_t *m, **msgpp;

	n->featureidx = 0;
	data[0] = n->featurecount;
	if (msg->datalen > 0 && msg->data[0] == 1) {		// stream all features without the MSG_FEATURE_GETNEXT
		data[1] = 1;
		msgpp = &m;
		*msgpp = bidib_genMessage(n, MSG_FEATURE_COUNT, 2, data);
		while (*msgpp && n->featureidx < n->featurecount) {
			msgpp = &(*msgpp)->next;
			*msgpp = BDBnf_readNextFeature(n, n->featureidx++);
		}
	} else {
		m = bidib_genMessage(n, MSG_FEATURE_COUNT, 1, data);
	}
	netBDB_postMessages(m);
}

void BDBnf_getFeature (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m;

	m = BDBnf_readFeature(n, msg->data[0]);
	netBDB_postMessages(m);
}

#if 0
static bool _BDBnf_setFeature (struct bidibnode *n, int feature, uint8_t val)
{
	int i;

	if (!n) return false;

	for (i = 0; i < n->featurecount; i++) {
		if (n->features[i].feature == feature) {
			n->features[i].value = val;
			return true;
		}
	}
	return false;
}
#endif

/**
 * A generic function to allow writing feature values.
 * Any value is accepted, no additional action is performed.
 *
 * \param n			the node that the feature belongs to - ignored
 * \param nf		the node feature that is to be changed - ignored
 * \param val		the new value for that feature
 * \return			the value that is meant to written to the feature
 */
uint8_t BDBnf_featureWrite (struct bidibnode *n, struct nodefeature *nf, uint8_t val)
{
	(void) n;
	(void) nf;

	return val;
}

/**
 * A generic function to allow writing 'boolean' feature values.
 * Any value is accepted, any non-zero value will be interpreted
 * as 'true' and returned as '1'.
 *
 * \param n			the node that the feature belongs to - ignored
 * \param nf		the node feature that is to be changed - ignored
 * \param val		the new value for that feature
 * \return			0 or 1 as new value for that feature
 */
uint8_t BDBnf_featureWriteBool (struct bidibnode *n, struct nodefeature *nf, uint8_t val)
{
	(void) n;
	(void) nf;

	return (val) ? 1 : 0;
}

void BDBnf_setFeature (struct bidibnode *n, bidibmsg_t *msg)
{
	struct nodefeature *ft;
	uint8_t data[2], f, val;
	bidibmsg_t *m = NULL;

	if (msg->datalen < 2) return;
	data[0] = f = msg->data[0];
	val = msg->data[1];

	if ((ft = bidib_readFeature(n, f)) != NULL) {
		if (ft->setter) {
			ft->value = ft->setter (n, ft, val);
			if (n->flags & NODEFLG_VIRTUAL) bidib_store();
		}
		data[1] = ft->value;
		m = bidib_genMessage(n, MSG_FEATURE, 2, data);
	} else {
		m = bidib_genMessage(n, MSG_FEATURE_NA, 1, data);
	}

	netBDB_postMessages(m);
}

void BDBnf_getString (struct bidibnode *n, bidibmsg_t *msg)
{
	bidibmsg_t *m = NULL;
	uint8_t ns, id;

	if (msg->datalen < 2) {		// data is invalid
		m = bidib_errorMessage(n, BIDIB_ERR_SIZE, 1, &msg->seq);
	} else {
		ns = msg->data[0];
		id = msg->data[1];
		if (ns == 0) {				// generic strings in node
			switch (id) {
				case 0:		// product name
					if (n) m = bidib_string (n, ns, id, n->product);
					break;
				case 1:		// user name (writeable)
					m = bidib_string (n, ns, id, n->user);
					break;
				default:
					m = bidib_string (n, ns, id, NULL);
					break;
			}
//		} else if (ns == 1) {		// debug streams (not implemented - yet ???)
//		} else {					// currently there are no other name spaces defined
		}
		if (!m) m = bidib_string (n, ns, id, NULL);
	}
	if (m) netBDB_postMessages(m);
}

void BDBnf_setString (struct bidibnode *n, bidibmsg_t *msg)
{
	struct sysconf *cfg;
	bidibmsg_t *m;
	uint8_t ns, id;
	int len;

	if (msg->datalen < 3) {		// data is invalid
		m = bidib_errorMessage(n, BIDIB_ERR_SIZE, 1, &msg->seq);
		id = 0;		// to keep compiler happy
	} else {
		ns = msg->data[0];
		id = msg->data[1];
		len = msg->data[2];
		m = NULL;
		if (ns == 0) {				// generic strings in node
			switch (id) {
				case 1:				// user name (the only writeable)
					cfg = cnf_getconfig();
					if (len > MAX_USER_STRING) len = MAX_USER_STRING;
					strncpy (n->user, (char *) &msg->data[3], len);
					n->user[len] = 0;
					m = bidib_string (n, ns, id, n->user);
					if (n->parent == NULL) {
						strcpy (cfg->bidib.user, n->user);
						cnf_triggerStore(__func__);
					}
					bidib_store();
					break;
				default:
					m = bidib_string (n, ns, id, NULL);
					break;
			}
		} else if (ns == 1) {		// debug streams (not implemented - yet ???)
		} else {					// currently there are no other name spaces defined
		}
	}
	if (!m) m = bidib_string (n, ns, id, NULL);
	netBDB_postMessages(m);
}

void BDBnf_getError (struct bidibnode *n, bidibmsg_t *msg)
{
	uint8_t data[2];
	bidibmsg_t *m;

	(void) msg;

	data[0] = n->errcode;
	n->errcode = 0;
	m = bidib_genMessage(n, MSG_SYS_ERROR, 1, data);
	netBDB_postMessages(m);
}

void BDBnf_sysClock (struct bidibnode *n, bidibmsg_t *msg)
{
	(void) msg;
	(void) n;

//	log_msg(LOG_BIDIB, "%s() %s\n", __func__, bidib_formatAdrStack(bidib_getAddress(n)));
}

