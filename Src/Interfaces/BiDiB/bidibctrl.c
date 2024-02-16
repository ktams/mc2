/*
 * bidibctrl.c
 *
 *  Created on: 23.01.2021
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
#include "config.h"
#include "bidib.h"
#include "decoder.h"
#include "events.h"

#define MESSAGE_QUEUE_LENGTH	32		///< how much messages should be buffered
#define MESSAGE_WAITTIME		100		///< a time to wait for new messages arriving; if timed out, check for individual node timeouts

static TaskHandle_t task;
static QueueHandle_t messages;

enum action {
	BDBCTRL_MESSAGE,				///< a BiDiB message was received from BiDiBus
	BDBCTRL_NEWNODE,				///< a new node was discovered by the BiDiBus module
	BDBCTRL_LOSTNODE,				///< a node was lost by the BiDiBus module
	BDBCTRL_BUSERROR,				///< a bus error occured on BiDiBus
	BDBCTRL_STOP,					///< a (local) STOP was commanded - send MSG_BOOST_OFF if we are in controller mode
	BDBCTRL_GO,						///< a (local) GO was commanded - send MSG_BOOST_ON if we are in controller mode
};

struct buserror {
	uint8_t				errcode;	///< the error code (one of the BIDIB_ERR_...)
	uint8_t				adr;
};

struct message {
	union {
		void				*arg;	///< the type of the object pointed to is determined by the action
		bidibmsg_t			*bm;	///< a BiDiB message
		struct bidibnode	*node;	///< a node that was added or lost (only toplevel from BiDiBus)
		struct buserror		*err;	///< information about BiDiBus errors
	};
	enum action	 cmd;				///< the requested action associated with the data
};

/*
 * ===================================================================================
 * constructing and sending of messages
 * ===================================================================================
 */

static void BDBctrl_queryFeatures (struct bidibnode *n)
{
	bidibmsg_t *m;
	uint8_t data;

	data = 1;
	if (n && (m = bidib_genMessage(n, MSG_FEATURE_GETALL, 1, &data))) {
		n->state = NS_READ_FEATURES;
		n->timeout = tim_timeout(100);
		BDBus_sendMessage(m);
	}
}

static void BDBctrl_getFeatureNext (struct bidibnode *n)
{
	bidibmsg_t *m;

	if (n && (m = bidib_genMessage(n, MSG_FEATURE_GETNEXT, 0, NULL))) {
		n->state = NS_READ_FEATURES;
		n->timeout = tim_timeout(100);
		BDBus_sendMessage(m);
	}
}

static void BDBctrl_getPVersion (struct bidibnode *n)
{
	bidibmsg_t *m;

	if (n && (m = bidib_genMessage(n, MSG_SYS_GET_P_VERSION, 0, NULL))) {
		n->state = NS_GET_P_VERSION;
		n->timeout = tim_timeout(100);
		BDBus_sendMessage(m);
	}
}

static void BDBctrl_getProdString (struct bidibnode *n)
{
	bidibmsg_t *m;
	uint8_t data[2];

	data[0] = 0;
	data[1] = 0;
	if (n && (m = bidib_genMessage(n, MSG_STRING_GET, 2, data))) {
		n->state = NS_GET_PRODSTRING;
		n->timeout = tim_timeout(100);
		BDBus_sendMessage(m);
	}
}

static void BDBctrl_getUserName (struct bidibnode *n)
{
	bidibmsg_t *m;
	uint8_t data[2];

	data[0] = 0;
	data[1] = 1;
	if (n && (m = bidib_genMessage(n, MSG_STRING_GET, 2, data))) {
		n->state = NS_GET_USERNAME;
		n->timeout = tim_timeout(100);
		BDBus_sendMessage(m);
	}
}

static void BDBctrl_getSWVersion (struct bidibnode *n)
{
	bidibmsg_t *m;

	if (n && (m = bidib_genMessage(n, MSG_SYS_GET_SW_VERSION, 0, NULL))) {
		n->state = NS_GET_SW_VERSION;
		n->timeout = tim_timeout(100);
		BDBus_sendMessage(m);
	}
}

static void BDBctrl_queryNodetable (struct bidibnode *n)
{
	bidibmsg_t *m;

	if (n && (m = bidib_genMessage(n, MSG_NODETAB_GETALL, 0, NULL))) {
		BDBnode_freeNodeList(n->children);
		n->children = NULL;
		n->state = NS_READ_NTABCOUNT;
		n->timeout = tim_timeout(250);
		BDBus_sendMessage(m);
	}
}

static void BDBctrl_getNtabNext (struct bidibnode *n)
{
	bidibmsg_t *m;

	if (n && (m = bidib_genMessage(n, MSG_NODETAB_GETNEXT, 0, NULL))) {
		n->state = NS_READ_NODETAB;
		n->timeout = tim_timeout(100);
		BDBus_sendMessage(m);
	}
}

static void BDBctrl_sysEnable (struct bidibnode *n)
{
	bidibmsg_t *m;

	if (n && (m = bidib_genMessage(n, MSG_SYS_ENABLE, 0, NULL))) {
		n->state = NS_IDLE;
		n->timeout = tim_timeout(100);
		BDBus_sendMessage(m);
	}
}

/*
 * ===================================================================================
 * Sequencing for node activities
 * ===================================================================================
 */

static void BDBctrl_sequence (struct bidibnode *n)
{
	struct nodefeature *ft;

	n->timeout = 0;
	switch (n->state) {
		case NS_GET_SYSMAGIC:		BDBctrl_getPVersion(n); break;
		case NS_GET_P_VERSION:		BDBctrl_queryFeatures(n); break;
		case NS_READ_FEATURES:
		case NS_AUTOREAD_FEATURES:
			if ((ft = bidib_readFeature(n, FEATURE_STRING_SIZE)) != NULL && ft->value > 0) {
				BDBctrl_getProdString(n);
			} else {
				BDBctrl_getSWVersion(n);
			}
			break;
		case NS_GET_PRODSTRING:		BDBctrl_getUserName(n); break;
		case NS_GET_USERNAME:		BDBctrl_getSWVersion(n); break;
		case NS_GET_SW_VERSION:
			if (n->uid[0] & BIDIB_CLASS_BRIDGE) {	// if we have a bridge/hub we should read the node tabe
				BDBctrl_queryNodetable(n);
			} else {
				BDBctrl_sysEnable(n);
			}
			break;
		case NS_READ_NODETAB:		BDBctrl_sysEnable(n); break;

		default:
			n->state = NS_IDLE;
			break;
	}
}
/*
 * ===================================================================================
 * Handling of received messages
 * ===================================================================================
 */

static void BDBctrl_sysMagic (struct bidibnode *n, bidibmsg_t *m)
{
	uint16_t magic;

	if (m->datalen >= 2) {
		magic = m->data[0] | (m->data[1] << 8);
		log_msg (LOG_BIDIB, "%s(): SYS_MAGIC = 0x%04X\n", __func__, magic);
		if (magic == BIDIB_SYS_MAGIC) {
			BDBctrl_sequence(n);
		} else if (magic == BIDIB_BOOT_MAGIC) {
			n->state = NS_BOOTMODE;
		} else {
			n->state = NS_FAILED;
		}
	}
}

static void BDBctrl_sysPversion (struct bidibnode *n, bidibmsg_t *m)
{
	if (m->datalen >= 2) {
		n->pversion = m->data[0] | (m->data[1] << 8);
		BDBctrl_sequence(n);
	}
}

static void BDBctrl_sysSWversion (struct bidibnode *n, bidibmsg_t *m)
{
	if (m->datalen >= 3) {
		log_msg(LOG_BIDIB,	"%s() [%s] V=%u.%u.%u\n", __func__, bidib_formatAdrStack(bidib_getAddress(n)), m->data[2], m->data[1], m->data[0]);
		BDBctrl_sequence(n);
	}
}

static void BDBctrl_identifyState (struct bidibnode *n, bidibmsg_t *m)
{
	if (m->datalen >= 1) {
		if (m->data[0]) n->flags |= NODEFLG_IDENTIFY;
		else n->flags &= ~NODEFLG_IDENTIFY;
		BDBnode_nodeEvent();
	}
}

static void BDBctrl_nodeTabCount (struct bidibnode *n, bidibmsg_t *m)
{
	if (m->datalen >= 1 && m->data[0] > 0) {	// if nodetab is not ready yet, let it time out to retry query
		n->stateidx = m->data[0];	// number of nodes that we expect a table entry for
		n->ntab_version = 0;		// table version is set with the first reported node
		BDBctrl_getNtabNext(n);
	}
}

static void BDBctrl_nodeTab (struct bidibnode *n, bidibmsg_t *m)
{
	struct bidibnode *bn;

	if (n->state != NS_READ_NODETAB) return;		// this message is ignored because we are not expecting node reporting
	if (m->datalen >= 9) {
		if (!n->ntab_version) n->ntab_version = m->data[0];
		if (n->ntab_version != m->data[0]) {		// the nodetab version changed - restart query
			BDBctrl_queryNodetable(n);
		} else {
			if (n->stateidx > 0) {			// yes, we expect this table entry
				if (m->data[1] != 0) {		// we ignore the node itself (adr 0)
					bn = BDBnode_createNode(&m->data[2], m->data[1]);
					BDBnode_insertNode(n, bn);
					BDBctrl_nodeNew(bn);
				}
				if (--n->stateidx == 0) {	// all nodes read in
					BDBctrl_sequence(n);
				} else {
					BDBctrl_getNtabNext(n);
				}
			}
		}
	}
}

static void BDBctrl_msgNodeNA (struct bidibnode *n, bidibmsg_t *m)
{
	if (m->datalen >= 1) {
		BDBnode_dropNode(BDBnode_lookupChild(n, m->data[0]));
	}
}

static void BDBctrl_msgNodeLost (struct bidibnode *n, bidibmsg_t *m)
{
	bidibmsg_t *msg;

	if (m->datalen >= 9) {
		if ((msg = bidib_genMessage(n, MSG_NODE_CHANGED_ACK, 1, m->data)) != NULL) BDBus_sendMessage(msg);
		if (++n->ntab_version == 0) n->ntab_version = 1;
		if (n->ntab_version == m->data[0]) {		// OK, we are in sync with node's view of the table
			BDBnode_dropNode(BDBnode_lookupChild(n, m->data[1]));
		} else {									// sorry, we must have missed changes
			BDBctrl_queryNodetable(n);
			BDBnode_nodeEvent();
		}
	}
}

static void BDBctrl_msgNodeNew (struct bidibnode *n, bidibmsg_t *m)
{
	bidibmsg_t *msg;
	struct bidibnode *bn;

	if (m->datalen >= 9) {
		if ((msg = bidib_genMessage(n, MSG_NODE_CHANGED_ACK, 1, m->data)) != NULL) BDBus_sendMessage(msg);
		if (++n->ntab_version == 0) n->ntab_version = 1;
		if (n->ntab_version == m->data[0]) {		// OK, we are in sync with node's view of the table
			bn = BDBnode_createNode(&m->data[2], m->data[1]);
			BDBnode_insertNode(n, bn);
			BDBctrl_nodeNew(bn);
		} else {									// sorry, we must have missed changes
			BDBctrl_queryNodetable(n);
			BDBnode_nodeEvent();
		}
	}
}

static void BDBctrl_featureCount (struct bidibnode *n, bidibmsg_t *m)
{
	uint8_t cnt;

	if (n == LOCAL_NODE()) return;		// we never change number of features or free features of our local node
	log_msg (LOG_INFO, "%s() node %s\n", __func__, bidib_formatAdrStack(bidib_getAddress(n)));
	if (m->datalen >= 1) {
		cnt = m->data[0];
		if (n->features) free(n->features);
		n->features = NULL;
		n->featureidx = 0;
		if (cnt > 0) {
			if ((n->features = calloc (cnt, sizeof(*n->features))) != NULL) {
				n->featurecount = cnt;
				if (m->datalen >= 2 && m->data[1] != 0) {
					n->state = NS_AUTOREAD_FEATURES;
				} else {
					BDBctrl_getFeatureNext(n);
				}
			} else {
				n->featurecount = 0;
			}
		} else {
			n->features = NULL;
		}
	}
}

static void BDBctrl_feature (struct bidibnode *n, bidibmsg_t *m)
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
				BDBctrl_sequence(n);
			} else if (n->state != NS_AUTOREAD_FEATURES) {
				BDBctrl_getFeatureNext(n);
			}
		}
	}
}

static void BDBctrl_featureNA (struct bidibnode *n, bidibmsg_t *m)
{
	(void) m;

	n->state = NS_IDLE;		// should probably go to READ NODE TAB
}

static void BDBctrl_string (struct bidibnode *n, bidibmsg_t *m)
{
	uint8_t ns, id, len;

	if (m->datalen >= 3) {
		ns = m->data[0];
		id = m->data[1];
		len = m->data[2];
		if (len > 0) {
			if (ns == 0) {			// standard node strings
				switch (id) {
					case 0:		// product name
						if (len > sizeof(n->product)) len = sizeof(n->product) - 1;
						strncpy(n->product, (char *) &m->data[3], len);
						n->product[len] = 0;
						break;
					case 1:		// user name
						if (len > sizeof(n->user)) len = sizeof(n->user) - 1;
						strncpy(n->user, (char *) &m->data[3], len);
						n->user[len] = 0;
						break;
				}
				BDBctrl_sequence(n);
			} else if (ns == 1) {	// DEBUG stream
			} else {				// not defined by BiDiB (yet)
			}
		}
	}
}

#ifndef CENTRAL_FEEDBACK
/**
 * Updates the external s88 bit array.
 *
 * \param idx			the zero-based feedback input in s88 semantics
 * \param occupy		if true, the feedback module signals an occupied state, else a free state was signaled
 * \return				true, if a change to the previous state was recognised
 */
static bool BDBctrl_bm2s88 (int idx, bool occupy)
{
	volatile uint16_t *input;

	input = s88_getInputs();
	if (occupy && !(input[idx >> 4] & (0x8000 >> (idx & 0x0F)))) {
		input[idx >> 4] |= 0x8000 >> (idx & 0x0F);
		return true;
	}
	if (!occupy && (input[idx >> 4] & (0x8000 >> (idx & 0x0F)))) {
		input[idx >> 4] &= ~(0x8000 >> (idx & 0x0F));
		return true;
	}
	return false;
}
#endif

void BDBctrl_bmOCC (struct bidibnode *n, bidibmsg_t *m)
{
	bidibmsg_t *msg;
	struct feedback_map *fbm;

	if (m->datalen >= 1) {
		if ((fbm = n->private) != NULL) {
#ifdef CENTRAL_FEEDBACK
			fb_BitInput(m->data[0] + fbm->base, true);
#else
			if (BDBctrl_bm2s88(m->data[0] + fbm->base, true)) s88_triggerUpdate();
#endif
		}

		if (   (bidib_opmode() == BIDIB_CONTROLLER)
			&& (msg = bidib_genMessage(n, MSG_BM_MIRROR_OCC, 1, m->data)) != NULL) {
			BDBus_sendMessage(msg);
		}
	}
}

void BDBctrl_bmFREE (struct bidibnode *n, bidibmsg_t *m)
{
	bidibmsg_t *msg;
	struct feedback_map *fbm;

	if (m->datalen >= 1) {
		if ((fbm = n->private) != NULL) {
#ifdef CENTRAL_FEEDBACK
			fb_BitInput(m->data[0] + fbm->base, false);
#else
			if (BDBctrl_bm2s88(m->data[0] + fbm->base, false)) s88_triggerUpdate();
#endif
		}

		if (   (bidib_opmode() == BIDIB_CONTROLLER)
			&& (msg = bidib_genMessage(n, MSG_BM_MIRROR_FREE, 1, m->data)) != NULL) {
				BDBus_sendMessage(msg);
		}
	}
}

void BDBctrl_bmMULTIPLE (struct bidibnode *n, bidibmsg_t *m)
{
	bidibmsg_t *msg;
	struct feedback_map *fbm;

	if (m->datalen >= 3) {
		if ((fbm = n->private) != NULL) {
			if (m->datalen >= m->data[1] / 8 + 2) {
#ifdef CENTRAL_FEEDBACK
				int numfb, bits;

				numfb = bidib_getFeatureValue (n, FEATURE_BM_SIZE);
				bits = m->data[1];
				if ((m->data[0] + bits) > numfb) bits = numfb - m->data[0];		// just in case we have less than 8 valid bits in the last data byte
				fb_rangeInput(m->data[0] + fbm->base, bits, &m->data[2]);
#else
				bool update;
				int i;

				for (i = 0, update = false; i < m->data[1]; i++) {
					update |= BDBctrl_bm2s88(m->data[0] + fbm->base + i, m->data[2 + (i >> 3)] & (1 << (i & 0x07)));
				}
				if (update) s88_triggerUpdate();
#endif
			}
		}

		if (   (bidib_opmode() == BIDIB_CONTROLLER)
			&& (msg = bidib_genMessage(n, MSG_BM_MIRROR_MULTIPLE, m->datalen, m->data)) != NULL) {
			BDBus_sendMessage(msg);
		}
	}
}

static void BDBctrl_errorMessage (struct bidibnode *n, bidibmsg_t *m)
{
	(void) n;

	bidib_debugError(__func__, m);
}

static void BDBctrl_POM_readMessage(struct bidibnode *n, bidibmsg_t *m)
{
	(void) n;
	int adr;
	cvadrT cva;

	adr = m->data[0] + (m->data[1] << 8);
	cva.cv = m->data[2] + (m->data[3] << 8);

	log_msg (LOG_INFO, "%s(): %d %ld = %d\n", __func__, adr, cva.cv, m->data[4]);
	if (m->datalen >= 5) {
		reply_deliver (DECODER_DCC_MOBILE, adr, m->datalen > 5 ? DECODERMSG_XPOM00 : DECODERMSG_POM, cva, fvNULL, m->datalen - 4, &m->data[4]);
	}
}

void BDBctrl_dcca(struct bidibnode *n, bidibmsg_t *m)
{
	log_msg (LOG_INFO, "%s(%s) MNUM %d: opcode %d\n", __func__, bidib_formatAdrStack(bidib_getAddress(n)), m->data[0], m->data[1]);
}

void BDBctrl_accessoryState(struct bidibnode *n, bidibmsg_t *m)
{
	turnoutT *t;
	int anum;

	log_msg (LOG_INFO, "%s() %s ANUM %d -> %d (%s)\n", __func__, bidib_formatUID(n->uid), m->data[0], m->data[1],
		m->data[3] & BIT(7) ? "ERROR" : m->data[3] & BIT(0) ? "ACTIVE" : "DONE");
	if (m->datalen >= 5) {
		anum = m->data[0];
		if (anum > 127) return;	// only ANUM 0..127, ASPECT 0 or 1
		if (m->data[3] & BIT(7)) return;		// this is an error message
		if ((t = db_lookupBidibTurnout(n->uid, anum)) == NULL)	return;		// we don't map this node output
		t->on = !!(m->data[3] & BIT(0));
		t->dir = !!(m->data[1]);
		event_fire(EVENT_TURNOUT, 0, t);
	}
}

void BDBctrl_boosterState(struct bidibnode *n, bidibmsg_t *m)
{
	struct sysconf *cfg;

	cfg = cnf_getconfig();

	(void) n;

	bidib_debugMessages(__func__, m, true);

	if (m->datalen >= 1) {
		switch (m->data[0]) {
			case BIDIB_BST_STATE_OFF_SHORT:
				if (cfg->sysflags & SYSFLAG_GLOBAL_BIDIB_SHORT) sig_setMode(TM_SHORT);
				break;
			case BIDIB_BST_STATE_ON_HERE:
			case BIDIB_BST_STATE_OFF_GO_REQ:
				if (cfg->sysflags & SYSFLAG_BIDIB_ONOFF) sig_setMode(TM_GO);
				break;
			case BIDIB_BST_STATE_OFF_HERE:
			case BIDIB_BST_STATE_ON_STOP_REQ:
				if (cfg->sysflags & SYSFLAG_BIDIB_ONOFF) sig_setMode(TM_STOP);
				break;

			// the following codes are not (yet) handled and finally ignored
//			case BIDIB_BST_STATE_OFF:
//			case BIDIB_BST_STATE_OFF_HOT:
//			case BIDIB_BST_STATE_OFF_NOPOWER:
//			case BIDIB_BST_STATE_OFF_NO_DCC:
//				break;
//			case BIDIB_BST_STATE_ON:
//			case BIDIB_BST_STATE_ON_LIMIT:
//			case BIDIB_BST_STATE_ON_HOT:
//			case BIDIB_BST_STATE_ON_STOP_REQ:
//				break;
		}
	}
}

static const struct msgdecoder upstream[] = {
	{ MSG_SYS_MAGIC,			BDBctrl_sysMagic },
	{ MSG_SYS_P_VERSION,		BDBctrl_sysPversion },
	{ MSG_SYS_SW_VERSION,		BDBctrl_sysSWversion },
	{ MSG_SYS_IDENTIFY_STATE,	BDBctrl_identifyState },
	{ MSG_NODETAB_COUNT,		BDBctrl_nodeTabCount },
	{ MSG_NODETAB,				BDBctrl_nodeTab },
	{ MSG_NODE_NA,				BDBctrl_msgNodeNA },
	{ MSG_NODE_LOST,			BDBctrl_msgNodeLost },
	{ MSG_NODE_NEW,				BDBctrl_msgNodeNew },
	{ MSG_FEATURE_COUNT,		BDBctrl_featureCount },
	{ MSG_FEATURE,				BDBctrl_feature },
	{ MSG_FEATURE_NA,			BDBctrl_featureNA },
	{ MSG_STRING,				BDBctrl_string },
	{ MSG_BM_OCC,				BDBctrl_bmOCC },
	{ MSG_BM_FREE,				BDBctrl_bmFREE },
	{ MSG_BM_MULTIPLE,			BDBctrl_bmMULTIPLE },
	{ MSG_SYS_ERROR,			BDBctrl_errorMessage },
	{ MSG_BM_CV,				BDBctrl_POM_readMessage },
	{ MSG_BM_DCCA,				BDBctrl_dcca },
	{ MSG_ACCESSORY_STATE,		BDBctrl_accessoryState },
	{ MSG_BOOST_STAT,			BDBctrl_boosterState },
	{ 0, NULL }
};

static void BDBctrl_sequenceError (struct bidibnode *n, uint8_t seq)
{
	if (n) {
		log_error("%s() [%s] expected %d but got %d\n", __func__, bidib_formatAdrStack(bidib_getAddress(n)), n->rxmsgnum, seq);
		n->timeout = 0;
		n->rxmsgnum = seq + 1;		// accept this sequence number
		switch (n->state) {
			case NS_AUTOREAD_FEATURES:
			case NS_READ_FEATURES:
				BDBctrl_queryFeatures(n);
				break;
			default:
				break;
		}
	}
}

static void BDBctrl_handleMessage (struct bidibnode *n, bidibmsg_t *m)
{
	const struct msgdecoder *u;

	if (n && m) {
		u = upstream;
		while (u->msg) {
			if (u->msg == m->msg) {
				if (u->handler) u->handler (n, m);
				break;
			}
			u++;
		}
		// don't mess up message sequence counting on our own virtual nodes!
		if ((n->uid[2] != hwinfo->manufacturer) || ((n->uid[3] & 0xF0) != BIDIB_PID_VIRTUAL)) {
			if (m->seq == 0) {			// the node wishes to reset it's sequencing
				n->rxmsgnum = 0;
			}
			if (m->seq != n->rxmsgnum) {				// we lost messages - restart the current action if possible
				BDBctrl_sequenceError(n, m->seq);
			} else {
				if (++n->rxmsgnum == 0) n->rxmsgnum = 1;
			}
		}
	}
}

static void BDBctrl_startNode (struct bidibnode *n)
{
	bidibmsg_t *m;

	if (n) {
		n->rxmsgnum = n->txmsgnum = 0;
		if ((m = bidib_genMessage(n, MSG_SYS_DISABLE, 0, NULL)) != NULL) {
			BDBus_sendMessage(m);
		} else {
			n->state = NS_FAILED;
		}
		if ((m = bidib_genMessage(n, MSG_SYS_GET_MAGIC, 0, NULL)) != NULL) {
			BDBus_sendMessage(m);
			n->state = NS_GET_SYSMAGIC;
			n->timeout = tim_timeout(3000);		// it really may take a long time for the first answer
			n->retry = 0;
		} else {
			n->state = NS_FAILED;
		}
	}
}

static void BDBctrl_nodeTimeout (struct bidibnode *n)
{
	bidibmsg_t *m = NULL;

	if (n && tim_isover(n->timeout)) {
		n->timeout = 0;
		switch (n->state) {
			case NS_GET_SYSMAGIC:
				n->txmsgnum = 0;		// as this is our first real message, we reset the counters
				if (++n->retry > 3) {
					m = bidib_genMessage(n, MSG_SYS_RESET, 0, NULL);
					n->timeout = 100;
					n->retry = 0;
				} else {
					m = bidib_genMessage(n, MSG_SYS_GET_MAGIC, 0, NULL);
					n->timeout = tim_timeout(3000);
				}
				break;
			case NS_AUTOREAD_FEATURES:
			case NS_READ_FEATURES:
				BDBctrl_queryFeatures(n);
				break;
			case NS_READ_NTABCOUNT:
			case NS_READ_NODETAB:
				BDBctrl_queryNodetable(n);
				break;
			default:
				n->timeout = 0;		// invalidate a running timeout
				break;
		}

		if (m) {
			BDBus_sendMessage(m);
		}
	}
}

void BDBctrl_controller (void *pvParameter)
{
	struct message msg;
	struct bidibnode *n;
	struct bidibmsg *m;
	uint8_t data[8];
	char *s;

	(void) pvParameter;

	if (!messages) {
		if ((messages = xQueueCreate(MESSAGE_QUEUE_LENGTH, sizeof(struct message))) == NULL) {
			log_error("%s(): cannot create queue - FATAL\n", __func__);
			vTaskDelete(NULL);
		}
	}

	task = xTaskGetCurrentTaskHandle();
	log_msg(LOG_INFO, "%s() running\n", __func__);

	for (;;) {
		if (xQueueReceive(messages, &msg, MESSAGE_WAITTIME)) {
			switch (msg.cmd) {
				case BDBCTRL_MESSAGE:
					n = BDBnode_lookupNode(msg.bm->adrstack);
					if (!n) {		// message from unknown node
						log_msg (LOG_BIDIB, "%s(): Message from unknown node %s\n", __func__, bidib_formatAdrStack(msg.bm->adrstack));
						free (msg.bm);
						break;
					}
					if (!bidib_isBroadcast(msg.bm->msg)) {
						BDBctrl_handleMessage(n, msg.bm);
					} else {
						// handle a broadcast message (what could that be?)
					}
					free (msg.bm);
					break;
				case BDBCTRL_NEWNODE:
					log_msg(LOG_BIDIB, "%s(): NEW NODE %s UID %s\n", __func__,
							bidib_formatAdrStack(bidib_getAddress(msg.node)), bidib_formatUID(msg.node->uid));
					BDBctrl_startNode(msg.node);
					break;
				case BDBCTRL_LOSTNODE:
					log_msg(LOG_BIDIB, "%s(): LOST NODE %s UID %s\n", __func__,
							bidib_formatAdrStack(bidib_getAddress(msg.node)), bidib_formatUID(msg.node->uid));
					BDBnode_dropNode(msg.node);
					BDBnode_nodeEvent();
					break;
				case BDBCTRL_BUSERROR:
					switch (msg.err->errcode) {
						case BIDIB_ERR_SUBTIME: s = "BIDIB_ERR_SUBTIME"; break;
						case BIDIB_ERR_SUBCRC: s = "BIDIB_ERR_SUBCRC"; break;
						case BIDIB_ERR_SUBPAKET: s = "BIDIB_ERR_SUBPACKET"; break;
						default: s = "(unknown error)"; break;
					}
					log_msg(LOG_BIDIB, "%s(): ERROR %s node %d\n", __func__, s, msg.err->adr);
					free (msg.err);
					break;
				case BDBCTRL_STOP:
					if (bidib_opmode() == BIDIB_CONTROLLER) {
						// send broadcast MSG_BOOST_OFF
						data[0] = 0;	// send as broadcast
						if ((m = bidib_genMessage(NULL, MSG_BOOST_OFF, 1, data)) != NULL) {
							BDBus_sendMessage(m);
						}
					}
					break;
				case BDBCTRL_GO:
					if (bidib_opmode() == BIDIB_CONTROLLER) {
						// send broadcast MSG_BOOST_ON
						data[0] = 0;	// send as broadcast
						if ((m = bidib_genMessage(NULL, MSG_BOOST_ON, 1, data)) != NULL) {
							BDBus_sendMessage(m);
						}
					}
					break;
			}
		} else {	// a timeout occured: check for node timeouts
			BDBnode_iterate (BDBctrl_nodeTimeout);
		}
	}
}

static bool BDBctrl_queueMessage (enum action cmd, void *arg)
{
	struct message msg;

	msg.cmd = cmd;
	msg.arg = arg;
	if (!messages || xQueueSendToBack(messages, &msg, 20) != pdTRUE) {
		log_error("%s() Could not queue message%s\n", __func__, messages ? "" : " (Queue not yet created)");
		return false;
	}
	return true;
}

void BDBctrl_messageReceived (bidibmsg_t *m)
{
	if (!BDBctrl_queueMessage(BDBCTRL_MESSAGE, m)) free (m);
}

void BDBctrl_nodeNew (struct bidibnode *n)
{
	BDBctrl_queueMessage(BDBCTRL_NEWNODE, n);
}

void BDBctrl_nodeLost (struct bidibnode *n)
{
	BDBctrl_queueMessage(BDBCTRL_LOSTNODE, n);
}

void BDBctrl_busError (uint8_t errcode, uint8_t adr)
{
	struct buserror *err;

	if ((err = malloc(sizeof(*err))) != NULL) {
		err->errcode = errcode;
		err->adr = adr;
		if (!BDBctrl_queueMessage(BDBCTRL_BUSERROR, err)) free (err);
	}
}

void BDBctrl_boosterOff (void)
{
	BDBctrl_queueMessage(BDBCTRL_STOP, LOCAL_NODE());
}

void BDBctrl_boosterOn (void)
{
	BDBctrl_queueMessage(BDBCTRL_GO, LOCAL_NODE());
}

