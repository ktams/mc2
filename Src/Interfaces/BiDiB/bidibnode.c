/*
 * bidibnode.c
 *
 *  Created on: 28.02.2021
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
#include "events.h"
#include "bidib.h"

static SemaphoreHandle_t	 mutex;			///< a mutex for manipulation the list of nodes
static struct bidibnode 	*BDBroot;			///< the root node of the system (i.e. this is the mc²)

// forward declaration for interdepending recursive functions
static void _BDBnode_freeNodeList (struct bidibnode *nodes);

void BDBnode_nodeEvent (void)
{
	event_fire(EVENT_BIDIDEV, 0, BDBroot->children);		// we only report other nodes - never our machine itself!
}

/**
 * Free a single node including its possible children.
 * This may be called recursive from itself.
 * The nodetab mutex should be held while doing that.
 *
 * \param n		the node to be freed
 */
static void BDBnode_freeNode (struct bidibnode *n)
{
	if (n) {
		if (n->children) _BDBnode_freeNodeList(n->children);
		if (n->features) free (n->features);
		if (n->private) free (n->private);
		free (n);
	}
}

/**
 * Free a contigious list of nodes including their children using a
 * recursive approach. The nodetab mutex should be locked before
 * calling this function.
 *
 * \param nodes		a list of nodes to be freed
 */
static void _BDBnode_freeNodeList (struct bidibnode *nodes)
{
	struct bidibnode *tmp;

	while ((tmp = nodes) != NULL) {
		nodes = nodes->next;
		BDBnode_freeNode(tmp);
	}
}

/**
 * Free a contigious list of nodes including their children using a recursive
 * approach. Locks the nodetab mutex before calling _BDBnode_freeNodeList()
 * that does the real work.
 *
 * \param nodes		a list of nodes to be freed
 */
void BDBnode_freeNodeList (struct bidibnode *nodes)
{
	if (mutex_lock(&mutex, 20, __func__)) {
		_BDBnode_freeNodeList(nodes);
		mutex_unlock(&mutex);
	}
}

/**
 * Remove given node from tree. This involves a recursive search for the
 * node and manipulating next-pointers inside the tree.
 *
 * \param n		the node to remove
 */
void BDBnode_dropNode (struct bidibnode *n)
{
	struct bidibnode *parent;
	struct bidibnode **list;

	if (n->parent == NULL) return;	// don't ever drop the root node with this function

	if (mutex_lock(&mutex, 20, __func__)) {
		parent = n->parent;
		list = &parent->children;
		while (*list && *list != n) list = &(*list)->next;	// try to find the node in the list of children
		if (*list == n) {									// node entry found
			*list = n->next;								// take node out of list
			n->next = NULL;
			BDBnode_freeNode(n);
		}
		mutex_unlock(&mutex);
		BDBnode_nodeEvent();
	}
}

static struct bidibnode *_BDBnode_lookupNode (struct bidibnode *n, adrstack_t adr)
{
	struct bidibnode *list;

	if (!adr) return n;
	list = n->children;
	while (list) {
		if ((adr >> 24) == list->localadr) {
			return _BDBnode_lookupNode(list, adr << 8);
		}
		list = list->next;
	}

	return NULL;		// not found
}

/**
 * Look up a node by it's full address stack. This is done recursively
 * from the ROOT node downwards.
 *
 * \param adr		the complete address stack of the node to look up with most significant byte as the first level address
 * \return			the node found for this address or NULL if the node could not be found
 */
struct bidibnode *BDBnode_lookupNode (adrstack_t adr)
{
	return _BDBnode_lookupNode(BDBroot, adr);
}

/**
 * A simple function to get the ROOT node, which is static in this module.
 *
 * \return		pointer to the BiDiB root node
 */
struct bidibnode *BDBnode_getRoot (void)
{
	return BDBroot;
}

/**
 * Look up a node by it's local address as child of a given node.
 * It uses the same recursive approach as BDBnode_lookupNode() but
 * will only dive one deep, because the address constists of only
 * the most significant address byte of a faked address stack.
 *
 * \param parent	the parent node where the node we look for is registrated as a child
 * \param adr		the local address byte of the child node to look up
 * \return			the node found for this address or NULL if the node could not be found
 */
struct bidibnode *BDBnode_lookupChild (struct bidibnode *parent, uint8_t adr)
{
	return _BDBnode_lookupNode(parent, adr << 24);
}

/**
 * Look up a node by it's UID.
 *
 * \param uid		pointer to the 7 byte UID as byte array
 * \param list		a starting point for the search - if NULL, the root node is taken as start point
 * \return			the pointer to the node found or NULL if no matching node could be found
 */
struct bidibnode *BDBnode_lookupNodeByUID (uint8_t *uid, struct bidibnode *list)
{
	struct bidibnode *n;

	if (!uid) return NULL;
	if (!list && BDBroot) list = BDBroot->children;
	while (list) {
		if (!memcmp(list->uid, uid, BIDIB_UID_LEN)) return list;
		if (list->children) {
			n = BDBnode_lookupNodeByUID(uid, list->children);
			if (n) return n;
		}
		list = list->next;
	}

	return NULL;
}

/**
 * Look up a node by it's UID without taking the class bits into account.
 *
 * \param uid		pointer to the 5 byte UID as byte array (i.e. &UID[2] for standard/full uid strings)
 * \param list		a starting point for the search - if NULL, the root node is taken as start point
 * \return			the pointer to the node found or NULL if no matching node could be found
 */
struct bidibnode *BDBnode_lookupNodeByShortUID (uint8_t *uid, struct bidibnode *list)
{
	struct bidibnode *n;

	if (!uid) return NULL;
	if (!list && BDBroot) list = BDBroot->children;
	while (list) {
		if (!memcmp(&list->uid[2], uid, BIDIB_UID_LEN - 2)) return list;
		if (list->children) {
			n = BDBnode_lookupNodeByShortUID(uid, list->children);
			if (n) return n;
		}
		list = list->next;
	}

	return NULL;
}

/**
 * Insert the new node sorted into the given list.
 * If the list pointer is NULL, we will insert the node in the main node
 * list.
 *
 * \param parent	the parent node where the new node should be put. If NULL, the root node is addressed
 * \param n			the new node to be put into the list
 */
void BDBnode_insertNode (struct bidibnode *parent, struct bidibnode *n)
{
	struct bidibnode **list;

	if (!n) return;		// nothing to do
	log_msg(LOG_BIDIB, "%s(): PARENT: %s += CHILD %s\n", __func__,
			bidib_formatAdrStack(bidib_getAddress(parent)), bidib_formatAdrStack(bidib_getAddress(n)));
	if (!parent) parent = BDBroot;
	list = &parent->children;

	if (mutex_lock(&mutex, 20, __func__)) {
		while (*list && (*list)->localadr < n->localadr) list = &(*list)->next;
		n->next = *list;
		*list = n;
		n->parent = parent;
		mutex_unlock(&mutex);
	}
	BDBnode_nodeEvent();
}

/**
 * Creates a new node structure with the given UID and supplied (sub)-address.
 * The node is not inserted in the list of nodes, because we don't have a clue
 * at which level it is to be inserted.
 *
 * \param uid		the seven bytes that make up the UID of the new node
 * \param adr		the local address on the bus where it is sitting
 * \return			an allocated node structure, filled with UID and address information
 * 					or NULL on any error
 */
struct bidibnode *BDBnode_createNode (uint8_t *uid, uint8_t adr)
{
	struct sysconf *cfg;
	struct bidib_feedback *fb;
	struct feedback_map *fm;
	struct bidibnode *n;

	if (!uid) return NULL;

	if ((n = calloc (1, sizeof(*n))) != NULL) {
		memcpy (n->uid, uid, sizeof(n->uid));
		n->localadr = adr;
		if (uid[0] & BIDIB_CLASS_OCCUPANCY) {		// occupancy nodes will get a feedback address to integrate them with s88 and Co.
			cfg = cnf_getconfig();
			fb = cfg->bidibfb;
			while (fb) {
				log_msg(LOG_INFO, "%s()  ----> fb->uid[2]: %02x %02x %02x %02x %02x ---> uid[2]: %02x %02x %02x %02x %02x\n", __func__,
					fb->uid[2], fb->uid[3], fb->uid[4], fb->uid[5], fb->uid[6], uid[2], uid[3], uid[4], uid[5], uid[6]);
				if (!memcmp(&fb->uid[2], &uid[2], BIDIB_UID_LEN - 2)) {
					if ((fm = malloc (sizeof(struct feedback_map))) != NULL) {
						fm->base = fb->s88base;
						n->private = fm;
						log_msg(LOG_INFO, "%s() %s CLASS OCCUPANCY -> s88 %d\n", __func__, bidib_formatUID(uid), fm->base);
					}
					break;
				}
				fb = fb->next;
			}
		}
	}
	return n;
}

void BDBnode_resetNodeList (void)
{
	BDBnode_freeNodeList(BDBroot);
	BDBroot = BDBsrv_genLocalNode();
	BDBvn_clearFbMappings();
	BDBvn_feedbackModules(0, cnf_getconfig()->s88Modules, MAX_S88MODULES, BIDIB_HUB_S88);
	// TODO how can we know how much modules there are and at what addresses they report occupancies?
	BDBvn_feedbackModules(0, cnf_getconfig()->canModules, MAX_CANMODULES, BIDIB_HUB_MCAN);
	BDBvn_feedbackModules(0, cnf_getconfig()->lnetModules, MAX_LNETMODULES, BIDIB_HUB_LNET);
	BDBnode_nodeEvent();
}

/**
 * The recursive code for BDBnode_iterate(). It traverses the tree and calls the
 * given function for every encountered node.
 *
 * \param n			the nodelist to process - if children are found, call this function with the list of children
 * \param func		the function to call (prototype: func(struct bidibnode *n)
 * \see				BDBnode_iterate()
 */
static void _BDBnode_iterate (struct bidibnode *n, void (*func)(struct bidibnode *))
{
	if (!func) return;

	while (n) {
		func (n);
		if (n->children) _BDBnode_iterate(n->children, func);
		n = n->next;
	}
}

/**
 * Iterate over all known nodes (except the local node) and call the
 * supplied function with each node encountered.
 *
 * \param func		the function to call (prototype: func(struct bidibnode *n)
 */
void BDBnode_iterate (void (*func)(struct bidibnode *))
{
	_BDBnode_iterate(BDBroot->children, func);
}

/*
 * ===================================================================================
 * Handling of our direct node table from BiDiBus.
 * If we are in the SERVER-role, we must try to report added or deleted nodes
 * periodically, until we get an acknowledge.
 * If we are the CONTROLLER ourself, we just report the node change immideately
 * to the controller thread.
 * ===================================================================================
 */

#define NTAB_REPORT_RETRIES		16			///< according to specs we should try 16 times to report a nodetab change
#define NTAB_REPORT_TIMEOUT		250			///< the report retries should be fired every 250ms

static struct {
	SemaphoreHandle_t	mutex;					///< a mutex to lock access to this structure
	TickType_t			timer;					///< a timeout when to send the next report
	volatile int		retry;					///< a retry counter, if <= 0 no report will be generated
	uint8_t				uid[BIDIB_UID_LEN];		///< the node's UID to report
	uint8_t				adr;					///< the node's address on the local BiDiBus interface
	uint8_t				version;				///< the node table version when this change occured
	bool				deletion;				///< if set, the node change was a deletion (lost node), else an addition (new node)
	bool				enable;					///< if set, the reporting is enabled (i.e. we are in SERVER mode and not SYS_DISABLED)
} nodeTabChange;

/**
 * Internal function that does the job of setting up the table change information.
 *
 * \param uid		the UID of the new node
 * \param adr		the address on the bus that this node has been assigned
 * \param version	the node tab version which should be reported upstream for this change
 * \param del		if true, the change was due to a lost node, else a new node was discovered
 * \see				BDBnode_pollChangeReport() for execution
 */
static void BDBnode_nodeTabChange (uint8_t *uid, uint8_t adr, uint8_t version, bool del)
{
	if (mutex_lock(&nodeTabChange.mutex, 100, __func__)) {
		memcpy (nodeTabChange.uid, uid, BIDIB_UID_LEN);
		nodeTabChange.adr = adr;
		nodeTabChange.version = version;
		nodeTabChange.deletion = del;
		nodeTabChange.retry = NTAB_REPORT_RETRIES;
		nodeTabChange.timer = xTaskGetTickCount();		// we can immedeately try to send the update as soon as the BiDiBus thread is calling the poll function
		mutex_unlock(&nodeTabChange.mutex);
	}
}

/**
 * Immideately take care of the changing node table.
 * Reporting to the network client must be handled.
 *
 * \param uid		the UID of the new node
 * \param adr		the address on the bus that this node has been assigned
 * \param version	the node tab version which should be reported upstream for this change
 */
void BDBnode_newBusNode (uint8_t *uid, uint8_t adr, uint8_t version)
{
	struct bidibnode *n;

	log_msg (LOG_BIDIB, "%s(): [%d] UID %s\n", __func__, adr, bidib_formatUID(uid));
	n = BDBnode_createNode(uid, adr);
	BDBnode_insertNode(BDBroot, n);

	if (bidib_opmode() == BIDIB_CONTROLLER) {
		BDBctrl_nodeNew(n);
	} else {
		BDBnode_nodeTabChange(uid, adr, version, false);
	}
	BDBnode_nodeEvent();
}

/**
 * Immideately take care of the changing node table.
 * Reporting to the network client is handled directly by the BiDiBus handler.
 *
 * \param uid		the UID of the lost node
 * \param adr		the address on the bus that this node had until it vanished
 * \param version	the node tab version which should be reported upstream for this change
 */
void BDBnode_lostBusNode (uint8_t *uid, uint8_t adr, uint8_t version)
{
	struct bidibnode *n;

	log_msg (LOG_BIDIB, "%s(): [%d] UID %s\n", __func__, adr, bidib_formatUID(uid));
	n = BDBnode_lookupChild(BDBroot, adr);
	if (!n) return;

	if (bidib_opmode() == BIDIB_CONTROLLER) {
		BDBctrl_nodeLost(n);			// CONTROLLER must take node out of list and trigger event
	} else {
		BDBnode_dropNode(n);
		BDBnode_nodeEvent();

		BDBnode_nodeTabChange(uid, adr, version, true);
	}
}

int BDBnode_getFreeAddress (struct bidibnode *n, int minadr)
{
	struct bidibnode **pp;

	if (minadr < 1) minadr = 1;
	if (minadr > 255) minadr = 255;
	pp = &n->children;
	while (*pp && (*pp)->localadr <= minadr) {
		if ((*pp)->localadr == minadr) minadr++;
		pp = &(*pp)->next;
	}
	if (!*pp || ((*pp)->localadr > minadr && minadr <= 255)) return minadr;
	return 0;	// no free address found
}

/**
 * Enable or disable node tab changes if we received a MSG_SYS_ENABLE or MSG_SYS_DISABLE.
 *
 * \param en		if true, reporting should be enabled, disabled otherwise
 */
void BDBnode_reportEnable (bool en)
{
	nodeTabChange.enable = en;
}

/**
 * If we receive a MSG_NODE_CHANGED_ACK with the correct version of our node tab,
 * we can stop sending the update.
 *
 * As a special exception, we also will stop sending updates if the function is
 * called with a NULL argument as message. This can be used to terminate the message
 * attempts in case the remote controller disconnected and we switch back to control
 * ourself.
 *
 * \param m		the message with the acknowledged version number of the node tab
 */
void BDBnode_changeACK (struct bidibnode *n, bidibmsg_t *m)
{
	uint8_t version;

	(void) n;

	if (m) version = m->data[0];
	else version = 0;

	if (mutex_lock(&nodeTabChange.mutex, 50, __func__)) {
		if (version == 0 || nodeTabChange.version == version) {		// this is the correct ACK for the current node tab version as we know it
			nodeTabChange.retry = 0;
			nodeTabChange.timer = 0;
		}
		mutex_unlock(&nodeTabChange.mutex);
	}
}

/**
 * Poll the node tab change info to see, if we should inform the netBiDiB controller of a
 * new or lost node.
 *
 * This polling is called from the BiDiBus thread. This makes an additional thread
 * unneccessary and is uncritical. We don't want to use a timer function here, because
 * timers shouldn't use calls that might need wait times (as is the case in bidib_uplink()
 * and mutex_lock()).
 */
void BDBnode_pollChangeReport (void)
{
	bidibmsg_t *m;
	uint8_t data[16];

	// the first checks don't need to lock the structure ...
	if (bidib_opmode() == BIDIB_CONTROLLER) return;						// if we are the controller, we don't need this stuff ...
	if ((nodeTabChange.retry <= 0) || !nodeTabChange.enable) return;	// nothing to do or reporting currently disabled
	if (!tim_isover(nodeTabChange.timer)) return;						// timer not yet done - wait a little more

	if (mutex_lock(&nodeTabChange.mutex, 10, __func__)) {
		data[0] = nodeTabChange.version;
		data[1] = nodeTabChange.adr;
		memcpy (&data[2], nodeTabChange.uid, BIDIB_UID_LEN);
		if ((m = bidib_genMessage(LOCAL_NODE(), (nodeTabChange.deletion) ? MSG_NODE_LOST : MSG_NODE_NEW, 9, data)) != NULL) {
			netBDB_postMessages(m);
			nodeTabChange.retry--;
			nodeTabChange.timer = xTaskGetTickCount() + pdMS_TO_TICKS(NTAB_REPORT_TIMEOUT);
		}
		mutex_unlock(&nodeTabChange.mutex);
	}
}

/**
 * Process DOWNLINK messages from controlling client.
 * The message memory is not freed!
 *
 * In the first step, the downlink function decoder is queried for an action to
 * take on this message. Then, if the message is a broadcast, it is forwared to
 * all children. As an exception to this, the message is not forwared to the
 * BiDiBus here, because that would free the message memory after sending out the
 * message on the bus.
 *
 * This forwarding will be done in BDBnode_downlink(), which is our caller. It
 * also will handling memory freeing after all is done.
 *
 * \param n		the node that is receiving the message
 * \param m		a single message to be interpreted
 * \return		a possible failure code or 0 for OK
 */
int BDBnode_handleMessage (struct bidibnode *n, bidibmsg_t *m)
{
	struct bidibnode *child;
	const struct msgdecoder *d;

	if (!n) return -1;

	// sequence number reset should be done, before acton on the message
	if (!bidib_isLocal(m->msg) && !bidib_isBroadcast(m->msg) && m->seq == 0) {	// check if the message sequence should be reset
		n->txmsgnum = n->rxmsgnum = 1;
	}

	d = n->downstream;
	while (d->msg) {
		if (d->msg == m->msg) {
			if (d->handler) d->handler (n, m);
			break;
		}
		d++;
	}
	if (bidib_isBroadcast(m->msg)) {	// probably forward the message down to our children
		child = n->children;
		while (child) {
			if (n != BDBroot || child->localadr > BIDIBUS_MAX_NODEADR) BDBnode_handleMessage(child, m);
			child = child->next;
		}
	}
	return 0;
}

/**
 * Pass a message down the tree of nodes.
 * This function works recursivly
 *
 * \param n		the node that is receiving the message
 * \param m		the message itself
 */
void BDBnode_downlink (struct bidibnode *n, bidibmsg_t *m)
{
	struct bidibnode *child;
	bidibmsg_t *err;
	uint8_t localadr;

	if (!n) n = BDBroot;
//	log_msg (LOG_BIDIB, "%s(): for %s (in Parent %s)\n", __func__, bidib_formatAdrStack(m->adrstack), bidib_formatAdrStack(bidib_getAddress(n)));

	if (m->adrstack) {		// this message is destined for a subnode (child)
		localadr = m->adrstack >> 24;
		child = n->children;
		while (child && child->localadr != localadr) child = child->next;
		if (!child) {		// child not found - report upstream
			err = bidib_genMessage(n, MSG_NODE_NA, 1, &localadr);
			BDBnode_uplink(n, err);
			free (m);
		} else {
			if (n == BDBroot && child->localadr <= BIDIBUS_MAX_NODEADR) {	// message is for physical node on the BiDiBus
//				log_msg (LOG_BIDIB, "%s(): forwarding %s to BiDiBus\n", __func__, bidib_formatAdrStack(m->adrstack));

				// ($$$$ TEST) Delay delivery of UPDATE-OP BIDIB_MSG_FW_UPDATE_OP_DONE for a short time
//				if (m->msg == MSG_FW_UPDATE_OP && m->data[0] == 0x04) vTaskDelay(20);

				BDBus_sendMessage(m);					// frees message memory
			} else {														// message is for any of our virtual nodes
				m->adrstack <<= 8;
				BDBnode_downlink(child, m);
			}
		}
	} else {				// this message is for the current node - it is not a node on BiDiBus!
		BDBnode_handleMessage(n, m);			// handle message with the downlink action table in the node, does NOT free message memory
		if (n == BDBroot && bidib_isBroadcast(m->msg)) {	// special exception: broadcasts on root node are finally forwarded on BiDiBus
			BDBus_sendMessage(m);	// broadcast to physical bus segment (frees message memory)
		} else {
			free (m);
		}
	}
}

/**
 * Pass a message up the tree of nodes. From root node on, we
 * pass the message to the netBiDiB or internal controller layer.
 *
 * \param n		the sending node
 * \param m		the message to send (starts with empty addrstack)
 */
void BDBnode_uplink (struct bidibnode *n, bidibmsg_t *m)
{
	if (!n || !n->parent) {	// this message was sent by the root node - either pass to network stack or local controller
		if (bidib_opmode() == BIDIB_CONTROLLER) {
			BDBctrl_messageReceived(m);
		} else {
			BDBsrv_upstream(m);
			netBDB_postMessages(m);
		}
	} else {
		m->adrstack >>= 8;
		m->adrstack |= n->localadr << 24;
		BDBnode_uplink(n->parent, m);
	}
}
