/*
 * bidib_implementation.h
 *
 *  Created on: 19.12.2020
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

#ifndef __BIDIB_IMPLEMENTATION_H__
#define __BIDIB_IMPLEMENTATION_H__

#define BIDIB_UID_LEN			7					/**< defined length in bytes of the UID for every type of node (must be defined
														 before including "decoder.h"!) */

#include "bidib_messages.h"
#include "decoder.h"			// include this to have "enum fmt", includes "bidib.h" itself to have "BIDIB_UID_LEN" defined ... (!)

#define BIDIB_PORT				62875				///< netBiDiB port for the UDP-announcer (fixed!), TCP gets it's port from configuration
#define BIDIB_SIGNATURE_TAMS	"BiDiB-mc2"			///< a signature identifyer which _must_ start with "BiDiB"
#define BIDIB_PRODUCTID_TAMS	0xA0				///< TAMS mc2 - 0xA0 to 0xAF are reserved for (virtual) mc2 peripherals
#define BIDIB_PID_VIRTUAL		BIDIB_PRODUCTID_TAMS		///< The base for the virtual product IDs
#define BIDIB_PID_VIRT_HUB		(BIDIB_PID_VIRTUAL | 0x1)	///< virtual hub
#define BIDIB_PID_VIRT_S88		(BIDIB_PID_VIRTUAL | 0x2)	///< virtual s88 modules
#define BIDIB_PID_VIRT_MCAN		(BIDIB_PID_VIRTUAL | 0x3)	///< virtual mcan modules
#define BIDIB_PID_VIRT_LNET		(BIDIB_PID_VIRTUAL | 0x4)	///< virtual LocoNet modules
#define BIDIB_PID_VIRT_BOOST	(BIDIB_PID_VIRTUAL | 0x5)	///< virtual Booster modules
#define BIDIB_MAX_FB_PER_TYPE	4095				///< maximum number of feedbacks per feedback type (s88, mcan, lnet)
#define BIDIB_PRODSTR_TAMS		"MC2"				///< Product string for the mc²
#define BIDIB_PRODSTR_VIRT_IF	"MC2 virtual hub"	///< Product string for virtual hub nodes
#define BIDIB_PRODSTR_VIRT_S88	"MC2 s88 feedback"	///< Product string for virtual s88 nodes
#define BIDIB_PRODSTR_VIRT_LN	"MC2 L-NET feedback"///< Product string for virtual loconet nodes
#define BIDIB_PRODSTR_VIRT_MC	"MC2 mCAN feedback"	///< Product string for virtual mCAN nodes

#define BIDIB_HUB_S88			1					///< the fixed serial suffix of the HUB node UID for s88 modules
#define BIDIB_HUB_MCAN			2					///< the fixed serial suffix of the HUB node UID for mcan modules
#define BIDIB_HUB_LNET			3					///< the fixed serial suffix of the HUB node UID for loconet modules

#define BIDIB_CLASS_SWITCH		0x01				///< contains switchable accessory functions
#define BIDIB_CLASS_BOOSTER		0x02				///< contains a booster
#define BIDIB_CLASS_ACCESSORY	0x04				///< contains accessory functions
#define BIDIB_CLASS_DCC_PROG	0x08				///< generates DCC programming signals
#define BIDIB_CLASS_DCC_MAIN	0x10				///< generates DCC track signals
#define BIDIB_CLASS_UI			0x20				///< has a user interface (UI) attached
#define BIDIB_CLASS_OCCUPANCY	0x40				///< contains occupancy functions
#define BIDIB_CLASS_BRIDGE		0x80				///< is a bridge interface (may control subnodes)

#define MAX_PRODUCT_STRING		24					///< according to documentation, the PRODUCT string must not be longer than 24 characters (plus null byte)
#define MAX_USER_STRING			24					///< according to documentation, the USER string must not be longer than 24 characters (plus null byte)

#define BIDIBUS_MAX_NODEADR		63					///< the maxmimum node address in BiDiBus protocol (6 bits) - virtual nodes get addresses beyond this
#define LOCAL_NODE()			BDBnode_lookupNode(0)
#define IS_ROOT_NODE(n)			((n) == BDBnode_lookupNode(0))

#define NODEFLG_VIRTUAL			0x0001				///< this node is an internal (virtual) node - including the root node
#define NODEFLG_SYSDISABLE		0x0002				///< current state is SYS_DISABLE (no spontaneous messages allowed)
#define NODEFLG_IDENTIFY		0x0004				///< the node is in identify state

/**
 * As a node in upstream direction we have:
 *   - interface (either controlling other nodes via IP or directly via BiDiBus)
 *   - NO (!) occupancy status from feedback physically located on s88N / MCAN / LocoNet. These devices are modeled by virtual nodes instead.
 *   - dcc signal generator for main and programming track
 *   - an integrated booster
 */
//#define BIDIB_CLASS		(BIDIB_CLASS_BRIDGE | BIDIB_CLASS_OCCUPANCY | BIDIB_CLASS_DCC_MAIN | BIDIB_CLASS_DCC_PROG | BIDIB_CLASS_BOOSTER)
#define BIDIB_CLASS		(BIDIB_CLASS_BRIDGE | BIDIB_CLASS_DCC_MAIN | BIDIB_CLASS_DCC_PROG | BIDIB_CLASS_BOOSTER)
#define BIDIB_XCLASS	0	// currently, this is still ununsed in BiDiB system ...

extern uint8_t myUID[BIDIB_UID_LEN];			///< our UID is defined in bidib.c but should be available in the whole stack

typedef struct bidibmsg bidibmsg_t;
typedef uint32_t adrstack_t;

enum nodestate {
	NS_IDLE = 0,						///< nothing to be done - we are satisfied with all
	NS_FAILED,							///< we are in trouble with this node - ignore it
	NS_BOOTMODE,						///< node is in boot mode
	NS_GET_SYSMAGIC,					///< get node-magic
	NS_GET_P_VERSION,					///< get P-VESRION
	NS_READ_FEATURES,					///< features are requested
	NS_AUTOREAD_FEATURES,				///< features are requested and reported automatically b the node
	NS_GET_PRODSTRING,					///< get product string (if available)
	NS_GET_USERNAME,					///< get the user name (if available)
	NS_GET_SW_VERSION,					///< get the software version from the node
	NS_READ_NTABCOUNT,					///< read the node table count
	NS_READ_NODETAB,					///< read the node table
};

struct bidibmsg {
	bidibmsg_t	*next;					///< list of messages (probably only after decomposing a packet with multiple messages)
	adrstack_t	 adrstack;				///< the four levels of address stack, always interpreted little endian
	uint8_t		 seq;					///< the "serial number" of the message to detect missing messages (1 .. 255, 0 for broadcasts)
	uint8_t		 msg;					///< the BiDiB message identifier
	uint8_t		 datalen;				///< the number of data bytes (in the final packet, the address stack, msgnum and msg are added)
	uint8_t		 data[];				///< any number of data bytes (the final length parameter is currently limited to 127, BiDiBus limits this further)
};

struct bidibnode {
	struct bidibnode	*next;			///< linked list of nodes
	struct bidibnode	*children;		///< a list of subnodes, if this bidibnode is a HUB
	struct bidibnode	*parent;		///< the parent node if not root node
	struct nodefeature	*features;		///< the list of available features on the bidibnode
	const struct msgdecoder	*downstream;	///< handler table for downstream messages
	const struct msgdecoder	*upstream;	///< handler table for upstream messages to sniff operation when on external control
	struct ntab_report	*ntab_rep;		///< a copy of the actual node table to report upstream
	uint16_t			 pversion;		///< the supported protocol version from other side
	uint8_t				 uid[BIDIB_UID_LEN];		///< the UID of the bidibnode
	uint8_t				 localadr;		///< the one-byte address on the local bus (may be a virtual bus)
	uint8_t				 featurecount;	///< the number of features in the '<code>features</code>'-member
	uint8_t				 featureidx;	///< when successively reading features, this is the index where a feature should be stored
	uint8_t				 txmsgnum;		///< the current TX message number for transmission
	uint8_t				 rxmsgnum;		///< the last RX message number received
	uint8_t				 ntab_version;	///< the version of the local nodetab (only for HUB-nodes)
	uint8_t				 errcode;		///< the current error code (0 for no error)
	char				 product[MAX_PRODUCT_STRING + 1];	///< allow for terminating null byte
	char				 user[MAX_USER_STRING + 1];			///< allow for terminating null byte
	enum nodestate		 state;			///< a statusmachine for the CONTROLLER mode
	int					 stateidx;		///< a counting index for things like receiving the node table
	TickType_t			 timeout;		///< timeout waiting for a response
	int					 retry;			///< a retry counter when waiting for answers
	uint32_t			 flags;			///< various flags to control behavior
	void				*private;		///< for virtual nodes: private data structure for state information
};

struct nodefeature {
	uint8_t		 feature;				///< the feature number
	uint8_t		 value;					///< the value of this feature
	uint8_t (*setter) (struct bidibnode *, struct nodefeature *, uint8_t);		///< an action function for settable features
};

struct ntab_report {
	uint8_t				 ntab_version;	///< the current node table version that is reported
	uint8_t				 nodecount;		///< total nodes to report
	uint8_t				 nodeidx;		///< the index of the node to report next
	struct {
		uint8_t			 nodeadr;		///< the local node address on this hub
		uint8_t			 uid[BIDIB_UID_LEN];	// the UID of a local node to report
	} nodes[];
};

/**
 * A structure to hold information for virtual feedback nodes.
 * These nodes map feedback bits from the 64k linear feedback
 * space to BiDiB modules.
 */
struct virtual_feedback {
	int				 base;			///< the feedback base as 0-based number (0 .. 64k - 1)
	int				 count;			///< the number of feedback bits supported by this node (up to 128 are allowed)
	uint32_t		*bitset;		///< representation as bitset for bitset utility functions, should point to the status array
	uint8_t			 status[];		///< the current status of the feedback, already in BiDiB order (LSBit of byte 0 is detector #1)
};

struct virtual_hub {
	uint8_t				nodetype;		///< one of the virtual node types that are gathered behind this hub (0xA2 .. 0xAF)
};

struct feedback_map {
	int					base;			///< the address of a BiDiB feedback module (MSG_BM_OCC, MSG_BM_FREE, ...) to map to the s88 system
};

/**
 * A strcuture to hold interpreter information used for handling the received messages
 */
struct msgdecoder {
	uint8_t		msg;					///< the message byte
	void (*handler)(struct bidibnode *n, bidibmsg_t *msg);	///< a function that handles the contents of the message data
};

enum opmode {
	BIDIB_CONTROLLER,					///< we control the BiDiB system ourself
	BIDIB_SERVER						///< we have the server role and are controlled by an upstream instance
};

/*
 * Prototypes Interfaces/BiDiB/bidib.c
 */
void bidib_busError (uint8_t errcode, uint8_t adr);
bidibmsg_t *bidib_errorMessage (struct bidibnode *n, uint8_t code, int len, uint8_t *extra);
void bidib_extControl (bool on);
enum opmode bidib_opmode (void);
bool bidib_isSysDisabled (void);
void bidib_sysDisable (void);
void bidib_sysEnable (void);
uint16_t bidib_getSysTime (void);
void bidib_identify (bool on);
void bidib_identifyToggle (void);

/*
 * Prototypes Interfaces/BiDiB/bidibctrl.c
 */
void BDBctrl_dcca(struct bidibnode *n, bidibmsg_t *m);
void BDBctrl_accessoryState(struct bidibnode *n, bidibmsg_t *m);
void BDBctrl_bmOCC (struct bidibnode *n, bidibmsg_t *m);
void BDBctrl_bmFREE (struct bidibnode *n, bidibmsg_t *m);
void BDBctrl_bmMULTIPLE (struct bidibnode *n, bidibmsg_t *m);
void BDBctrl_controller (void *pvParameter);
void BDBctrl_messageReceived (bidibmsg_t *m);
void BDBctrl_nodeNew (struct bidibnode *n);
void BDBctrl_nodeLost (struct bidibnode *n);
void BDBctrl_busError (uint8_t errcode, uint8_t adr);
void BDBctrl_boosterOff (void);
void BDBctrl_boosterOn (void);

/*
 * Prototypes Interfaces/BiDiB/bidibus.c
 */
void BDBus_resetBus (bidibmsg_t *msg);
void BDBus_sendMessage (bidibmsg_t *bm);
void BDBus (void *pvParameter);

/*
 * Prototypes Interfaces/BiDiB/bidibnode.c
 */
void BDBnode_freeNodeList (struct bidibnode *nodes);
void BDBnode_dropNode (struct bidibnode *n);
void BDBnode_nodeEvent (void);
struct bidibnode *BDBnode_lookupNode (adrstack_t adr);
struct bidibnode *BDBnode_getRoot (void);
struct bidibnode *BDBnode_lookupChild (struct bidibnode *parent, uint8_t adr);
struct bidibnode *BDBnode_lookupNodeByUID (uint8_t *uid, struct bidibnode *list);
struct bidibnode *BDBnode_lookupNodeByShortUID (uint8_t *uid, struct bidibnode *list);
void BDBnode_insertNode (struct bidibnode *parent, struct bidibnode *n);
struct bidibnode *BDBnode_createNode (uint8_t *uid, uint8_t adr);
void BDBnode_resetNodeList (void);
void BDBnode_iterate (void (*func)(struct bidibnode *));
void BDBnode_newBusNode (uint8_t *uid, uint8_t adr, uint8_t version);
void BDBnode_lostBusNode (uint8_t *uid, uint8_t adr, uint8_t version);
int BDBnode_getFreeAddress (struct bidibnode *n, int minadr);
void BDBnode_reportEnable (bool en);
void BDBnode_changeACK (struct bidibnode *n, bidibmsg_t *m);
void BDBnode_pollChangeReport (void);
void BDBnode_downlink (struct bidibnode *n, bidibmsg_t *m);
void BDBnode_uplink (struct bidibnode *n, bidibmsg_t *m);

/*
 * Prototypes Interfaces/BiDiB/bidibserver.c
 */
//void BDBsrv_handleMessage (bidibmsg_t *m);
void BDBsrv_upstream (bidibmsg_t *bm);
void BDBsrv_readControls (bidibmsg_t *msgs);
void BDBsrv_updateFeatures (void);
struct bidibnode *BDBsrv_genLocalNode (void);
void BDBsrv_start (void);

/*
 * Prototypes Interfaces/BiDiB/bidibutil.c
 */
char *bidib_formatAdrStack (uint32_t stack);
char *bidib_formatUID (uint8_t *uid);
int bidib_readAdrStack (uint8_t *p, uint32_t *adr);
void bidib_debugSingleMessage (const char *caller, struct bidibmsg *msg, bool up);
void bidib_debugMessages (const char *caller, struct bidibmsg *msg, bool up);
void bidib_debugError (const char *caller, struct bidibmsg *msg);
//adrstack_t bidib_genSubAdr (adrstack_t nodeadr, uint8_t subadr);
adrstack_t bidib_num2stack (uint32_t adr);
adrstack_t bidib_getAddress (struct bidibnode *n);
int bidib_packSize (bidibmsg_t *bm);
uint8_t *bidib_addUID (uint8_t *p, uint8_t *uid);
uint8_t *bidib_addString (uint8_t *msgdata, char *str, int maxlen);
uint8_t *bidib_addNsIdString (uint8_t *msgdata, uint8_t ns, uint8_t id, char *str, int maxlen);
void bidib_freeMessages (bidibmsg_t *m);
bidibmsg_t *bidib_genMessage (struct bidibnode *n, uint8_t msg, int len, uint8_t *data);
uint8_t *bidib_packMessage (bidibmsg_t *bm, uint8_t *data);
int bidib_packAllMessages (bidibmsg_t *bm, uint8_t *data, int maxlen);
bidibmsg_t *bidib_unpackMessages (uint8_t *pkt, int packetlen, uint8_t adr);
bool bidib_isBroadcast (uint8_t msgcode);
bool bidib_isLocal (uint8_t msgcode);
uint8_t bidib_current2code (int current);
int bidib_code2current (uint8_t code);
uint8_t bidib_msg2speed (uint8_t speed, enum fmt targetfmt);
uint8_t bidib_speed2msg (uint8_t speed, enum fmt sourcefmt);
uint8_t bidib_fmt2code (enum fmt fmt);
bidibmsg_t *bidib_string (struct bidibnode *n, uint8_t ns, uint8_t id, char *str);
enum fmt bidib_code2fmt (uint8_t code);
void bidib_sortFeature (struct bidibnode *n);
struct nodefeature *bidib_readFeature (struct bidibnode *n, uint8_t ft);
uint8_t bidib_getFeatureValue (struct bidibnode *n, uint8_t ft);
void bidib_setFeature (struct bidibnode *n, uint8_t ft, uint8_t val);
void bidib_addFBmap (uint8_t *uid, int s88base);
void bidib_dropFBmap (uint8_t *uid);
void bidib_store (void);
void bidib_load (void);

/*
 * Prototypes Interfaces/BiDiB/netbidib.c
 */
TaskHandle_t netBDB_getTask (void);
void netBDB_postMessages (bidibmsg_t *m);
int netBDB_logon (uint8_t *uid);
int netBDB_logoff (void);
void netBDB_addTrustedClient (uint8_t *uid, char *product, char *user);
void netBDB_genClientStore (struct ini_section **root);
void netBDB_start (void);

/*
 * Prototypes Interfaces/BiDiB/nodefuncs.c
 */
void BDBnf_sendSysMagic (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_sendPVersion (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_sendUniqueID (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_sendVersionInfo (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_sendPong (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_reportNodetab (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_nextNodetab (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_getNextFeature (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_reportFeatures (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_getFeature (struct bidibnode *n, bidibmsg_t *msg);
uint8_t BDBnf_featureWrite (struct bidibnode *n, struct nodefeature *nf, uint8_t val);
uint8_t BDBnf_featureWriteBool (struct bidibnode *n, struct nodefeature *nf, uint8_t val);
void BDBnf_setFeature (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_getString (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_setString (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_getError (struct bidibnode *n, bidibmsg_t *msg);
void BDBnf_sysClock (struct bidibnode *n, bidibmsg_t *msg);

/*
 * Prototypes Interfaces/BiDiB/virtualnode.c
 */
struct bidibnode *BDBvn_newBridge (struct bidibnode *parent, int serial);
struct bidibnode *BDBvn_newS88 (struct bidibnode *parent, int serial);
struct bidibnode *BDBvn_newMCAN (struct bidibnode *parent, int serial);
struct bidibnode *BDBvn_newLNET (struct bidibnode *parent, int serial);
int BDBvn_feedbackModules (int oldcount, int count, int maxcount, int hubSerial);
void BDBvn_clearFbMappings (void);
//void BDBvn_feedbackStatus (struct bidibnode *n, uint16_t newstate);
//struct bidibnode *BDBvn_getFeebackNode (int pid, int idx);

#endif /* __BIDIB_IMPLEMENTATION_H__ */
