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
#include "events.h"
#include "decoder.h"
#include "config.h"
#include "bidib.h"

/**
 * @file	Interfaces/loconet.c
 *
 * Loconet uses CSMA/CD with a UART communication.
 * Because the communication is block oriented, the upper software
 * layers deal with complete blocks and block assembly / transmission
 * is done in the interrupt context.
 *
 * Philosophy
 * ==========
 *
 * Most of the functionality is implemented in a (logically) shared memory
 * which represents "refresh slots". As data bytes in the LocoNet system
 * can only be 7 bits wide, there are 128 slots. Each slot can contain
 * 10 bytes of data. Slot 0 and the slots 120 to 127 are special purpose slots,
 * so there are 119 slots left for loco control (the so called "documentation"
 * from Digitrax claims that there are 120 slots available, but that is
 * simply not true).
 *
 * Slot #0 is a DISPATCH slot for slot move commands or a status slot
 * for the whole system (when read or written). This whole concept is
 * currently not clear to me, so I may be wrong ...
 *
 * From the slots 120 to 127 only two of them are mentioned in the documentation:
 *	- #123 is used as "FAST Clock", a virtual model time which may be stopped,
 *	  1:1 or run at any factor up to 127:1.
 *	- #124 is for programming tasks on either the main or programming track
 *
 * Even though there is no central polling for communication, there is a
 * MASTER defined in the system. This MASTER is the device, that generates
 * (DCC-)packets for the track layout. This device therefor holds the refresh
 * stack and is consequently the one that hosts the shared memory. It has the
 * privilege to ignore the CD BACKOFF times and may seize the bus at any
 * time when no other communication is going on.
 *
 * Blocks
 * ======
 *
 * The following rules apply to blocks:
 *   - the first byte is the OPCode and always has a set MSBit (bit 7).
 *   - the two bits of the OPCode byte encode the length of the packet as 2,
 *     4 or 6 bytes or variable length packet. For variable length the length
 *     is encoded in the first data byte of the packet. The length includes
 *     the OPCode and the check byte.
 *   - all bytes except the OPCode are 7-bit bytes with their MSB being 0.
 *   - the checksum is a negated XOR sum of all bytes except the checksum itself.
 *
 * Reception
 * =========
 *
 * Any byte that has the MSB set will (re)start a new block. If the currently
 * received block is not valid at this point, it is simply regarded as an
 * errornous block and forgotten about.
 *
 * After reception of the second byte of a block we can calculate the length
 * of the block to receive. It is either one of the fixed sizes coded in the
 * OPCode or the second byte received. The length can be a maximum of 127 bytes
 * but in practice will seldomly be more than 14 Bytes. We can define a maximum
 * length by looking at the longest supported command and ignore all blocks
 * that would be longer than that.
 *
 * The checksum is calcuated on the fly with every received byte and should read
 * 0xFF when all bytes according to the length have been read in.
 *
 * There seems to be the definition of a timeout when sending blocks (which
 * obviously will be necessary). The docs say, that we may transmit bytes back
 * to back - but there is no really clear maximum time mentioned between bytes.
 * I think that some words speak of 20 bits (two bytes, 1.200µs) that equal
 * the minimum CD BACKOFF. I will treat that as the timeout for receiving blocks.
 *
 * Successfully received blocks are then put to a reception queue for further
 * handling in the upper layer thread.
 *
 * Transmission
 * ============
 *
 * Transmission must be managed from an independent thread. It controls a buffer
 * that will be filled from a transmission request queue and retried until it is
 * sent successfully or aborted due to excessive retries.
 *
 * A transmission may be started on an idle line. There are different CD BACKOFF
 * times defined, but because we are the MASTER, we can simply ignore it.
 *
 * Collision detection and avoidance
 * =================================
 *
 * ... to be documented ...
 *
 * Hardware
 * ========
 *
 * LocoNet uses LPUART1 in half duplex wired-AND mode. The documentation speaks
 * from wired-OR (which may be correct from a logical view), but physically it
 * is wired-AND on the hardware basis.
 *
 * Data format is 16,66kBaud, 8n1
 * The hardware needs both RX and TX lines to be treated inverted.
 *
 * Timeouts are handled via TIM16.
 */

#define PACKET_DUMP				0				///< set to != 0 to dump all packets

#define RXERR_FRAMING			0x0001			///< framing error on reception
#define RXERR_OVERRUN			0x0002			///< overrun error on reception (because of the FIFO probably never seen!)
#define RXERR_NOISE				0x0004			///< noise detected
#define LPUART_ICR_ALL			(USART_ICR_WUCF | USART_ICR_CMCF | USART_ICR_CTSCF | USART_ICR_TCCF | USART_ICR_IDLECF | \
								 USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF)

#define LN_MAX_BLOCK_LEN		24				///< we are not aware of any block longer than 21 bytes so far
#define LN_PACKET_TIMEOUT		1200			///< the minimum GAP between two packets and also the timeout for incomplete blocks
#define LN_TX_RETRY_ATTEMPTS	10				///< maximum attempts we are doing when transmitting a block
#define NUMBER_OF_SLOTS			120				///< slot 0 (DISPATCH!) + 1 to 119 for loco slots

enum commstate {
	COMM_IDLE = 0,								///< no communication is going on
	COMM_RECEIVE,								///< we simply receive the current block
	COMM_ARBITRATE,								///< wait for backoff before checking for idle line (not needed - we are MASTER)
	COMM_TRANSMIT,								///< we are transmitting and must look out for collisions
	COMM_COLLISION,								///< a collision was detected - send a BREAK
	COMM_TIMEOUT,								///< a timeout occured
	COMM_TXOK,									///< a transmission was successfull
	COMM_TXFAIL,								///< transmission finally failed
};

/*
 * some OPCode found in loconetpersonaledition.pdf, the rocrail wiki
 * and own investigations with different hardware.
 */
// 2-byte OPCodes
#define OPC_BUSY			0x81				///< MASTER busy code
#define OPC_GPOFF			0x82				///< GLOBAL power OFF request
#define OPC_GPON			0x83				///< GLOBAL power ON request
#define OPC_IDLE			0x85				///< FORCE IDLE state, B'cast emerg. STOP

// 4-byte OPCodes
#define OPC_LOCO_SPD		0xA0				///< Set slot speed (without direction bit)
#define OPC_LOCO_DIRF		0xA1				///< Set slot direction and function
#define OPC_LOCO_SND		0xA2				///< Set slot sound function (i.e. functions > F4)
#define OPC_LOCO_F9F12		0xA3				///< Set slot extended functions
#define OPC_SW_REQ			0xB0				///< request switch function
#define OPC_SW_REP			0xB1				///< Turnout sensor state report
#define OPC_INPUT_REP		0xB2				///< General sensor input codes
#define OPC_LONG_ACK		0xB4				///< long acknowledgement (0x00 is usually an error code)
#define OPC_SLOT_STAT1		0xB5				///< write slot stat1
#define OPC_CONSIST_FUNC	0xB6				///< Set FUNC bits in a CONSIST uplink element
#define OPC_UNLINK_SLOTS	0xB8				///< Unlink slot ARG1 from slot ARG2
#define OPC_LINK_SLOTS		0xB9				///< Link slot ARG1 to slot ARG2
#define OPC_MOVE_SLOTS		0xBA				///< move slot SRC to DST
#define OPC_RQ_SL_DATA		0xBB				///< request slot data/status block
#define OPC_SW_STATE		0xBC				///< request state of switch
#define OPC_SW_ACK			0xBD				///< request switch with acknowledge
#define OPC_LOCO_ADR		0xBF				///< request loco address

// 6-byte OPCodes
#define OPC_MULTI_SENSE		0xD0				///< power management and transponding
#define OPC_UHLI_FUN		0xD4				///< Function 9-28 by Uhlenbrock

// variable length OPCodes
#define OPC_IMM_PACKET		0xED				///< Send n-byte packet immediate, used with different length code by IB
#define OPC_LISSY_REP		0xE4				///< Lissy IR report /  Wheel counter report / RFID-5 report / RFID-7 report (depending on value of COUNT and ARG1)
#define OPC_PEER_XFER		0xE5				///< Move 8 bytes peer to peer, SRC -> DST
#define OPC_SL_RD_DATA_E	0xE6				///< slot read response data extended
#define OPC_SL_RD_DATA		0xE7				///< slot read response data
#define OPC_WR_SL_DATA		0xEF				///< write slot data

struct irq_block {
	uint8_t				data[LN_MAX_BLOCK_LEN];	///< the data to transmit or receive
	volatile int		idx;					///< the position where we write or read characters
	volatile int		len;					///< the (planned) length of this block
	volatile uint8_t	chksum;					///< the intermediate checksum for block checking in receiver interrupt
};

struct txrequest {
	volatile size_t		txidx;					///< the TX position inside the block
	volatile size_t		cmpidx;					///< the comparision index for received (echoed) characters
	volatile int		retry;					///< the current retry count to check for exzessive retries
	volatile size_t		len;					///< the valid length of this block
	uint8_t				data[LN_MAX_BLOCK_LEN];	///< the data to transmit
	volatile bool		req;					///< if set, this block is requested to be sent out (else it is done - old data)
};

enum slotstate {
	SLOT_FREE = 0,								///< slot is FREE (data invalid)
	SLOT_COMMON,								///< slot is in COMMON mode (loco valid and refreshed)
	SLOT_IDLE,									///< slot is IDLE (loco valid but not refreshed - unused here)
	SLOT_INUSE									///< slot is IN USE by a throttle  (loco valid and refreshed, owned by a throttle)
};

typedef struct slot {
	int					adr;					///< the loco address controlled in this slot
	enum slotstate		status;					///< the status of this slot
	int					id;						///< the ID from the original LocoNet slot definition
	volatile uint32_t	lastfuncs;				///< the last known state of the functions F0 to F31
	volatile int		lastspeed;				///< the last known speed including the direction bit
} slotT;

static slotT slots[NUMBER_OF_SLOTS];			///< all the slots that are possible, slot #0 will not be used because it is special
static TaskHandle_t rxtask;						///< the task to be notified after RX communication completion
static QueueHandle_t rxqueue;					///< the receive queue, filled by receiver interrupt
static TaskHandle_t txtask;						///< the task to be notified after TX communication completion
static QueueHandle_t txqueue;					///< the transmit queue, filled by upper layer and sent out by interrupt
static struct irq_block rxblock;				///< the block used by the interrupt to receive data frames
static volatile int backoff;					///< the backofftime currently used by the system in micro seconds (us) - 0 for MASTER
//static struct irq_block txblock;				///< the block used by the interrupt to transmit data frames
static struct txrequest txreq;					///< the block used by the interrupt to transmit  data frames
//static volatile enum commstate cs;			///< the interrupt-internal system state

// forward declare the functions for the function table
static int ln_pwrOff (uint8_t *blk);
static int ln_pwrOn (uint8_t *blk);
static int ln_emergencyStop (uint8_t *blk);
static int ln_slotSpeed (uint8_t *blk);
static int ln_slotDirFunc (uint8_t *blk);
static int ln_slotFunc58 (uint8_t *blk);
static int ln_slotFunc912 (uint8_t *blk);
static int ln_trntSwitch (uint8_t *blk);
static int ln_Input (uint8_t *blk);
static int ln_writeSlotStat (uint8_t *blk);
static int ln_slotMove (uint8_t *blk);
static int ln_slotRead (uint8_t *blk);
static int ln_trntQuery (uint8_t *blk);
static int ln_reqLoco (uint8_t *blk);
static int ln_slotFuncUH (uint8_t *blk);
static int ln_FuncDigitrax (uint8_t *blk);
static int ln_IB_configRequest (uint8_t *blk);
static int ln_slotWrite (uint8_t *blk);

static const uint8_t speed28[] = {
	0, 2, 7, 11, 16, 20, 25, 29, 34, 38, 43, 47, 52, 56, 61, 65,	// speed codes 0 .. 15 (without emergency stop!)
	70, 74, 79, 83, 88, 92, 97, 101, 106, 110, 115, 119, 124		// speed codes 16 .. 28
};
static const uint8_t speed27[] = {
	0, 2, 7, 12, 16, 21, 26, 30, 35, 40, 44, 49, 54, 58, 63, 68,	// speed codes 0 .. 15 (without emergency stop!)
	72, 77, 82, 86, 91, 96, 100, 105, 110, 114, 119, 124			// speed codes 16 .. 27
};
static const uint8_t speed14[] = {
	0, 2, 11, 21, 30, 40, 49, 59, 68, 78, 88, 97, 107, 116, 126		// speed codes 0 .. 14 (without emergency stop!)
};

static const uint8_t slot0data[] = {		// the static 10 bytes of SLOT#0 data that are to be reported to throttles
		0x00, 0x00, 0x02, 0x00, 0x07, 0x00, 0x00, 0x00, 0x49, 0x42
};
static int slot0stack;						///< the slot that is DISPATCHED (put), if any

static const struct decoder {
	uint8_t			cmd;					///< the received OPCode (first byte of message)
	uint8_t			len;					///< the length code
	const char		*name;					///< the function name (for debug purposes)
	int (*func) (uint8_t *blk);				///< the interpreter function to call
} lncmds[] = {
	{ OPC_GPOFF,		 2, "OPC_GPOFF",		ln_pwrOff },
	{ OPC_GPON,			 2, "OPC_GPON",			ln_pwrOn },
	{ OPC_IDLE,			 2, "OPC_IDLE",			ln_emergencyStop },
	{ OPC_LOCO_SPD,		 4, "OPC_LOCO_SPD",		ln_slotSpeed },
	{ OPC_LOCO_DIRF,	 4, "OPC_LOCO_DIRF",	ln_slotDirFunc },
	{ OPC_LOCO_SND,		 4, "OPC_LOCO_SND",		ln_slotFunc58 },
	{ OPC_LOCO_F9F12,	 4, "OPC_LOCO_F9F12",	ln_slotFunc912 },
	{ OPC_SW_REQ,		 4, "OPC_SW_REQ",		ln_trntSwitch },
	{ OPC_SW_REP,		 4, "OPC_SW_REP",		NULL },					// currently not handled - seems to report back turnout positions
	{ OPC_INPUT_REP,	 4, "OPC_INPUT_REP",	ln_Input },
	{ OPC_SLOT_STAT1,	 4, "OPC_SLOT_STAT1",	ln_writeSlotStat },
	{ OPC_MOVE_SLOTS,	 4, "OPC_MOVE_SLOTS",	ln_slotMove },
	{ OPC_RQ_SL_DATA,	 4, "OPC_RQ_SL_DATA",	ln_slotRead },
	{ OPC_SW_STATE,		 4, "OPC_SW_STATE",		ln_trntQuery },
	{ OPC_LOCO_ADR,		 4, "OPC_LOCO_ADR",		ln_reqLoco },
	{ OPC_UHLI_FUN,		 6, "OPC_UHLI_FUN",		ln_slotFuncUH },
	{ OPC_IMM_PACKET,	11, "OPC_IMM_PACKET",	ln_FuncDigitrax },		// with 11 bytes, it probably is the Digitrax func packet
	{ OPC_IMM_PACKET,	15, "OPC_CONFIG_REQ",	ln_IB_configRequest },	// with 15 bytes, it probably is an IB specific config request
	{ OPC_WR_SL_DATA,	14, "OPC_WR_SL_DATA",	ln_slotWrite },

	// some send-only blocks to have them as debug output
	{ OPC_SL_RD_DATA,	14,	"OPC_SL_RD_DATA",	NULL },
	{ OPC_LONG_ACK,		 4,	"OPC_LONG_ACK",		NULL },
	{ OPC_PEER_XFER,	15, "OPC_PEER_XFER",	NULL },

	// end of list
	{ 0x00, 0, "(**unknown**)", NULL }
};


static void lpuart1_init (void)
{
	uint32_t cr1;

	LPUART1->CR1 = 0;										// disable LPUART1
	cr1 = USART_CR1_FIFOEN;									// enable FIFO mode
	cr1 |= USART_CR1_TE | USART_CR1_RE;						// set 8 bits of data, enable transmitter and receiver
	LPUART1->CR1 = cr1;

	LPUART1->CR2 = USART_CR2_TXINV | USART_CR2_RXINV;		// TX + RX pins are inverted, 1 stop bit

	LPUART1->CR3 = (0b010 << USART_CR3_RXFTCFG_Pos);		// set RX-FIFO threshold interrupt at half full (but do not enable it for now)

	LPUART1->PRESC = 0b1001;								// prescaler = 64 -> 100MHz / 64 = 1,5625MHz kernel clock
	LPUART1->BRR = 24000;									// 1,5625MHz * 256 / 24000 -> 16,666kbit/s (60µs/bit)

	NVIC_SetPriority(LPUART1_IRQn, 12);
	NVIC_ClearPendingIRQ(LPUART1_IRQn);
	NVIC_EnableIRQ(LPUART1_IRQn);
	LPUART1->ICR = 0xFFFFFFFF;					// clear all interrupt flags

	SET_BIT (LPUART1->CR1, USART_CR1_UE | USART_CR1_RXNEIE_RXFNEIE);		// enable the UART and RX-FIFO not empty interrupt
}

static void tim16_init (void)
{
	TIM16->CR1 = TIM_CR1_OPM;		// one-pulse mode
	TIM16->CR2 = 0;
	TIM16->DIER = TIM_DIER_UIE;
	TIM16->CCMR1 = 0;				// no capture or compare - the reset value anyway
	TIM16->CCER = 0;				// no capture or compare in- / outputs used- the reset value anyway
	TIM16->CNT = 0;					// counter is reset
	TIM16->PSC = 199;				// 200MHz / 200 -> 1MHz -> 1µs per tick
	TIM16->ARR = LN_PACKET_TIMEOUT;	// 1,2ms (it is really 1,201ms plus a delay of 1 clock - who cares?)
	TIM16->RCR = 0;					// we don't use the repetition counter
	TIM16->BDTR = 0;				// don't enable any of these settings - the reset value anyway
	TIM16->AF1 = 0;					// disable the preset "break input enable"
	TIM16->EGR = TIM_EGR_UG;		// generate an update event to get all settings to their real registers

	TIM16->SR = 0;					// clear all interrupts
	NVIC_SetPriority(TIM16_IRQn, 11);
	NVIC_ClearPendingIRQ(TIM16_IRQn);
	NVIC_EnableIRQ(TIM16_IRQn);
}

static void ln_controlEvent (int busadr, int reason)
{
	struct extDevice *dev;

	if ((dev = calloc (1, sizeof(*dev))) == NULL) return;
	dev->bus = BUS_LOCONET;
	dev->id = busadr;
	dev->tp = DEV_CONTROL;
	dev->serial = slots[busadr].id;
	event_fireEx(EVENT_CONTROLS, reason, dev, EVTFLAG_FREE_SRC, QUEUE_WAIT_TIME);
}

void ln_reportControls (void)
{
	int i;

	for (i = 1; i < NUMBER_OF_SLOTS; i++) {
		if (slots[i].status == SLOT_INUSE) ln_controlEvent(i, 1);
	}
}

#if 0
static int _lnet_setModules (int oldcount, int count)
{
	struct bidibnode *root, *n;

	if (count > MAX_LNETMODULES) count = MAX_LNETMODULES;
	if (count < 0) count = 0;
	if (oldcount < 0) oldcount = 0;

	if (oldcount != count) {
		root = BDBnode_lookupNodeByUID(lnetHubUID, NULL);
		if (count == 0) {
			if (root) BDBnode_dropNode(root);
		} else {
			if (!root) {
				oldcount = 0;
				root = BDBvn_newBridge(BDBnode_getRoot(), BIDIB_HUB_LNET);
			}
			while (oldcount < count) {
				oldcount++;
				BDBvn_newLNET(root, BIDIB_LNET_SNR_OFFSET + oldcount);
			}
			while (oldcount > count) {
				if ((n = BDBnode_lookupChild(root, oldcount)) != NULL) BDBnode_dropNode(n);
				oldcount--;
			}
		}
		s88_triggerUpdate();
	}
	return count;
}

void lnet_setModules (int count)
{
	struct sysconf *cnf;

	cnf = cnf_getconfig();
	if (cnf->lnetModules != count) {
		cnf->lnetModules = _lnet_setModules(cnf->lnetModules, count);
		cnf_triggerStore();
	}
}
#else
void lnet_setModules (int count)
{
	struct sysconf *cnf;

	cnf = cnf_getconfig();
	if (cnf->lnetModules != count) {
		cnf->lnetModules = BDBvn_feedbackModules(cnf->lnetModules, count, MAX_LNETMODULES, BIDIB_HUB_LNET);
		cnf_triggerStore(__func__);
#ifdef CENTRAL_FEEDBACK
		event_fire(EVENT_FBPARAM, 0, NULL);
#else
		s88_triggerUpdate();
#endif
	}
}
#endif

static int ln_blockLen (const uint8_t *blk)
{
	int len;

	switch (blk[0] & 0x60) {
		case 0x00: return 2;
		case 0x20: return 4;
		case 0x40: return 6;
	}
	// case 0x60:
	len = blk[1];
	if (len < 5 || len > LN_MAX_BLOCK_LEN) len = 0;	// illeagl length
	return len;
}

static void ln_sendBlock (uint8_t *blk)
{
	uint8_t chk;
	int i, len;

	len = ln_blockLen(blk);
	for (i = 0, chk = 0; i < len - 1; i++) {
		chk ^= blk[i];
	}
	blk[len - 1] = ~chk;
	xQueueSend(txqueue, blk, 100);
}

static void ln_longACK (uint8_t request, uint8_t code)
{
	uint8_t blk[LN_MAX_BLOCK_LEN];

	blk[0] = OPC_LONG_ACK;		// Long ACK
	blk[1] = request & 0x7F;	// echo back the request with dropped MSB
	blk[2] = code & 0x7F;		// the error code
	ln_sendBlock(blk);
}

static uint8_t ln_slotstatus (uint8_t n)
{
	ldataT *l;
	enum fmt fmt;
	uint8_t stat;

	if (n == 0 || n >= 120) return 0;
	if ((l = loco_call(slots[n].adr, false)) != NULL) {
		fmt = l->loco->fmt;
	} else {
		fmt = db_getLoco(0, false)->fmt;		// get the format from the default loco
//		fmt = FMT_DCC_28;
	}
	stat = slots[n].status << 4;
	switch (fmt) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_DCC_14:	// 14 steps mode
			stat |= 0x02;
			break;
		case FMT_MM2_27A:
		case FMT_MM2_27B:
		case FMT_DCC_28:
		default:			// 28 steps mode (code 0b000, so nothing to do)
			break;
		case FMT_DCC_126:
		case FMT_DCC_SDF:
		case FMT_M3_126:	// 126 steps mode
			stat |= 0x7;
			break;
	}
	return stat;
}

static uint8_t ln_trackstatus (void)
{
	switch (rt.tm) {
		default:
		case TM_STOP:		return 0x06;
		case TM_SHORT:		return 0x04;
		case TM_DCCPROG:	return 0x0C;
		case TM_GO:			return 0x05;
	}
}

static uint8_t ln_msg2speed (int speeds, uint8_t msg)
{
	uint8_t locospeed;
	const uint8_t *speedtab;

	switch (speeds) {
		case 14:
			speedtab = speed14;
			break;
		case 27:
			speedtab = speed27;
			break;
		case 28:
			speedtab = speed28;
			break;
		default:
			if (msg) msg--;		// account for the usual emergency stop code '1'
			return msg;			// no other mapping available / needed
	}

	for (locospeed = 0; speedtab[locospeed] < msg && locospeed < speeds; locospeed++) /* iterate over table */ ;
	return locospeed & 0x7F;
}

static uint8_t ln_speed2msg (int speeds, int speed)
{
	speed &= 0x7F;		// drop direction bit
	switch (speeds) {
		case 14:
			return speed14[speed];
		case 27:
			return speed27[speed];
		case 28:
			return speed28[speed];
		default:
			if (speed) speed++;		// account for the usual emergency stop code '1'
			return speed;			// no other mapping available / needed
	}
}

/**
 * Transform a binary block (using all 8 bits of a byte) to the
 * LocoNet message format with 7 bits per byte only. A maximum of
 * 7 bytes can be tranformed to 8 bytes of LocoNet data.
 *
 * The first byte of the subpart of the message will receive all
 * the MSBits of the following bytes which then are simply put there
 * with their MSB stipped away.
 *
 * The resulting message array 'msg' must at least have len + 1 bytes
 * of space to hold the result. The first byte will contain the MSBit
 * from the follow-up bytes.
 *
 * \param bin		pointer to the real binary bytes containing the message to transform
 * \param msg		pointer to the resulting message bytes, each only allowed to contain 7-bit bytes
 * \param len		the length of the binary package (max 7 bytes)
 */
static void ln_bin2msg (uint8_t *bin, uint8_t *msg, int len)
{
	uint8_t *p, mask;

	if (!bin || !msg || len <= 0) return;	// nothing to do
	if (len > 7) len = 7;					// take care not to overwrite other stuff :-)
	p = msg;
	mask = 0x01;
	*p++ = 0;								// initialize the MSBit storage and let p point to &msg[1]
	while (len) {
		if (*bin & 0x80) *msg |= mask;
		*p++ = *bin++ & 0x7F;
		mask <<= 1;
		len--;
	}
}

/**
 * Transfor a sequence of 7-bit bytes to a binary data array holding the
 * corresponding 8-bit bytes. This is the inverser of \ref ln_bin2msg().
 *
 * The first byte of the message contains the MSBits of the followup
 * 7-bit bytes. The target buffer 'bin' must supply space for len bytes
 * and len + 1 bytes from the message are interpreted.
 *
 * \param bin		pointer to the real binary bytes that will contai the resulting 8-bit bytes
 * \param msg		pointer to the original message bytes, each only contain 7-bit bytes
 * \param len		the length of the binary package (max 7 bytes)
 */
static void ln_msg2bin (uint8_t *bin, uint8_t *msg, int len)
{
	uint8_t *p, mask;

	if (!bin || !msg || len <= 0) return;	// nothing to do
	if (len > 7) len = 7;					// take care not to overwrite other stuff :-)
	p = &msg[1];
	mask = 0x01;
	while (len) {
		*bin = *p++;
		if (*msg & mask) *bin |= 0x80;
		bin++;
		mask <<= 1;
		len--;
	}
}

static int ln_lookupSlot (int adr)
{
	int slot;

	for (slot = 1; slot < 120; slot++) {
		if ((slots[slot].adr == adr) && (slots[slot].status != SLOT_FREE)) return slot;
	}

	return -1;
}

static int ln_searchFreeSlot (void)
{
	int slot;

	for (slot = 1; slot < 120; slot++) {
		if (slots[slot].status == SLOT_FREE) return slot;
	}

	return -1;
}

#if 0	// doesn't work as expected
static void ln_sendClearSlot (uint8_t n)
{
	uint8_t blk[LN_MAX_BLOCK_LEN];

	if (n > 0 && n < 120) {
		memset (blk, 0, sizeof(blk));
#if 1
		blk[0] = OPC_SLOT_STAT1;
		blk[1] = n;
		blk[2] = 0;
#else
		blk[0] = OPC_WR_SL_DATA;
		blk[1] = 14;
		blk[3] = n;
		blk[7] = ln_trackstatus();
#endif
		ln_sendBlock(blk);
	}
}
#endif

static void ln_sendSlot (uint8_t n)
{
	uint8_t blk[LN_MAX_BLOCK_LEN];
	ldataT *l;

	if (n == 0) {	// SLOT#0 configuration data
		blk[0] = OPC_SL_RD_DATA;			// Slot read
		blk[1] = 14;						// 14 bytes
		blk[2] = n;							// the slot number as requested
		memcpy (&blk[3], slot0data, 10);	// copy the 10 bytes from specification to the block
	} else {
		if (n >= 120) return;
		l = loco_call(slots[n].adr, false);

		blk[0] = OPC_SL_RD_DATA;			// Slot read
		blk[1] = 14;						// 14 bytes
		blk[2] = n;							// the slot number as requested
		blk[3] = ln_slotstatus (n);
		blk[4] = slots[n].adr & 0x7F;		// loco address 7 LSBits
		if (l) {
			blk[5] = ln_speed2msg(loco_getSpeeds(l->loco), l->speed);
			blk[6] = ((l->funcs[0] & FUNC_LIGHT) << 4) | ((l->funcs[0] & FUNC_F1_F4) >> 1);
			if (!(l->speed & 0x80)) blk[6] |= 0x20;		// loconet personal edition seems to document the direction bit errneous ...
			blk[10] = (l->funcs[0] & FUNC_F5_F8) >> 5;
			slots[n].lastfuncs = l->funcs[0];
			slots[n].lastspeed = l->speed;
		} else {
			blk[5] = blk[6] = blk[10] = 0x00;
			slots[n].lastfuncs = 0;
			slots[n].lastspeed = 0x80;
		}
		blk[7] = ln_trackstatus();
		blk[8] = 0;
		blk[9] = (slots[n].adr >> 7) & 0x7F;
		blk[11] = slots[n].id & 0x7F;
		blk[12] = (slots[n].id >> 7) & 0x7F;
	}
	ln_sendBlock(blk);
}

static int ln_pwrOff (uint8_t *blk)
{
	(void) blk;

	sig_setMode(TM_STOP);
	return 0;
}

static int ln_pwrOn (uint8_t *blk)
{
	(void) blk;

	sig_setMode(TM_GO);
	return 0;
}

static int ln_emergencyStop (uint8_t *blk)
{
	(void) blk;

	sig_setMode(TM_SHORT);
	return 0;
}

static int ln_slotSpeed (uint8_t *blk)
{
	ldataT *l;
	int slot, adr;

	slot = blk[1];
	if (slot > 0 && slot < 120) {
		adr = slots[slot].adr;
		if ((l = loco_call(adr, false)) == NULL) return -1;
//		log_msg (LOG_INFO, "%s(): SLOT #%d loco %d new speed %d\n", __func__, slot, adr, blk[2]);
		if (blk[2] == 1) {	// EMERGENCY STOP
			rq_emergencyStop(adr);
		} else {
			rq_setSpeed(adr, (l->speed & 0x80) | ln_msg2speed(loco_getSpeeds(l->loco), blk[2]));
		}
		slots[slot].lastspeed = l->speed;
	}
	return 0;
}

static int ln_slotDirFunc (uint8_t *blk)
{
	ldataT *l;
	int slot, adr, newspeed;
	uint32_t newfuncs;

	slot = blk[1];
	if (slot > 0 && slot < 120) {
		adr = slots[slot].adr;
		if ((l = loco_call(adr, false)) == NULL) return -1;
		newspeed = (l->speed & 0x7F);
		if ((blk[2] & 0x20) == 0) newspeed |= 0x80;		// loconet personal edition seems to document the direction bit errneous ...
		newfuncs = ((blk[2] & 0x10) >> 4) | ((blk[2] & 0x0F) << 1);
		if ((l->speed & 0x80) != (newspeed & 0x80)) {
			rq_setSpeed(adr, l->speed & 0x80);	// intermediate speed 0
		}
		rq_setSpeed(adr, newspeed);
		rq_setFuncMasked(adr, newfuncs, FUNC_F0_F4);
		slots[slot].lastfuncs = l->funcs[0];
		slots[slot].lastspeed = l->speed;
	}
	return 0;
}

static int ln_slotFunc58 (uint8_t *blk)
{
	ldataT *l;
	int slot, adr;
	uint32_t newfuncs;

	slot = blk[1];
	if (slot > 0 && slot < 120) {
		adr = slots[slot].adr;
		if ((l = loco_call(adr, false)) == NULL) return -1;
		newfuncs = (blk[2] & 0x0F) << 5;
		rq_setFuncMasked(adr, newfuncs, FUNC_F5_F8);
		slots[slot].lastfuncs = l->funcs[0];
	}
	return 0;
}

static int ln_slotFunc912 (uint8_t *blk)
{
	ldataT *l;
	int slot, adr;
	uint32_t newfuncs;

	slot = blk[1];
	if (slot > 0 && slot < 120) {
		adr = slots[slot].adr;
		if ((l = loco_call(adr, false)) == NULL) return -1;
		newfuncs = (blk[2] & 0x0F) << 9;
		log_msg (LOG_INFO, "%s(%d) NEW 0x%08lx MASK 0x%08x\n", __func__, adr, newfuncs, FUNC_F9_F12);
		rq_setFuncMasked(adr, newfuncs, FUNC_F9_F12);
		slots[slot].lastfuncs = l->funcs[0];
	}
	return 0;
}

static int ln_slotFuncUH (uint8_t *blk)
{
	ldataT *l;
	int slot, adr;
	uint32_t newfuncs;

	if (blk[1] != 0x20) return -1;
	slot = blk[2];
	if (slot > 0 && slot < 120) {
		adr = slots[slot].adr;
		if ((l = loco_call(adr, false)) == NULL) return -1;

		switch (blk[3]) {
			case 0x05:
				newfuncs = ((blk[4] & 0x10) << (12 - 4));
				newfuncs |= ((blk[4] & 0x20) << (20 - 5));
				newfuncs |= ((blk[4] & 0x40) << (28 - 6));
				rq_setFuncMasked(adr, newfuncs, FUNC_F12_F20_F28);
				break;
			case 0x07:
				newfuncs = ((blk[4] & 0x7F) << 5);
				rq_setFuncMasked(adr, newfuncs, FUNC_F5_F11);
				break;
			case 0x08:
				newfuncs = ((blk[4] & 0x7F) << 13);
				rq_setFuncMasked(adr, newfuncs, FUNC_F13_F19);
				break;
			case 0x09:
				newfuncs = ((blk[4] & 0x7F) << 21);
				rq_setFuncMasked(adr, newfuncs, FUNC_F21_F27);
				break;
		}
		slots[slot].lastfuncs = l->funcs[0];
	}
	return 0;
}

static int ln_FuncDigitrax (uint8_t *blk)
{
	ldataT *l;
	int adr, slot;
	uint32_t newfuncs;
	uint8_t db[5];
	uint8_t db2[2];

	if (blk[2] != 0x7F) return -1;					// no - this is not an immediate N-Byte paket
	// blk[4] contains the following byte's MSBits
	db[0] = db[1] = db[2] = db[3] = db[4] = 0;		// just to keep compiler happy (maybe used uninitialized)
	ln_msg2bin(db, &blk[4], 5);

	if (!(db[0] & 0x80)) { //short address
		adr = db[0];
		db2[0] = db[1];
		db2[1] = db[2];
	} else if ((db[0] & 0xC0) == 0x80) {			//accessory decoder
		return -1;
	} else {
		if (db[0]>231) return -1;					//reserved
		adr = db[0] | (db[1] << 8);
		db2[0] = db[2];
		db2[1] = db[3];
	}
	if ((l = loco_call(adr, false)) == NULL) return -1;

	if ((db2[0] & 0xF0) == 0xA0) {
		newfuncs = (db2[0] & 0x0F) << 9;
		rq_setFuncMasked(adr, newfuncs, FUNC_F9_F12);
	} else if (db2[0] == 0xDE) {
		newfuncs = db2[1] << 13;
		rq_setFuncMasked(adr, newfuncs, FUNC_F13_F20);
	} else if (db2[0] == 0xDF) {
		newfuncs = db2[1] << 21;
		rq_setFuncMasked(adr, newfuncs, FUNC_F21_F28);
	} else if (db2[0] == 0xD8) {
		newfuncs = (db2[1] & 0x7) << 29;
		rq_setFuncMasked(adr, newfuncs, FUNC_F29_F31);
	} else {
		return -1;
	}
	if ((slot = ln_lookupSlot(adr)) > 0) {
		slots[slot].lastfuncs = l->funcs[0];		// update the found slot with updated function information
	}

	return 0;
}

static int ln_trntSwitch (uint8_t *blk)
{
	int adr;

	adr = (blk[1] | ((blk[2] & 0x0F) << 7)) + 1;		// our internal system counts from 1
	log_msg (LOG_INFO, "%s() ADR %d %s %s\n", __func__, adr, (blk[2] & 0x20) ? "THROWN": "STRAIGHT", (blk[2] & 0x10) ? "ON" : "OFF");
	trnt_switch(adr, !(blk[2] & 0x20), blk[2] & 0x10);
	return 0;
}


/*
 * LocoNet simple feed back modules
 * Addresses can range from 0 to 4095 equivalent to 256 s88 modules
 */
static int ln_Input (uint8_t *blk)
{
#ifdef CENTRAL_FEEDBACK
	uint16_t adr;

	adr = ((blk[2] & 0x0F) << 7) + blk[1];
	adr <<= 1;
	if(blk[2] & 0x20) adr++;

	fb_BitInput(adr + FB_LNET_OFFSET, !!(blk[2] & 0x10));
#else
	volatile uint16_t *input;
	uint16_t adr;

	input = s88_getInputs();
	adr = ((blk[2] & 0x0F) << 7) + blk[1];
	adr <<= 1;
	if(blk[2] & 0x20) adr++;
	input[adr/16] &= ~(0x8000>>(adr % 16));
	if(blk[2] & 0x10) {
		input[adr/16] |= 0x8000>>(adr % 16);
	}
	s88_triggerUpdate();
#endif
	log_msg (LOG_INFO, "%s() Modul %d, input %X %s\n", __func__, adr / 16, adr % 16, (blk[2] & 0x10)? "on" : "off");
	return 0;
}

static int ln_writeSlotStat (uint8_t *blk)
{
	int slot;

	slot = blk[1];
	log_msg (LOG_INFO, "%s(#%d) STATUS 0x%02x (old 0x%02x)\n", __func__, slot, blk[2], ln_slotstatus(slot));
	if (slot > 0 && slot < NUMBER_OF_SLOTS) {
		slots[slot].status = blk[2] >> 4;
		if (slots[slot].status == SLOT_INUSE) ln_controlEvent(slot, 1);
		else ln_controlEvent(slot, 0);
	}
	return 0;
}

static int ln_slotMove (uint8_t *blk)
{
	int src, dest;

	src = blk[1];
	dest = blk[2];
	log_msg (LOG_INFO, "%s(): src #%d dest #%d\n", __func__, src, dest);

	if (src >= NUMBER_OF_SLOTS || dest >= NUMBER_OF_SLOTS) {
		ln_longACK(blk[0], 0);
	} else if (src == 0) {
		if (slot0stack) {		// get Block from DISPATCH if a slot was put there before
			log_msg (LOG_INFO, "%s() DISPATCH GET slot#%d\n", __func__, slot0stack);
			slots[slot0stack].status = SLOT_INUSE;
			ln_sendSlot(slot0stack);
			ln_controlEvent(slot0stack, 1);
			slot0stack = 0;
		} else {				// no slot DISPATCHed: send error message
			log_msg (LOG_INFO, "%s() DISPATCH GET stack is empty\n", __func__);
			ln_longACK(blk[0], 0);
		}
	} else if (dest == 0) {		// mark slot as DISPATCH (which one? --> probably src)
		log_msg (LOG_INFO, "%s() DISPATCH PUT slot#%d\n", __func__, src);
		slots[src].status = SLOT_COMMON;
		slots[src].id = 0;					// zero out slot-ID when dispatching for a new throttle
		slot0stack = src;
		ln_controlEvent(src, 0);
	} else {
		if (src == dest) {
			slots[src].status = SLOT_INUSE;	// NULL-move, occupy this slot
			slots[src].id = 0;
			ln_controlEvent(src, 1);
		}
		ln_sendSlot(dest);
	}
	return 0;
}

static int ln_slotRead (uint8_t *blk)
{
	ln_sendSlot(blk[1]);
	return 0;
}

static int ln_trntQuery (uint8_t *blk)
{
	int adr;
	turnoutT *t;
	uint8_t state;

	adr = (blk[1] | ((blk[2] & 0x0F) << 7)) + 1;		// our internal system counts from 1
	log_msg (LOG_INFO, "%s() ADR %d\n", __func__, adr);
	state = 0;
	t = db_lookupTurnout(adr);
	if (!t || !t->dir) state |= 0x20;
	ln_longACK(blk[0], state);
	return 0;
}

static int ln_reqLoco (uint8_t *blk)
{
	ldataT *l;
	int adr, slot;

	adr = blk[1] << 7 | blk[2];
	log_msg (LOG_INFO, "%s(): Loco %d\n", __func__, adr);
	if (adr <= 0) {
		ln_longACK(blk[0], 0);
		return -1;
	}

	slot = ln_lookupSlot(adr);
	if (slot < 0) slot = ln_searchFreeSlot();
	if (slot < 0) {			// loco not found in any slot and no free slot found
		ln_longACK(blk[0], 0);
		return -1;
	}
	if ((l = loco_call(adr, true)) == NULL) {	// loco could not be activated
		ln_longACK(blk[0], 0);
		return -2;
	}

	if (slots[slot].status == SLOT_FREE) {		// we allocate a new slot and put loco there
		slots[slot].adr = adr;
		slots[slot].id = 0;
		slots[slot].status = SLOT_COMMON;
	}

	ln_sendSlot(slot);
	log_msg (LOG_INFO, "%s() loco %d allocated to SLOT #%d\n", __func__, adr, slot);

	return 0;
}

/**
 * Send 7 bytes of loco configuration data encoded in a 15 byte
 * LocoNet packet with OPCode 0xE5.
 */
static int ln_sendLocConfig(int adr)
{
	ldataT *l;
	uint8_t blk[LN_MAX_BLOCK_LEN];
	uint8_t db[7];					// a temporary storage to handle the MSBs correct

	if ((l = loco_call(adr, true)) == NULL) return -1;

	db[0] = adr & 0xFF;		//loco virtual address, IB-stuff....
	db[1] = adr >> 8;
	// db[2], db[3] format dependant, see switch below
	db[4] = adr & 0xFF;		//loco real address
	db[5] = adr >> 8;
	db[6] = 0x1;
	switch (loco_getSpeeds(l->loco)) {
		case 14:
			if (FMT_IS_MM1(l->loco->fmt)) {
				db[2] = 0xC0;
				db[3] = 0x02;
			} else if (FMT_IS_MM2(l->loco->fmt)) {
				db[2] = 0xC1;
				db[3] = 0x14;
			} else {
				db[2] = 0xD0;
				db[3] = 0x01;
			}
			break;
		case 27:
			db[2] = 0xD1;
			db[3] = 0x01;
			break;
		case 28:
		default:
			db[2] = 0xD2;
			db[3] = 0x01;
			break;
		case 126:
			db[2] = 0xD3;
			db[3] = 0x51;
			break;
	}
    blk[0] = OPC_PEER_XFER;
    blk[1] = 15;		// block len 15 bytes
    blk[2] = 0x00;
    blk[3] = 'I';
    blk[4] = 'K';		// 'K' in reply!
    blk[5] = 0x0B;
	ln_bin2msg(db, &blk[6], 7);
	ln_sendBlock(blk);

	return 0;
}

static int ln_IB_configRequest (uint8_t *blk)
{
	int adr;
	uint8_t db[2];

	if (blk[3] != 'I' || blk[4] != 'B' || blk[5] != 0x0D) return -1;	// no - this is not IB
	// blk[6] contains the following byte's MSBits
	db[0] = db[1] = 0;		// just to keep compiler happy (maybe used uninitialized)
	ln_msg2bin(db, &blk[6], 2);
	adr = db[0] | (db[1] << 8);
	ln_sendLocConfig(adr);
	return 0;
}

static enum fmt ln_fmtFromStatus (uint8_t status)
{
	switch (status & 0x07) {	// not all codes are used (or at least not documented)
		case 0b000:		// 28 steps / 3 BYTE PKT regular mode
			return FMT_DCC_28;
		case 0b001:		// 28 steps / Generate Trinary packets (-> Märklin-Motorola?)
			return FMT_MM2_27B;
		case 0b010:		// 14 steps
			return FMT_DCC_14;
		case 0b011:		// 128 steps (i.e. 126)
			return FMT_DCC_126;
		case 0b100:		// 28 steps / allow advanced DCC consisting
			return FMT_DCC_28;
		case 0b101:		// UNDEFINED
		case 0b110:		// UNDEFINED
			return FMT_UNKNOWN;
		case 0b111:		// 128 steps (i.e. 126) / allow advanced DCC consisting
			return FMT_DCC_126;
	}
	return FMT_UNKNOWN;		// to keep compiler happy
}

static int ln_slotWrite (uint8_t *blk)
{
	locoT *l;
	int n;
//	int i;

	// TODO: maybe we must interpret the written data - for now let's just acknowledge a success
	n = blk[2];		// slot number
	if (n < 0 || n >= NUMBER_OF_SLOTS) {
		ln_longACK(blk[0], 0x00);
		return -1;
	}

//	log_msg (LOG_INFO, "%s(): ", __func__);	// test for manipulation of speed step setting
//	for (i = 0; i < 14; i++) {
//		log_msg (LOG_INFO, " 0x%02x", blk[i]);
//	}
//	putchar ('\n');
	slots[n].id = blk[11] | (blk[12] << 7);		// currently we only accept the ID for FRED(I) throttles
	if (slots[n].status == SLOT_INUSE) {
		if ((l = db_getLoco(slots[n].adr, false)) != NULL) {
			if (db_getSpeeds(ln_fmtFromStatus(blk[3])) != loco_getSpeeds(l)) {
				db_setLocoFmt(slots[n].adr, ln_fmtFromStatus(blk[3]));
			}
		}
		ln_controlEvent(n, 1);
	} else {
		ln_controlEvent(n, 0);
	}
	ln_longACK(blk[0], 0x7F);
	return 0;
}

static bool ln_eventHandler (eventT *e, void *arg)
{
	fbeventT *fbev;
	uint8_t blk[LN_MAX_BLOCK_LEN];
	ldataT *l;
	turnoutT *t;
	int slot, fb;
	uint16_t mask;

	(void) arg;		// not used here

	if (e->tid == rxtask) return true;		// this event is triggered by our own activity - ignore it

	switch (e->ev) {
		case EVENT_SYS_STATUS:
			switch (e->param) {
				case SYSEVENT_STOP:
					blk[0] = OPC_GPOFF;	// Power OFF request
					break;
				case SYSEVENT_HALT:
				case SYSEVENT_GO:
				case SYSEVENT_TESTDRIVE:
					blk[0] = OPC_GPON;	// Power ON request
					break;
				case SYSEVENT_SHORT:
					blk[0] = OPC_IDLE;	// Broadcast Emergency STOP
					break;
				default:
					return true;		// all otzher states are not reported on LocoNet
			}
			ln_sendBlock(blk);
			break;
		case EVENT_LOCO_FUNCTION:
			l = (ldataT *) e->src;
			if ((slot = ln_lookupSlot(l->loco->adr)) < 0) break;	// this loco is not active on LocoNet, so ignore it
			log_msg (LOG_INFO, "%s() update functions for SLOT #%d\n", __func__, slot);
			if ((l->funcs[0] & FUNC_F0_F4) != (slots[slot].lastfuncs & FUNC_F0_F4)) {	// functions F0 to F4 have changed
				blk[0] = OPC_LOCO_DIRF;		// set func (and direction) bits
				blk[1] = slot;
				blk[2] = ((l->funcs[0] & FUNC_LIGHT) << 4) | ((l->funcs[0] & FUNC_F1_F4) >> 1);
				if (!(l->speed & 0x80)) blk[2] |= 0x20;
				ln_sendBlock(blk);
			}
			if ((l->funcs[0] & FUNC_F5_F8) != (slots[slot].lastfuncs & FUNC_F5_F8)) {	// functions F5 to F8 have changed
				blk[0] = OPC_LOCO_SND;		// set func F5 - F8 ("sound") bits
				blk[1] = slot;
				blk[2] = (l->funcs[0] & FUNC_F5_F8) >> 5;
				ln_sendBlock(blk);
			}
			if ((l->funcs[0] & FUNC_F9_F12) != (slots[slot].lastfuncs & FUNC_F9_F12)) {	// functions F9 to F12 have changed
				blk[0] = OPC_LOCO_F9F12;	// set func F9 - F12 bits
				blk[1] = slot;
				blk[2] = (l->funcs[0] & FUNC_F9_F12) >> 9;
				ln_sendBlock(blk);
			}
			if ((l->funcs[0] & FUNC_F12_F20_F28) != (slots[slot].lastfuncs & FUNC_F12_F20_F28)) {	// functions F12, F20 or F28 have changed
				blk[0] = OPC_UHLI_FUN;
				blk[1] = 0x20;
				blk[2] = slot;
				blk[3] = 0x05;
				blk[4] = ((l->funcs[0] & FUNC(12)) >> (12 - 4)) | ((l->funcs[0] & FUNC(20)) >> (20 - 5)) | ((l->funcs[0] & FUNC(28)) >> (28 - 6));
				ln_sendBlock(blk);
			}
			if ((l->funcs[0] & FUNC_F13_F19) != (slots[slot].lastfuncs & FUNC_F13_F19)) {	// functions F13 to F19 have changed
				blk[0] = OPC_UHLI_FUN;
				blk[1] = 0x20;
				blk[2] = slot;
				blk[3] = 0x08;
				blk[4] = ((l->funcs[0] & FUNC_F13_F19) >> 13);
				ln_sendBlock(blk);
			}
			if ((l->funcs[0] & FUNC_F21_F27) != (slots[slot].lastfuncs & FUNC_F21_F27)) {	// functions F21 to F27 have changed
				blk[0] = OPC_UHLI_FUN;
				blk[1] = 0x20;
				blk[2] = slot;
				blk[3] = 0x09;
				blk[4] = ((l->funcs[0] & FUNC_F21_F27) >> 21);
				ln_sendBlock(blk);
			}
			slots[slot].lastfuncs = l->funcs[0];
			break;
		case EVENT_LOCO_SPEED:
			l = (ldataT *) e->src;
			if ((slot = ln_lookupSlot(l->loco->adr)) < 0) break;	// this loco is not active on LocoNet, so ignore it
			log_msg (LOG_INFO, "%s() update speed for SLOT #%d\n", __func__, slot);
			if ((l->speed & 0x80) != (slots[slot].lastspeed & 0x80)) {	// direction changed - send "dirf"
				blk[0] = OPC_LOCO_DIRF;		// set func (and direction) bits
				blk[1] = slot;
				blk[2] = ((l->funcs[0] & FUNC_LIGHT) << 4) | ((l->funcs[0] & FUNC_F1_F4) >> 1);
				if (!(l->speed & 0x80)) blk[2] |= 0x20;
				ln_sendBlock(blk);
			}
			if ((l->speed & 0x7F) != (slots[slot].lastspeed & 0x7F)) {	// speed magnitude changed - send "spd"
				blk[0] = OPC_LOCO_SPD;		// set speed
				blk[1] = slot;
				blk[2] = ln_speed2msg(loco_getSpeeds(l->loco), l->speed);
				ln_sendBlock(blk);
			}
			slots[slot].lastspeed = l->speed;
			break;
		case EVENT_TURNOUT:
			t = (turnoutT *) e->src;
			if (t->adr <= 2048) {
				blk[0] = OPC_SW_REQ;
				blk[1] = (t->adr - 1) & 0x7F;
				blk[2] = ((t->adr - 1) >> 7) & 0x0F;
				if (t->on) blk[2] |= 0x10;
				if (!t->dir) blk[2] |= 0x20;
				ln_sendBlock(blk);
			}
			break;
		case EVENT_FBNEW:
			fbev = (fbeventT *) e->src;
			fb = fbev->module * 16;
			mask = 0x8000;
			while (mask) {
				if (fbev->chgflag & mask) {
					blk[0] = OPC_INPUT_REP;
					blk[1] = (fb >> 1) & 0x7F;
					blk[2] = 0x40 | ((fb >> 8) & 0x0F);		// bit #6 is a control bit an must be set to 1, 0 is reserved for future use
					if (fb & 1) blk[2] |= 0x20;				// the LSB of the feedback address
					if (fbev->status & mask) blk[2] |= 0x10;
					ln_sendBlock(blk);
				}
				mask >>= 1;
				fb++;
			}
			break;
		default:		// we handle no other events
			break;
	}
	return true;
}

#if PACKET_DUMP != 0
static void ln_dumpPacket (uint8_t *blk, bool tx)
{
	const struct decoder *d;
	int n;

	if (!blk) return;
	n = ln_blockLen(blk);
	d = lncmds;
	while (d->len) {
		if ((d->cmd == blk[0]) && (d->len == n)) break;
		d++;
	}
	if (d->name) log_msg (LOG_INFO, "%sLN %s(0x%02x) len=%d", (tx) ? ">>>> " : "<<<< ", d->name, blk[0],n );
	blk++;
	while (n > 2) {
		log_msg (LOG_INFO, " 0x%02x", *blk++);
		n--;
	}
	putchar('\n');
}
#endif

static int ln_txBlock (uint8_t *b)
{
	unsigned long notification;

	if (!b || txreq.req) return -1;					// either no data or the txreq block is still in use (should never happen!)
	txreq.len = ln_blockLen(b);
	if (txreq.len > sizeof(txreq.data)) return -2;	// block is too large to be transmitted (maybe you should adjust LN_MAX_BLOCK_LEN)
	memcpy (txreq.data, b, txreq.len);

#if PACKET_DUMP != 0
	ln_dumpPacket(b, true);
#else
//	log_msg (LOG_INFO, "%s() transmitting block 0x%02x len %d\n", __func__, txreq.data[0], txreq.len);
#endif

	xTaskNotifyStateClear(NULL);
	txreq.retry = 0;
	txreq.req = true;					// the request stays set until the block is delievered or retries are all used up
	SET_BIT(LPUART1->CR1, USART_CR1_TXEIE_TXFNFIE);		// enable TX-FIFO not full interrupt to signal the request
	// wait for result
	do {
		notification = ulTaskNotifyTake(pdTRUE, 100);
		switch (notification) {
			case COMM_TXOK:
//				log_msg (LOG_INFO, "%s(try %d): TX OK\n", __func__, txreq.retry);
				break;
			case COMM_TIMEOUT:
				log_msg (LOG_INFO, "%s(try %d): TIMEOUT tx-idx %d rx-idx %d\n", __func__, txreq.retry, txreq.txidx, txreq.cmpidx);
				break;
			case COMM_COLLISION:
				log_msg (LOG_INFO, "%s(try %d): COLLISION tx-idx %d rx-idx %d\n", __func__, txreq.retry, txreq.txidx, txreq.cmpidx);
				break;
			case COMM_TXFAIL:
				log_msg (LOG_INFO, "%s(try %d): TX FAIL tx-idx %d rx-idx %d\n", __func__, txreq.retry, txreq.txidx, txreq.cmpidx);
				break;
			default:
				log_msg (LOG_INFO, "%s(try %d): notification %lu tx-idx %d rx-idx %d\n", __func__, txreq.retry, notification, txreq.txidx, txreq.cmpidx);
				break;
		}
	} while ((notification != COMM_TXOK) && (notification != COMM_TXFAIL));

	return 0;
}

static void ln_sender (void *pvParameter)
{
	uint8_t blk[LN_MAX_BLOCK_LEN];

	(void) pvParameter;

	txtask = xTaskGetCurrentTaskHandle();

	for (;;) {
		if (xQueueReceive(txqueue, blk, portMAX_DELAY)) {
			ln_txBlock(blk);
		}
	}
}

int ln_dispatchLoco (int adr)
{
	int slot;

	db_getLoco (adr, true);
	slot = ln_lookupSlot (adr);
	if (slot < 1) {
		slot = ln_searchFreeSlot();
		if (slot < 0) return -1;
		slots[slot].adr = adr;
	}
	slots[slot].id = 0;
	slots[slot].status = SLOT_COMMON;
	slot0stack = slot;
	ln_controlEvent(slot, 0);
	return slot;
}

void vLocoNet (void *pvParameter)
{
	uint8_t blk[LN_MAX_BLOCK_LEN];
	const struct decoder *d;
	int n;

	(void) pvParameter;

	lpuart1_init();
	tim16_init();

	log_msg (LOG_INFO, "%s() startup\n", __func__);

	rxqueue = xQueueCreate(8, LN_MAX_BLOCK_LEN);
	txqueue = xQueueCreate(8, LN_MAX_BLOCK_LEN);

	if (!txqueue || !rxqueue) {
		log_error ("%s(): cannot create RX/TX queues\n", __func__);
		if (txqueue) vQueueDelete(txqueue);
		if (rxqueue) vQueueDelete(rxqueue);
		txqueue = rxqueue = NULL;	// just in case someone will restart this thread later
		vTaskDelete(NULL);
	}

	rxtask = xTaskGetCurrentTaskHandle();
	backoff = 0;		// superfluous - but for clarity: we are MASTER and have no addition al timeouts!
	xTaskCreate(ln_sender, "LN-TX", 1024, NULL, 1, NULL);
	event_register(EVENT_SYS_STATUS, ln_eventHandler, NULL, 0);
	event_register(EVENT_LOCO_FUNCTION, ln_eventHandler, NULL, 0);
	event_register(EVENT_LOCO_SPEED, ln_eventHandler, NULL, 0);
	event_register(EVENT_TURNOUT, ln_eventHandler, NULL, 0);
	event_register(EVENT_FBNEW, ln_eventHandler, NULL, 0);
//	_lnet_setModules(0, cnf_getconfig()->lnetModules);

#if 0	// this doesn't work. perhaps we can go with a slow write status (OPC_SLOT_STAT1, 0xB5 -> tested -> NO, doesn't work)
	for (n = 1; n < 120; n++) {
		ln_sendClearSlot(n);
		vTaskDelay(2);
	}
#endif

	for (;;) {
		if (xQueueReceive(rxqueue, blk, portMAX_DELAY)) {
#if PACKET_DUMP != 0
			ln_dumpPacket(blk, false);
#endif
			n = ln_blockLen(blk);
			d = lncmds;
			while (d->len) {
				if ((d->cmd == blk[0]) && (d->len == n)) {
					if (d->func) d->func (blk);
					break;
				}
				d++;
			}
#if PACKET_DUMP == 0		// if we are dumping the packet anyways, this message is superfous
			if (d->func == NULL) {
				log_msg (LOG_INFO, "%s(): unsupported CMD 0x%02x LEN=%d\n", __func__, blk[0], n);
			}
#endif
		}
	}
}

/**
 * (Re-)start the timout counter TIM16 with the given amount of microseconds.
 *
 * \param us	number of microseconds for the timer to fire a timeout
 */
static void ln_startTimeout (int us)
{
	CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);
	TIM16->CNT = 0;
	TIM16->ARR = us;
	SET_BIT(TIM16->CR1, TIM_CR1_CEN);
}

#if 0	// this function is not really needed!
/**
 * Stop the timout counter TIM16.
 * This should not be necessary due to the timer disabling itself
 * after a timeout dur to the one shot configuration.
 */
static void ln_stopTimeout (void)
{
	CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);
}
#endif

static enum irqstat {
	IRQSTAT_IDLE = 0,
	IRQSTAT_RECEIVE,
	IRQSTAT_GAP,
	IRQSTAT_BACKOFF,
	IRQSTAT_TRANSMIT,
	IRQSTAT_COLLISION
} irqcs = IRQSTAT_IDLE;

/**
 * The LPUART1 interrupt handles character reception and transmission and takes care of the
 * communication status changes.
 * Any received character (re-)triggers the timeout counter TIM16 with the standard inter
 * packet gap time of 20 bits (i.e. 1.200µs, LN_PACKET_TIMEOUT). If state is not IRQSTAT_TRANSMIT,
 * it is set to IRQSTAT_RECEIVE.
 * Every byte with a set MSBit (0x80) resets the receiver index (or compare index if the state
 * is IRQSTAT_TRANSMIT) to zero.
 *
 * Reception
 * ---------
 *
 * After having received two bytes for a packet, the packet length is calculated and
 * then reception comments until this length is reached (which well be immedeately, if packet
 * length comes out to be two bytes). The XOR checksum is calculated with every received byte.
 * If the packet is complete (according to it's expected length) and the checksum reads 0xFF
 * the block is valid and posted to the receive queue. The status is changed to IRQSTATE_GAP.
 * It is also changed to GAP if a stray byte without set MSBit is receieved as first byte.
 *
 * Transmission
 * ------------
 * When the TX FIFO NOT FULL interrupt is enabled and triggered,
 */
void LPUART_IRQHandler (void)
{
	BaseType_t xHigherPriorityTaskWoken = 0;
	int c;

	while (LPUART1->ISR & USART_ISR_RXNE_RXFNE) {
		c = LPUART1->RDR;
		ln_startTimeout(LN_PACKET_TIMEOUT);	// GAP / timeout (re-)starts with every received byte
		if (irqcs == IRQSTAT_TRANSMIT) {	// we are receiving the echo of the transmitted block, compare it with the TX buffer
			if (c & 0x80) {			// this begins a new block
				txreq.cmpidx = 0;
			}
			if (c != txreq.data[txreq.cmpidx]) {
				irqcs = IRQSTAT_COLLISION;
				LPUART1->RQR = USART_RQR_TXFRQ | USART_RQR_RXFRQ | USART_RQR_SBKRQ;		// clear all LPUART FIFOs and send a break (only 10 bits!)
				CLEAR_BIT(LPUART1->CR1, USART_CR1_TXEIE_TXFNFIE);		// don't transmit any further bytes
				if (txreq.retry >= LN_TX_RETRY_ATTEMPTS) {
					if (txtask) xTaskNotifyFromISR(txtask, COMM_TXFAIL, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);		// final fail message
					txreq.req = false;		// give up!
				} else {
					if (txtask) xTaskNotifyFromISR(txtask, COMM_COLLISION, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);	// intermediate error code
					txreq.txidx = txreq.cmpidx = 0;
				}
			} else if (++txreq.cmpidx >= txreq.len) {
				irqcs = IRQSTAT_GAP;
				if (txtask) xTaskNotifyFromISR(txtask, COMM_TXOK, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);			// final success message
				txreq.req = false;		// we are finished!
			}
		} else {							// we are receiving charcaters from other pepole
			irqcs = IRQSTAT_RECEIVE;		// always set this state irrespective of the previous state
			if (c & 0x80) {					// this begins a new block
				rxblock.idx = 0;
				rxblock.data[rxblock.idx++] = rxblock.chksum = c;	// checksum is initialized with start byte (chksum = 0 ^ c)
				rxblock.len = 2;			// the bare minimum
			} else if (rxblock.idx == 0) {
				irqcs = IRQSTAT_GAP;		// unexpected char - switch back to GAP state
			} else {
				if (rxblock.idx < rxblock.len) {
					rxblock.data[rxblock.idx++] = c & 0xFF;
					rxblock.chksum ^= c & 0xFF;
				}
				if (rxblock.idx == 2) {			// at this point we can check for the real packet length
					rxblock.len = ln_blockLen(rxblock.data);
				}
				if (rxblock.idx == rxblock.len) {
					irqcs = IRQSTAT_GAP;		// the inter packet GAP timeout is now running
					if (rxblock.chksum == 0xFF) {
						xQueueSendFromISR(rxqueue, rxblock.data, &xHigherPriorityTaskWoken);
					}
					rxblock.idx = 0;
				}
			}
		}
	}

	// check if we are ready to start or continue a transmission
	if ((LPUART1->CR1 & USART_CR1_TXEIE_TXFNFIE) && (LPUART1->ISR & USART_ISR_TXE_TXFNF)) {		// TX FIFO NOT FULL interrupt enabled and triggered
		if ((irqcs == IRQSTAT_IDLE) && !(LPUART1->ISR & USART_ISR_BUSY)) {		// if we just come from idle state, check if a startbit has been detected
			irqcs = IRQSTAT_TRANSMIT;
			txreq.txidx = txreq.cmpidx = 0;
			txreq.retry++;						// count this attempt
		}
		if (irqcs == IRQSTAT_TRANSMIT) {		// only if we successfully transitioned to state IRQSTAT_TRANSMIT, we may send the block
			while ((LPUART1->ISR & USART_ISR_TXE_TXFNF) && (txreq.txidx < txreq.len)) {		// push as much bytes as possible to LPUART1 FIFO
				LPUART1->TDR = txreq.data[txreq.txidx++];
			}
			ln_startTimeout(LN_PACKET_TIMEOUT);	// start initial transmit timeout
		}
		if ((irqcs != IRQSTAT_TRANSMIT) || txreq.txidx >= txreq.len) {		// we couldn't change state to IRQSTAT_TRANSMIT or all bytes are put to the TX-FIFO
			CLEAR_BIT(LPUART1->CR1, USART_CR1_TXEIE_TXFNFIE);
		}
	}

	LPUART1->ICR = LPUART_ICR_ALL;		// clear all interrupt flags
	NVIC_ClearPendingIRQ(LPUART1_IRQn);
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}

/**
 * The TIM16 interrupt handles gaps, backoff and timeouts.
 * It is called every time, TIM16 CNT reaches its ARR (Automatic Reload Register).
 * Since the timer is set to it's one-shot mode, it is disabled when entering this handler.
 *
 * If a BACKOFF time is defined and we are not in IRQSTAT_BACKOFF, the state transitions
 * to IRQSTAT_BACKOFF by reprogramming the timer via ln_startTimeout(). In all other cases
 * it transistions to IRQSTAT_IDLE.
 *
 * In all cases we can reset the receiver index, because any ongoing reception will be
 * interrupted due to exzessive timeout. We also can clear the indizees of the transmitter
 * buffer because of this timeout.
 *
 * If the final state is IRQSTAT_IDLE we check for a still existing transmission request and,
 * if it is active, we enable the LPUART1 transmitter interrupt to try restarting the block-
 * transmission.
 */
void TIM16_IRQHandler (void)
{
	BaseType_t xHigherPriorityTaskWoken = 0;

	if (TIM16->SR & TIM_SR_UIF) {	// the timer will now be disabled, because it is set to one pulse mode
		// timeout triggered - reset state to IDLE and clear reception
		if (irqcs == IRQSTAT_TRANSMIT) {		// timout receiving the echo from the transmit attempt
			if (txreq.retry >= LN_TX_RETRY_ATTEMPTS) {
				if (txtask) xTaskNotifyFromISR(txtask, COMM_TXFAIL, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);		// final fail message
				txreq.req = false;		// give up!
			} else {
				if (txtask) xTaskNotifyFromISR(txtask, COMM_TIMEOUT, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);	// intermediate error code
			}
		}
		if (irqcs != IRQSTAT_BACKOFF && backoff > 0) {
			ln_startTimeout(backoff);
			irqcs = IRQSTAT_BACKOFF;
		} else {
			irqcs = IRQSTAT_IDLE;
		}

		rxblock.idx = txreq.txidx = txreq.cmpidx = 0;
		if (txreq.req && irqcs == IRQSTAT_IDLE) {			// if the request is still valid, we can start transmission from beginning
			SET_BIT(LPUART1->CR1, USART_CR1_TXEIE_TXFNFIE);	// enable TX interrupt to retry sending the block
		}
	}

	TIM16->SR = 0;		// clear all interrupt flags
	NVIC_ClearPendingIRQ(TIM16_IRQn);
	portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
