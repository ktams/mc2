/**
 * @file bidibus.c
 *
 * @author Andi
 * @date   04.04.2020
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

/**
 * @file
 *
 * BiDiBus uses USART2 in half duplex with RS485 drive enable.
 * The frame format is 9 data bits, no parity, 1 stopbit (9n1) at 500kBit/s.
 * There are several timing constraints detailed on www.bidib.org/bidibus/bidibus.html
 *
 * The only responsibility of this module is handling the BiDiBus traffic, giving
 * potential devices a chance to register and take care of vanished devices.
 * This module doesn't handle message sequence numbers nor any kind of higher
 * intelligence.
 *
 * The module has it's own local device table that only contains the modules found on
 * the bus. It only records the 8-bit address and the UID of the devices. Furthermore
 * it forwards packets up and down, report communication errors and attaches or
 * detaches devices. It may be queried for a device table.
 *
 * The device table represents the level below this interface and virtually includes
 * this interface itself (as defined by the BiDiB standard). It can handle device
 * addresses between 1 and 63 (limitation of the BiDiBus protocol, only supporting
 * a six bit address).
 *
 * This will result in the following interaction with upper layers:
 *   - accept a nodelist reset (when upper layer is switching between SERVER and CONTROLLER mode)
 *   - accept messages to be forwared on the bus (downstream)
 *   - forward received non-local messages from the bus (upstream)
 *   - report new and lost nodes
 *   - report communication errors for queried nodes
 *   - query and iterate over the internal node list
 */

#include <stdio.h>
#include <string.h>
#include "rb2.h"
#include "bidib.h"

#define MAX_MESSAGE_SIZE		64			///< size (in bytes) of messages in a block
#define HIGHWATER_FILL			35			///< if a node fills a packet more than this level, it is queried once more to give it more bandwith
#define BIDIBUS_TIMEOUT			250			///< a timeout after which a node is considered disconnected
#define BIDIBUS_PING_TIMEOUT	170			///< a timeout after which a node should be pinged if no other traffic was sent
#define BIDIBUS_XFER_TIMEOUT	5			///< if communication for a single PACKET lasts longer give up and take it as timeout
#define TXQUEUELEN				64			///< maximum number of concurrently queued up messages for DOWNSTREAM

// flags for the node tab
#define NTAB_REPORT_DISABLED	0x0001		///< we received a MSG_SYS:DISABLE and therefore should not post updates
#define NTAB_REPORT_NODELOST	0x0002		///< last nodetab change was a node that we lost (else it was a new node added to the table)

#define NTAB_REPORT_RETRIES		16			///< according to specs we should try 16 times to report a nodetab change
#define NTAB_REPORT_TIMEOUT		250			///< the report retries should be fired every 250ms

// define some communication result codes
#define COMMRES_TIMEOUT			0			///< ulTaskNotifyTake() returns zero, which means that a timeout on waiting occured
#define COMMRES_TX_OK			1			///< transmission was OK
#define COMMRES_RX_OK			2			///< reception was OK
#define COMMRES_LOGON_EMPTY		3			///< the LOGON was not answered by anyone
#define COMMRES_LOGON_SINGLE	4			///< a valid answer was received for the LOGON - must have been a single node
#define COMMRES_LOGON_MULTIPLE	5			///< the LOGON was answered by multiple nodes, so character errors or other misfitting stuff was received
#define COMMRES_TX_FAILED		6			///< transmission was disturbed
#define COMMRES_RX_NOANSWER		7			///< reception timed out with no characters received
#define COMMRES_RX_TIMEOUT		8			///< reception timed out (either at start or after reception had started)
#define COMMRES_RX_CRC			9			///< CRC of received block was wrong
#define COMMRES_RX_LENGTH		10			///< A length above the maximum supported was received

struct bidibus_packet {
	int			idx;						///< receive or transmit index
	uint8_t		data[MAX_MESSAGE_SIZE + 2];	///< the message(s) (add two bytes for P_LENGTH and CRC)
};

/**
 * A structure that is used only for the first level on the BiDiBus.
 * It is intended to manage bus queries merely on the hardware level.
 * The system representation of these nodes is done parallel using
 * struct bidibnode.
 */
struct bidibus_node {
	struct bidibus_node		*next;			///< linked list of nodes
	TickType_t				 alive;			///< timer for checking if the node is off bus (250ms)
	uint8_t					 adr;			///< the bus address between 1 and 63, the controller uses adr 0 and is not managed with this struct
	uint8_t					 uid[BIDIB_UID_LEN];	///< the UID of the node
};

struct bidibus_ntab {
	struct bidibus_node		*busnodes;		///< linked list of known subnodes
	int						 flags;			///< some flags
	TickType_t				 ntab_to;		///< timeout for nodetab changes - invalid nodetab reported until it is stable
	uint8_t					 version;		///< version of the nodetab (wrap 255 -> 1)
};

/**
 * The states for the interrupt handler
 */
enum busstate {
	BUSSTATE_IDLE = 0,						///< bus is idle waiting for the next system command
	BUSSTATE_WAITTX,						///< we will transmit our own packet after a short delay (we received our POLL(0) command)
	BUSSTATE_TXPACKET,						///< we now are transmitting our own packet or wainting for interpacket gap after transmission
	BUSSTATE_TXERROR,						///< any mismatch in received vs. transmitted character or receiving less or more than transmitted is a TX-Error
	BUSSTATE_RXPACKET,						///< we receive a packet from another node after having received our POLL(x) command
	BUSSTATE_LOGON,							///< try to receive logon messages from new nodes
	BUSSTATE_ERROR,							///< a reception error occured, wait for reception timed out and ignore this packet
};

static struct bidibus_packet rxpacket;		///< the currently received packet
static struct bidibus_packet txpacket;		///< a packet for sending out
static TaskHandle_t task;					///< the BiDiBus task that gets informed about TX and RX results
static QueueHandle_t txpipe;				///< a dedicated TX pipe with bidibmsg_t pointers
static struct bidibus_ntab ntab;			///< the local bus table
static bidibmsg_t * volatile reset;			///< if this packet pointer is set, the packet is transferred to bus and reinit including a 1s pause is inserted

static const uint8_t crc_array[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

static const uint8_t parity[] = {
   0x00,	//  0: 0000	0	E
   0x80,	//  1: 0001	1	O
   0x80,	//  2: 0010	1	O
   0x00,	//  3: 0011	2	E
   0x80,	//  4: 0100	1	O
   0x00,	//  5: 0101	2	E
   0x00,	//  6: 0110	2	E
   0x80,	//  7: 0111	3	O
   0x80,	//  8: 1000	1	O
   0x00,	//  9: 1001	2	E
   0x00,	// 10: 1010	2	E
   0x80,	// 11: 1011	3	O
   0x00,	// 12: 1100	2	E
   0x80,	// 13: 1101	3	O
   0x80,	// 14: 1110	3	O
   0x00		// 15: 1111	4	E
};

/*
 * ===================================================================================
 * Hardware initialisation and helpers
 * ===================================================================================
 */

static void usart2_init (void)
{
	uint32_t cr1, cr3;

	USART2->CR1 = 0;									// disable USART2
	USART2->CR2 = USART_CR2_RTOEN;						// use 1 stop bit and enable the receiver timeout function

	cr1 = USART_CR1_FIFOEN;								// enable FIFO mode
	cr1 |= USART_CR1_M0 | USART_CR1_TE | USART_CR1_RE;	// set 9 bits of data, enable transmitter and receiver
	// driver (de-)assertion timings: each time is set to 1/2 bit time, i.e. 1µs
	cr1 |= 20 << USART_CR1_DEAT_Pos;						// 8 / 16 bit times (i.e. half a bit time)
	cr1 |= 12 << USART_CR1_DEDT_Pos;						// 8 / 16 bit times (i.e. half a bit time, 1µs)
	USART2->CR1 = cr1;

	cr3 = (0b010 << USART_CR3_RXFTCFG_Pos);				// set RX-FIFO threshold interrupt at half full (but do not enable it for now)
	cr3 |= USART_CR3_DEM | USART_CR3_HDSEL;				// set driver enable mode with driver enable active high, half duplex mode
	USART2->CR3 = cr3;

	USART2->PRESC = 0b0101;								// prescaler = 10 -> 100MHz / 10 = 10MHz kernel clock
	USART2->BRR = 20;									// 10MHz / 20 -> 500kbit/s

	NVIC_SetPriority(USART2_IRQn, 12);
	NVIC_ClearPendingIRQ(USART2_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->ICR = 0xFFFFFFFF;							// clear all interrupt flags

	SET_BIT (USART2->CR1, USART_CR1_UE);				// enable the USART
	SET_BIT (USART2->CR1, USART_CR1_RXNEIE_RXFNEIE);	// enable RX FIFO not empty interrupt
}

//static void BDBus_setupACK (void)
//{
//	// setup EXTI6 pin (PD6) as BIDI ACK interrupt
//	MODIFY_REG(SYSCFG->EXTICR[1], SYSCFG_EXTICR2_EXTI6_Msk, SYSCFG_EXTICR2_EXTI6_PD);		// Port D for external IRQ6 (-> PD6)
//	CLEAR_BIT(EXTI->RTSR1, EXTI_RTSR1_TR6);		// no interrupt on rising edge
//	SET_BIT(EXTI->FTSR1, EXTI_FTSR1_TR6);		// generate interrupt on falling edge
//	SET_BIT(EXTI->IMR1, EXTI_IMR1_IM6);			// enable the interrupt in the interrupt mask register
//	EXTI->PR1 = EXTI_PR1_PR6;					// clear pending bit by writing a 1 to the PR register position
//    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);			// clear a possibly pending interrupt request
//    NVIC_SetPriority(EXTI9_5_IRQn, 14);
//	NVIC_EnableIRQ(EXTI9_5_IRQn);				// enable this interrupt in NVIC
//
//	log_msg (LOG_INFO, "%s(): ACK IRQ ready\n", __func__);
//}

static void BDBus_flushRxTx (void)
{
	CLEAR_BIT (USART2->CR1, USART_CR1_RTOIE);						// disable receiver timeout interrupt
	USART2->RQR = USART_RQR_TXFRQ | USART_RQR_RXFRQ;				// flush queues
	while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) taskYIELD();		// wait for the TX flush to be finished
	xTaskNotifyStateClear(NULL);
}

/*
 * ===================================================================================
 * BUS-Tokens and helpers
 * ===================================================================================
 */

/**
 * Calculate the paraity bit (MSB, bit 7) for command bytes.
 *
 * \param cmdbyte	the command byte that needs a correct parity bit
 * \return			the commandbyte that now contains an adequate parity bit
 */
static uint8_t BDBus_parity (uint8_t cmdbyte)
{
	uint8_t c;

	c = cmdbyte & 0x7F;
	c |= parity[c & 0x0f] ^ parity[c >> 4];
	return c;
}

/**
 * Check if a command byte has got the correct parity. This function is used
 * by the interrupt handler to verify it received a valid command (b.t.w. with
 * a set multiprocessor bit #8).
 *
 * \param c			the received character to be checked
 * \return			true, if parity is OK, false otherwise
 */
static bool BDBus_chkparity (uint16_t c)
{
	c &= 0xFF;		// strip MP-bit (bit 8)
	return !(parity[c & 0x0f] ^ parity[c >> 4]);
}

/**
 * Calculate the checksum (CRC8) over the given block of data.
 *
 * \param data		pointer to the data bytes
 * \param len		number of bytes in data to process
 * \return			the resulting CRC byte (should be 0x00 if a check with included CRC8 is performed)
 */
static uint8_t BDBus_chksum (uint8_t *data, int len)
{
	uint8_t crc = 0;

	while (len > 0) {
		crc = crc_array[*data++ ^ crc];
		len--;
	}

	return crc;
}

/**
 * Send a POLL for the given address on the bus.
 *
 * \param adr	the address that is to be polled (0 = interface itself, 1 .. 63 possible client)
 * \return		the communication outcome from the inetrrupt handler
 */
static int BDBus_poll (struct bidibus_node *n)
{
	uint8_t cmd;

	BDBus_flushRxTx();
	cmd = BDBus_parity((n) ? n->adr : 0);
	USART2->TDR = 0x100 | cmd;
	return ulTaskNotifyTake(pdTRUE, BIDIBUS_XFER_TIMEOUT);
}

#if 0	/* currently unused (deprecated) */
/**
 * Send a POWER-UP token on the bus.
 *
 * \return		the communication outcome from the inetrrupt handler
 */
static int BDBus_pwrup (void)
{
	BDBus_flushRxTx();
	USART2->TDR = 0x100 | BIDIBUS_POWER_UPx_par;
	return ulTaskNotifyTake(pdTRUE, BIDIBUS_XFER_TIMEOUT);
}
#endif

/**
 * Send a LOGON token on the bus.
 *
 * \return		the communication outcome from the inetrrupt handler
 */
static int BDBus_logon (void)
{
	BDBus_flushRxTx();
	USART2->TDR = 0x100 | BIDIBUS_LOGON_par;
	return ulTaskNotifyTake(pdTRUE, BIDIBUS_XFER_TIMEOUT);
}

#if 0	/* currently unused */
/**
 * Send a BUSY token on the bus.
 *
 * \return		the communication outcome from the inetrrupt handler
 */
static int BDBus_busy (void)
{
	BDBus_flushRxTx();
	USART2->TDR = 0x100 | BIDIBUS_BUSY_par;
	return ulTaskNotifyTake(pdTRUE, BIDIBUS_XFER_TIMEOUT);
}
#endif

/*
 * ===================================================================================
 * LOCAL Messages
 * ===================================================================================
 */

static void BDBus_localLogonAck (struct bidibus_node *n)
{
	bidibmsg_t *bm;
	uint8_t data[8];

	if (n) {
		data[0] = n->adr;			// the new address on our bus
		memcpy (&data[1], n->uid, BIDIB_UID_LEN);
		if ((bm = bidib_genMessage(NULL, MSG_LOCAL_LOGON_ACK, 8, data)) != NULL) {
			log_msg (LOG_BIDIB, "%s() for node %d\n", __func__, n->adr);
			if (xQueueSend(txpipe, &bm, 10) != pdTRUE) {
				log_error ("%s() cannot queue up message\n", __func__);
				free (bm);
			} else {
				n->alive = tim_timeout(BIDIBUS_TIMEOUT);
			}
		}
	}
}

static void BDBus_localLogonReject (uint8_t *uid)
{
	bidibmsg_t *bm;
	uint8_t data[8];

	log_msg (LOG_BIDIB, "%s() %s\n", __func__, bidib_formatUID(uid));
	memcpy (data, uid, 7);	// the UID of the node to reject
	if ((bm = bidib_genMessage(NULL, MSG_LOCAL_LOGON_REJECTED, 7, data)) != NULL) {
		if (xQueueSend(txpipe, &bm, 10) != pdTRUE) free (bm);
	}
	// TODO inform upper host with BIDIB_ERR_BUS (too much nodes in this layer)
}

#if 0	/* currently unused */
static void BDBus_localPing (int n)
{
	bidibmsg_t *bm;

	log_msg (LOG_BIDIB, "%s() for %d\n", __func__, n);
	if ((bm = bidib_genMessage(n, 0, MSG_LOCAL_PING, 0, NULL)) != NULL) {
		if (xQueueSend(txpipe, &bm, 10) != pdTRUE) free (bm);
	}
}
#endif

/*
 * ===================================================================================
 * Node Handling
 * ===================================================================================
 */

/**
 * Find a free bus address in the range of 1 to 63 to assign to a new
 * node on the bus. If no more free addresses are available, a zero is
 * returned indicating that the table is full.
 *
 * As a prerequisite the node 0 (we as the master of the BiDiBus) must
 * always be created first to occupy the node #0. This master node is
 * not getting it's address from this function though.
 *
 * \param virtual	a boolean value which denotes the creation of an internal node
 * 					which is not part of the BiDiBus-polling cycle
 * \return			a valid address in the range 1 .. 63 (physical node) or 64 .. 255 (virtual)
 * 					or 0 in case of a full device table
 */
static uint8_t BDBus_findFreeAddress (bool virtual)
{
	uint8_t adr;
	struct bidibus_node *bn;

	adr = (virtual) ? BIDIBUS_MAX_NODEADR + 1 : 0;
	bn = ntab.busnodes;
	while (bn) {
		if (bn->adr > adr) break;
		if (bn->adr == adr) adr++;
		bn = bn->next;
	}
	if (virtual && adr <= BIDIBUS_MAX_NODEADR) return 0;
	if (!virtual && adr > BIDIBUS_MAX_NODEADR) return 0;
	return adr;
}

/**
 * Allocate and prepare a busnode structure for the new UID.
 * The new node gets a free address and is inserted sorted into
 * the list of active BiDiBus-nodes.
 *
 * If everything goes right, the new node will get a LOGON-ACK
 * packet with it's new bus address and the new node will be
 * announced upstream.
 *
 * If all addresses are exhausted or no memory can be allocated
 * for the new node, a LOGON-REJECT is sent. Nothing will be
 * reported upstream in this case.
 *
 * \param uid		the seven bytes UID of the new node
 */
static void BDBus_allocNode (uint8_t *uid)
{
	struct bidibus_node *bn, **pp;
	uint8_t adr;

	if (!uid) return;

	if ((adr = BDBus_findFreeAddress(false)) > 0) {
		if ((bn = malloc(sizeof(*bn))) != NULL) {
			bn->adr = adr;
			memcpy (bn->uid, uid, BIDIB_UID_LEN);
			pp = &ntab.busnodes;
			while (*pp && (*pp)->adr < adr) pp = &(*pp)->next;
			bn->next = *pp;
			*pp = bn;
			if (++ntab.version == 0) ntab.version = 1;
			BDBus_localLogonAck(bn);
			BDBnode_newBusNode(bn->uid, bn->adr, ntab.version);
			return;
		}
	}
	BDBus_localLogonReject(uid);	// something went wrong
}

/**
 * Take the given node out of the node list and free the associated memory.
 * Report the loss of this node upstream.
 *
 * \param n		pointer to the local busnode that vanished
 */
static void BDBus_releaseNode (struct bidibus_node *n)
{
	struct bidibus_node **pp;

	if (n) {
		pp = &ntab.busnodes;
		while (*pp && (*pp) != n) pp = &(*pp)->next;
		if (*pp == n) {
			*pp = n->next;
		}
		if (++ntab.version == 0) ntab.version = 1;
		log_msg (LOG_BIDIB, "%s(): lost node %d UID=%s\n", __func__, n->adr, bidib_formatUID(n->uid));
		BDBnode_lostBusNode(n->uid, n->adr, ntab.version);
		free (n);
	}
}

static void BDBus_clearNode (int adr)
{
	struct bidibus_node *n;

	if (adr <= 0) return;	// never delete ourself!
	n = ntab.busnodes;
	while (n && n->adr != adr) n = n->next;
	BDBus_releaseNode(n);
}

static void BDBus_clearAllNodes (void)
{
	struct bidibus_node *bn;

	log_msg (LOG_BIDIB, "%s()\n", __func__);
	BDBnode_changeACK(NULL, NULL);	// clear nodetab change reporting
	while ((bn = ntab.busnodes) != NULL) {
		ntab.busnodes = bn->next;
		free (bn);
	}
	if ((bn = calloc(1, sizeof(*bn))) != NULL) {
		memcpy (bn->uid, myUID, BIDIB_UID_LEN);
		ntab.busnodes = bn;
	}
}

/*
 * ===================================================================================
 * Message handling and communication loop
 * ===================================================================================
 */

/**
 * Pack messages into the static transmit buffer. If more than a single message is
 * available in the messages list, we try to pack as much as we can into the single
 * buffer. All messages successfully packed into the buffer are freed (memory released).
 *
 * If there are messages left that didn't fit into the current packet, these messages
 * are returned as the left over list and should be packed in the next round.
 *
 * Messages that are greater than the maximum transmission size (MAX_MESSAGE_SIZE)
 * are ignored and thrown away.
 *
 * 07.04.2021 discovered by Markus Herzog: The basic receiver routine of many nodes
 * has a bug not allowing packets of maximum size. Because of a faulty consistency check
 * they may only receive packets with a maximum message payload of 62 bytes instead of
 * 64 Bytes as stated in the SPEC. Therefor we use (MAX_MESSAGE_SIZE - 2) as the longest
 * payload allowed.
 *
 * A special handling is done for MSG_SYS_RESET sent as broadcast. This means that we
 * have to clear our nodelist and must not use any reference to it. As we are currently
 * handling our own send window (node 0, not physically in nodelist) we can free the
 * list here without problems.
 *
 * \param bm		a list of concatenated BiDiB messages
 * \return			the left over message list or NULL, if all messages could be packed
 * 					into the buffer
 */
#define MAX_PAYLOAD		(MAX_MESSAGE_SIZE - 2)

static bidibmsg_t *BDBus_packMessages (bidibmsg_t *bm)
{
	bidibmsg_t *tmp;
	uint8_t *p, len;

	p = &txpacket.data[1];
	while (bm) {
		if (p > &txpacket.data[1] && bm->msg == MSG_LOCAL_LOGON_ACK) break;		// we will isolate MSG_LOCAL_LOGON_ACK in a single packet!
		len = bidib_packSize(bm);
		if (len > MAX_PAYLOAD) {			// this message simply cannot be forwarded on BiDiBus
			log_error ("%s(): oversized message discarded (len=%d)\n", __func__, len);
		} else {
			if (len > (MAX_PAYLOAD - (p - txpacket.data - 1))) break;	// this message doesn't fit in the current packet anymore
			p = bidib_packMessage(bm, p);
			bidib_debugSingleMessage(__func__, bm, false);
			if (bm->adrstack == 0 && bm->msg == MSG_SYS_RESET) {	// we must not use our nodelist beyond this point
				BDBus_clearAllNodes();
			} else if ((bm->adrstack & 0x00FFFFFF) == 0 && bm->msg == MSG_SYS_RESET) {	// reset a single node directly beneath us
				BDBus_clearNode (bm->adrstack >> 24);
			}
		}
		tmp = bm;
		bm = bm->next;
		if (tmp->msg == MSG_LOCAL_LOGON_ACK) {		// we will isolate MSG_LOCAL_LOGON_ACK in a single packet!
			free (tmp);
			break;
		}
		free(tmp);		// free only this single message
	}
	len = p - txpacket.data - 1;
	txpacket.data[0] = len;
	txpacket.data[len + 1] = BDBus_chksum(txpacket.data, len + 1);

#if 0
	// DEBUG: test this packet
	log_msg (LOG_BIDIB, "%s(): LEN=%d CRC=0x%02x should be 0x%02x\n\t", __func__, txpacket.data[0],
			txpacket.data[txpacket.data[0] + 1], BDBus_chksum(txpacket.data, txpacket.data[0] + 1));
	for (int i = 0; i <= txpacket.data[0] + 1; i++) {
		log_msg (LOG_BIDIB, "0x%02x ", txpacket.data[i]);
	}
	putchar ('\n');
#endif

	return bm;		// return the rest of the packets that did not fit in a single BiDiBus packet
}

/**
 * Unpack a packet buffer to a BiDiB message list. It is assumed, that only
 * valid packets are tried to be unpacked (i.e. a valid P_LEN and CRC8 is found
 * in the packet before calling this function).
 *
 * \param pkt		the pointer to the bytes that make up the BiDiBus packet
 * 					(the packet contains a length information as first byte)
 * \param adr		the address of the scanned node (will only be used to report
 * 					errneous messages upstream)
 * \return			a linked list of BiDiB messages
 */
static bidibmsg_t *BDBus_unpackMessages (uint8_t *pkt, uint8_t adr)
{
	bidibmsg_t *msgs;
	int packetlen;

	if (!pkt) return NULL;
//	log_msg (LOG_BIDIB, "%s() LEN=%d from Node %d\n", __func__, *pkt, adr);
	if ((packetlen = *pkt++) == 0) return NULL;		// pkt now points to the first message
	msgs = bidib_unpackMessages(pkt, packetlen, adr);
	bidib_debugMessages(__func__, msgs, (adr) ? true : false);

	return msgs;
}

/**
 * Handle received messages. Local messages are directly handled here.
 * All others are forwared upstream with the address of the node we received
 * the packets from added to the address stack.
 *
 * For handling the messages, they are taken from the list and formed to
 * single messages outside of a list. Locally handled messages are freed
 * after their handling, uplinked messages continue to exist.
 *
 * \param msgs		the linked list of messages to handle
 * \param n			the node we received the messages from
 */
static void BDBus_handleMessages (bidibmsg_t *msgs, struct bidibus_node *n)
{
	bidibmsg_t *m;

	while ((m = msgs) != NULL) {
		msgs = msgs->next;
		m->next = NULL;			// cut the message from the message stream
		if (bidib_isLocal(m->msg)) {		// handle local messages here
			switch (m->msg) {
				case MSG_LOCAL_LOGON: break;		// this message is directly handled in LOGON procedure - it should never be seen
				case MSG_LOCAL_PONG: break;			// Answer to a MSG_LOCAL_PING - can be ignored
				case MSG_LOCAL_LOGOFF:				// the node wants to LOG OFF from the bus
					BDBus_releaseNode(n);			// release this node
					bidib_freeMessages(msgs);		// drop all further messages as this should be the last message ever seen from this node
					msgs = NULL;					// take care to end the loop
					break;
				case MSG_LOCAL_ANNOUNCE: break;		// TODO the node announces something - not handled yet
				case MSG_LOCAL_BIDIB_UP: break;		// Answer to MSG_LOCAL_BIDIB_DOWN - a wrapper mode for proxy communication, not used here
					break;
			}
			bidib_freeMessages(m);					// locally handled messages must be freed after they are interpreted
		} else {							// forward normal communication upstream
			BDBnode_uplink(NULL, m);
		}
	}
}

/**
 * Check for the next node to poll.
 * If the timeout for LOGON queries is over, we return 0 instead of a normal
 * poll index to send out a MSG_LOGON. If no active nodes are known, we permanently
 * send LOGON messages.
 *
 * Remarks for timings (reference: Artemis, software by Wolfgang Kufer
 * 	- 2.700+ LOGON polls/s if no node is attached
 * 	- with one node it generates 4.100+ polls/s, alternating between node poll and LOGON
 *	- with two nodes it generates around 3.000 polls/s, alternating between nodes and LOGON with no recognisable system
 * Here are our results with the FULLSPEED macro defined (26.05.2021 A.Kre: macro removed, we now always use the full speed method):
 * 	- 7.000+ LOGON polls/s before the first node attaches
 * 	- 7.400+ polls/s incl. LOGON with 2 nodes attached
 *
 * \return		pointer to the next poll node or NULL, if we want to send a LOGON query
 */
static struct bidibus_node *BDBus_nextPoll (void)
{
	static int adr;

	struct bidibus_node *n;

	// if there are no nodes beside our node 0 yet, we scan every 10ms for new nodes
	if (!ntab.busnodes || !ntab.busnodes->next) {
		return NULL;
	}


	n = ntab.busnodes;
	while (n && n->adr <= adr && n->adr <= BIDIBUS_MAX_NODEADR) n = n->next;
	if (!n || n->adr > BIDIBUS_MAX_NODEADR) {		// insert a LOGON (virtual nodes also mark the end of the list)
		adr = -1;
		return NULL;
	}
	adr = n->adr;
	return n;
}

void BDBus_resetBus (bidibmsg_t *msg)
{
	if (reset) bidib_freeMessages(reset);
	reset = msg;
	while (reset) vTaskDelay(1);
}

/**
 * Forward a message down on the BiDiBus.
 * The message memory will be freed either directly if forwarding is not possible
 * or later after it is sent out on the bus.
 *
 * \param bm	the message to send on BiDiBus
 */
void BDBus_sendMessage (bidibmsg_t *bm)
{
	if (!bm) return;
	if (!txpipe) {
		log_error ("%s() message queue not setup yet\n", __func__);
		free (bm);
	} else {
		if (xQueueSend(txpipe, &bm, 10) != pdTRUE) {
			log_error ("%s() cannot queue up message\n", __func__);
			free (bm);
		}
	}
}

#ifdef BIDIB_SNIFFER
static volatile uint8_t poll;

void BDBus (void *pvParameter)
{
	bidibmsg_t *msgs;
	unsigned long rc;
	uint16_t logon_cnt = 0;

	(void) pvParameter;

	task = xTaskGetCurrentTaskHandle();

	BDBus_setupACK();
	usart2_init();

	log_msg (LOG_INFO, "%s(): running\n", __func__);
	for (;;) {
		rc = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		switch (rc) {
			case COMMRES_TIMEOUT:
				log_error ("%s(): TIMEOUT\n", __func__);
				usart2_init();
				break;
			case COMMRES_TX_OK:
//				log_msg (LOG_BIDIB, "%s(): TX OK with %d bytes\n", __func__, txpacket.idx);
				break;
			case COMMRES_RX_OK:
				switch (rxpacket.data[0]) {
					case 0:
//						log_msg (LOG_BIDIB, "%s(%d): Node ready\n", __func__, poll);
						break;
					case 1:
						log_msg (LOG_BIDIB, "%s(%d): Node BUSY\n", __func__, poll);
						break;
					case 2:
					case 3:
						log_msg (LOG_BIDIB, "%s(%d): RESERVED (%d)\n", __func__, poll, rxpacket.data[0]);
						break;
					default:
						msgs = BDBus_unpackMessages(rxpacket.data, poll);
						bidib_freeMessages(msgs);
						break;
				}
				break;
			case COMMRES_LOGON_EMPTY:
//				log_msg (LOG_BIDIB, "%s(): LOGON #%u Empty\n", __func__, logon_cnt);
				logon_cnt++;
				break;
			case COMMRES_LOGON_SINGLE:
				log_msg (LOG_BIDIB, "%s(): LOGON #%u %s\n", __func__, logon_cnt, bidib_formatUID(&rxpacket.data[5]));
				logon_cnt++;
				break;
			case COMMRES_LOGON_MULTIPLE:
				log_msg (LOG_BIDIB, "%s(): LOGON #%u Multiple\n", __func__, logon_cnt);
				logon_cnt++;
				break;
			case COMMRES_TX_FAILED:
				log_error ("%s(): TX failed\n", __func__);
				break;
			case COMMRES_RX_NOANSWER:
				log_msg (LOG_BIDIB, "%s(RX %d) no answer\n", __func__, poll);
				break;
			case COMMRES_RX_TIMEOUT:
				log_msg (LOG_BIDIB, "%s(RX %d) Timeout after %d chars\n", __func__, poll, rxpacket.idx);
				break;
			case COMMRES_RX_CRC:
				log_msg (LOG_BIDIB, "%s%s(RX %d) CRC error%s\n", log_ansiColor(RED, NONE, BOLD), __func__, poll, ANSI_RESET);
				break;
			case COMMRES_RX_LENGTH:
				log_msg (LOG_BIDIB, "%s%s(RX %d) LENGTH error (announced %d / is %d bytes)%s\n", log_ansiColor(RED, NONE, BOLD),
						__func__, poll, rxpacket.data[0] + 2, rxpacket.idx, ANSI_RESET);
				break;
		}
	}
}
#else	/* !BIDIB_SNIFFER */
void BDBus (void *pvParameter)
{
	bidibmsg_t *msgs, *tx, **txpp;
	struct bidibus_node *n;
	unsigned long rc = 0;
	bool repoll = false;
	uint16_t logon_cnt;

	(void) pvParameter;

	task = xTaskGetCurrentTaskHandle();

//	BDBus_setupACK();
	usart2_init();
	txpipe = xQueueCreate(TXQUEUELEN, sizeof(bidibmsg_t *));

	tx = NULL;
	n = NULL;
	logon_cnt = 0;
	BDBus_clearAllNodes();		// establishes node 0 as the node avaliable

	log_msg (LOG_INFO, "%s(): running\n", __func__);
	for (;;) {
		BDBnode_pollChangeReport();					// check if we must report a changed node tab
		if (reset) {								// we are requested to reset the bus after having transmitted this packet
			bidib_freeMessages(tx);
			while (xQueueReceive(txpipe, &tx, 10) == pdTRUE) {
				bidib_freeMessages(tx);
			}
			tx = reset;
			BDBus_clearAllNodes();					// establishes node 0 as the only node avaliable
			while (tx) {
				tx = BDBus_packMessages(tx);			// pack all messages until the packet is full or no more messages are available
				do {
					rc = BDBus_poll(NULL);
				} while (rc != COMMRES_TX_OK);
			}
			ntab.ntab_to = xTaskGetTickCount() + 2000;	// 1s for vTaskDelay() below plus another second to wait for table stability
			reset = NULL;								// release the waiting thread that triggered the reset
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		if (tim_isover(ntab.ntab_to)) {				// check if the node tab reporting timeout is over and we may transmit a valid nodetab upstream
			ntab.ntab_to = 0;
		}

		if (rc == COMMRES_TX_FAILED) {				// something went wrong transmitting the current block - we should repeat that packet
			vTaskDelay(2);
			rc = BDBus_poll(NULL);
		} else {
			// now check for the next node have access to the bus - if the node pointer is set and a repoll is requested we will repeat the last poll slot
			if (!n || !repoll) n = BDBus_nextPoll();
			if (n) {		// poll a node
				if (n->adr == 0) {				// we are polled to transmit our own packets
					repoll = false;						// in any case never let repoll be true for node 0
					txpp = &tx;
					while (*txpp) txpp = &(*txpp)->next;		// go to end of list
					while (xQueueReceive(txpipe, txpp, 0) == pdTRUE) {
						while (*txpp) txpp = &(*txpp)->next;	// go to end of expanded list
					}
					if (tx) {									// check if we really have something to transmit
						tx = BDBus_packMessages(tx);			// pack all messages until the packet is full or no more messages are available
						rc = BDBus_poll(NULL);
					} else {									// else continue with a different node
						continue;
					}
				} else {						// a subnode is given bandwith on the bus
	//				log_msg (LOG_BIDIB, "%s(): Next POLL to %d\n", __func__, n->adr);
					rc = BDBus_poll(n);
				}
			} else {							// allow a bus logon
	//			log_msg (LOG_BIDIB, "%s(): LOGON poll\n", __func__);
				rc = BDBus_logon();
				logon_cnt++;
			}
		}

		// we should repoll a node once if we successfully received a block of adequate length
		// and the repeat was not set before (i.e. only one repeat should be allowed per node per round)
		if (rc == COMMRES_RX_OK && !repoll && rxpacket.data[0] > HIGHWATER_FILL) repoll = true;
		else repoll = false;
		switch (rc) {
			case COMMRES_TIMEOUT:
				log_error ("%s(%d): TIMEOUT\n", __func__, (n) ? n->adr : -1);
//				usart2_init();
				break;
			case COMMRES_TX_OK:
//				log_msg (LOG_BIDIB, "%s(): TX OK with %d bytes\n", __func__, txpacket.idx);
				break;
			case COMMRES_RX_OK:
				n->alive = tim_timeout(BIDIBUS_TIMEOUT);
//				if (tim_isover(n->ping)) BDBus_localPing (n->nodeadr);
				switch (rxpacket.data[0]) {
					case 0:
//						log_msg (LOG_BIDIB, "%s(%d): Node ready\n", __func__, n->adr);
						break;
					case 1:
						log_msg (LOG_BIDIB, "%s(%d): Node BUSY\n", __func__, n->adr);
						break;
					case 2:
					case 3:
						log_msg (LOG_BIDIB, "%s(%d): RESERVED (%d)\n", __func__, n->adr, rxpacket.data[0]);
						break;
					default:
						msgs = BDBus_unpackMessages(rxpacket.data, n->adr);
						BDBus_handleMessages (msgs, n);
						break;
				}
				break;
			case COMMRES_LOGON_EMPTY:
//				log_msg (LOG_BIDIB, "%s(): LOGON #%u Empty\n", __func__, logon_cnt);
				break;
			case COMMRES_LOGON_SINGLE:
				log_msg (LOG_BIDIB, "%s(): LOGON #%u %s\n", __func__, logon_cnt, bidib_formatUID(&rxpacket.data[5]));
				BDBus_allocNode(&rxpacket.data[5]);
				if (ntab.ntab_to) ntab.ntab_to = xTaskGetTickCount() + 500;	// if timeout is running, elongate it
				break;
			case COMMRES_LOGON_MULTIPLE:
//				log_msg (LOG_BIDIB, "%s(): LOGON #%u Multiple\n", __func__, logon_cnt);
				if (ntab.ntab_to) ntab.ntab_to = xTaskGetTickCount() + 500;	// if timeout is running, elongate it
				break;
			case COMMRES_TX_FAILED:
				log_error ("%s(): TX failed\n", __func__);
				break;
			case COMMRES_RX_NOANSWER:
			case COMMRES_RX_TIMEOUT:
//				log_msg (LOG_BIDIB, "%s(RX %d) Timeout after %d chars\n", __func__, n->adr, rxpacket.idx);
				if (n && n->adr && (!n->alive || tim_isover(n->alive))) {
					log_msg (LOG_BIDIB, "%s(RX %d) Timeout after %d chars\n", __func__, n->adr, rxpacket.idx);
					BDBus_releaseNode(n);
				} else if (rxpacket.idx > 0) {
					bidib_busError(BIDIB_ERR_SUBTIME, n->adr);
				}
				break;
			case COMMRES_RX_CRC:
				log_msg (LOG_BIDIB, "%s%s(RX %d) CRC error%s\n", log_ansiColor(RED, NONE, BOLD), __func__, n->adr, ANSI_RESET);
				bidib_busError(BIDIB_ERR_SUBCRC, n->adr);
				break;
			case COMMRES_RX_LENGTH:
				if (rxpacket.idx > 0) {	// for an unknown reason this code sometimes is set even when no characters were received
					log_msg (LOG_BIDIB, "%s%s(RX %d) LENGTH error (announced %d / is %d bytes)%s\n", log_ansiColor(RED, NONE, BOLD),
							__func__, n->adr, rxpacket.data[0] + 2, rxpacket.idx, ANSI_RESET);
					bidib_busError(BIDIB_ERR_SUBPAKET, n->adr);
				}
				break;
		}
	}
}
#endif	/* !BIDIB_SNIFFER */

/*
 * The IRQ handler tracks a state machine, that is driven by received characters. As we always
 * receive the echo from the characters we transmit, the status changes are simply accomplished
 * by sending out a start character from upper level whenever no active communication is going
 * on.
 *
 * A completed transaction is reported back to the upper layer by setting the task's notification
 * value. Except for the logon state, where malformed characters can be received due to collisions
 * from multiple clients answering, any errneous character leads to the idle state invalidating
 * the current transaction. Timeouts are defined as serial bit times counting from the last
 * received stop bit. So if we are speaking of a 10 bit timeout (20µs), it means 20µs of silence
 * after a received serial frame. In other words: this is not an absolute timeout, it restarts
 * with every character received and thus signals a free line, where noone is sending anymore.
 *
 * The state machine starts in BUSSTATE_IDLE and waits for the first character with the MP-bit
 * (bit 8) set and a valid parity bit (bit 7). From this idle state we always get to one of the
 * other states:
 *
 * BUSSTATE_TXPACKET is reached when a poll on address 0 is received. This will set the receiver
 * timeout to 5 bits (10µs) and the timeout interrupt then triggers sending of the prepared block.
 * The transaction is complete, when all sent bytes are received back.
 *
 * BUSSTATE_RXPACKET is reached when a poll to an address > 0 is seen. The RX-timeout is set to
 * 10 bittimes (20µs) and if the reception doesn't start within this timeout, the timeout is
 * reported back and transaction ends. As soon as a character is received, the timeout is expanded
 * to 25 bittimes (50µs) and when this timeout triggers before the block to receive is complete,
 * the transaction is stopped and thrown away. As soon as the block is completed with a valid CRC
 * the block is reported to the upper layer.
 *
 * BUSSTATE_LOGON is entered when the BIDIBUS_LOGON token is received. The timeout is set to 25 bits
 * (50µs) and there are three possible things that can happen. The easiest one is when no new nodes
 * are connected and therefore no answer is received. In that case the timeout triggers without any
 * characters received. In case there is just one new node, we will receive a valid block and so know
 * what node we have. The third scenario is when multiple nodes are answering, generating collisions.
 * These collisions lead to characters received but an invalid block beeing produced, probably with
 * defective characters in between. This must then be resolved by the upper layer.
 */
static void usart2_rxtimeout (int us)
{
	int to;

	to = (us + 1) >> 1;		// timout is programmed in bit-times - @ 500kBaud each bit takes 2µs
	if (to > 0) {
		USART2->RTOR = to & 0x00FFFFFF;
		SET_BIT (USART2->ICR, USART_ICR_RTOCF);		// clear timeout interrupt flag
		SET_BIT (USART2->CR1, USART_CR1_RTOIE);		// enable the timeout interrupt
	} else {
		SET_BIT (USART2->ICR, USART_ICR_RTOCF);		// clear timeout interrupt flag
		CLEAR_BIT (USART2->CR1, USART_CR1_RTOIE);	// disable the timeout interrupt
	}
}

void USART2_IRQHandler (void)
{
	static enum busstate state;
	static int rxerrors;
	static int txcompare;
	static uint8_t crc;

	uint16_t c;
	int len = 0;

	BaseType_t xHigherPriorityTaskWoken = 0;

	// check for receiver timeout
	if ((USART2->CR1 & USART_CR1_RTOIE) && (USART2->ISR & USART_ISR_RTOF)) {
		CLEAR_BIT (USART2->CR1, USART_CR1_RTOIE);		// disable receiver timeout interrupt (this should always be a oneshot)
		USART2->ICR = USART_ICR_RTOCF;
		switch (state) {
			case BUSSTATE_IDLE:		// ignore this interrupt - it should not have happened!
				xTaskNotifyFromISR(task, COMMRES_RX_LENGTH, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				break;
			case BUSSTATE_TXPACKET:	// packet transmission is over after a short interpacket gap
				len = txpacket.data[0];
				if (len < 4) len = 1;			// P_LENGTH is special message - standard packet have at least 4 bytes
				else len += 2;					// we account for the P_LENGTH itself and the CRC
				if (txcompare == len) {
					xTaskNotifyFromISR(task, COMMRES_TX_OK, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				} else {
					xTaskNotifyFromISR(task, COMMRES_TX_FAILED, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				}
				state = BUSSTATE_IDLE;
				rxerrors = 0;
				break;
			case BUSSTATE_TXERROR:	// we have had an error in transmission
				xTaskNotifyFromISR(task, COMMRES_TX_FAILED, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				state = BUSSTATE_IDLE;
				rxerrors = 0;
				break;
			case BUSSTATE_WAITTX:	// we have had a short pause - now let's start transmission
				SET_BIT (USART2->CR1, USART_CR1_TXEIE_TXFNFIE);
				state = BUSSTATE_TXPACKET;
				usart2_rxtimeout(20);		// after 20µs of pause we terminate the TX transaction
				txcompare = 0;
				break;
			case BUSSTATE_ERROR:
			case BUSSTATE_RXPACKET:
				if (rxpacket.idx > 0) {
					len = rxpacket.data[0];
					if (len < 4) len = 1;			// P_LENGTH is special message and has no CRC - standard packet have at least 4 bytes and need CRC
					else len += 2;					// we account for the P_LENGTH itself and the CRC
					if (rxpacket.idx >= len) {		// packet reception complete
						if (len == 1 || crc == 0) xTaskNotifyFromISR(task, COMMRES_RX_OK, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
						else xTaskNotifyFromISR(task, COMMRES_RX_CRC, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
					} else {						// timeout when packet incomplete
						xTaskNotifyFromISR(task, COMMRES_RX_TIMEOUT, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
					}
				} else {
					xTaskNotifyFromISR(task, COMMRES_RX_NOANSWER, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				}
				state = BUSSTATE_IDLE;
				rxerrors = 0;
				break;
			case BUSSTATE_LOGON:	// the logon sequence is over - check packet
				if (rxpacket.idx == 0) {															// no answer at all
					xTaskNotifyFromISR(task, COMMRES_LOGON_EMPTY, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				} else if (rxpacket.idx == (rxpacket.data[0] + 2) && crc == 0 && !rxerrors) {		// a single correct answer
					xTaskNotifyFromISR(task, COMMRES_LOGON_SINGLE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				} else {																			// multiple answers resulting in collisions
					xTaskNotifyFromISR(task, COMMRES_LOGON_MULTIPLE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				}
				state = BUSSTATE_IDLE;
				rxerrors = 0;
				break;
		}
	}

	while ((USART2->CR1 & USART_CR1_RXNEIE_RXFNEIE) && (USART2->ISR & USART_ISR_RXNE_RXFNE)) {
		if (USART2->ISR & USART_ISR_PE) {		// we don't use parity, so this should never happen!
			USART2->ICR = USART_ICR_PECF;
			rxerrors++;
		}
		if (USART2->ISR & USART_ISR_FE) {
			USART2->ICR = USART_ICR_FECF;
			rxerrors++;
		}
//		if (USART2->ISR & USART_ISR_NE) {
//			USART2->ICR = USART_ICR_NECF;
//			rxerrors++;
//		}
		if (USART2->ISR & USART_ISR_ORE) {
			USART2->ICR = USART_ICR_ORECF;
			rxerrors++;
		}
		c = USART2->RDR;
		if (state == BUSSTATE_LOGON) {		// in BUSSATE_LOGON there is a good chance we receive any kind of illegal characters (collisions)
//			if (c & 0x100) rxerrors++;				// this is just an error as the other USART errors
		} else {							// in all other states, errors from USART are treated as fatal
			if (rxerrors) state = BUSSTATE_ERROR;	// any kind of error invalidates the current state
			if (c & 0x100) state = BUSSTATE_IDLE;	// a set MP-bit (bit 8) leads to idle state
		}
		switch (state) {
			case BUSSTATE_IDLE:
				// we wait for a character with the MP bit (bit 8) set with even parity in lower 8 bits
				if (!(c & 0x100) || !BDBus_chkparity(c)) {	// if the character doesn't fulfill these requirements, we simply ignore it
					rxerrors = 0;
					break;
				}
				rxerrors = 0;
				if (c & BIDIBUS_SYS_MSG) {			// Bit 6 set: a system command
					switch (c & 0x7F) {
						case BIDIBUS_LOGON:
							state = BUSSTATE_LOGON;
							rxpacket.idx = 0;
							crc = 0;
#ifdef BIDIB_SNIFFER
							usart2_rxtimeout(80);
#else
							usart2_rxtimeout(100);		// give nodes 100µs to start sending their answer
#endif
							break;
						case BIDIBUS_BUSY:
							break;
					}
				} else {							// Bit 6 clear: a poll command
#ifdef BIDIB_SNIFFER
					state = BUSSTATE_RXPACKET;
					poll = c & 0x3F;
					rxpacket.idx = 0;
					rxpacket.data[0] = 0;
					crc = 0;
					usart2_rxtimeout(20);
#else
					if ((c & 0x3F) == 0) {				// lower 6 bits are the address to poll, if zero send out the txpacket
						state = BUSSTATE_WAITTX;
						txpacket.idx = 0;
						usart2_rxtimeout(10);			// delay 10µs before start sending the packet
					} else {
						state = BUSSTATE_RXPACKET;
						rxpacket.idx = 0;
						crc = 0;
						usart2_rxtimeout(30);			// give nodes 30µs to start sending their answer
					}
#endif
				}
				break;
			case BUSSTATE_WAITTX:			// this is an error - we received something while waiting for a gap between poll(0) and start of transmission
				/* this character nevertheless should be ignored! */
				break;
			case BUSSTATE_TXERROR:			// any characters in these states are ignored, a character timeout will end the communication
			case BUSSTATE_ERROR:
				break;
			case BUSSTATE_TXPACKET:
				if ((c != txpacket.data[txcompare++]) || rxerrors) {
					state = BUSSTATE_TXERROR;
					usart2_rxtimeout(50);
				}
				break;
			case BUSSTATE_RXPACKET:
				if (rxpacket.idx < (int) sizeof(rxpacket.data)) rxpacket.data[rxpacket.idx++] = c;
				len = rxpacket.data[0];
				if (len < 4) len = 1;			// P_LENGTH is special message and has no CRC - standard packet have at least 4 bytes and need CRC
				else len += 2;					// we account for the P_LENGTH itself and the CRC
				if (len > (int) sizeof(rxpacket.data)) {		// the announced packet is too large - ignore it!
					state = BUSSTATE_IDLE;
					break;
				}
				crc = crc_array[(c & 0xFF) ^ crc];
				if (rxpacket.idx >= len) {		// packet reception complete
#ifdef BIDIB_SNIFFER
					usart2_rxtimeout(6);		// wait for 6µs of silence - the inter-packet-gap's minimum is 10µs
#else
					usart2_rxtimeout(20);		// wait for 20µs of silence - the inter-packet-gap's minimum is 10µs
#endif
				}
				break;
			case BUSSTATE_LOGON:
				// we try to receive every character and don't know if the block is valid up to a timeout
				if (rxpacket.idx < 15) {		// a logon packet may never be longer than this (as per the SPEC it is 13 bytes)
					rxpacket.data[rxpacket.idx++] = c;
				} else {
					usart2_rxtimeout(0);
					xTaskNotifyFromISR(task, COMMRES_LOGON_MULTIPLE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
					state = BUSSTATE_IDLE;
				}
				crc = crc_array[(c & 0xFF) ^ crc];
				break;
		}
	}

	len = txpacket.data[0];
	if (len < 4) len = 1;			// P_LENGTH is special message - standard packet have at least 4 bytes
	else len += 2;					// we account for the P_LENGTH itself and the CRC
	while ((USART2->CR1 & USART_CR1_TXEIE_TXFNFIE) && (USART2->ISR & USART_ISR_TXE_TXFNF)) {
		if (txpacket.idx < len) {
			USART2->TDR = txpacket.data[txpacket.idx++];
		} else {	// all characters are transmitted, disable TXFNF interrupt and wait for echoed characters
			CLEAR_BIT (USART2->CR1, USART_CR1_TXEIE_TXFNFIE);
		}
	}

	NVIC_ClearPendingIRQ(USART2_IRQn);
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}

//void EXTI9_5_IRQHandler(void)
//{
//	if (EXTI->PR1 & EXTI_PR1_PR6) {				// EXTI6 is pending, inform signal generation of this ACK
//		EXTI->PR1 = EXTI_PR1_PR6;				// clear pending bit by writing a 1 to the register position
//		sig_ACK();
//	}
//
//	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);			// clear pending interrupt request
//}
