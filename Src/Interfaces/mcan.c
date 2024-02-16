/*
 * mcan.c
 *
 *  Created on: 24.10.2020
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
#include "decoder.h"
#include "events.h"
#include "timers.h"
#include "config.h"
#include "bidib.h"

#define CAN_WORDS_PER_MSG		4			///< the number of 32bit words used for every buffer position (max. 8 data bytes)
#define CAN_WORDS_PER_TXEVENT	2			///< the number of 32bit words used for every TX event entry
#define CAN_RXFIFO0_BUFFERS		64			///< the number of entries in RX FIFO0
#define CAN_TXQUEUE_BUFFERS		32			///< the number of entries in TX QUEUE
#define CAN_TXEVENT_BUFFERS		32			///< the number of entries in TX event QUEUE
// NOTE: the CAN RAM buffers are located at SRAMCAN_BASE (0x4000AC00) - this is not really documented anywhere ...
// NOTE: the CAN RAM buffers MUST only be accessed with 32 bit width

#define CAN_EIDMASK		0x1FFFFFFFu			///< the mask for 29 extended identifier bits
#define CAN_SIDMASK		0x000007FFu			///< the mask for 11 standard identifier bits

#define RXQUEUE_LEN				16			///< number of entries in the RTOS-queue for received frames
#define TXQUEUE_LEN				16			///< number of entries in the RTOS-queue for frames to transmit

#define CAN_MAXUNIT				16
#define ALIVE_VALUE				10			// full live

#define CAN_SYS					0x00
#define CAN_SUB_STOP			0x00
#define CAN_SUB_GO				0x01

#define CAN_SW					0x18
#define CAN_BL					0x1B
#define CAN_LS					0x04
#define CAN_LD					0x05
#define CAN_LF					0x06
#define CAN_AC					0x0B
#define CAN_S88					0x11

typedef union {
	struct {
		unsigned		id:29;				///< the CAN id (either standard or extended) of the frame
		unsigned		rtr:1;				///< remote transmission request
		unsigned		xtd:1;				///< extended identifyer (i.e. the ID is a 29 bit extended ID)
		unsigned		esi:1;				///< error state indicator (unused here)
		unsigned		rxts:16;			///< RX time stamp
		unsigned		dlc:4;				///< number of data bytes in frame (max 8)
		unsigned		brs:1;				///< bitrate switch (unused here)
		unsigned		fdf:1;				///< FD format (extended DLC-coding and CRC, unused here)
		unsigned		:2;					///< two reserved bits
		unsigned		fidx:7;				///< the filter that matched this packet (invalid if ANMF = 1)
		unsigned		anmf:1;				///< "accepted non-matching frame" - i.e. no filter needed to globally accept this frame
		union {
			uint8_t			data[8];		///< the eight data bytes
			uint32_t		wdata[2];		///< the same data in 32 bit words
		};
	};
	uint32_t	rb[CAN_WORDS_PER_MSG];		///< all the data in four 32bit words
} canrxbuf;

typedef union {
	struct {
		unsigned		id:29;				///< the CAN id (either standard or extended) of the frame
		unsigned		rtr:1;				///< remote transmission request
		unsigned		xtd:1;				///< extended identifyer (i.e. the ID is a 29 bit extended ID)
		unsigned		esi:1;				///< error state indicator (unused here)
		unsigned		:16;				///< 16 reserved bits
		unsigned		dlc:4;				///< number of data bytes in frame (max 8)
		unsigned		brs:1;				///< bitrate switch (unused here)
		unsigned		fdf:1;				///< FD format (extended DLC-coding and CRC, unused here)
		unsigned		:1;					///< a reserved bit
		unsigned		efc:1;				///< event FIFO control (if set, en event is posted to the event FIFO)
		unsigned		mm:8;				///< an arbitrary message marker that is written to the event FIFO for identification of a corresponding packet
		union {
			uint8_t			data[8];		///< the eight data bytes
			uint32_t		wdata[2];		///< the same data in 32 bit words
		};
	};
	uint32_t	tb[CAN_WORDS_PER_MSG];		///< all the data in four 32bit words
} cantxbuf;

typedef union {
	struct {
		unsigned		id:29;				///< the CAN id (either standard or extended) of the frame
		unsigned		rtr:1;				///< remote transmission request
		unsigned		xtd:1;				///< extended identifyer (i.e. the ID is a 29 bit extended ID)
		unsigned		esi:1;				///< error state indicator (unused here)
		unsigned		txts:16;			///< timestamp of transmission start time
		unsigned		dlc:4;				///< number of data bytes in frame (max 8)
		unsigned		brs:1;				///< bitrate switch (unused here)
		unsigned		edl:1;				///< extended data length
		unsigned		et:2;				///< event type (0b01: Tx-Event, 0b10: Tx in spite of cacellation, others: reserved)
		unsigned		mm:8;				///< the message marker from transmitted package
	};
	uint32_t	eb[2];						///< all the event data in two 32bit words
} canevtbuf;

typedef union {
	struct {
		unsigned		hash:16;			///< hash
		unsigned		resp:1;				///< respond bit
		unsigned		cmd:8;				///< command
		unsigned		prio:4;				///< prio
	};
	uint32_t	msgID;						///< all the data in a 32bit word
} MCAN_MSG_ID;

static volatile int can_modules;			///< the actual number of used can modules
static uint16_t mc_hash;

static struct CAN_Client {
	uint32_t	UID;
	uint32_t	dev_id;
	uint32_t	sw_no;
	uint32_t	hw_no;
	int			alive;
	uint16_t	adr;		// loco adress if any
	uint16_t	hash;
} can_clients[CAN_MAXUNIT];

static QueueHandle_t txqueue;
static QueueHandle_t rxqueue;
static TaskHandle_t rx_taskid;
static TaskHandle_t tx_taskid;

/*
 * The FDCAN1-Module is clocked by the PPL1_Q output @ 8MHz.
 * We will need a Bitrate of 250kBit/s. A time quanta (tq) will be 8MHz / 2 = 250ns.
 * There will be 1tq (not changeable) for synch, 11tq for DTSEG1 and 4tq for DTSEG2.
 * There will be a synchronisation jump width of 4tq. This is almost the default
 * setting of the FDCAN, except for the divider, which by default is set to 1 and
 * would give a CAN bitrate of 500kBit/s.
 *
 * A word to the nomenclature used here: 'standard ID' means 11 bit identifier
 * and 'extended ID' means 29 bit identifier.
 */
static void mcan_init(void)
{
	int adr = 0;	// the message RAM address in 32bit words

	FDCAN1->CCCR = FDCAN_CCCR_TXP | FDCAN_CCCR_CCE | FDCAN_CCCR_INIT;		// we allow a pause of 2 TQ between messages
	while (FDCAN1->CCCR != (FDCAN_CCCR_CCE | FDCAN_CCCR_INIT)) taskYIELD();
	FDCAN1->DBTP = (1 << FDCAN_DBTP_DBRP_Pos) | (10 << FDCAN_DBTP_DTSEG1_Pos) | (3 << FDCAN_DBTP_DTSEG2_Pos) | (3 << FDCAN_DBTP_DSJW_Pos);
	FDCAN1->NBTP = (3 << FDCAN_NBTP_NSJW_Pos) | (1 << FDCAN_NBTP_NBRP_Pos) | (10 << FDCAN_NBTP_NTSEG1_Pos) | (3 << FDCAN_NBTP_NTSEG2_Pos);
	FDCAN1->GFC = 0;		// accept all Frames to FIFO0
	FDCAN1->SIDFC = 0;		// we have no additional filters for standard ID frames
	FDCAN1->XIDFC = 0;		// we have no additional filters for extended ID frames

	// configuration of Rx-FIFO0: blocking mode, no watermark, 64 entries, address: 0x0000
	FDCAN1->RXF0C = (CAN_RXFIFO0_BUFFERS << FDCAN_RXF0C_F0S_Pos) | (adr << FDCAN_RXF0C_F0SA_Pos);
	adr += CAN_RXFIFO0_BUFFERS * CAN_WORDS_PER_MSG;

	// configuration of Tx-Path: Queue mode, 32 entries in queue, no dedicated TX buffers, address: directly behind RX FIFO0
	FDCAN1->TXBC = FDCAN_TXBC_TFQM | (CAN_TXQUEUE_BUFFERS << FDCAN_TXBC_TFQS_Pos) | (adr << FDCAN_TXBC_TBSA_Pos);
	adr += CAN_TXQUEUE_BUFFERS * CAN_WORDS_PER_MSG;

//	// configuration of Tx-Event FIFO: 32 entries, Watermark at 20 entries, address: directly behind TX QUEUE
//	FDCAN1->TXEFS = (20 << FDCAN_TXEFC_EFWM_Pos) | (CAN_TXEVENT_BUFFERS << FDCAN_TXEFC_EFS_Pos) | (adr << FDCAN_TXEFC_EFSA_Pos);
//	adr += CAN_TXEVENT_BUFFERS * CAN_WORDS_PER_TXEVENT;		// adr is currently not used further on ...

	FDCAN1->IR = 0x3FCFFFFF;							// clear all interrupt flags
	// switch all error handling to Line-1 interrupt (keeping normal and unsused interrupts in Line-0)
	FDCAN1->ILS = FDCAN_ILS_ELOE | FDCAN_ILS_EPE | FDCAN_ILS_EWE | FDCAN_ILS_BOE | FDCAN_ILS_WDIE |
				  FDCAN_ILS_PEAE | FDCAN_ILS_PEDE | FDCAN_ILS_ARAE;
	NVIC_SetPriority(FDCAN1_IT0_IRQn, 12);				// priority same as USART in XPressNet
	NVIC_SetPriority(FDCAN1_IT1_IRQn, 12);
	NVIC_ClearPendingIRQ(FDCAN1_IT0_IRQn);
	NVIC_ClearPendingIRQ(FDCAN1_IT1_IRQn);
	NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
	FDCAN1->ILE = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;	// enable both interrupt lines
	FDCAN1->IE = FDCAN_IE_RF0NE | FDCAN_IE_TCE;			// enable the receiver and transmission complete interrupts

	// clearing init bit also clears the CCE bit and starts CAN communication
	FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
	// building hash from serial number
	mc_hash = (uint16_t)hwinfo->serial ^ (uint16_t)(hwinfo->serial>>16);
	mc_hash &= ~0x80;
	mc_hash |= 0x300;
}

#if 0
static int _can_setModules (int oldcount, int count)
{
	struct bidibnode *root, *n;

	if (count > MAX_CANMODULES) count = MAX_CANMODULES;
	if (count < 0) count = 0;
	if (oldcount < 0) oldcount = 0;

	if (oldcount != count) {
		root = BDBnode_lookupNodeByUID(mcanHubUID, NULL);
		if (count == 0) {
			if (root) BDBnode_dropNode(root);
		} else {
			if (!root) {
				oldcount = 0;
				root = BDBvn_newBridge(BDBnode_getRoot(), BIDIB_HUB_MCAN);
			}
			while (oldcount < count) {
				oldcount++;
				BDBvn_newMCAN(root, BIDIB_MCAN_SNR_OFFSET + oldcount);
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

void can_setModules (int count)
{
	struct sysconf *cnf;

	cnf = cnf_getconfig();
	if (cnf->canModules != count) {
		cnf->canModules = _can_setModules(cnf->canModules, count);
		cnf_triggerStore();
	}
}
#else
void can_setModules (int count)
{
	struct sysconf *cnf;

	cnf = cnf_getconfig();
	if (cnf->canModules != count) {
		cnf->canModules = BDBvn_feedbackModules(cnf->canModules, count, MAX_CANMODULES, BIDIB_HUB_MCAN);
		cnf_triggerStore(__func__);
#ifdef CENTRAL_FEEDBACK
		event_fire(EVENT_FBPARAM, 0, NULL);
#else
		s88_triggerUpdate();
#endif
	}
}
#endif

static void mcan_controlEvent (int busadr, int reason)
{
	struct extDevice *dev;

	if ((dev = calloc (1, sizeof(*dev))) == NULL) return;
	dev->bus = BUS_MCAN;
	dev->id = busadr;
	dev->tp = DEV_CONTROL;
	dev->serial = can_clients[busadr].UID;
	snprintf (dev->swrev, sizeof(dev->swrev), "%ld.%ld", (can_clients[busadr].sw_no >> 8) & 0xFF, (can_clients[busadr].sw_no >> 0) & 0xFF);
	snprintf (dev->hwrev, sizeof(dev->hwrev), "%lx", can_clients[busadr].hw_no);
	event_fireEx(EVENT_CONTROLS, reason, dev, EVTFLAG_FREE_SRC, QUEUE_WAIT_TIME);
}

void mcan_reportControls (void)
{
	int i;

	for (i = 0; i < CAN_MAXUNIT; i++) {
		if (can_clients[i].alive > 0) mcan_controlEvent(i, 1);
	}
}

static int mcan_sendframe (uint32_t id, bool eid, int len, void *data)
{
	cantxbuf buf;
	int i;
	uint8_t *d;

	if (len < 0 || len > 8) return -1;

	// prepare buffer
	memset (&buf, 0, sizeof(buf));
	if (eid) buf.xtd = 1;
	else id <<= 18;				// a standard ID (11 bits) must be present in the upper ID bits
	buf.id = id;
	buf.dlc = len;
	buf.efc = 0;				// no event will be reported
	if (data && len > 0) {		// copy data to buffer
		d = (uint8_t *) data;
		for (i = 0; i < len; i++) buf.data[i] = d[i];
	}

	xQueueSendToBack(txqueue, &buf, 100);

	return 0;
}

static void mcan_dump (canrxbuf *fr)
{
    int i;
    int prio, cmd, resp, hash;

    if (fr->xtd) {
    	prio = (fr->id >> 25) & 0x0F;
    	cmd  = (fr->id >> 17) & 0xFF;
    	resp = (fr->id >> 16) & 0x01;
    	hash = (fr->id >> 0) & 0xFFFF;
    	printf ("%s(): prio %d cmd 0x%02x %s, DLC %d, hash 0x%04x\n", __func__, prio, cmd, (resp) ? "RESP" : "CMD", fr->dlc, hash);
    	printf ("%s(): EID 0x%08x ", __func__, fr->id);
    } else {
    	printf ("%s(): SID 0x%04x ", __func__, (fr->id >> 18) & CAN_SIDMASK);
    }
    for (i = 0; i < fr->dlc; i++) {
    	printf ("%02x ", fr->data[i]);
    }
    for (i = 0; i < fr->dlc; i++) {
    	printf ("%c", isprint(fr->data[i]) ? fr->data[i] : '.');
    }
    putchar('\n');
}

static bool mcan_eventhandler (eventT *e, void *priv)
{
	MCAN_MSG_ID msgid;
	uint8_t data[8];
	ldataT *l;
	uint16_t speed;

	(void) priv;

//	printf ("%s() event %d e->tix=%p taskid=%p\n", __func__, e->ev, e->tid, rx_taskid);
	if (e->tid == rx_taskid) return true;				// this is an event we triggered ourself, so don't report back!
	msgid.msgID = 0;	// initializing the ID to zero
	msgid.hash = mc_hash;

	switch (e->ev) {
		case EVENT_LOCO_SPEED:
			msgid.prio = 0;
			msgid.resp = 1;
			msgid.cmd = CAN_LS;
			data[0] = 0;
			data[1] = 0;
			l = (ldataT *) e->src;
			speed = l->speed & 0x7F;
//			printf ("%s(); speed: %d, fmt: %d\n", __func__, l->speed, l->loco->fmt);
			switch(l->loco->fmt) {
				case FMT_MM1_14:
				case FMT_MM2_14:
					speed *= 77;
					data[2] = 0x20;
					data[3] = (uint8_t)l->loco->adr;
					data[4] = speed >> 8;
					data[5] = (uint8_t)speed;
					break;
				case FMT_DCC_14:
					speed *= 77;
					data[2] = (uint8_t)(l->loco->adr >> 8) | 0xC0;
					data[3] = (uint8_t)l->loco->adr;
					data[4] = speed >> 8;
					data[5] = (uint8_t)speed;
					break;
				case FMT_MM2_27A:
				case FMT_MM2_27B:
					speed *= 38;
					data[2] = 0x20;
					data[3] = (uint8_t)l->loco->adr;
					data[4] = speed >> 8;
					data[5] = (uint8_t)speed;
					break;
				case FMT_DCC_28:
					speed *= 37;
					data[2] = (uint8_t)(l->loco->adr >> 8) | 0xC0;
					data[3] = (uint8_t)l->loco->adr;
					data[4] = speed >> 8;
					data[5] = (uint8_t)speed;
					break;
				case FMT_DCC_126:
				case FMT_M3_126:
					speed *= 8;
					data[2] = (uint8_t)(l->loco->adr >> 8) | 0x40;
					data[3] = (uint8_t)l->loco->adr;
					data[4] = speed >> 8;
					data[5] = (uint8_t)speed;
					break;
				default:
					break;
			}
			for (int i = 0; i < CAN_MAXUNIT; i++) {
				if (can_clients[i].alive > 0) {
					if(can_clients[i].adr == l->loco->adr) {
						msgid.hash = can_clients[i].hash;
						mcan_sendframe (msgid.msgID, true, 6, data);
						msgid.cmd = CAN_LD;
						data[4] = l->speed & 0x80 ? 1 : 2;
						mcan_sendframe (msgid.msgID, true, 5, data);
					}
				}
			}
			break;
		case EVENT_LOCO_FUNCTION:
			msgid.prio = 0;
			msgid.resp = 1;
			msgid.cmd = CAN_LF;
			data[0] = 0;
			data[1] = 0;
			l = (ldataT *) e->src;
			switch(l->loco->fmt) {
				case FMT_MM1_14:
				case FMT_MM2_14:
				case FMT_MM2_27A:
				case FMT_MM2_27B:
					data[2] = 0x20;
					data[3] = (uint8_t)l->loco->adr;
					break;
				case FMT_DCC_14:
				case FMT_DCC_28:
				case FMT_DCC_126:
					data[2] = (uint8_t)(l->loco->adr >> 8) | 0xC0;
					data[3] = (uint8_t)l->loco->adr;
					break;
				case FMT_M3_126:
					data[2] = (uint8_t)(l->loco->adr >> 8) | 0x40;
					data[3] = (uint8_t)l->loco->adr;
					break;
				default:
					break;
			}

			for (int i = 0; i < CAN_MAXUNIT; i++) {
				if (can_clients[i].alive > 0) {
					if(can_clients[i].adr == l->loco->adr) {
						msgid.hash = can_clients[i].hash;
						for(int i = 0; i < 32; i++) {
							data[4] = i;
							data[5] = (l->funcs[0] & (1<<i)) ? 1 : 0;
							mcan_sendframe (msgid.msgID, true, 6, data);
						}
						printf ("%s(); hash: %d\n", __func__, msgid.hash);
					}
				}
			}
			break;
		case EVENT_CONTROLS:
			msgid.cmd = CAN_SW;
			msgid.prio = 0;
			msgid.resp = 1;
			data[0] = 0x12;
			data[1] = 0x34;
			data[2] = 0x56;
			data[3] = 0x78;
			data[4] = 0;
			data[5] = 0x04;
			data[6] = 0;
			data[7] = 0x10;
			mcan_sendframe (msgid.msgID, true, 0, data);
			break;
		case EVENT_SYS_STATUS:
			msgid.cmd = CAN_SYS;
			msgid.prio = 0;
			msgid.resp = 1;
			data[0] = 0x00;		// to all devices
			data[1] = 0x00;
			data[2] = 0x00;
			data[3] = 0x00;
			switch (e->param) {
				case SYSEVENT_STOP:
					data[4] = 0x00;		// Subcommand STOP
					break;
				case SYSEVENT_HALT:
					data[4] = 0x02;		// Subcommand HALT
					break;
				case SYSEVENT_GO:
					data[4] = 0x01;		// Subcommand GO
					break;
				case SYSEVENT_SHORT:
					data[4] = 0x00;		// Subcommand STOP
					break;
				case SYSEVENT_TESTDRIVE:
					data[4] = 0x00;		// Subcommand STOP
					break;
				case SYSEVENT_RESET:
					data[4] = 0x00;		// Subcommand STOP
					break;
			}
			mcan_sendframe (msgid.msgID, true, 5, data);
			break;
		default:
			break;
	}
	return true;
}

static void updateDev(int dev, ldataT *l)
{
	MCAN_MSG_ID msgid;
	uint8_t data[8];
	uint16_t speed;

	msgid.msgID = 0;	// initializing the ID to zero
	msgid.hash = mc_hash;
	msgid.prio = 0;
	msgid.resp = 1;
	msgid.cmd = CAN_LS;
	data[0] = 0;
	data[1] = 0;
	speed = l->speed & 0x7F;
	printf ("%s(); speed: %d, fmt: %d\n", __func__, l->speed, l->loco->fmt);
	switch(l->loco->fmt) {
		case FMT_MM1_14:
		case FMT_MM2_14:
			speed *= 77;
			data[2] = 0x20;
			data[3] = (uint8_t)l->loco->adr;
			data[4] = speed >> 8;
			data[5] = (uint8_t)speed;
			break;
		case FMT_DCC_14:
			speed *= 77;
			data[2] = (uint8_t)(l->loco->adr >> 8) | 0xC0;
			data[3] = (uint8_t)l->loco->adr;
			data[4] = speed >> 8;
			data[5] = (uint8_t)speed;
			break;
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			speed *= 38;
			data[2] = 0x20;
			data[3] = (uint8_t)l->loco->adr;
			data[4] = speed >> 8;
			data[5] = (uint8_t)speed;
			break;
		case FMT_DCC_28:
			speed *= 37;
			data[2] = (uint8_t)(l->loco->adr >> 8) | 0xC0;
			data[3] = (uint8_t)l->loco->adr;
			data[4] = speed >> 8;
			data[5] = (uint8_t)speed;
			break;
		case FMT_DCC_126:
		case FMT_M3_126:
			speed *= 8;
			data[2] = (uint8_t)(l->loco->adr >> 8) | 0x40;
			data[3] = (uint8_t)l->loco->adr;
			data[4] = speed >> 8;
			data[5] = (uint8_t)speed;
			break;
		default:
			break;
	}
	msgid.hash = can_clients[dev].hash;
	mcan_sendframe (msgid.msgID, true, 6, data);
	msgid.cmd = CAN_LD;
	data[4] = l->speed & 0x80 ? 1 : 2;
	mcan_sendframe (msgid.msgID, true, 5, data);
	msgid.prio = 0;
	msgid.resp = 1;
	msgid.cmd = CAN_LF;
	data[0] = 0;
	data[1] = 0;
	switch(l->loco->fmt) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			data[2] = 0x20;
			data[3] = (uint8_t)l->loco->adr;
			break;
		case FMT_DCC_14:
		case FMT_DCC_28:
		case FMT_DCC_126:
			data[2] = (uint8_t)(l->loco->adr >> 8) | 0xC0;
			data[3] = (uint8_t)l->loco->adr;
			break;
		case FMT_M3_126:
			data[2] = (uint8_t)(l->loco->adr >> 8) | 0x40;
			data[3] = (uint8_t)l->loco->adr;
			break;
		default:
			break;
	}

	msgid.hash = can_clients[dev].hash;
	for(int i = 0; i < 32; i++) {
		data[4] = i;
		data[5] = (l->funcs[0] & (1<<i)) ? 1 : 0;
		mcan_sendframe (msgid.msgID, true, 6, data);
	}
}

static void mcan_receive (canrxbuf *rx)
{
#ifndef CENTRAL_FEEDBACK
	volatile uint16_t *input;
	uint16_t tmp;
#endif
	int cmd = 0xFF;
	MCAN_MSG_ID msgid;
	uint8_t data[8], ui8, dir, fmt = 0;
	uint16_t ui16;
	uint16_t adr;
	uint32_t ui32;
	ldataT *l;

	mcan_dump(rx);

	cmd = (rx->id >> 17) & 0xFF;
	memset (data, 0, sizeof(data));
	msgid.msgID = 0;
	msgid.cmd = cmd;
	msgid.hash = mc_hash;
	msgid.prio = (rx->id >> 25) & 0x0F;
	msgid.resp = 1;
	switch(cmd) {
		case CAN_SYS:
			msgid.msgID = rx->rb[0] & 0x1FFFFFFF;
			msgid.resp = 1;
			for(ui8 = 0; ui8 < 8; ui8++) {
				data[ui8] = rx->data[ui8];
			}
			switch(rx->data[4]) {	// Sub command
				case 0:	// STOP
					sig_setMode(TM_STOP);
					mcan_sendframe (msgid.msgID, true, 5, data);
					break;

				case 1:	// GO
					sig_setMode(TM_GO);
					mcan_sendframe (msgid.msgID, true, 5, data);
					break;

				case 2:	// HALT
					sig_setMode(TM_HALT);
					mcan_sendframe (msgid.msgID, true, 5, data);
					break;

				case 5:	// change loco protocol
					fprintf(stderr,"%s() new protocol: %d -> ToDo: implementieren\n", __func__, rx->data[5]);
/* von EasyNet übernommen -> anpassen					if ((l = loco_call (adr, true)) != NULL) {
						db_setLocoFmt(adr, bus_en2locofmt(blk->data[2]));
						p = bus_set14bit(data, adr);
						*p++ = bus_locofmt2en(l->loco->fmt);
						en_sendBlock(unit, CMD_LOCOFORMAT, data);
					}*/
					mcan_sendframe (msgid.msgID, true, 6, data);
					break;

				case 6:	// change accessory switch time
					ui16 = (rx->data[5]<<8 | rx->data[6])*10;
					printf ("%s(); Accessory switch time: %d\n", __func__, ui16);
					trnt_setMinTime(ui16);
					trnt_setMaxTime(ui16);
					mcan_sendframe (msgid.msgID, true, 7, data);
					break;

				case 0x80:	// reset
					printf ("%s(); RESET: %d\n", __func__, rx->data[5]);
					mcan_sendframe (msgid.msgID, true, 6, data);
					break;

				default:
					printf ("%s(); sub cmd: 0x%x\n", __func__, rx->data[4]);
					break;
			}
			break;

		case CAN_SW:	// ask SW Version / Ping
			data[0] = (hwinfo->serial >> 24) & 0xFF;
			data[1] = (hwinfo->serial >> 16) & 0xFF;
			data[2] = (hwinfo->serial >> 8) & 0xFF;
			data[3] = hwinfo->serial & 0xFF;
			data[4] = 0x01;
			data[5] = 0x0;
			data[6] = 0x00;
			data[7] = 0x10;
			if(!rx->dlc) {
				msgid.resp = 0;
				msgid.cmd = CAN_SW;
				msgid.hash = mc_hash;
				mcan_sendframe (msgid.msgID, true, 8, data);
				break;
			}
			ui32 = (rx->data[0]<<24) + (rx->data[1]<<16) + (rx->data[2]<<8) + rx->data[3];
			for(ui8 = 0; ui8 < CAN_MAXUNIT; ui8++) {
				if(can_clients[ui8].UID == ui32) break;
			}
			if(ui8 == CAN_MAXUNIT) {	// UID not known -> new device
				for(ui8 = 0; ui8 < CAN_MAXUNIT; ui8++) {
					if(!can_clients[ui8].alive) {
						can_clients[ui8].UID = ui32;
						can_clients[ui8].dev_id = (rx->data[6]<<8) + rx->data[7];
						can_clients[ui8].hw_no = (rx->data[6]<<8) + rx->data[7];
						can_clients[ui8].sw_no = (rx->data[4]<<8) + rx->data[5];
						if(can_clients[ui8].UID && can_clients[ui8].dev_id && can_clients[ui8].sw_no) {
							can_clients[ui8].alive = ALIVE_VALUE;
							can_clients[ui8].hash = rx->rb[0] & 0xFFFF;
							printf("%s(); New Device -> SW version UID: 0x%lx; SW: 0x%lx, device: 0x%lx, hash: 0x%x\n", __func__, can_clients[ui8].UID, can_clients[ui8].sw_no, can_clients[ui8].dev_id, can_clients[ui8].hash);
							mcan_controlEvent(ui8, 1);
						}
						break;
					} else {
						if(can_clients[ui8].hash == (rx->rb[0] & 0xFFFF)) {
							can_clients[ui8].alive = ALIVE_VALUE;
							break;
						}
					}
				}
			} else {
				can_clients[ui8].alive = ALIVE_VALUE;
			}
			mcan_sendframe (msgid.msgID, true, 8, data);
			break;

		case CAN_BL:	// Bootloader
			printf ("%s(); BL: %x; DLC: %d\n", __func__, cmd, rx->dlc);
			msgid.resp = 0;
			msgid.cmd = CAN_SW;
			data[0] = 0x12;
			data[1] = 0x34;
			data[2] = 0x56;
			data[3] = 0x78;
			data[4] = 0;
			data[5] = 0x04;
			data[6] = 0;
			data[7] = 0x10;
			mcan_sendframe (msgid.msgID, true, 0, data);
			break;

		case CAN_LD:	// loco direction
			adr = (rx->data[2]<<8 | rx->data[3]) & ~0xC000;
			if ((l = loco_call (adr, true)) == NULL) return;		// cannot grant access to loco - something is wrong
			dir = l->speed & 0x80;
			msgid.msgID = rx->rb[0] & 0x1FFFFFFF;
			for (int i = 0; i < CAN_MAXUNIT; i++) {
				if (can_clients[i].alive > 0) {
					printf ("%s(); hash %d\n", __func__, can_clients[i].hash);
					if(can_clients[i].hash == (msgid.msgID & 0xFFFF)) {
						if(can_clients[i].adr != adr) {
							can_clients[i].adr = adr;
							updateDev(i, l);
							return;
						}
					}
				}
			}
			msgid.resp = 1;
			if(rx->dlc == 5) {
				switch(rx->data[4]) {
					case 0:	// same direction
						break;

					case 1:	// dir forward
						if(!dir) {
							rq_setSpeed (adr, 0x80);
						}
						break;

					case 2:	// dir reverse
						if(dir) {
							rq_setSpeed (adr, 0);
						}
						break;
				}
				for (ui8 = 0; ui8 < 8; ui8++) {
					data[ui8] = rx->data[ui8];
				}
			} else {
				ui16 = adr & 0x3FFF;
				data[0] = 0;
				data[1] = 0;
				data[2] = (ui16 >> 8) & fmt;
				data[3] = (uint8_t)ui16;
				data[4] = dir ? 2 : 1;
			}
			mcan_sendframe (msgid.msgID, true, 5, data);
			// speed 0
			msgid.msgID = rx->rb[0] & 0x1FFFFFFF;
			msgid.resp = 1;
			msgid.cmd = CAN_LS;
			switch (l->loco->fmt) {
				case FMT_MM1_14:
				case FMT_MM2_14:
					fmt = 0;
					break;
				case FMT_DCC_14:
					fmt = 0xC0;
					break;
				case FMT_MM2_27A:
				case FMT_MM2_27B:
					fmt = 0;
					break;
				case FMT_DCC_28:
					fmt = 0xC0;
					break;
				case FMT_DCC_126:
				case FMT_DCC_SDF:
					fmt = 0xC0;
					break;
				case FMT_M3_126:
					fmt = 0x40;
					break;
				default:
					fmt = 0xC0;
					break;
			}
			ui16 = adr & 0x3FFF;
			data[0] = 0;
			data[1] = 0;
			data[2] = (ui16 >> 8) | fmt;
			data[3] = (uint8_t)ui16;
			data[4] = 0;
			data[5] = 0;
			mcan_sendframe (msgid.msgID, true, 6, data);
			break;

		case CAN_LS:	// loco speed
			adr = (rx->data[2]<<8 | rx->data[3]) & ~0xC000;
			if ((l = loco_call (adr, true)) == NULL) return;		// cannot grant access to loco - something is wrong
			switch (l->loco->fmt) {
				case FMT_MM1_14:
				case FMT_MM2_14:
					ui8 = 14;
					fmt = 0;
					break;
				case FMT_DCC_14:
					fmt = 0xC0;
					ui8 = 14;
					break;
				case FMT_MM2_27A:
				case FMT_MM2_27B:
					fmt = 0;
					ui8 = 27;
					break;
				case FMT_DCC_28:
					fmt = 0xC0;
					ui8 = 28;
					break;
				case FMT_DCC_126:
				case FMT_DCC_SDF:
					fmt = 0xC0;
					ui8 = 126;
					break;
				case FMT_M3_126:
					fmt = 0x40;
					ui8 = 126;
					break;
				default:
					fmt = 0xC0;
					ui8 = 126;
					break;
			}
			msgid.msgID = rx->rb[0] & 0x1FFFFFFF;
			for (int i = 0; i < CAN_MAXUNIT; i++) {
				if (can_clients[i].alive > 0) {
					printf ("%s(); hash %d\n", __func__, can_clients[i].hash);
					if(can_clients[i].hash == (msgid.msgID & 0xFFFF)) {
						if(can_clients[i].adr != adr) {
							can_clients[i].adr = adr;
							updateDev(i, l);
							return;
						}
					}
				}
			}
			msgid.resp = 1;
			if(rx->dlc == 6) {	//set speed
				ui16 = rx->data[4]<<8 | rx->data[5];
				ui8 = ui16*ui8/1000;
				dir = l->speed & 0x80;
				rq_setSpeed (adr, ui8 | dir);
				for(ui8 = 0; ui8 < 8; ui8++) {
					data[ui8] = rx->data[ui8];
				}
			} else {
				ui16 = adr & 0x3FFF;
				data[0] = 0;
				data[1] = 0;
				data[2] = (ui16 >> 8) | fmt;
				data[3] = (uint8_t)ui16;
				ui16 = (l->speed & 0x7F) * 1000 / ui8;
				data[4] = ui16 >> 8;
				data[5] = (uint8_t)ui16;
			}
			mcan_sendframe (msgid.msgID, true, 6, data);
			break;

		case CAN_LF:	// loco function 0..31
			adr = (rx->data[2]<<8 | rx->data[3]) & ~0xC000;
			if ((l = loco_call (adr, true)) == NULL) return;		// cannot grant access to loco - something is wrong
			msgid.msgID = rx->rb[0] & 0x1FFFFFFF;
			for (int i = 0; i < CAN_MAXUNIT; i++) {
				if (can_clients[i].alive > 0) {
					printf ("%s(); hash %d\n", __func__, can_clients[i].hash);
					if(can_clients[i].hash == (msgid.msgID & 0xFFFF)) {
						if(can_clients[i].adr != adr) {
							can_clients[i].adr = adr;
							updateDev(i, l);
							return;
						}
					}
				}
			}
			msgid.resp = 1;
			if(rx->dlc == 6) {	//set function
				loco_setFunc (adr, rx->data[4], rx->data[5] ? 1 : 0);
				for(ui8 = 0; ui8 < 8; ui8++) {
					data[ui8] = rx->data[ui8];
				}
			} else {
				switch (l->loco->fmt) {
					case FMT_MM1_14:
					case FMT_MM2_14:
						fmt = 0;
						break;
					case FMT_DCC_14:
						fmt = 0xC0;
						break;
					case FMT_MM2_27A:
					case FMT_MM2_27B:
						fmt = 0;
						break;
					case FMT_DCC_28:
						fmt = 0xC0;
						break;
					case FMT_DCC_126:
					case FMT_DCC_SDF:
						fmt = 0xC0;
						break;
					case FMT_M3_126:
						fmt = 0x40;
						break;
					default:
						fmt = 0xC0;
						break;
				}
				ui16 = adr & 0x3FFF;
				data[0] = 0;
				data[1] = 0;
				data[2] = (ui16 >> 8) | fmt;
				data[3] = (uint8_t)ui16;
				data[4] = rx->data[4];
				data[5] = (l->funcs[0] & 1<<rx->data[5]) ? 1 : 0;
			}
			mcan_sendframe (msgid.msgID, true, 6, data);
			break;

		case CAN_AC:	// switch an accessory
			adr = (rx->data[2]<<8 | rx->data[3]) & 0x01FF;
			trnt_switch (adr + 1, rx->data[4], rx->data[5]);
			break;

		case CAN_S88:	// s88 event
#ifdef CENTRAL_FEEDBACK
			ui16 = (rx->data[2] << 8) + rx->data[3] - 1;	// the 1-based feedback address
			fb_BitInput(ui16 + FB_MCAN_OFFSET, !!rx->data[5]);
#else
			ui16 = ((rx->data[2] << 8) + rx->data[3] - 1) / 16;		// module no
			tmp =  ((rx->data[2] << 8) + rx->data[3] - 1) % 16;		// input no
			input = s88_getInputs();
			if(rx->data[5]) {
				input[ui16] |= 0x8000 >> tmp;
			} else {
				input[ui16] &= ~(0x8000 >> tmp);
			}
			s88_triggerUpdate();
#endif
			break;

		default:
			printf ("%s(); cmd: %x; DLC: %d\n", __func__, cmd, rx->dlc);
			break;
	}
}

static void mcan_txhandler (void *pvParameter)
{
    cantxbuf canbuf;
	uint32_t *p;
	int idx;

	(void) pvParameter;

	printf ("%s() started\n", __func__);
	tx_taskid = xTaskGetCurrentTaskHandle();
	for (;;) {
		if (xQueueReceive(txqueue, &canbuf, portMAX_DELAY)) {
			idx = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;
			p = (uint32_t *) (SRAMCAN_BASE + (FDCAN1->TXBC & FDCAN_TXBC_TBSA_Msk) + (sizeof(cantxbuf) * idx));
			// copy buffer to MESSAGE RAM using word access only
			*p++ = canbuf.tb[0];
			*p++ = canbuf.tb[1];
			*p++ = canbuf.tb[2];
			*p++ = canbuf.tb[3];
			// now set the interrupt enable flag of the buffer and trigger transmission
			FDCAN1->TXBTIE |= 1 << idx;
			FDCAN1->TXBAR = 1 << idx;
		}
	}
}

static void aliveTimer( TimerHandle_t xTimer )
{
	MCAN_MSG_ID msgid;
	uint8_t data[8];
	(void) xTimer;

	for (uint8_t ui8 = 0; ui8 < CAN_MAXUNIT; ui8++) {
		if (can_clients[ui8].alive > 0) {
			if(can_clients[ui8].alive == 3) {
				msgid.resp = 0;
				msgid.cmd = CAN_SW;
				msgid.hash = mc_hash;
				msgid.prio = 0;
				data[0] = (hwinfo->serial >> 24) & 0xFF;
				data[1] = (hwinfo->serial >> 16) & 0xFF;
				data[2] = (hwinfo->serial >> 8) & 0xFF;
				data[3] = hwinfo->serial & 0xFF;
				data[4] = 0x01;
				data[5] = 0x0;
				data[6] = 0xFF;
				data[7] = 0xFF;
				mcan_sendframe (msgid.msgID, true, 8, data);
			}
			if (--can_clients[ui8].alive == 0) {
				can_clients[ui8].UID = 0;
				mcan_controlEvent(ui8, 0);
			}
		}
	}
}

void vMCanHandler (void *pvParameter)
{
	canrxbuf rxbuf;

	(void) pvParameter;

	txqueue = xQueueCreate(16, sizeof(cantxbuf));
	rxqueue = xQueueCreate(16, sizeof(canrxbuf));

	if (!txqueue || !rxqueue) {
		fprintf (stderr, "%s(): cannot create RX/TX queues\n", __func__);
		if (txqueue) vQueueDelete(txqueue);
		if (rxqueue) vQueueDelete(rxqueue);
		txqueue = rxqueue = NULL;	// just in case someone will restart this thread later
		vTaskDelete(NULL);
	}

	printf ("%s() started\n", __func__);
	mcan_init();
	can_modules = cnf_getconfig()->canModules;
	rx_taskid = xTaskGetCurrentTaskHandle();
	xTaskCreate(mcan_txhandler, "MCAN-TX", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	event_register(EVENT_LOCO_SPEED, mcan_eventhandler, NULL, 0);
	event_register(EVENT_LOCO_FUNCTION, mcan_eventhandler, NULL, 0);
	event_register(EVENT_CONTROLS, mcan_eventhandler, NULL, 0);
	event_register(EVENT_SYS_STATUS, mcan_eventhandler, NULL, 0);
	// start alive timer
	xTimerStart(xTimerCreate ( "aliveTimer", pdMS_TO_TICKS( 2000 ), pdTRUE, NULL, aliveTimer ), 0);
	for (;;) {
		if (xQueueReceive(rxqueue, &rxbuf, portMAX_DELAY)) {
			mcan_receive(&rxbuf);
		}
	}
}

/**
 * Line-0 interrupt handler
 */
void FDCAN1_IT0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = 0;
    uint32_t canbuf[CAN_WORDS_PER_MSG];		// we don't interpret the contents of the queue buffers - so treat them as array of words
	uint32_t *p;
	int idx;

	if ((FDCAN1->IE & FDCAN_IE_RF0NE) && (FDCAN1->IR & FDCAN_IR_RF0N)) {	// a message was revceived in FIFO0
		while (((FDCAN1->RXF0S & FDCAN_RXF0S_F0FL_Msk) >> FDCAN_RXF0S_F0FL_Pos) > 0) {
			idx = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
			p = (uint32_t *) (SRAMCAN_BASE + (FDCAN1->RXF0C & FDCAN_RXF0C_F0SA_Msk) + (sizeof(canrxbuf) * idx));
			// copy buffer from MESSAGE RAM using word access only
			canbuf[0] = *p++;
			canbuf[1] = *p++;
			canbuf[2] = *p++;
			canbuf[3] = *p++;
			if (xQueueSendToBackFromISR(rxqueue, &canbuf, &xHigherPriorityTaskWoken) != pdTRUE) break;
			FDCAN1->RXF0A = idx;
		}
		FDCAN1->IR = FDCAN_IR_RF0N;		// clear interrupt flag
	}

	if (FDCAN1->IR & (FDCAN_IR_TC)) {		// transmission complete is triggered
		FDCAN1->TXBTIE &= ~(FDCAN1->TXBTO);	// we will reset all interrupt request bits for packets that are done
		FDCAN1->IR = FDCAN_IR_TC;			// clear interrupt flags
	}

    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}

/**
 * Line-1 interrupt handler
 */
void FDCAN1_IT1_IRQHandler(void)
{
	// currently, we simply clear these interrupts (maybe this will be a TODO)
	FDCAN1->IR = FDCAN_IR_ELO | FDCAN_IR_EP | FDCAN_IR_EW | FDCAN_IR_BO |
				 FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA;
}
