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
#include <stdarg.h>
#include "rb2.h"
#include "xpressnet.h"
#include "decoder.h"
#include "events.h"

/*
 * XPressNet uses USART1 in half duplex RS485 mode.
 * Data format is 62,5kBaud, 9n1
 */

#define WAIT_BITS			25			///< number of bits for the receiver timeout (see USART1->RTOR): 400µs
#define MAX_BLKLEN			32			///< size (in characters) of the RX/TX block buffers
/*
 * bits for the flags member in struct xpn_node
 */
#define NODEFLG_ACTIVE		0x0001		///< this node is in use (if not set, this node address is free
#define NODEFLG_EVENT		0x0002		///< an event (speed or function change) was received for the loco of this node
#define NODEFLG_INFORM		0x0004		///< inform the node, that it's loco is now under foreign control
#define NODEFLG_LB			0x0008		///< this loco adress is in use by another device

#define RXERR_FRAMING		0x0001		///< framing error on reception
#define RXERR_OVERRUN		0x0002		///< overrun error on reception (because of the FIFO probably never seen!)
#define RXERR_NOISE			0x0004		///< noise detected

struct blockbuf {
	uint8_t			buf[MAX_BLKLEN];	///< the block buffer memory itself (can be 8 bit, because the callbyte is put to transmitter directly)
	volatile int	len;				///< the current len (we always "rewind" to the beginning at a block reception / transmission)
	union {
		volatile int	idx;			///< the current index (mostly used by the TX handling in interrupt)
		volatile int	flags;			///< for RX the IRQ handler can set error flags like framing or overrun errors
	};
};
static struct blockbuf txbuf;			///< the transmitter queue
static struct blockbuf rxbuf;			///< the receiver queue

static int nodeidx;						///< the current node to be polled / with which we have an active communication
static uint8_t pollcycle;				///< a roll-over count for complete cycles - every 256 cycles a poll for new devices is started
static TaskHandle_t xpn_task;			///< the task to be notified after communication completion

// Definitions for the CALL-Byte
#define XPN_REQUEST_ACK		0x00
#define XPN_FB_BROADCAST	0x20		// reserved from v3.0 for individual answers (it is already used as a broadcast)
#define XPN_NORMAL_INQUIRY	0x40
#define XPN_ANSWER			0x60		// in v3.6 (v3.0???): BROADCAST

static struct pomCV {
	uint16_t	CV;						///< No of CV
	uint8_t		val;					///< value of CV
} lastPOM_CV;
uint8_t PT_active;

static struct xpn_node {
	uint8_t		adr;					///< the index (i.e. XN bus address) of this node. This value helps functions, that only get a pointer to this structure.
	uint16_t	loco;					///< which loco is under control by that node
	int			flags;					///< some flags for status
	uint16_t	alive;
} nodes[MAX_NODES];

static void usart1_init (void)
{
	uint32_t cr1, cr3;

	USART1->CR1 = 0;		// disable USART1
	USART1->CR2 = USART_CR2_RTOEN;							// use 1 stop bit and enable the receiver timeout function
	cr1 = USART_CR1_FIFOEN;									// enable FIFO mode
	cr1 |= USART_CR1_M0 | USART_CR1_TE | USART_CR1_RE;		// set 9 bits of data, enable transmitter and receiver
	// driver (de-)assertion timings
	cr1 |= 8 << USART_CR1_DEAT_Pos;							// 8 / 16 bit times (i.e. half a bit time)
	cr1 |= 0 << USART_CR1_DEDT_Pos;							// 0 / 16 bit times (i.e. switch off directly after the stopbit)
	USART1->CR1 = cr1;

	cr3 = (0b010 << USART_CR3_RXFTCFG_Pos);					// set RX-FIFO threshold interrupt at half full (but do not enable it for now)
	cr3 |= USART_CR3_DEM | USART_CR3_HDSEL;					// set driver enable mode with driver enable active high, half duplex mode
	USART1->CR3 = cr3;

	USART1->PRESC = 0b0111;	// prescaler = 16 -> 100MHz / 16 = 6,25MHz kernel clock
	USART1->BRR = 100;		// 6,25MHz / 100 -> 62,5kbit/s
	USART1->RTOR = WAIT_BITS;

	NVIC_SetPriority(USART1_IRQn, 12);
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	USART1->ICR = 0xFFFFFFFF;					// clear all interrupt flags

	SET_BIT (USART1->CR1, USART_CR1_UE);		// enable the UART
	SET_BIT (USART1->CR3, USART_CR3_RXFTIE);	// enable RX-FIFO threshold interrupt
}

static void xn_controlEvent (struct xpn_node *node, int reason)
{
	struct extDevice *dev;

	if ((dev = calloc (1, sizeof(*dev))) == NULL) return;
	dev->bus = BUS_XPRESSNET;
	dev->id = node->adr;
	dev->tp = DEV_CONTROL;
	event_fireEx(EVENT_CONTROLS, reason, dev, EVTFLAG_FREE_SRC, QUEUE_WAIT_TIME);
}

void xn_reportControls (void)
{
	int i;

	for (i = 0; i < MAX_NODES; i++) {
		if (nodes[i].alive > 0) xn_controlEvent(&nodes[i], 1);
	}
}

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

struct modeltime *theTime;
static uint8_t bTimeUpdate;

static uint16_t xpn_parity (uint8_t callbyte)
{
	uint16_t c;
	c = callbyte & 0x7F;
	c |= parity[c & 0x0f] ^ parity[c >> 4];
	return c | 0x100;		// add the nineth bit for the callbyte
}

static void xpn_bufferMessage (uint8_t cmd, uint8_t *data)
{
	uint8_t i, xor, len, *p;

	xor = cmd;
	len = cmd & 0x0F;
	p = txbuf.buf;
	*p++ = cmd;
	if (len > 0 && data) {
		for (i = 0; i < len; i++) {
		  xor ^= *data;
		  *p++ = *data++;
		}
	}
	*p++ = xor;
	txbuf.len = len + 2;		// account for CMD and XOR
}

/*
 * xpn_sendmessage() send out immedeately this message
 */
static void xpn_sendmessage (uint8_t callbyte, uint8_t cmd, uint8_t *data)
{
	uint16_t c;

	c = xpn_parity(callbyte);
	txbuf.len = txbuf.idx = 0;
	rxbuf.len = 0;
	rxbuf.flags = 0;
	switch (callbyte & 0x60) {
		case XPN_REQUEST_ACK:
			break;
		case XPN_FB_BROADCAST:
			break;
		case XPN_NORMAL_INQUIRY:
			break;
		case XPN_ANSWER:
			xpn_bufferMessage(cmd, data);
			break;
	}

	// now put 'c' to USART1->TDR and enable transmitter interrupt
	USART1->TDR = c;
	USART1->ICR = USART_ICR_RTOCF | USART_ICR_TCCF | USART_ICR_TXFECF;
	SET_BIT (USART1->CR1, USART_CR1_TXEIE_TXFNFIE);
}

/**
 * Send a message with variable number of bytes (arguments passed in as integers).
 *
 * @param callbyte	the first byte to send out, will get a parity bit and the nineth bit set
 * @param cmd		the second byte specifying the command for the node
 * @param ...		"any" number of intergers representing the rest of the message
 */
static void _xpn_sendmessage (uint8_t callbyte, uint8_t cmd, ...)
{
	uint8_t data[15], *p, cnt;
	va_list ap;

	cnt = cmd & 0x0F;
	p = data;
	va_start (ap, cmd);
	while (cnt) {
		*p++ = (uint8_t) va_arg(ap, int);
		cnt--;
	}
	va_end (ap);
	xpn_sendmessage (callbyte, cmd, data);
}

static int xpn_unknown (struct xpn_node *node, struct blockbuf *msg)
{
    int i;

    _xpn_sendmessage (XPN_ANSWER | node->adr, 0x61, 0x82);

    printf("%s(%d)", __func__, node->adr);
	for (i = 0; i <= (msg->buf[0] & 0x0F); i++) {
		printf(" %02x", msg->buf[i]);
	}
	dbg_putc ('\n');
	return 1;
}

static int xpn_requests(struct xpn_node *node, struct blockbuf *msg)
{
	uint8_t x;

	if (msg->buf[0] == 0x20) {	// this is an acknowledgement response
//		xcp->nextstate = 0;	// clear request to cancel this unit
		return 0;
	}

//	printf("msg: %d\n", msg->buf[1]);

	switch (msg->buf[1]) {
		case CMD_PTRESULT:
			printf ("%s(): PT read: CV: %d, Value: %d\n", __func__, lastPOM_CV.CV, lastPOM_CV.val);
			if(lastPOM_CV.CV < 256)
				_xpn_sendmessage(XPN_ANSWER | node->adr, 0x63, 0x14, (uint8_t)lastPOM_CV.CV, lastPOM_CV.val);
			else if(lastPOM_CV.CV < 512)
				_xpn_sendmessage(XPN_ANSWER | node->adr, 0x63, 0x15, (uint8_t)lastPOM_CV.CV, lastPOM_CV.val);
			else if(lastPOM_CV.CV < 768)
				_xpn_sendmessage(XPN_ANSWER | node->adr, 0x63, 0x16, (uint8_t)lastPOM_CV.CV, lastPOM_CV.val);
			else if(lastPOM_CV.CV < 1024)
				_xpn_sendmessage(XPN_ANSWER | node->adr, 0x63, 0x17, (uint8_t)lastPOM_CV.CV, lastPOM_CV.val);
			else	// here is CV = 1024
				_xpn_sendmessage(XPN_ANSWER | node->adr, 0x63, 0x14, 0, lastPOM_CV.val);
			return 1;

		case CMD_PTREAD:
		case CMD_PTREAD+1:
		case CMD_PTREAD+2:
		case CMD_PTREAD+3:
			PT_active = true;
			lastPOM_CV.CV = (msg->buf[1] & 3)<<8 | msg->buf[2];
			if(!lastPOM_CV.CV) lastPOM_CV.CV = 1024;
			lastPOM_CV.val = dccpt_cvReadByte (lastPOM_CV.CV-1);
			return 0;

		case 0x21:	// report software version
			//_xpn_sendmessage(XPN_ANSWER | node->adr, 0x63, 0x21, 0x36, 0x00);	// V3.6 / LZ100
			_xpn_sendmessage (XPN_ANSWER | node->adr, 0x63, 0x21, 0x39, 0x01);	// V3.9 / LH200
			//_xpn_sendmessage (XPN_ANSWER | adr, 0x62, 0x21, 0x23);		// V2.3
			return 1;
		case 0x23:	// report status
			fprintf(stderr,"%s(): report extended SW-version: DUMMY DATA!\n", __func__);
			_xpn_sendmessage (XPN_ANSWER | node->adr, 0x67, 0x23, 0, 0x01, 0x01, 0, 0x01, 0x01);	// dummy!!!
			return 1;
		case 0x24:	// report status
			printf("report STATUS\n");
			printf("status: %d \n", rt.tm);
			x = 0;
			if (rt.tm == TM_HALT) x |= 0x03;
			if (rt.tm == TM_STOP) x |= 0x02;
			if (rt.tm == TM_DCCPROG) x |= 0x08;
			if (rt.tm == TM_SHORT) x |= 0x02;
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0x62, 0x22, (int) x);
			return 1;
		case 0x27:	//POM read: call for result
			printf ("%s(): POM read: CV: %d, lastPOM_CV: %d\n", __func__, lastPOM_CV.CV, lastPOM_CV.val);
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0x64, 0x24, lastPOM_CV.CV>>8, (uint8_t)lastPOM_CV.CV, lastPOM_CV.val);
			return 1;
		case 0x2A:
			_xpn_sendmessage (XPN_ANSWER | node->adr, 0x64, 0x25, (theTime->mday<<5)|(theTime->hour), theTime->min, theTime->speedup);
			return 1;
		case 0x2B:
//			printf ("%s(): Modeltime: day: %d, hour: %d, Minute: %d\n", __func__, theTime->mday, theTime-> hour, theTime-> min);
			mt_setdatetime(theTime->year, theTime->mon, msg->buf[2] >> 5, msg->buf[2] & 0x1F, msg->buf[3]);
			mt_speedup(msg->buf[4]);
			return 0;
		case CMD_STOP:	// emergency stop
			sig_setMode(TM_STOP);
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0x62, 0x22, 0x02);
			return 1;
		case CMD_START:	// resume normal operation
			sig_setMode(TM_GO);
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0x62, 0x22, 0);
			return 1;
		default:
			xpn_unknown(node, msg);
			return 1;
	}

	return 0;
}

static int xpn_tostat(struct xpn_node *node, struct blockbuf *msg)
{
	turnoutT *t;
	int to;
	int stat = 0;

	to = msg->buf[1] << 2;
	if (msg->buf[2] & 0x01) to += 2;

	if (to & 0x02) stat |= 0x10;
	if ((t = db_lookupTurnout (to + 1)) != NULL) {
		if (t->on) stat |= 0x80;	// turnout still active
		stat |= ((t->dir) ? 0b01 : 0b10) << 0;
	}
	if ((t = db_lookupTurnout (to + 2)) != NULL) {
		if (t->on) stat |= 0x80;	// turnout still active
		stat |= ((t->dir) ? 0b01 : 0b10) << 2;
	}

	_xpn_sendmessage(XPN_ANSWER + node->adr, 0x42, to >> 2, stat);
	return 1;
}

static int xpn_toaction(struct xpn_node *node, struct blockbuf *msg)
{
	int to;

	(void) node;

	to = ((msg->buf[1] << 2) | ((msg->buf[2] >> 1) & 0x03)) + 1;

	trnt_switch(to, !(msg->buf[2] & 0x01), !!(msg->buf[2] & 0x08));
	return 0;
}

static void xpn_lostControl (struct xpn_node *node)
{
	int loco;

	loco = node->loco;
	if (loco >= 100) loco |= 0xC000;
	_xpn_sendmessage(XPN_ANSWER + node->adr, 0xE3, 0x40, loco >> 8, loco & 0xFF);
}

static void xpn_locoinformation (struct xpn_node *node)
{
	uint8_t speed, id, funcA, funcB, ui8;
	ldataT *l;
	struct consist *c;
	int loco = 0;

	if (node->loco == 0) {
		speed = id = funcA = funcB = 0;
	} else {

		if ((l = loco_call (node->loco, true)) == NULL) {		// cannot grant access to loco - something is wrong (?)
			speed = id = funcA = funcB = 0;
		} else {
			speed = l->speed & 0x7F;

			switch (l->loco->fmt) {		// speed mode (14, 27, 28 or 128 speeds)
				case FMT_MM1_14:
				case FMT_MM2_14:
				case FMT_DCC_14:
				default:
					if (speed) speed++;
					id = 0x00;
					break;
				case FMT_MM2_27A:
				case FMT_MM2_27B:
					/* FALL THRU */		// imitate a 28-Speedstep decoder (for multimaus' sake)
				case FMT_DCC_28:
					id = 0x02;
					if (speed) speed += 3;
					speed = ((speed >> 1) & 0x0F) | ((speed & 1) << 4);
					break;
				case FMT_DCC_126:
				case FMT_DCC_SDF:
				case FMT_M3_126:
					if (speed) speed++;		// skip emergency step
					id = 0x04;
					break;
			}
			if (l->speed & 0x80) speed |= 0x80;		// direction bit must be included in bit 7
			id |= (node->flags & NODEFLG_LB);
			funcA = (uint8_t)((l->funcs[0] >> 1) & 0x0F) | ((l->funcs[0] & 1) ? 0x10 : 0);	// function A	(F0, F4 .. F1)
			funcB = (uint8_t)((l->funcs[0] >> 5) & 0xFF);									// function B	(F12 .. F5)
			for(ui8 = 0; ui8 < MAX_NODES; ui8++) {
				if(nodes[ui8].alive) {
					if(node->adr != nodes[ui8].adr) {
						if(node->loco == nodes[ui8].loco) id |= 0x08;
					}
				}
			}

			if((c = consist_findConsist(node->loco)) != NULL) {
				for (int i = 0; i < MAX_CONSISTLENGTH; i++) {
					if (node->loco != abs(c->adr[i])) loco = abs(c->adr[i]);
				}
				printf ("%s() Lok: %d ist in DT mit Lok: %d\n", __func__, node->loco, loco);
				id |= 0x60;
				if(loco > 99) loco |= 0xC000;
				_xpn_sendmessage(XPN_ANSWER | node->adr, 0xE6, (int) id, (int) speed, (int) funcA, (int) funcB, loco >> 8, loco);
				return;
			}

			printf ("%s() node: %d, id: %d, speed: %d, funcA: %d, funcB: %d\n", __func__, node->adr, id, speed, funcA, funcB);
		}
	}
	_xpn_sendmessage(XPN_ANSWER | node->adr, 0xE4, (int) id, (int) speed, (int) funcA, (int) funcB);
}

static bool POM_Result (struct decoder_reply *msg, flexval priv)
{
	(void) priv;

	lastPOM_CV.val = msg->data[0];
	if (msg->mt == DECODERMSG_NOANSWER) lastPOM_CV.val = -1;
	printf ("\n\n%s() lastPOM_CV: %d\n", __func__, lastPOM_CV.val);
	return false;
}

static int xpn_loco(struct xpn_node *node, struct blockbuf *msg)
{
	int loco, locoDT;
	uint8_t speed, funcA, funcB;
	ldataT *l;

	loco = ((msg->buf[2] << 8) | msg->buf[3]) & 0x3FFF;		// loco addresses in the range of 0 .. 16383

	switch (msg->buf[1]) {
		case 0x00:	// request loco information
			if (loco && loco != node->loco) {
				node->loco = loco;
			}
			xpn_locoinformation(node);
			return 1;

		case 0x07:// request function information switch / momentary (F0 - F12)
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0xE3, 0x50, 0x00, 0x00);
			return 1;

		case 0x08:// request function information switch / momentary (F13 - F28)
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0xE4, 0x51, 0x00, 0x00, 0x0F);
			return 1;

		case 0x09:	// request function information status (F13 - F28)
//			funcA = (xcp->funcs >> 13) & 0xFF;	// function A	(F13 - F20)
//			funcB = (xcp->funcs >> 21) & 0xFF;	// function B	(F21 - F28)
			funcA = funcB = 0;
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0xE3, 0x52, (int) funcA, (int) funcB);
			return 1;

		case 0x0A:	// request function information status (F13 - F28)
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0xE6, 0x54, 0, 0, 0, 0, 0);	// F29 - F68 not supported at this time
			return 1;

		case 0x0B:	// request function information status (F29 - F68)
			_xpn_sendmessage(XPN_ANSWER | node->adr, 0xE6, 0x53, 0, 0, 0, 0, 0);	// F29 - F68 not supported at this time
			return 1;

		case CMD_SPEED27:// set speed and direction (27 speed, see coding scheme above)
		case CMD_SPEED28:// set speed and direction (28 speed, see coding scheme above)
			speed = ((msg->buf[4] & 0x0F) << 1) | ((msg->buf[4] & 0x10) >> 4);
			if (speed > 0 && speed < 3) {	// Emergency-STOP
				speed = (msg->buf[4] & 0x80);
//				xpn_emergencystop(xcp);
			} else {		// normal speed setting
				if (speed) speed -= 3;
				speed = (msg->buf[4] & 0x80) | speed;
			}
			rq_setSpeed (loco, speed);
			node->loco = loco;
			node->flags &= ~NODEFLG_LB;
			return 0;

		case CMD_SPEED14:	// set speed and direction (14 speed)
			msg->buf[4] &= 0x8F;
			/* FALL THRU */
		case CMD_SPEED128:	// set speed and direction (128 speed)
			speed = msg->buf[4] & 0x7F;
			if (speed == 1) {	// Emergency-STOP
				speed = (msg->buf[4] & 0x80);
//				xpn_emergencystop(xcp);
			} else {		// normal speed setting
				if (speed) speed--;
				speed = (msg->buf[4] & 0x80) | speed;
			}
			rq_setSpeed (loco, speed);
			node->loco = loco;
			node->flags &= ~NODEFLG_LB;
			return 0;

		case CMD_FG1:	// set function group 1 (F0, F4 .. F1)
			rq_setFuncMasked(loco, ((msg->buf[4] & 0x0F) << 1) | ((msg->buf[4] & 0x10) >> 4), FUNC_F0_F4);
			node->loco = loco;
			node->flags &= ~NODEFLG_LB;
			return 0;

		case CMD_FG2:	// set function group 2 (F8 .. F5)
			rq_setFuncMasked(loco, (msg->buf[4] & 0x0F) << 5, FUNC_F5_F8);
			node->loco = loco;
			node->flags &= ~NODEFLG_LB;
			return 0;

		case CMD_FG3:	// set function group 3 (F12 .. F9)
			rq_setFuncMasked(loco, (msg->buf[4] & 0x0F) << 9, FUNC_F9_F12);
			node->loco = loco;
			node->flags &= ~NODEFLG_LB;
			return 0;

		case CMD_FG4:	// set function group 4 (F20 .. F13)
		case CMD_FG4R:	// set function group 4 (F20 .. F13) -> ROCO MultiMaus
			rq_setFuncMasked(loco, msg->buf[4] << 13, FUNC_F13_F20);
			node->loco = loco;
			node->flags &= ~NODEFLG_LB;
			return 0;

		case CMD_FG5:	// set function group 5 (F28 .. F21)
			rq_setFuncMasked(loco, msg->buf[4] << 21, FUNC_F21_F28);
			node->loco = loco;
			node->flags &= ~NODEFLG_LB;
			return 0;

		case CMD_BUILD_DT:	// build or dissolve a consist of two locos
			loco = (msg->buf[2] & ~0xC0)<<8 | msg->buf[3];
			locoDT = (msg->buf[4] & ~0xC0)<<8 | msg->buf[5];
			l = loco_call (loco, true);
			if(l->speed < 128) {
				loco *= -1;
			}
			if(locoDT) {
				l = loco_call (locoDT, true);
				if(l->speed < 128) {
					locoDT *= -1;
				}
				if (consist_couple(loco, locoDT) == NULL) {
					printf ("\n%s() DT1: %d / DT2: %d cannot be build\n", __func__, loco, locoDT);
				} else {
					printf ("\n%s() DT1: %d / DT2: %d -> OK\n", __func__, loco, locoDT);
				}
			} else {
				printf ("\n%s() DT1: %d DT dissolved\n", __func__, loco);
				consist_remove(loco);
			}
			return 0;

		case CMD_POM:	// POM read / write
			lastPOM_CV.CV = ((msg->buf[4] & 0x03) << 8) | msg->buf[5];
			if((msg->buf[4] & 0xFC) == 0xE4) {			//POM read
				log_msg (LOG_INFO, "%s(): POM read Adr.: %d, CV: %d\n", __func__, loco, lastPOM_CV.CV);
				dccpom_readByte(loco, DECODER_DCC_MOBILE, lastPOM_CV.CV, POM_Result, fvNULL);
			} else if((msg->buf[4] & 0xFC) == 0xEC) {	// POM write
				log_msg (LOG_INFO, "%s(): POM write Adr.: %d, CV: %d = %d\n", __func__, loco, lastPOM_CV.CV, msg->buf[6]);
				dccpom_writeByte(loco, DECODER_DCC_MOBILE, lastPOM_CV.CV, msg->buf[6], POM_Result, fvNULL);
			} else if((msg->buf[4] & 0xFC) == 0x7C) {	// POM write bit
				log_msg (LOG_INFO, "%s(): POM write bit Adr.: %d, CV: %d, bit %d=%d\n", __func__, loco, lastPOM_CV.CV,
						msg->buf[6] & 0x07, (msg->buf[6] & 0x08) ? 1 : 0);
				dccpom_writeBit(loco, DECODER_DCC_MOBILE, lastPOM_CV.CV, msg->buf[6] & 0x07, !!(msg->buf[6] & 0x08), POM_Result, fvNULL);
			}
			node->loco = loco;
			node->flags &= ~NODEFLG_LB;
			return 0;

		default:
			xpn_unknown(node, msg);
			return 1;
	}
	return 0;
}

/**
 * First level interpreter function. It checks the header byte for the requested
 * kind of function and delegates to further handling routines.
 *
 * @param node	the node index that sent us this query (i.e. the current node as hold in global 'nodeidx'
 * @param msg	the pointer to the rxbuf with the currently received data
 * @return		0 if no answer was sent, 1 if an answer was sent, a negative value as error code
 */
static int xpn_interpret (struct xpn_node *node, struct blockbuf *msg)
{
	if(!(node->flags & NODEFLG_ACTIVE)) {
		node->flags |= NODEFLG_ACTIVE;
		xn_controlEvent (node, 1);
	}
	node->alive = XPN_ALIVE;

	switch (msg->buf[0] >> 4) {
		case 0x02:	// system requests
			printf("SYS ");
			return xpn_requests(node, msg);
		case 0x04:	// turnout status requests
			printf("TOstat\n");
			return xpn_tostat(node, msg);
			break;
		case 0x05:	// turnout action requests
			printf("TOact\n");
			return xpn_toaction(node, msg);
			break;
		case 0x08:	// halt all locos
			printf("HALTALL\n");
//			return bus_sendbyte(0, ANS_REQUESTSTATUS, STAT_HALT);
			break;
		case 0x09:	// emergency stop individual
			printf("HALT\n");
//			return xpn_emergencystop(xcp);
			break;
		case 0x0E:	// loco control and information
			printf("LOCO\n");
			switch(msg->buf[0] & 0xF) {
				case 3:
					switch(msg->buf[1]) {
						case 0:		// ask for loco information
							printf ("%s() Loco: %d \n", __func__, msg->buf[2] ? ((msg->buf[2] << 8) | 0xC000) + msg->buf[3] : msg->buf[3]);
							break;
					}
					break;
			}
			return xpn_loco(node, msg);
		default:
			printf("UNKNOWN\n");
			return xpn_unknown(node, msg);
			break;
	}

	return 0;
}

static int xpn_checkMessage (struct blockbuf *msg)
{
	int i;
	uint8_t xor;

	if (!msg) return -1;		// parameter error
	if ((msg->len < 2) || (msg->len != ((msg->buf[0] & 0x0F) + 2))) return 2;	// length error
	for (i = 0, xor = 0; i < msg->len; i++) {
		xor ^= msg->buf[i];
	}
	if (xor) return 3;			// checksum error

	return 0;
}

static int xpn_nextNode (int curr_node)
{
	do {
		if (++curr_node >= MAX_NODES) {
			curr_node = 1;
			pollcycle++;
		}
	} while ((pollcycle != 0) && (!(nodes[curr_node].flags & NODEFLG_ACTIVE)));

	return curr_node;
}

static bool xpn_eventhandler (eventT *e, void *priv)
{
	ldataT *l;
	int i;
	theTime = (struct modeltime *)e->src;

	(void) priv;

//	printf ("%s() %s\n", __func__, (e->tid == xpn_task) ? "OWN EVENT" : "REMOTE EVENT");
	if (e->tid == xpn_task) return true;				// this is an event we triggered ourself, so don't report back!
	if ((l = (ldataT *) e->src) == NULL) return true;	// no loco given as source ... ignore this event

	if(e->ev == EVENT_MODELTIME) {
//		printf ("%s(): Modeltime: day: %d, hour: %d, Minute: %d\n", __func__, theTime->mday, theTime-> hour, theTime-> min);
		bTimeUpdate = true;
	}
	for (i = 0; i < MAX_NODES; i++) {
		if ((nodes[i].flags & NODEFLG_ACTIVE) && (nodes[i].loco == l->loco->adr)) {
			printf ("%s(): X-Node %d Loco %d %s: speed %d funcs 0x%08lx\n", __func__, i, l->loco->adr,
					(e->ev == EVENT_LOCO_SPEED) ? "EVENT_LOCO_SPEED" : "EVENT_LOCO_FUNCTION", l->speed, l->funcs[0]);
			nodes[i].flags |= NODEFLG_LB | NODEFLG_EVENT | NODEFLG_INFORM;
		}
	}

	return true;
}

struct xpn_node *get_nodes(void)
{
	return nodes;
}

void vXpressNet (void *pvParameter)
{
	int i;
	enum trackmode oldState;

	(void) pvParameter;

	oldState = !rt.tm;

	usart1_init();

	printf ("%s() startup\n", __func__);

	nodeidx = 0;
	pollcycle = 0;
	for (i = 0; i < MAX_NODES; i++) {		// one-time initialsation of the node addresses
		nodes[i].adr = i;
	}
	xpn_task = xTaskGetCurrentTaskHandle();
	event_register(EVENT_LOCO_FUNCTION, xpn_eventhandler, NULL, 0);
	event_register(EVENT_LOCO_SPEED, xpn_eventhandler, NULL, 0);
	event_register(EVENT_MODELTIME, xpn_eventhandler, NULL, 0);
	mt_report();

	for (;;) {

		// Check for modeltime event
		if(bTimeUpdate) {
			bTimeUpdate = false;
			_xpn_sendmessage (XPN_ANSWER, 0x63, 3, (theTime->mday << 5) | (theTime->hour), theTime->min);
			ulTaskNotifyTake(pdTRUE, 1000);
		}
		// Check for changed system state
		if(rt.tm != oldState) {
			// some global messages if necessary
			if (oldState == TM_DCCPROG && rt.tm != TM_GO) {		// release PROG-status for XpressNet
				_xpn_sendmessage (XPN_ANSWER, 0x61, 1);
			}
			switch(rt.tm) {
				case TM_HALT:
					if(!PT_active) _xpn_sendmessage (XPN_ANSWER, 0x81, 0);
					break;

				default:
				case TM_STOP:
				case TM_SHORT:
				case TM_RESET:
					if(!PT_active) _xpn_sendmessage (XPN_ANSWER, 0x61, 0);
					break;

				case TM_DCCPROG:
					_xpn_sendmessage (XPN_ANSWER, 0x61, 2);
					break;

				case TM_GO:
					PT_active = false;
					_xpn_sendmessage (XPN_ANSWER, 0x61, 1);
					break;
			}
			ulTaskNotifyTake(pdTRUE, 1000);

			oldState = rt.tm;
		}

		// check for events to report to nodes
		for (i = 0; i < MAX_NODES; i++) {
			if (nodes[i].flags & NODEFLG_INFORM) {
				xpn_lostControl(&nodes[i]);
				nodes[i].flags &= ~NODEFLG_INFORM;
				ulTaskNotifyTake(pdTRUE, 100);						// wait for the message to be sent completely
			}
		}

		nodeidx = xpn_nextNode(nodeidx);
		xpn_sendmessage(XPN_NORMAL_INQUIRY | nodeidx, 0, NULL);
		ulTaskNotifyTake(pdTRUE, 1000);
		if (xpn_checkMessage(&rxbuf) == 0) {
			printf ("%s(%d): %d bytes", __func__, nodeidx, rxbuf.len);
			for (i = 0; i < rxbuf.len; i++) {
				printf (" 0x%02x", rxbuf.buf[i]);
			}
			printf ("\n");
			if (xpn_interpret (&nodes[nodeidx], &rxbuf) == 1) {		// if the interpreter returns 1, it did send an answer message to the requesting node
				ulTaskNotifyTake(pdTRUE, 100);						// wait for the answer message to be sent completely
			}
		} else if (rxbuf.len > 0) {
			fprintf (stderr, "%s(): error in message (%d bytes):", __func__, rxbuf.len);
			for (i = 0; i < rxbuf.len; i++) {
				fprintf (stderr, " 0x%02x", rxbuf.buf[i]);
			}
			fprintf (stderr, "\n");
		}
		if(nodes[nodeidx].alive) {
/*			if(nodes[nodeidx].alive < 100) {
				_xpn_sendmessage(XPN_ANSWER | nodeidx, 0x61, 0x80);	// error as ping
				xpn_sendmessage( nodeidx, 0, NULL);	// error as ping
				ulTaskNotifyTake(pdTRUE, 100);
			}*/
			if(!--nodes[nodeidx].alive) {
				fprintf (stderr, "node: %d lost\n", nodeidx);
				nodes[nodeidx].flags = 0;
				xn_controlEvent(&nodes[nodeidx], 0);
			}
		}
	}
}

/*
 * The IRQ handler manages the different phases of the communication regarding a single
 * information exchange.
 *
 * It starts with enabled TXFNE (TX-FIFO not empty) interrupt and fills up the TX-FIFO
 * until no more data is in the txbuf.
 *
 * $$$$$ OLD $$$$$
 * If the last data byte is put to the TX-FIFO the
 * TXFNE interrupt is disabled and the TC (transmission complete) interrupt is enabled.
 *
 * When TC triggers, all data bytes are sent out and the echoed characters in the RX-FIFO
 * can be flushed, a possible overrun flag be cleared and the TC interrupt itself can be
 * disabled.
 *
 * To begin reception the RXFTIE (RX-FIFO threshold flag) together with the RTOIE (receiver
 * timeout flag) is set. As soon as the RTOF flag gets set, the remaining characters from
 * the FIFO are read and put to the receiver buffer. The XPressNet-thread is then notified,
 * that communication is done and further interrupts are disabled.
 */
void USART1_IRQHandler (void)
{
	static int skip_rx;

	BaseType_t xHigherPriorityTaskWoken = 0;

	// Step 1: put all TX characters to queue
	while ((USART1->CR1 & USART_CR1_TXEIE_TXFNFIE) && (USART1->ISR & USART_ISR_TXE_TXFNF)) {
		if (txbuf.idx == 0) skip_rx = 1;	// the callbyte is already sent in xpn_sendmessage()
		if (txbuf.idx < txbuf.len) {
			USART1->TDR = txbuf.buf[txbuf.idx++];
			skip_rx++;
		} else {	// all characters are transmitted, switch from TXFNF to TC interrupt
			CLEAR_BIT (USART1->CR1, USART_CR1_TXEIE_TXFNFIE);
			SET_BIT (USART1->CR1, USART_CR1_TCIE);
		}
		USART1->ICR = USART_ICR_TCCF;				// clear transmission complete interrupt flag
	}

	// Step2: clear overrun, and enable receiver timeout
	if ((USART1->CR1 & USART_CR1_TCIE) && (USART1->ISR & USART_ISR_TC)) {
		CLEAR_BIT (USART1->CR1, USART_CR1_TCIE);			// disable TC interrupt
		USART1->ICR = USART_ICR_RTOCF | USART_ICR_ORECF;	// clear receiver timeout and overrun error interrupt flag
		SET_BIT (USART1->CR1, USART_CR1_RTOIE);				// enable receiver timeout interrupt
	}

	// intermediate step: receive characters from RX-FIFO (until FIFO is empty), ignoring the number of sent chars
	while ((USART1->CR3 & USART_CR3_RXFTIE) && (USART1->ISR & USART_ISR_RXNE_RXFNE)) {
		if (USART1->ISR & USART_ISR_FE) rxbuf.flags |= RXERR_FRAMING;
		if (USART1->ISR & USART_ISR_NE) rxbuf.flags |= RXERR_NOISE;
		if (USART1->ISR & USART_ISR_ORE) rxbuf.flags |= RXERR_OVERRUN;
		if (rxbuf.len >= MAX_BLKLEN) {
			SET_BIT (USART1->RQR, USART_RQR_RXFRQ);		// flush receiver queue
		} else {
			rxbuf.buf[rxbuf.len++] = USART1->RDR & 0xFF;
			if (skip_rx > 0) {
				rxbuf.len--;
				skip_rx--;
			}
		}
	}

	// Step 4 (final): as soon as receiver timeout is set, clear receiver interrupts and read all remaining
	// characters. When finished, wake up the XPressNet task.
	if ((USART1->CR1 & USART_CR1_RTOIE) && (USART1->ISR & USART_ISR_RTOF)) {
		CLEAR_BIT (USART1->CR1, USART_CR1_RTOIE);		// disable receiver timeout interrupt
		while ((rxbuf.len < MAX_BLKLEN) && (USART1->ISR & USART_ISR_RXNE_RXFNE)) {
			rxbuf.buf[rxbuf.len++] = USART1->RDR & 0xFF;
			if (skip_rx > 0) {
				rxbuf.len--;
				skip_rx--;
			}
		}
		if (rxbuf.len >= MAX_BLKLEN) SET_BIT (USART1->RQR, USART_RQR_RXFRQ);		// flush receiver queue
		vTaskNotifyGiveFromISR(xpn_task, &xHigherPriorityTaskWoken);
	}

	USART1->ICR = 0xFFFFFFFF;		// clear all interrupt flags
	NVIC_ClearPendingIRQ(USART1_IRQn);
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
