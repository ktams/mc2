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
 * @file	Track/railcom.c
 *
 * This module deals with reception of DCC railcom packets. It is triggered by the
 * signal generation in Track/signal.c - in all cases from an interrupt context.
 *
 * Some provision have been made to allow senseful decode of answers to POM write
 * commands. Some decoders just sit there and answer only when enough packets
 * (according to RCN-214 two packet are needed) are received and stable storage is
 * done, others reply for some packets with the old content of the overwritten CV
 * (for example TAMS).
 *
 * Therefor, a filter method had to be implemented to ignore the premature messages
 * with old content and additional effort was needed to send the last non-matching
 * reply or a timeout message if the write didn't succeed in the end. So, please,
 * always keep in mind, that the whole thing is more complicated that it may look at
 * a first glance.
 *
 * Dealing with DCC-A even gives us more problems. It bundles the two windows and
 * partly switches to a non-ID-based data transfer. That enables RailCom decoders to
 * transmit up to 6 data bytes in one go. Additionally, a CRC for the data blocks is
 * implemented (see dcc_a.c for the implementation).
 */

#include <stdio.h>
#include <string.h>
#include "rb2.h"
#include "decoder.h"

#define WINDOW_DELAY			50		///< the first delay before we enable the UART (i.e. until the railcom cutout starts)
#define WINDOW1_DURATION		165		///< the time after which the receive window is switched to #2 (in µs)
#define WINDOW2_DURATION		270		///< the time after which the receive window #2 is terminated (in µs)
#define BIDIB_ACK_DELAY			1500	///< the time after end of window #2 where the ACK line of the BiDiBus is checked

#define MAX_RAW_BYTES_WIN2		6		///< a maximum of six raw bytes can be received in window #2
#define MAX_RC_DATA				6		///< a maximum of four data bytes can be received in window #2
#define MAX_MSG_WIN2			3		///< a maximum of three messages can be received in window #2
#define MAX_MESSAGES			4		///< a maximum of four messages can be received in railcom window
#define MSGQUEUE_LEN			16		///< the queue need not to be too long - everything should be done in a timely fashion

#define RC_WINDOWDATA			0		///< received data from railcom (new version, no window splitting anymore)
#define RC_FINISH				1		///< the railcom window is closed now and contents can be interpreted
#define RC_DEFINETARGET			2		///< define a message target for reporting to reply_deliver()
#define RC_DEFINEFILTER			3		///< define a message filter for reporting to reply_deliver()
#define RC_SENDLASTGOOD			4		///< send the last message received that previously was filtered out (thereby canceling the filter)
#define RC_CLR_DECADR			5		///< sent with the output start to clear the "Last received Address" variables

enum railcom_mode {
	RCMODE_STANDARD = 0,				///< the standard railcom mode, ACK is every leagal answer in Win#2
	RCMODE_DCCA_ID,						///< windows bundeld for 6 Bytes answer to DCC-A commands, ID13 / ID15 expected
	RCMODE_DCCA_SHORT_INFO,				///< windows bundeld for 6 Bytes answer to DCC-A commands, the ShortInfo block is expected
	RCMODE_DCCA_BLK_HEAD,				///< windows bundeld for 6 Bytes answer to DCC-A commands, the block header is expected
	RCMODE_DCCA_BLK_DATA,				///< windows bundeld for 6 Bytes answer to DCC-A commands, block data (everything that follows the header)
};

struct cmd_decoder {
	const char	*name;					///< the verbose name of the command (for debugging purposes)
	uint8_t		 bytes;					///< the number of raw bytes this command needs
	uint8_t		 bits;					///< the number of data bits this cmd occupies (inlcuding the 4 command bits)
	dec_msgtype	 mt;					///< map a railcom message to a message type
};

/**
 * Additional information that must be registered to generate appropriate replies
 */
struct rc_target {
	int			 adr;					///< the decoder address
	dec_type	 dt;					///< the type of decoder
	rdbk_type	 rdt;					///< the expected RailCom interpretion method (read back type)
	cvadrT		 cva;					///< if we are awaiting an answer to a CV read or write, this is the CV address
	flexval		 fv;					///< an additional information for the receiving (notified) function
};

enum windowStat {
	WSTAT_DELAY = 0,					///< a delay before the cutout starts, UART5 is disabled
	WSTAT_WIN1,							///< we are in Channel #1, UART5 will be enabled
	WSTAT_WIN2,							///< we are in Channel #2, UART5 is enabled
	WSTAT_BIDIB_ACK						///< we wait a little before checking the ACK line of the BiDiBus, UART5 is already disabled
};

struct rc_status {
	struct bitbuffer	*sigbuf;		///< the signal buffer that is related to this railcom cutout
	rdbk_type			 type;			///< the type of the railcom window(s)
	enum windowStat		 wstat;			///< started by railcomTrigger() and stepped by TIM7 IRQ
	uint8_t				 data[8];		///< all (decoded) bytes of the railcom answer
	int					 idx;			///< receiver index (forced to 2 if smaller on entry of win #2)
	volatile uint8_t	*interpret;		///< the point where we need to start interpretion of next message
	bool				 reply_sent;	///< we have sent an answer (currently only relevant for DCC-A)
};

static const struct cmd_decoder app_common[] = {
	{ "POM",	2, 12, DECODERMSG_POM },		// 0x0: app:pom
	{ "ADR-H",	2, 12, DECODERMSG_ADRH },		// 0x1: app:adr_high (belongs to CH1)
	{ "ADR-L",	2, 12, DECODERMSG_ADRL },		// 0x2: app:adr_low (belongs to CH1)
	{ "EXT",	3, 18, DECODERMSG_EXT },		// 0x3: app:ext (location service) via XF1 OFF
	{ "STAT1",	2, 12, DECODERMSG_STAT1 },		// 0x4: app:stat1
	{ "TIME",	2, 12, DECODERMSG_TIME },		// 0x5: app:time
	{ "ERRM",	2, 12, DECODERMSG_ERR },		// 0x6: app:error
	{ "DYN",	3, 18, DECODERMSG_DYN },		// 0x7: app:dyn
	{ "XPOM00",	6, 36, DECODERMSG_XPOM00 },		// 0x8: app:xpom (serialisation SS = 0b00)
	{ "XPOM01",	6, 36, DECODERMSG_XPOM01 },		// 0x9: app:xpom (serialisation SS = 0b01)
	{ "XPOM10",	6, 36, DECODERMSG_XPOM10 },		// 0xA: app:xpom (serialisation SS = 0b10)
	{ "XPOM11",	6, 36, DECODERMSG_XPOM11 },		// 0xB: app:xpom (serialisation SS = 0b11)
	{ "TEST",	0,  0, DECODERMSG_ANY },		// 0xC: app:Test Feature (should be ignored here)
	{ "STATE",	6, 36, DECODERMSG_DECSTATE },	// 0xD: app:Decode_State (DCC-A)
	{ "SEARCH",	2, 12, DECODERMSG_TIME },		// 0xE: time for ontrack searching of decoders via XF2 OFF
	{ "DID",	6, 36, DECODERMSG_UNIQUE },		// 0xF: app:Decode_Unique (DCC-A)
};

//static const struct cmd_decoder app_mobile_alt[16] = {
//	{ "POM",	2, 12 },		// 0x0: app:pom
//	{ "ADR-H",	2, 12 },		// 0x1: app:adr_high (belongs to CH1)
//	{ "ADR-L",	2, 12 },		// 0x2: app:adr_low (belongs to CH1)
//	{ "SPEED",	2, 12 },		// 0x3: app:speed (old meaning of ID3)
//	{ "STAT1",	2, 12 },		// 0x4: app:stat1
//	{ "TIME",	2, 12 },		// 0x5: app:time
//	{ "ERRM",	2, 12 },		// 0x6: app:error
//	{ "DYN",	3, 18 },		// 0x7: app:dyn
//	{ "XPOM00",	6, 36 },		// 0x8: app:xpom (serialisation SS = 0b00)
//	{ "XPOM01",	6, 36 },		// 0x9: app:xpom (serialisation SS = 0b01)
//	{ "XPOM10",	6, 36 },		// 0xA: app:xpom (serialisation SS = 0b10)
//	{ "XPOM11",	6, 36 },		// 0xB: app:xpom (serialisation SS = 0b11)
//	{ "TEST",	0,  0 },		// 0xC: app:Test Feature (should be ignored here)
//	{ "STATE",	6, 36 },		// 0xD: app:Decode_State (DCC-A)
//	{ "SEARCH",	2, 12 },		// 0xE: time for ontrack searching of decoders via XF2 OFF
//	{ "DID",	6, 36 },		// 0xF: app:Decode_Unique (DCC-A)
//};

// databytes with special meanings
#define RCB_INVALID		0x40		// invalid 6/8 coding detected
#define RCB_ACK1		0xF0		// special entity ACK1
#define RCB_ACK2		0xF1		// special entity ACK2
#define RCB_RSVD1		0xF2		// special entity RSVD1 (RSVD = reserved)
#define RCB_RSVD2		0xF3		// special entity RSVD2
#define RCB_RSVD3		0xF4		// special entity RSVD3
#define RCB_NACK		0xF5		// special entity NACK
#define RCB_ERROR		0xFF		// UART error like parity (not used), overflow (unlikely) or framing (stop bit related)

static const uint8_t RC_revtable[] = {
	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID,	//0
	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_ACK1,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x33,		//1
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x34, 		RCB_INVALID, 0x35, 		0x36, 		RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x3A,		//2
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x3B, 		RCB_INVALID, 0x3C, 		0x37, 		RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x3F, 		RCB_INVALID, 0x3D, 		0x38, 		RCB_INVALID,	//3
	RCB_INVALID, 0x3E, 		0x39, 		RCB_INVALID, RCB_NACK, 	RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x24,		//4
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x23, 		RCB_INVALID, 0x22, 		0x21, 		RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x1F, 		RCB_INVALID, 0x1E, 		0x20, 		RCB_INVALID,	//5
	RCB_INVALID, 0x1D, 		0x1C, 		RCB_INVALID, 0x1B, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x19, 		RCB_INVALID, 0x18, 		0x1A, 		RCB_INVALID,	//6
	RCB_INVALID, 0x17, 		0x16, 		RCB_INVALID, 0x15, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, 0x25, 		0x14, 		RCB_INVALID, 0x13, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,		//7
	0x32, 		RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_RSVD3,	//8
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x0E, 		RCB_INVALID, 0x0D, 		0x0C, 		RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x0A, 		RCB_INVALID, 0x09, 		0x0B, 		RCB_INVALID,	//9
	RCB_INVALID, 0x08, 		0x07, 		RCB_INVALID, 0x06, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, 0x04, 		RCB_INVALID, 0x03, 		0x05, 		RCB_INVALID,	//A
	RCB_INVALID, 0x02, 		0x01, 		RCB_INVALID, 0x00, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, 0x0F, 		0x10, 		RCB_INVALID, 0x11, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,		//B
	0x12, 		RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_RSVD2, 	RCB_INVALID, 0x2B, 		0x30, 		RCB_INVALID,	//C
	RCB_INVALID, 0x2A, 		0x2F, 		RCB_INVALID, 0x31, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, 0x29, 		0x2E, 		RCB_INVALID, 0x2D, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,		//D
	0x2C, 		RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_INVALID, RCB_RSVD1, 	0x28,		RCB_INVALID, 0x27, 		RCB_INVALID, RCB_INVALID, RCB_INVALID,	//E
	0x26, 		RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID,
	RCB_ACK2, 	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID,	//F
	RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID, RCB_INVALID,
};

//static const uint8_t RC_fwdtable[] = {
//	0xAC, 0xAA, 0xA9, 0xA5, 0xA3, 0xA6, 0x9C, 0x9A, 0x99, 0x95, 0x93, 0x96,	0x8E, 0x8D, 0x8B, 0xB1,	// 0x00 - 0x0F
//	0xB2, 0xB4, 0xB8, 0x74, 0x72, 0x6C, 0x6A, 0x69, 0x65, 0x63, 0x66, 0x5C, 0x5A, 0x59, 0x55, 0x53,	// 0x10 - 0x1F
//	0x56, 0x4E, 0x4D, 0x4B, 0x47, 0x71, 0xE8, 0xE4, 0xE2, 0xD1, 0xC9, 0xC5, 0xD8, 0xD4, 0xD2, 0xCA,	// 0x20 - 0x2F
//	0xC6, 0xCC, 0x78, 0x17, 0x1B, 0x1D, 0x1E, 0x2E, 0x36, 0x3A, 0x27, 0x2B, 0x2D, 0x35, 0x39, 0x33	// 0x30 - 0x3F
//};

static volatile uint8_t filter[MAX_RAW_BYTES_WIN2];	///< a filter applied to received content in window#2 (filled with the expected raw bytes)
static volatile int filter_len;						///< the length of the filter
static volatile struct rc_status stat;

/*
 * RailCom RX 250kBaud, 8n1
 */
static void uart5_init (void)
{
	UART5->CR1 = 0;		// disable UART5
	UART5->CR2 = 0;		// nothing used here
	UART5->CR1 = USART_CR1_FIFOEN | USART_CR1_RXNEIE_RXFNEIE;		// enable FIFO mode and FIFO not empty interrupt
	UART5->CR3 = (0b010 << USART_CR3_RXFTCFG_Pos);					// set FIFO threshold interrupt at half full (but do not enable it for now)

	UART5->PRESC = 0;	// no prescaler -> 100MHz kernel clock
	UART5->BRR = 400;	// 100MHz / 400 -> 250kbit/s

	NVIC_SetPriority(UART5_IRQn, 8);
	NVIC_ClearPendingIRQ(UART5_IRQn);
	NVIC_EnableIRQ(UART5_IRQn);
	UART5->ICR = 0xFFFFFFFF;				// clear all interrupt flags
	SET_BIT (UART5->CR1, USART_CR1_UE);		// enable the UART
}

static void tim7_init (void)
{
	TIM7->CR1 = TIM_CR1_ARPE;				// switch off and disable timer, set auto-Preload enable
	TIM7->CR2 = 0;
	TIM7->DIER = 0;							// disable interrupts
	TIM7->PSC = 199;						// count with 1MHz (i.e. 1µs/tick)

	NVIC_SetPriority(TIM7_IRQn, 9);
	NVIC_ClearPendingIRQ(TIM7_IRQn);
	NVIC_EnableIRQ(TIM7_IRQn);
}

void rc_init (void)
{
	uart5_init();
	tim7_init();
}

static void rc_copyDCCAdata (uint8_t *target, uint8_t *src)
{
	target[0] = ((src[0] & 0x3F) << 2) | ((src[1] & 0x30) >> 4);	// data[0] only contains 4 bits ID + 4 bits data
	target[1] = ((src[1] & 0x0F) << 4) | ((src[2] & 0x3C) >> 2);	// data[1] contains 4 bits from channel #1 and th top most 4 bits of channel #2
	target[2] = ((src[2] & 0x03) << 6) |  (src[3] & 0x3F);
	target[3] = ((src[4] & 0x3F) << 2) | ((src[5] & 0x30) >> 4);
	target[4] = ((src[5] & 0x0F) << 4) | ((src[6] & 0x3C) >> 2);
	target[5] = ((src[6] & 0x03) << 6) |  (src[7] & 0x3F);
}

void UART5_IRQHandler (void)
{
	BaseType_t xHigherPriorityTaskWoken = 0;
	const struct cmd_decoder *cd;
	uint8_t c, msg, id;
	uint8_t data[8];
	dec_msgtype mt;
	uint32_t status;
	int i;

	if (UART5->ISR & USART_ISR_ORE) UART5->ICR = USART_ICR_ORECF;
	while (UART5->ISR & USART_ISR_RXNE_RXFNE) {
		status = UART5->ISR;
		if (stat.wstat > WSTAT_WIN2) {		// we already are beyond window #2 - ignore received chars
			c = UART5->RDR;
			continue;
		}

		// translate raw character from UART thru 8-6 converion to six real data bits or special entity (ACK/NACK/...)
		c = RC_revtable[UART5->RDR];	// in any case, we MUST read the byte (i.e. pop it from FIFO)
		if ((status & (USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) != 0) {		// a character reception error occured
//			if (stat.type == READBACK_DCCA_ID) irqdbg_printf("%s() c=0x%02x%s%s%s\n", __func__, c, (status & USART_ISR_NE) ? " NE" : "",
//				(status & USART_ISR_FE) ? " FE" : "", (status & USART_ISR_PE) ? " PE" : "");
			UART5->ICR = USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;		// clear the character errors from FIFO
			c = RCB_ERROR;
		}

//		if ((stat.type == READBACK_DCCA_DATA) || (stat.type == READBACK_DCCA_ID) || (stat.type == READBACK_DCCA_SHORTINFO)) {
//			const char *dcca;
//			switch (stat.type) {
//				case READBACK_DCCA_DATA: dcca = "DCCA-DATA"; break;
//				case READBACK_DCCA_ID: dcca = "DCCA-ID"; break;
//				case READBACK_DCCA_SHORTINFO: dcca = "DCCA-SHORTINFO"; break;
//				default: dcca = "Impossible"; break;
//			}
//			irqdbg_printf("%s(%s) c=0x%02x w#%d@%lu (idx %d)\n", __func__, dcca, c, stat.wstat, TIM7->CNT, stat.idx);
//		}

//		if (stat.idx == 0) irqdbg_printf("%s() ", __func__);
//		irqdbg_printf("%02x ", c);
		// store received byte (6 bits) or entity (RCB_ACK1/2, RCB_NACK, RCB_RSVD1/2/3, RC_INVALID) in data array
		if (stat.idx < DIM(stat.data)) stat.data[stat.idx++] = c;
		else continue;		// indeed, we may receive a stray 9th character when switching off the cutout - don't handle it!!!!

		// we only start interpreting when we reached channel #2 (so we definitely can decide, what is in channel #1)
		if ((stat.wstat == WSTAT_WIN1) && (stat.idx < 2)) continue;

		switch (stat.type) {
			case READBACK_STANDARD:
			case READBACK_XPOM:
				if (stat.interpret == stat.data) {	// interpret channel #1 data
					// check if data in channel #1 is valid - if not just advance to byte #2 (first byte of window #2)
					if ((stat.data[0] < RCB_INVALID) && (stat.data[1] < RCB_INVALID)) {
						id = (stat.data[0] >> 2) & 0xFF;
						if (id == 1 || id == 2) {	// only these two message IDs are allowed in channel #1
							data[0] = ((stat.data[0] & 0x03) << 6) | (stat.data[1] & 0x3F);
							reply_callback(NULL, app_common[id].mt, 1, data);
						}
					}
					stat.interpret = &stat.data[2];			// advance to beginning of channel #2
				}
				while (stat.interpret < &stat.data[stat.idx]) {
					msg = *stat.interpret;
					if (msg >= RCB_INVALID) {	// interpret single-byte codes
						switch (msg) {
							case RCB_ACK1:
							case RCB_ACK2:
								reply_callback(stat.sigbuf, DECODERMSG_ACK, 0, NULL);
								sig_rcAck(stat.sigbuf);
								// stop further interpretion
								stat.idx = DIM(stat.data);
								stat.interpret = &stat.data[stat.idx];	// beyond interpretable range
								break;
							case RCB_NACK:
								reply_callback(stat.sigbuf, DECODERMSG_NACK, 0, NULL);
								sig_rcAck(stat.sigbuf);
								// stop further interpretion
								stat.idx = DIM(stat.data);
								stat.interpret = &stat.data[stat.idx];	// beyond interpretable range
								break;
							case RCB_RSVD1:
							case RCB_RSVD2:
							case RCB_RSVD3:	// these codes are reserved and ignored here
								break;
							default:		// anything else is an error and should stop interpretion
								stat.idx = DIM(stat.data);
								stat.interpret = &stat.data[stat.idx];	// beyond interpretable range
								break;
						}
						stat.interpret++;
					} else {
						id = (msg >> 2) & 0x0F;
						cd = &app_common[id];
						if ((&stat.data[stat.idx] - stat.interpret) < cd->bytes) break;		// not enougth data to interpret this message
						// stat.interpret now points to an interpretable message
						switch (cd->bits) {
							case 0:		// this might have been the test command - in any case, taking this literal would produce an endless loop
								stat.idx = DIM(stat.data);
								stat.interpret = &stat.data[stat.idx];	// beyond interpretable range
								break;
							case 12:	// 1 data byte with 8 bits
								data[0] = ((stat.interpret[0] & 0x03) << 6) | (stat.interpret[1] & 0x3F);
								reply_callback(stat.sigbuf, cd->mt, 1, data);
								sig_rcAck(stat.sigbuf);
								break;
							case 18:	// two data bytes, one full 8-bit, last with 6 LSBits
								data[0] = ((stat.interpret[0] & 0x03) << 6) | (stat.interpret[1] & 0x3F);
								data[1] = stat.interpret[2] & 0x3F;
								reply_callback(stat.sigbuf, cd->mt, 2, data);
								sig_rcAck(stat.sigbuf);
								break;
							case 24:	// three data bytes, two full 8-bit, last with 4 LSBits (currently not used!)
								data[0] = ((stat.interpret[0] & 0x03) << 6) | (stat.interpret[1] & 0x3F);
								data[1] = ((stat.interpret[2] & 0x3F) << 2) | ((stat.interpret[3] & 0x30) >> 4);
								data[2] = stat.interpret[3] & 0x0F;
								reply_callback(stat.sigbuf, cd->mt, 3, data);
								sig_rcAck(stat.sigbuf);
								break;
							case 36:	// four bytes, four full 8-bit)
								data[0] = ((stat.interpret[0] & 0x03) << 6) | (stat.interpret[1] & 0x3F);
								data[1] = ((stat.interpret[2] & 0x3F) << 2) | ((stat.interpret[3] & 0x30) >> 4);
								data[2] = ((stat.interpret[3] & 0x0F) << 4) | ((stat.interpret[4] & 0x3C) >> 2);
								data[3] = ((stat.interpret[4] & 0x03) << 6) | (stat.interpret[5] & 0x3F);
								reply_callback(stat.sigbuf, cd->mt, 4, data);
								sig_rcAck(stat.sigbuf);
								break;
						}
						stat.interpret += cd->bytes;
					}
				}
				break;
			case READBACK_POM:
			case READBACK_POMWRITE:		// these two cases are handled special as we are only interested in ID0 answers
				if (stat.interpret == stat.data) {	// interpret channel #1 data
					// check if data in channel #1 is valid - if not just advance to byte #2 (first byte of window #2)
					if ((stat.data[0] < RCB_INVALID) && (stat.data[1] < RCB_INVALID)) {
						id = (stat.data[0] >> 2) & 0xFF;
						if (id == 1 || id == 2) {	// only these two message IDs are allowed in channel #1
							data[0] = ((stat.data[0] & 0x03) << 6) | (stat.data[1] & 0x3F);
							reply_callback(NULL, app_common[id].mt, 1, data);
						}
					}
					stat.interpret = &stat.data[2];			// advance to beginning of channel #2
				}
				while (stat.interpret < &stat.data[stat.idx]) {
					msg = *stat.interpret;
					if (msg >= RCB_INVALID) {	// interpret single-byte codes
						switch (msg) {
							case RCB_ACK1:
							case RCB_ACK2:		// ACK will be ignored
//								irqdbg_printf("%s() adr %d: ACK\n", __func__, stat.sigbuf->adr);
								break;
							case RCB_NACK:		// NACK will cancel the read / write
//								irqdbg_printf("%s() adr %d: NACK\n", __func__, stat.sigbuf->adr);
								reply_callback(stat.sigbuf, DECODERMSG_NACK, 0, NULL);
								sig_rcAck(stat.sigbuf);
								// stop further interpretion
								stat.idx = DIM(stat.data);
								stat.interpret = &stat.data[stat.idx];	// beyond interpretable range
								break;
							case RCB_RSVD1:
							case RCB_RSVD2:
							case RCB_RSVD3:	// these codes are reserved and ignored here
//								irqdbg_printf("%s() adr %d: RSVD\n", __func__, stat.sigbuf->adr);
								break;
							default:		// anything else is an error and should stop interpretion
//								irqdbg_printf("%s() adr %d: UNDEF %#02x\n", __func__, stat.sigbuf->adr, msg);
								stat.idx = DIM(stat.data);
								stat.interpret = &stat.data[stat.idx];	// beyond interpretable range
								break;
						}
						stat.interpret++;
					} else {
						id = (msg >> 2) & 0x0F;
						cd = &app_common[id];
						if ((&stat.data[stat.idx] - stat.interpret) < cd->bytes) break;		// not enougth data to interpret this message
//						irqdbg_printf("%s() LOK %d CV %ld RPLY %s (%d bits)\n", __func__, stat.sigbuf->adr, stat.sigbuf->cva.cv, cd->name, cd->bits);
						// stat.interpret now points to an interpretable message
						switch (cd->bits) {
							case 0:		// this might have been the test command - in any case, taking this literal would produce an endless loop
								stat.idx = DIM(stat.data);
								stat.interpret = &stat.data[stat.idx];	// beyond interpretable range
								break;
							case 12:	// 1 data byte with 8 bits
								data[0] = ((stat.interpret[0] & 0x03) << 6) | (stat.interpret[1] & 0x3F);
								if (id == 0) {	// we are only interested in ID0 answers - anything else will be ignored
									if (stat.type == READBACK_POM || stat.sigbuf->dcc.valreceived) {
//										irqdbg_printf("%s(@%d) adr %d cv %ld=%d (%d) retries %d\n", __func__, stat.interpret - &stat.data[2],
//											stat.sigbuf->adr, stat.sigbuf->cva.cv + 1, data[0], stat.sigbuf->dcc.targetval, stat.sigbuf->repeat);
										reply_callback(stat.sigbuf, cd->mt, 1, data);
										sig_rcAck(stat.sigbuf);
									} else {
										stat.sigbuf->dcc.valreceived = true;	// we have received one answer - next one counts
									}
								}
								break;
							case 18:	// two data bytes, one full 8-bit, last with 6 LSBits
							case 24:	// three data bytes, two full 8-bit, last with 4 LSBits (currently not used!)
							case 36:	// four bytes, four full 8-bit)
								break;
						}
						stat.interpret += cd->bytes;
					}
				}
				break;
			case READBACK_DCCA_ID:			// response to LOGON_ENABLE (ID15) or LOGON_ASSIGN (ID13)
			case READBACK_DCCA_DATA:		// response to DCCA_SEELCT or DCCA_GETDATA
			case READBACK_DCCA_SHORTINFO:	// response to DCCA_SELECT_SHORTINFO
				if (stat.idx >= 8) {	// both windows are fully received - now interpret the whole block
					if (stat.type == READBACK_DCCA_DATA) mt = DECODERMSG_DCCABLOCK;				// ID is fixed - all bytes represent data
					else if (stat.type == READBACK_DCCA_SHORTINFO) mt = DECODERMSG_SHORTINFO;	// ID is fixed - all bytes represent data
					else mt = app_common[(stat.data[0] >> 2) & 0x0F].mt;						// ID is contained in the top most 4 bits
					for (i = 0; i < 8; i++) {
						if (stat.data[i] == RCB_ACK1 || stat.data[i] == RCB_ACK2) {
							if (i == 0) mt = DECODERMSG_ACK;	// if we had other valid data before, we stay with the original message type
							break;
						} else if (stat.data[i] >= RCB_INVALID) mt = DECODERMSG_COLLISION;				// some illeagl data was received
					}
					rc_copyDCCAdata (data, (uint8_t *) stat.data);
					reply_callback(stat.sigbuf, mt, 6, data);
					stat.reply_sent = true;
					sig_rcAck(stat.sigbuf);
				}
				break;
			case READBACK_DCCA_ACK:		// an expected ACK may be anywhere in the reply and the reply may be shorter than 8 bytes!
				for (i = 0; i < stat.idx; i++) {
					if (stat.data[i] == RCB_ACK1 || stat.data[i] == RCB_ACK2) {
						reply_callback(stat.sigbuf, DECODERMSG_ACK, 0, NULL);
						stat.reply_sent = true;
						sig_rcAck(stat.sigbuf);
						break;
					}
					if (stat.data[i] >= RCB_INVALID) break;		// some illeagl data was received - stop further reading
				}
				break;
			case READBACK_ACC_SRQ:
				// TODO check how the railcom message must be interpreted
//				reply_callback(stat.sigbuf, DECODERMSG_SRQ, 6, rcm.data);
				break;
			default:
				break;
		}
	}

	UART5->ICR = 0xFFFFFFFF;	// clear all interrupt flags
	NVIC_ClearPendingIRQ(UART5_IRQn);
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}

/**
 * Called from TIM1 interrupt to start the TIM7 that manages the receive windows.
 * After a delay it enables UART5 receiver and later switches the window to #2.
 * After disabling UART5 at the end of window #2 it checks the BiDiBus ACK line
 * after another delay.
 *
 * RCN-217 specifies a delay of 80µs before the begin of window #1. We already
 * start at 60µs, but that should be no problem. Window #1 should last for
 * around 100µs, but some decoders have severe jitter and are a little late.
 * So we will give this window a little more time.
 *
 * The window #2 should start at around 190µs (RCN-217: 193µs). At this point
 * we will switch to receiving window #2 bytes.
 *
 * \param bb		the bitbuffer from signal generation that this cutout belongs to
 */
void railcomTrigger (struct bitbuffer *bb)
{
	// program timer for two executions: window switch (#1 -> #2) and termination of window #2
	TIM7->DIER = 0;
	TIM7->ARR = WINDOW_DELAY - 1;			// set timing for start of railcom window #1
	TIM7->EGR = TIM_EGR_UG;					// update event: ARR is transferred to active register and counter + prescaler reset to zero
	TIM7->ARR = WINDOW1_DURATION - 1;		// prepare timing for window #1 (switchpoint between the two windows / channels)
	TIM7->SR = 0;							// clear update event
	TIM7->DIER = TIM_DIER_UIE;				// enable update event interrupt
	SET_BIT (TIM7->CR1, TIM_CR1_CEN);		// enable the timer

	stat.type = bb->rdt;
	stat.sigbuf = bb;
	stat.wstat = WSTAT_DELAY;
}

/**
 * This interrupt is dedicated to the railcom window timing and
 * BiDiBus ACK line handling.
 *
 * Attention: the timing to load into the ARR is the timing of the
 * phase after the next phase (double buffering of the timer registers).
 * This means: the timer already has the first two timings prepared
 * when it is started. So at the first interrupt we can setup the third
 * timing and so on.
 */
void TIM7_IRQHandler (void)
{
	BaseType_t xHigherPriorityTaskWoken = 0;

	if (TIM7->DIER & TIM_DIER_UIE && TIM7->SR & TIM_SR_UIF) {
		switch (stat.wstat) {
			case WSTAT_DELAY:				// here the railcom window begins
				stat.idx = 0;
				stat.reply_sent = false;
				stat.interpret = stat.data;
				memset ((void *) stat.data, RCB_INVALID, sizeof(stat.data));	// be sure to have no left overs
				if (MAINBST_ISON()) {		// if the booster is off, we get in trouble with the UART
//					if ((stat.type == READBACK_DCCA_DATA) || (stat.type == READBACK_DCCA_ID) || (stat.type == READBACK_DCCA_SHORTINFO)) {
//						irqdbg_printf("----------------------\n");
//					}
					SET_BIT (UART5->CR1, USART_CR1_RE /*| USART_CR1_UE */);	// enable the UART incl. receiver part
					UART5->RQR = USART_RQR_RXFRQ;		// flush receiver register/FIFO
				}
				TIM7->ARR = WINDOW2_DURATION - 1;		// prepare next timing
				stat.wstat = WSTAT_WIN1;
				break;
			case WSTAT_WIN1:				// channel #1 window is over - advance to channel #2
				TIM7->ARR = BIDIB_ACK_DELAY - 1;		// prepare next timing
				stat.idx = 2;
				stat.wstat = WSTAT_WIN2;
				break;
			case WSTAT_WIN2:				// channel #2 window is over - disable uart and wait for BiDiBus ACK sample point
				CLEAR_BIT(UART5->CR1, USART_CR1_RE /* | USART_CR1_UE */);	// disabling the UART resets all logic and cancels any ongoing reception
				stat.wstat = WSTAT_BIDIB_ACK;
				if (!stat.reply_sent) switch (stat.type) {
					case READBACK_DCCA_ID:
					case READBACK_DCCA_DATA:
					case READBACK_DCCA_SHORTINFO:
					case READBACK_DCCA_ACK:
						if (stat.idx > 2) {		// we have received something (at least more than window #1), but it was not OK
							reply_callback(stat.sigbuf, DECODERMSG_COLLISION, 0, NULL);
						} else {				// we really had no answer at all!
							reply_callback(stat.sigbuf, DECODERMSG_NOANSWER, 0, NULL);
						}
						break;
					default:
						break;
				}
//				if (stat.type == READBACK_DCCA_ACK && stat.sigbuf->adr == 0xFE) {
//					irqdbg_printf("%s() #%d %#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x\n", __func__, stat.idx,
//						stat.data[0], stat.data[1], stat.data[2], stat.data[3], stat.data[4], stat.data[5], stat.data[6], stat.data[7]);
//				}
				break;
			case WSTAT_BIDIB_ACK:
				TIM7->DIER = 0;
				CLEAR_BIT(TIM7->CR1, TIM_CR1_CEN);
				if (BIDIBUS_ACK()) sig_bidibACK();
				break;
		}
	}
	TIM7->SR = 0;					// clear update event
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
