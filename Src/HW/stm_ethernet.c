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
#include "lwip/etharp.h"
#include "ethernet.h"

#define TX_DESCRIPTORS		128						///< number of TX descriptors
#define RX_DESCRIPTORS		128						///< number of RX descriptors
#define RX_BUFFERSIZE		256						///< the size of each buffer used for the RX descriptors
#define MIN_HEAP_FREE		(1024 * 1024)			///< minimum free heap for receiving further packets from ethernet

#define ETH_MAC_US_TICK		1000000uL
#define STATUS_MASK			(BIT(5) | BIT(2))		///< auto negotioation complete and link is up

/**
 * The standard transmit descriptor for ethernet DMA
 */
struct tx_descriptor {
	union {
		__IO void		*buf1ap;	///< transmit buffer 1 address pointer
		__IO uint32_t	ttsl;		///< 32 LSB of the transmit time stamp after DMA is finished (if enabled)
		__IO uint32_t	tdes0;		///< transmit descriptor word 0
	};
	union {
		__IO void		*buf2ap;	///< transmit buffer 2 address pointer
		__IO uint32_t	ttsh;		///< 32 MSB of the transmit time stamp after DMA is finished (if enabled)
		__IO uint32_t	tdes1;		///< transmit descriptor word 1
	};
	union {
		struct {
			__IO uint32_t	bl1:14;		///< buffer length 1 or header length (if TSE is enabled)
			__IO uint32_t	vtir:2;		///< VLAN tag insertion or replacement
			__IO uint32_t	b2l:14;		///< buffer length 2
			__IO uint32_t	ttse:1;		///< transmit time stamp enable
			__IO uint32_t	ioc:1;		///< interrupt on completion
		};
		__IO uint32_t	tdes2;		///< transmit descriptor word 2
	};
	union {
		struct {	// meaning when handing this descriptor over to DMA (in manual: Read format)
			__IO uint32_t	fl_tpl:15;	///< frame length or TCP payload length
			__IO uint32_t	tpl:1;		///< another bit of TCP payload length if TSE bit is set
			__IO uint32_t	cic_tpl:2;	///< checksum insertion control or upper bits of TCP payload length if TSE bit is set
			__IO uint32_t	tse:1;		///< TCP segmentation enable (only in first descriptor - FD is set)
			__IO uint32_t	thl:4;		///< TCP header length if TSE bit is set
			__IO uint32_t	saic:3;		///< SA insertion control (topmost bit decides if MAC address0 or 1 is used)
			__IO uint32_t	cpc:2;		///< CRC/Pad control (only in first descriptor - FD is set)
			__IO uint32_t	ld:1;		///< last descriptor
			__IO uint32_t	fd:1;		///< first descriptor
			__IO uint32_t	ctxt:1;		///< context type descriptor if set to 1
			__IO uint32_t	own:1;		///< OWN bit (0 = CPU, 1 = DMA hardware)
		};
		struct {	// meaning when this descriptor is given back to CPU (in manual: write-back format)
			__IO uint32_t	ihe:1;		///< IP header error (only when checksum offload is used)
			__IO uint32_t	db:1;		///< deferred bit
			__IO uint32_t	uf:1;		///< underflow error
			__IO uint32_t	ed:1;		///< excessive deferral
			__IO uint32_t	cc:4;		///< collision count
			__IO uint32_t	ec:1;		///< excessive collision
			__IO uint32_t	lc:1;		///< late collision
			__IO uint32_t	nc:1;		///< no carrier
			__IO uint32_t	loc:1;		///< loss of carrier
			__IO uint32_t	pce:1;		///< payload checksum error (only when checksum offload is used)
			__IO uint32_t	ff:1;		///< packet flushed
			__IO uint32_t	jt:1;		///< jabber timeout
			__IO uint32_t	es:1;		///< error summary
			__IO uint32_t	:1;			///< RESERVED
			__IO uint32_t	tts:1;		///< TX timestamt status (only in last descriptor - LD is set)
			__IO uint32_t	:10;		///< RESERVED
			__IO uint32_t	:4;			///< last 4 (ld, fd, ctxt, own) bits are identical to the "read format" struct above
		};
		__IO uint32_t	tdes3;		///< transmit descriptor word 3
	};
};

/**
 * The standard receive descriptor for ethernet DMA
 */
struct rx_descriptor {
	union {
		__IO void		*buf1ap;	///< receive buffer 1 address pointer
		struct {	// meaning when this descriptor is given back to CPU (in manual: write-back format, only when RDES3:rs0v is set)
			__IO uint32_t	ovt:16;		///< outer VLAN tag
			__IO uint32_t	ivt:16;		///< inner VLAN tag
		};
		__IO uint32_t	rdes0;		///< receive descriptor word 0
	};
	union {
		struct {	// meaning when this descriptor is given back to CPU (in manual: write-back format, only for last descriptor - LD is set)
			__IO uint32_t	pt:3;		///< payload type (if receive checksum offload engine is used)
			__IO uint32_t	iphe:1;		///< IP header error (valid only if ipv4 or ipv6 is set)
			__IO uint32_t	ipv4:1;		///< IPv4 header present
			__IO uint32_t	ipv6:1;		///< IPv6 header present
			__IO uint32_t	ipcb:1;		///< IP checksum bypassed
			__IO uint32_t	ipce:1;		///< IP payload error
			__IO uint32_t	pmt:4;		///< PTP message type (PTP = Precision Time Protocol)
			__IO uint32_t	pft:1;		///< PTP packet type
			__IO uint32_t	pv:1;		///< PTP version (IEEE 1588 version 1 or version 2)
			__IO uint32_t	tsa:1;		///< Timestamp available
			__IO uint32_t	td:1;		///< Timestamp dropped
			__IO uint32_t	opc:16;		///< OAM Sub-Type code or MAC Control Packet code
		};
		__IO uint32_t	rdes1;		///< receive descriptor word 1 (reserved field for "read format")
	};
	union {
		__IO void		*buf2ap;	///< receive buffer 2 address pointer
		struct {
			__IO uint32_t	:10;		///< RESERVED
			__IO uint32_t	arpnr:1;	///< ARP reply not generated
			__IO uint32_t	:4;			///< RESERVED
			__IO uint32_t	vf:1;		///< VLAN filter status
			__IO uint32_t	saf:1;		///< Source Address filter fail
			__IO uint32_t	daf:1;		///< Destination Address filter fail
			__IO uint32_t	hf:1;		///< Hash filter status
			__IO uint32_t	madrm:8;	///< MAC address match or hash value
			__IO uint32_t	l3fm:1;		///< Layer 3 filter match
			__IO uint32_t	l4fm:1;		///< Layer 4 filter match
			__IO uint32_t	l3l4fm:3;	///< Layer 3 and Layer 4 filter number matched
		};
		__IO uint32_t	rdes2;		///< receive descriptor word 2
	};
	union {
		struct {
			__IO uint32_t	:24;		///< RESERVED
			__IO uint32_t	buf1v:1;	///< buffer 1 address valid
			__IO uint32_t	buf2v:1;	///< buffer 2 address valid
			__IO uint32_t	:4;			///< RESERVED
			__IO uint32_t	ioc:1;		///< interrupt enable on completion
			__IO uint32_t	own:1;		///< OWN bit (0 = CPU, 1 = DMA hardware)
		};
		struct {	// meaning when this descriptor is given back to CPU (in manual: write-back format
			__IO uint32_t	pl:15;		///< Packet length (only in last descriptor - LD is set)
			__IO uint32_t	es:1;		///< Error summary
			__IO uint32_t	lt:3;		///< Length / Type filed
			__IO uint32_t	de:1;		///< dribble bit error
			__IO uint32_t	re:1;		///< receive error
			__IO uint32_t	oe:1;		///< overflow error
			__IO uint32_t	rwt:1;		///< receive watchdog timeout
			__IO uint32_t	gp:1;		///< giant packet
			__IO uint32_t	ce:1;		///< CRC error
			__IO uint32_t	rs0v:1;		///< receive status RDES0 valid
			__IO uint32_t	rs1v:1;		///< receive status RDES1 valid
			__IO uint32_t	rs2v:1;		///< receive status RDES2 valid
			__IO uint32_t	ld:1;		///< last descriptor
			__IO uint32_t	fd:1;		///< first descriptor
			__IO uint32_t	ctxt:1;		///< receive context descriptor
			__IO uint32_t	:1;			///< last bit (own) is identical to the "read format" struct above
		};
		__IO uint32_t	rdes3;		///< receive descriptor word 3
	};
};

// @TODO Cache-Coherency beachten - Cache muss nach setzen der Descriptoren und Buffer geflusht werden!
static uint8_t __attribute__((aligned(32))) volatile rx_buffers[RX_DESCRIPTORS * RX_BUFFERSIZE];
static struct tx_descriptor __attribute__((aligned(8), section(".sram2"))) volatile txd[TX_DESCRIPTORS];
static struct rx_descriptor __attribute__((aligned(8), section(".sram2"))) volatile rxd[RX_DESCRIPTORS];

static volatile int rxidx;
//static struct iface *interface;

static struct pbuf *txpackets[TX_DESCRIPTORS];	// contains pointers to the packets that are in the transmit descriptors
static struct {
	struct tx_descriptor volatile * volatile bdphead;
	struct tx_descriptor volatile * volatile bdptail;
    struct pbuf ** volatile pb_head;
    struct pbuf ** volatile pb_tail;
} txbuf;

// forward declaration of the EMAC handler task and a handle for the created task
static void EMACDeferredInterruptHandlerTask (void *pvParameters);
static TaskHandle_t EMACtask;

static void eth_prepareBuffers (void)
{
	int i;

	memset((void *) txd, 0, sizeof(txd));
	memset((void *) rxd, 0, sizeof(rxd));
	memset(txpackets, 0, sizeof(txpackets));

	for (i = 0; i < RX_DESCRIPTORS; i++) {
		rxd[i].buf1ap = &rx_buffers[i * RX_BUFFERSIZE];
		rxd[i].buf1v = 1;
		rxd[i].ioc = 1;
		rxd[i].own = 1;
	}

	txbuf.bdphead = txbuf.bdptail = txd;
	txbuf.pb_head = txbuf.pb_tail = &txpackets[0];
	rxidx = 0;

	ETH->DMACTDRLR = TX_DESCRIPTORS - 1;
	ETH->DMACRDRLR = RX_DESCRIPTORS - 1;
	ETH->DMACRDLAR = (uint32_t) rxd;
	ETH->DMACRDTPR = (uint32_t) &rxd[RX_DESCRIPTORS - 1];	// we start with all but the last RX-Descriptor available as receiver buffers
	ETH->DMACTDLAR = (uint32_t) txd;
	ETH->DMACTDTPR = (uint32_t) &txd[0];			// we start with no TX-Descriptor to send
}

/**
 * Called when link is down
 */
static void eth_stop (void)
{
	NVIC_DisableIRQ(ETH_IRQn);						// enable ethernet interrupt in NVIC
	CLEAR_BIT (ETH->DMACTCR, ETH_DMACTCR_ST);		// stop transmit DMA
	CLEAR_BIT (ETH->MACCR, ETH_MACCR_RE);			// disable MAC receiver
	SET_BIT (ETH->MTLTQOMR, ETH_MTLTQOMR_FTQ);		// flush TX-FIFO
	CLEAR_BIT (ETH->MACCR, ETH_MACCR_TE);			// disable MAC transmitter

	printf("%s()\n", __func__);
}

/**
 * called when link is established
 */
void stm_ethStart (linkstate speed)
{
	uint32_t maccr;

	maccr = ETH->MACCR & ~(ETH_MACCR_FES | ETH_MACCR_DM);	// clear 100MBit/s + full duplex (i.e. set to 10MBit/s / half duplex)

	switch (speed) {
		case e10HDX:
			/* maccr already contains correct value */
			printf("%s() 10MBit/s HDX\n", __func__);
			break;
		case e10FDX:
			maccr |= ETH_MACCR_DM;
			printf("%s() 10MBit/s FDX\n", __func__);
			break;
		case e100HDX:
			maccr |= ETH_MACCR_FES;
			printf("%s() 100MBit/s HDX\n", __func__);
			break;
		case e100FDX:
			maccr |= ETH_MACCR_FES | ETH_MACCR_DM;
			printf("%s() 100MBit/s FDX\n", __func__);
			break;
		case eLINKDOWN:
			eth_stop();
			return;
	}

	ETH->MACCR = maccr;									// set speed and duplex mode
	SET_BIT (ETH->DMACTCR, ETH_DMACTCR_ST);				// start TX-DMA
	SET_BIT (ETH->MACCR, ETH_MACCR_RE | ETH_MACCR_TE);	// enable transmitter and receiver
    NVIC_ClearPendingIRQ(ETH_IRQn);						// clear a possibly pending interrupt request
    NVIC_SetPriority(ETH_IRQn, 10);
	NVIC_EnableIRQ(ETH_IRQn);							// enable ethernet interrupt in NVIC
}

static struct tx_descriptor volatile *stm_enetNextTxBdes (struct tx_descriptor volatile *bdp)
{
	struct tx_descriptor volatile *d;

	d = bdp + 1;
	if (d == &txd[TX_DESCRIPTORS]) d = txd;
	return d;
}

/**
 * Prepare a TX descriptor with the supplied packet buffer
 */
static struct tx_descriptor volatile *stm_enetTxBdes (struct tx_descriptor volatile *bdp, void *buf, int len)
{
	if (!bdp || bdp->own) return NULL;	    // this buffer is in use by hardware
	if (!buf || len <= 0) return NULL;	    // nothing to send
	cache_flush((uint32_t) buf, len);
	bdp->buf1ap = buf;				// TDES0: buffer address
	bdp->tdes1 = 0;					// TDES1: buffer2 address unused
	bdp->tdes2 = bdp->tdes3 = 0;	// TDES2/TDES3: initialize all flags to zero
	bdp->bl1 = len;

	return bdp;
}

/**
 * Handover the list of buffer descriptors from first to last
 * (inclusive) to the hardware.
 *
 * @param first	    first buffer descriptor in the list
 * @param last	    last buffer descriptor in the list
 */
static void stm_enetTxBd2hw (volatile struct tx_descriptor *first, volatile struct tx_descriptor *last)
{
	first->fd = 1;
	first->saic = 0b10;		// replace the source MAC address
	while (first != last) {
		first->own = 1;	// buffer now belongs to hardware
		first = stm_enetNextTxBdes(first);
	}
	last->ld = 1;
    last->ioc = 1;		// enable interrupt on completion
	last->own = 1;
}

static err_t stm_enetOutput (struct netif *netif, struct pbuf *p)
{
	volatile struct tx_descriptor *bdes;
	struct pbuf *q;

	(void) netif;

	if (!p) return ERR_OK;
	q = p;
#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

	/* put buffers to DMA memory */
	bdes = txbuf.bdphead;
	while (bdes && q) {
		bdes = stm_enetTxBdes(bdes, q->payload, q->len);
		if (bdes != NULL) {	// check if the buffer was really available
			if (q->len == q->tot_len) {		// this is the last part of the packet
				pbuf_ref(p);								// increment reference count to keep it available throughout the transmission process
				*txbuf.pb_head = p;							// remember the pbuf chain that was sent with this list of descriptors
				if (++txbuf.pb_head >= &txpackets[TX_DESCRIPTORS]) txbuf.pb_head = &txpackets[0];
				if (txbuf.pb_head == txbuf.pb_tail) {
					fprintf (stderr, "%s(): list of remembered PBUFs overflowed!\n", __func__);
				}
				stm_enetTxBd2hw(txbuf.bdphead, bdes);		// hand over the descriptors to hardware
			    txbuf.bdphead = stm_enetNextTxBdes(bdes);
			    ETH->DMACTDTPR = (uint32_t) txbuf.bdphead;	// set the new tail (triggers DMA transmitter to check for new packets)
				break;
			} else {
				bdes = stm_enetNextTxBdes(bdes);
			}
		} else {
			fprintf (stderr, "%s() WARNING: no buffer descriptor for packet to send\n", __FUNCTION__);
		}
		q = q->next;
	}

#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
	return ERR_OK;
}

#if 0	/* currently unused (and that will probably never change) */
static void stmenet_deinit (struct iface *iface)
{
    if (iface == interface) interface = NULL;
}
#endif

#define MAC_ADDR		0xFA

static void i2c_loadmac (uint8_t *mac)
{
	int rc;
	uint8_t *uid;

	rc = i2c_read(I2C4, MAC_EEPROM, MAC_ADDR, 1, mac, 6);
	switch (rc) {
		case 0:
			printf("%s(): MAC successfully read: %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
			return;
		case -1:
			fprintf(stderr, "%s(): TIMEOUT error\n", __func__);
			break;
		case -2:
			fprintf(stderr, "%s(): NACK received\n", __func__);
			break;
		case -3:
			fprintf(stderr, "%s(): wrong parameters in call to i2c_read()\n", __func__);
			break;
		default:
			fprintf(stderr, "%s(): unknown error returned (%d)\n", __func__, rc);
			break;
	}

	// MAC reading was unsuccessfull, so generate a dummy address
	mac[0] = 0x0A;		// 0b00001010 -> a locally administered address, unicast address
	mac[1] = 0x00;
	mac[2] = 0x27;
	uid = (uint8_t *) UID_BASE;
	mac[3] = *uid++;
	mac[4] = *uid++;
	mac[5] = *uid++;
}

/**
 * Called from IP stack to init the hardware and wait for a network connection.
 *
 * From the FreeRTOS-Plus-TCP source you can see, that an eNetworkDownEvent
 * triggers all actions for "Network Down" and then calls this function to set
 * up the network again. If this function returns success, it immedeately calls
 * the "Network Up" functions and starts working.
 *
 * This means, that, if everythings works well, we should block the IP-Task and
 * return not before we have established a physical connection to the network.
 *
 * @return  pdPASS if initialisation succeed, pdFALSE else
 */

err_t stmenet_init (struct netif *netif)
{
    if (netif != NULL) {
    	// start with an ethernet peripheral reset
    	ETH->DMAMR = ETH_DMAMR_SWR;

    	i2c_loadmac (netif->hwaddr);
    	printf("%s(): MAC %02x:%02x:%02x:%02x:%02x:%02x\n", __func__,
    			netif->hwaddr[0], netif->hwaddr[1], netif->hwaddr[2], netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);
		netif->mtu = 1500;
		netif->name[0] = 'e';
		netif->name[1] = 'n';
		netif->num = 0;
		netif->hwaddr_len = 6;
		netif->output = etharp_output;
		netif->linkoutput = stm_enetOutput;
		netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

		netif->link_callback = dbg_link_cb;
		netif->status_callback = dbg_status_cb;

		// create the task for the deferred interrupt handler
		if (EMACtask == NULL) {
			if (xTaskCreate (EMACDeferredInterruptHandlerTask, "EMAC", configMINIMAL_STACK_SIZE * 4,
				netif, 2, &EMACtask) != pdPASS) return ERR_MEM;
		}

	    // now wait for completion of ethernet internal reset
	    while (ETH->DMAMR & ETH_DMAMR_SWR) ;

	    // MAC LPU 1us Tick counter
	    ETH->MAC1USTCR = (HCLK_FREQ / ETH_MAC_US_TICK) - 1;

	    // configure the MAC part ot the ethernet
	    ETH->MACCR = ETH_MACCR_SARC_REPADDR0 | ETH_MACCR_IPG_96BIT | ETH_MACCR_FES | ETH_MACCR_DM | ETH_MACCR_BL_10 | ETH_MACCR_PRELEN_7;
	    ETH->MACECR = 0;
	    ETH->MACWTR = ETH_MACWTR_WTO_2KB;
	    ETH->MACTFCR = ETH_MACTFCR_TFE;
	    ETH->MACRFCR = 0;	// receive flow control disabled?? (-> ETH_MACRFCR_RFE)
		// set the station address
		ETH->MACA0HR = (netif->hwaddr[5] << 8) | (netif->hwaddr[4] << 0);
		ETH->MACA0LR = (netif->hwaddr[3] << 24) | (netif->hwaddr[2] << 16) | (netif->hwaddr[1] << 8) | (netif->hwaddr[0] << 0);

	    // configure the MTL part of the ethernet
	    MODIFY_REG(ETH->MTLTQOMR, ETH_MTLTQOMR_TTC, ETH_MTLTQOMR_TTC_128BITS | ETH_MTLTQOMR_FTQ | ETH_MTLTQOMR_TSF);
	    MODIFY_REG(ETH->MTLRQOMR, ETH_MTLRQOMR_RTC | ETH_MTLRQOMR_RSF, ETH_MTLRQOMR_RTC_64BITS);

	    // configure the DMA part of the ethernet
	    ETH->DMAMR = ETH_DMAMR_PR_1_1;
	    ETH->DMASBMR = ETH_DMASBMR_AAL | ETH_DMASBMR_FB;
	    ETH->DMACCR = (536 << ETH_DMACCR_MSS_Pos);		// not really needed, as TSE bit in DMACTCR is not set
	    ETH->DMACTCR = ETH_DMACTCR_TPBL_16PBL;
	    ETH->DMACRCR = ETH_DMACRCR_RPBL_16PBL | (RX_BUFFERSIZE << ETH_DMACRCR_RBSZ_Pos);

	    eth_prepareBuffers();
	    ksz8081_setup_phy(EMACtask);
	    SET_BIT (ETH->DMACRCR, ETH_DMACRCR_SR);

	    ETH->DMACIER = ETH_DMACIER_NIE | ETH_DMACIER_RIE | ETH_DMACIER_TIE;	// enable normal interrupts (RIE and TIE)
		ETH->MMCRIMR = 0;	// disable management counter interrupts (receive side)
		ETH->MMCTIMR = 0;	// disable management counter interrupts (transmit side)
		/* ETH->DMACCR can stay at reset value 0 */
    }

    return ERR_OK;
}

/* ============================================================================================ */
/* Interrupt handling																			*/
/* ============================================================================================ */

static void stm_enetCheckTx (void)
{
	struct tx_descriptor volatile *bdes;
	struct pbuf *pb = NULL;

	bdes = txbuf.bdptail;
	while (bdes != txbuf.bdphead && !bdes->own) {
		// only the first pbuf of a packet is remembered
		if (bdes->fd) {
			if (txbuf.pb_head != txbuf.pb_tail) {   // additional check: the list of PBUFs must not be empty
				pb = *txbuf.pb_tail;
			}
		}
		if (bdes->ld && pb) {
			pbuf_free(pb);
			if (++txbuf.pb_tail >= &txpackets[TX_DESCRIPTORS]) txbuf.pb_tail = &txpackets[0];
			pb = NULL;
		}
		bdes = stm_enetNextTxBdes(bdes);
		if (!pb) txbuf.bdptail = bdes;
	}
}

static bool stm_enetCheckRx (struct netif *netif)
{
    struct rx_descriptor volatile *bdes;
    int idx;
    struct pbuf *pb;
    size_t len, l;
    uint8_t *p;
    bool ld;

    // skip over orphaned descriptors until we find a cpu-owned descriptor with the FD (first descriptor) bit set
    bdes = &rxd[rxidx];
    while (!bdes->own && !bdes->fd) {
        fprintf (stderr, "%s() WARNING: first BufferDescriptor is owned by CPU but FD is not set\r\n", __func__);
        // put this buffer back to BufferManagement
        bdes->buf1ap = &rx_buffers[rxidx * RX_BUFFERSIZE];	// RDES0: buffer address
        bdes->rdes1 = 0;	// RDES1: reserved field - cleared
        bdes->rdes2 = 0;	// RDES2: buffer2 address - not used
        bdes->rdes3 = 0;	// RDES3: flags - start cleared
		bdes->buf1v = 1;	// Buffer1 address is valid
		bdes->ioc = 1;		// set interrupt on completion
        bdes->own = 1;		// ... and hand over to hardware
        ETH->DMACRDTPR = (uint32_t) bdes;
        if (++rxidx >= RX_DESCRIPTORS) rxidx = 0;
        bdes = &rxd[rxidx];
    }
    if (bdes->own) return false;    // no buffers to deal with

    // first run: check, that the LD (Last-Descriptor) bit can be found in any of the descriptors that are owned by CPU
    idx = rxidx;
    bdes = &rxd[idx];
    while (!bdes->own && !bdes->ld) {
    	if (++idx >= RX_DESCRIPTORS) idx = 0;
        bdes = &rxd[idx];
        if (idx == rxidx) {
            fprintf (stderr, "%s() FATAL: All buffers owned by CPU and no LD found\r\n", __func__);
            // TODO: desaster recovery: return all descriptors to hardware
            return false;
        }
    }
    if (bdes->own || !bdes->ld) return false;   // buffer not yet complete - just don't touch anything!
    if (bdes->ce) printf ("%s(): CRC-Error\n", __func__);
    if (bdes->gp) printf ("%s(): Giant Packet\n", __func__);
    if (bdes->rwt) printf ("%s(): Watchdog-timeout\n", __func__);
    if (bdes->oe) printf ("%s(): Overflow-Error\n", __func__);
    if (bdes->re) printf ("%s(): Receive-Error\n", __func__);
    if (bdes->de) printf ("%s(): DribbleBit-Error\n", __func__);

    // Here we must take the BufferDescriptors from rxidx to idx (including)
    // and form a pbuf from it. The chain contains only one block of
    // memory allocated from the heap.
    // The buffer in the buffer descriptor can directly be reused for next
    // reception, so no need for a dynamic buffer pool.
    // Finally increment rxbidx to the descriptor following the bdes containing EOP.
    len = bdes->pl;		// the packet length is written to the last descriptor (having the LD flag set)

	// Here we must take the BufferDescriptors from rxbdp to bdes (including)
	// and form pbufs from it.
	// The buffer in the buffer descriptor can directly be reused for next
	// reception, so no need for a dynamic buffer pool.
	// Finally increment rxbdp to the descriptor following the bdes containing EOP.
	p = NULL;   // PRESET: don't copy received frame but hand the buffers back to hardware again
	pb = NULL;
    if (!bdes->es) {	// allocate a packet only if no error occured (ES bit = Error summary)
#if ETH_PAD_SIZE
		if ((pb = pbuf_alloc(PBUF_RAW, len + ETH_PAD_SIZE, PBUF_RAM)) != NULL) {
			pbuf_header(pb, -ETH_PAD_SIZE); /* drop the padding word */
#else
			if ((pb = pbuf_alloc(PBUF_RAW, len, PBUF_RAM)) != NULL) {
#endif
			p = pb->payload;
		} else {
			fprintf (stderr, "%s() cannot allocate a network buffer with descriptor (discarding frame)\n", __func__);
		}
	}

    idx = rxidx;
    bdes = &rxd[idx];	// start from first descriptor
    do {
        ld = bdes->ld;        // remember LD bit
        if (p) {
        	cache_invalidate((uint32_t) bdes->buf1ap, RX_BUFFERSIZE);
        	l = (len > RX_BUFFERSIZE) ? RX_BUFFERSIZE : len;
            memcpy (p, (void *) bdes->buf1ap, l);
            p += l;
            len -= l;
        }
        bdes->buf1ap = &rx_buffers[idx * RX_BUFFERSIZE];	// RDES0: Buffer1 address
        bdes->rdes1 = 0;		// RDES1: reserved - cleared
        bdes->rdes2 = 0;		// RDES2: Buffer2 address - not used
        bdes->rdes3 = 0;		// RDES3: flags field - start with cleared value
		bdes->buf1v = 1;		// Buffer1 address is valid
		bdes->ioc = 1;			// set interrupt on completion
		bdes->own = 1;			// mark buffer belonging to HW
		if (!ld) {
			if (++idx >= RX_DESCRIPTORS) idx = 0;
			bdes = &rxd[idx];
		}
    } while (!ld);
    ETH->DMACRDTPR = (uint32_t) &rxd[idx];		// the last desciptor of this packet marks the end of the list
    if ((rxidx = ++idx) >= RX_DESCRIPTORS) rxidx = 0;

	if (pb) {
#if ETH_PAD_SIZE
		pbuf_header(pb, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
		if (netif->input(pb, netif) != ERR_OK) {
			fprintf (stderr, "%s(): could not post packet to TCPIP thread\n", __func__);
			pbuf_free (pb);
		}
	}
    return true;
}

/**
 * The delayed interrupt handling task.
 *
 * The task sits there until it is unblocked from an ISR by vTaskNotifyGiveFromISR()
 * whenever the EMAC needs handling (frames received or transmitted).
 *
 * A third action is to monitor the cable linkstate and set the link up and down
 * whenever the cable is (dis-)connected.
 *
 * @param pvParameters	the network interface
 */
static void EMACDeferredInterruptHandlerTask (void *pvParameters)
{
	linkstate phystat, last_phystat = eLINKDOWN;
	int lost_packets, rx_count;

	struct netif *netif = (struct netif *) pvParameters;

	ETHLED_OFF();							// initial state

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//	printf ("woke up\n");
		phystat = ksz8081_getstate();

		if (ksz8081_isup(last_phystat) && !ksz8081_isup(phystat)) {
			ETHLED_OFF();
			//	    LOCK_TCPIP_CORE();
			netifapi_netif_set_link_down (netif);
			//	    UNLOCK_TCPIP_CORE();
			eth_stop();
			ksz8081_autonegotiation ();
			//	    debug ("Network down\n");
		} else if (!ksz8081_isup(last_phystat) && ksz8081_isup(phystat)) {
			ETHLED_ON();
			stm_ethStart(phystat);
			//	    LOCK_TCPIP_CORE();
			netifapi_netif_set_link_up (netif);
			//	    UNLOCK_TCPIP_CORE();
			//	    debug ("Network up\n");
		}
		last_phystat = phystat;

		stm_enetCheckTx();		// free network buffers that are done
		rx_count = 0;
		while (stm_enetCheckRx(netif)) {	// process all received frames (one call will receive only one single packet)
			if (++rx_count >= 4) {		// after 4 contigously received frames we give other tasks a chance
				taskYIELD();
				stm_enetCheckTx();		// free network buffers that are done
				rx_count = 0;
			}
			if (xPortGetFreeHeapSize() < MIN_HEAP_FREE) break;		// temporary stop receive frames, when memory is low
		}
		if ((lost_packets = ETH->DMACMFCR) != 0) {
			fprintf (stderr, "%s(): Lost %d packets%s\n", __func__, lost_packets & 0x3FF, (lost_packets & 0x8000) ? " (OVERFLOWED)" : "");
		}
	}
}

void ETH_IRQHandler (void)
{
	BaseType_t xHigherPriorityTaskWoken = 0;
	uint32_t status, macst, mtlst;

	status = ETH->DMAISR;			// read status (interrupt) information
	if (status & ETH_DMAISR_MACIS) {	// handle MAC status interrupt
		macst = ETH->MACISR;
		(void) macst;
		ETH->MACIER = 0;				// just disable those MAC interrupts
	}
	if (status & ETH_DMAISR_MTLIS) {	// handle MTL status interrupt
		mtlst = ETH->MTLISR;
		(void) mtlst;
		ETH->MTLQICSR = ETH_MTLQICSR_RXOVFIS | ETH_MTLQICSR_TXUNFIS;	// disable these interrupts and clear the status flags
	}
	if (status & ETH_DMAISR_DMACIS) {	// handle DMA status interrupt
		if (ETH->DMACSR & ETH_DMACSR_RI) {
			ETH->DMACSR |= ETH_DMACSR_NIS | ETH_DMACSR_RI;
		}
		if (ETH->DMACSR & ETH_DMACSR_TI) {
			ETH->DMACSR |= ETH_DMACSR_NIS | ETH_DMACSR_TI;
		}
	}

	vTaskNotifyGiveFromISR (EMACtask, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
