/**
 * @file spi.c
 *
 * @author Andi
 * @date   16.04.2020
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

/*
 * ATTENTION! Hardware-Bug in STM32H7 (erratum (Rev 7) 2.14.2):
 *
 * 2.12.2 Master data transfer stall at system clock much faster than SCK
 * With the system clock (spi_pclk) substantially faster than SCK (spi_ker_ck divided by a
 * prescaler), SPI/I2S master data transfer can stall upon setting the CSTART bit within one
 * SCK cycle after the EOT event (EOT flag raise) signaling the end of the previous transfer.
 *
 * There are three possible workarounds documented. As we are already in the need of a short
 * delay between block transfers, we take this way using an additional timer (TIM13) that
 * kicks the next transaction with a short delay after the previous transaction is done setting
 * the EOT-Bit in the status Register (SPI6->SR). Pay attention to the fact, that the interrupt
 * is shared with the TIM8-Update interrupt (see signal.c TIM8_UP_TIM13_IRQHandler()).
 */

#include <stdio.h>
#include "rb2.h"
#include "easynet.h"

#define DATA_PACKET_SIZE		4
#define INTERBLOCK_PAUSE		50			///< a delay in between blocks of BUS_BLOCKLEN bytes in µs
#define TXQUEUE_LEN				1
#define RXQUEUE_LEN				16

// defines for bootloader mode
#define BOOTMODE_QUEUELEN		64			///< for both RX and TX - we use a character queue
#define BOOTMODE_FILLER			0xFEFEFEFE	///< a value transmitted if nothing else is to transmit. this byte cannot start a block so it will be ignored

#ifdef EASYNET_USE_SPI1
#define ENSPI					SPI1
#define ENSPI_IRQn				SPI1_IRQn
#else
#define ENSPI					SPI6
#define ENSPI_IRQn				SPI6_IRQn
#endif

static volatile QueueHandle_t rxqueue;
static volatile QueueHandle_t txqueue;
static volatile QueueHandle_t bootrx;		// character queue for boot mode reception
static volatile int rxidx;
static bool boot;

/**
 * Setup SPI1 (or SPI6) for EasyNet. SPI kernel clock is from PLL1-Q (or SPI6: PLL2-Q)
 * output at 8MHz. We want a bitrate of 62,5kBit/s (switchable to 125kBit/s or 256kBit/s
 * on request).
 *
 * For normal operation (not Bootloader mode), we use LSB first, Mode 1 (CPOL=0, CPHA=1).
 * always use 8 bit / data item, CRC mode disabled.
 *
 * For inter-packet gaps we use TIM13 and init it here too.
 *
 * \param bootmode	if set, we will update an EasyNet client node - use MSB-first-mode
 * \return			0 if initialisation is OK, an errorcode otherwise
 */
int spi_init (bool bootmode)
{
	volatile QueueHandle_t qtmp;
	TickType_t to;

//	log_msg (LOG_INFO, "%s() begin\n", __func__);
	NVIC_DisableIRQ(ENSPI_IRQn);
	CLEAR_BIT (ENSPI->IER, SPI_IER_EOTIE | SPI_IER_TXPIE | SPI_IER_RXPIE);
	TIM13->CR1 = 0;		// disable timer
	if ((ENSPI->CR1 & (SPI_CR1_CSTART | SPI_CR1_SPE)) == (SPI_CR1_CSTART | SPI_CR1_SPE)) {
//		log_msg (LOG_INFO, "%s() suspending SPI\n", __func__);
		to = tim_timeout(30);
		SET_BIT (ENSPI->CR1, SPI_CR1_CSUSP);
		while (!tim_isover(to) && !(ENSPI->SR & SPI_SR_SUSP)) {
			ENSPI->TXDR = 0;
			vTaskDelay(2);
		}
		ENSPI->IFCR = SPI_IFCR_SUSPC;
//		if (!(ENSPI->SR & SPI_SR_SUSP)) {
//			log_msg (LOG_INFO, "%s() SPI suspending times out\n", __func__);
//		} else {
//			log_msg (LOG_INFO, "%s() SPI suspended\n", __func__);
//		}
	}
	vTaskDelay(50);
	ENSPI->CR1 = 0;				// switch off SPI completely
	ENSPI->IFCR = 0xFF8;		// clear all clearable status flags
//	log_msg (LOG_INFO, "%s() SOFT-Resetting SPI1 peripheral\n", __func__);
	SET_BIT (RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
	vTaskDelay(5);
	CLEAR_BIT (RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);

//	log_msg (LOG_INFO, "%s() check queues\n", __func__);
	if (rxqueue) { qtmp = rxqueue; rxqueue = NULL; vQueueDelete(qtmp); }
	if (txqueue) { qtmp = txqueue; txqueue = NULL; vQueueDelete(qtmp); }
	if (bootrx) { qtmp = bootrx; bootrx = NULL; vQueueDelete(qtmp); }
	if (bootmode) {
		bootrx  = xQueueCreate(BOOTMODE_QUEUELEN, sizeof(uint8_t));
		if (!bootrx) {
			fprintf (stderr, "%s(): could not allocate the queue\n", __func__);
			return -1;
		}
	} else {
		rxqueue = xQueueCreate(RXQUEUE_LEN, BUS_BLOCKLEN);
		txqueue = xQueueCreate(TXQUEUE_LEN, BUS_BLOCKLEN);
		if (!rxqueue || !txqueue) {
			fprintf (stderr, "%s(): could not allocate the queues\n", __func__);
			return -1;
		}
	}

//	log_msg (LOG_INFO, "%s() configure SPI (1)\n", __func__);
	// use ker_ck / 128 (-> 62,5kBit/s), 8 "frames" FIFO threshold, 8bit frames
	ENSPI->CFG1 = (0b110 << SPI_CFG1_MBR_Pos) | (0b00111 << SPI_CFG1_CRCSIZE_Pos) | (0b0111 << SPI_CFG1_FTHLV_Pos) | (0b00111 << SPI_CFG1_DSIZE_Pos);
	if (bootmode) {
		// /SS is HW driven, CPOL=0, CPHA=1, MSB first, Master mode, 5 SPI-Clocks idle between frames (bytes), 5 SPI-Clocks between assertion if /SS and first data bit
		ENSPI->CFG2 = SPI_CFG2_AFCNTR | SPI_CFG2_SSOM | SPI_CFG2_SSOE | SPI_CFG2_CPHA | SPI_CFG2_MASTER
				| (5 << SPI_CFG2_MIDI_Pos) | (5 << SPI_CFG2_MSSI_Pos);
		ENSPI->CR2 = 0;				// we always transfer as stream
	} else {
		// /SS is HW driven, CPOL=0, CPHA=1, LSB first, Master mode, 2 SPI-Clocks idle between frames (bytes), 5 SPI-Clocks between assertion if /SS and first data bit
		ENSPI->CFG2 = SPI_CFG2_AFCNTR | SPI_CFG2_SSOM  | SPI_CFG2_SSOE | SPI_CFG2_CPHA | SPI_CFG2_LSBFRST | SPI_CFG2_MASTER | (2 << SPI_CFG2_MIDI_Pos) | (5 << SPI_CFG2_MSSI_Pos);
		ENSPI->CR2 = BUS_BLOCKLEN;	// must not be changed while SPI is enabled!
	}

//	log_msg (LOG_INFO, "%s() configure SPI (2)\n", __func__);
    ENSPI->IER = 0;					// start with disabled interrupts
	ENSPI->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;		// fix /SS input to inactive and enable the peripheral

//	log_msg (LOG_INFO, "%s() configure TIM13\n", __func__);
	TIM13->CR1 = TIM_CR1_URS;			// Update-Interrupt only by update event
	TIM13->DIER = 0;					// disable all interrupts
	TIM13->SR = 0;						// clear all interrupt status flags
	TIM13->CCMR1 = 0;					// we don't use input capture or output compare
	TIM13->CCER = 0;					// we don't use input capture or output compare
	TIM13->PSC = 199;					// prescaler = 100 (200MHz kernel clock / 200 => 1MHz count rate, i.e. one tick is 1µs)
	TIM13->ARR = INTERBLOCK_PAUSE - 1;	// specify the timing in µs
	TIM13->CR1 |= TIM_CR1_CEN;			// let the timer run (no interrupts are allowed by now)

	boot = bootmode;	// remember bootmode setting for interrupt

//	log_msg (LOG_INFO, "%s() configure NVIC\n", __func__);
	NVIC_SetPriority(ENSPI_IRQn, 15);
	NVIC_ClearPendingIRQ(ENSPI_IRQn);
	NVIC_EnableIRQ(ENSPI_IRQn);

#ifdef HW_REV07		// for the old hardware we must initialise the NVIC here
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 14);		// use a rather low priority
	NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
#endif
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

//	log_msg (LOG_INFO, "%s() SR=0x%08lx CR1=0x%08lx CR2=0x%08lx CFG1=0x%08lx CFG2=0x%08lx IER=0x%08lx\n", __func__,
//			ENSPI->SR, ENSPI->CR1, ENSPI->CR2, ENSPI->CFG1, ENSPI->CFG2, ENSPI->IER);
//	log_msg (LOG_INFO, "%s() start data flow\n", __func__);
	if (bootmode) {
	    ENSPI->IER = /* SPI_IER_EOTIE | */ SPI_IER_TXPIE | SPI_IER_RXPIE;
//	    ENSPI->TXDR = BOOTMODE_FILLER;
		ENSPI->CR1 |= SPI_CR1_CSTART;
	} else {
		rxidx = 0;
	    ENSPI->CR1 |= SPI_CR1_CSTART;
	    while (!(ENSPI->SR & SPI_SR_TXTF)) ENSPI->TXDR = 0;		// send a block of nulls
	    ENSPI->IER = SPI_IER_EOTIE | SPI_IER_RXPIE;
	}

	log_msg (LOG_INFO, "%s() OK for %s SR=0x%08lx\n", __func__, (boot) ? "BOOTMODE" : "standard mode", ENSPI->SR);
    return 0;
}

bool spi_getblock (en_block *blk)
{
	if (!blk || !rxqueue) return false;

	return (xQueueReceive(rxqueue, blk, 0));
}

void spi_sendblock (en_block *blk)
{
	if (!txqueue) return;

	if (!xQueueSend(txqueue, blk, 2000)) {
//		fprintf (stderr, "%s(): could not queue up a block\n", __func__);
		fprintf (stderr, "%s(): TIM13: UIE %s ENSPI: TXTFIE %s, TXPIE %s, EOTIE %s, CT-Size=%ld, SR=0x%04lx\n", __func__, (TIM13->DIER & TIM_DIER_UIE) ? "ON" : "OFF",
				(ENSPI->IER & SPI_IER_TXTFIE) ? "ON" : "OFF", (ENSPI->IER & SPI_IER_TXPIE) ? "ON" : "OFF", (ENSPI->IER & SPI_IER_EOTIE) ? "ON" : "OFF", ENSPI->SR >> 16, ENSPI->SR & 0xFFFF);
		SET_BIT(ENSPI->IER, SPI_IER_TXTFIE);
	}
}

int spi_getchar (void)
{
	uint8_t c;

	if (xQueueReceive(bootrx, &c, 0) != pdTRUE) {
		vTaskDelay(2);
		return EOF;
	}
	return c;
}

volatile uint32_t *txdata;
volatile int txlen;

int spi_write (uint32_t *data, int len)
{
	txlen = 0;
	txdata = data;
	txlen = len;
	xQueueReset(bootrx);
	return 0;
}

/**
 * Called from interrupt handler of TIM8 + TIM13 located in Track/signal.c
 */
void tim13_updateIRQ (void)
{
	TIM13->DIER &= ~TIM_DIER_UIE;	// disable timer interrupt
	if (boot) {
		SET_BIT(ENSPI->IER, SPI_IER_TXPIE | SPI_IER_EOTIE);
		ENSPI->CR1 |= SPI_CR1_CSTART;
		irqdbg_printf("%s() re-enable SPI-IRQ\n", __func__);
	} else {
		ENSPI->IER |= SPI_IER_TXTFIE | SPI_IER_TXPIE | SPI_IER_EOTIE;	// enable the TX-FIFO threshold interrupt (this will fetch the next block from the queue, if any
	}
}

static void spi_irqBootRx (uint8_t c, BaseType_t *pxHigherPriorityTaskWoken)
{
	if (bootrx && c != 0x00 && c != 0xFF) xQueueSendFromISR(bootrx, &c, pxHigherPriorityTaskWoken);
}

/**
 * Bootmode IRQ handling (character based)
 */
static void spi_irqBoot (BaseType_t *pxHigherPriorityTaskWoken)
{
	uint32_t dataword;

//	irqdbg_printf("%s() SR=0x%08lx IER=0x%08lx CR1=0x%08lx CR2=0x%08lx\n", __func__, ENSPI->SR, ENSPI->IER, ENSPI->CR1, ENSPI->CR2);
	while ((ENSPI->SR & SPI_SR_RXP) || (ENSPI->SR & SPI_SR_RXWNE)) {
		dataword = ENSPI->RXDR;
		if (txlen <= 0 || !txdata) {	// ignore everything that is received during a blocktransfer
			spi_irqBootRx((dataword >> 0) & 0xFF, pxHigherPriorityTaskWoken);
			spi_irqBootRx((dataword >> 8) & 0xFF, pxHigherPriorityTaskWoken);
			spi_irqBootRx((dataword >> 16) & 0xFF, pxHigherPriorityTaskWoken);
			spi_irqBootRx((dataword >> 24) & 0xFF, pxHigherPriorityTaskWoken);
		}
	}

	while ((ENSPI->IER & SPI_IER_TXPIE) && (ENSPI->SR & SPI_SR_TXP) && !(ENSPI->SR & SPI_SR_TXTF)) {
		if ((txlen > 0) && txdata) {
			dataword = *txdata++;
			txlen -= sizeof(*txdata);
		} else {
			dataword = BOOTMODE_FILLER;
		}
		ENSPI->TXDR = dataword;
		ENSPI->CR1 |= SPI_CR1_CSTART;
	}
}

/**
 * Operation mode IRQ handling (block based)
 */
static void spi_irqBlock (BaseType_t *pxHigherPriorityTaskWoken)
{
	static en_block rxblock;
	static en_block txblock;
	static uint32_t *txp = NULL;

	uint32_t dataword;

#if 0
	if (ENSPI->SR & SPI_SR_RXP) {		// we have a packet size of 8 bytes
		rxblock.words[0] = ENSPI->RXDR;
		rxblock.words[1] = ENSPI->RXDR;
	}
	if ((ENSPI->SR & SPI_SR_EOT) && !(ENSPI->SR & SPI_SR_RXWNE) && (ENSPI->SR & SPI_SR_RXPLVL) > 0) {	// some bytes are leftovers
		dataword = ENSPI->RXDR;
		rxblock.crc = dataword & 0xFF;
		if (rxqueue != NULL && bus_chkblock(&rxblock)) {
			xQueueSendFromISR(rxqueue, &rxblock, pxHigherPriorityTaskWoken);
		}
	}
#else
	while (ENSPI->SR & SPI_SR_RXP) {
		if (ENSPI->SR & SPI_SR_RXWNE) {		// a full word can be read
			dataword = ENSPI->RXDR;
			if (rxidx == 0) {					// this is the first word from RxFIFO, it contains the last byte of the previous block and three new bytes
				rxblock.crc = dataword & 0xFF;
				if (rxqueue != NULL && bus_chkblock(&rxblock)) {
					xQueueSendFromISR(rxqueue, &rxblock, pxHigherPriorityTaskWoken);
				}
				rxblock.words[0] = dataword >> 8;
				rxidx++;
			} else if (rxidx == 1) {							// this is the second word from RxFIFO, it contains 4 more bytes for the current block
				rxblock.words[0] |= dataword << 24;
				rxblock.words[1] = dataword >> 8;
				rxidx++;
			} else {
				rxblock.words[1] |= dataword << 24;
				rxidx = 0;
			}
		}
	}
#endif

	// A full message transfer was completed, we can fetch the next message from the queue
	if ((ENSPI->IER & SPI_IER_TXTFIE) && (ENSPI->SR & SPI_SR_TXTF)) {
		ENSPI->IER &= ~SPI_IER_TXTFIE;	// disable this interrupt
		if (!txqueue || !xQueueReceiveFromISR(txqueue, &txblock, pxHigherPriorityTaskWoken)) {	// we have no new block to send, so send a null block
			txblock.words[0] = txblock.words[1] = 0;
			txblock.crc = 0;
		}
		ENSPI->IER |= SPI_IER_TXPIE | SPI_IER_EOTIE;
		ENSPI->CR1 |= SPI_CR1_CSTART;
		txp = txblock.words;
		ENSPI->IFCR = SPI_IFCR_TXTFC;	// acknowledge the interrupt
	}

	while ((ENSPI->IER & SPI_IER_TXPIE) && (ENSPI->SR & SPI_SR_TXP)) {
		if (txp) ENSPI->TXDR = *txp++;
	}

	if ((ENSPI->IER & SPI_IER_EOTIE) && (ENSPI->SR & SPI_SR_EOT)) {
//		ENSPI->IER &= ~SPI_IER_EOTIE;
		TIM13->EGR = TIM_EGR_UG;		// re-initialize the counter and prescaler
		ENSPI->IFCR = SPI_IFCR_EOTC;
		TIM13->SR &= ~TIM_SR_UIF;		// clear old update-interrupt status
		TIM13->DIER |= TIM_DIER_UIE;	// enable timer interrupt
	}
}

#ifdef EASYNET_USE_SPI1
void SPI1_IRQHandler (void)
#else
void SPI6_IRQHandler (void)
#endif
{
	BaseType_t xHigherPriorityTaskWoken = 0;

	if (boot) spi_irqBoot(&xHigherPriorityTaskWoken);
	else spi_irqBlock(&xHigherPriorityTaskWoken);

	NVIC_ClearPendingIRQ (ENSPI_IRQn);
	portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
