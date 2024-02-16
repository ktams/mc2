/**
 * @file esp01.c
 *
 * @author Andi
 * @date   07.04.2020
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
#include <stdlib.h>
#include <string.h>
#include "rb2.h"
#include "yaffsfs.h"
#include "esp.h"

#define USART_BASE_CLOCK	100000000u				///< the base clock after the prescaler (DIV-1 -> 100MHz)
#define BAUDRATE			230400
//#define BAUDRATE			74880

static struct bootpacket rx;
static struct bootpacket tx;
static volatile bool do_update;
static enum slipstat rxst;
static enum slipstat txst;

struct ringbuffer {
	volatile int	idxin;
	volatile int	idxout;
	int				len;
	uint8_t			buffer[];
};

static TaskHandle_t task;
static struct ringbuffer *rxbuf;

static struct ringbuffer *esp_allocbuffer (int size)
{
	struct ringbuffer *rb;

	if ((rb = malloc (sizeof(struct ringbuffer) + size)) == NULL) return NULL;
	rb->len = size;
	rb->idxin = rb->idxout = 0;
	return rb;
}

static void esp_baudrate (int baud)
{
	int brr;

	brr = (USART_BASE_CLOCK + baud / 2) / baud;
	USART6->BRR = brr;

	return;
}

static void esp_inituart (int baud)
{
	uint32_t cr1, cr3;

	USART6->CR1 = 0;		// disable USART6
//	USART6->CR2 = USART_CR2_SWAP;							// use 1 stop bit and SWAP Rx/Tx (needed for HW1.0)
	USART6->CR2 = 0;										// use 1 stop bit

	cr1 = USART_CR1_FIFOEN;									// enable FIFO mode
	cr1 |= USART_CR1_TE | USART_CR1_RE;						// set 8 bits of data, enable transmitter and receiver
	USART6->CR1 = cr1;

	cr3 = (0b001 << USART_CR3_RXFTCFG_Pos);					// set RX-FIFO threshold interrupt at quarter full
	cr3 |= (0b001 << USART_CR3_TXFTCFG_Pos);				// set TX-FIFO threshold interrupt at quarter full
	USART6->CR3 = cr3;

	USART6->PRESC = 0b0000;									// prescaler = 1 -> 100MHz kernel clock
	USART6->RTOR = 100;

	esp_baudrate(baud);

	NVIC_SetPriority(USART6_IRQn, 8);
	NVIC_ClearPendingIRQ(USART6_IRQn);
	NVIC_EnableIRQ(USART6_IRQn);
	USART6->ICR = 0xFFFFFFFF;								// clear all interrupt flags

	SET_BIT (USART6->CR1, USART_CR1_UE);					// enable the UART
	SET_BIT (USART6->CR3, USART_CR3_RXFTIE);				// enable RX FIFO threshold interrupt
}

static uint32_t esp_checksum (uint8_t *data, int len)
{
	uint32_t sum = 0xEF;	// start value

	while (len > 0) {
		sum ^= *data++;
		len--;
	}
	return sum;
}

static int esp_transfer (uint32_t tout)
{
	int rc, retry;

	retry = 0;
	while (retry < 5) {
		txst = rxst = SLIP_IDLE;
		SET_BIT (USART6->CR3, USART_CR3_TXFTIE);			// enable TX FIFO threshold interrupt
		if ((rc = ulTaskNotifyTake(pdTRUE, tout)) > 0) {
			if (rx.data[rx.size - 2]) {
				log_msg (LOG_INFO, "%s(): ERROR from ESP-01 CMD=0x%02X len=%d ERRCode 0x%02x\n", __func__, rx.cmd, rx.size, rx.data[rx.size - 1]);
			} else {
				log_msg (LOG_INFO, "%s(): SUCCESS from ESP-01 CMD=0x%02X len=%d\n", __func__, rx.cmd, rx.size);
			}
			if (rx.cmd == tx.cmd && rx.data[rx.size - 2] == 0) return 0;
			if (tx.cmd == ESPBL_FLASH_END && rx.data[rx.size - 1] == 0x06) return -1;	// failed but won't react to retries!
		} else {
			log_msg (LOG_WARNING, "%s(): no answer from ESP-01\n", __func__);
		}
		retry++;
	}
	return -1;
}

static int esp_sendCmdBlock (uint8_t cmd, void *data, int len, uint32_t tout)
{
	tx.dir = 0;
	tx.cmd = cmd;
	tx.size = len;
	tx.chk = 0;			// checksum only used with data packets
	memcpy (tx.data, data, len);
	return esp_transfer(tout);
}

/**
 * Send a flash data block. Length is always fixed to BLOCKSIZE_FLASH bytes.
 */
static int esp_sendDataBlock (void *data, int seq)
{
	uint32_t *words;

	tx.dir = 0;
	tx.cmd = ESPBL_FLASH_DATA;
	tx.size = BLOCKSIZE_FLASH + 4 * sizeof(uint32_t);
	tx.chk = esp_checksum(data, BLOCKSIZE_FLASH);
	words = (uint32_t *) &tx.data;
	words[0] = BLOCKSIZE_FLASH;
	words[1] = seq;
	words[2] = words[3] = 0;
	memcpy (&words[4], data, BLOCKSIZE_FLASH);
	return esp_transfer(1000);
}

static int esp_sendSync (void)
{
	tx.dir = 0;				// always: direction to module
	tx.cmd = ESPBL_SYNC;
	tx.size = 36;			// fixed size: 0x07, 0x07, 0x12, 0x20 + 32x 0x55
	tx.chk = 0;				// checksum unused for this block
	tx.data[0] = tx.data[1] = 0x07;
	tx.data[2] = 0x12;
	tx.data[3] = 0x20;
	memset (&tx.data[4], 0x55, 32);
	return esp_transfer(200);
}

static void esp_resetBootloader (void)
{
	do {
		log_msg (LOG_INFO, "%s() sync to Bootloader\n", __func__);
		ESP_GP0_LOW();
		ESP_RST_ON();
		vTaskDelay(50);
		ESP_RST_OFF();
		vTaskDelay(200);	// give the module time to launch the bootloader
		ESP_GP0_HIGH();
		while (ulTaskNotifyTake(pdTRUE, 100)) /* wait for silence after bootloader debug output */;
	} while (esp_sendSync() != 0);
}

static void esp_resetApplication (void)
{
	ESP_GP0_HIGH();
	ESP_RST_ON();
	rxst = txst = SLIP_STARTUP;
	vTaskDelay(50);
	ESP_RST_OFF();
}

/**
 * From the manual:
 *		SPIEraseArea function in the esp8266 ROM has a bug which causes extra area to be erased.
 *		If the address range to be erased crosses the block boundary,
 *		then extra head_sector_count sectors are erased.
 *		If the address range doesn't cross the block boundary,
 *		then extra total_sector_count sectors are erased.
 * Therefore, we work around this bug as the esptool does (a python program).
 *
 * Personal opinion: if the start address doesn't fall onto a sector boundary, the
 * calculation still is faulty! The total_sector_count should then be increased by 1!
 */
static uint32_t esp_calcEraseSize (uint32_t addr, uint32_t size)
{
	const uint32_t first_sector_index = addr / SECTOR_SIZE;							// the sector that contains the start address
	const uint32_t total_sector_count = (size + SECTOR_SIZE - 1) / SECTOR_SIZE;		// round up to complete sectors
	const uint32_t max_head_sector_count = SECTORS_PER_BLOCK - (first_sector_index % SECTORS_PER_BLOCK);
	const uint32_t head_sector_count = (max_head_sector_count > total_sector_count) ? total_sector_count : max_head_sector_count;
//	const int adjusted_sector_count = (total_sector_count < (2 * head_sector_count))
//		?  (total_sector_count + 1) / 2 : (total_sector_count - head_sector_count);
	const int adjusted_sector_count = (total_sector_count < (2 * head_sector_count))
		?  (total_sector_count + 1) / 2 : total_sector_count;
	log_msg (LOG_INFO, "%s() first_sector_index=%lu total_sector_count=%lu\n", __func__, first_sector_index, total_sector_count);
	log_msg (LOG_INFO, "%s() max_head_sector_count=%lu head_sector_count=%lu\n", __func__, max_head_sector_count, head_sector_count);
	log_msg (LOG_INFO, "%s() adjusted_sector_count=%d total size %d\n", __func__, adjusted_sector_count, adjusted_sector_count * SECTOR_SIZE);
	return adjusted_sector_count * SECTOR_SIZE;
}

static int esp_flashBegin (uint32_t addr, uint32_t size)
{
	uint32_t words[4];

	words[0] = esp_calcEraseSize(addr, size);
	words[1] = (size + BLOCKSIZE_FLASH - 1) / BLOCKSIZE_FLASH;
	words[2] = BLOCKSIZE_FLASH;
	words[3] = addr;
	return esp_sendCmdBlock(ESPBL_FLASH_BEGIN, words, sizeof(words), 30000);
}

static void esp_flashEnd (bool reboot)
{
	uint32_t data;

	data = (reboot) ? 0 : 1;
	esp_sendCmdBlock(ESPBL_FLASH_END, &data, sizeof(data), 1000);
}

static int esp_flashFile (char *fname, uint32_t addr)
{
	struct yaffs_stat st;
	uint8_t buf[BLOCKSIZE_FLASH];
	int len, fd, seq;

	if (yaffs_lstat(fname, &st) != 0) {
		log_error("%s(): stat(\"%s\") returns error\n", __func__, fname);
		return -1;
	}
	log_msg (LOG_INFO, "%s('%s'): Size %llu bytes @ 0x%08lx\n", __func__, fname, st.st_size, addr);
	if ((fd = yaffs_open(fname, O_RDONLY, 0)) >= 0) {
		esp_flashBegin(addr, (uint32_t) st.st_size);
		seq = 0;
		while ((len = yaffs_read(fd, buf, sizeof(buf))) > 0) {
			if (len < (int) sizeof(buf)) memset (&buf[len], 0xFF, sizeof(buf) - len);		// fill rest of block with 0xFF
			esp_sendDataBlock(buf, seq++);
		}
		yaffs_close(fd);
	} else {
		log_error ("%s() cannot open \"%s\" for reading\n", __func__, fname);
	}
	return 0;
}

static void esp_update (void)
{
	do_update = false;
	log_msg(LOG_INFO, "%s(): starting\n", __func__);
	esp_flashFile(ESP_DIR BOOTLOADER_FILE, BOOT_ADDR);
	esp_flashFile(ESP_DIR PARTITION_FILE, PARTITION_ADDR);
	esp_flashFile(ESP_DIR APPLICATION_FILE, APPLICATION_ADDR);
	esp_flashEnd(false);
	vTaskDelay(100);
	log_msg(LOG_INFO, "%s(): finished ... start application\n", __func__);
	esp_resetApplication();
}

void esp_testthread (void *pvParameter)
{
	uint8_t buf[128], *s, *end, c;
	unsigned long rc;
	bool booting = false;

	(void) pvParameter;

	task = xTaskGetCurrentTaskHandle();
	s = buf;
	end = buf + sizeof(buf) - 1;

	rxbuf = esp_allocbuffer(1024);
	esp_inituart (BAUDRATE);
	esp_resetApplication();

	log_msg(LOG_INFO, "%s() Started\n", __func__);

	for (;;) {
		rc = ulTaskNotifyTake(pdTRUE, 5000);
		if (rc > 0) {
			if (do_update) {
				esp_resetBootloader();
				esp_update();
				booting = false;
			} else if (booting) {
				log_msg (LOG_INFO, "%s(): got packet from ESP-01 CMD=0x%02X len=%d %s ERRCode 0x%02x\n", __func__, rx.cmd,
						rx.size, rx.data[rx.size - 2] ? "FAILURE" : "SUCCESS", rx.data[rx.size - 1]);
				if (rx.data[rx.size - 2] != 0) {
					esp_resetBootloader();
				}
			} else {
				while (rxbuf->idxin != rxbuf->idxout) {
					c = rxbuf->buffer[rxbuf->idxout];
					if (++rxbuf->idxout >= rxbuf->len) rxbuf->idxout = 0;
					if (s < end) *s++ = c;
					*s = 0;
					if (c == '\n') {
						log_msg (LOG_INFO, "ESP: %s", buf);
						s = buf;
					}
				}
			}
		}
	}

}

void esp_triggerUpdate (void)
{
	if (task) {
		do_update = true;
		xTaskNotifyGive(task);
		log_msg (LOG_INFO, "%s() update triggered\n", __func__);
	} else {
		log_error("%s() ESP-Thread not running (?)\n", __func__);
	}
}

void USART6_IRQHandler (void)
{
	static uint8_t *rxdata, *txdata;

	uint8_t c;

	BaseType_t xHigherPriorityTaskWoken = 0;

	// Step 1: receive characters from RX-FIFO (until FIFO is empty)
	while ((USART6->CR3 & USART_CR3_RXFTIE) && (USART6->ISR & USART_ISR_RXNE_RXFNE)) {
		c = USART6->RDR;
		switch (rxst) {
			case SLIP_STARTUP:
				if (rxbuf) {
					rxbuf->buffer[rxbuf->idxin] = c;
					if (++rxbuf->idxin >= rxbuf->len) rxbuf->idxin = 0;
					if (rxbuf->idxin == rxbuf->idxout && ++rxbuf->idxout > rxbuf->len) rxbuf->idxout = 0;
					if (c == '\r' || c == '\n') vTaskNotifyGiveFromISR(task, &xHigherPriorityTaskWoken);
				}
				break;
			case SLIP_IDLE:
				if (c == SLIP_BLOCK) {
					rxst = SLIP_TRANSFER;	// advance to block reception
					rxdata = (uint8_t *) &rx;
					USART6->ICR = USART_ICR_RTOCF;					// clear RX timeout
					SET_BIT(USART6->CR2, USART_CR2_RTOEN);			// and wait for silence ...
				}
				break;
			case SLIP_TRANSFER:
				if (c == SLIP_BLOCK) {		// unexpected block termination, treat as new block begin
					rxdata = (uint8_t *) &rx;
				} else if (c == SLIP_ESCAPE) {
					rxst = SLIP_ESCSTATE;
				} else {
					if (rxdata) *rxdata++ = c;
				}
				break;
			case SLIP_ESCSTATE:
				rxst = SLIP_TRANSFER;
				if (rxdata) {
					if (c == SLIP_ESC_ESCAPE) *rxdata++ = SLIP_ESCAPE;
					else if (c == SLIP_ESC_BLOCK) *rxdata++ = SLIP_BLOCK;
					else {
						rxst = SLIP_IDLE;
						rxdata = NULL;
					}
				}
				break;
			case SLIP_COMPLETE:
				rxst = SLIP_IDLE;			// no matter what we received - we end this block
				rxdata = NULL;
				CLEAR_BIT(USART6->CR2, USART_CR2_RTOEN);		// don't wait any longer
				if (c == SLIP_BLOCK) {		// this is a valid termination character - wake up upper thread
					vTaskNotifyGiveFromISR(task, &xHigherPriorityTaskWoken);
				}
				break;
		}
		if (rxst == SLIP_TRANSFER && rxdata >= rx.data && rxdata == &rx.data[rx.size]) rxst = SLIP_COMPLETE;
		if (USART6->CR2 & USART_CR2_RTOEN && USART6->ISR & USART_ISR_RTOF) {
			CLEAR_BIT(USART6->CR2, USART_CR2_RTOEN);		// don't wait any longer
		}
	}

	while (USART6->CR3 & USART_CR3_TXFTIE && (USART6->ISR & USART_ISR_TXE_TXFNF)) {
		switch (txst) {
			case SLIP_STARTUP:
				if (rxst == SLIP_STARTUP) rxst = SLIP_IDLE;
				/*  FALL THRU */
			case SLIP_IDLE:
				USART6->TDR = SLIP_BLOCK;
				txdata = (uint8_t *) &tx;
				txst = SLIP_TRANSFER;
				break;
			case SLIP_TRANSFER:
				if (!txdata) {		// this is an error
					txst = SLIP_IDLE;
					txdata = NULL;
					CLEAR_BIT(USART6->CR3, USART_CR3_TXFTIE);
					break;
				}
				c = *txdata;
				if (c == SLIP_ESCAPE || c == SLIP_BLOCK) {
					USART6->TDR = SLIP_ESCAPE;
					txst = SLIP_ESCSTATE;
				} else {
					USART6->TDR = c;
					txdata++;
				}
				break;
			case SLIP_ESCSTATE:
				c = *txdata++;
				txst = SLIP_TRANSFER;
				if (c == SLIP_ESCAPE) {
					USART6->TDR = SLIP_ESC_ESCAPE;
				} else if (c == SLIP_BLOCK) {
					USART6->TDR = SLIP_ESC_BLOCK;
				} else {		// this should never happen - we must only escape SLIP_ESCAPE and SLIP_BLOCK
					USART6->TDR = c;
				}
				break;
			case SLIP_COMPLETE:
				USART6->TDR = SLIP_BLOCK;
				txst = SLIP_IDLE;
				txdata = NULL;
				CLEAR_BIT(USART6->CR3, USART_CR3_TXFTIE);
				break;
		}
		if (txst == SLIP_TRANSFER && txdata >= tx.data && txdata == &tx.data[tx.size]) {
			txst = SLIP_COMPLETE;
		}
	}

	NVIC_ClearPendingIRQ(USART6_IRQn);
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
