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
#include "rb2.h"
#include "qspi.h"

void qspi_init (void)
{
	uint32_t dummy;

	(void) dummy;	// is set but never used - make compiler happy

	printf("%s() starting ...\n", __func__);
	QUADSPI->CR = 0;	// disable QSPI and reset it
	while (QUADSPI->SR & QUADSPI_SR_FLEVEL_Msk) dummy = QUADSPI->DR;	// empty out the FIFO
	while (QUADSPI->SR & QUADSPI_SR_BUSY) /* wait until QSPI is ready */ ;

	printf("%s() QSPI is ready ...\n", __func__);

	QUADSPI->FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;	// clear all error flags

	// Prescaler = (2 + 1) = 200MHz / 3 = 66,6MHz, FIFO threshold 24 Bytes (FIFO has 32 bytes capacity)
	QUADSPI->CR = (2 << QUADSPI_CR_PRESCALER_Pos) | (24 << QUADSPI_CR_FTHRES_Pos);
	// The flash is 256MByte, that makes up 2^^28, so put 27 as FSIZE in this register
	QUADSPI->DCR = (27 << QUADSPI_DCR_FSIZE_Pos) | (7 << QUADSPI_DCR_CSHT_Pos);

	SET_BIT (QUADSPI->CR, QUADSPI_CR_EN);	// enable QSPI
	printf("%s() QSPI is now enabled ...\n", __func__);
}

uint8_t *qspi_readFIFO (uint8_t *buf, size_t maxbytes)
{
	uint32_t data;
	uint8_t *p;
	int len;

	p = buf;
	while (((QUADSPI->SR & QUADSPI_SR_FLEVEL_Msk) >> QUADSPI_SR_FLEVEL_Pos) >= 4) {
		data = QUADSPI->DR;
		for (len = 0; len < 4; len++) {
			if (p && (p - buf) < (int) maxbytes) *p++ = data & 0xFF;
			data >>= 8;
		}
	}

	if (QUADSPI->SR & QUADSPI_SR_TCF) {		// if transaction is complete, we can read the remaining bytes
		if ((len = (QUADSPI->SR & QUADSPI_SR_FLEVEL_Msk) >> QUADSPI_SR_FLEVEL_Pos) > 0) {
			data = QUADSPI->DR;
			while (len) {
				if (p && (p - buf) < (int) maxbytes) *p++ = data & 0xFF;
				data >>= 8;
				len--;
			}
		}
	}

	return p;		// return advanced buffer pointer
}

/**
 * Setup the CCR register variable with the supplied command.
 *
 * The command is always transferred on a single lane (technically, the QSPI
 * module could handle 2-lane and 4-lane mode, but we will not use this).
 *
 * @param ccr	the current ccr value to be modfied
 * @param cmd	the 8-bit command to put to use
 * @param dmy	the amount of dummy clocks before a possible data phase (0 - 31)
 * @return		the updated ccr register value containg the cmd and beeing set to single lane transfer
 */
uint32_t qspi_ccrSetCommand (uint32_t ccr, uint8_t cmd, int dmy)
{
	ccr &= ~(QUADSPI_CCR_DCYC_Msk | QUADSPI_CCR_IMODE_Msk | QUADSPI_CCR_INSTRUCTION_Msk);
	ccr |= ((dmy & 0x1F) << QUADSPI_CCR_DCYC_Pos) | (0b01 << QUADSPI_CCR_IMODE_Pos) | cmd;

	return ccr;
}

/**
 * Setup the CCR register variable with the supplied address setting.
 *
 * The address can be transferred on a single, dual or quad lanes. It
 * can consist of 8, 16, 24 or 32 bits. If you don't have an address
 * phase, just set no values (but be sure to start with an empty ccr
 * value). But you can also specify ADR_NONE as the lanes parameter,
 * which will clear the corresponding bits in the ccr variable
 *
 * @param ccr	the current ccr value to be modfied
 * @param lanes	one of the lane settings QSPI_NOLANE, QSPI_1LANE, QSPI_2LANE, QSPI_4LANE.
 * @param len	the length of the address to transfer, if any. Use QSPI_8BITS, QSPI_16BITS, QSPI_24BITS or QSPI_32BITS.
 * @return		the updated ccr register value containg the address settings
 */
uint32_t qspi_ccrSetAddrConfig (uint32_t ccr, qspiLanesT lanes, qspiBitLenT len)
{
	ccr &= ~(QUADSPI_CCR_ADSIZE_Msk | QUADSPI_CCR_ADMODE_Msk);
	ccr |= (len << QUADSPI_CCR_ADSIZE_Pos) | (lanes << QUADSPI_CCR_ADMODE_Pos);
	return ccr;
}

/**
 * Setup the CCR register variable with the supplied alternate mode bytes setting.
 * The used lanes and field size has the same idiom as for the address bytes,
 * therefor we can use the same enum types here. The values are just stored in
 * a different position of the ccr register.
 *
 * The alternate bytes can be transferred on a single, dual or quad lanes.
 * there can be 8, 16, 24 or 32 bits of alterate bytes. If you don't have need
 * the alternate bytes phase, just set no values (but be sure to start with an
 * empty ccr value). But you can also specify ADR_NONE as the lanes parameter,
 * which will clear the corresponding bits in the ccr variable
 *
 * @param ccr	the current ccr value to be modfied
 * @param lanes	one of the lane settings QSPI_NOLANE, QSPI_1LANE, QSPI_2LANE, QSPI_4LANE.
 * @param len	the length of the alternate bytes to transfer, if any. Use QSPI_8BITS, QSPI_16BITS, QSPI_24BITS or QSPI_32BITS.
 * @return		the updated ccr register value containg the alternate bytes settings
 */
uint32_t qspi_ccrSetAlternateBytesConfig (uint32_t ccr, qspiLanesT lanes, qspiBitLenT len)
{
	ccr &= ~(QUADSPI_CCR_ABSIZE_Msk | QUADSPI_CCR_ABMODE_Msk);
	ccr |= (len << QUADSPI_CCR_ABSIZE_Pos) | (lanes << QUADSPI_CCR_ABMODE_Pos);
	return ccr;
}

/**
 * Wait for the QSPI to be idle (i.e. ready for a new job).
 * Usually, this should be immediately the case, but let's
 * be sure.
 */
static bool qspi_waitReady (const char *caller)
{
	TickType_t to;

	(void) caller;

	to = xTaskGetTickCount() + 80;

	while (QUADSPI->SR & QUADSPI_SR_BUSY) {
		if ((to - xTaskGetTickCount()) & (1 << 31)) {
//			if (caller) {
//				printf ("%s(): from %s(): timout waiting for QSPI to get ready (SR=0x%04lx)\n", __func__, caller, QUADSPI->SR);
//			} else {
//				printf ("%s(): timout waiting for QSPI to get ready (SR=0x%04lx)\n", __func__, QUADSPI->SR);
//			}
			return false;		// timeout waiting for QSPI to get ready
		}
		portYIELD();
	}
	return true;
}

/**
 * Start a READ transfer on the QSPI
 *
 * The supplied ccr value is enriched with the function mode code for "indirect read" (FMODE = 0b01).
 * All other settings in ccr should be already set appropriate.
 *
 * If you need any alternate bytes, they should have been written to the ABR beforehand.
 *
 * @param ccr	preset register contents for CCR, specifying the details of the transfer
 * @param adr	an optional address, if ccr contains the marker, that an address is neeeded
 * @param len	the expected data length, if ccr contains the marker, that a data phase is requested
 * @param datalanes		the number of lanes used for the data phase, one of QSPI_NOLANE, QSPI_1LANE, QSPI_2LANE, QSPI_4LANE
 * @return		0 for success or -1 for error (if QSPI is not ready for a new transaction)
 */
int qspi_triggerRead (uint32_t ccr, uint32_t adr, size_t len, qspiLanesT datalanes)
{
	if (!qspi_waitReady(__func__)) {
		return -1;	// QSPI not available
	}

	ccr &= ~(QUADSPI_CCR_FMODE_Msk | QUADSPI_CCR_DMODE_Msk);
	ccr |= (0b01 << QUADSPI_CCR_FMODE_Pos) | (datalanes << QUADSPI_CCR_DMODE_Pos);	// this will be an indirect read with adequate number of lanes

	// request to read len bytes of data (if a data phase was specified
	if (datalanes != QSPI_NOLANE) {
//		printf ("%s() DLR=%d\n", __func__, len - 1);
		QUADSPI->DLR = len - 1;
	}

//	printf ("%s() CCR=0x%08X\n", __func__, ccr);
	QUADSPI->CCR = ccr;
	// if an address phase is specified, trigger the action by writing to the ADR register
	if (ccr & QUADSPI_CCR_ADMODE_Msk) {
//		printf ("%s() AR=%08X\n", __func__, adr);
		QUADSPI->AR = adr;
	}

//	if ((ccr & QUADSPI_CCR_DMODE_Msk) == 0) {
//		qspi_waitReady(__func__);
//	}

	return 0;
}

/**
 * Start or at least prepare a WRITE transfer on the QSPI
 *
 * The supplied ccr value is enriched with the function mode code for "indirect write" (FMODE = 0b00).
 * All other settings in ccr should already be set appropriate.
 *
 * If you need any alternate bytes, they should have been written to the ABR beforehand.
 *
 * Note that the transaction may beginn, if no data is needed. If data must be supplied,
 * the transaction is started with the first write to the DR register.
 *
 * @param ccr	preset register contents for CCR, specifying the details of the transfer
 * @param adr	an optional address, if ccr contains the marker, that an address is neeeded
 * @param len	the expected data length, if ccr contains the marker, that a data phase is requested
 * @param datalanes		the number of lanes used for the data phase, one of QSPI_NOLANE, QSPI_1LANE, QSPI_2LANE, QSPI_4LANE
 * @return		0 for success or -1 for error (if QSPI is not ready for a new transaction)
 */
int qspi_triggerWrite (uint32_t ccr, uint32_t adr, size_t len, qspiLanesT datalanes)
{
	if (!qspi_waitReady(__func__)) {
		return -1;	// QSPI not available
	}

	ccr &= ~(QUADSPI_CCR_FMODE_Msk | QUADSPI_CCR_DMODE_Msk);	// clearing the FMODE bits means, that this will be an indirect write
	ccr |= (datalanes << QUADSPI_CCR_DMODE_Pos);

	// request to write len bytes of data (if a data phase was specified
	if (datalanes != QSPI_NOLANE) {
//		printf ("%s() DLR=%d\n", __func__, len - 1);
		QUADSPI->DLR = len - 1;
	}

//	printf ("%s() CCR=0x%08X\n", __func__, ccr);
	QUADSPI->CCR = ccr;
	// if an address phase is specified, write it to the ADR register (without data phase this triggers the transaction)
	if (ccr & QUADSPI_CCR_ADMODE_Msk) {
//		printf ("%s() AR=%08X\n", __func__, adr);
		QUADSPI->AR = adr;
	}

//	if ((ccr & QUADSPI_CCR_DMODE_Msk) == 0) {
//		qspi_waitReady(__func__);
//	}
	return 0;
}

/**
 * Do a WRITE transfer on the QSPI
 *
 * The supplied ccr value is enriched with the function mode code for "indirect write" (FMODE = 0b00).
 * All other settings in ccr should already be set appropriate.
 *
 * If you need any alternate bytes, they should have been written to the ABR beforehand.
 *
 * @param ccr	preset register contents for CCR, specifying the details of the transfer
 * @param adr	an optional address, if ccr contains the marker, that an address is neeeded
 * @param data	the data to transfer (if any), needs not to be aligned
 * @param len	the expected data length, if ccr contains the marker, that a data phase is requested
 * @param datalanes		the number of lanes used for the data phase, one of QSPI_NOLANE, QSPI_1LANE, QSPI_2LANE, QSPI_4LANE
 * @return		0 for success, error bits from status register after data phase (always positive, maybe timeout or transfer error)
 * 				or -1 for error (if QSPI is not ready for a new transaction)
 */
int qspi_sendData (uint32_t ccr, uint32_t adr, const uint8_t *data, size_t len, qspiLanesT datalanes)
{
	int status = 0;

	if (!qspi_waitReady(__func__)) {
		return -1;	// QSPI not available
	}

//	printf ("%s(%u, %p, %d)\n", __func__, adr, data, len);

	ccr &= ~(QUADSPI_CCR_FMODE_Msk | QUADSPI_CCR_DMODE_Msk);	// clearing the FMODE bits means, that this will be an indirect write
	ccr |= (datalanes << QUADSPI_CCR_DMODE_Pos);

	// request to write len bytes of data (if a data phase was specified
	if (datalanes != QSPI_NOLANE) QUADSPI->DLR = len - 1;

	QUADSPI->CCR = ccr;
	// if an address phase is specified, write it to the ADR register (without data phase this triggers the transaction)
	if (ccr & QUADSPI_CCR_ADMODE_Msk) QUADSPI->AR = adr;

	if (data && len && (ccr & QUADSPI_CCR_DMODE_Msk) != 0) {
		while (((uint32_t) data & 3) && (len > 0)) {	// write the first bytes until aligned for 32-bit accesses
			QUADSPI->DR = *data++;
			len--;
		}
		while (len > 3) {	// write the rest as 32bit values
			const uint32_t *p = (uint32_t *) data;
			while (len > 3 && ((QUADSPI->SR & QUADSPI_SR_FLEVEL_Msk) >> QUADSPI_SR_FLEVEL_Pos) <= 28) {
				QUADSPI->DR = *p++;
				len -= 4;
			}
			while (!(QUADSPI->SR & (QUADSPI_SR_TEF | QUADSPI_SR_FTF))) /* taskYIELD() */;		// wait, until the threshold is reached again
			data = (uint8_t *) p;
		}
		while (len > 0) {					// write the last bytes that are not integral 32-bit words
			QUADSPI->DR = *data++;
			len--;
		}

		while (!(QUADSPI->SR & (QUADSPI_SR_TCF | QUADSPI_SR_TEF))) /* taskYIELD() */;
		status = QUADSPI->SR & (QUADSPI_SR_TOF | QUADSPI_SR_TEF);
		QUADSPI->FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;	// clear all error flags
	}
//	printf ("%s() finished\n", __func__);
	return status;
}
