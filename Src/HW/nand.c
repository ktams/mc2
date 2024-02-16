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

#include "rb2.h"
#include "qspi.h"
#include "nandflash.h"

#define NAND_PROGRAM_LOAD			0x02	///< write data to internal buffer
#define NAND_READ_BUFFER			0x03	///< read the buffered cell array data (using one data lane)
#define NAND_WRITE_DISABLE			0x04	///< disable all kind of write accesses
#define NAND_WRITE_ENABLE			0x06	///< enable all kind of write accesses
#define NAND_GET_FEATURE			0x0F	///< read a feature byte
#define NAND_PROGRAM_EXECUTE		0x10	///< Program data from buffer to cell array
#define NAND_READ_CELLARRAY			0x13	///< read the specified cell array to internal buffers
#define NAND_SET_FEATURE			0x1F	///< write a feature byte
#define NAND_PROTEXT_EXECUTE		0x2A	///< protect a Block
#define NAND_READ_BUFFERx2			0x3B	///< read the buffered cell array data (using two data lanes)
#define NAND_READ_BUFFERx4			0x6B	///< read the buffered cell array data (using four data lanes)
#define NAND_PROGRAM_LOAD_RANDOM	0x84	///< write data to internal buffer at random addresses
#define NAND_BLOCK_ERASE			0xD8	///< erase the addressed block
#define NAND_READ_ID				0x9F	///< command code for reading device ID
#define NAND_RESET					0xFF	///< reset NAND-logic

struct nandfuncs {
	const char		*desc;					///< a descriptive String to distinguish the flashes in a human readable form
	uint32_t		 uid_blk;				///< the block where the UID is stored (in the OTP area)
};

/*
 * ====================================================================================================================
 * different functions for specialitoies in NAND FLASH memory chips
 * ====================================================================================================================
 */

const struct nandfuncs toshiba = {
	.desc = "TOSHIBA/KIOXIA TC58CVG1S3H",
	.uid_blk = 0,
};

const struct nandfuncs gigadev = {
	.desc = "GigaDevice GD5F2GQ5",
	.uid_blk = 6,
};

const struct nandfuncs *nand;
static int current_chunk;

static void nand_writeEnable (void)
{
	uint32_t ccr;

	ccr = qspi_ccrSetCommand(0, NAND_WRITE_ENABLE, 0);
	qspi_triggerWrite(ccr, 0, 0, QSPI_NOLANE);
}

static void nand_writeDisable (void)
{
	uint32_t ccr;

	ccr = qspi_ccrSetCommand(0, NAND_WRITE_DISABLE, 0);
	qspi_triggerWrite(ccr, 0, 0, QSPI_NOLANE);
}

static void nand_programLoad (uint32_t adr, const uint8_t *data, int len)
{
	uint32_t ccr;

	ccr = qspi_ccrSetCommand(0, NAND_PROGRAM_LOAD, 0);
	ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_16BITS);
	qspi_sendData(ccr, adr, data, len, QSPI_1LANE);
}

static void nand_programLoadRandom (uint32_t adr, const uint8_t *data, int len)
{
	uint32_t ccr;

	ccr = qspi_ccrSetCommand(0, NAND_PROGRAM_LOAD_RANDOM, 0);
	ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_16BITS);
	qspi_sendData(ccr, adr, data, len, QSPI_1LANE);
}

static void nand_setFeature (uint8_t adr, uint8_t val)
{
	uint32_t ccr;

	ccr = qspi_ccrSetCommand(0, NAND_SET_FEATURE, 0);
	ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_8BITS);
	qspi_triggerWrite(ccr, adr, 1, QSPI_1LANE);

	QUADSPI->DR = val;		// start action by writing the data byte

	while (!(QUADSPI->SR & (QUADSPI_SR_TCF | QUADSPI_SR_TEF))) taskYIELD();
	QUADSPI->FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;	// clear all error flags
//	log_msg (LOG_INFO, "%s(%02X) = 0x%02x\n", __func__, adr, val);
}

static uint8_t nand_getFeature (uint8_t adr)
{
	uint32_t ccr;
	uint8_t status;

	ccr = qspi_ccrSetCommand(0, NAND_GET_FEATURE, 0);
	ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_8BITS);
	qspi_triggerRead(ccr, adr, 1, QSPI_1LANE);

	while (!(QUADSPI->SR & (QUADSPI_SR_TCF | QUADSPI_SR_TEF))) taskYIELD();

	status = QUADSPI->DR;
	QUADSPI->FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;	// clear all error flags
//	log_msg (LOG_INFO, "%s(%02X) = 0x%02X\n", __func__, adr, status);
	return status;
}

static uint8_t nand_waitReady (void)
{
	uint8_t status;

	while ((status = nand_getFeature(0xC0)) & 0x01) taskYIELD();
	return status;
}

static void nand_readid (void)
{
	uint32_t ccr;
	uint16_t id;

	ccr = qspi_ccrSetCommand(0, NAND_READ_ID, 8);
	qspi_triggerRead(ccr, 0, 2, QSPI_1LANE);

	while (!(QUADSPI->SR & (QUADSPI_SR_TCF | QUADSPI_SR_TEF))) taskYIELD();

	id = QUADSPI->DR;
	switch (id & 0xFF) {	// select by manufacturer ID
		case 0xC8:	// GigaDevice (newer flashes, DEV-ID 0x52)
			nand = &gigadev;
			break;
		case 0x98:	// Toshiba (first devices, DEV-ID 0xCB or 0xEB)
			nand = &toshiba;
			break;
	}
	QUADSPI->FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;	// clear all error flags
	log_msg (LOG_INFO, "%s() NAND Read-ID: 0x%02x / 0x%02x (%s)\n", __func__, id & 0xFF, (id >> 8) & 0xFF, nand->desc);
}

/**
 * Reads the addressed page to the flash-internal buffer and returns the
 * status of this operation (giving info about ECC corrections)
 *
 * @param adr	the page number (0-based)
 * @return		the status byte from GET_FEATURE(0xC0)
 */
static uint8_t nand_readCellArray (uint32_t adr)
{
	uint32_t ccr;
	uint8_t status = 0;

	adr &= 0x1FFFF;

	ccr = qspi_ccrSetCommand(0, NAND_READ_CELLARRAY, 0);
	ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_24BITS);

	qspi_triggerWrite(ccr, adr, 0, QSPI_NOLANE);

	while (QUADSPI->SR & QUADSPI_SR_BUSY) taskYIELD();
	QUADSPI->FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;	// clear all error flags

	while ((status = nand_getFeature(0xC0)) & 0x01) taskYIELD();
//	log_msg (LOG_INFO, "%s(%06X) ready\n", __func__, adr);
	return status;
}

static void nand_readBuffer (uint16_t adr, uint8_t *buf, size_t len)
{
	uint32_t ccr;
	uint8_t *p;

	ccr = qspi_ccrSetCommand(0, NAND_READ_BUFFERx4, 8);
	ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_16BITS);
	qspi_triggerRead(ccr, adr, len, QSPI_4LANE);
	p = buf;

	while (!(QUADSPI->SR & (QUADSPI_SR_TCF | QUADSPI_SR_TEF))) {
		while (!(QUADSPI->SR & (QUADSPI_SR_TCF | QUADSPI_SR_TEF | QUADSPI_SR_FTF))) taskYIELD();	// wait for FIFO threshold level
		p = qspi_readFIFO(p, len - (p - buf));
	}
	p = qspi_readFIFO(p, len - (p - buf));

	QUADSPI->FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;	// clear all error flags
	if ((p - buf) != (int) len) {
		log_error ("%s() @ 0x%04x req. %u read %d\n", __func__, adr, len, p - buf);
	}
}

static void nand_read_uid (void)
{
	uint8_t id[32];
	uint8_t i, status;

	nand_waitReady();
	status = nand_getFeature (0xB0);
	status |= 0x01;								// enable QSPI-commands
	nand_setFeature (0xB0, status | 0x40);		// enable access to configuration area
	nand_readCellArray(nand->uid_blk);
	nand_readBuffer(0, id, sizeof(id));
	nand_setFeature (0xB0, status & ~0x40);		// disable access to configuration area

	printf ("%s():\t", __func__);
	for (i = 0; i < 16; i++) printf ("%02x ", id[i]);
	printf ("\n\t\t\t");
	for (i = 16; i < 32; i++) printf ("%02x ", id[i]);
	printf ("\n");

	for (i = 0; i < 16; i++) {
		if (id[i] != ((~id[i + 16]) & 0xFF)) log_error ("%s(): ~ID[%d] should be %02x\n", __func__, i, ~id[i]);
	}
}

static void nand_unlock (void)
{
	nand_setFeature(0xA0, 0);
}

int nand_write_chunk (struct yaffs_dev *dev, int nand_chunk, const u8 *data, int data_len, const u8 *oob, int oob_len)
{
	uint32_t ccr;
	uint8_t status;

	(void) dev;

//	log_msg (LOG_INFO, "%s(%d) DATA %d OOB %d\n", __func__, nand_chunk, data_len, oob_len);

	nand_waitReady();
	if ((!data || !data_len) && (!oob || !oob_len)) return YAFFS_OK;	// nothing to be written here, so ignore this request

	current_chunk = -1;			// the current page is not valid anymore (regardless of success or failure of this command)
//	nand_writeEnable();

	nand_waitReady();
	if (data && data_len > 0) {
		if ((u32) data_len > dev->param.total_bytes_per_chunk) data_len = dev->param.total_bytes_per_chunk;
		nand_programLoad(0, data, data_len);
	}
	if (oob && oob_len > 0) {
		if ((u32) oob_len > dev->param.spare_bytes_per_chunk) oob_len = dev->param.spare_bytes_per_chunk;
		if (!data || data_len <= 0) nand_programLoad(dev->param.total_bytes_per_chunk, oob, oob_len);
		else nand_programLoadRandom(dev->param.total_bytes_per_chunk, oob, oob_len);
	}

	nand_writeEnable();
	ccr = qspi_ccrSetCommand(0, NAND_PROGRAM_EXECUTE, 0);
	ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_24BITS);
	qspi_triggerWrite(ccr, nand_chunk, 0, QSPI_NOLANE);

	status = nand_waitReady();

	nand_writeDisable();
	return (status & 0x08) ? YAFFS_FAIL : YAFFS_OK;		// bit 3 reports program errors (if set, a write error occured)
}

int nand_read_chunk (struct yaffs_dev *dev, int nand_chunk, u8 *data, int data_len, u8 *oob, int oob_len, enum yaffs_ecc_result *ecc_result)
{
	uint8_t status;

//	log_msg (LOG_INFO, "%s(%d) DATA %d OOB %d\n", __func__, nand_chunk, data_len, oob_len);

	nand_waitReady();
	if (nand_chunk != current_chunk) {		// we must read the page to the internal buffer
		status = nand_readCellArray(nand_chunk);
		switch (status & 0x30) {
			case 0x00:
				if (ecc_result) *ecc_result = YAFFS_ECC_RESULT_NO_ERROR;
				current_chunk = nand_chunk;	// OK, this chunk is now loaded in internal FLASH buffer
				break;
			case 0x10:		// bit flips corrected, bit flip count did not exceed threshold
			case 0x30:		// bit flips corrected, bit flip count exceeded threshold
				if (ecc_result) *ecc_result = YAFFS_ECC_RESULT_FIXED;
				current_chunk = -1;			// we may want to re-read this chunk
				break;
			case 0x20:
				if (ecc_result) *ecc_result = YAFFS_ECC_RESULT_UNFIXED;
				current_chunk = -1;			// we may want to re-read this chunk
				break;
		}
		if (status & 0x30) log_msg (LOG_WARNING, "%s(): Status 0x%02X for chunk %d\n", __func__, status & 0x30, nand_chunk);
	} else {						// if the page is current, then the ECC result must have been OK (with no bit flips)
		if (ecc_result) *ecc_result = YAFFS_ECC_RESULT_NO_ERROR;
	}

	if (data && data_len > 0) {
		if ((u32) data_len > dev->param.total_bytes_per_chunk) data_len = dev->param.total_bytes_per_chunk;
		nand_readBuffer(0, data, data_len);
	}

	if (oob && oob_len > 0) {
		if ((u32) oob_len > dev->param.spare_bytes_per_chunk) oob_len = dev->param.spare_bytes_per_chunk;
		nand_readBuffer(dev->param.total_bytes_per_chunk, oob, oob_len);
	}

	return YAFFS_OK;
}

int nand_erase (struct yaffs_dev *dev, int block_no)
{
	uint32_t ccr;
	uint8_t status;
	int rc;

	(void) dev;

	log_msg (LOG_INFO, "%s(%d)\n", __func__, block_no);

	nand_waitReady();
	nand_writeEnable();

	nand_waitReady();
	ccr = qspi_ccrSetCommand(0, NAND_BLOCK_ERASE, 0);
	ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_24BITS);
	rc = qspi_triggerWrite(ccr, block_no << 6, 0, QSPI_NOLANE);		// adrdess the first page in the given block (64 pages / block)
	if (rc) {
		log_error ("%s(): Problems scheduling erase request\n", __func__);
//	} else {
//		log_msg (LOG_INFO, "%s(): erasure stared\n", __func__);
	}

	status = nand_waitReady();

	nand_writeDisable();
	current_chunk = -1;

//	log_msg (LOG_INFO, "%s(): erasure ended\n", __func__);
	return (status & 0x04) ? YAFFS_FAIL : YAFFS_OK;		// bit 2 reports erase errors (if set, an error occured)
}

#if 0
static int nand_eraseFlash (int startblk, int endblk)
{
	int i, rc;

	for (i = startblk; i <= endblk; i++) {
		if ((rc = nand_erase(NULL, i)) != YAFFS_OK) return rc;
	}
	return YAFFS_OK;
}
#endif

int nand_mark_bad (struct yaffs_dev *dev, int block_no)
{
	uint32_t ccr;
	uint32_t marker = 0;
	int i, nand_chunk;

	log_msg (LOG_WARNING, "%s(%d)\n", __func__, block_no);

	nand_waitReady();
	nand_erase(dev, block_no);
	nand_waitReady();
	nand_writeEnable();

	nand_chunk = block_no * dev->param.chunks_per_block;

	for (i = 0; (unsigned) i < dev->param.chunks_per_block; i++, nand_chunk++) {
		nand_waitReady();
		nand_programLoad(dev->param.total_bytes_per_chunk + dev->param.spare_bytes_per_chunk, (uint8_t *) &marker, sizeof(marker));
		ccr = qspi_ccrSetCommand(0, NAND_PROGRAM_EXECUTE, 0);
		ccr = qspi_ccrSetAddrConfig(ccr, QSPI_1LANE, QSPI_24BITS);
		qspi_triggerWrite(ccr, nand_chunk, 0, QSPI_NOLANE);
	}

	nand_writeDisable();
	current_chunk = -1;
	return YAFFS_OK;
}

int nand_check_bad (struct yaffs_dev *dev, int block_no)
{
	uint32_t marker;
	int i, nand_chunk;

	nand_chunk = block_no * dev->param.chunks_per_block;


	for (i = 0; (unsigned) i < dev->param.chunks_per_block; i++, nand_chunk++) {
		marker = 0;
		nand_waitReady();
		nand_readCellArray(nand_chunk);			// status is ignored because we must be prepared to have a real bad block
		nand_readBuffer(dev->param.total_bytes_per_chunk + dev->param.spare_bytes_per_chunk, (uint8_t *) &marker, sizeof(marker));
		if (marker != 0xFFFFFFFF) break;
	}

	if (marker != 0xFFFFFFFF) log_msg (LOG_WARNING, "%s(): Block %d BAD @ chunk %d marker=0x%08lx\n", __func__, block_no, i, marker);
	current_chunk = -1;

	return (marker != 0xFFFFFFFF) ? YAFFS_FAIL : YAFFS_OK;
}

int nand_initialise (struct yaffs_dev *dev)
{
	(void) dev;

	qspi_init();
	nand_readid();
	nand_read_uid();
	log_msg (LOG_INFO, "%s(): feature A0 = 0x%02x\n", __func__, nand_getFeature(0xA0));
	nand_unlock();
	log_msg (LOG_INFO, "%s(): feature after unlock A0 = 0x%02x\n", __func__, nand_getFeature(0xA0));

//	nand_eraseFlash(0, 2047);

	current_chunk = -1;	// we have not yet read a page in the flash
	log_msg (LOG_INFO, "%s() finished\n", __func__);
	return YAFFS_OK;
}

int nand_deinitialise (struct yaffs_dev *dev)
{
	(void) dev;

	log_msg (LOG_INFO, "%s() finished\n", __func__);
	return YAFFS_OK;
}
