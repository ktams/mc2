/*
 * qspi.h
 *
 *  Created on: 24.10.2019
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

#ifndef __QSPI_H__
#define __QSPI_H__

typedef enum {
	QSPI_NOLANE	= 0b00,
	QSPI_1LANE	= 0b01,
	QSPI_2LANE	= 0b10,
	QSPI_4LANE	= 0b11,
} qspiLanesT;

typedef enum {
	QSPI_8BITS	= 0b00,
	QSPI_16BITS	= 0b01,
	QSPI_24BITS	= 0b10,
	QSPI_32BITS	= 0b11,
} qspiBitLenT;

void qspi_init (void);
uint8_t *qspi_readFIFO (uint8_t *buf, size_t maxbytes);
uint32_t qspi_ccrSetCommand (uint32_t ccr, uint8_t cmd, int dmy);
uint32_t qspi_ccrSetAddrConfig (uint32_t ccr, qspiLanesT lanes, qspiBitLenT len);
uint32_t qspi_ccrSetAlternateBytesConfig (uint32_t ccr, qspiLanesT lanes, qspiBitLenT len);
int qspi_triggerRead (uint32_t ccr, uint32_t adr, size_t len, qspiLanesT datalanes);
int qspi_triggerWrite (uint32_t ccr, uint32_t adr, size_t len, qspiLanesT datalanes);
int qspi_sendData (uint32_t ccr, uint32_t adr, const uint8_t *data, size_t len, qspiLanesT datalanes);

#endif /* __QSPI_H__ */
