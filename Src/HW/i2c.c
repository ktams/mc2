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

#define TIMEOUT		20		///< 20 ms should be enougth for every operation

/**
 * Initialize the given i2c hardware to be used with standard speed (100kHz)
 * The I/O-Pins and PCLK shoud have been setup before.
 *
 * @param i2c	the IC2 hardware that should be initialized (base address of register file)
 *
 * Timing I2C: Kernel-Clock = 100MHz, Prescaler = (9+1) -> 10MHz (100ns)
 * SCLDEL 100ns * 10 -> 1µs
 * SDADEL 100ns *  5 -> 500ns
 * SCLH = 100ns * 40 -> 4µs
 * SCLL = 100ns * 50 -> 5µs
 */
void i2c_init (I2C_TypeDef *i2c)
{
	i2c->CR1 = 0;		// STOP and reset I2C
	i2c->TIMINGR = (9 << I2C_TIMINGR_PRESC_Pos) | (9 << I2C_TIMINGR_SCLDEL_Pos) | (4 << I2C_TIMINGR_SDADEL_Pos) \
			| (39 << I2C_TIMINGR_SCLH_Pos) | (49 << I2C_TIMINGR_SCLL_Pos);
	i2c->CR1 = I2C_CR1_ANFOFF | (3 << I2C_CR1_DNF_Pos);	// kein analoger filter, dafür 3 Takte digitaler Filter
	SET_BIT(i2c->CR1, I2C_CR1_PE);
}

/**
 * Read some data from a I2C device.
 *
 * @param i2c		the I2C hardware to use (should have been initialised before)
 * @param devadr	the 7 bit device address of the chip to read from
 * @param regadr	an address inside the chip, will be ignored if reglen is zero
 * @param reglen	the length of the register address (from 0 to 4 inclusive)
 * @param data		the buffer to put the data to
 * @param datalen	the amount of data to read
 * @return			0 for success or a negative error code
 *
 * Error codes: -1: Timeout, -2: NACK received, -3: parameter error
 */
int i2c_read (I2C_TypeDef *i2c, uint8_t devadr, uint32_t regadr, int reglen, uint8_t *data, int datalen)
{
	TickType_t start;

	if (!data || (datalen <= 0)) return 0;		// nothing to do
	if (!i2c || (reglen < 0) || (reglen > 4)) return -3;

	// clear all possible error flags
	i2c->ICR =   I2C_ICR_ALERTCF | I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF \
			   | I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_STOPCF | I2C_ICR_NACKCF \
			   | I2C_ICR_ADDRCF;

	start = xTaskGetTickCount();

	if (reglen > 0) {	// we must start with a write sequence and continue with repeated start
		// Start write access
		i2c->CR2 = I2C_CR2_START | ((reglen << 16) | (devadr << 1));

		while (reglen > 0) {
			while (!(i2c->ISR & I2C_ISR_TXIS)) { /* wait for TXIS to be set */
				if ((xTaskGetTickCount() - start) > TIMEOUT) return -1;
				if (i2c->ISR & I2C_ISR_NACKF) return -2;
				taskYIELD();
			}
			reglen--;
			i2c->TXDR = (regadr >> (reglen * 8)) & 0xFF;
		}

		while (!(i2c->ISR & I2C_ISR_TC)) {
			if ((xTaskGetTickCount() - start) > TIMEOUT) return -1;
			if (i2c->ISR & I2C_ISR_NACKF) return -2;
			taskYIELD();
		}
	}

	// Start read access with a START or RESTART
	i2c->CR2 = I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | (datalen << 16) | (devadr << 1);

	while (datalen > 0) {
		while ((i2c->ISR & I2C_ISR_RXNE) == 0) {
			if ((xTaskGetTickCount() - start) > TIMEOUT) {
				i2c->CR1 |= I2C_CR2_STOP;		// force a STOP condition
				return -1;
			}
			taskYIELD();
		}

		*data++ = (uint8_t) i2c->RXDR;
		datalen--;
	}

	while ((i2c->ISR & I2C_ISR_STOPF) == 0) {
		if ((xTaskGetTickCount() - start) > TIMEOUT) return -1;
		taskYIELD();
	}

	return 0;
}

/**
 * Write some data to a I2C device.
 *
 * @param i2c		the I2C hardware to use (should have been initialised before)
 * @param devadr	the 7 bit device address of the chip to write to
 * @param regadr	an address inside the chip, will be ignored if reglen is zero
 * @param reglen	the length of the register address (from 0 to 4 inclusive)
 * @param data		the buffer of the data to write
 * @param datalen	the amount of data to write
 * @return			0 for success or a negative error code
 *
 * Error codes: -1: Timeout, -2: NACK received, -3: parameter error
 */
int i2c_write (I2C_TypeDef *i2c, uint8_t devadr, uint32_t regadr, int reglen, uint8_t *data, int datalen)
{
	TickType_t start;
	int len;

	if (!data || (datalen <= 0)) return 0;		// nothing to do
	if (!i2c || (reglen < 0) || (reglen > 4)) return -3;

	len = reglen + datalen;

	// clear all possible error flags
	i2c->ICR =   I2C_ICR_ALERTCF | I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF \
			   | I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_STOPCF | I2C_ICR_NACKCF \
			   | I2C_ICR_ADDRCF;

	// Start write access
	i2c->CR2 = I2C_CR2_START | I2C_CR2_AUTOEND | ((uint32_t)(len << 16)) | ((uint32_t) devadr << 1);

	start = xTaskGetTickCount();

	while (len > 0) {
		while (!(i2c->ISR & I2C_ISR_TXIS)) { /* wait for TXIS to be set */
			if ((xTaskGetTickCount() - start) > TIMEOUT) return -1;
			if (i2c->ISR & I2C_ISR_NACKF) return -2;
			taskYIELD();
		}
		if (reglen > 0) {
			reglen--;
			i2c->TXDR = (regadr >> (reglen * 8)) & 0xFF;
		} else {
			i2c->TXDR = *data++;
		}
		len--;
	}

	while ((i2c->ISR & I2C_ISR_STOPF) == 0) {
		if ((xTaskGetTickCount() - start) > TIMEOUT) return -1;
		taskYIELD();
	}

	return 0;
}
