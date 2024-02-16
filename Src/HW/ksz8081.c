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
#include "task.h"
#include "ethernet.h"

#define MAC_SPEED		(ETH_MACMDIOAR_CR_DIV102)		// set clock ratio for MDC (HCLK / 102) => 1,96MHz

static volatile linkstate status;
static TaskHandle_t EMACtask;

static void mac_waitready (void)
{
	while ((ETH->MACMDIOAR & ETH_MACMDIOAR_MB) != 0) /* wait for MAC to be ready */ ;
}

static uint16_t mac_readRegister (uint8_t phy, uint8_t reg)
{
	mac_waitready();
	phy &= 0x1F;			// limit phy address to 5 bits
	reg &= 0x1F;			// limit register address to 5 bits
	ETH->MACMDIOAR = MAC_SPEED | (phy << ETH_MACMDIOAR_PA_Pos) | (reg << ETH_MACMDIOAR_RDA_Pos) | ETH_MACMDIOAR_MOC_RD | ETH_MACMDIOAR_MB;
	mac_waitready();
	return ETH->MACMDIODR & 0xFFFF;
}

static void mac_writeRegister (uint8_t phy, uint8_t reg, uint16_t val)
{
	mac_waitready();
	phy &= 0x1F;			// limit phy address to 5 bits
	reg &= 0x1F;			// limit register address to 5 bits
	ETH->MACMDIODR = val;
	ETH->MACMDIOAR = MAC_SPEED | (phy << ETH_MACMDIOAR_PA_Pos) | (reg << ETH_MACMDIOAR_RDA_Pos) | ETH_MACMDIOAR_MOC_WR | ETH_MACMDIOAR_MB;
	mac_waitready();
}

bool ksz8081_isup (linkstate state)
{
    switch (state) {
		case e100FDX:
		case e100HDX:
		case e10FDX:
		case e10HDX:
			return true;
		case eLINKDOWN:
			return false;
    }
    return false;
}

linkstate ksz8081_getstate (void)
{
    return status;
}

static linkstate ksz8081_map_state(uint16_t regval)
{
    if (regval & BIT(8)) {	// Bit8: Link is UP
		switch (regval & 0x07) {
			case 1: return e10HDX;
			case 2: return e100HDX;
			case 5: return e10FDX;
			case 6: return e100FDX;
		}
    }
    return eLINKDOWN;
}

/**
 * Setup EXTI11 pin (PD11) as interrupt for PHY.
 * Setup PHY to deliver interrupts as falling edge on LinkUP and LinkDOWN events
 */
static void setup_interrupt_pin (void)
{
	MODIFY_REG(SYSCFG->EXTICR[2], SYSCFG_EXTICR3_EXTI11, SYSCFG_EXTICR3_EXTI11_PD);		// Port D for external IRQ11 (-> PD11)
	CLEAR_BIT(EXTI->RTSR1, EXTI_RTSR1_TR11);	// no interrupt on rising edge
	SET_BIT(EXTI->FTSR1, EXTI_FTSR1_TR11);		// generate interrupt on falling edge
	SET_BIT(EXTI->IMR1, EXTI_IMR1_IM11);		// enable the interrupt in the interrupt mask register
	EXTI->PR1 = EXTI_PR1_PR11;					// clear pending bit by writing a 1 to the register position
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);		// clear a possibly pending interrupt request
    NVIC_SetPriority(EXTI15_10_IRQn, 14);
	NVIC_EnableIRQ(EXTI15_10_IRQn);				// enable this interrupt in NVIC
}

/**
 * Preset PHY-Chip to 100MBit/s, full duplex, autonegotiation enabled.
 * (Re-)start the autonegotiation process.
 */
void ksz8081_autonegotiation (void)
{
    mac_writeRegister(PHY_ADDR, 0x00, BIT(13) | BIT(12) | BIT(9) | BIT(8));		// preset 100MBit/FDX and enable/(re-)start autonegotiation
}

void ksz8081_setup_phy (TaskHandle_t deferredHandler)
{
    EMACtask = deferredHandler;

	mac_writeRegister(PHY_ADDR, 0x00, BIT(15) | BIT(13) | BIT(12) | BIT(8));	// reset PHY-Chip, 100MBit/s, autonegotiation, Duplex-Mode
	while (mac_readRegister(PHY_ADDR, 0) & BIT(15))	;							// wait for software reset flag to clear

	mac_writeRegister(PHY_ADDR, 0x04, BIT(8) | BIT(7) | BIT(6) | BIT(5) | 0b00001);	// we support 10/100 Tx in both full and half duplex
    mac_writeRegister(PHY_ADDR, 0x1B, BIT(10) | BIT(8));						// enable link up and down interrupts
    ksz8081_autonegotiation();

	setup_interrupt_pin();
    status = ksz8081_map_state (mac_readRegister(PHY_ADDR, 0x1E));				// initialize the local status word
    mac_readRegister(PHY_ADDR, 0x1B);											// read interrupt status to clear it
}

/**
 * Interrupt handler for EXTI11 (handles interrupts EXTI15 .. EXTI10, but
 * here we only use EXTI11).
 */
void EXTI15_10_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = 0;
    
    mac_readRegister(PHY_ADDR, 0x1B);			// read interrupt mask and status register - clears interrupt status
	EXTI->PR1 = EXTI_PR1_PR11;					// clear pending bit by writing a 1 to the register position
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);		// clear pending interrupt request

    status = ksz8081_map_state (mac_readRegister(PHY_ADDR, 0x1E));	// update the local status word
    vTaskNotifyGiveFromISR (EMACtask, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}

