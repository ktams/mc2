/**
 * @file setup.c
 *
 * @author Andi
 * @date   01.05.2019
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

#include "rb2.h"
#include "defaults.h"

#define DMA_RAMBASE					D2_AHBSRAM_BASE
#define	WATCHDOG_EARLIEST_RESET		0x60	///< see the WWDG1 peripheral, W[6:0] in WWDG_CFR

struct cpu_info cpu;	///< the global struct that hosts the CPU information

static void mpu_init (void)
{
	int rnr;

	SCB->ITCMCR = SCB_ITCMCR_EN_Msk;
	SCB->DTCMCR = SCB_DTCMCR_EN_Msk;

	__ISB();
	__DSB();
	// define region 0 as 4kb device memory / full access @ SRAM2_BASE

	MPU->RBAR = (DMA_RAMBASE & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (0 << MPU_RBAR_REGION_Pos);				// set MPU region 0 base address
	MPU->RASR = MPU_RASR_XN_Msk | (0b011 << MPU_RASR_AP_Pos) | MPU_RASR_B_Msk | (11 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

	// disable all other regions (just in case ...)
	for (rnr = 1; rnr < 16; rnr++) {
		MPU->RNR = (uint32_t) rnr;
		MPU->RBAR = 0;
		MPU->RASR = 0;
	}

	// now enable MPU with a global background region
	MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
	__DSB();
}

/**
 * Configure System to run at 400MHz with 2 wait states for flash access.
 *
 * Calculation:
 *    HSI = 64MHz (output divider is 1 / bypass), also used by ADC
 *    PLL1 is used with an input divider (DIVM) of 4 (= 16MHz), multiplicator (DIVN) of 50 (= 800MHz)
 *		- P-Output: post divider 2 -> 400MHz (used as system and derived peripheral clocks for most units)
 *		- Q-Output: post divider 100 -> 8MHz (used for Märklin CAN and EasyNet via SPI1 125kBit/s or 62,5kBit/s)
 *    PLL2 is used with an input divider (DIVM) of 32, multiplicator (DIVN) of 80
 *		- Q-Ouput: post divider 20 -> 8MHz (used for EasyNet SPI 125kBit/s or 62,5kBit/s !!OLD!!)
 *    PLL3 shut off
 *
 *    (64MHz / 1) / 4 = 16MHz PLL1 input (range is selected as 8-16MHz)
 *    (16MHz * 50) / 2 = 400MHz PLL1p output frequency (pll1_p_ck)
 *    (16MHz * 50) / 100 = 8MHz PLL1q output frequency (pll1_q_ck) - SPI1 (EasyNet), Märklin CAN (250kBit/s)
 *    (64MHz / 1) / 32 = 2MHz PLL2 input (range is selected as 1-2MHz)
 *    (2MHz * 80) / 20 = 8MHz PLL2q output frequency (pll2_q_ck) - OLD: used for EasyNet SPI6 (125kBit/s or 62,5kBit/s)
 */
static void clock_init (void)
{
	uint32_t dummy, pllcfgr;

	RCC->CR = RCC_CR_HSION;					// this is the reset configuration with HSI at 64MHz and all PLLs switched off
	while (!(RCC->CR & RCC_CR_HSIRDY)) ;	// wait until it is stable

	// PLL1 receives a clock signal (HSI / 4) = 64MHz / 4 = 16MHz, PLL2 (HSI / 32) = 2MHz
	RCC->PLLCKSELR = (32 << RCC_PLLCKSELR_DIVM2_Pos) | (4 << RCC_PLLCKSELR_DIVM1_Pos);
	// select PLL1 DIVN = 50, DIVP1 = 2 and DIVQ1 = 100
	RCC->PLL1DIVR = (99 << RCC_PLL1DIVR_Q1_Pos) | (1 << RCC_PLL1DIVR_P1_Pos) | (49 << RCC_PLL1DIVR_N1_Pos);
	RCC->PLL1FRACR = 0;							// be sure to have no fractional divider
	// select PLL2 DIVN = 80, DIVQ2 = 20
	RCC->PLL2DIVR = (19 << RCC_PLL2DIVR_Q2_Pos) | (79 << RCC_PLL2DIVR_N2_Pos);
	RCC->PLL2FRACR = 0;							// be sure to have no fractional divider
	// select 8-16MHz input range for PLL1 (Range3) with wide VCO range (192 - 836MHz)
	// PLL1 integer mode, enable P- and Q-Output
	pllcfgr = RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVP1EN | (3 << RCC_PLLCFGR_PLL1RGE_Pos);
	// select 1-2MHz input range for PLL2 (Range0) with medium VCO range (150 - 420MHz)
	// PLL2 integer mode, enable Q-Output
	pllcfgr |= RCC_PLLCFGR_DIVQ2EN | (0 << RCC_PLLCFGR_PLL2RGE_Pos) | RCC_PLLCFGR_PLL2VCOSEL;
	RCC->PLLCFGR = pllcfgr;

	RCC->CR |= RCC_CR_PLL2ON | RCC_CR_PLL1ON;
	while ((RCC->CR & (RCC_CR_PLL2RDY | RCC_CR_PLL1RDY)) != (RCC_CR_PLL2RDY | RCC_CR_PLL1RDY)) ;	// wait for PLL1 and PLL2 to be ready

	// FLASH: programming delay 0b10 (185MHz - 210MHz @VOS1), 2 Waitstates
#define FLASH_ACR_VALUE		((0b10 << FLASH_ACR_WRHIGHFREQ_Pos) | FLASH_ACR_LATENCY_2WS)
	do {	// according to data sheet: check back that the register contains correct values
		FLASH->ACR = FLASH_ACR_VALUE;
		dummy = FLASH->ACR;
	} while (dummy != FLASH_ACR_VALUE);

	// sysclk prescaler is 1 (400 MHz to CPU and main bus divider, D1CPRE is 0)
	// the main bus divider is 2 (200MHz to peripheral busses, HPRE set to DIV2)
	// APB3 is again divided by 2 (100MHz, D1PPRE set to DIV2)
	RCC->D1CFGR = RCC_D1CFGR_D1PPRE_DIV2 | RCC_D1CFGR_HPRE_DIV2;

	// APB1 and APB2 both run at 100MHz (divided by 2 from main bus divider)
	// APB1 and APB2 timers will run at 2x this speed (200MHz)
	RCC->D2CFGR = RCC_D2CFGR_D2PPRE2_DIV2 | RCC_D2CFGR_D2PPRE1_DIV2;

	// APB4 runs at 100MHz (divided by 2 from main bus divider)
	RCC->D3CFGR = RCC_D3CFGR_D3PPRE_DIV2;

	// now switch sysclock to PLL and wait until switch is done
	// only the SW[2:0] bits need to be set, all others can rest at zero as is the default after power up
	RCC->CFGR = RCC_CFGR_SW_PLL1;
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL1) ;

	SystemCoreClockUpdate();

	SysTick->CTRL  = 0;		// currently switch off SysTick timer

	// enable L1 I- and D-Cache
	SCB_EnableICache();
#ifdef USE_CACHE
	SCB_EnableDCache();
#endif
}

/**
 * Enable the clocks to the peripherals in use
 *   - FMC (SDRAM)
 *   - Ethernet (RMII)
 *   - SYSCFG
 *   - SRAM1, SRAM2, SRAM3
 *   - TIM1, TIM2, TIM3, TIM7, TIM8, TIM12, TIM13, TIM15, TIM16, TIM17
 *   - DAC, ADC
 *   - SPI1
 *   - I2C4
 *   - QUADSPI
 *   - USART1, USART2, UART5, USART6, LPUART1
 *   - FDCAN1 (RxD and TxD are swapped in HW0.7, so do not enable this module for first prototype boards!)
 *	 - GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH (GPIO I, J and K are not available in this package)
 *
 * After each register write, there should be a dummy read of that register.
 */
static void pclk_init (void)
{
	uint32_t dummy;

	RCC->AHB1ENR	|= RCC_AHB1ENR_ETH1MACEN | RCC_AHB1ENR_ETH1RXEN | RCC_AHB1ENR_ETH1TXEN | RCC_AHB1ENR_DMA2EN \
					|  RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_ADC12EN;
	dummy = RCC->AHB1ENR;
	(void) dummy;

	RCC->AHB2ENR	|= RCC_AHB2ENR_D2SRAM1EN | RCC_AHB2ENR_D2SRAM2EN | RCC_AHB2ENR_D2SRAM3EN;
	dummy = RCC->AHB2ENR;
	(void) dummy;

	RCC->AHB3ENR	|= RCC_AHB3ENR_FMCEN | RCC_AHB3ENR_QSPIEN;
	dummy = RCC->AHB3ENR;
	(void) dummy;

	RCC->AHB4ENR	|= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN \
					|  RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOFEN | RCC_AHB4ENR_GPIOGEN | RCC_AHB4ENR_GPIOHEN \
					|  RCC_AHB4ENR_ADC3EN;
	dummy = RCC->AHB4ENR;
	(void) dummy;

	RCC->APB1LENR	|= RCC_APB1LENR_DAC12EN | RCC_APB1LENR_UART5EN | RCC_APB1LENR_USART2EN | RCC_APB1LENR_TIM13EN \
					|  RCC_APB1LENR_TIM12EN | RCC_APB1LENR_TIM7EN | RCC_APB1LENR_TIM4EN | RCC_APB1LENR_TIM3EN | RCC_APB1LENR_TIM2EN;
	dummy = RCC->APB1LENR;
	(void) dummy;

#ifndef HW_REV07
	RCC->APB1HENR	|= RCC_APB1HENR_FDCANEN;
	dummy = RCC->APB1HENR;
	(void) dummy;
#endif

	RCC->APB2ENR	|= RCC_APB2ENR_TIM17EN | RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM15EN | RCC_APB2ENR_SPI1EN \
					|  RCC_APB2ENR_USART6EN | RCC_APB2ENR_USART1EN |  RCC_APB2ENR_TIM8EN | RCC_APB2ENR_TIM1EN;
	dummy = RCC->APB2ENR;
	(void) dummy;

#ifdef EASYNET_USE_SPI1
	RCC->APB4ENR	|= RCC_APB4ENR_VREFEN | RCC_APB4ENR_I2C4EN | RCC_APB4ENR_LPUART1EN | RCC_APB4ENR_SYSCFGEN;
#else
	RCC->APB4ENR	|= RCC_APB4ENR_VREFEN | RCC_APB4ENR_I2C4EN | RCC_APB4ENR_SPI6EN | RCC_APB4ENR_LPUART1EN | RCC_APB4ENR_SYSCFGEN;
#endif
	dummy = RCC->APB4ENR;
	(void) dummy;

	//@TODO: check, if we can forget about it, because we use the HCLK directly as conversion clock source
	// use hsi_ker_ck as per_ck (according to clock_init() this is HSI / 1 = 64MHz)
	MODIFY_REG(RCC->D1CCIPR, RCC_D1CCIPR_CKPERSEL_Msk, 0b00 << RCC_D1CCIPR_CKPERSEL_Pos);
	// use pll1_q_ck as kernel clock for FDCAN (i.e. 8MHz)
	MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_FDCANSEL_Msk, 0b01 << RCC_D2CCIP1R_FDCANSEL_Pos);
	// use per_ck as kernel clock for ADCs (i.e. 64MHz)
	MODIFY_REG(RCC->D3CCIPR, RCC_D3CCIPR_ADCSEL_Msk, 0b10 << RCC_D3CCIPR_ADCSEL_Pos);
#ifdef EASYNET_USE_SPI1
	// use pll1_q_ck as kernel clock for SPI1 (i.e. 8MHz, is already the default setting)
	MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_SPI123SEL_Msk, 0b000 << RCC_D2CCIP1R_SPI123SEL_Pos);
#else
	// use pll2_q_ck as kernel clock for SPI6 (i.e. 8MHz)
	MODIFY_REG(RCC->D3CCIPR, RCC_D3CCIPR_SPI6SEL_Msk, 0b001 << RCC_D3CCIPR_SPI6SEL_Pos);
#endif
}

/**
 * Initialisation of all I/O-Pins.
 * Pins can be:
 *   - input, output, alternate function (AFxx) or analog
 *   - push/pull or open drain (od)
 *   - output speed: low (ls), medium (ms), high (hs) or very high (vs)
 *   - pull up (pu), pull down (pd) or none of these (the equivalent value of the resistor is typical 40kOhm)
 *   - in alternate function mode, the funtion must be selected in AFRL (AFR[0]) or AFRH (AFR[1])
 *   - when used as general purpose output, the initial output polarity is specified as H or L
 * If nothing special is written, then the pin is configured as
 *	push/pull, low speed, no pull up/down
 *	(alternate funtion setting is ignored and programmed as AF00).
 *
 * The output speed is dependant on the supply voltage and capacitive load.
 * Values are @3,3V with load of 50pF, 30pF and 10pF (in MHz):
 *   - ls: 12, 12, 16
 *   - ms: 60, 80, 110
 *   - hs: 85, 110, 166
 *   - vs: 100, 133, 220
 * This leads to the decision, that only RAM will need vs, ethernet and QSPI will get hs
 * and the rest is probably satisfied with ls.
 */
static void gpio_init (void)
{
	/*
	 * GPIOA
	 * PA00: ISENSE_AD (analog)
	 * PA01: ENET-REFCLK (AF11, hs)
	 * PA02: ENET-MDIO (AF11)
	 * PA03: M3-IN (AF01, pu)
	 * PA04: TRACK_VOLTAGE (analog output)
	 * PA05: UIN_AD (analog)
	 * PA06: S88-DATA (input, pu) -> HW1.1: EASYNET-MISO (AF05)
	 * PA07: ENET-RXDV (AF11, hs)
	 * PA08: RC_CUTOUT (AF01)
	 * PA09: PH1 (AF01)
	 * PA10: PH3 (AF01)
	 * PA11: DCC-SIGNAL (AF01)
	 * PA12: XPRESSNET-DE (AF07)
	 * PA13: SWDIO (AF00, vs)
	 * PA14: SWCLK (AF00, pd)
	 * PA15: /EASYNET-SS (output, H) (alternative/currently used: AF07 for HW-controlled /SS -> HW1.1: AF05)
	 */										// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
#ifdef EASYNET_USE_SPI1
	GPIOA->MODER	= 0xAAAAAFAB;			// 10 10 10 10 10 10 10 10 10 10 11 11 10 10 10 11
#else
	GPIOA->MODER	= 0xAAAA8FAB;			// 10 10 10 10 10 10 10 10 10 00 11 11 10 10 10 11
#endif
//	GPIOA->OTYPER	= 0b0000000000000000;
	GPIOA->OSPEEDR	= 0x0C008008;			// 00 00 11 00 00 00 00 00 10 00 00 00 00 00 10 00
#ifdef EASYNET_USE_SPI1
	GPIOA->PUPDR	= 0x20000040;			// 00 10 00 00 00 00 00 00 00 00 00 00 01 00 00 00
	GPIOA->AFR[0]	= 0xB5001BB0;
	GPIOA->AFR[1]	= 0x50071111;
#else
	GPIOA->PUPDR	= 0x20001040;			// 00 10 00 00 00 00 00 00 00 01 00 00 01 00 00 00
	GPIOA->AFR[0]	= 0xB0001BB0;
	GPIOA->AFR[1]	= 0x70071111;
#endif
	GPIOA->ODR		= 0b1000000000000000;

	/* GPIOB
	 * PB00: SIG_5POL (AF02)			=> was PH2 (output, L) in HW0.7
	 * PB01: DCC-SHORT (input)			=> was PH4 (output, L) in HW0.7
	 * PB02: ETH_LED (output, L)
	 * PB03: SNIFFER (AF01)
	 * PB04: S88-PS (output, L)
	 * PB05: EASYNET-MOSI (AF08)		=> HW1.1: SPI1 instear of SPI6, AF05
	 * PB06: /LOCONET_TXD (AF08)
	 * PB07: /LOCONET_RXD (AF08)
	 * PB08: CAN_RXD (AF09)				=> was swapped with CAN_TXD and therefor ignored (input, pd) in HW0.7
	 * PB09: CAN_TXD (AF09)				=> was swapped with CAN_RXD and therefor ignored (input, pd) in HW0.7
	 * PB10: MBST-ON (output, L)		=> was unused pin (input, pd) in HW0.7
	 * PB11: ENET-TXEN (AF11, hs)
	 * PB12: ENET-TXD0 (AF11, hs)
	 * PB13: ENET-TXD1 (AF11, hs)
	 * PB14: XPRESSNET-RXTX (AF04, od)
	 * PB15: RGB_LEDS (AF02)
	 */										// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
#ifdef HW_REV07
	GPIOB->MODER	= 0xAA80A995;			// 10 10 10 10 10 00 00 00 10 10 10 01 10 01 01 01
#else
	GPIOB->MODER	= 0xAA9AA992;			// 10 10 10 10 10 01 10 10 10 10 10 01 10 01 00 10
#endif
	GPIOB->OTYPER	= 0b0100000000000000;	//    od
	GPIOB->OSPEEDR	= 0x0A800000;			// 00 00 10 10 10 00 00 00 00 00 00 00 00 00 00 00
#ifdef HW_REV07
	GPIOB->PUPDR	= 0x002A0000;			// 00 00 00 00 00 10 10 10 00 00 00 00 00 00 00 00
#else
	GPIOB->PUPDR	= 0x00000000;			// 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#endif
#ifdef EASYNET_USE_SPI1
	GPIOB->AFR[0]	= 0x88501002;
#else
	GPIOB->AFR[0]	= 0x88801002;
#endif
	GPIOB->AFR[1]	= 0x24BBBB99;
	GPIOB->ODR		= 0b0000000000000000;

	/* GPIOC
	 * PC00: /SDRAM_WE (AF12, vs)
	 * PC01: ENET-MDC (AF11)
	 * PC02: /SDRAM_E0 (AF12, vs)
	 * PC03: SDRAM_CKE0 (AF12, vs)
	 * PC04: ENET-RXD0 (AF11, hs)
	 * PC05: ENET-RXD1 (AF11, hs)
	 * PC06: SIG_CDE (AF03)				=> was DCC-SHORT (input) in HW0.7
	 * PC07: BST-ON (output, L)			=> was MBST-ON (output, L) in HW0.7
	 * PC08: 7SEG_A (output, L)
	 * PC09: 7SEG_B (output, L)
	 * PC10: 7SEG_C (output, L)
	 * PC11: 7SEG_D (output, L)
	 * PC12: 7SEG_E (output, L)
	 * PC13: 7SEG_F (output, L)
	 * PC14: 7SEG_G (output, L)
	 * PC15: 7SEG_DP (output, L)
	 */										// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
#ifdef HW_REV07
	GPIOC->MODER	= 0x55554AAA;			// 01 01 01 01 01 01 01 01 01 00 10 10 10 10 10 10
#else
	GPIOC->MODER	= 0x55556AAA;			// 01 01 01 01 01 01 01 01 01 10 10 10 10 10 10 10
#endif
//	GPIOC->OTYPER	= 0b0000000000000000;
	GPIOC->OSPEEDR	= 0x00000AF3;			// 00 00 00 00 00 00 00 00 00 00 10 10 11 11 00 11
//	GPIOC->PUPDR	= 0x00000000;
#ifdef HW_REV07
	GPIOC->AFR[0]	= 0x00BBCCBC;
	GPIOC->AFR[1]	= 0x00000000;
#else
	GPIOC->AFR[0]	= 0x03BBCCBC;
	GPIOC->AFR[1]	= 0x00000000;
#endif
	GPIOC->ODR		= 0b0000000000000000;

	/* GPIOD
	 * PD00: SDRAM_D2 (AF12, vs)
	 * PD01: SDRAM_D3 (AF12, vs)
	 * PD02: RC_DATA (AF08)
	 * PD03: S88-RESET (output, L)
	 * PD04: BIDI-DE (AF07)
	 * PD05: BIDI-RXTX (AF07)
	 * PD06: /ACK_IN (input)
	 * PD07: ACK_out (output, L)
	 * PD08: SDRAM_D13 (AF12, vs)
	 * PD09: SDRAM_D14 (AF12, vs)
	 * PD10: SDRAM_D15 (AF12, vs)
	 * PD11: MAC_INTR (input)
	 * PD12: I2C4_SCL (AF04, od)
	 * PD13: I2C4_SDA (AF04, od)
	 * PD14: SDRAM_D0 (AF12, vs)
	 * PD15: SDRAM_D1 (AF12, vs)
	 */										// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	GPIOD->MODER	= 0xAA2A4A6A;			// 10 10 10 10 00 10 10 10 01 00 10 10 01 10 10 10
	GPIOD->OTYPER	= 0b0011000000000000;	//       od od
	GPIOD->OSPEEDR	= 0xF03F000F;			// 11 11 00 00 00 11 11 11 00 00 00 00 00 00 11 11
	GPIOD->PUPDR	= 0x05000000;			//       pu pu											$$$ I2C4 hat in HW REV0.7 keine externen pullups!
	GPIOD->AFR[0]	= 0x007708CC;
	GPIOD->AFR[1]	= 0xCC440CCC;
	GPIOD->ODR		= 0b0000000000000000;

	/* GPIOE
	 * PE00: SDRAM_BL0 (AF12, vs)
	 * PE01: SDRAM_BL1 (AF12, vs)
	 * PE02: 7SEG_A1 (output, L)
	 * PE03: 7SEG_A2 (output, L)
	 * PE04: ESP-RST (output, H)
	 * PE05: AUDIO_L (AF04)
	 * PE06: AUDIO_R (AF04)
	 * PE07: SDRAM_D4 (AF12, vs)
	 * PE08: SDRAM_D5 (AF12, vs)
	 * PE09: SDRAM_D6 (AF12, vs)
	 * PE10: SDRAM_D7 (AF12, vs)
	 * PE11: SDRAM_D8 (AF12, vs)
	 * PE12: SDRAM_D9 (AF12, vs)
	 * PE13: SDRAM_D10 (AF12, vs)
	 * PE14: SDRAM_D11 (AF12, vs)
	 * PE15: SDRAM_D12 (AF12, vs)
	 */										// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	GPIOE->MODER	= 0xAAAAA95A;			// 10 10 10 10 10 10 10 10 10 10 10 01 01 01 10 10
//	GPIOE->OTYPER	= 0b0000000000000000;
	GPIOE->OSPEEDR	= 0xFFFFC00F;			// 11 11 11 11 11 11 11 11 11 00 00 00 00 00 11 11
//	GPIOE->PUPDR	= 0x00000000;
	GPIOE->AFR[0]	= 0xC44000CC;
	GPIOE->AFR[1]	= 0xCCCCCCCC;
	GPIOE->ODR		= 0b0000000000010000;

	/* GPIOF
	 * PF00: SDRAM_A0 (AF12, vs)
	 * PF01: SDRAM_A1 (AF12, vs)
	 * PF02: SDRAM_A2 (AF12, vs)
	 * PF03: SDRAM_A3 (AF12, vs)
	 * PF04: SDRAM_A4 (AF12, vs)
	 * PF05: SDRAM_A5 (AF12, vs)
	 * PF06: QSPI_IO3 (AF09, hs)
	 * PF07: QSPI_IO2 (AF09, hs)
	 * PF08: QSPI_IO0 (AF10, hs)
	 * PF09: QSPI_IO1 (AF10, hs)
	 * PF10: QSPI_SCK (AF09, hs)
	 * PF11: /SDRAM_RAS (AF12, vs)
	 * PF12: SDRAM_A6 (AF12, vs)
	 * PF13: SDRAM_A7 (AF12, vs)
	 * PF14: SDRAM_A8 (AF12, vs)
	 * PF15: SDRAM_A9 (AF12, vs)
	 */										// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	GPIOF->MODER	= 0xAAAAAAAA;			// 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10
//	GPIOF->OTYPER	= 0b0000000000000000;
	GPIOF->OSPEEDR	= 0xFFEAAFFF;			// 11 11 11 11 11 10 10 10 10 10 11 11 11 11 11 11
//	GPIOF->PUPDR	= 0x00000000;
	GPIOF->AFR[0]	= 0x99CCCCCC;
	GPIOF->AFR[1]	= 0xCCCCC9AA;
	GPIOF->ODR		= 0b0000000000000000;

	/* GPIOG
	 * PG00: SDRAM_A10 (AF12, vs)
	 * PG01: SDRAM_A11 (AF12, vs)
	 * PG02: SDRAM_A12 (AF12, vs)
	 * PG03: MBST-SHORT (input)
	 * PG04: SDRAM_BA0 (AF12, vs)
	 * PG05: SDRAM_BA1 (AF12, vs)
	 * PG06: /QSPI_CS (AF10, hs)
	 * PG07: PROG_RELAIS (output, low)
	 * PG08: SDRAM_CLK (AF12, vs)
	 * PG09: ESP-RX (AF07)			// in HW1.0 -> should become TX!
	 * PG10: ESP-GP0 (output, pu, od, high)
	 * PG11: S88-CLK (output, low)	// in HW1.1: EASYNET-SCK (AF05)
	 * PG12: EASYNET-MISO (AF05)	// in HW1.1: s88-DATA (input)
	 * PG13: EASYNET-SCK (AF05)		// in HW1.1: s88-CLK (output, low)
	 * PG14: ESP-TX (AF07)			// in HW1.0 -> should become RX!
	 * PG15: /SDRAM_CAS (AF12, vs)
	 */										// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
#ifdef EASYNET_USE_SPI1
	GPIOG->MODER	= 0xA49A6A2A;			// 10 10 01 00 10 01 10 10 01 10 10 10 00 10 10 10
#else
	GPIOG->MODER	= 0xAA5A6A2A;			// 10 10 10 10 01 01 10 10 01 10 10 10 00 10 10 10
#endif
	GPIOG->OTYPER	= 0b0000010000000000;	//                od
	GPIOG->OSPEEDR	= 0xC0032F3F;			// 11 00 00 00 00 00 00 11 00 10 11 11 00 11 11 11
	GPIOG->PUPDR	= 0x00100000;			//                pu
	GPIOG->AFR[0]	= 0x0ACC0CCC;
#ifdef EASYNET_USE_SPI1
	GPIOG->AFR[1]	= 0xC700507C;
#else
	GPIOG->AFR[1]	= 0xC755007C;
#endif
	GPIOG->ODR		= 0b0000010000000000;

	/* GPIOH
	 * PH00: TASTER2 (input, pu)
	 * PH01: TASTER1 (input, pu)
	 *
	 * No further IOs are brought out to pins on this package,
	 * so AFRH and ODR is not needed.
	 */										// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	GPIOH->MODER	= 0xFFFFFFF0;			// 11 11 11 11 11 11 11 11 11 11 11 11 11 11 00 00
//	GPIOH->OTYPER	= 0b0000000000000000;
	GPIOH->OSPEEDR	= 0x00000000;			// 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
	GPIOH->PUPDR	= 0x00000005;			// 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 01
//	GPIOH->AFR[0]	= 0x00000000;
}

static void nvic_init (void)
{
	unsigned i;

	for (i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;			// disable all interrupts
		NVIC->ICPR[i] = 0xFFFFFFFF;			// clear all pending flags
	}

	for (i = 0; i < sizeof(NVIC->IP); i++) {
		NVIC->IP[i] = 0xFF;					// set all interrupt priorities to 0xFF (priority 15 - lowest priority possible)
	}

	/*
	 * the priority grouping defines, where the split between priority group and -subgroup is.
	 * 0: 1 LSB is used as subpriority, the 7 MSBs are the real priority.
	 * 1: 2 LSBs are used as subpriority, the 6 MSBs are the real priority.
	 * ...
	 * 7: 8 LSB are used as subpriority and all interrupts share a single main priority.
	 *
	 * The STM32H7xx implements only the four MSBs and we use it to have 16 real priorities
	 * from 0 (highest priority) to 15 (lowest priority).
	 * FreeRTOS can be configured to allow system calls up to a certain level (i.e. down
	 * to certain priority) above which the interrupts are not allowed to call FreeRTOS-
	 * functions.
	 *
	 * Currently, the FreeRTOSConfig.h defines the configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
	 * to 4 which means, that priorities 0 to 3 are not allowed to call FreeRTOS functions.
	 * If you set a Priority using NVIC_SetPriority() you can ignore the shifting and everything
	 * else, becase this macro already takey care of it. So just use priority values between
	 * 15 (lowest priority) and 0 (highest priority).
	 */
	NVIC_SetPriorityGrouping(0);	// all bits in the priority grouping define different priority levels (with 0 = highest priority)
}

/**
 * RMII uses:
 *  - PA1	REFCLK in
 *  - PA2	MDIO
 *  - PA7	CRS_DV
 *  - PC1	MDC
 *  - PC4	RXD0
 *  - PC5	RXD1
 *  - PB10	RXER	???? das Signal gibt es eigentlich im RMII nicht -> Input ????
 *  - PB11	TX_EN
 *  - PB12	TXD0
 *  - PB13	TXD1
 */
static void rmii_init (void)
{
	SYSCFG->PMCR |= (0b100 << SYSCFG_PMCR_EPIS_SEL_Pos);
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	ETH->MACMDIOAR = (0b100 << ETH_MACMDIOAR_CR_Pos);		// set clock ratio for MDC (HCLK / 102) => 1,96MHz
}

static void dac_init (void)
{
	DAC1->CR = 0;			// switch off both DAC channels
	DAC1->MCR = 0;			// select Mode=0b000 -> DAC connected only to external pin using Buffer, normal mode
	DAC1->DHR12R1 = 0;		// shut output down to 0V
	DAC1->CR = DAC_CR_EN1;
}

static char map_revID (uint32_t idc)
{
	switch (idc >> 16) {
		case 0x1001: return 'Z';
		case 0x1003: return 'Y';
		case 0x2001: return 'X';
		case 0x2003: return 'V';
	}
	return '?';
}

static void cpu_getinfo (void)
{
	cpu.cpuid = SCB->CPUID;
	cpu.idcode = DBGMCU->IDCODE;
	cpu.r = (cpu.cpuid >> 20) & 0x0F;
	cpu.p = cpu.cpuid & 0x0F;
	cpu.revcode = map_revID(cpu.idcode);
}

/**
 * Setup the windowed watchdog.
 * Since it is clocked from the APB-Clock, the clock source is 100MHz.
 */
static void wwdg_setup (void)
{
	RCC->GCR |= RCC_GCR_WW1RSC;				// WWDG1 reset scope control
	RCC->APB3ENR |= RCC_APB3ENR_WWDG1EN;	// enable clock to WWDG1
	WWDG1->CR = 0x7F;						// preset to highest possible value without setting the (non-resettable) activation bit
	// Timeout: 100MHz / 4096 / 32 = 763 Hz clocking
	// coming from counter value of 0x7F down to 0x3F is 0x40 (64) counts = 83,9ms
	// the watchdog reset is earliest done when the downcounter reached 0x60 (WATCHDOG_EARLIEST_RESET)
	// this means, almost 21ms must pass, before the watchdog may be reset.
	WWDG1->CFR = (0b101 << WWDG_CFR_WDGTB_Pos) | 0x7F;						// in the first step, enable presetting the counter at any time
	WWDG1->CR = WWDG_CR_WDGA | 0x7F;		// enable the watchdog by setting it to highest counter value and enable it.
	WWDG1->CFR = (0b101 << WWDG_CFR_WDGTB_Pos) | WATCHDOG_EARLIEST_RESET;	// as a second step, we reduce the presetting to a shorter period
}

void hw_setup (void)
{
	PWR->CR3 = PWR_CR3_LDOEN;					// set the one-time settable power supply to LDO
	PWR->D3CR = 0b11 << PWR_D3CR_VOS_Pos;		// switch to VOS1
	while (!(PWR->D3CR & PWR_D3CR_VOSRDY)) ;	// wait for VOS-switch to be done

	mpu_init();			// handle the MPU settings for certain DMA buffers
	pclk_init();		// enable the peripheral clocks
	clock_init();		// updates SystemCoreClock after setting up PLL for system clock

	cpu_getinfo();

	gpio_init();		// set I/O-Modes of all processor pins
	nvic_init();		// preset the NVIC controller
	rmii_init();		// setup RMII for Ethernet
	sdram_init();		// setup SDRAM
	dac_init();			// initialize DAC output
	wwdg_setup();		// start watchdog timer
	i2c_init(I2C4);

	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
}
