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

// mode commands for SDCMR (SD Command Mode Register)
#define SDRAM_NORMAL_MODE			0b000	///< Normal Mode
#define SDRAM_CLOCK_CONFIG_ENABLE	0b001	///< Clock Configuration enable
#define SDRAM_PALL					0b010	///< PALL (All Bank Precharge") command
#define SDRAM_AUTO_REFFRESH			0b011	///< Auto-refresh command
#define SDRAM_LOAD_MODE_REGISTER	0b100	///< Load Mode Register
#define SDRAM_SELF_REFRESH			0b101	///< Self-refresh command
#define SDRAM_POWER_DOWN			0b110	///< Power-down command
#define SDRAM_RESERVED_CMD			0b111	///< (reserved)

static void sdram_mode (int mrd, int nrfs, int mode, bool tb1, bool tb2)
{
	FMC_Bank5_6_TypeDef *sd = FMC_Bank5_6_R;

	sd->SDCMR = (mrd << FMC_SDCMR_MRD_Pos) | (nrfs << FMC_SDCMR_NRFS_Pos) \
			| ((tb1) ? FMC_SDCMR_CTB1 : 0) | ((tb2) ? FMC_SDCMR_CTB2 : 0) | (mode << FMC_SDCMR_MODE_Pos);
}

int sdram_init (void)
{
	FMC_Bank5_6_TypeDef *sd;
	int mrd, i;

//	RCC->AHB3RSTR = RCC_AHB3RSTR_FMCRST;
//	__NOP();
//	__NOP();
//	__NOP();
//	__NOP();
//	RCC->AHB3RSTR = 0;

	// disable FMC controller to update relevant settings
	// swap SDRAM with NOR/PSRAM -> SDRAM11 is accessible at 0x60000000
	MODIFY_REG(FMC_Bank1_R->BTCR[0], FMC_BCR1_FMCEN | FMC_BCR1_BMAP_Msk, (0b01 << FMC_BCR1_BMAP_Pos));

	sd = FMC_Bank5_6_R;
	/*
	 * SDRAM Control register 1
	 * ---------------------------------
	 * Read Pipe: No HCLK-Delay
	 * single read requests as bursts
	 * SDCLK = HCLK / 2 (100MHz)
	 * CAS-Latency = 2Clk
	 * four internal Banks
	 * bus width 16bits
	 * 12bit row addressing
	 * 8bit column addressing
	 */
	sd->SDCR[0] = /* FMC_SDCR1_RBURST | */ (0b10 << FMC_SDCRx_SDCLK_Pos) | (0b10 << FMC_SDCRx_CAS_Pos) | FMC_SDCRx_NB \
			| (0b01 << FMC_SDCRx_MWID_Pos) | (0b01 << FMC_SDCRx_NR_Pos) | (0b00 << FMC_SDCRx_NC_Pos);
	/*
	 * SDRAM Timing register 1 (timing = 0 => 1CK, 1 => 2CK, ...)
	 * ---------------------------------
	 * Trcd = 2CK
	 * Trp = 2CK
	 * Twr = 2CK
	 * Trc = 6CK
	 * Tras = 4CK	-> ACTIVE-to-PRECHARGE command
	 * Txsr = 6CK
	 * Tmrd = 2CK
	 */
	sd->SDTR[0] = (1 << FMC_SDTRx_TRCD_Pos) | (1 << FMC_SDTRx_TRP_Pos) | (1 << FMC_SDTRx_TWR_Pos) | (5 << FMC_SDTRx_TRC_Pos) \
			 | (3 << FMC_SDTRx_TRAS_Pos) | (5 << FMC_SDTRx_TXSR_Pos) | (1 << FMC_SDTRx_TMRD_Pos);

	SET_BIT(FMC_Bank1_R->BTCR[0], FMC_BCR1_FMCEN);	// enable the FMC

	sdram_mode(0, 0, SDRAM_CLOCK_CONFIG_ENABLE, true, false);

	/* delay 100us */
	for (i = 0; i < 100000; i++) {
		__NOP();
	}
//	printf("%s(): delay finished\n", __func__);

	sdram_mode(0, 0, SDRAM_PALL, true, false);
	sdram_mode(0, 7, SDRAM_AUTO_REFFRESH, true, false);		// eigentlich sollten 2x Autorefresh genügen ... (1 als Parameter übergeben)

	/*
	 * program SD-RAM Mode register:
	 * Burst length = 4		!! according to reference manual of STM32F745 it should be 1, example uses 4!!
	 * Burst type = sequential
	 * CAS latency = 2
	 * Standard operation
	 * Write burst mode = Programmed burst length
	 */
	mrd = (0b0 << 9) | (0b00 << 7) | (0b010 << 4) | (0b0 << 3) | (0b000 << 0);
	sdram_mode(mrd, 0, SDRAM_LOAD_MODE_REGISTER, true, false);

	sdram_mode(0, 0, SDRAM_NORMAL_MODE, true, false);

	sd->SDRTR = (1292 << FMC_SDRTR_COUNT_Pos);
	sd->SDCR[0] &= ~FMC_SDCRx_WP;

//	for (int i = 0; i < 100; i++) sdram_test();
	return 0;
}

#if 0
#define SDRAM_INTS	(2*1024*1024)

int sdram_test (void)
{
	int *p, i, errors;

	p = (int *) SDRAM_BASE;		// begin of SDRAM Bank 1
	for (i = 0; i < SDRAM_INTS; i++) {
		*p++ = i;
	}
	p = (int *) SDRAM_BASE;		// begin of SDRAM Bank 1
	errors = 0;
	for (i = 0; i < SDRAM_INTS && errors < 20; i++) {
		if (*p != i) {
//			fprintf(stderr, "%s(): SDRAM error at %p (expecting 0x%08x reading 0x%08x)\n", __func__, p, i, *p);
			errors++;
		}
		p++;
	}

//	printf("%s(): SDRAM-Test with %d errors%s\n", __func__, errors, (errors >= 20) ? " (ABORTED)" : "");
	return (errors > 0);
}
#endif
