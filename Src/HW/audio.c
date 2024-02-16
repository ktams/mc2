/*
 * audio.c
 *
 * Uses TIM15, CH1 on PE5 (Pin 4) and CH2 on PE6 (Pin 5).
 * Data is transferred thru DMA1/Stream1
 *
 * -----------------------------------------------------------------------------------
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
#include <math.h>
#include "rb2.h"

#define BASE_FREQUENCY		200000000	///< base frequency for timer
#define BUFFERLEN			4800		///< number of samples buffered for output

static struct soundsample {
	uint16_t	left;
	uint16_t	right;
} samples[BUFFERLEN];					///< interleaved samples of left + right channel (16 bits each)

static void init_tim15 (void)
{
	TIM15->CR1 = 0;	// disable and reset TIM15

	TIM15->CR1 = TIM_CR1_ARPE;			// ARR is buffered
	TIM15->CR2 = TIM_CR2_CCDS;			// no settings are used, idle states are LOW, DMA request on update event
	TIM15->SMCR = 0;					// no settings are used (slave mode is disabled)
	TIM15->DIER = 0;					// start with disabling interrupts
	TIM15->SR = 0;						// clear all status bits
	TIM15->BDTR = TIM_BDTR_OSSI;		// we keep control over the outputs, event if if MOE is cleared

	// channel 1 and channel 2 are compare outputs (PWM mode 1)
	TIM15->CCMR1 = (0b0110 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE | (0b0110 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

	// enable both channels
	TIM15->CCER = TIM_CCER_CC2E | TIM_CCER_CC1E;

	TIM15->PSC = 0;						// select a prescaler of 1 (PSC + 1)
	TIM15->RCR = 0;						// don't use any repetition
	TIM15->BDTR = TIM_BDTR_MOE;			// no dead time generation, master output enable is set
	TIM15->AF1 = 0;						// we don't use any break input (we break for nobody!)

    TIM15->EGR = TIM_EGR_UG;			// update the registers
	TIM15->SR = 0;						// clear a possibly pending interrupt
}

static void init_dma (void)
{
	// configure the TIM15 DMA control register to use bursts of 2 and starting at register CCR1
	TIM15->DCR = (0b00001 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);

	/*
	 * Memory transfer size is 32bit, peripheral transfer size is 2x16bit (but uses 2 bursts),
	 * memory is incremented after each data transfer, peripheral address stays fixed
	 * Memory to peripheral, DMA controls the flow
	 */
	// channel data will be interleaved!
	DMA1_Stream1->CR = (0b10 << DMA_SxCR_MSIZE_Pos) | (0b01 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | (0b01 << DMA_SxCR_DIR_Pos);
	DMA1_Stream1->NDTR = BUFFERLEN;
	DMA1_Stream1->PAR = (uint32_t) &TIM15->DMAR;	// target is the DMAR indirection register of TIM15 (addressing CCR1 + CCR2)
	DMA1_Stream1->M0AR = (uint32_t) samples;
	DMA1_Stream1->FCR = DMA_SxFCR_DMDIS | (0b01 << DMA_SxFCR_FTH_Pos);	// use FIFO (Direct Mode Disable) and set reload trigger to 1/2 full
	// now clear all interrupt flags of DMA1 Stream1: Transfer complete, half transfer complete, Transfer error, Direct mode error and FIFO error
	DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

	/*
	 * Now configure the DMAMUX1:
	 * Use DMAMUX1 channel 1
	 * Use DMA Request MUX input 106 (TIM15_UP)
	 */
	DMAMUX1_Channel1->CCR = (106 << DMAMUX_CxCR_DMAREQ_ID_Pos);
	SET_BIT(DMA1_Stream1->CR, DMA_SxCR_EN);
	SET_BIT (TIM15->DIER, TIM_DIER_UDE);
}

static int audio_setSamplingFrequency (int freq)
{
	int period, amplitude_zero;

	period = BASE_FREQUENCY / freq;
	if (period & 1) period++;
	TIM15->ARR = period - 1;
	amplitude_zero = period / 2;
	TIM15->CCR1 = amplitude_zero;		// output a 50:50 wave (that is half the supply voltage)
	TIM15->CCR2 = amplitude_zero;		// output a 50:50 wave (that is half the supply voltage)
	TIM15->EGR = TIM_EGR_UG;
	SET_BIT (TIM15->CR1, TIM_CR1_CEN);	// enable the timer (currently without any interrupts - just free running

	return amplitude_zero;
}

static void sinus (int zero, int amplitude, int freq, int samplefreq, struct soundsample *smp, bool right)
{
	double d;
	int i;

	for (i = 0; i < BUFFERLEN; i++) {
		d = (double) (i * freq) / samplefreq;
		d = sin(2 * M_PI * d) * amplitude;
		if (right) {
			smp->right = (uint16_t) (d + zero);
		} else {
			smp->left = (uint16_t) (d + zero);
		}
		smp++;
	}
	cache_flush((uint32_t) samples, sizeof(samples));
}

void vAudioTest (void *pvParameter)
{
	int amplitude_zero;

	(void) pvParameter;

	init_tim15();
	amplitude_zero = audio_setSamplingFrequency(48000);

	printf ("%s(): start with 48kHz, zero @ %d\n", __func__, amplitude_zero);

	sinus (amplitude_zero, 100, 1000, 48000, samples, false);
	sinus (amplitude_zero, 100, 500, 48000, samples, true);
	init_dma();

	vTaskDelete(NULL);
}
