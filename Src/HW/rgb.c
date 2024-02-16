/*
 * rgb.c
 *
 * Uses TIM12, CH2 on PB15 (Pin 76).
 * Data is transferred thru DMA1/Stream0
 * Signal must be inverted!
 * Uses Timer17 to trigger LED updates.
 *
 * Data format is 24bit GRB with MSB of green sent first.
 * Start with > 50µs reset pulse.
 * The base clock of TIM12 is 200MHz (5ns / tick)
 *
 * The 2812 Chip is also known as NeoPixel.
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
#include <string.h>
#include "rb2.h"
#include "config.h"

#ifdef HW_REV07
#define LEDS			8						///< number of LEDs that are chained
#else
#define LEDS			9						///< number of LEDs that are chained
#endif
#define BITS_PER_LED	24						///< the number of bits fo a single LED
#define DATABITS		(BITS_PER_LED * LEDS)	///< total number of bits to send
#define DMA_TRANSFERS	DATABITS + 2			///< transmit 2 extra bits (with zero-Timing) to revert the data line to constant zero (RESET-Level)
#define T0TIME			7						///< 350ns H + 1150ns L (all timings are measured in 50ns/20MHz clock tick units)
#define T1TIME			14						///< 700ns H + 800ns L
#define BITTIME			30						///< 1500ns bit time
#define RESET_TIME		6000					///< reset time is 300µs

static enum rgbmode {
	RGB_READY = 0,					///< the function is idle, the color is steady - nothing to do
	RGB_STARTUP,					///< swell and decay intensity during startup (see RGB_SWELL_WHITE)
	RGB_SINGLE_COLOR,				///< set all LEDs to a specific color
	RGB_SWELL_WHITE,				///< swell and decay intensity
	RGB_DARK,						///< decay intensity
	RGB_GO,							///< go green
	RGB_STOP,						///< go red
	RGB_SHORT,						///< blink pink....
	RGB_BORING,						///< ....
	RGB_IDENTIFY,					///< BiDiB Identify
	RGB_HOT,						///< overtemperature
} volatile mode;

static TaskHandle_t RGBtask;							///< the task handle to be signaled by the interrupt handler requesting next packet preparation
static volatile int step;
static volatile int red, green, blue;	// the values that should sent to the LEDs for RGB_SINGLE_COLOR

static uint16_t timings[DMA_TRANSFERS];

/**
 * Prepare TIM12 for startup as soon as beeing enabled.
 * The first round is configured to output a reset pulse (permanent LOW) for the duration
 * defined by the RESET_TIME. The second round will be latched into effect as soon as the
 * first round is over and repesents a single bit time with LOW level (so it effectively
 * elongates the reset pulse by one bit time).
 *
 * The update event after the first round triggers the first DMA transfer which will begin
 * to output the bit pattern until all bits are output.
 */
static void tim12_preset(void)
{
	TIM12->CR1 &= ~TIM_CR1_CEN;		// stop timer by disabling counting
	TIM12->ARR = RESET_TIME - 1;	// the first run will output the reset time
	TIM12->CCR2 = 0;				// set to 0 to keep the output in the inactive level (RESET for WS2812)
    TIM12->EGR = TIM_EGR_UG;		// generate update event to preload the registers
	TIM12->ARR = BITTIME - 1;		// all following runs (from the next update event on) will have the duration of the bit time
}

static void tim12_init (void)
{
	TIM12->CR1 = TIM_CR1_ARPE;		// disable timer counting
	TIM12->CCER = TIM_CCER_CC2P;	// both compare channels disabled, CH2 is set to active LOW (inversion)
	TIM12->CR2 = (0b010 << TIM_CR2_MMS_Pos);	// update event genrates TRGO (trigger output, used here for DMA trigger)
	TIM12->SMCR = 0;				// we use the timer as master (stand alone) timer - so keep at reset value
	TIM12->DIER = 0;				// we dont use (timer) interrupts
	TIM12->SR = 0;					// clear all status bits
	TIM12->CCMR1 = TIM_CCMR1_OC2PE | (0b0110 << TIM_CCMR1_OC2M_Pos);		// channel 2 uses PWM mode 1 with preload enabled
	TIM12->CNT = 0;					// preset counter to 0
	TIM12->PSC = 9;					// prescaling 1:10 (-> 20MHz)
	TIM12->CCER |= TIM_CCER_CC2E;	// enable compare channel 2

	tim12_preset();
}

static void tim17_init (void)
{
	TIM17->CR1 = 0;					// disable timer counting
	TIM17->SMCR = 0;				// we use the timer as master (stand alone) timer - so keep at reset value
	TIM17->CNT = 0;					// start counting at zero
	TIM17->PSC = 199;				// timer counts with microseconds
	TIM17->ARR = 999;				// the overflow rate will be 1ms
	TIM17->DIER = TIM_DIER_UIE;		// the update interrupt happens on overflow
}

static void tim17_stop (void)
{
	NVIC_DisableIRQ(TIM17_IRQn);
	TIM17->CR1 = 0;					// stop and disable timer
}

static void tim17_start (int ms)
{
	tim17_stop();

	if (ms < 1) ms = 1;				// minimum delay is 1ms
	if (ms > 256) ms = 256;			// maximum delay is 256ms

	TIM17->CR1 = 0;					// stop and disable timer
	TIM17->CNT = 0;
	TIM17->RCR = ms - 1;
	NVIC_EnableIRQ(TIM17_IRQn);
	TIM17->CR1 |= TIM_CR1_CEN;		// enable the counter
}

static void ws2812_setColor (int led, uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t *p;
	int i;

	if (led < 0 || led >= LEDS) return;

	p = &timings[BITS_PER_LED * led];		// position pointer to the 24-bit slot of the desired LED
	for (i = 0; i < 8; i++) {
		*p++ = (g & 0x80) ? T1TIME : T0TIME;
		g <<= 1;
	}
	for (i = 0; i < 8; i++) {
		*p++ = (r & 0x80) ? T1TIME : T0TIME;
		r <<= 1;
	}
	for (i = 0; i < 8; i++) {
		*p++ = (b & 0x80) ? T1TIME : T0TIME;
		b <<= 1;
	}
}

static void ws2812_update (void)
{
	tim12_preset();

	cache_flush((uint32_t) timings, sizeof(timings));
	DMA1_Stream0->CR = 0;
	while (DMA1_Stream0->CR & DMA_SxCR_EN) ;	// wait until the DMA is really disabled (just in case ...)

	/*
	 * Memory transfer size is 32bit, peripheral transfer size is 16bit,
	 * memory is incremented after each data transfer, peripheral address stays fixed
	 * Memory to peripheral, DMA controls the flow
	 */
	DMA1_Stream0->CR = (0b10 << DMA_SxCR_MSIZE_Pos) | (0b01 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC | (0b01 << DMA_SxCR_DIR_Pos);
	DMA1_Stream0->NDTR = DMA_TRANSFERS;
	DMA1_Stream0->PAR = (uint32_t) &TIM12->CCR2;	// target is the CCR2 register of TIM12
	DMA1_Stream0->M0AR = (uint32_t) timings;
	DMA1_Stream0->FCR = DMA_SxFCR_DMDIS | (0b01 << DMA_SxFCR_FTH_Pos);	// use FIFO (Direct Mode Disable) and set reload trigger to 1/2 full
	// now clear all interrupt flags of DMA1 Stream0: Transfer complete, half transfer complete, Transfer error, Direct mode error and FIFO error
	DMA1->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

	/*
	 * Now configure the DMAMUX1:
	 * Use DMAMUX1 channel 0
	 * Use trigger input (SIG_ID) 7 (TIM12_TRGO), i.e. set DMA Request MUX input to 1 (Request generator 0)
	 */
	DMAMUX1_Channel0->CCR = (1 << DMAMUX_CxCR_DMAREQ_ID_Pos);
	/*
	 * Configure Request generator 0
	 * rising edge of TIM12-TRGO is trigger
	 * generate exactly one DMA-Trigger per TIM12_TRGO (i.e. GNBREQ[4:0] = 0)
	 */
	DMAMUX1_RequestGenerator0->RGCR = 0;	// first: disable request generator
	DMAMUX1_RequestGenerator0->RGCR = (0b01 << DMAMUX_RGxCR_GPOL_Pos) | (7 << DMAMUX_RGxCR_SIG_ID_Pos);
	SET_BIT(DMAMUX1_RequestGenerator0->RGCR, DMAMUX_RGxCR_GE);	// enable request generator

	TIM12->CR1 |= TIM_CR1_CEN;		// enable and start TIM12
	// now enable DMA and transfer complete interrupt
	SET_BIT(DMA1_Stream0->CR, (DMA_SxCR_EN | DMA_SxCR_TCIE));
	NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);		// clear a possibly pending interrupt request in NVIC
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);				// enable interrupt in NVIC
}

static void ws2812_color (int r, int g, int b)
{
	int i;

	for (i = 0; i < LEDS; i++) {
		ws2812_setColor(i, r, g, b);
	}
	ws2812_update();
}

static void rgb_init (void)
{
	tim12_init();
	tim17_init();

    NVIC_SetPriority(DMA1_Stream0_IRQn, 15);		// very low priority
    NVIC_SetPriority(TIM17_IRQn, 15);				// very low priority

	// TOP: 0 - 5, BOT: 6 - 8	(HW0.7: TOP: 0, 2, 4, 6 BOTTOM: 1, 3, 5, 7)
	timings[DATABITS + 0] = 0;
	timings[DATABITS + 1] = 0;
	ws2812_color(0, 0, 0xB0);
}

void rgb_handler (void *pvParameter)
{
	int intensity;
	int bot=0, bor = 0;
	struct sysconf *sc;

	(void) pvParameter;

	rgb_init();

	RGBtask = xTaskGetCurrentTaskHandle();
	sc = cnf_getconfig();
	vTaskDelay(200);		// give initialisation time to clock out the reset values from rgb_init()

	red = green = blue = 0x80;
#if 0
	if (netif_is_up(rt.en)) {
		printf ("%s() IP-Addr = %s\n", __func__, ip4addr_ntoa(&rt.en->ip_addr));
		ws2812_color(0x70, 0x10, 0x10);
		mode = RGB_READY;
	} else {
		// start with the white swelling ... (see rgb_swell())
		mode = RGB_SWELL_WHITE;
	}
#else
	// start with the white swelling ... (see rgb_swell())
	mode = RGB_STARTUP;
#endif
	step = 0;
	xTaskNotifyGive(RGBtask);

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		switch (mode) {
			case RGB_READY:
				tim17_stop();		// just as a precaution
				if((rt.tm == TM_STOP) || (rt.tm == TM_GO)) {
					tim17_start(250);
					if(step++ > 120) {
						if (sc->sysflags & SYSFLAG_LIGHTEFFECTS)  mode = RGB_BORING;	//after 30s nothing to do -> boring mode ;-)
						step = 0;
						bot = 0;
						bor = 1;
					}
				}
				break;
			case RGB_BORING:
				if(rt.tm == TM_STOP) {
					if(bor++ > 2) {
						bor = 0;
						step %= 6;
						if(!step) {
							ws2812_setColor(5, 70, 0, 0);
							ws2812_setColor(0, 250, 0, 0);
						} else {
							ws2812_setColor(step - 1, 70, 0, 0);
							ws2812_setColor(step, 250, 0, 0);
						}
						step++;
					}
					if(bor & 1) {
						switch(bot) {
							case 1:
								ws2812_setColor(7, 10, 0, 0);
								ws2812_setColor(8, 10, 0, 0);
								ws2812_setColor(6, 250, 0, 0);
								break;
							case 2:
								ws2812_setColor(6, 10, 0, 0);
								ws2812_setColor(8, 10, 0, 0);
								ws2812_setColor(7, 250, 0, 0);
								break;
							case 3:
								ws2812_setColor(7, 10, 0, 0);
								ws2812_setColor(6, 10, 0, 0);
								ws2812_setColor(8, 250, 0, 0);
								break;
							case 4:
								ws2812_setColor(6, 10, 0, 0);
								ws2812_setColor(8, 10, 0, 0);
								ws2812_setColor(7, 250, 0, 0);
								bot = 0;
						}
						bot++;
					}
					if(!(sc->sysflags & SYSFLAG_LIGHTEFFECTS)) {
						//old colors go down
						step = 2;	//its stop
						tim17_start(5);
						mode = RGB_DARK;
					} else {
						ws2812_update();
						tim17_start(250);
					}
				} else {
					if(bor) {
						if(bot++ > 40) {
							bor = 0;
						}
					} else {
						if(bot-- == 1) {
							bor = 1;
						}
					}
					if(!(sc->sysflags & SYSFLAG_LIGHTEFFECTS)) {
						//old colors go down
						step = 1;	//its go
						tim17_start(5);
						mode = RGB_DARK;
					} else {
						ws2812_color(0, 90 + (bot < 20 ? 20 - bot : 0), bot > 20 ? bot - 20 : 0);
						tim17_start(200);
					}
				}
				break;
			case RGB_HOT:
				ws2812_color (100 + (rand() & 0x7F), 10 + (rand() & 0xF), 0);
				break;
			case RGB_SINGLE_COLOR:
				ws2812_color(red, green, blue);
				mode = RGB_READY;
				break;
			case RGB_STARTUP:
			case RGB_SWELL_WHITE:
				if (step == 0) {
					tim17_start(50);
				}
				if (step > 20) step = 1;
				if (step <= 10) {
					intensity = step;
				} else {
					intensity = 20 - step;
				}
				if(hwinfo->manufacturer != 62) {	// Nur Tams hat weiß, andere blau
					red = 0;
					green = 0;
				}
				ws2812_color(red * intensity / 10 + 20, green * intensity / 10 + 20, blue * intensity / 10 + 20);
				step++;
				if (mode == RGB_STARTUP && xTaskGetTickCount() > 4000) {
					tim17_start(5);
					step = 2;
					mode = RGB_DARK;
				}
				break;
			case RGB_DARK:
				if(red) red--;
				if(green) green--;
				if(blue) blue--;
				ws2812_color(red, green, blue);
				if(!red && !green && !blue)
				{
					tim17_stop();
					switch(step)
					{
						case 1:
							mode = RGB_GO;
							tim17_start(15);
							break;
						case 3:
							mode = RGB_IDENTIFY;
							tim17_start(3);
							break;
						case 2:
						default:
							mode = RGB_STOP;
							tim17_start(15);
							break;
					}
				}
				break;
			case RGB_IDENTIFY:
				if(step < 10) {
					step++;
					bot = 0;
					bor++;
				} else if(step > 240) {
					step--;
					bot = 1;
				} else {
					if(bot) {
						step--;
					} else {
						step++;
					}
					ws2812_setColor(bor % 6, 0, 0, step);
				}
				if(rt.tm == TM_GO) {
					ws2812_setColor(6, 0, 150, 0);
					ws2812_setColor(7, 0, 150, 0);
					ws2812_setColor(8, 0, 150, 0);
				} else {
					ws2812_setColor(6, 150, 0, 0);
					ws2812_setColor(7, 150, 0, 0);
					ws2812_setColor(8, 150, 0, 0);
				}
				ws2812_update();
				break;
			case RGB_GO:
				step++;
				if(step < 90) ws2812_setColor(0, 0, step, 0);
				if((step > 9) && (step < 99)) ws2812_setColor(1, 0, step - 9, 0);
				if((step > 18) && (step < 108)) ws2812_setColor(2, 0, step - 18, 0);
				if((step > 27) && (step < 117)) ws2812_setColor(3, 0, step - 27, 0);
				if((step > 36) && (step < 126)) ws2812_setColor(4, 0, step - 36, 0);
				if((step > 45) && (step < 135)) ws2812_setColor(5, 0, step - 45, 0);
				if((step > 54) && (step < 144)) ws2812_setColor(6, 0, step - 54, 0);
				if((step > 63) && (step < 153)) ws2812_setColor(7, 0, step - 63, 0);
				if((step > 72) && (step < 162)) ws2812_setColor(8, 0, step - 72, 0);
				ws2812_update();
				if(step == 162)
				{
					if(sc->sysflags & SYSFLAG_LIGHTSOFF) {
						rgb_color (0, 10, 0);
						mode = RGB_SINGLE_COLOR;
						break;
					} else {
						green = step;
					}
					step = -1;
					mode = RGB_READY;
				}
				break;
			case RGB_STOP:
				step++;
				if(step < 90) ws2812_setColor(0, step, 0, 0);
				if((step > 9) && (step < 99)) ws2812_setColor(1, step - 9, 0, 0);
				if((step > 18) && (step < 108)) ws2812_setColor(2, step - 18, 0, 0);
				if((step > 27) && (step < 117)) ws2812_setColor(3, step - 27, 0, 0);
				if((step > 36) && (step < 126)) ws2812_setColor(4, step - 36, 0, 0);
				if((step > 45) && (step < 135)) ws2812_setColor(5, step - 45, 0, 0);
				if((step > 54) && (step < 144)) ws2812_setColor(6, step - 54, 0, 0);
				if((step > 63) && (step < 153)) ws2812_setColor(7, step - 63, 0, 0);
				if((step > 72) && (step < 162)) ws2812_setColor(8, step - 72, 0, 0);
				ws2812_update();
				if(step == 162)
				{
					if(sc->sysflags & SYSFLAG_LIGHTSOFF) {
						rgb_color (10, 0, 0);
						mode = RGB_SINGLE_COLOR;
						break;
					} else {
						red = step;
					}
					step = -1;
					mode = RGB_READY;
				}
				break;
			case RGB_SHORT:
				step++;
				if(step == 30) {
					ws2812_color(220, 0, 220);
					ws2812_update();
				}
				if(step > 100) {
					ws2812_color(220, 0, 0);
					ws2812_update();
					step = 0;
				}
				break;
		}
	}
}

void rgb_color (int r, int g, int b)
{
	red = r;
	green = g;
	blue = b;
	mode = RGB_SINGLE_COLOR;
	if (RGBtask) xTaskNotifyGive(RGBtask);		// if called too early, the RGB task may not have been started yet
}

void rgb_go (void)
{
	if (rt.tm == TM_GO) return;			// nothing to do
	if (rt.tm == TM_OVERTTEMP) return;	// this mode is locking mode switch
	if (mode == RGB_IDENTIFY) return;	// this mode is locking mode switch
	//old colors go down
	step = 1;	//its go
	tim17_start(5);
	mode = RGB_DARK;
}

void rgb_stop (void)
{
	if (rt.tm == TM_OVERTTEMP) return;	// this mode is locking mode switch
	if (mode == RGB_IDENTIFY) return;	// this mode is locking mode switch
	//old colors go down
	step = 2;	// its stop
	tim17_start(5);
	mode = RGB_DARK;
}

void rgb_identify (bool on)
{
	if (rt.tm == TM_OVERTTEMP) return;	// this mode is locking mode switch
	if (on) {
		step = 3;
	} else {
		if(rt.tm == TM_STOP) {
			step = 2;	// its stop
		} else {
			step = 1;	// its go
		}
	}
	mode = RGB_DARK;
	tim17_start(5);
}

void rgb_overtemp (void)
{
	rgb_color (200, 180, 10);
	mode = RGB_HOT;
	tim17_start(140);
}

void rgb_off (void)
{
	rgb_color(0, 0, 0);
}

void rgb_short (void)
{
	rgb_color (220, 0, 220);
	step = 0;	// its stop
	tim17_start(5);
	mode = RGB_SHORT;
}

void rgb_swell (void)
{
	if(hwinfo->manufacturer == 62) {
		red = green = blue = 0x80;
	} else {
		red = green = 0;
		blue = 0x80;
	}
	mode = RGB_SWELL_WHITE;
	step = 0;
	if (RGBtask) xTaskNotifyGive(RGBtask);
}

void DMA_STR0_IRQHandler (void)
{
	if (DMA1->LIFCR & DMA_LISR_TCIF0) {
		CLEAR_BIT(DMAMUX1_RequestGenerator0->RGCR, DMAMUX_RGxCR_GE);
		CLEAR_BIT(DMA1_Stream0->CR, DMA_SxCR_EN);
		TIM12->CR1 &= ~TIM_CR1_CEN;		// stop timer by disabling counting
	}
	// clear all DMA1 Stream0 interrupt flags
	DMA1->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
}

void TIM17_IRQHandler (void)
{
	BaseType_t xHigherPriorityTaskWoken = 0;

	TIM17->SR = ~TIM_SR_UIF;
	if (RGBtask) vTaskNotifyGiveFromISR (RGBtask, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
