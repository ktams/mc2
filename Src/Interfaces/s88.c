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

/*
 * Reading the s88 bus (via optocouplers). The data is read via GPIO bitbanging.
 * Signals are as follows:
 *   - s88 data (GPIO, input): PG12
 *   - clock (GPIO, output): PG13
 *   - s88 reset (GPIO, output): PD03
 *   - s88 P/S or LOAD (GPIO, output): PB04
 * ON HW V0.7 DATA and CLK was on different pins:
 *   - s88 data (GPIO, input): PA06
 *   - clock (GPIO, output): PG11
 *
 * The s88 is organized in modules with 16 bits of information each.
 * When shifting in, the MSB is sent first, which is contact '1' as
 * defined by Märklin. On the P50 interface, the bytes of this 16 bit
 * integer are output with the MSB first.
 *
 * Kernel clock for timers is 200MHz. We can use a prescaler of 200 to get
 * down to a tickrate of 1MHz. Setting the reload register (ARR) to 100
 * can give us an interrupt rate of 10kHz. Making one clock edge in each
 * interrupt gives an s88 clock rate of 5kHz with equal LOW and HIGH
 * portions of 100µs. The frequency can be further reduced on request
 * (theoretical down to below 10Hz) by providing larger values for ARR.
 *
 * The data on the s88 modules is shifted out and loaded parallel with
 * the rising edge of the clock line. We should read bit values together
 * with the falling edge of the clock.
 *
 * A final remark: All signals are inverted because of the optocouplers
 * we are using. So at the GPIO hardware level, the bits must be inverted
 * to the real levels for the outside world. This also includes the s88
 * DATA input pin!
 */

#include <string.h>
#include <stdio.h>
#include "rb2.h"
#include "events.h"
#include "config.h"
#include "bidib.h"

#define TIMER_CLOCKRATE				1000000			///< the timer runs at a base clock rate of 1MHz
#define S88_CS2_SEQUENCE			0				///< if defined to != 0, we use the sequence of the CS2 s88 bus (see explanation above interrupt handler)
//#define S88_CLK_DEBUG								///< if defined, the Märklin booster ON/OFF is a copy of the clock signal

#ifdef CENTRAL_FEEDBACK
static volatile uint16_t input[MAX_S88MODULES];		///< the current status of all information bits read in
static volatile int s88_modules;					///< the actual number of used s88 modules

#else

static volatile uint16_t input[MAX_FBMODULES];		///< the current status of all information bits read in
static struct s88_status status;					///< the current status to be reported in events
static volatile int s88_modules;					///< the actual number of used s88 modules
static volatile int can_modules;					///< the actual number of used can modules
static volatile int lnet_modules;					///< the actual number of used loconet modules
static volatile int modules;						///< the total number of available feedback modules

#endif

static TaskHandle_t s88_task;						///< the task to wake up every time there is a full reading done

static void s88_initTimer (void)
{
	TIM4->CR1 = 0;	// disable and reset TIM4

	TIM4->CR1 = TIM_CR1_ARPE;		// ARR is buffered
	TIM4->CR2 = 0;					// no settings are used, idle states are LOW
	TIM4->SMCR = 0;					// no settings are used (slave mode is disabled)
	TIM4->DIER = 0;					// start with disabling interrupts
	TIM4->SR = 0;					// clear all status bits
	TIM4->CCER = 0;					// we don't use the capture / compare channels
	TIM4->CCMR1 = 0;				// we use simple timer mode - no output compare or input capture
	TIM4->CCMR2 = 0;				// we use simple timer mode - no output compare or input capture

	TIM4->PSC = 199;				// select a prescaler of 200 (PSC + 1)
	TIM4->RCR = 0;					// we don't use the repetition counter
	TIM4->AF1 = 0;					// we don't use any ETR input stuff

	TIM4->ARR = 99;					// set ARR to 100 ticks (giving 10kHz interrupt rate)
	TIM4->EGR = TIM_EGR_UG;			// trigger an update to get ARR value loaded

	NVIC_SetPriority(TIM4_IRQn, 14);
	NVIC_ClearPendingIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM4_IRQn);

	TIM4->SR = 0;							// clear a possibly pending interrupt
	SET_BIT (TIM4->DIER, TIM_DIER_UIE);		// enable update interrupt (i.e. at timer overflow)
}

#if 0
static int _s88_setModules (int oldcount, int count)
{
	struct bidibnode *root, *n;

	if (count > MAX_S88MODULES) count = MAX_S88MODULES;
	if (count < 0) count = 0;
	if (oldcount < 0) oldcount = 0;

	if (oldcount != count) {
		root = BDBnode_lookupNodeByUID(s88HubUID, NULL);
		if (count == 0) {
			if (root) BDBnode_dropNode(root);
		} else {
			if (!root) {
				oldcount = 0;
				root = BDBvn_newBridge(BDBnode_getRoot(), BIDIB_HUB_S88);
			}
			while (oldcount < count) {
				oldcount++;
				BDBvn_newS88(root, BIDIB_S88_SNR_OFFSET + oldcount);
			}
			while (oldcount > count) {
				if ((n = BDBnode_lookupChild(root, oldcount)) != NULL) BDBnode_dropNode(n);
				oldcount--;
			}
		}
		s88_modules = count;
		s88_triggerUpdate();
	}

	return count;
}

/**
 * Create the virtual nodes that correspond to the given number of
 * feedback modules on each of the supported busses. Each module counts
 * 16 inputs.
 *
 * It starts by deleting the current nodes (and their respective parent HUB nodes).
 * It then creates new nodes with a HUB for each feedback bus.
 *
 * \param s88		number of s88 modules to create virtual nodes for
 * \param mcan		number of mcan feedback modules to create virtual nodes for
 * \param lnet		number of LocoNet feedback modules to create virtual nodes for
 * \return			number of virtual nodes created for feedback purposes
 */
static int s88_createBiDiBnodes (int s88, int mcan, int lnet)
{
	struct bidibnode *root;
	int i, count;

	log_msg(LOG_INFO, "%s() #s88=%d #mcan=%d #lnet=%d\n", __func__, s88, mcan, lnet);

	// step 1: drop all parent nodes. This will also drop the feedback nodes as well.
	if ((root = BDBnode_lookupNodeByUID(s88HubUID, NULL)) != NULL) BDBnode_dropNode(root);
	if ((root = BDBnode_lookupNodeByUID(mcanHubUID, NULL)) != NULL) BDBnode_dropNode(root);
	if ((root = BDBnode_lookupNodeByUID(lnetHubUID, NULL)) != NULL) BDBnode_dropNode(root);

	if ((s88 + mcan + lnet) == 0) return 0;		// nothing else to do

	count = 0;

	if ((s88 > 0) && (root = BDBvn_newBridge(BDBnode_getRoot(), BIDIB_HUB_S88)) != NULL) {
		memcpy (s88HubUID, root->uid, sizeof(s88HubUID));
		for (i = 1; i <= s88; i++) {
			BDBvn_newS88(root, BIDIB_S88_SNR_OFFSET + i);
			count++;
		}
	}

	if ((mcan > 0) && (root = BDBvn_newBridge(BDBnode_getRoot(),BIDIB_HUB_MCAN)) != NULL) {
		memcpy (mcanHubUID, root->uid, sizeof(mcanHubUID));
		for (i = 1; i <= mcan; i++) {
			BDBvn_newMCAN(root, BIDIB_MCAN_SNR_OFFSET + i);
			count++;
		}
	}

	if ((lnet > 0) && (root = BDBvn_newBridge(BDBnode_getRoot(), BIDIB_HUB_LNET)) != NULL) {
		memcpy (lnetHubUID, root->uid, sizeof(lnetHubUID));
		for (i = 1; i <= lnet; i++) {
			BDBvn_newLNET(root, BIDIB_LNET_SNR_OFFSET + i);
			count++;
		}
	}

	return s88 + mcan + lnet;
}
#endif

/**
 * The main thread function for s88 handling.
 *
 * After enabling the timer TIM4 we wait for the completion of reading
 * the bits from the modules. The interupt handler disables the timer and
 * sets our task notification value. We now have time to analyse the read
 * bits.
 *
 * For detecting changes, we have the static input[] array now containing the
 * current state as read in by the interrupt. The status.sum[] array contains
 * the last knwon state, so we can determine any difference. For each module
 * where a difference is seen, the event-bit in status.evFlag[] is set (this
 * array is cleared before we start our analysis).
 *
 * If any changes were detected (i.e. the event bits array is not empty) we
 * fire an event. The event data contains a pointer to our local status
 * structure, while the timer interrupt in the next round will fill the
 * independant input[] array.
 *
 * This continues endless. The minimum time between two rounds is at 5kHz
 * clock rate with just one single s88 module. The timer interrupt fires at
 * 10kHz (every 100µs) and needs (s88_modules * 16 * 2 + 4) steps to complete.
 * This gives the minium time of 36 steps = 3,6ms. The maximum time is when
 * we drop the clockrate to 500Hz with MAX_S88MODULES (64). With 500Hz clock
 * rate we use a 1kHz interrupt rate (1ms). This leads to 2052 steps, in other
 * words more than 2 seconds.
 *
 * If no s88 modules are defined, the thread just sleeps for 200ms and then
 * checks again if the number of defined modules is above zero.
 *
 * \param pvParameter	unused private data pointer from xTaskCreate() call
 */
#ifdef CENTRAL_FEEDBACK

void vS88Bus (void *pvParameter)
{
	struct sysconf *cnf;

	(void) pvParameter;

	s88_task = xTaskGetCurrentTaskHandle();
	s88_initTimer();
	cnf = cnf_getconfig();
	s88_setFrequency(cnf->s88Frequency);

	s88_modules = cnf->s88Modules;

	log_msg(LOG_INFO, "%s() startup with %d s88 modules on HW %X.%X\n", __func__, s88_modules, hwinfo->HW >> 4, hwinfo->HW & 0xF);

	for (;;) {
		if (s88_modules > 0) {
			SET_BIT (TIM4->CR1, TIM_CR1_CEN);			// enable and start the timer
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	// if we return, the timer is disabled - we have time to do our job
			fb_s88input(s88_modules, (uint16_t *) input);
		} else {
			vTaskDelay(200);
		}
	}
}

void s88_setModules (int count)
{
	struct sysconf *cnf;

	cnf = cnf_getconfig();
	if (cnf->s88Modules != count) {
		cnf->s88Modules = s88_modules = BDBvn_feedbackModules(cnf->s88Modules, count, MAX_S88MODULES, BIDIB_HUB_S88);
		cnf_triggerStore(__func__);
		event_fire(EVENT_FBPARAM, 0, NULL);
	}
}

int s88_getModules(void)
{
	return s88_modules;
}

void s88_setFrequency (int hz)
{
	struct sysconf *cnf;

	if (hz < 500) hz = 500;
	if (hz > 5000) hz = 5000;
	hz = (hz / 50) * 50;		// only increments of 50Hz are accepted

	cnf = cnf_getconfig();
	if (cnf->s88Frequency != hz) {
		cnf->s88Frequency = hz;
		cnf_triggerStore(__func__);
		event_fire(EVENT_FBPARAM, 0, NULL);
	}
	hz <<= 1;					// frequency must be doubled
	TIM4->ARR = (TIMER_CLOCKRATE / hz) - 1;
}

int s88_getFrequency(void)
{
	int hz;

	hz = (TIMER_CLOCKRATE / (TIM4->ARR + 1)) / 2;
	hz = (hz / 50) * 50;		// round to nearest 50Hz value
	return hz;
}

#else

void vS88Bus (void *pvParameter)
{
	struct bidibnode *n;
	struct sysconf *cnf;
	int i;

	(void) pvParameter;

	s88_task = xTaskGetCurrentTaskHandle();
	s88_initTimer();
	cnf = cnf_getconfig();
	s88_setFrequency(cnf->s88Frequency);

	s88_modules = cnf->s88Modules;
	can_modules = cnf->canModules;
	lnet_modules = cnf->lnetModules;
//#if 1
//	_s88_setModules(0, s88_modules);
//#else
//	modules = s88_createBiDiBnodes(s88_modules, can_modules, lnet_modules);
//#endif

//ToDo
	modules = s88_modules + can_modules + lnet_modules + 3;	//ACHTUNG!!! 3 Fakemodule als work around!!!
	log_msg(LOG_INFO, "%s() startup with %d s88 modules, %d can modules and %d L-Net modules (%d total)\n",
			__func__, s88_modules, can_modules, lnet_modules, modules);

	for (;;) {
		if (modules > 0) {
			SET_BIT (TIM4->CR1, TIM_CR1_CEN);			// enable and start the timer
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	// if we return, the timer is disabled - we have time to do our job
			memset (status.evFlag, 0, sizeof(status.evFlag));
#ifdef CENTRAL_FEEDBACK
			fb_s88input(modules, (uint16_t *) input);
#else
			for (i = 0; i < modules; i++) {
				if (input[i] != status.sum[i]) bs_set(status.evFlag, i);
				status.sum[i] = input[i];
				if (i >= (cnf->s88Modules + cnf->canModules)) {
					n = BDBvn_getFeebackNode(BIDIB_PID_VIRTUAL_LNET, i - cnf->s88Modules - cnf->canModules);
				} else if (i >= cnf->s88Modules) {
					n = BDBvn_getFeebackNode(BIDIB_PID_VIRTUAL_MCAN, i - cnf->s88Modules);
				} else {
					n = BDBvn_getFeebackNode(BIDIB_PID_VIRTUAL_S88, i);
				}
				if (n != NULL) {
					BDBvn_feedbackStatus (n, status.sum[i]);
				}
			}
			status.modcnt = modules;
			if (!bs_isempty(status.evFlag, modules)) {
				event_fire(EVENT_FEEDBACK, 0, &status);
			}
#endif
		} else {
			vTaskDelay(200);
		}
	}
}

/**
 * Get the s88 status from a module as it was last reported on an event.
 * This can be taken as the "current" status of the s88 input bits.
 *
 * \param m		the zero based module number to return the bits from
 * \return		the 16 bits that the s88 module 'm' reported in the last round.
 */
uint16_t s88_getInput(int m)
{
	if (m < 0 || m >= MAX_FBMODULES) return 0;
	return status.sum[m];
}

volatile uint16_t *s88_getInputs()
{
	return input;
}

void s88_triggerUpdate (void)
{
	status.modcnt = s88_modules;
	event_fire(EVENT_FEEDBACK, 0, &status);
}

void s88_setModules (int count)
{
	struct sysconf *cnf;

	cnf = cnf_getconfig();
	if (cnf->s88Modules != count) {
		cnf->s88Modules = s88_modules = BDBvn_feedbackModules(cnf->s88Modules, count, MAX_S88MODULES, BIDIB_HUB_S88);
		cnf_triggerStore(__func__);
		s88_triggerUpdate();
	}
}

int s88_getModules(void)
{
	return s88_modules;
}

int s88_getCanModules(void)
{
	return can_modules;
}

void s88_setFrequency (int hz)
{
	struct sysconf *cnf;

	if (hz < 500) hz = 500;
	if (hz > 5000) hz = 5000;
	hz = (hz / 50) * 50;		// only increments of 50Hz are accepted

	cnf = cnf_getconfig();
	if (cnf->s88Frequency != hz) {
		cnf->s88Frequency = hz;
		cnf_triggerStore(__func__);
	}
	hz <<= 1;					// frequency must be doubled
	TIM4->ARR = (TIMER_CLOCKRATE / hz) - 1;
	s88_triggerUpdate();
}

int s88_getFrequency(void)
{
	int hz;

	hz = (TIMER_CLOCKRATE / (TIM4->ARR + 1)) / 2;
	hz = (hz / 50) * 50;		// round to nearest 50Hz value
	return hz;
}

#endif

/*
 * LOAD (P/S) and RESET never changed since HW V0.7, so we can use macros
 * instead of functions.
 */
#define LOAD_HIGH()		do { GPIOB->BSRR = GPIO_BSRR_BR4; } while(0)
#define LOAD_LOW()		do { GPIOB->BSRR = GPIO_BSRR_BS4; } while(0)
#define RESET_HIGH()	do { GPIOD->BSRR = GPIO_BSRR_BR3; } while(0)
#define RESET_LOW()		do { GPIOD->BSRR = GPIO_BSRR_BS3; } while(0)

/*
 *   STEP	-2 -1  0  1  2  3  4  5  6  7  8  9  10 11
 *		      __       __             __    __    __
 *   CLK	_|  |_____|  |___________|  |__|  |__|  |_
 *		            ______________
 *   LOAD	_______|              |___________________
 *		                     __
 *   RES	________________|  |______________________
 *
 *   DATA	-<#last>--<#00>----------<#01>-<#02>-<#03>-
 *   readpos	^        ^              ^     ^     ^
 *
 * Steps:
 *  -2: the last rising CLK edge of the previous cycle shifts out the last s88 data bit
 *  -1: for cycle start we take the CLK line LOW and read the last s88 data bit
 *  ------ here a new cycle starts off -------
 *   0: CLK is low and we raise the "LOAD" or "P/S" pin to switch to parallel mode
 *   1: CLK is brought high, the shift register reads the summed states of the input latches, the first data bit is output to S88_BIT line
 *   2: CLK is brought LOW and the first data bit is read in
 *   3: the RESET line is taken HIGH to reset the input latches
 *   4: the RESET line is taken LOW, reset in latches ends (i.e. from now on new LOW pulses on inputs set the corresponding latch)
 *   5: LOAD (P/S) is taken LOW again to switch to serial shift mode
 *   6: CLK gets HIGH to shift out the stored second bit of s88 data stream
 *   7: CLK is taken LOW and this second bit is read in
 *   8 .. n: on every even step CLK line is set HIGH to shift out the next s88 data bit, on the odd steps it is set LOW and the bit is read
 *
 * The parallel latches sum up data between step 4 and until step 1 is reached again.
 * Low-High-Low glitches that happen between step 1 and 4 will be lost! This makes a
 * blind spot of 0,3ms - 3ms depending on the actual clockrate choosen.
 *
 * -------------------------------------------------------------------------------------
 * The CS2 timing (read position estimated). There is a pause between the last
 * bit read and the beginning of a new cycle. This pause is probably determined
 * by the set read intervall. In reality, the LOW impulse on the CLK line is quite
 * short in contrast to the HIGH portion.
 *
 *   STEP	-2  -1     0  1   2 3   4 5   6 7   8
 *		      ___          ___   ___   ___   ___
 *   CLK	_|   |_...____|   |_|   |_|   |_|   |_
 *		                _______
 *   LOAD	_______..._|       |__________________	<- falling edge very close to next rising edge of clock!
 *		                           ____
 *   RES	_______...____________|    |__________  <- not on clock edges!
 *
 *   DATA	--<#last>-...--<#00>-<#01>-<#02>-<#03>-
 *   readpos     ^            ^     ^     ^     ^
 *
 * -------------------------------------------------------------------------------------
 * The HW used changed thru the different versions. Here is a list:
 *
 *   HW       | CLK  | DATA | LOAD | RESET | Comments
 * -----------+------+------+------+-------+----------------------
 * HW >= 1.5  | PG13 | PG12 | PB04 | PD03  | additional inverter for the CLK line on the isolated side
 * HW >= 1.1  | PG13 | PG12 | PB04 | PD03  | all delivered units were modified to also include the inverter of V1.5
 * HW >= 0.7  | PG11 | PD02 | PB04 | PD03  |
 *
 * All signales are coupled with opto couplers and the a processor pin HIGH state
 * drives current thru the LED and therefor switches the open collector on the
 * secondary side to LOW! So there is in fact an inversion in the first step.
 */

/**
 * Set or clear the mask bit in the result array depending on the
 * value of the s88 input serial bit. This function must account for
 * the HW version. Remember: the read bit is inverted due to the
 * opto coupler used to isolate the s88 port from the rest of the
 * hardware.
 *
 * \param mask		a 16-Bit mask with the positional bit set (MSB is used first and then right-shifting until zero)
 * \param p			a pointer in the uint16_t array of s88 module status bits
 * \return			the next mask to be used (mask >> 1, may drop down to zero)
 */
static uint16_t s88_readBit (uint16_t mask, volatile uint16_t *p)
{
	if (p && mask) {	// we need p and the mask should not be zero!
		if (hwinfo->HW >= HW11) {		// all HW from V1.1 onwards uses PG12 as DATA line
			if ((GPIOG->IDR & GPIO_IDR_ID12) == 0) *p |= mask;
			else *p &= ~mask;
		} else {						// hardware V0.7 and V1.0 used PD02 as DATA line
			if ((GPIOD->IDR & GPIO_IDR_ID2) == 0) *p |= mask;
			else *p &= ~mask;
		}
	}
	return mask >> 1;
}

/**
 * Switch the CLK line to a logical state on the s88N connector.
 * This accounts for the HW version.
 *
 * \param high		the requiered state of the CLK signal on the s88N connector
 */
static void s88_clock (bool high)
{
	if (hwinfo->HW >= HW11) {			// additional inverter in CLK line from HW V1.5 (on isolated/secondary side)
										// this is also the case on HW from V1.1 onwards, see commented lines below
		if (high) GPIOG->BSRR = GPIO_BSRR_BS13;		// PG13 HIGH
		else GPIOG->BSRR = GPIO_BSRR_BR13;			// PG13 LOW
//	} else if (hwinfo->HW >= HW11) {	// from HW 1.1: CLK on PG13, DATA on PG12, RESET on PD3, P/S on PB4 - inverter was added by HW-patch!
//		if (high) GPIOG->BSRR = GPIO_BSRR_BR13;		// PG13 LOW
//		else GPIOG->BSRR = GPIO_BSRR_BS13;			// PG13 HIGH
	} else {							// up to HW 1.0 we used PG11 as CLK and PA6 as DATA pin
		if (high) GPIOG->BSRR = GPIO_BSRR_BR11;		// PG11 LOW
		else GPIOG->BSRR = GPIO_BSRR_BS11;			// PG11 HIGH
	}
}

void TIM4_IRQHandler (void)
{
	static int step;		// the steps counted from 0 to (s88_modules * 16(bits) * 2(edges)) + 4(extra steps for load/reset)
	static volatile uint16_t *p;
	static uint16_t mask;
//	static uint8_t filter;

	BaseType_t xHigherPriorityTaskWoken = 0;
	TIM4->SR = 0;						// clear all interrupt flags

#if S88_CS2_SEQUENCE
	if (step < 0 || step >= (s88_modules * 16 * 2 + 1)) step = 0;

	switch (step) {
		case 0:
			CLEAR_BIT (TIM4->CR1, TIM_CR1_CEN);
			s88_clock(LOW);			// should already be the case
			LOAD_HIGH();			// switch to parallel inputs (from latches)
			vTaskNotifyGiveFromISR(s88_task, &xHigherPriorityTaskWoken);
			break;
		case 1:
			s88_clock(HIGH);		// this latches the parallel data to the shift register
			break;
		case 2:
			s88_clock(LOW);
			LOAD_LOW();
			p = input;
			mask = s88_readBit(0x8000, p);
			break;
		case 3:
			s88_clock(HIGH);		// shift out next bit
			RESET_HIGH();			// clear parallel input latches
			break;
		case 4:
			s88_clock(LOW);			// no shifting is done, but we now can safely read the s88 bit
			mask = s88_readBit(mask, p);
			break;
		case 5:
			s88_clock(HIGH);		// shift out next bit
			RESET_LOW();			// release clear enabling the LOW-active latch setting
			break;
		default:
			if (step & 1) {			// rising edge on odd steps
				s88_clock(HIGH);	// shift out next bit
			} else {				// falling edge on even steps
				s88_clock(LOW);		// no shifting is done, but we now can safely read the s88 bit
				mask = s88_readBit(mask, p);
				if (!mask) {
					p++;
					mask = 0x8000;
				}
			}
			break;
	}
#else	/* S88_CS2_SEQUENCE */
	if (step < 0 || step >= (s88_modules * 16 * 2 + 4)) step = 0;

	switch (step) {
		case 0:
			s88_clock(LOW);			// should already be the case
			LOAD_HIGH();			// switch to parallel inputs (from latches)
			vTaskNotifyGiveFromISR(s88_task, &xHigherPriorityTaskWoken);
			CLEAR_BIT (TIM4->CR1, TIM_CR1_CEN);
			break;
		case 1:
			s88_clock(HIGH);		// this latches the parallel data to the shift register
			break;
		case 2:
			s88_clock(LOW);
			p = input;
			/*if (s88_modules)*/ mask = s88_readBit(0x8000, p);		// read first bit of new cycle
			break;
		case 3:
			RESET_HIGH();			// clear parallel input latches
			break;
		case 4:
			RESET_LOW();			// release clear enabling the LOW-active latch setting
			break;
		case 5:
			LOAD_LOW();
			break;
		default:
			if (!(step & 1)) {
				s88_clock(HIGH);					// shift out next bit
			} else {
				s88_clock(LOW);				// no shifting is done, but we now can safely read the s88 bit
				mask = s88_readBit(mask, p);
				if (!mask) {
					p++;
					mask = 0x8000;
				}
			}
			break;
	}
#endif	/* !S88_CS2_SEQUENCE */

	step++;
	NVIC_ClearPendingIRQ(TIM4_IRQn);
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
