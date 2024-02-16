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
#include "events.h"
#include "decoder.h"

/*
 * We are converting:
 *  - the track current			(PA00, ADC1 - INP16)
 *  - the track current			(PA00, ADC2 - INP16) - this channel is used as a fast constant current limitation during PT operations
 *  - the supply voltage		(PA05, ADC1 - INP19)
 *  - the temperature sensor	(ADC3 - INP18)
 *  - the internal reference	(ADC3 - INP19)
 *
 * There are some minimum sampling time requirements for the channels:
 *  - normal channels: 1,5 ADclk
 *  - temperature sensor: 9,0µs
 *  - internal reference: 4,3µs
 *  - Vbat reference: 9,0µs (not used here)
 *
 * We will let run the ADC quite slow with ADclk = 4MHz (we are not in a hurry!).
 * With 4MHz, one ADclk is 0,25µs, so sampling time with 1,5 ADclk is only 0,375µs.
 * To give all channels an equal sampling time, we choose the sampling time
 * according to the above requirements:
 *  - normal channels:		8,5 ADclk (more than neccessary, but might give better results)
 *  - internal reference:	32,5 ADclks (need 17,2 but the next lower value would be only 16,5)
 *  - temperatur sensor:	64,5 ADclks (need 36 but the next lower value would be only 32,5)
 *
 * The conversion itself needs 8,5 ADclk for 16 bit resolution. A single conversion
 * takes the following time:
 *  - normal channels:		8,5 + 8,5 = 17 ADclk = 4,25µs
 *  - internal reference:	32,5 + 8,5 = 41 ADclks = 10,25µs
 *  - temperatur sensor:	64,5 + 8,5 = 73 ADclks = 18,25µs
 *
 * To get rid of some noise on external channels, we will use a 64 times oversampling
 * and finally shifting the result right by 3 bits. This expands the result to 19 bits.
 * The internal channels (Vrefint + temperature sensor) are oversampled 16 times and
 * shifted right by 1 which also yields 19 bits.
 * For a single cycle, we will convert two standard channels (Usupply + Itrack), the
 * temperature sensor and internal voltage reference (to measure the VDDA / Vref+ and
 * finally have quite exact measure of the other channels). The total time this takes
 * is calculated as follow:
 *   t = 2*(64*4,25) + 16*10,25 + 16*18,25 = 1.000µs -> 1ms
 *
 * Programming track constant current regulation:
 * This job needs a real fast A/D conversion of the track current. ADC2 in combination
 * with an analog watchdog (aka. window comparator) is used for that purpose.
 * Because all ADCs are clocked by the same prescaler, ADC2 will also use 4MHz.
 * The track current will be aquired with 16 bits, no oversampling and the minimum
 * sampling time of 1,5 A/D-ticks. The conversion itself is done in another 8,5 A/D
 * ticks and so a conversion uses 10 A/D tick which will give us a free running
 * conversion rate of 400kHz (2,5µs/conversion).
 * The resulting values will not be used directly but fed into the analog window
 * watchdog which will trigger interrupts, when the conversion result shows a value
 * above a certain threshold (programmable).
 *
 * Undervoltage shutdown: The input voltage is permanently observed in the 1ms A/D cycle.
 * If a single measurement is beloq a certain threshold (UIN_MIN), the shutdown is triggered.
 * This behavior is only enabled once the supply is relatively stable above a threshold of
 * UIN_OK.
 *
 * Temperature: If the internal temperature measurement show > 75°C, we shut off the
 * track and wait until the temperature goes below 70°C.
 */

#define SWAP_UIN_ISENSE

#define ADC_QUEUE_LENGTH	200			///< how much values will be used to filter the analog value
#define FULL_SCALE			(1 << 19)	///< 19 bits full scale for all channels
#define FULL_SCALE16		(1 << 16)	///< native full scale value for 16 bits
#define FACTOR_UIN			11			///< the inputvoltage is divided by this factor before A/D conversion (10k + 1k)
#define FACTOR_ITRACK_HW00	450			///< initial config: RItrack = 15mR, Gopamp = 31,3 => 470mV/A, but we go with 450mV/A (+4,25%)
#define FACTOR_ITRACK_HW16	320			///< newer config: RItrack = 12mR, Gopamp = 28,6 => 343mV/A, but we go with 320mV/A (+6,7%)
#define MAX_DAC				3300		///< for CC-Mode: this is a voltage close to 1V

// access to STs factory calibrated values
#define TS_CAL1				(*((uint16_t *) 0x1FF1E820))		///< calibrated temperature reading @ 30°C
#define TS_CAL2				(*((uint16_t *) 0x1FF1E840))		///< calibrated temperature reading @ 110°C
#define TS_CAL_LOW			30			///< 30°C is the lower temperature calibration point
#define TS_CAL_DIFF			80			///< 80K between the two temperature calibration points
#define TS_OFFSET			17			///< subtract some Kelvin to get closer to ambient (sensor is measuring the DIE temperature!)
#define VREF_INT_CAL		(*((uint16_t *) 0x1FF1E860))		///< calibratied readout @ 3,3V Vref / VDDA
#define VREF_CAL_VDDA		3300		///< the Vref voltage in mV used to acquire the calibration reading (3,3V)

#define UIN_MIN				20000		///< a level where we signal a powerfail
#define UIN_OK				22000		///< a level where we take UIN as OK
#define TEMP_SHUTDOWN		75			///< shut the system down on this temperature (in °C)
#define TEMP_COOLDOWN		70			///< re-enable the system on this temperature (in °C)

// All A/D results in mV measured at the I/O pin. Individual factors will be applied later
static volatile uint32_t track_current[ADC_QUEUE_LENGTH];
static volatile uint32_t supply_voltage[ADC_QUEUE_LENGTH];
static volatile uint32_t temp_sensor[ADC_QUEUE_LENGTH];
static volatile uint32_t ref_voltage[ADC_QUEUE_LENGTH];
static volatile int adc_idx;
static volatile int supply, temperature, itrack;
static TaskHandle_t analog_task;
static volatile int temp_testoff;			///< an internal temperature offset for testing purposes

struct event_check {
	TickType_t			lastevent;		///< the time of the last event (to output unchanged data regularly)
};

static void adc_channelSampling (ADC_TypeDef *adc, int ch, int samples)
{
	int shift;

	if (ch < 0 || ch > 19) return;
	samples &= 0b111;		// only three bits are valid

	if (ch <= 9) {
		shift = 3 * ch;
		MODIFY_REG (adc->SMPR1, 0b111 << shift, samples << shift);
	} else {
		shift = 3 * (ch - 10);
		MODIFY_REG (adc->SMPR2, 0b111 << shift, samples << shift);
	}
}

static void adc_calibration (ADC_TypeDef *adc, char *step)
{
	TickType_t cal_to;
//	TickType_t start;

	cal_to = tim_timeout(250);									// a standard calibration should last about 41ms
//	start = xTaskGetTickCount();
	SET_BIT (adc->CR, ADC_CR_ADCAL);							// start calibration
	while (!tim_isover(cal_to) && adc->CR & ADC_CR_ADCAL) {		// wait until ADC is ready
		vTaskDelay(2);
	}
	if (adc->CR & ADC_CR_ADCAL) {
		fprintf (stderr, "%s(): Calibration %s aborted\n", __func__, step);
		CLEAR_BIT(adc->CR, ADC_CR_ADCAL);		// according to datasheet this is not allowed - we give it a try, though
//	} else {
//		log_msg (LOG_INFO, "%s(): %s done in %lums\n", __func__, step, xTaskGetTickCount() - start);
	}
}

static void adc_init (void)
{
	ADC1->CR = 0;					// stop deep power down mode
	ADC2->CR = 0;					// stop deep power down mode
	ADC3->CR = 0;					// stop deep power down mode
	ADC1->CR = ADC_CR_ADVREGEN;		// now enable the ADC voltage regulator and wait the stabilisation time (10µs)
	ADC2->CR = ADC_CR_ADVREGEN;		// now enable the ADC voltage regulator and wait the stabilisation time (10µs)
	ADC3->CR = ADC_CR_ADVREGEN;		// now enable the ADC voltage regulator and wait the stabilisation time (10µs)
	vTaskDelay(50);

	if (cpu.revcode == 'Y') {	// rev. 'Y' doesn't have the additional divider (/2)
		// we will use HSI 64MHz / 16 => 4MHz ADC clock, so set boost level to 0b00
		ADC12_COMMON->CCR = (0b111 << ADC_CCR_PRESC_Pos) | (0b00 << ADC_CCR_CKMODE_Pos);
		ADC3_COMMON->CCR = ADC_CCR_TSEN | ADC_CCR_VREFEN | (0b111 << ADC_CCR_PRESC_Pos) | (0b00 << ADC_CCR_CKMODE_Pos);
	} else {
		// we will use HSI 64MHz / 8 / 2 => 4MHz ADC clock, so set boost level to 0b00
		ADC12_COMMON->CCR = (0b100 << ADC_CCR_PRESC_Pos) | (0b00 << ADC_CCR_CKMODE_Pos);
		ADC3_COMMON->CCR = ADC_CCR_TSEN | ADC_CCR_VREFEN | (0b100 << ADC_CCR_PRESC_Pos) | (0b00 << ADC_CCR_CKMODE_Pos);
	}
	MODIFY_REG(ADC1->CR, ADC_CR_BOOST_Msk, 0b00 << ADC_CR_BOOST_Pos);
	MODIFY_REG(ADC3->CR, ADC_CR_BOOST_Msk, 0b00 << ADC_CR_BOOST_Pos);

	// ADC1: trigger a calibration sequence (single ended inputs + linear)
	CLEAR_BIT (ADC1->CR, ADC_CR_ADCALDIF);
	SET_BIT (ADC1->CR, ADC_CR_ADCALLIN);
	adc_calibration(ADC1, "ADC1 (single ended)");

	// ADC1: trigger a calibration sequence (differential inputs)
	CLEAR_BIT (ADC1->CR, ADC_CR_ADCALLIN);
	SET_BIT (ADC1->CR, ADC_CR_ADCALDIF);
	adc_calibration(ADC1, "ADC1 (differential)");
	CLEAR_BIT (ADC1->CR, ADC_CR_ADCALDIF);

	// ADC2: trigger a calibration sequence (single ended inputs + linear)
	CLEAR_BIT (ADC2->CR, ADC_CR_ADCALDIF);
	SET_BIT (ADC2->CR, ADC_CR_ADCALLIN);
	adc_calibration(ADC2, "ADC2 (single ended)");

	// ADC2: trigger a calibration sequence (differential inputs)
	CLEAR_BIT (ADC2->CR, ADC_CR_ADCALLIN);
	SET_BIT (ADC2->CR, ADC_CR_ADCALDIF);
	adc_calibration(ADC2, "ADC2 (differential)");
	CLEAR_BIT (ADC2->CR, ADC_CR_ADCALDIF);

	// ADC3: trigger a calibration sequence (single ended inputs + linear)
	CLEAR_BIT (ADC3->CR, ADC_CR_ADCALDIF);
	SET_BIT (ADC3->CR, ADC_CR_ADCALLIN);
	adc_calibration(ADC3, "ADC3 (single ended)");
	SET_BIT (ADC3->CR, ADC_CR_ADCAL);		// start calibration
	while (ADC3->CR & ADC_CR_ADCAL) /* wait until ADC is ready */;

	// ADC3: trigger a calibration sequence (differential inputs)
	CLEAR_BIT (ADC3->CR, ADC_CR_ADCALLIN);
	SET_BIT (ADC3->CR, ADC_CR_ADCALDIF);
	adc_calibration(ADC3, "ADC3 (differential)");
	CLEAR_BIT (ADC3->CR, ADC_CR_ADCALDIF);

	// enable the ADCs
	ADC1->ISR = ADC_ISR_ADRDY;			// clear ready bit
	ADC2->ISR = ADC_ISR_ADRDY;			// clear ready bit
	ADC3->ISR = ADC_ISR_ADRDY;			// clear ready bit
	SET_BIT (ADC1->CR, ADC_CR_ADEN);	// enable ADC
	SET_BIT (ADC2->CR, ADC_CR_ADEN);	// enable ADC
	SET_BIT (ADC3->CR, ADC_CR_ADEN);	// enable ADC
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* wait for ADC1 to be enabled */;
	while ((ADC2->ISR & ADC_ISR_ADRDY) == 0) /* wait for ADC2 to be enabled */;
	while ((ADC3->ISR & ADC_ISR_ADRDY) == 0) /* wait for ADC3 to be enabled */;

	// 64x oversampling, shift 3 bit right after oversampling, convert channels 16 + 19
	ADC1->CFGR2 = (63 << ADC_CFGR2_OVSR_Pos) | (3 << ADC_CFGR2_OVSS_Pos) | ADC_CFGR2_ROVSE;
	adc_channelSampling(ADC1, 16, 0b010);	// Tsamp = 8,5 ADclk
	adc_channelSampling(ADC1, 19, 0b010);	// Tsamp = 8,5 ADclk
	ADC1->PCSEL = ADC_PCSEL_PCSEL_19 | ADC_PCSEL_PCSEL_16;
	ADC1->SQR1 = (19 << ADC_SQR1_SQ2_Pos) | (16 << ADC_SQR1_SQ1_Pos) | (1 << ADC_SQR1_L_Pos);
	ADC1->CFGR = /* ADC_CFGR_CONT | */ ADC_CFGR_OVRMOD;

	// no oversampling, no shift, convert channels 16, continuous mode
	ADC2->CFGR2 = 0;						// all oversampling and shift operations inhibited
	adc_channelSampling(ADC2, 16, 0b000);	// Tsamp = 1,5 ADclk
	ADC2->PCSEL = ADC_PCSEL_PCSEL_19;
	ADC2->SQR1 = (19 << ADC_SQR1_SQ1_Pos) | (0 << ADC_SQR1_L_Pos);
	ADC2->CFGR = ADC_CFGR_CONT | ADC_CFGR_OVRMOD;	// continuous conversion, data register is overwritten with the latest conversion result
	ADC2->CFGR |= (19 << ADC_CFGR_AWD1CH_Pos) | ADC_CFGR_AWD1EN;	// analog watchdog 1 for channel 19
	ADC2->AWD2CR = ADC_AWD2CR_AWD2CH_19;							// analog watchdog 2 for channel 19
	SET_BIT (ADC2->CR, ADC_CR_ADSTART);								// always let ADC2 run

	// 16x oversampling, shift 1 bit right after oversampling, convert channels 18 + 19
	ADC3->CFGR2 = (15 << ADC_CFGR2_OVSR_Pos) | (1 << ADC_CFGR2_OVSS_Pos) | ADC_CFGR2_ROVSE;
	adc_channelSampling(ADC3, 18, 0b101);	// Tsamp = 64,5 ADclk
	adc_channelSampling(ADC3, 19, 0b100);	// Tsamp = 32,5 ADclk
	ADC3->PCSEL = ADC_PCSEL_PCSEL_19 | ADC_PCSEL_PCSEL_18;
	ADC3->SQR1 = (19 << ADC_SQR1_SQ2_Pos) | (18 << ADC_SQR1_SQ1_Pos) | (1 << ADC_SQR1_L_Pos);
	ADC3->CFGR = /* ADC_CFGR_CONT | */ ADC_CFGR_OVRMOD;

	NVIC_SetPriority (ADC_IRQn, 4);		// give it a quite high priority to allow a fast reaction to overcurrent events (ADC2)
	NVIC_ClearPendingIRQ (ADC_IRQn);
	NVIC_EnableIRQ (ADC_IRQn);
	ADC1->IER = ADC_IER_EOCIE;

	NVIC_SetPriority (ADC3_IRQn, 14);
	NVIC_ClearPendingIRQ (ADC3_IRQn);
	NVIC_EnableIRQ (ADC3_IRQn);
	ADC3->IER = ADC_IER_EOCIE;
}

static uint32_t an_average (volatile uint32_t *vals)
{
	uint32_t sum;
	int i;

	sum = 0;
	for (i = 0; i < ADC_QUEUE_LENGTH; i++) {
		sum += vals[i];
	}

	sum = (sum + ADC_QUEUE_LENGTH / 2) / ADC_QUEUE_LENGTH;
	return sum;
}

int an_getSupply (void)
{
	return supply;
}

int an_getTemperature (void)
{
	return temperature;
}

int an_getTrackCurrent (void)
{
	return itrack;
}

int an_getProgCurrent (int samples)
{
	int i, idx, sum;

	sum = 0;
	idx = adc_idx - 1;
	for (i = 0; i < samples; i++) {
		if (--idx < 0) idx = ADC_QUEUE_LENGTH - 1;
		sum += track_current[idx];
	}

	sum = (sum * 1000 + FACTOR_ITRACK_HW00 / 2) / FACTOR_ITRACK_HW00;
	return (sum + samples / 2) / samples;
}

void an_temperaturTest (int newoffs)
{
	temp_testoff = newoffs;
}

void vAnalog (void *pvParameter)
{
	TickType_t lastevent, lastenviron;
	int vref, uin, uin_unfiltered;
	int itrack_last, iInst, fTrackCurrent;
	int ts1_mv, ts2_mv, temp, i, pwr;
	bool pwr_ok, temp_ok, power_up, power_state = false;

	(void) pvParameter;

	analog_task = xTaskGetCurrentTaskHandle();
	adc_init();

	ts1_mv = (TS_CAL1 * VREF_CAL_VDDA + FULL_SCALE16 / 2) / FULL_SCALE16;
	ts2_mv = (TS_CAL2 * VREF_CAL_VDDA + FULL_SCALE16 / 2) / FULL_SCALE16;
	adc_idx = 0;
	pwr_ok = power_up = false;

	printf("%s() ready (TS 30°C=%dmV 110°C=%dmV)\n", __func__, ts1_mv, ts2_mv);

	itrack = itrack_last = uin = temp = 0;
	lastevent = lastenviron = xTaskGetTickCount();
	for (i = 0; i < ADC_QUEUE_LENGTH; i++) {	// pre-init to dummy 30°C
		temp_sensor[i] = ts1_mv;
	}
	temp_ok = true;
	pwr = 0;

	if (hwinfo->HW >= HW16) {
		fTrackCurrent = FACTOR_ITRACK_HW16;
	} else {
		fTrackCurrent = FACTOR_ITRACK_HW00;
	}

//	while (xTaskGetTickCount() < 2000) {		// wait a while before really using the A/D results
//		SET_BIT (ADC1->CR, ADC_CR_ADSTART);				// start ADC1, it will trigger ADC3 when finished
//		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));	// just let the A/D converter run and forget about it's results
//	}

	for (;;) {
		SET_BIT (ADC1->CR, ADC_CR_ADSTART);		// start ADC1, it will trigger ADC3 when finished
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)) != 0) {	// we really are notified (not a timeout)
			// first calculate the real current Vref/VDDA value of the current sample
			vref = ref_voltage[adc_idx];
			vref = ((VREF_CAL_VDDA * (VREF_INT_CAL << 3)) + vref / 2) / vref;	// vref is calculated in mV
			ref_voltage[adc_idx] = vref;

			// now convert all actual measurements to mV using actual measured Vref/VDDA and individual A/D result using only the current sample
			supply_voltage[adc_idx] = (supply_voltage[adc_idx] * vref + FULL_SCALE / 2) / FULL_SCALE;

			// calcuate track current and send an immediate update to check for overcurrent situation
			track_current[adc_idx] = (track_current[adc_idx] * vref + FULL_SCALE / 2) / FULL_SCALE;
			iInst = (track_current[adc_idx] * 1000 + fTrackCurrent / 2) / fTrackCurrent;
			event_fire(EVENT_INSTANEOUS_CURRENT, iInst, NULL);

			temp_sensor[adc_idx] = (temp_sensor[adc_idx] * vref + FULL_SCALE / 2) / FULL_SCALE;

			temp = ((int) an_average(temp_sensor) - ts1_mv) * TS_CAL_DIFF / (ts2_mv - ts1_mv) + TS_CAL_LOW - TS_OFFSET + temp_testoff;
			itrack = (an_average(track_current) * 1000 + fTrackCurrent / 2) / fTrackCurrent;
			uin = an_average(supply_voltage) * FACTOR_UIN;
			uin_unfiltered = supply_voltage[adc_idx] * FACTOR_UIN;

			if (!power_up && xTaskGetTickCount() > 2000) {	// OK, the first few seconds are gone, now it counts!
				power_up = true;
				if (!pwr_ok) log_msg (LOG_WARNING, "%s() Power supply is lower than expected (%d.%dV)\n", __func__, uin / 1000, (uin / 100) % 10);
			}
			if (!power_up) temp = 30;			// during the first time after a real cold boot, the temperature sensor is unreliable

			// check for power fail event
			if (pwr_ok) {
				if (uin_unfiltered < UIN_MIN) {
//					log_msg (LOG_WARNING, "Uin = %d.%03d\n", uin_unfiltered / 1000, uin_unfiltered % 1000);
					if(pwr == 0) power_state = MAINBST_ISON();
					pwr++;
					if (pwr == 3) sig_setMode(TM_SHORT);
					if (pwr >= 40) pwrfail();
//					pwr_ok = false;
				} else {
					pwr = 0;
					if(power_state) MAINBST_ON();
				}
			} else {
				if (uin_unfiltered > UIN_OK && uin > UIN_OK) {
					pwr = 0;
					pwr_ok = true;
				}
			}

			if (++adc_idx >= ADC_QUEUE_LENGTH) adc_idx = 0;
		}

		if (temp_ok) {
			if (temp >= TEMP_SHUTDOWN) {
				log_msg(LOG_WARNING, "%s() Temperature rises to %d\n", __func__, temp);
				temp_ok = false;
				sig_setMode(TM_OVERTTEMP);
			}
		} else {
			if (temp <= TEMP_COOLDOWN) {
				log_msg(LOG_WARNING, "%s() Temperature now uncritical %d\n", __func__, temp);
				temp_ok = true;
				sig_setMode(TM_TEMPOK);
			}
		}

		// check if we should post an update event for current consumption
		if ((xTaskGetTickCount() - lastevent > 500 && ((itrack + 50) / 100 != itrack_last)) || (xTaskGetTickCount() - lastevent) > 5000) {
			itrack_last = (itrack + 50) / 100;		// itrack_last holds the last track current reported rounded to the nearest 100mA
			event_fire(EVENT_CURRENT, itrack_last, NULL);
			lastevent = xTaskGetTickCount();
		}

		// check if we should post an update event for supply voltage or temperature
		if ((xTaskGetTickCount() - lastenviron) > 1000 && (uin > (supply + 80) || uin < (supply - 80) || temp != temperature)) {
			supply = uin;
			temperature = temp;
			event_fire(EVENT_ENVIRONMENT, 0, NULL);
			lastenviron = xTaskGetTickCount();
		}
	}
}

/*
 * ===========================================================================================
 * Fast current detection for current limiter and programming ACK recognition
 * ===========================================================================================
 */

static void (*ACK_callback)(int);

/**
 * Configure ADC2 (INP16) to regulate the current by manipulation the output voltage.
 * To switch off constant current regulation, call this function with a current of 0.
 *
 * \param current	the current level that is to be maintained in a CC fashion (in mA)
 */
void adc_CCmonitor (int current)
{
	if ((ADC2->CR & ADC_CR_ADSTART) && !(ADC2->CR & ADC_CR_ADDIS)) {
		SET_BIT (ADC2->CR, ADC_CR_ADSTP);
		while (ADC2->CR & ADC_CR_ADSTP) taskYIELD();	// wait for the ADC to really be stopped
	}
	if (current == 0) {
		CLEAR_BIT (ADC2->IER, ADC_IER_AWD1IE);	// disable the interrupt
	} else {
		ADC2->LTR1 = 0;
		if (hwinfo->HW >= HW16) {
			ADC2->HTR1 = current * 6;				// one A/D tick is around 146,7µA, 1mA / 146,7µA = 6,817 => factor 6
		} else {
			ADC2->HTR1 = current * 9;				// one A/D tick is around 107,3µA, 1mA / 107,3µA = 9,324 => factor 9
		}
		SET_BIT (ADC2->IER, ADC_IER_AWD1IE);	// enable the interrupt
	}
	if (ADC2->IER != 0) SET_BIT (ADC2->CR, ADC_CR_ADSTART);		// restart ADC2
}

/**
 * Specify an ACK current level for programming track observation.
 * The callback function is called for every A/D-sample, that is above the ACK
 * threshold. The parameter will be the raw A/D converter value (about 9 ticks per mA)
 *
 * ATTENTION: the callback is called in the interrupt context of the A/D-Interrupt
 * and must be very short, because this interrupt may fire at 400kHz. You should,
 * for example, only increment a variable to see how often that current threshold
 * was tripped.
 *
 * \param current	the current level for ACK threshold (in mA)
 * \param cb		the callback function, called every time the A/D result is above the programmed threshold
 */
void adc_ACKcurrent (int current, void (*cb)(int))
{
	if ((ADC2->CR & ADC_CR_ADSTART) && !(ADC2->CR & ADC_CR_ADDIS)) {
		SET_BIT (ADC2->CR, ADC_CR_ADSTP);
		while (ADC2->CR & ADC_CR_ADSTP) taskYIELD();	// wait for the ADC to really be stopped
	}
	ACK_callback = cb;
	if (current == 0 || cb == NULL) {
		CLEAR_BIT (ADC2->IER, ADC_IER_AWD2IE);	// disable the interrupt
	} else {
		ADC2->LTR2 = 0;
		ADC2->HTR2 = current * 9;				// one A/D tick is around 110µA
		SET_BIT (ADC2->IER, ADC_IER_AWD2IE);	// enable the interrupt
	}
	SET_BIT (ADC2->CR, ADC_CR_ADSTART);		// start the ADC2 if any interrupts are enabled for it
}

/**
 * Read a rough estimate of the track current (in mA) without any filtering.
 * This function is used by the DCC programming track mechanism.
 *
 * \return		the instaneous, unfiltered track current in mA
 */
int adc_getFastCurrent (void)
{
	return (ADC2->DR + 4) / 9;
}

void ADC1_2_IRQHandler (void)
{
	static int chidx = 0, ovr;

	int dac;

	// current limiter / over current regulation
	if ((ADC2->IER & ADC_IER_AWD1IE) && (ADC2->ISR & ADC_ISR_AWD1)) {
//		if (MKLNBST_ISON()) MKLNBST_OFF();
//		else MKLNBST_ON();
		ADC2->ISR = ADC_ISR_AWD1;
		dac = DAC1->DOR1;
		dac += (MAX_DAC - dac) >> 7;
		DAC1->DHR12R1 = dac;					// lower output voltage
	}

	// programming track ACK
	if ((ADC2->IER & ADC_IER_AWD2IE) && (ADC2->ISR & ADC_ISR_AWD2)) {
		ADC2->ISR = ADC_ISR_AWD2;
		if (ACK_callback) ACK_callback(ADC2->DR);
	}

	if (ADC1->ISR & ADC_ISR_OVR) {
		irqdbg_printf("%s() ADC1 Overrun\n", __func__);
		ADC1->ISR = ADC_ISR_OVR;
		ovr++;
	}
	if ((ADC1->IER & ADC_IER_EOCIE) && (ADC1->ISR & ADC_ISR_EOC)) {
#ifdef SWAP_UIN_ISENSE
		if (chidx == 0) supply_voltage[adc_idx] = ADC1->DR;
		if (chidx == 1) track_current[adc_idx] = ADC1->DR;
#else
		if (chidx == 0) track_current[adc_idx] = ADC1->DR;
		if (chidx == 1) supply_voltage[adc_idx] = ADC1->DR;
#endif
		ADC1->ISR = ADC_ISR_EOC;
		chidx++;

		if (ADC1->ISR & ADC_ISR_EOS) {		// this is no extra interrupt, we only trigger on EOC!
			ADC1->ISR = ADC_ISR_EOS;
			chidx = 0;
			SET_BIT (ADC3->CR, ADC_CR_ADSTART);		// now start the ADC3
		}
	}

	NVIC_ClearPendingIRQ(ADC_IRQn);
}

void ADC3_IRQHandler (void)
{
	static int chidx = 0, ovr;

	BaseType_t xHigherPriorityTaskWoken = 0;

	if (ADC3->ISR & ADC_ISR_OVR) {
		irqdbg_printf("%s() ADC3 Overrun\n", __func__);
		ADC3->ISR = ADC_ISR_OVR;
		ovr++;
	}
	if ((ADC3->IER & ADC_IER_EOCIE) && (ADC3->ISR & ADC_ISR_EOC)) {
		ADC3->ISR = ADC_ISR_EOC;
		if (chidx == 0) temp_sensor[adc_idx] = ADC3->DR;
		if (chidx == 1) ref_voltage[adc_idx] = ADC3->DR;
		chidx++;
		if (ADC3->ISR & ADC_ISR_EOS) {		// this is no extra interrupt, we only trigger on EOC!
			ADC3->ISR = ADC_ISR_EOS;
			chidx = 0;
		    vTaskNotifyGiveFromISR (analog_task, &xHigherPriorityTaskWoken);	// wake up main thread
		}
	}

	NVIC_ClearPendingIRQ(ADC3_IRQn);
    portEND_SWITCHING_ISR (xHigherPriorityTaskWoken);
}
