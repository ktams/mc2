/**
 * @file tracksupply.c
 *
 * @author Andi
 * @date   09.04.2020
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

#include <stdio.h>
#include <string.h>
#include "rb2.h"
#include "events.h"
#include "config.h"
#include "decoder.h"

#define FB_VOLTAGE					12		///< the feedback voltage of the regulator is 1,2V nominal
#define REF_VOLTAGE					33		///< the reference voltage of the D/A converter is 3,3V nominal
#define DA_STEPS					4096	///< we have a 12 bit D/A converter
#define DA_PASSIVE					(FB_VOLTAGE * DA_STEPS / REF_VOLTAGE)	///< the D/A-ticks to make output control passive
#define PASSIVE_VOLTAGE				132		///< the output voltage if no additional current flows into the feedback (i.e.DAC = FB_VOLTAGE, in 0,1V)
#define DACSTEPS_PER_MS				20		///< how much DAC steps we do per call of ts_handler() (i.e. per 1ms)

static struct {
	volatile int		target_voltage;		///< the desired track voltage
	volatile int		program_voltage;	///< the desired track voltage for the programming track
	volatile int		max_current;		///< the maximum current allowed on track
	volatile int		current_limiter;	///< if set > 0 the track current is limited in a CC fashion (for programming track)
	volatile int		inrush_time;		///< a blanking time in ms for short recognition when turning on booster
	volatile int		short_time;			///< a time in ms after which a short is reported if current is beyond the limit
} boosterconfig;

static struct {
	volatile uint32_t	target_dac;			///< the calculated target voltage setting in DAC ticks
	int					actual_voltage;		///< the real voltage in 0,1V on the booster output (as set by D/A converter)
	volatile int		actual_current;		///< the track current in mA as reported by A/D converter (via event)
	volatile int		inrush_time;		///< the time that already passed when switching on booster
	volatile int		short_time;			///< the time that already passed when an overcurrent is detected
	volatile bool		booster_on;			///< the requested status of the booster output
	volatile bool		prog_track;			///< if set, the booster should be switched on for the programming track
} boosterstatus;

static bool ts_currentMonitor (eventT *e, void *arg)
{
	(void) arg;

	// this event should reach us every ms
	if (e->ev == EVENT_INSTANEOUS_CURRENT) {
		boosterstatus.actual_current = e->param;	// current in events is reported in mA
		if (MAINBST_ISON()) {
			if (boosterstatus.inrush_time > 0) boosterstatus.inrush_time--;
			if (boosterstatus.inrush_time <= 0) {	// from now on, we monitor the current for overcurrent conditions
				if (boosterstatus.actual_current > boosterconfig.max_current) boosterstatus.short_time += 2;
				else if (boosterstatus.short_time > 0) boosterstatus.short_time--;
				if (boosterstatus.short_time > (boosterconfig.short_time * 2)) {
					sig_setMode(TM_SHORT);
					fprintf (stderr, "%s(): SHORT @%dmA\n", __func__, boosterstatus.actual_current);
				}
			}
		}
	}
	return true;		// we continue to listen for events!
}

/**
 * Calculate the D/A output value for the desired out voltage (in 0,1V)
 *
 * \param volt		the desired output voltage for the bootster
 * \return			the value to program to D/A output register to achive this voltage
 */
static uint32_t ts_volt2dac (int volt)
{
	int diff;

	diff = PASSIVE_VOLTAGE - volt;		/* Positive for lowering the output voltage, negative to raise it.
										   This is the amount of µA that should be fed into the feedback path */
	diff = (diff * 149 + 5) / 10;		// 1µA is 14,9 D/A-ticks
	diff += DA_PASSIVE;
	if (diff < 0) diff = 0;
	if (diff >= DA_STEPS) diff = DA_STEPS - 1;
	return (uint32_t) diff;
}

/**
 * Return a string in JSON format with the ranges for settings regarding
 * the physical track parameters (voltages, current, timeouts).
 *
 * \return		a JSON string with ranges
 */
char *ts_getRanges (void)
{
	static char response[256];

	int maxcurrent;

	if (!*response) {	// first call -> fill the string
		switch (hwinfo->manufacturer) {
			case DCC_MANUFACTURER_TAMS:
				maxcurrent = MAX_CURRENT_TAMS;
				break;
			default:
				maxcurrent = MAX_CURRENT_KM1;
				break;
		}
		sprintf (response, "{ \"track\": { \"umin\": %d, \"umax\": %d, \"imin\": %d, \"imax\": %d,"
				" \"sensmin\": %d, \"sensmax\": %d, \"inrushmin\": %d, \"inrushmax\": %d,"
				" \"prgumin\": %d, \"prgumax\": %d }}\n",
				MIN_VOLTAGE, MAX_VOLTAGE, MIN_CURRENT / 100, maxcurrent / 100, MIN_SENSITIVITY, MAX_SENSITIVITY,
				MIN_INRUSH, MAX_INRUSH, MIN_PRGVOLTAGE, MAX_PRGVOLTAGE);
	}
	return response;
}

/**
 * Set desired track voltage in 1/10V
 *
 * \param volt		the desired track voltage in 100mV steps
 * \param pt		if true, the voltage is ment for the programming track (else standard output)
 */
static void _ts_setVoltage (int volt, bool pt)
{
	if (pt) {
		boosterconfig.program_voltage = volt;
		if (PRGRELAIS_ISON()) boosterstatus.target_dac = ts_volt2dac(volt);
	} else {
		boosterconfig.target_voltage = volt;
		if (!PRGRELAIS_ISON()) boosterstatus.target_dac = ts_volt2dac(volt);
	}
	if (pt) {
		printf("%s(PT) %d.%dV\n", __func__, boosterconfig.program_voltage / 10, boosterconfig.program_voltage % 10);
	} else {
		printf("%s() %d.%dV\n", __func__, boosterconfig.target_voltage / 10, boosterconfig.target_voltage % 10);
	}
	event_fire (EVENT_BOOSTER, 0, NULL);
}

/**
 * Set desired programming track voltage in 1/10V
 *
 * \param volt		the desired track voltage in 100mV steps
 * \return			the voltage that really is set (trimmed down to allowed values)
 */
int ts_setVoltage (int volt)
{
	if (volt < MIN_VOLTAGE) volt = MIN_VOLTAGE;
	if (volt > MAX_VOLTAGE) volt = MAX_VOLTAGE;
	_ts_setVoltage(volt, false);
	return volt;
}

/**
 * Set desired track voltage in 1/10V
 *
 * \param volt		the desired track voltage in 100mV steps
 */
void ts_setPtVoltage (int volt)
{
	if (volt < MIN_PRGVOLTAGE) volt = MIN_PRGVOLTAGE;
	if (volt > MAX_PRGVOLTAGE) volt = MAX_PRGVOLTAGE;
	_ts_setVoltage(volt, true);
}

/**
 * Get the currently set track voltage in 1/10V
 *
 * \return			the currently programmed track voltage in 1/10V
 */
int ts_getVoltage (void)
{
	return boosterconfig.target_voltage;
}

/**
 * Get the currently set programming track voltage in 1/10V
 *
 * \return			the currently programmed track voltage in 1/10V set for the programming track
 */
int ts_getPtVoltage (void)
{
	return boosterconfig.program_voltage;
}

static void ts_overcurrentProtection (int maxcurrent)
{
	if (maxcurrent < MIN_CURRENT) maxcurrent = MIN_CURRENT;
	maxcurrent += 500;		// 500mA on top of setting
	switch (hwinfo->manufacturer) {
		case DCC_MANUFACTURER_TAMS:
			if (maxcurrent > SHORT_CURRENT_TAMS) maxcurrent = SHORT_CURRENT_TAMS;
			break;
		default:	// currently, KM-1 uses public domain manufacturer code. So treat all NON-Tams as KM-1.
			if (maxcurrent > SHORT_CURRENT_KM1) maxcurrent = SHORT_CURRENT_KM1;
			break;
	}
	adc_CCmonitor(maxcurrent);
}

/**
 * Set the track current limit for short recognition in mA
 *
 * \param current	the current limit for track operation in mA
 * \return			the new value taken as max. booster current trimmed to allowed values
 */
int ts_setCurrentMilliAmpere (int current)
{
	if (current < MIN_CURRENT) current = MIN_CURRENT;
	switch (hwinfo->manufacturer) {
		case DCC_MANUFACTURER_TAMS:
			if (current > MAX_CURRENT_TAMS) current = MAX_CURRENT_TAMS;
			break;
		default:
			if (current > MAX_CURRENT_KM1) current = MAX_CURRENT_KM1;
			break;
	}
	boosterconfig.max_current = current;
	printf("%s() %d.%dA\n", __func__, boosterconfig.max_current / 1000, (boosterconfig.max_current / 100) % 10);
	if (MAINBST_ISON() && !boosterstatus.prog_track) {
		ts_overcurrentProtection(current);
	}
	event_fire (EVENT_BOOSTER, 0, NULL);
	return current;
}

/**
 * Set the track current limit for short recognition in 0,1A
 *
 * \param current	the current limit for track operation in 100mA steps
 * \return			the new value taken as max. booster current trimmed to allowed values
 */
int ts_setCurrent (int current)
{
	return (ts_setCurrentMilliAmpere(current * 100) + 50) / 100;
}

void ts_setCCMode (int limit)
{
	if (limit == 0) {
		boosterconfig.current_limiter = 0;
	} else {
		limit *= 100;
		if (limit > MAX_LIMITER) limit = MAX_LIMITER;
		boosterconfig.current_limiter = limit;
	}
}

/**
 * Get the currently set short level in mA
 *
 * \return		the currently programmed short level in mA
 */
int ts_getCurrentMilliAmpere (void)
{
	return boosterconfig.max_current;
}

/**
 * Get the currently set short level in 0,1A
 *
 * \return		the currently programmed short level in 0,1A
 */
int ts_getCurrent (void)
{
	return (ts_getCurrentMilliAmpere() + 50) / 100;
}

/**
 * Set the current limit sensitivity in ms
 *
 * \param ms		the time that needs to pass in overcurrent condition before a short is triggered
 */
void ts_setSensitivity (int ms)
{
	if (ms < MIN_SENSITIVITY) ms = MIN_SENSITIVITY;
	if (ms > MAX_SENSITIVITY) ms = MAX_SENSITIVITY;
	boosterconfig.short_time = ms;
	printf("%s() %dms\n", __func__, boosterconfig.short_time);
	event_fire (EVENT_BOOSTER, 0, NULL);
}

/**
 * Get the currently set sensitivity timeout in ms
 *
 * \return		the currently programmed sensitivity timeout in ms
 */
int ts_getSensitivity (void)
{
	return boosterconfig.short_time;
}

/**
 * Set the current inrush timeout in ms
 *
 * \param ms		the time that an overcurrent condition is ignored at track voltage startup
 */
void ts_setInrush (int ms)
{
	if (ms < MIN_INRUSH) ms = MIN_INRUSH;
	if (ms > MAX_INRUSH) ms = MAX_INRUSH;
	boosterconfig.inrush_time = ms;
	printf("%s() %dms\n", __func__, boosterconfig.inrush_time);
	event_fire (EVENT_BOOSTER, 0, NULL);
}

/**
 * Get the currently set inrush timeout in ms
 *
 * \return		the currently programmed inrush timeout in ms
 */
int ts_getInrush (void)
{
	return boosterconfig.inrush_time;
}

/**
 * Check if the set targetvoltage is already reached.
 * If the Main-Booster is not switched on, than the target voltage
 * is taken to be NOT reached. Else, we check the D/A converter
 * against the setting in the booster status structure.
 *
 * \return	true, if main booster is running and the D/A converter outputs a
 * 			value that conforms to the set target voltage. False otherwise.
 */
bool ts_voltageLevelReached (void)
{
	if (!MAINBST_ISON()) return false;
	return (DAC1->DOR1 == boosterstatus.target_dac);
}

void ts_boosteron (bool pt)
{
	int maxcurrent;

	switch (hwinfo->manufacturer) {
		case DCC_MANUFACTURER_TAMS:
			maxcurrent = MAX_CURRENT_TAMS;
			break;
		default:
			maxcurrent = MAX_CURRENT_KM1;
			break;
	}

	event_register(EVENT_INSTANEOUS_CURRENT, ts_currentMonitor, NULL, 0);
	if (boosterstatus.booster_on) {		// protect against switching booster to main track while in programming mode and vice versa
		if (boosterstatus.prog_track == pt) return;		// OK, nothing to do - this was a superflous call
		ts_boosteroff();								// now we can restart with the other track (timings observed in ts_handler())
	}

	// make sure, we have sensefull limits set
	if (boosterconfig.target_voltage < MIN_VOLTAGE) boosterconfig.target_voltage = MIN_VOLTAGE;
	if (boosterconfig.target_voltage > MAX_VOLTAGE) boosterconfig.target_voltage = MAX_VOLTAGE;
	if (boosterconfig.program_voltage < MIN_PRGVOLTAGE) boosterconfig.program_voltage = MIN_PRGVOLTAGE;
	if (boosterconfig.program_voltage > MAX_PRGVOLTAGE) boosterconfig.program_voltage = MAX_PRGVOLTAGE;
	if (boosterconfig.max_current < MIN_CURRENT) boosterconfig.max_current = MIN_CURRENT;
	if (boosterconfig.max_current > maxcurrent) boosterconfig.max_current = maxcurrent;
	if (boosterconfig.short_time < MIN_SENSITIVITY) boosterconfig.short_time = MIN_SENSITIVITY;
	if (boosterconfig.short_time > MAX_SENSITIVITY) boosterconfig.short_time = MAX_SENSITIVITY;
	if (boosterconfig.inrush_time < MIN_INRUSH) boosterconfig.inrush_time = MIN_INRUSH;
	if (boosterconfig.inrush_time > MAX_INRUSH) boosterconfig.inrush_time = MAX_INRUSH;
	boosterstatus.prog_track = pt;
	boosterstatus.target_dac = ts_volt2dac((pt) ? boosterconfig.program_voltage : boosterconfig.target_voltage);
	boosterstatus.booster_on = true;
	// 2022/11/15 A.Kre: why should we only have a limit when on main track?
	// Overcurrent protection should also work with test drive and programming
	/* if (!pt) */ ts_overcurrentProtection(boosterconfig.max_current);
	// real switching will be done in timer callback ts_handler()
}

void ts_boosteroff (void)
{
	MAINBST_OFF();
	boosterstatus.booster_on = false;
	boosterstatus.short_time = 0;
	boosterstatus.inrush_time = 0;
	adc_CCmonitor(0);
}

void ts_init (void)
{
	memset (&boosterstatus, 0, sizeof(boosterstatus));
	memset (&boosterconfig, 0, sizeof(boosterconfig));
	boosterconfig.target_voltage = MIN_VOLTAGE;
	boosterconfig.program_voltage = MIN_PRGVOLTAGE;
	boosterconfig.max_current = MIN_CURRENT;
	boosterconfig.short_time = MIN_SENSITIVITY;
	boosterconfig.inrush_time = MIN_INRUSH;
	event_register(EVENT_INSTANEOUS_CURRENT, ts_currentMonitor, NULL, 0);
}

/**
 * Called by timer hook function to smoothly approach target voltage and
 * handle discharge timeout.
 * This function is called in the interrupt context of the system timer.
 */
void ts_handler (void)
{
	static int timing = 0;
	static int relais_timeout = 0;

	uint32_t dac;

	if (boosterstatus.booster_on) {			// booster is on or should be switched on
		if (!MAINBST_ISON()) {
			if (timing > 0) {				// wait a litte before switching on again
				timing--;
				return;
			}
			// if booster voltage surely has dropped, we may switch the relais
			if (   (boosterstatus.prog_track && !PRGRELAIS_ISON())
				|| (!boosterstatus.prog_track && PRGRELAIS_ISON())) {
				if (boosterstatus.prog_track) PRGRELAIS_ON();
				else PRGRELAIS_OFF();
				timing = RELAIS_TIMEOUT;		// if relais had to be switched, give it some time to do it's job
				return;
			}
			DAC1->DHR12R1 = ts_volt2dac(0);
			MAINBST_ON();
			timing = BOOSTER_TIMEOUT;									// preset timing for next switch off
			if (PRGRELAIS_ISON()) relais_timeout = RELAIS_DISENGAGE;	// preset hold time for PT relais after booster off
			boosterstatus.inrush_time = boosterconfig.inrush_time;
		}
//		if (boosterstatus.inrush_time > 0) boosterstatus.inrush_time--;
//		if (boosterstatus.inrush_time <= 0) {	// from now on, we monitor the current for overcurrent conditions
//			if (boosterstatus.actual_current > boosterconfig.max_current) boosterstatus.short_time += 2;
//			else if (boosterstatus.short_time > 0) boosterstatus.short_time--;
//		}
	} else {					// booster is off
		if (timing > 0) {
			timing--;
		}
		if (relais_timeout > 0) {
			if (--relais_timeout == 0) PRGRELAIS_OFF();
		}
		return;
	}

	// we only get here, if booster is really switched on and output voltage shall follow the setting
	dac = DAC1->DOR1;
	if (dac != boosterstatus.target_dac) {
		if (dac > boosterstatus.target_dac) {
			if (dac < DACSTEPS_PER_MS) dac = 0;
			else dac -= DACSTEPS_PER_MS;
			if (dac < boosterstatus.target_dac) dac = boosterstatus.target_dac;
		} else {
			dac += DACSTEPS_PER_MS;
			if (dac > boosterstatus.target_dac) dac = boosterstatus.target_dac;
		}
		DAC1->DHR12R1 = dac;
	}
}
