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
#include "events.h"

static volatile uint8_t segdata[2];
static bool show_current;
static unsigned blink;

static const uint8_t digits[] = {
	0b0111111,		// 0
	0b0000110,		// 1
	0b1011011,		// 2
	0b1001111,		// 3
	0b1100110,		// 4
	0b1101101,		// 5
	0b1111101,		// 6
	0b0000111,		// 7
	0b1111111,		// 8
	0b1101111,		// 9
	0b1110111,		// A
	0b1111100,		// b
	0b0111001,		// C
	0b1011110,		// d
	0b1111001,		// E
	0b1110001,		// F
};

/**
 * This function controls the 7-segment display. It is
 * called by vApplicationTickHook() in main.c which in
 * turn is called from the FreeRTOS ticktimer interrupt.
 * For this reason, the function must be treated as
 * interrupt handler.
 */
void seg_timer (void)
{
	TickType_t timer;

	timer = xTaskGetTickCountFromISR();

#if 0
	if (timer % 1000 >= 500) {
		segdata[0] &= ~0x40;
		segdata[1] |= 0x40;
	} else {
		segdata[0] |= 0x40;
		segdata[1] &= ~0x40;
	}
#endif

	if ((blink > 0) && (timer % blink) < (blink / 2)) {
		SEG_OFF();
	} else {
		switch (timer % 8) {
			case 0:
				SEG_OFF();
				break;
			case 1:
			case 2:
			case 3:
				GPIOC->ODR = (GPIOC->ODR & ~0xFF00) | (segdata[0] << 8);
				SEG_A1();
				break;
			case 4:
				SEG_OFF();
				break;
			case 5:
			case 6:
			case 7:
				SEG_A2();
				GPIOC->ODR = (GPIOC->ODR & ~0xFF00) | (segdata[1] << 8);
				break;
		}
	}
}

void seg_decimal (int n, bool dp)
{
	if (n >= 100) n = 99;
	if (n < 0) n = 0;

	if (n / 10 == 0 && !dp) {
		segdata[0] = 0;		// display as blank
	} else {
		segdata[0] = digits[n / 10];
		if (dp) segdata[0] |= 0x80;
	}
	segdata[1] = digits[n % 10];
}

void seg_display (uint8_t left, uint8_t right)
{
	segdata[0] = left;
	segdata[1] = right;
}

uint8_t seg_getHexDigit (int v)
{
	if (v < 0 || v > 15) return 0;
	return digits[v];
}

/**
 * Track Stop mode: Show "St." on the display
 */
void seg_stop (void)
{
	segdata[0] = 0b01101101;		// the 'S'
	segdata[1] = 0b11111000;		// the 't' + '.'
}

/**
 * Pause: Show PAUSE symbol on the display
 */
void seg_pause (void)
{
	segdata[0] = 0b00000110;		// the ' |'
	segdata[1] = 0b00110000;		// the '| '
}

/**
 * Track / Booster Short: Show "SH" on the display
 */
void seg_short (void)
{
	segdata[0] = 0b01101101;		// the 'S'
	segdata[1] = 0b01110110;		// the 'H'
}

/**
 * Track Power on: Show "Go" on the display
 */
void seg_go (void)
{
	segdata[0] = 0b00111101;		// the 'G'
	segdata[1] = 0b01011100;		// the 'o'
}

/**
 * Reset: Show "rE" on the display
 */
void seg_reboot (void)
{
	segdata[0] = 0b01010000;		// the 'r'
	segdata[1] = 0b01111001;		// the 'E'
}

/**
 * Programming mode: Show "Pr" on the display
 */
void seg_progmode (void)
{
	segdata[0] = 0b01110011;		// the 'P'
	segdata[1] = 0b01010000;		// the 'r'
}

/**
 * Test Drive: Show "td" on the display
 */
void seg_testdrive (void)
{
	segdata[0] = 0b01111000;		// the 't'
	segdata[1] = 0b01011110;		// the 'd'
}

/**
 * Factory Reset: Show "Fr" on the display
 */
void seg_factoryReset (void)
{
	segdata[0] = 0b01110001;		// the 'F'
	segdata[1] = 0b01010000;		// the 'r'
}

/**
 * Powerfail: Show "PF" on the display
 */
void seg_powerfail (void)
{
	segdata[0] = 0b01110011;		// the 'P'
	segdata[1] = 0b01110001;		// the 'F'
}

/**
 * over temperature: Show "ot" on the display
 */
void seg_overtemp (void)
{
	segdata[0] = 0b01011100;		// the 'o'
	segdata[1] = 0b01111000;		// the 't'
}

/**
 * (BiDiB-)Pairing: Show "PA" on the display
 */
void seg_pairing (bool on)
{
	if (on) {
		show_current = false;
		blink = 600;
		segdata[0] = 0b01110011;		// the 'P'
		segdata[1] = 0b01110111;		// the 'A'
	} else {
		show_current = (rt.tm == TM_GO);
		blink = 0;
	}
}

static bool seg_current (eventT *e, void *arg)
{
	(void) arg;

	if (e) switch (e->ev) {
		case EVENT_CURRENT:
			if (show_current) {
				seg_decimal(e->param, true);
			}
			break;
		case EVENT_SYS_STATUS:
			show_current = false;			// for most states don't disturb display content
			switch (e->param) {
				case SYSEVENT_STOP:
					seg_stop();
					break;
				case SYSEVENT_HALT:
					seg_pause();
					break;
				case SYSEVENT_GO:
					show_current = true;	// the only state where we show track current on display
					break;
				case SYSEVENT_SHORT:
					seg_short();
					break;
				case SYSEVENT_TESTDRIVE:
					seg_testdrive();
					break;
				case SYSEVENT_RESET:
					seg_reboot();
					break;
				case SYSEVENT_OVERTEMP:
					seg_overtemp();
					break;
			}
			break;
		default:
			break;
	}
	return true;
}

void seg_registerEvents (void)
{
	event_register(EVENT_CURRENT, seg_current, NULL, 0);
	event_register(EVENT_SYS_STATUS, seg_current, NULL, 0);
}
