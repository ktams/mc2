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
 * calendar.c
 *
 *  Created on: 01.09.2020
 *      Author: Andi
 */

#include <stdio.h>
#include "rb2.h"

static const uint8_t days[] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static const char *weekdays[] = {
	"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"
};

char *timestamp (TickType_t t)
{
	char *tmp;

	tmp = tmp64();
	sprintf (tmp, "%ld:%02ld:%02ld.%03ld", t / (60*60*1000), (t / (60*1000)) % 60, (t / 1000) % 60, t % 1000);
	return tmp;
}

bool isLeapYear(int year)
{
	return (((year % 4) == 0 && (year % 100) != 0) || (year % 400) == 0);
}

int daysInMonth(int year, int mon)
{
	if (mon < 1 || mon > 12) return 30;
	if (mon == 2 && isLeapYear(year)) return 29;
	return days[mon - 1];
}

int calc_weekday (int year, int mon, int mday)
{
	int c, y, d;

	mon -= 2;		// for this calculation march is the first month of the year
	if (mon < 1) {	// january and february are counted as the last months of the previous year
		mon += 12;
		year--;
	}
	c = year / 100;		// 'c' is the century
	y = year % 100;		// 'y' ist the two digit year in this century
	d = mday + ((13 * mon - 1) / 5) + y + (y / 4) + (c / 4) - (2 * c);
	d %= 7;
	if (d < 0) d += 7;	// the resulting day for 0 = sunday .. 6 = saturday
	d--;				// we need 0 = monday, so subtract one
	if (d < 0) d = 6;	// and correct the sunday to '6'

	return d;
}

const char *weekday (int wday)
{
	return weekdays[wday % 7];
}
