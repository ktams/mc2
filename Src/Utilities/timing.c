/**
 * @file timing.c
 *
 * @author Andi
 * @date   24.05.2020
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

/**
 * Caclulate a timeout instance (a tickcount value) for when the the given
 * milliseconds have past since this call.
 *
 * If the specified timeout is just zero or negative, a zero is returned. When
 * checking the timeout using tim_isover() such a timeout will be considered
 * "not set" and yields "timeout did not happen".
 *
 * For any other value, it is assured, that the resulting time (in system ticks)
 * a value different from zero. That may mean, that the timeout is elongated by
 * one tick (ms in this system).
 *
 * \param ms	the amount of milliseconds until the check should yield timeout
 * \return		the resulting tick time from which on it will be treated as timed out
 */
TickType_t tim_timeout (int ms)
{
	TickType_t to;

	if (ms <= 0) return 0;
	to = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
	if (to == 0) to = 1;		// avoid a timeout of 0 because that may mean an unset timeout
	return to;
}

/**
 * Check if a timeout has occured. A timeout can be set by use of the
 * function tim_timeout() and will be checked here.
 *
 * A timeout instance of zero (0) is counted as not set. Consequently the time
 * will never be over.
 *
 * If a timeout is set, the function checks for two possibilities. One is, that
 * the timeout is set to a value that matches the current time. The other is,
 * that the difference (i.e. subtraction) yields a negative value which is indicated
 * by having the topmost bit set.
 *
 * \param to	the timeout instance to check
 * \return		true, if time is over (or just hit), false otherwise
 */
bool tim_isover (TickType_t to)
{
	TickType_t t;

	if (!to) return false;							// if no timeout was set (to == 0), we are not timed out at all
	t = xTaskGetTickCount();
	return ((t == to) || ((to - t) & (1 << 31)));	// if we exactly reached the timeout val or diffence to current time is negative
}

/**
 * Check if a timeout has occured. A timeout can be set by use of the
 * function tim_timeout() and will be checked here.
 *
 * In contrast to tim_timeout() a timeout instance of zero (0) is taken as the first
 * time a time controlled function may be used and so it is taken as elapsed timeout
 * to give it a chance to do it's work and set the next (real) timeout.
 *
 * \param to	the timeout instance to check
 * \return		true, if time is over (or just hit), false otherwise
 * \see			tim_timeout()
 */
bool tim_isoverUnset (TickType_t to)
{
	if (to == 0) return true;
	return tim_isover(to);
}
