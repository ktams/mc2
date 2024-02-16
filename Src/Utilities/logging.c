/*
 * logging.c
 *
 *  Created on: 26.02.2021
 *      Author: andi
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
#include <stdarg.h>
#include <string.h>
#include "rb2.h"
#include "logging.h"

static volatile uint32_t logflag = LOG_INFO | LOG_WARNING | LOG_ERROR;

//static char *log_timestamp (void)
//{
//	TickType_t t;
//	char *tmp;
//
//	tmp = tmp64();
//	t = xTaskGetTickCount();
//	sprintf (tmp, "%ld:%02ld:%02ld.%03ld", t / (60*60*1000), (t / (60*1000)) % 60, (t / 1000) % 60, t % 1000);
//	return tmp;
//}

char *log_ansiColor (enum ansiColor fgc, enum ansiColor bgc, enum outline ol)
{
	char *tmp;
	int components;

	components = 0;
	if (fgc != NONE) components |= 1;
	if (bgc != NONE) components |= 2;
	if (ol != NORMAL) components |= 4;

	tmp = tmp64();
	switch (components) {
		case 0:		// nothing is set - this may be used as COLOR-RESET
			sprintf (tmp, "%s", ANSI_RESET);
			break;
		case 1:		// FGC
			sprintf (tmp, "%s%dm", ANSI_COLOR, fgc);
			break;
		case 2:		// BGC
			sprintf (tmp, "%s%dm", ANSI_COLOR, bgc + 10);
			break;
		case 3:		// FGC + BGC
			sprintf (tmp, "%s%d;%dm", ANSI_COLOR, fgc, bgc + 10);
			break;
		case 4:		// OUTLINE
			sprintf (tmp, "%s%dm", ANSI_COLOR, ol);
			break;
		case 5:		// OUTLINE + FGC
			sprintf (tmp, "%s%d;%dm", ANSI_COLOR, ol, fgc);
			break;
		case 6:		// OUTLINE + BGC
			sprintf (tmp, "%s%d;%dm", ANSI_COLOR, ol, bgc + 10);
			break;
		case 7:		// OUTLINE + FGC + BGC
			sprintf (tmp, "%s%d;%d;%dm", ANSI_COLOR, ol, fgc, bgc + 10);
			break;
	}
	return tmp;
}

static char *log_internal (char *pre, const char *fmt, va_list ap)
{
	char *buf, *s;
	bool nl = false;

	s = buf = tmp1k();
	if (pre) s += sprintf (s, "%s", pre);
	s += sprintf (s, "%s ", timestamp(xTaskGetTickCount()));

	s += vsprintf (s, fmt, ap);
	if (pre) {
		if (s > buf && s[-1] == '\n') {		// check if output ends in newline character
			s--;
			nl = true;
		}
		s+= sprintf (s, "%s", ANSI_RESET);
		if (nl) {
			*s++ = '\n';
			*s = 0;
		}
	}

	return buf;
}

void log_msg (uint32_t level, const char *fmt, ...)
{
	va_list ap;
	char *buf, *enhance;

	if (!fmt) return;
	if (!(level & logflag)) return;

	enhance = NULL;
	if (level == LOG_WARNING) enhance = log_ansiColor(BLUE, NONE, NORMAL);
	else if (level == LOG_DEBUG) enhance = log_ansiColor(GREEN, NONE, NORMAL);
	va_start (ap, fmt);
	buf = log_internal(enhance, fmt, ap);
	va_end (ap);

	/*if (level == LOG_BIDIB) dbg_write(buf, strlen(buf));		// this gets too much for logging in a browser
	else write (STDOUT_FILENO, buf, strlen(buf));*/

	write (STDOUT_FILENO, buf, strlen(buf));
}

void log_error (const char *fmt, ...)
{
	va_list ap;
	char *buf;

	if (!fmt) return;

	va_start (ap, fmt);
	buf = log_internal(log_ansiColor(RED, NONE, BOLD), fmt, ap);
	va_end (ap);

	write (STDOUT_FILENO, buf, strlen(buf));
}

uint32_t log_enable (uint32_t lvl)
{
	uint32_t old;

	old = logflag;
	logflag |= lvl;
	return old;
}

uint32_t log_disable (uint32_t lvl)
{
	uint32_t old;

	old = logflag;
	logflag &= ~lvl;
	return old;
}
