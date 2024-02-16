/*
 * logging.h
 *
 *  Created on: 27.02.2021
 *      Author: Andi
 */

/*
 * RB2, next generation model railroad controller software
 * Copyright (C) 2021 Tams Elektronik GmbH and Andreas Kretzer
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

#ifndef __LOGGING_H__
#define __LOGGING_H__

#define ANSI_COLOR		"\033["
#define ANSI_RESET		"\033[0m"

enum ansiColor {
	NONE = 0,
	BLACK = 30,
	RED = 31,
	GREEN = 32,
	YELLOW = 33,
	BLUE = 34,
	MAGENTA = 35,
	CYAN = 36,
	WHITE = 37
};

enum outline {
	NORMAL = 0,
	BOLD = 1,
	DIMMED = 2
};

#define LOG_INFO		0x00000001
#define LOG_WARNING		0x00000002
#define LOG_ERROR		0x00000004
#define LOG_HTTPD		0x00000008
#define LOG_BIDIB		0x00000010
#define LOG_EASYNET		0x00000020
#define LOG_RAILCOM		0x00000040
#define LOG_M3READ		0x00000080
#define LOG_DCCA		0x00000100
#define LOG_DEBUG		0x00000200

/*
 * Prototypes Utilities/logging.c
 */
char *log_ansiColor (enum ansiColor fgc, enum ansiColor bgc, enum outline ol);
void log_msg (uint32_t level, const char *fmt, ...) __attribute__((format (printf,2,3)));
void log_error (const char *fmt, ...) __attribute__((format (printf,1,2)));
uint32_t log_enable (uint32_t lvl);
uint32_t log_disable (uint32_t lvl);

#endif /* __LOGGING_H__ */
