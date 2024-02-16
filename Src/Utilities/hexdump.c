/*
 * hexdump.c
 *
 *  Created on: 30.04.2022
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

#include <stdio.h>
#include "rb2.h"

static void hexdump_line (uint8_t *addr, uint8_t *end, uint32_t offs)
{
	char linebuf[128];
	uint8_t *p;
	char *s;
	int i;

	p = addr;
	s = linebuf;

	s += sprintf (linebuf, "\t0x%04lx  ", offs);
	for (i = 0; i < 16; i++, addr++) {
		if (addr < end) s += sprintf (s, " %02x", *addr);
		else s += sprintf (s, "   ");
		if (i == 7) s += sprintf (s, "  ");
	}

	s += sprintf (s, "   ");
	for (i = 0; i < 16; i++, p++) {
		if (p < end) {
			if (isprint((int) *p)) s += sprintf (s, "%c", *p);
			else *s++ = '.';
		} else *s++ = ' ';
		if (i == 7) *s++ = ' ';
	}
	*s = 0;
	log_msg (LOG_INFO, "%s\n", linebuf);
}

void hexdump (void *addr, int len)
{
	uint8_t *end;
	uint32_t offs;

	log_msg (LOG_INFO, "%s() %d bytes @ %p\n", __func__, len, addr);
	offs = 0;
	end = addr +len;
	while ((uint8_t *) addr < end) {
		hexdump_line((uint8_t *) addr, end, offs);
		addr += 16;
		offs += 16;
	}
}
