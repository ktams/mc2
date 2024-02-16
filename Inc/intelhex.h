/*
 * intelhex.h
 *
 *  Created on: 13.03.2021
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

#ifndef __INTELHEX_H__
#define __INTELHEX_H__

#define IHEX_MAXDATA		64		///< the maximum amount of data bytes encoded in a iHex line

enum ihexstate {
	IHEX_READING = 0,						///< actively reading iHex content
	IHEX_END,								///< end marker encountered
	IHEX_ERROR,								///< an error was detected
};

struct ihexdata {
	uint32_t		segadr;					///< actual segment address
	uint16_t		reladr;					///< the relative address from the iHex line
	enum ihexstate	state;					///< the status when reading data
	uint8_t			data[IHEX_MAXDATA];		///< the decoded binary data, if any
};

/*
 * Prototypes Utilities/intelhex.c
 */
uint8_t hex_byte (char *s);
uint16_t hex_short (char *s);
uint32_t hex_word (char *s);
int ihex_readline (struct ihexdata *d, char *s);

#endif /* __INTELHEX_H__ */
