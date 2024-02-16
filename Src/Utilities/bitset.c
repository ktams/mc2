/**
 * @file bitset.c
 *
 * @author Andi
 * @date   01.06.2020
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

void bs_set (uint32_t *bitset, int bit)
{
	int idx;

	idx = bit / 32;
	bitset[idx] |= 1 << (bit & 31);
}

void bs_clear (uint32_t *bitset, int bit)
{
	int idx;

	idx = bit / 32;
	bitset[idx] &= ~(1 << (bit & 31));
}

bool bs_isset (uint32_t *bitset, int bit)
{
	int idx;

	idx = bit / 32;
	return !!(bitset[idx] & (1 << (bit & 31)));
}

bool bs_isempty (uint32_t *bitset, int bits)
{
	int i, count;

	count = (bits + 31) / 32;
	for (i = 0; i < count; i++) {
		if (bitset[i] != 0) return false;
	}
	return true;
}

/**
 * Count bits in a byte. This is a medium fast routine with nibble lookups.
 * There are sure more optimisable versions out there, especialy versions
 * that may use special hardware instruction on platforms that support it.
 * For GCC we could have used __builtin__popcount().
 *
 * \param b		the byte to count the 1-bits from
 * \return		the number of 1-bits in this byte
 */
int bc_byte (uint8_t b)
{
	static const uint8_t bits[] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4 };
	return bits[b & 0x0F] + bits[b >> 4];
}

/**
 * Count bits in a 16-bit short. Uses the byte-variant to add the counts
 * from the two bytes of the parameter.
 *
 * \param s		the short (unsigned 16-bit integer) to count the 1-bits from
 * \return		the number of 1-bits in this short
 */
int bc_short (uint16_t s)
{
	return bc_byte(s & 0xFF) + bc_byte(s >> 8);
}

/**
 * Count bits in a 32-bit long. Uses the short-variant to add the counts
 * from the two shorts of the parameter.
 *
 * \param l		the long (unsigned 32-bit integer) to count the 1-bits from
 * \return		the number of 1-bits in this long
 */
int bc_long (uint32_t l)
{
	return bc_short(l & 0xFFFF) + bc_short(l >> 16);
}
