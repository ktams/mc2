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

void cache_flushall (void)
{
#ifdef USE_CACHE
	SCB_CleanInvalidateDCache();
#endif
}

void cache_flush (uint32_t adr, int size)
{
#ifdef USE_CACHE
	uint32_t a;

	a = adr & ~0x1F;
	size += adr - a;
	SCB_CleanDCache_by_Addr((uint32_t *) a, size);
#endif
}

void cache_invalidate (uint32_t adr, int size)
{
#ifdef USE_CACHE
	uint32_t a;

	a = adr & ~0x1F;
	size += adr - a;
	SCB_InvalidateDCache_by_Addr((uint32_t *) a, size);
#endif
}
