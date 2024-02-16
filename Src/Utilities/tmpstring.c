/**
 * @file tmpstring.c
 *
 * @author Andi
 * @date   26.04.2020
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

#define POOLBUFFER

#ifdef POOLBUFFER

#define MAXBUFFER		(2 * 1024)
#define ALIGNMENT		8

static char pool[D1_DTCMRAM_SIZE] __attribute__((section(".dtcmram")));		// uses the whole DTCM-RAM!
static volatile int poolidx;

/**
 * Supply a memory array of given size for temporary usage.
 *
 * There is a memory pool where the requested buffer space is "allocated".
 * An index pointer jumps to the next free offset in the pool to be prepared
 * for the next request. If the requested buffer won't fit into the rest of
 * the pool memory, the index pointer restarts at the beginning. This makes
 * up a round robin fashioned allocation.
 *
 * The allocation is proteced against concurrent accesses by a critical section.
 * This is a very leightweight alternative to a mutex, which in the end could
 * fail to be acquired. The action inside the critical section is very short,
 * so should not do any harm even to critical interrupt latencies.
 *
 * The buffers are aligned to ALIGNMENT bytes (should be 8, see above). This
 * makes it possible to use it as a buffer for arbitrary data types.
 *
 * The lifetime of such buffers should be very short, so not two threads at the
 * same time use these strings. They would overwrite each other's content.
 *
 * ATTENTION: These buffers come from DTCMRAM, which is not reachable by the
 * ETHERNET peripheral. Whenever you try to build up a network buffer for
 * transmission, these buffers are a no-go! Receiving to such a buffer would
 * succeed, because the reception involves a copy operation, but as you never
 * know when your packet arrives, the livetime restiction may be critical.
 *
 * \param siz	the size of the requested buffer
 * \return		a pointer into the pool area
 * 				or NULL if the size was above maximum allowed (\see MAXBUFFER)
 */
void *tmpbuf (size_t siz)
{
	void *s;

	if (siz <= 0 || siz > MAXBUFFER) return NULL;

	siz = ((siz + ALIGNMENT - 1) / ALIGNMENT) * ALIGNMENT;
	taskENTER_CRITICAL();
	if ((poolidx + siz) > DIM(pool)) poolidx = 0;
	s = &pool[poolidx];
	poolidx += siz;
	taskEXIT_CRITICAL();
	return s;
}

/**
 * Supply a 64 byte character array for temporary usage.
 *
 * \return		a pointer to a buffer that reserves 64 bytes from the pool
 * 				or NULL if the mutex could not be aquired in time
 * \see			tmpstr()
 */
char *tmp64 (void)
{
	return (char *) tmpbuf (64);
}

/**
 * Supply a 256 byte character array for temporary usage.
 *
 * \return		a pointer to a buffer that reserves 256 bytes from the pool
 * 				or NULL if the mutex could not be aquired in time
 * \see			tmpstr()
 */
char *tmp256 (void)
{
	return (char *) tmpbuf (256);
}

/**
 * Supply a 1024 byte character array for temporary usage.
 *
 * \return		a pointer to a buffer that reserves 1024 bytes from the pool
 * 				or NULL if the mutex could not be aquired in time
 * \see			tmpstr()
 */
char *tmp1k (void)
{
	return (char *) tmpbuf (1024);
}

#else
#define MAX_STR64		32
#define MAX_STR256		32
#define MAX_STR1K		16

static SemaphoreHandle_t mutex;			///< a mutex to control access to the string indexes

static char string64[MAX_STR64][64] __attribute__((section(".dtcmram")));
static char string256[MAX_STR256][256] __attribute__((section(".dtcmram")));
static char string1k[MAX_STR1K][1024] __attribute__((section(".dtcmram")));
static int idx64;
static int idx256;
static int idx1k;

/**
 * Supply a 64 byte character array for temporary usage.
 *
 * There is an array of these strings from which one is taken
 * in a round robin way for every call of this function.
 *
 * The lifetime of such strings should be very short, so not
 * two threads at the same time use these strings. They would
 * overwrite each other's content.
 *
 * \return		a pointer to one of several strings of 64 bytes length each
 * 				or NULL if the mutex could not be aquired in time
 */
char *tmp64 (void)
{
	char *s;

	if (!mutex_lock(&mutex, 20, __func__)) return NULL;
	idx64++;
	if (idx64 < 0 || idx64 >= MAX_STR64) idx64 = 0;
	s = string64[idx64];
	mutex_unlock(&mutex);
	return s;
}

/**
 * Supply a 256 byte character array for temporary usage.
 *
 * \return		a pointer to one of several strings of 256 bytes length each
 * 				or NULL if the mutex could not be aquired in time
 * \see			tmp64()
 */
char *tmp256 (void)
{
	char *s;

	if (!mutex_lock(&mutex, 20, __func__)) return NULL;
	idx256++;
	if (idx256 < 0 || idx256 >= MAX_STR256) idx256 = 0;
	s = string256[idx256];
	mutex_unlock(&mutex);
	return s;
}

/**
 * Supply a 1024 (1k) byte character array for temporary usage.
 *
 * \return		a pointer to one of several strings of 256 bytes length each
 * 				or NULL if the mutex could not be aquired in time
 * \see			tmp64()
 */
char *tmp1k (void)
{
	char *s;

	if (!mutex_lock(&mutex, 20, __func__)) return NULL;
	idx1k++;
	if (idx1k < 0 || idx1k >= MAX_STR1K) idx1k = 0;
	s = string1k[idx1k];
	mutex_unlock(&mutex);
	return s;
}

#endif
