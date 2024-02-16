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
#include <string.h>
#include <assert.h>
#include "rb2.h"

void *__wrap_malloc (size_t size)
{
    return pvPortMalloc(size);
}

void *__wrap_calloc (size_t units, size_t size)
{
    void *buf;

    if ((buf = pvPortMalloc(units * size)) == NULL) return NULL;
    memset (buf, 0, units * size);
    return buf;
}

void *__wrap_realloc (void *mem, size_t newsize)
{
    void *buf;

    if (mem && newsize == 0) {
    	vPortFree(mem);
    	buf = NULL;
    } else {
		if ((buf = pvPortMalloc(newsize)) == NULL) return NULL;
		if (mem) {
			memmove(buf, mem, newsize);
			vPortFree(mem);
		}
    }
    return buf;
}

void __wrap_free (void *mem)
{
    vPortFree(mem);
}

void *__wrap__malloc_r (struct _reent *reent, size_t size)
{
	void *b;

	b = __wrap_malloc(size);
	if (b == NULL) reent->_errno = ENOMEM;
	return b;
}

void *__wrap__calloc_r (struct _reent *reent, size_t units, size_t size)
{
	void *b;

	b = __wrap_calloc (units, size);
	if (b == NULL) reent->_errno = ENOMEM;
	return b;
}

void *__wrap__realloc_r (struct _reent *reent, void *mem, size_t newsize)
{
	void *b;

	b = __wrap_realloc(mem, newsize);
	if (b == NULL) reent->_errno = ENOMEM;
	return b;
}

void __wrap__free_r (struct _reent *reent, void *mem)
{
	(void) reent;

	__wrap_free (mem);
}

void *dbgmalloc (size_t size, const char *file, const char *func, int line)
{
	void *b;

	if ((b = pvPortMalloc(size)) == NULL) {
		fprintf (stderr, "malloc(%d): %s(): out of memory in %s:%d\n", size, func, file, line);
	}
	return b;
}

void *dbgcalloc (size_t units, size_t size, const char *file, const char *func, int line)
{
	void *b;

	if ((b = pvPortMalloc(units * size)) == NULL) {
		fprintf (stderr, "calloc(%d, %d): %s(): out of memory in %s:%d\n", units, size, func, file, line);
	} else {
	    memset (b, 0, units * size);
	}
	return b;
}

void *dbgrealloc (void *mem, size_t newsize, const char *file, const char *func, int line)
{
	void *b;

	if ((b = __wrap_realloc(mem, newsize)) == NULL) {
		fprintf (stderr, "realloc(%p, %d): %s(): out of memory in %s:%d\n", mem, newsize, func, file, line);
	}
	return b;
}
