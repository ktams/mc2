/**
 * @file myalloc.h
 *
 * @author Andi
 * @date   12.04.2020
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

#ifndef __MYALLOC_H__
#define __MYALLOC_H__

#include <stdlib.h>		// make sure, the original malloc() and calloc() can be overridden

/*
 * Prototypes System/myalloc.c
 */
void *dbgmalloc (size_t size, const char *file, const char *func, int line);
void *dbgcalloc (size_t units, size_t size, const char *file, const char *func, int line);
void *dbgrealloc (void *mem, size_t newsize, const char *file, const char *func, int line);

#define malloc(X)		dbgmalloc( X, __FILE__, __func__, __LINE__)
#define calloc(X,Y)		dbgcalloc( X, Y, __FILE__, __func__, __LINE__)
#define realloc(X,Y)	dbgrealloc( X, Y, __FILE__, __func__, __LINE__)

#endif /* __MYALLOC_H__ */
