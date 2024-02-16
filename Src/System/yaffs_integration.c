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
#include <errno.h>
#include "rb2.h"
#include "nandflash.h"
#include "yaffs_trace.h"

/*
 * Implementation of functions needed by YAFFS
 */

unsigned yaffs_trace_mask =
		/*
	YAFFS_TRACE_SCAN |
	YAFFS_TRACE_GC |
	YAFFS_TRACE_ERASE |
	YAFFS_TRACE_ERROR |
	YAFFS_TRACE_TRACING |
	YAFFS_TRACE_ALLOCATE |
	YAFFS_TRACE_BAD_BLOCKS |
	YAFFS_TRACE_VERIFY |
	*/
	0;

static SemaphoreHandle_t yaffs_semaphore;

void yaffs_bug_fn (const char *file_name, int line_no)
{
	fprintf (stderr, "YAFFS-BUG: %s line %d\n", file_name, line_no);
}

void yaffsfs_Lock(void)
{
	xSemaphoreTake(yaffs_semaphore, portMAX_DELAY);
}

void yaffsfs_Unlock(void)
{
	xSemaphoreGive(yaffs_semaphore);
}

u32 yaffsfs_CurrentTime(void)
{
	return 0;
}

void yaffsfs_SetError(int err)
{
	errno = err;
}

void *yaffsfs_malloc(size_t size)
{
	return malloc (size);
}

void yaffsfs_free(void *ptr)
{
	free (ptr);
}

int yaffsfs_CheckMemRegion(const void *addr, size_t size, int write_request)
{
	size_t flashsize;

	if (addr == NULL) return -1;
	if (addr >= (void *) D1_DTCMRAM_BASE && ((addr + size) <= (void *) (D1_DTCMRAM_BASE + D1_DTCMRAM_SIZE))) return 0;
	if (addr >= (void *) D1_AXISRAM_BASE && ((addr + size) <= (void *) (D1_AXISRAM_BASE + D1_AXISRAM_SIZE))) return 0;

	// D2 RAM accessible over AXI and AHB!
	if (addr >= (void *) D2_AXISRAM_BASE && ((addr + size) <= (void *) (D2_AXISRAM_BASE + D2_AXISRAM_SIZE))) return 0;
	if (addr >= (void *) D2_AHBSRAM_BASE && ((addr + size) <= (void *) (D2_AHBSRAM_BASE + D2_AXISRAM_SIZE))) return 0;

	if (addr >= (void *) D3_SRAM_BASE && ((addr + size) <= (void *) (D3_SRAM_BASE + D3_SRAM_SIZE))) return 0;

	if (addr >= (void *) SDRAM_BASE && ((addr + size) <= (void *) (SDRAM_BASE + SDRAM_SIZE))) return 0;

	if (!write_request) {
		flashsize = (*(uint32_t *) FLASHSIZE_BASE) << 10;
		if (addr >= (void *) D1_AXIFLASH_BASE && ((addr + size) <= (void *) (D1_AXIFLASH_BASE + flashsize))) return 0;
	}
	fprintf (stderr, "%s() illegal memory access %p (size %u)\n", __func__, addr, size);
	return -1;
}

void yaffsfs_OSInitialisation(void)
{
	if (!yaffs_semaphore) {
		yaffs_semaphore = xSemaphoreCreateMutex();
	}
}
