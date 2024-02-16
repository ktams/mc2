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

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "rb2.h"
#include "yaffsfs.h"
#include "lwip/sockets.h"
#include "events.h"

#define MAX_STRINGLEN		256		///< the maximum resulting length allowed for string to write to the file

/*******************************************************************************************************
 * Implementation of stdio helper functions to access YAFFS and debug output via network
 *******************************************************************************************************/
#define PRINT_RESET			"\033[m"
#define PRINT_BRIGHTRED		"\033[1;91m"

int _open (const char *path, int oflag, int mode)
{
	return yaffs_open(path, oflag, mode);
}

int _close (int fd)
{
	if ((fd >= YAFFSFS_OFFSET_HANDLES) && (fd < (YAFFSFS_OFFSET_HANDLES + YAFFSFS_N_HANDLES))) {
		return yaffs_close(fd);
	}

	if (fd >= LWIP_SOCKET_OFFSET && (fd < (LWIP_SOCKET_OFFSET + MEMP_NUM_NETCONN))) {
		return lwip_close(fd);
	}

	return -1;
}

int _lseek (int fd, int ptr, int dir)
{
	if ((fd >= YAFFSFS_OFFSET_HANDLES) && (fd < (YAFFSFS_OFFSET_HANDLES + YAFFSFS_N_HANDLES))) {
		return yaffs_lseek(fd, ptr, dir);
	}

	return -1;
}

int _read (int fd, char *data, int nbyte)
{
	if ((fd >= YAFFSFS_OFFSET_HANDLES) && (fd < (YAFFSFS_OFFSET_HANDLES + YAFFSFS_N_HANDLES))) {
		return yaffs_read(fd, data, nbyte);
	}

	if (fd >= LWIP_SOCKET_OFFSET && (fd < (LWIP_SOCKET_OFFSET + MEMP_NUM_NETCONN))) {
		return lwip_read(fd, data, nbyte);
	}

	return -1;
}

int _write (int fd, char *data, int nbyte)
{
	static bool log_truncated;

	char *tmp, *s;

	if ((fd == STDOUT_FILENO) || (fd == STDERR_FILENO)) {
		if (fd == STDERR_FILENO) dbg_write (PRINT_BRIGHTRED, sizeof(PRINT_BRIGHTRED) - 1);
		dbg_write(data, nbyte);
		if (fd == STDERR_FILENO) dbg_write (PRINT_RESET, sizeof(PRINT_RESET) - 1);
		if (log_truncated) {
			if (event_fireEx(EVENT_LOGMSG, 1, "LOG TRUNCATED", 0, 0) == 0) log_truncated = false;
		}
		if ((tmp = malloc(nbyte + 1)) != NULL) {
			memcpy (tmp, data, nbyte);
			s = tmp + nbyte;
			*s = 0;		// terminate string with null byte
			while (s > tmp && (s[-1] == '\n' || s[-1] == '\r')) s--;
			*s = 0;
			if (event_fireEx(EVENT_LOGMSG, (fd == STDERR_FILENO) ? 1 : 0, tmp, EVTFLAG_FREE_SRC, 0)) log_truncated = true;
		}
		return nbyte;
	}

	if ((fd >= YAFFSFS_OFFSET_HANDLES) && (fd < (YAFFSFS_OFFSET_HANDLES + YAFFSFS_N_HANDLES))) {
		return yaffs_write(fd, data, nbyte);
	}

	if (fd >= LWIP_SOCKET_OFFSET && (fd < (LWIP_SOCKET_OFFSET + MEMP_NUM_NETCONN))) {
		return lwip_write(fd, data, nbyte);
	}

	return -1;
}

int _stat (char *file, struct stat *st)
{
	struct yaffs_stat yst;
	int rc;

	if (!st) return -1;

	memset (st, 0, sizeof(*st));

	if ((rc = yaffs_stat(file, &yst)) != 0) return rc;
	// the fields inside the stat structure have different types and position, so we must copy
	st->st_dev = yst.st_dev;
	st->st_ino = yst.st_ino;
	st->st_mode = yst.st_mode;
	st->st_nlink = yst.st_nlink;
	st->st_uid = yst.st_uid;
	st->st_gid = yst.st_gid;
	st->st_rdev = yst.st_rdev;
	st->st_size = yst.st_size;
	st->st_blksize = yst.st_blksize;
	st->st_blocks = yst.st_blocks;
	st->st_atime = yst.yst_atime;
	st->st_mtime = yst.yst_mtime;
	st->st_ctime = yst.yst_ctime;
	return 0;
}

int _fstat (int fd, struct stat *st)
{
	struct yaffs_stat yst;
	int rc;

	if (!st) return -1;

	if ((fd >= YAFFSFS_OFFSET_HANDLES) && (fd < (YAFFSFS_OFFSET_HANDLES + YAFFSFS_N_HANDLES))) {
		memset (st, 0, sizeof(*st));

		if ((rc = yaffs_fstat(fd, &yst)) != 0) return rc;
		// the fields inside the stat structure have different types and position, so we must copy
		st->st_dev = yst.st_dev;
		st->st_ino = yst.st_ino;
		st->st_mode = yst.st_mode;
		st->st_nlink = yst.st_nlink;
		st->st_uid = yst.st_uid;
		st->st_gid = yst.st_gid;
		st->st_rdev = yst.st_rdev;
		st->st_size = yst.st_size;
		st->st_blksize = yst.st_blksize;
		st->st_blocks = yst.st_blocks;
		st->st_atime = yst.yst_atime;
		st->st_mtime = yst.yst_mtime;
		st->st_ctime = yst.yst_ctime;
		return 0;
	}

	return -1;
}

int _isatty (int fd)
{
	if ((fd == STDOUT_FILENO) || (fd == STDERR_FILENO)) return 1;

	return 0;
}

pid_t _getpid (void)
{
	return (int) xTaskGetCurrentTaskHandle();
}

int _kill (pid_t pid, int sig)
{
	(void) pid;
	(void) sig;

	return -1;
}
