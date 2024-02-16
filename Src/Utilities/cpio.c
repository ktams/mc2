/*
 * cpio.c
 *
 *  Created on: 09.03.2021
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
#include <string.h>
#include <fcntl.h>
#include "rb2.h"
#include "yaffsfs.h"

#define READSIZE		4096
#define TRAILER			"TRAILER!!!"

struct header_old_cpio {
	uint16_t   c_magic;
	uint16_t   c_dev;
	uint16_t   c_ino;
	uint16_t   c_mode;
	uint16_t   c_uid;
	uint16_t   c_gid;
	uint16_t   c_nlink;
	uint16_t   c_rdev;
	uint16_t   c_mtime[2];
	uint16_t   c_namesize;
	uint16_t   c_filesize[2];
};

struct cpio_odc_header {
	char    c_magic[6];
	char    c_dev[6];
	char    c_ino[6];
	char    c_mode[6];
	char    c_uid[6];
	char    c_gid[6];
	char    c_nlink[6];
	char    c_rdev[6];
	char    c_mtime[11];
	char    c_namesize[6];
	char    c_filesize[11];
};

struct cpio_newc_header {
	char    c_magic[6];
	char    c_ino[8];
	char    c_mode[8];
	char    c_uid[8];
	char    c_gid[8];
	char    c_nlink[8];
	char    c_mtime[8];
	char    c_filesize[8];
	char    c_devmajor[8];
	char    c_devminor[8];
	char    c_rdevmajor[8];
	char    c_rdevminor[8];
	char    c_namesize[8];
	char    c_check[8];
};

struct fileheader {
	uint32_t		 m_time;		///< modification time in seconds since the eproc
	unsigned		 mode;			///< the file mode as for standard stat-value
	uint32_t		 fsize;			///< filesize in bytes (may need padding in the CPIO archive)
	char			*fname;			///< the (local) name of the file
};

enum format {
	CPIO_INVALID = 0,				///< unknown / nusupported CPIO format
	CPIO_BIN_LE,					///< CPIO binary format little endian
	CPIO_BIN_BE,					///< CPIO binary format big endian
	CPIO_ASCII_OLD,					///< CPIO ASCII old format (odc)
	CPIO_ASCII_NEW,					///< CPIO ASCII new format
	CPIO_ASCII_CRC,					///< CPIO ASCII new format with "CRC" (it really is only a summation of bytes, no CRC algorith!)
};

static uint32_t cpio_getOctalNumber (char *octal, int len)
{
	uint32_t val;

	val = 0;
	while (len) {
		val <<= 3;
		val |= *octal - '0';
		len--;
		octal++;
	}
	return val;
}

static uint32_t cpio_getHexNumber (char *hex, int len)
{
	uint32_t val;

	val = 0;
	while (len) {
		val <<= 4;
		if (*hex >= '0' && *hex <= '9') val |= *hex - '0';
		else if (*hex >= 'A' && *hex <= 'F') val |= (*hex - 'A') + 10;
		else if (*hex >= 'a' && *hex <= 'f') val |= (*hex - 'a') + 10;
		len--;
		hex++;
	}
	return val;
}

static int cpio_readHeader (int fd, struct fileheader *fhd, enum format fmt)
{
	union {
		struct header_old_cpio bin_header;
		struct cpio_odc_header odc_header;
		struct cpio_newc_header newc_header;
	} hd;
	int i, len, namelen;

	switch (fmt) {
		case CPIO_BIN_LE:
		case CPIO_BIN_BE:
			len = sizeof(struct header_old_cpio);
			break;
		case CPIO_ASCII_OLD:
			len = sizeof(struct cpio_odc_header);
			break;
		case CPIO_ASCII_NEW:
		case CPIO_ASCII_CRC:
			len = sizeof(struct cpio_newc_header);
			break;
		default:
			return -1;
	}

	read (fd, &hd, len);

	if (fmt == CPIO_BIN_BE) {
		uint16_t *p = &hd.bin_header.c_magic;
		for (i = 0; i < (int) (sizeof(struct header_old_cpio) / sizeof(uint16_t)); i++, p++) {
			*p = ntohs(*p);
		}
	}

	switch (fmt) {
		case CPIO_BIN_LE:
		case CPIO_BIN_BE:
			fhd->fsize = hd.bin_header.c_filesize[0] << 16 | hd.bin_header.c_filesize[1];
			fhd->mode = hd.bin_header.c_mode;
			namelen = hd.bin_header.c_namesize;
			namelen = (namelen + 1) & ~1;		// must be a multiple of two to always align the header on a 2-byte boundary
			break;
		case CPIO_ASCII_OLD:
			fhd->fsize = cpio_getOctalNumber(hd.odc_header.c_filesize, sizeof(hd.odc_header.c_filesize));
			fhd->mode = cpio_getOctalNumber(hd.odc_header.c_mode, sizeof(hd.odc_header.c_mode));
			namelen = cpio_getOctalNumber(hd.odc_header.c_namesize, sizeof(hd.odc_header.c_namesize));
			break;
		case CPIO_ASCII_NEW:
		case CPIO_ASCII_CRC:
			fhd->fsize = cpio_getHexNumber(hd.newc_header.c_filesize, sizeof(hd.newc_header.c_filesize));
			fhd->mode = cpio_getHexNumber(hd.newc_header.c_mode, sizeof(hd.newc_header.c_mode));
			namelen = cpio_getHexNumber(hd.newc_header.c_namesize, sizeof(hd.newc_header.c_namesize));
			namelen = ((namelen + 5) & ~3) - 2;		// must be a multiple of four to always align the header on a 4-byte boundary incl. the header itself
			break;
		default:
			return -1;
	}
	fhd->fname = tmp256();
	read (fd, fhd->fname, namelen);
	return 0;
}

static int cpio_extractFile (int fd, const char *fname, size_t len, size_t align)
{
	uint8_t *buf;
	int rc, file;

	ensure_path(fname);
	if ((file = yaffs_open(fname, O_CREAT | O_TRUNC | O_RDWR, S_IREAD | S_IWRITE)) < 0) {
		log_error ("%s(): cannot create file '%s'\n", __func__, fname);
		return -1;
	}
	if ((buf = malloc(READSIZE)) == NULL) {
		yaffs_close (file);
		return -1;
	}

	rc = 0;
	while (len > 0) {
		rc = read (fd, buf, (len > READSIZE) ? READSIZE : len);
		if (rc <= 0) break;
		write (file, buf, rc);
		len -= rc;
	}
	yaffs_close(file);
	if (align) rc = read (fd, buf, align);		// read padding bytes
	free (buf);

	return (rc < 0) ? -1 : 0;
}

static int cpio_extractDirectory (int fd, const char *fname, size_t len, size_t align)
{
	uint8_t buf[32];
	int rc;

	if (yaffs_access(fname, 0) != 0) {
		ensure_path(fname);
		if (yaffs_mkdir(fname, S_IREAD | S_IWRITE | S_IEXEC) < 0) {
			log_error ("%s(): cannot create directory '%s'\n", __func__, fname);
			return -1;
		}
	}
	rc = 0;
	len += align;								// adjust for padding (usually, no bytes are involved in directories)
	while (len > 0) {
		rc = read (fd, buf, (len > sizeof(buf)) ? sizeof(buf) : len);
		if (rc <= 0) break;
		len -= rc;
	}

	return (rc < 0) ? -1 : 0;
}

static int cpio_extractSymLink (int fd, const char *fname, size_t len, size_t align)
{
	char *buf;
	int rc;

	if (len >= 256) return -1;
	buf = tmp256();
	rc = read (fd, buf, len);
	buf[len] = 0;

	if (yaffs_access(fname, 0) != 0) {
		ensure_path(fname);
		if (yaffs_symlink(buf, fname) < 0) {
			log_error ("%s(): cannot symlink '%s' to '%s'\n", __func__, fname, buf);
			return -1;
		}
	}
	if (align) rc = read (fd, buf, align);		// read padding bytes

	return (rc < 0) ? -1 : 0;
}

static int cpio_readFile (int fd, const char *path, enum format fmt)
{
	struct fileheader hd;
	size_t align = 0;
	char *fname;

	cpio_readHeader(fd, &hd, fmt);

	switch (fmt) {
		case CPIO_BIN_LE:
		case CPIO_BIN_BE:
			align = (hd.fsize & 1);		// align to 2-byte
			break;
		case CPIO_ASCII_OLD:
			align = 0;					// no alignment neccessary
			break;
		case CPIO_ASCII_NEW:
		case CPIO_ASCII_CRC:
			align = (hd.fsize & 3);		// align to 4-byte
			if (align) align = 4 - align;
			break;
		default:
			return -1;
	}

	fname = tmp256();
	while (*hd.fname == '/') hd.fname++;
	canonical_path(fname, path, hd.fname);
	if (!strcmp(hd.fname, TRAILER)) return 1;

	switch (hd.mode & S_IFMT) {
		case S_IFDIR:
//			log_msg(LOG_INFO, "%s() DIRECTORY '%s'\n", __func__, fname);
			if (cpio_extractDirectory(fd, fname, hd.fsize, align) < 0) return -1;
			break;
		case S_IFLNK:
//			log_msg(LOG_INFO, "%s() SYMLINK '%s'\n", __func__, fname);
			if (cpio_extractSymLink(fd, fname, hd.fsize, align) < 0) return -1;
			break;
		case S_IFREG:
//			log_msg(LOG_INFO, "%s() FILE '%s' size = %lu\n", __func__, fname, hd.fsize);
			if (cpio_extractFile(fd, fname, hd.fsize, align) < 0) return -1;
			break;
	}

	return 0;
}

static enum format cpio_decodeMagic (const char *magic)
{
	if (magic[0] == 0xC7 && magic[1] == 0x71) return CPIO_BIN_LE;
	if (magic[0] == 0x71 && magic[1] == 0xC7) return CPIO_BIN_BE;
	if (!strncmp (magic, "070707", 6)) return CPIO_ASCII_OLD;
	if (!strncmp (magic, "070701", 6)) return CPIO_ASCII_NEW;
	if (!strncmp (magic, "070702", 6)) return CPIO_ASCII_CRC;
	return CPIO_INVALID;
}

int cpio_copyIn (const char *fname, const char *path)
{
	enum format fmt;
	int fd, rc;
	char magic[6];

	if ((fd = open(fname, O_RDONLY)) < 0) {
		log_error ("%s(): failed to open '%s' for reading\n", __func__, fname);
		return -1;
	}

	if (read(fd, magic, sizeof(magic)) != sizeof(magic)) {
		log_error ("%s(): cannot read magic of '%s'\n", __func__, fname);
		close(fd);
		return -1;
	}
	lseek(fd, 0, SEEK_SET);
	fmt = cpio_decodeMagic(magic);

	switch (fmt) {
		case CPIO_INVALID:		log_msg (LOG_INFO, "%s(): format of '%s' is unknown/unsupported\n", __func__, fname); break;
		case CPIO_BIN_LE:		log_msg (LOG_INFO, "%s(): '%s' is in little endian binary format\n", __func__, fname); break;
		case CPIO_BIN_BE:		log_msg (LOG_INFO, "%s(): '%s' is in big endian binary format\n", __func__, fname); break;
		case CPIO_ASCII_OLD:	log_msg (LOG_INFO, "%s(): '%s' is in old ASCII format\n", __func__, fname); break;
		case CPIO_ASCII_NEW:	log_msg (LOG_INFO, "%s(): '%s' is in new ASCII format\n", __func__, fname); break;
		case CPIO_ASCII_CRC:	log_msg (LOG_INFO, "%s(): '%s' is in new ASCII format + CHKSUM\n", __func__, fname); break;
	}

	rc = -1;
	if (fmt != CPIO_INVALID) {
		while ((rc = cpio_readFile (fd, path, fmt)) == 0) ;
	}
	close (fd);
	return (rc < 0) ? rc : 0;
}
