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
#include "rb2.h"
#include "yaffsfs.h"

static void copy_clean_path (char *buf, const char *path)
{
	const char *s;
	char *p;
	bool slash;

	p = buf;
	while (*p) p++;
	if (p == buf || p[-1] != '/') *p++ = '/';	// the current path now end in a slash character
	slash = true;
	*p = 0;

//	printf ("%s(%s, %s)\n", __func__, buf, path);

	s = path;
	while (*s) {
		if (slash && s[0] == '.' && s[1] == '.' && (s[2] == '/' || s[2] == 0)) {    // go one directory up
			s += 2;
			if (*s) s++;	// if s = "../" also skip the '/'
			p--;
			while (p > buf && p[-1] != '/') p--;    // go back one directory
			if (p == buf) *p++ = '/';		    	// from whereever we get up - we always stay below root
			*p = 0;
		} else if (slash && s[0] == '.' && s[1] == '/') { // stay in current directory
			s += 2;
		} else if (slash && s[0] == '.' && s[1] == 0) { // stay in current directory and advance 's' to the null byte
			s += 1;
		} else if (*s == '/') {
			if (!slash) *p++ = '/';
			slash = true;
			s++;
		} else {
			*p++ = *s++;
			slash = false;
		}
	}
	*p = 0;
	if (slash && p > buf + 1) p[-1] = 0;	// kill slash at end of path (if not root slash)
//	printf ("%s() = '%s'\n", __func__, buf);
}

char *canonical_path(char *buf, const char *cwd, const char *fname)
{
	char *s;

	*buf = 0;
	s = (char *) (fname + strlen(fname));	    // s now points to the terminating null byte
	// strip off tralling CR/NL and '.' and '/'
	while (s > fname && (s[-1] == '\r' || s[-1] == '\n')) *--s = 0;
//	printf("%s() cwd='%s', fname='%s'\n", __func__, cwd, fname);

	if (*fname != '/') {    // we'v got a relative pathname, so start with cwd copied to buffer
		copy_clean_path(buf, cwd);
	}
	copy_clean_path(buf, fname);
//	printf ("%s() = '%s'\n", __func__, buf);
	return buf;
}

static int _ensure_path (char *fname)
{
	struct yaffs_stat st;
	char *s;
	int rc;

//	printf ("%s(): '%s'\n", __func__, fname);
	if (*fname == 0) return 0;		// OK, the root directory always exists

	s = fname + strlen(fname) - 1;
	while (s > fname && *s != '/') s--;

	if (*s != '/') return -1;		// no '/' found ... cannot proceed
	*s = 0;							// temporarily truncate the path at this point
	rc = _ensure_path(fname);
	*s = '/';						// reset this character to the '/' that was found earlier
	if (rc) return rc;				// oh no, we had a failure up in thre tree!

	rc = yaffs_stat(fname, &st);
	if (rc != 0) {		// stat() failed - this directory does not exist: create it
		rc = yaffs_mkdir(fname, S_IREAD | S_IWRITE | S_IEXEC);
	} else {
		if ((st.st_mode & S_IFMT) != S_IFDIR) return -1;		// file exists but is not a directory
	}
	return rc;
}

/**
 * Make sure that the path to the given file exists (i.e. "mkdir -p ...").
 * The last part usually denotes a plain file. If you want to specify only
 * directories (i.e. the last part should count as directory, too), just append
 * a slash '/' character.
 *
 * \param fname		the name of the file you wish to have it's directory ensured
 * \return			0 on success or -1 on error
 */
int ensure_path (const char *fname)
{
	char path[FILENAME_MAX], *s;

	if (!fname || !*fname) return 0;		// nothing to do, but that's OK
	canonical_path(path, "/", fname);
	s = path + strlen(path) - 1;
	while (s > path && *s != '/') s--;
	*s = 0;
//	printf ("%s(): '%s'\n", __func__, path);
	return _ensure_path(path);
}
