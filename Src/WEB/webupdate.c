/*
 * webupdate.c
 *
 *  Created on: 10.03.2021
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

#include <sys/stat.h>
#include "rb2.h"
#include "config.h"
#include "yaffsfs.h"

/**
 * Recursively remove a given directory.
 *
 * \param dir		the absolute path of the directory to be removed
 * \return			0 for OK, an error code otherwise
 */
static int webup_removeDirectory (const char *path)
{
	yaffs_DIR *dir;
	struct yaffs_dirent *dentry;
	struct yaffs_stat st;
	bool rescan;
	char *fname;

	if (yaffs_access(path, 0) != 0) return 0;		// the directory does not exist, so it is OK
	fname = tmp256();
	if ((dir = yaffs_opendir(path)) != NULL) {
		do {
			yaffs_rewinddir(dir);
			rescan = false;
			while ((dentry = yaffs_readdir(dir)) != NULL) {
				snprintf (fname, 256, "%s/%s", path, dentry->d_name);
				yaffs_lstat(fname, &st);
				switch (st.st_mode & S_IFMT) {
					case S_IFREG:
					case S_IFLNK:
//						log_msg (LOG_INFO, "%s(): %s %s\n", __func__, ((st.st_mode & S_IFMT) == S_IFREG) ? "FILE" : "SYMLINK", fname);
						if (yaffs_unlink(fname) < 0) {
							log_error ("%s(): cannot remove '%s'\n", __func__, fname);
							yaffs_closedir(dir);
							return -1;
						}
						rescan = true;
						break;
					case S_IFDIR:
//						log_msg (LOG_INFO, "%s(): DIRECTORY %s\n", __func__, fname);
						if (webup_removeDirectory(fname) != 0) return -1;
						rescan = true;
						break;
				}
			}
		} while (rescan);
		yaffs_closedir(dir);
	}

	if (yaffs_rmdir(path) < 0) {		// remove the directory itself
		log_error ("%s(): cannot remove DIR '%s'\n", __func__, path);
		return -1;
	}
	return 0;
}

static bool webup_assureDirectory (const char *dir)
{
	if (yaffs_access (dir, 0) != 0) {	// check if <dir> directory exists - if not, create it
		if (yaffs_mkdir(dir, S_IREAD | S_IWRITE | S_IEXEC) != 0) {
			log_error("%s(): cannot create %s directory\n", __func__, dir);
			return false;
		}
		log_msg(LOG_INFO, "%s() '%s' created\n", __func__, dir);
	}
	return true;
}

/**
 * Remove last backup ("/html.old") and move current "/html" to this
 * backup location.
 * Create a new (empty) "/html" and in there create two symbolic links
 * to "/config" and "/uploads" respectively.
 *
 * \return		0 if everything went OK, an errorcode otherwise
 */
static int webup_prepare (void)
{
	if (!webup_assureDirectory("/tmp")) return -1;				// make sure /tmp exists
	if (!webup_assureDirectory("/userimages")) return -1;		// make sure /userimages exists

	if (webup_removeDirectory("/tmp/html") < 0) {
		log_error("%s(): cannot remove old extraction from /tmp\n", __func__);
		return -1;
	}
	log_msg(LOG_INFO, "%s() '/tmp/html' removed\n", __func__);

	return 0;
}

/**
 * Do final cleanup and return the given resultcode
 *
 * \param cpio	the filename of the downloaded file to be removed
 * \param rc	the overall result code that should be returned to the caller
 * \return		the value given in rc
 */
static int webup_cleanup (const char *cpio, int rc)
{
	if (webup_removeDirectory("/tmp/html") < 0) {
		log_error("%s(): cannot remove old extraction from /tmp\n", __func__);
	} else {
		log_msg(LOG_INFO, "%s() '/tmp/html' removed\n", __func__);
	}
	if (webup_removeDirectory("/html.old") < 0) {	// may be a leftover from older software versions
		log_error("%s(): cannot remove old extraction from /tmp\n", __func__);
	}
	if (yaffs_unlink(cpio) < 0) {
		log_error ("%s(): cannot remove '%s'\n", __func__, cpio);
	} else {
		log_msg(LOG_INFO, "%s() '%s' cleaned up\n", __func__, cpio);
	}
	return rc;
}

int webup_manuals (void)
{
	yaffs_DIR *dir;
	struct yaffs_dirent *dentry;
	struct yaffs_stat st;
	char *fname, *path, *target;

	path = "/html/documents";
	if ((dir = yaffs_opendir(path)) != NULL) {
		while ((dentry = yaffs_readdir(dir)) != NULL) {
			fname = tmp256();
			snprintf (fname, 256, "%s/%s", path, dentry->d_name);
			target = tmp256();
			snprintf (target, 256, "%s/%s", MANUALS_DIR, dentry->d_name);
			yaffs_lstat(fname, &st);
			switch (st.st_mode & S_IFMT) {
				case S_IFREG:
					log_msg (LOG_INFO, "%s(): moving '%s' to '%s'\n", __func__, fname, target);
					yaffs_unlink(target);
					yaffs_rename(fname, target);
					break;
			}
		}
	}
	if ((dir = yaffs_opendir(MANUALS_DIR)) != NULL) {
		while ((dentry = yaffs_readdir(dir)) != NULL) {
			fname = tmp256();
			snprintf (fname, 256, "%s/%s", path, dentry->d_name);
			target = tmp256();
			snprintf (target, 256, "%s/%s", MANUALS_DIR, dentry->d_name);
			if (yaffs_access(fname, R_OK) != 0) {
				log_msg (LOG_INFO, "%s(): linking '%s' to '%s'\n", __func__, fname, target);
				yaffs_symlink(target, fname);
			}
		}
	}
	return 0;
}

int webup_update (const char *cpio)
{
	if (webup_prepare()) return webup_cleanup(cpio, -1);
	if (cpio_copyIn(cpio, "/tmp") < 0) return webup_cleanup(cpio, -1);

	if (webup_removeDirectory("/tmp/html/config") < 0) {
		log_error("%s(): cannot remove extracted html/config directory\n", __func__);
		return webup_cleanup(cpio, -1);
	}

	if (yaffs_symlink("/config", "/tmp/html/config") < 0) {
		log_error ("%s(): cannot create symlink '/tmp/html/config' -> '/config'\n", __func__);
		return webup_cleanup(cpio, -1);
	}
	log_msg(LOG_INFO, "%s() created symlink '/tmp/html/config' -> '/config'\n", __func__);

	if (yaffs_symlink("/userimages", "/tmp/html/userimages") < 0) {
		log_error ("%s(): cannot create symlink '/tmp/html/userimages' -> '/userimages'\n", __func__);
		return webup_cleanup(cpio, -1);
	}
	log_msg(LOG_INFO, "%s() created symlink '/tmp/html/userimages' -> '/userimages'\n", __func__);

	if (webup_removeDirectory("/html") < 0) {
		log_error("%s(): cannot remove old '/html' directory\n", __func__);
		return webup_cleanup(cpio, -1);
	}
	if (yaffs_rename("/tmp/html", "/html") < 0) {
		log_error ("%s(): cannot move '/tmp/html' into place\n", __func__);
		return webup_cleanup(cpio, -1);
	}
	log_msg(LOG_INFO, "%s() moved '/tmp/html' into place\n", __func__);

	return webup_cleanup(cpio, 0);
}
