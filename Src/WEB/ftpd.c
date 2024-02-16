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
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "rb2.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "yaffsfs.h"

#define MAX_USERNAME		64		///< maximum length for the user name
#define FTP_PORT			21		///< the port we are listening on in normal FTP-Mode
//#define WWW_PORT			221		///< the port we are listening on in HTML FTP-Mode (for updating webserver contents)
#define DYNPORT_MIN			45000	///< an arbitrary port range for PASV command
#define DYNPORT_MAX			45500	///< end of port range for PASV
#define COPY_BUF_SIZE		4096	///< size for an allocated buffer when storing files to SPIFFS

#define FTP_MAIN_STACK		2048	///< stack size of FTP main thread
#define FTP_SUB_STACK		1024	///< stack size of FTP helper thread
#define FTP_PRIO			2		///< the priority for the main thread

#define ANS_LOGOUT			221		///< the answer code for closing the connection
#define ANS_PORTERR			421		///< the answer code for "unable to open port" and subsequently closing the connection

enum datamode {
	ASCII,
	EBCDIC,
	BINARY,
};

enum transfermode {
	STREAM,
	BLOCK,
	COMPRESSED,
};

enum connmode {
	MODE_NORMAL,
	MODE_PASSIVE,
};

struct context {
	int				cmdsocket;		///< a socket for command transfer
	int				datasocket;		///< a socket for data transfer if opened or -1 otherwise
	in_addr_t		remote;			///< the ip address of the remote side
	enum datamode	datamode;		///< the data mode (ASCII or BINARY) for transfers
	enum transfermode	transfermode;		///< the transfermode (STREAM only implemented)
	in_addr_t		datahost;		///< the host to which we want a data connection
	uint16_t		dataport;		///< the port numer we must connect to for the data transfer
	enum connmode	mode;			///< the data connection mode (active or passive)
	TaskHandle_t	helper;			///< a running helper task
	char			fname[NAME_MAX];	///< the name of a file to retrieve or store (RETR / STOR / APPE)
	char			cwd[NAME_MAX];	///< the name of the current working diretory
	char			root[16];		///< a pseudo root diretory
	bool			append;			///< if true and the file already exists, the file is opened in append mode
};

struct command {
	const char	    *cmd;		///< the command to compare
	int (*func)(struct context *, const struct command *, char *);	///< the function to call for this command
};

static const struct reply {
	int		code;		///< the code in the answer
	const char	*msg;		///< the message string appended to the code
} replies[] = {
		{ 125, "Data connection already open; transfer starting." },
		{ 150, "File status okay; about to open data connection." },

		{ 200, "Command okay." },
		{ 202, "Command not implemented, superfluous at this site." },
		{ 215, "UNIX Type: L8" },
		{ 220, "Service ready for new user." },
		{ 221, "Service closing control connection." },
		{ 226, "Closing data connection." },
		{ 230, "User logged in, proceed." },
		{ 250, "Requested file action okay, completed." },

		{ 331, "User name okay, need password." },
		{ 350, "Requested file action pending further information." },

		{ 425, "Can't open data connection." },
		{ 426, "Connection closed; transfer aborted." },
		{ 450, "Requested file action not taken." },

		{ 500, "Syntax error, command unrecognized." },
		{ 501, "Syntax error in parameters or arguments." },
		{ 502, "Command not implemented." },
		{ 503, "Bad sequence of commands." },
		{ 504, "Command not implemented for that parameter." },
		{ 530, "Not logged in." },
		{ 550, "Requested action not taken. File unavailable." },
		{ 552, "Requested file action aborted. Exceeded storage allocation." },
		{ 553, "Requested action not taken. Filename not allowed." },

		{ 0, NULL }	    // end of list
};

static void ftpd_sendstring (int sock, const char *str)
{
	const char *s;
	int len, sent;

	len = strlen(str);
	s = str;
	while (len > 0) {
		sent = lwip_write(sock, s, len);
		if (sent <= 0) break;
		s += sent;
		len -= sent;
	}
}

static void ftpd_answer_ex (int sock, int code, const char *msg)
{
	char buf[128];

	snprintf (buf, sizeof(buf), "%d %s\r\n", code, msg);
	ftpd_sendstring(sock, buf);
	//    printf (buf);
}

static void ftpd_answer (int sock, int code)
{
	const struct reply *rp;

	rp = replies;
	while (rp->code != 0 && rp->code != code) rp++;
	if (rp->code != 0) ftpd_answer_ex(sock, code, rp->msg);
	else ftpd_answer_ex(sock, 500, "INTERNAL SERVER ERROR");
}

static int ftpd_datasocket (struct context *ctx)
{
	int datasock;
	struct sockaddr_in laddr;
	socklen_t slen;

	switch (ctx->mode) {
		case MODE_NORMAL:
			if (ctx->datasocket >= 0) lwip_close(ctx->datasocket);
			ctx->datasocket = -1;
			if ((datasock = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
				fprintf(stderr, "%s(ACTIVE): failed to create socket\n", __func__);
				ctx->datasocket = -1;
				return -1;
			}

			laddr.sin_family = AF_INET;
			laddr.sin_port = ctx->dataport;
			laddr.sin_addr.s_addr = ctx->datahost;
			slen = sizeof(laddr);
			if (lwip_connect(datasock, (struct sockaddr *) &laddr, slen) < 0) {
				lwip_close(datasock);
				return -1;
			}
			break;

		case MODE_PASSIVE:  // the ctx->datasocket is already opened and in a listening state - now accpet incoming connection
			if (ctx->datasocket < 0) return -1;
			slen = sizeof(laddr);
			datasock = lwip_accept(ctx->datasocket, (struct sockaddr *) &laddr, &slen);
			lwip_close (ctx->datasocket);
			ctx->datasocket = -1;
			if (datasock < 0) return -1;
			ctx->datahost = 0;
			ctx->dataport = 0;
			break;
		default:
			return -1;
	}
	return datasock;
}

static int ftpd_user (struct context *ctx, const struct command *c, char *cmd)
{
	(void) ctx;
	(void) c;
	(void) cmd;

	return 220;
}

static int ftpd_pass (struct context *ctx, const struct command *c, char *cmd)
{
	(void) ctx;
	(void) c;
	(void) cmd;

	return 220;
}

static int ftpd_quit (struct context *ctx, const struct command *c, char *cmd)
{
	(void) ctx;
	(void) c;
	(void) cmd;

	return 221;
}

static int ftpd_syst (struct context *ctx, const struct command *c, char *cmd)
{
	(void) ctx;
	(void) c;
	(void) cmd;

	return 215;
}

static int ftpd_port (struct context *ctx, const struct command *c, char *cmd)
{
	int h1, h2, h3, h4,  p1, p2;

	(void) c;

	if (sscanf(cmd, "%d,%d,%d,%d,%d,%d", &h1, &h2, &h3, &h4, &p1, &p2) != 6) {
		return 501;	// syntax error in parameters
	}

	ctx->datahost = htonl(h1 << 24 | h2 << 16 | h3 << 8 | h4);
	ctx->dataport = htons(p1 << 8 | p2);
	ctx->mode = MODE_NORMAL;
	return 200;
}

static int ftpd_pasv (struct context *ctx, const struct command *c, char *cmd)
{
	struct sockaddr_in laddr;
	socklen_t slen;
	char reply[32];
	int dsock;
	uint16_t port;
	uint32_t addr;

	(void) c;
	(void) cmd;

	if (ctx->datasocket >= 0) {
		lwip_close(ctx->datasocket);
		ctx->datasocket = -1;
	}
	if ((dsock = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		fprintf(stderr, "%s() cannot open socket\n", __func__);
		return 421;
	}

	laddr.sin_family = AF_INET;
	laddr.sin_addr.s_addr = INADDR_ANY;
	laddr.sin_len = sizeof(laddr);
	for (port = DYNPORT_MIN; port < DYNPORT_MAX; port++) {
		/* Bind the socket to that port. */
		laddr.sin_port = htons(port);
		if (lwip_bind (dsock, (struct sockaddr *) &laddr, sizeof(laddr)) == 0) break;
	}
	if (port >= DYNPORT_MAX) {
		fprintf(stderr, "%s(): cannot bind to any port in foreseen range\n", __func__);
		lwip_close(dsock);
		return 421;
	}

	/* Set the socket into a listening state so it can accept connections.
    The maximum number of simultaneous connections is limited to 1. */
	if (lwip_listen (dsock, 1) != 0) {
		fprintf (stderr, "%s(): listen failed\n", __func__);
		lwip_close(dsock);
		return 421;
	}
	ctx->datasocket = dsock;
	ctx->mode = MODE_PASSIVE;

	slen = sizeof(laddr);
	lwip_getsockname(ctx->cmdsocket, (struct sockaddr *) &laddr, &slen);
	addr = ntohl(laddr.sin_addr.s_addr);
	snprintf (reply, sizeof(reply), "=(%ld,%ld,%ld,%ld,%d,%d)",
			(addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, (addr >> 0) & 0xFF,
			(port >> 8) & 0xFF, (port >> 0) & 0xFF);
	ftpd_answer_ex(ctx->cmdsocket, 227, reply);

	return 0;
}

static int ftpd_type (struct context *ctx, const struct command *c, char *cmd)
{
	(void) c;

	switch (toupper(*cmd)) {
		case 'A':
			ctx->datamode = ASCII;
			break;
		case 'I':
			ctx->datamode = BINARY;
			break;
		default:
			return 504;
	}
	return 200;
}

static int ftpd_mode (struct context *ctx, const struct command *c, char *cmd)
{
	(void) c;

	switch (toupper(*cmd)) {
		case 'S':
			ctx->transfermode = STREAM;
			break;
		default:
			return 504;
	}
	return 200;
}

static int ftpd_structure (struct context *ctx, const struct command *c, char *cmd)
{
	(void) c;

	switch (toupper(*cmd)) {
		case 'S':
			ctx->transfermode = STREAM;
			break;
		default:
			return 504;
	}
	return 200;
}

//static char *ftpd_fname_cleanup (char *fname)
//{
//    char *s;
//    
//    while (*fname && (isspace(*fname) || *fname == '/')) fname++;
//    s = fname;
//    while (*s && *s != '\r' && *s != '\n') s++;
//    *s-- = 0;
//    while (s >= fname && (isspace(*s) || *s == '/')) *s-- = 0;
//    
//    return fname;
//}

static int ftpd_dele (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];
	int rc;

	(void) c;

	snprintf (ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, cmd));
//	printf("%s(): Filename '%s'\n", __func__, ctx->fname);

	if ((rc = yaffs_unlink(ctx->fname)) == 0) return 250;
	else fprintf (stderr, "%s(): yaffs_unlink() = %d\n", __func__, rc);
	return 550;
}

static void retr_thread (void *pvParameters)
{
	char buf[128];
	struct context *ctx;
	int clntsock, len, rc, fd;

	ctx = (struct context *) pvParameters;

	if ((clntsock = ftpd_datasocket(ctx)) < 0) {
		fprintf(stderr, "%s(): cannot open data connection\n", __func__);
		ftpd_answer(ctx->cmdsocket, 425);
		vTaskDelete(NULL);
	}

	if ((fd = yaffs_open(ctx->fname, O_RDONLY, 0)) <= 0) {
		fprintf(stderr, "%s(): Cannot open '%s'\n", __func__, ctx->fname);
		rc = 550;
	} else {
		while ((len = yaffs_read(fd, buf, sizeof(buf))) > 0) {
			lwip_write(clntsock, buf, len);
		}
		yaffs_close(fd);
		rc = 250;
	}

	lwip_close(clntsock);
	ftpd_answer(ctx->cmdsocket, rc);
	vTaskDelete(NULL);
}

static int ftpd_retr (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];
	TaskHandle_t tid;

	(void) c;

	snprintf (ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, cmd));
	//    printf("%s(): Filename '%s'\n", __func__, ctx->fname);

	ftpd_answer(ctx->cmdsocket, 150);
	xTaskCreate(retr_thread, "FTP-RETR", FTP_SUB_STACK, ctx, 1, &tid);

	return 0;
}

static void stor_thread (void *pvParameters)
{
	char *buf;
	struct context *ctx;
	int fd, clntsock, len, rc, size, oflag;

	ctx = (struct context *) pvParameters;
	if ((clntsock = ftpd_datasocket(ctx)) < 0) {
		fprintf(stderr, "%s(): cannot open data connection\n", __func__);
		ftpd_answer(ctx->cmdsocket, 425);
		vTaskDelete(NULL);
	}

	if ((buf = malloc (COPY_BUF_SIZE)) == NULL) {
		lwip_close(clntsock);
		ftpd_answer(ctx->cmdsocket, 552);
		vTaskDelete(NULL);
	}

	oflag = O_WRONLY | O_CREAT | O_TRUNC;
	if (ctx->append) oflag = O_WRONLY | O_CREAT | O_APPEND;
	if ((fd = yaffs_open(ctx->fname, oflag, S_IREAD | S_IWRITE)) < 0) {
		fprintf(stderr, "%s(): Cannot open '%s'\n", __func__, ctx->fname);
		rc = 550;
	} else {
		size = 0;
		while ((len = lwip_read(clntsock, buf, COPY_BUF_SIZE)) > 0) {
			size += len;
			rc = yaffs_write(fd, buf, len);
			if (rc != len) fprintf (stderr, "%s('%s') write(%d) = %d\n", __func__, ctx->fname, len, rc);
		}
		yaffs_close(fd);
//		printf ("%s('%s') FD=%d TOTAL %d bytes written\n", __func__, ctx->fname, clntsock, size);
		rc = 250;
	}

	free (buf);
	lwip_close(clntsock);
	ftpd_answer(ctx->cmdsocket, rc);
	vTaskDelete(NULL);
}

static int ftpd_stor (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];
	TaskHandle_t tid;

	ctx->append = (!strcmp ("APPE", c->cmd));		// for the append command we set the append flag to true
	snprintf (ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, cmd));
	//    printf("%s(): Filename '%s'\n", __func__, ctx->fname);

	ftpd_answer(ctx->cmdsocket, 150);
	xTaskCreate(stor_thread, "FTP-STOR", FTP_SUB_STACK, ctx, 1, &tid);

	return 0;
}

static void list_thread (void *pvParameters)
{
	char line[NAME_MAX + 32];
	char path[NAME_MAX], file[NAME_MAX], mode[16];
	struct context *ctx;
	int clntsock;
	yaffs_DIR *d;
	struct yaffs_dirent *de;
	struct yaffs_stat stat;

	ctx = (struct context *) pvParameters;

//	printf ("%s(): root='%s', cwd='%s'\n", __func__, ctx->root, ctx->cwd);
	if ((clntsock = ftpd_datasocket(ctx)) < 0) {
		fprintf(stderr, "%s(): cannot open data connection\n", __func__);
		ftpd_answer(ctx->cmdsocket, 425);
		vTaskDelete(NULL);
	}

	snprintf (ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, "."));
	if ((d = yaffs_opendir(ctx->fname)) == NULL) {
		fprintf (stderr, "%s(%s) %s -> %s cannot open dir (errno %d)\n", __func__, ctx->fname, ctx->root, path, errno);
		ftpd_answer(ctx->cmdsocket, 550);
		vTaskDelete(NULL);
	}

	while ((de = yaffs_readdir(d)) != NULL) {
//		printf("%s(%s): %s\n", __func__, ctx->fname, de->d_name);
		snprintf (file, sizeof(file), "%s/%s", ctx->fname, de->d_name);
		yaffs_lstat (file, &stat);
		sprintf (mode, "----------");
		if (stat.st_mode & S_IREAD) mode[1] = mode[4] = mode[7] = 'r';
		if (stat.st_mode & S_IWRITE) mode[2] = mode[5] = mode[8] = 'w';
		if (stat.st_mode & S_IEXEC) mode[3] = mode[6] = mode[9] = 'x';
		switch (stat.st_mode & S_IFMT) {
			case S_IFDIR:
				mode[0] = 'd';
				mode[3] = mode[6] = mode[9] = 'x';
				snprintf (line, sizeof(line), "%s 1 1000 1000 0 Jan 01 2020 %s\r\n", mode, de->d_name);
				break;
			case S_IFREG:
				snprintf (line, sizeof(line), "%s 1 1000 1000 %lu Jan 01 2020 %s\r\n", mode, (uint32_t) stat.st_size, de->d_name);
				break;
			case S_IFLNK:
				sprintf (mode, "lrwxrwxrwx");
				snprintf (line, sizeof(line), "lrwxrwxrwx 1 1000 1000 %lu Jan 01 2020 %s\r\n", (uint32_t) stat.st_size, de->d_name);
				break;
		}
//		printf ("%s(): %s", __func__, line);
		ftpd_sendstring(clntsock, line);
	}
	yaffs_closedir(d);

	lwip_close(clntsock);
	ftpd_answer(ctx->cmdsocket, 226);
	vTaskDelete(NULL);
}

static int ftpd_list (struct context *ctx, const struct command *c, char *cmd)
{
	TaskHandle_t tid;

	(void) c;
	(void) cmd;

	ftpd_answer(ctx->cmdsocket, 150);
	xTaskCreate(list_thread, "FTP-LIST", FTP_SUB_STACK, ctx, 1, &tid);
	return 0;
}

static int ftpd_size (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];
	struct yaffs_stat s;

	(void) c;

	snprintf (ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, cmd));
	//    printf("%s(): Filename '%s'\n", __func__, ctx->fname);
	if (yaffs_lstat(ctx->fname, &s) != 0) return 550;
	sprintf (path, "%lld", s.st_size);
	ftpd_answer_ex(ctx->cmdsocket, 213, path);

	return 0;
}

static int ftpd_cwd (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];
	struct yaffs_stat st;
	int rc;

//	printf ("%s(): root='%s', cwd='%s', cmd='%s'\n", __func__, ctx->root, ctx->cwd, cmd);

	if (!strcmp ("CDUP", c->cmd)) {
		canonical_path(path, ctx->cwd, "..");
	} else {
		canonical_path(path, ctx->cwd, cmd);
	}
	if (!strcmp ("/", path)) {	// we are going back to the root directory
		strcpy (ctx->cwd, path);
		rc = 250;
	} else {		    	// we are not accessing the root directory
		snprintf(ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, path);
		if (yaffs_stat(ctx->fname, &st) == 0 && (st.st_mode & S_IFMT) == S_IFDIR) {
			strcpy (ctx->cwd, path);
			rc = 250;
		} else {
			rc = 550;
		}
	}

//	if (rc == 250) printf("%s(): new path '%s'\n", __func__, path);
//	else printf("%s(): path not allowed '%s'\n", __func__, path);
	return rc;
}

static int ftpd_pwd (struct context *ctx, const struct command *c, char *cmd)
{
	char dirname[NAME_MAX + 4];

	(void) c;
	(void) cmd;

	sprintf (dirname, "\"%s\"", ctx->cwd);
	ftpd_answer_ex(ctx->cmdsocket, 257, dirname);
	return 0;
}

static int ftpd_opts (struct context *ctx, const struct command *c, char *cmd)
{
	(void) ctx;
	(void) c;

	if (!strncasecmp("UTF8 ", cmd,  5)) return 200;
	return 451;
}

static int ftpd_rnfr (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];

	(void) c;

	snprintf (ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, cmd));
//	printf("%s(): Filename '%s'\n", __func__, ctx->fname);

	return 350;
}

static int ftpd_rnto (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];
	char fname[NAME_MAX];

	(void) c;

	snprintf (fname, sizeof(fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, cmd));
	//    printf("%s(): Filename '%s'\n", __func__, fname);

	if (yaffs_rename(ctx->fname, fname) != 0) return 553;
	return 250;
}

static int ftpd_mkdir (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];
	int rc = 250;

	(void) c;

	snprintf (ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, cmd));
//	log_msg (LOG_INFO, "%s(): Dirname '%s'\n", __func__, ctx->fname);
	if (yaffs_mkdir(ctx->fname, S_IREAD | S_IWRITE | S_IEXEC)) rc = 550;
	return rc;
}

static int ftpd_rmdir (struct context *ctx, const struct command *c, char *cmd)
{
	char path[NAME_MAX];
	int rc = 550;

	(void) c;

	snprintf (ctx->fname, sizeof(ctx->fname), "%s%s", ctx->root, canonical_path(path, ctx->cwd, cmd));
	if (yaffs_rmdir(ctx->fname) == 0) rc = 250;
	return rc;
}

static int ftpd_noop (struct context *ctx, const struct command *c, char *cmd)
{
	(void) ctx;
	(void) c;
	(void) cmd;

	return 200;
}

//static int ftpd_unimplemented (struct context *ctx, const struct command *c, char *cmd)
//{
//    return 502;
//}

static const struct command commands[] = {
		{ "USER", ftpd_user },
		{ "PASS", ftpd_pass },
		{ "QUIT", ftpd_quit },
		{ "SYST", ftpd_syst },
		{ "PORT", ftpd_port },
		{ "PASV", ftpd_pasv },
		{ "TYPE", ftpd_type },
		{ "MODE", ftpd_mode },
		{ "STRU", ftpd_structure },
		{ "RETR", ftpd_retr },
		{ "STOR", ftpd_stor },
		{ "APPE", ftpd_stor },
		{ "DELE", ftpd_dele },
		{ "LIST", ftpd_list },
		{ "SIZE", ftpd_size },
		{ "CWD",  ftpd_cwd },
		{ "CDUP", ftpd_cwd },		// short for "CWD .."
		{ "PWD",  ftpd_pwd },
		{ "OPTS", ftpd_opts },
		{ "RNFR", ftpd_rnfr },
		{ "RNTO", ftpd_rnto },
		{ "MKD",  ftpd_mkdir },
		{ "RMD",  ftpd_rmdir },
		{ "NOOP", ftpd_noop },

		{ NULL, NULL }
};

static int ftpd_parse_command (struct context *ctx, char *cmd)
{
	const struct command *c;
	char *s;

	s = cmd;
	while (*s && !isspace(*s)) s++;
	if (*s) *s++ = 0;

	while (*s && isspace(*s)) s++;
	for (c = commands; c->cmd != NULL; c++) {
		if (!strcasecmp(cmd, c->cmd) && (c->func)) {
//			printf("%s(): exec %s %s\n", __func__, c->cmd, s);
			return c->func(ctx, c, s);
		}
	}

	fprintf(stderr, "%s(): unknown command '%s' '%s'\n",  __func__, cmd, s);
	return 500;
}

static void ftpd (void *pvParameters)
{
	struct sockaddr_in name;
	socklen_t namelen;
	int sock, rc;
	size_t len;
	char buf[128], *s;
	struct context ctx;

	sock = (int) pvParameters;

	namelen = sizeof(name);
	lwip_getpeername(sock, (struct sockaddr *) &name, &namelen);

	ftpd_answer(sock, 220);
	memset (&ctx, 0, sizeof(ctx));
	ctx.remote = name.sin_addr.s_addr;
	ctx.datamode = BINARY;
	ctx.transfermode = STREAM;
	ctx.datasocket = -1;
	ctx.cmdsocket = sock;
	sprintf (ctx.cwd, "/");

	namelen = sizeof(name);
	lwip_getsockname(sock, (struct sockaddr *) &name, &namelen);
//	if (ntohs(name.sin_port) == WWW_PORT) sprintf (ctx.root, WWW_DIR);
	if (ntohs(name.sin_port) == FTP_PORT) sprintf (ctx.root, "/");
	//    printf("%s(): Remote site is '%s'\n", __func__, inet_ntoa(ctx.remote));

	s = buf;
	for (;;) {
		rc = lwip_read(sock, s, sizeof(buf) - (s - buf) - 1);
		if (rc == 0) break;	// closed by the other end

		if (rc > 0) {
			// update len, s (the end-pointer) and terminate the receive buffer as a string
			len = (s - buf) + rc;
			s = &buf[len];
			*s = 0;

			if (strchr(buf, '\r')) {
				rc = ftpd_parse_command(&ctx, buf);
				if (rc != 0) {
					ftpd_answer(ctx.cmdsocket, rc);			// rc == 0 means, that a local answer was generated
					if ((rc == ANS_LOGOUT) || (rc == ANS_PORTERR)) break;	// Close connection
				}
				s = buf;
			} else if (len >= sizeof(buf) - 1) {
				s = buf;    // restart with an empty buffer
			}
		} else {
			fprintf (stderr, "%s():  some error occured in lwip_read() = %d\n", __func__, rc);
			break;
		}
	}

	if ((rc = lwip_close (sock)) != 0) {
		fprintf (stderr, "%s(): close failed with rc=%d\n", __func__, rc);
	}
	yaffs_sync("/");
	printf ("%s(): finished\n",  __func__);
	vTaskDelete(NULL);
}

int ftpd_start (void)
{
	return tcpsrv_startserver(FTP_PORT, ftpd, FTP_MAIN_STACK, FTP_PRIO);
}
