/**
 * @file    httpd.c
 * @author  Andi
 * @date	24.12.2019
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

/**
 * @ingroup HTTPD
 * @{
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "rb2.h"
#include "lwip/sockets.h"
#include "yaffsfs.h"
#include "httpd.h"

#define HTTPD_STACK			2048	///< the thread size for the started HTTPD-threads
#define HTTPD_PRIO			2		///< the priority of the started HTTPD-threads

static const struct contentT {
	const char	*ext;
	const char	*type;
} content_type[] = {
	{ "html",	"text/html" },
	{ "htm",	"text/html" },
	{ "txt",	"text/plain" },
	{ "css",	"text/css" },
	{ "gif",	"image/gif" },
	{ "svg",	"image/svg+xml" },
	{ "ico",	"image/vnd.microsoft.icon" },
	{ "js",		"text/javascript" },
	{ "pdf",	"application/pdf" },
	{ NULL, NULL }
};

static const struct requestT {
	const char		*str;
	const enum req	 request;
} request_type[] = {
	{ "GET", GET },
	{ "HEAD", HEAD },
	{ "POST", POST },
	{ "PUT", PUT },
	{ "DELETE", DELETE },
	{ "CONNECT", CONNECT },
	{ "OPTIONS", OPTIONS },
	{ "TRACE", TRACE },
	{ NULL, UNKNOWN_REQ }
};

/**
 * Parse all headerlines except the first one. It is expected, that the lines
 * are formed as "key: value"
 *
 * @param hdp	a pointer to a pointer to http-request header structure. The address
 *				of the allocated structure is placed here (so this is kind of a VAR-parameter).
 * @param line	the characters received from the socket. They should be line terminated
 *				with at least one CR/NL
 * @return	    the pointer to the line that is to be interpreted next
 */
static const char *httpd_parse_header (struct key_value **hdp, const char *line)
{
	const char *s, *key_end, *val;
	struct key_value *h;
	size_t key_len, val_len;

	if (!hdp || !line) return NULL;
	if (line[0] == '\r' && line[1] == '\n') return NULL;	// END OF HEADER: stop parsing and handle request

	s = line;
	while (*s && *s != '\r' && *s != ':') s++;
	if (*s != ':') return NULL;						// ERROR, stop parsing headers
	key_end = s++;
	while (*s && isspace(*s) && *s != '\r') s++;
	if (isspace(*s) || *s == '\r') return NULL;		// ERROR, stop parsing headers
	val = s;
	while (*s && *s != '\r') s++;
	if (*s != '\r') return NULL;					// ERROR, stop parsing headers

	// we now have key and the value of this header and it is assured, that this header is well formed
	// the length as specified with key_len and val_len does not include the terminating nulls (elsewhere a simple strlen() is used!)
	key_len = key_end - line;
	val_len = s - val;

	if ((h = kv_addEx(NULL, line, key_len, val, val_len)) == NULL) return NULL;
	while (*hdp) hdp = &(*hdp)->next;
	*hdp = h;

	return s + 2;
}

/**
 * Calculate the length of a string by unescaping it.
 * The length will not account for any null bytes needed later. This is
 * consistent with a call strlen() that will ignore the null byte too.
 *
 * \param s		the string to parse up to a space, '=' or '&'
 * \return		the number of character found without any null byte needed to make it a c-string
 */
static int httpd_unescape_len (const char *s)
{
	int len = 0;

	if (!s || !*s) return 0;

	while (*s != '=' && *s != '&' && !isspace(*s)) {	// key and value are terminated by different characters
		len++;
		if (*s == '%') s += 2;
		s++;
	}
	return len;
}

/**
 * Copy a string from s to d and unescape the URI coding using '%xx' coding.
 * The string ends on the first verbatim '=', '&' or whitespace character.
 *
 * @param d		destination string pointer, guarateed to be null terminated - if NULL, nothing will be copied
 * @param s		source string - if NULL, nothing is copied (i.e. results in empty string in d)
 */
static void httpd_unescape_copy (char *d, const char *s)
{
	if (!d) return;		// at least, we've got nothing to copy to - so ignore this call
	*d = 0;				// else we must ensure a zero terminated string
	if (!s) return;		// if we have no source, we are done

	while (*s != '=' && *s != '&' && !isspace(*s)) {
		if (*s == '%') {		// decode hex character
			s++;
			if (*s >= '0' && *s <= '9') *d = *s - '0';
			if (*s >= 'A' && *s <= 'F') *d = *s - 'A' + 10;
			if (*s >= 'a' && *s <= 'f') *d = *s - 'a' + 10;
			*d <<= 4;
			s++;
			if (*s >= '0' && *s <= '9') *d += *s - '0';
			if (*s >= 'A' && *s <= 'F') *d += *s - 'A' + 10;
			if (*s >= 'a' && *s <= 'f') *d += *s - 'a' + 10;
			d++;
			s++;
		} else {				// copy verbatim
			*d++ = *s++;
		}
	}
	*d = 0;
}

static struct key_value *httpd_gen_param (const char *key, const char *val)
{
	struct key_value *kv;
	int klen, vlen;

	if (!key) return NULL;	// a key is always required - a value is optional

	klen = httpd_unescape_len(key);
	if (val && *val) {
		vlen = httpd_unescape_len(val);
	} else {
		vlen = 0;
	}

//	printf ("%s() kv_addEx(NULL, '%*.*s', %d, '%*.*s', %d)\n", __func__, klen, klen, key, klen, vlen, vlen, (val) ? val : "", vlen);
//	vTaskDelay(20);
	if ((kv = kv_addEx(NULL, key, klen, val, vlen)) == NULL) return NULL;
	httpd_unescape_copy(kv->key, key);
	httpd_unescape_copy(kv->value, val);

	return kv;
}

static struct key_value *httpd_parse_params (const char *line)
{
	struct key_value *ret, *kv, **kvpp;
	const char *s, *key, *val;

	if (!line) return NULL;

	ret = NULL;
	kvpp = &ret;
	key = val = NULL;
	s = line;
	while (*s && !isspace(*s) && *s != '\r') {
		if (!key) {
			key = s;
			val = NULL;
		}
		switch (*s) {
			case '%':		// two digit hex coding follows
				if (!isxdigit(s[1]) || !isxdigit(s[2])) {	// ERROR - stop parsing and return what we have up to here
					return ret;
				}
				s += 2;
				break;
			case '&':		// a new key/value pair follows
				kv = httpd_gen_param (key, val);
				key = val = NULL;
				*kvpp = kv;
				if (kv) kvpp = &kv->next;
				break;
			case '=':		// this was the key, now comes the value
				val = s + 1;
				break;
		}
		s++;
	}

	// now parse the last key/value pair (if any)
	kv = httpd_gen_param (key, val);
	*kvpp = kv;

	return ret;
}

static enum req httpd_getRequestType (const char *s)
{
	const struct requestT *r = request_type;

	if (!s || !*s) return UNKNOWN_REQ;

	while (r->str != NULL) {
		if (!strncmp (s, r->str, strlen(r->str))) break;
		r++;
	}

	return r->request;
}

static const char *httpd_request2string (enum req request)
{
	const struct requestT *r = request_type;

	while (r->str != NULL) {
		if (r->request == request) break;
		r++;
	}

	return (r->str) ? r->str : "(UNKNOWN)";
}

/**
 * Try to interpret the first line of the HTTP request header.
 * We are expecting a line like 'GET /index.html HTTP/1.1'.
 * 
 * @param hrp	a pointer to a pointer to http-request header structure. The address
 *				of the allocated structure is placed here (so this is kind of a VAR-parameter).
 * @param line	the characters received from the socket. They should be line terminated
 *				with at least one CR/NL
 * @return	    the pointer to the line that is to be interpreted next
 * @see			httpd_parse_header() for parsing the remaining lines of the request header
 */
static const char *httpd_parse_request (struct http_request **hrp, const char *line)
{
	const char *s, *uri, *version, *params, *q;
	char *p;
	size_t uri_len, version_len;
	struct http_request *hr;
	enum req request;

	if (!hrp || !line) return NULL;

	*hrp = NULL;

//	log_msg (LOG_HTTPD, "%s(): '%s'\n", __func__, line);
//	vTaskDelay(20);

	request = httpd_getRequestType(line);
	s = line;
	while (*s && !isspace(*s)) s++;		// position behind the method verb

	// scan URI and parameters
	while (*s && isspace(*s) && *s != '\r') s++;
	if (!*s || *s == '\r') return NULL;			// ERROR: no URI found
	uri = s;
	params = NULL;
	uri_len = 0;
	while (*s && !isspace(*s) && *s != '\r') {
		if (*s == '?') {
			params = s + 1;
			uri_len = s - uri + 1;
		}
		s++;
	}
	if (!*s || *s == '\r') return NULL;			// ERROR: no end of URI found
	if (!uri_len) uri_len = s - uri + 1;

//	log_msg (LOG_HTTPD, "%s(): %*.*s\n", __func__, (s - uri), (s - uri), uri);	// DEBUG complete request with parameters instead of httpd_debug_request()

	while (*s && isspace(*s) && *s != '\r') s++;
	if (!*s || *s == '\r') return NULL;			// ERROR: no HTTP-VERSION found
	version = s;
	while (*s && *s != '\r') s++;
	version_len = s - version + 1;

	// now let's alloc a buffer that includes all parts: the structure, the uri and the version string
	// still not included are the URI parameters (if any) and the other header lines
	if ((hr = malloc (sizeof(*hr) + uri_len + version_len)) == NULL) {
		return NULL;
	}

	hr->headers = NULL;
	hr->request = request;
	hr->uri = ((char *) hr) + sizeof(*hr);
	hr->param = httpd_parse_params(params);
	hr->version = hr->uri + uri_len;

	p = hr->uri;
	q = uri;
	while (*q && !isspace(*q) && *q != '?') *p++ = *q++;
	*p = 0;

	p = hr->version;
	q = version;
	while (*q && *q != '\r') *p++ = *q++;
	*p = 0;

	*hrp = hr;
	return s + 2;	// the beginning of the next line
}

/**
 * Free the given request and take care of the substructures thereof.
 *
 * @param hr	the request to free
 */
void httpd_free_request (struct http_request *hr)
{
	if (!hr) return;

	kv_free(hr->headers);
	kv_free(hr->param);
	free (hr);
}

void httpd_header (int sock, const char *status, struct key_value *hdrs)
{
	socket_sendstring (sock, status);
	while (hdrs) {
		socket_printf (sock, "%s: %s\r\n", hdrs->key, hdrs->value);
		hdrs = hdrs->next;
	}
	socket_sendstring (sock, HEADER_END);
}

static const char *httpd_contentType (char *ext)
{
	const struct contentT *ct;

	ct = content_type;
	while (ct->ext != NULL) {
		if (!strcasecmp(ct->ext, ext)) return ct->type;
		ct++;
	}
	return CONTENT_TEXT;
}

void httpd_serve_file (int sock, char *uri, struct key_value *hdrs)
{
	struct key_value *h;
	char path[NAME_MAX], fname[NAME_MAX], *ext;
	uint8_t buf[256];
	int fd, len;

	snprintf (fname, sizeof(fname), WWW_DIR "%s", canonical_path(path, "", uri));
	ext = path + strlen(path);
	while (ext > path && ext[-1] != '.') ext--;
//	printf("%s() URI '%s' -> PATH '%s' (ext='%s')\n", __func__, uri, fname, ext);

	h = kv_add (hdrs, "Server", SERVER_STRING);
	if (!hdrs) hdrs = h;

	if ((fd = yaffs_open(fname, O_RDONLY, 0)) >= 0) {
		h = kv_add (h, "Content-Type", httpd_contentType(ext));
		if (!strcmp (HTML_404, uri)) {
			httpd_header(sock, FILE_NOT_FOUND, hdrs);
		} else {
			httpd_header(sock, FILE_OK, hdrs);
		}

		while ((len = yaffs_read(fd, buf, sizeof(buf))) > 0) {
			if (socket_senddata(sock, buf, len) != len) break;
		}
		yaffs_close(fd);
	} else {
		// do not recurse if the HTML404 file is not found!
		if (!strcmp (HTML_404, uri)) {
			// we already tried to send the 404.html which doesn't seem to exist
			h = kv_add (h, "Content-type", CONTENT_TEXT);
			httpd_header(sock, FILE_NOT_FOUND, hdrs);
			socket_sendstring (sock, "\r\n");
		} else {
			httpd_serve_file(sock, HTML_404, NULL);
		}
	}

	kv_free(hdrs);
}

static void httpd_serve_request (int sock, struct http_request *hr, const char *rest, int sz)
{
	if (!hr || !hr->uri) return;

	switch (hr->request) {
		case GET:
			if (!strcmp(hr->uri, "/") || *hr->uri == 0) {
				httpd_serve_file(sock, "/index.html", NULL);
			} else {
				if (cgi_check_request(sock, hr, rest, sz)) return;
				httpd_serve_file(sock, hr->uri, NULL);
			}
			break;
		case POST:
		case PUT:
			if (cgi_check_request(sock, hr, rest, sz)) return;
			httpd_header(sock, METHOD_NOT_ALLOWED, NULL);
			break;
		default:
			httpd_header(sock, METHOD_NOT_ALLOWED, NULL);
			break;
	}
}

#if 0		// DEBUG-function currently not used (to keep compiler happy)
static void httpd_debug_keyValue (int indent, struct key_value *kv)
{
	while (kv) {
		log_msg (LOG_HTTPD, "\t%*.*s%s=%s\n", indent, indent, "", kv->key, kv->value);
		kv = kv->next;
	}
}
#endif

static void httpd_debug_request (struct http_request *hr)
{
	struct key_value *kv;
	char *tmp = tmp1k();
	char *s;

	if (!hr) return;
	s = tmp;
	s += sprintf (s, "%s(%p): %s %s", __func__, xTaskGetCurrentTaskHandle(), httpd_request2string(hr->request), hr->uri);
#if 1
	kv = hr->param;
	while (kv) {
		if (*kv->value) {
			s += sprintf (s, "%c%s=%s", (kv == hr->param) ? '?' : '&', kv->key, kv->value);
		} else {
			s += sprintf (s, "%c%s", (kv == hr->param) ? '?' : '&', kv->key);
		}
		kv = kv->next;
	}
#else
	httpd_debug_keyValue(4, hr->param);
	log_msg (LOG_HTTPD, "\t--- headers: ---\n");
	httpd_debug_keyValue(4, hr->headers);
#endif
	log_msg (LOG_HTTPD, "%s\n", tmp);
}

static void httpd (void *pvParameters)
{
	int sock, rc;
	size_t len;
	struct http_request *hr = NULL;
	uint8_t buf[512], *s;
	const char *p, *crnl;
	bool end = false;

	sock = (int) pvParameters;

	log_msg (LOG_HTTPD, "%s() started\n",  __func__);

	s = buf;
	while (!end) {
//		log_msg (LOG_HTTPD, "%s(): current len = %d s=%p\n", __func__, s - buf, s);
		rc = lwip_recv(sock, s, sizeof(buf) - (s - buf) - 1, 0);
		if (rc <= 0) break;	// closed by the other end or error

		// update len, s (the end-pointer) and terminate the receive buffer as a string
		len = (s - buf) + rc;
		s = &buf[len];
		*s = 0;
		log_msg (LOG_HTTPD, "%s(%p FD=%d): received %d bytes => len = %d\n", __func__, xTaskGetCurrentTaskHandle(), sock, rc, s - buf);
//		vTaskDelay(20);

		p = (const char *) buf;
		while ((crnl = strstr (p, CRNL)) != NULL) {
//			log_msg (LOG_HTTPD, "%s(): CRNL @ offset %d\n", __func__, (uint8_t *) crnl - buf);
//			vTaskDelay(20);
			if (!hr) {		// first line, MUST fit in the first packet received
				httpd_parse_request(&hr, p);
				if (!hr) {
					log_error ("%s(): cannot interpret header '%s'\n", __func__, buf);
					break;
				}
			} else {
				if (p == crnl) {    // we have an empty line, so start serving the request!
					if (hr) {		// there is a request, start with replaying the request header as debug output
#if 0	/* DEBUG output is not needed anymore */
						struct key_value *h;

						log_msg (LOG_HTTPD, "%s(): %s %s %s\n",  __func__, (hr->request == GET) ? "GET" : "POST", hr->uri, hr->version);
						h = hr->headers;
						while (h) {
							log_msg (LOG_HTTPD, "%s(): %s: %s\n",  __func__, h->key, h->value);
							h = h->next;
						}
						log_msg (LOG_HTTPD, "%s(): serve request ...\n", __func__);
#endif

						httpd_debug_request(hr);
						while (*p == '\r' || *p == '\n') p++;
						httpd_serve_request(sock, hr, p, s - (uint8_t *) p);
					} else {
						log_error ("%s(): illegal request ...\n", __func__);
					}
					end = true;
					break;
				}

				httpd_parse_header(&hr->headers, p);
			}
			p = crnl + 2;
		}

		// p now points to the part that is not yet interpreted, move it to front of buffer and try to read more ...
		if ((uint8_t *) p > buf) {
			len -= (uint8_t *) p - buf;	    // subtract the length of the interpreted parts
			if (len > 0) memmove(buf, p, len);
			s = &buf[len];
		}
	}

	httpd_free_request(hr);
	if ((rc = lwip_close (sock)) != 0) {
		log_error ("%s(%p FD=%d): close failed with rc=%d\n", __func__, xTaskGetCurrentTaskHandle(), sock, rc);
	}
	log_msg (LOG_HTTPD, "%s(%p FD=%d): finished\n",  __func__, xTaskGetCurrentTaskHandle(), sock);
	vTaskDelete(NULL);
}

int httpd_start (void)
{
//	log_enable(LOG_HTTPD);
	return tcpsrv_startserver(80, httpd, HTTPD_STACK, HTTPD_PRIO);
}

/**
 * @}
 */
