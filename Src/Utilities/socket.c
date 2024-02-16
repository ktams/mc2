/*
 * socket.c
 *
 *  Created on: 20.04.2022
 *      Author: Andi
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "rb2.h"
#include "lwip/sockets.h"

int socket_senddata (int sock, const void *data, int len)
{
	const void *p;
	int sent;

	if ((p = data) == NULL) return 0;

	while (len > 0) {
		sent = lwip_write(sock, p, len);
		if (sent <= 0) break;
		p += sent;
		len -= sent;
	}
	return p - data;
}

int socket_sendstring (int sock, const char *str)
{
	return socket_senddata(sock, str, strlen(str));
}

int socket_printf (int sock, const char *fmt, ...)
{
	va_list ap;
	char *tmp;
	int len;

	if (!fmt || !*fmt) return 0;

	tmp = tmp1k();					// fetches one of the 1024-byte buffers
	va_start (ap, fmt);
	len = vsnprintf(tmp, 1024, fmt, ap);
	va_end (ap);

	if (len > 1023) len = 1023;		// output was truncated, but string was null terminated, so we should send out first 1023 bytes
	return socket_senddata(sock, tmp, len);
}
