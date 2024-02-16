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
#include "rb2.h"
#include "semphr.h"
#include "timers.h"
#include "lwip/sockets.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/apps/mdns.h"

/**
 * Dieses Modul ermöglicht das Logging auf einen UDP-Multicast Port.
 * Unter Linux kann die Ausgabe mit folgendem Kommando mitgelesen
 * werden (p1p1 ist der Name des Netzwerkadapters):
 *    socat -u UDP-RECV:21928,ip-add-membership=225.0.0.37:p1p1 STDOUT
 * 
 * Unter Windows geht das mit CygWin im Prinzip genauso. Als Bezeichnung
 * des Netzwerkadapters ist allerdings eine Zahl (Interface-Index) zu
 * verwenden. Diesen Index kann man mittels des Kommandos "route print"
 * aus der Aufstellung der Netzwerkadapter ganz oben in der Ausgabe
 * des Kommandos ablesen.
 * socat -u UDP-RECV:21928,ip-add-membership=225.0.0.37:17 STDOUT
 */

#define LOGBUFFER_SIZE				(8 * 1024)
#define LOG_DESTINATION_IP			PP_HTONL(LWIP_MAKEU32(225, 0, 0, 37))
#ifdef BIDIB_SNIFFER
#define LOG_DESTINATION_PORT		PP_HTONS(21930)
#else
#define LOG_DESTINATION_PORT		PP_HTONS(21928)
#endif
#define MAX_LOG_PACKETSIZE			512

static volatile int sock = -1;
static char logbuffer[LOGBUFFER_SIZE];
static volatile char *head, *tail;
static SemaphoreHandle_t mutex;
static TaskHandle_t SenderTask;

static int dbg_open_socket (void)
{
	int s = -1;
	unsigned long val;

	if (sock >= 0) return sock;	// socket already open

	if (rt.en && netif_is_link_up(rt.en) && netif_is_up(rt.en)) {
		s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (s >= 0) {
			val = 1;
			lwip_ioctl(s, FIONBIO, &val);
		}
		sock = s;
	}
	return sock;
}

static void dbg_sendbuffer (int s, char *buf, size_t len)
{
	struct sockaddr_in xDestinationAddress;

	if (s < 0) return;	// buffer is lost!

	xDestinationAddress.sin_family = AF_INET;
	xDestinationAddress.sin_len = sizeof(xDestinationAddress);
	xDestinationAddress.sin_addr.s_addr = LOG_DESTINATION_IP;
	xDestinationAddress.sin_port = LOG_DESTINATION_PORT;
	sendto(s, buf, len, 0, (struct sockaddr *) &xDestinationAddress, sizeof(xDestinationAddress));
}

static void dbg_message_sender(void *pvParameter)
{
	int s;
	size_t len;

	(void) pvParameter;

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (rt.en && rt.en->ip_addr.addr != 0 && mutex_lock(&mutex, 500, __func__)) {
			if (netif_is_up(rt.en) && netif_is_link_up(rt.en)) {
				if ((s = dbg_open_socket()) >= 0) {
					while (tail != head) {
						if (tail > head) {
							len = sizeof(logbuffer) - (tail - logbuffer);
						} else {
							len = head - tail;
						}
						if (len > MAX_LOG_PACKETSIZE) len = MAX_LOG_PACKETSIZE;
						dbg_sendbuffer(sock, (char *) tail, len);
						tail += len;
						if (tail >= logbuffer + sizeof(logbuffer)) tail = logbuffer;	// wrap around
					}
				}
			} else {
				if (sock >= 0) {
					closesocket(sock);
					sock = -1;
				}
			}
			mutex_unlock(&mutex);
		}
	}
}

static void dbg_putChar2Buffer (char c)
{
	if (!head || !tail) head = tail = logbuffer;

	*head++ = c;
	if (head >= logbuffer + sizeof(logbuffer)) head = logbuffer;		// wrap around
	if (head == tail) {
		if (++tail >= logbuffer + sizeof(logbuffer)) tail = logbuffer;	// wrap around
		while (*tail != '\n') {
			if (++tail >= logbuffer + sizeof(logbuffer)) tail = logbuffer;	// wrap around
			if (tail == head) break;    // the buffer is now empty again (one line with full buffer size????)
		}
	}
}

static void dbg_putString2Buffer (const char *s)
{
	while (*s) {
		dbg_putChar2Buffer(*s++);
	}
}

static void irqdbg_timer (TimerHandle_t t)
{
	(void) t;
	dbg_write(NULL, 0);
}

void dbg_init(void)
{
	TimerHandle_t t;

	if (SenderTask == NULL) {
		xTaskCreate (dbg_message_sender, "DBGsend", configMINIMAL_STACK_SIZE * 4, NULL, 4, &SenderTask);
		t = xTimerCreate("IRQDBG Output", 10, pdTRUE, NULL, irqdbg_timer);
		xTimerStart(t, 10);
	}
}

#if 1
static char irqbuf[1024];
static char *irqbuf_filled;

void irqdbg_printf (const char *fmt, ...)
{
	va_list ap;
	int len;

	if (!fmt) return;
	if (!irqbuf_filled) irqbuf_filled = irqbuf;

	va_start (ap, fmt);
	len = vsnprintf(irqbuf_filled, sizeof(irqbuf) - (irqbuf_filled - irqbuf), fmt, ap);
	va_end (ap);
	irqbuf_filled += len;
}
#endif

void dbg_puts (const char *str)
{
	if (!str) return;

	if (!mutex_lock (&mutex, 10, __func__)) return;
	dbg_putString2Buffer(str);
	mutex_unlock(&mutex);
	if (SenderTask) xTaskNotifyGive (SenderTask);
}

void dbg_write (const char *s, int len)
{
	char *p, *end;
	bool output = false;

	if (!mutex_lock (&mutex, 10, __func__)) return;

	if ((end = irqbuf_filled) != NULL) {
		p = irqbuf;
		irqbuf_filled = NULL;
		output = true;
		while (p < end) dbg_putChar2Buffer(*p++);
	}

	if (s && len > 0) {
		while (len > 0) {
			dbg_putChar2Buffer(*s++);
			len--;
		}
		output = true;
	}
	mutex_unlock(&mutex);
	if (output && SenderTask) xTaskNotifyGive (SenderTask);
}

void dbg_putc (const char c)
{
	if (!mutex_lock (&mutex, 10, __func__)) return;
	dbg_putChar2Buffer(c);
	mutex_unlock(&mutex);
	if (SenderTask) xTaskNotifyGive (SenderTask);
}

void dbg_link_cb (struct netif *netif)
{
	if (!netif) return;

	//    if (netif_is_link_up(netif)) LED_ETH_ON();
	//    else LED_ETH_OFF();

	if (SenderTask) xTaskNotifyGive (SenderTask);
}

void dbg_status_cb (struct netif *netif)
{
	if (!netif) return;

	if (netif_is_up(netif)) {
		printf ("%s() IP-Addr = %s\n", __func__, ip4addr_ntoa(&netif->ip_addr));
		mdns_resp_announce(netif);
//		rgb_color(0x70, 0x10, 0x10);
	} else {
		printf ("%s() link down\n", __func__);
	}
	if (SenderTask) xTaskNotifyGive (SenderTask);
}
