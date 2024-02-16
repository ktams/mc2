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
#include "rb2.h"
#include "lwip/sockets.h"

struct tcp_args {
	void (*accept_func)(void *);	///< an accept function (FreeRTOS thread routine) that should be started on a connection event
	int			stacksize;			///< the requested stack size for the accept-thread
	int			prio;				///< the priority for the created thread
	uint16_t	port;				///< the port that the server should be listening on
};

/**
 * Check if the given socket is still connected (as far as the system can tell).
 * This is a quick check, if the remote side has gracefully shut down the socket.
 * If the connection is disrupted by any other means, it can only be detected by
 * the stack if a (long) timeout has gone.
 *
 * @param s		the socket to check for remote shutdown
 * @return		<i>true</i> if socket appears alive, <i>false</i> otherwise
 */
bool tcp_checkSocket (int s)
{
	return (lwip_recv(s, NULL, 0, MSG_DONTWAIT) != 0);
}

int tcp_listenSocket (int port, int timeout)
{
	struct sockaddr_in BindAddress;
	struct timeval tv;
	int rc, s;

	/* Attempt to open the socket. */
	s = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	/* Check the socket was created. */
	configASSERT (s >= 0);

	if (timeout > 0) {
		/* Set a time out so accept() will just wait for a connection. */
		tv.tv_sec = timeout;
		tv.tv_usec = 0;
		if (lwip_setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv))) {
			fprintf (stderr, "%s(%d): cannot set timeout on socket (errno = %d)\n", __func__, port, errno);
		}
	}

	/* Set the listening port */
	memset (&BindAddress, 0, sizeof(BindAddress));
	BindAddress.sin_family = AF_INET;
	BindAddress.sin_len = sizeof(BindAddress);
	BindAddress.sin_port = lwip_htons(port);

	/* Bind the socket to that port. */
	while ((rc = lwip_bind (s, (struct sockaddr *)&BindAddress, sizeof(BindAddress))) != 0) {
		fprintf (stderr, "%s(%d): bind failed with errno=%d\n", __func__, port, errno);
		vTaskDelay(5000);
	}

	/* Set the socket into a listening state so it can accept connections.
    The maximum number of simultaneous connections is limited to 20. */
	if ((rc = lwip_listen (s, 20)) != 0) {
		fprintf (stderr, "%s(%d): listen failed with rc=%d\n", __func__, port, rc);
	}

	return s;
}

/**
 * Opens a listening TCP socket and waits for connections. This is a tiny
 * helper function which implements all steps of a TCP server up tp the
 * point where a client connects successfully. It then starts the handler
 * function as a new thread with the accepted socket as a parameter and
 * the stack space specified in the arguments struct (pvParameters).
 * 
 * @param pvParameters	    a pointer to a struct tcp_args which contains all necessary information for the server
 */
static void tcpsrv_serverloop (void *pvParameters)
{
	struct sockaddr_in xClient;
	struct tcp_args *args;
	int xListeningSocket, xConnectedSocket;
	socklen_t xSize = sizeof (xClient);
	BaseType_t rc;
	TaskHandle_t task;
//	char ipaddr[32];

	args = (struct tcp_args *) pvParameters;

	xListeningSocket = tcp_listenSocket(args->port, 10);

	log_msg (LOG_INFO, "%s(%d): waiting for connections\n",  __func__, args->port);
	for (;;) {
		/* Wait for incoming connections. */
		xConnectedSocket = lwip_accept(xListeningSocket, (struct sockaddr *)&xClient, &xSize);
		if (xConnectedSocket >= 0) {
//			inet_ntoa_r(xClient.sin_addr, ipaddr, sizeof(ipaddr));
//			log_msg (LOG_INFO, "%s(%d): got a connect from %s:%d\n",  __func__, args->port, ipaddr, lwip_ntohs(xClient.sin_port));

			/* Spawn a RTOS task to handle the connection. */
			xTaskCreate(args->accept_func, "TCP_CLNT", args->stacksize, (void *) xConnectedSocket, args->prio, &task);
//			log_msg (LOG_INFO, "%s(%d) task created %p\n", __func__, args->port, task);
		} else {
//			printf ("%s(): timeout waiting for connection\n",  __func__);
		}
	}

	// currently there is no run-control, so the serverloop will never quit and therefor this
	// part will never be reached ...
	log_msg (LOG_INFO, "%s(%d): shutting down server socket\n",  __func__, args->port);
	lwip_shutdown(xListeningSocket, SHUT_RDWR);
	if ((rc = lwip_close (xListeningSocket)) != 0) {
		log_error ("%s(%d): close failed with rc=%ld\n", __func__, args->port, rc);
	}
	log_msg (LOG_INFO, "%s(%d): finished\n",  __func__, args->port);
	free (args);
	vTaskDelete(NULL);
}

int tcpsrv_startserver (uint16_t port, void (*accept_func)(void *), int stacksize, int prio)
{
	struct tcp_args *args;

	if ((args = malloc(sizeof(*args))) == NULL) {
		return -1;
	}

	args->port = port;
	args->accept_func = accept_func;
	args->stacksize = stacksize;
	args->prio = prio;
	return xTaskCreate(tcpsrv_serverloop, "TCP_SRV", 512, args, 1, NULL);
}
