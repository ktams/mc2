/**
 * @file    httpd.h
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

#ifndef __HTTPD_H__
#define __HTTPD_H__

/**
 * @defgroup HTTPD HTTP-Server
 *
 * The HTTP-Server is a simple daemon that delivers files from builtin NAND-Flash
 * file system. The root of the documents is fixed at "/html". If no file or just
 * the root "/" is specified as request, then "/index.html" is dellivered, which
 * must be available under "/html" directory.
 *
 * Active contents is handled by defined functions acting upon a fixed list of URLs.
 * Currently, all these URLs reside in a virtual directory "/cgi". There is of course
 * no provision is made, to support real CGI handling with script languages like PHP.
 * See @ref vFuncT for the definition of this structure.
 *
 * Currently, there are two supported CGI-functions:
 * <ul>
 * <li> /cgi/action.html: give commands and receive one-time information on request. See @ref CGI_ACTION
 * <li> /cgi/events: register for reception of various events. See @ref CGI_EVENTS
 * </ul>
 *
 * Usually, answers containing data are formatted JSON objects (the string representation of it),
 * so you can easyly convert this data to a JavaScript object.
 *
 * To send a request from inside JavaScript and read the result you can use a construction like
 * the following code snipped (request a loco specified by the variable *locoID* to take control
 * and parse the current status from the answer):
 *
 * <pre>
 *     var req = new XMLHttpRequest();
 *     req.overrideMimeType("application/json");
 *     req.open("GET", "/cgi/action.html?lok=" + locoID + "&cmd=get");
 *     req.onload = function() {
 *         HandleEvent(JSON.parse(this.responseText));
 *     };
 *     req.send();
 *
 *	   // this function also handles the events coming from the server using server-sent events (SSE)
 *     function HandleEvent (obj) {
 *		   if (typeof(obj.funcs) !== 'undefined') {		// check, if 'funcs' is defined in answer
 *		   	   // interpret the obj.funcs
 *         }
 *		   if (typeof(obj.speed) !== 'undefined') {		// check, if 'speed' is defined in answer
 *		   	   // interpret the obj.speed and probably show it on an output field
 *         }
 *		   if (typeof(obj.state) !== 'undefined') {		// check, if 'state' is defined in answer
 *		   	   // interpret the obj.state system state (STOP, GO, ...)
 *         }
 *         ... more settings may have been sent ...
 *     }
 * </pre>
 *
 * @{
 */

#define CRNL					"\r\n"						///< a line terminator as requested by HTML standard
#define HTML_404				"/404.html"					///< the filename to send, when requested file was not found
#define SERVER_STRING			"FreeRTOS v10.2.1/lwIP v2.1.2"
#define FILE_OK					"HTTP/1.1 200 Ok\r\n"
#define RESOURCE_CREATED		"HTTP/1.1 201 Created\r\n"
#define BAD_REQUEST				"HTTP/1.1 400 Bad Request\r\n"
#define FILE_NOT_FOUND			"HTTP/1.1 404 Not Found\r\n"
#define METHOD_NOT_ALLOWED		"HTTP/1.1 405 Method Not Allowed\r\n"
#define NOT_ACCEPTABLE			"HTTP/1.1 406 Not Acceptable\r\n"
#define CONFLICT				"HTTP/1.1 409 Conflict\r\n"
#define LENGTH_REQUIRED			"HTTP/1.1 411 Length Required\r\n"
#define PRECONDITION_FAILED		"HTTP/1.1 412 Precondition Failed\r\n"
#define INTERNAL_SERVER_ERROR	"HTTP/1.1 500 Internal Server Error\r\n"
#define CONTENT_TEXT			"text/plain"				///< a standard format for everything that is somehow unknown
#define CONTENT_EVENT			"text/event-stream"			///< special type for streaming events to the client
#define CONTENT_JSON			"application/json"			///< type for answers containing variables
#define HEADER_END				CRNL

#define WWW_DIR				"/html/"

enum req {
	UNKNOWN_REQ = 0,	///< marks an invalid / unknown request type
	GET,				///< a GET request
	HEAD,				///< a HEAD request
	POST,				///< a POST request
	PUT,				///< a PUT request
	DELETE,				///< a DELETE request
	CONNECT,			///< a CONNECT request
	OPTIONS,			///< a OPTIONS request
	TRACE,				///< a TRACE request
};

struct http_request {
	struct key_value	*headers;	///< the linked list of request headers
	char				*uri;		///< the requested URI
	struct key_value	*param;		///< the linked list of parameters in URI (everything behind '?')
	char				*version;	///< points to the version of the request (i.e. HTTP/1.1)
	enum req			 request;	///< the type of request to serve
};



/*
 * prototypes WEB/cgi.c
 */
bool cgi_check_request (int sock, struct http_request *hr, const char *rest, int sz);

/*
 * (local) prototypes WEB/httpd.c
 */
void httpd_header (int sock, const char *status, struct key_value *hdrs);
void httpd_free_request (struct http_request *hr);
//int httpd_sendbuf (int sock, uint8_t *buf, int len);
//int httpd_sendstring (int sock, const char *str);
void httpd_serve_file (int sock, char *uri, struct key_value *hdrs);

/**
 * @}
 */
#endif /* __HTTPD_H__ */
