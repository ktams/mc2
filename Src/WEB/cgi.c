/**
 * @file    cgi.c
 * @author  Andi
 * @date	30.12.2019
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
 * @page CGI_ACTION HTTPD: Using /cgi/action.html
 *
 * This is a virtual page that you can retrieve and trigger some actions by supplying parameters.
 * Parameters are supplied in the usual way by adding a question mark ('?') after the page call
 * and delimiting the different "param=value" pairs by ampersand character ('&amp;').
 *
 * The following parameters are supported:
 * <ul>
 * <li> <code>cmd</code>: a command (may be related to a loco, if given)
 * <li> <code>lok</code>: the ID (number) of the loco the whole request is meant for (if no loco is given,
 * 		the command will be system command).
 * <li> <code>speed</code>: set a new speed and direction of a loco - only valid when *lok* is supplied
 * <li> <code>fu</code>: set or clear a function bit on a loco - only valid when *lok* is supplied
 * </ul>
 *
 * The list of valid commands (*cmd*) is:
 * <ul>
 * <li> <code>go</code>: Set the operational mode to GO
 * <li> <code>stop</code>: Set the operational mode to STOP
 * <li> <code>get</code>: Call the loco (get it into the refresh buffer of signal generation) and return
 * 		some information about it's current state and settings (needs a loco number supplied with the *lok* parameter)
 * </ul>
 *
 * <h2>Starting and stopping booster output (i.e. setting system operation mode)</h2>
 *
 * You can switch the state of the system by calling
 * <pre>
 * /cgi/action.html?cmd=go		*or*
 * /cgi/action.html?cmd=stop
 * </pre>
 * In this case you will receive an empty answer, but the system will switch it's state accordingly.<br>
 * By the way: this will trigger an event, that you can receive. For events see @ref CGI_EVENTS
 *
 * <h2>Requesting a loco to take control</h2>
 *
 * To take control of a loco and get the current information about it's state (speed, direction, functions, ...)
 * you call the action.html page with the loco number and the command "get":
 * <pre>
 * /cgi/action.html?lok=123&cmd=get
 * </pre>
 *
 * The result is delivered as a JSON object containig the following information:
 * <ul>
 * <li> <code>lok</code>: the loco number (should match your request)
 * <li> <code>speed</code>: the current speed and direction (see below)
 * <li> <code>funcs</code>:	an array of four integers (unsigned 32 bit) containing the states of the functions F0 to F127
 * <li> <code>state</code>: the current system state as string (GO, STOP, HALT, SHORT)
 * </ul>
 * More information will be added in the near future. At least you should be informed about the decoder
 * protocol, number of controllable functions, number of commandable speeds and a name.
 *
 * <h2>Changing speed and functions</h2>
 *
 * These actions involve a call to <code>/cgi/action.html</code> that includes the *lok* parameter
 * and one of the *fu* or *speed* parameters.
 *
 * The values supplied to the *fu* and the *speed* parameter have special format restrictions:
 *
 * <h3>The speed parameter and formatted answer</h3>
 *
 * Speeds should include a direction (the special case of MM1 not having a specific direction
 * is not yet implemented). So the first letter of the speed string must be an 'R' for reverse
 * direction or an 'F' for forward direction.
 *
 * Example:<br>
 * A forward speed of 22 must be coded as <code>speed=F22</code><br>
 * A reverse speed of 0 (loco stopped, but headlight show a reverse direction) is coded as <code>speed=R0</code>
 *
 * When interpreting the answer of the *get*-command you will find the 'speed' member of the
 * returned object having this coding, too. Let's imagine you create a JavaScript object from the
 * answer using obj=JSON.parse(answerstring), you will have an object of the name 'obj' that will
 * have a member called 'speed' (as shown above). If the loco had a speed of reverse 8, the contents
 * of obj.speed will read <code>R8</code>. So all you need to do is check the first character for
 * 'F' or 'R' to decode the direction and then integer-parse the rest of the string to the the
 * speed step:
 *
 * <pre>
 *    forward = obj.speed.charAt(0) == 'F';					// setting variable forward to 0 or 1
 *    speedstep = parseInt(obj.speed.substring(1), 10);		// variable speedstep now contains the speed information
 * </pre>
 *
 * <h3>The function parameter</h3>
 *
 * Functions (currently) can only be controlled with a single function switch per call. The *fu*
 * parameter must include a specification, if the the function is to be switched ON or OFF and
 * the function number itself.
 *
 * The first character of this property should be the state encoded as '1' to activate the function
 * or '0' to deactivate it. The rest of the setting simply represents the number of the function
 * to control
 *
 * Example:<br>
 * To set the function 15 to ON send <code>fu=115</code><br>
 * To set the function 3 to OFF send <code>fu=03</code>
 */

/**
 * @page CGI_EVENTS HTTPD: Using /cgi/events
 *
 * Requesting simple files or even sending commands using parameters like explained in @ref CGI_ACTION
 * is a simple but powerfull tool. But if you want a web based HTML-application to behave really
 * responsive, you want to be informed of what is going on the server side (i.e. the RB2).
 *
 * In history of web application several attempts where made to get this done. One of these early
 * tries were so called *long calls*. That is simply a request that is not answered by the server
 * until the requested event happens. The drawback of this approach is, that you will see the web
 * browser waiting and the JavaScript will need to immediately re-request that hanging call to not
 * miss any new events.
 *
 * A quiet heavyweighted construction are the HTML5 Web-Sockets. They are used with AJAX and are
 * really powerful. But the server side implementation is a burden we tried to avoid.
 *
 * As we only want to implement events that are sent from the server to the client browser, the
 * HTML5 Server-Sent-Events (SSE) are the perfect vehicle for that task. They are implemented in
 * all modern browsers even on smartphones. Only M$ is still not able to support them (but I
 * heard rumors, that this could possibly change in the near future).
 *
 * So, how does SSE work? You simply make a standard request and get an answer but no content.
 * The socket your browser opened to send the request stays open and whenever the server wants
 * to send an event, he sends some formatted text lines terminated by lineendings. Lineendings
 * may be coded as CR, NL or CR+NL, the textual part of the message(s) is always coded in UTF-8.
 * A message ends, when an empty line is transmitted (or you can say, the last line of the
 * message is terminated by two consecutive lineendings).
 *
 * Neither the client browser nor the server will close the connection, so the server may send
 * the next event any time he is pleased to. Closing the socket should be done by the client
 * browser if it is no longer interested in further event notifications. This will automatically
 * happen if the browser is closed or should be done programmatically in the 'beforeunload' handler
 * of the current window.
 *
 * The following JavaScript code demonstrates how that can be used. In this example we register
 * our event handler to be informed about events regarding a loco with the number *locoID*,
 * changes in system state and the current consumption of the builtin booster:
 *
 * <pre>
 *     var evt = new EventSource("/cgi/events?lok=" + locoID + "&status&current");
 *     evt.onerror = function(err) {
 *         console.error("Events:", err)
 *     }
 *     evt.onopen = function() {
 *         console.log("Events OPENED");
 *     };
 *     evt.onmessage = function(e) {
 *         console.log("Event: " + e.data);
 *         HandleEvent(JSON.parse(e.data));
 *     };
 *     window.addEventListener('beforeunload', function(e) {
 *         evt.close();
 *     });
 *
 *     function HandleEvent (obj) {
 *         ... (filled in later)
 *     }
 * </pre>
 *
 * The *onerror* handler could be used to inform the user with a popup, that either the connection
 * failed or that the browser doesn't support server-sent events. The *onopen* handler is just a
 * debug message and not needed in a real world application, it can be helpfull in the beginning though.
 * The event listener for the window event *beforeunload* takes care of closing the event socket when
 * the page is left. This should be done to signal the server, that it should not send further events
 * on this channel.
 *
 * The most important part of that construction is setting a handler for the incoming messages. The
 * *onmessage* handler of the event source object is in our case a function that logs the received text
 * (again just as debugging aid) and then calls a specific handler. We already could have written the
 * handler code here but it makes sense to write an extra function that also can handle results from
 * the <code>/cgi/action.html</code> as can be seen in the example in @ref HTTPD.
 *
 * For the details of the contents of the text lines making up an event, a lot of information can be
 * found on the web (for example in Mozillas development network MDN). Currently, we only use the data
 * field for events and define a single JSON object with this text. The JSON object is then translated
 * to a JavaScript object using the *JSON.parse()* method. The resulting object may contain several
 * fields depending on the event the server wants to describe.
 *
 * To be prepared to receive events, where only a subset of the possible members is set, you should
 * check each member to not be 'undefined' before you act on it. This is accomplished by using the
 * *typeof*-operator of the JavaScript language. Let's take a closer look at the HandleEvent()
 * function:
 *
 * <pre>
 *     function HandleEvent (obj) {
 *         if (typeof(obj.state) !== 'undefined') {		// check that the state member exists
 *             alert ('New system state is ' + obj.state);
 *         }
 *     }
 * </pre>
 *
 * <h2>The events to register for</h2>
 *
 * To register for events, you should call the <code>/cgi/events</code> script with parameters describing
 * the events you want to register for. You also could call this several times, each time with a different
 * event type (or even with the same ones). But this will occupy a TCP socket for each call during the whole
 * lifetime of that page. So please be kind and merge all event requests in a single call and have an event
 * handler that is able to distinguish the meaning of the event by carefully checking the delivered members
 * as shown above.
 *
 * In the following paragraphs I will explain what registered event will give you what information and how to
 * specify it. As of writing this (January 2020) the list of possible events and the information sent by the
 * server is still somewhat limited.
 *
 * An additional parameter can be supplied to set a timeout in milli seconds using <code>timeout=10000</code>.
 * A minimum value of 1000 (i.e. 1s) will be enforced, if the timeout is not specified as 0 (which means no
 * timeout at all). The standard timeout, if no timeout parameter is supplied, is 60s. The timeout will (currently)
 * not trigger an event, but check the TCP socket for beeing closed. In this case, the socket can be closed on
 * the server side, too, and the resources of the event callback be released. A timeout can be useful if it is not
 * guaranteed, that the registered event will ever fire - in that case the resources on the server side would
 * be blocked for ever. If you have no special needs for a timeout, just don't supply this parameter and leave
 * it to the server to handle this for you.
 *
 * To give an example, I will show you the request which should be sent to register for all event types.
 * For loco speed and function events we will register for those of loco #123:
 *
 * <pre>
 *    /cgi/events?lok=123&status&s88&current
 * </pre>
 *
 * <h3>LOK event</h3>
 *
 * Currently, the *lok* event is the only one requiring a parameter: the ID of the loco you want events for.
 * These events currently consist of speed and function events. With each event, the following object members
 * are transmitted:
 * <ul>
 * <li> <code>lok</code>: the loco id this event relates to (present for all speed and function events, will
 * 		match the loco number you registered this handler for)
 * <li> <code>speed</code>: the new speed code and direction if the event is a speed event. The coding is the
 * 		same as outlined in @ref CGI_ACTION.
 * <li> <code>funcs</code>: an array of (currently) four integers representing the four 32 bit unsigned values
 * 		containing the functions F0 - F127 if the event is a function event.
 * </ul>
 *
 * <h3>STATUS event</h3>
 *
 * The *status* event needs no value to be registered and can be coded with or without the equal sign in the URI.
 *
 * If the status of the system changes you will receive an event with only one member:
 *
 * <code>state</code>: a string with the content "STOP", "GO", "HALT" or "SHORT" reflecting the new system state.
 *
 * <h3>s88 event</h3>
 *
 * (Not yet implemented)
 *
 * <h3>Current event</h3>
 *
 * This event will report a change in track current supplied by the internal booster. It is limited to send
 * updates earliest 500ms after the last event and will repeat its message after 5s even if the current didn't
 * change. The current is scaled to 100mA (i.e.0,1A). For example the current 3.7A will be reported as a value
 * of 37. State changes to <b>STOP</b> or <b>GO</b> will send an isolated event showing 0A to reset a corresponding
 * display widget.
 *
 * This event will have a single member supplied:
 *
 * <code>current</code>: integer number of 0.1A units currently supplied to the internal booster output.
 */

/**
 * @ingroup HTTPD
 * @{
 */


#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include "rb2.h"
#include "lwip/sockets.h"
#include "decoder.h"
#include "events.h"
#include "yaffsfs.h"
#include "httpd.h"
#include "json.h"
#include "intelhex.h"
#include "config.h"
#include "bidib.h"
#include "easynet.h"
#include "defaults.h"

#define RX_BUFSIZE		2048				///< size of an allocated buffer for receiving files

struct cgiquery {
	char		*cmd;									///< the command string (case insensitive) from option "cmd"
	int (*func) (int sock, struct http_request *hr);	///< the action function that will be called, may terminate the thread and thus not return at all
};

//static int html_action (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_query (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_command (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_regEvent (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_consist (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_update (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_readfile (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_modeltime (int sock, struct http_request *hr, const char *rest, int sz);
//static int cgi_sound (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_internal (int sock, struct http_request *hr, const char *rest, int sz);
static int cgi_esp (int sock, struct http_request *hr, const char *rest, int sz);

/**
 * A table defining the active components of the HTTP-server.
 *
 * This is simply a list of URIs that map to C-functions that act on the client request.
 * The list is terminated by adding an element that contains NULL for bothe the URI
 * and the function.
 */
static const struct vFuncT {
	const char	*path;						///< the path that is compared to the URI request from the client
	enum req	request;					///< the possible request
	int (* const func) (int sock, struct http_request *hr, const char *rest, int sz);		///< the action function that is executed, if the path matches
} vFuncs[] = {
//	{ "/cgi/action.html", GET, html_action },	///< command actions and receive results from simple queries (deprecated)
	{ "/cgi/command", GET, cgi_command },		///< command actions, always returns an empty result (only "HTTP/1.1 200 Ok")
	{ "/cgi/query", GET, cgi_query },			///< command queries and receive results as JSON object
	{ "/cgi/events", GET, cgi_regEvent },		///< describe to events (Server-Sent Events, SSE)
	{ "/cgi/consist", GET, cgi_consist },		///< handle consist management
	{ "/cgi/update", POST, cgi_update },		///< transfer a file for update purpose (using POST method)
	{ "/cgi/update", PUT, cgi_update },			///< transfer a file for update purpose (using PUT method)
	{ "/cgi/store", POST, cgi_update },			///< transfer a file for general purpose (using POST method)
	{ "/cgi/readfile", GET, cgi_readfile },		///< transfer a file from RB2 to PC for general purpose (using GET method)
	{ "/cgi/modeltime", GET, cgi_modeltime },	///< commands to manipulate modeltime subsystem
//	{ "/cgi/sound", GET, cgi_sound },			///< sound control
	{ "/cgi/internal", GET, cgi_internal },		///< query internal information (mostly for production use)
	{ "/cgi/esp", GET, cgi_esp },				///< commands to the ESP module
	{ NULL, UNKNOWN_REQ, NULL }
};		///< The list of virtual functions


struct cbdata {
	int						sock;	///< the connected socket to send an answer to
	struct http_request		*hr;	///< the original request headers
	int						loco;	///< if events are related to a loco, filter them by this loco id
};

static bool html_finishEventHandler (struct cbdata *cb)
{
	int rc;

	log_msg (LOG_INFO, "%s(): client socket closed\n", __func__);
	if ((rc = lwip_close (cb->sock)) != 0) {
		log_error ("%s(): close failed with rc=%d\n", __func__, rc);
	}

	httpd_free_request(cb->hr);
	free (cb);
	return false;
}

static void html_sendStatus (int sock)
{
	struct key_value *h, *hdrs;

	hdrs = h = kv_add(NULL, "Content-Type", CONTENT_EVENT);
	h = kv_add(h, "Connection", "keep-alive");
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
}

static int cgi_sendJsonValue (int sock, json_valT *val);
static int cgi_sendJsonItem (int sock, json_itmT *itm);

static int cgi_sendJsonValue (int sock, json_valT *val)
{
	char buf[32];
	int rc = 0;
	bool comma = false;

	while (val && rc >= 0) {
		if (comma) rc = lwip_send(sock, ", ", 2, MSG_MORE);
		switch (val->type) {
			case JSON_OBJECT:
				if (rc >= 0) rc = lwip_send (sock, "{ ", 2, MSG_MORE);
				if (rc >= 0) rc = cgi_sendJsonItem(sock, val->itm);
				if (rc >= 0) rc = lwip_send (sock, " }", 2, MSG_MORE);
				break;
			case JSON_ARRAY:
				if (rc >= 0) rc = lwip_send (sock, "[ ", 2, MSG_MORE);
				if (rc >= 0) rc = cgi_sendJsonValue(sock, val->array);
				if (rc >= 0) rc = lwip_send (sock, " ]", 2, MSG_MORE);
				break;
			case JSON_STRING:
				if (rc >= 0) rc = lwip_send (sock, "\"", 1, MSG_MORE);
				if (rc >= 0) rc = lwip_send (sock, val->string, strlen(val->string), MSG_MORE);
				if (rc >= 0) rc = lwip_send (sock, "\"", 1, MSG_MORE);
				break;
			case JSON_INTEGER:
				sprintf (buf, "%d", val->intval);
				if (rc >= 0) rc = lwip_send (sock, buf, strlen(buf), MSG_MORE);
				break;
			case JSON_UNSIGNED:
				sprintf (buf, "%u", val->uintval);
				if (rc >= 0) rc = lwip_send (sock, buf, strlen(buf), MSG_MORE);
				break;
			case JSON_TRUE:
				if (rc >= 0) rc = lwip_send (sock, "true", 4, MSG_MORE);
				break;
			case JSON_FALSE:
				if (rc >= 0) rc = lwip_send (sock, "false", 5, MSG_MORE);
				break;
			case JSON_NULL:
				if (rc >= 0) rc = lwip_send (sock, "null", 4, MSG_MORE);
				break;
			default:
				rc = -1;
				break;
		}
		val = val->next;
		comma = true;
	}
	return rc;
}

static int cgi_sendJsonItem (int sock, json_itmT *itm)
{
	int rc = 0;
	bool comma = false;

	while (itm && rc >= 0) {
		if (comma) rc = lwip_send(sock, ", ", 2, MSG_MORE);
		if (rc >= 0) rc = lwip_send (sock, "\"", 1, MSG_MORE);
		if (rc >= 0) rc = lwip_send (sock, itm->name, strlen(itm->name), MSG_MORE);
		if (rc >= 0) rc = lwip_send (sock, "\": " , 3, MSG_MORE);
		if (rc >= 0) rc = cgi_sendJsonValue(sock, itm->value);
		itm = itm->next;
		comma = true;
	}
	return rc;
}

static int cgi_sendJSON (int sock, json_valT *root)
{
	int rc;

	if (!root || root->type != JSON_OBJECT) return 0;		// we are expecting a JSON_OBJECT as the root element - else don't send anything
	rc = cgi_sendJsonValue(sock, root);
	if (rc >= 0) rc = lwip_send (sock, "\n\n", 2, 0);

	return rc;
}

static int cgi_sendJSONeventdata (int sock, json_valT *root)
{
	int rc;

	if (!root || root->type != JSON_OBJECT) return 0;		// we are expecting a JSON_OBJECT as the root element - else don't send anything
	rc = lwip_send (sock, "data: ", 6, MSG_MORE);
	if (rc >= 0) rc = cgi_sendJSON(sock, root);
	return rc;
}

static int cgi_listBIDIBnodes (json_stackT *jstk, struct bidibnode *bn)
{
	json_valT *val;
	int cnt = 0;
	uint32_t adr;

	while (bn) {
		val = json_addObject(jstk);
		jstk = json_pushObject(jstk, val);
		adr = bidib_getAddress(bn);
		while (adr && ((adr & 0xFF) == 0)) adr >>= 8;	// the web client needs a different coding ...
		json_addUintItem(jstk, "adr", adr);
		json_addIntItem(jstk, "class", bn->uid[0]);
		json_addIntItem(jstk, "xclass", bn->uid[1]);
		json_addIntItem(jstk, "manufacturer", bn->uid[2]);
		json_addUintItem(jstk, "identify", (bn->flags & NODEFLG_IDENTIFY) ? 1 : 0);
		json_addUintItem(jstk, "serial", (bn->uid[3] << 24) | (bn->uid[4] << 16) | (bn->uid[5] << 8) | (bn->uid[6] << 0));
		jstk = json_pop(jstk);
		if (bn->children) cnt += cgi_listBIDIBnodes(jstk, bn->children);
		bn = bn->next;
		cnt++;
	}
	return cnt;
}

static bool cgi_eventHandler (eventT *e, void *prv)
{
	struct cbdata *cb;
	struct sysconf *sc;
	struct fmtconfig *fc;
	struct modeltime *mt;
	struct decoder_reply *msg;
	struct extDevice *dev;
	struct bidibnode *bn;
	struct en_bootProgress *enprogress;
	struct consist *c;
	ldataT *l;
	locoT *ldb;
	turnoutT *t;
	char *s;
	struct s88_status *s88;
	fbeventT *fbevt;
	uint16_t *s88data;
	int rc = 0, i;
	json_valT *root, *val;
	json_itmT *itm;
	json_stackT *jstk;
	enum fmt trnt_deffmt;

//	log_msg (LOG_INFO, "%s()\n", __func__);

	cb = (struct cbdata *) prv;
	l = (ldataT *) e->src;
	ldb = (locoT *) e->src;		// used by EVENT_LOCO_PARAMETER
	if (!tcp_checkSocket(cb->sock)) return html_finishEventHandler(cb);		// check if socket is still alive
	root = NULL;
	jstk = NULL;

	switch (e->ev) {
		case EVENT_SYS_STATUS:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			itm = json_addItem(jstk, "state");
			switch (e->param) {
				case SYSEVENT_STOP:
					itm->value = json_addStringValue(NULL, "STOP");
					break;
				case SYSEVENT_GO:
					itm->value = json_addStringValue(NULL, "GO");
					break;
				case SYSEVENT_HALT:
					itm->value = json_addStringValue(NULL, "HALT");
					break;
				case SYSEVENT_SHORT:
					itm->value = json_addStringValue(NULL, "SHORT");
					break;
				case SYSEVENT_OVERTEMP:
					itm->value = json_addStringValue(NULL, "HOT");
					break;
				default:		// others are ignored (just in case future expansion to more system states)
					itm->value = json_addNull(NULL);
					break;
			}
//			json_debug(root);
			break;
		case EVENT_LOCO_SPEED:
			if (e->param != cb->loco || !l) return true;		// we only work on events for this loco (ignore others and continue)
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "lok", e->param);
			json_addFormatStringItem(jstk, "speed", "%c%d", (l->speed & 0x80) ? 'F' : 'R', l->speed & 0x7F);
			break;
		case EVENT_LOCO_FUNCTION:
			if (e->param != cb->loco || !l) return true;		// we only work on events for this loco (ignore others and continue)
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "lok", e->param);
			itm = json_addArrayItem(jstk, "funcs");
			jstk = json_pushArray(jstk, itm);
			json_addUintValue(jstk, l->funcs[0]);
			json_addUintValue(jstk, l->funcs[1]);
			json_addUintValue(jstk, l->funcs[2]);
			json_addUintValue(jstk, l->funcs[3]);
			break;
		case EVENT_LOCO_PARAMETER:
			if (e->param != cb->loco || !ldb) return true;		// we only work on events for this loco (ignore others and continue)
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "lok", e->param);
			json_addStringItem(jstk, "name", ldb->name);
			json_addIntItem(jstk, "maxfunc", ldb->maxfunc);
			break;
		case EVENT_NEWLOCO:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			if (e->param > 0) {
				json_addIntItem(jstk, "NewLoco", e->param);
			} else {
				json_addIntItem(jstk, "PurgeLoco", -e->param);
			}
			break;
		case EVENT_LOCO_DB:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			itm = json_addArrayItem(jstk, "addresses");
			jstk = json_pushArray(jstk, itm);
			i = 0;
			while ((ldb = db_lookupLocoSorted(i++)) != NULL) {
				json_addIntValue(jstk, ldb->adr);
			}
			break;
		case EVENT_TURNOUT:
			if ((t = (turnoutT *) e->src) != NULL) {
				if (!t->dir && t->on) s = "G";
				else if (t->dir && !t->on) s = "r";
				else if (t->dir && t->on) s = "R";
				else s = "g";	// at least a default
				root = json_addObject(NULL);	// create root object
				jstk = json_pushObject(NULL, root);
				json_addIntItem(jstk, "trnt", t->adr);
				json_addStringItem(jstk, "stat", s);
			}
			break;
		case EVENT_FEEDBACK:
			s88 = (struct s88_status *) e->src;
			s88data = s88->sum;
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "s88module", cnf_getconfig()->s88Modules);
			json_addIntItem(jstk, "canmodule", cnf_getconfig()->canModules);
			json_addIntItem(jstk, "lnetmodule", cnf_getconfig()->lnetModules);
			json_addIntItem(jstk, "s88frequency", s88_getFrequency());
			itm = json_addArrayItem(jstk, "s88");
			jstk = json_pushArray(jstk, itm);
			for (i = 0; i < s88->modcnt; i++) {
				json_addIntValue(jstk, s88data[i]);
			}
			break;
		case EVENT_FBPARAM:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "s88module", cnf_getconfig()->s88Modules);
			json_addIntItem(jstk, "canmodule", cnf_getconfig()->canModules);
			json_addIntItem(jstk, "lnetmodule", cnf_getconfig()->lnetModules);
			json_addIntItem(jstk, "s88frequency", s88_getFrequency());
			break;
		case EVENT_FBNEW:
			fbevt = (fbeventT *) e->src;
			log_msg (LOG_INFO, "%s(): MOD %d: 0x%04x\n", __func__, fbevt->module, fbevt->status);
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "module", fbevt->module);
			json_addIntItem(jstk, "occupy", fbevt->status);
			break;
		case EVENT_CURRENT:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "current", e->param);
			break;
		case EVENT_BOOSTER:
			sc = cnf_getconfig();
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "trackvoltage", ts_getVoltage());
			json_addIntItem(jstk, "maxcurrent", ts_getCurrent());
			json_addIntItem(jstk, "shortsens", ts_getSensitivity());
			json_addIntItem(jstk, "inrushtime", ts_getInrush());
			json_addIntItem(jstk, "route_i", !(sc->sysflags & SYSFLAG_NOMAGONMAINBST));
			json_addIntItem(jstk, "route_m", !(sc->sysflags & SYSFLAG_NOMAGONMKLNBST));
			json_addIntItem(jstk, "route_d", !(sc->sysflags & SYSFLAG_NOMAGONCDEBST));
			json_addIntItem(jstk, "bidi_global_short", !!(sc->sysflags & SYSFLAG_GLOBAL_BIDIB_SHORT));
			json_addIntItem(jstk, "bidi_remote_onoff", !!(sc->sysflags & SYSFLAG_BIDIB_ONOFF));
			json_addIntItem(jstk, "ptvoltage", ts_getPtVoltage());
			json_addIntItem(jstk, "mmsens", sc->mmshort);
			json_addIntItem(jstk, "dccsens", sc->dccshort);
			break;
		case EVENT_PROTOCOL:
			sc = cnf_getconfig();
			fc = cnf_getFMTconfig();
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "Def_L_Proto", db_getLoco(0, false)->fmt);
			json_addIntItem(jstk, "MM_Pause", fc->mm.pause);
			json_addIntItem(jstk, "PreambleBits", fc->dcc.preamble);
			json_addIntItem(jstk, "Bit1length", fc->dcc.tim_one);
			json_addIntItem(jstk, "Bit0length", fc->dcc.tim_zero);
			json_addIntItem(jstk, "Adr_Repeats", fc->dcc.repeat);
			json_addIntItem(jstk, "Adr_RepeatsMM", fc->mm.repeat);
			json_addIntItem(jstk, "Adr_Repeatsm3", fc->m3.repeat);
			json_addIntItem(jstk, "POM_Repeats", fc->dcc.pomrepeat);
			json_addIntItem(jstk, "RailCom", (fc->sigflags & SIGFLAG_RAILCOM) ? 1 : 0);
			json_addIntItem(jstk, "NOP", (fc->sigflags & SIGFLAG_DCCNOP) ? 1 : 0);
			json_addIntItem(jstk, "DCCA", (fc->sigflags & SIGFLAG_DCCA) ? 1 : 0);
			json_addIntItem(jstk, "DCClong", (fc->sigflags & SIGFLAG_DCC_LONG_ADR) ? 1 : 0);
			json_addIntItem(jstk, "Purge", sc->locopurge);
			json_addIntItem(jstk, "M3enable", (fc->sigflags & SIGFLAG_M3ENABLED) ? 1 : 0);
			json_addIntItem(jstk, "bidibacclogic", (sc->sysflags & SYSFLAG_ACC_LOGICAL) ? 1 : 0);
			break;
		case EVENT_SNIFFER:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addUintItem(jstk, "filter", ui32DisplayFilter);
			break;
		case EVENT_ACCFMT:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			itm = json_addArrayItem(jstk, "accfmt");
			jstk = json_pushArray(jstk, itm);
			trnt_deffmt = db_getTurnout(0)->fmt;
			for (i = 1; i <= MAX_MM_TURNOUT; i++) {
				if ((t = db_lookupTurnout(i)) != NULL && t->fmt != trnt_deffmt) {
					json_addIntValue(jstk, db_getTurnout(i)->adr);
				}
			}
			json_addIntValue(jstk, -1);
			break;
		case EVENT_ACCESSORY:
			fc = cnf_getFMTconfig();
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "Def_A_Proto", (db_getTurnout(0)->fmt == TFMT_DCC) ? 1 : 0);
			json_addIntItem(jstk, "min_switch_time", trnt_getMinTime());
			json_addIntItem(jstk, "max_switch_time", trnt_getMaxTime());
			json_addIntItem(jstk, "repeats", fc->accrepeat);
			break;
		case EVENT_ENVIRONMENT:
			sc = cnf_getconfig();
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "supply", (an_getSupply() + 50) / 100);
			json_addIntItem(jstk, "temperature", an_getTemperature());
			json_addIntItem(jstk, "startstate", !!(sc->sysflags & SYSFLAG_STARTSTATE));
			break;
		case EVENT_RAILCOM:
			msg = e->src;
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "adr", msg->adr);
			json_addIntItem(jstk, "dec", msg->dtype);
			json_addIntItem(jstk, "cv", msg->cva.cv);
			json_addIntItem(jstk, "msg", msg->mt);
			json_addIntItem(jstk, "len", msg->len);
			json_addIntItem(jstk, "d0", msg->data[0]);
			json_addIntItem(jstk, "d1", msg->data[1]);
			json_addIntItem(jstk, "d2", msg->data[2]);
			json_addIntItem(jstk, "d3", msg->data[3]);
			json_addIntItem(jstk, "d4", msg->data[4]);
			break;
		case EVENT_CONTROLS:
			if ((dev = (struct extDevice *) e->src) != NULL) {
				root = json_addObject(NULL);	// create root object
				jstk = json_pushObject(NULL, root);
				switch (e->param) {
					case 0:
						json_addStringItem(jstk, "action", "DISCONNECT");
						break;
					case 1:
						json_addStringItem(jstk, "action", "CONNECT");
						break;
					case 2:
						json_addStringItem(jstk, "action", "CHANGE");
						break;
					default:		// to always have the 'itm' set to a value
						json_addStringItem(jstk, "action", "unknown");
						break;
				}
				switch (dev->bus) {
					case BUS_EASYNET:
						json_addStringItem(jstk, "bus", "EN");
						json_addIntItem(jstk, "control", dev->id);
						json_addUintItem(jstk, "serial", dev->serial);
						json_addStringItem(jstk, "SW", dev->swrev);
						json_addStringItem(jstk, "HW", dev->hwrev);
						json_addStringItem(jstk, "type", "CONTROL");
						break;
					case BUS_XPRESSNET:
						json_addStringItem(jstk, "bus", "XN");
						json_addIntItem(jstk, "control", dev->id);
						break;
					case BUS_LOCONET:
						json_addStringItem(jstk, "bus", "LN");
						json_addIntItem(jstk, "control", dev->id);
						json_addUintItem(jstk, "serial", dev->serial);
						json_addStringItem(jstk, "type", "CONTROL");
						break;
					case BUS_MCAN:
						json_addStringItem(jstk, "bus", "MC");
						json_addIntItem(jstk, "control", dev->id);
						json_addUintItem(jstk, "serial", dev->serial);
						json_addStringItem(jstk, "SW", dev->swrev);
						json_addStringItem(jstk, "HW", dev->hwrev);
						break;
					case BUS_BIDIBUS:
						json_addStringItem(jstk, "bus", "BB");
						break;
				}
			}
			break;
		case EVENT_MODELTIME:
			if ((mt = (struct modeltime *) e->src) != NULL) {
				root = json_addObject(NULL);	// create root object
				jstk = json_pushObject(NULL, root);
				json_addIntItem(jstk, "year", mt->year);
				json_addIntItem(jstk, "mon", mt->mon);
				json_addIntItem(jstk, "mday", mt->mday);
				json_addIntItem(jstk, "wday", mt->wday);
				json_addIntItem(jstk, "hour", mt->hour);
				json_addIntItem(jstk, "min", mt->min);
				json_addIntItem(jstk, "factor", mt->speedup);
			}
			break;
		case EVENT_BIDIDEV:
			bn = (struct bidibnode *) e->src;		// root of the node tree
			root = json_addObject(NULL);			// create root object
			jstk = json_pushObject(NULL, root);
			itm = json_addArrayItem(jstk, "dev");	// array of devices with three integers each
			jstk = json_pushArray(jstk, itm);
			i = cgi_listBIDIBnodes(jstk, bn);
			jstk = json_pop(jstk);
			json_addIntItem(jstk, "bidibmodule", i);		// total count of devices reported
			break;
		case EVENT_LIGHTS:
			sc = cnf_getconfig();
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "effect", (sc->sysflags & SYSFLAG_LIGHTEFFECTS) ? 1 : ((sc->sysflags & SYSFLAG_LIGHTSOFF) ? 2 : 0));
			break;
		case EVENT_LOGMSG:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addIntItem(jstk, "level", e->param);
			json_addStringItem(jstk, "msg", (char *) e->src);
			break;
		case EVENT_EXTCONTROL:
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			json_addStringItem(jstk, "extcontrol", "");
			json_addIntItem(jstk, "locked",	(e->param & EXTCTRL_LOCKED)	? 1 : 0);
			json_addIntItem(jstk, "p50x",	(e->param & EXTCTRL_P50X)	? 1 : 0);
			json_addIntItem(jstk, "bidib",	(e->param & EXTCTRL_BIDIB)	? 1 : 0);
			break;
		case EVENT_ENBOOT:			// the boot progress for EasyNet devices
			enprogress = (struct en_bootProgress *) e->src;
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			itm = json_addArrayItem(jstk, "enboot");
			if (enprogress) {		// at the end of transmission, the array length will be zero
				jstk = json_pushArray(jstk, itm);
				json_addIntValue(jstk, enprogress->current);
				json_addIntValue(jstk, enprogress->total);
			}
			break;
		case EVENT_CONSIST:
			c = (struct consist *) e->src;
			root = json_addObject(NULL);	// create root object
			jstk = json_pushObject(NULL, root);
			itm = json_addArrayItem(jstk, "consists");	// array of arrays with the loco addresses
			jstk = json_pushArray(jstk, itm);
			while (c) {
				val = json_addArray(jstk);
				jstk = json_pushArrayValue(jstk, val);
				for (i = 0; i < MAX_CONSISTLENGTH; i++) {
					if (c->adr[i]) json_addIntValue(jstk, c->adr[i]);
				}
				jstk = json_pop(jstk);
				c = c->next;
			}
			break;
		default:
			rc = 0;
			break;
	}
	if (root != NULL)  {	// send JSON structure to client
		rc = cgi_sendJSONeventdata(cb->sock, root);
		json_free(root);
		json_popAll(jstk);
	}

	if (rc >= 0) return true;		// OK, continue waiting for more events

	return html_finishEventHandler(cb);
}

static int cgi_regEvent (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv;
	struct cbdata *cb;
	uint32_t ev_mask = 0;
	int loco = 0, rc, i;
	int tout = 60000;
	flexval fv;
//	struct en_client *encp;
//	struct CAN_Client *cancp;

	(void) rest;
	(void) sz;

	fv.i32 = sock;

	kv = hr->param;
	while (kv) {
		if (!strcasecmp("lok", kv->key)) {
			loco = atoi(kv->value);
			if (loco > 0) {
				ev_mask |= 1 << EVENT_LOCO_SPEED;
				ev_mask |= 1 << EVENT_LOCO_FUNCTION;
				ev_mask |= 1 << EVENT_LOCO_PARAMETER;
			}
		} else if (!strcasecmp("locodb", kv->key)) {
			ev_mask |= 1 << EVENT_LOCO_DB;
		} else if (!strcasecmp("turnout", kv->key)) {
			ev_mask |= 1 << EVENT_TURNOUT;
		} else if (!strcasecmp("status", kv->key)) {
			ev_mask |= 1 << EVENT_SYS_STATUS;
		} else if (!strcasecmp("bidibdev", kv->key)) {
			ev_mask |= 1 << EVENT_BIDIDEV;
		} else if (!strcasecmp("s88", kv->key)) {
			ev_mask |= 1 << EVENT_FEEDBACK;
			ev_mask |= 1 << EVENT_FBPARAM;
			ev_mask |= 1 << EVENT_FBNEW;
		} else if (!strcasecmp("current", kv->key)) {
			ev_mask |= 1 << EVENT_CURRENT;
		} else if (!strcasecmp("booster", kv->key)) {
			ev_mask |= 1 << EVENT_BOOSTER;
		} else if (!strcasecmp("newloco", kv->key)) {
			ev_mask |= 1 << EVENT_NEWLOCO;
		} else if (!strcasecmp("protocol", kv->key)) {
			ev_mask |= 1 << EVENT_PROTOCOL;
		} else if (!strcasecmp("accfmt", kv->key)) {
			ev_mask |= 1 << EVENT_ACCFMT;
		} else if (!strcasecmp("accessory", kv->key)) {
			ev_mask |= 1 << EVENT_ACCESSORY;
		} else if (!strcasecmp("sniffer", kv->key)) {
			ev_mask |= 1 << EVENT_SNIFFER;
		} else if (!strcasecmp("environment", kv->key)) {
			ev_mask |= 1 << EVENT_ENVIRONMENT;
		} else if (!strcasecmp("controls", kv->key)) {
			ev_mask |= 1 << EVENT_CONTROLS;
		} else if (!strcasecmp("railcom", kv->key)) {
			ev_mask |= 1 << EVENT_RAILCOM;
			reply_register (DECODER_ANY, 0, DECODERMSG_ANY, rc_event_handler, fv, 0);
		} else if (!strcasecmp("timeout", kv->key) && *kv->key != 0) {
			tout = atoi(kv->value);
			if (tout < 0) tout = 0;					// no timeout at all
			else if (tout < 1000) tout = 1000;		// minimum timeout is 1s (btw.: max timeout is around 23 days)
		} else if (!strcasecmp("modeltime", kv->key)) {
			ev_mask |= 1 << EVENT_MODELTIME;
		} else if (!strcasecmp("log", kv->key)) {
			ev_mask |= 1 << EVENT_LOGMSG;
		} else if (!strcasecmp("extcontrol", kv->key)) {
			ev_mask |= 1 << EVENT_EXTCONTROL;
		} else if (!strcasecmp("lights", kv->key)) {
			ev_mask |= 1 << EVENT_LIGHTS;
		} else if (!strcasecmp("enboot", kv->key)) {
			ev_mask |= 1 << EVENT_ENBOOT;
		} else if (!strcasecmp("consist", kv->key)) {
			ev_mask |= 1 << EVENT_CONSIST;
		}
		kv = kv->next;
	}

	if (ev_mask == 0) return 0;

	if ((cb = malloc (sizeof(*cb))) == NULL) return 0;
	cb->sock = sock;
	cb->hr = hr;
	cb->loco = loco;

	html_sendStatus(sock);		// send status first - we must not send events before having sent the request status (200 OK)
	for (i = 0, rc = 0; i < EVENT_MAX_EVENT && rc == 0; i++) {
		if (ev_mask & (1 << i)) {
			log_msg (LOG_INFO, "%s(): registering %d\n", __func__, i);
			rc = event_register(i, cgi_eventHandler, cb, tout);
		}
	}
	if (!rc) {
		if (ev_mask & (1 << EVENT_MODELTIME))	event_fire (EVENT_MODELTIME, 0, NULL);
		if (ev_mask & (1 << EVENT_BOOSTER))		event_fire (EVENT_BOOSTER, 0, NULL);
#ifdef CENTRAL_FEEDBACK
		if (ev_mask & (1 << EVENT_FBPARAM))		event_fire (EVENT_FBPARAM, 0, NULL);
#else
		if (ev_mask & (1 << EVENT_FEEDBACK))	s88_triggerUpdate();
#endif
		if (ev_mask & (1 << EVENT_PROTOCOL))	event_fire (EVENT_PROTOCOL, 0, NULL);
		if (ev_mask & (1 << EVENT_BIDIDEV))		BDBnode_nodeEvent();
		if (ev_mask & (1 << EVENT_ACCESSORY))	event_fire (EVENT_ACCESSORY, 0, NULL);
		if (ev_mask & (1 << EVENT_ACCFMT))		event_fire (EVENT_ACCFMT, 0, NULL);
		if (ev_mask & (1 << EVENT_ENVIRONMENT))	event_fire (EVENT_ENVIRONMENT, 0, NULL);
		if (ev_mask & (1 << EVENT_LOCO_DB))		event_fire (EVENT_LOCO_DB, 0, NULL);
		if (ev_mask & (1 << EVENT_EXTCONTROL))	event_fire (EVENT_EXTCONTROL, rt.ctrl, NULL);
		if (ev_mask & (1 << EVENT_LIGHTS))		event_fire (EVENT_LIGHTS, 0, NULL);
		if (ev_mask & (1 << EVENT_CONSIST))		consist_event();
		if (ev_mask & (1 << EVENT_CONTROLS)) {
			en_reportControls();
			ln_reportControls();
			xn_reportControls();
			mcan_reportControls();
		}
		vTaskDelete(NULL);	// end this task and hold socket open
	}

	// event registration failed
	log_error ("%s() could not register eventlistener\n", __func__);
	free (cb);
	return 0;
}

static int cgi_consist (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv, *hdrs;
	int base , adr;
	const char *res;

	(void) rest;
	(void) sz;

	res = NULL;

	if ((kv = kv_lookup(hr->param, "adr")) != NULL) {	// we have a base address (for add actions)
		base = atoi(kv->value);
		if ((kv = kv_lookup(hr->param, "add")) != NULL) {	// yes, there is a loco we want to add
			adr = atoi(kv->value);
			if (consist_couple(base, adr) == NULL) res = NOT_ACCEPTABLE;
		} else {
			res = BAD_REQUEST;
		}
	} else {	// dissolve/drop command?
		if ((kv = kv_lookup(hr->param, "drop")) != NULL) {	// we will drop a loco from a consist
			adr = atoi(kv->value);
			if (!consist_remove(adr)) res = PRECONDITION_FAILED;
		} else if ((kv = kv_lookup(hr->param, "dissolve")) != NULL) {	// we will dissolve the consist completely
			adr = atoi(kv->value);
			if (!consist_dissolve(adr)) res = PRECONDITION_FAILED;
		} else {
			res = BAD_REQUEST;
		}
	}
	if (res) consist_event();	// if nothing changed due to an error or non-existing consist, we send the unchanged list back to the client

	hdrs = kv_add(NULL, "Content-Length", "0");
	httpd_header(sock, (res) ? res : FILE_OK, hdrs);
	kv_free(hdrs);
	return 0;
}

/**
 * Just store content in file identified by fd
 */
static int cgi_fileStorage (void *arg, uint8_t *buf, int len)
{
	int fd, rc;

	fd = (int) arg;
	if (!buf) {		// reception finished - close file
		yaffs_close(fd);
	} else {
		while (fd >= 0 && len > 0) {
			rc = yaffs_write(fd, buf, len);
			if (rc <= 0) return -1;
			len -= rc;
		}
	}
	return 0;
}

/**
 * Find a given pattern in a stream of data.
 * The fundction reads from the file or socket 'fd' until it either finds the
 * given pattern or encounters EOF or error on 'fd'.
 *
 * The pattern must be a C-string (null terminated) and the file contents should be
 * text inclunding line breaks and everything except a null byte because we use the
 * standard C library function strstr() to locate the pattern in the buffer. Also,
 * the buffer must be greater than the string length of the pattern (at least twice
 * the size).
 *
 * After return of the function, the buffer starts with searched pattern, if found.
 * In this case, the returned length is at least the string length of the search pattern.
 * A return code of less than the pattern length will indicate some kind of error.
 *
 * \param fd		the file descriptor or socket to read from
 * \param buf		a buffer for the operation
 * \param bufsize	the size of the buffer in bytes (one byte less is used to be able to
 * 					terminate the contents with a null byte to mimic a C-string)
 * \param valid		the number of already read (i.e. valid) bytes in the buffer when this function is called
 * \param pattern	the pattern we are looking for
 * \param skipped	if provided, the function adds the amount of discared input so far to this variable
 * \return			the number of valid bytes in the buffer with the pattern begiinning at the buffer start,
 * 					zero if pattern was not found at end of stream or a negative value if an error occured
 */
int cgi_findPatternInStream (int fd, uint8_t *buf, int bufsize, int valid, char *pattern, int *skipped)
{
	uint8_t *rcv, *s;
	int rest, rc, len, discard;

	if (!pattern) return -1;
	len = strlen(pattern);
	if (!buf || bufsize < (len * 2)) return -1;

	rcv = buf + valid;
	do {
		*rcv = 0;
		if ((s = (uint8_t *) strstr((char *) buf, pattern)) != NULL) {
			rest = rcv - s;		// this is the rest we will keep in buffer to return to caller
			discard = s - buf;
//			log_msg (LOG_INFO, "%s() pattern found, discarding '%*.*s'\n", __func__, discard, discard, buf);
			memmove(buf, s, rest);
			if (skipped) *skipped += discard;
			return rest;
		} else if (rcv > (buf + len)) {
			discard = rcv - len - buf;
//			log_msg (LOG_INFO, "%s() discarding '%*.*s'\n", __func__, discard, discard, buf);
			memmove (buf, rcv - len, len);					// shift the last bytes (defined by pattern length) to the beginning of the buffer
			rcv = buf + len;
			if (skipped) *skipped += discard;
		}
		rc = read (fd, rcv, bufsize - (rcv - buf) - 1);		// refill buffer and reserve the last byte for a terminating null
		if (rc > 0) rcv += rc;
	} while (rc > 0);
	return rc;
}

static int cgi_createFile (int sock, char *dir, char *name)
{
	char fname[FILENAME_MAX];
	int fd;

	canonical_path(fname, dir, name);
	ensure_path(fname);
	log_msg (LOG_INFO, "%s() fname = '%s'\n", __func__, fname);
	if ((fd = yaffs_open(fname, O_CREAT | O_RDWR | O_TRUNC, S_IREAD | S_IWRITE)) < 0) {
		log_error ("%s() Cannot open file (errno = %d)\n", __func__, errno);
		httpd_header(sock, FILE_NOT_FOUND, NULL);
	}

	return fd;
}

static int cgi_update (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv;
	int (*func)(void *, uint8_t *, int);
	void *arg;
	char *boundary = NULL;
	uint8_t *buf;
	int fd, discard, len;

	if ((kv = kv_lookup(hr->headers, "Content-Length")) != NULL) {
		len = atoi(kv->value);
	} else {
		httpd_header(sock, LENGTH_REQUIRED, NULL);
		return 0;
	}

	func = NULL;
	if ((kv = kv_lookup(hr->param, "device")) != NULL) {	// this is an update request
		if (!strcmp ("zentrale", kv->value)) {
			if ((fd = cgi_createFile(sock, FIRMWARE_DIR, kv->value)) < 0) return 0;
			arg = (void *) fd;
			func = cgi_fileStorage;
			sig_setMode(TM_RESET);
		} else if (!strcmp ("ENContr", kv->value)) {
			func = en_bootReadBuffer;
			en_bootReadBuffer(NULL, NULL, -1);		// init the memory and line buffer
			arg = NULL;
		} else if (!strcmp ("web", kv->value)) {
			if ((fd = cgi_createFile(sock, FIRMWARE_DIR, "html.cpio")) < 0) return 0;
			arg = (void *) fd;
			func = cgi_fileStorage;
		}
	} else {
		if ((kv = kv_lookup(hr->param, "fname")) != NULL) {
			if ((fd = cgi_createFile(sock, "/", kv->value)) < 0) return 0;
			arg = (void *) fd;
			func = cgi_fileStorage;
		} else {
			httpd_header(sock, BAD_REQUEST, NULL);
			return 0;
		}
	}
	if (func == NULL) {		// we need some kind of target for the file data
		httpd_header(sock, FILE_NOT_FOUND, NULL);
		return 0;
	}

	if ((kv = kv_lookup(hr->headers, "Content-Type")) != NULL) {
		if ((boundary = strstr(kv->value, "boundary=")) != NULL) {
			while (*boundary && *boundary != '=') boundary++;
			if (*boundary == '=') boundary++;
		}
	}

	// let's allocate a buffer for data reception
	if ((buf = malloc(RX_BUFSIZE)) == NULL) {
		httpd_header(sock, INTERNAL_SERVER_ERROR, NULL);
		return 0;
	}
	if (sz > 0) {
//		log_msg(LOG_INFO, "%s() already received: '%*.*s'\n", __func__, sz, sz, rest);
		memcpy(buf, rest, sz);		// copy the already received part to our local buffer
	}
	// if we are looking for a boundary, scan for it and then for two lineending (i.e. an empty line)
	if (boundary) {
		discard = 0;
		if ((sz = cgi_findPatternInStream(sock, buf, RX_BUFSIZE, sz, boundary, &discard)) > 0) {	// if found, look for two line delimiters after the boundary
			sz = cgi_findPatternInStream(sock, buf, RX_BUFSIZE, sz, "\r\n\r\n", &discard);
		}
		if (sz <= 0) {
			log_error ("%s(): boundary not found in stream - give up\n", __func__);
			httpd_header(sock, BAD_REQUEST, NULL);
			return 0;
		}
		sz -= 4;
		memmove (buf, buf + 4, sz);
		// calculate effective file length, transmission ends in "\r\n--<boundary>--\r\n" -> subtract 8 chars + strlen(boundary) from flen!
		len -= discard + 4 + 8 + strlen(boundary);
		log_msg (LOG_INFO, "%s() resulting file length %d\n", __func__, len);
	}

	if (len > 0 && sz > 0) {	// we have the first few bytes available
		func(arg, buf, (len > sz) ? sz : len);
		len -= sz;
	}
	while (len > 0) {
		sz = lwip_recv(sock, buf, RX_BUFSIZE, 0);
		if (sz <= 0) {	// error or closed by the other end
			if (sz < 0) log_error ("%s(): ERROR %d\n", __func__, sz);
			break;
		}
		func (arg, buf, (len > sz) ? sz : len);
		len -= sz;
	}

	func (arg, NULL, 0);		// signal end of data

	if (len > 0) {
		log_error ("%s(): premature end-of-transmission with %d bytes left\n", __func__, len);
	}
	log_msg (LOG_INFO, "%s(): upload finished\n", __func__);
	httpd_header(sock, RESOURCE_CREATED, NULL);
	free (buf);
	return 0;
}

static int cgi_readfile (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv, *hdrs;
	struct yaffs_stat stat;
	char *fname, *tmp;
	uint8_t *buf;
	int fd, len;

	(void) rest;
	(void) sz;

	if ((kv = kv_lookup(hr->param, "file")) == NULL) {
		log_msg (LOG_INFO, "%s(): parameter 'file' not found\n", __func__);
		httpd_header(sock, FILE_NOT_FOUND, NULL);
		return 0;
	}

	tmp = tmp64();
	if (!strcasecmp (kv->value, "CONFIG.INI")) {
		fname = CONFIG_SYSTEM;
		sprintf (tmp, "attachment; filename=\"config.ini\"");
	} else if (!strcasecmp (kv->value, "LOCO.INI")) {
		fname = CONFIG_LOCO;
		sprintf (tmp, "attachment; filename=\"loco.ini\"");
	} else {
		log_msg (LOG_INFO, "%s(): request for '%s' not supported\n", __func__, kv->value);
		httpd_header(sock, FILE_NOT_FOUND, NULL);
		return 0;
	}

	if ((buf = malloc(1024)) == NULL) {
		log_error ("%s(): cannot allocate copy buffer\n", __func__);
		httpd_header(sock, FILE_NOT_FOUND, NULL);
		return 0;
	}
	if ((fd = yaffs_open (fname, 0, 0)) <= 0) {
		log_error ("%s(): cannot open '%s'\n", __func__, fname);
		httpd_header(sock, FILE_NOT_FOUND, NULL);
		free (buf);
		return 0;
	}
	if (yaffs_fstat(fd, &stat) != 0) {
		log_error ("%s(): cannot stat '%s'\n", __func__, fname);
		httpd_header(sock, FILE_NOT_FOUND, NULL);
		free (buf);
		yaffs_close(fd);
		return 0;
	}

	hdrs = kv_add(NULL, "Content-Type", "application/octet-stream");
	hdrs = kv_add(hdrs, "Content-Disposition", tmp);
	sprintf (tmp, "%lld", stat.st_size);
	hdrs = kv_add(hdrs, "Content-Length", tmp);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);

	while ((len = yaffs_read(fd, buf, sizeof(buf))) > 0) {
		socket_senddata(sock, buf, len);
	}

	free (buf);
	yaffs_close(fd);
	return 0;
}

static void cgi_postBiDiB_Dev (int sock, uint32_t dev)
{
	json_valT *root, *ar;
	json_itmT *itm;
	json_stackT *jstk;
	struct bidibnode *bn;
	int i;

	log_msg (LOG_INFO, "%s() looking for node 0x%08lx\n", __func__, dev);
	if ((bn = BDBnode_lookupNode(dev)) == NULL) {
		log_msg (LOG_INFO, "%s(): Node %s not found\n", __func__, bidib_formatAdrStack(dev));
		return;
	}
	root = json_addObject(NULL);	// create root object
	jstk = json_pushObject(NULL, root);
	json_addStringItem(jstk, "dev", bidib_formatAdrStack(dev));
	json_addStringItem(jstk, "product", utf8_iso2utf(bn->product));
	json_addStringItem(jstk, "user", utf8_iso2utf(bn->user));
	json_addUintItem(jstk, "identify", (bn->flags & NODEFLG_IDENTIFY) ? 1 : 0);
	itm = json_addArrayItem(jstk, "features");
	jstk = json_pushArray(jstk, itm);
	for (i = 0; i < bn->featurecount; i++) {
		ar = json_addArray(jstk);
		jstk = json_pushArrayValue(jstk, ar);
		json_addUintValue(jstk, bn->features[i].feature);
		json_addUintValue(jstk, bn->features[i].value);
		jstk = json_pop(jstk);
	}
	jstk = json_pop(jstk);
	cgi_sendJSON(sock, root);
	json_free(root);
	json_popAll(jstk);
}

static void cgi_postLocoDBdata (int sock, locoT *l)
{
	struct dccaInfo *dcca;
	char response[256], *s, *p, loconame[LOCO_NAME_LEN], c;
	funcT *ft;
	int i;

	if (!l) return;

	socket_printf (sock, "\"lok\": %d,\n", l->adr);
	ft = db_getLocoFunc (l, 0);
	i = 0;
	s = response;
	s += sprintf (s, "\"funcicons\": [ %u,%u,%u", ft->fnum, ft->icon, ft->timing);
	i++;
	while (i < 69) {
		for (; i < 69; i++) {
			ft = db_getLocoFunc (l, i);
			if (ft->icon || ft->timing)
				s += sprintf (s, ", %u,%u,%u", ft->fnum, ft->icon, ft->timing);
			if (strlen(response) > 220) break;
		}
		if (i == 69) break;
		socket_sendstring (sock, response);
		response[0] = 0;
		s = response;
	}
	s += sprintf (s, " ],\n");
	socket_sendstring (sock, response);
	socket_printf (sock, "\"fmt\": \"%s\",\n", db_fmt2string(l->fmt));
	socket_printf (sock, "\"uid\": %lu,\n", l->uid);
	socket_printf (sock, "\"vid\": %lu,\n", l->vid);
	socket_printf (sock, "\"maxfunc\": %d,\n", l->maxfunc);
	switch (l->config) {
		case CONF_DCCA:			c = 'A'; break;
		case CONF_M3: 			c = 'X'; break;
		case CONF_RAILCOMPLUS:	c = 'R'; break;
		case CONF_MANUAL:
		default:				c = 'M'; break;
	}
	socket_printf (sock, "\"conf\": \"%c\",\n", c);
	s = l->name;
	p = loconame;
	while (*s) {
		*p++ = (*s == '"') ? '/' : *s;
		s++;
	}
	*p = 0;
//	log_msg (LOG_INFO, "%s(): loconame: %s\n", __func__, loconame);
	socket_printf (sock, "\"name\": \"%s\",\n", loconame);
	if ((dcca = l->dcca) != NULL) {
		socket_printf (sock, "\"vendor\": \"%s\",\n", dcca->vendor);
		socket_printf (sock, "\"product\": \"%s\",\n", dcca->product);
		socket_printf (sock, "\"HW\": \"%s\",\n", dcca->hw_version);
		socket_printf (sock, "\"FW\": \"%s\",\n", dcca->fw_version);
		socket_printf (sock, "\"shortname\": \"%s\",\n", dcca->shortname);
		socket_printf (sock, "\"desc\": \"%s\",\n", dcca->userdesc);
		socket_printf (sock, "\"image\": %d,\n", dcca->decoderimage);
		socket_printf (sock, "\"icon\": %d,\n", dcca->decodericon);
		socket_printf (sock, "\"userimage\": %d,\n", dcca->userimage);
		socket_printf (sock, "\"adr_req\": %d,\n", dcca->adr_req);
	}

	switch (rt.tm) {
		case TM_STOP:
			socket_sendstring (sock, "\"state\": \"STOP\"");
			break;
		case TM_SHORT:
			socket_sendstring (sock, "\"state\": \"SHORT\"");
			break;
		case TM_HALT:
			socket_sendstring (sock, "\"state\": \"HALT\"");
			break;
		case TM_GO:
			socket_sendstring (sock, "\"state\": \"GO\"");
			break;
		case TM_TESTDRIVE:
			socket_sendstring (sock, "\"state\": \"TESTDRIVE\"");
			break;
		default:		// others are ignored (just in case future expansion to more system states)
			break;
	}
}

static void cgi_postLocoOPdata (int sock, ldataT *l)
{
	if (!l) return;

	socket_printf (sock, "\"speed\": \"%c%d\",\n", (l->speed & 0x80) ? 'F' : 'R', l->speed & 0x7F);
	socket_printf (sock, "\"funcs\": [ %lu, %lu, %lu, %lu ]\n", l->funcs[0], l->funcs[1], l->funcs[2], l->funcs[3]);
}

static void cgi_locoInfo (int sock, locoT *l)
{
	struct key_value *hdrs;

	if (!l) return;
	hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	socket_printf (sock, "{ \"refresh\": %d,\n", loco_call(l->adr, false) ? 1: 0);

	cgi_postLocoDBdata (sock, l);

	socket_sendstring (sock, "\n}\n");
}

static void cgi_postLoco (int sock, ldataT *l)
{
	struct key_value *hdrs;

	if (!l || !l->loco) return;
	hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	socket_sendstring (sock, "{ \"refresh\": 1,\n");

	cgi_postLocoDBdata (sock, l->loco);
	socket_sendstring (sock, ",\n");
	cgi_postLocoOPdata (sock, l);

	socket_sendstring (sock, "}\n");
}

static void cgi_postTurnout (int sock, turnoutT *t)
{
	struct key_value *hdrs;

	if (!t) return;

	hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	socket_printf (sock, "{ \"trnt\": %d, \"fmt\": \"%s\" }\n", t->adr, db_fmt2string(t->fmt));
}

static bool cgi_cvCallback (struct decoder_reply *msg, flexval priv)
{
	struct key_value *hdrs;
	char response[256], *s;
	int i, sock;

	log_msg (LOG_INFO, "%s()\n", __func__);
	if ((sock = priv.i32) <= 0) return false;		// no socket provided (0 might be legal, but we know it can't be a socket in this system!)
	hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);

#if 1
	switch (msg->mt) {
		case DECODERMSG_POM:		// a single CV byte from a DCC decoder
			sprintf (response, "{ \"cv\": [ %ld, %d ] }\n", msg->cva.cv, msg->data[0]);
			break;
		case DECODERMSG_XPOM00:		// XPOM00 4 Bytes
		case DECODERMSG_XPOM01:		// XPOM01 4 Bytes
		case DECODERMSG_XPOM10:		// XPOM10 4 Bytes
		case DECODERMSG_XPOM11:		// XPOM11 4 Bytes
			sprintf (response, "{ \"xpom\": [ %ld, %d, %d, %d, %d ] }\n", msg->cva.cv, msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
			break;
		case DECODERMSG_M3DATA:
			s = response;
			s += sprintf (s, "{ \"cv\": %d, \"sub\": %d, \"data\": [", msg->cva.m3cv, msg->cva.m3sub);
			for (i = 0; i < msg->len; i++) s += sprintf (s, "%s %u", (i == 0) ? "" : ",", msg->data[i]);
			sprintf (s, " ] }\n");
			break;
		case DECODERMSG_TIMEOUT:
			sprintf (response, "{ \"cvNoAnswer\": [ %ld, -1 ] }\n", msg->cva.cv);
			break;
		case DECODERMSG_INVALID:	// TODO wrong answer: this code is not correct - we possibly need yet another type of message ...
			sprintf (response, "{ \"cvWrong\": [ %ld, %d ] }\n", msg->cva.cv, msg->data[0]);
			break;
		case DECODERMSG_NOANSWER:	// TODO wrong answer: this code is not correct - we possibly need yet another type of message ...
			sprintf (response, "{ \"cvEmpty\": [ %ld, -1 ] }\n", msg->cva.cv);
			break;
		default:				// in any case: send an answer!
			sprintf (response, "{ \"cv\": [ %ld, -1 ] }\n", msg->cva.cv);
			break;
	}
#else
	switch (msg->id[0]) {
		case RC_ID0:		// a single CV byte from a DCC decoder
			sprintf (response, "{ \"cv\": [ %ld, %d ] }\n", msg->cva.cv, msg->data[0]);
			break;
		case RC_ID8:		// XPOM00 4 Bytes
		case RC_ID9:		// XPOM01 4 Bytes
		case RC_ID10:		// XPOM10 4 Bytes
		case RC_ID11:		// XPOM11 4 Bytes
			sprintf (response, "{ \"xpom\": [ %ld, %d, %d, %d, %d ] }\n", msg->cva.cv, msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
			break;
		case RC_M3DATA:
			s = response;
			s += sprintf (s, "{ \"cv\": %d, \"sub\": %d, \"data\": [", msg->cva.m3cv, msg->cva.m3sub);
			for (i = 0; i < msg->len[0]; i++) s += sprintf (s, "%s %u", (i == 0) ? "" : ",", msg->data[i]);
			sprintf (s, " ] }\n");
			break;
		case RC_NO_ANSWER:
			sprintf (response, "{ \"cv-noAnswer\": [ %ld, -1 ] }\n", msg->cva.cv);
			break;
		case RC_WRONG_ANSWER:
			sprintf (response, "{ \"cv-wrong\": [ %ld, %d ] }\n", msg->cva.cv, msg->data[0]);
			break;
		case RC_EMPTY:
			sprintf (response, "{ \"cv-empty\": [ %ld, -1 ] }\n", msg->cva.cv);
			break;
		default:				// in any case: send an answer!
			sprintf (response, "{ \"cv\": [ %ld, -1 ] }\n", msg->cva.cv);
			break;
	}
#endif
	log_msg(LOG_HTTPD, response);
	socket_sendstring (sock, response);
	lwip_close(sock);
	return false;
}

static int cgi_getDevice (int sock, struct http_request *hr)
{
	struct key_value *hdrs;
	struct key_value *kv;
	ldataT *l = NULL;
	int v, min, max, adr;
	uint32_t uv;

	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		v = atoi(kv->value);
		if (v > 0) l = loco_call(v, true);
		cgi_postLoco(sock, l);
		return -1;
	} else if ((kv = kv_lookup(hr->param, "bidibDev")) != NULL) {
		uv = strtoul(kv->value, NULL, 0);
		while (uv && ((uv & (0xFF << 24)) == 0)) uv <<= 8;
		cgi_postBiDiB_Dev(sock, uv);
		return -1;
	} else if ((kv = kv_lookup(hr->param, "mmsearch")) != NULL) {
		min = MIN_LOCO_ADR;
		max = MAX_MM_ADR;
		if ((kv = kv_lookup(hr->param, "min")) != NULL) {
			min = atoi(kv->value);
		}
		if ((kv = kv_lookup(hr->param, "max")) != NULL) {
			max = atoi(kv->value);
		}

		adr = mmpt_findDecoder(min, max);

		hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);
		socket_printf (sock, "{ \"mmadr\": %d, \"min\":  %d, \"max\":  %d}\n", adr, min, max);
		return -1;
	}

	log_error ("%s(): LOK parameter missing\n", __func__);
	return 1;
}

static int cgi_infoDevice (int sock, struct http_request *hr)
{
	struct key_value *kv;
	locoT *l = NULL;
	int v;

	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		v = atoi(kv->value);
		if (v > 0) l = db_getLoco(v, false);
		cgi_locoInfo(sock, l);
		return -1;
	}

	log_error ("%s(): LOK parameter missing\n", __func__);
	return 1;
}

static int cgi_cfgdecoder (int sock, struct http_request *hr)
{
	struct key_value *kv;
	ldataT *l;
	enum fmt format;
	int adr;

	if ((kv = kv_lookup(hr->param, "fmt")) != NULL) {
		format = db_string2fmt(kv->value);
	} else {
		format = FMT_UNKNOWN;
	}

	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		if ((l = loco_call(atoi(kv->value), true)) != NULL) {		// check for illegal decoder address or out of memory
			if (format != FMT_UNKNOWN) l->loco->fmt = format;
			if ((kv = kv_lookup(hr->param, "maxfunc")) != NULL) {
				l->loco->maxfunc = atoi(kv->value);
			}
			if ((kv = kv_lookup(hr->param, "uid")) != NULL) {
				l->loco->uid = strtoul(kv->value, NULL, 0);
			}
			if ((kv = kv_lookup(hr->param, "vid")) != NULL) {
				l->loco->vid = strtoul(kv->value, NULL, 0);
			}
			db_locoSanitize(l->loco);
			db_triggerStore(__func__);
		}
		cgi_postLoco(sock, l);
	} else if ((kv = kv_lookup(hr->param, "acc")) != NULL) {
		adr = atoi(kv->value);
		if (format != FMT_UNKNOWN) db_setTurnoutFmt(adr, format);
		cgi_postTurnout (sock, db_lookupTurnout(adr));
		db_triggerStore(__func__);
	} else if ((kv = kv_lookup(hr->param, "xcc")) != NULL) {
		/* no settings supported yet */
		return 1;
	}

	return -1;
}

static int cgi_cvread (int sock, struct http_request *hr)
{
	struct key_value *kv;
	ldataT *l;
	int cv, sub, count;
	flexval fv;
	cvadrT cva;

	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TESTDRIVE) return 1;
	if ((kv = kv_lookup(hr->param, "cv")) == NULL) {
		log_error ("%s(): CV parameter missing\n", __func__);
		return 1;
	}
	cv = atoi(kv->value);
	fv.i32 = sock;
	sub = 0;
	count = 1;
	if ((kv = kv_lookup(hr->param, "sub")) != NULL) sub = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "count")) != NULL) count = atoi(kv->value);

	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		if ((l = loco_call(atoi(kv->value), true)) != NULL) {		// check for illegal decoder address or out of memory
			if (FMT_IS_DCC(l->loco->fmt)) {
				if (dccpom_readByte(l->loco->adr, DECODER_DCC_MOBILE, cv, cgi_cvCallback, fv) == 0) {
					httpd_free_request(hr);
					vTaskDelete(NULL);	// end this task and hold socket open
				}
			} else if (FMT_IS_M3(l->loco->fmt)) {
				cva.m3cv = cv;
				cva.m3sub = sub;
				m3pom_readCV(l->loco->adr, cva, count, cgi_cvCallback, fv);
				httpd_free_request(hr);
				vTaskDelete(NULL);	// end this task and hold socket open
			}
		}
	} else if ((kv = kv_lookup(hr->param, "acc")) != NULL) {
		if (dccpom_readByte(atoi(kv->value), DECODER_DCC_ACC, cv, cgi_cvCallback, fv) == 0) {
			httpd_free_request(hr);
			vTaskDelete(NULL);	// end this task and hold socket open
		}
	} else if ((kv = kv_lookup(hr->param, "xcc")) != NULL) {
		return 1;
	}
	return 0;
}

static int cgi_cvwrite (int sock, struct http_request *hr)
{
	struct key_value *kv;
	ldataT *l;
	int cv, sub, val, bytes;
	char *s;
	uint8_t varray[4];
	flexval fv;
	cvadrT cva;

	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TESTDRIVE) return 1;
	if ((kv = kv_lookup(hr->param, "cv")) == NULL) {
		log_error ("%s(): CV parameter missing\n", __func__);
		return 1;
	}
	cv = atoi(kv->value);
	fv.i32 = sock;
	sub = 0;
	if ((kv = kv_lookup(hr->param, "sub")) != NULL) sub = atoi(kv->value);

	if ((kv = kv_lookup(hr->param, "val")) == NULL) {
		log_error ("%s(): VAL parameter missing\n", __func__);
		return 1;
	}
	val = atoi(kv->value);
	if (strchr(kv->value, ',')) {	// multiple, comma separated values
		s = kv->value;
		bytes = 0;
		while (*s && bytes < DIM(varray)) {
			varray[bytes++] = atoi(s);
			while (*s && *s != ',') s++;	// look for comma or terminating null
			if (*s) s++;
		}
	} else {
		varray[0] = val;
		bytes = 1;
	}
	if (val < 0 || val > 255) {
		log_error ("%s(): VAL parameter out of range\n", __func__);
		return 1;
	}

	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		if ((l = loco_call(atoi(kv->value), true)) != NULL) {		// illegal decoder address or out of memory
			if (FMT_IS_DCC(l->loco->fmt)) {
				if (dccpom_writeByte(l->loco->adr, DECODER_DCC_MOBILE, cv, val, cgi_cvCallback, fv) == 0) {
					httpd_free_request(hr);
					vTaskDelete(NULL);	// end this task and hold socket open
				} else {
					return 1;
				}
			} else if (FMT_IS_M3(l->loco->fmt)) {
				cva.m3cv = cv;
				cva.m3sub = sub;
				if (m3pom_writeCVar(l->loco->adr, cva, varray, bytes, 1, cgi_cvCallback, fv) == 0) {
					httpd_free_request(hr);
					vTaskDelete(NULL);	// end this task and hold socket open
				}
			} else {
				return 1;
			}
		}
	} else if ((kv = kv_lookup(hr->param, "acc")) != NULL) {
		if (dccpom_writeByte(atoi(kv->value), DECODER_DCC_ACC, cv, val, cgi_cvCallback, fv) == 0) {
			httpd_free_request(hr);
			vTaskDelete(NULL);	// end this task and hold socket open
		} else {
			return 1;
		}
	} else if ((kv = kv_lookup(hr->param, "xcc")) != NULL) {
		return 1;
	}
	return 1;
}

static int cgi_cvwritebit (int sock, struct http_request *hr)
{
	struct key_value *kv;
	ldataT *l;
	int cv, val, bytes, bit;
	char *s;
	flexval fv;
	uint8_t varray[4];

	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TESTDRIVE) return 1;
	if ((kv = kv_lookup(hr->param, "cv")) == NULL) {
		log_error ("%s(): CV parameter missing\n", __func__);
		return 1;
	}
	cv = atoi(kv->value);
	fv.i32 = sock;
	if ((kv = kv_lookup(hr->param, "bit")) == NULL) {
		log_error ("%s(): bit parameter missing\n", __func__);
		return 1;
	}
	bit = atoi(kv->value);

	if ((kv = kv_lookup(hr->param, "val")) == NULL) {
		log_error ("%s(): VAL parameter missing\n", __func__);
		return 1;
	}
	val = atoi(kv->value);
	if (strchr(kv->value, ',')) {	// multiple, comma separated values
		s = kv->value;
		bytes = 0;
		while (*s && bytes < DIM(varray)) {
			varray[bytes++] = atoi(s);
			while (*s && *s != ',') s++;	// look for comma or terminating null
			if (*s) s++;
		}
	} else {
		varray[0] = val;
		bytes = 1;
	}
	if (val < 0 || val > 1) {
		log_error ("%s(): VAL parameter out of range\n", __func__);
		return 1;
	}

	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		if ((l = loco_call(atoi(kv->value), true)) != NULL) {		// illegal decoder address or out of memory
			if (FMT_IS_DCC(l->loco->fmt)) {
				if (dccpom_writeBit(l->loco->adr, DECODER_DCC_MOBILE, cv, bit, val, cgi_cvCallback, fv) == 0) {
					httpd_free_request(hr);
					vTaskDelete(NULL);	// end this task and hold socket open
				} else {
					return 1;
				}
				return 1;
			}
		}
	}
	return 1;
}

static int cgi_cvshortwrite (int sock, struct http_request *hr)
{
	struct key_value *kv;
	int cv, val, bytes;
	char *s;
	uint8_t varray[2];
	flexval fv;

	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TESTDRIVE) return 1;
	if ((kv = kv_lookup(hr->param, "cv")) == NULL) {
		log_error ("%s(): CV parameter missing\n", __func__);
		return 1;
	}
	cv = atoi(kv->value);
	fv.i32 = sock;

	if ((kv = kv_lookup(hr->param, "val")) == NULL) {
		log_error ("%s(): VAL parameter missing\n", __func__);
		return 1;
	}
	val = atoi(kv->value);
	if (strchr(kv->value, ',')) {	// multiple, comma separated values
		s = kv->value;
		bytes = 0;
		while (*s && bytes < DIM(varray)) {
			varray[bytes++] = atoi(s);
			while (*s && *s != ',') s++;	// look for comma or terminating null
			if (*s) s++;
		}
	} else {
		varray[0] = val;
		bytes = 1;
	}
	if (val < 0 || val > 255) {
		log_error ("%s(): VAL parameter out of range\n", __func__);
		return 1;
	}

	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		if (dccpom_writeBytesShortForm(atoi(kv->value), DECODER_DCC_MOBILE, cv, varray, bytes, cgi_cvCallback, fv) == 0) {
			httpd_free_request(hr);
			vTaskDelete(NULL);	// end this task and hold socket open
		} else {
			return 1;
		}
	}
	return 1;
}

static int cgi_xpomwrite (int sock, struct http_request *hr)
{
	struct key_value *kv;
	int cv, val, bytes;
	char *s;
	uint8_t varray[4];
	flexval fv;

	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TESTDRIVE) return 1;
	if ((kv = kv_lookup(hr->param, "cv")) == NULL) {
		log_error ("%s(): CV parameter missing\n", __func__);
		return 1;
	}
	cv = atoi(kv->value);
	fv.i32 = sock;

	if ((kv = kv_lookup(hr->param, "val")) == NULL) {
		log_error ("%s(): VAL parameter missing\n", __func__);
		return 1;
	}
	val = atoi(kv->value);
	if (strchr(kv->value, ',')) {	// multiple, comma separated values
		s = kv->value;
		bytes = 0;
		while (*s && bytes < DIM(varray)) {
			varray[bytes++] = atoi(s);
			while (*s && *s != ',') s++;	// look for comma or terminating null
			if (*s) s++;
		}
	} else {
		varray[0] = val;
		bytes = 1;
	}
	if (val < 0 || val > 255) {
		log_error ("%s(): VAL parameter out of range\n", __func__);
		return 1;
	}

	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		if (dccxpom_writeBytes(atoi(kv->value), DECODER_DCC_MOBILE, cv, varray, bytes, cgi_cvCallback, fv) == 0) {
			httpd_free_request(hr);
			vTaskDelete(NULL);	// end this task and hold socket open
		} else {
			return 1;
		}
	}
	return 1;
}

static int cgi_pgcvread (int sock, struct http_request *hr)
{
	struct key_value *kv, *hdrs;
	ldataT *l = NULL;
	int cv, count, rc;
	cvadrT cva;
	flexval fv;

	if ((kv = kv_lookup(hr->param, "cv")) == NULL) {
		log_error ("%s(): CV parameter missing\n", __func__);
		return 1;
	}
	cv = atoi(kv->value);
	cva.m3sub = 0;
	fv.i32 = sock;
	count = 1;
	if ((kv = kv_lookup(hr->param, "sub")) != NULL) cva.m3sub = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "count")) != NULL) count = atoi(kv->value);

	rc = -1;
	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		l = loco_call(atoi(kv->value), true);			// check for illegal decoder address or out of memory
	}
	if (!l || FMT_IS_DCC(l->loco->fmt)) {
		rc = dccpt_cvReadByte(cv);
	} else if (FMT_IS_M3(l->loco->fmt)) {
		cva.m3cv = cv;
		rc = m3pt_readCV(l->loco->adr, cva, count, 1, cgi_cvCallback, fv);
		httpd_free_request(hr);
		vTaskDelete(NULL);	// end this task and hold socket open
	}
	hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	socket_printf (sock, "{ \"cv\": [ %d, %d ] }\n", cv, rc);
	return -1;
}

static int cgi_pgcvwrite (int sock, struct http_request *hr)
{
	struct key_value *kv, *hdrs;
	int cv, val, rc;

	if ((kv = kv_lookup(hr->param, "cv")) == NULL) {
		log_error ("%s(): CV parameter missing\n", __func__);
		return 1;
	}
	cv = atoi(kv->value);

	if ((kv = kv_lookup(hr->param, "val")) == NULL) {
		log_error ("%s(): VAL parameter missing\n", __func__);
		return 1;
	}
	val = atoi(kv->value);
	if (val < 0 || val > 255) {
		log_error ("%s(): VAL parameter out of range\n", __func__);
		return 1;
	}

	rc = dccpt_cvWriteByte(cv, val);
	hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	socket_printf (sock, "{ \"cv\": [ %d, %d ] }\n", cv, rc);
	return -1;
}

static int cgi_m3read (int sock, struct http_request *hr)
{
	struct key_value *kv, *hdrs;
	ldataT *l = NULL;

	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TESTDRIVE) return 1;
	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		if ((l = loco_call(atoi(kv->value), true)) != NULL) {		// check for illegal decoder address or out of memory
			if (FMT_IS_M3(l->loco->fmt)) {
				m3_readDecoder(l->loco->adr);
			}
		}
	}

	hdrs = kv_add(NULL, "Content-Length", "0");
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	return -1;
}

static int cgi_m3info (int sock, struct http_request *hr)
{
	struct key_value *kv, *hdrs;
	int rc = -99;

	if (rt.tm != TM_GO && rt.tm != TM_HALT && rt.tm != TM_TESTDRIVE) return 1;
	if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
		rc = m3_readFuncs(atoi(kv->value));
	}

	hdrs = kv_add(NULL, "Content-Length", "0");
	httpd_header(sock, (rc) ? PRECONDITION_FAILED : FILE_OK, hdrs);
	kv_free(hdrs);
	return -1;
}

static int cgi_m3name (int sock, struct http_request *hr)
{
	struct key_value *kv, *hdrs;
	char *name = NULL;
	int rc = -99;

	if ((kv = kv_lookup(hr->param, "name")) != NULL) {	// check if we are supplied with a new name
		name = kv->value;
		if ((kv = kv_lookup(hr->param, "lok")) != NULL) {	// check if we are supplied with a mobile decoder address
			rc = m3_setName(atoi(kv->value), name);
		}
	}

	hdrs = kv_add(NULL, "Content-Length", "0");
	httpd_header(sock, (rc) ? PRECONDITION_FAILED : FILE_OK, hdrs);
	kv_free(hdrs);
	return -1;
}

static int cgi_getBiDiBtrntMapping (int sock, struct http_request *hr)
{
	struct key_value *kv, *hdrs;
	struct bidibnode *n;
	struct nodefeature *ft;
	turnoutT *t;
	uint8_t uid[BIDIB_UID_LEN];
	int i, outputs;
	char *tmp, *s, *end;

	hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	if ((kv = kv_lookup(hr->param, "node")) != NULL) {
		log_msg (LOG_INFO, "%s() node='%s'\n", __func__, kv->value);
		for (i = 0; i < 5; i++) {
			uid[i] = hex_byte(&kv->value[i * 2]);	// only 5 bytes of the UID are used here (class / xclass bits are missing)
		}
		log_msg (LOG_INFO, "%s() node='%s' (%02x %02x %02x %02x %02x)\n", __func__, kv->value, uid[0], uid[1], uid[2], uid[3], uid[4]);
		tmp = "0";		// just in case ...
		if ((n = BDBnode_lookupNodeByShortUID(uid, NULL)) != NULL) {
			outputs = 0;
			if ((ft = bidib_readFeature (n, FEATURE_ACCESSORY_COUNT)) != NULL && ft->value > 0) outputs += ft->value;
			if ((ft = bidib_readFeature (n, FEATURE_CTRL_SWITCH_COUNT)) != NULL && ft->value > 0) outputs += ft->value;
			if ((ft = bidib_readFeature (n, FEATURE_CTRL_LIGHT_COUNT)) != NULL && ft->value > 0) outputs += ft->value;

			log_msg (LOG_INFO, "%s()  =>   %s (%d outputs)\n", __func__, bidib_formatUID(n->uid), outputs);
			end = s = tmp = tmp1k();
			for (i = 0; i < outputs; i++) {
				if (s > tmp) *s++ = ',';
				if ((t = db_lookupBidibTurnout(n->uid, i)) != NULL) {
					s += sprintf (s, "%d", t->adr);
					end = s;	// skip trainlling zeros
				} else {
					*s++ = '0';
					*s = 0;
					if (end == tmp) end = s;	// if no mappings are set at all, we want to give at least a single "0"
				}
			}
			*end = 0;
		}
		socket_printf (sock, "{ \"UID\": \"%s\", \"mapping\": \"%s\" }\n", kv->value, tmp);
	}

	return -1;
}

static int cgi_getBiDiBs88Mapping (int sock, struct http_request *hr)
{
	struct key_value *kv, *hdrs;
	struct sysconf *cfg;
	struct bidib_feedback *bf;
	struct bidibnode *n;
	struct nodefeature *ft;
	uint8_t uid[BIDIB_UID_LEN];
	int i;
	char tmp[16];

	cfg = cnf_getconfig();
	bf = cfg->bidibfb;
	hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	if ((kv = kv_lookup(hr->param, "node")) != NULL) {		// query a single node
		log_msg (LOG_INFO, "%s() node='%s'\n", __func__, kv->value);
		for (i = 0; i < 5; i++) {
			uid[i] = hex_byte(&kv->value[i * 2]);	// only 5 bytes of the UID are used here (class / xclass bits are missing)
		}
		log_msg (LOG_INFO, "%s() node='%s' (%02x %02x %02x %02x %02x)\n", __func__, kv->value, uid[0], uid[1], uid[2], uid[3], uid[4]);
		while (bf) {
			if (!memcmp(uid, &bf->uid[2], BIDIB_UID_LEN - 2)) break;
			bf = bf->next;
		}
		if (bf) {
			if ((n = BDBnode_lookupNodeByShortUID(uid, NULL)) != NULL) {
				if ((ft = bidib_readFeature (n, FEATURE_BM_SIZE)) != NULL && ft->value > 0) {
					log_msg (LOG_INFO, "%s()  =>   %s:%d (%d outputs)\n", __func__, bidib_formatUID(n->uid), bf->s88base, ft->value);
				} else {
					log_msg (LOG_INFO, "%s()  =>   %s:%d (FEATURE_BM_SIZE not found)\n", __func__, bidib_formatUID(n->uid), bf->s88base);
				}
			} else {
				log_msg (LOG_INFO, "%s()  =>   %s:%d (currently not connected)\n", __func__, bidib_formatUID(bf->uid), bf->s88base);
			}
		}
		socket_printf (sock, "{ \"UID\": \"%s\", \"s88\": %d }\n", kv->value, (bf) ? bf->s88base : -1);
	} else {												// output a list of all mappings
		socket_printf (sock, "{ \"s88map\": [\n");
		while (bf) {
			log_msg (LOG_INFO, "%s()  =>   %s:%d\n", __func__, bidib_formatUID(bf->uid), bf->s88base);
			sprintf (tmp, "%02x%02x%02x%02x%02x", bf->uid[2], bf->uid[3], bf->uid[4], bf->uid[5], bf->uid[6]);
			socket_printf (sock, "{ \"UID\": \"%s\", \"s88\": %d }%s\n", tmp, bf->s88base, (bf->next) ? "," : "");
			bf = bf->next;
		}
		socket_printf (sock, "] }\n");
	}

	return -1;
}

static const struct cgiquery queries[] = {
	{ "get", cgi_getDevice },			// get decoder (loco) information and control (including refresh-info)
	{ "info", cgi_infoDevice },			// get decoder (loco) information without refresh-info or pulling the loco into refresh list
	{ "cfg", cgi_cfgdecoder },			// configure a decoder
	{ "cvread", cgi_cvread },			// On-Track CV reading from decoder
	{ "cvwrite", cgi_cvwrite },			// On-Track CV writing to decoder
	{ "cvwritebit", cgi_cvwritebit },	// On-Track CV writing of one bit to decoder
	{ "cvshortwrite", cgi_cvshortwrite },	// On-Track CV writing to decoder short form for CV31/32 and CV17/18/29
	{ "xpomwrite", cgi_xpomwrite },		// On-Track CV writing (XPOM) to decoder
	{ "pgcvread", cgi_pgcvread },		// Programming-Track CV reading from decoder
	{ "pgcvwrite", cgi_pgcvwrite },		// Programming-Track CV writing to decoder
	{ "m3read", cgi_m3read },			// On-Track reading a M3 decoder (lots of configuration)
	{ "m3info", cgi_m3info },			// On-Track reading a M3 decoder (only function icons and name)
	{ "m3name", cgi_m3name },			// write a loco name to the m3 decoder
	{ "BiDiMapping", cgi_getBiDiBtrntMapping },	// map accessory numbers to BiDiB outputs
	{ "BiDis88", cgi_getBiDiBs88Mapping },	// map BiDiB inputs to s88 system
	{ NULL, NULL }
};

static int cgi_query (int sock, struct http_request *hr, const char *rest, int sz)
{
	Y_LOFF_T total, avail, used;
	int percent, rc, module, count;
	struct key_value *kv, *hdrs;
	uint32_t uid;
	const struct cgiquery *p;
	json_valT *root, *obj;
	json_itmT *itm;
	json_stackT *jstk;
	enum fmt f;

	(void) rest;
	(void) sz;

	if ((kv = kv_lookup(hr->param, "cmd")) != NULL) {
		p = queries;
		rc = 1;
		while (p->cmd != NULL) {
			if (!strcasecmp(p->cmd, kv->value)) {
				rc = p->func(sock, hr);
				break;
			}
			p++;
		}
		// if the function returns a positive value, we must supply an answer to the caller
		if (rc >= 0) {
			hdrs = kv_add(NULL, "Content-Length", "0");
			httpd_header(sock, (rc == 0) ? FILE_OK : FILE_NOT_FOUND, hdrs);
			kv_free(hdrs);
		}
		return 0;
	} else if ((kv = kv_lookup(hr->param, "info")) != NULL) {
		hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);
		total = yaffs_totalspace("/");
		avail = yaffs_freespace("/");
		used = total - avail;
		percent = (int) ((used * 10000) / total);
		root = json_addObject(NULL);	// create root object
		jstk = json_pushObject(NULL, root);
		json_addUintItem(jstk, "flashMax", total / 1024);
		json_addIntItem(jstk, "flashPercent", percent);
		json_addUintItem(jstk, "ramMax", rt.totalHeap / 1024);
		percent = (int) (((rt.totalHeap - xPortGetFreeHeapSize()) * 10000LL) / rt.totalHeap);
		json_addIntItem(jstk, "ramPercent", percent);
		itm = json_addArrayItem(jstk, "infos");
		jstk = json_pushArray(jstk, itm);
		json_addFormatStringValue(jstk, "V%x.%x", hwinfo->HW >> 4, hwinfo->HW & 0x0F);
		json_addStringValue(jstk, SOFT_VERSION);
		json_addUintValue(jstk, hwinfo->serial);
		json_addFormatStringValue(jstk, "%lu.%lu.%lu.%lu",
			(rt.en->ip_addr.addr >> 0) & 0xFF, (rt.en->ip_addr.addr >> 8) & 0xFF,
			(rt.en->ip_addr.addr >> 16) & 0xFF, (rt.en->ip_addr.addr >> 24) & 0xFF);
		json_addFormatStringValue(jstk, "%02X:%02X:%02X:%02X:%02X:%02X",
				rt.en->hwaddr[0], rt.en->hwaddr[1],	rt.en->hwaddr[2], rt.en->hwaddr[3], rt.en->hwaddr[4], rt.en->hwaddr[5]);
		jstk = json_pop(jstk);
		itm = json_addArrayItem(jstk, "m3station");
		jstk = json_pushArray(jstk, itm);
		json_addUintValue(jstk, sig_getM3Beacon());
		json_addUintValue(jstk, sig_getM3AnnounceCounter());
		cgi_sendJSON(sock, root);
		json_free(root);
		json_popAll(jstk);
		return 0;
	} else if ((kv = kv_lookup(hr->param, "m3search")) != NULL) {
		hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);
		uid = m3pt_getUID();
		root = json_addObject(NULL);	// create root object
		jstk = json_pushObject(NULL, root);
		json_addUintItem(jstk, "m3uid", uid);
		cgi_sendJSON(sock, root);
		json_free(root);
		json_popAll(jstk);
		return 0;
	} else if ((kv = kv_lookup(hr->param, "tracklimits")) != NULL) {
		hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);
		socket_sendstring (sock, ts_getRanges());
		return 0;
	} else if ((kv = kv_lookup(hr->param, "turnoutlimits")) != NULL) {
		hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);
		socket_sendstring (sock, trnt_getRanges());
		return 0;
	} else if ((kv = kv_lookup(hr->param, "boosterlimits")) != NULL) {
		hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);
		socket_sendstring (sock, cnf_getBoosterLimits());
		return 0;
	} else if ((kv = kv_lookup(hr->param, "s88query")) != NULL) {
#ifdef CENTRAL_FEEDBACK
		hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);
		module = atoi(kv->value);
		count = 1;
		if ((kv = kv_lookup(hr->param, "count")) != NULL) count = atoi(kv->value);
		root = json_addObject(NULL);	// create root object
		jstk = json_pushObject(NULL, root);
		json_addIntItem(jstk, "module", module);
		itm = json_addArrayItem(jstk, "occupy");
		jstk = json_pushArray(jstk, itm);
		for (; count > 0; module++, count--) {
			json_addIntValue(jstk, fb_getModuleState(module));
		}
		cgi_sendJSON(sock, root);
		json_free(root);
#else
		s88_triggerUpdate();
		hdrs = kv_add(NULL, "Content-Length", "0");
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);
#endif
		return 0;
	} else if ((kv = kv_lookup(hr->param, "locoformats")) != NULL) {
		hdrs = kv_add(NULL, "Content-Type", CONTENT_JSON);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);

		root = json_addObject(NULL);	// create root object
		jstk = json_pushObject(NULL, root);
		itm = json_addArrayItem(jstk, "formats");
		jstk = json_pushArray(jstk, itm);
		for (f = FMT_MM1_14; f <= FMT_DCC_SDF; f++) {
			obj = json_addObject(jstk);
			jstk = json_pushObject(jstk, obj);
			json_addIntItem(jstk, "fmtid", f);
			json_addStringItem(jstk, "fmt", db_fmt2string(f));
			jstk = json_pop(jstk);
		}
		cgi_sendJSON(sock, root);
		json_free(root);
		return 0;
	}

	hdrs = kv_add(NULL, "Content-Length", "0");
	httpd_header(sock, FILE_NOT_FOUND, hdrs);
	kv_free(hdrs);
	return 0;
}

static int cgi_go (int sock, struct http_request *hr)
{
	(void) sock;
	(void) hr;

	sig_setMode(TM_GO);
	return 0;
}

static int cgi_stop (int sock, struct http_request *hr)
{
	(void) sock;
	(void) hr;

	sig_setMode(TM_STOP);
	return 0;
}

static int cgi_testdrv (int sock, struct http_request *hr)
{
	(void) sock;
	(void) hr;

	log_msg (LOG_WARNING, "%s(): aktiviere ProgGleis\n", __func__);
	sig_setMode(TM_TESTDRIVE);
	return 0;
}

static int cgi_reboot (int sock, struct http_request *hr)
{
	(void) sock;
	(void) hr;

	reboot();
	return 0;
}

static int cgi_m3adr (int sock, struct http_request *hr)
{
	struct key_value *kv;
	uint32_t uid;
	int loco;

	(void) sock;

	if ((kv = kv_lookup(hr->param, "uid")) == NULL) return -1;
	uid = strtoul(kv->value, NULL, 0);
	if ((kv = kv_lookup(hr->param, "lok")) == NULL) return -2;
	loco = atoi(kv->value);

	m3pt_setAddress(uid, loco);
	return 0;
}

static int cgi_m3assign (int sock, struct http_request *hr)
{
	struct key_value *kv;
	uint32_t uid;
	int loco;

	(void) sock;

	if ((kv = kv_lookup(hr->param, "uid")) == NULL) return -1;
	uid = strtoul(kv->value, NULL, 0);
	if ((kv = kv_lookup(hr->param, "lok")) == NULL) return -2;
	loco = atoi(kv->value);

	log_error ("%s() adr: %d, uid: 0x%lx\n", __func__, loco, uid);
	m3_setAddress(uid, loco);
	return 0;
}

static int cgi_m3beacon (int sock, struct http_request *hr)
{
	struct key_value *kv;
	uint32_t m3station;
	uint16_t m3announce;

	(void) sock;

	if ((kv = kv_lookup(hr->param, "m3station")) == NULL) return -1;
	m3station = strtoul(kv->value, NULL, 0);
	if ((kv = kv_lookup(hr->param, "m3announce")) == NULL) return -2;
	m3announce = (uint16_t) strtoul(kv->value, NULL, 0);

	sig_setM3Beacon(m3station, m3announce);
	return 0;
}

static int cgi_webupdate (int sock, struct http_request *hr)
{
	(void) sock;
	(void) hr;

	webup_update("/uploads/html.cpio");
	return 0;
}

static int cgi_dispatch (int sock, struct http_request *hr)
{
	struct key_value *kv;
	int loco;

	(void) sock;

	if ((kv = kv_lookup(hr->param, "lok")) == NULL) return -1;
	loco = atoi(kv->value);

	ln_dispatchLoco(loco);
	return 0;
}

static int cgi_removeLoco (int sock, struct http_request *hr)
{
	struct key_value *kv;
	locoT *l;
	int loco;

	(void) sock;

	if ((kv = kv_lookup(hr->param, "lok")) == NULL) return -1;
	loco = atoi(kv->value);

	if ((l = db_getLoco(loco, false)) != NULL) {
		db_removeLoco (l);
		log_msg(LOG_WARNING, "%s(): remove loco %d\n", __func__, loco);
	}
	return 0;
}

static int cgi_syscfg (int sock, struct http_request *hr)
{
	struct sysconf *sc;
	struct key_value *kv;

	(void) sock;

	sc = cnf_getconfig();

	if ((kv = kv_lookup(hr->param, "bidibacclogic")) != NULL) {
		if (atoi(kv->value)) sc->sysflags |= SYSFLAG_ACC_LOGICAL;
		else sc->sysflags &= ~SYSFLAG_ACC_LOGICAL;
		event_fire (EVENT_PROTOCOL, 0, NULL);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "purge")) != NULL) {
		sc->locopurge = atoi(kv->value);
		event_fire (EVENT_PROTOCOL, 0, NULL);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "sniffer")) != NULL) {
		ui32DisplayFilter = atoi(kv->value);
		if (_RC_) log_enable(LOG_RAILCOM);
		else log_disable(LOG_RAILCOM);
		event_fire (EVENT_SNIFFER, 0, NULL);
	}
	if ((kv = kv_lookup(hr->param, "locked")) != NULL) {
		if(atoi(kv->value) == 1) rt.ctrl |= EXTCTRL_LOCKED;
		else rt.ctrl &= ~EXTCTRL_LOCKED;
		event_fire (EVENT_EXTCONTROL, rt.ctrl, NULL);
	}
	if ((kv = kv_lookup(hr->param, "lighteffect")) != NULL) {
		if (atoi(kv->value) == 1) sc->sysflags |= SYSFLAG_LIGHTEFFECTS;
		else sc->sysflags &= ~SYSFLAG_LIGHTEFFECTS;
		if (atoi(kv->value) == 2) sc->sysflags |= SYSFLAG_LIGHTSOFF;
		else sc->sysflags &= ~SYSFLAG_LIGHTSOFF;
		event_fire (EVENT_LIGHTS, 0, NULL);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "s88Mod")) != NULL) s88_setModules (atoi(kv->value));
	if ((kv = kv_lookup(hr->param, "canMod")) != NULL) can_setModules (atoi(kv->value));
	if ((kv = kv_lookup(hr->param, "lnetMod")) != NULL) lnet_setModules (atoi(kv->value));
	if ((kv = kv_lookup(hr->param, "s88Freq")) != NULL) s88_setFrequency(atoi(kv->value));
	if ((kv = kv_lookup(hr->param, "startstate")) != NULL) {
		if (atoi(kv->value) == 1) sc->sysflags |= SYSFLAG_STARTSTATE;
		else sc->sysflags &= ~SYSFLAG_STARTSTATE;
		log_msg (LOG_DEBUG, "%s STARTSTATE: %d\n", __func__,!!(sc->sysflags & SYSFLAG_STARTSTATE));
		event_fire (EVENT_ENVIRONMENT, 0, NULL);
		cnf_triggerStore(__func__);
	}

	return 0;
}

static void _cgi_railcomFeature (struct bidibnode *n)
{
	struct fmtconfig *fc;
	struct nodefeature *nf;

	fc = cnf_getFMTconfig();
	if ((nf = bidib_readFeature (n, FEATURE_BST_CUTOUT_AVAILABLE)) == NULL) return;	// feature is not known
	if (nf->value == 0) return;														// feature shows, that cutout is not supported
	if ((nf = bidib_readFeature (n, FEATURE_BST_CUTOUT_ON)) == NULL) return;		// feature is not known
	if (!!nf->value == !!(fc->sigflags & SIGFLAG_RAILCOM)) return;					// feature shows, that the setting is already correct
	bidib_setFeature (n, FEATURE_BST_CUTOUT_ON, (fc->sigflags & SIGFLAG_RAILCOM) ? 1 : 0);
}

static void _cgi_railcomSwitch (bool on)
{
	dccpom_boosterConf(DCC_MANUFACTURER_TAMS, (on) ? 51 : 52);
	BDBnode_iterate(_cgi_railcomFeature);
}

static int cgi_fmtcfg (int sock, struct http_request *hr)
{
	struct fmtconfig *fc;
	struct key_value *kv;

	(void) sock;

	fc = cnf_getFMTconfig();

	if ((kv = kv_lookup(hr->param, "minswitchtime")) != NULL) trnt_setMinTime(atoi(kv->value));
	if ((kv = kv_lookup(hr->param, "maxswitchtime")) != NULL) trnt_setMaxTime(atoi(kv->value));
	if ((kv = kv_lookup(hr->param, "accrepeats")) != NULL) {
		fc->accrepeat = atoi(kv->value);
		event_fire (EVENT_ACCESSORY, 0, NULL);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "deflproto")) != NULL) {
		db_getLoco(0, false)->fmt = atoi(kv->value);
		db_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "defaproto")) != NULL) {
		db_setTurnoutFmt (0, atoi(kv->value) ? TFMT_DCC : TFMT_MM);
		event_fire (EVENT_ACCESSORY, 0, NULL);
	}
// /cgi/command?dcca=1&railcom=1&preamblebits=16&bit0length=200&bit1length=116&addrrep=3&addrrepMM=3&addrrepm3=3&pomrep=3&mmpause=1500&m3=1
	if ((kv = kv_lookup(hr->param, "dcca")) != NULL) {
		if (atoi(kv->value)) fc->sigflags |= SIGFLAG_DCCA;
		else fc->sigflags &= ~SIGFLAG_DCCA;
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "railcom")) != NULL) {
		if (atoi(kv->value)) fc->sigflags |= SIGFLAG_RAILCOM;
		else fc->sigflags &= ~SIGFLAG_RAILCOM;
		cnf_triggerStore(__func__);
		_cgi_railcomSwitch(fc->sigflags & SIGFLAG_RAILCOM);
	}
	if ((kv = kv_lookup(hr->param, "dcclong")) != NULL) {
		if (atoi(kv->value)) fc->sigflags |= SIGFLAG_DCC_LONG_ADR;
		else fc->sigflags &= ~SIGFLAG_DCC_LONG_ADR;
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "nop")) != NULL) {
		if (atoi(kv->value)) fc->sigflags |= SIGFLAG_DCCNOP;
		else fc->sigflags &= ~SIGFLAG_DCCNOP;
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "m3")) != NULL) {
		if (atoi(kv->value)) fc->sigflags |= SIGFLAG_M3ENABLED;
		else fc->sigflags &= ~SIGFLAG_M3ENABLED;
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "preamblebits")) != NULL) {
		fc->dcc.preamble = atoi(kv->value);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "bit0length")) != NULL) {
		fc->dcc.tim_zero = atoi(kv->value);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "bit1length")) != NULL) {
		fc->dcc.tim_one = atoi(kv->value);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "addrrep")) != NULL) {
		fc->dcc.repeat = atoi(kv->value);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "addrrepMM")) != NULL) {
		fc->mm.repeat = atoi(kv->value);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "addrrepm3")) != NULL) {
		fc->m3.repeat = atoi(kv->value);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "accrepeats")) != NULL) {
		fc->accrepeat = atoi(kv->value);
		event_fire (EVENT_ACCESSORY, 0, NULL);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "pomrep")) != NULL) {
		fc->dcc.pomrepeat = atoi(kv->value);
		cnf_triggerStore(__func__);
	}
	if ((kv = kv_lookup(hr->param, "mmpause")) != NULL) {
		fc->mm.pause = atoi(kv->value);
		cnf_triggerStore(__func__);
	}

	if (!(fc->sigflags & SIGFLAG_RAILCOM)) fc->sigflags &= ~SIGFLAG_DCCA;		// no DCC-A without RailCom

	event_fire (EVENT_PROTOCOL, db_getLoco(0, false)->fmt, NULL);	// we can do this in general, because most changes will require it anyways
	return 0;
}

static int cgi_setBiDiBtrntMapping (struct key_value *kv)
{
	struct bidibnode *n;
	struct nodefeature *ft;
	turnoutT *t;
	uint8_t uid[BIDIB_UID_LEN];
	int i, trnt, outputs;
	char *s;
	bool changed;

	if (!kv) return 0;
	log_msg (LOG_INFO, "%s() node='%s'\n", __func__, kv->value);
	for (i = 0; i < 5; i++) {
		uid[i + 2] = hex_byte(&kv->value[i * 2]);	// only 5 bytes of the UID are used here (class / xclass bits are ignored and were not sent)
	}
	log_msg (LOG_INFO, "%s() node='%s' (%02x %02x %02x %02x %02x)\n", __func__, kv->value, uid[2], uid[3], uid[4], uid[5], uid[6]);

	// 1. step: delete all previously defined mappings for this UID
	// this can be done, even if the node is currently not in the system (so no need to have a valid node pointer)!
	changed = db_clearBidibTurnout(uid);

	// 2. step: define new mappings, but only if this node is really there
	if ((n = BDBnode_lookupNodeByShortUID(&uid[2], NULL)) != NULL) {
		outputs = 0;
		if ((ft = bidib_readFeature (n, FEATURE_ACCESSORY_COUNT)) != NULL && ft->value > 0) outputs += ft->value;
		if ((ft = bidib_readFeature (n, FEATURE_CTRL_SWITCH_COUNT)) != NULL && ft->value > 0) outputs += ft->value;
		if ((ft = bidib_readFeature (n, FEATURE_CTRL_LIGHT_COUNT)) != NULL && ft->value > 0) outputs += ft->value;

		if (outputs > 0) {
			log_msg (LOG_INFO, "%s()  =>   %s (%d outputs)\n", __func__, bidib_formatUID(n->uid), outputs);
			i = 0;
			s = kv->value;
			while (i < outputs && *s && (s = strchr(s, ',')) != NULL) {
				s++;
				if ((trnt = atoi(s)) > 0 && (t = db_getTurnout(trnt)) != NULL) {
					log_msg (LOG_INFO, "%s() T %d => aspect %d\n",__func__, t->adr, i);
					if (t->fmt != TFMT_BIDIB || memcmp(t->uid, n->uid, BIDIB_UID_LEN) || t->aspect != i) {
						t->fmt = TFMT_BIDIB;
						memcpy (t->uid, n->uid, sizeof(t->uid));
						t->aspect = i;
						changed = true;
					}
				}
				i++;
			}
		}
	}
	if (changed) db_triggerStore(__func__);

	return 0;
}

static int cgi_setBiDiBs88Mapping (struct key_value *root)
{
	struct key_value *kv;
	uint8_t uid[BIDIB_UID_LEN];
	char string[16];
	int i;

	if (!root) return 0;
	if ((kv = kv_lookup(root, "node")) == NULL) return -1;
	sprintf (string, "0000000000");
	sprintf (&string[10 - strlen(kv->value)], "%s", kv->value);
	log_msg (LOG_INFO, "%s() '%s' -> '%s'\n", __func__, kv->value, string);

	uid[0] = BIDIB_CLASS_OCCUPANCY;
	uid[1] = 0;
	for (i = 0; i < BIDIB_UID_LEN - 2; i++) {
		uid[i + 2] = hex_byte(&string[i * 2]);	// only 5 bytes of the UID are used here (class / xclass bits are missing)
	}

	if ((kv = kv_lookup(root, "s88base")) != NULL) {	// should never fail, because we came here after finding it in the list of params
		if (kv->value && *kv->value) {
			bidib_addFBmap(uid, atoi(kv->value));
		} else {
			bidib_dropFBmap(uid);
		}
		bidib_store();
	}
	return 0;
}

static int cgi_bidib (int sock, struct http_request *hr)
{
	struct key_value *kv;
	bidibmsg_t *m;
	uint32_t adr;
	uint8_t data;

	(void) sock;

	if ((kv = kv_lookup(hr->param, "resNode")) != NULL) {
		adr = atoi(kv->value);
		if (adr > 0) {
			m = bidib_genMessage(BDBnode_lookupNode(bidib_num2stack(adr)), MSG_SYS_RESET, 0, NULL);
			if (m) BDBnode_downlink(NULL, m);
		}
	}
	if ((kv = kv_lookup(hr->param, "identifyOn")) != NULL) {
		adr = atoi(kv->value);
		if (adr > 0) {
			data = 1;
			m = bidib_genMessage(BDBnode_lookupNode(bidib_num2stack(adr)), MSG_SYS_IDENTIFY, 1, &data);
			if (m) BDBnode_downlink(NULL, m);
		} else if (adr == 0) bidib_identify(true);
	}
	if ((kv = kv_lookup(hr->param, "identifyOff")) != NULL) {
		adr = atoi(kv->value);
		if (adr > 0) {
			data = 0;
			m = bidib_genMessage(BDBnode_lookupNode(bidib_num2stack(adr)), MSG_SYS_IDENTIFY, 1, &data);
			if (m) BDBnode_downlink(NULL, m);
		} else if (adr == 0) bidib_identify(false);
	}
	if((kv = kv_lookup(hr->param, "mapping")) != NULL) {
		log_msg (LOG_INFO, "%s(): Params: %s\n", __func__, kv->value);
		return cgi_setBiDiBtrntMapping(kv);
	}
	if((kv = kv_lookup(hr->param, "s88base")) != NULL) {
		return cgi_setBiDiBs88Mapping(hr->param);
	}
	return 0;
}

static int cgi_mmprog (int sock, struct http_request *hr)
{
	struct key_value *kv;
	int adr, cv, val;

	(void) sock;

	cv = val = adr = -1;
	if ((kv = kv_lookup(hr->param, "adr")) != NULL) adr = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "val")) != NULL) val = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "cv")) != NULL) cv = atoi(kv->value);
	if (cv >= 0 && val >= 0) return mmpt_cvProg(adr, cv, val);
	return mmpt_enterProgram(adr);
}

static int cgi_loco (int adr, int sock, struct http_request *hr)
{
	struct key_value *kv;
	char direction, *s, *delim;
	int speed, func, icon, timing;
	uint32_t vid, uid, newfuncs;
	ldataT *loco;
	locoT *l;
	bool on;

	(void) sock;
	(void) hr;

	if (adr <= 0) return -1;
	direction = 0;
	speed = func = -1;
	vid = uid = 0;

	if ((kv = kv_lookup(hr->param, "speed")) != NULL && kv->value && (strlen(kv->value) >= 2)) {
		direction = *kv->value;
		speed = atoi(kv->value + 1);
		if (speed >= 0 && (direction == 'F' || direction == 'R')) {
			speed &= 0x7F;
			if (direction == 'F' || direction == 'f') speed |= 0x80;
			rq_setSpeed(adr, speed);
		}
	}
	if ((kv = kv_lookup(hr->param, "fu")) != NULL && kv->value && (strlen(kv->value) >= 2)) {
		if (*kv->value == '0') on = false;
		if (*kv->value == '1') on = true;
		func = atoi(kv->value + 1);
		if (func >= 0 && func <= 31) {
			loco = loco_call(adr, true);
			if (loco) newfuncs = loco->funcs[0];
			else newfuncs = 0;
			if (on) newfuncs |= (1 << func);
			else newfuncs &= ~(1 << func);
			rq_setFuncMasked(adr, newfuncs, (1 << func));
		} else {
			if (func >= 32) loco_setFunc(adr, func, on);
		}
	}

	if ((kv = kv_lookup(hr->param, "vid")) != NULL) vid = strtoul(kv->value, NULL, 0);
	if ((kv = kv_lookup(hr->param, "uid")) != NULL) uid = strtoul(kv->value, NULL, 0);
	if (uid) {
		if ((l = db_findLocoUID(vid, uid)) != NULL) {		// the loco with this VID/UID should get a new loco address
			db_changeAdr (adr, vid, uid);
		} else {											// we will store the given VID/UID for this loco
			db_setLocoVID(adr, vid);
			db_setLocoUID(adr, uid);
		}
	}

	if ((kv = kv_lookup(hr->param, "name")) != NULL) db_setLocoName(adr, kv->value);
	if ((kv = kv_lookup(hr->param, "fmt")) != NULL) db_setLocoFmt(adr, atoi(kv->value));
	if ((kv = kv_lookup(hr->param, "maxfunc")) != NULL) db_setLocoMaxfunc(adr, atoi(kv->value));
	if ((kv = kv_lookup(hr->param, "fuico")) != NULL && kv->value) {
		// the string is constructed as a space-delimited list of "<func>|<icon>|<timing>" with each
		// element beeing an integer number.
		if ((l = db_getLoco(adr, false)) != NULL) {
			s = kv->value;
			while (*s) {
				delim = s;
				while (*delim && !isspace(*delim)) delim++;
				if (*delim) *delim++ = 0;

				// now parse s
				func = atoi(s);
				while (*s && *s != '|') s++;
				if (*s == '|') s++;
				icon = atoi(s);
				while (*s && *s != '|') s++;
				if (*s == '|') s++;
				timing = atoi(s);
				db_locoFuncIcon (l, func, icon);
				db_locoFuncTiming (l, func, timing);

				s = delim;
				while (*s && isspace(*s)) s++;
			}
		}
	}

	return 0;
}

static int cgi_acc (struct key_value *kv, int sock, struct http_request *hr)
{
	struct key_value *kvtmp;
	int adr;
	char ctrl;

	(void) sock;
	(void) hr;

	if (!kv || !hr) return 0;

	if (!strcasecmp ("acc", kv->key)) {		// "?acc=<adr>&dir=<ctrl>"
		adr = atoi(kv->value);
		if ((kvtmp = kv_lookup(hr->param, "dir")) == NULL) return 0;
		ctrl = kvtmp->value[0];
	} else {								// "?w=<ctrl><adr>"
		if (strlen(kv->value) < 2) return 0;
		ctrl = kv->value[0];
		adr = atoi(&kv->value[1]);
	}

	switch (ctrl) {
		case 'g':		// german: geradeaus oder "grün" / english: "green"
		case 'G':
		case 's':		// english: straight
		case 'S':
		case '0':
			trnt_switchTimed(adr, false, 1);
			break;
		case 'r':		// german: rund (für Abbiegen) oder "rot" / english: "red"
		case 'R':
		case 't':		// english: thrown
		case 'T':
		case '1':
			trnt_switchTimed(adr, true, 1);
			break;
	}
	return 0;
}

/**
 * Settings for booster. Multiple booster commands may be contained
 * in a single call, so parameters are position dependent.
 *
 * \param sock		the socket over which this request came in (unused here)
 * \param hr		the request header containing all the parameters
 * \return			currently always 0
 */
static int cgi_booster(int sock, struct http_request *hr)
{
	struct key_value *kv;
	struct sysconf *sc;
	int booster = -1;
	int val;

	(void) sock;

	sc = cnf_getconfig();
	kv = hr->param;

	while (kv) {
		if (!strcasecmp("booster", kv->key)) {
			if (!strcasecmp("intern", kv->value)) booster = BOOSTER_BUILTIN;
			else if (!strcasecmp("mm", kv->value)) booster = BOOSTER_MM;
			else if (!strcasecmp("dcc", kv->value)) booster = BOOSTER_CDE;
			else if (!strcasecmp("bidib", kv->value)) booster = BOOSTER_BIDIB;
		} else if (!strcasecmp("vtrack", kv->key)) {
			if (booster == BOOSTER_BUILTIN) {
				ts_setVoltage(atoi(kv->value) * 10);
				cnf_triggerStore(__func__);
			}
		} else if (!strcasecmp("ptrack", kv->key)) {
			if (booster == BOOSTER_BUILTIN) {
				ts_setPtVoltage(atoi(kv->value));
				cnf_triggerStore(__func__);
			}
		} else if (!strcasecmp("itrack", kv->key)) {
			if (booster == BOOSTER_BUILTIN) {
				ts_setCurrent(atoi(kv->value));
				cnf_triggerStore(__func__);
			}
		} else if (!strcasecmp("sens", kv->key)) {
			switch (booster) {
				case BOOSTER_BUILTIN:		// internal booster
					ts_setSensitivity(atoi(kv->value));
					cnf_triggerStore(__func__);
					break;
				case BOOSTER_MM:		// MM booster
					val = atoi(kv->value);
					if (val < EXTERNSHORT_MIN) val = EXTERNSHORT_MIN;
					if (val > EXTERNSHORT_MAX) val = EXTERNSHORT_MAX;
					sc->mmshort = val;
					cnf_triggerStore(__func__);
					break;
				case BOOSTER_CDE:		// DCC booster
					val = atoi(kv->value);
					if (val < EXTERNSHORT_MIN) val = EXTERNSHORT_MIN;
					if (val > EXTERNSHORT_MAX) val = EXTERNSHORT_MAX;
					sc->dccshort = val;
					cnf_triggerStore(__func__);
					break;
			}
		} else if (!strcasecmp("inrush", kv->key)) {
			if (booster == BOOSTER_BUILTIN) {
				ts_setInrush(atoi(kv->value));
				cnf_triggerStore(__func__);
			}
		} else if (!strcasecmp("global", kv->key)) {
			if (atoi(kv->value)) sc->sysflags |= SYSFLAG_GLOBAL_BIDIB_SHORT;
			else sc->sysflags &= ~SYSFLAG_GLOBAL_BIDIB_SHORT;
			cnf_triggerStore(__func__);
		} else if (!strcasecmp("remote", kv->key)) {
			if (atoi(kv->value)) sc->sysflags |= SYSFLAG_BIDIB_ONOFF;
			else sc->sysflags &= ~SYSFLAG_BIDIB_ONOFF;
			cnf_triggerStore(__func__);
		} else if (!strcasecmp("route", kv->key)) {
			switch (booster) {
				case BOOSTER_BUILTIN:
					if (!atoi(kv->value)) sc->sysflags |= SYSFLAG_NOMAGONMAINBST;
					else sc->sysflags &= ~SYSFLAG_NOMAGONMAINBST;
					cnf_triggerStore(__func__);
					break;
				case BOOSTER_MM:
					if (!atoi(kv->value)) sc->sysflags |= SYSFLAG_NOMAGONMKLNBST;
					else sc->sysflags &= ~SYSFLAG_NOMAGONMKLNBST;
					cnf_triggerStore(__func__);
					break;
				case BOOSTER_CDE:
					if (!atoi(kv->value)) sc->sysflags |= SYSFLAG_NOMAGONCDEBST;
					else sc->sysflags &= ~SYSFLAG_NOMAGONCDEBST;
					cnf_triggerStore(__func__);
					break;
			}
		}
		kv = kv->next;
	}

	event_fire(EVENT_BOOSTER, 0, NULL);
	return 0;
}

static const struct cgiquery commands[] = {
	{ "go", cgi_go },				// set trackmode GO
	{ "stop", cgi_stop },			// set trackmode STOP
	{ "testdrv", cgi_testdrv },		// set trackmode TESTDRV
	{ "reboot", cgi_reboot },		// reboot mc2
	{ "m3adr", cgi_m3adr },			// assign SID to m3 loco on programming track
	{ "m3assign", cgi_m3assign },	// assign SID to m3 loco on main track
	{ "m3beacon", cgi_m3beacon },	// specify the m3 beacon (station address and announce counter)
	{ "webupdate", cgi_webupdate },	// update HTML folder (WEB site contents) from flashfile
	{ "dispatch", cgi_dispatch },	// dipatch a LocoNet loco
	{ "removeLoco", cgi_removeLoco },	// remove a loco from loco DB
	{ "syscfg", cgi_syscfg },		// configure system behavioral aspects
	{ "fmtcfg", cgi_fmtcfg },		// configure signal format aspects
	{ "bidib", cgi_bidib },			// handle BiDiB subsystem
	{ "mmprog", cgi_mmprog },		// programming track for MM locos
	{ NULL, NULL }
};

static int cgi_command (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv, *hdrs;
	const struct cgiquery *p;
	int loco;
	locoT *l;

	(void) rest;
	(void) sz;

	if ((kv = kv_lookup(hr->param, "cmd")) != NULL) {
//		log_msg (LOG_INFO, "%s(): cmd='%s'\n", __func__, kv->value);
		p = commands;
		while (p->cmd != NULL) {
			if (!strcasecmp(p->cmd, kv->value)) {
				p->func(sock, hr);
				break;
			}
			p++;
		}
	} else {
		if ((kv = kv_lookup(hr->param, "lok")) != NULL) {
			loco = atoi(kv->value);
			cgi_loco(loco, sock, hr);
		} else if ((kv = kv_lookup(hr->param, "w")) != NULL) {
			cgi_acc(kv, sock, hr);
		} else if ((kv = kv_lookup(hr->param, "acc")) != NULL) {	// same as "w"
			cgi_acc(kv, sock, hr);
		} else if ((kv = kv_lookup(hr->param, "booster")) != NULL) {
			cgi_booster(sock, hr);
		} else if ((kv = kv_lookup(hr->param, "removeLoco")) != NULL) {
			if ((l = db_getLoco(atoi(kv->value), false)) != NULL) {
				db_removeLoco (l);
				log_msg(LOG_WARNING, "%s(): removed loco %d\n", __func__, atoi(kv->value));
			}
		}
	}

	hdrs = kv_add(NULL, "Content-Length", "0");
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	return 0;
}

static int cgi_modeltime (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv, *hdrs;
	int year, mon, mday, hour, min, factor;

	(void) rest;
	(void) sz;

	year = mon = mday = hour = min = factor = -1;
	if ((kv = kv_lookup(hr->param, "y")) != NULL) year = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "year")) != NULL) year = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "mon")) != NULL) mon = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "d")) != NULL) mday = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "day")) != NULL) mday = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "mday")) != NULL) mday = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "h")) != NULL) hour = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "hour")) != NULL) hour = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "min")) != NULL) min = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "f")) != NULL) factor = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "factor")) != NULL) factor = atoi(kv->value);
	if ((kv = kv_lookup(hr->param, "speedup")) != NULL) factor = atoi(kv->value);

	mt_setdatetime(year, mon, mday, hour, min);
	if (factor >= 0) mt_speedup(factor);

	hdrs = kv_add(NULL, "Content-Length", "0");
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	return 0;
}

#if 0
static int cgi_sound (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv, *hdrs;

	(void) rest;
	(void) sz;

	if ((kv = kv_lookup(hr->param, "play")) != NULL) {
		player_play(kv->value);
	}
	if ((kv = kv_lookup(hr->param, "stop")) != NULL) {
		player_stop();
	}
	if ((kv = kv_lookup(hr->param, "volume")) != NULL) {
		player_volume(atoi(kv->value));
	}

	hdrs = kv_add(NULL, "Content-Length", "0");
	httpd_header(sock, FILE_OK, hdrs);
	kv_free(hdrs);
	return 0;
}
#endif

static int cgi_internal (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv, *hdrs;
	int fb, state;

	(void) rest;
	(void) sz;

	if ((kv = kv_lookup(hr->param, "info")) != NULL) {
		hdrs = kv_add(NULL, "Content-Type", CONTENT_TEXT);
		httpd_header(sock, FILE_OK, hdrs);
		kv_free(hdrs);

		socket_printf (sock, "%s\t%x.%x\t%s\t%02x\t%d\t%02X:%02X:%02X:%02X:%02X:%02X\n",
				(*hwinfo->proddate != 0xFF) ? hwinfo->proddate : "---",
				(hwinfo->HW >> 4) & 0xF, (hwinfo->HW >> 0) & 0xF, SOFT_VERSION, hwinfo->manufacturer, hwinfo->serial,
				rt.en->hwaddr[0], rt.en->hwaddr[1], rt.en->hwaddr[2], rt.en->hwaddr[3], rt.en->hwaddr[4], rt.en->hwaddr[5]);
		return 0;
	} else if ((kv = kv_lookup(hr->param, "tempoffset")) != NULL) {
		an_temperaturTest(atoi(kv->value));
		httpd_header(sock, FILE_OK, NULL);
		return 0;
	} else if ((kv = kv_lookup(hr->param, "fb")) != NULL) {
		fb = atoi(kv->value);
		state = 0;
		if ((kv = kv_lookup(hr->param, "stat")) != NULL) state = atoi(kv->value);
		fb_rangeInput(fb * 16, 16, (uint8_t *) &state);
		return 0;
	}

	httpd_header(sock, FILE_NOT_FOUND, NULL);
	return 0;
}

static int cgi_esp (int sock, struct http_request *hr, const char *rest, int sz)
{
	struct key_value *kv;

	(void) rest;
	(void) sz;

	if ((kv = kv_lookup(hr->param, "update")) != NULL) {
		esp_triggerUpdate();
	}
	httpd_header(sock, FILE_OK, NULL);
	return 0;
}

bool cgi_check_request (int sock, struct http_request *hr, const char *rest, int sz)
{
	const struct vFuncT *vf;

	vf = vFuncs;
	if (!hr || !hr->uri) return false;

	vf = vFuncs;
	while (vf->path != NULL) {
		if (!strcmp(hr->uri, vf->path) && hr->request == vf->request) {
			if (vf->func) vf->func(sock, hr, rest, sz);
			else httpd_serve_file(sock, HTML_404, NULL);
			return true;
		}
		vf++;
	}
	return false;
}

/**
 * @}
 */
