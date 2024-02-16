/**
 * @file reply.c
 *
 * @author Andi
 * @date   02.05.2020
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
 * \ingroup Track
 * \{
 *
 * @file
 *
 * This is a kind of meeting point between threads waiting for messages
 * from decoders and interfaces that are able to read these messages and
 * forward them to the registered listeners.
 *
 * Listeners may be registered any time and will return a boolean, that
 * will deregister them or stay in the listening state. This is similar
 * to the event system in System/eventlistener.c.
 *
 * 01/2022: NEW Approach:
 * There are two types of listeners: the registered listeners that wait for
 * decoder messages in general and callbacks that are attached to specific
 * decoder commands. The latter ones are not registered in that sense, but
 * are called whenever a decoder message event with a provided callback
 * function is received. In this case, only the given callback is called
 * and all other listeners don't get to see this message.
 *
 * On the other hand, registered listeners will receive such messages thru
 * the messaging comming directly from the railcom UART interrupt.
 *
 * To get this done, we have a thread running that is waiting for forwardable
 * messages from a queue. The queue entries will supply the folling information:
 *   - the decoder type the message came from
 *   - the address of that decoder
 *   - the type of message
 *   - additional data like CV address, data, etc.
 *   - a callback function and a private argument (if not NULL, only this function will be called)
 *
 * All callbacks are executed on behalf of this thread and should not block
 * or wait. The regsitered listeners may also receive timout messages, when
 * they specified a timeout on registration. These listeners may deregsiter
 * themselve by returning a false value. The track packet bounded callbacks
 * are only called once at the end of the relevant packet and thus will be
 * void functions.
 */

#include <stdio.h>
#include <string.h>
#include "rb2.h"
#include "timers.h"
#include "decoder.h"
#include "events.h"

#define MAX_MUTEX_WAIT		20			///< maximum waittime (in ms) for the list mutex to become available
#define TIMER_OVERFLOW		(1 << 31)	///< the topmost bit marks a time difference, that tells us that the current time is later than the defined timeout
#define CB_QUEUE_LEN		8			///< number of queue entries for the callback handling thread (not really much needed!)

struct msgListener {
	struct msgListener	*next;					///< singly linked list of listeners
	reply_handler		 handler;				/**< The handler function that is to be called with the message and it's private data.
												 *	 If this handler returns <i>false</i>, it is removed from the listener list. In
												 *	 this case, it should release any resources that it might have allocated when
												 *	 registering itself as an event listener.
												 */
	flexval				 priv;					/**< Private data for the callback function to identify the requester. If this
												 *	 data is dynamically allocated (malloc(), etc.) it must be freed before returning
												 *	 <i>false</i> from the handler function (which means that the handler wishes
												 *	 to unregister itself).
	 	 	 	 	 	 	 	 	 	 	 	 */
	TickType_t			 timeout;				///< a possible timeout when waiting for events
	TickType_t			 to_tim;				///< the time at which the currently running timeout triggers - recalculated after each handler call
	dec_type			 dt;					///< filter: the decoder type
	dec_msgtype			 mt;					///< filter: the message type
	int					 adr;					///< filter: the address
};

/**
 * These messages (dec_msgtype) can come from either railcom or m3 readback:
 *
 *						|						|     |    | max. | prog |
 * MSG-Type				| Decodertype			| ADR | CV | Data | val  |
 * ---------------------+-----------------------+-----+----+------+------+
 * DECODERMSG_POM		| DECODER_DCC_MOBILE	|  X  |  X | 1	  |  *   |	(progval only with write commands)
 * DECODERMSG_POM		| DECODER_DCC_ACC		|  X  |  X | 1	  |  *   |	(progval only with write commands)
 * DECODERMSG_POM		| DECODER_DCC_EXT		|  X  |  X | 1	  |  *   |	(progval only with write commands)
 * DECODERMSG_ADRL		| DECODER_DCC_MOBILE	|     |    | 1    |      |
 * DECODERMSG_ADRH		| DECODER_DCC_MOBILE	|     |    | 1    |      |
 * DECODERMSG_XPOMnn	| DECODER_DCC_MOBILE	|  X  |  X | 4	  |  *   |	(progval only with write commands for direct callback, not for listeners)
 * DECODERMSG_XPOMnn	| DECODER_DCC_ACC		|  X  |  X | 4	  |  *   |	(progval only with write commands for direct callback, not for listeners)
 * DECODERMSG_XPOMnn	| DECODER_DCC_EXT		|  X  |  X | 4	  |  *   |	(progval only with write commands for direct callback, not for listeners)
 * DECODERMSG_EXT		| DECODER_DCC_MOBILE	|  X  |    | 2	  |      |
 * DECODERMSG_STAT1		| DECODER_DCC_ACC		|  X  |    | 1	  |      |
 * DECODERMSG_STAT1		| DECODER_DCC_EXT		|  X  |    | 1	  |      |
 * DECODERMSG_TIME		| DECODER_DCC_ACC		|  X  |    | 1	  |      |
 * DECODERMSG_TIME		| DECODER_DCC_EXT		|  X  |    | 1	  |      |
 * DECODERMSG_ERR		| DECODER_DCC_ACC		|  X  |    | 1	  |      |
 * DECODERMSG_ERR		| DECODER_DCC_EXT		|  X  |    | 1	  |      |
 * DECODERMSG_DYN		| DECODER_DCC_MOBILE	|  X  |    | 2	  |      |
 * DECODERMSG_DYN		| DECODER_DCC_ACC		|  X  |    | 2	  |      |
 * DECODERMSG_DYN		| DECODER_DCC_EXT		|  X  |    | 2	  |      |
 * DECODERMSG_RUNTIME	| DECODER_DCC_MOBILE	|  X  |  X | 1	  |      |
 * DECODERMSG_DECSTATE	| DECODER_DCC_MOBILE	| UID |    | 6	  |      |	(DCC-A, decoder type???)
 * DECODERMSG_DECSTATE	| DECODER_DCC_ACC		| UID |    | 6	  |      |	(DCC-A, decoder type???)
 * DECODERMSG_DECSTATE	| DECODER_DCC_EXT		| UID |    | 6	  |      |	(DCC-A, decoder type???)
 * DECODERMSG_UNIQUE	| DECODER_DCC_MOBILE	|     |    | 6	  |      |	(DCC-A, decoder type???)
 * DECODERMSG_UNIQUE	| DECODER_DCC_ACC		|     |    | 6	  |      |	(DCC-A, decoder type???)
 * DECODERMSG_UNIQUE	| DECODER_DCC_EXT		|     |    | 6	  |      |	(DCC-A, decoder type???)
 * DECODERMSG_SRQ		| DECODER_DCC_ACC		|     |    | 2	  |      |
 * DECODERMSG_SRQ		| DECODER_DCC_EXT		|     |    | 2	  |      |
 *
 * DECODERMSG_TIMEOUT	| DECODER_ANY			|     |  X |  	  |  X   |	(programming track - no address used)
 * DECODERMSG_READERROR	| DECODER_ANY			|     |  X |  	  |  X   |	(programming track - no address used)
 * DECODERMSG_PGACK		| DECODER_ANY			|     |  X | 1	  |  X   |	(programming track - no address used)
 *
 * DECODERMSG_POM		| DECODER_M3_MOBILE		|  X  |  X |	  |  X   |	(POM write is executed but no answer is returned)
 * DECODERMSG_M3BIN		| DECODER_M3_MOBILE		| UID |    | 1	  |      |	(search, ping)
 * DECODERMSG_M3DATA	| DECODER_M3_MOBILE		|  X  |  X | 8	  |      |	(POM read only)
 *
 * Basically, every POM, XPOM or equivalent command should have a callback
 * attached. POM write for m3 decoders won't return a value, but you will be
 * sure that the command was sent out to the track.
 *
 * The special commands to write two CVs (namely CV17/18 or CV31/32) should have
 * a special code to pack the two messages from the railcom window into a single
 * combined message.
 */
#if 0
struct msg {	// hyphotecical structure for event triggering (queue entry), $$$$ currently 36 Bytes (@ progval as last element)
	reply_handler		 handler;				///< if not NULL: the handler to call
	void				*priv;					///< a private parameter for the callback function (either direct or registered callback)
	dec_type			 dt;					///< te type of decoder this message stems from
	dec_msgtype			 mt;					///< the message that was received
	union {
		int					 adr;				///< the decoder address if relevant and known
		uint32_t			 uid;				///< a decoder UID if relevant
	};
	cvadrT				 cva;					///< a CV address if relevant
	int					 len;					///< the valid length of the data associated with the message
	uint8_t				 data[8];				///< up to eight data bytes may be received (m3 read block, DCC railcom may deliver up to 6 bytes)
	flexval				 progval;				///< for comparision: the value(s) that should have been written ((X)POM Write commands)
};
#endif

static volatile struct msgListener *listener;	///< the currently active listeners
static SemaphoreHandle_t mutex;					///< locking for access to listener list
static TimerHandle_t timer;						///< a timer for timeout handling of registered listeners
static QueueHandle_t cbQueue;					///< the callback handler queue for decoder command replies

static void reply_timerFire(TimerHandle_t t)
{
	cvadrT cva;

	(void) t;

	cva.cv = 0;
	reply_deliver(DECODER_ANY, 0, DECODERMSG_TIMEOUT, cva, fvNULL, 0, NULL);
}

static void reply_stopTimer (void)
{
	if (timer) {
		xTimerStop(timer, 10);
	}
}

static void reply_startTimer (TickType_t tim)
{
	reply_stopTimer();
	if (!timer) {
		timer = xTimerCreate("replyTimer", tim, pdFALSE, NULL, reply_timerFire);
		if (!timer) return;
	}

	if (tim == 0 || tim & TIMER_OVERFLOW) return;		// don't care about a timer that has a duration of > 24 days!
	xTimerChangePeriod(timer, tim, 10);
}

/**
 * Calculates the current shortest timeout that we have to wait for.
 * This function should only be called when the mutex is held, because
 * we must scan the list of listeners (but don't change them).
 */
static TickType_t reply_calcTimeout (void)
{
	struct msgListener *l;
	TickType_t now, diff, d;

	if (!listener) return 0;			// no more listeners - we calculate no timeout

	now = xTaskGetTickCount();
	l = (struct msgListener *) listener;

	diff = TIMER_OVERFLOW;
	while (l) {
		if (l->timeout) {
			d = l->to_tim - now;
			if (d & TIMER_OVERFLOW) {
				log_error ("%s(): handler %p already timed out (@%s to=%lu)\n", __func__, l->handler, timestamp(l->to_tim), l->timeout);
				diff = 0;	// "immediately" time out
			} else {
				if (d < diff) diff = d;
			}
		}
		l = l->next;
	}
	if (diff < 2) return 2;
	return diff;
}


bool rc_event_handler(struct decoder_reply *msg, flexval priv)
{
	int sock, i;
	char *tmp = tmp256();
	char dump[128], *s;
	uint8_t *p;

	sock = priv.i32;
	if (!tcp_checkSocket(sock)) {
		log_msg (LOG_INFO, "%s(): socket is closed - deregistering\n", __func__);
		return false;	// socket is closed - deregister callback
	}

	memcpy(tmp, msg, sizeof(*msg));
	if((msg->adr > 0) && (msg->adr <= MAX_DCC_ADR) && msg->mt != DECODERMSG_NOANSWER) {
		event_fire (EVENT_RAILCOM, 0, tmp);
	}
//	log_msg (LOG_INFO, "%s(): adr: %d, Dec-type: %d, Msg-type: %d CV: %lu Param %ld\n", __func__,
//		msg->adr, msg->dtype, msg->mt, msg->cva.cv, msg->param.i32);
	s = dump;
	p = msg->data;
	s += sprintf (dump, "\t DATA");
	for (i = 0; i < msg->len; i++, p++) {
		s += sprintf (dump, " %#02x", *p);
	}
//	log_msg (LOG_INFO, "%s\n", dump);
	return true;
}

/**
 * Register an reply/message handler for a specified decoder reply.
 *
 * \param dt		the type of the decoder we are expecting a message from (specify DECODER_ANY for a wildcard match)
 * \param adr		the address of the decoder we are expecting a message from (specify 0 for a wildcard match)
 * \param msgtp		the message type you are interested in - specify DECODERMSG_ANY for a wildcard match
 * \param handler	the handler function that is called if one of the registered events fires
 * \param prv		private data for the handler
 * \param timeout	A timeout in ticks (that is ms here) or 0 to define no timeout (infinit waiting for an event).
 * 					This timeout can only be specified once. The first call that sets a timeout is the "winning" one.
 * \return			0 for successful register or an error code otherwise
 */
int reply_register (dec_type dt, int adr, dec_msgtype msgtp, reply_handler handler, flexval priv, TickType_t timeout)
{
	struct msgListener *l, **lpp;
	TickType_t to;

	if (msgtp == DECODERMSG_TIMEOUT || !handler) return -2;	// wrong parameters
	if (adr < 0 || adr > MAX_LOCO_ADR) return -3;			// wrong parameter

	if ((l = calloc(1, sizeof(*l))) == NULL) return -4;		// no RAM?

	if (timeout) {		// YES, we want to set a timeout
		to = xTaskGetTickCount() + timeout;
	} else {			// NO, don't care for timeouts
		to = 0;
	}

	l->handler = handler;
	l->timeout = timeout;
	l->to_tim = to;
	l->dt = dt;
	l->mt = msgtp;
	l->adr = adr;
	l->priv = priv;

//	if (dt == DECODER_DCC_MOBILE)
//		log_msg (LOG_DEBUG, "%s() DT=%d ADR=%d MT=%d timeout %lu l %p handler %p\n", __func__, dt, adr, msgtp, timeout, l, handler);

	if (!mutex_lock(&mutex, MAX_MUTEX_WAIT, __func__)) {
		free (l);
		return -1;		// we could not get the lock - bad luck
	}

	if (timeout) reply_stopTimer();		// YES, we want to set a timeout, so we must stop current timer action
	lpp = (struct msgListener **) &listener;
	while ((*lpp) != NULL) lpp = &(*lpp)->next;				// goto to last entry in list ...
	*lpp = l;												// ... and append the new listener
	if (timeout) reply_startTimer(reply_calcTimeout());		// if we stopped the timer, we must restart it here

	mutex_unlock(&mutex);
//	log_msg (LOG_DEBUG, "%s(): listener for ADR %d / decoder %d / msgtype %d registered\n", __func__, adr, dt, msgtp);
	return 0;
}

static bool reply_isDue (struct msgListener *l, struct decoder_reply *msg, TickType_t now)
{
	// first check for timeout event
	if (msg->mt == DECODERMSG_TIMEOUT) {
		if (l->timeout == 0) return false;				// this listener is not waiting for timeouts
		return !!(time_check(now, l->to_tim));
	}

	// now check for individual filter options
	if ((l->adr > 0) && (l->adr != msg->adr)) return false;						// the address does not match
    if ((l->dt != DECODER_ANY) && (l->dt != msg->dtype)) return false;			// the decoder type does not match
	if ((l->mt != DECODERMSG_ANY && (l->mt != msg->mt))) return false;			// the message type does not match
	return true;	// OK, every filter criterium matches - we should call the handler
}

static void reply_removeHandler (struct msgListener *l)
{
	struct msgListener *lp, **lpp;
	struct msgListener search;

	// copy the data we are trying to find in handler entries (to remove multiple instances)
	// the original memory 'l' is pointing to may get freed while scanning the list!
	memcpy (&search, l, sizeof(search));

//	log_msg(LOG_INFO, "%s() removing @%p MT=%d\n", __func__, l, l->mt);
	lpp = (struct msgListener **) &listener;
	while ((lp = *lpp) != NULL) {
		if (lp->adr == search.adr && lp->dt == search.dt && lp->handler == search.handler && lp->priv.u32 == search.priv.u32) {
//			log_msg(LOG_INFO, "%s() @%p MT=%d\n", __func__, lp, lp->mt);
			*lpp = lp->next;
			free (lp);
		} else {
			lpp = &lp->next;
		}
	}
}

/**
 * This is a thread function, that calls all the registered handlers for an
 * event that has fired. All callbacks are executed in the context of this
 * thread.
 *
 * The list of listeners is scanned for interested ones and then the handler
 * function is called in the context of this thread. If the handler returns
 * <i>false</i> it is removed from the listener list.
 *
 * While this thread is running, the list mutex is taken.
 *
 * @param pvParameter	the thread invocation parameter - this is the allocated eventT from the thread creator and must be freed in the end
 */
static void reply_worker (void *pvParameter)
{
	struct msgListener *l, **lpp;
	struct decoder_reply *msg;
	TickType_t now;

	msg = (struct decoder_reply *) pvParameter;
	if (!msg) vTaskDelete(NULL);		// should never happen ... but to be sure

	if (mutex_lock(&mutex, MAX_MUTEX_WAIT, __func__)) {
		reply_stopTimer();									// we can stop the timer and will recalculate the timeout after the callbacks are done
		now = xTaskGetTickCount();
		lpp = (struct msgListener **) &listener;
		while ((l = *lpp) != NULL) {
			if (reply_isDue(l, msg, now)) {
				if (msg->mt == DECODERMSG_TIMEOUT) {	// for a timeout, the listener filter values must be copied to the message
					msg->adr = l->adr;
					msg->dtype = l->dt;
				}
//				log_msg (LOG_DEBUG, "%s() MSG=%d ADR=%d/%d l=%p handler=%p\n", __func__, msg->mt, msg->adr, l->adr, l, l->handler);
				if (!l->handler(msg, l->priv)) {
					reply_removeHandler(l);
					lpp = (struct msgListener **) &listener;		// restart from beginning
					continue;
				} else if (l->timeout) {
					l->to_tim = now + l->timeout;
				}
			}
			lpp = &l->next;
		}

		reply_startTimer(reply_calcTimeout());
		mutex_unlock(&mutex);
	}

	free (msg);
	vTaskDelete(NULL);
}

/**
 * Deliver a reply.
 * An independant thread is started to do the real work. It gets the reply as it's
 * parameter. This thread then checks all listeners and serially calls their handler
 * functions. If no listeners are registered we can shortcut this to an immediate
 * return, because the pointer can be atomically checked without taking the mutex.
 *
 * Also, a decoder message of DECODERMSG_INVALID will be ignored.
 *
 * \param dt		the decoder type
 * \param adr		the decoder address
 * \param mt		the message type
 * \param cva		a CV address structure if relevant
 * \param fv		a value encoded in a flexible variable
 * \param len		the length of the data field (if any)
 * \param data		the optional data field (up to 16 bytes / check sizeof(m->data))
 * \see				reply_worker()
 */
void reply_deliver (dec_type dt, int adr, dec_msgtype mt, cvadrT cva, flexval param, int len, uint8_t *data)
{
	struct decoder_reply *m;

//	if (data) {
//		printf ("%s(): DT=%d adr=%d MT=%d PARAM=%ld DATA=%d\n", __func__, dt, adr, mt, fv.i32, data[0]);
//	} else {
//		printf ("%s(): DT=%d adr=%d MT=%d PARAM=%ld\n", __func__, dt, adr, mt, fv.i32);
//	}


//	log_msg (LOG_DEBUG, "%s(): DT=%d adr=%d MT=%d CV=%ld, Val=%d\n", __func__, dt, adr, mt, cva.cv, data[0]);


	if (mt == DECODERMSG_INVALID) return;		// this is no real message - we shouldn't have get here (check the caller)
//	if (mt == DECODERMSG_ANY) return;			// this would make no sense $$$$ currently a lot of messages are of this type (railcom.c!)

	if (listener) {								// only if we have listeners registered - else ignore this call
		if ((m = malloc(sizeof(*m))) == NULL) return;	// no RAM to post this message
		m->dtype = dt;
		m->adr = adr;
		m->mt = mt;
		m->cva = cva;
		m->param.u32 = param.u32;

		// care about optional data field
		if (len < 0) len = 0;
		if (len > (int) sizeof(m->data)) len = (int) sizeof(m->data);
		m->len = len;
		if (data && len) {
			memcpy (m->data, data, len);
		}

		xTaskCreate(reply_worker, "REPLYworker", 2048, m, 3, NULL);		// run with slighly raised priority
	}
}

/*
 * =========================================================================================================
 * A new implementation of handling railcom and m3 readback actions.
 * =========================================================================================================
 */

/**
 * A thread that waits for decoder replies delivered from an interrupt handler via
 * reply_callback(). It just waits on it's queue and calls the handlers when a
 * message arrives.
 *
 * This handler is started from init() and should run with a slightly raised priority.
 *
 * @param pvParameter	the thread invocation parameter - ignored here
 */
void reply_callbackHandler (void *pvParameter)
{
	struct decoder_reply *m = NULL;

	(void) pvParameter;

	if ((cbQueue = xQueueCreate(CB_QUEUE_LEN, sizeof(*m))) == NULL) {
		log_error("%s(): cannot create Queue - exit!\n", __func__);
		vTaskDelete(NULL);
	}
	log_msg (LOG_INFO, "%s() running\n", __func__);

	for (;;) {
		while (!m && (m = malloc (sizeof(*m))) == NULL) vTaskDelay(20);			// wait for memory to be available
		if (xQueueReceive(cbQueue, m, portMAX_DELAY)) {
			if (listener) {														// skip task creation if no listeners are registered
				xTaskCreate(reply_worker, "REPLYworker", 2048, m, 3, NULL);		// run with slighly raised priority
				m = NULL;	// we must allocate a new buffer
			}
		}
	}
}

/**
 * This function is called from the UART5 railcom receiver interrupt handler or the
 * m3reply_disable() function which was called from TIM1 signal generation interrupt.
 * It posts the result of a RailCom or m3 readback to a function that was specified
 * when creating the decoder command. So this is the point, where the answer to the
 * request (i.e. the decoder command) is known.
 *
 * Remember that this function is called from an interrupt context! The callback itself
 * is then called from a foreground thread.
 *
 * \param bb		pointer to the bitbuffer that triggered this readback
 * \param mt		the message type to deliver to the listener
 * \param len		length of data (if any)
 * \param data		pointer to the received data (if any)
 */
void reply_callback (struct bitbuffer *bb, dec_msgtype mt, int len, uint8_t *data)
{
	struct decoder_reply msg;

	memset (&msg, 0, sizeof(msg));
	if (listener) {		// ignore everything if no listener is registered
		if (bb) {			// normal responses that pertain to a specific decoder
			msg.dtype = bb->dt;
			msg.adr = bb->adr;
			msg.mt = mt;
			msg.cva = bb->cva;
			msg.param.u32 = bb->param.u32;
		} else {			// generic answer from mobile decoder for detecting a decoder address (ID1 / ID2)
			msg.dtype = DECODER_ANY;
			msg.mt = mt;
		}

		// care about optional data field
		if (len < 0) len = 0;
		if (len > (int) sizeof(msg.data)) len = (int) sizeof(msg.data);
		msg.len = len;
		if (len && data) memcpy (msg.data, data, len);
		xQueueSendToBackFromISR(cbQueue, &msg, NULL);
	}
}

/**
 * \}
 */
