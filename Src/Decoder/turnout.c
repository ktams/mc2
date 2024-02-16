/**
 * @file turnout.c
 *
 * @author Andi
 * @date   30.04.2020
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

#include <stdio.h>
#include "rb2.h"
#include "config.h"
#include "decoder.h"
#include "events.h"

#define TURNOUTS_PER_GROUP		4		///< a decoder controls four turnouts as a group
#define TURNOUT_MIN_TIME		100		///< minimum switching time in ms
#define TURNOUT_MAX_TIME		5000	///< maximum switching time in ms
#define TURNOUT_QUEUELEN		16		///< number of entries in the command queue
#define TURNOUT_MIN_DELAY		20		///< a minimum delay between TURN-ON commands on the track
#define TURNOUT_MAX_ACTIVE		16		///< the maximum length of the active list, i.e. the maximum number of concurrently energized turnouts

static QueueHandle_t queue;
static volatile TickType_t mintime = TURNOUT_MIN_TIME;
static volatile TickType_t maxtime = TURNOUT_MAX_TIME;

struct trnt_command {
	TickType_t			 duration;		///< the activation time can be set in advance, an automated switch OFF will be remembered in trnt_action
	uint16_t			 adr;			///< the address of the turnout to switch (1-based)
	bool				 dir;			///< the direction: straight or thrown
	bool				 on;			///< energize or de-energize (with de-energize the direction can be ignored)
};

struct trnt_action {
	struct trnt_action	*next;			///< a linked list of turnout requests / stats
	turnoutT			*t;				///< the turnout structure of the turnout to switch
	struct packet		*pck;			///< a prepared packet for switching ON (in req_start list) or OFF (in active list)
	TickType_t			 start;			///< the time at which the turnout was switched ON (or the scheduled OFF time if timed_off is set)
	TickType_t			 duration;		///< the maximum time for which the turnout should be energized (may be maxtime value)
	unsigned			 dir:1;			///< if set, direction indicates "thrown"
	unsigned			 req_off:1;		///< OFF was requested for this turnout - wait at least until the minimum time has elapsed
};

static struct trnt_action *active;		///< a list of currently active switching turnouts and their timing
static struct trnt_action *req_start;	///< a list of TURN-ON requests to be scheduled
static TickType_t last;					///< the time of the last TURN-ON command (we should not schedule too fast!)

/**
 * Check if another turnout in the same group is currently active.
 * Traditionally, an accessory decoder handles four turnouts and a single decoder
 * should not activate more than one turnout. For the original MM-decoders, this
 * restriction was even based on hardware design!
 *
 * New decoders may only control a single turnout or be able to activate all outputs
 * at the same time, but for backward compatibility and to not overstress local powerdistribution,
 * we will keep this limitation intact.
 *
 * \param trnt		the turnout to check its group for (1-based)
 * \return			true if another turnout is currently beeing switched
 */
static bool trnt_groupActive (int trnt)
{
	turnoutT *t;
	int grp, i;

	grp = (trnt - 1) / TURNOUTS_PER_GROUP;
	for (i = grp * TURNOUTS_PER_GROUP + 1; i <= (grp + 1) * TURNOUTS_PER_GROUP; i++) {
		if ((i != trnt) && ((t = db_lookupTurnout(i)) != NULL)) {
			if (t->on) return true;
		}
	}
	return false;
}

static void trnt_listAppend (struct trnt_action **lst, struct trnt_action *a)
{
	a->next = NULL;
	while (lst && *lst) lst = &(*lst)->next;
	if (lst) *lst = a;
}

static void trnt_requestOff (struct trnt_action *a, int trnt)
{
	while (a) {
		if (a->t->adr == trnt) a->req_off = 1;
		a = a->next;
	}
}

static bool trnt_checkBusy (struct trnt_action *a, int trnt)
{
	while (a) {
		if (a->t->adr == trnt) return true;
		a = a->next;
	}
	return false;
}

static int trnt_listlength (struct trnt_action *lst)
{
	int count = 0;

	while (lst) {
		count++;
		lst = lst->next;
	}
	return count;
}

/**
 * Check for active turnouts that can/must be switched off (either because
 * commanded to do so or maximum time reached).
 */
static void trnt_checkDone (void)
{
	struct trnt_action **lst = &active;
	struct trnt_action *a;
	TickType_t now = xTaskGetTickCount();

	while ((a = *lst) != NULL) {
		// check if OFF is requested and mintime has passed or planned duration (may be maxtime) is over
		if (   (a->req_off && time_check(now, a->start + mintime))
			|| (time_check(now, a->start + a->duration))) {
			sigq_queuePacket(a->pck);
			a->t->on = false;
			event_fire(EVENT_TURNOUT, 0, a->t);
//			log_msg (LOG_INFO, "%s() %d OFF @ %lu\n", __func__, a->t->adr, now);
			*lst = a->next;
			free (a);
		} else {
			lst = &a->next;
		}
	}
}

static void trnt_checkStart (void)
{
	struct trnt_action **lst = &req_start;
	struct trnt_action *a;
	TickType_t now = xTaskGetTickCount();

	if (!req_start) return;			// no requests waiting, so nothing to do
	if (trnt_listlength(active) >= TURNOUT_MAX_ACTIVE) return;	// we do not want to have more than that active magnets on the layout
	if (!time_check(now, last + TURNOUT_MIN_DELAY)) return;		// we must wait a little to make sure, that maximum pace of TURNOUT-ON command is not violated

	// OK, we potentially may fire another turnout ON-command
	while ((a = *lst) != NULL) {
		if (!trnt_groupActive(a->t->adr)) {
			*lst = a->next;		// take this request out of the list
			sigq_queuePacket(a->pck);									// send prepared ON-packet
			a->pck = sigq_magnetPacket(a->t, a->dir, false);			// prepare SWITCH-OFF packet
			a->start = now;												// write down activation time (doesn't take queue timing into account though)
			a->t->dir = a->dir;								// update turnout status and fire event
			a->t->on = true;
			event_fire(EVENT_TURNOUT, 0, a->t);
			trnt_listAppend(&active, a);					// append this request to the active list
//			log_msg (LOG_INFO, "%s() %d ON @ %lu\n", __func__, a->t->adr, now);
			return;
		}
		lst = &a->next;
	}
}

static TickType_t trnt_calcTimeout (void)
{
	struct trnt_action *a;
	TickType_t now, delay, off;

	now = xTaskGetTickCount();

	delay = portMAX_DELAY;
	a = active;
	while (a) {
		off = (a->start + a->duration) - now;	// delay until latest point where we should shut off this turnout
		if (a->req_off) off = (a->start + mintime) - now;
		if (off < delay) delay = off;
		a = a->next;
	}

	if (req_start) {	// we still want to schedule turnout SWITCH-ON commands, so we will wait no longer than until last + TURNOUT_MIN_DELAY
		off = last + TURNOUT_MIN_DELAY - now;
		if (off < delay) delay = off;
	}

	return delay;
}

void trnt_service (void *pvParameter)
{
	struct trnt_command tc;
	struct trnt_action *a;
	TickType_t wait;
	int rc;

	(void) pvParameter;

	log_msg (LOG_INFO, "%s() started\n", __func__);

	if ((queue = xQueueCreate(TURNOUT_QUEUELEN, sizeof(struct trnt_command))) == NULL) {
		log_error ("%s(): cannot create command queue - give up\n", __func__);
		vTaskDelete(NULL);
	}

	for (;;) {
		wait = trnt_calcTimeout();
		rc = xQueueReceive(queue, &tc, wait);
		if (rc) {		// we have a request
			if (tc.on) {	// request for switching a turnout ON
				if (trnt_checkBusy(active, tc.adr) || trnt_checkBusy(req_start, tc.adr)) {
//					log_msg (LOG_INFO, "%s() ON-Request %d ALREADY ACTIVE\n", __func__, tc.adr);
				} else {
//					log_msg (LOG_INFO, "%s() ON-Request %d\n", __func__, tc.adr);
					if ((a = calloc(1, sizeof(*a))) != NULL) {
						if (loco_lock(__func__)) {
							a->t = db_getTurnout(tc.adr);
							loco_unlock();
							if (a->t) {
								if (tc.duration > 0 && tc.duration < mintime) tc.duration = mintime;				// enforce mintime requirement
								a->duration = (tc.duration > 0 && tc.duration < maxtime) ? tc.duration : maxtime;	// enforce maxtime requirement
								a->dir = tc.dir;
								a->pck = sigq_magnetPacket(a->t, a->dir, true);			// prepare SWITCH-ON packet
								trnt_listAppend(&req_start, a);
							} else {
								free (a);
							}
						} else {
							free (a);
						}
					}
				}
			} else {		// request for switching a turnout OFF
//				log_msg (LOG_INFO, "%s() OFF-Request %d\n", __func__, tc.adr);
				trnt_requestOff(active, tc.adr);		// search the turnout in the active list
				trnt_requestOff(req_start, tc.adr);		// also search in the request list in case it is not yet switched on (delayed for track queue)
			}
		}

		trnt_checkDone();
		trnt_checkStart();
	}
}

/**
 * Return a string in JSON format with the ranges for settings regarding
 * the turnout parameters (mintime, maxtime and repeat count).
 *
 * \return		a JSON string with ranges
 */
char *trnt_getRanges (void)
{
	static char response[256];

	if (!*response) {	// first call -> fill the string
		sprintf (response, "{ \"turnouts\": { \"tmin\": %d, \"tmax\": %d }}\n", TURNOUT_MIN_TIME, TURNOUT_MAX_TIME);
	}
	return response;
}

void trnt_setMinTime (int ms)
{
	if (ms < TURNOUT_MIN_TIME) ms = TURNOUT_MIN_TIME;
	if (ms > TURNOUT_MAX_TIME) ms = TURNOUT_MAX_TIME;
	if ((TickType_t) ms > maxtime) maxtime = ms;
	if (mintime != (TickType_t) ms) {
		mintime = ms;
		cnf_triggerStore(__func__);
		event_fire (EVENT_ACCESSORY, 0, NULL);
	}
}

int trnt_getMinTime (void)
{
	return mintime;
}

void trnt_setMaxTime (int ms)
{
	if (ms < TURNOUT_MIN_TIME) ms = TURNOUT_MIN_TIME;
	if (ms > TURNOUT_MAX_TIME) ms = TURNOUT_MAX_TIME;
	if ((TickType_t) ms < mintime) mintime = ms;
	if (maxtime != (TickType_t) ms) {
		maxtime = ms;
		cnf_triggerStore(__func__);
		event_fire (EVENT_ACCESSORY, 0, NULL);
	}
}

int trnt_getMaxTime (void)
{
	return maxtime;
}

static int _trnt_BiDiB (turnoutT *t, bool thrown)
{
	struct bidibnode *node;
	bidibmsg_t *m;
	uint8_t data[2];

	if ((node = BDBnode_lookupNodeByUID(t->uid, NULL)) != NULL) {	// if this node is not operative, we simply ignore the request
		log_msg (LOG_INFO, "%s() UID=%s aspect %d %s\n", __func__, bidib_formatUID(node->uid), t->aspect, thrown ? "THROWN" : "STRAIGHT");
		data[0] = t->aspect;
		data[1] = thrown ? 1 : 0;
		if ((m = bidib_genMessage(node, MSG_ACCESSORY_SET, 2, data)) != NULL) {
			// Switch accessory on BiDiB node
			BDBnode_downlink(NULL, m);
		}
	}
	return 0;
}

/**
 * The functional part for trnt_switch() and trnt_switchTimed().
 *
 * \param adr		the 1-based address of the turnout to switch
 * \param thrown	if true, the turnout is switched to thrown direction, else to straight
 * \param on		if set, the magnet is switched ON, else OFF
 * \param tim		the time in ms the magnet should be energized
 * \return			0 if everything is OK, else -1
 * \see				trnt_switch()
 * \see				trnt_switchTimed()
 */
static int _trnt_switch (int adr, bool thrown, bool on, TickType_t tim)
{
	struct trnt_command tc;
	turnoutT *t;

	if ((t = db_lookupTurnout(adr)) != NULL && t->fmt== TFMT_BIDIB) return _trnt_BiDiB(t, thrown);
	if (adr <= 0 || adr > MAX_TURNOUT) return -1;
	if (rt.tm != TM_HALT && rt.tm != TM_GO) return -3;	// track not supplied - ignore call

	tc.adr = adr;
	tc.dir = thrown;
	tc.on = on;
	tc.duration = tim;
	xQueueSendToBack(queue, &tc, 100);
//	log_msg (LOG_INFO, "%s(): ADR %d %s %s @ %lu\n", __func__, adr, (thrown) ? "THROWN" : "STRAIGHT",
//			(on) ? "ON" : "OFF", xTaskGetTickCount());
	return 0;
}

/**
 * Command a turnout to switch to the indicated direction.
 * The turnout magnet is energized for the specified time and
 * then shut off automatically.
 * There are also automatic time guards (mintime and maxtime) that control the switch behavior.
 *
 * \param adr		the 1-based address of the turnout to switch
 * \param thrown	if true, the turnout is switched to thrown direction, else to straight
 * \param tim		the time in ms the magnet should be energized
 * \return			0 if everything is OK, else -1
 */
int trnt_switchTimed (int adr, bool thrown, TickType_t tim)
{
	return _trnt_switch(adr, thrown, true, tim);
}

/**
 * Switch a turnout relay ON or OFF.
 * The turnout magnet can only be switched off if found in the active or req_start list.
 * There are automatic time guards (mintime and maxtime) that control the switch behavior.
 *
 * \param adr		the 1-based address of the turnout to switch
 * \param thrown	if true, the turnout is switched to thrown direction, else to straight
 * \param on		if set, the magnet is switched ON, else OFF
 * \return			0 if everything is OK, else -1
 */
int trnt_switch (int adr, bool thrown, bool on)
{
	return _trnt_switch(adr, thrown, on, 0);
}
