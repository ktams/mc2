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
#include <fcntl.h>
#include "rb2.h"
#include "decoder.h"
#include "config.h"
#include "events.h"

#define LOCO_FNAME		"/loco.db"		///< the file where the loco definition is stored
#define LOCO_TMP		"/loco.tmp"		///< a transient file to keep old definitions intact while storing new ones

static ldataT *locolist;				///< the locos that are actually active - entries reference the locodb @see loco.h
//static volatile bool dirty;				///< the list of loco definitions is dirty and should be written to stable storage

static SemaphoreHandle_t mutex;			///< a mutex to control access to the list of locos

bool loco_lock(const char *caller)
{
	return mutex_lock(&mutex, 20, caller);
}

void loco_unlock (void)
{
	mutex_unlock(&mutex);
}

TickType_t loco_purgetime (void)
{
	struct sysconf *sc;

	sc = cnf_getconfig();
	if (sc->locopurge <= 0) return 0;
	return tim_timeout(sc->locopurge * 60 * configTICK_RATE_HZ);
}

/**
 * Remove a given loco from the refresh list.
 * This is an internal function and should only be called, if the lock is held.
 *
 * If this loco definition has a reference from the refresh list, the entry in
 * the refresh list is removed.
 *
 * \param l		pointer to the loaded loco definition.
 */
void _loco_remove (ldataT *l)
{
	ldataT **ldpp;

	if (!l) return;

	// TODO: remove this loco from consist chain!
	ldpp = &locolist;
	while (*ldpp != l) ldpp = &(*ldpp)->next;
	if (*ldpp == l) {
		*ldpp = l->next;
		event_fire(EVENT_NEWLOCO, -(l->loco->adr), NULL);
		free (l);
	}
}

/**
 * Find a link from the refresh list to the given loco definition.
 * This is an internal function and should only be called, if the lock is held.
 *
 * \param l		pointer to the loaded loco definition.
 * \return		pointer to the loco data entry in refresh list or NULL, if not found
 */
ldataT *_loco_getRefreshLink (locoT *l)
{
	ldataT **ldpp;

	if (!l) return NULL;
	ldpp = &locolist;

	while (*ldpp != NULL && (*ldpp)->loco != l) ldpp = &(*ldpp)->next;
	return *ldpp;
}

/**
 * Get the lock and remove loco from refresh stack
 *
 * \param l		pointer to the live loco data structure to be removed
 */
void loco_remove (ldataT *l)
{
	if (l && loco_lock(__func__)) {
		_loco_remove(l);
		loco_unlock();
	}
}

/**
 * Looks up a loco with the given address.
 * If this loco is not found, a new one is created with it's link to the
 * loco description as found by loco_lookup().
 *
 * This function should only be called with the mutex held.
 *
 * \param adr	the number (ID) of the loco description to look up
 * \param add	add a new loco if existing one is not found in the refresh list
 * \return		a pointer to the current data of the found or created loco
 */
static ldataT *_loco_callLocked (int adr, bool add)
{
	ldataT *l, **lpp;
	locoT *loco;

	if (adr <= 0 || adr > MAX_LOCO_ADR) return NULL;
	lpp = &locolist;
	while ((l = *lpp) != NULL && l->loco && l->loco->adr != adr) lpp = &l->next;
	if (!l && add) {
		log_msg (LOG_INFO, "%s() adding loco %d\n", __func__, adr);
		if ((loco = _db_getLoco(adr, add)) == NULL) return NULL;	// loco not found and could not be created (add is always true here)
		if ((l = calloc (1, sizeof(*l))) == NULL) return NULL;		// no refresh list entry could be allocted
		l->loco = loco;		// reference the loco dictionary entry
		l->speed = 0x80;	// standard speed: forward 0 (i.e. stopped)
		l->purgeTime = loco_purgetime();
		*lpp = l;			// append to end of the list
		event_fire(EVENT_NEWLOCO, adr, NULL);
	}

	return l;
}

/**
 * Looks up a loco with the given address.
 * If this loco is not found, a new one is created with it's link to the
 * loco description as found by loco_lookup(). If the loco is part of a
 * consist and this consist is not yet loaded, all locos in this consist
 * are loaded approiatly and the consist link ring is established.
 *
 * This function should only be called with the mutex held.
 *
 * \param adr	the number (ID) of the loco description to look up
 * \param add	add a new loco if existing one is not found in the refresh list
 * \return		a pointer to the current data of the found or created loco
 * \see			_loco_callLocked()
 */
static ldataT *loco_callLocked (int adr, bool add)
{
	ldataT *l, *tmp, **pp;
	struct consist *c;
	int i;

	if ((c = consist_findConsist(adr)) == NULL) return _loco_callLocked(adr, add);
	if ((l = _loco_callLocked(adr, add)) == NULL) return NULL;		// could not get the loco - stop here
	if (l->consist) return l;										// consist already established nothing else to do

	pp = &l->consist;
	l->flags &= ~LOCO_CONSIST_REVERSE;
	for (i = 0; i < MAX_CONSISTLENGTH; i++) {
		if (c->adr[i] == -adr) {
			l->flags |= LOCO_CONSIST_REVERSE;	// even the original called loco may be reversed regarding this consist
		} else if ((c->adr[i] != 0) && (c->adr[i] != adr)) {
			if ((tmp = _loco_callLocked(abs(c->adr[i]), true)) != NULL) {
				if (c->adr[i] < 0) tmp->flags |= LOCO_CONSIST_REVERSE;
				else tmp->flags &= ~LOCO_CONSIST_REVERSE;
				*pp = tmp;
				tmp->consist = l;	// always make it a ring structure
				pp = &tmp->consist;
			}
		}
	}

	return l;
}

/**
 * Looks up a loco with the given ID.
 * If this loco is not found, a new one is created with it's link to the
 * loco description as found by loco_lookup().
 *
 * This function is a wrapper around the static loco_callLocked() which first
 * gets the mutex locked and than call this function to do the real work.
 * It afterwards releases the locked mutex and returns the result.
 *
 * \param adr	the number (ID) of the loco description to look up
 * \param add	add a new loco if existing one is not found in the refresh list
 * \return		a pointer to the current data of the found or created loco
 * \see			loco_lookup()
 * \see			loco_callLocked()
 */
ldataT *loco_call (int adr, bool add)
{
	ldataT *l;

	if (!loco_lock(__func__)) return NULL;
	l = loco_callLocked(adr, add);
	loco_unlock();
	return l;
}

static inline uint32_t loco_replacebits (uint32_t oldbits, uint32_t newbits, uint32_t mask)
{
	return (oldbits & ~mask) | (newbits & mask);
}

/**
 * Setting any combination of functions according to the mask value.
 * This function may only deal with the lower 32 functions F0 - F31!
 *
 * @param adr		the loco address
 * @param newfuncs	the new status of the masked functions
 * @param mask		a bitset with the functions that should be changed
 * @return			0 if everything is OK or the functions results in a NOP, an errorcode otherwise
 */
int loco_setFuncMasked (int adr, uint32_t newfuncs, uint32_t mask)
{
	ldataT *l;
	struct packet *p;
	uint32_t changemask;
	int f;

	if (adr <= 0 || adr > MAX_LOCO_ADR) return -1;
	if (!loco_lock(__func__)) return -1;

	if ((l = loco_callLocked(adr, true)) != NULL) {
		changemask = (l->funcs[0] & mask) ^ (newfuncs & mask);
		if (!changemask) {		// no functions need to be changed, so avoid sending an event
			loco_unlock();
			return 0;
		}
		l->purgeTime = loco_purgetime();
//		printf ("%s(%d):\tOLD 0x%08lx NEW 0x%08lx MASK 0x%08lx\n", __func__, adr, l->funcs[0], newfuncs, mask);
		l->funcs[0] = loco_replacebits(l->funcs[0], newfuncs, changemask);
//		printf ("\t\t\t->  0x%08lx changemask 0x%08lx\n", l->funcs[0], changemask);
		while (changemask) {
			p = NULL;
			switch (l->loco->fmt) {
				case FMT_MM1_14:
					if (changemask & FUNC_LIGHT) {
						p = sigq_speedPacket(l, l->speed);
						changemask &= ~FUNC_LIGHT;
					} else if (changemask & FUNC_F1_F4) {
						p = sigq_genPacket(l, 0, QCMD_MM_FDFUNCS);
						changemask &= ~FUNC_F1_F4;
					} else changemask = 0;
					break;
				case FMT_MM2_14:
				case FMT_MM2_27A:
				case FMT_MM2_27B:
					if (changemask & FUNC_LIGHT) {
						p = sigq_speedPacket(l, l->speed);
						changemask &= ~FUNC_LIGHT;
					} else if (changemask & FUNC(1)) {
						p = sigq_genPacket(l, 0, QCMD_MM_SETF1);
						if (p) p->value.i32 = l->speed & 0xFF;	// the function packets for MM2 also need the speed - else 0 would be transmitted!
						changemask &= ~FUNC(1);
					} else if (changemask & FUNC(2)) {
						p = sigq_genPacket(l, 0, QCMD_MM_SETF2);
						if (p) p->value.i32 = l->speed & 0xFF;	// see above ...
						changemask &= ~FUNC(2);
					} else if (changemask & FUNC(3)) {
						p = sigq_genPacket(l, 0, QCMD_MM_SETF3);
						if (p) p->value.i32 = l->speed & 0xFF;	// see above ...
						changemask &= ~FUNC(3);
					} else if (changemask & FUNC(4)) {
						p = sigq_genPacket(l, 0, QCMD_MM_SETF4);
						if (p) p->value.i32 = l->speed & 0xFF;	// see above ...
						changemask &= ~FUNC(4);
					} else changemask = 0;
					break;
				case FMT_DCC_14:
				case FMT_DCC_28:
				case FMT_DCC_126:
				case FMT_DCC_SDF:
					if (l->loco->fmt == FMT_DCC_14 && (changemask & FUNC_LIGHT)) {	// F0 is included in speed packet for the 14 speed decoders only
						p = sigq_speedPacket(l, l->speed);
						changemask &= ~FUNC_LIGHT;
					} else if (l->loco->fmt == FMT_DCC_14 && (changemask & FUNC_F1_F4)) {
						p = sigq_genPacket(l, 0, QCMD_DCC_SETF1_4);
						changemask &= ~FUNC_F1_F4;
					} else if (changemask & FUNC_F0_F4) {							// this is hit only for 28 and 126 speed steps
						p = sigq_genPacket(l, 0, QCMD_DCC_SETF1_4);
						changemask &= ~FUNC_F0_F4;
					} else if (changemask & FUNC_F5_F8) {
						p = sigq_genPacket(l, 0, QCMD_DCC_SETF5_8);
						changemask &= ~FUNC_F5_F8;
					} else if (changemask & FUNC_F9_F12) {
						p = sigq_genPacket(l, 0, QCMD_DCC_SETF9_12);
						changemask &= ~FUNC_F9_F12;
					} else if (changemask & FUNC_F13_F20) {
						p = sigq_genPacket(l, 0, QCMD_DCC_SETF13_20);
						changemask &= ~FUNC_F13_F20;
					} else if (changemask & FUNC_F21_F28) {
						p = sigq_genPacket(l, 0, QCMD_DCC_SETF21_28);
						changemask &= ~FUNC_F21_F28;
					} else if (changemask & FUNC_F29_F31) {
						p = sigq_genPacket(l, 0, QCMD_DCC_SETF29_36);
						changemask &= ~FUNC_F29_F31;
					} else changemask = 0;
					break;
				case FMT_M3_126:
					if (changemask & FUNC_F0_F15) {
						p = sigq_genPacket(l, 0, QCMD_SETFUNC);
						changemask &= ~FUNC_F0_F15;
					} else if (changemask & FUNC_F16_F31){
						for (f = 16; f < 32; f++) {
							if (changemask & FUNC(f)) {
								p = sigq_genPacket(l, 0, QCMD_M3_SINGLEFUNC);
								if (p) p->param.i32 = f;
								changemask &= ~FUNC(f);
								break;
							}
						}
					} else changemask = 0;
					break;
				default:
					changemask = 0;
					break;
			}
			sigq_queuePacket(p);
		}
		loco_unlock();
		event_fire(EVENT_LOCO_FUNCTION, adr, l);
	} else {
		loco_unlock();
	}
	return 0;
}

int loco_setFunc (int adr, int f, bool on)
{
	return loco_setFuncMasked(adr, (on) ? (1 << f) : 0, 1 << f);
}

int loco_setBinState (int adr, int state, bool on)
{
	ldataT *l;
	struct packet *p = NULL;

	if (adr <= 0 || adr > MAX_LOCO_ADR) return -1;

	if (!loco_lock(__func__)) return -1;

	if ((l = loco_callLocked(adr, true)) != NULL) {
		l->purgeTime = loco_purgetime();
		if (FMT_IS_DCC(l->loco->fmt)) {				// only DCC support binary states
			p = sigq_binStatePacket(l, state, on);
		}
		loco_unlock();
		if (p) {
			sigq_queuePacket(p);
		}
	} else {
		loco_unlock();
	}

	return 0;
}

int loco_getSpeeds (locoT *l)
{
	if (!l) return 0;
	return db_getSpeeds(l->fmt);
}

static int loco_clipSpeed (locoT *l, int speed)
{
	bool rev;
	int maxspeed;

	if (!l) return 0;
	if (speed < 0) return 0;
	rev = (speed & 0x80) == 0;
	speed &= 0x7F;
	maxspeed = loco_getSpeeds(l);
	if (speed > maxspeed) speed = maxspeed;

	return (rev) ? speed : 0x80 | speed;
}

/**
 * Special handling for MM27a locos. We probably must send two different
 * speed steps to get to the intermediate speed step. The intermediate
 * speed steps are all the even speeds, while the odd steps are the native
 * speed.
 *
 * This function is called with the lock already held. It must be released
 * before the packets are queued. If no packets are queued, it still must
 * be released before returning to loco_setSpeed().
 *
 * Functionality of MM27a:
 * If an even speed should be send to the loco, we first must send a higher
 * odd speed and then the lower speed. As the speeds are mangled in sig_mmSpeed()
 * we must precompensate here. Examples:
 *    - Speed=20: send speed 21 (is 12 on the track) and then speed 20 (is 11 on the track)
 *    - Speed=15: send speed 15 (is 9 on the track), no second speed code needs to be sent
 *
 * \param l			the loco data structure with the actual settings
 * \param speed		the new speed (0 or 1 .. 27, either forward or backwards)
 * \see				loco_setSpeed()
 * \see				sig_mmSpeed()
 */
static void loco_MM27aSpeed (ldataT *l, int speed)
{
	struct packet *p, *r, *s;

	s = r = NULL;
	if ((l->speed & 0x7F) && (l->speed & 0x80) != (speed & 0x80)) {	// direction change when running: send emergency stop
		l->speed &= 0x80;
		s = sigq_emergencyStopPacket(l);
	}
	if ((speed & 0x7F) != 0) {					// we have a non-null speed
		if ((speed & 1) == 0) {					// we have an intermediate speed inbetween two real speeds
			r = sigq_speedPacket(l, speed + 1);
		} else if (speed == l->speed - 1) {		// we go down from half step to the lower full step
			r = sigq_speedPacket(l, speed - 1);
		}
	}
	l->speed = speed;
	p = sigq_speedPacket(l, l->speed);
	loco_unlock();
	event_fire(EVENT_LOCO_SPEED, l->loco->adr, l);
	if (s) sigq_queuePacket(s);					// output HALT packet (speed = 0)
	if (r) sigq_queuePacket(r);					// for intermediate speeds: output temporary speed packet
	sigq_queuePacket(p);						// for all: send new speed
}

static int _loco_setSpeed (ldataT *l, int speed)
{
	struct packet *p, *r, *s;

	if (!l || !loco_lock(__func__)) return -1;

//	log_msg (LOG_INFO, "%s(%d) Speed %c%d\n", __func__, l->loco->adr, (speed & 0x80) ? 'F' : 'R', speed & 0x7F);
	l->purgeTime = loco_purgetime();
	speed = loco_clipSpeed(l->loco, speed);
	if (speed != l->speed) {
		if (l->loco->fmt == FMT_MM2_27A) {		// needs special handling
			loco_MM27aSpeed(l, speed);
		} else {
			s = r = NULL;
			if ((l->speed & 0x7F) && (l->speed & 0x80) != (speed & 0x80)) {		// direction change when running: send emergency stop
				s = sigq_emergencyStopPacket(l);
//				} else if (l->loco->fmt == FMT_MM1_14 && (l->speed & 0x80) != (speed & 0x80)) {	// direction change on MM1
			} else if (FMT_IS_MM(l->loco->fmt) && (l->speed & 0x80) != (speed & 0x80)) {	// direction change on MM1
				r = sigq_genPacket(l, 0, QCMD_MM_REVERSE);	// send a REVERSE packet (which in the end is the same as the emergency stop)
				if (r) {
					r->repeat = 10;
					r->value.i32 = l->speed & 0x80;
				}
			}
			l->speed = speed;
			p = sigq_speedPacket(l, l->speed);
			loco_unlock();
			event_fire(EVENT_LOCO_SPEED, l->loco->adr, l);
			if (s) sigq_queuePacket(s);					// output EMERGENCY STOP packet (speed = 0, old direction)
			if (r) sigq_queuePacket(r);					// for MM1: output REVERSE packet
			sigq_queuePacket(p);						// for all: send new speed
		}
	} else {
		loco_unlock();
	}
	return 0;
}

int loco_setSpeed (int adr, int speed)
{
	ldataT *l, *c;
	int rc = 0;

	if (adr <= 0 || adr > MAX_LOCO_ADR) return -1;

	if ((l = loco_call(adr, true)) != NULL) {
		if (l->flags & LOCO_CONSIST_REVERSE) speed ^= 0x80;		// if a reversed loco inside a consist is the source, direction must be reversed
		c = l;
		do {
			rc |= _loco_setSpeed(c, (c->flags & LOCO_CONSIST_REVERSE) ? (speed ^ 0x80) : speed);
			c = c->consist;
		} while (c && c != l);
	}

	return rc;
}

int loco_emergencyStop (int adr)
{
	ldataT *l;
	struct packet *p;


	if (adr <= 0 || adr > MAX_LOCO_ADR) return -1;
	if (!loco_lock(__func__)) return -1;

	if ((l = loco_callLocked(adr, true)) != NULL) {
//		if (l->speed & 0x7F) {
			l->speed &= 0x80;
			p = sigq_emergencyStopPacket(l);
			loco_unlock();
			event_fire(EVENT_LOCO_SPEED, adr, l);
			sigq_queuePacket(p);
//		} else {
//			loco_unlock();
//		}
	} else {
		loco_unlock();
	}
	return 0;
}

/**
 * Scans the refresh list for an m3 loco. If not found, we need not send the
 * m3 beacon and can operate mfx(R) decoders in DCC format without explicitly
 * switch off m3 support.
 */
bool m3_inRefresh (void)
{
	ldataT *l;

	l = locolist;

	while (l) {
		if (l->loco && FMT_IS_M3(l->loco->fmt)) return true;
		l = l->next;
	}
	return false;
}

/**
 * must be called with lock held!
 */
void loco_freeRefreshList(void)
{
	ldataT *l;

	while ((l = locolist) != NULL) {
		locolist = l->next;
		free (l);
	}
}

ldataT *loco_refresh(void)
{
	static ldataT *refresh;					///< a refresh pointer that circulates over all active locos

	if (!loco_lock(__func__)) return NULL;
	if (!refresh || (refresh = refresh->next) == NULL) refresh = locolist;

	struct sysconf *sc;
	sc = cnf_getconfig();
	if (refresh) {
		if (sc->locopurge) {
			if (tim_isover(refresh->purgeTime)) {	// purge time is over ...
				_loco_remove(refresh);
				refresh = NULL;
			} else {
				refresh->age++;
			}
		} else {
			refresh->age++;
		}
	}

	loco_unlock();
	return refresh;
}

/**
 * Iterate over the list of locos that are in the refresh list.
 * The current position in the refreshlist is given as a parameter.
 * We must check out, that this loco really still exists. If not,
 * a derefence of the next-pointer will direct us into garbage.
 * Remember that the list could be modified in between calls to this
 * function and we cannot lock the whole thing for the duration of
 * this enumeration.
 *
 * This way, we should keep in mind, that we possibly reach a premature
 * end of the list. The only other way to overcome this problem would
 * be a complete list copy, that would have to be freed afterwards
 * (something like a hypothecial function locoT *loco_cloneList()).
 *
 * \param cur		the current position in the loco list or NULL if we want to start from the beginning
 * \return			the next loco in the list, if it exists.
 */
ldataT *loco_iterateNext (ldataT *cur)
{
	ldataT *l;

	if (!cur) return locolist;
	if (!loco_lock(__func__)) return NULL;
	l = locolist;
	while (l && l != cur) l = l->next;
	if (l) l = l->next;
	loco_unlock();
	return l;
}
