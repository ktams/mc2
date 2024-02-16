/*
 * consist.c
 *
 *  Created on: 07.04.2021
 *      Author: Andi
 */

/*
 * RB2, next generation model railroad controller software
 * Copyright (C) 2021 Tams Elektronik GmbH and Andreas Kretzer
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

#include "rb2.h"
#include "decoder.h"
#include "events.h"

static struct consist *consists;

/**
 * Count the number of locos in a consist. When the given loco is not
 * in a consist, zero is returned even though you could argue, that it
 * forms a consist of length one with itself.
 *
 * \param l		the loco data structure of the loco toc check consist count
 * \return		0 if the loco is not consisted or 2 .. n for a real consist
 */
static int consist_countLength (ldataT *l)
{
	ldataT *tmp;
	int count = 0;

	if (!l || !l->consist) return 0;
	tmp = l;
	do {
		count++;
		tmp = tmp->consist;
	} while (tmp && tmp != l);
	return count;
}

static bool consist_isInConsist (struct consist *c, int adr)
{
	int i;

	adr = abs(adr);
	if (c) {
		for (i = 0; i < MAX_CONSISTLENGTH; i++) {
			if (adr == abs(c->adr[i])) return true;
		}
	}
	return false;
}

/**
 * Remove the given loco address from the array of the given consist
 * and compact that array.
 *
 * \param c		pointer to the consist in question
 * \param adr	the address that is to be removed from the array
 */
static void consist_removeFromArray (struct consist *c, int adr)
{
	int *p1, *p2;

	if (!c || !adr) return;

	adr = abs(adr);

	for (p1 = p2 = c->adr; p2 < &c->adr[MAX_CONSISTLENGTH]; p2++) {
		if (*p2 && abs(*p2) != adr) *p1++ = *p2;
	}
	while (p1 < &c->adr[MAX_CONSISTLENGTH]) *p1++ = 0;
}

struct consist *consist_findConsist(int adr)
{
	struct consist *c;

	c = consists;
	while (c) {
		if (consist_isInConsist(c, adr)) return c;
		c = c->next;
	}
	return NULL;
}

/**
 * Try to create or expand a consist with the given locos.
 * It must be checked, that the speed parameters match before
 * creating this consist.
 *
 * This is the heart of the function. It is only called directly
 * on system startup to create the read in consists without fire
 * an event or trigger the storage procedure. For runtime purposes,
 * use \ref consist_couple().
 *
 * \param adr1		the address of the first loco (negative if reversed)
 * \param adr2		the address of the second loco (negative if reversed)
 * \return			pointer to the loco data structure of the first loco if constist could be built,
 * 					NULL otherwise
 */
struct consist *_consist_couple (int adr1, int adr2)
{
	locoT *l1, *l2;
	struct consist *c, *c1, *c2, **cpp;
	int i, a;

	log_msg (LOG_INFO, "%s() try %d + %d\n", __func__, adr1, adr2);

	if (!adr1 || !adr2 || (abs(adr1) == abs(adr2))) {
		log_msg (LOG_ERROR, "%s() %d + %d is invalid\n", __func__, adr1, adr2);
		return NULL;			// this is an idiotic coupling, ignore it
	}
	l1 = db_getLoco(abs(adr1), false);
	l2 = db_getLoco(abs(adr2), false);
	if (!l1 || !l2) {
		if (!l1) log_msg (LOG_ERROR, "%s() %d could not be found\n", __func__, adr1);
		if (!l2) log_msg (LOG_ERROR, "%s() %d could not be found\n", __func__, adr2);
		return NULL;											// at least one the locos is unknown
	}
	if (db_getSpeeds(l1->fmt) != db_getSpeeds(l2->fmt)) {
		log_msg (LOG_ERROR, "%s() speeds don't match: %d=%d, %d=%d\n", __func__, adr1, db_getSpeeds(l1->fmt), adr2, db_getSpeeds(l2->fmt));
		return NULL;		// speed steps do not agree
	}
	if (FMT_IS_MM1(l1->fmt) || FMT_IS_MM1(l2->fmt)) {
		if (FMT_IS_MM1(l1->fmt)) log_msg (LOG_ERROR, "%s() %d is in MM1 format\n", __func__, adr1);
		if (FMT_IS_MM1(l2->fmt)) log_msg (LOG_ERROR, "%s() %d is in MM1 format\n", __func__, adr2);
		return NULL;			// MM1 locos cannot build a consist (they are direction agnostic)
	}

	c1 = consist_findConsist(adr1);
	c2 = consist_findConsist(adr2);
	if (c1 && c2 && c1 != c2) return NULL;		// the locos are already in different consists
	if (c1 && c1 == c2) return c1;				// the locos are already in the same consist - that's OK, nothing to do

	// the following is already tailored for consists of more than two locos
	if (!c1 && !c2) {		// none of the locos is in a consist, create a new one with adr1 as the first loco
		if ((c = calloc (1, sizeof(*c))) == NULL) return NULL;		// cannot aquire memory for new consist
		c->adr[0] = adr1;
		a = adr2;
	} else if (c1) {		// adr1 is already in a consist, add adr2 to it
		c = c1;
		a = adr2;
	} else {				// adr2 is already in a consist, add adr1 to it
		c = c2;
		a = adr1;
	}
	for (i = 1; i < MAX_CONSISTLENGTH; i++) {
		if (c->adr[i] == 0) {
			c->adr[i] = a;
			break;
		}
	}
	if (i >= MAX_CONSISTLENGTH) return NULL;		// the consist is already populated with the maximum locos - adding another one is impossible

	if (!c1 && !c2) {		// we just created a new consist, add it to the end of the list
		cpp = &consists;
		while (*cpp != NULL) cpp = &(*cpp)->next;
		*cpp  = c;
	}

	log_msg (LOG_INFO, "%s() %d + %d\n", __func__, l1->adr, l2->adr);

	return c;
}

/**
 * This is the original coupling function for runtime management.
 * If coupling succeeds, it will fire an event and trigger storage
 * of loco information.
 *
 * \param adr1		the address of the first loco (negative if reversed)
 * \param adr2		the address of the second loco (negative if reversed)
 * \return			pointer to the loco data structure of the first loco if constist could be built,
 * 					NULL otherwise
 * \see				_consist_couple()
 */
struct consist *consist_couple (int adr1, int adr2)
{
	struct consist *c;

	// first, break any consist linkage in refresh list
	_consist_unlink(loco_call(abs(adr1), true));
	_consist_unlink(loco_call(abs(adr2), true));
	if ((c = _consist_couple(adr1, adr2)) != NULL) {
		db_triggerStore(__func__);
		event_fire(EVENT_CONSIST, 0, consists);
		loco_call(adr1, false);		// if coupling succeeded, we possibly recall the linkage in the refresh list
	}
	return c;
}
/**
 * This is a variant of the original coupling function. It is used at runtime
 * to form a consist and make sure, that the required locos exist. That is
 * accomplished by calling these locos prior to consist building with the "add"
 * parameter set to true.
 *
 * \param adr1		the address of the first loco (negative if reversed)
 * \param adr2		the address of the second loco (negative if reversed)
 * \return			pointer to the loco data structure of the first loco if constist could be built,
 * 					NULL otherwise
 * \see				consist_couple()
 * \see				_consist_couple()
 */
struct consist *consist_coupleAdd (int adr1, int adr2)
{
	ldataT *l1, *l2;

	// first, force the locos into life
	l1 = loco_call(adr1, true);
	l2 = loco_call(adr2, true);
	if ((l1->speed & 0x80) != (l2->speed & 0x80)) {
		adr2 = -adr2;
	}
	return consist_couple(adr1, adr2);
}

/**
 * Take a loco out of the consist ring list. This function schould only
 * be called, when the loco lock is held.
 *
 * \param l		pointer to the loco refresh data which should be isolated (taken out of the ring)
 */
void _consist_unlink (ldataT *l)
{
	struct consist *c, **cpp;
	ldataT *tmp, *ring;

	if (!l || !l->consist) return;		// no loco or loco not in a consist ring
	ring = l->consist;					// this will be the new next-pointer after the current loco has been taken out
	tmp = l->consist;

	cpp = &consists;
	while ((c = *cpp) != NULL && !consist_isInConsist(c, l->loco->adr)) cpp = &c->next;
	// c is now the consist in question and cpp points to the next-pointer of the previous consist (or consist root pointer)

	while (tmp && tmp->consist != l) tmp = tmp->consist;
	if (tmp) {							// just to be sure that there is no error in ring concatenation ...
		if (tmp == ring) {				// oh! This was a consist with only two locos. This consist is now completely dissolved
			tmp->consist = NULL;
			if (c) {					// but it may already have been removed ... so check for a real consist in c
				*cpp = c->next;				// the consist structure can be unlinked and freed
				free (c);
			}
		} else {						// the consist still exists with at least two locos
			tmp->consist = ring;		// skip over the loco to be removed from consist and close the ring in it's new composition
			consist_removeFromArray(c, l->loco->adr);	// update the array with the remaining loco addresses
		}
	}
	l->consist = NULL;
	db_triggerStore(__func__);
}

/**
 * Dissolve a consist completely. After having cleared the consist
 * here, we also break the consist linkage in the refresh list by
 * calling \ref loco_dissolveConsist().
 *
 * \param adr		the address of any loco inside the consist
 * \return			true, if the consist was removed
 */
bool consist_dissolve (uint16_t adr)
{
	struct consist *c, **cpp;

	if ((c = consist_findConsist(adr)) != NULL) {
		cpp = &consists;
		while (*cpp && *cpp != c) cpp = &(*cpp)->next;
		if (*cpp == c) {
			*cpp = c->next;
			free (c);
		}
		db_triggerStore(__func__);
		event_fire(EVENT_CONSIST, 0, consists);
	}
	_consist_unlink(_loco_getRefreshLink(_db_getLoco (adr, false)));
	return (c != NULL);
}

/**
 * Take a single loco out of a consist.
 *
 * \param adr		the address of the loco to take out of the consist
 * \return			true, if the consist was changed
 */
bool consist_remove (uint16_t adr)
{
	return consist_dissolve(adr);	// currently the same ...
}

void consist_event (void)
{
	event_fire(EVENT_CONSIST, 0, consists);
}

struct consist *consist_getConsists(void)
{
	return consists;
}
