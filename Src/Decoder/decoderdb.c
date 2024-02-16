/**
 * @file decoderdb.c
 *
 * @author Andi
 * @date   26.04.2020
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
#include <string.h>
#include <ctype.h>
#include "rb2.h"
#include "timers.h"
#include "decoder.h"
#include "config.h"
#include "intelhex.h"
#include "events.h"
#include "defaults.h"

#define MAX_FUNCDUMMY		64			///< function dummies with standard settings (all zero except for function number
#define STORAGE_TIMEOUT		pdMS_TO_TICKS(3 * 1000)

static locoT *locodb;					///< all defined locos read from file system
static turnoutT *turnouts;				///< all known turnouts
static extaccT *xaccessories;			///< all known extended accessory decoders
static TimerHandle_t storage_timer;		///< a delay after the last change before the filesystem is updated

static locoT defLoco = {
	.adr = 0,
	.fmt = CNF_DEF_LOCO_FMT,
	.maxfunc = 28,
};

/* currently not used yet - keep compiler happy
static locoT tpmLoco = {	// a special loco for TAMS programming mode
	.adr = 3,
	.fmt = FMT_MM2_14,
	.maxfunc = 4,
};
*/

static turnoutT defTurnout = {
	.adr = 0,
	.fmt = CNF_DEF_TURNOUT_FMT,
};

static funcT fdummies[MAX_FUNCDUMMY];
static int fdummy_idx;

struct keyhandler {
	const char					*key;
	void (*reader)(void *, struct key_value *);
	struct key_value * (*writer)(void *, struct key_value *, const char *);
};

/*
 * Forward declarations of function prototypes for reading
 */
static void db_rdFmt (void *p, struct key_value *kv);
static void db_rdConfig (void *p, struct key_value *kv);
static void db_rdMaxfunc (void *p, struct key_value *kv);
static void db_rdName (void *p, struct key_value *kv);
static void db_rdShortName (void *p, struct key_value *kv);
static void db_rdVendor (void *p, struct key_value *kv);
static void db_rdProduct (void *p, struct key_value *kv);
static void db_rdHWversion (void *p, struct key_value *kv);
static void db_rdFWversion (void *p, struct key_value *kv);
static void db_rdImage (void *p, struct key_value *kv);
static void db_rdAdrReq (void *p, struct key_value *kv);
static void db_rdVID (void *p, struct key_value *kv);
static void db_rdUID (void *p, struct key_value *kv);
static void db_rdIcon (void *p, struct key_value *kv);
static void db_rdFlags (void *p, struct key_value *kv);
static void db_rdFtime (void *p, struct key_value *kv);
static void db_rdTrntFmt (void *p, struct key_value *kv);
static void db_rdTrntUID (void *p, struct key_value *kv);
static void db_rdTrntAspect (void *p, struct key_value *kv);
static void db_rdXaccFmt (void *p, struct key_value *kv);

/*
 * Forward declarations of function prototypes for writing
 */
static struct key_value *db_wrFmt (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrConfig (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrMaxfunc (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrName (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrShortName (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrVendor (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrProduct (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrHWversion (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrFWversion (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrImage (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrAdrReq (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrVID (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrUID (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrIcon (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrFlags (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrFtime (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrTrntFmt (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrTrntUID (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrTrntAspect (void *p, struct key_value *kv, const char *key);
static struct key_value *db_wrXaccFmt (void *p, struct key_value *kv, const char *key);

static const struct keyhandler loco_entries[] = {
	{ "fmt",		db_rdFmt,			db_wrFmt },
	{ "config",		db_rdConfig,		db_wrConfig },
	{ "maxfunc",	db_rdMaxfunc,		db_wrMaxfunc },
	{ "name",		db_rdName,			db_wrName },
	{ "vid",		db_rdVID,			db_wrVID },
	{ "uid",		db_rdUID,			db_wrUID },
	{ "shortname",	db_rdShortName,		db_wrShortName },
	{ "vendor",		db_rdVendor,		db_wrVendor },
	{ "product",	db_rdProduct,		db_wrProduct },
	{ "HW",			db_rdHWversion,		db_wrHWversion },
	{ "FW",			db_rdFWversion,		db_wrFWversion },
	{ "image",		db_rdImage,			db_wrImage },
	{ "icon",		db_rdIcon,			db_wrIcon },
	{ "AdrReq",		db_rdAdrReq,		db_wrAdrReq },
	{ "flags",		db_rdFlags,			db_wrFlags },
	{ "ftime",		db_rdFtime,			db_wrFtime },
	{ NULL,			NULL,				NULL }
};

static const struct keyhandler turnout_entries[] = {
	{ "fmt",		db_rdTrntFmt,		db_wrTrntFmt },
	{ "uid",		db_rdTrntUID,		db_wrTrntUID },
	{ "aspect",		db_rdTrntAspect,	db_wrTrntAspect },
	{ NULL,			NULL,				NULL }
};

static const struct keyhandler extacc_entries[] = {
	{ "fmt",		db_rdXaccFmt,		db_wrXaccFmt },
	{ NULL,			NULL,				NULL }
};

void db_triggerStore (const char *caller)
{
	log_msg (LOG_INFO, "%s(): from %s()\n", __func__, caller);
	if (storage_timer) {
		xTimerReset(storage_timer, 20);
	}
}

void db_freeLocos (void)
{
	locoT *l;
	funcT *f;

	loco_lock(__func__);
	loco_freeRefreshList();
	while ((l = locodb) != NULL) {
		locodb = l->next;
		while ((f = l->funcs) != NULL) {
			l->funcs = f->next;
			free (f);
		}
		free (l);
	}
	loco_unlock();
}

void db_freeTurnouts (void)
{
	turnoutT *t;

	loco_lock(__func__);
	loco_freeRefreshList();
	while ((t = turnouts) != NULL) {
		turnouts = t->next;
		free (t);
	}
	loco_unlock();
}

/**
 * must not be called with lock held!
 */
static void db_freeDB (void)
{
	db_freeLocos();
	db_freeTurnouts();
}

/* ========================================================================================
 * Handling of loco decoders
 * ======================================================================================== */

/**
 * Remove a given loco (format-)definition from the list of known entries.
 * This is an internal function and should only be called, if the lock is held.
 *
 * @param l		pointer to the loaded loco definition.
 */
static void _db_removeLoco (locoT *l)
{
	locoT **lpp;
	funcT *f;

	if (!l || l == &defLoco) return;

	// first step: remove the possible reference from the refreshlist to this loco
	_loco_remove (_loco_getRefreshLink(l));

	// second step: remove the loco definition itself from the list of known locos
	lpp = &locodb;
	while (*lpp && *lpp != l) lpp = &(*lpp)->next;
	if (*lpp == l) {
		*lpp = l->next;
		consist_dissolve(l->adr);		// if this loco was in a consist, we should take it out of the consist
		log_msg (LOG_INFO, "%s(): LOCO %d removed\n", __func__, l->adr);
		while ((f = l->funcs) != NULL) {
			l->funcs = f->next;
			free (f);
		}
		free (l);
	}
}

static enum fmt db_defaultFormat (int adr)
{
	enum fmt fmt;

	fmt = defLoco.fmt;
	if (adr > MAX_MM_ADR && FMT_IS_MM(fmt)) fmt = FMT_DCC_28;
	if (adr > MAX_DCC_ADR) fmt = FMT_M3_126;
	return fmt;
}

/**
 * Look up the function structure of a loco describing the given function.
 *
 * @param l		the loco structure holding the data describing a loco
 * @param func	the numeric function that will be looked up
 * @return		a pointer to the found function description or NULL, if it was not found
 */
static funcT *_db_getLocoFunc (locoT *l, int func)
{
	funcT *f;

	if (!l || func < 0) return NULL;
	f = l->funcs;
	while (f && f->fnum != func) f = f->next;
	return f;
}

funcT *db_getLocoFunc (locoT *l, int func)
{
	funcT *f;

	if ((f = _db_getLocoFunc(l, func)) == NULL) {
		if (fdummy_idx < 0 || fdummy_idx >= MAX_FUNCDUMMY) fdummy_idx = 0;
		f = &fdummies[fdummy_idx++];
		memset (f, 0, sizeof(*f));
		f->fnum = func;
	}
	return f;
}

static funcT *db_addLocoFunc (locoT *l, int func)
{
	funcT *f, **fp;

	if (!l || func < 0 || func > LOCO_MAX_FUNCS) return NULL;
	if ((f = _db_getLocoFunc(l, func)) != NULL) return f;	// already exists - return this existing structure
	if ((f = calloc (1, sizeof(*l))) == NULL) return NULL;	// no memory
	f->fnum = func;

	// now insert sorted into the function list
	fp = &l->funcs;
	while ((*fp) && (*fp)->fnum < func) fp = &(*fp)->next;
	f->next = *fp;
	*fp = f;
	return f;
}

static void _db_locoFuncIcon (locoT *l, int func, int icon)
{
	funcT *f;

	if (icon < 0 || icon > MAX_ICON_INDEX) return;		// icon outside range
	if ((f = db_addLocoFunc(l, func)) == NULL) return;	// could not get a function structure - call is ignored
	f->icon = (uint16_t) icon;
}

void db_locoFuncIcon (locoT *l, int func, int icon)
{
	_db_locoFuncIcon (l, func, icon);
	event_fire(EVENT_LOCO_PARAMETER, l->adr, l);
	db_triggerStore(__func__);
}

/**
 * Specify a function timing in 1/10s
 * Special values:
 *   -1 = momentary switch
 *    0 = toggle switch (default)
 *    all other values are in 1/10s and stored as ms in the structure
 *
 * @param l		the loco structure holding the data describing a loco
 * @param func	the numeric function that will be looked up
 * @param tim	the time to set in 1/10s or the special values -1 and 0
 */
static void _db_locoFuncTiming (locoT *l, int func, int tim)
{
	funcT *f;

	if (tim < -1 || tim > 1000) return;					// timing outside range
	if ((f = db_addLocoFunc(l, func)) == NULL) return;	// could not get a function structure - call is ignored
	f->timing = tim * 100;								// switch to ms
}

void db_locoFuncTiming (locoT *l, int func, int tim)
{
	_db_locoFuncTiming (l, func, tim);
	db_triggerStore(__func__);
}

static int db_nameCompare (const void *p1, const void *p2)
{
	locoT *l1, *l2;
	int rc;

	l1 = *((locoT **) p1);
	l2 = *((locoT **) p2);

	rc = strcmp(l1->name, l2->name);
	if (rc == 0) rc = l1->adr - l2->adr;	// if the names are the same, sort by address
	return rc;
}

/**
 * Genenrate a sorted list of available locos and return that list.
 *
 * \return		a sorted list of pointers pointing to the defined locos (this array of pointers must be freed after use)
 */
static locoT **db_sortedList (void)
{
	locoT **l, *p;
	int listlen, i;

	listlen = list_len(locodb);
	if ((l = calloc(listlen, sizeof(*l))) == NULL) return NULL;

	for (i = 0, p = locodb; p && (i < listlen); i++, p = p->next) l[i] = p;
	qsort(l, listlen, sizeof(*l), db_nameCompare);
	return l;
}

/**
 * Calculate the next index for loco data base access.
 * This index is 0-based and includes all known lococs.
 * To find the first loco having a name (convinience for EasyNet)
 * look out for loco index -1.
 *
 * \param idx	the current 0-based index in the sorted list or -1 to search for the first loco having a name
 * \return		the 0-based index of the next loco (or first loco having a name) - takes wrap-around into account
 */
int db_indexSorted_next (int idx)
{
	locoT **l;
	int listlen;

	listlen = list_len(locodb);
	if (idx < 0) {		// search for first (named) loco
		if ((l = db_sortedList()) == NULL) return 0;
		for (idx = 0; idx < listlen; idx++) {
			if (*l[idx]->name) {
				free (l);
				return idx;
			}
		}
		free (l);
		return 0;
	}

	if ((idx + 1) >= listlen) return 0;	// wrap-around to start of list
	return idx + 1;
}

/**
 * Calculate the preious index for loco data base access.
 * This index is 0-based and includes all known lococs.
 *
 * \param idx	the current 0-based index in the sorted list
 * \return		the 0-based index of the next loco - takes wrap-around into account
 */
int db_indexSorted_prev (int idx)
{
	int listlen;

	listlen = list_len(locodb);
	if ((idx - 1) < 0) return listlen - 1;	// wrap-around to end of list
	return idx - 1;
}

locoT *db_lookupLocoSorted (int idx)
{
	locoT **l, *p;
	int listlen;

	if (idx < 0) return NULL;
	listlen = list_len(locodb);
	if (idx >= listlen) return NULL;
	if ((l = db_sortedList()) == NULL) return NULL;
	p = l[idx];
	free (l);
	return p;
}

/**
 * Inverted function to db_lookupLocoSorted(): lookup the index that
 * the loco (given as parameter) would have in the sorted list or -1 if
 * loco is not found.
 *
 * \param loco		the loco structure for which we search the DB index
 * \return			the 0-based index of the loco or -1
 */
int db_lookupIndex (locoT *loco)
{
	locoT **l;
	int listlen, idx;

	listlen = list_len(locodb);
	if ((l = db_sortedList()) == NULL) return -1;
	for (idx = 0; idx < listlen && l[idx] != loco; idx++) ;
	free (l);
	return (idx >= listlen) ? -1 : idx;
}

int db_getSpeeds (enum fmt fmt)
{
	switch (fmt) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_DCC_14:
			return 14;
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			return 27;
		case FMT_DCC_28:
			return 28;
		case FMT_DCC_126:
		case FMT_DCC_SDF:
		case FMT_M3_126:
			return 126;
		default:
			return 0;
	}
}

/**
 * Add the given loco structure to the list of known locos.
 * The loco definition may add or replace an existant one in the list.
 * The loco is inserted sorted by loco address.
 *
 * This function should only be called, when the lock is held.
 *
 * \param l		the loco to add (if NULL is supplied, this is a NOP)
 * \return		the loco structure pointer (same as the parameter l)
 */
static locoT *_db_addLoco (locoT *l)
{
	locoT **lpp, *old;

	if (!l) return NULL;

	lpp = &locodb;
	while (*lpp && (*lpp)->adr < l->adr) lpp = &(*lpp)->next;
	if (*lpp && (*lpp)->adr == l->adr) {		// we have an entry with the same loco address - remove it!
		// remove old entry with same number from loco list
		old = *lpp;
		l->next = old->next;
		*lpp = l;
		_db_removeLoco(old);
	} else {
		l->next = (*lpp);
		*lpp = l;
	}
	return l;
}

/**
 * A stub function for _db_addLoco() that first aquires the lock and releases
 * it afterwards.
 *
 * \param l		the preallocated structure that should be inserted into the list of existing locos
 * \return		the loco structure that now is inserted into the global list (same as the parameter)
 * \see			_db_addLoco()
 */
static locoT *db_addLoco (locoT *l)
{
	loco_lock(__func__);
	l = _db_addLoco(l);
	loco_unlock();
	return l;
}

/**
 * Check this loco definition for sane values and correct the if necessary
 */
locoT *db_locoSanitize (locoT *l)
{
	if (!l) return NULL;
	if (l->adr < 0 || l->adr > MAX_LOCO_ADR) {
		_db_removeLoco(l);	// IDs < 0 or beyond the supported address range are simply not allowed!
		return NULL;
	}

	switch (l->fmt) {
		case FMT_MM1_14:
		case FMT_MM2_14:
		case FMT_MM2_27A:
		case FMT_MM2_27B:
			if (l->adr > MAX_MM_ADR) {
				_db_removeLoco(l);			// for MM, addresses are most limited
				return NULL;
			}
			if (l->maxfunc > 4) l->maxfunc = 4;
			break;
		case FMT_DCC_14:
		case FMT_DCC_28:
		case FMT_DCC_126:
		case FMT_DCC_SDF:
			if (l->adr > MAX_DCC_ADR) {
				_db_removeLoco(l);			// for DCC, the address range is wider but still limited
				return NULL;
			}
			if (l->maxfunc > 28) l->maxfunc = 31;
			break;
		case FMT_M3_126:
			if (l->maxfunc >= LOCO_MAX_FUNCS) l->maxfunc = LOCO_MAX_FUNCS - 1;
			break;
		default:
			_db_removeLoco(l);
			return NULL;
	}
	return l;
}

/**
 * Looks up the description of a loco with the given ID.
 * If no such loco ID is found, NULL is returned.
 * See \ref db_getLoco() to automatically create a loco with the standard
 * format if the requested loco address is not known yet.
 *
 * @param adr	the number (ID) of the loco description to look up
 * @return		a pointer to the structure holding the loco definition
 */
static locoT *db_lookupLoco (int adr)
{
	locoT *l;

	if (adr == 0) return &defLoco;

	if (adr < MIN_LOCO_ADR || adr > MAX_LOCO_ADR) return NULL;

	l = locodb;
	while (l && l->adr != adr) l = l->next;
	return l;
}

/**
 * Looks up the description of a loco with the given ID.
 * If no such loco ID is found, a new one is created with the default
 * settings and the timer for storage of loco db is started.
 *
 * The loco "dictionary" is a sorted list (sorted by ID) of all known
 * locos in the system. It does not imply, that this loco is part of the
 * actual refresh list. Instead, the @ref locolist forms the refresh list and
 * their members point to the dictionary entries (at most one loco from
 * the locolist may point to a single dictionary entry).
 *
 * This function must be called with mutex held!
 *
 * \param adr	the number (ID) of the loco description to look up
 * \param add	if the loco is not known, create a new one with default parameters
 * \return		a pointer to the structure holding the loco definition
 */
locoT *_db_getLoco (int adr, bool add)
{
	locoT *l;

	if ((l = db_lookupLoco(adr)) == NULL && adr >= MIN_LOCO_ADR && adr <= MAX_LOCO_ADR && add) {	// create a new loco
		if ((l = calloc (1, sizeof(*l))) != NULL) {
			l->fmt = db_defaultFormat (adr);
			l->maxfunc = defLoco.maxfunc;
			l->adr = adr;
			l = _db_addLoco(l);
		}
	}

	return l;
}

locoT *db_getLoco (int adr, bool add)
{
	locoT *l = NULL;

	if (adr == 0) return &defLoco;	// shortcut the request for the default format loco

	if (loco_lock(__func__)) {
		l = _db_getLoco(adr, add);
		loco_unlock();
	}
	return l;
}

/**
 * Create a loco entry with a free address. This can be used with DCC-A or m3
 * automatic registration functions.
 *
 * \param base		the lowest address to search from
 * \return			pointer to an allocated loco structure with filled-in address and default format
 */
locoT *db_addFreeAdr (int base)
{
	locoT *l;

	loco_lock(__func__);
	l = locodb;
	while (l && l->adr < base) l = l->next;
	while (l && l->adr == base) {	// find a gap
		l = l->next;
		base++;
	}
	if (!l || l->adr > base) {
		if ((l = calloc (1, sizeof(*l))) != NULL) {
			l->fmt = db_defaultFormat (base);
			l->maxfunc = defLoco.maxfunc;
			l->adr = base;
			l = _db_addLoco(l);
		}
	} else {
		l = NULL;		// just in case ...
	}
	loco_unlock();
	return l;
}

/**
 * Try to find a loco entry with the given vendor ID and UID.
 * Because a lot of locos will have no IDs at all, a search for
 * UID==0 is not allowed and will return a NULL pointer immediately.
 * If VID is not specified (VID=0) then the match is done only on
 * the UID.
 *
 * \param vid		the vendor ID to look up (may be null)
 * \param uid		the UID of the decoder to look up (must not be null)
 * \return			a pointer to the structure holding the loco definition or NULL if not found
 */
locoT *db_findLocoUID (uint32_t vid, uint32_t uid)
{
	locoT *l;

	if (!uid) return NULL;

	loco_lock(__func__);
	l = locodb;
	while (l && ((vid && l->vid != vid) || l->uid != uid)) l = l->next;
	loco_unlock();
	return l;
}

locoT *db_changeAdr (int adr, uint32_t vid, uint32_t uid)
{
	locoT *l, **lpp;

	loco_lock(__func__);
	if ((l = db_lookupLoco(adr)) != NULL) {
		loco_unlock();
		if (l->uid == uid && l->vid == vid) return l;	// nothing to do
		return NULL;									// adr is already in use by a different loco
	}
	if ((l = db_findLocoUID(vid, uid)) != NULL) {
		lpp = &locodb;
		while (*lpp && *lpp != l) lpp = &(*lpp)->next;
		if (*lpp) {		// take loco out of list and re-insert at the right position
			(*lpp) = l->next;
			l->next = NULL;
			l->adr = adr;
			_db_addLoco(l);
		}
	}
	loco_unlock();

	return l;
}

void db_setLocoFmt (int adr, enum fmt fmt)
{
	locoT *l;

	log_msg (LOG_WARNING, "%s() ADR=%d new format %s\n", __func__, adr, db_fmt2string(fmt));
	if (FMT_IS_MM(fmt) && adr > MAX_MM_ADR) return;
	if (FMT_IS_DCC(fmt) && adr > MAX_DCC_ADR) return;
	if (FMT_IS_M3(fmt) && adr > MAX_M3_ADR) return;

	loco_lock(__func__);
	if ((l = _db_getLoco(adr, true)) != NULL) {
		if (l->fmt != fmt) {
			l->fmt = fmt;
			db_locoSanitize(l);
			db_triggerStore(__func__);
			event_fire(EVENT_LOCO_PARAMETER, adr, l);
		}
	}
	loco_unlock();
}

void db_setLocoVID (int adr, uint32_t vid)
{
	locoT *l;

	loco_lock(__func__);
	if ((l = _db_getLoco(adr, true)) != NULL) {
		if (l->vid != vid) {
			l->vid = vid;
			db_locoSanitize(l);
			db_triggerStore(__func__);
		}
	}
	loco_unlock();
}

void db_setLocoUID (int adr, uint32_t uid)
{
	locoT *l;

	loco_lock(__func__);
	if ((l = _db_getLoco(adr, true)) != NULL) {
		if (l->uid != uid) {
			l->uid = uid;
			db_locoSanitize(l);
			db_triggerStore(__func__);
		}
	}
	loco_unlock();
}

void db_setLocoMaxfunc (int adr, int maxfunc)
{
	locoT *l;

	loco_lock(__func__);
	if ((l = db_lookupLoco(adr)) != NULL) {
		if (l->maxfunc != maxfunc) {
			l->maxfunc = maxfunc;
			db_locoSanitize(l);
			db_triggerStore(__func__);
			event_fire(EVENT_LOCO_PARAMETER, adr, l);
		}
	}
	loco_unlock();
}

/**
 * Setting the name of a loco.
 *
 * \param adr		the address of the loco to set the name
 * \param name		the new name of the loco - a NULL string is treated as equivalent to an empty string
 */
void db_setLocoName (int adr, char *name)
{
	locoT *l;

	loco_lock(__func__);
	if ((l = _db_getLoco(adr, true)) != NULL) {
		if (!name) name = "";			// to ease handling in this function and make sure, that name is not a NULL pointer
		if (strcmp(l->name, name)) {	// names are different - so update our DB
			strncpy (l->name, name, sizeof(l->name));
			l->name[sizeof(l->name) - 1] = 0;			// force a null terminated string
			db_triggerStore(__func__);
			event_fire(EVENT_LOCO_PARAMETER, adr, l);
		}
	}
	loco_unlock();
}

locoT *db_newLoco (int adr, enum fmt fmt, int maxfunc, char *name, char *uid)
{
	locoT *l;

	loco_lock(__func__);
	if ((l = _db_getLoco(adr, true)) != NULL) {
		l->fmt = fmt;
		l->maxfunc = maxfunc;
		if (name) strncpy (l->name, name, sizeof(l->name));
		if (uid) {
			if (strlen(uid) == 10) {
				if (!strncasecmp(uid,"0x",2)){
					l->uid = strtoul(uid, NULL, 0);
				}
			}
		}
		db_locoSanitize(l);
		event_fire(EVENT_LOCO_DB, 0, NULL);
		db_triggerStore(__func__);
	}
	loco_unlock();
	return l;
}

void db_removeLoco (locoT *l)
{
	if (loco_lock(__func__)) {
		_db_removeLoco (l);
		loco_unlock();
	}
	db_triggerStore(__func__);
}

/* ========================================================================================
 * Handling of turnout decoders
 * ======================================================================================== */

/**
 * Add the given turnout structure to the list of known turnouts.
 * The turnout definition may add or replace an existant one in the list.
 * The turnout is inserted sorted by turnout address.
 *
 * This function should only be called, when the lock is held.
 *
 * \param t		the turnout to add (if NULL is supplied, this is a NOP)
 * \return		the turnout structure pointer (same as the parameter t)
 */
static turnoutT *db_addTurnout (turnoutT *t)
{
	turnoutT **tpp, *old;

	if (!t) return NULL;

	tpp = &turnouts;
	while (*tpp && (*tpp)->adr < t->adr) tpp = &(*tpp)->next;
	if (*tpp && (*tpp)->adr == t->adr) {		// we have an entry with the same turnout address - remove it!
		// remove old entry with same number from turnout list
		old = *tpp;
		t->next = old->next;
		*tpp = t;
		free (old);
	} else {
		t->next = (*tpp);
		*tpp = t;
	}
	return t;
}

/**
 * Remove a given turnout definition from the list of known entries.
 * This is an internal function and should only be called, if the lock is held.
 *
 * @param t		pointer to the loaded turnout definition.
 */
static void _db_removeTurnout (turnoutT *t)
{
	turnoutT **tpp;

	if (!t || t == &defTurnout) return;

	tpp = &turnouts;
	while (*tpp && *tpp != t) tpp = &(*tpp)->next;
	if (*tpp == t) *tpp = t->next;
	free (t);
}

/**
 * Check this turnout definition for sane values and correct the if necessary
 */
turnoutT *db_turnoutSanitize (turnoutT *t)
{
	if (!t) return NULL;
	if (t->adr < 0 || t->adr > MAX_TURNOUT) {
		_db_removeTurnout(t);
		return NULL;
	}

	switch (t->fmt) {
		case TFMT_MM:
			if (t->adr > MAX_MM_TURNOUT) {		// for MM, addresses are most limited
				_db_removeTurnout(t);
				return NULL;
			}
			break;
		case TFMT_DCC:
			if (t->adr > MAX_DCC_ACCESSORY) {	// for DCC, the address range is wider
				_db_removeTurnout(t);
				return NULL;
			}
			break;
		case TFMT_BIDIB:
//			log_msg (LOG_INFO, "%s(BiDiB) T%d: %s %d\n",__func__, t->adr, bidib_formatUID(t->uid), t->aspect);
			if ((t->uid[0] & (BIDIB_CLASS_ACCESSORY | BIDIB_CLASS_SWITCH)) == 0) {		// this node cannot switch turnouts / accessories
				_db_removeTurnout(t);
				return NULL;
			}
			if (t->aspect > 127) {				// aspects on BiDiB nodes may only use numbers 0 .. 127
				_db_removeTurnout(t);
				return NULL;
			}
			break;
		default:
			_db_removeTurnout(t);
			return NULL;
	}

	return t;
}

/**
 * Looks up the description of a turnout with the given ID.
 * If no such turnout ID is found, NULL is returned.
 * See \ref db_getTurnout() to automatically create a turnout with the standard
 * format if the requested turnout address is not known yet.
 *
 * @param adr	the number (ID) of the turnout to look up
 * @return		a pointer to the structure holding the turnout definition
 */
turnoutT *db_lookupTurnout (int adr)
{
	turnoutT *t;

	if (adr == 0) return &defTurnout;

	if (adr < MIN_TURNOUT || adr > MAX_TURNOUT) return NULL;

	t = turnouts;
	while (t && t->adr != adr) t = t->next;
	return t;
}

/**
 * Looks up the description of a BiDiB turnout with the given UID
 * ans aspect number.
 * If no such turnout ID is found, NULL is returned.
 *
 * @param uid		the UID of the BiDiB node which controls the turnout to look up
 * @param aspect	the aspect (index number, 0 .. 127) of the output on this node
 * @return			a pointer to the structure holding the turnout definition
 */
turnoutT *db_lookupBidibTurnout (uint8_t *uid, int aspect)
{
	turnoutT *t;

	if (!uid || aspect < 0 || aspect > 127) return NULL;

	t = turnouts;
	while (t) {
		if (t->fmt == TFMT_BIDIB && !memcmp(&t->uid[2], &uid[2], BIDIB_UID_LEN - 2) && t->aspect == aspect) return t;
		t = t->next;
	}
	return NULL;
}

/**
 * Clear all BiDiB-Mappings for turnouts that are mapped to the given BiDiB UID.
 * The format is then set to the default-Format.
 *
 * \param uid		the UID of the BiDiB node to serach for (only the vendor ID and serial number are compared)
 * \return			true, if at least one turnout was changed so that the turnouts should be store in config
 */
bool db_clearBidibTurnout (uint8_t *uid)
{
	turnoutT *t;
	bool changed = false;

	if (uid) {
		t = turnouts;
		while (t) {
			if (!memcmp(&t->uid[2], &uid[2], BIDIB_UID_LEN - 2)) {
				changed = true;
				memset (t->uid, 0, sizeof(t->uid));
				if (t->fmt == TFMT_BIDIB) t->fmt = defTurnout.fmt;
				t->aspect = 0;
			}
			t = t->next;
		}
	}
	return changed;
}

/**
 * Looks up the description of a turnout with the given ID.
 * If no such turnout ID is found, a new one is created with the default
 * settings and the timer for storage of DB is started.
 *
 * The turnout "dictionary" is a sorted list (sorted by ID) of all known
 * turnouts in the system.
 *
 * This function must be called with mutex held!
 *
 * @param adr	the number (ID) of the turnout to look up
 * @return		a pointer to the structure holding the turnout definition
 */
turnoutT *db_getTurnout (int adr)
{
	turnoutT *t;

	if ((t = db_lookupTurnout(adr)) == NULL && adr >= MIN_TURNOUT && adr <= MAX_TURNOUT) {	// create a new Turnout
		if ((t = calloc (1, sizeof(*t))) != NULL) {
			t->fmt = defTurnout.fmt;
			if (adr > MAX_MM_TURNOUT && t->fmt == TFMT_MM) t->fmt = TFMT_DCC;
			t->adr = adr;
			t = db_addTurnout(t);
		}
	}
	return t;
}

/**
 * Specify the format of a turnout.
 *
 * \param adr	the turnout address in the range of 1..1024 (MM or DCC) and 1025..2044 (DCC only)
 * \param fmt	the new format of that turnout (TFMT_MM or TFMT_DCC)
 */
void db_setTurnoutFmt (int adr, enum fmt fmt)
{
	turnoutT *t;

	if ((fmt == TFMT_MM) && (adr > MAX_MM_TURNOUT)) return;
	if ((fmt == TFMT_DCC) && (adr > MAX_DCC_ACCESSORY)) return;

	loco_lock(__func__);
	if ((t = db_getTurnout(adr)) != NULL) {
		t->fmt = fmt;
		db_turnoutSanitize(t);
		if (adr == 0) {		// global setting of all turnouts to the given format
			t = turnouts;
			while (t && t->adr <= MAX_MM_TURNOUT) {		// implicitly all turnouts beyond MAX_MM_TURNOUT are at TFMT_DCC
				t->fmt = fmt;
				t = t->next;
			}
		}
		db_triggerStore(__func__);
	}
	loco_unlock();
	event_fire (EVENT_ACCFMT, 0, NULL);
}

/* ========================================================================================
 * Handling of extended accessory decoders
 * ======================================================================================== */

/**
 * Add the given extended accessory structure to the list of known extended accessory decoders.
 * The extended accessory definition may add or replace an existant one in the list.
 * The extended accessory is inserted sorted by extended accessory address.
 *
 * This function should only be called, when the lock is held.
 *
 * \param x		the extended accessory to add (if NULL is supplied, this is a NOP)
 * \return		the extended accessory structure pointer (same as the parameter x)
 */
static extaccT *db_addExtacc (extaccT *x)
{
	extaccT **xpp, *old;

	if (!x) return NULL;

	xpp = &xaccessories;
	while (*xpp && (*xpp)->adr < x->adr) xpp = &(*xpp)->next;
	if (*xpp && (*xpp)->adr == x->adr) {		// we have an entry with the same extended accessory address - remove it!
		// remove old entry with same number from extended accessory list
		old = *xpp;
		x->next = old->next;
		*xpp = x;
		free (old);
	} else {
		x->next = (*xpp);
		*xpp = x;
	}
	return x;
}

/**
 * Remove a given extended accessory definition from the list of known entries.
 * This is an internal function and should only be called, if the lock is held.
 *
 * @param x		pointer to the loaded extended accessory definition.
 */
static void _db_removeExtacc (extaccT *x)
{
	extaccT **xpp;

	if (!x) return;

	xpp = &xaccessories;
	while (*xpp && *xpp != x) xpp = &(*xpp)->next;
	if (*xpp == x) *xpp = x->next;
	free (x);
}

/**
 * Check this extended accessory definition for sane values and correct the if necessary
 */
extaccT *db_extaccSanitize (extaccT *x)
{
	if (!x) return NULL;
	if (x->adr <= 0 || x->adr > MAX_DCC_EXTACC) {
		_db_removeExtacc(x);
		return NULL;
	}

	switch (x->fmt) {
		case TFMT_DCC:	// currently nothing more to check
			break;
		default:
			_db_removeExtacc(x);
			return NULL;
	}

	return x;
}

/**
 * Looks up the description of an extended accessory with the given ID.
 * If no such extended accessory ID is found, NULL is returned.
 * See \ref db_getExtacc() to automatically create a extended accessory with the standard
 * format if the requested extended accessory address is not known yet.
 *
 * @param adr	the number (ID) of the extended accessory to look up
 * @return		a pointer to the structure holding the extended accessory definition
 */
extaccT *db_lookupExtacc (int adr)
{
	extaccT *x;

	if (adr <= 0 || adr > MAX_DCC_EXTACC) return NULL;

	x = xaccessories;
	while (x && x->adr != adr) x = x->next;
	return x;
}

/**
 * Looks up the description of a extended accessory with the given ID.
 * If no such extended accessory ID is found, a new one is created with the default
 * settings and the timer for storage of DB is started.
 *
 * The extended accessory "dictionary" is a sorted list (sorted by ID) of all known
 * extended accessory in the system.
 *
 * This function must be called with mutex held!
 *
 * @param adr	the number (ID) of the extended accessory to look up
 * @return		a pointer to the structure holding the extended accessory definition
 */
extaccT *db_getExtacc (int adr)
{
	extaccT *x;

	if ((x = db_lookupExtacc(adr)) == NULL && adr >= 1 && adr <= MAX_DCC_EXTACC) {	// create a new extended accessory decoder
		if ((x = calloc (1, sizeof(*x))) != NULL) {
			x->fmt = TFMT_DCC;
			x->adr = adr;
			x = db_addExtacc(x);
		}
	}
	return x;
}

/* ========================================================================================
 * Format handling, storage, etc.
 * ======================================================================================== */

struct fmt_code {
	enum fmt	 fmt;
	const char	*string;
};

static const struct fmt_code fmt_match[] = {
	{ FMT_MM1_14,	"MM1/14" },
	{ FMT_MM1_14,	"MM1" },
	{ FMT_MM2_14,	"MM2/14" },
	{ FMT_MM2_27A,	"MM2/27A" },
	{ FMT_MM2_27B,	"MM2/27B" },
	{ FMT_DCC_14,	"DCC/14" },
	{ FMT_DCC_28,	"DCC/28" },
	{ FMT_DCC_126,	"DCC/126" },
	{ FMT_DCC_SDF,	"DCC/SDF" },
	{ FMT_M3_126,	"m3/126" },
	{ TFMT_MM,		"MM" },
	{ TFMT_DCC,		"DCC" },
	{ TFMT_BIDIB,	"BiDiB" },
	{ FMT_UNKNOWN,	NULL }
};

enum fmt db_string2fmt (char *s)
{
	const struct fmt_code *f;
	int len;

	if (!s) return FMT_UNKNOWN;
	while (*s && isspace(*s)) s++;

	f = fmt_match;
	while (f->string != NULL) {
		len = strlen(f->string);
		if (!strncasecmp (f->string, s, len)) break;
		f++;
	}
	return f->fmt;
}

const char *db_fmt2string(enum fmt format)
{
	const struct fmt_code *f;

	f = fmt_match;
	while (f->string != NULL) {
		if (f->fmt == format) return f->string;
		f++;
	}

	return "";
}

/* ========================================================================================
 * INI file handling functions
 * ======================================================================================== */

static void db_rdFmt (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	l->fmt = db_string2fmt(kv->value);
}

static struct key_value *db_wrFmt (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;

	return kv_add(kv, key, db_fmt2string(l->fmt));
}

static void db_rdConfig (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	l->config = CONF_MANUAL;	// the default!
	if (kv->value) {
		if (!strncasecmp ("DCCA", kv->value, 4)) l->config = CONF_DCCA;
		else if (!strncasecmp ("M3", kv->value, 2)) l->config = CONF_M3;
		else if (!strncasecmp ("RC+", kv->value, 3)) l->config = CONF_RAILCOMPLUS;
	}
}

static struct key_value *db_wrConfig (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	char *cfg;

	switch (l->config) {
		case CONF_DCCA: cfg = "DCCA"; break;
		case CONF_M3: cfg = "M3"; break;
		case CONF_RAILCOMPLUS: cfg = "RC+"; break;
		case CONF_MANUAL:
		default: return kv;		// no entry means "MANUAL"
	}
	return kv_add(kv, key, cfg);
}

static void db_rdMaxfunc (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	l->maxfunc = atoi(kv->value);
}

static struct key_value *db_wrMaxfunc (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	char tmp[32];

	sprintf (tmp, "%d", l->maxfunc);
	return kv_add(kv, key, tmp);
}

static void db_rdName (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!kv->value) *l->name = 0;
	else strncpy (l->name, kv->value, sizeof(l->name));
	l->name[sizeof(l->name) - 1] = 0;		// if string from ini is too long
}

static struct key_value *db_wrName (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;

	if (! *l->name) return NULL;
	return kv_add(kv, key, l->name);
}

static void db_rdShortName (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));
	if (l->dcca) {
		if (!kv->value) *l->dcca->shortname = 0;
		else strncpy (l->dcca->shortname, kv->value, sizeof(l->dcca->shortname));
		l->dcca->shortname[sizeof(l->dcca->shortname) - 1] = 0;		// if string from ini is too long
	}
}

static struct key_value *db_wrShortName (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;

	if (!l->dcca || ! *l->dcca->shortname) return NULL;
	return kv_add(kv, key, l->dcca->shortname);
}

static void db_rdVendor (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));
	if (l->dcca) {
		if (!kv->value) *l->dcca->vendor = 0;
		else strncpy (l->dcca->vendor, kv->value, sizeof(l->dcca->vendor));
		l->dcca->vendor[sizeof(l->dcca->vendor) - 1] = 0;		// if string from ini is too long
	}
}

static struct key_value *db_wrVendor (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;

	if (!l->dcca || ! *l->dcca->vendor) return NULL;
	return kv_add(kv, key, l->dcca->vendor);
}

static void db_rdProduct (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));
	if (l->dcca) {
		if (!kv->value) *l->dcca->product = 0;
		else strncpy (l->dcca->product, kv->value, sizeof(l->dcca->product));
		l->dcca->product[sizeof(l->dcca->product) - 1] = 0;		// if string from ini is too long
	}
}

static struct key_value *db_wrProduct (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;

	if (!l->dcca || ! *l->dcca->product) return NULL;
	return kv_add(kv, key, l->dcca->product);
}

static void db_rdHWversion (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));
	if (l->dcca) {
		if (!kv->value) *l->dcca->hw_version = 0;
		else strncpy (l->dcca->hw_version, kv->value, sizeof(l->dcca->hw_version));
		l->dcca->hw_version[sizeof(l->dcca->hw_version) - 1] = 0;		// if string from ini is too long
	}
}

static struct key_value *db_wrHWversion (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;

	if (!l->dcca || ! *l->dcca->hw_version) return NULL;
	return kv_add(kv, key, l->dcca->hw_version);
}

static void db_rdFWversion (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));
	if (l->dcca) {
		if (!kv->value) *l->dcca->fw_version = 0;
		else strncpy (l->dcca->fw_version, kv->value, sizeof(l->dcca->fw_version));
		l->dcca->fw_version[sizeof(l->dcca->fw_version) - 1] = 0;		// if string from ini is too long
	}
}

static struct key_value *db_wrFWversion (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;

	if (!l->dcca || ! *l->dcca->fw_version) return NULL;
	return kv_add(kv, key, l->dcca->fw_version);
}

static void db_rdVID (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!kv->value) l->vid = 0;
	else l->vid = strtoul (kv->value, NULL, 0);
}

static struct key_value *db_wrVID (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	char tmp[32];

	if (!l->vid) return NULL;
	sprintf (tmp, "0x%lx", l->vid);
	return kv_add(kv, key, tmp);
}

static void db_rdUID (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!kv->value) l->uid = 0;
	else l->uid = strtoul (kv->value, NULL, 0);
}

static struct key_value *db_wrUID (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	char tmp[32];

	if (!l->uid) return NULL;
	sprintf (tmp, "0x%lx", l->uid);
	return kv_add(kv, key, tmp);
}

static void db_rdIcon (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;
	int icon;

	if (!kv->value) return;
	icon = atoi(kv->value);
	if (icon < 0 || icon > MAX_ICON_INDEX) return;
//	printf ("%s(): LOCO %d: %s(%d) = %d\n", __func__, l->adr, kv->key, kv->idx, icon);
	_db_locoFuncIcon(l, kv->idx, icon);
}

static struct key_value *db_wrIcon (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	funcT *f;
	char buf[16];

	f = l->funcs;
	while (f && kv) {
		if (f->icon) {
			sprintf (buf, "%d", f->icon);
			kv = kv_addIndexed(kv, key, f->fnum, buf);
		}
		f = f->next;
	}
	return kv;
}

static void db_rdImage (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;
	int icon;

	if (!kv->value) return;
	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));
	if (l->dcca) {
		icon = atoi(kv->value);
		if (icon < 0) return;
		if (kv->idx == 0) {
			l->dcca->decoderimage = icon;
//			printf ("%s(): LOCO %d: Image = %d\n", __func__, l->adr, icon);
		} else if (kv->idx == 1) {
			l->dcca->decodericon = icon;
//			printf ("%s(): LOCO %d: Icon = %d\n", __func__, l->adr, icon);
		}
	}
}

static struct key_value *db_wrImage (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	char buf[16];

	if (l->dcca && l->dcca->decoderimage > 0) {
		sprintf (buf, "%d", l->dcca->decoderimage);
		kv = kv_addIndexed(kv, key, 0, buf);
	}
	if (l->dcca && l->dcca->decodericon > 0) {
		sprintf (buf, "%d", l->dcca->decodericon);
		kv = kv_addIndexed(kv, key, 1, buf);
	}
	return kv;
}

static void db_rdAdrReq (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;

	if (!kv->value) return;
	if (!l->dcca) l->dcca = calloc (1, sizeof(*l->dcca));
	if (l->dcca) l->dcca->adr_req = atoi(kv->value);
}

static struct key_value *db_wrAdrReq (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	char buf[16];

	if (l->dcca && l->dcca->adr_req > 0) {
		sprintf (buf, "%d", l->dcca->adr_req);
		kv = kv_add(kv, key, buf);
	}
	return kv;
}

static void db_rdFlags (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;
	char *tok, *s;

	l->flags = 0;
	if ((s = kv->value) == NULL || !*s) return;
	do {
		while (*s && isspace((int) *s)) s++;
		tok = s;
		while (*s && !isspace((int) *s)) s++;
		if (*s) *s++ = 0;
		if (!strcmp (tok, "DCCA")) l->flags |= DEC_DCCA;
//		else if (!strcmp (tok, "BLUB")) l->flags |= DEC_BLUB;		// ... whatever flags we define ;-)
	} while (*s);
}

static struct key_value *db_wrFlags (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	char *tmp, *s;

	if (l->flags == 0) return kv;
	tmp = tmp256();

	if (l->flags & DEC_DCCA) strcat (tmp, "DCCA ");
//	if (l->flags & DEC_BLUB) strcat (tmp, "BLUB ");			// ... whatever flags we define ;-)
	s = tmp +strlen(tmp);
	while (s > tmp && isspace ((int) s[-1])) s--;				// kill trailling white space
	*s = 0;
	return kv_add(kv, key, tmp);
}

static void db_rdFtime (void *p, struct key_value *kv)
{
	locoT *l = (locoT *) p;
	int tim;

	if (!kv->value) return;
	tim = atoi(kv->value);
	if (tim < -1 || tim > 1000) return;
//	if (tim <= 0) {
//		printf ("%s(): LOCO %d: %s(%d) = %s\n", __func__, l->adr, kv->key, kv->idx, (tim < 0) ? "momentary" : "toggle");
//	} else {
//		printf ("%s(): LOCO %d: %s(%d) = %d.%ds\n", __func__, l->adr, kv->key, kv->idx, tim / 10, tim % 10);
//	}
	_db_locoFuncTiming(l, kv->idx, tim);
}

static struct key_value *db_wrFtime (void *p, struct key_value *kv, const char *key)
{
	locoT *l = (locoT *) p;
	funcT *f;
	char buf[16];

	f = l->funcs;
	while (f && kv) {
		if (f->timing) {
			sprintf (buf, "%d", f->timing / 100);
			kv = kv_addIndexed(kv, key, f->fnum, buf);
		}
		f = f->next;
	}
	return kv;
}

static void db_rdTrntFmt (void *p, struct key_value *kv)
{
	turnoutT *t = (turnoutT *) p;

	t->fmt = db_string2fmt(kv->value);
}

static struct key_value *db_wrTrntFmt (void *p, struct key_value *kv, const char *key)
{
	turnoutT *t = (turnoutT *) p;

	return kv_add(kv, key, db_fmt2string(t->fmt));
}

static void db_rdTrntUID (void *p, struct key_value *kv)
{
	char *s;
	int i;

	turnoutT *t = (turnoutT *) p;

	if ((s = kv->value) == NULL) return;

	for (i = 0; i < BIDIB_UID_LEN; i++, s += 2) {
		t->uid[i] = hex_byte(s);
	}
}

static struct key_value *db_wrTrntUID (void *p, struct key_value *kv, const char *key)
{
	char buf[32], *s;
	turnoutT *t = (turnoutT *) p;
	int i;

	if (t->fmt != TFMT_BIDIB) return kv;

	s = buf;
	for (i = 0; i < BIDIB_UID_LEN; i++) {
		s += sprintf (s, "%02X", t->uid[i]);
	}
	return kv_add(kv, key, buf);
}

static void db_rdTrntAspect (void *p, struct key_value *kv)
{
	turnoutT *t = (turnoutT *) p;

	t->aspect = atoi(kv->value);
}

static struct key_value *db_wrTrntAspect (void *p, struct key_value *kv, const char *key)
{
	char buf[16];
	turnoutT *t = (turnoutT *) p;

	if (t->fmt != TFMT_BIDIB) return kv;

	sprintf (buf, "%d",t->aspect);
	return kv_add(kv, key, buf);
}

static void db_rdXaccFmt (void *p, struct key_value *kv)
{
	extaccT *x = (extaccT *) p;

	x->fmt = db_string2fmt(kv->value);
}

static struct key_value *db_wrXaccFmt (void *p, struct key_value *kv, const char *key)
{
	extaccT *x = (extaccT *) p;

	return kv_add(kv, key, db_fmt2string(x->fmt));
}

static locoT *db_interpretLoco (struct ini_section *ini)
{
	struct key_value *kv;
	const struct keyhandler *kh;
	locoT *l;
	char *num, *end;
	int adr;

	if (!ini || (*ini->name != 'L' && *ini->name != 'l')) return NULL;

	num = &ini->name[1];
	adr = strtol(num, &end, 10);
	if (end == num || adr < 0 || adr > MAX_LOCO_ADR) return NULL;			// the value behind the 'L' is not numeric or out of range

//	printf ("[L%d]\n", adr);
	if (adr == 0) {
		l = &defLoco;
	} else {
		if ((l = calloc(1, sizeof(*l))) == NULL) return NULL;
	}
	l->adr = adr;

	kv = ini->kv;
	while (kv) {
		kh = loco_entries;
		while (kh->key && strcasecmp(kh->key, kv->key)) kh++;
		if (kh->key) {
//			printf ("\t'%s' = '%s'\n", kv->key, (kv->value) ? kv->value : "(NULL)");
			if (kh->reader) kh->reader (l, kv);
		}
		kv = kv->next;
	}

	return l;
}

static turnoutT *db_interpretTurnout (struct ini_section *ini)
{
	struct key_value *kv;
	const struct keyhandler *kh;
	turnoutT *t;
	char *num, *end;
	int adr;

	if (!ini || (*ini->name != 'T' && *ini->name != 't')) return NULL;

	num = &ini->name[1];
	adr = strtol(num, &end, 10);
	if (end == num || adr < 0 || adr > MAX_TURNOUT) return NULL;			// the value behind the 'T' is not numeric or out of range

//	printf ("[T%d]\n", adr);
	if (adr == 0) {
		t = &defTurnout;
	} else {
		if ((t = calloc(1, sizeof(*t))) == NULL) return NULL;
		memcpy (t, &defTurnout, sizeof(*t));
	}
	t->adr = adr;

	kv = ini->kv;
	while (kv) {
		kh = turnout_entries;
		while (kh->key && strcasecmp(kh->key, kv->key)) kh++;
		if (kh->key) {
//			printf ("\t'%s' = '%s'\n", kv->key, (kv->value) ? kv->value : "(NULL)");
			if (kh->reader) kh->reader (t, kv);
		}
		kv = kv->next;
	}

	return t;
}

static extaccT *db_interpretExtendedAccessory (struct ini_section *ini)
{
	struct key_value *kv;
	const struct keyhandler *kh;
	extaccT *x;
	char *num, *end;
	int adr;

	if (!ini || (*ini->name != 'X' && *ini->name != 'x')) return NULL;

	num = &ini->name[1];
	adr = strtol(num, &end, 10);
	if (end == num || adr <= 0 || adr > MAX_DCC_EXTACC) return NULL;			// the value behind the 'X' is not numeric or out of range

//	printf ("[X%d]\n", adr);
	if ((x = calloc(1, sizeof(*x))) == NULL) return NULL;
	x->adr = adr;
	x->fmt = TFMT_DCC;

	kv = ini->kv;
	while (kv) {
		kh = extacc_entries;
		while (kh->key && strcasecmp(kh->key, kv->key)) kh++;
		if (kh->key) {
//			printf ("\t'%s' = '%s'\n", kv->key, (kv->value) ? kv->value : "(NULL)");
			if (kh->reader) kh->reader (x, kv);
		}
		kv = kv->next;
	}

	return x;
}

static void db_readConsists (struct ini_section *ini)
{
	struct key_value *kv;
	char *s;
	int adr1, adr;

	kv = ini->kv;
	while (kv) {
		s = kv->value;
		adr1 = strtol(s, &s, 10);
		while (s && *s) {
			while (*s && !isdigit(*s) && *s != '-') s++;
			adr = strtol (s, &s, 10);
			if (adr1 && adr) _consist_couple(adr1, adr);
		}
		kv = kv->next;
	}
}

static void db_interpretIni (struct ini_section *ini)
{
	struct ini_section *consist;
	locoT *l;
	turnoutT *t;
	extaccT *x;

	consist = NULL;
//	loco_lock(__func__);
	while (ini) {
		switch (*ini->name) {
			case 'l':
			case 'L':
				l = db_interpretLoco(ini);
				if (l) l = db_locoSanitize(l);
				if (l && l->adr > 0) db_addLoco(l);
				break;
			case 't':
			case 'T':
				t = db_interpretTurnout(ini);
				if (t) t = db_turnoutSanitize(t);
				if (t && t->adr > 0) db_addTurnout(t);
				break;
#if 0		// @todo will be implemented later
			case 'a':
			case 'A':
				db_interpretAccessory(ini);
				break;
#endif
			case 'x':
			case 'X':
				x = db_interpretExtendedAccessory(ini);
				if (x) x = db_extaccSanitize(x);
				if (x && x->adr > 0) db_addExtacc(x);
				break;
			default:
				if (!strcasecmp("Consists", ini->name)) {
					consist = ini;		// will be handled later (after all locos are read and lock is released)
				}
				break;
		}
		ini = ini->next;
	}

//	loco_unlock();
	if (consist) db_readConsists (consist);
}

static struct ini_section *db_writeLoco (struct ini_section *ini, locoT *l)
{
	struct key_value *kv, *tmp;
	const struct keyhandler *kh;
	char buf[32];

	sprintf (buf, "L%d", l->adr);
	if ((ini = ini_add(ini, buf)) == NULL) return NULL;

	kh = loco_entries;
	kv = NULL;
	while (kh->key) {
		if (kh->writer) {
			tmp = kh->writer(l, kv, kh->key);
			if (tmp != NULL) kv = tmp;
			if (!ini->kv) ini->kv = tmp;
		}
		kh++;
	}

	return ini;
}

static struct ini_section *db_writeConsists (struct ini_section *ini, struct consist *c)
{
	struct key_value *kv, *tmp;
	char *val, *s;
	int i;

	if ((ini = ini_add(ini, "Consists")) == NULL) return NULL;

	kv = NULL;
	while (c) {
		s = val = tmp256();
		*s = 0;
		for (i = 0; i < MAX_CONSISTLENGTH; i++) {
			if (c->adr[i]) s += sprintf (s, "%s%d", (s == val) ? "" : ", ", c->adr[i]);
		}
		tmp = kv_add(kv, "C", val);		// all entries can use the same "key" - it is irrelevant
		if (tmp != NULL) kv = tmp;
		if (!ini->kv) ini->kv = tmp;
		c = c->next;
	}

	return ini;
}

static struct ini_section *db_writeTurnout (struct ini_section *ini, turnoutT *t)
{
	struct key_value *kv, *tmp;
	const struct keyhandler *kh;
	char buf[32];

	sprintf (buf, "T%d", t->adr);
	if ((ini = ini_add(ini, buf)) == NULL) return NULL;

	kh = turnout_entries;
	kv = NULL;
	while (kh->key) {
		if (kh->writer) {
			tmp = kh->writer(t, kv, kh->key);
			if (tmp != NULL) kv = tmp;
			if (!ini->kv) ini->kv = tmp;
		}
		kh++;
	}

	return ini;
}

static struct ini_section *db_writeExtacc (struct ini_section *ini, extaccT *x)
{
	struct key_value *kv, *tmp;
	const struct keyhandler *kh;
	char buf[32];

	sprintf (buf, "X%d", x->adr);
	if ((ini = ini_add(ini, buf)) == NULL) return NULL;

	kh = extacc_entries;
	kv = NULL;
	while (kh->key) {
		if (kh->writer) {
			tmp = kh->writer(x, kv, kh->key);
			if (tmp != NULL) kv = tmp;
			if (!ini->kv) ini->kv = tmp;
		}
		kh++;
	}

	return ini;
}

static struct ini_section *db_generateIni (void)
{
	struct ini_section *ini, *root;
	locoT *l;
	turnoutT *t;
	extaccT *x;

	ini = root = db_writeLoco(NULL, &defLoco);
	l = locodb;
	while (l) {
		ini = db_writeLoco(ini, l);
		l = l->next;
	}

	ini = db_writeConsists(ini, consist_getConsists());

	ini = db_writeTurnout(ini, &defTurnout);
	t = turnouts;
	while (t) {
		ini = db_writeTurnout(ini, t);
		t = t->next;
	}

	x = xaccessories;
	while (x) {
		ini = db_writeExtacc(ini, x);
		x = x->next;
	}

	return root;
}

static void db_store (TimerHandle_t t)
{
	struct ini_section *ini;

	log_msg (LOG_INFO, "%s() Storing loco DB\n", __func__);
	loco_lock(__func__);
	xTimerStop(t, 100);
	ini = db_generateIni();
	loco_unlock();

	ini_writeFile(CONFIG_LOCO, ini);
	ini_free(ini);
	event_fire(EVENT_LOCO_DB, 0, NULL);
	log_msg (LOG_INFO, "%s() Storage finished\n", __func__);
}

void db_iterateLoco (bool (*func)(locoT *, void *), void *priv)
{
	locoT *l;

	if (!func) return;

	loco_lock(__func__);
	for (l = locodb; l; l = l->next) {
		if (!func(l, priv)) break;
	}
	loco_unlock();
}

int db_init (void)
{
	struct ini_section *ini;

	db_freeDB();

	if ((ini = ini_readFile(CONFIG_LOCO)) != NULL) {
		db_interpretIni(ini);
		ini_free(ini);
	}

	if (!storage_timer) {
		storage_timer = xTimerCreate("FMT-Storage", STORAGE_TIMEOUT, 0, NULL, db_store);
	}

	return 0;
}
