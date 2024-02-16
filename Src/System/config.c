/*
 * config.c
 *
 *  Created on: 24.04.2020
 *      Author: andi
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
#include "rb2.h"
#include "timers.h"
#include "config.h"
#include "decoder.h"
#include "defaults.h"

#define STORAGE_TIMEOUT		pdMS_TO_TICKS(3 * 1000)

struct keyhandler {
	const char					*key;
	const int					 param1;
	void (*reader)(int param1, struct key_value *kv);
	struct key_value * (*writer)(struct key_value *kv, const char *key, int param1);
};

struct section_map {
	const char					*section;
	const struct keyhandler		*handlers;
};

static struct sysconf syscfg;
static struct fmtconfig fmtcfg;			// TODO: maybe we should put this in signal.c (track signal generation)
static TimerHandle_t storage_timer;		///< a delay after the last change before the filesystem is updated

/*
 * Forward declarations of function prototypes for reading
 */
static void cnf_ipConfig (int param1, struct key_value *kv);
static void cnf_ipport (int param1, struct key_value *kv);
static void cnf_ipv4Adr (int param1, struct key_value *kv);
static void cnf_rdSystem (int param1, struct key_value *kv);
static void cnf_rdBiDiB (int param1, struct key_value *kv);
static void cnf_rdTrack (int param1, struct key_value *kv);
static void cnf_rdDCC (int param1, struct key_value *kv);
static void cnf_rdMM (int param1, struct key_value *kv);
static void cnf_rdM3 (int param1, struct key_value *kv);
static void cnf_rdTrnt (int param1, struct key_value *kv);

/*
 * Forward declarations of function prototypes for writing
 */
static struct key_value *cnf_ipConfigWrite (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_ipportWrite (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_ipv4AdrWrite (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_wrSystem (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_wrBiDiB (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_wrTrack (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_wrDCC (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_wrMM (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_wrM3 (struct key_value *kv, const char *key, int param1);
static struct key_value *cnf_wrTrnt (struct key_value *kv, const char *key, int param1);

/* === the handler functions for all settings ================================================ */
static const struct keyhandler network[] = {
	{ "config",			0, cnf_ipConfig, cnf_ipConfigWrite },
	{ "address",		0, cnf_ipv4Adr, cnf_ipv4AdrWrite },
	{ "netmask",		1, cnf_ipv4Adr, cnf_ipv4AdrWrite },
	{ "gateway",		2, cnf_ipv4Adr, cnf_ipv4AdrWrite },
	{ "p50port",		0, cnf_ipport, cnf_ipportWrite },
	{ NULL,				0, NULL, NULL }
};

static const struct keyhandler booster[] = {
	{ "voltage",		0, cnf_rdTrack, cnf_wrTrack },		// in 0,1V
	{ "prgvoltage",		1, cnf_rdTrack, cnf_wrTrack },		// in 0,1V
	{ "current",		2, cnf_rdTrack, cnf_wrTrack },		// in 0,1A
	{ "short",			3, cnf_rdTrack, cnf_wrTrack },		// short time in ms
	{ "inrush",			4, cnf_rdTrack, cnf_wrTrack },		// inrush time in ms
	{ "mmshort",		5, cnf_rdTrack, cnf_wrTrack },		// short time for MM booster in ms
	{ "dccshort",		6, cnf_rdTrack, cnf_wrTrack },		// short time for DCC booster in ms
	{ NULL,				0, NULL, NULL }
};

static const struct keyhandler sysconfig[] = {
	{ "locopurge",			0, cnf_rdSystem, cnf_wrSystem },
	{ "s88Modules",			1, cnf_rdSystem, cnf_wrSystem },
	{ "s88Frequency",		2, cnf_rdSystem, cnf_wrSystem },
	{ "lighteffects",		3, cnf_rdSystem, cnf_wrSystem },
	{ "bidibacclogic",		4, cnf_rdSystem, cnf_wrSystem },
	{ "canModules",			5, cnf_rdSystem, cnf_wrSystem },
	{ "lnetModules",		6, cnf_rdSystem, cnf_wrSystem },
	{ "StartState",			7, cnf_rdSystem, cnf_wrSystem },
	{ "BiDiGlobalShort",	8, cnf_rdSystem, cnf_wrSystem },
	{ "BiDiRemoteOnOff",	9, cnf_rdSystem, cnf_wrSystem },

//#define SYSFLAG_LONGPAUSE			0001	// MM long pause
//#define SYSFLAG_DEFAULTDCC		0010	// locos are DCC by default
//#define SYSFLAG_NOMAGONMAINBST	0100	// no magnet command on internal booster
//#define SYSFLAG_NOMAGONCDEBST		0200	// no magnet command on CDE booster output
//#define SYSFLAG_NOMAGONMKLNBST	0400	// no magnet command on MM booster output
	{ NULL,				0, NULL, NULL }
};

static const struct keyhandler bidib[] = {
	{ "port",			0, cnf_rdBiDiB, cnf_wrBiDiB },
	{ "user",			1, cnf_rdBiDiB, cnf_wrBiDiB },
	{ NULL,				0, NULL, NULL }
};

static const struct keyhandler dcc[] = {
	{ "repeat",			0, cnf_rdDCC, cnf_wrDCC },			// repeat of packets
	{ "pomrepeat",		1, cnf_rdDCC, cnf_wrDCC },			// repeat of POM packets
	{ "preamble",		2, cnf_rdDCC, cnf_wrDCC },			// preamble length in bits
	{ "bittime_one",	3, cnf_rdDCC, cnf_wrDCC },			// length of a 1-bit in µs
	{ "bittime_zero",	4, cnf_rdDCC, cnf_wrDCC },			// length of a 0-bit in µs
	{ "railcom",		5, cnf_rdDCC, cnf_wrDCC },			// boolean for railcom generation
	{ "dcca",			6, cnf_rdDCC, cnf_wrDCC },			// boolean for dcca generation
	{ "acc_nop",		7, cnf_rdDCC, cnf_wrDCC },			// boolean for acc NOP generation
	{ "dcc_long",		8, cnf_rdDCC, cnf_wrDCC },			// boolean for DCC loco to always use long addresses
	{ NULL,				0, NULL, NULL }
};

static const struct keyhandler mm[] = {
	{ "repeat",			0, cnf_rdMM, cnf_wrMM },			// repeat of packets
	{ "pause",			1, cnf_rdMM, cnf_wrMM },			// pause between packets in µs
	{ NULL,				0, NULL, NULL }
};

static const struct keyhandler m3[] = {
	{ "repeat",			0, cnf_rdM3, cnf_wrM3 },			// repeat of packets
	{ "enable",			1, cnf_rdM3, cnf_wrM3 },			// optionally disable m3 output
	{ NULL,				0, NULL, NULL }
};

static const struct keyhandler trnt[] = {
	{ "mintime",		0, cnf_rdTrnt, cnf_wrTrnt },		// minimum switching time
	{ "maxtime",		1, cnf_rdTrnt, cnf_wrTrnt },		// maximum switching time
	{ "outputmain",		2, cnf_rdTrnt, cnf_wrTrnt },		// output commands to main booster
	{ "outputcde",		3, cnf_rdTrnt, cnf_wrTrnt },		// output commands to CDE booster
	{ "outputmkln",		4, cnf_rdTrnt, cnf_wrTrnt },		// output commands to Märklin booster
	{ "repeat",			5, cnf_rdTrnt, cnf_wrTrnt },		// number of repeats for accessory commands (in either format)
	{ NULL,				0, NULL, NULL }
};

/* === the sections ========================================================================== */
static const struct section_map sections[] = {
	{ "network",		network },
	{ "booster",		booster },
	{ "system",			sysconfig },
	{ "bidib",			bidib },
	{ "protocol-dcc",	dcc },
	{ "protocol-mm",	mm },
	{ "protocol-m3",	m3 },
	{ "turnouts",		trnt },
	{ NULL,				NULL }
};

// ==============================================================================================
// === Helper functions =========================================================================
// ==============================================================================================

static int cnf_decimal (char *val, int decimals)
{
	int v = 0;
	bool sign, dec, neg;

	if (!val || !*val) return 0;

	sign = dec = neg = false;
	while (*val && isspace(*val)) val++;
	while (*val && (isdigit(*val) || (!dec && (*val == '.' || *val == ',')) || (!sign && (*val == '-' || *val == '+')))) {
		switch (*val) {
			case '-':		// remember that the value will be negative
				neg = true;
				/* FALL THRU */
			case '+':		// a plus is more or less ignored
				sign = true;		// only one sign will be allowed
				break;
			case '.':		// american decimal separator
			case ',':		// german decimal separator
				dec = true;
				break;
			default:		// only digits can match this case
				v *= 10;
				v += *val - '0';
				if (dec && decimals > 0) decimals--;
				break;
		}
		if (dec && decimals <= 0) break;
		val++;
	}

	while (decimals > 0) {
		v += 10;
		decimals--;
	}
	return v;
}

static int cnf_boundedInteger (char *val, int min, int max)
{
	int v;

	v = atoi(val);
	if (v < min) return min;
	if (v > max) return max;
	return v;
}

static bool cnf_boolean (char *val)
{
	if (*val == '1' || *val == 'y' || *val == 'Y') return true;
	return false;
}

// ==============================================================================================
// === Network configuration ====================================================================
// ==============================================================================================

static void cnf_ipConfig (int param1, struct key_value *kv)
{
	(void) param1;

	if (!strcasecmp ("DHCP", kv->value)) {
		syscfg.ipm = IPMETHOD_DHCP;
	} else if (!strcasecmp ("MANUAL", kv->value)) {
		syscfg.ipm = IPMETHOD_MANUAL;
	} else {
		fprintf (stderr, "%s: illegal value '%s'\n", kv->key, kv->value);
	}
}

static struct key_value *cnf_ipConfigWrite (struct key_value *kv, const char *key, int param1)
{
	(void) param1;

	switch (syscfg.ipm) {
		case IPMETHOD_DHCP:
			return kv_add (kv, key, "DHCP");
			break;
		case IPMETHOD_MANUAL:
			return kv_add (kv, key, "MANUAL");
			break;
		default:
			return kv_add (kv, key, "DHCP");
			break;
	}
}

static void cnf_ipport (int param1, struct key_value *kv)
{
	int port;

	port = atoi(kv->value);
	if (port <= 0 || port > UINT16_MAX) {
		fprintf (stderr, "%s(%s): invalid port value %d\n", __func__, kv->key, port);
	}

	switch (param1) {
		case 0:		// P50(X[ab]) TCP port specified
			syscfg.p50_port = port;
			break;
	}
}

static struct key_value *cnf_ipportWrite (struct key_value *kv, const char *key, int param1)
{
	char tmp[16];

	*tmp = 0;

	switch (param1) {
		case 0:		// P50(X[ab]) TCP port specified
			sprintf (tmp, "%d", syscfg.p50_port);
			break;
	}
	return kv_add (kv, key, tmp);
}

static void cnf_ipv4Adr (int param1, struct key_value *kv)
{
	uint32_t ipv4;

	ipv4 = ipaddr_addr(kv->value);
	(void) ipv4;

	switch (param1) {
		case 0:		// IPv4 Address specified
			syscfg.ip_addr.addr = ipv4;
			break;
		case 1:		// IPv4 Netmask specified
			syscfg.ip_mask.addr = ipv4;
			break;
		case 2:		// IPv4 default gateway specified
			syscfg.ip_gw.addr = ipv4;
			break;
	}
}

static struct key_value *cnf_ipv4AdrWrite (struct key_value *kv, const char *key, int param1)
{
	char tmp[32];

	*tmp = 0;

	switch (param1) {
		case 0:		// IPv4 Address specified
			sprintf (tmp, "%s", ip_ntoa(&syscfg.ip_addr));
			break;
		case 1:		// IPv4 Netmask specified
			sprintf (tmp, "%s", ip_ntoa(&syscfg.ip_mask));
			break;
		case 2:		// IPv4 default gateway specified
			sprintf (tmp, "%s", ip_ntoa(&syscfg.ip_gw));
			break;
	}
	return kv_add (kv, key, tmp);
}

// ==============================================================================================
// === Generic system stuff =====================================================================
// ==============================================================================================

static void cnf_rdSystem (int param1, struct key_value *kv)
{
	switch (param1) {
		case 0:		// loco purge timeout in minutes
			syscfg.locopurge = cnf_boundedInteger(kv->value, 0, CNF_DEF_MAX_PURGE);
			break;
		case 1:		// No of s88 modules
			syscfg.s88Modules = cnf_boundedInteger(kv->value, 0, CNF_DEF_MAX_S88MODULES);
			break;
		case 2:		// Frequency of s88 bus
			syscfg.s88Frequency = cnf_boundedInteger(kv->value, CNF_DEF_MIN_S88FREQUENCY, CNF_DEF_MAX_S88FREQUENCY);
			break;
		case 3:		// light effects on/off
			if (cnf_boundedInteger(kv->value, 0, 2) & 1) syscfg.sysflags |= SYSFLAG_LIGHTEFFECTS;
			else syscfg.sysflags &= ~SYSFLAG_LIGHTEFFECTS;
			if (cnf_boundedInteger(kv->value, 0, 2) & 2) syscfg.sysflags |= SYSFLAG_LIGHTSOFF;
			else syscfg.sysflags &= ~SYSFLAG_LIGHTSOFF;
			break;
		case 4:		// BiDiB ACC address as logical on/off
			if (cnf_boolean(kv->value)) syscfg.sysflags |= SYSFLAG_ACC_LOGICAL;
			else syscfg.sysflags &= ~SYSFLAG_ACC_LOGICAL;
			break;
		case 5:		// No of can modules
			syscfg.canModules = cnf_boundedInteger(kv->value, 0, CNF_DEF_MAX_CANMODULES);
			break;
		case 6:		// No of loconet modules
			syscfg.lnetModules = cnf_boundedInteger(kv->value, 0, CNF_DEF_MAX_LNETMODULES);
			break;
		case 7:		// Start state
			if (cnf_boolean(kv->value)) syscfg.sysflags |= SYSFLAG_STARTSTATE;
			else syscfg.sysflags &= ~SYSFLAG_STARTSTATE;
			break;
		case 8:		// BiDi global short
			if (cnf_boolean(kv->value)) syscfg.sysflags |= SYSFLAG_GLOBAL_BIDIB_SHORT;
			else syscfg.sysflags &= ~SYSFLAG_GLOBAL_BIDIB_SHORT;
			break;
		case 9:		// BiDi remote On/Off control
			if (cnf_boolean(kv->value)) syscfg.sysflags |= SYSFLAG_BIDIB_ONOFF;
			else syscfg.sysflags &= ~SYSFLAG_BIDIB_ONOFF;
			break;
	}
}

static struct key_value *cnf_wrSystem (struct key_value *kv, const char *key, int param1)
{
	char tmp[64];

	if (!key || !*key) return NULL;

	switch (param1) {
		case 0:		// loco purge timeout in minutes
			sprintf (tmp, "%d", syscfg.locopurge);
			break;
		case 1:		// No of s88 modules
			sprintf (tmp, "%d", syscfg.s88Modules);
			break;
		case 2:		// Frequency of s88 bus
			sprintf (tmp, "%d", syscfg.s88Frequency);
			break;
		case 3:		// light effects
			sprintf (tmp, "%d", (syscfg.sysflags & SYSFLAG_LIGHTEFFECTS) ? 1 : (syscfg.sysflags & SYSFLAG_LIGHTSOFF) ? 2 : 0);
			break;
		case 4:		// BiDiB ACC address as logical
			sprintf (tmp, "%s", (syscfg.sysflags & SYSFLAG_ACC_LOGICAL) ? "yes" : "no");
			break;
		case 5:		// No of can modules
			sprintf (tmp, "%d", syscfg.canModules);
			break;
		case 6:		// No of loconet modules
			sprintf (tmp, "%d", syscfg.lnetModules);
			break;
		case 7:		// Start state
			sprintf (tmp, "%s", (syscfg.sysflags & SYSFLAG_STARTSTATE) ? "yes" : "no");
			break;
		case 8:		// BiDi global short
			sprintf (tmp, "%s", (syscfg.sysflags & SYSFLAG_GLOBAL_BIDIB_SHORT) ? "yes" : "no");
			break;
		case 9:		// BiDi remote On/Off control
			sprintf (tmp, "%s", (syscfg.sysflags & SYSFLAG_BIDIB_ONOFF) ? "yes" : "no");
			break;
		default:
			return NULL;
	}
	return kv_add (kv, key, tmp);
}

// ==============================================================================================
// === BiDiB configuration ======================================================================
// ==============================================================================================

static void cnf_rdBiDiB (int param1, struct key_value *kv)
{
	int port;

	switch (param1) {
		case 0:		// netBiDiB TCP port specified
			if (kv->value) {
				port = atoi(kv->value);
				if (port <= 0 || port > UINT16_MAX) {
					log_error("%s(%s): invalid port value %d\n", __func__, kv->key, port);
				}
				syscfg.bidib.port = port;
			}
			break;
		case 1:
			if (kv->value) {
				strncpy(syscfg.bidib.user, kv->value, sizeof(syscfg.bidib.user));
				syscfg.bidib.user[sizeof(syscfg.bidib.user) - 1] = 0;
			}
			break;
	}
}

static struct key_value *cnf_wrBiDiB (struct key_value *kv, const char *key, int param1)
{
	char tmp[32];

	*tmp = 0;

	switch (param1) {
		case 0:		// netBiDiB TCP port specified
			sprintf (tmp, "%d", syscfg.bidib.port);
			break;
		case 1:		// netBiDiB user name
			sprintf (tmp, "%s", syscfg.bidib.user);
			break;
	}
	return kv_add (kv, key, tmp);
}

// ==============================================================================================
// === Track configuration ======================================================================
// ==============================================================================================

static void cnf_rdTrack (int param1, struct key_value *kv)
{
	int val;

	switch (param1) {
		case 0:		// main track voltage in 0,1V
			val = cnf_decimal(kv->value, 1);
			ts_setVoltage(val);
			break;
		case 1:		// programming track voltage in 0,1V
			val = cnf_decimal(kv->value, 1);
			ts_setPtVoltage(val);
			break;
		case 2:		// max. track current in 0,1A
			val = cnf_decimal(kv->value, 1);
			ts_setCurrent(val);
			break;
		case 3:		// short sensitivity in ms
			val = cnf_decimal(kv->value, 0);
			ts_setSensitivity(val);
			break;
		case 4:		// inrush time in ms
			val = cnf_decimal(kv->value, 0);
			ts_setInrush(val);
			break;
		case 5:		// short sensitivity for MM booster in ms
			syscfg.mmshort = cnf_decimal(kv->value, 0);
			if (syscfg.mmshort < EXTERNSHORT_MIN) syscfg.mmshort = EXTERNSHORT_MIN;
			if (syscfg.mmshort > EXTERNSHORT_MAX) syscfg.mmshort = EXTERNSHORT_MAX;
			break;
		case 6:		// short sensitivity for DCC booster in ms
			syscfg.dccshort = cnf_decimal(kv->value, 0);
			if (syscfg.dccshort < EXTERNSHORT_MIN) syscfg.dccshort = EXTERNSHORT_MIN;
			if (syscfg.dccshort > EXTERNSHORT_MAX) syscfg.dccshort = EXTERNSHORT_MAX;
			break;
	}
}

static struct key_value *cnf_wrTrack (struct key_value *kv, const char *key, int param1)
{
	char tmp[64];
	int val;

	if (!key || !*key) return NULL;

	switch (param1) {
		case 0:		// main track voltage
			val = ts_getVoltage();
			sprintf (tmp, "%d.%d", val / 10, val % 10);
			break;
		case 1:		// programming track voltage
			val = ts_getPtVoltage();
			sprintf (tmp, "%d.%d", val / 10, val % 10);
			break;
		case 2:		// current
			val = ts_getCurrent();
			sprintf (tmp, "%d.%d", val / 10, val % 10);
			break;
		case 3:		// current sensitivity
			val = ts_getSensitivity();
			sprintf (tmp, "%d", val);
			break;
		case 4:		// inrush timeout
			val = ts_getInrush();
			sprintf (tmp, "%d", val);
			break;
		case 5:		// short sensitivity for MM booster in ms
			sprintf (tmp, "%d", syscfg.mmshort);
			break;
		case 6:		// short sensitivity for DCC booster in ms
			sprintf (tmp, "%d", syscfg.dccshort);
			break;
		default:
			return NULL;
	}
	return kv_add (kv, key, tmp);
}

// ==============================================================================================
// === DCC signal configuration =================================================================
// ==============================================================================================

static void cnf_rdDCC (int param1, struct key_value *kv)
{
	switch (param1) {
		case 0:		// block repetitions
			fmtcfg.dcc.repeat = cnf_boundedInteger(kv->value, 1, CNF_DEF_MAXdccrepeat);
			break;
		case 1:		// POM block repetitions
			fmtcfg.dcc.pomrepeat = cnf_boundedInteger(kv->value, 1, CNF_DEF_MAXdccpomrepeat);
			break;
		case 2:		// preamble length in bits
			fmtcfg.dcc.preamble = cnf_boundedInteger(kv->value, 9, CNF_DEF_MAXdccpreamble);
			break;
		case 3:		// length of a 1-bit in µs
			fmtcfg.dcc.tim_one = cnf_boundedInteger(kv->value, CNF_DEF_MINdcctim_one, CNF_DEF_MAXdcctim_one);
			break;
		case 4:		// length of a 0-bit in µs
			fmtcfg.dcc.tim_zero = cnf_boundedInteger(kv->value, CNF_DEF_MINdcctim_zero, CNF_DEF_MAXdcctim_zero);
			break;
		case 5:		// boolean for railcom generation
			if (cnf_boolean(kv->value)) fmtcfg.sigflags |= SIGFLAG_RAILCOM;
			else fmtcfg.sigflags &= ~SIGFLAG_RAILCOM;
			break;
		case 6:		// boolean for dcca generation
			if (cnf_boolean(kv->value)) fmtcfg.sigflags |= SIGFLAG_DCCA;
			else fmtcfg.sigflags &= ~SIGFLAG_DCCA;
			break;
		case 7:		// NOP for accessory decoders
			if (cnf_boolean(kv->value)) fmtcfg.sigflags |= SIGFLAG_DCCNOP;
			else fmtcfg.sigflags &= ~SIGFLAG_DCCNOP;
			break;
		case 8:		// always use long addresses for DCC locos
			if (cnf_boolean(kv->value)) fmtcfg.sigflags |= SIGFLAG_DCC_LONG_ADR;
			else fmtcfg.sigflags &= ~SIGFLAG_DCC_LONG_ADR;
			break;
	}
}

static struct key_value *cnf_wrDCC (struct key_value *kv, const char *key, int param1)
{
	char tmp[64];

	if (!key || !*key) return NULL;

	switch (param1) {
		case 0:		// block repetitions
			sprintf (tmp, "%d", fmtcfg.dcc.repeat);
			break;
		case 1:		// POM block repetitions
			sprintf (tmp, "%d", fmtcfg.dcc.pomrepeat);
			break;
		case 2:		// preamble length in bits
			sprintf (tmp, "%d", fmtcfg.dcc.preamble);
			break;
		case 3:		// length of a 1-bit in µs
			sprintf (tmp, "%d", fmtcfg.dcc.tim_one);
			break;
		case 4:		// length of a 0-bit in µs
			sprintf (tmp, "%d", fmtcfg.dcc.tim_zero);
			break;
		case 5:		// boolean for railcom generation
			sprintf (tmp, "%s", (fmtcfg.sigflags & SIGFLAG_RAILCOM) ? "yes" : "no");
			break;
		case 6:		// boolean for dcca generation
			sprintf (tmp, "%s", (fmtcfg.sigflags & SIGFLAG_DCCA) ? "yes" : "no");
			break;
		case 7:		// NOP for accessory decoders
			sprintf (tmp, "%s", (fmtcfg.sigflags & SIGFLAG_DCCNOP) ? "yes" : "no");
			break;
		case 8:		// always use long addresses for DCC locos
			sprintf (tmp, "%s", (fmtcfg.sigflags & SIGFLAG_DCC_LONG_ADR) ? "yes" : "no");
			break;
		default:
			return NULL;
	}
	return kv_add (kv, key, tmp);
}

// ==============================================================================================
// === MM signal configuration ==================================================================
// ==============================================================================================

static void cnf_rdMM (int param1, struct key_value *kv)
{
	switch (param1) {
		case 0:		// block repetitions
			fmtcfg.mm.repeat = cnf_boundedInteger(kv->value, 1, CNF_DEF_MAXmmrepeat);
			break;
		case 1:		// inter block pause
			fmtcfg.mm.pause = cnf_boundedInteger(kv->value, CNF_DEF_MINmmpause, CNF_DEF_MAXmmpause);
			break;
	}
}

static struct key_value *cnf_wrMM (struct key_value *kv, const char *key, int param1)
{
	char tmp[64];

	if (!key || !*key) return NULL;

	switch (param1) {
		case 0:		// block repetitions
			sprintf (tmp, "%d", fmtcfg.mm.repeat);
			break;
		case 1:		// inter block pause
			sprintf (tmp, "%lu", fmtcfg.mm.pause);
			break;
		default:
			return NULL;
	}
	return kv_add (kv, key, tmp);
}

// ==============================================================================================
// === M3 signal configuration ==================================================================
// ==============================================================================================

static void cnf_rdM3 (int param1, struct key_value *kv)
{
	switch (param1) {
		case 0:		// block repetitions
			fmtcfg.m3.repeat = cnf_boundedInteger(kv->value, 1, CNF_DEF_MAXm3repeat);
			break;
		case 1:		// switch m3 on/off
			if (cnf_boolean(kv->value)) fmtcfg.sigflags |= SIGFLAG_M3ENABLED;
			else fmtcfg.sigflags &= ~SIGFLAG_M3ENABLED;
			break;
	}
}

static struct key_value *cnf_wrM3 (struct key_value *kv, const char *key, int param1)
{
	char tmp[64];

	if (!key || !*key) return NULL;

	switch (param1) {
		case 0:		// block repetitions
			sprintf (tmp, "%d", fmtcfg.m3.repeat);
			break;
		case 1:		// boolean for m3 output enable
			sprintf (tmp, "%s", (fmtcfg.sigflags & SIGFLAG_M3ENABLED) ? "yes" : "no");
			break;
		default:
			return NULL;
	}
	return kv_add (kv, key, tmp);
}

// ==============================================================================================
// === Turnout configuration ====================================================================
// ==============================================================================================

static void cnf_rdTrnt (int param1, struct key_value *kv)
{
	switch (param1) {
		case 0:		// MIN-time
			trnt_setMinTime(atoi(kv->value));
			break;
		case 1:		// MAX-time
			trnt_setMaxTime(atoi(kv->value));
			break;
		case 2:		// boolean for main booster output
			if (cnf_boolean(kv->value)) syscfg.sysflags &= ~SYSFLAG_NOMAGONMAINBST;
			else syscfg.sysflags |= SYSFLAG_NOMAGONMAINBST;
			break;
		case 3:		// boolean for CDE booster output
			if (cnf_boolean(kv->value)) syscfg.sysflags &= ~SYSFLAG_NOMAGONCDEBST;
			else syscfg.sysflags |= SYSFLAG_NOMAGONCDEBST;
			break;
		case 4:		// boolean for Märklin booster output
			if (cnf_boolean(kv->value)) syscfg.sysflags &= ~SYSFLAG_NOMAGONMKLNBST;
			else syscfg.sysflags |= SYSFLAG_NOMAGONMKLNBST;
			break;
		case 5:		// accessory repeat for all formats
			cnf_getFMTconfig()->accrepeat = atoi(kv->value);
			break;
	}
}

static struct key_value *cnf_wrTrnt (struct key_value *kv, const char *key, int param1)
{
	char tmp[16];

	switch (param1) {
		case 0:		// MIN-time
			sprintf (tmp, "%d", trnt_getMinTime());
			break;
		case 1:		// MAX-time
			sprintf (tmp, "%d", trnt_getMaxTime());
			break;
		case 2:		// boolean for main booster output
			sprintf (tmp, "%s", (syscfg.sysflags & SYSFLAG_NOMAGONMAINBST) ? "no" : "yes");
			break;
		case 3:		// boolean for CDE booster output
			sprintf (tmp, "%s", (syscfg.sysflags & SYSFLAG_NOMAGONCDEBST) ? "no" : "yes");
			break;
		case 4:		// boolean for Märklin booster output
			sprintf (tmp, "%s", (syscfg.sysflags & SYSFLAG_NOMAGONMKLNBST) ? "no" : "yes");
			break;
		case 5:		// accessory repeat for all formats
			sprintf (tmp, "%d", cnf_getFMTconfig()->accrepeat);
			break;
		default:
			return NULL;
	}
	return kv_add (kv, key, tmp);
}

// ==============================================================================================
// === handling of ini file contents ============================================================
// ==============================================================================================

static void cnf_interpretIni (struct ini_section *ini)
{
	const struct section_map *sm;
	const struct keyhandler *kh;
	struct key_value *kv;

	while (ini) {
		sm = sections;
		while (sm->section && strcasecmp (sm->section, ini->name)) sm++;
		if (sm->section) printf ("[%s]\n", sm->section);
		kv = ini->kv;
		while (kv) {
			if ((kh = sm->handlers) == NULL) break;		// no handlers defined in this section
			while (kh->key && strcasecmp(kh->key, kv->key)) kh++;
			if (kh->key) {
				printf ("\t'%s' = '%s'\n", kv->key, (kv->value) ? kv->value : "(NULL)");
				if (kh->reader) kh->reader (kh->param1, kv);
			}
			kv = kv->next;
		}
		ini = ini->next;
	}
}

static struct ini_section *cnf_generateIni (void)
{
	const struct section_map *sm;
	const struct keyhandler *kh;
	struct ini_section *root, *ini;
	struct key_value *kv, *tmp;

	sm = sections;
	ini = root = NULL;
	while (sm->section) {
		ini = ini_add(ini, sm->section);
		if (!root) root = ini;
		if ((kh = sm->handlers) != NULL) {
			kv = NULL;
			while (kh->key) {
				if (kh->writer) {
					tmp = kh->writer(kv, kh->key, kh->param1);
					if (tmp != NULL) kv = tmp;
					if (!ini->kv) ini->kv = tmp;
				}
				kh++;
			}
		}
		sm++;
	}

	return root;
}

static void cnf_defconfig (void)
{
	memset (&syscfg, 0, sizeof(syscfg));
	memset (&fmtcfg, 0, sizeof(fmtcfg));

	// generic system defaults
	syscfg.sysflags = CNF_DEF_Sysflags;
	syscfg.ipm = CNF_DEF_IPMETHOD;
	syscfg.p50_port = CNF_DEF_P50_port;					// P50 -> P = ASCII 80, 50 -> 50, so we take 8050 as default
	syscfg.bidib.port = CNF_DEF_BIDIB_port;
	strcpy (syscfg.bidib.user, CNF_DEF_BIDIB_user);
	syscfg.locopurge = CNF_DEF_Locopurge;
	syscfg.mmshort = CNF_DEF_mmshort;					// external MM booster short after 100ms
	syscfg.dccshort = CNF_DEF_dccshort;					// external DCC booster short after 100ms
	syscfg.s88Modules = CNF_DEF_s88modules;
	syscfg.canModules = 0;
	syscfg.s88Frequency = CNF_DEF_s88frequency;

	fmtcfg.sigflags = CNF_DEF_Sigflags;

	// setup MM defaults
	fmtcfg.mm.repeat = CNF_DEF_mmrepeat;
	fmtcfg.mm.interpck_fast = CNF_DEF_mminterpck_fast;
	fmtcfg.mm.interpck_slow = CNF_DEF_mminterpck_slow;
	fmtcfg.mm.pause = CNF_DEF_mmpause;

	// setup DCC defaults
	fmtcfg.dcc.repeat = CNF_DEF_dccrepeat;
	fmtcfg.dcc.pomrepeat = CNF_DEF_dccpomrepeat;
	fmtcfg.dcc.preamble = CNF_DEF_dccpreamble;
	fmtcfg.dcc.tailbits = CNF_DEF_tailbits;
	fmtcfg.dcc.rc_tailbits = CNF_DEF_rc_tailbits;
	fmtcfg.dcc.tim_one = CNF_DEF_dcctim_one;
	fmtcfg.dcc.tim_zero = CNF_DEF_dcctim_zero;

	// setup M3 defaults
	fmtcfg.m3.repeat = CNF_DEF_m3repeat;
	fmtcfg.m3.beacon = CNF_DEF_m3beacon;
	fmtcfg.m3.announce = CNF_DEF_m3announce;

	// setup accessory default
	fmtcfg.accrepeat = CNF_DEF_accrepeat;
}

static void cnf_store (TimerHandle_t t)
{
	struct ini_section *ini;

	printf ("%s() Storing configuration\n", __func__);
	xTimerStop(t, 100);
	// some consistancy checks ...
	if (syscfg.mmshort < EXTERNSHORT_MIN) syscfg.mmshort = EXTERNSHORT_MIN;
	if (syscfg.mmshort > EXTERNSHORT_MAX) syscfg.mmshort = EXTERNSHORT_MAX;
	if (syscfg.dccshort < EXTERNSHORT_MIN) syscfg.dccshort = EXTERNSHORT_MIN;
	if (syscfg.dccshort > EXTERNSHORT_MAX) syscfg.dccshort = EXTERNSHORT_MAX;
	ini = cnf_generateIni();
	ini_writeFile(CONFIG_SYSTEM, ini);
	ini_free(ini);
	printf ("%s() Storage finished\n", __func__);
}

struct sysconf *cnf_getconfig (void)
{
	return &syscfg;
}

char *cnf_getBoosterLimits (void)
{
	static char response[256];

	if (!*response) {	// first call -> fill the string
		sprintf (response, "{ \"booster\": { \"sensmin\": %d, \"sensmax\": %d }}\n", EXTERNSHORT_MIN, EXTERNSHORT_MAX);
	}
	return response;
}

struct fmtconfig *cnf_getFMTconfig (void)
{
	return &fmtcfg;
}

struct sysconf *cnf_readConfig (void)
{
	struct ini_section *ini;

	cnf_defconfig();

	if ((ini = ini_readFile(CONFIG_SYSTEM)) != NULL) {
		cnf_interpretIni(ini);
		ini_free(ini);
		if (!(fmtcfg.sigflags & SIGFLAG_RAILCOM)) fmtcfg.sigflags &= ~SIGFLAG_DCCA;		// no DCC-A without RailCom
	}

	if (!storage_timer) {
		storage_timer = xTimerCreate("CFG-Storage", STORAGE_TIMEOUT, 0, NULL, cnf_store);
	}

	return &syscfg;
}

void cnf_triggerStore (const char *caller)
{
	if (storage_timer) {
		log_msg (LOG_INFO, "%s(): from %s()\n", __func__, caller);
		xTimerReset(storage_timer, 20);
	} else {
		log_msg (LOG_INFO, "%s(): from %s() ignored (timer not yet active)\n", __func__, caller);
	}
}
