/**
 * @file config.h
 *
 * @author Andi
 * @date   12.04.2020
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

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define CONFIG_DIR		"/config/"					///< the directory where all our configuration files should lie
#define FIRMWARE_DIR	"/uploads/"					///< the directory where all our firmware update files should lie
#define MANUALS_DIR		"/manuals/"					///< the directory where all our PDF manuals go
#define CONFIG_LOCO		CONFIG_DIR"loco.ini"		///< the ini file that holds the settings for all known locos
#define CONFIG_SYSTEM	CONFIG_DIR"config.ini"		///< system settings like IP-configuration, track voltage and so on
#define CONFIG_BIDIB	CONFIG_DIR"bidib.ini"		///< known and trusted netBiDiB clients and node configurations
#define FLASH_FILE		FIRMWARE_DIR"zentrale"		///< if this file exists, it was used to flash the application - truncate it to zero length at startup

// sys flags
#define SYSFLAG_LONGPAUSE			0x0001	///< MM long pause, not used (old EasyControl)
//#define SYSFLAG_M3ENABLED			0x0002	///< Output m3 packets (should default to "on") / 05.02.2022: this belongs to format settings
#define SYSFLAG_NEGATIVESHORT		0x0004	///< not used (old EasyControl)
#define SYSFLAG_DEFAULTDCC			0x0008	///< locos are DCC by default, not used (old EasyControl)
#define SYSFLAG_DCMODE				0x0010	///< not used (old EasyControl)
#define SYSFLAG_AUTOSTEP			0x0020	///< not used (old EasyControl)
#define SYSFLAG_NOMAGONMAINBST		0x0040	///< no magnet command on internal booster
#define SYSFLAG_NOMAGONCDEBST		0x0080	///< no magnet command on CDE booster output
#define SYSFLAG_NOMAGONMKLNBST		0x0100	///< no magnet command on MM booster output
#define SYSFLAG_LIGHTEFFECTS		0x0200	///< Enable RGB light effects (entertainment)
#define SYSFLAG_ACC_LOGICAL			0x0400	///< BiDiB: ACC addresses are logical, not DCC (i.e. user turnout 1 is specified as 0)
#define SYSFLAG_LIGHTSOFF			0x0800	///< RGB lights nearly off
#define SYSFLAG_STARTSTATE			0x1000	///< After Power on -> Stop or Go
#define SYSFLAG_GLOBAL_BIDIB_SHORT	0x2000	///< a SHORT on a BiDiB-Booster will set the whole system to SHORT status (controller mode only)
#define SYSFLAG_BIDIB_ONOFF			0x4000	///< if set, BiDiB Booster's STOP and GO keys are functional and work system wide

// format flags
#define SIGFLAG_RAILCOM				0x0001	///< generate railcom cutout
#define SIGFLAG_DCCA				0x0002	///< handle DCCA (RCN-218, automatic address assignment for decoders)
#define SIGFLAG_DCCNOP				0x0004	///< send out ACC-NOPs to give accessories the opportunity to report status
#define SIGFLAG_M3ENABLED			0x0008	///< Output m3 packets (should default to "on")
#define SIGFLAG_DCC_LONG_ADR		0x0010	///< always use long addresses with DCC (debug aid)

#define EXTERNSHORT_MIN				50		///< a minimum short time for external boosters in ms that can be set
#define EXTERNSHORT_MAX				2000	///< a maximum short time for external boosters in ms that can be set

#define MILLISECONDS_PER_STEP		5		///< step the voltage by 0,1V every 5 calls (-> milli seconds)
#define BOOSTER_TIMEOUT				1000	///< the minimum time between power OFF and ON again to allow for discharging the capacitor
#define RELAIS_TIMEOUT				50		///< additional time after switching the programming track relais
#define RELAIS_DISENGAGE			5000	///< time (in ms) that an activated programming track relais is kept engaged in booster off mode
#define MIN_VOLTAGE					80		///< minimum output voltage is 8,0V
#define MAX_VOLTAGE					220		///< maximum output voltage is 22,0V
#define MIN_PRGVOLTAGE				80		///< minimum programming voltage is 8,0V
#define MAX_PRGVOLTAGE				200		///< maximum programming voltage is 20,0V
#define MIN_CURRENT					1000	///< the minimum short current settable is 1,0A
#define MAX_CURRENT_TAMS			9000	///< the maximum short current settable is 6,5A (Tams)
#define MAX_CURRENT_KM1				9000	///< the maximum short current settable is 9,0A (KM-1)
#define MAX_LIMITER					500		///< the maximum short current in current limiter situation (CC regulation)
#define MIN_SENSITIVITY				20		///< the minimum sensitivity in ms
#define MAX_SENSITIVITY				2000	///< the maximum sensitivity in ms
#define MIN_INRUSH					100		///< the minimum inrush time in ms
#define MAX_INRUSH					500		///< the maximum inrush time in ms

#define SHORT_CURRENT_TAMS			9200			///< a current that is taken as absolute short overcurrent. Leads to immedeate switch off
#define SHORT_CURRENT_KM1			9200			///< a current that is taken as absolute short overcurrent. Leads to immedeate switch off

#define VTRACK						18				///< default track voltage [V]
#define TRACKCURRENT				50				///< default max. track current [100mA]
#define SHORTSENS					100				///< default short sensitivity [ms]
#define INRUSH						500				///< default inrush current time [ms]

enum ipmethod {
	IPMETHOD_DHCP = 0,					///< configuration per DHCP
	IPMETHOD_MANUAL,					///< manual IPv4 configuration
};

/**
 * This structure is filled from the configuration file and interrogated whenever a
 * module is added to the system that supports the BIDIB_CLASS_OCCUPANCY in it's
 * UID.
 */
struct bidib_feedback {
	struct bidib_feedback	*next;		///< linked list
	uint8_t					 uid[7];	///< the UID of a node (only the 5 bytes from uid[2]..uid[6] are used to identify a node)
	int						 s88base;	///< the base address in s88 domain tp map this node to (0 based, no need to align to s88 module boundary)
};

struct sysconf {
	enum ipmethod		ipm;			///< the choosen type of IPv4 configuration
    ip4_addr_t			ip_addr;		///< IPv4 address to use, when manual configuration is requested (in network byte order)
    ip4_addr_t			ip_mask;		///< IPv4 netmask to use, when manual configuration is requested (in network byte order)
    ip4_addr_t			ip_gw;			///< IPv4 default gateway to use, when manual configuration is requested (in network byte order)
    uint16_t			p50_port;		///< Port to use for P50(X[ab]) in host byte order
    volatile uint32_t	sysflags;		///< some flags controlling overall system behavior (SYSFLAG_...)
    int					locopurge;		///< time for purging unused locos (in minutes)
    int					mmshort;		///< timing for MM booster short recognition
    int					dccshort;		///< timing for DCC booster short recognition
    int					s88Modules;		///< No of s88 modules
    int					canModules;		///< No of can modules
    int					lnetModules;	///< No of loconet modules
    int					s88Frequency;	///< speed of s88 bus in Hz
    struct {
        uint16_t			port;		///< Port to use for netBiDiB TCP in host byte order
        char				user[32];	///< a user configurable name of this device (up to 24 characters + null byte)
    } bidib;
    struct bidib_feedback	*bidibfb;	///< the configured BiDiB feedback modules to map to s88 space
};

struct fmtconfig {
	struct {
		int			repeat;				///< number of repetition for each command
		uint32_t	interpck_slow;		///< pause between the two blocks of the double-block (SLOW, 1250us)
		uint32_t	interpck_fast;		///< pause between the two blocks of the double-block (FAST, 625us)
		uint32_t	pause;				///< pause between successive (double-)blocks (SHORT: 1.5ms or LONG: 4.025ms)
	} mm;
	struct {
		int			repeat;				///< number of repetition for each command
		int			pomrepeat;			///< number of repetition for POM commands
		int			preamble;			///< preamble length in bits
		int			tailbits;			///< number of tailbits without railcom
		int			rc_tailbits;		///< number of tailbits when using railcom
		int			tim_one;			///< length of a 1-bit in µs (both levels)
		int			tim_zero;			///< length of a 0-bit in µs (both levels)
	} dcc;
	struct {
		int			repeat;				///< number of repetition for each command
		uint32_t	beacon;				///< beacon-ID for M3
		uint16_t	announce;			///< announce counter for M3
	} m3;
	volatile uint32_t	sigflags;		///< additional information: railcom, dcca, ...
	int			accrepeat;				///< number of repetition for accessory decoders in all formats
};

/*
 * Prototypes System/config.c
 */
struct sysconf *cnf_getconfig (void);
char *cnf_getBoosterLimits (void);
struct fmtconfig *cnf_getFMTconfig (void);
struct sysconf *cnf_readConfig (void);
void cnf_triggerStore (const char *caller);


#endif /* __CONFIG_H__ */
