/*
 * events.h
 *
 *  Created on: 15.12.2019
 *      Author: Andi
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

#ifndef __EVENTS_H__
#define __EVENTS_H__

#define QUEUE_WAIT_TIME				100			///< time to wait for standrad requests finding a free slot in the event queue

enum event {
	EVENT_TIMEOUT = 0,							///< a dummy event that is fired if a timeout is specified and no event happened
	EVENT_SYS_STATUS,							///< system status has changed (STOP/GO/HALT/SHORT/...)
	EVENT_LOCO_SPEED,							///< a loco has changed it's speed
	EVENT_LOCO_FUNCTION,						///< a loco has changed functions
	EVENT_LOCO_PARAMETER,						///< parameters of a loco have changed
	EVENT_TURNOUT,								///< a turnout was switched (straight/thrown, ON/OFF)
	EVENT_FEEDBACK,								///< an event on the feedback busses happened (s88, can, lnet, bidib)
	EVENT_CURRENT,								///< a change in track current occured (uses a threshold to not permanently nag)
	EVENT_INSTANEOUS_CURRENT,					///< a change in track current occured (is reported immedeately - used for overcurrent protection)
	EVENT_NEWLOCO,								///< a new loco was detected on the track (DCC/Railcom and M3)
	EVENT_BOOSTER,								///< a booster settings and routings to the interfaces
	EVENT_SNIFFER,								///< a display filter in modul sniffer
	EVENT_PROTOCOL,								///< several protocol settings
	EVENT_ACCESSORY,							///< accessory settings
	EVENT_ENVIRONMENT,							///< the measured temperature or supply voltage has changed
	EVENT_CONTROLS,								///< changes regarding the external controls
	EVENT_RAILCOM,								///< RailCom messages except ACK and NACK
	EVENT_ACCFMT,								///< turnout format changed
	EVENT_LOCO_DB,								///< deliver all loco decoder adresses stored in the loco data base
	EVENT_MODELTIME,							///< fired every model minute
	EVENT_LOGMSG,								///< System-Logs for WEB interface
	EVENT_BIDIDEV,								///< new BiDiB device, device disapeared or a BiDiB pairing request
	EVENT_EXTCONTROL,							///< the status of the external control changed (param is the new controlling interface)
	EVENT_LIGHTS,								///< controls the light effects
	EVENT_ENBOOT,								///< EasyNet boot progress
	EVENT_CONSIST,								///< a consist changed, inform the WEB client
	EVENT_FBNEW,			///< TODO: temporary dummy event to replace EVENT_FEEDBACK!
	EVENT_FBPARAM,								///< some configuration in s88 system changed

	EVENT_MAX_EVENT,							///< a marker for the highest defined event type
	EVENT_DEREGISTER_ALL = 255					///< a pseudo event to deregister all events at once for a handler
};

/**
 * Events for system status changes
 */
enum sys_events {
	SYSEVENT_STOP,								///< system (track-) status has changed to STOP
	SYSEVENT_HALT,								///< system (track-) status has changed to HALT
	SYSEVENT_GO,								///< system (track-) status has changed to GO
	SYSEVENT_SHORT,								///< system (track-) status has changed to SHORT
	SYSEVENT_TESTDRIVE,							///< system (track-) status has changed to TESTDRIVE (limited current on programming track)
	SYSEVENT_RESET,								///< system is preparing for a RESET
	SYSEVENT_OVERTEMP,							///< system is too hot
	SYSEVENT_SIGON,								///< system will provide a track signal but internal booster stays off

	// new approach with distinction of booster / signal generation / overall system status
	// Events for signal generation status / prog track stati
	SYSEVENT_STOP_REQUEST,						///< a STOP was requested via STOP button on mc2 or a control attached to one of the interfaces
	SYSEVENT_HALT_REQUEST,						///< a SOFT STOP was requested (all locos receive speed 0) -> our old HALT state
	SYSEVENT_GO_REQUEST,						///< a GO was requested via GO button on mc2 or a control attached to one of the interfaces
	SYSEVENT_GO_WD_REQUEST,						///< same as GO, but including a watchdog (BiDiB-Feature)

	// Events for booster related stati
	SYSEVENT_INT_SHORT,							///< internal booster reports SHORT
	SYSEVENT_INT_OVERHEAT,						///< internal booster reports OVERHEAT
	SYSEVENT_INT_COOLDOWN,						///< internal booster reports cooled down again
	SYSEVENT_MRK_SHORT,							///< märklin booster reports SHORT
	SYSEVENT_CDE_SHORT,							///< CDE booster reports SHORT
	SYSEVENT_BIDIB_SHORT,						///< a BiDiB booster reports SHORT via its communication channel (not the ACK pin)
	SYSEVENT_BIDIB_EMERGENCY,					///< the BiDiBus fires an ermergency stop (> 10ms of LOW at ACK pin)
};

/**
 * A special structure holding the information for feedback events.
 * For the sake of the old s88 bus and P50x interface, a "module" is
 * the equivalent of a s88 module, reporting 16 feedback bits each.
 *
 * Internally, the module number is 0-based but to the outside world
 * (i.e. WEB, P50/P50x/P50xa) it should look 1-based.
 *
 * Each report only ever contains the updated information of a single
 * s88 module (i.e. 16 bits, no matter what the source of this input was).
 */
typedef struct {
#if 1
	int				module;						///< the s88 module number (0-based)
	uint16_t		status;						///< the status bits for the 16 inputs (a set bit represents an occupied track)
	uint16_t		chgflag;					///< a set bit for every feedback bit that changed
#else
	int				 modcnt;					///< count of reported 16-bit feedback units (i.e. dimension information for sum and evFlag)
	uint16_t		 *sum;						///< summation of feedback status (each module contains 16 bits)
	uint32_t		 evFlag[];					///< a bit flag for each changed s88 module
#endif
} fbeventT;

#define EVTFLAG_FREE_SRC		0x0001			////< src was an allocated structure, which should be freed after all callbacks are done with it

typedef struct {
	enum event		ev;							///< the event type that is reported with this event
	int				param;						///< an integer parameter describing the event in more detail (i.e. what key is pressed for EVENT_KEY_PRESS)
	TaskHandle_t	tid;						///< the task handle (task ID) of the task that generated the event (can be used to check for own events)
	void			*src;						///< an additional pointer to something that might have triggered the event (i.e. a loco)
	uint32_t		flags;						///< sone flags ...
} eventT;

struct evtListener {
	struct evtListener	*next;					///< singly linked list of listeners
	bool (*handler) (eventT *e, void *prv);		/**< The handler function that is to be called with the event and it's private data.
												 *	 If this handler returns <i>false</i>, it is removed from the listener list. In
												 *	 this case, it should release any resources that it might have allocated when
												 *	 registering itself as an event listener.
												 */
	TickType_t			 timeout;				///< a possible timeout when waiting for events
	TickType_t			 to_tim;				///< the time at which the currently running timeout triggers - recalculated after each handler call
	uint32_t			 ev_mask;				///< a mask for the events that this listener is interested in
	void				*private;				/**< Private data for the called back function to identify the requester. If this
												 *	 data is dynamically allocated (malloc(), etc.) it must be freed before returning
												 *	 <i>false</i> from the handler function (which means that the handler wishes
												 *	 to unregister itself).
	 	 	 	 	 	 	 	 	 	 	 	 */
};

typedef bool(*ev_handler)(eventT *, void *);

/*
 * Prototypes System/eventlistener.c
 */
int event_register (enum event evt, ev_handler handler, void *prv, TickType_t timeout);
int event_deregister (enum event evt, ev_handler handler, void *prv);
int event_fireEx (enum event evt, int param, void *src, uint32_t flags, TickType_t timeout);
int event_fire (enum event evt, int param, void *src);

/*
 * Prototypes Interface/easynet.c
 */
void en_reportControls (void);

/*
 * Prototypes Interface/loconet.c
 */
void ln_reportControls (void);

/*
 * Prototypes Interface/mcan.c
 */
void mcan_reportControls (void);

/*
 * Prototypes Interface/xpressnet.c
 */
void xn_reportControls (void);

#endif /* __EVENTS_H__ */
