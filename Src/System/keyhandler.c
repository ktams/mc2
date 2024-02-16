/*
 * keyhandler.c
 *
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

#include <stdio.h>
#include "rb2.h"
#include "events.h"
#include "bidib.h"

static volatile TaskHandle_t pairing;		// if set, GO means OK and STOP means "NO" and also switch off track

static void disp_ip (void)
{
	keyevent k;
	char buf[20], *s;
	uint8_t cl, cr;

	if (rt.en) {
		seg_display(0b0110000,  0b1110011);
		do {
			k = key_getEvent(portMAX_DELAY);
			if (KEY(k) != KEY_GO) return;
		} while (k != BREAK(KEY_GO));

		// format IPv4 Address as four tuples of 3 digits each.
		// ATTENTION: IP-Address is in network byte order!
		sprintf (buf, "%03lu.%03lu.%03lu.%03lu",
				(rt.en->ip_addr.addr >> 0) & 0xFF, (rt.en->ip_addr.addr >> 8) & 0xFF,
				(rt.en->ip_addr.addr >> 16) & 0xFF, (rt.en->ip_addr.addr >> 24) & 0xFF);
		printf ("%s() IP-Addr = %s\n", __func__, buf);
		s = buf;
		while (*s) {
			cl = seg_getHexDigit(*s++ - '0');
			if (*s == '.') {
				cl |= 0x80;
				s++;
			}
			if (*s) {
				cr = seg_getHexDigit(*s++ - '0');
				if (*s == '.') {
					cr |= 0x80;
					s++;
				}
			} else {
				cr = 0;
			}
			seg_display(cl,  cr);
			do {
				k = key_getEvent(portMAX_DELAY);
				if (KEY(k) != KEY_GO) return;
			} while (k != BREAK(KEY_GO));
		}
	} else {
		fprintf (stderr, "%s(): Interface not (yet) defined\n", __func__);
		seg_display(0b1000000, 0b1000000);	// show two dashes ('--')
		key_getEvent(portMAX_DELAY);		// wait for any key event (probably release of the GO key)!
	}
}

void vKeyHandler (void *pvParameter)
{
	keyevent k;
	int go, stop;
	bool reboot_flag = false;

	(void) pvParameter;

	go = stop = 0;

	for (;;) {
		k = key_getEvent(pdMS_TO_TICKS(100));
//		if (KEY(k) == KEY_STOP || KEY(k) == KEY_GO) event_fire(EVENT_KEY_PRESS, k, NULL);	KEY-Events were only a development aid
		switch (k) {
			case NOKEY:
				if (go) go++;		// count GO as long as GO is not released
				if (stop) stop++;	// count STOP as long as STOP is not released
				if (!stop && (go > 10)) {	// show IP if GO (but not STOP) is pressed for more than a second
					disp_ip ();
					seg_display(0, 0);
					vTaskDelay(pdMS_TO_TICKS(500));
					seg_display(0, 0);
					go = 0;
				}
				if (!go && (stop > 10)) {	// BiDiB identify if STOP (but not GO) is pressed for more than a second
					bidib_identifyToggle();
					stop = 0;
				}
				if (go > 20 && stop > 20) {
					go = stop = 0;		// we only fire once!
					sig_setMode(TM_RESET);
					reboot_flag = true;
				}
				break;
			case MAKE(KEY_STOP):
//				player_play("/detodos.opus");
				sig_setMode(TM_STOP);
				if (pairing) xTaskNotify(pairing, 0, eSetValueWithOverwrite);
				stop = 1;
				break;
			case BREAK(KEY_STOP):
				if (reboot_flag && !go) {	// this reboot is triggered, when the GO button is released first
					reboot();			// RESET if both GO and STOP are hold pressed for more than two seconds and then released
				}
				stop = go = 0;		// when releasing STOP + GO you never will know which one comes first ...
				break;
			case MAKE(KEY_GO):
				if (pairing) xTaskNotify(pairing, 1, eSetValueWithOverwrite);
				else go = 1;
				break;
			case BREAK(KEY_GO):
				if (reboot_flag && !go && !stop) {	// this reboot is triggered, when the STOP button is released first
					reboot();			// RESET if both GO and STOP are hold pressed for more than two seconds and then released
				}
				if (!stop && go > 0) {
					sig_setMode(TM_GO);
				}
				go = 0;
				break;
			case MAKE(KEY_BIDIB_ACK):
				// @TODO handle according to BiDiB SPEC (==> STOP BOOSTER and signal generation?)
				log_msg(LOG_WARNING, "%s() BiDiBus Emergency STOP\n", __func__);
				sig_setMode(TM_STOP);
				break;
			case BREAK(KEY_BIDIB_ACK):
				log_msg(LOG_INFO, "%s() BiDiBus Emergency STOP ended\n", __func__);
				break;
			case MAKE(MB_SHORT):
				if (rt.tm == TM_GO || rt.tm == TM_HALT) {		// in the other modes this is probably a late reaction of the booster to a STOP
					sig_setMode(TM_SHORT);
					printf("MB SHORT!\n");
				}
				break;
			case MAKE(DCC_SHORT):
				if (rt.tm == TM_GO || rt.tm == TM_HALT) {		// in the other modes this is probably a late reaction of the booster to a STOP
					sig_setMode(TM_SHORT);
					printf("DCC SHORT!\n");
				}
				break;
		}
	}
}

bool key_pairing (void)
{
	unsigned long rc;

	pairing = xTaskGetCurrentTaskHandle();
	seg_pairing(true);
	rc = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30000));
	seg_pairing(false);
	pairing = NULL;
	sig_setMode(rt.tm);
	return (rc == 1);
}
