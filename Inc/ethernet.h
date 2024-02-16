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

#ifndef __ETHERNET_H__
#define  __ETHERNET_H__

typedef enum {
    eLINKDOWN,	    ///< wait for a cable to connect and auto negotiation to complete
    e100FDX,	    ///< 100Mbs full duplex
    e100HDX,	    ///< 100Mbs half duplex
    e10FDX,	    	///< 10Mbs full duplex
    e10HDX,	    	///< 10Mbs half duplex
} linkstate;	    ///< the status of the cable link

#define PHY_ADDR		0	///< the address of the phy in use (usually 1, but micrel only supports 0 and 3)

/*
 * Prototypes ksz8081.c
 */
bool ksz8081_isup (linkstate state);
linkstate ksz8081_getstate (void);
void ksz8081_autonegotiation (void);
void ksz8081_setup_phy (TaskHandle_t deferredHandler);

/*
 * Prototypes stm_ethernet.c
 */
err_t stmenet_init (struct netif *netif);

#endif	/* __ETHERNET_H__ */
