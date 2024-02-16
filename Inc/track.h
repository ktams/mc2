/*
 * track.h
 *
 *  Created on: 27.12.2019
 *      Author: Andi
 *
 * THIS FILE NOW ONLY INCLUDES DOCUMENTATION - no need to include anywhere!
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
 * @page MM Märklin-Motorola format
 * <h2>MM (Märklin-Motorola in general)</h2>
 *
 * MM is original limited to 14 speeds, 5 functions (F0 - F4) and loco IDs 1 - 80
 * plus 0 as a broadcast. MM1 and MM2 use subtle different encodings. MM2 is constructed
 * in a way, that it is backward compatible to MM1.
 *
 * MM uses so called "trits" (trinary digits) to convey information in the packets. That
 * stems from the original Motorola (IR) remote control chips. To define an address you
 * had to tie a pin to GND, VCC or leave it open - so only three levels could be coded
 * per pin. Each trinary information was then coded in two "bits" (signals on the transport
 * medium) using only three codings out of the four possible: "00" for 0/GND, "11" for
 * 1/VCC and "10" for an open pin. Most protocol extensions are based on the use of the
 * double-bit trits as two distinct bits and take advantage of the full coding range.
 *
 * Both formats where "expanded" to the use of 255 loco IDs on non-Märklin / homebrew
 * decoders and professional decoders nowadays always include these extensions. Another
 * extension was made to support 27 speed steps. Unfortunately, two different methods
 * where developed and hence we have to distinguish between MM_27A and MM_27B (more on
 * that later).
 *
 * The MM format supports two different "address ranges": mobile and stationary decoders.
 * The stationary decoders are used for turnouts and function decoders (see MM1 below).
 * Stationary decoders are accessed with twice the bitrate of mobile decoders, so the
 * two ranges can't undestand the other ranges commands because of the bitrate mismatch.
 *
 * Function decoders only ever support the MM1 format.
 *
 * Commands to MM decoders always consist of four trits for the address, a "control" trit
 * (trit 5) and another four trits of "data". They are sent as double packets with a short
 * interpacket gap to make transport reliable (decoders only accept the packet, if it was
 * received two times with identical contents). This double packet is the MM base packet
 * which might be repeated sereveral times just as for other formats.
 *
 * In case of _mobile_ decoders, the data originally represented the speed only (no direction
 * information - there is only a "reverse" command). The "control" trit is used for the
 * "function" (F0, lights).
 *
 * _Stationary_ decoders use the "control" trit to distinguish between turnout and function
 * decoders. Function decoders use the "data" portion directly to control their four outputs.
 * Turnout decoders use three trits to decode one of eight outputs and the last trit as
 * "state" (ON/OFF), where OFF just ignores the decoded output and switches all outputs
 * to off.
 *
 * <h2>MM1 (Märklin-Motorola 1 - the old format)</h2>
 *
 * This format is somewhat limited for historical reasons. It supports only "the function"
 * (i.e. F0, usually lights). The need for more functions lead to the function decoders.
 * These decoders support the four functions F1 - F4 and where originally destined to
 * stationary gimmicks and therefore accessed with the double-speed packets.
 *
 * In the RB2 implementation, function decoders are dedicated to a MM1 decoder and supply
 * it with the missing functions F1 - F4. If such a decoder is really stand-alone without
 * (logical) coupling to a mobile decoder, you should not define a loco with such an ID
 * (to make it clear: you DO define a loco with that ID and format set to MM1, but should
 * not use a loco with the same ID).
 *
 * With MM1 the coding of the last four trits is always the speed - and only the codings
 * "0" and "1" are used - the "open" coding is not used.
 *
 * <h2>MM2 (Märklin-Motorola 2 - the new format)</h2>
 *
 * Except for the independantly from Märklin developed address and speed extensions, only the
 * four "data" trits are now taken as 8 data bits. To stay compatible with the older decoders,
 * the first bit of each trit in the data portion had to keep it's meaning (the MM1 decoders
 * only decoded the first bit of each trit). On the other hand, all MM1 codings, using the
 * standard coding of "00" and "11" had to keep their meaning, because a MM2 decoder should
 * also behave as it's MM1 cousin if it receives the MM1 standard packets.
 *
 * The goal was to include control for four functions (F1 - F4) and a speed coding, that
 * has an absolute direction information. It looks like there where 5 more bits necessary
 * plus a bit more to circumvent the backward compatibility issues.
 *
 * Because no additional bits should be added, there was a compromise made:
 * <ul>
 * <li> There are distinct codings for specifiying a speed, that includes a direction information
 * <li> Each function is sent alone together with a speed function, that doesn't include the
 * direction.
 * </ul>
 *
 * The representation of that information in the data portion of the packet is quite complicated.
 * It will be discussed in the package construction section.
 *
 * <h2>27 Speed steps</h2>
 *
 * This is also an unofficial extension to the format and can (in theory) be used for both MM1
 * and MM2. The original coding had four bits and so allowed 14 speeds plus STOP (speed 0) and
 * a "reverse" function. The coding used 0 for STOP, 1 for REVERSE and 2 to 15 for the speed
 * steps 1 to 14.
 *
 * The reversing resulted from the original AC design of the analog locos, that had no "polarity"
 * change like DC systems. A high voltage pulse was activating a relais, that switched one of
 * the motor windings. This is still one of the drawbacks in MM1 format today: You are not able
 * to tell the direction for MM1 locos (from controller point of view).
 *
 * 27 speed steps are constructed by inserting a middle step between two real steps. That
 * leads to 14 full steps plus 13 steps lying between these full steps. This way you could
 * label the available speed steps as decimals 1.0, 1.5, 2.0, 2.5, ... 14.0 - and there is
 * neither a 14.5 nor a 0.5!
 *
 * Two codings where developed independantly. One used a second speed packet that signaled
 * a speed just on step below the preceeding one, and the other just redefined the trit 5
 * (the "control" trit used for F0).
 *
 * The first method with two packets is called MM_27A and has the drawback of using more
 * bandwith on the track. Espacially, if you want two switch from an odd speed (this is the
 * coding of one of the original speeds 1, 3, 5, ... 27) to the next lower odd speed (i.e.
 * switch down two "half" steps) you first have to send a lower speed and then the higher
 * speed that makes up the requested speed step.
 *
 * The convenient way to command the intermediate steps is manipulating the second bit of
 * trit 5 (the control trit for F0). Usually, this trit is coded as "11" for F0=ON and "00"
 * for F0=OFF. The intermediate speed is coded as the one speed step higher with the second
 * bit of the control trit inverted (i.e. "10" for F0=ON and half speed step active or "01"
 * for F0=OFF and half speed step active). This coding indeed would allow 28 steps, because
 * there really is an additional bit of information, but the "half step lowered" 1.0 (to
 * achive 0.5) is not implemented, because it is unclear wether the decoders understand this
 * or not.
 *
 * <h2>Address range</h2>
 *
 * Historically the MM format is limited to 80 loco addresses plus a broadcast. The coding
 * on the track is quite "complicate", because each two signal bits form a trit and that
 * trit cannot (could not) take on all four values (see above). To further make life harder,
 * Märklin decided to transmit the trits in "little endian" (beginning with the LSB).
 *
 * The "weight" of a trit when calculating the resulting coding of a searched address is
 * "00" for 0, "11" for 1 and "10" for "open" - numerically 2. The coded numbers from
 * 1 to 79 are straight forward, but for address 80 Märklin choosed the "0"-Coding (four
 * trits with the value "00"). The broadcast address "0" is coded as four open trits "10"
 * or trinary 2222.
 *
 * The addresses where later expanded to use the originally forbidden coding "01" and thus
 * make the address simply an eight bit number. To keep compatibility, the coding of the
 * first 81 addresses was left as they were and only the upper addresses where defined
 * different. I never checked the scheme which was used here. There are tables available
 * containing the coding of all 256 addresses.
 *
 * <h2>Error checking</h2>
 *
 * MM doesn't support a CRC or something like this. It simply tries to receive two consecutive
 * packets with identical contents and then accepts the packet data.
 */

/**
 * @page DCC DCC by NMRA and RailCommunity
 *
 * <h2>DCC in general</h2>
 *
 * The DCC format was originally developed by Lenz and standardised by the NMRA and nowadays
 * clearified and extended by the RailCommunity.
 *
 * DCC allows 14, 28 and 126 speed steps, currently up to 29 functions (F0 + F1 - F28) and
 * always keeps track of the direction of travel (just to point out a difference to MM1).
 * It furthermore allows the control of up to 32767 state bits, which are designed to be
 * controlled but not stored in the command station (and in turn will never be part of a
 * refresh cycle).
 *
 * DCC has three address ranges: mobile decoders plus basic and extended accessory decoders.
 * Basic accessory decoders are used to switch turnouts. The ranges are kept apart by extra
 * coding bits, that officially are not part of the address.
 *
 * The DCC data stream is made up of whole bytes and the datagrams may have different lengths.
 * The length information is implcitly specified by the command used. The same command coding
 * may have different meaning in the three address spaces.
 *
 * <h2>DCC function control</h2>
 *
 * Traditionally, DCC supported a "function" (F0, usually lights), which is transmitted
 * together with the speed commands. A function control command adds the functions F1-F4.
 * This was then extended to support two more groups of four functions each (F5-F8 and
 * F9-F12). Later, two commands supported the functions F13-F20 and F21-F28.
 *
 * The binary state control is independant of the functions, i.e. the binary states 0 to 28
 * do not interfere with the functions of the same number.
 *
 * <h2>Error checking</h2>
 *
 * DCC uses an 8-bit XOR sum of all preceeding bytes. Modern decoders supporting RailCom may
 * acknowledge a received package to shortcut further repetitions of the same packet to raise
 * the probability of a successful packet reception.
 */

/**
 * @page M3 The M3 format
 *
 * To keep up with DCC, Märklin had to renew it's digital control format. Together with ESU
 * they created a new format which is not backward compatible with the Märklin-Motorola codings
 * any more.
 *
 * We tried to adopt this format to achive a certain compatibilty and named our interpretion
 * "m3" (the third format from Märklin).
 *
 * MFX<sup>&copy;</sup> is a registered trademark of Gebrüder Märklin & Cie. GmbH, 73033 Göppingen, Germay.<br>
 * ESU<sup>&copy;</sup> is a registered trademark of ESU electronic solutions ulm GmbH & Co. KG, 89081 Ulm, Germany.
 *
 * Most of the information needed to implement this signal scheme in a compatible way stems
 * from Stefan Krauß and his Team. The implemtation itself is done by Tams and Kretzer.
 *
 * <h2>M3 key points</h2>
 *
 * M3 supports 126 speed steps, 128 functions and a feedback channel on the track.
 *
 * Of course it supports a lot more interesting things, but this is the basic stuff that we are
 * corcerned with.
 *
 * The format allows to combine speed and function information in a single packet and shortcut
 * some commands under certain conditions to save bandwith on the track. The loco ID 0 is reserved
 * for broadcast commands. Thru bit stuffing, a coexistance with DCC is guaranteed. An eight bit
 * package CRC is included for reliable transport.
 *
 * The format is not byte aligned. Any number of data bits may make up a packet. The CRC "byte"
 * is defined to consist of the last 8 bits of a packet and needs not to start at a bit number
 * divisable by 8.
 *
 * The M3 format is only defined for mobile decoders. For turnouts, there still is the MM1
 * format in use.
 *
 * <h2>M3 address range</h2>
 *
 * M3 allows loco IDs from 1 to 16383 with 0 as a broadcast. This makes up a complete 14 bit address.
 * To save bandwith, the address may be coded with less bits, if the ID fits into this number of bits.
 * The address may be transmitted as 7 bits, 9 bits, 11 bits or the full 14 bits. As with DCC these
 * ranges do not make up a distinct address ranges!
 *
 * <h2>M3 speed steps</h2>
 *
 * The coding of 126 speed steps + STOP (= 0) and Emergency stop (coded as 1) is almost the same
 * as in DCC. The direction bit can be seen as bit #7 of the speed information but has an inverse
 * meaning to DCC: if set, this marks a reverse direction.
 *
 * <h2>M3 functions</h2>
 *
 * The format supports functions from F0 to F127. There are several ways to control these functions.
 * Two commands address the functions F0-F3 or F0-F7. These commands explicitly clear the function
 * bits F4-F15 or F8-F15 respectively and so can only be used, if the implicitly cleared functions
 * are at the cleared state in the control. The third command directly controls all 16 functions
 * F0-F15. None of these three commands cares about function F16 and above.
 *
 * The last command to control functions consists of a function number and its state. With this
 * command it is possible to control each function F0-F127 individually.
 *
 * <h2>Error checking</h2>
 *
 * M3 supports a real 8-bit CRC and automatically take the last 8 bits of a command packet as this
 * CRC. It has however no support for an acknowledgement. Packet therefore must always be repeated
 * a certain number of times to assure a reliant delivery.
 */

/**
 * @file
 *
 * @defgroup Track Signal generation and Formats
 *
 * <h2>Internal representation of locos</h2>
 *
 * Irrespective of the format of each individual loco, the information for each loco is identical.
 * That means, that this internal representation must meet all supported options for all supported
 * formats.
 *
 * <b>Speed</b>: We use a modified version of the DCC coding. The speed fits into a single byte (but the
 * structure member is defined as an int - so it occupies 32 bits). Bit #7 denotes the direction,
 * just as in DCC with a set bit meaning _forward_. The speed itself is noted as a simple number
 * from 0 to 126 (the individual maximum depends on the format for that decoder).
 *
 * <b>Functions</b>: Currently we support up to 128 function bits in four unsigned 32 bit variables.
 * If this should ever be insufficient, it can easyly be extended.
 *
 * <h2>Addresses</h2>
 *
 * The basic formats and even some subformat information devides the addressable range of loco IDs
 * in different address domains. Technically, it would be possible to have a decoder with ID 10 in
 * at least three formats: MM, DCC and M3. From the user perspective, this would require to choose
 * a format in addition to a loco ID to gain control over that loco. This is not practical!
 *
 * Therefore, the address range of all mobile decoders is packed together and only the accessory /
 * turnout decoders have a different address range. That means:
 * <ul>
 * <li> All three formats may define locos with IDs 1 to 255.
 * <li> DCC and M3 may additionally define locos with addresses 256 to 10239.
 * <li> M3 may additionally define locos with addresses 10240 to 16383.
 * <li> Each loco ID exists only once in the system and may denote a loco with any of the
 * supported formats.
 * <li> Turnout decoders may be defined for decoder addresses 1 to 255 with MM or DCC format.
 * <li> Turnout decoders with DCC format can additionally have adresses 256 to 510.
 * </ul>
 *
 * <h2>Addressing Turnouts / Accessory decoders</h2>
 *
 * Traditionally, turnout decoders handled 4 two-way turnouts each. A two way turnout is
 * controlled with two electro magnets (the real implementation may vary, some people use
 * model servos for that). This scheme leads to a "decoder" address, that is subdivided
 * into four individual turnouts which each have two _paired_ outputs.
 *
 * Märklin constructed the format (and it's hardware) in a way, that each of the eight outputs
 * could be switched on individually and all are switched off by just one command. The decoder
 * address can range from 1 to 80 (or 255 in newer systems). That allows MM to control up to
 * 1020 turnouts or 2040 individual outputs.
 *
 * DCC controls all eight outputs more or less individual. It has an address range of 1 to
 * 510 that allows to control 2040 turnouts or 4080 individual outputs. The addresses 0
 * and 511 were originally not used and later inserted "irregular" by some controls.
 *
 * <h2>more information regarding the formats</h2>
 *
 * For some information related to the Märklin-Motorola format see @ref MM.<br>
 * For a few remarks on the widely used DCC format see @ref DCC.<br>
 * Märklin's new format M3 is introduced in @ref M3.
 *
 */
