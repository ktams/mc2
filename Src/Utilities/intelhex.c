/*
 * intelhex.c
 *
 *  Created on: 13.03.2021
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
#include "intelhex.h"

static unsigned ihex_value (char c)
{
   if (c >= '0' && c <= '9') return (c - '0');
   if (c >= 'A' && c <= 'F') return (c - 'A' + 10);
   if (c >= 'a' && c <= 'f') return (c - 'a' + 10);
   return 0;
}

/**
 * Scans two characters of string pointed to by 's'
 * and tries to interpret them as 8-bit byte.
 *
 * \param s		the string that should be examined
 * \return		the byte that was decoded (no error code possible)
 */
uint8_t hex_byte (char *s)
{
	if (s) return (ihex_value(s[0]) << 4) | ihex_value(s[1]);
	return 0;
}

/**
 * Scans four characters of string pointed to by 's'
 * and tries to interpret them as 16-bit value.
 *
 * \param s		the string that should be examined
 * \return		the short value that was decoded (no error code possible)
 * \see			hex_byte()
 */
uint16_t hex_short (char *s)
{
	if (s) return (hex_byte(s) << 8) | hex_byte(s + 2);
	return 0;
}

/**
 * Scans eight characters of string pointed to by 's'
 * and tries to interpret them as 32-bit value.
 *
 * \param s		the string that should be examined
 * \return		the word value that was decoded (no error code possible)
 * \see			hex_byte()
 */
uint32_t hex_word (char *s)
{
	if (s) return (hex_short(s) << 16) | hex_short(s + 4);
	return 0;
}

/**
 * Interpret a line of text as intel hex. The line doesn't need to be
 * terminated by CR/NL or even a null byte.
 *
 * Intel hex lines start with a colon ':'. every other character is
 * treated as comment and results in returning 0 bytes of decoded
 * data.
 *
 * The data structure will be filled with decoded data and relative address
 * if it is a data record or the segment address in case of segment address
 * record. The end marker is returned as having read zero bytes but with the
 * status set to IHEX_END.
 *
 * \param d		pointer to the structure holding status, address and decoded data
 * \param s		pointer to the beginning of the lien to be decoded, NULL pointer is ignored
 * \return		number of valid decoded data bytes in the result buffer, 0 for uninterpreted
 * 				lines or lines that do not supply real data (segment address record, end
 * 				record, ...) or a negative value on error (status is set to IHEX_ERROR in
 * 				this case).
 */
int ihex_readline (struct ihexdata *d, char *s)
{
   int i;
   uint8_t sum, bytes, type;
   char *ln = s;

   if (!d) return -1;				// we definitely need the status/data structure!
   if (!s || *s != ':') return 0;	// may be a comment or something else - this line is ignored

   bytes = hex_byte(s + 1);
   d->reladr = hex_short(s + 3);
   type = hex_byte(s + 7);
   if (bytes > sizeof(d->data)) {
	   log_msg(LOG_WARNING, "%s(): excessive data length %d bytes\n", __func__, bytes);
	   d->state = IHEX_ERROR;
	   return -2;
   }
   s += 9;		// now pointing to begin of data section
   sum = -1 + bytes + (d->reladr >> 8) + (d->reladr & 0xff) + type;
   for (i = 0; i < bytes; i++, s+= 2) {
	   d->data[i] = hex_byte(s);
	   sum += d->data[i];
   }
   i = hex_byte(s);
   if ((~sum & 0xFF) != i) {	// error in checksum
	   log_msg(LOG_WARNING, "%s(): '%*.*s': wrong checksum 0x%02x (should be 0x%02x)\n", __func__, bytes * 2 + 11, bytes * 2 + 11, ln, i, ~sum & 0xFF);
	   d->state = IHEX_ERROR;
	   return -3;
   }

   switch (type) {
	   case 0:	// data record
		   return bytes;
	   case 1:	// end of file record
		   d->state = IHEX_END;
		   return 0;
	   case 2:	// extended segment adress record
	   case 4:	// extended linear adress record
		   d->segadr = 0;
		   for (i = 0; i < bytes; i++) {
			   d->segadr <<= 8;
			   d->segadr |= d->data[i];
		   }
		   if (type == 4) d->segadr <<= 16;
		   else d->segadr <<= 4;
		   return 0;
	   default:
		   log_msg (LOG_WARNING, "%s(): unknown type 0x%02x\n", __func__, type);
		   break;
   }
   return 0;
}
