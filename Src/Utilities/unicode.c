/*
 * unicode.c
 *
 *  Created on: 26.02.2022
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

#include <string.h>
#include "rb2.h"

#define UTF8_CONTINUATION(c)	(((c) & 0xC0) == 0x80)

/**
 * Check for the length of the UTF-8 encoding sequence. The character presented
 * to the function is checked for a valid start pattern that specifies the total
 * length of the character in bytes.
 *
 * \param c			the first character of a (possible) multibyte sequence
 * \return			the length of the sequence or zero if the character is coded
 * 					as a continuation character (binary 0b10....)
 */
static size_t utf8_getLenCode (uint8_t c)
{
	if ((c & 0x80) == 0x00) return 1;		// standard 1-byte character (ASCII)
	if ((c & 0xE0) == 0xC0) return 2;		// code starting with 0b110
	if ((c & 0xF0) == 0xE0) return 3;		// code starting with 0b1110
	if ((c & 0xF8) == 0xF0) return 4;		// code starting with 0b11110
	return 0;								// possible continuation character starting with 0b10 or illegal code with more than 4 '1'-bits
}

/**
 * Count number of bytes needed to code a unicode codepoint in UTF-8.
 * Note: UTF-8 may use up to 21 effective bits to code a character. That
 * would allow codepoints from 0 to 0x1FFFFF. But unicode defines only
 * codepoints up to 0x10FFFF, so codes beyond that value result in uncoded
 * (ignored) values taking zero bytes!
 *
 * \param codepoint		the unicode codepoint in the range of 0x000000 to 0x10FFFF
 * \return				number of bytes need to represent the codepoint in UTF-8
 * 						or zero if the codepoint is out of range
 */
int utf8_bytelen (uint32_t codepoint)
{
	if (codepoint < 0x80) return 1;
	if (codepoint < 0x800) return 2;
	if (codepoint < 0x10000) return 3;
	if (codepoint < 0x11000) return 4;
	return 0;
}

uint32_t utf8_codepoint (const void *s)
{
	uint8_t *p;

	if ((p = (uint8_t *) s) != NULL) {
		switch (utf8_getLenCode(p[0])) {
			case 1:
				return p[0];
			case 2:
				if (!UTF8_CONTINUATION(p[1])) break;
				return ((p[0] & 0x1F) << 6) | (p[1] & 0x3F);
			case 3:
				if (!UTF8_CONTINUATION(p[1]) || !UTF8_CONTINUATION(p[2])) break;
				return ((p[0] & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);
			case 4:
				if (!UTF8_CONTINUATION(p[1]) || !UTF8_CONTINUATION(p[2]) || !UTF8_CONTINUATION(p[3])) break;
				return ((p[0] & 0x07) << 18) | ((p[1] & 0x3F) << 12) | ((p[2] & 0x3F) << 6) | (p[3] & 0x3F);
		}
	}
	return 0;
}

/**
 * Write a uncode character / codepoint to a buffer using UTF-8 Coding.
 * If the buffer is not supplied, only the length of the coding is returned
 * (the same you get by calling utf8_bytelen() above).
 *
 * Remember: no terminating null byte is written to the buffer!
 *
 * ATTENTION: if a buffer is supplied it should be big enough to hold the
 * resulting coding. You could either specify an at least 4 byte long buffer
 * or first check for the resulting length for the given codepoint.
 *
 * \param codepoint		the codepoint to write to the buffer as UTF-8 encodeing
 * \param buf			the buffer where to put the resulting UTF-8 bytes (if specified)
 * \return				number of bytes written to the buffer
 * \see					utf8_bytelen()
 */
int utf8_writeBuffer (uint32_t codepoint, void *buf)
{
	uint8_t *p;
	int len;

	len = utf8_bytelen(codepoint);
	if ((p = (uint8_t *) buf) != NULL && len > 0) {
		switch (len) {
			case 1:
				*p++ = codepoint & 0x7F;
				break;
			case 2:
				*p++ = 0b11000000 | ((codepoint >> 6) & 0x1F);
				*p++ = 0b10000000 | ((codepoint >> 0) & 0x3F);
				break;
			case 3:
				*p++ = 0b11100000 | ((codepoint >> 12) & 0x0F);
				*p++ = 0b10000000 | ((codepoint >> 6) & 0x3F);
				*p++ = 0b10000000 | ((codepoint >> 0) & 0x3F);
				break;
			case 4:
				*p++ = 0b11110000 | ((codepoint >> 18) & 0x07);
				*p++ = 0b10000000 | ((codepoint >> 12) & 0x3F);
				*p++ = 0b10000000 | ((codepoint >> 6) & 0x3F);
				*p++ = 0b10000000 | ((codepoint >> 0) & 0x3F);
				break;
		}
	}
	return len;
}

/**
 * Scan a string of UTF-8 characters to find the beginning of the next
 * valid character.
 *
 * \param s			the string pointer to the current character to start searching from
 * \return			the beginning of the next character, the position of the terminating null
 * 					byte or NULL, if a NULL-pointer was specified when calling the function
 */
char *utf8_advance (char *s)
{
	if (s && *s) {
		s++;
		while (*s && (*s & 0x80) && ((*s & 0xC0) != 0xC0)) s++;
	}
	return s;
}

/**
 * Copy string src to buffer dst with at most maxbytes bytes. In contrast to the
 * classical strncpy(), the destination will always be null terminated and the copy
 * function stops copying in a ways, that UTF-8 chars are not cut off in the middle
 * of a sequence. The len parameter specifies the size of src in bytes. This allows
 * us to copy UTF-8 strings from information structures that allow names to use the
 * maximum space available without supplying a terminating null byte.
 *
 * Additionally, the function cleans out illegal UTF-8 sequences, but won't check
 * the unicode codepoints to lie in the valid range (0x000000 .. 0x10FFFF).
 *
 *
 * \param dst		the destination buffer where the resulting string including it's null byte is stored
 * \param maxbytes	the number of bytes available in the destination buffer
 * \param src		the soruce string that may be but needs not be terminated with a null byte
 * \param len		the maximum length in bytes that are available in the source string
 */
char *utf8_strncpy (char *dst, size_t maxbytes, const char *src, size_t len)
{
	char *d;
	uint32_t codepoint;
	size_t l;

	if (!dst || !maxbytes) return NULL;		// nothing to copy to or no length specified
	*dst = 0;								// at least we should write a null byte to the destination buffer
	if (!src || !len) return dst;			// nothing to copy from or no length specified

	d = dst;
	while (len > 0) {
		l = utf8_getLenCode(*src);
		if (l > len) break;						// the length of the current sequence reaches beyond the source and is therefor ignored
		codepoint = utf8_codepoint(src);
		if (codepoint == 0) break;				// the character was either illegal or really a termination null byte
		if ((maxbytes - (d - dst)) <= l) break;	// the sequence won't fit into the destination - so stop copying further
		// OK - we may now copy the character (composed of l bytes) copy to the destination
		utf8_writeBuffer(codepoint, d);
		d += l;
		src +=l;
		len -= l;
	}
	*d = 0;
	return dst;
}

/**
 * Translate an ISO8859-1 String to UTF-8 encoding.
 * Since ISO8859-1 uses encodings, that map 1:1 to unicode codepoints, we simply
 * need to convert all characters >= 0x80 to it's codepoint representation.
 *
 * This is always a combination of 0xC2 or 0xC3 plus a second byte 0x80 - 0xBF.
 * No table is used - the conversion is simply done in an arithmetic fashion.
 *
 * \param s			the ISO8859-1 source string (null terminated
 * \return			a temporary string ("allocated" with tmpbuf()) that holds the UTF-8 result.
 */
char *utf8_iso2utf (char *s)
{
	char *d, *tmp;

	if (!s) return s;
	d = tmp = tmpbuf(strlen(s) * 2 + 1);
	while (*s) {
		if (*s < 0x80) {
			*d++ = *s++;
		} else if (*s < 0xC0) {
			*d++ = 0xC2;
			*d++ = (*s++ & 0x3F) | 0x80;
		} else {
			*d++ = 0xC3;
			*d++ = (*s++ & 0x3F) | 0x80;
		}
	}
	*d = 0;
	return tmp;
}
