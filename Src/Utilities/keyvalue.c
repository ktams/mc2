/**
 * @file keyvalue.c
 *
 * @author Andi
 * @date   15.01.2020
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

#include <stdlib.h>
#include <string.h>
#include "rb2.h"

/**
 * Build up a memory containing the two strings (key and value) in one contiguous
 * memory block. The key is copied right behind the struct and the value is appended
 * right behind the copied key.
 *
 * Note 1:	the key does not contain a ':' or '=' that should be supplied for example in the http headers<br>
 * Note 2:	the whole structure including it's strings is freed by a singe call to
 * 			free() with the pointer to the structure.
 *
 * \param kv		if supplied, the newly generated header is placed as the next-member
 * 					of this supplied header, while the original next-field is placed
 * 					in the allocated structure. Thus, the newly created struct is inserted
 * 					between the given hdr-struct and it's original next member which means,
 * 					it is simply appended to 'kv'.
 * \param key		the string representing the key of a header field, must not be NULL
 * \param key_len	the length of the key string (the key needs not to be null terminated)
 * \param value		the string representing the value for the key, must not be NULL
 * \param val_len	the length of the value string (the value needs not to be null terminated)
 * \return			a pointer to an allocated structure holding this key/value pair
 * 					or a NULL pointer if something went wrong (missing parameters, or
 * 					out of memory)
 */
struct key_value *kv_addEx (struct key_value *kv, const char *key, int key_len, const char *value, int val_len)
{
	struct key_value *h;
	int len;

	if (!key || !*key || key_len <= 0) return NULL;
	if (!value || !*value) val_len = 0;
	if (val_len < 0) val_len = 0;
	len = sizeof(*h) + key_len + val_len + 2;	// allow for two null characters to terminate the strings

	if ((h = malloc(len)) == NULL) {
		return NULL;
	}

	h->next = NULL;
	h->value = h->key + key_len + 1;
	strncpy (h->key, key, key_len);
	h->key[key_len] = 0;
	if (val_len > 0) strncpy (h->value, value, val_len);
	h->value[val_len] = 0;
	h->idx = 0;
	h->indexed = false;

	if (kv) {
		h->next = kv->next;
		kv->next = h;
	}
	return h;
}

/**
 * Build up a memory containing the two strings (key and value) in one contiguous
 * memory block. The key is copied right behind the struct and the value is appended
 * right behind the copied key.
 *
 * This version expects two null terminated strings and calls \ref kv_addEx() with
 * the strlen() of the supplied parameters.
 *
 * Note 1:	the key does not contain a ':' or '=' that should be for example be supplied in the http headers<br>
 * Note 2:	the whole structure including it's strings is freed by a singe call to
 * 			free() with the pointer to the structure.
 *
 * \param kv		if supplied, the newly generated header is placed as the next-member
 * 					of this supplied header, while the original next-field is placed
 * 					in the allocated structure. Thus, the newly created struct is inserted
 * 					between the given hdr-struct and it's original next member which means,
 * 					it is simply appended to 'kv'.
 * \param key		the null terminated string representing the key of a header field, must not be NULL
 * \param value		the null terminated string representing the value for the key, must not be NULL
 * \return			a pointer to an allocated structure holding this key/value pair
 * 					or a NULL pointer if something went wrong (missing parameters, or
 * 					out of memory)
 * \see				kv_addEx()
 */
struct key_value *kv_add (struct key_value *kv, const char *key, const char *value)
{
	return kv_addEx(kv, key, strlen(key), value, strlen(value));
}

/**
 * Build up a memory containing the two strings (key and value) in one contiguous
 * memory block. The key is copied right behind the struct and the value is appended
 * right behind the copied key.
 *
 * This version expects two null terminated strings and calls \ref kv_add() which
 * in turn calls \ref kv_addEx() with the strlen() of the supplied parameters. It
 * then sets the index and marks this KV as beeing indexed.
 * Indexed KVs are only used for INI files.
 *
 * Note 1:	the key does not contain a ':' or '=' that should be for example be supplied in the http headers<br>
 * Note 2:	the whole structure including it's strings is freed by a singe call to
 * 			free() with the pointer to the structure.
 *
 * \param kv		if supplied, the newly generated header is placed as the next-member
 * 					of this supplied header, while the original next-field is placed
 * 					in the allocated structure. Thus, the newly created struct is inserted
 * 					between the given hdr-struct and it's original next member which means,
 * 					it is simply appended to 'kv'.
 * \param key		the null terminated string representing the key of a header field, must not be NULL
 * \param idx		the index that this key should have (i.e. key="icon" + idx=5 should write "icon(5) = ..." to INI file)
 * \param value		the null terminated string representing the value for the key, must not be NULL
 * \return			a pointer to an allocated structure holding this key/value pair
 * 					or a NULL pointer if something went wrong (missing parameters, or
 * 					out of memory)
 * \see				kv_add()
 */
struct key_value *kv_addIndexed (struct key_value *kv, const char *key, int idx, const char *value)
{
	if ((kv = kv_add(kv, key, value)) != NULL) {
		kv->idx = idx;
		kv->indexed = true;
	}

	return kv;
}

/**
 * Free a complete linked list of struct key_value.
 *
 * \param kv	pointer to the first struct in the list
 */
void kv_free (struct key_value *kv)
{
	struct key_value *h;

	while ((h = kv) != NULL) {
		kv = kv->next;
		free (h);
	}
}

/**
 * Look up a certain key in the list of key-value pairs and return the found structure.
 * The search is done in a case insensitive manner, i.e. a search for "cmd" will also find
 * a key named "cMd".
 *
 * In general, the first full match is returned. If more than one match would fit, only
 * the first one is returned. Further entries can be found by calling this function again
 * with the next-pointer of the found entry as kv-parameter.
 *
 * \param kv	the list of key-value pairs to search for keys
 * \param key	the key that should be looked up
 * \return		a pointer to the found entry in the list or NULL, if no match could be found
 */
struct key_value *kv_lookup (struct key_value *kv, char *key)
{
	if (!key) return NULL;

	while (kv) {
		if (!strcasecmp(key, kv->key)) return kv;
		kv = kv->next;
	}
	return NULL;
}

/**
 * Copy the value string of a key-value pair to a given buffer taken the length of
 * the target buffer into account. This is much like strncpy(), but takes care, that
 * the target buffer will be null terminated.
 *
 * \param kv		the key-value pair where we need to take the value from
 * \param dest		the target buffer to copy the value string to (and terminate it with a null byte)
 * \param maxlen	the length of the target buffer including the room for the null byte
 * \return			the buf argument (same semantics as the strncpy() function) or NULL if something went wrong
 */
char *kv_strcpy (struct key_value *kv, char *dest, size_t maxlen)
{
	if (!kv || !dest || maxlen <= 0) return NULL;
	if (kv->value) {
		strncpy (dest, kv->value, maxlen);
		dest[maxlen - 1] = 0;
	} else {
		*dest = 0;
	}
	return dest;
}
