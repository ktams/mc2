/*
 * json.c
 *
 *  Created on: 17.11.2020
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

/**
 * \ingroup HTTPD
 * \{
 *
 * \file json.c Handling of JSON objects for the WEB interface
 *
 * Answers to WEB-Queries and event reporting to browsers are formatted as
 * JSON objects. This fairly simple text based format allows a flexible way
 * to define even complex data structures which can be handled in inside the
 * Browser using JavaScript.
 *
 * In short, there are only two types of 'things' that can be used in a JSON
 * structure: *Items* and *values*. The following value object types are defined:
 *   - \ref JSON_OBJECT a container to hold *items*, comparable to a C struct
 *   - \ref JSON_ARRAY a container to hold a list of other *values*
 *   - \ref JSON_STRING an escaped string that contains UTF-8 characters
 *   - \ref JSON_INTEGER integral numbers (officially, JSON only knows about *number*, which maybe integral or float)
 *   - \ref JSON_FLOAT float numbers (officially, JSON only knows about *number*, which maybe integral or float)
 *   - \ref JSON_TRUE the expression / special value 'true'
 *   - \ref JSON_FALSE the expression / special value 'false'
 *   - \ref JSON_NULL the expression / special value 'null'
 *
 * Items are simply the combination of a name and a value, while a value is the bare
 * representation of various data types - one data at a time. The general rule is,
 * that a value of type **object** may only contain a list of *items*, an **array**
 * may only contain a list of *values* and the other value types stand for a single
 * value of the respective type.
 *
 * JSON objects for the WEB interface are generally composed of an outer \ref JSON_OBJECT
 * (represented as "{ ... }" in the data stream) and so may contain any number of *items*
 * that need to be sent to the browser.
 *
 * Keep in mind, that anywhere, where a value is allowed, any of the complex value
 * types \ref JSON_OBJECT or \ref JSON_ARRAY may be used! And: *items* may only be
 * added as subobjects of a \ref JSON_OBJECT.
 *
 * ---
 */

#include <stdio.h>
#include <stdarg.h>
#include "rb2.h"
#include "json.h"

static const char hex[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

static const char *json_type (enum jtype tp)
{
	switch (tp) {
		case JSON_OBJECT: return xstr(JSON_OBJECT);
		case JSON_ARRAY: return xstr(JSON_ARRAY);
		case JSON_STRING: return xstr(JSON_STRING);
		case JSON_INTEGER: return xstr(JSON_INTEGER);
		case JSON_UNSIGNED: return xstr(JSON_UNSIGNED);
		case JSON_FLOAT: return xstr(JSON_FLOAT);
		case JSON_TRUE: return xstr(JSON_TRUE);
		case JSON_FALSE: return xstr(JSON_FALSE);
		case JSON_NULL: return xstr(JSON_NULL);
		default: return "(unknown)";
	}
}

/**
 * Calculate the length of a string that holds the given C-string in it's escaped
 * form. The length does not include the terminating null byte, to be "compatible"
 * with the standard C-function strlen().
 *
 * \param s		the original string that should be escaped
 * \return		the length of the resulting escaped string not including the terminating null byte
 */
static size_t json_stringLength (const char *s)
{
	size_t len = 0;

	while (*s) {
		switch (*s) {
			case '"':
			case '\\':
			case '/':
			case '\b':
			case '\f':
			case '\n':
			case '\r':
			case '\t':
				len += 2;
				break;
			default:
				if (*s < ' ') len += 6;		// coded as "\uxxxx" (4 HEX digits)
				else len++;
		}
		s++;
	}
	return len;
}

/**
 * Escapes a string for representation as JSON string. All disallowed characters
 * are escaped either using the short form (i.e. two characters "\n" for a newline)
 * or coded as 4 hexadecimal digits. A JSON string usually consists of UTF-8
 * chars and so no escaping should be necessary for character codes > 0x7F or 0xFF.
 * Only control codes (below SPACE character, 32 or 0x20) must be escaped using the
 * hexadecimal notation.
 *
 * The given taget buffer must be long enougth to hold the escaped string representation
 * including a terminating null character.
 *
 * \param buf	the target buffer where the resulting string is written to
 * \param s		the C-string that needs to be converted
 * \return		the given target buffer containing the C-string with all necessary escape sequences
 */
static char *json_escapeString (char *buf, const char *s)
{
	char *p;

	if (!buf) return NULL;
	p = buf;
	while (*s) {
		switch (*s) {
			case '"':
				*p++ = '\\';
				*p++ = '"';
				break;
			case '\\':
				*p++ = '\\';
				*p++ = '\\';
				break;
			case '/':
				*p++ = '\\';
				*p++ = '/';
				break;
			case '\b':
				*p++ = '\\';
				*p++ = 'b';
				break;
			case '\f':
				*p++ = '\\';
				*p++ = 'f';
				break;
			case '\n':
				*p++ = '\\';
				*p++ = 'n';
				break;
			case '\r':
				*p++ = '\\';
				*p++ = 'r';
				break;
			case '\t':
				*p++ = '\\';
				*p++ = 't';
				break;
			default:
				if (*s < ' ') {		// coded as "\uxxxx" (4 HEX digits)
					*p++ = '\\';
					*p++ = 'u';
					*p++ = hex[(*s >> 12) & 0xF];
					*p++ = hex[(*s >>  8) & 0xF];
					*p++ = hex[(*s >>  4) & 0xF];
					*p++ = hex[(*s >>  0) & 0xF];
				} else {
					*p++ = *s;
				}
		}
		s++;
	}
	*p = 0;
	return buf;
}

/**
 * Remove the topmost stack item, free it's allocated memory and return the
 * remaining Stack to the caller.
 *
 * \param stack		the current stack (pointer to the top-most element)
 * \return			the new stack with top-most item removed, may result in an empty stack (NULL)
 */
json_stackT *json_pop (json_stackT *stack)
{
	json_stackT *p;

	if ((p = stack) != NULL) {
		stack = stack->next;
		free (p);
	}
	return stack;
}

/**
 * Just pop all the stack items to free the allocated memory
 *
 * \param stack		the current stack (pointer to the top-most element)
 */
void json_popAll (json_stackT *stack)
{
	while (stack) stack = json_pop(stack);
}

/**
 * Push a JSON_ARRAY on the stack. Only values should be added to this stack level
 *
 * \param stack		the current stack (pointer to the top-most element or NULL, if creating a new stack)
 * \return			the new stack with top-most item pointing to the value pointer of the array item
 */
json_stackT *json_pushArray (json_stackT *stack, json_itmT *ar)
{
	json_stackT *p;

	if (ar && ar->value && ar->value->type == JSON_ARRAY) {
		if ((p = malloc (sizeof(*p))) != NULL) {
			p->next = stack;
			p->val = &ar->value->array;
			stack = p;
//		} else {
//			json_popAll (stack);
		}
	}
	return stack;
}

/**
 * Push a JSON_OBJECT on the stack. Only items should be added to this stack level
 *
 * \param stack		the current stack (pointer to the top-most element or NULL, if creating a new stack)
 * \return			the new stack with top-most item pointing to the item pointer of the object
 */
json_stackT *json_pushObject (json_stackT *stack, json_valT *obj)
{
	json_stackT *p;

	if (obj && (obj->type == JSON_OBJECT || obj->type == JSON_ARRAY)) {
		if ((p = malloc (sizeof(*p))) != NULL) {
			p->next = stack;
			p->itm = &obj->itm;
			stack = p;
//		} else {
//			json_popAll (stack);
		}
	}
	return stack;
}

/**
 * Add a JSON *item* to a list of items.
 *
 * An item consists of a name and a reference to a value. The value
 * itself must be set / added with one of the json_addXXX() functions.
 * The item is allocated as a single block of memory containing both the
 * structure and the JSON-escaped name of the item in the flexible member
 * at the very end of the structure (see \ref struct json_item).
 *
 * The 'stack' parameter may be NULL if you want to start a new list of items.
 * In our case, this makes no sense at all, but a non-NULL value is not enforced
 * in any way.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this item
 * \param item_name	the name of this item in it's unescaped form (must not be NULL or an empty string)
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 */
json_itmT *json_addItem (json_stackT *stack, const char *item_name)
{
	json_itmT *p;
	size_t len;

	if (item_name == NULL || !*item_name) return NULL;
	len = json_stringLength(item_name) + 1;					// add room for the terminating null byte
	if ((p = calloc (1, sizeof(*p) + len)) == NULL) return NULL;

	json_escapeString(p->name, item_name);
	if (stack) {
		*stack->itm = p;
		stack->itm = &p->next;
	}
	return p;
}

/**
 * Add a JSON *value* to a list of values.
 *
 * This is a utility function which is called by one of the json_addXXX() functions.
 * The value is allocated as a single block of memory containing the structure and the
 * real value, even if that value is of type JSON_STRING. In that latter case, the
 * required memory for the string is appended to the very end of the structure by
 * allocating memory for the flexible member (see \ref struct json_value).
 *
 * The 'append' parameter may be NULL if you want to start a new list of values.
 * In our case, this makes only sense for the first JSON_OBJECT, but may be used
 * anywhere.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param tp		the type of the value, in case of JSON_STRING with a NULL pointer 's', this is automatically changed to JSON_NULL
 * \param s			in case of a string value, this is the string in it's unescaoed form
 * \return			a pointer to the allocated value structure on success or NULL if allocation failed
 */
static json_valT *json_addValue (json_stackT *stack, enum jtype tp, const char *s)
{
	json_valT *p;
	size_t len;

	if (tp == JSON_STRING && s == NULL) tp = JSON_NULL;		// change NULL strings (not empty strings!) to JSON null object
	if (tp == JSON_STRING) {
		len = json_stringLength(s) + 1;						// add room for the terminating null byte
	} else {
		len = 0;
	}
	if ((p = calloc (1, sizeof(*p) + len)) == NULL) return NULL;
	p->type = tp;
	if (tp == JSON_STRING) json_escapeString(p->string, s);
	if (stack) {
		*stack->val = p;
		stack->val = &p->next;
	}
	return p;
}

/**
 * Add a JSON_OBJECT to a list of values or start a new JSON structure.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \return			a pointer to the allocated value structure on success or NULL if allocation failed
 * \see				json_addValue()
 */
json_valT *json_addObject (json_stackT *stack)
{
	return json_addValue(stack, JSON_OBJECT, NULL);
}

/**
 * Add a JSON_ARRAY to a list of values.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \return			a pointer to the allocated value structure on success or NULL if allocation failed
 * \see				json_addValue()
 */
json_valT *json_addArray (json_stackT *stack)
{
	return json_addValue(stack, JSON_ARRAY, NULL);
}

/**
 * Add a JSON_STRING to a list of values.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param s			the string that should be represented in this value (still unescaped)
 * \return			a pointer to the allocated value structure on success or NULL if allocation failed
 * \see				json_addValue()
 */
json_valT *json_addStringValue (json_stackT *stack, const char *s)
{
	return json_addValue(stack, JSON_STRING, s);
}

/**
 * Create a JSON_STRING value by first creating a printf-formatted string.
 *
 * A temporary string of 256 characters is used to do the formatting. The
 * string is "allocated" by tmp256().
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param fmt		the format string that should be used to create the final string
 * \return			a pointer to the allocated value structure on success or NULL if allocation failed
 * \see				json_addStringValue()
 * \see				tmp256()
 */
json_valT *json_addFormatStringValue (json_stackT *stack, const char *fmt, ...)
{
	char *tmp = tmp256();
	va_list ap;

	if (!fmt) return NULL;
	va_start (ap, fmt);
	vsnprintf(tmp, 256, fmt, ap);
	va_end (ap);
	return json_addStringValue(stack, tmp);
}

/**
 * Add a JSON_INTEGER to a list of values.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param n			the integer value to be represented by this value
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addValue()
 */
json_valT *json_addIntValue (json_stackT *stack, int n)
{
	json_valT *p;

	if ((p = json_addValue(stack, JSON_INTEGER, NULL)) != NULL) p->intval = n;
	return p;
}

/**
 * Add a JSON_UNSIGNED to a list of values.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param n			the unsigned integer value to be represented by this value
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addValue()
 */
json_valT *json_addUintValue (json_stackT *stack, unsigned n)
{
	json_valT *p;

	if ((p = json_addValue(stack, JSON_UNSIGNED, NULL)) != NULL) p->uintval = n;
	return p;
}

/**
 * Add a JSON_TRUE to a list of values.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addValue()
 */
json_valT *json_addTrue (json_stackT *stack)
{
	return json_addValue(stack, JSON_TRUE, NULL);
}

/**
 * Add a JSON_FALSE to a list of values.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addValue()
 */
json_valT *json_addFalse (json_stackT *stack)
{
	return json_addValue(stack, JSON_FALSE, NULL);
}

/**
 * Add a JSON_NULL to a list of values.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addValue()
 */
json_valT *json_addNull (json_stackT *stack)
{
	return json_addValue(stack, JSON_NULL, NULL);
}

/**
 * Combination of adding an *item* and than set the value of that item
 * to an array.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param item		the name of this item in it's unescaped form (must not be NULL or an empty string)
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addItem()
 * \see				json_addIntValue()
 */
json_itmT *json_addArrayItem (json_stackT *stack, const char *item)
{
	json_itmT *itm;

	if ((itm = json_addItem(stack, item)) != NULL) {
		itm->value = json_addArray(NULL);
	}
	return itm;
}

/**
 * Combination of adding an *item* and than set the value of that item
 * to an integer value.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param item		the name of this item in it's unescaped form (must not be NULL or an empty string)
 * \param n			the integer values that should be set for this item
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addItem()
 * \see				json_addIntValue()
 */
json_itmT *json_addIntItem (json_stackT *stack, const char *item, int n)
{
	json_itmT *itm;

	if ((itm = json_addItem(stack, item)) != NULL) {
		itm->value = json_addIntValue(NULL, n);
	}
	return itm;
}

/**
 * Combination of adding an *item* and than set the value of that item
 * to an unsigned integer value.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param item		the name of this item in it's unescaped form (must not be NULL or an empty string)
 * \param n			the unsigned integer values that should be set for this item
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addItem()
 * \see				json_addUintValue()
 */
json_itmT *json_addUintItem (json_stackT *stack, const char *item, unsigned n)
{
	json_itmT *itm;

	if ((itm = json_addItem(stack, item)) != NULL) {
		itm->value = json_addUintValue(NULL, n);
	}
	return itm;
}

/**
 * Combination of adding an *item* and than set the value of that item
 * to a string value.
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param item		the name of this item in it's unescaped form (must not be NULL or an empty string)
 * \param s			the string that should be set as value for this item (unescaped for)
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addItem()
 * \see				json_addStringValue()
 */
json_itmT *json_addStringItem (json_stackT *stack, const char *item, const char *s)
{
	json_itmT *itm;

	if ((itm = json_addItem(stack, item)) != NULL) {
		itm->value = json_addStringValue(NULL, s);
	}
	return itm;
}

/**
 * Combination of adding an *item* and than set the value of that item
 * to a string value. The string value is built using a format string and
 * a variable number of parameters for that string.
 *
 * A temporary string of 256 characters is used to do the formatting. The
 * string is "allocated" by tmp256().
 *
 * \param stack		a pointer to the stack structure holding a pointer-pointer to where to add this value
 * \param item		the name of this item in it's unescaped form (must not be NULL or an empty string)
 * \param s			the string that should be set as value for this item (unescaped for)
 * \return			a pointer to the allocated item structure on success or NULL if allocation failed
 * \see				json_addItem()
 * \see				json_addStringValue()
 * \see				tmp256()
 */
json_itmT *json_addFormatStringItem (json_stackT *stack, const char *item, const char *fmt, ...)
{
	char *tmp = tmp256();
	va_list ap;

	if (!fmt) return NULL;
	va_start (ap, fmt);
	vsnprintf(tmp, 256, fmt, ap);
	va_end (ap);
	return json_addStringItem(stack, item, tmp);
}

static void json_freeItem (json_itmT *itm);
static void json_freeValue (json_valT *val);

static void json_freeItem (json_itmT *itm)
{
	json_itmT *p;

	while ((p = itm) != NULL) {
		itm = itm->next;
		json_freeValue(p->value);
		free (p);
	}
}

static void json_freeValue (json_valT *val)
{
	json_valT *p;

	while ((p = val) != NULL) {
		val = p->next;
		if (p->type == JSON_OBJECT) json_freeItem(p->itm);
		else if (p->type == JSON_ARRAY) json_freeValue(p->array);
		free(p);
	}
}

void json_free (json_valT *root)
{
	json_freeValue(root);
}

static void json_debugItem (json_itmT *itm, int indent);
static void json_debugValue (json_valT *val, int indent);

static void json_debugItem (json_itmT *itm, int indent)
{
	while (itm) {
		printf ("%*.*sITM '%s':\n", indent, indent, "", itm->name);
		json_debugValue(itm->value, indent + 2);
		itm = itm->next;
	}
}

static void json_debugValue (json_valT *val, int indent)
{
	while (val) {
		printf ("%*.*s%s", indent, indent, "", json_type(val->type));
		switch (val->type) {
			case JSON_OBJECT:
				putchar ('\n');
				json_debugItem(val->itm, indent + 2);
				break;
			case JSON_ARRAY:
				putchar ('\n');
				json_debugValue(val->array, indent + 2);
				break;
			case JSON_STRING:
				printf (" '%s'\n", val->string);
				break;
			case JSON_INTEGER:
				printf (" %d\n", val->intval);
				break;
			case JSON_UNSIGNED:
				printf (" %u\n", val->uintval);
				break;
			case JSON_FLOAT:
				printf (" (not implemented)\n");
				break;
			case JSON_TRUE:
			case JSON_FALSE:
			case JSON_NULL:
				putchar ('\n');
				break;
		}
		val = val->next;
	}
}

void json_debug (json_valT *root)
{
	json_debugValue(root, 3);
}

/**
 * \}
 */
