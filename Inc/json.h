/*
 * json.h
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

#ifndef __JSON_H__
#define __JSON_H__

/**
 * \ingroup HTTPD
 * \{
 */

enum jtype {
	JSON_OBJECT,				///< an "OBJECT" that can contain JSON variables
	JSON_ARRAY,					///< an array of values of any kind
	JSON_STRING,				///< a JSON string value
	JSON_INTEGER,				///< a JSON number value for integral numbers
	JSON_UNSIGNED,				///< a JSON number value for unsigned integral numbers
	JSON_FLOAT,					///< a JSON number value for double precision floating point numbers (ATTENTION: unused!)
	JSON_TRUE,					///< the expression 'true'
	JSON_FALSE,					///< the expression 'false'
	JSON_NULL,					///< the expression 'null'
};

typedef struct json_value	json_valT;
typedef struct json_item	json_itmT;
typedef struct json_stack	json_stackT;

struct json_item {
	json_itmT			*next;		///< linked list of items in a JSON_OBJECT
	json_valT			*value;		///< any type of value that is possible within JSON
	char				 name[];	///< the json string containing the name of this variable
};

struct json_value {
	json_valT			*next;		///< linked list of objects of any kind (root object must set this member to NULL)
	enum jtype			 type;		///< the type of this object. JSON_TRUE, JSON_FALSE and JSON_NULL need no further value as the type already stands for it
	union {
		json_itmT			*itm;		///< JSON_OBJECT: objects contain a list of items
		json_valT			*array;		///< JSON_ARRAY: arrays contain a list of values
		int					 intval;	///< JSON_INTEGER: value for integral numbers - json handles these as generic NUMERIC values
		unsigned			 uintval;	///< JSON_UNSIGNED: value for integral numbers - json handles these as generic NUMERIC values
//		double				 floatval;	///< JSON_FLOAT: value for float numbers - json handles these as generic NUMERIC values (ATTENTION: unused!)
	};
	char				 string[];	/**< JSON_STRING: a string (already escaped for string representation) - the characters should
										 already be allocated right behind the structure space as a single block to ease memory handling.
										 As this is a flexible member, no space is used if this value is something other than a JSON_STRING. */
};

struct json_stack {
	json_stackT			*next;		///< the stack is a linked list
	union {
		json_valT			**val;		///< here we can add values (used with JSON_ARRAY)
		json_itmT			**itm;		///< here we can add items (used with JSON_OBJECT)
	};
};

#define json_pushArrayValue(stack, ar)	json_pushObject (stack, ar)

/*
 * Prototypes WEB/json.c
 */
json_stackT *json_pop (json_stackT *stack);
void json_popAll (json_stackT *stack);
json_stackT *json_pushArray (json_stackT *stack, json_itmT *ar);
json_stackT *json_pushObject (json_stackT *stack, json_valT *obj);
json_itmT *json_addItem (json_stackT *stack, const char *item_name);
json_valT *json_addObject (json_stackT *stack);
json_valT *json_addArray (json_stackT *stack);
json_valT *json_addStringValue (json_stackT *stack, const char *s);
json_valT *json_addFormatStringValue (json_stackT *stack, const char *fmt, ...) __attribute((format(printf, 2, 3)));
json_valT *json_addIntValue (json_stackT *stack, int n);
json_valT *json_addUintValue (json_stackT *stack, unsigned n);
json_valT *json_addTrue (json_stackT *stack);
json_valT *json_addFalse (json_stackT *stack);
json_valT *json_addNull (json_stackT *stack);
json_itmT *json_addArrayItem (json_stackT *stack, const char *item);
json_itmT *json_addIntItem (json_stackT *stack, const char *item, int n);
json_itmT *json_addUintItem (json_stackT *stack, const char *item, unsigned n);
json_itmT *json_addStringItem (json_stackT *stack, const char *item, const char *s);
json_itmT *json_addFormatStringItem (json_stackT *stack, const char *item, const char *fmt, ...) __attribute((format(printf, 3, 4)));
void json_free (json_valT *root);
void json_debug (json_valT *root);

/**
 * @}
 */
#endif /* __JSON_H__ */
