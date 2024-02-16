/**
 * @file ini.c
 *
 * @author Andi
 * @date   09.04.2020
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

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "rb2.h"
#include "yaffsfs.h"

#define MAX_LINE_LENGTH		256		///< the maximum length of a single line in the in file

/**
 * Build up a memory containing the struct data and the name string in one contiguous
 * memory block. The name forms the variable structure element.
 *
 * \param ini		if supplied, the pointer to the newly generated section is placed as
 * 					the next-member	of this supplied ini section, while the original next-field
 * 					is placed in the allocated structure. Thus, the newly created struct is
 * 					inserted between the given ini-struct and it's original next member
 * 					which means, it is simply appended to 'ini'.
 * \param name		the string representing the name of the ini section, must not be NULL
 * \param name_len	the length of the name string (the name needs not to be null terminated)
 * \return			a pointer to an allocated structure holding this ini section name and
 * 					the list of key/value pairs	or a NULL pointer if something went wrong
 * 					(missing parameters, or out of memory)
 */
struct ini_section *ini_addEx (struct ini_section *ini, const char *name, int name_len)
{
	struct ini_section *sec;
	int len;

	if (!name) return NULL;
	len = sizeof(*sec) + name_len + 1;	// allow for a null character to terminate the string

	if ((sec = malloc(len)) == NULL) {
		return NULL;
	}

	sec->next = NULL;
	sec->kv = NULL;
	strncpy (sec->name, name, name_len);
	sec->name[name_len] = 0;

	if (ini) {
		sec->next = ini->next;
		ini->next = sec;
	}
	return sec;
}

/**
 * Build up a memory containing the struct data and the name string in one contiguous
 * memory block. The name forms the variable structure element.
 *
 * This version expects a null terminated string and calls \ref ini_addEx() with
 * the strlen() of the supplied parameter.
 *
 * \param ini		if supplied, the pointer to the newly generated section is placed as
 * 					the next-member	of this supplied ini section, while the original next-field
 * 					is placed in the allocated structure. Thus, the newly created struct is
 * 					inserted between the given ini-struct and it's original next member.
 * \param name		the null terminated string representing the name of the ini section, must not be NULL
 * \return			a pointer to an allocated structure holding this ini section name and
 * 					the list of key/value pairs	or a NULL pointer if something went wrong
 * 					(missing parameters, or out of memory)
 */
struct ini_section *ini_add (struct ini_section *ini, const char *name)
{
	return ini_addEx(ini, name, strlen(name));
}

/**
 * Build up a memory containing the struct data and the name string in one contiguous
 * memory block. The name forms the variable structure element.
 *
 * The created ini-structure is then appenden to the end of the list given by the root
 * parameter.
 *
 * \param root		a pointer to the root pointer of the ini structure list to be created
 * \param name		the null terminated string representing the name of the ini section, must not be NULL
 */
struct ini_section *ini_addSection (struct ini_section **root, const char *name)
{
	struct ini_section *ini;

	if (!name) return NULL;
	ini = ini_add (NULL, name);
	if (root) {
		while (*root) root = &(*root)->next;
		*root = ini;
	}
	return ini;
}

/**
 * Create a key-value pair to add to an ini section. The created key-value structure is
 * appended to the list of key-value structures found in the supplied ini-section structure.
 * If no ini-section structure is given, only the key-value structure is created an returned.
 *
 * \param ini		the ini section structure where the key-value pair should be added (my be NULL)
 * \param name		the name or key of the key-value pair
 * \param value		the string value corresponding to the key parameter
 * \return			the created key value structure, that was added to the ini structure (if given)
 * 					or NULL, if an error occured
 */
struct key_value *ini_addItem (struct ini_section *ini, const char *name, const char *value)
{
	struct key_value *kv, **pp;

	if (!name) return NULL;
	kv = kv_add(NULL, name, value);
	if (ini) {
		pp = &ini->kv;
		while (*pp) pp = &(*pp)->next;
		*pp = kv;
	}
	return kv;
}

/**
 * Create a key-value pair to add to an ini section. The created key-value structure is
 * appended to the list of key-value structures found in the supplied ini-section structure.
 * If no ini-section structure is given, only the key-value structure is created an returned.
 * This version of the function takes an integer argument which is first converted to a string.
 *
 * \param ini		the ini section structure where the key-value pair should be added (my be NULL)
 * \param name		the name or key of the key-value pair
 * \param value		the integer value corresponding to the key parameter
 * \return			the created key value structure, that was added to the ini structure (if given)
 * 					or NULL, if an error occured
 * \see				ini_addItem()
 */
struct key_value *ini_addIntItem (struct ini_section *ini, const char *name, int value)
{
	char *tmp = tmp64();

	sprintf (tmp, "%d", value);
	return ini_addItem(ini, name, tmp);
}

/**
 * Create a key-value pair to add to an ini section. The created key-value structure is
 * appended to the list of key-value structures found in the supplied ini-section structure.
 * If no ini-section structure is given, only the key-value structure is created an returned.
 * This version of the function takes a boolean argument which is first converted to a string.
 *
 * \param ini		the ini section structure where the key-value pair should be added (my be NULL)
 * \param name		the name or key of the key-value pair
 * \param value		the integer value corresponding to the key parameter
 * \return			the created key value structure, that was added to the ini structure (if given)
 * 					or NULL, if an error occured
 * \see				ini_addItem()
 */
struct key_value *ini_addBoolItem (struct ini_section *ini, const char *name, bool value)
{
	return ini_addItem(ini, name, (value) ? "Y" : "N");
}

/**
 * Free a complete linked list of struct ini_section.
 *
 * \param ini	pointer to the first struct in the list
 */
void ini_free (struct ini_section *ini)
{
	struct ini_section *i;

	while ((i = ini) != NULL) {
		ini = ini->next;
		kv_free(i->kv);
		free (i);
	}
}

/**
 * Parse an already opened file from its current position and put the read structures
 * to the ini section root pointer. This contents of the pointer should be initialized
 * with either NULL or a real list of other ini section structures (\see ini_addEx()).
 *
 * \param fd		the file descriptor to read from
 * \param root		the pointer to a ini section root. This may simply be a pointer to base
 * 					pointer instead of a full struct, because the only element that is used
 * 					is the next pointer which will be the first item in the structure.
 * \return			0 for success or an error code otherwise
 */
struct ini_section *ini_parseFile (FILE *fp)
{
	struct ini_section *ini, *root;
	struct key_value *kv;
	char buf[MAX_LINE_LENGTH];
	char *s, *sec_name, *equal_sign, *index;
	char *key, *value;
	bool empty;
	int line = 0;

	if (fp == NULL) return NULL;

	root = ini = NULL;
	kv = NULL;

	while (fgets(buf, MAX_LINE_LENGTH, fp) != NULL) {
		line++;
		s = buf;
		empty = true;
		sec_name = equal_sign = index = NULL;
		while (*s && *s != '#') {
			if (empty && !isspace(*s)) empty = false;
			if (*s == '=' && !equal_sign) equal_sign = s;
			if (*s == '[' && !sec_name) sec_name = s;
			if (*s == '(' && !index) index = s;
			s++;
		}
		if (empty) continue;		// no useful information in this line, read next one
		if (*s == '#') *s = 0;		// drop the rest of the line - it is a comment

		if (sec_name) {				// we have found an opening bracket, lets see, if we can read a section name
			s = sec_name + 1;
			while (*s && isspace(*s)) s++;
			if (*s && *s != ']') {
				sec_name = s;
				while (*s && !isspace(*s) && *s != ']') s++;
				if (!*s) {			// no matching closing bracket found!
					log_msg (LOG_WARNING, "%s() line %d: no closing bracket up to the end of the line in '%s'\n", __func__, line, buf);
					continue;
				}
				if (isspace(*s)) *s++ = 0;	// null-terminate this section name
				while (*s && *s != ']') s++;
				if (*s != ']') {	// no matching closing bracket found!
					log_msg (LOG_WARNING, "%s() line %d: no closing bracket after white space '%s'\n", __func__, line, buf);
					continue;
				}
				*s = 0;
				ini = ini_add(ini, sec_name);
				if (!root) root = ini;
				kv = NULL;
			}
		} else if (equal_sign) {	// there was an equal sign like in "key = value" - interpret this line
			if (!ini) {
				log_msg (LOG_WARNING, "%s() line %d: key-value found outside section\n", __func__, line);
				continue;
			}
			if (index && index > equal_sign) index = NULL;	// this opening parenthesis belongs to the value!
			key = value = NULL;
			s = buf;
			while (*s && isspace(*s) && s < equal_sign) s++;
			if (s == equal_sign || s == index) {
				log_msg (LOG_WARNING, "%s() line %d: no valid key found in '%s'", __func__, line, buf);
				continue;
			}
			key = s;
			s = equal_sign;			// step backwards from equal-sign
			if (index) s = index;	// instead of equal-sign, step backwards from left parenthesis
			while (s > key && isspace(s[-1])) s--;
			*s = 0;		// terminate the key, probably replacing the '=' or '('
			s = equal_sign + 1;
			while (*s && isspace(*s)) s++;
			if (!*s) {
				log_msg (LOG_WARNING, "%s() line %d: no value for key '%s'\n", __func__, line, key);
				continue;
			}
			value = s;
			while (*s) s++;		// go to end of the line
			while (s > value && isspace(s[-1])) s--;
			*s = 0;
			if (!key || *key == 0 || !value || *value == 0) {
				log_msg (LOG_WARNING, "%s() line %d: either key or value are null strings\n", __func__, line);
				continue;
			}
			if (index) {
				kv = kv_addIndexed(kv, key, atoi(index + 1), value);
			} else {
				kv = kv_add(kv, key, value);
			}
			if (!ini->kv) ini->kv = kv;
		} else {		// this is an empty 'variable' like "user\n"
			s += strlen(s);
			while (s > buf && isspace(s[-1])) s--;
			*s = 0;
			log_msg (LOG_WARNING, "%s() line %d: '%s' is not a section header nor an assignment - ignored\n", __func__, line, buf);
		}
	}

	return root;
}

#if 1
static void ini_printStruct (struct ini_section *root) __attribute__((unused));
static void ini_printStruct (struct ini_section *root)
{
	struct key_value *kv;

	while (root) {
		printf ("   [%s]\n", root->name);
		kv = root->kv;
		while (kv) {
			if (kv->indexed) {
				printf ("      %s(%d) = \"%s\"\n", kv->key, kv->idx, kv->value);
			} else {
				if (kv->value) {
					printf ("      %s = \"%s\"\n", kv->key, kv->value);
				} else {
					printf ("      %s\n", kv->key);
				}
			}
			kv = kv->next;
		}
		root = root->next;
	}
}
#endif

struct ini_section *ini_readFile (const char *fname)
{
	FILE *fp;
	struct ini_section *root;

	if ((fp = fopen (fname, "r")) != NULL) {
		log_msg (LOG_INFO, "%s() '%s' opened successfully\n", __func__, fname);
		root = ini_parseFile(fp);
		fclose (fp);
//		ini_printStruct(root);
	} else {
		log_msg (LOG_WARNING, "%s(): cannot open '%s'\n", __func__, fname);
		root = NULL;
	}

	return root;
}

int ini_writeFile (const char *fname, struct ini_section *ini)
{
	FILE *fp;
	struct key_value *kv;

	if (!fname  || !*fname || !ini) return 1;	// non-fatal NOP (?)

	if ((fp = fopen (fname, "w")) == NULL) {
		log_error ("%s(): cannot open '%s'\n", __func__, fname);
		return -1;
	}
	log_msg (LOG_INFO, "%s() '%s' opened successfully\n", __func__, fname);
//	ini_printStruct(ini);

	while (ini) {
		fprintf(fp,  "[%s]\n", ini->name);
		kv = ini->kv;
		while (kv) {
			if (kv->value && *kv->value) {
				if (kv->indexed) {
					fprintf (fp, "%s(%d) = %s\n", kv->key, kv->idx, kv->value);
				} else {
					fprintf (fp, "%s = %s\n", kv->key, kv->value);
				}
			} else {
				fprintf (fp, "%s\n", kv->key);
			}
			kv = kv->next;
		}
		putc('\n', fp);
		ini = ini->next;
	}

	fclose (fp);

	return 0;
}
