/**
 * @file lists.c
 *
 * @author Andi
 * @date   04.06.2020
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

#include "rb2.h"

typedef struct list listT;

struct list {
	listT		*next;			///< this must be the header of the linked list structure
};

/**
 * Append an entry to the end of singly linked list. The next-pointer must
 * be the first element in the structure! The list must not be circular
 * connected - an inifnite loop would result!
 *
 * \param lst		a pointer to the list start (i.e. a pointer-pointer!)
 * \param entry		the entry to append to the list
 */
void list_append (void *lst, void *entry)
{
	listT **l;

	l = (listT **) lst;
	while (*l) l = &(*l)->next;
	*l = (listT *) entry;
	(*l)->next = NULL;
}

/**
 * Count the number of entries in the linked list.The list must not be circular
 * connected - an inifnite loop would result!
 *
 * \param lst		pointer to the first element of the list
 * \return			the number of elements in this list
 */
int list_len (void *lst)
{
	listT *l;
	int count = 0;

	l = (listT *) lst;
	while (l) {
		count++;
		l = l->next;
	}
	return count;
}

/**
 * Get the n-th entry of a list. If the requested index lies outside the list length
 * NULL is returned.
 *
 * \param lst		pointer to the first element of the list
 * \param idx		the requested index in the range of 0 .. listlen - 1
 * \return			pointer to the n-th element or NULL if beyond the list
 */
void *list_getIndexed (void *lst, int idx)
{
	listT *l;

	l = (listT *) lst;
	while (l && idx > 0) l = l->next;
	return (void *) l;
}
