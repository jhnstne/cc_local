/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** dstructMem.h
**
** Author: Michael Lounsbery
** Created: Sat Aug 12, 1989 at 03:19:30 PM
** Last Modified: Sat Aug 12, 1989 at 03:19:30 PM
**
** Purpose: 
*/


#define	FreeLnode(d)		pFreeLnode(&(d))
#define	FreeString(s)		pFreeString(&(s))

extern Lnode	*NewLnode();

extern char	*NewStringMem();

extern void	pFreeLnode(Lnode **p);
extern void	pFreeString(char **p);
