/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  usage.h
 *  Last Modified: Fri Jul 07, 1989 at 08:28:14 AM
 *  Purpuse: Interface to default usage routine for commandline.c
 *----------------------------------------------------------------------
 */

#ifndef _USAGE_H_
#define _USAGE_H_

/*
 *----------------------------------------------------------------------
 *  Function:  Usage
 *	Print a message telling what parameters to call function with.
 *  Parameters:
 *	none
 *  Return Value:
 *	void
 *----------------------------------------------------------------------
 */
void Usage(char **argv);

extern char* Banner;
extern char* UsageString;

#endif _USAGE_H_
