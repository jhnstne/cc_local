/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
**
** Author: Tony DeRose
** Created: Sun Jul 02, 1989 at 08:29:10 AM
** Last Modified: Wed Jul 05, 1989 at 03:00:40 PM
**
** Purpose: Command line option processing with "forced" documentation.
*/

#include <stdio.h>
#include "commandline.h"


void ParseCommandLine(int argc, char **argv)
{
    int i;
	
    /* Jump over the program name */
    argc--; argv++;

    while (argc) {
	if (argv[0][0] != '-') break; /* out of while loop */

	/* Search for matching option */
	for (i = 0; Options[i].name != NULL; i++) {
	    if (strcmp( &(argv[0][1]), Options[i].name) == 0) {
		/* Match found, call the handler */
		argc--; argv++;
		if (argc < Options[i].args) {
		    fprintf(stderr, "%d args expected, %d found.\n",
			                              argc, Options[i].args);
		    PrintOption( Options[i]);
		    exit(1);
		}
		(*Options[i].handler)(argv);
		argc -= Options[i].args;
		argv += Options[i].args;
		break; /* out of for loop */
	    }
	}
	/* See if a match was found */
	if (Options[i].name == NULL) {
	    fprintf(stderr, "\nUnknown option %s\n", argv[0]);
	    Usage( NULL);
	    exit(1);
	}
    }
}
    

void PrintOption( Option option )
{
    fprintf(stderr," -%s %s\n", option.name, option.doc);
}
