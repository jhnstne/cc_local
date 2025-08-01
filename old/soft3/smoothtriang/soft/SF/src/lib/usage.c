/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*------------------------------------------------------------*/
/*                   Command Line Handlers                    */
/*------------------------------------------------------------*/

#include <stdio.h>
#include "commandline.h"
#include "usage.h"

void Usage(char **argv)
{
    int i;

    /* Print out a banner, and show synopsis */
    fprintf(stderr, "\n%s\n", Banner);
    fprintf(stderr, "Usage: \n\t%s\n", UsageString);

    /* Now print out a list of available options */
    fprintf(stderr, "Options:\n");

    for (i = 0; Options[i].name != NULL; i++) {
	fprintf(stderr, "\t");
	PrintOption(Options[i]);
    }
    exit(1);
}
