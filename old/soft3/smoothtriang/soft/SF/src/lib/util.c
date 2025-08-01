/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: 03/12/89 at 13:26:37
** Purpose: Random Utilities.
**
*/
#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "util.h"
#include "math.h"

/*
** Read a line of input, skipping over blank lines and lines
** beginning with %.  Zero is returned on error or end of file,
** one is returned otherwise.
*/
int ReadLine(FILE *fp, char *buffer)
{
    extern char *fgets();
    char *ret_code;

    while (1) {
	if (fgets( buffer, BUFSIZE, fp) == NULL) {
	    return 0;
	}
	if ((buffer[0] != '%') && (buffer[0] != '\n')) {
	    return 1;
	}
    }
}
