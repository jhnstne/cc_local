/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#ifndef _COMMANDLINE_H_
#define _COMMANDLINE_H_

/*
**
** Author: Tony DeRose
** Created: Sun Jul 02, 1989 at 08:29:55 AM
** Last Modified: Sun Jul 02, 1989 at 08:40:08 AM
**
** Purpose: Header file for command line processing.
**          This package provides support for command line options
**          with relatively automatic help.
**
** TO ADD AN OPTION:
**
** Create a handling function defined such as SetX below, foward declare
** it, then add a line to the Options table.
**
** The handling function can expect to find its first argument in
** argv[0], the second in argv[1], and so on.
**
** An example handler expecting one numeric argument:
**
**      void SetX(argv)
**      char **argv;
**      {
**          int x;
**
**          x = atoi( argv[0]);
**       }
*/

typedef struct {                       /* Command line option structure */
    char *name;                        /* The option name, ex: "help"   */
    void (*handler)();                 /* Handling function             */
    int args;                          /* Number of args expected       */
    char *doc;                         /* Documentation string          */
} Option;


extern Option Options[];
void PrintOption( Option option );
void ParseCommandLine(int argc, char **argv);


#endif /* _COMMANDLINE_H_ */
