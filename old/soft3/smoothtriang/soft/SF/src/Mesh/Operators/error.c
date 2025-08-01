/*
       Stops program execution with an error message and abort()
*/

#include <stdio.h>

void Error( char *s )
{
    if ( s != NULL )
        fprintf( stderr, "%s\n", s );
    abort();
}

