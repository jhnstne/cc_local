/* For systems without strdup */
#include <stdio.h>
#include <strings.h>

char* strdup( char* s )
{
	char* rs;

	rs = (char*) malloc ( strlen(s)+1 );
	strcpy(rs, s);
	return rs;
}
