%{
/*
**
** Purpose: Dynamic structure parser.
**
** Author: Tony DeRose
** Created: Tue Jul 25, 1989 at 08:34:38 PM
** Last Modified: Tue Jul 31, 1989 
**
*/
#include <stdio.h>
#include <math.h>
#include "dstruct.h"

/* Import from lex */
extern int yyleng;
extern char *yytext;
extern int lineno;

static Lnode* dsp;

int error=0;

extern Lnode* din;
%}

%start START
%union {
    int ival;
    Lnode *lval;
    char *sval;
    double rval;
}
%type <lval> namevalue value nvlist valuearray valuelist atom
%type <sval> symbol
%token <sval> IDENT STRING
%token <rval> REAL
%%
START   : value ';'
             {
		 if ( !error ) {
		   dsp = $1;
		   return (int)$1;
		 } else
		     dsp = 0;
		     return 0;
	     }
	|
	  {
	    dsp = 0;
	    return 0;
	  }
        ;

namevalue : '(' symbol '.' value ')'
             {
		 $$ = NewNVPair( $2, $4 );
	     }
     ;

value 	:	atom
	|	nvlist
		{
		  $1->sval = NULL; /* clean up hacking in NewNVList() */
		}
	|	valuearray
	;

nvlist	:	nvlist namevalue
		{
		  $$ = NewNVList( $1, $2 );
		}
	|	namevalue
		{
		  $$ = NewNVList( NULL, $1 );
		}
	;

valuearray	:	'[' valuelist ']'
			{
			  $$ = $2;
			  $2->sval = NULL; /* clean up hacking in NewVlist() */
			}
	;

valuelist	:	valuelist ',' value
			{
			  $$ = NewVList( $1, $3 );
			}
		|	value
			{
			  $$ = NewVNode( $1 );
			}
		;


symbol : IDENT
       ;

atom : STRING 
             {
		 $$ = NewString( $1);
	     }
        | REAL
             {
		 $$ = NewReal($1);
	     }
        ;

%%


yyerror(s)
char* s;
{
  fprintf(stderr,"%s; in dstruct near line %d.  Exiting.\n",s,lineno);
  exit(1);
}


DSREAL()
{
  return REAL;
}

DSIDENT()
{
  return IDENT;
}

DSSTRING()
{
  return STRING;
}

extern Lnode* DSparse()
{
	yyparse();
	return dsp;
}

#if 0
/*
** Main program for testing purposes only.
*/

Lnode *din;

main(argc, argv)
int argc;
char **argv;
{
    while(yyparse()){
      DstructPrint(din);
    }
}
#endif

yystring(s)
char* s;
{
  void* malloc();

  yylval.sval = malloc(sizeof(char)*(strlen(s)+1));
  strcpy( yylval.sval, s );
}

yyreal(s)
char* s;
{
  yylval.rval = atof( s );
}
