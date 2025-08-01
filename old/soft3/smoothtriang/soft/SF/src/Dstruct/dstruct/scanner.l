%{
#include "dstruct.h"
#include "y.tab.h"

extern char *malloc();
extern double atof();
extern int lineno;
%}
%%
[ \t]*			;
\n			{
			    lineno++;
			}
\"[^"\n]*\"		{
                            int i;
                            yylval.sval = malloc(sizeof(char)*(yyleng-1));

                            /* Copy all but the quotes into yylval */
                            for (i=0; i<yyleng-2; i++) 
                                yylval.sval[i] = yytext[i+1];
                            yylval.sval[i] = NULL;
			    return( STRING);
                        };
[A-Za-z][A-Za-z0-9_-]*  {
			    yystring(yytext);
			    return( IDENT);
                        };
[0-9]+			{ 
			    yyreal(yytext);
			    return( REAL);
			};
-[0-9]+			{ 
			    yyreal(yytext);
			    return( REAL);
			};
[0-9]"."[0-9]+e[-+][0-9]+ {
			    yyreal(yytext);
			    return( REAL );
			}
-[0-9]"."[0-9]+e[-+][0-9]+ {
			    yyreal(yytext);
			    return( REAL );
			}
[0-9]e[-+][0-9]+ {
			    yyreal(yytext);
			    return( REAL );
			}
-[0-9]e[-+][0-9]+ {
			    yyreal(yytext);
			    return( REAL );
			}
[0-9]*"."[0-9]+		{
			    yyreal(yytext);
			    return( REAL);
                        };
-[0-9]*"."[0-9]+		{
			    yyreal(yytext);
			    return( REAL);
                        };
[0-9]+"."[0-9]*		{
			    yyreal(yytext);
			    return( REAL);
                        };
-[0-9]+"."[0-9]*		{
			    yyreal(yytext);
			    return( REAL);
                        };
.                       return( yytext[0]);

%%
