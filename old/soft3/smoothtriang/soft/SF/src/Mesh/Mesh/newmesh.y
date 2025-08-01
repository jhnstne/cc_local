%{

/*
 *----------------------------------------------------------------------
 *  File:  newmesh.y
 *  Last Modified: Sun Jun 25, 1989 at 09:27:48 AM
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <strings.h>
#include <ctype.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "userData.h"
#include "meshsym.h"

#ifndef NULL
#define NULL 0
#endif


extern SymbolEntry *InstallSymbol(), *LookupSymbol();

Link*   vertexList=NULL;
Link*	faceList=NULL;
Link*	edgeList=NULL;
Edge    *firstedge, *edge;
SymbolEntry *sym;
int     edgecnt, Msp = 0, level = 0;
char	yaccErrorString[1000];
static void    (*etoiV)();
static void    (*etoiE)();
static void    (*etoiF)();
static FILE *fp=stdin;
static void* e_mptr;

%}
%union {
	Mesh    *m;
	Node	*n;
	Edge    *e;
        double  val;
	char    *name;
	SymbolEntry *sym;
}
%start  meshfile
%token  <name>  IDENT
%token  <name>  DSTRUCT_TOKEN
%token  END
%type   <m>     mesh meshfile
%type	<n>	point pointName face faceName facedata pointfacelist
%type   <e>     namedPointList
%type   <sym>   ident
%right  '='
%%

meshfile	:	pointfacelist mesh
			{
			  e_mptr = $2;
			  return 0;
			}
	 	|	mesh
			{
			  e_mptr = $1;
			  return 0;
			}
		|	END { return NULL; }
		;

pointfacelist	:	pointfacelist point
		|	pointfacelist face
		|	point
		|	face
		;

point		:	ident '=' DSTRUCT_TOKEN
			{
			  $1->type = POINT;
			  $1->u.n = NewNode();
			  $1->u.n->name = $1->name;
			  fpReadDstruct(fp, &($1->u.n->externalData));
			  $1->u.n->tag = POINT;
			  $1->u.n->rep = NULL;
			  if ( etoiV != NULL ) {
				  etoiV($1->u.n);
				  dDeleteLnode($1->u.n->externalData);
				  $1->u.n->externalData = NULL;
			  }
			  AddToList(&vertexList,$1->u.n);

			}
		| 	ident ';'
			{
			  $$ = NewNode();
			  $$->tag = POINT;
			  $$->rep = NULL;
			  $$->name = $1->name;
			  $1->type = POINT;
			  $1->u.n = $$;
			  AddToList(&vertexList,$$);
			}
		;

face		:	ident '=' facedata
  			{
			  $3->name = $1->name;
			  $1->type = FACE;
			  $1->u.n = $3;
			  CheckFace($3);
			}
		;
facedata	:	'[' namedPointList ']'  DSTRUCT_TOKEN 
			{
			  $$ = NewNode();
			  fpReadDstruct(fp, &($$->externalData));
			  $$->tag = FACE;
			  $$->rep = $2;	/* this points face to an edge */
			  $2->next = firstedge;
			  firstedge->prev = $2;
			  InstallEdge($2->p,firstedge);

			  /* loop to set dual pointers */
			  edge = firstedge;
			  do{
			    edge->d = $$;
			    edge = edge->next;
			  } while ( edge != firstedge );
			  AddToList(&faceList,$$);
			}
		|	'[' namedPointList ']' ';'
			{
			  $$ = NewNode();
			  $$->tag = FACE;
			  $$->rep = $2;	/* this points face to an edge */
			  $2->next = firstedge;
			  firstedge->prev = $2;
			  InstallEdge($2->p,firstedge);

			  /* loop to set dual pointers */
			  edge = firstedge;
			  do{
			    edge->d = $$;
			    edge = edge->next;
			  } while ( edge != firstedge );
			  AddToList(&faceList,$$);
			}
		;

namedPointList	:	pointName
			{
			  $$ = NewEdge();
			  $$->p = $1;
			  $1->rep = $$;	/* this points point to edge */
			  firstedge = $$;
			}
		|	namedPointList ',' pointName
			{
			  $$ = NewEdge();
			  $$->p = $3;
			  $1->next = $$;
			  $$->prev = $1;
			  $3->rep = $$;	/* this points point to edge */
			  InstallEdge($1->p,$$);
			}
		;

pointName	:	IDENT
			{
			  if ( (sym = LookupSymbol($1))  == NULL ) {
			    smtcerror("unknown identifier", $1);
			  } else if ( sym->type != POINT ) {
			    sprintf(yaccErrorString,"%s not of type POINT",$1);
			    smtcerror(yaccErrorString,"");
			  } else {
			    $$ = sym->u.n;
			  }
			}


		|	DSTRUCT_TOKEN
			{
			  $$ = NewNode();
			  fpReadDstruct(fp, &($$->externalData));
			  $$->tag = POINT;
			  $$->rep = NULL;
			  AddToList(&vertexList,$$);
			}
		;

mesh		:	ident '=' '{' namedFaceList '}'  DSTRUCT_TOKEN
			{
			  $$ = NewMesh();
			  fpReadDstruct(fp, &($$->externalData));
			  $$->name = $1->name;
			  $$->faceList = extractMarkedElements(&faceList);
			  markEdgesAndVertices($$->faceList);
			  $$->edgeList = extractMarkedElements(&edgeList);
			  $$->vertexList = extractMarkedElements(&vertexList);
			  fixAndCheckEdgeList($$->edgeList);
			}
		|	ident '=' '{' namedFaceList '}' ';'
			{
			  $$ = NewMesh();
			  $$->name = $1->name;
			  $$->faceList = extractMarkedElements(&faceList);
			  markEdgesAndVertices($$->faceList);
			  $$->edgeList = extractMarkedElements(&edgeList);
			  $$->vertexList = extractMarkedElements(&vertexList);
			  fixAndCheckEdgeList($$->edgeList);
			}
		;

namedFaceList	:	faceName
		|	namedFaceList ',' faceName
		;

faceName	:	IDENT
			{
			  if ( (sym = LookupSymbol($1))  == NULL ) {
			    smtcerror("unknown identifier", $1);
		 	  } else if ( sym->type == POINT ) {
				  SetIFlags(sym->u.n, I_REP);
			  } else if ( sym->type != FACE ) {
			    sprintf(yaccErrorString,"%s not of type POINT or FACE",$1);
			    smtcerror(yaccErrorString,"");
			  } else {
			    if ( QIFlags(sym->u.n, I_REP) ) {
				    sprintf(yaccErrorString,"Face %s has already appeared in a mesh\n",ReturnName(sym->u.n));
				    yyerror(yaccErrorString,"");
				    exit(1);
			 		  	    
			    }
			    SetIFlags(sym->u.n, I_REP);
			  }
			}
		|	facedata
			{
			  SetIFlags($1, I_REP);
			}
		;
 
ident		:	IDENT
			{
			  $$ = InstallSymbol($1);
			}
		;


%%

/*****************************************************************************/

#include <ctype.h>

char    sbuf[100],*progname;
int     lineno = 1;


#define NORMAL 0
#define STRING 1
#define MAX_LEN 500




/*
 *----------------------------------------------------------------------
 *  Function:  yylex
 *----------------------------------------------------------------------
 */
yylex()
{
        int c;
yylexStart:
	while ((c=getc(fp)) == ' ' || c == '\t' || c == '\n')
                if (c == '\n')
                        lineno++;

	if ( c != '(' ) {
	    if (c == EOF)
		    return END;
	    if (c == '%'){
	      while(getc(fp)!='\n');
	      lineno++;
	      goto yylexStart;
	    }
	    if (isalpha(c)) {
		    SymbolEntry*sp;
		    char *p = sbuf;
		    do {
			    *p++ = c;
		    } while ((c=getc(fp)) != EOF && isalnum(c));
		    ungetc(c, fp);
		    *p = '\0';
		    yylval.name = sbuf;
		    return IDENT;
	    }
	    return c;
	} else {
	    ungetc(c,fp);
	    return DSTRUCT_TOKEN;
	  }
      }




int yaccErrorFlag=0;

/*
 *----------------------------------------------------------------------
 *  Function:  yyerror
 *----------------------------------------------------------------------
 */
yyerror(s)
        char *s;
{
  yaccErrorFlag = 1;
  warning(s, (char *) 0);
}





/*
 *----------------------------------------------------------------------
 *  Function:  smtcerror
 *----------------------------------------------------------------------
 */
smtcerror(s, t)
        char *s, *t;
{
  yaccErrorFlag = 1;
  warning(s, t);
}





/*
 *----------------------------------------------------------------------
 *  Function:  warning
 *----------------------------------------------------------------------
 */
warning(s, t)
        char *s, *t;
{
  fprintf(stderr, "mesh yyerror:%s: %s", progname, s);
  if (t)
    fprintf(stderr, " %s", t);
  fprintf(stderr, " near line %d\n", lineno);
}



/*
 *----------------------------------------------------------------------
 *  Function:  meshparse
 *----------------------------------------------------------------------
 */
Mesh *meshparse(fpIN)
FILE *fpIN;
{
  Mesh* m;

  fp = fpIN;

  e_mptr = NULL;
  m = (Mesh *) yyparse();	/* but not on alphas! */
  m = e_mptr;		
  if ( yaccErrorFlag ) {
    fprintf(stderr,"Error in reading mesh.  Exiting.\n");
    exit(1);
  } else {
	  if(debugLevel(1)&&!CheckEdge(268524864))
	    fprintf(stderr,"Bad edge after parse %d\n",QIFlags(268524864,7));
    return m;
  }
}

/*
 *----------------------------------------------------------------------
 *  Function:  meshEtoIparse
 *----------------------------------------------------------------------
 */
Mesh* meshEtoIparse(fpIN,eiV,eiE,eiF)
FILE* fpIN;
void (*eiV)();
void (*eiE)();
void (*eiF)();
{
	Mesh* m;

	etoiV = eiV;
	etoiE = eiE;
	etoiF = eiF;
	m = meshparse(fpIN);
	etoiV = NULL;
	etoiE = NULL;
	etoiF = NULL;
}

/*
 *----------------------------------------------------------------------
 *  Function:  AddToList
 *----------------------------------------------------------------------
 */
void AddToList(l,d)
Link** l;
char* d;
{
  Link* n;

  n = (Link*) malloc (sizeof(Link));
  if ( n == NULL ) {
    fprintf(stderr,"newmesh.y: AddToList(): malloc failed.  Exiting.\n");
    exit(1);
  }
  n->data = d;
  n->next = *l;
  *l = n;
}


int cmpPtr(a, b)
void** a;
void** b;
{
	return (int)*a - (int)*b;
}

CheckFace(f)
Face* f;
{
	Edge* e;
	Edge* ef;
	static Vertex** va;
	static size=0;
	int n;
	int i;

	ef = e = f->rep;

	/* Make sure there are 3 or edges in the face */
	if ( e == e->next  ||  e == e->next->next ) {
		if ( ReturnName(f) ) {
			fprintf(stderr,"Face %s fewer than 3 sides.\n",
				ReturnName(f));
			exit(1);
		} else {
			fprintf(stderr,"Unnamed Face fewer than 3 sides.\n",
				ReturnName(f));
			exit(1);
		}
	}

	/* Now, make sure no duplicate vertex in face */
	/* count the vertices */
	n = 0;
	for ( e = ef->next; e != ef; e = e->next ) {
		n++;
	}

	/* make sure our array is big enough */
	if ( n > size ) {
		free(va);
		va = (Vertex**) malloc ( n * sizeof(Vertex*) );
		size = n;
	}

	/* put the vertices in the list */
	i = 0;
	for ( e = ef->next; e != ef; e = e->next ) {
		va[i++] = e->p;
	}
	if ( i != n ) {
		fprintf(stderr,"Huh?\n");
		exit(1);
	}

	qsort(va, n, sizeof(Edge*), cmpPtr);

	/* make sure no duplicates occur */
	for (i=0; i<n-1; i++) {
		if ( (int)(va[i]) >= (int)(va[i+1]) ) {
			if ( ReturnName(va[i])  &&  ReturnName(f) ) {
				fprintf(stderr,
					"Multiple occurance of vertex %s in face %s\n",
					ReturnName(va[i]),ReturnName(f));
				exit(1);
			}
		}
	}
	
}


static Link* extractMarkedElements(l)
Link** l;
{
	Link* lm;
	Link** p;
	Link* tmp;

	lm = NULL;
	p = l;
	while ( *p != NULL ) {
		if ( QIFlags((Node*)((*p)->data), I_REP) ) {
			tmp = *p;
			*p = (*p)->next;
			tmp->next = lm;
			lm = tmp;
		} else {
			p = &((*p)->next);
		}
	}
	return lm;
}

static void markEdgesAndVertices(l)
Link* l;
{
	Edge* e;

	while ( l != NULL ) {
		e = (Edge*)(((Face*)(l->data))->rep);
		do {
			SetIFlags(e, I_REP);
			SetIFlags(e->p, I_REP);
			e = e->next;
		} while ( e != (Edge*)(((Face*)(l->data))->rep) );
		l = l->next;
	}
}


static void fixAndCheckEdgeList(l)
Link* l;
{
	Edge* e;

	while ( l != NULL ) {
		e = (Edge*)(l->data);
		if ( !QIFlags(e, I_REP) ) {
			fprintf(stderr,"fixAndCheckEdgeList: unmarked edge\n");
			exit(1);
		}
		if ( !QIFlags(e->d, I_REP) ) {
			fprintf(stderr,"fixAndCheckEdgeList: edge marked but face %s isn't\n",ReturnName(e->d));
			exit(1);
		}
		if ( e->sym != NULL ) {
			if ( !QIFlags(e->sym->d, I_REP) ) {
				fprintf(stderr,"fixAndCheckEdgeList: edge marked but sym->face isn't\n");
				exit(1);
			}
			ClearIFlags(e->sym, I_REP);
		}
		l = l->next;
	}
}
