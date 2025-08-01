/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/* File: meshsym.c
** Author: Charles Loop
** Purpose: Symbol and Edge table routines required for the creation of a mesh
** Last Modified: 2 July, 1987
** Reason : Creation.
*/

#include <stdio.h>
#include "grailTypes.h"
#include "geometry.h"
#include "dstruct.h"
#include "mesh.h"
#include "meshsym.h"

#define SYMBOL_HASH_SIZE 167
#define EDGE_HASH_SIZE 257

static SymbolEntry *SymbolTable[SYMBOL_HASH_SIZE];
static EdgeEntry *EdgeTable[EDGE_HASH_SIZE];

int SymbolHash(char *s)
{
        char *p;
	unsigned h = 0, g;
	
	for (p = s; *p != '\0'; p++) {
	        h = (h << 4) + (*p);
		if (g = h&0xf0000000) {
		        h = h ^ (g >> 24);
			h = h ^ g;
		}
	}
	return h % SYMBOL_HASH_SIZE;
}

SymbolEntry *LookupSymbol(char *s)
{
        SymbolEntry *sp;
	
        for (sp = SymbolTable[SymbolHash(s)]; 
	     sp != (SymbolEntry *) 0; sp = sp->next) {
		if (strcmp(sp->name, s) == 0) {
			return sp;
		}
	}
        return (SymbolEntry *) 0;
}

SymbolEntry *InstallSymbol(char  *s)
{
        SymbolEntry *sp;
	int    hashval;
	
	if ((sp = LookupSymbol(s)) == (SymbolEntry *) 0) {
	        sp = (SymbolEntry *) calloc(1,sizeof(SymbolEntry));
		sp->name = (char *) calloc(1,sizeof(char)*(strlen(s)+1));
		strcpy(sp->name, s);
		hashval = SymbolHash(sp->name);
		sp->next = SymbolTable[hashval];
		SymbolTable[hashval] = sp;
	}
        return sp;
}

FreeSymbol(void)
{
	SymbolEntry *sp, *tp;
	int i;
	
	for (i = 0; i < SYMBOL_HASH_SIZE; i++) {
		for (sp = SymbolTable[i]; sp != (SymbolEntry *) 0;) {
			tp = sp;
			sp = sp->next;
			free((char *) tp);
		}
		SymbolTable[i] =(SymbolEntry *) 0;
	}
}

EdgeHash(Node *p,Node *q)
{
        return (((unsigned int)p + (unsigned int)q) % EDGE_HASH_SIZE);
}

Edge *LookupEdge(Node *p,Node *q)
{
        EdgeEntry *ep;
	
	for (ep = EdgeTable[EdgeHash(p,q)];
	     ep != (EdgeEntry *) 0; ep = ep->next) {
		if (ep->p == p && ep->q == q) { /* entry found */
			return ep->e;
		}
	}
	
	return (Edge *) 0;
}

InstallEdge(Node *p,Edge  *newedge)
{
	EdgeEntry  *ep;
	Edge       *oldedge;
	Node      *q = newedge->p;
	int        hashval;
	
	if (oldedge = LookupEdge(q, p)) {
		if (oldedge->sym)
		  fprintf(stderr, "edge usage conflict %s %s\n",
			  ReturnName(p), ReturnName(q));
		
		newedge->sym = oldedge;
		oldedge->sym = newedge;
	} else {
		if (LookupEdge(p, q)){
			fprintf(stderr, "edge orientation conflict at edge between %s and %s\n",p->name,q->name);
		} else {
			extern Link* edgeList;
			
			AddToList(&edgeList,newedge);
		}
		
		ep = (EdgeEntry *) calloc(1,sizeof(EdgeEntry));
		ep->p = p;
		ep->q = q;
		ep->e = newedge;
		hashval = EdgeHash(p,q);
		ep->next = EdgeTable[hashval];
		EdgeTable[hashval] = ep;
	}
}
