/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  meshsym.h
 *----------------------------------------------------------------------
 */

#ifndef _MESHSYM_H_
#define _MESHSYM_H_

#define EntityType short

typedef union Datum {
  double val;
  Node *n;
  Edge *e;
  Mesh *m;
} Datum;

typedef struct SymbolEntry {
  char *name;
  Datum	u;
  struct SymbolEntry   *next;
  short type;   /* POINT, FACE, MESH */
} SymbolEntry;

/* Edge Table data structures */

typedef struct EdgeEntry {
  Node *p,*q;
  Edge *e;
  struct EdgeEntry *next;
} EdgeEntry;

typedef struct Entity {
  EntityType type;
  union {
    Mesh *m;
    Edge *e;
    Point *p; 
  } sub;
} Entity;

#endif /* _MESHSYM_H_ */
