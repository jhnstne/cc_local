/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  libgeo.h
 *----------------------------------------------------------------------
 */

BOOLEAN pGetVertexPosition(Lnode *d, char *s, Frame f, Point *p);
BOOLEAN pGetVertexNormal(Lnode *d, char *s, Frame f, Normal *n);
BOOLEAN pGetVector(Lnode *d, char *s, Frame f, Vector *v);
BOOLEAN dPutVertexPosition(Lnode **d, char *s, Point p);
BOOLEAN dPutVertexNormal(Lnode **d, char *s, Normal n);
BOOLEAN dPutVector(Lnode **d, char *s, Vector v);

#define pGetVertexPosition(ds, path, frame, vert)	\
  dGetVertexPosition(*(ds), (path), (frame), (vert))
#define pGetVertexNormal(ds, path, frame, norm)	\
  dGetVertexNormal(*(ds), (path), (frame), (norm))
#define pGetVector(ds, path, frame, vect)	\
  dGetVertexNormal(*(ds), (path), (frame), (vect))

#define dPutVertexPosition(ds,path,vert)	\
  pPutVertexPosition(&(ds),(path),(vert))
#define dPutVertexNormal(ds,path,norm)	\
  pPutVertexNormal(&(ds),(path),(norm))
#define dPutVector(ds,path,vect)	\
  pPutVertexNormal(&(ds),(path),(vect))
  

#define GetVertexPosition(d,f,v) \
				  dGetVertexPosition( din, (d), (f), (v) )
#define GetVertexNormal(d,f,n)   \
				  dGetVertexNormal( din, (d), (f), (n) )
#define GetVector(d,f,v)   \
				  dGetVector( din, (d), (f), (v) )
#define PutVertexPosition(d,v)   \
				  dPutVertexPosition( dout, (d), (v) )
#define PutVertexNormal(d,n)     \
				  dPutVertexNormal( dout, (d), (n) )
#define PutVector(d,v)     \
				  dPutVector( dout, (d), (v) )
