/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: Wed Jul 05, 1989 at 10:25:24 AM
** Purpose: Interface module for (triangular) mesh face neighborhoods.
**
** $Header: face.h,v 1.3 89/06/23 07:24:52 derose Locked $
*/

#ifndef _NHOOD_H_
#define _NHOOD_H_

#define MAX_NEIGHBORS    128

typedef struct {
    int normaldim;          /* 0 if no normals read, 3 otherwise.           */
    int nr, ns, nt;         /* Num vertices adj to verts of central face    */
    VERTEX *r, *s, *t;      /* Pointers to the vertices of the central face */
    VERTEX *Pr, *Ps, *Pt;   /* The rings of neighboring vertices.           */
} Nhood;

/*
  int NhoodRead( fp, nhood, space)
  FILE *fp;
  Nhood *nhood;
  Space space;

  Read a face neighborhood description from the file fp.  One is returned
  on success, zero on end of file. The coordinates read in are assumed to
  be coordinates relative to the standard frame of space.
*/
extern int NhoodRead(FILE *fp, Nhood *nhood, Space space);

/*
  void NhoodFree( nhood)
  Nhood *nhood;
  
  Free the storage allocated to the face neighborhood.
*/
extern void FaceNhoodFree( Nhood nhood);


void ReadRing( int ring, int* num, VERTEX** vertices, 
		int normaldim, Space space);
void ReadVertices( int ring, VERTEX P[], int n, int normaldim, Space range);

#endif /* _NHOOD_H_ */
