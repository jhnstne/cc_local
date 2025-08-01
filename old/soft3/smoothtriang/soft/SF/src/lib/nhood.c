/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Tony DeRose
** Last Modified: Wed Jul 05, 1989 at 10:24:02 AM
** Purpose: Interface module for (triangular) mesh face neighborhoods.
**
** $Header: face.c,v 1.1 89/04/09 15:22:08 derose Locked $
**
*/
#include <stdio.h>
#include "util.h"
#include "geometry.h"
#include "vertex.h"
#include "nhood.h"


/* Forward declarations */
extern void ReadRing(int ring, int* num, VERTEX** vertices, 
		int normaldim, Space space);
extern void ReadVertices(int ring, VERTEX P[], int n, int normaldim, Space range);

/*
**  Read a face neighborhood description from the file fp.  One is returned
**  on success, zero on end of file. The coordinates read in are assumed to
**  be coordinates relative to the standard frame of space.
*/
int NhoodRead( FILE *fp, Nhood *nhood, Space space)
{
    char buf[BUFSIZE];
    int numsides, dim, normaldim, ratdim;
    Scalar scbuf[4];

    /* NOTE: READS FROM STDIN, NOT fp! */
    if ( ReadDstruct() == EOF ) {
	return 0;
    }
    if ( GetScalar("Face.numVertices",&scbuf[0]) == FALSE ) {
      fprintf(stderr,
        "NhoodRead: Error reading neighborhood num vertices.  Exiting.\n");
      exit(1);
    }
    if ( GetScalar("Face.euclideanDim",&scbuf[1]) == FALSE ) {
      fprintf(stderr,
        "NhoodRead: Error reading neighborhood euclidean dimension.  Exiting.\n");
      exit(1);
    }
    if ( GetScalar("Face.rationalDim",&scbuf[2]) == FALSE ) {
      fprintf(stderr,
        "NhoodRead: Error reading neighborhood rational dimension.  Exiting.\n");
      exit(1);
    }
    if ( GetScalar("Face.normalDim",&scbuf[3]) == FALSE ) {
      fprintf(stderr,
        "NhoodRead: Error reading neighborhood normal dimension.  Exiting.\n");
      exit(1);
    }
    numsides = scbuf[0]; dim = scbuf[1];
    ratdim = scbuf[2]; normaldim = scbuf[3];

    if ((numsides != 3) || (dim != 3) || (ratdim != 0)) {
      fprintf(stderr, 
      "NhoodRead: Can't support numside=%d, dim=%d, ratdim=%d, normaldim=%d\n",
	      numsides, dim, ratdim, normaldim);
      exit(1);
    }

    if ((normaldim != 0) && (normaldim != 3)) {
	fprintf(stderr, 
	"NhoodRead: Can't support numside=%d, dim=%d, ratdim=%d, normaldim=%d\n",
		numsides, dim, ratdim, normaldim);
	exit(1);
    }

    nhood->normaldim = normaldim;

    /* Read in the r, s, and t rings */
    ReadRing( 0, &(nhood->nr), &(nhood->Pr), normaldim, space);
    ReadRing( 1, &(nhood->ns), &(nhood->Ps), normaldim, space);
    ReadRing( 2, &(nhood->nt), &(nhood->Pt), normaldim, space);

    /* Extract the vertices of the nhood from the neighbor rings */
    nhood->r = nhood->Pt;
    nhood->s = nhood->Pr;
    nhood->t = nhood->Ps;
    return 1;
}


/*
** Read in a ring of vertices.  Abort on errors.
int ring;		   the ring to read 
int *num;                  Pointer to where number of verts goes/
VERTEX **vertices;         Pointer to buffer where vertices go   
int normaldim;             0 if no normals, three otherwise.     
Space space;
*/
void ReadRing( int ring, int* num, VERTEX** vertices, 
		int normaldim, Space space)
{
    char buf[BUFSIZE];
    Scalar scbuf;

    /* Read the numbers of vertices in the ring */
    sprintf(buf,"Face.vertexRing[%d].numNeighbors",ring);
    if ( GetScalar(buf,&scbuf) == FALSE ) {
      fprintf(stderr, "NhoodRead: Error reading vertex ring header; 1 arg expected .\n");
      exit(1);
    }
    *num = scbuf;

    if (*num < 3) {
	fprintf(stderr, "NhoodRead: vertex ring with %d neighbors; 3 is minimum.\n", *num);
	exit(1);
    }

    /* Allocate storage for the ring of vertices */
    *vertices = (VERTEX *) malloc( sizeof( VERTEX) * (*num));

    /* Read in the ring */
    ReadVertices( ring, *vertices, *num, normaldim, space);
}


/*
** Read in a ring of point.  Abort on error.
*/
void ReadVertices( int ring, VERTEX P[], int n, int normaldim, Space range)
{
  char buf[BUFSIZE];
  int i, nr, ns, nt;
  Scalar x, y, z, nx, ny, nz;

  for (i = 0; i < n; i++) {
    sprintf(buf,"Face.vertexRing[%d].neighbors[%d]",ring,i);
    if ( GetVertexPosition(buf,StdFrame(range),&(P[i].position)) == FALSE ) {
      fprintf(stderr,"ReadVertices: %s.pos doesn't exist.  Exiting\n",buf);
      exit(1);
    }
    if (normaldim == 3) {
      if ( GetVertexNormal(buf,StdFrame(range),&(P[i].normal)) == FALSE ) {
	fprintf(stderr,"ReadVertices: %s.normal doesn't exist. Exiting\n",buf);
	exit(1);
      }
    }
  }
}
    

/*
** Free the storage allocated to the face neighborhood
*/
void NhoodFree( Nhood nhood)
{
    free( nhood.Pr);
    free( nhood.Ps);
    free( nhood.Pt);
}

