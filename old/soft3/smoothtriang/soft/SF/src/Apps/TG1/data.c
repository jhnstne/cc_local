/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  data.c
 *	Keep track of which balls/sticks have been seen.
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include "math.h"
#include "all.h"
#include "avl.h"


#define float SQ(a) ((a)*(a))

/* global variables */

double tolerance;

AVL_TREE point_avl;
AVL_TREE stick_avl;

extern Space world;
extern Frame worldF;


/*
 *----------------------------------------------------------------------
 *  Function:  comparePoints
 *	qsort type comparison of two points.  
 *----------------------------------------------------------------------
 */
int comparePoints(Point *a, Point *b)
{
  int i;
  Scalar p1[3],p2[3];

  if ( VMag(PPDiff(*a,*b)) < tolerance ){
    return 0;
  }

  PCoords(*a, StdFrame(world), p1, p1+1, p1+2 );
  PCoords(*b, StdFrame(world), p2, p2+1, p2+2 );

  for(i=0; i<3; i++){
    if ( p1[i] < p2[i] )
      return -1;
    else if ( p1[i] > p2[i] )
      return 1;
  }
  return 0;
}





void PrintPoint(Point *p)
{
  Scalar x,y,z;

  PCoords(*p, worldF, &x, &y, &z);
  printf("(%g, %g, %g)",x,y,z);
}

/*
 *----------------------------------------------------------------------
 *  Function:  InitDataStructure
 *----------------------------------------------------------------------
 */
void InitDataStructure()
{
  AVL_TREE create();

  point_avl = create( comparePoints, PrintPoint );
}


struct mypoint {
  VERTEX v;
  Scalar gk;
};


/*
 *----------------------------------------------------------------------
 *  Function:  CheckAndAddPoint
 *	See if a point is already in our point list.  If it is, return
 *  OLD_POINT.  If it isn't, add it and return NEW_POINT
 *----------------------------------------------------------------------
 */
void CheckAndAddPoint(Point p, Normal n, double tol, double gk)
{
  struct mypoint v1, *v2;
  extern int findMin;
  extern double minDot;
  extern double maxGk;

  tolerance = tol;

  v1.v.position = p;
  v1.v.normal = n;
  v1.gk = gk;
  if ( (v2 = (struct mypoint*)find(point_avl,&v1)) != NULL ){
    if ( !findMin  &&  fabs( VVDot( VNormalize(NDual(v1.v.normal )), 
		      VNormalize(NDual(v2->v.normal)) ) - 1.0 ) > tol ){
      fprintf(stderr,"Not G1.  Exiting.\n");
      exit(0);
    } else {
      double d;

      d = fabs(VVDot( VNormalize(NDual(v1.v.normal)),
		      VNormalize(NDual(v2->v.normal))) );
      if ( d < minDot ){
	minDot = d;
      }
      if ( fabs(gk-v2->gk) > maxGk ) {
	maxGk = fabs(gk-v2->gk);
      }
    }
  } else {
    v2 = (struct mypoint*) malloc ( sizeof(struct mypoint) );
    if ( v2 == NULL ) {
      fprintf(stderr,"CheckAndAddPoint():Out of Memory.  Exiting.\n");
      exit(1);
    }
    *v2 = v1;
    insert(point_avl,v2);
  }
}
