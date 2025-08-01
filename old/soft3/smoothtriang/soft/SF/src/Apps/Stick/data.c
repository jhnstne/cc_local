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
#include "all.h"
#include "stick.h"
#include "avl.h"


#define float SQ(a) ((a)*(a))

/* global variables */

double tolerance;

AVL_TREE point_avl;
AVL_TREE stick_avl;




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



/*
 *----------------------------------------------------------------------
 *  Function:  minMaxPoints
 *	Find the min and max of two points.
 *----------------------------------------------------------------------
 */
void minMaxPoints(Point *a, Point *b, Point **min, Point **max)
{
  int i;

  switch( comparePoints(a,b) ){
  case -1:
    *min = a;
    *max = b;
    break;
  case 0:
  case 1:
    *min = b;
    *max = a;
    break;
  default:
    fprintf(stderr,"minMaxPoints: internal error, unknown case statement.  Exiting.\n");
    exit(1);
  }
}



/*
 *----------------------------------------------------------------------
 *  Function:  compareSticks
 *	qsort type of comparison of two sticks.
 *----------------------------------------------------------------------
 */
int compareSticks(Point a[2], Point b[2])
{
  int r;
  Point *mina=NULL,*maxa,*minb=NULL,*maxb;

  minMaxPoints(&a[0],&a[1],&mina,&maxa);
  minMaxPoints(&b[0],&b[1],&minb,&maxb);

  r = comparePoints(mina,minb);
  if ( r != 0 ){
    return r;
  } else {
    return comparePoints(maxa,maxb);
  }
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
  stick_avl = create( compareSticks, PrintPoint );
}


#define OLD_POINT 0
#define NEW_POINT 1


/*
 *----------------------------------------------------------------------
 *  Function:  CheckAndAddPoint
 *	See if a point is already in our point list.  If it is, return
 *  OLD_POINT.  If it isn't, add it and return NEW_POINT
 *----------------------------------------------------------------------
 */
int CheckAndAddPoint(Point p, double tol)
{
  Point *np;

  tolerance = tol;

  if ( find(point_avl,&p) != NULL ){
    return OLD_POINT;
  } else {
    np = (Point*) malloc ( sizeof(Point) );
    if ( np == NULL ) {
      fprintf(stderr,"CheckAndAddPoint():Out of Memory\n");
      exit(1);
    }
    *np = p;
    insert(point_avl,np);
    return NEW_POINT;
  }
}


#define OLD_STICK 0
#define NEW_STICK 1



/*
 *----------------------------------------------------------------------
 *  Function:  CheckAndAddStick
 *	See if a stick is already in our stick list.  If it is, return
 *  OLD_STICK.  If it isn't, add it and return NEW_STICK.  If the
 *  distance between the two vertices is less than the tolerance,
 *  consider the stick to be an OLD_STICK, and don't add it.
 *----------------------------------------------------------------------
 */
int CheckAndAddStick(Point p, Point q, double tol)
{
  Point s[2];
  Point *ns;

  if ( VMag( PPDiff(p,q) ) < tolerance ){
    return OLD_STICK;
  }

  s[0] = p;
  s[1] = q;

  tolerance = tol;

  if ( find(stick_avl,s) != NULL ) {
    return OLD_STICK;
  } else {
    ns = (Point*) malloc ( 2 * sizeof(Point) );
    if ( ns == NULL ){
      fprintf(stderr,"CheckAndAddStick(): Out of Memory.  Exiting.\n");
      exit(1);
    }

    ns[0] = p;
    ns[1] = q;

    insert(stick_avl,ns);
    return NEW_POINT;
  }
}


