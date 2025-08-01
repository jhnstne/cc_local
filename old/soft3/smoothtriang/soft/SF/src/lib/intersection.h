/*
 *----------------------------------------------------------------------
 *  File:  intersection.h
 *----------------------------------------------------------------------
 */

#ifndef _INTERSECTION_H_
#define _INTERSECTION_H_

#include "geometry.h"

#define I_EMPTY 0
#define I_POINT 1
#define I_LINE 2
#define I_PLANE 3

typedef struct {
  int type;
  Point p;
  Vector v;
  Normal n;
} Intersection;


void PrintIntersection(FILE* fp, Intersection i, Frame WF);
Intersection IntersectTwoPlanes(Point p1, Normal n1, Point p2, Normal n2);
Intersection LineIntersectP(Point p1, Vector v1, Point p2, Normal n2);
Intersection IntersectThreePlanes(Point p1, Normal n1, Point p2, Normal n2,
				  Point p3, Normal n);
Intersection IntersectThreePlanesWF(Point p1, Normal n1, Point p2, Normal n2,
				    Point p3, Normal n, Point C);

#endif /* _INTERSECTION_H_ */
