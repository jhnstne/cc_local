/*
 *----------------------------------------------------------------------
 *  File:  spatch.h
 *----------------------------------------------------------------------
 */

#ifndef _SPATCH_H_
#define _SPATCH_H_

#include "geometry.h"
#include "sff.h"

typedef struct {
	int s,d,np;
	Point* net;
} Spatch;

extern Spatch fReadSpatch(FILE* fp, Frame f);
extern fWriteSpatch(FILE* fp, Frame f, Spatch s);
extern Point EvalSpatch(Spatch, Scalar [2]);
extern void EvalSpatchWNormalSFF(Spatch sp, Scalar d[2], 
				 Point* p, Normal* n, SFF* sff, int sflag);
extern fTessSpatch(FILE* fp, Spatch s, int n, int sflag);
FreeSpatch(Spatch sp);
extern void EvalSpatchFirstSecond(Spatch sp, Scalar d[2],
				  Point* p,
	  			  Vector* dx, Vector* dy,
				  Vector* dxdx, Vector* dxdy, Vector* dydy);

#endif
