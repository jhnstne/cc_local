/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#ifndef _POLYROOTS_H_
#define _POLYROOTS_H_

/*
 *----------------------------------------------------------------------
 *  File:  PolyRoots.h
 *	This file contains declarations for C++.
 *----------------------------------------------------------------------
 */

void QuadraticRoots(double C[3], int *nroots, double roots[2]);
void CubicRoots(double C[3], int *nroots, double roots[3]);
void QuarticRoots(double C[4], int *nroots, double roots[4]);

#endif /* _POLYROOTS_H_ */
