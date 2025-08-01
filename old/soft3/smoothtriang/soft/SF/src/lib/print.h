/*
 *----------------------------------------------------------------------
 *  File:  print.h
 *----------------------------------------------------------------------
 */

#ifndef _PRINT_H_
#define _PRINT_H_

#include "geometry.h"

int PrintPoint(FILE* fp, Point p);
int PrintVector(FILE* fp, Vector v);
int PrintNormal(FILE* fp, Normal n);
char* PString(Point p);
char* VString(Vector v);
char* NString(Normal n);

#endif /* _PRINT_H_ */
