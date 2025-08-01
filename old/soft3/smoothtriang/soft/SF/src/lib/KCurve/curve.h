/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  curve.h
 *----------------------------------------------------------------------
 */

#define MAX 10

typedef struct{
  int degree;
  Point cp[MAX];
} Curve;

