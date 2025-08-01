/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  stick.h
 *----------------------------------------------------------------------
 */

typedef struct {
  Point p[3];
  Normal n[3];
  Material m;
} Triangle;

typedef struct {
  Point c;
  Scalar r;
  Material m;
} Sphere;

typedef struct {
  Normal n;
  Point p;
  double r;
  Material m;
} Square;

typedef struct {
  Point p1, p2;
  Scalar r;
  Material m;
} Stick;

/* constants */
#define I_DSTRUCT 1
#define I_SGP 2
#define I_A3D 3
#define I_SEGMENT 4
#define I_S3D 4

#define LOW_QUALITY 1
#define MED_QUALITY 2
#define HIGH_QUALITY 3

extern Space world;
extern Frame worldF;
extern int quality;
