/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  triangulate.c
 *  NOTE:
 *	The code in this file is translated C++ code.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "all.h"
#include "stick.h"

int quality=MED_QUALITY;



/*
 *----------------------------------------------------------------------
 *  Function:  LowQualitySphere
 *	Print out a triangular approximation (octahedron) of a
 *  sphere if it hasn't been seen before.
 *----------------------------------------------------------------------
 */
void LowQualitySphere(Sphere s)
{
  Vector top,bottom,center[6];
  Point tp[3];
  Vector tv[3];
  Triangle t;
  int i;
  extern int outtype;

  top    = SVMult(s.r, VCreate(worldF, 0.0, 0.0, 1.0));
  bottom = SVMult(s.r, VCreate(worldF, 0.0, 0.0,-1.0));

  center[0] = SVMult(s.r, VCreate(worldF,   1.0, 0.0, 0.0));
  center[1] = SVMult(s.r, VCreate(worldF,   0.0, 1.0, 0.0));
  center[2] = SVMult(s.r, VCreate(worldF,  -1.0, 0.0, 0.0));
  center[3] = SVMult(s.r, VCreate(worldF,   0.0,-1.0, 0.0));
  center[4] = SVMult(s.r, VCreate(worldF,   1.0, 0.0, 0.0));


  if ( outtype == I_S3D ) {
	  PrintS3dOcta(top,bottom,center,s.c,s.m);
	  return;
  }
  t.m = s.m;

  for(i=0;i<4;i++){
    t.p[0] = PVAdd( s.c, top );
    t.n[0] = VDual( top );
    t.p[1] = PVAdd( s.c, center[i] );
    t.n[1] = VDual( center[i] );
    t.p[2] = PVAdd( s.c, center[i+1] );
    t.n[2] = VDual( center[i+1] );
    PrintTriangle(t);

    t.p[0] = PVAdd( s.c, bottom );
    t.n[0] = VDual( bottom );
    t.p[1] = PVAdd( s.c, center[i+1] );
    t.n[1] = VDual( center[i+1] );
    t.p[2] = PVAdd( s.c, center[i] );
    t.n[2] = VDual( center[i] );
    PrintTriangle(t);
  }
}

/*
 *----------------------------------------------------------------------
 *  Function:  MedQualitySphere
 *	Print out a triangular approximation (icosohedron) of a
 *  sphere if it hasn't been seen before.
 *----------------------------------------------------------------------
 */
void MedQualitySphere(Sphere s)
{
  Vector top,bottom,topRow[6],bottomRow[6];
  Point tp[3];
  Vector tv[3];
  Triangle t;
  int i;


  top    = SVMult(s.r, VCreate(worldF, 0.0, 0.0, 1.0));
  bottom = SVMult(s.r, VCreate(worldF, 0.0, 0.0,-1.0));

  topRow[0] = SVMult(s.r, VCreate(worldF,   .866,     0., .5));
  topRow[1] = SVMult(s.r, VCreate(worldF,  .2676,   .823, .5));
  topRow[2] = SVMult(s.r, VCreate(worldF, -.7006,  .5090, .5));
  topRow[3] = SVMult(s.r, VCreate(worldF, -.7006, -.5090, .5));
  topRow[4] = SVMult(s.r, VCreate(worldF,  .2676,  -.823, .5));
  topRow[5] = SVMult(s.r, VCreate(worldF,   .866,     0., .5));

  for(i=0;i<6;i++)
    bottomRow[i] = SVMult(-1.0, topRow[(i+3)%5]);

  t.m = s.m;

  for(i=0;i<5;i++){
    t.p[0] = PVAdd( s.c, top );
    t.n[0] = VDual( top );
    t.p[1] = PVAdd( s.c, topRow[i] );
    t.n[1] = VDual( topRow[i] );
    t.p[2] = PVAdd( s.c, topRow[i+1] );
    t.n[2] = VDual( topRow[i+1] );
    PrintTriangle(t);

    t.p[0] = PVAdd( s.c, bottom );
    t.n[0] = VDual( bottom );
    t.p[1] = PVAdd( s.c, bottomRow[i] );
    t.n[1] = VDual( bottomRow[i] );
    t.p[2] = PVAdd( s.c, bottomRow[i+1] );
    t.n[2] = VDual( bottomRow[i+1] );
    PrintTriangle(t);
  }
  for(i=0;i<5;i++){
    t.p[0] = PVAdd( s.c, topRow[i] );
    t.n[0] = VDual( topRow[i] );
    t.p[1] = PVAdd( s.c, bottomRow[i] );
    t.n[1] = VDual( bottomRow[i] );
    t.p[2] = PVAdd( s.c, topRow[i+1] );
    t.n[2] = VDual( topRow[i+1] );
    PrintTriangle(t);

    t.p[0] = PVAdd( s.c, bottomRow[i] );
    t.n[0] = VDual( bottomRow[i] );
    t.p[1] = PVAdd( s.c, topRow[i+1] );
    t.n[1] = VDual( topRow[i+1] );
    t.p[2] = PVAdd( s.c, bottomRow[i+1] );
    t.n[2] = VDual( bottomRow[i+1] );
    PrintTriangle(t);
  }
}

/*
 *----------------------------------------------------------------------
 *  Function:  TriangulateSphere
 *	Print out a triangular approximation (icosohedron) of a
 *  sphere if it hasn't been seen before.
 *----------------------------------------------------------------------
 */
void TriangulateSphere(Sphere s)
{
	if ( quality == LOW_QUALITY ) {
		LowQualitySphere(s);
	} else {
		MedQualitySphere(s);
	}
}




Vector UnitPerpToVector(Vector v)
{
  /* Find vector perpendicular to v */
  Vector b1, b2;	/* basis vectors */
  Vector v1,v2;

  b1 = VCreate( worldF, 1.0, 0.0, 0.0 );
  b2 = VCreate( worldF, 0.0, 1.0, 0.0 );


  /* take cross product of v with two basis vectors; 
     make v1 be the longer of these two vectors */
  v1 = VVCross(b1,v);
  v2 = VVCross(b2,v);

  if ( VMag(v1) < VMag(v2) ){
    v = SVMult(1.0/VMag(v2), v2);
  } else {
    v = SVMult(1.0/VMag(v1), v1);
  }
  return VNormalize(v);
}


/*
 *----------------------------------------------------------------------
 *  Function:  TriangleNormal
 *----------------------------------------------------------------------
 */
void TriangulateNormal(Square sq)
{
  Point tp[3];
  Vector tv[3];
  Vector v1,v2;
  Triangle t;

  t.m = sq.m;

  t.n[0] = sq.n;
  t.n[1] = sq.n;
  t.n[2] = sq.n;

  v1 = SVMult(sq.r, UnitPerpToVector(NDual(sq.n)));
  v2 = SVMult( 1.0/VMag(NDual(sq.n)), VVCross( NDual(sq.n), v1));

  t.p[0] = sq.p;
  t.p[1] = PVAdd( sq.p, v1);
  t.p[2] = PVAdd( sq.p, v2);
  PrintTriangle(t);


  t.p[1] = PVAdd( sq.p, v2);
  t.p[2] = PVAdd( sq.p, SVMult(-1.0, v1));
  PrintTriangle(t);

  t.p[1] = PVAdd( sq.p, SVMult(-1.0, v1));
  t.p[2] = PVAdd( sq.p, SVMult(-1.0, v2));
  PrintTriangle(t);

  t.p[1] = PVAdd( sq.p, SVMult(-1.0, v2));
  t.p[2] = PVAdd( sq.p, v1);
  PrintTriangle(t);

if(getenv("TESTING")){
Scalar x,y,z;
PCoords(sq.p,StdFrame(SpaceOf(sq.p)),&x,&y,&z);
printf("%g %g %g 0\n",x,y,z);
PCoords(PVAdd(sq.p,NDual(sq.n)),StdFrame(SpaceOf(sq.p)),&x,&y,&z);
printf("%g %g %g 1\n",x,y,z);
}
}


int delta = 1;

/*
 *----------------------------------------------------------------------
 *  Function:  TriangulateStick
 *	Print out a triangular approximation for a stick if it hasn't
 *  been seen before.
 *----------------------------------------------------------------------
 */
void TriangulateStick(Stick st)
{
  Vector v;
  Vector v1;
  double dist;
  Vector e1[9];
  int i;
  Triangle t;
  extern int outtype;

  t.m = st.m;

  /* should check to see that dimension of space is 3 */

  v = PPDiff(st.p1, st.p2);
  dist = VMag(v);
  v = SVMult(1.0/dist, v);

  v1 = SVMult(st.r, UnitPerpToVector(v));


  /* Make arrays of vectors to form vertices of triangles */

  e1[0] = v1;
  e1[2] = VVCross(v1,v);
  e1[4] = SVMult(-1.0, e1[0]);
  e1[6] = SVMult(-1.0, e1[2]);
  e1[8] = e1[0];
  for(i=0; i<4; i++){
    e1[2*i+1] = SVMult( 0.5, VVAdd(e1[2*i], e1[2*(i+1)]) );
  }

  if ( outtype == I_S3D ) {
	  PrintS3dStick( st.p1, st.p2, e1, delta, st.m );
	  return;
  }

  /* Print the triangles */
  for(i=0; i<8; i += 1+delta){
    t.p[0] = PVAdd( st.p1, e1[i+0] );
    t.n[0] = VDual( e1[i+0] );
    t.p[1] = PVAdd( st.p2, e1[i+0] );
    t.n[1] = VDual( e1[i+0] );
    t.p[2] = PVAdd( st.p1, e1[i+1+delta] );
    t.n[2] = VDual( e1[i+1+delta] );
    PrintTriangle(t);

    t.p[0] = PVAdd( st.p2, e1[i+0] );
    t.n[0] = VDual( e1[i+0] );
    t.p[1] = PVAdd( st.p2, e1[i+1+delta] );
    t.n[1] = VDual( e1[i+1+delta] );
    t.p[2] = PVAdd( st.p1, e1[i+1+delta] );
    t.n[2] = VDual( e1[i+1+delta] );
    PrintTriangle(t);
  }
}
