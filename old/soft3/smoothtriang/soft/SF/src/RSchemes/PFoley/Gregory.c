/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Steve Mann
** 
*/
#include <stdio.h>
#include <math.h>
#include "all.h"
#include "../../lib/KCurve/curve.h"
#include <sys/time.h>
#include <sys/resource.h>

typedef struct {
  Patch p;
  Point pt[3][2];
} GregoryPatch;

/* Forward declarations */
static Point EdgeTangent();
static void TweakBoundary();
Point LineIntersectPlane();
static GregoryPatch AdjustPatch();
static Point EvalGregory();

/* Global variables */
static int fourNinths=0;
static int verbose=0;
static int new=0;
static int furthBoundary=0;
static int tweakBoundary=0;
static int goodBoundary=0;
static double tweak=0.5858;
static int gk=0;
static Scalar jensen=0.;

static char *RoutineName;
Space World;                 /* Space where faces and patches reside */
static Scalar ShapeParameter = 0.25;/* Controls magnitude of edge tangents  */







/*
 * GregoryProcessFace
 *
 * bb0, bb1, bb2 - construct the corresponding boundaries if true
 */
void GregoryProcessFace( Patch* patch0, Patch* patch1, Patch* patch2, 
	Vertex* vr[3], 
	BOOLEAN bc0, BOOLEAN bc1, BOOLEAN bc2, int samples )
{
    Patch patch;
    Point P[9];
    Scalar w[9];
    int i;
    GregoryPatch gp;
    Vertex *r, *s,  *t;

    r = vr[0];
    s = vr[1];
    t = vr[2];


    /* Create the cubic patch */
    patch = PatchCreate( 3, World, 0);

    /* Set the corner points of the net to the vertices of the face */
    P[6]= PatchGetPoint (*patch0, 3, 0, 0);
    PatchSetPoint( &patch, P[6], 3, 0, 0); 
    P[7]= PatchGetPoint (*patch0, 0, 3, 0);
    PatchSetPoint( &patch, P[7], 0, 3, 0); 
    P[8]= PatchGetPoint (*patch0, 0, 0, 3);
    PatchSetPoint( &patch, P[8], 0, 0, 3); 

#if 1
    /* Do the rs edge points */
    if (bc2) {
	P[0]=EdgeTangent( r, s );
	P[1]=EdgeTangent( s, r );
    } else {
	P[0]=PatchGetPoint (*patch2, 2, 1, 0);
	P[1]=PatchGetPoint (*patch2, 1, 2, 0);
	PatchSetPoint( &patch, P[0], 2, 1, 0);
	PatchSetPoint( &patch, P[1], 1, 2, 0);
    } /* if  */

    /* Do the st edge points */
    if (bc0) {
	P[2]=EdgeTangent( s, t );
	P[3]=EdgeTangent( t, s );
    } else {
	P[2]=PatchGetPoint (*patch0, 0, 2, 1);
	P[3]=PatchGetPoint (*patch0, 0, 1, 2);
	PatchSetPoint( &patch, P[2], 0, 2, 1);
	PatchSetPoint( &patch, P[3], 0, 1, 2);
    } /* if  */

    /* Do the tr edge points */
    if (bc1) {
	P[4]=EdgeTangent( t, r );
	P[5]=EdgeTangent( r, t );
    } else {
	P[4]=PatchGetPoint (*patch1, 1, 0, 2);
	P[5]=PatchGetPoint (*patch1, 2, 0, 1);
	PatchSetPoint( &patch, P[4], 1, 0, 2);
	PatchSetPoint( &patch, P[5], 2, 0, 1);
    } /* if  */
#else
	P[0]=PatchGetPoint (*patch2, 2, 1, 0);
	P[1]=PatchGetPoint (*patch2, 1, 2, 0);
	PatchSetPoint( &patch, P[0], 2, 1, 0);
	PatchSetPoint( &patch, P[1], 1, 2, 0);

	P[2]=PatchGetPoint (*patch0, 0, 2, 1);
	P[3]=PatchGetPoint (*patch0, 0, 1, 2);
	PatchSetPoint( &patch, P[2], 0, 2, 1);
	PatchSetPoint( &patch, P[3], 0, 1, 2);

	P[4]=PatchGetPoint (*patch1, 1, 0, 2);
	P[5]=PatchGetPoint (*patch1, 2, 0, 1);
	PatchSetPoint( &patch, P[4], 1, 0, 2);
	PatchSetPoint( &patch, P[5], 2, 0, 1);
#endif

    /* Set the middle point to the centroid of the edge points */
    w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
    w[6] = w[7] = w[8] = -(1.0/6.0);
    PatchSetPoint( &patch, PPacN( 9, P, w), 1, 1, 1);

    gp = AdjustPatch(patch);

    if ( gk ) {
      gkTessPatch(&(gp),EvalGregory,samples,StdFrame(World));
    } else {
      TessPatch(&(gp),EvalGregory,samples,StdFrame(World));
    }

    PatchFree(patch);
    PatchFree(gp.p);
}



#define J ((I+1)%3)
#define K ((I+2)%3)

/*
 *                                                                      
 *                     s=T2=030
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *                                                                      
 *      r=T1=300                     t=T3=003
 *                                                                      
 */
static GregoryPatch AdjustPatch(patch)
Patch patch;
{
  void ComputeCrossBoundary();
  GregoryPatch gp;
  Point T1,T2;
  Point V[3];
  Point C[3][3];
  Point Q[3][3];
  int I;

  gp.p = PatchDegreeRaise(patch); 

  V[0] = PatchGetPoint( patch, 3, 0, 0 );
  V[1] = PatchGetPoint( patch, 0, 3, 0 );
  V[2] = PatchGetPoint( patch, 0, 0, 3 );

  C[0][1] = PatchGetPoint (patch, 2, 1, 0);
  C[1][0] = PatchGetPoint (patch, 1, 2, 0);
  C[0][2] = PatchGetPoint (patch, 2, 0, 1);
  C[2][0] = PatchGetPoint (patch, 1, 0, 2);
  C[1][2] = PatchGetPoint (patch, 0, 2, 1);
  C[2][1] = PatchGetPoint (patch, 0, 1, 2);

  Q[0][1] = PatchGetPoint (gp.p, 3, 1, 0);
  Q[1][0] = PatchGetPoint (gp.p, 1, 3, 0);
  Q[0][2] = PatchGetPoint (gp.p, 3, 0, 1);
  Q[2][0] = PatchGetPoint (gp.p, 1, 0, 3);
  Q[1][2] = PatchGetPoint (gp.p, 0, 3, 1);
  Q[2][1] = PatchGetPoint (gp.p, 0, 1, 3);

  for( I=0; I<3; I++){
    if ( jensen != 0. ){
      Jensen( V[I], C[I][J], Q[I][K],  Q[J][K], C[J][I], V[J],
		      &(gp.pt[K][0]), &(gp.pt[K][1]), jensen);
    } else {
      ChiyokuraKimura( V[I], C[I][J], Q[I][K],  Q[J][K], C[J][I], V[J],
		      &(gp.pt[K][0]), &(gp.pt[K][1]),
		       V[I], V[I], 0); /* dummy arguments */
    }
  }
  return gp;
}


static Point EvalGregory(p,b)
GregoryPatch* p;
Scalar b[];
{
  Point P[3];
  int I;

  if ( b[0] == 0.  ||  b[1] == 0.  ||  b[2] == 0. ){
    return PatchEval(p->p,b[0],b[1],b[2]);
  }
  for(I=0; I<3; I++){
    P[I] = PPac( p->pt[J][1], p->pt[K][0], b[K]*(1.-b[J])/
		                     ( b[K]*(1.-b[J]) + b[J]*(1.-b[K]) ) );
  }
  PatchSetPoint(&(p->p), P[0], 2, 1, 1);
  PatchSetPoint(&(p->p), P[1], 1, 2, 1);
  PatchSetPoint(&(p->p), P[2], 1, 1, 2);
  return PatchEval(p->p,b[0],b[1],b[2]);
}



int newBoundary=0;

/*
 *  Function:  EdgeTangent
 *	Compute a point in the plane through the hyperplane described by 
 *  vertex.
 */
static Point EdgeTangent( v1, v2 )
Vertex *v1;
Vertex *v2;
{
  Point p1;
  Point p2;
  void FurthBoundary();

  if ( goodBoundary ) {
    if ( dBHS(v1,v2,&p1,&p2) ) {
      return p1;
    } else {
      Vector v;
      fprintf(stderr,"no dBHS boundary; using SS boundary\n");
      p1 = PPac(ReturnUDPoint(v1), 
		ReturnUDPoint(v2), 4.0/9.0);
      p2 = LineIntersectPlane(p1,NDual(ReturnUDNormal(v1)), 
			      ReturnUDPoint(v1),
			      ReturnUDNormal(v1));
      v = PPDiff(p2, ReturnUDPoint(v1));
      v = SVMult(VMag(PPDiff(ReturnUDPoint(v1),
			     ReturnUDPoint(v2)))/(3.0*VMag(v)), v);
      p2 = PVAdd(ReturnUDPoint(v1),v);
      return p2;
    }
  }
  if ( tweakBoundary ) {
    TweakBoundary(ReturnUDPoint(v1), 
		  ReturnUDNormal(v1),
		  ReturnUDPoint(v2), 
		  ReturnUDNormal(v2),
		  &p1, &p2);
    return p1;
  } else if ( furthBoundary ) {
    FurthBoundary(ReturnUDPoint(v1), 
		  ReturnUDNormal(v1),
		  ReturnUDPoint(v2), 
		  ReturnUDNormal(v2),
		  &p1, &p2);
    return p1;
  } 

  if ( !fourNinths ) 
    p1 = PPac(ReturnUDPoint(v1), 
	      ReturnUDPoint(v2), 2.0/3.0);
  else
    p1 = PPac(ReturnUDPoint(v1), 
	      ReturnUDPoint(v2), 4.0/9.0);
  p2 = LineIntersectPlane(p1,NDual(ReturnUDNormal(v1)), 
			  ReturnUDPoint(v1),
			  ReturnUDNormal(v1));

  if ( newBoundary ) {
    Vector v;
    v = PPDiff(p2, ReturnUDPoint(v1));
    v = SVMult(VMag(PPDiff(ReturnUDPoint(v1),
			   ReturnUDPoint(v2)))/(3.0*VMag(v)), v);
    p2 = PVAdd(ReturnUDPoint(v1),v);
  }
  return p2;
}



    




/*
 *----------------------------------------------------------------------
 *  Function:  TweakBoundary
 *----------------------------------------------------------------------
 */
static void TweakBoundary(p0, n0, p1, n1, r0, r1)
Point	p0, p1;
Normal	n0, n1;
Point *r0, *r1;
{
  Vector	u1, u2, u3;
  Vector v0, v1;
  Point pr;
  double nv,nv2;

  if ( (nv=VMag(VVCross(NDual(n0), NDual(n1)))) < 0.0000000001 ) {
/* not correct */fprintf(stderr,"TweakBoundary: assumed linear\n");
    *r0 = PPac(p0,p1,1.0/3.0);
    *r1 = PPac(p0,p1,2.0/3.0);
    return;
  }

  u1 = PPDiff(p1, p0);
  u3 = VVCross(u1, VVAdd(NDual(n0), NDual(n1)));

  /* make sure u1 not parallel to n0+n1 */
  if ( VMag( u3 ) < 0.000000001 ) {
    fprintf(stderr,"TweakBoundary: u1 and u3 parallel.  Using Furth.\n");
    FurthBoundary(p0, n0, p1, n1, r0, r1);
    return;
  }

  /* determine tangents at 0, 1 values along border curve i */
  /* pp0 stands for "p prime at value 0" */

  v0 = VVCross(NDual(n0), u3);
  v1 = VVCross(NDual(n1), u3);

  /* make sure v0 and v1 lie in tangent planes */
  /* SHOULD NEVER BE TRUE */
  if ( NVApply(n0,v0) > 0.0000000001  ||
       NVApply(n1,v1) > 0.0000000001 ) {
    fprintf(stderr,"TweakBoundary: Wierdness -- v0=v1 but don't lie in tangent planes!.  Calling FurthBoundary().\n");
    FurthBoundary(p0, n0, p1, n1, r0, r1);
  }

  /* special case: normals NOT parallel, but v0=v1 common to both */
  if ( (nv2=VMag(VVCross(v0, v1))) < 0.00000001 ) {
    Scalar x,y,z;
fprintf(stderr,"TweakBoundary: (2) assumed linear\n");
    NCoords(n0,StdFrame(SpaceOf(n0)),&x,&y,&z);
    fprintf(stderr,"n0 = (%g,%g,%g), ",x,y,z);

    NCoords(n1,StdFrame(SpaceOf(n1)),&x,&y,&z);
    fprintf(stderr,"n1 = (%g,%g,%g)\n",x,y,z);

    PCoords(p0,StdFrame(SpaceOf(p0)),&x,&y,&z);
    fprintf(stderr,"p0 = (%g,%g,%g), ",x,y,z);

    VCoords(v0,StdFrame(SpaceOf(v0)),&x,&y,&z);
    fprintf(stderr,"v0 = (%g,%g,%g), ",x,y,z);

    PCoords(p1,StdFrame(SpaceOf(p1)),&x,&y,&z);
    fprintf(stderr,"p1 = (%g,%g,%g), ",x,y,z);

    VCoords(v1,StdFrame(SpaceOf(v1)),&x,&y,&z);

    fprintf(stderr,"v1 = (%g,%g,%g).\n",x,y,z);

    *r0 = PPac(p0,p1,1.0/3.0);
    *r1 = PPac(p0,p1,2.0/3.0);

    return;
  }

  if ( !LineIntersectLine(p0,v0, p1,v1, &pr) ){
    Scalar x,y,z;

    fprintf(stderr,"TweakBoundary: non-planar lines! nv = %g,nv2=%g\n",nv,nv2);

    NCoords(n0,StdFrame(SpaceOf(n0)),&x,&y,&z);
    fprintf(stderr,"n0 = (%g,%g,%g), ",x,y,z);

    NCoords(n1,StdFrame(SpaceOf(n1)),&x,&y,&z);
    fprintf(stderr,"n1 = (%g,%g,%g)\n",x,y,z);

    PCoords(p0,StdFrame(SpaceOf(p0)),&x,&y,&z);
    fprintf(stderr,"p0 = (%g,%g,%g), ",x,y,z);

    VCoords(v0,StdFrame(SpaceOf(v0)),&x,&y,&z);
    fprintf(stderr,"v0 = (%g,%g,%g), ",x,y,z);

    PCoords(p1,StdFrame(SpaceOf(p1)),&x,&y,&z);
    fprintf(stderr,"p1 = (%g,%g,%g), ",x,y,z);

    VCoords(v1,StdFrame(SpaceOf(v1)),&x,&y,&z);
    fprintf(stderr,"v1 = (%g,%g,%g).\nCalling Furth boundaries\n",x,y,z);

    FurthBoundary(p0, n0, p1, n1, r0, r1);
    return;
  }

  *r0 = PPac(p0,pr,1.0-tweak);
  *r1 = PPac(p1,pr,1.0-tweak);

}




