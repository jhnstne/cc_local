/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Steve Mann
** 
** Purpose: An implementation of the surface interpolation
**	described in the paper by Shirman-Sequin, with the
**	following modification:
**		To compute the internal control points, we first
**		construct a single cubic patch w/quadratic precision.
**		We then split patch.  The crossboundary control points
**		are computed using Chiyokura-Kimura, but the Nij
**		control points are taken from the quadratic precision
**		patches.
**		To compute the rest of the internal cp, we average.
**
*/
#include <stdio.h>
#include <math.h>
#include "all.h"
#include "../../lib/KCurve/curve.h"
#include <sys/time.h>
#include <sys/resource.h>

/* Forward declarations */
void ProcessFace();
Point EdgeTangent();
void TweakBoundary();
void QuadricBoundary();
void AdjustPatch();
Point LineIntersectPlane();

#define LOOP 1
#define CENTROID 2
#define CROSS 3
#define SLOAN 4
#define BISECT 5
#define PERP 6

/* Global variables */
int normalType=LOOP;
int fourNinths=0;
int verbose=0;
int new=0;
int piperBoundary=0;
int furthBoundary=0;
int quadricBoundary=0;
int tweakBoundary=0;
int goodBoundary=0;
double tweak=0.5858;

int noOutput=0;
static char *RoutineName;
static Space World;                 /* Space where faces and patches reside */
static Scalar ShapeParameter = 0.25;/* Controls magnitude of edge tangents  */








/* centroid of cubic points */
double alpha1 = -.25;
double alpha2 = 2.25;
double alpha3 = -.75;
double alpha4 = 2.75;

/* centroid of quartic points 
double alpha1 = -1.0;
double alpha2 =  3.0;
double alpha3 = -.75;
double alpha4 = 2.75;
*/

double alpha;


main(argc, argv)
int argc;
char **argv;
{
  Mesh* m;
  Face* f;
  int num;

  /* Interpret command line options */
  ParseCommandLine( argc, argv);
  alpha = 1.0 - alpha1 -alpha2;

  /* Create the world space */
  World = SCreate( "World", 3);

  m = MeshParse(stdin);
  AddGeometry(World,m);

  /* If no normals, complain and quit. */
  if ( !(ReturnUDGeoFlags(m) & G_NORMAL) ) {
    fprintf(stderr,"NShirmanSequin: normals not given.  Exiting.\n");
    exit(1);
  } 

  /* Main Loop */
  num=0;
  ForeachMeshFace(m,f){
	
    if ( ! BoundaryFace(f) ) {
      if ( f->name != NULL ) 
	fprintf(stderr,"process face %s\n",f->name);
      ProcessFace(f);
    }
    num += 1;

  } EndForeach;
  if (verbose){
    struct rusage u;

    getrusage(RUSAGE_SELF, &u);
    fprintf(stderr,"NShirmanSequin: finished.  ");
    fprintf(stderr,"%d neighborhoods processed in %d cpu seconds.\n",
	    num, u.ru_utime);
  }
  exit(0);
}

int draw;

void ProcessFace( face )
Face* face;
{
    Patch patch;
    Patch pr,ps,pt;
    Point P[9];
    Scalar w[9];
    static int normalFlag = 0;
    Vertex *v, *vr[3];
    int i;
    Vertex* r;
    Vertex* s;
    Vertex* t;

    i = 0;
    ForeachFaceVertex(face,v){
      vr[i++] = v;
    } EndForeach;

    r = vr[0];
    s = vr[1];
    t = vr[2];

    /* Create the cubic patch */
    patch = PatchCreate( 3, World, 0);

    /* Set the corner points of the net to the vertices of the face */
    PatchSetPoint( &patch, ReturnUDPoint(r), 3, 0, 0); 
    P[6]=ReturnUDPoint(r);
    PatchSetPoint( &patch, ReturnUDPoint(s), 0, 3, 0); 
    P[7]=ReturnUDPoint(s);
    PatchSetPoint( &patch, ReturnUDPoint(t), 0, 0, 3); 
    P[8]=ReturnUDPoint(t);

draw = 1;
    /* Do the rs edge points */
    P[0]=EdgeTangent( r, s );
    P[1]=EdgeTangent( s, r );
    PatchSetPoint( &patch, P[0], 2, 1, 0);
    PatchSetPoint( &patch, P[1], 1, 2, 0);

    /* Do the st edge points */
    P[2]=EdgeTangent( s, t );
    P[3]=EdgeTangent( t, s );
    PatchSetPoint( &patch, P[2], 0, 2, 1);
    PatchSetPoint( &patch, P[3], 0, 1, 2);

    /* Do the tr edge points */
    P[4]=EdgeTangent( t, r );
    P[5]=EdgeTangent( r, t );
    PatchSetPoint( &patch, P[4], 1, 0, 2);
    PatchSetPoint( &patch, P[5], 2, 0, 1);

    /* Set the middle point to the centroid of the edge points */
    w[0] = w[1] = w[2] = w[3] = w[4] = w[5] = (1.0/4.0);
    w[6] = w[7] = w[8] = -(1.0/6.0);
    PatchSetPoint( &patch, PPacN( 9, P, w), 1, 1, 1);

    AdjustPatch(patch,&pr,&ps,&pt,r,s,t);

/*if ( noOutput != 0 ){*/
if ( draw ) {
    PatchWrite(stdout,pr);
    PatchWrite(stdout,ps);
    PatchWrite(stdout,pt);
}
/*  };
noOutput = 0;
*/

    PatchFree(patch);
    PatchFree(pr);
    PatchFree(ps);
    PatchFree(pt);
}



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
void AdjustPatch(patch,t1,t2,t3,r,s,t)
Patch patch;
Patch *t1;
Patch *t2;
Patch *t3;
Vertex* r;
Vertex* s;
Vertex* t;
{
  Point T1,T2,T3;
  Point B1,B2,B3;
  Point C1,C2,C3;
  Point D1,D2,D3;
  Point S1,S2,S3;
  Point S1a, S2a, S3a, S1b, S2b, S3b;
  Point SP1, SP2, SP3;
  Point L12,L13;
  Point M13,M23;
  Point K23,K12;
  Point N12,N13,N23;
  Point P1,P2,P3;
  Point Z;
  int n;
  Point p[10];
  Scalar sc[10];
  void ComputeCrossBoundary();
  Patch dpatch;
Point q[10];

  /* extract the known points from the cubic patch */
  T1 = PatchGetPoint(patch,3,0,0);
  T2 = PatchGetPoint(patch,0,3,0);
  T3 = PatchGetPoint(patch,0,0,3);
  B1 = PatchGetPoint(patch,2,0,1);
  B2 = PatchGetPoint(patch,1,0,2);
  C1 = PatchGetPoint(patch,2,1,0);
  C2 = PatchGetPoint(patch,1,2,0);
  D1 = PatchGetPoint(patch,0,1,2);
  D2 = PatchGetPoint(patch,0,2,1);

  dpatch = PatchDegreeRaise(patch);

  /* Contruct Three New Patches */
  PatchSplit(dpatch, t1, t2, t3, 1.0/3.0, 1.0/3.0, 1.0/3.0);
  PatchFree(dpatch);

  /* NOTE: The .25's come in because we need to convert from
     cubic to quartic control points */
  /* compute S's */
  S1 = PPac3( T1,             PPac(T1,B1,.25), PPac(T1,C1,.25), 
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
  S2 = PPac3( T2, PPac(T2,C2,.25), PPac(T2,D2,.25),
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
  S3 = PPac3( T3, PPac(T3,D1,.25), PPac(T3,B2,.25),
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);

/* BLEAH!  This will be a real mess! */
  S1a = PPac3(T1, 
	      PPac(T1, EdgeTangent(r,VertexPath(s,r,"-1")),.25),
	      PPac(T1,C1,.25), 
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
  S2a = PPac3(T2,
	      PPac(T2, EdgeTangent(s,VertexPath(t,s,"-1")),.25),
	      PPac(T2,D2,.25),
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
  S3a = PPac3( T3, 
	      PPac(T3, EdgeTangent(t,VertexPath(r,t,"-1")),.25),
	      PPac(T3,B2,.25),
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);

  S1b = PPac3(T1,             
	      PPac(T1,B1,.25), 
	      PPac(T1,EdgeTangent(r,VertexPath(t,r,"1")),.25), 
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
  S2b = PPac3(T2, 
	      PPac(T2,C2,.25), 
	      PPac(T2,EdgeTangent(s,VertexPath(r,s,"1")),.25),
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);
  S3b = PPac3(T3,
	      PPac(T3,D1,.25),
	      PPac(T3,EdgeTangent(t,VertexPath(s,t,"1")),.25),
	     -alpha1/alpha2,  -alpha/alpha2,   1.0/alpha2);


  /* compute K's, L's, and M's */
  /* rs boundary */
  ComputeCrossBoundary(T1,C1,PPac(T1,S1,.25), PPac(T2,S2,.25),C2,T2, &L12,&K12,
		       PPac(T1,S1a,.25), 
		       PPac(T2,S2b,.25));
  /* rt boundary */
  ComputeCrossBoundary(T1,B1,PPac(T1,S1,.25),PPac(T3,S3,.25),B2,T3,&L13,&M13,
		       PPac(T1,S1b,.25),
		       PPac(T3,S3a,.25));
  /* st boundary */
  ComputeCrossBoundary(T2,D2,PPac(T2,S2,.25),PPac(T3,S3,.25),D1,T3,&K23,&M23,
		       PPac(T2,S2a,.25),
		       PPac(T3,S3b,.25));




  /* Extract Si */
  S1 = PatchGetPoint( *t2, 3, 1, 0 );
  S2 = PatchGetPoint( *t3, 0, 3, 1 );
  S3 = PatchGetPoint( *t2, 0, 1, 3 );

  /* Extract Ni */
  N23 = PatchGetPoint( *t1, 2, 1, 1 );
  N13 = PatchGetPoint( *t2, 1, 2, 1 );
  N12 = PatchGetPoint( *t3, 1, 1, 2 );

  /* Calc SPi */
  SP1 = PPac3( S1, L13, L12, 1.0/3.0, 1.0/3.0, 1.0/3.0 );
  SP2 = PPac3( S2, K12, K23, 1.0/3.0, 1.0/3.0, 1.0/3.0 );
  SP3 = PPac3( S3, M13, M23, 1.0/3.0, 1.0/3.0, 1.0/3.0 );

  /* Calc quartic Pi */
  P1 = PPac3( SP1, N12, N13, 1.0/3.0, 1.0/3.0, 1.0/3.0 );
  P2 = PPac3( SP2, N12, N23, 1.0/3.0, 1.0/3.0, 1.0/3.0 );
  P3 = PPac3( SP3, N23, N13, 1.0/3.0, 1.0/3.0, 1.0/3.0 );


  Z = PPac3( P1, P2, P3, 1.0/3.0, 1.0/3.0, 1.0/3.0);
 

  /* PUT ALL POINTS INTO PATCHES */

  /* External Boundaries, points Si, and points Ni are 
     already in new patches */

  PatchSetPoint( t1, K23, 1, 2, 1 );
  PatchSetPoint( t1, M23, 1, 1, 2 );
  PatchSetPoint( t1, SP3, 2, 0, 2);
  PatchSetPoint( t1, SP2, 2, 2, 0);
  PatchSetPoint( t1, P3, 3, 0, 1 );
  PatchSetPoint( t1, P2, 3, 1, 0 );
  PatchSetPoint( t1, Z,  4, 0, 0 );

  PatchSetPoint( t2, L13, 2, 1, 1 );
  PatchSetPoint( t2, M13, 1, 1, 2 );
  PatchSetPoint( t2, SP1, 2, 2, 0 );
  PatchSetPoint( t2, SP3, 0, 2, 2 );
  PatchSetPoint( t2, P1, 1, 3, 0 );
  PatchSetPoint( t2, P3, 0, 3, 1 );
  PatchSetPoint( t2, Z,  0, 4, 0 );

  PatchSetPoint( t3, L12, 2, 1, 1 );
  PatchSetPoint( t3, K12, 1, 2, 1 );
  PatchSetPoint( t3, SP2, 0, 2, 2 );
  PatchSetPoint( t3, SP1, 2, 0, 2 );
  PatchSetPoint( t3, P2, 0, 1, 3 );
  PatchSetPoint( t3, P1, 1, 0, 3 );
  PatchSetPoint( t3, Z,  0, 0, 3 );

}


/*
 *----------------------------------------------------------------------
 *  Function:  ComputeCrossBoundary
 *----------------------------------------------------------------------
 */
void ComputeCrossBoundary(S0,S1,T0, T3,S2,S3, T1,T2,a0,a3)
Point S0, S1, T0, T3, S2, S3;
Point *T1, *T2;
Point a0,a3;
{
  Vector A0, A1, A2, A3;
  Vector B0, B1, B2, B3;
  Vector C0, C1, C2;
  Scalar h0,h1,k0,k1,z;
  Vector UnitNormPerpTo_InPlaneOf();
  Frame f;

  B0 = PPDiff(T0, S0);
  B3 = PPDiff(T3, S3);
  C0 = PPDiff(S1, S0);
  C1 = PPDiff(S2, S1);
  C2 = PPDiff(S3, S2);

  if ( new ) {
    A0 = PPDiff(T0,a0);
    A0 = SVMult(1.0/VMag(A0),A0);
    A3 = PPDiff(T3,a3);
    A3 = SVMult(1.0/VMag(A3),A3);
  } else {
    A0 = UnitNormPerpTo_InPlaneOf(C0, B0);
    A3 = UnitNormPerpTo_InPlaneOf(C2, B3);
  }
  A1 = VVAdd( SVMult( 2.0/3.0, A0 ),  SVMult( 1.0/3.0, A3 ) );
  A2 = VVAdd( SVMult( 1.0/3.0, A0 ),  SVMult( 2.0/3.0, A3 ) );

  /* extract h0,h1,k0,k1 */
  f = FCreate("f1",S0,A0,C0,VVCross(A0,C0));
  VCoords(B0,f,&k0,&h0,&z);
  f = FCreate("f1",S3,A3,C2,VVCross(A3,C2));
  VCoords(B3,f,&k1,&h1,&z);

  /* calc B1, B2 */
  B1 = VVAdd( VVAdd( SVMult((k1-k0)/3.0,A0), SVMult(k0, A1)),
	      VVAdd( SVMult(2.0*h0/3.0, C1), SVMult(h1/3.0,C0)));
  B2 = VVAdd( VVAdd( SVMult(k1,A2), SVMult((k0-k1)/3.0,A3)),
	      VVAdd( SVMult(h0/3.0,C2), SVMult(2.0*h1/3.0, C1)));


  *T1 = PVAdd(S1, B1);
  *T2 = PVAdd(S2, B2);

}


Vector UnitNormPerpTo_InPlaneOf(V1, V2)
Vector V1, V2;
{
  Vector R;

  R = VVCross(V1,V2);
  R = VVCross(R,V1);
  R = SVMult(1.0/VMag(R), R);
  return R;
}



int newBoundary=0;

/*
 *  Function:  EdgeTangent
 *	Compute a point in the plane through the hyperplane described by 
 *  vertex.
 */
Point EdgeTangent( v1, v2 )
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
draw = 0;
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
  } else if ( quadricBoundary ) {
    QuadricBoundary(ReturnUDPoint(v1), 
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
  } else if ( piperBoundary ) {
    double l,vl;
    Vector v;

    v = PPDiff(p2,ReturnUDPoint(v1));
    vl = VMag(v);
    l = (VMag(PPDiff(p2,ReturnUDPoint(v1))))/9.0 + vl;
    p2 = PVAdd(ReturnUDPoint(v1), SVMult(l/vl, v));
  }
  return p2;
}



    

/*
 *  Function:  FurthBoundary
 *	Sets tangents at either end of a single border curve, according to the
 *  notation on p. 9 of Furth
 */
void FurthBoundary(p0, n0, p1, n1, r0, r1)
Point	p0, p1;
Normal	n0, n1;
Point *r0, *r1;
{
  Vector	u1, u2, u3;
  Vector v0, v1;
  
  u1 = VNormalize(VVDiff(p1, p0));
  u3 = VNormalize(VVCross(u1, VVAdd(NDual(n0), NDual(n1))));
  u2 = VNormalize(VVCross(u3, u1));
  
  /* determine tangents at 0, 1 values along border curve i */
  /* pp0 stands for "p prime at value 0" */
  
  v0 = SVMult(VMag(PPDiff(p1, p0)), VNormalize(VVCross(NDual(n0), u3)));
  v1 = SVMult(VMag(PPDiff(p1, p0)), VNormalize(VVCross(NDual(n1), u3)));
  
  if (VVDot(VVCross(NDual(n1), u3), u1) < 0.0) {
    v0 = SVMult(-1.0, v0);
  } else {
    v1 = SVMult(-1.0, v1);
  }
  *r0 = PVAdd(p0, SVMult(1.0/3.0, v0));
  *r1 = PVAdd(p1, SVMult(1.0/3.0, v1));

}



/*
 *----------------------------------------------------------------------
 *  Function:  QuadricBoundary
 *----------------------------------------------------------------------
 */
void QuadricBoundary(p0, n0, p1, n1, r0, r1)
Point	p0, p1;
Normal	n0, n1;
Point *r0, *r1;
{
  Vector	u1, u2, u3;
  Vector v0, v1;
  Point pr;
  double nv,nv2;

  if ( (nv=VMag(VVCross(NDual(n0), NDual(n1)))) < 0.0000000001 ) {
/* not correct */fprintf(stderr,"Quadratic Boundary: assumed linear\n");
    *r0 = PPac(p0,p1,1.0/3.0);
    *r1 = PPac(p0,p1,2.0/3.0);
    return;
  }

  u1 = VVDiff(p1, p0);
  u3 = VVCross(u1, VVAdd(NDual(n0), NDual(n1)));

  /* make sure u1 not parallel to n0+n1 */
  if ( VMag( u3 ) < 0.000000001 ) {
    fprintf(stderr,"QuadricBoundary: u1 and u3 parallel.  Using Furth.\n");
    FurthBoundary(p0, n0, p1, n1, r0, r1);
    return;
  }

  /* determine tangents at 0, 1 values along border curve i */
  /* pp0 stands for "p prime at value 0" */
  
  v0 = VVCross(NDual(n0), u3);
  v1 = VVCross(NDual(n1), u3);

  /* make sure v0 and v1 lie in tangent planes */
  /* SHOULD NEVER BE TRUE */
  if ( fabs(NVApply(n0,v0)) > 0.0000000001  ||
       fabs(NVApply(n1,v1)) > 0.0000000001 ) {
    fprintf(stderr,"QuadricBoundary: Wierdness -- v0=v1 but don't lie in tangent planes!.  Calling FurthBoundary().\n");
    FurthBoundary(p0, n0, p1, n1, r0, r1);
    return ;
  }

  /* special case: normals NOT parallel, but v0=v1 common to both */
  if ( (nv2=VMag(VVCross(v0, v1))) < 0.00000001 ) {
    Scalar x,y,z;
fprintf(stderr,"Quadratic Boundary: (2) assumed linear\n");
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
noOutput++;
    return;
  }

  if ( !LineIntersectLine(p0,v0, p1,v1, &pr) ){
    Scalar x,y,z;

    fprintf(stderr,"QuadricBoundary: non-planar lines! nv = %g,nv2=%g\n",nv,nv2);

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

  *r0 = PPac(p0,pr,1.0/3.0);
  *r1 = PPac(p1,pr,1.0/3.0);

}
/*
 *----------------------------------------------------------------------
 *  Function:  TweakBoundary
 *----------------------------------------------------------------------
 */
void TweakBoundary(p0, n0, p1, n1, r0, r1)
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

  u1 = VVDiff(p1, p0);
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
noOutput++;
    return;
  }

  if ( !LineIntersectLine(p0,v0, p1,v1, &pr) ){
    Scalar x,y,z;

    fprintf(stderr,"QuadricBoundary: non-planar lines! nv = %g,nv2=%g\n",nv,nv2);

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







    


char* Banner = "ShirmanSequin";
char* UsageString = "ShirmanSequin [option] < mesh ";

void SetAlpha(a)
char* a[];
{
  alpha1 = atof(a[0]);
  alpha2 = atof(a[1]);
  if ( alpha1 + alpha2 != alpha3 + alpha4 ) {
    fprintf(stderr,"Bad alpha values.  Exiting.\n");
    exit(1);
  }
}

void SetFour()
{
  fourNinths = 1;
}

extern void Usage();

void Verbose()
{
  verbose = 1;
}


void New()
{
  new = 1;
}

void NewBoundary()
{
  newBoundary=1;
}

void SetFurthBoundary()
{
  furthBoundary = 1;
}

void PiperBoundary()
{
  piperBoundary = 1;
}


void SetQuadricBoundary()
{
  quadricBoundary = 1;
}

void SetTweakBoundary(a)
char* a[];
{
  tweakBoundary=1;
  tweak = atof(a[0]);
fprintf(stderr,"Tweak = %f\n",tweak);
}


void SetGoodBoundary()
{
  goodBoundary=1;
}


Option Options[] = {
/* Name, 	Routine,	#args,	Help String */
   "h",		Usage,		0,	": 	Print available options.",
   "a",		SetAlpha,	2,	"alpha1 alpha2: Set alpha values.\n\
		alpha1+alpha2 must equal 2.0.",
   "4",		SetFour,	0,	":	Set edge tangents to be 4/9.",
   "v",		Verbose,	0,	":	Verbose mode.",
   "n",		New,		0,	":	New method of continuity cond.",
   "nb",	NewBoundary,    0,      ":	New method of computing boundary curves.",
   "fb",	SetFurthBoundary,0,     ":	Use Furth Boundaries.",
   "pb",	PiperBoundary,  0,      ":	Use Piper Boundaries.",
   "qb",	SetQuadricBoundary,0,	":	Use Quadric Boundaries.",
   "tb",	SetTweakBoundary,  1,	"tweak:	Use 'tweak' frac of dist from\
 vertices for boundary.",
   "gb",	SetGoodBoundary, 0,	":	Use deBoor, Hollig, Sabin boundaries.",

   NULL,	NULL,		0,	NULL
   };
