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
#include <sys/time.h>
#include <sys/resource.h>
#include "all.h"

typedef struct {
  Patch p;
  Point b[3][4];
  Vector cb[3][4];
  Vector mp[3][2];
  Point pt[3][2];
} GregoryPatch;

/* Forward declarations */
void ProcessFace();
Point EdgeTangent();
void TweakBoundary();
Point LineIntersectPlane();
GregoryPatch AdjustPatch();
Point EvalGregory();

/* Global variables */
int fourNinths=0;
int verbose=0;
int new=0;
int furthBoundary=0;
int tweakBoundary=0;
int goodBoundary=0;
int davidchukBoundary = 0;
double tweak=0.5858;
int samples=2;
int gk=0;
Scalar jensen=0.;

static char *RoutineName;
static Space World;                 /* Space where faces and patches reside */
static Space World2;                 /* Space where faces and patches reside */
static Scalar ShapeParameter = 0.25;/* Controls magnitude of edge tangents  */







main( int argc, char **argv)
{
  Mesh* m;
  Face* f;
  int num;

  /* Interpret command line options */
  ParseCommandLine( argc, argv);

  /* Create the world space */
  World = SCreate( "World", 3);
  World2 = SCreate( "World2", 2);

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
      if ( verbose  &&  f->name != NULL ) 
	fprintf(stderr,"process face %s\n",f->name);
      ProcessFace(f);
    }
    num += 1;

  } EndForeach;
  if (verbose){
    struct rusage u;

    getrusage(RUSAGE_SELF, &u);
    fprintf(stderr,"GregoryCharrot: finished.  ");
    fprintf(stderr,"%d neighborhoods processed in %d cpu seconds.\n",
	    num, u.ru_utime);
  }
  exit(0);
}


void ProcessFace( Face* face )
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
    GregoryPatch gp;

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

GregoryPatch AdjustPatch( Patch patch)
{
  GregoryPatch gp;
  Point V[3];		/* Corner vertices */
  Point C[3][3];	/* Cubic boundary control points */
  Point Q[3][3];	/* Quartic boundary control points */
  int I;

  gp.p = PatchDegreeRaise(patch);

  V[0] = PatchGetPoint( patch, 3, 0, 0 );
  V[1] = PatchGetPoint( patch, 0, 3, 0 );
  V[2] = PatchGetPoint( patch, 0, 0, 3 );

  /* Point C[I][J] represents the bcp between V_I and V_J closer to I */
  C[0][1] = PatchGetPoint (patch, 2, 1, 0);
  C[1][0] = PatchGetPoint (patch, 1, 2, 0);
  C[0][2] = PatchGetPoint (patch, 2, 0, 1);
  C[2][0] = PatchGetPoint (patch, 1, 0, 2);
  C[1][2] = PatchGetPoint (patch, 0, 2, 1);
  C[2][1] = PatchGetPoint (patch, 0, 1, 2);

  /* Point Q[I][J] represents the bcp between V_I and V_J closest to I */
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

    /* Set up boundary */
    gp.b[K][0] = V[I];
    gp.b[K][1] = C[I][J];
    gp.b[K][2] = C[J][I];
    gp.b[K][3] = V[J];

    /* Set up cross boundary */
    gp.cb[K][0] = SVMult(4., PPDiff(Q[I][K], gp.b[K][0]));
    gp.cb[K][1] = SVMult(4., PPDiff(gp.pt[K][0], gp.b[K][1]));
    gp.cb[K][2] = SVMult(4., PPDiff(gp.pt[K][1], gp.b[K][2]));
    gp.cb[K][3] = SVMult(4., PPDiff(Q[J][K], gp.b[K][3]));

    /* Set up mixed partials */
    gp.mp[K][0] = SVMult(12., VVDiff( PPDiff(gp.pt[K][0], Q[I][J]), 
				      PPDiff(Q[I][K], V[I]) ));
    gp.mp[K][1] = SVMult(12., VVDiff( PPDiff(gp.pt[K][1], Q[J][I]), 
				      PPDiff(Q[J][K], V[J]) ));

  }
  return gp;
}

Vector EvalTangent( Point p[], int n, Scalar t)
{
  int i;
  int j;
  Point pt[10];

  if ( n > 9 ) {
    fprintf(stderr,"EvalTangent: degree too high. Exiting.\n");
    exit(1);
  }
  for(i=0; i<=n; i++) {
    pt[i] = p[i];
  }

  for(i=n; i>1; i--) {
    for(j=0; j<i; j++) {
      pt[j] = PPac(pt[j], pt[j+1], 1-t);
    }
  }
  return SVMult( (Scalar)n, PPDiff(pt[1], pt[0]) );
}

Point EvalPointCurve( Point p[], int n, Scalar t)
{
  int i;
  int j;
  Point pt[10];

  if ( n > 9 ) {
    fprintf(stderr,"EvalPointCurve: degree too high. Exiting.\n");
    exit(1);
  }
  for(i=0; i<=n; i++) {
    pt[i] = p[i];
  }

  for(i=n; i>0; i--) {
    for(j=0; j<i; j++) {
      pt[j] = PPac(pt[j], pt[j+1], 1-t);
    }
  }
  return pt[0];
}

Vector EvalVectorCurve( Vector v[], int n, Scalar t)
{
  int i;
  int j;
  Vector vec[10];

  if ( n > 9 ) {
    fprintf(stderr,"EvalPointCurve: degree too high. Exiting.\n");
    exit(1);
  }
  for(i=0; i<=n; i++) {
    vec[i] = v[i];
  }

  for(i=n; i>0; i--) {
    for(j=0; j<i; j++) {
      vec[j] = VVAdd(SVMult(1-t, vec[j]),
		     SVMult(t, vec[j+1]));
    }
  }
  return vec[0];
}

/*
 *----------------------------------------------------------------------
 *  Function:  ComputeCB
 *	Given a crossboundary field where the domain varies as an
 *  affine combo of two domain vectors, aV + bW, compute the cross boundary
 *  vector relative to the domain vector AV + BW, where A = a^2/(a^2+b^2)
 *  and B = b^2/(a^2+b^2).
 *----------------------------------------------------------------------
 */
Vector ComputeCB( Vector cb[4], Scalar t, Vector tan)
{
  Vector v;
  Vector w;
  Vector x;
  Vector cb1;
  Vector cb2;
  Frame f;
  Scalar a, b;
  Scalar denom;

  denom = (1.-t)*(1.-t) + t*t;

  v = FV( StdFrame(World2), 0 );
  x = FV( StdFrame(World2), 1 );
  w = VVDiff( v, x );
  cb1 = VVAdd( SVMult((1.-t), v), SVMult(t, w) );
  cb2 = VVAdd( SVMult((1.-t)*(1.-t)/denom, v), SVMult(t*t/denom, w) );
  f = FCreate( "CB", FOrg(StdFrame(World2)), cb1, x );
  VCoords( cb2, f, &a, &b );
  cb1 = EvalVectorCurve(cb, 3, t);
  cb2 = VVAdd( SVMult(a, cb1), SVMult(b, tan) );
  return cb2;
}


Point EvalSubpatch( GregoryPatch* p, int I, Scalar b[3])
{
  Vector v[7];
  Scalar w[7];
  Vector tan;

  tan = EvalTangent(p->b[J], 3, 1.-b[K]);
  v[0] = ComputeCB(p->cb[J], 1.-b[K], tan);
  w[0] = b[J];
  tan = EvalTangent(p->b[K], 3, b[J]);
  v[1] = ComputeCB(p->cb[K], b[J], tan);
  w[1] = b[K];
  v[2] = SVMult(3.0*b[J], PPDiff(p->b[K][1], p->b[K][0]));
  w[2] = -1.0;
  v[3] = SVMult(3.0*b[K], PPDiff(p->b[J][2], p->b[J][3]));
  w[3] = -1.0;
  v[4] = SVMult(b[K]/(b[J]+b[K]), p->mp[K][0]);
  w[4] = -b[K]*b[J];
  v[5] = SVMult(b[J]/(b[J]+b[K]), p->mp[J][1]);
  w[5] = -b[K]*b[J];

  /* We have three points, two with +1 coefficient, one with -1; arbitrarily
     choose one of the +1's to diff with the -1 */
  v[6] = PPDiff(EvalPointCurve(p->b[K], 3, b[J]),  p->b[K][0]);
  w[6] = 1.0;

  return PVAdd(EvalPointCurve(p->b[J], 3, (1-b[K])), VVlcN(7, v, w));
}

/*
 * EvalGregory
 *
 * Evaluate normal Gregory patch
 *     or
 * if the 'Davidchuk' option is on evaluate a Gregory patch
 * with Matthew Davidchuk's premium quality rationally blended 
 * boundary control points.
 */
Point EvalGregory( GregoryPatch* p, Scalar b[3])
{
    if (!davidchukBoundary) {
	Point P[3];
	Scalar w[3];
	Scalar denom;
	int I;

	if ( b[0]+b[1] == 0.  ||  b[1]+b[2] == 0.  ||  b[2]+b[0] == 0. ){
	    return PatchEval(p->p,b[0],b[1],b[2]);
	}
	for(I=0; I<3; I++){
	    P[I] = EvalSubpatch(p, I, b);
	}

	denom = b[0]*b[0] + b[1]*b[1] + b[2]*b[2];

	w[0] = b[0]*b[0]/denom;
	w[1] = b[1]*b[1]/denom;
	w[2] = b[2]*b[2]/denom;

	return PPacN(3, P, w);
    } else {
	fprintf (stderr, "Chuk boundaries not implemented!\n");
	exit(1);
    } /* if */
}



int newBoundary=0;

/*
 *  Function:  EdgeTangent
 *	Compute a point in the plane through the hyperplane described by 
 *  vertex.
 */
Point EdgeTangent( Vertex *v1, Vertex *v2)
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







    


char* Banner = "GregoryCharrot";
char* UsageString = "GregoryCharrot [option] < mesh ";

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


void SetTweakBoundary( char* a[])
{
  tweakBoundary=1;
  tweak = atof(a[0]);
fprintf(stderr,"Tweak = %f\n",tweak);
}


void SetGoodBoundary()
{
  goodBoundary=1;
}

void SetSamples(arg)
char* arg[];
{
  samples = atoi(arg[0]);
}

void SetGaussian()
{
  gk = 1;
}

void SetJensen(argv)
char* argv[];
{
  jensen = atof(argv[0]);
}

Option Options[] = {
/* Name, 	Routine,	#args,	Help String */
   "h",		Usage,		0,	": 	Print available options.",
   "4",		SetFour,	0,	":	Set edge tangents to be 4/9.",
   "v",		Verbose,	0,	":	Verbose mode.",
   "n",		New,		0,	":	New method of continuity cond.",
   "nb",	NewBoundary,    0,      ":	New method of computing boundary curves.",
   "fb",	SetFurthBoundary,0,     ":	Use Furth Boundaries.",
   "tb",	SetTweakBoundary,  1,	"tweak:	Use 'tweak' frac of dist from\
 vertices for boundary.",
   "dbhs",	SetGoodBoundary, 0,	":	Use deBoor, Hollig, Sabin boundaries.",
   "s",		SetSamples,	1,	"s:	Set the tesselation level.",
   "gk",	SetGaussian,	0,	":	Sample for Gaussian Curvature.",
   "j",		SetJensen,	1,	"c:	Use Jensen crossboundaries.",

   NULL,	NULL,		0,	NULL
   };
