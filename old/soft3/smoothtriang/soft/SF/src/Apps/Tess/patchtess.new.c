/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  patchtess.c
 *	Routines to tessellate a triangular Bezier patch.
 *----------------------------------------------------------------------
 */
#include <stdio.h>
#include "all.h"


#define MAX_DATA 5000

extern void (*OutputTriangle)();
extern int DoGaussian;
extern int DoMean;
extern int EdgeSamples;
extern int WhichFunctional;
extern Space World;

extern	Scalar	KGauss(), KMean();


static Scalar *gKv;	/* stores Gaussian curvature values at each vertex */
static Scalar gk1, gk2, gk3;
static Scalar *mKv;	/* stores Mean curvature values at each vertex */
static Scalar mk1, mk2, mk3;
static int First = 1;
static int KSpace = 0;


extern	SFF PatchEvalSFF();

/******************************************************************************
** GetPatchPrincipalK:
** Sets *k1 and *k2 to be the derivatives of principal curvature at (r,s,t)
** on patch *p.
******************************************************************************/

void	GetPatchPrincipalK(p, r, s, t, k1, k2)
Patch *p;
Scalar r, s, t, *k1, *k2;
    {
    SFF	sff;
    Vector	v1, v2;
    Scalar	s1, s2;

    sff = PatchEvalSFF(*p, r, s, t);
    ComputePrincipalVectors(sff, &v1, &v2, &s1, &s2);

    *k1 = s1;
    *k2 = s2;
    }


/******************************************************************************
/* PrincipalKFun:
/* Returns the sum of the principal curvatures at point x on f
******************************************************************************/

double	PrincipalKFun(f, r, s, t)
Patch *f;
double r, s, t;
    {
    Scalar	k1, k2, Result, x[3];

    x[0] = r;	x[1] = s;	x[2] = t;

    GetPatchPrincipalK(f, x[0], x[1], x[2], &k1, &k2);

    Result = (k1 * k1 ) + (k2 * k2);

    return Result;
    }


/******************************************************************************
** CGFun
** Evaluates the Celniker-Gossard functional (SIGGRAPH '91) at a single
** point x on face *f.
******************************************************************************/

double	CGPatchFun(f, r, s, t)
Patch *f;
double r, s, t;
    {
    double	Result, Dot1a, Dot1b, Dot1c, Dot2a, Dot2b, Dot2c;
    Vector	wu, wv, ww, wuw, www, wuv, wuu, wvv, u, v;
    double	ux[3], vx[3];
    double	a11, a12, a22, b11, b12, b22;
    double	x[3];

    x[0]= r;	x[1] = s;	x[2] = t;

    a11 = a12 = a22 = b11 = b12 = b22 = 1.0;

/*    ux[0] = 1.0;	ux[1] = 0.0;	ux[2] = -1.0; */
/*    vx[0] = 1.0;	vx[1] = -1.0;	vx[2] = 0.0; */

/*    ux[0] = 1.0;	ux[1] = -1.0;	ux[2] = 0.0; */
/*    vx[0] = 0.0;	vx[1] = -1.0;	vx[2] = 1.0; */

    ux[0] = 1.0;	ux[1] = -1.0;	ux[2] = 0.0;
    vx[0] = -0.5;	vx[1] = -0.5;	vx[2] = 1.0;

    u = VNormalize(VCreate(StdFrame(World), ux[0], ux[1], ux[2]));
    v = VNormalize(VCreate(StdFrame(World), vx[0], vx[1], vx[2]));

    VCoords(u, StdFrame(World), &ux[0], &ux[1], &ux[2]);
    VCoords(v, StdFrame(World), &vx[0], &vx[1], &vx[2]);

    wu = PatchDerivEval(*f, x[0], x[1], x[2], ux[0], ux[1], ux[2]);
    wv = PatchDerivEval(*f, x[0], x[1], x[2], vx[0], vx[1], vx[2]);
    wuu = PatchDeriv2Eval(*f, x[0], x[1], x[2], ux[0], ux[1], ux[2], ux[0], ux[1], ux[2]);
    wuv = PatchDeriv2Eval(*f, x[0], x[1], x[2], ux[0], ux[1], ux[2], vx[0], vx[1], vx[2]);
    wvv = PatchDeriv2Eval(*f, x[0], x[1], x[2], vx[0], vx[1], vx[2], vx[0], vx[1], vx[2]);

    Dot1a = VVDot(wu, wu);
    Dot1b = VVDot(wu, wv);
    Dot1c = VVDot(wv, wv);

/*fprintf(stderr, "\t\tDot1a = %lg, Dot1b = %lg, Dot1c = %lg\n", Dot1a,
Dot1b, Dot1c); */

    Dot2a = VVDot(wuu, wuu);
    Dot2b = VVDot(wuv, wuv);
    Dot2c = VVDot(wvv, wvv);

/*fprintf(stderr, "\t\tDot2a = %lg, Dot2b = %lg, Dot2c = %lg\n", Dot2a,
Dot2b, Dot2c); */

    Result = a11 * Dot1a + 2 * a12 * Dot1b + a22 * Dot1c;

/*fprintf(stderr, "\t\tResult 1 = %lg\n", Result); */

    Result += b11 * Dot2a + 2 * b12 * Dot2b + b22 * Dot2c;

/*fprintf(stderr, "\t\tResult 2 = %lg\n", Result); */

/*fprintf(stderr, "\tEval at (%lg, %lg, %lg): %lg\n", x[0], x[1], x[2],
Result); */

    return Result;
    }




/*
 *----------------------------------------------------------------------
 *  Function:  OutputPatchNet
 * Output (as SGP triangles) the control net of the given patch.
 *----------------------------------------------------------------------
 */
OutputPatchNet( fp, p)
FILE *fp;
Patch p;
{
  int i1, i2, i3;              /* Multi-indices summing to p.degree - 1 */
  int kup = p.degree - 1;
  int kdown = p.degree - 2;
  VERTEX v1, v2, v3;
  Lnode* copy=NULL;
  Lnode* lv1=NULL;
  Lnode* lv2=NULL;
  Lnode* lv3=NULL;
  Normal Nv;
  char s1[MAX_DATA],s2[MAX_DATA],s3[MAX_DATA];

  dCopyDstructFields(p.ln, "BezierTriangle", copy, "BezierTriangle");
  dDeleteField(copy, "BezierTriangle.net");
  dDeleteField(copy, "BezierTriangle.degree");
  dDeleteField(copy, "BezierTriangle.normaldim");

  /* Output the "up" triangular panels */
  for (i1 = 0; i1 <= kup; i1++) {
    for (i2 = 0; i2 <= kup - i1; i2++) {
      i3 = kup - i1 - i2;
      
      /* Pull out an upward panel of the control net */
      v1 = PatchGetVertex(p, i1+1,i2,i3);
      v2 = PatchGetVertex(p, i1,i2+1,i3),
      v3 = PatchGetVertex(p, i1,i2,i3+1);

      /* Fetch the curvature values, if set */

      if ((EdgeSamples != 0) && (DoGaussian))
         {
         gk1 = *(gKv + VertexIndex(i1+1, i2, i3));
         gk2 = *(gKv + VertexIndex(i1, i2+1, i3));
         gk3 = *(gKv + VertexIndex(i1, i2, i3+1));
	 }

      if ((EdgeSamples != 0) && (DoMean))
         {
         mk1 = *(mKv + VertexIndex(i1+1, i2, i3));
         mk2 = *(mKv + VertexIndex(i1, i2+1, i3));
         mk3 = *(mKv + VertexIndex(i1, i2, i3+1));
	 }
      
      /* Build a normal if necessary */
      if (p.normaldim == 0) {
	Nv = PPPNormal( v1.position, v2.position, v3.position);
	v1.normal = v2.normal = v3.normal = Nv;
      }

      /* Output the triangle */
      (*OutputTriangle)( fp, v1, v2, v3, copy);
    }
  }
  
  /* Output the "down" triangular panels */
  for (i1 = 0; i1 <= kdown; i1++) {
    for (i2 = 0; i2 <= kdown - i1; i2++) {
      i3 = kdown - i1 - i2;
      
      /* Pull out a downward panel of the control net */
      v1 = PatchGetVertex(p, i1,i2+1,i3+1);
      v2 = PatchGetVertex(p, i1+1,i2,i3+1),
      v3 = PatchGetVertex(p, i1+1,i2+1,i3);
      
      /* Fetch the curvature values, if set */

      if ((EdgeSamples != 0) && (DoGaussian))
         {
         gk1 = *(gKv + VertexIndex(i1, i2+1, i3+1));
         gk2 = *(gKv + VertexIndex(i1+1, i2, i3+1));
         gk3 = *(gKv + VertexIndex(i1+1, i2+1, i3));
	 }
      
      if ((EdgeSamples != 0) && (DoMean))
         {
         mk1 = *(mKv + VertexIndex(i1, i2+1, i3+1));
         mk2 = *(mKv + VertexIndex(i1+1, i2, i3+1));
         mk3 = *(mKv + VertexIndex(i1+1, i2+1, i3));
	 }
      
      /* Build a normal if necessary */
      if (p.normaldim == 0) {
	Nv = PPPNormal( v1.position, v2.position, v3.position);
	v1.normal = v2.normal = v3.normal = Nv;
      }
      
      /* Output the triangle */
      (*OutputTriangle)( fp, v1, v2, v3, copy);
    }
  }

  dDeleteLnode(copy);
}


/*
 *----------------------------------------------------------------------
 *  Function:  wireframeOutput
 *----------------------------------------------------------------------
 */
void wireframeOutput( fp, v1, v2, v3)
FILE *fp;
VERTEX v1, v2, v3;
{
  Scalar p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z;
  Frame F;
  
  F = StdFrame(SpaceOf( v1.position));
  
  PCoords( v1.position, F, &p1x, &p1y, &p1z);
  PCoords( v2.position, F, &p2x, &p2y, &p2z);
  PCoords( v3.position, F, &p3x, &p3y, &p3z);
  
  fprintf( fp, "%lg %lg %lg %lg %lg %lg\n",
	  p1x, p1y, p1z, p2x, p2y, p2z);
  fprintf( fp, "%lg %lg %lg %lg %lg %lg\n",
	  p1x, p1y, p1z, p3x, p3y, p3z);
  fprintf( fp, "%lg %lg %lg %lg %lg %lg\n",
	  p3x, p3y, p3z, p2x, p2y, p2z);
  
}


/*
 *----------------------------------------------------------------------
 *  Function:  g3dTriangleOutput
 *----------------------------------------------------------------------
 */
void g3dTriangleOutput( fp, v1, v2, v3, ds)
FILE *fp;
VERTEX v1, v2, v3;
Lnode* ds;
{
  static int count=0;
  Frame F;
  char outputString[1000];
  char Access[81], VertexStr[81], Field[81];
  Scalar x,y,z;
  
  F = StdFrame(SpaceOf( v1.position));

  PCoords(v1.position, F, &x,&y,&z);
  fprintf(fp, "%g %g %g m\n", x, y, z);
  NCoords(v1.normal, F, &x,&y,&z);
  fprintf(fp, "%g %g %g n\n", x, y, z);
  PCoords(v2.position, F, &x,&y,&z);
  fprintf(fp, "%g %g %g l\n", x, y, z);
  NCoords(v2.normal, F, &x,&y,&z);
  fprintf(fp, "%g %g %g n\n", x, y, z);
  PCoords(v3.position, F, &x,&y,&z);
  fprintf(fp, "%g %g %g l\n", x, y, z);
  NCoords(v3.normal, F, &x,&y,&z);
  fprintf(fp, "%g %g %g n\n", x, y, z);

  count++;
}



/*
 *----------------------------------------------------------------------
 *  Function:  sgpTriangleOutput
 *----------------------------------------------------------------------
 */
void sgpTriangleOutput( fp, v1, v2, v3, ds)
FILE *fp;
VERTEX v1, v2, v3;
Lnode* ds;
{
  static int count=0;
  Frame F;
  char outputString[1000];
  char Access[81], VertexStr[81], Field[81];
  
  F = StdFrame(SpaceOf( v1.position));

  dCopyDstructFields(ds,"BezierTriangle",dout,"Triangle");
/*if(count==130)fprintf(stderr,"creating tri 130\n");*/
  sprintf(outputString,"tri-%d",count);
  PutString("Triangle.name", outputString);
  dCopyDstructFields(ds,"BezierTriangle",dout,"Triangle.vertex1");
  PutVertexPosition("Triangle.vertex1",v1.position);
  PutVertexNormal("Triangle.vertex1",v1.normal);

  dCopyDstructFields(ds,"BezierTriangle",dout,"Triangle.vertex2");
  PutVertexPosition("Triangle.vertex2",v2.position);
  PutVertexNormal("Triangle.vertex2",v2.normal);

  dCopyDstructFields(ds,"BezierTriangle",dout,"Triangle.vertex3");
  PutVertexPosition("Triangle.vertex3",v3.position);
  PutVertexNormal("Triangle.vertex3",v3.normal);

   if ((EdgeSamples != 0) && (DoGaussian))
     {
     strcpy(VertexStr, "Triangle.vertex");

     strcpy(Field, "curvature.gaussian");

     sprintf(Access, "%s1.%s", VertexStr, Field);
     PutScalar(Access, gk1);

     sprintf(Access, "%s2.%s", VertexStr, Field);
     PutScalar(Access, gk2);

     sprintf(Access, "%s3.%s", VertexStr, Field);
     PutScalar(Access, gk3);
     }

   if ((EdgeSamples != 0) && (DoMean))
     {
     strcpy(VertexStr, "Triangle.vertex");

     strcpy(Field, "curvature.mean");

     sprintf(Access, "%s1.%s", VertexStr, Field);
     PutScalar(Access, mk1);

     sprintf(Access, "%s2.%s", VertexStr, Field);
     PutScalar(Access, mk2);

     sprintf(Access, "%s3.%s", VertexStr, Field);
     PutScalar(Access, mk3);
     }

  FlushDstruct();
  count++;
}




/*
 *----------------------------------------------------------------------
 *  Function:  TessellateBezierTriangle
 *----------------------------------------------------------------------
 */
void TessellateBezierTriangle(ThisPatch)
Patch ThisPatch;
{
  Patch SampledPatch;
  double r, s, t, step;
  int i1, i2, i3, i;
  Point pnt;
  Normal norm;
  Scalar x;
  extern int IsoLines;
  void LineBezierTriangle();
  extern Scalar offsetDist;
  extern int boundary;


  if ( IsoLines ){
    LineBezierTriangle(ThisPatch);
    return;
  }

  if ( boundary ) {
    OutputG3dBoundary(ThisPatch);
    return;
  }


  if (EdgeSamples != 0) {
    step = 1.0 / ((double) EdgeSamples);
  }
  
  if (EdgeSamples == 0) {

    /* Output the control net of the patch */
    OutputPatchNet( stdout, ThisPatch);
  } else {
    /* Create a new net by sampling the patch */
    SampledPatch = PatchCreate( EdgeSamples, World, 3);
    
    /* copy dstruct fields of ThisPatch */
    dCopyDstructFields(ThisPatch.ln,"BezierTriangle",
		       SampledPatch.ln,"BezierTriangle");


    /* only malloc space when needed */
    if (DoMean && (First || (ThisPatch.degree > KSpace))) {
      if ( !First ){
	free(mKv);
      }
      mKv = (Scalar *) malloc((NetSize(SampledPatch.degree)+1)*sizeof(Scalar));
      KSpace = SampledPatch.degree;
    }
    if (DoGaussian && (First || (SampledPatch.degree > KSpace)))	 {
      if ( !First ){
	free(gKv);
      }
      gKv = (Scalar *) malloc((NetSize(SampledPatch.degree)+1)*sizeof(Scalar));
      KSpace = SampledPatch.degree;
    }
    
    First = 0;
    
    /* Uniformly step sample in parameter space */
    for (i1 = 0, r = 0.0; i1 <= EdgeSamples; i1++, r += step) {
      for (i2 = 0, s = 0.0; i2 <= EdgeSamples - i1; i2++, s += step){
	i3 = EdgeSamples - i1 - i2;
	t  = 1.0 - r - s;
	PatchEvalWithNormal(ThisPatch, &pnt, &norm, r, s, t);
	norm = VDual(VNormalize(NDual(norm)));
	if ( offsetDist != 0. ) {
		pnt = PVAdd(pnt, SVMult(offsetDist,NDual(norm)));
	}
	PatchSetPoint( &SampledPatch, pnt, i1, i2, i3);
	PatchSetNormal( &SampledPatch, norm, i1, i2, i3);
	if (DoGaussian)	{
	  *(gKv + VertexIndex(i1, i2, i3)) = KGauss(ThisPatch, r, s, t);
	}
	if (DoMean)	{
	    switch (WhichFunctional)
		{
		case -1:
		    *(mKv + VertexIndex(i1, i2, i3)) = KMean(ThisPatch, r, s, t);
		    break;

		case 0:
		    *(mKv + VertexIndex(i1, i2, i3)) = CGPatchFun(&ThisPatch, r, s, t);
		    break;

		case 1:
		    *(mKv + VertexIndex(i1, i2, i3)) = PrincipalKFun(&ThisPatch, r, s, t);
		    break;
		
		default:
		    fprintf(stderr, "Tess: Unknown functional type\n");
		    break;
		}
	    

/*	  *(mKv + VertexIndex(i1, i2, i3)) = KMean(ThisPatch, r, s, t);

	  *(mKv + VertexIndex(i1, i2, i3)) = CGPatchFun(&ThisPatch, r, s, t);
*/
	}
      }
    }
    OutputPatchNet( stdout, SampledPatch);
    PatchFree( SampledPatch);
  }
}



/* Handle G3D Output */

void G3DMoveTo(p,n)
Point p;
Normal n;
{
  Scalar x,y,z;

  PCoords(p,StdFrame(SpaceOf(p)),&x,&y,&z);
  printf("%f %f %f 0\n",x,y,z);
  NCoords(n,StdFrame(SpaceOf(n)),&x,&y,&z);
  printf("%f %f %f n\n",x,y,z);
}


void G3DDrawTo(p,n)
Point p;
Normal n;
{
  Scalar x,y,z;

  PCoords(p,StdFrame(SpaceOf(p)),&x,&y,&z);
  printf("%f %f %f 1\n",x,y,z);
  NCoords(n,StdFrame(SpaceOf(n)),&x,&y,&z);
  printf("%f %f %f n\n",x,y,z);
}


void LineBezierTriangle(ThisPatch)
Patch ThisPatch;
{
  int i,j;
  Point p;
  Normal n;
  extern int EdgeSamples;
  double es; /* = EdgeSamples */
  double r,s,t;
  Scalar x,y,z;
  extern Scalar offsetDist;

  es = EdgeSamples;

  /* step with r constant */
  for(i=0;i<EdgeSamples;i++){
    r = (Scalar)i/es;
    s = 0.0;
    t = 1.0 -r;
    PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
    n = VDual(VNormalize(NDual(n)));
    if ( offsetDist != 0. ) {
	    p = PVAdd(p, SVMult(offsetDist,NDual(n)));
    }
    G3DMoveTo(p,n);
    for(j=1;j<=(EdgeSamples+2-i)*2;j++){
      s = (1.0-r) * (Scalar)j/(Scalar)((EdgeSamples+2-i)*2);
      t = 1.0-r-s;
      PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
      n = VDual(VNormalize(NDual(n)));
      if ( offsetDist != 0. ) {
	      p = PVAdd(p, SVMult(offsetDist,NDual(n)));
      }
      G3DDrawTo(p,n);
    }
  }

  /* step with s constant */
  for(i=0;i<EdgeSamples;i++){
    s = (Scalar)i/es;
    r = 0.0;
    t = 1.0 -s;
    PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
    n = VDual(VNormalize(NDual(n)));
    if ( offsetDist != 0. ) {
	    p = PVAdd(p, SVMult(offsetDist,NDual(n)));
    }
    G3DMoveTo(p,n);
    for(j=1;j<=(EdgeSamples+2-i)*2;j++){
      r = (1.0-s) * (Scalar)j/(Scalar)((EdgeSamples+2-i)*2);
      t = 1.0-s-r;
      PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
      n = VDual(VNormalize(NDual(n)));
      if ( offsetDist != 0. ) {
	      p = PVAdd(p, SVMult(offsetDist,NDual(n)));
      }
      G3DDrawTo(p,n);
    }
  }

  /* step with t constant */
  for(i=0;i<EdgeSamples;i++){
    t = (Scalar)i/es;
    s = 0.0;
    r = 1.0 -t;
    PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
    n = VDual(VNormalize(NDual(n)));
    if ( offsetDist != 0. ) {
	    p = PVAdd(p, SVMult(offsetDist,NDual(n)));
    }
    G3DMoveTo(p,n);
    for(j=1;j<=(EdgeSamples+2-i)*2;j++){
      s = (1.0-t) * (Scalar)j/(Scalar)((EdgeSamples+2-i)*2);
      r = 1.0-t-s;
      PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
      n = VDual(VNormalize(NDual(n)));
      if ( offsetDist != 0. ) {
	      p = PVAdd(p, SVMult(offsetDist,NDual(n)));
      }
      G3DDrawTo(p,n);
    }
  }

}


OutputG3dBoundary(pat)
Patch pat;
{
  int i;
  Point p;
  Normal n;
  Scalar x,y,z;
  extern Scalar offsetDist;

  for (i=0; i<EdgeSamples; i++) {
    PatchEvalWithNormal(pat, &p, &n, 0., (double)i/(double)EdgeSamples,
			(double)(EdgeSamples-i)/(double)EdgeSamples);
    n = VDual(VNormalize(NDual(n)));
    if ( offsetDist != 0. ) {
	    p = PVAdd(p, SVMult(offsetDist,NDual(n)));
    }
    if ( i==0 ) {
      G3DMoveTo(p, n);
    } else {
      G3DDrawTo(p, n);
    }
  }

  for (i=0; i<EdgeSamples; i++) {
    PatchEvalWithNormal(pat, &p, &n, (double)i/(double)EdgeSamples,
			(double)(EdgeSamples-i)/(double)EdgeSamples, 0.);
    n = VDual(VNormalize(NDual(n)));
    if ( offsetDist != 0. ) {
	    p = PVAdd(p, SVMult(offsetDist,NDual(n)));
    }
    G3DDrawTo(p, n);
  }

  for (i=0; i<EdgeSamples; i++) {
    PatchEvalWithNormal(pat, &p, &n, 
			(double)(EdgeSamples-i)/(double)EdgeSamples,
			0., (double)i/(double)EdgeSamples);
    n = VDual(VNormalize(NDual(n)));
    if ( offsetDist != 0. ) {
	    p = PVAdd(p, SVMult(offsetDist,NDual(n)));
    }
    G3DDrawTo(p, n);
  }
}
