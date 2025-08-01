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
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "commandline.h"
#include "usage.h"
#include "patch.h"
#include "ratpatch.h"
#include "sff.h"
#include "libgeo.h"
#include "tpbezier.h"



#define MAX_DATA 5000

extern void (*OutputTriangle)();
extern int DoGaussian;
extern int DoMean;
extern int EdgeSamples;
extern Space World;

extern int s3dWidth;
extern Scalar offsetDist;

extern	Scalar	KGauss(), KMean();


static Scalar *gKv;	/* stores Gaussian curvature values at each vertex */
static Scalar gk1, gk2, gk3;
static Scalar *mKv;	/* stores Mean curvature values at each vertex */
static Scalar mk1, mk2, mk3;
static int First = 1;
static int KSpace = 0;


/* set nf to non-zero if you want the normal */
void OutputS3dVertex(FILE* fp, VERTEX v, int nf)
{
	Scalar x,y,z;
	Frame f;

	f = StdFrame(SpaceOf(v.position));

	if ( nf ) {
		NCoords(v.normal, f, &x,&y,&z);
		fprintf(fp, "n %lg %lg %lg\n", x,y,z);
	}
	PCoords(v.position, f, &x,&y,&z);
	fprintf(fp, "v %lg %lg %lg\n", x,y,z);
}


void OutputS3dFace(FILE* fp, int i, int j, int k)
{
	fprintf(fp, "f %d %d %d\n",i,j,k);
}


void OutputS3dNet(FILE* fp, Patch p)
{
	int i,j,k;
	VERTEX v;
	extern Scalar s3dBoundaries;

	if ( s3dBoundaries > 0 ) {
		fprintf(fp, "d .58 .19 .07\n");
		fprintf(fp, "s .4 .4 .4\n");
		fprintf(fp, "g 60 0 0\n");
		fprintf(fp, "o 0 0 0\n");
	}


	fprintf(fp, 
		"M %d %d 0\n",
		((p.degree+2)*(p.degree+1))/2,
		p.degree*p.degree);
	
	for (i = 0; i <= p.degree; i++) {
		for (j = 0; j <= p.degree - i; j++) {
			k = p.degree - i - j;
			v = PatchGetVertex(p, i,j,k);
			if ( DoGaussian ) {
				Material m;

				m = KMat(gKv[VertexIndex(i,j,k)], -4.,4.,.625);
				fprintf(fp, "d %g %g %g\n",
					m.diffuse.r, m.diffuse.g, m.diffuse.b);
				/* fprintf(fp, "# gk %f\n", 
					gKv[VertexIndex(i,j,k)]);*/
			}
			OutputS3dVertex(fp, v, 1);
		}
	}

	for (i = 1; i <= p.degree; i++) {
		for (j = 0; j <= p.degree - i; j++) {
			k = p.degree - i - j;
			OutputS3dFace(fp, 
				      VertexIndex(i,j,k),
				      VertexIndex(i-1,j+1,k),
				      VertexIndex(i-1,j,k+1));
		}
	}
	for (i = 1; i < p.degree; i++) {
		for (j = 1; j <= p.degree - i; j++) {
			k = p.degree - i - j;
			OutputS3dFace(fp, 
				      VertexIndex(i,j,k),
				      VertexIndex(i-1,j,k+1),
				      VertexIndex(i,j-1,k+1));
		}
	}

	fprintf(fp, "E 0 0 0\n\n");

	if ( s3dBoundaries > 0 ) {
		fprintf(fp, "d 0 0 0\n");
		fprintf(fp, "s 0 0 0\n");
		fprintf(fp, "g 1 0 0\n");
		fprintf(fp, "o 0 1 0\n");

		fprintf(fp, "L %d %d 0\n", 3*p.degree+1, s3dWidth);
		for (i=0; i<=p.degree; i++) {
			v = PatchGetVertex(p, i,p.degree-i,0);
			v.position = PVAdd(v.position, 
					   SVMult(s3dBoundaries, 
						  VNormalize(NDual(v.normal))));
			OutputS3dVertex(fp, v, 0);
		}
		for (i=1; i<=p.degree; i++) {
			v = PatchGetVertex(p, p.degree-i,0,i);
			OutputS3dVertex(fp, v, 0);
		}
		for (i=1; i<=p.degree; i++) {
			v = PatchGetVertex(p, 0,i,p.degree-i);
			OutputS3dVertex(fp, v, 0);
		}
		fprintf(fp, "E 0 0 0\n\n\n");
	}
}

static void OutputS3dVertexNormal(FILE* fp, VERTEX v, Scalar len)
{
	Frame f;
	Scalar x,y,z;
	Scalar nx,ny,nz;

	f = StdFrame(SpaceOf(v.position));

	fprintf(fp, "L 2 %d 0\n", s3dWidth);
		
	PCoords(v.position, f, &x,&y,&z);
	VCoords(VNormalize(NDual(v.normal)), f, &nx,&ny,&nz);

	fprintf(fp, "v %lg %lg %lg\n", x,y,z);
	fprintf(fp, "v %lg %lg %lg\n", x+len*nx, y+len*ny, z+len*nz);

	fprintf(fp, "E 0 0 0\n\n");
}


static void OutputS3dCBNormals(FILE* fp, Patch p, Scalar len)
{
	VERTEX v;
	int i,j;

	for (i=0; i<p.degree; i++) {
		v = PatchGetVertex(p, p.degree-i, 0, i);
		OutputS3dVertexNormal(fp, v, len);

		v = PatchGetVertex(p, 0, p.degree-i, i);
		OutputS3dVertexNormal(fp, v, len);

		v = PatchGetVertex(p, p.degree-i, i, 0);
		OutputS3dVertexNormal(fp, v, len);
	}
}


void OutputIvNet(FILE* fp, Patch p)
{
	int i,j,k;
	VERTEX v;
	Frame f;
	Scalar x,y,z;

	fprintf(fp, "Separator {\n");

	fprintf(fp, "  Coordinate3 {\n");
	fprintf(fp, "    point [\n");
	for (i = 0; i <= p.degree; i++) {
		for (j = 0; j <= p.degree - i; j++) {
			k = p.degree - i - j;
			v = PatchGetVertex(p, i,j,k);
			f = StdFrame(SpaceOf(v.position));
			PCoords(v.position, f, &x,&y,&z);
			fprintf(fp, "      %lg %lg %lg,\n", x,y,z);
		}
	}
	fprintf(fp, "    ]\n");
	fprintf(fp, "  }\n");

	fprintf(fp, "  Normal {\n");
	fprintf(fp, "    vector [\n");
	for (i = 0; i <= p.degree; i++) {
		for (j = 0; j <= p.degree - i; j++) {
			k = p.degree - i - j;
			v = PatchGetVertex(p, i,j,k);
			f = StdFrame(SpaceOf(v.position));
			NCoords(v.normal, f, &x,&y,&z);
			fprintf(fp, "      %lg %lg %lg,\n", x,y,z);
		}
	}
	fprintf(fp, "    ]\n");
	fprintf(fp, "  }\n");

	fprintf(fp, "  IndexedFaceSet {\n");
	fprintf(fp, "    coordIndex [\n");
	for (i = 1; i <= p.degree; i++) {
		for (j = 0; j <= p.degree - i; j++) {
			k = p.degree - i - j;
			fprintf(fp, "      %d, %d, %d, -1,\n",
				      VertexIndex(i,j,k),
				      VertexIndex(i-1,j+1,k),
				      VertexIndex(i-1,j,k+1));
		}
	}
	for (i = 1; i < p.degree; i++) {
		for (j = 1; j <= p.degree - i; j++) {
			k = p.degree - i - j;
			fprintf(fp, "      %d, %d, %d, -1,\n",
				      VertexIndex(i,j,k),
				      VertexIndex(i-1,j,k+1),
				      VertexIndex(i,j-1,k+1));
		}
	}
	fprintf(fp, "    ]\n");
	fprintf(fp, "  }\n");
	fprintf(fp, "}\n\n");
}

/*
 *----------------------------------------------------------------------
 *  Function:  OutputPatchNet
 * Output (as SGP triangles) the control net of the given patch.
 *----------------------------------------------------------------------
 */
OutputPatchNet(FILE *fp, Patch p)
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
  extern int s3d;
  extern int iv;

  if ( s3d ) {
	  extern Scalar cblen;

	  OutputS3dNet(fp, p);
	  if ( cblen > 0 ) {
		  OutputS3dCBNormals(fp, p, cblen);
	  }
	  return;
  }

  if ( iv ) {
	  OutputIvNet(fp, p);
	  return;
  }

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

      if ((EdgeSamples != 0) && (DoGaussian)) {
	      gk1 = *(gKv + VertexIndex(i1+1, i2, i3));
	      gk2 = *(gKv + VertexIndex(i1, i2+1, i3));
	      gk3 = *(gKv + VertexIndex(i1, i2, i3+1));
      }
      
      if ((EdgeSamples != 0) && (DoMean)) {
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
void wireframeOutput(FILE *fp, VERTEX v1, VERTEX v2, VERTEX v3)
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
void g3dTriangleOutput(FILE *fp, VERTEX v1, VERTEX v2, VERTEX v3, Lnode* ds)
{
  static int count=0;
  Frame F;
  char outputString[1000];
  char Access[81], VertexStr[81], Field[81];
  Scalar x,y,z;
  
  F = StdFrame(SpaceOf( v1.position));

#if 0
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
#else
  fprintf(fp, "P 3 0 0\n");
  NCoords(v1.normal, F, &x,&y,&z);
  fprintf(fp, "n %g %g %g\n", x, y, z);
  PCoords(v1.position, F, &x,&y,&z);
  fprintf(fp, "v %g %g %g\n", x, y, z);

  NCoords(v2.normal, F, &x,&y,&z);
  fprintf(fp, "n %g %g %g\n", x, y, z);
  PCoords(v2.position, F, &x,&y,&z);
  fprintf(fp, "%v %g %g %g\n", x, y, z);

  NCoords(v3.normal, F, &x,&y,&z);
  fprintf(fp, "n %g %g %g\n", x, y, z);
  PCoords(v3.position, F, &x,&y,&z);
  fprintf(fp, "v %g %g %g\n", x, y, z);

  fprintf(fp, "E 0 0 0\n");
#endif

  count++;
}





/*
 *----------------------------------------------------------------------
 *  Function:  sgpTriangleOutput
 *----------------------------------------------------------------------
 */
void sgpTriangleOutput(FILE *fp, VERTEX v1, VERTEX v2, VERTEX v3, Lnode* ds)
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
void TessellateBezierTriangle(Patch ThisPatch)
{
  Patch SampledPatch;
  double r, s, t, step;
  int i1, i2, i3, i;
  Point pnt;
  Normal norm;
  Scalar x;
  extern int IsoLines;
  void LineBezierTriangle();
static int count=0;
/*fprintf(stderr,"bt %d\n",++count);*/


  if ( IsoLines ){
    LineBezierTriangle(ThisPatch);
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
/*if(count==15)fprintf(stderr,"%d %d %d: %s %s\n",i1,i2,i3,PString(pnt),NString(norm));*/
	if (DoGaussian)	{
	  *(gKv + VertexIndex(i1, i2, i3)) = KGauss(ThisPatch, r, s, t);
	}
	if (DoMean)	{
	  *(mKv + VertexIndex(i1, i2, i3)) = KMean(ThisPatch, r, s, t);
	}
      }
    }
    OutputPatchNet( stdout, SampledPatch);
    PatchFree( SampledPatch);
  }
}

/*
 *----------------------------------------------------------------------
 *  Function:  TessellateRationalBezierTriangle
 *----------------------------------------------------------------------
 */
void TessellateRationalBezierTriangle(Lnode* ds)
{
	RationalPatch ThisPatch;
	Patch SampledPatch;
	Point pnt;
	Normal norm;
	int i1, i2, i3;
	Scalar r, s, t, step;

	pGetRationalPatch(&ds, &ThisPatch, World);

	step = 1.0 / ((double) EdgeSamples);

	/* Create a new net by sampling the patch */
	SampledPatch = PatchCreate( EdgeSamples, World, 3);
    
	/* copy dstruct fields of ThisPatch */
	dCopyDstructFields(ThisPatch.ln,"RationalBezierTriangle",
			   SampledPatch.ln,"RationalBezierTriangle");

	/* Uniformly step sample in parameter space */
	for (i1 = 0, r = 0.0; i1 <= EdgeSamples; i1++, r += step) {
		for (i2 = 0, s = 0.0; i2 <= EdgeSamples - i1; i2++, s += step){
			i3 = EdgeSamples - i1 - i2;
			t  = 1.0 - r - s;
			RationalPatchEvalWithNormal(ThisPatch, 
						    &pnt, &norm, r, s, t);
			norm = VDual(VNormalize(NDual(norm)));
			if ( offsetDist != 0. ) {
				pnt = PVAdd(pnt, SVMult(offsetDist,
							NDual(norm)));
			}
			PatchSetPoint( &SampledPatch, pnt, i1, i2, i3);
			PatchSetNormal( &SampledPatch, norm, i1, i2, i3);
#if 0
	if (DoGaussian)	{
	  *(gKv + VertexIndex(i1, i2, i3)) = KGauss(ThisPatch, r, s, t);
	}
	if (DoMean)	{
	  *(mKv + VertexIndex(i1, i2, i3)) = KMean(ThisPatch, r, s, t);
	}
#endif
		}
	}
	OutputPatchNet( stdout, SampledPatch);
	PatchFree( SampledPatch);
}



/* Handle G3D Output */

void G3DMoveTo(Point p, Normal n)
{
  Scalar x,y,z;

  PCoords(p,StdFrame(SpaceOf(p)),&x,&y,&z);
  printf("%f %f %f 0\n",x,y,z);
  NCoords(n,StdFrame(SpaceOf(n)),&x,&y,&z);
  printf("%f %f %f n\n",x,y,z);
}


void G3DDrawTo(Point p, Normal n)
{
  Scalar x,y,z;

  PCoords(p,StdFrame(SpaceOf(p)),&x,&y,&z);
  printf("%f %f %f 1\n",x,y,z);
  NCoords(n,StdFrame(SpaceOf(n)),&x,&y,&z);
  printf("%f %f %f n\n",x,y,z);
}


void LineBezierTriangle(Patch ThisPatch)
{
  int i,j;
  Point p;
  Normal n;
  extern int EdgeSamples;
  double es; /* = EdgeSamples */
  double r,s,t;
  Scalar x,y,z;

  es = EdgeSamples;

  /* step with r constant */
  for(i=0;i<EdgeSamples;i++){
    r = (Scalar)i/es;
    s = 0.0;
    t = 1.0 -r;
    PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
    G3DMoveTo(p,n);
    for(j=1;j<=(EdgeSamples+2-i)*2;j++){
      s = (1.0-r) * (Scalar)j/(Scalar)((EdgeSamples+2-i)*2);
      t = 1.0-r-s;
      PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
      G3DDrawTo(p,n);
    }
  }

  /* step with s constant */
  for(i=0;i<EdgeSamples;i++){
    s = (Scalar)i/es;
    r = 0.0;
    t = 1.0 -s;
    PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
    G3DMoveTo(p,n);
    for(j=1;j<=(EdgeSamples+2-i)*2;j++){
      r = (1.0-s) * (Scalar)j/(Scalar)((EdgeSamples+2-i)*2);
      t = 1.0-s-r;
      PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
      G3DDrawTo(p,n);
    }
  }

  /* step with t constant */
  for(i=0;i<EdgeSamples;i++){
    t = (Scalar)i/es;
    s = 0.0;
    r = 1.0 -t;
    PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
    G3DMoveTo(p,n);
    for(j=1;j<=(EdgeSamples+2-i)*2;j++){
      s = (1.0-t) * (Scalar)j/(Scalar)((EdgeSamples+2-i)*2);
      r = 1.0-t-s;
      PatchEvalWithNormal( ThisPatch, &p, &n, r, s, t);
      G3DDrawTo(p,n);
    }
  }

}
