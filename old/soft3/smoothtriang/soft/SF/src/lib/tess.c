/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
** Author: Steve Mann
** 
**
*/
#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "vertex.h"
#include "material.h"
#include "patch.h"
#include "libgeo.h"


void s3dTriOutput(Scalar *gk, FILE *fp, VERTEX v1, VERTEX v2, VERTEX v3, 
		  Frame f)
{
	static int count=0;
	char outputString[1000];
	char Access[81], VertexStr[81], Field[81];
	
	sprintf(outputString,"tri-%d",count);
	PutString("Triangle.name", outputString);
	PutVertexPosition("Triangle.vertex1",v1.position);
	PutVertexNormal("Triangle.vertex1",v1.normal);
	
	PutVertexPosition("Triangle.vertex2",v2.position);
	PutVertexNormal("Triangle.vertex2",v2.normal);
	
	PutVertexPosition("Triangle.vertex3",v3.position);
	PutVertexNormal("Triangle.vertex3",v3.normal);
	
	if ( gk != NULL ){
		PutScalar("Triangle.vertex1.curvature.gaussian",gk[0]);
		PutScalar("Triangle.vertex2.curvature.gaussian",gk[1]);
		PutScalar("Triangle.vertex3.curvature.gaussian",gk[2]);
	}
	FlushDstruct();
	count++;
}

#define MAX_DATA 5000

/*
 ** Output (as s3d triangles) the control net of the given patch.
 */
void	OutputPatchNetS3d(FILE *fp, Patch p, Frame f, Scalar* gk)
{
  int i1, i2, i3;              /* Multi-indices summing to p.degree - 1 */
  int kup = p.degree - 1;
  int kdown = p.degree - 2;
  VERTEX v1, v2, v3;
  Normal Nv;
  char s1[MAX_DATA],s2[MAX_DATA],s3[MAX_DATA];
  Scalar gk3[3];
  Scalar *gkp;
  
  /* Output the "up" triangular panels */
  for (i1 = 0; i1 <= kup; i1++) {
    for (i2 = 0; i2 <= kup - i1; i2++) {
      i3 = kup - i1 - i2;
      
      /* Pull out an upward panel of the control net */
      v1 = PatchGetVertex(p, i1+1,i2,  i3);
      v2 = PatchGetVertex(p, i1,  i2+1,i3),
      v3 = PatchGetVertex(p, i1,  i2,  i3+1);

      if ( gk != NULL ){
	gk3[0] = *(gk+VertexIndex(i1+1,i2,  i3));
	gk3[1] = *(gk+VertexIndex(i1,  i2+1,i3));
	gk3[2] = *(gk+VertexIndex(i1,  i2,  i3+1));
	gkp = gk3;
      } else {
	gkp = NULL;
      }

      /* Build a normal if necessary */
      if (p.normaldim == 0) {
	Nv = PPPNormal( v1.position, v2.position, v3.position);
	v1.normal = v2.normal = v3.normal = Nv;
      }
      
      /* Output the triangle */
      s3dTriOutput( gkp, fp, v1, v2, v3, f );
    }
  }
  
  /* Output the "down" triangular panels */
  for (i1 = 0; i1 <= kdown; i1++) {
    for (i2 = 0; i2 <= kdown - i1; i2++) {
      i3 = kdown - i1 - i2;
      
      /* Pull out a downward panel of the control net */
      v1 = PatchGetVertex(p, i1,  i2+1,i3+1);
      v2 = PatchGetVertex(p, i1+1,i2,  i3+1),
      v3 = PatchGetVertex(p, i1+1,i2+1,i3);

      if ( gk != NULL ){
	gk3[0] = *(gk+VertexIndex(i1,  i2+1,i3+1));
	gk3[1] = *(gk+VertexIndex(i1+1,i2,  i3+1));
	gk3[2] = *(gk+VertexIndex(i1+1,i2+1,i3));
	gkp = gk3;
      } else {
	gkp = NULL;
      }
      
      /* Build a normal if necessary */
      if (p.normaldim == 0) {
	Nv = PPPNormal( v1.position, v2.position, v3.position);
	v1.normal = v2.normal = v3.normal = Nv;
      }
      
      /* Output the triangle */
      s3dTriOutput( gkp, fp, v1, v2, v3, f );
    }
  }
  
}


/*
 ** Output (as iv nets) the control net of the given patch.
 */
void	OutputPatchNetIV(FILE *fp, Patch p, Frame f, Scalar* gk)
{
	int i,j,k;
	VERTEX v;
	Scalar x,y,z;
	static first=1;

	if ( first ) {
		fprintf(fp, "#Inventor V2.1 ascii\n");
		fprintf(fp, "Separator { \n");
		fprintf(fp, "  Material{\n");
		fprintf(fp, "    specularColor 0.5 0.5 0.5\n");
		fprintf(fp, "    diffuseColor 0.9 0.4 0.4\n");
		fprintf(fp, "    shininess 1.0 }\n");
		fprintf(fp, "  MaterialBinding { value OVERALL }\n");
		fprintf(fp, "  DirectionalLight{ color 0.05 0.05 0.6 direction  0.1 0.1 0.0 }\n");
		fprintf(fp, "  DirectionalLight { color 0.6 0.05 0.05\n");
		fprintf(fp, "    direction -0.1 0.1 0.0 }\n");
		fprintf(fp, "  ShapeHints { vertexOrdering COUNTERCLOCKWISE }\n");
		fprintf(fp, "\n");
		first = 0;
	}

/*	fprintf(fp, "Separator {\n");*/

	fprintf(fp, "  Coordinate3 {\n");
	fprintf(fp, "    point [\n");
	for (i = 0; i <= p.degree; i++) {
		for (j = 0; j <= p.degree - i; j++) {
			k = p.degree - i - j;
			v = PatchGetVertex(p, i,j,k);
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
			Scalar len;
			k = p.degree - i - j;
			v = PatchGetVertex(p, i,j,k);
			NCoords(v.normal, f, &x,&y,&z);
			len = sqrt(x*x+y*y+z*z);
			fprintf(fp, "      %lg %lg %lg,\n", x/len,y/len,z/len);
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
/*	fprintf(fp, "}\n\n");  */
}



double EPS = 1e-7;

void SetTessEps(double e)
{
	EPS = e;
}


Normal CalcNormal(void* patch, Point (*eval)(Patch*, Scalar[]), 
		  int i, int j, int k)
{
  Scalar b[3];
  double sum;
  Point p,q,r;
  Vector vn;

  sum = i+j+k;

  b[0] = (double)i/sum;
  b[1] = (double)j/sum;
  b[2] = 1. - b[0] - b[1];

  p = eval(patch,b);
  if ( i > 0  &&  j > 0 ) {
    b[0] -= EPS;
    b[2] += EPS;
    r = eval(patch,b);
    b[0] += EPS;
    b[1] -= EPS;
    q = eval(patch,b);
  } else if ( i + j == 0 ) {
    b[0] += EPS;
    b[2] -= EPS;
    r = eval(patch,b);
    b[0] -= EPS;
    b[1] += EPS;
    q = eval(patch,b);
  } else if ( i == 0 ) {
    b[1] -= EPS;
    b[2] += EPS;
    r = eval(patch,b);
    b[0] += EPS;
    b[2] -= EPS;
    q = eval(patch,b);
  } else /* j == 0 */ {
    b[0] -= EPS;
    b[2] += EPS;
    q = eval(patch,b);
    b[2] -= EPS;
    b[1] += EPS;
    r = eval(patch,b);
  }
  vn = VVCross(PPDiff(r,p),PPDiff(q,p));
  if ( VMag(vn) > 1e-14) {
	  return VDual(VNormalize(vn));
  } else {
	  static int first=1;

	  if ( first ) {
		  fprintf(stderr, 
	      "CalcNormal: Warning: zero normal.  Use try larger epsilon.\n");
		  first = 0;
	  }
	  return VDual(vn);
  }
}

#define S3D_OUT 1
#define IV_OUT 2

static int tessmethod = S3D_OUT;

void SetTessMethod(char* method)
{
	if ( strcmp(method, "s3d") == 0 ) {
		tessmethod = S3D_OUT;
	} else if ( strcmp(method, "iv") == 0 ) {
		tessmethod = IV_OUT;
	} else {
		fprintf(stderr, "SetTessMethod: Warning, unknown method %s\n",
			method);
		fprintf(stderr, "               Not changing method\n");
	}
}

static void OutputPatchNet(FILE* fp, Patch p, Frame f, Scalar* gk)
{
	if ( tessmethod == S3D_OUT ) {
		OutputPatchNetS3d(fp, p, f, gk);
	} else if ( tessmethod == IV_OUT ) {
		OutputPatchNetIV(fp, p, f, gk);
	} else {
		fprintf(stderr, "OutputPatchNet: Warning, unknown method %d\n",
			tessmethod);
		fprintf(stderr, "               No output\n");
	}
}

void TessPatch(void* patch, Point (*eval)(Patch*, Scalar[]), int samples, Frame frame)
{
  Scalar b[3];
  int i,j;
  Patch SamplePatch;
  Point p;
  Normal n;

  SamplePatch = PatchCreate( samples, SpaceOf(frame), 3 );

  for(i=0;i<=samples;i++){
    for(j=0;j<=samples-i;j++){
      b[0] = (double)i/(double)samples;
      b[1] = (double)j/(double)samples;
      b[2] = 1. - b[0] - b[1];
      p = eval(patch,b);
      n = CalcNormal(patch,eval,i,j,samples-i-j);
      PatchSetPoint(&SamplePatch, p, i, j, samples-i-j );
      PatchSetNormal(&SamplePatch, n, i, j, samples-i-j );
    }
  }
  OutputPatchNet(stdout, SamplePatch, frame, NULL);
  PatchFree( SamplePatch );
}

#define SQ(A) ((A)*(A))


static Scalar E(void* patch, Point (*eval)(Patch*, Scalar[]), Scalar b[3])
{
  Point p1,p2;
  Vector v;

#if 0
  p1 = eval(patch,b);
  b[0] += EPS;
  b[1] -= EPS;
  p2 = eval(patch,b);
  b[1] += EPS;
  b[0] -= EPS;
  v = PPDiff(p1,p2);
  return VVDot(v,v)/SQ(EPS);
#else
  Scalar beps[3];

  beps[0] = b[0] + EPS;
  beps[1] = b[1] - EPS;
  beps[2] = b[2];
  p2 = eval(patch,beps);
  beps[0] = b[0] - EPS;
  beps[1] = b[1] + EPS;
  p1 = eval(patch,beps);
  v = PPDiff(p1,p2);
  return VVDot(v,v)/SQ(2.*EPS);
#endif

}

static Scalar F(void* patch, Point (*eval)(Patch*, Scalar[]), Scalar b[3])
{
  Point p1,p2;
  Vector v,w;

#if 0
  p1 = eval(patch,b);
  b[0] += EPS;
  b[1] -= EPS;
  p2 = eval(patch,b);
  b[0] -= EPS;
  b[1] += EPS;
  v = PPDiff(p1,p2);

  p1 = eval(patch,b);
  b[0] += EPS;
  b[2] -= EPS;
  p2 = eval(patch,b);
  b[0] -= EPS;
  b[2] += EPS;
  w = PPDiff(p1,p2);

  return VVDot(v,w)/SQ(EPS);
#else
  Scalar beps[3];

  beps[0] = b[0] + EPS;
  beps[1] = b[1] - EPS;
  beps[2] = b[2];
  p2 = eval(patch,beps);
  beps[0] = b[0] - EPS;
  beps[1] = b[1] + EPS;
  p1 = eval(patch,beps);
  v = PPDiff(p1,p2);

  beps[0] = b[0] + EPS;
  beps[1] = b[1];
  beps[2] = b[2] - EPS;
  p2 = eval(patch,beps);
  beps[0] = b[0] - EPS;
  beps[2] = b[2] + EPS;
  p1 = eval(patch,beps);
  w = PPDiff(p1,p2);

  return VVDot(v,w)/SQ(2.*EPS);

#endif
}

static Scalar G(void* patch, Point (*eval)(Patch*, Scalar[]), Scalar b[3])
{
  Point p1,p2;
  Vector v;

#if 0
  p1 = eval(patch,b);
  b[0] += EPS;
  b[2] -= EPS;
  p2 = eval(patch,b);
  b[0] -= EPS;
  b[2] += EPS;
  v = PPDiff(p1,p2);
  return VVDot(v,v)/SQ(EPS);
#else
  Scalar beps[3];

  beps[0] = b[0] + EPS;
  beps[1] = b[1];
  beps[2] = b[2] - EPS;
  p2 = eval(patch,beps);
  beps[0] = b[0] - EPS;
  beps[2] = b[2] + EPS;
  p1 = eval(patch,beps);
  v = PPDiff(p1,p2);
  return VVDot(v,v)/SQ(2.*EPS);
#endif

}

Scalar wt[3]={1.,-2.,1.};

static Scalar L(void* patch, Point (*eval)(Patch*, Scalar[]), Scalar b[3], 
		Normal n)
{
  Point p[3];
  Vector v;

  p[1] = eval(patch,b);
  b[0] += EPS;
  b[1] -= EPS;
  p[0] = eval(patch,b);
  b[0] -= 2.*EPS;
  b[1] += 2.*EPS;
  p[2] = eval(patch,b);
  b[0] += EPS;
  b[1] -= EPS;

  v = PPvcN(3, p, wt);
  return NVApply(n,v)/SQ(EPS);
}


Scalar wt2[4]={1.,-1.,-1.,1.};

static Scalar M(void* patch, Point (*eval)(Patch*, Scalar[]), Scalar b[3], 
		Normal n)
{
  Point p[4];
  Vector v;

  b[0] += 2.*EPS;
  b[1] -= EPS;
  b[2] -= EPS;
  p[0] = eval(patch,b);
  b[0] -= 2.*EPS;
  b[1] += 2.*EPS;
  p[1] = eval(patch,b);
  b[0] -= 2.*EPS;
  b[2] += 2.*EPS;
  p[3] = eval(patch,b);
  b[0] += 2.*EPS;
  b[1] -= 2.*EPS;
  p[2] = eval(patch,b);
  b[0] += 0.;
  b[1] += EPS;
  b[2] -= EPS;

  v = PPvcN(4, p, wt2);
  return NVApply(n,v)/(4.*SQ(EPS));
}

static Scalar N(void* patch, Point (*eval)(Patch*, Scalar[]), Scalar b[3], 
		Normal n)
{
  Point p[3];
  Vector v;

  p[1] = eval(patch,b);
  b[0] += EPS;
  b[2] -= EPS;
  p[0] = eval(patch,b);
  b[0] -= 2.*EPS;
  b[2] += 2.*EPS;
  p[2] = eval(patch,b);
  b[0] += EPS;
  b[2] -= EPS;

  v = PPvcN(3, p, wt);
  return NVApply(n,v)/SQ(EPS);
}





Scalar EstGaussianCurvature(void* patch, Point (*eval)(Patch*, Scalar[]), 
			    int i, int j, int k, Normal normal)
{
  Scalar b[3];
  Scalar e,f,g,l,m,n;
  Scalar gk;
  Scalar sum;

  sum=i+j+k;

  /* first set b's to true values */
  b[0] = (double)i/sum;
  b[1] = (double)j/sum;
  b[2] = 1. - b[0] - b[1];
  
  /* now peturb point to be slighty inside the patch if needed */
  if ( i == 0  ||  j == 0  ||  k == 0 ){
    if ( i == 0 ){
      b[0] += 2.*EPS;
      b[1] -= EPS;
      b[2] -= EPS;
    }
    if ( j == 0 ){
      b[1] += 2.*EPS;
      b[0] -= EPS;
      b[2] -= EPS;
    }
    if ( k == 0 ){
      b[2] += 2.*EPS;
      b[0] -= EPS;
      b[1] -= EPS;
    }
    /* need more adjustments if at vertex */
    if ( i==0 && j==0 ){
      b[2] -= 2.*EPS;
      b[0] += EPS;
      b[1] += EPS;
    }
    if ( i==0 && k==0 ){
      b[1] -= 2.*EPS;
      b[0] += EPS;
      b[2] += EPS;
    }
    if ( j==0 && k==0 ){
      b[0] -= 2.*EPS;
      b[1] += EPS;
      b[2] += EPS;
    }
  }
  /* now compute all points needed for approx */
  e = E(patch,eval,b);
  f = F(patch,eval,b);
  g = G(patch,eval,b);
  l = L(patch,eval,b,normal);
  m = M(patch,eval,b,normal);
  n = N(patch,eval,b,normal);
  gk = (l*n - m*m)/(e*g - f*f);

  return gk;
}



void gkTessPatch(void* patch, Point (*eval)(Patch*, Scalar[]), int samples,Frame frame)
{
  Scalar b[3];
  int i,j;
  Patch SamplePatch;
  Point p;
  Normal n;
  Scalar* gk;

  SamplePatch = PatchCreate( samples, SpaceOf(frame), 3 );

  gk = (Scalar*)malloc( NetSize(samples)*sizeof(Scalar) );

  for(i=0;i<=samples;i++){
    for(j=0;j<=samples-i;j++){
      b[0] = (double)i/(double)samples;
      b[1] = (double)j/(double)samples;
      b[2] = 1. - b[0] - b[1];
      p = eval(patch,b);
      PatchSetPoint(&SamplePatch, p, i, j, samples-i-j );
      n = CalcNormal(patch,eval,i,j,samples-i-j);
      PatchSetNormal(&SamplePatch, n, i, j, samples-i-j );

      *(gk+VertexIndex(i,j,samples-i-j)) = 
	EstGaussianCurvature(patch,eval,i,j,samples-i-j,n);

    }
  }
  OutputPatchNet( stdout, SamplePatch, frame, gk );
  PatchFree( SamplePatch );
}


Vector FirstDer(void* patch, Point (*eval)(Patch*, Scalar[]), Scalar b[3], int dir)
{
	Point p1,p2;
	Vector v;
	Scalar beps[3];

	beps[dir] = b[dir];
	beps[(dir+1)%3] = b[(dir+1)%3]-EPS;
	beps[(dir+2)%3] = b[(dir+2)%3]+EPS;
	p1 = eval(patch,beps);
	beps[dir] = b[dir];
	beps[(dir+1)%3] = b[(dir+1)%3]+EPS;
	beps[(dir+2)%3] = b[(dir+2)%3]-EPS;
	p2 = eval(patch,beps);
	v = PPDiff(p1,p2);
	return SVMult(.5/EPS,v);
}

Vector SecondDer(void* patch, Point (*eval)(Patch*, Scalar[]), 
		 Scalar b[3], int dir1, int dir2)
{
	Point p[3];
	Vector v;
	Scalar beps[3];
	
	if ( dir1 == dir2 ) {
		beps[dir1] = b[dir1];
		beps[(dir1+2)%3] = b[(dir1+2)%3] + EPS;
		beps[(dir1+1)%3] = b[(dir1+1)%3] - EPS;
		p[0] = eval(patch,beps);

		p[1] = eval(patch,b);

		beps[dir1] = b[dir1];
		beps[(dir1+2)%3] = b[(dir1+2)%3] - EPS;
		beps[(dir1+1)%3] = b[(dir1+1)%3] + EPS;
		p[2] = eval(patch,beps);
		
		v = PPvcN(3, p, wt);
		return SVMult(1./SQ(EPS),v);
	} else {
		fprintf(stderr,"SecondDer: mixed partial not implemented.\n");
		exit(1);
	}
}

static int MAX3(Scalar a,Scalar b,Scalar c)
{
	if ( a >= b  &&  a >= c ) {
		return 0;
	} else if ( b >= a  &&  b >= c ) {
		return 1;
	} else if ( c >= a  &&  c >= b ) {
		return 2;
	} else {
		fprintf(stderr,"MAX3: error.\n");
		exit(1);
	}
}

static Scalar CUBE(Scalar a)
{
	return a*a*a;
}

void CompCurvature(void* patch, Point (*eval)(Patch*, Scalar[]), 
		   Scalar b0, Scalar b1, Scalar b2)
{
	int base;
	Scalar k;
	Vector vfd,vsd;
	Scalar b[3];

	b[0] = b0;
	b[1] = b1;
	b[2] = b2;
	base = MAX3(b[0],b[1],b[2]);

#if 1
	if ( 1-b[base] < EPS ) {
		b[base] -= EPS;
		b[(base+2)%3] += EPS;
	}
#endif

	vfd = FirstDer(patch,eval,b,(base+1)%3);
	vsd = SecondDer(patch,eval,b,(base+1)%3,(base+1)%3);
	k = VMag( VVCross(vfd, vsd) ) / CUBE(VMag(vfd));
	{static int first=1; static Scalar prev;
	if ( first || EPS != prev )
	  fprintf(stderr,"eps = %g\n",EPS);
	 prev = EPS; first=0;
	}
	fprintf(stderr,"CompCurvature: %g %g => %g / %g = %17.17f\n",
		VMag(vfd), VMag(vsd), VMag(VVCross(vfd,vsd)), CUBE(VMag(vfd)),k);
}
