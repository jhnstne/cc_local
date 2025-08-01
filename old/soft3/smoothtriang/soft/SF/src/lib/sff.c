/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  sff.c
 *	Routines for manipulating second fundemental forms.
 *----------------------------------------------------------------------
 */


#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "sff.h"

#define EPS  0.00000001
#define MAX_STRING 100

#define L (sff->m[0][0])
#define M (sff->m[0][1])
#define N (sff->m[1][1])


/*
 *----------------------------------------------------------------------
 *  Function:  EvalSFF
 *	Evaluate the second fundemental form to extract curvature in
 *  a dirction.
 *----------------------------------------------------------------------
 */
Scalar EvalSFF(SFF *sff, Vector v)
{
  Frame f;
  Scalar dx,dy,dz;

  f = FCreate( "this space for rent",
	       FOrg(StdFrame(SpaceOf(v))), sff->v0, sff->v1, 
	       VVCross(sff->v0,sff->v1) );

  VCoords( v, f, &dx, &dy, &dz );

  if ( fabs(dz) > EPS ) {
    fprintf(stderr,"EvalSFF(): vector not in tangent plane; dz = %g.  Proceeding anyway.\n",dz);
  }

  return ( L * dx*dx  +  2*M * dx*dy  +  N * dy*dy );
}


/*
 *----------------------------------------------------------------------
 *  Function:  EvalSFF2
 *	Evaluate the second fundemental form to extract a mixed partial.
 *----------------------------------------------------------------------
 */
Scalar EvalSFF2(SFF *sff, Vector v1, Vector v2)
{
  Frame f;
  Scalar dx1,dy1,dz1;
  Scalar dx2,dy2,dz2;

  f = FCreate( "this space for rent",
	       FOrg(StdFrame(SpaceOf(v1))), sff->v0, sff->v1, 
	       VVCross(sff->v0,sff->v1) );

  VCoords( v1, f, &dx1, &dy1, &dz1 );
  VCoords( v2, f, &dx2, &dy2, &dz2 );

  if ( fabs(dz1) > EPS ) {
    fprintf(stderr,"\
EvalSFF2(): vector not in tangent plane; dz1 = %g.  Proceeding anyway.\n",dz1);
  }
  if ( fabs(dz2) > EPS ) {
    fprintf(stderr,"\
EvalSFF2(): vector not in tangent plane; dz2 = %g.  Proceeding anyway.\n",dz2);
  }

  return ( L * dx1*dx2  +  M * dx1*dy2  + M * dx2*dy1  +  N * dy1*dy2 );
}



/*
 *----------------------------------------------------------------------
 *  Function:  dGetSFF
 *	extract a second fundemental from from a dstruct.
 *----------------------------------------------------------------------
 */
BOOLEAN dGetSFF(Lnode* ds, char* path, Frame f, SFF* sff)
{
  Lnode* ln1;
  Lnode* ln2;
  
  if ( ! dLookUpLnode(ds, path, ln1 ) ){
    return FALSE;
  } else {
ln1 = ln1->cdr; /* temp fix for now */
    dGetVector( ln1, "v0", f, &(sff->v0) );
    dGetVector( ln1, "v1", f, &(sff->v1) );
    dGetScalar( ln1, "m[0][0]", &(sff->m[0][0]) );
    dGetScalar( ln1, "m[0][1]", &(sff->m[0][1]) );
    dGetScalar( ln1, "m[1][0]", &(sff->m[1][0]) );
    dGetScalar( ln1, "m[1][1]", &(sff->m[1][1]) );
    if ( fabs( sff->m[1][0] - sff->m[0][1] ) > EPS ) {
      fprintf(stderr,"dGetSFF(): bad fundemental form.  Proceeding anyway.\n");
    }
    return TRUE;
  }
}



/*
 *----------------------------------------------------------------------
 *  Function:  pPutSFF
 *	Write a second fundemental form to a dstruct.
 *----------------------------------------------------------------------
 */
BOOLEAN pPutSFF(Lnode** ds, char* path, SFF* sff)
{
  Lnode* ln1;
  Lnode* ln2;
  char ppath[MAX_STRING];
  char* p;
  int result=TRUE;

  if ( strlen(path) + 9 > MAX_STRING ) {
    return FALSE;
  }
  strcpy(ppath,path);
  p = ppath + strlen(ppath);

  sprintf(p,".v0");
  result &= pPutVector(ds,ppath,sff->v0);
  sprintf(p,".v1");
  result &= pPutVector(ds,ppath,sff->v1);
  
  sprintf(p,".m[0][0]");
  result &= pPutScalar( ds, ppath, (sff->m[0][0]) );
  
  sprintf(p,".m[0][1]");
  result &= pPutScalar( ds, ppath, (sff->m[0][1]) );
  
  sprintf(p,".m[1][0]");
  result &= pPutScalar( ds, ppath, (sff->m[1][0]) );
  
  sprintf(p,".m[1][1]");
  result &= pPutScalar( ds, ppath, (sff->m[1][1]) );

  return result;
}


/*
 *----------------------------------------------------------------------
 *  Function:  CreateConjugateSFF
 *	Return a second fundemental form given two conjugate directions
 *	and the curvatures in those direction.
 *----------------------------------------------------------------------
 */
SFF CreateConjugateSFF(Vector v0, Vector v1, Scalar k0, Scalar k1)
{
  SFF s;

  if ( 1.-fabs(VVDot(VNormalize(v0), VNormalize(v1))) < EPS ) {
    fprintf(stderr,"CreateConjugateSFF: vectors nearly parallel.  Exiting.\n");
    exit(1);
  }

  s.v0 = v0;
  s.v1 = v1;
  s.m[0][0] = k0;
  s.m[1][1] = k1;
  s.m[0][1] = s.m[1][0] = 0.;

  return s;
}

/*
 *----------------------------------------------------------------------
 *  Function:  SffConjugate
 *	Give a conjugate vector to the vector v relative to the SFF s.
 *----------------------------------------------------------------------
 */
Vector SFFConjugate(SFF* s, Vector v)
{
  Frame f;
  /* coords of v relative to basis of s.  'e' should be zero */
  Scalar a;
  Scalar b;
  Scalar c;

  /* result after multiplying by v */
  Scalar d;
  Scalar e;

  /* coords of conjugate relative to basis of s */
  Scalar g;
  Scalar h;


  if ( VMag(v) < 0.00001 ) {
    fprintf(stderr,"SFFConjugate: Vector v too small %g %g.  Exiting.\n",
	    d,e);
    fprintf(stderr,"a %g, b %g\n",a,b);
    fprintf(stderr,"0 0 %g,  0 1 %g,  1 0 %g,   1 1 %g\n",
	    s->m[0][0],s->m[0][1],s->m[1][0],s->m[1][1]);
    exit(1);
  }

  f = FCreate( "SFFConjugate", FOrg(StdFrame(SpaceOf(s->v0))), 
	       s->v0, s->v1, VVCross(s->v0, s->v1));
  VCoords(v, f, &a, &b, &c);
  if ( fabs(c) > 0.00001 ) {
    fprintf(stderr,"SFFConjugate: vector not in tangent plane.  Exiting.\n");
    exit(1);
  }

  d = a*s->m[0][0] + b*s->m[1][0];
  e = a*s->m[0][1] + b*s->m[1][1];
  if ( fabs(e) > 0.00001  ||  fabs(d) > 0.00001 ) {
    if ( fabs(e) > fabs(d) ) {
      g = 1.;
      h = -g*d/e;
    } else {
      h = 1.;
      g = -h*e/d;
    }
  } else {
    /* if d and e are (near) zero, then any direction is conjugate */
    return v;
  }

  return VVAdd( SVMult(g, s->v0), SVMult(h, s->v1) );
}


/*
 *----------------------------------------------------------------------
 *  Function:  SFFGaussianCurvature
 *	Return the Gaussian Curvature associated with a second
 *	fundemental form.  
 *----------------------------------------------------------------------
 */
Scalar SFFGaussianCurvature(SFF* sff)
{
	Scalar E,F,G;

	E = VVDot(sff->v0,sff->v0);
	F = VVDot(sff->v0,sff->v1);
	G = VVDot(sff->v1,sff->v1);
	return (L*N - M*M)/(E*G - F*F);
}


/*
 *----------------------------------------------------------------------
 *  Function:  SFFMeanCurvature
 *	Return the Mean Curvature associated with a second
 *	fundemental form.  
 *----------------------------------------------------------------------
 */
Scalar SFFMeanCurvature(SFF* sff)
{
	Scalar E,F,G;

	E = VVDot(sff->v0,sff->v0);
	F = VVDot(sff->v0,sff->v1);
	G = VVDot(sff->v1,sff->v1);
	return (G*L + E*N - 2*F*M)/(2*(E*G-F*F));
}





SFF ComputeOrthonormalSFF(SFF sff)
{
	SFF rsff;

	rsff.v0 = VNormalize(sff.v0);
	rsff.v1 = VNormalize(VVCross(VVCross(sff.v0, sff.v1), sff.v0));

	rsff.m[0][0] = EvalSFF(&sff, rsff.v0);
	rsff.m[1][1] = EvalSFF(&sff, rsff.v1);
	rsff.m[0][1] = rsff.m[1][0] = EvalSFF2(&sff, rsff.v0, rsff.v1);

	return rsff;
}


static Compute2x2Eigenvalues(Scalar m[2][2], Scalar s[2])
{
	double C[3];
	double r[2];
	int n;

	C[0] = 1.;
	C[1] = -(m[0][0]+m[1][1]);
	C[2] = m[0][0]*m[1][1] - m[0][1]*m[1][0];

	QuadraticRoots(C, &n, r);
	switch (n) {
	      case 0:
		fprintf(stderr, "Compute2x2Eigenvalues: no roots.\n");
		exit(1);
	      case 1:
		s[0] = s[1] = r[0];
		return;
	      case 2:
		s[0] = r[0];
		s[1] = r[1];
		return;
	}
}

static ComputeEigen(Scalar m[2][2], Scalar e[2][2], Scalar s[2])
{
	
	Compute2x2Eigenvalues(m, s);
	if ( fabs(m[1][0]) < 1e-9 ) {
		e[0][0] = 1.;
		e[0][1] = 0.;
		s[0] = m[0][0];

		e[1][0] = 0.;
		e[1][1] = 1.;
		s[1] = m[1][1];
	} else {
		e[0][0] = 1.;
		e[0][1] = (s[0]-m[0][0])/m[1][0];

		e[1][0] = 1.;
		e[1][1] = (s[1]-m[0][0])/m[1][0];
	}
}


void ComputePrincipalVectors(SFF sff, Vector* v1, Vector* v2, 
			     Scalar* s0, Scalar* s1)
{
	Scalar e[2][2];
	Scalar s[2];

	sff = ComputeOrthonormalSFF(sff);

	if ( fabs(sff.m[1][0]) <= EPS  &&  fabs(sff.m[0][1]) <= EPS ) {
		*v1 = sff.v0;
		*v2 = sff.v1;
		*s0 = sff.m[0][0];
		*s1 = sff.m[1][1];
		return;
	}

	ComputeEigen(sff.m, e, s);

	*v1 = VVAdd( SVMult(e[0][0], sff.v0), SVMult(e[0][1], sff.v1) );
	*v2 = VVAdd( SVMult(e[1][0], sff.v0), SVMult(e[1][1], sff.v1) );

	if ( fabs(s[0]) > 1e-4 ) {
		*v1 = SVMult(s[0], VNormalize(*v1));
	}
	if ( fabs(s[1]) > 1e-4 ) {
		*v2 = SVMult(s[1], VNormalize(*v2));
	}

	*s0 = s[0];
	*s1 = s[1];

	if ( fabs(EvalSFF2(&sff, *v1,*v2)) > 1e-4  ||  
	     fabs(VVDot(*v1,*v2)) > 1e-4 ) {
		if ( fabs(sff.m[0][1]) < 1e-6 ) {
			*v1 = sff.v0;
			*v2 = sff.v1;
			*s0 = 0.;
			*s1 = 0.;
		} else {
			fprintf(stderr,"ComputePrincipalVectors: I failed.\n");
			fprintf(stderr,"%g, %g\n",fabs(EvalSFF2(&sff, *v1,*v2)),
				fabs(VVDot(*v1,*v2)));
			fprintf(stderr,"s : %g %g\n",s[0],s[1]);
			fprintf(stderr,"%s and %s\n",VString(*v1),VString(*v2));
			fprintf(stderr," v0 = %s\n v1 = %s\n",VString(sff.v0),
				VString(sff.v1));
			fprintf(stderr,"%g %g\n%g %g\n",
				sff.m[0][0],sff.m[0][1],
				sff.m[1][0],sff.m[1][1]);
			exit(1);
		}
	}
}
