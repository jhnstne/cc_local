/*
 *----------------------------------------------------------------------
 *
 * Author: Steve Mann
 * Created: Wed Feb 07, 1990 at 10:27:20 AM
 * Last Modified: Fri Feb 09, 1990 at 04:02:30 PM
 *
 *  File:  tpbezier.c
 *	Routines to handle implement tensor product Bezier patches.
 *
 * The V parametric direction corresponds to rows.  The U parametric
 * direction corresponds to columns.  Thus, rows are numbered from 0 to
 * degreeU, and columns are numbered from 0 to degreeV, even though this
 * at first sounds backwards.
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include "geometry.h"
#include "dstruct.h"
#include "sff.h"
#include "tpbezier.h"
#include "libgeo.h"



/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierCreate
 *	Allocate space and otherwise initialize a TPBezier.
 *----------------------------------------------------------------------
 */
TPBezier TPBezierCreate(int degreeU, int degreeV, Space range)
{
  TPBezier new;

  new.range = range;
  new.degreeU = degreeU;
  new.degreeV = degreeV;
  new.net = (Point*) malloc ( sizeof(Point) * (degreeU + 1) * (degreeV + 1));
  return new;
}



/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierFree
 *	Free the space used by a patch.
 *----------------------------------------------------------------------
 */
void TPBezierFree(TPBezier patch)
{
  free( patch.net );
}



/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierRead
 *	Read a TPBezier from standard input.
 *----------------------------------------------------------------------
 */
int TPBezierRead(TPBezier *patch, Space range)
{
  char buf[100];
  Scalar dU,dV;
  int i,j;
  Point vertex;

  if ( ReadDstruct() == EOF )
    return 0;

  if ( !QueryDstructPath("TPBezier") ) {
    fprintf(stderr, "TPBezierRead: TPBezier not read in.  Exiting.\n");
    exit(1);
  }

  if ( !GetScalar("TPBezier.degreeU",&dU) ){
    fprintf(stderr, "TPBezierRead: TPBezier.degreeU does not exist!  Exiting.\n");
    exit(1);
  } else {
    i = dU;
  }

  if ( !GetScalar("TPBezier.degreeV",&dV) ){
    fprintf(stderr, "TPBezierRead: TPBezier.degreeU does not exist!  Exiting.\n");
    exit(1);
  } else {
    j = dV;
  }

  *patch = TPBezierCreate(i, j, range);

  for(i=0; i<=patch->degreeU; i++){
    for(j=0; j<=patch->degreeV; j++){
      sprintf(buf, "TPBezier.net[%d][%d]",i,j);
      if ( !GetVertexPosition(buf,StdFrame(range),&vertex) ) {
	fprintf(stderr,"TTPatchRead: %s not read in.  Exiting.\n",buf);
	exit(1);
      }
      TPBezierSetPoint(*patch, vertex, i, j);
    }
  }

  return 1;
}

/*
 *----------------------------------------------------------------------
 *  Function:  GetTPBezier
 *	Read a TPBezier from din.
 *----------------------------------------------------------------------
 */
int GetTPBezier(TPBezier *patch, Space range)
{
  char buf[100];
  Scalar dU,dV;
  int i,j;
  Point vertex;

  if ( !QueryDstructPath("TPBezier") ) {
    fprintf(stderr, "GetTPBezier: TPBezier not read in.  Exiting.\n");
    exit(1);
  }

  if ( !GetScalar("TPBezier.degreeU",&dU) ){
    fprintf(stderr, "GetTPBezier: TPBezier.degreeU does not exist!  Exiting.\n");
    exit(1);
  } else {
    i = dU;
  }

  if ( !GetScalar("TPBezier.degreeV",&dV) ){
    fprintf(stderr, "GetTPBezier: TPBezier.degreeU does not exist!  Exiting.\n");
    exit(1);
  } else {
    j = dV;
  }

  *patch = TPBezierCreate(i, j, range);

  for(i=0; i<=patch->degreeU; i++){
    for(j=0; j<=patch->degreeV; j++){
      sprintf(buf, "TPBezier.net[%d][%d]",i,j);
      if ( !GetVertexPosition(buf,StdFrame(range),&vertex) ) {
	fprintf(stderr,"TTPatchRead: %s not read in.  Exiting.\n",buf);
	exit(1);
      }
      TPBezierSetPoint(*patch, vertex, i, j);
    }
  }

  return 1;
}


/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierWrite
 *	Write a TPBezier to standard output.
 *----------------------------------------------------------------------
 */
void TPBezierWrite(TPBezier patch)
{
  char buf[100];
  int i,j;

  PutScalar("TPBezier.degreeU",(Scalar)patch.degreeU);
  PutScalar("TPBezier.degreeV",(Scalar)patch.degreeV);
  for(i=0; i<=patch.degreeU; i++){
    for(j=0; j<=patch.degreeV; j++){
      sprintf(buf,"TPBezier.net[%d][%d]",i,j);
      PutVertexPosition(buf,TPBezierGetPoint(patch, i, j));
    }
  }
  FlushDstruct();
}



/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierGetPoint
 *	Return the point in the TPBezier corresponding to the index.
 *----------------------------------------------------------------------
 */
Point TPBezierGetPoint(TPBezier patch, int i1,int i2)
{
  if ( i1 > patch.degreeU  ||  i1 < 0  ||  i2 > patch.degreeV  ||  i2 < 0 ) {
    fprintf(stderr,"TPBezierGetPoint: point %d %d does not exist!  Returning Origin\n",i1,i2);
    return FOrg(StdFrame(patch.range));
  } else {
    return( patch.net[i1*(patch.degreeV+1) + i2] );
  }
}

/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierSetPoint
 *	Set the point in the TPBezier corresponding to the index.
 *----------------------------------------------------------------------
 */
void TPBezierSetPoint(TPBezier patch, Point pt, int i1, int i2)
{
  if ( i1 > patch.degreeU  ||  i1 < 0  ||  i2 > patch.degreeV  ||  i2 < 0 ) {
    fprintf(stderr,"TPBezierSetPoint: point %d %d does not exist!\n",i1,i2);
  } else {
    patch.net[i1*(patch.degreeV+1) + i2] = pt;
  }
}


/*
 *----------------------------------------------------------------------
 * Function: TPBezierEval( patch, u, v)
 * Return the patch evaluated at the point whose coordinates are (u,v).
 *----------------------------------------------------------------------
 */
Point TPBezierEval(TPBezier patch, Scalar u, Scalar v)
{
    Point surfacePoint;
    Normal normal;

    TPBezierEvalWithNormal( patch, &surfacePoint, &normal, u, v);
    return surfacePoint;
}

/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierEvalWithNormal
 * Evaluate the patch at the point whose coordinates are (u,v).
 * Also computed is the normal vector to the surface at the point.
 *----------------------------------------------------------------------
 */
void TPBezierEvalWithNormal(TPBezier patch, Point *point, Normal *normal, 
			    Scalar u, Scalar v)
{
    int i;
    Point *tmp;                   /* Temporary net for deCasteljau alg */
    Vector partialU, partialV;
    int iu, iv;                   /* Row, column indices.              */
    int k;                        /* The step of deCasteljau's alg     */
    Point fv0,fv1,f0u,f1u;

    /* Create the temporary storage */
    tmp = (Point *) malloc(sizeof(Point)*(patch.degreeU+1)*(patch.degreeV+1));

    /* Copy the patches net into tmp */
    for (i = 0; i < (patch.degreeU+1)*(patch.degreeV+1); i++) {
      tmp[i] = patch.net[i];
    }

    /* Do de Casteljau's curve algorithm on the rows of the net */
    for (iu = 0; iu <= patch.degreeU; iu++) { /* for each row */
	for (k = 1; k < patch.degreeV; k++) {
	    for (iv = 0; iv <= patch.degreeV - k; iv++) {
		int offset = iu*(patch.degreeV+1)+iv;
		tmp[offset] = PPrr( tmp[offset], tmp[offset+1], v, 1.0 - v);
	    }
	}
    }

    /* Now do de Casteljau on the first two columns of the net */
    for (iv = 0; iv <= 1; iv++){ /* for each column */
      for (k = 1; k < patch.degreeU; k++) {
	for (iu = 0; iu <= patch.degreeU - k; iu++) {
	  int offset = iu*(patch.degreeV+1)+iv;
	  tmp[offset]=PPrr(tmp[offset],tmp[offset+patch.degreeV+1],u,1.0-u);
	}
      }
    }

    /* now we have a bilinear patch in tmp */
    fv0 = PPrr(tmp[0],tmp[1],v,1.0-v);
    fv1 = PPrr(tmp[patch.degreeV+1],tmp[patch.degreeV+1 +1],v,1.0-v);
    f0u = PPrr(tmp[0],tmp[patch.degreeV+1],u,1.0-u);
    f1u = PPrr(tmp[1],tmp[patch.degreeV+1 +1],u,1.0-u);

    *point = PPrr(fv0,fv1,u,1.0-u);

    /*-----------------------------------------------------*/
    /* Compute the normal by computing partial derivatives */
    /*-----------------------------------------------------*/

    /* partial in u is easy to compute from first column in tmp */
    partialU = PPDiff(fv0,fv1);
    partialV = PPDiff(f0u,f1u);

    /* finally, compute the normal...*/
    *normal = VDual(VNormalize(VVCross( partialU, partialV)));

    /* free storage */
    free(tmp);
}


static int loffset(p,u,v)
TPBezier* p;
int u;
int v;
{
	return u*(p->degreeV+1)+v;
}

#define LO(u,v) loffset(&patch, (u), (v))

/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierEvalWithSecondOrder
 * Evaluate the patch at the point whose coordinates are (u,v)
 * for upto second order information.
 *----------------------------------------------------------------------
 */
void TPBezierEvalWithSecondOrder(TPBezier patch, Point *point, 
				 Scalar u, Scalar v,
				 Vector *du, Vector *dv,
				 Vector *duu, Vector *duv, Vector *dvv)
{
    int i;
    Point *tmp;                   /* Temporary net for deCasteljau alg */
    Vector partialU, partialV;
    int iu, iv;                   /* Row, column indices.              */
    int k;                        /* The step of deCasteljau's alg     */
    Point fv0,fv1,f0u,f1u;
    Vector d2u;
    Vector d2uv;
    Vector d2v;

    if ( patch.degreeU < 2  ||  patch.degreeV < 2 ) {
	    fprintf(stderr,"TPBezierEvalWithSecondOrder: too low of degree.\n");
	    exit(1);
    }

    /* Create the temporary storage */
    tmp = (Point *) malloc(sizeof(Point)*(patch.degreeU+1)*(patch.degreeV+1));

    /* Copy the patches net into tmp */
    for (i = 0; i < (patch.degreeU+1)*(patch.degreeV+1); i++) {
      tmp[i] = patch.net[i];
    }

    /* Do de Casteljau's curve algorithm on the rows of the net */
    for (iu = 0; iu <= patch.degreeU; iu++) { /* for each row */
	for (k = 1; k <= patch.degreeV-2; k++) { /* stop w/3 cols to go */
	    for (iv = 0; iv <= patch.degreeV - k; iv++) {
		tmp[LO(iu,iv)] = PPrr( tmp[LO(iu,iv)], tmp[LO(iu,iv+1)], 
				       v, 1.0 - v);
	    }
	}
    }

    /* Now do de Casteljau on the remaining 3 columns of the net */
    for (iv = 0; iv <= 2; iv++){ /* for each column */
      for (k = 1; k <= patch.degreeU-2; k++) { /* stop w/3 rows to go */
	for (iu = 0; iu <= patch.degreeU - k; iu++) {
	  tmp[LO(iu,iv)] = PPrr(tmp[LO(iu,iv)], tmp[LO(iu+1,iv)], u, 1.0-u);
	}
      }
    }

    /* now we have a biquadratic patch in tmp */
    { Point p1[3][3]; Point p2[3][3]; Scalar w[3]; Point pts[3];

      /* make two copies of the biquad patch */
      for (iu = 0; iu <= 2; iu++) {
	      for(iv=0; iv<=2; iv++) {
		      p1[iu][iv] = tmp[LO(iu,iv)];
		      p2[iu][iv] = tmp[LO(iu,iv)];
	      }
      } 

      w[0] = 1; w[1] = -2; w[2] = 1;
      for(k=1; k<=2; k++) {
	      for(iu=0; iu<=2; iu++){
		      for(iv=0; iv<=2-k; iv++){
			      p1[iu][iv] = PPrr(p1[iu][iv], p1[iu][iv+1], 
					       v, 1.-v);
			      p2[iv][iu] = PPrr(p2[iv][iu], p2[iv+1][iu], 
					       u, 1.-u);
		      }
	      }
	      if ( k == 1 ) { /* compute mixed partial */
		      Scalar w[3];
		      Vector ve[3];
Vector t;
		      w[0] = -(1-u); w[1] = ((1-u)-u); w[2] = u;
		      ve[0] = PPDiff(p1[0][1],p1[0][0]);
		      ve[1] = PPDiff(p1[1][1],p1[1][0]);
		      ve[2] = PPDiff(p1[2][1],p1[2][0]);
		      d2uv = SVMult((float)(patch.degreeU*
					    patch.degreeV),
				    VVlcN( 3, ve, w));
		      w[0] = -(1-v); w[1] = ((1-v)-v); w[2] = v;
		      ve[0] = PPDiff(p2[1][0],p2[0][0]);
		      ve[1] = PPDiff(p2[1][1],p2[0][1]);
		      ve[2] = PPDiff(p2[1][2],p2[0][2]);
		      t = SVMult((float)(patch.degreeU*
					 patch.degreeV),
				 VVlcN( 3, ve, w));
if ( fabs(VMag(VVAdd(d2uv,SVMult(-1.,t))))>1e-4){
Scalar x,y,z;
fprintf(stderr,"mixed partials don't match.  ");
VCoords(t,StdFrame(SpaceOf(t)),&x,&y,&z);
fprintf(stderr,"%g %g %g  vs  ",x,y,z);
VCoords(d2uv,StdFrame(SpaceOf(d2uv)),&x,&y,&z);
fprintf(stderr,"%g %g %g\n",x,y,z);
}
	      }
      }
      pts[0] = p1[0][0]; pts[1] = p1[1][0]; pts[2] = p1[2][0];
      d2u = SVMult((float)(patch.degreeU*(patch.degreeU-1)),PPvcN(3, pts, w));
      pts[0] = p2[0][0]; pts[1] = p2[0][1]; pts[2] = p2[0][2];
      d2v = SVMult((float)(patch.degreeV*(patch.degreeV-1)),PPvcN(3, pts, w));
    }


    /* Do de Casteljau's curve algorithm once on the rows of the net */
    for (iu = 0; iu <= 2; iu++) { /* for each row */
	for (iv = 0; iv <= 1; iv++) {
	    int offset = iu*(patch.degreeV+1)+iv;
	    tmp[offset] = PPrr( tmp[offset], tmp[offset+1], v, 1.0 - v);
	}
    }

    /* Now do de Casteljau once on the columns of the net */
    for (iv = 0; iv <= 2; iv++){ /* for each column */
	for (iu = 0; iu <= 1; iu++) {
	  int offset = iu*(patch.degreeV+1)+iv;
	  tmp[offset]=PPrr(tmp[offset],tmp[offset+patch.degreeV+1],u,1.0-u);
	}
    }

    /* now we have a blinear patch in tmp */
    fv0 = PPrr(tmp[LO(0,0)],tmp[LO(0,1)],v,1.0-v);
    fv1 = PPrr(tmp[LO(1,0)],tmp[LO(1,1)],v,1.0-v);
    f0u = PPrr(tmp[LO(0,0)],tmp[LO(1,0)],u,1.0-u);
    f1u = PPrr(tmp[LO(0,1)],tmp[LO(1,1)],u,1.0-u);

    *point = PPrr(fv0,fv1,u,1.0-u);

    /*-----------------------------------------------------*/
    /* Compute the normal by computing partial derivatives */
    /*-----------------------------------------------------*/

    /* partial in u is easy to compute from first column in tmp */
    partialU = SVMult((float)(patch.degreeU), PPDiff(fv0,fv1));
    partialV = SVMult((float)(patch.degreeV), PPDiff(f0u,f1u));


    *du = partialU;
    *dv = partialV;
    *duu = d2u;
    *duv = d2uv;
    *dvv = d2v;

    /* free storage */
    free(tmp);
}

/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierEvalWithNormalAndGK
 * Evaluate the patch at the point whose coordinates are (u,v).
 * Also computed is the normal vector to the surface at the point
 * and the gaussian curvature of the surface at the point.
 *----------------------------------------------------------------------
 */
void TPBezierEvalWithNormalAndGK(TPBezier patch, Point *point, Normal *normal, 
				 Scalar *gk, Scalar u, Scalar v)
{
    Vector partialU, partialV;
    Vector d2u;
    Vector d2uv;
    Vector d2v;

    if ( patch.degreeU < 2  ||  patch.degreeV < 2 ) {
	    TPBezierEvalWithNormal(patch,point,normal,u,v);
	    *gk = 0.;
	    return;
    }

    TPBezierEvalWithSecondOrder(patch, point, u,v, &partialU, &partialV,
			  &d2u, &d2uv, &d2v);

    *normal = VDual(VNormalize(VVCross( partialU, partialV)));
    *gk = GaussianCurvature(d2u,d2uv,d2v,partialU,partialV,*normal);
}

static Scalar C(d,dd)
Vector d;
Vector dd;
{
	Scalar f;

	f = VMag(d);
	return VMag(VVCross(d,dd))/(f*f*f);
}


/*
 *----------------------------------------------------------------------
 *  Function:  TPBezierEvalWithNormalAndSFF
 * Evaluate the patch at the point whose coordinates are (u,v).
 * Also computed is the normal vector to the surface at the point
 * and the gaussian curvature of the surface at the point.
 *----------------------------------------------------------------------
 */
void TPBezierEvalWithNormalAndSFF(TPBezier patch, Point *point, Normal *normal,
				  SFF* sff, Scalar u, Scalar v)
{
    Vector partialU, partialV;
    Vector d2u;
    Vector d2uv;
    Vector d2v;
    Vector n;
    Scalar C();

    if ( patch.degreeU < 2  ||  patch.degreeV < 2 ) {
	    TPBezierEvalWithNormal(patch,point,normal,u,v);
	    return;
    }

    TPBezierEvalWithSecondOrder(patch, point, u,v, &partialU, &partialV,
				&d2u, &d2uv, &d2v);

    *normal = VDual(VNormalize(VVCross( partialU, partialV)));
    sff->v0 = partialU;
    sff->v1 = partialV;
    n = VNormalize(VVCross(partialU,partialV));
    sff->m[0][0] = VVDot(d2u, n);
    sff->m[0][1] = sff->m[1][0] = VVDot(d2uv, n);
    sff->m[1][1] = VVDot(d2v, n);
}
