/*
** geometry.c:
**    Routines to implement 2 and 3 dimensional affine geometry.
**    This package exports the coordinate-free abstractions of affine 
**    spaces, coordinate frames, points, vectors, normal vectors, and 
**    affine transformations.  Implementation details can be found in
**    Tony D. DeRose, "A Coordinate-Free Approach to Geometric Programming,"
**    Siggraph '89 Tutorial Notes, Courses #14 and #23, August 1989.
**
** Copyright (c) 1989, Graphics and AI Laboratory, University of Washington
** Copying, use and development for non-commercial purposes permitted.
**                  All rights for commercial use reserved.
**		    This software is unsupported.
**
** $Header: geometry.c,v 1.12 89/11/15 10:47:53 smann Exp $
**
*/
#include <stdio.h>
#include "geometry.h"

/* Convenient macros */
/* Fill in a row of a matrix */
#define SetRow(mat,row,rowvec)	{ME(mat,row,0) = ME(rowvec,0,0); \
			        ME(mat,row,1) = ME(rowvec,0,1); \
			        ME(mat,row,2) = ME(rowvec,0,2); \
			        ME(mat,row,3) = ME(rowvec,0,3);}

#define GetRow(rowvec,mat,row)  {ME(rowvec,0,0) = ME(mat,row,0); \
			         ME(rowvec,0,1) = ME(mat,row,1); \
			         ME(rowvec,0,2) = ME(mat,row,2); \
			         ME(rowvec,0,3) = ME(mat,row,3);}
			
/* Imports */
extern char *malloc();
extern double fabs();

/* Forward declarations */
extern int DefAffineError();


/* Local Variables */
static int (*AffineError)() = DefAffineError;    /* Error handler */


/*--------------------------------------------------------------*/
/*			Error Handling 				*/
/*--------------------------------------------------------------*/

/*
** Set the affine error handling routine.
*/
void SetAffineError( ErrorFunc)
int (*ErrorFunc)();
{
	AffineError = ErrorFunc;
}

/*
** Default error handling routine.
*/
DefAffineError( errmesg)
char *errmesg;
{
	fprintf(stderr, "Affine error: %s...dumping core.\n", errmesg);
	abort();
}


/*--------------------------------------------------------------*/
/*			Predicates				*/
/*--------------------------------------------------------------*/


/*
** Return true if p really is a point.
*/
int IsPoint(p)
Point p;
{
  if (Dim(SpaceOf(p)) == 3){
    if ( MatrixRows(p.p) == 1  &&  MatrixColumns(p.p) == 4  &&  
	 fabs(ME(p.p, 0,3)-1.)<EPSILON ){
      return 1;
    } else {
      return 0;
    }
  } else {
    if ( MatrixRows(p.p) == 1  &&  MatrixColumns(p.p) == 3  &&  
	 fabs(ME(p.p, 0,2)-1.)<EPSILON ){
      return 1;
    } else {
      return 0;
    }
  }
}


/*
** Return true if v really is a vector.
*/
int IsVector(v)
Vector v;
{
  if (Dim(SpaceOf(v)) == 3){
    if ( MatrixRows(v.v) == 1  &&  MatrixColumns(v.v) == 4  &&  
	 fabs(ME(v.v, 0,3))<EPSILON ){
      return 1;
    } else {
      return 0;
    }
  } else {
    if ( MatrixRows(v.v) == 1  &&  MatrixColumns(v.v) == 3  &&  
	 fabs(ME(v.v, 0,2))<EPSILON ){
      return 1;
    } else {
      return 0;
    }
  }
}


/*
** Return true if n really is a normal.
*/
int IsNormal(n)
Normal n;
{
  if (Dim(SpaceOf(n)) == 3){
    if ( MatrixRows(n.n) == 4  &&  MatrixColumns(n.n) == 1  &&  
	 fabs(ME(n.n, 3,0))<EPSILON ){
      return 1;
    } else {
      return 0;
    }
  } else {
    if ( MatrixRows(n.n) == 3  &&  MatrixColumns(n.n) == 1  &&  
	 fabs(ME(n.n, 2,0))<EPSILON ){
      return 1;
    } else {
      return 0;
    }
  }
}



/*--------------------------------------------------------------*/
/*			Creation Routines			*/
/*--------------------------------------------------------------*/

/*
** Create a new affine space of the specified dimension.
*/
Space SCreate(name, dim)
char *name;
int dim;
{
	Space NewSpace;
	
	NewSpace = (Space) malloc(sizeof(struct euclideanSpace));
	
	NewSpace->dim = dim;
	NewSpace->name = name;
	NewSpace->stdf.s = NewSpace;
	NewSpace->stdf.tostdf = MatrixIdentity( dim+1);
	NewSpace->stdf.fromstdf = MatrixIdentity( dim+1);

	return NewSpace;
}


/*
** Define a new coordinate frame.
*/
Frame FCreate( name, origin, v0, v1, v2)
char *name;
Point origin;
Vector v0, v1, v2;
{
	Frame NewFrame;
	int isValid = TRUE;
	
#ifdef DEBUG
	PPrintf( stderr, origin);
	VPrintf( stderr, v0);
	Vprintf( stderr, v1);
	if (Dim(SpaceOf(origin)) == 3) VPrintf( stderr, v2);
#endif

	if ( !IsPoint(origin) ){
	  (*AffineError)("FCreate: origin not a Point.");
	}
	if ( !IsVector(v0) ){
	  (*AffineError)("FCreate: v0 not a Vector.");
	}
	if ( !IsVector(v1) ){
	  (*AffineError)("FCreate: v1 not a Vector.");
	}

	isValid = (SpaceOf(origin) == SpaceOf(v0)) && (SpaceOf(origin) == SpaceOf(v1));
	if (Dim(SpaceOf(origin)) == 3) {
		isValid = isValid && (SpaceOf(origin) == SpaceOf(v2));
		if ( !IsVector(v2) ){
		  (*AffineError)("FCreate: v2 not a Vector.");
		}
	}
	if (isValid) {
	    	
	    SpaceOf(NewFrame) = SpaceOf(origin);
	    NewFrame.name = name;
	    
	    switch (Dim(SpaceOf(NewFrame))) {
	    case 2:
		/* Create a frame for an affine 2-space */
	    	NewFrame.tostdf = MatrixZero( 3);

	    	/* The ith row of tostdf consists stdcoords of vi */
	    	SetRow(NewFrame.tostdf, 0, v0.v);
	    	SetRow(NewFrame.tostdf, 1, v1.v);

	    	/* The last row consists of stdcoords of origin */
	    	SetRow(NewFrame.tostdf, 2, origin.p);
		break;
	    case 3:
		/* Create a frame for an affine 3-space */
	    	NewFrame.tostdf = MatrixZero( 4);

	    	/* The ith row of tostdf consists stdcoords of vi */
	    	SetRow(NewFrame.tostdf, 0, v0.v);
	    	SetRow(NewFrame.tostdf, 1, v1.v);
	    	SetRow(NewFrame.tostdf, 2, v2.v);

	    	/* The last row consists of stdcoords of origin */
	    	SetRow(NewFrame.tostdf, 3, origin.p);
		break;
	    }

	    /* Check to make sure a valid frame has been specified */
	    if (fabs(MatrixDet( NewFrame.tostdf)) < EPSILON) {
#ifdef DEBUG
		MatrixPrint( NewFrame.tostdf, stderr);
#endif
	    	(*AffineError)("FCreate: Vectors not linearly independent.");
	    } else {
		/* A valid frame, so compute fromstdf */
	    	NewFrame.fromstdf = MatrixInverse( NewFrame.tostdf);
	    }
	    
	} else {
	    (*AffineError)("FCreate: elements not from same space.");
	}
	return NewFrame;
}
	
/*
** Create and return a new point.  The coordinates of the
** point are (c0,c1,[c2,]1) relative to the frame F.  The
** coordinate c2 is only used if F spans a 3-space.
*/
Point PCreate( F, c0, c1, c2)
Frame F;
Scalar c0, c1, c2;
{
	Point NewPoint;
	
	SpaceOf(NewPoint) = SpaceOf(F);
	switch (Dim(SpaceOf(NewPoint))) {
	case 2:
	    NewPoint.p = MatrixCreate( 1, 3);

	    /* Compute standard coords */
	    ME(NewPoint.p, 0, 0) = c0;
	    ME(NewPoint.p, 0, 1) = c1;
	    ME(NewPoint.p, 0, 2) = 1.0;
	    break;
	case 3:
	    NewPoint.p = MatrixCreate( 1, 4);

	    /* Compute standard coords */
	    ME(NewPoint.p, 0, 0) = c0;
	    ME(NewPoint.p, 0, 1) = c1;
	    ME(NewPoint.p, 0, 2) = c2;
	    ME(NewPoint.p, 0, 3) = 1.0;
	    break;
	}

	NewPoint.p = MatrixMult( NewPoint.p, F.tostdf);
	return NewPoint;
}

	
/*
** Create and return a new vector.  The coordinates of the
** vector are (c0,c1,[c2,]1) relative to the frame F.  The
** coordinate c2 is only used if F spans a 3-space.
*/
Vector VCreate( F, c0, c1, c2)
Frame F;
Scalar c0, c1, c2;
{
	Vector NewVector;
	
	SpaceOf(NewVector) = SpaceOf(F);
	switch (Dim(SpaceOf(NewVector))) {
	case 2:
	    NewVector.v = MatrixCreate( 1, 3);

	    /* Compute standard coords */
	    ME(NewVector.v, 0, 0) = c0;
	    ME(NewVector.v, 0, 1) = c1;
	    ME(NewVector.v, 0, 2) = 0.0;
	    break;
	case 3:
	    NewVector.v = MatrixCreate( 1, 4);

	    /* Compute standard coords */
	    ME(NewVector.v, 0, 0) = c0;
	    ME(NewVector.v, 0, 1) = c1;
	    ME(NewVector.v, 0, 2) = c2;
	    ME(NewVector.v, 0, 3) = 0.0;
	    break;
	}

	NewVector.v = MatrixMult( NewVector.v, F.tostdf);
	return NewVector;
}

/*
** Create and return a new normal vector.
*/
Normal NCreate( f, c0, c1, c2)
Frame f;
Scalar c0, c1, c2;
{
	return VDual( VCreate( f, c0, c1, c2));
}

/*
** Return the zero vector in space S.
*/
Vector VZero(S)
Space S;
{
	Vector ZeroVec;
	
	SpaceOf(ZeroVec) = S;
	ZeroVec.v = MatrixCreate(1, Dim(SpaceOf(ZeroVec)) + 1);
	
	return ZeroVec;
}

/*
** Return the transformation that carries the origin of F onto
** Oprime, and the basis in F onto v0prime, v1prime, [and v2prime.]
*/
AffineMap ACreate( F, Oprime, v0prime, v1prime, v2prime)
Frame F;
Point Oprime;
Vector v0prime, v1prime, v2prime;
{
	AffineMap T;			/* The returned transform */
	Matrix Tprime;			/* Rows built from primed objects */
	int isValid = 1;
	

	if ( !IsPoint(Oprime) ){
	  (*AffineError)("ACreate: Oprime not a Point.");
	}
	if ( !IsVector(v0prime) ){
	  (*AffineError)("ACreate: v0prime not a Vector.");
	}
	if ( !IsVector(v1prime) ){
	  (*AffineError)("ACreate: v1prime not a Vector.");
	}

	isValid = (SpaceOf(Oprime) == SpaceOf(v0prime)) &&
	    	  (SpaceOf(Oprime) == SpaceOf(v1prime));
	if (Dim(SpaceOf(F)) == 3) {
	  	if ( !IsVector(v2prime) ){
	    	  (*AffineError)("ACreate: v2prime not a Vector.");
	  	}
		isValid = isValid && (SpaceOf(Oprime) == SpaceOf(v2prime));
	}
	if (isValid) {
		T.domain = SpaceOf(F);
		T.range  = SpaceOf(Oprime);

		/* Assume for now that the map isn't invertible */
		T.invertible = FALSE;
		
		/* Build Tprime */
		Tprime = MatrixCreate( Dim(SpaceOf(F))+1, Dim(SpaceOf(Oprime))+1);

		switch (Dim(SpaceOf(F))) {
		case 2:
		    SetRow(Tprime, 0, v0prime.v);
		    SetRow(Tprime, 1, v1prime.v);
		    SetRow(Tprime, 2, Oprime.p);
		    break;
		case 3:
		    SetRow(Tprime, 0, v0prime.v);
		    SetRow(Tprime, 1, v1prime.v);
		    SetRow(Tprime, 2, v2prime.v);
		    SetRow(Tprime, 3, Oprime.p);
		    break;
		}
		    
		/*-----------------------------------------*/
		/* T.t satisfies: F.tostdf * T.t = Tprime, */
		/* So T.t = F.fromstdf * Tprime.           */
		/*-----------------------------------------*/
		T.t = MatrixMult( F.fromstdf, Tprime);

		/*-------------------------------------------------------*/
		/* If T's domain and range have the same dimension, then */
		/* the inverse matrix may also be stored.                */
		/*-------------------------------------------------------*/
		if (Dim(SpaceOf(F)) == Dim(SpaceOf(Oprime))) {
		    /* See if T.t is invertible */
		    if (fabs(MatrixDet(T.t)) > EPSILON) {
			T.invt = MatrixInverse( T.t);
			T.invertible = TRUE;
		    }
		}
	} else {
		(*AffineError)("ACreate: image objects not in common space.");
	}
#ifdef DEBUG
	fprintf( stderr, "ACreated...\n");
	TPrintf( stderr, T);
#endif
	return T;
}


/*
** Return the affine transformation that carries the frame
** F1 onto the frame F2.  This could be done by calling ACreate,
** but the routine provided is a bit more efficient.
*/
AffineMap ACreateF( F1, F2)
Frame F1, F2;
{
	AffineMap T;

	if (Dim(SpaceOf(F1)) != Dim(SpaceOf(F2))) {
		(*AffineError)("ACreate: dim(F1) != dim(F2).");
	} else {
		T.domain = SpaceOf(F1);
		T.range = SpaceOf(F2);
		T.invertible = TRUE;
		T.t = MatrixMult( F1.fromstdf, F2.tostdf);
		T.invt = MatrixInverse(T.t);
	}
	return T;
}


/*
** Return the identity transformation from space S onto itself.
*/
AffineMap AIdentity(S)
Space S;
{
	AffineMap Identity;
	
	Identity.domain = S;
	Identity.range = S;
	Identity.invertible = TRUE;

	Identity.t = MatrixIdentity( Dim(S) + 1);
	Identity.invt = Identity.t;
	return Identity;
}




/*--------------------------------------------------------------*/
/*			Coordinate Routines			*/
/*--------------------------------------------------------------*/


/*
** Return the coordinates of the point p relative to the frame F.
*/
void PCoords( p, F, c0, c1, c2)
Point p;
Frame F;
Scalar *c0, *c1, *c2;
{
	Matrix Coords;
	
	if ( !IsPoint(p) ){
	  (*AffineError)("PCoords: p not a Point.");
	}

	Coords = MatrixMult( p.p, F.fromstdf);
	*c0 = ME(Coords,0,0);
	*c1 = ME(Coords,0,1);

	if (Dim(SpaceOf(p)) == 3) {
		*c2 = ME(Coords,0,2);
	}
}


/*
** Return the coordinates of the vector v relative to the frame F.
*/
void VCoords( v, F, c0, c1, c2)
Vector v;
Frame F;
Scalar *c0, *c1, *c2;
{
	Matrix Coords;
	
	if ( !IsVector(v) ){
	  (*AffineError)("VCoords: v not a Vector.");
	}

	Coords = MatrixMult( v.v, F.fromstdf);
	*c0 = ME(Coords,0,0);
	*c1 = ME(Coords,0,1);

	if (Dim(SpaceOf(v)) == 3) {
		*c2 = ME(Coords,0,2);
	}
}

/*
** Return the coordinates of the normal vector n relative to the frame F.
*/
void NCoords( n, F, c0, c1, c2)
Normal n;
Frame F;
Scalar *c0, *c1, *c2;
{
	Matrix Coords;
	
	if ( !IsNormal(n) ){
	  (*AffineError)("NCoords: n not a Normal.");
	}

	Coords = MatrixMult( MatrixTranspose(n.n), F.fromstdf);
	*c0 = ME(Coords,0,0);
	*c1 = ME(Coords,0,1);

	if (Dim(SpaceOf(n)) == 3) {
		*c2 = ME(Coords,0,2);
	}
}

/*
** Return the origin of the given frame.
*/
Point FOrg(F)
Frame F;
{
	Point Origin;
	
	SpaceOf(Origin) = SpaceOf(F);
	Origin.p = MatrixCreate( 1, Dim(SpaceOf(Origin)) + 1);

	if (Dim(SpaceOf(Origin)) == 2) {
		GetRow(Origin.p, F.tostdf, 2);
	} else {
		GetRow(Origin.p, F.tostdf, 3);
	}

	return Origin;
}

/*
** Return the ith basis vector in F (numbered starting at 0).
*/
Vector FV(F,i)
Frame F;
int i;
{
	Vector Bi;
	
	SpaceOf(Bi) = SpaceOf(F);
	Bi.v = MatrixCreate( 1, Dim(SpaceOf(Bi)) + 1);
	GetRow(Bi.v, F.tostdf, i);

	return Bi;
}


/*--------------------------------------------------------------*/
/*			Affine Space Operators			*/
/*--------------------------------------------------------------*/

/*
** Return the vector difference of two points.  Returned
** is p-q, the vector from q to p.
*/
Vector PPDiff( p, q)
Point p, q;
{
	Vector v;
	
	if ( !IsPoint(p) ){
	  (*AffineError)("PPDiff: p not a Point.");
	}

	if ( !IsPoint(q) ){
	  (*AffineError)("PPDiff: q not a Point.");
	}

	if (SpaceOf(p) != SpaceOf(q)) {
		(*AffineError)("PPDiff: points not in same space.");
	}
	SpaceOf(v) = SpaceOf(p);
	v.v = MatrixCreate( 1, Dim(SpaceOf(v))+1);
	ME(v.v,0,0) = ME(p.p,0,0) - ME(q.p,0,0);
	ME(v.v,0,1) = ME(p.p,0,1) - ME(q.p,0,1);
	if (Dim(SpaceOf(v)) == 3) {
		ME(v.v,0,2) = ME(p.p,0,2) - ME(q.p,0,2);
		ME(v.v,0,3) = 0.0;
	} else {
		ME(v.v,0,2) = 0.0;
	}
	
	return v;
}

/*
** Return the point q = p + v.
*/
Point PVAdd( p, v)
Point p;
Vector v;
{
	if ( !IsPoint(p) ){
	  (*AffineError)("PVAdd: p not a Point.");
	}

	if ( !IsVector(v) ){
	  (*AffineError)("PVAdd: v not a Vector.");
	}

	if (SpaceOf(p) != SpaceOf(v)) {
		(*AffineError)("PVAdd: point and vector from different spaces.");
	}
	ME(p.p,0,0) += ME(v.v,0,0);
	ME(p.p,0,1) += ME(v.v,0,1);
	if (Dim(SpaceOf(p)) == 3) {
		ME(p.p,0,2) += ME(v.v,0,2);
		ME(p.p,0,3) = 1.0;
	} else {
		ME(p.p,0,2) = 1.0;
	}
	return p;
}

	
/*
** Return the point that breaks line segment p1,p2 into
** relative ratios r1 and r2.
*/
Point PPrr( p1, p2, r1, r2)
Point p1, p2;
Scalar r1, r2;
{
	if ( !IsPoint(p1) ){
	  (*AffineError)("PPrr: p1 not a Point.");
	}

	if ( !IsPoint(p2) ){
	  (*AffineError)("PPrr: p2 not a Point.");
	}

	return PPac( p1, p2, r2/(r2+r1));
}

/*
** Return the point a1*p1 + (1-a1)*p2.
**
** This point can also be written as p2 + a1*(p1-p2), which
** is how its actually calculated.
*/
Point PPac( p1, p2, a1)
Point p1, p2;
Scalar a1;
{
	if ( !IsPoint(p1) ){
	  (*AffineError)("PPac: p1 not a Point.");
	}

	if ( !IsPoint(p2) ){
	  (*AffineError)("PPac: p2 not a Point.");
	}

	return PVAdd( p2, SVMult( a1, PPDiff( p1, p2)));
}


/*
** Return the point a1*p1 + a2*p2 + a3*p3, where
** a1 + a2 + a3 = 1.
*/
Point PPac3( p1, p2, p3, a1, a2, a3)
Point p1, p2, p3;
Scalar a1, a2, a3;
{
    Point p[3];
    Scalar a[3];

    if ( !IsPoint(p1) ){
      (*AffineError)("PPac3: p1 not a Point.");
    }

    if ( !IsPoint(p2) ){
      (*AffineError)("PPac3: p2 not a Point.");
    }

    if ( !IsPoint(p3) ){
      (*AffineError)("PPac3: p3 not a Point.");
    }

    p[0] = p1;  p[1] = p2;  p[2] = p3;
    a[0] = a1;  a[1] = a2;  a[2] = a3;
    return PPacN( 3, p, a);
}

    
/*
** Do an affine combination of n points. Returned is the point 
**
**       sum_{i=0}^{n-1} a[i] * p[i]
**
** where the a's sum to one.
*/
Point PPacN( n, p, a)
int n;
Point p[];
Scalar a[];
{
    Scalar x, y, z, sum;
    Space S = SpaceOf(p[0]);
    int i, dim = Dim(SpaceOf(p[0]));

    /* For efficiency, appeal to coordinates */
    x = 0.0;  y = 0.0;  z = 0.0;  sum = 0.0;
    for (i = 0; i < n; i++) {
        if ( !IsPoint(p[i]) ){
	   (*AffineError)("PPacN: non-Point passed as Point.");
    	}

	if (SpaceOf(p[i]) != S) {
	    (*AffineError)("PPacN: points not from a common space.\n");
	}
	sum += a[i];
	x += a[i] * ME(p[i].p,0,0);
	y += a[i] * ME(p[i].p,0,1);
	if (dim == 3) {
	    z += a[i] * ME(p[i].p,0,2);
	}
    }
    if (fabs(sum - 1.0) > EPSILON) {
	(*AffineError)("PPacN: coefficients don't sum to one.\n");
    }
    if ( dim == 3 ) {
      return PCreate( StdFrame(S), x, y, z);
    } else {
      return PCreate( StdFrame(S), x, y);
    }
}


    
/*
** Do a "vector combination" of n points. Returned is the vector
**
**       sum_{i=0}^{n-1} a[i] * p[i]
**
** where the a's sum to zero.
*/
Vector PPvcN( n, p, a)
int n;
Point p[];
Scalar a[];
{
    Scalar x, y, z, sum;
    Space S = SpaceOf(p[0]);
    int i, dim = Dim(SpaceOf(p[0]));

    /* For efficiency, appeal to coordinates */
    x = 0.0;  y = 0.0;  z = 0.0;  sum = 0.0;
    for (i = 0; i < n; i++) {
        if ( !IsPoint(p[i]) ){
	   (*AffineError)("PPvcN: non-Point passed as Point.");
    	}

	if (SpaceOf(p[i]) != S) {
	    (*AffineError)("PPvcN: points not from a common space.");
	}
	sum += a[i];
	x += a[i] * ME(p[i].p,0,0);
	y += a[i] * ME(p[i].p,0,1);
	if (dim == 3) {
	    z += a[i] * ME(p[i].p,0,2);
	}
    }
    if (fabs(sum) > EPSILON) {
	(*AffineError)("PPvcN: coefficients don't sum to zero.");
    }
    if ( dim == 3 ) {
      return VCreate( StdFrame(S), x, y, z);
    } else {
      return VCreate( StdFrame(S), x, y);
    }
}

	
/*
** Return the outward pointing normal vector to the plane defined
** by p1, p2 and p3.  The normal points toward a viewer positioned
** so that p1,p2,p3 is counterclockwise.
** pointing.
*/
Normal PPPNormal( p1, p2, p3)
Point p1, p2, p3;
{
  if ( !IsPoint(p1) ){
    (*AffineError)("PPPNormal: p1 not a Point.");
  }

  if ( !IsPoint(p2) ){
    (*AffineError)("PPPNormal: p2 not a Point.");
  }

  if ( !IsPoint(p3) ){
    (*AffineError)("PPPNormal: p3 not a Point.");
  }

  return VDual(VVCross( PPDiff( p2, p1), PPDiff( p3, p1)));
}


/*--------------------------------------------------------------*/
/*			Vector Space Operators			*/
/*--------------------------------------------------------------*/

/*
** Scalar-Vector Multiplication.
*/
Vector SVMult( scalar, vector)
Scalar scalar;
Vector vector;
{
	Vector Vprime;
	
	if ( !IsVector(vector) ){
	  (*AffineError)("SVMult: vector is not a Vector.");
	}

	SpaceOf(Vprime) = SpaceOf(vector);
	Vprime.v = MatrixCreate( 1, Dim(SpaceOf(Vprime)) + 1);
	ME(Vprime.v,0,0) = ME(vector.v,0,0) * scalar;
	ME(Vprime.v,0,1) = ME(vector.v,0,1) * scalar;
	if ( Dim(SpaceOf(vector)) == 3 ) {
	  ME(Vprime.v,0,2) = ME(vector.v,0,2) * scalar;
	  ME(Vprime.v,0,3) = 0.0;
	} else {
	  ME(Vprime.v,0,2) = 0.0;
	}

	return Vprime;
}

/*
** Return the sum of the two vectors.
*/
Vector VVAdd( v1, v2)
Vector v1, v2;
{
    Vector v;

    if ( !IsVector(v1) ){
	 (*AffineError)("VVAdd: v1 is not a Vector.");
    }

    if ( !IsVector(v2) ){
	 (*AffineError)("VVAdd: v2 is not a Vector.");
    }

    if (SpaceOf(v1) != SpaceOf(v2)) {
	(*AffineError)("VVAdd: vectors from different spaces.");
    }
    SpaceOf(v) = SpaceOf(v1);
    v.v = MatrixCreate( 1, Dim(SpaceOf(v1)) + 1);
    ME(v.v,0,0) = ME(v1.v,0,0)+ME(v2.v,0,0);
    ME(v.v,0,1) = ME(v1.v,0,1)+ME(v2.v,0,1);
    if (Dim(SpaceOf(v)) == 3) {
	ME(v.v,0,2) = ME(v1.v,0,2)+ME(v2.v,0,2);
	ME(v.v,0,3) = 0.0;
    } else {
	ME(v.v,0,2) = 0.0;
    }
    return v;
}

    
/*
** Do a linear combination of n vectors. Returned is the vector
**
**       sum_{i=0}^{n-1} a[i] * v[i]
**
*/
Vector VVlcN( n, v, a)
int n;
Vector v[];
Scalar a[];
{
    Scalar x, y, z;
    Space S = SpaceOf(v[0]);
    int i, dim = Dim(SpaceOf(v[0]));

    /* For efficiency, appeal to coordinates */
    x = 0.0;  y = 0.0;  z = 0.0;
    for (i = 0; i < n; i++) {

        if ( !IsVector(v[i]) ){
	   (*AffineError)("VVlcN: non-Vector passed as Vector.");
        }

	if (SpaceOf(v[i]) != S) {
	    (*AffineError)("VVlcN: vectors not from a common space.\n");
	}
	x += a[i] * ME(v[i].v,0,0);
	y += a[i] * ME(v[i].v,0,1);
	if (dim == 3) {
	    z += a[i] * ME(v[i].v,0,2);
	}
    }
    if ( dim == 3 ) {
      return VCreate( StdFrame(S), x, y, z);
    } else {
      return VCreate( StdFrame(S), x, y);
    }
}


/*
** Return the difference of vectors: v1 - v2;
*/
Vector VVDiff( v1, v2)
Vector v1, v2;
{
    Vector v;

    if ( !IsVector(v1) ){
      (*AffineError)("VVDiff: v1 is not a Vector.");
    }

    if ( !IsVector(v2) ){
      (*AffineError)("VVDiff: v2 is not a Vector.");
    }

    if (SpaceOf(v1) != SpaceOf(v2)) {
	(*AffineError)("VVDiff: vectors from different spaces.");
    }
    SpaceOf(v) = SpaceOf(v1);
    v.v = MatrixCreate( 1, Dim(SpaceOf(v1)) + 1);
    ME(v.v,0,0) = ME(v1.v,0,0) - ME(v2.v,0,0);
    ME(v.v,0,1) = ME(v1.v,0,1) - ME(v2.v,0,1);
    if (Dim(SpaceOf(v)) == 3) {
	ME(v.v,0,2) = ME(v1.v,0,2) - ME(v2.v,0,2);
	ME(v.v,0,3) = 0.0;
    } else {
	ME(v.v,0,2) = 0.0;
      }
    return v;
}



/*
** Return the geometric dot product of v1 and v2.
*/
Scalar VVDot( v1, v2)
Vector v1, v2;
{
	Scalar dot;

	if ( !IsVector(v1) ){
	  (*AffineError)("VVDot: v1 is not a Vector.");
	}

	if ( !IsVector(v2) ){
	  (*AffineError)("VVDot: v2 is not a Vector.");
	}

	if (SpaceOf(v1) != SpaceOf(v2)) {
		(*AffineError)("VVDot: vectors from different spaces.");
	}
	dot = ME(v1.v,0,0)*ME(v2.v,0,0) + ME(v1.v,0,1)*ME(v2.v,0,1);
	if (Dim(SpaceOf(v1)) == 3) {
		dot += ME(v1.v,0,2)*ME(v2.v,0,2);
	}
	return dot;
}

	
/*
** Return the vector given by v1 cross v2.
*/
Vector VVCross( v1, v2)
Vector v1, v2;
{
	Vector v;
	
	if ( !IsVector(v1) ){
	  (*AffineError)("VVCross: v1 is not a Vector.");
	}

	if ( !IsVector(v2) ){
	  (*AffineError)("VVCross: v2 is not a Vector.");
	}

	if (SpaceOf(v1) != SpaceOf(v2)) {
		(*AffineError)("VVCross: vectors from different spaces.");
	}
	if (Dim(SpaceOf(v1)) != 3) {
		(*AffineError)("VVCross: vectors must be from a 3-space.");
	}
	SpaceOf(v) = SpaceOf(v1);
	v.v = MatrixCreate( 1, Dim(SpaceOf(v1)) + 1);
	ME(v.v,0,0) = ME(v1.v,0,1)*ME(v2.v,0,2) - ME(v1.v,0,2)*ME(v2.v,0,1);
	ME(v.v,0,1) = ME(v1.v,0,2)*ME(v2.v,0,0) - ME(v1.v,0,0)*ME(v2.v,0,2);
	ME(v.v,0,2) = ME(v1.v,0,0)*ME(v2.v,0,1) - ME(v1.v,0,1)*ME(v2.v,0,0);
	
	return v;
}


/*
** Return the orthogonal projection of v onto w.
*/
Vector VVProj( v, w)
Vector v, w;
{
    Scalar factor;

    if ( !IsVector(v) ){
      (*AffineError)("VVProj: v is not a Vector.");
    }

    if ( !IsVector(w) ){
      (*AffineError)("VVProj: w is not a Vector.");
    }

    factor = VVDot( v, w) / VVDot( w, w);
    return SVMult( factor, w);
}


/*
** Turn the primal vector into a normal.  In vector space
** terminology, v is mapping to a linear functional according
** to the rule v --> phi_v(w) = (v .)(w), where (.) is the dot
** product.  Normals are represented as column matrices.
*/
Normal VDual(v)
Vector v;
{
	Normal phi;
	
	if ( !IsVector(v) ){
	  (*AffineError)("VDual: v is not a Vector.");
	}

	SpaceOf(phi) = SpaceOf(v);
	phi.n = MatrixTranspose( v.v);
	
	return phi;
}


/*
** Turn the normal vector into a primal vector.
*/
Vector NDual(phi)
Normal phi;
{
	Vector v;
	
	if ( !IsNormal(phi) ){
	  (*AffineError)("NDual: phi is not a Normal.");
	}

	SpaceOf(v) = SpaceOf(phi);
	v.v = MatrixTranspose( phi.n);

	return v;
}


/*
** Apply a linear functional (ie a normal vector) to a
** primal vector.  
*/
Scalar NVApply( phi, v)
Normal phi;
Vector v;
{
    Matrix M;

    if ( !IsNormal(phi) ){
      (*AffineError)("NVApply: phi is not a Normal.");
    }

    if ( !IsVector(v) ){
      (*AffineError)("NVApply: v is not a Vector.");
    }

    if (SpaceOf(phi) != SpaceOf(v)) {
	(*AffineError)("NVApply: vector and normal not from a common space.");
    }
    M = MatrixMult( v.v, phi.n);
    return ME(M, 0, 0);
}
	

/*
** Normalize the given vector.
*/
Vector VNormalize( v)
Vector v;
{
	Scalar mag = VMag(v);
	
	if ( !IsVector(v) ){
	   (*AffineError)("VNormalize: v is not a Vector.");
	}

	if (mag < EPSILON) {
		(*AffineError)("VNormalize: can't normalize zero vector");
	}
	
	return SVMult( 1.0/mag, v);
}

/*--------------------------------------------------------------*/
/*			Transformation Routines			*/
/*--------------------------------------------------------------*/

/*
** Return the point p transformed by transformation T.
*/
Point PAxform( p, T)
Point p;
AffineMap T;
{
	Point pp;			/* pp = T(p) = */
	
	if ( !IsPoint(p) ){
	   (*AffineError)("PAxform: p is not a Point.");
	}

	if (SpaceOf(p) != T.domain) {
		(*AffineError)("PAxform: point not in domain space.");
		return pp;
	}
	SpaceOf(pp) = T.range;
	pp.p = MatrixMult( p.p, T.t);
#ifdef DEBUG
	fprintf(stderr, "AffineMap: ");
	PPrintf(stderr, p);
	fprintf(stderr, " --> ");
	PPrintf(stderr, pp);
	fprintf(stderr, "\n");
#endif
	return pp;
}


/*
** Return the vector v transformed by transformation T.
*/
Vector VAxform( v, T)
Vector v;
AffineMap T;
{
	Vector vv;			/* vv = T(v) = */
	
	if ( !IsVector(v) ){
	   (*AffineError)("VAxform: v is not a Vector.");
	}

	if (SpaceOf(v) != T.domain) {
		(*AffineError)("VAxform: vector not in domain space.");
		return vv;
	}
	SpaceOf(vv) = T.range;
	vv.v = MatrixMult( v.v, T.t);
#ifdef DEBUG
	fprintf(stderr, "AffineMap: ");
	VPrintf(stderr, v);
	fprintf(stderr, " --> ");
	VPrintf(stderr, vv);
	fprintf(stderr, "\n");
#endif
	return vv;
}



/*
** Return the linear functional (ie, normal vector) transformed by
** transformation T.
*/
Normal NAxform( n, T)
Normal n;
AffineMap T;
{
	Normal nn;
	
	if ( !IsNormal(n) ){
	   (*AffineError)("NAxform: n is not a Normal.");
	}

	if (SpaceOf(n) != T.domain) {
		(*AffineError)("NAxform: normal not in domain space.");
		return nn;
	}
	SpaceOf(nn) = T.range;

	/* Normals transform by multiplication on the left by the   */
	/* inverse of the matrix that transforms vectors and points.*/
	if (!T.invertible) {
	  (*AffineError)
	    ("NAxform: attempt to map a normal through a non-invertible map.");
	}
	nn.n = MatrixMult( T.invt, n.n);

	/* Only the linear portion of T should have been inverted.     */
	/* To compensate, we must zero out the last component manually.*/
	switch (Dim(SpaceOf(nn))) {
	  case 2:
	    ME(nn.n,2,0) = 0.0;
	    break;
	  case 3:
	    ME(nn.n,3,0) = 0.0;
	    break;
	}

#ifdef DEBUG
	fprintf(stderr, "AffineMap: ");
	NPrintf(stderr, n);
	fprintf(stderr, " --> ");
	NPrintf(stderr, nn);
	fprintf(stderr, "\n");
#endif
	return nn;
}


/*
** Return the composite transformation T2(T1(.)).
*/
AffineMap AACompose( T1, T2)
AffineMap T1, T2;
{
    AffineMap T;

    if (T1.range != T2.domain) {
	(*AffineError)("AACompose: Range and domains don't match.");
    } else {
	T.domain = T1.domain;
	T.range  = T2.range;
	T.t = MatrixMult( T1.t, T2.t);
	if (T1.invertible && T2.invertible) {
	    T.invertible = TRUE;
	    T.invt = MatrixMult( T2.invt, T1.invt);
	} else {
	    T.invertible = FALSE;
	}
    }
    return T;
}

    

/*--------------------------------------------------------------*/
/*			Utility Routines			*/
/*--------------------------------------------------------------*/


F_Printf( fp, F)
FILE *fp;
Frame F;
{
	fprintf( fp, "Frame:\n");
	VPrintf( fp, FV(F,0));
	VPrintf( fp, FV(F,1));
	if (Dim(SpaceOf(F))==3) VPrintf( fp, FV(F,2));
	PPrintf( fp, FOrg(F));
}

PPrintf( fp, p)
FILE *fp;
Point p;
{
	fprintf(fp, "Point: space: %s ", Name(SpaceOf(p)));
	MatrixPrint(p.p, fp);
}

VPrintf( fp, v)
FILE *fp;
Vector v;
{
	fprintf(fp, "Vector: space: %s ", Name(SpaceOf(v)));
	MatrixPrint(v.v, fp);
}

APrintf( fp, T)
FILE *fp;
AffineMap T;
{
	fprintf( fp, "AffineMap: domain: %s, range: %s\n",
				 Name(T.domain), Name(T.range));
	MatrixPrint( T.t, fp);
}

