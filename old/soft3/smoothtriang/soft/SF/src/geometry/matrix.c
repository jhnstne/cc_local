/*
** matrix.c: Routines to implement a general matrix package.
**
** $Header: matrix.c,v 1.4 89/05/17 08:52:27 derose Locked $
**
** Matrices are represented by square arrays of ``Scalars''
** (Scalar must be either of type float or double.)  Associated
** with a matrix is the number of rows and columns it possesses.
**
** Copyright (c) 1989, Graphics and AI Laboratory, University of Washington
** Copying, use and development for non-commercial purposes permitted.
**                  All rights for commercial use reserved.
**		    This software is unsupported.
**
*/
#include <stdio.h>
#include "matrix.h"


/* 3x3 determinant macro */
#define det3x3(m00,m01,m02,m10,m11,m12,m20,m21,m22) \
		 ( m00*(m11*m22-m21*m12) \
		  -m01*(m10*m22-m20*m12) \
		  +m02*(m10*m21-m20*m11))

/* Imports */
extern double fabs();

/* Forward declarations */
extern int DefMatrixError();

int (*MatrixError)() = DefMatrixError;	/* Error handling routine */

/*
** Set the matrix error handling routine.
*/
void SetMatrixError( ErrorFunc)
int (*ErrorFunc)();
{
	MatrixError = ErrorFunc;
}


/*
** Default error handling routine.
*/
DefMatrixError( errmesg)
char *errmesg;
{
	fprintf(stderr, "Matrix error: %s...dumping core.\n", errmesg);
	abort();
}


/*
** Return TRUE if the matrix is ``affine'', i.e., if it
** is a matrix whose last column is of the form
** [0.0 ... 0.0 1.0]^T.  Return FALSE otherwise.
*/
IsAffineMatrix(M)
Matrix M;
{
	int i;
	
	if (IsNullMatrix(M)) {
		return FALSE;
	}

	for (i = 0; i < M.rows-1; i++) {
		if (M.x[i][M.cols-1] != 0.0) {
			return FALSE;
		}
	}
	return (M.x[M.rows-1][M.cols-1] == 1.0);
}


/*
** Print a matrix
*/
void MatrixPrint(M, fp)
Matrix M;
FILE *fp;
{
	int i, j;
	
	if (IsNullMatrix(M)) {
		fprintf(fp, "[]\n");
		return;
	}
	for (i = 0; i < M.rows; i++) {
		fprintf(fp, "[");
		for (j = 0; j < M.cols; j++) {
			fprintf(fp, " %g ", (float) M.x[i][j]);
		}
		fprintf(fp, "]\n");
	}
}

/*
** Return the Null matrix
*/
Matrix MatrixNull()
{
	Matrix Null;
	
	Null.rows = Null.cols = 0;
	return Null;
}


/*
** Create and initialize to zero a matrix of the given size.
*/
Matrix MatrixCreate(rows, cols)
int rows, cols;
{
	Matrix Z;
	int i,j;	

	if (rows > MAX_MAT_SIZE || cols > MAX_MAT_SIZE) {
		(*MatrixError)("Attempt to create zero larger than max.");
		return MatrixNull();
	}
	Z.rows = rows;
	Z.cols = cols;

	for (i = 0; i < Z.rows; i++) {
		for (j = 0; j < Z.cols; j++) {
			Z.x[i][j] = 0.0;
		}
	}
	return Z;
}

/*
** Return the identity matrix of the given dimension.
*/
Matrix MatrixIdentity(dim)
int dim;
{
	Matrix I;
	int i,j;	

	if (dim > MAX_MAT_SIZE) {
		(*MatrixError)("Attempt to create identity larger than max.");
		return MatrixNull();
	}
	I.rows = I.cols = dim;

	for (i = 0; i < I.rows; i++) {
		for (j = 0; j < I.cols; j++) {
			I.x[i][j] = ( i == j ? 1.0 : 0.0);
		}
	}
	return I;
}

/*
** Multiply a matrix by a scale factor.
*/
Matrix MatrixScale( s, M)
Scalar s;
Matrix M;
{
	int i, j;
	
	for (i = 0; i < M.rows; i++) {
		for (j = 0; j < M.cols; j++) {
			M.x[i][j] *= s;
		}
	}
	return M;
}


/*
** Multiply two matrices.
** On an error, call MatrixError.
*/
Matrix MatrixMult( A, B)
Matrix A, B;
{
	int i,j,k;
	Matrix C;
	
	if (A.cols != B.rows) {
		(*MatrixError)( "Attempt to multiply matrices of incompatible dimensions.");
		return MatrixNull();
	}

	C.rows = A.rows;
	C.cols = B.cols;
	
	for (i = 0; i < C.rows; i++) {
		for (j = 0; j < C.cols; j++) {
			C.x[i][j] = 0.0;
			for (k = 0; k < A.cols; k++) {
				C.x[i][j] += A.x[i][k] * B.x[k][j];
			}
		}
	}
	return C;
}

/*
** Return the transpose of the given matrix.
*/
Matrix MatrixTranspose(M)
Matrix M;
{
	Matrix Mt;
	int i,j;
	
	Mt.rows = M.cols;
	Mt.cols = M.rows;

	for (i = 0; i < Mt.rows; i++) {
		for (j = 0; j < Mt.cols; j++) {
			Mt.x[i][j] = M.x[j][i];
		}
	}
	return Mt;
}


/*
** Compute the determinant of the given matrix.
**
*/
Scalar MatrixDet(M)
Matrix M;
{
	if (IsNullMatrix(M)) {
		(*MatrixError)("Attempt to compute determinant on Null matrix.");
		return 0.0;
	}
	if (M.rows  != M.cols) {
		(*MatrixError)("Attempt to compute determinant on non-square matrix.");
		return 0.0;
	}

	switch (M.rows) {
	case 1:
		return M.x[0][0];
		break;
	case 2:
		return (M.x[0][0] * M.x[1][1] - M.x[1][0] * M.x[0][1]);
		break;
	case 3:
		return det3x3( M.x[0][0], M.x[0][1], M.x[0][2],
		               M.x[1][0], M.x[1][1], M.x[1][2],
		               M.x[2][0], M.x[2][1], M.x[2][2]);
		break;
	case 4:
		/* Check for an affine matrix */
		if (IsAffineMatrix(M)) {
			return det3x3( M.x[0][0], M.x[0][1], M.x[0][2],
		               	       M.x[1][0], M.x[1][1], M.x[1][2],
		               	       M.x[2][0], M.x[2][1], M.x[2][2]);
		} else {
			/* Do full 4x4 calculation */
			Scalar d00, d01, d02, d03;

			d00 = det3x3( M.x[1][1], M.x[1][2], M.x[1][3],
				      M.x[2][1], M.x[2][2], M.x[2][3],
				      M.x[3][1], M.x[3][2], M.x[3][3]);
			d01 = det3x3( M.x[1][0], M.x[1][2], M.x[1][3],
				      M.x[2][0], M.x[2][2], M.x[2][3],
				      M.x[3][0], M.x[3][2], M.x[3][3]);
			d02 = det3x3( M.x[1][0], M.x[1][1], M.x[1][3],
				      M.x[2][0], M.x[2][1], M.x[2][3],
				      M.x[3][0], M.x[3][1], M.x[3][3]);
			d03 = det3x3( M.x[1][0], M.x[1][1], M.x[1][2],
				      M.x[2][0], M.x[2][1], M.x[2][2],
				      M.x[3][0], M.x[3][1], M.x[3][2]);
			return M.x[0][0]*d00 - M.x[0][1]*d01 + M.x[0][2]*d02 - M.x[0][3]*d03;
		}
		break;
	}
}



/*
** Invert a matrix using the adjoint method.
**
*/
Matrix MatrixInverse(M)
Matrix M;
{
  Matrix Minv;
  Scalar detM, detA;
  int i, j, index, size, sign;

#ifdef DEBUG
  fprintf(stderr,"Inverting...\n");
  MatrixPrint( M, stderr);
#endif
  
  if (M.rows != M.cols) {
	(*MatrixError)("Attempt to invert non-square matrix.");
	return MatrixNull();
  }
  if (IsNullMatrix(M)) {
	(*MatrixError)("Attempt to invert null matrix.");
	return MatrixNull();
  }

  if (M.rows > 4) {
  	(*MatrixError)("Can't invert matrices larger than 4x4.");
  	return MatrixNull();
  }

  size = M.rows;
  detM = MatrixDet(M);  
  if (fabs(detM) < EPSILON) {
	(*MatrixError)("Attempt to invert a singular matrix.");
	return MatrixNull();
  }

  Minv = MatrixZero(size);
  for(i = 0; i < size; i++) {			
	sign = (i&1) ? ~0 : 0;			
	for(j = 0; j < size; j++) {		
	    detA = MatrixDet( MatrixAdjoint( M, i, j));
	    Minv.x[j][i] = sign ? -detA : detA;
	    sign = ~sign;		
	}				
  }					

#ifdef DEBUG
	fprintf(stderr, "Result...\n");
	MatrixPrint( MatrixScale( 1.0/detM, Minv), stderr);
#endif
	
  return MatrixScale( 1.0/detM, Minv);
}


/*
** Return the Adjoint matrix A_{i,j}(M).  That is,
** the matrix obtained by eliminating the ith row and
** jth column of M.  
**
** NOTE: M is currently assumed to be square.
*/
Matrix MatrixAdjoint( M, i, j)
Matrix M;
int i,j;
{
    int i3, j3, row = 0, col, size = M.rows;
    Matrix Adjoint;
    
    Adjoint = MatrixZero(size-1);
    for(i3 = 0; i3 < size; i3++) {	
	if (i3 != i) {
            col = 0;
	    for(j3 = 0; j3 < size; j3++) {
		if(j3 != j) {
			Adjoint.x[row][col++] = M.x[i3][j3];
		}
	    }				
	    row++;
	}
    }					
    return Adjoint;
}


