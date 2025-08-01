/*
** Matrix.h: Header file for users of the matrix package.
**
** Copyright (c) 1989, Graphics and AI Laboratory, University of Washington
** Copying, use and development for non-commercial purposes permitted.
**                  All rights for commercial use reserved.
**		    This software is unsupported.
**
*/

#define EPSILON		(1e-15)   /* Essentially zero */
#define MAX_MAT_SIZE	4	  /* Max # of rows or columns         */
#define Scalar		double	  /* Elements of vectors and matrices */

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif


/* Matrix access macros */
#define MatrixRows(M)		(M.rows)
#define MatrixColumns(M) 	(M.cols)
#define MatrixEntry(M,i,j) 	(ME(M,i,j))
#define ME(M,i,j)		(M.x[i][j])

/* Cover macros */
#define MatrixZero(size)	MatrixCreate(size,size)

/* Predicates */
#define IsNullMatrix(M)		(M.rows==0 && M.cols==0)

typedef struct {
	int rows, cols;		  		/* Dimensions */
	Scalar x[MAX_MAT_SIZE][MAX_MAT_SIZE];	/* The components */
} Matrix;

extern void MatrixPrint();
extern void SetMatrixError();
extern Scalar MatrixDet();
extern int IsAffineMatrix();

extern Matrix
	MatrixAdjoint(),
	MatrixCreate(),
	MatrixIdentity(),
	MatrixInverse(),
	MatrixMult(),
	MatrixNull(),
	MatrixScale(),
	MatrixTranspose(),
	MatrixZeroRC();
	
