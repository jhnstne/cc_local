void d_real_symmetric_eigens(/* A, RR, E, N */ );
/* float A[], RR[], E[];
   int N; 
   */

/*							eigens.c
 *
 *	Eigenvalues and eigenvectors of a real symmetric matrix
 *
 *
 *
 * SYNOPSIS:
 *
 * int n;
 * float A[n*(n+1)/2], EV[n*n], E[n];
 * void eigens( A, EV, E, n );
 *
 *
 *
 * DESCRIPTION:
 *
 * The algorithm is due to J. vonNeumann.
 *                   -     -
 * A[] is a symmetric matrix stored in lower triangular form.
 * That is, A[ row, column ] = A[ (row*row+row)/2 + column ]

BEGIN JPW COMMENTARY

^^^ I think he mixed up row and column here if so:
JPW : for 3x3 this means : [xx,yx,yy,zx,zy,zz]
JPW : for 4x4 this means : [ww,xw,xx,yw,yx,yy,zw,zx,zy,zz]
^^^ If he has this one correct then the orders will be
JPW : for 3x3 this means : [xx,xy,xz,yy,yz,zz] etc


The way he calls it is the first way ([xx,yx,yy,zx,zy,zz])
END COMMENTARY

 * or equivalently with row and column interchanged.  The
 * indices row and column run from 0 through n-1.
 *
 * EV[] is the output matrix of eigenvectors stored columnwise.
 * That is, the elements of each eigenvector appear in sequential
 * memory order.  The jth element of the ith eigenvector is
 * EV[ n*i+j ] = EV[i][j].
 *
 * E[] is the output matrix of eigenvalues.  The ith element
 * of E corresponds to the ith eigenvector (the ith row of EV).
 *
 * On output, the matrix A will have been diagonalized and its
 * orginal contents are destroyed.
 *
 * ACCURACY:
 *
 * The error is controlled by an internal parameter called RANGE
 * which is set to 1e-10.  After diagonalization, the
 * off-diagonal elements of A will have been reduced by
 * this factor.
 *
 * ERROR MESSAGES:
 *
 * None.
 *
 */
/*
Copyright 1973, 1991 by Stephen L. Moshier
Copyleft version.
*/
