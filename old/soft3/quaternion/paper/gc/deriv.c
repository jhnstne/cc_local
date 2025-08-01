/********************************************************************
	Compute the derivative of a Bezier curve
	at the point with parameter value u=a.

	Method: construct the Bezier curve obtained by differencing
	the control points of the original Bezier curve.

	WRONG: NEED TO MULTIPLY BY FACTOR OF N (DEGREE) AND DIVIDE
	BY LENGTH OF PARAMETER INTERVAL (DELTA_I)

	WOULD BE BETTER TO USE INTERMEDIATE DE CASTELJAU POINTS.
********************************************************************/

void DerivBez (Bez *bez, REAL a, V3r deriv)
{
 Bez diffbez;
 int i,j;
	
 diffbez.d = bez->d - 1;
 diffbez.L = bez->L;
 diffbez.knots = (REAL *) malloc((diffbez.L + 1) * sizeof(REAL));
 diffbez.x1 = (REAL *) malloc((diffbez.d*diffbez.L + 1) * sizeof(REAL));
 diffbez.x2 = (REAL *) malloc((diffbez.d*diffbez.L + 1) * sizeof(REAL));
 diffbez.x3 = (REAL *) malloc((diffbez.d*diffbez.L + 1) * sizeof(REAL));

 for (i=0; i<=diffbez.L; i++)
   diffbez.knots[i] = bez->knots[i];
 for (i=0; i<=diffbez.L; i++)	/* each segment */
   for (j=0; j<=diffbez.d; j++)
    {
	diffbez.x1[i*diffbez.d + j] = bez->x1[i*bez->d + j+1]  /* diff */
				    - bez->x1[i*bez->d + j];
	diffbez.x2[i*diffbez.d + j] = bez->x2[i*bez->d + j+1]  
				    - bez->x2[i*bez->d + j];
	diffbez.x3[i*diffbez.d + j] = bez->x3[i*bez->d + j+1]  
				    - bez->x3[i*bez->d + j];
    }
 PointOnBez (&diffbez, a, deriv);
}
