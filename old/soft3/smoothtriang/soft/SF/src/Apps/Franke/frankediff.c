#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include "all.h"

#define SQ(A) ((A)*(A))


double f1(double x, double y)
{
	double z;

	z = 
	  3./4.*exp( -(SQ(9*x-2) + SQ(9*y-2))/4. ) 
	    + 3./4.*exp( -SQ(9*x+1)/49. -SQ(9*y+1)/10.) 
	      + 1./2.*exp(-( SQ(9*x-7) + SQ(9*y-3) )/4. ) 
		- 1./5.*exp(-SQ(9*x-4) - SQ(9*y-7));
	
	return z;
}


double f2(double x, double y)
{
	return 2*exp(18.*y-18.*x)/(9*exp(18.*y-18.*x)+9.);
}

double f3(double x, double y)
{
	return ( 5./4. + cos(27./5. * y)) / ( 6. + 6*SQ(3*x-1));
}


double f4(double x, double y)
{
	return 1./3. * exp( -81./16.*(SQ(x-1./2.)+SQ(y-1./2.)));
}


double f5(double x, double y)
{
	return 1./3. * exp( -81./4.*(SQ(x-1./2.)+SQ(y-1./2.)));
}


double f6(double x, double y)
{
	return 1./9.*sqrt(64.-81.*(SQ(x-1./2.)+SQ(y-1./2.)))-.5;
}


#define SWAP(A,B) {double t; t = (A); (A) = (B); (B) = t;}

double maxf=0;
double maxz= -1e20;
double minz= 1e20;

void CheckIt(double (*f)(double,double),double x, double y, double z)
{
	double fv, fz;
	fz = f(x,y);
	fv = fabs(fz-z);
	if ( fv > maxf ) {
		maxf = fv;
	}
	if ( fz > maxz ) {
		maxz = fz;
	}
	if ( fz <minz ) {
		minz = fz;
	}
}

void	s3dFormat(double (*f)(double, double), Lnode *l)
{
	static	Lnode		*Cur, *ThisVertex;
	static	int		i;
	static	Scalar		x[3], y[3], z[3], nx[3], ny[3], nz[3];
	static	Material	m[3];
	static	Material	tm;
	static	char		*Field, FieldMem[81];

	Field = FieldMem;

	if ((l == NULL) || 
	    (l->car == NULL) || 
	    (strcmp(l->car->name, "Triangle"))) {
		fprintf(stderr, "ERROR in s3dFormat: Lnode isn't a valid triangle\n");
		exit(1);
	}

	dGetRGBMaterial(l,"Triangle",&tm);

	for (i = 0; i <= 2; i++) {
		sprintf(Field, "Triangle.vertex%d", i + 1);

		if (!dLookUpLnode(l, Field, Cur)) {
			fprintf(stderr, "ERROR in s3dFormat: vertex not found\n");
			exit(1);
		}

		ThisVertex = Cur->cdr;

		if ( (!dGetScalar(ThisVertex, "pos[0]", &(x[i])))
		    ||
		    (!dGetScalar(ThisVertex, "pos[1]", &(y[i])))
		    ||
		    (!dGetScalar(ThisVertex, "pos[2]", &(z[i]))) ) {
			fprintf(stderr, "ERROR in s3dFormat: bad value in position vertex\n");
			exit(1);
		}

	}

	if (i <= 2) {
		fprintf(stderr, "ERROR in s3dFormat: bad vertex count\n");
		exit(1);
	}

	CheckIt(f,x[0],y[0],z[0]);
	CheckIt(f,x[1],y[1],z[1]);
	CheckIt(f,x[2],y[2],z[2]);
}


main(int argc, char* argv[])
{
	int n;
	int ac;
	char** av;
	double (*f)(double,double);

	if ( argc < 2  ||  !isdigit(argv[1][0]) ) {
		fprintf(stderr, "Usage: %s n [args]\n",argv[0]);
		exit(1);
	}
	n = atoi(argv[1]);
	argv[1] = argv[0];

	switch (n) {
	      case 1:
		f = f1;
		break;
	      case 2:
		f = f2;
		break;
	      case 3:
		f = f3;
		break;
	      case 4:
		f = f4;
		break;
	      case 5:
		f = f5;
		break;
	      case 6:
		f = f6;
		break;
	      default:
		fprintf(stderr, "First arg must be in {1,2,3,4,5,6}.\n");
		exit(1);
	}
	while( ReadDstruct() != EOF ){
		s3dFormat(f, din);
		dDeleteDstruct(din);
	}
	fprintf(stderr,"Max distance is %f.  Z range [%g, %g]\n",
		maxf,minz,maxz);
	exit(0);
}
