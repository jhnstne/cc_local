#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include "all.h"

#define SQ(A) ((A)*(A))

Space s;

void f1Header()
{
	printf("%% Franke function 1\n");
	printf("%%\n");
	printf("%% f(x,y) =\n");
	printf("%%	3./4.*exp( -(SQ(9*x-2) + SQ(9*y-2))/4. ) \n");
	printf("%%	  + 3./4.*exp( -SQ(9*x+1)/49. -SQ(9*y+1)/10.) \n");
	printf("%%	    + 1./2.*exp(-( SQ(9*x-7) + SQ(9*y-3) )/4. ) \n");
	printf("%%	      - 1./5.*exp(-SQ(9*x-4) - SQ(9*y-7));\n");
	printf("%% (Note that a sign error in the paper was corrected.)\n");
}


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

double f1dx(double x, double y)
{
	double dx;

	dx = 3./4. * -1./4. * 2 * (9*x-2) *9* exp( -(SQ(9*x-2) + SQ(9*y-2))/4. )
	  -3./4./49.*2*(9*x+1) *9* exp( -SQ(9*x+1)/49. -SQ(9*y+1)/10.) 
	    -1./2. /4.* 2.*(9*x-7)*9 *  exp(-( SQ(9*x-7) + SQ(9*y-3) )/4. ) 
	      +1./5.*2*(9*x-4)*9*exp(-SQ(9*x-4) - SQ(9*y-7));
	return dx;
}

double f1dy(double x, double y)
{
	double dy;

	dy = 3./4. * -1./4.*2*(9*y-2)*9 * exp( -(SQ(9*x-2) + SQ(9*y-2))/4. )
	  -3./4. /10.*2*(9*y+1)*9 * exp( -SQ(9*x+1)/49. -SQ(9*y+1)/10.) 
	    -1./2./4.*2.*(9*y-3)*9 *  exp(-( SQ(9*x-7) + SQ(9*y-3) )/4. ) 
	      +1./5.*2*(9*y-7)*9 *exp(-SQ(9*x-4) - SQ(9*y-7));
	return dy;
}

Normal f1n(double x, double y)
{
	Vector v1;
	Vector v2;

	v1 = VCreate(StdFrame(s), 1., 0., f1dx(x,y));
	v2 = VCreate(StdFrame(s), 0., 1., f1dy(x,y));
	return VDual(VNormalize(VVCross(v1, v2)));
}

/* From Maple */
double f1dxx(double x, double y)
{
	return -0.30375E2*exp(-0.25*pow(9.0*x-2.0,2.0)-0.25*pow(9.0*y-2.0,2.0))+
0.75*pow(-0.405E2*x+0.9E1,2.0)*exp(-0.25*pow(9.0*x-2.0,2.0)-0.25*pow(9.0*y-2.0,
2.0))-0.2479591838E1*exp(-0.2040816327E-1*pow(9.0*x+1.0,2.0)-0.1*pow(9.0*y+1.0,
2.0))+0.75*pow(-0.330612245E1*x-0.3673469389,2.0)*exp(-0.2040816327E-1*pow(9.0*
x+1.0,2.0)-0.1*pow(9.0*y+1.0,2.0))-0.2025E2*exp(-0.25*pow(9.0*x-7.0,2.0)-0.25*
pow(9.0*y-3.0,2.0))+0.5*pow(-0.405E2*x+0.315E2,2.0)*exp(-0.25*pow(9.0*x-7.0,2.0
)-0.25*pow(9.0*y-3.0,2.0))+0.324E2*exp(-pow(9.0*x-4.0,2.0)-pow(9.0*y-7.0,2.0))
-0.2*pow(-162.0*x+72.0,2.0)*exp(-pow(9.0*x-4.0,2.0)-pow(9.0*y-7.0,2.0));

}

double f1dyy(double x, double y)
{
	return -0.30375E2*exp(-0.25*pow(9.0*x-2.0,2.0)-0.25*pow(9.0*y-2.0,2.0))+
0.75*pow(-0.405E2*y+0.9E1,2.0)*exp(-0.25*pow(9.0*x-2.0,2.0)-0.25*pow(9.0*y-2.0,
2.0))-0.1215E2*exp(-0.2040816327E-1*pow(9.0*x+1.0,2.0)-0.1*pow(9.0*y+1.0,2.0))+
0.75*pow(-0.162E2*y-0.18E1,2.0)*exp(-0.2040816327E-1*pow(9.0*x+1.0,2.0)-0.1*
pow(9.0*y+1.0,2.0))-0.2025E2*exp(-0.25*pow(9.0*x-7.0,2.0)-0.25*pow(9.0*y-3.0,
2.0))+0.5*pow(-0.405E2*y+0.135E2,2.0)*exp(-0.25*pow(9.0*x-7.0,2.0)-0.25*pow(9.0
*y-3.0,2.0))+0.324E2*exp(-pow(9.0*x-4.0,2.0)-pow(9.0*y-7.0,2.0))-0.2*pow(-162.0
*y+126.0,2.0)*exp(-pow(9.0*x-4.0,2.0)-pow(9.0*y-7.0,2.0));

}

double f1dxy(double x, double y)
{
	return 0.75*(-0.405E2*x+0.9E1)*(-0.405E2*y+0.9E1)*exp(-0.25*pow(9.0*x-2.0,
2.0)-0.25*pow(9.0*y-2.0,2.0))+0.75*(-0.330612245E1*x-0.3673469389)*(-0.162E2*y
-0.18E1)*exp(-0.2040816327E-1*pow(9.0*x+1.0,2.0)-0.1*pow(9.0*y+1.0,2.0))+0.5*(
-0.405E2*x+0.315E2)*(-0.405E2*y+0.135E2)*exp(-0.25*pow(9.0*x-7.0,2.0)-0.25*pow(
9.0*y-3.0,2.0))-0.2*(-162.0*x+72.0)*(-162.0*y+126.0)*exp(-pow(9.0*x-4.0,2.0)-
pow(9.0*y-7.0,2.0));

}

SFF f1sff(double x, double y)
{
        SFF sff;
        Normal n;
	Frame wf;

	wf = StdFrame(s);

        n = f1n(x, y);

        sff.v0 = VCreate(wf, 1., 0., f1dx(x,y));
        sff.v1 = VCreate(wf, 0., 1., f1dy(x,y));
        sff.m[0][0] = NVApply(n, VCreate(wf, 0., 0., f1dxx(x,y)));
        sff.m[0][1] = NVApply(n, VCreate(wf, 0., 0., f1dxy(x,y)));
        sff.m[1][0] = sff.m[0][1];
        sff.m[1][1] = NVApply(n, VCreate(wf, 0., 0., f1dyy(x,y)));

	return sff;
}


void f2Header()
{
	printf("%% Franke function 2\n");
	printf("%%\n");
	printf("%% f(x,y) =\n");
	printf("%%	2*exp(18.*y-18.*x)/(9*exp(18.*y-18.*x)+9.);\n");
}

double f2(double x, double y)
{
	return 2*exp(18.*y-18.*x)/(9*exp(18.*y-18.*x)+9.);
}

/* + */
Normal f2n(double x, double y)
{
	Vector v1;
	Vector v2;
	double dx,dy;

	dx = -2.*18.*exp(18*y - 18*x)/(9*exp(18*y - 18*x)+9) +
	  2.*exp(18.*y-18.*x)* 9.*18.*exp(18*y - 18*x)/
	    SQ((9*exp(18*y - 18*x)+9));
	dy = -dx;
	v1 = VCreate(StdFrame(s), 1., 0., dx);
	v2 = VCreate(StdFrame(s), 0., 1., dy);
	return VDual(VNormalize(VVCross(v1, v2)));
}

void f3Header()
{
	printf("%% Franke function 3\n");
	printf("%%\n");
	printf("%% f(x,y) =\n");
	printf("%%	( 5./4. + cos(27./5. * y)) / ( 6. + 6*SQ(3*x-1));\n");
}

double f3(double x, double y)
{
	return ( 5./4. + cos(27./5. * y)) / ( 6. + 6*SQ(3*x-1));
}

/* + */
Normal f3n(double x, double y)
{
	Vector v1;
	Vector v2;
	double dx,dy;

	dx = -(5./4. + cos(27./5.*y))*6*2*(3*x-1)/SQ(6+6*SQ(3*x-1));
	dy = (-27./5.*sin(27./5.*y))/( 6. + 6*SQ(3*x-1));
	v1 = VCreate(StdFrame(s), 1., 0., dx);
	v2 = VCreate(StdFrame(s), 0., 1., dy);
	return VDual(VNormalize(VVCross(v1, v2)));
}

void f4Header()
{
	printf("%% Franke function 4\n");
	printf("%%\n");
	printf("%% f(x,y) =\n");
	printf("%%	1./3. * exp( -81./16.*(SQ(x-1./2.)+SQ(y-1./2.)));\n");
}

double f4(double x, double y)
{
	return 1./3. * exp( -81./16.*(SQ(x-1./2.)+SQ(y-1./2.)));
}

/* + */
Normal f4n(double x, double y)
{
	Vector v1;
	Vector v2;
	double dx,dy;;

	dx = -81./16.*2.*(x-1./2.)*f4(x,y);
	dy = -81./16.*2.*(y-1./2.)*f4(x,y);
	v1 = VCreate(StdFrame(s), 1., 0., dx);
	v2 = VCreate(StdFrame(s), 0., 1., dy);
	return VDual(VNormalize(VVCross(v1, v2)));
}

void f5Header()
{
	printf("%% Franke function 5\n");
	printf("%%\n");
	printf("%% f(x,y) =\n");
	printf("%%	1./3. * exp( -81./4.*(SQ(x-1./2.)+SQ(y-1./2.)));\n");
}

double f5(double x, double y)
{
	return 1./3. * exp( -81./4.*(SQ(x-1./2.)+SQ(y-1./2.)));
}

/* + */
Normal f5n(double x, double y)
{
	Vector v1;
	Vector v2;
	double dx,dy;

	dx = -81./4.*2.*(x-1./2.)*f5(x,y);
	dy = -81./4.*2.*(y-1./2.)*f5(x,y);
	v1 = VCreate(StdFrame(s), 1., 0., dx);
	v2 = VCreate(StdFrame(s), 0., 1., dy);
	return VDual(VNormalize(VVCross(v1, v2)));
}

void f6Header()
{
	printf("%% Franke function 6\n");
	printf("%%\n");
	printf("%% f(x,y) =\n");
	printf("%%	1./9.*sqrt(64.-81.*(SQ(x-1./2.)+SQ(y-1./2.)))-.5;\n");
}

double f6(double x, double y)
{
	return 1./9.*sqrt(64.-81.*(SQ(x-1./2.)+SQ(y-1./2.)))-.5;
}

/* + */
Normal f6n(double x, double y)
{
	Vector v1;
	Vector v2;
	double dx,dy;

	dx = -9.*(x-1./2.)/sqrt(64.-81.*(SQ(x-1./2.)+SQ(y-1./2.)));
	dy = -9.*(y-1./2.)/sqrt(64.-81.*(SQ(x-1./2.)+SQ(y-1./2.)));
	v1 = VCreate(StdFrame(s), 1., 0., dx);
	v2 = VCreate(StdFrame(s), 0., 1., dy);
	return VDual(VNormalize(VVCross(v1, v2)));
}

NewArgs(int argc, char* argv[], int* ac, char*** av)
{
	int i;
	char** nav;

	nav = (char**)malloc((argc+4)*sizeof(char*));
	nav[0] = argv[0];
	nav[1] = "-xy";
	nav[2] = "0";
	nav[3] = "0";
	nav[4] = "1";
	nav[5] = "1";
	for (i=2; i<argc; i++) {
		nav[i+4] = argv[i];
	}
	*av = nav;
	*ac = argc+4;
}

static header(int n)
{
	printf("%% Franke function, as presented in T. Grandine,\n");
	printf("%%  'An iterative method for computing multivariate C1\
piecewise polynomial \n");
	printf("%%  interpolants,' CAGD 1987, Vol 4, No 4, pg 307-319\n");
	printf("%%\n");
	switch(n) {
	      case 1:
		f1Header();
		break;
	      case 2:
		f2Header();
		break;
	      case 3:
		f3Header();
		break;
	      case 4:
		f4Header();
		break;
	      case 5:
		f5Header();
		break;
	      case 6:
		f6Header();
		break;
	}
	printf("%%\n");
}


main(int argc, char* argv[])
{
	int n;
	int ac;
	char** av;

	if ( argc < 2  ||  !isdigit(argv[1][0]) ) {
		fprintf(stderr, "Usage: %s n [args]\n",argv[0]);
		exit(1);
	}
	n = atoi(argv[1]);
	argv[1] = argv[0];

	s = SCreate("world",3);

	NewArgs(argc, argv, &ac, &av);

	header(n);

	switch (n) {
	      case 1:
		mfunction(ac, av, s, f1, f1n, f1sff);
		break;
	      case 2:
		mfunction(ac, av, s, f2, f2n, NULL);
		break;
	      case 3:
		mfunction(ac, av, s, f3, f3n, NULL);
		break;
	      case 4:
		mfunction(ac, av, s, f4, f4n, NULL);
		break;
	      case 5:
		mfunction(ac, av, s, f5, f5n, NULL);
		break;
	      case 6:
		mfunction(ac, av, s, f6, f6n, NULL);
		break;
	      default:
		fprintf(stderr, "First arg must be in {1,2,3,4,5,6}.\n");
		exit(1);
	}
}
