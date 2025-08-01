/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include <stdio.h>
#include  <math.h>


double PolishRoot();


void CheckRoots(p,r,nr)
double p[];
double r[];
int nr;
{
  int i,j;
  double sum;

  for(i=0;i<nr;i++){
    sum = 0.0;
    for(j=0;j<=4;j++){
      sum = r[i]*sum + p[j];
    }
/*    if ( fabs(sum) > 0.00001 ) {*/
      printf("root %d = %+12.12e yields %+12.12e\n",i,r[i],sum);
/*    }*/
  }
}


main(argc,argv)
int argc;
char* argv[];
{
  double c[5];
  double C;
  double r[5];
  int i;
  int n;

  if ( argc != 6 ) {
    fprintf(stderr,"format: blah a4 a3 a2 a1 a0.  Exiting.\n");
    exit(1);
  }

  for(i=0;i<=4;i++){
    c[i] = atof(argv[i+1]);
    if ( i==0 ) C = c[0];
    c[i] /= C;
  }
  printf("poly = 1 ");
  for(i=1;i<=4;i++){
    printf("%g ",c[i]);
  }
  printf("\n");
  QuarticRoots(c+1, &n, r);
  CheckRoots(c,r,n);
}
