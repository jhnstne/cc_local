/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include <stdio.h>
#include  <math.h>


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
