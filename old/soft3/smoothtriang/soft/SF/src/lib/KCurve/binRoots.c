/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#include <stdio.h>
#include <math.h>

double eval(c,t)
double c[];
double t;
{
  int i; double sum;

  sum = 0.0;
  for(i=0;i<=4;i++){
    sum = sum*t + c[i];
  }
  return sum;
}


main(argc,argv)
int argc;
char* argv[];
{
  double c[5];
  double t,tmax,tmin;
  double ft,ftmax,ftmin;

  scanf(" %g %g %g %g %g",&c[0],&c[1],&c[2],&c[3],&c[4]);
  
  tmax = 2.0; tmin = 0.0; 
  ftmax = eval(c,tmax);
  ftmin = eval(c,tmin);
  if ( ftmin > ftmax ) {
    ft = ftmin;
    ftmin = ftmax;
    ftmax = ft;
    t = tmin;
    tmin = tmax;
    tmax = t;
  }
  if ( ftmin*ftmax > 0 ){
    printf("no root in range.  ftmin = %g, ftmax = %g\n",ftmin,ftmax);
    exit(0);
  }

  while(fabs(tmax-tmin)>.0000001){
    t = (tmax+tmin)/2.0;
    ft = eval(c,t);
    if ( ft == 0 ){
      printf("root is %g\n",t);
      exit(0);
    } else if ( ft > 0 ){
      ftmax = ft;
      tmax = t;
    } else {
      ftmin = ft;
      tmin = t;
    }
  }
  printf("root is between %g and %g\n",tmin,tmax);
}

