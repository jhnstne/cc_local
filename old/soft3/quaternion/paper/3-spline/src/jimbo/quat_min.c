#include <stdio.h>
#include <math.h>
#include "deigens.h"

/* The idea: given a set of n quaternions, q1..qn,
   find the quaternion qd that minimizes the quantity
   <qd,q1>+...+<qd,qn> -- the sum of angular distance 
   (cos(theta)) between unit vectors 

   The unit vector corresponding to the minumum eigenvalue
   of the covariance matrix of the input quaternions Qmin will
   minimize this quantity.  Perturbing the input by Qmin will
   maximize average distance to the pole in the least-squares
   sense.

   JPW - Mar 31, 1997
   */


/* 
   How to test it: (theoretically it works)

   Generate n random quaternions.
   Construct cov. matrix
   Find Qmin

   Compute sum of squares distance for Qmin = sumQmin

   Generate a lot of random quaternions, and comute 
   sum of squares distance for these quaternions.  Verify
   that these sums are less that sumQmin
   */


/* number of quaternions in the set */

#define NUM_N	10	

double	**input_quaternions;
double  *min_quaternion;



double	dot_product(double *v1, double *v2, int dim)
{
  int i;
  double result = 0.0;

  for(i=0;i<dim;i++)
    result += v1[i]*v2[i];
  return result;
}


double	sum_of_squares_dist(double **vects, double *ref_vect, int nvect)
{
  int i,j;
  double dp;
  double result =0;

  for(i=0;i<nvect;i++)
    {
      dp = dot_product(vects[i],ref_vect,4);
      result += dp*dp;
    }
  return result;
}

void	normalize_vector(double *v,int dim)
{
  int i;
  double len;

  len = 0;
  for(i=0;i<dim;i++)
    {
      len += v[i]*v[i];
    }
  len = sqrt(len);
  for(i=0;i<dim;i++)
    {
      v[i] = v[i]/len;
    }
}

double	*random_quaternion()
{
  static double result[4];
  int i;

  for(i=0;i<4;i++) result[i] = (double) ((random() % 10000) - 5000);
  normalize_vector(result,4);
  return result;
}


void	init_quaternions()
{
  int i,j;
  


  input_quaternions = (double **) malloc(sizeof(double *) * NUM_N);

  for(i=0;i<NUM_N;i++)
    {
      input_quaternions[i] = (double *) malloc(sizeof(double) * 4);
      for(j=0;j<4;j++)
	{
	  input_quaternions[i][j] = (double) (random()%10000 - 5000);
	}
      normalize_vector(input_quaternions[i],4);
    }
}


double	*min_eigenvector4X4(double **vectors, int nvect)  
{
  double	lower_tri_matrix[10];
  int		i,j,k;
  int		dex,mindex;
  double	res_evects[16];
  double	res_evals[4],min_eval;
  double	*result;

  result = (double *) malloc(sizeof(double)*4);
  
  for(i=0;i<10;i++)
    lower_tri_matrix[i] = 0.0;

  for(i=0;i<nvect;i++)
    {
      dex = 0;
      for(j=0;j<4;j++)
	{
	  for(k=0;k<=j;k++)
	    {
	      lower_tri_matrix[dex] += vectors[i][j]*vectors[i][k];
	      dex++;
	    }
	}
    }

  d_real_symmetric_eigens(lower_tri_matrix,res_evects,res_evals,4);
  

  /* output eigen vectors and values */
  for(i=0;i<4;i++)
    {
      printf("%lf \t:",res_evals[i]);
      for(j=0;j<4;j++)
	printf("%lf \t",res_evects[i*4+j]);
      printf("\n");
    }

  min_eval = 99999.9;
  for(i=0;i<4;i++)
    {
      if(res_evals[i] < min_eval)
	{
	  mindex = i;
	  min_eval = res_evals[i];
	}
    }
  
  for(i=0;i<4;i++) result[i] = res_evects[mindex*4 + i];
  printf("min vector is #%d\n",mindex);
  return result;
}


main(int argc, char **argv)
{
int i,j;
int seed;
double minsos,best_sos,cur_sos;
double best_guess[4],*guess;


if(argc!=2)
  {
  printf("usage:\t%s <integer seed>\n",argv[0]);
  exit(1);
  }

sscanf(argv[1],"%d",&seed);
srandom(seed);
init_quaternions();
min_quaternion = min_eigenvector4X4(input_quaternions,NUM_N);
minsos = sum_of_squares_dist(input_quaternions,min_quaternion,NUM_N);
printf("min S of S = %lf\n",minsos);
best_sos = 9999999.9;
for(i=0;i<100000;i++)
  {
    guess = random_quaternion();
    cur_sos = sum_of_squares_dist(input_quaternions,guess,NUM_N);
    if(cur_sos<best_sos)
      {
	best_sos = cur_sos;
	for(j=0;j<4;j++) best_guess[j] = guess[j];
      }
  }
   
printf("best guess SOS = %lf\n",best_sos);
for(i=0;i<4;i++)
  printf("\t%lf",best_guess[i]);
printf("\n");
}
