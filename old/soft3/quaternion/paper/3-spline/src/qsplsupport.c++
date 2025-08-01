/*
  File: 	 qsplsupport.c++
  Author:	 J.K. Johnstone
  Created:	 circa 23 August 1996
  Last Modified: 15 March 1998
  Purpose:	 Support routines for quaternion splines.
  History:
*/

#include <GL/glut.h>
#include <math.h>
#include <stdlib.h>
#include <iostream.h>
#include <time.h>
#include "Miscellany.h"
#include "Vector.h"
#include "BsplineCurve.h"

// Paper discussion. Input: sampling criterion, visualization mode x3=0 quaternions

/******************************************************************************
	Read in the unit keyframe quaternions, or randomly generate them 
	(random #, random unit vector in R^4) and record in `random.dat'.
	Maintain the sampling criterion (consecutive quaternions 
	at angular distance < samplingAngle).
	If in visualization mode, transform quaternions to x3=0 hyperplane.
******************************************************************************/

void Input(int &n, V4f *&pt, GLboolean RANDOMINPUT, GLboolean NPTKNOWN, 
	   GLboolean INPUTANGLEAXIS, GLboolean VISUALIZE, float samplingAngle)
{
  FILE *fp;
  int   i,j;
  float angle;
  V3f   axis;

  if (RANDOMINPUT)
   {
    srand(time(0));
    if (NPTKNOWN) 
     { if (n<2) fatalError("Error: need at least two input points."); }
    else 
      n = rand() % 98 + 3; 	// generate 3-100 points
    pt = new V4f[n];
    for (i=0; i<n; i++)	   // generate random pts, within the sampling criterion
      do
       {
        for (j=0; j<4; j++)
          pt[i][j] = myRand();		// random number in [-1,1]
	if (VISUALIZE) pt[i][2] = 0.;	// (--,--,0,--) quaternions for display
	pt[i].normalize();
       }
      while (i!=0 && pt[i].angle(pt[i-1]) > samplingAngle);

    fp = fopen ("random.dat", "w");
    fprintf (fp,"%i\n", n);
    for (i=0; i<n; i++)
      fprintf(fp,"%f %f %f %f\n", pt[i][0], pt[i][1], pt[i][2], pt[i][3]);
    fclose(fp);
    cout << "Random input of " << n << " points, saved in `random.dat'." << endl;
    cout << "Sampling criterion: angular difference between consecutive \
    	     points is bounded at " << samplingAngle << " radians." << endl;
    if (VISUALIZE)
      cout << "Visualization mode: Input generated in x3=0 hyperplane." << endl;
   }
  else
   {
    cin >> n;
    if (n < 2) fatalError ("Error: need at least two input points.");
    pt = new V4f[n];
    if (INPUTANGLEAXIS)
      for (i=0; i<n; i++)
       {
        cin >> angle >> axis[0] >> axis[1] >> axis[2];
        angle = deg2rad(angle);
        axis.normalize();
	axis.scalarmult(sin(angle/2));
        pt[i][0] = cos(angle/2);
        for (j=0; j<3; j++)	pt[i][j+1] = axis[j];
       }
    else
      for (i=0; i<n; i++)
        cin >> pt[i][0] >> pt[i][1] >> pt[i][2] >> pt[i][3]; 

    if (VISUALIZE)
     {
      for (i=0; i<n; i++)  { pt[i][2] = 0; pt[i].normalize(); }
      cout << "Visualization mode: input projected to x3=0 hyperplane." << endl;
     }
    // test that input meets sampling criterion: no large gaps
    for (i=0; i<n-1; i++)
      if (pt[i].angle(pt[i+1]) > samplingAngle)
       {
       	cout << endl << "Angle between (" 
		<< pt[i][0] << "," << pt[i][1] << "," << pt[i][2]
		<< "," << pt[i][3] << ") and ("
		<< pt[i+1][0] << "," << pt[i+1][1] << "," << pt[i+1][2]
		<< "," << pt[i+1][3] << ") is " << 
		rad2deg (pt[i].angle(pt[i+1])) 
		<< " degrees, which exceeds the sampling criterion." << endl;
        fatalError ("Error: Sampling criterion violated.\n");
       }
   }
}			  

/******************************************************************************
	Map input data to inverse space.
******************************************************************************/

void MapInput (const int n, V4f *pt, V4f *&imagept, GLboolean IMAGEONS3)
{
  int i,j;
  
  imagept  = new V4f[n];  assert(imagept!=0);
  for (i=0; i<n; i++)
   {				// (p2,p3,p4,1-p1)
    for (j=0; j<3; j++)  imagept[i][j] = pt[i][j+1];
    imagept[i][3] = 1 - pt[i][0];
    if (IMAGEONS3) imagept[i].normalize();
   }
}



