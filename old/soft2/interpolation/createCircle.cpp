/*
  File:          createCircle.cpp
  Author:        J.K. Johnstone 
  Created:	 8 January 2004
  Last Modified: 8 January 2004
  Purpose:       Compute the data points of an arbitrary circle.
  Input: 	 Center and radius of circle, angle between samples.
  History: 	 
*/

#include <GL/glut.h>
#include <GL/glu.h>
#include <fstream.h>
#include <iostream.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>

#include "Miscellany.h"

static char *RoutineName;
static void usage()
 {
   cout << "Compute samples on a circle" << endl;
   cout << "Usage is " << RoutineName << endl;
   cout << "\t[-r radius] (default: 1)" << endl;
   cout << "\t[-a angle in degrees between samples] (default: 20)" << endl;
   cout << "\t[-c (x,y)] coordinates of center; default (0,0)" << endl;
   cout << "\t[-h]   (this help message)" << endl;
 }

int main (int argc, char **argv)
{
  int       i;
  int       ArgsParsed=0;
  float     radius=1,           // circle radius
            xcen=0, ycen=0,     // circle center
            angle=20;           // angle between samples

  RoutineName = argv[ArgsParsed++];
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'r': radius = atof(argv[ArgsParsed++]);	        break;
      case 'c': xcen   = atof(argv[ArgsParsed++]);
	        ycen   = atof(argv[ArgsParsed++]);              break;
      case 'a': angle  = atof(argv[ArgsParsed++]);	        break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
    else ArgsParsed++;
   }

  float theta=0;
  while (theta < 360)
    {
      cout << radius * cos(deg2rad(theta)) + xcen << " "
	   << radius * sin(deg2rad(theta)) + ycen << endl;
      theta += angle;
    }
  return 0;
}
