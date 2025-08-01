/*
  File: 	 qsplsupport.h
  Author:	 J.K. Johnstone
  Created:	 20 March 1998
  Last Modified: 26 March 1998
  Purpose:	 Support routines for quaternion splines.
  History:
*/

#ifndef _QSPLSUPPORT_
#define _QSPLSUPPORT_

#include "Vector.h"

extern void Input (int &n, V4f *&pt, GLboolean RANDOMINPUT, GLboolean NPTKNOWN, 
	   	   GLboolean INPUTANGLEAXIS, GLboolean VISUALIZE, 
		   float samplingAngle);
extern void MapInput (const int n, V4f *pt, V4f *&imagept, 
		      GLboolean IMAGEONS3);

#endif
