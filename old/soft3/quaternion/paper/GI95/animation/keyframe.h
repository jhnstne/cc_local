/*
        File: animation.h
        Author: J.K. Johnstone
        Last Modified: October 17, 1994
	Purpose:
*/

#define MAXFRAMES       20      /* maximum number of key frames (orientations) 
                                   in animation */
#define PT_RADIUS       .03
#ifndef PI
#define PI              3.141593
#endif

extern void 	animate      (const int          n,
	   		      const unsigned int m,
			      const Qion         q[],
		   	      const V3d	         pos[],
			      const ratbez_4d    *sphereratbez,
			      double display_sphereratbez[4][MAXDISPLAYPTS],
			      const int spherebez_num,
			      const bez_3d     *directrixbez,
			      double display_dirbez[3][MAXDISPLAYPTS],
			      const int dirbez_num);

