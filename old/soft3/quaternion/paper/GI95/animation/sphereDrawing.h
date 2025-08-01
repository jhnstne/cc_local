/*
	File: sphereDrawing.h
	Author: J.K. Johnstone
	Last Modified: March 8, 1995
	Purpose:
	
*/

#define MAXFRAMES	20	/* maximum number of key frames (orientations) 
				   in animation */
#define PT_RADIUS	.03

extern void 	 viz_q(const int n, const Qion q[], const v4dh p[],
		       double display_pbspl[4][MAXDISPLAYPTS], 
		       const int pbspl_num, 
		       const bspl_4d *pCurveBspl, 
		       double display_pbez[4][MAXDISPLAYPTS], 
		       const int pbez_num, 
		       const bez_4d *pCurveBez,
		       double display_qratbez[4][MAXDISPLAYPTS],
		       const int qratbez_num, 
		       const ratbez_4d *qCurveRatBez);






