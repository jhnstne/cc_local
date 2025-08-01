/*
	File: sphereDrawing.c
	Author: J.K. Johnstone
	Last Modified: Dec. 5, 1994
	Purpose: Compute a Bezier curve on the unit 4-sphere
		 interpolating n quaternions.
		 The spherical Bezier curve defines the orientation
		 of an object in-between the n given quaternion
		 orientations.
		 The quaternions are input intuitively
		 as (\theta, axis) rather than 
		 (cos \theta/2, sin \theta/2 axis).
	
*/

#include <device.h>
#include <gl/gl.h>
#include <gl/sphere.h>
#include <math.h>
#include <stdio.h>

#include "/usr/people/jj/cbin/vec.h"
#include "bez.h"
#include "/usr/people/jj/cbin/fit.h"
#include "display.h"
#include "/usr/people/jj/cbin/lights.h"
#include "/usr/people/jj/cbin/materials.h"
#include "/usr/people/jj/cbin/misc.h"

#include "quaternion.h"
#include "drw.h"
#include "/usr/people/jj/cbin/M.h"
#include "sphereDrawing.h"

#include "bez.c"
#include "/usr/people/jj/cbin/fit.c"
#include "display.c"
#include "quaternion.c"	/* cbin's quaternion changed to v4d as of Dec. 6 */
#include "drw.c"
#include "/usr/people/jj/cbin/vec.c"
#include "/usr/people/jj/cbin/misc.c"
#include "/usr/people/jj/cbin/M.c"

int 	xmax,ymax;	/* screen dimensions */
int	zmax;		/* z-buffer size */
float	rotx,roty,rotz;
int	rot_bool = 0;	/* start rotating? */

main()
{

	unsigned int   	n;		/* # of quaternions (orientations) */
	Qion	       	q[MAXFRAMES];	/* quaternions */
	v4dh	       	p[MAXFRAMES];	/* 4-space images of quaternions
					   (under M^{-1}) */

	bspl_4d		pCurveBspl;	/* cubic B-spline interpolating
					   images of quaternions in 4-space */
	double 		display_pbspl[4][MAXDISPLAYPTS]; /* points for display */
	int 		pbspl_num;	/* # of display points */

	bez_4d 		pCurveBez;	/* cubic Bezier spline interpolating
		  			   images of quaternions in 4-space */
	double 		display_pbez[4][MAXDISPLAYPTS];
	int		pbez_num;
	ratbez_4d	qCurveRatBez;	/* rational sextic Bezier spline
				interpolating quaternions on 4-sphere */
	double 		display_qratbez[4][MAXDISPLAYPTS];
	int		qratbez_num;
	FILE		*fp;

	long		sphere_wid;
        Boolean 	exitflag;	/* window */
        short   	attached = 0;
        short   	value;
        int     	dev;
	int 	       	i;

	fp = fopen("quaternion.input","r");
	inputQion(&n,q,fp); /* n keyframe orientations */
	fclose(fp);
	
	/* long term: orientations could be input interactively
	* and intuitively by using a Motif scrollbar (also with
	* angle (s), giving immediate feedback by rotating
	* the object to the associated orientation, 
	* so that the user can alter/correct input immediately.
	* For context, the previous orientation(s) should
	* be displayed as ghosts or in a nearby window. 
	*/

	for (i=0;i<n;i++) {	/* quaternions --> 4-space */
		invM(q[i],p[i]);
	}
	/* interpolating cubic B-spline in 4-space */
	fitCubicBspl_4dh(n,p,&pCurveBspl);
	print_bspl(&pCurveBspl);
	prepare_draw_bspl_in_4d(&pCurveBspl,display_pbspl,&pbspl_num);

	bspl_to_bezier_4d(&pCurveBspl,&pCurveBez);
	print_bez(&pCurveBez);
	prepare_draw_bez_in_4d(&pCurveBez,display_pbez,&pbez_num);

	/* cubic Bezier 4-space --> rational sextic spherical Bezier */
	M_rat(&pCurveBez,&qCurveRatBez);
	print_ratbez(&qCurveRatBez);
	prepare_draw_ratbez_in_4d(&qCurveRatBez,display_qratbez,&qratbez_num);
	fp = fopen("sphereratbez.input","w");
	output_ratbez4d(&qCurveRatBez,fp);
	fclose(fp);
	
	sphere_wid = initialize_3d_window("Quaternion sphere",25,625,200);
	winset(sphere_wid);
	rotx = 900; roty = 0; rotz = 450;
	exitflag=FALSE;
	while (exitflag == FALSE) {
                viz_q(n,q,p,display_pbspl,pbspl_num,&pCurveBspl,
			    display_pbez, pbez_num, &pCurveBez,
			    display_qratbez, qratbez_num, &qCurveRatBez);
				/* show sphere, quaternions and images in 3-space,
				   by zeroing the x_3 component of quaternion,
				   and the x_2 component in 4-space:
		   		   M^{-1}(1,x_1,x_2,0,x_4) = (\pm 2 \sqrt{(x_0-x_1)/2},
						x_2,0,x_4,x_0 - x_1)
		   		   M((\pm 2 \sqrt{(x_0-x_1)/2},x_2,0,x_4,x_0 - x_1) = 
				  	     (y_0,y_1,y_2,0,y_4)
				   that is, can project to 3dspace with x_3=0 for viz of sphere
				   and x_2=0 for viz of 4-space */

                while ((exitflag == FALSE) && (qtest() || !attached)) {
                        dev = qread(&value);
                        if (((dev == ESCKEY) && (value == 0)))
                                exitflag = TRUE;
			else if (dev == LEFTMOUSE)
				rot_bool = 1;
                        else if (dev == WINFREEZE || dev == WINTHAW || dev == REDRAWICONIC) {
                                frontbuffer(TRUE);
                                pushmatrix();
                                reshapeviewport();
                                viz_q(n,q,p,display_pbspl,pbspl_num,&pCurveBspl,
					    display_pbez,pbez_num, &pCurveBez,
				    display_qratbez,qratbez_num,&qCurveRatBez);
                                popmatrix();
                                frontbuffer(FALSE);
                        }
                        else if (dev == REDRAW)
                                reshapeviewport();
                        else if (dev == INPUTCHANGE)
                                attached = value;
                }
        }
}

void viz_q(const int n, const Qion q[], const v4dh p[], 
	   double display_pbspl[4][MAXDISPLAYPTS], 
	   const int pbspl_point_num,
	   const bspl_4d *pCurveBspl,
	   double display_pbez[4][MAXDISPLAYPTS],
	   const int pbez_point_num,
	   const bez_4d *pCurveBez,
	   double display_qratbez[4][MAXDISPLAYPTS],  
	   const int qratbez_point_num, 
	   const ratbez_4d *qCurveRatBez)
{
	/* visualize quaternion 4d-sphere, unit quaternions Q and */
	/* their images P under M^{-1}, in 3d through projection */
	/* sphere as several lines of longitude (circles) */

	extern int xmax,ymax,zmax;
	extern float rotx,roty,rotz;
	int   i;

	/* czclear(0xFFFF55,zmax); */
	czclear(0xFFFFFF,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 10.0);
        pushmatrix();
	ortho(-2*xmax/ymax,2*xmax/ymax,-2,2,-2,2);
	translate(0.,0.,0.);
	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');
/*
*	define a function `orientation(x,y,z,rx,ry,rz)'
*	to accompany ortho command;
*
* 	polarview(dist, azimuth, incidence, twist); 
*/

	RGBcolor(255,255,255);
	draw_sphere(0,0,0,1); 
	for (i=0;i<n;i++) {
		RGBcolor(0,255,0);
		draw_quaternion(q[i]);
/*		draw_4dhpoint(p[i]); */
	}
	RGBcolor(255,0,0);
/*	draw_curve_in_4d (display_pbspl,pbspl_point_num); */
/*	draw_bspl_control_in_4d (pCurveBspl);   */

/*	test_M(display_pbspl,pbspl_point_num); */ /* test if M maps correctly
		by applying directly to each point, and drawing
		to compare with output of later Bezier curve on sphere */

	/* to compare output for error check */
/*	draw_bez_control_in_4d (pCurveBez); */
/*	draw_curve_in_4d (display_pbez,pbez_point_num); */

	RGBcolor(0,0,0);
	linewidth(3);
	draw_curve_on_4d_sphere (display_qratbez,qratbez_point_num);
	draw_ratbez_control_on_4d_sphere (qCurveRatBez);

	if (rot_bool == 1) {
		rotx += 20; 
		roty += 20;
		rotz += 20; 
	}
	linewidth(1);

	popmatrix();
	swapbuffers();
}
