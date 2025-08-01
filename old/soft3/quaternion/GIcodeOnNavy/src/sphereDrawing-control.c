/*
	File: sphereDrawing.c
	Author: J.K. Johnstone
	Last Modified: Oct. 16, 1994
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

#include "vec.h"
#include "bez.h"
#include "fit.h"
#include "display.h"
#include "lights.h"
#include "materials.h"
#include "misc.h"

#include "quaternion.h"
#include "drw.h"
#include "sphereDrawing.h"

#include "bez.c"
#include "fit.c"
#include "display.c"
#include "quaternion.c"
#include "drw.c"
#include "vec.c"
#include "misc.c"

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
	bez_4d 		qCurveBez; 	/* sextic Bezier spline interpolating
					   quaternions on 4-sphere */
	double 		display_qbez[4][MAXDISPLAYPTS];
	int 		qbez_num;
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
	
	sphere_wid = initialize_3d_window("Quaternion sphere",950,1250,600);
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
	ortho(-3*xmax/ymax,3*xmax/ymax,-3,3,-3,3);
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

	RGBcolor(0,255,255);
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
	draw_curve_on_4d_sphere (display_qratbez,qratbez_point_num);
	draw_ratbez_control_on_4d_sphere (qCurveRatBez);   

	if (rot_bool == 1) {
		rotx += 20; 
		roty += 20;
		rotz += 20; 
	}

	popmatrix();
	swapbuffers();
}

void invM(const Qion q, v4dh p)
{
	/* Apply the map M^{-1} to the unit quaternion q,
		placing result in p */
	
	if (q[0] != q[1]) {
		p[0] = 2*sqrt((q[0]-q[1])/2);
		p[1] = q[2];
		p[2] = q[3];
		p[3] = q[4];
		p[4] = q[0]-q[1];
	}
	else {
		/* tentative choice: (1,1,1,1,0) */
		/* any point on the hyperplane x4=0 will work */
		/* but don't choose origin since M(1,0,0,0,0)=infinity */
		p[0] = p[1] = p[2] = p[3] = 1;
		p[4] = 0;
	
	}
	printf("M^{-1}(%.3f,%.3f,%.3f,%.3f)=(%.3f,%.3f,%.3f,%.3f)\n",
		q[1]/q[0],q[2]/q[0],q[3]/q[0],q[4]/q[0],
		p[1]/p[0],p[2]/p[0],p[3]/p[0],p[4]/p[0]);
}

void M_rat(const bez_4d *pbez,
	   ratbez_4d    *qratbez)
{
	/* Map the cubic Bezier spline PBEZ back to the unit 4-sphere,
	   by mapping each cubic Bezier segment to a rational sextic 
	   Bezier segment */

	int i,j,k,s;
	double foo1,foo2,foo3;
	double sum;

	qratbez->d = 6;
	qratbez->L = pbez->L;
	for (i=0;i<=qratbez->L;i++) {
		qratbez->knots[i] = pbez->knots[i];
	}
	for (s=0;s<pbez->L;s++) {	/* sth cubic Bezier segment */
	   /* weights */
	   for (k=0;k<=6;k++) { 
		sum=0.;
		for (i=0;i<=3;i++) {	
		   j=k-i;
	  	   if (j<=3 && j>=0) {
			foo1 = choose(3,i);
			foo2 = choose(3,j);
			foo3 = choose(6,k);
			sum += ((foo1*foo2)/foo3)
			      * (pbez->x1[3*s+i] * pbez->x1[3*s+j] +
				 pbez->x2[3*s+i] * pbez->x2[3*s+j] +
				 pbez->x3[3*s+i] * pbez->x3[3*s+j] +
				 pbez->x4[3*s+i] * pbez->x4[3*s+j]);
/*			printf("weight: (%i,%i,%i,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)\n",
*				k,i,j,
*				pbez->x1[3*s+i],pbez->x1[3*s+j],
*			 	pbez->x2[3*s+i],pbez->x2[3*s+j],
*				pbez->x3[3*s+i],pbez->x3[3*s+j],
*				pbez->x4[3*s+i],pbez->x4[3*s+j],sum);
*/
		   }
		}
/*		printf("(%i,x4,%.2f) finished \n\n",k,sum);
*/
		qratbez->weights[6*s+k] = sum;
	   }

	   /* x1 component */
	   for (k=0;k<=6;k++) { 	/* kth control point */
		/* \sum_{0 \leq i \leq 3, 0 \leq j \leq 3, i+j=k} */
		sum=0.;
		for (i=0;i<=3;i++) {
		   j=k-i;
	  	   if (j<=3 && j>=0) {
			foo1 = choose(3,i);	/* for some strange reason */
			foo2 = choose(3,j);	/* choose(3,i)*choose(3,j) */
			foo3 = choose(6,k);	/* returns 0, so I use foo's */
			sum += ((foo1*foo2)/foo3)
			      * (pbez->x1[3*s+i] * pbez->x1[3*s+j] +
				 pbez->x2[3*s+i] * pbez->x2[3*s+j] +
				 pbez->x3[3*s+i] * pbez->x3[3*s+j] -
				 pbez->x4[3*s+i] * pbez->x4[3*s+j]);
/*			printf("(%i,%i,%i,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)\n",
*				k,i,j,
*				pbez->x1[3*s+i],pbez->x1[3*s+j],
*			 	pbez->x2[3*s+i],pbez->x2[3*s+j],
*				pbez->x3[3*s+i],pbez->x3[3*s+j],
*				pbez->x4[3*s+i],pbez->x4[3*s+j],sum);
*/
		   }
		}
		qratbez->x1[6*s+k] = sum/qratbez->weights[6*s+k]; /* kth control point
							       on sth segment */
/*		printf("(%i,x1,%.2f) finished \n\n",k,qratbez->x1[6*s+k]); */
	   }

	   /* x2 */
	   for (k=0;k<=6;k++) {
		sum=0.;
		for (i=0;i<=3;i++) {	
		   j=k-i;
	  	   if (j<=3 && j>=0) {
			foo1 = choose(3,i);
			foo2 = choose(3,j);
			foo3 = choose(6,k);
			sum += ((foo1*foo2)/foo3)
			      * 2 * pbez->x1[3*s+i] * pbez->x4[3*s+j];
		   }
		}
		qratbez->x2[6*s+k] = sum/qratbez->weights[6*s+k];
	   }

	   /* x3 */
	   for (k=0;k<=6;k++) {
		sum=0.;
		for (i=0;i<=3;i++) {	
		   j=k-i;
	  	   if (j<=3 && j>=0) {
			foo1 = choose(3,i);
			foo2 = choose(3,j);
			foo3 = choose(6,k);
			sum += ((foo1*foo2)/foo3)
			      * 2 * pbez->x2[3*s+i] * pbez->x4[3*s+j];
		   }
		}
		qratbez->x3[6*s+k] = sum/qratbez->weights[6*s+k];
	   }

	   /* x4 */
	   for (k=0;k<=6;k++) {
		sum=0.;
		for (i=0;i<=3;i++) {	
		   j=k-i;
	  	   if (j<=3 && j>=0) {
			foo1 = choose(3,i);
			foo2 = choose(3,j);
			foo3 = choose(6,k);
			sum += ((foo1*foo2)/foo3)
			      * 2 * pbez->x3[3*s+i] * pbez->x4[3*s+j];
		   }
		}
		qratbez->x4[6*s+k] = sum/qratbez->weights[6*s+k];
	   }
	}	/* of sth cubic Bezier segment */
}

void test_M(double display_pbspl[4][MAXDISPLAYPTS], int pbspl_num)
{
	/* test if M maps correctly
	   by applying directly to each point, and compare
	   with output of later Bezier curve on sphere */
	
	int i;
	double v[3],x1,x2,x3,x4;
	printf("\nBeginning direct M test\n");
	bgnline();
	for (i=0;i<pbspl_num;i++) {
		M_pt(	display_pbspl[0][i],
			display_pbspl[1][i],
			display_pbspl[2][i],
			display_pbspl[3][i],&x1,&x2,&x3,&x4);
		v[0]=x1; v[1]=x2; v[2]=x4;
		v3d(v);
		if (fabs(sqrt(x1*x1+x2*x2+x3*x3+x4*x4) - 1) > .01)
		   printf("(%.2f,%.2f,%.2f,%.2f) is %.2f off sphere\n",
			x1,x2,x3,x4,sqrt(x1*x1+x2*x2+x3*x3+x4*x4) - 1.); 
		else	printf("(%.2f,%.2f,%.2f,%.2f) is OK\n",
				x1,x2,x3,x4);
	}
	endline();
	printf("Ending direct M test\n");
}

void M_pt (const double x1, const double x2, const double x3, const double x4,
	   double *y1, double *y2, double *y3, double *y4)
{
	double denom;

	denom=x1*x1+x2*x2+x3*x3+x4*x4;
	*y1=(x1*x1+x2*x2+x3*x3-x4*x4)/denom;
	*y2=(2*x1*x4)/denom;
	*y3=(2*x2*x4)/denom;
	*y4=(2*x3*x4)/denom;
}
