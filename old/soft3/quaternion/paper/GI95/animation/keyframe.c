/*
        File: old animation.c
        Author: J.K. Johnstone
        Last Modified: Oct. 26, 1994
        Purpose: Display of intermediate frames interpolating 
		 n orientations (quaternions) and n positions,
		 using rational Bezier curves.
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
#include "animation.h"

#include "bez.c"
#include "/usr/people/jj/cbin/fit.c"
#include "display.c"
#include "quaternion.c"	/* Dec. 6: cbin/quaternion.c now defines a quaternion */
			/* as a v4d rather than a v4dh.  Will want to eventually */
			/* make this change in animation directory, but don't */
			/* have time now. */
#include "drw.c"
#include "/usr/people/jj/cbin/vec.c"
#include "/usr/people/jj/cbin/misc.c"

int     xmax,ymax;      /* screen dimensions */
int     zmax;           /* z-buffer size */
/* int 	frame=0; */	/* present frame in animation */

main()
{
        unsigned int    m;              /* # of keyframes (quaternions) */
        V3d             pos[MAXFRAMES]; /* positions of reference vertex */
	Qion		q[MAXFRAMES];	/* keyframe quaternions */
        bspl_3d         directrixBspl;  /* directrix curve */
	bez_3d		directrixBez;
	double		display_dirbez[3][MAXDISPLAYPTS];
	int		dirbez_num;
	ratbez_4d	sphereRatBez;	/* Bezier curve on unit 4-sphere
				that defines orientation of object */
	double 		display_sphereratbez[4][MAXDISPLAYPTS];
	int		spherebez_num;
	int 		num_intermediate; /* # of intermediate frames between
				each pair of keyframes, counting keyframes */
	FILE 		*fp;

        long            animation_wid;
        Boolean         exitflag;       /* window */
        short           attached = 0;
        short           value;
        int             dev;
        int             i;

	/* m keyframe positions */
	fp = fopen("pos.input","r");
	inputV3d(&m,pos,fp);  /* n reference point positions */
	fclose(fp);

	/* m keyframe quaternions */
	fp = fopen("quaternion.input","r");
	inputQion(&m,q,fp);
	fclose(fp);

	/* fit directrix curve */
	fitCubicBspl_3d(m,pos,&directrixBspl); 
	/* need in Bezier form to match segments with spherical curve */
	bspl_to_bezier_3d(&directrixBspl,&directrixBez); 
	prepare_draw_bez_in_3d(&directrixBez,display_dirbez,&dirbez_num);

	/* spherical curve: already computed by sphereDrawing.c */
	fp = fopen("sphereratbez.input","r");
	input_ratbez4d(&sphereRatBez,fp);
	fclose(fp);
	print_ratbez(&sphereRatBez);
	prepare_draw_ratbez_in_4d(&sphereRatBez,display_sphereratbez,&spherebez_num);

	/* make knot sequences the same */
	for (i=0;i<=directrixBez.L;i++) {
		directrixBez.knots[i] = sphereRatBez.knots[i];
	}
/*	print_bez3d(&directrixBez); */	

	animation_wid = initialize_3d_window("Animation",650,1250,200);

        winset(animation_wid);
        exitflag=FALSE;
	num_intermediate = 7;
        while (exitflag == FALSE) {
		animate(num_intermediate,m,q,pos,&sphereRatBez, display_sphereratbez, spherebez_num,
				&directrixBez,display_dirbez,dirbez_num);
                while ((exitflag == FALSE) && (qtest() || !attached)) {
                        dev = qread(&value);
                        if (((dev == ESCKEY) && (value == 0)))
                                exitflag = TRUE;
/*			else if (dev == LEFTMOUSE)
*				frame++;
*				if (frame>99)
*					frame = 0; */ /* loop back to beginning of animation */
                        else if (dev == WINFREEZE || dev == WINTHAW || dev == REDRAWICONIC) {
                                frontbuffer(TRUE);
                                pushmatrix();
                                reshapeviewport();
				animate(num_intermediate,m,q,pos,&sphereRatBez,display_sphereratbez,spherebez_num,
					&directrixBez,display_dirbez,dirbez_num);
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

void animate      (const int 	      n,
		   const unsigned int m,
		   const Qion	      keyq[],
		   const V3d	      keypos[],
		   const ratbez_4d      *sphereratbez,
		   double display_sphereratbez[4][MAXDISPLAYPTS],
		   const int spherebez_num,
		   const bez_3d     *directrixbez,
	           double display_dirbez[3][MAXDISPLAYPTS],
                   const int dirbez_num)
{
	/* draw animation loop of object (cube for prototype) */
	/* of n frames */
	/* with orientation controlled by spherebez */
	/* and position by directrixbspl */

        extern int xmax,ymax,zmax;
	double delta_q; 	/* increment in t between frames for quaternions */
	double delta_p;		/* increment in t between frames for positions */
	double t_q;		/* parameter value for this frame for quaternion */
	double t_p;		/* parameter value for this frame for position */
/*	extern int frame; */	/* which frame to draw (from 0 to n-1) */
	int frame;
	Qion   q;		/* orientation of this frame */
	V3d    pt;		/* position of this frame */
	int i;

	/* czclear(0xFFFF55,zmax); */
	czclear(0xFFFFFF,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 20.0);
        pushmatrix();
        ortho(-11*xmax/ymax,11*xmax/ymax,-11,11,-11,11);

	/* draw keyframes */
	RGBcolor(0,0,0);
	for (i=0;i<m;i++) {
		draw_oriented_wire_cube (keyq[i],keypos[i]); 
	}

	/* draw quaternions and quaternion curve for debugging */
/*	RGBcolor(0,255,0);
*	for (i=0;i<m;i++) {
*		draw_quaternion(keyq[i]);
*	}
*	draw_curve_on_4d_sphere (display_sphereratbez, spherebez_num);
*/

	/* between each pair of keyframes */
	for (i=0;i<m-1;i++) {

		/* generating intermediate frames between keyframe i and i+1,
			associated with knots i and i+1 on the Bezier curves */
		/* draw intermediate frames */	
	        RGBcolor(255,100,0);
		/* instead of frame from 0 to n-1, do from 1 to n-2
		   so that keyframes are not covered */
		for (frame=1;frame<n-1;frame++) {
			/* quaternion */
			delta_q = (sphereratbez->knots[i+1] - 
				   sphereratbez->knots[i]) / (n-1);
			t_q = sphereratbez->knots[i] + frame*delta_q;
	
			point_on_ratbez_4dh (sphereratbez, t_q, q);
		
			/* position */
			point_on_bez_3d (directrixbez, t_q, pt);
	
/*			printf("Drawing object at position (%.2f,%.2f,%.2f) and quaternion (%.2f,%.2f,%.2f,%.2f,%.2f)\n",
*				pt[0],pt[1],pt[2],
*				q[0],q[1],q[2],q[3],q[4]);
*	
*			printf("frame = %i \t t_q = %.2f \t t_p = %.2f \n",frame,t_q,t_p);
*/
			draw_oriented_wire_cube (q,pt);  
		}
	}
/***********************************************************/

/* animation version */
/*	frame++; 
*	if (frame > (n-1)) 
*		frame = 0;
*/

/*	draw_curve_in_3d (display_dirbez, dirbez_num); */
        popmatrix();
        swapbuffers();
}

