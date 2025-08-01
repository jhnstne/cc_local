/*
        File: animation.c
        Author: J.K. Johnstone
        Last Modified: Feb. 10, 1995
        Purpose: Given n keyframes (n quaternions and n reference point positions),
	  	 generate a smooth rational animation through these keyframes.
		 Either display keyframes (-k), keyframes plus intermediate 
		 frames (-i), or animation (-a).

		 Reference: John K. Johnstone and James T. Williams (1995)
		 `Rational control of orientation for animation'. 
		 Graphics Interface '95.
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
#include "/usr/people/jj/cbin/drw.h"
#include "animation.h"

#include "/usr/people/jj/cbin/vec.c"
#include "bez.c"
#include "/usr/people/jj/cbin/fit.c"
#include "display.c"
#include "quaternion.c"	/* Dec. 6: cbin/quaternion.c now defines a quaternion */
			/* as a v4d rather than a v4dh.  Will want to eventually */
			/* make this change in animation directory, but don't */
			/* have time now. */
#include "/usr/people/jj/cbin/drw.c"
#include "/usr/people/jj/cbin/misc.c"

int     xmax,ymax;      /* screen dimensions */
int     zmax;           /* z-buffer size */
int 	frame=0; 	/* present frame in animation */
int 	num_frames=100;	/* number of frames for animation */

int 	KEYFRAMES = 0;	/* display keyframes only? */
int 	ALLFRAMES = 0;	/* display keyframes and intermediate frames? */
int 	ANIMATION = 0;	/* animate the object? */
int 	DEBUG = 0;		/* print debug statements? */

main(int argc, char *argv[])
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
				each pair of keyframes, not counting keyframes */
	FILE 		*fp;

        long            animation_wid;
        Boolean         exitflag;       /* window */
        short           attached = 0;
        short           value;
        int             dev;
        int             i;
	extern int KEYFRAMES, ALLFRAMES, ANIMATION, DEBUG;

	for (i=0; i<argc; i++) {
		if (!(strncmp(argv[i], "-k", 2)))
		    KEYFRAMES = 1;
		else if (!(strncmp(argv[i], "-i", 2)))
		    ALLFRAMES = 1;
		else if (!(strncmp(argv[i], "-a", 2)))
		    ANIMATION = 1;
		else if (!(strncmp(argv[i], "-d", 2)))
		    DEBUG = 1;
	}

	/* m keyframe positions */
	fp = fopen("pos.input","r");
	inputV3d(&m,pos,fp);  
	fclose(fp);

	/* m keyframe quaternions */
	fp = fopen("quaternion.input","r");
	inputQion(&m,q,fp);
	fclose(fp);

	/* object to animate */
	fp = fopen("object.input","r");
	inputPolygon(&k,p);
	/* START HERE */
	generate_extruded_tmesh   (int k, 
				double p[MAX][3],
				double dist,
				double V[MAX][3], 
				double N[MAX][3], 
				int *num_meshes,
				int num_per_mesh[MAXMESHES]);


	/* fit directrix curve */
	fitCubicBspl_3d(m,pos,&directrixBspl); 
	/* need in Bezier form to match segments with spherical curve */
	bspl_to_bezier_3d(&directrixBspl,&directrixBez); 
	prepare_draw_bez_in_3d(&directrixBez,display_dirbez,&dirbez_num);

	/* spherical curve: already computed by sphereDrawing.c */
	fp = fopen("sphereratbez.input","r");
	input_ratbez4d(&sphereRatBez,fp);
	fclose(fp);
	if (DEBUG) 
		print_ratbez(&sphereRatBez);
	prepare_draw_ratbez_in_4d(&sphereRatBez,display_sphereratbez,&spherebez_num);

	/* make knot sequences the same */
	for (i=0;i<=directrixBez.L;i++) {
		directrixBez.knots[i] = sphereRatBez.knots[i];
	}

	animation_wid = initialize_3d_window("Animation",650,1250,200);
        winset(animation_wid);
        exitflag=FALSE;
	num_intermediate = 5;
        while (exitflag == FALSE) {
		animate(num_intermediate,m,q,pos,&sphereRatBez,
			display_sphereratbez,spherebez_num,
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
				animate(num_intermediate,m,q,pos,&sphereRatBez,
					display_sphereratbez,spherebez_num,
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
		   const ratbez_4d    *sphereratbez,
		   double display_sphereratbez[4][MAXDISPLAYPTS],
		   const int 	      spherebez_num,
		   const bez_3d       *directrixbez,
	           double display_dirbez[3][MAXDISPLAYPTS],
                   const int   	      dirbez_num)
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
	extern int frame; 	/* which frame to draw (from 0 to n-1) */
	double adelta;  	/* animation's t-increment */
	extern int num_frames; 	/* number of frames to create for animation */
	int iframe;		/* intermediate frame counter */
	Qion   q;		/* orientation of this frame */
	V3d    pt;		/* position of this frame */
	int i;
	extern int KEYFRAMES, ALLFRAMES, ANIMATION, DEBUG;

	czclear(0xFFFF55,zmax);	/* originally, OxFFFFFF white */
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 20.0);
        pushmatrix();
        ortho(-11*xmax/ymax,11*xmax/ymax,-11,11,-11,11);

	/* draw keyframes */
	RGBcolor(0,0,0);
	for (i=0;i<m;i++) {
		draw_oriented_wire_cube (keyq[i],keypos[i]); 
	}

	/* AND HERE */
extern void 	draw_oriented_shaded_object (Qion q, V3d pos,
				  int n, int num_per_mesh[MAXMESHES],
				  double V[MAX][3], double N[MAX][3]);

	if (ALLFRAMES) {
	    /* display intermediate frames */
	    for (i=0;i<m-1;i++) {
		/* generating intermediate frames between keyframe i and i+1,
	 	   associated with knots i and i+1 on the Bezier curves */
	        RGBcolor(255,100,0);
		for (iframe=0;iframe<n;iframe++) {
			/* quaternion */
			delta_q = (sphereratbez->knots[i+1] - 
				   sphereratbez->knots[i]) / (n+1);
			t_q = sphereratbez->knots[i] + (iframe+1)*delta_q;
			point_on_ratbez_4dh (sphereratbez, t_q, q);

			/* position */
			point_on_bez_3d (directrixbez, t_q, pt);

			draw_oriented_wire_cube (q,pt);  

			if (DEBUG) {
			    printf("Drawing object at position (%.2f,%.2f,%.2f) and quaternion (%.2f,%.2f,%.2f,%.2f,%.2f)\n",
				pt[0],pt[1],pt[2],
				q[0],q[1],q[2],q[3],q[4]);
			    printf("frame = %i \t t_q = %.2f \t t_p = %.2f \n",
				iframe,t_q,t_p);
			}
		}
	    }
	}

	if (ANIMATION) {
	        RGBcolor(255,100,0);
		/* parameter interval of quaternion curve is (knots[0],knots[m-1]) */
		adelta = (sphereratbez->knots[m-1] - sphereratbez->knots[0])
			/num_frames;
		t_q = sphereratbez->knots[0] + frame*adelta;
		if (t_q > sphereratbez->knots[m-1])
			/* truncate at end, to interpolate last keyframe */
			t_q = sphereratbez->knots[m-1];
		point_on_ratbez_4dh (sphereratbez, t_q, q);
		point_on_bez_3d (directrixbez, t_q, pt);
		draw_oriented_wire_cube (q,pt);  

		frame++; 
		if (frame > num_frames+1) {
/*		   ceil((sphereratbez->knots[m-1]-sphereratbez->knots[0])/adelta)) */
			frame = 0; 
			sleep(2);
		}
	}	
	if (DEBUG)
		draw_curve_in_3d (display_dirbez, dirbez_num);

        popmatrix();
        swapbuffers();
}

