/*
        File: animation.c
        Author: J.K. Johnstone
        Last Modified: May 4, 1995
        Purpose: Given n keyframes (n quaternions and n reference point positions),
	  	 generate a smooth rational animation through these keyframes.

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
#include "/usr/people/jj/cbin/display.h"
#include "/usr/people/jj/cbin/lights.h"
#include "/usr/people/jj/cbin/materials.h"
#include "/usr/people/jj/cbin/misc.h"

#include "quaternion.h"
#include "/usr/people/jj/cbin/drw.h"
#include "/usr/people/jj/cbin/M.h"
#include "animation.h"

#include "/usr/people/jj/cbin/vec.c"
#include "bez.c"
#include "/usr/people/jj/cbin/fit.c"
#include "/usr/people/jj/cbin/display.c"
#include "quaternion.c"	/* Dec. 6: cbin/quaternion.c now defines a quaternion */
			/* as a v4d rather than a v4dh.  Will want to eventually */
			/* make this change in animation directory, but don't */
			/* have time now. */
#include "/usr/people/jj/cbin/drw.c"
#include "/usr/people/jj/cbin/misc.c"
#include "/usr/people/jj/cbin/M.c"

static char *RoutineName;
static void usage()
{
  printf("Usage is %s\n", RoutineName);
  printf("\t[- ] - default: keyframes\n");
  printf("\t[-D] - input demo\n");
  printf("\t[-I] - qions + their (I)nverse images under M\n");
  printf("\t[-s] - cubic Bspl (s)pace curve (+ ctrlpolygon)\n");
  printf("\t[-S] - cubic Bezier (s)pace curve (+ ctrlpolygon)\n");
  printf("\t[-m] - M(space curve) = sphere curve (+ ctrlpolygon)\n");
  printf("\t[-i] - (i)ntermediate+key frames (+ sphere curve)\n");
  printf("\t[-a] - (a)nimation (while tracing spherical curve)\n");
  printf("\t[-A] - (A)nimation without keyframes\n");
  printf("\t[-p] - antiPodal (bad) choice of 2nd quaternion\n");
  printf("\t[-c] - (c)ontrolling speed along curve + animation\n");
  printf("\t[-d] - (d)ebug\n");
  printf("\t[  ] - Major examples: maple leaf, hockey stick\n");
  printf("\t[- ]  - no example of manipulating space Bezier control polygon\n");
  printf("\t     since it is uniquely defined by interpolation and param\n");
  printf("\t     (e.g., centripetal); manipulation of speed is only\n");
  printf("\t     realistic example of control\n");
}

int     xmax,ymax;      /* screen dimensions */
int     zmax;           /* z-buffer size */
float	rotx,roty,rotz;
int	rot_bool=0;	/* rotate quaternion sphere? */
int 	frame=0; 	/* present frame in animation */
int	slowdownctr=0;
int	slowdownfactor=2;
int 	num_frames=100;	/* number of frames for animation */
long    animation_wid, sphere_wid;
Qion	q;		/* orientation of this frame */

int	KEYS=1;		/* draw keyframes? */
int	INPUTDEMO=0;	/* draw one keyframe at a time to illustrate input? */
int	INVERSE=0;	/* draw inverse images of quaternions? */
int 	ALLFRAMES = 0;	/* display keyframes and intermediate frames? */
int 	ANIMATION = 0;	/* animate the object? */
int 	DEBUG = 0;	/* print debug statements? */
int	SPACEBSPL = 0;	/* draw space curve (as B-spline)? */
int 	SPACEBEZ = 0;   /* draw space curve (as Bezier)? */
int 	ORICURVE = 0;   /* draw orientation curve? */
int	ORICTRLPOLY = 0;/* draw control polygon of orientation curve? */
int	ANTIPODAL=0;	/* use antipodal quaternion for 2nd quaternion? */
int	BIGSPHEREWIN=0; /* use bigger sphere window (since its only one)? */
int 	presentm=0;	/* present subset of keyframes to draw in INPUTDEMO */


main(int argc, char *argv[])
{
	extern int KEYS, INPUTDEMO, ALLFRAMES, ANIMATION, DEBUG, ANTIPODAL;
	extern int INVERSE, SPACEBSPL, SPACEBEZ, ORICURVE, ORICTRLPOLY;
	extern int presentm;
	unsigned int    m,n;            /* # of keyframes (quaternions) (m=n)*/

        V3d             pos[MAXFRAMES]; /* positions of reference vertex */
        bspl_3d         directrixBspl;  /* position curve */
	bez_3d		directrixBez;
	REAL		display_dirbez[3][MAXDISPLAYPTS];
	int		dirbez_num;

	Qion		q[MAXFRAMES];	/* keyframe quaternions */
	ratbez_4d	sphereRatBez;	/* orientation curve on unit 4-sphere */
				/* a rational sextic Bezier spline */
	REAL 		display_sphereratbez[4][MAXDISPLAYPTS];
	int		spherebez_num;
	V4d		pt1,pt2;

        v4dh            p[MAXFRAMES];   /* images of quaternions under M^{-1} */
	bspl_4d         pBspl;     	/* cubic B-spline interpolating p[] */
        REAL          display_pbspl[4][MAXDISPLAYPTS];
        int             pbspl_num;      /* # of display points */
        bez_4d          pBez;      	/* cubic Bezier spline interpolating p */
        REAL          display_pbez[4][MAXDISPLAYPTS];
        int             pbez_num;

	int 		num_intermediate; /* # of intermediate frames between
				each pair of keyframes, not counting keyframes */
	FILE 		*fp;
	
	int k;				/* # of vertices in object polygon */
	REAL poly[MAX][3];   /* vertices of object polygon */
	REAL dist;		/* extrusion distance */
	REAL V[MAX][3];   /* vertices of tmesh of object */
	REAL N[MAX][3];   /* normals  of tmesh of object */
	int swap[MAX] = {0}; /* swap bits, initialized to 0 for no swaptmesh() */
	int num_meshs;
	int num_per_mesh[MAXMESHES]; /* number of vertices in ith tmesh */
		
        Boolean         exitflag;       /* window */
        short           attached = 0;
        short           value;
        int             dev;
        int             i,j,count;
	int		ArgsParsed=0;
	extern int	rot_bool;

        RoutineName = argv[ArgsParsed++];
        for (; ArgsParsed<argc; ArgsParsed++)
                if ('-' == argv[ArgsParsed][0])
                   switch (argv[ArgsParsed][1]) {
		     case 'D':
			INPUTDEMO=1;
			KEYS=0;
			break;
		     case 'I':
			INVERSE=1;
			BIGSPHEREWIN=1;
			break;
		     case 'i':
			ALLFRAMES=1;
			ORICURVE=1;
			break;
		     case 'a':
			ANIMATION=1;
			ORICURVE=1;
			break;
		     case 'A':
			ANIMATION=1;
			ORICURVE=1;
			KEYS=0;
			break;
		     case 's':
			INVERSE=1;
			SPACEBSPL=1;
			BIGSPHEREWIN=1;
			break;
		     case 'S':
			INVERSE=1;
			SPACEBEZ=1;
			BIGSPHEREWIN=1;
			break;
		     case 'm':
			ORICURVE=1;
			ORICTRLPOLY=1;
			break;
		     case 'p':
			ANTIPODAL=1;
			break;
		     case 'd':
			DEBUG=1;
			break;
                     case 'h':
                     default:
                        usage(); exit(-1);
                   }

	/* m keyframe positions */
	fp = fopen("pos.input","r");
	inputV3d(&m,pos,fp);  
	fclose(fp);

	/* m keyframe orientations */
	fp = fopen("quaternion.input","r");
	inputQion(&n,q,fp);
	if (m!=n) {
		printf("Need same number of positions as quaternions.\n");
		exit(-1);
	}
/****************************************************************/
	/* testing antipodal quaternions */
	/* just switch the second quaternion to its antipodal counterpart */
	if (ANTIPODAL)
	for (i=1; i<2; i++) {
	   q[i][1] = -q[i][1];
	   q[i][2] = -q[i][2];
	   q[i][3] = -q[i][3];
	   q[i][4] = -q[i][4];
	}  
/***************************************************************/
	fclose(fp);

	        /* project: orientations could be input interactively
	           using a Motif scrollbar (with angle (s) giving 
		   immediate feedback by rotating the object to the 
		   associated orientation, so that the user can alter/correct 
		   input immediately.
		   For context, the previous orientations should
		   be displayed as ghosts or in a nearby window. */

	/* object to animate: cube, maple leaf, hockey stick, ... */
	fp = fopen("object.input","r");
	inputPolygon(&k,poly,fp);
	generate_extruded_tmesh (k-1,poly,.3,V,N,swap,&num_meshs,num_per_mesh);
	if (DEBUG) {
	   count=0;
	   for (i=0; i<num_meshs; i++) {
		printf("Next mesh.\n");
		for (j=0; j<num_per_mesh[i]; j++) {
			printf("V[%i]=(%.2f,%.2f,%.2f)\t N[%i]=(%.2f,%.2f,%.2f)\n",
				count,V[count][0],V[count][1],V[count][2],
				count,N[count][0],N[count][1],N[count][2]);
			count++;
		}
	   }
	}
	
	/* fit position curve */
	fitCubicBspl_3d(m,pos,&directrixBspl); 
	/* need in Bezier form to match segments with orientation curve */
	bspl_to_bezier_3d(&directrixBspl,&directrixBez); 
	prepare_draw_bez_in_3d(&directrixBez,display_dirbez,&dirbez_num);

	/* fit preimage of orientation curve */
	for (i=0;i<m;i++) {      /* quaternions --> 4-space */
                invM(q[i],p[i]);
		if (DEBUG)
		   printf("Inverse image of quaternion (%.1f,%.1f,%.1f,%.1f) is (%.1f,%.1f,%.1f,%.1f).\n",
			q[i][1]/q[i][0],q[i][2]/q[i][0],q[i][3]/q[i][0],
			q[i][4]/q[i][0],
			p[i][1]/p[i][0],p[i][2]/p[i][0],p[i][3]/p[i][0],
			p[i][4]/p[i][0]);
	}
        fitCubicBspl_4dh(m,p,&pBspl);
	if (DEBUG) 	print_bspl(&pBspl);
        prepare_draw_bspl_in_4d(&pBspl,display_pbspl,&pbspl_num);
	bspl_to_bezier_4d(&pBspl,&pBez);
        if (DEBUG)	print_bez(&pBez);
        prepare_draw_bez_in_4d(&pBez,display_pbez,&pbez_num);

        /* cubic Bezier 4-space --> rational sextic Bezier orientation curve */
        M_rat(&pBez,&sphereRatBez);

        if (DEBUG) 	print_ratbez(&sphereRatBez);
        prepare_draw_ratbez_in_4d(&sphereRatBez,display_sphereratbez,&spherebez_num);

       		/* fp = fopen("sphereratbez.input","w");
	        output_ratbez4d(&qCurveRatBez,fp);
        	fclose(fp); */

		/* spherical curve: already computed by sphereDrawing.c */
		/* fp = fopen("sphereratbez.input","r");
		input_ratbez4d(&sphereRatBez,fp);
		fclose(fp); */

/*	sphereRatBez.knots[0] = 0;  */ /* arbitrary choice */
/*	for (i=1; i<=sphereRatBez.L; i++) {
		pt1[0] = sphereRatBez.x1[3*(i-1)];
		pt1[1] = sphereRatBez.x2[3*(i-1)];
		pt1[2] = sphereRatBez.x3[3*(i-1)];
		pt1[3] = sphereRatBez.x4[3*(i-1)];
		pt2[0] = sphereRatBez.x1[3*i];
		pt2[1] = sphereRatBez.x2[3*i];
		pt2[2] = sphereRatBez.x3[3*i];
		pt2[3] = sphereRatBez.x4[3*i];
		sphereRatBez.knots[i] = sphereRatBez.knots[i-1] + 
					spherical_dist_4d(pt1,pt2);
	} */

	/* set knot sequence for arc length along position curve */
	/* so that we get a smooth animation */
	/* in future, the speed along the position and orientation curves */
	/* should be decoupled so that both can be arc length */
	for (i=0;i<=directrixBez.L;i++) 
		sphereRatBez.knots[i] = directrixBez.knots[i];

	/* animation_wid = initialize_3d_window("Animation",640,1240,200); */
	if (BIGSPHEREWIN)
	   animation_wid = initialize_3d_window("Animation",700,800,700);
	else
	animation_wid = initialize_3d_window("Animation",334,605,625);
	/* sphere_wid = initialize_3d_window("Quaternion sphere",25,625,200); */
	if (BIGSPHEREWIN)
	   sphere_wid = initialize_3d_window("Quaternion sphere",100,555,600);
	else
	sphere_wid = initialize_3d_window("Quaternion sphere",50,321,625);
	rotx = 900; roty = 0; rotz = 400; /* 730 for #2 visible control polygon */
        exitflag=FALSE;
	num_intermediate = 2;
        while (exitflag == FALSE) {
		animate(num_intermediate,m,q,pos,&sphereRatBez,
			display_sphereratbez,spherebez_num,
			V,N,swap,num_meshs,num_per_mesh,
			&directrixBez,display_dirbez,dirbez_num);
		sphere(m,q,p, display_pbspl, pbspl_num, &pBspl,
			      display_pbez, pbez_num, &pBez,
                              display_sphereratbez, spherebez_num, &sphereRatBez);
                while ((exitflag == FALSE) && (qtest() || !attached)) {
                        dev = qread(&value);
                        if (((dev == ESCKEY) && (value == 0)))
                                exitflag = TRUE;
			else if (dev == LEFTMOUSE)
				rot_bool=1;
/*				frame++;
*				if (frame>99)
*					frame = 0; */ /* loop back to beginning of animation */
			else if (dev == RIGHTMOUSE) {
				if (value && presentm < m) 
				   presentm = presentm+1;  /* next frame */
			}
                        else if (dev == WINFREEZE || dev == WINTHAW || dev == REDRAWICONIC) {
                                frontbuffer(TRUE);
                                pushmatrix();
                                reshapeviewport();
				animate(num_intermediate,m,q,pos,&sphereRatBez,
					display_sphereratbez,spherebez_num,
					V,N,swap,num_meshs,num_per_mesh,
					&directrixBez,display_dirbez,dirbez_num);
				sphere(m,q,p, display_pbspl, pbspl_num, &pBspl,
				      display_pbez, pbez_num, &pBez,
                        	      display_sphereratbez, spherebez_num, &sphereRatBez);
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
		   REAL display_sphereratbez[4][MAXDISPLAYPTS],
		   const int 	      spherebez_num,
		   REAL V[MAX][3],
		   REAL N[MAX][3],
		   int swap[MAX],
		   int num_meshs,
		   int num_per_mesh[MAXMESHES],
		   const bez_3d       *directrixbez,
	           REAL display_dirbez[3][MAXDISPLAYPTS],
                   const int   	      dirbez_num)
{
	/* draw animation loop of object (cube for prototype) */
	/* of n frames */
	/* with orientation controlled by spherebez */
	/* and position by directrixbspl */

        extern int xmax,ymax,zmax;
	REAL delta_q; 	/* increment in t between frames for quaternions */
	REAL delta_p;		/* increment in t between frames for positions */
	REAL t_q;		/* parameter value for this frame for quaternion */
	REAL t_p;		/* parameter value for this frame for position */
	extern int frame; 	/* which frame to draw (from 0 to n-1) */
	REAL adelta;  	/* animation's t-increment */
	extern int num_frames; 	/* number of frames to create for animation */
	int iframe;		/* intermediate frame counter */
	V3d    pt;		/* position of this frame */
	extern Qion q;		/* orientation of this frame */
	int i;
	extern int KEYS,ALLFRAMES,ANIMATION,DEBUG,presentm;

	winset(animation_wid);
	czclear(0xFFFF55,zmax);	/* originally, OxFFFFFF white */
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 20.0);
        pushmatrix();
        ortho(-11*xmax/ymax,11*xmax/ymax,-11,11,-11,11);  /* used to be 11 */

	lmbind(MATERIAL,2);	/* reddest maple leaf */

	/* draw one keyframe at a time for input example */
	if (INPUTDEMO) {
	   for (i=0; i<presentm; i++)
		draw_oriented_shaded_object (keyq[i],keypos[i],
					num_meshs,num_per_mesh, V, N, swap);
	}
	else if (KEYS)
	   /* draw keyframes */
	   for (i=0;i<m;i++) {
/*		draw_oriented_wire_cube (keyq[i],keypos[i]);  */
		draw_oriented_shaded_object (keyq[i],keypos[i],
					num_meshs,num_per_mesh, V, N, swap);
	   }

	lmbind(MATERIAL,2);	

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

/*			draw_oriented_wire_cube (q,pt);   */
			draw_oriented_shaded_object (q,pt,num_meshs,num_per_mesh, 
				V, N,swap);

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
		/* param interval of quaternion curve is (knots[0],knots[m-1]) */
		adelta = (sphereratbez->knots[m-1] - sphereratbez->knots[0])
			/num_frames;
		t_q = sphereratbez->knots[0] + frame*adelta;
		if (t_q > sphereratbez->knots[m-1])
			/* truncate at end, to interpolate last keyframe */
			t_q = sphereratbez->knots[m-1];
		point_on_ratbez_4dh (sphereratbez, t_q, q);
		point_on_bez_3d (directrixbez, t_q, pt);
/*		draw_oriented_wire_cube (q,pt);   */
		draw_oriented_shaded_object (q,pt,num_meshs,num_per_mesh, 
				V, N, swap);

		slowdownctr++;
		if (slowdownctr > slowdownfactor) {
			frame++;
			slowdownctr=0;
		}
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

void sphere(const int m, const Qion keyq[], const v4dh p[], 
            REAL display_pbspl[4][MAXDISPLAYPTS], 
            const int pbspl_num,
            const bspl_4d *pBspl,
            REAL display_pbez[4][MAXDISPLAYPTS],
            const int pbez_num,
            const bez_4d *pBez,
            REAL display_sphereratbez[4][MAXDISPLAYPTS],  
            const int spherebez_num, 
            const ratbez_4d *sphereRatBez)
{
        /* Visualize quaternion 4d-sphere, unit quaternions KEYQ and */
        /* their images P under M^{-1}, in 3d through projection. */
        /* Sphere is displayed as several lines of longitude (circles) for speed. */

	/* projection: 
	   show sphere, quaternions and images in 3-space,
           by zeroing the x_3 component of quaternion,
           and the x_2 component in 4-space:
           M^{-1}(1,x_1,x_2,0,x_4) = (\pm 2 \sqrt{(x_0-x_1)/2},
                                       x_2,0,x_4,x_0 - x_1)
           M((\pm 2 \sqrt{(x_0-x_1)/2},x_2,0,x_4,x_0 - x_1) = 
                                       (y_0,y_1,y_2,0,y_4)
           that is, can project to 3dspace with x_3=0 for viz of sphere
           and x_2=0 for viz of 4-space */

	extern int INVERSE,presentm;
        extern int xmax,ymax,zmax;
	extern Qion q;		/* orientation of this frame */
	extern int rot_bool;
        int i;

	winset(sphere_wid);
	czclear(0xFFFF55,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 10.0);
	pushmatrix();
        ortho(-2*xmax/ymax,2*xmax/ymax,-2,2,-2,2);
 	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');

	/*      define a fn. `orientation(x,y,z,rx,ry,rz)' to accompany ortho command; */
	/*      polarview(dist, azimuth, incidence, twist);  */

        RGBcolor(255,255,255);
        draw_wire_sphere(0,0,0,1); 		/* sphere */

	lmbind(MATERIAL,2);	/* red, to match maple leaf */
	if (INPUTDEMO && !INVERSE)
	   for (i=0; i<presentm; i++)
                draw_quaternion(keyq[i]);	/* quaternions */
	else if (KEYS || INVERSE)
	   for (i=0; i<m; i++)
                draw_quaternion(keyq[i]);	/* quaternions */

	if (INVERSE) {
		lmbind(MATERIAL,3);	/* blue, it's cold away from hot sphere */
		if (INPUTDEMO)
		   for (i=0; i<presentm; i++)
	        	draw_4dhpoint(p[i]);	/* M^{-1}(quaternions) */
		else
		   for (i=0; i<m; i++)
	        	draw_4dhpoint(p[i]);	/* M^{-1}(quaternions) */
	}
 
	if (SPACEBSPL) {
	        RGBcolor(255,255,0);
	        draw_curve_in_4d (display_pbspl,pbspl_num);   /* space curve */
	        draw_bspl_control_in_4d (pBspl);
	}

        if (SPACEBEZ) {					     /* space curve */
		RGBcolor(0,0,255);
	        draw_curve_in_4d (display_pbez,pbez_num);
	        draw_bez_control_in_4d (pBez);
	}

	/*      test_M(display_pbspl,pbspl_point_num); */ /* test if M maps correctly
                by applying directly to each point, and drawing
                to compare with output of later Bezier curve on sphere */

	if (ORICURVE) {
	        RGBcolor(255,0,0);			/* orientation curve! */
	        linewidth(3);
	        draw_curve_on_4d_sphere (display_sphereratbez,spherebez_num);
	}
	if (ORICTRLPOLY) {
		RGBcolor(0,0,0);
		linewidth(3);
	        draw_ratbez_control_on_4d_sphere (sphereRatBez);
	}
	
	if (ANIMATION) {
		lmbind(MATERIAL,5);	/* yellow, to stand out */
		draw_quaternion(q);	/* bead moving along orientation curve */
	}

	if (rot_bool) {
                rotx += 20; 
	        roty += 20;
                rotz += 20; 
	} 

	lmbind(MATERIAL,1);	/* return original material */
        linewidth(1);
	popmatrix();
	swapbuffers();
}

void inputPolygon (int *k, REAL p[MAX][3], FILE *fp)
{
	char string[15];
	int i;
	
	/* put in different file */
	fscanf(fp,"%s",string);	/* NUM */
	fscanf(fp,"%i",k); 
	fscanf(fp,"%s",string);  /* VERTICES */
	for (i=0;i<*k; i++) 
		fscanf(fp,"%f %f %f",p[i],p[i]+1,p[i]+2);
}
