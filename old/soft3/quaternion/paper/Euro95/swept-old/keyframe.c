/*
        File: keyframe.c
        Author: J.K. Johnstone
        Last Modified: Dec. 5, 1994
        Purpose: Display of keyframe curves, for swept surface program.
*/

#include <device.h>
#include <gl/gl.h>
#include <gl/sphere.h>
#include <math.h>
#include <stdio.h>

#include "/usr/people/jj/cbin/vec.h"
#include "/usr/people/jj/cbin/bez.h"
#include "/usr/people/jj/cbin/display.h"
#include "/usr/people/jj/cbin/lights.h"
#include "/usr/people/jj/cbin/materials.h"
#include "/usr/people/jj/cbin/misc.h"
#include "/usr/people/jj/cbin/quaternion.h"
#include "/usr/people/jj/cbin/drw.h"
#include "keyframe.h"

#include "/usr/people/jj/cbin/bez.c"
#include "/usr/people/jj/cbin/display.c"
#include "/usr/people/jj/cbin/quaternion.c"
#include "/usr/people/jj/cbin/drw.c"
#include "/usr/people/jj/cbin/vec.c"
#include "/usr/people/jj/cbin/misc.c"

int     xmax,ymax;      /* screen dimensions */
int     zmax;           /* z-buffer size */
long	org[2],size[2];	/* window characteristics */
double	winmin,winmax;	/* desired window dimensions */
int 	rotx,roty,rotz,initrotx,initroty,initrotz;
float	transx,transy,transz;
float	zoom=1;
int	rot_bool=0;	/* start rotating? */

main()
{
	FILE 		*fp;
	ratbez_3d	sweepcurve; /* rational Bezier curve sweeping out the surface */
				    /* with first control point at the origin */
	double 		display_sweepcurve[3][MAXDISPLAYPTS];
	int		sweepcurve_num;

	unsigned int	m;	    /* # of curve instances to be interpolated */
	V3d		pos[MAXINST];	/* positions of curve */
	ratbez_4d	ori;		/* orientation curve */
	int 		scalem;		/* hopefully equal to m */
	double		scale[MAXINST];	/* scales */
	
        long            sweep_wid;
        Boolean         exitflag;       /* window */
        short           attached = 0;
        short           val,mval[2],lastval[2];
        Device          dev,mdev[2];
	int		leftmouse_down=0;
	int		middlemouse_down=0;
	int		rightmouse_down=0;
	double		wincen[3];
	double		xmin_data,xmax_data,ymin_data,ymax_data,
				zmin_data,zmax_data; /* min/max input coords */
	extern		long org[2],size[2];
	extern 		double winmin,winmax;
	extern 		int rotx,roty,rotz,initrotx,initroty,initrotz;
	extern		float transx,transy,transz;
	extern 		float zoom;
        int             i,j,k,l,p,s,I,J;

	/*****************************************************/
	/* 		   sweep curve	 		     */
	/*****************************************************/

	fp = fopen("curve.input","r");
	input_ratbez3d(&sweepcurve,fp);
	fclose(fp);
	if (!(sweepcurve.x1[0] == 0 && sweepcurve.x2[0] == 0 && sweepcurve.x3[0] == 0))
		printf("First control point of the sweep curve does not lie at origin, as it must!\n");
	printf("Sweep curve.\n");
	print_ratbez3d(&sweepcurve);
	printf("\n");
	prepare_draw_ratbez_in_3d(&sweepcurve,display_sweepcurve,&sweepcurve_num);

	/*****************************************************/
	/* 		   position curve 		     */
	/*****************************************************/

	/* m (reference point) positions of curve */
	fp = fopen("pos.input","r");
	inputV3d(&m,pos,fp);
	fclose(fp);

	/*****************************************************/
	/* 		   orientation curve 		     */
	/*****************************************************/

	/* already computed by ../animation/sphereDrawing.c */
	fp = fopen("sphereratbez.input","r");
	input_ratbez4d(&ori,fp);
	fclose(fp);
	printf("\nOrientation curve.\n");
	print_ratbez(&ori);

	/*****************************************************/
	/* 		   scale curve 			     */
	/*****************************************************/

	/* m scales of curve */
	fp = fopen("scale.input","r");
	fscanf(fp,"%i",&scalem);
	if (scalem != m) 
	   printf("Number of scales does not match number of curves.\n");
	for (i=0;i<m;i++) {
		fscanf(fp,"%lf",scale+i);
	}
	fclose(fp);
	
	sweep_wid = initialize_3d_window("Rational interpolating swept surface",
					 25,625,200);
        winset(sweep_wid);
	getorigin(&org[X],&org[Y]);	/* center of window */
	getsize(&size[X],&size[Y]);	/* size of window */
	mdev[X] = MOUSEX;
	mdev[Y] = MOUSEY;
	xmin_data = pos[0][0]; ymin_data = pos[0][1]; zmin_data = pos[0][2];
	for (i=1;i<m;i++) {
		if (pos[i][0] < xmin_data)
			xmin_data = pos[i][0];
		else if (pos[i][0] > xmax_data)
			xmax_data = pos[i][0];
		if (pos[i][1] < ymin_data)
			ymin_data = pos[i][1];
		else if (pos[i][1] > ymax_data)
			ymax_data = pos[i][1];
		if (pos[i][2] < zmin_data)
			zmin_data = pos[i][2];
		else if (pos[i][2] > zmax_data)
			zmax_data = pos[i][2];
	}
	wincen[0] = (xmin_data+xmax_data)/2; 
	wincen[1] = (ymin_data+ymax_data)/2; 
	wincen[2] = (zmin_data+zmax_data)/2;
	transx = -wincen[0]; transy = -wincen[1]; transz = -wincen[2];
	rotx = 0; roty = 0; rotz = 0;
	initrotx = rotx; initroty = roty; initrotz = rotz;
	
        exitflag=FALSE;
        while (exitflag == FALSE) {
		viz_sweep(display_sweepcurve,sweepcurve_num,
			  m,pos,&ori,scale);
                while ((exitflag == FALSE) && (qtest() || !attached)) {
               	   switch (dev = qread(&val)) {
		   	case LEFTMOUSE:
				if (rot_bool==2)
					leftmouse_down = val;
				else if (rot_bool==0)
					/* start rotating */
					rot_bool = 1;
				else if (val) {  /* start interactive positioning */
					rot_bool = 2; 
					leftmouse_down = val;
				}
				break;
			case MIDDLEMOUSE:
				middlemouse_down = val;
				break;
			case RIGHTMOUSE:
				rightmouse_down = val;
				break;
			case MOUSEX:
				mval[X]=val-org[X];
				if (leftmouse_down && !middlemouse_down)
				   /* POSITION */
				   transx = -wincen[0] - (xmax_data-xmin_data)
		+ (float)((xmax_data-xmin_data)*((float) 2*mval[X]/size[X]));
					/* vary from -(xmax-xmin) to (xmax-xmin) */
					/* since 2*mval[X]/size[X] varies */
					/* from -1 to 1 */
				else if (middlemouse_down) {
				   /* ORIENTATION */
				   if (!leftmouse_down)
					/* Z */
					rotz=initrotz-(1800*2*mval[X])/size[X];
					/* from -1800 to 1800 */
					/* 2*mval[X]/size[X] puts value */
					/* into range (-1,1) */
				   else
					/* Y */
			 		roty=initroty-(1800*2*mval[X])/size[X];
				}
				else if (rightmouse_down)
				   /* ZOOM */
				   zoom=(float) (2*mval[X])/size[X] + 1.;
						/* from 0 to 2 */
				break;
			case MOUSEY:
				mval[Y]=val-org[Y];
				if (leftmouse_down)
				   /* POSITION */
				   transy = -wincen[1] - (ymax_data-ymin_data)
		+(float)((ymax_data-ymin_data)*((float) 2*mval[Y]/size[Y]));
				else if (middlemouse_down && !leftmouse_down)
				   /* ORIENTATION */
				   /* X */
				   rotx=initrotx+(1800*2*mval[Y])/size[Y];
				break;
			case ESCKEY:
			    if (val==0)
                                exitflag = TRUE;
				break;
                        case WINFREEZE:
			case WINTHAW:
			case REDRAWICONIC:
                                frontbuffer(TRUE);
                                pushmatrix();
                                reshapeviewport();
				viz_sweep(display_sweepcurve,sweepcurve_num,
					  m,pos,&ori, scale);
				popmatrix();
                                frontbuffer(FALSE);
				break;
                        case REDRAW:
                                reshapeviewport();
				break;
                        case INPUTCHANGE:
                                attached = val;
				break;
		   } /* switch */
                } /* while */
        } /* while */
}

void viz_sweep    (double	display_sweepcurve[3][MAXDISPLAYPTS],
		   const int	sweepcurve_num,
		   const int	m,
		   const V3d	pos[MAXINST],
		   const ratbez_4d *ori,
		   const double scale[MAXINST])
{
	/* draw sweep curve, position curve, and swept surface */

        extern int xmax,ymax,zmax;
	extern int rotx,roty,rotz;
	extern int rot_bool;
	extern double winmin,winmax;
	extern float transx,transy,transz;
	extern float zoom;
	double zoommin,zoommax;
	extern int SCALING;
	Qion keyq[MAXINST];	/* orientations of keyframes (given curves) */
	int n=7; /* number of intermediate frames between keyframes */
	int frame;
        Qion   q;               /* orientation of this frame */
        V3d    pt;              /* position of this frame */
	double s;		/* scale of this frame */
	int i,j;
	double v[3];
	int DRAWTP,drawINT;

	czclear(0xFFFF55,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 50.0);
        pushmatrix();
	zoommin = -25*zoom;
	zoommax = 25*zoom;
        ortho(zoommin*xmax/ymax,zoommax*xmax/ymax,zoommin,zoommax,zoommin,zoommax);
	translate(transx,transy,transz);
	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');

	/* sweep curve */
	RGBcolor(255,0,0);
	draw_curve_in_3d (display_sweepcurve, sweepcurve_num);

	/* draw keyframes */
	linewidth(5);
	RGBcolor(0,0,0);
	for (i=0;i<m;i++) {
		keyq[i][0] = 1;
		keyq[i][1] = ori->x1[6*i];	/* keyframes at joints of ori curve */
		keyq[i][2] = ori->x2[6*i];
		keyq[i][3] = ori->x3[6*i];
		keyq[i][4] = ori->x4[6*i];
		draw_oriented_scaled_curve (display_sweepcurve,
			sweepcurve_num,keyq[i],scale[i],pos[i]);
	}
	linewidth(1);

	if (rot_bool == 1) {
		rotx += 20;
		roty += 20;
		rotz += 20;
	}
        popmatrix();
        swapbuffers();
}
