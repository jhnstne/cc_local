/*
        File: design.c
        Author: J.K. Johnstone
        Last Modified: Dec. 5, 1994
        Purpose: Design of m keyframe curves for input to swept surface program.
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
#include "design.h"

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
	FILE 		*fp1;
	ratbez_3d	sweepcurve; /* rational Bezier curve sweeping out the surface */
				    /* with first control point at the origin */
	double 		display_sweepcurve[3][MAXDISPLAYPTS];
	int		sweepcurve_num;
	int		present=0;	/* present keyframe being edited */
	int		candidate;	/* candidate for another keyframe to edit */
	int		status=POS; /* which part of the keyframe is being edited */
	keyframe	keys[MAXKEYFRAMES]; /* given keyframe placements of sweeping curve */
	int		stage=1;	/* stage in input sequence */
	int		roll,pitch,yaw; /* Euler angles, in degs */
					/* of current orientation */
	double		axis[3];
	
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

	fp1 = fopen("curve.input","r");
	input_ratbez3d(&sweepcurve,fp1);
	fclose(fp1);
	if (!(sweepcurve.x1[0] == 0 && sweepcurve.x2[0] == 0 && sweepcurve.x3[0] == 0))
		printf("First control point of the sweep curve does not lie at origin, as it must!\n");
	printf("Sweep curve.\n");
	print_ratbez3d(&sweepcurve);
	printf("\n");
	prepare_draw_ratbez_in_3d(&sweepcurve,display_sweepcurve,&sweepcurve_num);

	/*****************************************************/
	/* 		      window	 		     */
	/*****************************************************/

	sweep_wid = initialize_3d_window("Rational interpolating swept surface",
					 25,425,500);
        winset(sweep_wid);
	getorigin(&org[X],&org[Y]);	/* center of window */
	getsize(&size[X],&size[Y]);	/* size of window */
	mdev[X] = MOUSEX;
	mdev[Y] = MOUSEY;
	xmin_data = 0; xmax_data = 18;
	ymin_data = 0; ymax_data = 18;
	zmin_data = 0; zmax_data = 18;
	wincen[0] = (xmin_data + xmax_data)/2;
	wincen[1] = (ymin_data + ymax_data)/2;
	wincen[2] = (zmin_data + zmax_data)/2;
	transx = -wincen[0]; transy = -wincen[1]; transz = -wincen[2];
	rotx = 0; roty = 0; rotz = 0;
	initrotx = rotx; initroty = roty; initrotz = rotz;

	/* initialize swept curve in middle of window */
	for (i=0;i<=2;i++)
		keys[0].pos[i] = 0;
	euler2Qion(10,20,30,keys[0].ori);
		/* previous choice (.7071,...,.7071) wasn't unit */
	keys[0].scale = 3;
        exitflag=FALSE;
	printf("Input Pos: (L: text) (M: skip) (R: mouse)\n");
        while (exitflag == FALSE) {
	   viz_sweep(display_sweepcurve,sweepcurve_num,present,status,keys);
           while ((exitflag == FALSE) && (qtest() || !attached)) {
           	switch (dev = qread(&val)) {
		case LEFTMOUSE:
		   leftmouse_down = val;
		   if (leftmouse_down) {
		   switch(stage) {
			case 1: /* position via text */
				keybd_input(keys+present,POS);
				stage=3; /* on to orientation */
				status=ORI;
				break;
			case 2:
			case 4:
			case 6:
			case 7:
				break;
			case 3: keybd_input(keys+present,ORI);
				stage=5; /* on to scaling */
				status=SCALE;
				break;
			case 5: keybd_input(keys+present,SCALE);
				stage=7; /* on to fly-through */
				status=NONE;
				break;
			case 8: printf("(L: NEXT curve) (M: PREVIOUS curve)\n");
			case 9: /* edit next curve */
				present++;
				/* initialize swept curve in middle of window */
				for (i=0;i<=2;i++)
					keys[present].pos[i] = 0;
				euler2Qion(10,20,30,keys[present].ori);
				keys[present].scale = 3;
				stage=1; 
				status=POS;
				printf("Input Pos: (L: text) (M: skip) (R: mouse)\n");
				break;
		   }
		   }
		   break;
		case MIDDLEMOUSE:
		   middlemouse_down = val;
		   if (middlemouse_down) {
		   switch(stage) {
			case 1: stage=3;  /* skip to orientation */
				status=ORI;
				printf("Pos = (%f,%f,%f).\n",keys[present].pos[0],
					keys[present].pos[1],keys[present].pos[2]);
				printf("Input Ori: (L: text) (M: skip) (R: mouse)\n");
				break;
			case 2: 
			case 4:
			case 6:
				break;
			case 3: stage=5; /* skip to scaling */
				status=SCALE;
				printf("Ori = (%f,%f,%f,%f).\n",
					keys[present].ori[1],keys[present].ori[2],
					keys[present].ori[3],keys[present].ori[4]);
				printf("Input Scale: (L: text) (M: skip) (R: mouse)\n");
				break;
			case 5: stage=7; /* skip to fly-through */
				status=NONE;
				printf("Scale = %f.\n",keys[present].scale);
				printf("Flythrough: (L: pos) (M,M+L: ori) "
					"(R: zoom) (R+M: quit)\n");
				break;
			case 7:
				if (rightmouse_down) {
				   stage=8; /* finished flythrough */
				   printf("(L: NEXT curve) (M: loop to POS)"
					  " (R: quit)\n");
			   	}
				break;
			case 8: /* loop back to POS */
				stage=1;
				status=POS;
				printf("Input Pos: (L: text) (M: skip) (R: mouse)\n");
				break;
			case 9:
				/* loop back to a previous curve */
				printf("Choose previous curve (0 to %i): ",
						present-1);
				scanf("%i",&candidate);
				if (candidate >= 0 && candidate < present) {
					present=candidate;
					stage=1;
					status=POS;
					printf("Input Pos: (L: text) (M: skip) (R: mouse)\n");
				}
				break;
		   }
		   }
		   break;
		case RIGHTMOUSE:
		   rightmouse_down = val;
		   if (rightmouse_down) {
		   switch(stage) {
		   	case 1: stage=2; /* position via mouse */
				printf("Pos control: (L: x+z) (M: y) (R: quit)\n");
			 	break;
			case 2: stage=3; /* finished positioning with mouse */
				status=ORI;
				printf("Pos = (%f,%f,%f).\n",keys[present].pos[0],
					keys[present].pos[1],keys[present].pos[2]);
				printf("Input Ori: (L: text) (M: skip) (R: mouse)\n");
				break;
			case 3: stage=4; /* orient via mouse */
				printf("Ori control: (M: z+x) (M+L: y) (R: quit)\n");
				break;
			case 4: stage=5; /* finished orienting with mouse */
				status=SCALE;
				printf("Ori = (%f,%f,%f,%f).\n",
					keys[present].ori[1],keys[present].ori[2],
					keys[present].ori[3],keys[present].ori[4]);
				printf("Input Scale: (L: text) (M: skip) (R: mouse)\n");
				break;
			case 5: stage=6; /* scale via mouse */
				printf("Scale control: (L: scaling) (R: Quit)\n");
				break;
			case 6: stage=7; /* finished scaling with mouse */
				status=NONE;
				printf("Scale = %f.\n",keys[present].scale);
				printf("Flythrough: (L: pos) (M,M+L: ori) "
					"(R: zoom) (R+M: quit)\n");
				break;
			case 7:
				if (middlemouse_down) {
				   stage=8; /* finished flythrough */
				   printf("(L: NEXT curve) (M: loop to POS)"
					  " (R: quit)\n");
			   	}
				break;
			case 8:
				exitflag=TRUE;
				/* output final keyframes to files */
				fp1=fopen("POS.input","w");
				fprintf(fp1,"NUM ");
				fprintf(fp1,"%i\n",present+1);
				fprintf(fp1,"POS,ORI,SCALE:\n\n");
				for (i=0;i<=present;i++) {
					fprintf(fp1,"%f %f %f\n",keys[i].pos[0],
						keys[i].pos[1],keys[i].pos[2]);
					/* orientations as (angle,axis) pairs */
					qion2axis(keys[i].ori,axis);
					fprintf(fp1,"%f %f %f %f\n",
						qion2angle(keys[i].ori),
						axis[0],axis[1],axis[2]);
					fprintf(fp1,"%f\n\n",keys[i].scale);
				}
				fclose(fp1); 
				break;
			case 9:
				break;
		   }
		   }
		   break;
		case MOUSEX:
		   mval[X]=val-org[X];
		   switch(stage) {
			case 1: 
			case 3:
			case 5:
			case 8:
			case 9:
				break;
			case 2:
				if (leftmouse_down) {
				   /* set x position */
				   keys[present].pos[0] = transl(mval[X],size[X],
						wincen[0],xmin_data,xmax_data);
				}
				break;
			case 4:
				if (middlemouse_down)
				   if (!leftmouse_down) {
				      	/* Z */
				      	yaw = rotamt(mval[X],size[X]);
				      	euler2Qion(roll,pitch,yaw,keys[present].ori);   
				   }
				   else {
					/* Y */
					pitch = rotamt(mval[X],size[X]);
					euler2Qion(roll,pitch,yaw,keys[present].ori);
				   }
				break;
			case 6:
				if (leftmouse_down)  
					/* SCALE */
					keys[present].scale=
					   (double)(5*mval[X])/size[X] + 1.;
						/* from 0 to 5 */
			case 7:
				if (leftmouse_down && !middlemouse_down)
				   /* POSITION */
				   transx = transl(mval[X],size[X],
						wincen[0],xmin_data,xmax_data);
				else if (middlemouse_down) {
				   /* ORIENTATION */
				   if (!leftmouse_down)
					/* Z */
					rotz=initrotz-rotamt(mval[X],size[X]);
				   else
					/* Y */
		 			roty=initroty-rotamt(mval[X],size[X]);
				}
				else if (rightmouse_down)
				   /* ZOOM */
				   zoom=(float) (2*mval[X])/size[X] + 1.;
						/* from 0 to 2 */
				break;
		   }
		   break;
		case MOUSEY:
		   mval[Y]=val-org[Y];
		   switch(stage) {
			case 1:
			case 3:
			case 5:
			case 6: 
			case 8:
			case 9:
				break;
			case 2:
				if (leftmouse_down) 
				   /* set z position */
				   keys[present].pos[2] = transl(mval[Y],size[Y],
						wincen[1],ymin_data,ymax_data);
				else if (middlemouse_down)
				   /* set y position */
				   keys[present].pos[1] = transl(mval[Y],size[Y],
						wincen[1],ymin_data,ymax_data);
				break;
			case 4:
				if (middlemouse_down && !leftmouse_down) {
				   /* X */
				   roll = rotamt(mval[Y],size[Y]);
				   euler2Qion(roll,pitch,yaw,keys[present].ori);
				}
				break;
			case 7:
				if (leftmouse_down)
				   /* POSITION */
				   transy = transl(mval[Y],size[Y],
						wincen[1],ymin_data,ymax_data);
				else if (middlemouse_down && !leftmouse_down)
				   /* ORIENTATION */
				   /* X */
				   rotx=initrotx+rotamt(mval[Y],size[Y]);
				break;
		   }
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
				  present, status, keys);
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
		   int		present,
		   int		status,
		   keyframe	keys[])
{
	/* draw keyframes created so far */

        extern int xmax,ymax,zmax;
	extern int rotx,roty,rotz;
	extern int rot_bool;
	extern double winmin,winmax;
	extern float transx,transy,transz;
	extern float zoom;
	double zoommin,zoommax;
	extern int SCALING;
	Qion keyq[MAXINST];	/* orientations of keyframes (given curves) */
        Qion   q;               /* orientation of this frame */
        V3d    pt;              /* position of this frame */
	double s;		/* scale of this frame */
	int i,j;
	double v[3];

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

	linewidth(5);
	/* PRESENT CURVE, in different colors based on status of editing */
	switch(status) {
		case NONE: RGBcolor(0,0,0); 	break;
		case POS:  RGBcolor(255,0,0);	break;
		case ORI:  RGBcolor(0,255,100);	break;
		case SCALE:RGBcolor(0,0,255);	break;
	}
	draw_keyframe(keys+present,display_sweepcurve,sweepcurve_num);

	/* OTHER CURVES, all in black */
	RGBcolor(0,0,0);
	for (i=0;i<present;i++) {
		draw_keyframe(keys+i,display_sweepcurve,sweepcurve_num);
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

void draw_keyframe(keyframe *key, double display_curve[3][MAXDISPLAYPTS],
			const int curve_num)
{
	draw_oriented_scaled_curve (display_curve,curve_num,
		key->ori,key->scale,key->pos);
/*	printf("Position (%f,%f,%f), ori (%f,%f,%f,%f), scale %f\n",
*		key->pos[0],key->pos[1],key->pos[2],
*		key->ori[1],key->ori[2],key->ori[3],key->ori[4],
*		key->scale);
*/
}

double transl(short mval,long size,double wincen,double min,double max)
{
	/* vary from -(xmax-xmin) to (xmax-xmin) */
	/* since 2*mval[X]/size[X] varies from -1 to 1 */

	return(-wincen-(max-min) + (float)((max-min)*((float) 2*mval/size)));
}

double rotamt(short mval, long size)
{
	/* from -1800 to 1800 */
	/* 2*mval/size puts value */
	/* into range (-1,1) */

	return((1800*2*mval)/size);
}

void keybd_input(keyframe *key, int kc)
{
	/* text input (vs. mouse input) of keyframe component */
	double	angle;
	V3d 	axis;
	switch (kc) {
	   case POS:
		printf("Input (x,y,z) of next keyframe: ");
		fscanf(stdin,"%lf %lf %lf", key->pos, key->pos+1, key->pos+2);
		printf("Successful input of %f,%f,%f.\n",
			key->pos[0],key->pos[1],key->pos[2]);
		break;
	   case ORI:
		printf("Input orientation (angle, axis): ");
		scanf("%lf %lf %lf %lf", &angle, axis, axis+1, axis+2);
		angleaxis2qion (angle,axis,key->ori);
		break;
	   case SCALE:
		printf("Input scale of next keyframe: ");
		scanf("%lf",&key->scale);
		break;
	   case NONE: 
		break;
	}
}
