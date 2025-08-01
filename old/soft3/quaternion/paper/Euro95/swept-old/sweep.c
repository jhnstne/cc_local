/*
        File: sweep.c
        Author: J.K. Johnstone
        Last Modified: Dec. 27, 1994
        Purpose: Creation of tensor product Bezier surface representing 
		 the sweep of a rational Bezier curve 
		 interpolating a finite number of known curve instances
		 (instance = position + orientation + scale).
*/

#include <device.h>
#include <gl/gl.h>
#include <gl/sphere.h>
#include <math.h>
#include <stdio.h>

#include "/usr/people/jj/cbin/vec.h"
#include "/usr/people/jj/cbin/bez.h"
#include "/usr/people/jj/cbin/bezsurf.h"
#include "/usr/people/jj/cbin/fit.h"
#include "/usr/people/jj/cbin/display.h"
#include "/usr/people/jj/cbin/lights.h"
#include "/usr/people/jj/cbin/materials.h"
#include "/usr/people/jj/cbin/misc.h"
#include "/usr/people/jj/cbin/quaternion.h"
#include "/usr/people/jj/cbin/drw.h"
#include "/usr/people/jj/cbin/oricurve.h"
#include "sweep.h"

#include "/usr/people/jj/cbin/bez.c"
#include "/usr/people/jj/cbin/bezsurf.c"
#include "/usr/people/jj/cbin/fit.c"
#include "/usr/people/jj/cbin/display.c"
#include "/usr/people/jj/cbin/quaternion.c"
#include "/usr/people/jj/cbin/drw.c"
#include "/usr/people/jj/cbin/vec.c"
#include "/usr/people/jj/cbin/misc.c"
#include "/usr/people/jj/cbin/oricurve.c"

int     xmax,ymax;      /* screen dimensions */
int     zmax;           /* z-buffer size */
long	org[2],size[2];	/* window characteristics */
double	winmin,winmax;	/* desired window dimensions */
int 	rotx,roty,rotz,initrotx,initroty,initrotz;
float	transx,transy,transz;
float	zoom;
int	rot_bool=0;	/* start rotating? */
int 	SCALING;	/* scaling included iff 1, otherwise 0 */

main()
{
	FILE 		*fp;
	ratbez_3d	sweepcurve; /* rational Bezier curve sweeping out the surface */
				    /* with first control point at the origin */
	double 		display_sweepcurve[3][MAXDISPLAYPTS];
	int		sweepcurve_num;

	unsigned int	num_frames;    /* # of curve instances to be interpolated */
	V3d		pos[MAXINST];	/* keyframe positions */
	bspl_3d		posBspl;	/* position curve */
	bez_3d		posBez;
	double		displayPosBez[3][MAXDISPLAYPTS];
	int		posBezNum;
	bez_3d		poscurve_de;	/* degree-elevated position curve */
					/* (to degree 12) */
	bez_3d		poscurve4,poscurve5,poscurve6,poscurve7,
			poscurve8,poscurve9,poscurve10,poscurve11,
			poscurve12,poscurve13,poscurve14;
	Qion 		q[MAXINST];   /* keyframe orientations */
	ratbez_4d	ori;	/* sextic rational Bezier orientation curve */
	int 		scalenum;
	double		scale[MAXINST];	/* keyframe scales */
	bspl_1d		scaleBspl;
	bez_1d		scaleBez;
	extern 	int	SCALING;
	tp_ratbez	surf;		/* tensor product rational Bezier */
					/* swept surface */
	double 		display_surf[3][MAXISOCURVES][MAXDISPLAYPTS];
	int		surf_pt_num, iso_num;
	
	int 		n;	/* # of columns */
	int		col;
	double		foo;
	double		w[MAXDEGREE];	/* weight */
	double		Mklb[MAXSPLINEPTS][MAXDEGREE][3];
				/* \sum_{ijk}^{6+6} M_{ij} b^s_n */
	double 		M[3][3];		/* matrix */
	double 		b[3],Mb[3];
	double		sum[MAXSPLINEPTS][MAXDEGREE][3];
	bez_1d		wbez,wplus1bez,wplus2bez,wplus3bez;

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
        int             h,i,j,k,l,m,s;

	/*****************************************************/
	/* 		   sweep curve	 		     */
	/*****************************************************/

	fp = fopen("curve.input","r");
	input_ratbez3d(&sweepcurve,fp);
	fclose(fp);
	prepare_draw_ratbez_in_3d(&sweepcurve,display_sweepcurve,&sweepcurve_num);

	/*****************************************************/
	/* 		   input keyframes 		     */
	/*****************************************************/

	inputPosOriScale(&num_frames,pos,q,scale);
	
	/*****************************************************/
	/* 		   position curve 		     */
	/*****************************************************/

	/* fit directrix (position) curve */
	fitCubicBspl_3d(num_frames,pos,&posBspl); 
	/* put in Bezier form for tensor product Bezier representation */
	bspl_to_bezier_3d(&posBspl,&posBez); 
	/* elevate to degree 12 = degree of orientation curve */
	/* Note: make more elegant later, by one big degree elevation */
	degree_elevate_bez3d(&posBez,&poscurve4);
	degree_elevate_bez3d(&poscurve4,&poscurve5);
	degree_elevate_bez3d(&poscurve5,&poscurve6);
	degree_elevate_bez3d(&poscurve6,&poscurve7);
	degree_elevate_bez3d(&poscurve7,&poscurve8);
	degree_elevate_bez3d(&poscurve8,&poscurve9);
	degree_elevate_bez3d(&poscurve9,&poscurve10);
	degree_elevate_bez3d(&poscurve10,&poscurve11);
	if (SCALING==0) { /* stop degree elevation at 12 */
		degree_elevate_bez3d(&poscurve11,&poscurve_de);
	}
	else {  /* keep going to degree 15 */
		degree_elevate_bez3d(&poscurve11,&poscurve12);
		degree_elevate_bez3d(&poscurve12,&poscurve13);
		degree_elevate_bez3d(&poscurve13,&poscurve14);
		degree_elevate_bez3d(&poscurve14,&poscurve_de);
	}
	prepare_draw_bez_in_3d(&poscurve_de,displayPosBez,&posBezNum);

	/*****************************************************/
	/* 		   orientation curve 		     */
	/*****************************************************/

	orientation_curve(q,num_frames,&ori); /* compute orientation curve */
	/* make knot sequences the same */
	for (i=0;i<=poscurve_de.L;i++) 
		poscurve_de.knots[i] = ori.knots[i];

	/*****************************************************/
	/* 		   scale curve 			     */
	/*****************************************************/

	/* NUM_FRAMES scales of curve */
	if (SCALING==1) {
/*		fitQuadraticBez_1d (num_frames,scale,&scaleBez);  */
		fitCubicBspl_1d    (num_frames,scale,&scaleBspl); 
		bspl_to_bezier_1d  (&scaleBspl,&scaleBez); 
		/* make knot sequences the same */
		for (i=0;i<=scaleBez.L;i++) {
			scaleBez.knots[i] = ori.knots[i];
		}
	}

	/*****************************************************/
	/* 		tensor product surface		     */
	/*****************************************************/

	/* old version, where only position, not orientation, changes */
/*	for (i=0;i<=sweepcurve.d * sweepcurve.L;i++) { 
*/		/* ith column of mesh */
/*		for (j=0;j<=posBez.d * posBez.L;j++) {
*			surf.x1[i][j] = sweepcurve.x1[i] + posBez.x1[j];
*			surf.x2[i][j] = sweepcurve.x2[i] + posBez.x2[j];
*			surf.x3[i][j] = sweepcurve.x3[i] + posBez.x3[j];
*			surf.weights[i][j] = 1; */ /* sweepcurve.weights[i]; */
/*		}
*	}
*/

	/* The tensor product surface is generated by sweeping */
	/* the sweep curve along the position curve.  	       */
	/* The control points of the ith column of the mesh are */
	/* the control points of the curve generated by        */
	/* the sweep of the ith control point of the sweep curve, */
	/* or P(t) + O(t)S(t)b_i (see paper) */

	/* row = sweep curve */
	surf.d_u = sweepcurve.d;
	surf.L_u = sweepcurve.L;
	for (i=0;i<=sweepcurve.L;i++) {
		surf.knots_u[i] = sweepcurve.knots[i];
	}
	/* column = position curve */
	surf.d_v = poscurve_de.d;
	surf.L_v = poscurve_de.L;
	for (i=0;i<=poscurve_de.L;i++) {
		surf.knots_v[i] = poscurve_de.knots[i];
	}

if (SCALING==0) { /* just position/orientation changes */
	/* compute control points of column curves */
	n = sweepcurve.d * sweepcurve.L; /* cols 0,...,n */
	for (s=0;s<poscurve_de.L;s++) {	/* sth Bezier seg of deg 12 pos curve */
	   for (j=0;j<=12;j++) { 	/* jth control point of segment */
		/* WEIGHT W_j^{path} */
		/* \sum_{0 \leq k \leq 6, 0 \leq l \leq 6, k+l=j} */
		w[j] = 0;
		for (k=0;k<=6;k++) {
		   l=j-k;
	  	   if (l<=6 && l>=0) {
			w[j] += ((((double) choose(6,k))
				 *((double) choose(6,l))
				 /((double) choose(12,j)))
				* ori.weights[6*s+k]*ori.weights[6*s+l]); 
		   }
		}
		/* (x,y,z) coordinates */
		/* \SUM_{0\LEQ k \LEQ 6,0\LEQ l \LEQ 6,k+l=j} M_{kl} B_{COL}^C*/
		for (col=0;col<=n;col++) {
			Mklb[col][j][0] = Mklb[col][j][1] = Mklb[col][j][2] = 0.;
		}
		for (k=0;k<=6;k++) {
		   l=j-k;
	  	   if (l<=6 && l>=0) {
			/* matrix M_{kl} */
			defineMkl(M,&ori,s,k,l);
			for (col=0;col<=n;col++) {
				b[0] = sweepcurve.x1[col];
				b[1] = sweepcurve.x2[col];
				b[2] = sweepcurve.x3[col];
				matrixXvect(M,b,Mb);
				foo = ((double) choose(6,k))
					 *((double) choose(6,l))
					 /((double) choose(12,j));
				Mklb[col][j][0] += foo * Mb[0];
				Mklb[col][j][1] += foo * Mb[1];
				Mklb[col][j][2] += foo * Mb[2];
			}
		   } /* l */
		} /* k */
		/* can now compute p+_j + \sum_{klj}^{6+6} M_{kl} b_{col}^c */
		/* note: should move this down below `for k' loop when scaling */
		for (col=0; col<=n; col++) {
			surf.x1[col][12*s+j] = poscurve_de.x1[12*s+j] 
					       + Mklb[col][j][0]/w[j];
				/* kth control point on sth segment */
			surf.x2[col][12*s+j] = poscurve_de.x2[12*s+j]
					       + Mklb[col][j][1]/w[j];
			surf.x3[col][12*s+j] = poscurve_de.x3[12*s+j]
					       + Mklb[col][j][2]/w[j];
			surf.weights[col][12*s+j] = sweepcurve.weights[col]*w[j];
			/* in paper notation: w_i^c * w_j^{path} */
		}
	   } /* for j */
	} /* for s */
}
else { 	/* scaling changes included */
	/* compute control points of column curves */
	n = sweepcurve.d * sweepcurve.L; /* cols 0,...,n */
	for (s=0;s<poscurve_de.L;s++) {	/* sth Bezier seg of deg 15 pos curve */
	   for (j=0;j<=12;j++) { 	/* jth control point of segment */
		/* WEIGHT W_J^{path} */
		/* \sum_{0 \leq k \leq 6, 0 \leq l \leq 6, k+l=j} */
		w[j] = 0;
		for (k=0;k<=6;k++) {
		   l=j-k;
	  	   if (l<=6 && l>=0) {
			w[j] += ((((double) choose(6,k))
				 *((double) choose(6,l))
				 /((double) choose(12,j)))
				* ori.weights[6*s+k]*ori.weights[6*s+l]); 
		   }
		}
		/* (x,y,z) coordinates */
		/* \SUM_{0\LEQ k \LEQ 6,0\LEQ l \LEQ 6,k+l=j} M_{kl} B_{COL}^C*/
		for (i=0;i<=n;i++) {
			Mklb[i][j][0] = Mklb[i][j][1] = Mklb[i][j][2] = 0.;
		}
		for (k=0;k<=6;k++) {
		   l=j-k;
	  	   if (l<=6 && l>=0) {
			/* matrix M_{kl} */
			defineMkl(M,&ori,s,k,l);
			for (i=0;i<=n;i++) {
				b[0] = sweepcurve.x1[i];
				b[1] = sweepcurve.x2[i];
				b[2] = sweepcurve.x3[i];
				matrixXvect(M,b,Mb);
				foo = ((double) choose(6,k))
					 *((double) choose(6,l))
					 /((double) choose(12,j));
				Mklb[i][j][0] += foo * Mb[0];
				Mklb[i][j][1] += foo * Mb[1];
				Mklb[i][j][2] += foo * Mb[2];
			}
		   } /* l */
		} /* k */
	   } /* for j */

	   /* create w_j^{(3)} Bezier function */
	   wbez.d=12;
	   wbez.L=1;
	   wbez.knots[0]=0; wbez.knots[1]=1;
	   for (h=0; h<=12; h++) 
		wbez.x[h] = w[h];
	   degree_elevate_bez1d(&wbez,&wplus1bez);
	   degree_elevate_bez1d(&wplus1bez,&wplus2bez);
	   degree_elevate_bez1d(&wplus2bez,&wplus3bez);

	   for (i=0;i<=n;i++) { /* columns */
		for (j=0; j<=15; j++) { /* control points of sth segment */
		   sum[i][j][0] = sum[i][j][1] = sum[i][j][2] = 0;
		   /* \sum_{0<=h<=12,0<=m<=3,h+m=j} (foo * Mklb[h] * scaleBez[m])*/  
		   for (h=0; h<=12; h++) {
			m=j-h;
			if (m<=3 && m>=0) {
			   foo = ((double) choose(12,h))
				*((double) choose(3,m))
				/((double) choose(15,j));
			   sum[i][j][0]+=foo*Mklb[i][h][0]*scaleBez.x[3*s+m];
			   sum[i][j][1]+=foo*Mklb[i][h][1]*scaleBez.x[3*s+m];
			   sum[i][j][2]+=foo*Mklb[i][h][2]*scaleBez.x[3*s+m];
			} /* m */
		   } /* h */

		   /* could pull computation of w2j out of COL loop */
		   /* since it is independent of COL */
/*		   if (j==0) 	w2j = w[0];
*		   else if (j==1) w2j = (2*w[0] + 12*w[1])/14;
*		   else  w2j=(j*(j-1)*w[j-2]+ 2*j*(14-j)*w[j-1]  
*				+ (14-j)*(13-j)*w[j])/(13*14);  
*/

		   surf.x1[i][15*s+j] = poscurve_de.x1[15*s+j]
					  + sum[i][j][0]/wplus3bez.x[j];
		   surf.x2[i][15*s+j] = poscurve_de.x2[15*s+j]
					  + sum[i][j][1]/wplus3bez.x[j];
		   surf.x3[i][15*s+j] = poscurve_de.x3[15*s+j]
					  + sum[i][j][2]/wplus3bez.x[j];
		   surf.weights[i][15*s+j] = sweepcurve.weights[i] * 
					     wplus3bez.x[j];
		}
	   }
	} /* for s */
} /* if (SCALING==0) else */


	/****************************************************************/

	print_diagnostics(&sweepcurve,&poscurve_de,&ori,&scaleBez);
	output_tp_ratbez(&surf);
	prepare_draw_surf_tpratbez (&surf,display_surf,&surf_pt_num,&iso_num);

/*	sweep_wid = initialize_3d_window("Keyframes",25,400,200);  */
/*	sweep_wid = initialize_3d_window("Swept surface",425,800,200);  */
	sweep_wid = initialize_3d_window("Isoparametric curves",825,1200,200); 
	

        winset(sweep_wid);
	getorigin(&org[X],&org[Y]);	/* center of window */
	getsize(&size[X],&size[Y]);	/* size of window */
	mdev[X] = MOUSEX;
	mdev[Y] = MOUSEY;
	xmin_data = pos[0][0]; ymin_data = pos[0][1]; zmin_data = pos[0][2];
	for (i=1;i<num_frames;i++) {
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
	transx = inittransx-wincen[0]; 
	transy = inittransy-wincen[1]; 
	transz = inittransz-wincen[2];
	/*	rotx = 0; roty = 0; rotz = 0; */
	/* initrotx = rotx; initroty = roty; initrotz = rotz; */
	rotx=initrotx; roty=initroty; rotz=initrotz;
	
        exitflag=FALSE;
        while (exitflag == FALSE) {
		viz_sweep(display_sweepcurve,sweepcurve_num,
			  num_frames,pos,&poscurve_de,
			  displayPosBez,posBezNum,
			  &ori, scale, &scaleBez,
			  display_surf,surf_pt_num,iso_num,&surf);
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
				   if (!leftmouse_down) {
					/* Z */
					rotz=initrotz-(1800*2*mval[X])/size[X];
					/* from -1800 to 1800 */
					/* 2*mval[X]/size[X] puts value */
					/* into range (-1,1) */
					printf("rotzoom:(%i,%i,%i,%f)\n",
						rotx,roty,rotz,zoom);
				   }
				   else {
					/* Y */
			 		roty=initroty-(1800*2*mval[X])/size[X];
				   	printf("rotzoom:(%i,%i,%i,%f)\n",
						rotx,roty,rotz,zoom);
				   }
				}
				else if (rightmouse_down) {
				   /* ZOOM */
				   zoom=(float) (2*mval[X])/size[X] + 1.;
						/* from 0 to 2 */
				   printf("rotzoom:(%i,%i,%i,%f)\n",
					rotx,roty,rotz,zoom);
				   printf("zoom:%f\n",zoom);
				}
				break;
			case MOUSEY:
				mval[Y]=val-org[Y];
				if (leftmouse_down)
				   /* POSITION */
				   transy = -wincen[1] - (ymax_data-ymin_data)
		+(float)((ymax_data-ymin_data)*((float) 2*mval[Y]/size[Y]));
				else if (middlemouse_down && !leftmouse_down) {
				   /* ORIENTATION */
				   /* X */
				   rotx=initrotx+(1800*2*mval[Y])/size[Y];
				   printf("rotzoom:(%i,%i,%i,%f)\n",
					rotx,roty,rotz,zoom);
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
					  num_frames,pos,&poscurve_de,
					  displayPosBez,posBezNum,
					  &ori, scale, &scaleBez,
				     display_surf,surf_pt_num,iso_num,&surf);
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
		   const bez_3d *poscurve_de,
		   double	displayPosBez[3][MAXDISPLAYPTS],
                   const int	posBezNum,
		   const ratbez_4d *ori,
		   const double scale[MAXINST],
		   const bez_1d	*scaleBez,
		   double	display_surf[3][MAXISOCURVES][MAXDISPLAYPTS],
		   const int	surf_pt_num,
		   const int	iso_num,
		   tp_ratbez 	*surf)
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
        double delta_q;         /* increment in t between frames for quaternions */
        double t_q;             /* parameter value for this frame for quaternion */
	int i,j;
	double v[3];
	int DRAWTP,drawINT;
	double u,uval;
	bez_3d sca;			/* 3d nonparametric scale curve */
	int sca_num;
	double display_sca[3][MAXDISPLAYPTS];

	czclear(0xFFFF55,zmax);
	/* czclear(0xFFFFFF,zmax); */
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 20.0);
        pushmatrix();
	zoommin = -14*zoom;
	zoommax = 14*zoom;
        ortho(zoommin*xmax/ymax,zoommax*xmax/ymax,zoommin,zoommax,zoommin,zoommax);
	translate(transx,transy,transz);
	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');

	/* sweep curve */
/*	RGBcolor(255,0,0); */
/*	draw_curve_in_3d (display_sweepcurve, sweepcurve_num); */

	/* position curve */
/*	RGBcolor(0,0,255); */
/*	draw_curve_in_3d (displayPosBez, posBezNum); */

	/* scale curve */
/*	RGBcolor(0,0,0);
*	sca.d=scaleBez->d;
*	sca.L=scaleBez->L;
*	for (i=0;i<=sca.L;i++)
*		sca.knots[i] = scaleBez->knots[i];
*	for (i=0;i<=sca.d * sca.L; i++) {
*/		/* p. 71, Farin */
/*		j=i/sca.d;
*		sca.x1[i] = sca.knots[j] + ((i%sca.d)/sca.d) *
*					(sca.knots[j+1]-sca.knots[j]);
*		sca.x2[i] = scaleBez->x[i];
*		sca.x3[i] = 0;
*	}
*	prepare_draw_bez_in_3d (&sca,display_sca,&sca_num);	
*	draw_curve_in_3d (display_sca, sca_num);
*	draw_bez_control_in_3d (&sca);
*/

	/* tensor product surface */
	DRAWTP=0;
	if (DRAWTP==1){
	RGBcolor(0,255,0);
	for (i=0;i<iso_num-1;i++) { 
		/* mesh between each ith and (i+1)th isoparametric curves */
		bgntmesh();  
/*		bgnline();     */
		for (j=0;j<surf_pt_num;j++) {
			/* jth point on ith isoparametric curve */
			v[0] = display_surf[0][i][j];
			v[1] = display_surf[1][i][j];
			v[2] = display_surf[2][i][j];
			v3d(v);
			/* jth point on (i+1)th isoparametric curve */
			v[0] = display_surf[0][i+1][j];
			v[1] = display_surf[1][i+1][j];
			v[2] = display_surf[2][i+1][j];
			v3d(v);
		}
		endtmesh();  
/*		endline();    */
	}
	}

	RGBcolor(0,0,255);
/*	draw_control_mesh_tpratbez(surf);   */

	/*****************************************************************/
	/* 		animation 					 */
	/*****************************************************************/

	/* draw keyframes */
	linewidth(5);
	RGBcolor(255,0,0);
	for (i=0;i<m;i++) {
		keyq[i][0] = 1;
		keyq[i][1] = ori->x1[6*i];	/* keyframes at joints of ori curve */
		keyq[i][2] = ori->x2[6*i];
		keyq[i][3] = ori->x3[6*i];
		keyq[i][4] = ori->x4[6*i];
		if (SCALING==0)
			draw_oriented_curve (display_sweepcurve,sweepcurve_num,
					     keyq[i],pos[i]); 
		else
			draw_oriented_scaled_curve (display_sweepcurve,
					sweepcurve_num,keyq[i],scale[i],pos[i]);
	}
	linewidth(1);

	/* between each pair of keyframes */
	drawINT=1;
	if (drawINT==1) {
	for (i=0;i<m-1;i++) {

		/* generating intermediate frames between keyframe i and i+1,
			associated with knots i and i+1 on the Bezier curves */
		/* draw intermediate frames */	
	        RGBcolor(0,0,255);
		/* instead of frame from 0 to n-1, do from 1 to n-2
		   so that keyframes are not covered */
		for (frame=1;frame<n-1;frame++) {
			/* quaternion */
			delta_q = (ori->knots[i+1] - 
				   ori->knots[i]) / (n-1);
			t_q = ori->knots[i] + frame*delta_q;
	
			point_on_ratbez_4dh (ori, t_q, q);
		
			/* position */
			point_on_bez_3d (poscurve_de, t_q, pt);
	
/*			printf("Drawing object at position (%.2f,%.2f,%.2f) and quaternion (%.2f,%.2f,%.2f,%.2f,%.2f)\n",
*				pt[0],pt[1],pt[2],
*				q[0],q[1],q[2],q[3],q[4]);
*	
*			printf("frame = %i \t t_q = %.2f \t t_p = %.2f \n",frame,t_q,t_p);
*/
			/* scale */
			if (SCALING==1) {
				point_on_bez_1d (scaleBez, t_q, &s);
			}
			if (SCALING==0)
				draw_oriented_curve (display_sweepcurve,
					sweepcurve_num,q,pt);
			else
				draw_oriented_scaled_curve (display_sweepcurve,
					sweepcurve_num,q,s,pt);
		}
	}
	}
/***********************************************************/

/* animation version */
/*	frame++; 
*	if (frame > (n-1)) 
*		frame = 0;
*/

	if (rot_bool == 1) {
		rotx += 20;
		roty += 20;
		rotz += 20;
	}

        popmatrix();
        swapbuffers();
}

void inputPosOriScale(unsigned int *n, V3d pos[], Qion q[], double scale[])
{
	FILE *fp;
	extern int SCALING;
	char s[20];
	int i,scalenum;
	double theta,axis[3];
	extern int initrotx,initroty,initrotz;
	extern float zoom;

	/* n (reference point) positions of curve */
	fp = fopen("POS.input","r");
	fscanf(fp,"%s",s);	/* ZOOM */
	fscanf(fp,"%f",&zoom);
	fscanf(fp,"%s",s);	/* ROTATE */
	fscanf(fp,"%i %i %i",&initrotx,&initroty,&initrotz);
	fscanf(fp,"%s",s);	/* SCALE or NOSCALE */
	if (s[0]=='N') 	SCALING=0; else SCALING=1;
	fscanf(fp,"%s",s);	/* NUM */
	fscanf(fp,"%i",n);
	fscanf(fp,"%s",s);	/* (POS,ORI,SCALE): */
        for (i=0;i<*n;i++) {
                fscanf(fp,"%lf %lf %lf",pos[i],pos[i]+1,pos[i]+2);
		fscanf(fp,"%lf",&theta); /* angle of rotation, in degrees */
		fscanf(fp,"%lf %lf %lf",axis,axis+1,axis+2); /* axis of rotation */
		angleaxis2qion(theta,axis,q[i]);
		if (SCALING==1)
			fscanf(fp,"%lf",scale+i);
	}
	fclose(fp);
}


void defineMkl(double M[3][3], 
	       ratbez_4d *ori, const int s, const int i, const int j)
{
	/* Shoemake has the diagonal elements backward in his SIGGRAPH */
	/* paper because in Graphics Gems II, p. 352, Shoemake says */
	/* that rotation is via q.v.q^{-1}, not q^{-1}.v.q as in SIGGRAPH */
	/* Seidel and Hoschek/Lasser also use q.v.q^{-1}

	/* define the matrix M_{ij} */
	double	qi1,qi2,qi3,qi4,qi5,qj1,qj2,qj3,qj4,qj5;

	qi5 = ori->weights[6*s+i];
	qj5 = ori->weights[6*s+j];
	qi1 = ori->x1[6*s+i] * qi5; /* we want the homogeneous coord q1 */
		   /* control point of ori is (q1/q5,q2/q5,...,q4/q5) */
		   /* and weight is q5 */
	qj1 = ori->x1[6*s+j] * qj5;
	qi2 = ori->x2[6*s+i] * qi5; qj2 = ori->x2[6*s+j] * qj5;
	qi3 = ori->x3[6*s+i] * qi5; qj3 = ori->x3[6*s+j] * qj5;
	qi4 = ori->x4[6*s+i] * qi5; qj4 = ori->x4[6*s+j] * qj5;
	def3x3matrix(M,
		/* first row */
		qi5*qj5 - 2*(qi3*qj3 + qi4*qj4),
		2*(qi2*qj3 - qi1*qj4),
		2*(qi2*qj4 + qi1*qj3),
		/* second row */
		2*(qi2*qj3 + qi1*qj4),
		qi5*qj5 - 2*(qi2*qj2 + qi4*qj4),
		2*(qi3*qj4 - qi1*qj2),
		/* third row */
		2*(qi2*qj4 - qi1*qj3),
		2*(qi3*qj4 + qi1*qj2),
		qi5*qj5 - 2*(qi2*qj2 + qi3*qj3));
}

void print_diagnostics(ratbez_3d *sweepcurve, bez_3d *poscurve_de, 
		  ratbez_4d *ori, bez_1d *scaleBez)
{
	extern int SCALING;
	printf("Sweep curve.\n");
	print_ratbez3d(sweepcurve);
	printf("\n");
	printf("Degree-elevated position curve.\n");
	print_bez_3d (poscurve_de);
	printf("\n");
	printf("\nOrientation curve.\n");
	print_ratbez(ori);
	if (SCALING == 1) {
		printf("\nScaling curve.\n");
		print_bez_1d(scaleBez);
		printf("\n");
	}
}


