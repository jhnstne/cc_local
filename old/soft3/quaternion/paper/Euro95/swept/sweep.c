/*
        File: sweep.c
        Author: J.K. Johnstone
        Last Modified: April 4, 1995
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

#include "/ca/jj/cbin/vec.h"

/***********************************************/

#ifndef REAL
#define REAL float
#endif REAL

/* #include "/ca/jj/cbin/datasize.h" */
#ifndef MAXDATAPTS
#define MAXDATAPTS	20
#endif
#define MAXSEG		MAXDATAPTS	/* max segments in B-spline or Bezier */
			/* used to be 20 */
#define MAXDEGREE	16	/* max degree of B-spline or Bezier */
			/* used to be 20 */
#define	MAXKNOTS	MAXSEG + 2*(MAXDEGREE-1)  
	/* start/end multiple knots for Bspl */
#define MAXCTRLPTS	MAXSEG * MAXDEGREE + 1	/* max # of control points on a Bezier */
#define DENSITY		20	/* used to be 5 */		
#define BSPL_DENSITY	DENSITY		/* # points per B-spline segment */
#define BEZ_DENSITY	DENSITY		/* # points per Bezier segment */
#define MAXDISPLAYPTS	(BSPL_DENSITY * MAXSEG)
#define MAXSPLINEPTS	MAXCTRLPTS

#include "/ca/jj/cbin/bez.h"

/***********************************************/

#include "bezsurfWithNormals.h"
#include "/ca/jj/cbin/matrix.h"
#include "/ca/jj/cbin/fit.h"
#include "/ca/jj/cbin/display.h"
#include "/ca/jj/cbin/lights.h"
#include "/ca/jj/cbin/materials.h"
#include "/ca/jj/cbin/misc.h"
#include "/ca/jj/cbin/quaternion.h"
#include "/ca/jj/cbin/drw.h"
#include "/ca/jj/cbin/oricurve.h"
#include "sweep.h"

#include "/ca/jj/cbin/bez.c"
#include "/ca/jj/cbin/bezsurfWithNormals.c"
#include "/ca/jj/cbin/matrix.c"
#include "/ca/jj/cbin/fit.c"
#include "/ca/jj/cbin/display.c"
#include "/ca/jj/cbin/quaternion.c"
#include "/ca/jj/cbin/drw.c"
#include "/ca/jj/cbin/vec.c"
#include "/ca/jj/cbin/misc.c"
#include "/ca/jj/cbin/oricurve.c"

static char *RoutineName;
static void usage()
{
  printf("Usage is %s\n", RoutineName);
  printf("\t[-k]  - keyframes \n");
  printf("\t[-s]  - swept surface \n");
  printf("\t[-i]  - intermediate frames\n");
  printf("\t[-m]  - control mesh\n");
  printf("\t[-S]  - (S)mall window (for video)\n");
  printf("\t[-d]  - debug\n");
}

int     xmax,ymax;      /* screen dimensions */
int     zmax;           /* z-buffer size */
long	org[2],size[2];	/* window characteristics */
REAL	winmin,winmax;	/* desired window dimensions */
int 	rotx,roty,rotz,initrotx,initroty,initrotz;
float	transx,transy,transz;
float	zoom;
int	rot_bool=0;	/* start rotating? */
long    keys_wid,sweep_wid,iso_wid;	/* windows */
int 	SCALING;	/* scaling included iff 1, otherwise 0 */

int	SURF=0;		/* draw tensor product surface? */
int	KEYFRAMES=1;	/* draw input keyframes? */
int 	INTERMEDIATE=0;	/* draw intermediate isoparametric curves? */
int	MESH=0;		/* draw control mesh? */
int	SMALL=0;	/* draw in small window? */
int	DEBUG=0;	/* output debugging information? */

main(int argc, char *argv[])
{
	extern int	SURF,KEYFRAMES,INTERMEDIATE,MESH,DEBUG;
	int		ArgsParsed=0;
	FILE 		*fp;
	ratbez_3d	sweepcurve; /* rational Bezier curve sweeping out the surface */
				    /* with first control point at the origin */
	REAL 		display_sweepcurve[3][MAXDISPLAYPTS];
	int		sweepcurve_num;

	unsigned int	num_frames;    /* # of curve instances to be interpolated */
	V3d		pos[MAXINST];	/* keyframe positions */
	bspl_3d		posBspl;	/* position curve */
	bez_3d		posBez;
	REAL		displayPosBez[3][MAXDISPLAYPTS];
	int		posBezNum;
	bez_3d		poscurve_de;	/* degree-elevated position curve */
					/* (to degree 12) */
	bez_3d		poscurve4,poscurve5,poscurve6,poscurve7,
			poscurve8,poscurve9,poscurve10,poscurve11,
			poscurve12,poscurve13,poscurve14;
	Qion 		q[MAXINST];   /* keyframe orientations */
	ratbez_4d	ori;	/* sextic rational Bezier orientation curve */
	int 		scalenum;
	REAL		scale[MAXINST];	/* keyframe scales */
	bspl_1d		scaleBspl;
	bez_1d		scaleBez;
	extern 	int	SCALING;
	tp_ratbez	surf;		/* tensor product rational Bezier */
					/* swept surface */
	display_surf 	display_tpsurf;
	
	int 		n;	/* # of columns */
	int		col;
	REAL		foo;
	REAL		w[MAXDEGREE];	/* weight */
	REAL		Mklb[MAXSPLINEPTS][MAXDEGREE][3];
				/* \sum_{ijk}^{6+6} M_{ij} b^s_n */
	REAL 		M[3][3];		/* matrix */
	REAL 		b[3],Mb[3];
	REAL		sum[MAXSPLINEPTS][MAXDEGREE][3];
	bez_1d		wbez,wplus1bez,wplus2bez,wplus3bez;

        Boolean         exitflag;       /* window */
        short           attached = 0;
        short           val,mval[2],lastval[2];
        Device          dev,mdev[2];
	int		leftmouse_down=0;
	int		middlemouse_down=0;
	int		rightmouse_down=0;
	REAL		wincen[3];
	REAL		xmin_data,xmax_data,ymin_data,ymax_data,
				zmin_data,zmax_data; /* min/max input coords */
	extern		long org[2],size[2];
	extern 		REAL winmin,winmax;
	extern 		int rotx,roty,rotz,initrotx,initroty,initrotz;
	extern		float transx,transy,transz;
	extern 		float zoom;
        int             h,i,j,k,l,m,s;

        RoutineName = argv[ArgsParsed++];
        for (; ArgsParsed<argc; ArgsParsed++)
                if ('-' == argv[ArgsParsed][0])
                   switch (argv[ArgsParsed][1]) {
                     case 'k':
			KEYFRAMES=1;
                        break;
		     case 's':
			SURF=1;
			KEYFRAMES=0;
			break;
		     case 'i':
			INTERMEDIATE=1;
			KEYFRAMES=0;
			break;
		     case 'm':
			MESH=1;
			KEYFRAMES=0;
			break;
		     case 'S':
			SMALL=1;
			break;
		     case 'd':
			DEBUG=1;
			break;
                     case 'h':
                     default:
                        usage(); exit(-1);
                   }

	/*****************************************************/
	/* 		   sweep curve	 		     */
	/*****************************************************/

	printf("Input sweep curve ...\n");

	fp = fopen("curve.input","r");
 	input_ratbez3d(&sweepcurve,fp);  
	fclose(fp);
	prepare_draw_ratbez_in_3d(&sweepcurve,display_sweepcurve,&sweepcurve_num);

	/*****************************************************/
	/* 		   input keyframes 		     */
	/*****************************************************/

	printf("Input keyframes ...\n");

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
	for (i=0; i<=poscurve_de.L; i++) 
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
			w[j] += ((((REAL) choose(6,k))
				 *((REAL) choose(6,l))
				 /((REAL) choose(12,j)))
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
				foo = ((REAL) choose(6,k))
					 *((REAL) choose(6,l))
					 /((REAL) choose(12,j));
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
			w[j] += ((((REAL) choose(6,k))
				 *((REAL) choose(6,l))
				 /((REAL) choose(12,j)))
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
				foo = ((REAL) choose(6,k))
					 *((REAL) choose(6,l))
					 /((REAL) choose(12,j));
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
			   foo = ((REAL) choose(12,h))
				*((REAL) choose(3,m))
				/((REAL) choose(15,j));
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
	prepare_draw_surf_tpratbez (&surf,&display_tpsurf);

	if (KEYFRAMES) {
	   keys_wid = initialize_3d_window ("Input",50,425,300);
/*	   if (SMALL)
		sweep_wid = initialize_3d_window("Input",100,500,625);
	   else 
		sweep_wid = initialize_3d_window("Input",25,425,200); */
	}
	if (SURF || MESH) {
	   sweep_wid = initialize_3d_window("Swept surface",440,815,300);
/*	   if (SMALL)
		sweep_wid = initialize_3d_window("Swept surface",100,500,625);
	   else
		sweep_wid = initialize_3d_window("Swept surface",440,840,200); */
/*		sweep_wid = initialize_3d_window("Swept surface",0,625,200); */
/*		sweep_wid = initialize_3d_window("Swept surface",425,800,200); */
	}
	if (INTERMEDIATE) {
	   iso_wid = initialize_3d_window("Isoparametric curves",830,1205,300);
/*	   if (SMALL)
		sweep_wid = initialize_3d_window("Isoparametric curves",100,500,625);
	   else 
		sweep_wid = initialize_3d_window("Isoparametric curves",855,1255,200); */
/*		sweep_wid = initialize_3d_window("Isoparametric curves",675,1300,200); */
/*		sweep_wid = initialize_3d_window("Isoparametric curves",825,1200,200); */
	}

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
	transx = -wincen[0]; transy = -wincen[1]; transz = -wincen[2];
	/*	rotx = 0; roty = 0; rotz = 0; */
	/* initrotx = rotx; initroty = roty; initrotz = rotz; */
	rotx=initrotx; roty=initroty; rotz=initrotz;
	
        exitflag=FALSE;
        while (exitflag == FALSE) {
		if (KEYFRAMES)
		   viz_keys(display_sweepcurve,sweepcurve_num,
			  num_frames,pos,&poscurve_de,
			  displayPosBez,posBezNum,
			  &ori, scale, &scaleBez,
			  &display_tpsurf,&surf);
		if (SURF || MESH) 
		   viz_sweep(display_sweepcurve,sweepcurve_num,
			  num_frames,pos,&poscurve_de,
			  displayPosBez,posBezNum,
			  &ori, scale, &scaleBez,
			  &display_tpsurf,&surf);
		if (INTERMEDIATE) 
		   viz_iso(display_sweepcurve,sweepcurve_num,
			  num_frames,pos,&poscurve_de,
			  displayPosBez,posBezNum,
			  &ori, scale, &scaleBez,
			  &display_tpsurf,&surf);
		
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
				if (KEYFRAMES)
	   			   viz_keys(display_sweepcurve,sweepcurve_num,
					  num_frames,pos,&poscurve_de,
					  displayPosBez,posBezNum,
					  &ori, scale, &scaleBez,
					  &display_tpsurf,&surf);
				if (SURF || MESH) 
				   viz_sweep(display_sweepcurve,sweepcurve_num,
					  num_frames,pos,&poscurve_de,
					  displayPosBez,posBezNum,
					  &ori, scale, &scaleBez,
					  &display_tpsurf,&surf);
				if (INTERMEDIATE) 
				   viz_iso(display_sweepcurve,sweepcurve_num,
					  num_frames,pos,&poscurve_de,
					  displayPosBez,posBezNum,
					  &ori, scale, &scaleBez,
					  &display_tpsurf,&surf);
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

void viz_sweep    (REAL	display_sweepcurve[3][MAXDISPLAYPTS],
		   const int	sweepcurve_num,
		   const int	m,
		   const V3d	pos[MAXINST],
		   const bez_3d *poscurve_de,
		   REAL	displayPosBez[3][MAXDISPLAYPTS],
                   const int	posBezNum,
		   const ratbez_4d *ori,
		   const REAL scale[MAXINST],
		   const bez_1d	*scaleBez,
		   display_surf	*display_tpsurf,
		   tp_ratbez 	*surf)
{
	/* draw sweep curve, position curve, and swept surface */

        extern int xmax,ymax,zmax;
	extern int rotx,roty,rotz;
	extern int rot_bool;
	extern REAL winmin,winmax;
	extern float transx,transy,transz;
	extern float zoom;
	REAL zoommin,zoommax;
	extern int SCALING;
	Qion keyq[MAXINST];	/* orientations of keyframes (given curves) */
	int n=7; /* number of intermediate frames between keyframes */
	int frame;
        Qion   q;               /* orientation of this frame */
        V3d    pt;              /* position of this frame */
	REAL s;		/* scale of this frame */
        REAL delta_q;         /* increment in t between frames for quaternions */
        REAL t_q;             /* parameter value for this frame for quaternion */
	int i,j;
	REAL v[3];
	REAL u,uval;
	bez_3d sca;			/* 3d nonparametric scale curve */
	int sca_num;
	REAL display_sca[3][MAXDISPLAYPTS];

	winset(sweep_wid);
	czclear(0xFFFFFF,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 1000.0);
        pushmatrix();
	zoommin = -14*zoom;
	zoommax = 14*zoom;
        ortho(zoommin*xmax/ymax,zoommax*xmax/ymax,zoommin,zoommax,zoommin,zoommax);
	translate(transx,transy,transz);
	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');

	lmbind(MATERIAL,2); 	/* yellow plastic */
	if (SURF)
		draw_surf_tpratbez (display_tpsurf);

	if (MESH) {
		RGBcolor(0,0,255); 	/* blue */
		draw_control_mesh_tpratbez(surf);
	}

	/* sweep curve */
/*	RGBcolor(255,0,0);  */
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

/* animation version */
/*	frame++; 
*	if (frame > (n-1)) 
*		frame = 0;
*/

	if (rot_bool == 1) {
		rotx += 10;
/*		roty += 20;
		rotz += 20; */
	} 
	printf("(rotx,roty,rotz)=(%i,%i,%i)\n",rotx,roty,rotz);
	printf("(transx,transy)=(%f,%f)\n",transx,transy);

        popmatrix();
        swapbuffers();
}

void viz_keys    (REAL	display_sweepcurve[3][MAXDISPLAYPTS],
		   const int	sweepcurve_num,
		   const int	m,
		   const V3d	pos[MAXINST],
		   const bez_3d *poscurve_de,
		   REAL	displayPosBez[3][MAXDISPLAYPTS],
                   const int	posBezNum,
		   const ratbez_4d *ori,
		   const REAL scale[MAXINST],
		   const bez_1d	*scaleBez,
		   display_surf	*display_tpsurf,
		   tp_ratbez 	*surf)
{
	/* draw sweep curve, position curve, and swept surface */

        extern int xmax,ymax,zmax;
	extern int rotx,roty,rotz;
	extern int rot_bool;
	extern REAL winmin,winmax;
	extern float transx,transy,transz;
	extern float zoom;
	REAL zoommin,zoommax;
	extern int SCALING;
	Qion keyq[MAXINST];	/* orientations of keyframes (given curves) */
	int n=7; /* number of intermediate frames between keyframes */
	int frame;
        Qion   q;               /* orientation of this frame */
        V3d    pt;              /* position of this frame */
	REAL s;		/* scale of this frame */
        REAL delta_q;         /* increment in t between frames for quaternions */
        REAL t_q;             /* parameter value for this frame for quaternion */
	int i,j;
	REAL v[3];
	REAL u,uval;
	bez_3d sca;			/* 3d nonparametric scale curve */
	int sca_num;
	REAL display_sca[3][MAXDISPLAYPTS];

	winset(keys_wid);
	czclear(0xFFFFFF,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 1000.0);
        pushmatrix();
	zoommin = -14*zoom;
	zoommax = 14*zoom;
        ortho(zoommin*xmax/ymax,zoommax*xmax/ymax,zoommin,zoommax,zoommin,zoommax);
	translate(transx,transy,transz);
	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');

	lmbind(MATERIAL,2); 	/* yellow plastic */

	   linewidth(5);
/*	   RGBcolor(255,0,0); */
	   RGBcolor(0,0,0);
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

/* animation version */
/*	frame++; 
*	if (frame > (n-1)) 
*		frame = 0;
*/

	if (rot_bool == 1) {
		rotx += 10;
/*		roty += 20;
		rotz += 20; */
	} 

        popmatrix();
        swapbuffers();
}

void viz_iso    (REAL	display_sweepcurve[3][MAXDISPLAYPTS],
		   const int	sweepcurve_num,
		   const int	m,
		   const V3d	pos[MAXINST],
		   const bez_3d *poscurve_de,
		   REAL	displayPosBez[3][MAXDISPLAYPTS],
                   const int	posBezNum,
		   const ratbez_4d *ori,
		   const REAL scale[MAXINST],
		   const bez_1d	*scaleBez,
		   display_surf	*display_tpsurf,
		   tp_ratbez 	*surf)
{
	/* draw sweep curve, position curve, and swept surface */

        extern int xmax,ymax,zmax;
	extern int rotx,roty,rotz;
	extern int rot_bool;
	extern REAL winmin,winmax;
	extern float transx,transy,transz;
	extern float zoom;
	REAL zoommin,zoommax;
	extern int SCALING;
	Qion keyq[MAXINST];	/* orientations of keyframes (given curves) */
	int n=7; /* number of intermediate frames between keyframes */
	int frame;
        Qion   q;               /* orientation of this frame */
        V3d    pt;              /* position of this frame */
	REAL s;		/* scale of this frame */
        REAL delta_q;         /* increment in t between frames for quaternions */
        REAL t_q;             /* parameter value for this frame for quaternion */
	int i,j;
	REAL v[3];
	REAL u,uval;
	bez_3d sca;			/* 3d nonparametric scale curve */
	int sca_num;
	REAL display_sca[3][MAXDISPLAYPTS];

	winset(iso_wid);
	czclear(0xFFFFFF,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 1000.0);
        pushmatrix();
	zoommin = -14*zoom;
	zoommax = 14*zoom;
        ortho(zoommin*xmax/ymax,zoommax*xmax/ymax,zoommin,zoommax,zoommin,zoommax);
	translate(transx,transy,transz);
	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');

	lmbind(MATERIAL,2); 	/* yellow plastic */

	   linewidth(5);
/*	   RGBcolor(255,0,0); */
	   RGBcolor(0,0,0);
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

	/* draw intermediate frames */
	   RGBcolor(0,0,255);
 	   for (i=0;i<m-1;i++) {

		/* generating intermediate frames between keyframe i and i+1,
			associated with knots i and i+1 on the Bezier curves */
		/* instead of frame from 0 to n-1, do from 1 to n-2
		   so that keyframes are not covered */
		for (frame=1;frame<n-1;frame++) {
			/* quaternion */
			delta_q = (ori->knots[i+1] - ori->knots[i]) / (n-1);
			t_q = ori->knots[i] + frame*delta_q;
			point_on_ratbez_4dh (ori, t_q, q);
		
			/* position */
			point_on_bez_3d (poscurve_de, t_q, pt);
	
/*			printf("Drawing object at position (%.2f,%.2f,%.2f) and quaternion (%.2f,%.2f,%.2f,%.2f,%.2f)\n",
*				pt[0],pt[1],pt[2],
*				q[0],q[1],q[2],q[3],q[4]);
*			printf("frame = %i \t t_q = %.2f \t t_p = %.2f \n",frame,t_q,t_p);
*/
			/* scale */
			if (SCALING==1)
				point_on_bez_1d (scaleBez, t_q, &s);
			if (SCALING==0)
				draw_oriented_curve (display_sweepcurve,
					sweepcurve_num,q,pt);
			else
				draw_oriented_scaled_curve (display_sweepcurve,
					sweepcurve_num,q,s,pt);
		}
	   }
/***********************************************************/

/* animation version */
/*	frame++; 
*	if (frame > (n-1)) 
*		frame = 0;
*/

	if (rot_bool == 1) {
		rotx += 10;
/*		roty += 20;
		rotz += 20; */
	} 
        popmatrix();
        swapbuffers();
}

void inputPosOriScale(unsigned int *n, V3d pos[], Qion q[], REAL scale[])
{
	FILE *fp;
	extern int SCALING;
	char s[20];
	int i,scalenum;
	REAL theta,axis[3];
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
                fscanf(fp,"%f %f %f",pos[i],pos[i]+1,pos[i]+2);
		fscanf(fp,"%f",&theta); /* angle of rotation, in degrees */
		fscanf(fp,"%f %f %f",axis,axis+1,axis+2); /* axis of rotation */
		angleaxis2qion(theta,axis,q[i]);
		if (SCALING==1)
			fscanf(fp,"%f",scale+i);
	}
	fclose(fp);
}


void defineMkl(REAL M[3][3], 
	       ratbez_4d *ori, const int s, const int i, const int j)
{
	/* Shoemake has the diagonal elements backward in his SIGGRAPH */
	/* paper because in Graphics Gems II, p. 352, Shoemake says */
	/* that rotation is via q.v.q^{-1}, not q^{-1}.v.q as in SIGGRAPH */
	/* Seidel and Hoschek/Lasser also use q.v.q^{-1}

	/* define the matrix M_{ij} */
	REAL	qi1,qi2,qi3,qi4,qi5,qj1,qj2,qj3,qj4,qj5;

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


