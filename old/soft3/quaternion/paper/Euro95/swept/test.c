/*
        File: sweep.c
        Author: J.K. Johnstone
        Last Modified: March 21, 1995
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

static char *RoutineName;
static void usage()
{
  printf("Usage is %s\n", RoutineName);
  printf("\t[-k]  - keyframes \n");
  printf("\t[-s]  - swept surface \n");
  printf("\t[-i]  - intermediate frames\n");
}

int     xmax,ymax;     
int     zmax;          
long	org[2],size[2];
REAL	winmin,winmax;	
int 	rotx,roty,rotz,initrotx,initroty,initrotz;
float	transx,transy,transz;
float	zoom;
int	rot_bool=0;	
int 	SCALING;	

int	SURF=0;		
int	KEYFRAMES=1;	
int 	INTERMEDIATE=0;	
int	DEBUG=0;	

main(int argc, char *argv[])
{
	extern int	SURF,KEYFRAMES,INTERMEDIATE,DEBUG;
	FILE 		*fp; 
	ratbez_3d	sweepcurve;
	REAL 		display_sweepcurve[3][MAXDISPLAYPTS];
	int		sweepcurve_num;

	unsigned int	num_frames; 
	V3d		pos[MAXINST];
	bspl_3d		posBspl;	
	bez_3d		posBez;
	REAL		displayPosBez[3][MAXDISPLAYPTS];
	int		posBezNum;
	bez_3d		poscurve_de;	
					
	bez_3d		poscurve4,poscurve5,poscurve6,poscurve7,
			poscurve8,poscurve9,poscurve10,poscurve11,
			poscurve12,poscurve13,poscurve14;
	Qion 		q[MAXINST];   
	ratbez_4d	ori;	
	int 		scalenum;
	REAL		scale[MAXINST];	
	bspl_1d		scaleBspl;
	bez_1d		scaleBez;
	extern 	int	SCALING;
	tp_ratbez	surf;		
	display_surf 	display_tpsurf;
	
	int 		n;	
	int		col;
	REAL		foo;
	REAL		w[MAXDEGREE];	
	REAL		Mklb[MAXSPLINEPTS][MAXDEGREE][3];
	REAL 		M[3][3];		
	REAL 		b[3],Mb[3];
	REAL		sum[MAXSPLINEPTS][MAXDEGREE][3];
	bez_1d		wbez,wplus1bez,wplus2bez,wplus3bez;

        long            sweep_wid;
        Boolean         exitflag;       
        short           attached = 0;
        short           val,mval[2],lastval[2];
        Device          dev,mdev[2];
	int		leftmouse_down=0;
	int		middlemouse_down=0;
	int		rightmouse_down=0;
	REAL		wincen[3];
	REAL		xmin_data,xmax_data,ymin_data,ymax_data,
				zmin_data,zmax_data;
	extern		long org[2],size[2];
	extern 		REAL winmin,winmax;
	extern 		int rotx,roty,rotz,initrotx,initroty,initrotz;
	extern		float transx,transy,transz;
	extern 		float zoom;
        int             h,i,j,k,l,m,s;

/*
	printf("Input sweep curve ...\n");
	fp = fopen("curve.input","r");
 	input_ratbez3d(&sweepcurve,fp);  
	fclose(fp);
	prepare_draw_ratbez_in_3d(&sweepcurve,display_sweepcurve,&sweepcurve_num);

	printf("Input keyframes ...\n");
	inputPosOriScale(&num_frames,pos,q,scale);
*/
	
	fitCubicBspl_3d(num_frames,pos,&posBspl); 
	bspl_to_bezier_3d(&posBspl,&posBez); 
	degree_elevate_bez3d(&posBez,&poscurve4);
	degree_elevate_bez3d(&poscurve4,&poscurve5);
	degree_elevate_bez3d(&poscurve5,&poscurve6);
	degree_elevate_bez3d(&poscurve6,&poscurve7);
	degree_elevate_bez3d(&poscurve7,&poscurve8);
	degree_elevate_bez3d(&poscurve8,&poscurve9);
	degree_elevate_bez3d(&poscurve9,&poscurve10);
	degree_elevate_bez3d(&poscurve10,&poscurve11);
	if (SCALING==0) {
		degree_elevate_bez3d(&poscurve11,&poscurve_de);
	}
	else { 
		degree_elevate_bez3d(&poscurve11,&poscurve12);
		degree_elevate_bez3d(&poscurve12,&poscurve13);
		degree_elevate_bez3d(&poscurve13,&poscurve14);
		degree_elevate_bez3d(&poscurve14,&poscurve_de);
	}
	prepare_draw_bez_in_3d(&poscurve_de,displayPosBez,&posBezNum);

	orientation_curve(q,num_frames,&ori); 
	for (i=0;i<=poscurve_de.L;i++) 
		poscurve_de.knots[i] = ori.knots[i];

	if (SCALING==1) {
		fitCubicBspl_1d    (num_frames,scale,&scaleBspl); 
		bspl_to_bezier_1d  (&scaleBspl,&scaleBez); 
		for (i=0;i<=scaleBez.L;i++) {
			scaleBez.knots[i] = ori.knots[i];
		}
	}

	surf.d_u = sweepcurve.d;
	surf.L_u = sweepcurve.L;
	for (i=0;i<=sweepcurve.L;i++)
		surf.knots_u[i] = sweepcurve.knots[i];
	surf.d_v = poscurve_de.d;
	surf.L_v = poscurve_de.L;
	for (i=0;i<=poscurve_de.L;i++)
		surf.knots_v[i] = poscurve_de.knots[i];

/*
if (SCALING==0) {
	n = sweepcurve.d * sweepcurve.L; 
	for (s=0;s<poscurve_de.L;s++) {	
	   for (j=0;j<=12;j++) { 
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
		for (col=0;col<=n;col++) {
			Mklb[col][j][0] = Mklb[col][j][1] = Mklb[col][j][2] = 0.;
		}
		for (k=0;k<=6;k++) {
		   l=j-k;
	  	   if (l<=6 && l>=0) {
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
		   } 
		} 
		for (col=0; col<=n; col++) {
			surf.x1[col][12*s+j] = poscurve_de.x1[12*s+j] 
					       + Mklb[col][j][0]/w[j];
			surf.x2[col][12*s+j] = poscurve_de.x2[12*s+j]
					       + Mklb[col][j][1]/w[j];
			surf.x3[col][12*s+j] = poscurve_de.x3[12*s+j]
					       + Mklb[col][j][2]/w[j];
			surf.weights[col][12*s+j] = sweepcurve.weights[col]*w[j];
		}
	   }
	} 
}
else { 	
	n = sweepcurve.d * sweepcurve.L;
	for (s=0;s<poscurve_de.L;s++) {
	   for (j=0;j<=12;j++) { 	
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
		for (i=0;i<=n;i++)
			Mklb[i][j][0] = Mklb[i][j][1] = Mklb[i][j][2] = 0.;
		for (k=0;k<=6;k++) {
		   l=j-k;
	  	   if (l<=6 && l>=0) {
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
		   } 
		} 
	   } 

	   wbez.d=12;
	   wbez.L=1;
	   wbez.knots[0]=0; wbez.knots[1]=1;
	   for (h=0; h<=12; h++) 
		wbez.x[h] = w[h];
	   degree_elevate_bez1d(&wbez,&wplus1bez);
	   degree_elevate_bez1d(&wplus1bez,&wplus2bez);
	   degree_elevate_bez1d(&wplus2bez,&wplus3bez);

	   for (i=0;i<=n;i++) { 
		for (j=0; j<=15; j++) {
		   sum[i][j][0] = sum[i][j][1] = sum[i][j][2] = 0;
		   for (h=0; h<=12; h++) {
			m=j-h;
			if (m<=3 && m>=0) {
			   foo = ((REAL) choose(12,h))
				*((REAL) choose(3,m))
				/((REAL) choose(15,j));
			   sum[i][j][0]+=foo*Mklb[i][h][0]*scaleBez.x[3*s+m];
			   sum[i][j][1]+=foo*Mklb[i][h][1]*scaleBez.x[3*s+m];
			   sum[i][j][2]+=foo*Mklb[i][h][2]*scaleBez.x[3*s+m];
			} 
		   }
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
	}
}
*/

	print_diagnostics(&sweepcurve,&poscurve_de,&ori,&scaleBez);
	output_tp_ratbez(&surf);
	prepare_draw_surf_tpratbez (&surf,&display_tpsurf);

	if (KEYFRAMES)
		sweep_wid = initialize_3d_window("Keyframes",25,400,200);
	else if (SURF)
		sweep_wid = initialize_3d_window("Swept surface",425,800,200);
	else if (INTERMEDIATE)
		sweep_wid = initialize_3d_window("Isoparametric curves",825,1200,200);

        winset(sweep_wid);
	getorigin(&org[X],&org[Y]);
	getsize(&size[X],&size[Y]);
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
	rotx=initrotx; roty=initroty; rotz=initrotz;
	
        exitflag=FALSE;
        while (exitflag == FALSE) {
		viz_sweep(display_sweepcurve,sweepcurve_num,
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
					rot_bool = 1;
				else if (val) {  
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
				   transx = -wincen[0] - (xmax_data-xmin_data)
		+ (float)((xmax_data-xmin_data)*((float) 2*mval[X]/size[X]));
				else if (middlemouse_down) {
				   if (!leftmouse_down)
					rotz=initrotz-(1800*2*mval[X])/size[X];
				   else
					/* Y */
			 		roty=initroty-(1800*2*mval[X])/size[X];
				}
				else if (rightmouse_down)
				   zoom=(float) (2*mval[X])/size[X] + 1.;
				break;
			case MOUSEY:
				mval[Y]=val-org[Y];
				if (leftmouse_down)
				   transy = -wincen[1] - (ymax_data-ymin_data)
		+(float)((ymax_data-ymin_data)*((float) 2*mval[Y]/size[Y]));
				else if (middlemouse_down && !leftmouse_down) {
				   rotx=initrotx+(1800*2*mval[Y])/size[Y];
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
		   } 
                } 
        } 
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
        extern int xmax,ymax,zmax;
	extern int rotx,roty,rotz;
	extern int rot_bool;
	extern REAL winmin,winmax;
	extern float transx,transy,transz;
	extern float zoom;
	REAL zoommin,zoommax;
	extern int SCALING;
	Qion keyq[MAXINST];	
	int n=7; 
	int frame;
        Qion   q;
        V3d    pt;
	REAL s;	
        REAL delta_q;
        REAL t_q;
	int i,j;
	REAL v[3];
	REAL u,uval;
	bez_3d sca;
	int sca_num;
	REAL display_sca[3][MAXDISPLAYPTS];

	czclear(0xFFFF55,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 20.0);
        pushmatrix();
	zoommin = -14*zoom;
	zoommax = 14*zoom;
        ortho(zoommin*xmax/ymax,zoommax*xmax/ymax,zoommin,zoommax,zoommin,zoommax);
	translate(transx,transy,transz);
	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');

	if (SURF)
		draw_surf_tpratbez (display_tpsurf);

	linewidth(5);
	RGBcolor(255,0,0);
	for (i=0;i<m;i++) {
		keyq[i][0] = 1;
		keyq[i][1] = ori->x1[6*i];
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

	if (INTERMEDIATE) {
	for (i=0;i<m-1;i++) {

	        RGBcolor(0,0,255);
		for (frame=1;frame<n-1;frame++) {
			delta_q = (ori->knots[i+1] - 
				   ori->knots[i]) / (n-1);
			t_q = ori->knots[i] + frame*delta_q;
	
			point_on_ratbez_4dh (ori, t_q, q);
		
			point_on_bez_3d (poscurve_de, t_q, pt);
	
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

	if (rot_bool == 1) {
		rotx += 20;
		roty += 20;
		rotz += 20;
	}

        popmatrix();
        swapbuffers();
}

/*
void inputPosOriScale(unsigned int *n, V3d pos[], Qion q[], REAL scale[])
{
	FILE *fp;
	extern int SCALING;
	char s[20];
	int i,scalenum;
	REAL theta,axis[3];
	extern int initrotx,initroty,initrotz;
	extern float zoom;

	fp = fopen("POS.input","r");
	fscanf(fp,"%s",s);	
	fscanf(fp,"%f",&zoom);
	fscanf(fp,"%s",s);	
	fscanf(fp,"%i %i %i",&initrotx,&initroty,&initrotz);
	fscanf(fp,"%s",s);	
	if (s[0]=='N') 	SCALING=0; else SCALING=1;
	fscanf(fp,"%s",s);	
	fscanf(fp,"%i",n);
	fscanf(fp,"%s",s);	
        for (i=0;i<*n;i++) {
                fscanf(fp,"%f %f %f",pos[i],pos[i]+1,pos[i]+2);
		fscanf(fp,"%f",&theta);
		fscanf(fp,"%f %f %f",axis,axis+1,axis+2); 
		angleaxis2qion(theta,axis,q[i]);
		if (SCALING==1)
			fscanf(fp,"%f",scale+i);
	}
	fclose(fp);
}
*/

/*
void defineMkl(REAL M[3][3], 
	       ratbez_4d *ori, const int s, const int i, const int j)
{
	REAL	qi1,qi2,qi3,qi4,qi5,qj1,qj2,qj3,qj4,qj5;

	qi5 = ori->weights[6*s+i];
	qj5 = ori->weights[6*s+j];
	qi1 = ori->x1[6*s+i] * qi5; 
	qj1 = ori->x1[6*s+j] * qj5;
	qi2 = ori->x2[6*s+i] * qi5; qj2 = ori->x2[6*s+j] * qj5;
	qi3 = ori->x3[6*s+i] * qi5; qj3 = ori->x3[6*s+j] * qj5;
	qi4 = ori->x4[6*s+i] * qi5; qj4 = ori->x4[6*s+j] * qj5;
	def3x3matrix(M,
		qi5*qj5 - 2*(qi3*qj3 + qi4*qj4),
		2*(qi2*qj3 - qi1*qj4),
		2*(qi2*qj4 + qi1*qj3),
		2*(qi2*qj3 + qi1*qj4),
		qi5*qj5 - 2*(qi2*qj2 + qi4*qj4),
		2*(qi3*qj4 - qi1*qj2),
		2*(qi2*qj4 - qi1*qj3),
		2*(qi3*qj4 + qi1*qj2),
		qi5*qj5 - 2*(qi2*qj2 + qi3*qj3));
}
*/

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


