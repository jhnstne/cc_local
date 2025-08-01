/*
  File:    	 s3spline.c
  Author:  	 J.K. Johnstone
  Created: 	 circa 23 August 1996
  Last modified: 6 July 1999
  Purpose: 	 Input:  n points on the sphere S^3.
  	   	 Output: a single rational interpolating Bezier spline.
		 	 No derivative interpolation or subcurves in this version.
  Method: 	 Perturb input away from pole, 
  		 map points to image space,
		 interpolate 
		 and map curve back.
  Reference: 	 John K. Johnstone and James T. Williams (1998)
		 `Rational quaternion splines as Euclidean curves'
  History: 4/14/97: Perturbed input points to avoid pole.
	   4/15/97: Used rotation matrix to rotate, rather than incorrect
		    quaternion multiplication.
	   4/16/97: Added pan.
	  	    Different perturbation routine using minimum eigenvector
		    of covariance matrix.
	   4/18/97: Distinguish between visualization and `true' computation.
	   4/21/97: Enforce minimal sampling rate of quaternions,
		    to avoid no perturbation with (0,0,1), (0,1,0), (1,0,0)
		    input.  This is overkill, since only this set exhibits
		    the bad behaviour of yielding no perturbation,
		    even if larger sampling distances are chosen;
		    but it is a reasonable assumption and a sufficient
		    condition to avoid trouble.
		    Moreover, it avoids the problem of choosing the
		    wrong quaternion from the quaternion pair:
		    this will create antipodally opposite quaternion,
		    which will be flagged by the sampling criterion.
	   4/23/97: Changed imagept from V5r to V4r.
		    Added random input option, for better testing.
	   4/24/97: Fit prescribed tangent at beginning of S3 curve.
	   8/5/97:  Added spherical projection version of image curve generation.
	   8/6/97:  Spit random input out to file (random.dat) for future use.
 		    Added quality analysis (net tangential acceleration).
 	   8/12/97: Added hyperplane display.
	   12/4/98: Remove all subcurves and derivative interpolation.
	   12/11/98: Don't define global knot vector on surface, 
	   	    using Riemannian metric.  Increases covariant acceleration
		    by a couple of orders of magnitude.
	   6/28/99: No strict need for sampling rate criterion anymore,
	   	    since we are not using the best-fitting plane method
		    of rotation away from the pole;
		    however, we keep it around since natural quaternion
		    data sets do not huge leaps between quaternions.
	   7/2/99:  Add rotation to canonical frame before rotation of empty
	   	    point to pole, for coordinate-frame invariance.
*/

#include <GL/glut.h>
#include <GL/glu.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "Bez4d.h"
#include "AllColor.h"
#include "Fit4d.h"
#include "Fit3sphere.h"
#include "Geom.h"
#include "Matrix.h"
#include "Matrix4d.h"
#include "Misc.h"
#include "Quality.h"
#include "REAL.h"
#include "Vec.h"
#include "Vec4d.h"

#define MAXANGLEDIFF 	 PI/9 	/* max ang dist between consecutive data points */
				/* formerly PI/2, then PI/3 */
#define PTSPERBEZSEGMENT 40	/* # of points to draw on each Bezier segment */
				/* previously 20 */
#define NQUALITYPROBES 	 5000	/* # of points to discretely sample in TangAccel_Spherical */
#define TOOCLOSEDIST	 PI/6   /* angular dist from pole that is considered */
				/* too close, affecting curve quality */

static char *RoutineName;
static void usage()
{
  printf("Usage is %s\n", RoutineName);
  printf("\t[-d]   - (d)on't visualize (model in true 4D)\n");
  printf("\t[-O]   - (o)ptimal perturbation (default is random)\n");
  printf("\t[-c]   - choose (c)losest inverse point (default)\n");
  printf("\t[-o]   - choose inverse points (o)n S3 \n");
  printf("\t[-D]   - choose (d)efining points as inverse points\n");
  printf("\t[-R #] - (R)andom input of this size\n\n");
  printf("\t[-P #] - # of probes for covariant acceleration computation (default=5000)\n");
  printf("\t[-b #] - # of display points per Bezier segment (default 40)\n");
  printf("\t[-m #] - maximum angular difference between consecutive quaternions, in degrees (default: 20 degrees)\n");
  printf("\t[-a]   - (a)ngle-axis input of quaternion\n");
  printf("\t[-p]   - no (p)erturbation of the input\n");
  printf("\t[-r]   - (r)andom input of random # of pts (between 3 and 100)\n");
  printf("\t[-g]   - debug output\n");
  printf("\t[-t]   - timing mode\n");
  printf("\t< InputData\n");
}

static GLfloat   transx, transy, transz, rotx, roty, rotz, zoom;
static GLfloat   initrotx, initroty, initrotz;
static GLboolean rotate		    =0;	/* start rotating? */
static GLboolean PAN		    =0;	/* rotate object back and forth for 3d effect? */
static int 	 panLeft	    =0; /* control panning for 3d effect */
static int 	 panRight	    =1;
static GLboolean leftMouseDown	    =0;
static GLboolean middleMouseDown    =0;
static int	 firstx		    =1;	/* first MOUSEX reading? */
static int 	 firsty		    =1;
static int	 oldx,oldy;		/* previous value of MOUSEX and MOUSEY */

static GLboolean DEBUG		    =0;	/* print debug statements? */
static GLboolean INPUTANGLEAXIS	    =0;
static GLboolean VISUALIZE	    =1;	/* display? */
static GLboolean RANDOMINPUT	    =0;	/* generate input randomly? */
static GLboolean NPTKNOWN	    =0;	/* # of random points given on command line? */
static GLboolean PERTURB	    =1;	/* perturb the input to avoid pole? */
static GLboolean RANDOMPERTURB	    =1; /* randomly perturb to avoid pole? */
static GLboolean IMAGEONS3	    =0;	/* use S3 point on inverse image line? */
static GLboolean CLOSESTIMAGE	    =1; /* use closest point on inverse line? */
static GLboolean TIMING		    =0;	/* calculating execution time? (so turn off covariant accel and trim curve computation) */

static GLboolean DRAWDATAPTS	    =1;
static GLboolean DRAWINVERSELINES   =0;
static GLboolean DRAWORDEROFPTS     =0; /* annotate data point by its number? */
static GLboolean DRAWPERTURBPTS	    =0;
static GLboolean DRAWIMAGEPTS	    =0;
static GLboolean DRAWIMAGELINES     =0;	/* draw lines from origin to defining pt of image? */
static GLboolean DRAWIMAGECURVE     =0;
static GLboolean DRAWIMAGECTLPOLY   =0;
static GLboolean DRAWPERTURBS3CURVE =0;
static GLboolean DRAWS3CURVE	    =1;
static GLboolean DRAWS3CTLPOLY      =0;

static GLboolean DRAWIMAGEPTSSP	    =0;
static GLboolean DRAWIMAGECURVESP   =0;
static GLboolean DRAWPERTURBS3CURVESP=0;
static GLboolean DRAWPERTURBS3CTLPOLY=0;
static GLboolean DRAWS3CURVESP      =0;

static GLboolean DRAWPOLE	    =0;	/* draw pole (1,0,0,0)? */
static GLboolean DRAWFIGURE	    =0;	/* draw figure of stereog proj map (for paper)? */
static GLboolean DRAWHYPERPLANE	    =0;	/* draw hyperplane of stereog proj? */

int       n;			/* # of data points */
V4r      *pt;			/* data points */
V4r 	 *perturbpt;		/* perturbed data points */
V4r      *imagept;		/* images of data points under M^{-1} */
Bez4d     imageBez;		/* cubic image curve under M^{-1} */
V4r      *imageptSP;		/* images of data points under stereog proj */
Bez4d     imageBezSP;		/* cubic image curve under stereog proj */
RatBez4d  perturbS3curve;	/* perturbed sextic curve on S3 (M map) */
RatBez4d  perturbS3curveSP;	/* perturbed sextic curve on S3 (stereo proj map) */
RatBez4d  S3curve;		/* sextic curve on S3 (M map) */
RatBez4d  S3curveSP;		/* sextic curve on S3 (stereo proj map) */
V4r      *imageCurveDisplay;
V4r      *perturbS3curveDisplay;
V4r      *S3curveDisplay;
V4r      *imageCurveDisplaySP;
V4r      *perturbS3curveDisplaySP;
V4r      *S3curveDisplaySP;
REAL  	  RestoringRot[4][4];
REAL	  covAccel=0, covAccelSP=0;
int	  ptsPerBezSegment=PTSPERBEZSEGMENT;
V4r       Pole = {1,0,0,0};
REAL 	  maxAngleDiff = MAXANGLEDIFF;
REAL      identity4x4[4][4] = { {1., 0., 0., 0.},
				{0., 1., 0., 0.},
				{0., 0., 1., 0.},
				{0., 0., 0., 1.}};
REAL      yToxmatrix[4][4]  = { {0., 1., 0., 0.},
				{1., 0., 0., 0.},
				{0., 0., 1., 0.},
				{0., 0., 0., 1.}};
V4r  e1 = {1,0,0,0};
V4r  e2 = {0,1,0,0};
V4r  e3 = {0,0,1,0};
V4r  e4 = {0,0,0,1};
V4f  me1 = {-1,0,0,0};
V4r  me3 = {0,0,-1,0};
V3r  e1_3d = {1,0,0};

void gfxinit()
{
  glClearColor (1.0, 1.0, 1.0, 1.0);
  glShadeModel (GL_FLAT);
  glEnable    (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable    (GL_POINT_SMOOTH); 
  glHint      (GL_POINT_SMOOTH_HINT, GL_FASTEST);
  glEnable    (GL_LINE_SMOOTH);
  glHint      (GL_LINE_SMOOTH_HINT, GL_NICEST);
  glPointSize (10.0);
  glLineStipple (1, 0xAAAA);
  transx = 0.0;  transy = 0.0;  transz = 0.0;
  rotx = initrotx = 90.0;
  roty = initroty = 0.0;
  rotz = initrotz = 0.0;	/* 73 for #2 visible control polygon; previously 40 */
  zoom = 1.5;
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -6.0, 6.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void Rotate ()
{
  rotz += 2.0; 
  if (rotz > 360.0) rotz -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void Pan ()
{
  if (panLeft)
   {
    rotz += 0.1;
    if (rotz > 360.0) rotz -= 360.0;
    panLeft++;
    if (panLeft==200) { panLeft=0; panRight=1; }
   }
  else if (panRight)
   {
    rotz -= 0.1;
    if (rotz < 0.0) rotz += 360.0;
    panRight++;
    if (panRight==200) { panRight=0; panLeft=1; }
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) {
    if (rotate || PAN) glutIdleFunc (NULL);
  }
  else if (rotate)     glutIdleFunc (Rotate);
  else if (PAN)	       glutIdleFunc (Pan);
}

/******************************************************************************/
/******************************************************************************/

void mouse (int button, int state, int x, int y)
{
  switch (button) {
  case GLUT_LEFT_BUTTON:
	switch (state) {
	case GLUT_DOWN: 
 	  leftMouseDown = firstx = firsty = 1;
	  glutSetCursor (GLUT_CURSOR_UP_DOWN);
	  break;
	case GLUT_UP: 
	  leftMouseDown = 0;
	  glutSetCursor (GLUT_CURSOR_INHERIT);
	  break;
	default: break;
	}
	break;
  case GLUT_MIDDLE_BUTTON:
	switch (state) {
	case GLUT_DOWN:
	  middleMouseDown = firstx = firsty = 1; 
	  glutSetCursor (GLUT_CURSOR_CYCLE);
	  break;
	case GLUT_UP: 		
	  middleMouseDown = 0; 
	  glutSetCursor (GLUT_CURSOR_INHERIT);
	  break;
	default: break;
	}
	break;
  default:
	break;
  }
}

/******************************************************************************/
/******************************************************************************/

void motion (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoom -= (float).02*(x-oldx);
    if (zoom < 0.) zoom = 0.;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transx += .01*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transy += .01*(y-oldy); /* TRANSLATION: Y */
   }
  else if (middleMouseDown) 
   {
    if (firstx)  firstx=0;
    else { roty += .5*(x-oldx); if (roty > 360.0) roty -= 360.0; } /* ORI: Y */

    if (firsty)  firsty=0;
    else { rotx += .5*(y-oldy); if (rotx > 360.0) rotx -= 360.0; } /* ORI: X */
   }
  oldx = x;  
  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1);	break;		/* ESCAPE */
  case 'r':	if (rotate) { rotate=0; glutIdleFunc (NULL); }  /* toggle rotation */
		else 	    { rotate=1; PAN=0; glutIdleFunc (Rotate); } break;
  case 'p':	if (PAN) { PAN=0; glutIdleFunc (NULL); }	/* toggle pan */
  		else 	 { PAN=1; rotate=0; glutIdleFunc (Pan); } break;
  default:	break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menu (int value)
{
  switch (value) {
  case 1:	DRAWDATAPTS = !DRAWDATAPTS;    			break;
  case 2: 	DRAWINVERSELINES = !DRAWINVERSELINES;		break;
  case 3: 	DRAWPERTURBPTS = !DRAWPERTURBPTS; 		break;
  case 4:	DRAWIMAGEPTS = !DRAWIMAGEPTS;  			break;
  case 5:	DRAWIMAGECURVE = !DRAWIMAGECURVE;		break;
  case 6:	DRAWIMAGECTLPOLY = !DRAWIMAGECTLPOLY;		break;
  case 7:	DRAWPERTURBS3CURVE = !DRAWPERTURBS3CURVE;    	break;
  case 8:	DRAWPERTURBS3CTLPOLY = !DRAWPERTURBS3CTLPOLY;	break;
  case 9:	DRAWS3CURVE = !DRAWS3CURVE; 			break;
  case 10:	DRAWS3CTLPOLY = !DRAWS3CTLPOLY;			break;
  case 11:	DRAWIMAGEPTSSP = !DRAWIMAGEPTSSP;		break;
  case 12:	DRAWIMAGECURVESP = !DRAWIMAGECURVESP;		break;
  case 13:	DRAWPERTURBS3CURVESP = !DRAWPERTURBS3CURVESP;   break;
  case 14:	DRAWHYPERPLANE = !DRAWHYPERPLANE;		break;
  case 15:	DRAWIMAGELINES = !DRAWIMAGELINES;		break;
  case 16:	DRAWPOLE = !DRAWPOLE;				break;
  case 17:	DRAWFIGURE = !DRAWFIGURE;			break;
  case 18: 	DRAWORDEROFPTS = !DRAWORDEROFPTS;		break;
  case 19:      DRAWS3CURVESP = !DRAWS3CURVESP;			break;
  default: 	break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void display ()
{
  int i,j;

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transx, transy, transz);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  glScalef  (zoom, zoom, zoom);

  glLineWidth (1.0);
  glColor3fv (Grey);
  glutWireSphere (1.0, 40, 40);
  
  if (DRAWDATAPTS)
   {
    glColor3fv (Blue);
    glBegin (GL_POINTS);	/* project to x3=0 in original space */
    for (i=0; i<n; i++)  glVertex3f (pt[i][0], pt[i][1], pt[i][3]);
    glEnd();
   }
  if (DRAWINVERSELINES) 
   {
    glColor3fv (Red);
    glBegin (GL_LINES);
    for (i=0; i<n; i++)
     {
      glVertex3f (-10.*imagept[i][0], -10.*imagept[i][2], -10.*imagept[i][3]);
      glVertex3f (10.*imagept[i][0], 10.*imagept[i][2], 10.*imagept[i][3]);
     }
    glEnd();
   }
  if (DRAWPERTURBPTS) 
   {
    glColor3fv (Green);
    glBegin (GL_POINTS);
    for (i=0; i<n; i++)  
      glVertex3f (perturbpt[i][0], perturbpt[i][1], perturbpt[i][3]);  
    glEnd();
   }
  if (DRAWIMAGEPTS) 
   {
    glColor3fv (Red);
    glBegin (GL_POINTS);
    for (i=0; i<n; i++)
      glVertex3f (imagept[i][0], imagept[i][2], imagept[i][3]);
    glEnd();
    glBegin (GL_LINES);
    for (i=0; i<n; i++)
     {
      glVertex3f (0.,0.,0.);
      glVertex3f (imagept[i][0], imagept[i][2], imagept[i][3]);
     }
    glEnd();
   }
  if (DRAWIMAGEPTSSP)
   {
    glColor3fv (Black);
    for (i=0; i<n; i++)
     {
      glPushMatrix();
      glTranslatef (imageptSP[i][0], imageptSP[i][1], imageptSP[i][3]); 
      glutWireCube (.02);
      glPopMatrix();
     }
   }
  if (DRAWIMAGECURVE) 
   {
    glColor3fv (Red);
    glLineWidth (2.0);
    glBegin (GL_LINE_STRIP);
      for (i=0; i<imageBez.L * ptsPerBezSegment + 1; i++)
	glVertex3f (imageCurveDisplay[i][0],
		    imageCurveDisplay[i][2],
		    imageCurveDisplay[i][3]);
    glEnd();
   }
  if (DRAWIMAGECTLPOLY) 
   {
    glColor3fv (Red);
    glLineWidth (2.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<=imageBez.d * imageBez.L; i++)
      glVertex3f (imageBez.x1[i], imageBez.x3[i], imageBez.x4[i]);
    glEnd();
   }
  if (DRAWIMAGECURVESP) 
   {
    glColor3fv (Black);
    glLineWidth (2.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<imageBezSP.L * ptsPerBezSegment + 1; i++)
      glVertex3f (imageCurveDisplaySP[i][0],
		  imageCurveDisplaySP[i][1],		/* ????? */
		  imageCurveDisplaySP[i][3]);
    glEnd();
   }
  if (DRAWS3CURVE) 
   {
    glColor3fv (Blue);
    glLineWidth (4.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<S3curve.L * ptsPerBezSegment + 1; i++)
      glVertex3f (S3curveDisplay[i][0],
		  S3curveDisplay[i][1],
		  S3curveDisplay[i][3]);
    glEnd();
   }
  if (DRAWPERTURBS3CURVE) 
   {
    glColor3fv (Green);
    glLineWidth (4.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<perturbS3curve.L * ptsPerBezSegment + 1; i++)
      glVertex3f (perturbS3curveDisplay[i][0],
		  perturbS3curveDisplay[i][1],
		  perturbS3curveDisplay[i][3]);
    glEnd();
   }
  if (DRAWS3CURVESP) 
   {
    glColor3fv (Black);
    glLineWidth (4.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<S3curveSP.L * ptsPerBezSegment + 1; i++)
      glVertex3f (S3curveDisplaySP[i][0],
		  S3curveDisplaySP[i][1],
		  S3curveDisplaySP[i][3]);
    glEnd();
   }
  if (DRAWPERTURBS3CURVESP) 
   {
    glColor3fv (Black);
    glLineWidth (4.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<perturbS3curveSP.L * ptsPerBezSegment + 1; i++)
      glVertex3f (perturbS3curveDisplaySP[i][0],
		  perturbS3curveDisplaySP[i][1],
		  perturbS3curveDisplaySP[i][3]);
    glEnd();
   }
  if (DRAWS3CTLPOLY) 
   {
    glColor3fv (Green);
    glLineWidth (2.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<=S3curve.d * S3curve.L; i++)
      glVertex3f (S3curve.x1[i], S3curve.x2[i], S3curve.x4[i]);
    glEnd();
   }
  if (DRAWPERTURBS3CTLPOLY) 
   {
    glColor3fv (Green);
    glLineWidth (2.0);
    glBegin (GL_LINE_STRIP);
    for (i=0; i<=perturbS3curve.d * perturbS3curve.L; i++)
      glVertex3f (perturbS3curve.x1[i], perturbS3curve.x2[i], 
      		  perturbS3curve.x4[i]);
    glEnd();
   }
  	/***************************/
  if (DRAWORDEROFPTS)
   {
    glColor3fv (Black);
    for (i=0; i<n; i++)
     {
      glRasterPos3f (pt[i][0], pt[i][1], pt[i][3]+.01);
      if (i<9)
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 49+i);   /* `1' = 49 in ASCII */
      else
       {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 48+(i+1)/10);   /* `0' = 48 in ASCII */
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 48+((i+1)%10)); 
       }
     }
   }
  if (DRAWPOLE)
   {
    glColor3fv (Red);
    glBegin (GL_POINTS);
    glVertex3i (1,0,0);
    glEnd();
   }
  if (DRAWIMAGELINES) 
   {
    glColor3fv (Black);
    glEnable (GL_LINE_STIPPLE);
    glBegin  (GL_LINES);
    for (i=0; i<n; i++)
     {
      glVertex3f (1., 0., 0.);
      glVertex3f (imageptSP[i][0], imageptSP[i][1], imageptSP[i][3]); 
     }
    glEnd();
    glDisable (GL_LINE_STIPPLE);
   }
  if (DRAWFIGURE)
   {
    glColor3fv (Black);
    glBegin (GL_LINE_LOOP);
    glVertex3f (1.75, -1.75, 0);
    glVertex3f (1.75, 1.75, 0);
    glVertex3f (-1.75, 1.75, 0);
    glVertex3f (-1.75, -1.75, 0);
    glEnd();
    
    glBegin (GL_POINTS);
    glVertex3f (0,0,1);
    glVertex3f (.4, sqrt (.68), .4);
    glVertex3f (.4/.6, sqrt(.68)/.6, 0);
    glEnd();

    glEnable (GL_LINE_STIPPLE);
    glBegin (GL_LINES);
    for (i=0; i<n; i++)
     {
      glVertex3f (0,0,1);
      glVertex3f (.4/.6, sqrt(.68)/.6, 0);
    }
    glEnd();
    glDisable (GL_LINE_STIPPLE);
   }
  if (DRAWHYPERPLANE)
   {
    glColor3fv (Grey);
/*     glPolygonMode (GL_FRONT_AND_BACK, GL_FILL); */
    glBegin (GL_POLYGON);
    glVertex3f (0, 1.75, -1.75);
    glVertex3f (0, 1.75, 1.75);
    glVertex3f (0, -1.75, 1.75);
    glVertex3f (0, -1.75, -1.75);
    glEnd();
   }

  glPopMatrix();
  glutSwapBuffers ();
}

/******************************************************************************
	Generate random point on S3.
	If VISUALIZE flag is set, generate random point on S3 and on x3=0,
	for visualization.
******************************************************************************/

static void randomPtOnS3 (V4r pt, int VISUALIZE)
{
  int i;
  for (i=0; i<4; i++)  pt[i] = myRand();	/* random number in [-1,1] */
  if (VISUALIZE) pt[2] = 0.;
  Unit4r (pt, pt);
}

/******************************************************************************
	Either randomly generate input (random # of points, random unit vector on S3)
	or read in the input.
	Maintain sampling criterion (consecutive pts at angular distance 
	< maxAngleDiff).
	If in visualization mode, transform to x3=0 hyperplane.
******************************************************************************/

static void Input ()
{
  FILE *fp;
  int i,j;
  float angle, axis[3], unitaxis[3];
  
  if (RANDOMINPUT)
   {
    if (NPTKNOWN)
     { if (n<2) FatalError ("Need at least two input points\n"); }
    else 
     {
      srand(time(0));
      n = rand() % 98 + 3; 	/* generate 3 to 100 points */
      srand(1);
     }

    printf("Input is %i random points.\n", n);
    printf("These points have been output to the file `random.dat'.\n");
    printf("Angular difference between consecutive points ");
    printf("is bounded at %f radians.\n", maxAngleDiff);
    if (VISUALIZE)
      printf("Input projected to x3=0 hyperplane for visualization...\n");

    pt = (V4r *) malloc (n * sizeof(V4r));
    for (i=0; i<n; i++)	   /* generate random pts, at most maxAngleDiff apart */
      do { randomPtOnS3 (pt[i], VISUALIZE); }
      while (SphericalDist4r (pt[i], pt[i==0?0:i-1]) > maxAngleDiff);
    fp = fopen ("random.dat", "w");
    fprintf (fp,"%i\n", n);
    for (i=0; i<n; i++)
      fprintf(fp,"%f %f %f %f\n", pt[i][0], pt[i][1], pt[i][2], pt[i][3]);
    fclose(fp);
   }
  else
   {
    scanf ("%i", &n);
    if (n < 2) FatalError ("Need at least two input points\n");
    pt = (V4r *) malloc (n * sizeof(V4r));
    if (INPUTANGLEAXIS)
      for (i=0; i<n; i++)
       {
        scanf ("%f %f %f %f", &angle, axis, axis+1, axis+2);
        angle = deg2radians(angle);
        pt[i][0] = cos(angle/2);
        Unit (axis, unitaxis);
        SMult (unitaxis, sin(angle/2), axis);
        for (j=0; j<3; j++)	pt[i][j+1] = axis[j];
       }
    else
      for (i=0; i<n; i++)
        scanf ("%f %f %f %f", pt[i], pt[i]+1, pt[i]+2, pt[i]+3);

    if (VISUALIZE)
     {
      for (i=0; i<n; i++)  { pt[i][2] = 0; Unit4r (pt[i], pt[i]); }
      printf("Input projected to x3=0 hyperplane for visualization...\n");
     }

    /* test that input meets sampling criterion: no large gaps */
    for (i=0; i<n-1; i++)
      if (SphericalDist4r (pt[i], pt[i+1]) > maxAngleDiff)
       {
        printf("\n(%f,%f,%f,%f) and (%f,%f,%f,%f) at angular distance of %f degrees.\n",
		  pt[i][0], pt[i][1], pt[i][2], pt[i][3], 
		  pt[i+1][0], pt[i+1][1], pt[i+1][2], pt[i+1][3], 
		  radian2deg (SphericalDist4r (pt[i], pt[i+1])));
        FatalError ("Unacceptable gap between consecutive quaternions.\n");
       }
   }
  if (DEBUG) for (i=0; i<n; i++) printf("pt[%i]: (%f,%f,%f,%f)\n", i, pt[i][0], pt[i][1], pt[i][2], pt[i][3]);
}			  

/******************************************************************************
	Rotate data set so that 1st point rotates to pole (1,0,0,0).
******************************************************************************/

static void rot1 (int n, V4r *pt, V4r *pt1, REAL rotCanon1[4][4])
{
  int i,j;
  
  if (ExactEqual4r (pt[0], e1))
    for (i=0; i<4; i++) for (j=0; j<4; j++) rotCanon1[i][j] = identity4x4[i][j];
  else if (ExactEqual4r (pt[0], e2))
    for (i=0; i<4; i++) for (j=0; j<4; j++) rotCanon1[i][j] = yToxmatrix[i][j];
  else
   {
    Copy4r (rotCanon1[0], pt[0]);
    TripleCrossProduct (pt[0], e1, e2, rotCanon1[1]);
    TripleCrossProduct (pt[0], e1, rotCanon1[1], rotCanon1[2]);
    TripleCrossProduct (pt[0], rotCanon1[1], rotCanon1[2], rotCanon1[3]);
    for (i=1; i<4; i++)  Unit4r (rotCanon1[i], rotCanon1[i]);
   }
  for (i=0; i<n; i++)
    MultMatrix4r (rotCanon1, pt[i], pt1[i]);
  if (DEBUG) for (i=0; i<n; i++) printf("pt1[%i]: (%f,%f,%f,%f)\n",i, pt1[i][0],  pt1[i][1],  pt1[i][2],  pt1[i][3]);
}

/******************************************************************************
	Rotate data set so that 2nd point rotates to {x2=0,x3=0} plane,
	without moving 1st point.
	That is, rotate about x1-axis.
******************************************************************************/

static void rot2 (int n, V4r *pt, V4r *pt2, REAL rotCanon2[4][4])
{
  int i,j;
  V3r v;	/* normalized bottom 3-vector of pt[1] */
  
  Copy4r (rotCanon2[0], e1);		   /* since rotation about x1-axis */
  for (i=1; i<4; i++) rotCanon2[i][0] = 0;
  /* build lower right 3x3 submatrix M: since M * v = vector along (0,0,1) */
  /* M is like rotating v to z-axis in 3d */
  for (i=0; i<3; i++) v[i] = pt[1][i+1];  Unit4r (v,v);

  Copy3r (&(rotCanon2[3][1]), v);	  		/* bottom row of M */
  Cross  (v, e1_3d, &(rotCanon2[2][1]));  		/* middle row of M */
  Cross  (v, &(rotCanon2[2][1]), &(rotCanon2[1][1]));	/* top row of M */
  
  for (i=0; i<n; i++)
    MultMatrix4r (rotCanon2, pt[i], pt2[i]);
  if (DEBUG) for (i=0; i<n; i++) printf("pt2[%i]: (%f,%f,%f,%f)\n",i, pt2[i][0],  pt2[i][1],  pt2[i][2],  pt2[i][3]);
}

/******************************************************************************
	Rotate data so that 3rd point rotates to {x3=0} hyperplane,
	without moving 1st and 2nd points.
	That is, use a rotation matrix
		[1 	0 	0	0
		 0	cos	-sin	0
		 0	sin	cos 	0
		 0	0	0	1]
	Want its product with pt[2] to be (x,x,0,x)
	so want sin * pt[2][1] + cos * pt[2][2] = 0
	or tan = -pt[2][2] / pt[2][1]
	or theta = atan (-pt[2][2] / pt[2][1])
******************************************************************************/

static void rot3 (int n, V4r *pt, V4r *pt3, REAL rotCanon3[4][4])
{
  int i,j;
  REAL theta,c,s;
  
  Copy4r (rotCanon3[0], e1);
  Copy4r (rotCanon3[3], e4);
  theta = atan (-pt[2][2] / pt[2][1]);
  c = cos(theta);  s = sin(theta);
  rotCanon3[1][0] = 0; rotCanon3[1][1] = c; rotCanon3[1][2] = -s; rotCanon3[1][3] = 0;
  rotCanon3[2][0] = 0; rotCanon3[2][1] = s; rotCanon3[2][2] =  c; rotCanon3[2][3] = 0;
  for (i=0; i<n; i++)
    MultMatrix4r (rotCanon3, pt[i], pt3[i]);
  if (DEBUG) for (i=0; i<n; i++) printf("pt3[%i]: (%f,%f,%f,%f)\n",i, pt3[i][0],  pt3[i][1],  pt3[i][2],  pt3[i][3]);
}

/******************************************************************************
	Rotate points into the canonical frame, as follows:
	1) rotate 1st point, p0, to pole (1,0,0,0)
	2) then rotate p1 to {x2=0,x3=0} plane, 
	   by rotating about x1-axis so p0 is unchanged
	3) then rotate p2 to {x3=0} hyperplane,
	   leaving p0 and p1 unchanged.
	This positioning of p0,p1,p2 is unique, without any remaining
	degrees of freedom.  Thus, the same set of data in a different
	coordinate frame will be rotated to the same place at the end
	of this rotation to canonical frame.
	   
	Use this entire rotation to perturb all of the data points.
******************************************************************************/

static void RotateCanonFrame (int n, V4r *pt, V4r *ptCanon, REAL rotCanon[4][4])
{
  int i;
  V4r *pt1, *pt2;
  REAL rotCanon1[4][4], rotCanon2[4][4], rotCanon3[4][4], foo[4][4];
  
  pt1 = (V4r *) malloc (n * sizeof(V4r));
  pt2 = (V4r *) malloc (n * sizeof(V4r));
  rot1 (n, pt,  pt1,     rotCanon1);
  rot2 (n, pt1, pt2, 	 rotCanon2);
  rot3 (n, pt2, ptCanon, rotCanon3);
  MatrixMatrix4r (rotCanon2, rotCanon1, foo);
  MatrixMatrix4r (rotCanon3, foo, rotCanon);
  for (i=0; i<n; i++)
    MultMatrix4r (rotCanon, pt[i], ptCanon[i]);
  free (pt1);  free(pt2);
}

/******************************************************************************
	Compute a rotation matrix M that rotates qNew to the pole (1,0,0,0).	
	To rotate a unit quaternion v to the x-axis (1,0,0,0),
	we use a 4-space rotation matrix:
		| v		|
	M = 	| unit (w1)	|
		| unit (w2)	|
		| unit (w3)	|
	where w1 = v   x   (0,0,1,0)   x   (1,0,0,0)
	      w2 = v   x   (1,0,0,0)   x    w1
	      w3 = v   x   w1  	       x    w2.
	If v lies in x3=0, M is a rotation about the x3-axis
	that maps points in x3=0 to points in x3=0.
	Here we are assuming that v is not +-e1 or +-e3: if it is, a trivial
	rotation matrix can be used.
******************************************************************************/

static void findRotToPole (V4r qNew, REAL M[4][4])
{
  int i,j;
  V4r  qNewImage;
  
  for (i=0; i<4; i++)  for (j=0; j<4; j++)  M[i][j] = 0.0;
  if (Equal4r(qNew, e1))	/* identity matrix */
    M[0][0] = M[1][1] = M[2][2] = M[3][3] = 1.;
  else if (Equal4r (qNew, me1))
    { M[0][0] = M[1][1] = -1.; M[2][2] = M[3][3] = 1.; }
  else if (Equal4r (qNew, e3))
    { M[0][2] = M[1][1] = M[3][3] = 1.; M[2][0] = -1.; }
  else if (Equal4r (qNew, me3))
    { M[2][0] = M[1][1] = M[3][3] = 1.; M[0][2] = -1.; }
  else
   {
    Copy4r (M[0], qNew);
    TripleCrossProduct (qNew, e1,   e3,   M[1]);
    TripleCrossProduct (qNew, e1,   M[1], M[2]);
    TripleCrossProduct (qNew, M[1], M[2], M[3]);
    for (i=1; i<4; i++)  Unit4r (M[i], M[i]);
   }
  	MultMatrix4r (M, qNew, qNewImage);
	if (!Equal4r(qNewImage,Pole))  FatalError ("Source of perturbation not mapped to pole.\n");
}

/******************************************************************************
	Prerotate data into canonical frame.
	Choose a point on S3 randomly.
	Test if it is sufficiently far (at least TOOCLOSEDIST degrees) 
	from all data points.  If not, choose again.
	Rotate this point in an empty region to the pole (1,0,0,0).
	Use the combination of the canonical and random rotation 
	to perturb all of the data points.
	
	If perturb = 0, don't move the data at all (override perturbation).
******************************************************************************/

static void PerturbInputRandom (int perturb, int n, V4r *pt, V4r *perturbpt, 
			  	REAL RestoringRot[4][4])
{
  int i,j;
  V4r p;	    	/* random point to be rotated to the pole */
  V4r pImage;
  int farEnough;    	/* are the data pts all far enough from the random pt? */
  int nRandomProbes=0;	/* # of random probes necessary to find `empty' point */
  REAL M[4][4] = {0.};
  V4r *ptCanon;		/* data points rotated to canonical frame */
  REAL rotCanon[4][4];	/* matrix for rotation to canonical frame */
  REAL PerturbRot[4][4];
  REAL minDist=5.;	/* minimum distance of an input point from pole */
  
  if (!perturb)
   {
    for (i=0; i<n; i++) Copy4r (perturbpt[i], pt[i]);
    for (i=0; i<4; i++) for (j=0; j<4; j++) RestoringRot[i][j] = identity4x4[i][j];
    return;
   }
  ptCanon = (V4r *) malloc (n * sizeof(V4r));
  RotateCanonFrame (n, pt, ptCanon, rotCanon);
  	if (DEBUG)
	 {
	  printf("After rotation to the canonical frame, the data points are:\n");
	  for (i=0; i<n; i++)
	    printf("(%f,%f,%f,%f)\n", ptCanon[i][0], ptCanon[i][1], ptCanon[i][2], ptCanon[i][3]);
	 }
  do
   {
    farEnough=1;
    randomPtOnS3 (p, VISUALIZE);
    nRandomProbes++;
    for (i=0; i<n; i++)
      if (SphericalDist4r (ptCanon[i], p) <= TOOCLOSEDIST) { farEnough=0; break; }
   }
  while (!farEnough);
  printf("%d random probes to find point to be rotated to pole during perturbation.\n",
  		nRandomProbes);
  
  			/* compute rotation matrix */
  findRotToPole (p, M);
  MultMatrix4r (M, p, pImage);
	if (!Equal4r(pImage,Pole))  FatalError ("Source of perturbation not mapped to pole.\n");
 			
			/* map all points under this rotation matrix */
  for (i=0; i<n; i++)  
    MultMatrix4r (M, ptCanon[i], perturbpt[i]);
        if (DEBUG)
	 {
	  printf("After rotation away from pole, data points are:\n");
	  for (i=0; i<n; i++)  printf("%i: (%f,%f,%f,%f)\n", i, perturbpt[i][0], 
				       perturbpt[i][1], perturbpt[i][2], perturbpt[i][3]);
         }
  free (ptCanon);
  /* compute restoration matrix (inverse of a rotation matrix is its transpose) */
  MatrixMatrix4r (M, rotCanon, PerturbRot);
  for (i=0; i<4; i++)
    for (j=0; j<4; j++)
      RestoringRot[i][j] = PerturbRot[j][i];
   	if (DEBUG)
	 {
	  for (i=0; i<n; i++)
	    if (SphericalDist4r (perturbpt[i], Pole) < minDist)
      	      minDist = SphericalDist4r (perturbpt[i], Pole);
	  printf("Closest point to pole after perturbation is at distance %f = %f degrees.\n", 
  			  minDist, radian2deg(minDist)); 
	 }
}		

/******************************************************************************
	Compute the optimal rotation of the point sample from the pole.
	Rotate the minimum eigenvector of sample covariance matrix
	to the pole (1,0,0,0),
	and in general the ith smallest eigenvector to the ith coord axis.
	This defines a unique rotation for the sample.
	
	Rotate the minimum eigenvector of sample covariance matrix
	(or its negation, whichever yields furthest distance of best-fitting
	plane from pole) to the pole (1,0,0,0),
	Using negation makes method coordinate-frame dependent:
	if best-fitting plane is on pole side, different things happen
	than if best-fitting plane is not on pole side.


	Since for visualization we are using input quaternions with q3=0,
	the minimum eigenvector will always be (0,0,1,0),
	so we use the 2nd smallest eigenvector in this code.
	
******************************************************************************/

void OptimalRotFromPole (int n, V4r *pt, REAL M[4][4])
{
  int  i,j;
  V4r  eigval;		/* unsorted eigenvalues of sample covariance matrix */
  V4r  eigvec[4];	/* associated eigenvectors */
  REAL minEig;
  int  minj, temp;
  int  sortEig[4];	/* sortEig[i] = index of ith smallest eigenvalue */
  REAL M2[4][4];	/* rotation w. opposite orientation for min e.v. */
  V4r  normImage;
  V4r  mean = {0.}; 		/* sample mean */
  V4r  Mmean, M2mean;		/* rotation of sample mean */

  eigenCovMatrix4r (pt, n, eigval, eigvec);
  /* selection sort 4 eigenvalues, symbolically */
  for (i=0; i<4; i++) sortEig[i] = i;
  for (i=0; i<3; i++)
   {
    minEig = eigval[sortEig[i]];  minj = i;
    for (j=i+1; j<4; j++)
      if (eigval[sortEig[j]] < minEig) { minEig = eigval[sortEig[j]]; minj = j; }
    temp = sortEig[i];		/* swap */
    sortEig[i] = sortEig[minj];
    sortEig[minj] = temp;
   }
   
printf("Sorted eigenvalues:\n");
for (i=0; i<4; i++) printf("%f ", eigval[sortEig[i]]);  printf("\n");
  
  /* ith smallest eigenvector is ith row of rotation matrix, for i=1,2,3 */
  if (!VISUALIZE)
   {
    for (i=0; i<3; i++)	
      for (j=0; j<4; j++)	
        M[i][j] = eigvec[ sortEig[i] ][j];
    TripleCrossProduct (M[0], M[1], M[2], M[3]);   /* don't use 4th e.vector */
    Unit4r (M[3], M[3]);		         /* since this may not yield det=1 */
   }
  else 
   {	/* in visualization mode, smallest e.v. is degenerate with eigenvalue 0 */
   	/* also want third row of rotation matrix to be the degenerate (0,0,1,0) */
    for (j=0; j<4; j++) M[0][j] = eigvec[ sortEig[1] ][j];
    for (j=0; j<4; j++) M[1][j] = eigvec[ sortEig[2] ][j];
    for (j=0; j<4; j++) M[3][j] = eigvec[ sortEig[3] ][j];
    TripleCrossProduct (M[0], M[1], M[3], M[2]);
    Unit4r (M[2], M[2]);
   }
  /* TEST THAT DETERMINANT IS 1 */
  		MultMatrix4r (M, eigvec[sortEig[VISUALIZE ? 1 : 0]], normImage);
  		if (!Equal4r(normImage,Pole))  FatalError ("Source of perturbation not mapped to pole.\n");

  /* now compute the same rotation matrix, but with smallest eigenvector */
  /* oriented in the opposite way */
  /* NO: DESTROYS COORDINATE-FRAME INDEPENDENCE */
/*  for (j=0; j<4; j++)  { M2[0][j] = -M[0][j]; M2[2][j] = -M[2][j]; }
*  for (j=0; j<4; j++)  { M2[1][j] = M[1][j];  M2[3][j] =  M[3][j]; } */
  /* TEST THAT DETERMINANT IS 1 */
  
  /* which rot is best? one that moves best-fitting plane furthest from (1,0,0,0) */
  /* use sample mean to test: it lies on the best-fitting plane x = k */
  /* so x-coordinate of rotated sample mean is the relevant issue */
  
/*  for (i=0; i<n; i++) for (j=0; j<4; j++) mean[j] += pt[i][j];
*  for (j=0; j<4; j++) mean[j] /= n;	*/	/* compute sample mean */
/*  MultMatrix4r (M, mean, Mmean);
*  MultMatrix4r (M2, mean, M2mean);
* printf("M[0], M2[0]: %f,%f\n", Mmean[0], M2mean[0]);
*  if (M2mean[0] < Mmean[0]) */	/* M2 rotates mean further from pole */
/*    for (i=0; i<4; i++) for (j=0; j<4; j++) M[i][j] = M2[i][j]; */	/* swap */
}

/******************************************************************************
	Rotate the input points `as far away' from pole (1,0,0,0) as
	possible, according to the best-fitting plane heuristic.
        Compute minimum eigenvector of covariance matrix of the input points
	on the quaternion sphere.
        This represents the minor axis of the best-fitting
        ellipsoid, and thus the normal of the best-fitting plane.
	In particular, if we rotate this point, q_new, to the pole
	then the input points are pushed far away from the pole,
	in the least-squares optimal sense.
	
	ROTATE AXES OF BEST-FITTING ELLIPSOID TO COORDINATE AXES,
	WITH MINOR AXIS AT (1,0,0,0).
******************************************************************************/

static void PerturbInputOptimal (GLboolean PERTURB, int n, V4r *pt, 
			  	 V4r *perturbpt, REAL RestoringRot[4][4])
{
  int  i,j;
  V4r  qNew;		/* point to be mapped to pole for perturbation */
  V4r  minusqNew;
  REAL M[4][4];		/* rotation matrix */
  REAL minDist=5.;	/* minimum distance of an input point from pole */

  if (!PERTURB)
   {
    for (i=0; i<n; i++)  Copy4r (perturbpt[i], pt[i]);
    for (i=0; i<4; i++)
      for (j=0; j<4; j++)
        if (i==j) RestoringRot[i][j] = 1; else RestoringRot[i][j] = 0;
    return;
   }
  OptimalRotFromPole (n, pt, M);
			
		/*  if (VISUALIZE)  minNonZeroEigenvectorCovMatrix4r (pt, n, qNew);
		  else		  minEigenvectorCovMatrix4r (pt, n, qNew); */
  
  		/* findRotToPole (qNew, M);
  		   for (i=0; i<4; i++) minusqNew[i] = -qNew[i];
		   findRotToPole (minusqNew, M2); */

printf("Rotation matrix:\n");
for (i=0; i<4; i++) printf("%f %f %f %f\n", M[i][0], M[i][1], M[i][2], M[i][3]);
  				
			/* rotate all points */
  for (i=0; i<n; i++)  MultMatrix4r (M, pt[i], perturbpt[i]);
  for (i=0; i<4; i++)	/* compute M^{-1} = M^t, the restoration matrix */
    for (j=0; j<4; j++)
      RestoringRot[i][j] = M[j][i];
   	if (DEBUG)
	 {
	  for (i=0; i<n; i++)
	    if (SphericalDist4r (perturbpt[i], Pole) < minDist)
      	      minDist = SphericalDist4r (perturbpt[i], Pole);
	  printf("Closest point to pole after perturbation is at distance %f = %f degrees.\n", 
  			  minDist, radian2deg(minDist)); 
	 }
}

/******************************************************************************
	Map input data to Euclidean space.
******************************************************************************/

static void MapInput (int n, V4r *pt, int SPflag, V4r *imagept)
{
  int i,j;
  V4r definingPt;
  V4r origin = {0.,0.,0.,0.};
  
  if (SPflag)
   {
     for (i=0; i<n; i++) SP (pt[i], imagept[i]); // stereographic projection
    return;
   }
  for (i=0; i<n; i++)
   {				/* (p2,p3,p4,1-p1) */
    for (j=0; j<3; j++)  definingPt[j] = pt[i][j+1];
    definingPt[3] = 1 - pt[i][0];
    if      (IMAGEONS3)     Unit4r (definingPt, imagept[i]);
    else if (CLOSESTIMAGE)
     {
      if (i==0) Unit4r (definingPt, imagept[i]);
      else ClosestPtOnLine4r (imagept[i-1], imagept[i], origin, definingPt);
     }
    else Copy4r (imagept[i], definingPt); /* use defining pt */
   }
	if (DEBUG) for (i=0; i<n; i++) printf("imagept[%i]: (%f,%f,%f,%f)\n", i, imagept[i][0], imagept[i][1], imagept[i][2], imagept[i][3]);
}

/******************************************************************************
	Design curve in image space.
******************************************************************************/

static void DesignImageCurve (int n, V4r *imagept, int SPflag, Bez4d *imageBez,
			      V4r **imageCurveDisplay, int ptsPerBezSegment)
{	
  int   i;

  FitCubicBez_4d (n, imagept, imageBez);
  /*  CheckC2Continuity_Bez4d (imageBez); */
  PrepareBezDisplay_4d (imageBez, &(*imageCurveDisplay), ptsPerBezSegment); 
  if (SPflag)
    /* prep SP's image curve for application of M_curve, by shifting left */
    for (i=0; i<=imageBez->d * imageBez->L; i++)
     {
      imageBez->x1[i] = imageBez->x2[i];
      imageBez->x2[i] = imageBez->x3[i];
      imageBez->x3[i] = imageBez->x4[i];
      imageBez->x4[i] = 1;
     }
}

/******************************************************************************
	Map image curve to S3.
******************************************************************************/

static void MapCurve (Bez4d *imageBez, RatBez4d *S3curve, V4r **S3curveDisplay,
		      int ptsPerBezSegment)
{
  M_curve (imageBez, S3curve);
  PrepareRatBezDisplay_4d (S3curve, S3curveDisplay, ptsPerBezSegment);
  CheckC1Continuity_RatBez4d (S3curve);
  /* CheckC2Continuity_RatBez4d (S3curve); */ /* not correct rational check yet */
} 

/******************************************************************************
******************************************************************************/

static void RestorePerturbation (RatBez4d *S3curve, RatBez4d *perturbS3curve, 
			         REAL RestoringRot[4][4])
{			  
  int   i,j;
  V4r   ctrlpt, newctrlpt; 

  S3curve->d = 6;
  S3curve->L = perturbS3curve->L;
  S3curve->knots  = (REAL *) malloc ((S3curve->L + 1) * sizeof(REAL));
  S3curve->x1 = (REAL *) malloc ((S3curve->d * S3curve->L + 1) * sizeof(REAL));
  S3curve->x2 = (REAL *) malloc ((S3curve->d * S3curve->L + 1) * sizeof(REAL));
  S3curve->x3 = (REAL *) malloc ((S3curve->d * S3curve->L + 1) * sizeof(REAL));
  S3curve->x4 = (REAL *) malloc ((S3curve->d * S3curve->L + 1) * sizeof(REAL));
  S3curve->weights = (REAL *) malloc ((S3curve->d * S3curve->L + 1) * sizeof(REAL));

  for (i=0; i<=S3curve->L; i++)
    S3curve->knots[i] = perturbS3curve->knots[i];
  for (i=0; i<=S3curve->d * S3curve->L; i++)
   {
    ctrlpt[0] = perturbS3curve->x1[i];		/* wow! inelegant!! */
    ctrlpt[1] = perturbS3curve->x2[i];
    ctrlpt[2] = perturbS3curve->x3[i];
    ctrlpt[3] = perturbS3curve->x4[i];
    MultMatrix4r (RestoringRot, ctrlpt, newctrlpt);
    S3curve->x1[i] = newctrlpt[0];
    S3curve->x2[i] = newctrlpt[1];
    S3curve->x3[i] = newctrlpt[2];
    S3curve->x4[i] = newctrlpt[3];
    S3curve->weights[i] = perturbS3curve->weights[i];
   }
}

/******************************************************************************/
/******************************************************************************/

int main(int argc, char **argv)
{
  int   ArgsParsed=0;
  int   i,j;
  int   nProbes = NQUALITYPROBES;	/* for covariant accel computation */
  REAL  Rot[3][3];
  V3r   foo, Rotfoo;
  
  RoutineName = argv[ArgsParsed++];
  while (ArgsParsed < argc)
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
       {
	case 'a':	INPUTANGLEAXIS = 1;	break;
	case 'd':	VISUALIZE = 0; 		break;
	case 'r':	RANDOMINPUT = 1;	break;
	case 'R': 	NPTKNOWN = RANDOMINPUT = 1;
			n = atoi(argv[ArgsParsed++]); break;
	case 'p':	PERTURB = 0;		break;
	case 'O':	RANDOMPERTURB=0;	break;
	case 'c': 	CLOSESTIMAGE=1; IMAGEONS3=0; break;
	case 'o':	IMAGEONS3=1; CLOSESTIMAGE=0; break;
	case 'D': 	IMAGEONS3=CLOSESTIMAGE=0;	break;
	case 'P':	nProbes = atoi(argv[ArgsParsed++]); break;
	case 'b':	ptsPerBezSegment = atoi(argv[ArgsParsed++]); break;
	case 'm':	maxAngleDiff = deg2radians(atoi(argv[ArgsParsed++])); break;
	case 'g':	DEBUG=1;  		break;
	case 't':	TIMING=1; VISUALIZE=0;	break;
        case 'h': 
	default: 	usage(); exit(-1);
       }
    else { usage(); exit(-1); }
 
  /************************************************************/

  Input();		/* sets n and pt */
  perturbpt = (V4r *) malloc (n * sizeof(V4r));
  imagept   = (V4r *) malloc (n * sizeof(V4r));
  imageptSP = (V4r *) malloc (n * sizeof(V4r));
  if (RANDOMPERTURB)
       PerturbInputRandom  (PERTURB, n, pt, perturbpt, RestoringRot);
  else PerturbInputOptimal (PERTURB, n, pt, perturbpt, RestoringRot);
   
  		  /* our version */
  MapInput 	   	  (n, perturbpt, 0, imagept);
  DesignImageCurve 	  (n, imagept, 0, &imageBez, &imageCurveDisplay, ptsPerBezSegment);
  MapCurve 	      	  (&imageBez, &perturbS3curve, &perturbS3curveDisplay, ptsPerBezSegment);
  RestorePerturbation 	  (&S3curve, &perturbS3curve, RestoringRot);
  if (VISUALIZE)
    PrepareRatBezDisplay_4d (&S3curve, &S3curveDisplay, ptsPerBezSegment);
  if (!TIMING)
   {
    covAccel = CovariantAccel_Spherical (&S3curve, nProbes);
    printf("Net squared tangential (covariant) acceleration of S3 curve generated by M  = %f\n", covAccel);
/*  printf("100 probes: %f\n",  CovariantAccel_Spherical (&S3curve, 100));
*   printf("1000 probes: %f\n", CovariantAccel_Spherical (&S3curve, 1000));
*   printf("5000 probes: %f\n", CovariantAccel_Spherical (&S3curve, 5000)); */
   }
	  
		  /* trim curve version */
  if (!TIMING)
   {
    MapInput 	   	  (n, perturbpt, 1, imageptSP);
    DesignImageCurve 	  (n, imageptSP, 1, &imageBezSP, &imageCurveDisplaySP, ptsPerBezSegment);
    MapCurve 	   	  (&imageBezSP, &perturbS3curveSP, &perturbS3curveDisplaySP, ptsPerBezSegment);
    RestorePerturbation   (&S3curveSP, &perturbS3curveSP, RestoringRot);
    PrepareRatBezDisplay_4d (&S3curveSP, &S3curveDisplaySP, ptsPerBezSegment);
    covAccelSP = CovariantAccel_Spherical (&S3curveSP, nProbes);
    printf("Net squared tangential (covariant) acceleration of S3 curve generated by SP = %f\n", covAccelSP); 
   }
	  
/*  printf("Data set (for Lie curve):\n");
*   printf("%i\n", n);
*   for (i=0; i<n; i++)  printf("%f %f %f %f\n", pt[i][0], pt[i][1], pt[i][2], pt[i][3]);
*   for (i=0; i<n; i++)  printf("%f ", S3curve.knots[i]);
*   printf("\n"); */
  
    /* rotated data set: rotation of (1,1,1) to x-axis so this is the first row */
/*  printf("Rotated data set: \n");
*   printf("%i\n", n);
*   Rot[0][0] = Rot[0][1] = Rot[0][2] = 1/sqrt(3);
*   Rot[1][0] = 0.;  Rot[1][1] = 1/sqrt(2);  Rot[1][2] = -1/sqrt(2);
*   Rot[2][0] = -2/sqrt(6);  Rot[2][1] = Rot[2][2] = 1/sqrt(6);   */
	/* for (i=0; i<3; i++) for (j=0; j<3; j++) Rot[i][j]=0.;
	     Rot[0][1] = Rot[1][0] = Rot[2][2] = 1.; */
/*  for (i=0; i<n; i++)
*    {
*     foo[0] = pt[i][0]; foo[1] = pt[i][1]; foo[2] = pt[i][3];
*     MultMatrix (Rot, foo, Rotfoo);
*     printf("%f %f %f %f\n", Rotfoo[0], Rotfoo[1], 0., Rotfoo[2]);
*    }  */
		
  /************************************************************/

  if (VISUALIZE)
   {
    glutInitWindowPosition (300,0);
    glutInitWindowSize (400,400);	/* was 500,500 */
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow ("Rational quaternion spline");
    glutDisplayFunc    (display);
    glutKeyboardFunc   (keyboard);
    glutMouseFunc      (mouse);
    glutMotionFunc     (motion);
    glutReshapeFunc    (reshape);
    glutVisibilityFunc (visibility);
    gfxinit();
    glutCreateMenu   (menu);
    glutAddMenuEntry ("Data points", 1);
    glutAddMenuEntry ("Perturbed data points", 3);
    glutAddMenuEntry ("Inverse image lines", 2);
    glutAddMenuEntry ("Image points", 4);
    glutAddMenuEntry ("Image curve", 5);
    glutAddMenuEntry ("Image curve control polygon", 6);
    glutAddMenuEntry ("Perturbed S3 curve", 7);
    glutAddMenuEntry ("Perturbed S3 curve control polygon", 8);
    glutAddMenuEntry ("S3 curve", 9); 
    glutAddMenuEntry ("S3 curve control polygon", 10); 
    glutAddMenuEntry ("Image points under SP", 11); 
    glutAddMenuEntry ("Image curve under SP", 12);
    glutAddMenuEntry ("Perturbed S3 curve under SP", 13);
    glutAddMenuEntry ("S3 curve under SP", 19);
    glutAddMenuEntry ("", 99);
    glutAddMenuEntry ("Order of data points", 18);
    glutAddMenuEntry ("Pole", 16);
    glutAddMenuEntry ("Hyperplane", 14);
    glutAddMenuEntry ("Stereographic projection", 15);
    glutAddMenuEntry ("Figure", 17);
    glutAttachMenu (GLUT_RIGHT_BUTTON);
    glutMainLoop();
   }
  return 0;
}
