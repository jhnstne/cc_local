/*
  File: 	 qspl.cpp
  Author:	 J.K. Johnstone
  Created:	 circa 23 August 1996
  Last Modified: 7 March 2004
  Purpose: 	 Input: n points on the sphere S^3
  	   	 Output: rational interpolating Bezier spline
  	   	 Basically: Perturb input away from pole, map points 
 		 to image space, interpolate and map curve back.
  Reference: John K. Johnstone and James T. Williams (1995)
	`Rational control of orientation for animation'
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

  Test for cusps?
  Compute knot sequence based on spherical chord-length parameterization?
  Split points into smaller sets, connected with C^2 continuity,
		if large point set forces some point too close to pole?
		(or two points on opposite sides of pole?)
  Compute amount of tangential acceleration of our curves.

            3/7/04: Moved qsplsupport routines (just 2 of them) here.
	            Perhaps they should go in the quaternion class instead.
		    Changed pt and imagePt from V4f * to V4fArr.
*/

#include <GL/glut.h>
#include <GL/glu.h>
#include <fstream.h>
#include <iostream.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>

#include "AllColor.h"
#include "Vector.h"
#include "BezierCurve.h"
#include "RatBezierCurve.h"
#include "CurveOnSn.h"

/* #define constant variables */

#define MAXANGLEDIFF 	 M_PI/3		// to enforce sampling criterion

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-a]   - (a)ngle-axis input style for quaternions" << endl;
  cout << "\t[-r]   - (r)andom input of random # of pts (between 3 and 100)" << endl;
  cout << "\t[-R #] - (R)andom input of this size" << endl;
  cout << "\t[-d]   - (d)on't visualize (modelling in honest-to-goodness 4D)" << endl;
  cout << "\t       < InputFile" << endl;
 }

/* GUI */
static GLfloat   transx, transy, transz, rotx, roty, rotz, zoom;
static GLfloat   initrotx, initroty, initrotz;
static int 	 panLeft=0;  			// control panning for 3d effect
static int 	 panRight=1;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1;	// first MOUSEX reading?
static GLboolean firsty=1;
static int	 oldx,oldy;	// previous value of MOUSEX and MOUSEY
static GLboolean ROTATE=0;	// start rotating?
static GLboolean PAN=1;	 	// rotate object back and forth for 3d effect?

/* menu flags */
static GLboolean DRAWDATAPTS = 1;	/* draw data points? */
static GLboolean DRAWIMAGEPTS = 0;	/* draw image points? */
static GLboolean DRAWIMAGEBSPLINE=0;	
static GLboolean DRAWIMAGEBSPLINECTRL=0;
static GLboolean DRAWIMAGEBEZ=0;	
static GLboolean DRAWIMAGEBEZCTRL=0;
static GLboolean DRAWS3BEZ=1;
static GLboolean DRAWS3BEZCTRL=0;
static GLboolean IMAGEONS3=1;	// choose point on inverse image line on S3
static GLboolean INPUTANGLEAXIS=0;
static GLboolean NPTKNOWN=0;
static GLboolean RANDOMINPUT=0;
static GLboolean VISUALIZE=1;
static GLboolean DEBUG = 1;

/* global variables */
int       	  n;			// # of input points
V4fArr      	  pt;			// input points
V4fArr     	  imagept;		// images of input points under M^{-1}
BsplineCurve4f    imagebspl;
BezierCurve4f	  imagebez;
//RatBezierCurve4f     S3bez;
CurveOnS3         S3bez;                // the quaternion spline
// GLUnurbsObj 	 *theNurb;

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  glClearColor (1.0, 1.0, 1.0, 1.0);
  glShadeModel (GL_FLAT);
  glEnable (GL_BLEND);				/* alpha-blending */
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			/* antialiasing */
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glEnable (GL_POINT_SMOOTH); 		/* too slow for rotation */
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (10.0);
  glLineWidth(1.0);
  glLineStipple (1, 0xAAAA);

  transx = 0.0;  transy = 0.0;  transz = 0.0;
  rotx = initrotx = 90.0;
  roty = initroty = 0;
  rotz = initrotz = 0;
  zoom = 1.5;

//  theNurb = gluNewNurbsRenderer();
//  gluNurbsProperty (theNurb, GLU_SAMPLING_TOLERANCE, 25.0);
//  gluNurbsProperty (theNurb, GLU_DISPLAY_MODE, GLU_FILL);
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

void Rotate (void)
{
  rotz += 2.0; 
  if (rotz > 360.0) rotz -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void Pan (void)
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
    if (ROTATE || PAN)  glutIdleFunc (NULL);
  }
  else if (ROTATE)      glutIdleFunc (Rotate);
  else if (PAN)         glutIdleFunc (Pan);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); break;		/* ESCAPE */
  case 'p':	PAN = !PAN;		/* toggle pan */
		if (PAN) glutIdleFunc (Pan); else glutIdleFunc (NULL); break;
  case 'r':	ROTATE = !ROTATE;	/* toggle rotation */
		if (ROTATE) glutIdleFunc (Rotate); else glutIdleFunc (NULL); break;
  default:      break;
  }
  glutPostRedisplay();
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
	  glutSetCursor (GLUT_CURSOR_UP_DOWN); break;
	case GLUT_UP: 
	  leftMouseDown = 0;
	  glutSetCursor (GLUT_CURSOR_INHERIT); break;
	default: break;
	}
	break;
  case GLUT_MIDDLE_BUTTON:
	switch (state) {
	case GLUT_DOWN:
	  middleMouseDown = firstx = firsty = 1; 
	  glutSetCursor (GLUT_CURSOR_CYCLE); break;
	case GLUT_UP: 		
	  middleMouseDown = 0; 
	  glutSetCursor (GLUT_CURSOR_INHERIT); break;
	default: break;
	}
	break;
  default: break;
  }
}

/******************************************************************************/
/******************************************************************************/

void motion (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoom -= (float).02*(x-oldx);
    if (zoom < 0.0) zoom = 0.0;
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
  oldx = x;  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menu (int value)
{
  switch (value) {
  case 1:     DRAWDATAPTS = !DRAWDATAPTS; 			break;
  case 2:     DRAWIMAGEPTS = !DRAWIMAGEPTS;			break;
  case 3:     DRAWIMAGEBSPLINE = !DRAWIMAGEBSPLINE;		break;
  case 4:     DRAWIMAGEBSPLINECTRL = !DRAWIMAGEBSPLINECTRL;	break;
  case 5:     DRAWIMAGEBEZ = !DRAWIMAGEBEZ;			break;
  case 6:     DRAWIMAGEBEZCTRL = !DRAWIMAGEBEZCTRL;		break;
  case 7:     DRAWS3BEZ = !DRAWS3BEZ;				break;
  case 8:     DRAWS3BEZCTRL = !DRAWS3BEZCTRL;			break;
  default:    break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void display (void)
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transx, transy, transz);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  glScalef  (zoom, zoom, zoom);

  glColor3fv (Grey);  
  glutWireSphere (1.0, 40, 40);
  if (DRAWDATAPTS)
   {
    glColor3fv (Black);
    glBegin (GL_POINTS);
    for (i=0; i<n; i++) glVertex3f (pt[i][0], pt[i][1], pt[i][3]);
    glEnd();
   } 
  if (DRAWIMAGEPTS)
   {
    glColor3fv (Red);
    glBegin (GL_POINTS);
    for (i=0; i<n; i++) glVertex3f (imagept[i][0], imagept[i][2], imagept[i][3]);
    glEnd();
   } 
  if (DRAWIMAGEBSPLINECTRL)
   {  
    glColor3fv (Blue);  
    imagebspl.drawCtrlPoly();
   }
  if (DRAWIMAGEBSPLINE)
   {
    glColor3fv (OrangeRed);
    imagebspl.draw();
//    glBegin(GL_LINE_STRIP);
//    for (i=0; i<imagebspl.getnSample(); i++)
//      glVertex3fv (imagebspl.getSample3f(i));
//    glEnd();
//    gluBeginCurve (theNurb);
//    gluNurbsCurve (theNurb, imagebspl.getnKnot(), 
//  			  imagebspl.getAllKnot(),
//			  3, 
//			  &(imagebspl3dCtrlPt[0][0]),
//			  imagebspl.getd() + 1, 
//			  GL_MAP1_VERTEX_3);
//    gluEndCurve (theNurb); 
   }
  if (DRAWIMAGEBEZ)
   {
    glColor3fv (Red);
    imagebez.draw();
//    for (float t = imagebez.getKnot(0); 
//         t<=imagebez.getKnot(imagebez.getnKnot()-1); t+=.01)
//         glEvalCoord1f (t);
   }
  if (DRAWIMAGEBEZCTRL)
   {
    glColor3fv (Orange);
    imagebez.drawCtrlPoly();
//    glBegin(GL_LINE_STRIP);
//    for (i=0; i<imagebez.getnCtrlPt(); i++)
//      glVertex3fv (imagebez.getCtrlPt3f(i));
//    glEnd();
   }
  if (DRAWS3BEZ)
   {
    glColor3fv (Black);
    S3bez.draw();
   }
  if (DRAWS3BEZCTRL)
   {
    glColor3fv (Grey);
    S3bez.drawCtrlPoly();
   }
  glPopMatrix();
  glutSwapBuffers ();
}

// Paper discussion. Input: sampling criterion, visualization mode x3=0 quaternions

/******************************************************************************
	Read in the unit keyframe quaternions, or randomly generate them 
	(random #, random unit vector in R^4) and record in `random.dat'.
	Maintain the sampling criterion (consecutive quaternions 
	at angular distance < samplingAngle).
	If in visualization mode, transform quaternions to x3=0 hyperplane.
******************************************************************************/

void Input(int &n, V4fArr &pt, GLboolean RANDOMINPUT, GLboolean NPTKNOWN, 
	   GLboolean INPUTANGLEAXIS, GLboolean VISUALIZE, float samplingAngle)
{
  FILE *fp;
  int   i,j;
  float angle;
  V3f   axis;

  if (RANDOMINPUT)
   {
    srand(time(0));
    if (NPTKNOWN) 
     { if (n<2) fatalError("Error: need at least two input points."); }
    else 
      n = rand() % 98 + 3; 	// generate 3-100 points
    pt.allocate(n);
    for (i=0; i<n; i++)	   // generate random pts, within the sampling criterion
      do
       {
        for (j=0; j<4; j++)
          pt[i][j] = myRand();		// random number in [-1,1]
	if (VISUALIZE) pt[i][2] = 0.;	// (--,--,0,--) quaternions for display
	pt[i].normalize();
       }
      while (i!=0 && pt[i].angle(pt[i-1]) > samplingAngle);

    fp = fopen ("random.dat", "w");
    fprintf (fp,"%i\n", n);
    for (i=0; i<n; i++)
      fprintf(fp,"%f %f %f %f\n", pt[i][0], pt[i][1], pt[i][2], pt[i][3]);
    fclose(fp);
    cout << "Random input of " << n << " points, saved in `random.dat'." << endl;
    cout << "Sampling criterion: angular difference between consecutive \
    	     points is bounded at " << samplingAngle << " radians." << endl;
    if (VISUALIZE)
      cout << "Visualization mode: Input generated in x3=0 hyperplane." << endl;
   }
  else
   {
    cin >> n;
    if (n < 2) fatalError ("Error: need at least two input points.");
    pt.allocate(n);
    if (INPUTANGLEAXIS)
      for (i=0; i<n; i++)
       {
        cin >> angle >> axis[0] >> axis[1] >> axis[2];
        angle = deg2rad(angle);
        axis.normalize();
	axis *= sin(angle/2);
        pt[i][0] = cos(angle/2);
        for (j=0; j<3; j++)	pt[i][j+1] = axis[j];
       }
    else
      for (i=0; i<n; i++)
        cin >> pt[i][0] >> pt[i][1] >> pt[i][2] >> pt[i][3]; 

    if (VISUALIZE)
     {
      for (i=0; i<n; i++)  { pt[i][2] = 0; pt[i].normalize(); }
      cout << "Visualization mode: input projected to x3=0 hyperplane." << endl;
     }
    // test that input meets sampling criterion: no large gaps
    for (i=0; i<n-1; i++)
      if (pt[i].angle(pt[i+1]) > samplingAngle)
       {
       	cout << endl << "Angle between (" 
		<< pt[i][0] << "," << pt[i][1] << "," << pt[i][2]
		<< "," << pt[i][3] << ") and ("
		<< pt[i+1][0] << "," << pt[i+1][1] << "," << pt[i+1][2]
		<< "," << pt[i+1][3] << ") is " << 
		rad2deg (pt[i].angle(pt[i+1])) 
		<< " degrees, which exceeds the sampling criterion." << endl;
        fatalError ("Error: Sampling criterion violated.\n");
       }
   }
}			  

/******************************************************************************
	Map input data to inverse space.
******************************************************************************/

void MapInput (V4fArr &pt, V4fArr &imagept, GLboolean IMAGEONS3)
{
  int i,j;
  
  imagept.allocate(pt.getn());
  for (i=0; i<imagept.getn(); i++)
   {				// (p2,p3,p4,1-p1)
    for (j=0; j<3; j++)  imagept[i][j] = pt[i][j+1];
    imagept[i][3] = 1 - pt[i][0];
    if (IMAGEONS3) imagept[i].normalize();
   }
}

/******************************************************************************/
/******************************************************************************/

int main(int argc, char **argv)
{
  int   i;
  int   ArgsParsed=0;

  RoutineName = argv[ArgsParsed++];
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
     {
     switch (argv[ArgsParsed++][1])
      {
	case 'a':	INPUTANGLEAXIS = 1;	break;
	case 'd':	VISUALIZE = 0; 		break;
	case 'r':	RANDOMINPUT = 1;	break;
	case 'R': 	NPTKNOWN = RANDOMINPUT = 1;
			n = atoi(argv[ArgsParsed++]); break;
        case 'h': 
        default:	usage(); exit(-1);
      }
    }
   else { usage(); exit(-1); }
  }  

  /************************************************************/

  Input (n,pt,RANDOMINPUT,NPTKNOWN,INPUTANGLEAXIS,VISUALIZE,MAXANGLEDIFF);
	  cout << "Input points:" << endl;
	  for (i=0; i<n; i++) printf("(%f,%f,%f,%f)\n", pt[i][0],
	  		pt[i][1],pt[i][2],pt[i][3]);
  MapInput (pt,imagept,IMAGEONS3);
	  cout << "Image points:" << endl;
	  for (i=0; i<n; i++) printf("(%f,%f,%f,%f)\n", imagept[i][0],
	  		imagept[i][1],imagept[i][2],imagept[i][3]);
  imagebspl.fit(imagept);
      cout << "Image B-spline: " << imagebspl;
    imagebspl.prepareDisplay(20,2);
  imagebez.createfrom(imagebspl);
      cout << "Image Bezier: " << imagebez;
    imagebez.prepareDisplay(20,2);
  S3bez.createFrom (imagebez);
    //  S3bez.createByS3Mapping(imagebez);
      cout << "S3 Bezier: " << S3bez; 
    S3bez.prepareDisplay(30,3);
//  float *sample;
//  cout << "Sample points on S3 curve:" << endl;
//  for (i=0; i<S3bez.getnSample(); i++)
//   {
//    sample = S3bez.getSample3f(i);
//    printf("(%f,%f,%f)\n", sample[0], sample[1], sample[2]);
//    cout << "dist " << sample[0]*sample[0] + sample[1]*sample[1] + sample[2]*sample[2] << endl;
//   }
  
  /************************************************************/

  glutInitWindowPosition (0,0);
  glutInitWindowSize (500,500);
//  int xmax = glutGet (GLUT_SCREEN_WIDTH);
//  int ymax = glutGet (GLUT_SCREEN_HEIGHT);
//  glutInitWindowSize (500,(int) 500*ymax/xmax); /* preserve aspect ratio */
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow ("Rational quaternion spline");
  glutDisplayFunc (display);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motion);
  glutReshapeFunc (reshape);
  glutVisibilityFunc (visibility);
  gfxinit();
  glutCreateMenu (menu);
  glutAddMenuEntry ("Data points", 1);
  glutAddMenuEntry ("Image points", 2);
  glutAddMenuEntry ("Image curve (B-spline)", 3);
  glutAddMenuEntry ("Image curve control polygon (B-spline)", 4);
  glutAddMenuEntry ("Image curve (Bezier)", 5);
  glutAddMenuEntry ("Image curve control polygon (Bezier)", 6);
  glutAddMenuEntry ("S3 curve (Bezier)", 7);
  glutAddMenuEntry ("S3 curve control polygon (Bezier)", 8);
  /* other menu entries */
  glutAttachMenu (GLUT_RIGHT_BUTTON);
  glutMainLoop();
  return 0;
}

