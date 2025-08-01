/*
  File:          flythru.cpp
  Author:        J.K. Johnstone 
  Created:	 3 June 2003 (from interpolate3d.cpp)
  Last Modified: 5 June 2003
  Purpose:       Flying along a space curve with a camera.
  Sequence:	 2nd in a sequence (interpolate3d, flythrough)
  Input: 	 1 set of 3d points
  History: 	 
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
#include "Miscellany.h"		// itoa
#include "Vector.h"		// V3f, V3fArr, cross
#include "MiscVector.h"		// read, scaleToUnitCube
#include "BezierCurve.h"	// draw, fit, prepareDisplay, eval, evalTang
#include "Matrix.h"             // coco2

#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment
#define WINDOWS 0		 // running under Windows?

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-e parameter step size in flythrough] (default: .01)" << endl;
  cout << "\t[-p] (set to Postscript printing mode; default is screen display)" << endl;
  cout << "\t[-l] (laptop)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts3d" << endl;
 }

static GLfloat   transxob, transyob, rotxob, rotyob, rotzob, zoomob;
static int 	 panLeft=0, panRight=1; // control panning for 3d effect
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean ROTATEOB=0;		// rotate flypath?
static GLboolean PANOB=0; 		// rotate object back and forth for 3d effect?
static GLboolean DRAWCURVE=1;		// draw curved flypath?
static GLboolean STARTFLY=0;            // performing flythrough?
static GLboolean FLYALONG=0;            // camera flies along?

BezierCurve3f 	 flypath;	// path of the flythrough
BezierCurve3f    hodo;          // hodograph of flypath
BezierCurve3f    hodohodo;      // 2nd hodograph of flypath (for 2nd derivative)
float            tPresent;      // parameter value of present point in flythrough
float            eps=.01;       // parameter step in flythrough
int		 obstacleWin;	// window identifier 
int       	 nPtsPerSegment = PTSPERBEZSEGMENT;
int 		 PRINTOUT=0;	// 0 for displaying on screen, 1 for printing out image
int              LAPTOP=0;      // display environment for laptop?

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  glClearColor (1.0, 1.0, 1.0, 1.0);

  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glEnable (GL_POINT_SMOOTH);
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);

  transxob = transyob = 0.0;
  rotxob = -90.; rotyob = rotzob = 0.0;
  zoomob = 1.8;
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  //  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -1000.,1000.);
  gluPerspective (60.0, (GLfloat)w/(GLfloat)h, 1.0, 20.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef (0.0, 0.0, -5.0);
}

/******************************************************************************/
/******************************************************************************/

void RotateOb (void)
{
  rotzob += 1.0; 
  if (rotzob > 360.0) rotzob -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void PanOb (void)
{
  if (panLeft)
   {
     if (LAPTOP) rotzob += 1.0; else rotzob += .3;
    if (rotzob > 360.0) rotzob -= 360.0;
    panLeft++;
    if ((LAPTOP && panLeft==30) || panLeft==100) { panLeft=0; panRight=1; }
   }
  else if (panRight)
   {
     if (LAPTOP) rotzob -= 1.0; else rotzob -= .3;
    if (rotzob < 0.0) rotzob += 360.0;
    panRight++;
    if ((LAPTOP && panRight==30) || panRight==100) { panRight=0; panLeft=1; }
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE)
    glutIdleFunc (NULL);
  else if (ROTATEOB) glutIdleFunc (RotateOb);
  else if (PANOB)    glutIdleFunc (PanOb);
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

void motionob (int x, int y)
{
  if (WINDOWS)
   {
    if (!leftMouseDown && middleMouseDown)  
     {
      if (firstx)  firstx=0; else zoomob -= (float).01*(x-oldx);
      if (zoomob < 0.0) zoomob = 0.0;
     }
    else if (leftMouseDown && middleMouseDown)
     {
      if (firstx)  firstx=0; else transxob += .01*(x-oldx); /* TRANSLATION: X */
      if (firsty)  firsty=0; else transyob += .01*(y-oldy); /* TRANSLATION: Y */
     }
    else if (leftMouseDown && !middleMouseDown) 
     {
      if (firstx)  firstx=0;
      else { rotyob += .5*(x-oldx); if (rotyob > 360.0) rotyob -= 360.0; } /* ORI: Y */

      if (firsty)  firsty=0;
      else { rotxob += .5*(y-oldy); if (rotxob > 360.0) rotxob -= 360.0; } /* ORI: X */
     }
   }
  else
   {
    if (leftMouseDown && !middleMouseDown)	   
     {
      if (firstx)  firstx=0; else zoomob -= (float).01*(x-oldx);
      if (zoomob < 0.0) zoomob = 0.0;
     }
    else if (leftMouseDown && middleMouseDown)
     {
      if (firstx)  firstx=0; else transxob += .01*(x-oldx); /* TRANSLATION: X */
      if (firsty)  firsty=0; else transyob += .01*(y-oldy); /* TRANSLATION: Y */
     }
    else if (middleMouseDown) 
     {
      if (firstx)  firstx=0;
      else { rotyob += .5*(x-oldx); if (rotyob > 360.0) rotyob -= 360.0; } /* ORI: Y */

      if (firsty)  firsty=0;
      else { rotxob += .5*(y-oldy); if (rotxob > 360.0) rotxob -= 360.0; } /* ORI: X */
     }
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
  case 27:	exit(1); 		        break;		// ESCAPE
  case ' ':     tPresent=flypath.getKnot(0); 
                STARTFLY = 1;                   break;          // start flythrough
  case '2':     DRAWCURVE = !DRAWCURVE;	        break;
  case 'r':     ROTATEOB = !ROTATEOB;			        // rotate
 	     	if (ROTATEOB) 
		     glutIdleFunc (RotateOb); 
		else glutIdleFunc (NULL); 	break;
  case 'p':	PANOB = !PANOB;				        // pan
		if (PANOB) 
		     glutIdleFunc (PanOb); 
		else glutIdleFunc (NULL); 	break;
  case 'v':     FLYALONG = !FLYALONG;           break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 2:  DRAWCURVE = !DRAWCURVE;	break;
  case 5:  FLYALONG  = !FLYALONG;       break;
  default:   				break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  // build Frenet frame
  V3f presentSample, presentTang, present2ndDeriv, presentBinormal;  
  flypath.eval     (tPresent, presentSample);
                                       //      cout << "presentSample = " << presentSample << endl;
  flypath.evalTang (tPresent, hodo, presentTang);
                                       //      cout << "presentTang = " << presentTang << endl;
  hodo.evalTang    (tPresent, hodohodo, present2ndDeriv);
                                       //      cout << "present2ndDeriv = " << present2ndDeriv << endl;
  presentTang.normalize(); present2ndDeriv.normalize();
  presentBinormal.cross (presentTang, present2ndDeriv);
                                       //      cout << "presentBinormal = " << presentBinormal << endl;

  glPushMatrix();
  if (FLYALONG)  // camera orientation based on Frenet frame
    {
      float roll,pitch,heading;  roll = pitch = heading = 10;
      glRotatef (roll,    0,0,1);
      glRotatef (pitch,   0,1,0);
      glRotatef (heading, 1,0,0);
      glTranslatef (-presentSample[0], -presentSample[1], -presentSample[2]);
      /* gluLookAt (presentSample[0], presentSample[1], presentSample[2],
	        presentSample[0]+presentTang[0],
	        presentSample[1]+presentTang[1],
	        presentSample[2]+presentTang[2],
	        presentBinormal[0], presentBinormal[1], presentBinormal[2]);  */
    }
  else
    {
      glScalef  (zoomob, zoomob, zoomob);
      glTranslatef (transxob, transyob, 0);
      glRotatef (rotxob, 1.0, 0.0, 0.0);
      glRotatef (rotyob, 0.0, 1.0, 0.0);
      glRotatef (rotzob, 0.0, 0.0, 1.0);
      glPushMatrix();                                    // orientation based on Frenet frame
      glTranslatef (presentSample[0], presentSample[1], presentSample[2]);
      Matrixf myRotMatrix(3);                            // 3x3 matrix of scalars
      myRotMatrix.coco2 (presentTang, 0, presentBinormal, 2);
      float M[16];  myRotMatrix.extractOpenGLRotMatrix (M);
      glMultMatrixf (M);
      glColor3fv (Red);
      glutWireCube(.1);
      glPopMatrix();
    }
  glutWireCube(.1);

  glPushMatrix();                          // draw cube at 20th sample
  V3f foo; flypath.getSample(20,foo);
  glTranslatef (foo[0], foo[1], foo[2]);
  glutWireCube(.1);
  glPopMatrix();

  glPushMatrix();                          // draw cube at 100th sample
           flypath.getSample(100,foo);
  glTranslatef (foo[0], foo[1], foo[2]);
  glutWireCube(.1);
  glPopMatrix();

  if (DRAWCURVE)                           // draw path
   {
    glColor3fv (Red);
    flypath.draw();
   }
  if (STARTFLY)                            // flythrough
    {
      tPresent += eps;
      if (tPresent >= flypath.getLastKnot())  
	{ STARTFLY=0; tPresent = flypath.getLastKnot(); }
    }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       i;
  int       ArgsParsed=0;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'e': eps = atof (argv[ArgsParsed++]);                break;
      case 'p': PRINTOUT = 1;					break;
      case 'l': LAPTOP = 1;                                     break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  inputCurve (argv[argc-1], nPtsPerSegment, flypath);  
  hodo.createHodograph (flypath);
  hodohodo.createHodograph (hodo);

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

  int xleft;		// x-coord of lefthand side
  int titleht; 		// top titlebar height
  int xsize,ysize;      // window size
  if (WINDOWS)
   {
    xleft 	= 0;
    xsize 	= 500; ysize = 500;
    titleht 	= 12;
   }
  else
   {
    xleft 	= 0;			// 164 for small windows
    if (PRINTOUT) xsize = ysize = 350;  // 350 for small windows (less reduction required ==> better image clarity)
    else	  xsize = ysize = 600;  // 600 for standard windows
    titleht 	= 0;
   }

  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Curves");  
  if (!PRINTOUT)
   {
    strcat (titlebar, " (");
    strcat (titlebar, argv[argc-1]);  
    strcat (titlebar, ")");
   }
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Interpolating cubic Bezier curve", 2);
  glutAddMenuEntry ("Switch viewing modes [v]",         5);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

