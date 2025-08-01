/*
  File:          animateEnvelope.cpp
  Author:        J.K. Johnstone 
  Created:	 24 May 2004
  Last Modified: 24 May 2004
  Purpose:       Animate a growing envelope of the given spine curve.
                 The spine is built of radius R and sampling step size delta, 
                 animating from the beginning to the end of the segment.

		 Used to illustrate umbral theory for 2d surrounding case.
		 Used to build dynamic datasets for the explanation of local back umbral theory.
  Sequence:	 sampleFunction --> envelope  --> animateEnvelope --> umbra
  Input: 	 envelope's spine, as data points defining an open Bezier curve
  Status:        
  History: 	 5/24/04: Built from envelope.cpp template.
*/

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif
#include <iostream>
#include <fstream>
using namespace std;
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>

#include "basic/AllColor.h"
#include "basic/Miscellany.h"		// itoa
#include "basic/Vector.h"		
#include "basic/MiscVector.h"		// read, scaleToUnitSquare
#include "curve/BezierCurve.h"	// draw, fit, fitClosed, prepareDisplay, buildEnvelope

#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment
#define WINDOWS 0		 // running under Windows?

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-p] (set to Postscript printing mode; default is screen display)" << endl;
  cout << "\t[-r envelope radius] (default .1)" << endl;
  cout << "\t[-s sampling step size] (default .1)" << endl;
  cout << "\t[-a beginning parameter value on spine] (default: first knot)" << endl;
  cout << "\t[-b ending parameter value on spine] (default: last knot)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <spineInput>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWPT=0;		// draw data points?
static GLboolean DRAWCURVE=0;		// draw curved spines?
static GLboolean LABELPT=0;		// number the data points and curves?
static GLboolean DRAWSAMPLE=0;          // draw sample points?
static GLboolean DRAWENVELOPEDATA=0;    // draw envelope probes?
static GLboolean DRAWENVELOPE=1;        // draw envelope curve?

V2fArr		        Pt;		// data points of spine curve
BezierCurve2f 	        spine;	        // spine, as interpolating cubic Bezier curve
float                   R=.1;           // radius of envelope
float                   delta=.1;       // sampling step size
float                   t1,t2;          // segment [t1,t2] of spine curve will be used; 
                                        // this allows dynamic construction of growing envelope
int                     t1set=0,t2set=0;// have t1 and t2 been set yet?
V2fArr                  envSample;      // data points of envelope
BezierCurve2f           envCurve;       // envelope curve
float                   tAnimate;       // present end parameter value of growing envelope
int			spineWin;	// window identifier 
int       		nPtsPerSegment = PTSPERBEZSEGMENT;
int 			PRINTOUT=0;	// 0 for displaying on screen, 1 for printing out image

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
  zoomob = 1.5;
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -1000.,1000.);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE)
    glutIdleFunc (NULL);
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
  oldx = x;  
  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 		  break;		// ESCAPE
  case 'p': 	DRAWPT     = !DRAWPT;	  break;
  case 'c':     DRAWCURVE  = !DRAWCURVE;  break;
  case 'n':	LABELPT    = !LABELPT; 	  break;
  case 's':     DRAWSAMPLE = !DRAWSAMPLE; break;
  case 'e':     DRAWENVELOPEDATA = !DRAWENVELOPEDATA; break;
  case 'E':     DRAWENVELOPE = !DRAWENVELOPE; break;
  case ' ':     tAnimate += .1;                           // animate the envelope growth 
                if (tAnimate > spine.getLastKnot())
		  tAnimate = spine.getLastKnot();  
		break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1:  DRAWPT     = !DRAWPT;	break;
  case 2:  DRAWCURVE  = !DRAWCURVE;	break;
  case 4:  LABELPT    = !LABELPT;	break;
  case 5:  DRAWSAMPLE = !DRAWSAMPLE;    break;
  case 6:  DRAWENVELOPEDATA = !DRAWENVELOPEDATA; break;
  case 7:  DRAWENVELOPE = !DRAWENVELOPE; break;
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
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  spine.buildEnvelope (spine.getKnot(0), tAnimate, delta, R, envCurve);

  if (DRAWPT)
   {
    glColor3fv (Black);
    if (WINDOWS)
      for (j=0; j<Pt.getn(); j++)
	drawPt (Pt[j][0], Pt[j][1]);
    else
     {
      glBegin(GL_POINTS);
      for (j=0; j<Pt.getn(); j++)
	glVertex2f (Pt[j][0], Pt[j][1]);
      glEnd();
     }
   }
  if (DRAWCURVE)
   {
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Green);
    spine.draw(); 
   }
  if (LABELPT)
   {
    glColor3fv (Black);
    for (j=0; j<Pt.getn(); j++)			  // label the points
      {
        glRasterPos2f (Pt[j][0]+.01, Pt[j][1]+.01);
	char str[10];  itoa (j, str);
	for (k=0; k<strlen(str); k++)
	  glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
      }
   }
  if (DRAWSAMPLE)
    {
      if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
      if (WINDOWS)
	for (j=0; j<spine.getnSample(); j++)
	  {
	    V2f sample;  spine.getSample (j, sample);
	    drawPt (sample[0], sample[1]);
	  }
      else
	spine.draw(1); 
    }
  if (DRAWENVELOPEDATA)                           // envelope probes
    {
      if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
      if (WINDOWS)
	for (j=0; j<envSample.getn(); j++)
	  drawPt (envSample[j][0], envSample[j][1]);
      else
	{
	  glBegin(GL_POINTS); 
	  for (j=0; j<envSample.getn(); j++) glVertex2f (envSample[j][0], envSample[j][1]); 
	  glEnd();
	}
    }
  if (DRAWENVELOPE)
    {
      if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
      envCurve.draw();       
    }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       i;
  int       ArgsParsed=0;

  RoutineName = argv[ArgsParsed++];
  if (argc < 2) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'p': PRINTOUT = 1;                                   break;
      case 'r': R = atof(argv[ArgsParsed++]);                   break;
      case 's': delta = atof(argv[ArgsParsed++]);               break;
      case 'a': t1 = atof(argv[ArgsParsed++]); t1set=1;         break;
      case 'b': t2 = atof(argv[ArgsParsed++]); t2set=1;         break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }
  
  ifstream infile;  infile.open(argv[argc-1]);  
  read (infile, Pt);
  spine.fit (Pt);
  spine.prepareDisplay (nPtsPerSegment,1);
  if (!t1set) t1 = spine.getKnot(0);
  if (!t2set) t2 = spine.getLastKnot();

  tAnimate = spine.getKnot(0) + .2;

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);

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
    titleht 	= 20;
   }

  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Animated envelope");  
  if (!PRINTOUT)
   {
    strcat (titlebar, " (");
    strcat (titlebar, argv[argc-1]);  
    strcat (titlebar, ")");
   }
  spineWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Data points [p]", 			     1);
  glutAddMenuEntry ("Spine [c]",                             2);
  glutAddMenuEntry ("Number data points [n]", 		     4);
  glutAddMenuEntry ("Sample points [s]",                     5);
  glutAddMenuEntry ("Envelope probes [e]",                   6);
  glutAddMenuEntry ("Envelope [E]",                          7);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
