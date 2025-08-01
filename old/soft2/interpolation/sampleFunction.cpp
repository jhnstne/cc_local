/*
  File:          sampleFunction.cpp
  Author:        J.K. Johnstone 
  Created:	 24 May 2004
  Last Modified: 24 May 2004
  Purpose:       Generate an open curve data set (e.g., a spiral) 
                 by sampling a mathematical function.
  Sequence:	 Precursor to interpolate.cpp
  Input: 	 Nothing; mathematical function is changed internally and recompiled.
  Status:        Spiral version at present.
  History: 	 5/24/04: built from interpolate.cpp
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
#include "Vector.h"		
#include "MiscVector.h"		// read, scaleToUnitSquare
#include "BezierCurve.h"	// draw, fit, fitClosed, prepareDisplay

#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment
#define WINDOWS 0		 // running under Windows?

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-s sampling step size] (default: .1)" << endl;
  cout << "\t[-a beginning parameter value] (default: 0)" << endl;
  cout << "\t[-b ending parameter value]    (default: 1)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <outputfile>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWPT=1;		// draw data points?
static GLboolean DRAWCURVE=1;		// draw curved obstacles?
static GLboolean LABELPT=0;		// number the data points?
static GLboolean DRAWCTRL=0;		// draw control polygon?

float                   t1=0,t2=1;      // beginning and end parameter values
float                   delta=.1;       // sampling step size
V2fArr		        Pt;		// data points of sampled curve
BezierCurve2f 	        obstacle;	// interpolating cubic Bezier curve
int			obstacleWin;	// window identifier 
int       		nPtsPerSegment = PTSPERBEZSEGMENT;

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
  zoomob = 1.8;
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
  case 'C':  	DRAWCTRL   = !DRAWCTRL;	  break;
  case 'n':	LABELPT    = !LABELPT; 	  break;
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
  case 3:  DRAWCTRL   = !DRAWCTRL;	break;
  case 4:  LABELPT    = !LABELPT;	break;
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
  if (DRAWCTRL)
   {
    glColor3fv (Black);
    obstacle.drawCtrlPoly();
   }
  if (DRAWCURVE)
   {
    glColor3fv (Red);
    obstacle.draw(); 
   }
  if (LABELPT)
   {
    glColor3fv (Black);
    for (j=0; j<Pt.getn(); j++)			  // label the points
      {
        glRasterPos2f (Pt[j][0]+.01, Pt[j][1]+.01);
	char str[10]; itoa (j, str);
	for (k=0; k<strlen(str); k++)
	  glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
      }
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
      case 'a': t1 = atof(argv[ArgsParsed++]);                  break;
      case 'b': t2 = atof(argv[ArgsParsed++]);                  break;
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 's': delta = atof(argv[ArgsParsed++]);               break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  Pt.allocate (ceil((t2 - t1) / delta));
  float t;
  for (t=t1, i=0; t<t2; t+=delta, i++) // sample the mathematical function
    {
      //      Pt[i][0] = cos(t) + t*sin(t);  // involute spiral, use t = [0,10]
      //      Pt[i][1] = sin(t) - t*cos(t);
      //      Pt[i][0] = cos(t) - t*sin(t);  // involute spiral, swapping -t for t in trig functions, use t = [0,10]
      //      Pt[i][1] = -sin(t) - t*cos(t);
      Pt[i][0] = cos(t) / t;  // hyperbolic spiral, use t = [9.2,16], delta = .5 (or less)
      Pt[i][1] = sin(t) / t;
    }
  scaleToUnitSquare (Pt);

  obstacle.fit (Pt);
  obstacle.prepareDisplay (nPtsPerSegment,1);

  ofstream outfile;  outfile.open(argv[argc-1]);  // output to a file
  outfile << "[ Curve dataset generated from a mathematical function by 'sampleFunction.cpp'."
	  << endl << "  delta = " << delta << ", (a,b) = (" << t1 << ", " << t2 << "). ]" << endl;
  outfile << "{" << endl;
  for (i=0; i<Pt.getn(); i++)
    outfile << Pt[i][0] << " " << Pt[i][1] << endl;
  outfile << "}" << endl;

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
    xleft 	= 0;	  // 164 for small windows
    xsize = ysize = 600;  // 600 for standard windows
    titleht 	= 20;
   }

  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Curves");  
  strcat (titlebar, " (");
  strcat (titlebar, argv[argc-1]);  
  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Data points [p]", 			     1);
  glutAddMenuEntry ("Interpolating cubic Bezier curve [c]",  2);
  glutAddMenuEntry ("Control polygon [C]",		     3);
  glutAddMenuEntry ("Number data points [n]", 		     4);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

