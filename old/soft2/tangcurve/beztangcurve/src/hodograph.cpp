/*
  File:          hodograph.cpp (tangentialcurve.cpp with tangential curves 
                                stripped out to test hodograph)
  Author:        J.K. Johnstone 
  Created:	 20 July 2012
  Last Modified: 20 July 2012
  Purpose:       Visualize tangent using hodograph of a curve.
  Input: 	 k 2d point sets, implicitly defining k interpolating cubic 
  		 Bezier curves.
*/

//       1         2         3         4         5         6         7         8
// 45678901234567890123456789012345678901234567890123456789012345678901234567890

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif
#include <fstream>
#include <iostream>
using namespace std;
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>

// START HERE

#include "basic/AllColor.h"
#include "basic/Miscellany.h"		       // drawImplicitLine, screen2world
#include "basic/Vector.h"		
#include "curve/BezierCurve.h"   // drawTangent/Pt, hodograph, InputClosedCurves

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// 0 for running under Unix, 1 for Windows

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-p] (set to Postscript printing mode; default is screen display)" << endl;
  cout << "\t[-l] (laptop)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob, zoomduala, zoomdualb;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWCURVE=1;           // draw curves?
static GLboolean DRAWTANG=0;		// draw tangent at active point?
static GLboolean DRAWDUAL=0;		// draw dual of active dual point?
static GLboolean DRAWFIELD=0;		// draw line field (all tangents)?
static GLboolean DRAWALLDUAL=0;		// draw entire unclipped tangential curves?
static GLboolean DRAWDUALTANG=0;	// draw tangent in dual space at active point?
static GLboolean DRAWDUALOFACTIVEPT=0;	// draw dual of active point?
static GLboolean DRAWDUALOFMOUSE=0;	// draw dual of mouse point?
static GLboolean DRAWCLIPLINE=0;        // draw clip lines?
static GLboolean rotateOb=0;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'counterclockwise' direction?

BezCurve2fArr obstacle;	                    // interpolating cubic Bezier curves
float colour[7][3]={{1,0,0},{0,0,1},{0,1,0},{0,0,0},{1,0,1},{0,1,1},{1,1,0}};
float greyscale[7][3] = {{0,0,0}, {.1,.1,.1}, {.2,.2,.2}, {.3,.3,.3}, 
			 {.4,.4,.4}, {.5,.5,.5}, {.6,.6,.6}};
BezCurve2f hodo0;  // hodograph of 1st obstacle, for interactive tangent display
float 		        tActive;	// interactive parameter value
float 			tDelta;		// increment of parameter value per step
int			obstacleWin;	// primal window identifier 
int			dualWin;	// a-dual window identifier
int			dualWin2;	// b-dual window identifier
int 			xsize, ysize;	// window size
float 			xMouse, yMouse; // world coordinates of mouse
int       		nPtsPerSegment = PTSPERBEZSEGMENT;
int PRINTOUT=0;	         // 0 for displaying on screen, 1 for printing out image
int LAPTOP=0;                                 // display environment for laptop?

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
  zoomduala = 1.5; zoomdualb = .75;
}

/******************************************************************************/
/******************************************************************************/

void reshape(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, 
	  -1000,1000);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void RotateOb (void)
{
  if (spinCCW)
   {
    tActive += tDelta;
    if (tActive > obstacle[0].knot[obstacle[0].getnKnot() - 1])
      tActive = obstacle[0].knot[0];
   }
  else
   {
    tActive -= tDelta;
    if (tActive < obstacle[0].knot[0])
      tActive = obstacle[0].knot[obstacle[0].getnKnot() - 1];
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE)
    glutIdleFunc (NULL);
  else if (rotateOb)
    glutIdleFunc (RotateOb);
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
	  if (!DRAWDUALOFMOUSE)
 	    glutSetCursor (GLUT_CURSOR_UP_DOWN); 
	  break;
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
  if (DRAWDUALOFMOUSE)
    screen2world (x,y,xsize,ysize,2,zoomob,transxob, transyob, xMouse, yMouse);
  else if (leftMouseDown && !middleMouseDown)	   
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

void motionduala (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomduala -= (float).001*(x-oldx);
    if (zoomduala < 0.0) zoomduala = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void motiondualb (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdualb -= (float).001*(x-oldx);
    if (zoomdualb < 0.0) zoomdualb = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 			    break;	// ESCAPE
  case 't':     DRAWTANG = !DRAWTANG;		    break;
  case 'r':  	rotateOb = !rotateOb;
 	   	if (rotateOb) 
		     glutIdleFunc (RotateOb); 
		else glutIdleFunc (NULL); 	    break;
  case 'b':  	spinCCW = !spinCCW;     	    break;
  case 'd':	DRAWDUAL = !DRAWDUAL;		    break;
  case 'k':	DRAWDUALOFMOUSE = !DRAWDUALOFMOUSE; break;
  case 'l': 	DRAWFIELD = !DRAWFIELD;		    break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 0: DRAWCURVE = !DRAWCURVE;             break;
  case 1: DRAWTANG = !DRAWTANG;		    break;	// draw tangent
  case 2: rotateOb = !rotateOb;				// spin tangent
  	  if (rotateOb) 
	    glutIdleFunc (RotateOb); 
	  else glutIdleFunc (NULL); 	    break;
  case 3: spinCCW=!spinCCW;		break;	// change spin direction
  case 4: DRAWDUAL = !DRAWDUAL;		break;	// dualization test
  case 5: DRAWFIELD = !DRAWFIELD;		break;	// line field
  case 6: DRAWDUALOFMOUSE = !DRAWDUALOFMOUSE; break; // draw dual of mouse click
  default:   					    break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1:     DRAWALLDUAL = !DRAWALLDUAL;			break;
  case 2:     DRAWDUALTANG = !DRAWDUALTANG;			break;
  case 3:     DRAWDUALOFACTIVEPT = !DRAWDUALOFACTIVEPT;		break;
  case 4:     DRAWDUALOFMOUSE = !DRAWDUALOFMOUSE;		break;
  case 5:     DRAWCLIPLINE = !DRAWCLIPLINE;                     break;
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  if (DRAWCURVE)
    for (i=0; i<obstacle.getn(); i++)
     {
       if (PRINTOUT) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
       obstacle[i].draw(); 
     }
  if (DRAWTANG)
   {
    if (PRINTOUT) glColor3fv (Black); else glColor3fv (Red);
    obstacle[0].drawTangent (tActive, hodo0, 1);
    obstacle[0].drawPt 	    (tActive);
   }
  if (DRAWDUALOFMOUSE)
   {
    glColor3fv(Black);
    if (WINDOWS)  drawPt (xMouse, yMouse);
    else { glBegin (GL_POINTS); glVertex2f (xMouse, yMouse); glEnd(); }
   }
     
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'p': PRINTOUT = 1;					break;
      case 'l': LAPTOP = 1;                                     break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  ifstream infile (argv[argc-1]);
  InputClosedCurves (infile, nPtsPerSegment, obstacle);
  obstacle[0].hodograph (hodo0); // for spinning tangent
  tActive = obstacle[0].knot[0];	// start at beginning
  if (WINDOWS || LAPTOP) tDelta = (obstacle[0].getLastKnot() - tActive) / 2000.;
  else	                 tDelta = (obstacle[0].getLastKnot() - tActive) / 8000.;

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int xleft;		// x-coord of lefthand side
  int barmargin; 	// width of side bar surrounding picture
  int titleht; 		// top titlebar height
  int adualy, bdualy;   // starting y-coordinate of right windows
  int halfysize;        // size of top right window
  int dualxleft;        // left side of right windows
  if (WINDOWS)
    {
      xleft 	= 0;
      xsize 	= 350; ysize = 350;
      barmargin = 6;
      titleht 	= 12;
      halfysize = (ysize - 2*titleht)/2;
      adualy    = titleht+10;
      bdualy    = titleht+10+halfysize+2*titleht+1;
      dualxleft = xleft+xsize+2*barmargin-1;
    }
  else if (LAPTOP)
    {
      xleft     = 0;
      xsize     = ysize = 500;
      barmargin = 12;
      titleht   = 0;
      halfysize = (ysize - 24)/2;
      adualy    = 0;
      bdualy    = titleht+halfysize+24;
      dualxleft = xleft+xsize+barmargin;
    }
  else if (PRINTOUT)
    {
      xleft     = 0;
      xsize     = ysize = 350;              // less reduction required ==> better image clarity)
      barmargin = 8;
      titleht = 20;
      halfysize = (ysize - 2*titleht)/2;
      adualy    = titleht+10;
      bdualy    = titleht+10+halfysize+2*titleht+1;
      dualxleft = xleft+xsize+2*barmargin-1;
    }
  else // LINUX box
    {
      xleft 	= 0;			
      xsize     = ysize = 600;                // 600 for standard windows
      barmargin = 5;
      //  halfysize = ysize/2 - 12;
      halfysize = ysize/2 - 12;
      adualy    = 0;
      //      bdualy    = 24 + halfysize;
      bdualy    = ysize/2 + 56;
      //      dualxleft = xsize+9;
      dualxleft = xsize + 6;
    }

  glutInitWindowPosition (xleft,titleht);		// primal window
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
  glutAddMenuEntry ("Curves",                                   0);
  glutAddMenuEntry ("Tangent at active point of 1st curve [t]", 1);
  glutAddMenuEntry ("Spin tangent on 1st curve [r]", 		2);
  glutAddMenuEntry ("Reverse direction of spin [b]", 		3);
  glutAddMenuEntry ("Dual line of active point on a-dual curve (as test of dualization back to primal space) [d]", 4);
  glutAddMenuEntry ("Line field on 1st curve [l]",		5);
  glutAddMenuEntry ("Compute dual of point at mouse click [k]", 6);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
