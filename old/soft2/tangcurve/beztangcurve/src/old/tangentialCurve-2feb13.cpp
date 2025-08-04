/*
  File:          tangentialCurve.cpp
  Author:        J.K. Johnstone 
  Created:	 10 August 2001 (from previous tangential curve software)
  Last Modified: 13 January 2013
  Purpose:       Compute the tangential curves of a collection of curves.
                 A tangential curve of C is a dual representation of C's
		 tangent space.
  Sequence: 	 2nd in a sequence (interpolate, tangCurve, bitang)
  Input: 	 k 2d point sets, implicitly defining k interpolating cubic 
  		 Bezier curves.
  Clean status:  1/13/12
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

#include "basic/AllColor.h"
#include "basic/Miscellany.h"		                         // screen2world
#include "basic/Vector.h"		
#include "basic/Draw.h"                                      // DrawImplicitLine
#include "771/BezierCurve.h"     // drawTangent/Pt, hodograph, InputClosedCurves
#include "curve/RatBezierCurve.h"                            // evalProj, drawPt
#include "tangcurve/TangCurve.h"                        // buildTangentialCurves

static char *RoutineName;
static void usage()
 {
  cout<< "View the tangential curve system of an interpolating curve." <<endl
      << endl
      << "   -d #: display density of Bezier segment (default 10)" << endl
      << "   -g: grayscale" << endl
      << "   -l: laptop display" << endl << endl
      << "Input: <file>.pts (two curve point clouds)" << endl;
 }

START HERE
change to mouse of recent program ---

// variables related to transforms
static GLfloat   transxob,transyob,rotx,roy,rotz,zoomob, zoomduala, zoomdualb;
static GLboolean leftMouseDown=0, middleMouseDown=0;
static GLboolean firstx=1;	            // is this the first MOUSEX reading?
static GLboolean firsty=1;                              // first MOUSEY reading?
static int	 oldx,oldy;		      // previous value of MOUSEX/MOUSEY
static GLboolean ROTATE=0;		 // start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'counterclockwise' direction?
const int ZOOM=0,ROT=1,TRANSL=2;                      // left-mouse mode toggles
static GLboolean transformMode=ZOOM;                  // 0=ZOOM, 1=ROT, 2=TRANSL
// toggle variables in display loop
static GLboolean DRAWCURVE=1;        // draw curves?
static GLboolean DRAWTANG=0;	     // draw tangent at active point?
static GLboolean DRAWDUAL=0;	     // draw dual of active dual point?
static GLboolean DRAWFIELD=0;	     // draw line field (all tangents)?
static GLboolean DRAWALLDUAL=0;	     // draw entire unclipped tangential curves?
static GLboolean DRAWDUALTANG=0;     // draw tangent in dual space at active pt?
static GLboolean DRAWDUALACTIVEPT=0; // draw dual of active point?
static GLboolean DRAWDUALOFMOUSE=0;  // draw dual of mouse point?
static GLboolean DRAWCLIPLINE=0;     // draw clip lines?
int GRAY=0;	                                 // gray scale mode (for paper)?
int LAPTOP=0;                                 // display environment for laptop?
float tActive;	                                  // interactive parameter value
float tDelta;		                // increment of parameter value per step
int   obstacleWin;	                             // primal window identifier
int   dualWin;	                                     // a-dual window identifier
int   dualWin2;	                                     // b-dual window identifier
int   xsize, ysize;	                                          // window size
float xMouse, yMouse;                              // world coordinates of mouse
int   nPtsPerSegment=10;                    // # pts to draw on each Bez segment

BezCurve2fArr obstacle;	                    // interpolating cubic Bezier curves
Array<TangentialCurve> obduala;	               // associated tangential a-curves
Array<TangentialCurve> obdualb;	               // associated tangential b-curves
BezCurve2f hodo0;  // hodograph of 1st obstacle, for interactive tangent display
float colour[7][3]={{1,0,0},{0,0,1},{0,1,0},{0,0,0},{1,0,1},{0,1,1},{1,1,0}};
float greyscale[7][3] = {{0,0,0}, {.1,.1,.1}, {.2,.2,.2}, {.3,.3,.3}, 
			 {.4,.4,.4}, {.5,.5,.5}, {.6,.6,.6}};

/******************************************************************************/
/******************************************************************************/

void gfxinit()
{
  glClearColor (1.0, 1.0, 1.0, 1.0);
  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glEnable (GL_POINT_SMOOTH);
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);
  transxob = transyob = 0.0; zoomob = 1.5; zoomduala = 1.5; zoomdualb = .75;
}

/******************************************************************************/
/******************************************************************************/

void reshape(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
   if (w <= h)
       glOrtho(-2.0, 2.0, -2.0*(GLfloat)h/(GLfloat)w, 2.0*(GLfloat)h/(GLfloat)w,
	       -600, 600);
  else glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0,2.0, 
	       -600, 600);
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
    if (tActive > obstacle[0].getLastKnot()) tActive = obstacle[0].knot[0];
   }
  else
   {
    tActive -= tDelta;
    if (tActive < obstacle[0].knot[0]) tActive = obstacle[0].getLastKnot();
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) glutIdleFunc (NULL);
  else if (ROTATE)            glutIdleFunc (RotateOb);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 't':     DRAWTANG = !DRAWTANG;		    break;
  case 'r':  	ROTATE = !ROTATE;
 	   	if (ROTATE) glutIdleFunc(RotateOb); 
		else        glutIdleFunc(NULL);     break;
  case 'b':  	spinCCW = !spinCCW;     	    break;
  case 'd':	DRAWDUAL = !DRAWDUAL;		    break;
  case 'k':	DRAWDUALOFMOUSE = !DRAWDUALOFMOUSE; break;
  case 'l': 	DRAWFIELD = !DRAWFIELD;		    break;
  case '[':     transformMode = ZOOM;         break;
  case ']':     transformMode = ROT;          break;
  case '\\':    transformMode = TRANSL;       break;      // backslash next to ]
  case 27:	exit(1); 			    break;	       // ESCAPE
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 0: DRAWCURVE = !DRAWCURVE;           break;
  case 1: DRAWTANG = !DRAWTANG;		    break;	        
  case 2: ROTATE = !ROTATE;				         // spin tangent
  	  if(ROTATE) glutIdleFunc(RotateOb); 
	  else       glutIdleFunc(NULL);    break;
  case 3: spinCCW=!spinCCW;		    break;	// change spin direction
  case 4: DRAWDUAL = !DRAWDUAL;		    break;	     // dualization test
  case 5: DRAWFIELD = !DRAWFIELD;	    break;	           // line field
  case 6: DRAWDUALOFMOUSE=!DRAWDUALOFMOUSE; break;   // draw dual of mouse click
  default:   				    break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1: DRAWALLDUAL = !DRAWALLDUAL;	        break;
  case 2: DRAWDUALTANG = !DRAWDUALTANG;	        break;
  case 3: DRAWDUALACTIVEPT = !DRAWDUALACTIVEPT; break;
  case 4: DRAWDUALOFMOUSE = !DRAWDUALOFMOUSE;   break;
  case 5: DRAWCLIPLINE = !DRAWCLIPLINE;         break;
  default:   					break;
  }
  glutPostRedisplay();
}

// START HERE

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
  // want 4 buttons (3 for transforms, 1 for menu) and only have 3,
  // so for symmetry assigning all transforms to left mouse
  else if (leftMouseDown && transformMode == ZOOM)   
   {
    if (firstx)  firstx=0; else zoomob -= (float).02*(x-oldx);
    if (zoomob < 0.0) zoomob = 0.0;
   }
  else if (leftMouseDown && transformMode == ROT)
   {
    if (firstx) firstx=0; else { roty+=.5*(x-oldx); if (roty>360.) roty-=360.;}
    if (firsty) firsty=0; else { rotx+=.5*(y-oldy); if (rotx>360.) rotx-=360.;}
   }
  else if (leftMouseDown && transformMode == TRANSL)
   {
    if (firstx) firstx=0; else transx += .001*(x-oldx);
    if (firsty) firsty=0; else transy += .001*(y-oldy);
   }
  oldx = x; oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void motionduala (int x, int y)
{
  if (leftMouseDown)
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
  if (leftMouseDown)
   {
    if (firstx)  firstx=0; else zoomdualb -= (float).001*(x-oldx);
    if (zoomdualb < 0.0) zoomdualb = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomob, zoomob, zoomob);
  glTranslatef (transxob, transyob, 0);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  if (DRAWCURVE)
    for (i=0; i<obstacle.getn(); i++)
     {
       if (GRAY) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
       obstacle[i].draw(); 
     }
  if (DRAWTANG)
   {
    if (GRAY) glColor3fv (Black); else glColor3fv (Red);
    obstacle[0].drawTangent (tActive, hodo0, 1);
    obstacle[0].drawPt 	    (tActive);
   }
  if (DRAWDUAL)
   {
    if (GRAY) glColor3fv (Black); else glColor3fv (Blue);
    V3f ptDual;  obduala[0].evalProj (tActive, ptDual);
    DrawImplicitLine (ptDual[2], ptDual[1], ptDual[0]);
   }
  if (DRAWFIELD)
   {
    glColor3fv (Black);
    float avgSegLength = obstacle[0].domainLength() / obstacle[0].L;
    float stepsize = avgSegLength / nPtsPerSegment;
    float last = obstacle[0].getLastKnot();
    for (float t=obstacle[0].knot[0]; t<last; t += stepsize)
      obstacle[0].drawTangent (t, hodo0, 1);
   }
  if (DRAWDUALTANG)                           // dual of tangent of C^*(activet)
   {
    if (GRAY) glColor3fv (Black); else glColor3fv (Blue); 
    V2f pt;   obduala[0].eval     (tActive, pt);
    V2f tang; obduala[0].evalTang (tActive, tang);
                                          // in dual space, tang line has coeffs
                           // (-tang[1], tang[0], pt[0]*tang[1] - pt[1]*tang[0])
            // so its dual is (pt[0]*tang[1] - pt[1]*tang[0], tang[0], -tang[1])
    float denom = -tang[1];
    V2f dualTang ((pt[0]*tang[1] - pt[1]*tang[0]) / denom, tang[0] / denom);
    glBegin(GL_POINTS); glVertex2f (dualTang[0], dualTang[1]); glEnd();
    glRasterPos2f (dualTang[0]+.05, dualTang[1]);	           // label C(t)
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'C');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '(');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 't');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ')');
   }
  if (DRAWDUALACTIVEPT)
   {
    glColor3fv (Black);
    obstacle[0].drawPt 	    (tActive);
    V2f pt;   obstacle[0].eval (tActive, pt);
    glRasterPos2f (pt[0]+.05, pt[1]);	                           // label C(t)
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'C');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '(');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 't');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ')');
   }
  if (DRAWDUALOFMOUSE)
   {
    glColor3fv(Black); glBegin(GL_POINTS); glVertex2f(xMouse, yMouse); glEnd();
   }
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayDual ()
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomduala, zoomduala, zoomduala);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  if (DRAWCURVE)
    for (i=0; i<obstacle.getn(); i++) // active segments of tangential a-curves
     {
      if (GRAY) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
      obduala[i].drawT (nPtsPerSegment/2);
     }
  if (DRAWALLDUAL)		                    // entire tangential a-curve
    for (i=0; i<obstacle.getn(); i++)
     {
      if (GRAY) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
      obduala[i].draw(); // obduala[i].drawEntire();  
     }
  if (DRAWTANG) 
   { 
    if (GRAY) glColor3fv (Black); else glColor3fv (Red); 
    obduala[0].drawPt(tActive); 
   }
  if (DRAWDUALTANG)
   {
    if (GRAY) glColor3fv (Black); else glColor3fv (Blue); 
    obduala[0].drawTangent (tActive);
   }
  if (DRAWDUALACTIVEPT)	
   {
    glColor3fv (Black);
    V2f p; obstacle[0].eval (tActive, p);
    DrawImplicitLine (1,p[1],p[0]);
    obduala[0].eval(tActive,p);		                          // label point
    glRasterPos2f (p[0]+.15, p[1]); 		
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'C');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '*');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '(');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 't');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ')');
   }
  if (DRAWDUALOFMOUSE)
   {
    glColor3fv (Black);
    DrawImplicitLine (1,yMouse,xMouse);
   }
  if (DRAWCLIPLINE)
   {
     glColor3fv (Black);
     glBegin(GL_LINES);
     glVertex2f (-10,1);  glVertex2f (10,1);
     glVertex2f (-10,-1); glVertex2f (10,-1);
     glEnd();
   }
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayDual2 ()
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdualb, zoomdualb, zoomdualb);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
   
  if (DRAWCURVE)
    for (i=0; i<obstacle.getn(); i++)  // active segments of tangential b-curves
     {
      if (GRAY) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
      obdualb[i].drawT (nPtsPerSegment/2);
     }
  if (DRAWALLDUAL)
    for (i=0; i<obstacle.getn(); i++)
     {
      if (GRAY) glColor3fv (greyscale[i%7]); else glColor3fv (colour[i%7]);
      obdualb[i].draw; // obdualb[i].drawEntire();
     }
  if (DRAWTANG) 
   { 
    if (GRAY) glColor3fv (Black); else glColor3fv (Red); 
    obdualb[0].drawPt(tActive); 
   }
  if (DRAWDUALOFMOUSE)
   {
    glColor3fv (Black);
    DrawImplicitLine (xMouse,1,yMouse);
   }
  if (DRAWCLIPLINE)
   {
     glColor3fv (Black);
     glBegin(GL_LINES);
     glVertex2f (1,-10);  glVertex2f (1,10);
     glVertex2f (-1,-10); glVertex2f (-1,10);
     glEnd();
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();
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
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]); break;
      case 'g': GRAY = 1;				   break;
      case 'l': LAPTOP = 1;                                break;
      case 'h': 
      default:	usage(); exit(-1);			   break;
      }
   else ArgsParsed++;
  }

  ifstream infile (argv[argc-1]);
  InputClosedCurves (infile, nPtsPerSegment, obstacle);
  // do in a for loop, one curve at a time
  buildTangentialCurves (obstacle, nPtsPerSegment/2, obduala, obdualb);

  obstacle[0].hodograph (hodo0);                         // for spinning tangent
  tActive = obstacle[0].knot[0];	                   // start at beginning
  if (LAPTOP) tDelta = (obstacle[0].getLastKnot() - tActive) / 2000.;
  else        tDelta = (obstacle[0].getLastKnot() - tActive) / 8000.;

  /****************************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int xleft;		                             // x-coord of lefthand side
  int barmargin; 	                // width of side bar surrounding picture
  int titleht; 		                                  // top titlebar height
  int adualy, bdualy;                  // starting y-coordinate of right windows
  int halfysize;                                     // size of top right window
  int dualxleft;                                   // left side of right windows
  if (LAPTOP)
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
  else if (GRAY)
   {
    xleft     = 0;
    xsize     = ysize = 350; // less reduction required => better image clarity
    barmargin = 8;
    titleht   = 20;
    halfysize = (ysize - 2*titleht)/2;
    adualy    = titleht+10;
    bdualy    = titleht+10+halfysize+2*titleht+1;
    dualxleft = xleft+xsize+2*barmargin-1;
   }
  else 
   {
    xleft 	= 0;			
    xsize     = ysize = 600;
    barmargin = 5;
    halfysize = ysize/2 - 12;
    adualy    = 0;
    bdualy    = ysize/2 + 56;                        // bdualy = 24 + halfysize;
    dualxleft = xsize + 6;                               // dualxleft = xsize+9;
   }

  glutInitWindowPosition (xleft,titleht);		        // primal window
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Curves");  
  if (!GRAY)
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
  glutAddMenuEntry ("Dual line of active pt on a-dual curve (as test of dualization back to primal space) [d]", 4);
  glutAddMenuEntry ("Line field on 1st curve [l]",		5);
  glutAddMenuEntry ("Compute dual of point at mouse click [k]", 6);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft,adualy);		        // a-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential a-curves: steep tangents");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionduala);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Unclipped tangential curves",	1);
  glutAddMenuEntry ("Tangent at active point",		2);
  glutAddMenuEntry ("Dual line of active point", 	3);
  glutAddMenuEntry ("Dual of pt at mouse click [k]",    4);
  glutAddMenuEntry ("Horizontal clipper",               5);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft, bdualy);		        // b-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential b-curves: shallow tangents");
  dualWin2 = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual2);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondualb);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Unclipped tangential curves",	1);
  glutAddMenuEntry ("Vertical clipper",                 5);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
