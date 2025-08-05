/*
  File:          shadow.c++
  Author:        J.K. Johnstone 
  Created:	 6 December 2001
  Last Modified: 6 December 2001
  Purpose:       Compute the penumbra and umbra cast by shining a curve
  		 light source onto a curve obstacle in 2-space.
		 Builds on tangentialCurve.c++, software for computing
		 the tangential curve of Bezier curves, and bitang.c++,
		 software for computing bitangents.
		 It needs the moving tangent and line field from 
		 tangentialCurve.c++ and the bitangents from bitang.c++.
  Sequence: 	 4th in a sequence (interpolate, tangentialCurve, bitang, 
  		 shadow)
  Input: 	 Two polygons in 2-space, implicitly defining two interpolating  
  		 cubic Bezier curves.  The first curve is assumed to be the 
		 light, while the second curve is assumed to be the obstacle.
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
#include "Miscellany.h"		// drawImplicitLine
#include "Vector.h"		
#include "MiscVector.h"		
#include "BezierCurve.h"	// drawTangent, drawPt
#include "TangCurve.h"		// create; evalProj, drawCtrlPoly, drawPt (from RatBezierCurve inheritance)

#define PTSPERBEZSEGMENT 30      // # pts to draw on each Bezier segment

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 30)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWTANG=0;		// draw tangent at active point?
static GLboolean DRAWDUAL=0;		// draw dual of active dual point?
static GLboolean DRAWFIELD=0;		// draw line field (all tangents)?
static GLboolean DRAWALLDUAL=0;		// draw entire unclipped tangential curves?
static GLboolean rotateOb=0;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'counterclockwise' direction?

Array<BezierCurve2f> 	obstacle;	// interpolating cubic Bezier curves
Array<TangentialCurve>	obduala;	// associated tangential a-curves
Array<TangentialCurve>	obdualb;	// associated tangential b-curves
float colour[7][3] = {{1,0,0}, {0,0,1}, {0,1,0}, {0,0,0}, {1,0,1}, {0,1,1}, {1,1,0}};
BezierCurve2f	 	hodo0;		// hodograph of 1st obstacle, for interactive tangent display
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
int			obstacleWin;	// primal window identifier 
int			dualWin;	// a-dual window identifier
int			dualWin2;	// b-dual window identifier
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
  zoomdual = 1;
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

void RotateOb (void)
{
  if (spinCCW)
   {
    tActive += tDelta;
    if (tActive > obstacle[0].getKnot (obstacle[0].getnKnot() - 1))
      tActive = obstacle[0].getKnot(0);
   }
  else
   {
    tActive -= tDelta;
    if (tActive < obstacle[0].getKnot(0))
      tActive = obstacle[0].getKnot (obstacle[0].getnKnot() - 1);
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

void motiondual (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdual -= (float).001*(x-oldx);
    if (zoomdual < 0.0) zoomdual = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 			break;		// ESCAPE
  case 't':     DRAWTANG = !DRAWTANG;		break;
  case 'r':  	rotateOb = !rotateOb;
 	   	if (rotateOb) 
		     glutIdleFunc (RotateOb); 
		else glutIdleFunc (NULL); 	break;
  case 'b':  	spinCCW = !spinCCW;     	break;
  case 'd':	DRAWDUAL = !DRAWDUAL;		break;
  case 'l': 	DRAWFIELD = !DRAWFIELD;		break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1:	DRAWTANG = !DRAWTANG;		break;		// draw tangent
  case 2:	rotateOb = !rotateOb;				// spin tangent
  		if (rotateOb) 
		     glutIdleFunc (RotateOb); 
		else glutIdleFunc (NULL); 	break;
  case 3:	spinCCW = !spinCCW;		break;		// change spin direction
  case 4:	DRAWDUAL = !DRAWDUAL;		break;		// dualization test
  case 5:	DRAWFIELD = !DRAWFIELD;		break;		// line field
  default:   					break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1:     DRAWALLDUAL = !DRAWALLDUAL;			break;
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
  
  distinguish the light
  distinguish the obstacle
  paint lines of different colors delimited by bitangents,
	  ramping from black at one bitangent to white at the other bitangent
	  (or some more visible color scheme)

  for (i=0; i<obstacle.getn(); i++)
   {
    glColor3fv (colour[i%7]);
    obstacle[i].draw(); 
   }
  if (DRAWTANG)
   {
    glColor3fv (Red);
    obstacle[0].drawTangent (tActive, hodo0, 1);
    obstacle[0].drawPt 	    (tActive);
   }
  if (DRAWDUAL)
   {
    glColor3fv (Blue);
    V3f ptDual;  obduala[0].evalProj (tActive, ptDual);
    drawImplicitLine (ptDual[2], ptDual[0], ptDual[1]);    
   }
  if (DRAWFIELD)
   {
    glColor3fv (Black);
    float last = obstacle[0].getKnot( obstacle[0].getnKnot()-1 );
    for (float t=obstacle[0].getKnot(0); t<last; t += .01)
      obstacle[0].drawTangent (t, hodo0, 0);
   }
     
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayDual ()
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  for (i=0; i<obstacle.getn(); i++)    // active segments of tangential a-curves
   {
    glColor3fv (colour[i%7]);
    obduala[i].drawT (nPtsPerSegment*5);
   }
  if (DRAWALLDUAL)		       // entire tangential a-curve
   {
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (colour[i%7]);
      obduala[i].drawEntire();
     }
   }
  if (DRAWTANG) 
   { 
    glColor3fv (Red); obduala[0].drawPt(tActive); 
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayDual2 ()
{
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
   
  for (i=0; i<obstacle.getn(); i++)    // active segments of tangential b-curves
   {
    glColor3fv (colour[i%7]);
    obdualb[i].drawT (nPtsPerSegment*5);
   }
  if (DRAWALLDUAL)
   {
    for (i=0; i<obstacle.getn(); i++)
     {
      glColor3fv (colour[i%7]);
      obdualb[i].drawEntire();
     }
   }
  if (DRAWTANG) 
   { 
    glColor3fv (Red); obdualb[0].drawPt(tActive); 
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

void inputCurves (char *file, Array<BezierCurve2f> &obstacle)
{
  ifstream infile;  infile.open(file);  
  V2fArrArr Pt;		// data points, organized into polygons
  read (infile, Pt);
  scaleToUnitSquare (Pt);
  obstacle.allocate(Pt.getn());
  for (int i=0; i<Pt.getn(); i++)
   { 
    obstacle[i].fitClosed (Pt[i]);
    obstacle[i].prepareDisplay (nPtsPerSegment);
   }
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
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  inputCurves(argv[argc-1], obstacle);
  hodo0.createHodograph (obstacle[0]);	// for spinning tangent
  obduala.allocate(obstacle.getn());  obdualb.allocate(obstacle.getn());
  for (int i=0; i<obstacle.getn(); i++)	// compute clipped tangential a/b-curves
   {
    obduala[i].createA(obstacle[i], i);
    obduala[i].prepareDisplay (nPtsPerSegment*5);
    obdualb[i].createB(obstacle[i], i);
    obdualb[i].prepareDisplay (nPtsPerSegment*5);
   }
  tActive = obstacle[0].getKnot(0);	// start at beginning
  tDelta = obstacle[0].getKnot (obstacle[0].getnKnot()-1) / 8000.;

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int titleht = 20; 	// top titlebar is 20 units high
  //  int xleft    = 164;		// x-coord of lefthand side for small windows
  //  int xsize = 400, ysize = 400;	// small windows
  int xleft = 0;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  int barmargin = 8; 	// width of side bar surrounding picture
  int halfysize = (ysize - 2*titleht)/2;
  int dualxleft = xleft+xsize+2*barmargin-1;
  int adualy    = titleht+10;
  int bdualy    = titleht+10+halfysize+2*titleht+1;

  glutInitWindowPosition (xleft,titleht);		// primal window
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Curves (");  
  strcat (titlebar, argv[argc-1]);  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Tangent at active point of 1st curve [t]", 1);
  glutAddMenuEntry ("Spin tangent on 1st curve [r]", 		2);
  glutAddMenuEntry ("Reverse direction of spin [b]", 		3);
  glutAddMenuEntry ("Dual line of active point on a-dual curve (as test of dualization back to primal space) [d]", 4);
  glutAddMenuEntry ("Line field on 1st curve [l]",		5);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft,adualy);		// a-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential a-curves: more vertical tangents");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Unclipped tangential curves",	1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft, bdualy);		// b-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential b-curves: more horizontal tangents");
  dualWin2 = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual2);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Unclipped tangential curves",	1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

