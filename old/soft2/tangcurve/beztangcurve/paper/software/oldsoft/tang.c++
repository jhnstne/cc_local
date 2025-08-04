/*
  File:          tang.c++ 
  Author:        J.K. Johnstone 
  Created:	 5 July 2000
  Last Modified: 3 August 2000
  Purpose:       Represent tangent space of a plane curve as a curve.
  		 A collection of lines can be represented by a curve,
		 by representing a line uniquely as a point.
  Input: 	 1 curve, using data points and interpolation.
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
#include "Miscellany.h"
#include "Contour.h"
#include "Vector.h"
#include "Polygon.h"
#include "BezierCurve.h"
#include "RatBezierCurve.h"

#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d displaydensity of Bezier segment]" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.rawctr" << endl;
 }

static GLfloat   rotx,roty,rotz, zoomob, zoomdual; // zoomdual2;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1;	// first MOUSEX reading?
static GLboolean firsty=1;
static int	 oldx,oldy;		// previous value of MOUSEX

static GLboolean DRAWPOLYGONOB=0;	// draw input polygonal obstacles?
static GLboolean DRAWCURVEOB=1;		// draw curved obstacles?
static GLboolean DRAWDUALCTRLPOLY=0;	// draw dual control polygons?
static GLboolean DRAWDUALCURVE=1;	// draw dual curves?
static GLboolean DRAWTANG=1;		// draw tangent from hodograph?
static GLboolean DRAWDUAL=1;		// draw dual line and dual point (for spinning)?
static GLboolean rotateOb=0;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'ccw' direction?

// static GLboolean DEBUG = 1;

Polygon2f   	obstaclePoly;
BezierCurve2f 	obstacle;
BezierCurve2f 	hodo;		// hodograph
BezierCurve3f	obdual;		// dual in projective space
float 		tActive;	// interactive parameter value 					
float 		tDelta;		// increment of parameter value per step
int		obstacleWin;	// identifier for left obstacle window
int		dualWin;	// identifier for top right dual window

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  glClearColor (1.0, 1.0, 1.0, 1.0);

  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glEnable (GL_POINT_SMOOTH); 			// too slow for rotation 
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);

  rotx = roty = rotz = 0.;
  zoomob = .6;
  zoomdual = 1.; 
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
    if (tActive > obstacle.getKnot (obstacle.getnKnot() - 1))
      tActive = obstacle.getKnot(0);
   }
  else
   {
    tActive -= tDelta;
    if (tActive < obstacle.getKnot(0))
      tActive = obstacle.getKnot (obstacle.getnKnot() - 1);
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) {
    if (rotateOb)
      glutIdleFunc (NULL);
  }
  else if (rotateOb)
    glutIdleFunc (RotateOb);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); break;					// ESCAPE
  case 'r':  	rotateOb = !rotateOb;
 	   	if (rotateOb) glutIdleFunc (RotateOb); else glutIdleFunc (NULL); break;
  case 'b':  	spinCCW = !spinCCW;     break;
  case 'd':	DRAWDUAL = !DRAWDUAL;	break;
  case 't':     DRAWTANG = !DRAWTANG;	break;
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
 	  leftMouseDown = firstx = 1;
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
  oldx = x;  
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

void menuOb (int value)
{
  switch (value) {
  case 1:  rotateOb = !rotateOb;
 	   if (rotateOb) glutIdleFunc (RotateOb); else glutIdleFunc (NULL); break;
  case 2:  DRAWTANG = !DRAWTANG; 					break;
  case 6:  spinCCW = !spinCCW;						break;
  case 10: DRAWDUAL = !DRAWDUAL;
  default:   								break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1:     DRAWDUALCURVE 	= !DRAWDUALCURVE; 		break;
  case 2:     DRAWDUALCTRLPOLY  = !DRAWDUALCTRLPOLY;		break;
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************
	Angle difference, in degrees, between computed and actual tangent
	vector at active point.
******************************************************************************/

float angleDiff ()
{
  V3f pt;  obdual.eval (tActive, pt);	// (a,b,c)
  V2f tangComputed(-pt[1],pt[0]);	// (-b,a)
  V2f tangPerfect;  hodo.eval (tActive, tangPerfect);
  return (rad2deg (tangComputed.angle (tangPerfect)));
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  glColor3fv (Red); glBegin(GL_POINTS);  glVertex2f (0,0);  glEnd(); // origin
  if (DRAWPOLYGONOB) { glColor3fv (Chocolate);  obstaclePoly.draw(1); }
  if (DRAWCURVEOB)   { glColor3fv (Red);	obstacle.draw(); }
  if (DRAWTANG) { glColor3fv(Red); 
  		  obstacle.drawTangent(tActive,hodo,1); 
  		  obstacle.drawPt(tActive); 
   		}
  if (DRAWDUAL)
   {
    float ang = angleDiff();  float delta = 1;
    if (ang<delta || ang>180-delta) glColor3fv (Red); else glColor3fv (Black);
    V3f ptDual;  obdual.eval (tActive, ptDual);
    drawImplicitLine (ptDual[0], ptDual[1], ptDual[2]);
   }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayDual ()
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  glColor3fv (Red);	obdual.draw();   
  if (DRAWDUALCTRLPOLY)
   {
    glColor3fv (Blue);  obdual.drawCtrlPoly(); 
   }
  if (DRAWTANG)	
   {
    float ang = angleDiff();
    float delta = 1;
    if (ang < delta || ang > 180 - delta)
      glColor3fv (Red);
    else glColor3fv (Black);
    obdual.drawPt(tActive);
   }
   
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  int       nPtsPerSegment = PTSPERBEZSEGMENT;
  ifstream  infile;
  ofstream  outfile;
  
  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }  
  
//  cout.precision(12);
  
  /************************INPUT************************************/
  
  infile.open(argv[argc-1]);
  // read data polygons as contours (glorified polygons with tablet-input software already written)
  // don't want to read as Body, otherwise contours will be hidden as private objects
  string filePrefix;  int format;  readFileName (infile, filePrefix, format);
  string comment;  readComment (infile, comment);
  getLeftBrace (infile);  string id;  infile >> id;  assert (id == "BODY");
  string name;  infile >> name;
  getLeftBrace (infile);  infile >> id;  assert (id == "SECTION"); infile >> name;
  infile >> id;  assert (id == "Z");  int zval;  infile >> zval;
  Contour ctrData;
    getLeftBrace(infile);
    infile >> id;  infile >> name; 
    infile >> id;  assert (id == "COLOR");  int color;  infile >> color;
    int mark = infile.tellg();  int nPt=0; V2f foo;	// count the points
    while (!tryToGetLeftBrace(infile) && !tryToGetRightBrace(infile))
     { 
      infile >> foo[0] >> foo[1];     nPt++;
     }
    assert(nPt>0);  V2f *Pt; Pt = new V2f[nPt];
    infile.seekg(mark);
    infile >> Pt[0][0] >> Pt[0][1];
    for (int j=1; j<nPt; j++)
     {
      infile >> Pt[j][0] >> Pt[j][1];
      if (Pt[j]==Pt[j-1]) { j--; nPt--; }	// skip duplicate
      if (j==nPt-1 && Pt[j]==Pt[0]) nPt--;	// skip duplicate at end, too
     }
    ctrData.create(nPt,Pt);  delete [] Pt;
    getRightBrace(infile);
  
  // store for future display and use as polygonal obstacles
  obstaclePoly = ctrData;
  obstacle.fit (obstaclePoly);  
  obstacle.prepareDisplay (nPtsPerSegment*3);
   
  /**************************DUALIZING**********************************/

  obdual.dualOfProj (obstacle);
  obdual.prepareDisplay (nPtsPerSegment*30);
  hodo.createHodograph (obstacle);
  tActive = obstacle.getKnot(0);		// start at beginning
  tDelta = obstacle.getKnot (obstacle.getnKnot()-1) / 4000.; 	// 8000.

  /************************************************************/

  glutInit (&argc, argv);			// Euclidean space window
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowPosition (164,20);
  glutInitWindowSize (545,545);
  char titlebar[100]; 
  strcpy (titlebar, "Curves (");  
  strcat (titlebar, filePrefix.c_str());  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Tangent [t]", 2);
  glutAddMenuEntry ("Dual line [d]", 10);
  glutAddMenuEntry ("Spin tangent on first curve [r]", 1);
  glutAddMenuEntry ("Reverse direction of spin [b]", 6);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (724,30);		// type-1 dual window
  glutInitWindowSize (545,545);
  strcpy (titlebar, "Type-1 dual curves");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Reflection", 6);
  glutAddMenuEntry ("Dual curves", 1);
  glutAddMenuEntry ("Dual control polygons", 2);
  glutAddMenuEntry ("Dual intersections", 4);
  glutAddMenuEntry ("Dual self-intersections", 5);
  glutAddMenuEntry ("Spin point on first dual [r]", 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}


/*  V2f tang;    hodo.eval (tActive, tang);
    V2f pt;  	 obstacle.eval (tActive, pt);
    drawImplicitLine (-tang[1], tang[0], pt[0]*tang[1] - pt[1]*tang[0]);
		      THIS VERSION MATCHES TANGENT PERFECTLY */
//  drawImplicitLine (-tang[1], tang[0], ptDual[2]);	// ax + by + c = 0
// cout << ptDual[2] << " " << tang[1]*pt[0] - tang[0]*pt[1] << endl;
