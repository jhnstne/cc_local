/*
  File:          bitangent.c++ (created from dual.c++ 5/18/01)
  Author:        J.K. Johnstone 
  Created:	 18 August 1999
  Last Modified: 18 May 2001
  Purpose:       Compute bitangents in dual space 
  		 of a collection of closed Bezier curves.
  Input: 	 n Bezier curves, each defined by an array
		 of sample points that it interpolates,
		 using the .rawctr contour format.
  Output: 	 Visibility graph
  History: 	 8/8/00: Fully successful computation of common tangents between
  		 	 two curves.
		 1/22/01: Different colours for tangents and intersections
		 	  to distinguish a-space from b-space.
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
  cout << "\t[-j] (just display obstacles)" << endl;
  cout << "\t[-n] (don't output V-graph to file)" << endl;
  cout << "\t[-e eps]  (accuracy at which intersections are made: default .0001)" << endl;
  cout << "\t[-p x y]  (pole coordinates: default (-4,-2))" << endl;
  cout << "\t[-d displaydensity of Bezier segment]" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.rawctr" << endl;
 }

static GLfloat   transxob, transyob, zoomob, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY

static GLboolean JUSTVIEWING=0;		// just viewing: don't process further
static GLboolean OUTFILE=1;		// output processed data to file?
static GLboolean ONEOBSTACLE=0;		// only draw main obstacle?
static GLboolean DRAWCURVEOB=1;		// draw curved obstacles?
static GLboolean DRAWPOLE=1;		// draw pole?
static GLboolean DRAWCOMMONTANG=0;	// draw common tangents?
static GLboolean DRAWSELFCOMMONTANG=0;	// draw self common tangents?
static GLboolean DRAWPOLETANG=0;	// draw pole tangents?
static GLboolean DRAWVISIBLETANG=0;	// draw visible common tangents?
static GLboolean DRAWSELFVISIBLETANG=0;	// draw visible self common tangents?
static GLboolean DRAWPOLEVISIBLETANG=0;	// draw visible pole tangents?
static GLboolean DRAWDUALCTRLPOLY=0;	// draw dual control polygons?
static GLboolean DRAWDUALCURVE=1;	// draw dual curves?
static GLboolean DRAWDUALPOLE=0;	// draw dual of pole?
static GLboolean DRAWTANG=0;		// draw tangent (for spinning)?
static GLboolean DRAWDUAL=0;		// draw dual line and dual point (for spinning)?
static GLboolean DRAWHIT=0;		// draw dual intersections?
static GLboolean DRAWSELFHIT=0;		// draw dual self-intersections?
static GLboolean DRAWPOLEHIT=0;		// draw pole intersections?
static GLboolean DRAWFIELD=0;		// draw line field (all tangents)?
static GLboolean rotateOb=1;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'ccw' direction?

// static GLboolean DEBUG = 1;

int 	    	   	n=0;		// # of obstacles
Array<BezierCurve2f> 	obstacle;
Array<BezierCurve2f> 	hodo;		// hodographs of obstacles, for interactive tangent display
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
V2f			pole(-4,-2);	// point from which to compute pole/polar (old -1,.1)
RatBezierCurve2f	poleDual,poleDual2;
int			nHitPoleVis[2];
FloatArr 		tHitPole[2], tHitPoleVis[2];
int			nHit2PoleVis[2];
FloatArr 		tHit2Pole[2], tHit2PoleVis[2];
Array<RatBezierCurve2f>	obdual;		// tangential a-curves of obstacles
Array<RatBezierCurve2f>	obdual2;	// tangential b-curves of obstacles
Array<IntArr>		active,active2; // active segments of tangential curves
Array<FloatArr> 	tHitZeroWt; // tHitZeroWt[i] = param values of 0 wts of obdual[i]
Array<FloatArr> 	tHit2ZeroWt;
FloatArr		tHit0;		// param values of intersections on obdual[0]
FloatArr		tHit1;		// param values of intersection on obdual[1]
FloatArr		tHit20,tHit21;	// analogs for obdual2
FloatArr		tHitSelf00, tHitSelf01;	// params of self-ints on dual[0]
FloatArr		tHitSelf10, tHitSelf11; // params of self-ints on dual[1]
int			nHitVis;	// analogous, for visible tangents
FloatArr		tHit0Vis, tHit1Vis;
int			nHitSelf0Vis, nHitSelf1Vis;
FloatArr		tHitSelf00Vis,  tHitSelf01Vis, tHitSelf10Vis,  tHitSelf11Vis;
// same for tangential b-curves
FloatArr		tHit2Self00, tHit2Self01;
FloatArr		tHit2Self10, tHit2Self11;
int			nHit2Vis;	
FloatArr		tHit20Vis, tHit21Vis;
int			nHit2Self0Vis, nHit2Self1Vis;
FloatArr		tHit2Self00Vis,  tHit2Self01Vis, tHit2Self10Vis,  tHit2Self11Vis;

int			obstacleWin;	// identifier for left obstacle window
int			dualWin;	// identifier for top right dual window
int			dualWin2;	// identifier for bottom right dual window
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
  glEnable (GL_POINT_SMOOTH); 			// too slow for rotation 
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);
//glLineStipple (1, 0xAAAA);

  transxob = transyob = 0.0;
  zoomob = 3.;
  zoomdual = 3.; 
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -1000.,1000.);
//  gluOrtho2D(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0);
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
cout << tActive << endl;
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
  case 't':     DRAWTANG = !DRAWTANG;	break;
  case 'd':	DRAWDUAL = !DRAWDUAL;	break;
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

/*
void motiondual2 (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdual2 -= (float).001*(x-oldx);
    if (zoomdual2 < 0.0) zoomdual2 = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}
*/

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1:  rotateOb = !rotateOb;
 	   if (rotateOb) glutIdleFunc (RotateOb); else glutIdleFunc (NULL); break;
  case 2:  DRAWTANG = !DRAWTANG; 					break;
  case 4:  DRAWCOMMONTANG = !DRAWCOMMONTANG;				
  	   if (DRAWCOMMONTANG) { DRAWVISIBLETANG=0;} 			break;
  case 5:  DRAWVISIBLETANG = !DRAWVISIBLETANG;
  	   if (DRAWVISIBLETANG) { DRAWCOMMONTANG=0;} 			break;
  case 6:  spinCCW = !spinCCW;						break;
  case 7:  ONEOBSTACLE = !ONEOBSTACLE;					break;
  case 9:  DRAWSELFCOMMONTANG = !DRAWSELFCOMMONTANG;
  	   if (DRAWSELFCOMMONTANG) { DRAWSELFVISIBLETANG=0;} break;
  case 10: DRAWDUAL = !DRAWDUAL;					break;
  case 11: DRAWSELFVISIBLETANG = !DRAWSELFVISIBLETANG;
  	   if (DRAWSELFVISIBLETANG) { DRAWSELFCOMMONTANG=0;} 		break;
  case 12: DRAWPOLE = !DRAWPOLE;					break;
  case 13: DRAWPOLETANG = !DRAWPOLETANG;	
  	   if (DRAWPOLETANG) { DRAWPOLEVISIBLETANG=0; } 		break;
  case 15: DRAWPOLEVISIBLETANG = !DRAWPOLEVISIBLETANG;
  	   if (DRAWPOLEVISIBLETANG) { DRAWPOLETANG=0; } 		break;
  case 16: DRAWFIELD = !DRAWFIELD;					break;
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
  case 4:     DRAWHIT		= !DRAWHIT;			break;
  case 5:     DRAWSELFHIT	= !DRAWSELFHIT;			break;
  case 6:     DRAWDUALPOLE 	= !DRAWDUALPOLE;		break;
  case 7:     DRAWPOLEHIT	= !DRAWPOLEHIT;			break;
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual2 (int value)
{
  switch (value) {
  case 1:     DRAWDUALCURVE 	= !DRAWDUALCURVE; 		break;
  case 2:     DRAWDUALCTRLPOLY  = !DRAWDUALCTRLPOLY;		break;
  case 4:     DRAWHIT		= !DRAWHIT;			break;
  case 5:     DRAWSELFHIT	= !DRAWSELFHIT;			break;
  case 6:     DRAWDUALPOLE 	= !DRAWDUALPOLE;		break;
  case 7:     DRAWPOLEHIT	= !DRAWPOLEHIT;			break;
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  if (DRAWCURVEOB)
   {
    glColor3fv (Red);			   obstacle[0].draw(); 
    if (!ONEOBSTACLE) { glColor3fv (Blue); obstacle[1].draw(); }
   }
  if (DRAWPOLE)
   {
    glColor3fv (Black); glBegin(GL_POINTS); glVertex2f(pole[0],pole[1]); glEnd();
   }
  if (DRAWTANG) { glColor3fv (Red);
		  obstacle[0].drawTangent (tActive, hodo[0], 1);
  		  obstacle[0].drawPt (tActive); }
  if (DRAWDUAL)
   {
    glColor3fv (Blue);
    V3f ptDual;  obdual[0].evalProj (tActive, ptDual);
    drawImplicitLine (ptDual[2], ptDual[0], ptDual[1]);    
   }
  if (DRAWFIELD)
   {
    glColor3fv (Black);
    float last = obstacle[0].getKnot( obstacle[0].getnKnot()-1 );
    for (float t=obstacle[0].getKnot(0); t<last; t += .01)
      obstacle[0].drawTangent (t, hodo[0], 0);
   }
  if (DRAWCOMMONTANG)
   {
    glColor3fv (Black);	// tangents found in a-space are black
    glBegin(GL_LINES);
    for (i=0; i<tHit0.getn(); i++)
     {
      V2f pt0;  obstacle[0].eval (tHit0[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit1[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
    glColor3fv (Green);	// tangents found in b-space are green
    glBegin(GL_LINES);
    for (i=0; i<tHit20.getn(); i++)
     {
      V2f pt0;  obstacle[0].eval (tHit20[i], pt0);
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit21[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   }
  if (DRAWSELFCOMMONTANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<tHitSelf00.getn(); i++)
     {
      V2f pt0;  obstacle[0].eval (tHitSelf00[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[0].eval (tHitSelf01[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<tHitSelf10.getn(); i++)
     {
      V2f pt0;  obstacle[1].eval (tHitSelf10[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHitSelf11[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<tHit2Self00.getn(); i++)
     {
      V2f pt0;  obstacle[0].eval (tHit2Self00[i], pt0);
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[0].eval (tHit2Self01[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<tHit2Self10.getn(); i++)
     {
      V2f pt0;  obstacle[1].eval (tHit2Self10[i], pt0);
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit2Self11[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   }
  if (DRAWPOLETANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<(ONEOBSTACLE?1:2); i++)			// for each obstacle
      for (j=0; j<tHitPole[i].getn(); j++)
       {
        V2f pt; obstacle[i].eval (tHitPole[i][j], pt);
	glVertex2f (pt[0], pt[1]);
	glVertex2f (pole[0], pole[1]);
       }
    for (i=0; i<(ONEOBSTACLE?1:2); i++)
      for (j=0; j<tHit2Pole[i].getn(); j++)
       {
        V2f pt; obstacle[i].eval (tHit2Pole[i][j], pt);
	glVertex2f (pt[0], pt[1]);
	glVertex2f (pole[0], pole[1]);
       }
    glEnd();
   }
  if (DRAWVISIBLETANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nHitVis; i++)
     {
      V2f pt0;  obstacle[0].eval (tHit0Vis[i], pt0);	
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit1Vis[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<nHit2Vis; i++)
     {
      V2f pt0;  obstacle[0].eval (tHit20Vis[i], pt0);	
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit21Vis[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   }
  if (DRAWSELFVISIBLETANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nHitSelf0Vis; i++)
     {
      V2f pt0;  obstacle[0].eval (tHitSelf00Vis[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[0].eval (tHitSelf01Vis[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<nHitSelf1Vis; i++)
     {
      V2f pt0;  obstacle[1].eval (tHitSelf10Vis[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHitSelf11Vis[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<nHit2Self0Vis; i++)
     {
      V2f pt0;  obstacle[0].eval (tHit2Self00Vis[i], pt0);
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[0].eval (tHit2Self01Vis[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<nHit2Self1Vis; i++)
     {
      V2f pt0;  obstacle[1].eval (tHit2Self10Vis[i], pt0);
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit2Self11Vis[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   }
  if (DRAWPOLEVISIBLETANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<(ONEOBSTACLE?1:2); i++)
      for (j=0; j<nHitPoleVis[i]; j++)
       {
        V2f pt; obstacle[i].eval (tHitPoleVis[i][j], pt);
	glVertex2f (pt[0], pt[1]);
	glVertex2f (pole[0], pole[1]);
       }
    for (i=0; i<(ONEOBSTACLE?1:2); i++)
      for (j=0; j<nHit2PoleVis[i]; j++)
       {
        V2f pt; obstacle[i].eval (tHit2PoleVis[i][j], pt);
	glVertex2f (pt[0], pt[1]);
	glVertex2f (pole[0], pole[1]);
       }
    glEnd();
   }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayDual ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  if (DRAWDUALCURVE)
   {
    glColor3fv (Red);	obdual[0].drawActive  (active[0], nPtsPerSegment*30);
    if (!ONEOBSTACLE) 
     { 
      glColor3fv (Blue);  
      for (i=1; i<n; i++) obdual[i].drawActive (active[i], nPtsPerSegment*30);
     }
   }
  if (DRAWDUALCTRLPOLY) 
   { 
    glColor3fv (Red);  obdual[0].drawCtrlPoly();
    glColor3fv (Blue); for (i=1; i<n; i++) obdual[i].drawCtrlPoly();
   }
  if (DRAWDUALPOLE)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    glVertex2f (poleDual.getCtrlPt(0,0), poleDual.getCtrlPt(0,1));
    glVertex2f (poleDual.getCtrlPt(1,0), poleDual.getCtrlPt(1,1));
    glEnd();
   }
  if (DRAWHIT)		// intersections in a-space are black
   {
    glColor3fv (Black);
    glBegin (GL_POINTS);
    for (i=0; i<tHit0.getn(); i++)  obdual[0].drawPt (tHit0[i]);
    glEnd();
   }
  if (DRAWSELFHIT)
   {
    glColor3fv (Purple);
    glBegin (GL_POINTS);
    for (i=0; i<tHitSelf00.getn(); i++)  { obdual[0].drawPt (tHitSelf00[i]);
    				   	   obdual[0].drawPt (tHitSelf01[i]); }
    for (i=0; i<tHitSelf10.getn(); i++)  { obdual[1].drawPt (tHitSelf10[i]);
    				 	   obdual[1].drawPt (tHitSelf11[i]); }
    glEnd();
   }
  if (DRAWPOLEHIT)
   {
    glColor3fv (Black);
    glBegin(GL_POINTS);
    for (i=0; i<(ONEOBSTACLE?1:2); i++)
      for (j=0; j<tHitPole[i].getn(); j++)
        obdual[i].drawPt (tHitPole[i][j]);
    glEnd();
   }
  if (DRAWTANG) { glColor3fv (Red); obdual[0].drawPt(tActive); }
   
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayDual2 ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  if (DRAWDUALCURVE)
   {
    glColor3fv (Red);	obdual2[0].drawActive (active2[0], nPtsPerSegment*30);
    if (!ONEOBSTACLE) 
     { 
      glColor3fv (Blue);
      for (i=1; i<n; i++) obdual2[i].drawActive (active2[i], nPtsPerSegment*30); 
     }
   }
  if (DRAWDUALCTRLPOLY) 
   { 
    glColor3fv (Red);  obdual2[0].drawCtrlPoly();
    glColor3fv (Blue); for (i=1; i<n; i++) obdual2[i].drawCtrlPoly();
   }
  if (DRAWDUALPOLE)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    glVertex2f (poleDual2.getCtrlPt(0,0), poleDual2.getCtrlPt(0,1));
    glVertex2f (poleDual2.getCtrlPt(1,0), poleDual2.getCtrlPt(1,1));
    glEnd();
   }
  if (DRAWHIT)
   {
    glColor3fv (Green);		// intersections in b-space are green
    glBegin (GL_POINTS);
    for (i=0; i<tHit20.getn(); i++)  obdual2[0].drawPt (tHit20[i]);
    glEnd();
   }
  if (DRAWSELFHIT)
   {
    glColor3fv (Purple);
    glBegin (GL_POINTS);
    for (i=0; i<tHit2Self00.getn(); i++)  { obdual2[0].drawPt (tHit2Self00[i]);
    				  	    obdual2[0].drawPt (tHit2Self01[i]); }
    for (i=0; i<tHit2Self10.getn(); i++)  { obdual2[1].drawPt (tHit2Self10[i]);
    				  	    obdual2[1].drawPt (tHit2Self11[i]); }
    glEnd();
   }
  if (DRAWPOLEHIT)
   {
    glColor3fv (Black);
    glBegin(GL_POINTS);
    for (i=0; i<(ONEOBSTACLE?1:2); i++)
      for (j=0; j<tHit2Pole[i].getn(); j++)
        obdual2[i].drawPt (tHit2Pole[i][j]);
    glEnd();
   }
  if (DRAWTANG) { glColor3fv (Red); obdual2[0].drawPt(tActive); }
   
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/***************************INPUT***************************************************/
/******************************************************************************/

void input(char *filename, string &filePrefix, int nPtsPerSegment,
	   Array<BezierCurve2f> &obstacle)
{
  string fileName(filename);		// find file prefix (for output file)
  string::size_type pos = fileName.find(".");
  fileName.erase (pos);
  filePrefix = fileName;

  ifstream infile;  infile.open(filename);
  string comment;  readComment (infile, comment);
  int nOb; infile >> nOb; 
  V2fArrArr Pt(nOb);
  for (int i=0; i<nOb; i++)
   {
    getLeftBrace(infile);
    int mark = infile.tellg(); V2f foo; int nPt=0;
    while (!tryToGetRightBrace(infile))
     {
      infile >> foo[0] >> foo[1];  nPt++;
     }
    assert (nPt>0);  Pt[i].allocate(nPt);
    infile.seekg(mark);
    infile >> Pt[i][0][0] >> Pt[i][0][1];
    for (int j=1; j<nPt; j++)
     {
      infile >> Pt[i][j][0] >> Pt[i][j][1];
      if (Pt[i][j]==Pt[i][j-1]) { j--; nPt--; }	// skip duplicate
      if (j==nPt-1 && Pt[i][j]==Pt[i][0]) nPt--;	// skip duplicate at end, too
     }
    getRightBrace(infile);
   }
   
  obstacle.allocate(nOb);	// generate curved obstacles by interpolation
  for (i=0; i<nOb; i++)
   { 
    obstacle[i].fitClosed (Pt[i]);
    obstacle[i].prepareDisplay (nPtsPerSegment*3);
   }
}
   
/******************************************************************************
******************************************************************************/

void printIntersections(V2fArr &hit, FloatArr &tHit0, FloatArr &tHit1) 
{		
  cout << tHit0.getn() << " intersections of duals" << endl;
  for (int i=0; i<tHit0.getn(); i++)
    cout << "(" << hit[i][0] << "," << hit[i][1] << ") with parameter value on obstacle[0] of " 
	 << tHit0[i] << " and obstacle[1] of " << tHit1[i] << endl;
}		       

/******************************************************************************
******************************************************************************/

void printSelfIntersections (FloatArr &t0, FloatArr &t1)
{
  cout << t0.getn() << " self-intersections" << endl;
  for (int i=0; i<t0.getn(); i++)  cout << t0[i] << " " << t1[i] << endl;
}			  

/******************************************************************************
******************************************************************************/
/*
void cullMistakes (int nHit, FloatArr &tHit0, FloatArr &tHit1,
		   int &nHitTrue, FloatArr &tHit0True, FloatArr &tHit1True)
{
  nHitTrue = 0;  
  tHit0True.allocate(nHit);  tHit1True.allocate(nHit);
  for (int i=0; i<nHit; i++)
   {
    // compare proposed common tangent with tangents at either end
    V2f pt0;   obstacle[0].eval (tHit0[i], pt0);	
    V2f pt1;   obstacle[1].eval (tHit1[i], pt1);
    V2f commonTang; for (int j=0; j<2; j++) commonTang[j] = pt1[j]-pt0[j];
    V2f tang0; hodo[0].eval (tHit0[i], tang0);
    V2f tang1; hodo[1].eval (tHit1[i], tang1);
    float angle0;  angle0 = rad2deg(commonTang.angle (tang0));
    float angle1;  angle1 = rad2deg(commonTang.angle (tang1));
    float radeps = 1;	// allowable angle difference in degrees
    if ((angle0 < radeps || angle0 > 180-radeps) &&	// vectors should match 
	(angle1 < radeps || angle1 > 180-radeps))
     {
      tHit0True[nHitTrue]   = tHit0[i];
      tHit1True[nHitTrue++] = tHit1[i];
     }
   }
}
*/
/******************************************************************************/
/******************************************************************************/

void cullInvisible (FloatArr &tHit0, FloatArr &tHit1,
		    int &nHitVis, FloatArr &tHit0Vis, FloatArr &tHit1Vis)
{		    
 nHitVis = 0; int nHit = tHit0.getn();
 tHit0Vis.allocate(nHit);  tHit1Vis.allocate (nHit);
 for (int i=0; i<nHit; i++)
  {
   V2fArr ptCommon(2);
   obstacle[0].eval (tHit0[i], ptCommon[0]);	
   obstacle[1].eval (tHit1[i], ptCommon[1]);
   FloatArr knotCommon(2);  knotCommon[0]=0.;  knotCommon[1]=1.;
   BezierCurve2f commonTang(1,1,ptCommon,knotCommon);	// tangent line
   int n1,n2;  V2fArr foopt;  FloatArr tfoo,btfoo;
   commonTang.intersectInterior (obstacle[0], n1, foopt, tfoo, btfoo, .0000001);
// cout << "n1: " << n1 << " intersections" << endl;
// for (int j=0; j<n1; j++) cout << tfoo[j] << endl;
   if (n1==0) 
    { commonTang.intersectInterior (obstacle[1], n2, foopt, tfoo, btfoo, .0000001);
// cout << "n2: " << n2 << " intersections" << endl;
// for (j=0; j<n2; j++) cout << tfoo[j] << endl; 
    }
   if (n1==0 && n2==0)	// common tangent doesn't intersect in interior
    {
     tHit0Vis[nHitVis]   = tHit0[i];
     tHit1Vis[nHitVis++] = tHit1[i];
    }
  }
}

/******************************************************************************/
/******************************************************************************/

void cullInvisibleSelf (BezierCurve2f &ob, FloatArr &tHit0, FloatArr &tHit1,
	int &nHitVis, FloatArr &tHit0Vis,  FloatArr &tHit1Vis)
{		
 int nHit = tHit0.getn();    
 nHitVis = 0; tHit0Vis.allocate(nHit);  tHit1Vis.allocate (nHit);
 for (int i=0; i<nHit; i++)
  {
   V2fArr ptCommon(2);
   ob.eval (tHit0[i], ptCommon[0]);	
   ob.eval (tHit1[i], ptCommon[1]);
   FloatArr knotCommon(2);  knotCommon[0]=0.;  knotCommon[1]=1.;
   BezierCurve2f commonTang(1,1,ptCommon,knotCommon);	// tangent line
   int n1,n2;  V2fArr foopt;  FloatArr tfoo,btfoo;
   commonTang.intersectInterior (obstacle[0], n1, foopt, tfoo, btfoo, .0000001);
   if (n1==0) 
     commonTang.intersectInterior (obstacle[1], n2, foopt, tfoo, btfoo, .0000001);
   if (n1==0 && n2==0)	// common tangent doesn't intersect in interior
    {
     tHit0Vis[nHitVis]   = tHit0[i];
     tHit1Vis[nHitVis++] = tHit1[i];
    }
  }
}

/******************************************************************************/
/******************************************************************************/

void cullInvisiblePole (BezierCurve2f &ob, FloatArr &tHit,
			int &nHitVis, FloatArr &tHitVis)
{
 int nHit = tHit.getn();
 nHitVis = 0; tHitVis.allocate(nHit);
 for (int i=0; i<nHit; i++)
  {
   V2fArr ptCommon(2);
   ob.eval (tHit[i], ptCommon[0]);		ptCommon[1] = pole;
   FloatArr knotCommon(2);  knotCommon[0]=0.;  knotCommon[1]=1.;
   BezierCurve2f commonTang(1,1,ptCommon,knotCommon);	// tangent line
   int n1,n2;  V2fArr foopt;  FloatArr tfoo,btfoo;
   commonTang.intersectInterior (obstacle[0], n1, foopt, tfoo, btfoo, .0000001);
   if (n1==0) 
     commonTang.intersectInterior (obstacle[1], n2, foopt, tfoo, btfoo, .0000001);
   if (n1==0 && n2==0)	// common tangent doesn't intersect in interior
     tHitVis[nHitVis++]   = tHit[i];
  }
}




/******************************************************************************
	(1) Dualize curves and pole. 
	(2) Intersect in dual space.
	(3) Cull invisible tangents.
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  float     eps = .0001;	// accuracy of intersection computation

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'j': JUSTVIEWING=1; 					break;
      case 'n': OUTFILE=0; 					break;
      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'p': pole[0] = atof(argv[ArgsParsed++]);
      		pole[1] = atof(argv[ArgsParsed++]);		break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }  
  
  string filePrefix;
  input(argv[argc-1], filePrefix, nPtsPerSegment, obstacle);
  
  if (!JUSTVIEWING)
   {
    // CHANGE TO CLIPPING BY X=\pm 1

    hodo.allocate(n);
    for (int i=0; i<n; i++)	hodo[i].createHodograph (obstacle[i]);

    // find diagonal tangents of each obstacle
    FloatArr Knot(2);   Knot[0]=0.;   Knot[1]=1.;
    V2fArr endPt(2); endPt[0][0]=endPt[0][1]=-100; endPt[1][0]=endPt[1][1]=100;
    BezierCurve2f diagLine(1,1,endPt,Knot);	// diagonal line x=y
    endPt[0][1]=100; endPt[1][1]=-100;	// (-100,100) --> (100,-100)
    BezierCurve2f negDiagLine(1,1,endPt,Knot);	// diagonal line x=-y
    int nfoo; V2fArr ptfoo; FloatArr tfoo; 
    Array<FloatArr> tDiag1(n), tDiag2(n), tDiag(n);
    for (i=0; i<n; i++)		
     {
      hodo[i].intersect(diagLine,    nfoo, ptfoo, tDiag1[i], tfoo, eps);
      hodo[i].intersect(negDiagLine, nfoo, ptfoo, tDiag2[i], tfoo, eps);
      tDiag[i].merge (tDiag1[i], tDiag2[i]);
      cout << "Diagonal tangents for obstacle " << i << endl; tDiag[i].print();
     }

    obdual.allocate(n);	  active.allocate(n);
    for (i=0; i<n; i++)  		// compute tangential curves
     {
      obdual[i].dualOfa (obstacle[i], tDiag[i], active[i]);
      obdual[i].print();  cout << "Active segments: "; active[i].print();
      obdual[i].prepareDisplay  (nPtsPerSegment*30);
     }

    V2fArr min(n), max(n); // compute bounding box of nonignored segments of dual curves
    for (i=0; i<n; i++)
      obdual[i].extremaOfActiveSegments (min[i], max[i], active[i]);
    for (i=0; i<n; i++)	// keep min of all boxes in min[0]/max[0]
      for (int j=0; j<2; j++)
       {
        if (min[i][j] < min[0][j]) min[0][j] = min[i][j];
	if (max[i][j] > max[0][j]) max[0][j] = max[i][j];
       } 
    poleDual.dualOfa  (pole, min[0], max[0]);	// compute dual of pole
//    poleDual.print();
    V2fArr hitOrig;		
    obdual[0].intersectActiveSegments (obdual[1], hitOrig, tHit0, tHit1, 
    				       active[0], active[1], eps); 
    printIntersections(hitOrig,tHit0,tHit1);

    obdual[0].selfIntersectActiveSegments (tHitSelf00, tHitSelf01, active[0], eps);
    printSelfIntersections (tHitSelf00, tHitSelf01);
    obdual[1].selfIntersectActiveSegments (tHitSelf10, tHitSelf11, active[1], eps);
    printSelfIntersections (tHitSelf10, tHitSelf11);

    FloatArr foo;	// ignorable param values of intersections with poleDual
    IntArr activePole(1); activePole[0] = 1;
    V2fArr hitPole[2];
    for (i=0; i<n; i++)	// intersect dual line of pole with dual of each obstacle
     {
      obdual[i].intersectActiveSegments (poleDual, hitPole[i], tHitPole[i], 
      					 foo, active[i], activePole, eps);
      cout << tHitPole[i].getn() << " intersections of a-pole with a-curve" << endl;
     }
      
    cullInvisible (tHit0, tHit1, nHitVis, tHit0Vis, tHit1Vis);
    cullInvisibleSelf (obstacle[0], tHitSelf00, tHitSelf01,
      		       nHitSelf0Vis, tHitSelf00Vis, tHitSelf01Vis);
    cullInvisibleSelf (obstacle[1], tHitSelf10, tHitSelf11,
      		       nHitSelf1Vis, tHitSelf10Vis, tHitSelf11Vis);
    for (i=0; i<n; i++)
      cullInvisiblePole (obstacle[i], tHitPole[i], nHitPoleVis[i], tHitPoleVis[i]);
			 
    /**************************/

      cout << endl << "Tangential b-curves." << endl;
      obdual2.allocate(n); active2.allocate(n);
      for (i=0; i<n; i++)
       {
        obdual2[i].dualOfb (obstacle[i], tDiag[i], active2[i]);
        obdual2[i].print();  cout << "Active2 segments: "; active2[i].print();
        obdual2[i].prepareDisplay (nPtsPerSegment*30);
       }
       
      V2fArr min2(n), max2(n);
      for (i=0; i<n; i++)
        obdual2[i].extremaOfActiveSegments (min2[i], max2[i], active2[i]);
      for (i=0; i<n; i++)	// keep min of both boxes in min[0]/max[0]
        for (int j=0; j<2; j++)
         {
          if (min2[i][j] < min2[0][j]) min2[0][j] = min2[i][j];
   	  if (max2[i][j] > max2[0][j]) max2[0][j] = max2[i][j];
         } 
      poleDual2.dualOfb (pole, min2[0], max2[0]);
//      poleDual2.print();
      V2fArr hitOrig2;
      obdual2[0].intersectActiveSegments (obdual2[1], hitOrig2, tHit20, tHit21,
      					  active2[0], active2[1], eps); 
      printIntersections(hitOrig2,tHit20,tHit21);
      
      obdual2[0].selfIntersectActiveSegments (tHit2Self00, tHit2Self01, active2[0], eps);
      printSelfIntersections (tHit2Self00, tHit2Self01);
      obdual2[1].selfIntersectActiveSegments (tHit2Self10, tHit2Self11, active2[1], eps);
      printSelfIntersections (tHit2Self10, tHit2Self11);
      
      V2fArr hit2Pole[2];
      for (i=0; i<n; i++)	// intersect dual line of pole with dual of each obstacle
       {
        obdual2[i].intersectActiveSegments (poleDual2, hit2Pole[i], tHit2Pole[i],
					    foo, active2[i], activePole, eps);
	cout << tHit2Pole[i].getn() << " intersections of b-pole with b-curve" << endl;
       }

      cullInvisible (tHit20, tHit21, nHit2Vis, tHit20Vis, tHit21Vis);
      cullInvisibleSelf (obstacle[0], tHit2Self00, tHit2Self01,
      			 nHit2Self0Vis, tHit2Self00Vis, tHit2Self01Vis);
      cullInvisibleSelf (obstacle[1], tHit2Self10, tHit2Self11,
      			 nHit2Self1Vis, tHit2Self10Vis, tHit2Self11Vis);
      for (i=0; i<n; i++)
        cullInvisiblePole (obstacle[i], tHit2Pole[i], nHit2PoleVis[i], tHit2PoleVis[i]);

    tActive = obstacle[0].getKnot(0);		// start at beginning (try 2.2 for ob1)
    tDelta = obstacle[0].getKnot (obstacle[0].getnKnot()-1) / 8000.; 	// 8000.
   }
   
  /************************************************************/
    
  if (OUTFILE) 			// output V-graph
   {
    // store common tangents as obstacle index/parameter value quartets
    // V-graph G = (V,E): V = {A,B} + points of common tangency 
    // (between 2 obstacles, the same obstacle, and A/B and obstacle);
    // E = common tangents + curved segments between vertices on the same obstacle
    // (not including segments between vertices that are already connected by a tangent)
    cout << "Finished V-graph" << endl;
    string vGraphFile = filePrefix + ".vgraph";
    ofstream outfile;
    outfile.open(vGraphFile.c_str());
    outfile << "{ " << vGraphFile << " }\n";
    // write polygonal obstacles out (repeat input) and then topology of Vgraph
    // ...
   }

  /************************************************************/

  glutInit (&argc, argv);			// primal space window
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);

  int titleht = 20; 	// top titlebar is 20 units high
//  int xleft    = 164;	// x-coord of lefthand side for small windows
//  int xsize = 400, ysize = 400;	// small windows
  int xleft = 0;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
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
  glutAddMenuEntry ("Common tangents", 4);
  glutAddMenuEntry ("Visible common tangents", 5);
  glutAddMenuEntry ("Common tangents between single curve", 9);
  glutAddMenuEntry ("Visible common tangents between single curve", 11);
  glutAddMenuEntry ("Tangents from pole", 13);
  glutAddMenuEntry ("Visible tangents from pole", 15);
  glutAddMenuEntry ("Tangent [t]", 2);
  glutAddMenuEntry ("Dual line [d]", 10);
  glutAddMenuEntry ("Pole", 12);
  glutAddMenuEntry ("Spin tangent on first curve [r]", 1);
  glutAddMenuEntry ("Reverse direction of spin [b]", 6);
  glutAddMenuEntry ("One obstacle only", 7);
  glutAddMenuEntry ("Line field", 16);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  int barmargin = 8; 	// width of side bar surrounding picture
  int halfysize = (ysize - 2*titleht)/2;
  glutInitWindowPosition (xleft+xsize+2*barmargin-1,titleht+10);  // a-dual space window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential a-curves: more vertical tangents");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Dual intersections", 4);
  glutAddMenuEntry ("Dual self-intersections", 5);
  glutAddMenuEntry ("Dual of pole", 6);
  glutAddMenuEntry ("Pole intersections", 7);
  glutAddMenuEntry ("Dual curves", 1);
  glutAddMenuEntry ("Dual control polygons", 2);
  glutAddMenuEntry ("Spin point on first dual [r]", 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (xleft+xsize+2*barmargin-1,
  			  titleht+10+halfysize+2*titleht+1);	// b-dual space window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential b-curves: more horizontal tangents");
  dualWin2 = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual2);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuDual2);
  glutAddMenuEntry ("Dual intersections", 4);
  glutAddMenuEntry ("Dual self-intersections", 5);
  glutAddMenuEntry ("Dual of pole", 6);
  glutAddMenuEntry ("Pole intersections", 7);
  glutAddMenuEntry ("Dual curves", 1);
  glutAddMenuEntry ("Dual control polygons", 2);
  glutAddMenuEntry ("Spin point on first dual [r]", 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

