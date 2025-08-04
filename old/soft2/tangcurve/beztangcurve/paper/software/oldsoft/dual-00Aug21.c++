/*
  File:          dual.c++ 
  Author:        J.K. Johnstone 
  Created:	 18 August 1999 
  Last Modified: 21 August 2000
  Purpose:       Use dual curves to compute a visibility graph amongst
		 curved obstacles in the plane, and then interactively
		 compute shortest paths between a source and
		 destination (source and destination position being
		 interactively controlled by the mouse).  
  Input: 	 n obstacles.  Each obstacle is represented by an array
		 of sample points, defining a closed Bezier spline
		 through interpolation.  Thus, since this looks like
		 the contour format, and since we are collecting the
		 test data from the tablet as contours, the input file
		 IS a .rawctr file.  
  Output: 	 V-graph (and interactive motion).
  History: 	 8/8/00: Fully successful computation of common tangents between
  		 	 two curves.
  
   
  if (NEWFILE) 			// output V-graph
   {
    // store common tangents as obstacle index/parameter value quartets
    // V-graph G = (V,E): V = {A,B} + points of common tangency 
    // (between 2 obstacles, the same obstacle, and A/B and obstacle);
    // E = common tangents + curved segments between vertices on the same obstacle
    // (not including segments between vertices that are already connected by a tangent)
    // cout << "Finished V-graph" << endl;
    string vGraphFile = filePrefix + ".vgraph";
    ofstream outfile;
    outfile.open(vGraphFile.c_str());
    outfile << "{ " << vGraphFile << " }\n";
    // write polygonal obstacles out (repeat input) and then topology of Vgraph
   }

  //  through mouse controls outside main:
  //    input source and destination through mouse; compute shortest path
      
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
  cout << "\t[-d displaydensity of Bezier segment]" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.rawctr" << endl;
 }

static GLfloat   zoomob, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1;	// first MOUSEX reading?
static int	 oldx;		// previous value of MOUSEX

static GLboolean JUSTVIEWING=0;		// just viewing: don't process further
// static GLboolean NEWFILE=1;		// output processed data to new file?
static GLboolean ONEOBSTACLE=0;		// only draw main obstacle?
static GLboolean DRAWPOLYGONOB=0;	// draw input polygonal obstacles?
static GLboolean DRAWCURVEOB=1;		// draw curved obstacles?
static GLboolean DRAWPOLE=1;		// draw pole?
//static GLboolean DRAWALLTANGFORINTS=0;	// draw all tangents assoc w. intersections?
//static GLboolean DRAWALLSELFTANG=0;	// draw all self tangents (one curve)
static GLboolean DRAWCOMMONTANG=0;	// draw common tangents?
static GLboolean DRAWSELFCOMMONTANG=0;	// draw self common tangents?
static GLboolean DRAWVISIBLETANG=0;	// draw visible common tangents?
static GLboolean DRAWSELFVISIBLETANG=0;	// draw visible self common tangents?
static GLboolean DRAWPOLETANG=0;	// draw all pole tangents?
//static GLboolean DRAWTRUEPOLETANG=0;	// draw true pole tangents?
static GLboolean DRAWPOLEVISIBLETANG=0;	// draw visible pole tangents?
static GLboolean DRAWDUALCTRLPOLY=0;	// draw dual control polygons?
static GLboolean DRAWDUALCURVE=1;	// draw dual curves?
static GLboolean DRAWDUALPOLE=0;	// draw dual of pole?
static GLboolean DRAWTANG=0;		// draw tangent (for spinning)?
static GLboolean DRAWDUAL=0;		// draw dual line and dual point (for spinning)?
static GLboolean DRAWHIT=1;		// draw dual intersections?
static GLboolean DRAWSELFHIT=0;		// draw dual self-intersections?
static GLboolean DRAWPOLEHIT=0;		// draw pole intersections?
static GLboolean SELFTANG=1;		// compute self-common-tangents?
static GLboolean rotateOb=1;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'ccw' direction?

// static GLboolean DEBUG = 1;

int 	    	   	n=0;		// # of obstacles
Array<Polygon2f>   	obstaclePoly;
Array<BezierCurve2f> 	obstacle;
V2f			pole(-4,-2);	// point from which to compute pole/polar (old -1,.1)
RatBezierCurve2f	poleDual;
int			nHitPole[2], nHitPoleTrue[2], nHitPoleVis[2];
FloatArr 		tHitPole[2], tHitPoleTrue[2], tHitPoleVis[2];
BezierCurve2f 		hodo[2];	// hodograph of 1st obstacle, 
					// for interactive tangent display
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
Array<RatBezierCurve2f>	obdual;		// type-1 dual curves of obstacles
int			nHit;		// # of intersections of duals
FloatArr		tHit0;		// param values of intersections on obdual[0]
FloatArr		tHit1;		// param values of intersection on obdual[1]
int			nHitSelf0, nHitSelf1;  	// # self-intersections of 2 duals
FloatArr		tHitSelf00, tHitSelf01;	// params of self-ints on dual[0]
FloatArr		tHitSelf10, tHitSelf11; // params of self-ints on dual[1]
/* int			nHitTrue;	// analogous, after filtering impostor common tangents
FloatArr 		tHit0True;
FloatArr		tHit1True;
int			nHitSelf0True, nHitSelf1True;
FloatArr		tHitSelf00True, tHitSelf01True, tHitSelf10True, tHitSelf11True; */
int			nHitVis;	// analogous, for visible tangents
FloatArr		tHit0Vis;
FloatArr		tHit1Vis;
int			nHitSelf0Vis, nHitSelf1Vis;
FloatArr		tHitSelf00Vis,  tHitSelf01Vis, tHitSelf10Vis,  tHitSelf11Vis;
int			obstacleWin;	// identifier for left obstacle window
int			dualWin;	// identifier for top right dual window

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

  zoomob = 1.;
  zoomdual = .1; 
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
	  middleMouseDown = firstx = 1; 
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
  oldx = x;  
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
//  case 3:  DRAWALLTANGFORINTS = !DRAWALLTANGFORINTS;
//  	   if (DRAWALLTANGFORINTS) { DRAWCOMMONTANG=DRAWVISIBLETANG=0;} break;
  case 4:  DRAWCOMMONTANG = !DRAWCOMMONTANG;				
  	   if (DRAWCOMMONTANG) { DRAWVISIBLETANG=0;} 			break;
  case 5:  DRAWVISIBLETANG = !DRAWVISIBLETANG;
  	   if (DRAWVISIBLETANG) { DRAWCOMMONTANG=0;} 			break;
  case 6:  spinCCW = !spinCCW;						break;
  case 7:  ONEOBSTACLE = !ONEOBSTACLE;					break;
//  case 8:  DRAWALLSELFTANG = !DRAWALLSELFTANG;
//  	   if (DRAWALLSELFTANG) { DRAWSELFCOMMONTANG=DRAWSELFVISIBLETANG=0;} break;
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

void displayOb ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

//  glColor3fv (Red); glBegin(GL_POINTS);  glVertex2f (0,0);  glEnd(); // origin
  if (DRAWPOLYGONOB) { glColor3fv(Chocolate); 
  		       for(i=0;i<n;i++) obstaclePoly[i].draw(1); }
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
    drawImplicitLine (ptDual[0], ptDual[1], ptDual[2]);    
   }
//  if (DRAWALLTANGFORINTS)
  if (DRAWCOMMONTANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nHit; i++)
     {
      V2f pt0;  obstacle[0].eval (tHit0[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit1[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   }
//  if (DRAWALLSELFTANG)
  if (DRAWSELFCOMMONTANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nHitSelf0; i++)
     {
      V2f pt0;  obstacle[0].eval (tHitSelf00[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[0].eval (tHitSelf01[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<nHitSelf1; i++)
     {
      V2f pt0;  obstacle[1].eval (tHitSelf10[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHitSelf11[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   }
  if (DRAWPOLETANG)
//  if (DRAWALLPOLETANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<n; i++)			// for each obstacle
      for (j=0; j<nHitPole[i]; j++)
       {
        V2f pt; obstacle[i].eval (tHitPole[i][j], pt);
	glVertex2f (pt[0], pt[1]);
	glVertex2f (pole[0], pole[1]);
       }
    glEnd();
   }
/*  if (DRAWCOMMONTANG)			// only true common tangents
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nHitTrue; i++)
     {
      V2f pt0;  obstacle[0].eval (tHit0True[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHit1True[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   } */
/*  if (DRAWSELFCOMMONTANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nHitSelf0True; i++)
     {
      V2f pt0;  obstacle[0].eval (tHitSelf00True[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[0].eval (tHitSelf01True[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    for (i=0; i<nHitSelf1True; i++)
     {
      V2f pt0;  obstacle[1].eval (tHitSelf10True[i], pt0);	// tangent point on ob0
      glVertex2f (pt0[0], pt0[1]);
      V2f pt1;  obstacle[1].eval (tHitSelf11True[i], pt1);
      glVertex2f (pt1[0], pt1[1]);
     }
    glEnd();
   } */
/*  if (DRAWTRUEPOLETANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<n; i++)
      for (j=0; j<nHitPoleTrue[i]; j++)
       {
        V2f pt; obstacle[i].eval (tHitPoleTrue[i][j], pt);
	glVertex2f (pt[0], pt[1]);
	glVertex2f (pole[0], pole[1]);
       }
    glEnd(); 
   } */
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
    glEnd();
   }
  if (DRAWPOLEVISIBLETANG)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<n; i++)
      for (j=0; j<nHitPoleVis[i]; j++)
       {
        V2f pt; obstacle[i].eval (tHitPoleVis[i][j], pt);
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

//  glColor3fv (Red); glBegin(GL_POINTS);  glVertex2f (0,0);  glEnd(); // origin
  if (DRAWDUALCURVE)
   {
    glColor3fv (Red);	obdual[0].draw();   
    if (!ONEOBSTACLE)
      glColor3fv (Blue);  obdual[1].draw(); 
   }
  if (DRAWDUALCTRLPOLY) 
   { 
    glColor3fv (Red);  obdual[0].drawCtrlPoly(); 
    glColor3fv (Blue); obdual[1].drawCtrlPoly(); 
   }
  if (DRAWDUALPOLE)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    glVertex2f (poleDual.getCtrlPt(0,0), poleDual.getCtrlPt(0,1));
    glVertex2f (poleDual.getCtrlPt(1,0), poleDual.getCtrlPt(1,1));
    glEnd();
   }
  if (DRAWHIT)
   {
    glColor3fv (Black);
    glBegin (GL_POINTS);
//    for (i=0; i<nHit; i++) glVertex2f (hit[i][0], hit[i][1]);
    for (i=0; i<nHit; i++)  obdual[0].drawPt (tHit0[i]);
    glEnd();
   }
  if (DRAWSELFHIT)
   {
    glColor3fv (Purple);
    glBegin (GL_POINTS);
    for (i=0; i<nHitSelf0; i++)  { obdual[0].drawPt (tHitSelf00[i]);
    				   obdual[0].drawPt (tHitSelf01[i]); }
    for (i=0; i<nHitSelf1; i++)  { obdual[1].drawPt (tHitSelf10[i]);
    				   obdual[1].drawPt (tHitSelf11[i]); }
    glEnd();
   }
  if (DRAWPOLEHIT)
   {
    glColor3fv (Black);
    glBegin(GL_POINTS);
    for (i=0; i<n; i++)
      for (j=0; j<nHitPole[i]; j++)
        obdual[i].drawPt (tHitPole[i][j]);
    glEnd();
   }
  if (DRAWTANG) { glColor3fv (Red); obdual[0].drawPt(tActive); }
/*  if (1==1)
   {
    switch (probe[activeProbe].classification) {
    case 0: glColor3fv (Blue); break;		// no overlap
    case 1: glColor3fv (Black); break;		// no intersection
    case 2: glColor3fv (Red); break;		// intersection
    default: break;
    };
    glBegin(GL_LINES);
    glVertex2f (probe[activeProbe].a[0][0], probe[activeProbe].a[0][1]);
    glVertex2f (probe[activeProbe].a[1][0], probe[activeProbe].a[1][1]);
    glVertex2f (probe[activeProbe].b[0][0], probe[activeProbe].b[0][1]);
    glVertex2f (probe[activeProbe].b[1][0], probe[activeProbe].b[1][1]);
    glEnd();
   } */
   
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/***************************INPUT***************************************************/
/******************************************************************************/

void input(char *filename, int nPtsPerSegment, string &filePrefix)
{
  ifstream infile;  infile.open(filename);
  // read data polygons as contours (glorified polygons with tablet-input software already written)
  // don't want to read as Body, otherwise contours will be hidden as private objects
  int format;  readFileName (infile, filePrefix, format);
  string comment;  readComment (infile, comment);
  getLeftBrace (infile);  string id;  infile >> id;  assert (id == "BODY");
  string name;  infile >> name;
  getLeftBrace (infile);  infile >> id;  assert (id == "SECTION"); infile >> name;
  infile >> id;  assert (id == "Z");  int zval;  infile >> zval;
  int mark = infile.tellg();  			// count the contours
  while (!tryToGetRightBrace(infile))		// look for ending brace
   {
    getLeftBrace(infile);  infile >> id;  assert (id == "CONTOUR");
    skipToMatchingRightBrace(infile); n++;
   }
  assert(n>0);  Array<Contour> ctrData(n);
  infile.seekg(mark);
  for (int i=0; i<n; i++)
   {
    getLeftBrace(infile);
    infile >> id;  infile >> name; 
    infile >> id;  assert (id == "COLOR");  int color;  infile >> color;
    mark = infile.tellg();  int nPt=0; V2f foo;	// count the points
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
    ctrData[i].create(nPt,Pt);  delete [] Pt;
    getRightBrace(infile);
   }
  getRightBrace(infile);
  getRightBrace(infile);
  
  // store for future display and use as polygonal obstacles
  obstaclePoly.allocate(n);
  for (i=0; i<n; i++)  obstaclePoly[i] = ctrData[i];

  // generate curved obstacles by interpolation
  obstacle.allocate(n);
  for (i=0; i<n; i++)  
   { 
    obstacle[i].fit (obstaclePoly[i]);  
    obstacle[i].prepareDisplay (nPtsPerSegment*3);
   }
}
   
/******************************************************************************
******************************************************************************/

void printIntersections(int nHitOrig, V2fArr &hitOrig, FloatArr &tHitOrig0, 
			FloatArr &tHitOrig1) {		
  cout << nHitOrig << " intersections of duals" << endl;
  for (int i=0; i<nHitOrig; i++)
    cout << "(" << hitOrig[i][0] << "," << hitOrig[i][1] << ") with parameter value on obstacle[0] of " 
	 << tHitOrig0[i] << " and obstacle[1] of " << tHitOrig1[i] << endl;
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
/******************************************************************************
	Culling mistakes from self tangents.
	Candidate tangents are from parameter values tHit0 to tHit1.
******************************************************************************/
/*
void cullMistakesSelf (BezierCurve2f &obstacle, BezierCurve2f &hodo,
		       int nHit,      FloatArr &tHit0, 	   FloatArr &tHit1,
		       int &nHitTrue, FloatArr &tHit0True, FloatArr &tHit1True)
{
  nHitTrue = 0;  tHit0True.allocate(nHit);  tHit1True.allocate(nHit);
  for (int i=0; i<nHit; i++)
   {
    // compare proposed common tangent with tangents at either end
    V2f pt0;   obstacle.eval (tHit0[i], pt0);	
    V2f pt1;   obstacle.eval (tHit1[i], pt1);
    V2f commonTang; for (int j=0; j<2; j++) commonTang[j] = pt1[j]-pt0[j];
    V2f tang0; hodo.eval (tHit0[i], tang0);
    V2f tang1; hodo.eval (tHit1[i], tang1); 
    float angle0;  angle0 = rad2deg(commonTang.angle (tang0));
    float angle1;  angle1 = rad2deg(commonTang.angle (tang1));
// cout << "Angle discrepancies: " << angle0 << " " << angle1 << endl;
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
/******************************************************************************
******************************************************************************/
/*
void cullMistakesPole (BezierCurve2f &obstacle, BezierCurve2f &hodo,
		       int nHit,      FloatArr &tHit, 
		       int &nHitTrue, FloatArr &tHitTrue)
{
  nHitTrue = 0;  
  tHitTrue.allocate(nHit);
  for (int i=0; i<nHit; i++)
   {
    V2f pt;   obstacle.eval (tHit[i], pt);	
    V2f commonTang; for (int j=0; j<2; j++) commonTang[j] = pt[j]-pole[j];
    V2f tang; hodo.eval (tHit[i], tang);
    float angle;  angle = rad2deg(commonTang.angle (tang));
    float radeps = 1;	// allowable angle difference in degrees
    if (angle < radeps || angle > 180-radeps)
      tHitTrue[nHitTrue++] = tHit[i];
   }
}
*/
/******************************************************************************/
/******************************************************************************/

void cullInvisible (int nHitTrue, FloatArr &tHit0True, FloatArr &tHit1True,
		    int &nHitVis, FloatArr &tHit0Vis,  FloatArr &tHit1Vis)
{		    
 nHitVis = 0;
 tHit0Vis.allocate(nHitTrue);  tHit1Vis.allocate (nHitTrue);
 for (int i=0; i<nHitTrue; i++)
  {
   V2fArr ptCommon(2);
   obstacle[0].eval (tHit0True[i], ptCommon[0]);	
   obstacle[1].eval (tHit1True[i], ptCommon[1]);
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
     tHit0Vis[nHitVis]   = tHit0True[i];
     tHit1Vis[nHitVis++] = tHit1True[i];
    }
  }
}

/******************************************************************************/
/******************************************************************************/

void cullInvisibleSelf (BezierCurve2f &ob,
	int nHitTrue, FloatArr &tHit0True, FloatArr &tHit1True,
	int &nHitVis, FloatArr &tHit0Vis,  FloatArr &tHit1Vis)
{		    
 nHitVis = 0; tHit0Vis.allocate(nHitTrue);  tHit1Vis.allocate (nHitTrue);
 for (int i=0; i<nHitTrue; i++)
  {
   V2fArr ptCommon(2);
   ob.eval (tHit0True[i], ptCommon[0]);	
   ob.eval (tHit1True[i], ptCommon[1]);
   FloatArr knotCommon(2);  knotCommon[0]=0.;  knotCommon[1]=1.;
   BezierCurve2f commonTang(1,1,ptCommon,knotCommon);	// tangent line
   int n1,n2;  V2fArr foopt;  FloatArr tfoo,btfoo;
   commonTang.intersectInterior (obstacle[0], n1, foopt, tfoo, btfoo, .0000001);
   if (n1==0) 
     commonTang.intersectInterior (obstacle[1], n2, foopt, tfoo, btfoo, .0000001);
   if (n1==0 && n2==0)	// common tangent doesn't intersect in interior
    {
     tHit0Vis[nHitVis]   = tHit0True[i];
     tHit1Vis[nHitVis++] = tHit1True[i];
    }
  }
}

/******************************************************************************/
/******************************************************************************/

void cullInvisiblePole (BezierCurve2f &ob,
			int nHitTrue, FloatArr &tHitTrue,
			int &nHitVis, FloatArr &tHitVis)
{
 nHitVis = 0; tHitVis.allocate(nHitTrue);
 for (int i=0; i<nHitTrue; i++)
  {
   V2fArr ptCommon(2);
   ob.eval (tHitTrue[i], ptCommon[0]);		ptCommon[1] = pole;
   FloatArr knotCommon(2);  knotCommon[0]=0.;  knotCommon[1]=1.;
   BezierCurve2f commonTang(1,1,ptCommon,knotCommon);	// tangent line
   int n1,n2;  V2fArr foopt;  FloatArr tfoo,btfoo;
   commonTang.intersectInterior (obstacle[0], n1, foopt, tfoo, btfoo, .0000001);
   if (n1==0) 
     commonTang.intersectInterior (obstacle[1], n2, foopt, tfoo, btfoo, .0000001);
   if (n1==0 && n2==0)	// common tangent doesn't intersect in interior
     tHitVis[nHitVis++]   = tHitTrue[i];
  }
}

/******************************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  int       nPtsPerSegment = PTSPERBEZSEGMENT;
  float     eps = .0001;	// accuracy of intersection computation

  
  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'j': JUSTVIEWING=1; 					break;
//    case 'n': NEWFILE=0; 					break;
      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }  
  
  string    filePrefix;
  input(argv[argc-1], nPtsPerSegment, filePrefix);
  
  /**************************DUALIZING**********************************/

  if (!JUSTVIEWING)			
   {
    hodo[0].createHodograph (obstacle[0]);
    hodo[1].createHodograph (obstacle[1]);
    obdual.allocate(n);
    Array<FloatArr> tHitZeroWt(n); // tHitZeroWt[i] = param values of 0 wts of obdual[i]
    for (int i=0; i<n; i++)  
     {
      obdual[i].dualOf (obstacle[i], tHitZeroWt[i]);
      obdual[i].prepareDisplay (nPtsPerSegment*30);
obdual[i].print();
     }
     		// dual of pole
    assert (!(pole[0] == 0 && pole[1] == 0));	// pole at origin causes trouble, since it maps to line at infinity
    V2f min[2], max[2]; // compute bounding box of nonignored segments of dual curves
    for (i=0; i<2; i++)
      obdual[i].extremaWithIgnoredSegments (min[i], max[i], tHitZeroWt[i]);
    for (i=0; i<2; i++)	// keep min of both boxes in min[0]/max[0]
      if (min[1][i] < min[0][i]) min[0][i] = min[1][i];
    poleDual.dualOf (pole,min[0],max[0]);

    /***************compute intersections of duals************************/
    
    V2fArr hitOrig;
    obdual[0].intersectWithIgnoredSegments (obdual[1], 
    		nHit, hitOrig, tHit0, tHit1,
    		tHitZeroWt[0], tHitZeroWt[1], eps); 
    printIntersections(nHit,hitOrig,tHit0,tHit1);
    // consider all combinations of intersection parameter values, one from dual[0], other from dual[1]
    // DON'T COMBINE ALL INTERSECTIONS!
/*    nHit = nHitOrig * nHitOrig;	tHit0.allocate (nHit);	tHit1.allocate (nHit);
*    for (i=0; i<nHitOrig; i++)
*      for (int j=0; j<nHitOrig; j++)
*       {
*	tHit0[i*nHitOrig + j] = tHitOrig0[i];
*	tHit1[i*nHitOrig + j] = tHitOrig1[j];
*       } */

    /***************compute self-intersections of duals************************/

    if (SELFTANG)
     {
      FloatArr tHitSelf00Unfiltered, tHitSelf01Unfiltered;
      int nHitSelf0Unfiltered;
      obdual[0].selfIntersectWithIgnoredSegments (nHitSelf0Unfiltered, 
      		tHitSelf00Unfiltered, tHitSelf01Unfiltered, tHitZeroWt[0], eps);
      // filter identical pairs (t,t): better to guarantee none in the first place, but...
      tHitSelf00.allocate(nHitSelf0Unfiltered);	// conservative estimate
      tHitSelf01.allocate(nHitSelf0Unfiltered);
      nHitSelf0=0;
      for (i=0; i<nHitSelf0Unfiltered; i++)
        if (fabs(tHitSelf00Unfiltered[i] - tHitSelf01Unfiltered[i]) > .01)
	 {
	  tHitSelf00[nHitSelf0]   = tHitSelf00Unfiltered[i];
	  tHitSelf01[nHitSelf0++] = tHitSelf01Unfiltered[i];
	 }
      cout << nHitSelf0 << " self-intersections of dual[0]" << endl;
for (i=0; i<nHitSelf0; i++)
  cout << tHitSelf00[i] << " " << tHitSelf01[i] << endl;
  
      FloatArr tHitSelf10Unfiltered, tHitSelf11Unfiltered;
      int nHitSelf1Unfiltered;
      obdual[1].selfIntersectWithIgnoredSegments (nHitSelf1Unfiltered, 
      		tHitSelf10Unfiltered, tHitSelf11Unfiltered, tHitZeroWt[1], eps);
      tHitSelf10.allocate(nHitSelf1Unfiltered);	
      tHitSelf11.allocate(nHitSelf1Unfiltered);
      nHitSelf1=0;
      for (i=0; i<nHitSelf1Unfiltered; i++)
        if (fabs(tHitSelf10Unfiltered[i] - tHitSelf11Unfiltered[i]) > .01)
	 {
	  tHitSelf10[nHitSelf1]   = tHitSelf10Unfiltered[i];
	  tHitSelf11[nHitSelf1++] = tHitSelf11Unfiltered[i];
	 }
      cout << nHitSelf1 << " self-intersections of dual[1]" << endl;
for (i=0; i<nHitSelf1; i++)
  cout << tHitSelf10[i] << " " << tHitSelf11[i] << endl;
     }
     
    /************ intersection of pole line **************/

    FloatArr foo, bar(0);	// ignorable param values of intersections with poleDual
    V2fArr hitPole[2];
    for (i=0; i<n; i++)	// intersect dual line of pole with dual of each obstacle
      obdual[i].intersectWithIgnoredSegments (poleDual, nHitPole[i], hitPole[i],
      			tHitPole[i], foo, tHitZeroWt[i], bar, eps);
      
    /************ remove common tangent mistakes **************/

//  no longer any mistakes
//    cullMistakes (nHit,     tHit0,     tHit1, 
//    		  nHitTrue, tHit0True, tHit1True);	// also depends on hodo,obstacle
//    if (SELFTANG)
//     {
//      cullMistakesSelf (obstacle[0], hodo[0], nHitSelf0, tHitSelf00, tHitSelf01,
//      		        nHitSelf0True, tHitSelf00True, tHitSelf01True);
//      cullMistakesSelf (obstacle[1], hodo[1], nHitSelf1, tHitSelf10, tHitSelf11,
//    		        nHitSelf1True, tHitSelf10True, tHitSelf11True);
//     }
//    for (i=0; i<n; i++)	
//      cullMistakesPole (obstacle[i], hodo[i],
//     			nHitPole[i],     tHitPole[i], 
//     			nHitPoleTrue[i], tHitPoleTrue[i]);

    /************ remove invisible common tangents **************/
    
    cullInvisible (nHit,     tHit0, 	tHit1, 
    		   nHitVis,  tHit0Vis,  tHit1Vis);
    if (SELFTANG)
     {
      cullInvisibleSelf (obstacle[0], 
      			 nHitSelf0, 	tHitSelf00, 	tHitSelf01,
      			 nHitSelf0Vis,  tHitSelf00Vis,  tHitSelf01Vis);
      cullInvisibleSelf (obstacle[1], 
      			 nHitSelf1, 	tHitSelf10, 	tHitSelf11,
      			 nHitSelf1Vis,  tHitSelf10Vis,  tHitSelf11Vis);
     }
    for (i=0; i<n; i++)
      cullInvisiblePole (obstacle[i],
      			 nHitPole[i], 	 tHitPole[i],
			 nHitPoleVis[i], tHitPoleVis[i]);

    tActive = obstacle[0].getKnot(0);		// start at beginning
//    tActive = 2.2;
    tDelta = obstacle[0].getKnot (obstacle[0].getnKnot()-1) / 8000.; 	// 8000.
   }
   
  /************************************************************/

  glutInit (&argc, argv);			// primal space window
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
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (724,30);		// dual space window
  glutInitWindowSize (545,545);
  strcpy (titlebar, "Dual curves");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Dual curves", 1);
  glutAddMenuEntry ("Dual control polygons", 2);
  glutAddMenuEntry ("Dual intersections", 4);
  glutAddMenuEntry ("Dual self-intersections", 5);
  glutAddMenuEntry ("Dual of pole", 6);
  glutAddMenuEntry ("Pole intersections", 7);
  glutAddMenuEntry ("Spin point on first dual [r]", 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
