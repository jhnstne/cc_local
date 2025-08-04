/*
  File:          dual.c++ 
  Author:        J.K. Johnstone 
  Created:	 18 August 1999 
  Last Modified: 3 July 2000
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
  cout << "\t[-j] (just display obstacles)" << endl;
  cout << "\t[-n] (don't output V-graph to file)" << endl;
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
static GLboolean NEWFILE=1;		// output processed data to new file?
static GLboolean ONEOBSTACLE=0;		// only draw main obstacle?
static GLboolean DRAWWIRE=1;		// draw polygons as wireframes?
static GLboolean DRAWPOLYGONOB=0;	// draw input polygonal obstacles?
static GLboolean DRAWCURVEOB=1;		// draw curved obstacles?
static GLboolean DRAWALLTANGFORINTS=0;	// draw all tangents assoc w. intersections?
static GLboolean DRAWALLSELFTANG=0;	// draw all self tangents (one curve)
static GLboolean DRAWCOMMONTANG=0;	// draw common tangents?
static GLboolean DRAWSELFCOMMONTANG=0;	// draw self common tangents?
static GLboolean DRAWVISIBLETANG=0;	// draw visible common tangents?
static GLboolean DRAWDUALCTRLPOLY=0;	// draw dual control polygons?
static GLboolean DRAWDUALCURVE=1;	// draw dual curves?
static GLboolean DRAWREFLEX=0;		// draw reflection of 1st dual curve?
static GLboolean DRAWTANG=0;		// draw tangent (for spinning)?
static GLboolean DRAWDUAL=0;		// draw dual line and dual point (for spinning)?
static GLboolean DRAWHIT=0;		// draw dual intersections?
static GLboolean DRAWSELFHIT=0;		// draw dual self-intersections?
static GLboolean rotateOb=0;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'ccw' direction?

// static GLboolean DEBUG = 1;

int 	    	   	n=0;		// # of obstacles
Array<Polygon2f>   	obstaclePoly;
Array<BezierCurve2f> 	obstacle;
BezierCurve2f 		hodo0;		// hodograph of 1st obstacle, 
					// for interactive tangent display
BezierCurve2f		hodo1;		// hodo of 2nd obstacle, for removing common tangent mistakes
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
Array<RatBezierCurve2f>	obdual;		// dual curves of obstacles
Array<RatBezierCurve2f> obdualreflex;	// reflection of duals
//Array<RatBezierCurve2f> obdualselfreflex; // reflection of duals for self-intersection
int			nHit;		
V2fArr			hit;		// intersections of duals
FloatArr		tHit0;		// param values of intersections on obdual[0]
FloatArr		tHit1;		// param values of intersection on obdual[1]
int			nHitSelf0, nHitSelf1;
V2fArr			hitSelf0, hitSelf1; 	// self-intersections of 2 duals
FloatArr		tHitSelf00, tHitSelf01;	// params of self-ints on dual[0]
FloatArr		tHitSelf10, tHitSelf11; // params of self-ints on dual[1]
int			nHitTrue;	// analogous, after filtering impostor common tangents
V2fArr 			hitTrue;
FloatArr 		tHit0True;
FloatArr		tHit1True;
int			nHitSelf0True, nHitSelf1True;
V2fArr			hitSelf0True, hitSelf1True;
FloatArr		tHitSelf00True, tHitSelf01True, tHitSelf10True, tHitSelf11True;
int			nHitVis;	// analogous, for visible tangents
V2fArr			hitVis;
FloatArr		tHit0Vis;
FloatArr		tHit1Vis;
int			obstacleWin;	// identifier for left obstacle window
int			dualWin;	// identifier for right dual window

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
//  glLineWidth(2.0);
//  glLineStipple (1, 0xAAAA);

  zoomob = 1.75;
  zoomdual = .025; 
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0);
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
  case 3:  DRAWALLTANGFORINTS = !DRAWALLTANGFORINTS;
  	   if (DRAWALLTANGFORINTS) { DRAWCOMMONTANG=DRAWVISIBLETANG=0;} break;
  case 4:  DRAWCOMMONTANG = !DRAWCOMMONTANG;				
  	   if (DRAWCOMMONTANG) { DRAWALLTANGFORINTS=DRAWVISIBLETANG=0;} break;
  case 5:  DRAWVISIBLETANG = !DRAWVISIBLETANG;
  	   if (DRAWVISIBLETANG) { DRAWCOMMONTANG=DRAWALLTANGFORINTS=0;} break;
  case 6:  spinCCW = !spinCCW;						break;
  case 7:  ONEOBSTACLE = !ONEOBSTACLE;					break;
  case 8:  DRAWALLSELFTANG = !DRAWALLSELFTANG;
  	   if (DRAWALLSELFTANG) { DRAWSELFCOMMONTANG=0;	}		break;
  case 9:  DRAWSELFCOMMONTANG = !DRAWSELFCOMMONTANG;
  	   if (DRAWSELFCOMMONTANG) { DRAWALLSELFTANG=0; }		break;
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
  case 4:     DRAWHIT		= !DRAWHIT;			break;
  case 5:     DRAWSELFHIT	= !DRAWSELFHIT;			break;
  case 6:     DRAWREFLEX	= !DRAWREFLEX;			break;
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
  glScalef  (zoomob, zoomob, zoomob);
  
  if (DRAWWIRE)
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  else
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_POINTS);  glVertex2f (0,0);  glEnd();		// origin
  if (DRAWPOLYGONOB)			// display polygonal obstacles
   {
    glColor3fv (Chocolate);
    for (i=0; i<n; i++)  obstaclePoly[i].draw(1);
   }
  if (DRAWCURVEOB)
   {
    glColor3fv (Red);			   obstacle[0].draw();
    if (!ONEOBSTACLE) { glColor3fv (Blue); obstacle[1].draw(); }
   }
  if (DRAWTANG)			// draw tangent on first curve at tActive
   {
    glColor3fv (Red);
    obstacle[0].drawPt (tActive);
    obstacle[0].drawTangent (tActive, hodo0, 1);
   }
  if (DRAWDUAL)
   {
    glColor3fv (Blue);
    obstacle[0].drawPt (tActive);
    // draw line ax+by+c=0 associated with tActive's point (a,b,c) on dual
    V3f pt;  obdual[0].evalProj (tActive, pt);	// (a,b,c)
// cout << pt[2] << endl;
    drawImplicitLine (pt[0],pt[1],pt[2]);	// ax + by + c = 0
   }
  if (DRAWALLTANGFORINTS)
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
  if (DRAWALLSELFTANG)
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
  if (DRAWCOMMONTANG)			// only true common tangents
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
   }
  if (DRAWSELFCOMMONTANG)
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
  int i;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomdual, zoomdual, zoomdual);
  
  if (DRAWWIRE)
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  else
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

  if (DRAWDUALCURVE)
   {
    glColor3fv (Red);	obdual[0].draw();   
//    glColor3fv (Green); obdual[0].drawCtrlPoly(); 
//    			obdualreflex[0].draw();
    if (!ONEOBSTACLE)
     {
      glColor3fv (Blue);  obdual[1].draw(); 
//      glColor3fv (Purple); obdual[1].drawCtrlPoly();
      if (DRAWREFLEX)
       {
        glColor3fv (Purple);  obdualreflex[1].draw();
       }
     }
   }
  if (DRAWDUALCTRLPOLY)
   {
    glColor3fv (Blue);  
    for (i=0; i<n; i++)  obdual[i].drawCtrlPoly(); 
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
  
  // draw point associated with tangent, on first dual at tActive
  if (DRAWTANG)
   {
    glColor3fv (Red);
    obdual[0].drawPt(tActive);
   }
   
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
	If selfFlag, culling mistakes from self tangents of obstacle 0.
******************************************************************************/

void cullMistakes (int nHit, V2fArr &hit, FloatArr &tHit0, FloatArr &tHit1,
		   int &nHitTrue, V2fArr &hitTrue, FloatArr &tHit0True, 
		   FloatArr &tHit1True, int selfFlag)
{
  nHitTrue = 0;  hitTrue.allocate (nHit);
  tHit0True.allocate(nHit);  tHit1True.allocate(nHit);
  for (int i=0; i<nHit; i++)
   {
    // compare proposed common tangent with tangents at either end
    V2f pt0;   obstacle[0].eval (tHit0[i], pt0);	
    V2f pt1;   obstacle[selfFlag ? 0 : 1].eval (tHit1[i], pt1);
    V2f commonTang; for (int j=0; j<2; j++) commonTang[j] = pt1[j]-pt0[j];
    V2f tang0; hodo0.eval (tHit0[i], tang0);
    V2f tang1; if (selfFlag) hodo0.eval (tHit1[i], tang1); 
    	       else 	     hodo1.eval (tHit1[i], tang1);
    float angle0;  angle0 = commonTang.angle (tang0);
    float angle1;  angle1 = commonTang.angle (tang1);
// cout << "Angles: " << angle0 << " " << angle1 << endl;
    float radeps = .1;
    if ((angle0 < radeps || angle0 > M_PI-radeps) &&	// vectors should match 
	(angle1 < radeps || angle1 > M_PI-radeps))
     {
//      hitTrue  [nHitTrue]   = hit[i];
      tHit0True[nHitTrue]   = tHit0[i];
      tHit1True[nHitTrue++] = tHit1[i];
     }
   }
}

/******************************************************************************/
/******************************************************************************/

void cullInvisible (int nHitTrue, V2fArr &hitTrue, FloatArr &tHit0True, FloatArr &tHit1True,
		    int &nHitVis, V2fArr &hitVis,  FloatArr &tHit0Vis,  FloatArr &tHit1Vis)
{		    
 nHitVis = 0;  hitVis.allocate (nHitTrue);
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
//       hitVis  [nHitVis]   = hitTrue[i];
       tHit0Vis[nHitVis]   = tHit0True[i];
       tHit1Vis[nHitVis++] = tHit1True[i];
      }
    }
}

/******************************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
  int  	    i;
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
      case 'j': JUSTVIEWING=1; 					break;
      case 'n': NEWFILE=0; 					break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }  
  
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
  int mark = infile.tellg();  			// count the contours
  while (!tryToGetRightBrace(infile))		// look for ending brace
   {
    getLeftBrace(infile);  infile >> id;  assert (id == "CONTOUR");
    skipToMatchingRightBrace(infile); n++;
   }
  assert(n>0);  Array<Contour> ctrData(n);
  infile.seekg(mark);
  for (i=0; i<n; i++)
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
// obstacle[i].print();
   }
   
  /**************************DUALIZING**********************************/

  if (!JUSTVIEWING)			
   {
    obdual.allocate(n);
    obdualreflex.allocate(n);
//    obdualselfreflex.allocate(n);
    for (i=0; i<n; i++)  
     {
      obdual[i].dualOf (obstacle[i]);
      obdual[i].prepareDisplay (nPtsPerSegment*30);
      obdualreflex[i].reflect (obdual[i]);	// reflect dual about origin
      obdualreflex[i].prepareDisplay (nPtsPerSegment*3);
//      obdualselfreflex[i].reflectx (obdual[i]); // reflect about x-axis
//      obdualselfreflex[i].prepareDisplay (nPtsPerSegment*30);
     }
     
    /***************compute intersections of duals************************/

if (0==1)
 {
    int nHitOrig, nHitReflex;	V2fArr hitOrig, hitReflex;
    FloatArr tHitOrig0, tHitOrig1, tHitReflex0, tHitReflex1;

    obdual[0].intersect (obdual[1], nHitOrig, hitOrig, tHitOrig0, 
    			 tHitOrig1,.0001);
cout << nHitOrig << " intersections of duals" << endl;
if (0==1)
 {
    obdual[0].intersect (obdualreflex[1], nHitReflex, hitReflex, tHitReflex0, 
    			 tHitReflex1, .0001);
cout << nHitReflex << " intersections of duals (reflex)" << endl;			 

    int nHitMate;  V2fArr hitMate;	// intersections with exact mate,
    FloatArr tHit0Mate, tHit1Mate;	// collecting from original and reflex
    nHitMate = nHitOrig + nHitReflex;	hitMate.allocate (nHitMate);  
    tHit0Mate.allocate(nHitMate);  tHit1Mate.allocate(nHitMate);
    for (i=0; i<nHitOrig;  i++)  
     { 
      hitMate[i] = hitOrig[i];  
      tHit0Mate[i] = tHitOrig0[i]; 
      tHit1Mate[i] = tHitOrig1[i];
     }
//    float firstknot = obdualreflex[1].getKnot(0);
//    float lastknot = obdualreflex[1].getKnot(obdualreflex[1].getnKnot()-1);
    for (i=0; i<nHitReflex; i++) 
     {
      hitMate[i+nHitOrig] = hitReflex[i]; 
      tHit0Mate[i+nHitOrig] = tHitReflex0[i];
      tHit1Mate[i+nHitOrig] = tHitReflex1[i];
//      tHit1Mate[i+nHitOrig] = lastknot - (tHitReflex1[i] - firstknot);	
      			// flip the knot on the reflection
     }
    
cout << nHitMate << " intersections: " << endl;
for (i=0; i<nHitMate; i++)
  cout << "(" << hitMate[i][0] << "," << hitMate[i][1] << ") with parameter value on obstacle[0] of " 
       << tHit0Mate[i] << " and obstacle[1] of " << tHit1Mate[i] << endl;

    // consider all combinations of intersection parameter values,
    // one from dual[0], other from dual[1]
    nHit = nHitMate * nHitMate;	hit.allocate(0);
    tHit0.allocate (nHit);	tHit1.allocate (nHit);
    for (i=0; i<nHitMate; i++)
      for (int j=0; j<nHitMate; j++)
       {
	tHit0[i*nHitMate + j] = tHit0Mate[i];
	tHit1[i*nHitMate + j] = tHit1Mate[j];
       }
       
    /***************compute self-intersections of duals************************/

    obdual[0].selfIntersect (nHitSelf0, hitSelf0, tHitSelf00, tHitSelf01, .0001);
 cout << nHitSelf0 << " self-intersections of dual[0]" << endl;
    obdual[1].selfIntersect (nHitSelf1, hitSelf1, tHitSelf10, tHitSelf11, .0001);
 cout << nHitSelf1 << " self-intersections of dual[1]" << endl;
//    nHitSelf0 = nHitSelf1 = 0;
    
    hodo0.createHodograph (obstacle[0]);
    hodo1.createHodograph (obstacle[1]);

    /************ remove common tangent mistakes **************/

    cullMistakes (nHit,     hit,     tHit0,     tHit1, 
    		  nHitTrue, hitTrue, tHit0True, tHit1True, 0);
    cullMistakes (nHitSelf0, hitSelf0, tHitSelf00, tHitSelf01,
    		  nHitSelf0True, hitSelf0True, tHitSelf00True, tHitSelf01True, 1);
    cullMistakes (nHitSelf1, hitSelf1, tHitSelf10, tHitSelf11,
    		  nHitSelf1True, hitSelf1True, tHitSelf10True, tHitSelf11True, 1);
    
    /************ remove invisible common tangents **************/
    
    cullInvisible (nHitTrue, hitTrue, tHit0True, tHit1True, 
    		   nHitVis,  hitVis,  tHit0Vis,  tHit1Vis);
} // if (0==1)
} // if (0==1)

if (1==1) {   
    hodo0.createHodograph (obstacle[0]);
    hodo1.createHodograph (obstacle[1]);
}
    tActive = obstacle[0].getKnot(0);		// start at beginning
    tDelta = obstacle[0].getKnot (obstacle[0].getnKnot()-1) / 8000.; 	// 2000.

    // store common tangents as obstacle index/parameter value quartets
    // V-graph G = (V,E): V = {A,B} + points of common tangency 
    // (between 2 obstacles, the same obstacle, and A/B and obstacle);
    // E = common tangents + curved segments between vertices on the same obstacle
    // (not including segments between vertices that are already connected by a tangent)
    // cout << "Finished V-graph" << endl;
 
   }

  if (NEWFILE) 			// output V-graph
   {
    string vGraphFile = filePrefix + ".vgraph";
    outfile.open(vGraphFile.c_str());
    outfile << "{ " << vGraphFile << " }\n";
    // write polygonal obstacles out (repeat input) and then topology of Vgraph
   }

  //  through mouse controls outside main:
  //    input source and destination through mouse; compute shortest path
      
  /************************************************************/

  glutInit (&argc, argv);
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
  glutAddMenuEntry ("One obstacle only", 7);
  glutAddMenuEntry ("Tangents associated w. all intersections", 3);
  glutAddMenuEntry ("Common tangents", 4);
  glutAddMenuEntry ("Visible common tangents", 5);
  glutAddMenuEntry ("All tangents between single curve", 8);
  glutAddMenuEntry ("Common tangents between single curve", 9);
  glutAddMenuEntry ("Tangent [t]", 2);
  glutAddMenuEntry ("Dual line [d]", 10);
  glutAddMenuEntry ("Spin tangent on first curve [r]", 1);
  glutAddMenuEntry ("Reverse direction of spin [b]", 6);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (724,30);
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

