/*
  File:          convexhull.cpp
  Author:        J.K. Johnstone 
  Created:	 19 November 2001
  Last Modified: 18 September 2002
  Purpose:       Compute the smooth convex hull of a curve.
		 Builds on tangentialCurve.c++, software for building the
		 tangential curve of a Bezier curve,
		 and extracts from selfbitang.c++.
  Sequence: 	 3rd in a sequence (interpolate, tangCurve, convexhull)
  Input: 	 a 2d polygon, implicitly defining an interpolating cubic 
  		 Bezier curve
  History: 	 9/16/02:  Added WINDOWS/PRINTOUT modes.
  		 9/18/02:  Added labels to bitangents.
		 	   Built starting point robustly.
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
#define WINDOWS 0		// 0 for running on SGI, 1 for Windows

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 30)" << endl;
  cout << "\t[-p] (set to Postscript printing mode; default is screen display)" << endl;
  cout << "\t[-e eps] (accuracy at which intersections are made: default .0001)" << endl;
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
static GLboolean DRAWHIT=1;		// draw dual self-intersections?
static GLboolean DRAWBITANG=0;		// draw self-bitangents?
static GLboolean LABELBITANG=0;	// number the bitangents?
static GLboolean DRAWCONVEXHULL=1;	// draw convex hull?
static GLboolean DRAWCURVE=1;		// draw input curve?
static GLboolean DRAWSTARTPT=0;		// draw starting point?
static GLboolean DRAWBEG=0;		// draw beginning point C(0)?
static GLboolean rotateOb=0;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'counterclockwise' direction?

Array<BezierCurve2f> 	obstacle;	// interpolating cubic Bezier curves
Array<TangentialCurve>	obduala;	// associated tangential a-curves
Array<TangentialCurve>	obdualb;	// associated tangential b-curves
Array<CommonTangentArr> bitangA;	// self-bitangents from a-space for each curve
Array<CommonTangentArr> bitangB;	// self-bitangents from b-space for each curve
V2fArr			bitang;		// self-bitangents, as parameter pairs
FloatArr		sortBitangEndpt;// sorted array of bitangent endpoints
float 			startPt;	// starting point's parameter value
V2fArr 			convHull;	// representation of convex hull = active parameter intervals
IntArr			bitangConv;	// bitangents on convex hull (indices from bitang)
IntArr 			active;		// active segments of convex hull on obstacle[0], after subdivideSpline
float colour[7][3] = {{1,0,0}, {0,0,1}, {0,1,0}, {0,0,0}, {1,0,1}, {0,1,1}, {1,1,0}};
BezierCurve2f	 	hodo0;		// hodograph of 1st obstacle, for interactive tangent display
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
int			obstacleWin;	// primal window identifier 
int			dualWin;	// a-dual window identifier
int			dualWin2;	// b-dual window identifier
int 			xsize, ysize;	// window size
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
  case 'c': 	DRAWCONVEXHULL = !DRAWCONVEXHULL; break;
  case 'i': 	DRAWCURVE = !DRAWCURVE; 	break;
  case '1':	DRAWBITANG = !DRAWBITANG;	break;
  case 's':	DRAWSTARTPT = !DRAWSTARTPT;	break;
  case 'B':     DRAWBITANG = !DRAWBITANG;	break;
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
  case 6:	DRAWBITANG = !DRAWBITANG;	break;
  case 7:	LABELBITANG = !LABELBITANG;	break;
  case 10:	DRAWCONVEXHULL = !DRAWCONVEXHULL; break;
  case 11: 	DRAWCURVE = !DRAWCURVE; 	break;
  case 12:	DRAWSTARTPT = !DRAWSTARTPT;	break;
  case 13: 	DRAWBEG = !DRAWBEG;		break;
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
  case 2:     DRAWHIT = !DRAWHIT;				break;
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
  glTranslatef (transxob, transyob, 0);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  int nOb = obstacle.getn(); 
  if (DRAWCURVE)
   {
    for (i=0; i<nOb; i++)
     {
      glColor3fv (colour[i%7]);
      obstacle[i].draw(); 
     }
   }
  if (DRAWTANG)
   {
    glColor3fv (Red);
    obstacle[0].drawTangent (tActive, hodo0, 1);
    if (WINDOWS) { V2f p; obstacle[0].eval (tActive,p); drawPt(p[0],p[1],.05); }
    else 		  obstacle[0].drawPt(tActive);
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
  if (DRAWBITANG)
   {
    glColor3fv (Black);
    for (i=0; i<bitang.getn(); i++)
     {
      V2f pt1, pt2;
      obstacle[0].eval (bitang[i][0], pt1);
      obstacle[0].eval (bitang[i][1], pt2);
      glBegin (GL_LINES);
      glVertex2f (pt1[0], pt1[1]);
      glVertex2f (pt2[0], pt2[1]);
      glEnd();
     }
   }
  if (LABELBITANG)
   {
    glColor3fv (Black);
    for (i=0; i<sortBitangEndpt.getn(); i++)
     {
      V2f p;	obstacle[0].eval (sortBitangEndpt[i], p);
      glRasterPos2f (p[0]+.01, p[1]+.01);
      char str[10];  itoa (i, str);
      for (int k=0; k<strlen(str); k++)
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
     }
   }
  if (DRAWCONVEXHULL)
   {
//    thicker
    glColor3fv (Blue);
    obstacle[0].drawActive (active, nPtsPerSegment);
    for (i=0; i<bitangConv.getn(); i++)
     {
      V2f pt1, pt2;
      obstacle[0].eval (bitang[bitangConv[i]][0], pt1);
      obstacle[0].eval (bitang[bitangConv[i]][1], pt2);
      glBegin (GL_LINES);
      glVertex2f (pt1[0], pt1[1]);
      glVertex2f (pt2[0], pt2[1]);
      glEnd();      
//    back to normal thickness
     }
   }
  if (DRAWSTARTPT)
   {
    glColor3fv (Red);
    V2f p; obstacle[0].eval (startPt,p); 
    if (WINDOWS) drawPt(p[0],p[1],.05);
    else 	 obstacle[0].drawPt (startPt);
    glRasterPos2f (p[0]+.06, p[1]+.01);
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'P');
   }
  if (DRAWBEG) 
   {
    glColor3fv (Black);
    V2f p;	obstacle[0].eval (obstacle[0].getKnot(0),p);
    if (WINDOWS) drawPt(p[0],p[1],.05);
    else 	 obstacle[0].drawPt (obstacle[0].getKnot(0));
    glRasterPos2f (p[0]+.06, p[1]+.01);
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'b');
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
    glColor3fv (Red); 
    if (WINDOWS) { V2f p; obduala[0].eval (tActive,p); drawPt(p[0],p[1],.05); }
    else 		  obduala[0].drawPt(tActive); 
   }
  if (DRAWHIT)				// intersections in a-space
   {
    glColor3fv (Black);
    int nOb = obstacle.getn(); 
    glBegin(GL_POINTS);
    for (i=0; i<nOb; i++)
      for (j=0; j<bitangA[i].getn(); j++)
        if (WINDOWS) { V2f p; obduala[i].eval (bitangA[i][j].param1,p); drawPt(p[0],p[1],.05); }
        else 		      obduala[i].drawPt (bitangA[i][j].param1);
    glEnd();
   }

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
    glColor3fv (Red);
    if (WINDOWS) { V2f p; obdualb[0].eval (tActive,p); drawPt(p[0],p[1],.05); }
    else 		  obdualb[0].drawPt(tActive); 
   }
  if (DRAWHIT)				// intersections in b-space
   {
    glColor3fv (Black);
    glBegin(GL_POINTS);
    int nOb = obstacle.getn(); 
    for (i=0; i<nOb; i++)
      for (j=0; j<bitangB[i].getn(); j++)
        if (WINDOWS) { V2f p; obdualb[i].eval (bitangB[i][j].param1,p); drawPt(p[0],p[1],.05); }
        else 		      obdualb[i].drawPt (bitangB[i][j].param1);
    glEnd();
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
    obstacle[i].fitClosed (Pt[i]);
}

/******************************************************************************
	Compute clipped tangential a/b-curves.
******************************************************************************/

void buildTangentialCurves (Array<BezierCurve2f> &obstacle)
{
cout << "Building" << endl;
  obduala.allocate(obstacle.getn());  obdualb.allocate(obstacle.getn());
  for (int i=0; i<obstacle.getn(); i++)	
   {
    obduala[i].createA(obstacle[i], i);
    obduala[i].prepareDisplay (nPtsPerSegment*5);
    obdualb[i].createB(obstacle[i], i);
    obdualb[i].prepareDisplay (nPtsPerSegment*5);
   }
cout << "Finished building" << endl;   
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  float     eps = .0001;	// accuracy of intersection computation
  int 	    i,j;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'p': PRINTOUT = 1;					break;
      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  inputCurves(argv[argc-1], obstacle);
  buildTangentialCurves (obstacle);
  hodo0.createHodograph (obstacle[0]);	// for spinning tangent
  int nOb = obstacle.getn();   // self-intersect tangential curves to find bitangents
  bitangA.allocate(nOb);  bitangB.allocate(nOb);
cout << "Intersecting" << endl;  
  for (i=0; i<nOb; i++)
   {
    obduala[i].selfIntersect (bitangA[i], eps);
    obdualb[i].selfIntersect (bitangB[i], eps);
   }
cout << "Finished intersecting" << endl;   
  
  assert(nOb==1);	// convex hull of single curve, for now
  // combine bitangents from a-space and b-space, and at same time 
  //	extract parameter pairs from bitangents, into V2f array
  bitang.allocate(bitangA[0].getn() + bitangB[0].getn());
  for (i=0; i<bitangA[0].getn(); i++)
   {
    bitang[i][0] = bitangA[0][i].param1;
    bitang[i][1] = bitangA[0][i].param2;
   }
  for (j=0; j<bitangB[0].getn(); i++,j++)
   {
    bitang[i][0] = bitangB[0][j].param1;
    bitang[i][1] = bitangB[0][j].param2;
   }
  // NOTE: implicitly assuming that bitang[i][0] < bitang[i][1]
  // this is true of parameter pairs generated by selfIntersect
  
cout << "Unsorted bitangents" << endl;  
for (i=0; i<bitang.getn(); i++) cout << bitang[i][0] << " " << bitang[i][1] << endl;

  // sort by first value in parameter pair
  FloatArr paramFirst(bitang.getn());
  for (i=0; i<bitang.getn(); i++) paramFirst[i] = bitang[i][0];
  IntArr sortIndex;  paramFirst.bubbleSort (sortIndex);
  V2fArr oldbitang(bitang);	// rearrange bitang array in sorted order
  for (i=0; i<bitang.getn(); i++)	bitang[i] = oldbitang[sortIndex[i]];

cout << "Sorted bitangents" << endl;  
for (i=0; i<bitang.getn(); i++) cout << bitang[i][0] << " " << bitang[i][1] << endl;

  /* find a point on convex hull to start sweep
  *  idea: if c(0) is not a point on the convex hull, then we can find one by 
  *  sweeping forward on the curve until a bitangent endpoint is found that 
  *  lies on the convex hull,
  *  since the entrance to the convex hull is protected by bitangents
  *  (1) is c(0) on convex hull? (i.e., does tangent at c(0) intersect curve?)
  *  (2) if so, advance to the end of the next bitangent (so that we don't start
  *	 in the middle of a convex hull segment);
  *      if not, consider bitangents starting after t=0, until you find one
  * 	 on the convex hull
  */

  convHull.create(bitang.getn()+1);  // max size is # of bitangents (see loop below)
  if (bitang.getn() == 0)		// convexhull is entire curve
   {
    convHull[0][0] = obstacle[0].getKnot(0);
    convHull[0][1] = obstacle[0].getLastKnot();
    bitangConv.create(0);
   }
  else
   {
    bitangConv.create(convHull.getn());
    startPt = obstacle[0].getKnot(0);	// start at the beginning
    V2fArr ptOfTang(2);
    obstacle[0].eval (startPt, ptOfTang[0]);
    ptOfTang[1] = ptOfTang[0]; 
    int nextend = 0;	// next endpoint to consider in sorted list
    
    sortBitangEndpt.allocate(2*bitang.getn());	// sort the bitangent endpoints
    IntArr sortIndex;	    // original position of endpoint in sortBitangEndpt
    for (i=0; i<bitang.getn(); i++)
     {
      sortBitangEndpt[2*i]   = bitang[i][0];
      sortBitangEndpt[2*i+1] = bitang[i][1];
     }
    sortBitangEndpt.bubbleSort (sortIndex);

    while (!obstacle[0].onConvexHull (startPt, ptOfTang, eps))
     {
      int nextBitang = sortIndex[nextend] / 2;	// advance to next bitangent endpoint
      int nextIndex  = sortIndex[nextend] % 2;
      startPt = bitang[nextBitang][nextIndex];
//    startPt = bitang[nextBitang][1]; // advance to end of next bitangent
      obstacle[0].eval (bitang[nextBitang][0], ptOfTang[0]);
      obstacle[0].eval (bitang[nextBitang][1], ptOfTang[1]);
      nextend++;
        // NOTE: if tritangent or more is possible,
        // need to add more points of tangency: tough in present implementation
     }
     
    // now move forward to the middle of this curve segment
    if (nextend==0)	// start point is beginning of curve
     {
      if (startPt == sortBitangEndpt[0])	// starting at an endpoint
       {
        float foo = (sortBitangEndpt[0] + sortBitangEndpt[1]) / 2.;
	obstacle[0].eval (foo, ptOfTang[0]);
	ptOfTang[1] = ptOfTang[0];
	if (obstacle[0].onConvexHull (foo, ptOfTang, eps))
	     startPt = foo;
	else startPt = (sortBitangEndpt[2*bitang.getn()-1] + 
			      sortBitangEndpt[0]) / 2.;
       }
     }
    else startPt = (sortBitangEndpt[nextend-1] +
    		    sortBitangEndpt[nextend]) / 2.;

cout << "The starting point of our sweep, on the convex hull, is c("
     << startPt << ") = ";
ptOfTang[0].print();
cout << endl;
  
    /* now sweep from this point, gathering segments on convex hull
     *  and jumping to next segment via bitangents
     *	consider each bitangent in turn, in sorted order
     *	[is this a bitangent of the convex hull? (does it intersect curve?)
     *		no need to ask question: it *will* be inherently]
     *	jump to next segment
     *	continue until back to starting point of sweep */
     
    // find bitangent endpoint immediately after starting point
    int I=0;	// index of this endpoint
    while (I < sortBitangEndpt.getn() && startPt > sortBitangEndpt[I]) I++;
    if (I == sortBitangEndpt.getn()) I = 0;
    int nCh = 0;	// index of convex hull segment under consideration
    int startFound = 0;	// flag: have we looped back to starting point?
    do {
        // add free bitang assoc with I
      bitangConv[nCh] = sortIndex[I] / 2;	
      float buddyPar = bitang[sortIndex[I] / 2][(sortIndex[I]+1)%2];
      		// parameter value of buddy (other end of bitangent)
cout << "next free bitangent: " << bitang[sortIndex[I] / 2][0] << ","
				<< bitang[sortIndex[I] / 2][1] << endl;		

        // search forward for buddy in sorted list
      int buddyI=(I+1)%sortBitangEndpt.getn();		
      while (sortBitangEndpt[buddyI] != buddyPar) 
        buddyI = (buddyI + 1)%sortBitangEndpt.getn();
cout << "buddyI = " << buddyI << endl;	

        // add curve segment from other end of this bitangent to beginning of next
      convHull[nCh][0] = sortBitangEndpt[buddyI];
      convHull[nCh][1] = sortBitangEndpt[(buddyI+1)%sortBitangEndpt.getn()];
cout << "next conv hull seg : " << convHull[nCh][0] << "," 
 				<< convHull[nCh][1] << endl;      

        // update index of free bitangent
      I = (buddyI+1)%sortBitangEndpt.getn();
      
      	// is the starting point on the newly added segment?
      if (convHull[nCh][0] < convHull[nCh][1])
       {
        if (startPt > convHull[nCh][0] && startPt < convHull[nCh][1])
	  startFound = 1;
       }
      else		// wraparound
        if (startPt > convHull[nCh][0] || startPt < convHull[nCh][1])
	  startFound = 1;
	  
      nCh++;
    } while (!startFound);
    if (nCh < convHull.getn())
     {
      // shrink convHull to correct size (will often have empty slots at end)
      V2fArr oldconvhull(convHull);	IntArr oldbitangconv(bitangConv);
      convHull.create(nCh);		bitangConv.create(nCh);
      for (i=0; i<nCh; i++)  convHull[i] = oldconvhull[i];
      for (i=0; i<nCh; i++)  bitangConv[i] = oldbitangconv[i];
     } 
   }
   
/************* OLD VERSION  
    convHull[0][0] = startPt;
    int nCh = 0;	// index of convex hull segment under consideration
    int nBt = 0; 	// index of bitangent under consideration
    while (nCh==0 || convHull[nCh][0] != startPt)	// haven't looped around yet
     {
      // starting after present sweep position
      while (nBt < bitang.getn() && bitang[nBt][0] < convHull[nCh][0])	nBt++;
      if (nBt == bitang.getn())  	// if last segment straddles t=0, 
       {	// it will be a convex segment stopped by beg. of 1st bitangent
        convHull[nCh][1] = bitang[0][0];
	bitangConv[nCh] = 0;
	convHull[nCh+1][0] = bitang[0][1];	// this will signal end next time thru loop
       }
      else
       { 
        // stop active interval at beginning of next bitangent 
        convHull[nCh][1] = bitang[nBt][0];
	bitangConv[nCh] = nBt;	// bitangent after this c.h. segment is nBt
        // start new active interval at end of this bitangent
        convHull[nCh+1][0] = bitang[nBt][1];
       }
// cout << "next convex hull segment: " << convHull[nCh][0] << "," 
// 				        << convHull[nCh][1] << endl;      
      nCh++;
     }
    if (nCh < convHull.getn())
     {
      // shrink convHull to correct size (will often have empty slots at end)
      V2fArr oldconvhull(convHull);	IntArr oldbitangconv(bitangConv);
      convHull.create(nCh);		bitangConv.create(nCh);
      for (i=0; i<nCh; i++)  convHull[i] = oldconvhull[i];
      for (i=0; i<nCh; i++)  bitangConv[i] = oldbitangconv[i];
     } 
   }
***************/
   
cout << "Convex hull = " << endl;
for (i=0; i<convHull.getn(); i++)  convHull[i].print(); 
cout << endl;
  
  FloatArr paramCH(2*convHull.getn());	// param values of convex hull endpts
  for (i=0,j=0; i<convHull.getn(); i++)  
   { 
    paramCH[j++] = convHull[i][0];
    paramCH[j++] = convHull[i][1];
   }
  paramCH.bubbleSort(sortIndex);	// needed if convex hull wraps around
  FloatArr knot(obstacle[0].getnKnot());
  for (i=0; i<obstacle[0].getnKnot(); i++) knot[i] = obstacle[0].getKnot(i);
  FloatArr paramCHfilter;		// with knots filtered out
  paramCH.filterOut (knot, paramCHfilter, 0);
  obstacle[0].subdivideSpline(paramCHfilter);
// obstacle[0].print();
  
  // identify intervals of spline that correspond to convex hull
  active.create(obstacle[0].getL());
  knot.create(obstacle[0].getnKnot());
  for (i=0; i<obstacle[0].getnKnot(); i++) knot[i] = obstacle[0].getKnot(i);
  for (i=0; i<obstacle[0].getL(); i++)
   {
    int done = active[i] = 0;
    for (j=0; !done && j<convHull.getn()-1; j++)
      if (knot[i] >= convHull[j][0] && // ith curve seg contained in jth conv hull seg
      	  knot[i] < convHull[j][1])	
        done = active[i] = 1;
   }
  j = convHull.getn()-1;	// last convex hull segment is special
  if (convHull[j][0] > convHull[j][1])	// wraparound in last convex hull segment
    for (i=0; i<obstacle[0].getL(); i++)
     {			
      if (knot[i] >= convHull[j][0] || knot[i] < convHull[j][1])
        active[i] = 1;
     }
  else
    for (i=0; i<obstacle[0].getL(); i++)
      if (knot[i] >= convHull[j][0] && knot[i] < convHull[j][1])
        active[i] = 1;
// cout << active << endl;
  
  for (i=0; i<obstacle.getn(); i++)	// wait until after subdivideSpline
    obstacle[i].prepareDisplay (nPtsPerSegment);
  tActive = obstacle[0].getKnot(0);	// start at beginning
  if (WINDOWS) tDelta = obstacle[0].getKnot (obstacle[0].getnKnot()-1) / 2000.;
  else	       tDelta = obstacle[0].getKnot (obstacle[0].getnKnot()-1) / 8000.;

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int xleft;		// x-coord of lefthand side
  int barmargin; 	// width of side bar surrounding picture
  int titleht; 		// top titlebar height
  if (WINDOWS)
   {
    xleft 	= 0;
    xsize 	= 500; ysize = 500;
    barmargin 	= 6;
    titleht 	= 12;
   }
  else
   {
    xleft 	= 0;			// 164 for small windows
    if (PRINTOUT) xsize = ysize = 250;  // 350 for small windows (less reduction required ==> better image clarity)
    else	  xsize = ysize = 600;  // 600 for standard windows
    barmargin 	= 8;
    titleht 	= 20;
   }
  int halfysize = (ysize - 2*titleht)/2;
  int dualxleft = xleft+xsize+2*barmargin-1;
  int adualy    = titleht+10;
  int bdualy    = titleht+10+halfysize+2*titleht+1;

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
  glutAddMenuEntry ("Convex hull [c]", 				10);
  glutAddMenuEntry ("Tangent at active point of 1st curve [t]", 1);
  glutAddMenuEntry ("Spin tangent on 1st curve [r]", 		2);
  glutAddMenuEntry ("Reverse direction of spin [b]", 		3);
  glutAddMenuEntry ("Dual line of active point on a-dual curve (as test of dualization back to primal space) [d]", 4);
  glutAddMenuEntry ("Line field on 1st curve [l]",		5);
  glutAddMenuEntry ("Self-bitangents [B]", 	    		6);
  glutAddMenuEntry ("Number the self-bitangents",		7);
  glutAddMenuEntry ("Input curve [i]", 				11);
  glutAddMenuEntry ("Starting point [s]",			12);
  glutAddMenuEntry ("Beginning of curve",			13);
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
  glutAddMenuEntry ("Dual self-intersections", 		2);
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
  glutAddMenuEntry ("Dual self-intersections", 		2);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

