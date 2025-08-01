/*
  File:          kernelseed.cpp
  Author:        J.K. Johnstone 
  Created:	 7 January 2003 (from kernel.cpp)
  Last Modified: 7 January 2003
  Purpose:       Compute a seed point of the kernel of the given curve,
  		 using tangential curve system.
  Sequence: 	 3rd in a sequence (interpolate, tangentialCurve, kernelseed, kernel)
  Input: 	 a 2d polygon, implicitly defining an interpolating cubic 
  		 Bezier curve
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
#include "Miscellany.h"		// drawImplicitLine, screen2world
#include "Vector.h"		
#include "MiscVector.h"		
#include "BezierCurve.h"	// drawTangent, drawPt
#include "TangCurve.h"		// create; evalProj, drawCtrlPoly, drawPt (from RatBezierCurve inheritance)

#define PTSPERBEZSEGMENT 30     // # pts to draw on each Bezier segment
#define WINDOWS 0		// 0 for running on SGI, 1 for Windows
#define EPS .001		// default accuracy for inflection points and intersection

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 30)" << endl;
  cout << "\t[-p] (set to Postscript printing mode; default is screen display)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t[-x tx] (translation amount in x)" << endl;
  cout << "\t[-e accuracy of inflection and intersection] (default: .001)" << endl;
  cout << "\t[-o] (open curve)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWTANG=0;		// draw tangent at active point?
static GLboolean DRAWFIELD=0;		// draw line field (all tangents)?
static GLboolean DRAWDUALTANG=0;	// draw tangent in dual space at active point?
static GLboolean DRAWDUALOFACTIVEPT=0;	// draw dual of active point?
static GLboolean DRAWDUALOFMOUSE=0;	// draw dual of mouse point?
static GLboolean DRAWINFL=0;		// draw inflection points?
static GLboolean DRAWINFLTANG=0;	// draw inflection tangents?
static GLboolean DRAWINFLHIT=0;		// draw intersections of inflection tangents?
static GLboolean DRAWCURVEHIT=0;	// draw intersections of infl tang w. curve?
static GLboolean DRAWKERHIT=0;		// draw intersections on kernel?
static GLboolean DRAWCTRL=0;		// draw control polygon?
static GLboolean DRAWORIGIN=0;		// draw origin?
static GLboolean DRAWTANGORIG=0;	// draw tangents through origin?
static GLboolean DRAWALLDUAL=0;		// draw entire unclipped tangential c-curve?
static GLboolean DRAWMARKKER=0;		// draw kernel points, marked as hitInfl or hitCurve?
static GLboolean CLOSED=1;		// closed input curve?
static GLboolean rotateOb=0;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'counterclockwise' direction?

BezierCurve2f 		obstacle;	// interpolating cubic Bezier curve
FloatArr 		tTangThruOrigin;// params of tangents through origin
TangentialCurve		obduala,obdualb;// associated tangential a,b-curves
FloatArr		tInfl;		// param values of inflection pts on obstacle
V2fArr 			hitInfl;	// intersections of the inflection bitangents
V2fArr 			hitCurve;	// intersections of infl tangents w. curve
V2fArr 			hitKernel;	// intersections representing kernel boundary
int 			nKernelFromInfl;// # of kernel pts from hitInfl
V2f			kernelMean(0,0);// mean of finite kernel pts
V2f			kernelCentroid;	// centroid of finite kernel pts
IntArr 			kernelSortIndex;// rearrangement of hitKernel from sorting
BezierCurve2f		kernelCurve;	// original curve with only kernel segments active
IntArr			activeKernel;	// active segments of kernelCurve
V2fArr 			hitCurveParameterPair;	// parameter values of kernel points associated with curve
		  				// only used for convex hull
float colour[7][3]    = {{1,0,0}, {0,0,1}, {0,1,0}, {0,0,0}, {1,0,1}, {0,1,1}, {1,1,0}};
float greyscale[7][3] = {{0,0,0}, {.1,.1,.1}, {.2,.2,.2}, {.3,.3,.3}, {.4,.4,.4}, {.5,.5,.5}, {.6,.6,.6}};
BezierCurve2f	 	hodo0;		// hodograph of 1st obstacle, for interactive tangent display
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
int			obstacleWin;	// primal window identifier 
int			dualWin;	// a-dual window identifier
int			dualWin2;	// b-dual window identifier
int 			xsize, ysize;	// window size
float 			xMouse=1, yMouse=0; // world coordinates of mouse
int       		nPtsPerSegment = PTSPERBEZSEGMENT;
int 			PRINTOUT=0;	// 0 for displaying on screen, 1 for printing out image
float 			xTranslate=0;

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
  zoomdual = .5;
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
// cout << tActive << endl;
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
  if (DRAWDUALOFMOUSE)
    screen2world (x, y, xsize, ysize, 2, zoomob, transxob, transyob, xMouse, yMouse);
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
  case 'k':	DRAWDUALOFMOUSE = !DRAWDUALOFMOUSE; break;
  case 'l': 	DRAWFIELD = !DRAWFIELD;		break;
  case '4':  	DRAWCTRL = !DRAWCTRL;		break;
  case '7': 	DRAWINFLTANG = !DRAWINFLTANG;	break;
  case '8': 	DRAWINFLHIT = !DRAWINFLHIT;	break;
  case '9':	DRAWCURVEHIT = !DRAWCURVEHIT;	break;
  case '0':	DRAWKERHIT = !DRAWKERHIT;	break;
  case 'T':	DRAWTANGORIG = !DRAWTANGORIG;	break;
  case 'o':	DRAWORIGIN = !DRAWORIGIN;	break;
  case 'm':	DRAWMARKKER = !DRAWMARKKER;	break;
  case 'i':	DRAWINFL = !DRAWINFL;		break;
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
  case 5:	DRAWFIELD = !DRAWFIELD;		break;		// line field
  case 6: 	DRAWINFLTANG = !DRAWINFLTANG;	break;		// inflection pts
  case 7:  	DRAWCTRL = !DRAWCTRL;		break;		// control polygon
  case 8:	DRAWINFLHIT = !DRAWINFLHIT;	break;
  case 9:	DRAWCURVEHIT = !DRAWCURVEHIT;	break;
  case 10:	DRAWKERHIT = !DRAWKERHIT;	break;
  case 11:	DRAWORIGIN = !DRAWORIGIN;	break;
  case 12: 	DRAWTANGORIG = !DRAWTANGORIG;	break;
  case 15:	DRAWMARKKER = !DRAWMARKKER;	break;
  case 17:	DRAWINFL = !DRAWINFL;		break;
  default:   					break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1:     DRAWDUALTANG = !DRAWDUALTANG;			break;
  case 2:     DRAWDUALOFACTIVEPT = !DRAWDUALOFACTIVEPT;		break;
  case 3:     DRAWDUALOFMOUSE = !DRAWDUALOFMOUSE;		break;
  case 4:     DRAWALLDUAL = !DRAWALLDUAL;			break;
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
  
  if (DRAWORIGIN)
   {
    glColor3fv (Black);
    if (WINDOWS)  drawPt (0,0); 
    else { glBegin (GL_POINTS); glVertex2f (0,0); glEnd(); }
   }

//  if (PRINTOUT) glColor3fv (greyscale[0]); else 
  glColor3fv (colour[0]);
  obstacle.draw();
  
  if (DRAWTANGORIG)
   {
    glColor3fv (Black);
    for (i=0; i<tTangThruOrigin.getn(); i++)
     { 
      obstacle.drawPt (tTangThruOrigin[i]); 
      obstacle.drawTangent (tTangThruOrigin[i], hodo0, 1);
     }
   }
  if (DRAWCTRL)
   {
    glColor3fv (Black);
    obstacle.drawCtrlPoly();
   }
  if (DRAWTANG)
   {
//    if (PRINTOUT) glColor3fv (Black); else 
    glColor3fv (Red);
    obstacle.drawTangent (tActive, hodo0, 1);
    obstacle.drawPt      (tActive);
   }
  if (DRAWFIELD)
   {
    glColor3fv (Black);
    float last = obstacle.getKnot( obstacle.getnKnot()-1 );
    for (float t=obstacle.getKnot(0); t<last; t += .01)
      obstacle.drawTangent (t, hodo0, 0);
   }
  if (DRAWDUALOFMOUSE)
   {
    glColor3fv(Black);
    if (WINDOWS)  drawPt (xMouse, yMouse);
    else { glBegin (GL_POINTS); glVertex2f (xMouse, yMouse); glEnd(); }
   }
  if (DRAWINFL)
   {
    glColor3fv(Black);
    for (i=0; i<tInfl.getn(); i++)    obstacle.drawPt (tInfl[i]); 
   }
  if (DRAWINFLTANG)
   {
    glColor3fv(Black);
    for (i=0; i<tInfl.getn(); i++)    obstacle.drawTangent (tInfl[i], hodo0, 1); 
   }
  if (DRAWINFLHIT)
   {
//    if (PRINTOUT) glColor3fv (Black); else
    glColor3fv (Green);
    for (i=0; i<hitInfl.getn(); i++)
      if (WINDOWS)  drawPt (hitInfl[i][0], hitInfl[i][1]);
      else { glBegin (GL_POINTS); glVertex2f (hitInfl[i][0], hitInfl[i][1]); glEnd(); }
   }
  if (DRAWCURVEHIT)
   {
//    if (PRINTOUT) glColor3fv (Black); else
    glColor3fv (Blue);
    for (i=0; i<hitCurve.getn(); i++)
      if (WINDOWS)  drawPt (hitCurve[i][0], hitCurve[i][1]);
      else { glBegin (GL_POINTS); glVertex2f (hitCurve[i][0], hitCurve[i][1]); glEnd(); }
   }
  if (DRAWKERHIT)
   {
//    if (PRINTOUT) glColor3fv (Black); else
    glColor3fv (Magenta);
    for (i=0; i<hitKernel.getn(); i++)
      if (WINDOWS)  drawPt (hitKernel[i][0], hitKernel[i][1]);
      else { glBegin (GL_POINTS); glVertex2f (hitKernel[i][0], hitKernel[i][1]); glEnd(); }
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

  glColor3fv (colour[0]);
  obduala.drawT (nPtsPerSegment*5);
  if (DRAWALLDUAL)
   {
    glColor3fv (colour[0]);
    obduala.drawEntire();
   }
  if (DRAWTANG) 
   { 
    glColor3fv (Red); 
    if (WINDOWS) { V2f p; obduala.eval(tActive,p); drawPt(p[0],p[1],.05); }
    else 		  obduala.drawPt(tActive); 
   }
  if (DRAWDUALTANG)
   {
    glColor3fv (Red); 
    obduala.drawTangent (tActive);
    V2f p; obduala.eval(tActive,p);		// label point Q
    glRasterPos2f (p[0]+.15, p[1]); 		
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'C');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '*');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '(');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 't');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ')');
   }
  if (DRAWDUALOFACTIVEPT)		// hypothesis: to show that duality does not preserve tangency
  				// is this dual a tangent?
   {
    glColor3fv (Black);
    V2f p; obstacle.eval (tActive, p);
    drawImplicitLine (1,p[1],p[0]);
   }
  if (DRAWDUALOFMOUSE)
   {
    glColor3fv (Black);
    drawImplicitLine (1,yMouse,xMouse);
   }
  if (DRAWINFLTANG)		// draw duals of inflection tangents
   {
    glColor3fv (Black);
    for (i=0; i<tInfl.getn(); i++) obduala.drawPt (tInfl[i]);
   }
  if (DRAWINFLHIT)
   {
    glColor3fv (Green);
    for (i=0; i<hitInfl.getn(); i++)
      drawImplicitLine (1, hitInfl[i][1], hitInfl[i][0]);
   }
  if (DRAWCURVEHIT)
   {
    glColor3fv (Blue);
    for (i=0; i<hitCurve.getn(); i++)
      drawImplicitLine (1, hitCurve[i][1], hitCurve[i][0]);
   }
  if (DRAWKERHIT)
   {
    glColor3fv (Magenta);
    for (i=0; i<hitKernel.getn(); i++)
      drawImplicitLine (1, hitKernel[i][1], hitKernel[i][0]);
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

  glColor3fv (colour[0]);
  obdualb.drawT (nPtsPerSegment*5);
  if (DRAWALLDUAL)
   {
    glColor3fv (colour[0]);
    obdualb.drawEntire();
   }
  if (DRAWTANG) 
   { 
    glColor3fv (Red); 
    if (WINDOWS) { V2f p; obdualb.eval(tActive,p); drawPt(p[0],p[1],.05); }
    else 		  obdualb.drawPt(tActive); 
   }
  if (DRAWDUALTANG)
   {
    glColor3fv (Red); 
    obdualb.drawTangent (tActive);
    V2f p; obdualb.eval(tActive,p);		// label point Q
    glRasterPos2f (p[0]+.15, p[1]); 		
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'C');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '*');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, '(');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 't');
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ')');
   }
  if (DRAWDUALOFACTIVEPT)		// hypothesis: to show that duality does not preserve tangency
  				// is this dual a tangent?
   {
    glColor3fv (Black);
    V2f p; obstacle.eval (tActive, p);
    drawImplicitLine (p[0],1,p[1]);
   }
  if (DRAWDUALOFMOUSE)
   {
    glColor3fv (Black);
    drawImplicitLine (xMouse,1,yMouse);
   }
  if (DRAWINFLTANG)		// draw duals of inflection tangents
   {
    glColor3fv (Black);
    for (i=0; i<tInfl.getn(); i++) obdualb.drawPt (tInfl[i]);
   }
  if (DRAWINFLHIT)
   {
    glColor3fv (Green);
    for (i=0; i<hitInfl.getn(); i++)
      drawImplicitLine (hitInfl[i][0], 1, hitInfl[i][1]);
   }
  if (DRAWCURVEHIT)
   {
    glColor3fv (Blue);
    for (i=0; i<hitCurve.getn(); i++)
      drawImplicitLine (hitCurve[i][0], 1, hitCurve[i][1]);
   }
  if (DRAWKERHIT)
   {
    glColor3fv (Magenta);
    for (i=0; i<hitKernel.getn(); i++)
      drawImplicitLine (hitKernel[i][0], 1, hitKernel[i][1]);
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

void inputCurves (char *file, BezierCurve2f &obstacle)
{
  int i,j;
  ifstream infile;  infile.open(file);  
  V2fArrArr Pt;		// data points, organized into polygons
  read (infile, Pt);
  scaleToUnitSquare (Pt);
  for (i=0; i<Pt.getn(); i++)
    for (j=0; j<Pt[i].getn(); j++)
      Pt[i][j][0] += xTranslate;
  if (CLOSED)  obstacle.fitClosed (Pt[0]);
  else         obstacle.fit 	  (Pt[0]);
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int i,j;
  int ArgsParsed=0;
  float eps = EPS;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'p': PRINTOUT = 1;					break;
      case 'x': xTranslate = atof(argv[ArgsParsed++]);		break;
      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'o': CLOSED=0;					break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  inputCurves(argv[argc-1], obstacle);
  hodo0.createHodograph (obstacle);	// for spinning tangent
//  V2f origin(0,0);			// compute tangs thru origin for display
//  tangThruPt (obstacle, origin, tTangThruOrigin, .0001); 
  	// apparent chicken and egg problem: need tangential curve to compute tTangThruOrigin
	// and worse, pts of tangency are inherently unstably computed infinite pts
	//	solution: use tangential curve system of ACM SE conference
  obduala.createA (obstacle, 0);	// compute tangential a-curve
  obdualb.createB (obstacle, 0);	// compute tangential b-curve
  obstacle.prepareDisplay (nPtsPerSegment);	// postpone, since obstacle changes in creating obdualc
  obduala.prepareDisplay (nPtsPerSegment*5);
  obdualb.prepareDisplay (nPtsPerSegment*5);
  obstacle.inflectionPt (tInfl, eps);	// compute inflection points
  
  /******* compute intersections of inflection tangents ********/
  	// these are duals of type 2 cusp bitangents

  int nInfl = tInfl.getn(), nHit = 0;
  V2fArr hitInflTemp((nInfl-1)*nInfl/2);	
  V2fArr linePair ((nInfl-1)*nInfl/2);	// linePair[i] = parameters of 2 points in dual space that define hitInfl[i] line
  for (i=0; i<nInfl; i++)
   {
    Line2f inflTang1;		// tangent at tInfl[i]
    obstacle.buildTangent (tInfl[i], hodo0, 1, inflTang1);
    for (j=i+1; j<nInfl; j++)
     {
      Line2f inflTang2;		// tangent at tInfl[j]
      obstacle.buildTangent (tInfl[j], hodo0, 1, inflTang2);
      float foo; 
      if (inflTang1.intersect (inflTang2, hitInflTemp[nHit], foo, 0, eps))
       {	// not parallel
        linePair[nHit][0]   = tInfl[i]; 
	linePair[nHit++][1] = tInfl[j];
       }
     }
   }
  hitInfl.create (hitInflTemp, nHit);	// some lines may be parallel
   
  /******* compute intersections of inflection tangents w. curve ********/
  	// these are duals of type 1 cusp bitangents

  hitCurve.allocate(0);
  for (i=0; i<nInfl; i++)
   {
    BezierCurve2f inflTang;		// tangent at tInfl[i]
    obstacle.buildTangent (tInfl[i], hodo0, 1, inflTang);
    int nFoo;  V2fArr hit;  FloatArr tFoo, btFoo;
    obstacle.intersect (inflTang, nFoo, hit, tFoo, btFoo, eps);
// cout << hit.getn() << " intersections of inflection tangent with curve" << endl;    
    hitCurve += hit;
   }
  V2fArr parameterPair (hitCurve.getn());  // parameterPair[i] = parameters of 2 points in dual space that define hitCurve[i] line
  nHit=0;
  for (i=0; i<nInfl; i++)
   {
    BezierCurve2f inflTang;		// tangent at tInfl[i]
    obstacle.buildTangent (tInfl[i], hodo0, 1, inflTang);
    int nFoo;  V2fArr hit;  FloatArr tFoo, btFoo;
    obstacle.intersect (inflTang, nFoo, hit, tFoo, btFoo, eps);		// wish we didn't have to compute again, but can't see how not
    for (j=0; j<hit.getn(); j++)
     {
      parameterPair[nHit][0]   = tInfl[i];
      parameterPair[nHit++][1] = tFoo[j];
     }
   }
  
  tActive = obstacle.getKnot(0);	// start at beginning
  if (WINDOWS) tDelta = obstacle.getKnot (obstacle.getnKnot()-1) / 2000.;
  else	       tDelta = obstacle.getKnot (obstacle.getnKnot()-1) / 8000.;

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
  strcpy (titlebar, "Kernel");  
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
  glutAddMenuEntry ("Control polygon",				7);
  glutAddMenuEntry ("Origin [o]",				11);
  glutAddMenuEntry ("Tangent at active point of 1st curve [t]", 1);
  glutAddMenuEntry ("Spin tangent on 1st curve [r]", 		2);
  glutAddMenuEntry ("Reverse direction of spin [b]", 		3);
  glutAddMenuEntry ("Dual line of active point on a-dual curve (as test of dualization back to primal space) [d]", 4);
  glutAddMenuEntry ("Line field on 1st curve [l]",		5);
  glutAddMenuEntry ("Tangents through origin [T]",	       	12);
  glutAddMenuEntry ("Inflection points [i]",			17);
  glutAddMenuEntry ("Inflection tangents [7]",			6);
  glutAddMenuEntry ("Intersections of inflection tangents [8]", 8);
  glutAddMenuEntry ("Intersections with curve [9]",		9);
  glutAddMenuEntry ("Kernel boundary [0]",			10);
  glutAddMenuEntry ("Marked kernel boundary [m]",		15);
  glutAddMenuEntry ("Clipped out segments [c]",			13);
  glutAddMenuEntry ("Centroid of kernel points [C]", 		14);
  glutAddMenuEntry ("Kernel [K]",				16);
  glutAddMenuEntry ("Trim region [R]", 				18);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft,adualy);		// a-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential a-curves");
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
  glutAddMenuEntry ("Tangent at active point",		2);
  glutAddMenuEntry ("Dual line of active point", 	3);
  glutAddMenuEntry ("Dual line of mouse point", 	4);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft, bdualy);		// b-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential b-curves");
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
  glutAddMenuEntry ("Tangent at active point",		2);
  glutAddMenuEntry ("Dual line of active point", 	3);
  glutAddMenuEntry ("Dual line of mouse point", 	4);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
