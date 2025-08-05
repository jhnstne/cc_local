/*
  File:          objectvis.cpp
  Author:        J.K. Johnstone 
  Created:	 27 September 2005 (from 2d/tangthrupt/src/poletang.cpp)
  Last Modified: 9 February 2006
  Purpose:       To analyze the visibility of a certain curve 
                 from a certain viewpoint in a scene of curves.
  Sequence:	 interpolate -> tangCurve -> poletang -> objectvis
  Input: 	 A scene of closed curves, a distinguished curve, and a viewpoint.
                 A closed curve and its interior defines an object.
		 Each curve is defined by an ordered list of points, which imply
		 a closed interpolating cubic B-spline.
		 The distinguished curve is defined by its index in the scene.
                 The viewpoint must lie outside any object.
		 It is controllable by the mouse.
  Paper: 	 'Can you see it? Object visibility in a smooth flat scene'
                 J.K. Johnstone (2005)
  History: 	 November 2005: touchups for paper submission
                 2/9/06: expository mode added
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

#include "basic/AllColor.h"
#include "basic/Miscellany.h"		
#include "basic/Vector.h"		
#include "basic/MiscVector.h"		
#include "curve/BezierCurve.h"	
#include "tangcurve/TangCurve.h" // BitangentArr, intersect, draw, visible
#include "basic2/Line.h"         // createRayInsideSquare

#define PTSPERBEZSEGMENT 30      // # pts to draw on each Bezier segment
#define WINDOWS 0		 // 0 for running on Linux, 1 for Windows

// functions to implement
void buildTangSegThruView();   // done
int isExtreme (TangThruPt &t); // done
void buildProbe (float angle, V2f source, Line2f &probe, float radiusRoom); // done
void buildProbe (V2f from, V2f towards, BezierCurve2f &probe, float radiusRoom); // done
void buildProbeThruView (Array<TangThruPtArr> &tangThruView, 
			 Array<BezierCurve2fArr> &probeThruView, 
			 float radiusRoom); // done
int firstPtHit (BezierCurve2f &ob, BezierCurve2f &probe, V2f &ptFirstHit, 
		float &tFirstHit, float radiusRoom, float epsInt);
int firstPtHit (BezierCurve2f &ob, BezierCurve2f &probe, V2f &ptFirstHit, 
		float &tFirstHit, V2f ptToIgnore, float featureSize,
		float radiusRoom, float epsInt);
int firstObjectHit (Array<BezierCurve2f> &object, BezierCurve2f &probe,
		    V2f &ptFirstHit, float radiusRoom, float epsInt); // done
int firstObjectHit (Array<BezierCurve2f> &object, BezierCurve2f &probe, V2f &ptFirstHit, 
		    V2f ptToIgnore, int objToIgnore, float featureSize,
		    float radiusRoom, float epsInt);
int ptLiesInConcavity (V2f &pt, int ob, int concavity);
int blockedBySingleOb (int star, int ob);
int ptInConcavity (V2f Q, BezierCurve2f &B, V2f conInterval, V2f viewpt, 
		   float epsInt, float radiusRoom);
int objInConcavity (BezierCurve2f &A, TangThruPt extreme1, TangThruPt extreme2, 
		    V2f angRangeA, 
		    BezierCurve2f &B, V2f angRangeCon, V2f intervalCon,
		    V2f viewpt, float epsInt, float radiusRoom);
int objLiesBehind (BezierCurve2f &A, V2f angRangeA, 
		   BezierCurve2f &B, V2f angRangeB, V2fArr conInterval,
		   V2f viewpt, float epsInt, float radiusRoom);
int backtrack (BezierCurve2f &probe, BezierCurve2fArr &obstacle, int star, 
	       TangThruPtArrArr &tangThruView, V2iArr &extreme, IntArrArr &piercing,
	       int piercingInBacktrack,
	       int newB, int oldB, int firstTime, V2f ptTang, V2fArr &angExtreme,
	       BezierCurve2fArr &probeAlg, IntArr &probeOb, V2fArr &probePt, 
	       V2fArr &probePtA, int &nProbe, BezierCurve2f &witness, V2f &range,
	       V2f viewpt, float epsInt, float radiusRoom);
int isStarVisible (int star, BezierCurve2fArr &obstacle, TangThruPtArrArr &tangThruView, 
		   V2iArr &extreme, IntArrArr &piercing, V2fArrArr &conInterval,
		   V2fArr &angRange, V2fArrArr &angRangeCon, V2fArr &angExtreme,
		   BezierCurve2fArr &probeAlg, IntArr &probeOb, V2fArr &probePt, 
		   V2fArr &probePtA, BezierCurve2f &witness, V2f &range, int reversePass,
		   V2f viewpt, float epsInt, float radiusRoom);
// scene
Array<BezierCurve2f>   obstacle;     // scene objects = interpolating cubic Bezier curves
int                    star=0;       // index of distinguished object
V2f		       viewpt(-2,0); // point through which to compute tangents
float                  radiusRoom=2; // radius of room enclosing the scene (for clipping)
// dual of scene, to find tangents through viewpoint
Array<TangentialCurve> obduala;	     // associated tangential a-curves
Array<TangentialCurve> obdualb;	     // associated tangential b-curves
TangentialCurve	       viewptduala;  // a-dual of viewpoint
TangentialCurve	       viewptdualb;  // b-dual of viewpoint
// tangents through viewpoint
Array<TangThruPtArr>    tangThruView;    // viewpoint tangents for each curve
Array<TangThruPt>       alltang;         // all viewpoint tangents (for expository access)
Array<IntArr>           vistang;         // is this viewpoint tangent visible?
Array<BezierCurve2fArr> tangSegThruView; // viewpoint tangents clipped to room
                                         // it is clipped at both ends, whereas a probe
                                         // is clipped at only one end, since the other
                                         // end is the viewpoint (since probe is a ray)
Array<BezierCurve2fArr> probeThruView;   // probes for viewpoint tangents clipped to room
// special tangents
V2iArr                 extreme;      // indices of extreme tangents, 2 per object
IntArrArr              piercing;     // piercing[i] = indices of piercing tangents
                                     // for obstacle[i], each of which represents 
                                     // a concavity
Array<PiercingTangArr> piercingTang; // also store them directly per object
                                     // since we need the piercing intersections
Array<TangThruPtArrArr> extremeCon;  // extreme tangents per concavity per object
// parameter and angular ranges
V2fArrArr              conInterval;  // parameter interval of each concavity
Array<V2fArrArr>       conSample;    // display pts for each concavity's bdry, for each object
V2fArr                 angExtreme;   // angle of extreme tangents;
                                     // angle is expressed in radians over [0,2PI)
FloatArrArr            angPiercing;  // angle of piercing tangents
V2fArr                 angRange;     // angular range of each object
V2fArrArr              angRangeCon;  // angular range of each concavity
V2f                    range;        // present angular range of star known to be covered
// probes
Array<V2fArr>          probeHitAll;  // probe intersections for all viewpoint tangents, 
                                     // organized by object
Array<IntArr>          probeHitAllObj; // ...[i] = object that probeHitAll[i] lies on
int                    nProbe;       // # of probes in probing pass that tests visibility
BezierCurve2fArr       probeAlg;     // probes (in order of consideration in algorithm)
Line2fArr              probeLine;    // probeAlg, as lines
IntArr                 probeOb;      // probed objects (in order of consideration) 
                                     // = chain of objects that block the star 
V2fArr                 probePt;      // endpoints of probe (probePt[i] lies on probeOb[i])
V2fArr                 probePtA;     // first probed point on A (for each probe)
Line2f                 witness;      // witness to visibility (if one exists)
int                    starIsVisible;// is the prescribed object visible?
int                    level=11;     // how many levels to compute?
float                  epsIntersect; // accuracy of intersection in primal space
float                  epsAngle;     // radian increment in angle between consecutive rays
                                     // in angular range display
float                  featureSize;  // the smallest recognizable feature;
                                     // points closer than this are indistinguishable;
                                     // used to filter intersections near pt of tangency
/******************************************************************************/
// GUI variables
static GLfloat   transxob, transyob, zoomob, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,           // first MOUSEX (MOUSEY) reading?
                 firsty=1;
static int	 oldx,oldy;	     // previous value of MOUSEX and MOUSEY
static GLboolean DRAWBOLD=0;	     // draw star in bold red and others in black?
static GLboolean DRAWTANG=0;	     // draw all tangents through viewpoint?
static GLboolean DRAWVISTANG=0;	     // draw visible tangents through viewpoint?
static GLboolean DRAWALLPROBE=0;     // draw all probes?
static GLboolean DRAWEXTREME=0;      // draw extreme tangents?
static GLboolean DRAWPIERCE=0;       // draw piercing tangents?
static GLboolean DRAWEXPROBE=0;      // draw all extreme/piercing probes?
static GLboolean DRAWCONCAVITY=0;    // draw concavities?
static GLboolean DRAWOBJRANGE=0;     // draw object angular ranges?
static GLboolean DRAWEXTREMEINCONCAVITY=0; // draw extreme pts in star's concavity?
static GLboolean DRAWCONEXTREME=0;   // draw extreme tangents of concavity?
static GLboolean DRAWCONRANGE=0;     // draw concavity angular ranges?
static GLboolean DRAWOBJINCON=0;     // draw objects that lie in star's concavity?
static GLboolean DRAWOBJBEHIND=0;    // draw objects that lie behind the star?
static GLboolean DRAWVISIBILITY=0;   // visualize visibility?
static GLboolean DRAWPROBE=0;        // draw all probes in probing pass?
static GLboolean DRAWVIRTUAL=0;      // draw virtual vertices?
static GLboolean DRAWVIEW=1;	     // draw viewpoint?
static GLboolean DRAWPIERPROBE=0;    // draw piercing tangent probes?
static GLboolean DRAWEXTRPROBE=0;    // draw extreme tangent probes?
static GLboolean DRAWEXTRSTAR=0;     // draw extreme tangents of star only?
static GLboolean LABELOBJECT=0;      // label the objects?
static GLboolean LABELTANG=0;        // label the tangents?
static GLboolean DRAWOBJ=1;          // draw the objects?
static GLboolean GREYSCALE=0;        // no colours, for print images
static GLboolean PRINTOUT=0;         // 0: displaying on screen, 1: printing out image
static GLboolean LAPTOP=0;           // running on laptop (with larger screen)?
static GLboolean EXPOSITORY=0;       // expository mode?
                                     // in this mode, the display loop reads from an action file
                                     // to step through the display of tangents and objects
                                     // in a way conducive to a discussion of the principles
                                     // or a simulation of the algorithm
int              nPtsPerSegment = PTSPERBEZSEGMENT;
float            colour[7][3]    = {{1,0,0}, {0,0,1}, {0,1,0}, {0,0,0}, {1,0,1}, 
				    {0,1,1}, {1,1,0}};
float            greyscale[7][3] = {{0,0,0}, {.1,.1,.1}, {.2,.2,.2}, {.3,.3,.3}, 
				    {.4,.4,.4}, {.5,.5,.5}, {.6,.6,.6}};
int              obstacleWin;	     // window identifier 
static char *RoutineName;
static void usage()
 {
  cout << "Analyze the visibility from a viewpoint of a certain curve in a scene "
       << " of curves." << endl;
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-L] (laptop)" << endl;
  cout << "\t[-d display density of Bezier segment] (default: 30)" << endl;
  cout << "\t[-E eps] (accuracy at which intersections are made in dual space" << endl
       << "\t \t to compute tangents through the viewpoint: default .0001)" << endl;
  cout << "\t[-e epsIntersect] (accuracy at which intersections are made in primal space"
       << ": default .001)" << endl;
  cout << "\t[-F featureSize] (points within this distance are indistinguishable:"
       << " default .05)" << endl;
  cout << "\t[-s #] (index of distinguished object, the star: default 0)" << endl;
  cout << "\t[-v x y] (viewpoint: default (-2,0))" << endl;
  cout << "\t[-r #] (radius of bounding room: default 2)" << endl;
  cout << "\t[-l #] (level up to which we should compute: default 11)" << endl;
  cout << "\t[-z #] (zoom factor: default 1)" << endl;
  cout << "\t[-R] (reverse the probe pass direction)" << endl;
  cout << "\t[-p] (use first piercing tangent in backtracking: a hack)" << endl;
  cout << "\t[-t] (expository or teaching mode)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

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

void visibility (int status)
{
  if (status != GLUT_VISIBLE)
    glutIdleFunc (NULL);
  /*  else if (rotateOb)
      glutIdleFunc (RotateOb); */
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
  case 27:   exit(1); 			                     break; // ESCAPE
  case '1':  DRAWTANG       = !DRAWTANG;                     break;
  case '2':  DRAWALLPROBE   = !DRAWALLPROBE;                 break;
  case '3':  DRAWEXTREME    = !DRAWEXTREME;                  break;    
  case '4':  DRAWOBJRANGE   = !DRAWOBJRANGE;                 break;
  case '5':  DRAWPIERCE     = !DRAWPIERCE;                   break;    
  case '6':  DRAWCONCAVITY  = !DRAWCONCAVITY;                break;    
  case '7':  DRAWEXTREMEINCONCAVITY=!DRAWEXTREMEINCONCAVITY; break;
  case '8':  DRAWCONEXTREME = !DRAWCONEXTREME;               break;
  case '9':  DRAWCONRANGE   = !DRAWCONRANGE;                 break;
  case '0':  DRAWOBJINCON   = !DRAWOBJINCON;                 break;
  case '-':  DRAWOBJBEHIND  = !DRAWOBJBEHIND;                break;
  case '=':  DRAWVISIBILITY = !DRAWVISIBILITY;               break;
  case '\'': DRAWPROBE      = !DRAWPROBE;                    break;
  default:   			                             break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 0:  DRAWBOLD       = !DRAWBOLD;                     break;
  case 1:  DRAWTANG       = !DRAWTANG;                     break;
  case 2:  DRAWALLPROBE   = !DRAWALLPROBE;                 break;
  case 3:  DRAWEXTREME    = !DRAWEXTREME;                  break;    
  case 4:  DRAWOBJRANGE   = !DRAWOBJRANGE;                 break;
  case 5:  DRAWPIERCE     = !DRAWPIERCE;                   break;    
  case 6:  DRAWCONCAVITY  = !DRAWCONCAVITY;                break;    
  case 7:  DRAWEXTREMEINCONCAVITY=!DRAWEXTREMEINCONCAVITY; break;
  case 8:  DRAWCONEXTREME = !DRAWCONEXTREME;               break;
  case 9:  DRAWCONRANGE   = !DRAWCONRANGE;                 break;
  case 10: DRAWOBJINCON   = !DRAWOBJINCON;                 break;
  case 11: DRAWOBJBEHIND  = !DRAWOBJBEHIND;                break;
  case 12: DRAWVISIBILITY = !DRAWVISIBILITY;               break;
  case 13: DRAWPROBE      = !DRAWPROBE;                    break;
  case 14: DRAWVIRTUAL    = !DRAWVIRTUAL;                  break;
  case 15: GREYSCALE      = !GREYSCALE;                    break;
  case 16: DRAWEXPROBE    = !DRAWEXPROBE;                  break;
  case 17: DRAWVIEW       = !DRAWVIEW;                     break;
  case 18: DRAWPIERPROBE  = !DRAWPIERPROBE;                break;
  case 19: DRAWEXTRPROBE  = !DRAWEXTRPROBE;                break;
  case 20: DRAWEXTRSTAR   = !DRAWEXTRSTAR;                 break;    
  case 21: LABELOBJECT    = !LABELOBJECT;                  break;
  case 22: LABELTANG      = !LABELTANG;                    break;
  case 23: DRAWOBJ        = !DRAWOBJ;                      break;
  default:   			                           break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j,k;
  char str[10];
  Line2f probe; 
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  int nOb = obstacle.getn();   V2f foo; int c=1; // colour index for non-stars
  if (LABELOBJECT)
    {
      glColor3fv (Blue);
      for (i=0; i<obstacle.getn(); i++)
	{
	  V2f cpt; obstacle[i].getCtrlPt(0, cpt);
	  glRasterPos2f (cpt[0], cpt[1]);
	  //	  glutBitmapCharacter(GLUT_BITMAP_9_BY_15, 'C');
	  itoa (i, str);
	  for (k=0; k<strlen(str); k++)
	    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
	}
    }
  if (LABELTANG)
    {
      glColor3fv (Black);
      for (i=0; i<alltang.getn(); i++)
	{
	  glRasterPos2f (alltang[i].ptTang[0], alltang[i].ptTang[1]);
	  itoa (i,str);
	  for (k=0; k<strlen(str); k++)
	    glutBitmapCharacter (GLUT_BITMAP_9_BY_15, str[k]);
	}
    }
  if (DRAWVIEW)
    {
      glColor3fv (Black); glBegin(GL_POINTS); glVertex2f(viewpt[0],viewpt[1]); glEnd();
    }
  if (DRAWOBJ)
    {
      if (DRAWBOLD)
	{
	  glLineWidth(3.0); if (GREYSCALE) glColor3fv(Black); else glColor3fv (Red);
	  obstacle[star].draw();
	  glLineWidth(1.0); glColor3fv (Black);
	  for (i=0; i<nOb; i++) if (i!=star) obstacle[i].draw();
	}
      else
	{
	  for (i=0; i<nOb; i++)
	    {
	      if (GREYSCALE) glColor3fv(Black);
	      else if (i==star) glColor3fv (Red); else { glColor3fv(colour[c]); c = (c+1)%7; }
	      obstacle[i].draw();
	    }
	}
    }
  if (DRAWTANG)
   {
    glColor3fv (Black);
    for (i=0; i<nOb; i++)
      for (j=0; j<tangThruView[i].getn(); j++)
  	tangThruView[i][j].draw();
   }
  if (DRAWVISTANG)
   {
    glColor3fv (Black);
    for (i=0; i<nOb; i++)
      for (j=0; j<tangThruView[i].getn(); j++)
	if (vistang[i][j]) tangThruView[i][j].draw();
   }
  if (DRAWALLPROBE)
    {
      glColor3fv (Black);
      glBegin(GL_POINTS);
      for (i=0; i<nOb; i++)
	for (j=0; j<probeHitAll[i].getn(); j++)
         if (probeHitAllObj[i][j] != -1)
	    glVertex2f (probeHitAll[i][j][0], probeHitAll[i][j][1]);
      glEnd();
    }
  if (DRAWEXTREME)
   {
    glColor3fv (Blue);
    for (i=0; i<nOb; i++)
      for (j=0; j<2; j++)
	{
	  tangThruView[i][extreme[i][j]].draw();
	  /*
	  glBegin(GL_POINTS);
	  glVertex2f (tangThruView[i][extreme[i][j]].ptTang[0],
		      tangThruView[i][extreme[i][j]].ptTang[1]);
	  glEnd();
	  */
	}
   }
  if (DRAWEXTRSTAR)  // draw extreme tangents of star only
   {
    glColor3fv (Black);
    for (j=0; j<2; j++)
       tangThruView[star][extreme[star][j]].draw();
   }
  if (DRAWEXTRPROBE)
   {
    glColor3fv (Black);
    glBegin(GL_LINES);
    for (i=0; i<nOb; i++)
      for (j=0; j<2; j++)
	{
	  glVertex2f (viewpt[0], viewpt[1]);
	  V2f ptRoom; probeThruView[i][extreme[i][j]].getCtrlPt (1,ptRoom);
	  glVertex2f (ptRoom[0], ptRoom[1]);
	}
    glEnd();
   }
  if (DRAWPIERCE)
   {
    glColor3fv (Red);
    for (i=0; i<nOb; i++)
      for (j=0; j<piercing[i].getn(); j++)
       {
	tangThruView[i][piercing[i][j]].draw();
	/*
	 glBegin(GL_POINTS); 
	 glVertex2f (piercingTang[i][j].ptPierce[0], piercingTang[i][j].ptPierce[1]);
	 glEnd();
	*/
       }
   }
  if (DRAWPIERPROBE)
   {
    glColor3fv (Red);
    glBegin(GL_LINES);
    for (i=0; i<nOb; i++)
      for (j=0; j<piercing[i].getn(); j++)
	{
	  glVertex2f (viewpt[0], viewpt[1]);
	  glVertex2f (piercingTang[i][j].ptPierce[0], piercingTang[i][j].ptPierce[1]);
	}
    glEnd();
   }
  if (DRAWEXPROBE)
    {
      glColor3fv (Black);
      glBegin(GL_POINTS);
      for (i=0; i<nOb; i++)
	{
	  for (j=0; j<2; j++) // extreme probes
	    {
	      int k=extreme[i][j];
	      if (probeHitAllObj[i][k] != -1)
		glVertex2f (probeHitAll[i][k][0], probeHitAll[i][k][1]);
	    }
	  for (j=0; j<piercing[i].getn(); j++) // piercing probes
	    {
	      int k=piercing[i][j];
	      if (probeHitAllObj[i][k] != -1)
		glVertex2f (probeHitAll[i][k][0], probeHitAll[i][k][1]);
	    }
	}
      glEnd();
    }
  if (DRAWCONCAVITY)
    {
      glLineWidth (7.0);
      glColor3fv (Red);
      for (i=0; i<conInterval.getn(); i++)
	for (j=0; j<conInterval[i].getn(); j++)
	  {
	    // draw curve segment defined by conInterval[i][j]
	    glBegin(GL_LINE_STRIP);
	    for (int k=0; k<conSample[i][j].getn(); k++)
	      glVertex2f (conSample[i][j][k][0], conSample[i][j][k][1]);
	    glEnd();
	    // draw line segment closing off the concavity
	    glBegin(GL_LINES);
	    glVertex2f (piercingTang[i][j].ptTang[0],   piercingTang[i][j].ptTang[1]);
	    glVertex2f (piercingTang[i][j].ptPierce[0], piercingTang[i][j].ptPierce[1]);
	    glEnd();
	  }
      glLineWidth (1.0);
    }
  if (DRAWOBJRANGE)
    {
      glColor3fv (Black);
      for (i=star; i<star+1; i++)
	{
	  // draw rays across angular range of object[i]
	  float angle;
	  if (angRange[i][0] < angRange[i][1])
	    for (angle = angRange[i][0]; angle < angRange[i][1]; angle += epsAngle)
	      {
		buildProbe (angle, viewpt, probe, radiusRoom);
		probe.draw();
	      }
	  else
	    {
	      for (angle = angRange[i][0]; angle < 2*M_PI; angle += epsAngle)
		{
		  buildProbe (angle, viewpt, probe, radiusRoom);
		  probe.draw();
		}
	      for (angle = 0; angle < angRange[i][1]; angle += epsAngle)
		{
		  buildProbe (angle, viewpt, probe, radiusRoom);
		  probe.draw();
		}
	    }
	  buildProbe (angRange[i][1], viewpt, probe, radiusRoom);
	  probe.draw();
	}
    }
  if (DRAWEXTREMEINCONCAVITY) // draw extreme points that lie in concavity of star
    {                         // as evidence that concavity test works
      glColor3fv (Black);
      glBegin(GL_POINTS);
      for (i=0; i<nOb; i++)
	if (i!=star)
	for (j=0; j<2; j++)  // for each extreme point
	  for (int k=0; k<piercingTang[star].getn(); k++) // for each concavity of star
	    {
	      cout << "Testing extreme tangent " << j << " of object " << i << endl;
	      if (ptInConcavity (tangThruView[i][extreme[i][j]].ptTang,
			       obstacle[star], conInterval[star][k], 
			       viewpt, epsIntersect, radiusRoom))
	      glVertex2f (tangThruView[i][extreme[i][j]].ptTang[0],
			  tangThruView[i][extreme[i][j]].ptTang[1]);
	    }
      glEnd();
    }
/*
  glColor3fv (Black);
  for (i=0; i<nOb; i++)  // sanity check the extreme tangent angles
    for (j=0; j<2; j++)
      {
	buildProbe (angExtreme[i][j], viewpt, probe, radiusRoom);
	probe.draw();
      }
*/
  if (DRAWCONEXTREME) // draw extreme tangents of each concavity
    {
      glColor3fv (Blue);
      for (i=0; i<nOb; i++)
	for (j=0; j<piercingTang[i].getn(); j++)
	  {
	    extremeCon[i][j][0].draw(); 
	    extremeCon[i][j][1].draw();
	  }
    }
  if (DRAWCONRANGE) // draw angular range of the concavities
   {
    glColor3fv (Black);
    for (i=0; i<nOb; i++)
      for (j=0; j<piercingTang[i].getn(); j++)
       {
	// draw rays across angular range of concavity
	 float angle;
	 if (angRangeCon[i][j][0] < angRangeCon[i][j][1])
	   for (angle=angRangeCon[i][j][0]; angle<angRangeCon[i][j][1]; angle += epsAngle)
	      {
		buildProbe (angle, viewpt, probe, radiusRoom);
		probe.draw();
	      }
	  else
	    {
	      for (angle = angRangeCon[i][j][0]; angle < 2*M_PI; angle += epsAngle)
		{
		  buildProbe (angle, viewpt, probe, radiusRoom);
		  probe.draw();
		}
	      for (angle = 0; angle < angRangeCon[i][j][1]; angle += epsAngle)
		{
		  buildProbe (angle, viewpt, probe, radiusRoom);
		  probe.draw();
		}
	    }
	  buildProbe (angRangeCon[i][j][1], viewpt, probe, radiusRoom);
	  probe.draw();
       }
   }
  if (DRAWOBJINCON) // draw objects that lie in the star's concavity
    {
      glLineWidth(3.0);
      glColor3fv (Orange);
      for (i=0; i<nOb; i++)
       for (j=0; j<piercingTang[star].getn(); j++)
	if (objInConcavity (obstacle[i], 
			    tangThruView[i][extreme[i][0]],
			    tangThruView[i][extreme[i][1]], angRange[i],
			    obstacle[star], angRangeCon[star][j],
			    conInterval[star][j], viewpt, epsIntersect, radiusRoom))
	  obstacle[i].draw();
      glLineWidth(1.0);
    }
  if (DRAWOBJBEHIND) // draw objects that lie behind the star
    {
      glLineWidth(3.0);
      glColor3fv (DodgerBlue);
      for (i=0; i<nOb; i++)
	if (objLiesBehind (obstacle[i],    angRange[i],
			   obstacle[star], angRange[star],
			   conInterval[star], viewpt, epsIntersect, radiusRoom))
	  obstacle[i].draw();
      glLineWidth(1.0);
    }
  if (DRAWVISIBILITY) // visualize the visibility decision
    {
      glLineWidth(5.0);
      glColor3fv (MidnightBlue);
      if (starIsVisible) witness.draw();
      else obstacle[star].draw(); 
      glLineWidth(1.0);
    }
  if (DRAWPROBE)  // draw all probes explored during probing pass
    {
      glColor3fv (Red);
      for (i=0; i<nProbe; i++)
	probeLine[i].draw();
    }
  if (DRAWVIRTUAL) // draw points of tangency of tangents through viewpoint (virtual vertices)
    {
      glColor3fv (Black);
      glBegin(GL_POINTS);
      for (i=0; i<nOb; i++)
	for (j=0; j<tangThruView[i].getn(); j++)
	  glVertex2f (tangThruView[i][j].ptTang[0], tangThruView[i][j].ptTang[1]);
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
   { 
    obstacle[i].fitClosed (Pt[i]);
    obstacle[i].prepareDisplay (nPtsPerSegment);
   }
}

/******************************************************************************
	Compute clipped tangential a/b-curves.
******************************************************************************/

void buildTangentialCurves (Array<BezierCurve2f> &obstacle)
{
  obduala.allocate(obstacle.getn());  obdualb.allocate(obstacle.getn());
  for (int i=0; i<obstacle.getn(); i++)	
   {
    obduala[i].createA(obstacle[i], i);
    obduala[i].prepareDisplay (nPtsPerSegment*5);
    obdualb[i].createB(obstacle[i], i);
    obdualb[i].prepareDisplay (nPtsPerSegment*5);
   }
}

/******************************************************************************
    Clip tangents through viewpoint to the room.

    -->tangThruView:    tangents through viewpoint
    <--tangSegThruView: clipped tangents through viewpoint
    -->radiusRoom:      radius of bounding room (needed to clip the infinite tangent)
******************************************************************************/

void buildTangSegThruView (Array<TangThruPtArr> &tangThruView, 
			   Array<BezierCurve2fArr> &tangSegThruView, 
			   float radiusRoom)
{
  V2f ptfoo, ptbar;   float tfoo, tbar;
  int nOb = tangThruView.getn();
  tangSegThruView.allocate(nOb);
  for (int i=0; i<nOb; i++)
    {
      tangSegThruView[i].allocate(tangThruView[i].getn());
      // build segment of tangent inside the room: it can effectively replace the tangent
      for (int j=0; j<tangThruView[i].getn(); j++)
	tangThruView[i][j].createTangInsideSquare (radiusRoom, tangSegThruView[i][j], 
						   ptfoo, ptbar, tfoo, tbar);
    }
}

/******************************************************************************
    Given an angle, build the ray segment from a source point at this angle,
    clipped to the room.

    -->angle:      angle of ray from viewpoint, in radians,
                   measured ccw from positive x-axis
    -->source:     the source of the ray
    <--probe:      probe ray clipped to the room, as a line segment
******************************************************************************/
 
void buildProbe (float angle, V2f source, Line2f &probe, float radiusRoom)
{
  V2f towards (source[0] + cos(angle), source[1] + sin(angle));
  probe.create (source, towards);
  probe.createRayInsideSquare (radiusRoom);
}

/******************************************************************************
    Given a ray P+t(Q-P), where P = from, and Q = towards,
    build a ray segment (as a Bezier curve) clipped to the room, from P to 
    the room boundary on P+t(Q-P), t>=0.

    -->from:       starting point of probe ray
    -->towards:    another point of the ray
    <--probe:      probe ray clipped to the room, as a linear Bezier curve;
                   first control point is viewpoint, second is room boundary
    -->radiusRoom: radius of bounding room 
******************************************************************************/

void buildProbe (V2f from, V2f towards, BezierCurve2f &probe, float radiusRoom)
{
  Line2f probevec(from, towards);             // probe vector
  probevec.createRayInsideSquare(radiusRoom); // shrink the probe vector to the room
  probe.createLine(probevec[0], probevec[1]);
}

/******************************************************************************
    Clip tangents through viewpoint to probes.
    A probe is a ray from the viewpoint clipped at the room boundary.

    -->tangThruView:    tangents through viewpoint
    <--probeThruView:   associated probes
    -->radiusRoom:      radius of bounding room (needed to clip the infinite ray)
******************************************************************************/

void buildProbeThruView (Array<TangThruPtArr> &tangThruView, 
			 Array<BezierCurve2fArr> &probeThruView, 
			 float radiusRoom)
{
  V2f ptfoo, ptbar;   float tfoo, tbar;
  int nOb = tangThruView.getn();
  probeThruView.allocate(nOb);
  for (int i=0; i<nOb; i++)
    {
      probeThruView[i].allocate(tangThruView[i].getn());
      for (int j=0; j<tangThruView[i].getn(); j++)
        buildProbe (tangThruView[i][j].pt, 
		    tangThruView[i][j].ptTang, 
		    probeThruView[i][j], radiusRoom);
    }
}

/******************************************************************************
    Is this tangent through the viewpoint extreme?
    A tangent T of B through the viewpoint is extreme if all of B lies on one side of T.
    That is, T has no intersections with B, or in practice, only intersections
    in the epsilon-neighbourhood of the point of tangency.
    This epsilon-neighbourhood is necessary because of the imperfection of any 
    intersection algorithm: it basically encodes the accuracy of intersection.
    This test is a simplified version of the 'outer' test for bitangents
    in UmbralBitang.cpp.

    -->T:            a tangent through the viewpoint
    -->clippedT:     clipped tangent through the viewpoint
    -->B:            a smooth object
    -->epsIntersect: tolerance for intersection computation
    -->featureSize:  size of epsilon-neighbourhood around point of tangency = 
                     tolerance for recognizable features: any concavity with opening
                     smaller than this tolerance will be ignored; in practice,
                     intersections of the tangent with B within this neighbourhood
                     of the point of tangency are ignored
    <--: is this tangent extreme?
******************************************************************************/

int 
isExtreme (TangThruPt &T, BezierCurve2f &clippedT, BezierCurve2f &B,
	   float epsIntersect, float featureSize)
{
  // count intersections with B, removing intersections near pt of tangency
  return(clippedT.countIntersectionsWithOb (B, T.ptTang, epsIntersect, featureSize) == 0);
}

/******************************************************************************
    Find the extreme tangents of each object,
    by filtering the tangents through the viewpoint.

    -->obstacle:        smooth objects in the scene
    -->tangThruView:    tangents through the viewpoint, for each object
    -->tangSegThruView: clipped tangents through the viewpoint
    <--extreme:         extreme tangents
    -->epsInt:          tolerance for intersection computation
    -->fSize:           tolerance for recognizable features; 
                        intersections of the tangent with B within this neighbourhood
                        of the point of tangency are ignored
******************************************************************************/

void findExtreme (Array<BezierCurve2f> &obstacle, 
		  Array<TangThruPtArr> &tangThruView, 
		  Array<BezierCurve2fArr> &tangSegThruView, 
		  V2iArr &extreme,
		  float epsInt, float fSize)
{
  extreme.allocate(obstacle.getn());
  for (int i=0; i<obstacle.getn(); i++)
   {
    int nExtreme=0;
    cout << "for object " << i << endl;
    for (int j=0; j<tangThruView[i].getn(); j++)
     if (isExtreme(tangThruView[i][j], tangSegThruView[i][j], obstacle[i], epsInt, fSize))
       if (nExtreme==2) { cout << "Found too many extreme tangents" << endl; exit(-1); }
       else extreme[i][nExtreme++] = j;
    assert (nExtreme==2);
   }
}

/******************************************************************************
    Is this tangent through the viewpoint piercing?
    A tangent T of B through the viewpoint is piercing if it intersects B 
    after its first point of tangency with B, but not before.
    (This implementation does not check whether it is the 'first' point of tangency.)

    This test is a simplified version of the 'piercing' test for bitangents
    in UmbralBitang.cpp.

    -->T:            a tangent through the viewpoint
    -->Tprobe:       associated tangent probe (from viewpoint to wall)
    -->B:            a smooth object
    <--pierceHit:    if this is a piercing tangent, the piercing intersection
    <--tPierceHit:   if this is a piercing tangent, the piercing intersection's parameter
    -->epsIntersect: tolerance for intersection computation
    -->featureSize:  size of epsilon-neighbourhood around point of tangency = 
                     tolerance for recognizable features: any concavity with opening
                     smaller than this tolerance will be ignored; in practice,
                     intersections of the tangent with B within this neighbourhood
                     of the point of tangency are ignored
    <--: is this tangent piercing?
******************************************************************************/

int 
isPiercing (TangThruPt &T, BezierCurve2f &Tprobe, BezierCurve2f &B, V2f &pierceHit,
	    float &tPierceHit, float epsIntersect, float featureSize)
{
  // intersect probe with B, ignoring intersections near point of tangency
  int nHit; V2fArr ptHit; FloatArr tHit,tCurve;
  Tprobe.intersectButIgnore (B, T.ptTang, featureSize, nHit, ptHit, 
			     tHit, tCurve, epsIntersect);
  // first intersection should be after point of tangency
  if (nHit == 0) return 0;
  if (viewpt.dist(ptHit[0]) > viewpt.dist(T.ptTang))
    {
       pierceHit = ptHit[0]; tPierceHit = tCurve[0];
       return 1;
    }
  else return 0;
}

/******************************************************************************
    Find the piercing tangents of each object,
    by filtering the tangents through the viewpoint.

    -->obstacle:        smooth objects in the scene
    -->tangThruView:    tangents through the viewpoint, for each object
    -->probeThruView:   associated probes
    <--piercing:        piercing tangents, as indices of tangents through viewpoint
    <--piercingTang:    piercing tangents, directly
    -->epsInt:          tolerance for intersection computation
    -->fSize:           tolerance for recognizable features; 
                        intersections of the tangent with B within this neighbourhood
                        of the point of tangency are ignored
******************************************************************************/

void findPiercing (Array<BezierCurve2f> &obstacle, 
		   Array<TangThruPtArr> &tangThruView, 
		   Array<BezierCurve2fArr> &probeThruView, 
		   IntArrArr &piercing,  Array<PiercingTangArr> &piercingTang,
		   float epsInt, float fSize)
{
  piercing.allocate(obstacle.getn());
  piercingTang.allocate(obstacle.getn());
  for (int i=0; i<obstacle.getn(); i++)
   {
    IntArr pierce(1000); // we assume that no object will have >1000 visible concavities
    V2fArr hit(1000);    // the piercing points for each piercing tangent
    FloatArr tHit(1000); // parameter values of the piercing points
    int j, nPiercing=0;
    for (j=0; j<tangThruView[i].getn(); j++)
     if (isPiercing(tangThruView[i][j], probeThruView[i][j], obstacle[i], hit[nPiercing], 
		    tHit[nPiercing], epsInt,fSize))
      {
       if (nPiercing == 1000) 
	 { cout << "Too many piercing tangents on object " << i << endl; exit(-1); }
       pierce[nPiercing++] = j;
      }
    piercing[i].allocate(nPiercing); // transfer to piercing, now that we know how many
    piercingTang[i].allocate(nPiercing);
    for (j=0; j<nPiercing; j++)
     {
      piercing[i][j] = pierce[j];
      piercingTang[i][j] = tangThruView[i][pierce[j]];
      piercingTang[i][j].ptPierce = hit[j];
      piercingTang[i][j].tPierce = tHit[j];
     }
   }
}

/******************************************************************************
    Probe #1: Does this probe hit the given object at all?
    And if so, what is the first point of the given object hit by this probe?
    Method: Compute the intersections of the probe ray with the object,
    as Bezier curves.

    -->ob:         given object
    -->probe:      probe ray clipped to the room
    <--ptFirstHit: first point hit, if it hits at all
    <--tFirstHit:  parameter t of this point (on the probe P+t(Q-P)) 
    -->radiusRoom: radius of room centered at the origin that bounds the scene
    -->epsInt:     tolerance for intersection computation
    <--:           1 iff probe hits given object
******************************************************************************/

int firstPtHit (BezierCurve2f &ob, BezierCurve2f &probe, V2f &ptFirstHit, 
		float &tFirstHit, float radiusRoom, float epsInt)
{
  // intersect the probe with the object
  int nHit;  V2fArr ptHit;  FloatArr tHit, foo;
  probe.intersect (ob, nHit, ptHit, tHit, foo, epsInt);
  if (nHit == 0) return 0;
  ptFirstHit = ptHit[0]; // first intersection
  tFirstHit  = tHit[0];
  return 1;
}

/******************************************************************************
    Probe #2: What is the first object in the scene hit by this probe?
    And what is the point hit on this first object?

    Compute first intersection with each object (EVENTUALLY: ONLY THOSE OBJECTS
    WHOSE RANGE OVERLAPS THE PROBE).
    Keep track of the closest intersection.

    -->object:     smooth objects in the scene
    -->probe:      probe ray clipped to the room
    <--ptFirstHit: first point hit on any object
    -->radiusRoom: radius of room centered at the origin that bounds the scene
    -->epsInt:     tolerance for intersection computation
    <--:           index of the first object hit (-1 if no object hit)
******************************************************************************/

int firstObjectHit (Array<BezierCurve2f> &object, BezierCurve2f &probe,
		    V2f &ptFirstHit, float radiusRoom, float epsInt)
{
  float tFirstHit = -1;  
  int   objFirstHit = -1;
  for (int i=0; i<object.getn(); i++)
    {
      V2f ptFirstHitHere;  float tFirstHitHere; 
      if (firstPtHit(object[i], probe, ptFirstHitHere, tFirstHitHere, radiusRoom, epsInt))
	if (tFirstHit == -1 || tFirstHitHere < tFirstHit)
	  {
	    tFirstHit = tFirstHitHere;
	    ptFirstHit = ptFirstHitHere;
	    objFirstHit = i;
	  }
    }
  return objFirstHit;
}

/******************************************************************************
    Version of firstPtHit that ignores intersections near a prescribed point.
    This is primarily necessary to avoid treating a pt of tangency as an intersection.
    Code is identical but for a call to intersectButIgnore rather than intersect.

    -->ob:         given object
    -->probe:      probe ray clipped to the room
    <--ptFirstHit: first point hit, if it hits at all
    <--tFirstHit:  parameter t of this point (on the probe P+t(Q-P)) 
    -->ptToIgnore:  point about which intersections should be ignored
    -->featureSize: size of epsilon-neighbourhood around ignored point =
                    tolerance for recognizable features; intersections within
                    this neighbourhood of the given point are ignored
    -->radiusRoom: radius of room centered at the origin that bounds the scene
    -->epsInt:     tolerance for intersection computation
    <--:           1 iff probe hits given object
******************************************************************************/

int firstPtHit (BezierCurve2f &ob, BezierCurve2f &probe, V2f &ptFirstHit, 
		float &tFirstHit, 
		V2f ptToIgnore, float featureSize,
		float radiusRoom, float epsInt)
{
  // intersect the probe with the object
  int nHit;  V2fArr ptHit;  FloatArr tHit, foo;
  probe.intersectButIgnore (ob, ptToIgnore, featureSize, nHit, ptHit, tHit, foo, epsInt);
  if (nHit == 0) return 0;
  ptFirstHit = ptHit[0]; // first intersection
  tFirstHit  = tHit[0];
  return 1;
}

/******************************************************************************
    Version of firstObjectHit that ignores intersections with a prescribed object
    near a prescribed point.  This is primarily necessary to avoid counting a 
    point of tangency as an intersection.

    see isPiercing for inspiration

    -->object:      smooth objects in the scene
    -->probe:       probe ray clipped to the room
    <--ptFirstHit:  first point hit on any object
    -->ptToIgnore:  point about which intersections should be ignored
    -->objToIgnore: on this object only
    -->featureSize: size of epsilon-neighbourhood around ignored point =
                    tolerance for recognizable features; intersections within
                    this neighbourhood of the given point are ignored
    -->radiusRoom:  radius of room centered at the origin that bounds the scene
    -->epsInt:      tolerance for intersection computation
    <--:            index of the first object hit (-1 if no object hit)
******************************************************************************/

int firstObjectHit (Array<BezierCurve2f> &object, BezierCurve2f &probe, V2f &ptFirstHit, 
		    V2f ptToIgnore, int objToIgnore, float featureSize,
		    float radiusRoom, float epsInt)
{
  float tFirstHit = -1;  
  int   objFirstHit = -1;
  for (int i=0; i<object.getn(); i++)
    {
      V2f ptFirstHitHere; float tFirstHitHere; 
      if (i == objToIgnore)
       {
	if (firstPtHit(object[i], probe, ptFirstHitHere, tFirstHitHere, 
		       ptToIgnore, featureSize, radiusRoom, epsInt))
	  if (tFirstHit == -1 || tFirstHitHere < tFirstHit)
	   {
	    tFirstHit = tFirstHitHere;
	    ptFirstHit = ptFirstHitHere;
	    objFirstHit = i;
	   }
       }
      else 
       {
	if (firstPtHit(object[i], probe, ptFirstHitHere, tFirstHitHere, radiusRoom, epsInt))
	  if (tFirstHit == -1 || tFirstHitHere < tFirstHit)
	   {
	    tFirstHit = tFirstHitHere;
	    ptFirstHit = ptFirstHitHere;
	    objFirstHit = i;
	   }
       }
    }
  return objFirstHit;
}

/******************************************************************************
    Does this point lie in the given concavity?
    See appendix of object visibility paper.

    -->Q:          the point being tested
    -->B:          object that contains the concavity
    -->conInterval:parameter interval of the concavity on B
    -->viewpt:     the viewpoint
    -->epsInt:     tolerance for intersection computation
    -->radiusRoom: radius of bounding room 
    <--:           1 iff point lies in the concavity
******************************************************************************/

int ptInConcavity (V2f Q, BezierCurve2f &B, V2f conInterval, V2f viewpt, 
		   float epsInt, float radiusRoom)
{
  BezierCurve2f probe;
  probe.createLine (viewpt, Q);
  int nHit; V2fArr ptHit; FloatArr tHit, tHitProbe;
  probe.intersect (B, nHit, ptHit, tHitProbe, tHit, epsInt); 
  // want intersections sorted along line
  int found=0, first, i; // find index of first intersection with concavity
  for (i=0; !found && i<nHit; i++)
    if (B.ptLiesOnSeg (tHit[i], conInterval)) { found = 1; first = i; }
  // if (found) cout << "Found an intersection: " << i << endl;
  if (!found) return 0;
  for ( ; i<nHit; i++) // check that all future intersections lie on concavity
    if (!B.ptLiesOnSeg(tHit[i], conInterval)) 
      { 
	// cout << "One of the future intersections leaves concavity" << endl; 
        return 0; 
      }
  return 1;
}

/******************************************************************************
    Does object A lie inside the given concavity of B?
    See lemma 2 of object visibility paper.

    First requirement: the angular range of A is a subset of the concavity's 
    angular range.  We certainly need A's range endpoints to lie inside 
    the concavity range but this is not sufficient, since A's range may be the 
    complement of the apparent interval (wrapping around instead).
    Therefore, check that the concavity's range endpoints do NOT lie in A's range
    (it is enough to check one of them).

    Second requirement: the points of tangency of the extreme tangents of A
    both lie inside the concavity.

    -->A:           the object that we are testing 
                    (never used, but more intuitive passing it in)
    -->extreme1:    first extreme tangent of A
    -->extreme2:    second extreme tangent of A
    -->angRangeA:   angular range of A
    -->B:           object that contains the concavity
    -->angRangeCon: angular range of the concavity on B
    -->intervalCon: parameter interval of the concavity on B
    -->viewpt:      the viewpoint
    -->epsInt:      tolerance for intersection computation
    -->radiusRoom:  radius of bounding room 
    <--:            1 iff object lies in the concavity
******************************************************************************/

int objInConcavity (BezierCurve2f &A, TangThruPt extreme1, TangThruPt extreme2, 
		    V2f angRangeA, 
		    BezierCurve2f &B, V2f angRangeCon, V2f intervalCon,
		    V2f viewpt, float epsInt, float radiusRoom)
{
  // is the angular range of A a subset of the angular range of the concavity?
  if (! (angRangeCon.contains (angRangeA[0]) &&
	 angRangeCon.contains (angRangeA[1]) &&
	 !angRangeA.contains  (angRangeCon[0])))
    return 0;
  if (!ptInConcavity(extreme1.ptTang, B, intervalCon, viewpt, epsInt, radiusRoom))
    return 0;
  if (!ptInConcavity(extreme2.ptTang, B, intervalCon, viewpt, epsInt, radiusRoom))
    return 0;
}

/******************************************************************************
    Does object A lies behind B?
    See lemma 1 of object visibility paper.

    1) angular range of A \subset angular range of B
    2) probe to A (in between the extreme tangents) 
      i)  hits B before A
      ii) the probed pt of A does not lie in any concavity of B

    -->A:           the object that we are testing 
    -->angRangeA:   angular range of A
    -->B:           potentially blocking object 
    -->angRangeB:   angular range of B
    -->conInterval: parameter interval of each of B's concavities
    -->viewpt:      the viewpoint
    -->epsInt:      tolerance for intersection computation
    -->radiusRoom:  radius of bounding room 
    <--:            1 iff A lies completely behind B
******************************************************************************/

int objLiesBehind (BezierCurve2f &A, V2f angRangeA, 
		   BezierCurve2f &B, V2f angRangeB, V2fArr conInterval,
		   V2f viewpt, float epsInt, float radiusRoom)
{
  if (! (angRangeB.contains (angRangeA[0]) &&   // range of A lies in range of B
	 angRangeB.contains (angRangeA[1]) &&
	 !angRangeA.contains (angRangeB[0])))
    return 0;
  float midAngle;  // probe at midangle to find a point of A
  if (angRangeA[0] < angRangeA[1])
       midAngle = (angRangeA[0] + angRangeA[1]) / 2;
  else 
    {
      float intervalSize=(A.getLastKnot() - angRangeA[0]) + (angRangeA[1] - A.getKnot(0));
      if (angRangeA[0] + intervalSize/2 <= A.getLastKnot())
	   midAngle = angRangeA[0] + intervalSize/2;
      else midAngle = angRangeA[1] - intervalSize/2;
    }
  BezierCurve2f midProbe;
  V2f towards (viewpt[0] + cos(midAngle), viewpt[1] + sin(midAngle));
  buildProbe (viewpt, towards, midProbe, radiusRoom);  
  V2f ptOnA, ptOnB; float tOnA, tOnB;
  firstPtHit (A, midProbe, ptOnA, tOnA, radiusRoom, epsInt); // choose pt of A
  firstPtHit (B, midProbe, ptOnB, tOnB, radiusRoom, epsInt);
  if (tOnA < tOnB) // probe should intersect B before A 
    return 0;
  for (int i=0; i<conInterval.getn(); i++)  // for each of B's concavities
    if (ptInConcavity (ptOnA, B, conInterval[i], viewpt, epsInt, radiusRoom))
      return 0;  // pt on A shouldnt lie in any concavity of B
  return 1;
}

/******************************************************************************
    Backtracking algorithm for object visibility.
    See table 2 of object visibility paper.

    -->probe:      present probe
    -->obstacle:   objects in the scene
    -->star:       the object being tested for visibility
    -->tangThruView: viewpoint tangents for each object
    -->extreme:    extreme tangents for each object
    -->piercing:   piercing tangents for each object
    -->newB:       present object in the chain
    -->oldB:       previous object in the chain
    -->firstTime:  is this the first step of the backtrack algorithm?
    -->ptTang:     point of tangency on oldB, if this is the first step of the backtrack
    -->angExtreme: angles of extreme tangents
    <-->probeAlg:  probes (in order of consideration by the algorithm)
    <-->probeOb:   chain of probed objects
    <-->probePt:   endpoints of probes (probePt[i] lies on probeOb[i])
    <-->probePtA:  first probed point on A (for each probe)
    <-->nProbe:    index of next probe
    <--witness:    witness to visibility, if star is visible
    -->range:      angular range of A that has been covered so far
    -->viewpt:     the viewpoint
    -->epsInt:     tolerance for intersection computation
    -->radiusRoom: radius of bounding room
    <--:           1 iff backtracking found a visible witness
******************************************************************************/

int backtrack (BezierCurve2f &probe, BezierCurve2fArr &obstacle, int star, 
	       TangThruPtArrArr &tangThruView, V2iArr &extreme, IntArrArr &piercing,
	       int piercingInBacktrack,
	       int newB, int oldB, int firstTime, V2f ptTang, V2fArr &angExtreme,
	       BezierCurve2fArr &probeAlg, IntArr &probeOb, V2fArr &probePt, 
	       V2fArr &probePtA, int &nProbe, Line2f &witness, V2f &range,
	       V2f viewpt, float epsInt, float radiusRoom)
{
  V2f ptFirstHitA, ptFirstHitB; float tFirstHitA, tFirstHitB; // hit with A and oldB
  assert(firstPtHit (obstacle[star], probe, ptFirstHitA, tFirstHitA, radiusRoom, epsInt));
  if (firstTime) 
    {
      V2f roomPt;
      probe.getCtrlPt (1, roomPt);  // extreme pt of probe on room boundary
      if (roomPt[0] != viewpt[0])
	   tFirstHitB = (ptTang[0] - viewpt[0]) / (roomPt[0] - viewpt[0]);
      else tFirstHitB = (ptTang[1] - viewpt[1]) / (roomPt[1] - viewpt[1]);
    }
  else assert(firstPtHit (obstacle[oldB], probe, ptFirstHitB, tFirstHitB, 
			  radiusRoom, epsInt));
  if (tFirstHitA < tFirstHitB) // does probe hit A before B?
   {
     // START HERE: NEED TO CONSIDER PIERCING TANGENTS MORE ELEGANTLY
     if (piercingInBacktrack)
       {
	 // HACK!!!
	 // assume for now that there is just one piercing tangent
	 buildProbe (viewpt, tangThruView[newB][piercing[newB][0]].ptTang, 
		     probeAlg[nProbe], radiusRoom);
       }
     else
       {
	 if (range.contains (angExtreme[newB][0]))  // choose the one in range (backwards)
	   buildProbe (viewpt, tangThruView[newB][extreme[newB][0]].ptTang, 
		       probeAlg[nProbe], radiusRoom);
	 else buildProbe (viewpt, tangThruView[newB][extreme[newB][1]].ptTang, 
			  probeAlg[nProbe], radiusRoom);
       }
     newB = probeOb[nProbe] 
          = firstObjectHit (obstacle, probeAlg[nProbe], probePt[nProbe],
			    radiusRoom, epsInt);
// skipped (may never be necessary): set probePtA[nProbe] (and set in comparable place in isStarVisible)
     nProbe++;
     if (newB == star || newB == -1)
       { witness[0] = viewpt; probeAlg[nProbe-1].getCtrlPt(1,witness[1]); return 1; }
     V2f foo; // not the first time, so don't need pt of tangency
     return backtrack (probeAlg[nProbe-1], obstacle, star, tangThruView, extreme, 
		       piercing, piercingInBacktrack,
		       newB, oldB, 0, foo, angExtreme,
		       probeAlg, probeOb, probePt, probePtA, nProbe, witness, range,
		       viewpt, epsInt, radiusRoom);
   }
  else return 0;
}

/******************************************************************************
    Is the given object visible from the given viewpoint?

    References to A (here and in the paper) are equivalent to star.

    Reference: objectvisibility algorithm in 'Can you see me?' paper.

    -->star:         the object to be tested for visibility
    -->obstacle:     objects in the scene
    -->tangThruView: viewpoint tangents for each object
    -->extreme:      extreme tangents for each object
    -->piercing:     piercing tangents for each object
    -->conInterval:  parameter interval of each concavity
    -->angRange:     angular range of each object
    -->angRangeCon:  angular range of each concavity
    -->angExtreme:   angles of extreme tangents
    -->angPiercing:  angles of piercing tangents
    <--nProbe:       # of probes in the pass (probeAlg is allocated with max of 1000)
    <--probeAlg:     probes (in order of consideration by the algorithm)
    <--probeOb:      chain of probed objects (in order of consideration)
    <--probePt:      endpoints of probes (probePt[i] lies on probeOb[i])
    <--probePtA:     first probed point on A (for each probe)
    <--witness:      witness to visibility, if star is visible
    <--range:        angular range of star that is covered
    -->reversePass:  should we use reverse (i.e., clockwise) direction of pass
    -->viewpt:       the viewpoint
    -->epsInt:       tolerance for intersection computation
    -->radiusRoom:   radius of bounding room
    <--:             1 iff given object is visible
******************************************************************************/

int isStarVisible (int star, BezierCurve2fArr &obstacle, TangThruPtArrArr &tangThruView, 
		   V2iArr &extreme, IntArrArr &piercing, int piercingInBacktrack,
		   V2fArrArr &conInterval,
		   V2fArr &angRange, V2fArrArr &angRangeCon, 
		   V2fArr &angExtreme, FloatArrArr &angPiercing, int &nProbe,
		   BezierCurve2fArr &probeAlg, IntArr &probeOb, V2fArr &probePt, 
		   V2fArr &probePtA, Line2f &witness, V2f &range, int reversePass,
		   V2f viewpt, float epsInt, float radiusRoom)
{
  int i;
  probeAlg.allocate (1000);  // assume algorithm will probe at most 1000 times
  probeOb.allocate (1000); 
  probePt.allocate(1000);
  probePtA.allocate(1000);
  cout << "Probing at extreme tangent of A" << endl;
  int extremeProbeIndex; // which extreme tangent of A should we start probing at?
  if ((!reversePass && angExtreme[star][0] == angRange[star][0]) ||
       (reversePass && angExtreme[star][0] == angRange[star][1]))
       extremeProbeIndex = 0;
  else extremeProbeIndex = 1;
  buildProbe (viewpt,                         // first probe = extreme tangent of A
	      // choose probe at beg (or end if reverse direction) of A's range
	      tangThruView[star][extreme[star][extremeProbeIndex]].ptTang,
	      probeAlg[0], radiusRoom);
  int B = probeOb[0] = firstObjectHit (obstacle, probeAlg[0], probePt[0], // find B
				       radiusRoom, epsInt);
  /*
  first probe: if first pt of intersection 
               is after ignored pt on A, count A as first object hit
			  (so, in practice, dont need to ignore pt)
  */
  nProbe++;
  cout << "Probe hits " << B << endl;
  if (B == star || B == -1) 
    { witness[0] = viewpt; probeAlg[0].getCtrlPt(1,witness[1]); return 1; }
  cout << "Testing if this object completely blocks the star" << endl;
  if (objLiesBehind (obstacle[star], angRange[star],   // is A blocked by B alone?
		     obstacle[B], angRange[B], conInterval[B], 
		     viewpt, epsInt, radiusRoom))
    return 0;
  cout << "Testing if any concavity of this object contains the star" << endl;
  for (i=0; i<piercing[B].getn(); i++)
    if (objInConcavity (obstacle[star], 
			tangThruView[star][extreme[star][0]],
			tangThruView[star][extreme[star][1]], angRange[star], 
			obstacle[B], angRangeCon[B][i], conInterval[B][i],
			viewpt, epsInt, radiusRoom))
    return 0;
  probePtA[0] = tangThruView[star][extreme[star][extremeProbeIndex]].ptTang;
  int inConcavity=0; float angleProbe; 
  V2f ptTangOnOldB;  // pt of tangency of the next probe
  cout << "Testing if probed pt of star lies in a concavity of this object" << endl;
  for (i=0; i<conInterval[B].getn(); i++)
    if (ptInConcavity (probePtA[0], obstacle[B], conInterval[B][i], 
		       viewpt, epsInt, radiusRoom))
      {
	inConcavity = 1;
	buildProbe (viewpt, 
		    tangThruView[B][piercing[B][i]].ptTang,
		    probeAlg[1], radiusRoom);
	angleProbe = tangThruView[B][piercing[B][i]].angle();
	ptTangOnOldB = tangThruView[B][piercing[B][i]].ptTang;
      }
  if (!inConcavity)
   if (angRange[star].contains(angExtreme[B][0])) // choose extr tang in A's angular range
    {
      buildProbe (viewpt,                        
		  tangThruView[B][extreme[B][0]].ptTang,
		  probeAlg[1], radiusRoom);
      angleProbe = tangThruView[B][extreme[B][0]].angle();
      ptTangOnOldB = tangThruView[B][extreme[B][0]].ptTang;
    }
   else 
    {
      buildProbe (viewpt,                        
		  tangThruView[B][extreme[B][1]].ptTang,
		  probeAlg[1], radiusRoom);
      angleProbe = tangThruView[B][extreme[B][1]].angle();
      ptTangOnOldB = tangThruView[B][extreme[B][1]].ptTang;
    }
  if (reversePass)
    {
      range[0] = angleProbe;
      range[1] = angRange[star][1]; // started probing at end of A's angular range
    }
  else 
    {
      range[0] = angRange[star][0]; // start probing at beg of A's angular range 
      range[1] = angleProbe;
    }
  nProbe = 1;
  do
   {
    int oldB = B;
    cout << "Probing" << endl;
    // probe, but ignore pts of intersection near the pt of tangency
    B = probeOb[nProbe] = firstObjectHit (obstacle, probeAlg[nProbe], probePt[nProbe],
					  ptTangOnOldB, oldB, featureSize,
					  radiusRoom, epsInt);
    cout << "Next probe hit " << B << endl;
    nProbe++;  // we've now actually probed at this ray
    if (B == star || B == -1) 
      { witness[0] = viewpt; probeAlg[nProbe-1].getCtrlPt(1,witness[1]); return 1; } 
    if (backtrack (probeAlg[nProbe-1], obstacle, star, tangThruView, extreme, piercing,
		   piercingInBacktrack,
		   B, oldB, 1, ptTangOnOldB, angExtreme,
		   probeAlg, probeOb, probePt, probePtA, nProbe, witness, range, 
		   viewpt, epsIntersect, radiusRoom))
      return 1; 
    // choose the extreme tangent of B not in range as next probe;
    // or if B has piercing tangents not in range, choose the first one of them instead;

    // first find the piercing tangent if it exists
    int piercingNotInRange=-1;// index of closest piercing tangent (if any) not in range
                              // distance is measured in sweep direction
    float optimalPiercingAngle;  // angle of this tangent
    for (i=0; i<piercing[B].getn(); i++)
      if (!range.contains (angPiercing[B][i])) // a candidate piercing tangent
       {
	float testAngle = angPiercing[B][i];
	if (reversePass && testAngle > angleProbe) //deal w. wraparound or crossing x-axis
	  testAngle -= 2*M_PI; // want to measure in clockwise direction
	else if (!reversePass && testAngle < angleProbe)
	  testAngle += 2*M_PI; // want to measure ccw
	if (piercingNotInRange == -1) // first piercing tangent found
	  {
	    piercingNotInRange = i;
	    optimalPiercingAngle = testAngle;
	  }
	else  // choose it only if it is better than the one already found
	 {
	  if (reversePass) // sweeping towards lesser angles if reversePass
	   {
	    if (testAngle > optimalPiercingAngle) // bigger is closer
	     {
	      optimalPiercingAngle = testAngle;
	      piercingNotInRange = i;
	     }
	   }
	  else
	   {
	    if (testAngle < optimalPiercingAngle)
	     {
	       optimalPiercingAngle = testAngle;
	       piercingNotInRange = i;
	     }
	   }
	 }
       }
    if (piercingNotInRange != -1)
     {
       cout << "Next probe is at a piercing tangent" << endl;
       buildProbe (viewpt, tangThruView[B][piercing[B][piercingNotInRange]].ptTang,
		   probeAlg[nProbe], radiusRoom);
       angleProbe = angPiercing[B][piercingNotInRange];
       ptTangOnOldB = tangThruView[B][piercing[B][piercingNotInRange]].ptTang;
     }
    else // choose extreme tangent
     {
      cout << "Next probe is at an extreme tangent" << endl;
      if (range.contains (angExtreme[B][0]))
       {  
	buildProbe (viewpt,  
		    tangThruView[B][extreme[B][1]].ptTang,
		    probeAlg[nProbe], radiusRoom);
	angleProbe = angExtreme[B][1];
	ptTangOnOldB = tangThruView[B][extreme[B][1]].ptTang;
       }
      else
       {
	buildProbe (viewpt,  
		    tangThruView[B][extreme[B][0]].ptTang,
		    probeAlg[nProbe], radiusRoom);
	angleProbe = angExtreme[B][0];
	ptTangOnOldB = tangThruView[B][extreme[B][0]].ptTang;
       }
     }
    cout << "Extending to angle " << angleProbe << endl;
    if (reversePass) range[0] = angleProbe; else range[1] = angleProbe;
   } 
  while (angRange[star].contains(angleProbe)); // haven't yet swept past the object
  return 0; // have swept past: it is not visible
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  float     epsDual = .0001; // accuracy of intersection computation in dual space
  float     epsInside=.01;   // step size to test which side of tangent the curve lies on
  float     epsSample;       // parameter distance between consecutive samples
  int       reversePass=0;   // should we reverse the pass direction (cw instead of ccw)
  int       piercingInBacktrack=0; // should we backtrack to piercing tangent instead (a hack)?
  int	    i,j;

  epsIntersect = .001; featureSize = .05; epsAngle = .1; zoomob = 1;
  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'E': epsDual = atof(argv[ArgsParsed++]);		break;
      case 'e': epsIntersect = atof(argv[ArgsParsed++]);        break;
      case 'F': featureSize = atof(argv[ArgsParsed++]);         break;
      case 's': star = atoi(argv[ArgsParsed++]);                break;
      case 'v': viewpt[0] = atof(argv[ArgsParsed++]);
      		viewpt[1] = atof(argv[ArgsParsed++]);	        break;
      case 'r': radiusRoom = atof(argv[ArgsParsed++]);          break;
      case 'l': level = atoi(argv[ArgsParsed++]);               break;
      case 'z': zoomob = atof(argv[ArgsParsed++]);              break;
      case 'R': reversePass = 1;                                break;
      case 'p': piercingInBacktrack = 1;                        break;
      case 'L': LAPTOP = 1;                                     break;
      case 't': EXPOSITORY = 1;                                 break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  inputCurves(argv[argc-1], obstacle);
  int nOb = obstacle.getn();
if (level > 0) 
{
cout << "Dualizing" << endl;
  // dualize
  buildTangentialCurves (obstacle);           
  viewptduala.createA (viewpt, -1);
  viewptdualb.createB (viewpt, -1);
}
  // compute tangents through viewpt
if (level > 1)
{
cout << "Computing tangents through the viewpoint" << endl;
  int nTang=0; 
  Array<TangThruPtArr>   tangA(nOb);    // viewpoint tangents from a-space for each curve
  Array<TangThruPtArr>   tangB(nOb);    // viewpoint tangents from b-space for each curve
  tangThruView.allocate(nOb);
  for (i=0; i<nOb; i++)
   {
    obduala[i].intersect (viewptduala, tangA[i], epsDual);
    obdualb[i].intersect (viewptdualb, tangB[i], epsDual);
    tangThruView[i].append (tangA[i], tangB[i]);
    nTang += tangThruView[i].getn();
   }
  // collect all tangents
  Array<TangThruPt> alltangorig(nTang);  nTang=0;
  for (i=0; i<nOb; i++)
    for (j=0; j<tangThruView[i].getn(); j++)
      alltangorig[nTang++] = tangThruView[i][j];
  // sort the tangents by angle from x-axis
  FloatArr angle(alltangorig.getn());
  for (i=0; i<alltangorig.getn(); i++)
    {
      V2f v(alltangorig[i].ptTang); v -= alltangorig[i].pt;
      v.normalize();
      angle[i] = v.angle();
    }
  IntArr sortIndex; angle.bubbleSort(sortIndex);
  alltang.allocate(alltangorig.getn());
  for (i=0; i<alltang.getn(); i++)        // extract tangents in sorted order
    alltang[i] = alltangorig [sortIndex[i]];
}
/*
  // identify visible tangents thru viewpt (unnecessary)
  Array<IntArr> vistangA(nOb);     // is this a-tangent visible?
  Array<IntArr> vistangB(nOb);     // is this b-tangent visible?
  vistang.allocate(nOb);
  V2f foo;
  for (i=0; i<nOb; i++)      
   {
    vistangA[i].allocate (tangA[i].getn());
    for (j=0; j<tangA[i].getn(); j++)
      if (tangA[i][j].visible(obstacle)) vistangA[i][j] = 1;
      else 		     	         vistangA[i][j] = 0;
    vistangB[i].allocate (tangB[i].getn());
    for (j=0; j<tangB[i].getn(); j++)
      if (tangB[i][j].visible(obstacle)) vistangB[i][j] = 1;
      else 			         vistangB[i][j] = 0;
    vistang[i].append (vistangA[i], vistangB[i]);
   }
*/
if (level > 2)
{
cout << "Computing all probes" << endl;
  buildTangSegThruView (tangThruView, tangSegThruView, radiusRoom);
  buildProbeThruView (tangThruView, probeThruView, radiusRoom);

  // global probe at each tangent through viewpoint 
  // and compute first point of intersection of each probe
  probeHitAll.allocate(nOb);
  probeHitAllObj.allocate (nOb);
  for (i=0; i<nOb; i++)
    {
      probeHitAll[i].allocate(tangThruView[i].getn());
      probeHitAllObj[i].allocate(tangThruView[i].getn());
      for (j=0; j<tangThruView[i].getn(); j++)
	probeHitAllObj[i][j] = firstObjectHit (obstacle, probeThruView[i][j], 
					       probeHitAll[i][j], 
					       radiusRoom, epsIntersect);
    }
}
  // compute extreme tangents of each object
if (level > 3)
{
cout << "Computing extreme tangents" << endl;
  findExtreme (obstacle, tangThruView, tangSegThruView, extreme, 
	       epsIntersect, featureSize);
}
  // compute piercing tangents, again by filtering
if (level > 4)
{
cout << "Computing piercing tangents" << endl;
  findPiercing (obstacle, tangThruView, probeThruView, piercing, piercingTang,
		epsIntersect, featureSize);
}
  // compute angles of extreme tangents
if (level > 5)
{
  cout << "Computing angles of extreme tangents" << endl;
  angExtreme.allocate(nOb);
  for (i=0; i<nOb; i++)
    for (j=0; j<2; j++)
      angExtreme[i][j] = tangThruView[i][extreme[i][j]].angle();
  cout << "Computing angles of piercing tangents" << endl;
  angPiercing.allocate(nOb);
  for (i=0; i<nOb; i++)
   {
    angPiercing[i].allocate(piercing[i].getn());
    for (j=0; j<piercing[i].getn(); j++)
      angPiercing[i][j] = tangThruView[i][piercing[i][j]].angle();
   }
}
  // compute parameter interval of each concavity (Definition 11)
if (level > 5)
{
cout << "Computing parameter interval of each concavity" << endl;
  conInterval.allocate(nOb);
  for (i=0; i<nOb; i++) 
   {
    conInterval[i].allocate(piercingTang[i].getn());
    for (j=0; j<conInterval[i].getn(); j++)
     {
      float t1 = piercingTang[i][j].tTang,   // parameter of point of tangency
	    t2 = piercingTang[i][j].tPierce; // parameter of piercing point
      V2f ptForward; obstacle[i].eval (t2 + epsInside, ptForward); // B(t2 + eps)
      // interval depends on whether ptForward lies inside tangent
      if (piercingTang[i][j].inside (ptForward, obstacle, epsInside)) 
       {
	conInterval[i][j][0] = t2;
	conInterval[i][j][1] = t1;
       }
      else
       {
	conInterval[i][j][0] = t1;
	conInterval[i][j][1] = t2;
       }
     }
   }
}
  // prepare curve samples on each concavity (for display only)
if (level > 6)
{
  cout << "Preparing curve samples on each concavity" << endl;
  conSample.allocate(nOb);
  for (i=0; i<nOb; i++)
    {
      conSample[i].allocate(piercingTang[i].getn());
      int density = 200;
      epsSample = (obstacle[i].getLastKnot() - obstacle[i].getKnot(0)) / density;
      for (j=0; j<conSample[i].getn(); j++) // for each concavity
	obstacle[i].sampleSegmentClosed (conInterval[i][j][0], conInterval[i][j][1],
					 conSample[i][j], epsSample);
    }
}
  // compute extreme tangents of each concavity (Definition 12)
if (level > 7)
{
  cout << "Computing extreme tangents of each concavity" << endl;
  // START DEBUGGING HERE (WRONG ON OPENING.PTS)
  extremeCon.allocate(nOb);
  for (i=0; i<nOb; i++)
    {
      if (piercingTang[i].getn()>0) extremeCon[i].allocate(piercingTang[i].getn());
      for (j=0; j<piercingTang[i].getn(); j++)
	{
	  extremeCon[i][j].allocate (2);
	  float minAngle, maxAngle;
	  minAngle = maxAngle = tangThruView[i][piercing[i][j]].angle();
cout << "minAngle = maxAngle = " << minAngle << endl;
	  extremeCon[i][j][0] = extremeCon[i][j][1] = tangThruView[i][piercing[i][j]];
	  for (int k=0; k<tangThruView[i].getn(); k++)
	    // does point of tangency lie in concavity?
	    if (obstacle[i].ptLiesOnSeg (tangThruView[i][k].tTang, conInterval[i][j]))
	      {
		float angle = tangThruView[i][k].angle();
		cout << "Considering tangent with angle " << angle << endl;
		if (angle < minAngle) 
		  {
		    minAngle = angle;
		    extremeCon[i][j][0] = tangThruView[i][k];
		  }
		else if (angle > maxAngle) 
		  {
		    maxAngle = angle;
		    extremeCon[i][j][1] = tangThruView[i][k];
		  }
	      }
	}
    }
}  
  // compute angular range of each object (and store elegantly): Definition 8
if (level > 8)
{
  cout << "Computing angular range of each object" << endl;
  angRange.allocate(nOb);
  for (i=0; i<nOb; i++)
    {
      // local probe at midAngle to find intersections with obstacle[i]
      float midAngle = (angExtreme[i][0] + angExtreme[i][1]) / 2;
      float minAngle = min(angExtreme[i][0], angExtreme[i][1]);
      float maxAngle = max(angExtreme[i][0], angExtreme[i][1]);
      BezierCurve2f midProbe;
      V2f towards (viewpt[0] + cos(midAngle), viewpt[1] + sin(midAngle));
      buildProbe (viewpt, towards, midProbe, radiusRoom); V2f ptHit; float tHit;
      if (firstPtHit (obstacle[i], midProbe, ptHit, tHit, radiusRoom, epsIntersect))
	{ 
	  angRange[i][0] = minAngle;
	  angRange[i][1] = maxAngle;
	}
      else
	{
	  angRange[i][0] = maxAngle;  // angular range wraps around the other way
	  angRange[i][1] = minAngle;
	}
    }
}
  // compute angular range of each concavity (Definition 13)
if (level > 9)
{
  cout << "Computing angular range of concavity" << endl;
  angRangeCon.allocate(nOb);
  for (i=0; i<nOb; i++)
   {
     cout << "A" << endl;
    if (piercingTang.getn()>0) angRangeCon[i].allocate(piercingTang.getn());
    for (j=0; j<piercingTang[i].getn(); j++)
     {
     cout << "B" << endl;
      float angle1=extremeCon[i][j][0].angle(); // angle of concavity's extreme tangents
     cout << "C" << endl;
      float angle2=extremeCon[i][j][1].angle();
      float minAngle = min(angle1,angle2), maxAngle = max(angle1,angle2);
      float midAngle=(angle1+angle2)/2;
     cout << "D" << endl;
      BezierCurve2f midProbe;                     // probe at midangle
      V2f towards (viewpt[0] + cos(midAngle), viewpt[1] + sin(midAngle));
      buildProbe (viewpt, towards, midProbe, radiusRoom);
      cout << "E" << endl;
      int nHit;  V2fArr ptHit;                    // look for intersections w concavity
      FloatArr tHitProbe, tHitOb; 
      midProbe.intersect (obstacle[i], nHit, ptHit, tHitProbe, tHitOb, epsIntersect);
      cout << "F" << endl;
      int found=0;
      for (int k=0; !found && k<nHit; k++)
	if (obstacle[i].ptLiesOnSeg (tHitOb[k], conInterval[i][j]))
	  found=1;
      cout << "G" << endl;
      if (found)                                  // assign angular range accordingly
	{
	  angRangeCon[i][j][0] = minAngle;
	  angRangeCon[i][j][1] = maxAngle;
	}
      else
	{
	  angRangeCon[i][j][0] = maxAngle;
	  angRangeCon[i][j][1] = minAngle;
	}
      cout << "H" << endl;
     }
   }
}
  cout << "Prep complete..." << endl;
 
  // probing pass
if (level > 10)
{
  cout << "Computing visibility" << endl;
  starIsVisible = isStarVisible(star, obstacle, tangThruView, extreme, piercing, 
				piercingInBacktrack,
				conInterval, angRange, angRangeCon, angExtreme, 
				angPiercing, nProbe, probeAlg, probeOb, probePt, 
				probePtA, witness, range,
				reversePass, viewpt, epsIntersect, radiusRoom);
  if (starIsVisible)
       cout << "The object is visible!" << endl;
  else cout << "The object is invisible!" << endl;
  probeLine.allocate(nProbe);
  for (i=0; i<nProbe; i++)
    {
      probeLine[i][0] = viewpt;
      probeAlg[i].getCtrlPt(1, probeLine[i][1]);
    }
    
}

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int titleht = 20; 	// top titlebar is 20 units high
  //  int xleft    = 164;		// x-coord of lefthand side for small windows
  //  int xsize = 400, ysize = 400;	// small windows
  int xleft = 0;	// x-coord of lefthand side for large windows
  int xsize = 800, ysize = 800;		// large windows
  int barmargin = 8; 	// width of side bar surrounding picture

  glutInitWindowPosition (LAPTOP ? 1100 : 600, titleht);		// primal window
  glutInitWindowSize (600,600);
  char titlebar[100]; 
  strcpy (titlebar, "Analysis of object visibility in a smooth flat scene (");  
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
  glutAddMenuEntry ("Objects",                                23);
  glutAddMenuEntry ("Object labels",                          21);
  glutAddMenuEntry ("Tangent labels",                         22);
  glutAddMenuEntry ("Star is red",                            -1);
  glutAddMenuEntry ("Viewpoint",                              17);
  glutAddMenuEntry ("Bold star",                               0);
  glutAddMenuEntry ("Tangents through viewpoint", 	       1);
  glutAddMenuEntry ("All probes",                              2);
  glutAddMenuEntry ("Extreme tangents",                        3);
  glutAddMenuEntry ("Extreme tangents of star only",           20);
  glutAddMenuEntry ("Extreme tangent probes",                  19);
  glutAddMenuEntry ("Angular range of star",                   4);
  glutAddMenuEntry ("Piercing tangents",                       5);
  glutAddMenuEntry ("Piercing tangent probes",                 18);
  glutAddMenuEntry ("Probes at extreme/piercing tangents",     16);
  glutAddMenuEntry ("Concavities",                             6);
  glutAddMenuEntry ("Extreme points in star's concavity",      7);
  glutAddMenuEntry ("Extreme tangents of concavity",           8);
  glutAddMenuEntry ("Angular range of concavity",              9);
  glutAddMenuEntry ("Objects in star's concavity",            10);
  glutAddMenuEntry ("Objects behind star",                    11);
  glutAddMenuEntry ("Visualize visibility",                   12);
  glutAddMenuEntry ("All probes in probing pass",             13);
  glutAddMenuEntry ("Virtual vertices",                       14);
  glutAddMenuEntry ("Colour vs. black+white",                 15);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
