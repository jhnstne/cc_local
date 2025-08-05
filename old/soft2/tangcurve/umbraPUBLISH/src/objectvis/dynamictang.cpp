/*
  File:          dynamictang.cpp
  Author:        J.K. Johnstone 
  Created:	 31 January 2006 (from objectvis.cpp)
  Last Modified: 31 January 2006
  Purpose:       To dynamically compute tangents through the viewpoint as the
                 viewpoint moves along a predefined curve or under mouse control
  Sequence:	 interpolate -> tangCurve -> poletang -> objectvis -> dynamictang
  Input: 	 A scene of closed curves, a distinguished curve, and a viewpoint.
                 A closed curve and its interior defines an object.
		 Each curve is defined by an ordered list of points, which imply
		 a closed interpolating cubic B-spline.
		 The distinguished curve is defined by its index in the scene.
                 The viewpoint must lie outside any object.
		 It is controllable by the mouse.
  Paper: 	 'Can you see it? Object visibility in a smooth flat scene'
                 J.K. Johnstone (2005)
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

// scene
Array<BezierCurve2f>   obstacle;     // scene objects = interpolating cubic Bezier curves
int                    star=0;       // index of distinguished object
V2f		       viewpt(-2,0); // point through which to compute tangents
Array<TangThruPtArr>   tangA;        // viewpoint tangents from a-space for each curve
Array<TangThruPtArr>   tangB;        // viewpoint tangents from b-space for each curve
float                  radiusRoom=2; // radius of room enclosing the scene (for clipping)
// dual of scene, to find tangents through viewpoint
Array<TangentialCurve> obduala;	     // associated tangential a-curves
Array<TangentialCurve> obdualb;	     // associated tangential b-curves
TangentialCurve	       viewptduala;  // a-dual of viewpoint
TangentialCurve	       viewptdualb;  // b-dual of viewpoint
// tangents through viewpoint
Array<TangThruPtArr>    tangThruView;    // viewpoint tangents for each curve
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
static GLboolean DRAWBOLD=1;	     // draw star in bold red and others in black?
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
static GLboolean GREYSCALE=1;        // no colours, for print images
static GLboolean PRINTOUT=0;         // 0: displaying on screen, 1: printing out image
static GLboolean LAPTOP=0;           // running on laptop (with larger screen)?
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
  case '0':  DRAWOBJINCON   = !DRAWOBJINCON;                 break;
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
  case 17: DRAWVIEW       = !DRAWVIEW;                     break;
  default:   			                           break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j;
  Line2f probe; 
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  int nOb = obstacle.getn();   V2f foo; int c=1; // colour index for non-stars
  if (DRAWVIEW)
    {
      glColor3fv (Black); glBegin(GL_POINTS); glVertex2f(viewpt[0],viewpt[1]); glEnd();
    }
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
  if (DRAWTANG)
   {
    glColor3fv (Black);
    for (i=0; i<nOb; i++)
      for (j=0; j<tangThruView[i].getn(); j++)
  	tangThruView[i][j].draw();
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
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  float     epsDual = .0001; // accuracy of intersection computation in dual space
  float     epsInside=.01;   // step size to test which side of tangent the curve lies on
  float     epsSample;       // parameter distance between consecutive samples
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
      case 'L': LAPTOP = 1;                                     break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  inputCurves(argv[argc-1], obstacle);

  int nOb = obstacle.getn();
  tangA.allocate (nOb);
  tangB.allocate (nOb);
  tangThruView.allocate(nOb);

  // dualize
  buildTangentialCurves (obstacle);         
  viewptduala.createA (viewpt, -1);
  viewptdualb.createB (viewpt, -1);

  // compute tangents through viewpt
                                cout << "Computing tangents through the viewpoint" << endl;
  for (i=0; i<nOb; i++)
   {
    obduala[i].intersect (viewptduala, tangA[i], epsDual);
    obdualb[i].intersect (viewptdualb, tangB[i], epsDual);
    tangThruView[i].append (tangA[i], tangB[i]);
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
  glutAddMenuEntry ("Star is red",                            -1);
  glutAddMenuEntry ("Viewpoint",                              17);
  glutAddMenuEntry ("Bold star",                               0);
  glutAddMenuEntry ("Tangents through viewpoint", 	       1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
