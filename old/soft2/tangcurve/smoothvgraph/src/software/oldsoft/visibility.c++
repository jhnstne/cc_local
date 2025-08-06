/*
  File:          visibility.c++ 
  Author:        J.K. Johnstone 
  Created:	 30 October 1999
  Last Modified: 1 November 1999 
  Purpose:       Compute visibility graph of polygons.
  Input: 	 n obstacles.  Each obstacle is represented by an array
		 of sample points.  Thus, since this looks like
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
#include "Graph.h"

#define XWINDOWSIZE	 545	
#define YWINDOWSIZE	 545
#define ORTHOSIZE	 2.0	 // amount of world viewed in orthograph projection
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
static int  	 xwindowsize = XWINDOWSIZE;
static int	 ywindowsize = YWINDOWSIZE;

static GLboolean JUSTVIEWING=0;		// just viewing: don't process further
static GLboolean NEWFILE=1;		// output processed data to new file?
static GLboolean DRAWWIRE=1;		// draw polygons as wireframes?
static GLboolean DRAWPOLYGONOB=1;	// draw input polygonal obstacles?
static GLboolean DRAWVGRAPH=1;		// draw visibility graph?
static GLboolean DRAWVERT=1;		// draw vertices as points?
static GLboolean rotateOb=0;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'ccw' direction?

// static GLboolean DEBUG = 1;

int 	    	   	n=0;		// # of obstacles
Array<Polygon2f>   	obstaclePoly;
VisibilityGraph		vgraph;

Array<BezierCurve2f> 	obstacle;
BezierCurve2f 		hodo0;		// hodograph of 1st obstacle, 
					// for interactive tangent display
BezierCurve2f		hodo1;		// hodo of 2nd obstacle, for removing common tangent mistakes
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
Array<RatBezierCurve2f>	obdual;		// dual curves of obstacles
Array<RatBezierCurve2f> obdualreflex;	// reflection of duals
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
  xwindowsize = w;  ywindowsize = h;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(-ORTHOSIZE*(GLfloat)w/(GLfloat)h, ORTHOSIZE*(GLfloat)w/(GLfloat)h, 
  	     -ORTHOSIZE, ORTHOSIZE);
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

/******************************************************************************
	Translate from screen coordinates (x,y) to world coordinates wpt.
******************************************************************************/

void screen2world (int x, int y, V2f &wpt)
{
  y = ywindowsize - y;		// reverse y, so that it decreases as you drop
  wpt[0] = float(x) - xwindowsize/2.;	// make the center of the screen the origin
  wpt[1] = float(y) - ywindowsize/2.;
  // scale screen coordinates to world coordinates
  wpt[0] *= (ORTHOSIZE * 2 * (float(xwindowsize)/float(ywindowsize)))  	// world window size
  	    / (zoomob * xwindowsize);				// screen size
  wpt[1] *= (ORTHOSIZE * 2) / (zoomob * ywindowsize);
}

/******************************************************************************/
/******************************************************************************/

void motionob (int x, int y)
{
  if (leftMouseDown && !middleMouseDown && firstx)
   {
    firstx=0;
    V2f source; screen2world (x,y,source);
    vgraph.updateSource (source);
   }
  else if (!leftMouseDown && middleMouseDown && firstx)
   {
    firstx=0;
    V2f dest; screen2world (x,y,dest);
    vgraph.updateDest (dest);
   }
  else if (leftMouseDown && middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomob -= (float).01*(x-oldx);
    if (zoomob < 0.0) zoomob = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1: 	DRAWVGRAPH = !DRAWVGRAPH;				break;
  case 2: 	DRAWVERT   = !DRAWVERT;					break;
  case 3: 	DRAWPOLYGONOB = !DRAWPOLYGONOB;				break;
  default:   								break;
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
    
  if (DRAWVERT)			// display all vertices of graph (input points)
   {
    glColor3fv (Black);
    vgraph.drawVert();
   }
  if (DRAWPOLYGONOB)		// display polygonal obstacles
   {
    glColor3fv (Red);
    for (i=0; i<n; i++)  obstaclePoly[i].draw(1);
   }
  if (DRAWVGRAPH)
   {
    glColor3fv (Black);
    vgraph.draw();   
   }
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
  	Read data polygons as contours 
	(glorified polygons with tablet-input software already written).
	Don't want to read as Body, otherwise contours will be hidden 
	as private objects.
******************************************************************************/

void readInput (ifstream &infile, string &filePrefix, 
		int &n, Array<Polygon2f> &obstaclePoly)
{
  int format;  readFileName (infile, filePrefix, format);
  string comment;  readComment (infile, comment);
  getLeftBrace (infile);  string id;  infile >> id;  assert (id == "BODY");
  string name;  infile >> name;
  getLeftBrace (infile);  infile >> id;  assert (id == "SECTION"); infile >> name;
  infile >> id;  assert (id == "Z");  int zval;  infile >> zval;
  n=0;  int mark = infile.tellg();  	// count the contours
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
  
  obstaclePoly.allocate(n);
  for (i=0; i<n; i++)  obstaclePoly[i] = ctrData[i];
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
  int       ArgsParsed=0;
  ifstream  infile;
  ofstream  outfile;
  string    filePrefix;
  
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
  
  // *******************************************************************
  
  infile.open(argv[argc-1]);
  readInput(infile, filePrefix, n, obstaclePoly);
//  scale to unit cube
  vgraph.create(obstaclePoly);
  V2f source(-5,-5), dest(5,5);
  vgraph.updateSourceDest(source, dest);
  
/*  if (NEWFILE) 			// output V-graph
   {
    string vGraphFile = filePrefix + ".vgraph";
    outfile.open(vGraphFile.c_str());
    outfile << "{ " << vGraphFile << " }\n";
    // write polygonal obstacles out (repeat input) and then topology of Vgraph
   } */

  //  through mouse controls outside main:
  //  input source and destination through mouse; compute shortest path
      
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowPosition (164,20);
  glutInitWindowSize (XWINDOWSIZE,YWINDOWSIZE);
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
  glutAddMenuEntry ("Visibility graph", 1);
  glutAddMenuEntry ("Vertices", 2);
  glutAddMenuEntry ("Obstacles", 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
