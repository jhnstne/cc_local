/*
  File:          visgraph.cpp
  Author:        J.K. Johnstone 
  Created:	 13 August 2001 (from 9-vgraph/svgraph.c++)
  Last Modified: 13 August 2001
  Purpose:       Compute a smooth visibility graph amongst
		 curved obstacles in the plane, and shortest path 
		 between source and destination.
		 Source and destination can be controlled by the mouse.
  Sequence:	 4th in a sequence (interpolate, tangCurve, bitang, visgraph)
  Input: 	 k 2d polygons, implicitly defining k interpolating cubic 
  		 Bezier curves.
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
#include "Vector.h"
#include "MiscVector.h"
#include "BezierCurve.h"
#include "TangCurve.h"
#include "Vgraph.h"		// create, updateSourceDest

#define PTSPERBEZSEGMENT 30     // # pts to draw on each Bezier segment
#define XWINDOWSIZE	 545
#define YWINDOWSIZE	 545
#define ORTHOSIZE	 2.0	// amount of world viewed in orthograph projection

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d displaydensity of Bezier segment] (default 30)" << endl;
  cout << "\t[-e eps]  (accuracy at which intersections are made: default .0001)" << endl;
  cout << "\t[-S xsource ysource] (default: (-2,0))" << endl;
  cout << "\t[-D xdest ydest] (default: (2,0))" << endl;
  cout << "\t[-s sampleeps] (polygon sampling rate: default .25)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1;	// first MOUSEX reading?
static int	 oldx;		// previous value of MOUSEX and MOUSEY
static int  	 xwindowsize = XWINDOWSIZE;
static int	 ywindowsize = YWINDOWSIZE;
static GLboolean SOURCEDEST=1;		// draw source/destination?
static GLboolean DRAWPOLYGON=0;		// draw polygonal obstacles?
static GLboolean DRAWPOLYVGRAPH=0;	// draw polygonal visibility graph?
static GLboolean DRAWPOLYPATH=0;	// draw polygonal shortest path?
static GLboolean DRAWCURVE=1;		// draw curved obstacles?
static GLboolean DRAWVGRAPH=1;		// draw smooth visibility graph?
static GLboolean DRAWPATH=0;		// draw shortest path?

Array<Polygon2f>   	obstaclePoly;
Array<BezierCurve2f> 	obstacle;
V2f			source(-2,0), dest(2,0);
VisibilityGraph		vgraph;		// polygonal visibility graph
SmoothVisibilityGraph	svgraph;	// smooth visibility graph
IntArr			polyShortestPath; 
IntArr			shortestPath;	// from source to destination
int			obstacleWin;
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

  transxob = transyob = 0.0;
  zoomob = 1.8;
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
******************************************************************************/

void printInfo()
{
  cout << "Smooth    visibility graph has (" << svgraph.getn() << ","
       << svgraph.getE() << ") (vertices,edges)." << endl;
  cout << "Polygonal visibility graph has (" << vgraph.getn() << ","
       << vgraph.getE() << ") (vertices,edges)." << endl;
  svgraph.dijkstra (svgraph.getn()-2,svgraph.getn()-1,shortestPath);
  vgraph.dijkstra (0,1,polyShortestPath);
  cout << "Length of smooth shortest path                                       = " 
       << svgraph.pathLength (shortestPath) << endl;
  cout << "Length of polygonal shortest path (using incorrect self-edge length) = "
       << vgraph.pathLength (polyShortestPath) << endl;
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
    screen2world (x,y,source);
    vgraph.updateSource  (source);
    svgraph.updateSource (source);
    printInfo();
   }
  else if (!leftMouseDown && middleMouseDown && firstx)
   {
    firstx=0;
    screen2world (x,y,dest);
    vgraph.updateDest (dest);
    svgraph.updateDest (dest);
    printInfo();
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

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 				break;	// ESCAPE
  case '1': 	SOURCEDEST = !SOURCEDEST;		break;
  case '2': 	DRAWPOLYGON=1; DRAWCURVE=0;
  		DRAWVGRAPH=0; DRAWPOLYVGRAPH=0;		break;
  case '3':	DRAWCURVE=1; DRAWPOLYGON=0;
  		DRAWPOLYVGRAPH=0; DRAWVGRAPH=0;		break;
  case '4':	DRAWPOLYVGRAPH=1; DRAWVGRAPH=0; 
  		DRAWCURVE=0;				break;
  case '5':  	DRAWVGRAPH=1; DRAWPOLYVGRAPH=0;		
  		DRAWCURVE=1; DRAWPOLYGON=0;		break;
  case '6':	DRAWPOLYPATH = !DRAWPOLYPATH;		break;
  case '7':	DRAWPATH = !DRAWPATH;			break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1: 	SOURCEDEST = !SOURCEDEST;		break;
  case 2: 	DRAWPOLYGON=1; DRAWCURVE=0;
  		DRAWVGRAPH=0; DRAWPOLYVGRAPH=0;		break;
  case 3:	DRAWCURVE=1; DRAWPOLYGON=0;
  		DRAWPOLYVGRAPH=0; DRAWVGRAPH=0;		break;
  case 4:	DRAWPOLYVGRAPH=1; DRAWVGRAPH=0; 
  		DRAWCURVE=0;				break;
  case 5:  	DRAWVGRAPH=1; DRAWPOLYVGRAPH=0;		
  		DRAWCURVE=1; DRAWPOLYGON=0;		break;
  case 6:	DRAWPOLYPATH = !DRAWPOLYPATH;		break;
  case 7:	DRAWPATH = !DRAWPATH;			break;
  default:   						break;
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

  if (SOURCEDEST)
   {
    glColor3fv (Red);
    glBegin(GL_POINTS);
    glVertex2f (source[0], source[1]); 
    glVertex2f (dest[0], dest[1]);
    glEnd();
   }
  if (DRAWPOLYGON)
   { 
    glColor3fv(Chocolate);  
    for (i=0; i<obstacle.getn(); i++) 	obstaclePoly[i].draw(1); 
   }
  if (DRAWCURVE)
   {
    glColor3fv (Red); 
    for (i=0; i<obstacle.getn(); i++) 	obstacle[i].draw();
   }
  if (DRAWPOLYVGRAPH)
   {
    glColor3fv (Black);
    vgraph.draw();
   } 
  if (DRAWVGRAPH)
   {
    glColor3fv (Blue);
    svgraph.drawVG();
   }
  if (DRAWPOLYPATH)
   {
    glColor3fv (Black);
    glLineWidth(2.0);
    vgraph.drawPath (polyShortestPath);
    glLineWidth(1.0);
   }
  if (DRAWPATH)
   {
    glColor3fv (Blue);
    glLineWidth(2.0);
    svgraph.drawPath (shortestPath);
    glLineWidth(1.0);
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
  float     eps = .0001;	// accuracy of intersection computation
  float     sampleeps = .25;	// sampling rate (one point per sampleeps) for polygon
  		// sample rate of .05 illustrates time extreme to construct polygonal visgraph

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'S': source[0] = atof(argv[ArgsParsed++]);
      		source[1] = atof(argv[ArgsParsed++]);		break;
      case 'D': dest[0] = atof(argv[ArgsParsed++]);
      		dest[1] = atof(argv[ArgsParsed++]);		break;
      case 's': sampleeps = atof(argv[ArgsParsed++]);		break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
    else ArgsParsed++;
   }  
  
  inputCurves(argv[argc-1], obstacle);
  obstaclePoly.allocate(obstacle.getn());  FloatArr foo;
  for (int i=0; i<obstacle.getn(); i++)	// generate polygons by sampling curves
    obstacle[i].uniformSample (sampleeps, obstaclePoly[i], foo, 0);
  svgraph.create (obstacle, eps);
  svgraph.updateSourceDest (source, dest);
  vgraph.create (obstaclePoly);
  vgraph.updateSourceDest(source, dest);
  printInfo();
  
  cout << "See 9-vgraph/software/svgraph.c++, end of main, for suggestion" << endl
       << "how to analyze improvement of smooth vgraph over polygonal vgraph." << endl;
   
  /************************************************************/

  glutInit (&argc, argv);			
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int titleht = 20; 	// top titlebar is 20 units high
  int xleft   = 164;	// x-coord of lefthand side for small windows
  int xsize = 544, ysize = 544;

  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100];
  strcpy (titlebar, "Visibility graph (");  
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
  glutAddMenuEntry ("Source/destination", 	  1);
  glutAddMenuEntry ("Polygons", 		  2);
  glutAddMenuEntry ("Curves", 			  3);
  glutAddMenuEntry ("Polygonal visibility graph", 4);
  glutAddMenuEntry ("Smooth visibility graph", 	  5);
  glutAddMenuEntry ("Polygonal shortest path", 	  6);
  glutAddMenuEntry ("Smooth shortest path", 	  7);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
