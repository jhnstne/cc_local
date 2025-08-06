/*
  File:          svgraph.c++ 
  Author:        J.K. Johnstone 
  Created:	 30 August 2000 (from dual.c++)
  Last Modified: 24 May 2001
  Purpose:       Compute a smooth visibility graph amongst
		 curved obstacles in the plane, and shortest path 
		 between source and destination.
		 Source and destination can be controlled by the mouse.
  Input: 	 n obstacles.  Each obstacle is represented by an array
		 of sample points, defining a closed Bezier spline
		 through interpolation.
  Output: 	 All bitangents (including invisible ones) of curves
  		 and smooth visibility graph.
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
#include "Miscellany.h"		// readComment, countPtSets
#include "Contour.h"
#include "Vector.h"
#include "Polygon.h"
#include "BezierCurve.h"
#include "RatBezierCurve.h"
#include "TangCurve.h"
#include "Vgraph.h"
#include "Matrix.h"

#define XWINDOWSIZE	 545	
#define YWINDOWSIZE	 545
#define ORTHOSIZE	 2.0	 // amount of world viewed in orthograph projection
#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-e eps]  (accuracy at which intersections are made: default .0001)" << endl;
  cout << "\t[-s sampleeps] (sampling rate: default .05)" << endl;
  cout << "\t[-d displaydensity of Bezier segment] (default 10)" << endl;
  cout << "\t[-j] (just display obstacles)" << endl;
  cout << "\t[-n] (don't output V-graph to file)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx;		// previous value of MOUSEX and MOUSEY
static int  	 xwindowsize = XWINDOWSIZE;
static int	 ywindowsize = YWINDOWSIZE;

static GLboolean JUSTVIEWING=0;		// just viewing: don't process further
static GLboolean OUTFILE=1;		// output processed data to new file?
static GLboolean DRAWPOLYGONOB=0;	// draw input polygonal obstacles?
static GLboolean DRAWCURVEOB=1;		// draw curved obstacles?
// static GLboolean DRAWCOMMONTANG=0;	// draw common tangents?
// static GLboolean DRAWVISIBLETANG=0;	// draw visible common tangents?
static GLboolean DRAWVGRAPH=0;		// draw visibility graph?
static GLboolean DRAWPATH=0;		// draw shortest path?
static GLboolean DRAWPOLYPATH=0;	// draw polygonal shortest path?
static GLboolean DRAWPOLYVGRAPH=0;	// draw polygonal visibility graph?
static GLboolean SOURCEDEST=1;		// draw source/destination?
static GLboolean rotateOb=1;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'ccw' direction?

// static GLboolean DEBUG = 1;

int 	    	   	n=0;		// # of obstacles
V2f			source,dest;
Array<Polygon2f>   	obstaclePoly;
Array<BezierCurve2f> 	obstacle;
SmoothVisibilityGraph	svgraph;	// smooth visibility graph
VisibilityGraph		vgraph;		// polygonal visibility graph
IntArr			shortestPath;	// from source to destination
IntArr			polyShortestPath; 

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
//glLineStipple (1, 0xAAAA);

  transxob = transyob = 0.0;
  zoomob = 1.;
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
/*
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomob -= (float).01*(x-oldx);
    if (zoomob < 0.0) zoomob = 0.0;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transxob += .01*(x-oldx); 
    if (firsty)  firsty=0; else transyob += .01*(y-oldy); 
   }
  oldx = x;  
  oldy = y;
*/
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1:  rotateOb = !rotateOb;
 	   if (rotateOb) glutIdleFunc (RotateOb); else glutIdleFunc (NULL); break;
//  case 4:  DRAWCOMMONTANG = !DRAWCOMMONTANG;				
//  	   if (DRAWCOMMONTANG) { DRAWVISIBLETANG=0;} 			break;
//  case 5:  DRAWVISIBLETANG = !DRAWVISIBLETANG;
//  	   if (DRAWVISIBLETANG) { DRAWCOMMONTANG=0;} 			break;
  case 7:  DRAWVGRAPH = !DRAWVGRAPH;					break;
  case 8:  DRAWPATH = !DRAWPATH;					break;
  case 9:  DRAWPOLYVGRAPH = !DRAWPOLYVGRAPH;				break;
  case 6:  spinCCW = !spinCCW;						break;
  case 10: DRAWPOLYGONOB = !DRAWPOLYGONOB;				break;
  case 11: DRAWPOLYPATH = !DRAWPOLYPATH;				break;
  case 12: DRAWCURVEOB = !DRAWCURVEOB;					break;
  case 13: SOURCEDEST = !SOURCEDEST;					break;
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
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  if (DRAWPOLYGONOB) 
   { 
    glColor3fv(Chocolate);  for(i=0;i<n;i++) obstaclePoly[i].draw(1); 
   }
  if (DRAWCURVEOB)
   {
    glColor3fv (Red); 
    for (i=0; i<n; i++) obstacle[i].draw();
//    for (i=0; i<n; i++) obstacle[i].drawCtrlPoly();
    if (SOURCEDEST)
     {
      glBegin(GL_POINTS); 
      glVertex2f (source[0], 	source[1]); 
      glVertex2f (dest[0], dest[1]);
      glEnd();
     }
   }
/*  if (DRAWCOMMONTANG)
*   {
*    glColor3fv (Black);
*    for (i=0; i<(SOURCEDEST?n+1:n-1); i++)
*      for (j=i; j<(SOURCEDEST?n+2:n); j++)
*        for (int k=0; k<commonTang[i*(n+2)+j].getn(); k++)
*	  commonTang[i*(n+2)+j][k].draw (obstacle,source,dest);
*   }
*  if (DRAWVISIBLETANG)
*   {
*    glColor3fv (Black);
*    for (i=0; i<(SOURCEDEST?n+1:n-1); i++)
*      for (j=i; j<(SOURCEDEST?n+2:n); j++)
*        for (int k=0; k<visTang[i*(n+2)+j].getn(); k++)
*	  visTang[i*(n+2)+j][k].draw (obstacle,source,dest);
*   } */
  if (DRAWVGRAPH)
   {
    glColor3fv (Black);
    svgraph.drawVG();
   }
  if (DRAWPOLYVGRAPH)
   {
    glColor3fv (Blue);
    vgraph.draw();
   }
  if (DRAWPATH)
   {
    glColor3fv (Black);
    glLineWidth(2.0);
    svgraph.drawPath (shortestPath);
    glLineWidth(1.0);
   }
  if (DRAWPOLYPATH)
   {
    glColor3fv (Blue);
    glLineWidth(2.0);
    vgraph.drawPath (polyShortestPath);
    glLineWidth(1.0);
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

void inputCurves()
{
  ifstream infile(argv[argc-1]);			// open file
  string comment;  readComment (infile, comment);	// read comment
  n = countPtSets (infile);  V2fArrArr Pt(n); 
cout << n << " obstacles" << endl;
  for (int i=0; i<n; i++) 				// read each point set
   {
    getLeftBrace(infile);
    int mark = infile.tellg();  int nPt=0; V2f foo;	// count the points
    while (!tryToGetRightBrace(infile))
     { 
      infile >> foo[0] >> foo[1];     nPt++;
     }
    assert(nPt>0);  Pt[i].allocate(nPt);
    infile.seekg(mark);
    int smaller=0;	// did we shrink the point set?
    infile >> Pt[i][0][0] >> Pt[i][0][1];
    for (int j=1; j<nPt; j++)
     {
      infile >> Pt[i][j][0] >> Pt[i][j][1];
      if (Pt[i][j]==Pt[i][j-1]) { j--; nPt--; smaller=1; }	// skip duplicate
      if (j==nPt-1 && Pt[i][j]==Pt[i][0]) { nPt--; smaller=1; }	// skip duplicate at end, too
     }
    getRightBrace(infile);
    if (smaller)		// point set shrinked, so redefine smaller set
     {
      V2fArr NewPt(Pt[i]);
      Pt[i].allocate(nPt);
      for (j=0; j<nPt; j++)  Pt[i][j] = NewPt[j];
     }
   }
  infile >> source[0] >> source[1] >> dest[0] >> dest[1];// read source and destination pts
  infile.close();
  obstacle.allocate(n);		// generate curved obstacles by interpolation
  for (i=0; i<n; i++)  
   {
    obstacle[i].fitClosed (Pt[i]);
    obstacle[i].prepareDisplay (nPtsPerSegment*3);
   }
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
  float     eps = .0001;	// accuracy of intersection computation
  float     sampleeps = .05;	// sampling rate (one point per sampleeps) for polygon

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
      case 's': sampleeps = atof(argv[ArgsParsed++]);		break;
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
    else ArgsParsed++;
   }  
  
  inputCurves();	// read in point sets and interpolate
//  else
//    while right button not pressed (ending all input)
//      accept mouse input of points ended by pressing of middle button
 
  if (!JUSTVIEWING)
   {
    cout << "Creating smooth visibility graph" << endl;
    svgraph.create (obstacle, eps);
    svgraph.updateSourceDest (source, dest);
   }  
   
  if (OUTFILE)
   {
    string filePrefix(argv[argc-1]);	// find file prefix, for output file
    string::size_type pos = filePrefix.find(".");
    filePrefix.erase (pos);
    string out = filePrefix + ".vgraph";
    ofstream outfile(out.c_str());	// print out bitangents and visibility graph
    svgraph.printAllCommonTang (outfile);
    svgraph.printSVGraph (outfile);
    outfile.close();
   }

////////////////////////////////////////////////////
  // REMOVE FOLLOWING FOR RELEASE
////////////////////////////////////////////////////

  // try sample rate of .05 on vg1.pts to show extreme of time to construct polygonal visibility graph  
  // POLYGONAL VISIBILITY GRAPH
  obstaclePoly.allocate(n);	// sample the curves to generate polygons
  for (i=0; i<n; i++)
   {
    FloatArr foo; 
    obstacle[i].uniformSample (sampleeps, obstaclePoly[i], foo, 0);
   }
  if (!JUSTVIEWING)
   {
    cout << "Creating polygonal visibility graph" << endl;
    vgraph.create (obstaclePoly);
    vgraph.updateSourceDest(source, dest);
    printInfo();
   }

//////////////////////////////////////

    // ANALYZE RESULTS
    // compute difference between points of tangency on polygonal vgraph
    // and smooth vgraph for the obstacles along the shortest paths; 
    // I think this is the best measure of the improvement in smooth graph
/*    while (i<shortestPath.getn())
     {
      home obstacle of each vertex on smooth path is easy to calculate using 'start'
      find first point on next obstacle on smooth path
      find first point on next obstacle on polygonal path
      find last point on next obstacle on smooth path
      find last point on next obstacle on polygonal path
      cout << "Discrepancy in incoming point to obstacle " << --- << " is " 
      	   << --- << endl;
      cout << "Discrepancy in outgoing point to obstacle " << --- << " is " 
      	   << --- << endl;
     } */
   
  /************************************************************/

  glutInit (&argc, argv);			
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);

  // top titlebar is 20 units high
  glutInitWindowPosition (164,20);
  glutInitWindowSize (544,544);
  char titlebar[100]; 
  strcpy (titlebar, "Visibility graph (");  
  strcat (titlebar, filePrefix.c_str());  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
//  glutAddMenuEntry ("Common tangents", 4);
//  glutAddMenuEntry ("Visible common tangents", 5);
  glutAddMenuEntry ("Smooth visibility graph", 7);
  glutAddMenuEntry ("Polygonal visibility graph", 9);
  glutAddMenuEntry ("Smooth shortest path", 8);
  glutAddMenuEntry ("Polygonal shortest path", 11);
  glutAddMenuEntry ("Polygons", 10);
  glutAddMenuEntry ("Curves", 12);
  glutAddMenuEntry ("Source/destination", 13);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
