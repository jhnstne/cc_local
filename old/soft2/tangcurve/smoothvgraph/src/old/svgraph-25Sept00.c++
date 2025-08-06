/*
  File:          svgraph.c++ 
  Author:        J.K. Johnstone 
  Created:	 30 August 2000 (from dual.c++)
  Last Modified: 10 September 2000
  Purpose:       Compute a (smooth) visibility graph amongst
		 curved obstacles in the plane.
		 Then interactively compute shortest paths between a source and
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
#include "TangCurve.h"
#include "Graph.h"

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

static GLfloat   transxob, transyob, zoomob;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY

static GLboolean JUSTVIEWING=0;		// just viewing: don't process further
// static GLboolean NEWFILE=1;		// output processed data to new file?
static GLboolean DRAWPOLYGONOB=0;	// draw input polygonal obstacles?
static GLboolean DRAWCURVEOB=1;		// draw curved obstacles?
static GLboolean DRAWCOMMONTANG=0;	// draw common tangents?
static GLboolean DRAWVISIBLETANG=0;	// draw visible common tangents?
static GLboolean DRAWVGRAPH=0;		// draw visibility graph?
static GLboolean DRAWPATH=0;		// draw shortest path?
static GLboolean DRAWPOLYVGRAPH=0;	// draw polygonal visibility graph?
static GLboolean SOURCEDEST=1;		// draw source/destination?
static GLboolean rotateOb=1;		// start rotating tangent on 1st curve?
static GLboolean spinCCW=1;		// spin in 'ccw' direction?

// static GLboolean DEBUG = 1;

int 	    	   	n=0;		// # of obstacles
V2f			source,destination;
Array<Polygon2f>   	obstaclePoly;
Array<BezierCurve2f> 	obstacle;
Array<CommonTangentArr> commonTang; 	// common tangs for each pair of curves
Array<CommonTangentArr> visTang; 	// visible common tangs for each pair of curves
EuclideanGraph<V2f>	svgraph;	// smooth visibility graph
IntArr			shortestPath;	// from source to destination
VisibilityGraph		vgraph;		// polygonal visibility graph

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

void menuOb (int value)
{
  switch (value) {
  case 1:  rotateOb = !rotateOb;
 	   if (rotateOb) glutIdleFunc (RotateOb); else glutIdleFunc (NULL); break;
  case 4:  DRAWCOMMONTANG = !DRAWCOMMONTANG;				
  	   if (DRAWCOMMONTANG) { DRAWVISIBLETANG=0;} 			break;
  case 5:  DRAWVISIBLETANG = !DRAWVISIBLETANG;
  	   if (DRAWVISIBLETANG) { DRAWCOMMONTANG=0;} 			break;
  case 7:  DRAWVGRAPH = !DRAWVGRAPH;					break;
  case 8:  DRAWPATH = !DRAWPATH;					break;
  case 9:  DRAWPOLYVGRAPH = !DRAWPOLYVGRAPH;				break;
  case 6:  spinCCW = !spinCCW;						break;
  case 10: DRAWPOLYGONOB = !DRAWPOLYGONOB;				break;
  case 13: SOURCEDEST = !SOURCEDEST;					break;
  default:   								break;
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
      glVertex2f (destination[0], destination[1]);
      glEnd();
     }
   }
  if (DRAWCOMMONTANG)
   {
    glColor3fv (Black);
    for (i=0; i<(SOURCEDEST?n+1:n-1); i++)
      for (j=i; j<(SOURCEDEST?n+2:n); j++)
        for (int k=0; k<commonTang[i*(n+2)+j].getn(); k++)
	  commonTang[i*(n+2)+j][k].draw (obstacle,source,destination);
   }  
  if (DRAWVISIBLETANG)
   {
    glColor3fv (Black);
    for (i=0; i<(SOURCEDEST?n+1:n-1); i++)
      for (j=i; j<(SOURCEDEST?n+2:n); j++)
        for (int k=0; k<visTang[i*(n+2)+j].getn(); k++)
	  visTang[i*(n+2)+j][k].draw (obstacle,source,destination);
   }
  if (DRAWVGRAPH)
   {
    glColor3fv (Black);
    svgraph.draw();
   }
  if (DRAWPATH)
   {
    glColor3fv (Black);
    glLineWidth(2.0);
    svgraph.drawPath (shortestPath);
    glLineWidth(1.0);
   }
  if (DRAWPOLYVGRAPH)
   {
    glColor3fv (Blue);
    vgraph.draw();
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/***************************INPUT***************************************************/
/******************************************************************************/

void input(char *filename, int nPtsPerSegment, string &filePrefix)
{
  ifstream infile;  infile.open(filename);
  // read source and destination of path
  infile >> source[0] 	   >> source[1];
  infile >> destination[0] >> destination[1];
  
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
cout << "Calling fit from svgraph" << endl;
    obstacle[i].myfit (obstaclePoly[i]);
cout << "Obstacle " << i << endl;
obstacle[i].print();
    obstacle[i].prepareDisplay (nPtsPerSegment*3);
   }
}

/******************************************************************************
	Find parameter value t in par's list.
******************************************************************************/

void search (float t, FloatArr &par, int &offset)
{
cout << "Searching for parameter value " << t << endl;
  offset=0;
  while (par[offset] != t)  	offset++;
cout << "Found it at location " << offset << endl;
}

      
/******************************************************************************
    Filter knots of ob from the list of parameter values in p, yielding filterp.
    Computing set difference: p - ob.knot.
    Walk through p, only gathering value if it is not a knot.
******************************************************************************/

void filterKnots (FloatArr &p, BezierCurve2f &ob, FloatArr &filterp)
{
  int nFilter=0;  int obi=0;
  for (int i=0; i<p.getn(); i++)
   {
    while (obi<ob.getnKnot() && ob.getKnot(obi) < p[i])		obi++;
    if (obi<ob.getnKnot() && ob.getKnot(obi) != p[i])  nFilter++;
   }
  filterp.allocate(nFilter);  nFilter=0;
  for (i=0; i<p.getn(); i++)
   {
    while (obi<ob.getnKnot() && ob.getKnot(obi) < p[i])		obi++;
    if (obi<ob.getnKnot() && ob.getKnot(obi) != p[i])  filterp[nFilter++]=p[i];
   }
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int 	    j,k;
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
//    case 'n': NEWFILE=0; 					break;
      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }  
  
  string    filePrefix;
  input(argv[argc-1], nPtsPerSegment, filePrefix);
cout << n << " obstacles" << endl;
  
  if (!JUSTVIEWING)
   {
    // compute all tangential curves
    Array<TangentialCurve> tangCurve;	// tangential curve packages of obstacles
    tangCurve.allocate(n+2);		// n obstacles and 2 poles
    for (int i=0; i<n; i++) 
     {
cout << "Computing tangential curve " << i << endl;
      tangCurve[i].create (obstacle[i], i);
     }
cout << "Computing tangential curve for source" << endl;
    tangCurve[n].create   (source, -1);	 
cout << "Computing tangential curve for destination" << endl;
    tangCurve[n+1].create (destination, -2); 
    
    // compute common tangents for each curve pair
    commonTang.allocate ((n+1)*(n+2));
    for (i=0; i<n; i++)
     {
cout << "Computing common tangents from " << i << endl;
      tangCurve[i].selfIntersect (commonTang[i*(n+2)+i], eps);
      for (j=i+1; j<n+2; j++)
       {
cout << "	to " << j << endl;
	tangCurve[i].intersect (tangCurve[j], commonTang[i*(n+2)+j], eps);
       }
     }
    // 'common tangent' from source to destination
    i=n*(n+2)+n+1;
    commonTang[i].allocate(1);
    commonTang[i][0].index1 = -1;	// source
    commonTang[i][0].index2 = -2;	// destination
    commonTang[i][0].param1 = commonTang[i][0].param2 = 0;	// irrelevant
    commonTang[n*(n+2)+n].allocate(0);	// place-holder for (n,n) pair
	  
    // cull invisible common tangents, using all obstacles
    visTang.allocate ((n+1)*(n+2));
    for (i=0; i<n+1; i++)
      for (j=i; j<n+2; j++)
       {
cout << "Culling from (" << i << "," << j << ")" << endl;
        IntArr vis(commonTang[i*(n+2)+j].getn());  // records visibility of each tang
	int nVis=0;
	for (k=0; k<commonTang[i*(n+2)+j].getn(); k++)
	  if (vis[k] = commonTang[i*(n+2)+j][k].visible (obstacle, source, destination))
	    nVis++;
	visTang[i*(n+2)+j].allocate(nVis);    
	nVis=0;
	for (k=0; k<commonTang[i*(n+2)+j].getn(); k++)
	  if (vis[k])
	    visTang[i*(n+2)+j][nVis++] = commonTang[i*(n+2)+j][k];
       }
       
/*   building the visibility graph (once all common tangents are known)
     
     for each obstacle,
       gather all parameter values of incoming (impinging) tangents
       	 	(float array just of parameter values)
       sort them by parameter value
       make a node for each parameter value (storing (x,y) only in node)
       		(and maintaining the sorted array of parameter values so that
		 a parameter value can be translated into a node index)
       create 'self-edge' (representing curved arc on obstacle, not straight line path)
       		between nodes with neighbouring parameter values 
       associate all these nodes with the obstacle, by maintaining
       	a master list of the beginning index of each obstacle's nodes
	(which are contiguous in the list of nodes) and the sorted list
	for each obstacle
       
     once all nodes have been made, for all obstacles:
       create edge for every incoming tangent, by finding node associated
       with every parameter value in its obstacle's collection
*/

/*	(1) For each obstacle, sort parameter values of all incoming tangents.
	(2) Build the associated array of nodes
	(3) Build edges
	  (a) traverse all common tangents, finding associated edge and
	  	assigning it a cost
	  (b) build self-edges between nodes with neighbouring parameter values
	  	on the same obstacle
		(these can be recognized by an edge (i,i+1) where i+1 is not one of
			the start positions of an obstacle's parameter values,
			or (i,j) where j is a start[x] and i is start[x+1]-1)
*/

cout << "Building visibility graph" << endl;
    Array<FloatArr> param(n);	// param[i] = parameter values of tangent endpts 
    				// 	      on obstacle[i]
    for (i=0; i<n; i++)
     {
      int nParam = 0;		// count all common tangents into obstacle[i]
      for (j=0; j<i; j++)
        nParam += visTang[j*(n+2)+i].getn();	// tangents from j to i
      nParam += 2 * visTang[i*(n+2)+i].getn();	// self-tangents
      for (j=i+1; j<n+2; j++)
        nParam += visTang[i*(n+2)+j].getn();	// tangents from i to j
      param[i].allocate (nParam);
      nParam = 0;  // store parameter values of common tangents from obstacle[i]
      for (j=0; j<i; j++)
        for (k=0; k<visTang[j*(n+2)+i].getn(); k++)
	  param[i][nParam++] = visTang[j*(n+2)+i][k].param2;
      for (k=0; k<visTang[i*(n+2)+i].getn(); k++)	// self-tangents
       {
        param[i][nParam++] = visTang[i*(n+2)+i][k].param1;
        param[i][nParam++] = visTang[i*(n+2)+i][k].param2;
       }
      for (j=i+1; j<n+2; j++)
        for (k=0; k<visTang[i*(n+2)+j].getn(); k++)
	  param[i][nParam++] = visTang[i*(n+2)+j][k].param1;
      IntArr foo;	param[i].bubbleSort (foo);	// sort
      // remove duplicates (CAN THERE BE DUPLICATES?)
      // shouldn't be a problem even if there are duplicates: length 0 self-edge between the duplicates
cout << "Tangent parameters on obstacle " << i << ":" << endl;
param[i].print();
     }
    IntArr start(n);	// start[i] = index of 1st node assoc with obstacle[i]
    start[0] = 0;
    for (i=1; i<n; i++)  start[i] = start[i-1] + param[i-1].getn();
    int nVert=0;  for (i=0; i<n; i++) nVert += param[i].getn();
    nVert += 2;		// source and destination too
    V2fArr vert(nVert);	// build all vertices of visibility graph
    nVert=0;
    for (i=0; i<n; i++)
      for (j=0; j<param[i].getn(); j++)
        obstacle[i].eval (param[i][j], vert[nVert++]);
    vert[nVert++] = source;
    vert[nVert++] = destination;
    float **edge;  edge = new float*[nVert]; 		// build edges
    for (i=0; i<nVert; i++) edge[i] = new float[nVert];
    for (i=0; i<nVert; i++) for (j=0; j<nVert; j++) edge[i][j] = -1; // initialize to infinity
    for (i=0; i<n; i++)		// traverse all common tangents to set edges
     {
      for (j=i; j<n+2; j++)
        for (k=0; k<visTang[i*(n+2)+j].getn(); k++)
	 {
	  CommonTangent t = visTang[i*(n+2)+j][k];  // from i to j
cout << "Adding edge for common tangent ";  t.print();
	  int p,q,offset; search (t.param1, param[i], offset);
	  p = start[i] + offset;
	  if (j<n)
	   {
	    search (t.param2, param[j], offset);
	    q = start[j] + offset;
	   }
	  else if (j==n) q = nVert-2;	// source 
	  else 		 q = nVert-1;	// destination
	  edge[p][q] = edge[q][p] = vert[p].dist (vert[q]);
	 }
     }
    if (visTang[n*(n+2)+n+1].getn() == 1)	// source-destination edge
      edge[nVert-2][nVert-1] = edge[nVert-1][nVert-2] = source.dist (destination);

cout << "Building self-edges" << endl;    
    for (i=0; i<n; i++)		// build self-edges
     {
cout << "Obstacle " << i << endl;
      // subdivide ith obstacle at parameter values of tangent endpoints that are not already knots
      FloatArr filterParam;	// param[i] minus any knots it may contain (that don't need to be added again)
      filterKnots (param[i], obstacle[i], filterParam);
cout << "Filtered param: ";  filterParam.print();
      obstacle[i].subdivideSpline (filterParam);
      for (j=0; j<param[i].getn()-1; j++) // build self-edge from param[j] 
       {				  // to param[j+1]
        int p = start[i] + j;		// consecutive nodes on same obstacle
	int q = start[i] + j+1;
	BezierCurve2f selfSeg;		// curve segment for this self-edge
        obstacle[i].extract (param[i][j], param[i][j+1], selfSeg);
// cout << "Self-edge from " << j << " to " << j+1 << endl;
// selfSeg.print();	
        edge[p][q] = edge[q][p] = selfSeg.arcLength (.001);
// cout << "Edge length = " << edge[p][q] << endl << endl;
       }
      // build self-edge from param[param[i].getn()-1] to param[0]
      j = param[i].getn()-1;
      int p = start[i] + j;
      int q = start[i];
      BezierCurve2f selfSeg1, selfSeg2;	float dist1, dist2;
      if (param[i][j] != obstacle[i].getLastKnot())
       {
cout << endl << "last tangent to end: ";
        obstacle[i].extract (param[i][j], obstacle[i].getLastKnot(), selfSeg1);
	dist1 = selfSeg1.arcLength(.001);
       }
      else dist1 = 0;
      if (obstacle[i].getKnot(0) != param[i][0])
       {
cout << "beginning to first tangent: ";
        obstacle[i].extract (obstacle[i].getKnot(0), param[i][0],    selfSeg2);
	dist2 = selfSeg2.arcLength(.001);
       }
      else dist2 = 0;
      edge[p][q] = edge[q][p] = dist1 + dist2;
     }
    svgraph.create(edge, vert);
// svgraph.print();
    cout << "Smooth visibility graph has " << svgraph.getn() << " vertices and " 
    		<< svgraph.getE() << " edges." << endl;
    svgraph.dijkstra (nVert-2,nVert-1,shortestPath);
// cout << "Shortest path: ";
// shortestPath.print();

    vgraph.create (obstaclePoly);
    vgraph.updateSourceDest(source, destination);
    cout << "Polygonal visibility graph has " << vgraph.getn() << " vertices and " 
    		<< vgraph.getE() << " edges." << endl;
   }
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
  glutAddMenuEntry ("Common tangents", 4);
  glutAddMenuEntry ("Visible common tangents", 5);
  glutAddMenuEntry ("Visibility graph", 7);
  glutAddMenuEntry ("Shortest path", 8);
  glutAddMenuEntry ("Polygons", 10);
  glutAddMenuEntry ("Polygonal visibility graph", 9);
  glutAddMenuEntry ("Source/destination", 13);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
