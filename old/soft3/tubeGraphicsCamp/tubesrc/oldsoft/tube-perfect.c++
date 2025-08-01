/*
  File: 	 tube.c++
  Author:	 J.K. Johnstone
  Created:	 24 April 2000
  Last Modified: 17 May 2000
  Purpose:	 Geometric modeling of a tube.
  		 A project for Computer Camp 2000.
  History:	 
*/

#include <GL/glut.h>
#include <GL/glu.h>
#include <fstream.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>
#include <assert.h>

#include "AllColor.h"

#define MAXCIRCPT	200
#define UNIFORM		0
#define CHORDLENGTH	1
#define CENTRIPETAL	2
#define DENSITY		10

// translate from radians to degrees
inline float rad2deg (float theta) { return (theta * 180/M_PI); }
inline float max (float a, float b) { return (a>b ? a : b); }

/******************************************************************************/
/******************************************************************************/

class V3f {

  public:
  
    V3f()	{ v[0] = v[1] = v[2] = 0; }
    V3f(float a, float b, float c) { v[0]=a; v[1]=b; v[2]=c; }
    V3f&   operator=(const V3f&);
    int    operator==(V3f&);
    float& operator[](int index) {assert(index>=0 && index<3); return v[index];}
    void   operator+= (V3f& rhs)  { for (int i=0;i<3;i++) v[i] += rhs[i]; }
    void   operator*= (float rhs) { for (int i=0;i<3;i++) v[i] *= rhs; }
    float  length() { return(sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])); }
    void   normalize();
    void   cross (V3f &a, V3f &b);

  private:
  
    float   v[3];

};

V3f& V3f::operator= (const V3f &a)
{
  if (this == &a) return *this;
  for (int i=0; i<3; i++) v[i] = a.v[i];
  return *this;
}

int V3f::operator== (V3f &a)
{
  for (int i=0; i<3; i++)
    if (v[i] != a[i]) return(0);
  return(1);
}

void V3f::normalize()
{
  float len = length();
  for (int i=0; i<3; i++) 
    v[i] /= len;
}

void V3f::cross (V3f &a, V3f &b)
{
  v[0] = a[1]*b[2] - a[2]*b[1];
  v[1] = a[2]*b[0] - a[0]*b[2];
  v[2] = a[0]*b[1] - a[1]*b[0];
}

/******************************************************************************/
/******************************************************************************/

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-u] uniform parameterization (default is centripetal)" << endl;
  cout << "\t[-c] chordlength parameterization" << endl;
  cout << "\t[-t] estimate tangents simply" << endl;
  cout << "\t[-d #] set density (default = 10)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <data file>" << endl;
 }
 
int nPt;		// # of data points
V3f *pt;		// data points for spine of tube
V3f *tang;		// estimated tangents at data points
int param=CENTRIPETAL;	// parameterization type
V3f *ctrlPt;		// control points of interpolating Bezier curve
float *knot;		// knots (parameter values at data points) of curve
int density=DENSITY;	// sampling density (# of samples per segment)
int nSample;		// # of samples
V3f *sample;		// samples on Bezier curve (for display)
V3f *sampleTang;	// intermediate tangents
V3f **sampleCirc;	// intermediate circles
int nCircPt=20;		// # of sample points on canonical circle
V3f *circle;		// canonical circle
float tuberadius=.1;	// radius of tube

static GLfloat   transx, transy, transz, rotx, roty, rotz, zoom;
static int 	 panLeft=0;  			// control panning for 3d effect
static int 	 panRight=1;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1;	// first MOUSEX reading?
static GLboolean firsty=1;
static int	 oldx,oldy;	// previous value of MOUSEX and MOUSEY
static GLboolean ROTATE=0;	// start rotating?
static GLboolean PAN=0;	 	// rotate object back and forth for 3d effect?
static GLboolean DRAWDATAPT=1;	// draw data points?
static GLboolean DRAWTANG=0;	// draw estimated tangents?
static GLboolean SIMPLETANG=0;	// estimate tangents simply?
static GLboolean DRAWDATACIRCLE=0; // draw circles at data points?
static GLboolean DRAWCTRLPOLY=0; // draw control polygon?
static GLboolean DRAWCURVE=0;	// draw curve?
static GLboolean DRAWINTERMEDTANG=0; // draw intermediate tangents?
static GLboolean DRAWINTERMEDCIRCLE=0; // draw intermediate circles?
static GLboolean DRAWTRIANG=0; 	// draw triangulation?
static GLboolean DRAWWIRE=0;	// draw wireframe (rather than filled)? 

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  GLfloat redplastic[]  = {0.8, 0.1, 0.1, 1.0};
  GLfloat redplastic_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat redplastic_shiny[]    = {60.0};		// set it high!
  GLfloat lmodel_ambient[] = { 0.5, 0.5, 0.5, 0.0 };	// increase ambient light
  GLfloat light_diffuse[]   = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_specular[]  = {1.0, 1.0, 1.0, 1.0};

  glClearColor (1.0, 1.0, 1.0, 0.0);
  glShadeModel (GL_SMOOTH);
  
  glMaterialfv (GL_FRONT, GL_AMBIENT_AND_DIFFUSE,   redplastic);
  glMaterialfv (GL_FRONT, GL_SPECULAR,  redplastic_specular);
  glMaterialfv (GL_FRONT, GL_SHININESS, redplastic_shiny);
  glLightModelfv (GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightfv (GL_LIGHT1, GL_DIFFUSE, light_diffuse);	// default is 0 for 2+ lights
  glLightfv (GL_LIGHT2, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT1, GL_SPECULAR, light_specular);
  glLightfv (GL_LIGHT2, GL_SPECULAR, light_specular);

  glEnable(GL_COLOR_MATERIAL);		// change material color through glColor
  glColorMaterial (GL_FRONT_AND_BACK, GL_DIFFUSE);

  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHT2);
  glEnable(GL_DEPTH_TEST);

  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable (GL_POINT_SMOOTH); 			
  glHint (GL_POINT_SMOOTH_HINT, GL_NICEST); 
  glPointSize (4.0);
  
  transx = 0.0;  transy = 0.0;  transz = 0.0;
  rotx = 0;		
  roty = 0;
  rotz = 0;
  zoom = 1.;
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (w <= h)
    glOrtho(-2.0, 2.0, -2.0*(GLfloat)h/(GLfloat)w, 2.0*(GLfloat)h/(GLfloat)w, -600, 600);
  else
    glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -600, 600);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void Rotate (void)
{
  rotz += 0.5; 
  if (rotz > 360.0) rotz -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void Pan (void)
{
  if (panLeft)
   {
    rotz += 0.1;
    if (rotz > 360.0) rotz -= 360.0;
    panLeft++;
    if (panLeft==200) { panLeft=0; panRight=1; }
   }
  else if (panRight)
   {
    rotz -= 0.1;
    if (rotz < 0.0) rotz += 360.0;
    panRight++;
    if (panRight==200) { panRight=0; panLeft=1; }
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) {
    if (ROTATE || PAN)  glutIdleFunc (NULL);
  }
  else if (ROTATE)      glutIdleFunc (Rotate);
  else if (PAN)         glutIdleFunc (Pan);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); break;					// ESCAPE
  case 'p':	PAN = !PAN;					// toggle pan
		if (PAN) glutIdleFunc (Pan); 
		else glutIdleFunc (NULL); 		break;
  case 'r':	ROTATE = !ROTATE;				// toggle rotation
		if (ROTATE) glutIdleFunc (Rotate); 
		else glutIdleFunc (NULL); 		break;
  case '1':     DRAWDATAPT = !DRAWDATAPT;		break;
  case '2':	DRAWTANG = !DRAWTANG;			break;
  case '3': 	DRAWDATACIRCLE = !DRAWDATACIRCLE;	break;
  case '4': 	DRAWCTRLPOLY = !DRAWCTRLPOLY;		break;
  case 'l': 	rotx = roty = rotz = 0.0; 		break;	// look down
  case 'u':     rotx+=180;				break;  // turn upside down
  case ' ':     // display next circle
							break;
  case 8:       // display previous circle
							break;
  default:      					break;
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

void motion (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoom -= (float).02*(x-oldx);
    if (zoom < 0.0) zoom = 0.0;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transx += .001*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transy += .001*(y-oldy); /* TRANSLATION: Y */
   }
  else if (middleMouseDown) 
   {
    if (firstx)  firstx=0;
    else { roty += .5*(x-oldx); if (roty > 360.0) roty -= 360.0; } /* ORI: Y */

    if (firsty)  firsty=0;
    else { rotx += .5*(y-oldy); if (rotx > 360.0) rotx -= 360.0; } /* ORI: X */
   }
  oldx = x;  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menu (int value)
{
  switch (value) {
    case 1:     DRAWDATAPT 	= !DRAWDATAPT; 			break;
    case 2:	DRAWTANG	= !DRAWTANG;			break;
    case 3:	DRAWDATACIRCLE	= !DRAWDATACIRCLE;		break;
    case 4: 	DRAWCTRLPOLY 	= !DRAWCTRLPOLY;		break;
    case 5:	DRAWCURVE	= !DRAWCURVE;			break;
    case 6: 	DRAWINTERMEDTANG = !DRAWINTERMEDTANG;		break;
    case 7: 	DRAWINTERMEDCIRCLE = !DRAWINTERMEDCIRCLE;	break;
    case 8: 	DRAWTRIANG	= !DRAWTRIANG;			break;
    case 9:	DRAWWIRE	= !DRAWWIRE;			break;
    default:   							break;
    }
  glutPostRedisplay();
}

/******************************************************************************
	Draw a circle with given center, normal and radius
	(normal need not be unit),
	by scaling, rotating, and translating canonical circle.
	Want rotation to move z-axis (normal of canonical circle)
	to the given normal.
	First, rotate z-axis to cone containing desired normal,
	by rotating about x-axis by the angle between the z-axis and normal,
	arccos(e3 . unit normal).
	Next, rotate around this cone to the desired position.
	The present projection of the normal lies on the negative(?) y-axis,
	so we want to rotate about z-axis by the angle between the 
	negative y-axis and the projection of the original normal onto z=0, 
	arccos(-n2 / || (n1,n2,0) ||).
******************************************************************************/

void drawCircle (V3f &center, V3f &normal, float radius)
{
  glPushMatrix();
  // notice that transformations must be done in reverse order
  glTranslatef (center[0], center[1], center[2]);

  double foo = -normal[1] / sqrt(normal[0]*normal[0] + normal[1]*normal[1]);
  glRotatef (rad2deg(acos(foo)), 0., 0., 1.);	 // second, rotate about z-axis
  foo = normal[2]/normal.length();
  glRotatef (rad2deg(acos(foo)), 1.0, 0.0, 0.0); // first, rotate about x-axis

  glScalef  (radius, radius, radius);
  
  glBegin(GL_LINE_LOOP);
  for (int i=0; i<nCircPt; i++)
    glVertex3f (circle[i][0], circle[i][1], circle[i][2]);
  glEnd();

  glPopMatrix();
}

/******************************************************************************
	Compute normal of triangle abc.
	Want float array rather than V3f object for normal, 
	so that it can be directly handed off to glNormal3fv.
******************************************************************************/

void computeTriNormal (V3f &a, V3f &b, V3f &c, float n[3])
{
 V3f ab,ac; for (int i=0; i<3; i++) { ab[i] = b[i]-a[i]; ac[i] = c[i]-a[i]; }
 V3f norm;  norm.cross (ab,ac); norm.normalize();
 for (i=0; i<3; i++) n[i] = norm[i];
}
   
/******************************************************************************
	Triangulate between two circles (with same number of samples).
******************************************************************************/

void drawTriangulation (V3f *c1, V3f *c2)
{
  float norm[3];
  glBegin (GL_TRIANGLES);
  for (int i=0; i<nCircPt; i++)
   {
    computeTriNormal (c1[i], c2[i], c1[(i+1)%nCircPt], norm);
    glNormal3fv (norm);
    glVertex3f (c1[i][0], c1[i][1], c1[i][2]);
    glVertex3f (c2[i][0], c2[i][1], c2[i][2]);
    glVertex3f (c1[(i+1)%nCircPt][0], c1[(i+1)%nCircPt][1], c1[(i+1)%nCircPt][2]);

    computeTriNormal (c2[i], c1[(i+1)%nCircPt], c2[(i+1)%nCircPt], norm);
    glNormal3fv (norm);
    glVertex3f (c2[i][0], c2[i][1], c2[i][2]);
    glVertex3f (c1[(i+1)%nCircPt][0], c1[(i+1)%nCircPt][1], c1[(i+1)%nCircPt][2]);
    glVertex3f (c2[(i+1)%nCircPt][0], c2[(i+1)%nCircPt][1], c2[(i+1)%nCircPt][2]);
   }
  glEnd();
}

/******************************************************************************/
/******************************************************************************/

void display ()
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoom, zoom, zoom);
  glTranslatef (transx, transy, transz);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  
  GLfloat light0_position[] = {1,0,1,0};	// move lights with object, for constant lighting
  GLfloat light1_position[] = {-1,0,0,0};
  GLfloat light2_position[] = {0,-1,1,0};
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
  glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
  glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
  
  if (DRAWWIRE)
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  else
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
  
  int i;
  if (DRAWDATAPT)		// draw data points
   {
    glColor3fv (Black);
    glBegin(GL_POINTS);
    for (i=0; i<nPt; i++)
      glVertex3f (pt[i][0], pt[i][1], pt[i][2]);
    glEnd();
   }
  if (DRAWTANG)			// draw tangents at data points
   {
    glColor3fv (Red);
    glBegin(GL_LINES);
    for (i=0; i<nPt; i++)
     {
      glVertex3f (pt[i][0], pt[i][1], pt[i][2]);
      glVertex3f (pt[i][0] + tang[i][0],
      		  pt[i][1] + tang[i][1], 
		  pt[i][2] + tang[i][2]);
     } 
    glEnd();
   }
  if (DRAWCTRLPOLY)		// draw control polygon
   {
    glColor3fv (Red);
    glBegin(GL_LINE_STRIP);
    for (i=0; i<3*nPt-2; i++)
      glVertex3f (ctrlPt[i][0], ctrlPt[i][1], ctrlPt[i][2]);
    glEnd();
   }
  if (DRAWDATACIRCLE)		// draw circles at data points
   {
    glColor3fv (Black);
    for (i=0; i<nPt; i++)
      drawCircle (pt[i], tang[i], tuberadius);
    // control radius of circle interactively through keyboard!
   }
   
  if (DRAWCURVE)		// draw Bezier curve
   {
    glColor3fv (Black);
    glBegin(GL_LINE_STRIP);
    for (i=0; i<nSample; i++)
      glVertex3f (sample[i][0], sample[i][1], sample[i][2]);
    glEnd();
   }
  if (DRAWINTERMEDTANG)		// draw intermediate tangents
   {
    glColor3fv (Red);
    glBegin(GL_LINES);
    for (i=0; i<nSample; i++)
     {
      glVertex3f (sample[i][0], sample[i][1], sample[i][2]);
      glVertex3f (sample[i][0] + sampleTang[i][0],
      		  sample[i][1] + sampleTang[i][1], 
		  sample[i][2] + sampleTang[i][2]);
     } 
    glEnd();
   }
  if (DRAWINTERMEDCIRCLE)	// draw intermediate circles
   {
    glColor3fv (Black);
    for (i=0; i<nSample; i++)
      drawCircle (sample[i], sampleTang[i], tuberadius);
   }
  if (DRAWTRIANG)		// draw triangulation between circles
   {
    glEnable(GL_LIGHTING);	// this is the only surface we are rendering
    glColor3fv (Blue);
    for (i=0; i<nSample-1; i++)
      drawTriangulation (sampleCirc[i], sampleCirc[i+1]);
    glDisable(GL_LIGHTING);
   }

  glPopMatrix();
  glutSwapBuffers ();
}

/******************************************************************************
******************************************************************************/

float dist (V3f a, V3f b)
{
  return (sqrt( (b[0]-a[0])*(b[0]-a[0]) + 
  		(b[1]-a[1])*(b[1]-a[1]) +
		(b[2]-a[2])*(b[2]-a[2]) ));
}

/******************************************************************************
	Build a collection of n points on the canonical circle, 
	of unit radius, centered at the origin, in the z=0 plane.
******************************************************************************/

void buildCircle (int n, V3f *circle)
{
  double delta = 2*M_PI/n;	// angle increment
  for (int i=0; i<n; i++)
   {
    circle[i][0] = cos(i*delta);
    circle[i][1] = sin(i*delta);
    circle[i][2] = 0;
   }
}

/************************************************************
	Build the 4-level de Casteljau triangle at the 
	parameter value t of the cubic Bezier curve
	defined by control points in ctrlPt.
	tri[0] = 0th level = original control points
	tri[1] = 1st level
	tri[2] = 2nd level = 2 points defining tangent
	tri[3] = 3rd level = 1 point, the curve point at this parameter value
************************************************************/
  
void deCasteljau (V3f *ctrlPt, float t, V3f tri[4][4])
{
  for (int i=0; i<4; i++)
    tri[0][i] = ctrlPt[i];
  for (i=1; i<4; i++)		// for each new level
    for (int j=0; j<4-i; j++)	// for each point in this level
      for (int k=0; k<3; k++)
       {
        tri[i][j][k] = (1-t) * tri[i-1][j][k] + t * tri[i-1][j+1][k];
       }
}

/************************************************************
	Multiply the vector a by the matrix m.
************************************************************/
  
void multMatrix (float m[3][3], V3f &a)
{
  V3f ma;
  for (int i=0; i<3; i++)
   {
    ma[i] = 0;
    for (int j=0; j<3; j++)
      ma[i] += m[i][j] * a[j];
   }
  a = ma;
}
  
/******************************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
  int 	    i,j;
  int       ArgsParsed=0;
  ifstream  infile;
  ofstream  outfile;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'u': param=UNIFORM; 				break;
      case 'c': param=CHORDLENGTH; 			break;
      case 't': SIMPLETANG=1;				break;
      case 'd': density = atoi(argv[ArgsParsed++]); 	break;
      case 'h': 
      default:	usage(); exit(-1);			break;
      }
   else ArgsParsed++;
  }  

  /************************************************************/

  // input points
  infile.open(argv[argc-1]);	// open file
  nPt = 0;			// count the points
  float foo;  while (infile >> foo) { nPt++; infile >> foo >> foo; }
  pt = new V3f[nPt];		// allocate enough room for points
  infile.clear();		// clear the end-of-file flag
  infile.seekg(0);		// return to front of file
  for (i=0; i<nPt; i++)		// read the points
    infile >> pt[i][0] >> pt[i][1] >> pt[i][2];
  infile.close();
  
  /************************************************************/

  // center input data at origin and scale to [-1,1]x[-1,1]x-[-1,1] cube
  V3f minCoord = pt[0], maxCoord = pt[0];	
  for (i=1; i<nPt; i++)
    for (j=0; j<3; j++)
      if (pt[i][j] < minCoord[j])	  minCoord[j] = pt[i][j];
      else if (pt[i][j] > maxCoord[j])    maxCoord[j] = pt[i][j];
  V3f center,scale;
  for (i=0; i<3; i++)
   {
    center[i] = (minCoord[i] + maxCoord[i]) / 2.;
    scale[i]  = (maxCoord[i] - minCoord[i]) / 2.; // e.g., -3,3 is 6: scale by 3
   }
  float maxScale = max (scale[0], max (scale[1], scale[2]));
  for (i=0; i<nPt; i++)
    for (j=0; j<3; j++)
      pt[i][j] = (pt[i][j] - center[j]) / maxScale;
  
  /************************************************************/
  
  // build knots (parameter values at data points): necessary for Bessel tangents
  knot = new float[nPt];
  knot[0] = 0;
  for (i=1; i<nPt; i++)
    switch(param) {
    case UNIFORM: 	knot[i] = knot[i-1] + 1;  			 break;
    case CHORDLENGTH:	knot[i] = knot[i-1] + dist(pt[i-1],pt[i]); 	 break;
    case CENTRIPETAL:   knot[i] = knot[i-1] + sqrt(dist(pt[i-1],pt[i])); break;
    default: 								 break;
    }
  float *delta = new float[nPt-1];	// lengths of parameter intervals
  for (i=0; i<nPt-1; i++)  delta[i] = knot[i+1] - knot[i];
  
  /************************************************************/

  // estimate tangents
  tang = new V3f[nPt];
  if (SIMPLETANG)		// naive simple method
   {
    for (j=0; j<3; j++)			// first: vector to next point
      tang[0][j] = pt[1][j] - pt[0][j];		
    for (i=1; i<nPt-1; i++)		// weighted average of in/out vectors
     {
      float totaldist = dist(pt[i-1],pt[i]) + dist(pt[i],pt[i+1]);
      float alpha     = dist(pt[i-1],pt[i]) / totaldist;
      for (j=0; j<3; j++)
        tang[i][j] = (1 - alpha) * (pt[i][j]   - pt[i-1][j])
		        + alpha  * (pt[i+1][j] - pt[i][j]);
     }
    for (j=0; j<3; j++)			// last: vector from previous point
      tang[nPt-1][j] = pt[nPt-1][j] - pt[nPt-2][j];	
   }
   		/************************************************************/
  else				// Bessel tangents: tangent of fitting parabola
   {			// reference: Farin, p. 103+119; Hoschek/Lasser, p. 147
    for (i=1; i<nPt-1; i++)		// intermediate tangents
     {
      float alpha = delta[i-1] / (delta[i-1] + delta[i]);
      V3f in,out;
      for (j=0; j<3; j++)
       {
        in[j]  = (pt[i][j]   - pt[i-1][j]) / delta[i-1];
	out[j] = (pt[i+1][j] - pt[i][j])   / delta[i];
	tang[i][j] = (1 - alpha) * in[j] + alpha * out[j];
       }
     }
    // end tangents
    int L = nPt-1;
    for (j=0; j<3; j++)
     {
      tang[0][j] = 2*(pt[1][j] - pt[0][j])   / delta[0]   - tang[1][j];
      tang[L][j] = 2*(pt[L][j] - pt[L-1][j]) / delta[L-1] - tang[L-1][j];
     }
   }
  
  /************************************************************/
  
  // build canonical circle (in preparation for circle drawing)
  circle = new V3f[nCircPt];		// allocate points
  buildCircle(nCircPt,circle);

  /************************************************************/
  
  // build Bezier control polygons
  ctrlPt = new V3f[3*(nPt-1)+1];
  for (i=0; i<nPt-1; i++)	// for each segment between data points
    for (j=0; j<3; j++)
     {
      ctrlPt[3*i][j]   = pt[i][j];
      ctrlPt[3*i+1][j] = pt[i][j]   + delta[i]/3 * tang[i][j];
      ctrlPt[3*i+2][j] = pt[i+1][j] - delta[i]/3 * tang[i+1][j];
      if (i==nPt-2)
        ctrlPt[3*i+3][j] = pt[i+1][j];
   }

  /************************************************************/
  
  // build samples on Bezier curve, using de Casteljau algorithm
  nSample = (density-1)*(nPt-1)+1; // 'density' samples per segment
  sample = new V3f[nSample];
  nSample=0;
  for (i=0; i<nPt-1; i++)	// for each segment
   {
    float delta = 1. / (density - 1);   // step size
    sample[nSample++] = pt[i];	// first sample on segment is data point
    for (j=1; j<density-1; j++)	// for each remaining sample
     {
      V3f tri[4][4];
      deCasteljau (&(ctrlPt[3*i]), j*delta, tri);
      sample[nSample++] = tri[3][0];
     }
   }
  sample[nSample-1] = pt[nPt-1];	// last sample = last data point
  
  /************************************************************/
  
  // build intermediate tangents
  sampleTang = new V3f[nSample];
  nSample=0;
  for (i=0; i<nPt-1; i++)
   {
    float delta = 1. / (density - 1);
    sampleTang[nSample++] = tang[i];
    float length = knot[i+1] - knot[i];   // t = (u-u_i)/delta_i, so d/dt = d/du * 1/delta_i
    for (j=1; j<density-1; j++)	
     {
      V3f tri[4][4];
      deCasteljau (&(ctrlPt[3*i]), j*delta, tri);
      for (int k=0; k<3; k++)	// use 2nd level of de Casteljau triangle
        sampleTang[nSample][k] = 3*(tri[2][1][k] - tri[2][0][k]) / length;
      nSample++;
     }
   }
  sampleTang[nSample-1] = tang[nPt-1];

  /************************************************************/
  
  // build intermediate circles (for triangulation)
  sampleCirc = new V3f*[nSample];
  for (i=0; i<nSample; i++)
   {
    sampleCirc[i] = new V3f[nCircPt];

    // prepare rotation matrix that rotates z-axis to ith intermediate tangent
    float rotmat[3][3] = {0};	 V3f zaxis(0,0,1);
    if (sampleTang[i] == zaxis)
     {
      rotmat[0][0] = rotmat[1][1] = rotmat[2][2] = 1;	// identity matrix
     }
    else
     {
      V3f first,second,third;
      third = sampleTang[i];  third.normalize();  // columns must be unit vectors
      first.cross (third,zaxis); 	first.normalize();
      second.cross (third, first);	second.normalize();
      for (j=0; j<3; j++)
       {
        rotmat[j][0] = first[j];
	rotmat[j][1] = second[j];
	rotmat[j][2] = third[j];
       }
     }

    for (j=0; j<nCircPt; j++)	// build circle by transforming canonical circle
     {
      sampleCirc[i][j] = circle[j];
      sampleCirc[i][j] *= tuberadius;		// scale
      multMatrix (rotmat, sampleCirc[i][j]); 	// rotate to intermediate tangent
      sampleCirc[i][j] += sample[i];		// translate center to sample
     }
   }
   
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize (555,555);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Tubing");
  gfxinit();
  glutDisplayFunc (display);
  glutReshapeFunc (reshape);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motion);
  glutVisibilityFunc (visibility);
  glutCreateMenu (menu);
  glutAddMenuEntry ("Data points", 	1);
  glutAddMenuEntry ("Tangents",    	2);
  glutAddMenuEntry ("Circles", 		3);
  glutAddMenuEntry ("Control polygon",  4);
  glutAddMenuEntry ("Curve", 		5);
  glutAddMenuEntry ("Intermediate tangents", 6);
  glutAddMenuEntry ("Intermediate circles",  7);
  glutAddMenuEntry ("Triangulation",    8);
  glutAddMenuEntry ("Wireframe/filled", 9);
  glutAttachMenu (GLUT_RIGHT_BUTTON);
  glutMainLoop();
  return 0;
}

