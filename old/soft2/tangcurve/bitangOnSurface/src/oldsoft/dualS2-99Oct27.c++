/*
  File:          dualS2.c++ 
  Author:        J.K. Johnstone 
  Created:	 22 September 1999 
  Last Modified: 22 September 1999
  Purpose:       Compute visibility graph on S2, using dual curves
  		 to compute common tangents.
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
#include "CurveOnSn.h"

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

static GLfloat   transx, transy, transz, rotx, roty, rotz, zoomob, zoomdual;
static GLfloat   initrotx, initroty, initrotz;
static GLboolean rotate		    =0;	// start rotating? 
static GLboolean PAN		    =0;	// rotate object back and forth for 3d effect?
static int 	 panLeft	    =0; // control panning for 3d effect
static int 	 panRight	    =1;
static GLboolean rotateTangentOnOb  =0;	// start rotating tangent on 1st curve?
static GLboolean leftMouseDown	    =0;
static GLboolean middleMouseDown    =0;
static int	 firstx		    =1;	// first MOUSEX reading?
static int 	 firsty		    =1;
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY

static GLboolean JUSTVIEWING=0;		// just viewing: don't process further
static GLboolean NEWFILE=1;		// output processed data to new file?
static GLboolean DRAWWIRE=1;		// draw polygons as wireframes?
static GLboolean DRAWPOLYGONOB=0;	// draw input polygonal obstacles?
static GLboolean DRAWCURVEOB=1;		// draw curved obstacles?
static GLboolean DRAWCOMMONTANG=1;	// draw common tangents?
static GLboolean DRAWDUALCTRLPOLY=0;	// draw dual control polygons?
static GLboolean DRAWDUALCURVE=1;	// draw dual curves?
static GLboolean DRAWTANG=1;		// draw tangent (for spinning)?
static GLboolean DRAWHIT=1;		// draw dual intersections?

// static GLboolean DEBUG = 1;

// CurveOnS2		qspline;	// to test new class CurveOnSn
int 	    	   	n=0;		// # of obstacles
Array<Polygon3f>   	obstaclePoly;
Array<CurveOnS2> 	obstacle;
BezierCurve3f		scaledHodo;	// scaled hodograph of 1st obstacle
float 			tActive;	// interactive parameter value 					
float 			tDelta;		// increment of parameter value per step
Array<RatBezierCurve2f>	obdual;		// dual curves of obstacles
Array<RatBezierCurve2f> obdualreflex;	// reflection of duals
int			nHit;		
V2fArr			hit;		// intersections of duals
FloatArr		tHit0;		// param values of intersections on obdual[0]
FloatArr		tHit1;		// param values of intersection on obdual[1]
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

  rotx = initrotx = 90.0;
  roty = initroty = 0.0;
  rotz = initrotz = 0.0;	
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
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -6.0, 6.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void Rotate ()
{
  rotz += 2.0; 
  if (rotz > 360.0) rotz -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void Pan ()
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

void RotateTangentOnOb (void)
{
  tActive += tDelta;
  if (tActive > obstacle[0].getKnot (obstacle[0].getnKnot() - 1))
    tActive = obstacle[0].getKnot(0);
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) {
    if (rotateTangentOnOb || rotate || PAN)  glutIdleFunc (NULL);
  }
  else if (rotateTangentOnOb)		     glutIdleFunc (RotateTangentOnOb);
  else if (rotate)			     glutIdleFunc (Rotate);
  else if (PAN) 			     glutIdleFunc (Pan);
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); break;					// ESCAPE
  case 'r':	if (rotate) { rotate=0; glutIdleFunc (NULL); }  /* toggle rotation */
		else 	    { rotate=1; PAN=0; rotateTangentOnOb=0;
			      glutIdleFunc (Rotate); } break;
  case 'p':	if (PAN) { PAN=0; glutIdleFunc (NULL); }	/* toggle pan */
  		else 	 { PAN=1; rotate=0; rotateTangentOnOb=0;
			   glutIdleFunc (Pan); } break;
  case 'R':  	if (rotateTangentOnOb) { rotateTangentOnOb=0; glutIdleFunc(NULL); }
  		else 		       { rotateTangentOnOb=1; PAN=rotate=0;
					 glutIdleFunc (RotateTangentOnOb); } break;
  case 't':     DRAWTANG = !DRAWTANG;	break;
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
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transx += .01*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transy += .01*(y-oldy); /* TRANSLATION: Y */
   }
  else if (middleMouseDown) 
   {
    if (firstx)  firstx=0;
    else { roty += .5*(x-oldx); if (roty > 360.0) roty -= 360.0; } /* ORI: Y */

    if (firsty)  firsty=0;
    else { rotx += .5*(y-oldy); if (rotx > 360.0) rotx -= 360.0; } /* ORI: X */
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

void menuOb (int value)
{
  switch (value) {
  case 1:  if (rotateTangentOnOb) { rotateTangentOnOb=0; glutIdleFunc(NULL); }
  	   else 		  { rotateTangentOnOb=1; PAN=rotate=0;
				    glutIdleFunc (RotateTangentOnOb); } break;
  case 2:  DRAWTANG = !DRAWTANG; 					break;
  case 3:  DRAWCOMMONTANG = !DRAWCOMMONTANG;				break;
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
  glTranslatef (transx, transy, transz);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);
  glScalef  (zoomob, zoomob, zoomob);
  
  glLineWidth (1.0);
  glColor3fv (Grey);
  glutWireSphere (1.0, 40, 40);

//  glColor3fv (Blue);	glLineWidth (4.0);
//  qspline.draw();
  
  if (DRAWWIRE)
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  else
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    
  // display polygonal obstacles
  if (DRAWPOLYGONOB)
   {
    glColor3fv (Green);
    for (i=0; i<n; i++)  obstaclePoly[i].draw(1);
   }
   
  if (DRAWCURVEOB)
   {
    glColor3fv (Red);	obstacle[0].draw();
    glColor3fv (Blue);	obstacle[1].draw();
   }
   
  if (DRAWCOMMONTANG)
   {
    glColor3fv (Black);
    for (i=0; i<nHit; i++)
      obstacle[0].drawTangentCircle (tHit0[i], scaledHodo);
   }  
   
  // draw tangent on first curve at tActive
  if (DRAWTANG)
   {
    glColor3fv (Red);
    obstacle[0].drawPt (tActive);
    obstacle[0].drawTangentCircle (tActive, scaledHodo);
//    obstacle[0].drawTangent (tActive, scaledHodo);
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

  // display dual curves
  
  if (DRAWDUALCURVE)
   {
    glColor3fv (Red);	obdual[0].draw(); 
    glColor3fv (Blue);	obdual[1].draw();
    glColor3fv (Purple);  obdualreflex[1].draw();
   }
  if (DRAWDUALCTRLPOLY)
   {
    glColor3fv (Blue);  
    for (int i=0; i<n; i++)  obdual[i].drawCtrlPoly(); 
   }
  // draw point associated with tangent, on first dual at tActive
  if (DRAWTANG)
   {
    glColor3fv (Red);
    obdual[0].drawPt(tActive);
   }  
  if (DRAWHIT)
   {
    glColor3fv (Black);
    glBegin (GL_POINTS);
//    for (i=0; i<nHit; i++) glVertex2f (hit[i][0], hit[i][1]);
    for (i=0; i<nHit; i++)  obdual[0].drawPt (tHit0[i]);
    glEnd();
   }

   
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
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
  
//  for (int w=0; w<100; w++)
//    cout << "{" << cos(w*2*M_PI/100.) << ", " << sin(w*2*M_PI/100.) << "}," << endl;
//  exit(-1);

  /************************************************************/
/*  Array<V3f> data5pt (5);
  data5pt[0][0] = -0.308946; data5pt[0][1] = -0.533351; data5pt[0][2] = 0.787457;
  data5pt[1][0] = -0.416845; data5pt[1][1] = -0.892654; data5pt[1][2] = 0.171489;
  data5pt[2][0] = -0.463797; data5pt[2][1] = -0.816283; data5pt[2][2] = -0.344345;
  data5pt[3][0] = -0.958332; data5pt[3][1] = -0.170527; data5pt[3][2] = -0.229173;
  data5pt[4][0] = -0.546269; data5pt[4][1] = -0.800437; data5pt[4][2] = 0.246760;
  qspline.fit (data5pt);
  qspline.prepareDisplay (nPtsPerSegment*3, 3);
*  qspline.print(); */
    
  infile.open(argv[argc-1]);		// input 2 polygonal obstacles
  n=2;  obstaclePoly.allocate(n);  Array<V3f> data;  int nData;
  for (int i=0; i<n; i++)
   {
    infile >> nData;  	data.allocate (nData);
    for (int j=0; j<nData; j++)  infile >> data[j][0] >> data[j][1] >> data[j][2];
    obstaclePoly[i].create(data);
   }

  // generate curved obstacles by interpolation
  obstacle.allocate(n);
  for (i=0; i<n; i++)  
   { 
    obstacle[i].fit (obstaclePoly[i]);  
    obstacle[i].prepareDisplay (nPtsPerSegment*3);
   }
   
  if (!JUSTVIEWING)			
   {
    // compute hodograph, to allow simple generation of tangents on obstacles
    obstacle[0].createScaledHodo (scaledHodo);
    tActive = obstacle[0].getKnot(0);		// start at beginning
    tDelta = obstacle[0].getKnot (obstacle[0].getnKnot()-1) / 2000.;

/**********************Creating dual curves***********************************/
    obdual.allocate(n);
    obdualreflex.allocate(n);
    for (i=0; i<n; i++)  
     {
      obstacle[i].dualize (obdual[i]);
      obdual[i].prepareDisplay (nPtsPerSegment*3);
      obdualreflex[i].reflect (obdual[i]);	// reflect dual about origin
      obdualreflex[i].prepareDisplay (nPtsPerSegment*3);
     }
     
/**********************Intersecting dual curves***********************************/ 

    int nHitOrig, nHitReflex;	V2fArr hitOrig, hitReflex;
    FloatArr tHitOrig0, tHitOrig1, tHitReflex0, tHitReflex1;
    obdual[0].intersect (obdual[1], nHitOrig, hitOrig, tHitOrig0, tHitOrig1, .00001);
    obdual[0].intersect (obdualreflex[1], nHitReflex, hitReflex, tHitReflex0, tHitReflex1, .00001);
    nHit = nHitOrig + nHitReflex;	hit.allocate (nHit);  
    tHit0.allocate(nHit);  tHit1.allocate(nHit);
    for (i=0; i<nHitOrig;  i++)
     { 
      hit[i] = hitOrig[i];  
      tHit0[i] = tHitOrig0[i]; 
      tHit1[i] = tHitOrig1[i];
     }
    for (i=0; i<nHitReflex; i++) 
     {
      hit[i+nHitOrig] = hitReflex[i]; 
      tHit0[i+nHitOrig] = tHitReflex0[i]; 
      tHit1[i+nHitOrig] = tHitReflex1[i]; 
     }
     
cout << nHit << " intersections: " << endl;
for (i=0; i<nHit; i++)
  cout << "(" << hit[i][0] << "," << hit[i][1] << ") with parameter value on obstacle[0] of " 
       << tHit0[i] << " and obstacle[1] of " << tHit1[i] << endl;

    // intersect duals to find common tangents
    // store common tangents as obstacle index/parameter value quartets
    // check each common tangent for visibility
    //
    // cout << "Creating V-graph" << endl;
    // V-graph G = (V,E): V = {A,B} + points of common tangency 
    // (between 2 obstacles, the same obstacle, and A/B and obstacle);
    // E = common tangents + curved segments between vertices on the same obstacle
    // (not including segments between vertices that are already connected by a tangent)
    // cout << "Finished V-graph" << endl;
   }

//  if (NEWFILE) 			// output V-graph
//   {
//   }

  //  through mouse controls outside main:
  //    input source and destination through mouse; compute shortest path
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowPosition (164,20);
  glutInitWindowSize (545,545);
  char titlebar[100]; 
  strcpy (titlebar, "Curves");  
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Common tangents", 3);
  glutAddMenuEntry ("Tangent [t]", 2);
  glutAddMenuEntry ("Spin tangent on first curve [R]", 1);
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
  glutAddMenuEntry ("Dual curves", 1);
  glutAddMenuEntry ("Dual control polygons", 2);
  glutAddMenuEntry ("Dual intersections", 4);
  glutAddMenuEntry ("Spin point on first dual [r]", 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

