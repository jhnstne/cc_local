/*
  File:          flythrough.cpp (from flythroughPos.cpp)
  Author:        J.K. Johnstone 
  Created:	 21 November 2004
  Last Modified: 11 May 2005
  Purpose:       Flythrough of captured keyframes.
  Input:         1) UniGrafix format, the UC Berkeley data format used by 
                 Carlo Sequin's group.
                 In particular, Seth Teller's Soda Hall data is in this format.
		 See the documentation at his webpage under Implementations and Data.
		 2) keyframe file, captured by keylocal.cpp, 
		    using model scaled to unit cube
  Discussion:    use of gluLookAt to control the camera is all that is needed;
                 create a position path and an orientation path from keyframes,
		 then sample at same rate t to generate motion frames.
		 The quaternion orientation is mapped to a rotation matrix,
		 then recall that the z-axis maps to the 3rd column and
		 the y-axis maps to the 2nd column.
		 gluLookAt requires a direction of view and an up direction:
		 in the graphics community, up is associated with z, so the
		 3rd column of the rotation matrix should be the center vector
		 and the 2nd column the up vector.
		 Whatever the choice, orientation is a relative concept, so
		 the place that keyframe orientations are collected must have the
		 same assumptions about up and forward as this piece of software.

		 see forwardDir and cameraOri initializations in keyfly
		 (Berkeley's unigrafix uses z-axis as up)
		 (cameraOri is not initialized to identity in Berkeley because
		 originally camera is pointing along z-axis and must be rotated to
		 looking horizontally in a floor)

		 discuss the camera model in the intro to the paper 
		 'Collision-free camera control under a rational basis',
		 with exact assumptions of OpenGL and Unigrafix/off/...

		 Future goal: develop motion from scratch, not from keyframes.
  History:
*/

#define APPLE 1
#ifdef APPLE
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
#include <fstream>
#include <iostream>
using namespace std;
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>
#include <tcl.h>    // for hashing

#include "basic/AllColor.h"
#include "basic/Vector.h"
#include "basic/MiscVector.h"   // read, scaleToUnitCube
#include "shape/Reader.h"       // readUnigrafix
#include "curve/BezierCurve.h"  // fit, uniformSamplePlusData, ...
#include "quaternion/Quaternion.h"   // Quaternion, toGLMatrix, RationalQuaternionSpline, fit

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image

static char     *RoutineName;
static GLfloat   transxob, transyob, transzob, rotxob, rotyob, rotzob, zoomob;
static int 	 panLeft=0, panRight=1; // control panning for 3d effect
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean ROTATEOB=0;		// rotate obstacles?
static GLboolean PANOB=0; 		// rotate object back and forth for 3d effect?
static GLboolean DRAWVERT=0;		// draw data points?
static GLboolean DRAWFACE=1;		// draw faces?
static GLboolean WIRE=0;                // draw faces in wireframe mode?
static GLboolean DRAWLIGHT=0;		// draw position of light?
static GLboolean DRAWCAMERA=0;          // draw camera in front?
static GLboolean DRAWKEY=1;             // draw keyframes?
static GLboolean DRAWPATH=1;            // draw flythrough path?
static GLboolean KEYFRAME=0;            // draw in keyframe mode?

V3fArr                   color;       // available colors
V3fArr                   vert;        // available vertices
IntArrArr                face;        // face[i] = vertex indices of ith face
IntArr                   faceColor;   // color indices of each face

int                      nKey=0;      // # of keyframes
V3fArr                   keypos;      // keyframe positions
QuatArr                  keyori;      // keyframe orientations
ofstream                 outfile;     // to store keyframes

BezierCurve3f            path;        // flythrough path
RationalQuaternionSpline qspline;     // orientation component of motion
V3fArr                   pathSample;  // uniform samples on path
FloatArr                 tpath;       // its parameter values
BezierCurve3f hodo;     // flythrough path's hodograph
V3fArr tangSample;      // uniformly sampled tangents on path
FloatArr thodo;         // its parameter values
FloatArr                 tqspline;    // tqspline[i]=parameter on qspline assoc w tpath[i]
QuatArr                  oriSample;   // orientation samples associated with pathSample
int                      pathFrame=0; // index of path frame
int                      keyFrame=0;  // index of keyframe
// int                   readyNow=0;  // are we ready to draw the next keyframe?

int                      obstacleWin; // window identifier 

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  GLfloat ambient[] 	   = {0.0, 0.0, 0.0, 1.0};
  GLfloat diffuse[] 	   = {1.0, 1.0, 1.0, 1.0};
  GLfloat position[]       = {0.0, 3.0, 3.0, 0.0};
  GLfloat position1[]      = {3.0, 0.0, 0.0, 0.0};
  GLfloat position2[]      = {-3.0,0.0, 0.0, 0.0};
  GLfloat position3[]      = {0.0,-3.0, 0.0, 0.0};
  GLfloat lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
  GLfloat local_view[] 	   = {0.0};

  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, position);
  glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, position1);
  glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT2, GL_POSITION, position2);
  glLightfv(GL_LIGHT3, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT3, GL_POSITION, position3);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);

  glFrontFace(GL_CW);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHT2);
  glEnable(GL_LIGHT3);
  glEnable(GL_AUTO_NORMAL);
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  glClearColor (1.0, 1.0, 1.0, 1.0);

/*
  glShadeModel (GL_SMOOTH);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
*/

  glEnable (GL_POINT_SMOOTH);
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (2.0);
  transxob = transyob = transzob = 0.0;
  //  rotxob = -90.0; rotyob = 0; rotzob = 0;
  rotxob = rotyob = rotzob = 0;
  zoomob = 1;
}

/******************************************************************************/
/******************************************************************************/

void reshape(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -1000.,1000.);
  gluPerspective (40.0, (GLfloat) w/(GLfloat) h, .1, 20.0); // orig from redbook/planet.c
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void RotateOb (void)
{
  rotzob += 0.5; 
  if (rotzob > 360.0) rotzob -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void PanOb (void)
{
  if (panLeft)
   {
    rotzob += 0.1;
    if (rotzob > 360.0) rotzob -= 360.0;
    panLeft++;
    if (panLeft==200) { panLeft=0; panRight=1; }
   }
  else if (panRight)
   {
    rotzob -= 0.1;
    if (rotzob < 0.0) rotzob += 360.0;
    panRight++;
    if (panRight==200) { panRight=0; panLeft=1; }
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE)
    glutIdleFunc (NULL);
  else if (ROTATEOB) glutIdleFunc (RotateOb);
  else if (PANOB)    glutIdleFunc (PanOb);
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
  if (WINDOWS)
   {
    if (!leftMouseDown && middleMouseDown)  
     {
      if (firstx)  firstx=0; else zoomob -= (float).01*(x-oldx);
      if (zoomob < 0.0) zoomob = 0.0;
     }
    else if (leftMouseDown && middleMouseDown)
     {
      if (firstx)  firstx=0; else transxob += .01*(x-oldx); /* TRANSLATION: X */
      if (firsty)  firsty=0; else transyob += .01*(y-oldy); /* TRANSLATION: Y */
     }
    else if (leftMouseDown && !middleMouseDown) 
     {
      if (firstx)  firstx=0;
      else { rotyob += .5*(x-oldx); if (rotyob > 360.0) rotyob -= 360.0; } /* ORI: Y */

      if (firsty)  firsty=0;
      else { rotxob += .5*(y-oldy); if (rotxob > 360.0) rotxob -= 360.0; } /* ORI: X */
     }
   }
  else
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
    else if (middleMouseDown) 
     {
      if (firstx)  firstx=0;
      else { rotyob += .5*(x-oldx); if (rotyob > 360.0) rotyob -= 360.0; } /* ORI: Y */

      if (firsty)  firsty=0;
      else { rotxob += .5*(y-oldy); if (rotxob > 360.0) rotxob -= 360.0; } /* ORI: X */
     }
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
  case 27:	exit(1); 			break;	// ESCAPE
  case '1': 	DRAWVERT = !DRAWVERT;		break;
  case '2':     DRAWFACE = !DRAWFACE;		break;
  case '3':     DRAWKEY  = !DRAWKEY;            break;
  case '9':     KEYFRAME = !KEYFRAME;           break;
  case 'w':     WIRE = !WIRE;			break; // wireframe
  case 'n':     transyob += .01;                break; // backward
  case 'u':     transyob -= .01;                break; // forward
  case 'h':     transxob += .01;                break; // left
  case 'k':     transxob -= .01;                break; // right
  case 'q':     transzob -= .01;                break; // up
  case 'a':     transzob += .01;                break; // down
  case 'x':     rotxob   += 1;                  break; // rotate about x
  case 'X':     rotxob   -= 1;                  break; 
  case 'y':     rotyob   += 1;                  break; // rotate about y
  case 'Y':     rotyob   -= 1;                  break; 
  case 'z':     rotzob   += 1;                  break; // rotate about z
  case 'Z':     rotzob   -= 1;                  break; 
  case 'b':     if (KEYFRAME)
                 { if (keyFrame>0) keyFrame--; }
                else 
		  if (pathFrame > 0) pathFrame--;      // back up flythrough
                break;
  case ' ':     if (KEYFRAME)
                 { if (keyFrame < keypos.getn()-1) keyFrame++; }
                else 
		  if (pathFrame < pathSample.getn()-1)
		   { pathFrame++; cout << oriSample[pathFrame] << " / " << pathSample[pathFrame] << endl; }                           
                break;
    /*
// record present position
                // locate the keyframe just in front of camera
                keypos[nKey][0] = 0;
		keypos[nKey][1] = 0;
		keypos[nKey][2] = -.1;
		rotAboutX (keypos[nKey], -rotxob);  // rotate out to site
		rotAboutY (keypos[nKey], -rotyob);
		rotAboutZ (keypos[nKey], -rotzob);
		keypos[nKey][0] = -transxob;        // translate out to site
		keypos[nKey][1] = -transyob;
		keypos[nKey][2] = -transzob;
		outfile << keypos[nKey][0] << " " 
		        << keypos[nKey][1] << " "
		        << keypos[nKey][2] << "      ";

		// record present orientation too
		  orientation can be expressed in Euler angles:
		  world was rotated by (rotxob,rotyob,rotzob) so object
		  should be rotated by (-rotxob,-rotyob,-rotzob) in that order
		  for orientation; output in this format;
		  these Euler angles can then be translated to quaternions
		  using the simple quaternion for each coordinate rotation and then
		  multiplying these 3 quaternions using standard quaternion arithmetic

		  test by drawing an avatar in this position and orientation;

		outfile << rotxob << " "
		        << rotyob << " "
		        << rotzob << endl;
		keyori[nKey][0] = rotxob;
		keyori[nKey][1] = rotyob;
		keyori[nKey][2] = rotzob;

		nKey++;
		if (nKey >= 1000) 
		  { cout << "Increase number of keyframes" << endl; exit(-1); }
    */
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 0:  WIRE 	= !WIRE;		break;
  case 1:  DRAWVERT 	= !DRAWVERT;		break;
  case 2:  DRAWFACE 	= !DRAWFACE;		break;
  case 3:  DRAWKEY      = !DRAWKEY;             break;
  case 4:  DRAWCAMERA   = !DRAWCAMERA;          break;
  case 5:  DRAWPATH     = !DRAWPATH;            break;
  case 8:  DRAWLIGHT    = !DRAWLIGHT;		break;	  
  case 9:  KEYFRAME     = !KEYFRAME;            break;
  default:   					break;
  }
  glutPostRedisplay();
}

/**************************************************************************
  Draw a 'camera' avatar at (a,b,c) of size eps.
  It should be asymmetrical and have a clear directional component
  pointing along the negative z-axis.
  so that it can represent an orientation.

->a,b,c: position of avatar
->rotx,roty,rotz: orientation of avatar
->eps: size of avatar (default in header file is .01)
**************************************************************************/

void drawCamera (float a, float b, float c, float rotx, float roty, float rotz, float eps)
{
  glPushMatrix();
  glTranslatef (a,b,c);
  glRotatef (-rotz, 0.0, 0.0, 1.0);
  glRotatef (-roty, 0.0, 1.0, 0.0);
  glRotatef (-rotx, 1.0, 0.0, 0.0);
  glBegin(GL_LINES);
  glVertex3f (0,0,0);
  glVertex3f (0,0,-6*eps);
  glVertex3f (0,0,0);
  glVertex3f (eps,0,0);
  glVertex3f (0,0,0);
  glVertex3f (0,eps,0);
  glEnd();
  glBegin(GL_POINTS);
  glVertex3f (eps,0,0);
  glVertex3f (0,eps,0);
  glEnd();
  /*
  glBegin(GL_QUADS);
  glVertex3f ( eps, eps,-6*eps);
  glVertex3f (-eps, eps,-6*eps);
  glVertex3f (-eps,-eps,-6*eps);
  glVertex3f ( eps,-eps,-6*eps);
  glVertex3f ( 2*eps, 2*eps,0);
  glVertex3f (-2*eps, 2*eps,0);
  glVertex3f (-2*eps,-2*eps,0);
  glVertex3f ( 2*eps,-2*eps,0);
  glEnd();
  */
  //  glutSolidTeapot(eps);
  glPopMatrix();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j,k;
  float ambient[4]  = {0,0,0,1};
  float diffuse[4]  = {0,0,0,1}; // opaque
  float specular[4] = {.7,.7,.7,1};
  float shininess   = .25;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /**********************************************************************************/
  // camera control (can remove to see scene in normal mode)

  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();

  // If you perform modeling transform of -90 degree rotation about x, 
  // rotate pathSample[pathFrame] (and tangSample and up vector) 
  // by -90 degrees about x too;
  // since order is modeling transformations (like glRotatef)
  // then viewing transforms (like gluLookAt/gluPerspective).
  // even better: don't rotate when you apply gluLookAt.

  float rot[16];  // rotation matrix associated with a quaternion
  if (KEYFRAME)
   {
    keyori[keyFrame].toGLMatrix (rot);
    gluLookAt (keypos[keyFrame][0],
	       keypos[keyFrame][1],
	       keypos[keyFrame][2],
	       // -rot[2], -rot[6], -rot[10],
	       keypos[keyFrame][0] - rot[2],
	       keypos[keyFrame][1] - rot[6],
	       keypos[keyFrame][2] - rot[10],
	        rot[1],  rot[5],  rot[9]);
   }
  else
   {
    oriSample[pathFrame].toGLMatrix (rot);
    // this is the fundamental step
    gluLookAt(pathSample[pathFrame][0], 
	      pathSample[pathFrame][1], 
	      pathSample[pathFrame][2],
	        // we start out looking along negative z-axis (see OpenGL manual), 
	        // so the image of z-axis (negated) yields the lookat vector;
	        // the image of z axis under quaternion (rotation matrix) is 3rd column
	      // -rot[2], -rot[6], -rot[10], // try 3rd row instead: yes, that works
	      pathSample[pathFrame][0] - rot[2],
	      pathSample[pathFrame][1] - rot[6],
	      pathSample[pathFrame][2] - rot[10],
	        // up-axis is initially y-axis, so use 2nd col of rotation matrix (which
	        // is the image of the y-axis under the rotation)
	        // -rot[4], -rot[5], -rot[6]);  // try 2nd column
	      rot[1], rot[5], rot[9]); // 2nd row instead: yes, that works
   }

  /**********************************************************************************/

  // scene code
  glPushMatrix();

  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
  glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
  glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
  glMaterialf (GL_FRONT, GL_SHININESS, shininess * 128.0);

  if (DRAWKEY)
    {
      diffuse[0] = 1; diffuse[1] = diffuse[2] = 0;
      glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
      for (i=0; i<nKey; i++)
	{
	  // draw an oriented camera at each keyframe
	  // [this would need to be changed to use quaternions rather than Euler angles]
	  // drawCamera (keypos[i][0], keypos[i][1], keypos[i][2], 
	  //             keyori[i][0], keyori[i][1], keyori[i][2], .005);
	  glRasterPos3f (keypos[i][0], keypos[i][1], keypos[i][2]);
	  char str[10]; itoa (i, str);
	  for (j=0; j<strlen(str); j++)
	    glutBitmapCharacter (GLUT_BITMAP_HELVETICA_10, str[j]);
	}
    }

  if (DRAWLIGHT)
   {
    glColor3fv (Black); glBegin(GL_POINTS); glVertex3f (0,3,3); glEnd();
   }
  if (DRAWVERT)
    {
      glColor3fv (Black);
      glDisable (GL_LIGHTING);
      glBegin(GL_POINTS);
      for (i=0; i<vert.getn(); i++)
	glVertex3f (vert[i][0], vert[i][1], vert[i][2]);
      glEnd();
      glEnable (GL_LIGHTING);
    }
  if (DRAWFACE)
   {
     if (WIRE)
       {
	 glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
	 glDisable (GL_LIGHTING);
	 glColor3fv (Black);
	 glBegin(GL_QUADS);
	 for (i=0; i<face.getn(); i++)
	   for (j=0; j<face[i].getn(); j++)
	     glVertex3f (vert[face[i][j]][0], vert[face[i][j]][1], vert[face[i][j]][2]);
	 glEnd();
	 glEnable (GL_LIGHTING);	 
       }
     else
       {
	 glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
	 glBegin(GL_QUADS);
	 for (i=0; i<face.getn(); i++)
	   {
	     for (j=0; j<3; j++) diffuse[j] = color[faceColor[i]][j];
	     glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	     V3f norm;
	     computeTriNormal(vert[face[i][0]], vert[face[i][1]], vert[face[i][2]], norm);
	     glNormal3f (norm[0], norm[1], norm[2]);
	     for (j=0; j<face[i].getn(); j++)
	       glVertex3f (vert[face[i][j]][0], vert[face[i][j]][1], vert[face[i][j]][2]);
	   }
	 glEnd();
       }
   } 
  if (DRAWPATH)
    {
      glDisable(GL_LIGHTING);
      glColor3fv (Blue);
      path.draw();
      glEnable(GL_LIGHTING);
    }
  // draw an oriented camera avatar in front as you move
  if (DRAWCAMERA)
    {
      V3f redPt(0,0,-.1); // reference pt travelling w. us
      // we are originally looking down the negative z-axis, so put the point 
      // just in front of us in this direction
      glMaterialfv(GL_FRONT, GL_DIFFUSE, Red);  
      // now apply the transformations in the opposite direction: if you translate 
      // every object 1 unit right, the object 1 unit to the left is now 
      // in front of the camera
      rotAboutX (redPt, -rotxob);
      rotAboutY (redPt, -rotyob);
      rotAboutZ (redPt, -rotzob);
      redPt[0] -= transxob;
      redPt[1] -= transyob;
      redPt[2] -= transzob;  // in another interpretation, we are simply drawing 
                             // the origin by undoing the transformations
      // drawPt3 (redPt[0], redPt[1], redPt[2], .005);
      drawCamera (redPt[0], redPt[1], redPt[2], rotxob, rotyob, rotzob, .005);
      // build model of camera pointing along negative z-axis
    }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
  //  if (pathFrame < pathSample.getn()-1)// && readyNow)
  //    { pathFrame++; } // readyNow = 0; }
}

/******************************************************************************
******************************************************************************/

static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-o xrot yrot zrot] (initial orientation)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.ug <file>.key" << endl;
 }

/******************************************************************************
******************************************************************************/

void parse (int argc, char **argv)
{
  int ArgsParsed=0;
  RoutineName = argv[ArgsParsed++];
  if (argc < 3) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'o': rotxob = atof(argv[ArgsParsed++]);
      		rotyob = atof(argv[ArgsParsed++]);
      		rotzob = atof(argv[ArgsParsed++]);		break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }
}
 
/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int i,j;
  parse (argc,argv);

  // read underlying scene

  ifstream infile(argv[argc-2]);
  readUnigrafix (infile, color, vert, face, faceColor);
  scaleToUnitCube (vert);

  /****************************************************/

  // read motion keyframes

  ifstream keyfile(argv[argc-1]);
  string comment; readComment (keyfile, comment);
  int mark = keyfile.tellg();
  float foo;
  nKey=0;
  while (keyfile >> foo)
    {
      nKey++;
      keyfile >> foo >> foo >> foo >> foo >> foo >> foo;
    }
  keypos.allocate(nKey);
  keyori.allocate(nKey);
  keyfile.clear();
  keyfile.seekg(mark);
  for (i=0; i<nKey; i++)
    keyfile >> keypos[i][0] >> keypos[i][1] >> keypos[i][2]
	    >> keyori[i][0]   >> keyori[i][1]   >> keyori[i][2] >> keyori[i][3];
  keyfile.close();

  /****************************************************/

  // fit position component of motion
                                    cout << "Fitting position component" << endl;
  path.fit (keypos);              cout << path << endl;
  cout << "Preparing position display" << endl; 
  path.prepareDisplay();            
  cout << "Sampling path" << endl;
  path.uniformSamplePlusData (.01, pathSample, tpath);
  cout << "pathSample:" << endl;
  for (i=0; i<pathSample.getn(); i++) cout << pathSample[i] << endl;

  cout << "Sampling tangents" << endl;
  hodo.createHodograph (path);
  hodo.uniformSamplePlusData (.01, tangSample, thodo);
  assert (tangSample.getn() == pathSample.getn());
  

  /****************************************************/

  // fit orientation component of motion
                                    cout << "Fitting orientation component" << endl;

  // detect consecutive duplicate quaternions
  // first shot: remove duplicates and design resulting qspline, but remember the gaps
  // second shot: design a different quaternion spline between each pair of duplicates, 
  //   and still remember the gaps
  int nDistinct=1; // # of distinct quaternions (distinct from its predecessor, 
                   // so could be duplicated much later in the sequence)
  IntArr constantInterval(nKey-1); // ci[i] = 1 iff ith interval in the quaternion 
                                   // dataset is constant: i.e., keyori[i] == keyori[i+1]
  for (i=0; i<nKey-1; i++) 
   if (keyori[i] == keyori[i+1])
       constantInterval[i] = 1;
   else
     {
       constantInterval[i] = 0;
       nDistinct++;
     }
  QuatArr keyoriDistinct(nDistinct);  keyoriDistinct[0] = keyori[0];
  for (i=1,j=1; i<nKey; i++)
    if (keyori[i] != keyori[i-1])     keyoriDistinct[j++] = keyori[i];
  cout << "distinct quaternions:" << endl;
  for (i=0; i<nDistinct; i++) cout << i << ": " << keyoriDistinct[i] << endl;

  qspline.readQuat (keyoriDistinct, 40);
  FloatArr qknot;
  qspline.defineKnotSequence(qknot);
  qspline.fit (40, qknot, 0);
  qspline.prepDisplay (10);  // necessary for next assertion
  assert (qspline.curveOnSphere()); 

  //  cout << "constantInterval = " << constantInterval << endl;
  //  cout << "qknot = " << qknot << endl;

  // expand the qknot sequence to include constant intervals
  FloatArr qknotExpanded(keyori.getn());
  qknotExpanded[0] = qknot[0];
  for (i=1,j=1; i<keyori.getn(); i++) // i steps thru constantInterval and qknotExpanded
                                      // j steps thru qknot
   {
    if (constantInterval[i-1]) qknotExpanded[i] = qknot[j-1]; // insert the constant knot
    else                       qknotExpanded[i] = qknot[j++];
    //    cout << "qknotExpanded[" << i << "] = " << qknotExpanded[i] << endl;
   }
  //  cout << "qknotExpanded = " << qknotExpanded << endl;

  /****************************************************/

  // sample quaternions at same rate as positions
  // argument: neither curve (position and orientation) needs to be dominant when choosing knots
  // since we are only interested in samples eventually anyway.
  // That is, the position timeframe and the orientation timeframe are totally independent, and
  // will inevitably be quite different.
  // When we come to sampling frames from the motion, it is simple to sample the two curves at
  // different rates, with care taken to arrive at the keyframes together.
  cout << "Sampling orientations" << endl;
  FloatArr pknot(path.getnKnot());  // knots of position curve
  for (i=0; i<path.getnKnot(); i++) pknot[i] = path.getKnot(i);
  tqspline.allocate(tpath.getn());
  cout << "tpath.getn() = " << tpath.getn() << endl;
  for (i=0; i<tpath.getn()-1; i++)  // for each position sample, except the last
   {
     // what knot interval does this sample lie in? 
     // tpath[i] \in (pknot[interval], pknot[interval+1]) 
     // or tpath[i] = pknot[interval]
     // or tpath[i] = pknot[last knot], which only happens at the last knot, 
     // which has been removed
     int interval = path.findKnotInterval (tpath[i]);
     // choose the quaternion in the assoc keyframe interval using linear interpolation
     //     cout << "(tpath[i], pknot[interval], pknot[interval+1] = (" 
     //	  << tpath[i] << "," << pknot[interval] << "," << pknot[interval+1] << ")" <<endl;
     if (constantInterval[interval])
       tqspline[i] = qknotExpanded[interval];
     else
       tqspline[i] = qknotExpanded[interval] + 
	             (tpath[i] - pknot[interval])/(pknot[interval+1] - pknot[interval])
                     * (qknotExpanded[interval+1] - qknotExpanded[interval]);
     //     cout << i << ": next tqspline: " << tqspline[i] << endl;
   }
  tqspline[tpath.getn()-1] = qspline.getLastKnot();
  cout << "last tqspline: " << tqspline[tpath.getn()-1];

  // now evaluate the quaternion spline at these parameter samples
  oriSample.allocate(tqspline.getn());
  for (i=0; i<tqspline.getn(); i++) qspline.eval(tqspline[i], oriSample[i]);

  cout << endl << "oriSample:" << endl;
  for (i=0; i<oriSample.getn(); i++) cout << oriSample[i] << endl;

  /************************************************************/

  cout << "Got here" << endl;

  glutInit (&argc, argv);
  cout << "Got in between" << endl; 
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  cout << "Got here 2" << endl;

  int titleht = 20; 	// top titlebar is 20 units high
  int xleft = 1200;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Flythrough (");  
  strcat (titlebar, argv[argc-2]);  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  gfxinit();
  glutReshapeFunc (reshape);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Shaded/wireframe [w]",		      0);
  glutAddMenuEntry ("Data points [1]",  		      1);
  glutAddMenuEntry ("Faces",                                  2);
  glutAddMenuEntry ("Keyframes",                              3);
  glutAddMenuEntry ("Flythrough path",                        5);
  glutAddMenuEntry ("Flythrough path tangents",               6);
  glutAddMenuEntry ("Camera avatar",                          4);
  glutAddMenuEntry ("Light position [l]",		      8);
  glutAddMenuEntry ("Keyframe mode [9]",                      9);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  cout << "Got here 3" << endl;

  glutMainLoop();
  return 0;
}
