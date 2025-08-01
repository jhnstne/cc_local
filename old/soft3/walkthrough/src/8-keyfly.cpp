/*
  File:          keyfly.cpp (from keylocal.cpp)
  Author:        J.K. Johnstone 
  Created:	 25 March 2005
  Last Modified: 11 May 2005
  Purpose:       More natural GUI for moving through a scene to collect keyframes.
  Input:         the UC Berkeley data format used by Carlo Sequin's group.
                 In particular, Seth Teller's Soda Hall data is in this format.
		 See the documentation at his webpage under Implementations and Data.
  History:       11/28/04: allow Princeton benchmark off format data too
                 3/4/05: change camera avatar to reflect size of window (in anticipation
		         of 4-corner collision detection)
		 4/5/05: main change is from trans(xyz)ob to cameraPos; this has simplifying
		         effects on keyframe storage and understanding of camera location
			 relative to the scene
                 5/4/05: added warning if consecutive quaternions are too far apart
		 5/11/05: abstracted reading to basic/MiscRead software
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
#include <tcl.h>    // for hashing

#include "basic/AllColor.h"
#include "basic/Miscellany.h"         // drawCamera
#include "basic/Vector.h"		// V3fArr
#include "basic/MiscVector.h"		// read, scaleToUnitCube, rotAboutX
#include "basic/MiscRead.h"     // readUnigrafix
#include "quaternion/Quaternion.h"

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image

static char     *RoutineName;
static GLfloat   transxob, transyob, transzob, rotxob, rotyob, rotzob;
static int 	 panLeft=0, panRight=1; // control panning for 3d effect
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean ROTATEOB=0;		// rotate obstacles?
static GLboolean PANOB=0; 		// rotate object back and forth for 3d effect?
static GLboolean DRAWVERT=0;		// draw data points?
static GLboolean DRAWFACE=1;		// draw faces?
static GLboolean WIRE=1;                // draw faces in wireframe mode?
static GLboolean DRAWLIGHT=0;		// draw position of light?
static GLboolean DRAWCAMERA=1;          // draw camera in front?
static GLboolean DRAWKEY=1;             // draw keyframes?
static GLboolean KEYFILE=0;             // is there a keyframe file?
static GLboolean BERKELEY=1;            // Berkeley's UniGrafix data?
static GLboolean PRINCETON=0;           // Princeton benchmark data (in off format)?
static GLfloat   NEARCLIPDIST=.1;       // distance of near clipping plane (used in gluPerspective)

// camera variables
V3f        cameraPos;      // position of camera [replaces transxob,transyob,transzob]
Quaternion cameraOri;      // orientation of camera
V3f        forwardDir;     // records the present forward direction for moving camera
V3f        leftDir;     
V3f        upDir;

// Unigrafix variables
V3fArr     color;          // available colors
V3fArr     vert;           // available vertices
IntArrArr  face;           // face[i] = vertex indices of ith face
IntArr     faceColor;      // color indices of each face

// keyframe file variables
int        nKey=0;         // # of keyframes so far
int        iKey=-1;        // index of present keyframe
V3fArr     keypos;         // keyframe positions
Array<Quaternion> keyori;  // keyframe orientations
V3fArr     keyforw;        // keyframe's forward direction
V3fArr     keyleft;        // keyframe's left direction
V3fArr     keyup;          // keyframe's up direction
float      maxAngleRad;    // max allowable angle (in radians) between consecutive q'ions

V3f        xaxis(1,0,0), yaxis(0,1,0), zaxis(0,0,1);
ofstream   outfile;                  // to store keyframes
int        obstacleWin;	             // window identifier 
int        windowWidth,windowHeight; // window size, potentially changed by reshape

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
  glPointSize (5.0);

  cameraPos.create (0.f, 0.f, 0.f); //  transxob = transyob = transzob = 0.0;
  if (BERKELEY)   // rotate by 90 about x-axis
    {
      cameraOri.create (xaxis, M_PI/2.); // start looking in horizontal plane, which is along y-axis
      forwardDir.create (0,1,0);
      upDir.create      (0,0,1);
    }
  else if (PRINCETON)  // no initial rotation
    {
      cameraOri[0] = 1; cameraOri[1] = cameraOri[2] = cameraOri[3] = 0;
      forwardDir.create (0,0,-1);
      upDir.create      (0,1,0);
    }
  leftDir.create (-1,0,0);
  /*
  if (BERKELEY)       rotxob = -90.0; 
  else if (PRINCETON) rotxob = 0.0;
  rotyob = 0; rotzob = 0;
  */
}

/******************************************************************************/
/******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  windowWidth = w; windowHeight = h;
  cout << "Width,height = (" << windowWidth << "," << windowHeight << ")" << endl;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  //  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -1000.,1000.);
  gluPerspective (40.0, (GLfloat) w/(GLfloat) h, NEARCLIPDIST, 20.0); // orig from redbook/planet.c
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void RotateOb (void)
{
  /*
  rotzob += 0.5; 
  if (rotzob > 360.0) rotzob -= 360.0;
  glutPostRedisplay();
  */
}

/******************************************************************************/
/******************************************************************************/

void PanOb (void)
{
  /*
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
  */
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
       if (firstx)  firstx=0; 
       if (firsty)  firsty=0; 
       // Change (camera) orientation using Shoemake's arcball.
       // Shoemake's idea is to hallucinate a sphere surrounding the window,
       // which we are rolling with the mouse to change orientation.
       // The present mouse point is lifted onto the sphere, as is the previous mouse
       // point, and the axis of rotation is orthogonal to the arc between these 2 pts.
       // The basic implementation is inspired by Kilgard's code in glh_glut.h
       // in the Cg release (Cg/SDK/DEMOS/OpenGL/inc/glh; update member function of 
       // glut_trackball, supported by motion member function of glut_simple_interactor)
       
       // a = new implied point on sphere, b = previous implied point on sphere
       // a and b are undefined (set rotation angle to 0) if mouse position is outside 
       //    the circle centered at the window center
       // a and b are initially the coordinates of the present and previous mouse points
       // in window coordinates (where the center of the window has coordinates (0,0) and
       // the extreme coordinates in the smaller window dimension are +-1).
       // They are then lifted onto the sphere.

       V3f offset (windowWidth/2.f, windowHeight/2.f, 0);
       float min = windowWidth < windowHeight ? windowWidth : windowHeight;  
       min /= 2.f;
       float dx = x-oldx, dy = oldy-y;  
       V3f a(x-dx, windowHeight - (y+dy), 0);
       V3f b(   x, windowHeight -     y , 0);
       a -= offset; a /= min;
       b -= offset; b /= min;

       if(a.dot(a) <= 1 && b.dot(b) <= 1) // do nothing if outside circle
	 {
	   // project to sphere
	   a[2] = sqrt(1.0 - a[0] * a[0] - a[1] * a[1]);
	   b[2] = sqrt(1.0 - b[0] * b[0] - b[1] * b[1]);
	   
	   V3f axis; axis.cross(a,b); // CHANGE TO a.cross(b) which returns V3f
	   axis.normalize();

	   float angle = acos(a.dot(b));

	   Quaternion oriChange (axis, angle);
	   cameraOri.mult (oriChange, cameraOri);  // update the camera orientation
	   oriChange.create (axis, -angle);        // update the frame in the opposite direction
	   oriChange.rotate (forwardDir);
	   oriChange.rotate (leftDir);
	   oriChange.rotate (upDir);

	   /*
	     MAY WANT TO USE -ANGLE SINCE CHANGES IN CAMERA ORIENTATION ARE 
	     NEGATIVE CHANGES IN SCENE.
	     IS THIS GLOBAL ORIENTATION USEFUL WHEN WE NEED TO FLY THE CAMERA 
	     ALONG THE PATH LATER?
	   */
	 }
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
  int i;
  Quaternion oriChange;
  switch (key) {
  case 27:	exit(1); 			break;	// ESCAPE
  case 'w':     WIRE = !WIRE;			break; // wireframe
  case 'u':     for(i=0;i<3;i++) cameraPos[i]+=.01*forwardDir[i]; // move forward
    /*
                if (BERKELEY) cameraPos[1] += .01; // forward
                else          cameraPos[2] -= .01;  
    */
		break;
  case 'n':     for(i=0;i<3;i++) cameraPos[i]-=.01*forwardDir[i]; // move backward
    /*
                if (BERKELEY) cameraPos[1] -= .01; // backward
                else          cameraPos[2] += .01;           
    */
		break; 
  case 'h':     for(i=0;i<3;i++) cameraPos[i]+=.01*leftDir[i]; // move left
                /* cameraPos[0] -= .01; // left */
                break;
  case 'k':     for(i=0;i<3;i++) cameraPos[i]-=.01*leftDir[i]; // move right
                /* cameraPos[0] += .01; // right */
                break;
  case 'q':     for(i=0;i<3;i++) cameraPos[i]+=.01*upDir[i]; // move up
    /*          if (BERKELEY) cameraPos[2] += .01;                  // up
                else          cameraPos[1] += .01;           
    */
                break;
  case 'a':     for(i=0;i<3;i++) cameraPos[i]-=.01*upDir[i]; // move down
    /*          if (BERKELEY) cameraPos[2] -= .01;                  // down
                else          cameraPos[1] -= .01;           
    */
                break;
  case 'x':     oriChange.create (xaxis, deg2rad(1));  // rotate camera about x
                cameraOri.mult  (oriChange, cameraOri);
		// DEFINE FRAME DIRECTLY FROM CAMERAORI RATHER THAN INDIRECTLY FROM ORICHANGE
		oriChange.create (xaxis, deg2rad(-1)); // rotate frame in opposite direction
		oriChange.rotate (forwardDir);
		oriChange.rotate (leftDir);
		oriChange.rotate (upDir);
                // rotxob   += 1;                  
		break;
  case 'X':     oriChange.create (xaxis, deg2rad(-1)); // rotate camera about x
                cameraOri.mult (oriChange, cameraOri);
		oriChange.create (xaxis, deg2rad(1)); // rotate frame in opposite direction
		oriChange.rotate (forwardDir);
		oriChange.rotate (leftDir);
		oriChange.rotate (upDir);
		// rotxob   -= 1;                  
		break; 
  case 'y':     oriChange.create (BERKELEY ? yaxis : zaxis, 
				  deg2rad(BERKELEY ? 1 : -1)); // rotate camera about y
                cameraOri.mult (oriChange, cameraOri);	
		oriChange.create (BERKELEY ? yaxis : zaxis, 
				  deg2rad(BERKELEY ? -1 : 1)); // rotate frame in opposite direction
		oriChange.rotate (forwardDir);
		oriChange.rotate (leftDir);
		oriChange.rotate (upDir);
		// if (BERKELEY) rotyob += 1; else rotzob -= 1; 
		break;
  case 'Y':     oriChange.create (BERKELEY ? yaxis : zaxis, 
				  deg2rad(BERKELEY ? -1 : 1)); // rotate camera about y
                cameraOri.mult (oriChange, cameraOri);
		oriChange.create (BERKELEY ? yaxis : zaxis, 
				  deg2rad(BERKELEY ? 1 : -1)); // rotate frame in opposite direction
		oriChange.rotate (forwardDir);
		oriChange.rotate (leftDir);
		oriChange.rotate (upDir);
		// if (BERKELEY) rotyob -= 1; else rotzob += 1;
		break;
  case 'z':     oriChange.create (BERKELEY ? zaxis : yaxis, 
				  deg2rad(BERKELEY ? 1 : -1)); // rotate camera about z
                cameraOri.mult (oriChange, cameraOri);
		oriChange.create (BERKELEY ? zaxis : yaxis, 
				  deg2rad(BERKELEY ? -1 : 1)); // rotate frame in opposite direction
		oriChange.rotate (forwardDir);
		oriChange.rotate (leftDir);
		oriChange.rotate (upDir);
		// if (BERKELEY) rotzob += 1; else rotyob -= 1;
		break;
  case 'Z':     oriChange.create (BERKELEY ? zaxis : yaxis, 
				  deg2rad(BERKELEY ? -1 : 1)); // rotate camera about z
                cameraOri.mult (oriChange, cameraOri);
		oriChange.create (BERKELEY ? zaxis : yaxis, 
				  deg2rad(BERKELEY ? 1 : -1)); // rotate frame in opposite direction
		oriChange.rotate (forwardDir);
		oriChange.rotate (leftDir);
		oriChange.rotate (upDir);
		// if (BERKELEY) rotzob -= 1; else rotyob += 1;
		break;
  case '\t':    if (BERKELEY)   // restore identity orientation
                 {
		   cameraOri.create (xaxis, M_PI/2.);
		   forwardDir.create (0,1,0);
		   upDir.create      (0,0,1);
		 }
                else if (PRINCETON)  // no initial rotation
		  { 
		    cameraOri[0] = 1; cameraOri[1] = cameraOri[2] = cameraOri[3] = 0;
		    forwardDir.create (0,0,-1);
		    upDir.create      (0,1,0);
		  }
                leftDir.create (-1,0,0);
                break;
  case ' ':     iKey++; // want to store in next keyframe when moving amongst keyframes to visualize
                if (iKey>0 && cameraOri.sphericalDist (keyori[iKey-1]) > maxAngleRad)
		  cout<< "Warning: orientation difference with last keyframe is too large"
		      << endl;
                cout << "Setting keyframe " << iKey << endl;
                keypos[iKey] = cameraPos;
                keyori[iKey] = cameraOri;
		keyforw[iKey]= forwardDir;
		keyleft[iKey]= leftDir;
		keyup[iKey]  = upDir;
		outfile << cameraPos[0] << " " 
			<< cameraPos[1] << " " 
			<< cameraPos[2] << "    "
			<< cameraOri[0] << " "
			<< cameraOri[1] << " "
			<< cameraOri[2] << " "
			<< cameraOri[3] << endl;

		/*
                // record present position
                // locate the keyframe just in front of camera
                keyframe[nKey][0] = 0;
		keyframe[nKey][1] = 0;
		keyframe[nKey][2] = -.1;
		rotAboutX (keyframe[nKey], -rotxob);  // rotate out to site
		rotAboutY (keyframe[nKey], -rotyob);
		rotAboutZ (keyframe[nKey], -rotzob);
		keyframe[nKey][0] = -transxob;        // translate out to site
		keyframe[nKey][1] = -transyob;
		keyframe[nKey][2] = -transzob;
		outfile << keyframe[nKey][0] << " " 
		        << keyframe[nKey][1] << " "
		        << keyframe[nKey][2] << "      ";
		*/

		// record present orientation too
		/*
		  orientation can be expressed in Euler angles:
		  world was rotated by (rotxob,rotyob,rotzob) so object
		  should be rotated by (-rotxob,-rotyob,-rotzob) in that order
		  for orientation; output in this format;
		  these Euler angles can then be translated to quaternions
		  using the simple quaternion for each coordinate rotation and then
		  multiplying these 3 quaternions using standard quaternion arithmetic

		  test by drawing an avatar in this position and orientation;
		*/
		/*
		outfile << rotxob << " "
		        << rotyob << " "
		        << rotzob << endl;
		keyori[nKey][0] = rotxob;
		keyori[nKey][1] = rotyob;
		keyori[nKey][2] = rotzob;
		*/

		if (iKey > nKey) nKey = iKey;
		if (nKey >= 1000) 
		  { cout << "Increase number of keyframes" << endl; exit(-1); }
		break;
  case '-':     if (iKey > 0)  // back up one keyframe, and move camera there
                 {
		   iKey--;
		   cameraPos = keypos[iKey]; // translate camera into position
		   cameraOri = keyori[iKey]; // rotate camera into position
		   forwardDir= keyforw[iKey];
		   leftDir   = keyleft[iKey];
		   upDir     = keyup[iKey];
		 }
                break; 
  case '=':     if (iKey < nKey) // jump forward one keyframe, and move camera there
                 {
		   iKey++; 
		   cameraPos = keypos[iKey]; 
		   cameraOri = keyori[iKey]; 
		   forwardDir= keyforw[iKey];
		   leftDir   = keyleft[iKey];
		   upDir     = keyup[iKey];
		 }
		break;
  case 'f':     if (iKey >=0) nKey = iKey; // forget about future keyframes
		break;
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
  case 8:  DRAWLIGHT    = !DRAWLIGHT;		break;	  
  default:   					break;
  }
  glutPostRedisplay();
}

/**************************************************************************
    Draw a 'camera' avatar at (a,b,c) of size eps, scaled to fill exactly the window.
    This time, draw the four corners in world coordinates,
    in anticipation of defining the four corner curves tracking the motion
    of the view frustrum.

    Idea:
    to draw these points without translation and rotation (as points in world space),
    we need to apply the transformations?
    try setting world point to above 4 points, then rotate by -rotx,-roty,-rotz,
    then translate to a,b,c, then draw these world points 
    without any extra transformation,
**************************************************************************/

void drawCameraWindowGlobal(float a, float b, float c, 
			    float rotx, float roty, float rotz, 
			    float eps, int windowWidth, int windowHeight)
{
  int i;
  V3f corner[4]; // 4 window corners (hopefully), in world coordinates
  corner[0][0] =  eps*windowWidth/windowHeight; corner[0][1] =  eps; corner[0][2] = 0;
  corner[1][0] =  eps*windowWidth/windowHeight; corner[1][1] = -eps; corner[1][2] = 0;
  corner[2][0] = -eps*windowWidth/windowHeight; corner[2][1] = -eps; corner[2][2] = 0;
  corner[3][0] = -eps*windowWidth/windowHeight; corner[3][1] =  eps; corner[3][2] = 0;
 
  // rotate
  for (i=0; i<4; i++)
    {
      rotAboutX (corner[i], -rotx);
      rotAboutY (corner[i], -roty);
      rotAboutZ (corner[i], -rotz);
    }

  // translate
  for (i=0; i<4; i++) 
    { 
      corner[i][0] += a; 
      corner[i][1] += b;  
      corner[i][2] += c;
    }
  glColor3fv (Blue);
  glBegin(GL_POINTS);
  for (i=0; i<4; i++)
    glVertex3f (corner[i][0], corner[i][1], corner[i][2]);
  glEnd();
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

  /*********************************************************************************/

  // rotations/translations

  glPushMatrix();
  V3f axis;  cameraOri.getAxis (axis);
                                       // cout << "axis = " << axis << "; angle = " 
                                       // << cameraOri.getAngleDeg() << endl;
  glRotatef (-cameraOri.getAngleDeg(), axis[0], axis[1], axis[2]); // rotate about room
                              /*
				glRotatef (rotxob, 1.0, 0.0, 0.0); // Euler angle version
				glRotatef (rotyob, 0.0, 1.0, 0.0);
				glRotatef (rotzob, 0.0, 0.0, 1.0);
			      */
  glTranslatef (-cameraPos[0], 
		-cameraPos[1], 
		-cameraPos[2]); // first translate to center of room

                            // remember that we are moving the scene rather than 
                            // moving the camera, so scene motions
                            // are reverse of camera motions
                            //  glTranslatef (transxob, transyob, transzob); 
                            /*
			      rotate, then translate: rotation is not local to room
			      translate, then rotate: translation directions are affected 
			      by rotation, so keyboard
			      directions constantly change; or more accurately, 
			      they change locally, but have constant global 
			      interpretations; 
			      'left' is a local phenomenon
			      could add color-coded coordinate frame to indicate 
			      what the movt directions are
			    */

  /*********************************************************************************/

  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
  float red[4] = {1,0,0,1};
  glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, red);  
  glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
  glMaterialf (GL_FRONT, GL_SHININESS, shininess * 128.0);

  // draw an oriented camera avatar in front as you move
  if (DRAWCAMERA)
    {
      // build model of camera pointing along negative z-axis
      glDisable(GL_LIGHTING);

      // starting with the center of the back plane of the 
      // view frustrum in the original position, 
      // this translates and rotates to find the center of the 
      // back plane in the present position (in world coordinates)
      V3f avatarPt(0,0,-NEARCLIPDIST); // reference pt travelling w. us
      // idea: we are originally looking down the negative z-axis, so put the point 
      // just in front of us in this direction;
      // now apply the transformations in the opposite direction: if you translate 
      // every object 1 unit right, the object 1 unit to the left is now 
      // in front of the camera
      rotAboutX (avatarPt, -rotxob);
      rotAboutY (avatarPt, -rotyob);
      rotAboutZ (avatarPt, -rotzob);
      avatarPt[0] -= transxob;
      avatarPt[1] -= transyob;
      avatarPt[2] -= transzob;  // in another interpretation, we are simply drawing 
                             // the origin by undoing the transformations
      // drawPt3 (avatarPt[0], avatarPt[1], avatarPt[2], .005);


      /*
	  DRAW FOUR CORNERS TOO (AND RECORD THEM WITH KEYFRAME ALTHOUGH WE SHOULD 
	  BE ABLE TO EXTRACT FROM POSITION AND ORIENTATION AND WINDOW SIZE), 
	  WINDOW AS SIMPLEST CASE OF MOVING OBJECT; PERHAPS THE TECHNIQUE OF TRACKING
          ALL VERTICES WILL WORK FOR ANY CONVEX OBJECT
      */
      /*
      glColor3fv (Red);
//    drawCamera (avatarPt[0], avatarPt[1], avatarPt[2], rotxob, rotyob, rotzob, .036);
// this uses local transformations rather than expressing in world coordinates
      drawCameraWindow (avatarPt[0], avatarPt[1], avatarPt[2], rotxob, rotyob, rotzob, .036, 
			windowWidth, windowHeight); // was .005
      */

      // given the position of the view frustrum (avatarPt) 
      // and its orientation (as Euler angles rather than quaternion),
      // compute the positions of the four window corners in world coordinates
      glColor3fv (Blue);
      drawCameraWindowGlobal (avatarPt[0], avatarPt[1], avatarPt[2], 
			      rotxob, rotyob, rotzob, .036,
			      windowWidth, windowHeight);
      glEnable(GL_LIGHTING);
    }

  if (DRAWKEY)
    {
      diffuse[0] = 1; diffuse[1] = diffuse[2] = 0;
      glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
      for (i=0; i<nKey; i++)
	{
	  // draw an oriented camera at each keyframe
	  //	  drawPt3 (keyframe[i][0], keyframe[i][1], keyframe[i][2], .005);

	  // SWITCH TO QUATERNION FOR 4TH PARAMETER
      	  /* drawCamera (keypos[i][0], keypos[i][1], keypos[i][2], 
	     keyori[i][0], keyori[i][1], keyori[i][2], .005); */

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
	 for (i=0; i<face.getn(); i++)
	  {
	   glBegin(GL_POLYGON);
	   for (j=0; j<face[i].getn(); j++)
	     glVertex3f (vert[face[i][j]][0], vert[face[i][j]][1], vert[face[i][j]][2]);
	   glEnd();
	  }
	 glEnable (GL_LIGHTING);	 
       }
     else
       {
	 glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
	 for (i=0; i<face.getn(); i++)
	   {
	     if (BERKELEY)
	       {
		 for (j=0; j<3; j++) diffuse[j] = color[faceColor[i]][j];
		 glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	       }
	     else
	       {
		 // for (j=0; j<3; j++) diffuse[j] = frand();
		 // glMaterialfv (GL_FRONT, GL_DIFFUSE, diffuse);
		 glMaterialfv (GL_FRONT, GL_DIFFUSE, Red);
	       }
	     V3f norm;
	     computeTriNormal(vert[face[i][0]], vert[face[i][1]], vert[face[i][2]], norm);
	     glBegin(GL_POLYGON);
	     glNormal3f (norm[0], norm[1], norm[2]);
	     for (j=0; j<face[i].getn(); j++)
	       glVertex3f (vert[face[i][j]][0], vert[face[i][j]][1], vert[face[i][j]][2]);
	     glEnd();
	   }
       }
   } 
  glPopMatrix();
  glutSwapBuffers ();
  //  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-o xrot yrot zrot] (initial orientation)" << endl;
  cout << "\t[-k keyframe-file with quaternions]" << endl;
  cout << "\t[-p] (Princeton off data)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.ug" << endl;
 }

/******************************************************************************
******************************************************************************/

void parse (int argc, char **argv)
{
  int ArgsParsed=0;
  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'o': rotxob = atof(argv[ArgsParsed++]);
      		rotyob = atof(argv[ArgsParsed++]);
      		rotzob = atof(argv[ArgsParsed++]);		break;
      case 'k': KEYFILE = 1;                                    break;
      case 'p': PRINCETON=1; BERKELEY=0;                        break;
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
  int i;
  string keyword, colorID, vertexID, faceID;
  string semicolon, leftparen, rightparen;
  string nextvertexid;
  V3f rgb;
  V3f vtx;
  int newPtr;

  parse (argc,argv);
  
  // read underlying scene

  ifstream infile (argv[argc-1]);
  if (BERKELEY)
    readUnigrafix (infile, color, vert, face, faceColor);
  else if (PRINCETON)
    readPrincetonOff (infile, vert, face);
  scaleToUnitCube (vert);

  // read existing keyframe file, if any

  if (KEYFILE)
    {
      ifstream keyfile(argv[argc-2]);
      string comment; readComment (keyfile, comment);
      int mark = keyfile.tellg();
      float foo;
      nKey=0;
      while (keyfile >> foo)
	{
	  nKey++;
	  keyfile >> foo >> foo >> foo >> foo >> foo;
	}
      keypos.allocate(1000+nKey);
      keyori.allocate(1000+nKey);
      keyforw.allocate(1000+nKey);
      keyleft.allocate(1000+nKey);
      keyup.allocate(1000+nKey);
      keyfile.clear();
      keyfile.seekg(mark);
      for (i=0; i<nKey; i++)
	keyfile >> keypos[i][0] >> keypos[i][1] >> keypos[i][2]
	        >> keyori[i][0]   >> keyori[i][1]   >> keyori[i][2] >> keyori[i][3];
      keyfile.close();
    }
  else 
   { 
     keypos.allocate (1000); keyori.allocate(1000); keyforw.allocate(1000);
     keyleft.allocate(1000); keyup.allocate(1000);
   }

  // initialize keyframe file

  outfile.open ("keyframe.out");
  outfile << "[ keyframes captured from " << argv[argc-1] 
	  << ", after scaling to unit cube ]" << endl;
  if (KEYFILE)
    {
      for (i=0; i<nKey; i++)
	outfile << keypos[i][0] << " " << keypos[i][1] << " " << keypos[i][2] << " "
		<< keyori[i][0] << " " << keyori[i][1] << " " << keyori[i][2] << " " 
		<< keyori[i][3] << endl;
    }
  maxAngleRad = deg2rad(20);

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  int titleht = 20; 	// top titlebar is 20 units high
  int xleft = 1200;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Walkthrough (");  
  strcat (titlebar, argv[argc-1]);  strcat (titlebar, ")");
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
  glutAddMenuEntry ("Camera avatar",                          4);
  glutAddMenuEntry ("Light position [l]",		      8);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
