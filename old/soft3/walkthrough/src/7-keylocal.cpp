/*
  File:          keylocal.cpp (from keyframe.cpp)
  Author:        J.K. Johnstone 
  Created:	 17 November 2004
  Last Modified: 4 March 2005
  Purpose:       Register keyframes for a flythrough.
                 Rotate locally about present point, rather than around global origin.
  Input:         the UC Berkeley data format used by Carlo Sequin's group.
                 In particular, Seth Teller's Soda Hall data is in this format.
		 See the documentation at his webpage under Implementations and Data.
  History:       11/28/04: allow Princeton benchmark off format data too
                 3/4/05: change camera avatar to reflect size of window (in anticipation
		         of 4-corner collision detection)
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

#include "AllColor.h"
#include "Miscellany.h"         // drawCamera
#include "Vector.h"		// V3fArr
#include "MiscVector.h"		// read, scaleToUnitCube, rotAboutX

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
static GLboolean WIRE=1;                // draw faces in wireframe mode?
static GLboolean DRAWLIGHT=0;		// draw position of light?
static GLboolean DRAWCAMERA=1;          // draw camera in front?
static GLboolean DRAWKEY=1;             // draw keyframes?
static GLboolean KEYFILE=0;             // is there a keyframe file?
static GLboolean BERKELEY=1;            // Berkeley's UniGrafix data?
static GLboolean PRINCETON=0;           // Princeton benchmark data (in off format)?
static int       NEARCLIPDIST=.1;       // distance of near clipping plane (used in gluPerspective)

V3fArr color;           // available colors
V3fArr vert;            // available vertices
IntArrArr face;         // face[i] = vertex indices of ith face
IntArr faceColor;       // color indices of each face
int    nKey=0;          // # of keyframes
V3fArr keyframe;        // keyframe positions
V3fArr keyori;          // keyframe orientations
ofstream outfile;       // to store keyframes
int    obstacleWin;	// window identifier 
int windowWidth,windowHeight; // window size, potentially changed by reshape

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
  transxob = transyob = transzob = 0.0;
  if (BERKELEY)       rotxob = -90.0; 
  else if (PRINCETON) rotxob = 0.0;
  rotyob = 0; rotzob = 0;
  // rotxob = rotyob = rotzob = 0;
  zoomob = 1;
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
    //  case '1': 	go to keyframe 1; difficult since key is stored ready for glutLookAt mode break;
    //  case '2':     go to keyframe 2; break;
    //  case '3':     go to keyframe 3; break;
  case 'w':     WIRE = !WIRE;			break; // wireframe

  case 'n':     if (BERKELEY) transyob += .01;                
                else transzob -= .01;           break; // backward
  case 'u':     if (BERKELEY) transyob -= .01; 
                else          transzob += .01;  break; // forward
  case 'h':     transxob += .01;                break; // left
  case 'k':     transxob -= .01;                break; // right
  case 'q':     if (BERKELEY) transzob -= .01;                
                else transyob -= .01;           break; // up
  case 'a':     if (BERKELEY) transzob += .01;
                else transyob += .01;           break; // down
  case 'x':     rotxob   += 1;                  break; // rotate about x
  case 'X':     rotxob   -= 1;                  break; 
  case 'y':     if (BERKELEY) rotyob += 1; else rotzob -= 1; break; // rotate about y
  case 'Y':     if (BERKELEY) rotyob -= 1; else rotzob += 1; break; 
  case 'z':     if (BERKELEY) rotzob += 1; else rotyob -= 1; break; // rotate about z
  case 'Z':     if (BERKELEY) rotzob -= 1; else rotyob += 1; break; 
  case ' ':     // record present position
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
		outfile << rotxob << " "
		        << rotyob << " "
		        << rotzob << endl;
		keyori[nKey][0] = rotxob;
		keyori[nKey][1] = rotyob;
		keyori[nKey][2] = rotzob;

		nKey++;
		if (nKey >= 1000) 
		  { cout << "Increase number of keyframes" << endl; exit(-1); }
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
  float diffuse[4] = {0,0,0,1}; // opaque
  float specular[4] = {.7,.7,.7,1};
  float shininess   = .25;

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  //  glScalef     (zoomob,   zoomob,   zoomob);
  glRotatef (rotxob, 1.0, 0.0, 0.0);           // then rotate about room
  glRotatef (rotyob, 0.0, 1.0, 0.0);
  glRotatef (rotzob, 0.0, 0.0, 1.0);
  glTranslatef (transxob, transyob, transzob); // first translate to center of room

  /*
    rotate, then translate: rotation is not local to room
    translate, then rotate: translation directions are affected by rotation, so keyboard
                            directions constantly change; or more accurately, they change
			    locally, but have constant global interpretations;
			    'left' is a local phenomenon
    could add color-coded coordinate frame to indicate what the movt directions are
   */

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
	  drawCamera (keyframe[i][0], keyframe[i][1], keyframe[i][2], 
		      keyori[i][0], keyori[i][1], keyori[i][2], .005);
	  glRasterPos3f (keyframe[i][0], keyframe[i][1], keyframe[i][2]);
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
  cout << "\t[-k keyframe-file]" << endl;
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
  ifstream infile;
  string keyword, colorID, vertexID, faceID;
  string semicolon, leftparen, rightparen;
  string nextvertexid;
  V3f rgb;
  V3f vtx;
  int newPtr;

  parse (argc,argv);

  /*
  char ch;
  cin >> ch;
  while (ch != '0')
    {
      cout << (int) ch << endl;
      cin >> ch;
    }
  */

  if (BERKELEY)
  {
  //  read it, just to count and allocate
  int nColor=0, nVert=0, nFace=0;
  infile.open(argv[argc-1]);
  while (tryToGetLeftBrace(infile))
    {
      skipToMatchingRightBrace(infile);
    }
  int markColor = infile.tellg();
  infile >> keyword;
  while (keyword == "c_rgb") // color
    {
      infile >> colorID >> rgb[0] >> rgb[1] >> rgb[2] >> semicolon >> keyword;
      nColor++;
    }
  while (keyword == "v")     // vertex
    {
      infile >> vertexID >> vtx[0] >> vtx[1] >> vtx[2] >> semicolon >> keyword;
      nVert++;
    }
  int markFace = infile.tellg();
  int done = 0;
  while (keyword == "f" && !done) // face line: f id (v1 v2 ...) (w1 w2 ...) ... colour ;
    {
      infile >> faceID;
      getLeftParen(infile);
      int nvPerFace=0;
      while (!tryToGetRightParen(infile)) { infile >> nextvertexid; nvPerFace++; }
      // if (nvPerFace != 4) {cout << "Nonquadrilateral face: adapt code" << endl; exit(-1);}
      // are not parsing nested faces: would need 'while (tryToGetLeftParen(infile))'
      infile >> colorID >> semicolon;
      if (!(infile >> keyword)) done = 1;
      nFace++;
    } 
  cout << "(" << nColor << "," << nVert << "," << nFace << ")" << endl;
  color.allocate (nColor);
  vert.allocate (nVert);
  face.allocate (nFace);
  faceColor.allocate (nFace);

  // read it again, for degree of each face
  infile.clear();
  infile.seekg(markFace);
  nFace=0;
  done = 0;
  while (keyword == "f" && !done) // face line: f id (v1 v2 ...) (w1 w2 ...) ... colour ;
    {
      infile >> faceID;
      getSymbol (infile, '(');
      int nvPerFace=0;
      while (!tryToGetRightParen(infile)) { infile >> nextvertexid; nvPerFace++; }
      face[nFace].allocate(nvPerFace);
      infile >> colorID >> semicolon;
      if (!(infile >> keyword)) done = 1;
      nFace++;
    }

  // read it again, for real
  infile.clear();
  infile.seekg(markColor);
  nColor = nVert = nFace = 0;

  Tcl_HashTable colorTable;
  Tcl_HashTable vertTable;
  Tcl_InitHashTable (&colorTable, TCL_STRING_KEYS);
  Tcl_InitHashTable (&vertTable,  TCL_STRING_KEYS);
  Tcl_HashEntry *colorEntry, *vertEntry;
  infile >> keyword;
  while (keyword == "c_rgb")
    {
      infile >> colorID >> rgb[0] >> rgb[1] >> rgb[2] >> semicolon >> keyword;
      color[nColor] = rgb;
      //      printf("Adding %s to the color hash table\n", colorID.c_str());
      colorEntry = Tcl_CreateHashEntry (&colorTable, colorID.c_str(), &newPtr);
      if (newPtr == 1) // new entry
	Tcl_SetHashValue (colorEntry, nColor);
      //      cout << "Color index = " << (int) Tcl_GetHashValue (colorEntry) << endl;
      nColor++;
    }
  while (keyword == "v")
    {
      infile >> vertexID >> vtx[0] >> vtx[1] >> vtx[2] >> semicolon >> keyword;
      vert[nVert] = vtx;
      //      printf("Adding %s to the vertex hash table\n", vertexID.c_str());
      vertEntry = Tcl_CreateHashEntry (&vertTable, vertexID.c_str(), &newPtr);
      if (newPtr == 1)
	Tcl_SetHashValue (vertEntry, nVert);
      //      cout << "Vertex index = " << (int) Tcl_GetHashValue (vertEntry) << endl;
      nVert++;
    }
  done = 0;
  while (keyword == "f" && !done) // face line: f id (v1 v2 ...) (w1 w2 ...) ... colour ;
    {
      infile >> faceID;
      getSymbol (infile, '(');
      for (i=0; i<face[nFace].getn(); i++)
	{ 
	  infile >> nextvertexid;
	  //	  cout << "Finding vertex " << nextvertexid << endl;
	  vertEntry = Tcl_FindHashEntry (&vertTable, nextvertexid.c_str());
	  face[nFace][i] = (int) Tcl_GetHashValue (vertEntry);
	}
      //      cout << "face[" << nFace << "] = " << face[nFace] << endl;
      getSymbol (infile, ')');
      infile >> colorID >> semicolon;
      //      cout << "Finding color " << colorID << endl;
      colorEntry = Tcl_FindHashEntry (&colorTable, colorID.c_str());
      faceColor[nFace] = (int) Tcl_GetHashValue (colorEntry);
      //      cout << "faceColor[" << nFace << "] = " << faceColor[nFace] << endl;
      if (!(infile >> keyword)) done = 1;
      nFace++;
    }
  } 
  else if (PRINCETON)
    {
      infile.open(argv[argc-1]);
      string offkeyword;
      int nVert, nFace, nEdge;
      infile >> offkeyword >> nVert >> nFace >> nEdge;
      vert.allocate (nVert);
      face.allocate (nFace);
      for (i=0; i<nVert; i++)
	infile >> vert[i][0] >> vert[i][1] >> vert[i][2];
      for (i=0; i<nFace; i++)
	{
	  int nv; // # of vertices in face
	  infile >> nv;
	  face[i].allocate (nv);
	  for (int j=0; j<nv; j++)
	    infile >> face[i][j];
	}
    }

  scaleToUnitCube (vert);
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
      keyframe.allocate(1000+nKey);
      keyori.allocate(1000+nKey);
      keyfile.clear();
      keyfile.seekg(mark);
      for (i=0; i<nKey; i++)
	keyfile >> keyframe[i][0] >> keyframe[i][1] >> keyframe[i][2]
	        >> keyori[i][0]   >> keyori[i][1]   >> keyori[i][2];
      keyfile.close();
    }
  else { keyframe.allocate (1000); keyori.allocate(1000); }
  outfile.open ("keyframe.out");
  outfile << "[ keyframes captured from " << argv[argc-1] 
	  << ", after scaling to unit cube ]" << endl;
  if (KEYFILE)
    {
      for (i=0; i<nKey; i++)
	outfile << keyframe[i][0] << " " << keyframe[i][1] << " " << keyframe[i][2] << " "
		<< keyori[i][0] << " " << keyori[i][1] << " " << keyori[i][2] << endl;
    }
     
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
