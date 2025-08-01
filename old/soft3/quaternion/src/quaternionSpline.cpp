/*
  File:          quaternionSpline.cpp (from surfinterpolate.cpp)
  Author:        J.K. Johnstone 
  Created:	 4 October 2004
  Last Modified: 1 March 2005
  Purpose:       Build the motion of an object from a set of positions and orientations,
                 and the rational quaternion spline interpolating the set of quaternions,
  Dependence:    Quaternion, Scene
  Input: 	 1) The object, defined by a scene file to allow the flexibility
                    of several formats
		 2) A position file, defining the keyframe positions of this object.
		 3) A quaternion file, defining the keyframe orientations of this object.
  Output: 	 The animation and the quaternion spline.
  History: 	 10/11/04: Added closed curves.
                 3/1/05: collapsed animation.cpp into this code
		         created two windows
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
#include "Scene.h"
#include "Quaternion.h"

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-r # of random pts to generate] (0 => random #)" << endl;
  cout << "\t[-a] (angle-axis input)" << endl;
  cout << "\t[-c] (closed curve)" << endl;
  cout << "\t[-m max angle between qions] (default: 20)" << endl;
  cout << "\t[-v] visualize the result, so restrict quaternions appropriately" << endl;
  cout << "\t[-l] (laptop)" << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-o xrot yrot zrot] (initial orientation)" << endl;
  cout << "\t[-h] (this help message)" << endl;

  cout << "\t <scene file, consisting of one object>" << endl;
  cout << "\t <keyframe positions>" << endl;
  cout << "\t <keyframe orientations>" << endl;
 }

static GLfloat   transxob, transyob, rotxob, rotyob, rotzob, zoomob;
static GLfloat   rotxqs, rotyqs, rotzqs, zoomqs;
static int 	 panLeft=0, panRight=1; // control panning for 3d effect
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWKEY=1;             // draw object keyframes?
static GLboolean DRAWPT=1;		// draw quaternions?
static GLboolean LABELPT=0;             // number the quaternions?
static GLboolean DRAWSPLINE=1;          // draw quaternion spline?
static GLboolean DRAWSPHERE=1;          // draw S3 (as S2)?
static GLboolean DRAWPOLE=1;            // draw pole of inverse map?
static GLboolean DRAWLIGHT=0;		// draw position of light?
static GLboolean ROTATE=0;            // rotate?
static GLboolean PAN=0;               // pan?
static GLboolean AXISANGLEINPUT=0;      // input as angle/axis pairs?
static GLboolean RANDOMINPUT=0;         // random input?
static GLboolean CLOSED=0;              // build closed curve?
static GLboolean VISUALIZE = 0;         // should quaternion spline be visualized?
static GLboolean LAPTOP=0;              // display environment for laptop?

int                      maxAngle = 20; // maximum allowable angle (in degrees) 
                                        // between consecutive q'ions
Scene                    scene;          // object to be animated
V3fArr                   pos;            // keyframe positions
BezierCurve3f            posSpline;      // definition of the position-motion
RationalQuaternionSpline qspline;
float                    tMotion;        // parameter defining present position in motion
float                    epsMotion;      // parameter increment per step of motion

int			 obstacleWin, qsWin; // window identifiers
int       		 nPtsPerSegment = PTSPERBEZSEGMENT;

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
  glPointSize (6.0);
  transxob = transyob = 0.0;

  rotxob = 0; rotyob = 0; rotzob = 0;
  string scenetype; scene.getobjtype(0, scenetype);
  if (scenetype == "BEZSURF") { rotxob = -90; } // see surfinterpolate.cpp
  rotxqs = -90; rotyqs = 0; rotzqs = 0;
  zoomob = .15;
  zoomqs = 1;
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

void RotateQS (void)
{
  rotzqs += 0.1; 
  if (rotzqs > 360.0) rotzqs -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void PanQS (void)
{
  if (panLeft)
   {
    rotzqs += 0.1;
    if (rotzqs > 360.0) rotzqs -= 360.0;
    panLeft++;
    if (panLeft==200) { panLeft=0; panRight=1; }
   }
  else if (panRight)
   {
    rotzqs -= 0.1;
    if (rotzqs < 0.0) rotzqs += 360.0;
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
  else if (ROTATE) glutIdleFunc (RotateQS);
  else if (PAN)    glutIdleFunc (PanQS);
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

void motionOb (int x, int y)
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

void motionQS (int x, int y)
{
  if (WINDOWS)
   {
    if (!leftMouseDown && middleMouseDown)  
     {
      if (firstx)  firstx=0; else zoomqs -= (float).01*(x-oldx);
      if (zoomqs < 0.0) zoomqs = 0.0;
     }
    else if (leftMouseDown && middleMouseDown)
     {
      if (firstx)  firstx=0; else transxob += .01*(x-oldx); /* TRANSLATION: X */
      if (firsty)  firsty=0; else transyob += .01*(y-oldy); /* TRANSLATION: Y */
     }
    else if (leftMouseDown && !middleMouseDown) 
     {
      if (firstx)  firstx=0;
      else { rotyqs += .5*(x-oldx); if (rotyqs > 360.0) rotyqs -= 360.0; } /* ORI: Y */

      if (firsty)  firsty=0;
      else { rotxqs += .5*(y-oldy); if (rotxqs > 360.0) rotxqs -= 360.0; } /* ORI: X */
     }
   }
  else
   {
    if (leftMouseDown && !middleMouseDown)	   
     {
      if (firstx)  firstx=0; else zoomqs -= (float).01*(x-oldx);
      if (zoomqs < 0.0) zoomqs = 0.0;
     }
    else if (leftMouseDown && middleMouseDown)
     {
      if (firstx)  firstx=0; else transxob += .01*(x-oldx); /* TRANSLATION: X */
      if (firsty)  firsty=0; else transyob += .01*(y-oldy); /* TRANSLATION: Y */
     }
    else if (middleMouseDown) 
     {
      if (firstx)  firstx=0;
      else { rotyqs += .5*(x-oldx); if (rotyqs > 360.0) rotyqs -= 360.0; } /* ORI: Y */

      if (firsty)  firsty=0;
      else { rotxqs += .5*(y-oldy); if (rotxqs > 360.0) rotxqs -= 360.0; } /* ORI: X */
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
  case '1': 	DRAWPT = !DRAWPT;		break;
  case '2':     DRAWSPLINE   = !DRAWSPLINE;     break;
  case '3':     LABELPT     = !LABELPT;       break;
  case '4':     DRAWSPHERE  = !DRAWSPHERE;    break;
  case 'a':     tMotion = posSpline.getKnot(0); break;  // replay animation
  case 'r':     ROTATE = !ROTATE;			// rotate
 	     	if (ROTATE) 
		     glutIdleFunc (RotateQS); 
		     else glutIdleFunc (NULL); 	break; 
  case 'p':	PAN = !PAN;				// pan
		if (PAN) 
		     glutIdleFunc (PanQS); 
		     else glutIdleFunc (NULL); 	break;
  case 'l':	DRAWLIGHT = !DRAWLIGHT;		break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1:  DRAWKEY 	= !DRAWKEY;		break;
  default:   					break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuQS (int value)
{
  switch (value) {
  case 1:  DRAWPT 	= !DRAWPT;		break;
  case 2:  DRAWSPLINE   = !DRAWSPLINE;          break;
  case 3:  LABELPT      = !LABELPT;             break;
  case 4:  DRAWSPHERE   = !DRAWSPHERE;          break;
  case 5:  DRAWPOLE     = !DRAWPOLE;            break;
  default:   					break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int i,j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomob, zoomob, zoomob);
  glTranslatef (transxob, transyob, 0);
  glRotatef (rotxob, 1.0, 0.0, 0.0);
  glRotatef (rotyob, 0.0, 1.0, 0.0);
  glRotatef (rotzob, 0.0, 0.0, 1.0);

  glMaterialfv(GL_FRONT, GL_AMBIENT,  material[0]);
  glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[0]+4);
  glMaterialfv(GL_FRONT, GL_SPECULAR, material[0]+8);
  glMaterialf(GL_FRONT, GL_SHININESS, material[0][12] * 128.0);
  for (i=0; i<pos.getn(); i++)     // keyframes
    {
      glPushMatrix();
      glTranslatef (pos[i][0], pos[i][1], pos[i][2]);
      float M[16]; qspline.toGLMatrix (i,M);
      glMultMatrixf (M);
      scene.draw();
      glPopMatrix();
    }
  /*
  glPushMatrix();                  // animation
  V3f position; posSpline.eval(tMotion, position);
  glTranslatef (position[0], position[1], position[2]);
  scene.draw();
  if (tMotion < posSpline.getLastKnot()) tMotion += epsMotion;  
  if (tMotion > posSpline.getLastKnot()) tMotion = posSpline.getLastKnot();
  glPopMatrix();
  */

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************/
/******************************************************************************/

void displayQS ()
{
  int i,j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomqs, zoomqs, zoomqs);
  glTranslatef (transxob, transyob, 0);
  glRotatef (rotxqs, 1.0, 0.0, 0.0);
  glRotatef (rotyqs, 0.0, 1.0, 0.0);
  glRotatef (rotzqs, 0.0, 0.0, 1.0);

  glMaterialfv(GL_FRONT, GL_AMBIENT,  material[1]);
  glMaterialfv(GL_FRONT, GL_SPECULAR, material[1]+8);
  glMaterialf(GL_FRONT, GL_SHININESS, material[1][12] * 128.0);     

  if (DRAWSPHERE)
   {
    glDisable (GL_LIGHTING);
    glLineWidth (1.0);             
    glColor3fv (Grey);
    glutWireSphere (1.0, 20, 20);
    glEnable  (GL_LIGHTING);
   }
  if (DRAWPOLE)
   {
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material[2]+4);
    glPushMatrix();
    glTranslatef(1,0,0);
    glutSolidSphere(.02, 40, 40);
    glPopMatrix();
   }
  if (DRAWPT)
   {
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material[1]+4);
    qspline.drawData();
   }
  if (LABELPT)
   {
    glDisable (GL_LIGHTING);
    glColor3fv (Black);
    qspline.labelData();
    glEnable  (GL_LIGHTING);
   }
  if (DRAWSPLINE)
   {
    glDisable (GL_LIGHTING);
    glColor3fv (Blue);
    qspline.draw();
    glEnable  (GL_LIGHTING);
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int i;
  int ArgsParsed=0;
  int nRandom;       // # of points in random data set

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'a': AXISANGLEINPUT = 1;                             break;
      case 'r': RANDOMINPUT = 1;                                
	        nRandom = atoi(argv[ArgsParsed++]);             break;
      case 'c': CLOSED=1;                                       break;
      case 'm': maxAngle = atoi(argv[ArgsParsed++]);            break;
      case 'v': VISUALIZE = 1;                                  break;
      case 'l': LAPTOP = 1;                                     break;
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'o': rotxob = atof(argv[ArgsParsed++]);
      		rotyob = atof(argv[ArgsParsed++]);
      		rotzob = atof(argv[ArgsParsed++]);		break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }

  ifstream infile;  
  infile.open(argv[argc-3]);                    // read the object, as a scene
  scene.read (infile);               // a scene allows flexibility of object type
  infile.close();
  scene.build(nPtsPerSegment);

  infile.open (argv[argc-2]);                   // read keyframe positions
  string comment;  readComment(infile, comment);
  readPtSet (infile, pos, 3);
  infile.close();

                                                // read keyframe orientations
  if (RANDOMINPUT) qspline.randomData (nRandom, maxAngle, 0);
  else
   {
    infile.open(argv[argc-1]);
    qspline.readQuat (infile, AXISANGLEINPUT ? 1 : 0, maxAngle, VISUALIZE);
    infile.close();
    assert (pos.getn() == qspline.getnq());
   }
  
  posSpline.fit (pos);                          // define entire motion for position
  posSpline.prepareDisplay();
  tMotion = posSpline.getKnot(0);
  epsMotion = (posSpline.getLastKnot() - posSpline.getKnot(0)) / 1000;

  if (CLOSED) qspline.fitClosed (maxAngle, 0);  // define entire motion for orientation
  else        qspline.fit       (maxAngle, 0);
  qspline.prepDisplay (nPtsPerSegment); 
  assert (qspline.curveOnSphere()); 


  // ADD INTERMEDIATE KEYFRAMES
  // ANIMATE OBJECT
  // ADD SUBSET DEFINITION THROUGH ANGLE TEST, ANTIPODE, AND *DERIVATIVE CONTROL*
  //  should use same knot sequence for position and orientation splines to coordinate

  /************************************************************/

  // 2 windows: one for the moving object and one for the quaternion spline

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  int xsize,ysize;      // window size
  int xleft;		// x-coord of lefthand side
  int barmargin; 	// width of side bar surrounding picture
  int titleht; 		// top titlebar height
  int ysecond;          // starting y-coordinate of right window
  int xleftsecond;      // left side of second window
  if (WINDOWS)
    {
      xleft 	  = 0;
      xsize 	  = 350; ysize = 350;
      barmargin   = 6;
      titleht 	  = 12;
      ysecond     = titleht+10;
      xleftsecond = xleft+xsize+2*barmargin-1;
    }
  else if (LAPTOP)
    {
      xleft       = 0;
      xsize       = ysize = 500;
      barmargin   = 12;
      titleht     = 0;
      ysecond     = 0;
      xleftsecond = xleft+xsize+barmargin;
    }
  else if (PRINTOUT)
    {
      xleft       = 0;
      xsize       = ysize = 350; // less reduction required ==> better image clarity
      barmargin   = 8;
      titleht     = 20;
      ysecond     = titleht+10;
      xleftsecond = xleft+xsize+2*barmargin-1;
    }
  else
    {
      xleft 	  = 0;			
      xsize       = ysize = 600;                // standard windows
      barmargin   = 8;
      titleht 	  = 20;
      ysecond     = titleht+10;
      xleftsecond = xleft+xsize+2*barmargin-1;
    }

  glutInitWindowPosition (xleft,titleht);		// left window
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Motion of the object");  
  if (!PRINTOUT)
   {
    strcat (titlebar, " (");
    strcat (titlebar, argv[argc-3]);  
    strcat (titlebar, ")");
   }
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionOb);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Keyframes", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (xleftsecond, ysecond);	// right window
  glutInitWindowSize (xsize,ysize);
  strcpy (titlebar, "Quaternion spline");
  if (!PRINTOUT)
   {
    strcat (titlebar, " (");
    strcat (titlebar, argv[argc-1]);
    strcat (titlebar, ")");
   }
  qsWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayQS);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionQS);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuQS);
  glutAddMenuEntry ("Sphere [4]",                             4);
  glutAddMenuEntry ("Pole [5]",                               5);
  glutAddMenuEntry ("Quaternions [1]",  		      1);
  glutAddMenuEntry ("Quaternion spline [2]",                  2);
  glutAddMenuEntry ("Number quaternions [3]",                 3);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
