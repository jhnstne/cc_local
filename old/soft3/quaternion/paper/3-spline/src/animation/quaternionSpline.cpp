/*
  File:          quaternionSpline.cpp (from surfinterpolate.cpp)
  Author:        J.K. Johnstone 
  Created:	 4 October 2004
  Last Modified: 11 October 2004
  Purpose:       Build a rational quaternion spline interpolating a set of quaternions.
  Sequence:	 
  Input: 	 Nothing (build random set) or 
                 a set of quaternions (either direct or axis/angle pairs).
  Output: 	 A quaternion spline.
  History: 	 10/11/04: Added closed curves.
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
#include "Quaternion.h"

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-o xrot yrot zrot] (initial orientation)" << endl;
  cout << "\t[-a] (angle-axis input)" << endl;
  cout << "\t[-r # of random pts to generate] (0 => random #)" << endl;
  cout << "\t[-c] (closed curve)" << endl;
  cout << "\t[-m max angle between qions] (default: 20)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <quaternion file>" << endl;
 }

static GLfloat   transxob, transyob, rotxob, rotyob, rotzob, zoomob;
static int 	 panLeft=0, panRight=1; // control panning for 3d effect
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWPT=1;		// draw quaternions?
static GLboolean LABELPT=0;             // number the quaternions?
static GLboolean DRAWSPLINE=1;          // draw quaternion spline?
static GLboolean DRAWSPHERE=1;          // draw S3 (as S2)?
static GLboolean DRAWLIGHT=0;		// draw position of light?
static GLboolean ROTATEOB=0;            // rotate?
static GLboolean PANOB=0;               // pan?
static GLboolean AXISANGLEINPUT=0;      // input as angle/axis pairs?
static GLboolean RANDOMINPUT=0;         // random input?
static GLboolean CLOSED=0;              // build closed curve?

RationalQuaternionSpline qspline;
int                      maxAngle = 20; // maximum allowable angle (in degrees) 
                                        // between consecutive q'ions
int			 obstacleWin;	// window identifier 
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
  rotxob = -90.0; rotyob = 0; rotzob = 0;
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

void RotateOb (void)
{
  rotzob += 0.1; 
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
  case '1': 	DRAWPT = !DRAWPT;		break;
  case '2':     DRAWSPLINE   = !DRAWSPLINE;     break;
  case '3':     LABELPT     = !LABELPT;       break;
  case '4':     DRAWSPHERE  = !DRAWSPHERE;    break;
  case 'r':     ROTATEOB = !ROTATEOB;			// rotate
 	     	if (ROTATEOB) 
		     glutIdleFunc (RotateOb); 
		     else glutIdleFunc (NULL); 	break; 
  case 'p':	PANOB = !PANOB;				// pan
		if (PANOB) 
		     glutIdleFunc (PanOb); 
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
  case 1:  DRAWPT 	= !DRAWPT;		break;
  case 2:  DRAWSPLINE   = !DRAWSPLINE;          break;
  case 3:  LABELPT      = !LABELPT;             break;
  case 4:  DRAWSPHERE   = !DRAWSPHERE;          break;
  case 8:  DRAWLIGHT = !DRAWLIGHT;		break;	  
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
  if (DRAWLIGHT)
   {
    glColor3fv (Black); glBegin(GL_POINTS); glVertex3f (0,3,3); glEnd();
   }
  if (DRAWSPHERE)
   {
  /*
    float foo[4] = {0,0,0,.5};
    glMaterialfv(GL_FRONT, GL_AMBIENT, foo);
    float bar[4] = {0.745098, 0.745098, 0.745098, .5};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, bar);
    float foobar[4] = {.7, .6, .6, .5};
    glMaterialfv(GL_FRONT, GL_SPECULAR, foobar);
    glMaterialf(GL_FRONT, GL_SHININESS, .25 * 128.0);     
    glutSolidSphere (1.0, 40, 40);  // transparent sphere attempt
  */
    glDisable (GL_LIGHTING);
    glLineWidth (1.0);             
    glColor3fv (Grey);
    glutWireSphere (1.0, 20, 20);
    glEnable  (GL_LIGHTING);
   }
  if (DRAWPT)
   {
    glMaterialfv(GL_FRONT, GL_AMBIENT,  material[1]);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[1]+4);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material[1]+8);
    glMaterialf(GL_FRONT, GL_SHININESS, material[1][12] * 128.0);     
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
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'o': rotxob = atof(argv[ArgsParsed++]);
      		rotyob = atof(argv[ArgsParsed++]);
      		rotzob = atof(argv[ArgsParsed++]);		break;
      case 'a': AXISANGLEINPUT = 1;                             break;
      case 'r': RANDOMINPUT = 1;                                
	        nRandom = atoi(argv[ArgsParsed++]);             break;
      case 'c': CLOSED=1;                                       break;
      case 'm': maxAngle = atoi(argv[ArgsParsed++]);            break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }
  
  if (RANDOMINPUT)                                          // input
    qspline.randomData (nRandom, maxAngle, 0);
  else
   {
    ifstream infile;  infile.open(argv[argc-1]);
    qspline.readQuat (infile, AXISANGLEINPUT ? 1 : 0, maxAngle, 0);
    infile.close();
   }
  cout << "Fitting ..." << endl;
  if (CLOSED) qspline.fitClosed (maxAngle, 0); 
  else        qspline.fit       (maxAngle, 0);
  qspline.prepDisplay (nPtsPerSegment); 
  assert (qspline.curveOnSphere()); 

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  int titleht = 20; 	// top titlebar is 20 units high
  int xleft = 0;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Quaternion spline (");  
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
  glutAddMenuEntry ("Quaternions [1]",  		      1);
  glutAddMenuEntry ("Quaternion spline [2]",                  2);
  glutAddMenuEntry ("Number quaternions [3]",                 3);
  glutAddMenuEntry ("Sphere [4]",                             4);
  glutAddMenuEntry ("Control mesh [m]", 		      9);
  glutAddMenuEntry ("Shaded/wireframe [w]",		      0);
  glutAddMenuEntry ("Active point [a]",      		      3);
  glutAddMenuEntry ("Move active point left [h]",	      4);
  glutAddMenuEntry ("Move active point right [j]",	      5);
  glutAddMenuEntry ("Move active point up [u]",	      	      6);
  glutAddMenuEntry ("Move active point down [n]",	      7);
  glutAddMenuEntry ("Light position [l]",		      8);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
