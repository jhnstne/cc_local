/*
  File:          scene.cpp
  Author:        J.K. Johnstone 
  Created:	 18 April 2003 (from revosurf.cpp)
  Last Modified: 18 April 2003
  Purpose:       Build a scene of objects, including surfaces of revolution,
                 Bezier surfaces, ruled surfaces and surfaces of extrusion.
  Sequence:	 3rd in a sequence (interpolate, with GUI of surfinterpolate; revosurf)
  Input: 	 For surface of revolution, input is a 2d-curve in z=0 plane, 
		 assuming axis of revolution is y-axis.
  Output: 	 
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
#include "Vector.h"		// V3f, V3fArrArrArr
#include "MiscVector.h"		// read, scaleToUnitCube
#include "BezierSurf.h"		// fit, prepareDisplay, draw
#include "RatBezierSurf.h"
#include "Scene.h"

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-d display density of Bezier segment] (default: 10)" << endl;
  cout << "\t[-l] (laptop)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <scene file>" << endl;
 }

static GLfloat   transxob, transyob, rotxob, rotyob, rotzob, zoomob;
static int 	 panLeft=0, panRight=1; // control panning for 3d effect
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean ROTATEOB=0;		// rotate obstacles?
static GLboolean PANOB=0; 		// rotate object back and forth for 3d effect?
static GLboolean DRAWPT=0;		// draw data points?
static GLboolean DRAWGEN=0;		// draw generatrix?
static GLboolean DRAWSURF=1;		// draw surface of revolution?
static GLboolean WIRE=0;		// draw surface in wireframe?
static GLboolean DRAWACTIVEPT=0;	// draw active point?
static GLboolean DRAWAXES=0;            // draw axes?
static GLboolean DRAWGRID=1;            // draw grid?
static GLboolean DRAWLIGHT=0;		// draw position of light?
static GLboolean DRAWAXIS=0;		// draw axis of revolution?
static GLboolean CTRLPOLY=0;		// control polygon input?
static GLboolean CTRLPOLYRAT=0;		// rational Bezier control polygon input?
int LAPTOP=0;                           // display environment for laptop?

Scene                   scene;
int			obstacleWin;	// window identifier 
int       		nPtsPerSegment = PTSPERBEZSEGMENT;

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
  zoomob = .1;
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
  rotyob += 0.5; 
  if (rotyob > 360.0) rotyob -= 360.0;
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
  if (WINDOWS || LAPTOP)
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
     else if (middleMouseDown)                     // dolly
     {
      if (firstx)  firstx=0;
      //      else { rotyob += .5*(x-oldx); if (rotyob > 360.0) rotyob -= 360.0; } /* ORI: Y */
      else { rotzob += .5*(x-oldx); if (rotzob > 360.0) rotzob -= 360.0; }

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
  case 'g':     DRAWGEN = !DRAWGEN;		break;
  case '2':     DRAWSURF = !DRAWSURF;		break;
  case 'r':     ROTATEOB = !ROTATEOB;			// rotate
 	     	if (ROTATEOB) 
		     glutIdleFunc (RotateOb); 
		else glutIdleFunc (NULL); 	break;
  case 'p':	PANOB = !PANOB;				// pan
		if (PANOB) 
		     glutIdleFunc (PanOb); 
		else glutIdleFunc (NULL); 	break;
  case 'w':     WIRE = !WIRE;			break;	// wireframe
  case 'c':     DRAWAXES = !DRAWAXES;           break;  // coordinate frame
  case 'G':     DRAWGRID = !DRAWGRID;           break;
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
  case 0:  WIRE 	= !WIRE;		break;
  case 1:  DRAWPT 	= !DRAWPT;		break;
  case 2:  DRAWSURF 	= !DRAWSURF;		break;
  case 3:  DRAWACTIVEPT = !DRAWACTIVEPT;	break;
  case 8:  DRAWLIGHT = !DRAWLIGHT;		break;	  
  case 9:  DRAWGEN = !DRAWGEN;			break;
  case 10: DRAWAXIS = !DRAWAXIS;		break;
  case 11: DRAWAXES = !DRAWAXES;                break;
  case 12: DRAWGRID = !DRAWGRID;                break;
  default:   					break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void displayOb ()
{
  int j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoomob, zoomob, zoomob);
  glTranslatef (transxob, transyob, 0);
  glRotatef (rotxob, 1.0, 0.0, 0.0);
  glRotatef (rotyob, 0.0, 1.0, 0.0);
  glRotatef (rotzob, 0.0, 0.0, 1.0);
  if (DRAWAXES)
    {
      glColor3fv (Black);
      glDisable (GL_LIGHTING);
      glBegin (GL_LINES);
      glVertex3f (0,0,0); glVertex3f (1,0,0);
      glVertex3f (0,0,0); glVertex3f (0,1,0);
      glVertex3f (0,0,0); glVertex3f (0,0,1);
      glEnd();
      glRasterPos3f (1,0,0);
      glutBitmapCharacter (GLUT_BITMAP_HELVETICA_10, 'x');
      glRasterPos3f (0,1,0);
      glutBitmapCharacter (GLUT_BITMAP_HELVETICA_10, 'y');
      glRasterPos3f (0,0,1);
      glutBitmapCharacter (GLUT_BITMAP_HELVETICA_10, 'z');
      glEnable (GL_LIGHTING);
    }
  if (DRAWGRID)       // draw reference grid (floor of scene) in y=0 plane
    {
      float extreme=20, increment=1, x,z;
      glColor3fv (Grey);
      glDisable (GL_LIGHTING);
      glBegin (GL_LINES);
      for (x=-extreme; x<=extreme; x+=increment)
	{
	  glVertex3f (x,0,-extreme);
	  glVertex3f (x,0, extreme);
	}
      for (z=-extreme; z<=extreme; z+=increment)
	{
	  glVertex3f (-extreme,0,z);
	  glVertex3f ( extreme,0,z);
	}
      glEnd();
      glEnable (GL_LIGHTING);
    }
  if (DRAWLIGHT)
   {
    glColor3fv (Black); glBegin(GL_POINTS); glVertex3f (0,3,3); glEnd();
   }
  if (DRAWSURF)
   {
    if (WIRE)
     {
      glDisable (GL_LIGHTING);
      glColor3fv(material[0]+4); 
      scene.draw(1);
      glEnable (GL_LIGHTING);
     }
    else
     {
      glMaterialfv(GL_FRONT, GL_AMBIENT,  material[0]);
      glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[0]+4);
      glMaterialfv(GL_FRONT, GL_SPECULAR, material[0]+8);
      glMaterialf(GL_FRONT, GL_SHININESS, material[0][12] * 128.0);
      scene.draw();
     }
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

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'd': nPtsPerSegment = atoi(argv[ArgsParsed++]);	break;
      case 'm': CTRLPOLY=1;					break;
      case 'w': CTRLPOLYRAT=1;  				break;
      case 'l': LAPTOP=1;                                       break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
   else ArgsParsed++;
  }
  
  ifstream infile;  infile.open(argv[argc-1]);
  scene.read (infile);
  infile.close();
  scene.build(nPtsPerSegment);

  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  int titleht; 	// top titlebar is 20 units high
  if (LAPTOP) titleht=0; else titleht=20;
  int xleft = 0;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Scene (");  
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
  glutAddMenuEntry ("Surface of revolution [2]", 	      2);
  glutAddMenuEntry ("Axis of revolution [r]", 		     10);
  glutAddMenuEntry ("Shaded/wireframe [w]",		      0);
  glutAddMenuEntry ("Draw coordinate axes [c]",              11);
  glutAddMenuEntry ("Draw floor grid [G]",                   12);
  glutAddMenuEntry ("Light position [l]",		      8);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
