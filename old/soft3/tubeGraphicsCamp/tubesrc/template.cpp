/*
  File: 	 template.cpp
  Author:	 J.K. Johnstone
  Created:	 10 July 2003 (from tube1-barebones.c++)
  Last Modified: 10 July 2003
  Purpose:	 Example OpenGL program.
  History:	 
*/

#pragma warning (disable : 4305)
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
#include <assert.h>

#include "AllColor.h"

/******************************************************************************/
/******************************************************************************/

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <data file>" << endl;
 }
 
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
  default:        	 break;
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
  if (leftMouseDown)	// change orientation
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
  case 1:    break; // do nothing;         
  default:   break;
  }
  glutPostRedisplay();
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
  
  glColor3f (1,0,0);
  glEnable (GL_LIGHTING);
  glutSolidTeapot(1.0);
  glDisable (GL_LIGHTING);
  glColor3f (0,0,1);
  glutWireDodecahedron();

  glPopMatrix();
  glutSwapBuffers ();
}

/******************************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize (555,555);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Teapot");
  gfxinit();
  glutDisplayFunc (display);
  glutReshapeFunc (reshape);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motion);
  glutVisibilityFunc (visibility);
  glutCreateMenu (menu);
  glutAddMenuEntry ("Example menu item", 	1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);
  glutMainLoop();
  return 0;
}

