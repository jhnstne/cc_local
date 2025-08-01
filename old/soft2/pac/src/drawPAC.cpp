/*
  File:          drawPAC.cpp
  Author:        J.K. Johnstone 
  Created:	 30 July 2002
  Last Modified: 16 August 2002
  Purpose:       Draw a piecewise algebraic curve of Sederberg.
  Input: 	 
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
#include "Miscellany.h"		// itoa
#include "Vector.h"		// V2f, V2fArrArr
#include "MiscVector.h"		// read, scaleToUnitSquare
#include "PAC.h"

#define PTSPERBEZSEGMENT 30      // # pts to draw on each Bezier segment
#define WINDOWS 0		 // running on Windows?

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-j] (just display PAC)" << endl;
  cout << "\t[-e #] (epsilon value: default .0001)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pac" << endl;
 }

static GLfloat   transxob, transyob, zoomob;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean DRAWBASETRI=1;		// draw base triangle?
static GLboolean DRAWPAC=1;		// draw PAC?
static GLboolean DRAWPOLARH=0;		// draw polar of horizontal tangents?
static GLboolean DRAWPOLARV=0;		// draw polar of vertical tangents?
static GLboolean DRAWHORIZ=0;		// draw horizontal tangents?
static GLboolean DRAWVERT=0;		// draw vertical tangents?
static GLboolean DRAWBOUNDARY=0;	// draw boundary extrema?
static GLboolean JUSTDISPLAY=0;		// just display original PAC?

PAC			pac, side;
V2fArr 			ptHoriz, ptVert;
PAC			polarH, polarV;
V2fArrArr 		hitSide(3);		
int			obstacleWin;	// window identifier 
int       		nPtsPerSegment = PTSPERBEZSEGMENT;
float 			eps = .0001;

/******************************************************************************/
/******************************************************************************/

void gfxinit(void)
{
  glClearColor (1.0, 1.0, 1.0, 1.0);

  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_FASTEST);
  glEnable (GL_POINT_SMOOTH);
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);

  transxob = transyob = 0.0;
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

void visibility (int status)
{
  if (status != GLUT_VISIBLE)
    glutIdleFunc (NULL);
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

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:	exit(1); 		break;		// ESCAPE
  case '1':	DRAWPAC = !DRAWPAC;		break;
  case '2': 	DRAWBASETRI = !DRAWBASETRI;	break;
  case '3':  	DRAWPOLARH  = !DRAWPOLARH;	break;
  case '4':  	DRAWPOLARV  = !DRAWPOLARV;	break;
  case '5':  	DRAWHORIZ   = !DRAWHORIZ;	break;
  case '6':  	DRAWVERT    = !DRAWVERT;	break;
  default:      break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1:  DRAWPAC     = !DRAWPAC;	break;
  case 2:  DRAWBASETRI = !DRAWBASETRI;	break;
  case 3:  DRAWPOLARH  = !DRAWPOLARH;	break;
  case 4:  DRAWPOLARV  = !DRAWPOLARV;	break;
  case 5:  DRAWHORIZ   = !DRAWHORIZ;	break;
  case 6:  DRAWVERT    = !DRAWVERT;	break;
  case 7:  DRAWBOUNDARY= !DRAWBOUNDARY;	break;
  default:   				break;
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
  glScalef  (zoomob, zoomob, zoomob);
  glTranslatef (transxob, transyob, 0);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  if (DRAWPAC)
   {
    glColor3fv (Black);
    pac.draw(0);
   }
  if (DRAWPOLARH)			// horizontal polar
   {    
    glColor3fv (Green);
    polarH.draw(0);
   }
  if (DRAWPOLARV)			// vertical polar
   {
    glColor3fv (Blue);
    polarV.draw(0);
   }
  if (DRAWHORIZ)			// horizontal tangents
   {
    glColor3fv (Green);			// points
    if (WINDOWS)
      for (i=0; i<ptHoriz.getn(); i++)
        drawPt (ptHoriz[i][0], ptHoriz[i][1]);
    else
     {
      glBegin(GL_POINTS);
      for (i=0; i<ptHoriz.getn(); i++)
        glVertex2f (ptHoriz[i][0], ptHoriz[i][1]);
      glEnd();
     }
    glBegin(GL_LINES);			// lines (assuming standard triangle)
    for (i=0; i<ptHoriz.getn(); i++)
     {
      glVertex2f (0,ptHoriz[i][1]);
      glVertex2f (1-ptHoriz[i][1],ptHoriz[i][1]);
     }
    glEnd();
   }
  if (DRAWVERT)				// vertical tangents
   {
    glColor3fv (Blue);			// points
    if (WINDOWS)
      for (i=0; i<ptVert.getn(); i++)
        drawPt (ptVert[i][0], ptVert[i][1]);
    else
     {
      glBegin(GL_POINTS);
      for (i=0; i<ptVert.getn(); i++)
        glVertex2f (ptVert[i][0], ptVert[i][1]);
      glEnd();
     }
    glBegin(GL_LINES);			// lines
    for (i=0; i<ptVert.getn(); i++)
     {
      glVertex2f (ptVert[i][0],0);
      glVertex2f (ptVert[i][0],-ptVert[i][0] + 1);
     }
    glEnd();
   }
  if (DRAWBOUNDARY)			// draw boundary extrema
   {
    glColor3fv (Blue);
    if (WINDOWS)
     {
      for (i=0; i<hitSide[0].getn(); i++)
        drawPt (hitSide[0][i][0], hitSide[0][i][1]);
      for (i=0; i<hitSide[2].getn(); i++)
        drawPt (hitSide[2][i][0], hitSide[2][i][1]);
     }
    else
     {
      glBegin(GL_POINTS);
      for (i=0; i<hitSide[0].getn(); i++)
        glVertex2f (hitSide[0][i][0], hitSide[0][i][1]);
      for (i=0; i<hitSide[2].getn(); i++)
        glVertex2f (hitSide[2][i][0], hitSide[2][i][1]);
      glEnd();
     }
    glBegin(GL_LINES);			// lines
    for (i=0; i<hitSide[0].getn(); i++)		// left boundary 
     {
      glVertex2f (0,hitSide[0][i][1]);
      glVertex2f (1-hitSide[0][i][1],hitSide[0][i][1]);
     }
    for (i=0; i<hitSide[2].getn(); i++)		// bottom boundary 
     {
      glVertex2f (hitSide[2][i][0],0);
      glVertex2f (hitSide[2][i][0],-hitSide[2][i][0] + 1);
     }
    glEnd();
   }
  if (DRAWBASETRI)
   {
    glColor3fv (Red);
    pac.draw(1);
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
      case 'e': eps = atof(argv[ArgsParsed++]);		break;
      case 'j': JUSTDISPLAY = 1;			break;
      case 'h': 
      default:	usage(); exit(-1);			break;
      }
   else ArgsParsed++;
  }
  
  ifstream infile;  infile.open(argv[argc-1]); 
  pac.input (infile);
  pac.prepareDisplay(eps);
cout << pac;

// PAC pacDE(pac);	pacDE.degreeElevate();
// cout << "Degree elevated PAC: " << endl << pacDE;

  if (!JUSTDISPLAY)
   {
    pac.polarHorizPtAtInf (polarH);
    polarH.prepareDisplay(eps);
cout << "Polar for horizontal tangents: " << endl << polarH;

    pac.polarVertPtAtInf (polarV);
    polarV.prepareDisplay(eps);
cout << "Polar for vertical tangents: " << endl << polarV;
  
    pac.vertical (ptVert, eps);
//  cout << "Vertical tangents:" << ptVert << endl;
    pac.horiz (ptHoriz, eps);
//  cout << "Horizontal tangents:" << ptHoriz << endl;

    for (int i=0; i<3; i++) 		// local extrema on boundaries
      pac.boundaryPtOnSide (i, hitSide[i], eps);

/*  float umin, umax, vmin, vmax;
    pac.extrema (umin, umax, vmin, vmax, eps);
    cout << "umin = " << umin << endl;
    cout << "vmin = " << vmin << endl;
    cout << "umax = " << umax << endl;
    cout << "vmax = " << vmax << endl; */
   }
  
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);

  int titleht = 20; 	// top titlebar is 20 units high
  //  int xleft    = 164;		// x-coord of lefthand side for small windows
  //  int xsize = 400, ysize = 400;	// small windows
  int xleft = 0;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  glutInitWindowPosition (xleft,titleht);
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "PAC (");  
  strcat (titlebar, argv[argc-1]);  strcat (titlebar, ")");
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("PAC",				1);
  glutAddMenuEntry ("Base triangle",  			2);
  glutAddMenuEntry ("Polar for horizontal tangents",	3);
  glutAddMenuEntry ("Polar for vertical tangents",	4);
  glutAddMenuEntry ("Horizontal tangents", 		5);
  glutAddMenuEntry ("Vertical tangents", 		6);
  glutAddMenuEntry ("Boundary extrema", 		7);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
