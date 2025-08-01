/*
  File:          chai.cpp
  Author:        J.K. Johnstone 
  Created:	 27 June 2001
  Last Modified: 28 February 2006
  Purpose:       Compute one Chaikin curve and its dual,
  		 illustrating the subdivision step in primal space
		 and its analog in dual space.
  Input: 	 One closed control polygons.
  History: 	 9/27/02: Cleaned up behaviour at infinity
  			  (dual points at infinity, edges that straddle infinity)
		 7/15/03: Added LAPTOP option.
		 10/14/03: Moved WINDOWS option to command-line parameter.
		           Changed windows from SGI to Linux format.
		 10/16/03: Store all levels of subdivision, not just present one,
		           to allow backup to previous level.
			   Hardwire to one curve.
		 11/12/03: Added cubic B-spline and DLG options.
		           Dualizing may not work with these?
		 2/28/06:  Updated to modern C++ library.
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

#include "basic/AllColor.h"
#include "basic/Miscellany.h"
#include "basic/Vector.h"
#include "basic/MiscVector.h"
#include "basic2/Line.h"

#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment
#define NLEVEL           1000    // maximum subdivision level

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-b] (cubic B-spline instead of Chaikin)" << endl;
  cout << "\t[-d] (DLG interpolatory scheme instead of Chaikin)" << endl;
  cout << "\t[-l] (laptop)" << endl;
  cout << "\t[-w] (Windows)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.pts" << endl;
 }

static GLfloat   transxob, transyob, zoomob, transxdual, transydual, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY

static GLboolean DRAWVERT=1;		// draw Chaikin control vertices?
static GLboolean DRAWEDGE=1;		// draw Chaikin control edges?
static GLboolean DRAWSPLIT=0;		// draw vertices after splitting?
static GLboolean DRAWBIGFIRST=0;	// emphasize mechanism of 1st vtx's nhood?
static GLboolean DRAWBIGSECOND=0;	// emphasize mechanism of 2nd vtx's nhood?
static GLboolean DONTDRAWBLUE=0;	// remove confusing blue lines to clarify red intersections
					// effectively stage=2.5 for 1st nhood display
static GLboolean LABELPT=0;		// label the points?					
static GLboolean DRAWORIG=0;            // draw original polygon?
static GLboolean WINDOWS=0;		// 0 for running on Linux, 1 for Windows
static GLboolean CHAIKIN=1;             // build Chaikin curve?
static GLboolean CUBICBSPLINE=0;        // build cubic B-spline subdivision curve?
static GLboolean DLG=0;                 // build Dyn-Levin-Gregory interpolatory curve?

V2fArrArr chai; 			// chai[i] = Chaikin control vertices for subdivision level i
V2fArrArr chaicirc;		        // control vertices after splitting
V2fArrArr chaida;			// a-dual Chaikin control vertices
V2fArrArr chaidb;			// b-dual Chaikin control vertices
IntArrArr finiteA;		        // finite a-dual vertices
IntArrArr finiteB;
int 	  stage=0;		        // display stage within subdivision level
int       level=0;                      // subdivision level

int	  obstacleWin;	                // identifier for left obstacle window
int	  dualWin;	                // identifier for top right dual window
int	  dualWin2;	                // identifier for bottom right dual window
int 	  PRINTOUT=0;	                // 0 for displaying on screen, 1 for printing out image
int       LAPTOP=0;                     // display environment for laptop?

/******************************************************************************
******************************************************************************/

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
  glLineStipple (1, 0xAAAA);

  transxob = transyob = 0;
  zoomob = 1.;
  zoomdual = .25; 
}

/******************************************************************************
******************************************************************************/

void reshape(GLsizei w, GLsizei h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0, -1000.,1000.);
//  gluOrtho2D(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0, 2.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************
******************************************************************************/

void visibility (int status)
{
  glutIdleFunc (NULL);
}

/******************************************************************************
	Map Chaikin polygons to dual Chaikin polygons.
******************************************************************************/

void dualize()
{
  chaida[level].allocate(chai[level].getn());
  chaidb[level].allocate(chai[level].getn());
  finiteA[level].allocate(chai[level].getn());
  finiteB[level].allocate(chai[level].getn());
  for (int i=0; i<chai[level].getn(); i++)	// for each edge
   {
    float a,b,c;	// coordinates of line ax+by+c=0 defining ith edge
    Line2f L(chai[level][i], chai[level][(i+1)%chai[level].getn()]);
    L.implicitEqn (a,b,c);
    if (a != 0)
     {
      chaida[level][i][0] = c/a;		// a-duality
      chaida[level][i][1] = b/a;
      finiteA[level][i] = 1;
     }
    else finiteA[level][i] = 0;
    if (b != 0)
     {
      chaidb[level][i][0] = a/b;		// b-duality
      chaidb[level][i][1] = c/b; 
      finiteB[level][i] = 1;
     }
    else finiteB[level][i] = 0;
   }
}

/******************************************************************************
	Splitting step (see p. 63 of Stollnitz, DeRose and Salesin `Wavelets')
******************************************************************************/

void split()
{
  int nPt = chai[level].getn();
  chaicirc[level].allocate (2*nPt);
  for (int i=0; i<nPt; i++)
    for (int j=0; j<2; j++)	// each coordinate
     {
      chaicirc[level][2*i][j]   = chai[level][i][j];
      chaicirc[level][2*i+1][j] = (chai[level][i][j] + chai[level][(i+1)%nPt][j]) / 2.;
     }
}

/******************************************************************************
	Averaging step (see p. 63 of Stollnitz, DeRose and Salesin `Wavelets')
******************************************************************************/

void average()
{
  int nPt = chaicirc[level].getn();
  chai[level+1].allocate (nPt);
  for (int i=0; i<nPt; i++)
    for (int j=0; j<2; j++)	// each coordinate
      if (CHAIKIN)
	chai[level+1][i][j] = (chaicirc[level][i][j] + chaicirc[level][(i+1)%nPt][j]) / 2.;
      else if (CUBICBSPLINE)
	chai[level+1][i][j] = chaicirc[level][mod(i-1,nPt)][j]/4 + 
	                      chaicirc[level][i][j]/2 + 
	                      chaicirc[level][(i+1)%nPt][j]/4;
      else if (DLG)
	{
	  if (i%2==0) chai[level+1][i][j] = chaicirc[level][i][j];
	  else chai[level+1][i][j] = (-2*chaicirc[level][mod(i-2,nPt)][j] + 
				       5*chaicirc[level][mod(i-1,nPt)][j] + 
				      10*chaicirc[level][i][j] + 
				       5*chaicirc[level][(i+1)%nPt][j] 
				      -2*chaicirc[level][(i+2)%nPt][j])/16.;
	}
  level++;
  if (level == NLEVEL) level--;
}

/******************************************************************************
******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case '1':	DRAWVERT = !DRAWVERT;		break;
  case '2': 	DRAWEDGE = !DRAWEDGE;		break;
  case '4':	DRAWBIGFIRST = !DRAWBIGFIRST;	break;
  case '5':	DRAWBIGSECOND = !DRAWBIGSECOND;	break;
  case '6':	DONTDRAWBLUE = !DONTDRAWBLUE;	break;
  case '7':	LABELPT = !LABELPT;		break;
  case '8':     DRAWORIG = !DRAWORIG;           break;
  case ' ':	if      (stage==0) 
     		 { split(); 
		   // dualsplit(); 
		   DRAWSPLIT=1; stage=1; }
		else if (stage==1) 
		 { average();  
		   DRAWSPLIT=1; stage=2; }
		else if (stage==2) 
		 { dualize();
		   DRAWSPLIT=1; stage=0; }
		 				break;	
  case 9:	split(); average(); dualize();  break; // TAB
  case 8:       level--; if (level<0) level=0;  stage=0; break; // backspace
  case 27:	exit(1); 			break; // ESCAPE
  default:      				break;
  }
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

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

/******************************************************************************
******************************************************************************/

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

/******************************************************************************
******************************************************************************/

void motiondual (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdual -= (float).01*(x-oldx);
    if (zoomdual < 0.0) zoomdual = 0.0;
   }
  else if (leftMouseDown && middleMouseDown)
   {
    if (firstx)  firstx=0; else transxdual += .01*(x-oldx); /* TRANSLATION: X */
    if (firsty)  firsty=0; else transydual += .01*(y-oldy); /* TRANSLATION: Y */
   }
  oldx = x;  
  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

/*
void motiondual2 (int x, int y)
{
  if (leftMouseDown && !middleMouseDown)	   
   {
    if (firstx)  firstx=0; else zoomdual2 -= (float).001*(x-oldx);
    if (zoomdual2 < 0.0) zoomdual2 = 0.0;
   }
  oldx = x;  
  glutPostRedisplay();
}
*/

/******************************************************************************
******************************************************************************/

void menuOb (int value)
{
  switch (value) {
  case 1: 	DRAWVERT = !DRAWVERT;			break;
  case 2:	DRAWEDGE = !DRAWEDGE;			break;
  case 3:       level--;                                break;
  case 4:	DRAWBIGFIRST = !DRAWBIGFIRST;		break;
  case 5:	DRAWBIGSECOND = !DRAWBIGSECOND;		break;
  case 6: 	DONTDRAWBLUE = !DONTDRAWBLUE;		break;
  case 7:	LABELPT = !LABELPT;			break;
  case 8:       DRAWORIG = !DRAWORIG;                   break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

void menuDual2 (int value)
{
  switch (value) {
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

void displayOb ()
{
  int j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

  if (DRAWORIG)
    {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (j=0; j<chai[0].getn(); j++)
	glVertex2f (chai[0][j][0], chai[0][j][1]);
      glEnd();
    }
  
  if (DRAWBIGFIRST)		// emphasize neighbourhood of 1st vertex
   {
    glPointSize(12.0);
    glLineWidth(3.0);
    glColor3fv (Red);
    if (WINDOWS)
     {
       for (j=(stage<2?0:1); j<2; j++)
	 drawPt (chai[level][j][0], chai[level][j][1]);
     }
    else
     {
      glBegin (GL_POINTS);
      for (j=(stage<2?0:1); j<2; j++)
	glVertex2f (chai[level][j][0], chai[level][j][1]);
      glEnd();
     }
    if (stage<2)
     {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (j=0; j<2; j++)
	glVertex2f (chai[level][j][0], chai[level][j][1]);
      glEnd();
     }
    else if (stage==3)
     {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (j=1; j<3; j++)
	glVertex2f (chai[level][j][0], chai[level][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      if (WINDOWS)
       {
	 for (j=(stage<2?0:1); j<3; j++)
	   drawPt (chaicirc[level][j][0], chaicirc[level][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
	for (j=(stage<2?0:1); j<3; j++)
	  glVertex2f (chaicirc[level][j][0], chaicirc[level][j][1]);
        glEnd();
       }
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWBIGSECOND)		// emphasize neighbourhood of 2nd vertex
   {
    glPointSize(12.0);
    glLineWidth(3.0);
    glColor3fv (Red);
    if (WINDOWS)
     {
       for (j=(stage<2?1:2); j<3; j++)
	 drawPt (chai[level][j][0], chai[level][j][1]);
     }
    else
     {
      glBegin (GL_POINTS);
      for (j=(stage<2?1:2); j<3; j++)
	glVertex2f (chai[level][j][0], chai[level][j][1]);
      glEnd();
     }
    if (stage<2)
     {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (j=(stage<2?1:2); j<(stage<2?3:4); j++)
	glVertex2f (chai[level][j][0], chai[level][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      if (WINDOWS)
       {
	 for (j=2; j<(stage<2?5:4); j++)
	   drawPt (chaicirc[level][j][0], chaicirc[level][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
	for (j=2; j<(stage<2?5:4); j++)
	  glVertex2f (chaicirc[level][j][0], chaicirc[level][j][1]);
        glEnd();
       }
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWVERT)			// display control vertices
   {
    glColor3fv (Red);
    if (WINDOWS)
     {
       for (j=0; j<chai[level].getn(); j++)
	 drawPt (chai[level][j][0], chai[level][j][1]);
     }
    else
     {
      glBegin (GL_POINTS);
      for (j=0; j<chai[level].getn(); j++)
	glVertex2f (chai[level][j][0], chai[level][j][1]);
      glEnd();
     }
   }
  if (LABELPT)			// label the points
   {
    glColor3fv (Red);
    for (j=0; j<chai[level].getn(); j++)
      {
        glRasterPos2f (chai[level][j][0]+.03, chai[level][j][1]+.03);
	char str[10];  itoa (j, str);
	for (k=0; k<strlen(str); k++)
	  glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
       }
   }
  if (DRAWEDGE)	// display new control edges
   {
    glColor3fv (Black);
    glBegin (GL_LINE_LOOP);
    for (j=0; j<chai[level].getn(); j++)
      glVertex2f (chai[level][j][0], chai[level][j][1]);
    glEnd();
   }
  if (DRAWSPLIT)		// display vertices after splitting stage
   {
    glColor3fv (Blue);
    if (WINDOWS)
     {
       for (j=0; j<chaicirc[level].getn(); j++)
	 drawPt (chaicirc[level][j][0], chaicirc[level][j][1]);
     }
    else
     {
      glBegin (GL_POINTS);
      for (j=0; j<chaicirc[level].getn(); j++)
	glVertex2f (chaicirc[level][j][0], chaicirc[level][j][1]);
      glEnd();
     }
   }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
	Does the dihedral angle at the primal vertex chai[i] 
	straddle the x-axis (straddle a horizontal tangent)?
	Assumes the input polygon is oriented ccw?
	Straddles iff the incoming vector and outgoing vectors are on
	different sides of the x-axis.
******************************************************************************/

int straddleHoriz (int i)
{
  int n=chai[level].getn();
  float yIncoming = chai[level][i][1] - chai[level][mod(i-1,n)][1];
  float yOutgoing = chai[level][(i+1)%n][1] - chai[level][i][1];
  return (yIncoming * yOutgoing <= 0);
}

/******************************************************************************
	Does the dihedral angle at the primal vertex chai[i] 
	straddle the y-axis (straddle a vertical tangent)?
	(Assumes the input polygon is oriented ccw? No.)
	Straddles iff the incoming vector and outgoing vectors are on
	different sides of the y-axis.
******************************************************************************/

int straddleVert (int i)
{
  int n=chai[level].getn();
  float xIncoming = chai[level][i][0] - chai[level][mod(i-1,n)][0];
  // float xIncoming = chai[level][mod(i-1,n)][0] - chai[level][i][0];
  float xOutgoing = chai[level][(i+1)%n][0] - chai[level][i][0];
  return (xIncoming * xOutgoing <= 0);
}

/******************************************************************************
******************************************************************************/

void displayDual ()
{
  int j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxdual, transydual, 0);
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  
  if (DRAWBIGFIRST)
   {
    glPointSize(12.0);
    glLineWidth(3.0);
    glColor3fv (Red);	// test
    for (j=(stage<2?0:1); j<2; j++)
      drawImplicitLine (1, chai[level][j][1], chai[level][j][0]); // (a,b,1)-> x+by+a=0
    if (stage<2)
     {
      glColor3fv (Black);
      if (WINDOWS)
       {
	 for (j=0; j<1; j++)
	   drawPt (chaida[level][j][0], chaida[level][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
	for (j=0; j<1; j++)
	  glVertex2f (chaida[level][j][0], chaida[level][j][1]);
        glEnd();
       }
     }
    else if (stage==3)
     {
      glColor3fv (Black);
      if (WINDOWS)
       {
	 for (j=1; j<2; j++)
	   drawPt (chaida[level][j][0], chaida[level][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
	for (j=1; j<2; j++)
	  glVertex2f (chaida[level][j][0], chaida[level][j][1]);
        glEnd();
       }
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      for (j=(stage<2?0:1); j<3; j++)
	drawImplicitLine (1, chaicirc[level][j][1], chaicirc[level][j][0]);
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWBIGSECOND)
   {
    glPointSize(12.0);
    glLineWidth(3.0);
    glColor3fv (Red);	// test
    for (j=(stage<2?1:2); j<3; j++)
      drawImplicitLine (1, chai[level][j][1], chai[level][j][0]); // (a,b,1)-> x+by+a=0
    if (stage<2)
     {
      glColor3fv (Black);
      if (WINDOWS)
       {
	 for (j=(stage<2?1:2); j<(stage<2?2:3); j++)
	   drawPt (chaida[level][j][0], chaida[level][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
	for (j=(stage<2?1:2); j<(stage<2?2:3); j++)
	  glVertex2f (chaida[level][j][0], chaida[level][j][1]);
        glEnd();
       }
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      for (j=2; j<(stage<2?5:4); j++)
	drawImplicitLine (1, chaicirc[level][j][1], chaicirc[level][j][0]);
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWEDGE)			// display control vertices
   {
    glColor3fv (Black);
    if (WINDOWS)
     {
       for (j=0; j<chaida[level].getn(); j++)
	 if (finiteA[level][j])
	   drawPt (chaida[level][j][0], chaida[level][j][1]);
     }
    else
     {
      glBegin (GL_POINTS);
      for (j=0; j<chaida[level].getn(); j++)
	if (finiteA[level][j])
	  glVertex2f (chaida[level][j][0], chaida[level][j][1]);
      glEnd();
     }
   }
  if (LABELPT)			// label the points
   {
    glColor3fv (Black);
    for (j=0; j<chaida[level].getn(); j++)
      if (finiteA[level][j])
	{
	  glRasterPos2f (chaida[level][j][0]+.03, chaida[level][j][1]+.03);
	  char str[10];  itoa (j, str);
	  for (k=0; k<strlen(str); k++)
	    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
	}
   }
  if (DRAWVERT)			// display control edges
   {
    glColor3fv (Red);
    int na = chaida[level].getn();
    for (j=0; j<chaida[level].getn(); j++)
      if (finiteA[level][j] && finiteA[level][(j+1)%na] && !straddleHoriz((j+1)%na))
	{	// primal vertex j+1 is assoc. w. this dual edge (j,j+1)
	  int jp1 = (j+1)%na;
          glBegin(GL_LINES);
          glVertex2f (chaida[level][j][0], chaida[level][j][1]);
  	  glVertex2f (chaida[level][jp1][0], chaida[level][jp1][1]);
	  glEnd();
         }
    glEnd();
   }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

void displayDual2 ()
{
  int j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxdual, transydual, 0);
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
   
  if (DRAWBIGFIRST)
   {
    glPointSize(12.0);
    glLineWidth (3.0);
    glColor3fv (Red);
    for (j=(stage<2?0:1); j<2; j++)
      drawImplicitLine (chai[level][j][0], 1, chai[level][j][1]); // (a,b,1)-> ax+y+b=0
    if (stage<2)
     {
      glColor3fv (Black);
      if (WINDOWS)
       {
	 for (j=0; j<1; j++)
	   drawPt (chaidb[level][j][0], chaidb[level][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
	for (j=0; j<1; j++)
	  glVertex2f (chaidb[level][j][0], chaidb[level][j][1]);
        glEnd();
       }
     }
    else if (stage==3)
     {
      glColor3fv (Black);
      if (WINDOWS)
       {
	 for (j=1; j<2; j++)
	   drawPt (chaidb[level][j][0], chaidb[level][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
	for (j=1; j<2; j++)
	  glVertex2f (chaidb[level][j][0], chaidb[level][j][1]);
        glEnd();
       }
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      for (j=(stage<2?0:1); j<3; j++)
	drawImplicitLine (chai[level][j][0], 1, chai[level][j][1]);
     }
    glPointSize(6.0);
    glLineWidth (1.0);
   }
  if (DRAWBIGSECOND)
   {
    glPointSize(12.0);
    glLineWidth (3.0);
    glColor3fv (Red);
    for (j=(stage<2?1:2); j<3; j++)
      drawImplicitLine (chai[level][j][0], 1, chai[level][j][1]); // (a,b,1)-> ax+y+b=0
    if (stage<2)
     {
      glColor3fv (Black);
      if (WINDOWS)
       {
	 for (j=(stage<2?1:2); j<(stage<2?2:3); j++)
	   drawPt (chaidb[level][j][0], chaidb[level][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
	for (j=(stage<2?1:2); j<(stage<2?2:3); j++)
	  glVertex2f (chaidb[level][j][0], chaidb[level][j][1]);
        glEnd();
       }
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      for (j=2; j<(stage<2?5:4); j++)
	drawImplicitLine (chai[level][j][0], 1, chai[level][j][1]);
     }
    glPointSize(6.0);
    glLineWidth (1.0);
   }
  if (DRAWEDGE)			// display control vertices
   {
    glColor3fv (Black);
    if (WINDOWS)
     {
       for (j=0; j<chaidb[level].getn(); j++)
	 if (finiteB[level][j])
	   drawPt (chaidb[level][j][0], chaidb[level][j][1]);
     }
    else
     {
      glBegin (GL_POINTS);
      for (j=0; j<chaidb[level].getn(); j++)
	if (finiteB[level][j])
	  glVertex2f (chaidb[level][j][0], chaidb[level][j][1]);
      glEnd();
     }
   }
  if (LABELPT)			// label the points
   {
    glColor3fv (Black);
    for (j=0; j<chaidb[level].getn(); j++)
      if (finiteB[level][j])
	{
	  glRasterPos2f (chaidb[level][j][0]+.03, chaidb[level][j][1]+.03);
	  char str[10];  itoa (j, str);
	  for (k=0; k<strlen(str); k++)
	    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[k]);
	}
   }
  if (DRAWVERT)			// display control edges
   {
    glColor3fv (Red);
    int nb = chaidb[level].getn();
    for (j=0; j<chaidb[level].getn(); j++)
      if (finiteB[level][j] && finiteB[level][(j+1)%nb] && !straddleVert((j+1)%nb))
	{
	  int jp1 = (j+1)%nb;
	  glBegin(GL_LINES);
	  glVertex2f (chaidb[level][j][0],   chaidb[level][j][1]);
	  glVertex2f (chaidb[level][jp1][0], chaidb[level][jp1][1]);
	  glEnd();
	 }
   }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
	Input control polygons into Pt.
******************************************************************************/

void input(char *filename, string &filePrefix, V2fArrArr &chai)
{
  int i,j;
  string fileName(filename);		// find file prefix (for output file)
  string::size_type pos = fileName.find(".");
  fileName.erase (pos);
  filePrefix = fileName;

  ifstream infile;  infile.open(filename);
  string foo;  readComment (infile, foo);
  chai.allocate(NLEVEL);
  V2fArrArr Pt(1);
  getLeftBrace(infile);
  int mark = infile.tellg(); V2f bar; int nPt=0;
  while (!tryToGetRightBrace(infile))
    {
      infile >> bar[0] >> bar[1];  nPt++;
    }
  assert (nPt>0);  Pt[0].allocate(nPt);
  infile.seekg(mark);
  infile >> Pt[0][0][0] >> Pt[0][0][1];
  for (j=1; j<nPt; j++)
    {
      infile >> Pt[0][j][0] >> Pt[0][j][1];
      if (Pt[0][j]==Pt[0][j-1]) { j--; nPt--; }		// skip duplicate
      if (j==nPt-1 && Pt[0][j]==Pt[0][0]) nPt--;	// skip duplicate at end, too
     }
  getRightBrace(infile);
  scaleToUnitSquare (Pt);
  chai[0].allocate(nPt);
  for (j=0; j<nPt; j++)	chai[0][j] = Pt[0][j];
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
//float     eps = .0001;	// accuracy of intersection computation

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
//    case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'b': CUBICBSPLINE = 1; CHAIKIN=0;                    break;
      case 'd': DLG = 1; CHAIKIN = 0;                           break;
      case 'l': LAPTOP = 1;                                     break;
      case 'w': WINDOWS = 1;                                    break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
    else ArgsParsed++;
   }
  
  string filePrefix;
  input(argv[argc-1], filePrefix, chai);
  chaicirc.allocate(NLEVEL);
  chaida.allocate(NLEVEL);  chaidb.allocate(NLEVEL);
  finiteA.allocate(NLEVEL); finiteB.allocate(NLEVEL);
  
  // compute dual Chaikin vertices
  dualize();
     
  /************************************************************/
    
  glutInit (&argc, argv);			// primal space window
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
  int xsize, ysize;	// window size
  int xleft;		// x-coord of lefthand side
  int barmargin; 	// width of side bar surrounding picture
  int titleht; 		// top titlebar height
  int adualy, bdualy;   // starting y-coordinate of right windows
  int halfysize;        // size of top right window
  int dualxleft;        // left side of right windows
  if (WINDOWS)
    {
      xleft 	= 0;
      xsize 	= 350; ysize = 350;
      barmargin = 6;
      titleht 	= 12;
      halfysize = (ysize - 2*titleht)/2;
      adualy    = titleht+10;
      bdualy    = titleht+10+halfysize+2*titleht+1;
      dualxleft = xleft+xsize+2*barmargin-1;
    }
  else if (LAPTOP)
    {
      xleft     = 0;
      xsize     = ysize = 500;
      barmargin = 12;
      titleht   = 0;
      halfysize = (ysize - 24)/2;
      adualy    = 0;
      bdualy    = titleht+halfysize+24;
      dualxleft = xleft+xsize+barmargin;
    }
  else if (PRINTOUT)
    {
      xleft     = 0;
      xsize     = ysize = 350;              // less reduction required ==> better image clarity)
      barmargin = 8;
      titleht = 20;
      halfysize = (ysize - 2*titleht)/2;
      adualy    = titleht+10;
      bdualy    = titleht+10+halfysize+2*titleht+1;
      dualxleft = xleft+xsize+2*barmargin-1;
    }
  else
    {
      xleft 	= 0;			
      xsize     = ysize = 600;                // 600 for standard windows
      barmargin = 12;
      titleht 	= 0;
      halfysize = (ysize - 24)/2;
      adualy    = 0;
      bdualy    = titleht+halfysize+24;
      dualxleft = xleft+xsize+barmargin;
    }

  glutInitWindowPosition (xleft,titleht);		// primal window
  glutInitWindowSize (xsize,ysize);
  char titlebar[100]; 
  strcpy (titlebar, "Chaikin curves");  
  if (!PRINTOUT)
   {
    strcat (titlebar, " (");
    strcat (titlebar, argv[argc-1]);  
    strcat (titlebar, ")");
   }
  obstacleWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutVisibilityFunc (visibility);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuOb);
  glutAddMenuEntry ("Vertices",                 1);
  glutAddMenuEntry ("Edges",                    2);
  glutAddMenuEntry ("Previous step",            3);
  glutAddMenuEntry ("1st nhood",                4);
  glutAddMenuEntry ("2nd nhood",                5);
  glutAddMenuEntry ("No blue in 1st/2nd nhood", 6);
  glutAddMenuEntry ("Point labels",             7);
  glutAddMenuEntry ("Original polygon",         8);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft,adualy);		// a-dual window
  glutInitWindowSize (xsize,halfysize);
//  glutInitWindowSize (xsize, ysize);	// TEMPORARY, FOR 1ST NHOOD MECHANISM BETTER VIEW
  strcpy (titlebar, "Tangential a-curves: steep tangents");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry (" ", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft, bdualy);		// b-dual window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Tangential b-curves: shallow tangents");
  dualWin2 = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual2);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual2);
  glutAddMenuEntry (" ", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

