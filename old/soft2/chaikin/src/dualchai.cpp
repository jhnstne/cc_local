/*
  File:          dualchai.cpp
  Author:        J.K. Johnstone 
  Created:	 18 July 2001
  Last Modified: 15 July 2003
  Purpose:       Input defines the dual Chaikin curve instead,
  		 otherwise equivalent to chai.cpp.
		 Only change is predualize and argument to input function.
		 And addition of chaib and removal of chaidb.
  Input: 	 Closed control polygons.
  Output: 	 Point pairs defining bitangents.
  History: 	 7/15/03:  Added LAPTOP option.
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
#include "Vector.h"
#include "Line.h"

#define PTSPERBEZSEGMENT 10      // # pts to draw on each Bezier segment
#define WINDOWS 0		// 0 for running on SGI, 1 for Windows

static char *RoutineName;
static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-j] (just display obstacles)" << endl;
  cout << "\t[-n] (don't output V-graph to file)" << endl;
  cout << "\t[-e eps]  (accuracy at which intersections are made: default .0001)" << endl;
  cout << "\t[-p x y]  (pole coordinates: default (-4,-2))" << endl;
  cout << "\t[-d displaydensity of Bezier segment]" << endl;
  cout << "\t[-l] (laptop)" << endl;
  cout << "\t[-h] (this help message)" << endl;
  cout << "\t <file>.rawctr" << endl;
 }

static GLfloat   transxob, transyob, zoomob, transxdual, transydual, zoomdual;
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY

static GLboolean DRAWVERT=1;		// draw Chaikin control vertices?
static GLboolean DRAWEDGE=1;		// draw Chaikin control edges?
static GLboolean DRAWPREV=0;		// draw previous step?
static GLboolean DRAWSPLIT=0;		// draw vertices after splitting?
static GLboolean DRAWAVE=0;		// draw vertices after averaging?
static GLboolean DRAWBIGFIRST=0;	// emphasize mechanism of 1st vtx's nhood?
static GLboolean DRAWBIGSECOND=0;	// emphasize mechanism of 2nd vtx's nhood?
static GLboolean DONTDRAWBLUE=0;	// remove confusing blue lines to clarify red intersections
					// effectively stage=2.5 for 1st nhood display
static GLboolean DRAWADUAL=1;		// draw dual Chaikin built from a-space
static GLboolean DRAWBDUAL=1;		// draw dual Chaikin built from b-space

// static GLboolean DEBUG = 1;

V2fArrArr	chai,chaib;		// Chaikin control vertices (for each polygon)
					// (they change with each subdivision)
					// for both primal spaces
V2fArrArr	chaicirc,chaicircb;	// control vertices after splitting
V2fArrArr	prevchai,prevchaib;	// chai from previous step
V2fArrArr	chaid,chaidb;		// a-dual and b-dual Chaikin control vertices
V2fArrArr	prevchaid,prevchaidb;  	// chaid and chaidb from previous step
int 		stage=0;		// display stage
IntArr		activeEdge;		// active edges for chai[0] (a-space)
IntArr		prevActiveEdge;		
IntArr		activeEdgeb;		// active edges for chaidb[0] (b-space)
IntArr 		prevActiveEdgeb;

int			obstacleWin;	// identifier for top right window
int 			obstacleWin2;   // identifier for bottom left window
int			dualWin;	// identifier for dual window
int 	                PRINTOUT=0;     // 0 for displaying on screen, 1 for printing out image
int                     LAPTOP=0;       // display environment for laptop?

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

  transxdual = transydual = 0.;
  zoomob = .25;
  zoomdual = .5; 
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
	Map dual Chaikin polygons to a-Chaikin and b-Chaikin polygons 
	(only at beginning).
	Since edge chai[0]-->chai[1] maps to chaid[0],
	we want chaid[0]-->chaid[1] to map to chai[1].
	(The vertices of this edge should map to the edges chai[0]-->chai[1]
	and chai[1]-->chai[2], which intersect in chai[1].)
******************************************************************************/

void predualize()
{
  int i;
  chai[0].allocate(chaid[0].getn());
  chaib[0].allocate(chaid[0].getn());
  activeEdge.allocate(chaid[0].getn());
  activeEdgeb.allocate(chaid[0].getn());
  for (i=0; i<chaid[0].getn(); i++)	// for each edge
   {
    float a,b,c;	// coordinates of line ax+by+c=0 defining ith edge
    Line2f L(chaid[0][i], chaid[0][(i+1)%chaid[0].getn()]);
    L.implicitEqn (a,b,c);
    chai[0][(i+1)%chai[0].getn()][0] = c/a;		// a-duality
    chai[0][(i+1)%chai[0].getn()][1] = b/a;
    chaib[0][(i+1)%chai[0].getn()][0] = a/b;		// b-duality
    chaib[0][(i+1)%chai[0].getn()][1] = c/b;
   }
  for (i=0; i<chai[0].getn(); i++)
    if (chai[0][i][0] * chai[0][(i+1)%chai[0].getn()][0] < 0) // straddles x=0?
         activeEdge[i] = 0;	// edge i->i+1 is inactive
    else activeEdge[i] = 1;
  for (i=0; i<chaib[0].getn(); i++)
    if (chaib[0][i][1] * chaib[0][(i+1)%chaib[0].getn()][1]<0) // straddles y=0?
         activeEdgeb[i] = 0;	// edge i->i+1 is inactive
    else activeEdgeb[i] = 1;
}

/******************************************************************************
	Map Chaikin polygons to dual Chaikin polygons.
	Use a-Chaikin for all but pseudo-horizontal edges in dual space.
	That is, use b-Chaikin for new edges chai[2j-1] --> chai[2j]
	such that prevchai[j][1] * prevchai[j+1][1] < 0	(y-coord sign change)
	or 	  prevchai[j][1] * prevchai[j-1][1] < 0
	(See notes.)
	
	Only have to dualize alternate edges (2j-1,2j) since 
	edges (2j,2j+1) remain the same as previous stage's (j,j+1).
******************************************************************************/

void dualize()
{
  int i;
  prevchaid = chaid;
  prevchaidb = chaidb;
  chaid[0].allocate(chai[0].getn());
  chaidb[0].allocate(chai[0].getn());
  for (i=0; i<chai[0].getn(); i+=2)	// these dual vertices remain the same
   {
    chaid[0][i] = prevchaid[0][i/2];
    chaidb[0][i] = prevchaidb[0][i/2];	
   }
  for (i=1; i<chai[0].getn(); i+=2)	// for each line (edge) that changes
   {
    float a,b,c;	// coordinates of line ax+by+c=0 defining ith edge
//    int j=((i+1)/2)%prevchai[0].getn();	// i+1 is the even number 2j
//    if (prevchai[0][j][0] * prevchai[0][(j+1)%prevchai[0].getn()][0] < 0 ||
//        prevchai[0][j][0] * prevchai[0][mod(j-1,prevchai[0].getn())][0] < 0)
//     {
      Line2f L(chaib[0][i], chaib[0][(i+1)%chaib[0].getn()]);
      L.implicitEqn (a,b,c);
      chaidb[0][i][0] = a/b;		// b-duality
      chaidb[0][i][1] = c/b;
//     }
//    else
//     { 
      Line2f La (chai[0][i], chai[0][(i+1)%chai[0].getn()]);
      La.implicitEqn (a,b,c);
      chaid[0][i][0] = c/a;		// a-duality
      chaid[0][i][1] = b/a;
//     }
   }
//  cout << prevchaid << endl << chaid << endl;
}

/******************************************************************************
	Splitting step (see p. 63 of Stollnitz, DeRose and Salesin `Wavelets')
******************************************************************************/

void split()
{
  int i,j;
  int nPt = chai[0].getn();
  chaicirc[0].allocate (2*nPt);
  for (i=0; i<nPt; i++)
    for (j=0; j<2; j++)	// each coordinate
     {
      chaicirc[0][2*i][j]   = chai[0][i][j];
      chaicirc[0][2*i+1][j] = (chai[0][i][j] + chai[0][(i+1)%nPt][j]) / 2.;
     }

  nPt = chaib[0].getn();
  chaicircb[0].allocate (2*nPt);
  for (i=0; i<nPt; i++)
    for (j=0; j<2; j++)	// each coordinate
     {
      chaicircb[0][2*i][j]   = chaib[0][i][j];
      chaicircb[0][2*i+1][j] = (chaib[0][i][j] + chaib[0][(i+1)%nPt][j]) / 2.;
     }
}

/******************************************************************************
	Averaging step (see p. 63 of Stollnitz, DeRose and Salesin `Wavelets')
******************************************************************************/

void average()
{
  int i,j;
  prevchai = chai;
  prevActiveEdge = activeEdge;
  prevActiveEdgeb = activeEdgeb;
  int nPt = chaicirc[0].getn();
  chai[0].allocate (nPt);
  activeEdge.allocate (nPt);
  activeEdgeb.allocate(nPt);
  for (i=0; i<nPt; i++)
    for (j=0; j<2; j++)	// each coordinate
      chai[0][i][j] = (chaicirc[0][i][j] + chaicirc[0][(i+1)%nPt][j]) / 2.;
      
  // update active edges: any edge in stage i+1 with a vertex lying on an
  // inactive edge of stage i is marked inactive.
  // That is, inactive edge (j,j+1) in stage i becomes
  // inactive edges (2j-1,2j), (2j,2j+1), and (2j+1,2j+2) in stage i+1
  for (i=0; i<activeEdge.getn(); i++) activeEdge[i] = activeEdgeb[i] = 1;
  for (j=0; j<prevActiveEdge.getn(); j++)
   {
    if (prevActiveEdge[j] == 0)
      activeEdge[mod(2*j-1,activeEdge.getn())] =
      activeEdge[2*j] = 
      activeEdge[(2*j+1)%activeEdge.getn()] = 0;
    if (prevActiveEdgeb[j] == 0)
      activeEdgeb[mod(2*j-1,activeEdgeb.getn())] = 
      activeEdgeb[2*j] = 
      activeEdgeb[(2*j+1)%activeEdgeb.getn()] = 0;
   }

  prevchaib = chaib;
  nPt = chaicircb[0].getn();
  chaib[0].allocate (nPt);
  for (i=0; i<nPt; i++)
    for (j=0; j<2; j++)	// each coordinate
      chaib[0][i][j] = (chaicircb[0][i][j] + chaicircb[0][(i+1)%nPt][j]) / 2.;
}

/******************************************************************************
******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case '1':	DRAWVERT = !DRAWVERT;		break;
  case '2': 	DRAWEDGE = !DRAWEDGE;		break;
  case '3':	DRAWPREV = !DRAWPREV;		break;
  case '4':	DRAWBIGFIRST = !DRAWBIGFIRST;	break;
  case '5':	DRAWBIGSECOND = !DRAWBIGSECOND;	break;
  case '6':	DONTDRAWBLUE = !DONTDRAWBLUE;	break;
  case ' ':	if      (stage==0) 
     		 { split(); 
		   // dualsplit(); 
		   DRAWSPLIT=1; DRAWAVE=0; DRAWPREV=0; stage=1; }
		else if (stage==1) 
		 { average();  
		   DRAWSPLIT=1; DRAWAVE=1; DRAWPREV=1; stage=2; }
		else if (stage==2) 
		 { dualize();
		   DRAWSPLIT=0; DRAWAVE=0; DRAWPREV=1; stage=3; }
		else if (stage==3)
		 { DRAWSPLIT=0; DRAWAVE=0; DRAWPREV=0; stage=0; }
		 				break;	
  case 8:	if (stage==3)   		       // backspace
  		 { DRAWSPLIT=1; DRAWAVE=1; DRAWPREV=1; stage=2; }
		else if (stage==1)
		 { DRAWSPLIT=0; DRAWAVE=0; DRAWPREV=0; stage=0; }
		else if (stage==0)
		 { DRAWSPLIT=0; DRAWAVE=0; DRAWPREV=1; stage=3; }
		 				break;
  case 9:	split(); average(); dualize();  break; // TAB
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
  case 3: 	DRAWPREV = !DRAWPREV;			break;
  case 4:	DRAWBIGFIRST = !DRAWBIGFIRST;		break;
  case 5:	DRAWBIGSECOND = !DRAWBIGSECOND;		break;
  case 6: 	DONTDRAWBLUE = !DONTDRAWBLUE;		break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

void menuOb2 (int value)
{
  switch (value) {
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1:	DRAWADUAL = !DRAWADUAL;				break; 
  case 2:	DRAWBDUAL = !DRAWBDUAL;				break;
  default:   							break;
  }
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

void displayOb ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  
  glColor3fv (Black);
  glBegin(GL_POINTS);
  glVertex2f(0,0);
  glEnd();
  
  if (DRAWBIGFIRST)		// emphasize neighbourhood of 1st vertex
   {
    glPointSize(12.0);
    glLineWidth(3.0);
    glColor3fv (Red);
    glBegin (GL_POINTS);
    for (i=0; i<1; i++)
      for (j=(stage<2?0:1); j<2; j++)
        glVertex2f (chai[i][j][0], chai[i][j][1]);
    glEnd();
    if (stage<2)
     {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (i=0; i<1; i++)
        for (j=0; j<2; j++)
          glVertex2f (chai[i][j][0], chai[i][j][1]);
      glEnd();
     }
    else if (stage==3)
     {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (i=0; i<1; i++)
        for (j=1; j<3; j++)
          glVertex2f (chai[i][j][0], chai[i][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=(stage<2?0:1); j<3; j++)
	  glVertex2f (chaicirc[i][j][0], chaicirc[i][j][1]);
      glEnd();
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWBIGSECOND)		// emphasize neighbourhood of 2nd vertex
   {
    glPointSize(12.0);
    glLineWidth(3.0);
    glColor3fv (Red);
    glBegin (GL_POINTS);
    for (i=0; i<1; i++)
      for (j=(stage<2?1:2); j<3; j++)
        glVertex2f (chai[i][j][0], chai[i][j][1]);
    glEnd();
    if (stage<2)
     {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (i=0; i<1; i++)
        for (j=(stage<2?1:2); j<(stage<2?3:4); j++)
          glVertex2f (chai[i][j][0], chai[i][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=2; j<(stage<2?5:4); j++)
	  glVertex2f (chaicirc[i][j][0], chaicirc[i][j][1]);
      glEnd();
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWVERT && DRAWPREV)
   {
    glColor3fv (Tomato);
    glBegin (GL_POINTS);
    for (i=0; i<1; i++)
      for (j=0; j<prevchai[i].getn(); j++)
        glVertex2f (prevchai[i][j][0], prevchai[i][j][1]);
    glEnd();
   }
  if (DRAWEDGE && DRAWPREV)
   {
    glColor3fv (Black);
    glEnable(GL_LINE_STIPPLE);
    glBegin (GL_LINE_LOOP);
    for (i=0; i<1; i++)
      for (j=0; j<prevchai[i].getn(); j++)
        glVertex2f (prevchai[i][j][0], prevchai[i][j][1]);
    glEnd();
    glDisable(GL_LINE_STIPPLE);
   }
  if (DRAWVERT)			// display control vertices
   {
    glColor3fv (Red);
    glBegin (GL_POINTS);
    for (j=0; j<chai[0].getn(); j++)
      if (activeEdge[mod(j-1,chai[0].getn())] && activeEdge[j])
        glVertex2f (chai[0][j][0], chai[0][j][1]);
    glEnd();
   }
  if (DRAWEDGE && !DRAWAVE)	// display new control edges
   {
    glColor3fv (Black);
    glBegin (GL_LINES);
    for (j=0; j<activeEdge.getn(); j++)
      if (activeEdge[j])
       {
        glVertex2f (chai[0][j][0], chai[0][j][1]);
	glVertex2f (chai[0][(j+1)%chai[0].getn()][0],
		    chai[0][(j+1)%chai[0].getn()][1]);
       }
    glEnd();
   }
  if (DRAWSPLIT)		// display vertices after splitting stage
   {
    glColor3fv (Blue);
    glBegin (GL_POINTS);
    for (i=0; i<1; i++)
      for (j=0; j<chaicirc[i].getn(); j++)
        glVertex2f (chaicirc[i][j][0], chaicirc[i][j][1]);
    glEnd();
   }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

void displayOb2 ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxob, transyob, 0);
  glScalef  (zoomob, zoomob, zoomob);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  
  if (DRAWVERT && DRAWPREV)
   {
    glColor3fv (Tomato);
    glBegin (GL_POINTS);
    for (i=0; i<1; i++)
      for (j=0; j<prevchaib[i].getn(); j++)
        glVertex2f (prevchaib[i][j][0], prevchaib[i][j][1]);
    glEnd();
   }
  if (DRAWEDGE && DRAWPREV)
   {
    glColor3fv (Black);
    glEnable(GL_LINE_STIPPLE);
    glBegin (GL_LINE_LOOP);
    for (i=0; i<1; i++)
      for (j=0; j<prevchaib[i].getn(); j++)
        glVertex2f (prevchaib[i][j][0], prevchaib[i][j][1]);
    glEnd();
    glDisable(GL_LINE_STIPPLE);
   }
  if (DRAWVERT)			// display control vertices
   {
    glColor3fv (Blue);
    glBegin (GL_POINTS);
    for (j=0; j<chaib[0].getn(); j++)
      if (activeEdgeb[mod(j-1,chaib[0].getn())] && activeEdgeb[j])
        glVertex2f (chaib[0][j][0], chaib[0][j][1]);
    glEnd();
   }
  if (DRAWEDGE && !DRAWAVE)	// display new control edges
   {
    glColor3fv (Green);
    glBegin (GL_LINES);
    for (j=0; j<chaib[0].getn(); j++)
      if (activeEdgeb[j])
       {
        glVertex2f (chaib[0][j][0], chaib[0][j][1]);
	glVertex2f (chaib[0][(j+1)%chaib[0].getn()][0],
		    chaib[0][(j+1)%chaib[0].getn()][1]);
       }
    glEnd();
   }
  if (DRAWSPLIT)		// display vertices after splitting stage
   {
    glColor3fv (Blue);
    glBegin (GL_POINTS);
    for (i=0; i<1; i++)
      for (j=0; j<chaicircb[i].getn(); j++)
        glVertex2f (chaicircb[i][j][0], chaicircb[i][j][1]);
    glEnd();
   }
  
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

void displayDual ()
{
  int i,j;
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
    for (i=0; i<1; i++)
      for (j=(stage<2?0:1); j<2; j++)
        drawImplicitLine (1, chai[i][j][1], chai[i][j][0]); // (a,b,1)-> x+by+a=0
    if (stage<2)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=0; j<1; j++)
          glVertex2f (chaid[i][j][0], chaid[i][j][1]);
      glEnd();
     }
    else if (stage==3)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=1; j<2; j++)
          glVertex2f (chaid[i][j][0], chaid[i][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      for (i=0; i<1; i++)
        for (j=(stage<2?0:1); j<3; j++)
          drawImplicitLine (1, chaicirc[i][j][1], chaicirc[i][j][0]);
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWBIGSECOND)
   {
    glPointSize(12.0);
    glLineWidth(3.0);
    glColor3fv (Red);	// test
    for (i=0; i<1; i++)
      for (j=(stage<2?1:2); j<3; j++)
        drawImplicitLine (1, chai[i][j][1], chai[i][j][0]); // (a,b,1)-> x+by+a=0
    if (stage<2)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=(stage<2?1:2); j<(stage<2?2:3); j++)
          glVertex2f (chaid[i][j][0], chaid[i][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Blue);
      for (i=0; i<1; i++)
        for (j=2; j<(stage<2?5:4); j++)
          drawImplicitLine (1, chaicirc[i][j][1], chaicirc[i][j][0]);
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWVERT && DRAWPREV)	
   {
    glColor3fv (Black);
    glBegin (GL_POINTS);
    for (i=0; i<1; i++)
      for (j=0; j<prevchaid[i].getn(); j++)
        glVertex2f (prevchaid[i][j][0], prevchaid[i][j][1]);
    glEnd();
   }
  if (DRAWEDGE && DRAWPREV)
   {
    glColor3fv (Red);
    glEnable(GL_LINE_STIPPLE);
    glBegin (GL_LINE_LOOP);
    for (i=0; i<1; i++)
      for (j=0; j<prevchaid[i].getn(); j++)
        glVertex2f (prevchaid[i][j][0], prevchaid[i][j][1]);
    glEnd();
    glDisable(GL_LINE_STIPPLE);
   }
  if (DRAWVERT)			// display control vertices
   {
    if (DRAWADUAL)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (j=0; j<chaid[0].getn(); j++)
//        if (activeEdge[j])
          glVertex2f (chaid[0][j][0], chaid[0][j][1]);
      glEnd();
     }
    if (DRAWBDUAL)
     {
      glColor3fv (Green);		// b-duality version
      glBegin (GL_POINTS);
      for (j=0; j<chaidb[0].getn(); j++)
//        if (activeEdgeb[j])
          glVertex2f (chaidb[0][j][0], chaidb[0][j][1]);
      glEnd();
     }
   }
  if (DRAWEDGE)			// display control edges
   {
    if (DRAWADUAL)
     {
      glColor3fv (Red);
      glBegin (GL_LINES);
      for (j=0; j<chaid[0].getn(); j++)
//        if (activeEdge[j] && activeEdge[(j+1)%chaid[0].getn()])
	 {
	  glVertex2f (chaid[0][j][0], chaid[0][j][1]);
	  glVertex2f (chaid[0][(j+1)%chaid[0].getn()][0],
	  	      chaid[0][(j+1)%chaid[0].getn()][1]);
	 }
      glEnd();
     }
    if (DRAWBDUAL)
     {
      glColor3fv (Blue);		// b-duality version
      glBegin (GL_LINES);
      for (j=0; j<chaidb[0].getn(); j++)
//        if (activeEdgeb[j] && activeEdgeb[(j+1)%chaidb[0].getn()])
	 {
          glVertex2f (chaidb[0][j][0], chaidb[0][j][1]);
	  glVertex2f (chaidb[0][(j+1)%chaidb[0].getn()][0], 
	  	      chaidb[0][(j+1)%chaidb[0].getn()][1]);
	 }	
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
  chai.allocate(2);			// 2 Chaikin curves
  V2fArrArr Pt(2);			
  for (i=0; i<1; i++)		// only 1 Chaikin for now, testing
   {
    getLeftBrace(infile);
    int mark = infile.tellg(); V2f foo; int nPt=0;
    while (!tryToGetRightBrace(infile))
     {
      infile >> foo[0] >> foo[1];  nPt++;
     }
    assert (nPt>0);  Pt[i].allocate(nPt);
    infile.seekg(mark);
    infile >> Pt[i][0][0] >> Pt[i][0][1];
    for (j=1; j<nPt; j++)
     {
      infile >> Pt[i][j][0] >> Pt[i][j][1];
      if (Pt[i][j]==Pt[i][j-1]) { j--; nPt--; }		// skip duplicate
      if (j==nPt-1 && Pt[i][j]==Pt[i][0]) nPt--;	// skip duplicate at end, too
     }
    getRightBrace(infile);
    chai[i].allocate(nPt);
    for (j=0; j<nPt; j++)	chai[i][j] = Pt[i][j];
   }
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  int       ArgsParsed=0;
//  float     eps = .0001;	// accuracy of intersection computation

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc)
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      {
//      case 'e': eps = atof(argv[ArgsParsed++]);			break;
      case 'l': LAPTOP = 1;                                     break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
    else ArgsParsed++;
   }
  
  string filePrefix;
  input(argv[argc-1], filePrefix, chaid);
  chaidb = chaid;
  chai.allocate(2);  chaicirc.allocate(2);
  chaib.allocate(2); chaicircb.allocate(2);

  // compute associated Chaikin vertices
  predualize();
  prevchai  = chai;	// no predecessor at beginning
  prevchaib = chaib;
  prevchaid = chaid;	
     
  /************************************************************/
    
  glutInit (&argc, argv);			// a-primal space window
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
      barmargin = 8;
      titleht 	= 20;
      halfysize = (ysize - 2*titleht)/2;
      adualy    = titleht+10;
      bdualy    = titleht+10+halfysize+2*titleht+1;
      dualxleft = xleft+xsize+2*barmargin-1;
    }

  /*  int titleht = 20; 	// top titlebar is 20 units high
//  int xleft    = 164;	// x-coord of lefthand side for small windows
//  int xsize = 400, ysize = 400;	// small windows
  int xleft = 0;	// x-coord of lefthand side for large windows
  int xsize = 600, ysize = 600;		// large windows
  int barmargin = 8; 	// width of side bar surrounding picture
  int halfysize = (ysize - 2*titleht)/2; */

  glutInitWindowPosition (xleft,adualy);
  glutInitWindowSize (xsize,halfysize);
  char titlebar[100]; 
  strcpy (titlebar, "Primal a-Chaikin curves");  
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
  glutAttachMenu (GLUT_RIGHT_BUTTON);
  
  glutInitWindowPosition (xleft,bdualy);	// primal b-Chaikin space window
  glutInitWindowSize (xsize,halfysize);
  strcpy (titlebar, "Primal b-Chaikin curves");
  obstacleWin2 = glutCreateWindow (titlebar);
  glutDisplayFunc (displayOb2);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motionob);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuOb2);
  glutAddMenuEntry (" ", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutInitWindowPosition (dualxleft,titleht);  // a-dual space window
  glutInitWindowSize (xsize, ysize);
  strcpy (titlebar, "Dual Chaikin curves");
  dualWin = glutCreateWindow (titlebar);
  glutDisplayFunc (displayDual);
  glutKeyboardFunc (keyboard);
  glutMouseFunc (mouse);
  glutMotionFunc (motiondual);
  glutReshapeFunc (reshape);
  gfxinit();
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("a-dual", 1);
  glutAddMenuEntry ("b-dual", 2);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

