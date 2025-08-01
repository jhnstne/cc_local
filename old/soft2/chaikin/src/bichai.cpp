/*
  File:          bichai.cpp
  Author:        J.K. Johnstone 
  Created:	 3 July 2001
  Last Modified: 28 February 2006
  Purpose:       Compute bitangents in dual space
  		 of two Chaikin subdivision curves.
  Input: 	 Closed control polygons.
  Output: 	 Point pairs defining bitangents.
  History: 	 9/27/02: Cleaned up infinite behaviour.
  		 9/28/02: Compute bitangents correctly right from stage 0.
		 	  Introduce sleeping segments (active but infinite,
			  and so not involved in intersection tests yet).
			  For example, in a-dual part of bichai2.pts,
			  there is no initial intersection, but the 
			  infinite segments eventually spawn finite children
			  segments that do intersect.
		 6/13/03: Displayed one vertex at a time and its matching dual edge,
		          to allow visual understanding and testing of duality.
		 7/15/03: Added LAPTOP option.
		 2/28/06: Updated to modern C++ library.
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
static GLboolean DRAWPREV=0;		// draw previous step?
static GLboolean DRAWSPLIT=0;		// draw vertices after splitting?
static GLboolean DRAWAVE=0;		// draw vertices after averaging?
static GLboolean DRAWBIGFIRST=0;	// emphasize mechanism of 1st vtx's nhood?
static GLboolean DRAWBIGSECOND=0;	// emphasize mechanism of 2nd vtx's nhood?
static GLboolean DONTDRAWBLUE=0;	// remove confusing blue lines to clarify red intersections
					// effectively stage=2.5 for 1st nhood display
static GLboolean DRAWBITANG=0;		// draw bitangents?					
static GLboolean DRAWACTIVE=0;		// draw active edges in dual space 
static GLboolean DRAWSPECIFICVTX=0;     // draw emphasized specific vertex in primal space
                                        // and its matching edge in dual space?
static GLboolean WINDOWS=0;		// 0 for running on Linux, 1 for Windows

// static GLboolean DEBUG = 1;

V2fArrArr	chai;			// Chaikin control vertices (for each polygon)
					// (they change with each subdivision)
V2fArrArr	chaicirc;		// control vertices after splitting
V2fArrArr	chaida;			// a-dual Chaikin control vertices
V2fArrArr	chaidb;			// b-dual Chaikin control vertices
IntArrArr	finiteA;		// finite a-dual vertices
IntArrArr	finiteB;
IntArrArr	prevFiniteA;		// finiteA from previous step
IntArrArr	prevFiniteB;
V2fArrArr	prevchai;		// chai from previous step
V2fArrArr	prevchaida,prevchaidb;  // chaida(b) from previous step
int 		stage=0;		// display stage
int		subStage=0;		// subdivision stage
IntArrArr	activeA,activeB;	// active flags for lazy subdivision, in each dual space
					// activeA[j] = 1 iff edge from chaida[j}
					// to chaida[j+1] is active (in dual space)
					// (equivalently, in primal space,
					// corner chai[j+1] is active)
IntArrArr	prevActiveA,prevActiveB;// activeA(b) from previous step
int		nBitang;		// # of bitangents at this stage
V2iArr		bitang;			// bitangents at this stage: (index on 0 curve, index on 1 curve)
			// equivalently, 2 active corners, arising from, and thus 
			// associated with, a single intersection in dual space
IntArr          presentVtx(2);          // present vtx to display on chai[i] in primal space

int			obstacleWin;	// identifier for left obstacle window
int			dualWin;	// identifier for top right dual window
int			dualWin2;	// identifier for bottom right dual window
int 			PRINTOUT=0;	// 0 for displaying on screen, 1 for printing out image
int                     LAPTOP=0;       // display environment for laptop?

/******************************************************************************
******************************************************************************/

void gfxinit(void)
{
  glClearColor (1.0, 1.0, 1.0, 1.0);

  glEnable (GL_BLEND);				
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable (GL_LINE_SMOOTH);			
  glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable (GL_POINT_SMOOTH); 			// too slow for rotation 
  glHint (GL_POINT_SMOOTH_HINT, GL_FASTEST); 
  glPointSize (6.0);
  glLineStipple (1, 0xAAAA);

  transxob = transyob = 0;
  zoomob = 1.;
//  zoomdual = .25; 
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
	Does the dihedral angle at the primal vertex chai[i][j]
	straddle the x-axis (straddle a horizontal tangent)?
	Assumes the input polygon is oriented ccw?
	Straddles iff the incoming vector and outgoing vectors are on
	different sides of the x-axis.
******************************************************************************/

bool straddleHoriz (int i, int j)
{
  int n=chai[i].getn();
  float yIncoming = chai[i][j][1]       - chai[i][mod(j-1,n)][1];
  float yOutgoing = chai[i][(j+1)%n][1] - chai[i][j][1];
  return (yIncoming * yOutgoing <= 0);
}

/******************************************************************************
	Does the dihedral angle at the primal vertex chai[i][j] 
	straddle the y-axis (straddle a vertical tangent)?
	Assumes the input polygon is oriented ccw?
	Straddles iff the incoming vector and outgoing vectors are on
	different sides of the y-axis.
******************************************************************************/

bool straddleVert (int i, int j)
{
  int n=chai[i].getn();
  float xIncoming = chai[i][j][0]       - chai[i][mod(j-1,n)][0];
  float xOutgoing = chai[i][(j+1)%n][0] - chai[i][j][0];
  return (xIncoming * xOutgoing <= 0);
}

/******************************************************************************
    If a low stage, retain all segments active and return.
    All children of inactive segments are set to inactive.
    All children of active segments are set to neutral.
    For all neutral yellow segments S from stage i+1
      For all neutral-or-active red segments T from stage i+1
	If S and T cross
	  Set both S and T to active (cannot break here, since yellow S may cross another red)
      If S is still neutral, set S to inactive
    For all neutral red segments from stage i+1, set to inactive
    aFlag = 1 iff chaidual lives in a-dual space (as opposed to b-dual space).
******************************************************************************/

void updateActive (IntArrArr &active, IntArrArr &oldactive, V2fArrArr &chaidual,
		   IntArrArr &finite, int aFlag)
{
 cout << "Upon entering, oldactive = " << oldactive << endl;
  int i,j;
  if (subStage > 0)	
   {
    for (i=0; i<2; i++)
      for (j=0; j<oldactive[i].getn(); j++) 
        if (oldactive[i][j] == 0)		// inactive segments remain inactive
          active[i][2*j] = active[i][2*j+1] = 0;
        else					// active segments are neutral (don't know if they will remain active)
          active[i][2*j] = active[i][2*j+1] = 2;
   }
  else // 1st time, active is not twice as big as oldactive (and no filtering will ever be done)
    for (i=0; i<2; i++)
      for (j=0; j<active[i].getn(); j++)
        active[i][j] = 2;
  IntArrArr sleepActive(active);  	// sleepActive[i][j] = 1 iff jth segment of ith curve involves infinite points (so it should sleep until it becomes finite)
  sleepActive[0].clear(); sleepActive[1].clear();
  for (i=0; i<2; i++)		// mark as 'sleeping' any segments abutting infinity (active but not yet testable for intersection)
    for (j=0; j<active[i].getn(); j++)
     {
      int jp1 = (j+1)%finite[i].getn();
      if (!finite[i][j] || 
      	  !finite[i][jp1] ||
	  (aFlag ? straddleHoriz(i,jp1)	// sleep any finite segment straddling infinity
		 : straddleVert (i,jp1)))
        active[i][j] = sleepActive[i][j] = 1;
      // THIS IS INAPPROPRIATE SINCE IT SETS THIS SEGMENT INACTIVE FOREVER (AT WORST IT SHOULD SLEEP)
      //      else if (aFlag ? straddleHoriz(i,jp1)	// mark as 'inactive' any finite segment straddling infinity
      //	  	     : straddleVert (i,jp1))	// THIS MAY ONLY NEED TO BE CHECKED FOR SUBSTAGE=0 (if we continue to inherit inactive segments)
      //	active[i][j] = 0;
     }
  for (i=0; i<active[0].getn(); i++)	
    // TRY NOT TURNING SEGMENTS INACTIVE
    if (!sleepActive[0][i])
    //    if (active[0][i] == 2) // for all neutral yellow segments that are not sleeping
     {
      for (j=0; j<active[1].getn(); j++)
	// TRY NOT TESTING JUST ACTIVE SEGMENTS
	if (!sleepActive[1][j])
        // if (active[1][j] > 0 && !sleepActive[1][j])	// for all neutral-active red segments that are not sleeping
	  {  // test for intersection
	    Line2f redSeg(chaidual[0][i], chaidual[0][(i+1)%chai[0].getn()]);
	    Line2f blueSeg   (chaidual[1][j], chaidual[1][(j+1)%chai[1].getn()]);
	    V2f foo; float bar;
	    if (redSeg.intersect (blueSeg, foo, bar, 1)) // intersect closed segments
	     {
	      if (nBitang == bitang.getn()) 
	       { cout << "Too many bitangents"; exit(-1); }
	      active[0][i] = active[1][j] = 1;
	      bitang[nBitang][0]   = (i+1)%chai[0].getn();  // yellow active corner is i+1
	      bitang[nBitang++][1] = (j+1)%chai[1].getn();  // red active corner is j+1
	      cout << "Bitangent found from " << bitang[nBitang-1][0] << " to " << bitang[nBitang-1][1] << endl;
	     }
	 }
      if (active[0][i] == 2)		// still neutral
        active[0][i] = 0;
     }
  for (i=0; i<active[1].getn(); i++)
    if (active[1][i] == 2)		// for all neutral red segments
      active[1][i] = 0;
}

/******************************************************************************
	Map Chaikin polygons to dual Chaikin polygons.
******************************************************************************/

void dualize()
{
  int i;
  prevchaida = chaida;
  prevchaidb = chaidb;
  prevFiniteA = finiteA; prevFiniteB = finiteB;
  prevActiveA = activeA; prevActiveB = activeB;
  for (i=0; i<2; i++)
   {
    chaida[i].allocate(chai[i].getn());  chaidb[i].allocate(chai[i].getn());
    finiteA[i].allocate(chai[i].getn()); finiteB[i].allocate(chai[i].getn());
    activeA[i].allocate(chai[i].getn()); activeB[i].allocate(chai[i].getn());
   }
  for (i=0; i<2; i++)
    for (int j=0; j<chai[i].getn(); j++)	// for each edge
     {
      float a,b,c;	// coordinates of line ax+by+c=0 defining jth edge
      Line2f L(chai[i][j], chai[i][(j+1)%chai[i].getn()]);
      L.implicitEqn (a,b,c);
      if (a != 0)
       {
        chaida[i][j][0] = c/a;		// a-duality
        chaida[i][j][1] = b/a;
	finiteA[i][j] = 1;
       }
      else 
	{
	  chaida[i][j].clear();
	  finiteA[i][j] = 0;
	}
      if (b != 0)
       {
        chaidb[i][j][0] = a/b;		// b-duality
	chaidb[i][j][1] = c/b; 
	finiteB[i][j] = 1;
       }
      else 
	{
	  chaidb[i][j].clear();
	  finiteB[i][j] = 0;
	}
     }
  nBitang = 0;
  //  bitang.allocate(chai[0].getn());	// very conservative estimate!!
  bitang.allocate(1000);
  updateActive (activeA, prevActiveA, chaida, finiteA, 1);
  updateActive (activeB, prevActiveB, chaidb, finiteB, 0);
}

/******************************************************************************
	Splitting step (see p. 63 of Stollnitz, DeRose and Salesin `Wavelets')
******************************************************************************/

void split()
{
  for (int c=0; c<2; c++)
   {
    int nPt = chai[c].getn();
    chaicirc[c].allocate (2*nPt);
    for (int i=0; i<nPt; i++)
      for (int j=0; j<2; j++)	// each coordinate
       {
        chaicirc[c][2*i][j]   = chai[c][i][j];
        chaicirc[c][2*i+1][j] = (chai[c][i][j] + chai[c][(i+1)%nPt][j]) / 2.;
       }
   }
}

/******************************************************************************
	Averaging step (see p. 63 of Stollnitz, DeRose and Salesin `Wavelets')
******************************************************************************/

void average()
{
  prevchai = chai;
  for (int c=0; c<2; c++)
   {
    int nPt = chaicirc[c].getn();
    chai[c].allocate (nPt);
    for (int i=0; i<nPt; i++)
      for (int j=0; j<2; j++)	// each coordinate
        chai[c][i][j] = (chaicirc[c][i][j] + chaicirc[c][(i+1)%nPt][j]) / 2.;
   }
  subStage++;
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
  case 'a':	DRAWACTIVE = !DRAWACTIVE; /* if (DRAWACTIVE)
    		 { DRAWACTIVE=DRAWVERT=0; DRAWEDGE=1; }
		else
		{ DRAWACTIVE=DRAWEDGE=1; DRAWVERT=0; } */
		 				break;
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
  case 's':     DRAWSPECIFICVTX = !DRAWSPECIFICVTX; break;
  case 'n':     presentVtx[0] = (presentVtx[0]+1)%chai[0].getn(); break;
  case 'm':     presentVtx[1] = (presentVtx[1]+1)%chai[1].getn(); break;
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
  case 7:	DRAWBITANG = !DRAWBITANG;		break;
  case 8:       presentVtx[0] = (presentVtx[0]+1)%chai[0].getn(); break;
  case 9:       presentVtx[1] = (presentVtx[1]+1)%chai[1].getn(); break;
  case 10:      DRAWSPECIFICVTX = !DRAWSPECIFICVTX;     break;
  default:   						break;
  }
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

void menuDual (int value)
{
  switch (value) {
  case 1: 	DRAWACTIVE = !DRAWACTIVE;               break;
  default:   						break;
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
  
  if (DRAWBIGFIRST)		// emphasize neighbourhood of 1st vertex
   {
    glPointSize(12.0);
    glLineWidth(3.0);
    glColor3fv (Red);
    if (WINDOWS)
     {
      for (i=0; i<1; i++)
        for (j=(stage<2?0:1); j<2; j++)
          drawPt (chai[i][j][0], chai[i][j][1]);
     }
    else
     {
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=(stage<2?0:1); j<2; j++)
          glVertex2f (chai[i][j][0], chai[i][j][1]);
      glEnd();
     }
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
      glColor3fv (Green);
      if (WINDOWS)
       {
        for (i=0; i<1; i++)
          for (j=(stage<2?0:1); j<3; j++)
	    drawPt (chaicirc[i][j][0], chaicirc[i][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
        for (i=0; i<1; i++)
          for (j=(stage<2?0:1); j<3; j++)
	    glVertex2f (chaicirc[i][j][0], chaicirc[i][j][1]);
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
      for (i=0; i<1; i++)
        for (j=(stage<2?1:2); j<3; j++)
          drawPt (chai[i][j][0], chai[i][j][1]);
     }
    else
     {
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=(stage<2?1:2); j<3; j++)
          glVertex2f (chai[i][j][0], chai[i][j][1]);
      glEnd();
     }
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
      glColor3fv (Green);
      if (WINDOWS)
       {
        for (i=0; i<1; i++)
          for (j=2; j<(stage<2?5:4); j++)
	    drawPt (chaicirc[i][j][0], chaicirc[i][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
        for (i=0; i<1; i++)
          for (j=2; j<(stage<2?5:4); j++)
	    glVertex2f (chaicirc[i][j][0], chaicirc[i][j][1]);
        glEnd();
       }
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  if (DRAWVERT && DRAWPREV)
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue); 		// Tomato : Gold
      if (WINDOWS)
       {
        for (j=0; j<prevchai[i].getn(); j++)
          drawPt (prevchai[i][j][0], prevchai[i][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
        for (j=0; j<prevchai[i].getn(); j++)
          glVertex2f (prevchai[i][j][0], prevchai[i][j][1]);
        glEnd();
       }
     }
   }
  if (DRAWEDGE && DRAWPREV)
   {
    glEnable(GL_LINE_STIPPLE);
    for (i=0; i<2; i++)
     {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (j=0; j<prevchai[i].getn(); j++)
        glVertex2f (prevchai[i][j][0], prevchai[i][j][1]);
      glEnd();
     }
    glDisable(GL_LINE_STIPPLE);
   }
  if (DRAWVERT)			// display control vertices
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      if (WINDOWS)
       {
        for (j=0; j<chai[i].getn(); j++)
          drawPt (chai[i][j][0], chai[i][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
        for (j=0; j<chai[i].getn(); j++)
          glVertex2f (chai[i][j][0], chai[i][j][1]);
        glEnd();
       }
     }
   }
  if (DRAWEDGE && !DRAWAVE)	// display new control edges
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (Black);
      glBegin (GL_LINE_LOOP);
      for (j=0; j<chai[i].getn(); j++)
        glVertex2f (chai[i][j][0], chai[i][j][1]);
      glEnd();
     }
   }
  if (DRAWSPLIT)		// display vertices after splitting stage
   {
    for (i=0; i<2; i++)
     {
      glColor3fv(i==0 ? Red : Blue);
      if (WINDOWS)
       {
        for (j=0; j<chaicirc[i].getn(); j++)
          drawPt (chaicirc[i][j][0], chaicirc[i][j][1]);
       }
      else
       {
        glBegin (GL_POINTS);
        for (j=0; j<chaicirc[i].getn(); j++)
          glVertex2f (chaicirc[i][j][0], chaicirc[i][j][1]);
        glEnd();
       }
     }
   }
  if (DRAWACTIVE)
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      for (j=0; j<activeA[i].getn(); j++)
        if (activeA[i][j] == 1)
	 {
          glBegin(GL_POINTS);
	  glVertex2f (chai[i][(j+1)%chai[i].getn()][0], chai[i][(j+1)%chai[i].getn()][1]);
	  glEnd();
	 }
     }	  
   }
  if (DRAWBITANG && !DRAWAVE)		// bitangents are screwed up during averaging stage (until dualize is again performed)
   {
    glColor3fv (Black);
    glEnable (GL_LINE_STIPPLE);
    glBegin(GL_LINES);
    for (i=0; i<nBitang; i++)
     {
      glVertex2f (chai[0][ bitang[i][0] ][0], chai[0][bitang[i][0]][1]);
      glVertex2f (chai[1][ bitang[i][1] ][0], chai[1][bitang[i][1]][1]);
     }
    glEnd();
    glDisable (GL_LINE_STIPPLE);
   }

  // draw a specific vertex (and its dual edge in the dual window)
  if (DRAWSPECIFICVTX)
   {
    glPointSize (12.);
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      glBegin (GL_POINTS);
      glVertex2f (chai[i][presentVtx[i]][0], chai[i][presentVtx[i]][1]);  // present vertex
      glEnd();
     }
    glPointSize(6.);
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
  
  // --------------------------------------------------------------------------
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
          glVertex2f (chaida[i][j][0], chaida[i][j][1]);
      glEnd();
     }
    else if (stage==3)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=1; j<2; j++)
          glVertex2f (chaida[i][j][0], chaida[i][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Green);
      for (i=0; i<1; i++)
        for (j=(stage<2?0:1); j<3; j++)
          drawImplicitLine (1, chaicirc[i][j][1], chaicirc[i][j][0]);
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  // --------------------------------------------------------------------------
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
          glVertex2f (chaida[i][j][0], chaida[i][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Green);
      for (i=0; i<1; i++)
        for (j=2; j<(stage<2?5:4); j++)
          drawImplicitLine (1, chaicirc[i][j][1], chaicirc[i][j][0]);
     }
    glPointSize(6.0);
    glLineWidth(1.0);
   }
  // --------------------------------------------------------------------------
  if (DRAWEDGE && DRAWPREV)	
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (j=0; j<prevchaida[i].getn(); j++)
        if (prevFiniteA[i][j])
	  glVertex2f (prevchaida[i][j][0], prevchaida[i][j][1]);
      glEnd();
     }
   }
  // --------------------------------------------------------------------------
  if (DRAWVERT && DRAWPREV)
   {
    glEnable(GL_LINE_STIPPLE);
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      int na = prevchaida[i].getn();
      for (j=0; j<prevchaida[i].getn(); j++)
        if (prevFiniteA[i][j] && prevFiniteA[i][(j+1)%na])
	 {
	  int jp1 = (j+1)%na;
	  glBegin(GL_LINES);
          glVertex2f (prevchaida[i][j][0], prevchaida[i][j][1]);
	  glVertex2f (prevchaida[i][jp1][0], prevchaida[i][jp1][1]);
	  glEnd();
	 }
     }
    glDisable(GL_LINE_STIPPLE);
   }
  // --------------------------------------------------------------------------
  if (DRAWEDGE)			// display control vertices
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (j=0; j<chaida[i].getn(); j++)
        if (finiteA[i][j])
          glVertex2f (chaida[i][j][0], chaida[i][j][1]);
      glEnd();
     }
   }
  // --------------------------------------------------------------------------
  if (DRAWVERT)			// display control edges
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      int na = chaida[i].getn();
      for (j=0; j<chaida[i].getn(); j++)
	if (finiteA[i][j] && finiteA[i][(j+1)%na] && !straddleHoriz(i,(j+1)%na))
     // if (finiteA[i][j] && finiteA[i][(j+1)%na])
         {
	  int jp1 = (j+1)%na;
          glBegin(GL_LINES);
          glVertex2f (chaida[i][j][0], chaida[i][j][1]);
  	  glVertex2f (chaida[i][jp1][0], chaida[i][jp1][1]);
	  glEnd();
         }
     }
   }
  // --------------------------------------------------------------------------
  if (DRAWACTIVE)		// draw active edges boldly
   {
    glLineWidth (3.0);
    glBegin(GL_LINES);
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      int na = chaida[i].getn();
      for (j=0; j<activeA[i].getn(); j++)
	//        if (activeA[i][j] == 1 && finiteA[i][j] && finiteA[i][(j+1)%na] && !straddleHoriz(i,(j+1)%na))
        if (activeA[i][j] == 1 && finiteA[i][j] && finiteA[i][(j+1)%na])
	 {
	  glVertex2f (chaida[i][j][0], chaida[i][j][1]);
	  glVertex2f (chaida[i][(j+1)%na][0], chaida[i][(j+1)%na][1]);
	 }
     }
    glEnd();
    glLineWidth (1.0);
   }  
  // --------------------------------------------------------------------------
  if (DRAWSPECIFICVTX)  //   // draw a specific dual edge: the vertex i maps to the dual edge (i-1,i)
   {
    glLineWidth (3.);
    for (i=0; i<2; i++)
      {
	glColor3fv (i==0 ? Red : Blue);
	glBegin (GL_LINES);
        int presentBackOne = mod(presentVtx[i]-1,chai[i].getn());
	glVertex2f (chaida[i][presentBackOne][0], chaida[i][presentBackOne][1]);
	glVertex2f (chaida[i][presentVtx[i]][0], chaida[i][presentVtx[i]][1]);
	glEnd();
      }
    glLineWidth(1.);
   }

  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

void displayDual2 ()
{
  int i,j;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glTranslatef (transxdual, transydual, 0);
  glScalef  (zoomdual, zoomdual, zoomdual);
  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
   
  // --------------------------------------------------------------------------
  if (DRAWBIGFIRST)
   {
    glPointSize(12.0);
    glLineWidth (3.0);
    glColor3fv (Red);
    for (i=0; i<1; i++)
      for (j=(stage<2?0:1); j<2; j++)
        drawImplicitLine (chai[i][j][0], 1, chai[i][j][1]); // (a,b,1)-> ax+y+b=0
    if (stage<2)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=0; j<1; j++)
          glVertex2f (chaidb[i][j][0], chaidb[i][j][1]);
      glEnd();
     }
    else if (stage==3)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=1; j<2; j++)
          glVertex2f (chaidb[i][j][0], chaidb[i][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Green);
      for (i=0; i<1; i++)
        for (j=(stage<2?0:1); j<3; j++)
          drawImplicitLine (chai[i][j][0], 1, chai[i][j][1]);
     }
    glPointSize(6.0);
    glLineWidth (1.0);
   }
  // --------------------------------------------------------------------------
  if (DRAWBIGSECOND)
   {
    glPointSize(12.0);
    glLineWidth (3.0);
    glColor3fv (Red);
    for (i=0; i<1; i++)
      for (j=(stage<2?1:2); j<3; j++)
        drawImplicitLine (chai[i][j][0], 1, chai[i][j][1]); // (a,b,1)-> ax+y+b=0
    if (stage<2)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (i=0; i<1; i++)
        for (j=(stage<2?1:2); j<(stage<2?2:3); j++)
          glVertex2f (chaidb[i][j][0], chaidb[i][j][1]);
      glEnd();
     }
    if (DRAWSPLIT && !DONTDRAWBLUE)
     {
      glColor3fv (Green);
      for (i=0; i<1; i++)
        for (j=2; j<(stage<2?5:4); j++)
          drawImplicitLine (chai[i][j][0], 1, chai[i][j][1]);
     }
    glPointSize(6.0);
    glLineWidth (1.0);
   }
  // --------------------------------------------------------------------------
  if (DRAWEDGE && DRAWPREV)
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (j=0; j<prevchaidb[i].getn(); j++)
        if (prevFiniteB[i][j])
	  glVertex2f (prevchaidb[i][j][0], prevchaidb[i][j][1]);
      glEnd();
     }
   }
  // --------------------------------------------------------------------------
  if (DRAWVERT && DRAWPREV)
   {
    glEnable(GL_LINE_STIPPLE);
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      int nb = prevchaidb[i].getn();
      for (j=0; j<prevchaidb[i].getn(); j++)
       {
        int jp1 = (j+1)%nb;
        if (prevFiniteB[i][j] && prevFiniteB[i][jp1])
	 {
	  glBegin(GL_LINES);
          glVertex2f (prevchaidb[i][j][0], prevchaidb[i][j][1]);
	  glVertex2f (prevchaidb[i][jp1][0], prevchaidb[i][jp1][1]);
	  glEnd();
	 }
       }
     }
    glDisable(GL_LINE_STIPPLE);
   }
  // --------------------------------------------------------------------------
  if (DRAWEDGE)			// display control vertices
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (Black);
      glBegin (GL_POINTS);
      for (j=0; j<chaidb[i].getn(); j++)
        if (finiteB[i][j])
          glVertex2f (chaidb[i][j][0], chaidb[i][j][1]);
      glEnd();
     }
   }
  // --------------------------------------------------------------------------
  if (DRAWVERT)			// display control edges
   {
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      int nb = chaidb[i].getn();
      for (j=0; j<chaidb[i].getn(); j++)
	if (finiteB[i][j] && finiteB[i][(j+1)%nb] && !straddleVert(i,(j+1)%nb))
     // if (finiteB[i][j] && finiteB[i][(j+1)%nb])
	 {
	  int jp1 = (j+1)%nb;
	  glBegin(GL_LINES);
	  glVertex2f (chaidb[i][j][0], chaidb[i][j][1]);
	  glVertex2f (chaidb[i][jp1][0], chaidb[i][jp1][1]);
	  glEnd();
	 }
     }
   }
  // --------------------------------------------------------------------------
  if (DRAWACTIVE)
   {
    glLineWidth (3.0);
    glBegin(GL_LINES);
    for (i=0; i<2; i++)
     {
      glColor3fv (i==0 ? Red : Blue);
      int nb = chaidb[i].getn();
      for (j=0; j<activeB[i].getn(); j++)
	//        if (activeB[i][j] == 1 && finiteB[i][j] && finiteB[i][(j+1)%nb] && !straddleVert(i,(j+1)%nb))
        if (activeB[i][j] == 1 && finiteB[i][j] && finiteB[i][(j+1)%nb])
	 {
	  glVertex2f (chaidb[i][j][0], chaidb[i][j][1]);
	  glVertex2f (chaidb[i][(j+1)%nb][0], chaidb[i][(j+1)%nb][1]);
	 }
     }
    glEnd();
    glLineWidth (1.0);
   }
  // --------------------------------------------------------------------------
  if (DRAWSPECIFICVTX)  //   // draw a specific dual edge: the vertex i maps to the dual edge (i-1,i)
   {
    glLineWidth (3.);
    for (i=0; i<2; i++)
      {
	glColor3fv (i==0 ? Red : Blue);
	glBegin (GL_LINES);
        int presentBackOne = mod(presentVtx[i]-1,chai[i].getn());
	glVertex2f (chaidb[i][presentBackOne][0], chaidb[i][presentBackOne][1]);
	glVertex2f (chaidb[i][presentVtx[i]][0], chaidb[i][presentVtx[i]][1]);
	glEnd();
      }
    glLineWidth(1.);
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
  string comment;  readComment(infile, comment);
  chai.allocate(2);  activeA.allocate(2); activeB.allocate(2);	// 2 Chaikin curves
  V2fArrArr Pt(2);			
  for (i=0; i<2; i++)		
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
    activeA[i].allocate(nPt); activeB[i].allocate(nPt);
    for (j=0; j<nPt; j++)	activeA[i][j] = activeB[i][j] = 1;
    for (j=0; j<nPt; j++)	chai[i][j] = Pt[i][j];
   }
  scaleToUnitSquare (chai);
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
      case 'l': LAPTOP = 1;     break;
      case 'w': WINDOWS = 1;    break;
      case 'h': 
      default:	usage(); exit(-1);				break;
      }
    else ArgsParsed++;
   }
  
  string filePrefix;
  input(argv[argc-1], filePrefix, chai);
  chaicirc.allocate(2);
  chaida.allocate(2);  chaidb.allocate(2);
  finiteA.allocate(2); finiteB.allocate(2);
  
  // compute dual Chaikin vertices
  dualize();
  prevchai   = chai;	// no predecessor at beginning
  prevchaida = chaida;	  prevchaidb = chaidb;
  prevFiniteA = finiteA;  prevFiniteB = finiteB;
  presentVtx[0] = presentVtx[1] = 0;
     
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
  strcpy (titlebar, "Chaikin bitangents");  
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
  glutAddMenuEntry ("Vertices",                      1);
  glutAddMenuEntry ("Edges",                         2);
  glutAddMenuEntry ("Previous step",                 3);
  glutAddMenuEntry ("1st nhood",                     4);
  glutAddMenuEntry ("2nd nhood",                     5);
  glutAddMenuEntry ("No blue in 1st/2nd nhood",      6);
  glutAddMenuEntry ("Bitangents",                    7);
  glutAddMenuEntry ("Draw specific vertex/edge [s]", 10);
  glutAddMenuEntry ("Next vertex on 1st object [n]", 8);
  glutAddMenuEntry ("Next vertex on 2nd object [m]", 9);
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
  glutAddMenuEntry ("Active edges", 1);
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
  glutCreateMenu (menuDual);
  glutAddMenuEntry ("Active edges", 1);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}

