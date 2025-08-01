/*
  File:          surf_view.cpp (earlier, surfbuild.cpp)
  Author:        J.K. Johnstone 
  Created:	 31 May 2006, from surfinterpolate.cpp 3/14/03 version;
                 extends to IRIT data
  Last Modified: 2 January 2013
  Purpose:       Construction of smooth Bezier models, from
                 (a) cubic Bezier meshes, 
		 (b) Bspline meshes in IRIT format, or 
		 (c) data points in tensor product format.
  Sequence:	 1st in a sequence (surfbuild, tangSurf, bisilh)
  Input:         Bezier mesh in cpt3 format,
                 Bspline mesh in IRIT format, or
		 rectangular mesh of data points.
*/ 

//       1         2         3         4         5         6         7         8
// 45678901234567890123456789012345678901234567890123456789012345678901234567890

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif
#include <assert.h>
#include <iostream>
#include <fstream>
using namespace std;
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
using std::string;
#include <time.h>

#include "basic/AllColor.h"
#include "basic/Vector.h"		
#include "surf/BsplineSurf.h"                        // ReadIRIT, CountIRITmodel
#include "surf/BezierSurf.h"		            // fit, prepareDisplay, draw
#include "shape/Reader.h"              // ScaleToUnitCube, ReadPtCloudBracedRect

static char *RoutineName;
static void usage()
 {
  cout << "View a Bezier or B-spline surface." << endl << endl
       << "Input: <file>.cpt3 (tensor product Bezier control mesh) or" << endl
       << "       <file>.pts3 (rectangular array of data points) or" << endl
       << "       <file>.itd (tensor product B-spline in IRIT format)" << endl
       << "  -d #: display density of Bezier segment; default 10" << endl
       << "  -m: direct bicubic Bezier control mesh input" << endl
       << "  -M: unified bicubic Bezier control mesh input" << endl
       << "  -o xrot yrot zrot: initial orientation" << endl;
 }

V3fArrArrArr	    Pt;		         // data points, organized into surfaces
Array<BezSurf3f> model;	                                        // Bezier models
Array<BsplineSurf3f> bsplmodel;                               // B-spline models
float uActive,vActive;                             // parameters of active point
float uDelta,vDelta;	                // increment of parameter value per step
float uFirstKnot,uLastKnot,vFirstKnot, vLastKnot;

static GLfloat   transx, transy, transz, rotx, roty, rotz, zoom;
static int 	 panLeft=0, panRight=1;
static GLfloat   rotIncrement=.02;
static GLfloat   rotBigIncrement=.2;
static int       panAmt=5000;
static GLboolean leftMouseDown=0, middleMouseDown=0;
static GLboolean translationMode = 0;
static GLboolean firstx=1;	
static GLboolean firsty=1;      
static int	 oldx,oldy;	
static GLboolean ROTATE=0;	
static GLboolean FASTSPEED=0;	
static GLboolean PAN=0;	 	

static GLboolean DRAWPT=0;		                    // draw data points?
static GLboolean DRAWSURF=1;		                       // draw surfaces?
static GLboolean DRAWMESH=0;		                   // draw control mesh?
static GLboolean WIRE=0;		          // draw surfaces in wireframe?
static GLboolean CTRLMESH=0;		 // input data is control mesh directly?
static GLboolean UNIFIEDCTRLMESH=0;	  // input data is unified control mesh?
                                          // (not just separate bicubic patches)
static GLboolean DRAWACTIVEPT=0;	                   // draw active point?
static GLboolean DRAWLIGHT=0;		              // draw position of light?

/******************************************************************************/
/******************************************************************************/

void gfxinit()
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
  transx = transy = 0.0;
  rotx = -90.0; roty = 0; rotz = 0;
  zoom = 1.8;
}

/******************************************************************************/
/******************************************************************************/

void reshape (int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (w <= h)
       glOrtho(-2.0, 2.0, -2.0*(GLfloat)h/(GLfloat)w, 2.0*(GLfloat)h/(GLfloat)w,
	       -600, 600);
  else glOrtho(-2.0*(GLfloat)w/(GLfloat)h, 2.0*(GLfloat)w/(GLfloat)h, -2.0,2.0, 
	       -600, 600);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/******************************************************************************/
/******************************************************************************/

void Rotate ()
{
  rotz += rotIncrement; 
  if (rotz > 360.0) rotz -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void RotateFast ()
{
  rotz += rotBigIncrement; 
  if (rotz > 360.0) rotz -= 360.0;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void Pan ()
{
  if (panLeft)
   {
    rotz += rotIncrement/5;
    if (rotz > 360.0) rotz -= 360.0;
    panLeft++;
    if (panLeft==panAmt) { panLeft=0; panRight=1; }
   }
  else if (panRight)
   {
    rotz -= rotIncrement/5;
    if (rotz < 0.0) rotz += 360.0;
    panRight++;
    if (panRight==panAmt) { panRight=0; panLeft=1; }
   }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void visibility (int status)
{
  if (status != GLUT_VISIBLE) 
    {
      if (ROTATE || PAN)  glutIdleFunc (NULL);
    }
  else if (ROTATE)      glutIdleFunc (Rotate);
  else if (PAN)         glutIdleFunc (Pan);
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
  if (middleMouseDown)   
   {
    if (firstx)  firstx=0; else zoom -= (float).02*(x-oldx);
    if (zoom < 0.0) zoom = 0.0;
   }
  else if (leftMouseDown && !translationMode) 
   {
    if (firstx)  firstx=0;
    else { roty += .5*(x-oldx); if (roty > 360.0) roty -= 360.0; }     // ORI: Y

    if (firsty)  firsty=0;
    else { rotx += .5*(y-oldy); if (rotx > 360.0) rotx -= 360.0; }     // ORI: X
   }
  else if (leftMouseDown && translationMode)
   {
    if (firstx)  firstx=0; else transx += .001*(x-oldx);       // TRANSLATION: X
    if (firsty)  firsty=0; else transy += .001*(y-oldy);       // TRANSLATION: Y
   }
  oldx = x;  oldy = y;
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case '1': 	DRAWPT = !DRAWPT;		break;
  case '2':     DRAWSURF = !DRAWSURF;		break;
  case 'm':  	DRAWMESH = !DRAWMESH;		break;

  case 'w':     WIRE = !WIRE;			break;	
  case 'a':  	DRAWACTIVEPT = !DRAWACTIVEPT;	break;	
  case 'h':	uActive -= uDelta;			            // move left
    		if (uActive < uFirstKnot)
		  uActive = uLastKnot;		break;
  case 'j':	uActive += uDelta;			           // move right
    		if (uActive > uLastKnot) 
		  uActive = uFirstKnot;		break;
  case 'u':	vActive += vDelta;			              // move up
    		if (vActive > vLastKnot) 
		  vActive = vFirstKnot;		break;
  case 'n':	vActive -= vDelta;			            // move down
    		if (vActive < vFirstKnot) 
		  vActive = vLastKnot;		break;
  case 'L':	DRAWLIGHT = !DRAWLIGHT;		break;

  // rigid transform
  case 'l': 	rotx=180; roty=rotz=0.0;        break;	            // look down
  case 'r':	ROTATE = !ROTATE;			      // toggle rotation
                if (ROTATE) 
		  if (FASTSPEED) 
		    glutIdleFunc (RotateFast);
		  else glutIdleFunc (Rotate);
		else glutIdleFunc (NULL); 	break;
  case 'R':     rotx=273; roty=.5; rotz=350;	break;             // reset view
  case 's':	FASTSPEED = !FASTSPEED;                 // adjust rotation speed
                if (ROTATE) 
		  if (FASTSPEED) 
		    glutIdleFunc (RotateFast);
		  else glutIdleFunc (Rotate);   break;
  case 'U':     rotx+=180;			break;       // turn upside down
  case '[':     translationMode = 0;            break;          // rotation mode
  case ']':     translationMode = 1;            break;       // translation mode
  case 27:	exit(1);                        break;	               // ESCAPE
  default:      				break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void menu (int value)
{
  switch (value) {
  case 0:  WIRE 	= !WIRE;		break;
  case 1:  DRAWPT 	= !DRAWPT;		break;
  case 2:  DRAWSURF 	= !DRAWSURF;		break;
  case 3:  DRAWACTIVEPT = !DRAWACTIVEPT;	break;
  case 4:  uActive -= uDelta;			                    // move left
           if (uActive < uFirstKnot)
	     uActive = uLastKnot;		break;
  case 5:  uActive += uDelta;			                   // move right
           if (uActive > uLastKnot) 
	     uActive = uFirstKnot;		break;
  case 6:  vActive += vDelta;	                                      // move up
           if (vActive > vLastKnot) 
	     vActive = vFirstKnot;		break;
  case 7:  vActive -= vDelta;		                            // move down
           if (vActive < vFirstKnot) 
	     vActive = vLastKnot;		break;
  case 8:  DRAWLIGHT = !DRAWLIGHT;		break;	  
  case 9:  DRAWMESH = !DRAWMESH;		break;
  default:   					break;
  }
  glutPostRedisplay();
}

/******************************************************************************/
/******************************************************************************/

void display ()
{
  int i,j,k;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  glScalef  (zoom, zoom, zoom);
  glTranslatef (transx, transy, 0);
  glRotatef (rotx, 1.0, 0.0, 0.0);
  glRotatef (roty, 0.0, 1.0, 0.0);
  glRotatef (rotz, 0.0, 0.0, 1.0);

  if (DRAWLIGHT)
   { glColor3fv (Black); glBegin(GL_POINTS); glVertex3f (0,3,3); glEnd(); }
  if (DRAWPT)
   {
    glColor3fv (Black);
    glDisable (GL_LIGHTING);
    glBegin(GL_POINTS);
    for (i=0; i<Pt.getn(); i++)
      for (j=0; j<Pt[i].getn(); j++)
        for (k=0; k<Pt[i][j].getn(); k++)
	  glVertex3f (Pt[i][j][k][0], Pt[i][j][k][1], Pt[i][j][k][2]);
    glEnd();
    glEnable (GL_LIGHTING);
   }
  if (DRAWSURF)
    if (WIRE)
     {
      glDisable (GL_LIGHTING);
      for (i=0;i<model.getn();i++) 
	{ glColor3fv(material[i%24]+4); model[i].draw(1); }
      glEnable (GL_LIGHTING);
     }
    else
      for (i=0; i<model.getn(); i++)	
       {
        glMaterialfv(GL_FRONT, GL_AMBIENT,  material[i%24]);
        glMaterialfv(GL_FRONT, GL_DIFFUSE,  material[i%24]+4);
        glMaterialfv(GL_FRONT, GL_SPECULAR, material[i%24]+8);
        glMaterialf(GL_FRONT, GL_SHININESS, material[i%24][12] * 128.0);
        model[i].draw();
       }
  if (DRAWMESH)
   {
    glColor3fv (Red);
    glDisable (GL_LIGHTING);
    for (i=0; i<model.getn(); i++) model[i].drawCtrlNet();
    glEnable (GL_LIGHTING);
   }
  if (DRAWACTIVEPT)
   {
    glDisable (GL_LIGHTING);
    glColor3fv (Red);                        // glColor3fv (Black); for printout
    model[0].drawPt (uActive, vActive);
    glEnable (GL_LIGHTING);
   }
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();
}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{ 
  int i;
  int ArgsParsed=0;
  ifstream infile;
  int density = 10;                              // # of pts to draw per segment

  RoutineName = argv[ArgsParsed++];
  if (argc == 1) { usage(); exit(-1); }
  while (ArgsParsed < argc) 
   {
    if ('-' == argv[ArgsParsed][0])
      switch (argv[ArgsParsed++][1])
      { 
      case 'd': density = atoi(argv[ArgsParsed++]);	break;
      case 'm': CTRLMESH = 1;				break;
      case 'M': UNIFIEDCTRLMESH = 1;			break;
      case 'o': rotx = atof(argv[ArgsParsed++]);
      		roty = atof(argv[ArgsParsed++]);
      		rotz = atof(argv[ArgsParsed++]);	break;
      case 'h': 
      default:	usage(); exit(-1);			break;
      }
   else ArgsParsed++;
  }
  
  /***************************************************************************/

  infile.open(argv[argc-1]);
  string suffix;  getSuffix (argv[argc-1], suffix);
  if (suffix == "itd")                                    // Elber's IRIT format
   {
    /*
   long term strategy to handle multiple surf types (not just poly Bspline surf)
     [given a single object, possibly consisting of many subobjects]
     count the objects using CountIRIT
     read past the opening object attributes
       for each model
		  read past the opening object attributes
		  ProbeIRIT to find its type
		  then call appropriate reader
     */
    int nModel = ReadIRIT (infile, bsplmodel); 
    model.allocate(nModel);
    infile.close();
    for (i=0; i<nModel; i++)
     {
                    // cout << "Bspl model is " << endl << bsplmodel[i] << endl;
      model[i].translate (bsplmodel[i]);
                         // cout<<"Bezier model "<<i<<":"<<endl<<model[i]<<endl;
     }
    } 
  else
   {
    string c; ReadComment (infile,'[',']',c);
    ReadPtCloudBracedRect (infile, Pt);               // Bezier mesh or data pts
    ScaleToUnitCube(Pt);
    IntArr numSegu(Pt.getn()), numSegv(Pt.getn());
    if (UNIFIEDCTRLMESH) 
      for (i=0; i<Pt.getn(); i++) infile >> numSegu[i] >> numSegv[i];
    infile.close();
    model.allocate(Pt.getn());
    for (i=0; i<Pt.getn(); i++)
     {
      if (CTRLMESH)	                        // each Pt[i] is a bicubic patch
       {
	FloatArr knot(2);  knot[0] = 0; knot[1] = 1;
	model[i].create (3,3,3,1,1,Pt[i],knot,knot);   // define 1 bicubic patch
       }
      else if (UNIFIEDCTRLMESH) // each Pt[i] is a collection of bicubic patches
       {
	FloatArr knotu(numSegu[i]+1), knotv(numSegv[i]+1);
	int j;
	for (j=0; j<=numSegu[i]; j++) knotu[j] = j;
	for (j=0; j<=numSegv[i]; j++) knotv[j] = j;
	model[i].create (3,3,3,numSegu[i],numSegv[i],Pt[i],knotu,knotv);
       }
      else model[i].fit (Pt[i]);   // fit by interpolating cubic Bezier surfaces
                         // cout<<"Bezier model "<<i<<":"<<endl<<model[i]<<endl;
     }
   }
  for (i=0; i<model.getn(); i++)  
    model[i].prepareDisplay (density); 
  uFirstKnot = model[0].knotu[0];	
  vFirstKnot = model[0].knotv[0];
  uLastKnot  = model[0].knotu[model[0].getnKnotu()-1];
  vLastKnot  = model[0].knotv[model[0].getnKnotv()-1];
  uActive    = (uLastKnot + uFirstKnot) / 2.;	              // start in middle
  vActive    = (vLastKnot + vFirstKnot) / 2.;	
  uDelta = (uLastKnot - uFirstKnot) / 800.;
  vDelta = (vLastKnot - vFirstKnot) / 800.;
  
  /***************************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize (555,555);	
  glutInitWindowPosition (518,0);
  char titlebar[100]; 
  strcpy (titlebar, "Surface viewer: ");
  strcat (titlebar, argv[argc-1]);  glutCreateWindow (titlebar);
  gfxinit();
  glutReshapeFunc (reshape);
  glutVisibilityFunc (visibility);
  glutMouseFunc (mouse);
  glutMotionFunc (motion);
  glutKeyboardFunc (keyboard);

  glutCreateMenu (menu);
  glutAddMenuEntry ("[1] data points",  		      1);
  glutAddMenuEntry ("[2] interpolating cubic Bezier surface", 2);
  glutAddMenuEntry ("[m] control mesh", 		      9);
  glutAddMenuEntry ("[w] shaded/wireframe",		      0);
  glutAddMenuEntry ("[a] active point",      		      3);
  glutAddMenuEntry ("[h] move active point left",	      4);
  glutAddMenuEntry ("[j] move active point right",	      5);
  glutAddMenuEntry ("[u] move active point up",	      	      6);
  glutAddMenuEntry ("[n] move active point down",	      7);
  glutAddMenuEntry ("[L] light position",		      8);
  glutAddMenuEntry ("",                                   100);
  glutAddMenuEntry ("rotation mode: [ then middle mouse", 100);
  glutAddMenuEntry ("translation mode: ], middle mouse",  100);
  glutAddMenuEntry ("[r] rotate", 100);
  glutAddMenuEntry ("[s] change rotation speed", 100);
  glutAddMenuEntry ("[l] look down", 100);
  glutAddMenuEntry ("[U] upend 180 about x-axis", 100);
  glutAddMenuEntry ("[R] reset", 100);
  glutAttachMenu (GLUT_RIGHT_BUTTON);
  glutDisplayFunc (display);
  glutMainLoop();
  return 0;
}
