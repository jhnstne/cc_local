/*
  File:          ugviewer.cpp (from 3d/surfbuild/src/surfinterpolate.cpp)
  Author:        J.K. Johnstone 
  Created:	 11 November 2004
  Last Modified: 11 November 2004
  Purpose:       Read and view a Unigrafix file,
  Input:         the UC Berkeley data format used by Carlo Sequin's group.
                 In particular, Seth Teller's Soda Hall data is in this format.
		 See the documentation at his webpage under Implementations and Data.
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
#include <tcl.h>    // for hashing


#include "AllColor.h"
#include "Vector.h"		// V3fArr
#include "MiscVector.h"		// read, scaleToUnitCube

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image

static char     *RoutineName;
static GLfloat   transxob, transyob, rotxob, rotyob, rotzob, zoomob;
static int 	 panLeft=0, panRight=1; // control panning for 3d effect
static GLboolean leftMouseDown=0;
static GLboolean middleMouseDown=0;
static GLboolean firstx=1,firsty=1;	// first MOUSEX (MOUSEY) reading?
static int	 oldx,oldy;		// previous value of MOUSEX and MOUSEY
static GLboolean ROTATEOB=0;		// rotate obstacles?
static GLboolean PANOB=0; 		// rotate object back and forth for 3d effect?
static GLboolean DRAWVERT=1;		// draw data points?
static GLboolean DRAWFACE=1;		// draw faces?
static GLboolean WIRE=1;                // draw faces in wireframe mode?
static GLboolean DRAWLIGHT=0;		// draw position of light?

V3fArr color;           // available colors
V3fArr vert;            // available vertices
V4iArr face;            // vertex indices of each face (assuming quadrilaterals)
IntArr faceColor;       // color indices of each face
int    obstacleWin;	// window identifier 

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
  glPointSize (2.0);
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
  case '1': 	DRAWVERT = !DRAWVERT;		break;
  case '2':     DRAWFACE = !DRAWFACE;		break;
  case 'r':     ROTATEOB = !ROTATEOB;			// rotate
 	     	if (ROTATEOB) 
		     glutIdleFunc (RotateOb); 
		else glutIdleFunc (NULL); 	break;
  case 'p':	PANOB = !PANOB;				// pan
		if (PANOB) 
		     glutIdleFunc (PanOb); 
		else glutIdleFunc (NULL); 	break;
  case 'l':	DRAWLIGHT = !DRAWLIGHT;		break;
  case 'w':     WIRE = !WIRE;			break;	// wireframe
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
	 glBegin(GL_QUADS);
	 for (i=0; i<face.getn(); i++)
	   for (j=0; j<4; j++)
	     glVertex3f (vert[face[i][j]][0], vert[face[i][j]][1], vert[face[i][j]][2]);
	 glEnd();
	 glEnable (GL_LIGHTING);	 
       }
     else
       {
	 glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
	 float ambient[4]  = {0,0,0,1};
	 float diffuse[4]; diffuse[3] = 1; // opaque
	 float specular[4] = {.7,.7,.7,1};
	 float shininess   = .25;
	 glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
	 glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	 glMaterialf (GL_FRONT, GL_SHININESS, shininess * 128.0);
	 glBegin(GL_QUADS);
	 for (i=0; i<face.getn(); i++)
	   {
	     for (j=0; j<3; j++) diffuse[j] = color[faceColor[i]][j];
	     glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	     V3f norm;
	     computeTriNormal(vert[face[i][0]], vert[face[i][1]], vert[face[i][2]], norm);
	     glNormal3f (norm[0], norm[1], norm[2]);
	     for (j=0; j<4; j++)
	       glVertex3f (vert[face[i][j]][0], vert[face[i][j]][1], vert[face[i][j]][2]);
	   }
	 glEnd();
       }
   } 
  glPopMatrix();
  glutSwapBuffers ();
  glutPostRedisplay();	// to keep animation running in both windows
}

/******************************************************************************
******************************************************************************/

static void usage()
 {
  cout << "Usage is " << RoutineName << endl;
  cout << "\t[-o xrot yrot zrot] (initial orientation)" << endl;
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
  //  read it, just to count and allocate
  int nColor=0, nVert=0, nFace=0;
  infile.open(argv[argc-1]);
  while (tryToGetLeftBrace(infile))
    {
      skipToMatchingRightBrace(infile);
    }
  int mark = infile.tellg();
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
  int done = 0;
  while (keyword == "f" && !done) // face line: f id (v1 v2 ...) (w1 w2 ...) ... colour ;
    {
      infile >> faceID;
      getLeftParen(infile);
      int nvPerFace=0;
      while (!tryToGetRightParen(infile)) { infile >> nextvertexid; nvPerFace++; }
      if (nvPerFace != 4) {cout << "Nonquadrilateral face: adapt code" << endl; exit(-1);}
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

  // read it again, for real
  infile.clear();
  infile.seekg(mark);
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
      int nvPerFace=0;
      for (i=0; i<4; i++)
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

  scaleToUnitCube (vert);
     
  /************************************************************/

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  int titleht = 20; 	// top titlebar is 20 units high
  int xleft = 0;	// x-coord of lefthand side for large windows
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
  glutAddMenuEntry ("Data points [1]",  		      1);
  glutAddMenuEntry ("Shaded/wireframe [w]",		      0);
  glutAddMenuEntry ("Light position [l]",		      8);
  glutAttachMenu (GLUT_RIGHT_BUTTON);

  glutMainLoop();
  return 0;
}
