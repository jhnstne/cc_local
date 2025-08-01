/*
  File:          wrlviewer.cpp (from ugviewer.cpp)
  Author:        J.K. Johnstone 
  Created:	 30 November 2004
  Last Modified: 2 December  2004
  Purpose:       Read and view a wrl file, the VRML/Inventor style.
  History:

  keyword = 
    DEF name node: named node (treat same as unnamed node, for now) (p. 297, Inv Mentor)
    Separator: increment level
    Info: skip
    Spotlight: increment nSpotLight (p. 91): 
        on(1)/intensity(1)/color(3)/location(3)/direction(3)/dropoffrate(1)/cutoffangle(1)
	13 values
    PointLight: increment nPtLight,  
        on/intensity/color/location
	8 values
    ShapeHints: record as flags (for this active level): 4 options (p. 122 InvMentor)
    Coordinate3: has one field 'point', count points inside []
    Normal: has one field 'vector', direct normal vectors, count normals inside []
    Material: 6 potential fields (p. 117 InvMentor): 
         ambientColor, diffuseColor, specularColor, emissiveColor, shininess, transparency
	 material = 14 values
    IndexedFaceSet: four potential fields (p. 105):
         coordIndex, materialIndex, normalIndex, textureCoordIndex
      coordIndex: faces are specified by index into Coordinate3 list, each ended by -1
      normalIndex: normals per vertex of a face, specified by index into Normal list, 
         each ended by -1 (should match sizes of face as specified by coordIndex)
    Translation: store in separator environment, like material
    Rotation: store in separator environment
    Scale: store in separator environment
    MaterialBinding: how to bind materials to shapes, several options e.g. PER_VERTEX (p. 127)
       OVERALL: current material for whole shape
       PER_FACE: cycle through materials, one per face
       PER_FACE_INDEXED: use material index to choose material
       ...
    NormalBinding: similar to material binding
    MatrixTransform: apparently not defined in Inventor Mentor
    anything else: error statement (lazy evaluation of other commands)

Reading an IV (or WRL) file:
look for Coordinate3 (each one represents part of the object)
	read Coordinate3
	read {
	read point
	read [
read points (separated by commas), until ]
read normals [optional]
	read DEF <name> Material
read ambientColor
read diffuseColor
read specularColor
read shininess
	read transparency
	read to coordIndex (past Separator, IndexedFaceSet)
read face vertices (until -1)
repeat
first time through just to count
see software/Cbin/data/translator/iv2scat.c++ for a beginning (just reading first point set though, without counting)

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
#include "Miscellany.h"         // readOpeningComment
#include "MiscVector.h"		// read, scaleToUnitCube

#define PTSPERBEZSEGMENT 10     // # pts to draw on each Bezier segment
#define WINDOWS 0		// running on Windows?
#define PRINTOUT 0		// 0 for displaying on screen, 1 for printing out image
#define INDEXEDFACESET 0        // one of the face modes

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

int nMaterial, nVert, nFace, nNorm, nSpotLight, nPtLight;
V3fArr vert;            // available vertices
IntArrArr face;         // face[i] = vertex indices of ith face
int begFaceIndex;       // index of first face in the present object
V3fArr norm;            // vertex normals
int normalIndex=0;      // are normals indexed? (at present assuming that this is 
                        // universal decision; eventually want to keep track of this 
                        // for each separator)
IntArrArr normPerFace;  // if normals are indexed, each vertex in a face 
                        // has a different normal, which is recorded here:
                        // normPerFace[i][j] = normal index of jth vertex of ith face
FloatVecArr mater;      // available materials, each material is 14 values 
                        // 3 ambient, 3 diff, 3 spec, 3 emissive, 1 shiny, 1 transparency
IntArr faceMaterial;    // material indices of each face, presently assigning the 
                        // active material (which is usually right)
FloatVecArr lightSpot;  // each spotlight is 13 values
     // on(1)/intensity(1)/color(3)/location(3)/direction(3)/dropoffrate(1)/cutoffangle(1)
FloatVecArr lightPt;    // each point light is 8 values
                        // on(1)/intensity(1)/color(3)/location(3)
string def("DEF"),
    separator("Separator"),
    info("Info"),
    spot("SpotLight"),
    ptlight("PointLight"),
    shapehints("ShapeHints"),
    coord3("Coordinate3"),
    normal("Normal"),
    point("point"),
    vec("vector"),
    mat("Material"),
    Ambient("AmbientColor"),  // two spellings: see cg.wrl
    ambient("ambientColor"),
    Diffuse("DiffuseColor"),  
    diffuse("diffuseColor"),
    Specular("SpecularColor"),
    specular("specularColor"),
    Emissive("EmissiveColor"),
    emissive("emissiveColor"),
    Shininess("Shininess"),
    shininess("shininess"),
    Transparency("Transparency"), 
    transparency("transparency"),
    indexedfaceset("IndexedFaceSet"),
    coordindex("coordIndex"),
    normalindex("normalIndex"),
    materialindex("materialIndex"),
    textureindex("textureCoordIndex"),
    translation("Translation"),
    rotation("Rotation"),
    scale("Scale"),
    materialbinding("MaterialBinding"),
    normalbinding("NormalBinding"),
    matrixtransform("MatrixTransform"),
    texcoord2("TextureCoordinate2");
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
  float ambient[4]; ambient[3] = 1;
  float diffuse[4]; diffuse[3] = 1;
  float specular[4]; specular[3] = 1;
  float emission[4]; emission[4] = 1;
  float shininess, transparency;

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
  // draw coordinate axes: red x, green y, blue z
  /*
  glDisable (GL_LIGHTING);
  glColor3fv (Red);
  glBegin(GL_LINES);
  glVertex3f (0,0,0);
  glVertex3f (1,0,0);
  glEnd();
  glColor3fv (Green);
  glBegin(GL_LINES);
  glVertex3f (0,0,0);
  glVertex3f (0,1,0);
  glEnd();
  glColor3fv (Blue);
  glBegin(GL_LINES);
  glVertex3f (0,0,0);
  glVertex3f (0,0,1);
  glEnd(); 
  */
  glEnable (GL_LIGHTING);
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
	 for (i=0; i<face.getn(); i++)
	   {
	     glBegin(GL_POLYGON);
	     for (j=0; j<face[i].getn(); j++)
	       glVertex3f (vert[face[i][j]][0], vert[face[i][j]][1], vert[face[i][j]][2]);
	     glEnd();
	   }
	 glEnable (GL_LIGHTING);	 
       }
     else
       {
	 glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
	 for (i=0; i<face.getn(); i++)
	   {
	     //	     cout << "Drawing face " << i << endl;
	     for (j=0; j<3; j++) 
	       {
		 ambient[j] = mater[faceMaterial[i]][j];
		 diffuse[j] = mater[faceMaterial[i]][j+3];
		 specular[j]= mater[faceMaterial[i]][j+6];
		 emission[j]= mater[faceMaterial[i]][j+9];
	       }
	     shininess   = mater[faceMaterial[i]][12];
	     transparency= mater[faceMaterial[i]][13];
	     ambient[3]  = 1-transparency;
	     diffuse[3]  = 1-transparency;
	     specular[3] = 1-transparency;
	     emission[3] = 1-transparency;
	     glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
	     glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	     glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	     glMaterialfv(GL_FRONT, GL_EMISSION, emission);
	     glMaterialf (GL_FRONT, GL_SHININESS, shininess * 128.0);
	     //	     cout << "past material" << endl;
	     // V3f norm;
	     // computeTriNormal(vert[face[i][0]], vert[face[i][1]], vert[face[i][2]], norm);
	     glBegin(GL_POLYGON);
	     //	     glNormal3f (norm[0], norm[1], norm[2]);
	     V3f zerovec(0,0,0);
	     for (j=0; j<face[i].getn(); j++)
	      {
	       if (normalIndex) glNormal3f (norm[normPerFace[i][j]][0],
					    norm[normPerFace[i][j]][1],
					    norm[normPerFace[i][j]][2]);
	       else if (norm[face[i][j]] == zerovec) // some normals are left unspecified
		 {
		   V3f compnorm;
		   computeTriNormal (vert[face[i][0]], 
				     vert[face[i][1]], 
				     vert[face[i][2]], compnorm);
		   glNormal3f (compnorm[0], compnorm[1], compnorm[2]);
		 }
	       else glNormal3f (norm[face[i][j]][0],
				norm[face[i][j]][1],
				norm[face[i][j]][2]);
	       //	       cout << "Normal fine for " << j << endl;
	       glVertex3f (vert[face[i][j]][0], vert[face[i][j]][1], vert[face[i][j]][2]);
	      }
	     glEnd();
	   }
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

void allocateWRL (ifstream &infile)
{
  int i;
  int facemode;
  int level=0; // separator level
  int insideFace=0; //are we inside a face (so matching paren should eventually be found)?

  cout << "Reading comments" << endl; 
  readOpeningComments (infile);   // assume all comments are at outset
  int mark = infile.tellg();
  nMaterial = nVert = nFace = nNorm = nSpotLight = nPtLight = 0;
  string key;  infile >> key;
  while (!infile.eof())
    {
      cout << "Read " << key << endl;
      if (key == def)
	{
	  infile >> key;  // name of node/path/engine, presently discarded
	  cout << key << endl;
	  infile >> key; // next keyword
	  cout << "Read " << key << endl;
	}
      if (key != coordindex && key != normalindex && key != materialindex) 
	getLeftBrace(infile);
      cout << "Recognized as ";
      if (key == separator) 
	{ 
	  cout << "separator" << endl;
	  level++;
	  cout << "Pushing to level " << level << endl;
	}
      else if (key == info) 
	{
	  cout << "info" << endl;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == spot) 
	{
	  cout << "spot" << endl; 
	  cout << "Ignoring spotlight information for now" << endl;
	  nSpotLight++;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == ptlight) 
	{
	  cout << "ptlight" << endl; 
	  cout << "Ignoring point light information for now" << endl;
	  nPtLight++;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == shapehints) 
	{
	  cout << "shapehints" << endl; 
	  cout << "Ignoring shape hints for now" << endl;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == coord3) 
	{
	  cout << "coord3" << endl; 
	  cout << "Have read " << nVert << " points so far" << endl;
	  infile >> key;  assert (key == point);
	  getSymbol(infile,'[');
	  float foo;
	  infile >> foo >> foo >> foo; nVert++;
	  int done=0;
	  while (!done && !tryToGetSymbol(infile,']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']')) // last triplet may have a comma too
		{ infile >> foo >> foo >> foo; nVert++; }
	      else done = 1;
	    }
	  getRightBrace(infile);
	  cout << nVert << " vertices so far" << endl;
	}
      else if (key == normal)  // triplets ended by commas
	{                      // last triplet may or may not have a comma
	  cout << "normal" << endl; 
	  infile >> key;  assert (key == vec);
	  getSymbol(infile,'[');
	  float foo;
	  infile >> foo >> foo >> foo; nNorm++;
	  int done=0;
	  while (!done && !tryToGetSymbol(infile,']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']')) // perversely, last triplet may have a comma too
		{ infile >> foo >> foo >> foo; nNorm++; }
	      else done = 1;
	    }
	  getRightBrace(infile);
	  cout << nNorm << " normals so far" << endl;
	}
      else if (key == mat) 
	{
	  cout << "material" << endl; 
	  nMaterial++;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == indexedfaceset) 
	{
	  cout << "indexedfaceset" << endl; 
	  facemode = INDEXEDFACESET;
	  insideFace = 1;
	}
      else if (key == coordindex) 
	{
	  cout << "coordindex" << endl; 
	  getSymbol (infile, '[');
	  int foo; infile >> foo;
	  while (foo != -1)
	    {
	      getSymbol(infile,',');
	      infile >> foo; 
	    }
	  nFace++;
	  int done = 0;
	  while (!done && !tryToGetSymbol(infile,']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']')) // last -1 may have trailing comma
		{
		  infile >> foo;
		  while (foo != -1)
		    {
		      getSymbol (infile,',');
		      infile >> foo;
		    }
		  nFace++;
		}
	      else done = 1;
	    }
	  cout << nFace << " faces so far" << endl;
	}
      else if (key == normalindex) 
	{
	  cout << "normalindex" << endl;
	  normalIndex = 1;
	  nFace = begFaceIndex;
	  getSymbol (infile, '[');
	  int foo; infile >> foo;
	  while (foo != -1)
	    {
	      getSymbol(infile,',');
	      infile >> foo; 
	    }
	  nFace++;
	  int done = 0;
	  while (!done && !tryToGetSymbol(infile,']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']')) // last -1 may have trailing comma
		{
		  infile >> foo;
		  while (foo != -1)
		    {
		      getSymbol (infile,',');
		      infile >> foo;
		    }
		  nFace++;
		}
	      else done = 1;
	    }
	  cout << nFace << " normal faces so far" << endl;
	  //	  skipToMatchingRightSquare (infile);
	}
      else if (key == materialindex) 
	{
	  cout << "material index" << endl;
	  nFace = begFaceIndex;
	  getSymbol (infile, '[');
	  int foo; infile >> foo;
	  nFace++;
	  int done = 0;
	  while (!done && !tryToGetSymbol(infile,']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']')) // possible trailing comma
		{
		  infile >> foo;
		  nFace++;
		}
	      else done = 1;
	    }
	}
      else if (key == textureindex) cout << "texture index" << endl;
      else if (key == texcoord2)
	{
	  cout << "Ignoring TextureCoordinate2 for now" << endl;
	  skipToMatchingRightBrace(infile);
	}
      else if (key == translation) 
	{
	  cout << "Ignoring translation for now" << endl;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == rotation) 
	{
	  cout << "Ignoring rotation for now" << endl;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == scale)
	{
	  cout << "Ignoring scale for now" << endl;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == materialbinding) 
	{
	  cout << "Ignoring material binding for now" << endl;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == normalbinding)
	{
	  cout << "Ignoring normal binding for now" << endl;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == matrixtransform)
	{
	  cout << "Ignoring matrix transform for now" << endl;
	  skipToMatchingRightBrace (infile);
	}
      else cout << "something else: " << key << endl;
      while (!infile.eof() && tryToGetRightBrace(infile)) 
	{
	  if (insideFace) 
	    {
	      cout << "Matching face bracket" << endl;
	      insideFace=0; // found matching brace to INDEXEDFACESET or the like
	    }
	  else 
	    {
	      level--;  // found the matching brace to Separator
	      cout << "Popping to level " << level << endl;
	      //	      if (level < 0) exit(-1);
	    }
	}
      if (!infile.eof()) infile >> key;
    }

  cout << nVert << " points" << endl;
  cout << nNorm << " normals" << endl;
  cout << nFace << " faces" << endl;
  cout << nMaterial << " materials" << endl;
  cout << nSpotLight << " spotlights" << endl;
  cout << nPtLight << " point lights" << endl;
  vert.allocate (nVert);
  norm.allocate (nNorm);
  if (normalIndex) 
    normPerFace.allocate (nFace);
  face.allocate (nFace);
  faceMaterial.allocate (nFace);
  norm.allocate (nNorm);

  mater.allocate (nMaterial); 
  for (i=0; i<nMaterial; i++) 
    {
      mater[i].allocate (14);             
      mater[i][0] = mater[i][1] = mater[i][2] = .2;  // ambient default values (p. 117)
      mater[i][3] = mater[i][4] = mater[i][5] = .8;  // diffuse 
      mater[i][6] = mater[i][7] = mater[i][8] = .0;  // specular
      mater[i][9] = mater[i][10]= mater[i][11]= .0;  // emissive
      mater[i][12]= .2;                              // shininess
      mater[i][13]= .0;                              // transparency
    }
  if (nSpotLight > 0)
    {
      lightSpot.allocate (nSpotLight); 
      for (i=0; i<nSpotLight; i++) lightSpot[i].allocate(13);
    }
  if (nPtLight > 0)
    {
      lightPt.allocate (nPtLight); 
      for (i=0; i<nPtLight; i++)   lightPt[i].allocate(8);
    }
  infile.clear();
  infile.seekg(mark);
}

/******************************************************************************
******************************************************************************/

int countVertPerFace (ifstream &infile)
{
  int mark = infile.tellg();
  int nv;  string comma;
  int foo; infile >> foo; // count vertices
  nv = 0;
  while (foo != -1)
    {
      nv++;
      getSymbol(infile,',');
      infile >> foo; 
    }
  infile.seekg(mark);
  return nv;
}

/******************************************************************************
******************************************************************************/

void readWRL (ifstream &infile)
{
  int i;
  int facemode;
  int level=0; // separator level
  int insideFace=0; //are we inside a face (so matching paren should eventually be found)?
  int insideMaterial=0; // are we inside a material?
  int begVertIndex; // index of first vertex in the present object; not the most general assumption
  int begNormIndex; // index of first normal in the present object; not the most general assumption
  string comma;

  cout << "Entering readWRL" << endl;

  nMaterial = nVert = nFace = nNorm = nSpotLight = nPtLight = 0;
  string key;  infile >> key;
  while (!infile.eof())
    {
      if (key == def)
	infile>>key>>key; // name of node/path/engine, presently discarded; next keyword
      if (key == separator) 
	{
	  getLeftBrace(infile);
	  level++;
	}
      else if (key == info) 
	{
	  getLeftBrace(infile);
	  skipToMatchingRightBrace (infile);
	}
      else if (key == spot)
	{
	  getLeftBrace(infile);
	  nSpotLight++;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == ptlight) 
	{
	  getLeftBrace(infile);
	  nPtLight++;
	  skipToMatchingRightBrace (infile);
	}
      else if (key == shapehints) 
	{
	  cout << "Ignoring shape hints" << endl;
	  getLeftBrace(infile);
	  skipToMatchingRightBrace (infile);
	}
      else if (key == coord3) 
	{
	  cout << "coord3" << endl;
	  // POINTS BEING SPECIFIED, SET A FLAG; IF NORMALS ARE NOT SPECIFIED BEFORE 
	  // THIS LEVEL IS POPPED, LEAVE ROOM FOR THEM ANYWAY
	  getLeftBrace(infile);
	  begVertIndex = nVert;
	  infile >> key;  assert (key == point);
	  getSymbol(infile,'[');
	  infile >> vert[nVert][0] >> vert[nVert][1] >> vert[nVert][2]; nVert++;
	  int done = 0;
	  while (!done && !tryToGetSymbol(infile,']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']'))
		{ infile >> vert[nVert][0] >> vert[nVert][1] >> vert[nVert][2]; nVert++; }
	      else done = 1;
	    }
	  getRightBrace(infile);
	}
      else if (key == normal)
	{
	  cout << "normal" << endl; 
	  getLeftBrace(infile);
	  begNormIndex = nNorm;
	  infile >> key;  assert (key == vec);
	  getSymbol(infile,'[');
	  infile >> norm[nNorm][0] >> norm[nNorm][1] >> norm[nNorm][2]; nNorm++;
	  int done = 0;
	  while (!done && !tryToGetSymbol(infile,']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']'))
		{ infile >> norm[nNorm][0] >> norm[nNorm][1] >> norm[nNorm][2]; nNorm++; }
	      else done = 1;
	    }
	  getRightBrace(infile);
	}
      else if (key == mat) 
	{
	  cout << "Material" << endl;
	  getLeftBrace(infile);
	  insideMaterial = 1;
	}
      else if (key == Ambient || key == ambient) 
	{
	  cout << "ambient" << endl;
	  infile >> mater[nMaterial][0] >> mater[nMaterial][1] >> mater[nMaterial][2];
	  // THERE MAY BE MORE THAN ONE MATERIAL SPECIFIED AT A TIME, e.g., balloon.wrl
	  // START HERE
	}
      else if (key == Diffuse || key == diffuse) 
	{
	  cout << "diffuse" << endl;
	  infile >> mater[nMaterial][3] >> mater[nMaterial][4] >> mater[nMaterial][5];
	}
      else if (key == Specular || key == specular)
	{
	  cout << "specular" << endl;
	  infile >> mater[nMaterial][6] >> mater[nMaterial][7] >> mater[nMaterial][8];
	  //  float foo; infile >> foo >> foo >> foo; // ignore specular
	}
      else if (key == Emissive || key == emissive)
	{
	  cout << "emissive" << endl;
	  infile >> mater[nMaterial][9] >> mater[nMaterial][10] >> mater[nMaterial][11];
	}
      else if (key == Shininess || key == shininess)
	{
	  cout << "shininess" << endl;
	  infile >> mater[nMaterial][12];
	}
      else if (key == Transparency || key == transparency)
	{
	  cout << "transparency" << endl;
	  infile >> mater[nMaterial][13];
	}
      else if (key == indexedfaceset) 
	{
	  getLeftBrace(infile);
	  facemode = INDEXEDFACESET;
	  insideFace = 1;
	} 
      else if (key == coordindex) 
	{
	  cout << "coordindex" << endl;
	  begFaceIndex = nFace;  // set a pointer to return to, if normalIndex follows
	  getSymbol (infile, '[');
	  int nv = countVertPerFace (infile);
	  cout << nv << endl;
	  face[nFace].allocate(nv);
	  for (i=0; i<nv; i++)
	    {
	      infile >> face[nFace][i] >> comma;
	      face[nFace][i] += begVertIndex;
	    }
	  cout << face[nFace] << endl;
	  assert (nMaterial > 0);
	  faceMaterial[nFace] = nMaterial-1;
	  nFace++;
	  int foo;
	  infile >> foo; // read -1
	  int done = 0;
	  while (!done && !tryToGetSymbol (infile, ']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']')) // last -1 may have a trailing comma
		{
		  nv = countVertPerFace (infile);
		  cout << nv << endl;
		  face[nFace].allocate(nv);
		  for (i=0; i<nv; i++)
		    {
		      infile >> face[nFace][i] >> comma;
		      face[nFace][i] += begVertIndex;
		    }
		  cout << face[nFace] << endl;
		  faceMaterial[nFace] = nMaterial-1;
		  nFace++;
		  infile >> foo;  // -1
		}
	      else done = 1;
	    }
	}
      else if (key == normalindex) // assumes that coordIndex comes before normalIndex,
	{                          // otherwise size information will not be available
	  // future: could alter to allow reverse action if coordIndex follows normalIndex
	  cout << "normalindex" << endl;
	  nFace = begFaceIndex;  // should be safe to return here, since we expect the 
	     // same # of faces in the normalIndex section as in the coordIndex section, 
	     // so we will return nFace to its present value
	  getSymbol (infile, '[');
	  int nv = countVertPerFace (infile);
	  normPerFace[nFace].allocate(nv);
	  for (i=0; i<nv; i++)
	    {
	      infile >> normPerFace[nFace][i] >> comma;
	      normPerFace[nFace][i] += begNormIndex;
	    }
	  nFace++;
	  int foo;
	  infile >> foo; // read -1
	  int done = 0;
	  while (!done && !tryToGetSymbol (infile, ']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']')) // last -1 may have a trailing comma
		{
		  nv = countVertPerFace (infile);
		  normPerFace[nFace].allocate(nv);
		  for (i=0; i<nv; i++)
		    {
		      infile >> normPerFace[nFace][i] >> comma;
		      normPerFace[nFace][i] += begNormIndex;
		    }
		  nFace++;
		  infile >> foo;  // -1
		}
	      else done = 1;
	    }
	}
      else if (key == materialindex) 
	{
	  cout << "material index" << endl;
	  nFace = begFaceIndex;
	  getSymbol (infile, '[');
	  infile >> faceMaterial[nFace];
	  nFace++;
	  int done = 0;
	  while (!done && !tryToGetSymbol(infile,']'))
	    {
	      getSymbol(infile,',');
	      if (!tryToGetSymbol(infile,']')) // possible trailing comma
		{
		  infile >> faceMaterial[nFace];
		  nFace++;
		}
	      else done = 1;
	    }
	}
      else if (key == textureindex) cout << "textureindex" << endl;
      else if (key == texcoord2)
	{
	  cout << "Ignoring TextureCoordinate2 for now" << endl;
	  skipToMatchingRightBrace(infile);
	}
      else if (key == translation) 
	{
	  cout << "Ignoring translation for now" << endl;
	  getLeftBrace(infile);
	  skipToMatchingRightBrace (infile);
	}
      else if (key == rotation) 
	{
	  cout << "Ignoring rotation for now" << endl;
	  getLeftBrace(infile);
	  skipToMatchingRightBrace (infile);
	}
      else if (key == scale)
	{
	  cout << "Ignoring scale for now" << endl;
	  getLeftBrace(infile);
	  skipToMatchingRightBrace (infile);
	}
      else if (key == materialbinding) 
	{
	  cout << "Ignoring material binding for now" << endl;
	  getLeftBrace(infile);
	  skipToMatchingRightBrace (infile);
	}
      else if (key == normalbinding) 
	{
	  cout << "Ignoring normal binding for now" << endl;
	  getLeftBrace(infile);
	  skipToMatchingRightBrace (infile);
	}
      else if (key == matrixtransform)
	{
	  cout << "Ignoring matrix transform for now" << endl;
	  getLeftBrace(infile);
	  skipToMatchingRightBrace (infile);
	}
      else cout << "something else: " << key << endl;
      while (!infile.eof() && tryToGetRightBrace(infile)) 
	{
	  if (insideFace) 
	    {
	      insideFace=0; // found matching brace to INDEXEDFACESET or the like
	    }
	  else if (insideMaterial)
	    {
	      insideMaterial=0; // found matching brace to Material
	      nMaterial++;
	    }
	  else 
	    {
	      level--;  // found the matching brace to Separator
	    }
	}
      if (!infile.eof()) { infile >> key; }
    }

}

/******************************************************************************
******************************************************************************/

int main (int argc, char **argv)
{
  parse (argc,argv);

  int i;
  ifstream infile;

  //  use kaufmann.wrl and chess4.wrl as templates

  infile.open(argv[argc-1]);
  allocateWRL (infile);
  readWRL (infile);
  scaleToUnitCube (vert);

  /*
  push in a level whenever 'Separator' is read
    template appears to be: keyword { }
  if the keyword is not one we are interested in, we can skip over the braces
	   ignorable: Info, Spotlight, PointLight (cout that we are ignoring)
	   parsable: Separator, Coordinate3, Normal, DEF -- Material, IndexedFaceSet, 
	             coordIndex, normalIndex, NormalBinding(?)
	   anchor.wrl has a strange coordIndex list (no -1)
  apply material to its level: may want a flag to identify if a material is defined at a certain level

  */
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
