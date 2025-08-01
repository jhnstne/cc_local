// Source: Edward Angel's 'Interactive Computer Graphics'
// via www.macdevcenter.com/pub/a/mac/2005/04/01/opengl.html
// with some extensions to test file I/O

#include <stdlib.h>
#include <GLUT/glut.h>
#include <iostream>
#include <fstream>
using namespace std;

#define kWindowWidth  640
#define kWindowHeight 480

typedef GLfloat point[3];

point v[4] = {{0.0, 0.0, 1.0},
	      {0.0, 0.942809, -0.333333},
	      {-0.816497, -0.471405, -0.333333},
	      {0.816497, -0.471405, -0.333333}};

void triangle (point a, point b, point c)
{
  glBegin(GL_LINE_LOOP);
  glVertex3fv(a);
  glVertex3fv(b);
  glVertex3fv(c);
  glEnd();
}

void tetrahedron()
{
  triangle(v[0],v[1],v[2]);
  triangle(v[3],v[2],v[1]);
  triangle(v[0],v[2],v[3]);
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT);
  glColor3f(1,1,1);
  glLoadIdentity();

  tetrahedron();
  glutSwapBuffers();
}

int main (int argc, char** argv)
{
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize (kWindowWidth, kWindowHeight);
  glutInitWindowPosition (100,100);
  glutCreateWindow ("simple opengl example");

  ifstream bar; bar.open("bar"); int foo; bar >> foo;
  cout << "you inputted " << foo << endl;
  
  glutDisplayFunc(display);
  glutMainLoop();
  return 0;
}
