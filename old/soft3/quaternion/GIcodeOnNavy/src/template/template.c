/*
	File: template.c
	Author: J.K. Johnstone
	Last Modified: Oct. 11, 1994
	Purpose: A template GL program that draws a single sphere.
	
*/

#include <device.h>
#include <gl/gl.h>
#include <gl/sphere.h>
#include <math.h>
#include <stdio.h>

#include "display.h"
#include "lights.h"
#include "materials.h"
#include "template.h"

#include "display.c"

int 	xmax,ymax;	/* screen dimensions */
int	zmax;		/* z-buffer size */
float	rotx,roty,rotz;

main()
{

	long		sphere_wid;

        Boolean 	exitflag;	/* window */
        short   	attached = 0;
        short   	value;
        int     	dev;
	int 	       	i;
	
	sphere_wid = initialize_3d_window("Sphere");
/*	winset(sphere_wid); */
	exitflag=FALSE;
	while (exitflag == FALSE) {
                drawscene(0,0,0,1);

                while ((exitflag == FALSE) && (qtest() || !attached)) {
                        dev = qread(&value);
                        if (((dev == ESCKEY) && (value == 0)) ||
			    dev == LEFTMOUSE)
                                exitflag = TRUE;
                        else if (dev == WINFREEZE || dev == WINTHAW || dev == REDRAWICONIC) {
                                frontbuffer(TRUE);
                                pushmatrix();
                                reshapeviewport();
                                drawscene(0,0,0,1);
                                popmatrix();
                                frontbuffer(FALSE);
                        }
                        else if (dev == REDRAW)
                                reshapeviewport();
                        else if (dev == INPUTCHANGE)
                                attached = value;
                }
        }
}

void drawscene(float x, float y, float z, float r)
{

	extern int xmax,ymax,zmax;
	extern float rotx,roty,rotz;
        float params[4];
	short rgbvec[3];

	czclear(0xFFFF55,zmax);
        perspective(1400, (float)xmax/(float)ymax, 0.00001, 10.0);
        pushmatrix();
	ortho(-3*xmax/ymax,3*xmax/ymax,-3,3,-3,3);
	translate(0.,0.,0.);
	rotate(rotx,'x');
	rotate(roty,'y');
	rotate(rotz,'z');

/*	RGBcolor(0,255,0); */
	rgbvec[0]=rgbvec[2]=0;
	rgbvec[1]=255;
	c3s(rgbvec);
	/* draw sphere */
        params[0]=x;
        params[1]=y;
        params[2]=z;
        params[3]=r;
        sphdraw( params );

	rotx += 20;
	roty += 20;
	rotz += 20; 
	popmatrix();
	swapbuffers();
}
