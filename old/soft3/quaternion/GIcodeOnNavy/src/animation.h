/*
        File: animation.h
        Author: J.K. Johnstone
        Last Modified: October 17, 1994
	Purpose:
*/

#define MAXFRAMES       20      /* maximum number of key frames (orientations) 
                                   in animation */
#define PT_RADIUS       .03
#ifndef PI
#define PI              3.141593
#endif

extern void animate (const int 	      n,
		   const unsigned int m,
		   const Qion	      keyq[],
		   const V3d	      keypos[],
		   const ratbez_4d    *sphereratbez,
		   REAL display_sphereratbez[4][MAXDISPLAYPTS],
		   const int 	      spherebez_num,
		   REAL V[MAX][3],
		   REAL N[MAX][3],
		   int swap[MAX],
		   int num_meshs,
		   int num_per_mesh[MAXMESHES],
		   const bez_3d       *directrixbez,
	           REAL display_dirbez[3][MAXDISPLAYPTS],
                   const int   	      dirbez_num);

extern void sphere(const int m, const Qion q[], const v4dh p[], 
            REAL display_pbspl[4][MAXDISPLAYPTS], 
            const int pbspl_num,
            const bspl_4d *pBspl,
            REAL display_pbez[4][MAXDISPLAYPTS],
            const int pbez_num,
            const bez_4d *pBez,
            REAL display_sphereratbez[4][MAXDISPLAYPTS],  
            const int spherebez_num, 
            const ratbez_4d *sphereRatBez);

extern void inputPolygon (int *k, REAL p[MAX][3], FILE *fp);
