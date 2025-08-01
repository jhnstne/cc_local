/*
	File: quaternion.h
	Author: J.K. Johnstone
	Last Modified: September 26, 1994
	Purpose:
	
*/

/* input of quaternion, translations to rotmatrix and Euler angles, 
	multiplication of quaternions, etc. */

#define MAX 1000 	/* max # of vertices in entire tmesh (consisting of
			  several tmeshes, one per face) of object */
#define MAXMESHES 20	/* max # of tmeshes on a single object */

typedef v4dh Qion;	/* unit quaternion */
			/* a unit quaternion is simply a vector in 4-space
				with special properties:
				v0 = 1, v1 = cos(\theta/2),
				(v2,v3,v4) = sin(\theta/2) * rotation-axis */

extern void 	quaternion_to_matrix (const Qion q, Matrix M);

extern void 	draw_quaternion (Qion q);

extern void 	draw_4dhpoint (v4dh p);

extern void 	draw_4dnurbs (const bspl_4d *bspl);

extern void 	inputQion (unsigned int *n, Qion q[], FILE *fp); 

extern void 	draw_oriented_shaded_object (Qion q, V3d pos,
				  int n, int num_per_mesh[MAXMESHES],
				  REAL V[MAX][3], REAL N[MAX][3],
				  int swap[MAX]);

extern void 	generate_extruded_tmesh   (int k, 
				REAL p[MAX][3],
				REAL dist,
				REAL V[MAX][3], 
				REAL N[MAX][3], 
				int swap[MAX],
				int *num_meshes,
				int num_per_mesh[MAXMESHES]);

extern void 	draw_oriented_wire_cube (Qion q, V3d pos);

extern void	draw_oriented_curve (REAL display_curve[3][MAXDISPLAYPTS],
			   const int point_num, const Qion q, const V3d pos);

extern void 	draw_oriented_scaled_curve (REAL display_curve[3][MAXDISPLAYPTS],
			  const int point_num,
			  const Qion q,
			  const REAL s,
			  const V3d pos);

