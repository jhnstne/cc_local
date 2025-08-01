#ifndef REAL
#define REAL float
#endif REAL

#define NUM_ISO	        5	/* # of isoparametric curves drawn per patch */
			        /* of tensor product surface */
				/* used to be 3, then 10 */
#define TP_DENSITY	20	/* number of points to be drawn per
				   segment of  isoparametric curve */
				/* used to be 2 */
#define MAXISOCURVES	MAXSEG * NUM_ISO  /* max # of isoparametric curves on 
				/* a tensor product surface */

typedef struct {
	int d_u;		/* degree in u-direction */
	int d_v;		/* degree in v-direction */
	int L_u;		/* control pts range from xi[0] to xi[d_u * L_u] */
				/* in u-direction, which is vertical direction */
				/* i.e., there are d_u*L_u rows */
				/* knots range from knots_u[0] to knots_u[L_u] */
				/* again in vertical direction */

	int L_v;		/* there are d_v * L_v columns */
				/* typical loop is for (j=0; j<=d_v*L_v; j++) */
				/* 			surf[i][j]  	      */
	REAL knots_u[MAXSEG]; /* knot sequence */
	REAL knots_v[MAXSEG];
	REAL x1[MAXSPLINEPTS][MAXSPLINEPTS]; /* control pts */
		/* x1[i-1][j-1] is the ith control point in u-direction and jth in v- */ 
	REAL x2[MAXSPLINEPTS][MAXSPLINEPTS];
	REAL x3[MAXSPLINEPTS][MAXSPLINEPTS];
	REAL weights[MAXSPLINEPTS][MAXSPLINEPTS];
} tp_ratbez; 		/* tensor product rational Bezier surface */

typedef struct {
	REAL point[MAXISOCURVES][MAXDISPLAYPTS][3];	/* points to be drawn */
			/* used to be called display */
	REAL normal[MAXISOCURVES][MAXDISPLAYPTS][3];	/* their unit normals */
	int  point_num;	/* # of points drawn on each isoparametric curve */
	int  iso_num;	/* # of isoparametric curves drawn */
} display_surf;

extern void	prepare_draw_surf_tpratbez (tp_ratbez *surf, 
		         display_surf *display);

extern void 	ratbezsurf_to_pointsAndNormals(int u_patch, int v_patch, 
			  tp_ratbez *patch,
			  int bezdensity, int isodensity,
			  display_surf *display);

extern void 	draw_surf_tpratbez (display_surf *display);

extern void 	draw_surf_tpratbez_flat (display_surf *display);

extern void 	draw_control_mesh_tpratbez (const tp_ratbez *surf);

extern void 	output_tp_ratbez (const tp_ratbez *surf);

extern void 	normal_tpratbez (tp_ratbez *surf, REAL a, REAL b, REAL normal[3]);




