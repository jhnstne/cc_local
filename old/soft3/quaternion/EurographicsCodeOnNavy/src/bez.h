#ifndef REAL
#define REAL float
#endif REAL

/* #include "/usr/people/jj/cbin/datasize.h" */
#define MAXDATAPTS	20
#define MAXSEG		MAXDATAPTS	/* max segments in B-spline or Bezier */
			/* used to be 20 */
#define MAXDEGREE	16	/* max degree of B-spline or Bezier */
			/* used to be 20 */
#define	MAXKNOTS	MAXSEG + 2*(MAXDEGREE-1)  
	/* start/end multiple knots for Bspl */
#define MAXCTRLPTS	MAXSEG * MAXDEGREE + 1	/* max # of control points on a Bezier */
#define DENSITY		20	/* used to be 5 */		
#define BSPL_DENSITY	DENSITY		/* # points per B-spline segment */
#define BEZ_DENSITY	DENSITY		/* # points per Bezier segment */
#define MAXDISPLAYPTS	(BSPL_DENSITY * MAXSEG)
#define MAXSPLINEPTS	MAXCTRLPTS

typedef struct {
	int d;			/* degree */
	int L;			/* number of intervals */
				/* control points range from x[0] to x[dL] */
				/* knots range from knots[0] to knots[L] */
	REAL knots[MAXSEG];	/* knot sequence */
	REAL x[MAXSPLINEPTS];	/* control points */
} bez_1d;

typedef struct {
	int d;				/* degree */
	int L;			/* control points range from x[0] to x[L+2] */
				/* knots range from knots[0] to 
			           knots[L+2*degree-2], with multiplicity d 
				   knots at beginning and end */
	REAL knots[MAXKNOTS];	/* knot sequence */
	REAL x[MAXCTRLPTS];	/* control points */
} bspl_1d;

typedef struct {
	int d;				/* degree */
	int L;			/* control points range from xi[0] to xi[dL] */
				/* knots range from knots[0] to knots[L] */
	REAL knots[MAXSEG];	 /* knot sequence */
	REAL x1[MAXSPLINEPTS]; /* control points */
	REAL x2[MAXSPLINEPTS];
	REAL x3[MAXSPLINEPTS];
} bez_3d;

typedef struct {
	int d;				/* degree */
	int L;			/* control points range from xi[0] to xi[L+2] */
				/* knots range from knots[0] to 
			           knots[L+2*degree-2], with multiplicity d 
				   knots at beginning and end */
	REAL knots[MAXKNOTS];	/* knot sequence */
	REAL x1[MAXCTRLPTS];	/* control points */
	REAL x2[MAXCTRLPTS];
	REAL x3[MAXCTRLPTS];
} bspl_3d;

typedef struct {
	int d;				/* degree */
	int L;			/* control points range from xi[0] to xi[dL] */
				/* knots range from knots[0] to knots[L] */
	REAL knots[MAXSEG];		/* knot sequence */
	REAL x1[MAXSPLINEPTS]; 	/* control points */
	REAL x2[MAXSPLINEPTS];
	REAL x3[MAXSPLINEPTS];
	REAL x4[MAXSPLINEPTS];
} bez_4d;

typedef struct {
	int d;				/* degree */
	int L;			/* control points range from xi[0] to xi[dL] */
				/* knots range from knots[0] to knots[L] */
	REAL knots[MAXSEG];	 /* knot sequence */
	REAL x1[MAXSPLINEPTS]; /* control points */
	REAL x2[MAXSPLINEPTS];
	REAL x3[MAXSPLINEPTS];
	REAL weights[MAXSPLINEPTS];
} ratbez_3d;

typedef struct {
	int d;				/* degree */
	int L;			/* control points range from xi[0] to xi[dL] */
				/* knots range from knots[0] to knots[L] */
	REAL knots[MAXSEG];	 /* knot sequence */
	REAL x1[MAXSPLINEPTS]; /* control points */
	REAL x2[MAXSPLINEPTS];
	REAL x3[MAXSPLINEPTS];
	REAL x4[MAXSPLINEPTS];
	REAL weights[MAXSPLINEPTS];
} ratbez_4d;

typedef struct {
	int d;				/* degree */
	int L;			/* control points range from xi[0] to xi[L+2] */
				/* knots range from knots[0] to 
			           knots[L+2*degree-2], with multiplicity d 
				   knots at beginning and end */
	REAL knots[MAXKNOTS];	/* knot sequence */	/* used to be MAXCTRLPTS */
	REAL x1[MAXCTRLPTS];	/* control points */
	REAL x2[MAXCTRLPTS];
	REAL x3[MAXCTRLPTS];
	REAL x4[MAXCTRLPTS];
} bspl_4d;

extern void 	bspl_to_bezier_1d(const bspl_1d *bspl, bez_1d *bez);

extern void 	bspl_to_bezier_3d(const bspl_3d *bspl, bez_3d *bez);

extern void 	bspl_to_bezier_4d(const bspl_4d *bspl, bez_4d *bez);

extern void 	bspline_to_bezier(const REAL bspl[], const REAL knot[], 
			      const int l, REAL bez[]);

extern void 	bspl_to_points(int degree,int l,REAL coeff_x[],REAL knot[], 
		    	   int dense, REAL points_x[], int *point_num);

extern REAL 	deboor(int degree,REAL coeff[],REAL knot[],REAL u,int i);

extern void 	point_on_bspl_3d (const bspl_3d *bspl, const REAL u, 
				  V3d pt);

extern void 	bez_to_points(int degree,int npoints,
				REAL coeff[],REAL points[]);

extern void 	bez_3d_to_points (bez_3d *bez, int npoints, V3d points[]);

extern REAL 	hornbez(int degree, REAL coeff[], REAL t);

extern void 	ratbez_to_points(int degree, int npoints, 
				REAL coeff[],REAL weight[],REAL points[]);

extern void 	ratbez_3d_to_points (ratbez_3d *rbez, int npoints, V3d points[]);

extern REAL 	ratbez(int degree, REAL coeff[], 
			REAL weight[], REAL t);

extern void 	point_on_bez_1d (const bez_1d *bez, const REAL u, REAL *pt);

extern void 	point_on_bez_3d (const bez_3d *bez, const REAL u, V3d pt);

extern void     point_on_bez_4dh (const bez_4d *bez, const REAL u,
				  v4dh pt);

extern void 	point_on_ratbez_3d (const ratbez_3d *rbez, const REAL u, 
				  V3d pt);

extern void 	point_on_ratbez_4dh (const ratbez_4d *rbez,
				  const REAL u,v4dh pt);

extern void 	deriv_bez (bez_3d *bez, REAL a, REAL deriv[3]);

extern void 	deriv_ratbez (ratbez_3d *rbez, REAL a, REAL deriv[3]);

extern void 	degree_elevate_bez1d (bez_1d *oldbez, bez_1d *newbez);

extern void 	degree_elevate_bez3d (bez_3d *oldbez, bez_3d *newbez);

extern void 	subdiv_ratbez3d (ratbez_3d *rbez, REAL t, 
			ratbez_3d *left, ratbez_3d *right);

extern void 	subdiv(int degree, REAL coeff[], REAL weight[], REAL t,
		REAL bleft[], REAL bright[], REAL wleft[], REAL wright[]);

extern void 	print_bspl3d (const bspl_3d *bspl);

extern void 	print_bspl (const bspl_4d *bspl);

extern void	print_bez_3d (const bez_3d *bez);

extern void 	print_bez  (const bez_4d *bez);

extern void 	print_ratbez3d (const ratbez_3d *ratbez);

extern void 	print_ratbez (const ratbez_4d *ratbez);

extern void 	input_bez3d(bez_3d *bez, FILE *fp);

extern void 	input_bez4d(bez_4d *bez, FILE *fp);

extern void 	input_ratbez3d(ratbez_3d *ratbez, FILE *fp);

extern void 	input_ratbez4d(ratbez_4d *ratbez, FILE *fp);

extern void 	output_bez3d(bez_3d *bez, FILE *fp);

extern void 	output_bez4d(bez_4d *bez, FILE *fp);

extern void 	output_ratbez4d(ratbez_4d *ratbez, FILE *fp);

extern void 	prepare_draw_bspl_in_3d (const bspl_3d *bspl, 
			      REAL display_bspl[3][MAXDISPLAYPTS],
			      int *point_num);
		/* display arrays really should be ...[MAXDISPLAYPTS][3], */
		/* not ...[3][MAXDISPLAYPTS], but historically difficult to change */
		/* wait until C++ shift */

extern void 	prepare_draw_bspl_in_4d (const bspl_4d *bspl,
			REAL display_bspl[4][MAXDISPLAYPTS], int *point_num);

extern void 	prepare_draw_bez_in_3d (const bez_3d *bez, 
			REAL display_bez[3][MAXDISPLAYPTS],  int *point_num);

extern void 	prepare_draw_bez_in_4d (const bez_4d *bez, 
			REAL display_bez[4][MAXDISPLAYPTS],  int *point_num);

extern void 	prepare_draw_ratbez_in_3d (const ratbez_3d *ratbez, 
			REAL display_ratbez[3][MAXDISPLAYPTS], int *point_num);

extern void 	prepare_draw_ratbez_in_4d (const ratbez_4d *ratbez, 
		        REAL display_ratbez[4][MAXDISPLAYPTS], int *point_num);

extern void 	draw_bspl_control_in_4d (const bspl_4d *bspl);

extern void 	draw_bez_control_in_1d (const bez_1d *bez);

extern void 	draw_bez_control_in_3d (const bez_3d *bez);

extern void 	draw_bez_control_on_4d_sphere(const bez_4d *bez);

extern void 	draw_ratbez_control_on_4d_sphere (const ratbez_4d *ratbez);

extern void 	draw_curve_in_3d(REAL display[3][MAXDISPLAYPTS], 
			const int point_num);

extern void 	draw_curve_in_4d(REAL display[4][MAXDISPLAYPTS], 
			const int point_num);

extern void 	draw_curve_on_4d_sphere (REAL display[4][MAXDISPLAYPTS],
			const int point_num);

extern void 	copy_bez3d(bez_3d *b, bez_3d *c);

/* extern void 	drawObject(const Object obj,const V3d pos,const Qion q) */ /* e.g., obj = cyclide or banana */ 

