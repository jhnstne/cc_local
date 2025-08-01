#define MAXINST		20	/* max # of curve instances to be interpolated
				   by swept surface */
#define PT_RADIUS	.03
#define MAXKEYFRAMES	20	/* max # of keyframes of swept curve */

enum keycomponent {NONE, POS, ORI, SCALE}; /* component of keyframe presently */
					   /* being edited */

typedef struct {
	V3d	pos;	/* position of reference vertex */
			/* = 1st control point of sweep curve */
	Qion	ori;
	double	scale;
} keyframe;

extern void viz_sweep	(double	display_sweepcurve[3][MAXDISPLAYPTS],
			 const int	sweepcurve_num,
			 int		present,
			 int		status,
			 keyframe	keys[]);

extern void draw_keyframe (keyframe *key, 
		  	   double display_curve[3][MAXDISPLAYPTS],
			   const int curve_num);

extern double transl	   (short mval,long size,double wincen,
			    double min,double max);

extern double rotamt	   (short mval, long size);

extern void keybd_input	  (keyframe *key, int kc);
