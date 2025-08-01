void bspl_to_bezier_3d(const bspl_3d *bspl, bez_3d *bez)
{
	/* cubic B-spline to cubic Bezier */

	int i,offset;
	if (bspl->d != 3)
		printf("Error: bspl_to_bezier applied to non-cubic\n");
	else {
		bez->d = bspl->d;
		bez->L = bspl->L;
		offset = bspl->d - 1;
		for (i=0;i<=bez->L;i++) {
			bez->knots[i] = bspl->knots[i+offset]; /* skip 
					over multiple instances of beg knot */
		}
		bspline_to_bezier(bspl->x1,bez->knots,bez->L,bez->x1);
		bspline_to_bezier(bspl->x2,bez->knots,bez->L,bez->x2);
		bspline_to_bezier(bspl->x3,bez->knots,bez->L,bez->x3);
	}
}

void bspl_to_bezier_4d(const bspl_4d *bspl, bez_4d *bez)
{
	/* cubic B-spline to cubic Bezier */

	int i,offset;
	if (bspl->d != 3)
		printf("Error: bspl_to_bezier applied to non-cubic\n");
	else {
		bez->d = bspl->d;
		bez->L = bspl->L;
		offset = bspl->d - 1;
		for (i=0;i<=bez->L;i++) {
			bez->knots[i] = bspl->knots[i+offset]; /* skip 
					over multiple instances of beg knot */
		}
		bspline_to_bezier(bspl->x1,bez->knots,bez->L,bez->x1);
		bspline_to_bezier(bspl->x2,bez->knots,bez->L,bez->x2);
		bspline_to_bezier(bspl->x3,bez->knots,bez->L,bez->x3);
		bspline_to_bezier(bspl->x4,bez->knots,bez->L,bez->x4);
	}
}

void bspline_to_bezier(const REAL bspl[], const REAL knot[], const int l, 
		       REAL bez[])
	/* converts  cubic B-spline polygon into piecewise Bezier polygon 
Input:	bspl: B-spline control polygon 
	knot: knot sequence
	l:    no. of intervals

Output:	bez:  piecewise Bezier polygon 
*/
	/* scavenged from Farin, 3rd edition diskette */
{
	int i, i3, l3;
/* The first points need special attention:  */

	bez[0]=bspl[0];
	bez[1]=bspl[1];
	if(l>1)
	{
		bez[2]=((knot[2]-knot[1])*bspl[1]
		+(knot[1]-knot[0])*bspl[2]    )/(knot[2]-knot[0]);
	}   
	if(l>2)
	{
		bez[4]=( (knot[3]-knot[1])*bspl[2]
	        +(  knot[1]-knot[0])*bspl[3]  )/(knot[3]-knot[0]);
		bez[3]=( (knot[2]-knot[1])*bez[2]
	        + (knot[1]-knot[0])*bez[4]    )/(knot[2]-knot[0]);
	}
/* Now the main part:   */

	for(i=2; i<l-1; i++)
	{		
	i3=3*i;
	bez[i3-1]=( (knot[i+1]-knot[i])*bspl[i]
	         +  (knot[i]-knot[i-2])*bspl[i+1]    )/(knot[i+1]-knot[i-2]);

	bez[i3+1]=( (knot[i+2]-knot[i])*bspl[i+1]
	         +(  knot[i]-knot[i-1])*bspl[i+2]  )/(knot[i+2]-knot[i-1]);

	bez[i3]  =( (knot[i+1]-knot[i])*bez[i3-1]
		  + (knot[i]-knot[i-1])*bez[i3+1]  )/(knot[i+1]-knot[i-1]);
	}
	
/* The last points need special attention:  */

	l3=l*3;
	if(l>2)
	{
	bez[l3-4]=( (knot[l]-knot[l-1])*bspl[l-1]
	         +  (knot[l-1]-knot[l-3])*bspl[l]  )/(knot[l]-knot[l-3]);
	}
	if(l>1)
	{
	bez[l3-2]=( (knot[l]-knot[l-1])*bspl[l]
		 +  (knot[l-1]-knot[l-2])*bspl[l+1]    )/(knot[l]-knot[l-2]);

	bez[l3-3]=( (knot[l]-knot[l-1])*bez[l3-4]
		 + (knot[l-1]-knot[l-2])*bez[l3-2]   )/(knot[l]-knot[l-2]);
	}
	bez[l3-1]= bspl[l+1];
	bez[l3]  = bspl[l+2];
}

void bspl_to_points(int degree, int l, REAL coeff_x[], REAL knot[], 
		    int dense, REAL points_x[], int *point_num)
/*
	generates points on B-spline curve. (one coordinate)
input:	degree:	polynomial degree of each piece of curve
	l:	number of intervals
	coeff:	B-spline control points
	knot:	knot sequence: knot[0]...knot[l+2*degree-2]
	dense:	how many points per segment
Output:	points:	output array.
	point_num: how many points are generated.
*/
	/* scavenged from Farin, 3rd edition diskette */
{
	int i,j,ii,kk,l2;
	REAL u, mmbox[4];

	l2=l+degree-1;

	*point_num=0;
	for (i=degree-1; i<l+degree-1; i++)
	{
	if(knot[i+1]>knot[i])
		for(ii=0; ii<=dense; ii++)
		{
		u=knot[i]+ii*(knot[i+1]-knot[i])/dense;
		points_x[*point_num]=deboor(degree,coeff_x,knot,u,i);

		*point_num= (*point_num)+1;
		}	
	}
}

REAL deboor(int degree, REAL coeff[], REAL knot[], REAL u, int i)
/*
	uses de Boor algorithm to compute one
	coordinate on B-spline curve for param. value u in interval i.
input:  degree:	polynomial degree of each piece of curve
	coeff:	B-spline control points
	knot:	knot sequence
	u:	evaluation abscissa
	i:	u's interval: u[i]<= u < u[i+1]
output:	coordinate value. 
*/
	/* scavenged from Farin, 3rd edition diskette */
{
	int k,j;
	REAL t1,t2;
	REAL coeffa[30];  /* might need adjustment! */
	
	for (j=i-degree+1; j<=i+1; j++)coeffa[j]=coeff[j];

	for (k=1; k<= degree; k++)
	for ( j=i+1 ;j>=i-degree+k+1; j--)
	{
		t1= (knot[j+degree-k] - u )/(knot[j+degree-k]-knot[j-1]);
		t2= 1.0-t1;

		coeffa[j]=t1* coeffa[j-1]+t2* coeffa[j];
	}	
	return (coeffa[i+1]);
}

void point_on_bspl_3d (const bspl_3d *bspl, const REAL u, V3d pt)
{
	/* find point on a B-spline with parameter value u */
	int maxknot, i=0;
	
	/* find u's interval: knot[i] <= u < knot[i+1] */

	maxknot = bspl->L + 2*bspl->d - 2;
	if (u<bspl->knots[0] || u>bspl->knots[maxknot])
		printf("Parameter %f out of range\n",u);
	else {
		if (u==bspl->knots[maxknot]) {
			i=maxknot;
		}
		else {
			while (bspl->knots[i+1] <= u)
				i++;
		}
	}

	if (i==maxknot) { /* u at end of knot sequence */
		pt[0]=bspl->x1[bspl->L + 2];
		pt[1]=bspl->x2[bspl->L + 2];
		pt[2]=bspl->x3[bspl->L + 2];
	}
	else {
		pt[0]=deboor(bspl->d, bspl->x1, bspl->knots, u, i);
		pt[1]=deboor(bspl->d, bspl->x2, bspl->knots, u, i);
		pt[2]=deboor(bspl->d, bspl->x3, bspl->knots, u, i);
	}
}

void bez_to_points(int degree, int npoints, REAL coeff[], REAL points[])
/*	Converts Bezier curve into point sequence. Works on
	one coordinate only.
 
	Input:   degree:  degree of curve.
	         npoints: # of coordinates to be generated. (counting
		          from 0!)
	         coeff:   coordinates of control polygon.
	Output:  points:  coordinates of points on curve.

	Remark: For a 2D curve, this routine needs to be called twice,
		once for the x-coordinates and once for y.
*/
	/* scavenged from Farin, 3rd edition diskette */
{	
	REAL t,delt;
	int i;

	delt=1.0/(REAL)npoints;
	t=0.0;
	for(i=0; i<=npoints; i++) 
	{
		points[i]=hornbez(degree,coeff,t);
		t = t+delt;
	}
}

REAL hornbez(int degree, REAL coeff[], REAL t)
/*
     uses  Horner's scheme to compute one coordinate
     value of a  Bezier curve. Has to be called 
     for each coordinate  (x,y, and/or z) of a control polygon.
     Input:   degree: degree of curve.
              coeff:  array with coefficients of curve.
              t:      parameter value.
      Output: coordinate value.
*/
	/* scavenged from Farin, 3rd edition diskette */
{
	int i;
	int n_choose_i;          /* shouldn't be too large! */
	REAL fact,t1,aux;

	t1=1.0 - t;  fact=1.0;
	n_choose_i=1;

	aux=coeff[0]*t1;          /* starting the evaluation
                                  loop */
	for(i=1; i<degree; i++)
	{
		fact=fact*t;
		n_choose_i=n_choose_i*(degree-i+1)/i;  /* always int! */
		aux=(aux + fact*n_choose_i*coeff[i])*t1;
	}
	aux = aux + fact*t*coeff[degree];

	return aux;
}

void ratbez_to_points(int degree, int npoints, REAL coeff[], REAL weight[],
		      REAL points[])
/*	Converts rational Bezier curve into point sequence. Works on
	one coordinate only.
 
	Input:   degree:  degree of curve.
	         npoints: # of coordinates to be generated. (counting
		          from 0!)
	         coeff:   coordinates of control polygon.
		 weight:  weights of control polygon
	Output:  points:  coordinates of points on curve.

	Remark: For a 2D curve, this routine needs to be called twice,
		once for the x-coordinates and once for y.
*/
{	
	REAL t,delt;
	int i;

	delt=1.0/(REAL)npoints;
	t=0.0;
	for(i=0; i<=npoints; i++) 
	{
		points[i]=ratbez(degree,coeff,weight,t);
		t = t+delt;
	}
}

REAL ratbez(int degree, REAL coeff[], REAL weight[], REAL t)
/*
	uses rational de casteljau to compute
	point on ratbez curve for param. value t.
*/
	/* scavenged from Farin, 3rd edition diskette */
{
	int r,i;
	REAL t1,t2,ww1,ww2;
	REAL coeffa[20], weighta[20];
	t1 = 1.0 - t;

	
	for (i=0;i<=degree; i++)
	{
		coeffa[i]=coeff[i];
		weighta[i]=weight[i];
	}

	for (r=1; r<= degree; r++)
	for (i=0; i<= degree - r; i++)
	{
	/*	t1= (i+r-degree*t)/(float)r;
		t2= 1.0-t1;  ... makes it interpolatory ...
	*/
		t1=1.0-t; t2=t;
		ww1 = weighta[i]; ww2= weighta[i+1];
		weighta[i] = t1*ww1 + t2*ww2;
		coeffa[i]  = ( t1*ww1 * coeffa[i]
			    + t2*ww2 * coeffa[i+1] ) / weighta[i];

	}	
	return (coeffa[0]);
}

void point_on_bez_1d (const bez_1d *bez, const REAL u, REAL *pt)
{
	/* find point on a polynomial Bezier spline with parameter value u */
	int i;
	REAL newu;
	
	/* find appropriate interval of spline: [knots[i],knots[i+1]] */
	i=0;
	while (u > bez->knots[i+1]) {
		i++;
	}
	/* apply map u = (u-a)/(b-a) to map from [a,b] to [0,1] */
	newu = (u - bez->knots[i])/(bez->knots[i+1] - bez->knots[i]);

	*pt=hornbez(bez->d,bez->x+(bez->d * i),newu);
}

void point_on_bez_3d (const bez_3d *bez, const REAL u, V3d pt)
{
	/* find point on a polynomial Bezier spline with parameter value u */
	int i;
	REAL newu;
	
	/* find appropriate interval of spline: [knots[i],knots[i+1]] */
	i=0;
	while (u > bez->knots[i+1]) {
		i++;
	}
	/* apply map u = (u-a)/(b-a) to map from [a,b] to [0,1] */
	newu = (u - bez->knots[i])/(bez->knots[i+1] - bez->knots[i]);

	pt[0]=hornbez(bez->d,bez->x1+(bez->d * i),newu);
	pt[1]=hornbez(bez->d,bez->x2+(bez->d * i),newu);
	pt[2]=hornbez(bez->d,bez->x3+(bez->d * i),newu);
}

void point_on_bez_4dh (const bez_4d *bez, const REAL u, v4dh pt)
{
	/* find point on a polynomial Bezier spline with parameter value u */
	int i;
	REAL newu;
	
	/* find appropriate interval of spline: [knots[i],knots[i+1]] */
	i=0;
	while (u > bez->knots[i+1]) {
		i++;
	}
	/* apply map u = (u-a)/(b-a) to map from [a,b] to [0,1] */
	newu = (u - bez->knots[i])/(bez->knots[i+1] - bez->knots[i]);

	pt[0]=1;
	pt[1]=hornbez(bez->d,bez->x1+(bez->d * i),newu);
	pt[2]=hornbez(bez->d,bez->x2+(bez->d * i),newu);
	pt[3]=hornbez(bez->d,bez->x3+(bez->d * i),newu);
	pt[4]=hornbez(bez->d,bez->x4+(bez->d * i),newu);
}

void point_on_ratbez_4dh (const ratbez_4d *rbez,const REAL u,v4dh pt)
{
	/* find point on a rational Bezier spline with parameter value u */
	int i;
	REAL newu;
	
	/* find appropriate interval of spline: [knots[i],knots[i+1]] */
	i=0;
	while (u > rbez->knots[i+1]) {
		i++;
	}
	/* apply map u = (u-a)/(b-a) to map from [a,b] to [0,1] */
	newu = (u - rbez->knots[i])/(rbez->knots[i+1] - rbez->knots[i]);
	
/*	printf("u=%.2f lies between %.2f and %.2f \n",u,rbez->knots[i],rbez->knots[i+1]);  
*	printf("new u is %.2f\n",newu);
*/

	/* now find point with parameter NEWU on ith segment */
	pt[0]=1.;
	pt[1]=ratbez(rbez->d,rbez->x1+(rbez->d * i),rbez->weights+(rbez->d * i),newu);
	pt[2]=ratbez(rbez->d,rbez->x2+(rbez->d * i),rbez->weights+(rbez->d * i),newu);
	pt[3]=ratbez(rbez->d,rbez->x3+(rbez->d * i),rbez->weights+(rbez->d * i),newu);
	pt[4]=ratbez(rbez->d,rbez->x4+(rbez->d * i),rbez->weights+(rbez->d * i),newu);
}

void degree_elevate_bez3d (bez_3d *oldbez, bez_3d *newbez)
{
	/* elevate degree of OLDBEZ by 1, putting result in NEWBEZ */

	int i,j,n,newoff,oldoff;
	
	newbez->d = oldbez->d+1;
	n = newbez->d;
	newbez->L = oldbez->L;
	for (i=0;i<=newbez->L;i++) {
		newbez->knots[i] = oldbez->knots[i];
	}
	for (i=0;i<newbez->L;i++) {
		/* ith segment */
		newoff = i*newbez->d;
		oldoff = i*oldbez->d;
		newbez->x1[newoff] = oldbez->x1[oldoff]; /* special at beginning */
		newbez->x2[newoff] = oldbez->x2[oldoff];
		newbez->x3[newoff] = oldbez->x3[oldoff];
		for (j=1;j<n;j++) {
			newbez->x1[newoff + j] = (j*oldbez->x1[oldoff + j-1]
					   + (n-j)*oldbez->x1[oldoff + j])/n;
			newbez->x2[newoff + j] = (j*oldbez->x2[oldoff + j-1]
					   + (n-j)*oldbez->x2[oldoff + j])/n;
			newbez->x3[newoff + j] = (j*oldbez->x3[oldoff + j-1]
					   + (n-j)*oldbez->x3[oldoff + j])/n;
		}
	}
	/* special at end */
	newbez->x1[newbez->d*newbez->L] = oldbez->x1[oldbez->d*oldbez->L];
	newbez->x2[newbez->d*newbez->L] = oldbez->x2[oldbez->d*oldbez->L];
	newbez->x3[newbez->d*newbez->L] = oldbez->x3[oldbez->d*oldbez->L];
}

void print_bspl3d (const bspl_3d *bspl)
{
	int i;
	printf("A Bspline of degree %i:\n",bspl->d);
	printf("knots = (");
	for (i=0;i<=bspl->L + 2*bspl->d - 2; i++)
		printf("%.2f,",bspl->knots[i]);
	printf(")\ncontrol points:\n");
	for (i=0;i<=bspl->L + 2; i++)
		printf("(%.2f,%.2f,%.2f)\n",bspl->x1[i],
			bspl->x2[i],bspl->x3[i]);
}

void print_bspl (const bspl_4d *bspl)
{
	int i;
	printf("A Bspline of degree %i:\n",bspl->d);
	printf("knots = (");
	for (i=0;i<=bspl->L + 2*bspl->d - 2; i++)
		printf("%.2f,",bspl->knots[i]);
	printf(")\ncontrol points:\n");
	for (i=0;i<=bspl->L + 2; i++)
		printf("(%.2f,%.2f,%.2f,%.2f)\n",bspl->x1[i],
			bspl->x2[i],bspl->x3[i],bspl->x4[i]);
}

void print_bez_1d (const bez_1d *bez)
{
	int i;
	printf("A Bezier curve of degree %i:\n", bez->d);
	printf("knots = (");
	for (i=0;i<=bez->L; i++)
		printf("%.2f,",bez->knots[i]);
	printf(")\ncontrol points:\n");
	for (i=0;i<=bez->d * bez->L; i++)
		printf("%f ",bez->x[i]);
	printf("\n");
}

void print_bez_3d (const bez_3d *bez)
{
	int i;
	printf("A Bezier curve of degree %i:\n", bez->d);
	printf("knots = (");
	for (i=0;i<=bez->L; i++)
		printf("%.2f,",bez->knots[i]);
	printf(")\ncontrol points:\n");
	for (i=0;i<=bez->d * bez->L; i++)
		printf("(%f,%f,%f)\n",bez->x1[i],
			bez->x2[i],bez->x3[i]);
}

void print_bez (const bez_4d *bez)
{
	int i;
	printf("A Bezier curve of degree %i:\n", bez->d);
	printf("knots = (");
	for (i=0;i<=bez->L; i++)
		printf("%.2f,",bez->knots[i]);
	printf(")\ncontrol points:\n");
	for (i=0;i<=bez->d * bez->L; i++)
		printf("(%f,%f,%f,%f)\n",bez->x1[i],
			bez->x2[i],bez->x3[i],bez->x4[i]);
}

void print_ratbez3d (const ratbez_3d *ratbez)
{
	int i;
	printf("A rational Bezier curve of degree %i:\n", ratbez->d);
	printf("knots = (");
	for (i=0;i<=ratbez->L; i++)
		printf("%.2f,",ratbez->knots[i]);
	printf(")\nweights and control points:\n");
	for (i=0;i<=ratbez->d * ratbez->L; i++)
		printf("(%f,%f,%f,%f)\n",ratbez->weights[i],ratbez->x1[i],
			ratbez->x2[i],ratbez->x3[i]);
}

void print_ratbez (const ratbez_4d *ratbez)
{
	int i;
	printf("A rational Bezier curve of degree %i:\n", ratbez->d);
	printf("knots = (");
	for (i=0;i<=ratbez->L; i++)
		printf("%.2f,",ratbez->knots[i]);
	printf(")\nweights and control points:\n");
	for (i=0;i<=ratbez->d * ratbez->L; i++)
		printf("(%f,%f,%f,%f,%f)\n",ratbez->weights[i],ratbez->x1[i],
			ratbez->x2[i],ratbez->x3[i],ratbez->x4[i]);
}

void input_bez3d(bez_3d *bez, FILE *fp)
{
	int i;
	fscanf(fp,"%i %i",&(bez->d),&(bez->L));
	for (i=0;i<=bez->L;i++) {
		fscanf(fp,"%f",(bez->knots)+i);
	}
	for (i=0;i<=(bez->d)*(bez->L);i++) {
		fscanf(fp,"%f",(bez->x1)+i);
		fscanf(fp,"%f",(bez->x2)+i);
		fscanf(fp,"%f",(bez->x3)+i);
	}
}

void input_bez4d(bez_4d *bez, FILE *fp)
{
	int i;
	fscanf(fp,"%i %i",&(bez->d),&(bez->L));
	for (i=0;i<=bez->L;i++) {
		fscanf(fp,"%f",(bez->knots)+i);
	}
	for (i=0;i<=(bez->d)*(bez->L);i++) {
		fscanf(fp,"%f",(bez->x1)+i);
		fscanf(fp,"%f",(bez->x2)+i);
		fscanf(fp,"%f",(bez->x3)+i);
		fscanf(fp,"%f",(bez->x4)+i);
	}
}

void input_ratbez3d(ratbez_3d *ratbez, FILE *fp)
{
	int i;
	fscanf(fp,"%i %i",&(ratbez->d),&(ratbez->L));
	for (i=0;i<=ratbez->L;i++) {
		fscanf(fp,"%f",(ratbez->knots)+i);
	}
	for (i=0;i<=(ratbez->d)*(ratbez->L);i++) {
		fscanf(fp,"%f",(ratbez->weights)+i);
		fscanf(fp,"%f",(ratbez->x1)+i);
		fscanf(fp,"%f",(ratbez->x2)+i);
		fscanf(fp,"%f",(ratbez->x3)+i);
	}
}

void input_ratbez4d(ratbez_4d *ratbez, FILE *fp)
{
	int i;
	fscanf(fp,"%i %i",&(ratbez->d),&(ratbez->L));
	for (i=0;i<=ratbez->L;i++) {
		fscanf(fp,"%f",(ratbez->knots)+i);
	}
	for (i=0;i<=(ratbez->d)*(ratbez->L);i++) {
		fscanf(fp,"%f",(ratbez->weights)+i);
		fscanf(fp,"%f",(ratbez->x1)+i);
		fscanf(fp,"%f",(ratbez->x2)+i);
		fscanf(fp,"%f",(ratbez->x3)+i);
		fscanf(fp,"%f",(ratbez->x4)+i);
	}
}

void output_bez3d(bez_3d *bez, FILE *fp)
{
	int i;
	fprintf(fp,"%i %i\n", bez->d, bez->L);
	for (i=0;i<=bez->L;i++) {
		fprintf(fp,"%f ",bez->knots[i]);
	}
	fprintf(fp,"\n \n");
	for (i=0;i<=(bez->d)*(bez->L);i++) {
		fprintf(fp,"%f ",bez->x1[i]);
		fprintf(fp,"%f ",bez->x2[i]);
		fprintf(fp,"%f ",bez->x3[i]);
	}
}

void output_bez4d(bez_4d *bez, FILE *fp)
{
	int i;
	fprintf(fp,"%i %i\n", bez->d, bez->L);
	for (i=0;i<=bez->L;i++) {
		fprintf(fp,"%f ",bez->knots[i]);
	}
	fprintf(fp,"\n \n");
	for (i=0;i<=(bez->d)*(bez->L);i++) {
		fprintf(fp,"%f ",bez->x1[i]);
		fprintf(fp,"%f ",bez->x2[i]);
		fprintf(fp,"%f ",bez->x3[i]);
		fprintf(fp,"%f \n",bez->x4[i]);
	}
}

void output_ratbez4d(ratbez_4d *ratbez, FILE *fp)
{
	int i;
	fprintf(fp,"%i %i\n", ratbez->d, ratbez->L);
	for (i=0;i<=ratbez->L;i++) {
		fprintf(fp,"%f ",ratbez->knots[i]);
	}
	fprintf(fp,"\n \n");
	for (i=0;i<=(ratbez->d)*(ratbez->L);i++) {
		fprintf(fp,"%f ",ratbez->weights[i]);
		fprintf(fp,"%f ",ratbez->x1[i]);
		fprintf(fp,"%f ",ratbez->x2[i]);
		fprintf(fp,"%f ",ratbez->x3[i]);
		fprintf(fp,"%f \n",ratbez->x4[i]);
	}
}

void prepare_draw_bspl_in_3d (const bspl_3d *bspl, 
			      REAL display_bspl[3][MAXDISPLAYPTS],
			      int *point_num)
{
	REAL display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS];
	int i;

	bspl_to_points(bspl->d, bspl->L, bspl->x1, bspl->knots, BSPL_DENSITY,
		       display_x1, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x2, bspl->knots, BSPL_DENSITY,
		       display_x2, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x3, bspl->knots, BSPL_DENSITY,
		       display_x3, point_num);
	for (i=0;i<*point_num;i++) {	/* collapse */
		display_bspl[0][i] = display_x1[i];
		display_bspl[1][i] = display_x2[i];
		display_bspl[2][i] = display_x3[i];
	}
}

void prepare_draw_bspl_in_4d (const bspl_4d *bspl, 
			      REAL display_bspl[4][MAXDISPLAYPTS],
			      int *point_num)
{
	REAL display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS], display_x4[MAXDISPLAYPTS];
	int i;

	bspl_to_points(bspl->d, bspl->L, bspl->x1, bspl->knots, BSPL_DENSITY,
		       display_x1, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x2, bspl->knots, BSPL_DENSITY,
		       display_x2, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x3, bspl->knots, BSPL_DENSITY,
		       display_x3, point_num);
	bspl_to_points(bspl->d, bspl->L, bspl->x4, bspl->knots, BSPL_DENSITY,
		       display_x4, point_num);
	for (i=0;i<*point_num;i++) {	/* collapse */
		display_bspl[0][i] = display_x1[i];
		display_bspl[1][i] = display_x2[i];
		display_bspl[2][i] = display_x3[i];
		display_bspl[3][i] = display_x4[i];
	}
}

void prepare_draw_bez_in_3d (const bez_3d *bez, 
			     REAL display_bez[3][MAXDISPLAYPTS],
			     int *point_num)
{
	/* prepare to draw Bezier curve in 3-space */
	/* by generating POINT_NUM points on curve, in DISPLAY_BEZ */

	REAL display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS];
	int i;
	REAL v[3];
	
	
	for (i=0;i<bez->L;i++) {	/* for ith segment of Bezier spline */
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x1+(bez->d * i), 
 			 display_x1+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x2+(bez->d * i), 
			 display_x2+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x3+(bez->d * i),
			 display_x3+(BEZ_DENSITY * i));
	}

	/* collapse */
	*point_num = (bez->L * BEZ_DENSITY) + 1; /*each segment has BEZ_DENSITY+1
			points but the last one in ith segment = first one in 
			i+1st segment */
	for (i=0;i<*point_num;i++) {
		display_bez[0][i]=display_x1[i];
		display_bez[1][i]=display_x2[i];
		display_bez[2][i]=display_x3[i];
	}
}

void prepare_draw_bez_in_4d (const bez_4d *bez, 
			     REAL display_bez[4][MAXDISPLAYPTS],
			     int *point_num)
{
	/* prepare to draw Bezier curve in 4-space */
	/* by generating POINT_NUM points on curve, in DISPLAY_BEZ */

	REAL display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS], display_x4[MAXDISPLAYPTS];
	int i;
	REAL v[3];
	
	
	for (i=0;i<bez->L;i++) {	/* for ith segment of Bezier spline */
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x1+(bez->d * i), 
 			 display_x1+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x2+(bez->d * i), 
			 display_x2+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x3+(bez->d * i),
			 display_x3+(BEZ_DENSITY * i));
	   bez_to_points(bez->d, BEZ_DENSITY, bez->x4+(bez->d * i),
			 display_x4+(BEZ_DENSITY * i));
	}

	/* collapse */
	*point_num = (bez->L * BEZ_DENSITY) + 1; /*each segment has BEZ_DENSITY+1
			points but the last one in ith segment = first one in 
			i+1st segment */
	for (i=0;i<*point_num;i++) {
		display_bez[0][i]=display_x1[i];
		display_bez[1][i]=display_x2[i];
		display_bez[2][i]=display_x3[i];
		display_bez[3][i]=display_x4[i];
	}
}

void prepare_draw_ratbez_in_3d (const ratbez_3d *ratbez, 
			        REAL display_ratbez[3][MAXDISPLAYPTS],
			        int *point_num)
{
	/* prepare to draw rational Bezier curve in 3-space */
	/* by generating POINT_NUM points on curve, in DISPLAY_RATBEZ */

	REAL display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS];
	int i;
	REAL v[3];
	
	for (i=0;i<ratbez->L;i++) {	/* for ith segment of Bezier spline */
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x1+(ratbez->d * i),
			ratbez->weights+(ratbez->d * i),
 			display_x1+(BEZ_DENSITY * i));
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x2+(ratbez->d * i), 
			ratbez->weights+(ratbez->d * i),
			display_x2+(BEZ_DENSITY * i));
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x3+(ratbez->d * i),
			ratbez->weights+(ratbez->d * i),
			display_x3+(BEZ_DENSITY * i));
	}

	/* collapse */
	*point_num = (ratbez->L * BEZ_DENSITY) + 1; 
			/*each segment has BEZ_DENSITY+1
			points but the last one in ith segment = first one in 
			i+1st segment */
	for (i=0;i<*point_num;i++) {
		display_ratbez[0][i]=display_x1[i];
		display_ratbez[1][i]=display_x2[i];
		display_ratbez[2][i]=display_x3[i];
	}
}

void prepare_draw_ratbez_in_4d (const ratbez_4d *ratbez, 
			        REAL display_ratbez[4][MAXDISPLAYPTS],
			        int *point_num)
{
	/* prepare to draw rational Bezier curve in 4-space */
	/* by generating POINT_NUM points on curve, in DISPLAY_RATBEZ */

	REAL display_x1[MAXDISPLAYPTS], display_x2[MAXDISPLAYPTS],
	       display_x3[MAXDISPLAYPTS], display_x4[MAXDISPLAYPTS];
	int i;
	REAL v[3];
	
	
	for (i=0;i<ratbez->L;i++) {	/* for ith segment of Bezier spline */
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x1+(ratbez->d * i),
			ratbez->weights+(ratbez->d * i),
 			display_x1+(BEZ_DENSITY * i));
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x2+(ratbez->d * i), 
			ratbez->weights+(ratbez->d * i),
			display_x2+(BEZ_DENSITY * i));
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x3+(ratbez->d * i),
			ratbez->weights+(ratbez->d * i),
			display_x3+(BEZ_DENSITY * i));
	   ratbez_to_points(ratbez->d, BEZ_DENSITY, ratbez->x4+(ratbez->d * i),
			ratbez->weights+(ratbez->d * i),
			display_x4+(BEZ_DENSITY * i));
	}

	/* collapse */
	*point_num = (ratbez->L * BEZ_DENSITY) + 1; /*each segment has BEZ_DENSITY+1
			points but the last one in ith segment = first one in 
			i+1st segment */
	for (i=0;i<*point_num;i++) {
		display_ratbez[0][i]=display_x1[i];
		display_ratbez[1][i]=display_x2[i];
		display_ratbez[2][i]=display_x3[i];
		display_ratbez[3][i]=display_x4[i];
	}
}

void draw_bspl_control_in_4d (const bspl_4d *bspl)
{
	/* draw control polygon */
	/* either project onto $x_2=0$ or perspective project */
	/* we do the former for simplicity at the present */

	int i;
	REAL v[3];
	bgnline();
	for (i=0; i<=bspl->L+2; i++) {
		v[0]=bspl->x1[i];
		v[1]=bspl->x3[i];
		v[2]=bspl->x4[i];
		v3f(v);
	}
	endline();
}

void draw_bez_control_on_4d_sphere (const bez_4d *bez)
{
        int i,n;
        REAL v[3],dist;

	n=bez->d * bez->L;
        bgnline();
        for (i=0; i<=n; i++) {
		if (i%(bez->d)==0) {
			/* this control point should lie on sphere */
			dist = 	sqrt(pow(bez->x1[i],2)+
				     pow(bez->x2[i],2)+
				     pow(bez->x3[i],2)+
				     pow(bez->x4[i],2));
			if (fabs(dist-1.) > .1)
				printf("control point (%.2f,%.2f,%.2f,%.2f) is %f from sphere\n",
				bez->x1[i],bez->x2[i],bez->x3[i],bez->x4[i],
				dist-1.);
/*			else printf("control point (%.2f,%.2f,%.2f,%.2f) is OK\n",
*				bez->x1[i],bez->x2[i],bez->x3[i],bez->x4[i]);
*/
		}
                v[0]=bez->x1[i];
                v[1]=bez->x2[i];
                v[2]=bez->x4[i];
                v3f(v);
        }
        endline();
}

void draw_ratbez_control_on_4d_sphere (const ratbez_4d *ratbez)
{
        int i,n;
        REAL v[3],dist;

	n=ratbez->d * ratbez->L;
	setlinestyle(1);
        bgnline();
        for (i=0; i<=n; i++) {
                v[0]=ratbez->x1[i];
                v[1]=ratbez->x2[i];
                v[2]=ratbez->x4[i];
                v3f(v);
        }
        endline();
	setlinestyle(0);
}

void draw_bez_control_in_4d (const bez_4d *bez)
{
        int i,n;
        REAL v[3];
	n=bez->d * bez->L;
        bgnline();
        for (i=0; i<=n; i++) {
                v[0]=bez->x1[i];
                v[1]=bez->x3[i];
                v[2]=bez->x4[i];
                v3f(v);
        }
        endline();
}

void draw_curve_in_3d(REAL display[3][MAXDISPLAYPTS], const int point_num)
{
	int i;
	REAL v[3];

	/* draw a Bezier or B-spline curve in 3-space */

	bgnline();
	for (i=0; i<point_num ;i++) {
		v[0]=display[0][i];
		v[1]=display[1][i];
		v[2]=display[2][i];
		v3f(v);
	}
	endline();
}

void draw_curve_in_4d(REAL display[4][MAXDISPLAYPTS], const int point_num)
{
	int i;
	REAL v[3];

	/* draw a Bezier or B-spline curve in 4-space */
	/* either project onto $x_2=0$ or perspective project */
	/* we do the former for simplicity at the present */

	setlinestyle(1); /* dashed */
	bgnline();
	for (i=0; i<point_num ;i++) {
		v[0]=display[0][i];
		v[1]=display[2][i];
		v[2]=display[3][i];
		v3f(v);
	}
	endline();
	setlinestyle(0); /* back to solid */
}

void draw_curve_on_4d_sphere (REAL display[4][MAXDISPLAYPTS],
			      const int point_num)
{
	/* visualize curve on sphere by projecting onto $x_3=0$ */

	int i;
	REAL v[3],dist;

	bgnline();
	for (i=0; i<point_num; i++) {
		/* check that point is indeed on sphere */
		dist = 	sqrt(pow(display[0][i],2)+
		       	pow(display[1][i],2)+
			pow(display[2][i],2)+
			pow(display[3][i],2));
		if (fabs(dist-1.) > .1)
			printf("(%.2f,%.2f,%.2f,%.2f) is %f from sphere\n",
				display[0][i],display[1][i],display[2][i],
				display[3][i],dist-1.);
/*		else printf("(%.2f,%.2f,%.2f,%.2f) is OK\n",
*				display[0][i],display[1][i],display[2][i],
*				display[3][i]);
*/
		v[0] = display[0][i];
		v[1] = display[1][i];
		v[2] = display[3][i];
		v3f(v);
	}
	endline();
}

/*
* void drawObject(const wireframe obj, const V3d pos, const Qion q)
* {
*/
	/* draw animated rigid object OBJ at POS in orientation Q */
	/* stub */
	/* wireframe of cube would be good starting place */

/*
* }
*/



