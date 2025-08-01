#include <stdio.h>
#include <gl/sphere.h>
#include <math.h>
#include "vec.h"
#include "bez.h"
#include "bez.c"

typedef double v4dh[5];
typedef v4dh Qion;

main() {

	Qion q[20];
	
	int i,n;
	double theta;
	double axis[3];
	float params[4];

	/* input n quaternions */
	scanf("%i",&n);
	for (i=0;i<n;i++) {
		scanf("%f",&theta); /* angle of rotation, in degrees */
		theta = (theta)/(6.28);	/* degrees to radians */
		q[i][0] = 1;
		q[i][1] = cos(theta/2);
		printf("%f\n",q[i][1]);

		scanf("%f %f %f",axis,axis+1,axis+2); /* axis of rotation */
		q[i][2] = axis[0];
		q[i][3] = axis[1];
		q[i][4] = axis[2];
	}
	
	printf("%f\n",acos(-1.));

	params[0]=params[1]=params[2]=0;
	params[3]=1;
	sphdraw(params);
}
