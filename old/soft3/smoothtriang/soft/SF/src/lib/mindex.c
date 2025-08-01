/*
 *----------------------------------------------------------------------
 *  File:  index.c
 *----------------------------------------------------------------------
 */

#include <stdio.h>
#include "mindex.h"

static int initCount=0;
static int freeCount=0;

MI InitMIndex()
{
	MI ind;

	initCount++;
	ind.dim = 0;
	ind.i = (int*)NULL;
	return ind;
}


void AllocIndex(MI* ind, int dim)
{
	int i;

	if ( ind->dim > 0 ) {
		if ( ind->dim < dim ) {
			ind->i = (int*)realloc(ind->i, sizeof(int)*(dim+1));
		}
	} else {
		ind->i = (int*)malloc(sizeof(int)*(dim+1));
	}
	ind->dim = dim;
	for (i=0; i<=dim; i++) {
		ind->i[i] = 0;
	}
}


void FreeMIndex(MI* ind)
{
	freeCount++;
	if ( ind->i != NULL ) {
		free(ind->i);
	}
	ind->dim = 0;
	ind->i = (int*)NULL;
}

void fAllocStats(fp)
FILE* fp;
{
	fprintf(fp, "init called %d time(s), free called %d time(s).\n",
		initCount, freeCount);
}

void IncDim(MI* ind)
{
	ind->dim++;
	ind->i = (int*)realloc(ind->i, sizeof(int)*(ind->dim+1));
}

int IncDeg(MI* ind, int i)
{
	if ( i < 0  ||  i > ind->dim ) {
		return 0;
	} else {
		ind->i[i]++;
	}
}

/*
 *----------------------------------------------------------------------
 *  Function:  Dim
 *----------------------------------------------------------------------
 */
int Dim(int d, int n)
{
	if ( n == -1 ) {
		return 0;
	} else if ( n == 0 ) {
		return 1;
	} else if ( d == 0 ) {
		return 1;
	} else {
		return Dim(d-1, n) + Dim(d, n-1);
	}
}

/*
 *----------------------------------------------------------------------
 *  Function:  SumMIndex
 *      Sum the indices of a multi-index.
 *----------------------------------------------------------------------
 */
int SumMIndex(MI a)
{
	int i;
	int sum;

	for( sum=0, i=0; i<=a.dim; i++ ) {
		sum += a.i[i];
	}
	return sum;
}

IndexOrder indOrd=REVLEX;

void SetMIOrder(IndexOrder io)
{
	indOrd = io;
}

/*
 *----------------------------------------------------------------------
 *  Function: MtoI
 *----------------------------------------------------------------------
 */
int MtoI(MI ind)
{
	switch (indOrd) {
	      case LEX:
		return MtoI3(ind);
	      case REVLEX:
		return MtoI1(ind);
	      case INVREVLEX:
		return MtoI2(ind);
	      default:
		fprintf(stderr,"MtoI: huh?\n");
		exit(1);
	}
}

int MtoI1(MI ind)
{
	int value;
	int sum;
	int i;
	
	sum = SumMIndex(ind);
	value = 0;
	for(i=0; i<ind.dim; i++){
		sum -= ind.i[i];
		value += Dim(ind.dim-i, sum-1);
	}
	return value;
}

int MtoI2(MI ind)
{
	int value;
	int sum;
	int i;
	
	sum = SumMIndex(ind);
	value = 0;
	for(i=ind.dim; i>=1; i--){
		sum -= ind.i[i];
		value += Dim(i, sum-1);
	}
	return value;
}

int MtoI3(MI ind)
{
	int value;
	int sum;
	int i;
	
	sum = SumMIndex(ind);
	value = Dim(sum,ind.dim)-1;
	sum--;
	for (i=0; i<ind.dim; i++ ) {
		sum -= ind.i[i];
		value -= Dim(ind.dim-i, sum);
	}
	return value;
}


/*
 *----------------------------------------------------------------------
 *  Function:  FirstMIndexLEX
 *----------------------------------------------------------------------
 */
static void FirstMIndexLEX(MI* first, int deg, int dim)
{
	int i;

	AllocIndex(first, dim);
	first->i[dim] = deg;
	for(i=0; i<dim; i++) {
		first->i[i] = 0;
	}
}

static void FirstMIndexREVLEX(MI* first, int deg, int dim)
{
	int i;
	
	AllocIndex(first, dim);
	first->i[0] = deg;
	for(i=1; i<=dim; i++) {
		first->i[i] = 0;
	}
}



void FirstMIndex(MI* first, int deg, int dim)
{
	switch(indOrd) {
	      case LEX:
		FirstMIndexLEX(first, deg, dim);
		break;
	      case REVLEX:
		FirstMIndexREVLEX(first, deg, dim);
		break;
	      default:
		FirstMIndexLEX(first, deg, dim);
		break;
	}
}

/*
 *----------------------------------------------------------------------
 *  Function:  NextMIndexLEX
 *      Given an index, make it into the previous index.  Return 1 if this
 *  was done successfully; return 0 if index is max index.
 *----------------------------------------------------------------------
 */
static int NextMIndexLEX(MI* ind)
{
	int i;
	
	for(i=ind->dim; i>=0; i--){
		if ( ind->i[i] != 0 ) {
			break;
		}
	}
	if ( i<=0 ) {
		return 0;
	}
	if ( i == ind->dim ) {
		ind->i[ind->dim]--;
		ind->i[ind->dim-1]++;
	} else {
		ind->i[ind->dim] = ind->i[i]-1;
		ind->i[i] = 0;
		ind->i[i-1]++;
	}
	return 1;
}


static int NextMIndexREVLEX(MI* ind)
{
	int i,j;

	for(i=ind->dim; i>=0; i--){
		if ( ind->i[i] != 0 ) {
			break;
		}
	}
	if ( i==ind->dim ) {
		for (j=i-1; j>=0; j--) {
			if ( ind->i[j] != 0 ) {
				break;
			}
		}
		if ( j < 0 ) {
			return 0;
		} else {
			int v;

			v = ind->i[i]+1;
			ind->i[i] = 0;
			ind->i[j+1] = v;
			ind->i[j]--;
		}
	} else if ( i < 0 ) {
		return 0;
	} else {
		ind->i[i]--;
		ind->i[i+1]++;
	}
	return 1;
}


int NextMIndex(MI* ind)
{
	switch(indOrd) {
	      case LEX:
		return NextMIndexLEX(ind);
	      case REVLEX:
		return NextMIndexREVLEX(ind);
	      default:
		return NextMIndexLEX(ind);
		break;
	}
}


void LastMIndex(MI* first, int deg, int dim)
{
	switch(indOrd) {
	      case LEX:
		FirstMIndexREVLEX(first, deg, dim);
		break;
	      case REVLEX:
		FirstMIndexLEX(first, deg, dim);
		break;
	      default:
		FirstMIndexREVLEX(first, deg, dim);
		break;
	}
}

int PrevMIndex(MI* ind)
{
	switch(indOrd) {
	      case LEX:
		return NextMIndexREVLEX(ind);
	      case REVLEX:
		return NextMIndexLEX(ind);
	      default:
		return NextMIndexREVLEX(ind);
	}
}



int SetIndex(MI* ind, int i, int v)
{
	if ( i > ind->dim ) {
		return 0;
	}
	ind->i[i] = v;
}


int SetMIndex(MI* ind, int v[])
{
	int i;

	for (i=0; i<=ind->dim; i++) {
		ind->i[i] = v[i];
	}
}

static int CompareIndicesLEX(MI i1, MI i2)
{
	int i;

	if ( i1.dim != i2.dim ) {
		fprintf(stderr,"CompareIndices: not of same dimension.\n");
	}

	for (i=0; i<=i1.dim; i++) {
		if ( i1.i[i] < i2.i[i] ) {
			return -1;
		} else if ( i1.i[i] > i2.i[i] ) {
			return 1;
		}
	}

	return 0;
}

static int CompareIndicesREVLEX(MI i1, MI i2)
{
	int i;

	if ( i1.dim != i2.dim ) {
		fprintf(stderr,"CompareIndices: not of same dimension.\n");
	}

	for (i=0; i<=i1.dim; i++) {
		if ( i1.i[i] > i2.i[i] ) {
			return -1;
		} else if ( i1.i[i] < i2.i[i] ) {
			return 1;
		}
	}

	return 0;
}

int CompareIndices(MI i1, MI i2)
{
	switch(indOrd) {
	      case LEX:
		return CompareIndicesLEX(i1, i2);
	      case REVLEX:
		return CompareIndicesREVLEX(i1, i2);
	      default:
		return CompareIndicesLEX(i1, i2);		
	}
}


void CopyMI(MI ind, MI* cind)
{
	int i;

	AllocIndex(cind, ind.dim);
	for (i=0; i<=ind.dim; i++) {
		cind->i[i] = ind.i[i];
	}
}
