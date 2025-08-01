/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

#ifndef _DSTRUCT_H_
#define _DSTRUCT_H_

/*
**
** Purpose: Header file for users of dynamic structures.
**
**
*/

typedef enum { UNDEF, NV, STRING, REAL, NVLIST, VLIST } LnodeTag;

typedef struct lnode {		/* Lisp-like nodes */
    LnodeTag tag;		/* Denotes what the node represents */
    char *name;			/* Used w/ NV tags */
    struct lnode *car, *cdr;	/* Used w/ NV and LIST tags */
    char *sval;			/* Used w/ STRING tag */
    double rval;		/* Used w/ REAL tag */
    int position;		/* Used w/ VLIST tag */
} Lnode, *LNODE;

extern Lnode *NewNVPair(char *name, Lnode *value);
extern Lnode *NewNVList(Lnode *list, Lnode *elem);
extern Lnode *NewVList(Lnode *head, Lnode *newValue);
extern Lnode *NewVNode(Lnode *car);
extern Lnode *NewString(char *buf);
extern Lnode *NewReal(double rval);
extern void LnodePrint(FILE *fp, Lnode *lnode, int nl);
	/* set nl to 1 if you want new lines, 0 if not */
extern Lnode *din, *dout;

#define IsAtom(node) ((node->tag == REAL) || (node->tag == STRING))

#ifndef _BOOLEAN
#define _BOOLEAN
typedef int BOOLEAN;
#endif

#ifndef Scalar
typedef	double	Scalar;
#endif

#include "getset.h"

#endif /* _DSTRUCT_H_ */
