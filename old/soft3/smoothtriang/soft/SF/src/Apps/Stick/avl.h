/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *----------------------------------------------------------------------
 *  File:  avl.h
 *----------------------------------------------------------------------
 */

#define LEFT -1
#define BALANCED 0
#define RIGHT 1

typedef struct avl
  {
    int size;
    char *data;
    struct avl *left;
    struct avl *right;
  }  *AVL;


typedef struct avl_tree
  {
    int (*func)();
    int (*print)();
    AVL tree;
  } *AVL_TREE;
