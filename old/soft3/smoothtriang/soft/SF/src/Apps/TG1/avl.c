/*
 * Copyright (c) 1990, Graphics and AI Laboratory, University of Washington
 * Copying, use and development for non-commercial purposes permitted.
 *                  All rights for commercial use reserved.
 */

/*
 *---------------------------------------------------------------------
 *  File:  avl.c
 *	An attempt at making generic avl routines.
 *---------------------------------------------------------------------
 */
#include "stdio.h"
#include "avl.h"

static void *recFind();
static recDestroy(    AVL *t);
static insertAvl(    AVL *node,    int (*func)(),     void *data);
static void *recFind(     AVL node,     int (*func)(),     void *data);
static balanced(     AVL t);
static maxSize(     AVL t1, AVL t2);
static int depth(    AVL t);
static leftHeavy(    AVL *t);
static rightHeavy(    AVL *t);
static singleRotateLeft(    AVL *t);
static doubleRotateLeft(    AVL *t);
static singleRotateRight(    AVL *t);
static doubleRotateRight(    AVL *t);
static recPrint(     AVL t,     int (*print)(),     int depth);



/*
 *---------------------------------------------------------------------
 *  Function:  create
 *	Create a new avl tree.
 *  Parameters:
 *	int (*func)()	- a comparison function.  This function
 *			  should take two arguments of the type
 *			  in the avl tree.  This function should
 *			  return -1 if the first is greater than
 *			  the second, 0 if they are equal, and 1
 *			  if the second is greater than the first.
 *	int (*print)()	- a function to print a node in the tree
 *  Return Value:
 *	AVL_TREE	- the avl new avl tree
 *---------------------------------------------------------------------
 */
AVL_TREE create( int (*func)(), int (*print)() )
  {
    AVL_TREE avl;

    avl = (AVL_TREE) malloc ( sizeof( struct avl_tree ) );
    if ( avl == NULL ) {
	fprintf(stderr,"AVL create(): out of memory. Exiting\n");
	exit(1);
    }
    avl->func = func;
    avl->print = print;
    avl->tree = NULL;
    return(avl);
  }

/*
 *---------------------------------------------------------------------
 *  Function:  destroy
 *	This function destroys an already created avl tree.
 *  Parameters:
 *	AVL_TREE *t	- the avl tree to destroy
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
void destroy( AVL_TREE *t )
  {
    recDestroy( &(*t)->tree);
    free( (*t) );
  }

/*
 *---------------------------------------------------------------------
 *  Function:  recDestroy
 *	This function recursively destroys all the nodes in the
 *	avl tree.
 *	It doesn't deallocate the space of the data for the node.
 *	This is a bug that should be fixed.
 *  Parameters:
 *	AVL *t		- a node to destroy
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
static recDestroy( AVL *t )
  {
    if ( *t == NULL ) return;
    recDestroy( &((*t)->left) );
    recDestroy( &((*t)->right) );
    free( (*t) );
  }

/*
 *---------------------------------------------------------------------
 *  Function:  insert
 *	Insert a new value into an avl tree.  This merely
 *	calls a recursive function to do the insert.
 *  Parameters:
 *	AVL_TREE avl	- the avl tree
 *	void *data	- the element to be inserted
 *  Return Value:
 *	void
 *---------------------------------------------------------------------
 */
void insert( AVL_TREE avl, void *data )
  {
    insertAvl( &(avl->tree), avl->func, data);
  }

AVL NewAVLNode()
{
  AVL a;
  a = (AVL) malloc ( sizeof( struct avl ) );
  if ( a == NULL ) {
    fprintf(stderr,"NewAVNode(): out of memory.  Exitin.\n");
    exit(1);
  }
  return a;
  /*return &avlNodes[currentAvlNode++];*/
}

/*
 *---------------------------------------------------------------------
 *  Function:  insertAvl
 *	Recursively find the spot to put a new element of data
 *	in the avl tree.  
 *	On return from the recursion, the avl tree is rebalanced
 *	(if needed).
 *  Parameters:
 *	AVL *node	- current location in recursion
 *	int (*func)()	- the comparison function
 *	void *data	- the new element to be inserted
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
static insertAvl( AVL *node, int (*func)(), void *data )
  {
    if ( *node == NULL )
      {
	*node = NewAVLNode();
	(*node)->data = data;
	(*node)->left = NULL;
	(*node)->right = NULL;
	(*node)->size = 1;
      }
    else
      {
	switch ( (*func)( data, (*node)->data ) )
	  {
	    case -1: insertAvl(  &((*node)->left), func, data  );
			 break;
	    case  0: 
		     return;
	    case  1: insertAvl( &((*node)->right), func, data );
			 break;
	    default:
	      fprintf(stderr,"insert: comparitor function returned value %d!.\n");
	      break;
	  }
	switch ( balanced(*node) )
	  {
	    case LEFT: leftHeavy( node );
			break;
	    case BALANCED: break;
	    case RIGHT: rightHeavy( node );
			break;
	  }
	(*node)->size = maxSize( (*node)->left, (*node)->right ) + 1;
      }
  }

/*
 *---------------------------------------------------------------------
 *  Function:  find
 *	Find a value in an avl tree.
 *	NULL is returned if the data is not found.
 *	The comparision function used to build the tree is used
 *	to find the data.
 *	This function merely calls a recursive routine to find
 *	the data.  A better idea would be to pass in the comparison
 *	routine to do the find with.
 *  Parameters:
 *	AVL_TREE	tree	- the node to find the data in
 *	void 		*data	- the data to find.
 *  Return Value:
 *	void *			- the data found (NULL if not in
 *				  tree).
 *---------------------------------------------------------------------
 */
void *find( AVL_TREE tree, void *data )
  {
    return( recFind( tree->tree, tree->func, data ) );
  }

/*
 *---------------------------------------------------------------------
 *  Function:  recFind
 *	Recursively find some data in an avl subtree.
 *  Parameters:
 *	AVL	node		- the subtree to search
 *	int	(*func)()	- the function to say when the data
 *				  has been found.
 *	void	*data		- the data to find
 *  Return Value:
 *	void *		- the data found (NULL if not found).
 *---------------------------------------------------------------------
 */
static void *recFind( AVL node, int (*func)(), void *data )
  {
    if ( node == NULL ) return( NULL );
    switch( (*func)( data, node->data ) )
      {
	case -1: return( recFind( node->left, func, data ) );
		 break;
	case  0: return( node->data );
		 break;
	case  1: return( recFind( node->right, func, data ) );
		 break;
      }
  }

/*
 *---------------------------------------------------------------------
 *  Function:  balanced
 *	Return the balance of an avl subtree.
 *  Parameters:
 *	AVL t	- the subtree
 *  Return Value:
 *	int	- LEFT if it is left heavy,
 *		  RIGHT if it is right heavy,
 *		  BALANCED if it is balanced
 *---------------------------------------------------------------------
 */
static balanced( AVL t )
  {
    int left, right;
    int diff;

    left = depth(t->left);
    right = depth(t->right);
    
    diff = left - right;

    if ( diff > 1 ) return LEFT;
    else if ( diff < -1 ) return RIGHT;
    else return BALANCED;
  }

/*
 *---------------------------------------------------------------------
 *  Function:  maxSize
 *	Return the maximum of the depths of two avl sub trees
 *  Parameters:
 *	AVL t1,t2	- the sub trees
 *  Return Value:
 *	int		- the depth of the deeper of the two subtrees.
 *---------------------------------------------------------------------
 */
static maxSize( AVL t1, AVL t2 )
  {
    int a,b;

    a = depth(t1);
    b = depth(t2);

    if ( a>b ) return a;
    else return b;
  }

/*
 *---------------------------------------------------------------------
 *  Function:  depth
 *	Return the depth of an avl subtree.
 *	If the tree is NULL, 0 is returned, otherwise the depth
 *	is stored at the node.
 *  Parameters:
 *	AVL	t	- the avl subtree
 *  Return Value:
 *	int		- the depth of the subtree
 *---------------------------------------------------------------------
 */
static int depth( AVL t )
  {
    if ( t == NULL ) return(0);
    return( t->size );
  }

/*
 *---------------------------------------------------------------------
 *  Function:  leftHeavy
 *	Balance a left heavy avl subtree
 *  Parameters:
 *	AVL *t		- a left heavy avl subtree
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
static leftHeavy( AVL *t )
  {
    if ( depth( (*t)->left->left ) > depth( (*t)->left->right ) )
	singleRotateLeft( t );
    else
	doubleRotateLeft( t );
  }

/*
 *---------------------------------------------------------------------
 *  Function:  rightHeavy
 *	Balance a right heavy avl subtree
 *  Parameters:
 *	AVL *t		- a right heavy avl subtree
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
static rightHeavy( AVL *t )
  {
    if ( depth( (*t)->right->right ) > depth( (*t)->right->left ) )
	singleRotateRight( t );
    else
	doubleRotateRight( t );
  }


/*
 *---------------------------------------------------------------------
 *  Function:  singleRotateLeft
 *	Do what is referred to as a "single rotate left" on an
 *	avl subtree.
 *	This does the following transformation:
 *
 *                           
 *          A            B   
 *         /     =>       \  
 *        B                A 
 *                           
 *                           
 *  Parameters:
 *	AVL	*t	- the subtree to rotate
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
static singleRotateLeft(AVL *t)
  {
    AVL temp1;

    temp1 = *t;
    *t = (*t)->left;
    temp1->left = (*t)->right;
    (*t)->right = temp1;
    temp1->size = maxSize( temp1->left, temp1->right ) + 1;
  }

/*
 *---------------------------------------------------------------------
 *  Function:  doubleRotateLeft
 *	Do what is referred to as a "double rotate left" on an
 *	avl subtree.
 *	This does the following transformation:
 *                                                            
 *              C                 B                           
 *             / \               / \                          
 *            A   t4            A   C                         
 *           / \        =>     / \ / \                        
 *          t1  B            t1 t2 t3 t4                     
 *             / \                                            
 *            t2 t3                                           
 *	
 *	Note that the depth of t1 stayed constant, t2 and t3
 *	decreased by 1, and t4 increased by 1.
 *
 *  Parameters:
 *	AVL	*t	- the tree to be rotated
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
static doubleRotateLeft(AVL *t)
  {
    AVL temp1, temp2;

    temp1 = *t;
    temp2 = (*t)->left;
    *t = temp2->right;
    temp2->right = (*t)->left;
    (*t)->left = temp2;
    temp1->left = (*t)->right;
    (*t)->right = temp1;

    temp1->size = maxSize( temp1->left, temp1->right ) + 1;
    temp2->size = maxSize( temp2->left, temp2->right ) + 1;
  }


/*
 *---------------------------------------------------------------------
 *  Function:  singleRotateRight
 *	Do what is referred to as a "single rotate right" on an
 *	avl subtree.
 *	This does the following transformation:
 *
 *                           
 *        A                B 
 *       / \     =>       / \
 *      t1  B            A   t3
 *         / \          / \  
 *        t2 t3        t1 t2 
 *            
 *	Note that the depth of t1 increased by 1, the depth of
 *	t2 didn't change, and the depth of t3 decreased by 1.
 *
 *  Parameters:
 *	AVL	*t	- the subtree to rotate
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
static singleRotateRight(AVL *t)
 {
    AVL temp1;

    temp1 = *t;
    *t = (*t)->right;
    temp1->right = (*t)->left;
    (*t)->left = temp1;
    temp1->size = maxSize( temp1->left, temp1->right ) + 1;
  }

/*
 *---------------------------------------------------------------------
 *  Function:  doubleRotateRight
 *	Do what is referred to as a "double rotate right" on an
 *	avl subtree.
 *	This does the following transformation:
 *                                                            
 *              A                 B                           
 *             / \               / \                          
 *            t1  C             A   C                         
 *               / \    =>     / \ / \                        
 *              B   t4       t1 t2 t3 t4                     
 *             / \                                            
 *            t2 t3                                           
 *	
 *	Note that the depth of t1 stayed increases by 1, t2 and t3
 *	decreased by 1, and t4 stays constant.
 *---------------------------------------------------------------------
 */
static doubleRotateRight(AVL *t)
  {
    AVL temp1, temp2;

    temp1 = *t;
    temp2 = (*t)->right;
    *t = temp2->left;
    temp2->left = (*t)->right;
    (*t)->right = temp2;
    temp1->right = (*t)->left;
    (*t)->left = temp1;
    temp1->size = maxSize( temp1->left, temp1->right ) + 1;
    temp2->size = maxSize( temp2->left, temp2->right ) + 1;
  }

/*
 *---------------------------------------------------------------------
 *  Function:  print
 *	Print an avl tree, using the print function passed in at
 *	creation.
 *	This routine merely calls a recursive print routine.
 *  Parameters:
 *	AVL_TREE	t	- the tree to print
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
void print(AVL_TREE t)
  {
    recPrint( t->tree, t->print, 0 );
    printf("\n");
  }


/*
 *---------------------------------------------------------------------
 *  Function:  recPrint
 *	Recursively print an avl subtree.
 *  Parameters:
 *	AVL	t	- the sub tree to print
 *	int (*print)()	- the print procedure
 *	int 	depth	- the indentation depth (also the depth
 *			  of the subtree).
 *  Return Value:
 *	none
 *---------------------------------------------------------------------
 */
static recPrint(AVL t, int (*print)(), int depth)
  {
    int i;

    if ( t == NULL ) return;
    recPrint( t->right, print, depth+1 );
    for (i=0;i<depth;i++) printf("  ");
    (*print)( t->data );
    printf(": %d",t->size);
    printf("\n");
    recPrint( t->left, print, depth+1 );
  }
