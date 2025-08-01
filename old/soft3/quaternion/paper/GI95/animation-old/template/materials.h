
/* YELLOW */

float material1_plastic[] = {  /* define a material properties array        */
     AMBIENT, 0.2, 0.1, 0.1, 
     DIFFUSE, 1.0, 0.0, 0.0,
     SPECULAR, 1.0, 1.0, 1.0,
     SHININESS, 30.0,
     ALPHA, 1.0,
     LMNULL
};

float material2_plastic[] = {/* define a transparent mat. props. array */
     AMBIENT, 0.2, 0.1, 0.1, 
     DIFFUSE, 0.1, 0.8, 0.5,
     SPECULAR, 1.0, 1.0, 1.0,
     SHININESS, 30.0,
     ALPHA, .3,
     LMNULL
};



/* PURPLE */

float material3_plastic[] = {  /* define a material properties array        */
     AMBIENT, 0.2, 0.1, 0.1, 
     DIFFUSE, 0.5, 0.1, 0.8,
     SPECULAR, 1.0, 1.0, 1.0,
     SHININESS, 30.0,
     LMNULL
};

/* MY */

static float mymat[] = {
     ALPHA, 1.0,
     AMBIENT, 0.2, 0.2, 0.2,
     COLORINDEXES, 0, 127.5, 255,
     DIFFUSE, .8, .8, .8,
     EMISSION, 0.0, 0.0, 0.0,
     SPECULAR, 1.0, 1.0, 1.0,
     SHININESS, 0.0,
     LMNULL
};


