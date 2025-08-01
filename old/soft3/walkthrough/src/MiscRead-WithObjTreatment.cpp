 /*
  File: 	 MiscRead.cpp
  Author: 	 J.K. Johnstone 
  Created:	 11 May 2005
  Last Modified: 11 May 2005
  History:	 5/11/05: Added readUnigrafix, readPrincetonOff (from walkthrough software).
 */

#include "basic/Miscellany.h" // tryToGetLeftBrace, skipToMatchingRightBrace, getLeftParen
                              // tryToGetRightParen, getSymbol
#include "basic/Vector.h"     // V3fArr, IntArr, IntArrArr, allocate
#include <fstream.h>          // tellg, seekg, clear
#include <tcl.h>              // Tcl_HashTable, Tcl_InitHashTable, Tcl_HashEntry, 
                              // Tcl_CreateHashEntry, Tcl_SetHashValue



/******************************************************************************
    Parse a model in Berkeley's UniGrafix format (see Teller homepage).

    <--infile:    Unigrafix file
    <--color:     available colors (RGB)
    <--vert:      available vertices
    <--face:      face[i] = vertex indices of ith face
    <--faceColor: faceColor[i] = color index of ith face
******************************************************************************/
void readUnigrafix (ifstream &infile, V3fArr &color, V3fArr &vert, IntArrArr &face, IntArr &faceColor)
{
  string keyword, colorID, vertexID, faceID;
  string semicolon, leftparen, rightparen, nextvertexid;
  V3f rgb, vtx;
  int newPtr, nColor=0, nVert=0, nFace=0;

  //  read it, just to count and allocate

  while (tryToGetLeftBrace(infile))
    skipToMatchingRightBrace(infile);
  int markColor = infile.tellg();
  infile >> keyword;
  while (keyword == "c_rgb") // color
    {
      infile >> colorID >> rgb[0] >> rgb[1] >> rgb[2] >> semicolon >> keyword;
      nColor++;
    }
  while (keyword == "v")     // vertex
    {
      infile >> vertexID >> vtx[0] >> vtx[1] >> vtx[2] >> semicolon >> keyword;
      nVert++;
    }
  int markFace = infile.tellg();
  int done = 0;
  while (keyword == "f" && !done) // face line: f id (v1 v2 ...) (w1 w2 ...) ... colour ;
    {
      infile >> faceID;
      getLeftParen(infile);
      int nvPerFace=0;
      while (!tryToGetRightParen(infile)) { infile >> nextvertexid; nvPerFace++; }
      // are not parsing nested faces: would need 'while (tryToGetLeftParen(infile))'
      infile >> colorID >> semicolon;
      if (!(infile >> keyword)) done = 1;
      nFace++;
    } 
  cout << "(" << nColor << "," << nVert << "," << nFace << ")" << endl;
  color.allocate (nColor);
  vert.allocate (nVert);
  face.allocate (nFace);
  faceColor.allocate (nFace);

  // read it again, for degree of each face
  infile.clear();
  infile.seekg(markFace);
  nFace=0;
  done = 0;
  while (keyword == "f" && !done) // face line: f id (v1 v2 ...) (w1 w2 ...) ... colour ;
    {
      infile >> faceID;
      getSymbol (infile, '(');
      int nvPerFace=0;
      while (!tryToGetRightParen(infile)) { infile >> nextvertexid; nvPerFace++; }
      face[nFace].allocate(nvPerFace);
      infile >> colorID >> semicolon;
      if (!(infile >> keyword)) done = 1;
      nFace++;
    }

  // read it again, for real
  infile.clear();
  infile.seekg(markColor);
  nColor = nVert = nFace = 0;

  Tcl_HashTable colorTable;
  Tcl_HashTable vertTable;
  Tcl_InitHashTable (&colorTable, TCL_STRING_KEYS);
  Tcl_InitHashTable (&vertTable,  TCL_STRING_KEYS);
  Tcl_HashEntry *colorEntry, *vertEntry;
  infile >> keyword;
  while (keyword == "c_rgb")
    {
      infile >> colorID >> rgb[0] >> rgb[1] >> rgb[2] >> semicolon >> keyword;
      color[nColor] = rgb;
      //      printf("Adding %s to the color hash table\n", colorID.c_str());
      colorEntry = Tcl_CreateHashEntry (&colorTable, colorID.c_str(), &newPtr);
      if (newPtr == 1) // new entry
	Tcl_SetHashValue (colorEntry, nColor);
      //      cout << "Color index = " << (int) Tcl_GetHashValue (colorEntry) << endl;
      nColor++;
    }
  while (keyword == "v")
    {
      infile >> vertexID >> vtx[0] >> vtx[1] >> vtx[2] >> semicolon >> keyword;
      vert[nVert] = vtx;
      //      printf("Adding %s to the vertex hash table\n", vertexID.c_str());
      vertEntry = Tcl_CreateHashEntry (&vertTable, vertexID.c_str(), &newPtr);
      if (newPtr == 1)
	Tcl_SetHashValue (vertEntry, nVert);
      //      cout << "Vertex index = " << (int) Tcl_GetHashValue (vertEntry) << endl;
      nVert++;
    }
  done = 0;
  while (keyword == "f" && !done) // face line: f id (v1 v2 ...) (w1 w2 ...) ... colour ;
    {
      infile >> faceID;
      getSymbol (infile, '(');
      for (int i=0; i<face[nFace].getn(); i++)
	{ 
	  infile >> nextvertexid;
	  //	  cout << "Finding vertex " << nextvertexid << endl;
	  vertEntry = Tcl_FindHashEntry (&vertTable, nextvertexid.c_str());
	  face[nFace][i] = (int) Tcl_GetHashValue (vertEntry);
	}
      //      cout << "face[" << nFace << "] = " << face[nFace] << endl;
      getSymbol (infile, ')');
      infile >> colorID >> semicolon;
      //      cout << "Finding color " << colorID << endl;
      colorEntry = Tcl_FindHashEntry (&colorTable, colorID.c_str());
      faceColor[nFace] = (int) Tcl_GetHashValue (colorEntry);
      //      cout << "faceColor[" << nFace << "] = " << faceColor[nFace] << endl;
      if (!(infile >> keyword)) done = 1;
      nFace++;
    } 
}

/******************************************************************************
    Parse a model in Princeton's off format (see Princeton Benchmark website).

    <--infile:    off file
    <--vert:      available vertices
    <--face:      face[i] = vertex indices of ith face
******************************************************************************/

void readPrincetonOff (ifstream &infile, V3fArr &vert, IntArrArr &face)
{
  int i,j;
  string offkeyword;
  int nVert, nFace, nEdge;

  infile >> offkeyword >> nVert >> nFace >> nEdge;
  vert.allocate (nVert);
  face.allocate (nFace);
  for (i=0; i<nVert; i++)
    infile >> vert[i][0] >> vert[i][1] >> vert[i][2];
  for (i=0; i<nFace; i++)
    {
      int nv; // # of vertices in face
      infile >> nv;
      face[i].allocate (nv);
      for (j=0; j<nv; j++)
	infile >> face[i][j];
    }
}

/******************************************************************************
    Parse a model in Object format.

    <--infile:    Object file
    <--vert:      available vertices
    <--vertNorm:  vertex normal
    <--face:      ex: 1234//1234 321//321 543//543  read in as IntArrArr
*******************************************************************************/

void readObject (ifstream &infile, V3fArr &vert, V3fArr &vertNorm, IntArrArr &face)
{
  string keyword; //identifier: v-vertex, vn-vertex normal, vt- vertext texture,f-face
  V3f vtx, vtxNorm;
  V2fArr vertText;//not displaying yet
  V2f vtxText;
  int numV = 0;  //number of vertices per face  
  string fElement; //used to read in faces since there are several different formats
  int nVert=0, nVertNorm=0, nVertText=0, nFace=0;
  int done=0; //test for end of file
  int markVertex=0, markVertNorm=0,markVertText=0, markFace=0; //mark beginning of vtx, vtx norm, and face lists
  
 
  //read for count and allocation

  infile >> keyword;
  while (keyword != "v")//pass through intro to list of vertices
    {
      infile >> keyword;
    }
  markVertex = infile.tellg();
  while (keyword == "v")//vertex 
    {
      infile >> vtx[0] >> vtx[1] >> vtx[2] >> keyword;
      nVert++;
    }
  markVertNorm = infile.tellg();
  while (keyword == "vn")//vertex normal
    {
      infile >> vtxNorm[0] >> vtxNorm[1] >> vtxNorm[2] >> keyword;
      nVertNorm++;
    }
  markVertText = infile.tellg();
  while (keyword == "vt")//vertex texture
    {
      infile >> vtxText[0] >> vtxText[1] >> keyword;
      nVertText++;
     }
  while (keyword != "f")//pass through space and face intro info
    {
      infile >> keyword;
    }
  markFace = infile.tellg();

  while (keyword == "f" && done == 0)//face
    { 
      getline(infile, fElement);
      nFace++;
 
      if (!(infile >> keyword))
      {
	done = 1;
      }
    }
   
  //allocate
  vert.allocate (nVert);
  vertNorm.allocate (nVertNorm);
  vertText.allocate (nVertText);
  face.allocate (nFace);
  int numVert[nFace];//need array to store number of vertices per face
  infile.clear();
  infile.seekg(markFace);
  keyword = "f";
  done = 0;
  int j = 0;
  while(keyword == "f" && done == 0)
   {
   	 numV = 0;
	 getline(infile, fElement); //each line may contain a different number of vertices
	
	 //traverse each string and count spaces, a space always preceeds the vertex
	 for (int i = 0; i < fElement.size(); i++)
	   {
	     if (fElement.substr(i,1) == " ")
	       numV++;
	   }
 
	 face[j].allocate(numV);
	 numVert[j] = numV;//need when reading for real
	 j++;
         if (!(infile >> keyword))
           {
             done = 1;
           }
    }
  
  //read again, for real
  infile.clear();
  infile.seekg(markVertex);
  nVert = nVertNorm = nVertText = 0;

  keyword = "v";

  while (keyword == "v")
    {
      infile >> vtx[0] >> vtx[1] >> vtx[2] >> keyword;
      vert[nVert] = vtx;
      nVert++;
    }

  infile.seekg(markVertNorm);
  while (keyword == "vn")
    {
      infile >> vtxNorm[0] >> vtxNorm[1] >> vtxNorm[2] >> keyword;
      vertNorm[nVertNorm] = vtxNorm;
      nVertNorm++;
    }
  while (keyword == "vt")
    {
      infile >> vtxText[0] >> vtxText[1] >> keyword;
      vertText[nVertText] = vtxText;
      nVertText++;
    }
  infile.seekg(markFace);
  keyword = "f";
  done = 0;
  int length = 0;
  int i = 0;
  string substring;
 
  while (keyword == "f" && done == 0)
    {
      infile>>fElement;
      for (int k = 0; k < numVert[i]; k++)
      //numVert[i] contains number of vertices we need to grab for the ith face
	{
	  length = fElement.size();
	  substring = "";
	  
	  //read one char at a time and stop at /, the vertex texture for each face follows the first /, and the vertex normal for each face follows the second /, right now, I'm only reading the vertices associated with each face
	  for (int n = 0; n < length; n++)
	    {
	      if(fElement.substr(n,1)!="/")
		substring += fElement.substr(n,1);
	      else
		 n=length;
	    }
	   face[i][k] = atoi(substring.c_str())-1;
	   if(k!=numVert[i]-1)//otherwise, we read the keyword
	      infile >> fElement;
         }			   	       
	      i++;
	      if (!(infile >> keyword))
		{
		  done = 1;
		}
	      
    }    
		
   
}
