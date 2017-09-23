#pragma once
#include "Types.h"

class Geometry
{
 public:
  Geometry(){

  };
  Geometry(Verticies V, Faces F);
  Verticies V;
  Faces F;

 public:
  void add(const Geometry g);

  // addFaces : add faces to geometry
  // F        : faces to add
  void addFaces(const Faces F);

  // addVerts : add verticies to geometry
  // F        : verticies to add
  void addVerts(const Verticies V);

  // removeFace : remove face from geometry
  // index      : index of face to remove
  bool removeFace(const unsigned int index);
};
