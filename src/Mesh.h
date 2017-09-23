#pragma once
#include "Types.h"

namespace Geotree
{
class Mesh
{
 public:
  Verticies V;
  Faces F;
  unsigned size();

  //Mesh operator +(const Mesh mesh);
  Mesh operator +(const Mesh mesh);
  void translate(const Vertex v);
};
}
