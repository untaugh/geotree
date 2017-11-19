#pragma once
#include "Mesh.h"

namespace Geotree
{
  class Mesh;

  class Point
  {
  public:
    Point(const Mesh& _mesh, const int _index) : index(_index), mesh(_mesh) {};
    Vector getVector() const;
    int getIndex() const { return index; };

      Point& operator=(Point& point)
      {
          return point;
      }

      bool operator!=(Point& point)
      {
          return this->index != point.index;
      }

  protected:
    const int index;
    const Mesh &mesh;
  };
}
