#pragma once
#include "Mesh.h"

namespace Geotree
{
  class Mesh;

    class Point
    {
    public:
      Point(const Mesh & _mesh, const int _index) : index(_index), mesh(_mesh) {};
      Vector getPoint() const;
    protected:
      const int index;
      const Mesh &mesh;
    };
}
