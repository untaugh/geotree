#pragma once
#include <vector>
#include "Types.h"
#include "Mesh.h"

namespace Geotree
{
  class PathT
  {
  public:
    PathT(Mesh &_mesh) : mesh(_mesh) {};
    void add(const Vector3d point);
    int size();
    Vector3d get(const int index);

    void operator +=(const Vector3d point)
    {
      add(point);
    }

  private:
    Mesh &mesh;
    std::vector <int> points;
  };
}
