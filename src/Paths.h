#pragma once
#include <vector>
#include <Eigen/Core>
#include "IntersectionPoints.h"

namespace Geotree
{
  class Paths
  {
  public:
  Paths(IntersectionPoints &intersection);
    IntersectionPoints &intersection;
    unsigned int count();
    unsigned int size(int i);
    std::set<int> faces(MeshID mesh);
  private:
    void findPaths();
    Point &nextPoint(Point &current, Point &previous);
  };
}
