#pragma once
#include <Eigen/Core>

using namespace Eigen;

namespace Geotree
{
  struct PathPoint
  {
    Vector3d vector;
    PointType type;
    int index0;
    int index1;
  };

  class Path
  {
  public:
    std::vector <Point> points;
  };
}
