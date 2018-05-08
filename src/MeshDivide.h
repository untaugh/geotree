#pragma once
#include <Eigen/Core>
#include "IntersectionPoints.h"

using namespace Eigen;

namespace Geotree
{
  class MeshDivide
  {
  public:
    MeshDivide(const IntersectionPoints &points);

    const IntersectionPoints &points;

    Matrix<double, Dynamic, 3> V;
    Matrix<int, Dynamic, 3> F0a;
    Matrix<int, Dynamic, 3> F0b;
    Matrix<int, Dynamic, 3> F1a;
    Matrix<int, Dynamic, 3> F1b;

    void divide();
  };
}
