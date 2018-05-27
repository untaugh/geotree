#pragma once
#include <Eigen/Core>
#include "Mesh.h"

using namespace Eigen;

namespace Geotree
{
  class MeshDivide
  {
  public:
    MeshDivide(const Mesh mesh0, const Mesh mesh1);

    Matrix<double, Dynamic, 3> V;
    Matrix<int, Dynamic, 3> F0a;
    Matrix<int, Dynamic, 3> F0b;
    Matrix<int, Dynamic, 3> F1a;
    Matrix<int, Dynamic, 3> F1b;

    void divide();
    const Mesh &mesh0;
    const Mesh &mesh1;
  };
}
