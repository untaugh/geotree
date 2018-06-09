#pragma once
#include <Eigen/Core>

using namespace Eigen;

namespace Geotree
{
  struct Cube
  {
    Vector3d p0;
    Vector3d p1;
  };

  std::ostream& operator<< (std::ostream& stream, const Cube& cube);
}
