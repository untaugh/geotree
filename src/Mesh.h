#pragma once
#include <Eigen/Core>
#include <vector>
#include "Segment.h"
#include "Face.h"

using namespace Eigen;

namespace Geotree
{
  class Mesh
  {
  public:
    Mesh(Matrix<double, Dynamic, 3> V, Matrix<int, Dynamic, 3> F);
    Mesh();

    Matrix<double, Dynamic, 3> V;
    Matrix<int, Dynamic, 3> F;

    Mesh operator +(const Mesh mesh);
    void translate(const Vector3d v);
    void getSegments(std::vector <Segment> &segments) const;
    Face getFace(int i) const;
  };

  std::ostream& operator<< (std::ostream& stream, const Mesh& mesh);
}
