#pragma once
#include <Eigen/Core>
#include <vector>
#include "Segment.h"
#include "Face.h"
#include "Cube.h"

using namespace Eigen;

namespace Geotree
{
  class Mesh
  {
  public:
    Mesh(Matrix<double, Dynamic, 3> V, Matrix<int, Dynamic, 3> F);
    Mesh();
    Mesh(int verts, int faces);

    Matrix<double, Dynamic, 3> V;
    Matrix<int, Dynamic, 3> F;

    Mesh operator +(const Mesh mesh);
    void translate(const Vector3d v);
    void getSegments(std::vector <Segment> &segments) const;
    Face getFace(int i) const;
    void getFaces(std::vector <Face> &faces) const;
    int add(Face face);
    Cube boundingBox() const;
    Cube boundingBox(std::set <int> &faces) const;

    Mesh unionOperation(Mesh &mesh);

    int numFaces() { return F.rows(); };
  };

  std::ostream& operator<< (std::ostream& stream, const Mesh& mesh);
}
