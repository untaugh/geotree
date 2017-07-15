#pragma once
#include <Eigen/Core>

using namespace Eigen;

class Geometry
{
 public:
  Geometry(MatrixXd V, MatrixXi F);
  MatrixXd V;
  MatrixXi F;

 public:
  bool intersect(MatrixXd * f, Vector3d * v1, Vector3d * v2, Vector3d * r);
  int add(Geometry *g);
};


namespace Calc
{
  // Given triangle and points, divide triangle into smaller triangles. 
  // Face : three verticies forming a face
  // P    : list of 1 to n points
  // V    : resulting verticies
  // F    : resulting faces
  bool divide(MatrixXd *Face, MatrixXd *P, MatrixXd *V, MatrixXi *F);
}
