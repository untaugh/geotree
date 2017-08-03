#pragma once
#include <Eigen/Core>

using namespace Eigen;

class Geometry
{
 public:
  Geometry(){};
  Geometry(MatrixXd V, MatrixXi F);
  MatrixXd V;
  MatrixXi F;

 public:
  bool intersect(MatrixXd * f, Vector3d * v1, Vector3d * v2, Vector3d * r);
  int add(Geometry &g);
};
