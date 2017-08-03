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
  int add(Geometry &g);
};
