#pragma once
#include <Eigen/Core>

using namespace Eigen;

typedef enum INTERSECT_TYPE {
  INTERSECT_EQUAL,
  INTERSECT_SIDE,
  INTERSECT_EDGE,
  INTERSECT_INSIDE
} INTERSECT_TYPE;

class Geometry
{
 public:
  Geometry(MatrixXd V, MatrixXi F);
  MatrixXd V;
  MatrixXi F;

 public:
  INTERSECT_TYPE intersect(MatrixXd * m1, MatrixXd * m2, Vector3d * v);
  bool intersect(MatrixXd * m1, MatrixXd * m2, MatrixXd * mr);
  bool intersect(MatrixXd * f, Vector3d * v1, Vector3d * v2, Vector3d * r);
  bool equal(Vector3d v1, Vector3d v2);
  bool equal(Vector3d v1, Vector3d v2, Vector3d w1, Vector3d w2);
  //  bool fequal(Matrix<double,3,3> f1, Matrix<double,3,3> f2);
  template <typename M> bool fequal(M, M);
  //template bool fequal(Matrix<double,3,3>, Matrix<double,3,3>);
};
