#include "Geometry.h"

Geometry::Geometry(MatrixXd V_, MatrixXi F_)
{
  this->V = V_;
  this->F = F_;
}

INTERSECT_TYPE Geometry::intersect(MatrixXd * m1, MatrixXd * m2, Vector3d * v)
{

  
  
  return INTERSECT_EQUAL;
}

bool Geometry::intersect(MatrixXd * m1, MatrixXd * m2, MatrixXd * mr)
{
  return true;
}

bool Geometry::intersect(MatrixXd * f, Vector3d * v1, Vector3d * v2, Vector3d * r)
{
  // face plane equation
  // x = xi * x0 + k, y = yi * y0 + l, z = zi * z0 + m

  // where does v1-v2 intersect face plane?

  // is intersection within face
  
  return true;
}

bool Geometry::equal(Vector3d v1, Vector3d v2)
{
  if ( (v1(0) == v2(0) && v1(1) == v2(1) && v1(2) == v2(2)) ||
       (v1(0) == v2(0) && v1(2) == v2(2) && v1(1) == v2(1)) ||
       (v1(0) == v2(1) && v1(1) == v2(0) && v1(2) == v2(2)) ||
       (v1(0) == v2(1) && v1(1) == v2(2) && v1(2) == v2(0)) ||
       (v1(0) == v2(2) && v1(1) == v2(0) && v1(2) == v2(1)) ||
       (v1(0) == v2(2) && v1(1) == v2(1) && v1(2) == v2(0)) )
    {
      return true;
    }
  return false;  
}

bool Geometry::equal(Vector3d v1, Vector3d v2, Vector3d w1, Vector3d w2)
{
  if ( equal(v1,w1) && equal(v2,w2) )
    {
      return true;
    }
  return false;
}

template <typename M> bool Geometry::fequal(M m1, M m2)
{
  if ( equal(m1.row(0),m2.row(0)) && equal(m1.row(1),m2.row(1))  && equal(m1.row(2),m2.row(2)) ||
       equal(m1.row(0),m2.row(0)) && equal(m1.row(1),m2.row(1))  && equal(m1.row(2),m2.row(2)) ||

       equal(m1.row(0),m2.row(1)) && equal(m1.row(1),m2.row(2))  && equal(m1.row(2),m2.row(0)) ||
       equal(m1.row(0),m2.row(1)) && equal(m1.row(1),m2.row(0))  && equal(m1.row(2),m2.row(2)) ||

       equal(m1.row(0),m2.row(2)) && equal(m1.row(1),m2.row(1))  && equal(m1.row(2),m2.row(0)) ||
       equal(m1.row(0),m2.row(2)) && equal(m1.row(1),m2.row(0))  && equal(m1.row(2),m2.row(1))
       )
    {
      return true;
    }
  return false;
}

template bool Geometry::fequal(Matrix<double,3,3>, Matrix<double,3,3>);


