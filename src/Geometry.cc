#include "Geometry.h"
#include <Eigen/Dense>

#include <iostream>

Geometry::Geometry(MatrixXd V_, MatrixXi F_)
{
  this->V = V_;
  this->F = F_;
}

bool Geometry::intersect(MatrixXd * f, Vector3d * s1, Vector3d * s2, Vector3d * p)
{
  // Plane and segment intersection
  
  Vector3d v1 = f->row(0) - f->row(1);
  Vector3d v2 = f->row(0) - f->row(2);

  // cross product to find normal
  Vector3d n = v1.cross(v2);
  n.normalize();
  
  // calculate u and w
  Vector3d u = *s2 - *s1;
  Vector3d w = *s1 - f->row(0).transpose();

  // calculate D and N
  float D = n.dot(u);
  float N = -n.dot(w);

  // is segment parallell to plane
  if (D == 0)
    {
      return false;
    }
  
  float sI = N/D;

  //std::cout << "sI " << sI << std::endl;
  
  // is segment outside plane
  if (sI < 0.0 || sI > 1.0)
    {
      return false;
    }
  
  // calulate point of inersection
  *p = *s1 + sI * u;
  
  // Test if point is inside triangle
  
  Vector3d e1 = f->row(1) - f->row(0);
  Vector3d e2 = f->row(2) - f->row(1);
  Vector3d e3 = f->row(0) - f->row(2);

  Vector3d c1 = *p - f->row(0).transpose();
  Vector3d c2 = *p - f->row(1).transpose();
  Vector3d c3 = *p - f->row(2).transpose();

  float r1 = n.dot(e1.cross(c1));
  float r2 = n.dot(e2.cross(c2));
  float r3 = n.dot(e3.cross(c3));
  
  // is point outisde triangle
  if (r1 < 0.0 || r2 < 0.0 || r3 < 0.0)
    {
      //std::cout << "r1: " << r1 << " r2: " << r2 << " r3: " << r3 << std::endl;
      return false;
    }
  
  return true;
}

int Geometry::add(Geometry &g)
{
  int i,V_off, F_off;

  MatrixXd V_ = MatrixXd(this->V.rows() + g.V.rows(),3);
  MatrixXi F_ = MatrixXi(this->F.rows() + g.F.rows(),3);

  // copy self to new matricies
  for(i=0; i<this->V.rows(); i++)
    {
      V_.row(i) = this->V.row(i);
    }

  V_off = i;
  
  for(i=0; i<this->F.rows(); i++)
    {
      F_.row(i) = this->F.row(i);
    }

  F_off = i;

  // copy other to new matricies

  for(i=0; i<g.V.rows(); i++)
    {
      V_.row(V_off+i) = g.V.row(i);
    }

  for(i=0; i<g.F.rows(); i++)
    {
      F_.row(i+F_off) = g.F.row(i);

      // add offset to face index
      F_.row(i+F_off)[0] += V_off;
      F_.row(i+F_off)[1] += V_off;
      F_.row(i+F_off)[2] += V_off;
    }
  
  this->V = V_;
  this->F = F_;
  
  return 0;
}
