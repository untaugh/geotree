#include "Geometry.h"
#include <Eigen/Dense>

#include <iostream>

Geometry::Geometry(MatrixXd V_, MatrixXi F_)
{
  this->V = V_;
  this->F = F_;
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
