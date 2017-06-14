#include "Geometry.h"

Geometry::Geometry(MatrixXd V_, MatrixXi F_)
{
  this->V = V_;
  this->F = F_;
}
