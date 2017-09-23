#pragma once
#include <Eigen/Core>
#include "Geometry.h"

using namespace Eigen;

namespace Geotree { 
  namespace Validate
  {
    // verticies of a face
    bool face(Matrix3d f);

    // indicies of a face
    bool face(Vector3i f);
    
    // geometry
    bool geometry(Geometry g);

    // planar : are points on same plane
    // ret    : true if yes
    // in P   : Nx3 matrix with verticies
    bool planar(MatrixXd P);
  }
}
