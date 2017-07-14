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

    // intersect : do segments intersect
    // ret       : true if yes
    // in v1a    : segment 1 start
    // in v1b    : segment 1 end
    // in v2a    : segment 2 start
    // in v2b    : segment 2 end
    bool intersect(Vector3d v1a, Vector3d v1b, Vector3d v2a, Vector3d v2b);
  }
}
