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
  }
}
