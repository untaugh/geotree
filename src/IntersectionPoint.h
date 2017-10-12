#pragma once
#include "Types.h"
#include <vector>
#include <memory>
#include <utility>
#include "Mesh.h"
#include "PointInfo.h"

namespace Geotree
{
  class Mesh;
  
  enum MeshIndex {
    FIRST,
    SECOND,
  };

  class IntersectionPoint
  {
  public:
    IntersectionPoint(PointInfo firstType, PointInfo secondType, Vector _point);

    PointInfo getPoint(const MeshIndex mesh);
    PointInfo first;
    PointInfo second;
    Vector point;
    FaceSet connectedPoints;
    bool operator==(const IntersectionPoint &ipoint);
  };
}
