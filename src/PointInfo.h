#pragma once
#include "Types.h"
#include "Mesh.h"

namespace Geotree
{
  class PointInfo
  {
  public:
    enum Type {
      FACE,
      SEGMENT,
      POINT
    } type;

    Vector3i index2;

    FaceSet faces;
    void getFaces(Mesh mesh);
    bool operator==(const PointInfo &pointType);
  };
}
