#pragma once
#include "PointInfo.h"

namespace Geotree
{
  class SegmentT
  {
  public:
    SegmentT(const Verticies &verticies, const Vector2i points);
    Line getVectors();
    PointInfo getType(Vector3d point);
  private:
    const Verticies &verticies;
    const Vector2i index;
  };
}
