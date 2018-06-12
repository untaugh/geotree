#pragma once
#include "Point.h"

using namespace Eigen;

namespace Geotree
{
  class EdgePoint : public Point
  {
  public:
    //EdgePoint(Face face, Point point);
    EdgePoint(const Point &point);
    double position;
    bool operator < (const EdgePoint &edgepoint) const;
  };
}
