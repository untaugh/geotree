#pragma once
#include "Point.h"

using namespace Eigen;

namespace Geotree
{
  class EdgePoint : public Point
  {
  public:
    int edge;
    double distance;
    bool operator < (const EdgePoint &edgepoint) const;
  };
}
