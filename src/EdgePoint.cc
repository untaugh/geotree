#include "EdgePoint.h"

namespace Geotree
{
  EdgePoint::EdgePoint(const Point &point)
    : Point(point.vector)
  {
    //this->vector = point.vector;
  }

  bool EdgePoint::operator < (const EdgePoint &point) const
  {
    if (this->face0edge < point.face0edge)
    {
      return true;
    }

    if (this->face0edge > point.face0edge)
    {
      return false;
    }

    if (this->position < point.position)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
