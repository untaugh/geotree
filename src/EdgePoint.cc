#include "EdgePoint.h"

namespace Geotree
{
  bool Point::operator < (const EdgePoint &point) const
  {
    if (this->path < point.path) return true;
    else if (this->path > point.path) return false;
    else if (this->number < point.number) return true;
    else return false;
  }

}
