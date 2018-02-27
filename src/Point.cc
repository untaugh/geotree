#include "Point.h"

namespace Geotree
{
  Vector Point::getVector() const
  {
      return Vector(0,0,0);
  }

    double Point::distance(const Point &point) const {
        return (this->getVector() - point.getVector()).norm();
    }

    bool Point::operator!=(const Point &point) const {
      return this->index != point.index;
    }
}
