#include "Point.h"

namespace Geotree
{
  Vector Point::getPoint() const
  {
    return mesh.V.row(this->index);
  }
}
