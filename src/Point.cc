#include "Point.h"

namespace Geotree
{
  Vector Point::getVector() const
  {
    return mesh.V.row(this->index);
  }
}
