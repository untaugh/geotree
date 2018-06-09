#include "Point.h"

namespace Geotree
{
  std::ostream& operator<< (std::ostream& stream, const Cube& cube)
  {
        stream << "(";
	stream << cube.p0.transpose() << "," << cube.p1.transpose() << std::endl;
        stream << ")";
    return stream;
  }
}
