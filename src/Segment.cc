#include "Segment.h"

namespace Geotree
{
  bool Segment::operator ==(Segment &segment)
    {
      return (this->begin == segment.begin && this->end == segment.end) ||
	(this->begin == segment.end && this->end == segment.begin);
    }
  
}
