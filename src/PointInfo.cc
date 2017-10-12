#include "IntersectionPoint.h"

namespace Geotree
{
  
  void PointInfo::getFaces(Mesh mesh)
  {
    if (type == PointInfo::FACE)
      {
	int i = mesh.getFaceIndex(index2);
	faces.insert(i);
      }
    else if (type == PointInfo::SEGMENT)
      {
	SegmentIndex segment = index2.block(0,0,2,1);
	faces = mesh.getFacesWithSegment(segment);
      }
    else if (type == PointInfo::POINT)
      {
	faces = mesh.getFacesWithPoint(index2[0]);
      }
  }
}


