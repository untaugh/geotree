#pragma once
#include "Types.h"
#include "IntersectionPoint.h"
#include "Segment.h"

namespace Geotree
{
  class IntersectionPoint;
  class PointInfo;

  class FaceT
  {
  public:
    FaceT(const Verticies &verticies, const Vector3i points);
    bool intersects(const Line segment);
    IntersectionPoint getIntersection(SegmentT segment);
  private:
    PointInfo getType(Vector3d point);
    bool hasPoint(Vector3d point);
    SegmentIndex getIndex(int index);
    int getPointIndex(Vector3d point);
    Plane getVectors();
    Line getVectors(int index);
    Vector3d getPoint(int index);
    const Verticies &verticies;
    Vector3i points;
  };
}

