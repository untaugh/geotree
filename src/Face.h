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
    //FaceT(Mesh &_mesh, const Vector3i points);
    FaceT(Mesh &_mesh, int index);
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
    Mesh &mesh;
    int index;
    //Vector3i points;
  };
}
