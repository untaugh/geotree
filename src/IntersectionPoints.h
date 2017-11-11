#pragma once
#include "Types.h"
#include "IntersectionPoint.h"

namespace Geotree {
  
  class IntersectionPoints
  {
  public:
    void addPoint(IntersectionPoint point);
    IntersectionPoint getPoint(const int index) const;

    FaceSet getIntersectedFaces();
    Faces getIntersectedFaces2();
    void calculateConnectedPoints();
    int size() const { return points.size(); };
  private:
    FaceSet getConnectedPoints(int pointIndex);
    std::vector <IntersectionPoint> points;
  };
}
