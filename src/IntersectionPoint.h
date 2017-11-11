#pragma once
#include "Types.h"
#include <vector>
#include <memory>
#include <utility>
#include "Mesh.h"
#include "PointInfo.h"
#include "Point.h"

namespace Geotree
{
  class IntersectionPoint : public Point
  {
  public:
    IntersectionPoint(Mesh &_mesh, PointInfo firstType, PointInfo secondType, Vector _point);

    bool isConnected(const IntersectionPoint point);
    FaceSet getIntersectedFaces() const;
    FaceSet faces;

    PointInfo first;
    PointInfo second;

    FaceSet connectedPoints;
    bool operator==(const IntersectionPoint &ipoint);

  private:
    void getFaces();
  };
}
