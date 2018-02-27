#pragma once
#include "Types.h"
#include <vector>
#include <memory>
#include <utility>
#include "Mesh.h"
#include "PointInfo.h"
#include "Point.h"
#include "FacePoint.h"

namespace Geotree
{
 class IntersectionPoint2 : public Point
 {
 public:
     IntersectionPoint2(const FaceT& face, const PointType type) : face(face), type(type){};
     const PointType type;
 private:
     const FaceT& face;
 };

  class IntersectionPoint : public Point
  {
  public:
    IntersectionPoint(Mesh &_mesh, PointInfo firstType, PointInfo secondType, Vector _point);

    bool isConnected(const IntersectionPoint point);
    FaceSet getIntersectedFaces() const;
      FacePoint getFacePoint(int faceIndex);
    FaceSet faces;

    PointInfo first;
    PointInfo second;

    FaceSet connectedPoints;
    bool operator==(const IntersectionPoint &ipoint);

  private:
    void getFaces();
  };
}
