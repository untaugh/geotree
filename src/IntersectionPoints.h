#pragma once
#include "Types.h"
#include "IntersectionPoint.h"
#include "Path.h"
namespace Geotree
{
  
  class IntersectionPoints
  {
  public:
      explicit IntersectionPoints(Mesh& _mesh) : mesh(_mesh) {};
      void addPoint(IntersectionPoint point);
      IntersectionPoint getPoint(const int index) const;

      FaceSet getIntersectedFaces();
      Faces getIntersectedFaces2();
      std::vector <PathX<FacePoint>> getFacePath(const int index);
      void calculateConnectedPoints();
      int size() const { return points.size(); };
      Mesh &mesh;

  private:
      FaceSet getConnectedPoints(int pointIndex);
      std::vector <IntersectionPoint> points;
  };
}
