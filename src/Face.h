#pragma once
#include <vector>
#include <Eigen/Core>
#include "Segment.h"
#include "Point.h"

using namespace Eigen;

namespace Geotree
{
  class Face
  {
  public:
    Face(const Vector3d v0, const Vector3d v1, const Vector3d v2, const int index, Vector3i face);

    const int index;
    const Vector3i face;
    Vector3d normal;
    const Vector3d point0;
    const Vector3d point1;
    const Vector3d point2;

    //std::vector <Path> paths;

    void intersect(const Segment segment, std::vector <Point> &points);
  private:
    bool intersects(Segment segment, std::vector <Point> &points) const;
    bool intersectsPlanar(Segment segment, std::vector <Point> &points);
    Segment getSegment(const int n) const;
    bool isPlanar(Segment segment);
    void getNormal();
    bool isInside(const Vector3d point) const;
    bool isPlanar(Vector3d point);
  };
}
