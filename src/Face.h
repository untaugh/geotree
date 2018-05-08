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
    Face(Vector3d v0, Vector3d v1, Vector3d v2, int index, Vector3i face);

    const int index;
    const Vector3i face;
    Vector3d normal;
    const Vector3d point0;
    const Vector3d point1;
    const Vector3d point2;

    //std::vector <Path> paths;

    void intersect(const Segment segment, std::vector <Point> &points);
  private:
    bool intersects(Segment segment, std::vector <Point> &points);
    bool intersectsPlanar(Segment segment, std::vector <Point> &points);
    Segment getSegment(int n);
    bool isPlanar(Segment segment);
    void getNormal();
    bool isInside(Vector3d point);
    bool isPlanar(Vector3d point);
  };
}
