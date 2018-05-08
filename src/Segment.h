#pragma once
#include <Eigen/Core>

using namespace Eigen;

namespace Geotree
{
  class Segment
  {
  public:
    Segment(Vector3d begin, Vector3d end) : begin(begin), end(end) {};
    Segment(Vector3d begin, Vector3d end, int p0, int p1) : begin(begin), end(end), point0(p0), point1(p1){};

    const Vector3d begin;
    const Vector3d end;

    int point0;
    int point1;
    int face0;
    int face1;

    bool intersects(Segment segment, Vector3d &point);
    bool intersects(Vector3d point);

    Matrix2d get2d();

    bool operator ==(Segment &segment)
    {
      return (((this->point0 == segment.point0) && (this->point1 == segment.point1))
              || (((this->point0 == segment.point1) && (this->point1 == segment.point0))));
    }
  };
}
