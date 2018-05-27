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
    Face(const Vector3d v0, const Vector3d v1, const Vector3d v2, const int index);

    const int index;
    Vector3d normal;
    const Vector3d point0;
    const Vector3d point1;
    const Vector3d point2;

    bool intersect(const Segment segment, std::vector <Point> &points) const;
    std::vector<Segment> getSegments()
      {
	std::vector<Segment> segments;
	segments.push_back(Segment(point0, point1));
	segments.push_back(Segment(point1, point2));
	segments.push_back(Segment(point2, point0));
	return segments;
      };
  private:
    void getNormal();
    bool isInside(const Vector3d point) const;
  };
}
