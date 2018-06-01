#pragma once
#include <Eigen/Core>
#include <earcut.hpp>
#include <vector>

using namespace Eigen;

namespace Geotree
{
  using Point2D = std::array<double, 2>;

  // class Segment2D
  // {
  //   Segment2D(Point2D start, Point2D end);
  //   const Point2D start;
  //   const Point2D end;
  //   bool onSegment(Point2D);
  // };

  class Triangulate
  {
  public:
    Triangulate(std::vector<Point2D> base, std::vector<std::vector<Point2D>> points);

    // Segment2D segment0;
    // Segment2D segment1;
    // Segment2D segment2;
  };
}
