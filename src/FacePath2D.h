#pragma once

using namespace Eigen;

namespace Geotree
{
  class FacePath2D
  {
  public:
    Face2D(Face face);

    const Axis dropaxis;

    Point2D p0;
    Point2D p1;
    Point2D p2;

    std::vector <Matrix<int, Dynamic, 3>> split(std::vector <std::set<Point>> paths);
  };
}
