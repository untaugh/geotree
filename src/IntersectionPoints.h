#pragma once
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include "Mesh.h"
#include "Point.h"
#include "Path.h"

namespace Geotree
{
  class IntersectionPoints
  {
  public:
    IntersectionPoints(const Mesh &mesh0, const Mesh &mesh1);
    int numPaths();
    void dividedFaces(std::set <int> &div0, std::set <int> &div1) const;

    std::vector <Path> getPaths0(int face);
    std::vector <Path> getPaths1(int face);

    std::vector <Point> points;
    const Mesh &mesh0;
    const Mesh &mesh1;

  private:
    bool contains(Point &point);
    void sort();
    void getPoints();
    void nextPoint(Point ** previous, Point ** point);
    bool firstPoint(Point ** point);
    int connectedPoints(Point point, Point ** point_find);
  };

  std::ostream& operator<< (std::ostream& stream, const IntersectionPoints& points);
}
