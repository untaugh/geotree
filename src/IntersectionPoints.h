#pragma once
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include "Mesh.h"
#include "Point.h"

namespace Geotree
{
  class IntersectionPoints
  {
  public:
    IntersectionPoints(const Mesh &mesh0, const Mesh &mesh1);

    std::vector <Point> points;
    const Mesh &mesh0;
    const Mesh &mesh1;

  private:
    bool contains(Point &point);
    void findPoints();
    void getIntersectionPoints(MeshID mesh, std::vector <Point> &points);
    void add(Point point);
    const Mesh &getMesh(MeshID meshID);
    const Mesh &getOtherMesh(MeshID meshID);
  };

  std::ostream& operator<< (std::ostream& stream, const IntersectionPoints& points);
}
