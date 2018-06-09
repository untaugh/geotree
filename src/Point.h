#pragma once
#include <Eigen/Core>
#include <set>
#include <iostream>
#include "Cube.h"

using namespace Eigen;

namespace Geotree
{
  enum MeshID
  {
    MESH0 = 0,
    MESH1 = 1,
  };

  class Point
  {
  public:
    Point(Vector3d vector) : vector(vector) {};

    bool operator !=(Point &point);
    bool operator == (Point &point);
    bool operator < (const Point &point) const;

    bool connected(Point point);
    void flip();
    bool inside(const Cube box) const;
    
    std::set <int> facesMesh0;
    std::set <int> facesMesh1;

    const Vector3d vector;
    int path = -1;
    int number = -1;
    const double NEAR_ZERO = 1.0e-14;
    
    std::set <int> &getFaces(MeshID mesh);
  private:
    bool connected(MeshID mesh, Point point);
    bool hasFace(MeshID mesh, int face);
  };

  std::ostream& operator<< (std::ostream& stream, const Point& point);
}
