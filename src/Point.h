#pragma once
#include <Eigen/Core>
#include <set>
#include <iostream>

using namespace Eigen;

namespace Geotree
{
  enum PointType
  {
    POINT,
    SEGMENT,
    FACE
  };

  enum MeshID
  {
    MESH0 = 0,
    MESH1 = 1,
  };

  class Point
  {
  public:
  Point(Vector3d vector) : vector(vector) {};
    bool connected(Point point);
    void flip();
    bool operator !=(Point &point);
    bool operator == (Point &point);
    PointType typeMesh0;
    PointType typeMesh1;

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

  std::ostream& operator<< (std::ostream& stream, const PointType& type);
  std::ostream& operator<< (std::ostream& stream, const Point& point);
}
