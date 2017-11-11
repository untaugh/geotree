#include "Path.h"

namespace Geotree
{
  void PathT::add(const Vector3d point)
  {
    mesh.V.conservativeResize(mesh.V.rows() + 1, NoChange);
    points.push_back(mesh.V.rows() - 1);
    mesh.V.row(mesh.V.rows() - 1) = point;
  }

  int PathT::size()
  {
    return points.size();
  }

  Vector3d PathT::get(int index)
  {
    return mesh.V.row(points[index]);
  }
}
