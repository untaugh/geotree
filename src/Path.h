#pragma once
#include <vector>
#include "Types.h"
#include "Mesh.h"

namespace Geotree
{
  class PathT
  {
  public:
    PathT(Mesh &_mesh) : mesh(_mesh) {};
    PathT(Mesh &_mesh, std::vector <int> _points) : mesh(_mesh) { points = _points; };
    void add(const Vector3d point);
      void add(Point point) { addExisting(point.getIndex()); };
    void addExisting(const int index) { points.push_back(index); };
    int size() const;
    Vector3d get(const int index);
    int start() const { return points.front(); };
    int end() { return points.back(); };
    int index(int i) { return points[i]; };
    std::vector <int> getPoints() const { return points; };

      Point begin() const;
      Point back() const;

      void operator +=(const Vector3d point)
    {
      add(point);
    }

      bool operator == (const PathT &path) const;
      bool operator != (const PathT &path) const;


  private:
    Mesh &mesh;
    std::vector <int> points;
  };

  std::ostream& operator<< (std::ostream& stream, const PathT& path);
}
