#pragma once
#include <vector>
#include "Mesh.h"

typedef struct Intersect {
  Vector2i planesMesh1;
  Vector2i planesMesh2;
  Vector3d point;
} Intersect;

namespace Geotree
{
class Intersection
{
 public:
  Intersection(const Mesh mesh1, const Mesh mesh2);

 private:
  std::vector <Intersect> points;
};
}
