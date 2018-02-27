#pragma once
#include "Types.h"
#include "Mesh.h"
#include "IntersectionPoint.h"
#include "IntersectionPoints.h"
#include "IntersectionPaths.h"

namespace Geotree
{
  class MeshIntersection
  {
  public:
    MeshIntersection(Mesh _mesh1, Mesh _mesh2);
      
    Mesh mesh;
    IntersectionPoints points;
    IntersectionPaths paths;
    const int secondMeshFOffset;
    const int secondMeshVOffset;

      Vector3i getFace(int face);

      void merge();

  private:
      void calculatePoints();
  };
}
