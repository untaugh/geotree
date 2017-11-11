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
    void calculatePoints();
    const int secondMeshFOffset;
    const int secondMeshVOffset;

    std::vector <EdgePoint> getEdgePoints(int face, std::vector <FaceSet> &subpaths);
    bool operator ()(EdgePoint first, EdgePoint second);
    void sortEdgePoints(int face, std::vector <EdgePoint> &edgePoints);
    FaceIndex getFace(int face);
  };
}
