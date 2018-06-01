#pragma once
#include <Eigen/Core>
#include <set>
#include <vector>
#include "Paths.h"

using namespace Eigen;

namespace Geotree
{

  struct FacePart
  {
    int faceindex;
    int newindex;
    MeshID mesh;
    std::set<int> newfaces;
  };

  class FacesDivide
  {
  public:
    FacesDivide(Paths &paths);
    Paths &paths;
    Mesh newMesh;

    std::vector<FacePart> faceparts;

    void divide(FacePart &facepart);
  };
}
