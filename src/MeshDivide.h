#pragma once
#include <Eigen/Core>
#include "Mesh.h"
#include "Paths.h"

using namespace Eigen;

namespace Geotree
{
  class MeshDivide
  {
  public:
    MeshDivide(const Paths &paths);

    std::set <int> facesMesh0A;
    std::set <int> facesMesh0B;
    std::set <int> facesMesh1A;
    std::set <int> facesMesh1B;

    bool inside0A;
    bool inside0B;
    bool inside1A;
    bool inside1B;

    const Mesh &mesh0;
    const Mesh &mesh1;
    const Paths &paths;

  private:
    bool contains(int face, std::set<int> faces);
    std::set<int> connectedfaces(MeshID mesh, int face);
    void divide(MeshID mesh);
    bool overlap();
    bool inside(MeshID meshid, std::set<int> faces, Cube box);
  };
}
