#include "gtest/gtest.h"
#include <Eigen/Core>
#include "MeshDivide.h"
#include "MeshFactory.h"

namespace {
  using namespace Geotree;

  class MeshDivideTest : public ::testing::Test {
  };

  TEST_F(MeshDivideTest, Basic)
  {
    MeshFactory factory;

    Mesh tetra0 = factory.tetra(10);
    Mesh tetra1 = factory.tetra(10);
    tetra1.translate(Vector3d(1, 1, 1));

    IntersectionPoints points0(tetra0, tetra1);
    MeshDivide divide(points0);
    divide.divide();
  }
}
