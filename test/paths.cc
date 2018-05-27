#include "gtest/gtest.h"
#include <Eigen/Core>
#include "MeshFactory.h"
#include "Paths.h"

namespace {
  using namespace Geotree;

  class PathsTest : public ::testing::Test {
  };

  TEST_F(PathsTest, CubeSide)
  {
    MeshFactory factory;
    Mesh cube0 = factory.cube(10,10,10);
    Mesh cube1 = factory.cube(10,1,1);
    Mesh cube2 = factory.cube(12,1,1);
    cube1.translate(Vector3d(1, 4.5, 1));
    cube2.translate(Vector3d(-1, 4.5, 1));
    IntersectionPoints points0(cube0, cube1);
    IntersectionPoints points1(cube0, cube2);

    Paths paths0(points0);
    Paths paths1(points1);
    
    EXPECT_EQ(1, paths0.count());
    EXPECT_EQ(2, paths1.count());
  }
}
