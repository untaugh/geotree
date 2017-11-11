#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>
#include "MeshIntersection.h"
#include "IntersectionPaths.h"
#include "MeshFactory.h"

namespace {
  using namespace Geotree;

  class IntersectionPathsTest : public ::testing::Test {
  };

  TEST_F(IntersectionPathsTest, basic)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(12,12,10);
    cube2.translate(Vector(-1, -1, 1));
    MeshIntersection mi(cube1,cube2);

    EXPECT_EQ(mi.points.size(), 8);
    EXPECT_EQ(mi.paths.size(), 1);

    FaceSet faces = mi.points.getIntersectedFaces();

    std::vector <FaceSet> subpaths = mi.paths.getSubPaths(2);

    EXPECT_EQ(subpaths.size(), 1);

    for (int i : subpaths[0])
      {
	std::cout << "intersected face:" << i << std::endl;
      }
    
  }  
}
