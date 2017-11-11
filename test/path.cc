#include "gtest/gtest.h"
#include <Eigen/Core>

#include "Path.h"
#include "Mesh.h"

namespace {
  using namespace Geotree;
  
  class PathTest : public ::testing::Test {
  };
  
  TEST_F(PathTest, first)
  {
    Mesh mesh;
    PathT path(mesh);

    path.add(Vector3d(0,0,0));

    EXPECT_EQ(path.size(), 1);

    EXPECT_EQ(path.get(0), Vector3d(0,0,0));    
  }

  TEST_F(PathTest, addPoints)
  {
    Mesh mesh;
    PathT path(mesh);

    path.add(Vector3d(1,0,0));
    path.add(Vector3d(0,2,0));
    path.add(Vector3d(0,0,3));

    EXPECT_EQ(path.size(), 3);

    EXPECT_EQ(path.get(0), Vector3d(1,0,0));
    EXPECT_EQ(path.get(1), Vector3d(0,2,0));
    EXPECT_EQ(path.get(2), Vector3d(0,0,3));
  }

  TEST_F(PathTest, addOperator)
  {
    Mesh mesh;
    PathT path(mesh);

    path.add(Vector3d(1,4,0));
    path += Vector3d(0,2,5);
    path += Vector3d(6,0,3);

    EXPECT_EQ(path.size(), 3);

    EXPECT_EQ(path.get(0), Vector3d(1,4,0));
    EXPECT_EQ(path.get(1), Vector3d(0,2,5));
    EXPECT_EQ(path.get(2), Vector3d(6,0,3));
  }
}

