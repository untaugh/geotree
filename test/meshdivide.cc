#include "gtest/gtest.h"
#include <Eigen/Core>
#include "MeshDivide.h"
#include "MeshFactory.h"

namespace {
  using namespace Geotree;

  class MeshDivideTest : public ::testing::Test {
  };

  TEST_F(MeshDivideTest, Cube0)
  {
    MeshFactory factory;
    Mesh cube0 = factory.cube(10,10,10);
    Mesh cube1 = factory.cube(10,1,1);
    cube1.translate(Vector3d(1, 1, 1));

    IntersectionPoints points(cube0, cube1);
    Paths paths(points);
    MeshDivide divide(paths);

    EXPECT_EQ(divide.facesMesh0A.size(), 10);
    EXPECT_EQ(divide.facesMesh0B.size(), 0);
    EXPECT_EQ(divide.facesMesh1A.size(), 2);
    EXPECT_EQ(divide.facesMesh1B.size(), 2);

    EXPECT_FALSE(divide.inside0A);
    EXPECT_FALSE(divide.inside0B);
    EXPECT_TRUE(divide.inside1A);
    EXPECT_FALSE(divide.inside1B);
  }

  TEST_F(MeshDivideTest, Cube1)
  {
    MeshFactory factory;
    Mesh cube0 = factory.cube(10,10,10);
    Mesh cube1 = factory.cube(10,10,10);
    cube1.translate(Vector3d(1, 1, 1));

    IntersectionPoints points(cube0, cube1);
    Paths paths(points);
    MeshDivide divide(paths);

    EXPECT_EQ(divide.facesMesh0A.size(), 6);
    EXPECT_EQ(divide.facesMesh0B.size(), 0);
    EXPECT_EQ(divide.facesMesh1A.size(), 6);
    EXPECT_EQ(divide.facesMesh1B.size(), 0);

    EXPECT_FALSE(divide.inside0A);
    EXPECT_FALSE(divide.inside0B);
    EXPECT_FALSE(divide.inside1A);
    EXPECT_FALSE(divide.inside1B);
  }

  TEST_F(MeshDivideTest, Cube2)
  {
    MeshFactory factory;
    Mesh cube0 = factory.cube(10,10,10);
    Mesh cube1 = factory.cube(12,8,8);
    cube1.translate(Vector3d(-1, 1, 1));

    IntersectionPoints points(cube0, cube1);
    Paths paths(points);
    MeshDivide divide(paths);

    EXPECT_EQ(divide.facesMesh0A.size(), 8);
    EXPECT_EQ(divide.facesMesh0B.size(), 0);
    EXPECT_EQ(divide.facesMesh1A.size(), 2);
    EXPECT_EQ(divide.facesMesh1B.size(), 2);

    EXPECT_FALSE(divide.inside0A);
    EXPECT_FALSE(divide.inside0B);
    EXPECT_FALSE(divide.inside1A);
    EXPECT_FALSE(divide.inside1B);
  }

}
