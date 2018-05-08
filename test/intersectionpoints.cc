#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>
#include "IntersectionPoints.h"
#include "MeshFactory.h"

namespace {
    using namespace Geotree;

    class IntersectionPointsTest : public ::testing::Test {
    };

    TEST_F(IntersectionPointsTest, CubeSlice) {

      MeshFactory factory;
      Mesh cube1 = factory.cube(10,10,10);
      Mesh cube2 = factory.cube(0.1,12,12);
      cube2.translate(Vector3d(1, -1, -1));

      IntersectionPoints points1(cube1, cube2);

      EXPECT_EQ(16, points1.points.size());

      EXPECT_EQ(2, points1.numPaths());
    }

    TEST_F(IntersectionPointsTest, CubeSame) {

      MeshFactory factory;
      Mesh cube1 = factory.cube(10,10,10);
      Mesh cube2 = factory.cube(10,10,10);
      IntersectionPoints points1(cube1, cube2);
      EXPECT_EQ(8, points1.points.size());
      EXPECT_EQ(0, points1.numPaths());
    }

    TEST_F(IntersectionPointsTest, DISABLED_TinyIncrement) {

      for (int i=10; i>0; i--)
      {
      MeshFactory factory;
      Mesh cube1 = factory.cube(1,1,1);
      Mesh cube2 = factory.cube(1,1,1);

      double inc = 1e-14 * i;

      cube2.translate(Vector3d(inc, inc, inc));

      IntersectionPoints points1(cube1, cube2);
      EXPECT_EQ(10, points1.points.size());
      EXPECT_EQ(1, points1.numPaths());
      }
    }

    TEST_F(IntersectionPointsTest, CubeMulti) {

      MeshFactory factory;
      Mesh cube1 = factory.cube(10,10,10);
      Mesh cube2 = factory.cube(0.1,12,12);
      cube2.translate(Vector3d(0.5, -1, -1));

      for (int i=2; i<9; i++)
      {
        Mesh cube3 = factory.cube(0.1,12,12);
        cube3.translate(Vector3d(0.5 * i, -1, -1));
        cube2 = cube2 + cube3;
      }

      IntersectionPoints points1(cube1, cube2);

      EXPECT_EQ(16*8, points1.points.size());
      EXPECT_EQ(16, points1.numPaths());
    }

    TEST_F(IntersectionPointsTest, TetraXYZ) {
      MeshFactory factory;

      Mesh tetra0 = factory.tetra(10);
      Mesh tetra1 = factory.tetra(10);
      tetra1.translate(Vector3d(1, 1, 1));

      IntersectionPoints points0(tetra0, tetra1);
      IntersectionPoints points1(tetra1, tetra0);

      EXPECT_EQ(3, points0.points.size());
      EXPECT_EQ(3, points1.points.size());
      EXPECT_EQ(1, points0.numPaths());

      std::vector <Path> paths0a = points0.getPaths0(0);
      std::vector <Path> paths0b = points0.getPaths0(1);
      std::vector <Path> paths0c = points0.getPaths0(2);
      std::vector <Path> paths0d = points0.getPaths0(3);

      EXPECT_EQ(0, paths0a.size());
      EXPECT_EQ(0, paths0b.size());
      EXPECT_EQ(0, paths0c.size());
      EXPECT_EQ(2, paths0d.size());

      std::set <int> f0, f1;
      points0.dividedFaces(f0, f1);
      EXPECT_EQ(1, f0.size());
      EXPECT_EQ(3, f1.size());


    }

    TEST_F(IntersectionPointsTest, TetraXY) {
      MeshFactory factory;
      Mesh tetra0 = factory.tetra(10);
      Mesh tetra1 = factory.tetra(10);
      tetra1.translate(Vector3d(1, 1, 0));
      IntersectionPoints points0(tetra0, tetra1);
      EXPECT_EQ(4, points0.points.size());
      EXPECT_EQ(1, points0.numPaths());

      std::set <int> f0, f1;
      points0.dividedFaces(f0, f1);
      EXPECT_EQ(2, f0.size());
      EXPECT_EQ(3, f1.size());
    }

    TEST_F(IntersectionPointsTest, TetraPoint) {
      MeshFactory factory;
      Mesh tetra0 = factory.tetra(10);
      Mesh tetra1 = factory.tetra(10);
      tetra1.translate(Vector3d(10, 0, 0));
      IntersectionPoints points0(tetra0, tetra1);
      EXPECT_EQ(1, points0.points.size());
      EXPECT_EQ(1, points0.numPaths());
    }

    TEST_F(IntersectionPointsTest, DISABLED_CubeSide) {
      MeshFactory factory;
      Mesh cube0 = factory.cube(10,10,10);
      Mesh cube1 = factory.cube(10,10,10);
      cube1.translate(Vector3d(10, 0, 0));
      IntersectionPoints points0(cube0, cube1);
      EXPECT_EQ(4, points0.points.size());
      EXPECT_EQ(1, points0.numPaths());
    }

    TEST_F(IntersectionPointsTest, CubeTwoPoints) {
      MeshFactory factory;
      Mesh cube0 = factory.cube(10,10,10);
      Mesh cube1 = factory.cube(10,10,10);
      cube1.translate(Vector3d(10, 10, 0));
      IntersectionPoints points0(cube0, cube1);
      EXPECT_EQ(2, points0.points.size());
      EXPECT_EQ(1, points0.numPaths());
    }

    TEST_F(IntersectionPointsTest, CubeFace0) {
      MeshFactory factory;
      Mesh cube0 = factory.cube(10,10,10);
      Mesh cube1 = factory.cube(10,1,1);
      Mesh cube2 = factory.cube(12,1,1);
      cube1.translate(Vector3d(1, 4.5, 1));
      cube2.translate(Vector3d(-1, 4.5, 1));
      IntersectionPoints points0(cube0, cube1);
      IntersectionPoints points1(cube1, cube0);
      IntersectionPoints points2(cube0, cube2);
      EXPECT_EQ(8, points0.points.size());
      EXPECT_EQ(8, points1.points.size());
      EXPECT_EQ(16, points2.points.size());
      EXPECT_EQ(1, points0.numPaths());
      EXPECT_EQ(2, points2.numPaths());
    }

    TEST_F(IntersectionPointsTest, CubeFace1) {
      MeshFactory factory;
      Mesh cube0 = factory.cube(10,10,10);
      Mesh cube1 = factory.cube(10,1,1);
      cube1.translate(Vector3d(1, 4.5, 4.5));
      IntersectionPoints points0(cube0, cube1);
      EXPECT_EQ(8, points0.points.size());
      EXPECT_EQ(1, points0.numPaths());
    }

    TEST_F(IntersectionPointsTest, CubeOff) {
      MeshFactory factory;
      Mesh cube0 = factory.cube(10,10,10);
      Mesh cube1 = factory.cube(10,10,10);
      Mesh cube2 = factory.cube(10,10,10);
      cube1.translate(Vector3d(1, 1, 1));
      cube2.translate(Vector3d(1, 2, 3));
      IntersectionPoints points0(cube0, cube1);
      IntersectionPoints points1(cube0, cube2);
      EXPECT_EQ(10, points0.points.size());
      EXPECT_EQ(14, points1.points.size());
      EXPECT_EQ(1, points0.numPaths());
      EXPECT_EQ(1, points1.numPaths());
    }

}
