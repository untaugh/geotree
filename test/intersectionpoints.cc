#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>
#include "IntersectionPoints.h"
#include "MeshFactory.h"
#include "Paths.h"

namespace {
    using namespace Geotree;

    class IntersectionPointsTest : public ::testing::Test {
    };

    TEST_F(IntersectionPointsTest, CubeSlice) {

      MeshFactory factory;
      Mesh cube1 = factory.cube(10,10,10);
      Mesh cube2 = factory.cube(1,12,12);
      cube2.translate(Vector3d(1, -1, -1));

      IntersectionPoints points1(cube1, cube2);

      EXPECT_EQ(16, points1.points.size());

      Paths paths(points1);
      EXPECT_EQ(2, paths.count());
      EXPECT_EQ(8, paths.size(0));
      EXPECT_EQ(8, paths.size(1));
    }

    TEST_F(IntersectionPointsTest, DISABLED_CubeSame) {

      MeshFactory factory;
      Mesh cube1 = factory.cube(10,10,10);
      Mesh cube2 = factory.cube(10,10,10);
      IntersectionPoints points1(cube1, cube2);
      EXPECT_EQ(8, points1.points.size());

      Paths paths(points1);
      EXPECT_EQ(0, paths.count());
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
      //EXPECT_EQ(1, points1.numPaths());
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

      IntersectionPoints points0(cube1, cube2);

      EXPECT_EQ(16*8, points0.points.size());

      Paths paths0(points0);
      EXPECT_EQ(16, paths0.count());
      EXPECT_EQ(8, paths0.size(0));
      EXPECT_EQ(8, paths0.size(1));
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

      Paths paths0(points0);
      Paths paths1(points1);
      EXPECT_EQ(1, paths0.count());
      EXPECT_EQ(1, paths1.count());
      EXPECT_EQ(3, paths0.size(0));
      EXPECT_EQ(3, paths1.size(0));
    }

    TEST_F(IntersectionPointsTest, TetraXY) {
      MeshFactory factory;
      Mesh tetra0 = factory.tetra(10);
      Mesh tetra1 = factory.tetra(10);
      tetra1.translate(Vector3d(1, 1, 0));
      IntersectionPoints points0(tetra0, tetra1);
      EXPECT_EQ(4, points0.points.size());

      Paths paths0(points0);
      EXPECT_EQ(1, paths0.count());
      EXPECT_EQ(4, paths0.size(0));
    }

    TEST_F(IntersectionPointsTest, TetraPoint) {
      MeshFactory factory;
      Mesh tetra0 = factory.tetra(10);
      Mesh tetra1 = factory.tetra(10);
      tetra1.translate(Vector3d(10, 0, 0));
      IntersectionPoints points0(tetra0, tetra1);
      EXPECT_EQ(1, points0.points.size());

      Paths paths0(points0);
      EXPECT_EQ(1, paths0.count());
      EXPECT_EQ(1, paths0.size(0));
    }

    TEST_F(IntersectionPointsTest, CubeSide) {
      MeshFactory factory;
      Mesh cube0 = factory.cube(10,10,10);
      Mesh cube1 = factory.cube(10,10,10);
      cube1.translate(Vector3d(10, 0, 0));
      IntersectionPoints points0(cube0, cube1);
      EXPECT_EQ(4, points0.points.size());

      Paths paths0(points0);
      EXPECT_EQ(1, paths0.count());
      EXPECT_EQ(4, paths0.size(0));
    }

    TEST_F(IntersectionPointsTest, DISABLED_CubeTwoPoints) {
      MeshFactory factory;
      Mesh cube0 = factory.cube(10,10,10);
      Mesh cube1 = factory.cube(10,10,10);
      cube1.translate(Vector3d(10, 10, 0));
      IntersectionPoints points0(cube0, cube1);
      EXPECT_EQ(2, points0.points.size());

      Paths paths0(points0);
      EXPECT_EQ(1, paths0.count());
      EXPECT_EQ(2, paths0.size(0));
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

      Paths paths0(points0);
      Paths paths1(points2);
      EXPECT_EQ(1, paths0.count());
      EXPECT_EQ(2, paths1.count());

      std::set<int> faces00 = paths0.faces(MESH0);
      std::set<int> faces01 = paths0.faces(MESH1);
      EXPECT_EQ(1, faces00.size());
      EXPECT_EQ(8, faces01.size());

      for (int face : faces01)
	{
	  
	}
    }

    TEST_F(IntersectionPointsTest, CubeFace1) {
      MeshFactory factory;
      Mesh cube0 = factory.cube(10,10,10);
      Mesh cube1 = factory.cube(10,1,1);
      cube1.translate(Vector3d(1, 4.5, 4.5));
      IntersectionPoints points0(cube0, cube1);
      EXPECT_EQ(8, points0.points.size());

      Paths paths0(points0);
      EXPECT_EQ(1, paths0.count());
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

      Paths paths0(points0);
      Paths paths1(points1);
      EXPECT_EQ(1, paths0.count());
      EXPECT_EQ(1, paths1.count());      
    }

}
