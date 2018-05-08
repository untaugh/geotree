#include "gtest/gtest.h"
#include <Eigen/Core>
#include "FaceDivide.h"

namespace {
  using namespace Geotree;

  class FaceDivideTest : public ::testing::Test {
  };

  TEST_F(FaceDivideTest, Basic)
  {
    // MeshFactory factory;
    //
    // Mesh tetra0 = factory.tetra(10);
    // Mesh tetra1 = factory.tetra(10);
    //
    // tetra1.translate(Vector3d(1, 1, 1));
    //
    // IntersectionPoints points0(tetra0, tetra1);

    Vector2d p0(0,0);
    Vector2d p1(1,0);
    Vector2d p2(0,1);

    Matrix <double, 2, 2> path;

    path << 0.5, 0.0, 0.0, 0.5;

    FaceDivide fd(p0, p1, p2);

    EXPECT_EQ(3, fd.V.rows());

    fd.cut(path);

    EXPECT_EQ(5, fd.V.rows());

    fd.divide();
  }

  TEST_F(FaceDivideTest, Edgepoint)
  {
    edgepoint p0 = {.segment = 0, .index = 0, .distance = 0.0};
    edgepoint p1 = {.segment = 0, .index = 0, .distance = 0.1};
    edgepoint p2 = {.segment = 0, .index = 0, .distance = 0.2};
    edgepoint p3 = {.segment = 1, .index = 0, .distance = 0.0};
    edgepoint p4 = {.segment = 2, .index = 0, .distance = 1.0};

    EXPECT_TRUE(p0 < p1);
    EXPECT_TRUE(p0 < p2);
    EXPECT_TRUE(p1 < p2);
    EXPECT_TRUE(p2 < p3);
    EXPECT_TRUE(p3 < p4);
  }
}
