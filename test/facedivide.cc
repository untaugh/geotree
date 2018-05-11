#include "gtest/gtest.h"
#include <Eigen/Core>
#include "FaceDivide.h"

namespace {
  using namespace Geotree;

  class FaceDivideTest : public ::testing::Test {
  };

  TEST_F(FaceDivideTest, Basic)
  {
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
    edgepoint p0 = {0, 0, 0.0};
    edgepoint p1 = {0, 0, 0.1};
    edgepoint p2 = {0, 0, 0.2};
    edgepoint p3 = {1, 0, 0.0};
    edgepoint p4 = {2, 0, 1.0};

    EXPECT_TRUE(p0 < p1);
    EXPECT_TRUE(p0 < p2);
    EXPECT_TRUE(p1 < p2);
    EXPECT_TRUE(p2 < p3);
    EXPECT_TRUE(p3 < p4);
  }
}
