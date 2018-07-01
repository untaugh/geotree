#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Face2D.h"
#include <algorithm>

namespace {
  using namespace Geotree;

  class Loop2DTest : public ::testing::Test {
  protected:
  };

  TEST_F(Loop2DTest, Basic)
  {
    Loop2D loop;

    Point2D p0 = {0,0};
    Point2D p1 = {1,0};
    Point2D p2 = {0,1};

    loop.points.push_back(p0);
    loop.points.push_back(p1);
    loop.points.push_back(p2);

    Point2D p3 = {0.5,0.5};

    EXPECT_TRUE(loop.contains(p3));
  }
}
