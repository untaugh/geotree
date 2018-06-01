#include "gtest/gtest.h"
#include "Triangulate.h"
#include <Eigen/Core>

using namespace Eigen;
using namespace Geotree;

namespace {
  class TriangulateTest : public ::testing::Test {
  };

  TEST_F(TriangulateTest, basic)
  {
    std::vector<Point2D> face;

    face.push_back(Point2D({0,0}));
    face.push_back(Point2D({1,0}));
    face.push_back(Point2D({0,1}));
    
    std::vector<Point2D> split;
    std::vector<std::vector<Point2D>> splits;

    Point2D p0 = {0.1, 0.1};
    Point2D p1 = {0.1, 0.2};
    Point2D p2 = {0.2, 0.1};

    split.push_back(p0);
    split.push_back(p1);
    split.push_back(p2);
    splits.push_back(split);

    Triangulate tri(face, splits);
      
    EXPECT_TRUE(0);
  }
}
