#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Face.h"

namespace {
  using namespace Geotree;

  class FaceTest : public ::testing::Test {
  protected:
  };

  TEST_F(FaceTest, Basic)
  {
    Face face(Vector3d(0,0,0), Vector3d(0,0,1), Vector3d(0,0,2), 0);
  }

  TEST_F(FaceTest, intersect)
  {
    Face face(Vector3d(0,0,0), Vector3d(3,0,0), Vector3d(0,3,0), 0);
    Segment segment(Vector3d(1,1,-1), Vector3d(1,1,1));

    std::vector <Point> points;
    
    face.intersect(segment, points);

    EXPECT_EQ(points.size(), 1);
  }
}
