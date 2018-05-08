#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Segment.h"

namespace {
    using namespace Geotree;

    class SegmentTest : public ::testing::Test {
    protected:
    };

    TEST_F(SegmentTest, Basic)
    {
      Segment segment0(Vector3d(-1,0,0), Vector3d(1,0,0));
      Segment segment1(Vector3d(0.5,1,0), Vector3d(0.5,-1,0));
      Segment segment2(Vector3d(-1,0,0), Vector3d(1,0.1,0));
      Segment segment3(Vector3d(0.7,1,0), Vector3d(0.9,-1,0));
      Segment segment4(Vector3d(10,0,0), Vector3d(0,10,0));
      Segment segment5(Vector3d(1,11,0), Vector3d(1,1,0));

      Vector3d point;

      segment0.intersects(segment1, point);
      EXPECT_EQ(Vector3d(0.5,0,0), point);

      segment2.intersects(segment3, point);
      EXPECT_DOUBLE_EQ(0.79104477611940327, point[0]);
      EXPECT_DOUBLE_EQ(0.089552238805970172, point[1]);
      EXPECT_DOUBLE_EQ(0.0, point[2]);

      EXPECT_TRUE(segment4.intersects(segment5, point));
      EXPECT_EQ(Vector3d(1,9,0), point);
      std::cout << "point: " << point << std::endl;
    }

    TEST_F(SegmentTest, None)
    {
      Segment segment0(Vector3d(0,0,0), Vector3d(1,0,0));
      Segment segment1(Vector3d(1,11,0), Vector3d(1,1,0));

      Vector3d point;

      EXPECT_FALSE(segment0.intersects(segment1, point));
    }

    TEST_F(SegmentTest, Z)
    {
      Segment segment0(Vector3d(0,0,0), Vector3d(0,0,1));
      Segment segment1(Vector3d(0,1,0.5), Vector3d(0,-1,0.5));

      Vector3d point;

      segment0.intersects(segment1, point);
      EXPECT_EQ(Vector3d(0,0,0.5), point);
    }
  }
