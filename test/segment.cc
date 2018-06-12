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
    Segment segment0(Vector3d(-1,0,0), Vector3d(1,0,0), 0);
  }
}
