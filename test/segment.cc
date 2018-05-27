#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Segment.h"

namespace {
  using namespace Geotree;

  class SegmentTest : public ::testing::Test {
  protected:
  };

  // ::testing::AssertionResult vectorsEqual(const Vector3d v0,
  // 					  const Vector3d v1)
  // {
  //   if (v0 == v1)
  //     return ::testing::AssertionSuccess();
  //   else
  //     {
  // 	return ::testing::AssertionFailure() << v0.transpose() << " != " << v1.transpose();
  //     }
  // }
  
  TEST_F(SegmentTest, Basic)
  {
    Segment segment0(Vector3d(-1,0,0), Vector3d(1,0,0));
  }
}
