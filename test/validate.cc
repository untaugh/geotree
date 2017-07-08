#include "gtest/gtest.h"
#include <Eigen/Core>

#include "Validate.h"

namespace {
  class ValidateTest : public ::testing::Test {
  };

  // test verticies form a valid triangular face
  TEST_F(ValidateTest, TriangularFace)
  {
    Eigen::Matrix3d face;    
    face << 1,0,0,2,0,0,3,0,0;

    EXPECT_TRUE(Geotree::Validate::face(face));
  }

  TEST_F(ValidateTest, Verticies)
  {
    const int num = 4;
    Eigen::Matrix3d f[num];

    // dupliate verticies
    f[0] << 0,0,0,0,0,0,0,0,0;
    f[1] << 1,2,3,1,2,3,4,5,6;
    f[2] << 0,0,1,32,75,10,32,75,10;
    f[3] << -34,21,19,-34,19,21,-34,21,19;
    
    for (int i=0; i<num; i++)
      {
	EXPECT_FALSE(Geotree::Validate::face(f[i]));	
      }
  }

  
}
