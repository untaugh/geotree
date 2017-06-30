#include "gtest/gtest.h"

#include "Calc.h"
#include <Eigen/Core>

using namespace Eigen;

namespace {

  class CalcTest : public ::testing::Test {
  };

  // get segments from a triangular face
  TEST_F(CalcTest, getSegments)
  {
    // face indicies
    MatrixXi F(1,3);
    F << 0,1,2;

    // segments row=segment col=start,end
    MatrixXi S(3,2);

    Calc::getSegments(F,S);
    
    EXPECT_EQ(S.row(0)[0], 0);
    EXPECT_EQ(S.row(0)[1], 1);
    EXPECT_EQ(S.row(1)[0], 1);
    EXPECT_EQ(S.row(1)[1], 2);
    EXPECT_EQ(S.row(2)[0], 2);
    EXPECT_EQ(S.row(2)[1], 0);
  }

  // get segments from a triangular face
  TEST_F(CalcTest, getSegments2)
  {
    // face indicies
    MatrixXi F(2,3);
    F << 0,1,2,7,5,3;

    // segments row=segment col=start,end
    MatrixXi S;

    // segments, expected
    MatrixXi S_exp(6,2);
    S_exp << 0,1,1,2,2,0,7,5,5,3,3,7;
      
    Calc::getSegments(F,S);
    
    EXPECT_EQ(S, S_exp);
  }
}
