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

  // get face segments from a triangular face
  TEST_F(CalcTest, getFaceSegments)
  {
    // face indicies
    MatrixXi F(4,3);
    F << 0,1,2, 2,1,3, 3,1,0, 0,2,3;

    // segments row=segment col=start,end
    MatrixXi S;

    // segments, expected
    MatrixXi S_exp(6,2);
    S_exp << 0,1, 0,2, 0,3, 1,2, 1,3, 2,3;

    Calc::getFaceSegments(F,S);
    
    EXPECT_EQ(S, S_exp);
  }

  TEST_F(CalcTest, sharedSegments)
  {
    Vector3i F1;
    Vector3i F2;

    F1 << 0,1,2;
    F2 << 2,0,1;
    
    EXPECT_EQ(Calc::sharedSegments(F1, F2),3);

    F1 << 0,1,2;
    F2 << 3,4,1;

    EXPECT_EQ(Calc::sharedSegments(F1, F2),0);

    F1 << 0,1,2;
    F2 << 4,2,0;

    EXPECT_EQ(Calc::sharedSegments(F1, F2),1);
  }

  TEST_F(CalcTest, toSegment)
  {
    MatrixXi F(4,3);
    F << 0,1,2, 2,1,3, 3,1,0, 0,2,3;

    unsigned int s1,s2;

    Calc::toSegment(F, 0,1,s1,s2);
    EXPECT_EQ(s1,1);
    EXPECT_EQ(s2,2);

    Calc::toSegment(F, 1,3,s1,s2);
    EXPECT_EQ(s1,2);
    EXPECT_EQ(s2,3);
  }
  
  TEST_F(CalcTest, getIntersection)
  {
    MatrixXd V1(3,3);
    MatrixXi F1(1,3);
    MatrixXd V2(4,3);
    MatrixXi F2(2,3);

    Vector3d p;
    Vector3d p_exp;
    p_exp << 0.1, 0.1, 0.0;
    
    V1 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    F1 << 0,1,2;
    V2 << 0.1, 0.1, -0.5, 0.5, 0.1, -0.5, 0.1, 0.5, -0.5, 0.1, 0.1, 0.5;
    F2 << 0,1,3,0,2,3;

    bool r = Calc::getIntersection(V1, F1, V2, F2, 0, 0, 1, p);

    EXPECT_EQ(p, p_exp);
  }

  // test if faces indicies describe same face
  TEST_F(CalcTest, FaceEqual)
  {
    Eigen::Vector3i f1;
    Eigen::Vector3i f2;

    f1 << 0,1,2;
    f2 << 0,1,3;
    EXPECT_FALSE(Calc::equal(f1,f2));

    f1 << 0,1,2;
    f2 << 0,1,2;
    EXPECT_TRUE(Calc::equal(f1,f2));

    f1 << 0,1,2;
    f2 << 2,0,1;
    EXPECT_TRUE(Calc::equal(f1,f2));

    f1 << 0,1,2;
    f2 << 1,2,0;
    EXPECT_TRUE(Calc::equal(f1,f2));
  }

  TEST_F(CalcTest, Divide)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(6,3);
    V << 0,0,0, 1,0,0, 0,1,0, 0.5,0,0, 0,0.5,0, 0.1,0.1,0;

    Eigen::MatrixXi F = Eigen::MatrixXi(1,3);
    F << 0,1,2;

    Eigen::MatrixXi P = Eigen::MatrixXi(1,3);
    P << 3,4,5;

    Eigen::VectorXi P1, P2;

    //divide(V, F, P, P1, P2);

    Eigen::VectorXi P1_exp = Eigen::VectorXi(6);
    P1_exp << 0,3,5, 0,4,5;

    Eigen::VectorXi P2_exp = Eigen::VectorXi(9);
    P2_exp << 1,2,5, 1,3,5, 2,4,5;
  }
  
  // triangulate path
  TEST_F(CalcTest, DISABLED_Triangulate)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(4,3);
    V << 0,0,0, 0,1,0, -0.7,0.7,0, 1,0,0;

    Eigen::MatrixXi P;
    Eigen::MatrixXi P_exp = Eigen::MatrixXi(2,3);
    P_exp << 0,1,2, 1,2,3;
    
    Calc::triangulate(V, P);

    EXPECT_EQ(P, P_exp);
  }

  // normal for three points in 3d space
  TEST_F(CalcTest, Normal)
  {
    Eigen::Vector3d n;
    Eigen::Vector3d v1 = Eigen::Vector3d(0.0,0.0,0.0);
    Eigen::Vector3d v2 = Eigen::Vector3d(2.0,0.0,0.0);
    Eigen::Vector3d v3 = Eigen::Vector3d(0.0,2.0,0.0);
    Eigen::Vector3d n_exp = Eigen::Vector3d(0.0,0.0,1.0);
	
    EXPECT_EQ(Calc::normal(v1,v2,v3), n_exp);

    v1 << 0.0, 0.0, 0.0;
    v2 << -1.0, 0.0, 0.0;
    v3 << 0.0, 1.0, 0.0;
    n_exp << 0.0, 0.0, 1.0;
    EXPECT_EQ(Calc::normal(v1,v2,v3), -n_exp);

    v1 << 0.0, 0.0, 0.0;
    v2 << 0.0, 1.0, 0.0;
    v3 << 0.8, 0.0, 0.6;
    n_exp << -0.6, 0.0, 0.8;
    EXPECT_EQ(Calc::normal(v1,v2,v3), -n_exp);
  }
}
