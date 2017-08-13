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

  // get all segments (face index) from faces
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

  // number of shared segments in faces
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

  // face indicies to segment indicies
  TEST_F(CalcTest, toSegment)
  {
    MatrixXi F(4,3);
    F << 0,1,2, 2,1,3, 3,1,0, 0,2,3;

    unsigned int s1,s2;

    Calc::toSegment(F, 0, 1, s1, s2);
    EXPECT_EQ(s1,1);
    EXPECT_EQ(s2,2);

    Calc::toSegment(F, 1, 3, s1, s2);
    EXPECT_EQ(s1,2);
    EXPECT_EQ(s2,3);
  }

  // intersection face and segment
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
    EXPECT_TRUE(r);
  }

  // intersection face and segment
  TEST_F(CalcTest, getIntersectionNew)
  {
    MatrixXd V1(3,3);
    MatrixXd S(2,3);	

    Vector3d p;
    Vector3d p_exp;
    p_exp << 0.1, 0.1, 0.0;
    
    V1 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    S << 0.1, 0.1, -0.5, 0.1, 0.1, 0.5;
    
    bool r = Calc::getIntersection(V1, S, p);

    EXPECT_EQ(p, p_exp);
    EXPECT_TRUE(r);
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
  TEST_F(CalcTest, TriangulateBasic)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(4,3); // verticies
    Eigen::VectorXi P = Eigen::VectorXi(4); // path
    Eigen::MatrixXi F; // resulting faces
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(2,3); // expected
    
    V << 0,0,0, 1,0,0, 0.7,0.7,0, 0,1,0;
    P << 0,1,2,3;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_EQ(Calc::crossing(V,P),0);
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // reverse
    V << 0,0,0, 0,1,0, 0.7,0.7,0, 1,0,0;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // X plane
    V << 0,0,0, 0,1,0, 0.0,0.7,0.3, 0,0,1.6;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);
  }

  // triangulate path outside
  TEST_F(CalcTest, TriangulateOutside)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(6,3); // verticies
    Eigen::VectorXi P = Eigen::VectorXi(6); // path
    Eigen::MatrixXi F; // resulting faces
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(4,3); // expected

    V << 0,0,0, 0.5,0.1,0, 0.1,0.5,0, 0,0.1,0, 0,1,0, 1,0,0;
    P << 0,1,2,3,4,5;
    F_exp << 2,3,4, 5,0,1, 1,2,4, 4,5,1;
    EXPECT_EQ(1, Calc::crossing(V,P));
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F.rows(),4);
    EXPECT_EQ(F, F_exp);
  }

  // triangulate path
  TEST_F(CalcTest, TriangulateSkip)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(4,3); // verticies
    Eigen::VectorXi P = Eigen::VectorXi(4); // path
    Eigen::MatrixXi F;
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(2,3);

    V << 1,0,0, 0.0,0.0,0, 0.0,1.0,0, 0.1,0.1,0;
    P << 0,1,2,3;
    F_exp << 1,2,3, 3,0,1;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);
  }

  // triangulate invalid
  TEST_F(CalcTest, TriangulateInvalid)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(4,3);
    Eigen::VectorXi P = Eigen::VectorXi(4); // path
    Eigen::MatrixXi F;
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(2,3);

    V << 0,0,0, 1,0,0, 1,1,0, 0,-1,0;    
    P << 0,1,2,3;
    F_exp << 0,1,2, 2,3,0;
    bool r = Calc::triangulate(V, P, F);
    EXPECT_FALSE(r);
  }

  // triangulate path
  TEST_F(CalcTest, TriangulateLonger)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(10,3);
    Eigen::VectorXi P = Eigen::VectorXi(10); // path
    Eigen::MatrixXi F;
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(8,3);

    P << 0,1,2,3,4,5,6,7,8,9;
    
    V << 0.0, 0.0, 0.0,
      -0.1, 1.0, 0.0,
      0.1, 2.0, 0.0,
      0.5, 0.5, 0.0,
      0.8, 2.4, 0.0,
      0.9, 0.3, 0.0,
      0.7, -0.4, 0.0,
      -1.0, -1.0, 0.0,
      -1.1, -1.2, 0.0,
      -2.0, 0.3, 0.0;

    F_exp << 0, 1, 2,
      3, 4, 5,
      5, 6, 7,
      7, 8, 9,
      0, 2, 3,
      7, 9, 0,
      0, 3, 5,
      5, 7, 0;
    
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);
  }

  // triangulate path with hole
  TEST_F(CalcTest, TriangulateHole)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(8,3);
    Eigen::VectorXi P = Eigen::VectorXi(10); // path
    Eigen::MatrixXi F;
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(8,3);

    P << 0,1,2,3,4,5,6,7,4,3;
    
    V << 0.0, 0.0, 0.0,
      2.0, 0.0, 0.0,
      2.0, 2.0, 0.0,
      0.0, 2.0, 0.0,
      0.5, 1.5, 0.0,
      1.5, 1.5, 0.0,
      1.5, 0.5, 0.0,
      0.5, 0.5, 0.0;

    F_exp << 2, 3, 4,
      7, 4, 3,
      2, 4, 5,
      7, 3, 0,
      1, 2, 5,
      6, 7, 0,
      1, 5, 6,
      6, 0, 1;

    EXPECT_TRUE(Calc::triangulate(V, P, F));
    //EXPECT_EQ(F, F_exp);
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
  
  // distance between segments in 3d space
  TEST_F(CalcTest, DistanceSegments)
  {
    Eigen::Vector3d v1a; // segment 1 start
    Eigen::Vector3d v1b; // segment 1 end
    Eigen::Vector3d v2a; // segment 2 start
    Eigen::Vector3d v2b; // segment 2 end
    double d; // distance

    // intersecting
    v1a << -1.0, 0.0, 0.0;
    v1b << 1.0, 0.0, 0.0;
    v2a << 0.0, -1.0, 0.0;
    v2b << 0.0, 1.0, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(0.0, d);

    // intersecting
    v1a << -1.0, 0.0, 3.0;
    v1b << 1.0, 0.0, 3.0;
    v2a << 0.0, -1.0, 3.0;
    v2b << 0.0, 1.0, 3.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(0.0, d);

    // not intersecting
    v1a << -1.0, 0.0, 0.0;
    v1b << 1.0, 0.0, 0.0;
    v2a << 0.0, -1.0, 0.0;
    v2b << 0.0, 1.0, 0.1;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_NEAR(0.05, d, 0.0001);

    // s2 away from intersection
    v1a << -1.0, 0.0, 0.0;
    v1b << 1.0, 0.0, 0.0;
    v2a << 0.0, 0.001, 0.0;
    v2b << 0.0, 2.0, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(0.001, d);

    // s2 away from intersection
    v1a << -1.0, 0.0, 0.0;
    v1b << 1.0, 0.0, 0.0;
    v2a << 0.0, 3.0, 0.0;
    v2b << 0.0, 0.003, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(0.003, d);

    // s1 away from intersection
    v1a << 0.0, 0.0, 0.0;
    v1b << 5.0, 0.0, 0.0;
    v2a << 6.5, 1.0, 0.0;
    v2b << 6.5, -2.0, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(1.5, d);

    // z distance 2.0
    v1a << 0.0, 0.0, 1.0;
    v1b << 1.0, 0.0, 1.0;
    v2a << 0.0, 0.0, -1.0;
    v2b << 0.0, 1.0, -1.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(2.0, d);

    // both segments not on intersection
    v1a << 1.0, 0.0, 0.0;
    v1b << 2.0, 0.0, 0.0;
    v2a << 0.0, 1.0, 0.0;
    v2b << 0.0, 2.0, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(sqrt(2.0), d);
  }

  // parallell and point segments
  TEST_F(CalcTest, DISABLED_DistanceSegmentsOther)
  {
    Eigen::Vector3d v1a; // segment 1 start
    Eigen::Vector3d v1b; // segment 1 end
    Eigen::Vector3d v2a; // segment 2 start
    Eigen::Vector3d v2b; // segment 2 end
    double d; // distance

    // parallel
    v1a << 0.0, 0.0, 0.0;
    v1b << 1.0, 0.0, 0.0;
    v2a << 0.0, 0.5, 0.0;
    v2b << 1.0, 0.5, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(0.5, d);

    // parallel
    v1b << 0.0, 0.0, 0.0;
    v1b << 1.1, 2.3, 4.2;
    v2a << 0.0, 0.0, 0.0;
    v2b = v1a * -2;

    v1a += Vector3d(4,5,6);
    v1b += Vector3d(4,5,6);
    v2a += Vector3d(10,3,1);
    v2b += Vector3d(10,3,1);
    
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(0.5, d);

    // parallel same
    v1a << 0.0, 0.0, 0.0;
    v1b << 1.0, 0.0, 0.0;
    v2a << 0.0, 0.5, 0.0;
    v2b << 1.0, 0.5, 0.0;
    d = Calc::distance(v1a, v1b, v1a, v1b);
    EXPECT_DOUBLE_EQ(0.0, d);

    // parallel not same
    v1a << 1.0, 0.0, 0.0;
    v1b << 2.0, 0.0, 0.0;
    v2a << -1.0, 0.5, 0.0;
    v2b << -2.0, 0.5, 0.0;
    d = Calc::distance(v1a, v1b, v1a, v1b);
    EXPECT_DOUBLE_EQ(2.0, d);
    
    // not segment
    v1a << 1.0, 1.0, 0.0;
    v1b << 1.0, 1.0, 0.0;
    v2a << 0.0, 0.0, 0.0;
    v2b << 2.0, 0.0, 0.0;
    d = Calc::distance(v1a, v1b, v1a, v1b);
    EXPECT_DOUBLE_EQ(1.0, d);
  }

  // is point inside triangular face
  TEST_F(CalcTest, pointInside)
  {
    Vector3d v1, v2, v3, p;

    v1 << 0.0, 0.0, 0.0;
    v2 << 1.0, 0.0, 0.0;
    v3 << 0.0, 1.0, 0.0;
    p << 0.1, 0.1, 0.0;
    EXPECT_TRUE(Calc::inside(v1,v2,v3,p));
    p << 0.7, 0.7, 0.0;
    EXPECT_FALSE(Calc::inside(v1,v2,v3,p));
    p << 1.0, 0.0, 0.0;
    EXPECT_TRUE(Calc::inside(v1,v2,v3,p));
  }

  // is any point inside triangular face
  TEST_F(CalcTest, pointsInside)
  {
    Vector3d v1, v2, v3;
    MatrixXd P = MatrixXd(5,3);
    
    v1 << 0.0, 0.0, 0.0;
    v2 << 1.0, 0.0, 0.0;
    v3 << 0.0, 1.0, 0.0;
    P << 1.1, 0.0, 0.0,
      0.0, 1.1, 0.0,
      0.6, 0.6, 0.0,
      0.7, 10.1, 0.0,
      1.0, -0.1, 0.0;
    EXPECT_FALSE(Calc::inside(v1,v2,v3,P));

    P << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.7, 10.1, 0.0,
      1.0, -0.1, 0.0;
    EXPECT_FALSE(Calc::inside(v1,v2,v3,P));

    
    P << 1.1, 0.0, 0.0,
      0.0, 0.9, 0.0,
      0.6, 0.6, 0.0,
      0.7, 10.1, 0.0,
      1.0, -0.1, 0.0;
    EXPECT_TRUE(Calc::inside(v1,v2,v3,P));
  }
  

  // angle between segments
  TEST_F(CalcTest, angle)
  {
    Vector3d v1, v2, v3, up;

    // 270
    v1 << 0.0, 0.0, 0.0;
    v2 << 1.0, 0.0, 0.0;
    v3 << 1.0, 1.0, 0.0;
    up << 0.0, 0.0, 1.0;
    EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), M_PI*3/2);

    //90
    v1 << 0.0, 0.0, 0.0;
    v2 << 1.0, 0.0, 0.0;
    v3 << 1.0, -1.0, 0.0;
    up << 0.0, 0.0, 1.0;
    EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), M_PI/2);

    //90
    v1 << 0.0, 0.0, 0.0;
    v2 << -0.5, 0.5, 0.0;
    v3 << 0.0, 1.0, 0.0;
    up << 0.0, 0.0, 1.0;
    EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), M_PI/2);

    //90
    v1 << 0.0, 0.0, 0.0;
    v2 << 0.0, -1.0, 0.0;
    v3 << -1.0, -1.0, 0.0;
    up << 0.0, 0.0, 1.0;
    EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), M_PI/2);
    
    v1 << 0.0, 0.0, 0.0;
    v2 << 1.0, 0.0, 0.0;
    v3 << 0.0, -1.0, 0.0;
    up << 0.0, 0.0, 1.0;
    EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), M_PI/4);

    v1 << 0.0, 0.0, 0.0;
    v2 << 1.0, 0.0, 0.0;
    v3 << 2.0, 0.0, 0.0;
    up << 0.0, 0.0, 1.0;
    EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), M_PI);

    v1 << 0.0, 0.0, 0.0;
    v2 << 1.0, 0.0, 0.0;
    v3 << 2.0, 0.0, 0.0;
    up << 0.0, 0.0, 1.0;
    EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), M_PI);

    v1 << 0.0, 0.0, 0.0;
    v2 <<  -0.1, 1.0, 0.0;
    v3 << 0.1, 2.0, 0.0;
    EXPECT_LE(Calc::angle(v1,v2,v3, up), M_PI);

    v1 << 0.0, 0.0, 0.0;
    v2 <<  -0.1, 1.0, 0.0;
    v3 << -0.3, 2.0, 0.0;
    EXPECT_GE(Calc::angle(v1,v2,v3, up), M_PI);
  }

  // angle between segments
  TEST_F(CalcTest, AngleLoop)
  {
    Vector3d v1, v2, v3, up;

    int n = 20;

    up << 0.0, 0.0, 1.0;

    // do a circle
    for (int i=0; i<n; i++)
      {
	double a = i*2*M_PI/n;

	v1 << 1.0, 0.0, 0.0;
	v2 << 0.0, 0.0, 0.0;
	v3 << cos(a), sin(a), 0.0;

	EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), a);
      }

    // offset
    for (int i=0; i<n; i++)
      {
	double a = i*2*M_PI/n;

	v1 << -1.0, -2.0, 0.0;
	v2 << -2.0, -2.0, 0.0;
	v3 << cos(a)-2.0, sin(a)-2.0, 0.0;

	EXPECT_DOUBLE_EQ(Calc::angle(v1,v2,v3, up), a);
      }
  }

  // next point
  TEST_F(CalcTest, nextPoint)
  {
    Eigen::VectorXi P = Eigen::VectorXi(10);
    std::set<int> skip;
    int i;

    i = 0;
    bool ret = Calc::next(P, i, skip);
    EXPECT_EQ(i, 0);
    EXPECT_TRUE(ret);

    skip.insert(0);
    skip.insert(1);

    ret = Calc::next(P, i, skip);
    EXPECT_EQ(i, 2);
    EXPECT_TRUE(ret);

    skip.insert(9);

    i = 9;
    ret = Calc::next(P, i, skip);
    EXPECT_EQ(i, 2);
    EXPECT_TRUE(ret);

    i = 10;
    ret = Calc::next(P, i, skip);
    EXPECT_EQ(i, 2);
    EXPECT_TRUE(ret);

    skip.insert(2);
    skip.insert(3);
    skip.insert(4);
    skip.insert(5);
    skip.insert(6);
    skip.insert(7);
    skip.insert(8);

    i = 0;
    ret = Calc::next(P, i, skip);
    EXPECT_FALSE(ret);

    i = 5;
    ret = Calc::next(P, i, skip);
    EXPECT_FALSE(ret);
  }

  // list all faces connected to this face
  TEST_F(CalcTest, ConnectedFaces)
  {
    MatrixXi F = MatrixXi(3,3);

    F << 0,1,2, 1,2,3, 2,3,4;
    
    std::set <int> skip;
    
    std::set <int> c = Calc::connected(F, skip, 0);
    std::set <int> c_exp = {0,1,2};

    EXPECT_EQ(c, c_exp);
  }

  TEST_F(CalcTest, ConnectedFaces2)
  {
    MatrixXi F = MatrixXi(5,3);

    F << 8,7,6, 1,2,3, 3,4,5, 4,5,6, 0,1,2;
    
    std::set <int> skip;
    
    std::set <int> c = Calc::connected(F, skip, 0);
    std::set <int> c_exp = {0,1,2,3,4};
    EXPECT_EQ(c, c_exp);
    
    skip.insert(2);

    c = Calc::connected(F, skip, 0);
    c_exp.clear();
    c_exp = {0,3};
    EXPECT_EQ(c, c_exp);

    c = Calc::connected(F, skip, 4);
    c_exp.clear();
    c_exp = {1,4};
    EXPECT_EQ(c, c_exp);
  }

  TEST_F(CalcTest, ConnectedFaces3)
  {
    MatrixXi F = MatrixXi(100,3);
    std::set <int> c_exp, c_exp2;
    std::set <int> skip;
    
    for (int i=0; i<100; i++)
      {
	F.row(i) << i, i+1, i+2;
	c_exp.insert(i);

	if (i > 25)
	  {
	    c_exp2.insert(i);
	  }
      }

    std::set <int> c = Calc::connected(F, skip, 0);
    EXPECT_EQ(c, c_exp);

    skip.insert(24);
    skip.insert(25);

    c = Calc::connected(F, skip, 65);
    EXPECT_EQ(c, c_exp2);    
  }

  // basix test
  TEST_F(CalcTest, BoundingBox)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(3,3);
    Eigen::MatrixXi F = Eigen::MatrixXi(1,3);
    Eigen::Vector3d B1,B2, B1_exp, B2_exp;

    V << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.5,
      0.0, 1.0, 0.0;
    F << 0,1,2;
    B1_exp << 0.0, 0.0, 0.0;
    B2_exp << 1.0, 1.0, 0.5;
    
    Calc::boundingBox(V, F, B1, B2);
    EXPECT_EQ(B1, B1_exp);
    EXPECT_EQ(B2, B2_exp);
  }

  // negative coordinates and skip a vertex
  TEST_F(CalcTest, BoundingBox2)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(6,3);
    Eigen::MatrixXi F = Eigen::MatrixXi(2,3);
    Eigen::Vector3d B1,B2, B1_exp, B2_exp;

    V << -10.2, 3.0, 1.0,
      12.4, 7.6, 5.5,
      -10.21, 1.2, 5.5,
      12.4, 6.4, 4.0,
      0.04, -0.2, 15.5,
      1.2, 1.0, 0.01;

    F << 0,1,2, 0,3,5;
    B1_exp << -10.21, 1.0, 0.01;
    B2_exp << 12.4, 7.6, 5.5;
    
    Calc::boundingBox(V, F, B1, B2);
    EXPECT_EQ(B1, B1_exp);
    EXPECT_EQ(B2, B2_exp);
  }

  // project 3d path in plane to 2d path, widest axes
  // not preserving size
  TEST_F(CalcTest, DropAxisBasic)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(3,3);
    Eigen::MatrixXi F = Eigen::MatrixXi(1,3);    

    V << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.1,
      0.0, 1.0, 0.1;
    F << 0,1,2;

    EXPECT_EQ(Calc::minAxis(V, F), AXIS_Z);

    V << 0.0, 0.0, 0.0,
      2.0, 0.0, 5.0,
      0.0, 1.0, 5.0;
    F << 0,1,2;

    EXPECT_EQ(Calc::minAxis(V, F), AXIS_Y);

    V << 0.0, 0.0, 0.0,
      1.0, 0.0, 5.0,
      0.0, 2.0, 5.0;
    F << 0,1,2;

    EXPECT_EQ(Calc::minAxis(V, F), AXIS_X);    
  }
  
  TEST_F(CalcTest, IntersectLineSegment)
  {
    Segment line, segment;
    int winding;

    // no intersect
    winding = 0;
    line << 0.0, 0.0, 1.0, 1.0;
    segment << 0.0, 1.0, 1.0, 2.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    // intersect
    winding = 0;
    line << 0.0, 0.0, 1.0, 1.0;
    segment << 1.0, 0.0, 1.0, 3.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -1);

    // from X line
    winding = 0;
    line << 0.0, 0.0, 1.0, 0.0; // same
    segment << 1.0, 0.0, 1.0, 1.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    winding = 0;
    segment << 1.0, 0.0, 1.0, -1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);

    winding = 0;
    segment << -1.0, 0.0, -1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);

    winding = 0;
    segment << -1.0, 0.0, -1.0, -1.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    winding = 0;
    segment << 1.0, 0.0, -1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -1);

    winding = 0;
    segment << 1.0, 0.0, -1.0, -1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 2);
    
    // from Y line
    winding = 0;
    segment << 0.0, 1.0, 1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);

    winding = 0;
    segment << 0.0, 1.0, -1.0, 1.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    winding = 0;
    segment << 0.0, -1.0, -1.0, -1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);

    winding = 0;
    segment << 0.0, -1.0, 1.0, -1.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    winding = 0;
    segment << 0.0, 1.0, -1.0, -1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -1);

    winding = 0;
    segment << 0.0, 1.0, 1.0, -1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 2);    

    // segment on line
    winding = 0;
    line << 0.0, 0.0, 1.0, 0.0; 
    segment << 1.0, 0.0, 2.0, 0.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    // large number
    winding = 0;
    line << 12345678.0, 12345678.0, 12345678.5, 12345678.5;
    segment << 12345679.0, 12345678.0, 12345679.0, 12345680.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -1);
    winding = 0;
    segment << 12345679.0, 12345680.0, 12345679.0, 12345678.0; // swap
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);

    // same start
    winding = 0;
    line << 0.0, 0.0, 1.0, 0.0; 
    segment << 0.0, 0.0, 0.0, 1.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    // no line
    winding = 0;
    line << 1.0, 0.0, 1.0, 0.0; 
    segment << 2.0, 0.0, 1.0, 1.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    // no segment
    winding = 0;
    line << 0.0, 0.0, 1.0, 0.0; 
    segment << 2.0, 0.0, 2.0, 0.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    // to X line
    winding = 0;
    line << 0.0, 0.0, 1.0, 0.0; // same
    segment << 1.0, 1.0, 1.0, 0.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    winding = 0;
    segment << 1.0, -1.0, 1.0, 0.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -1);

    winding = 0;
    segment << -1.0, 1.0, -1.0, 0.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -1);

    winding = 0;
    segment << -1.0, -1.0, -1.0, 0.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    winding = 0;
    segment << -1.0, 1.0, 1.0, 0.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);

    winding = 0;
    segment << -1.0, -1.0, 1.0, 0.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -2);

    // to Y line
    winding = 0;
    segment << 1.0, 1.0, 0.0, 1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -1);

    winding = 0;
    segment << -1.0, 1.0, 0.0, 1.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    winding = 0;
    segment << -1.0, -1.0, 0.0, -1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -1);

    winding = 0;
    segment << 1.0, -1.0, 0.0, -1.0;
    EXPECT_FALSE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 0);

    winding = 0;
    segment << -1.0, -1.0, 0.0, 1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);

    winding = 0;
    segment << 1.0, -1.0, 0.0, 1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, -2);
  }
  
  // line intersect
  TEST_F(CalcTest, DISABLED_IntersectLineSegment2)
  {
    Vector2d l1, l2, s1, s2;

    int wind;

    wind = 0;
    l1 << 0.0, 0.0; l2 << 1.0, 1.0; s1 << 2.0, -1.0; s2 << -1.0, 3.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, -1);
    wind = 0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s2, s1, wind)); // swap s1,s2
    EXPECT_EQ(wind, 1);

    // from the line
    wind = 0;
    l1 << 0.0, 0.0; l2 << 1.0, 1.0; s1 << 2.0, 2.0; s2 << 3.0, 0.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, 1);
    wind = 0;
    // to the line
    EXPECT_FALSE(Calc::intersect(l1, l2, s2, s1, wind)); //swap s1,s2
    EXPECT_EQ(wind, 0);
    s2 << 1.0, 3.0;
    wind = 0;
    // from the line
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, -1);

    // same line
    l1 << 0.0, 0.0; l2 << 1.0, 0.0; s1 << 0.0, 0.0; s2 << 1.0, 0.0;
    EXPECT_FALSE(Calc::intersect(l1, l2, s1, s2, wind));

    // same start
    l1 << 0.0, 0.0; l2 << 1.0, 0.0; s1 << 0.0, 0.0; s2 << 0.0, 1.0;
    EXPECT_FALSE(Calc::intersect(l1, l2, s1, s2, wind));
    
    l1 << 0.0, 0.0; l2 << 1.0, 0.0; s1 << 0.0, -1.0; s2 << 0.0, 1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));

    l1 << 0.0, 0.0; l2 << 1.0, 1.0; s1 << 1.0, -1.0; s2 << -1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));

    wind = 0;
    l1 << 0.0, 0.0; l2 << 1.0, 0.0; s1 << 0.0, 1.0; s2 << 1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, 1);

    // large number
    wind = 0;
    l1 << 12345678.0, 12345678.0; l2 << 12345678.5, 12345678.5;
    s1 << 12345679.0, 12345678.0; s2 << 12345679.0, 12345680.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, -1);
    wind = 0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s2, s1, wind));
    EXPECT_EQ(wind, 1);

    // winding number x
    l1 << 0.0, 0.0; l2 << 1.0, 0.0; 
    wind = 0;
    s1 << 1.0, -1.0; s2 << 1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, -1);
    wind = 0;
    s1 << -1.0, -1.0; s2 << -1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, 1);
    wind = 0;
    s1 << 1.0, 1.0; s2 << 1.0, -1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, 1);
    wind = 0;
    s1 << -1.0, 1.0; s2 << -1.0, -1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, -1);

    // winding number y
    wind = 0;
    s1 << 1.0, -1.0; s2 << -1.0, -1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, 1);
    wind = 0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s2, s1, wind));
    EXPECT_EQ(wind, -1);
    wind = 0;
    s1 << -1.0, 1.0; s2 << 1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, 1);
    wind = 0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s2, s1, wind));
    EXPECT_EQ(wind, -1);

    // winding number xy
    wind = 0;
    s1 << 1.0, -1.0; s2 << -2.0, 1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, 2);
    wind = 0;
    s1 << 1.0, -1.0; s2 << -1.0, 2.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, -2);

    wind = 0;
    l1 << 0.0, 0.0; l2 << -0.1, 1.0; s1 << -1.0, 1.0; s2 << 1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(l1, l2, s1, s2, wind));
    EXPECT_EQ(wind, 1);
  }
  
  TEST_F(CalcTest, Crossing)
  {

    MatrixXd V(24,3);
    VectorXi P1(8);
    VectorXi P2(13);
    
    V <<   0,   0,  0,
      0,  10,   0,
      10,  10,   0,
      10,   0,   0,
      0,   0,  10,
      0,  10,  10,
      10,  10,  10,
      10,   0,  10,
      1,   1,   8,
      1,   2,   8,
      11,   2,   8,
      11,   1,   8,
      1,   1,   9,
      1,   2,   9,
      11,   2,   9,
      11,   1,   9,
      10, 1.9,   8,
      10,   2,   8,
      10,   1,   8,
      10,   2, 8.1,
      10,   2,   9,
      10,   1, 8.9,
      10,   1,   9,
      10, 1.9,   9;
    
    P1 << 16, 17, 19, 20, 23, 22, 21, 18;
    P2 << 16, 17, 19, 20, 23, 22, 21, 18, 3, 6, 7, 3, 18;
    
    std::cout << Calc::crossing(V, P1) << std::endl;
    std::cout << Calc::crossing(V, P2) << std::endl;
  }
}
