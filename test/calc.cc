#include "gtest/gtest.h"
#include "Calc.h"
#include "Node.h"
#include <Eigen/Core>

using namespace Eigen;

namespace {

  class CalcTest : public ::testing::Test {
  };

  // get segments from a triangular face
  TEST_F(CalcTest, getSegments)
  {
    // face indicies
    Faces F(1,3);
    F << 0,1,2;

    // segments row=segment col=start,end
    MatrixXi S;

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
    Faces F(4,3);
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
    Faces F(4,3);
    F << 0,1,2, 2,1,3, 3,1,0, 0,2,3;

    unsigned int s1,s2;

    Calc::toSegment(F, 0, 1, s1, s2);
    EXPECT_EQ(s1,1);
    EXPECT_EQ(s2,2);

    Calc::toSegment(F, 1, 3, s1, s2);
    EXPECT_EQ(s1,2);
    EXPECT_EQ(s2,3);
  }

  TEST_F(CalcTest, Intersection)
  {
    Verticies F(3,3);
    Verticies S(2,3);
    Vertex P;
    
    F << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0;
    S << 0.0, 0.0, -1.0,
      0.0, 0.0, 1.0;

    EXPECT_TRUE(Calc::intersectsFace(F,S));

    F << 0,0,0,
      0,10,0,
      10,10,0;
    S << 1,1,1,
      -9, -9, -9;
    EXPECT_TRUE(Calc::intersectsFace(F,S));
  }
  
  // intersection face and segment
  TEST_F(CalcTest, IntersectionNew)
  {
    MatrixXd F(3,3);
    MatrixXd S(2,3);	
    Vector3d p_exp;
    
    p_exp << 0.1, 0.1, 0.0;
    
    F << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    S << 0.1, 0.1, -0.5, 0.1, 0.1, 0.5;

    EXPECT_TRUE(Calc::intersectsFace(F,S));
    EXPECT_EQ(Calc::intersection(F,S), p_exp);
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
    //Eigen::MatrixXi F(0); // resulting faces
    Faces F(0,3); // resulting faces
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(2,3); // expected
    
    V << 0,0,0, 1,0,0, 0.7,0.7,0, 0,1,0;
    P << 0,1,2,3;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_EQ(Calc::winding(V,P),-1);
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // reverse
    V << 0,0,0, 0,1,0, 0.7,0.7,0, 1,0,0;
    F = Faces();;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_EQ(Calc::winding(V,P),1);
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // X plane
    V << 0,0,0, 0,1,0, 0.0,0.7,0.3, 0,0,1.6;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // backwards
    V << 0,0,0, 1,0,0,  -1.0,-0.1,0.0, 1,1,0;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // 180 start
    V << 0,0,0, 1,0,0, 2,0,0, 1,1,0;
    F_exp << 1,2,3, 3,0,1;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // normal calc 180 test1
    V << 0,0,0, 1,0,0, 0,1,0, -1,0,0;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // normal calc 180 test2
    V << 0,0,0, 1,0,0, 0,-1,0, -1,0,0;
    F_exp << 0,1,2, 2,3,0;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);
  }

  TEST_F(CalcTest, TriangulateNormalTest)
  {
    Verticies V(6,3);
    Path P(6);
    //Faces P(2,3);
    Faces F(0,3);
    Faces F_exp(4,3);
    
    V << 0,0,0, 1,0,0, 2,-1,0, 2,1,0, 1,2,0, 0,1,0;
    P << 0,1,2,3,4,5;
    F_exp << 1,2,3, 3,4,5, 5,0,1, 1,3,5;
    EXPECT_EQ(-1, Calc::winding(V,P));
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    V << 0,0,0, 1,0,0, 2,-1,0, 2,1,0, 1,2,0, -0.1,0,0;
    P << 0,1,2,3,4,5;
    F_exp << 1,2,3, 3,4,5, 0,1,3, 3,5,0;
    EXPECT_EQ(-2, Calc::winding(V,P));
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // normal != 0,0,0?
    V << 0,0,0, 1,0,0, 2,0.1,0, 2,1,0, 1,2,0, -0.1,0.001,0;
    P << 0,1,2,3,4,5;
    F_exp << 0,1,2, 2,3,4, 4,5,0, 0,2,4;
    EXPECT_EQ(-1, Calc::winding(V,P));
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // normal = 0,0,0?
    V << 0,0,0, 1,0,0, 2,0.1,0, 2,1,0, 1,2,0, -0.1,0,0;
    P << 0,1,2,3,4,5;
    F_exp << 0,1,2, 2,3,4, 4,5,0, 0,2,4;
    EXPECT_EQ(-2, Calc::winding(V,P));
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F, F_exp);

    // no normal, all on same line
    V << 0,0,0, 1,0,0, 2,0,0, -3,0,0, 10,0,0, -0.1,0,0;
    P << 0,1,2,3,4,5;
    EXPECT_FALSE(Calc::triangulate(V, P, F));
  }
  // triangulate path outside
  TEST_F(CalcTest, TriangulateOutside)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(6,3); // verticies
    Eigen::VectorXi P = Eigen::VectorXi(6); // path
    Faces F(0,3); // resulting faces
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(4,3); // expected

    V << 0,0,0, 0.5,0.1,0, 0.1,0.5,0, 0,0.1,0, 0,1,0, 1,0,0;
    P << 0,1,2,3,4,5;
    F_exp << 2,3,4, 5,0,1, 1,2,4, 4,5,1;
    EXPECT_EQ(1, Calc::winding(V,P));
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    EXPECT_EQ(F.rows(),4);
    EXPECT_EQ(F, F_exp);
  }

  // triangulate path
  TEST_F(CalcTest, TriangulateSkip)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(4,3); // verticies
    Eigen::VectorXi P = Eigen::VectorXi(4); // path
    Faces F(0,3);
    Eigen::MatrixXi F_exp = Eigen::MatrixXi(2,3);

    V << 1,0,0, 0.0,0.0,0, 0.0,1.0,0, 0.1,0.1,0;
    P << 0,1,2,3;
    F_exp << 1,2,3, 3,0,1;
    EXPECT_TRUE(Calc::triangulate(V, P, F));
    ASSERT_EQ(F.size(), F_exp.size());
    EXPECT_EQ(F, F_exp);
  }

  // triangulate invalid
  TEST_F(CalcTest, TriangulateInvalid)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(4,3);
    Eigen::VectorXi P = Eigen::VectorXi(4); // path
    Faces F(0,3);
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

    EXPECT_EQ(4, Calc::winding(V, P));
    //EXPECT_TRUE(Calc::triangulate(V, P, F));
    //EXPECT_EQ(F, F_exp);
  }

  // triangulate path with hole
  TEST_F(CalcTest, TriangulateHole)
  {
    Eigen::MatrixXd V = Eigen::MatrixXd(8,3);
    Eigen::VectorXi P = Eigen::VectorXi(10); // path
    Faces F(0,3);
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
    Line line1, line2;
    
    // intersecting
    line1 << -1.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    line2 << 0.0, -1.0, 0.0, 0.0, 1.0, 0.0;
    EXPECT_DOUBLE_EQ(0.0, Calc::distance(line1, line2));

    // intersecting
    line1 << -1.0, 0.0, 3.0, 1.0, 0.0, 3.0;
    line2 << 0.0, -1.0, 3.0, 0.0, 1.0, 3.0;
    EXPECT_DOUBLE_EQ(0.0, Calc::distance(line1, line2));

    // not intersecting
    line1 << -1.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    line2 << 0.0, -1.0, 0.0, 0.0, 1.0, 0.1;
    EXPECT_NEAR(0.05, Calc::distance(line1, line2), 0.0001);

    // s2 away from intersection
    line1 << -1.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    line2 << 0.0, 0.001, 0.0, 0.0, 2.0, 0.0;
    EXPECT_DOUBLE_EQ(0.001, Calc::distance(line1, line2));

    // s2 away from intersection
    line1 << -1.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    line2 << 0.0, 3.0, 0.0, 0.0, 0.003, 0.0;
    EXPECT_DOUBLE_EQ(0.003, Calc::distance(line1, line2));

    // s1 away from intersection
    line1 << 0.0, 0.0, 0.0, 5.0, 0.0, 0.0;
    line2 << 6.5, 1.0, 0.0, 6.5, -2.0, 0.0;
    EXPECT_DOUBLE_EQ(1.5, Calc::distance(line1, line2));

    // z distance 2.0
    line1 << 0.0, 0.0, 1.0, 1.0, 0.0, 1.0;
    line2 << 0.0, 0.0, -1.0, 0.0, 1.0, -1.0;
    EXPECT_DOUBLE_EQ(2.0, Calc::distance(line1, line2));

    // both segments not on intersection
    line1 << 1.0, 0.0, 0.0, 2.0, 0.0, 0.0;
    line2 << 0.0, 1.0, 0.0, 0.0, 2.0, 0.0;
    EXPECT_DOUBLE_EQ(sqrt(2.0), Calc::distance(line1, line2));
  }

  // parallell and point segments
  TEST_F(CalcTest, DistanceSegmentsOther)
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

    // parallel z
    v1a << 0.0, 0.0, 0.0;
    v1b << 0.0, 0.0, 1.0;
    v2a << 2.0, 3.0, 0.0;
    v2b << 2.0, 3.0, 0.5;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(Vertex(2,3,0).norm(), d);

    // parallel
    v1a << 0.0, 0.0, 0.0;
    v1b << 1.1, 2.3, 4.2;
    v2a << 0.0, 0.0, 0.0;
    v2b = v1b * 2;

    v1a += Vector3d(4,5,6);
    v1b += Vector3d(4,5,6);
    v2a += Vector3d(10,3,1);
    v2b += Vector3d(10,3,1);
    
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(7.074289188518077, d);

    // parallel same
    v1a << 0.0, 0.0, 0.0;
    v1b << 1.0, 0.0, 0.0;
    v2a << 0.0, 0.0, 0.0;
    v2b << 1.0, 0.0, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(0.0, d);

    // parallel not same
    v1a << 1.0, 0.0, 0.0;
    v1b << 2.0, 0.0, 0.0;
    v2a << -2.0, 0.0, 0.0;
    v2b << -3.0, 0.0, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(3.0, d);

    // not segment
    v1a << 1.0, 1.0, 0.0;
    v1b << 1.0, 1.0, 0.0;
    v2a << 0.0, 0.0, 0.0;
    v2b << 2.0, 0.0, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(1.0, d);
    d = Calc::distance(v2a, v2b, v1a, v1b);
    EXPECT_DOUBLE_EQ(1.0, d);

    // both zero length
    v1a << 1.0, 1.0, 0.0;
    v1b << 1.0, 1.0, 0.0;
    v2a << 2.0, 0.0, 0.0;
    v2b << 2.0, 0.0, 0.0;
    d = Calc::distance(v1a, v1b, v2a, v2b);
    EXPECT_DOUBLE_EQ(sqrt(2.0), d);

  }

  TEST_F(CalcTest, DistancePointSegment)
  {
    Vertex point, seg0, seg1;

    point << 1.0, 0.0, 0.0;
    seg0 << -1.0, 0.0, 0.0;
    seg1 << -2.0, 0.0, 0.0;
    EXPECT_EQ(2.0, Calc::distance(point, seg0, seg1));

    // zero length
    point << 1.0, 0.0, 0.0;
    seg0 << 2.0, 0.0, 0.0;
    seg1 << 2.0, 0.0, 0.0;
    EXPECT_EQ(1.0, Calc::distance(point, seg0, seg1));
  }

  // is point inside triangular face
  TEST_F(CalcTest, pointInside)
  {
    Vertex p;
    Plane face;
    
    face << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0;
    p << 0.1, 0.1, 0.0;
    EXPECT_TRUE(Calc::inside(face,p));
    p << 0.7, 0.7, 0.0;
    EXPECT_FALSE(Calc::inside(face,p));
    p << 1.0, 0.0, 0.0;
    EXPECT_TRUE(Calc::inside(face,p));

    face << 0,0,0,
      0,10,0,
      0,0,10;
    p << 11,1,1;
    EXPECT_FALSE(Calc::inside(face,p));

    p << 0,1,1;
    EXPECT_TRUE(Calc::inside(face,p));
  }

  // is any point inside triangular face
  TEST_F(CalcTest, pointsInside)
  {
    //Vector3d v1, v2, v3;
    Plane face;
    MatrixXd P = MatrixXd(5,3);
    
    face << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0;
    
    P << 1.1, 0.0, 0.0,
      0.0, 1.1, 0.0,
      0.6, 0.6, 0.0,
      0.7, 10.1, 0.0,
      1.0, -0.1, 0.0;
    EXPECT_FALSE(Calc::pointsInside(face,P));

    P << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.7, 10.1, 0.0,
      1.0, -0.1, 0.0;
    EXPECT_FALSE(Calc::pointsInside(face,P));

    
    P << 1.1, 0.0, 0.0,
      0.0, 0.9, 0.0,
      0.6, 0.6, 0.0,
      0.7, 10.1, 0.0,
      1.0, -0.1, 0.0;
    
    EXPECT_TRUE(Calc::pointsInside(face,P));
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

  TEST_F(CalcTest, Angle2D)
  {
    Point p0, p1;

    // p0 at x=1
    p0 << 1.0, 0.0; p1 << 0.0, 1.0;
    EXPECT_EQ(M_PI/2, Calc::angle(p0, p1));

    p0 << 1.0, 0.0; p1 << 1.0, 0.0;
    EXPECT_EQ(0, Calc::angle(p0, p1));

    p0 << 1.0, 0.0; p1 << 0.0, -1.0;
    EXPECT_EQ(-M_PI/2, Calc::angle(p0, p1));

    p0 << 1.0, 0.0; p1 << 1.0, 1.0;
    EXPECT_EQ(M_PI/4, Calc::angle(p0, p1));

    p0 << 1.0, 0.0; p1 << 1.0, -1.0;
    EXPECT_EQ(-M_PI/4, Calc::angle(p0, p1));

    p0 << 1.0, 0.0; p1 << -1.0, 1.0;
    EXPECT_EQ(M_PI*3/4, Calc::angle(p0, p1));

    p0 << 1.0, 0.0; p1 << -1.0, -1.0;
    EXPECT_EQ(-M_PI*3/4, Calc::angle(p0, p1));

    p0 << 1.0, 0.0; p1 << 0.5, sqrt(3)/2;
    EXPECT_EQ(M_PI/3, Calc::angle(p0, p1));

    p0 << 1.0, 0.0; p1 << 0.5, -sqrt(3)/2;
    EXPECT_EQ(-M_PI/3, Calc::angle(p0, p1));

    // p0 not x=0
    p0 << -1.0, 0.0; p1 << 0.0, 1.0;
    EXPECT_EQ(-M_PI/2, Calc::angle(p0, p1));

    p0 << -1.0, 0.0; p1 << 0.0, -1.0;
    EXPECT_EQ(M_PI/2, Calc::angle(p0, p1));

    p0 << -1.0, 1.0; p1 << -1.0, -1.0;
    EXPECT_EQ(M_PI/2, Calc::angle(p0, p1));

    p0 << -1.0, -1.0; p1 << -1.0, 1.0;
    EXPECT_EQ(-M_PI/2, Calc::angle(p0, p1));
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
    
    std::set <unsigned> skip;
    
    std::set <unsigned> c = Calc::connected(F, skip, 0);
    std::set <unsigned> c_exp = {0,1,2};

    EXPECT_EQ(c, c_exp);

    // index is in skip list
    skip.insert(0);
    c_exp.clear();
    c = Calc::connected(F, skip, 0);
    EXPECT_EQ(c_exp, c);
  }

  TEST_F(CalcTest, ConnectedFaces2)
  {
    MatrixXi F = MatrixXi(5,3);

    F << 8,7,6, 1,2,3, 3,4,5, 4,5,6, 0,1,2;
    
    std::set <unsigned> skip;
    
    std::set <unsigned> c = Calc::connected(F, skip, 0);
    std::set <unsigned> c_exp = {0,1,2,3,4};
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
    std::set <unsigned> c_exp, c_exp2;
    std::set <unsigned> skip;
    
    for (int i=0; i<100; i++)
      {
	F.row(i) << i, i+1, i+2;
	c_exp.insert(i);

	if (i > 25)
	  {
	    c_exp2.insert(i);
	  }
      }

    std::set <unsigned> c = Calc::connected(F, skip, 0);
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
    Path F(3);
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
    Path F(6);
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

  // vary dimension
  TEST_F(CalcTest, BoundingBoxDimension)
  {
    Verticies V(6,3);
    Path P;
    Vertex B1,B2, B1_exp, B2_exp;
    V << 0.0, 10.0, 800.0,
      1.0, 20.0, 600.0,
      5.0, 50.0, 200.0,
      4.0, 40.0, 100.0,
      3.0, 60.0, 300.0,
      2.0, 70.0, 400.0;

    P = Path(3);
    P << 0,1,2;
    B1_exp << 0.0, 10.0, 200.0;
    B2_exp << 5.0, 50.0, 800.0;
    Calc::boundingBox(V, P, B1, B2);
    EXPECT_EQ(B1_exp, B1);
    EXPECT_EQ(B2_exp, B2);

    P = Path(6);
    P << 0,1,2,3,4,5;
    B1_exp << 0.0, 10.0, 100.0;
    B2_exp << 5.0, 70.0, 800.0;
    Calc::boundingBox(V, P, B1, B2);
    EXPECT_EQ(B1_exp, B1);
    EXPECT_EQ(B2_exp, B2);

    Path F(6);// = Faces(2,3);
    P << 0,1,2,3,4,5;
    B1_exp << 0.0, 10.0, 100.0;
    B2_exp << 5.0, 70.0, 800.0;
    Calc::boundingBox(V, F, B1, B2);
    EXPECT_EQ(B1_exp, B1);
    EXPECT_EQ(B2_exp, B2);
    
  }

  // project 3d path in plane to 2d path, widest axes
  // not preserving size
  TEST_F(CalcTest, DropAxisBasic)
  {
    Verticies V(3,3);
    Path P(3);    
    //Faces P(1,3);
    
    V << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.1,
      0.0, 1.0, 0.1;
    P << 0,1,2;

    EXPECT_EQ(Calc::minAxis(V, P), AXIS_Z);

    V << 0.0, 0.0, 0.0,
      2.0, 0.0, 5.0,
      0.0, 1.0, 5.0;
    P << 0,1,2;

    EXPECT_EQ(Calc::minAxis(V, P), AXIS_Y);

    V << 0.0, 0.0, 0.0,
      1.0, 0.0, 5.0,
      0.0, 2.0, 5.0;
    P << 0,1,2;

    EXPECT_EQ(Calc::minAxis(V, P), AXIS_X);
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

    //
    winding = 0;
    line << 0.0, 0.0, 0.0, 1.0;
    segment << -1.0, 1.0, 1.0, 1.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);
    // V << 0.0, 0.0, 0.0,
    //   -0.1, 1.0, 0.0,
    //   0.1, 2.0, 0.0,
    //   0.5, 0.5, 0.0,
    //   0.8, 2.4, 0.0,
    //   0.9, 0.3, 0.0,
    //   0.7, -0.4, 0.0,
    //   -1.0, -1.0, 0.0,
    //   -1.1, -1.2, 0.0,
    //   -2.0, 0.3, 0.0;

    winding = 0;
    line << 0.0, 0.0, -0.1, 1.0;
    segment << 0.9, 0.3,
      0.7, -0.4;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 1);

    segment << -0.1, 1.0,
      0.0, 2.0;
    EXPECT_TRUE(Calc::intersect(line, segment, winding));
    EXPECT_EQ(winding, 2);
  }
  
  TEST_F(CalcTest, WindingBasic)
  {
    Verticies V(3,3);
    Path P(3);

    P << 0,1,2;
    V << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      1.0, 1.0, 0.0;
    EXPECT_EQ(0, Calc::winding(V, P));

    V << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      1.0, -1.0, 0.0;
    EXPECT_EQ(1, Calc::winding(V, P));

    V << 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      -1.0, 1.0, 0.0;
    EXPECT_EQ(0, Calc::winding(V, P));

    V << 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 1.0, 0.0;
    EXPECT_EQ(1, Calc::winding(V, P));

    // xz
    V << 0.0, 0.0, 1.0,
      0.0, 0.0, -1.0,
      1.0, 0.0, -1.0;
    EXPECT_EQ(0, Calc::winding(V, P));

    // yz
    V << 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 1.0;
    EXPECT_EQ(1, Calc::winding(V, P));
  }
  
  TEST_F(CalcTest, Winding)
  {
    Verticies V(10,3);
    Path P(10);

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
  }

  TEST_F(CalcTest, Inside3D)
  {
    CubeNode * cube = new CubeNode(10,10,10);

    EXPECT_TRUE(Calc::inside(cube->g, Vertex(1,2,3)));
    EXPECT_TRUE(Calc::inside(cube->g, Vertex(1,1,1)));
    EXPECT_TRUE(Calc::inside(cube->g, Vertex(0,0,0)));
    EXPECT_TRUE(Calc::inside(cube->g, Vertex(10,10,10)));
    EXPECT_TRUE(Calc::inside(cube->g, Vertex(0,0,10)));
    EXPECT_TRUE(Calc::inside(cube->g, Vertex(1,1,8)));

    EXPECT_TRUE(Calc::inside(cube->g, Vertex(1,2,8)));
    EXPECT_TRUE(Calc::inside(cube->g, Vertex(1,1,8)));
    EXPECT_TRUE(Calc::inside(cube->g, Vertex(1,2,9)));
    EXPECT_TRUE(Calc::inside(cube->g, Vertex(1,1,9)));

    EXPECT_FALSE(Calc::inside(cube->g, Vertex(11,1,1)));
    EXPECT_FALSE(Calc::inside(cube->g, Vertex(-0.0000001, 0, 0)));
    EXPECT_FALSE(Calc::inside(cube->g, Vertex(0,0,100000)));
    EXPECT_FALSE(Calc::inside(cube->g, Vertex(-0.0000001, 10, 10)));
  }

  TEST_F(CalcTest, Inside3Dmore)
  {
    CubeNode * cube = new CubeNode(10,10,10);

    EXPECT_FALSE(Calc::inside(cube->g, Vertex(11,1,8)));
    //EXPECT_FALSE(Calc::inside(cube->g, Vertex(11,1,9)));
    
  }

  // Test if a line passes through faces, or just touches a point
  TEST_F(CalcTest, IntersectPoint)
  {
    Verticies V(5,3);
    Faces F(4,3);
    unsigned pointIndex;
    Vertex segment;
    
    V << 0,0,1, 1,1,0, -1,1,0, -1,-1,0, 1,-1,0;
    F << 0,1,2, 0,2,3, 0,3,4, 0,4,1;
    Geometry G(V,F);
    pointIndex = 0;

    segment << 1,0,1;
    EXPECT_FALSE(Calc::intersect(G, pointIndex, segment));

    segment << 0,0,2;
    EXPECT_TRUE(Calc::intersect(G, pointIndex, segment));    
  }

  // Test if line passes through face or just touches segment. 
  TEST_F(CalcTest, IntersectSegment)
  {
    Verticies verticies(4,3);
    Faces faces(2,3);
    Line line;

    verticies << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    faces << 0,1,2, 1,2,3;
    
    Geometry geometry(verticies, faces);

    line << 0,0,0.5, 0.5,0.5,0;
    EXPECT_TRUE(Calc::intersect(geometry, 0, 1, line));

    line << 1,1,0.5, 0.5,0.5,0;
    EXPECT_FALSE(Calc::intersect(geometry, 0, 1, line));
  }

  TEST_F(CalcTest, AbovePlane)
  {
    Vertex point;
    Plane plane;
    
    plane << 0,0,0, 1,0,0, 0,1,0;

    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(0,0,1)));
    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(0,0,0.000000001)));
    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(-10,-10,0.1)));
    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(-10,10,1)));

    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0,0,-1)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0,0,-0.000000001)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(-10,-10,-0.1)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(-10,10,-1)));

    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0,0,0)));
    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(10000,-10,4000)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(10000,-4000,-10)));

    plane << 0,0,1, 1,0,1, 0,-1,1;

    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0,0,2)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0,0,1.000000001)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(-10,-10,1.1)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(-10,10,2)));

    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(0,0,0)));
    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(0,0,1-0.000000001)));
    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(-10,-10,1-0.1)));
    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(-10,10,0)));

    plane << 1,0,0, 0,1,0, 0,0,1;

    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(0.7, 0.7, 0.7)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0.3, 0.3, 0.3)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0.0, 0.0, 0.9999)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(1,0,0)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0,1,0)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(0,0,1)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(1/3.0, 1/3.0, 1/3.0)));
    EXPECT_TRUE(Calc::abovePlane(plane, Vertex(1/3.0, 1/3.0, 1.0000000001/3.0)));
    EXPECT_FALSE(Calc::abovePlane(plane, Vertex(1.0/2, 1.0/2, 0)));
  }

  TEST_F(CalcTest, HasPoint)
  {
    Verticies verticies(3,3);

    verticies << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 2.1;

    EXPECT_TRUE(Calc::hasPoint(verticies, Vertex(0.0, 0.0, 0.0)));
    EXPECT_TRUE(Calc::hasPoint(verticies, Vertex(1.0, 0.0, 0.0)));
    EXPECT_TRUE(Calc::hasPoint(verticies, Vertex(0.0, 1.0, 2.1)));

    EXPECT_FALSE(Calc::hasPoint(verticies, Vertex(0.0, 0.0, 0.1)));
    EXPECT_FALSE(Calc::hasPoint(verticies, Vertex(0.0, 1.0, 0.0)));
    EXPECT_FALSE(Calc::hasPoint(verticies, Vertex(0.0, 0.0, 1.0)));
  }

  TEST_F(CalcTest, PointIndex)
  {
    Verticies verticies(3,3);

    verticies << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 2.1;

    EXPECT_EQ(0, Calc::pointIndex(verticies, Vertex(0.0, 0.0, 0.0)));
    EXPECT_EQ(1, Calc::pointIndex(verticies, Vertex(1.0, 0.0, 0.0)));
    EXPECT_EQ(2, Calc::pointIndex(verticies, Vertex(0.0, 1.0, 2.1)));

    EXPECT_EQ(-1, Calc::pointIndex(verticies, Vertex(0.0, 0.0, 0.1)));
  }

  TEST_F(CalcTest, AtEdge)
  {
    Plane face(3,3);

    face << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0;

    EXPECT_TRUE(Calc::atEdge(face, Vertex(0.0, 0.0, 0.0)));
    EXPECT_TRUE(Calc::atEdge(face, Vertex(0.5, 0.0, 0.0)));
    EXPECT_TRUE(Calc::atEdge(face, Vertex(0.0, 0.5, 0.0)));
    EXPECT_TRUE(Calc::atEdge(face, Vertex(0.5, 0.5, 0.0)));
    EXPECT_FALSE(Calc::atEdge(face, Vertex(1.5, 0.0, 0.0)));
    EXPECT_FALSE(Calc::atEdge(face, Vertex(-0.5, 0.0, 0.0)));
    EXPECT_FALSE(Calc::atEdge(face, Vertex(0.5, 0.0, 0.00001)));
  }

  TEST_F(CalcTest, GetSegment)
  {
    
  }
}
