#include "gtest/gtest.h"

#include "Geometry.h"
#include <Eigen/Core>

namespace {

  class GeometryTest : public ::testing::Test {
  protected:
  
  };

  TEST_F(GeometryTest, Object) {
    Eigen::MatrixXd V(3,3);
    Eigen::MatrixXi F(1,3);

    V << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0;
    F << 0,1,2;
    
    Geometry *g = new Geometry(V,F);
    
    EXPECT_EQ(3, g->V.rows());
    EXPECT_EQ(1, g->F.rows());
  }

  TEST_F(GeometryTest, VectEqual) {
      Eigen::MatrixXd V1(3,3);
      Eigen::MatrixXi F1(1,3);
      Eigen::Vector3d v1;
      Eigen::Vector3d v2;

      Geometry *g = new Geometry(V1, F1);

      v1 << 0.3, 0.2, 0.1;
      v2 << 0.3, 0.2, 0.11;
      EXPECT_FALSE(g->equal(v1, v2));
      
      v1 << 0.3, 0.2, 0.1;
      v2 << 0.3, 0.2, 0.1;
      EXPECT_TRUE(g->equal(v1, v2));

      v2 << 0.1, 0.3, 0.2;
      EXPECT_TRUE(g->equal(v1, v2));

      v2 << 0.2, 0.1, 0.3;
      EXPECT_TRUE(g->equal(v1, v2));
    }

  TEST_F(GeometryTest, LineEqual) {
    Eigen::MatrixXd V1(3,3);
    Eigen::MatrixXd V2(3,3);
    Eigen::MatrixXi F1(1,3);

    V1 << 0.9, 0.0, 0.0, 0.5, 0.1, 0.0, 0.55, 0.55, -0.7;
    V2 << 0.0, 0.5, 0.1, 0.9, 0.0, 0.0, -0.7, 0.55, 0.55;
    
    Geometry *g = new Geometry(V1, F1);
    
    EXPECT_FALSE(g->equal(V1.row(0), V2.row(0)));
    EXPECT_TRUE(g->equal(V1.row(1), V2.row(0)));
    EXPECT_TRUE(g->equal(V1.row(2), V2.row(2)));
    EXPECT_TRUE(g->equal(V1.row(0), V2.row(1)));
  }

  TEST_F(GeometryTest, FaceEqual) {
    Eigen::Matrix<double,3,3> V1(3,3);
    Eigen::Matrix<double,3,3> V2(3,3);
    Eigen::Matrix<double,3,3> V3(3,3);
    Eigen::MatrixXi F1(1,3);

    V1 << 0.9, 0.0, 0.0, 0.5, 0.1, 0.0, 0.55, 0.55, -0.7;
    V2 << 0.0, 0.5, 0.1, 0.9, 0.0, 0.0, -0.7, 0.55, 0.55;
    V3 << 0.0, 0.5, 0.1, 0.9, 0.0, 0.0, -0.7, 0.55, 0.54;
    
    Geometry *g = new Geometry(V1, F1);
    
    EXPECT_FALSE(g->fequal(V1, V3));
    EXPECT_TRUE(g->fequal(V1, V2));
  }
  
  TEST_F(GeometryTest, Intersect) {

    Eigen::MatrixXd V1(3,3);
    Eigen::MatrixXi F1(1,3);
    Eigen::MatrixXd V2(3,3);
    Eigen::MatrixXi F2(1,3);

    Eigen::Vector3d v;
    Eigen::Vector3d ve;
    Eigen::Vector3d s1;
    Eigen::Vector3d s2;
    
    V1 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0;
    F1 << 0,1,2;
    V2 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0, 0.5, 0.5;
    F2 << 0,1,2;
    
    Geometry *g = new Geometry(V1,F1);

    // face - segment intersect
    s1 << 0.1, 0.1, -1.0;
    s2 << 0.1, 0.1, 1.0;
    ve << 0.1, 0.1, 0.0;
    EXPECT_TRUE(g->intersect(&V1, &s1, &s2, &v));
    EXPECT_TRUE(g->equal(v, ve));

    V1 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0, 0.5, 0.5;	
    s1 << 0.1, 0.1, 0.0;
    s2 << 0.1, 0.1, 1.0;
    ve << 0.1, 0.10000000000000001, 0.20000000298023224;
    
    EXPECT_TRUE(g->intersect(&V1, &s1, &s2, &v));
    EXPECT_DOUBLE_EQ(v(0), ve(0));
    EXPECT_DOUBLE_EQ(v(1), ve(1));
    EXPECT_DOUBLE_EQ(v(2), ve(2));

    V1 << 0.0, 0.0, 10.0, 10.0, 10.0, 10.0, 0.0,  10.0, 10.0;
    s1 << 1.0, 2.0, 1.0;
    s2 << 1.0, 2.0, 13.0;
    ve << 1.0, 2.0, 10;
    
    EXPECT_TRUE(g->intersect(&V1, &s1, &s2, &v));
    EXPECT_DOUBLE_EQ(v(0), ve(0));
    EXPECT_DOUBLE_EQ(v(1), ve(1));
    EXPECT_DOUBLE_EQ(v(2), ve(2));
  }

  TEST_F(GeometryTest, DISABLED_IntersectFace) {
    Eigen::MatrixXd V1(3,3);
    Eigen::MatrixXi F1(1,3);
    Eigen::MatrixXd V2(3,3);
    Eigen::MatrixXi F2(1,3);

    Eigen::Vector3d v;
	  
    V1 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0, 0.5, 0.5;
    F1 << 0,1,2;
    V2 << 0.1, 0.1, 0.0, 0.1, 0.1, 5.0, 5.2, 5.2, 0.0;
    F2 << 0,1,2;

    Geometry *g = new Geometry(V1,F1);
	  
    unsigned int count = g->intersect(&V1, &V2, &v);
    EXPECT_EQ(count,1);
  }
  
  TEST_F(GeometryTest, DISABLED_Divide) {
    Eigen::MatrixXd V1(3,3);
    Eigen::MatrixXi F1(1,3);
    Eigen::MatrixXd V2(3,3);
    Eigen::MatrixXi F2(1,3);

    Eigen::Vector3d v;
	  
    V1 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0, 0.5, 0.5;
    F1 << 0,1,2;
    V2 << 0.1, 0.1, 0.0, 0.1, 0.1, 5.0, 5.2, 5.2, 0.0;
    F2 << 0,1,2;

    Geometry *g = new Geometry(V1,F1);

    v << V1.row(0);
    
    //unsigned int count = g->divide(&V1, &v, &);
    //EXPECT_EQ(count,1);
  }
  
  TEST_F(GeometryTest, add) {
    Eigen::MatrixXd V1(3,3);
    Eigen::MatrixXi F1(1,3);
    Eigen::MatrixXd V2(3,3);
    Eigen::MatrixXi F2(1,3);

    Eigen::Vector3d v;
	  
    V1 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.0, 0.5, 0.5;
    F1 << 0,1,2;
    V2 << 0.1, 0.1, 0.0, 0.1, 0.1, 5.0, 5.2, 5.2, 0.0;
    F2 << 0,1,2;

    Geometry *g1 = new Geometry(V1,F1);
    Geometry *g2 = new Geometry(V2,F2);
    
    int g1Vrows = g1->V.rows();
    int g1Frows = g1->F.rows();
    
    g1->add(g2);
    
    
    EXPECT_EQ(g1Vrows + g2->V.rows(),  g1->V.rows());
    EXPECT_EQ(g1Frows + g2->F.rows(),  g1->F.rows());
  }

  TEST_F(GeometryTest, DISABLED_divideFace)
  {
    Eigen::MatrixXd V(3,3);
    Eigen::MatrixXi F(1,3);
    Eigen::MatrixXd P(2,3);
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    
    V << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    F << 0,1,2;

    P << 0.0, 0.5, 0.0, 0.5, 0.0, 0.0;

    Geometry *g = new Geometry(V,F);

    Calc::divide(&V, &P, &V2, &F2);
    
    EXPECT_EQ(V2.rows(), 5);
    EXPECT_EQ(F2.rows(), 3);
  }
  
}
