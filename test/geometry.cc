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
    Eigen::MatrixXd V3(3,3);
    Eigen::MatrixXi F3(1,3);

    Eigen::Vector3d v;
    Eigen::Vector3d ve;
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    
    V1 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0;
    F1 << 0,1,2;
    V2 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0;
    F2 << 0,1,2;
    V3 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, -0.5, 0.0;
    F3 << 0,1,2;
    
    Geometry *g = new Geometry(V1,F1);


    // face - vector intersect
    v1 << 0.1, 0.2, -1.0;
    v2 << 0.1, 0.2, 1.0;
    ve << 0.1, 0.2, 0.0;
    v << 0.0, 0.0, 0.0;
    bool r  = g->intersect(&V1, &v1, &v2, &v);
    EXPECT_TRUE(g->equal(v, ve));
    
    INTERSECT_TYPE t = g->intersect(&V1, &V2, &v);
    EXPECT_EQ(t, INTERSECT_EQUAL);

    //EXPECT_EQ(g->intersect(&V1, &V3, &v), INTERSECT_SIDE);
    
  }
}
