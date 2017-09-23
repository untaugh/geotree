#include "gtest/gtest.h"

#include "Geometry.h"
#include <Eigen/Core>

namespace {

  class GeometryTest : public ::testing::Test {
  protected:
  };

  TEST_F(GeometryTest, Object)
  {
    Eigen::MatrixXd V(3,3);
    Eigen::MatrixXi F(1,3);

    V << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0;
    F << 0,1,2;
    
    Geometry *g = new Geometry(V,F);
    
    EXPECT_EQ(3, g->V.rows());
    EXPECT_EQ(1, g->F.rows());
  }
  
  TEST_F(GeometryTest, add)
  {
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
    
    g1->add(*g2);
    
    
    EXPECT_EQ(g1Vrows + g2->V.rows(),  g1->V.rows());
    EXPECT_EQ(g1Frows + g2->F.rows(),  g1->F.rows());
  }

  TEST_F(GeometryTest, AddEmpty)
  {
    Verticies V(3,3);
    Faces F(1,3);
    Geometry G1;
    Geometry G2(V,F);

    G1.add(G2);

    EXPECT_EQ(3, G1.V.rows());
    EXPECT_EQ(1, G1.F.rows());
  }

  TEST_F(GeometryTest, addFaces)
  {
    Verticies V(4,3);
    Faces F(1,3);
    Faces F_new(2,3);
    Faces F_exp(3,3);
    Faces F_empty(0,3);

    V << 0.0, 0.0, 0.0,  0.5, 0.0, 0.0,  0.0, 0.5, 0.0,  1,2,3;
      F << 0,1,2;
      F_new << 0,1,3, 1,2,3;
      F_exp << 0,1,2, 0,1,3, 1,2,3;
      
      Geometry *g = new Geometry(V,F);
    
      EXPECT_EQ(4, g->V.rows());
      EXPECT_EQ(1, g->F.rows());

      g->addFaces(F_empty);

      EXPECT_EQ(4, g->V.rows());
      EXPECT_EQ(1, g->F.rows());
      
      g->addFaces(F_new);

      EXPECT_EQ(4, g->V.rows());
      EXPECT_EQ(3, g->F.rows());
      EXPECT_EQ(F_exp, g->F);
  }

  TEST_F(GeometryTest, DISABLED_removeFace)
  {
    Verticies V(4,3);
    Faces F(3,3);
    Faces F_exp1(2,3);
    Faces F_exp2(1,3);
    Faces F_exp3(0,3);

    V << 0.0, 0.0, 0.0,  0.5, 0.0, 0.0,  0.0, 0.5, 0.0,  1,2,3;
    F << 0,1,2, 0,1,3, 1,2,3;
    F_exp1 << 0,1,2, 1,2,3;
    F_exp2 << 1,2,3;
      
    Geometry *g = new Geometry(V,F);
	
    g->removeFace(1);
    EXPECT_EQ(F_exp1, g->F);

    g->removeFace(0);
    EXPECT_EQ(F_exp2, g->F);

    g->removeFace(0);
    EXPECT_EQ(F_exp3, g->F);
    
  }
  
}
