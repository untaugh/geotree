#include "gtest/gtest.h"

#include "Node.h"
#include <Eigen/Core>

namespace {

  class NodeTest : public ::testing::Test {
  protected:

    ::testing::AssertionResult containsVector(Eigen::MatrixXd m,
					      double x,
					      double y,
					      double z);
  };

  ::testing::AssertionResult NodeTest::containsVector(Eigen::MatrixXd m,
						      double x,
						      double y,
						      double z)
  {
    bool contains = false;

    Eigen::MatrixXd v = Eigen::MatrixXd(1,3);
    v << x,y,z;
    
    for (int i=0; i<m.rows(); i++)
      {
	if (m.row(i) == v)
	  {
	    return ::testing::AssertionSuccess();
	  }
      }
    
    return ::testing::AssertionFailure() << "does not contain";
  }

  testing::AssertionResult AssertionSuccess();
  testing::AssertionResult AssertionFailure();


  
  // create an object
  TEST_F(NodeTest, Object) {
    Eigen::MatrixXd V(3,3);
    Eigen::MatrixXi F(1,3);

    V << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0;
    F << 0,1,2;
    
    Geometry *g = new Geometry(V,F);

    GeometryNode *n = new GeometryNode(g);

    EXPECT_EQ(3, n->g->V.rows());
  }

  // union between separate 2d geometries
  TEST_F(NodeTest, Union) {
    Eigen::MatrixXd V1(3,3);
    Eigen::MatrixXi F1(1,3);

    V1 << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0;
    F1 << 0,1,2;

    Eigen::MatrixXd V2(3,3);
    Eigen::MatrixXi F2(1,3);

    V2 << -0.5, -0.5, -0.1, -0.5, 0.0, 0.0, -0.1, -0.5, 0.0;
    F2 << 2,1,0;

    Geometry *g1 = new Geometry(V1,F1);
    Geometry *g2 = new Geometry(V2,F2);

    GeometryNode *n1 = new GeometryNode(g1);
    GeometryNode *n2 = new GeometryNode(g2);
    UnionNode *n3 = new UnionNode();
    
    n3->add(n1);
    
    n3->add(n2);

    n3->build();
    
    EXPECT_EQ(6, n3->g->V.rows());
    EXPECT_EQ(2, n3->g->F.rows());

    Eigen::MatrixXd v(1,3);
    Eigen::MatrixXi f(1,3);

    v << 0.5, 0.0, 0.0;
    EXPECT_EQ(v, n3->g->V.row(1));

    v << -0.1, -0.5, 0.0;
    EXPECT_EQ(v, n3->g->V.row(5));
    
    f << 2,1,0;
    EXPECT_EQ(f, n3->g->F.row(1));
  }

  // tall union tree
  TEST_F(NodeTest, UnionTree) {
    Eigen::MatrixXd V(3,3);
    Eigen::MatrixXi F(1,3);

    V << 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0;
    F << 0,1,2;
    
    Geometry *g = new Geometry(V, F);
    GeometryNode *n = new GeometryNode(g);

    UnionNode *n1 = new UnionNode();
    n1->add(n);
    UnionNode *n2 = new UnionNode();
    n2->add(n1);
    UnionNode *n3 = new UnionNode();
    n3->add(n2);
    UnionNode *n4 = new UnionNode();
    n4->add(n3);
    UnionNode *n5 = new UnionNode();
    n5->add(n4);

    n5->build();

    EXPECT_EQ(3, n5->g->V.rows());
    EXPECT_EQ(1, n5->g->F.rows());

    Eigen::MatrixXd v(1,3);
    Eigen::MatrixXi f(1,3);

    v << 0.5, 0.0, 0.0;
    EXPECT_EQ(v, n5->g->V.row(1));
    
    f << 0,1,2;
    EXPECT_EQ(f, n5->g->F.row(0));
  }

  // cube node
  TEST_F(NodeTest, CubeNode) {

    CubeNode * n = new CubeNode(10,10,10);

    EXPECT_EQ(8, n->g->V.rows());
    EXPECT_EQ(12, n->g->F.rows());
  }

  // translate node
  TEST_F(NodeTest, TranslateNode) {

    CubeNode * c = new CubeNode(10,10,10);
    TranslateNode * t = new TranslateNode(10,2,-5);

    t->add(c);

    t->build();
    
    EXPECT_EQ(8, t->g->V.rows());
    EXPECT_EQ(12, t->g->F.rows());

    double xmin = 1000.0, xmax = 0.0;
    double ymin = 1000.0, ymax = 0.0;
    double zmin = 1000.0, zmax = 0.0;
    
    for (int i=0; i < t->g->V.rows(); i++)
      {
	Eigen::Vector3d v = t->g->V.row(i);
	xmin = std::min(xmin,v(0));
	xmax = std::max(xmax,v(0));
	ymin = std::min(ymin,v(1));
	ymax = std::max(ymax,v(1));
	zmin = std::min(zmin,v(2));
	zmax = std::max(zmax,v(2));
      }

    EXPECT_EQ(10.0, xmin);
    EXPECT_EQ(20.0, xmax);
    EXPECT_EQ(2.0, ymin);
    EXPECT_EQ(12.0, ymax);
    EXPECT_EQ(-5.0, zmin);
    EXPECT_EQ(5.0, zmax);    
  }

    // union between intersecting 3d cubes
  TEST_F(NodeTest, Union3D) {

    CubeNode * c1 = new CubeNode(10,10,10);
    CubeNode * c2 = new CubeNode(10,10,10);
    TranslateNode * t = new TranslateNode(1,2,3);
    UnionNode * u = new UnionNode();
    
    t->add(c2);
    t->build();

    GeometryNode * g = new GeometryNode(t->g);
        
    u->add(c1);
    u->add(g);
    u->build();

    EXPECT_EQ(20, u->g->V.rows());
    EXPECT_EQ(33, u->g->F.rows());

    EXPECT_TRUE(containsVector(u->g->V, 0.0, 0.0, 0.0));
    
  }
}
