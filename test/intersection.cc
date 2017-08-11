#include "gtest/gtest.h"

#include "Intersect.h"
#include <Node.h>
#include <Eigen/Core>
#include <vector>
#include <set>

using namespace Eigen;

namespace {
  class IntersectionTest : public ::testing::Test {

  protected:
    IntersectionTest()
    {
    // testverticies << 0.0, 0.0, 0.0,
    // 		     0.0, 1.0, 0.0,
    // 		     1.0, 0.0, 0.0;

    // testpath1 << 0,1,2;
    // testpath2 << 0,2,1;
    
    }
    MatrixXd testverticies = MatrixXd(3,3);
    // VectorXi testpath1(3);
    // VectorXi testpath2(3);
    
  };

  // Intersections between four triangular faces
  TEST_F(IntersectionTest, basic)
  {
    Intersections * I = new Intersections();
    std::vector<std::vector<int>> paths;
    std::vector<int> path;

    I->add(0, Vector2i(3,4), Vector3d(1,1,0));
    I->add(3, Vector2i(0,1), Vector3d(1,0,0));
    I->add(4, Vector2i(0,1), Vector3d(2,0,0));
    I->add(1, Vector2i(3,4), Vector3d(3,0,0));

    // test numPoints
    EXPECT_EQ(I->I.size(), 4);
    
    // test getPaths
    paths = I->getPaths(0);
    path = {1,0,2};
    EXPECT_EQ(path, paths[0]);
    path = {1,2,0};
    EXPECT_NE(path, paths[0]);

    paths = I->getPaths(3);
    path = {0,1,3};
    EXPECT_EQ(paths[0], path);

    // test getPoints
    Eigen::MatrixXd points;
    Eigen::MatrixXd points_exp = Eigen::MatrixXd(4,3);
    points_exp << 1,1,0, 1,0,0, 2,0,0, 3,0,0;
    I->getPoints(points);    
    EXPECT_EQ(points, points_exp);
  }

  // Intersections between a few triangular faces
  TEST_F(IntersectionTest, DISABLED_test1)
  {
    Intersections * I = new Intersections();
    std::vector<std::vector<int>> paths;
    std::vector<int> path;

    I->add(1, Vector2i(3,4), Vector3d(5,0,0));
    I->add(1, Vector2i(4,5), Vector3d(5,0,0));
    I->add(2, Vector2i(5,6), Vector3d(4,0,0));
    I->add(3, Vector2i(0,1), Vector3d(1,0,0));
    I->add(21, Vector2i(0,20), Vector3d(1,0,0));
    I->add(5, Vector2i(1,2), Vector3d(9,0,0));
    I->add(5, Vector2i(9,8), Vector3d(9,0,0));
    I->add(5, Vector2i(8,11), Vector3d(7,0,0));

    // test numPoints
    EXPECT_EQ(I->I.size(), 7);

    // test getPaths
    paths = I->getPaths(1);
    path = {3,0,1,4};
    EXPECT_EQ(paths[0], path);
    path = {3,0,4,1};
    EXPECT_NE(paths[0], path);

    paths = I->getPaths(3);
    path = {3,2,4};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(2);
    path = {0,3,2};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(8);
    path = {5,6};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(11);
    EXPECT_EQ(paths.size(), 0);

    paths = I->getPaths(120);
    EXPECT_EQ(paths.size(), 0);
    
    // test getPoints
    Eigen::MatrixXd points;
    Eigen::MatrixXd points_exp = Eigen::MatrixXd(7,3);
    points_exp << 5,0,0, 4,0,0, 3,0,0, 2,0,0, 1,0,0, 9,0,0, 7,0,0;
    I->getPoints(points);    
    EXPECT_EQ(points, points_exp);
  }

  // get faces of outside/inside/intersection
  TEST_F(IntersectionTest, faceInfo1)
  {
    Intersections * I = new Intersections();

    // create two cube geometries
    CubeNode * c1 = new CubeNode(10,10,10);
    CubeNode * c2 = new CubeNode(10,1,1);
    TranslateNode * t = new TranslateNode(1,1,8);

    t->add(c2);
    t->build();

    I->add(*c1->g, *t->g);

    std::set <unsigned> Fi, Fo, Ft, F_tot, F_exp, div1, div2;

    for (int i=0; i<12; i++)
      {
	F_exp.insert(i);
      }

    bool ret = I->getIntersectingFaces(Ft);

    EXPECT_EQ(Ft.size(), 9);

    for (unsigned f : Ft)
      {
	I->divide(f, div1, div2);
      }

    

    
    // I->faceInfo(0, Fi, Fo, Ft);
    // F_tot.insert(Fi.begin(), Fi.end());
    // F_tot.insert(Fo.begin(), Fo.end());
    // F_tot.insert(Ft.begin(), Ft.end());
    // EXPECT_EQ(Fi.size(), 0);
    // EXPECT_EQ(Fo.size(), 11);
    // EXPECT_EQ(Ft.size(), 1);
    //EXPECT_EQ(F_tot, F_exp);

    // divide faces
    // Eigen::MatrixXi F1,F2;
    
    // for(int f : Ft)
    //   {
    // 	I->divide(0, f, F1, F2);
    //   }

    // Fi.clear(); Fo.clear(); Ft.clear();
    // I->faceInfo(1, Fi, Fo, Ft);
    // F_tot.insert(Fi.begin(), Fi.end());
    // F_tot.insert(Fo.begin(), Fo.end());
    // F_tot.insert(Ft.begin(), Ft.end());
    // EXPECT_EQ(Fi.size(), 2);
    // EXPECT_EQ(Fo.size(), 2);
    // EXPECT_EQ(Ft.size(), 8);
    //EXPECT_EQ(F_tot, F_exp);

    // divide faces
    // for(int f : Ft)
    //   {
    // 	I->divide(1, f, F1, F2);
    //   }
  }

  // get faces of outside/inside/intersection
  TEST_F(IntersectionTest, DISABLED_faceInfo2)
  {
    Intersections * I = new Intersections();

    // create two cube geometries
    CubeNode * c1 = new CubeNode(10,10,10);
    CubeNode * c2 = new CubeNode(10,10,10);
    TranslateNode * t = new TranslateNode(1,2,3);

    t->add(c2);
    t->build();

    I->add(*c1->g, *t->g);

    std::set <int> Fi, Fo, Ft, F_tot, F_exp;

    for (int i=0; i<12; i++)
      {
	F_exp.insert(i);
      }
    
    // I->faceInfo(0, Fi, Fo, Ft);
    // F_tot.insert(Fi.begin(), Fi.end());
    // F_tot.insert(Fo.begin(), Fo.end());
    // F_tot.insert(Ft.begin(), Ft.end());
    // EXPECT_EQ(0, Fi.size());
    // EXPECT_EQ(6, Fo.size());
    // EXPECT_EQ(6, Ft.size());
    // EXPECT_EQ(F_tot, F_exp);

    // Fi.clear(); Fo.clear(); Ft.clear();
    // I->faceInfo(1, Fi, Fo, Ft);
    // F_tot.insert(Fi.begin(), Fi.end());
    // F_tot.insert(Fo.begin(), Fo.end());
    // F_tot.insert(Ft.begin(), Ft.end());
    // EXPECT_EQ(0, Fi.size());
    // EXPECT_EQ(6, Fo.size());
    // EXPECT_EQ(6, Ft.size());
    // EXPECT_EQ(F_tot, F_exp);
  }

  TEST_F(IntersectionTest, addGeometryBasic)
  {
    Intersections * I = new Intersections();

    // create two cube geometries
    CubeNode * c1 = new CubeNode(10,10,10);
    CubeNode * c2 = new CubeNode(10,10,10);
    TranslateNode * t = new TranslateNode(1,2,3);

    t->add(c2);
    t->build();

    I->add(*c1->g, *t->g);

    // test numPoints
    EXPECT_EQ(I->I.size(), 14);
    
    int gxSize = I->g1.V.rows() + I->g1.V.rows() + I->I.size();
    EXPECT_EQ(gxSize, I->gx.V.rows());

    Vector3d V_exp, V_res;

    V_exp << 0.0, 0.0, 0.0;
    V_res = I->gx.V.row(0);
    EXPECT_EQ(V_exp, V_res);

    V_exp << 1.0, 2.0, 3.0;
    V_res = I->gx.V.row(8);
    EXPECT_EQ(V_exp, V_res);

    V_exp = I->I.front().point;
    V_res = I->gx.V.row(8+8);
    EXPECT_EQ(V_exp, V_res);

    V_exp = I->I.back().point;
    V_res = I->gx.V.row(8+8+13);
    EXPECT_EQ(V_exp, V_res);    
  }

  TEST_F(IntersectionTest, DISABLED_invalidGeometries)
  {
    Intersections * I = new Intersections();

    Eigen::MatrixXd V1 = Eigen::MatrixXd(5,3);
    Eigen::MatrixXd V2 = Eigen::MatrixXd(3,3);
    Eigen::MatrixXi F1 = Eigen::MatrixXi(3,3);
    Eigen::MatrixXi F2 = Eigen::MatrixXi(1,3);
    Eigen::MatrixXi Fa;
    Eigen::MatrixXi Fb;

    V1 << 0,0,0, 1,0,0, 0,1,0, 1,1,0, 1,-1,0;
    F1 << 0,1,2, 1,2,3, 0,1,4;
    V2 << 0.5,-1,-1, 0.5,2,-1, 0.5,0.5,5;
    F2 << 0,1,2;
    
    Geometry *g1 = new Geometry(V1, F1);
    Geometry *g2 = new Geometry(V2, F2);

    I->add(*g1, *g2);

    //EXPECT_EQ(I->numPoints(0), 3);
    //EXPECT_EQ(I->numPoints(1), 3);


  }

  // find normal, which side of polygon is inside
  TEST_F(IntersectionTest, FindNormal)
  {
    
  }

}
