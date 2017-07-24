#include "gtest/gtest.h"

#include "Intersect.h"
#include <Node.h>
#include <Eigen/Core>
#include <vector>
#include <set>

using namespace Eigen;

namespace {
  class IntersectionTest : public ::testing::Test {
  };

  // Intersections between four triangular faces
  TEST_F(IntersectionTest, basic)
  {
    Intersections * I = new Intersections();
    std::vector<std::set<unsigned>> paths;
    std::set<unsigned> path;

    I->add(0, 0, Vector2i(3,4), Vector3d(1,1,0));
    I->add(1, 3, Vector2i(0,1), Vector3d(1,0,0));
    I->add(1, 4, Vector2i(0,1), Vector3d(2,0,0));
    I->add(0, 1, Vector2i(3,4), Vector3d(3,0,0));

    // test numPoints
    EXPECT_EQ(I->numPoints(0), 2);
    EXPECT_EQ(I->numPoints(1), 2);

    // test getPaths
    paths = I->getPaths(0,0);
    path = {1,0,2};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(1,3);
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
  TEST_F(IntersectionTest, test1)
  {
    Intersections * I = new Intersections();
    std::vector<std::set<unsigned>> paths;
    std::set<unsigned> path;

    I->add(1, 1, Vector2i(1,2), Vector3d(5,0,0));
    I->add(0, 1, Vector2i(0,1), Vector3d(4,0,0));
    I->add(1, 2, Vector2i(2,3), Vector3d(3,0,0));
    I->add(0, 2, Vector2i(1,2), Vector3d(2,0,0));
    I->add(0, 3, Vector2i(2,3), Vector3d(1,0,0));

    // test numPoints
    EXPECT_EQ(I->numPoints(0), 3);
    EXPECT_EQ(I->numPoints(1), 2);

    // test getPaths
    paths = I->getPaths(1,1);
    path = {1,0,3};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(1,2);
    path = {3,2,4};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(0,2);
    path = {0,3,2};
    EXPECT_EQ(paths[0], path);

    // test getPoints
    Eigen::MatrixXd points;
    Eigen::MatrixXd points_exp = Eigen::MatrixXd(5,3);
    points_exp << 5,0,0, 4,0,0, 3,0,0, 2,0,0, 1,0,0;
    I->getPoints(points);    
    EXPECT_EQ(points, points_exp);
  }

  // get faces of outside/inside/intersection
  TEST_F(IntersectionTest, faceInfo1)
  {
    Intersections * I = new Intersections();

    // create two cube geometries
    CubeNode * c1 = new CubeNode(10,10,10);
    CubeNode * c2 = new CubeNode(10,5,5);
    TranslateNode * t = new TranslateNode(1,1,1);

    t->add(c2);
    t->build();

    I->add(*c1->g, *t->g);

    std::set <int> Fi, Fo, Ft, F_tot, F_exp;

    for (int i=0; i<12; i++)
      {
	F_exp.insert(i);
      }
    
    I->faceInfo(0, Fi, Fo, Ft);
    F_tot.insert(Fi.begin(), Fi.end());
    F_tot.insert(Fo.begin(), Fo.end());
    F_tot.insert(Ft.begin(), Ft.end());
    EXPECT_EQ(Fi.size(), 0);
    EXPECT_EQ(Fo.size(), 10);
    EXPECT_EQ(Ft.size(), 2);
    EXPECT_EQ(F_tot, F_exp);

    Fi.clear(); Fo.clear(); Ft.clear();
    I->faceInfo(1, Fi, Fo, Ft);
    F_tot.insert(Fi.begin(), Fi.end());
    F_tot.insert(Fo.begin(), Fo.end());
    F_tot.insert(Ft.begin(), Ft.end());
    EXPECT_EQ(Fi.size(), 2);
    EXPECT_EQ(Fo.size(), 2);
    EXPECT_EQ(Ft.size(), 8);
    EXPECT_EQ(F_tot, F_exp);
  }

  // get faces of outside/inside/intersection
  TEST_F(IntersectionTest, faceInfo2)
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
    
    I->faceInfo(0, Fi, Fo, Ft);
    F_tot.insert(Fi.begin(), Fi.end());
    F_tot.insert(Fo.begin(), Fo.end());
    F_tot.insert(Ft.begin(), Ft.end());
    EXPECT_EQ(0, Fi.size());
    EXPECT_EQ(6, Fo.size());
    EXPECT_EQ(6, Ft.size());
    EXPECT_EQ(F_tot, F_exp);

    Fi.clear(); Fo.clear(); Ft.clear();
    I->faceInfo(1, Fi, Fo, Ft);
    F_tot.insert(Fi.begin(), Fi.end());
    F_tot.insert(Fo.begin(), Fo.end());
    F_tot.insert(Ft.begin(), Ft.end());
    EXPECT_EQ(0, Fi.size());
    EXPECT_EQ(6, Fo.size());
    EXPECT_EQ(6, Ft.size());
    EXPECT_EQ(F_tot, F_exp);
  }

  TEST_F(IntersectionTest, geometries1)
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
    EXPECT_EQ(I->numPoints(0), 7);
    EXPECT_EQ(I->numPoints(1), 7);

    // get intermediate geometries
    Eigen::MatrixXd V;
    Eigen::MatrixXi F1i;
    Eigen::MatrixXi F1o;
    Eigen::MatrixXi F2i;
    Eigen::MatrixXi F2o;
    
    I->get(V, F1o, F1i, F2o, F2i);
  }

  TEST_F(IntersectionTest, divide)
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

}
