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

  TEST_F(IntersectionTest, geometries1)
  {
    Intersections * I = new Intersections();

    // create two cube geometries
    CubeNode * c1 = new CubeNode(10,10,10);
    CubeNode * c2 = new CubeNode(10,10,10);
    TranslateNode * t = new TranslateNode(1,2,3);
    UnionNode * u = new UnionNode();    
    t->add(c2);
    t->build();

    I->add(*c1->g, *t->g);

    // test numPoints
    EXPECT_EQ(I->numPoints(0), 7);
    EXPECT_EQ(I->numPoints(1), 7);

    
  }

}
