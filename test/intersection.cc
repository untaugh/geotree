#include "gtest/gtest.h"

#include "Intersect.h"
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
    I->add(0, 0, Vector2i(3,4), Vector3d(1,1,0));
    I->add(1, 3, Vector2i(0,1), Vector3d(1,0,0));
    I->add(1, 4, Vector2i(0,1), Vector3d(2,0,0));
    I->add(0, 1, Vector2i(3,4), Vector3d(3,0,0));

    EXPECT_EQ(I->numPoints(0), 2);
    EXPECT_EQ(I->numPoints(1), 2);

    std::vector<std::set<unsigned>> paths;
    std::set<unsigned> path;

    paths = I->getPaths(0,0);
    path = {1,0,2};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(1,3);
    path = {0,1,3};
    EXPECT_EQ(paths[0], path);
  }

  // Intersections between a few triangular faces
  TEST_F(IntersectionTest, )
  {
    Intersections * I = new Intersections();
    std::vector<std::set<unsigned>> paths;
    std::set<unsigned> path;

    I->add(1, 1, Vector2i(1,2), Vector3d(0,0,0));
    I->add(0, 1, Vector2i(0,1), Vector3d(0,0,0));
    I->add(1, 2, Vector2i(2,3), Vector3d(0,0,0));
    I->add(0, 2, Vector2i(1,2), Vector3d(0,0,0));
    I->add(0, 3, Vector2i(2,3), Vector3d(0,0,0));

    EXPECT_EQ(I->numPoints(0), 3);
    EXPECT_EQ(I->numPoints(1), 2);

    paths = I->getPaths(1,1);
    path = {1,0,3};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(1,2);
    path = {3,2,4};
    EXPECT_EQ(paths[0], path);

    paths = I->getPaths(0,2);
    path = {0,3,2};
    EXPECT_EQ(paths[0], path);
  }
  
}
