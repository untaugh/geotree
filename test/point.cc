#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Point.h"

namespace {
  using namespace Geotree;

  class PointTest : public ::testing::Test {
  };

  TEST_F(PointTest, Basic)
  {
    Vector3d v(0,0,0);
    Point point(v);

    EXPECT_TRUE(v == point.vector);
  }

  TEST_F(PointTest, Compare)
  {
    Point p1(Vector3d(0,0,0));
    Point p2(Vector3d(0,0,0));
    Point p3(Vector3d(0,0,1));

    EXPECT_TRUE(p1 == p2);
    EXPECT_TRUE(p1 != p3);
    EXPECT_FALSE(p1 == p3);
  }

  TEST_F(PointTest, Connected)
  {
    Point p1(Vector3d(0,0,0));
    Point p2(Vector3d(0,0,1));
    Point p3(Vector3d(0,0,2));

    p1.facesMesh0.insert(0);
    p1.facesMesh0.insert(1);
    p1.facesMesh1.insert(2);
    
    p2.facesMesh0.insert(0);
    p2.facesMesh1.insert(1);
    p2.facesMesh1.insert(2);

    p3.facesMesh0.insert(0);
    p3.facesMesh1.insert(3);
    p3.facesMesh1.insert(4);

    EXPECT_TRUE(p1.connected(p2));
    EXPECT_FALSE(p1.connected(p3));
  }

  TEST_F(PointTest, Flip)
  {
    Point p1(Vector3d(0,0,0));

    p1.facesMesh0.insert(0);
    p1.facesMesh0.insert(1);
    p1.facesMesh1.insert(2);

    p1.flip();
    
    EXPECT_EQ(p1.facesMesh0.size(), 1);
    EXPECT_EQ(p1.facesMesh1.size(), 2);
  }

  TEST_F(PointTest, Inside)
  {
    Point p0(Vector3d(1,1,1));
    Point p1(Vector3d(1,1,10));

    Cube box;
    box.p0 = Vector3d(0,0,0);
    box.p1 = Vector3d(2,2,2);

    EXPECT_TRUE(p0.inside(box));
    EXPECT_FALSE(p1.inside(box));
  }

  TEST_F(PointTest, InsideEdge)
  {
    Point p0(Vector3d(1,0.5,0.5));
    Point p1(Vector3d(1.0001,0.5,0.5));

    Cube box;
    box.p0 = Vector3d(0,0,0);
    box.p1 = Vector3d(1,1,1);

    EXPECT_TRUE(p0.inside(box));
    EXPECT_FALSE(p1.inside(box));
  }

  TEST_F(PointTest, InsideNegative)
  {
    Point p0(Vector3d(-1.5, -1.5, -1.5));
    Point p1(Vector3d(-1.5, -1.5, 1.5));

    Cube box;
    box.p0 = Vector3d(-2,-2,-2);
    box.p1 = Vector3d(-1,-1,-1);

    EXPECT_TRUE(p0.inside(box));
    EXPECT_FALSE(p1.inside(box));
  }
}
