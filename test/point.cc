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

    p1.typeMesh0 = SEGMENT;
    p1.facesMesh0.insert(0);
    p1.facesMesh0.insert(1);
    p1.typeMesh1 = FACE;
    p1.facesMesh1.insert(2);
    
    p2.typeMesh0 = FACE;
    p2.facesMesh0.insert(0);
    p2.typeMesh1 = SEGMENT;
    p2.facesMesh1.insert(1);
    p2.facesMesh1.insert(2);

    p3.typeMesh0 = FACE;
    p3.facesMesh0.insert(0);
    p3.typeMesh1 = SEGMENT;
    p3.facesMesh1.insert(3);
    p3.facesMesh1.insert(4);

    EXPECT_TRUE(p1.connected(p2));
    EXPECT_FALSE(p1.connected(p3));
  }

  TEST_F(PointTest, Flip)
  {
    Point p1(Vector3d(0,0,0));

    p1.typeMesh0 = SEGMENT;
    p1.facesMesh0.insert(0);
    p1.facesMesh0.insert(1);
    p1.typeMesh1 = FACE;
    p1.facesMesh1.insert(2);

    p1.flip();
    
    EXPECT_TRUE(p1.typeMesh0 == FACE);
    EXPECT_TRUE(p1.typeMesh1 == SEGMENT);
  }
}
