#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Face.h"
#include "Point.h"

namespace {
  using namespace Geotree;

  class FaceTest : public ::testing::Test {
  protected:
  };

  TEST_F(FaceTest, Basic)
  {
    Face face(Vector3d(0,0,0), Vector3d(0,0,1), Vector3d(0,0,2), 0);
  }

  TEST_F(FaceTest, intersect)
  {
    Face face(Vector3d(0,0,0), Vector3d(3,0,0), Vector3d(0,3,0), 0);
    Segment segment(Vector3d(1,1,-1), Vector3d(1,1,1), 0);

    std::vector <Point> points;
    
    face.intersect(segment, points);

    EXPECT_EQ(points.size(), 1);
  }

  TEST_F(FaceTest, split)
  {
    Face face(Vector3d(0,0,0), Vector3d(1,0,0), Vector3d(0,1,0), 0);

    std::vector <std::set<Point>> paths;
    std::set<Point> path;
    
    Point p0(Vector3d(0.5,0,0));
    Point p1(Vector3d(0,0.5,0));

    path.insert(p0);
    path.insert(p1);
    paths.push_back(path);
    std::vector <Matrix<int, Dynamic, 3>> split = face.split(paths);

    EXPECT_EQ(split.size(), 2);
  }
}
