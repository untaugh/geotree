#include "gtest/gtest.h"
#include <Eigen/Core>
#include "Face2D.h"
#include <algorithm>

namespace {
  using namespace Geotree;

  class Face2DTest : public ::testing::Test {
  protected:
  };

  TEST_F(Face2DTest, Basic)
  {
    Face2D face;

    face.p0 = {0,0};
    face.p1 = {1,0};
    face.p2 = {0,1};

    Path2D path;
    
    Point2D p0 = {0.5, 0};
    Point2D p1 = {0, 0.5};

    path.points.push_back(p0);
    path.points.push_back(p1);
    path.edgetoedge = true;
    path.begin = {0, 0.5, &p0};
    path.end = {2, 0.5, &p1};

    std::vector <Path2D> paths;
    paths.push_back(path);
    
    std::vector <std::vector<uint32_t>> groups = face.split(paths);

    for (std::vector<uint32_t> faces : groups)
    {
      for (uint32_t i : faces)
      {
	std::cout << i << ", ";
      }
      std::cout << std::endl;
    }

    std::vector<uint32_t> group0_ref = {2, 4, 3, 3, 1, 2};
    std::vector<uint32_t> group1_ref = {3, 4, 0};

    EXPECT_EQ(groups.size(), 2);
    EXPECT_EQ(groups[0], group0_ref);
    EXPECT_EQ(groups[1], group1_ref);
  }

  TEST_F(Face2DTest, Double)
  {
    Face2D face;

    face.p0 = {0,0};
    face.p1 = {1,0};
    face.p2 = {0,1};

    Path2D path0;
    Path2D path1;

    Point2D p0 = {0.5, 0};
    Point2D p1 = {0, 0.5};
    Point2D p2 = {0.7, 0};
    Point2D p3 = {0, 0.7};
    
    path0.points.push_back(p0);
    path0.points.push_back(p1);
    path0.edgetoedge = true;
    path0.begin = {0, 0.5, &p0};
    path0.end = {2, 0.5, &p1};

    path1.points.push_back(p2);
    path1.points.push_back(p3);
    path1.edgetoedge = true;
    path1.begin = {0, 0.7, &p2};
    path1.end = {2, 0.3, &p3};

    std::vector <Path2D> paths;
    paths.push_back(path0);
    paths.push_back(path1);
    
    std::vector <std::vector<uint32_t>> groups = face.split(paths);

    std::vector<uint32_t> group0_ref = {2, 6, 5, 5, 1, 2};
    std::vector<uint32_t> group1_ref = {6, 4, 3, 3, 5, 6};
    std::vector<uint32_t> group2_ref = {3, 4, 0};

    EXPECT_EQ(groups.size(), 3);
    EXPECT_EQ(groups[0], group0_ref);
    EXPECT_EQ(groups[1], group1_ref);
    EXPECT_EQ(groups[2], group2_ref);
  }

  TEST_F(Face2DTest, DoubleReverse)
  {
    Face2D face;

    face.p0 = {0,0};
    face.p1 = {1,0};
    face.p2 = {0,1};

    Path2D path0;
    Path2D path1;

    Point2D p0 = {0.5, 0};
    Point2D p1 = {0, 0.5};
    Point2D p2 = {0.7, 0};
    Point2D p3 = {0, 0.7};
    
    path0.points.push_back(p1);
    path0.points.push_back(p0);
    path0.edgetoedge = true;
    path0.begin = {2, 0.5, &p1};
    path0.end = {0, 0.5, &p0};

    path1.points.push_back(p3);
    path1.points.push_back(p2);
    path1.edgetoedge = true;
    path1.begin = {2, 0.7, &p3};
    path1.end = {0, 0.3, &p2};

    std::vector <Path2D> paths;
    paths.push_back(path1);
    paths.push_back(path0);
    
    std::vector <std::vector<uint32_t>> groups = face.split(paths);

    std::vector<uint32_t> group0_ref = {2, 5, 6, 6, 1, 2};
    std::vector<uint32_t> group1_ref = {6, 4, 3, 3, 5, 6};
    std::vector<uint32_t> group2_ref = {4, 3, 0};

    EXPECT_EQ(groups.size(), 3);
    EXPECT_EQ(groups[0], group0_ref);
    EXPECT_EQ(groups[1], group1_ref);
    EXPECT_EQ(groups[2], group2_ref);
  }
}
