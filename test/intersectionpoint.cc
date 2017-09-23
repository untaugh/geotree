#include "gtest/gtest.h"
#include <Eigen/Core>
#include "IntersectionPoint.h"
#include <vector>

namespace {

  using namespace Geotree;
  
  class IntersectionPointTest : public ::testing::Test {
  protected:
    IntersectionPointTest()
    {
    }
    ~IntersectionPointTest()
    {
    }
  };

  TEST_F(IntersectionPointTest, first)
  {
    PointType pointFace(PointType::FACE, 0);
    PointType pointSegment(PointType::SEGMENT, 0);
    PointType pointPoint(PointType::POINT, 0);

    EXPECT_EQ(pointFace.type, PointType::FACE);
    EXPECT_EQ(pointSegment.type, PointType::SEGMENT);
    EXPECT_EQ(pointPoint.type, PointType::POINT);    
  }

   TEST_F(IntersectionPointTest, MeshIntersection)
   {
     MeshIntersection mi;

     PointType pt1(PointType::FACE, 0);
     PointType pt2(PointType::SEGMENT, 0);
     Vector v(0,0,0);
     IntersectionPoint ip(pt1, pt2, v);
       
     mi.addPoint(ip);
   }
  
  // TEST_F(IntersectionPointTest, intersectionPoint)
  // {
  //   PointTypeFace pointFace;
  //   PointTypeSegment pointSegment;
  //   PointTypePoint pointPoint;

  //   std::vector<IntersectionPoint> points;

  //   IntersectionPoint ip(pointFace, pointSegment);
  //   IntersectionPoint ip2(PointTypeFace(), pointSegment);
      
  //   points.push_back(ip);

  //   EXPECT_EQ(ip.type.first.type, PointType::FACE);
  //   EXPECT_EQ(ip.type.second.type, PointType::SEGMENT);
    
  // }
}
