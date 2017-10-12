#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>
#include "MeshIntersection.h"
#include "IntersectionPoint.h"
#include "MeshFactory.h"

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

  int countPointsAdjascent(MeshIntersection mi)
  {
    mi.calculatePointFaces();
    mi.calculateConnectedPoints();
    std::pair <int, int> points;
    FaceSet pointsset;
    
    int previouspoint = -1;
    int point = 0;
    int count = 0;
    do
      {
	pointsset = mi.getPoint(point).connectedPoints;
	
	if (pointsset.size() == 3)
	  {
	    for (int i: pointsset)
	      {
		if (mi.getPoint(i).connectedPoints.size() == 3)
		  {
		    pointsset.erase(i);
		  }
	      }
	  }

	if (pointsset.size() != 2)
	  {
	    std::cout << "Size is not 2: " << pointsset.size() <<std::endl;
	    return -1;
	  }

	points.first = *pointsset.begin();
	points.second = *pointsset.rbegin();
	//std::cout << point << " points: " << points.first << ", " <<points.second << std::endl;
	if (points.first != previouspoint)
	  {
	    previouspoint = point;
	    point = points.first;
	  }
	else if (points.second != previouspoint)
	  {
	    previouspoint = point;
	    point = points.second;
	  }
	else
	  {
	    return -1;
	  }
	count++;
      } while(point != 0);

    return count;
  }

  TEST_F(IntersectionPointTest, MeshIntersection)
  {
    Mesh m, n;

    m.V = Verticies(4,3);
    m.F = Faces(4,3);
    m.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    m.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

    n.V = Verticies(4,3);
    n.F = Faces(4,3);
    n.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    n.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;
    n.translate(Vector(0.1, 0.1, 0.1));

    MeshIntersection mi(m,n);


    mi.calculatePoints();
    mi.calculatePointFaces();  
    mi.calculateConnectedPoints();
    mi.calculatePaths();    
    mi.calculatePointFaces();
    
    EXPECT_EQ(mi.mesh1.size(), 4);
    EXPECT_EQ(mi.mesh2.size(), 4);
    //EXPECT_EQ(mi.mesh1.S.rows(), 6);
    //EXPECT_EQ(mi.mesh2.S.rows(), 6);
    EXPECT_EQ(mi.points.size(), 3);

    for (IntersectionPoint p : mi.points)
      {
	EXPECT_EQ(p.first.type, PointInfo::FACE);
	EXPECT_EQ(p.second.type, PointInfo::SEGMENT);  
      }

    EXPECT_EQ(countPointsAdjascent(mi),3);

    std::pair <FaceSet, FaceSet> ifaces;
    std::vector <FaceSet> subpaths;
    MeshFace face;
    std::vector <EdgePoint> edgePoints;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 1);
    EXPECT_EQ(ifaces.second.size(), 3);

    face.mesh = FIRST;
    face.index = *ifaces.first.begin();
    subpaths = mi.getSubPaths(face);
    ASSERT_EQ(subpaths.size(), 1);
    EXPECT_EQ(subpaths[0].size(), 3);
    edgePoints = mi.getEdgePoints(face, subpaths);
    EXPECT_EQ(edgePoints.size(), 3);

    face.mesh = SECOND;
    for (int i : ifaces.second)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 1);
	EXPECT_EQ(subpaths[0].size(), 2);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
	mi.sortEdgePoints(face, edgePoints);
	mi.printPoints(edgePoints);
      }
  }

  TEST_F(IntersectionPointTest, MeshIntersectionSegment)
  {
    Mesh m, n;

    m.V = Verticies(4,3);
    m.F = Faces(4,3);
    m.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    m.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

    n.V = Verticies(4,3);
    n.F = Faces(4,3);
    n.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    n.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;
    n.translate(Vector(0.1, 0.1, 0.0));

    MeshIntersection mi(m,n);


    mi.calculatePoints();
    mi.calculatePointFaces();
    mi.calculateConnectedPoints();
    mi.calculatePaths();    
    
    EXPECT_EQ(mi.points.size(), 4); 

    int i = 0;
    std::pair <int,int> points;
    
    for (IntersectionPoint p : mi.points)
      {
	double d9 = 0.9;
	(*(long*)&d9) -=1;

	double d8 = 0.8;
	(*(long*)&d8) -=1;

	if (p.point == Vector(0.1, 0.1, 0.0))
	  {
	    EXPECT_EQ(p.first.type, PointInfo::FACE);
	    EXPECT_EQ(p.second.type, PointInfo::POINT);
	    EXPECT_EQ(p.connectedPoints.size(), 2);
	  }
	else if (p.point == Vector(d9, 0.1, 0))
	  {
	    EXPECT_EQ(p.first.type, PointInfo::SEGMENT);
	    EXPECT_EQ(p.second.type, PointInfo::SEGMENT);
	    EXPECT_EQ(p.connectedPoints.size(), 3);
	  }
	else if (p.point == Vector(0.1, d9, 0.0))
	  {
	    EXPECT_EQ(p.first.type, PointInfo::SEGMENT);
	    EXPECT_EQ(p.second.type, PointInfo::SEGMENT);
	    EXPECT_EQ(p.connectedPoints.size(), 3);
	  }
	else if (p.point == Vector(0.1, 0.1, d8))
	  {
	    EXPECT_EQ(p.first.type, PointInfo::FACE);
	    EXPECT_EQ(p.second.type, PointInfo::SEGMENT);
	    EXPECT_EQ(p.connectedPoints.size(), 2);
	  }
	else
	  {
	    ASSERT_TRUE(0);
	  }
	i++;
      }

    EXPECT_EQ(countPointsAdjascent(mi),4);

    mi.calculatePointFaces();

    std::pair <FaceSet, FaceSet> ifaces;
        std::vector <FaceSet> subpaths;
        MeshFace face;
    std::vector <EdgePoint> edgePoints;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 2);
    EXPECT_EQ(ifaces.second.size(), 3);

    face.mesh = FIRST;
    for (int i : ifaces.first)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 1);
	EXPECT_EQ(subpaths[0].size(), 3);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
      }

    face.mesh = SECOND;
    for (int i : ifaces.second)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 1);
	EXPECT_EQ(subpaths[0].size(), 3);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
      }    
  }

  TEST_F(IntersectionPointTest, MeshIntersection2)
  {
    Mesh m, n;

    m.V = Verticies(4,3);
    m.F = Faces(4,3);
    m.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    m.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

    n.V = Verticies(4,3);
    n.F = Faces(4,3);
    n.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    n.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;
    n.translate(Vector(0.1, 0.1, 0.1));

    MeshIntersection mi(n,m);


    mi.calculatePoints();

    EXPECT_EQ(mi.points.size(), 3);
  }

  TEST_F(IntersectionPointTest, MeshIntersectionPoint)
  {
    Mesh m, n;

    m.V = Verticies(4,3);
    m.F = Faces(4,3);
    m.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    m.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

    n.V = Verticies(4,3);
    n.F = Faces(4,3);
    n.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    n.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;
    n.translate(Vector(1.0, 0, 0));

    MeshIntersection mi(m,n);


    mi.calculatePoints();

    EXPECT_EQ(mi.points.size(), 1);

    for (IntersectionPoint p : mi.points)
      {
	EXPECT_EQ(p.first.type, PointInfo::POINT);
	EXPECT_EQ(p.second.type, PointInfo::POINT);
	EXPECT_EQ(p.point, Vector(1,0,0));
      }

    mi.calculatePointFaces();

    std::pair <FaceSet, FaceSet> ifaces;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 0);
    EXPECT_EQ(ifaces.second.size(), 0);
  }

  TEST_F(IntersectionPointTest, MeshIntersectionPoint2)
  {
    Mesh m, n;

    m.V = Verticies(4,3);
    m.F = Faces(4,3);
    m.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    m.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

    n.V = Verticies(4,3);
    n.F = Faces(4,3);
    n.V << 0,0,0, 1,0,0, 1,1,0, 1,0,1;
    n.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;
    n.translate(Vector(-1.0, 0, 0));
    
    MeshIntersection mi(m,n);


    mi.calculatePoints();

    EXPECT_EQ(mi.points.size(), 3);

    for (IntersectionPoint p : mi.points)
      {
	EXPECT_EQ(p.first.type, PointInfo::POINT);
	EXPECT_EQ(p.second.type, PointInfo::POINT);
	EXPECT_TRUE(p.point == Vector(0,0,0) || p.point == Vector(0,1,0) || p.point == Vector(0,0,1));
      }

    mi.calculatePointFaces();

    std::pair <FaceSet, FaceSet> ifaces;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 0);
    EXPECT_EQ(ifaces.second.size(), 0);    
  }

  TEST_F(IntersectionPointTest, MeshIntersectionPoint3)
  {
    Mesh m, n;

    m.V = Verticies(4,3);
    m.F = Faces(4,3);
    m.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    m.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

    n.V = Verticies(4,3);
    n.F = Faces(4,3);
    n.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    n.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;
    
    MeshIntersection mi(m,n);


    mi.calculatePoints();
    mi.calculatePointFaces();
    
    EXPECT_EQ(mi.points.size(), 4);

    for (IntersectionPoint p : mi.points)
      {
	EXPECT_EQ(p.first.type, PointInfo::POINT);
	EXPECT_EQ(p.second.type, PointInfo::POINT);
	EXPECT_TRUE(p.point == Vector(0,0,0) || p.point == Vector(0,1,0) || p.point == Vector(0,0,1) || p.point == Vector(1,0,0));
      }

    std::pair <FaceSet, FaceSet> ifaces;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 0);
    EXPECT_EQ(ifaces.second.size(), 0);
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(10,1,1);
    cube2.translate(Vector(0.1, 0.1, 8));
    MeshIntersection mi(cube1,cube2);


    mi.calculatePoints();
    mi.calculatePointFaces();
    mi.calculateConnectedPoints();
    mi.calculatePaths();    
    
    EXPECT_EQ(mi.points.size(), 8);

    for (IntersectionPoint p : mi.points)
      {
	EXPECT_EQ(p.first.type, PointInfo::FACE);
	EXPECT_EQ(p.second.type, PointInfo::SEGMENT);
	EXPECT_EQ(p.point(0), 10);
	
	//std::cout << p.first.type << ":"<<p.second.type<<":("<<p.point.transpose()<<")"<<std::endl;
      }
   
    EXPECT_EQ(countPointsAdjascent(mi),8);

    std::pair <FaceSet, FaceSet> ifaces;
        std::vector <FaceSet> subpaths;
        MeshFace face;
    std::vector <EdgePoint> edgePoints;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 1);
    EXPECT_EQ(ifaces.second.size(), 8);

    face.mesh = FIRST;
    face.index = *ifaces.first.begin();
    subpaths = mi.getSubPaths(face);
    ASSERT_EQ(subpaths.size(), 1);
    EXPECT_EQ(subpaths[0].size(), 8);
    edgePoints = mi.getEdgePoints(face, subpaths);
    EXPECT_EQ(edgePoints.size(), 3);
    mi.sortEdgePoints(face, edgePoints);
    mi.printPoints(edgePoints);

    face.mesh = SECOND;
    face.index = *ifaces.second.begin();
    subpaths = mi.getSubPaths(face);
    ASSERT_EQ(subpaths.size(), 1);
    EXPECT_EQ(subpaths[0].size(), 2);
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube2)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(10,1,1);
    cube2.translate(Vector(1, 1, 1));
    MeshIntersection mi(cube1,cube2);


    mi.calculatePoints();
    mi.calculatePointFaces();
    mi.calculateConnectedPoints();
    mi.calculatePaths();
    
    EXPECT_EQ(mi.points.size(), 8);

    for (IntersectionPoint p : mi.points)
      {
	EXPECT_EQ(p.second.type, PointInfo::SEGMENT);
	EXPECT_EQ(p.point(0), 10);
	
	//std::cout << p.first.type << ":"<<p.second.type<<":("<<p.point.transpose()<<")"<<std::endl;
      }
   
    EXPECT_EQ(countPointsAdjascent(mi),8);

    std::pair <FaceSet, FaceSet> ifaces;
        std::vector <FaceSet> subpaths;
        MeshFace face;
    std::vector <EdgePoint> edgePoints;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 2);
    EXPECT_EQ(ifaces.second.size(), 8);
    
    face.mesh = FIRST;
    for (int i : ifaces.first)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 1);
	EXPECT_EQ(subpaths[0].size(), 5);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
      }

    face.mesh = SECOND;
    for (int i : ifaces.second)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 1);
	EXPECT_EQ(subpaths[0].size(), 2);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
      }    
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube3)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(10,10,10);
    cube2.translate(Vector(1, 1, 1));
    MeshIntersection mi(cube1,cube2);

    mi.calculatePoints();
    mi.calculatePointFaces();
    mi.calculateConnectedPoints();
    mi.calculatePaths();

    EXPECT_EQ(mi.points.size(), 10);
   
    EXPECT_EQ(countPointsAdjascent(mi),10);

    std::pair <FaceSet, FaceSet> ifaces;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 6);
    EXPECT_EQ(ifaces.second.size(), 6);
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube4)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(12,12,4);
    cube2.translate(Vector(-1, -1, 2));
    MeshIntersection mi(cube1,cube2);


    mi.calculatePoints();
    mi.calculatePointFaces();
    mi.calculateConnectedPoints();
    mi.calculatePaths();
    
    EXPECT_EQ(mi.points.size(), 16);
   
    EXPECT_EQ(countPointsAdjascent(mi),8);

    EXPECT_EQ(mi.paths.size(),2);

    std::pair <FaceSet, FaceSet> ifaces;
        std::vector <FaceSet> subpaths;
        MeshFace face;
    std::vector <EdgePoint> edgePoints;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 8);
    EXPECT_EQ(ifaces.second.size(), 4);

    face.mesh = FIRST;
    for (int i : ifaces.first)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 2);
	EXPECT_EQ(subpaths[0].size(), 2);
	EXPECT_EQ(subpaths[1].size(), 2);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 7);
	mi.sortEdgePoints(face, edgePoints);
	mi.printPoints(edgePoints);
      }   

    face.mesh = SECOND;
    for (int i : ifaces.second)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 1);
	EXPECT_EQ(subpaths[0].size(), 5);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
      }
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube5)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,320);

    Mesh cube2;

    for (int i=1; i<101; i++)
      {
	Mesh cube3 = mf.makeCube(12,12,0.5);
	cube3.translate(Vector(-1, -1, i));
	cube2 = cube2 + cube3;	
      }

    MeshIntersection mi(cube1,cube2);


    //EXPECT_EQ(mi.mesh2.S.rows(), 100 * 18);
    
    mi.calculatePoints();

    EXPECT_EQ(mi.points.size(), 100 * 16);

    mi.calculatePointFaces();
    mi.calculateConnectedPoints();
    
    mi.calculatePaths();
    EXPECT_EQ(mi.paths.size(), 200);

    mi.calculatePointFaces();

    std::pair <FaceSet, FaceSet> ifaces;
    std::vector <FaceSet> subpaths;
        MeshFace face;
    std::vector <EdgePoint> edgePoints;
    ifaces = mi.getIntersectedFaces();
    EXPECT_EQ(ifaces.first.size(), 8);
    EXPECT_EQ(ifaces.second.size(), 400);

    face.mesh = FIRST;
    for (int i : ifaces.first)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	EXPECT_EQ(subpaths.size(), 200);
	for (FaceSet subpath : subpaths)
	  {
	    EXPECT_EQ(subpath.size(), 2);
	  }
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 403);
	mi.sortEdgePoints(face, edgePoints);
	mi.printPoints(edgePoints);
      }
    
    face.mesh = SECOND;
    for (int i : ifaces.second)
      {
	face.index = i;
	subpaths = mi.getSubPaths(face);
	EXPECT_EQ(subpaths.size(), 1);
	for (FaceSet subpath : subpaths)
	  {
	    EXPECT_EQ(subpath.size(), 5);
	  }
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
      }
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube6)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,120);

    Mesh cube2;
    
    Mesh cube3 = mf.makeCube(12,12,0.5);
    cube3.translate(Vector(-1, -1, 15));
    
    MeshIntersection mi(cube1,cube3);


    //EXPECT_EQ(mi.mesh2.S.rows(), 1 * 18);
    
    mi.calculatePoints();

    EXPECT_EQ(mi.points.size(), 16);

    mi.calculatePointFaces();
    mi.calculateConnectedPoints();
    mi.printPoints();
    
    mi.calculatePaths();
    EXPECT_EQ(mi.paths.size(),2);
  }
  
  // TEST_F(IntersectionPointTest, DISABLED_AdjascentPoints)
  // {
  //   Mesh m, n;

  //   m.V = Verticies(4,3);
  //   m.F = Faces(4,3);
  //   m.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
  //   m.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;

  //   n.V = Verticies(4,3);
  //   n.F = Faces(4,3);
  //   n.V << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
  //   n.F << 0,1,2, 0,1,3, 0,2,3, 1,2,3;   

  //   MeshIntersection mi(m, n);



  //   //std::cout << mi.mesh1.S << std::endl;

  //   // PointInfo p1(PointInfo::FACE, 3);
  //   // PointInfo p2(PointInfo::SEGMENT, 0);
  //   // PointInfo p3(PointInfo::SEGMENT, 1);
  //   // PointInfo p4(PointInfo::SEGMENT, 2);
  //   // PointInfo p5(PointInfo::SEGMENT, 10);
  //   // PointInfo p6(PointInfo::FACE, 0);
  //   IntersectionPoint ip1(p1, p2, Vector(0,0,0));
  //   IntersectionPoint ip2(p1, p3, Vector(0,0,0));
  //   IntersectionPoint ip3(p1, p4, Vector(0,0,0));
  //   IntersectionPoint ip4(p1, p5, Vector(0,0,0));

  //   mi.addPoint(ip1);
  //   mi.addPoint(ip2);
  //   mi.addPoint(ip3);
  //   mi.addPoint(ip4);

  //   std::pair <int, int> points;

  //   mi.calculatePointFaces();
  //   mi.calculateConnectedPoints();

  //   EXPECT_EQ(*mi.getPoint(0).connectedPoints.begin(), 1);
  //   EXPECT_EQ(*mi.getPoint(0).connectedPoints.rbegin(), 2);
  //   EXPECT_EQ(*mi.getPoint(1).connectedPoints.begin(), 0);
  //   EXPECT_EQ(*mi.getPoint(1).connectedPoints.rbegin(), 2);
  //   EXPECT_EQ(*mi.getPoint(2).connectedPoints.begin(), 0);
  //   EXPECT_EQ(*mi.getPoint(2).connectedPoints.rbegin(), 1);  
  // }
}
