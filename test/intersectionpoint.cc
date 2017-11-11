#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>
#include "MeshIntersection.h"
#include "IntersectionPoint.h"
#include "MeshFactory.h"

namespace {
  using namespace Geotree;

  class IntersectionPointTest : public ::testing::Test {
  };

  int countPointsAdjascent(IntersectionPoints ip)
  {
    std::pair <int, int> points;
    FaceSet pointsset;
    
    int previouspoint = -1;
    int point = 0;
    int count = 0;
    do
      {
	pointsset = ip.getPoint(point).connectedPoints;       
	
	if (pointsset.size() == 3)
	  {
	    for (int i: pointsset)
	      {
		if (ip.getPoint(i).connectedPoints.size() == 3)
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

    EXPECT_EQ(mi.mesh.size(), 8);
    
    EXPECT_EQ(mi.points.size(), 3);

    for (int i =0; i<mi.points.size(); i++)      
      {
	IntersectionPoint p = mi.points.getPoint(i);
	EXPECT_EQ(p.first.type, FACE);
	EXPECT_EQ(p.second.type, SEGMENT);  
      }

    EXPECT_EQ(countPointsAdjascent(mi.points),3);

    std::vector <FaceSet> subpaths;
    int face;
    std::vector <EdgePoint> edgePoints;
    FaceSet faces = mi.points.getIntersectedFaces();

    EXPECT_EQ(faces.size(), 3 + 1);

    face = *faces.begin();
    subpaths = mi.paths.getSubPaths(face);
    ASSERT_EQ(subpaths.size(), 1);
    EXPECT_EQ(subpaths[0].size(), 3);
    edgePoints = mi.getEdgePoints(face, subpaths);
    EXPECT_EQ(edgePoints.size(), 3);

    //IntersectionFace iface = mi.paths.getFace(*faces.begin());
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


    //mi.calculatePoints();
    //mi.calculatePointFaces();
    //mi.points.calculateConnectedPoints();
    //mi.calculatePaths();    
    
    EXPECT_EQ(mi.points.size(), 4); 

    //int i = 0;
    std::pair <int,int> points;
    
    //for (IntersectionPoint p : mi.points.getPoints())
    for (int i =0; i<mi.points.size(); i++)
      {
	IntersectionPoint p = mi.points.getPoint(i);
	double d9 = 0.9;
	(*(long*)&d9) -=1;

	double d8 = 0.8;
	(*(long*)&d8) -=1;

	if (p.getPoint() == Vector(0.1, 0.1, 0.0))
	  {
	    EXPECT_EQ(p.first.type, FACE);
	    EXPECT_EQ(p.second.type, POINT);
	    EXPECT_EQ(p.connectedPoints.size(), 2);
	  }
	else if (p.getPoint() == Vector(d9, 0.1, 0))
	  {
	    EXPECT_EQ(p.first.type, SEGMENT);
	    EXPECT_EQ(p.second.type, SEGMENT);
	    EXPECT_EQ(p.connectedPoints.size(), 3);
	  }
	else if (p.getPoint() == Vector(0.1, d9, 0.0))
	  {
	    EXPECT_EQ(p.first.type, SEGMENT);
	    EXPECT_EQ(p.second.type, SEGMENT);
	    EXPECT_EQ(p.connectedPoints.size(), 3);
	  }
	else if (p.getPoint() == Vector(0.1, 0.1, d8))
	  {
	    EXPECT_EQ(p.first.type, FACE);
	    EXPECT_EQ(p.second.type, SEGMENT);
	    EXPECT_EQ(p.connectedPoints.size(), 2);
	  }
	else
	  {
	    ASSERT_TRUE(0);
	  }
	i++;
      }

    EXPECT_EQ(countPointsAdjascent(mi.points),4);

    //mi.calculatePointFaces();

    
        std::vector <FaceSet> subpaths;
        int face;
    std::vector <EdgePoint> edgePoints;
    FaceSet faces = mi.points.getIntersectedFaces();

    EXPECT_EQ(faces.size(), 2 + 3);
    
    //face.mesh = FIRST;
    for (int i : faces)
      {
	face = i;
	subpaths = mi.paths.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 1);
	EXPECT_EQ(subpaths[0].size(), 3);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
      }

    //face.mesh = SECOND;
    for (int i : faces)
      {
	face = i;
	subpaths = mi.paths.getSubPaths(face);
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


    //mi.calculatePoints();

    EXPECT_EQ(mi.points.size(), 1);

    //for (IntersectionPoint p : mi.points.getPoints())
      for (int i =0; i<mi.points.size(); i++)
      {
	IntersectionPoint p = mi.points.getPoint(i);
	EXPECT_EQ(p.first.type, POINT);
	EXPECT_EQ(p.second.type, POINT);
	EXPECT_EQ(p.getPoint(), Vector(1,0,0));
      }

    //mi.calculatePointFaces();

    
    FaceSet faces = mi.points.getIntersectedFaces();

    EXPECT_EQ(faces.size(), 0);
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

    EXPECT_EQ(mi.points.size(), 3);

    //for (IntersectionPoint p : mi.points.getPoints())
      for (int i =0; i<mi.points.size(); i++)
      {
	IntersectionPoint p = mi.points.getPoint(i);
	EXPECT_EQ(p.first.type, POINT);
	EXPECT_EQ(p.second.type, POINT);
	EXPECT_TRUE(p.getPoint() == Vector(0,0,0) || p.getPoint() == Vector(0,1,0) || p.getPoint() == Vector(0,0,1));
      }
    
    FaceSet faces = mi.points.getIntersectedFaces();
    EXPECT_EQ(faces.size(), 0);
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

    EXPECT_EQ(mi.points.size(), 4);

    //for (IntersectionPoint p : mi.points.getPoints())
      for (int i =0; i<mi.points.size(); i++)
      {
	IntersectionPoint p = mi.points.getPoint(i);
	EXPECT_EQ(p.first.type, POINT);
	EXPECT_EQ(p.second.type, POINT);
	EXPECT_TRUE(p.getPoint() == Vector(0,0,0) || p.getPoint() == Vector(0,1,0) || p.getPoint() == Vector(0,0,1) || p.getPoint() == Vector(1,0,0));
      }

    
    FaceSet faces = mi.points.getIntersectedFaces();
    EXPECT_EQ(faces.size(), 0);
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(10,1,1);
    cube2.translate(Vector(0.1, 0.1, 8));
    MeshIntersection mi(cube1,cube2);

    EXPECT_EQ(mi.points.size(), 8);

    //for (IntersectionPoint p : mi.points.getPoints())
      for (int i =0; i<mi.points.size(); i++)
      {
	IntersectionPoint p = mi.points.getPoint(i);
	EXPECT_EQ(p.first.type, FACE);
	EXPECT_EQ(p.second.type, SEGMENT);
	EXPECT_EQ(p.getPoint()(0), 10);
      }

    EXPECT_EQ(countPointsAdjascent(mi.points),8);

    
    std::vector <FaceSet> subpaths;
    int face;
    std::vector <EdgePoint> edgePoints;
    FaceSet faces = mi.points.getIntersectedFaces();
    EXPECT_EQ(faces.size(), 8 + 1);

    face = *faces.begin();
    subpaths = mi.paths.getSubPaths(face);
    ASSERT_EQ(subpaths.size(), 1);
    EXPECT_EQ(subpaths[0].size(), 8);
    edgePoints = mi.getEdgePoints(face, subpaths);
    EXPECT_EQ(edgePoints.size(), 3);
    mi.sortEdgePoints(face, edgePoints);
    //mi.printPoints(edgePoints);

    //mi.printPoints();
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube2)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(10,1,1);
    cube2.translate(Vector(1, 1, 1));
    MeshIntersection mi(cube1,cube2);
    
    EXPECT_EQ(mi.points.size(), 8);

    //for (IntersectionPoint p : mi.points.getPoints())
      for (int i =0; i<mi.points.size(); i++)
      {
	IntersectionPoint p = mi.points.getPoint(i);
	EXPECT_EQ(p.second.type, SEGMENT);
	EXPECT_EQ(p.getPoint()(0), 10);	
      }
   
    EXPECT_EQ(countPointsAdjascent(mi.points),8);

    
        std::vector <FaceSet> subpaths;
        int face;
    std::vector <EdgePoint> edgePoints;
    FaceSet faces = mi.points.getIntersectedFaces();
    EXPECT_EQ(faces.size(), 2 + 8);
    
    for (int i : faces)
      {
	face = i;
	subpaths = mi.paths.getSubPaths(face);
	ASSERT_EQ(subpaths.size(), 1);
	//EXPECT_EQ(subpaths[0].size(), 5);
	edgePoints = mi.getEdgePoints(face, subpaths);
	EXPECT_EQ(edgePoints.size(), 5);
      }

    // for (int i : faces)
    //   {
    // 	face = i;
    // 	subpaths = mi.paths.getSubPaths(face);
    // 	ASSERT_EQ(subpaths.size(), 1);
    // 	EXPECT_EQ(subpaths[0].size(), 2);
    // 	edgePoints = mi.getEdgePoints(face, subpaths);
    // 	EXPECT_EQ(edgePoints.size(), 5);
    //   }    
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube3)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(10,10,10);
    cube2.translate(Vector(1, 1, 1));
    MeshIntersection mi(cube1,cube2);

    EXPECT_EQ(mi.points.size(), 10);
   
    EXPECT_EQ(countPointsAdjascent(mi.points),10);

    FaceSet faces = mi.points.getIntersectedFaces();
    EXPECT_EQ(faces.size(), 12);    
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube4)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,10);
    Mesh cube2 = mf.makeCube(12,12,4);
    cube2.translate(Vector(-1, -1, 2));
    MeshIntersection mi(cube1,cube2);
    
    EXPECT_EQ(mi.points.size(), 16);
   
    EXPECT_EQ(countPointsAdjascent(mi.points),8);

    EXPECT_EQ(mi.paths.size(),2);

    
    std::vector <FaceSet> subpaths;
    //int face;
    std::vector <EdgePoint> edgePoints;
    FaceSet faces = mi.points.getIntersectedFaces();
    EXPECT_EQ(faces.size(), 8 + 4);

    // for (int i : faces)
    //   {
    // 	face = i;
    // 	subpaths = mi.paths.getSubPaths(face);
    // 	ASSERT_EQ(subpaths.size(), 2);
    // 	EXPECT_EQ(subpaths[0].size(), 2);
    // 	EXPECT_EQ(subpaths[1].size(), 2);
    // 	edgePoints = mi.getEdgePoints(face, subpaths);
    // 	EXPECT_EQ(edgePoints.size(), 7);
    // 	mi.sortEdgePoints(face, edgePoints);
    // 	//mi.printPoints(edgePoints);
    //   }   

    // for (int i : faces)
    //   {
    // 	face = i;
    // 	subpaths = mi.paths.getSubPaths(face);
    // 	ASSERT_EQ(subpaths.size(), 1);
    // 	EXPECT_EQ(subpaths[0].size(), 5);
    // 	edgePoints = mi.getEdgePoints(face, subpaths);
    // 	EXPECT_EQ(edgePoints.size(), 5);
    //   }
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


    EXPECT_EQ(mi.points.size(), 100 * 16);

    EXPECT_EQ(mi.paths.size(), 200);


    std::vector <FaceSet> subpaths;
    //int face;
    std::vector <EdgePoint> edgePoints;
    FaceSet faces = mi.points.getIntersectedFaces();
    EXPECT_EQ(faces.size(), 400 + 8);

    // for (int i : faces)
    //   {
    // 	face = i;
    // 	subpaths = mi.paths.getSubPaths(face);
    // 	EXPECT_EQ(subpaths.size(), 200);
    // 	for (FaceSet subpath : subpaths)
    // 	  {
    // 	    EXPECT_EQ(subpath.size(), 2);
    // 	  }
    // 	edgePoints = mi.getEdgePoints(face, subpaths);
    // 	EXPECT_EQ(edgePoints.size(), 403);
    // 	mi.sortEdgePoints(face, edgePoints);
    // 	//mi.printPoints(edgePoints);
    //   }

    // for (int i : faces)
    //   {
    // 	face = i;
    // 	subpaths = mi.paths.getSubPaths(face);
    // 	EXPECT_EQ(subpaths.size(), 1);
    // 	for (FaceSet subpath : subpaths)
    // 	  {
    // 	    EXPECT_EQ(subpath.size(), 5);
    // 	  }
    // 	edgePoints = mi.getEdgePoints(face, subpaths);
    // 	EXPECT_EQ(edgePoints.size(), 5);
    //   }
  }

  TEST_F(IntersectionPointTest, MeshIntersectionCube6)
  {
    MeshFactory mf;

    Mesh cube1 = mf.makeCube(10,10,120);

    Mesh cube2;
    
    Mesh cube3 = mf.makeCube(12,12,0.5);
    cube3.translate(Vector(-1, -1, 15));
    
    MeshIntersection mi(cube1,cube3);

    EXPECT_EQ(mi.points.size(), 16);

    //mi.printPoints();
    
    EXPECT_EQ(mi.paths.size(),2);
  }

  // TEST_F(IntersectionPointTest, ConnectedFace)
  // {
  //   PointInfo pi1, pi2;

  //   Mesh mesh;
  //   mesh.F = Faces(3,3);
  //   mesh.F << 1,2,3, 3,2,1, 1,2,4;

  //   pi1.type = FACE;
  //   pi1.index2 << 1,2,3;
  //   pi2.type = FACE;
  //   pi2.index2 << 1,2,3;
  //   EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 3,2,1;
  //   EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 1,2,4;
  //   EXPECT_FALSE(pi1.isConnected(pi2));

  //   pi2.type = SEGMENT;
  //   pi2.index2 << 1,2,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,1,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,3,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 3,1,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 1,4,0; EXPECT_FALSE(pi1.isConnected(pi2));

  //   pi2.type = POINT;
  //   pi2.index2 << 1,0,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,1,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 3,3,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 4,1,0; EXPECT_FALSE(pi1.isConnected(pi2));
  // }

  // TEST_F(IntersectionPointTest, ConnectedSegment)
  // {
  //   PointInfo pi1, pi2;

  //   pi1.type = SEGMENT;
  //   pi1.index2 << 1,2,0;
  //   pi2.type = FACE;
  //   pi2.index2 << 1,2,3;
  //   EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 3,2,1;
  //   EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 5,3,4;
  //   EXPECT_FALSE(pi1.isConnected(pi2));

  //   pi2.type = SEGMENT;
  //   pi1.faceIndex = Faces(2,3);
  //   pi1.faceIndex << 1,2,3, 0,1,2;
  //   pi2.index2 << 1,2,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 1,3,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,3,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 1,4,0; EXPECT_FALSE(pi1.isConnected(pi2));
  //   pi2.index2 << 3,4,0; EXPECT_FALSE(pi1.isConnected(pi2));

  //   pi2.type = POINT;
  //   pi2.index2 << 1,0,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,1,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 4,1,0; EXPECT_FALSE(pi1.isConnected(pi2));
  // }

  // TEST_F(IntersectionPointTest, ConnectedPoint)
  // {
  //   PointInfo pi1;
  //   PointInfo pi2;

  //   pi1.type = POINT;
  //   pi1.index2 << 1,0,0;
  //   pi2.type = FACE;
  //   pi2.index2 << 1,2,3;
  //   EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 3,2,1;
  //   EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,3,4;
  //   EXPECT_FALSE(pi1.isConnected(pi2));

  //   pi2.type = SEGMENT;
  //   pi2.index2 << 1,2,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,1,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,3,0; EXPECT_FALSE(pi1.isConnected(pi2));

  //   pi2.type = POINT;
  //   pi2.index2 << 1,0,0; EXPECT_TRUE(pi1.isConnected(pi2));
  //   pi2.index2 << 2,1,0; EXPECT_FALSE(pi1.isConnected(pi2));
  //   pi2.index2 << 4,1,0; EXPECT_FALSE(pi1.isConnected(pi2));
  // }

  TEST_F(IntersectionPointTest, IntersectedFaces)
  {
    
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

  //   // PointInfo p1(FACE, 3);
  //   // PointInfo p2(SEGMENT, 0);
  //   // PointInfo p3(SEGMENT, 1);
  //   // PointInfo p4(SEGMENT, 2);
  //   // PointInfo p5(SEGMENT, 10);
  //   // PointInfo p6(FACE, 0);
  //   IntersectionPoint ip1(p1, p2, Vector(0,0,0));
  //   IntersectionPoint ip2(p1, p3, Vector(0,0,0));
  //   IntersectionPoint ip3(p1, p4, Vector(0,0,0));
  //   IntersectionPoint ip4(p1, p5, Vector(0,0,0));

  //   mi.addPoint(ip1);
  //   mi.addPoint(ip2);
  //   mi.addPoint(ip3);
  //   mi.addPoint(ip4);

  //   std::pair <int, int> points;

  //   //mi.calculatePointFaces();
  //   //mi.points.calculateConnectedPoints();

  //   EXPECT_EQ(*mi.getPoint(0).connectedPoints.begin(), 1);
  //   EXPECT_EQ(*mi.getPoint(0).connectedPoints.rbegin(), 2);
  //   EXPECT_EQ(*mi.getPoint(1).connectedPoints.begin(), 0);
  //   EXPECT_EQ(*mi.getPoint(1).connectedPoints.rbegin(), 2);
  //   EXPECT_EQ(*mi.getPoint(2).connectedPoints.begin(), 0);
  //   EXPECT_EQ(*mi.getPoint(2).connectedPoints.rbegin(), 1);  
  // }
}
