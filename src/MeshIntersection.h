#pragma once
#include "Types.h"
#include <vector>
#include <memory>
#include <utility>
#include "Mesh.h"
#include "IntersectionPoint.h"

namespace Geotree
{ 
  struct MeshFace {
    MeshIndex mesh;
    int index;
  };

  enum EdgePointType {
    SEGMENT,
    POINT
  };
  
  struct EdgePoint {
    EdgePointType type;
    int index;
    double position;
    SegmentIndex segment;

    bool operator <(const EdgePoint otherPoint)
    {
      if (this->segment[0] < otherPoint.segment[0])
      	{
      	  return true;
      	}
      else if (this->segment[0] > otherPoint.segment[0])
      	{
      	  return false;
      	}
      else
	{
	  if (this->segment[1] < otherPoint.segment[1])
	    {
	      return true;
	    }
	  else if (this->segment[1] > otherPoint.segment[1])
	    {
	      return false;	      
	    }
	  else if (this->position < otherPoint.position)
	    {
	      return true;
	    }
	  else
	    {
	      return false;
	    }
	}
    }
  };

  class IntersectionPoints
  {
  public:
    IntersectionPoints();
    void addPoint(IntersectionPoint _point);
  private:
    std::vector <IntersectionPoint> points;    
  };
  
  class MeshIntersection
  {
  public:
    MeshIntersection(Mesh _mesh1, Mesh _mesh2);

    Mesh mesh1;
    Mesh mesh2;

    Mesh mesh;

    void calculatePoints();
    void calculatePaths();
    void calculatePointFaces();
    void calculateConnectedPoints();
    void divideFaces();

    std::pair <FaceSet, FaceSet> getIntersectedFaces();
    int getPathStartIndex(Path path, MeshFace face);
    std::vector <FaceSet> getSubPaths(MeshFace face);

    std::vector <EdgePoint> getEdgePoints(MeshFace face, std::vector <FaceSet> &subpaths);
    
    void sortEdgePoints(MeshFace face, std::vector <EdgePoint> &edgePoints);
    
    bool operator ()(EdgePoint first, EdgePoint second);

    void addPoint(IntersectionPoint _point);
    IntersectionPoint getPoint(int pointIndex);

    FaceSet getFaces(Mesh mesh, PointInfo point);
    FaceIndex getFace(MeshFace face);
    Mesh getMesh(MeshIndex mesh);
    FaceSet getIntersectionPoints(FaceSet facesFirst, FaceSet facesSecond);
    FaceSet getConnectedPoints(int pointIndex);
    Path calculatePath(int intersectionPointIndex);
    int totalPathSize();
    int getFreePoint();
    void printPoints();
    void printPoints(std::vector <EdgePoint> edgePoints);
    std::vector <IntersectionPoint> points;

    std::vector <Path> paths;
  };
}
