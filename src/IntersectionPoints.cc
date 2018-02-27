#include "IntersectionPoints.h"
#include "Log.h"

namespace Geotree
{
void IntersectionPoints::addPoint(IntersectionPoint _point)
  {
    bool exists = false;
    for (IntersectionPoint ip : points)
      {	
	if (ip == _point)
	  {
	    exists = true;
	    break;
	  }
      }

    if (!exists)
      {
	Geotree::Log().Get(LOG_DEBUG)
	  << "Adding point: "  << _point.first.type << ", " << _point.second.type << ", " << _point.getVector().transpose();
	points.push_back(_point);
      }
  }


    IntersectionPoint IntersectionPoints::getPoint(int pointIndex)
  const {
    return points[pointIndex];
  }

  FaceSet IntersectionPoints::getConnectedPoints(int pointIndex)
  {
    FaceSet intersectionPoints;
    IntersectionPoint thisPoint = getPoint(pointIndex);
    int i =0;
    for (IntersectionPoint ip : points)
      {
	if (thisPoint.isConnected(ip))
	  {
	    intersectionPoints.insert(i);
	  }
	i++;
      }

    intersectionPoints.erase(pointIndex);

    return intersectionPoints;
  }

  void IntersectionPoints::calculateConnectedPoints()
  {
    for (unsigned i=0; i<points.size(); i++)
      {
  	points[i].connectedPoints = getConnectedPoints(i);	
      }
  }

  // Faces IntersectionPoints::getIntersectedFaces2()
  // {
  //   std::set <Face> faces;

  //   for (IntersectionPoint ip : points)
  //     {
  // 	if (ip.first.type == FACE)
  // 	  {
  // 	    //faces.insert(ip.first.index2);
  // 	  }
  // 	else if(ip.first.type == SEGMENT)
  // 	  {
  // 	    faces.insert(ip.first.faceIndex.row(0));
  // 	    faces.insert(ip.first.faceIndex.row(1));
  // 	  }

  // 	if (ip.second.type == FACE)
  // 	  {
  // 	    //faces.insert(ip.second.index2);
  // 	  }
  // 	else if(ip.second.type == SEGMENT)
  // 	  {
  // 	    faces.insert(ip.second.faceIndex.row(0));
  // 	    faces.insert(ip.second.faceIndex.row(1));
  // 	  }

  //     }
  //   Faces resultFaces(faces.size(),3);
  //   int i = 0;
  //   for (Face face : faces)
  //     {
  // 	resultFaces.row(i) = face;
  // 	i++;
  //     }
  //   return resultFaces;    
  // }
  
  FaceSet IntersectionPoints::getIntersectedFaces()
  {
    FaceSet intersectedFaces;

    for (IntersectionPoint ip : points)
      {
	FaceSet pointFaces = ip.getIntersectedFaces();

	intersectedFaces.insert(pointFaces.begin(), pointFaces.end());
      }
    
    return intersectedFaces;
  }

    std::vector<PathX<FacePoint>> IntersectionPoints::getFacePath(const int index) {
        std::vector<PathX<FacePoint>> paths;
        PathX<FacePoint> path(mesh);
        path.add(Vector3d(0,0,0));
        path.add(Vector3d(1,0,0));
        paths.push_back(path);
        return paths;
    }

}
