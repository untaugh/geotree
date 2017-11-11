#include "MeshIntersection.h"



namespace Geotree
{
  std::vector <FaceSet> IntersectionPaths::getSubPaths(int face)
  {
    std::vector <FaceSet> subpaths;
    
    for (Path path : paths)
      {
	int pathStart = getPathStartIndex(path, face);
	int i = 0;

	while(i < path.size())
	  {
	    FaceSet subpath;
	    int pathIndex = (pathStart + i) % path.size();

	    //IntersectionPoint pathPoint = intersectionPoints.getPoint(path[pathIndex]);
	    //while (i < path.size() && pathPoint.getPoint(face.mesh).faces.count(face.index))
	    while (i < path.size() && intersectionPoints.getPoint(path[pathIndex]).faces.count(face))	    
	      {
		subpath.insert(path[pathIndex]);
		++i;
		pathIndex = (pathStart + i) % path.size();
		//pathPoint = intersectionPoints.getPoint(path[pathIndex]);
	      }
	    
	    if (subpath.size() > 0)
	      {
		subpaths.push_back(subpath);
		subpath.clear();
	      }
	    else
	      {
		i++;
	      }
	  }
      }
    return subpaths;
  }
  // std::vector <Path> IntersectionPaths::getPathsForFace(Face face)
  // {
  //   for (Path path : paths)
  //     {
	
  //     }
  // }

    int IntersectionPaths::totalPathSize()
  {
    int totalSize = 0;
    for(Path path : paths)
      {
	totalSize += path.size();
      }
    return totalSize;
  }

  int IntersectionPaths::getFreePoint()
  {
    for (int i=0; i<intersectionPoints.size(); i++)
      {
	bool freePoint = true;
	for (Path p: paths)
	  {
	    for (int j=0; j<p.size(); j++)
	      {
		if (p[j] == i)
		  {
		    freePoint = false;
		  }
	      }
	  }
	if (freePoint)
	  {
	    return i;
	  }
      }
    return -1;
  }
  
  void IntersectionPaths::calculatePaths()
  {
    //calculateConnectedPoints();
    
    int intersectionPointIndex = 0;

    do {
      //std::cout << "Intersection point is " << intersectionPointIndex << std::endl;
      //std::cout << "Total path size is " << totalPathSize() << std::endl;
      //std::cout << "Points size is " << points.size() << std::endl;

      paths.push_back(calculatePath(intersectionPointIndex));

      int newIntersectionPointIndex = getFreePoint();
      if (intersectionPointIndex != newIntersectionPointIndex)
	  {
	    intersectionPointIndex = newIntersectionPointIndex;
	  }
      else
	{
	  return;
	}
    } while(totalPathSize() < intersectionPoints.size());
  }

  int IntersectionPaths::getPathStartIndex(Path path, int face)
  {
    int i = 0;

    //IntersectionPoint ip = intersectionPoints.getPoint(path[i]);

    while(intersectionPoints.getPoint(path[i]).faces.count(face))
      {
	if (++i >= path.size())
	  {
	    return 0;
	  }

	//ip = intersectionPoints.getPoint(path[i]);
      }
    return i;
  }

  // IntersectionFace IntersectionPaths::getFace(int face)
  // {
  //   IntersectionFace face(this->mesh, *this, face);

  //   return face;
  // }
}
