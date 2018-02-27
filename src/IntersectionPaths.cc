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

				while (i < path.size() && intersectionPoints.getPoint(path[pathIndex]).faces.count(face))
				{
					subpath.insert(path[pathIndex]);
					++i;
					pathIndex = (pathStart + i) % path.size();
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
    int intersectionPointIndex = 0;

    do {
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

    while(intersectionPoints.getPoint(path[i]).faces.count(face))
      {
	if (++i >= path.size())
	  {
	    return 0;
	  }

      }
    return i;
  }

    std::vector<PathX<FacePoint>> IntersectionPaths::getSubPaths2(int face) {
		std::vector <PathX<FacePoint>> subpaths;

		for (Path path : paths)
		{
            std::cout << "path:" << path << std::endl;
			int pathStart = getPathStartIndex(path, face);
			int i = 0;

			while(i < path.size())
			{
				PathX<FacePoint> subpath(intersectionPoints.mesh);

				int pathIndex = (pathStart + i) % path.size();

				while (i < path.size() && intersectionPoints.getPoint(path[pathIndex]).faces.count(face))
				{
					IntersectionPoint ip = intersectionPoints.getPoint(path[pathIndex]);
					int meshPointIndex = intersectionPoints.getPoint(path[pathIndex]).getIndex();

					FacePoint fp = ip.getFacePoint(face);

					subpath.addExisting(fp);
					++i;
					pathIndex = (pathStart + i) % path.size();
				}

				if (subpath.size() > 0)
				{
					subpaths.push_back(subpath);
				}
				else
				{
					i++;
				}
			}
		}

		return subpaths;
    }
}
