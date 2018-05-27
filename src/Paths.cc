#include "Paths.h"

namespace Geotree
{
  Paths::Paths(IntersectionPoints &intersection)
    : intersection(intersection)
  {
    findPaths();
  }
  
  unsigned int Paths::count()
  {
    int pathindex = 0;
    
    for (Point &point : this->intersection.points)
      {
	pathindex = point.path > pathindex ? point.path : pathindex;
      }
    
    int pathcount = pathindex + 1;
    return pathcount;
  }

  unsigned int Paths::size(int i)
  {
    unsigned int size = 0;

    for (Point &point : this->intersection.points)
      {
	
	if (i == point.path)
	  {
	    size++;
	  }
      }
    return size;
  }
  
  Point &Paths::nextPoint(Point &current, Point &previous)
  {    
    for (Point &findpoint : intersection.points)
      {
	if (findpoint != previous && findpoint != current && current.connected(findpoint))
	  {
	    return findpoint;
	  }
      }
    return current;
  }

  void Paths::findPaths()
  {
    unsigned int pathcount = 0;
    unsigned int pointcount = 0;
    
    for (Point &point : intersection.points)
      {
	if (point.path < 0)
	  {
	    Point *current = &point;
	    Point *previous = &point;

	      do {
		Point &newpoint = nextPoint(*current, *previous);
		current->number = pointcount++;
		current->path = pathcount;
		std::cout << *current << std::endl;
		previous = current;
		current = &newpoint;

		if (pointcount > intersection.points.size())
		  {
		    throw;
		  }
	      } while (*current != point);

	      pathcount++;
	      pointcount = 0;
	  }
      }
  }

  std::set<int> Paths::faces(MeshID mesh)
  {
    std::set<int> faces;
    for (Point point : this->intersection.points)
      {
	std::set<int> newfaces = point.getFaces(mesh);
	faces.insert(newfaces.begin(), newfaces.end());	
      }
    return faces;
  }
}
