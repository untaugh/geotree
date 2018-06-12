#include "Log.h"
#include "Face2D.h"
#include <algorithm>

namespace Geotree
{

  Face2D::Face2D(Face face)
    : dropaxis(face.shortestside())
  {
    Axis axis0;
    Axis axis1;

    if (dropaxis == X)
    {
      axis0 = Y;
      axis1 = Z;
    }

    if (dropaxis == Y)
    {
      axis0 = X;
      axis1 = Z;
    }

    if (dropaxis == Z)
    {
      axis0 = X;
      axis1 = Y;
    }

    p0.x = face.point0[axis0];
    p0.y = face.point0[axis1];
    p1.x = face.point1[axis0];
    p1.y = face.point1[axis1];
    p2.x = face.point2[axis0];
    p2.y = face.point2[axis1];
  }

  void Face2D::findpath(EdgePoint2D edgepoint, std::vector<Path2D> paths, int &path, bool &reverse)
  {
    for (uint32_t i=0; i<paths.size(); i++)
    {
      if (*paths[i].begin.point == *edgepoint.point)
      {
	path = i;
	reverse = false;
	break;
      }

      if (*paths[i].end.point == *edgepoint.point)
      {
	path = i;
	reverse = true;
	break;
      }	  
    }
  }

  /* If first loop contains second */
  // bool Face2D::contains(Loop2D loopA, Loop2D loopB)
  // {
    
  // }
  
  void Face2D::createloops(int startpoint, std::vector <EdgePoint2D> edgepoints, std::vector<Path2D> paths, std::vector<Loop2D> &loops)
  {
    uint32_t currentpoint = startpoint + 1;

    Loop2D loop;

    loop.points.push_back(*edgepoints[startpoint].point);
    std::cout << "new path " << startpoint << ":" << startpoint << std::endl;
    
    while(edgepoints[startpoint].point != edgepoints[currentpoint].point)
    {
      std::cout << startpoint << ":" << currentpoint << std::endl;
      loop.points.push_back(*edgepoints[currentpoint].point);
	    
      if (edgepoints[currentpoint].distance > NEAR_ZERO)
      {
	int path = -1;
	bool reverse;

	findpath(edgepoints[currentpoint], paths, path, reverse);

	/* add path points to loop */
	if (reverse)
	{
	  for (int i=paths[path].points.size() - 2; i > 0; i--)
	  {
	    std::cout << "p0: " << std::endl;
	    loop.points.push_back(paths[path].points[i]);
	  }
	}
	else
	{
	  for (uint32_t i=1; i < paths[path].points.size() - 1; i++)
	  {
	    std::cout << "p1: " << std::endl;
	    loop.points.push_back(paths[path].points[i]);
	  }
	}

	/* find edgepoint at the end of path */
	int nextpoint = 0;

	if (reverse)
	{
	  while (*edgepoints[nextpoint].point != *paths[path].begin.point)
	  {
	    nextpoint++;
	  }
	}
	else
	{
	  while (*edgepoints[nextpoint].point != *paths[path].end.point)
	  {
	    nextpoint++;
	  }
	}

	/* if path ends with this startpoint */
	if (*edgepoints[nextpoint].point == *edgepoints[startpoint].point)
	{
	  break;
	}
	else
	{
	  std::cout << "e: " << nextpoint << std::endl;
	  loop.points.push_back(*edgepoints[nextpoint].point);
	}

	createloops(currentpoint, edgepoints, paths, loops);

	nextpoint++;

	currentpoint = nextpoint;
      }
      else
      {
	currentpoint++;
      }

      if (currentpoint >= edgepoints.size())
      {
	currentpoint = 0;
      }
    }

    std::cout << "path created " << loops.size() << std::endl;

    loops.push_back(loop);
  }

  std::vector<uint32_t> Face2D::triangulate(Loop2D loop, std::vector<Loop2D> &holes)
  {
    std::vector<std::vector<Point2D>> polygons;

    polygons.push_back(loop.points);

    if (holes.size() > 0)
    {
      polygons.push_back(holes[0].points);
    }

    return mapbox::earcut<uint32_t>(polygons);
  }

  std::vector<uint32_t> Face2D::triangulate(Loop2D loop)
  {
    std::vector<Loop2D> holes;

    return triangulate(loop, holes);
  }

  void Face2D::createholes(std::vector<Path2D> paths, std::vector<Loop2D> &loops)
  {
    for (Path2D path : paths)
    {
      if (!path.edgetoedge)
      {
	Loop2D loop;

	loop.points = path.points;

	loops.push_back(loop);
      }
    }
  }

  std::vector <EdgePoint2D> Face2D::getEdgepoints(std::vector<Path2D> paths)
  {
    std::vector <EdgePoint2D> edgepoints;

    /* insert points of face to edge points */
    EdgePoint2D e0 = {0, 0.0, &p0};
    EdgePoint2D e1 = {1, 0.0, &p1};
    EdgePoint2D e2 = {2, 0.0, &p2};

    edgepoints.push_back(e0);
    edgepoints.push_back(e1);
    edgepoints.push_back(e2);

    /* insert points of paths to edge poins */
    for (Path2D path : paths)
    {
      if (path.edgetoedge)
      {
	edgepoints.push_back(path.begin);
	edgepoints.push_back(path.end);
      }
    }

    std::sort(edgepoints.begin(), edgepoints.end());

    Log().debug() << "edgepoints: ";
    for (EdgePoint2D ep : edgepoints)
    {
      Log().debug() << ep;
    }

    return edgepoints;
  }

  std::vector <std::vector<uint32_t>> Face2D::split(std::vector<Path2D> paths)
  {
    std::vector <std::vector<uint32_t>> groups;

    std::vector <EdgePoint2D> edgepoints = getEdgepoints(paths);

    std::vector<Loop2D> loops;

    std::vector<Loop2D> holes;

    createloops(0, edgepoints, paths, loops);

    createholes(paths, holes);

    std::cout << "holes:" << holes.size() << std::endl;

    for (Loop2D loop : holes)
    {
      std::vector<uint32_t> faces = triangulate(loop);

      for (uint32_t &face : faces)
      {
	face = getIndex(loop.points[face], paths);
      }

      groups.push_back(faces);
    }

    for (Loop2D loop : loops)
    {
      std::vector<uint32_t> faces = triangulate(loop, holes);

      if (holes.size() > 0)
      {
	loop.points.insert(loop.points.end(), holes[0].points.begin(), holes[0].points.end());
      }

      for (uint32_t &face : faces)
      {
	face = getIndex(loop.points[face], paths);
      }

      groups.push_back(faces);      
    }

    return groups;
  }

  uint32_t Face2D::getIndex(Point2D point, std::vector<Path2D> paths)
  {
    if (point == p0) return 0;
    if (point == p1) return 1;
    if (point == p2) return 2;

    const uint32_t faceoffset = 3; 

    uint32_t pathoffset = 0;
    
    for (Path2D path : paths)
    {
      for (uint32_t i=0; i<path.points.size(); i++)
      {
	if (point == path.points[i])
	{
	  uint32_t index = pathoffset + faceoffset + i;
		  
	  return index;
	}
      }
      pathoffset += path.points.size(); 
    }

    throw;
  } 
}
