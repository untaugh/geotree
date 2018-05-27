#include "IntersectionPoints.h"
#include <Eigen/Dense>
#include <iostream>
#include "Face.h"
#include "Log.h"

namespace Geotree
{
  IntersectionPoints::IntersectionPoints(const Mesh &mesh0, const Mesh &mesh1)
  : mesh0(mesh0), mesh1(mesh1)
  {
    findPoints();
  }

  bool IntersectionPoints::contains(Point &point)
  {
    for (Point p : points)
    {
      if (p == point)
      {
        return true;
      }
    }
    return false;
  }

  const Mesh &IntersectionPoints::getMesh(MeshID meshID)
  {
    switch (meshID)
      {
      case MESH0: return this->mesh0;
      case MESH1: return this->mesh1;
      }
    throw;
  }

  const Mesh &IntersectionPoints::getOtherMesh(MeshID meshID)
  {
    switch (meshID)
      {
      case MESH1: return this->mesh0;
      case MESH0: return this->mesh1;
      }
    throw;
  }

  void IntersectionPoints::getIntersectionPoints(MeshID meshID, std::vector <Point> &points)
  {
    std::vector <Segment> segments;
    std::vector <Face> faces;

    const Mesh &mesh = getMesh(meshID);
    const Mesh &othermesh = getOtherMesh(meshID);

    mesh.getFaces(faces);
    othermesh.getSegments(segments);

    for (Face face : faces)
      {
	for (Segment segment : segments)
	  {
	    face.intersect(segment, points);
	  }
      }    
  }

  void IntersectionPoints::add(Point point)
  {

    if (this->contains(point))
      {
	for (Point &p : points)
	  {
	    if (p == point)
	      {
		p.facesMesh0.insert(point.facesMesh0.begin(), point.facesMesh0.end());
		p.facesMesh1.insert(point.facesMesh1.begin(), point.facesMesh1.end());
		Log().debug() << point << " MODIFY";
	      }
	  }
      }
      else
      {
	Log().debug() << point << " INSERT";
	this->points.push_back(point);
      }
  }
  
  void IntersectionPoints::findPoints()
  {
    std::vector <Point> points0;
    std::vector <Point> points1;
    
    getIntersectionPoints(MESH0, points0);
    getIntersectionPoints(MESH1, points1);

    for (Point &point : points0)
      {
	this->add(point);
      }

    for (Point &point : points1)
      {
	point.flip();
	this->add(point);
      }
  }

  std::ostream& operator<< (std::ostream& stream, const IntersectionPoints& points)
  {
    for (Point p : points.points)
    {
      stream << p << std::endl;
    }
    return stream;
  }
}
