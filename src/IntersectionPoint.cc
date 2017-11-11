#include "IntersectionPoint.h"
#include "MeshIntersection.h"
#include "Calc.h"
#include <iostream>
#include "Log.h"
#include <algorithm>
#include "Face.h"
#include "Segment.h"

namespace Geotree
{
  IntersectionPoint::IntersectionPoint(Mesh &_mesh, PointInfo firstType, PointInfo secondType, Vector _point)
    : Point::Point(_mesh, _mesh.add(_point)),
      first(firstType), second(secondType)
  {
    getFaces();
  }

  bool IntersectionPoint::operator==(const IntersectionPoint &ipoint)
  {
    if (Calc::vectorEqual(ipoint.getPoint(), getPoint()))
      {
	if (ipoint.first == this->first && ipoint.second == this->second )
	  {
	    return true;
	  }
	else if (ipoint.first == this->second && ipoint.second == this->first)
	  {
	    return true;
	  }
	else
	  {
	    return false;
	  }
      }
    else
      {
	return false;
      }
  }

  void IntersectionPoint::getFaces()
  {
    first.setFaces(mesh);
    second.setFaces(mesh);
    FaceSet faces;
    faces = first.getFaces(mesh);
    this->faces.insert(first.faces.begin(), first.faces.end());
    faces = second.getFaces(mesh);
    this->faces.insert(second.faces.begin(), second.faces.end());
  }

  bool IntersectionPoint::isConnected(const IntersectionPoint point)
  {
    if ( (this->first.isConnected(point.first) && this->second.isConnected(point.second)) ||
	(this->first.isConnected(point.second) && this->second.isConnected(point.first)))
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  FaceSet IntersectionPoint::getIntersectedFaces() const
  {
    FaceSet intersectedFaces;

    if (first.type == FACE || first.type == SEGMENT)
      {
	intersectedFaces.insert(first.faces.begin(), first.faces.end());
      }
    
    if (second.type == FACE || second.type == SEGMENT)
      {
	intersectedFaces.insert(second.faces.begin(), second.faces.end());
      }

    return intersectedFaces;
  }

}
