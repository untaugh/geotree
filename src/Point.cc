#include "Point.h"

namespace Geotree
{
  std::ostream& operator<< (std::ostream& stream, const PointType& type)
  {
    stream << "(";
    switch(type)
    {
      case FACE: stream << "FACE:"; break;
      case SEGMENT: stream << "SEGM:"; break;
      case POINT: stream << "POIN:"; break;
    }
    stream << ")";
    return stream;
  }

  std::ostream& operator<< (std::ostream& stream, const Point& point)
  {
        stream << "(";
        stream << point.vector.transpose() << "), ";
        stream << point.path << ":";
        stream << point.number << ":";
        stream << "F0[";
        for (int face : point.facesMesh0)
        {
          stream << face << ",";
        }
        stream << "]F1[";
        for (int face : point.facesMesh1)
        {
          stream << face << ",";
        }
        stream << "]";
    return stream;
  }

  void Point::flip()
  {
    PointType typeTmp = this->typeMesh0;
    this->typeMesh0 = this->typeMesh1;
    this->typeMesh1 = typeTmp;

    std::set <int> facesTmp = this->facesMesh0;
    this->facesMesh0 = this->facesMesh1;
    this->facesMesh1 = facesTmp;
  }

  std::set <int> &Point::getFaces(MeshID mesh)
  {
    switch (mesh)
      {
      case MESH0: return this->facesMesh0;
      case MESH1: return this->facesMesh1;
      }

    throw;
  }

  bool Point::hasFace(MeshID mesh, int face)
  {
    std::set <int> &faces = getFaces(mesh);

    if (faces.find(face) != faces.end())
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  bool Point::connected(MeshID mesh, Point point)
  {
    for (int face : this->getFaces(mesh))
      {
	if (point.hasFace(mesh, face))
	  {
	    return true;
	  }
      }
    return false;
  }

  bool Point::connected(Point point)
  {
    if (this->connected(MESH0, point) && this->connected(MESH1, point))
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  bool Point::operator !=(Point &point)
  {
    return !(point.vector == this->vector);
  }

  bool Point::operator == (Point &point)
    {
      for (int i=0; i<3; i++)
        {
          double diff = point.vector[i] - this->vector[i];

          if (diff > NEAR_ZERO || diff < -NEAR_ZERO)
	    {
	      return false;
	    }
        }
      return true;
    }
}
