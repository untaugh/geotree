#include "Point.h"
#include "Zero.h"

namespace Geotree
{
  std::ostream& operator<< (std::ostream& stream, const Point& point)
  {
        stream << "[(";
        stream << point.vector.transpose() << ");";
        stream << point.path << ";";
        stream << point.number << ";";
        for (int face : point.facesMesh0)
        {
          stream << face << ",";
        }
        stream << ";";
        for (int face : point.facesMesh1)
        {
          stream << face << ",";
        }
        stream << "]";
    return stream;
  }

  void Point::flip()
  {
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

  bool Point::inside(const Cube box) const
  {
    for (int axis=0; axis<3; axis++)
      {
	if (this->vector[axis] < box.p0[axis]
	    || this->vector[axis] > box.p1[axis])
	  {
	    return false;
	  }
      }
    return true;
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

  bool Point::operator < (const Point &point) const
    {
      if (this->path < point.path) return true;
      else if (this->path > point.path) return false;
      else if (this->number < point.number) return true;
      else return false;
    }

}
