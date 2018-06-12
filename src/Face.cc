#include "Log.h"
#include "Face.h"
#include <Eigen/Dense>
#include "EdgePoint.h"
#include "Face2D.h"

namespace Geotree
{
  Face::Face(const Vector3d v0, const Vector3d v1, const Vector3d v2, int _index)
    : index(_index), point0(v0), point1(v1), point2(v2)
  {
    getNormal();
  }

  bool Face::intersect(const Segment segment, std::vector <Point> &points) const
  {
    Vector3d U = segment.end - segment.begin;
    Vector3d W = segment.begin - point0;

    double D = normal.dot(U);
    double N = normal.dot(W);

    if (D == 0) // segment parallell to plane
    {
      return false;
    }

    double sI = -N/D;

    if (sI < 0.0 || sI > 1.0) // segment outside plane
    {
      return false;
    }

    Vector3d vector = segment.begin - N/D * U;

    if (!isInside(vector))
    {
      return false;
    }

    Point point(vector);
    point.facesMesh0.insert(this->index);
    point.facesMesh1.insert(segment.face0);
    point.facesMesh1.insert(segment.face1);
    point.face0edge = segment.face0edge;
    point.face1edge = segment.face1edge;
    points.push_back(point);

    return true;
  }

  void Face::getNormal()
  {
    normal = (point1 - point0).cross(point2 - point0).normalized();
  }

  bool Face::isInside(const Vector3d point) const
  {
    double dot = normal.dot(point - point0);

    // is point on face plane
    if (dot > 1.0e-14 && dot < -1.0e-14)
    {
      return false;
    }

    Vector3d e1 = point1 - point0;
    Vector3d e2 = point2 - point1;
    Vector3d e3 = point0 - point2;

    Vector3d c1 = point - point0;
    Vector3d c2 = point - point1;
    Vector3d c3 = point - point2;

    double r1 = normal.dot(e1.cross(c1));
    double r2 = normal.dot(e2.cross(c2));
    double r3 = normal.dot(e3.cross(c3));

    if (r1 < 0.0 || r2 < 0.0 || r3 < 0.0)
    {
      return false;
    }

    return true;
  }

  std::vector<Segment> Face::getSegments()
  {
    std::vector<Segment> segments;
    segments.push_back(Segment(point0, point1, 0));
    segments.push_back(Segment(point1, point2, 1));
    segments.push_back(Segment(point2, point0, 2));
    return segments;
  }

  bool Face::connected(const Vector3d point) const
  {
    if (this->point0 == point
	|| this->point1 == point
	|| this->point2 == point)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool Face::connected(const Face face) const
  {
    if (connected(face.point0)) return true;
    else if (connected(face.point1)) return true;
    else if (connected(face.point2)) return true;
    else return false;
  }

  double Face::lengthside(Axis axis)
  {
    double min = point0[axis];
    double max = point0[axis];

    if (point1[axis] < min) min = point1[axis];
    if (point1[axis] > max) max = point1[axis];

    if (point2[axis] < min) min = point2[axis];
    if (point2[axis] > max) max = point2[axis];

    return max - min;
  }

  Axis Face::shortestside()
  {
    double dX = lengthside(X);
    double dY = lengthside(Y);
    double dZ = lengthside(Z);

    if (dX < dY)
    {
      if (dX < dZ)
      {
	return X;
      }
      else
      {
	return Z;
      }
    }
    else if (dY < dZ)
    {
      return Y;
    }
    else
    {
      return Z;
    }
  }
  
  
  std::vector <Matrix<int, Dynamic, 3>> Face::split(std::vector <std::set<Point>> paths)
  {
    std::vector <Matrix<int, Dynamic, 3>> groups;

    std::vector <EdgePoint> edgepoints;

    for (std::set<Point> path : paths)
    {
      EdgePoint ep0(*path.begin());
      EdgePoint ep1(*path.end());
      
      edgepoints.push_back(ep0);
      edgepoints.push_back(ep1);
    }

    Face2D face2d(*this);

    Log().debug() << face2d.dropaxis;
    
    // sort edge points
    (void) &paths;
    return groups;
  }
}
