#include "Face.h"
#include <Eigen/Dense>

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
    point.typeMesh0 = FACE;
    point.facesMesh0.insert(this->index);
    point.typeMesh1 = SEGMENT;
    point.facesMesh1.insert(segment.face0);
    point.facesMesh1.insert(segment.face1);
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
}
