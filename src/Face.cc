#include "Face.h"
#include <Eigen/Dense>

namespace Geotree
{
  Face::Face(Vector3d v0, Vector3d v1, Vector3d v2, int _index, Vector3i _face)
  : index(_index), face(_face), point0(v0), point1(v1), point2(v2)
  {
    getNormal();
  }

  void Face::intersect(const Segment segment, std::vector <Point> &points)
  {
    //std::cout << "segment[" << segment.begin.transpose() << " " <<segment.end.transpose();
    //std::cout << "] face[" << point0.transpose() << ", " << point1.transpose() << ", " << point2.transpose() << "]"<< std::endl;

    if (segment.begin == point0)
    {
      Point point(segment.begin);
      point.mesh0.type = POINT;
      point.mesh0.index0 = face[0];
      point.mesh1.type = POINT;
      point.mesh1.index0 = segment.point0;
      points.push_back(point);
    }
    else if (segment.begin == point1)
    {
      Point point(segment.begin);
      point.mesh0.type = POINT;
      point.mesh0.index0 = face[1];
      point.mesh1.type = POINT;
      point.mesh1.index0 = segment.point0;
      points.push_back(point);
    }
    else if (segment.begin == point2)
    {
      Point point(segment.begin);
      point.mesh0.type = POINT;
      point.mesh0.index0 = face[2];
      point.mesh1.type = POINT;
      point.mesh1.index0 = segment.point0;
      points.push_back(point);
    }
    else if (segment.end == point0)
    {
      Point point(segment.end);
      point.mesh0.type = POINT;
      point.mesh0.index0 = face[0];
      point.mesh1.type = POINT;
      point.mesh1.index0 = segment.point1;
      points.push_back(point);
    }
    else if (segment.end == point1)
    {
      Point point(segment.end);
      point.mesh0.type = POINT;
      point.mesh0.index0 = face[1];
      point.mesh1.type = POINT;
      point.mesh1.index0 = segment.point1;
      points.push_back(point);
    }
    else if (segment.end == point2)
    {
      Point point(segment.end);
      point.mesh0.type = POINT;
      point.mesh0.index0 = face[2];
      point.mesh1.type = POINT;
      point.mesh1.index0 = segment.point1;
      points.push_back(point);
    }
    else if (intersectsPlanar(segment, points))
    {
    }
    else if (intersects(segment, points))
    {
    }
  }

  Segment Face::getSegment(int n)
  {
    switch(n)
    {
      case 0: return Segment(point0, point1, face[0], face[1]);
      case 1: return Segment(point1, point2, face[1], face[2]);
      case 2: return Segment(point2, point0, face[2], face[0]);
      default: return Segment(point0, point1);
    }
  }

  bool Face::intersectsPlanar(Segment segment, std::vector <Point> &points)
  {
    if (!isPlanar(segment))
    {
      return false;
    }
    //std::cout << "planar" << std::endl;

    if (isInside(segment.begin))
    {
      Point point(segment.begin);
      point.mesh0.type = FACE;
      point.mesh0.index0 = index;
      point.mesh1.type = POINT;
      point.mesh1.index0 = segment.point0;
      points.push_back(point);

      if (isInside(segment.end))
      {
        Point point(segment.end);
        point.mesh0.type = FACE;
        point.mesh0.index0 = index;
        point.mesh1.type = POINT;
        point.mesh1.index0 = segment.point1;
        points.push_back(point);
        return true;
      }
      else
      {
        for (int i=0; i<3; i++)
        {
          Segment facesegment = getSegment(i);
          Vector3d vector;
          if (facesegment.intersects(segment, vector))
          {
            Point point(vector);
            point.mesh0.type = SEGMENT;
            point.mesh0.index0 = facesegment.point0;
            point.mesh0.index1 = facesegment.point1;
            point.mesh1.type = SEGMENT;
            point.mesh1.index0 = segment.point0;
            point.mesh1.index1 = segment.point1;
            points.push_back(point);
            return true;
          }
        }
      }
    }
    else if (isInside(segment.end))
    {
      Point point(segment.end);
      point.mesh0.type = FACE;
      point.mesh0.index0 = index;
      point.mesh1.type = POINT;
      point.mesh1.index0 = segment.point1;
      points.push_back(point);

      for (int i=0; i<3; i++)
      {
        Segment facesegment = getSegment(i);
        Vector3d vector;
        if (facesegment.intersects(segment, vector))
        {
          Point point(vector);
          point.mesh0.type = SEGMENT;
          point.mesh0.index0 = facesegment.point0;
          point.mesh0.index1 = facesegment.point1;
          point.mesh1.type = SEGMENT;
          point.mesh1.index0 = segment.point0;
          point.mesh1.index1 = segment.point1;
          points.push_back(point);
          return true;
        }
      }
    }

    return false;
  }

  bool Face::intersects(Segment segment, std::vector <Point> &points)
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

    for (int i=0; i<3; i++)
    {
      Segment facesegment = getSegment(i);

      if (facesegment.intersects(vector))
      {
        Point point(vector);
        point.mesh0.type = SEGMENT;
        point.mesh0.index0 = facesegment.point0;
        point.mesh0.index1 = facesegment.point1;
        point.mesh1.type = SEGMENT;
        point.mesh1.index0 = segment.point0;
        point.mesh1.index1 = segment.point1;
        points.push_back(point);

        return true;
      }
    }

    Point point(vector);
    point.mesh0.type = FACE;
    point.mesh0.index0 = index;
    point.mesh1.type = SEGMENT;
    point.mesh1.index0 = segment.point0;
    point.mesh1.index1 = segment.point1;
    points.push_back(point);

    return true;
  }

  void Face::getNormal()
  {
    normal = (point1 - point0).cross(point2 - point0).normalized();
  }

  bool Face::isPlanar(Segment segment)
  {
    double dot;

    dot = normal.dot(segment.begin - point0);

    // is point on face plane
    if (dot < 1.0e-14 && dot > -1.0e-14)
    {
      dot = normal.dot(segment.end - point0);

      // is point on face plane
      if (dot < 1.0e-14 && dot > -1.0e-14)
      {
        return true;
      }
    }
    return false;
  }

  bool Face::isInside(Vector3d point)
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
