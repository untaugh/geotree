#include "IntersectionPoints.h"
#include <Eigen/Dense>
#include <iostream>
#include "Face.h"

namespace Geotree
{
  IntersectionPoints::IntersectionPoints(const Mesh &mesh0, const Mesh &mesh1)
  : mesh0(mesh0), mesh1(mesh1)
  {
    getPoints();
    //std::cout << *this << std::endl;
    sort();
  }

  void IntersectionPoints::dividedFaces(std::set <int> &div0, std::set <int> &div1) const
  {
    for (Point p : this->points)
    {
      div0.insert(p.faces0.begin(), p.faces0.end());
      div1.insert(p.faces1.begin(), p.faces1.end());
    }
  }

  std::vector <Path> IntersectionPoints::getPaths0(int face)
  {
    std::vector <Path> paths;

    std::set <int> pathnums;

    for (Point p : points)
    {
      if (p.faces0.find(face) != p.faces0.end())
      {
        pathnums.insert(p.path);
      }
    }

    for (int pathnum : pathnums)
    {
      Path path;

      for (Point p : points)
      {
        if (p.faces0.find(face) != p.faces0.end())
        {
          if (p.path == pathnum)
          {
            path.points.push_back(p);
          }
        }
      }

      paths.push_back(path);
    }

    return paths;
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

  void IntersectionPoints::getPoints()
  {
    std::vector <Segment> segments0;
    std::vector <Segment> segments1;

    mesh0.getSegments(segments0);
    mesh1.getSegments(segments1);

    for (int f=0; f<mesh0.F.rows(); f++)
    {
      for (Segment segment : segments1)
      {
        Vector3i facerow = mesh0.F.row(f);

        Face face(mesh0.V.row(facerow[0]), mesh0.V.row(facerow[1]), mesh0.V.row(facerow[2]), f, facerow);

        std::vector <Point> retpoints;

        face.intersect(segment, retpoints);

        for (Point new_point : retpoints)
        {
          if (!contains(new_point))
          {
            //std::cout << "Adding " << new_point << std::endl;
            points.push_back(new_point);
          }
        }
      }
    }

    for (int f=0; f<mesh1.F.rows(); f++)
    {
      for (Segment segment : segments0)
      {
        Vector3i facerow = mesh1.F.row(f);

        Face face(mesh1.V.row(facerow[0]), mesh1.V.row(facerow[1]), mesh1.V.row(facerow[2]), f, facerow);

        std::vector <Point> retpoints;
        face.intersect(segment, retpoints);

        for (Point new_point : retpoints)
        {
          if (!contains(new_point))
          {
            new_point.flip();
            //std::cout << "Adding2 " << new_point << std::endl;
            points.push_back(new_point);
          }
        }
      }
    }

    // set faces
    for (Point &point : points)
    {
      switch(point.mesh0.type)
      {
        case FACE: point.faces0.insert(point.mesh0.index0); break;
        case SEGMENT:
        {
          for (Segment segment : segments0)
          {
            if ((segment.point0 == point.mesh0.index0 && segment.point1 == point.mesh0.index1)
            || (segment.point0 == point.mesh0.index1 && segment.point1 == point.mesh0.index0))
            {
              point.faces0.insert(segment.face0);
              point.faces0.insert(segment.face1);
              break;
            }
          }
          break;
        }
        case POINT:
        {
          for (int f=0; f<mesh0.F.rows(); f++)
          {
            if (mesh0.F.row(f)[0] == point.mesh0.index0
            || mesh0.F.row(f)[1] == point.mesh0.index0
            || mesh0.F.row(f)[2] == point.mesh0.index0)
            {
              point.faces0.insert(f);
            }
          }
          break;
        }
      }

      switch(point.mesh1.type)
      {
        case FACE: point.faces1.insert(point.mesh1.index0); break;
        case SEGMENT:
        {
          for (Segment segment : segments1)
          {
            if ((segment.point0 == point.mesh1.index0 && segment.point1 == point.mesh1.index1)
            || (segment.point0 == point.mesh1.index1 && segment.point1 == point.mesh1.index0))
            {
              point.faces1.insert(segment.face0);
              point.faces1.insert(segment.face1);
              break;
            }
          }
          break;
        }
        case POINT:
        {
          for (int f=0; f<mesh1.F.rows(); f++)
          {
            if (mesh1.F.row(f)[0] == point.mesh0.index0
            || mesh1.F.row(f)[1] == point.mesh0.index0
            || mesh1.F.row(f)[2] == point.mesh0.index0)
            {
              point.faces1.insert(f);
            }
          }
          break;
        }
      }
    }
  }

  bool IntersectionPoints::firstPoint(Point ** point)
  {
    for (Point &p : points)
    {
      if (p.number == -1)
      {
        *point = &p;
        return true;
      }
    }

    return false;
  }

  int IntersectionPoints::connectedPoints(Point point, Point ** point_ret)
  {
    int n = 0;

    for(Point &point_find : points)
    {
      if (point_find != point)
      {
        if (point_find.connected(point))
        {
          point_ret[n++] =  &point_find;
        }
      }
    }

    return n;
  }

  void IntersectionPoints::nextPoint(Point ** previous, Point ** point)
  {
    Point *point_find[3];

    int n = connectedPoints(**point, point_find);

    // if (n == 3)
    // {
    //   std::cout << "three points found" << std::endl;
    // }

    for (int i=0; i<n; i++)
    {
      Point *pp = point_find[i];

      // If connected to more than 2,
      // check that the point isn't on same face as point and previous.
      if (*pp != **previous && (n < 3 || !pp->connected(**previous)))
      {
        *previous = *point;
        *point = pp;
        return;
      }
    }

    std::cout << "point not found:" << **point << std::endl;
    std::cout << "previous was:" << **previous << std::endl;

    exit(1);
  }

  int IntersectionPoints::numPaths()
  {
    int path = -1;

    for (Point &p : points)
    {
      path = p.path > path ? p.path : path;
    }
    return path + 1;
  }

  void IntersectionPoints::sort()
  {
    Point *first, *current, *previous;

    // Special case one and two points
    if (points.size() <= 2)
    {
      points.begin()->number = 0;
      points.begin()->path = 0;

      if (points.size() == 2)
      {
        if (points[0].connected(points[1]))
        {
          points.end()->number = 1;
          points.end()->path = 0;
        }
        else
        {
          points.end()->number = 0;
          points.end()->path = 1;
        }
      }
      return;
    }

    unsigned int pointcount = 0;

    for (Point point : points)
    {
      if (point.mesh0.type == POINT && point.mesh1.type == POINT)
      {
        pointcount++;
      }
    }

    // Special case all points, equal meshes
    if (pointcount == points.size())
    {
      return;
    }

    int path = 0;

    while (firstPoint(&first))
    {
      int n = 0;

      current = first;
      previous = first;

      do {
        current->number = n++;
        current->path = path;
        nextPoint(&previous, &current);
      } while(*current != *first);

      path++;
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
