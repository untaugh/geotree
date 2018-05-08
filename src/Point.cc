#include "Point.h"

namespace Geotree
{
  std::ostream& operator<< (std::ostream& stream, const Info& info)
  {
    stream << "(";
    switch(info.type)
    {
      case FACE: stream << "FACE:"; break;
      case SEGMENT: stream << "SEGM:"; break;
      case POINT: stream << "POIN:"; break;
    }
    stream << info.index0 << ":";
    stream << info.index1;
    stream << ")";
    return stream;
  }

  std::ostream& operator<< (std::ostream& stream, const Point& point)
  {
        stream << "(";
        stream << point.vector.transpose() << "), ";
        stream << point.mesh0 << point.mesh1 << " ";
        stream << point.path << ":";
        stream << point.number << ":";
        stream << "F0[";
        for (int face : point.faces0)
        {
          stream << face << ",";
        }
        stream << "]F1[";
        for (int face : point.faces1)
        {
          stream << face << ",";
        }
        stream << "]";
    return stream;
  }

  void Point::flip()
  {
    Info tmp = mesh0;
    mesh0 = mesh1;
    mesh1 = tmp;
  }

  bool Point::connected(Point point)
  {
    for (int f : point.faces0)
    {
      if (this->faces0.find(f) != this->faces0.end())
      {
        for (int g : point.faces1)
        {
          if (this->faces1.find(g) != this->faces1.end())
          {
            //std::cout << "connected:" << *this << " and " << point << std::endl;
            return true;
          }
        }
      }
    }
    return false;
  }
}
