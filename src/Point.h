#pragma once
#include <Eigen/Core>
#include <set>
#include <iostream>

using namespace Eigen;

namespace Geotree
{
    enum PointType
    {
      POINT,
      SEGMENT,
      FACE
    };

    struct Info
    {
      PointType type;
      int index0;
      int index1;
    };

    class Point
    {
    public:
      Point(Vector3d vector) : vector(vector) {};
      bool connected(Point point);
      void flip();

      const Vector3d vector;
      Info mesh0;
      Info mesh1;
      int path = -1;
      int number = -1;
      std::set <int> faces0;
      std::set <int> faces1;
      const double NEAR_ZERO = 1.0e-14;

      bool operator !=(Point &point)
      {
        return !(point.vector == this->vector);
      }

      bool operator == (Point &point)
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
    };

    std::ostream& operator<< (std::ostream& stream, const Info& info);
    std::ostream& operator<< (std::ostream& stream, const Point& point);
}
