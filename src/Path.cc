#include "Path.h"
#include <vector>
#include <iostream>

namespace Geotree
{
  void PathT::add(const Vector3d point)
  {
    mesh.V.conservativeResize(mesh.V.rows() + 1, NoChange);
    points.push_back(mesh.V.rows() - 1);
    mesh.V.row(mesh.V.rows() - 1) = point;
  }

    Point PathT::begin() const
    {
        return Point(mesh, points.front());
    }

    Point PathT::back() const
    {
        return Point(mesh, points.back());
    }

    int PathT::size() const
  {
    return points.size();
  }

  Vector3d PathT::get(int index)
  {
    return mesh.V.row(points[index]);
  }
    bool PathT::operator != (const PathT &path) const
    {
        return !(operator==(path));
    }

    bool PathT::operator == (const PathT &path) const
    {
        if (this->size() != path.size())
        {
            return false;
        }

        if (this->points == path.points)
        {
            return true;
        }

        bool pathsEqual = true;

        for (int i=0; i<path.size(); i++)
        {
            if (this->points.front() == path.points[i])
            {
                for (int j=0; j<this->size(); j++)
                {
                    if (this->points[j] != path.points[i++])
                    {
                        pathsEqual = false;
                    }

                    if (i >= path.size())
                    {
                        i = 0;
                    }
                }
                break;
            }
        }

        if (!pathsEqual) {
            pathsEqual = true;
            for (int i = 0; i < path.size(); i++) {
                if (this->points.back() == path.points[i]) {
                    for (int j = (this->points.size() - 1); j > 0; j--) {
                        if (this->points[j] != path.points[i++]) {
                            pathsEqual = false;
                        }

                        if (i >= path.size()) {
                            i = 0;
                        }
                    }
                    break;
                }
            }
        }

        return pathsEqual;
    }

    std::ostream& operator<< (std::ostream& stream, const PathT& path)
  {
    stream << "Path:";

    for (int point : path.getPoints())
      {
	stream << point << ",";
       }

    return stream;
  }
}
