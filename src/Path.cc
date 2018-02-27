#include "Path.h"
//#include "IntersectionFace.h"
#include <vector>
#include <iostream>

namespace Geotree
{

    template<>
    bool PathX<FacePoint>::edgeToEdge() const
    {
        return this->begin().type == SEGMENT;
    }

//    template<>
//    int PathT::size() const
//    {
//        return points.size();
//    }

    template<>
  Vector3d PathT::get(int index)
  {
    //return mesh.V.row(points[index].getIndex());
  }

    template<>
    bool PathT::operator != (const PathT &path) const
    {
        return !(operator==(path));
    }

    template<typename T>
    bool PathX<T>::operator == (const PathX<T> &path) const
    {
        if (this->size() != path.size())
        {
            return false;
        }

        if (this->points == path.points)
        {
            return true;
        }

        bool pathsEqual = false;

        for (int i=0; i<path.size(); i++)
        {
            if (this->points.front() == path.points[i])
            {
                pathsEqual = true;

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
            for (int i = 0; i < path.size(); i++) {
                if (this->points.back() == path.points[i]) {
                    pathsEqual = true;
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

}
