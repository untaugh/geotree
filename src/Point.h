#pragma once
//#include "Mesh.h"
#include "Types.h"
namespace Geotree
{
    class Mesh;

    class Point
    {
    public:
        Point(const Mesh& _mesh, const int _index) : index(_index), mesh(_mesh) {};
        Vector getVector() const;
        int getIndex() const { return index; };
        double distance(const Point& point) const;
        bool operator!=(const Point& point) const;
        bool operator == (const Point & point) const { return this->index == point.index; };

    protected:
        int index;
        const Mesh &mesh;
    };


}
