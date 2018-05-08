#pragma once
#include "Mesh.h"

namespace Geotree
{
    class MeshFactory
    {
    public:
        Mesh cube(const double width, const double height, const double depth);
        Mesh tetra(const double size);
        Mesh sphere(const double radius);
        Mesh cylinder(const double radius, const double height);
    };
}
