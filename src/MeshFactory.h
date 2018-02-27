#pragma once
#include "Mesh.h"

namespace Geotree
{
    class MeshFactory
    {
    public:
        Mesh makeCube(const double width, const double height, const double depth);
        Mesh makeTetra(const double size);
        Mesh makeSphere(const double radius);
        Mesh makeCylinder(const double radius, const double height);
    };
}
