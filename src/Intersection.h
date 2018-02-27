#pragma once
#include "Mesh.h"

namespace Geotree
{
    class Intersection
    {
    public:
        Intersection(const Mesh _mesh1, const Mesh _mesh2);
        uint32_t pathcount(void);
    private:
        const Mesh mesh1;
        const Mesh mesh2;
    };
}
