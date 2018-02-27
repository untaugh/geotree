#include "MeshFactory.h"

namespace Geotree
{
    Mesh MeshFactory::makeCube(const double width, const double height, const double depth)
    {
        Verticies V(8,3);
        Faces F(12,3);

        if (width <= 0.0 || height <= 0.0 || depth <= 0.0)
        {
            //return cube;
        }

        V << 0.0, 0.0, 0.0,
                0.0, height, 0.0,
                width, height, 0.0,
                width, 0.0, 0.0,
                0.0, 0.0, depth,
                0.0, height, depth,
                width, height, depth,
                width, 0.0, depth;

        F << 0,1,2,
                0,2,3,
                0,1,4,
                1,4,5,
                1,2,5,
                2,5,6,
                2,3,6,
                3,6,7,
                3,0,7,
                0,7,4,
                4,5,6,
                4,6,7;

        return Mesh(V,F);
    }

    Mesh MeshFactory::makeTetra(const double size)
    {
        Verticies V(4,3);
        Faces F(4,3);

        V << 0,0,0, size,0,0, 0,size,0, 0,0,size;

        F << 0,1,2, 0,2,3, 0,1,3, 1,2,3;

        return Mesh(V,F);
    }

    Mesh MeshFactory::makeSphere(const double radius)
    {
        Verticies V(0,3);
        Faces F(0,3);
        return Mesh(V,F);
    }

    Mesh MeshFactory::makeCylinder(const double radius, const double height)
    {
        Verticies V(8,3);
        Faces F(0,3);
        return Mesh(V,F);

    }
}
