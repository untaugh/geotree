#include "MeshFactory.h"

namespace Geotree
{
    Mesh MeshFactory::cube(const double width, const double height, const double depth)
    {
        Matrix<double, Dynamic, 3> V(8,3);
        Matrix<int, Dynamic, 3> F(12,3);

        if (width <= 0.0 || height <= 0.0 || depth <= 0.0)
        {
            //return cube;
        }

        V << 0.0, 0.0, 0.0,
	  width, 0.0, 0.0,	  
	  0.0, height, 0.0,
	  width, height, 0.0,
	  0.0, 0.0, depth,
	  width, 0.0, depth,
	  0.0, height, depth,
	  width, height, depth;

        F << 2,1,0,
	  1,2,3,
	  4,5,6,
	  7,6,5,
	  0,1,4,
	  5,4,1,
	  6,3,2,
	  3,6,7,
	  4,2,0,
	  2,4,6,
	  1,3,5,
	  7,5,3;

        return Mesh(V,F);
    }

    Mesh MeshFactory::tetra(const double size)
    {
      Matrix<double, Dynamic, 3> V(4,3);
      Matrix<int, Dynamic, 3> F(4,3);

        V << 0,0,0, size,0,0, 0,size,0, 0,0,size;

        F << 0,1,2, 0,2,3, 0,1,3, 1,2,3;

        return Mesh(V,F);
    }

    Mesh MeshFactory::sphere(const double radius)
    {
      Matrix<double, Dynamic, 3> V(0,3);
      Matrix<int, Dynamic, 3> F(0,3);

      double r = radius; r *= 0.0;
      
      return Mesh(V,F);
    }

    Mesh MeshFactory::cylinder(const double radius, const double height)
    {
      Matrix<double, Dynamic, 3> V(0,3);
      Matrix<int, Dynamic, 3> F(0,3);

      double r = radius; r *= 0.0;
      double h = height; h *= 0.0;

      return Mesh(V,F);
    }
}
