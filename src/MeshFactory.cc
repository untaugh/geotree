#include "MeshFactory.h"

namespace Geotree
{
  Mesh MeshFactory::makeCube(const double width, const double height, const double depth)
  {
    Mesh cube;

    if (width <= 0.0 || height <= 0.0 || depth <= 0.0)
      {
	return cube;
      }

    cube.V = Verts(8,3);
    cube.F = Faces(12,3);
    
    cube.V << 0.0, 0.0, 0.0,
      0.0, height, 0.0,
      width, height, 0.0,
      width, 0.0, 0.0,
      0.0, 0.0, depth,
      0.0, height, depth,
      width, height, depth,
      width, 0.0, depth;

    cube.F << 0,1,2,
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

    return cube;
  }

  Mesh MeshFactory::makeSphere(const double radius)
  {
    Mesh sphere;
    
    return sphere;
  }

  Mesh MeshFactory::makeCylinder(const double radius, const double height)
  {
    Mesh cylinder;
    
    return cylinder;
  }
}
