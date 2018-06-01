#include "Triangulate.h"
#include <iostream> //debug

namespace Geotree
{
  // Triangulate::Triangulate(std::vector<Point2D> face, std::vector<std::vector<Point2D>> holes, std::vector<std::vector<Point2D>> splits)
  // {
  //   (void)face;
  //   (void)holes;
  //   (void)splits;
  // }

  Triangulate::Triangulate(std::vector<Point2D> face, std::vector<std::vector<Point2D>> holes)
  {
    (void)face;
    (void)holes;

    using Point = std::array<double, 2>;
    std::vector<std::vector<Point>> polygon;
    
    polygon.push_back(face);

    for (std::vector<Point2D> hole : holes)
      {
	polygon.push_back(hole);
      }
    
    std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polygon);

    for (uint32_t index : indices)
      {
	std::cout << index << std::endl; 
      }
    std::cout << indices.size() << std::endl;
  }
}
