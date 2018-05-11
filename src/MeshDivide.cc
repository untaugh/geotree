#include "MeshDivide.h"
#include <set>
#include "Path.h"

namespace Geotree
{
  MeshDivide::MeshDivide(const IntersectionPoints &_points)
  : points(_points)
  {
  }

  void MeshDivide::divide()
  {
    std::set <int> div0, div1;

    points.dividedFaces(div0, div1);

    for (int i : div0)
    {
      std::vector <Path> paths;

      //std::cout << "div0: " << i << std::endl;
      
      //Face face = points.mesh0.getFace(i);

      Path path;

      for (Point p : points.points)
      {
        if (p.faces0.find(i) != p.faces0.end())
        {
          //std::cout << "adding: " << i << " " << p << std::endl;

          paths.push_back(path);
        }
      }
    }

    // for (int i : div1)
    // {
    //   std::cout << "div1: " << i << std::endl;
    // }
  }
}
