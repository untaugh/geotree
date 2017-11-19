#pragma once
#include "Types.h"
#include "EdgePoint.h"

namespace Geotree {

  class IntersectionFace;
  
  class IntersectionPaths
  {
  public:
  IntersectionPaths(IntersectionPoints &_points)
    : intersectionPoints(_points){}
    void calculatePaths();

    int size() { return paths.size(); };
    std::vector <FaceSet> getSubPaths(int face);
    std::vector <Path> getPaths(){ return paths; };

    //IntersectionFace getFace(int face);
  private:
    int getPathStartIndex(Path path, int face);
    int totalPathSize();
    int getFreePoint();
    Path calculatePath(int intersectionPointIndex);
    std::vector <Path> paths;
    const IntersectionPoints &intersectionPoints;
  };
}
