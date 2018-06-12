#pragma once
#include <Eigen/Core>

using namespace Eigen;

namespace Geotree
{
  enum Axis
  {
    X=0,
    Y=1,
    Z=2,
  };

  class Segment
  {
  public:
    Segment(Vector3d begin, Vector3d end, int edge)
      : begin(begin), end(end), face0edge(edge) {};

    const Vector3d begin;
    const Vector3d end;

    int face0;
    int face1;
    int face0edge;
    int face1edge;

    bool operator == (Segment &segment);
  };
}
