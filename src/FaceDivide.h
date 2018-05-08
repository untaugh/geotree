#pragma once
#include <Eigen/Core>
#include <set>
#include <vector>

using namespace Eigen;

namespace Geotree
{
  struct edgepoint
  {
    unsigned segment;
    unsigned index;
    double distance;

    bool operator !=(const edgepoint &ep) const
    {
      return this->index != ep.index;
    }

    bool operator <(const edgepoint &ep) const
    {
      if (this->segment < ep.segment) return true;
      else if (this->segment > ep.segment) return false;
      else
      {
        if (this->distance < ep.distance) return true;
        else return false;
      }
    }
  };

  class FaceDivide
  {
  public:
    FaceDivide(const Vector2d p0, const Vector2d p1, const Vector2d p2);

    void cut(Matrix <double, Dynamic, 2> path);

    void hole(Matrix <double, Dynamic, 2> path);

    void divide();
    void triangulate();

    Matrix <double, Dynamic, 2> V;
    std::vector <Matrix <int, Dynamic, 3>> Fs;

  private:
    edgepoint getEdgepoint(int index);
    const edgepoint findEdgepoint(unsigned index);
    const edgepoint nextEdgepoint(edgepoint point);
    const edgepoint previousEdgepoint(edgepoint point);

    std::set <edgepoint> edgepoints;

    std::vector<std::vector <int>> cutpaths;
    std::vector<std::vector <int>> completepaths;
  };
}
