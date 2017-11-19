#pragma once
#include "Types.h"
#include "Mesh.h"
#include "Point.h"

namespace Geotree
{
  class Mesh;

  class PointInfo
  {
  public:
    PointType type;
    int index;
    int index2 = 0;
    FaceSet faces;

    bool isSegment();
    SegmentIndex getSegment();
    bool isConnected(PointInfo point);

    void setFaces(Mesh mesh);

    FaceSet getFaces(Mesh mesh);

    bool operator==(const PointInfo &pointType) const;
  };
}
