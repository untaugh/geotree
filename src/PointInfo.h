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

    //bool hasFace(FaceIndex face);
    //bool hasSegment(SegmentIndex segment);
    //bool hasPoint(int point);

    //bool isConnected2(PointInfo point);


    //Faces faceIndex;

    void setFaces(Mesh mesh);
    //void setFaceIndex(Mesh mesh);
    FaceSet getFaces(Mesh mesh);
    
    bool operator==(const PointInfo &pointType) const;
  };
}
