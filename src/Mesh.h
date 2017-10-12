#pragma once
#include "Types.h"
//#include "Face.h"

namespace Geotree
{
  class FaceT;
  class SegmentT;
  
  class Mesh
  {
  public:
    Verticies V;
    Faces F;

    unsigned size();

    Mesh operator +(const Mesh mesh); // union
    Mesh operator -(const Mesh mesh); // difference
    Mesh operator &(const Mesh mesh); // intersection
    Mesh operator +=(const Mesh mesh); // union
    Mesh operator -=(const Mesh mesh); // difference
    Mesh operator &=(const Mesh mesh); // intersection
    void translate(const Vertex v);

    FaceT getFaceT(const unsigned i);
    Segments getSegments();

    int getFaceIndex(FaceIndex face);

    FaceSet getFacesWithPoint(int pointIndex);
    FaceSet getFacesWithSegment(SegmentIndex segment);
  };
}
