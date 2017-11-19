#pragma once
#include "Types.h"
#include "PointInfo.h"

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
    int add(const Vector3d point);
    
    Mesh operator +(const Mesh mesh); // union
    Mesh operator -(const Mesh mesh); // difference
    Mesh operator &(const Mesh mesh); // intersection
    Mesh operator +=(const Mesh mesh); // union
    Mesh operator -=(const Mesh mesh); // difference
    Mesh operator &=(const Mesh mesh); // intersection
    void translate(const Vertex v);

    FaceT getFaceT(const unsigned i);
    Plane getFaceVectors(int index);
    Segments getSegments();

    FaceSet getFaces(PointInfo point);
    FaceSet getFacesWithPoint(int pointIndex);
    FaceSet getFacesWithSegment(SegmentIndex segment);
  };

  std::ostream& operator<< (std::ostream& stream, const Mesh& mesh);  
}
