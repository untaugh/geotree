#pragma once
#include "Types.h"
#include "Point.h"
#include "PointInfo.h"
#include "Face.h"

namespace Geotree
{
    class FaceT;
    class SegmentT;
    class Point;
    class PointInfo;

    class Mesh
    {
    private:
        Matrix<double, Dynamic, 3> V;
        Matrix<int, Dynamic, 3> F;
    public:
        Mesh(Matrix<double, Dynamic, 3> V, Matrix<int, Dynamic, 3> F);
        unsigned size();
        unsigned facecount();
        unsigned vectorcount();
        int add(const Vector3d point);
        int addFace(const Vector3i face);

        Mesh operator +(const Mesh mesh); // union
//    Mesh operator -(const Mesh mesh); // difference
//    Mesh operator &(const Mesh mesh); // intersection
//    Mesh operator +=(const Mesh mesh); // union
//    Mesh operator -=(const Mesh mesh); // difference
//    Mesh operator &=(const Mesh mesh); // intersection
        void translate(const Vertex v);

        FaceT getFaceT(const unsigned i);
        Plane getFaceVectors(int index);
        Segments getSegments();

        Point getPoint(int index) const;

        FaceSet getFaces(PointInfo point);
        FaceSet getFacesWithPoint(int pointIndex);
        FaceSet getFacesWithSegment(SegmentIndex segment);
    };

    std::ostream& operator<< (std::ostream& stream, const Mesh& mesh);
}
