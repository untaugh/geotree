#pragma once
#include "Types.h"
//#include "IntersectionPoint.h"
#include "Segment.h"
#include "Point.h"

namespace Geotree
{
    class IntersectionPoint;
    class PointInfo;

    class FaceT
    {
    public:
        FaceT(Mesh &_mesh, int index);
        //bool intersects(SegmentT segment);
        bool intersects(const Line segment);
        //IntersectionPoint getIntersection(SegmentT segment);
        //std::vector <IntersectionPoint2> getIntersection(FaceT face);
        Point getPoint(int index);
        bool operator == (const FaceT & face) const;
        bool operator != (const FaceT & face) const;

    private:
        PointInfo getType(Vector3d point);
        bool hasPoint(Vector3d point);
        SegmentIndex getIndex(int index);
        int getPointIndex(Vector3d point);
        Plane getVectors();
        Line getVectors(int index);
        //Vector3d getPoint(int index);
    protected:
        //int getMeshIndex(int i) { return mesh.F.row(index)[i]; };
        Mesh &mesh;
        int index;
    };
}
