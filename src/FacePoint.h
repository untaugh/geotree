#pragma once
#include "Point.h"

namespace Geotree
{

class FacePoint : public Point {
public:
    FacePoint(const Mesh &mesh, int index) : Point(mesh, index) {};
    FacePoint(const Mesh &mesh, int index, PointType _type, int _segment) : Point(mesh, index), type(_type), segment(_segment) {};
    FacePoint(Mesh &mesh, Vector3d point, PointType _type, int _segment);
    PointType type;
    int segment;
};

}