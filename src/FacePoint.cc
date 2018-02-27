#include "FacePoint.h"

namespace Geotree
{
    FacePoint::FacePoint(Mesh &mesh, Vector3d point, PointType _type, int _segment) : Point(mesh, mesh.add(point)) {
        type = _type;
        segment = _segment;
    }
}