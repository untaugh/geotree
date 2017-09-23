#pragma once

#include <Eigen/Core>
#include <set>

using namespace Eigen;

typedef Vector2d Point;
typedef Vector3d Vertex;
typedef Vector3d Vector;
typedef Vector3i Face;
typedef Vector3i FaceIndex;
typedef Matrix2d Segment;
typedef Vector2i SegmentIndex;
typedef MatrixXd Verticies;
typedef MatrixXd Verts;
//typedef MatrixXi Faces;
typedef Matrix<int, Dynamic, 3> Faces;
typedef VectorXi Path;
typedef VectorXi Indicies;
typedef std::set <unsigned> FaceSet;
typedef Matrix<double, 3, 3> Plane;
typedef Matrix<double, 2, 3> Line;
