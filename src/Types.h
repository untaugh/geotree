#pragma once

#define EIGEN_MATRIX_PLUGIN "MatrixBaseAddons.h"
#include <Eigen/Core>
#include <set>

using namespace Eigen;

enum PointType
  {
    POINT,
    SEGMENT,
    FACE,
  };

//typedef Vector2i IndexPair;
typedef Vector3d Vertex;
typedef Vector3d Vector;
//typedef Vector3i Face;
//typedef Vector3i FaceIndex;
typedef Matrix2d Segment;
typedef Vector2i SegmentIndex;
typedef MatrixXd Verts;
typedef VectorXi Path;
typedef VectorXi Indicies;
typedef Matrix<double, Dynamic, 3> Verticies;
typedef Matrix<double, Dynamic, 2> Points;
typedef Matrix<int, Dynamic, 3> Faces;
typedef Matrix<int, Dynamic, 2> Segments;
typedef std::set <unsigned> FaceSet;
typedef Matrix<double, 3, 3> Plane;
typedef Matrix<double, 2, 3> Line;
