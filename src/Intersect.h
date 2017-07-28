#pragma once
#include <vector>
#include <set>
#include <Eigen/Core>
#include "Geometry.h"

using namespace Eigen;

// intersection between face and a segment shared by two faces
typedef struct Intersect {
  int index;
  int plane;
  Vector2i segment;
  Vector3d point;
} Intersect;

class Intersections
{
 public:
  std::vector <Intersect> I;

  Geometry g1, g2;

  // intersections from geometries
  void add(Geometry &g1, Geometry &g2);
  
  // add an intersection point
  void add(int index, int plane, Vector2i segment, Vector3d point);

  // getPaths : get all paths dividing a face
  // in index : geometry index
  // in face  : face index
  std::vector<std::vector<int>> getPaths(int index, int face);

  // number of points in geometry
  int numPoints(int index);

  // get all points from intersection
  void getPoints(Eigen::MatrixXd &points);

  // get     : get intermediate geometries
  // out V   : all verticies
  // out F1o : faces of geometry 1 outside
  // out F1i : faces of geometry 1 inside
  // out F2o : faces of geometry 2 outside
  // out F2i : faces of geometry 2 inside
  void get(MatrixXd V, MatrixXi F1o, MatrixXi F1i, MatrixXi F2o, MatrixXi F2i);

  // faceInfo : info of faces
  // in index : geometry index
  // out Fi   : faces inside
  // out Fo   : faces outside
  // out Ft   : faces intersection
  void faceInfo(int index, std::set<int> &Fi, std::set<int> &Fo, std::set<int> &Ft);

  // divide  : divide a face and triangulate
  // in int  : index of geometry
  // in face : index of face
  // out F1  : resulting faces
  // out F2  : resulting faces
  void divide(int index, int face, MatrixXi &F1, MatrixXi &F2);

  
};
