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

  // intersections from geometries
  void add(Geometry &g1, Geometry &g2);
  
  // add an intersection point
  void add(int index, int plane, Vector2i segment, Vector3d point);

  // get all paths dividing a face
  std::vector<std::set<unsigned>> getPaths(int index, int face);

  // number of points in geometry
  int numPoints(int index);

  // get all points from intersection
  void getPoints(Eigen::MatrixXd &points);
};
