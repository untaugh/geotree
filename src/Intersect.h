#pragma once
#include <vector>
#include <set>
#include <Eigen/Core>

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

  // add an intersection point
  void add(int index, int plane, Vector2i segment, Vector3d point);

  // get all paths dividing a face
  std::vector<std::set<unsigned>> getPaths(int index, int face);

  int numPoints(int index);
  
  // get all points from intersection
  //getPoints(Eigen::MatrixXd points);
};
