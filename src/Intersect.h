#pragma once
#include <vector>
#include <set>
#include <Eigen/Core>
#include "Geometry.h"

using namespace Eigen;

// intersection between face and a segment shared by two faces
typedef struct Intersect {
  int plane;
  Vector2i segment;
  Vector3d point;
} Intersect;

class Intersections
{
public:
  // intersections between gemetries
  std::vector <Intersect> I;

  // g1 : input geometry 1
  // g2 : input geometry 2
  // gx : resulting geometry, g1 + g2 + (intersections)
  Geometry g1, g2, gx;

  FaceSet A0, A1, B0, B1;

  Geometry CSGUnion, CSGDifference, CSGIntersection;

  // intersections from geometries
  void add(Geometry &g1, Geometry &g2);
  
  // add an intersection point
  void add(int plane, Vector2i segment, Vector3d point);

  // getPaths : get all paths dividing a face
  // in face  : face index
  std::vector<std::vector<int>> getPaths(int face);
  static bool getPathsNext(int face, std::vector<int> &path, std::vector <Intersect> I);

  // get all points from intersection
  Verticies getPoints();

  // divide   : divide a face and triangulate
  // in face  : face index to divide
  // out div1 : resulting faces
  // out div2 : resulting faces
  void divide(int face, std::set <unsigned> &div1, std::set <unsigned> &div2);

  void divide();
  
  // getIntersectingFaces : get faces that intersect with another
  // ret                  : true if successfull
  // in faces             : intersecting faces
  bool getIntersectingFaces(std::set <unsigned> &faces);

  
  bool getFaceGroups(std::vector <std::set <unsigned>> &faces);

  // removeFaces : remove faces from gx
  // faces       : list of faces to remove
  void removeFaces(const std::set <unsigned> faces);

};
