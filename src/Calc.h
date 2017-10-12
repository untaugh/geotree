#pragma once
#include "Types.h"
#include <Eigen/Core>
#include <set>
#include "Geometry.h"

using namespace Eigen;

enum Axis
  {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2
  };

namespace Calc
{
  // getSegments : Get unique segments from face indicies
  // in  F       : Face indicies
  // out S       : Segments in 3D space
  void getSegments(const Faces F, Segments &S);

  // getSegments : Get segments of triangular faces
  // in  F       : Face indicies (N,3)
  // out Fo       : List of paired faces sharing unique segments (N,2)
  void getFaceSegments(const Faces F, MatrixXi &Fo);

  // sharedSegments : Number of shared segments
  // ret            : number of shared segments (0, 1 or 3)
  // in F1          : indicies of face 1
  // in F2          : indicies of face 2
  unsigned int sharedSegments(Face F1, Face F2);

  // do face and segment intersect
  bool intersectsFace(const Plane face, const Line segment);

  // do plane and segment intersect
  bool intersectsPlane(const Plane plane, const Line segment);

  // intersection between plane and segment
  Vertex intersection(const Plane plane, const Line Segment);
  
  // getSegment : Face indicies to segment indicies
  // in F       :
  // in f1      :
  // in f2      :
  // out s1     :
  // out s2     :
  void toSegment(Faces &F, unsigned int f1, unsigned int f2,
		  unsigned int &s1, unsigned int &s2);
  Segment toSegment(Face face);

  // equal : test if indexies describe the same face
  // ret   : true if equal
  // in f1 : indicies of face 1
  // in f2 : indicies of face 2
  bool equal(Face face1, Face face2);

  // triangulate : triangulate path in 3d space, points in one plane
  // ret         : true if successfull
  // in V        : verticies
  // in P        : path indicies
  // out F       : face indicies
  bool triangulate(const Verticies V, const VectorXi P, Faces &F);

  // normal : get notmal for three points in 3d space
  // ret    : normal
  // in v1  : v1 vector
  // in v2  : v2 vector
  // in v3  : v3 vector
  Vector3d normal(Vector3d v1, Vector3d v2, Vector3d v3);
  Vertex normal(const Plane plane);
  
  // distance : distance between segments
  // ret      : the distance
  // in v1a   : segment 1 start
  // in v1b   : segment 1 end
  // in v2a   : segment 2 start
  // in v2b   : segment 2 end
  double distance(Vector3d v1a, Vector3d v1b, Vector3d v2a, Vector3d v2b);

  double distance(Line segment0, Line segment1);
  
  // distance : distance between point and segment
  double distance(Vertex point, Vertex seg0, Vertex seg1);
  double distance(Vector point, Line segment1);
  
  // is point inside face
  bool inside(const Verticies face, const Vertex point);

  // are points inside face
  bool pointsInside(const Plane face, const Verticies points);

  Verticies select(Verticies verticies, Indicies points);

  // angle : angle between segments in radians from 0 to pi
  // ret   : angle in degrees
  // in v1  : point 1
  // in v2  : point 2
  // in v3  : point 3
  // in up  : planeup vector
  double angle(Vector3d v1, Vector3d v2, Vector3d v3, Vector3d up);

  // angle : angle between 2d vectors
  // ret   : angle
  double angle(Point p0, Point p1);
  
  // inside : any point inside triangle
  // ret    : true if yes
  // in v1  : vertex 1 of face
  // in v2  : vertex 2 of face
  // in v3  : vertex 3 of face
  // in P   : list of points to test
  bool inside(Vector3d v1, Vector3d v2, Vector3d v3, MatrixXd P);

  // next    : next point in list
  // ret     : true if successfull
  // in P    : List of points in path
  // out i   : start looking from index
  // in skip : skip these points
  bool next(VectorXi P, int &i, std::set<int> skip);

  // connected : list face connected to this face
  // ret       : the list
  // in F      : N*3 matrix with faces
  // in skip   : skip these faces
  // in index  : start from this index
  std::set <unsigned> connected(const MatrixXi F, std::set <unsigned> skip, const unsigned index);

  // boundingBox : get bounding box
  // in V        : verticies
  // in F        : triangular faces
  // out B1      : point 1 of box
  // out B2      : point 2 of box
  void boundingBox(Verticies V, Path F, Vector3d &B1, Vector3d &B2);
  // ret         : 3D Verticies in two rows
  Verticies boundingBox(const Geometry G);

  // winding : winding number of path
  // ret     : winding number
  // in V    : list of verticies
  // in P    : path
  int winding(Verticies V, VectorXi P);

  // crossing : crossing number of path
  // ret      : crossing number
  // in V     : list of verticies
  // in P     : path
  int crossing(MatrixXd V, VectorXi P);

  // intersect   : Does line and segment intersect
  // ret         : true if yes
  // in line     : line in direction of segment
  // in segment  : segment
  // out winding : winding number
  bool intersect(const Segment line, const Segment segment, int &winding);

  // intersect : Does segment pass through at point
  // ret       : true if yes
  // G         : geometry 
  // point     : vertex index of point
  // segment   : other end of segment from segment to point
  bool intersect(const Geometry G, const unsigned point, const Vertex segment);

  // Does line pass through the segment between two faces
  bool intersect(const Geometry G, const int face0, const int face1, const Line line);
  
  // dropAxis : Return smalles axes
  // ret      : smallest axis of three
  // in V
  // in F
  Axis minAxis(const Verticies V, const Path F);

  bool connected(Faces F, FaceSet f1, FaceSet f2);
  bool connectedPoint(Faces F, FaceSet f1, FaceSet f2);
  unsigned sharedPoints(Face face0, Face face1);
  bool hasPoint(Face face, unsigned index);
      
  // inside : is point inside geometry
  // G      : geometry
  // point  :
  bool inside(const Geometry G, const Vertex point);

  // getNotPoint : get point that is not this
  // ret         : return a point that is not
  // F           : a face with three points
  // notPoint    : not this point
  int getNotPoint(Face F, int notPoint);
  int getNotNotPoint(Face F, int notPoint0, int notPoint1);
  
  // findPoint : find face with point 1 and 2, but not 3
  int findPoint(const Faces F, const unsigned p0, const unsigned p1, const unsigned p2_not);

  bool abovePlane(Plane plane, Vertex point);

  bool hasPoint(Verticies verticies, Vertex point);

  int pointIndex(Verticies verticies, Vertex point);

  bool atEdge(Plane face, Vertex point);
  bool atSegment(Line segment, Vertex point);

  SegmentIndex getSegment(const Geometry geometry, const int faceIndex, const Vertex point);

  Vector2i getFaces(const Geometry geometry, SegmentIndex segment);
  bool hasSegment(Face face, SegmentIndex segment);
  bool closeToZero(double value);

  bool isEndPoint(Line segment, Vector point);
  bool isEndPoint(Plane face, Vector point);

  bool vectorEqual(Vector v1, Vector v2);
  bool equal(double d1, double d2);
  bool contains(FaceSet first, FaceSet second);
}
