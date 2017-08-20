#include <Eigen/Core>
#include <set>

using namespace Eigen;

typedef Vector2d Point;
typedef Vector3d Vertex;
typedef Vector3i Face;
typedef Matrix2d Segment;
typedef MatrixXd Verticies;
typedef MatrixXi Faces;
typedef VectorXi Path;

enum Axis
  {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2
  };

namespace Calc
{
  // getSegments : Get segments of triangular faces
  // in  F       : Face indicies
  // out S       : Segments in 3D space
  void getSegments(MatrixXi &F, MatrixXi &S);

  // getSegments : Get segments of triangular faces
  // in  F       : Face indicies (N,3)
  // out Fo       : List of paired faces sharing unique segments (N,2)
  void getFaceSegments(MatrixXi &F, MatrixXi &Fo);

  // sharedSegments : Number of shared segments
  // ret            : number of shared segments (0, 1 or 3)
  // in F1          : indicies of face 1
  // in F2          : indicies of face 2
  unsigned int sharedSegments(Vector3i F1, Vector3i F2);

  // getIntersection : Get point of intersection between face and segment
  // ret (bool)      : Intersection found
  // in V1           : Verticies geometry 1
  // in F1           : Faces in geometry1
  // in V2           : Verticies geometry 2
  // in F2           : segments in geometry2
  // in f1           : Face index in VF1
  // in f2a          : Face index in VF2
  // in f2b          : Segment end in VS2
  // out p           : Point of intersection
  bool getIntersection(MatrixXd &V1, MatrixXi &F1,
		       MatrixXd &V2, MatrixXi &F2,
		       unsigned int f1,
		       unsigned int f2a, unsigned int f2b,
		       Vector3d &p);

  // getIntersection : Get point of intersection between face and segment
  // ret (bool)      : Intersection found
  // in F            : 3x3 matrix verticies of a face 
  // in segment      : 2x3 matrix start and end points of a segment
  // in point        : intersecting point
  bool getIntersection(MatrixXd F, MatrixXd segment, Vector3d &point);

  // getSegment : Face indicies to segment indicies
  // in F       :
  // in f1      :
  // in f2      :
  // out s1     :
  // out s2     :
  void toSegment(MatrixXi &F, unsigned int f1, unsigned int f2,
		  unsigned int &s1, unsigned int &s2);

  // equal : test if indexies describe the same face
  // ret   : true if equal
  // in f1 : indicies of face 1
  // in f2 : indicies of face 2
  bool equal(Vector3i f1, Vector3i f2);

  // triangulate : triangulate path in 3d space, points in one plane
  // ret         : true if successfull
  // in V        : verticies
  // in P        : path indicies
  // out F       : face indicies
  bool triangulate(const MatrixXd V, const VectorXi P, MatrixXi &F);

  // normal : get notmal for three points in 3d space
  // ret    : normal
  // in v1  : v1 vector
  // in v2  : v2 vector
  // in v3  : v3 vector
  Vector3d normal(Vector3d v1, Vector3d v2, Vector3d v3);

  // distance : distance between segments
  // ret      : the distance
  // in v1a   : segment 1 start
  // in v1b   : segment 1 end
  // in v2a   : segment 2 start
  // in v2b   : segment 2 end
  double distance(Vector3d v1a, Vector3d v1b, Vector3d v2a, Vector3d v2b);

  double distance(Vertex point, Vertex seg0, Vertex seg1);

  // inside : point inside triangle
  // ret    : true if yes
  // in v1  : vertex 1 of face
  // in v2  : vertex 2 of face
  // in v3  : vertex 3 of face
  // in p   : point to test
  bool inside(Vector3d v1, Vector3d v2, Vector3d v3, Vector3d p);
  bool inside(Vector3d v1, Vector3d v2, Vector3d v3, MatrixXd V, VectorXi P);
  
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
  std::set <int> connected(MatrixXi F, std::set <int> skip, int index);

  // boundingBox : get bounding box
  // in V        : verticies
  // in F        : triangular faces
  // out B1      : point 1 of box
  // out B2      : point 2 of box
  void boundingBox(MatrixXd V, MatrixXi F, Vector3d &B1, Vector3d &B2);

  // winding : winding number of path
  // ret     : winding number
  // in V    : list of verticies
  // in P    : path
  int winding(MatrixXd V, VectorXi P);

  // crossing : crossing number of path
  // ret      : crossing number
  // in V     : list of verticies
  // in P     : path
  int crossing(MatrixXd V, VectorXi P);

  // intersect : Does line and segment intersect
  // ret       : true if yes
  // in l1, l2 : line
  // in s1, s2 : segment
  //bool intersect(Vector2d l1, Vector2d l2, Vector2d s1, Vector2d s2, int &wind);

  // intersect   : Does line and segment intersect
  // ret         : true if yes
  // in line     : line in direction of segment
  // in segment  : segment
  // out winding : winding number
  bool intersect(const Segment line, const Segment segment, int &winding);

  // dropAxis : Return smalles axes
  // ret      : smallest axis of three
  // in V
  // in F
  Axis minAxis(const Verticies V, const Faces F);
}
