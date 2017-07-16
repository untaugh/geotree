#include <Eigen/Core>

using namespace Eigen;

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

  // divide : divide triangular by path into polygons
  // in V        : verticies
  // in F        : face
  // in P        : path dividing face
  // out P1      : resulting part 1
  // out P2      : resulting part 2
  void divide(const MatrixXd V,
	      const Vector3i F,
	      const MatrixXi P,
	      VectorXi P1,
	      VectorXi P2);

  // triangulate : divide triangular face by path and triangulate 
  // in V        : verticies
  // in F        : face
  // in P        : path dividing face
  // out F1      : resulting part 1
  // out F2      : resulting part 2
  void triangulate(const MatrixXd V,
		   const Vector3i F,
		   const MatrixXi P,
		   MatrixXi F1,
		   MatrixXi F2);

  // triangulate : triangulate path in 3d space, points in one plane
  // in P        : path
  // out F       : face indicies
  void triangulate(const MatrixXd P, MatrixXi &F);

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

  // inside : point inside triangle
  // ret    : true if yes
  // in v1  : vertex 1 of face
  // in v2  : vertex 2 of face
  // in v3  : vertex 3 of face
  // in p   : point to test
  bool inside(Vector3d v1, Vector3d v2, Vector3d v3, Vector3d p);
}
