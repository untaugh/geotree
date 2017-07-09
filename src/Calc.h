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

  // equal : test if indexies are same
  // ret   : true if equal
  // in f1 : indicies of face 1
  // in f2 : indicies of face 2
  bool equal(Vector3i f1, Vector3i f2);
}
