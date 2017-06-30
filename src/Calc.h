#include <Eigen/Core>

using namespace Eigen;

namespace Calc
{
  // Calculate segments from triangular faces
  // in  V: Verticies in 3D space
  // in  F: Face indicies
  // out S: Segments in 3D space
  void getSegments(MatrixXi &F, MatrixXi &S);
}
