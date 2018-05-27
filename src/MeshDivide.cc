#include "MeshDivide.h"
#include <set>
#include "Paths.h"
namespace Geotree
{
  MeshDivide::MeshDivide(const Mesh mesh0, const Mesh mesh1)
    : mesh0(mesh0), mesh1(mesh1)
  {
    IntersectionPoints ip(this->mesh0, this->mesh1);
    Paths ps(ip);
  }

  void MeshDivide::divide()
  {
    
  }
}
