#include "Mesh.h"

namespace Geotree
{
  unsigned Mesh::size()
  {
    return this->F.rows();
  }
  
  Mesh Mesh::operator +(const Mesh meshAdd)
  {
    Mesh mesh;

    mesh.V = Verts(meshAdd.V.rows() + this->V.rows(), 3);
    mesh.F = Faces(meshAdd.F.rows() + this->F.rows(), 3);

    return mesh;
  }

  void Mesh::translate(const Vertex vertex)
  {
    this->V.rowwise() += vertex.transpose();
  }
}
