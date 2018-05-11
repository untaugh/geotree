#include "Mesh.h"
#include <iostream>

namespace Geotree
{
  Mesh::Mesh(Matrix<double, Dynamic, 3> V, Matrix<int, Dynamic, 3> F) : V(V), F(F)
  {
  }

  Mesh Mesh::operator +(const Mesh mesh)
  {
    Matrix <double, Dynamic, 3> V(mesh.V.rows() + this->V.rows(), 3);
    Matrix<int, Dynamic, 3> F(mesh.F.rows() + this->F.rows(), 3);

    V.block(0,0,this->V.rows(),3) = this->V;
    V.block(this->V.rows(),0,mesh.V.rows(),3) = mesh.V;

    F.block(0,0,this->F.rows(),3) = this->F;
    F.block(this->F.rows(),0,mesh.F.rows(),3) = mesh.F;

    int offset = this->V.rows();

    for (int i=this->F.rows(); i< F.rows(); i++)
    {
      F.row(i)(0) += offset;
      F.row(i)[1] += offset;
      F.row(i)[2] += offset;
    }

    return Mesh(V,F);
  }

  Mesh::Mesh()
  {
  }

  void Mesh::translate(const Vector3d vertex)
  {
    this->V.rowwise() += vertex.transpose();
  }

  Face Mesh::getFace(int index) const
  {
    Vector3i Vi = F.row(index);

    Vector3d v0 = V.row(Vi[0]);
    Vector3d v1 = V.row(Vi[1]);
    Vector3d v2 = V.row(Vi[2]);

    Face face(v0, v1, v2, index, Vi);

    return face;
  }

  void Mesh::getSegments(std::vector <Segment> &segments) const
  {
    for (int i=0; i < F.rows(); i++)
    {
      for (int j=0; j<3; j++)
      {
        bool duplicate = false;
        int i1 = F.row(i)[j];
        int i2 = F.row(i)[(j+1)%3];

        Segment newSegment(V.row(i1), V.row(i2));
        newSegment.point0 = i1;
        newSegment.point1 = i2;
        newSegment.face0 = i;

        for (Segment &s : segments)
        {
          if (s == newSegment)
          {
            s.face1 = i;
            duplicate = true;
            break;
          }
        }

        if (!duplicate)
        {
          segments.push_back(newSegment);
        }
      }
    }
  }

  std::ostream& operator<< (std::ostream& stream, const Mesh& mesh)
  {
    stream << "Mesh:";

    //for (int i=0; i<mesh.V.rows(); i++)
    {
      //stream << "(" << mesh.V.row(i) << "),";
    }

    return stream;
  }
}
