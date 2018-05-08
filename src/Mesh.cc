#include "Mesh.h"
#include <iostream>

namespace Geotree
{
  Mesh::Mesh(Matrix<double, Dynamic, 3> V, Matrix<int, Dynamic, 3> F) : V(V), F(F)
  {
  }

  Mesh Mesh::operator +(const Mesh meshAdd)
  {
    Matrix <double, Dynamic, 3> V(meshAdd.V.rows() + this->V.rows(), 3);
    Matrix<int, Dynamic, 3> F(meshAdd.F.rows() + this->F.rows(), 3);

    V.block(0,0,this->V.rows(),3) = this->V;
    V.block(this->V.rows(),0,meshAdd.V.rows(),3) = meshAdd.V;

    F.block(0,0,this->F.rows(),3) = this->F;
    F.block(this->F.rows(),0,meshAdd.F.rows(),3) = meshAdd.F;

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

  Face Mesh::getFace(int i) const
  {
    Vector3i row = F.row(i);
    Face face(V.row(row[0]), V.row(row[1]), V.row(row[2]), i, row);

    //Face(Vector3d v0, Vector3d v1, Vector3d v2, int index, Vector3i face);

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
