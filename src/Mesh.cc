#include "Mesh.h"
#include "Calc.h"
#include <iostream>
#include "Face.h"

namespace Geotree
{
  unsigned Mesh::size()
  {
    return this->F.rows();
  }

  FaceT Mesh::getFaceT(const unsigned i)
  {
    FaceT face(this->V, this->F.row(i));
    return face;
  }

  Mesh Mesh::operator +(const Mesh meshAdd)
  {
    Mesh mesh;

    mesh.V = Verts(meshAdd.V.rows() + this->V.rows(), 3);
    mesh.F = Faces(meshAdd.F.rows() + this->F.rows(), 3);

    mesh.V.block(0,0,this->V.rows(),3) = this->V;
    mesh.V.block(this->V.rows(),0,meshAdd.V.rows(),3) = meshAdd.V;

    mesh.F.block(0,0,this->F.rows(),3) = this->F;
    mesh.F.block(this->F.rows(),0,meshAdd.F.rows(),3) = meshAdd.F;

    int offset = this->V.rows();
    
    for (int i=this->F.rows(); i< mesh.F.rows(); i++)
      {
	mesh.F.row(i)(0) += offset;
	mesh.F.row(i)[1] += offset;
	mesh.F.row(i)[2] += offset;
      }
    return mesh;
  }

  void Mesh::translate(const Vertex vertex)
  {
    this->V.rowwise() += vertex.transpose();
  }

  Segments Mesh::getSegments()
  {
    Segments segments;
    Calc::getSegments(this->F, segments);
    return segments;
  }
  
  int Mesh::getFaceIndex(Face face)
  {
    for (int i=0; i<this->F.rows(); i++)
      {
	if (Calc::equal(this->F.row(i), face))
	  {
	    return i;
	  }
      }
    return -1;
  }

  FaceSet Mesh::getFacesWithPoint(int pointIndex)
  {
    FaceSet faces;
    for (int i=0; i<F.rows(); i++)
      {
	if (Calc::hasPoint(F.row(i), pointIndex))
	  {
	    faces.insert(i);
	  }	
      }
    return faces;
  }

  FaceSet Mesh::getFacesWithSegment(SegmentIndex segment)
  {
    FaceSet faces;

    for (int i=0; i<F.rows(); i++)
      {
	if (Calc::hasSegment(F.row(i), segment))
	  {
	    faces.insert(i);
	  }
      }
    
    return faces;
  } 
}
