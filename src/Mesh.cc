#include "Mesh.h"
#include <iostream>

namespace Geotree
{
  Mesh::Mesh(Matrix<double, Dynamic, 3> V, Matrix<int, Dynamic, 3> F) : V(V), F(F)
  {
  }

  Mesh::Mesh(int verts, int faces)
  {
    V(verts,3);
    F(faces,3);
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

    Face face(v0, v1, v2, index);

    return face;
  }

  void Mesh::getSegments(std::vector <Segment> &segments) const
  {
    std::vector <Face> faces;
    this->getFaces(faces);
    
    for (Face face : faces)
      {
	for (Segment newsegment : face.getSegments())
	  {
	    newsegment.face0 = face.index;

	    bool duplicate = false;

	    for (Segment &segment : segments)
	      {
		if (segment == newsegment)
		  {
		    segment.face1 = newsegment.face0;
		    segment.face1edge = newsegment.face0edge;
		    duplicate = true;
		    break;
		  }
	      }

	    if (!duplicate)
	      {
		segments.push_back(newsegment);
	      }	    
	  }
      }    
  }

  std::ostream& operator<< (std::ostream& stream, const Mesh& mesh)
  {
    stream << "Mesh:";

    for (int i=0; i<mesh.V.rows(); i++)
    {
      stream << "(" << mesh.V.row(i) << "),";
    }

    return stream;
  }

  void Mesh::getFaces(std::vector <Face> &faces) const
  {
    for (int f=0; f<this->F.rows(); f++)
      {
	Vector3i facerow = this->F.row(f);

	Face face(this->V.row(facerow[0]),
		  this->V.row(facerow[1]),
		  this->V.row(facerow[2]),
		  f);
	faces.push_back(face);
      }
  }

  int Mesh::add(Face face)
  {
    int vsize = this->V.rows();
    int fsize = this->F.rows();			
    
    this->V.conservativeResize(vsize + 3, NoChange);
    this->F.conservativeResize(fsize + 1, NoChange);

    this->V.row(vsize) = face.point0;
    this->V.row(vsize + 1) = face.point1;
    this->V.row(vsize + 2) = face.point2;

    this->F.row(fsize) = Vector3i(vsize, vsize+1, vsize+2);

    return fsize;
  }

  Cube Mesh::boundingBox(std::set <int> &faces) const
  {
    Cube box;

    if (this->F.rows() == 0)
      {
	box.p0 = Vector3d(0,0,0);
	box.p1 = Vector3d(0,0,0);
	return box;
      }

    for (int fi : faces)
      {
	Vector3i face = this->F.row(fi);
	
	for (int v=0; v<3; v++)
	  {
	    int vi = face[v];
	    Vector3d point = this->V.row(vi);

	    if (fi == 0 && v == 0)
	      {
		/* initialize with first point */
		box.p0 = point;
		box.p1 = point;
		continue;
	      }

	    for (int axis=0; axis<3; axis++)
	      {
		if (point[axis] < box.p0[axis]) box.p0[axis] = point[axis];
		if (point[axis] > box.p1[axis]) box.p1[axis] = point[axis];
	      }
	  }
      }

    return box;    
  }
  
  Cube Mesh::boundingBox() const
  {
    Cube box;

    if (this->F.rows() == 0)
      {
	box.p0 = Vector3d(0,0,0);
	box.p1 = Vector3d(0,0,0);
	return box;
      }

    for (int fi=0; fi<this->F.rows(); fi++)
      {
	Vector3i face = this->F.row(fi);
	
	for (int v=0; v<3; v++)
	  {
	    int vi = face[v];
	    Vector3d point = this->V.row(vi);

	    if (fi == 0 && v == 0)
	      {
		/* initialize with first point */
		box.p0 = point;
		box.p1 = point;
		continue;
	      }

	    for (int axis=0; axis<3; axis++)
	      {
		if (point[axis] < box.p0[axis]) box.p0[axis] = point[axis];
		if (point[axis] > box.p1[axis]) box.p1[axis] = point[axis];
	      }
	  }
      }

    return box;
  }
}
