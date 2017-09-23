#include "Geometry.h"
#include <Eigen/Dense>
#include <iostream>

Geometry::Geometry(Verticies V_, Faces F_)
{
  this->V = V_;
  this->F = F_;
}

void Geometry::add(const Geometry g)
{
  unsigned V_size = this->V.rows();
  unsigned F_size = this->F.rows();

  // empty geometry
  if ( ! V_size && ! F_size )
    {
      this->V = g.V;
      this->F = g.F;
      return;
    }
  
  unsigned gV_size = g.V.rows();
  unsigned gF_size = g.F.rows();

  // add verticies
  if (gV_size)
    {
      this->V.conservativeResize(V_size + gV_size, NoChange);
      this->V.block(V_size, 0, gV_size, 3) = g.V;
    }

  // add faces
  if (gF_size)
    {
      this->F.conservativeResize(F_size + gF_size, NoChange);  
      this->F.block(F_size, 0, gF_size, 3) = g.F;

      // add offset to faces
      for (unsigned i = F_size; i < F_size + gF_size; i++)
	{
	  this->F.row(i)[0] += V_size;	  
	  this->F.row(i)[1] += V_size;
	  this->F.row(i)[2] += V_size;
	}
    }
}

void Geometry::addFaces(const Faces F_add)
{
  int size = this->F.rows();
  int add = F_add.rows();

  if (add)
    {
      this->F.conservativeResize(size + add, NoChange);
      this->F.block(size, 0, add, 3) = F_add;
    }
}

void Geometry::addVerts(const Verticies V_add)
{
  int size = this->V.rows();
  int add = V_add.rows();

  if (add)
    {
      this->V.conservativeResize(size + add, NoChange);
      this->V.block(size, 0, add, 3) = V_add;
    }
}

bool Geometry::removeFace(const unsigned int index)
{
  return true;
}
