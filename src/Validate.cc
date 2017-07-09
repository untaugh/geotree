#include "Validate.h"
#include "Calc.h"
#include <iostream>
bool Geotree::Validate::face(Matrix3d f)
{
  // duplicate vertex
  if (f.row(0) == f.row(1) ||
      f.row(0) == f.row(2) ||
      f.row(1) == f.row(2)) 
    {
      return false;
    }
  
  return true;
}

bool Geotree::Validate::face(Vector3i f)
{
  // duplicate index
  if (f[0] == f[1] ||
      f[0] == f[2] ||
      f[1] == f[2])
    {
      return false;
    }

  // negative index
  if (f[0] < 0  ||
      f[1] < 0  ||
      f[2] < 0) 
    {
      return false;
    }

  return true;
}

bool Geotree::Validate::geometry(Geometry g)
{
  // check for duplicate verticies
  for(int i=0; i<g.V.rows(); i++)
    {
      for(int j=i+1; j<g.V.rows(); j++)
	{
	  if ( g.V.row(i) == g.V.row(j))
	    {
	      return false;
	    }
	}
    }

  // index size
  if ( g.F.cols() != 3)
    {
      return false;
    }
  
  // check for index overflow
  int vertex_max = g.V.rows();

  for (int i=0; i<g.F.size(); i++)
    {
      if (g.F(i) >= vertex_max)
	{
	  return false;
	}
    }

  // check valid faces
  for (int i=0; i<g.F.rows(); i++)
    {
      if (! face( (Eigen::Vector3i)g.F.row(i) ))
	{
	  return false;
	}
    }

  // duplicate facse
  for (int i=0; i<g.F.rows(); i++)
    {
      for (int j=i+1; j<g.F.rows(); j++)
	{
	  if ( Calc::equal(g.F.row(i), g.F.row(j)) )
	    {
	      return false;
	    }
	}
    }  
  
  return true;
}
