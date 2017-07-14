#include "Validate.h"
#include "Calc.h"
#include <iostream>
#include <stdlib.h>

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

bool Geotree::Validate::planar(MatrixXd P)
{
  Vector3d normal_ref = Calc::normal(P.row(0), P.row(1), P.row(2));
  
  for (int i=3; i<P.rows(); i++)
    {
      Vector3d normal = Calc::normal(P.row(0), P.row(1), P.row(i));
      
      Vector3d normal2 = -normal;

      long int tmp1[3];
      tmp1[0] = *(long int*) &normal_ref[0];
      tmp1[1] = *(long int*) &normal_ref[1];
      tmp1[2] = *(long int*) &normal_ref[2];

      long int tmp2[3];
      tmp2[0] = *(long int*) &normal[0];
      tmp2[1] = *(long int*) &normal[1];
      tmp2[2] = *(long int*) &normal[2];
      
      long int tmp3[3];
      tmp3[0] = *(long int*) &normal2[0];
      tmp3[1] = *(long int*) &normal2[1];
      tmp3[2] = *(long int*) &normal2[2];

      // printf("\nnormal_ref:    0x%lX, 0x%lx, 0x%lx\n", tmp1[0], tmp1[1], tmp1[2]);
      // printf("normal:        0x%lX, 0x%lx, 0x%lx\n", tmp2[0], tmp2[1], tmp2[2]);
      // printf("normal2:       0x%lX, 0x%lx, 0x%lx\n", tmp3[0], tmp3[1], tmp3[2]);

      // printf("\nnormal_ref:    %ld, %ld, %ld\n", tmp1[0], tmp1[1], tmp1[2]);
      // printf("normal:        %ld, %ld, %ld\n", tmp2[0], tmp2[1], tmp2[2]);
      // printf("normal2:       %ld, %ld, %ld\n", tmp3[0], tmp3[1], tmp3[2]);
      
      // printf("calc0:    0x%ld\n", abs(tmp1[0] - tmp2[0]));
      // printf("calc0:    0x%ld\n", abs(tmp1[0] - tmp3[0]));
      // printf("calc1:    0x%ld\n", abs(tmp1[1] - tmp2[1]));
      // printf("calc1:    0x%ld\n", abs(tmp1[1] - tmp3[1]));
      // printf("calc2:    0x%ld\n", abs(tmp1[2] - tmp2[2]));
      // printf("calc2:    0x%ld\n", abs(tmp1[2] - tmp3[2]));
      
      if ( (abs(tmp1[0] - tmp2[0]) > 2 ||
	    abs(tmp1[1] - tmp2[1]) > 2 ||
	    abs(tmp1[2] - tmp2[2]) > 2) &&
	   (abs(tmp1[0] - tmp3[0]) > 2 ||
	    abs(tmp1[1] - tmp3[1]) > 2 ||
	    abs(tmp1[2] - tmp3[2]) > 2) )
	{
	  return false;
	}      
    }

  return true;

}

bool Geotree::Validate::intersect(Vector3d v1a, Vector3d v1b, Vector3d v2a, Vector3d v2b)
{
  return true;
}
