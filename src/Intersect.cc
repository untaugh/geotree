#include "Intersect.h"
#include "Calc.h"
#include <iostream>

void Intersections::add(Geometry &g1, Geometry &g2)
{
  MatrixXd V1 = g1.V;
  MatrixXi F1 = g1.F;
  
  MatrixXd V2 = g2.V;
  MatrixXi F2 = g2.F;

  Vector3d v;
  
  // get segments
  MatrixXi S1, S2;
  Calc::getFaceSegments(F1, S1);
  Calc::getFaceSegments(F2, S2);

  for (int i=0; i<F1.rows(); i++)
    {
      for (int j=0; j<S2.rows(); j++)
	{
	  bool ret = Calc::getIntersection(V1, F1, V2, F2, i,
					   S2.row(j)[0], S2.row(j)[1], v);
	  if (ret)
	    {
	      Intersect is = {0, i, S2.row(j), v};
	      I.push_back(is);
	    }
	}	  
    }
  
  for (int i=0; i<F2.rows(); i++)
    {
      for (int j=0; j<S1.rows(); j++)
	{
	  bool ret = Calc::getIntersection(V2, F2, V1, F1, i,
					   S1.row(j)[0], S1.row(j)[1], v);
	  if (ret)
	    {
	      Intersect is = {1, i, S1.row(j), v};
	      I.push_back(is);
	    }
	}
    }  
}

int Intersections::numPoints(int index)
{
  int count = 0;

  for (Intersect i : I)
    {
      if (index == i.index)
	{
	  count++;
	}
    }
  return count;
}

void Intersections::add(int index, int plane, Vector2i segment, Vector3d point)
{
  Intersect is = {index, plane, segment, point};

  this->I.push_back(is);  
}

std::vector<std::set<unsigned>> Intersections::getPaths(int index, int face)
{  
  std::vector<std::set<unsigned>> is;

  std::set<unsigned> path;

  std::vector<Intersect> ipath;

  Intersect intersect_prev;

  int count = 0;
  
  // get start point
  for(Intersect i : this->I)
    {
      if ( (i.segment[0] == face || i.segment[1] == face) && i.index != index)
	{
	  ipath.push_back(i);
	  intersect_prev = i;
	  path.insert(count);
	  break;
	}
      count++;
    }

  count = 0;
  
  // midpoints
  for(Intersect i : this->I)
    {
      if ( i.plane == face && i.index == index &&
	    ( intersect_prev.plane == i.segment[0] ||
	      intersect_prev.plane == i.segment[1] ))
	{
	  ipath.push_back(i);
	  intersect_prev = i;
	  path.insert(count);
	  break;	  
	}
      count++;
    }

  count = 0;
  
  // get end point
  for(Intersect i : this->I)
    {
      if ( (i.segment[0] == face || i.segment[1] == face) && i.index != index &&
	   ipath[0].plane != i.plane)
	{
	  ipath.push_back(i);
	  path.insert(count);
	  break;
	}
      count++;
    }

  is.push_back(path);
  
  return is;
}

void Intersections::getPoints(Eigen::MatrixXd &points)
{
  int n = numPoints(0) + numPoints(1);
  points = Eigen::MatrixXd(n, 3);

  for (int i=0; i<n; i++)
    {
      points.row(i) = this->I[i].point;
    }
}
