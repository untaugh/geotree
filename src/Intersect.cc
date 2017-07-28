#include "Intersect.h"
#include "Calc.h"
#include "Validate.h"
#include <iostream>

void Intersections::add(Geometry &g1, Geometry &g2)
{

  if (!Geotree::Validate::geometry(g1) || !Geotree::Validate::geometry(g2))
    {
      return;
    }
  
  this->g1 = g1;
  this->g2 = g2;

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
	  // face points
	  MatrixXd F = MatrixXd(3,3);
	  F.row(0) = V1.row(F1.row(i)[0]);
	  F.row(1) = V1.row(F1.row(i)[1]);
	  F.row(2) = V1.row(F1.row(i)[2]);

	  // segment points
	  MatrixXd S = MatrixXd(2,3);
	  unsigned int n1,n2;
	  Calc::toSegment(F2, S2.row(j)[0], S2.row(j)[1], n1, n2);
	  S.row(0) = V2.row(n1);
	  S.row(1) = V2.row(n2);
	  
	  bool ret = Calc::getIntersection(F, S, v);

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
	  // face points
	  MatrixXd F = MatrixXd(3,3);
	  F.row(0) = V2.row(F2.row(i)[0]);
	  F.row(1) = V2.row(F2.row(i)[1]);
	  F.row(2) = V2.row(F2.row(i)[2]);

	  // segment points
	  MatrixXd S = MatrixXd(2,3);
	  unsigned int n1,n2;
	  Calc::toSegment(F1, S1.row(j)[0], S1.row(j)[1], n1, n2);
	  S.row(0) = V1.row(n1);
	  S.row(1) = V1.row(n2);
	  
	  bool ret = Calc::getIntersection(F, S, v);

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

std::vector<std::vector<int>> Intersections::getPaths(int index, int face)
{
  std::vector<std::vector<int>> paths;
  std::vector<int> path;

  int segment = -1;
  int segment_start = -1;
  
  // get start point
  for (int i=0; i<this->I.size(); i++)
    {
      Intersect is = this->I[i];
      
      if ( (is.segment[0] == face || is.segment[1] == face)
	   && is.index != index )
	{
	  std::cout << "start " << i << std::endl;
	  path.push_back(i);
	  break;
	}
    }

  if (path.size() == 0)
    {
      // get start point inside
      for (int i=0; i<this->I.size(); i++)
	{
	  Intersect is = this->I[i];
      
	  if ( is.plane == face && is.index == index )
	    {
	      std::cout << "start inside" << i << std::endl;
	      path.push_back(i);
	      break;
	    }
	}
    }

  // if no start found
  if (path.size() == 0)
    {
      return paths;
    }
  
  bool last = false;
  
  while(!last)
    {
      Intersect is_prev = this->I[path.back()];
      Intersect is_start = this->I[path.front()];

      last = true;
      
      // mid points
      for (int i=0; i<this->I.size(); i++)
	{
	  
	}
      
    }

  Intersect is_prev = this->I[path.back()];
  Intersect is_start = this->I[path.front()];

  for (int i=0; i<this->I.size(); i++)
    {
      Intersect is = this->I[i];
      
      if ( (is.segment[0] == face || is.segment[1] == face)
	   && is.index != index)
	{
	  
	  if (path.size() == 1 && is_start.plane == is.plane
	      && (is.segment[0] != is_prev.segment[0] ||
		  is.segment[1] != is_prev.segment[1]))
	    {
	      std::cout << "End " << i << std::endl;
	      path.push_back(i);
	      break;		  
	    }

	  if ( is_prev.segment[0] == is.plane || is_prev.segment[1] == is.plane )
	    {
	      if ( is_start.plane != is.plane )
		{
		  std::cout << "end " << i << std::endl;
		  path.push_back(i);
		  break;
		}
	    }
	}
    }

  if (path.size() > 1)
    {
      paths.push_back(path);
    }
  
  return paths;
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

void Intersections::get(MatrixXd V, MatrixXi F1o, MatrixXi F1i, MatrixXi F2o, MatrixXi F2i)
{
  
}

void Intersections::faceInfo(int index, std::set<int> &Fi, std::set<int> &Fo, std::set<int> &Ft)
{
  // get intersecting faces
  for (Intersect i : this->I)
    {
      if (i.index == index)
	{
	  Ft.insert(i.plane);
	}
      else
	{
	  Ft.insert(i.segment[0]);
	  Ft.insert(i.segment[1]);
	}
    }

  // more faces
  int starti;
  std::set <int> c1, c2;
  MatrixXi F;
  
  if (index == 0)
    {
      F = g1.F;
    }
  else
    {
      F = g2.F;
    }

  // first set
  for (starti=0; starti<F.rows(); starti++)
    {
      if (! Ft.count(starti))
	{
	  break;
	}
    }
  
  c1 = Calc::connected(g1.F, Ft, starti);

  // second set
  for (starti=0; starti<F.rows(); starti++)
    {
      if (! Ft.count(starti) && ! c1.count(starti))
	{
	  break;
	}
    }

  // second set not found
  if (starti < F.rows())
    {
      c2 = Calc::connected(g1.F, Ft, starti);
    }
  
  Fo = c1;
  Fi = c2;  
}


void Intersections::divide(int index, int face, MatrixXi &F1, MatrixXi &F2)
{

  int c = 0;
  for (Intersect i: I )
    {
      std::cout << c++;
      std::cout << " index: " << i.index;
      std::cout << ", plane: " << i.plane;
      std::cout << ", segment: " << i.segment.transpose();
      std::cout << ", point: " << i.point.transpose() << std::endl;
    }
  
  std::vector<std::vector<int>> ps = getPaths(index,face);

  std::cout << "divide: " << face << std::endl;
	
  for (std::vector<int> s: ps)
    {
      std::cout << "path:" << std::endl;
      for (int i: s)
	{
	  std::cout << "p " << i
		    << ", " << this->I[i].plane
		    << ", " << this->I[i].segment.transpose() << std::endl;
	}
    }
}
