#include "Intersect.h"
#include "Calc.h"
#include "Validate.h"
#include <algorithm>
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

  // add face offset for second geometry
  int offset = g1.F.rows();
  
  for (unsigned i=0; i< this->I.size(); i++)
    {
      if (this->I[i].index == 1)
	{
	  this->I[i].plane += offset;
	}
      else
	{
	  this->I[i].segment[0] += offset;
	  this->I[i].segment[1] += offset;
	}
    }

  // add to gx
  MatrixXd V; 
  MatrixXi F = MatrixXi(0, 3); 
  this->getPoints(V);
  Geometry G_is(V,F); 
    
  this->gx.add(this->g1);
  this->gx.add(this->g2);
  this->gx.add(G_is);
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

bool Intersections::getPathsNext(int index, int face, std::vector<int> &path, std::vector <Intersect> I)
{
  // get first point
  if (path.size() == 0)
    {
      // find an edge point
      for (unsigned i=0; i<I.size(); i++)
	{
	  if ( (I[i].segment[0] == face || I[i].segment[1] == face) &&
	       index != I[i].index )
	    {
	      std::cout << "Start edge: " << i << std::endl;
	      path.push_back(i);
	      return true;
	    }
	}

      // find an inner start point
      for (unsigned i=0; i<I.size(); i++)
	{
	  if ( I[i].plane == face && I[i].index == index)
	    {
	      std::cout << "Start inner: " << i << std::endl;
	      path.push_back(i);
	      return true;
	    }
	}

      // no start point found
      std::cout << "No start found." << std::endl;
      return false;      
    }

  int path_prev = path.back();
  int path_prev_prev = -1;
  std::cout << "Last point: " << path_prev << std::endl;
  if (path.size() > 1)
    {
      path_prev_prev = path[path.size()-2];
      std::cout << "Point before last: " << path_prev_prev << std::endl;
    }
  int plane_cut; // face of cutting plane

  // last point was an inner point
  if (I[path_prev].index == index)
    {
      // no second point, go any direction
      if (path_prev_prev < 0)
	{
	  std::cout << "No second point." << std::endl;
	  plane_cut = I[path_prev].segment[0];
	}
      else
	{
	  // point before last was inner
	  if (I[path_prev_prev].index == index)
	    {
	      // don't get face in last and before last 
	      if ( I[path_prev_prev].segment[0] == I[path_prev].segment[0] ||
		   I[path_prev_prev].segment[1] == I[path_prev].segment[0])
		{
		  plane_cut = I[path_prev].segment[1];
		}
	      else if ( I[path_prev_prev].segment[0] == I[path_prev].segment[1] ||
			I[path_prev_prev].segment[1] == I[path_prev].segment[1])
		{
		  plane_cut = I[path_prev].segment[0];
		}
	      else
		{
		  std::cout << "Error: incorrect iteration of path." << std::endl;
		}
	      
	    }
	  else
	    {
	      if ( I[path_prev_prev].plane == I[path_prev].segment[0] )
		{
		  plane_cut = I[path_prev].segment[1];
		}
	      else if ( I[path_prev_prev].plane == I[path_prev].segment[1] )
		{
		  plane_cut = I[path_prev].segment[0];		  
		}
	      else
		{
		  std::cout << "Error: incorrect iteration of path." << std::endl;
		}
	    }
      
	}
    }
  else
    {
      plane_cut = I[path_prev].plane;
    }
  std::cout << "Cutting plane: " << plane_cut << std::endl;	
  
  // get mid or end point
  for (unsigned i=0; i<I.size(); i++)
    {
      // point exists
      if ( std::find(path.begin(), path.end(), i) != path.end() )
	{
	  continue;
	}
      
      if ( (I[i].segment[0] == plane_cut || I[i].segment[1] == plane_cut ) &&
	   I[i].index == index && I[i].plane == face)
	{
	  std::cout << "Adding mid: " << i << std::endl;
	  path.push_back(i);
	  return true;
	}
    }

  // get end point on edge
  for (unsigned i=0; i<I.size(); i++)
    {
      // point exists
      if ( std::find(path.begin(), path.end(), i) != path.end() )
	{
	  continue;
	}
      
      if ( (I[i].segment[0] == face || I[i].segment[1] == face) &&
	   index != I[i].index && I[i].plane == plane_cut)
	{
	  std::cout << "Adding end: " << i << std::endl;
	  path.push_back(i);
	  return true;
	}
    }

  std::cout << "No point found." << std::endl;
  return false;
  
}

std::vector<std::vector<int>> Intersections::getPaths(int index, int face)
{
  std::vector<std::vector<int>> paths;
  std::vector<int> path;

  bool ret = true;

  while (ret)
    {
      ret = getPathsNext(index, face, path, this->I);
    }

  // one point is no path
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

  std::cout << "Ft: " << std::endl;
  for (int i : Ft)
    {
      std::cout << i << std::endl;
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

  F = gx.F;

  // first set
  for (starti=0; starti<F.rows(); starti++)
    {
      if (! Ft.count(starti))
	{
	  break;
	}
    }
  
  c1 = Calc::connected(F, Ft, starti);

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
      c2 = Calc::connected(F, Ft, starti);
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
  
  std::vector<std::vector<int>> paths = getPaths(index,face);

  std::cout << "divide: " << face << std::endl;
	
  for (std::vector<int> path: paths)
    {
      std::cout << "path:" << std::endl;

      int start = path.front();
      int end = path.back();
      
      MatrixXd P1,P2;
      getPoints(P1);	      
      P2 = P1;
      
      // path from edge to edge
      if ( I[start].index != index && I[end].index != index)
	{
	  // start and end on same edge
	  if (I[start].segment == I[end].segment)
	    {
	      std::cout << "Start end on same edge." << std::endl;
	    }
	  else
	    {
	      std::cout << "Edge to edge" << std::endl;
	      
	      MatrixXd V;
	      MatrixXi F;
	      int sharedVertex, singleVertex1, singleVertex2;
	      unsigned int start1, start2, end1, end2;

	      if (index == 0)
		{
		  V = g1.V;
		  F = g1.F;
		}
	      else
		{
		  V = g2.V;
		  F = g2.F;
		}
	      
	      Calc::toSegment(F, I[start].segment[0], I[start].segment[1],
			      start1, start2);
	      Calc::toSegment(F, I[end].segment[0], I[end].segment[1],
			      end1, end2);
	      
	      if (start1 == end1)
		{
		  sharedVertex = start1;
		  singleVertex1 = end2;
		  singleVertex2 = start2;
		}
	      else if (start1 == end2)
		{
		  sharedVertex = start1;
		  singleVertex1 = end1;
		  singleVertex2 = start2;
		}
	      else if (start2 == end1)
		{
		  sharedVertex = start2;
		  singleVertex1 = end2;
		  singleVertex2 = start1;
		}
	      else if (start2 == end2)
		{
		  sharedVertex = start2;
		  singleVertex1 = end1;
		  singleVertex2 = start1;
		}
	      else
		{
		  std::cout << "Invalid path." << std::endl; 		  
		}
	      	      
	      // path1 is path + shared vertex
	      P1.conservativeResize(P1.rows()+1, NoChange);
	      P1.row(P1.rows()-1) << V.row(sharedVertex);
	      //path + 
	      std::cout << P1 << std::endl;

	      // path2 is path + other 
	      P2.conservativeResize(P2.rows()+2, NoChange);
	      P2.row(P2.rows()-2) << V.row(singleVertex1);
	      P2.row(P2.rows()-1) << V.row(singleVertex2);
	      std::cout << P2 << std::endl;
	    }
	}
      // path inside face
      else if (I[start].index == index && I[end].index == index)
	{
	  std::cout << "Path inside face." << std::endl;

	  //MatrixXd V;
	  //MAtrixXi F;
	  
	  // if (index == 0)
	  //   {
	  //     V = g1.V;
	  //     F = g1.F;
	  //   }
	  // else
	  //   {
	  //     V = g2.V;
	  //     F = g2.F;
	  //   }

	  // find closest point
	  // Vector3d p = V.row(F.row(face)[0]);
	  // int closest;
	  // double distance = I[path[0]];
	    
	  // for (int i : path)
	  //   {
	      
	  //   }
	}
      else
	{
	  std::cout << "Invalid path." << std::endl;
	}
      
      for (int i: path)
	{
	  std::cout << i
		    << " plane: " << this->I[i].plane
		    << ", segment: " << this->I[i].segment.transpose()
		    << " point: " << this->I[i].point.transpose() << std::endl;
	}
    }
}
