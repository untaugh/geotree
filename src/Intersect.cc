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

  int offset = g1.F.rows();
  
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
	      Vector2i S = S2.row(j);
	      S[0] += offset;
	      S[1] += offset;
	      Intersect is = {i, S, v};
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
	      Intersect is = {i+offset, S1.row(j), v};
	      I.push_back(is);
	    }
	}
    }

  // add face offset for second geometry
  
  // for (unsigned i=0; i< this->I.size(); i++)
  //   {
  //     if (this->I[i].index == 1)
  // 	{
  // 	  this->I[i].plane += offset;
  // 	}
  //     else
  // 	{
  // 	  this->I[i].segment[0] += offset;
  // 	  this->I[i].segment[1] += offset;
  // 	}
  //   }

  // add to gx
  MatrixXd V; 
  MatrixXi F = MatrixXi(0, 3); 
  this->getPoints(V);
  Geometry G_is(V,F); 
    
  this->gx.add(this->g1);
  this->gx.add(this->g2);
  this->gx.add(G_is);
}

void Intersections::add(int plane, Vector2i segment, Vector3d point)
{
  Intersect is = {plane, segment, point};

  this->I.push_back(is);
}

bool Intersections::getPathsNext(int face, std::vector<int> &path, std::vector <Intersect> I)
{
  // get first point
  if (path.size() == 0)
    {
      // find an edge point
      for (unsigned i=0; i<I.size(); i++)
	{
	  if ( (I[i].segment[0] == face || I[i].segment[1] == face) )
	    {
	      std::cout << "Start edge: " << i << std::endl;
	      path.push_back(i);
	      return true;
	    }
	}

      // find an inner start point
      for (unsigned i=0; i<I.size(); i++)
	{
	  if ( I[i].plane == face )
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
  std::cout << "Previous point: " << path_prev << std::endl;
  if (path.size() > 1)
    {
      path_prev_prev = path[path.size()-2];
      std::cout << "Point before previous: " << path_prev_prev << std::endl;
    }
  int plane_cut; // face of cutting plane

  // last point was an inner point
  if (I[path_prev].plane == face)
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
	  if (I[path_prev_prev].plane == face)
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
	   I[i].plane == face)
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
	   I[i].plane == plane_cut)
	{
	  std::cout << "Adding end: " << i << std::endl;
	  path.push_back(i);
	  return true;
	}
    }

  std::cout << "No point found." << std::endl;
  return false;
  
}

std::vector<std::vector<int>> Intersections::getPaths(int face)
{
  std::vector<std::vector<int>> paths;
  std::vector<int> path;

  bool ret = true;
  
  while (ret)
    {
      ret = getPathsNext(face, path, this->I);
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
  int n = this->I.size();
  points = Eigen::MatrixXd(n, 3);

  for (int i=0; i<n; i++)
    {
      points.row(i) = this->I[i].point;
    }
}

// void Intersections::divide(int index, int face, MatrixXi &F1, MatrixXi &F2)
// {

//   int c = 0;
//   for (Intersect i: I )
//     {
//       std::cout << c++;
//       std::cout << " index: " << i.index;
//       std::cout << ", plane: " << i.plane;
//       std::cout << ", segment: " << i.segment.transpose();
//       std::cout << ", point: " << i.point.transpose() << std::endl;
//     }
  
//   std::vector<std::vector<int>> paths = getPaths(index,face);

//   std::cout << "divide: " << face << std::endl;
	
//   for (std::vector<int> path: paths)
//     {
//       std::cout << "path:" << std::endl;

//       int start = path.front();
//       int end = path.back();
      
//       MatrixXd P1,P2;
//       getPoints(P1);	      
//       P2 = P1;
      
//       // path from edge to edge
//       if ( I[start].index != index && I[end].index != index)
// 	{
// 	  // start and end on same edge
// 	  if (I[start].segment == I[end].segment)
// 	    {
// 	      std::cout << "Start end on same edge." << std::endl;
// 	    }
// 	  else
// 	    {
// 	      std::cout << "Edge to edge" << std::endl;
	      
// 	      MatrixXd V;
// 	      MatrixXi F;
// 	      int sharedVertex, singleVertex1, singleVertex2;
// 	      unsigned int start1, start2, end1, end2;

// 	      if (index == 0)
// 		{
// 		  V = g1.V;
// 		  F = g1.F;
// 		}
// 	      else
// 		{
// 		  V = g2.V;
// 		  F = g2.F;
// 		}
	      
// 	      Calc::toSegment(F, I[start].segment[0], I[start].segment[1],
// 			      start1, start2);
// 	      Calc::toSegment(F, I[end].segment[0], I[end].segment[1],
// 			      end1, end2);
	      
// 	      if (start1 == end1)
// 		{
// 		  sharedVertex = start1;
// 		  singleVertex1 = end2;
// 		  singleVertex2 = start2;
// 		}
// 	      else if (start1 == end2)
// 		{
// 		  sharedVertex = start1;
// 		  singleVertex1 = end1;
// 		  singleVertex2 = start2;
// 		}
// 	      else if (start2 == end1)
// 		{
// 		  sharedVertex = start2;
// 		  singleVertex1 = end2;
// 		  singleVertex2 = start1;
// 		}
// 	      else if (start2 == end2)
// 		{
// 		  sharedVertex = start2;
// 		  singleVertex1 = end1;
// 		  singleVertex2 = start1;
// 		}
// 	      else
// 		{
// 		  std::cout << "Invalid path." << std::endl; 		  
// 		}
	      	      
// 	      // path1 is path + shared vertex
// 	      P1.conservativeResize(P1.rows()+1, NoChange);
// 	      P1.row(P1.rows()-1) << V.row(sharedVertex);
// 	      //path + 
// 	      std::cout << P1 << std::endl;

// 	      // path2 is path + other 
// 	      P2.conservativeResize(P2.rows()+2, NoChange);
// 	      P2.row(P2.rows()-2) << V.row(singleVertex1);
// 	      P2.row(P2.rows()-1) << V.row(singleVertex2);
// 	      std::cout << P2 << std::endl;
// 	    }
// 	}
//       // path inside face
//       else if (I[start].index == index && I[end].index == index)
// 	{
// 	  std::cout << "Path inside face." << std::endl;

// 	  //MatrixXd V;
// 	  //MAtrixXi F;
	  
// 	  // if (index == 0)
// 	  //   {
// 	  //     V = g1.V;
// 	  //     F = g1.F;
// 	  //   }
// 	  // else
// 	  //   {
// 	  //     V = g2.V;
// 	  //     F = g2.F;
// 	  //   }

// 	  // find closest point
// 	  // Vector3d p = V.row(F.row(face)[0]);
// 	  // int closest;
// 	  // double distance = I[path[0]];
	    
// 	  // for (int i : path)
// 	  //   {
	      
// 	  //   }
// 	}
//       else
// 	{
// 	  std::cout << "Invalid path." << std::endl;
// 	}
      
//       for (int i: path)
// 	{
// 	  std::cout << i
// 		    << " plane: " << this->I[i].plane
// 		    << ", segment: " << this->I[i].segment.transpose()
// 		    << " point: " << this->I[i].point.transpose() << std::endl;
// 	}
//     }
// }

bool Intersections::getIntersectingFaces(std::set <unsigned> &faces)
{
  faces.clear();

  for (Intersect i : this->I)
    {
      faces.insert(i.plane);
      faces.insert(i.segment[0]);
      faces.insert(i.segment[1]);
    }

  return true;  
}

void Intersections::divide(int face, std::set <unsigned> div1, std::set <unsigned> div2)
{
  std::vector<std::vector<int>> paths = getPaths(face);

  int offset = this->g1.V.rows() + this->g2.V.rows();
  
  std::cout << "Face " << face << ": ";
  
  // for (int i : paths[0])
  //   {
  //     std::cout << i+offset << ", ";
  //   }
  // std::cout << std::endl;

  for ( std::vector<int> path : paths)
    {
      // paths
      std::vector <int> P1, P2;
	      
      Intersect * start = &this->I[path.front()];
      Intersect * end = &this->I[path.back()];

      if ( start->plane == face && end->plane == face )
	{
	  std::cout << "Path lies inside face" << std::endl;

	  // find closest point to first point in face
	  Vector3d v = this->gx.V.row(this->gx.F.row(face)[0]);
	  int near_point = path.front();
	  double d = (v - this->I[near_point].point).norm();

	  for (int i=0; i<path.size(); i++)
	    {
	      double d_new = (v - this->I[i].point).norm();

	      if (d_new < d)
		{
		  near_point = i;
		}
	    }
	  std::cout << "Nearest point " << near_point + offset << "." << std::endl;

	  // create paths
	  for (int i: path)
	    {
	      P1.push_back(i+offset);
	      P2.push_back(i+offset);

	      if (i == near_point)
		{
		  P2.push_back(this->gx.F.row(face)[0]);
		  P2.push_back(this->gx.F.row(face)[1]);
		  P2.push_back(this->gx.F.row(face)[2]);
		  P2.push_back(this->gx.F.row(face)[0]);
		  P2.push_back(i+offset);
		}
	    }
	}
      else if ( (start->segment[0] == face || start->segment[1] == face) &&
		(end->segment[0] == face || end->segment[1] == face) )
	{
	  if (start->segment == end->segment)
	    {
	      std::cout << "Path starts and ends on same edge.." << std::endl;
	    }
	  else
	    {
	      std::cout << "Path goes from edge to edge." << std::endl;

	      int sharedVertex, singleVertex1, singleVertex2; 
	      unsigned int ps1, ps2, pe1, pe2;
	      
	      Calc::toSegment(this->gx.F, start->segment[0], start->segment[1],
			      ps1, ps2);
	      Calc::toSegment(this->gx.F, end->segment[0], end->segment[1],
			      pe1, pe2);

	      if (ps1 == pe1)
		{
		  sharedVertex = ps1;
		  singleVertex1 = pe2;
		  singleVertex2 = ps2;
		}
	      else if (ps1 == pe2)
		{
		  sharedVertex = ps1;
		  singleVertex1 = pe1;
		  singleVertex2 = ps2;
		}
	      else if (ps2 == pe1)
		{
		  sharedVertex = ps2;
		  singleVertex1 = pe2;
		  singleVertex2 = ps1;
		}
	      else if (ps2 == pe2)
		{
		  sharedVertex = ps2;
		  singleVertex1 = pe1;
		  singleVertex2 = ps1;
		}
	      else
		{
		  std::cout << "Invalid path." << std::endl; 		  
		}

	      for (int i: path)
		{
		  P1.push_back(i+offset);
		  P2.push_back(i+offset);
		}

	      P1.push_back(sharedVertex);
	      P2.push_back(singleVertex1);
	      P2.push_back(singleVertex2);

	    }
	}
      else
	{
	  std::cout << "Invalid path." << std::endl;	  
	}
      std::cout << "V: " << this->gx.V << std::endl;
      
      std::cout << "P1: ";
      for (int i : P1) std::cout << i << ", ";
      std::cout << std::endl;

      std::cout << "P2: ";
      for (int i : P2) std::cout << i << ", ";
      std::cout << std::endl;

      Eigen::Map<const RowVectorXi> Pv1(P1.data(), P1.size());
      Eigen::Map<const RowVectorXi> Pv2(P2.data(), P2.size());

      std::cout << "Pv1: " << Pv1 << std::endl;
      std::cout << "Pv2: " << Pv2 << std::endl;      
      //std::cout << "gxV: " << this->gx.V << std::endl;      
      
      MatrixXi F1, F2;

      
      
      Calc::triangulate(this->gx.V, Pv1, F1);
      Calc::triangulate(this->gx.V, Pv2, F2);

      std::cout << "F1: " << F1 << std::endl;
      std::cout << "F2: " << F2 << std::endl;
      //Eigen::Map<const Vector3i, 1, 3> Pv(pi, 1,3);

      ///      Eigen::VectorXi Pv1(P1.data());
      //Eigen::VectorXi Pv2(P2.data()); 
      
    }  
}
