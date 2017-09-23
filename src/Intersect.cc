#include "Intersect.h"
#include "Calc.h"
#include "Validate.h"
#include <algorithm>
#include <iostream>
#include <Log.h>

void Intersections::add(Geometry &g1, Geometry &g2)
{

  if (!Geotree::Validate::geometry(g1) || !Geotree::Validate::geometry(g2))
    {
      return;
    }

  this->g1 = g1;
  this->g2 = g2;

  // get segments
  MatrixXi S1, S2;
  Calc::getFaceSegments(g1.F, S1);
  Calc::getFaceSegments(g2.F, S2);

  int offset = g1.F.rows();
  
  for (int i=0; i<g1.F.rows(); i++)
    {
      for (int j=0; j<S2.rows(); j++)
	{
	  // face points
	  MatrixXd F = MatrixXd(3,3);
	  F.row(0) = g1.V.row(g1.F.row(i)[0]);
	  F.row(1) = g1.V.row(g1.F.row(i)[1]);
	  F.row(2) = g1.V.row(g1.F.row(i)[2]);

	  // segment points
	  MatrixXd S = MatrixXd(2,3);
	  unsigned int n1,n2;
	  Calc::toSegment(g2.F, S2.row(j)[0], S2.row(j)[1], n1, n2);
	  S.row(0) = g2.V.row(n1);
	  S.row(1) = g2.V.row(n2);

	  Vertex v;
	  
	  if ( Calc::intersectsFace(F, S) )
	    {
	      v = Calc::intersection(F,S);
	      Vector2i S = S2.row(j);
	      S[0] += offset;
	      S[1] += offset;
	      Intersect is = {i, S, v};
	      I.push_back(is);
	    }
	}	  
    }

  for (int i=0; i<g2.F.rows(); i++)
    {
      for (int j=0; j<S1.rows(); j++)
	{
	  // face points
	  MatrixXd F = MatrixXd(3,3);
	  F.row(0) = g2.V.row(g2.F.row(i)[0]);
	  F.row(1) = g2.V.row(g2.F.row(i)[1]);
	  F.row(2) = g2.V.row(g2.F.row(i)[2]);

	  // segment points
	  MatrixXd S = MatrixXd(2,3);
	  unsigned int n1,n2;
	  Calc::toSegment(g1.F, S1.row(j)[0], S1.row(j)[1], n1, n2);
	  S.row(0) = g1.V.row(n1);
	  S.row(1) = g1.V.row(n2);
	  
	  Vertex v;
	  
	  if ( Calc::intersectsFace(F, S) )
	    {
	      v = Calc::intersection(F,S);
	      Intersect is = {i+offset, S1.row(j), v};
	      I.push_back(is);
	    }
	}
    }
    
  this->gx.add(this->g1);
  this->gx.add(this->g2);
  this->gx.addVerts( this->getPoints() );
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
	      path.push_back(i);
	      return true;
	    }
	}

      // find an inner start point
      for (unsigned i=0; i<I.size(); i++)
	{
	  if ( I[i].plane == face )
	    {
	      path.push_back(i);
	      return true;
	    }
	}

      // no start point found
      return false;      
    }

  int path_prev = path.back();
  int path_prev_prev = -1;

  if (path.size() > 1)
    {
      path_prev_prev = path[path.size()-2];
    }
  
  int plane_cut; // face of cutting plane

  // last point was an inner point
  if (I[path_prev].plane == face)
    {
      // no second point, go any direction
      if (path_prev_prev < 0)
	{
	  // no second point
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
	  // adding mid
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
	  // adding end
	  path.push_back(i);
	  return true;
	}
    }

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

  // log path
  Geotree::Log * log = new Geotree::Log();

  log->Get(LOG_DEBUG) << "getPaths: ";

  for (int i: path)
    {
      log->Get(LOG_DEBUG) << i << ", ";      
    }
    
  delete log;
  
  // one point is no path
  if (path.size() > 1)
    {
      paths.push_back(path);
    }
  
  return paths;
}

Verticies Intersections::getPoints()
{
  Verticies points(this->I.size(), 3);

  for (int i=0; i<points.rows(); i++)
    {
      points.row(i) = this->I[i].point;
    }

  return points;
}

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

bool Intersections::getFaceGroups(std::vector <std::set <unsigned>> &facelists)
{
  std::set <unsigned> skip;

  // skip intersecting faces
  if ( ! getIntersectingFaces(skip))
    {
      return false;
    }

  for (int i=0; i<this->gx.F.rows(); i++ )
    {
      std::set <unsigned> faces;
      
      faces = Calc::connected(this->gx.F, skip, i);

      if (! faces.size())
	{
	  continue;
	}
      
      skip.insert(faces.begin(), faces.end());
      facelists.push_back(faces);      
    }
	
  return true;
}

void Intersections::divide()
{
  std::set <unsigned> facesIntersecting;

  std::vector <std::set <unsigned>> facelists;
  std::vector <std::set <unsigned>> facelists_add;

  getFaceGroups(facelists);
  
  getFaceGroups(facelists_add);
	
  getIntersectingFaces(facesIntersecting);

  for (unsigned f : facesIntersecting)
    {
      std::set <unsigned> div1, div2;

      divide(f, div1, div2);

      //for (unsigned i=0; i<facelists.size(); i++)
      int i = 0;

      bool div1add = false;
      bool div2add = false;

      for (FaceSet faces : facelists)
	{	    
	  if (Calc::connectedPoint(gx.F, faces, div1))
	    {
	      std::cout << f << " Adding faces1: " << i << std::endl;
	      facelists_add[i].insert(div1.begin(), div1.end());
	      div1add = true;
	    }
	  else if (Calc::connectedPoint(gx.F, faces, div2))
	    {
	      std::cout << f << " Adding faces2: " << i << std::endl;
	      facelists_add[i].insert(div2.begin(), div2.end());
	      div2add = true;
	    }
	  else
	    {
	      std::cout << f << "no connect found: " << i << std::endl;
	    }
	  i++;
	}
      
      if (! div1add)
	{
	  facelists_add.push_back(div1);
	}
      
      if (! div2add)
	{
	  facelists_add.push_back(div2);
	}
    }


  // for (int i=0; i<gx.V.rows(); i++)
  //   {
  //     std::cout << i << ": " << gx.V.row(i) << std::endl;
  //   }
  
  // for (int i=0; i<gx.F.rows(); i++)
  //   {
  //     std::cout << i << ": " << gx.F.row(i) << std::endl;
  //   }

    
  std::cout << "Facelists:";
  
  
  for (FaceSet f : facelists_add)
    {
      for (unsigned i : f)
	  {
	    std::cout << i << ", ";

	  }
      std::cout << std::endl;

	for (unsigned i : f)
	  {
	    for (int j=0; j<3; j++)
	      {
		Vertex point = gx.V.row(gx.F.row(i)[j]);

		if (Calc::inside(g1, point))
		  {
		    std::cout << "#";
		  }
		else
		  {
		    std::cout << "*";
		  }
	      }
	  }	
	std::cout << std::endl;

	for (unsigned i : f)
	  {
	    for (int j=0; j<3; j++)
	      {
		Vertex point = gx.V.row(gx.F.row(i)[j]);

		if (Calc::inside(g2, point))
		  {
		    std::cout << "%";
		  }
		else
		  {
		    std::cout << "~";
		  }
	      }
	  }	
	std::cout << std::endl;
      }

  FaceSet facesInA = facelists_add[3];
  FaceSet facesOutA = facelists_add[0];
  FaceSet facesInB = facelists_add[1];
  FaceSet facesOutB = facelists_add[2];

  // create union
  Faces FOutA(facesOutA.size(),3);
  int j = 0;
  for (int i : facesOutA)
    {
      FOutA.row(j++) = gx.F.row(i);
    }

  Faces FOutB(facesOutB.size(),3);
  j = 0;
  for (int i : facesOutB)
    {
      FOutB.row(j++) = gx.F.row(i);
    }

  Faces FInB(facesInB.size(),3);
  j = 0;
  for (int i : facesInB)
    {
      FInB.row(j++) = gx.F.row(i);
    }
  
  CSGUnion = Geometry(gx.V, FOutA);
  CSGUnion.addFaces(FOutB);

  CSGDifference = Geometry(gx.V, FOutA);
  CSGDifference.addFaces(FInB);
  
}

void Intersections::divide(int face, std::set <unsigned> &div1, std::set <unsigned> &div2)
{
  std::vector<std::vector<int>> paths = getPaths(face);

  int offset = this->g1.V.rows() + this->g2.V.rows();
  
  Geotree::Log().Get(LOG_DEBUG) << "divide: " << face << " offset:" << offset;

  for ( std::vector<int> path : paths)
    {
      // paths
      std::vector <int> P1, P2;
	      
      Intersect * start = &this->I[path.front()];
      Intersect * end = &this->I[path.back()];

      if ( start->plane == face && end->plane == face )
	{
	  Geotree::Log().Get(LOG_DEBUG) << " Inside face.";
	  // find closest point to first point in face
	  Vector3d v = this->gx.V.row(this->gx.F.row(face)[0]);
	  int near_point = path.front();
	  double d = (v - this->I[near_point].point).norm();

	  for (unsigned int i=0; i<path.size(); i++)
	    {
	      double d_new = (v - this->I[i].point).norm();

	      if (d_new < d)
		{
		  near_point = i;
		}
	    }

	  // create paths
	  for (int i: path)
	    {
	      P1.push_back(i+offset);
	      P2.push_back(i+offset);

	      if (i == near_point)
		{
		  P2.push_back(this->gx.F.row(face)[0]);
		  P2.push_back(this->gx.F.row(face)[2]);
		  P2.push_back(this->gx.F.row(face)[1]);
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
	      Geotree::Log().Get(LOG_DEBUG) << " Same edge.";
	    }
	  else
	    {
	      Geotree::Log().Get(LOG_DEBUG) << " Eedge to edge.";

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
		  Geotree::Log().Get(LOG_ERROR) << "Invalid path.";
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
	  Geotree::Log().Get(LOG_ERROR) << "Invalid path.";
	}

      Geotree::Log * log = new Geotree::Log();

      log->Get(LOG_DEBUG) << " path1: ";

      for (int i : P1)
	{
	  log->Get(LOG_DEBUG) << i << ", ";
	}

      log->Get(LOG_DEBUG) << std::endl << " path2: ";

      for (int i : P2)
	{
	  log->Get(LOG_DEBUG) << i << ", ";
	}

      delete log;

      Eigen::Map<const RowVectorXi> Pv1(P1.data(), P1.size());
      Eigen::Map<const RowVectorXi> Pv2(P2.data(), P2.size());
      
      Faces F1, F2;
      
      Calc::triangulate(this->gx.V, Pv1, F1);
      Calc::triangulate(this->gx.V, Pv2, F2);

      this->gx.addFaces(F1);

      for (int i = this->gx.F.rows() - F1.rows(); i < this->gx.F.rows(); i++)
	{
	  div1.insert(i);
	}

      this->gx.addFaces(F2);
	    
      for (int i = this->gx.F.rows() - F2.rows(); i < this->gx.F.rows(); i++)
	{
	  div2.insert(i);
	}
      
    }  
}
