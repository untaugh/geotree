#include "FaceDivide.h"
#include <iostream>
#include <Eigen/Dense>

namespace Geotree
{
  FaceDivide::FaceDivide(const Vector2d p0, const Vector2d p1, const Vector2d p2)
  {
    V = Matrix <double, Dynamic, 2> (3, 2);

    V.row(0) = p0;
    V.row(1) = p1;
    V.row(2) = p2;

    edgepoints.insert({0,0,0.0});
    edgepoints.insert({1,1,0.0});
    edgepoints.insert({2,2,0.0});
  }

  edgepoint FaceDivide::getEdgepoint(int index)
  {
    edgepoint point;

    point.index = index;

    for (int i = 0; i<3; i++)
    {
      Vector2d p0 = (V.row((i+1)%3) - V.row(i));
      Vector2d p1 = (V.row(index) - V.row(i));

      if (p0[0]/p0[1] == p1[0]/p1[1])
      {
        point.segment = i;
        point.distance = (V.row(i) - V.row(index)).norm();
        return point;
      }
    }

    std::cout << "Edge point not found " << index << " " << V.row(index) << std::endl;

    exit(0);
  }

  const edgepoint FaceDivide::findEdgepoint(unsigned index)
  {
    for (const edgepoint &ep : edgepoints)
    {
      if (ep.index == index)
      {
        return ep;
      }
    }

    std::cout << "findEdgepoint: Edge point not found " << index << std::endl;
  }

  const edgepoint FaceDivide::nextEdgepoint(edgepoint point)
  {
    std::set<Geotree::edgepoint>::iterator findpoint = edgepoints.find(point);

    findpoint++;

    if (findpoint == edgepoints.end())
    {
      findpoint = edgepoints.begin();
    }

    std::cout << "point: "<< point.index << std::endl;
    std::cout << "next: "<< findpoint->index << std::endl;

    return *findpoint;
  }

  void FaceDivide::divide()
  {
    for (std::vector <int> path : cutpaths)
    {
      std::vector <int> path0(path);
      std::vector <int> path1(path.rbegin(), path.rend());

      edgepoint first = getEdgepoint(path0[0]);
      edgepoint last = getEdgepoint(path0[path0.size()-1]);

      for (edgepoint current = nextEdgepoint(last); current != first; current = nextEdgepoint(current))
      {
        path0.push_back(current.index);
      }

      for (edgepoint current = nextEdgepoint(first); current != last; current = nextEdgepoint(current))
      {
        path1.push_back(current.index);
      }

      for (int i : path0)
      {
        std::cout << "p0 " << i << std::endl;
      }

      for (int i : path1)
      {
        std::cout << "p1 " << i << std::endl;
      }

      completepaths.push_back(path0);
      completepaths.push_back(path1);
    }
  }

  void FaceDivide::triangulate()
  {
    for (std::vector <int> path : completepaths)
    {
      
    }
  }

  void FaceDivide::cut(Matrix <double, Dynamic, 2> path)
  {
    std::vector <int> cutpath;

    int firstindex = V.rows();
    int lastindex = firstindex + path.rows() - 1;

    V.conservativeResize(firstindex + path.rows(),2);
    V.block(firstindex,0, path.rows(), 2) = path;

    edgepoint first = getEdgepoint(firstindex);
    edgepoint last = getEdgepoint(lastindex);

    edgepoints.insert(first);
    edgepoints.insert(last);

    for (edgepoint ep : edgepoints)
    {
      std::cout << ep.index << ": " << ep.segment << ", " << ep.distance << std::endl;
    }

    for(int i=firstindex; i<=lastindex; i++)
    {
      cutpath.push_back(i);
    }

    cutpaths.push_back(cutpath);
  }
}
