#include "Intersect.h"
#include <iostream>

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
  
  // switch (index)
  //   {
  //   case 0: this->I1.push_back(is); break;
  //   case 1: this->I2.push_back(is); break;
  //   default: break;
  //   }
}

std::vector<std::set<unsigned>> Intersections::getPaths(int index, int face)
{
  // std::vector<Intersect> *A = &I1;
  // std::vector<Intersect> *B = &I2;

  // switch (index)
  //   {
  //   case 0:
  //     A = &I1;
  //     B = &I2;
  //     break;
  //   case 1:
  //     B = &I1;
  //     A = &I2;
  //     break;
  //   default:
  //     break;
  //   }
  
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
