#pragma once

namespace Geotree {

  class EdgePoint2
  {
  public:
    EdgePoint2(int path, int step, int segment, double position)
      : path(path), step(step), position(position), segment(segment), trianglePoint(false) {};

	  EdgePoint2(int triangleIndex) : segment(triangleIndex), trianglePoint(true) {};
    int path;
    int step;
    double position;
    int segment;

	bool trianglePoint;

    bool operator <(const EdgePoint2 point)
    const {
      if (segment < point.segment)
	{
	  return true;
	}
      else if (segment > point.segment)
	{
	  return false;
	}
      else
	{
	  if (this->position < point.position)
	    {
	      return true;
	    }
	  else
	    {
	      return false;
	    }
	}
    }
  };

  struct EdgePoint {
    PointType type;
    int index;
    double position;
    SegmentIndex segment;

    bool operator <(const EdgePoint otherPoint)
    const {
      if (this->segment[0] < otherPoint.segment[0])
      	{
      	  return true;
      	}
      else if (this->segment[0] > otherPoint.segment[0])
      	{
      	  return false;
      	}
      else
	{
	  if (this->segment[1] < otherPoint.segment[1])
	    {
	      return true;
	    }
	  else if (this->segment[1] > otherPoint.segment[1])
	    {
	      return false;	      
	    }
	  else if (this->position < otherPoint.position)
	    {
	      return true;
	    }
	  else
	    {
	      return false;
	    }
	}
    }
  };

}
