#pragma once

namespace Geotree {
  
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
