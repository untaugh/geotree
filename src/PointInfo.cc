#include "IntersectionPoint.h"
#include "Calc.h"
#include <iostream>

namespace Geotree
{
  bool PointInfo::isSegment()
  {
    if (type == SEGMENT)
      {
	return true;
      }
    else
      {
	return false;
      }
  }

  SegmentIndex PointInfo::getSegment()
  {
    SegmentIndex segment(index, index2);
    return segment;
    //return index2.block(0,0,2,1);
  }
  
  void PointInfo::setFaces(Mesh mesh)
  {
    faces = mesh.getFaces(*this);
  }

  // void PointInfo::setFaceIndex(Mesh mesh)
  // {
  //   faceIndex = Faces(faces.size(), 3);
  //   int i = 0;
  //   for (int f : faces)
  //     {
  // 	faceIndex.row(i++) = mesh.F.row(f);
  //     }
  // }

  FaceSet PointInfo::getFaces(Mesh mesh)
  {
    FaceSet faces;
    
    if (type == FACE)
      {
	//int i = mesh.getFaceIndex(index2);
	faces.insert(index);
      }
    else if (type == SEGMENT)
      {
	//SegmentIndex segment = index2.block(0,0,2,1);
	//faces = mesh.getFacesWithSegment(segment);
	faces = mesh.getFacesWithSegment(getSegment());
      }
    else if (type == POINT)
      {
	faces = mesh.getFacesWithPoint(index);
      }

    return faces;
  }

  bool PointInfo::isConnected(PointInfo point)
  {
    for (int f : point.faces)
      {
	if (faces.count(f))
	  {
	    return true;
	  }
      }
    return false;
  }
  
  // bool PointInfo::isConnected2(PointInfo point)
  // {
  //   switch(point.type)
  //     {
  //     case FACE: return hasFace(point.index2);
  //     case POINT: return hasPoint(point.index2[0]);
  //     case SEGMENT:
  // 	{
  // 	  switch (type)
  // 	    {
  // 	    case SEGMENT:
  // 	      {
  // 		if (point.hasFace(faceIndex.row(0)) ||
  // 		    point.hasFace(faceIndex.row(1)))
  // 		  {
  // 		    return true;
  // 		  }
  // 		else
  // 		  {
  // 		    return false;
  // 		  }
  // 	      }
  // 	    default: return hasSegment(point.getSegment());
  // 	    }
  // 	}
  //     }
  // }
  
  // bool PointInfo::hasSegment(SegmentIndex segment)
  // {
  //   switch(type)
  //     {
  //     case FACE:
  // 	{
  // 	  if (hasPoint(segment[0]) && hasPoint(segment[1]))
  // 	    {
  // 	      return true;
  // 	    }
  // 	  break;
  // 	}
  //     case POINT:
  //     case SEGMENT:
  // 	{
  // 	  if (hasPoint(segment[0]) || hasPoint(segment[1]))
  // 	    {
  // 	      return true;
  // 	    }
  // 	  break;
  // 	}
  //     }

  //   return false;       
  // }

  // bool PointInfo::hasFace(FaceIndex face)
  // {
  //   switch(type)
  //     {
  //     case POINT:
  // 	{
  // 	  if (face[0] == index2[0] || face[1] == index2[0] || face[2] == index2[0])
  // 	    {
  // 	      return true;
  // 	    }
  // 	  break;
  // 	}
  //     case FACE:
  // 	{
  // 	  if (Calc::equal(face,index2))
  // 	    {
  // 	    return true;
  // 	    }
  // 	  break;
  // 	}      
  //     case SEGMENT:
  // 	{
  // 	  if (Calc::hasSegment(face, getSegment()))
  // 	  {
  // 	    return true;
  // 	  }
  // 	  break;
  // 	}
  //     }
  //   return false;
  // }

  // bool PointInfo::hasPoint(int point)
  // {
  //   switch(type)
  //     {
  //     case FACE:
  // 	{
  // 	  if (index2[2] == point)
  // 	    {
  // 	      return true;
  // 	    }
  // 	}
  //     case SEGMENT:
  // 	{
  // 	  if (index2[1] == point)
  // 	    {
  // 	      return true;
  // 	    }
  // 	  }
  //     case POINT:
  // 	{
  // 	  if (index2[0] == point)
  // 	    {
  // 	      return true;
  // 	    }
  // 	}	
  //     }
  //   return false;
  // }

  bool PointInfo::operator==(const PointInfo &point) const
  {
    if (this->index == point.index && this->index2 == point.index2 && this->type == point.type)
      {
	return true;
      }
    else
      {
	return false;
      }
  }
  
}


