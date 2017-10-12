#include "Face.h"
#include "Calc.h"

namespace Geotree
{
  FaceT::FaceT(const Verticies &_verticies, const Vector3i _points)
    : verticies(_verticies), points(_points)
  {
  }

  bool FaceT::hasPoint(Vector3d point)
  {
    Plane face = getVectors();
    return Calc::isEndPoint(face, point);
  }

  int FaceT::getPointIndex(Vector3d point)
  {
    Plane face = getVectors();
    if (Calc::closeToZero((point - face.row(0).transpose()).norm()))
      {
	return points[0];
      }
    else if (Calc::closeToZero((point - face.row(1).transpose()).norm()))
      {
	return points[1];
      }
    else if (Calc::closeToZero((point - face.row(2).transpose()).norm()))
      {
	return points[2];
      }
    else
      {
	return -1;
      }
  }

  SegmentIndex FaceT::getIndex(int index)
  {
    SegmentIndex segment;
    
    segment[0] = points[index%3];
    segment[1] = points[(index+1)%3];

    if (segment[0] > segment[1])
      {
	int temp = segment[0];
	segment[0] = segment[1];
	segment[1] = temp;
      }

    return segment;
  }
  
  PointInfo FaceT::getType(Vector3d point)
  {
    Plane face = getVectors();

    Line segment1 = getVectors(0);
    Line segment2 = getVectors(1);
    Line segment3 = getVectors(2);

    PointInfo pointInfo;

    pointInfo.index2 << 0,0,0;

    if (hasPoint(point))
      {
	pointInfo.index2[0] = getPointIndex(point);
	pointInfo.type = PointInfo::POINT;
	//return POINT_ON_POINT;
      }
    else if (Calc::closeToZero(Calc::distance(point, segment1)))
      {
	//pointInfo.index2[0] = points[0];
	//pointInfo.index2[1] = points[1];
	pointInfo.index2.block(0,0,2,1) = getIndex(0);
	pointInfo.type = PointInfo::SEGMENT;
	//return POINT_ON_SEGMENT;
      }
    else if (Calc::closeToZero(Calc::distance(point, segment2)))
      {
	//pointInfo.index2[0] = points[1];
	//pointInfo.index2[1] = points[2];
	pointInfo.index2.block(0,0,2,1) = getIndex(1);
	pointInfo.type = PointInfo::SEGMENT;
	//return POINT_ON_SEGMENT;
      }
    else if (Calc::closeToZero(Calc::distance(point, segment3)))
      {
	pointInfo.index2.block(0,0,2,1) = getIndex(2);
	//pointInfo.index2[0] = points[0];
	//pointInfo.index2[1] = points[2];
	pointInfo.type = PointInfo::SEGMENT;
	//return POINT_ON_SEGMENT;
      }
    else
      {
	pointInfo.index2 = points;
	pointInfo.type = PointInfo::FACE;
	//return POINT_ON_FACE;
      }

    return pointInfo;
  }

  PointInfo SegmentT::getType(Vector3d point)
  {
    Line segment = getVectors();
    PointInfo pointInfo;

    pointInfo.index2 << 0,0,0;
	
    if (Calc::closeToZero((point - segment.row(0).transpose()).norm()))
      {
	pointInfo.index2[0] = index[0];
	pointInfo.type = PointInfo::POINT;
      }
    else if (Calc::closeToZero((point - segment.row(1).transpose()).norm()))
      {
	pointInfo.index2[0] = index[1];
	pointInfo.type = PointInfo::POINT;
      }
    else
      {
	pointInfo.index2[0] = index[0];
	pointInfo.index2[1] = index[1];
	pointInfo.type = PointInfo::SEGMENT;
	//return POINT_ON_SEGMENT;
      }
    return pointInfo;
  }
  
  SegmentT::SegmentT(const Verticies &_verticies, const Vector2i _points)
    : verticies(_verticies), index(_points)
  {
  }

  bool FaceT::intersects(const Line segment)
  {
    Plane face = getVectors();
    return Calc::intersectsFace(face, segment);
  }


  Plane FaceT::getVectors()
  {
    Plane face;
    face.row(0) = verticies.row(points[0]);
    face.row(1) = verticies.row(points[1]);
    face.row(2) = verticies.row(points[2]);
    return face;
  }

  Line FaceT::getVectors(int index)
  {
    Line segment;
    segment.row(0) = verticies.row(points[index%3]);
    segment.row(1) = verticies.row(points[(index+1)%3]);
    return segment;
  }

  Line SegmentT::getVectors()
  {
    Line segment;
    segment.row(0) = this->verticies.row(index[0]);
    segment.row(1) = this->verticies.row(index[1]);
    return segment;
  }
  
  IntersectionPoint FaceT::getIntersection(SegmentT segment)
  {
    
    Line seg = segment.getVectors();
    Plane face = this->getVectors();
    Vector point = Calc::intersection(face, seg);

    PointInfo first = this->getType(point);
    PointInfo second = segment.getType(point);
    //PointType second = segment.getType(point);
    //PointInfo info;
    IntersectionPoint ipoint(first, second, point);
    return ipoint;
  }
}
