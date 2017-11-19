#include "Face.h"
#include "Calc.h"
#include <iostream>

namespace Geotree
{
  FaceT::FaceT(Mesh &_mesh, const int _index)
  : mesh(_mesh), index(_index)
  {
  }

  // FaceT::FaceT(FaceT face)
  // : mesh(face.mesh), index(face.index)
  // {
  // }

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
	return mesh.F.row(index)(0);
      }
    else if (Calc::closeToZero((point - face.row(1).transpose()).norm()))
      {
	return mesh.F.row(index)(1);
      }
    else if (Calc::closeToZero((point - face.row(2).transpose()).norm()))
      {
	return mesh.F.row(index)(2);
      }
    else
      {
	return -1;
      }
  }

  SegmentIndex FaceT::getIndex(int faceindex)
  {
    SegmentIndex segment;
    
    segment[0] = mesh.F.row(index)(faceindex%3);
    segment[1] = mesh.F.row(index)((faceindex+1)%3);
    
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
    PointInfo pointInfo;

    Line segment1 = getVectors(0);
    Line segment2 = getVectors(1);
    Line segment3 = getVectors(2);
    
    if (hasPoint(point))
      {
	pointInfo.index = getPointIndex(point);	
	pointInfo.type = POINT;
      }
    else if (Calc::closeToZero(Calc::distance(point, segment1)))
      {
	SegmentIndex si = getIndex(0);
	pointInfo.index = si(0);
	pointInfo.index2 = si(1);
	pointInfo.type = SEGMENT;
      }
    else if (Calc::closeToZero(Calc::distance(point, segment2)))
      {
	SegmentIndex si = getIndex(1);
	pointInfo.index = si(0);
	pointInfo.index2 = si(1);
	pointInfo.type = SEGMENT;
      }
    else if (Calc::closeToZero(Calc::distance(point, segment3)))
      {
	SegmentIndex si = getIndex(2);
	pointInfo.index = si(0);
	pointInfo.index2 = si(1);
	pointInfo.type = SEGMENT;
      }
    else
      {
	pointInfo.index = index;
	pointInfo.type = FACE;
      }

    return pointInfo;
  }

  PointInfo SegmentT::getType(Vector3d point)
  {
    Line segment = getVectors();
    PointInfo pointInfo;
	
    if (Calc::closeToZero((point - segment.row(0).transpose()).norm()))
      {
	pointInfo.index = index[0];
	pointInfo.type = POINT;
      }
    else if (Calc::closeToZero((point - segment.row(1).transpose()).norm()))
      {
	pointInfo.index = index[1];
	pointInfo.type = POINT;
      }
    else
      {
	pointInfo.index = index[0];
	pointInfo.index2 = index[1];
	pointInfo.type = SEGMENT;
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
    return mesh.getFaceVectors(index);
  }

  Line FaceT::getVectors(int segmentindex)
  {
    Plane face = mesh.getFaceVectors(index);
    Line segment;
    segment.row(0) = face.row(segmentindex%3);
    segment.row(1) = face.row((segmentindex+1)%3);
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

    IntersectionPoint ipoint(mesh, first, second, point);

    return ipoint;
  }
}
