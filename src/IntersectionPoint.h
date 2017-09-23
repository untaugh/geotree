#pragma once
#include "Types.h"
#include <utility>

namespace Geotree
{

class PointType
{
public:    
  enum Type {
      FACE,
      SEGMENT,
      POINT
    } type;

  PointType(const Type _type, const int _index) {  this->type = _type; this->index = _index; }

  int index;
};

/* class PointTypeFace : public PointType */
/* { */
/* public: */
/*  PointTypeFace() : PointType(FACE) {}; */
/*   FaceIndex face; */
/* }; */

/* class PointTypeSegment : public PointType */
/* { */
/*  public:   */
/*  PointTypeSegment() : PointType(SEGMENT) {}; */
/*   SegmentIndex segment; */
/* }; */

/* class PointTypePoint : public PointType */
/* { */
/* public: */
/*  PointTypePoint() : PointType(POINT) {}; */
/*   int point; */
/* }; */  

 class IntersectionPoint
 {
 public:
 IntersectionPoint(PointType firstType, PointType secondType, Vector _point)
   : type(firstType, secondType), point(_point) {};
   
 private:
   std::pair <PointType, PointType> type;
   Vector point;
 };
 
 class MeshIntersection
 {
 public:
   void addPoint(IntersectionPoint _point){};
   std::vector <IntersectionPoint> points;
 };
 
}
