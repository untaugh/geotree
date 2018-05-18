#include "Segment.h"
#include <Eigen/Dense>
#include "Log.h"

namespace Geotree
{
  Matrix2d Segment::get2d(Axis &skip) const
  {
    Matrix2d m;

    Log().debug() << "get2d skip: " << skip;
    
    switch (skip)
      {
      case X:
	{
	  m.row(0) << begin[Y], begin[Z];
	  m.row(1) << end[Y], end[Z];
	  break;
	}
      case Y:
	{
	  m.row(0) << begin[X], begin[Z];
	  m.row(1) << end[X], end[Z];
	  break;
	}
      case Z:
	{
	  m.row(0) << begin[X], begin[Y];
	  m.row(1) << end[X], end[Y];
	  break;
	}
      }

    Log().debug() << "  m: " << m;
    
    return m;
  }

  bool Segment::intersects(Vector3d point)
  {
    Vector3d cross = (end-begin).cross(point-begin);

    return (cross == Vector3d(0,0,0));
  }

  bool Segment::intersects(const Segment segment, Vector3d &point) const
  {
    Axis skip;
    
    if ((this->length(X) > 0 || this->length(Y) > 0)
	&& (segment.length(X) > 0 || segment.length(Y) > 0))
      {
	skip = Z;	
      }
    else if ((this->length(X) > 0 || this->length(Z) > 0)
	&& (segment.length(X) > 0 || segment.length(Z) > 0))
      {
	skip = Y;
      }
    else if ((this->length(Y) > 0 || this->length(Z) > 0)
	&& (segment.length(Y) > 0 || segment.length(Z) > 0))
      {
	skip = X;
      }
    else
      {
	throw;
      }

    const Matrix2d segment0_2d = this->get2d(skip);
    const Matrix2d segment1_2d = segment.get2d(skip);
    
    const Vector2d U = segment0_2d.row(1) - segment0_2d.row(0);
    const Vector2d V = segment1_2d.row(1) - segment1_2d.row(0);
    const Vector2d W = segment0_2d.row(0) - segment1_2d.row(0);

    Log().debug() << "  U: " << U.transpose()
		  << ", V: " << V.transpose()
		  << ", W:" << W.transpose();
    
#define perp(u,v) (u[0] * v[1] - u[1] * v[0])

    const double D = perp(U,V);

    Log().debug() << "  D: " << D;
    
    if (D == 0)
      {
	return false;
      }

    const double s1 = perp(V,W) / D;

    Log().debug() << "  s1: " << s1;
    
    if (s1 > 1.0 || s1 < 0.0)
    {
      return false;
    }

    double s2 = perp(U,W) / D;

    Log().debug() << "  s2: " << s2;

    if (s2 > 1.0 || s2 < 0.0)
    {
      return false;
    }

    point = (begin + s1 * (end - begin));
    Vector3d otherpoint = (segment.begin + s2 * (segment.end - segment.begin));

    Log().debug() << "  point: " << point.transpose();
    Log().debug() << "  otherpoint: " << otherpoint.transpose();

    if (point != otherpoint)
      {
	return false;
      }

    return true;
  }

  double Segment::length(const Axis axis) const
  {
    return abs(begin[axis] - end[axis]);
  }
}
