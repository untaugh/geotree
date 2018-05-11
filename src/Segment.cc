#include "Segment.h"
#include <Eigen/Dense>
#include "Log.h"

namespace Geotree
{
  Matrix2d Segment::get2d() const
  {
    // double dx = vectors.row(1)[0] - vectors.row(0)[0];
    // double dz = vectors.row(1)[2] - vectors.row(0)[2];
    //
    // dx = dx > 0 ? dx : -dx;
    // dz = dz > 0 ? dz : -dz;
    Matrix2d m;
    m.row(0) << begin[0], begin[1];
    m.row(1) << end[0], end[1];
    return m;

    // if (i)
    // {
    //   return vectors.block(0,1,2,2);
    // }
    // else
    // {
    //   return vectors.block(0,0,2,2);
    // }
  }

  bool Segment::intersects(Vector3d point)
  {
    Vector3d cross = (end-begin).cross(point-begin);

    //std::cout << "cross " << cross.transpose() << ":"<< (cross == Vector3d(0,0,0)) << std::endl;

    return (cross == Vector3d(0,0,0));
  }

  bool Segment::intersects(const Segment segment, Vector3d &point) const
  {
    const Matrix2d segment0_2d = this->get2d();
    const Matrix2d segment1_2d = segment.get2d();

    const Vector2d U = segment0_2d.row(1) - segment0_2d.row(0);
    const Vector2d V = segment1_2d.row(1) - segment1_2d.row(0);
    const Vector2d W = segment0_2d.row(0) - segment1_2d.row(0);

#define perp(u,v) (u[0] * v[1] - u[1] * v[0])

    const double D = perp(U,V);
    const double s1 = perp(V,W) / D;

    log::debug << "afgd";
        
    if (s1 > 1.0 || s1 < 0.0)
    {
      return false;
    }

    double s2 = perp(U,W) / D;

    //std::cout << "s2: " << s2 << ", s0:" << segment0_2d << "s1: "<< segment1_2d << std::endl;

    if (s2 > 1.0 || s2 < 0.0)
    {
      return false;
    }

    point = (begin + s1 * (end - begin));

    return true;
  }
}
