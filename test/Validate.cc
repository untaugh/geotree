#include "Validate.h"

bool Geotree::Validate::face(Matrix3d f)
{
  if (f.row(0) == f.row(1) ||
      f.row(0) == f.row(2) ||
      f.row(1) == f.row(2))
    {
      return false;
    }

  return true;
}
