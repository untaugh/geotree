#pragma once
#include <iostream>

namespace Geotree
{
class Log
{
 public:
  std::ostream& operator<< (std::ostream& stream)
    {
      stream << "Log";
      return stream;
    }


};

 std::ostream& operator<< (std::ostream& stream, Geotree::Log log, const char * msg)
    {
      return stream;
    }
}
