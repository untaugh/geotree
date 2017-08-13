#include "Log.h"
#include <chrono>
#include <ctime>
#include <iostream>

namespace Geotree {

  Log::Log()
  {

  }

  Log::~Log()
  {
    os << std::endl;
    fprintf(stderr, "%s", os.str().c_str());
    fflush(stderr);
  }
  
  std::ostringstream& Log::Get(LOG_LEVEL level)
  {
    std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    //    os << "- " << std::s(&end_time);

    return os;
  }

}
