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

    if (level < LOG_VERBOSE)
      {
	fprintf(stderr, "%s", os.str().c_str());
      }
    fflush(stderr);
  }

  std::ostringstream& Log::debug()
  {
    return Get(LOG_DEBUG);    
  }

  std::ostringstream& Log::Get(LOG_LEVEL level)
  {
    //std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    switch (level)
      {
      case LOG_DEBUG: break;
	os << "ERROR: ";
	break;
      case LOG_INFO: break;
      case LOG_VERBOSE: break;
      case LOG_ERROR:
      case LOG_WARNING:
	{
	  os << "ERROR: ";
	  break;
	}
      }

    //    os << "- " << std::s(&end_time);
    this->level = level;
    return os;
  }

  std::ostringstream& operator<< (Geotree::Log &log, const char * m)
  {
    std::cout << m << std::endl;
		      return log.os;
  }
}
