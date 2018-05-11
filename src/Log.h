#pragma once
#include <sstream>

enum LOG_LEVEL {
  LOG_ERROR,
  LOG_WARNING,
  LOG_INFO,
  LOG_DEBUG,
  LOG_VERBOSE
};

namespace Geotree {
 
  class Log
  {
  public:
    Log();
  Log(LOG_LEVEL level) : level(level){};
    ~Log();
    std::ostringstream& debug();
    std::ostringstream& Get(LOG_LEVEL level = LOG_INFO);
    LOG_LEVEL level;
    //protected:
    std::ostringstream os;
  };

  namespace log
  {
    static Log debug(LOG_DEBUG);
    static Log error;
  }
  
  std::ostringstream& operator<< (Geotree::Log &log, const char * m);
}
