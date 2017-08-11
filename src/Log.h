#include <sstream>

enum LOG_LEVEL {
  LOG_ERROR,
  LOG_INFO,
  LOG_DEBUG
};

namespace Geotree {

  class Log
  {
  public:
    Log();
    ~Log();
    std::ostringstream& Get(LOG_LEVEL level = LOG_INFO);
    LOG_LEVEL messageLevel;
  protected:
    std::ostringstream os;
  };
}
