#include "rabcl/controller/command_watchdog.hpp"

namespace rabcl
{

CommandWatchdog::CommandWatchdog(float timeout_s)
: timeout_s_(timeout_s)
{
}

void CommandWatchdog::Feed(float now_s)
{
  last_fed_s_ = now_s;
  fed_ = true;
}

bool CommandWatchdog::Expired(float now_s) const
{
  if (!fed_) {
    return true;
  }
  return (now_s - last_fed_s_) > timeout_s_;
}

}  // namespace rabcl
