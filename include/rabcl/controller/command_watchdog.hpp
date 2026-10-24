// 指令途絶の監視: 最後に Feed された時刻から timeout_s 以上経てば Expired
#ifndef RABCL__CONTROLLER__COMMAND_WATCHDOG_HPP_
#define RABCL__CONTROLLER__COMMAND_WATCHDOG_HPP_

namespace rabcl
{

class CommandWatchdog
{
public:
  explicit CommandWatchdog(float timeout_s);
  void Feed(float now_s);
  // 一度も Feed されていなければ true
  bool Expired(float now_s) const;

private:
  float timeout_s_;
  float last_fed_s_ = 0.0f;
  bool fed_ = false;
};

}  // namespace rabcl

#endif  // RABCL__CONTROLLER__COMMAND_WATCHDOG_HPP_
