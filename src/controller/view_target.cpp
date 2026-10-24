#include "rabcl/controller/view_target.hpp"

#include <algorithm>

#include "rabcl/utils/utils.hpp"

namespace rabcl
{

void ViewTarget::AddDelta(float dyaw, float dpitch)
{
  target_.yaw = Utils::WrapPi(target_.yaw + dyaw);
  target_.pitch = std::clamp(target_.pitch + dpitch, pitch_min_, pitch_max_);
}

void ViewTarget::SetPitchLimits(float min, float max)
{
  pitch_min_ = min;
  pitch_max_ = max;
  target_.pitch = std::clamp(target_.pitch, pitch_min_, pitch_max_);
}

Attitude ViewTarget::Get() const
{
  return target_;
}

void ViewTarget::Reset(const Attitude & target)
{
  target_.yaw = Utils::WrapPi(target.yaw);
  target_.pitch = std::clamp(target.pitch, pitch_min_, pitch_max_);
  target_.roll = target.roll;
}

}  // namespace rabcl
