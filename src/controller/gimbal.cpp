#include "rabcl/controller/gimbal.hpp"

#include "rabcl/utils/utils.hpp"

namespace rabcl
{

NeckAngles Gimbal::Solve(const Attitude & desired, const Attitude & body) const
{
  NeckAngles joint;
  joint.yaw = Utils::WrapPi(desired.yaw - body.yaw);
  joint.pitch = desired.pitch - body.pitch;
  joint.roll = desired.roll - body.roll;
  return joint;
}

}  // namespace rabcl
