#include "rabcl/controller/body_attitude.hpp"

#include "rabcl/utils/utils.hpp"

namespace rabcl
{

void BodyAttitude::UpdateFromImu(const Attitude & imu, const NeckAngles & joint_at_imu_time)
{
  body_.yaw = Utils::WrapPi(imu.yaw - joint_at_imu_time.yaw);
  body_.pitch = imu.pitch - joint_at_imu_time.pitch;
  body_.roll = imu.roll - joint_at_imu_time.roll;
}

void BodyAttitude::UpdateFromChassisRate(float wz, float dt)
{
  body_.yaw = Utils::WrapPi(body_.yaw + wz * dt);
}

Attitude BodyAttitude::Get() const
{
  return body_;
}

void BodyAttitude::Reset()
{
  body_ = {0.0f, 0.0f, 0.0f};
}

}  // namespace rabcl
