#include "rabcl/controller/body_attitude.hpp"

#include "rabcl/utils/utils.hpp"

namespace rabcl
{

void BodyAttitude::UpdateFromImu(const Attitude & imu, const NeckAngles & joint_at_imu_time)
{
  if (yaw_tracking_) {
    if (anchor_pending_) {
      AnchorImu(imu, joint_at_imu_time);
      anchor_pending_ = false;
    }
    body_.yaw = Utils::WrapPi(imu.yaw + imu_yaw_offset_ - joint_at_imu_time.yaw);
  }
  body_.pitch = imu.pitch - joint_at_imu_time.pitch;
  body_.roll = imu.roll - joint_at_imu_time.roll;
}

void BodyAttitude::AnchorImu(const Attitude & imu, const NeckAngles & joint_at_imu_time)
{
  imu_yaw_offset_ = Utils::WrapPi(body_.yaw - (imu.yaw - joint_at_imu_time.yaw));
}

void BodyAttitude::UpdateFromChassisRate(float wz, float dt)
{
  if (yaw_tracking_) {
    body_.yaw = Utils::WrapPi(body_.yaw + wz * dt);
  }
}

void BodyAttitude::SetYawTracking(bool enabled)
{
  if (enabled && !yaw_tracking_) {
    anchor_pending_ = true;
  }
  yaw_tracking_ = enabled;
}

bool BodyAttitude::yaw_tracking() const
{
  return yaw_tracking_;
}

Attitude BodyAttitude::Get() const
{
  return body_;
}

void BodyAttitude::Reset()
{
  body_ = {0.0f, 0.0f, 0.0f};
  imu_yaw_offset_ = 0.0f;
  anchor_pending_ = false;
}

}  // namespace rabcl
