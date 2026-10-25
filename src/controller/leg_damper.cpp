#include "rabcl/controller/leg_damper.hpp"

namespace rabcl
{
LegDamper::LegDamper(const LegDamperConfig & config)
: config_(config)
{
}

void LegDamper::Reset(float angle)
{
  ramped_ = angle;
}

MitCommand LegDamper::Update(float target, float angle, float vel, float dt, float load_torque)
{
  (void)vel;  // 減衰はモータ側の kd に任せる
  if (dt > 0.0f) {
    const float step = config_.ramp_rate * dt;
    const float diff = target - ramped_;
    if (diff > step) {
      ramped_ += step;
    } else if (diff < -step) {
      ramped_ -= step;
    } else {
      ramped_ = target;
    }
  }
  float tff = load_torque;
  const float err = ramped_ - angle;
  if (err > config_.deadband) {
    tff += config_.friction_ff;
  } else if (err < -config_.deadband) {
    tff -= config_.friction_ff;
  }
  return {ramped_, 0.0f, config_.kp, config_.kd, tff};
}
}  // namespace rabcl
