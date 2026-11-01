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
  float ratio;  // −1 〜 +1
  if (config_.friction_band > 0.0f) {
    ratio = err / config_.friction_band;
    ratio = ratio > 1.0f ? 1.0f : (ratio < -1.0f ? -1.0f : ratio);
  } else {
    ratio = err > 0.0f ? 1.0f : (err < 0.0f ? -1.0f : 0.0f);
  }
  tff += config_.friction_ff * ratio;
  return {ramped_, 0.0f, config_.kp, config_.kd, tff};
}
}  // namespace rabcl
