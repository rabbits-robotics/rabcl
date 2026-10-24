#include "rabcl/controller/body_posture.hpp"

#include <cmath>

namespace rabcl
{
namespace
{
float Clamp(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

BodyPosture::BodyPosture(const BodyPostureConfig & config)
: config_(config)
{
}

void BodyPosture::Solve(float height, float roll, float pitch, float yaw_offset, float out[4]) const
{
  const float h = Clamp(height, 0.0f, 1.0f);
  const float base = config_.angle_min + h * (config_.angle_max - config_.angle_min);
  for (int i = 0; i < 4; ++i) {
    // カメラ座標系で見た脚の方位角。傾きベクトル (roll, pitch) に対する脚位置の高さ変化が
    // sin(φ)·roll − cos(φ)·pitch（小角度）。下がる側の脚は縮む（角が小さい）
    const float phi = config_.leg_azimuth[i] - yaw_offset;
    const float delta = config_.tilt_gain * (std::sin(phi) * roll - std::cos(phi) * pitch);
    out[i] = Clamp(base + delta, config_.angle_min, config_.angle_max);
  }
}
}  // namespace rabcl
