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
: config_(config), linkage_(config.linkage)
{
  // 設定の下がりがリンクの範囲（メカスト）を超えていたらメカストに丸める。
  // 丸めないと Inverse が解けず、車高 1 が最低車高に化けるなど黙って誤る
  config_.drop_min = Clamp(config_.drop_min, linkage_.drop_min(), linkage_.drop_max());
  config_.drop_max = Clamp(config_.drop_max, linkage_.drop_min(), linkage_.drop_max());
  theta_min_ = 0.0f;
  theta_max_ = config_.linkage.theta_max;
  linkage_.Inverse(config_.drop_min, theta_min_);
  linkage_.Inverse(config_.drop_max, theta_max_);
}

void BodyPosture::Solve(float height, float roll, float pitch, float yaw_offset, float out[4]) const
{
  const float h = Clamp(height, 0.0f, 1.0f);
  const float base = config_.drop_min + h * (config_.drop_max - config_.drop_min);
  for (int i = 0; i < 4; ++i) {
    // カメラ座標系で見た脚の方位角。傾きベクトル (roll, pitch) に対する脚位置の高さ変化が
    // tilt_radius × (sin(φ)·roll − cos(φ)·pitch)（小角度）。下がる側の脚は縮む（下がりが小さい）
    const float phi = config_.leg_azimuth[i] - yaw_offset;
    const float delta = config_.tilt_radius * (std::sin(phi) * roll - std::cos(phi) * pitch);
    const float drop = Clamp(base + delta, config_.drop_min, config_.drop_max);
    if (!linkage_.Inverse(drop, out[i])) {
      out[i] = (drop - config_.drop_min) < (config_.drop_max - drop) ? theta_min_ : theta_max_;
    }
  }
}
}  // namespace rabcl
