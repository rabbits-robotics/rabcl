#ifndef RABCL__CONTROLLER__BODY_POSTURE_HPP_
#define RABCL__CONTROLLER__BODY_POSTURE_HPP_

#include "rabcl/controller/leg_linkage.hpp"

namespace rabcl
{
struct BodyPostureConfig
{
  LegLinkageConfig linkage;  // 脚の 4 節リンクの寸法
  float drop_min;            // [m] 車高 0 のときの車輪の下がり（リンクの範囲内で余裕を取った値。範囲外ならメカストに丸める）
  float drop_max;            // [m] 車高 1
  float tilt_radius;         // [m] 車体中心から車輪の接地点までの水平距離。傾き 1 rad で脚の下がりに付ける差
  float leg_azimuth[4];      // 各脚の方位角 [rad]（車体 x 軸から左回り）。gamma: FR −π/4, FL π/4, BR −3π/4, BL 3π/4
};

// 車高（0〜1）・ロール・ピッチ → 4 本の脚のモータ目標角。脚は円状に配置されている前提
// 車高は車輪の下がり [m] に線形に割り、傾きは tilt_radius × 角度（小角度）を各脚の下がりに足して、
// LegLinkage の逆運動学でモータ角にする（下がりは [drop_min, drop_max] にクランプ）
// ロール・ピッチは視点（カメラ）座標系で受け取り、yaw_offset（首 Yaw の関節角）で車体座標系へ回す
// 座標系は REP-103: ロール正 = 右が下がる（右脚の下がりが小さい）、ピッチ正 = 前が下がる（前脚が小さい）
class BodyPosture
{
public:
  explicit BodyPosture(const BodyPostureConfig & config);
  void Solve(float height, float roll, float pitch, float yaw_offset, float out[4]) const;
  const LegLinkage & linkage() const {return linkage_;}

private:
  BodyPostureConfig config_;
  LegLinkage linkage_;
  float theta_min_ = 0.0f;  // drop_min / drop_max に対応するモータ角（Inverse が解けないときの代用）
  float theta_max_ = 0.0f;
};
}  // namespace rabcl

#endif  // RABCL__CONTROLLER__BODY_POSTURE_HPP_
