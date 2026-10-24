#ifndef RABCL__CONTROLLER__BODY_POSTURE_HPP_
#define RABCL__CONTROLLER__BODY_POSTURE_HPP_

namespace rabcl
{
struct BodyPostureConfig
{
  float angle_min;       // 脚の使用範囲 [rad]（メカストの余裕込み）。0 側が最低車高
  float angle_max;
  float tilt_gain;       // [rad/rad] 車体の傾き 1 rad あたり、脚の取り付け半径の位置で脚角に付ける差
  float leg_azimuth[4];  // 各脚の方位角 [rad]（車体 x 軸から左回り）。gamma: FR −π/4, FL π/4, BR −3π/4, BL 3π/4
};

// 車高（0〜1）・ロール・ピッチ → 4 本の脚の目標角。脚は円状に配置されている前提
// ロール・ピッチは視点（カメラ）座標系で受け取り、yaw_offset（首 Yaw の関節角）で車体座標系へ回す
// 座標系は REP-103: ロール正 = 右が下がる（右脚が縮む）、ピッチ正 = 前が下がる（前脚が縮む）
class BodyPosture
{
public:
  explicit BodyPosture(const BodyPostureConfig & config);
  void Solve(float height, float roll, float pitch, float yaw_offset, float out[4]) const;

private:
  BodyPostureConfig config_;
};
}  // namespace rabcl

#endif  // RABCL__CONTROLLER__BODY_POSTURE_HPP_
