// 脚の 4 節リンク機構の運動学（gamma の脚。設計書 docs/superpowers/specs/2026-10-25-gamma-leg-linkage-design.md）。
//
// 脚 1 本の面内 2 次元。原点 M = 脚モータ軸、x = 半径方向の外向き（水平）、z = 上。
//   M → クランク（長さ Lc、向き α = α0 − θ）→ 先端ピン K → 連結リンク（長さ Lr）→ ホルダ上端ピン J
//   胴体のピン B1 → バー（長さ Lb、角度 φ）→ ホルダのピン H1。ホルダは胴体と平行を保つ（2 本目のバーは同形）
//   J = H1 + coupler_pin、車輪中心 W = H1 + wheel
// θ = モータ角（0 = 原点 = 車高が最も低い、増えると車体が上がる）。
// 拘束 |J − K| = Lr を円の交点で解く。交点は 2 つあり、ホルダが胴体より下にある枝（J の z が低い方）を常に選ぶ。
#ifndef RABCL__CONTROLLER__LEG_LINKAGE_HPP_
#define RABCL__CONTROLLER__LEG_LINKAGE_HPP_

namespace rabcl
{

struct LegLinkageConfig
{
  float crank_length;      // Lc [m]
  float crank_angle_zero;  // α0 [rad]（θ = 0 でのクランクの向き、+x から上向き正）
  float coupler_length;    // Lr [m]
  float bar_length;        // Lb [m]
  float bar_pivot_x;       // B1（M から）[m]
  float bar_pivot_z;
  float coupler_pin_x;     // H1 → J [m]
  float coupler_pin_z;
  float wheel_x;           // H1 → W [m]
  float wheel_z;
  float theta_max;         // モータ角の上限 [rad]（0 が下限）
};

struct LegPose
{
  float bar_angle;      // φ [rad]（+x から上向き正）
  float coupler_angle;  // ψ − α [rad]（クランク先端に付く連結リンク関節の角度。ψ = K → J の向き）
  float wheel_x;        // W.x [m]（M から外向き）
  float wheel_drop;     // d = −W.z [m]（M からの車輪中心の下がり）
};

class LegLinkage
{
public:
  explicit LegLinkage(const LegLinkageConfig & config);
  // θ → 姿勢。円が交わらなければ false で out は変えない
  bool Forward(float theta, LegPose & out) const;
  // 車輪の下がり → θ。[0, theta_max] に解が無ければ false で theta は変えない
  bool Inverse(float wheel_drop, float & theta) const;
  // dd/dθ [m/rad]（中心差分 Δθ = 1e-3）。どちらかが解けなければ 0
  float DropJacobian(float theta) const;
  float drop_min() const {return drop_min_;}  // Forward(0) の d
  float drop_max() const {return drop_max_;}  // Forward(theta_max) の d

private:
  LegLinkageConfig config_;
  float drop_min_ = 0.0f;
  float drop_max_ = 0.0f;
};

}  // namespace rabcl

#endif  // RABCL__CONTROLLER__LEG_LINKAGE_HPP_
