#ifndef RABCL__CONTROLLER__LEG_DAMPER_HPP_
#define RABCL__CONTROLLER__LEG_DAMPER_HPP_

namespace rabcl
{
struct LegDamperConfig
{
  float kp;           // [Nm/rad] MIT の剛性（弱くして柔らかく）
  float kd;           // [Nm·s/rad] MIT の減衰
  float load_ff;      // [Nm] 荷重分のフィードフォワード（車体質量の分担をレバーで換算）
  float friction_ff;  // [Nm] 摩擦分。目標と現在角の差の向きに足す
  float deadband;     // [rad] 差がこの範囲内なら摩擦分を足さない
  float ramp_rate;    // [rad/s] 目標の移動速度の上限
};

// DAMIAO の MIT 指令の中身
struct MitCommand
{
  float pos;  // [rad]
  float vel;  // [rad/s]
  float kp;
  float kd;
  float tff;  // [Nm]
};

// 脚 1 本のアクティブダンパ: 目標角をランプで追いかけ、弱い kp / kd + tff の MIT 指令を作る
class LegDamper
{
public:
  explicit LegDamper(const LegDamperConfig & config);
  // dt [s] ごとに呼ぶ。dt <= 0 なら目標は動かさない
  MitCommand Update(float target, float angle, float vel, float dt);
  void Reset(float angle);  // 目標を現在角に合わせる（起動時、復帰時）
  float ramped_target() const {return ramped_;}

private:
  LegDamperConfig config_;
  float ramped_ = 0.0f;
};
}  // namespace rabcl

#endif  // RABCL__CONTROLLER__LEG_DAMPER_HPP_
