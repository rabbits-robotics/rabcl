#include "rabcl/controller/leg_linkage.hpp"

#include <cmath>

#include "rabcl/utils/utils.hpp"

namespace rabcl
{
namespace
{
struct Vec2
{
  float x;
  float z;
};

// 中心 c0 半径 r0 と中心 c1 半径 r1 の円の交点 2 つ。交わらなければ false
bool CircleIntersections(Vec2 c0, float r0, Vec2 c1, float r1, Vec2 & p0, Vec2 & p1)
{
  const float dx = c1.x - c0.x;
  const float dz = c1.z - c0.z;
  const float d = std::sqrt(dx * dx + dz * dz);
  if (d < 1e-9f || d > r0 + r1 || d < std::fabs(r0 - r1)) {
    return false;
  }
  const float a = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
  const float h2 = r0 * r0 - a * a;
  const float h = h2 > 0.0f ? std::sqrt(h2) : 0.0f;
  const float px = c0.x + a * dx / d;
  const float pz = c0.z + a * dz / d;
  p0 = {px - h * dz / d, pz + h * dx / d};
  p1 = {px + h * dz / d, pz - h * dx / d};
  return true;
}
}  // namespace

LegLinkage::LegLinkage(const LegLinkageConfig & config)
: config_(config)
{
  LegPose p;
  if (Forward(0.0f, p)) {
    drop_min_ = p.wheel_drop;
  }
  if (Forward(config_.theta_max, p)) {
    drop_max_ = p.wheel_drop;
  }
}

bool LegLinkage::Forward(float theta, LegPose & out) const
{
  const float alpha = config_.crank_angle_zero - theta;
  const Vec2 k = {config_.crank_length * std::cos(alpha), config_.crank_length * std::sin(alpha)};
  const Vec2 c = {config_.bar_pivot_x + config_.coupler_pin_x,
    config_.bar_pivot_z + config_.coupler_pin_z};
  Vec2 j0, j1;
  if (!CircleIntersections(c, config_.bar_length, k, config_.coupler_length, j0, j1)) {
    return false;
  }
  const Vec2 j = j0.z < j1.z ? j0 : j1;  // ホルダが胴体より下にある枝
  const float phi = std::atan2(j.z - c.z, j.x - c.x);
  const float h1x = config_.bar_pivot_x + config_.bar_length * std::cos(phi);
  const float h1z = config_.bar_pivot_z + config_.bar_length * std::sin(phi);
  const float psi = std::atan2(j.z - k.z, j.x - k.x);
  out.bar_angle = phi;
  out.coupler_angle = Utils::WrapPi(psi - alpha);
  out.wheel_x = h1x + config_.wheel_x;
  out.wheel_drop = -(h1z + config_.wheel_z);
  return true;
}

bool LegLinkage::Inverse(float wheel_drop, float & theta) const
{
  const float s = (-wheel_drop - config_.bar_pivot_z - config_.wheel_z) / config_.bar_length;
  if (s < -1.0f || s > 1.0f) {
    return false;
  }
  const float phi = std::asin(s);  // cos φ > 0（バーは外向き）
  const Vec2 j = {
    config_.bar_pivot_x + config_.coupler_pin_x + config_.bar_length * std::cos(phi),
    config_.bar_pivot_z + config_.coupler_pin_z + config_.bar_length * std::sin(phi)};
  Vec2 k0, k1;
  if (!CircleIntersections({0.0f, 0.0f}, config_.crank_length, j, config_.coupler_length, k0, k1)) {
    return false;
  }
  bool found = false;
  float best = 0.0f;
  const Vec2 candidates[2] = {k0, k1};
  for (const Vec2 & k : candidates) {
    const float alpha = std::atan2(k.z, k.x);
    const float t = Utils::WrapPi(config_.crank_angle_zero - alpha);
    if (t < -1e-6f || t > config_.theta_max + 1e-6f || (found && t >= best)) {
      continue;
    }
    LegPose check;  // 上の枝（ホルダが胴体より上）の解を除く: 順運動学で同じ下がりに戻ること
    if (!Forward(t, check) || std::fabs(check.wheel_drop - wheel_drop) > 1e-5f) {
      continue;
    }
    best = t;
    found = true;
  }
  if (!found) {
    return false;
  }
  theta = best < 0.0f ? 0.0f : best;
  return true;
}

float LegLinkage::DropJacobian(float theta) const
{
  constexpr float DELTA = 1e-3f;
  LegPose lo, hi;
  if (!Forward(theta - DELTA, lo) || !Forward(theta + DELTA, hi)) {
    return 0.0f;
  }
  return (hi.wheel_drop - lo.wheel_drop) / (2.0f * DELTA);
}

}  // namespace rabcl
