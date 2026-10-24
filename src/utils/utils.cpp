#include "rabcl/utils/utils.hpp"

namespace rabcl
{
void Utils::DegToRad(double deg, double & rad)
{
  rad = deg * M_PI / 180.0;
}

double Utils::DegToRad(double deg)
{
  double rad;
  DegToRad(deg, rad);
  return rad;
}

void Utils::RadToDeg(double rad, double & deg)
{
  deg = rad * 180.0 / M_PI;
}

double Utils::RadToDeg(double rad)
{
  double deg;
  RadToDeg(rad, deg);
  return deg;
}

double Utils::ShortestPathMultiTurn(double current, double target, double full_rotation)
{
  double half = full_rotation / 2.0;
  double target_mod = std::fmod(target, full_rotation);
  if (target_mod < 0.0) {
    target_mod += full_rotation;
  }
  double current_mod = std::fmod(current, full_rotation);
  if (current_mod < 0.0) {
    current_mod += full_rotation;
  }
  double diff = target_mod - current_mod;
  if (diff > half) {
    diff -= full_rotation;
  }
  if (diff < -half) {
    diff += full_rotation;
  }
  return current + diff;
}
static uint32_t xorshift_state_ = 1;

float Utils::WrapPi(float angle)
{
  const float two_pi = 2.0f * static_cast<float>(M_PI);
  return angle - two_pi * std::floor((angle + static_cast<float>(M_PI)) / two_pi);
}

void Utils::SetRandomSeed(uint32_t seed)
{
  xorshift_state_ = (seed == 0) ? 1 : seed;
}

uint32_t Utils::Random()
{
  xorshift_state_ ^= xorshift_state_ << 13;
  xorshift_state_ ^= xorshift_state_ >> 17;
  xorshift_state_ ^= xorshift_state_ << 5;
  return xorshift_state_;
}

float Utils::RandomFloat(float min, float max)
{
  return min + (Random() % 10000) / 10000.0f * (max - min);
}
}  // namespace rabcl
