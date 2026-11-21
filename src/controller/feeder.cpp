#include "rabcl/controller/feeder.hpp"

#include <algorithm>
#include <cmath>

namespace rabcl
{

Feeder::Feeder(const FeederConfig & config)
: config_(config)
{
}

int16_t Feeder::Update(Mode mode, float now_s)
{
  const bool held = mode != Mode::STOP;
  if (held && mode != last_mode_) {
    pending_ = true;  // 押下（または向きの切り替え）のエッジ: 1 セルを保留
    pending_mode_ = mode;
  }
  last_mode_ = mode;
  if (held) {
    pending_mode_ = mode;
  }
  const bool due = !stepped_ || (now_s - last_step_s_ + PERIOD_TOLERANCE_S >= config_.period_s);
  if (!(pending_ || held) || !due) {
    return 0;
  }
  const Mode dir = held ? mode : pending_mode_;
  const float delta = dir == Mode::FORWARD ? config_.counts_per_cell : -config_.counts_per_cell;
  target_ += delta;
  const int32_t goal = static_cast<int32_t>(std::lround(target_));
  const int32_t steps = goal - sent_;
  sent_ = goal;
  cells_ += dir == Mode::FORWARD ? 1 : -1;
  last_step_s_ = now_s;
  stepped_ = true;
  pending_ = false;
  return static_cast<int16_t>(std::clamp<int32_t>(steps, INT16_MIN, INT16_MAX));
}

void Feeder::Cancel()
{
  pending_ = false;
}

namespace
{

// 穴からの位相 [0, cell)
float Phase(int32_t position, float phase_offset, float cell)
{
  float phase = std::fmod(static_cast<float>(position) - phase_offset, cell);
  if (phase < 0.0f) {
    phase += cell;
  }
  return phase;
}

}  // namespace

int16_t Feeder::AlignSteps(int32_t position, float phase_offset) const
{
  const float cell = config_.counts_per_cell;
  const float phase = Phase(position, phase_offset, cell);
  const float steps = phase < cell / 2.0f ? -phase : cell - phase;
  return static_cast<int16_t>(std::lround(steps));
}

int16_t Feeder::AlignStepsBackward(int32_t position, float phase_offset, float tolerance) const
{
  const float cell = config_.counts_per_cell;
  const float phase = Phase(position, phase_offset, cell);
  const float steps = cell - phase <= tolerance ? cell - phase : -phase;
  return static_cast<int16_t>(std::lround(steps));
}

}  // namespace rabcl
