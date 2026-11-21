#include "rabcl/controller/jam_recovery.hpp"

#include <cmath>
#include <cstdlib>

namespace rabcl
{

JamRecovery::JamRecovery(const JamRecoveryConfig & config)
: config_(config)
{
}

bool JamRecovery::Stalled(int16_t remaining, int16_t load, int16_t current)
{
  const bool heavy = !Arrived(remaining) &&
    (std::abs(load) >= config_.load_limit || std::abs(current) >= config_.current_limit);
  stall_count_ = heavy ? static_cast<uint8_t>(stall_count_ + 1) : 0;
  if (stall_count_ < config_.stall_reads) {
    return false;
  }
  stall_count_ = 0;
  return true;
}

bool JamRecovery::Arrived(int16_t remaining) const
{
  return std::abs(remaining) <= config_.settle_counts;
}

int16_t JamRecovery::Limit() const
{
  return static_cast<int16_t>(std::lround(config_.queue_limit_cells * config_.counts_per_cell));
}

JamRecovery::Action JamRecovery::Observe(
  int16_t remaining, int16_t load, int16_t current, uint8_t error, float now_s)
{
  Action a = {};
  if ((error & ERR_OVERLOAD) != 0) {
    // 保護が入るとトルク 20 % で動けない。トルクを切って再接続してもらう（Reset / ResumeAfterReconnect まで留まる）
    a.block_forward = true;
    if (state_ != State::RECONNECT) {
      a.reconnect = true;
      ++tries_;
      state_ = State::RECONNECT;
    }
    return a;
  }
  if (state_ == State::NORMAL) {
    const bool stalled = Stalled(remaining, load, current);
    if (stalled && remaining > 0) {
      const int k = static_cast<int>(std::floor(remaining / config_.counts_per_cell)) + 1;
      back_counts_ = static_cast<int16_t>(std::lround(k * config_.counts_per_cell));
      a.move = static_cast<int16_t>(-back_counts_);
      a.block_forward = true;
      state_ = State::BACKING;
      started_s_ = now_s;
      return a;
    }
    a.block_forward = remaining > Limit();
    a.block_reverse = remaining < -Limit() || (stalled && remaining < 0);
    // 詰まらずに 1 セル送れた（動いて → 到達）ら試行回数を 0 に。合わせ直した穴で静止しているだけでは戻さない
    if (remaining > config_.settle_counts) {
      cell_in_flight_ = true;
    } else if (cell_in_flight_ && Arrived(remaining)) {
      cell_in_flight_ = false;
      tries_ = 0;
    }
    return a;
  }
  a.block_forward = true;
  if (state_ == State::BACKING) {
    if (Arrived(remaining)) {
      a.move = back_counts_;
      stall_count_ = 0;
      state_ = State::RETRYING;
      started_s_ = now_s;
    } else if (now_s - started_s_ > config_.timeout_s) {
      // 後ろにも動けない。目標が前にあるまま JAMMED に入ると押し続けて保護に落ちる（実機 2026-11-21）ので
      // トルクを切って位相合わせからやり直してもらう
      ++jams_;
      ++tries_;
      a.reconnect = true;
      state_ = State::RECONNECT;
    }
  } else if (state_ == State::RETRYING) {
    const bool stalled = Stalled(remaining, load, current);
    if (stalled || now_s - started_s_ > config_.timeout_s) {
      ++tries_;
      a.move = static_cast<int16_t>(-back_counts_);  // あきらめるときも戻してから（押し続けない）
      stall_count_ = 0;
      if (tries_ >= config_.retry_limit) {
        ++jams_;
        state_ = State::JAMMED;
        started_s_ = now_s;
      } else {
        state_ = State::BACKING;
        started_s_ = now_s;
      }
    } else if (Arrived(remaining)) {
      ++recoveries_;
      tries_ = 0;
      a.block_forward = false;
      state_ = State::NORMAL;
    }
  } else if (state_ == State::JAMMED) {
    if (Arrived(remaining)) {
      started_s_ = now_s;  // 戻し切っていれば引っ張っていない。操作者の逆転を待つ
    } else if (now_s - started_s_ > config_.timeout_s) {
      a.reconnect = true;  // 最後の戻しも効かない。引っ張り続けない
      ++tries_;
      state_ = State::RECONNECT;
    }
  }
  return a;
}

void JamRecovery::NotifyReverse()
{
  if (state_ == State::BACKING || state_ == State::RETRYING || state_ == State::JAMMED) {
    Reset();
  }
}

void JamRecovery::Reset()
{
  state_ = State::NORMAL;
  tries_ = 0;
  stall_count_ = 0;
  cell_in_flight_ = false;
}

void JamRecovery::ResumeAfterReconnect(float now_s)
{
  stall_count_ = 0;
  cell_in_flight_ = false;
  if (tries_ >= config_.retry_limit) {
    ++jams_;
    state_ = State::JAMMED;
    started_s_ = now_s;  // 後ろ合わせの Move がまだ動いている。到達を timeout_s 待つ
  } else {
    state_ = State::NORMAL;
  }
}

}  // namespace rabcl
