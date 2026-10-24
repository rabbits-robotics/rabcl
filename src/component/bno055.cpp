#include "rabcl/component/bno055.hpp"

namespace rabcl
{
BNO055::BNO055() {}
BNO055::~BNO055() {}

void BNO055::UpdateAccel(const uint8_t raw[6], ImuInfo & imu)
{
  imu.acc_x_ = static_cast<int16_t>((raw[1] << 8) | raw[0]) * ACC_SCALE;
  imu.acc_y_ = static_cast<int16_t>((raw[3] << 8) | raw[2]) * ACC_SCALE;
  imu.acc_z_ = static_cast<int16_t>((raw[5] << 8) | raw[4]) * ACC_SCALE;
}

void BNO055::UpdateGyro(const uint8_t raw[6], ImuInfo & imu)
{
  imu.gyro_x_ = static_cast<int16_t>((raw[1] << 8) | raw[0]) * GYRO_SCALE;
  imu.gyro_y_ = static_cast<int16_t>((raw[3] << 8) | raw[2]) * GYRO_SCALE;
  imu.gyro_z_ = static_cast<int16_t>((raw[5] << 8) | raw[4]) * GYRO_SCALE;
}

void BNO055::UpdateEuler(const uint8_t raw[6], ImuInfo & imu)
{
  imu.euler_heading_ = static_cast<int16_t>((raw[1] << 8) | raw[0]) * EULER_SCALE;
  imu.euler_roll_ = static_cast<int16_t>((raw[3] << 8) | raw[2]) * EULER_SCALE;
  imu.euler_pitch_ = static_cast<int16_t>((raw[5] << 8) | raw[4]) * EULER_SCALE;
}
}  // namespace rabcl

// --- BNO055Uart ---

namespace rabcl
{
size_t BNO055Uart::BuildRead(uint8_t reg, uint8_t len, uint8_t * out)
{
  out[0] = START;
  out[1] = 0x01;
  out[2] = reg;
  out[3] = len;
  return 4;
}

size_t BNO055Uart::BuildWrite(uint8_t reg, const uint8_t * data, uint8_t len, uint8_t * out)
{
  out[0] = START;
  out[1] = 0x00;
  out[2] = reg;
  out[3] = len;
  for (uint8_t i = 0; i < len; ++i) {
    out[4 + i] = data[i];
  }
  return 4 + static_cast<size_t>(len);
}

bool BNO055Uart::Push(uint8_t byte, Response & out)
{
  switch (state_) {
    case State::HEAD:
      if (byte == RESPONSE) {
        state_ = State::DATA_LEN;
      } else if (byte == ACK) {
        state_ = State::STATUS;
      }
      return false;
    case State::DATA_LEN:
      if (byte == 0 || byte > MAX_READ) {
        state_ = State::HEAD;  // あり得ない長さ。先頭探しに戻る
        return false;
      }
      len_ = byte;
      pos_ = 0;
      state_ = State::DATA;
      return false;
    case State::DATA:
      buf_[pos_++] = byte;
      if (pos_ < len_) {
        return false;
      }
      out.is_data = true;
      out.len = len_;
      for (uint8_t i = 0; i < len_; ++i) {
        out.data[i] = buf_[i];
      }
      state_ = State::HEAD;
      return true;
    case State::STATUS:
      out.is_data = false;
      out.len = 0;
      out.status = static_cast<Status>(byte);
      state_ = State::HEAD;
      return true;
  }
  return false;
}

void BNO055Uart::Reset()
{
  state_ = State::HEAD;
  len_ = 0;
  pos_ = 0;
}
}  // namespace rabcl

// --- BNO055 の rad 換算 ---

namespace rabcl
{
float BNO055::ScaleGyroRad(int16_t raw)
{
  return static_cast<float>(raw) / RAD_LSB;
}

float BNO055::ScaleEulerRad(int16_t raw)
{
  return static_cast<float>(raw) / RAD_LSB;
}
}  // namespace rabcl

// --- BNO055Sequencer ---

namespace rabcl
{
namespace
{
int16_t ReadI16(const uint8_t * p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

const char * BNO055Sequencer::StateName(State state)
{
  switch (state) {
    case State::BOOT_WAIT: return "BOOT_WAIT";
    case State::CHECK_ID: return "CHECK_ID";
    case State::TO_CONFIG: return "TO_CONFIG";
    case State::SET_PAGE: return "SET_PAGE";
    case State::SET_UNITS: return "SET_UNITS";
    case State::SET_AXIS_MAP: return "SET_AXIS_MAP";
    case State::SET_AXIS_SIGN: return "SET_AXIS_SIGN";
    case State::CHECK_CLOCK: return "CHECK_CLOCK";
    case State::SET_CLOCK: return "SET_CLOCK";
    case State::TO_FUSION: return "TO_FUSION";
    case State::CHECK_STATUS: return "CHECK_STATUS";
    case State::READ_SYS_ERR: return "READ_SYS_ERR";
    case State::RUNNING: return "RUNNING";
    case State::SOFT_RESET: return "SOFT_RESET";
  }
  return "?";
}

BNO055Sequencer::BNO055Sequencer(const BNO055Config & config)
: config_(config)
{
}

void BNO055Sequencer::Issue(const BNO055Op & op, uint32_t now_ms)
{
  pending_ = op;
  issued_ms_ = now_ms;
  waiting_ = op.expect_response;
}

void BNO055Sequencer::IssueWrite(uint8_t reg, uint8_t value, uint32_t now_ms, bool expect_response)
{
  BNO055Op op = {};
  op.kind = BNO055Op::Kind::WRITE;
  op.reg = reg;
  op.len = 1;
  op.data[0] = value;
  op.expect_response = expect_response;
  Issue(op, now_ms);
}

void BNO055Sequencer::IssueRead(uint8_t reg, uint8_t len, uint32_t now_ms)
{
  BNO055Op op = {};
  op.kind = BNO055Op::Kind::READ;
  op.reg = reg;
  op.len = len;
  op.expect_response = true;
  Issue(op, now_ms);
}

void BNO055Sequencer::EnterBootWait(uint32_t now_ms)
{
  state_ = State::BOOT_WAIT;
  boot_start_ms_ = now_ms;
  waiting_ = false;
  wait_until_ms_ = 0;
}

void BNO055Sequencer::Fail(uint32_t now_ms)
{
  waiting_ = false;
  ++error_count_;
  if (error_count_ > config_.max_consecutive_errors) {
    Restart(now_ms);
  }
}

void BNO055Sequencer::Succeed()
{
  waiting_ = false;
  error_count_ = 0;
}

void BNO055Sequencer::Restart(uint32_t now_ms)
{
  (void)now_ms;
  state_ = State::SOFT_RESET;
  waiting_ = false;
  wait_until_ms_ = 0;
}

bool BNO055Sequencer::Next(uint32_t now_ms, BNO055Op & op)
{
  if (waiting_) {
    if (now_ms - issued_ms_ < config_.response_timeout_ms) {
      return false;
    }
    Fail(now_ms);  // タイムアウト。同じ段をやり直す（しきい値ならソフトリセットへ）
    if (state_ == State::SOFT_RESET) {
      IssueWrite(REG_SYS_TRIGGER, TRIGGER_RST_SYS, now_ms, false);
      op = pending_;
      return true;
    }
  }
  if (now_ms < wait_until_ms_) {
    return false;
  }
  switch (state_) {
    case State::BOOT_WAIT:
      if (now_ms - boot_start_ms_ < config_.boot_wait_ms) {
        return false;
      }
      state_ = State::CHECK_ID;
      IssueRead(REG_CHIP_ID, 1, now_ms);
      break;
    case State::CHECK_ID:
      IssueRead(REG_CHIP_ID, 1, now_ms);
      break;
    case State::TO_CONFIG:
      IssueWrite(REG_OPR_MODE, BNO055::MODE_CONFIG, now_ms);
      break;
    case State::SET_PAGE:
      IssueWrite(REG_PAGE_ID, 0x00, now_ms);
      break;
    case State::SET_UNITS:
      IssueWrite(REG_UNIT_SEL, UNIT_SEL_RAD, now_ms);
      break;
    case State::SET_AXIS_MAP:
      IssueWrite(REG_AXIS_MAP_CONFIG, config_.axis_map_config, now_ms);
      break;
    case State::SET_AXIS_SIGN:
      IssueWrite(REG_AXIS_MAP_SIGN, config_.axis_map_sign, now_ms);
      break;
    case State::CHECK_CLOCK:
      IssueRead(REG_SYS_CLK_STATUS, 1, now_ms);
      break;
    case State::SET_CLOCK:
      IssueWrite(REG_SYS_TRIGGER, TRIGGER_CLK_SEL, now_ms);
      break;
    case State::TO_FUSION:
      IssueWrite(REG_OPR_MODE, config_.operation_mode, now_ms);
      break;
    case State::CHECK_STATUS:
      IssueRead(REG_SYS_STATUS, 1, now_ms);
      break;
    case State::READ_SYS_ERR:
      IssueRead(REG_SYS_ERR, 1, now_ms);
      break;
    case State::RUNNING:
      if (config_.sample_period_ms > 0) {  // 周期を自分で持つ場合（I2C など）。0 なら呼ばれるたびに読む
        if (static_cast<int32_t>(now_ms - next_read_ms_) < 0) {
          return false;
        }
        // 周期から大きく遅れたら今から数え直す（まとめて連射しない）
        next_read_ms_ = (now_ms - next_read_ms_ > config_.sample_period_ms) ?
          now_ms + config_.sample_period_ms : next_read_ms_ + config_.sample_period_ms;
      }
      ++read_count_;
      if (read_count_ % CALIB_EVERY_READS == 0) {
        IssueRead(REG_CALIB_STAT, 1, now_ms);
      } else {
        IssueRead(REG_GYR_DATA, DATA_LEN, now_ms);
      }
      break;
    case State::SOFT_RESET:
      IssueWrite(REG_SYS_TRIGGER, TRIGGER_RST_SYS, now_ms, false);
      break;
  }
  op = pending_;
  return true;
}

bool BNO055Sequencer::Complete(const BNO055Result & result, uint32_t now_ms, BNO055Sample & sample)
{
  if (state_ == State::SOFT_RESET) {
    // リセットは応答を待たない。送られた時点で起動待ちへ
    ++reinit_count_;
    error_count_ = 0;
    EnterBootWait(now_ms);
    return false;
  }
  if (!waiting_) {
    return false;  // 待っていない結果は捨てる
  }
  const bool is_read = pending_.kind == BNO055Op::Kind::READ;
  if (!result.ok || (is_read && result.len != pending_.len)) {
    Fail(now_ms);
    return false;
  }
  switch (state_) {
    case State::CHECK_ID:
      if (result.data[0] != CHIP_ID) {
        Fail(now_ms);
        return false;
      }
      Succeed();
      state_ = State::TO_CONFIG;
      break;
    case State::TO_CONFIG:
      Succeed();
      wait_until_ms_ = now_ms + CONFIG_SWITCH_MS;
      state_ = State::SET_PAGE;
      break;
    case State::SET_PAGE:
      Succeed();
      state_ = State::SET_UNITS;
      break;
    case State::SET_UNITS:
      Succeed();
      state_ = State::SET_AXIS_MAP;
      break;
    case State::SET_AXIS_MAP:
      Succeed();
      state_ = State::SET_AXIS_SIGN;
      break;
    case State::SET_AXIS_SIGN:
      Succeed();
      state_ = config_.use_external_crystal ? State::CHECK_CLOCK : State::TO_FUSION;
      break;
    case State::CHECK_CLOCK:
      if ((result.data[0] & 0x01) != 0) {
        Fail(now_ms);  // クロック切替中。少し待って読み直す
        wait_until_ms_ = now_ms + 1;
        return false;
      }
      Succeed();
      state_ = State::SET_CLOCK;
      break;
    case State::SET_CLOCK:
      Succeed();
      wait_until_ms_ = now_ms + config_.clock_settle_ms;
      state_ = State::TO_FUSION;
      break;
    case State::TO_FUSION:
      Succeed();
      wait_until_ms_ = now_ms + FUSION_SWITCH_MS;
      state_ = State::CHECK_STATUS;
      break;
    case State::CHECK_STATUS:
      if (result.data[0] == SYS_STATUS_ERROR) {
        waiting_ = false;
        state_ = State::READ_SYS_ERR;  // 理由を読んでからエラーに数える
        return false;
      }
      if (result.data[0] != SYS_STATUS_FUSION) {
        // 初期化中（2 / 3 など）。エラーには数えず、少し待って読み直す
        waiting_ = false;
        wait_until_ms_ = now_ms + STATUS_RETRY_MS;
        return false;
      }
      Succeed();
      state_ = State::RUNNING;
      next_read_ms_ = now_ms;
      read_count_ = 0;
      break;
    case State::READ_SYS_ERR:
      sys_err_ = result.data[0];
      Fail(now_ms);
      if (state_ != State::SOFT_RESET) {
        state_ = State::CHECK_STATUS;
        wait_until_ms_ = now_ms + STATUS_RETRY_MS;
      }
      return false;
    case State::RUNNING:
      Succeed();
      if (pending_.reg == REG_CALIB_STAT) {
        calib_stat_ = result.data[0];
        return false;
      }
      for (int i = 0; i < 3; ++i) {
        sample.gyro[i] = BNO055::ScaleGyroRad(ReadI16(&result.data[2 * i]));
        sample.euler[i] = BNO055::ScaleEulerRad(ReadI16(&result.data[6 + 2 * i]));
      }
      sample.calib_stat = calib_stat_;
      return true;
    case State::BOOT_WAIT:
    case State::SOFT_RESET:
      waiting_ = false;
      break;
  }
  return false;
}
}  // namespace rabcl

// --- BNO055UartDriver ---

namespace rabcl
{
BNO055UartDriver::BNO055UartDriver(const BNO055Config & config)
: sequencer_(config)
{
}

size_t BNO055UartDriver::Tick(uint32_t now_ms, uint8_t * out, size_t cap)
{
  BNO055Op op = {};
  if (!sequencer_.Next(now_ms, op)) {
    return 0;
  }
  const size_t need = op.kind == BNO055Op::Kind::READ ? 4 : 4 + static_cast<size_t>(op.len);
  if (cap < need) {
    return 0;
  }
  uart_.Reset();  // 新しい要求。途中まで来ていた古い応答は捨てる
  const size_t n = op.kind == BNO055Op::Kind::READ ?
    BNO055Uart::BuildRead(op.reg, op.len, out) :
    BNO055Uart::BuildWrite(op.reg, op.data, op.len, out);
  if (!op.expect_response) {
    BNO055Result r = {};
    r.ok = true;
    BNO055Sample unused = {};
    sequencer_.Complete(r, now_ms, unused);
  }
  return n;
}

bool BNO055UartDriver::Feed(
  const uint8_t * data, size_t len, uint32_t now_ms, BNO055Sample & sample)
{
  for (size_t i = 0; i < len; ++i) {
    if (!sequencer_.waiting()) {
      return false;  // 待っていないバイトは捨てる
    }
    BNO055Uart::Response resp = {};
    if (!uart_.Push(data[i], resp)) {
      continue;
    }
    const BNO055Op & op = sequencer_.pending();
    BNO055Result r = {};
    if (resp.is_data) {
      if (op.kind != BNO055Op::Kind::READ) {
        r.ok = false;
        r.error = static_cast<uint8_t>(BNO055Uart::Status::WRONG_START_BYTE);
      } else if (resp.len != op.len) {
        r.ok = false;
        r.error = static_cast<uint8_t>(BNO055Uart::Status::MIN_LENGTH_ERROR);
      } else {
        r.ok = true;
        r.len = resp.len;
        for (uint8_t k = 0; k < resp.len; ++k) {
          r.data[k] = resp.data[k];
        }
      }
    } else {
      const bool success = resp.status == BNO055Uart::Status::WRITE_SUCCESS &&
        op.kind == BNO055Op::Kind::WRITE;
      r.ok = success;
      r.error = success ? BNO055Result::ERROR_NONE : static_cast<uint8_t>(resp.status);
    }
    return sequencer_.Complete(r, now_ms, sample);
  }
  return false;
}

size_t BNO055UartDriver::expected_response_len() const
{
  if (!sequencer_.waiting()) {
    return 0;
  }
  const BNO055Op & op = sequencer_.pending();
  return op.kind == BNO055Op::Kind::READ ? 2 + static_cast<size_t>(op.len) : 2;
}

// --- BNO055I2cDriver ---

BNO055I2cDriver::BNO055I2cDriver(const BNO055Config & config, BNO055RegisterBus & bus)
: sequencer_(config), bus_(bus)
{
}

bool BNO055I2cDriver::Tick(uint32_t now_ms, BNO055Sample & sample)
{
  BNO055Op op = {};
  if (!sequencer_.Next(now_ms, op)) {
    return false;
  }
  BNO055Result r = {};
  if (op.kind == BNO055Op::Kind::READ) {
    r.ok = bus_.Read(op.reg, r.data, op.len);
    r.len = op.len;
  } else {
    r.ok = bus_.Write(op.reg, op.data, op.len);
  }
  if (!r.ok) {
    r.error = ERROR_BUS;
  }
  return sequencer_.Complete(r, now_ms, sample);
}
}  // namespace rabcl
