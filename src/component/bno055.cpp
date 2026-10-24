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
