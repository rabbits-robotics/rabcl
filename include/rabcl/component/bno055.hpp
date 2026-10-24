// BNO055（Bosch の 9 軸 IMU、センサ融合内蔵）。
//   BNO055      レジスタ番地と生データの換算（バスに依らない）
//   BNO055Uart  UART モード（PS1 = High、115200 8N1）のフレームの組み立てと受信パーサ
// 設計書: docs/superpowers/specs/2026-10-24-bno055-uart-design.md
#ifndef RABCL__COMPONENT__BNO055_HPP_
#define RABCL__COMPONENT__BNO055_HPP_

#include <cstddef>
#include <cstdint>
#include <cmath>
#include "rabcl/utils/type.hpp"

namespace rabcl
{
class BNO055
{
public:
  static constexpr uint8_t I2C_ADDR = 0x28 << 1;  // I2C で使うときのアドレス（PS1 = Low、COM3 = Low）
  static constexpr uint8_t CHIP_ID_ADDR = 0x00;
  static constexpr uint8_t OPR_MODE_ADDR = 0x3D;
  static constexpr uint8_t ACC_DATA_X_LSB = 0x08;
  static constexpr uint8_t GYR_DATA_X_LSB = 0x14;
  static constexpr uint8_t EULER_H_LSB = 0x1A;
  static constexpr uint8_t EXPECTED_CHIP_ID = 0xA0;
  static constexpr uint8_t MODE_CONFIG = 0x00;
  static constexpr uint8_t MODE_NDOF = 0x0C;

  BNO055();
  ~BNO055();

  // 既定の単位（加速度 m/s²、ジャイロ °/s、オイラー °）の生データを SI に換算する
  void UpdateAccel(const uint8_t raw[6], ImuInfo & imu);
  void UpdateGyro(const uint8_t raw[6], ImuInfo & imu);
  void UpdateEuler(const uint8_t raw[6], ImuInfo & imu);

private:
  static constexpr float ACC_SCALE = 0.01f;
  static constexpr float GYRO_SCALE = (1.0f / 16.0f) * (M_PI / 180.0f);
  static constexpr float EULER_SCALE = (1.0f / 16.0f) * (M_PI / 180.0f);
};

// UART モードのフレーム。
//   要求: AA 00 reg len data（書き）/ AA 01 reg len（読み）
//   応答: BB len data（読みの結果）/ EE status（書きの結果、またはエラー）
class BNO055Uart
{
public:
  static constexpr uint8_t START = 0xAA;
  static constexpr uint8_t RESPONSE = 0xBB;
  static constexpr uint8_t ACK = 0xEE;
  static constexpr size_t MAX_READ = 128;

  enum class Status : uint8_t
  {
    WRITE_SUCCESS = 0x01,
    READ_FAIL = 0x02,
    WRITE_FAIL = 0x03,
    REGMAP_INVALID_ADDRESS = 0x04,
    REGMAP_WRITE_DISABLED = 0x05,
    WRONG_START_BYTE = 0x06,
    BUS_OVER_RUN_ERROR = 0x07,
    MAX_LENGTH_ERROR = 0x08,
    MIN_LENGTH_ERROR = 0x09,
    RECEIVE_CHARACTER_TIMEOUT = 0x0A,
  };

  struct Response
  {
    bool is_data;             // true: 読みの結果（len, data）。false: status
    uint8_t len;
    uint8_t data[MAX_READ];
    Status status;
  };

  // 要求の組み立て。書いたバイト数を返す（out は 4 + len 以上）
  static size_t BuildRead(uint8_t reg, uint8_t len, uint8_t * out);
  static size_t BuildWrite(uint8_t reg, const uint8_t * data, uint8_t len, uint8_t * out);

  // 受信パーサ。1 バイトずつ入れ、応答が揃った瞬間だけ true を返して out に置く。
  // 先頭が BB / EE 以外のバイトは捨てる（同期ずれからの回復）。len が 0 または MAX_READ 超も捨てる
  bool Push(uint8_t byte, Response & out);
  void Reset();  // 要求を捨てたとき（タイムアウト）に途中のフレームを捨てる

private:
  enum class State : uint8_t
  {
    HEAD,
    DATA_LEN,
    DATA,
    STATUS,
  };

  State state_ = State::HEAD;
  uint8_t len_ = 0;
  uint8_t pos_ = 0;
  uint8_t buf_[MAX_READ] = {};
};

}  // namespace rabcl

#endif  // RABCL__COMPONENT__BNO055_HPP_
