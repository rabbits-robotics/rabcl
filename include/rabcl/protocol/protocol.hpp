// PC ⇔ ファームの共通プロトコル（機体に依らない）
// フレーム: | 0xA5 | 0x5A | type | len | payload (len B) | crc8 |
// crc8 は type + len + payload に対する CRC-8（多項式 0x07、初期値 0x00、非反転）。
// 数値はリトルエンディアン、float は binary32。
// 設計書: docs/superpowers/specs/2026-10-22-robot-protocol-design.md
#ifndef RABCL__PROTOCOL__PROTOCOL_HPP_
#define RABCL__PROTOCOL__PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace rabcl::protocol
{

constexpr uint8_t VERSION = 1;
constexpr uint8_t HEADER_0 = 0xA5;
constexpr uint8_t HEADER_1 = 0x5A;
constexpr size_t MAX_PAYLOAD = 255;
constexpr size_t FRAME_OVERHEAD = 5;  // ヘッダ 2 + type 1 + len 1 + crc 1
constexpr size_t MAX_FRAME = FRAME_OVERHEAD + MAX_PAYLOAD;  // 260
constexpr uint8_t MAX_JOINT_STATES = 14;  // 1 + 14 * 18 = 253 B
constexpr uint8_t MAX_JOINT_CMDS = 12;    // 1 + 12 * 21 = 253 B
constexpr uint8_t MAX_MODES = 127;        // 1 + 127 * 2 = 255 B

// 上位 1 bit が方向: 0x00〜0x7F は上り（ファーム → PC）、0x80〜0xFF は下り（PC → ファーム）
enum class Type : uint8_t
{
  INFO = 0x01,
  JOINT_STATE = 0x02,
  IMU = 0x03,
  SYSTEM = 0x04,
  CHASSIS_VEL = 0x81,
  JOINT_CMD = 0x82,
  MODE = 0x83,
  BODY_POSE = 0x84,
};

struct Info
{
  uint8_t protocol_version;
  uint8_t joint_count;
  char firmware_id[8];  // ASCII、末尾 0 埋め
};

struct JointState
{
  uint8_t id;
  float position;     // [rad]
  float velocity;     // [rad/s]
  float effort;       // [Nm] または [A]（関節ごとに決まる）
  float temperature;  // [℃]
  uint8_t error;      // 0 = 正常。意味はモータ種別ごと
};

struct Imu
{
  float acc[3];    // [m/s^2]
  float gyro[3];   // [rad/s]
  float euler[3];  // [rad] heading, roll, pitch
};

struct System
{
  float bus_voltage;    // [V]
  uint32_t error_flags;
  uint32_t uptime_ms;
};

struct ChassisVel
{
  float vx;  // [m/s]
  float vy;  // [m/s]
  float wz;  // [rad/s]
};

struct JointCmd
{
  uint8_t id;
  float position;  // [rad]
  float velocity;  // [rad/s]
  float effort;    // [Nm] フィードフォワード
  float kp;
  float kd;
};

struct ModeCmd
{
  uint8_t id;
  uint8_t value;
};

struct BodyPose
{
  float height;  // 0〜1（0 = 最低車高）
  float roll;    // [rad] 正 = 右が下がる（REP-103）
  float pitch;   // [rad] 正 = 前が下がる
};

constexpr uint8_t INFO_LEN = 10;
constexpr uint8_t JOINT_STATE_ITEM_LEN = 18;
constexpr uint8_t IMU_LEN = 36;
constexpr uint8_t SYSTEM_LEN = 12;
constexpr uint8_t CHASSIS_VEL_LEN = 12;
constexpr uint8_t JOINT_CMD_ITEM_LEN = 21;
constexpr uint8_t MODE_ITEM_LEN = 2;
constexpr uint8_t BODY_POSE_LEN = 12;

uint8_t Crc8(const uint8_t * data, size_t len);

// エンコード: buf にフレーム全体を書き、フレーム長を返す。容量不足または個数超過なら 0
size_t EncodeInfo(const Info & info, uint8_t * buf, size_t cap);
size_t EncodeJointState(const JointState * joints, uint8_t count, uint8_t * buf, size_t cap);
size_t EncodeImu(const Imu & imu, uint8_t * buf, size_t cap);
size_t EncodeSystem(const System & system, uint8_t * buf, size_t cap);
size_t EncodeChassisVel(const ChassisVel & vel, uint8_t * buf, size_t cap);
size_t EncodeJointCmd(const JointCmd * cmds, uint8_t count, uint8_t * buf, size_t cap);
size_t EncodeMode(const ModeCmd * modes, uint8_t count, uint8_t * buf, size_t cap);
size_t EncodeBodyPose(const BodyPose & pose, uint8_t * buf, size_t cap);

// デコード: payload（len B）から構造体へ。len がレイアウトと合わなければ false。
// 配列型は count が max を超えると false
bool DecodeInfo(const uint8_t * payload, uint8_t len, Info & info);
bool DecodeJointState(
  const uint8_t * payload, uint8_t len, JointState * joints, uint8_t max, uint8_t & count);
bool DecodeImu(const uint8_t * payload, uint8_t len, Imu & imu);
bool DecodeSystem(const uint8_t * payload, uint8_t len, System & system);
bool DecodeChassisVel(const uint8_t * payload, uint8_t len, ChassisVel & vel);
bool DecodeJointCmd(
  const uint8_t * payload, uint8_t len, JointCmd * cmds, uint8_t max, uint8_t & count);
bool DecodeMode(
  const uint8_t * payload, uint8_t len, ModeCmd * modes, uint8_t max,
  uint8_t & count);
bool DecodeBodyPose(const uint8_t * payload, uint8_t len, BodyPose & pose);

}  // namespace rabcl::protocol

#endif  // RABCL__PROTOCOL__PROTOCOL_HPP_
