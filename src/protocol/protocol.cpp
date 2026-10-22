#include "rabcl/protocol/protocol.hpp"

#include <cstring>

namespace rabcl::protocol
{

namespace
{

constexpr uint8_t CRC8_POLY = 0x07;

// payload を書き込む位置を返す。容量不足なら nullptr
uint8_t * BeginFrame(Type type, uint8_t len, uint8_t * buf, size_t cap)
{
  if (buf == nullptr || cap < FRAME_OVERHEAD + len) {
    return nullptr;
  }
  buf[0] = HEADER_0;
  buf[1] = HEADER_1;
  buf[2] = static_cast<uint8_t>(type);
  buf[3] = len;
  return &buf[4];
}

// CRC を付けてフレーム長を返す
size_t EndFrame(uint8_t * buf, uint8_t len)
{
  buf[4 + len] = Crc8(&buf[2], static_cast<size_t>(len) + 2);
  return FRAME_OVERHEAD + len;
}

void PutF32(uint8_t *& p, float v)
{
  std::memcpy(p, &v, 4);
  p += 4;
}

void PutU32(uint8_t *& p, uint32_t v)
{
  std::memcpy(p, &v, 4);
  p += 4;
}

float GetF32(const uint8_t *& p)
{
  float v;
  std::memcpy(&v, p, 4);
  p += 4;
  return v;
}

uint32_t GetU32(const uint8_t *& p)
{
  uint32_t v;
  std::memcpy(&v, p, 4);
  p += 4;
  return v;
}

}  // namespace

uint8_t Crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ CRC8_POLY) :
        static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

// --- Encode ---

size_t EncodeInfo(const Info & info, uint8_t * buf, size_t cap)
{
  uint8_t * p = BeginFrame(Type::INFO, INFO_LEN, buf, cap);
  if (p == nullptr) {
    return 0;
  }
  p[0] = info.protocol_version;
  p[1] = info.joint_count;
  std::memcpy(&p[2], info.firmware_id, sizeof(info.firmware_id));
  return EndFrame(buf, INFO_LEN);
}

size_t EncodeJointState(const JointState * joints, uint8_t count, uint8_t * buf, size_t cap)
{
  if (count > MAX_JOINT_STATES) {
    return 0;
  }
  const uint8_t len = static_cast<uint8_t>(1 + JOINT_STATE_ITEM_LEN * count);
  uint8_t * p = BeginFrame(Type::JOINT_STATE, len, buf, cap);
  if (p == nullptr) {
    return 0;
  }
  *p++ = count;
  for (uint8_t i = 0; i < count; ++i) {
    *p++ = joints[i].id;
    PutF32(p, joints[i].position);
    PutF32(p, joints[i].velocity);
    PutF32(p, joints[i].effort);
    PutF32(p, joints[i].temperature);
    *p++ = joints[i].error;
  }
  return EndFrame(buf, len);
}

size_t EncodeImu(const Imu & imu, uint8_t * buf, size_t cap)
{
  uint8_t * p = BeginFrame(Type::IMU, IMU_LEN, buf, cap);
  if (p == nullptr) {
    return 0;
  }
  for (int i = 0; i < 3; ++i) {
    PutF32(p, imu.acc[i]);
  }
  for (int i = 0; i < 3; ++i) {
    PutF32(p, imu.gyro[i]);
  }
  for (int i = 0; i < 3; ++i) {
    PutF32(p, imu.euler[i]);
  }
  return EndFrame(buf, IMU_LEN);
}

size_t EncodeSystem(const System & system, uint8_t * buf, size_t cap)
{
  uint8_t * p = BeginFrame(Type::SYSTEM, SYSTEM_LEN, buf, cap);
  if (p == nullptr) {
    return 0;
  }
  PutF32(p, system.bus_voltage);
  PutU32(p, system.error_flags);
  PutU32(p, system.uptime_ms);
  return EndFrame(buf, SYSTEM_LEN);
}

size_t EncodeChassisVel(const ChassisVel & vel, uint8_t * buf, size_t cap)
{
  uint8_t * p = BeginFrame(Type::CHASSIS_VEL, CHASSIS_VEL_LEN, buf, cap);
  if (p == nullptr) {
    return 0;
  }
  PutF32(p, vel.vx);
  PutF32(p, vel.vy);
  PutF32(p, vel.wz);
  return EndFrame(buf, CHASSIS_VEL_LEN);
}

size_t EncodeJointCmd(const JointCmd * cmds, uint8_t count, uint8_t * buf, size_t cap)
{
  if (count > MAX_JOINT_CMDS) {
    return 0;
  }
  const uint8_t len = static_cast<uint8_t>(1 + JOINT_CMD_ITEM_LEN * count);
  uint8_t * p = BeginFrame(Type::JOINT_CMD, len, buf, cap);
  if (p == nullptr) {
    return 0;
  }
  *p++ = count;
  for (uint8_t i = 0; i < count; ++i) {
    *p++ = cmds[i].id;
    PutF32(p, cmds[i].position);
    PutF32(p, cmds[i].velocity);
    PutF32(p, cmds[i].effort);
    PutF32(p, cmds[i].kp);
    PutF32(p, cmds[i].kd);
  }
  return EndFrame(buf, len);
}

size_t EncodeMode(const ModeCmd * modes, uint8_t count, uint8_t * buf, size_t cap)
{
  if (count > MAX_MODES) {
    return 0;
  }
  const uint8_t len = static_cast<uint8_t>(1 + MODE_ITEM_LEN * count);
  uint8_t * p = BeginFrame(Type::MODE, len, buf, cap);
  if (p == nullptr) {
    return 0;
  }
  *p++ = count;
  for (uint8_t i = 0; i < count; ++i) {
    *p++ = modes[i].id;
    *p++ = modes[i].value;
  }
  return EndFrame(buf, len);
}

// --- Decode ---

bool DecodeInfo(const uint8_t * payload, uint8_t len, Info & info)
{
  if (len != INFO_LEN) {
    return false;
  }
  info.protocol_version = payload[0];
  info.joint_count = payload[1];
  std::memcpy(info.firmware_id, &payload[2], sizeof(info.firmware_id));
  return true;
}

bool DecodeJointState(
  const uint8_t * payload, uint8_t len, JointState * joints, uint8_t max, uint8_t & count)
{
  if (len < 1) {
    return false;
  }
  const uint8_t n = payload[0];
  if (n > max || len != 1 + JOINT_STATE_ITEM_LEN * n) {
    return false;
  }
  const uint8_t * p = &payload[1];
  for (uint8_t i = 0; i < n; ++i) {
    joints[i].id = *p++;
    joints[i].position = GetF32(p);
    joints[i].velocity = GetF32(p);
    joints[i].effort = GetF32(p);
    joints[i].temperature = GetF32(p);
    joints[i].error = *p++;
  }
  count = n;
  return true;
}

bool DecodeImu(const uint8_t * payload, uint8_t len, Imu & imu)
{
  if (len != IMU_LEN) {
    return false;
  }
  const uint8_t * p = payload;
  for (int i = 0; i < 3; ++i) {
    imu.acc[i] = GetF32(p);
  }
  for (int i = 0; i < 3; ++i) {
    imu.gyro[i] = GetF32(p);
  }
  for (int i = 0; i < 3; ++i) {
    imu.euler[i] = GetF32(p);
  }
  return true;
}

bool DecodeSystem(const uint8_t * payload, uint8_t len, System & system)
{
  if (len != SYSTEM_LEN) {
    return false;
  }
  const uint8_t * p = payload;
  system.bus_voltage = GetF32(p);
  system.error_flags = GetU32(p);
  system.uptime_ms = GetU32(p);
  return true;
}

bool DecodeChassisVel(const uint8_t * payload, uint8_t len, ChassisVel & vel)
{
  if (len != CHASSIS_VEL_LEN) {
    return false;
  }
  const uint8_t * p = payload;
  vel.vx = GetF32(p);
  vel.vy = GetF32(p);
  vel.wz = GetF32(p);
  return true;
}

bool DecodeJointCmd(
  const uint8_t * payload, uint8_t len, JointCmd * cmds, uint8_t max, uint8_t & count)
{
  if (len < 1) {
    return false;
  }
  const uint8_t n = payload[0];
  if (n > max || len != 1 + JOINT_CMD_ITEM_LEN * n) {
    return false;
  }
  const uint8_t * p = &payload[1];
  for (uint8_t i = 0; i < n; ++i) {
    cmds[i].id = *p++;
    cmds[i].position = GetF32(p);
    cmds[i].velocity = GetF32(p);
    cmds[i].effort = GetF32(p);
    cmds[i].kp = GetF32(p);
    cmds[i].kd = GetF32(p);
  }
  count = n;
  return true;
}

bool DecodeMode(const uint8_t * payload, uint8_t len, ModeCmd * modes, uint8_t max, uint8_t & count)
{
  if (len < 1) {
    return false;
  }
  const uint8_t n = payload[0];
  if (n > max || len != 1 + MODE_ITEM_LEN * n) {
    return false;
  }
  const uint8_t * p = &payload[1];
  for (uint8_t i = 0; i < n; ++i) {
    modes[i].id = *p++;
    modes[i].value = *p++;
  }
  count = n;
  return true;
}

}  // namespace rabcl::protocol
