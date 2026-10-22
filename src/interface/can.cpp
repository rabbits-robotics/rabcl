#include "rabcl/interface/can.hpp"

#include <cmath>

namespace rabcl
{
Can::Can()
{
  // NOP
}

Can::~Can()
{
  // NOP
}

bool Can::UpdateData(
  uint32_t idx, const uint8_t can_data[8], Info & data,
  float yaw_offset, float pitch_offset)
{
  union {
    float f;
    int32_t ui;
  } buf;

  // --- UART reference data (from turret board via internal CAN, proto2 compatible)
  if (idx == static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_X_Y)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF));
    data.chassis_vel_x_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF));
    data.chassis_vel_y_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_Z_YAW)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF));
    data.chassis_vel_z_ = buf.f;
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[4]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[5]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[6]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[7]) << 0) & 0x000000FF));
    data.yaw_pos_ = buf.f;
  } else if (idx == static_cast<uint32_t>(CAN_ID::CAN_PITCH_MODES)) {
    buf.ui = static_cast<int32_t>(
      ((static_cast<int32_t>(can_data[0]) << 24) & 0xFF000000) |
      ((static_cast<int32_t>(can_data[1]) << 16) & 0x00FF0000) |
      ((static_cast<int32_t>(can_data[2]) << 8) & 0x0000FF00) |
      ((static_cast<int32_t>(can_data[3]) << 0) & 0x000000FF));
    data.pitch_pos_ = buf.f;
    data.load_mode_ = can_data[4 + static_cast<int>(MODE_ID::LOAD)];
    data.fire_mode_ = can_data[4 + static_cast<int>(MODE_ID::FIRE)];
    data.speed_mode_ = can_data[4 + static_cast<int>(MODE_ID::SPEED)];
    data.chassis_mode_ = can_data[4 + static_cast<int>(MODE_ID::CHASSIS)];

  // --- LK motor feedback
  } else if (idx == static_cast<uint32_t>(CAN_ID::YAW_RX)) {
    ParseLKMotorFeedback(can_data, data.yaw_act_);
    data.yaw_act_.position_ -= yaw_offset;
  // --- PITCH: DM3507 feedback (output-shaft → GR=1)
  } else if (idx == static_cast<uint32_t>(CAN_ID::PITCH_RX)) {
    ParseDMMotorFeedback(
      can_data, data.pitch_act_,
      DM3507_PMAX, DM3507_VMAX, DM3507_TMAX, 1.0f);
    data.pitch_act_.position_ -= pitch_offset;

  // --- DM2325 feedback (chassis wheels)
  } else if (idx == static_cast<uint32_t>(CAN_ID::CHASSIS_FRONT_RIGHT_RX)) {
    ParseDMMotorFeedback(
      can_data, data.chassis_fr_act_,
      DM2325_PMAX, DM2325_VMAX, DM2325_TMAX, DM2325_GR);
  } else if (idx == static_cast<uint32_t>(CAN_ID::CHASSIS_FRONT_LEFT_RX)) {
    ParseDMMotorFeedback(
      can_data, data.chassis_fl_act_,
      DM2325_PMAX, DM2325_VMAX, DM2325_TMAX, DM2325_GR);
  } else if (idx == static_cast<uint32_t>(CAN_ID::CHASSIS_BACK_RIGHT_RX)) {
    ParseDMMotorFeedback(
      can_data, data.chassis_br_act_,
      DM2325_PMAX, DM2325_VMAX, DM2325_TMAX, DM2325_GR);
  } else if (idx == static_cast<uint32_t>(CAN_ID::CHASSIS_BACK_LEFT_RX)) {
    ParseDMMotorFeedback(
      can_data, data.chassis_bl_act_,
      DM2325_PMAX, DM2325_VMAX, DM2325_TMAX, DM2325_GR);

  } else {
    return false;
  }
  return true;
}

void Can::Prepare2FloatData(float in_1, float in_2, uint8_t can_data[8])
{
  union {
    float f;
    int32_t ui;
  } buf;
  buf.f = in_1;
  can_data[0] = static_cast<uint8_t>((buf.ui & 0xFF000000) >> 24);
  can_data[1] = static_cast<uint8_t>((buf.ui & 0x00FF0000) >> 16);
  can_data[2] = static_cast<uint8_t>((buf.ui & 0x0000FF00) >> 8);
  can_data[3] = static_cast<uint8_t>((buf.ui & 0x000000FF) >> 0);
  buf.f = in_2;
  can_data[4] = static_cast<uint8_t>((buf.ui & 0xFF000000) >> 24);
  can_data[5] = static_cast<uint8_t>((buf.ui & 0x00FF0000) >> 16);
  can_data[6] = static_cast<uint8_t>((buf.ui & 0x0000FF00) >> 8);
  can_data[7] = static_cast<uint8_t>((buf.ui & 0x000000FF) >> 0);
}

void Can::Prepare1Float4IntData(float in_1, const uint8_t in_2[4], uint8_t can_data[8])
{
  union {
    float f;
    int32_t ui;
  } buf;
  buf.f = in_1;
  can_data[0] = static_cast<uint8_t>((buf.ui & 0xFF000000) >> 24);
  can_data[1] = static_cast<uint8_t>((buf.ui & 0x00FF0000) >> 16);
  can_data[2] = static_cast<uint8_t>((buf.ui & 0x0000FF00) >> 8);
  can_data[3] = static_cast<uint8_t>((buf.ui & 0x000000FF) >> 0);
  can_data[4] = in_2[0];
  can_data[5] = in_2[1];
  can_data[6] = in_2[2];
  can_data[7] = in_2[3];
}

void Can::PrepareLKMotorMotorOff(uint8_t can_data[8])
{
  can_data[0] = 0x80;
  can_data[1] = 0x00;
  can_data[2] = 0x00;
  can_data[3] = 0x00;
  can_data[4] = 0x00;
  can_data[5] = 0x00;
  can_data[6] = 0x00;
  can_data[7] = 0x00;
}

void Can::PrepareLKMotorMotorOn(uint8_t can_data[8])
{
  can_data[0] = 0x88;
  can_data[1] = 0x00;
  can_data[2] = 0x00;
  can_data[3] = 0x00;
  can_data[4] = 0x00;
  can_data[5] = 0x00;
  can_data[6] = 0x00;
  can_data[7] = 0x00;
}

void Can::PrepareLKMotorMotorStop(uint8_t can_data[8])
{
  can_data[0] = 0x81;
  can_data[1] = 0x00;
  can_data[2] = 0x00;
  can_data[3] = 0x00;
  can_data[4] = 0x00;
  can_data[5] = 0x00;
  can_data[6] = 0x00;
  can_data[7] = 0x00;
}

void Can::PrepareLKMotorTorqueCmd(int16_t current, uint8_t can_data[8])
{
  can_data[0] = 0xA1;
  can_data[1] = 0x00;
  can_data[2] = 0x00;
  can_data[3] = 0x00;
  can_data[4] = static_cast<uint8_t>(current & 0xFF);
  can_data[5] = static_cast<uint8_t>((current >> 8) & 0xFF);
  can_data[6] = 0x00;
  can_data[7] = 0x00;
}

void Can::PrepareLKMotorPositionCmd(
  int32_t pos, uint16_t max_speed, uint8_t can_data[8])
{
  can_data[0] = 0xA4;
  can_data[1] = 0x00;
  can_data[2] = static_cast<uint8_t>( max_speed & 0xFF);
  can_data[3] = static_cast<uint8_t>((max_speed >> 8) & 0xFF);
  can_data[4] = static_cast<uint8_t>( pos & 0xFF);
  can_data[5] = static_cast<uint8_t>((pos >> 8) & 0xFF);
  can_data[6] = static_cast<uint8_t>((pos >> 16) & 0xFF);
  can_data[7] = static_cast<uint8_t>((pos >> 24) & 0xFF);
}

void Can::PrepareLKMotorReadParam(uint8_t param_id, uint8_t can_data[8])
{
  can_data[0] = 0xC0;
  can_data[1] = param_id;
  can_data[2] = 0x00;
  can_data[3] = 0x00;
  can_data[4] = 0x00;
  can_data[5] = 0x00;
  can_data[6] = 0x00;
  can_data[7] = 0x00;
}

void Can::PrepareLKMotorWritePID(
  uint8_t param_id, uint16_t kp, uint16_t ki, uint16_t kd,
  uint8_t can_data[8])
{
  can_data[0] = 0xC1;
  can_data[1] = param_id;
  can_data[2] = static_cast<uint8_t>( kp & 0xFF);
  can_data[3] = static_cast<uint8_t>((kp >> 8) & 0xFF);
  can_data[4] = static_cast<uint8_t>( ki & 0xFF);
  can_data[5] = static_cast<uint8_t>((ki >> 8) & 0xFF);
  can_data[6] = static_cast<uint8_t>( kd & 0xFF);
  can_data[7] = static_cast<uint8_t>((kd >> 8) & 0xFF);
}

void Can::PrepareRMDMotorReadPID(uint8_t can_data[8])
{
  can_data[0] = 0x30;
  can_data[1] = 0x00;
  can_data[2] = 0x00;
  can_data[3] = 0x00;
  can_data[4] = 0x00;
  can_data[5] = 0x00;
  can_data[6] = 0x00;
  can_data[7] = 0x00;
}

void Can::PrepareRMDMotorWritePIDToRAM(
  uint8_t curr_kp, uint8_t curr_ki,
  uint8_t speed_kp, uint8_t speed_ki,
  uint8_t pos_kp, uint8_t pos_ki,
  uint8_t can_data[8])
{
  can_data[0] = 0x31;
  can_data[1] = 0x00;
  can_data[2] = curr_kp;
  can_data[3] = curr_ki;
  can_data[4] = speed_kp;
  can_data[5] = speed_ki;
  can_data[6] = pos_kp;
  can_data[7] = pos_ki;
}

void Can::PrepareLKMotorReadMotorState2(uint8_t can_data[8])
{
  can_data[0] = 0x9C;
  can_data[1] = 0x00;
  can_data[2] = 0x00;
  can_data[3] = 0x00;
  can_data[4] = 0x00;
  can_data[5] = 0x00;
  can_data[6] = 0x00;
  can_data[7] = 0x00;
}

void Can::PrepareDMMotorEnable(uint8_t can_data[8])
{
  can_data[0] = 0xFF;
  can_data[1] = 0xFF;
  can_data[2] = 0xFF;
  can_data[3] = 0xFF;
  can_data[4] = 0xFF;
  can_data[5] = 0xFF;
  can_data[6] = 0xFF;
  can_data[7] = 0xFC;
}

void Can::PrepareDMMotorVelocityCmd(float velocity, uint8_t can_data[8])
{
  union {
    float f;
    uint8_t b[4];
  } buf;
  buf.f = velocity;  // output shaft rad/s
  can_data[0] = buf.b[0];
  can_data[1] = buf.b[1];
  can_data[2] = buf.b[2];
  can_data[3] = buf.b[3];
  can_data[4] = 0x00;
  can_data[5] = 0x00;
  can_data[6] = 0x00;
  can_data[7] = 0x00;
}

static uint16_t FloatToUint(float x, float lo, float hi, uint8_t bits)
{
  if (x < lo) {x = lo;}
  if (x > hi) {x = hi;}
  return static_cast<uint16_t>(
    (x - lo) * (static_cast<float>((1u << bits) - 1u) / (hi - lo)));
}

void Can::PrepareDMMotorMITCmd(
  float pos, float vel, float kp, float kd, float tff,
  float pmax, float vmax, float tmax, uint8_t can_data[8])
{
  uint16_t p = FloatToUint(pos, -pmax, pmax, 16);
  uint16_t v = FloatToUint(vel, -vmax, vmax, 12);
  uint16_t kp_u = FloatToUint(kp, 0.0f, DM_KP_MAX, 12);
  uint16_t kd_u = FloatToUint(kd, 0.0f, DM_KD_MAX, 12);
  uint16_t t = FloatToUint(tff, -tmax, tmax, 12);
  can_data[0] = static_cast<uint8_t>(p >> 8);
  can_data[1] = static_cast<uint8_t>(p & 0xFF);
  can_data[2] = static_cast<uint8_t>(v >> 4);
  can_data[3] = static_cast<uint8_t>(((v & 0x0F) << 4) | (kp_u >> 8));
  can_data[4] = static_cast<uint8_t>(kp_u & 0xFF);
  can_data[5] = static_cast<uint8_t>(kd_u >> 4);
  can_data[6] = static_cast<uint8_t>(((kd_u & 0x0F) << 4) | (t >> 8));
  can_data[7] = static_cast<uint8_t>(t & 0xFF);
}

void Can::ParseLKMotorFeedback(const uint8_t can_data[8], MotorInfo & motor)
{
  motor.temperature_ = static_cast<float>(can_data[1]);

  int16_t raw_current = static_cast<int16_t>(
    (static_cast<uint16_t>(can_data[3]) << 8) | can_data[2]);
  motor.current_ = raw_current * 0.01f;  // 0.01 A/LSB

  int16_t raw_speed = static_cast<int16_t>(
    (static_cast<uint16_t>(can_data[5]) << 8) | can_data[4]);
  motor.velocity_ = raw_speed * (static_cast<float>(M_PI) / 180.0f);  // deg/s → rad/s

  uint16_t raw_pos = (static_cast<uint16_t>(can_data[7]) << 8) | can_data[6];
  motor.position_ = raw_pos / 65535.0f * 2.0f * static_cast<float>(M_PI);  // uint16(0~65535) → 0~2π rad
}

void Can::ParseDMMotorFeedback(
  const uint8_t can_data[8], MotorInfo & motor,
  float pmax, float vmax, float tmax, float gr)
{
  uint16_t pos_raw = (static_cast<uint16_t>(can_data[1]) << 8) | can_data[2];
  uint16_t vel_raw = (static_cast<uint16_t>(can_data[3]) << 4) | (can_data[4] >> 4);
  uint16_t tor_raw = (static_cast<uint16_t>(can_data[4] & 0x0F) << 8) | can_data[5];

  motor.position_ = (pos_raw / 65535.0f * 2.0f * pmax - pmax) / gr;   // uint16(0~65535) → [-PMAX, +PMAX] → /GR
  motor.velocity_ = vel_raw / 4095.0f * 2.0f * vmax - vmax;           // uint12(0~4095) → [-VMAX, +VMAX] (output shaft)
  motor.torque_ = tor_raw / 4095.0f * 2.0f * tmax - tmax;             // uint12(0~4095) → [-TMAX, +TMAX]
  motor.temperature_mos_ = static_cast<float>(can_data[6]);           // D[6]=T_MOS
  motor.temperature_ = static_cast<float>(can_data[7]);               // D[7]=T_Rotor
}

uint8_t Can::DMFeedbackId(const uint8_t can_data[8])
{
  return can_data[0] & 0x0F;
}

uint8_t Can::DMFeedbackError(const uint8_t can_data[8])
{
  return (can_data[0] >> 4) & 0x0F;
}

}  // namespace rabcl
