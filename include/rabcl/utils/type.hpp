#ifndef RABCL__UTILS__TYPE_HPP_
#define RABCL__UTILS__TYPE_HPP_

#include <cstdint>

namespace rabcl
{
enum UART_ID
{
  UART_CHASSIS_X = 1,
  UART_CHASSIS_Y,
  UART_CHASSIS_Z,
  UART_YAW,
  UART_PITCH,
  UART_MODES
};

enum CAN_ID
{
  CAN_CHASSIS_X_Y = 0x711,
  CAN_CHASSIS_Z_YAW,
  CAN_PITCH_MODES,
  CAN_YAW_CMD_ACT,
  CAN_CHASSIS_FRONT_CMD,
  CAN_CHASSIS_BACK_CMD,
  CAN_CHASSIS_FRONT_ACT,
  CAN_CHASSIS_BACK_ACT,
  PITCH_TX = 0x42,    // DM3507 MIT command ID (= Slave/CAN_ID; tool 0x142 stored 8-bit → 0x42)
  PITCH_RX = 0x52,    // DM3507 feedback ID (= Master/MST_ID; tool 0x152 stored 8-bit → 0x52)
  YAW_TX = 0x141,
  YAW_RX = 0x141,
  CHASSIS_FRONT_RIGHT_TX = 0x43,
  CHASSIS_FRONT_RIGHT_RX = 0x53,
  CHASSIS_FRONT_LEFT_TX = 0x44,
  CHASSIS_FRONT_LEFT_RX = 0x54,
  CHASSIS_BACK_RIGHT_TX = 0x45,
  CHASSIS_BACK_RIGHT_RX = 0x55,
  CHASSIS_BACK_LEFT_TX = 0x46,
  CHASSIS_BACK_LEFT_RX = 0x56
};

enum MODE_ID
{
  LOAD,
  FIRE,
  SPEED,
  CHASSIS
};

class ImuInfo
{
public:
  float acc_x_;    // [m/s²]
  float acc_y_;
  float acc_z_;
  float gyro_x_;   // [rad/s]
  float gyro_y_;
  float gyro_z_;
  float euler_heading_;  // [rad]
  float euler_roll_;     // [rad]
  float euler_pitch_;    // [rad]
};

class MotorInfo
{
public:
  float position_;        // [rad]
  float velocity_;        // [rad/s]
  float torque_;          // [Nm] DM only
  float current_;         // [A] LK only
  float temperature_;     // [°C] LK: motor, DM: rotor
  float temperature_mos_; // [°C] DM only
};

class Info
{
public:
  // reference
  float chassis_vel_x_;
  float chassis_vel_y_;
  float chassis_vel_z_;
  float yaw_pos_;
  float pitch_pos_;
  uint8_t load_mode_;    // 0:Stop, 1:Forward, 2:Reverse
  uint8_t fire_mode_;    // 0:Stop, 1:Low, 2:High
  uint8_t speed_mode_;    // 0:Low, 1:High
  uint8_t chassis_mode_;    // 0:Nomal, 1:InfiniteRot

  // command
  MotorInfo yaw_cmd_;
  MotorInfo pitch_cmd_;
  MotorInfo chassis_fr_cmd_;
  MotorInfo chassis_fl_cmd_;
  MotorInfo chassis_br_cmd_;
  MotorInfo chassis_bl_cmd_;

  // imu
  ImuInfo imu_;

  // actual
  MotorInfo yaw_act_;
  MotorInfo pitch_act_;
  MotorInfo chassis_fr_act_;
  MotorInfo chassis_fl_act_;
  MotorInfo chassis_br_act_;
  MotorInfo chassis_bl_act_;
};
}  // namespace rabcl

#endif  // RABCL__UTILS__TYPE_HPP_
