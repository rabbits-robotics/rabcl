#ifndef RABCL__INTERFACE__CAN_HPP_
#define RABCL__INTERFACE__CAN_HPP_

#include <cstdint>

#include "rabcl/utils/type.hpp"

namespace rabcl
{
class Can
{
public:
  // DM decode/encode parameters (must match DM AC-Tools settings)
  // DM2325 (chassis wheels)
  static constexpr float DM2325_PMAX = 78.5f;   // [rad] motor shaft
  static constexpr float DM2325_VMAX = 200.0f;  // [rad/s] motor shaft
  static constexpr float DM2325_TMAX = 3.0f;    // [Nm] output shaft
  static constexpr float DM2325_GR = 25.0f;     // gear ratio
  // DM3507 / DM-J3507-2EC (pitch axis); position/vel/torque are output-shaft → GR=1 in decode
  static constexpr float DM3507_PMAX = 12.566f;  // [rad] (= 4*pi)
  static constexpr float DM3507_VMAX = 50.0f;    // [rad/s]
  static constexpr float DM3507_TMAX = 5.0f;     // [Nm]
  static constexpr float DM3507_GR = 7.0f;       // gear ratio (informational)
  // DM4310 / DM-J4310-2EC (gamma legs); output-shaft values → GR=1 in decode
  static constexpr float DM4310_PMAX = 12.5f;   // [rad]
  static constexpr float DM4310_VMAX = 30.0f;   // [rad/s]
  static constexpr float DM4310_TMAX = 10.0f;   // [Nm]
  static constexpr float DM4310_GR = 10.0f;     // gear ratio (informational)
  // MIT-mode gain ranges (DAMIAO standard)
  static constexpr float DM_KP_MAX = 500.0f;
  static constexpr float DM_KD_MAX = 5.0f;

public:
  Can();
  ~Can();
  static bool UpdateData(
    uint32_t idx,
    const uint8_t can_data[8],
    Info & data,
    float yaw_offset /* [rad] */ = 0.0f,
    float pitch_offset /* [rad] */ = 0.0f);
  static void Prepare2FloatData(float in_1, float in_2, uint8_t can_data[8]);
  static void Prepare1Float4IntData(float in_1, const uint8_t in_2[4], uint8_t can_data[8]);
  static void PrepareLKMotorMotorOff(uint8_t can_data[8]);
  static void PrepareLKMotorMotorOn(uint8_t can_data[8]);
  static void PrepareLKMotorMotorStop(uint8_t can_data[8]);
  static void PrepareLKMotorTorqueCmd(
    int16_t current /* 0.01A/LSB */,
    uint8_t can_data[8]);
  static void PrepareLKMotorPositionCmd(
    int32_t pos /* [0.01 deg] */,
    uint16_t max_speed /* [dps] */,
    uint8_t can_data[8]);
  static void PrepareLKMotorReadParam(uint8_t param_id, uint8_t can_data[8]);
  static void PrepareLKMotorWritePID(
    uint8_t param_id,
    uint16_t kp, uint16_t ki, uint16_t kd,
    uint8_t can_data[8]);
  static void PrepareRMDMotorReadPID(uint8_t can_data[8]);
  static void PrepareRMDMotorWritePIDToRAM(
    uint8_t curr_kp, uint8_t curr_ki,
    uint8_t speed_kp, uint8_t speed_ki,
    uint8_t pos_kp, uint8_t pos_ki,
    uint8_t can_data[8]);
  static void PrepareLKMotorReadMotorState2(uint8_t can_data[8]);  // 0x9C: temp/current/speed/position (same format as 0xA4 response)
  static void PrepareDMMotorEnable(uint8_t can_data[8]);
  static void PrepareDMMotorVelocityCmd(
    float velocity /* [rad/s, output shaft] */,
    uint8_t can_data[8]);
  static void PrepareDMMotorMITCmd(
    float pos /* [rad] */, float vel /* [rad/s] */,
    float kp, float kd, float tff /* [Nm] */,
    float pmax, float vmax, float tmax,
    uint8_t can_data[8]);

  // Feedback parsers. UpdateData() dispatches by the beta CAN_ID table; firmware with its own
  // ID table (gamma) calls these directly.
  static void ParseLKMotorFeedback(const uint8_t can_data[8], MotorInfo & motor);
  static void ParseDMMotorFeedback(
    const uint8_t can_data[8], MotorInfo & motor,
    float pmax, float vmax, float tmax, float gr);
  // DM feedback D[0] = ID | ERR << 4
  static uint8_t DMFeedbackId(const uint8_t can_data[8]);
  // 0: disabled, 1: enabled, 8: overvoltage, 9: undervoltage, A: overcurrent,
  // B: MOS overtemperature, C: rotor overtemperature, D: communication lost, E: overload
  static uint8_t DMFeedbackError(const uint8_t can_data[8]);
};
}  // namespace rabcl

#endif  // RABCL__INTERFACE__CAN_HPP_
