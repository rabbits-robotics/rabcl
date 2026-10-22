#include <cmath>
#include <gtest/gtest.h>
#include "rabcl/interface/can.hpp"
#include "rabcl/utils/type.hpp"

namespace rabcl
{

class CanTest : public ::testing::Test
{
protected:
    // float を 8バイト CAN フレームに書き込むヘルパー（前半4バイト）
  static void PackFloat(float value, uint8_t * buf, int offset = 0)
  {
    union { float f; int32_t ui; } b;
    b.f = value;
    buf[offset + 0] = static_cast<uint8_t>((b.ui >> 24) & 0xFF);
    buf[offset + 1] = static_cast<uint8_t>((b.ui >> 16) & 0xFF);
    buf[offset + 2] = static_cast<uint8_t>((b.ui >> 8) & 0xFF);
    buf[offset + 3] = static_cast<uint8_t>((b.ui >> 0) & 0xFF);
  }
};

TEST_F(CanTest, UpdateDataChassisXY)
{
  uint8_t data[8] = {};
  PackFloat(1.0f, data, 0);
  PackFloat(2.0f, data, 4);
  Info info;
  EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_X_Y), data, info));
  EXPECT_FLOAT_EQ(info.chassis_vel_x_, 1.0f);
  EXPECT_FLOAT_EQ(info.chassis_vel_y_, 2.0f);
}

TEST_F(CanTest, UpdateDataChassisZYaw)
{
  uint8_t data[8] = {};
  PackFloat(3.0f, data, 0);
  PackFloat(4.0f, data, 4);
  Info info;
  EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_CHASSIS_Z_YAW), data, info));
  EXPECT_FLOAT_EQ(info.chassis_vel_z_, 3.0f);
  EXPECT_FLOAT_EQ(info.yaw_pos_, 4.0f);
}

TEST_F(CanTest, UpdateDataPitchModes)
{
  uint8_t data[8] = {};
  PackFloat(0.5f, data, 0);
  data[4 + static_cast<int>(MODE_ID::LOAD)] = 1;
  data[4 + static_cast<int>(MODE_ID::FIRE)] = 2;
  data[4 + static_cast<int>(MODE_ID::SPEED)] = 0;
  data[4 + static_cast<int>(MODE_ID::CHASSIS)] = 1;
  Info info;
  EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::CAN_PITCH_MODES), data, info));
  EXPECT_FLOAT_EQ(info.pitch_pos_, 0.5f);
  EXPECT_EQ(info.load_mode_, 1);
  EXPECT_EQ(info.fire_mode_, 2);
  EXPECT_EQ(info.speed_mode_, 0);
  EXPECT_EQ(info.chassis_mode_, 1);
}

TEST_F(CanTest, UpdateDataPitchRx)
{
  // DM3507 feedback (output-shaft, GR=1), max values → pos=PMAX, vel=VMAX, torque=TMAX
  // D[0]=id/err, D[1:2]=pos, D[3:4]=vel, D[4:5]=torque, D[6]=T_MOS, D[7]=T_Rotor
  uint8_t data[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x25, 0x00};
  Info info;
  EXPECT_TRUE(Can::UpdateData(static_cast<uint32_t>(CAN_ID::PITCH_RX), data, info));
  EXPECT_NEAR(info.pitch_act_.position_, Can::DM3507_PMAX, 0.01f);  // pos_raw=0xFFFF → +PMAX
  EXPECT_NEAR(info.pitch_act_.velocity_, Can::DM3507_VMAX, 0.05f);  // vel_raw=0xFFF  → +VMAX
  EXPECT_NEAR(info.pitch_act_.torque_, Can::DM3507_TMAX, 0.01f);    // tor_raw=0xFFF  → +TMAX
  EXPECT_FLOAT_EQ(info.pitch_act_.temperature_mos_, 37.0f);         // 0x25 = 37
}

TEST_F(CanTest, PrepareDMMotorMITCmd)
{
  // pos=0, vel=0, kp=0, kd=0, tff=0 → pos/vel/tff land on midpoints, kp/kd = 0
  uint8_t out[8] = {};
  Can::PrepareDMMotorMITCmd(
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    Can::DM3507_PMAX, Can::DM3507_VMAX, Can::DM3507_TMAX, out);

  uint16_t p = (static_cast<uint16_t>(out[0]) << 8) | out[1];
  uint16_t v = (static_cast<uint16_t>(out[2]) << 4) | (out[3] >> 4);
  uint16_t kp = (static_cast<uint16_t>(out[3] & 0x0F) << 8) | out[4];
  uint16_t kd = (static_cast<uint16_t>(out[5]) << 4) | (out[6] >> 4);
  uint16_t t = (static_cast<uint16_t>(out[6] & 0x0F) << 8) | out[7];

  EXPECT_NEAR(p, 32767, 1);  // 0 over [-PMAX,PMAX], 16-bit → midpoint
  EXPECT_NEAR(v, 2047, 1);   // 0 over [-VMAX,VMAX], 12-bit → midpoint
  EXPECT_EQ(kp, 0);
  EXPECT_EQ(kd, 0);
  EXPECT_NEAR(t, 2047, 1);   // 0 over [-TMAX,TMAX], 12-bit → midpoint
}

TEST_F(CanTest, ParseDMMotorFeedbackDM4310Max)
{
  // DM4310 (legs): output-shaft values, GR=1 in decode. Max raw → +PMAX / +VMAX / +TMAX
  uint8_t data[8] = {0x17, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x25, 0x30};
  MotorInfo motor = {};
  Can::ParseDMMotorFeedback(
    data, motor, Can::DM4310_PMAX, Can::DM4310_VMAX, Can::DM4310_TMAX, 1.0f);
  EXPECT_NEAR(motor.position_, Can::DM4310_PMAX, 0.01f);
  EXPECT_NEAR(motor.velocity_, Can::DM4310_VMAX, 0.05f);
  EXPECT_NEAR(motor.torque_, Can::DM4310_TMAX, 0.01f);
  EXPECT_FLOAT_EQ(motor.temperature_mos_, 37.0f);  // 0x25
  EXPECT_FLOAT_EQ(motor.temperature_, 48.0f);      // 0x30 = T_Rotor
}

TEST_F(CanTest, ParseDMMotorFeedbackMidpointIsZero)
{
  uint8_t data[8] = {0x07, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x00, 0x00};
  MotorInfo motor = {};
  Can::ParseDMMotorFeedback(
    data, motor, Can::DM4310_PMAX, Can::DM4310_VMAX, Can::DM4310_TMAX, 1.0f);
  EXPECT_NEAR(motor.position_, 0.0f, 0.001f);
  EXPECT_NEAR(motor.velocity_, 0.0f, 0.01f);
  EXPECT_NEAR(motor.torque_, 0.0f, 0.01f);
}

TEST_F(CanTest, DMFeedbackIdAndError)
{
  // D[0] = ID | ERR << 4
  uint8_t data[8] = {0x87, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_EQ(Can::DMFeedbackId(data), 0x07);
  EXPECT_EQ(Can::DMFeedbackError(data), 0x08);  // 8 = overvoltage
}

TEST_F(CanTest, ParseLKMotorFeedbackPublic)
{
  // D[1]=temp, D[2:3]=current(0.01A), D[4:5]=speed(dps), D[6:7]=pos(uint16 → 0~2π)
  uint8_t data[8] = {0xA1, 0x28, 0x64, 0x00, 0xB4, 0x00, 0xFF, 0xFF};
  MotorInfo motor = {};
  Can::ParseLKMotorFeedback(data, motor);
  EXPECT_FLOAT_EQ(motor.temperature_, 40.0f);
  EXPECT_NEAR(motor.current_, 1.0f, 0.001f);
  EXPECT_NEAR(motor.velocity_, static_cast<float>(M_PI), 0.001f);  // 180 dps
  EXPECT_NEAR(motor.position_, 2.0f * static_cast<float>(M_PI), 0.001f);
}

TEST_F(CanTest, UpdateDataUnknownIdx)
{
  uint8_t data[8] = {};
  Info info;
  EXPECT_FALSE(Can::UpdateData(0xFFFF, data, info));
}

TEST_F(CanTest, Prepare2FloatData)
{
  uint8_t out[8] = {};
  Can::Prepare2FloatData(1.0f, 2.0f, out);

  uint8_t expected[8] = {};
  PackFloat(1.0f, expected, 0);
  PackFloat(2.0f, expected, 4);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(out[i], expected[i]);
  }
}

TEST_F(CanTest, Prepare1Float4IntData)
{
  uint8_t out[8] = {};
  uint8_t modes[4] = {1, 2, 3, 4};
  Can::Prepare1Float4IntData(1.0f, modes, out);

  uint8_t expected[8] = {};
  PackFloat(1.0f, expected, 0);
  expected[4] = 1; expected[5] = 2; expected[6] = 3; expected[7] = 4;
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(out[i], expected[i]);
  }
}

TEST_F(CanTest, PrepareLKMotorTorqueCmd)
{
  uint8_t out[8] = {};
  Can::PrepareLKMotorTorqueCmd(0x0102, out);

  EXPECT_EQ(out[0], 0xA1);
  EXPECT_EQ(out[1], 0x00);
  EXPECT_EQ(out[2], 0x00);
  EXPECT_EQ(out[3], 0x00);
  EXPECT_EQ(out[4], 0x02);  // current low byte
  EXPECT_EQ(out[5], 0x01);  // current high byte
  EXPECT_EQ(out[6], 0x00);
  EXPECT_EQ(out[7], 0x00);
}

TEST_F(CanTest, PrepareLKMotorTorqueCmdNegative)
{
  uint8_t out[8] = {};
  Can::PrepareLKMotorTorqueCmd(-500, out);

  EXPECT_EQ(out[0], 0xA1);
  int16_t reconstructed = static_cast<int16_t>(
    (static_cast<uint16_t>(out[5]) << 8) | out[4]);
  EXPECT_EQ(reconstructed, -500);
}

TEST_F(CanTest, PrepareLKMotorPositionCmd)
{
  uint8_t out[8] = {};
  Can::PrepareLKMotorPositionCmd(0x00010203, 0x0405, out);

  EXPECT_EQ(out[0], 0xA4);
  EXPECT_EQ(out[1], 0x00);
  EXPECT_EQ(out[2], 0x05);  // max_speed low byte
  EXPECT_EQ(out[3], 0x04);  // max_speed high byte
  EXPECT_EQ(out[4], 0x03);  // pos byte0
  EXPECT_EQ(out[5], 0x02);  // pos byte1
  EXPECT_EQ(out[6], 0x01);  // pos byte2
  EXPECT_EQ(out[7], 0x00);  // pos byte3
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
