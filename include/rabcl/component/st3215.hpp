// Feetech ST3215（Waveshare 版）バスサーボ。半二重 UART、1 Mbps、ID 1 が出荷時。
// 設計書 docs/superpowers/specs/2026-10-25-st3215-design.md。
//
// 命令パケット: FF FF ID LEN INST [ADDR DATA…] CHK。LEN = パラメータ数 + 2、CHK = ~(ID + LEN + INST + ADDR + DATA…)。
// 応答:        FF FF ID LEN ERR [DATA…] CHK。
// 16 bit はリトルエンディアン。負値は bit15 の符号付き絶対値（現在負荷だけ bit10）。
// レジスタ番地と命令コードは Waveshare 配布の SCServo ライブラリ（SMS_STS.h / INST.h）と同じ。
#ifndef RABCL__COMPONENT__ST3215_HPP_
#define RABCL__COMPONENT__ST3215_HPP_

#include <cstddef>
#include <cstdint>

namespace rabcl
{

class St3215Protocol
{
public:
  static constexpr uint8_t INST_PING = 0x01;
  static constexpr uint8_t INST_READ = 0x02;
  static constexpr uint8_t INST_WRITE = 0x03;
  static constexpr uint8_t INST_REG_WRITE = 0x04;
  static constexpr uint8_t INST_ACTION = 0x05;
  static constexpr uint8_t INST_SYNC_WRITE = 0x83;

  static constexpr uint8_t REG_MODEL = 3;                // 2 B
  static constexpr uint8_t REG_ID = 5;
  static constexpr uint8_t REG_BAUD = 6;                 // 0 = 1 Mbps
  static constexpr uint8_t REG_MIN_ANGLE = 9;            // 2 B
  static constexpr uint8_t REG_MAX_ANGLE = 11;           // 2 B
  static constexpr uint8_t REG_OFS = 31;                 // 2 B
  static constexpr uint8_t REG_MODE = 33;                // Mode
  static constexpr uint8_t REG_TORQUE_ENABLE = 40;
  static constexpr uint8_t REG_ACC = 41;
  static constexpr uint8_t REG_GOAL_POSITION = 42;       // 2 B（ステップモードでは相対量）
  static constexpr uint8_t REG_GOAL_TIME = 44;           // 2 B
  static constexpr uint8_t REG_GOAL_SPEED = 46;          // 2 B
  static constexpr uint8_t REG_TORQUE_LIMIT = 48;        // 2 B
  static constexpr uint8_t REG_LOCK = 55;                // 0 = EEPROM 書き込み可
  static constexpr uint8_t REG_PRESENT_POSITION = 56;    // 2 B
  static constexpr uint8_t REG_PRESENT_SPEED = 58;       // 2 B
  static constexpr uint8_t REG_PRESENT_LOAD = 60;        // 2 B（bit10 が符号）
  static constexpr uint8_t REG_PRESENT_VOLTAGE = 62;     // 0.1 V
  static constexpr uint8_t REG_PRESENT_TEMPERATURE = 63;  // ℃
  static constexpr uint8_t REG_MOVING = 66;
  static constexpr uint8_t REG_PRESENT_CURRENT = 69;     // 2 B

  enum class Mode : uint8_t
  {
    POSITION = 0,
    WHEEL = 1,
    PWM = 2,
    STEP = 3,
  };

  static constexpr size_t MAX_PACKET = 32;
  static constexpr size_t MAX_DATA = 16;
  static constexpr size_t HEADER_SEARCH = 10;  // FF FF を探す範囲

  static size_t BuildPing(uint8_t id, uint8_t * out);                            // 6 B
  static size_t BuildRead(uint8_t id, uint8_t reg, uint8_t len, uint8_t * out);  // 8 B
  static size_t BuildWrite(
    uint8_t id, uint8_t reg, const uint8_t * data, uint8_t len, uint8_t * out);
  static size_t ResponseLength(uint8_t data_len) {return 6 + data_len;}
  static uint16_t EncodeSigned(int16_t v);   // 符号付き絶対値（bit15）
  static int16_t DecodeSigned(uint16_t raw);
  static int16_t DecodeLoad(uint16_t raw);   // 負荷だけ bit10 が符号
  static void PutU16(uint16_t v, uint8_t * out);  // リトルエンディアン
  static uint16_t GetU16(const uint8_t * in);

  struct Response
  {
    uint8_t id;
    uint8_t error;
    uint8_t len;  // DATA のバイト数
    uint8_t data[MAX_DATA];
  };
  // buf の先頭 HEADER_SEARCH バイト以内の FF FF から 1 パケットを取り出す。
  // 長さ不足・チェックサム不一致・DATA が MAX_DATA 超なら false。consumed = 使ったバイト数
  static bool ParseResponse(const uint8_t * buf, size_t n, Response & out, size_t & consumed);
};

// 半二重バス 1 本。送って、期待する長さの応答を timeout_ms まで待つ。
// 返り値 = 受けたバイト数（エコーは除く）、負はエラー。実装は機体側（rabcl_esp32 の HalfDuplexSerial を包む）
class St3215Port
{
public:
  virtual ~St3215Port() = default;
  virtual int Transfer(
    const uint8_t * tx, size_t tx_len, uint8_t * rx, size_t rx_len, uint32_t timeout_ms) = 0;
};

struct St3215Status
{
  int16_t position;     // カウント（4096 / 回転）
  int16_t speed;        // カウント/s
  int16_t load;         // 0.1 %（bit10 符号）
  uint8_t voltage;      // 0.1 V
  uint8_t temperature;  // ℃
  bool moving;
  int16_t current;      // 6.5 mA
};

// サーボ 1 個
class St3215
{
public:
  St3215(St3215Port & port, uint8_t id, uint32_t timeout_ms = 10);

  bool Ping();
  bool Read(uint8_t reg, uint8_t * data, uint8_t len);
  bool ReadU8(uint8_t reg, uint8_t & v);
  bool ReadU16(uint8_t reg, uint16_t & v);
  bool ReadS16(uint8_t reg, int16_t & v);
  bool Write(uint8_t reg, const uint8_t * data, uint8_t len);
  bool WriteU8(uint8_t reg, uint8_t v);
  bool WriteU16(uint8_t reg, uint16_t v);
  bool SetMode(St3215Protocol::Mode mode);  // LOCK 0 → MODE → LOCK 1
  bool SetTorque(bool on);
  // ACC〜GOAL_SPEED の 7 バイトを 1 回で書く。位置モードは絶対位置、ステップモードは相対カウント
  bool Move(int16_t position_or_steps, uint16_t speed, uint8_t acc);
  bool ReadStatus(St3215Status & out);  // 56〜70 の 15 バイトを 1 回の READ で

  uint8_t id() const {return id_;}
  uint8_t last_error() const {return last_error_;}  // 応答の ERR
  uint32_t timeouts() const {return timeouts_;}     // 応答が揃わなかった回数

private:
  // 送って応答（DATA expect バイト）を受ける。失敗なら false
  bool Exchange(const uint8_t * tx, size_t tx_len, uint8_t expect, St3215Protocol::Response & resp);

  St3215Port & port_;
  uint8_t id_;
  uint32_t timeout_ms_;
  uint8_t last_error_ = 0;
  uint32_t timeouts_ = 0;
};

}  // namespace rabcl

#endif  // RABCL__COMPONENT__ST3215_HPP_
