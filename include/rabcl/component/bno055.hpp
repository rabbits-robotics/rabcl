// BNO055（Bosch の 9 軸 IMU、センサ融合内蔵）。
//   BNO055      レジスタ番地と生データの換算（バスに依らない）
//   BNO055Uart  UART モード（PS1 = High、115200 8N1）のフレームの組み立てと受信パーサ
//   BNO055Sequencer  起動シーケンス・周期読み・再初期化の状態機械（バスに依らない）
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

  // UNIT_SEL で rad / rad/s にしたときの換算（900 LSB/rad、900 LSB/(rad/s)）
  static float ScaleGyroRad(int16_t raw);
  static float ScaleEulerRad(int16_t raw);

private:
  static constexpr float RAD_LSB = 900.0f;
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

// --- バスに依らない状態機械 ---

struct BNO055Config
{
  uint8_t axis_map_config = 0x24;    // AXIS_MAP_CONFIG（既定: X→X, Y→Y, Z→Z）
  uint8_t axis_map_sign = 0x00;      // AXIS_MAP_SIGN
  uint8_t operation_mode = 0x08;     // OPR_MODE（既定 IMU: 加速度 + ジャイロ、磁気なし）
  bool use_external_crystal = false;  // SYS_TRIGGER の CLK_SEL。モジュールの水晶を確認したら true
  uint32_t sample_period_ms = 10;    // RUNNING の読み周期（融合出力は 100 Hz）
  uint32_t response_timeout_ms = 20;  // 応答待ち
  uint32_t boot_wait_ms = 700;       // 電源投入 / ソフトリセットから最初の要求まで（POR 400 / RST 650 を覆う）
  uint32_t clock_settle_ms = 20;     // CLK_SEL を書いたあとの待ち
  uint8_t max_consecutive_errors = 10;  // これを超えたらソフトリセットして初期化をやり直す
};

struct BNO055Sample
{
  float gyro[3];   // [rad/s] x, y, z（軸入れ替え後）
  float euler[3];  // [rad] heading, roll, pitch（BNO055 の約束のまま。heading は時計回りが正）
  uint8_t calib_stat;  // 直近の CALIB_STAT
};

// 状態機械が出す操作
struct BNO055Op
{
  enum class Kind : uint8_t
  {
    READ,
    WRITE,
  };
  Kind kind;
  uint8_t reg;
  uint8_t len;
  uint8_t data[8];
  bool expect_response;  // false = 応答を待たない（ソフトリセットだけ）
};

// 操作の結果
struct BNO055Result
{
  static constexpr uint8_t ERROR_NONE = 0;
  static constexpr uint8_t ERROR_TIMEOUT = 0xFF;
  bool ok;
  uint8_t len;
  uint8_t data[BNO055Uart::MAX_READ];
  uint8_t error;  // 0 = なし。UART では EE の status、I2C ではバスの失敗コード
};

// 起動シーケンス・周期読み・エラーと再初期化。「次の操作」を出し「結果」を受けるだけで、バスも時計も知らない
class BNO055Sequencer
{
public:
  enum class State : uint8_t
  {
    BOOT_WAIT,
    CHECK_ID,
    TO_CONFIG,
    SET_PAGE,
    SET_UNITS,
    SET_AXIS_MAP,
    SET_AXIS_SIGN,
    CHECK_CLOCK,
    SET_CLOCK,
    TO_FUSION,
    CHECK_STATUS,
    READ_SYS_ERR,
    RUNNING,
    SOFT_RESET,  // 終端状態は無い。ここから BOOT_WAIT に戻る
  };

  static constexpr uint8_t REG_CHIP_ID = 0x00;
  static constexpr uint8_t REG_PAGE_ID = 0x07;
  static constexpr uint8_t REG_GYR_DATA = 0x14;   // ここから 12 B でジャイロ + オイラー
  static constexpr uint8_t REG_CALIB_STAT = 0x35;
  static constexpr uint8_t REG_SYS_CLK_STATUS = 0x38;
  static constexpr uint8_t REG_SYS_STATUS = 0x39;
  static constexpr uint8_t REG_SYS_ERR = 0x3A;
  static constexpr uint8_t REG_UNIT_SEL = 0x3B;
  static constexpr uint8_t REG_OPR_MODE = 0x3D;
  static constexpr uint8_t REG_SYS_TRIGGER = 0x3F;
  static constexpr uint8_t REG_AXIS_MAP_CONFIG = 0x41;
  static constexpr uint8_t REG_AXIS_MAP_SIGN = 0x42;
  static constexpr uint8_t CHIP_ID = 0xA0;
  static constexpr uint8_t UNIT_SEL_RAD = 0x06;        // ジャイロ rad/s、オイラー rad、Windows の約束
  static constexpr uint8_t SYS_STATUS_FUSION = 5;
  static constexpr uint8_t TRIGGER_RST_SYS = 0x20;
  static constexpr uint8_t TRIGGER_CLK_SEL = 0x80;
  static constexpr uint8_t DATA_LEN = 12;
  static constexpr uint32_t CONFIG_SWITCH_MS = 19;     // 動作モード → CONFIG
  static constexpr uint32_t FUSION_SWITCH_MS = 7;      // CONFIG → 動作モード
  static constexpr uint32_t CALIB_EVERY_READS = 100;   // 100 回に 1 回 CALIB_STAT を読む

  explicit BNO055Sequencer(const BNO055Config & config);

  // 時刻を進め、出すべき操作があれば op に入れて true。応答待ち中は、timeout 以上経っていれば
  // タイムアウトとして処理（エラー +1、同じ操作を再発行）、まだなら false。待機中も false
  bool Next(uint32_t now_ms, BNO055Op & op);
  // 直前の操作の結果。データ読みで揃ったサンプルなら sample に入れて true
  bool Complete(const BNO055Result & result, uint32_t now_ms, BNO055Sample & sample);

  bool waiting() const {return waiting_;}
  const BNO055Op & pending() const {return pending_;}
  uint32_t last_op_ms() const {return issued_ms_;}
  State state() const {return state_;}
  bool running() const {return state_ == State::RUNNING;}
  uint32_t error_count() const {return error_count_;}
  uint32_t reinit_count() const {return reinit_count_;}
  uint8_t calib_stat() const {return calib_stat_;}
  uint8_t sys_err() const {return sys_err_;}
  void Restart(uint32_t now_ms);  // SOFT_RESET へ

private:
  void Issue(const BNO055Op & op, uint32_t now_ms);
  void IssueWrite(uint8_t reg, uint8_t value, uint32_t now_ms, bool expect_response = true);
  void IssueRead(uint8_t reg, uint8_t len, uint32_t now_ms);
  void Fail(uint32_t now_ms);          // 連続エラー +1、しきい値でソフトリセットへ
  void Succeed();                      // 連続エラー 0
  void EnterBootWait(uint32_t now_ms);

  BNO055Config config_;
  State state_ = State::BOOT_WAIT;
  uint32_t boot_start_ms_ = 0;  // 起動待ちの起点（時刻 0 = マイコン起動 = BNO055 の POR。ソフトリセット後はその時刻）
  uint32_t wait_until_ms_ = 0;         // この時刻まで次の操作を出さない（モード切替・クロックの待ち）
  bool waiting_ = false;
  BNO055Op pending_ = {};
  uint32_t issued_ms_ = 0;
  uint32_t next_read_ms_ = 0;
  uint32_t read_count_ = 0;
  uint32_t error_count_ = 0;
  uint32_t reinit_count_ = 0;
  uint8_t calib_stat_ = 0;
  uint8_t sys_err_ = 0;
};


}  // namespace rabcl

#endif  // RABCL__COMPONENT__BNO055_HPP_
