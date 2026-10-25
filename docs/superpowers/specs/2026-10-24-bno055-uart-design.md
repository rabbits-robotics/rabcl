# BNO055 の UART ドライバと gamma の IMU タスク 設計

2026-10-24。対象: rabcl（`component/bno055`、機体に依らない部分）、gamma_turret_fw（IMU タスク、定数）、rabbits_description（`imu_link`）。

## 1. 目的

首の安定化（`BodyAttitude` → `Gimbal`）に使う姿勢と角速度を、BNO055 から **できるだけ速く・途切れず** 供給する。移動しながら無限回転しながら狙うため、姿勢の更新が遅いと視点が流れる。

背景: gamma_turret 基板には BNO055（秋月 AE-BNO055-BO）が UART で直載せされている（モジュールの送信 → P4 の G15 = UART3 RX、受信 ← G12 = UART3 TX、115200 bps 固定、3.3V、RESET / INT は未接続）。rabcl の既存 `BNO055` は I2C 時代の「6 バイトの生データ → 加速度 / ジャイロ / オイラー角の換算」だけで、通信も起動シーケンスも持たない。ファーム側は `ImuStore`（`src/imu.hpp`）が置き場として用意済みで、`Controller::Tick` は `valid` が立つまで旋回速度の積分で胴体の向きを補っている（制御設計書 4.3）。

実機のモジュールは電源系の内部短絡で不良、再注文中。今回は rabcl のユニットテストとファームのビルドまで。疎通は新しいモジュールが届いてから。

## 2. ゴール・非ゴール・アンチゴール

ゴール:

- 融合出力 100 Hz（BNO055 の上限）を落とさずに読む。1 周期でジャイロ 3 軸 + オイラー角 3 軸をまとめて取り、`ImuStore` に姿勢（REP-103、カメラ座標系）・角速度・有効フラグ・計測時刻・計測時の首 Yaw 角を書く。
- 起動時に待たない。IMU は `valid = false` で始め、初期化が終わった時点から使う。ゼロ点合わせの静止待ちはしない（復電後に素早く動き出す要件）。
- 通信が途絶・エラー続きなら `valid = false` に落とし、再初期化を繰り返す。制御側は `error_flags` bit4 で分かる。
- 軸の入れ替え（基板の取り付け向き）は BNO055 のレジスタで行い、値は `robot.hpp` の定数。本番の向きと今の試験の向きの 2 組を README に書き、実機で手順を踏んで確定する。
- rabcl の部分（プロトコルの組み立てと解釈、起動シーケンスの状態機械、換算）は UART も時計も知らず、バイト列でユニットテストする。

非ゴール:

- 磁気を使った方位（NDOF）。モータの磁気で乱れるので使わない。方位は起動時（モード投入時）の向きが基準。
- クォータニオンの利用、加速度の利用（将来の脚のダンパ制御で使うかもしれないが今回は読まない。読み足すのは 6 バイト増やすだけ）。
- IMU の割り込み（INT 未接続）、ハードリセット（RESET 未接続）。
- SPI の IMU への差し替え（基板 v1.1 の話。読み出しの口 `ImuStore` は共通にしてある）。

アンチゴール:

- IMU の初期化や再接続のためにモータの起動や指令が遅れる。
- rabcl に gamma の値（軸の向き、ピン）が入る。
- UART のタイミングに依存したコードが rabcl に入り、PC でテストできなくなる。

## 3. BNO055 の事実（データシート 1.8 と秋月の説明書より。実機で裏取りする）

UART プロトコル（PS1 = High。115200 8N1）:

| 向き | 形 |
|---|---|
| 書き要求 | `0xAA 0x00 reg len data[len]` |
| 読み要求 | `0xAA 0x01 reg len`（len ≤ 128） |
| 読み応答 | `0xBB len data[len]` |
| 書き応答 / エラー | `0xEE status`。status: `0x01` WRITE_SUCCESS、`0x02` READ_FAIL、`0x03` WRITE_FAIL、`0x04` REGMAP_INVALID_ADDRESS、`0x05` REGMAP_WRITE_DISABLED、`0x06` WRONG_START_BYTE、`0x07` BUS_OVER_RUN_ERROR、`0x08` MAX_LENGTH_ERROR、`0x09` MIN_LENGTH_ERROR、`0x0A` RECEIVE_CHARACTER_TIMEOUT |

- 1 回の要求に 1 回の応答。応答を受け取る前に次の要求を送ると `0x07` が返りやすい（既知の癖）。1 周期 = 読み要求 4 B + 応答 14 B = 18 B ≈ 1.6 ms（115200 bps）。100 Hz（10 ms）に十分収まる。
- 応答待ちのタイムアウトは 20 ms とする（通常 2 ms 以内に来る。超えたら要求を捨てて次の周期へ）。

レジスタ（ページ 0）:

| 名前 | アドレス | 内容 |
|---|---|---|
| CHIP_ID | 0x00 | 0xA0 |
| PAGE_ID | 0x07 | 0 にしておく |
| GYR_DATA_X_LSB〜GYR_DATA_Z_MSB | 0x14〜0x19 | ジャイロ 3 軸、int16 LE |
| EUL_HEADING_LSB〜EUL_PITCH_MSB | 0x1A〜0x1F | オイラー角 heading, roll, pitch、int16 LE |
| CALIB_STAT | 0x35 | bit7:6 SYS、bit5:4 GYR、bit3:2 ACC、bit1:0 MAG（3 = 済） |
| SYS_STATUS / SYS_ERR | 0x39 / 0x3A | 5 = 融合動作中 / 0 = エラーなし |
| UNIT_SEL | 0x3B | bit0 加速度（0 = m/s²）、bit1 ジャイロ（1 = rad/s）、bit2 オイラー（1 = rad）、bit7 向きの約束（0 = Windows、1 = Android） |
| OPR_MODE | 0x3D | 0x00 CONFIG、0x08 IMU（加速度 + ジャイロ、磁気なし）、0x0C NDOF |
| SYS_CLK_STATUS | 0x38 | bit0 = 0 のときクロック設定を変えてよい |
| SYS_TRIGGER | 0x3F | bit5 RST_SYS（ソフトリセット。UART では ACK が返らず即リセット、その後 650 ms）、bit7 CLK_SEL（1 = 外部 32 kHz 水晶。AE-BNO055-BO に水晶が載っているかは秋月の回路図で未確認 → 確認するまで 0 = 内蔵） |
| AXIS_MAP_CONFIG / AXIS_MAP_SIGN | 0x41 / 0x42 | 軸の入れ替え（既定 0x24 = X→X, Y→Y, Z→Z）と符号（既定 0x00） |

- 換算: rad 設定のとき 900 LSB/rad、900 LSB/(rad/s)。ジャイロの生データは rad 設定でも 16 LSB/(°/s) のまま？ → **データシートでは UNIT_SEL の bit1 が 1 のとき 900 LSB/rad/s**。実機で両方の読みを比べて確定する（テストは定数を差し替えられる形にする）。
- 時間: 電源投入から CONFIG モードで応答できるまで約 400 ms（POR）、ソフトリセット後は約 650 ms。CONFIG → 融合モードの切り替えに 7 ms、融合 → CONFIG に 19 ms。レジスタの書き込みは CONFIG モードでだけ有効（UNIT_SEL、AXIS_MAP、PWR_MODE）。
- heading は 0〜2π で **時計回りが正**（上から見て右回りで増える）。REP-103 の yaw（左回りが正）とは逆。UNIT_SEL bit7 の約束（Windows / Android）で変わるのは **pitch の符号だけ**（heading と roll は同じ）。Windows（0）では pitch は時計回りで減る、と読めるが **実機で確認**（手順は 6 章）。UNIT_SEL の電源投入時の既定は 0x80（Android）なので、SET_UNITS で必ず書く。
- CLK_SEL を書いたあとの待ちはデータシートに数値がない（Adafruit は 10 ms）。設定値（既定 20 ms）にする。書く前に SYS_CLK_STATUS bit0 = 0 を確認する。
- IMU モードの方位は起動時の向きが 0。ジャイロのゼロ点は静止を検出したとき自動で合う（CALIB_STAT の GYR が 3 になる）。合う前は方位が流れる（数 °/分 程度）。

## 4. rabcl（機体に依らない部分）

`include/rabcl/component/bno055.hpp` を拡張する。既存の `BNO055`（レジスタ番地と換算。バスに依らない。`I2C_ADDR` は I2C で使うときのアドレスとコメントする）は残す。新しい部品は名前を既存に合わせて `BNO055…` とし、**バスに依らない状態機械**と **バスごとの薄い層**に分ける。

```
BNO055Sequencer（起動シーケンス・周期読み・エラー / 再初期化。「次の操作」を出し「結果」を受ける）
   ├── BNO055UartDriver = BNO055Uart（フレームの組み立て / 解釈）+ Sequencer   ← gamma（UART）
   └── BNO055I2cDriver  = BNO055RegisterBus（抽象。レジスタ読み書きの口）+ Sequencer   ← 将来の I2C 基板（ESP32 の実バスは rabcl_esp32 に後で）
```

### 4.1 `BNO055Uart` — UART フレームの組み立てと受信パーサ

```cpp
namespace rabcl {
class BNO055Uart {
public:
  static constexpr uint8_t START = 0xAA, RESPONSE = 0xBB, ACK = 0xEE;
  static constexpr size_t MAX_READ = 128;
  enum class Status : uint8_t { WRITE_SUCCESS = 0x01, READ_FAIL = 0x02, WRITE_FAIL = 0x03, REGMAP_INVALID_ADDRESS = 0x04,
    REGMAP_WRITE_DISABLED = 0x05, WRONG_START_BYTE = 0x06, BUS_OVER_RUN_ERROR = 0x07, MAX_LENGTH_ERROR = 0x08,
    MIN_LENGTH_ERROR = 0x09, RECEIVE_CHARACTER_TIMEOUT = 0x0A };
  struct Response { bool is_data; uint8_t len; uint8_t data[MAX_READ]; Status status; };
  static size_t BuildRead(uint8_t reg, uint8_t len, uint8_t * out);                        // AA 01 reg len → 4
  static size_t BuildWrite(uint8_t reg, const uint8_t * data, uint8_t len, uint8_t * out);  // AA 00 reg len data → 4 + len
  bool Push(uint8_t byte, Response & out);  // 揃った瞬間だけ true
  void Reset();
};
}
```

- `Push` は `BB len data…` と `EE status` の 2 形だけを受ける。先頭が `BB` / `EE` 以外のバイトは捨てる（同期ずれからの回復）。`len` が 0 または `MAX_READ` 超は捨てて先頭探しに戻る。

### 4.2 `BNO055Sequencer` — 起動シーケンスと周期読みの状態機械（バスも時計も知らない）

```cpp
namespace rabcl {
struct BNO055Config {
  uint8_t axis_map_config = 0x24; uint8_t axis_map_sign = 0x00; uint8_t operation_mode = 0x08;
  bool use_external_crystal = false;  // モジュールの水晶を確認したら true
  uint32_t sample_period_ms = 10;  // 0 = 周期は呼び出し側が持ち、RUNNING では Next のたびに読む（gamma はこれ。tick と二重管理しない）
  uint32_t response_timeout_ms = 20; uint32_t boot_wait_ms = 700;
  uint32_t clock_settle_ms = 20; uint8_t max_consecutive_errors = 10;
};
struct BNO055Sample { float gyro[3]; float euler[3]; uint8_t calib_stat; };  // gyro [rad/s] x,y,z。euler [rad] heading, roll, pitch（BNO055 の約束のまま）

struct BNO055Op {   // 状態機械が出す操作
  enum class Kind : uint8_t { READ, WRITE };
  Kind kind; uint8_t reg; uint8_t len; uint8_t data[8];
  bool expect_response;  // false = 応答を待たない（ソフトリセットだけ）
};
struct BNO055Result {  // 操作の結果
  bool ok; uint8_t len; uint8_t data[BNO055Uart::MAX_READ];
  uint8_t error;         // 0 = なし。UART では EE の status、I2C ではバスの失敗コード。TIMEOUT = 0xFF
};

class BNO055Sequencer {
public:
  enum class State : uint8_t { BOOT_WAIT, CHECK_ID, TO_CONFIG, SET_PAGE, SET_UNITS, SET_AXIS_MAP, SET_AXIS_SIGN,
    CHECK_CLOCK, SET_CLOCK, TO_FUSION, CHECK_STATUS, READ_SYS_ERR, RUNNING, SOFT_RESET };  // 終端状態は無い
  static const char * StateName(State);  // ログ用
  explicit BNO055Sequencer(const BNO055Config &);
  // 時刻を進め、出すべき操作があれば op に入れて true。応答待ち中は、timeout を超えていればタイムアウトとして
  // 処理（エラー +1、同じ操作を再発行）し、超えていなければ false。待機（BOOT_WAIT、周期待ち、モード切替待ち）中も false
  bool Next(uint32_t now_ms, BNO055Op & op);
  // 直前の操作の結果。データ読みで揃ったサンプルなら sample に入れて true
  bool Complete(const BNO055Result & result, uint32_t now_ms, BNO055Sample & sample);
  bool waiting() const;            // 応答待ち中
  const BNO055Op & pending() const;  // 待っている操作（UART 層が応答の長さを知るため）
  uint32_t last_op_ms() const;     // 最後に操作を出した時刻（ファームが time_us に使う）
  State state() const; bool running() const;
  uint32_t error_count() const; uint32_t reinit_count() const; uint8_t calib_stat() const; uint8_t sys_err() const;
  void Restart(uint32_t now_ms);   // SOFT_RESET へ
};
}
```

- 状態機械の順: BOOT_WAIT（`boot_wait_ms`。初回は電源投入からの POR 待ち）→ CHECK_ID（READ 0x00,1 → 0xA0）→ TO_CONFIG（WRITE 0x3D=0x00、成功後 19 ms 待ち）→ SET_PAGE（WRITE 0x07=0）→ SET_UNITS（WRITE 0x3B=0x06: ジャイロ rad/s、オイラー rad、Windows）→ SET_AXIS_MAP（WRITE 0x41）→ SET_AXIS_SIGN（WRITE 0x42）→ [`use_external_crystal` のときだけ] CHECK_CLOCK（READ 0x38,1 → bit0 = 0）→ SET_CLOCK（WRITE 0x3F=0x80、`clock_settle_ms` 待ち）→ TO_FUSION（WRITE 0x3D=operation_mode、7 ms 待ち）→ CHECK_STATUS（READ 0x39,1 → 5。1 = エラーなら READ 0x3A,1 を `sys_err` に保持してエラーに数え、2 / 3 = 初期化中ならエラーに数えず 10 ms 待って読み直す）→ RUNNING（`sample_period_ms` ごとに READ 0x14,12。100 回に 1 回は代わりに READ 0x35,1 → `calib_stat`。その周期のサンプルは落ちる）。
- エラー: タイムアウト、`error != 0`、期待値違い（CHIP_ID、SYS_STATUS、データ長）はすべて連続エラー +1、成功で 0。`> max_consecutive_errors` で SOFT_RESET: WRITE 0x3F=0x20（`expect_response = false`）→ BOOT_WAIT（`boot_wait_ms`）→ CHECK_ID から。`reinit_count` +1、`running()` false。終端状態は無く、何度でも繰り返す。`0x07`（BUS_OVER_RUN）も同じ扱いで、しきい値に達しない限り再初期化しない。
- 換算: `BNO055::ScaleGyroRad(int16)` / `ScaleEulerRad(int16)`（1/900）を足し、`Complete` がデータ読みの 12 B を `BNO055Sample` に直す。既存の `UpdateGyro` 等（°単位の定数）は残す。

### 4.3 バスごとの薄い層

```cpp
namespace rabcl {
class BNO055UartDriver {   // gamma で使う
public:
  explicit BNO055UartDriver(const BNO055Config &);
  size_t Tick(uint32_t now_ms, uint8_t * out, size_t cap);   // Sequencer::Next → BNO055Uart::Build* で out に（expect_response = false の操作は送って即 Complete(ok)）
  bool Feed(const uint8_t * data, size_t len, uint32_t now_ms, BNO055Sample & sample);  // 応答待ち中のバイトだけ Push → 揃ったら Complete。待ちでないバイトは捨てる
  size_t expected_response_len() const;  // 応答待ち中: READ なら 2 + len、WRITE なら 2。待ちでなければ 0
  uint32_t last_request_ms() const;      // = Sequencer::last_op_ms
  const BNO055Sequencer & sequencer() const;  // state / running / error_count / reinit_count / calib_stat / sys_err
  void Restart(uint32_t now_ms);
};

class BNO055RegisterBus {  // I2C など「レジスタをその場で読み書きできる」バスの口。実装は rabcl_esp32 などに
public:
  virtual ~BNO055RegisterBus() = default;
  virtual bool Read(uint8_t reg, uint8_t * data, uint8_t len) = 0;
  virtual bool Write(uint8_t reg, const uint8_t * data, uint8_t len) = 0;
};
class BNO055I2cDriver {   // 将来の I2C 基板用。今回は rabcl 側（抽象バスでのテスト）まで
public:
  BNO055I2cDriver(const BNO055Config &, BNO055RegisterBus & bus);
  bool Tick(uint32_t now_ms, BNO055Sample & sample);  // Sequencer::Next → bus.Read / Write（同期）→ Complete。サンプルなら true
  const BNO055Sequencer & sequencer() const;
  void Restart(uint32_t now_ms);
};
}
```

- UART のタイムアウト後に遅れて届いた応答は、`Feed` が「待ち中でなければ捨てる」「待ち中なら形と長さで検証する」ことで誤受理しない（再送した要求の応答と同じ形なら受けるが、それは正しい応答として扱ってよい）。**ただしこれはドライバに渡したバイトにしか効かない**: UART の受信バッファに残った古い応答は次の要求の応答として読まれ、以後ずっと 1 周期前の値を返し続ける（エラーにならない）。呼び出し側（ファーム）は **要求を送る直前に受信バッファを空にする**。
- I2C 側は同期なので `BNO055Result.error` にバスの失敗を入れるだけ。タイムアウトは起きない（バスの実装が自分で持つ）。

### 4.4 テスト（gtest、`test/test_bno055.cpp`）

- `BNO055Uart`: `BuildRead(0x14, 12)` → `AA 01 14 0C`、`BuildWrite(0x3D, {0x08})` → `AA 00 3D 01 08`、`Push` で `BB 0C …12 B` → data、`EE 01` → WRITE_SUCCESS、先頭ゴミ `12 34 BB 01 A0` → A0、`BB 00` / `BB 81` は捨てて次を待つ、1 バイトずつで揃う、`Reset` で途中のフレームを捨てる。
- `BNO055Sequencer`（テストは `Next` / `Complete` を直接呼ぶ。応答は手で作る）: 起動の操作列が順に出る、全部成功で `running()`、RUNNING で 10 ms ごとに READ 0x14,12、100 回に 1 回 0x35、サンプルの換算（ジャイロ z 900 → 1.0 rad/s、heading 2827 → π）、タイムアウトで同じ操作を再発行しエラー +1、成功で 0、連続 11 回で SOFT_RESET（WRITE 0x3F 0x20、`expect_response` false）→ `reinit_count` 1 → `boot_wait_ms` 後に CHECK_ID、`0x07` を 5 回でも再初期化しない、CHIP_ID が違い続けても固まらず running false、SYS_STATUS 1 → SYS_ERR を読んでエラー、`use_external_crystal` false で CHECK_CLOCK / SET_CLOCK が出ない、`Utils::WrapPi(-heading)` が 2π 付近で連続。
- `BNO055UartDriver`（台本どおり返す偽 BNO055 を UART のバイト列で）: 起動列のバイト列、`expected_response_len`（データ読み 14、書き 2、待ちでない 0）、分割到着、タイムアウト後の遅れ応答を捨てる、BOOT_WAIT 中のゴミを無視、ソフトリセットは送って待たない。
- `BNO055I2cDriver`（`BNO055RegisterBus` の偽実装: レジスタ配列を持ち、Read / Write を記録）: 起動列が bus に Write / Read として届く、RUNNING でサンプルが取れる、bus が false を返すとエラーに数え、しきい値で SOFT_RESET（Write 0x3F 0x20 が届く）。
- 既存の `UpdateAccel` / `UpdateGyro` / `UpdateEuler`（°単位の定数）にもテストを足す。

## 5. gamma ファーム

### 5.1 `src/imu_task.hpp / .cpp` — `ImuTask`

- `rabcl_esp32::SerialPort`（`board::IMU_UART_PORT = UART_NUM_3`、`IMU_UART_TX_PIN = GPIO_NUM_12`、`IMU_UART_RX_PIN = GPIO_NUM_15`、115200）を開き、FreeRTOS タスク（優先度 5、スタック 4096）で回す。FreeRTOS の tick は 100 Hz（10 ms）のままなので 1 ms 周期のタスクは作れない。代わりに **応答の到着で起きる**形にする: `BNO055UartDriver::Tick` が要求を出したら、ドライバが知っている応答の長さ（`expected_response_len()`。データ読みなら 14 B、書きなら 2 B、CHIP_ID 読みなら 3 B）を `SerialPort::Read(buf, expected, 2 tick)` でブロック読みする（UART ドライバはその長さが揃った時点で返るので、応答は約 2 ms で手に入る）。揃わなければタイムアウト（2 tick = 20 ms）として読めた分だけ `Feed` に渡し、ドライバが `response_timeout_ms` でエラーを数える。RUNNING では次の要求まで `vTaskDelayUntil` で 10 ms 境界に合わせる（`sample_period_ms`）。起動シーケンスの待ち（7 / 19 / 650 ms）は `vTaskDelay` で tick 単位に丸める（10 ms 刻み。遅くなる方向なので安全）。
- サンプルが揃ったら `ImuState` を作って `ImuStore::Set`:
  - `attitude.yaw = WrapPi(-euler.heading)`（時計回り正 → 左回り正）、`attitude.pitch = IMU_PITCH_SIGN * euler.pitch`、`attitude.roll = IMU_ROLL_SIGN * euler.roll`（符号定数は `robot.hpp`、実機で確定。既定 +1 / +1）。
  - `gyro[3]` はそのまま（入れ替え後 = カメラ座標系。符号は yaw 軸だけ heading と逆にならないよう注意: ジャイロ z は右手系で左回り正なので **そのまま**）。
  - `time_us` = 読み要求を送った時刻（`BNO055UartDriver::last_request_ms()` を µs に。応答の 1.6 ms 前）。
  - `neck_yaw` = その時刻の首 Yaw の関節角（`Controller` から読む口を足す。制御設計書 4.3「その時刻の関節角」に合わせる。レビューで後回しにしていた Minor の解消）。
  - `valid = driver.sequencer().running()`。RUNNING から落ちたら `valid = false` の `ImuState` を 1 回書く。
- 1 秒ごとに INFO ログ（state、サンプル数、エラー数、再初期化回数、calib_stat、yaw / pitch / roll）。
- `Comm::SetImu` にも 100 Hz で渡す（`protocol::Imu`: acc は 0、gyro、euler は REP-103 に直したもの）。bridge の `imu/data` が `imu_link` で出る。RUNNING から落ちたら `Comm::ClearImu()` で送信を止める。

### 5.2 `robot.hpp` の定数

| 定数 | 意味 | 初期値（仮） |
|---|---|---|
| `IMU_AXIS_MAP_CONFIG / IMU_AXIS_MAP_SIGN` | BNO055 の軸入れ替え | 既定 0x24 / 0x00。本番（部品面が後ろ、USB-C が左）と試験（部品面が上、USB-C が右）の 2 組を README に書き、実機で確定 |
| `IMU_PITCH_SIGN / IMU_ROLL_SIGN` | BNO055 のオイラー角 → REP-103 の符号 | +1 / +1（実機で確定） |
| `IMU_OPERATION_MODE` | 融合モード | 0x08（IMU、磁気なし） |
| `IMU_USE_EXTERNAL_CRYSTAL` | CLK_SEL | true（AE-BNO055-BO は XT1 FC-135 32.768 kHz を実装。秋月説明書 2022-04-13 の回路図で確認） |
| `IMU_BOOT_WAIT_MS` | 起動 / リセット後の待ち | 700 |
| `IMU_CLOCK_SETTLE_MS` | CLK_SEL 後の待ち | 20 |
| `IMU_RESPONSE_TIMEOUT_MS` | 応答待ち | 20 |
| `IMU_MAX_CONSECUTIVE_ERRORS` | 再初期化（ソフトリセット）のしきい値 | 10 |

### 5.3 `Controller` の変更

- `ImuState` に `neck_yaw` が入るので、`Tick` の `UpdateFromImu` にはそれを渡す（今は現在の首角）。
- 起動時に IMU を待たない（今のまま）。`error_flags` bit4 は `valid` から。

### 5.4 `main.cpp` と sdkconfig

- `ImuTask` を生成して Start。sdkconfig は変えない。

## 6. 実機での確定手順（モジュールが届いたら。README に書く）

1. 配線とジャンパ（J3 接続、J5 除去）を確認し、挿す前にジャンパ線給電で VOUT 2.8 V を確認（前回の不良の教訓）。
2. 起動ログで `CHECK_ID` → `RUNNING` まで進むこと、サンプルが 100 Hz（1 秒あたり約 100）で来ること。
3. 静止させて CALIB_STAT の GYR が 3 になるまでの時間を見る。
4. 軸: カメラを前に向けた状態から、カメラを下に向ける → `pitch` が正、機体を右に傾ける → `roll` が正、上から見て左に回す → `yaw` が増える。合わなければ `IMU_AXIS_MAP_*` と `IMU_*_SIGN` を直す。本番の取り付け向きで再確認。
5. ジャイロの単位: 一定速度で回して `gyro[2]` と yaw の変化率が一致すること（900 LSB/(rad/s) の裏取り）。
6. 無限回転中に首が視点を保つこと（`Gimbal` の経路）。

## 7. rabbits_description

- `urdf/gamma/neck.xacro` の `camera_link` の子に `imu_link`（固定関節 `imu_joint`）。**向きは camera_link と同じ（回転なし）**: 軸の入れ替えを BNO055 のレジスタで済ませるので、`imu/data` は既にカメラ座標系で出る。TF で二重に回さない。位置だけ仮（カメラの後ろ 0.03 m）。bridge の `imu/data` の `frame_id` は既に `imu_link`。

## 8. 設計者の判断（ユーザー未確認）

- 融合モードは IMU（0x08、磁気なし）。方位は起動時基準。
- 1 周期でジャイロ + オイラー角の 12 B を 1 回で読む。加速度とクォータニオンは読まない。
- CALIB_STAT は 1 秒に 1 回、データ読みの代わりに読む（その周期のサンプルは落ちる）。
- 応答タイムアウト 20 ms、連続エラー 10 回（成功で 0 に戻す）でソフトリセット（RST_SYS、ACK 待ちなし）→ 起動待ち 700 ms → 初期化やり直し。終端状態は作らない。`0xEE 0x07` もこの数に入れるが再試行で済ませる。
- 外部水晶（CLK_SEL）は既定 off。AE-BNO055-BO の回路図で 32 kHz 水晶を確認したら on にする（帰宅後の TODO）。CHECK_STATUS で SYS_STATUS = 5 を確認してから RUNNING。
- `imu_link` は `camera_link` と同じ向き（回転なし）。
- 軸の入れ替えは BNO055 のレジスタで行い、ファームは符号の約束だけ直す。
- `ImuTask` は「要求 → 応答の長さだけブロック読み → 10 ms 境界で次」の FreeRTOS タスク。tick（100 Hz）は変えない。
- 状態機械（`BNO055Sequencer`）をバスから切り離し、UART と I2C の薄い層を別に持つ（ユーザーの提案 2026-10-24: I2C ドライバも作る）。I2C は rabcl の抽象バス `BNO055RegisterBus` までで、ESP32 の実バスは I2C の基板が出たときに rabcl_esp32 へ。名前は既存の `BNO055` に合わせて `BNO055…`。
- `ImuState` に `neck_yaw` を足す。
- IMU の yaw 基準（モード投入時 = 0）と胴体の基準（起動時の向き）は違うので、`Controller::Tick` は IMU 経路に入る瞬間（有効化、再初期化、安定化の入）に `BodyAttitude::AnchorImu` で今の推定に繋ぐ（レビュー指摘。切り替えで砲塔が跳ばない）。
- 応答待ちの `SerialPort::Read` のタイムアウトは `pdMS_TO_TICKS(20) + 1`（tick の丸めで短くならないように）。
- IMU が RUNNING から落ちたら `Comm::ClearImu()`（新規。`imu_present_ = false`）で IMU フレームの送信を止め、古い値が PC に流れ続けないようにする。
- 24 V 断で P4 が USB 給電で生き続ける場合、BNO055 は P4 の 3.3 V 給電なので IMU も生き続ける（この設計に追加は不要）。首 Yaw のフィードバックが止まる間の扱いは制御側（モータ電源断の対応: フィードバック途絶で指令停止、復帰で再イネーブル）で決めた。
- 換算定数（900 LSB/rad）は実機で裏取りするまで仮。

## 9. ユーザーが決めたこと（2026-10-24）

- 速さ優先。移動 + 無限回転しながら狙う。
- 起動時に待たない。復電後は素早く動き出す（Jetson の USB から P4 へ給電して 24V 断でも落とさない配線も行う）。
- IMU の向きは本番 = 部品面が後ろ・USB-C が左、試験中 = 部品面が上・USB-C が右（帰宅後に実物で確認）。軸の入れ替えは設定で変えられるように。
- URDF に `imu_link` を生やす。
- 実装は C++、rabcl 部分はユニットテスト必須。
- I2C ドライバも作る（状態機械を共通にし、バスごとの層を分ける）。

## 10. 実装者に任せること

- `BNO055Sequencer` の内部状態の持ち方（待ち時間の数え方、再発行の実装）。
- ログの文言、`ImuTask` のスタックサイズの最終値。
- テストの偽 BNO055 の作り（UART のバイト列版と、`BNO055RegisterBus` のレジスタ配列版）。
