# ロボットプロトコル（PC ⇔ ファーム）設計

- 日付: 2026-10-22
- 対象リポジトリ: rabcl（本体）、rabcl_ros2、rabbits_bridge、rabbits_msgs、gamma_turret_fw、beta_main
- 状態: 設計中（ビルダーチェック前）

## 1. 目的と背景

rabcl は「機体に依らない共通ライブラリ」のはずだが、現状は beta の構成に結びついた部分を持っている。

| 場所 | 機体依存の中身 |
|---|---|
| `rabcl/utils/type.hpp` の `CAN_ID` enum | beta の CAN ID 表（yaw 0x141、pitch 0x42、車輪 0x43〜0x46） |
| `rabcl::Can::UpdateData` | その ID 表で分岐してフィードバックを `Info` に書く |
| `rabcl::Info` | yaw / pitch / 車輪 4 つ、load / fire / speed / chassis モードという beta の軸構成 |
| `rabcl::Uart` | `Info` を固定レイアウト（指令 28 B、状態 112 B）で詰める |

gamma は首 Roll と脚 4 本（DM4310）が増え、今後も項目が増える（ユーザー要望: 「今のモータと IMU の状態は上げたい。ほかにも欲しい情報は増えるかも」）。機体ごとにレイアウトを書き換えるのではなく、**中身が自己記述的な共通プロトコル**を rabcl に置き、機体ごとの違いを「データ（番号表と名前表）」だけにする。

beta 側もこのプロトコルへ移行する前提で設計する（ユーザー決定）。

## 2. ゴール / 非ゴール / やってはいけないこと

**ゴール**

- rabcl に機体の値（CAN ID、軸構成、レイアウト）を一切持たせない。
- PC ⇔ ファームのプロトコルを全機体で共通にし、rabcl に実装してユニットテストで固める。
- 状態（全モータ、IMU）と指令（車体速度、関節目標、モード）を送れ、種別を足すだけで項目を増やせる。
- ROS 2 側（bridge）を機体非依存にし、機体の違いはパラメータ YAML だけにする。
- beta_main と gamma_turret_fw の両方がこのプロトコルで動く。

**非ゴール**

- CAN 側（モータとの通信）の抽象化。`Prepare* / Parse*` は今のまま rabcl に残し、ID の振り分けは各ファームが行う。
- Wi-Fi / Ethernet など UART 以外のトランスポート（枠組みはバイト列なので後から載せられる）。
- ゲーム側（rabbits_interface の gamepad / game_client）の作り替え。入口のメッセージ変換だけを扱う。

**やってはいけないこと**

- 機体名で分岐するコードを rabcl / rabcl_ros2 / rabbits_bridge に書く。
- 既存の beta の動作を、移行完了前に壊す（移行は手順を分け、旧コードは最後に消す）。

## 3. 制約

- rabcl は C++17、`<cstdint>` と標準ライブラリのみ。STM32（beta、HAL + DMA）と ESP32-P4（gamma、ESP-IDF）と Linux（Jetson）で同じコードがビルドされる。動的確保・例外・RTTI は使わない。
- UART: beta は 230400 bps（STM32 USART2）、gamma は 921600 bps（P4 UART2 ⇔ Jetson）。ボーレートは機体の設定値で、プロトコルは依存しない。
- 1 メッセージのペイロードは 255 B 以下（長さは 1 B）。
- rabcl 系（rabcl / rabcl_ros2 / rabcl_esp32）はユニットテスト必須、CI で実行（handbook common-lib.md）。
- 命名: 定数は UPPER_SNAKE_CASE、include は外部 `<>`、自リポジトリ `""`。

## 4. 設計

### 4.1 用語

- **関節（joint）**: モータ 1 つ。機体ごとに 0 始まりの番号（joint id）を振る。番号と名前の対応は機体側（ファームの `robot.hpp` と ROS 2 の YAML）が持つ。
- **上り**: ファーム → PC（状態）。**下り**: PC → ファーム（指令）。

### 4.2 フレーム

```
| 0xA5 | 0x5A | type (1 B) | len (1 B) | payload (len B) | crc8 (1 B) |
```

- `type`: メッセージ種別（4.3）。上位 1 bit で方向を表す（0x00〜0x7F 上り、0x80〜0xFF 下り）。
- `len`: payload のバイト数（0〜255）。
- `crc8`: `type`、`len`、`payload` に対する CRC-8（多項式 0x07、初期値 0x00、非反転）。今の XOR チェックサムは置き換える。
- 数値はリトルエンディアン（ARM / RISC-V / x86 すべて LE なので変換が要らない）。float は IEEE 754 binary32。
- 受信側は「ヘッダ 2 B を探す → type / len を読む → len + 1 B を待つ → CRC 照合」のバイト列パーサで、どこから入ってきても再同期できる。ヘッダ一致後に CRC が合わなければ、ヘッダの次のバイトから探し直す。

### 4.3 メッセージ種別

**上り（ファーム → PC）**

| type | 名前 | payload | 送信契機 |
|---|---|---|---|
| 0x01 | `INFO` | `protocol_version u8`, `joint_count u8`, `firmware_id u8[8]`（ASCII、末尾 0 埋め） | 起動時と 1 秒ごと |
| 0x02 | `JOINT_STATE` | `count u8` + count × { `id u8`, `position f32 [rad]`, `velocity f32 [rad/s]`, `effort f32 [Nm または A]`, `temperature f32 [℃]`, `error u8` } | 制御周期ごと（gamma 100 Hz、beta 100 Hz） |
| 0x03 | `IMU` | `acc f32[3] [m/s²]`, `gyro f32[3] [rad/s]`, `euler f32[3] [rad]（heading, roll, pitch）` | IMU 更新ごと |
| 0x04 | `SYSTEM` | `bus_voltage f32 [V]`, `error_flags u32`, `uptime_ms u32` | 1 秒ごと（最初は gamma のみ） |

`JOINT_STATE` の 1 関節は 18 B。1 メッセージに最大 14 関節（253 B）。それを超える機体は複数メッセージに分けて送る（`count` が示す）。`effort` の意味（トルクか電流か）は関節ごとに決まり、ROS 2 側の YAML に `effort_unit` として書く。

**下り（PC → ファーム）**

| type | 名前 | payload |
|---|---|---|
| 0x81 | `CHASSIS_VEL` | `vx f32 [m/s]`, `vy f32 [m/s]`, `wz f32 [rad/s]` |
| 0x82 | `JOINT_CMD` | `count u8` + count × { `id u8`, `position f32`, `velocity f32`, `effort f32`, `kp f32`, `kd f32` } |
| 0x83 | `MODE` | `count u8` + count × { `id u8`, `value u8` } |
| 0x84 | `BODY_POSE` | `height f32 [0〜1]`, `roll f32 [rad]`, `pitch f32 [rad]`（車体の目標姿勢。2026-10-24 追加、gamma の脚制御用。ロール / ピッチは REP-103 の符号） |

- `JOINT_CMD` の 1 関節は 21 B。位置制御の関節（yaw / pitch / roll）は `position` だけを使い、MIT 制御の関節（脚）は 5 つすべてを使う。どの欄を使うかはファームの関節定義が決め、使わない欄は 0 を入れる。
- `MODE` の `id` は機体ごとのモード番号（例: 0 = load、1 = fire、2 = speed、3 = chassis）。番号と名前の対応も YAML と `robot.hpp` に置く。

### 4.4 rabcl の API（`rabcl/protocol/`）

```cpp
namespace rabcl::protocol
{
constexpr uint8_t VERSION = 1;
constexpr uint8_t HEADER_0 = 0xA5;
constexpr uint8_t HEADER_1 = 0x5A;
constexpr uint8_t MAX_PAYLOAD = 255;
constexpr size_t MAX_FRAME = 2 + 1 + 1 + MAX_PAYLOAD + 1;  // 260

enum class Type : uint8_t { INFO = 0x01, JOINT_STATE = 0x02, IMU = 0x03, SYSTEM = 0x04,
                            CHASSIS_VEL = 0x81, JOINT_CMD = 0x82, MODE = 0x83, BODY_POSE = 0x84 };

struct JointState { uint8_t id; float position; float velocity; float effort; float temperature; uint8_t error; };
struct JointCmd   { uint8_t id; float position; float velocity; float effort; float kp; float kd; };
struct ModeCmd    { uint8_t id; uint8_t value; };
struct Info       { uint8_t protocol_version; uint8_t joint_count; char firmware_id[8]; };
struct Imu        { float acc[3]; float gyro[3]; float euler[3]; };
struct System     { float bus_voltage; uint32_t error_flags; uint32_t uptime_ms; };
struct ChassisVel { float vx; float vy; float wz; };

uint8_t Crc8(const uint8_t * data, size_t len);

// エンコード: buf にフレームを書き、フレーム長を返す（容量不足なら 0）
size_t EncodeInfo(const Info &, uint8_t * buf, size_t cap);
size_t EncodeJointState(const JointState * joints, uint8_t count, uint8_t * buf, size_t cap);
size_t EncodeImu(const Imu &, uint8_t * buf, size_t cap);
size_t EncodeSystem(const System &, uint8_t * buf, size_t cap);
size_t EncodeChassisVel(const ChassisVel &, uint8_t * buf, size_t cap);
size_t EncodeJointCmd(const JointCmd * cmds, uint8_t count, uint8_t * buf, size_t cap);
size_t EncodeMode(const ModeCmd * modes, uint8_t count, uint8_t * buf, size_t cap);

// デコード: payload から構造体へ。長さ不正なら false
bool DecodeInfo(const uint8_t * payload, uint8_t len, Info &);
bool DecodeJointState(const uint8_t * payload, uint8_t len, JointState * joints, uint8_t max, uint8_t & count);
bool DecodeImu(const uint8_t * payload, uint8_t len, Imu &);
bool DecodeSystem(const uint8_t * payload, uint8_t len, System &);
bool DecodeChassisVel(const uint8_t * payload, uint8_t len, ChassisVel &);
bool DecodeJointCmd(const uint8_t * payload, uint8_t len, JointCmd * cmds, uint8_t max, uint8_t & count);
bool DecodeMode(const uint8_t * payload, uint8_t len, ModeCmd * modes, uint8_t max, uint8_t & count);

// 受信パーサ: バイト列を何回に分けて渡してもよい。フレームが揃うたびに on_frame を呼ぶ
class Parser
{
public:
  using FrameHandler = void (*)(Type type, const uint8_t * payload, uint8_t len, void * ctx);
  void Feed(const uint8_t * data, size_t len, FrameHandler on_frame, void * ctx);
  uint32_t crc_errors() const;   // CRC 不一致の回数
  uint32_t resyncs() const;      // ヘッダ探索に戻った回数
  void Reset();
private:
  uint8_t frame_[MAX_FRAME];
  size_t pos_ = 0;
  // 状態: SCAN_H0 / SCAN_H1 / HEADER / PAYLOAD
};
}  // namespace rabcl::protocol
```

- 関数ポインタ + `void * ctx` にしているのは、STM32 の割り込み文脈と ESP-IDF のタスクの両方から `std::function` 無しで使うため。
- `Parser` は内部バッファ 260 B を 1 つ持つだけで、動的確保はしない。

### 4.5 機体ごとに持つデータ

**ファーム（`src/robot.hpp`）**

```cpp
namespace robot
{
constexpr char FIRMWARE_ID[8] = "gamma";
enum Joint : uint8_t { YAW = 0, PITCH, ROLL, WHEEL_FR, WHEEL_FL, WHEEL_BR, WHEEL_BL,
                       LEG_FR, LEG_FL, LEG_BR, LEG_BL, JOINT_COUNT };
enum Mode : uint8_t { LOAD = 0, FIRE, SPEED, CHASSIS, MODE_COUNT };
// 関節ごとの CAN ID / MST_ID / モータ種別 / PMAX 等 / 原点オフセット / ゲイン はここ
}
```

**ROS 2（rabbits_bridge の `config/<機体>.yaml`）**

```yaml
robot_bridge:
  ros__parameters:
    device: /dev/ttyTHS1
    baud: 921600
    joints:            # 番号順。JointState の name 配列になる
      - yaw
      - pitch
      - roll
      - wheel_fr
      - wheel_fl
      - wheel_br
      - wheel_bl
      - leg_fr
      - leg_fl
      - leg_br
      - leg_bl
    effort_unit: [Nm, Nm, Nm, Nm, Nm, Nm, Nm, Nm, Nm, Nm, Nm]  # LK モータは A
    modes: [load, fire, speed, chassis]
```

beta 用の YAML は joints = [yaw, pitch, wheel_fr, wheel_fl, wheel_br, wheel_bl]。

### 4.6 ROS 2 側

**rabbits_bridge**: `robot_bridge` ノード（`stm32_bridge_node` の置き換え）。

| 向き | トピック | 型 | 変換 |
|---|---|---|---|
| 出力 | `joint_states` | `sensor_msgs/JointState` | `JOINT_STATE` → name は YAML、position / velocity / effort |
| 出力 | `imu/data` | `sensor_msgs/Imu` | `IMU` → orientation はオイラー角からクォータニオン、angular_velocity、linear_acceleration |
| 出力 | （後回し）`system_state` | `rabbits_msgs/SystemState`（bus_voltage、error_flags、uptime。gamma v1.0 基板に電圧測定が無いため、必要になってから作る） | `SYSTEM` |
| 入力 | `reference_command` | `rabbits_msgs/ReferenceCommand`（新規。下り全体を 1 つにまとめた汎用型） | 下記 |

`ReferenceCommand`（ユーザー決定: `AttractsCommand` の代わりに rabcl プロジェクト共通の汎用 PC → ファーム型を作る）:

```
std_msgs/Header header
geometry_msgs/Vector3 chassis_vel   # x, y [m/s], z = 回転 [rad/s] → CHASSIS_VEL
string[]  joint_name                # URDF の関節名 → YAML で id に変換 → JOINT_CMD
float32[] joint_position
float32[] joint_velocity
float32[] joint_effort
float32[] joint_kp
float32[] joint_kd
string[]  mode_name                 # YAML で id に変換 → MODE
uint8[]   mode_value
```

- 配列は同じ長さ。空なら対応する下りメッセージを送らない（車体速度は常に送る）。
- gamepad / game_client は `AttractsCommand` の代わりにこれを出す。「右スティック → `yaw_joint` の目標」の対応はゲーム側ノードが持つ。

- YAML の関節名は URDF（rabbits_description）の関節名と同じにする（例: `yaw_joint`、`right_front_wheel_joint`）。こうすると `joint_states` を robot_state_publisher がそのまま使える。今 gamepad / game_client が指令から作っている擬似の `joint_states` は、実フィードバックが出るようになった時点でやめる（ユーザー決定）。
- `INFO` の `protocol_version` が rabcl の `VERSION` と違えばエラーログ、`joint_count` が YAML の数と違えば警告。
- 受信スレッド（タイマ）はシリアルから読めた分を `Parser::Feed` に流すだけ。
- 既存の `AttractsCommand` との橋渡し: rabbits_interface（gamepad / game_client）が `cmd_vel` / `joint_cmd` / `mode_cmd` を直接出す形に変える（**ユーザー確認事項 Q1**）。

**rabcl_ros2**: `rabcl::protocol` の構造体 ⇔ ROS メッセージの変換関数を持つ（`JointStateToMsg(joints, names, msg)` など）。bridge は I/O と YAML だけを持つ。`Info` ベースの `Utils` は移行完了後に削除。

### 4.7 ファーム側

**gamma_turret_fw**

- rabcl_esp32 に `SerialPort`（esp_driver_uart の包み。`Read(buf, len, timeout)` / `Write(buf, len)`、受信は driver のリングバッファ、fake によるユニットテストあり）を追加。
- 通信タスク: 受信 → `Parser::Feed` → 指令を共有状態へ。送信は制御タスクの周期で `JOINT_STATE`、IMU タスクで `IMU`、1 Hz で `INFO` / `SYSTEM`。
- 本設計の範囲（手順 4）は「通信」まで: CAN から受けた関節のフィードバックを `JOINT_STATE` で上げ、受けた指令を共有状態に置く。つないである関節だけを送る（最初は脚 1 本）。車輪の運動学、首の位置制御、脚の MIT 制御、BNO055（UART）ドライバは別の設計・計画で扱う。`IMU` メッセージは BNO055 ドライバができた時点で送り始める。
- 下りの指令が途絶えたときの安全停止は本設計の範囲外（ユーザー決定: 後で考える）。今の beta と同じく、最後の指令を保持する。入れる場合の案は 7 章の Q2 に残す。

**beta_main（移行）**

- `Info` / `Uart` を `rabcl::protocol` に置き換える。受信は USART2 の DMA（循環バッファ + IDLE 割り込み）で溜まった分を `Parser::Feed` に渡す形にする（今の「固定長 DMA + 再同期」の方式から変更）。
- beta の CAN ID 表は beta_main 内の `robot.hpp` に移す（rabcl の `CAN_ID` を使わない）。
- 送信内容は今と同じ（IMU は BNO055 未検証なので 0 のまま）。
- beta_main の `lib/rabcl` submodule は旧組織名（robot-trainee）の URL を指しているので、移行時に `rabbits-robotics/rabcl` に付け替える（TODO.md の既存項目）。

### 4.8 rabcl から消すもの（最終タスク）

beta と gamma と ROS 2 の移行が終わった時点で削除する: `utils/type.hpp` の `CAN_ID` / `MODE_ID` / `Info`、`Can::UpdateData`、`interface/uart.hpp` と `uart.cpp`、それらのテスト。`MotorInfo` / `ImuInfo` は `Prepare* / Parse*` が使うので残す。

### 4.9 テスト

- rabcl: `test/test_protocol.cpp`。CRC の既知値、各メッセージの encode → decode 往復、`len` 不正で decode が false、`count` 上限、`Parser` に 1 バイトずつ / まとめて / 途中で切って流しても同じ結果、ゴミ → 正しいフレームで再同期、CRC 不一致でカウンタが増えて次のフレームを拾う、ヘッダ値がペイロード中に現れても誤同期しない。
- rabcl_ros2: 変換の往復と、name → id の対応。
- rabbits_bridge: `test_fd` 方式で、バイト列を流して `joint_states` が出ること、`cmd_vel` を流して `CHASSIS_VEL` フレームが書かれること。
- rabcl_esp32: `SerialPort` の fake テスト。
- 実機: gamma は Jetson（または PC の USB-UART）と直結して `INFO` / `JOINT_STATE` を見る。beta は既存の動作（ゲームパッドで走る）が新プロトコルで再現すること。

### 4.10 通信量の見積もり

8N1 なので 1 B = 10 bit。上りと下りは全二重で別勘定。

| 機体 | ボーレート | 上り 1 周期（`JOINT_STATE` + `IMU`） | 100 Hz の使用率 | 下り 1 周期（全関節 + 車体 + モード） | 100 Hz の使用率 |
|---|---|---|---|---|---|
| gamma（11 関節） | 921600 | 208 + 41 = 約 250 B | 約 27% | 約 270 B | 約 29% |
| beta（6 関節） | 230400 | 118 + 41 = 約 160 B | 約 69% | 約 160 B | 約 70% |

- gamma は 100 Hz で余裕があり、200 Hz でも 55% 程度。beta は 100 Hz が上限。ROS 2 側のタイマのジッタを考えると実用は 50〜100 Hz。
- 1 フレームの送出時間（= 遅延の目安）は gamma で約 2.7 ms、beta で約 7 ms。
- 足りなくなったときの手: gamma はボーレートを 2 Mbps 以上に上げる（P4 も Jetson も対応）、温度のような遅い項目を別メッセージにして低い周期で送る（関節あたり 4 B 減）。

## 5. 実施順

1. rabcl: `protocol/` と `test_protocol.cpp`（既存コードは触らない）。
2. rabbits_msgs: `ReferenceCommand` / `SystemState`。rabcl_ros2: 変換関数とテスト。
3. rabbits_bridge: `robot_bridge` ノード + `config/beta.yaml` / `config/gamma.yaml`（URDF の関節名）+ テスト。旧 `stm32_bridge_node` は残す。
4. rabbits_interface: gamepad / game_client が `ReferenceCommand` を出す形に変更し、擬似 `joint_states` をやめる（beta の実機確認より前に行う）。
5. gamma_turret_fw: `robot.hpp`、rabcl_esp32 `SerialPort`、通信タスク。実機で INFO / JOINT_STATE 確認。
6. （後回し。2026-10-22 ユーザー決定）beta_main: 新プロトコルへ移行、`robot.hpp` に ID 表、submodule URL の付け替え。実機でゲームパッド走行を確認。
7. （後回し。6 の後）旧コード削除: rabcl（4.8）、rabcl_ros2 の `Utils`、rabbits_bridge の `stm32_bridge_node`、rabbits_msgs の `AttractsCommand`。

6 と 7 を行うまでは、rabcl の旧 API（`Info` / `Uart` / `CAN_ID` / `UpdateData`）と新しい `protocol/` が共存する。

各段階で CI を通し、コミットはユーザー承認のうえ 1 行メッセージ。

## 6. 設計者の判断（ユーザー未確認の決定）

- フレームの CRC を XOR から CRC-8（0x07）に変更。
- 数値をリトルエンディアンに変更（現行は float をビッグエンディアン）。
- `JOINT_CMD` を位置制御と MIT 制御で共通の 5 欄構造にし、使わない欄は 0。
- `effort` の単位は関節ごと（トルクか電流か）で、YAML に書く。
- `INFO` に `firmware_id`（8 文字）と `joint_count` を入れ、bridge 側で不一致を検出。
- `ReferenceCommand` の配列は同じ長さ、空なら送らない、車体速度は常に送る。

## 7. ユーザー決定（2026-10-22）

- **Q1**: `AttractsCommand` を廃止し、rabcl プロジェクト共通の汎用 PC → ファーム型 `rabbits_msgs/ReferenceCommand` を作る。gamepad / game_client はこれを出す。
- **Q2**: 指令途絶時の安全停止は後で考える（本設計の範囲外）。入れる場合の案: 下りのどの種別でも 1 フレームで監視タイマをリセット、500 ms 途絶で車体速度 0・射出と装填を停止・関節目標は保持、beta にも入れる。
- **Q3**: 関節名は URDF 基準。gamepad / game_client の擬似 `joint_states` はやめる。
