# ロボットプロトコル（PC ⇔ ファーム）実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** rabcl に機体非依存の PC ⇔ ファームプロトコル（`rabcl::protocol`）を実装し、ROS 2 側（rabbits_msgs / rabcl_ros2 / rabbits_bridge / rabbits_interface）と gamma のファームをそれに載せる。beta の移行と、rabcl から beta 固有の `CAN_ID` / `UpdateData` / `Info` / 固定レイアウトの `Uart` を取り除く作業は後回し（Task 6 / 7 は beta を移行するときに実施。2026-10-22 ユーザー決定）。`SYSTEM` メッセージは rabcl に種別と Encode / Decode だけ用意し、ROS メッセージと bridge の配信は必要になるまで作らない（gamma v1.0 基板に電源電圧の測定が無いため）。

**Architecture:** フレームは `A5 5A | type | len | payload | crc8`。上りは `INFO` / `JOINT_STATE` / `IMU` / `SYSTEM`、下りは `CHASSIS_VEL` / `JOINT_CMD` / `MODE`。機体ごとの違いは「関節番号と名前の対応」だけで、ファームは `robot.hpp`、ROS 2 は bridge の YAML が持つ。ROS 2 の入口は `rabbits_msgs/ReferenceCommand` 1 つ、出口は `sensor_msgs/JointState`（URDF の関節名）/ `sensor_msgs/Imu` / `rabbits_msgs/SystemState`。

**Tech Stack:** C++17、gtest、ROS 2 Jazzy（ament_cmake、rosidl）、ESP-IDF 5.5.5（esp_driver_uart、PlatformIO + pioarduino）、STM32Cube HAL（F303K8、DMA）、Docker（rabbits/ros2-jazzy-dev）でのホストビルド。

**Spec:** `docs/superpowers/specs/2026-10-22-robot-protocol-design.md`（rabcl リポジトリ）

## Global Constraints

- rabcl は `<cstdint>` と標準ライブラリのみ。動的確保・例外・RTTI なし。STM32 / ESP32-P4 / Linux で同じコードがビルドされる。
- フレーム: ヘッダ `0xA5 0x5A`、`len` は payload 長（0〜255）、CRC-8 は多項式 0x07・初期値 0x00・非反転、対象は `type` + `len` + `payload`。数値はリトルエンディアン、float は binary32。
- メッセージ種別: `INFO 0x01`、`JOINT_STATE 0x02`、`IMU 0x03`、`SYSTEM 0x04`、`CHASSIS_VEL 0x81`、`JOINT_CMD 0x82`、`MODE 0x83`。`PROTOCOL_VERSION = 1`。
- payload レイアウト: `INFO` = version u8, joint_count u8, firmware_id char[8]（10 B）。`JOINT_STATE` = count u8 + count × 18 B（id u8, position f32, velocity f32, effort f32, temperature f32, error u8）、count ≤ 14。`IMU` = acc[3], gyro[3], euler[3] f32（36 B）。`SYSTEM` = bus_voltage f32, error_flags u32, uptime_ms u32（12 B）。`CHASSIS_VEL` = vx, vy, wz f32（12 B）。`JOINT_CMD` = count u8 + count × 21 B（id u8, position, velocity, effort, kp, kd f32）、count ≤ 12。`MODE` = count u8 + count × 2 B（id u8, value u8）。
- 機体の値（CAN ID、PMAX 等、関節名）を rabcl / rabcl_ros2 / rabbits_bridge のコードに書かない。
- rabcl 系（rabcl / rabcl_ros2 / rabcl_esp32）はユニットテスト必須、CI で実行。README の 2 行目に CI バッジ。
- C++: 定数は UPPER_SNAKE_CASE、include は外部 `<>`・自リポジトリ `""`・グループを空行で分け、.cpp は対応ヘッダを先頭に置く。ament の uncrustify に通す（行長 100）。
- コミットは各タスクの最後にユーザー承認を得てから行う。メッセージは 1 行の日本語、Co-Authored-By なし、経緯は書かない。push も承認後。
- ビルド: ROS 2 パッケージは Docker（`rabbits/ros2-jazzy-dev`）で `colcon build / test`。ESP32 は PowerShell から `pio run`（同時に 2 つ走らせない）。STM32 は beta_main で `pio run`。
- 安全停止（指令途絶時）は実装しない（設計書 7 章 Q2）。

## Review Focus

- payload の中にヘッダ値 `A5 5A` が現れるフレーム（float のビット列として普通に起きる）で `Parser` が誤同期せず、次の正しいフレームも落とさない → Task 1 のテスト `ParserIgnoresHeaderBytesInsidePayload`。
- `JOINT_STATE` に 15 関節以上を渡したとき `Encode` が 0 を返し、ファームが分割して送る → Task 1 `EncodeJointStateRejectsMoreThan14`、Task 5 の 2 分割送信。
- bridge が YAML の関節数以上の `id` を受けたとき落ちずに読み飛ばし、1 回だけ警告する → Task 3 `UnknownJointIdIsSkipped`。
- `ReferenceCommand` の配列長が揃っていないメッセージを bridge が捨てて警告する → Task 2 `ReferenceWithMismatchedArraysIsRejected`。
- バイト列をどの位置で分割して `Feed` しても結果が同じ → Task 1 `ParserHandlesEverySplitPoint`。

## File Structure

| リポジトリ | 作る / 変える | 役割 |
|---|---|---|
| rabcl | `include/rabcl/protocol/protocol.hpp`, `src/protocol/protocol.cpp` | 型、定数、CRC、Encode / Decode |
| rabcl | `include/rabcl/protocol/parser.hpp`, `src/protocol/parser.cpp` | 受信パーサ |
| rabcl | `test/test_protocol.cpp`, `CMakeLists.txt`, `README.md` | テスト、ビルド登録、説明 |
| rabbits_msgs | `msg/ReferenceCommand.msg`, `CMakeLists.txt` | ROS メッセージ |
| rabcl_ros2 | `include/rabcl_ros2/convert.hpp`, `src/convert.cpp`, `test/test_convert.cpp` | protocol 構造体 ⇔ ROS メッセージ |
| rabbits_bridge | `include/rabbits_bridge/robot_bridge_node.hpp`, `src/robot_bridge_node.cpp`, `config/beta.yaml`, `config/gamma.yaml`, `launch/robot_bridge.launch.py`, `test/test_robot_bridge.cpp` | 汎用ブリッジ |
| rabbits_interface | `src/gamepad_node.cpp`, `src/game_client_node.cpp`, 各ヘッダとテスト | `ReferenceCommand` を出す |
| rabcl_esp32 | `include/rabcl_esp32/serial_port.hpp`, `src/serial_port.cpp`, `test/fakes/driver/uart.h`, `test/test_serial_port.cpp` | UART の包み |
| gamma_turret_fw | `src/robot.hpp`, `src/comm.hpp`, `src/comm.cpp`, `src/board.hpp`, `src/main.cpp` | 機体定義、通信タスク |
| beta_main（後回し） | `Inc/robot.hpp`, `Inc/robot_state.hpp`, `Src/main.cpp`, `Src/usart.c`, `.gitmodules` | 新プロトコルへ移行 |

---

### Task 1: rabcl `protocol/`（型・CRC・Encode / Decode・Parser）

**Files:**
- Create: `include/rabcl/protocol/protocol.hpp`, `src/protocol/protocol.cpp`, `include/rabcl/protocol/parser.hpp`, `src/protocol/parser.cpp`, `test/test_protocol.cpp`
- Modify: `CMakeLists.txt`（IDF の SRCS と ament の `add_library`、`ament_add_gtest(test_protocol ...)`）、`README.md`（`protocol/` の行を構成表に追加）

**Interfaces:**
- Produces（設計書 4.4 のとおり、名前空間 `rabcl::protocol`）:
  - 定数 `VERSION`, `HEADER_0`, `HEADER_1`, `MAX_PAYLOAD = 255`, `FRAME_OVERHEAD = 5`, `MAX_FRAME = 260`, `MAX_JOINT_STATES = 14`, `MAX_JOINT_CMDS = 12`
  - `enum class Type : uint8_t`
  - 構造体 `Info`, `JointState`, `Imu`, `System`, `ChassisVel`, `JointCmd`, `ModeCmd`
  - `uint8_t Crc8(const uint8_t * data, size_t len)`
  - `size_t EncodeInfo(const Info &, uint8_t * buf, size_t cap)` ほか 7 つの `Encode*`（戻り値 = フレーム長、容量不足または count 超過なら 0）
  - `bool DecodeInfo(const uint8_t * payload, uint8_t len, Info &)` ほか 7 つの `Decode*`（`len` がレイアウトと合わなければ false。配列型は `max` を超えると false）
  - `class Parser { void Feed(const uint8_t *, size_t, FrameHandler, void * ctx); uint32_t crc_errors() const; uint32_t resyncs() const; void Reset(); }`、`using FrameHandler = void (*)(Type, const uint8_t * payload, uint8_t len, void * ctx)`
- Consumes: なし

- [ ] **Step 1: テストを書く（RED）** `test/test_protocol.cpp`

```cpp
TEST(Protocol, Crc8KnownValue)  // "123456789" → 0xF4（CRC-8 poly 0x07）
TEST(Protocol, EncodeChassisVelFrameBytes)
  // {1.0f, -2.0f, 0.5f} → A5 5A 81 0C [1.0f LE][-2.0f LE][0.5f LE] crc、長さ 17、crc == Crc8(buf+2, 14)
TEST(Protocol, InfoRoundTrip)        // version 1, joint_count 11, "gamma" → 同じ値、firmware_id 残りは 0
TEST(Protocol, JointStateRoundTrip)  // 3 関節、id {0, 7, 10}、各欄に異なる値、error 0x0A
TEST(Protocol, JointStateDecodeRejectsWrongLen)   // len = 1 + 18*2 + 1 → false
TEST(Protocol, EncodeJointStateRejectsMoreThan14) // 15 関節 → 0
TEST(Protocol, EncodeReturnsZeroWhenBufferTooSmall)
TEST(Protocol, ImuRoundTrip)
TEST(Protocol, SystemRoundTrip)
TEST(Protocol, JointCmdRoundTrip)    // 12 関節（上限）
TEST(Protocol, EncodeJointCmdRejectsMoreThan12)
TEST(Protocol, ModeRoundTrip)        // {{0,1},{3,2}}
TEST(Protocol, DecodeRejectsCountOverMax)  // JointState count 3 を max 2 で → false

TEST(Parser, SingleFrameInOneFeed)        // ハンドラが 1 回、type / len / payload が一致
TEST(Parser, TwoFramesInOneFeed)
TEST(Parser, ParserHandlesEverySplitPoint)
  // 2 フレーム連結したバイト列を、分割位置 i = 1..N-1 すべてで 2 回に分けて Feed → 毎回 2 フレーム
TEST(Parser, GarbageThenFrameResyncs)     // 00 11 A5 22 のあとに正しいフレーム → 1 フレーム、resyncs() >= 1
TEST(Parser, CrcErrorIsCountedAndNextFrameIsReceived) // crc を壊したフレーム + 正しいフレーム → 1 フレーム、crc_errors() == 1
TEST(Parser, ParserIgnoresHeaderBytesInsidePayload)
  // payload に A5 5A を含むフレーム（ChassisVel で vx のビット列を細工）→ 正しく 1 フレーム
TEST(Parser, TruncatedFrameThenNewFrame)  // 途中で切れたフレームの直後に新しいフレーム → 新しい方だけ受かる
TEST(Parser, ZeroLengthPayloadFrame)      // len 0 のフレームも受かる
```

- [ ] **Step 2: RED を確認**

Run（rabcl ルート、Docker）:
```bash
docker run --rm -v "$PWD:/mnt/rabcl:ro" rabbits/ros2-jazzy-dev bash -lc 'mkdir -p ~/ws/src && cp -r /mnt/rabcl ~/ws/src/rabcl && cd ~/ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select rabcl 2>&1 | grep -E "error|Finished|Failed" | head'
```
Expected: `rabcl/protocol/protocol.hpp: No such file` のコンパイルエラー

- [ ] **Step 3: `protocol.hpp` / `protocol.cpp` を実装**

`Encode*` は `buf[0..1]` にヘッダ、`buf[2]` に type、`buf[3]` に len、payload を書いてから `buf[4+len] = Crc8(buf+2, len+2)`。float は `memcpy` で 4 B（LE 前提。ビッグエンディアン環境は対象外）。`Decode*` は `len` の一致を先に検査する。

- [ ] **Step 4: `parser.hpp` / `parser.cpp` を実装**

状態 `SCAN_H0 → SCAN_H1 → TYPE → LEN → PAYLOAD_AND_CRC`。フレームが揃ったら CRC を照合し、一致なら `on_frame`、不一致なら `crc_errors_++` して「`frame_[1]` から `pos_` までのバイトを先頭に詰めて `SCAN_H0` から再走査」する（ヘッダ候補を取りこぼさないため）。`SCAN_H1` で `A5` 以外が来たら `resyncs_++`。

- [ ] **Step 5: CMake とテスト登録**

IDF ブロックの SRCS に `src/protocol/protocol.cpp src/protocol/parser.cpp`、ament の `add_library` にも同じ 2 つ、`ament_add_gtest(test_protocol test/test_protocol.cpp)` + `target_link_libraries(test_protocol rabcl)`。

- [ ] **Step 6: GREEN を確認（全テストと uncrustify）**

Run: Step 2 のコマンドに `colcon test --packages-select rabcl && colcon test-result --verbose | grep -E "Summary|FAIL"` を続ける
Expected: `Summary: N tests, 0 errors, 0 failures`（N は既存 156 + 追加分）

- [ ] **Step 7: README を更新**（構成表に `protocol/` の行: 「PC ⇔ ファームの共通プロトコル。フレーム化、CRC、メッセージの Encode / Decode、受信パーサ」）

- [ ] **Step 8: コミット（承認後）** `PC とファームの共通プロトコルを追加`

---

### Task 2: rabbits_msgs の新メッセージと rabcl_ros2 の変換

**Files:**
- Create: `rabbits_msgs/msg/ReferenceCommand.msg`
- Modify: `rabbits_msgs/CMakeLists.txt`（`msg_files` に追加）、`rabbits_msgs/README.md`
- Create: `rabcl_ros2/include/rabcl_ros2/convert.hpp`, `rabcl_ros2/src/convert.cpp`, `rabcl_ros2/test/test_convert.cpp`
- Modify: `rabcl_ros2/CMakeLists.txt`（`add_library` に `src/convert.cpp`、`ament_add_gtest(test_convert ...)`）、`rabcl_ros2/README.md`

**Interfaces:**
- Consumes: Task 1 の `rabcl::protocol` 構造体
- Produces:
  - `ReferenceCommand.msg`:
    ```
    std_msgs/Header header
    geometry_msgs/Vector3 chassis_vel
    string[] joint_name
    float32[] joint_position
    float32[] joint_velocity
    float32[] joint_effort
    float32[] joint_kp
    float32[] joint_kd
    string[] mode_name
    uint8[] mode_value
    ```
  - `rabcl_ros2::Convert`（static 関数）:
    - `bool JointStatesToMsg(const rabcl::protocol::JointState * joints, uint8_t count, const std::vector<std::string> & names, sensor_msgs::msg::JointState & msg)` — `id < names.size()` のものだけ詰める。読み飛ばしがあれば false
    - `void ImuToMsg(const rabcl::protocol::Imu &, sensor_msgs::msg::Imu &)` — 今の `Utils::InfoToImuMsg` と同じ変換
    - `bool ReferenceIsConsistent(const rabbits_msgs::msg::ReferenceCommand &)` — joint 系 6 配列の長さが等しく、mode 系 2 配列の長さが等しい
    - `void ReferenceToChassisVel(const ReferenceCommand &, rabcl::protocol::ChassisVel &)`
    - `uint8_t ReferenceToJointCmds(const ReferenceCommand &, const std::vector<std::string> & names, rabcl::protocol::JointCmd * out, uint8_t max)` — 名前を `names` の添字に変換。無い名前は読み飛ばす。戻り値は詰めた数
    - `uint8_t ReferenceToModes(const ReferenceCommand &, const std::vector<std::string> & mode_names, rabcl::protocol::ModeCmd * out, uint8_t max)`

- [ ] **Step 1: `ReferenceCommand.msg` を追加し、rabbits_msgs をビルド**

Run（rabbits_ws、Docker、`src/` に rabbits_msgs がある前提）: `colcon build --packages-select rabbits_msgs`
Expected: `Finished <<< rabbits_msgs`

- [ ] **Step 2: テストを書く（RED）** `rabcl_ros2/test/test_convert.cpp`

```cpp
TEST(Convert, JointStatesToMsgUsesNamesByIndex)   // names {"a","b","c"}, joints id {2,0} → name {"c","a"}, 値一致, true
TEST(Convert, JointStatesToMsgSkipsUnknownId)     // id 5 を含む → その分は入らず false
TEST(Convert, ImuToMsgQuaternionIdentityForZeroEuler)
TEST(Convert, ReferenceIsConsistentDetectsMismatch)
TEST(Convert, ReferenceWithMismatchedArraysIsRejected)  // joint_name 2 個, joint_position 1 個 → false
TEST(Convert, ReferenceToJointCmdsMapsNames)      // names {"yaw_joint","pitch_joint"}, msg name {"pitch_joint"} → out[0].id == 1
TEST(Convert, ReferenceToJointCmdsSkipsUnknownName)
TEST(Convert, ReferenceToModesMapsNames)
TEST(Convert, ReferenceToJointCmdsRespectsMax)
```

- [ ] **Step 3: RED を確認** Run: `colcon build --packages-select rabcl rabbits_msgs rabcl_ros2`（rabcl は Task 1 済みのもの） Expected: `convert.hpp: No such file`

- [ ] **Step 4: `convert.hpp` / `convert.cpp` を実装**（`Utils` は触らない）

- [ ] **Step 5: GREEN** Run: `colcon test --packages-select rabcl_ros2 && colcon test-result --verbose | grep -E "Summary|FAIL"` Expected: 0 failures

- [ ] **Step 6: README 更新**（rabbits_msgs: `ReferenceCommand` の説明。rabcl_ros2: `Convert` の説明、`Utils` は旧プロトコル用で beta 移行後に削除と明記）

- [ ] **Step 7: コミット（承認後）** rabbits_msgs `ReferenceCommand を追加`、rabcl_ros2 `新プロトコルと ROS メッセージの変換を追加`

---

### Task 3: rabbits_bridge の汎用ノード `robot_bridge`

**Files:**
- Create: `include/rabbits_bridge/robot_bridge_node.hpp`, `src/robot_bridge_node.cpp`, `config/beta.yaml`, `config/gamma.yaml`, `launch/robot_bridge.launch.py`, `test/test_robot_bridge.cpp`
- Modify: `CMakeLists.txt`（実行ファイル、テスト用 lib、`install(DIRECTORY config ...)`）、`package.xml`（`sensor_msgs` を `depend` に明記）、`README.md`

**Interfaces:**
- Consumes: Task 1 `rabcl::protocol::Parser` / `Encode*`、Task 2 `rabcl_ros2::Convert`、`rabbits_msgs::msg::ReferenceCommand`
- Produces: ノード `robot_bridge`。パラメータ `device`（string、既定 `/dev/ttyACM0`）、`baud`（int、既定 230400。対応表 115200 / 230400 / 460800 / 921600 / 1000000 / 2000000）、`joints`（string[]）、`modes`（string[]）。購読 `reference_command`、配信 `joint_states`（`sensor_msgs/JointState`）/ `imu/data`（frame_id `imu_link`）。`SYSTEM` フレームは受けても無視する（配信は後で）。テスト用コンストラクタ `RobotBridge(int test_fd, const rclcpp::NodeOptions &)`。

`config/beta.yaml`:
```yaml
robot_bridge:
  ros__parameters:
    device: /dev/ttyACM0
    baud: 230400
    joints: [yaw_joint, pitch_joint, right_front_wheel_joint, left_front_wheel_joint, right_back_wheel_joint, left_back_wheel_joint]
    modes: [load, fire, speed, chassis]
```
`config/gamma.yaml`: device `/dev/ttyTHS1`、baud 921600、joints `[yaw_joint, pitch_joint, roll_joint, right_front_wheel_joint, left_front_wheel_joint, right_back_wheel_joint, left_back_wheel_joint, right_front_leg_joint, left_front_leg_joint, right_back_leg_joint, left_back_leg_joint]`、modes 同じ。関節名は URDF と同じにする（roll と脚は URDF 追加時に同じ名前を使う）。設計書の `effort_unit` は YAML のコメントとして残し、ノードは読まない（計画作成者の判断）。

- [ ] **Step 1: テストを書く（RED）** `test/test_robot_bridge.cpp`（既存 `test_stm32_bridge.cpp` と同じ pipe 方式。パラメータは `NodeOptions().parameter_overrides` で `joints` 3 個 `{"j0","j1","j2"}`、`modes` `{"m0","m1"}`）

```cpp
TEST_F(RobotBridgeTest, JointStateFramePublishesJointStates)   // id {0,2} → name {"j0","j2"}
TEST_F(RobotBridgeTest, ImuFramePublishesImu)
TEST_F(RobotBridgeTest, SystemFrameIsIgnored)                  // 落ちない、何も配信しない
TEST_F(RobotBridgeTest, UnknownJointIdIsSkipped)               // id 5 を含む → name 2 つだけ、落ちない
TEST_F(RobotBridgeTest, CorruptedCrcNotPublished)
TEST_F(RobotBridgeTest, GarbageThenFrameRecovers)
TEST_F(RobotBridgeTest, InfoVersionMismatchDoesNotCrash)       // version 99 の INFO → 配信なし、落ちない
TEST_F(RobotBridgeTest, ReferenceCommandWritesFrames)
  // test_fd を socketpair にして書き込みを読む: chassis_vel のみ → CHASSIS_VEL 1 フレーム（17 B）
  // joint_name {"j1"} 付き → CHASSIS_VEL + JOINT_CMD（len 22）の 2 フレーム
TEST_F(RobotBridgeTest, EmptyArraysSendOnlyChassisVel)
TEST_F(RobotBridgeTest, MismatchedReferenceIsDropped)         // 何も書かれない
```

- [ ] **Step 2: RED を確認** Run: `colcon build --packages-select rabbits_bridge` Expected: `robot_bridge_node.hpp: No such file`

- [ ] **Step 3: ノードを実装**

受信: 1 ms タイマで `read()` できた分を `Parser::Feed`。ハンドラで type ごとに `Decode*` → `Convert` → publish。`INFO` は version 不一致で `RCLCPP_ERROR` を 1 回、joint_count と `joints.size()` の不一致で `RCLCPP_WARN` を 1 回。送信: `reference_command` のコールバックで `ReferenceIsConsistent` を確認（不一致は WARN して捨てる）→ `CHASSIS_VEL` を必ず、`JOINT_CMD` / `MODE` は要素があれば、それぞれ `write()`。

- [ ] **Step 4: GREEN** Run: `colcon test --packages-select rabbits_bridge && colcon test-result --verbose | grep -E "Summary|FAIL"` Expected: 0 failures（既存 `test_stm32_bridge` も残して通す）

- [ ] **Step 5: launch と README**（`robot_bridge.launch.py` は引数 `config`（YAML パス、既定 `config/beta.yaml`）。README に YAML の書き方と、`stm32_bridge_node` は旧プロトコル用で移行後に削除と明記）

- [ ] **Step 6: コミット（承認後）** `新プロトコル用の汎用ブリッジ robot_bridge を追加`

---

### Task 4: rabbits_interface が `ReferenceCommand` を出す

**Files:**
- Modify: `include/rabbits_interface/gamepad_node.hpp`, `src/gamepad_node.cpp`, `include/rabbits_interface/game_client_node.hpp`, `src/game_client_node.cpp`, `test/test_gamepad.cpp`, `test/test_game_client.cpp`, `README.md`, `launch/*.launch.py`（トピック名が変わる場合）

**Interfaces:**
- Consumes: `rabbits_msgs::msg::ReferenceCommand`
- Produces: 両ノードが `reference_command` に配信。`joint_name = {"yaw_joint", "pitch_joint"}`、`joint_position` に目標角、他の joint 配列は同じ長さで 0。`mode_name = {"load", "fire", "speed", "chassis"}`、`mode_value` に今の `load_mode` 等。`UpdateCmdVel(rabbits_msgs::msg::ReferenceCommand &)` のシグネチャ。擬似 `/joint_states` の配信と `positions_` の車輪分（添字 0〜3）は削除。yaw / pitch の前回目標（添字 4, 5）は残す（指令が増分だから）。

- [ ] **Step 1: テストを更新（RED）** 既存テストの `AttractsCommand` を `ReferenceCommand` に置き換え、`cmd.chassis_vel.x` の検証はそのまま。追加:

```cpp
TEST_F(GamepadTest, ReferenceCarriesYawPitchAndModes)
  // joint_name == {"yaw_joint","pitch_joint"}, 6 配列とも長さ 2, mode_name == {"load","fire","speed","chassis"}, mode_value 長さ 4
TEST_F(GamepadTest, FireButtonSetsFireMode)     // buttons[5] = 1 → mode_value[1] == 1
TEST_F(GamepadTest, NoJointStatesPublished)     // /joint_states に 100 ms 何も来ない
```

- [ ] **Step 2: RED を確認** Run: `colcon build --packages-select rabbits_interface` Expected: `UpdateCmdVel` の型不一致でコンパイルエラー

- [ ] **Step 3: 両ノードを実装**（game_client は gamepad と同じ変換。`ReferenceCommand` を組む共通関数を `include/rabbits_interface/reference_builder.hpp`（ヘッダのみ、`inline`）に置いて 2 ノードで共有）

- [ ] **Step 4: GREEN** Run: `colcon test --packages-select rabbits_interface && colcon test-result --verbose | grep -E "Summary|FAIL"` Expected: 0 failures

- [ ] **Step 5: README 更新、コミット（承認後）** `ReferenceCommand を出す形に変更し擬似 joint_states を廃止`

---

### Task 5: gamma_turret_fw の通信（rabcl_esp32 `SerialPort` + `robot.hpp` + 通信タスク）

**Files:**
- rabcl_esp32: Create `include/rabcl_esp32/serial_port.hpp`, `src/serial_port.cpp`, `test/fakes/driver/uart.h`, `test/test_serial_port.cpp`。Modify `CMakeLists.txt`（SRCS、`REQUIRES esp_driver_uart`）、`test/CMakeLists.txt`、`test/fakes/fake_state.hpp/.cpp`（UART の記録）、`README.md`
- gamma_turret_fw: Create `src/robot.hpp`, `src/comm.hpp`, `src/comm.cpp`。Modify `src/board.hpp`（`PC_UART_TX_PIN = GPIO_NUM_26`, `PC_UART_RX_PIN = GPIO_NUM_27`, `PC_UART_BAUD = 921600`, `PC_UART_PORT = UART_NUM_2`）、`src/main.cpp`、`src/CMakeLists.txt`（SRCS に comm.cpp）、`README.md`、submodule `components/rabcl` / `components/rabcl_esp32`

**Interfaces:**
- Consumes: Task 1 `rabcl::protocol`、rabcl の `Can::ParseDMMotorFeedback` 等
- Produces:
  - `rabcl_esp32::SerialPortConfig { uart_port_t port; gpio_num_t tx; gpio_num_t rx; uint32_t baud; size_t rx_buffer = 1024; size_t tx_buffer = 1024; }`
  - `class SerialPort { esp_err_t Start(); esp_err_t Stop(); int Read(uint8_t * buf, size_t len, TickType_t timeout); int Write(const uint8_t * buf, size_t len); }`（`Read` は読めた数、`Write` は書けた数。負は `esp_err_t` 相当のエラー）
  - gamma `robot.hpp`: `namespace robot { constexpr char FIRMWARE_ID[8] = "gamma"; enum Joint : uint8_t { YAW, PITCH, ROLL, WHEEL_FR, WHEEL_FL, WHEEL_BR, WHEEL_BL, LEG_FR, LEG_FL, LEG_BR, LEG_BL, JOINT_COUNT }; enum Mode : uint8_t { LOAD, FIRE, SPEED, CHASSIS, MODE_COUNT }; }` と、関節 → バス / CAN_ID / MST_ID / モータ種別 の表（board.hpp の ID 定数を参照）
  - gamma `comm.hpp`: `class Comm { esp_err_t Start(); void SetJointState(const rabcl::protocol::JointState &); void SetImu(const rabcl::protocol::Imu &); bool GetJointCmd(uint8_t id, rabcl::protocol::JointCmd &); bool GetChassisVel(rabcl::protocol::ChassisVel &); bool GetMode(uint8_t id, uint8_t &); uint32_t rx_frames() const; }` — 内部に送信タスク（100 Hz で `JOINT_STATE`（持っている関節のみ、14 を超えたら分割）、1 Hz で `INFO` / `SYSTEM`）と受信タスク（`Read` → `Parser::Feed` → 共有状態、mutex で保護）

- [ ] **Step 1: rabcl_esp32 のテストを書く（RED）** `test/test_serial_port.cpp`（fake の `driver/uart.h` は `uart_driver_install / uart_param_config / uart_set_pin / uart_write_bytes / uart_read_bytes / uart_driver_delete` を記録し、`fake_state::uart()` に受信用のバイト列を仕込める）

```cpp
TEST(SerialPort, StartConfiguresPortPinsAndBaud)   // 記録に port / tx / rx / baud / 8N1 が残る
TEST(SerialPort, StartTwiceIsInvalidState)
TEST(SerialPort, WriteBeforeStartIsInvalidState)
TEST(SerialPort, WritePassesBytes)
TEST(SerialPort, ReadReturnsQueuedBytes)           // 仕込んだ 5 B → Read(…, 8, …) == 5
TEST(SerialPort, StopDeletesDriverAndAllowsRestart)
```

- [ ] **Step 2: RED → 実装 → GREEN**（Docker で `cmake -S test -B build && cmake --build build -j && ctest --test-dir build`） Expected: 既存 28 + 6 件すべて PASS

- [ ] **Step 3: rabcl_esp32 の README とコミット（承認後）** `SerialPort を追加`。push 後、gamma の submodule を上げる

- [ ] **Step 4: gamma の `robot.hpp` / `comm.hpp` / `comm.cpp` を実装し、`main.cpp` を次の形にする**

DM4310 試験ループはそのまま（段階 2 の `SEND_MIT_ZERO_CMD` は true にしてよい）。フィードバックを受けるたびに `robot::LEG_FR` の `JointState` を `Comm::SetJointState` に渡す。受け取った `JOINT_CMD`（LEG_FR）と `CHASSIS_VEL` は 1 秒ごとにログに出すだけ（制御には使わない）。

- [ ] **Step 5: ビルド** Run（PowerShell）: `pio run -d fw_ws/gamma_turret_fw` Expected: SUCCESS、警告なし

- [ ] **Step 6: ループバック確認（実機）** G26 と G27 をジャンパ線で直結し、`Comm` が自分の `INFO` を受信して `rx_frames()` が 1 秒に 1 以上増えることをログで確認。増えなければ G26 / G27 の USB 機能（USB-Serial-JTAG の FS PHY）が邪魔している可能性があるので、`sdkconfig.defaults` で USB-Serial-JTAG の PHY 選択を確認し、必要なら `usb_serial_jtag` を無効化して再確認（設計時の懸念事項）。
Expected: `rx_frames` が増える、CRC エラー 0

- [ ] **Step 7: PC 側確認（実機）** Jetson がまだ無いので、Windows の USB-UART（3.3 V）を G26 / G27 / GND につなぎ、Python（pyserial）の使い捨てスクリプト（scratchpad に置き、リポジトリには入れない）で `INFO` と `JOINT_STATE` を復号して表示。`JOINT_CMD` を 1 つ送って gamma 側のログに出ることも確認。
Expected: `INFO` version 1 / joint_count 11 / "gamma"、`JOINT_STATE` に id 7 の位置・温度が 100 Hz 前後で来る

- [ ] **Step 8: README 更新、コミット（承認後）** `robot.hpp と通信タスクを追加`（必要なら submodule 更新は別コミット `rabcl と rabcl_esp32 を更新`）

---

### Task 6（後回し。beta を移行するときに実施）: beta_main を新プロトコルへ移行

**Files:**
- Create: `Inc/robot.hpp`（CAN ID 表: `YAW_ID 0x141`, `PITCH_CAN_ID 0x42`, `PITCH_MST_ID 0x52`, `WHEEL_CAN_ID[4] {0x43..0x46}`, `WHEEL_MST_ID[4] {0x53..0x56}`、関節番号 enum は bridge の `beta.yaml` と同じ順）、`Inc/robot_state.hpp`（`struct RobotState`: 今の `rabcl::Info` と同じ欄）
- Modify: `Src/main.cpp`（`rabcl::Info` → `RobotState`、`rabcl::Can::UpdateData` → 自前の ID 振り分け + `ParseDMMotorFeedback` / `ParseLKMotorFeedback`、UART を新プロトコルへ）、`Src/usart.c`（`hdma_usart2_rx.Init.Mode = DMA_CIRCULAR`）、`.gitmodules`（URL を `https://github.com/rabbits-robotics/rabcl.git`）、`README.md`

**Interfaces:**
- Consumes: Task 1 `rabcl::protocol`、rabcl の `Parse*`
- Produces: 100 Hz で `JOINT_STATE`（6 関節: yaw = LK の current を effort に、pitch = DM3507、車輪 4 = DM2325、順番は `beta.yaml`）+ `IMU`（0 のまま）、1 Hz で `INFO`（version 1, joint_count 6, "beta"）。受信は `CHASSIS_VEL` → `chassis_vel_*`、`JOINT_CMD` の id 0 / 1 → `yaw_pos_` / `pitch_pos_`、`MODE` → 4 つのモード。

- [ ] **Step 1: submodule を付け替え**（`git submodule set-url lib/rabcl https://github.com/rabbits-robotics/rabcl.git` → Task 1 のコミットに checkout）。Run: `pio run` Expected: `rabcl::Info` が無い等のエラーは **まだ出ない**（旧 API は Task 7 まで残る）→ SUCCESS

- [ ] **Step 2: `robot.hpp` / `robot_state.hpp` を作り、CAN 受信の振り分けを自前にする**（`HAL_CAN_RxFifo0MsgPendingCallback`。DM2325 の符号反転もここ）。Run: `pio run` Expected: SUCCESS

- [ ] **Step 3: UART 受信を `ReceiveToIdle_DMA`（循環 256 B）に変更**。`HAL_UARTEx_RxEventCallback(huart, size)` で前回位置から `size` までの新着分（折り返し対応）を `Parser::Feed`。ハンドラで `Decode*` → `RobotState`。`HAL_UART_ErrorCallback` ではフラグを消して `ReceiveToIdle_DMA` を再開。今の「DMA 見張り（RxState が READY なら再開）」は残す。

- [ ] **Step 4: UART 送信を `EncodeJointState` + `EncodeImu`（連結して 1 回の `HAL_UART_Transmit_IT`）に変更、1 Hz で `EncodeInfo`**。Run: `pio run` Expected: SUCCESS

- [ ] **Step 5: 実機確認** beta を PC（Docker ではなく、Jetson か Linux 機。無ければ WSL に USB をパススルー）につなぎ、`robot_bridge`（`config/beta.yaml`）+ gamepad で: `ros2 topic echo /joint_states` に 6 関節が 100 Hz 前後で出る、ゲームパッドで車体が走り砲塔が動く、射出と装填のモードが効く。
Expected: 今の beta と同じ操作感

- [ ] **Step 6: README 更新、コミット（承認後）** `新プロトコルに移行し CAN ID 表を機体側に移す`

---

### Task 7（後回し。Task 6 の後に実施）: 旧コードの削除

**Files:**
- rabcl: Delete `include/rabcl/interface/uart.hpp`, `src/interface/uart.cpp`, `test/test_uart.cpp`。Modify `include/rabcl/utils/type.hpp`（`CAN_ID`, `MODE_ID`, `Info` を削除。`MotorInfo` / `ImuInfo` は残す）、`include/rabcl/interface/can.hpp` / `src/interface/can.cpp`（`UpdateData` を削除）、`test/test_can.cpp`（`UpdateData*` のテストを削除）、`CMakeLists.txt`、`README.md`
- rabcl_ros2: Delete `include/rabcl_ros2/utils.hpp`, `src/utils.cpp`, `test/test_utils.cpp`。Modify `CMakeLists.txt`、`README.md`
- rabbits_bridge: Delete `include/rabbits_bridge/stm32_bridge_node.hpp`, `src/stm32_bridge_node.cpp`, `test/test_stm32_bridge.cpp`, `launch/stm32_bridge.launch.py`, `launch/stm32_bringup.launch.py`（中身を `robot_bridge` に置き換えて残す場合はその旨）。Modify `CMakeLists.txt`、`README.md`
- rabbits_msgs: Delete `msg/AttractsCommand.msg`。Modify `CMakeLists.txt`、`README.md`
- rabbits_interface: `AttractsCommand` 参照が残っていないことを確認

- [ ] **Step 1: 削除前に、各パッケージで `grep -rn "AttractsCommand\|rabcl::Info\|CAN_ID::\|UpdateData\|rabcl/interface/uart" src/` が rabbits_ws 内で 0 件（削除対象のファイル以外）であることを確認**

- [ ] **Step 2: rabcl から削除し、Docker でビルド / テスト** Expected: 0 failures（テスト数は減る）

- [ ] **Step 3: rabcl_ros2、rabbits_bridge、rabbits_msgs から削除し、rabbits_ws 全体を `colcon build && colcon test`** Expected: 全パッケージ 0 failures

- [ ] **Step 4: gamma_turret_fw と beta_main の submodule を新しい rabcl に上げてビルド** Expected: 両方 SUCCESS

- [ ] **Step 5: README（rabcl / rabcl_ros2 / rabbits_bridge / rabbits_msgs / handbook common-lib.md の該当箇所）を更新、コミット（承認後）** rabcl `beta 固有の CAN ID 表と固定レイアウトの UART を削除` ほか各リポジトリ 1 コミット

---

## 実施メモ

- 各タスクの実機確認は、ユーザーの作業（配線、電源、ゲームパッド操作）を伴う。確認手順と期待値を伝えて結果を聞く。
- Task 5 の Step 6〜7 と Task 6 の Step 5 は機材が必要（USB-UART、Jetson または Linux 機）。無ければその旨を台帳に記録して、できるところまで進める。
- CI は各リポジトリの push 後に `gh run watch` で確認する。
