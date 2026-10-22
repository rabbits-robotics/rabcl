# rabcl
[![CI](https://github.com/rabbits-robotics/rabcl/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabcl/actions/workflows/ci.yml)

Rabbits Common Library。モータ・センサのドライバ、制御器、上位コントローラとの通信フレームを、ファームウェア（STM32 / ESP32）と ROS 2 の両方から使う C++17 ライブラリ。
特定のマイコンや OS に依存せず（`<cstdint>` と標準ライブラリのみ）、機体固有の値（CAN ID、ゲイン、ピン）は持たない。

## 構成

| ディレクトリ | 内容 |
|---|---|
| `component/` | モータ・センサ: JGA25-370（エンコーダ付き DC モータ）、LD-20MG（サーボ）、BNO055（IMU） |
| `controller/` | 制御器: `OmniDrive`（オムニ 4 輪の速度分配）、`PdGravityFf`（重力補償付き PD） |
| `interface/` | モータとの通信: `Can`（LK / DAMIAO の CAN ペイロードの組み立て `Prepare*` と解析 `Parse*`。ID の振り分けは各ファームが行う）。`Uart`（`Info` の固定レイアウト）は beta 専用で、beta を `protocol/` に移行したあと削除する |
| `protocol/` | PC ⇔ ファームの共通プロトコル。フレーム化（ヘッダ、長さ、CRC-8）、メッセージ（`INFO` / `JOINT_STATE` / `IMU` / `SYSTEM` / `CHASSIS_VEL` / `JOINT_CMD` / `MODE`）の Encode / Decode、受信パーサ `Parser`。機体ごとの違いは関節番号と名前の対応だけで、rabcl は持たない（設計書 `docs/superpowers/specs/2026-10-22-robot-protocol-design.md`） |
| `utils/` | `Info`（機体の指令・状態をまとめた構造体）、CAN ID の定数、数値ユーティリティ |

通信の実体（CAN コントローラ、UART ペリフェラル）は持たない。`Can::Prepare*` が作る `uint8_t[8]` を各プラットフォームの CAN 送信に渡し、受信した 8 バイトを `Can::UpdateData` に渡す。

## 使い方

### STM32 ファームウェアから（PlatformIO + STM32Cube）

```bash
git submodule add https://github.com/rabbits-robotics/rabcl.git lib/rabcl
```

PlatformIO が `lib/` 以下を自動でビルドする。

### ESP32 ファームウェアから（PlatformIO + ESP-IDF）

```bash
git submodule add https://github.com/rabbits-robotics/rabcl.git components/rabcl
```

`CMakeLists.txt` が ESP-IDF のコンポーネントとしても動く（`ESP_PLATFORM` で分岐）ので、`src/CMakeLists.txt` の `REQUIRES` に `rabcl` を足す。
ESP-IDF のドライバを包んだ `CanBus` などは [rabcl_esp32](https://github.com/rabbits-robotics/rabcl_esp32) にある。

### ROS 2 から

`rabbits_ws` の `機体名.repos` に追加する。ROS 2 向けの変換（メッセージ ⇔ `rabcl::Info`）は [rabcl_ros2](https://github.com/rabbits-robotics/rabcl_ros2) にある。

## ビルド・テスト

```bash
colcon build --packages-select rabcl
colcon test --packages-select rabcl
colcon test-result --verbose
```

テストは `test/` に gtest で置いている。rabcl 系のリポジトリはユニットテストを必ず書き、CI で実行する（handbook の [common-lib.md](https://github.com/rabbits-robotics/.github/blob/main/docs/common-lib.md)）。コード整形は ament の uncrustify に従う。

## ライセンス

MIT。[LICENSE](./LICENSE) を参照。
