# gamma の脚リンク機構の運動学と実寸 URDF 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** gamma の脚の 4 節リンクの運動学を rabcl に置き、ファーム・偽ファーム・URDF（rviz）が同じ寸法で車高・傾きを出すようにする。

**Architecture:** rabcl に `LegLinkage`（θ ↔ φ・車輪の下がり）を新設し、`BodyPosture` を「車輪の下がり」基準に、`LegDamper` を荷重トルク引数付きに変える。寸法は gamma_turret_fw `robot.hpp` と rabbits_description `config/gamma_linkage.yaml` の 2 か所に置き、xacro は `load_yaml`、偽ファームと派生関節ノードは ROS パラメータとして同じ YAML を読む。rviz の閉ループ（バー角・連結リンク角）は rabbits_description の C++ ノード `linkage_state_publisher`（ミキサー）が `motor_joint_states` を受けて派生関節を足した `joint_states` を出す（1 トピック 1 発行元）。

**Tech Stack:** C++17、gtest、ROS 2 Jazzy（rclcpp、xacro、robot_state_publisher）、ESP-IDF（pioarduino）。テストは Docker（`rabbits/ros2-jazzy-dev`）で `colcon build / test`、ファームは PowerShell から `pio run`。

**Spec:** `rabbits_ws/src/rabcl/docs/superpowers/specs/2026-10-25-gamma-leg-linkage-design.md`

## Global Constraints

- 実装は C++（launch の `.launch.py` だけ例外）。rabcl 系はユニットテスト必須。
- 定数は UPPER_SNAKE_CASE。include は外部 `<>` / 自リポ `""`、グループを空行で分け、.cpp は対応ヘッダを先頭に。
- rabcl に機体の値（gamma の寸法）を書かない。テストの中の値は可。
- コミットはユーザーのレビューと明示的な承認後。メッセージは 1 行の日本語。ファームのコードは実機確認後にコミット（ビルドまで）。
- rabcl の API は m / rad。寸法（設計書 2 章）を m / rad に直した値:
  `crank_length 0.041`, `crank_angle_zero 1.0471976`（π/3）, `coupler_length 0.03975`, `bar_length 0.060`, `bar_pivot (−0.026, −0.049)`, `bar_pivot_lower (−0.026, −0.069)`, `coupler_pin (0.0, 0.0175)`, `wheel (0.044, −0.0325)`, `theta_max 1.5707963`, `drop_min 0.05756`, `drop_max 0.10377`, `leg_radius 0.092`, `tilt_radius 0.170`, `wheel_radius 0.0375`, `wheel_width 0.013`, `yaw_origin_z −0.0255`, `BODY_D 0.34`, `WHEEL_D 0.075`。
- 設計書 3.1 の期待値（m / rad）: θ = 0 → φ 0.4821, drop 0.05368, coupler −2.4496；θ = 0.15 → φ 0.4105, drop 0.05756；θ = π/6 → φ 0.2073, drop 0.06915；θ = π/3 → φ −0.1260, drop 0.08904, coupler −1.7600；θ = 1.42 → φ −0.3803, drop 0.10377；θ = π/2 → φ −0.4829, drop 0.10936, coupler −1.2593。dd/dθ: 0.0273（θ 0.15）, 0.0385（θ 0.8）。
- 関節の回転軸は +y（正 = 下向き）。`bar_joint` = −φ、`coupler_joint` = −coupler_angle。

## Review Focus

1. `LegLinkage::Forward` が θ 範囲の外側（θ < 0 や > theta_max でも円が交わる領域）で上の枝（φ ≈ 1.45）を選ばないこと → Task 1 のテスト `ForwardPicksLowerBranchEverywhere`。
2. `BodyPosture::Solve` で傾きが大きく `d_i` が範囲外になったとき、クランプ後に `Inverse` が必ず解けて `theta` が [0.15, 1.42] に収まること → Task 2 のテスト `SolveClampsEveryLeg`。
3. `linkage_state_publisher` が脚モータ関節を含まない `motor_joint_states`（首だけの更新など）を受けたとき、入力をそのまま写して派生 8 関節（直前の姿勢）を付けて出すこと → Task 3 のテスト `PassesThroughJointStatesWithoutLegs`。
4. 偽ファームの脚の初期角 0（θ = 0）が `drop_min` より下（0.05368 < 0.05756）で、起動直後の `BodyPosture` 目標へのランプが今までどおり θ の速度で進むこと → Task 4 のテスト `StartsAtLowestHeightAndRampsToBodyPostureTarget`。
5. URDF の xacro が `load_yaml` の値で展開でき、`check_urdf` が通り、可動関節 11 + 派生 8 + mimic 8 になること → Task 3 の検証ステップ。

---

### Task 1: rabcl `LegLinkage`

**Files:**
- Create: `rabbits_ws/src/rabcl/include/rabcl/controller/leg_linkage.hpp`
- Create: `rabbits_ws/src/rabcl/src/controller/leg_linkage.cpp`
- Create: `rabbits_ws/src/rabcl/test/test_leg_linkage.cpp`
- Modify: `rabbits_ws/src/rabcl/CMakeLists.txt`（`add_library` の SRCS に `src/controller/leg_linkage.cpp`、`ament_add_gtest(test_leg_linkage test/test_leg_linkage.cpp)` + link）
- Modify: `rabbits_ws/src/rabcl/README.md`（controller の表に 1 行）

**Interfaces:**
- Produces:
  ```cpp
  namespace rabcl {
  struct LegLinkageConfig {
    float crank_length; float crank_angle_zero; float coupler_length; float bar_length;
    float bar_pivot_x; float bar_pivot_z; float coupler_pin_x; float coupler_pin_z;
    float wheel_x; float wheel_z; float theta_max;
  };
  struct LegPose { float bar_angle; float coupler_angle; float wheel_x; float wheel_drop; };
  class LegLinkage {
  public:
    explicit LegLinkage(const LegLinkageConfig & config);
    bool Forward(float theta, LegPose & out) const;      // 円が交わらなければ false、out は変えない
    bool Inverse(float wheel_drop, float & theta) const;  // 同上。θ は [0, theta_max] に収まる解、無ければ false
    float DropJacobian(float theta) const;               // 中心差分 Δθ = 1e-3
    float drop_min() const;  // Forward(0).wheel_drop
    float drop_max() const;  // Forward(theta_max).wheel_drop
  };
  }
  ```

- [ ] **Step 1: 失敗するテストを書く**

```cpp
// test/test_leg_linkage.cpp（gamma の寸法はテスト内の定数。rabcl 本体には書かない）
#include <gtest/gtest.h>
#include <cmath>
#include "rabcl/controller/leg_linkage.hpp"

namespace rabcl
{
constexpr LegLinkageConfig GAMMA = {0.041f, 1.0471976f, 0.03975f, 0.060f, -0.026f, -0.049f,
  0.0f, 0.0175f, 0.044f, -0.0325f, 1.5707963f};

TEST(LegLinkage, ForwardMatchesCadAtZero)
{
  LegLinkage link(GAMMA);
  LegPose p;
  ASSERT_TRUE(link.Forward(0.0f, p));
  EXPECT_NEAR(p.bar_angle, 0.4821f, 1e-3f);
  EXPECT_NEAR(p.wheel_drop, 0.05368f, 1e-4f);
  EXPECT_NEAR(p.wheel_x, 0.0712f, 1e-4f);
  EXPECT_NEAR(p.coupler_angle, -2.4496f, 1e-3f);
}

TEST(LegLinkage, ForwardMatchesCadAcrossRange)
{
  LegLinkage link(GAMMA);
  LegPose p;
  ASSERT_TRUE(link.Forward(static_cast<float>(M_PI / 6), p));
  EXPECT_NEAR(p.bar_angle, 0.2073f, 1e-3f);
  EXPECT_NEAR(p.wheel_drop, 0.06915f, 1e-4f);
  ASSERT_TRUE(link.Forward(static_cast<float>(M_PI / 3), p));
  EXPECT_NEAR(p.bar_angle, -0.1260f, 1e-3f);
  EXPECT_NEAR(p.wheel_drop, 0.08904f, 1e-4f);
  EXPECT_NEAR(p.coupler_angle, -1.7600f, 1e-3f);
  ASSERT_TRUE(link.Forward(static_cast<float>(M_PI / 2), p));
  EXPECT_NEAR(p.bar_angle, -0.4829f, 1e-3f);
  EXPECT_NEAR(p.wheel_drop, 0.10936f, 1e-4f);
  EXPECT_NEAR(p.coupler_angle, -1.2593f, 1e-3f);
}

TEST(LegLinkage, ForwardPicksLowerBranchEverywhere)
{
  LegLinkage link(GAMMA);
  LegPose p;
  for (float theta = -0.5f; theta <= 2.0f; theta += 0.05f) {
    if (link.Forward(theta, p)) {
      EXPECT_LT(p.bar_angle, 0.6f) << "theta " << theta;  // 上の枝は 1.45 付近
    }
  }
}

TEST(LegLinkage, InverseRoundTrips)
{
  LegLinkage link(GAMMA);
  for (float theta = 0.0f; theta <= 1.5707963f; theta += 0.1f) {
    LegPose p;
    ASSERT_TRUE(link.Forward(theta, p));
    float back = -1.0f;
    ASSERT_TRUE(link.Inverse(p.wheel_drop, back)) << "theta " << theta;
    EXPECT_NEAR(back, theta, 1e-4f);
  }
}

TEST(LegLinkage, OutOfRangeReturnsFalseAndKeepsOutput)
{
  LegLinkage link(GAMMA);
  float theta = 0.3f;
  EXPECT_FALSE(link.Inverse(0.03f, theta));   // drop_min 0.05368 より浅い
  EXPECT_FALSE(link.Inverse(0.13f, theta));   // drop_max 0.10936 より深い
  EXPECT_FLOAT_EQ(theta, 0.3f);
  LegPose p = {1.0f, 2.0f, 3.0f, 4.0f};
  EXPECT_FALSE(link.Forward(4.0f, p));        // クランクが連結リンクの届く範囲外
  EXPECT_FLOAT_EQ(p.bar_angle, 1.0f);
}

TEST(LegLinkage, DropRangeAndJacobian)
{
  LegLinkage link(GAMMA);
  EXPECT_NEAR(link.drop_min(), 0.05368f, 1e-4f);
  EXPECT_NEAR(link.drop_max(), 0.10936f, 1e-4f);
  EXPECT_NEAR(link.DropJacobian(0.15f), 0.0273f, 5e-4f);
  EXPECT_NEAR(link.DropJacobian(0.8f), 0.0385f, 5e-4f);
  EXPECT_GT(link.DropJacobian(0.0f), 0.0f);
}
}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

`Forward(4.0f)` が false になることはステップ 2 の実行で確かめる（θ = 4.0 rad でクランクが真下を越え、連結リンクが届かない配置になる。届いてしまったら 3.5 など別の値に替え、理由をテストのコメントに書く）。

- [ ] **Step 2: 失敗を確認**

Run（Docker、rabbits_ws を bind mount）:
```
colcon build --packages-select rabcl 2>&1 | grep -E "error|Summary"
```
Expected: `leg_linkage.hpp: No such file` のコンパイルエラー。

- [ ] **Step 3: 実装**

`leg_linkage.hpp` に Interfaces の宣言（ヘッダ先頭コメントに設計書 2〜3 章の記号と枝の選び方を書く）。`leg_linkage.cpp`:
- `Forward`: α = crank_angle_zero − θ、K = Lc(cos α, sin α)、C = (bar_pivot + coupler_pin)。2 円（C, Lb）と（K, Lr）の交点を求め、J.z が低い方を選ぶ。φ = atan2(J − C)。H1 = bar_pivot + Lb(cos φ, sin φ)、W = H1 + wheel。`out = {φ, atan2(J − K) − α, W.x, −W.z}`。交点が無ければ false。
- `Inverse`: sin φ = (−d − bar_pivot_z − wheel_z) / Lb、|sin φ| > 1 で false、cos φ > 0。J = C + Lb(cos φ, sin φ)。2 円（M = 0, Lc）と（J, Lr）の交点 K の候補 2 つから α = atan2(K)、θ = crank_angle_zero − α を [−π, π) に正規化し、[0, theta_max] に入る方（両方なら 0 に近い方）。無ければ false。
- `DropJacobian`: (Forward(θ+1e-3).drop − Forward(θ−1e-3).drop) / 2e-3。どちらかが false なら 0。
- コンストラクタで drop_min_ / drop_max_ を Forward(0) / Forward(theta_max) から計算（false なら 0）。

- [ ] **Step 4: テストが通ることを確認**

Run: `colcon test --packages-select rabcl` → `colcon test-result --verbose`
Expected: test_leg_linkage 6/6 PASS、uncrustify / cpplint 差分なし、既存テストも GREEN。

- [ ] **Step 5: README の controller 表に `LegLinkage` を足し、コミット（承認後）**

`脚リンク機構の運動学 LegLinkage を追加`

---

### Task 2: rabcl `BodyPosture` を車輪の下がり基準に、`LegDamper` に荷重トルク引数

**Files:**
- Modify: `rabbits_ws/src/rabcl/include/rabcl/controller/body_posture.hpp`, `src/controller/body_posture.cpp`, `test/test_body_posture.cpp`
- Modify: `rabbits_ws/src/rabcl/include/rabcl/controller/leg_damper.hpp`, `src/controller/leg_damper.cpp`, `test/test_leg_damper.cpp`
- Modify: `rabbits_ws/src/rabcl/README.md`

**Interfaces:**
- Consumes: Task 1 の `LegLinkage` / `LegLinkageConfig`
- Produces:
  ```cpp
  struct BodyPostureConfig {
    LegLinkageConfig linkage;
    float drop_min; float drop_max;   // 車高 0 / 1 の車輪の下がり [m]
    float tilt_radius;                // [m]
    float leg_azimuth[4];
  };
  class BodyPosture {
  public:
    explicit BodyPosture(const BodyPostureConfig & config);
    void Solve(float height, float roll, float pitch, float yaw_offset, float out[4]) const;  // out = θ [rad]
    const LegLinkage & linkage() const;
  };
  struct LegDamperConfig { float kp; float kd; float friction_ff; float deadband; float ramp_rate; };  // load_ff を削除
  MitCommand LegDamper::Update(float target, float angle, float vel, float dt, float load_torque);
  ```

- [ ] **Step 1: 失敗するテストに書き換える（test_body_posture.cpp）**

fixture を新 config に（`GAMMA` の linkage は Task 1 のテストと同じ値、`drop_min 0.05756`, `drop_max 0.10377`, `tilt_radius 0.170`, azimuth は今のまま）。`ExpectDrops(fr, fl, br, bl)` ヘルパを足す: `out_[i]` を `posture_.linkage().Forward` で `wheel_drop` に戻し、`1e-4` で比較。テスト:
- `HeightMapsToDropLinearly`: height 0 → θ 全部 0.15 ± 1e-3、height 1 → 1.42 ± 1e-3、height 0.5 → drop 全部 0.080665 ± 1e-4。
- `RollPositiveLowersRightSide`: height 0.5, roll 0.1 → drop FR / BR = 0.068644、FL / BL = 0.092686（±1e-4）。
- `PitchPositiveLowersFront`: height 0.5, pitch 0.1 → drop FR / FL = 0.068644、BR / BL = 0.092686。
- `YawOffsetRotatesTiltIntoChassisFrame`: height 0.5, pitch 0.1, yaw_offset π/2 → drop FL / BL = 0.068644、FR / BR = 0.092686。
- `YawOffsetPiFlipsSign`: height 0.5, roll 0.1, yaw_offset π → FL / BL = 0.068644、FR / BR = 0.092686。
- `SolveClampsEveryLeg`: (2, 5, −5, 0.3) と (−1, 0, 0, 0) で全 θ が [0.15 − 1e-3, 1.42 + 1e-3]、後者は全部 0.15。

- [ ] **Step 2: 失敗するテストに書き換える（test_leg_damper.cpp）**

config から `load_ff` を外し（kp 1, kd 0.1, friction_ff 0.25, deadband 0.02, ramp_rate 0.5）、`Update(...)` に第 5 引数 `0.5f` を足す（期待値はそのまま）。追加:
- `LoadTorqueAddsToTff`: `Update(0.5, 0.505, 0, 0.01, 1.25).tff == 1.25`、`Update(0.5, 0.4, 0, 0.01, 1.25).tff == 1.5`。

- [ ] **Step 3: 失敗を確認**

Run: `colcon build --packages-select rabcl 2>&1 | grep -E "error|Summary"`
Expected: `BodyPostureConfig` に `linkage` が無い / `Update` の引数が 4 つ、のコンパイルエラー。

- [ ] **Step 4: 実装**

`BodyPosture`: メンバに `LegLinkage linkage_`（config.linkage から構築）。`Solve`: d0 = drop_min + h·(drop_max − drop_min)、d_i = d0 + tilt_radius·(sin(az_i − yaw_offset)·roll − cos(az_i − yaw_offset)·pitch)、[drop_min, drop_max] にクランプ、`linkage_.Inverse(d_i, out[i])`。false のときは `out[i]` を d_i が drop_min 側なら `Inverse(drop_min)`、そうでなければ `Inverse(drop_max)` の値（コンストラクタで一度解いて持つ `theta_min_` / `theta_max_`）。
`LegDamper`: `tff = load_torque` から始める。ヘッダのコメントで「荷重分は呼び出し側が `F · LegLinkage::DropJacobian(angle)` で作る」と書く。

- [ ] **Step 5: テストが通ることを確認**

Run: `colcon test --packages-select rabcl` → `colcon test-result --verbose`
Expected: body_posture 6/6、leg_damper 6/6、全体 GREEN、lint 差分なし。

- [ ] **Step 6: README の `BodyPosture` / `LegDamper` の説明を更新し、コミット（承認後）**

`BodyPosture を車輪の下がり基準にし、LegDamper の荷重分を引数にする`

---

### Task 3: rabbits_description — 寸法 YAML、実寸 URDF、派生関節ノード

**Files:**
- Create: `rabbits_ws/src/rabbits_description/config/gamma_linkage.yaml`
- Modify: `rabbits_ws/src/rabbits_description/urdf/gamma/leg.xacro`（書き直し）, `urdf/gamma/gamma.xacro`, `urdf/gamma/neck.xacro`（Pitch / Roll 原点）
- Create: `rabbits_ws/src/rabbits_description/include/rabbits_description/linkage_state_publisher.hpp`, `src/linkage_state_publisher.cpp`, `test/test_linkage_state_publisher.cpp`
- Modify: `rabbits_ws/src/rabbits_description/CMakeLists.txt`, `package.xml`, `README.md`, `launch/display.launch.py`（gamma のとき派生関節ノードも起動）

**Interfaces:**
- Consumes: Task 1 の `LegLinkage`
- Produces:
  - `config/gamma_linkage.yaml`（`/**:` → `ros__parameters:` → `linkage:` 以下に Global Constraints の値。キー: `crank_length, crank_angle_zero, coupler_length, bar_length, bar_pivot: [x, z], bar_pivot_lower: [x, z], coupler_pin: [x, z], wheel: [x, z], theta_max, drop_min, drop_max, leg_radius, tilt_radius, wheel_radius, wheel_width, yaw_origin_z`）。先頭コメントに「gamma_turret_fw の src/robot.hpp と対で更新する」。
  - ノード `linkage_state_publisher`（実行ファイル名同じ）。パラメータ: `linkage.*`（上の YAML）、`leg_joints`（既定 `[right_front_leg_joint, left_front_leg_joint, right_back_leg_joint, left_back_leg_joint]`）、`prefixes`（既定 `[right_front, left_front, right_back, left_back]`）。購読 `motor_joint_states`、配信 `joint_states`（入力の全関節をそのまま + `<p>_bar_joint`, `<p>_coupler_joint` × 4）。
  - URDF の関節名: `<p>_leg_joint`（θ）、`<p>_coupler_joint`、`<p>_bar_joint`、`<p>_bar_lower_joint`（mimic bar ×1）、`<p>_holder_joint`（mimic bar ×−1）、`<p>_wheel_joint`。
  - クラス `LinkageStatePublisher : rclcpp::Node` に `void JointStateCB(const sensor_msgs::msg::JointState::SharedPtr)`（テストから直接呼ぶ）。

- [ ] **Step 1: YAML を書く**

上のキーで `config/gamma_linkage.yaml`。

- [ ] **Step 2: 失敗するテストを書く（test_linkage_state_publisher.cpp）**

Fixture: `rclcpp::init` を `SetUpTestSuite`、`NodeOptions().parameter_overrides` で `linkage.*` を Global Constraints の値で与え（YAML は読まない）、`joint_states` を購読して受信を貯める。
- `MixesMotorAndDerivedJoints`: 4 脚の θ = 0 / π/6 / π/3 / π/2 と `yaw_joint 0.7` を 1 つの `JointState`（velocity / effort も 5 個）で `JointStateCB` → 受信 1 件、名前 13 個（入力 5 + 派生 8、入力の順を保ち派生は後ろ）、`yaw_joint` の position / velocity / effort がそのまま、`right_front_bar_joint = −0.4821`、`right_front_coupler_joint = +2.4496`、`left_front_bar_joint = −0.2073`、`right_back_bar_joint = +0.1260`、`left_back_coupler_joint = +1.2593`（±1e-3）、派生の velocity / effort は 0、stamp が入力と同じ。
- `PassesThroughJointStatesWithoutLegs`: `yaw_joint` だけの `JointState` → 受信 1 件、名前 9 個、`right_front_bar_joint = −0.4821`（Forward(0) の姿勢）。
- `KeepsLastPoseWhenOutOfRange`: θ = 0.3 → 受信、次に θ = 4.0 → 受信するが派生の値は 0.3 のときと同じ。

- [ ] **Step 3: 失敗を確認**

Run: `colcon build --packages-select rabbits_description 2>&1 | grep -E "error|Summary"`
Expected: ヘッダ無しのコンパイルエラー（CMake にテストと実行ファイルを足した状態で）。

- [ ] **Step 4: ノードを実装、CMake / package.xml**

CMake: C++17、`find_package(rclcpp sensor_msgs rabcl)`、実行ファイル `linkage_state_publisher`、テスト用 STATIC ライブラリ（main は `RABBITS_DESCRIPTION_EXCLUDE_MAIN` で除外。rabbits_interface と同じ型）、`install(DIRECTORY launch urdf meshes rviz config ...)`。package.xml に `rclcpp`, `sensor_msgs`, `rabcl` の `depend`、`ament_cmake_gtest` の `test_depend`。
ノード: 4 脚分の `LegPose` を `Forward(0)` で初期化。`JointStateCB`: 出力 = 入力のコピー。`msg->name` に `leg_joints[i]` があれば `position` を読んで `Forward`、true なら更新。派生 8 個を name / position に足し（velocity / effort は入力に同じ長さの配列があるときだけ 0 を足す）、stamp は msg と同じで配信。

- [ ] **Step 5: テストが通ることを確認**

Run: `colcon test --packages-select rabbits_description` → GREEN、lint 差分なし。

- [ ] **Step 6: URDF を実寸に**

`gamma.xacro`: `<xacro:property name="L" value="${xacro.load_yaml('$(find rabbits_description)/config/gamma_linkage.yaml')['/**']['ros__parameters']['linkage']}" />` で読み、`leg_radius = L['leg_radius']` など。首の `mount_z = L['yaw_origin_z']`、`yaw_height`（Yaw → Pitch）= 0.1264、`pitch_offset` = 0（Pitch と Roll は同じ点）。胴体の箱は仮のまま（中心 z = −0.02、0.24 × 0.24 × 0.08 にして脚モータ面を跨ぐ）。
`leg.xacro` を設計書 7.1 の構成に書き直す（マクロ引数は `prefix azimuth L`）。各リンクの visual は箱 / 円筒で、クランク（長さ Lc、原点で +x から α0 上向き）、連結リンク（長さ Lr、K から J 向き）、上下バー（長さ Lb）、ホルダ（H1 から J と W を含む箱）、車輪（半径 / 幅は YAML）。関節 limit: `leg_joint` 0〜theta_max、`coupler_joint` / `bar_joint` は ±π（派生）。
`neck.xacro`: `pitch_offset` 0 でも roll_joint の origin が重なるだけで問題ないことを確認。

- [ ] **Step 7: xacro と check_urdf**

Run（Docker、`source install/setup.bash` 後）:
```
xacro src/rabbits_description/urdf/gamma/gamma.xacro > /tmp/g.urdf && check_urdf /tmp/g.urdf | head -3 && grep -c '<joint' /tmp/g.urdf && grep -c '<mimic' /tmp/g.urdf
```
Expected: `Successfully Parsed XML`、joint 数 = 首 3 + 脚 4 × 6 + camera + imu = 29、mimic 8。

- [ ] **Step 8: README（gamma の節に派生関節とミキサー、`motor_joint_states` → `joint_states` の流れ、YAML の約束。「寸法は仮」を「脚は実寸、首と胴体は仮」に）、display.launch.py は gamma のとき `joint_state_publisher_gui` の出力を `motor_joint_states` にリマップしてミキサーを挟む。コミット（承認後）**

`gamma の脚を 4 節リンクの実寸にし、派生関節を出すノードを追加`

---

### Task 4: rabbits_sim — 新しい `BodyPosture` と寸法 YAML

**Files:**
- Modify: `rabbits_ws/src/rabbits_sim/include/rabbits_sim/gamma_model.hpp`, `src/gamma_model.cpp`, `src/gamma_fake_node.cpp`, `config/gamma.yaml`, `test/test_gamma_model.cpp`, `test/test_gamma_fake_node.cpp`（あれば）, `README.md`

**Interfaces:**
- Consumes: Task 2 の `BodyPostureConfig` / `LegDamper::Update(…, load_torque)`、Task 3 の YAML キー（`linkage.*` を ROS パラメータとして読む）
- Produces: `GammaConfig` から `leg_angle_min / leg_angle_max / leg_tilt_gain` を削除し `rabcl::BodyPostureConfig posture` を追加（`leg_azimuth` は `posture.leg_azimuth` に移動）。`gamma_fake_node` は `linkage.*`（`crank_length` … `tilt_radius`、`bar_pivot` などは 2 要素配列）を `Require` で読む。`config/gamma.yaml` から `leg.angle_min / angle_max / tilt_gain` を削除、`wheel.wheel_d 0.075`、`wheel.body_d 0.34`。

- [ ] **Step 1: 失敗するテストに書き換える（test_gamma_model.cpp）**

Fixture の `GammaConfig` に `posture`（linkage は Task 1 の値、drop 0.05756 / 0.10377、tilt_radius 0.170）。期待値の変更:
- `StartsAtLowestHeightAndZeroNeck`: 脚の初期角 0（変更なし）。
- `BodyHeightRampsToBodyPostureTarget`: height 1 を送って十分進めたら全脚 θ = 1.42 ± 1e-3、height 0 で 0.15。
- `RollPositiveLowersRightLegs`: height 0.5, roll 0.1 → 右脚 2 本の θ < 左脚 2 本（`posture.linkage()` で drop に戻して 0.068644 / 0.092686 ± 1e-4）。
- `PitchIsCameraFrameWhenYawIs90deg`: 既存の意味のまま drop で比較。
- 追加 `StartsAtLowestHeightAndRampsToBodyPostureTarget`（Review Focus 4）: 起動直後に BODY_POSE 無しで 0.1 s 進めても θ = 0 のまま。height 0 を送ると 0.15 へ `ramp_rate` で進む（0.1 s で 0.05 ± 1e-3）。

- [ ] **Step 2: 失敗を確認**

Run: `colcon build --packages-select rabbits_sim 2>&1 | grep -E "error|Summary"`
Expected: `GammaConfig` に `leg_angle_min` が無い等のコンパイルエラー。

- [ ] **Step 3: 実装**

`GammaModel`: `posture_` を `config.posture` から構築。`LegDamper::Update(target, before, 0, dt, 0.0f)`（偽ファームは荷重 0）。`gamma_fake_node::LoadConfig`: `linkage.*` を読んで `c.posture` を組む（配列は `std::vector<double>` で受けて 2 要素を確認）。`leg.azimuth` は `c.posture.leg_azimuth` へ。

- [ ] **Step 4: テストが通ることを確認**

Run: `colcon test --packages-select rabbits_sim` → GREEN、lint 差分なし。

- [ ] **Step 5: `config/gamma.yaml` と README（挙動の節に「脚は 4 節リンクの運動学で車高 → θ」、寸法は rabbits_description の YAML を launch で渡す）を更新し、コミット（承認後）**

`偽ファームの脚を 4 節リンクの運動学にする`

---

### Task 5: rabbits_bringup — YAML を偽ファームに渡し、派生関節ノードを起動

**Files:**
- Modify: `rabbits_ws/src/rabbits_bringup/launch/gamma_sim.launch.py`, `launch/gamma.launch.py`, `README.md`

**Interfaces:**
- Consumes: Task 3 の `config/gamma_linkage.yaml` と `linkage_state_publisher`、Task 4 の `gamma_fake_node` のパラメータ要件

- [ ] **Step 1: launch を直す**

両 launch: `linkage_config = os.path.join(get_package_share_directory('rabbits_description'), 'config', 'gamma_linkage.yaml')`。`gamma_sim.launch.py` の `fake_node` の `parameters=[sim_config, linkage_config, {...}]`。`robot_bridge` の Node に `remappings=[('joint_states', 'motor_joint_states')]`。両方に `Node(package='rabbits_description', executable='linkage_state_publisher', name='linkage_state_publisher', parameters=[linkage_config])` を `robot_state_publisher` の隣に（実機 launch では `rviz` 条件に合わせ、`rviz:=false` のときはリマップもしない = bridge が `joint_states` を直接出す）。

- [ ] **Step 2: lint と画面なしの通し確認**

Run: `colcon test --packages-select rabbits_bringup` → GREEN。
Run（Docker、`--privileged -v /dev:/dev`）: `ros2 launch rabbits_bringup gamma_sim.launch.py rviz:=false input:=none` を 6 s 動かし、`ros2 topic pub -1 /reference_command rabbits_msgs/msg/ReferenceCommand "{body_height: 1.0}"` 後 3 s で `ros2 topic echo --once /joint_states` を 2 回（偽ファーム発と派生ノード発が混ざるので名前で読む）。
Expected: `/joint_states` の発行元が `linkage_state_publisher` だけ（`ros2 topic info -v /joint_states`）、1 メッセージに 19 関節、`right_front_leg_joint` が 1.42 に向かって増え、`right_front_bar_joint` が負に、`right_front_coupler_joint` が 2.4 付近から減る。

- [ ] **Step 3: README の launch 一覧に派生関節ノードを 1 行、コミット（承認後）**

`脚リンクの寸法 YAML を偽ファームに渡し、派生関節ノードを起動する`

---

### Task 6: gamma_turret_fw — 寸法定数と `BodyPosture` / `LegDamper` の呼び出し

**Files:**
- Modify: `fw_ws/gamma_turret_fw/src/robot.hpp`, `src/control.cpp`, `src/control.hpp`, `README.md`, `docs/superpowers/specs/2026-10-23-gamma-control-design.md`（4.5 の `LEG_TILT_GAIN` の記述）
- Modify: `fw_ws/gamma_turret_fw/components/rabcl`（Task 2 のコミットに進める）

**Interfaces:**
- Consumes: Task 2 の API

- [ ] **Step 1: `robot.hpp`**

`LEG_ANGLE_MIN / MAX / TILT_GAIN / LOAD_FF` を削除。追加: `constexpr rabcl::LegLinkageConfig LEG_LINKAGE = {0.041f, PI_F / 3.0f, 0.03975f, 0.060f, -0.026f, -0.049f, 0.0f, 0.0175f, 0.044f, -0.0325f, PI_F / 2.0f};`、`LEG_DROP_MIN 0.05756f`、`LEG_DROP_MAX 0.10377f`、`LEG_TILT_RADIUS 0.170f`、`LEG_LOAD_FORCE 0.0f`（[N]、組み上がってから）、`WHEEL_D 0.075`、`BODY_D 0.34`。コメントに「rabbits_description の config/gamma_linkage.yaml と対で更新する」。

- [ ] **Step 2: `control.cpp`**

`LEG_CONFIG` から `LEG_LOAD_FF` を外す。`posture_` の構築を `rabcl::BodyPostureConfig{robot::LEG_LINKAGE, robot::LEG_DROP_MIN, robot::LEG_DROP_MAX, robot::LEG_TILT_RADIUS, {...azimuth}}` に。`leg.damper.Update(target, pos, vel, dt, robot::LEG_LOAD_FORCE * posture_.linkage().DropJacobian(pos))`。

- [ ] **Step 3: ビルド**

Run（PowerShell、submodule を Task 2 のコミットに checkout してから）: `pio run`
Expected: SUCCESS、警告なし。

- [ ] **Step 4: README の定数表（脚の行）と設計書 4.5 を更新。コードはコミットしない（実機確認後）。文書だけ承認後にコミット**

`脚リンクの寸法と車高基準の姿勢制御を記録`

---

### Task 7: rviz での確認（ユーザーと一緒に）

- [ ] Dev Container で `ros2 launch rabbits_bringup gamma_sim.launch.py gamepad:=onefun_ps4` を起動し、□（しゃがみ）で 4 本の脚のクランク・連結リンク・バー・ホルダが閉ループを保ったまま動き、連結リンクの先端がホルダ上端ピンに重なって見えること。
- [ ] 背面ボタンのリーンで左右の車輪の下がりに差が出ること。CAD の 3 姿勢（原点でバーが約 25° 上向き、上限でクランクが −30°）と見比べる。
- [ ] 結果を台帳に書き、TODO.md の段 4 と記憶を更新する。
