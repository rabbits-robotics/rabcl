# gamma の脚リンク機構の運動学と実寸 URDF 設計書（2026-10-25）

## 1. 背景と目的

gamma の脚（DM4310 × 4）は単純な腕ではなく、「モータ軸 → クランク → 連結リンク → 車輪ホルダ」+「ホルダを胴体と平行に保つ 2 本のバー」の 4 節リンクである（2026-10-25 に CAD で確定）。今の rabcl `BodyPosture` は車高 0〜1 をモータ角に線形に割り、傾きを `tilt_gain` 定数で脚角の差にしているので、本物では車高が指令どおりにならず、傾きも正しく出ない。URDF も仮寸法の 1 本腕で、rviz で見える車高が本物と違う。

目的:
- モータ角 ↔ 車輪の高さの関係を幾何どおりにし、ファームと偽ファームが同じ式で車高・ロール・ピッチを出す。
- URDF を実寸にし、クランク・連結リンク・バー・ホルダを別リンクとして rviz に出す（メッシュは後で差し替える）。
- 寸法の置き場を 2 か所（ファーム `robot.hpp` と rabbits_description の YAML）に絞る。

非目標: 首の実寸（Yaw 原点の高さ以外は仮のまま）、メッシュ、脚の動力学（慣性・摩擦の同定）、`OmniDrive` の車輪配置径を車高に追従させること（定数のまま。6 章）。

## 2. 機構と記号

脚 1 本の面内 2 次元。原点 M = 脚モータ軸、x = 半径方向の外向き（水平）、z = 上。右手系で y = z × x（回転軸の向き）。4 本とも同じ形で、方位角だけ違う（FR −π/4、FL π/4、BR −3π/4、BL 3π/4）。

| 記号 | 意味 | 値（mm / deg） |
|---|---|---|
| r | base 原点から M までの水平距離 | 92 |
| B1, B2 | 上 / 下のバーの胴体側ピン（M から） | (−26, −49), (−26, −69) |
| Lc | クランクの長さ（M → 先端ピン K） | 41 |
| α0 | ゼロ姿勢（モータ原点）でのクランクの向き（+x から、上向き正） | 60 |
| θ | モータ角。0 = 原点（車高が最も低い）、増えると車体が上がる。クランクの向きは α = α0 − θ | 0〜90（90 はメカスト。使用範囲は両端 0.15 rad の余裕を取り 0.15〜1.42 rad） |
| Lr | 連結リンクの長さ（K → ホルダ上端ピン J） | 39.75 |
| Lb | バーの長さ（B1 → H1 = B2 → H2、2 本同じ） | 60 |
| φ | バーの角度（B1 → H1 の向き、+x から上向き正）。ホルダは胴体と平行を保つ | 原点で +27.6、上限で −27.7 |
| j | H1 → J（ホルダ上のベクトル） | (0, 17.5) |
| w | H1 → 車輪中心 W（ホルダ上のベクトル。**上のバーのホルダ側ピン H1 から**測った値。バーは両側にあるので脚の中心面で見れば車輪は中心、面に垂直な y 方向のずれは 0。holder.STL の車輪軸で確認） | (38.3, −44) |
| 車輪 | 直径 / 幅 | φ75 / 13 |
| base 原点 | 4 本の脚モータ軸を含む水平面と首 Yaw 軸の交点。x 前、y 左、z 上 | |
| 首 Yaw 原点 | base 原点から z | −25.5 |
| 首 Pitch / Roll 原点 | Yaw 原点から z（同じ点。仮） | +126.4 |

バーの胴体側ピンがモータ軸より内側（x 負）かつ下にある。上から見ると車輪は脚の中心線上、バーは左右対称に 2 組（片側 2 本、計 4 本）で、面内の幾何は 1 組で表す。

## 3. 運動学

### 3.1 順運動学（θ → φ、ψ、W）

K = Lc (cos α, sin α)、α = α0 − θ。C = B1 + j とおくと J = C + Lb (cos φ, sin φ)。拘束 |J − K| = Lr。

C を中心とする半径 Lb の円と、K を中心とする半径 Lr の円の交点を求め、φ を得る。交点は 2 つあり、**ホルダが胴体より下にある枝（φ が小さい方 = J の z が低い方）** を常に選ぶ。もう一方はホルダが M より上に来る物理的にあり得ない枝。

- 円が交わらない（|K − C| > Lb + Lr または < |Lb − Lr|）ときは可動範囲外。実装は最も近い点に投影せず、`false` を返し、呼び出し側が直前の値を保つ。
- 連結リンクの向き ψ = atan2(J − K)。URDF の連結リンク関節（クランク先端 K にぶら下がる）の角度は ψ − α。
- 車輪中心 W = B1 + Lb (cos φ, sin φ) + w。車輪の下がり d = −W.z（M からの深さ、正）。接地面から M までの高さは d + 車輪半径。

数値の裏取り（2 章の値、w は H1 基準）:

| θ | φ | W.x | d（車輪の下がり） | 接地面 → M（+37.5） | ψ − α（連結リンク関節） |
|---|---|---|---|---|---|
| 0° | +27.62° | 65.5 | 65.2 | 102.7 | −140.35° |
| 0.15 rad（使用下限） | +23.52° | 67.3 | 69.1 | 106.6 | −136.44° |
| 30° | +11.88° | 71.0 | 80.7 | 118.2 | −124.03° |
| 60° | −7.22° | 71.8 | 100.5 | 138.0 | −100.84° |
| 1.42 rad（使用上限） | −21.79° | 68.0 | 115.3 | 152.8 | −80.94° |
| 90°（メカスト） | −27.67° | 65.4 | 120.9 | 158.4 | −72.15° |

車高の変化幅は 0〜90° で 55.7 mm、使用範囲で 46.2 mm。（2026-10-25 に車輪の位置を (44, −32.5) → (38.3, −44) に訂正。holder.STL と CAD の再計測による）dd/dθ は 24（原点）〜38（中間）mm/rad。CAD の 3 姿勢の絵と整合（原点でバーが約 25° 上向き、上限でクランクが −30° 付近）。

### 3.2 逆運動学（d → θ）

d から sin φ = (B1.z + w.z + (−d) ... ) を解く: W.z = B1.z + Lb sin φ + w.z = −d なので sin φ = (−d − B1.z − w.z) / Lb。|sin φ| > 1 なら範囲外。cos φ は正（バーは外向き）。J = C + Lb (cos φ, sin φ) が決まり、M を中心とする半径 Lc の円と J を中心とする半径 Lr の円の交点が K。2 つの交点のうち **α が可動範囲 [α0 − θmax, α0] に入る方** を選ぶ（両方入るなら α0 に近い方）。θ = α0 − α。交わらなければ範囲外。

### 3.3 ヤコビアン（重力トルクのため）

（dd/dθ > 0 なので、荷重トルクは常に「車体を上げる向き」に正。）

dd/dθ を数値微分（中心差分、Δθ = 1e-3 rad）で出す。脚の荷重 F [N]（車体質量の分担）のモータ軸回りトルクは τ = F · dd/dθ [Nm]（d が増える向き = 車体が上がる向きに正）。閉じた式は要らない（制御周期 100 Hz × 4 脚で十分軽い）。

### 3.4 単位

rabcl の API は m と rad。設定の数値は 2 章の mm / deg を m / rad に直して渡す。

## 4. rabcl

### 4.1 `LegLinkage`（新規、`controller/leg_linkage.hpp`）

```cpp
struct LegLinkageConfig
{
  float crank_length;      // Lc [m]
  float crank_angle_zero;  // α0 [rad]
  float coupler_length;    // Lr [m]
  float bar_length;        // Lb [m]
  float bar_pivot_x, bar_pivot_z;      // B1（M から）[m]
  float coupler_pin_x, coupler_pin_z;  // j = H1 → J [m]
  float wheel_x, wheel_z;              // w = H1 → W [m]
  float theta_max;                     // モータ角の上限 [rad]（0 が下限）
};

struct LegPose
{
  float bar_angle;      // φ [rad]
  float coupler_angle;  // ψ − α [rad]（クランク先端に付く連結リンク関節の角度）
  float wheel_x;        // W.x [m]（M から外向き）
  float wheel_drop;     // d = −W.z [m]
};

class LegLinkage
{
public:
  explicit LegLinkage(const LegLinkageConfig & config);
  bool Forward(float theta, LegPose & out) const;   // 範囲外（円が交わらない）なら false、out は変えない
  bool Inverse(float wheel_drop, float & theta) const;  // 同上
  float DropJacobian(float theta) const;            // dd/dθ [m/rad]、中心差分
  float drop_min() const;  // Forward(0) の d
  float drop_max() const;  // Forward(theta_max) の d
};
```

`drop_min / drop_max` は構築時に計算して保持する（Forward が false を返す設定なら 0）。

### 4.2 `BodyPosture`（変更）

```cpp
struct BodyPostureConfig
{
  LegLinkageConfig linkage;
  float drop_min, drop_max;  // 車高 0 / 1 に対応する車輪の下がり [m]（θ = 0.15 / 1.42 rad の d。5 章の YAML / robot.hpp に数値で持つ）
  float tilt_radius;         // 車体中心から車輪の接地点までの水平距離 [m] = r + W.x（中間の車高の値 0.164。車高で ±2% 変わるのは無視）
  float leg_azimuth[4];      // 変更なし
};
// Solve(height, roll, pitch, yaw_offset, out[4]) の意味は同じ。中身:
//   d0 = drop_min + h (drop_max − drop_min)
//   脚 i: d_i = d0 + tilt_radius (sin(az_i − yaw_offset) · roll − cos(az_i − yaw_offset) · pitch)（小角度）
//   d_i を [drop_min, drop_max] にクランプし、Inverse で θ_i（クランプ後なので必ず解ける）
```

`tilt_gain` と `angle_min / angle_max` は消える。符号の約束は変えない（ロール正 = 右が下がる = 右脚の d が小さい、ピッチ正 = 前が下がる = 前脚の d が小さい）。傾きの腕の長さは脚の取り付け半径 r ではなく接地点の半径 r + W.x を使う（r のままだと指令した傾きの約 57% しか出ない）。

### 4.3 `LegDamper`（変更）

`LegDamperConfig.load_ff` [Nm] を消し、`Update(target, angle, vel, dt, load_torque)` に引数で受ける（呼び出し側が `F · DropJacobian(angle)` で作る）。`friction_ff` の扱いは変えない。既存のテストは `load_torque = 0` に読み替える。

### 4.4 テスト（gtest、`test/test_leg_linkage.cpp` + 既存の body_posture / leg_damper）

- 2 章の値で Forward(0 / 30 / 60 / 90°) が 3.1 の数値（φ、d）と ±0.1 mm / ±0.1° で合う。
- Inverse(Forward(θ).d) = θ（往復、±1e-4 rad）を範囲全体で。
- 範囲外（d が drop_min 未満 / drop_max 超）で false、出力が変わらない。
- 枝の選択: Forward の φ が常に原点の +27.6° 以下（上の枝 82.9° を選ばない）。
- DropJacobian が中心差分の定義と一致、原点付近で正。
- BodyPosture: height 0 / 1 が drop_min / drop_max の θ（0.15 / 1.42 rad）、roll 0.1 rad で右脚と左脚の d の差が tilt_radius × 0.1 × 2 sin(π/4) になる（Forward で d を戻して確かめる）、pitch 正で前脚が小さい、yaw_offset = π/2 で roll が前後の差になる（既存テストの読み替え）、クランプで範囲内に収まる。
- LegDamper: load_torque が tff にそのまま足される。

## 5. 寸法の置き場（2 か所）

- **ファーム** `gamma_turret_fw/src/robot.hpp`: `LEG_LINKAGE`（`LegLinkageConfig` 定数）、`LEG_DROP_MIN / MAX`、`LEG_RADIUS`、`LEG_LOAD_FORCE` [N]。`LEG_ANGLE_MIN / MAX / TILT_GAIN / LOAD_FF` は消す。
- **rabbits_description** `config/gamma_linkage.yaml`: 同じ値を m / rad で。先頭を `/**:` にして、どのノード名でも読み込める ROS 2 パラメータファイルにする。
  - URDF の xacro は `${xacro.load_yaml(...)}` で同じファイルから寸法を取る。
  - rabbits_sim の `gamma_fake_node` は launch（bringup）でこのファイルを `parameters` に足して読む。`config/gamma.yaml` の `leg.angle_min / angle_max / tilt_gain` は消し、`leg.azimuth / ramp_rate` は残す。
  - 派生関節ノード（7 章）も同じファイルを読む。
- ファームと YAML の両方に「対で更新する」と注記。

YAML のキー（`linkage.` 以下）: `crank_length`, `crank_angle_zero`, `coupler_length`, `bar_length`, `bar_pivot: [x, z]`, `bar_pivot_lower: [x, z]`（B2、URDF 用）, `coupler_pin: [x, z]`, `wheel: [x, z]`, `theta_max`（1.5708、メカスト。URDF の limit）, `drop_min`, `drop_max`（0.0691 / 0.1153 = θ 0.15 / 1.42 rad）, `leg_radius`（0.092）, `tilt_radius`（0.164）, `wheel_radius`（0.0375）, `wheel_width`（0.013）, `yaw_origin_z`（−0.0255）。

## 6. 他の定数への影響

- `BODY_D`（`OmniDrive` の車輪配置径）: 車輪中心の半径は r + W.x で、車高によって 157〜164 mm と変わる。本設計では中間の **0.328 m**（= 2 × tilt_radius）を定数にする（最大 2% の誤差。追従は非目標）。`WHEEL_D` は **0.075 m** に直す。偽ファームの `wheel.body_d / wheel_d` も同じ値に。
- `LEG_RAMP_RATE` は θ の速度のまま。

## 7. URDF と派生関節

### 7.1 リンク構成（脚 1 本、`leg.xacro` を書き直す）

```
base_link
 ├─ <p>_leg_joint      revolute（M、軸 y、0〜theta_max。joint_states の脚モータ角 θ）→ <p>_crank_link（クランク。visual は α0 の向きに描く）
 │    └─ <p>_coupler_joint  revolute（K、軸 y、派生: ψ − α）→ <p>_coupler_link（連結リンク、長さ Lr。先端 J は閉じない）
 ├─ <p>_bar_joint      revolute（B1、軸 y、派生: φ）→ <p>_bar_link（上のバー、長さ Lb）
 │    └─ <p>_holder_joint   revolute（H1、軸 y、mimic bar × −1）→ <p>_holder_link（ホルダ。J、W を含む）
 │         └─ <p>_wheel_joint  continuous（W、車輪の z = 半径方向外向き）→ <p>_wheel
 └─ <p>_bar_lower_joint revolute（B2、軸 y、mimic bar × 1）→ <p>_bar_lower_link（下のバー）
```

**リンクのフレーム（2026-10-25 変更、メッシュの座標系と一致させる）**: 各リンクは「z = その関節の回転軸、x = ゼロ姿勢でのリンクの長さ方向」。回転軸は脚の面の y_left（= z_up × x_out）で、z 軸は右ねじで回すとリンクの先が下がる = 車体が上がる向き。胴体に付く関節の origin は rpy = (−π/2, −a, azimuth)（a = ゼロ姿勢でのリンクの向き、+x から上向き正: クランク α0、バー φ0 = 0.4821）。連結リンクはクランクの子で origin rpy = (0, 0, α0 − ψ0)（ψ0 = −1.4024 = ゼロ姿勢での K → J の向き）、ホルダはバーの子で rpy = (0, 0, φ0)（x = 水平外向き、y = 下）、車輪はホルダの子で origin xyz = (wheel.x, −wheel.z, 0)、rpy = (−π/2, 0, −π/2)（z = 回転軸 = 外向き、x = 上）。φ0 と ψ0 は YAML に `bar_angle_zero` / `coupler_dir_zero` として持つ。

関節の値はすべて「ゼロ姿勢からの変化分、正 = 車体が上がる向き」: `leg_joint` = θ、`bar_joint` = −(φ − φ0)、`coupler_joint` = −((ψ − α) − (ψ0 − α0))。ゼロ姿勢では全関節が 0。mimic は `holder_joint` = −1 × bar、`bar_lower_joint` = +1 × bar。visual はどのリンクも自分の +x に沿った円筒（メッシュに差し替えるときは origin 0）。

### 7.2 派生関節ノード `linkage_state_publisher`（rabbits_description、C++）

- ミキサー。購読: `motor_joint_states`（bridge の `joint_states` を launch でリマップしたもの。モータ 11 関節）。配信: `joint_states`（モータ 11 関節をそのまま写し、派生 8 関節 `<p>_bar_joint` / `<p>_coupler_joint` を足した 1 本）。`/joint_states` の発行元はこのノードだけにする（1 トピック 1 発行元）。
- パラメータ: `linkage.*`（5 章の YAML）、`leg_joints`（4 個の脚モータ関節名）、`prefixes`（4 個、`right_front` など）。
- 受けたメッセージの name / position / velocity / effort をそのまま写し、脚モータ関節が含まれていれば各脚で `LegLinkage::Forward(θ)` を解いて派生の姿勢を更新する。派生 8 関節（`bar_joint` = −(φ − φ0)、`coupler_joint` = −(coupler_angle − coupler_angle0)、φ0 / coupler_angle0 は Forward(0)。7.1 の符号。velocity / effort は 0）は脚の有無によらず毎回足して、同じ stamp で配信する。Forward が false の脚は直前の値を使う（起動直後は Forward(0)）。
- rabbits_description は rclcpp と rabcl に依存する（C++17 に上げる）。テストは gtest で、既知の θ で配信される値が 3.1 の数値の符号反転になること。
- bringup の `gamma_sim.launch.py` / `gamma.launch.py` で `robot_state_publisher` と一緒に起動し、`robot_bridge` の `joint_states` を `motor_joint_states` にリマップする。ノードが落ちると `robot_state_publisher` に何も届かなくなる（表示が止まる）が、1 トピック 1 発行元を優先する。

### 7.3 rviz

床は描かないので、車高が上がると車輪が下に伸びて見える（今までどおり）。

## 8. 変えないもの

- プロトコル（`BODY_POSE`、`JOINT_STATE`）と bridge。
- 首の yaw の扱い（`BodyAttitude::SetYawTracking`）。
- 脚モータ角の符号（原点 0、車体が上がる向きが正）。DM4310 の実際の回転方向との対応は実機で確認し、必要ならファームの符号で吸収する（今までどおり）。

## 9. 確認の手順

1. rabcl のテスト（Docker）。
2. 偽ファーム + rviz: `body_height` 0 → 1 で車輪が 46 mm 下がる（接地面 → M が 106.6 → 152.8 mm に相当）、ロールで左右の差、ピッチで前後の差。rviz でクランク・連結リンク・バーの閉ループが破綻せず（連結リンクの先端が J に重なって見える）。
3. ファームはビルドのみ。実機は TODO.md 段 2 で。
