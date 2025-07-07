# CentroidalManager::update 関数の詳細解説

本ドキュメントでは、`CentroidalManager::update()` 関数の全体構造と各ステップについて、制御理論・数式・実装観点から詳細に解説する。以下の処理は、ロボットが歩行時に安定性を保ちつつ、滑らかに目標軌道を追従するために不可欠な制御機構群を示している。

---

## 1. MPC状態の設定

```cpp
if(config().useActualStateForMpc)
{
  mpcCom_ = actualCom();
  mpcComVel_ = ctl().realRobot().comVelocity();
}
else
{
  mpcCom_ = ctl().comTask_->com();
  mpcComVel_ = ctl().comTask_->refVel();
}
```

この部分では、MPC（Model Predictive Control）で使用する初期状態を設定している。

- 実ロボットの状態を用いる場合（センサ計測を信頼する場合）:
  - `mpcCom_` は現在の重心位置（CoM）
  - `mpcComVel_` は現在のCoM速度
- そうでない場合は、前の制御ステップでのタスク目標値を使用する。

---

## 2. ZMPの基準軌道を取得

```cpp
refZmp_ = ctl().footManager_->calcRefZmp(ctl().t());
```

- 時刻 `t` における基準ZMPを計算。
- 主に歩行パターン生成器（FootManager）が提供する軌道を元にする。

---

## 3. MPC計算の実行

```cpp
runMpc();
```

- 重心軌道や力制御の予測制御計算を実行。
- MPCは、ロボットの未来の状態を予測して最適な制御入力を計算する。

---

## 4. センサからのZMPの測定

```cpp
for(const auto & foot : ctl().footManager_->getCurrentContactFeet())
{
  const auto & surfaceName = ctl().footManager_->surfaceName(foot);
  const auto & sensorName = ctl().robot().indirectSurfaceForceSensor(surfaceName).name();
  const auto & sensor = ctl().robot().forceSensor(sensorName);
  const auto & sensorWrench = sensor.worldWrenchWithoutGravity(ctl().robot());
  sensorWrenchList.emplace(foot, sensorWrench);
}
measuredZMP_ = calcZmp(sensorWrenchList, refZmp_.z());
```

---

### ZMPの定義式（2次元）

ロボットの足裏接地面をXY平面とした場合、ZMPのX・Y座標は以下で与えられる：

$$
\begin{aligned}
p_x &= -\frac{\tau_y}{f_z} \\
p_y &= \frac{\tau_x}{f_z}
\end{aligned}
$$

ここで：
- $( \tau_x, \tau_y )$：CoMまわりのモーメント
- $ f_z $：鉛直方向の支持力

---

### 実装における流れと対応

| ステップ | 処理内容 | 説明 |
|--------|--------|------|
| ① | `getCurrentContactFeet()` | 接地している足のみを取得 |
| ② | `indirectSurfaceForceSensor()` | 足裏表面に対応する力センサ名を取得 |
| ③ | `worldWrenchWithoutGravity()` | 重力を除いた外力とモーメントを取得 |
| ④ | `sensorWrenchList.emplace()` | 各足のセンサ値をマップに格納 |
| ⑤ | `calcZmp(sensorWrenchList, refZmp_.z())` | 力・モーメントからZMPのXY座標を算出 |


## 5. ZMP制御補正とDCM補償

```cpp
controlZmp_ = plannedZmp_;
Eigen::Vector3d refZmpVel = ctl().footManager_->calcRefZmp(ctl().t(), 1);
controlZmp_.head<2>() += config().zmpVelGain * refZmpVel.head<2>();
```

### ZMP遅れ補償

この部分では、ZMP遅れに対する予測補償を行うことで、ZMP制御の応答性と安定性を向上させている。

```cpp
Eigen::Vector3d refZmpVel = ctl().footManager_->calcRefZmp(ctl().t(), 1);
controlZmp_.head<2>() += config().zmpVelGain * refZmpVel.head<2>();
```

---

#### 数式的背景

この処理は、以下のようなZMPの予測補償則に基づいている：

$$
p_d = p_{pg} + T_p \cdot \dot{p}_{pg}
$$

- $ p_d $：補償後のZMP指令値
- $ p_{pg} $：プランナーが出力した基準ZMP
- $ \dot{p}_{pg} $：ZMPの時間微分（速度）
- $ T_p $：ZMP予測補償ゲイン（`zmpVelGain`）

この補償は、ZMP指令の応答遅れ（モータや力制御のラグ）に対して、先読みすることで制御性能を改善する目的で導入されている。

---

#### 実装との対応

| 項目 | 内容 |
|------|------|
| `calcRefZmp(t, 1)` | ZMP基準軌道の1階微分（$ \dot{p}_{pg} $）を取得 |
| `config().zmpVelGain` | ゲイン $ T_p $：遅れを見越した補償量を決定 |
| `controlZmp_.head<2>() += ...` | XY平面におけるZMP指令を補正する |

---

### DCM補償の詳細

ロボットの歩行時におけるDCM（Divergent Component of Motion）は、システムの不安定成分を明示的に扱うことで、ZMP制御における安定性を高めるために導入される。

---

#### DCMの定義

DCMは、LIPM（Linear Inverted Pendulum Model）において次のように定義される：

$$
\xi = x + \frac{\dot{x}}{\omega}
$$

- $ \xi $：DCMベクトル（発散成分の位置）
- $ x $：重心（CoM）の位置
- $ \dot{x} $：重心速度
- $ \omega $：自然周波数（下記参照）

---

#### 自然周波数 $ \omega $ の導出

```cpp
double omega = std::sqrt(plannedForceZ_ / (robotMass_ * (mpcCom_.z() - refZmp_.z())));
```

DCMの自然周波数は、重力加速度と支持基底面からの高さに依存し、以下の式に基づく：

$$
\omega = \sqrt{\frac{F_z}{m (z_{\text{com}} - z_{\text{zmp}})}}
$$

- $ F_z $：鉛直方向の支持力
- $ m $：ロボットの質量
- $ z_{\text{com}} $：重心の高さ
- $ z_{\text{zmp}} $：ZMPの高さ

---

#### DCM誤差と補償項の導出

```cpp
Eigen::Vector3d plannedDcm = ctl().comTask_->com() + ctl().comTask_->refVel() / omega;
Eigen::Vector3d actualDcm = actualCom() + ctl().realRobot().comVelocity() / omega;
Eigen::Vector3d dcmError = actualDcm - plannedDcm;
```

このコードでは以下の計算を行っている：

- `plannedDcm`: MPCによる計画DCM（理想状態）
- `actualDcm`: 実際のセンサ状態からのDCM
- `dcmError`: その差分（誤差）

$$
\Delta p = K_\xi (\xi_{\text{actual}} - \xi_{\text{planned}})
$$

ここで：

- $ \Delta p $：ZMP指令に加算する補償項
- $ K_\xi $：DCMフィードバックゲイン（`config().dcmGainP`）

---

#### ZMP指令への反映

```cpp
controlZmp_.head<2>() += config().dcmGainP * dcmError.head<2>();
```

DCM誤差に比例した補償量 $ \Delta p $ を、XY方向のZMP指令に加算することで、歩行中の安定性を改善する。

## 6. 垂直方向の力制御（PD制御）

```cpp
double plannedComZ = ctl().comTask_->com().z();
double actualComZ = actualCom().z();
double plannedComVelZ = ctl().comTask_->refVel().z();
double actualComVelZ = ctl().realRobot().comVelocity().z();
controlForceZ_ -= config().comZGainP * (actualComZ - plannedComZ)
                + config().comZGainD * (actualComVelZ - plannedComVelZ);
```

ロボットの重心（CoM）の高さ $z$ とその速度 $\dot{z}$ に目標との差異がある場合に、それを修正するために鉛直方向の力（=重心加速度に相当）を調整する。

$$
F_z \leftarrow F_z - K_p (z_{\text{actual}} - z_{\text{ref}}) - K_d (\dot{z}_{\text{actual}} - \dot{z}_{\text{ref}})
$$

- $F_z$：Z方向の制御力（controlForceZ_）
- $z_actual$：現在の重心高さ（actualCom().z()）
- $z_ref$：目標の重心高さ（comTask_->com().z()）
- $\dot{z}_{actual}$：現在の重心速度（realRobot().comVelocity().z()）
- $\dot{z}_{ref}$：目標の重心速度（comTask_->refVel().z()）
- $K_p, K_d$：PD制御ゲイン（comZGainP, comZGainD）

| 項目      | 内容                                                                                                                     |
| ------- | ---------------------------------------------------------------------------------------------------------------------- |
| 機能      | Z方向の重心（高さと速度）の誤差を補正するための PD 制御                                                                                         |
| 数式      | $F_z \leftarrow F_z - K_p (z_{\text{actual}} - z_{\text{ref}}) - K_d (\dot{z}_{\text{actual}} - \dot{z}_{\text{ref}})$ |
| 使用される変数 | `controlForceZ_`, `comZGainP`, `comZGainD`                                                                             |
| 効果      | ロボットの上下のブレや振動を抑え、安定した垂直姿勢を維持する                                                                                         |


---

## 7. レンチ変換と分配

```cpp
Eigen::Vector3d comForWrenchDist = (config().useActualComForWrenchDist ? actualComUnbiased() : ctl().comTask_->com());
sva::ForceVecd controlWrench;
controlWrench.force() << controlForceZ_ / (comForWrenchDist.z() - refZmp_.z()) * (comForWrenchDist.head<2>() - controlZmp_.head<2>()),
                         controlForceZ_;
```

ロボットの歩行安定化制御における ZMP（Zero Moment Point）→ 総合レンチ（力・モーメント）への変換と、足裏への力の分配 を行う部分である。これは、ZMP制御で計算された制御指令を、実際の複数の接触点（足裏など）に現実的に適用可能な形へと変換するための処理である。

### 水平力の導出

- ZMPの定義から、水平方向の力 $ f_{xy} $ は以下で導出される：
$$
\mathbf{f}_{xy} = \frac{F_z}{z_{\text{CoM}} - z_{\text{ZMP}}} \cdot (\mathbf{x}_{\text{CoM}} - \mathbf{p}_{\text{ZMP}})
$$

$$
\mathbf{f} = \begin{bmatrix}
f_x \\
f_y \\
F_z
\end{bmatrix}
$$

- $F_z$ : 鉛直方向の総合制御力（controlForceZ_）
- $\mathbf{x}_{\text{CoM}}$ : 重心位置（comForWrenchDist.head<2>()）
- $\mathbf{p}_{\text{ZMP}}$ : 重心位置（comForWrenchDist.head<2>()）

これは、ZMPの定義式を逆に解いて、必要な水平力 $f_x, f_y$ を導出するものである。

ZMP定義式からの導出：
$$
\mathbf{p}_{\text{ZMP}} = \mathbf{x}_{\text{CoM}} - \frac{z_{\text{CoM}} - z_{\text{ZMP}}}{F_z} \cdot \mathbf{f}_{xy}
$$
$$
\mathbf{f}_{xy} = \frac{F_z}{z_{\text{CoM}} - z_{\text{ZMP}}} \cdot (\mathbf{x}_{\text{CoM}} - \mathbf{p}_{\text{ZMP}})
$$

#### モーメント（回転力）はゼロに設定
```cpp
controlWrench.moment().setZero(); // Moment is represented around CoM
```
この controlWrench は CoM 周りのレンチとして表現されており、モーメント成分を 0 にしているのは、足裏に分配する際に個別に求め直すため。
また、ZMP制御自体がモーメントゼロの仮定（LIPM）に基づいているため、ここではモーメントを持たない。

```cpp
contactList_ = ctl().footManager_->calcCurrentContactList();
wrenchDist_ = std::make_shared<ForceColl::WrenchDistribution>(
    ForceColl::getContactVecFromMap(contactList_), config().wrenchDistConfig);
wrenchDist_->run(controlWrench, comForWrenchDist);
```
ここでは、複数の接触点（例：右足・左足）に対して、このレンチを物理的に可能な形で分配している：
- WrenchDistribution は静力学的解に基づいて、
  - 各接触点が出せる力の制限（摩擦錐、圧力分布、トルク限界など）
  - 接触点の位置や数
  - ロボットの構造

などを考慮して、各接触点ごとの力とモーメントの最適な分配を行っている。この処理で、物理的に実現可能な形で力制御が実行される。

---

## 8. CoMとFoot Taskの目標設定

```cpp
Eigen::Vector3d plannedComAccel = calcPlannedComAccel();
Eigen::Vector3d nextPlannedCom = mpcCom_ + dt * mpcComVel_ + 0.5 * dt^2 * plannedComAccel;
Eigen::Vector3d nextPlannedComVel = mpcComVel_ + dt * plannedComAccel;
```
$$
\begin{aligned}
\mathbf{x}_{\text{next}} &= \mathbf{x}_{\text{now}} + \Delta t \cdot \dot{\mathbf{x}}_{\text{now}} + \frac{1}{2} \Delta t^2 \cdot \ddot{\mathbf{x}} \\
\dot{\mathbf{x}}_{\text{next}} &= \dot{\mathbf{x}}_{\text{now}} + \Delta t \cdot \ddot{\mathbf{x}}
\end{aligned}
$$

- CoMの目標位置・速度・加速度を次の制御ステップ分だけオイラー積分で予測。
- この値を `comTask_` に与えることで、次の制御サイクルに向けた運動指令を生成。

```cpp
const auto & targetWrenchList = ForceColl::calcWrenchList(contactList_, wrenchDist_->resultWrenchRatio_);
for(const auto & foot : Feet::Both)
{
  sva::ForceVecd targetWrench = sva::ForceVecd::Zero();
  if(targetWrenchList.count(foot))
  {
    targetWrench = targetWrenchList.at(foot);
  }
  ctl().footTasks_.at(foot)->targetWrenchW(targetWrench);
}
```

- `WrenchDistribution` で得た分配比に従い、各接触点に与えるべき力（レンチ）を計算。
- これを足裏タスクに反映することで、目標ZMPを再現できるよう接触力を制御。

---

## 9. サポート領域の計算

```cpp
supportRegion_[0] = supportRegion_[0].cwiseMin(vertexWithRidge.vertex.head<2>());
supportRegion_[1] = supportRegion_[1].cwiseMax(vertexWithRidge.vertex.head<2>());
```

- 各接触点の凸包を求め、支持多角形のXY軸最小・最大範囲を決定。
- `supportRegion_` によって、ZMPが安定領域内にあるかを確認可能。

---

## 10. 可視化情報の更新

```cpp
ctl().gui()->removeCategory(...);
wrenchDist_->addToGUI(...);
```

- GUI表示用に、最新のレンチ分配状態を更新。
- デバッグ・ログ・モニタリングのために視覚情報として表示される。