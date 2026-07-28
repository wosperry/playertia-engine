# 《超真实机械指针交互系统设计文档 v2》

> ESP32-S3 实时物理仿真开发指导稿  
> 文档性质：物理模型审计、数学依据与 C/C++ 实现规范  
> 适用硬件：ESP32-S3-Touch-AMOLED-1.8、QMI8658 六轴 IMU、触摸屏  
> 前版文档：[ESP32-S3 物理指针手表核心交互与数学模型 v1](./esp32-s3-physical-watch-interaction-design.md)

本文使用以下修订标记：

- **[保留 v1]**：v1 的设计思想和模型仍然成立；
- **[修正 v2]**：v1 的表达不够严格，或需要增加适用边界；
- **[新增 v2]**：为直接指导固件开发而增加的新模型、新接口或新测试。

---

## 0. [新增 v2] v2 修订说明

### 0.1 本次审查结论

v1 的核心思想是正确的：

1. 世界坐标系是物理真值层；
2. 表盘是移动、旋转的非惯性参考系；
3. 时针、分针、秒针是共享同一移动铰点、但具有独立状态和质量属性的刚体；
4. 指针响应重力、基座惯性、阻尼、触摸和时间目标力矩；
5. 正确时间应作为目标约束，不能在物理模式中直接覆盖指针角度。

v2 对以下容易误用的部分作出正式修正：

1. `gravityVectorBody` 来自姿态四元数，只表示世界重力在表盘坐标系中的三轴分量；
2. 加速度计在动力学链中提供的是比力。表心有效场可直接由 `-specificForceAtPivot`
   得到；
3. `gravityVectorBody` 与 `-specificForceAtPivot` 不能作为两份外力同时相加，否则会
   重复计算重力；
4. `F_inertia = -mA_O` 只描述表心平移产生的惯性力。任意三维转腕还需考虑欧拉项和
   离心广义力矩；
5. “材质惯性”必须落实为密度、几何、质心、质量一阶矩和转动惯量。材质名称本身不是
   一项物理力；
6. 指针在世界中具有惯性，但仍受到移动铰点和旋转表盘平面的约束。它不是自由悬浮的
   三维陀螺；
7. 六轴 IMU 无法长期恢复可信的世界绝对位置，本引擎也不需要该位置；
8. v1 中 `p = I(θ̇ + Ω_z)` 更准确的名称是“消去基座平移后的约化广义动量”，而不是
   未化简世界系拉格朗日量的完整共轭动量。

### 0.2 模型分层

所有实现和配置必须标明属于哪一层：

| 层级 | 含义 | 示例 |
|---|---|---|
| 严格物理层 | 从刚体、移动铰点和传感器测量推导 | 重力、比力、惯量、阻尼、三维离心项 |
| 工程近似层 | 为实时性采用、且有明确适用条件 | 细杆模型、忽略很小的 IMU 偏心、100 Hz 刚体步 |
| 艺术增强层 | 为手感主动偏离真实物理 | 虚拟重力倍率、独立方向偏置、夸张柔性、额外拖尾 |

艺术增强不是错误，但不得伪装成真实材料参数，也不得污染严格物理层的单位和调试数据。

### 0.3 推荐阅读和实现路径

首版刚性三针应按以下顺序实现：

1. 第 4 章：统一坐标、角度、四元数和单位；
2. 第 6 章：得到可靠的 `gravityVectorBody` 和校准后比力；
3. 第 7、9 章：实现表心有效场和单针主方程；
4. 第 10、12 章：实现时间回归和触摸交接；
5. 第 15～18 章：固定步长、数据结构、实时任务和调试；
6. 第 19 章：通过合成输入与实机录制验收后，再实现柔性指针。

---

## 1. [修正 v2] 文档目的与模块边界

### 1.1 目的

本文只定义“仿真解压表盘”的物理数学依据与实时软件架构。目标是让开发者可以据此
实现一个独立的 `PhysicalDialEngine`：

```text
IMU / 触摸 / 连续时间
          ↓
PhysicalDialEngine
          ↓
三根指针的渲染角度、形变与调试遥测
```

该引擎应能产生以下核心体验：

- 正常状态下，三根指针准确跟随连续时间目标；
- 进入物理交互后，设备静止竖放时指针受重力趋向六点；
- 缓慢旋转表盘时，指针按照重力、惯量和阻尼连续滞后、摆动；
- 故意甩动表盘时，移动铰点的加速度通过各针质心产生不同力矩；
- 触摸某根指针时，触摸对该针拥有最高控制优先级，但仍可表现有限抓持、重量和弯曲；
- 停止有意操作后，三根指针分别回到持续前进的正确时间目标。

### 1.2 输入

引擎只接收带时间戳的、与系统其余部分解耦的输入：

```cpp
struct DialInputs {
    ImuSample imu;
    TouchFrame touch;
    TimeTargets timeTargets[3];
    DialCommand command;
    uint64_t timestampUs;
};
```

主要输入为：

- QMI8658 三轴加速度计和三轴陀螺仪；
- 触摸点、触摸速度、接触状态；
- 时针、分针、秒针的连续目标角和目标角速度；
- 进入物理模式、立即回归、切换调试等离散事件。

### 1.3 输出

```cpp
struct DialOutputs {
    HandPose hand[3];
    DialPhysicsState state;
    PhysicsTelemetry telemetry;
};
```

至少输出：

- 每根指针的未包裹角度 `theta`；
- 相对表盘角速度 `omegaRelative`；
- 可选的柔性关节角；
- 用于渲染插值的前后两个物理快照；
- 调试模式所需的输入、力矩、能量和状态机数据。

### 1.4 非目标

本文不设计：

- 菜单、应用切换和多指手势产品逻辑；
- Wi-Fi、蓝牙、OTA、文件系统和资源管理；
- RTC 芯片驱动、NTP、时区和日历业务；
- 电池、充电、休眠和整机电源管理；
- 表盘皮肤、图标、声音和普通 UI 框架。

这些模块只能通过输入接口向物理引擎提供时间或命令，不得反向侵入动力学求解器。

---

## 2. [修正 v2] 核心体验、实现范围与物理假设

### 2.1 [保留 v1] 核心体验

正常显示时，三根指针具有各自持续前进的 `TIME_REF_i(t)`。进入物理交互时，解除时间
目标的主导控制权，指针开始响应设备运动和触摸；这不代表 RTC 或正确时间停止。

每根指针独立保存：

- 角度、角速度和约化动量；
- 质量、质心、质量一阶矩和转动惯量；
- 轴承阻尼、摩擦和速度上限；
- 时间回归参数；
- 触摸捕获状态；
- 可选柔性状态。

三根指针共享：

- 同一表盘中心；
- 同一表壳姿态、角速度和表心有效加速度；
- 同一物理时间步；
- 同一帧传感器质量标志。

### 2.2 基本物理假设

首版刚性模型采用以下假设：

1. 世界重力在设备活动范围内均匀且恒定；
2. 表盘和指针的共同铰点位于表盘中心 `O`；
3. 每根指针只有一个相对表盘的转动自由度 `θ_i`；
4. 指针始终被约束在随表壳运动的表盘平面内；
5. 指针之间首版不发生实体碰撞；
6. 指针相对转轴的质量属性不随时间变化；
7. 表壳运动是由用户规定的基座运动，指针对表壳的反作用不反馈到手腕和 IMU；
8. 空气阻力、轴承摩擦和主动控制以广义力矩表示。

### 2.3 “世界系是真实参考”的准确含义

世界系用于定义真实的重力、表心运动和约束。对指针中心线上质量元 `s`：

```text
x_i^W(s,t) =
    x_O^W(t)
  + R_WB(t) s u_i^B(θ_i)
```

这就是“铰点被表壳带着在三维世界运动，而指针保留自身惯性”的严格表达。

但固件不应把 `x_i^W` 当作必须直接积分的状态。用 `R_WB + θ_i` 作为最小坐标，会自动
满足定长和表盘平面约束；它与世界系约束模型等价，计算更稳定，也更适合 ESP32-S3。

### 2.4 不能把指针理解为自由三维陀螺

当表壳俯仰或翻滚时，表盘平面法向也在变化。真实机械指针被轴和表盘约束，必然受到
平面约束反力。因此：

- 只绕屏幕法向转动时，低摩擦指针可近似保持世界朝向；
- 表盘平面本身改变时，指针不可能无条件冻结为一个世界三维向量；
- 将世界向量强行投影回屏幕会造成长度、方向或能量不连续；
- 本项目模拟的是移动平面内的机械指针，不是中心球铰上的自由陀螺。

---

## 3. [修正 v2] 物理审查结论与数学基础

### 3.1 正确、需修正和禁止的模型

| 命题 | 审查结果 | 实现结论 |
|---|---|---|
| 世界坐标系是真实物理参考 | 正确 | 用世界系推导，用表盘最小坐标求解 |
| 每根指针是独立刚体 | 正确 | 独立状态和质量属性，共享基座输入 |
| 陀螺仪直接计算重力加速度 | 错误 | 陀螺仪提供角速度；重力方向由姿态融合得到 |
| 只需知道表盘倾斜角 | 不完整 | 需要重力在表盘系中的三轴向量 |
| 甩动就是给指针一个经验力 | 错误 | 甩动先规定基座加速度，再由质心产生广义力矩 |
| `F_inertia=-mA_O` 是全部非惯性项 | 不完整 | 还可能有欧拉、离心和科里奥利项 |
| 材质直接改变重力 | 错误 | 重力场不变；材料和几何改变质量分布、柔性和耗散 |
| 时间回归直接写角度 | 错误 | 物理模式中必须通过力矩、约束或独立的动画状态 |
| 需要双积分恢复手表世界位置 | 错误 | 相对动力学只需表心有效加速度、姿态和角速度 |

### 3.2 所需理论

| 理论 | 用途 |
|---|---|
| 向量、矩阵和叉积 | 坐标转换、质心力矩 |
| 四元数和 `SO(3)` | 无奇异地累计表壳三维姿态 |
| 姿态观测器 | 融合陀螺仪短时响应与加速度计长期方向参考 |
| 拉格朗日力学 | 从世界系移动铰点模型消去约束 |
| 刚体动力学 | `Iα=Στ`、质量分布和转动惯量 |
| 非惯性参考系 | 平移、欧拉、离心和科里奥利项 |
| 势能与虚功 | 从 `U` 推导重力和时间目标力矩 |
| 控制理论 | 时间目标、触摸约束和混合捕获 |
| 数值分析 | 固定步长、稳定性、限幅和子步进 |
| 数字信号处理 | 校准、滤波、门控、时间对齐和饱和处理 |

### 3.3 物理量单位

物理内核只使用 SI 单位：

| 量 | 单位 |
|---|---|
| 长度、质心距离 | `m` |
| 质量 | `kg` |
| 时间 | `s` |
| 角度 | `rad` |
| 角速度 | `rad/s` |
| 加速度 | `m/s²` |
| 转动惯量 | `kg·m²` |
| 力矩 | `N·m` |
| 线性阻尼 | `N·m·s/rad` |
| 扭转刚度 | `N·m/rad` |
| 能量 | `J` |

驱动层若输出 `g` 或 `°/s`，必须在进入物理引擎前完成换算。

---

## 4. [修正 v2] 术语、坐标系、四元数与旋转约定

### 4.1 [新增 v2] 世界坐标系 `W`

定义本地、右手、近似惯性的世界坐标系 `W`：

- 原点可以在启动时任意选取；
- `+z_W` 竖直向上；
- `x_W/y_W` 的水平朝向不影响本项目；
- 世界重力固定为：

```text
gravityVectorWorld = g^W = (0, 0, -g0)
g0 = 9.80665 m/s²
```

本设备没有磁力计，世界水平面的绝对航向不可长期观测。该航向自由度不会改变
`gravityVectorBody`，因此不影响本项目的重力指针效果。

### 4.2 [新增 v2] 表盘坐标系 `B`

定义固定在表壳上的右手坐标系 `B`，原点位于表盘中心 `O`：

- `+x_B`：屏幕向右；
- `+y_B`：屏幕向下；
- `+z_B`：指向屏幕内部；
- 因而 `x_B × y_B = z_B`。

物理引擎、触摸坐标和渲染角度统一使用 `B` 系。

### 4.3 [新增 v2] IMU 坐标系 `M`

`M` 是 QMI8658 芯片原始坐标系。芯片丝印方向、PCB 方向和屏幕方向不得假定一致。

定义固定旋转矩阵：

```text
C_BM : M 系向量 → B 系向量
```

校准后的加速度和角速度为：

```text
f_S^B =
    unitScaleA × C_BM × M_a × (a_raw^M - b_a)

Ω^B =
    unitScaleG × C_BM × M_g × (ω_raw^M - b_g)
```

其中 `M_a/M_g` 是比例与交叉轴修正矩阵，`b_a/b_g` 是零偏。首版至少必须完成六面
静态加速度测试和三轴正向旋转测试。

### 4.4 [新增 v2] 姿态矩阵和四元数唯一约定

全文只使用以下约定：

```text
R_WB : B 系向量 → W 系向量
R_BW = R_WBᵀ : W 系向量 → B 系向量
q_WB : 与 R_WB 相同，表示 B 相对 W 的姿态
```

同一物理向量的表达转换为：

```text
v^W = R_WB v^B
v^B = R_BW v^W
```

四元数采用 Hamilton 乘法、标量在前：

```text
q = (w, x, y, z)
```

向量写成纯四元数 `[0,v]`，则：

```text
[0,v^W] = q_WB ⊗ [0,v^B] ⊗ q_WB*
[0,v^B] = q_WB* ⊗ [0,v^W] ⊗ q_WB
```

如果所用库定义的是 `q_BW`、标量在后或左乘角速度，必须在适配层转换；不得在引擎
内部混用两套约定。

### 4.5 [新增 v2] 重力向量转换

姿态融合输出单位四元数 `q_WB` 后：

```text
gravityVectorBody
    = g^B
    = R_BW g^W
    = vector(
        q_WB* ⊗ [0,g^W] ⊗ q_WB
      )
```

这就是指针重力模型所需的三轴重力向量：

```text
gravityVectorBody = (g_x, g_y, g_z)
```

系统不需要先求“表盘相对水平面的倾斜角”，也不应把单个倾角作为物理输入。真正需要
知道的是：

> 世界重力在当前表盘坐标系中的 `x/y/z` 三轴分量。

屏幕内静态重力只使用：

```text
g_parallel = (g_x, g_y, 0)
|g_parallel| = sqrt(g_x² + g_y²)
```

`g_z` 虽然不直接产生理想平面刚性针的中心轴力矩，但用于姿态质量、平放退化和调试。

### 4.6 [保留 v1] 钟表角度约定

指针角 `θ` 从十二点开始、顺时针为正：

```text
十二点 = 0
三点   = π/2
六点   = π
九点   = 3π/2
```

指针轴向单位向量和正角切向量：

```text
u(θ) = (sin θ, -cos θ, 0)
t(θ) = ∂u/∂θ = (cos θ, sin θ, 0)
```

对任意表盘系向量 `a=(a_x,a_y,a_z)`：

```text
[l_c u(θ) × m a]_z
    = m l_c t(θ)ᵀ a
    = m l_c (a_x cosθ + a_y sinθ)
```

该向量式应作为代码的基础，方向角只用于调试显示。

### 4.7 周期角与未包裹状态

内部 `theta` 保存未包裹角度，允许多圈运动。只有显示或计算最短目标差时才包裹：

```cpp
inline float shortestAngle(float from, float to)
{
    return atan2f(sinf(to - from), cosf(to - from));
}
```

---

## 5. [修正 v2] 仿真状态机与输入仲裁

### 5.1 全局表盘状态

```text
TIME_TRACKING
    │ 进入物理交互
    ▼
PHYSICS_PLAY
    │ 有意操作超时
    ▼
AUTO_RETURN
    │ 所有指针进入捕获容差
    ▼
TIME_TRACKING
```

这些状态只属于表盘引擎，不包含整机休眠、电源或应用状态。

### 5.2 单指针状态

```text
TIME_CONTROLLED
IMU_FREE
TOUCH_CONSTRAINED
RETURNING
```

触摸只改变被命中指针的单针状态，不应暂停另外两根针的动力学。

`settled` 是由角度、速度和持续时间共同计算的完成标志，不是停止积分的第五种模式。
当三根针的 `settled` 都成立时，全局进入 `TIME_TRACKING`，并将三根针明确切换为
`TIME_CONTROLLED`；不能让某根针停留在一个含义不明、不会继续走时的 `SETTLED` 状态。

### 5.3 力矩优先级

“优先级”不等于同时计算多套角度后选择一个。统一做法是控制各力矩提供器的启用和
权重：

1. **触摸约束**：最高；被捕获时暂停该针的时间回归，可保留有限的重力与材料形变；
2. **安全约束**：限速、限能量、异常恢复；
3. **时间回归控制**：只在 `AUTO_RETURN/TIME_TRACKING` 生效；
4. **基座物理**：重力、平移惯性、旋转基座项；
5. **艺术增强**：最后加入，并在遥测中单独列项。

禁止同一物理步既写入 `theta`，又对相同目标施加控制力矩。

“触摸优先于 IMU”表示触摸成为该针的主要约束、时间回归暂停，并不表示停止记录
`Ω/a_eff`。表盘平面仍在真实运动，约化动量关系和释放交接仍需要这些输入。若产品选择
触摸期间弱化基座力矩，也应只平滑调整其权重，同时继续更新基座状态，避免松手时跳变。

### 5.4 有意操作与无操作

无操作计时器只由以下事件重置：

- 用户有效触摸、拖动或释放；
- 明确的实体按键/命令；
- 超过阈值并满足持续时间的有意甩动或旋转手势。

以下行为不重置：

- IMU 静止噪声；
- 指针自身继续摆动；
- 时间目标持续前进；
- 渲染帧和普通传感器样本到达。

---

## 6. [修正 v2] IMU 前端、姿态融合与重力方向估计

### 6.1 两条并行信号链

必须并行维护、但不得重复叠加两条链：

```text
姿态链：
加速度计 + 陀螺仪
    → Mahony / Madgwick / EKF
    → q_WB
    → gravityVectorBody

动力学链：
原始加速度计
    → 零偏/比例/轴向校准
    → 传感器比力 f_S^B
    → IMU 偏心修正
    → 表心有效场 a_eff,O^B = -f_O^B
```

姿态链回答“世界重力在表盘坐标中指向哪里”；动力学链保留用户甩动造成的快速基座
激励。

两条链必须共享同一份最终角速度：

```text
Ω_final^B =
    unitScaleG C_BM M_g
    (ω_raw^M - b_g,static - b_g,online)
```

其中 `b_g,online` 可由 Mahony 的低动态积分估计。姿态传播、离心项、约化动量关系和
IMU 偏心修正都使用 `Ω_final^B`；不得让姿态链使用在线零偏，而动力学链仍使用另一份
未修正陀螺数据。

### 6.2 [新增 v2] 姿态融合算法选择

| 算法 | 优点 | 代价/风险 | 本项目建议 |
|---|---|---|---|
| Mahony | 结构简单、可估计陀螺零偏、易门控 | 需正确调 `Kp/Ki` | 首版推荐 |
| Madgwick | 轻量、成熟、快速收敛 | `β` 与动态门控需实测 | 可直接采用 |
| 乘法 EKF/误差状态 EKF | 可表达协方差、偏置和更多传感器 | 状态、矩阵和调参复杂 | 首版不需要 |

三者对 ESP32-S3 的运算量都不是瓶颈。选择应优先考虑可校准、可观测和代码可靠性，
而不是追求最复杂的算法。

### 6.3 四元数传播

`q_WB` 将 `B` 映射到 `W`，陀螺仪角速度在 `B` 系表达，因此连续传播为：

```text
q̇_WB = 0.5 q_WB ⊗ [0, Ω^B]
```

离散更新可写为：

```text
δ = |Ω^B| Δt
û = Ω^B / |Ω^B|

δq =
    [
        cos(δ/2),
        û sin(δ/2)
    ]

q_pred = normalize(q_WB ⊗ δq)
```

当 `|Ω|` 很小时使用：

```text
δq ≈ [1, 0.5 Ω^B Δt]
```

若代码使用 `ExpQuaternion(v)`，本文将其明确定义为
`[cos|v|, normalize(v)sin|v|]`，此时才可传入 `v=0.5ΩΔt`。有些库把输入解释成完整
旋转向量并在内部再次除以二；直接照搬会产生半角错误，必须通过已知 `90°` 旋转测试。

融合算法再用加速度计方向残差进行小量修正。每次更新后必须归一化四元数，并检测
`NaN`、零范数和异常时间步。

启动时先等待一个满足低角速度、`|f|≈g0` 且无饱和的可靠窗口，用平均比力初始化
roll/pitch；没有磁力计时 yaw 可任取 `0`。初始化完成前将
`attitudeConfidence` 置低，不输出“可信纯重力方向”。动力学链仍可使用经过校准的
`-f_O`，但应保持安全限能量和启动平滑。

### 6.4 加速度计不是任意时刻的纯重力计

理想加速度计测量传感器位置 `S` 的比力：

```text
f_S^B = R_BW (A_S^W - g^W)
```

静止时 `A_S=0`，所以：

```text
f_S^B = -gravityVectorBody
```

因此姿态观测器在低动态窗口中应比较：

```text
measuredUpBody  = normalize(f_S^B)
predictedUpBody = normalize(-gravityVectorBody)
```

而不是把 `normalize(f_S)` 与向下的 `gravityVectorBody` 直接对齐。这个符号必须通过
六面测试确认，因为不同驱动可能对原始寄存器做过额外取反。

快速甩动时 `A_S` 不为零，加速度方向不能直接当作重力方向。姿态融合必须根据以下信息
降低加速度校正权重：

- `abs(|f_S|-g0)`；
- 加加速度 `|df/dt|`；
- 当前角速度、偏心预期向心加速度；
- 传感器饱和；
- 样本丢失和时间戳异常；
- 动作后的恢复迟滞。

门控只降低该样本对姿态校正的权重，不能把它从动力学链删除；甩动本身正是需要保留的
物理输入。

Mahony 的陀螺零偏积分也只应在姿态校正置信度足够高的低动态窗口更新；快速线性运动
时冻结或缓慢衰减该积分，避免把平移加速度误学习为陀螺零偏。

仅靠六轴 IMU，不可能在任意持续线性加速下完美区分“倾斜后的重力”与“平移加速度”。
门控只能降低误差，不能创造缺失的外部观测。这也是动力学链优先使用 `-f_O` 的原因：
即使姿态链短时把纯重力方向估偏，指针所需的合成有效场仍由比力直接给出。

### 6.5 `gravityVectorBody` 输出契约

```cpp
struct AttitudeOutput {
    Quatf qWB;
    Vec3f gravityVectorBody;   // m/s²
    Vec3f omegaBody;           // rad/s
    float accelCorrectionWeight;
    float attitudeConfidence; // 0..1
    uint64_t timestampUs;
    uint32_t flags;
};
```

必须保证：

- `qWB` 为单位四元数；
- `gravityVectorBody = rotateInverse(qWB, {0,0,-g0})`；
- `|gravityVectorBody|` 理论上恒等于 `g0`；
- 输出含时间戳和质量标志；
- 六轴模式的 yaw 漂移不应改变该重力向量；
- 物理引擎不得从欧拉倾角重新构造重力。

### 6.6 平放退化

当：

```text
sqrt(g_x² + g_y²) → 0
```

静态重力对理想平面指针的力矩自然趋近零。实现应直接使用向量计算力矩，而不是依赖
噪声主导的 `atan2` 方向。

界面若需要显示“重力方向”，可冻结最后可靠方向并使用迟滞；这只是显示策略，不得删除
动力学链中沿屏幕平面的真实甩动分量。

### 6.7 时间戳、滤波和延迟

- IMU 数据必须使用采样时间而不是任务读取时间；
- 优先使用 FIFO 或数据就绪中断获得稳定节奏；
- 姿态、比力和角速度必须在同一时间基准；
- 物理步对相邻 IMU 样本做时间插值或零阶保持；
- 任何低通都应记录群延迟；
- 偏心修正中的角加速度和比力必须时间对齐；
- 达到量程的样本标为饱和，不能假装峰值仍可信。

---

## 7. [修正 v2] 移动表盘、表心运动与甩动模型

### 7.1 世界系移动铰点模型

表壳首先在世界中运动，指针根部由铰点强制跟随表盘中心 `O`。用户“甩表”不是直接给
每根针一个经验力或相同角速度，而是规定：

- 表心平移加速度 `A_O^W`；
- 表壳角速度 `Ω^B`；
- 表壳角加速度 `α^B`；
- 随时间变化的表盘平面约束。

每根针再根据自身质量分布将这些共同输入转换成不同的广义力矩。

### 7.2 表心有效场

表盘系中的世界重力：

```text
g^B = R_BW g^W = gravityVectorBody
```

表心世界平移加速度转换到表盘系：

```text
A_O^B = R_BW A_O^W
```

指针在随表壳平移的参考系中感受到的合成有效场：

```text
a_eff,O^B =
    g^B - A_O^B
    = R_BW (g^W - A_O^W)
```

它满足：

- 静止：`A_O=0`，`a_eff=g^B`；
- 向右加速：额外出现向左的惯性场；
- 自由落体：`A_O≈g`，`a_eff≈0`；
- 任意平移甩动：响应 `g-A_O`，不是 `g+A_O`。

### 7.3 显式重力和惯性力拆分

对质心位置 `r_c^B(θ)`：

```text
F_gravity^B = m g^B
F_inertia,translation^B = -m A_O^B

τ_gravity =
    [r_c^B × F_gravity^B]_z

τ_inertia,translation =
    [r_c^B × F_inertia,translation^B]_z
```

所以：

```text
τ_gravity + τ_inertia,translation
    = [r_c^B × m(g^B - A_O^B)]_z
    = [r_c^B × m a_eff,O^B]_z
```

用户提出的：

```text
Iα =
    τgravity
  + τinertia
  + τcontrol
  + τdamping
```

对“只平移、不显著旋转的基座”是正确骨架；对任意三维转腕还不完整，第 9 章会增加
欧拉和离心广义力矩。

### 7.4 [修正 v2] 由加速度计直接得到有效场

若加速度计已经换算到表心 `O`：

```text
f_O^B = A_O^B - g^B
```

因此：

```text
a_eff,O^B = -f_O^B
```

这条式子不需要先分离重力和线性加速度，特别适合实时动力学。

如需调试时显式拆分：

```text
A_O^B = f_O^B + gravityVectorBody
A_O^W = R_WB A_O^B
```

则必须满足一致性检查：

```text
τ_gravity + τ_inertia,translation
≈ [r_c × m(-f_O^B)]_z
```

禁止以下错误实现：

```text
错误：
τ = τ(gravityVectorBody) + τ(-f_O^B)
```

因为 `-f_O^B` 已包含重力与表心平移惯性的合成结果。

### 7.5 IMU 安装偏心修正

令 `r_OS^B` 为表心 `O` 指向 IMU 安装点 `S` 的固定向量。刚性表壳上：

```text
A_S^W =
    A_O^W
  + R_WB [
        α^B × r_OS^B
      + Ω^B × (Ω^B × r_OS^B)
    ]
```

所以：

```text
a_eff,O^B =
    -f_S^B
  + α^B × r_OS^B
  + Ω^B × (Ω^B × r_OS^B)
```

保守误差上界：

```text
|Δa| ≤ |α| |r_OS| + |Ω|² |r_OS|
```

若实测该值远小于动作噪声和所需手感，首版可令 `r_OS=0`；否则必须校正。`α` 应由
带限微分器或小型观测器估计，不得直接对两个原始陀螺样本作无滤波差分。

### 7.6 为什么不恢复绝对位置

六轴 IMU 不直接观测绝对位置和速度。很小的加速度零偏经两次积分也会快速变成巨大
位置漂移；持续线性加速与重力在六轴系统中也不能被任意准确地长期分离。

本引擎只需要：

```text
a_eff,O^B
Ω^B
必要时的 α^B
```

绝对位置 `x_O^W` 和匀速平移速度会在相对指针动力学中消去。因此，世界系应作为推导和
验证基准，而不是要求固件维护一个虚假的房间坐标轨迹。

### 7.7 连续输入与冲量

正式仿真逐物理步积分 `a_eff,O^B(t)`。对远短于指针固有周期的动作，可用冲量做测试：

```text
J_eff = ∫ a_eff,O^B(t) dt
```

若冲击期间 `θ` 近似不变：

```text
I Δω ≈ [r_c × m J_eff]_z - I ΔΩ_z
```

连续积分和冲量近似是同一物理过程的两种表达。检测到“甩动手势”后不得在已经积分
加速度的基础上再次施加同一冲量。

---

## 8. [保留 v1] 每根指针的独立时间基准

### 8.1 连续目标

令：

```text
s = second + fraction
m = minute + s/60
h = (hour mod 12) + m/60
```

则：

```text
θ_target,second = 2π s / 60
θ_target,minute = 2π m / 60
θ_target,hour   = 2π h / 12

ω_target,second = 2π / 60
ω_target,minute = 2π / 3600
ω_target,hour   = 2π / 43200
```

三根针必须分别保存目标角、目标角速度和控制参数。`TIME_REF` 始终在后台连续前进，
即使当前处于物理交互或触摸状态。

### 8.2 接口

```cpp
struct TimeTarget {
    float angleWrapped;
    float angularVelocity;
    uint64_t timestampUs;
    uint32_t discontinuityId;
};
```

物理引擎只消费连续目标。时区、校时或系统时间跳变由外部模块转换为新的
`discontinuityId`；引擎据此选择平滑追赶或显式重初始化，而不是在普通物理步中突然
覆盖角度。

每个物理子步必须在明确的求值时刻 `t_eval` 使用目标，而不是整段复用基础步开始时的
角度：

```text
θ_target(t_eval) =
    θ_target(timestamp)
  + ω_target (t_eval - timestamp)
```

通常取子步中点评估力矩、子步末端评估速度关系。触摸目标也必须按时间戳做零阶保持或
插值，并在数据过旧时降低置信度。

---

## 9. [修正 v2] 单根刚性指针的质量、势能与统一动力学

### 9.1 [新增 v2] 每根指针的最小质量属性

每根指针至少拥有：

```cpp
struct HandMassDefinition {
    float massKg;                 // m
    float visualLengthM;          // L，仅用于几何/渲染
    Vec3f centerOfMassAtZeroM;     // θ=0 时相对表心的质心
    float inertiaPivotKgM2;       // I = ∫r_perp² dm
};

struct HandMassDerived {
    Vec3f firstMomentAtZeroKgM;    // h_0 = m*r_c,0，由定义派生
    float collinearFirstMomentKgM; // H；仅细杆快速路径
};

struct HandDissipation {
    float viscousNmSecPerRad;     // c1
    float quadraticNmSec2PerRad2; // c2
    float coulombNm;              // τc
    float smoothVelocityRadSec;   // ε
};
```

`centerOfMass`、`length` 和 `momentOfInertia` 不是可互换参数：

- `length` 决定视觉几何和积分范围；
- `centerOfMass` 决定均匀力场的力臂；
- `momentOfInertia` 决定同一力矩产生的角加速度；
- `mass` 与质心共同给出质量一阶矩；
- `damping` 描述耗散，不属于质量属性。

所有质量属性必须来自同一质量分布，或至少通过一致性校验。
`firstMomentAtZero` 不作为可独立编辑的第四个输入；它必须在加载阶段由
`mass*centerOfMassAtZero` 派生。若文件格式为兼容旧版本同时保存两者，加载时必须断言：

```text
|h_0 - m r_c,0| < tolerance
```

不满足时拒绝该配置，不能静默选择其中一个。

### 9.2 质量分布

一般形式：

```text
m = ∫ dm
r_c = (1/m) ∫ r dm
I = ∫ |e_z × r|² dm

h_0 = ∫ r_0 dm = m r_c,0
h(θ) = R_z(θ) h_0
```

对中心线细杆，以有符号坐标 `s` 表示距转轴的位置：

```text
H = ∫ s dm = m l_c
I = ∫ s² dm
```

使用有符号 `H` 可自然描述尾部配重。若配重使 `H=0`，均匀重力和表心平移有效场不再
产生一阶力矩，但 `I` 仍然为正，指针仍响应表壳法向旋转、触摸和控制力矩。

一般二维形状或质心相对视觉轴存在侧向偏置时，应使用向量 `h(θ)`；`H u(θ)` 只是
`h(θ)` 与视觉轴共线时的快速路径。这样可以真实表达每根针不同的质心轴，而无需为三根
针虚构三套世界重力。

若第 `i` 根针的质量一阶矩相对视觉轴有真实几何偏角 `δ_com,i`：

```text
h_i(θ_i) =
    |h_i| u(θ_i + δ_com,i)
```

在静态有效场方向 `φ_eff` 下，其稳定视觉角约为：

```text
θ_stable,i =
    φ_eff - δ_com,i
```

因此，“每根指针有自己的重力基准”在严格物理层应解释为每根针有自己的质心轴和质量
分布，而不是世界里存在三套不同重力。若仍需额外方向偏置，必须放入艺术增强层。

### 9.3 [新增 v2] 世界重力势能

世界质心位置：

```text
x_c^W =
    x_O^W
  + R_WB r_c^B(θ)
```

取世界 `+z` 向上，重力势能：

```text
E_p = m g0 z_c^W
    = m g0 h_height
```

这里 `h_height` 是质心世界高度；不要与后文的质量一阶矩向量 `h(θ)` 混淆。

与坐标无关的向量形式：

```text
U_g =
    -m (g^W)ᵀ x_c^W
```

删除与 `θ` 无关的表心常数项后：

```text
U_g(θ,t) =
    -m (g^B(t))ᵀ r_c^B(θ)
```

用户改变表盘姿态时，本质上是改变 `g^B` 和指针质心相对世界重力场的高度：

```text
表盘姿态变化
    → gravityVectorBody 变化
    → 质心高度和势能变化
    → 势能梯度产生力矩
    → 指针运动
```

### 9.4 [新增 v2] 重力力矩

由叉积：

```text
F_g^B = m g^B
τ_g = [r_c^B(θ) × F_g^B]_z
```

或由势能：

```text
τ_g = -∂U_g/∂θ
```

两式必须数值一致。对细杆：

```text
r_c^B = l_c u(θ)
τ_g = H t(θ)ᵀ g^B
```

对一般平面质心：

```text
h(θ) = m r_c(θ)
∂h/∂θ = e_z × h(θ)

τ_g =
    [h(θ) × g^B]_z
  = [∂h(θ)/∂θ]ᵀ g^B
```

重力不是“让指针角度等于重力方向”的赋值，而是随当前角度连续变化的力矩。指针朝向
重力时力矩为零；偏离后产生恢复力矩；惯性使其越过最低势能点并摆动。

### 9.5 平移惯性力矩与合成有效势

```text
F_inertia,translation = -m A_O^B
τ_inertia,translation =
    [r_c × (-m A_O^B)]_z
```

对给定时刻的 `a_eff=g^B-A_O^B`，可定义瞬时伪势能：

```text
U_eff(θ,t) =
    -m a_eff,O^B(t)ᵀ r_c^B(θ)

τ_eff =
    -∂U_eff/∂θ
    = [∂h(θ)/∂θ]ᵀ a_eff,O^B
```

中心线细杆时再简化为：

```text
τ_eff = H t(θ)ᵀ a_eff,O^B
```

由于基座输入随时间变化，`U_eff` 是时变的；甩动可以向指针注入或抽取能量，因此不能
要求甩动期间机械能守恒。

### 9.6 [修正 v2] 旋转表盘的非惯性项

在旋转的表盘参考系中，质量元还会出现：

```text
dF_Euler       = -dm [α^B × r]
dF_centrifugal = -dm [Ω^B × (Ω^B × r)]
dF_Coriolis    = -2dm [Ω^B × v_relative]
```

对质量集中在表盘平面中心线上的一自由度细长指针，积分后得到：

```text
I θ̈ =
    H t(θ)ᵀ a_eff,O^B
  - I α_z
  - I (Ω^B·u(θ))(Ω^B·t(θ))
  + τ_relative
```

其中：

- `-Iα_z` 是表壳绕屏幕法向角加速度的欧拉惯性力矩；
- `-I(Ω·u)(Ω·t)` 是任意三维角速度产生的离心广义力矩；
- 理想一自由度刚性指针的科里奥利力对允许的切向虚位移不做功，因此其广义力矩为零；
- 若 `Ω_x≈Ω_y≈0`，离心广义力矩也为零；
- 若明显俯仰、横滚或甩腕，不能无条件删除离心项。

对有限宽度、质量位于表盘平面内的薄片指针，更一般的离心广义力矩为：

```text
τ_centrifugal =
    0.5 Ωᵀ [∂J_B(θ)/∂θ] Ω
```

`J_B(θ)` 是指针关于表心、在 `B` 系表达的三维惯量张量。首版使用细杆式即可，但应以
实机动作录制评估 `Ω_x/Ω_y` 是否足够大。

### 9.7 [修正 v2] 统一动力学方程

每根指针统一满足：

```text
I_i α_i = Στ_i
```

在显式角加速度形式中：

```text
I_i θ̈_i =
    τ_effectiveField,i
  + τ_Euler,i
  + τ_centrifugal,i
  + τ_drive,i
  + τ_timeControl,i
  + τ_touch,i
  + τ_elastic,i
  + τ_collision,i
  + τ_damping,i
```

其中：

```text
τ_effectiveField = H tᵀ a_eff,O^B
τ_Euler          = -I α_z
τ_centrifugal    = -I(Ω·u)(Ω·t)   // 细杆
```

力矩提供器的含义：

| 力矩 | 含义 |
|---|---|
| `τ_effectiveField` | 重力与表心平移惯性的合成 |
| `τ_Euler` | 表壳法向角加速度 |
| `τ_centrifugal` | 任意三维旋转基座效应 |
| `τ_drive` | 明确标记的主动艺术驱动或机械事件 |
| `τ_timeControl` | 正确时间目标的主动吸引 |
| `τ_touch` | 有限刚度、有限力矩的触摸约束 |
| `τ_elastic` | 指针内部柔性或独立扭簧 |
| `τ_collision` | 可选碰撞约束 |
| `τ_damping` | 轴承、介质和摩擦耗散 |

所有分项都必须以 `N·m` 表示，并在 Debug Mode 中独立可见。

### 9.8 [修正 v2] 推荐的约化广义动量形式

直接计算 `α_z=dΩ_z/dt` 会放大陀螺仪噪声。对当前薄片/细杆假设，定义消去基座平移后
的约化广义动量：

```text
p = I(θ̇ + Ω_z)
```

注意：若从尚未消去表心平移速度的世界系拉格朗日量直接求
`∂T/∂θ̇`，还会出现与表心速度有关的项。因此这里的 `p` 应称为“约化动量状态”或
“关于移动表心的法向角动量分量”，不要在理论说明中误称为未化简系统的全部正则动量。

更新方程：

```text
ṗ =
    H tᵀ a_eff,O^B
  + τ_centrifugal
  + τ_drive
  + τ_timeControl
  + τ_touch
  + τ_elastic
  + τ_collision
  + τ_damping

θ̇ = p/I - Ω_z
```

使用该形式后：

- 不再显式加入 `-Iα_z`；
- `p` 是权威速度状态，`θ̇` 由 `p/I-Ω_z` 派生；
- 不能另外独立积分一个长期存在的 `omegaRelative`；
- 如果动画或异常恢复直接设定了 `θ̇`，必须同步设置
  `p=I(θ̇+Ω_z)`；
- IMU 偏心修正仍可能需要带限 `α`，但它只用于把 `S` 点比力换算到 `O` 点，不能再
  重复进入主方程。

这是 ESP32-S3 首版的推荐实现。

### 9.9 阻尼和摩擦

推荐的被动阻力：

```text
τ_damping =
    -c1 θ̇
    -c2 |θ̇|θ̇
    -τ_c tanh(θ̇/ω_ε)
```

分别表示线性粘性阻尼、高速二次阻力和平滑库仑摩擦。必须满足：

```text
P_damping = τ_damping θ̇ ≤ 0
```

阻尼的参考速度必须与物理来源一致：

- 轴承阻尼：相对表壳，使用 `θ̇`；
- 时间捕获阻尼：相对时间目标，使用 `θ̇-ω_target`；
- 触摸阻尼：相对手指目标，使用 `θ̇-ω_touch`；
- 柔性内部阻尼：使用相邻段相对角速度。

把所有阻尼都写成相对重力方向的速度，会把真实轴承阻尼变成追踪控制器。

### 9.10 能量和功率

相对转动动能可用于工程调试：

```text
T_relative = 0.5 I θ̇²
```

重力势能、时间势能和柔性势能分别为：

```text
U_gravity = -m g^B·r_c(θ)
U_time    = K[1-cos(θ-θ_target)]
U_flex    = 0.5 k β²
```

每个力矩通道的瞬时功率：

```text
P_j = τ_j θ̇
```

表盘运动、触摸和时间控制器都能向系统做功。因此只有在“表盘固定、重力恒定、无触摸、
无主动控制、无碰撞注能”时，才应要求总机械能不持续增加。移动基座下能量增加可能是
真实的约束做功，不应误判为积分器发散。

---

## 10. [修正 v2] `TIME_REF` 与时间回归控制

### 10.1 时间是目标，不是角度写入

物理模式中的硬性规则：

```text
禁止：
theta = thetaTarget

允许：
thetaTarget → error → controlTorque → integrator → theta
```

`TIME_REF_i(t)` 是持续移动的目标信号，不拥有物理状态的写权限。

### 10.2 机械模式：周期势能或限幅 PD

定义：

```text
e_θ =
    atan2(
        sin(θ_target - θ),
        cos(θ_target - θ)
    )

e_ω = ω_target - θ̇
```

推荐周期控制器：

```text
τ_time =
    K_p sin(e_θ)
  + K_d e_ω
```

它等价于周期势能：

```text
U_time =
    K_p [1-cos(θ-θ_target)]
```

正弦势能在误差恰为 `π` 且相对速度为零时存在不稳定平衡点。工程上应根据进入前速度
选择确定方向、加入极小确定性偏置，或改用下面的限幅最短角 PD；不要依赖随机噪声决定
每次回归方向。

若产品必须始终选择最短方向，可使用限幅线性 PD：

```text
τ_time =
    clamp(
        K_p e_θ + K_d e_ω,
        -τ_time,max,
        +τ_time,max
    )
```

用户给出的 `Kp(error)+Kd(velocity)` 实际是 **PD**，不是完整 PID。一般不建议加入积分
项，否则会遇到周期角绕圈、饱和和积分 wind-up。若确有稳态误差，应优先使用外力矩
前馈或混合捕获。

用期望自然频率和阻尼比配置：

```text
K_p = I ω_n²
K_d = 2 ζ I ω_n
```

这样三根惯量不同的指针可以获得相近的可预测回归时间。

欠阻尼线性化下，经过 `N` 个完整周期后的振幅近似为：

```text
A_N/A_0 ≈
    exp[
        -2π ζ N / sqrt(1-ζ²)
    ]
```

所以“摆动几次后快速收敛”应通过目标振幅比反推 `ζ`，再由上式计算 `K_d`，而不是随意
给阻尼乘一个材质系数。非线性大角摆动和力矩限幅下仍需实测，但该式提供可重复的初值。

### 10.3 非时间力矩与精确指时的矛盾

若回归完成后仍保留重力、基座旋转、轴承阻尼或其他非时间力矩，单纯 PD 会在非目标
位置平衡。即使基座场已关闭，秒针以 `ω_target≠0` 连续运动时，轴承阻尼
`-cθ̇` 也不是零：

```text
τ_time + τ_nonTime = 0
```

精确跟踪的目标约化动量为：

```text
p_target =
    I(ω_target + Ω_z)
```

若要求解析上的精确跟踪，所需主动时间力矩为：

```text
τ_time,required =
    ṗ_target
  - τ_nonTime(
        θ_target,
        ω_target
    )
```

其中 `ṗ_target` 包含目标角加速度以及表壳 `Ω_z` 的变化。必须明确选择一种策略：

1. 捕获阶段将基座场权重平滑衰减到 `0`，并将轴承阻尼关闭或改为相对时间目标的捕获
   阻尼；
2. 保留物理项，同时用上式对已知重力、离心、阻尼和目标运动做前馈补偿；
3. 进入很小捕获区后使用有力矩/冲量上限的主动时间约束，使预测动量趋向
   `p_target`，再配合位置误差稳定；这仍是约束冲量，不是直接覆盖 `theta`；
4. 使用明确的动画模式；
5. 接受可测量的稳态误差，仅用于纯玩具模式。

默认混合模式建议组合第 1 和第 3 种：互动阶段保持真实物理，回归后段逐渐衰减被动
物理场和轴承阻尼，再由有上限的时间约束保持准确走时。若不实现前馈或主动约束，文档
和验收只能承诺“误差在容差内”，不能声称数学上零误差。

### 10.4 动画模式

动画模式是明确的主动轨迹生成器，不属于被动力学。可使用满足位置、速度和加速度边界
连续的五次多项式：

```text
x(s) = 10s³ - 15s⁴ + 6s⁵
s = clamp((t-t0)/T, 0, 1)
```

更严格的实现应根据进入时的 `θ/θ̇/θ̈` 与预计结束时持续前进的
`θ_target/ω_target/α_target` 求完整五次系数，而不是只做零速度插值。

动画期间：

- 轨迹生成器是角度唯一写入者；
- 物理积分器不能同时修改该针；
- 退出时同步 `p=I(θ̇+Ω_z)`；
- 目标使用预计结束时刻的连续时间，不使用回归启动时的旧角度。

### 10.5 混合模式

混合模式是推荐默认体验：

1. 前段保留重力、甩动和欠阻尼自然摆动；
2. 进入捕获区域后，平滑降低物理场权重；
3. 增加接近临界阻尼的时间控制；
4. 必要时最后切入短五次轨迹；
5. 完成后进入 `TIME_TRACKING`。

各阶段必须互斥拥有角度写权限，切换时至少保证 `θ` 和 `θ̇/p` 连续，最好保证角加速度
也连续。

### 10.6 捕获完成条件

单针同时满足：

```text
|e_θ| < angleTolerance
|e_ω| < velocityTolerance
持续时间 > settleDuration
```

才可令该针的 `settled=true`。三根针全部稳定后，全局状态进入 `TIME_TRACKING`，并将
各针模式切换为 `TIME_CONTROLLED`。不能仅因角度瞬间穿过目标就结束，否则会截断自然
动量并产生跳变。

---

## 11. [保留 v1] 有意操作检测与无操作回归定时器

### 11.1 动作量

可构造仅用于手势判断的动作指标：

```text
motionScore =
    w_a max(0, |f|-g0-a_dead)
  + w_ω max(0, |Ω|-ω_dead)
  + w_j max(0, |df/dt|-j_dead)
```

该指标不是新的物理力。它只决定是否重置空闲计时、是否进入物理模式和是否调整传感器
量程/滤波。

### 11.2 迟滞和最短持续时间

动作进入和退出使用不同阈值，并要求最短持续时间。这样可以避免：

- 静止噪声不断重置计时器；
- 指针自身摆动被误认为用户操作；
- 单个冲击样本触发错误模式；
- 快速甩动结束后立即回归。

### 11.3 回归触发

```text
now - lastIntentionalInteraction >= idleTimeout
AND 当前无触摸捕获
AND 非异常传感器恢复窗口
```

`idleTimeout` 只属于物理表盘配置；其设置界面不在本文范围内。

---

## 12. [保留 v1] 触摸拨动、有限抓持、释放与连续性

### 12.1 命中和目标角

触摸点相对表心：

```text
d = touchPosition - dialCenter
θ_touch = atan2(d_x, -d_y)
```

触摸控制器的原始方向、屏幕旋转和镜像必须先映射到同一 `B.x/B.y` 约定，并通过四角
测试确认；不得在命中函数里临时交换轴。触摸帧同样需要单调时间戳。

命中测试应考虑指针宽度、针尾、层级和中心死区。接近表心时方向对像素噪声极敏感，
不能据此产生无限角速度。

### 12.2 有限触摸约束

推荐以力矩约束拖动，而不是每帧覆盖角度：

```text
e_touch =
    shortestAngle(θ, θ_touch)

τ_touchDemand =
    K_touch e_touch
  + C_touch(ω_touch-θ̇)

τ_touch =
    clamp(
        τ_touchDemand,
        -τ_grip,max,
        +τ_grip,max
    )
```

这样可以表现重量、滞后和材料弯曲。`K_touch` 过高会提高固有频率，需要物理子步；
`τ_grip,max` 决定手指能否被弯曲储能或甩动力“挣脱”。
调试和自动脱手必须保留未限幅的 `τ_touchDemand` 以及 `gripSaturated` 标志。

### 12.3 释放速度

使用最近约 `30～80 ms` 的未包裹触摸角和时间戳做加权直线拟合，得到
`ω_release`。不要只用最后两个像素点差分。

释放时：

```text
θ̇ ← ω_release
p  ← I(ω_release + Ω_z)
```

若触摸全程使用力矩约束，当前物理 `θ̇` 已包含部分释放速度，可在手指估计与物理速度
之间按置信度融合；无论哪种方式都不能同时保留旧 `p` 和写入新的独立 `θ̇`。

### 12.4 控制权交接

触摸开始和结束时，在短窗口内平滑改变力矩权重。释放后的第一物理步必须继承：

- 当前 `θ`；
- 一致的 `p/θ̇`；
- 柔性储能；
- 当前表壳 `Ω_z`；
- 当前时间目标和物理状态。

禁止通过重建默认状态消除松手瞬间的动量。

---

## 13. [修正 v2] 材料、几何、配重、转轴与阻力参数化

### 13.1 “材质惯性”的严格含义

材料不直接施加“惯性力”。材料通过以下物理参数影响体验：

| 来源 | 参数 | 主要视觉结果 |
|---|---|---|
| 材料密度 | `ρ` | 与几何共同决定 `m/H/I` |
| 几何和配重 | 形状、厚度、针尾 | 改变质心、质量一阶矩和惯量 |
| 转轴结构 | `c1/c2/τc` | 改变整体摆动衰减 |
| 材料弹性 | `E`、截面二次矩 | 改变弯曲刚度 |
| 内部损耗 | 损耗因子、关节阻尼 | 改变柔性振动衰减 |
| 接触属性 | `τ_grip,max`、触摸刚度 | 改变拖动和脱手 |

“金属、木、橡胶”可以是参数预设名称，但求解器只能读取有单位的物理系数。

### 13.2 从材料和几何派生质量属性

```text
m = ∫_V ρ(r) dV
r_c = (1/m) ∫_V r ρ(r) dV
H = ∫_V s(r) ρ(r) dV
I = ∫_V |e_z×r|² ρ(r) dV
```

参数加载后应计算并冻结这些派生量。运行循环不得一边积分、一边任意修改 `m/r_c/I`。

一致性边界：

```text
m > 0
I > 0
I_z ≥ m |r_c,xy|²
I_z ≤ m L_max²      // 所有质量均在 L_max 内时
```

### 13.3 仅改变密度不一定改变刚性轨迹

若同一几何的密度整体乘以常数 `k`：

```text
m → km
H → kH
I → kI
```

理想、无阻尼的均匀场角加速度：

```text
θ̈ ∝ H/I
```

保持不变。因此，仅把“金材质”的质量全部乘大，并不会自动让同形状刚性指针在重力下
摆得更慢。可辨识的真实差异应来自：

- 质量重新分布或加入配重；
- 指针长度、宽度和厚度改变；
- 轴承阻尼相对惯量的比例改变；
- 弯曲刚度和内部损耗改变；
- 接触和碰撞属性改变。

若产品仍希望同几何材质产生夸张差异，应使用单独的 `artisticResponseScale`，并明确
标记为艺术增强。

### 13.4 转动惯量与弯曲截面惯量不得混名

- 刚体动力学的质量转动惯量：`I_mass`，单位 `kg·m²`；
- Euler–Bernoulli 梁的截面二次矩：`I_area`，单位 `m⁴`；
- 弯曲刚度：`E I_area`，单位 `N·m²`。

代码中禁止都命名为 `I` 后依赖上下文猜测。

### 13.5 推荐配置分层

```cpp
struct MaterialPreset {
    float densityKgM3;
    float youngModulusPa;
    float internalLoss;
    float surfaceDragScale;
};

struct HandGeometry {
    // 固定容量轮廓、厚度、配重和转轴位置
};

struct PivotPreset {
    float viscousNmSecPerRad;
    float quadraticNmSec2PerRad2;
    float coulombNm;
};

struct ArtisticPreset {
    float gravityScale;
    float responseScale;
    float stylizedDirectionOffsetRad;
};
```

前三者形成物理参数，最后一项必须可一键关闭，以便与严格模型对照。

若艺术层只想夸张重力、不想同时夸张用户甩动，应使用：

```text
a_eff,art =
    -f_O^B
  + (gravityScale - 1) gravityVectorBody
```

不能直接计算 `gravityScale*(-f_O)`，因为这会把表心平移惯性也一起缩放。

---

## 14. [保留 v1] 可选柔性指针与达到夹角后脱手

### 14.1 首版边界

刚性三针、重力/甩动、触摸和回归通过全部验收前，不应让柔性模型进入主路径。柔性会
提高自由度、最高固有频率和触摸约束难度。

### 14.2 二段近似

首个可用模型将指针分为根段和尖端段：

```text
θ_root：根段相对表盘角
β：尖端相对根段弯曲角
θ_tip = θ_root + β
```

弯曲势能和阻尼：

```text
U_bend = 0.5 k_bend β²
τ_bend = -k_bend β - c_bend β̇
```

这两式只定义内部关节的本构关系，**不是完整的 `β` 独立振子方程**。物理二段模型必须
给两段分别配置质量、质心长度和质心转动惯量，并联立求解：

```text
q = [θ_root, β]ᵀ

M(q) q̈
+ C(q,q̇)
= Q_effective
 + Q_rotatingBase
 + Q_touch
 + Q_control
 + [0, τ_bend]ᵀ
```

令：

```text
d1：根段转轴到弯曲关节的距离
l1：根段质心距离
l2：尖端段相对弯曲关节的质心距离
J1/J2：两段关于各自质心的转动惯量
b = m2 d1 l2
```

在非旋转基座的标准二连杆部分：

```text
M11 =
    J1 + m1 l1²
  + J2 + m2(d1² + l2² + 2d1l2 cosβ)

M12 = M21 =
    J2 + m2(l2² + d1l2 cosβ)

M22 =
    J2 + m2 l2²

C1 =
    -b sinβ(2 θ̇_root β̇ + β̇²)

C2 =
    +b sinβ θ̇_root²
```

均匀有效场的广义力矩：

```text
Q_effective,1 =
    (m1l1 + m2d1)t(θ_root)ᵀa_eff
  + m2l2 t(θ_root+β)ᵀa_eff

Q_effective,2 =
    m2l2 t(θ_root+β)ᵀa_eff
```

内部关节真实承受等大反向力矩；在 `q=[θ_root,β]` 的相对坐标中，其虚功恰好对应
`[0,τ_bend]ᵀ`，因此不会凭空改变两段总角动量。移动、三维旋转表盘的附加项应从各段
点速度：

```text
v^W =
    V_O^W
  + R_WB[
        Ω×r^B(q)
      + J_r(q)q̇
    ]
```

统一形成 `M/C/Q_rotatingBase`，不得把单根刚针的离心经验式分别复制到两段。
`2×2` 质量矩阵可每子步直接求逆，再做：

```text
q̈ = M(q)⁻¹ [Q_total - C(q,q̇)]
q̇ ← q̇ + q̈ Δt_sub
q  ← q  + q̇ Δt_sub
```

并检查 `det(M)>ε` 和正定性。`k_bend` 可从梁模型量级估算，再由实机调参，但不能复用
时间回归刚度。

若只把 `β` 做成不反作用根部的从属动画，必须命名为 `VisualFlex`，并明确它不守恒，
不能再声称脱手后弯曲储能会物理地转换成两段运动。

### 14.3 自动脱手

触摸抓持可在以下任一条件成立并持续超过各自迟滞时间后释放：

```text
|β| > β_release
U_bend > E_release
(|τ_touchDemand| >= τ_grip,max)
    AND saturatedDuration > saturationHold
```

`τ_touchDemand` 是夹紧前的触摸需求力矩；实际施加的 `τ_touch` 已被限制在
`±τ_grip,max`，所以不能再用 `|τ_touch|>τ_grip,max` 作为条件。

释放时不得把 `β/β̇` 清零。储存的弯曲能应在脱手后继续转化为根部和尖端运动，形成
用户期待的“掰弯到一定夹角后松开弹走”。

### 14.4 多段模型

更高版本可使用固定数量的离散刚体链或 XPBD/PBD。此时：

- 各段共享移动表盘约束；
- 质量矩阵、科里奥利/离心耦合不再能用单针经验项完整描述；
- 需要 `200～400 Hz` 子步或基于约束的稳定求解；
- 段数必须编译期设上限，实时循环不动态分配。

---

## 15. [修正 v2] 固定步长、数值积分与多速率循环

### 15.1 三种积分方式

#### 显式 Euler

```text
θ_{k+1} = θ_k + ω_k Δt
ω_{k+1} = ω_k + α_k Δt
```

优点是最简单；缺点是对摆、弹簧和接触约束容易错误注入能量。只适合公式冒烟测试，
不推荐作为正式求解器。

#### 半隐式 Euler

```text
ω_{k+1} = ω_k + α_k Δt
θ_{k+1} = θ_k + ω_{k+1} Δt
```

每步只评估一次力矩，对振子比显式 Euler 稳定，容易处理触摸、摩擦、限幅和状态切换。
这是 ESP32-S3 刚性首版的推荐积分器。

#### RK4

RK4 对光滑、连续、可在中间时刻正确评估的微分方程有高阶精度，但本项目包含：

- 离散 IMU 样本；
- 触摸开始/释放；
- 库仑摩擦和平滑死区；
- 力矩饱和；
- 状态机切换和可选碰撞。

RK4 每步约需四次力矩评估，还需构造中间时刻的 IMU、触摸和目标输入。理论高阶优势
很容易被非光滑输入抵消。建议只用于 PC 离线参考解和回归测试。

### 15.2 推荐的动量半隐式更新

对约化动量状态：

```text
p_{k+1} =
    p_k
  + Δp_effectiveField
  + (
        τ_centrifugal
      + τ_drive
      + τ_timeControl
      + τ_touch
      + τ_elastic
      + τ_collision
      + τ_damping
    ) Δt

θ̇_{k+1} =
    p_{k+1}/I - Ω_z(t_{k+1})

θ_{k+1} =
    θ_k + θ̇_{k+1} Δt
```

该顺序等价于半隐式 Euler。`Ω_z` 要插值到对应物理时刻。

### 15.3 [新增 v2] 高采样 IMU 到低频物理步的冲量保持

QMI8658 六轴常用 ODR 档位包含 `235 Hz`，而物理默认 `100 Hz`。不能在每个物理步只取
最新加速度样本；一个短促甩动可能刚好落在两个物理采样点之间。

在物理区间 `[t_k,t_{k+1}]` 累积：

```text
J_eff =
    ∫ a_eff,O^B(t) dt
≈   Σ 0.5(a_eff,n + a_eff,n+1) Δt_n
```

若一个 IMU 样本区间跨过物理步边界，按时间比例拆分该梯形积分，使每份冲量只进入一个
物理区间。FIFO 中同批样本仍必须保留各自时间戳，不能全部使用读取时刻。

严格梯形插值需要边界后的下一条样本，可让物理时间相对墙钟固定落后约一个 IMU 周期；
若必须最低延迟，则使用因果零阶保持或预测，并在遥测中记录其相位延迟。姿态、基座
冲量、触摸和渲染必须采用一致的时间对齐策略，不能各自补偿不同延迟。

若一个物理步内 `θ` 变化不大：

```text
Δp_effectiveField =
    H t(θ_eval)ᵀ J_eff
```

`θ_eval` 可用当前角或半步预测角。若该近似在强甩动时误差过大，应按 IMU 样本执行
物理子步，而不是退回“只取最后一帧”。

三维离心项也应使用时间加权平均。更严格的区间量为：

```text
M_Ω =
    ∫ Ω(t) Ω(t)ᵀ dt
```

细杆在 `θ` 近似不变时：

```text
Δp_centrifugal =
    -I u(θ_eval)ᵀ M_Ω t(θ_eval)
```

首版也可逐 IMU 样本累计 `τ_centrifugal Δt`。

### 15.4 线性阻尼的低成本隐式处理

当高阻尼导致显式更新僵硬时，对：

```text
τ_damping = -c θ̇ = -c(p/I - Ω_z)
```

可使用：

```text
p_{k+1} =
    [
        p_k
      + Δp_nonDamping
      + c Ω_z(t_{k+1}) Δt
    ]
    /
    [
        1 + c Δt/I
    ]
```

它只增加一次除法，却能避免大阻尼把速度符号来回翻转。

### 15.5 固定步长和追赶

- 基础调度步固定 `Δt_base=0.01 s`，即 `100 Hz`；
- 每个基础步按加载后的稳定性配置执行 `N=1/2/4` 个固定子步，
  `Δt_sub=Δt_base/N`；
- 调度延迟不直接成为可变 `dt`；
- 落后时补做有限个完整基础步，建议最多 `2～4` 个；这与单步内部的稳定性子步不是
  同一个概念；
- 超过上限则丢弃过量累计时间、记录 `physicsOverrun` 并降级；
- 禁止用一次巨大 `dt` 追赶；
- 渲染帧率不得改变物理轨迹。

### 15.6 稳定性和子步判据

对局部等效刚度：

```text
K_eff,max ≈
    |H| |a_eff|max
  + K_time
  + K_touch
  + K_elastic,equivalent

ω_max ≈ sqrt(K_eff,max / I)
```

保守精度条件：

```text
ω_max Δt_sub ≤ 0.2～0.25
```

建议在加载参数或进入高刚度状态时确定固定子步数：

```text
substeps =
    nextPowerOfTwo(
        clamp(
            ceil(ω_max × physicsDt / 0.25),
            1,
            4
        )
    )
```

子步只取 `1/2/4`，便于预先把每个 `10 ms` 基础步切成四个可合并的时间片。若达到四个
子步仍不满足条件，应降低刚度、改用更稳定的约束求解器或提高基础物理频率。不要每帧
根据 CPU 抖动随意改变 `dt`。

### 15.7 物理和渲染解耦

物理任务发布时间为 `t_k/t_{k+1}` 的不可变快照。渲染在二者之间插值未包裹角：

```text
θ_render =
    lerp(θ_previous, θ_current, renderAlpha)
```

柔性关节也使用相同快照插值。渲染掉帧只影响视觉平滑度，不应使物理时间变慢或改变
能量。

---

## 16. [修正 v2] C/C++ 模块架构与数据结构

### 16.1 模块边界

```text
ImuFrontend
    原始读取、时间戳、校准、轴映射、饱和标志

AttitudeEstimator
    Mahony/Madgwick、q_WB、gravityVectorBody、置信度

BaseKinematics
    IMU 偏心修正、a_eff、Ω、α、区间冲量

TouchConstraint
    命中、角目标、速度估计、有限抓持和释放

TimeTargetProvider
    三根针连续 TIME_REF；由物理引擎外部实现

PhysicalDialEngine
    状态机、力矩组合、固定步积分、唯一状态写入

TimeReturnController
    机械/动画/混合回归

PhysicsDebug
    遥测、录制、回放、合成输入和不变量检查
```

### 16.2 数据契约

```cpp
struct Vec3f {
    float x, y, z;
};

struct Quatf {
    float w, x, y, z;
};

struct ImuRawSample {
    uint64_t timestampUs;
    int16_t accelRaw[3];
    int16_t gyroRaw[3];
    int16_t temperatureRaw;
    uint16_t flags;
};

struct ImuCalibratedSample {
    uint64_t timestampUs;
    Vec3f specificForceSensorBodyMps2;
    Vec3f angularVelocityStaticCorrectedBodyRadSec;
    uint32_t qualityFlags;
};

struct AttitudeEstimate {
    uint64_t timestampUs;
    Quatf qWB;
    Vec3f gravityBodyMps2;
    Vec3f gyroBiasRadSec;
    Vec3f angularVelocityFinalBodyRadSec;
    float accelCorrectionWeight;
    float confidence;
    uint32_t qualityFlags;
};

struct BodyExcitationSample {
    uint64_t timestampUs;
    Vec3f specificForceCenterBodyMps2;
    Vec3f effectiveAccelCenterBodyMps2;
    Vec3f angularVelocityBodyRadSec; // 复制姿态估计器的 final Ω
    Vec3f angularAccelerationBodyRadSec2;
    uint32_t qualityFlags;
};

struct BaseSubInterval {
    uint64_t startUs;
    uint64_t endUs;
    Vec3f effectiveAccelImpulseMps;
    float omegaOuterProductIntegral[3][3];
    Vec3f omegaBodyAtStartRadSec;
    Vec3f omegaBodyAtEndRadSec;
    Vec3f gravityBodyAtEndMps2;
    uint32_t qualityFlags;
};

static constexpr size_t kBaseQuarterCount = 4;

struct BaseInterval {
    uint64_t startUs;
    uint64_t endUs;
    BaseSubInterval quarter[kBaseQuarterCount];
    uint32_t qualityFlags;
};
```

姿态估计和基座激励必须使用不同结构，防止开发者误把 `gravityBodyMps2` 与
`effectiveAccelCenterBodyMps2` 相加。
`BodyExcitationSample.angularVelocityBodyRadSec` 必须逐样本复制
`AttitudeEstimate.angularVelocityFinalBodyRadSec`，确保两条链使用相同在线零偏版本。

`ImuTask/BaseKinematics` 应按物理时间边界把每个 `10 ms` 基础步预积分成四个
`2.5 ms` 时间片。物理使用 1、2 或 4 子步时分别合并 `4`、`2+2` 或逐个消费。每个
时间片的冲量只能消费一次；严禁在四个子步中重复使用整步 `J_eff/M_Ω`。

### 16.3 指针参数与状态

```cpp
enum class HandMode : uint8_t {
    TimeControlled,
    ImuFree,
    TouchConstrained,
    Returning
};

struct HandDefinition {
    float massKg;
    float visualLengthM;
    Vec3f centerOfMassAtZeroM;
    float inertiaPivotKgM2;
    // 其余几何、转轴、回归和触摸配置
};

struct HandParameters {
    // 由 HandDefinition 验证并派生的只读实时参数
    float massKg;
    float visualLengthM;
    Vec3f firstMomentAtZeroKgM;
    float inertiaPivotKgM2;

    float viscousDampingNmSecPerRad;
    float quadraticDampingNmSec2PerRad2;
    float coulombFrictionNm;
    float frictionSmoothVelocityRadSec;

    float timeKpNmPerRad;
    float timeKdNmSecPerRad;
    float maxTimeTorqueNm;
    float touchKpNmPerRad;
    float touchKdNmSecPerRad;
    float maxTouchTorqueNm;

    float maxAngularSpeedRadSec;
    float maxRelativeKineticEnergyJ;
};

struct RigidHandState {
    float angleUnwrappedRad;
    float reducedMomentumKgM2RadSec;
    HandMode mode;
    bool settled;
    uint32_t flags;
};

struct RelativeTorqueBreakdown {
    float driveNm;
    float timeControlNm;
    float touchNm;
    float rootElasticNm; // 仅刚针根部可选扭簧，不是二段柔性关节
    float collisionNm;
    float dampingNm;
    float totalNm;
};

struct TorqueBreakdown {
    float effectiveFieldNm;
    float centrifugalNm;
    RelativeTorqueBreakdown relative;
    float totalNm;
};

struct FlexibleHandState2D {
    float qRad[2];       // [thetaRoot, beta]
    float qDotRadSec[2];
    HandMode mode;
    bool settled;
    uint32_t flags;
};

struct GeneralizedForce2D {
    float rootNm;
    float bendNm;
};

struct FlexibleForceBreakdown2D {
    GeneralizedForce2D effectiveField;
    GeneralizedForce2D rotatingBase;
    GeneralizedForce2D touch;
    GeneralizedForce2D control;
    GeneralizedForce2D internalBend;
    GeneralizedForce2D damping;
    GeneralizedForce2D total;
};
```

工程约束：

- 刚针 `reducedMomentum` 是权威速度状态；
- `angularVelocity = p/I - Ω_z` 按需派生；
- `HandDefinition` 是唯一可编辑来源，`firstMomentAtZero=m*centerOfMassAtZero`；
- `HandParameters` 在加载时派生、验证，初始化后只读；
- `maxRelativeKineticEnergyJ` 只限制 `0.5Iθ̇²`；移动基座下不存在一个可直接硬夹的
  固定“总能量零点”；
- 柔性数组使用编译期固定上限；
- 物理热循环不使用堆分配、字符串和容器增长；
- 全部物理量用单精度 `float`；
- 长期运行时可同时给当前角和目标角减去整数倍 `2π`，避免未包裹角过大降低
  `float` 分辨率。

`RigidHandState` 和下面第 16.5 节的动量伪代码只适用于单自由度刚针。二段柔性模型必须
使用 `FlexibleHandState2D`，每子步求解 `2×2` 质量矩阵后同时更新
`qDot[0]/qDot[1]`；不能先按刚针公式更新一个 `p`，再把 `beta` 当独立从属振子积分。
刚性和柔性可用固定容量 tagged union 或两条编译期路径，禁止含糊共用同一标量力矩。

### 16.4 力矩提供器

首版不需要虚函数或运行期多态。可用固定函数和启用掩码：

```cpp
struct TorqueContext {
    float dt;
    uint64_t evalTimeUs;
    float theta;
    float omegaRelative;
    Vec3f u;
    Vec3f tangent;
    const HandParameters* parameters;
    const TimeTarget* timeTarget;
    const TouchTarget* touchTarget;
};

RelativeTorqueBreakdown evaluateRelativeTorques(
    const TorqueContext& context,
    uint32_t enabledMask);
```

`evaluateRelativeTorques()` 的 `totalNm` **只允许**包含
`drive/time/touch/rootElastic/collision/damping`。有效场和离心项由基座区间积分单独加入；
完整 `TorqueBreakdown` 只在合并遥测时生成。该契约防止有效场或离心力矩被累计两次。
固定顺序和分项输出比复杂插件系统更适合实时调试。

### 16.5 参考物理步伪代码

```cpp
void PhysicalDialEngine::integrateSubstep(
    const BaseSubInterval& base,
    const TimeTarget timeTargets[3],
    const TouchTarget touchTargets[3],
    float dt)
{
    const uint64_t evalTimeUs =
        base.startUs + (base.endUs - base.startUs) / 2;

    for (size_t i = 0; i < 3; ++i) {
        RigidHandState& s = rigidStates_[i];
        const HandParameters& p = parameters_[i];

        const float omegaStart =
            s.reducedMomentumKgM2RadSec / p.inertiaPivotKgM2
          - base.omegaBodyAtStartRadSec.z;

        const float thetaEval =
            s.angleUnwrappedRad + 0.5f * omegaStart * dt;

        const Vec3f u = {
            sinf(thetaEval), -cosf(thetaEval), 0.0f
        };
        const Vec3f t = {
            cosf(thetaEval),  sinf(thetaEval), 0.0f
        };

        const Vec3f firstMoment = rotateAroundZ(
            p.firstMomentAtZeroKgM, thetaEval);
        const Vec3f firstMomentDerivative = {
            -firstMoment.y, firstMoment.x, 0.0f
        };

        const float deltaPEffective =
            dot(firstMomentDerivative,
                base.effectiveAccelImpulseMps);

        const float deltaPCentrifugal =
            -p.inertiaPivotKgM2
          * dot(u, multiply(base.omegaOuterProductIntegral, t));

        const TimeTarget timeTarget =
            extrapolateTimeTarget(timeTargets[i], evalTimeUs);
        const TouchTarget touchTarget =
            sampleTouchTarget(touchTargets[i], evalTimeUs);

        const TorqueContext torqueContext = {
            dt,
            evalTimeUs,
            thetaEval,
            omegaStart,
            u,
            t,
            &p,
            &timeTarget,
            &touchTarget
        };
        const RelativeTorqueBreakdown relative =
            evaluateRelativeTorques(
                torqueContext, enabledTorqueMask(i));

        const float candidateMomentum =
            s.reducedMomentumKgM2RadSec
          + deltaPEffective
          + deltaPCentrifugal
          + relative.totalNm * dt;

        const float omegaNew =
            limitMomentumAndDeriveOmega(
                i,
                candidateMomentum,
                base.omegaBodyAtEndRadSec.z,
                &s.reducedMomentumKgM2RadSec);

        // limitMomentumAndDeriveOmega 必须记录：
        // safeMomentum - candidateMomentum 对应的安全冲量。
        s.angleUnwrappedRad += omegaNew * dt;

        if (!isfinite(s.angleUnwrappedRad)) {
            recoverRigidHandState(i);
        }
    }
}
```

外层 `step()` 根据固定配置选择 `1/2/4` 子步，用 `coalesceBaseQuarters()` 为每个子步
生成互不重叠的 `BaseSubInterval`，依次调用上面的函数，并且只在全部子步完成后发布
一次不可变快照。

外层还必须断言相邻子区间满足：

```text
previous.endUs == current.startUs
previous.omegaBodyAtEnd ≈ current.omegaBodyAtStart
```

首次进入物理模式时，应使用当前 `Ω_z,start` 显式初始化
`p=I(θ̇_initial+Ω_z,start)`。发生时间缺口、重叠或角速度不连续且没有相应质量标志时，
禁止沿用隐含旧状态，必须进入传感器降级/重新同步路径。

若启用隐式线性阻尼，应从 `relative.totalNm` 中移除该线性项，并使用第 15.4 节的分母
更新。速度、能量和有限值检查必须先修正权威动量，再用修正后的 `omegaNew` 积分角度；
不能先用非法/超限速度更新 `theta`，随后才限幅。
若一个物理步内 `θ` 变化过大，则拆成固定子步并分别消费相应的 IMU 区间量。
伪代码的有效场力矩支持任意平面质心偏置；离心行仍采用细杆近似。有限宽度薄片应以
`0.5 Ωᵀ(∂J_B/∂θ)Ω` 替换该行。

---

## 17. [新增 v2] ESP32-S3 可实现性与资源预算

### 17.1 结论

三根刚性指针、六轴姿态融合和 `100～235 Hz` 的传感器处理对 ESP32-S3 很轻。主要
资源压力通常来自 368×448 AMOLED 的绘制和帧缓冲，而不是刚体方程。

该开发板公开规格包括：

- 双核 Xtensa LX7，最高 `240 MHz`；
- `512 KB` 片上 SRAM；
- `8 MB` PSRAM；
- `16 MB` 外部 Flash；
- 368×448 AMOLED；
- QMI8658 六轴 IMU。

### 17.2 推荐频率

| 模块 | 推荐首版 | 说明 |
|---|---:|---|
| QMI8658 加速度计+陀螺仪 | `235 Hz` | 接近用户提出的 200 Hz，且是公开数据手册中的六轴 ODR 档位 |
| 姿态融合 | `235 Hz` | 每个有效 IMU 样本更新一次 |
| 刚性物理 | `100 Hz` 基础调度 + `1/2/4` 固定子步 | 是否单步足够由 `ω_max Δt_sub` 判据和实测决定 |
| 触摸事件 | 面板允许的稳定频率 | 带时间戳送入物理任务 |
| 柔性/硬触摸子步 | `200～400 Hz` | 由 `ω_max Δt_sub` 判据决定 |
| 渲染 | 目标 `60 FPS` | 可降级到 45/30 FPS，物理频率不变 |
| 屏上调试文字 | `10～20 Hz` | 避免调试 UI 本身拖慢渲染 |

“IMU 200 Hz”应理解为频率等级，不能强行配置芯片不存在的精确档位。最终以实际驱动和
芯片版本数据手册为准。

短指针自身的重力摆固有频率也可能使单个 `10 ms` 步不满足保守精度条件，所以子步机制
不是只为柔性或碰撞准备。`100 Hz` 表示调度和快照基础周期，不保证每个周期只积分一次。

### 17.3 实时任务

```text
QMI8658 DRDY/FIFO ISR
    │ 只通知，不做 I²C 和浮点计算
    ▼
ImuTask（高优先级）
    校准、时间戳、姿态融合、偏心修正、区间量累计
    │
    ▼
PhysicsTask（高优先级、固定 100 Hz）
    唯一物理状态写入者、处理事件、积分、发布快照
    │
    ▼
RenderTask（中优先级、目标 60 FPS）
    只读最近两个快照、插值、局部重绘

LoggerTask（最低优先级）
    从环形缓冲异步导出，不阻塞物理
```

规则：

- 物理任务不得等待显示、TF 卡、串口、网络或大段日志；
- ISR 不访问 I²C，不运行融合和物理；
- 若触摸和 IMU 共用总线，总线事务必须短，并记录等待时间；
- 使用单调微秒时钟作为仿真时间，RTC 只提供目标时间；
- 任务绑核方案应在启用实际显示驱动后实测，不能仅凭双核数量假定没有竞争。

### 17.4 CPU 预算

三根刚性针在 `100 Hz` 基础调度、`1～4` 子步下，每秒约有 `300～1200` 次状态更新；
即使包含三角函数、点积、离心项和多个力矩通道，运算量也很小。Mahony/Madgwick 在约
`235 Hz` 同样不是主要瓶颈。

建议以实测时延作为验收，不在设计阶段承诺未经测量的 CPU 百分比：

```text
ImuTask 单样本 P99       < 1 ms
PhysicsTask 单个 10 ms 步 P99 < 1 ms
IMU FIFO overflow        = 0
物理 overrun             = 0（正常模式）
启用日志不改变回放轨迹
```

### 17.5 内存预算

数量级建议：

| 项目 | 预算 |
|---|---:|
| 三针参数、状态和最新力矩 | `< 4 KB` |
| 姿态、基座区间和双物理快照 | `< 8 KB` |
| 256 条 IMU 环形缓冲 | 约 `6～16 KB`，取决于字段 |
| 完整物理调试历史 | `32～128 KB`，优先放 PSRAM |
| RGB565 单全屏缓冲 | `368×448×2 = 329,728 B` |
| RGB565 双全屏缓冲 | `659,456 B` |

完整帧缓冲不宜挤占实时热状态所需的片上 SRAM。帧缓冲和长日志放 PSRAM；DMA 有特殊
内存要求时使用小块片上行缓冲。

60 FPS 全屏 RGB565 的未压缩数据量约：

```text
329,728 × 60 ≈ 19.8 MB/s
```

所以 60 FPS 是目标，不是未测量的保证。优先重绘旧/新指针包围盒，或采用显示驱动
支持的局部区域提交。

### 17.6 降级策略

按以下顺序降级：

1. 渲染从 `60 → 45 → 30 FPS`，保持物理 `100 Hz`；
2. 关闭粒子、阴影和高阶柔性；
3. 减少柔性段数，但保留三根刚性根部；
4. 暂停低优先级日志；
5. IMU 短时异常时，用陀螺预测姿态并将动力学输入平滑退回姿态重力；
6. 长时间无可靠 IMU 时，进入舒适时间回归；
7. 检测到 `NaN/Inf` 时重建受影响指针，不继续传播非法状态。

加速度样本缺失时不能无限保持最后一次甩动力；FIFO 溢出后也不能跨未知时间段伪造
冲量。

---

## 18. [修正 v2] Physics Debug Mode

### 18.1 传感器层

显示或记录：

- 原始 `accel/gyro`；
- 校准后的 `f_S^B`；
- QMI ODR、量程、FIFO 水位、溢出和饱和；
- 样本时间间隔、年龄、丢样数；
- `C_BM`、零偏和校准版本。

### 18.2 姿态与基座层

- `q_WB` 和 `|q|-1`；
- `gravityVectorBody`；
- 加速度计姿态修正权重；
- `f_O^B`；
- `a_eff,O^B`；
- 调试拆分的 `A_O^B=f_O^B+g^B`；
- `Ω^B`、带限 `α^B`；
- IMU 偏心的切向和向心修正；
- 当前姿态/动力学质量标志。

屏上可在表心画两根不同颜色的向量：

```text
纯重力：gravityVectorBody
合成有效场：effectiveAccelerationAtCenter
```

二者在静止时重合，甩动时分离。这是发现重力重复计算最直观的方法。

### 18.3 单指针层

每根针显示：

- `θ`、`θ̇`、`p`；
- `θ_target`、`ω_target`、最短角误差；
- `τ_effectiveField`；
- `τ_gravity` 和 `τ_inertiaTranslation` 的调试拆分；
- `τ_centrifugal`；
- 显式模式下的 `τ_Euler`；
- `τ_timeControl/τ_touch/τ_elastic/τ_collision/τ_damping`；
- `τ_total`；
- 柔性角、触摸误差、`τ_touchDemand`、`gripSaturated` 和抓持余量；
- 单针状态和当前控制权拥有者。

### 18.4 能量层

- `T_relative`；
- `U_gravity`；
- `U_time`；
- `U_flex`；
- 各力矩通道的功率 `P_j=τ_j θ̇`；
- 基座、触摸和时间控制累计做功；
- 阻尼累计耗散；
- 限能量和速度夹紧次数。

只在基座固定且没有主动输入时使用“总能量不持续增加”作为不变量。

### 18.5 实时系统层

- IMU、融合、物理、渲染实际频率；
- 各任务 P95/P99/最大执行时间；
- `physicsOverrun`；
- 队列和环形缓冲高水位；
- Render FPS；
- 丢弃日志数；
- 当前子步和降级等级。

### 18.6 调试操作

Physics Debug Mode 至少支持：

- 暂停/继续；
- 单物理步；
- 开关各力矩通道；
- 显示严格物理和艺术增强差值；
- 注入合成重力、平移加速度和角速度；
- 录制原始 IMU、触摸、命令和时间戳；
- 确定性回放；
- 导出参数版本、固件模型版本和配置哈希。

屏上文字只以 `10～20 Hz` 更新。完整遥测写入固定环形缓冲，再由低优先级任务异步
导出；禁止在物理循环大量 `printf`。

---

## 19. [修正 v2] 验收测试与数学一致性测试

以下测试应同时存在于 PC 离线测试和实机 Debug Mode。没有通过这些测试前，不应通过
“看起来挺顺”判断模型正确。

### 19.1 坐标和四元数

不变量：

```text
|q| ≈ 1
RᵀR ≈ Identity
det(R) ≈ +1
R_BW R_WB v ≈ v
C_BMᵀ C_BM ≈ Identity
det(C_BM) ≈ +1
```

测试：

1. `q` 和 `-q` 产生完全相同的 `gravityVectorBody`；
2. 绕世界重力轴只改变全局航向时，`gravityVectorBody` 不变；
3. 六面静态测试中 `|gravityVectorBody|≈g0`；
4. 静止时：

   ```text
   f_S^B + gravityVectorBody ≈ 0
   ```

5. 正向绕各轴旋转时，陀螺仪符号符合右手规则；
6. 屏幕竖直正放时，表盘内重力接近六点方向；
7. 屏幕平放时，平面重力力矩连续趋近零，不因 `atan2` 噪声跳动。

### 19.2 防止重力重复

1. 静止时：

   ```text
   a_eff,O^B = -f_O^B ≈ gravityVectorBody
   ```

2. 合成自由落体输入中：

   ```text
   f_O^B≈0
   a_eff,O^B≈0
   ```

3. 静止力矩不得接近 `2g`；
4. 拆分式和合并式满足：

   ```text
   τ_gravity + τ_inertia,translation
   ≈ τ_effectiveField
   ```

5. 关闭姿态链后，只要校准比力仍有效，动态求解结果不应突然多一份或少一份重力。

### 19.3 质心、势能和力矩

用中心有限差分验证：

```text
τ_gravity(θ)
≈
-[U_g(θ+ε)-U_g(θ-ε)] / (2ε)
```

测试：

- 质心指向重力时力矩为零、势能为局部最小；
- 质心反向重力时力矩为零、势能为局部最大；
- 有效场沿指针径向时力矩为零；
- 有效场沿切向时力矩模最大；
- `I_z≥m|r_c,xy|²`；
- 同一几何均匀缩放密度时，无阻尼自由轨迹应基本重合；
- 修改配重位置但保持总质量时，轨迹应随 `H/I` 改变。

### 19.4 表心平移和旋转

1. 匀速平移不产生平移惯性力矩；
2. 表心加速度反向时，初始角加速度反向；
3. 向有效场方向的径向冲量不产生一阶角冲量；
4. 切向冲量满足：

   ```text
   I Δω ≈ H tᵀ J_eff - I ΔΩ_z
   ```

5. 纯法向恒角速度旋转时 `τ_centrifugal=0`；
6. 无相对力矩时 `p` 保持常数；
7. `Ω_z` 突变且无外广义冲量时：

   ```text
   Δθ̇ ≈ -ΔΩ_z
   ```

8. 三维离心项在 `Ω→-Ω` 时不变；
9. 合成纯旋转数据经 IMU 偏心修正后，不得残留虚假表心平移；
10. 完整三维公式在 `Ω_x/Ω_y→0` 时收敛到二维简化式。

### 19.5 阻尼、积分和多速率

```text
τ_damping θ̇ ≤ 0
```

测试：

- 固定基座、无主动控制时总机械能不持续增长；
- 将物理步长减半，轨迹应向同一结果收敛；
- `30/45/60 FPS` 渲染产生相同物理快照；
- 同一短促甩动相对物理步相位改变时，冲量保持版本的结果基本不变；
- “只取最新 IMU 样本”对照版本应被测试明确识别为相位敏感；
- 显式 `-Iα_z` 版本与约化动量版本在平滑输入下结果一致；
- 固定输入回放多次，状态差异只在预设浮点容差内。

### 19.6 时间回归

- `RETURNING` 中 RTC 不直接覆盖 `θ`；
- `e_θ=0、e_ω=0` 时反馈力矩为零；
- 时间力矩有明确上限；
- 机械、动画和混合模式不会同时写角度；
- 模式切换时 `θ` 和 `p/θ̇` 连续；
- 回归终点使用持续更新的时间目标；
- 捕获期间保持重力时，测试必须验证前馈或权重衰减能消除稳态误差；
- 秒针跨十二点回归不走错误长路径；
- 三根指针可以在不同时间满足各自 `settled` 条件，但不共享同一个角状态；全局切换时
  三根针都进入 `TIME_CONTROLLED`。

### 19.7 触摸和柔性

- 触摸某针不暂停另外两针；
- 触摸力矩达到上限后不会数值爆炸；
- 中心死区不会产生无限触摸角速度；
- 释放后的第一步保持角度和动量连续；
- 弯曲脱手后不清除 `β/β̇`；
- 高刚度触摸启用子步后，减半子步长轨迹应收敛；
- 触摸、重力和时间控制的启用掩码符合状态机。
- 二段模型每步满足 `M=Mᵀ`，且 `M` 的两个特征值均为正；
- 固定基座、无外力/无阻尼时，内部弹簧的等大反向力矩不改变系统总角动量；
- 二段模型将子步减半后，角度、角速度和能量轨迹向同一参考结果收敛；
- `VisualFlex` 与物理二段模型在遥测和验收中使用不同模式标志。

### 19.8 实机实时性

- IMU FIFO 正常操作无溢出；
- 传感器和物理时间戳单调；
- Physics P99 满足预算；
- Render 掉帧不造成物理步长变化；
- 开启完整日志不改变确定性回放；
- 连续运行后无角度精度明显下降、内存增长和 `NaN`；
- 传感器断流、削顶和任务超时能进入明确降级状态。

---

## 20. [修正 v2] 推荐实现顺序与默认物理配置

### 20.1 四周实施顺序

#### 第 1 周：坐标、传感器和可观测性

- 完成 `C_BM`、单位换算和六面/三轴校准；
- 接通 QMI8658 FIFO/DRDY 与时间戳；
- 实现 Mahony 或 Madgwick；
- 输出 `q_WB` 和 `gravityVectorBody`；
- 建立原始 IMU 录制、PC 回放和基础 Debug Mode；
- 通过第 19.1、19.2 节测试。

#### 第 2 周：刚性世界

- 实现 `HandMassProperties`；
- 实现 `a_eff=-f_O` 和可选 IMU 偏心修正；
- 实现约化动量、三维离心项和半隐式积分；
- 加入被动阻尼、限能量和固定时间步；
- 做静态重力、缓慢旋转和甩动冲量测试。

#### 第 3 周：触摸和时间

- 实现单针命中、有限抓持和稳健释放速度；
- 实现持续 `TIME_REF`；
- 完成机械、动画和混合回归；
- 加入有意操作计时和完整状态机；
- 验证优先级、切换连续性和精确指时。

#### 第 4 周：材质手感和性能收敛

- 从几何/密度派生 `m/H/I`；
- 调整轴承阻尼、回归阻尼比和力矩上限；
- 完成确定性回放、性能统计和长时间测试；
- 优化局部重绘和渲染插值；
- 仅在刚性内核稳定后，试验二段柔性与自动脱手。

### 20.2 推荐首版默认值和策略

| 项目 | 推荐 |
|---|---|
| 姿态算法 | 六轴 Mahony；Madgwick 为等价备选 |
| QMI8658 ODR | `235 Hz` |
| 初始动作录制量程 | 加速度 `±16g`、陀螺仪 `±2048°/s` |
| 刚性物理 | `100 Hz` 基础调度；按加载判据使用 `1/2/4` 子步 |
| 强甩动/硬触摸 | 通常 `2～4` 个固定子步，仍以稳定性测试为准 |
| 渲染 | 目标 `60 FPS`，允许 45/30 降级 |
| 主积分状态 | `θ + p` |
| 旋转模型 | 默认启用廉价的三维细杆离心项 |
| 时间回归 | 混合模式 |
| 空闲回归 | 初始可用约 `5 s`，最终以体验测试决定 |
| 艺术重力偏置 | 默认 `0` |
| 柔性 | 首版关闭，刚性验收后启用 |

量程首次使用最大档是为了记录故意甩动峰值，不代表最终量产设置。根据录制数据，在不
削顶且保留余量的前提下缩小量程以提高分辨率。

### 20.3 参数调节顺序

不要同时拖动所有滑杆。固定动作录制后按以下顺序：

1. 校准传感器和坐标符号；
2. 确认 `m/r_c/H/I` 一致；
3. 在无阻尼下验证解析方向和固有频率；
4. 调轴承阻尼，使自然摆动次数符合预期；
5. 调高速二次阻力和安全限能量；
6. 调触摸刚度与抓持上限；
7. 用 `ω_n/ζ` 调时间回归；
8. 最后增加柔性和艺术增强。

---

## 21. [修正 v2] 最终数学模型与 C/C++ 实现摘要

### 21.1 共享基座输入

```text
g^W = (0,0,-g0)

g^B =
    R_BW(q_WB) g^W
    = gravityVectorBody

f_S^B =
    calibratedSpecificForce

a_eff,O^B =
    -f_S^B
  + α^B × r_OS^B
  + Ω^B × (Ω^B × r_OS^B)
```

若 `r_OS=0`：

```text
a_eff,O^B = -f_O^B
```

静止调试关系：

```text
f_O^B ≈ -g^B
a_eff,O^B ≈ g^B
```

### 21.2 每根指针

```text
u_i = (sinθ_i,-cosθ_i,0)
t_i = (cosθ_i, sinθ_i,0)

h_i,0 = ∫r_0 dm = m_i r_c,i,0
h_i(θ_i) = R_z(θ_i) h_i,0
I_i = ∫|e_z×r|² dm

τ_eff,i =
    [∂h_i(θ_i)/∂θ_i]ᵀ a_eff,O^B
```

对中心线细杆：

```text
H_i = ∫s dm = m_i l_c,i
h_i(θ_i) = H_i u_i
I_i = ∫s² dm
τ_eff,i = H_i t_iᵀ a_eff,O^B

τ_cf,i =
    -I_i(Ω·u_i)(Ω·t_i)

τ_damping,i =
    -c1,i θ̇_i
    -c2,i |θ̇_i|θ̇_i
    -τc,i tanh(θ̇_i/ε_i)
```

显式形式：

```text
I_i θ̈_i =
    τ_eff,i
  - I_i α_z
  + τ_cf,i
  + τ_relative,i
```

推荐约化动量形式：

```text
p_i = I_i(θ̇_i + Ω_z)

ṗ_i =
    τ_eff,i
  + τ_cf,i
  + τ_relative,i

θ̇_i = p_i/I_i - Ω_z
```

其中：

```text
τ_relative =
    τ_drive
  + τ_timeControl
  + τ_touch
  + τ_elastic
  + τ_collision
  + τ_damping
```

### 21.3 时间目标

```text
e_θ = shortestAngle(θ, θ_target)
e_ω = ω_target - θ̇

τ_time =
    K_p sin(e_θ)
  + K_d e_ω
```

或限幅最短角 PD。物理模式永远不执行 `θ=θ_target`。

### 21.4 固定步

```text
J_eff = ∫step a_eff dt

Δp_eff = H tᵀ J_eff

p ← p
   + Δp_eff
   + (τ_cf + τ_relative) Δt

θ̇ ← p/I - Ω_z,end
θ  ← θ + θ̇ Δt
```

实际代码还需执行隐式线性阻尼、子步、时间戳对齐、饱和标记和有限值保护。

### 21.5 最终防错清单

```text
[ ] q_WB 的旋转方向唯一
[ ] C_BM 经实机验证
[ ] 重力使用三轴向量，不使用单一倾角
[ ] gravityVectorBody 与 -f_O 不重复相加
[ ] m、r_c、H、I 来自同一质量分布
[ ] 显式 -Iα_z 与约化动量二选一
[ ] 三维甩腕启用离心广义力矩
[ ] TIME_REF 只产生目标或力矩
[ ] p 是唯一权威速度状态
[ ] IMU 降采样保留冲量
[ ] 物理固定步与渲染帧解耦
[ ] 阻尼功率不为正
[ ] Debug 能显示全部分项力矩和外部功
[ ] 合成输入和确定性回放通过
```

---

## 附录 A [新增 v2] 符号、坐标和单位速查

| 符号 | 含义 | 单位/方向 |
|---|---|---|
| `W` | 世界局部惯性系 | `+z` 向上 |
| `B` | 表盘/表壳系 | `+x` 右，`+y` 下，`+z` 入屏 |
| `M` | IMU 芯片原始系 | 由 `C_BM` 转到 `B` |
| `q_WB` | `B→W` 姿态四元数 | Hamilton、标量在前 |
| `R_WB` | `B→W` 旋转矩阵 | 无量纲 |
| `g^W` | 世界重力 | `(0,0,-9.80665) m/s²` |
| `g^B` | `gravityVectorBody` | `R_BW g^W` |
| `f_S^B` | IMU 点比力 | `A_S^B-g^B`，`m/s²` |
| `a_eff,O^B` | 表心有效场 | `g^B-A_O^B=-f_O^B` |
| `Ω^B` | 表壳角速度 | `rad/s` |
| `α^B` | 表壳角加速度 | `rad/s²` |
| `r_OS^B` | 表心到 IMU 的固定偏移 | `m` |
| `θ` | 指针相对表盘角 | 十二点 0，顺时针正，`rad` |
| `u(θ)` | 指针轴向单位向量 | `(sinθ,-cosθ,0)` |
| `t(θ)` | 正角切向单位向量 | `(cosθ,sinθ,0)` |
| `m` | 指针质量 | `kg` |
| `r_c` | 指针质心向量 | `m` |
| `h(θ)` | 旋转后的质量一阶矩向量 `m r_c(θ)` | `kg·m` |
| `H` | 质量一阶矩 | `kg·m` |
| `I` | 表心法向质量转动惯量 | `kg·m²` |
| `p` | 约化广义动量 | `kg·m²·rad/s` |
| `τ` | 广义力矩 | `N·m` |
| `J_eff` | 有效加速度冲量 | `m/s` |

---

## 附录 B [新增 v2] 主方程推导

### B.1 世界系位置和速度

对细长指针质量元 `s`：

```text
x^W(s,t) =
    x_O^W(t)
  + R_WB(t) s u(θ)
```

令 `V_O^B=R_BW V_O^W`，则：

```text
ẋ^W =
    V_O^W
  + R_WB s[
        Ω^B×u
      + θ̇ t
    ]
```

定义：

```text
H = ∫s dm
I = ∫s² dm
```

### B.2 动能和势能

忽略与 `θ` 完全无关的常数后，动能中与广义坐标有关的部分可写为：

```text
T =
    H (V_O^B)ᵀ(Ω×u + θ̇t)
  + 0.5 I[
        |Ω|²
      + 2Ω_z θ̇
      + θ̇²
      - (Ω·u)²
    ]
  + constant
```

重力势能中与 `θ` 有关的部分：

```text
U =
    -H (g^B)ᵀu
  + constant
```

### B.3 拉格朗日方程

```text
d/dt(∂T/∂θ̇)
- ∂T/∂θ
+ ∂U/∂θ
= τ_relative
```

展开时，所有表心速度项相互抵消，只留下表心加速度：

```text
I(θ̈+α_z)
+ H tᵀ A_O^B
+ I(Ω·u)(Ω·t)
- H tᵀ g^B
= τ_relative
```

整理：

```text
Iθ̈ =
    H tᵀ(g^B-A_O^B)
  - Iα_z
  - I(Ω·u)(Ω·t)
  + τ_relative
```

即：

```text
Iθ̈ =
    H tᵀa_eff,O^B
  - Iα_z
  + τ_centrifugal
  + τ_relative
```

该推导说明：

- 世界位置和匀速平移速度不会进入最终相对动力学；
- 表心加速度通过移动铰点约束进入；
- 使用表盘相对角只是消除约束的坐标选择，不是把表盘错误当成惯性世界；
- 指针“保持原世界惯性”的视觉来自角动量状态，而不是人为延迟动画。

### B.4 约化动量

定义：

```text
p=I(θ̇+Ω_z)
```

则：

```text
ṗ =
    H tᵀa_eff,O^B
  - I(Ω·u)(Ω·t)
  + τ_relative
```

它消去了主方程对 `α_z` 的显式依赖。该 `p` 是经过移动基座项化简后的实用动量状态。

---

## 附录 C [新增 v2] QMI8658 采样、量程和标定记录

### C.1 首版建议

```text
六轴 ODR：235 Hz
初始加速度量程：±16g
初始陀螺量程：±2048°/s
姿态融合：每有效样本一次
FIFO：低延迟水位或逐样本 DRDY
```

QMI8658 数据手册列出的六轴 ODR 包括
`7520/3760/1880/940/470/235/117.5/58.75/29.375 Hz`。实际驱动必须读取并记录最终
寄存器配置，不能只在注释中写“200 Hz”。

### C.2 加速度六面标定记录

每次固件/PCB 版本至少记录：

| 放置状态 | 原始均值 | 转到 B 系后的预期 | 误差 |
|---|---|---|---|
| `+x_B` 竖直向上 | 待测 | `f_x≈+g0` | 待填 |
| `-x_B` 竖直向上 | 待测 | `f_x≈-g0` | 待填 |
| `+y_B` 竖直向上 | 待测 | `f_y≈+g0` | 待填 |
| `-y_B` 竖直向上 | 待测 | `f_y≈-g0` | 待填 |
| `+z_B` 竖直向上 | 待测 | `f_z≈+g0` | 待填 |
| `-z_B` 竖直向上 | 待测 | `f_z≈-g0` | 待填 |

这里“某正轴竖直向上”时，加速度计静止比力沿该正轴，符合
`f=-g^B`。

### C.3 陀螺仪和偏心

- 分别绕 `+x_B/+y_B/+z_B` 按右手方向转动，验证正号；
- 静止多温度点记录零偏；
- 从 PCB/CAD 或实测得到 `r_OS^B`；
- 录制纯旋转动作，检查偏心修正前后的表心有效加速度；
- 比力、角速度和角加速度通道记录滤波类型与群延迟；
- 任何饱和、FIFO 溢出或时间戳跳变都写入质量标志。

---

## 附录 D [新增 v2] v1 结构迁移对应

| v1 章节 | v2 对应 | 处理 |
|---|---|---|
| 1～4 文档、需求、术语 | 1～4 | 保留并正式化模块边界与四元数约定 |
| 5～6 状态机与仲裁 | 5、11、12 | 限定为表盘内部状态 |
| 7 IMU | 6、7、附录 C | 拆分姿态链和动力学链，增加 `gravityVectorBody` 契约 |
| 8 独立时间基准 | 8 | 保留 |
| 9 刚性动力学 | 9、附录 B | 增加质心势能，澄清约化动量名称 |
| 10 回归策略 | 10 | 保留三模式，规定写权限和稳态误差处理 |
| 11～12 定时器与触摸 | 11～12 | 保留并补充连续性交接 |
| 13 材质 | 13 | 区分密度、质量分布、转轴、柔性和艺术参数 |
| 14 柔性 | 14 | 保留为可选第二阶段 |
| 15～16 积分与数据结构 | 15～17 | 增加冲量保持、多速率任务和资源预算 |
| 17～18 按键与系统生命周期 | 1、5、8 | 删除整机设计，只保留输入事件与连续时间接口 |
| 19～21 调试、测试、顺序 | 18～20 | 扩充为可观测、可回放、可验收系统 |
| 22～23 产品决策与默认值 | 20 | 仅保留仿真物理配置 |
| 24 数学摘要 | 21 | 重写为 C/C++ 可执行摘要 |

---

## 参考资料

1. [Espressif：ESP32-S3 Series Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
2. [Waveshare：ESP32-S3-Touch-AMOLED-1.8 官方资料](https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.8)
3. [QST / Waveshare：QMI8658C Datasheet](https://files.waveshare.com/wiki/common/QMI8658A.pdf)
4. [Sebastian O. H. Madgwick：An efficient orientation filter for inertial and inertial/magnetic sensor arrays](https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf)
5. [Mahony, Hamel, Pflimlin：Nonlinear Complementary Filters on the Special Orthogonal Group](https://doi.org/10.1109/TAC.2008.923738)

---

## 文档结论

这套表盘不需要伪造一个“手表移动动画”。正确的软件结构是：

1. 在世界系中定义重力和移动表盘；
2. 用姿态四元数得到重力在表盘系中的三轴分量；
3. 用加速度计比力得到重力与表心平移惯性的合成有效场；
4. 用每根针自己的质心、质量一阶矩、惯量、阻尼和柔性计算力矩；
5. 在表盘最小坐标中以固定步长积分独立动量；
6. 将正确时间作为受控目标，而不是覆盖物理状态；
7. 通过分项力矩、能量、冲量和确定性回放验证真实性。

世界坐标系负责说明“真实发生了什么”，表盘坐标和约化动量负责让 ESP32-S3 以很低
成本稳定算出同一个结果。两者不是互相竞争的模型，而是同一物理系统的推导层与实现层。

---

## 附录 E [规划] 少量自由钢珠与碰撞音频事件

### E.1 产品设想

未来可在表盘中加入最多 `3～4` 颗小钢珠，作为解压交互对象。钢珠可以先支持圆形轨道，
后续再扩展为表盘平面内自由活动。该扩展不得改变现有三指针的方程、积分顺序、状态机
和确定性结果。

推荐系统边界：

```text
SimulationWorld
├── BaseMotionContext       # 重力、表心加速度、角速度和固定时间步
├── PointerSystem           # 现有三指针物理引擎
├── FreeBallSystem          # 3～4 颗二维钢珠
├── ContactSystem           # 钢珠、边界和可选指针接触
└── CollisionEventQueue     # 只向外输出碰撞事实
```

三指针和钢珠共享只读的基座运动输入，但分别拥有状态、物性、积分器、安全限制和快照。
钢珠模块关闭，或钢珠未与指针接触时，三指针输出必须与当前版本位级一致。

### E.2 圆形轨道阶段

圆形轨道钢珠可以使用单一广义坐标 `θ_ball`。钢珠球心位置为：

```text
x = Cx + R cos(θ_ball)
y = Cy + R sin(θ_ball)
```

理想滑动物体的广义惯量：

```text
I_track = m R²
```

若模拟半径为 `r` 的实心钢球无滑动滚动：

```text
I_ball = 2/5 m r²
I_effective = m R² + I_ball R²/r² = 7/5 m R²
```

重力、表盘平移惯性、旋转参考系项、摩擦和接触力均投影到轨道切线，再由固定时间步
积分得到下一帧角度。

### E.3 二维自由钢珠阶段

自由钢珠至少保存：

```text
position = (x, y)
velocity = (vx, vy)
spinAngle
spinVelocity
```

在表盘旋转参考系中，应考虑：

```text
m a_relative =
    m g_body
  - m a_base
  - m α × r
  - 2m Ω × v_relative
  - m Ω × (Ω × r)
  + F_contact
  + F_friction
```

首版可只计算二维平移和圆形碰撞；钢珠纹理自转作为可选视觉状态。若允许钢珠脱离
表盘平面，则问题升级为六自由度三维刚体，不属于本扩展的默认范围。

### E.4 碰撞与音效事件

碰撞由物理引擎求解，音频系统只消费结果，不能反向阻塞或修改物理状态：

```text
Physics Contact Solve
        ↓
CollisionEventQueue
        ↓
Audio / Haptic Adapter
```

`CollisionEvent` 建议包含：

```text
eventId
timestampUs
bodyA
bodyB
materialA
materialB
contactPoint
contactNormal
preImpactRelativeVelocity
normalImpulse
tangentImpulse
collisionEnergyDelta
```

音量优先由法向冲量或碰撞能量决定，音色由材质组合决定，左右声像可由碰撞点决定。
事件必须在接触开始或新的有效撞击时产生，不能在静止接触求解的每次迭代中重复发声。
应设置最小冲量、最小接近速度、接触滞回和同一对象对的冷却时间。

滚动声与撞击声分离：持续滚动可以输出低频率、低优先级的 `RollingEvent`，但不得
用连续 `CollisionEvent` 模拟。

### E.5 固定容量与兼容门禁

最多四颗钢珠只有：

```text
C(4, 2) = 6
```

组钢珠—钢珠配对。结合边界和可选指针接触，首版使用固定容量 `16～32` 的接触/音频
事件队列即可。队列保持热路径零动态分配，并记录溢出计数。

扩展验收至少包括：

1. 钢珠模块关闭时，三指针逐步快照和确定性哈希与当前版本一致；
2. 钢珠存在但没有接触时，三指针轨迹不发生变化；
3. 钢珠撞击指针时，双方动量变化与接触冲量账本闭合；
4. 持续静止接触不重复产生撞击音频事件；
5. 音频队列消费速度不影响物理时间步；
6. 三四颗钢珠在 ESP32-S3 目标频率下满足固定内存和单步时延预算。
