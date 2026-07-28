# 物理指针核心引擎

本目录是可独立构建、可跨平台复用的生产物理核心，不属于验证工程。

## 包含内容

- 固定三指针状态，其中一根为二段柔性指针；
- 世界重力与移动/旋转表盘输入；
- 约化动量、阻尼、驱动、触摸和 TIME_REF；
- 机械、动画和混合回归；
- 固定容量通用接触约束；
- 力矩、冲量、能量、安全状态和确定性快照。

## 不包含内容

- x86 参考模型、GoogleTest、Fuzz、Benchmark 和报告；
- ESP32-S3、Android、HarmonyOS 或 iOS 平台适配；
- QMI8658 驱动和姿态融合；
- AMOLED 渲染、触摸识别、音频和产品 UI。

## 接口

公共接口位于 `include/dial/physics.hpp`：

```text
configure
→ reset
→ step(BaseInterval, StepInput)
→ snapshot
```

生产状态使用 `float32`、固定三针数组和固定容量事件/接触队列。热路径不依赖动态
内存分配。

## 独立构建

```bash
cmake -S engine/physics -B engine/physics/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release
cmake --build engine/physics/build
```

验证工程位于 `validation/x86-physics-engine`，通过 `add_subdirectory` 引用本库。
