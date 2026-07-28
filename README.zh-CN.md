# Playertia Engine 微型物理交互引擎

[English](README.md) | 简体中文

这是一个面向小屏幕、IMU、触摸、声音和振动设备的可移植微型物理交互引擎。

它不追求成为 Unity 一类的通用引擎，而是专注于少量物体构成的“小型真实物理世界”：

- 设备倾斜、旋转、平移和甩动会真实影响场景；
- 触摸和按键通过力、约束或冲量进入物理系统；
- 物体具有质量、质心、惯量、阻尼、摩擦和柔性；
- 物理状态由固定时间步推进，不由屏幕帧率决定；
- 碰撞向外产生声音和触觉事件；
- 同一套核心可以运行在 ESP32-S3、桌面和移动平台。

第一批目标场景：

1. 可自由甩动、触摸和回归时间的物理指针手表；
2. 三四颗自由活动并带碰撞声音的钢珠壁纸；
3. 具有尺寸、墙体、洞口和层次感的微型钢珠迷宫；
4. 发条、摆锤、陀螺、弹簧、棘轮等后续物理玩具。

## 设计原则

1. **先做场景，再抽象能力**：相同问题第二次出现时才进入公共引擎。
2. **真实物理优先**：从参考系、自由度、质量属性和力开始，而不是拼动画规则。
3. **小而确定**：固定容量、固定时间步、可回放、热路径零动态分配。
4. **物理与表现分离**：引擎输出状态和事件，渲染、音频和振动只消费结果。
5. **场景彼此隔离**：增加钢珠或迷宫不能改变无接触时的三指针结果。
6. **跨平台核心**：平台层只提供传感器、时钟、触摸、渲染和设备生命周期。
7. **不提前建造万能系统**：暂不引入复杂 ECS、脚本语言、编辑器或三维通用刚体。
8. **公共边界稳定**：内部 C++ 模型可以演进，应用只依赖版本化 C ABI 和平台包。
9. **接入体验可验证**：示例必须只使用公开接口，并能作为平台包的集成测试。

## 项目结构

```text
.
├── CMakeLists.txt                 # 核心与 SDK 的统一构建入口
├── engine/
│   ├── physics/                   # 已实现的生产物理核心
│   └── runtime/                   # 未来共享场景时钟、生命周期和事件边界
├── sdk/
│   ├── c/                         # 已实现：版本化、不透明的跨语言 C ABI
│   ├── packages/                  # Android / Apple / HarmonyOS 原生包
│   └── bindings/                  # 后续按需求增加的框架绑定
├── scenes/
│   ├── physical-watch/            # 当前主场景：物理指针手表
│   ├── free-balls/                # 规划：三四颗自由钢珠壁纸
│   └── ball-maze/                 # 规划：微型钢珠迷宫
├── platforms/                     # ESP32-S3 等具体设备适配
├── examples/                      # 只使用公开 SDK 的跨平台示例
└── validation/
    ├── requirements/              # 独立验证要求
    └── x86-physics-engine/        # 参考模型、测试、Fuzz、Benchmark 和报告
```

## 当前实现状态

已经完成：

- 独立 C++20 三指针物理核心；
- 两根刚性指针和一根二段柔性指针；
- 世界重力与移动/旋转表盘模型；
- 触摸、TIME_REF、机械/动画/混合回归；
- 通用接触约束、能量/冲量/安全账本；
- 可独立构建的版本化 C ABI；
- x86 独立高精度 Oracle 和 G0～G5 验证体系。

尚未实现：

- ESP32-S3 固件和 QMI8658 驱动；
- AMOLED 指针与材质渲染；
- 实际触摸命中和手势；
- 自由钢珠与迷宫几何；
- 音频、振动和平台生命周期适配。
- Android、Apple 和 HarmonyOS 原生安装包。

## 入口

- [生产物理核心](engine/physics/README.md)
- [SDK 接入边界](sdk/README.md)
- [引擎演进边界](engine/runtime/README.md)
- [场景目录](scenes/README.md)
- [物理指针手表](scenes/physical-watch/README.md)
- [平台适配边界](platforms/README.md)
- [公开示例](examples/README.md)
- [验证体系](validation/README.md)

## 当前开发策略

物理能力继续由真实场景推动；公共接入边界可以与场景并行验证。ESP32-S3、Android、
iOS 和 HarmonyOS 示例共享同一套输入与快照语义，平台差异留在各自包和适配层内。

## 构建

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

默认构建生产物理核心、C ABI 和 C 接口冒烟测试；完整 x86 验证体系继续由
`validation/x86-physics-engine` 独立执行。

## 许可证

Playertia Engine 当前为闭源专有软件，保留所有权利。未经版权所有者事先书面授权，
不得使用、复制、修改或分发本项目。详见 [LICENSE](LICENSE)。
