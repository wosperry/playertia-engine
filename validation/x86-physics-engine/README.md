# x86 物理仿真引擎验证项目

本工程验证根目录 `engine/physics/` 中的独立生产物理核心。它只判断数学状态、数值稳定性、确定性、故障安全和性能，不包含 UI、硬件驱动、姿态融合器实现或 ESP32-S3 实机验收。

验证依据：

- [《实时物理仿真表盘引擎测试验证要求说明书》](../requirements/realtime-physics-engine-validation.md)
- [《超真实机械指针交互系统设计文档 v2》](../../scenes/physical-watch/docs/mechanical-pointer-engine-design-v2.md)

## 已实现范围

- 固定三针生产模型，其中分针为二段柔性刚体；
- 世界重力、移动/旋转表壳的非惯性项和 IMU 偏心修正；
- TIME_REF、触摸约束与释放、机械/动画/混合回归、主动驱动；
- 固定容量通用接触约束和冲量求解；
- `configure → reset → step(BaseInterval, targets, events) → snapshot` 接口；
- `float32` 权威状态、定长队列、热路径零动态分配；
- `long double` 独立参考模型和 RK4 Oracle；
- 解析、场景、Property、Fuzz、长期、性能和变异验证。

碰撞检测由外部系统负责；引擎消费法向间隙、恢复系数和 Jacobian，不包含几何形状检测。

## 目录

```text
x86-physics-engine/
├── config/                 # G0 参数和 G1～G5 门禁冻结值
├── include/dial/           # 独立参考接口和测试支持
├── src/                    # 独立参考模型
├── tests/
│   ├── unit/               # 坐标、配置、浮点模式
│   ├── model/              # 力矩、积分、柔性体、碰撞
│   ├── scenarios/          # 状态机和交互场景
│   ├── property/           # 随机性质
│   ├── fuzz/               # 非法输入与 libFuzzer
│   └── performance/        # LOAD-FULL 和规模基准
├── testdata/               # 种子、轨迹和 Golden 数据
├── tools/                  # 门禁运行器与变异脚本
└── reports/                # JSON、Markdown、JUnit、覆盖率和性能结果
```

## 构建与基础回归

```bash
cmake -S . -B build/gcc-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/gcc-release -j2
ctest --test-dir build/gcc-release --output-on-failure
```

离线重建可复用已下载依赖：

```bash
cmake -S . -B build/offline -G Ninja \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=build/gcc-release/_deps/googletest-src \
  -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=build/gcc-release/_deps/nlohmann_json-src \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLEBENCHMARK=build/gcc-release/_deps/googlebenchmark-src
```

## 验证入口

```bash
# 一亿步 G3
./build/gcc-release/dial_validate --gate G3 --output reports/g3-gcc-release

# G4 十亿子步等效、千万跨界和百万状态循环；0 表示跳过真实墙钟等待
./build/gcc-release/dial_validate --gate G4 --wall-hours 0 \
  --output reports/g4-accelerated

# 真实 24 小时 G4，不传 --wall-hours
./build/gcc-release/dial_validate --gate G4 \
  --output reports/g4-wallclock-24h

# 一百万随机性质
DIAL_PROPERTY_CASES=1000000 ./build/gcc-release/dial_tests \
  --gtest_filter='Properties.*'

# 十个关键变异
./tools/run_mutations.sh
```

门禁运行中每一百万步更新 `progress.json`。最终结果写入同目录的 `validation-report.json` 和 `validation-report.md`。

## 验收边界

只有真实连续 24 小时 G4 也完成后，才能签署“G0～G5 全部通过”。加速长期结果不替代墙钟稳定性；x86 性能结果也只作为 ESP32-S3 静态移植预审。
