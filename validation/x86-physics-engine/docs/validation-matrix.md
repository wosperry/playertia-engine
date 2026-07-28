# 验证矩阵

## SUT 与 Oracle 隔离

根目录 `engine/physics/` 中的 `playertia_dial_physics` 是被测生产实现，采用 `float32`、固定三针状态和固定容量输入队列。`dial_reference` 留在验证工程内，是采用 `long double` 和 RK4 微步积分的独立 Oracle；它不调用生产力矩或生产积分函数。

| 验证对象 | 入口 | 主要判据 |
|---|---|---|
| 坐标、四元数、重力 | `tests/unit/math_tests.cpp` | 解析值、往返变换、重力模长 |
| IMU 偏心修正 | `tests/unit/math_tests.cpp` | 刚体偏心加速度公式 |
| 刚针动力学 | `tests/model/dynamics_tests.cpp` | 解析动量、符号、RK4 收敛 |
| 柔性二段针 | `tests/model/dynamics_tests.cpp` | 质量矩阵 SPD、内力反作用、恢复方向 |
| 通用接触约束 | `tests/model/dynamics_tests.cpp` | 冲量—动量、单次消费 |
| TIME_REF/触摸/回归 | `tests/scenarios/scenario_tests.cpp` | 权威状态和状态转换 |
| 故障与输入包络 | `tests/fuzz/fault_tests.cpp` | 拒绝、降级、有限状态 |
| 随机性质 | `tests/property/property_tests.cpp` | 有限、有界、阻尼耗散、SPD |
| 结构化 Fuzz | `tests/fuzz/input_fuzzer.cpp` | ASan/UBSan 下无崩溃、越界和非法状态 |
| 性能 | `tests/performance/physics_benchmark.cpp` | LOAD-FULL 和对象规模曲线 |
| 长期门禁 | `tools/validation_runner.cpp` | 步数、跨界、状态循环、资源、分位耗时 |

## G0～G5

| 门禁 | 自动化内容 | 产物 |
|---|---|---|
| G0 | 固定输入包络、1/2/4 子步、三针参数、容差和故障策略 | `config/g0-baseline-v1.json` |
| G1 | 解析、单步、参考模型 | JUnit、CTest |
| G2 | 状态机、交互、多速率和千万步 | 门禁 JSON/Markdown |
| G3 | 百万随机、百万 Fuzz、一亿步、确定性和 PC 性能 | JUnit、Fuzz 语料、门禁和 Benchmark JSON |
| G4 | 十亿子步等效、千万跨界、百万状态循环、24 小时墙钟 | 进度及门禁 JSON/Markdown |
| G5 | GCC/Clang、FMA 开关、FTZ/DAZ、ASan/UBSan、覆盖率、变异 | JUnit、覆盖率和变异 JSON |

生产状态固定为 `float32`。G5 的高精度路径由独立 `long double` Oracle 承担，其精度高于要求的 `float64` 对照；它不是另一套生产状态，也不会改变 MCU 数据布局。`fast-math` 未计划用于产品，门禁记为 N/A。

## 失败证据

- GoogleTest/JUnit 保留用例名和断言首个差异；
- Property 测试使用固定种子，命令和种子清单进入总报告；
- libFuzzer 将最小化崩溃输入写入 `reports/fuzz/`，语料可位级重放；
- 长期运行每一百万步更新一次 `progress.json`，最终保存冻结配置哈希、确定性状态哈希和首个失败计数；
- 变异测试保存每个变异的独立日志及汇总 JSON。

没有失败时不会伪造反例包；出现失败时，相关框架的原始输入、种子、日志和冻结配置共同组成回放包。
