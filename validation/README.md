# Validation

验证体系与生产引擎、平台适配和场景表现分离。

- `requirements/`：数学正确性、稳定性、性能、长期运行和可移植性要求。
- `x86-physics-engine/`：独立参考模型、解析测试、场景测试、Property、Fuzz、
  Benchmark、变异测试和 G0～G5 报告。

验证只判断物理状态，不以“看起来好看”代替数学正确性。新增公共引擎模块后，应先扩展
独立 Oracle 和门禁，再允许场景依赖它。

